#!/usr/bin/env python3
# Archivo: scripts/test_move_cli.py
# Ubicación: ~/sim_ws_qupa/Ks6_Real_Qupa/scripts/test_move_cli.py
# Descripción:
#   Herramienta interactiva de calibración y ejecución de secuencias para Functions/move.py en Qupa físico real.
#   Usa directamente Functions/move.py (el mismo módulo de los experimentos).
#   - T1: ros2 launch qupa_hardware hardware.launch.py namespace:=qupa_48
#   - T2: python3 scripts/test_move_cli.py --id 48
#   - Espera OBLIGATORIAMENTE confirmación de la IMU antes de aceptar comandos.
#   - Entrada esperada:
#       (ang1,dis1),(ang2,dis2),...,(ang_i,dis_i), vel_ang, vel_lin, n
#       Ejemplo (triángulo repetido 2 veces):
#         (120,25),(120,25),(120,25), 44,7,2
#       Ejemplo (un solo segmento, 1 vez):
#         (90,30), 44,7,1
#       Nota: cada par (ang,dis) se convierte internamente a Move.move(dis, ang, vel_lin, vel_ang).
#   - Controles en el prompt:
#       secuencia -> ejecuta los segmentos n veces
#       R         -> repite la última secuencia
#       T         -> ejecuta /tare manual en la IMU
#       N o Ctrl+C -> frena motores y cierra limpiamente

import argparse
import math
import os
import re
import signal
import subprocess
import sys
import threading
import time

# Configurar rutas del proyecto Ks6_Real_Qupa
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.dirname(_THIS_DIR) if os.path.basename(_THIS_DIR) == "scripts" else _THIS_DIR
for _p in (_PROJECT_ROOT, os.path.join(_PROJECT_ROOT, "Controller"), os.path.join(_PROJECT_ROOT, "Functions")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

# ============================================================================
#                      VARIABLES CONFIGURABLES DE CALIBRACIÓN
# ============================================================================
DEFAULT_ROBOT_ID               = "qupa_48"
TICKS_PER_SECOND               = 8        # 8 Hz recomendado en físico (dt=0.125s < 0.3s watchdog)
FOOTBOT_INTERWHEEL_DISTANCE    = 8.6      # cm (wheel_base = 0.086 m)

# Lanzador local de IMU (los parámetros de control IMU y v_real por robot viven en Functions/move.py)
AUTO_LAUNCH_IMU                = True     # True: lanza qupa_imu_test/mpu6050 automáticamente
SHOW_IMU_LOGS                  = False    # True: muestra [INFO] de mpu6050 en consola
IMU_PACKAGE                    = "qupa_imu_test"
IMU_EXECUTABLE                 = "mpu6050"
IMU_SUBNAMESPACE               = "mpu6050"
WAIT_AFTER_IMU_SEC             = 4.0
IMU_GYRO_SCALE                 = 1.0      # Multiplicador opcional por CLI (--gyro_scale) sobre Move.IMU_GYRO_Z_SIGN
TARE_ON_FINISH                 = True     # True: ejecuta /tare al FINALIZAR toda la secuencia

# ============================================================================
#                            PARSER DE ENTRADA CLI
# ============================================================================
_PAIR_RE = re.compile(r"\(\s*([^(),]+?)\s*,\s*([^(),]+?)\s*\)")


def parse_sequence_command(raw_text: str):
    """
    Parsea una entrada con formato:
      (ang1,dis1),(ang2,dis2),...,(ang_i,dis_i), vel_ang, vel_lin, n
    Ejemplo:
      (120,25),(120,25),(120,25), 44,7,2
    Retorna:
      (segments, speed_g_deg_s, speed_d_cm_s, repetitions)
      donde segments = [(ang1, dis1), (ang2, dis2), ...]
    """
    text = raw_text.strip()
    matches = list(_PAIR_RE.finditer(text))
    if not matches:
        raise ValueError(
            "Formato inválido. Usa: (ang1,dis1),(ang2,dis2),..., vel_ang, vel_lin, n\n"
            "Ejemplo: (120,25),(120,25),(120,25), 44,7,2"
        )

    # Verificar que entre pares solo haya comas y espacios (sin texto suelto)
    last_end = 0
    segments = []
    for idx, m in enumerate(matches):
        between = text[last_end:m.start()].strip()
        if idx == 0:
            if between != "":
                raise ValueError(f"Texto inesperado antes del primer segmento: '{between}'")
        else:
            if between not in (",", ""):
                raise ValueError(f"Separador inválido entre segmentos: '{between}' (usa comas).")
        try:
            ang_deg = float(m.group(1))
            dist_cm = float(m.group(2))
        except ValueError:
            raise ValueError(f"Segmento inválido '{m.group(0)}': ángulo y distancia deben ser numéricos.")
        segments.append((ang_deg, dist_cm))
        last_end = m.end()

    tail = text[last_end:].strip()
    if tail.startswith(","):
        tail = tail[1:].strip()

    tail_parts = [p.strip() for p in tail.split(",") if p.strip() != ""]
    if len(tail_parts) != 3:
        raise ValueError(
            "Después de los segmentos deben ir exactamente 3 valores: vel_ang, vel_lin, n\n"
            "Ejemplo: (90,30),(90,20), 44,7,2"
        )

    try:
        speed_g = float(tail_parts[0])
        speed_d = float(tail_parts[1])
        reps_f = float(tail_parts[2])
        if not reps_f.is_integer() or int(reps_f) < 1:
            raise ValueError("El número de repeticiones 'n' debe ser un entero >= 1.")
        repetitions = int(reps_f)
    except ValueError as exc:
        raise ValueError(
            f"Valores finales inválidos ({tail}): usa vel_ang, vel_lin, n (n entero >= 1). Detalle: {exc}"
        )

    return segments, speed_g, speed_d, repetitions


def normalize_robot_id(raw_id: str) -> str:
    clean = raw_id.strip().lstrip("/")
    if not clean.lower().startswith("qupa"):
        return f"qupa_{clean}"
    return clean


def launch_imu_process(robot_id: str, run_env: dict):
    imu_cmd = [
        "ros2", "run", IMU_PACKAGE, IMU_EXECUTABLE,
        "--ros-args",
        "-r", f"__ns:=/{robot_id}/{IMU_SUBNAMESPACE}",
        "-p", f"frame_id:={robot_id}/imu_link",
    ]
    if not SHOW_IMU_LOGS:
        imu_cmd.extend(["--log-level", "error"])
    try:
        from ament_index_python.packages import get_package_share_directory
        imu_yaml = os.path.join(get_package_share_directory(IMU_PACKAGE), "config", "mpu6050.yaml")
        if os.path.isfile(imu_yaml):
            imu_cmd.extend(["--params-file", imu_yaml])
    except Exception:
        pass

    print("[IMU] Lanzando driver mpu6050 (mantén el robot quieto para tare)...", flush=True)
    proc = subprocess.Popen(imu_cmd, env=run_env, start_new_session=True)
    time.sleep(WAIT_AFTER_IMU_SEC)
    return proc


def stop_process_group(proc):
    if proc is None or proc.poll() is not None:
        return
    try:
        pgid = os.getpgid(proc.pid)
        os.killpg(pgid, signal.SIGINT)
        try:
            proc.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            os.killpg(pgid, signal.SIGTERM)
            proc.wait(timeout=2.0)
    except Exception:
        try:
            proc.kill()
        except Exception:
            pass


# ============================================================================
#                        NODO ROS 2 DE PRUEBA DE MOVE()
# ============================================================================
def run_cli():
    parser = argparse.ArgumentParser(
        description="CLI interactivo de secuencias repetibles para Functions/move.py en Qupa real"
    )
    parser.add_argument("--id", type=str, default=DEFAULT_ROBOT_ID, help=f"ID/namespace del robot (default: {DEFAULT_ROBOT_ID})")
    parser.add_argument("--tps", type=int, default=TICKS_PER_SECOND, help=f"Ticks por segundo (default: {TICKS_PER_SECOND})")
    parser.add_argument(
        "--v_real",
        type=float,
        default=None,
        help="Sobreescribe Move.v_real en cm/s (si se omite, usa v_real definido al inicio de Functions/move.py; 0 desactiva calibración)",
    )
    parser.add_argument("--gyro_scale", type=float, default=IMU_GYRO_SCALE, help=f"Factor de escala del giroscopio Z (default: {IMU_GYRO_SCALE})")
    parser.add_argument("--tare_on_finish", action="store_true", default=TARE_ON_FINISH, help="Ejecutar /tare al finalizar toda la secuencia")
    parser.add_argument("--no_tare_on_finish", action="store_true", help="Desactivar /tare al finalizar toda la secuencia")
    parser.add_argument("--no_retare_start", action="store_true", help="Desactivar el /tare automático al iniciar cada giro en Move.move()")
    parser.add_argument("--no_imu", action="store_true", help="No lanzar el driver mpu6050 automáticamente")
    args = parser.parse_args()

    robot_id = normalize_robot_id(args.id)
    tps = max(1, int(args.tps))
    dt = 1.0 / float(tps)
    tare_on_finish = False if args.no_tare_on_finish else bool(args.tare_on_finish)

    # Importar módulos del proyecto y ROS 2
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Imu
    from std_srvs.srv import Trigger

    from Functions import shared_config
    from Functions import move as Move
    from Functions import utils as Utils

    # Todos los parámetros de IMU y v_real por robot vienen de Functions/move.py
    retare_each = False if args.no_retare_start else bool(Move.IMU_RETARE_EACH_SEGMENT)
    effective_gyro_sign = float(Move.IMU_GYRO_Z_SIGN) * float(args.gyro_scale)
    imu_topic_suffix = Move.IMU_TOPIC_SUFFIX
    imu_tare_suffix = Move.IMU_TARE_SERVICE_SUFFIX
    imu_tare_timeout_s = float(Move.IMU_TARE_TIMEOUT_S)

    if args.v_real is not None:
        active_v_real = float(args.v_real)
        Move.ROBOT_V_REAL[robot_id] = active_v_real
        Move.v_real = active_v_real
        v_real_source = "CLI --v_real"
    else:
        active_v_real = float(Move.get_v_real(robot_id))
        in_dict = any(k.lower() == robot_id.lower() for k in Move.ROBOT_V_REAL)
        v_real_source = f"Functions/move.py (ROBOT_V_REAL['{robot_id}'])" if in_dict else "Functions/move.py (v_real)"

    # Propagar solo lo de este CLI (sin pisar el resto de parámetros IMU de Functions/move.py)
    cfg_vars = {
        "ticks_per_second": tps,
        "FOOTBOT_INTERWHEEL_DISTANCE": FOOTBOT_INTERWHEEL_DISTANCE,
        "IMU_GYRO_Z_SIGN": effective_gyro_sign,
        "IMU_RETARE_EACH_SEGMENT": retare_each,
        "IMU_MEASURED_LINEAR_SPEED_CM_S": active_v_real,
    }
    main_mod = sys.modules[__name__]
    for _k, _v in cfg_vars.items():
        setattr(main_mod, _k, _v)
        setattr(shared_config, _k, _v)
        if hasattr(Move, _k):
            setattr(Move, _k, _v)
    sys.modules["Controller.Ks6_master_controller"] = main_mod
    shared_config.sync_from_controller(main_mod, extra_targets=[Move])

    imu_proc = None
    if AUTO_LAUNCH_IMU and not args.no_imu:
        imu_proc = launch_imu_process(robot_id, os.environ.copy())

    rclpy.init()

    class MoveTestNode(Node):
        def __init__(self):
            super().__init__("ks6_move_test_cli")
            self.cmd_vel_pub = self.create_publisher(Twist, f"/{robot_id}/cmd_vel", 10)
            self._cmd_vel_lock = threading.Lock()
            self._tick_cmd_samples = []

            self.rob = Utils.QupaRobotInstance(
                robot_id,
                0.0, 0.0, 0.0,
                "",
                "master",
                ticks_per_second=tps,
                interwheel_distance=FOOTBOT_INTERWHEEL_DISTANCE,
            )
            self.rob.odom_received = False
            self.imu_tare_client = self.create_client(Trigger, f"/{robot_id}/{imu_tare_suffix}")
            self.rob.imu_tare_client = self.imu_tare_client

            self.create_subscription(Imu, f"/{robot_id}/{imu_topic_suffix}", self._imu_cb, 50)
            self.create_subscription(Twist, f"/{robot_id}/cmd_vel", self._cmd_vel_cb, 50)

        def _imu_cb(self, msg):
            Move.update_imu(self.rob, msg)

        def _cmd_vel_cb(self, msg):
            with self._cmd_vel_lock:
                self._tick_cmd_samples.append((float(msg.linear.x), float(msg.angular.z)))

        def clear_tick_cmd_samples(self):
            with self._cmd_vel_lock:
                self._tick_cmd_samples.clear()

        def consume_tick_cmd_averages(self):
            with self._cmd_vel_lock:
                samples = list(self._tick_cmd_samples)
                self._tick_cmd_samples.clear()
            if not samples:
                return 0.0, 0.0, 0
            avg_v = sum(v for v, _ in samples) / len(samples)
            avg_w = sum(w for _, w in samples) / len(samples)
            return avg_v, avg_w, len(samples)

        def publish_cmd(self, lin_x: float, ang_z: float):
            msg = Twist()
            msg.linear.x = float(lin_x)
            msg.angular.z = float(ang_z)
            self.cmd_vel_pub.publish(msg)

        def stop_motors(self):
            self.rob.cmd_linear_x = 0.0
            self.rob.cmd_angular_z = 0.0
            for _ in range(3):
                self.publish_cmd(0.0, 0.0)
                time.sleep(0.03)

        def call_tare(self, reason: str = "manual") -> bool:
            if not self.imu_tare_client.service_is_ready():
                print(f"[IMU] Servicio /{robot_id}/{imu_tare_suffix} no está listo aún.", flush=True)
                return False
            print(f"[IMU] Ejecutando /tare ({reason}) — mantén el robot quieto...", flush=True)
            fut = self.imu_tare_client.call_async(Trigger.Request())
            t0 = time.monotonic()
            while not fut.done() and (time.monotonic() - t0) < imu_tare_timeout_s:
                time.sleep(0.05)
            if fut.done():
                try:
                    res = fut.result()
                    ok = bool(getattr(res, "success", True))
                    msg_txt = getattr(res, "message", "")
                    self.rob.imu_yaw = 0.0
                    self.rob.yaw = 0.0
                    print(f"[IMU] Tare completado (ok={ok}) {msg_txt} | imu_yaw reiniciado a 0.00°.", flush=True)
                    return ok
                except Exception as exc:
                    print(f"[IMU] Error en respuesta de /tare: {exc}", flush=True)
                    return False
            print(f"[IMU] Timeout ({imu_tare_timeout_s:.1f}s) esperando /tare.", flush=True)
            return False

    node = MoveTestNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print("\n===========================================================")
    print(f" EJECUTOR DE SECUENCIAS DE Move.move() -> {robot_id}")
    print("===========================================================")
    print(f" Ticks/seg: {tps} Hz (dt={dt:.3f}s) | v_real: {active_v_real:.2f} cm/s [{v_real_source}]")
    print(f" Escala gyro Z: {effective_gyro_sign:+.3f} | tare_inicio={retare_each} | tare_fin={tare_on_finish}")
    print(f" Tópico cmd_vel: /{robot_id}/cmd_vel")
    print(f" Tópico IMU:     /{robot_id}/{imu_topic_suffix}")

    last_command = None
    global_tick = 0

    try:
        # Bloquear arranque hasta confirmar que la IMU publica datos y el servicio /tare está listo
        print(f"\n[IMU] Esperando confirmación de la IMU en /{robot_id}/{imu_topic_suffix} (Ctrl+C para cancelar)...", flush=True)
        t_wait = time.monotonic()
        last_log_t = 0.0
        while True:
            imu_ok = bool(getattr(node.rob, "imu_received", False))
            tare_srv_ok = node.imu_tare_client.service_is_ready()
            if imu_ok and tare_srv_ok:
                break
            elapsed_w = time.monotonic() - t_wait
            if elapsed_w - last_log_t >= 2.0:
                print(
                    f"[IMU] Aún esperando ({elapsed_w:.0f}s)... tópico={'OK' if imu_ok else 'pendiente'} | "
                    f"servicio /tare={'OK' if tare_srv_ok else 'pendiente'}",
                    flush=True,
                )
                last_log_t = elapsed_w
            time.sleep(0.1)

        print(
            f"[OK] IMU confirmada en {time.monotonic() - t_wait:.1f}s "
            f"(yaw inicial = {math.degrees(node.rob.imu_yaw):+.2f}°).",
            flush=True,
        )

        print("\nFormato de entrada:")
        print("  (ang1,dis1),(ang2,dis2),...,(ang_i,dis_i), vel_ang, vel_lin, n")
        print("  Ejemplo: (120,25),(120,25),(120,25), 44,7,2")
        print("  Comandos: R = repetir última secuencia | T = tare manual | N o Ctrl+C = salir\n")

        while True:
            try:
                user_in = input("Comando > ").strip()
            except EOFError:
                break

            if not user_in:
                continue

            upper = user_in.upper()
            if upper == "N":
                print("[CLI] Saliendo por solicitud del usuario (N).")
                break

            if upper == "T":
                node.stop_motors()
                node.call_tare("manual")
                continue

            if upper == "R":
                if last_command is None:
                    print("[AVISO] Aún no hay ninguna secuencia previa para repetir.")
                    continue
                segments, speed_g, speed_d, repetitions = last_command
                seg_str = ", ".join(f"({a:g},{d:g})" for a, d in segments)
                print(
                    f"[CLI] Repitiendo última secuencia: {seg_str} | "
                    f"vel_ang={speed_g:g} °/s, vel_lin={speed_d:g} cm/s, n={repetitions}"
                )
            else:
                try:
                    segments, speed_g, speed_d, repetitions = parse_sequence_command(user_in)
                    last_command = (segments, speed_g, speed_d, repetitions)
                except ValueError as err:
                    print(f"[ERROR] {err}")
                    continue

            seq_t0 = time.monotonic()
            total_segs = len(segments)
            print(
                f"\n[SECUENCIA] {total_segs} segmento(s) x {repetitions} repetición(es) | "
                f"vel_ang={speed_g:.2f} °/s, vel_lin={speed_d:.2f} cm/s, v_real={active_v_real:.2f} cm/s",
                flush=True,
            )

            for rep_idx in range(1, repetitions + 1):
                rep_t0 = time.monotonic()
                print(f"\n==================== REPETICIÓN {rep_idx}/{repetitions} ====================", flush=True)

                for seg_idx, (ang_deg, dist_cm) in enumerate(segments, start=1):
                    Move.r_move(node.rob)
                    node.rob.c1 = 0
                    node.rob.c2 = 0
                    seg_t0 = time.monotonic()
                    all_v_samples = []
                    all_w_samples = []

                    print(
                        f"\n[REP {rep_idx}/{repetitions} | SEG {seg_idx}/{total_segs}] "
                        f"Objetivo -> ang={ang_deg:+.2f}°, dis={dist_cm:.2f} cm | "
                        f"vel_ang={speed_g:.2f} °/s, vel_lin={speed_d:.2f} cm/s",
                        flush=True,
                    )
                    print(
                        "  tick | fase        | ang_rec(°) | dist_rec(cm) | avg_v(m/s) | avg_w(rad/s) | n_cmd | imu_yaw(°)"
                    )
                    print(
                        "  -----+-------------+------------+--------------+------------+--------------+-------+-----------"
                    )

                    seg_tick = 0
                    while node.rob.c1 != 2:
                        tick_start = time.monotonic()
                        global_tick += 1
                        seg_tick += 1

                        node.clear_tick_cmd_samples()

                        # Orden de Move.move: (rob, distance, target_angle, speed_d, speed_g, timer, stop)
                        c2_out, c1_out, moved_dist, moved_ang_rad = Move.move(
                            node.rob,
                            dist_cm,
                            ang_deg,
                            speed_d,
                            speed_g,
                            global_tick,
                            False,
                        )
                        node.publish_cmd(node.rob.cmd_linear_x, node.rob.cmd_angular_z)

                        elapsed_tick = time.monotonic() - tick_start
                        sleep_s = dt - elapsed_tick
                        if sleep_s > 0.0:
                            time.sleep(sleep_s)

                        avg_v, avg_w, n_cmd = node.consume_tick_cmd_averages()
                        if n_cmd > 0:
                            all_v_samples.append(avg_v)
                            all_w_samples.append(avg_w)

                        if c1_out == 0:
                            fase = f"GIRO:{getattr(node.rob, 'imu_turn_substate', 'TURN')}"
                        elif c1_out == 1:
                            fase = "AVANCE"
                        else:
                            fase = "FIN"

                        moved_ang_deg = math.degrees(moved_ang_rad)
                        imu_yaw_deg = math.degrees(float(getattr(node.rob, "imu_yaw", 0.0)))
                        print(
                            f"  {seg_tick:4d} | {fase:<11s} | {moved_ang_deg:+10.2f} | {moved_dist:12.2f} | "
                            f"{avg_v:+10.4f} | {avg_w:+12.4f} | {n_cmd:5d} | {imu_yaw_deg:+9.2f}",
                            flush=True,
                        )

                    node.stop_motors()
                    seg_dur = time.monotonic() - seg_t0
                    seg_avg_v = (sum(all_v_samples) / len(all_v_samples)) if all_v_samples else 0.0
                    seg_avg_w = (sum(all_w_samples) / len(all_w_samples)) if all_w_samples else 0.0
                    print(
                        f"[FIN SEG {seg_idx}/{total_segs}] Tiempo={seg_dur:.2f}s | "
                        f"Objetivo=({ang_deg:+.2f}°, {dist_cm:.2f} cm) | "
                        f"Reportado=({math.degrees(node.rob.move_angle):+.2f}°, {node.rob.move_distance:.2f} cm) | "
                        f"Prom /cmd_vel: v={seg_avg_v:+.4f} m/s, w={seg_avg_w:+.4f} rad/s",
                        flush=True,
                    )

                print(
                    f"[FIN REP {rep_idx}/{repetitions}] Duración repetición={time.monotonic() - rep_t0:.2f}s",
                    flush=True,
                )

            node.stop_motors()
            print(
                f"\n[FIN SECUENCIA] {repetitions} repetición(es) ({total_segs * repetitions} segmentos) "
                f"completadas en {time.monotonic() - seq_t0:.2f}s.",
                flush=True,
            )

            if tare_on_finish:
                time.sleep(0.3)
                node.call_tare("fin de secuencia")

            print("\n--- Ingresa otra secuencia, 'R' repetir, 'T' tare o 'N' salir. ---\n")

    except KeyboardInterrupt:
        print("\n[CLI] Interrumpido por Ctrl+C.")
    finally:
        print("[CLI] Deteniendo motores y cerrando recursos...", flush=True)
        try:
            node.stop_motors()
        except Exception:
            pass
        try:
            executor.shutdown()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        stop_process_group(imu_proc)
        print("[CLI] Cerrado correctamente.", flush=True)


if __name__ == "__main__":
    run_cli()