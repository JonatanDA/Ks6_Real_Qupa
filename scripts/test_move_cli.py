#!/usr/bin/env python3
# Archivo: scripts/test_move_cli.py
# Ubicación: ~/sim_ws_qupa/Ks6_Real_Qupa/scripts/test_move_cli.py
# Descripción:
#   Herramienta interactiva de calibración y prueba para Functions/move.py en Qupa físico real.
#   - T1: ros2 launch qupa_hardware hardware.launch.py namespace:=qupa_D2
#   - T2: python3 scripts/test_move_cli.py --id D2
#   - Entrada esperada: ({ang1,dist1}, {ang2,dist2}, ...), vel_angular_deg_s, vel_lineal_cm_s
#       Ejemplo: ({90,30} , {90,20} , {30,45}), 40, 5
#       Nota: cada par {ang, dist} se convierte internamente al orden de Move.move(dist, ang).
#   - Controles tras finalizar:
#       Nueva secuencia -> ejecuta los nuevos segmentos
#       R               -> repite el último comando
#       N o Ctrl+C      -> frena motores y cierra limpiamente

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
DEFAULT_ROBOT_ID               = "qupa_D2"
TICKS_PER_SECOND               = 8        # 8 Hz recomendado en físico (dt=0.125s < 0.3s watchdog)
FOOTBOT_INTERWHEEL_DISTANCE    = 8.6      # cm (wheel_base = 0.086 m)

# Parámetros IMU (mismos nombres que Ks6_master_controller.py)
AUTO_LAUNCH_IMU                = True     # True: lanza qupa_imu_test/mpu6050 automáticamente
SHOW_IMU_LOGS                  = False    # True: muestra [INFO] de mpu6050 en consola
IMU_PACKAGE                    = "qupa_imu_test"
IMU_EXECUTABLE                 = "mpu6050"
IMU_SUBNAMESPACE               = "mpu6050"
IMU_TOPIC_SUFFIX               = "mpu6050/imu/data_raw"
IMU_TARE_SERVICE_SUFFIX        = "mpu6050/tare"
WAIT_AFTER_IMU_SEC             = 4.0

IMU_GYRO_Z_SIGN                = 1.0      # -1.0 si el eje Z del MPU6050 apunta hacia abajo
IMU_ANGULAR_SPEED_SLOW_RAD_S   = 0.72     # rad/s en zona lenta (> 0.70 rad/s deadband físico)
IMU_TURN_SLOW_ZONE_DEG         = 30.0     # ° finales a velocidad lenta
IMU_TURN_EARLY_STOP_DEG        = 15.0     # ° iniciales antes de la meta para frenar por inercia
IMU_ADAPTIVE_EARLY_STOP        = True     # Ajusta early_stop con el sobrepaso medido
IMU_RETARE_EACH_SEGMENT        = True     # True: re-encera en cada vértice
IMU_TURN_PAUSE_S               = 0.5      # s de pausa de frenado tras cada giro
IMU_SETTLE_S                   = 0.3      # s tras el tare antes de girar
IMU_TARE_TIMEOUT_S             = 5.0      # s máximo de espera al servicio /tare
IMU_TIMEOUT_FACTOR             = 2.5      # Seguridad: aborta giro tras factor * tiempo esperado
IMU_MEASURED_LINEAR_SPEED_CM_S = 0.0      # cm/s real medido con regla (0.0 -> usa vel_lineal pedida)

# ============================================================================
#                            PARSER DE ENTRADA CLI
# ============================================================================
_NUM_RE = r"[+-]?(?:\d+(?:\.\d*)?|\.\d+)"
_PAIR_RE = re.compile(rf"\{{\s*({_NUM_RE})\s*,\s*({_NUM_RE})\s*\}}")
_FULL_RE = re.compile(
    rf"^\s*\((.+)\)\s*,\s*({_NUM_RE})\s*,\s*({_NUM_RE})\s*$"
)


def parse_move_command(raw_text: str):
    """
    Parsea una entrada tipo:
      ({90,30} , {90,20} , {30,45}), 40, 5
    Retorna:
      (segments, speed_g_deg_s, speed_d_cm_s)
      donde segments = [(ang_deg, dist_cm), ...]
    """
    m = _FULL_RE.match(raw_text.strip())
    if not m:
        raise ValueError(
            "Formato inválido. Usa: ({ang1,dist1}, {ang2,dist2}), vel_ang_deg_s, vel_lin_cm_s\n"
            "Ejemplo: ({90,30} , {90,20} , {30,45}), 40, 5"
        )

    inner_str, raw_vg, raw_vd = m.group(1), m.group(2), m.group(3)
    pairs = _PAIR_RE.findall(inner_str)
    if not pairs:
        raise ValueError("No se encontraron pares {angulo, distancia} dentro de los paréntesis.")

    # Verificar que no haya texto sobrante extraño fuera de pares y comas
    leftover = _PAIR_RE.sub("", inner_str)
    leftover = re.sub(r"[\s,]+", "", leftover)
    if leftover:
        raise ValueError(f"Texto no reconocido dentro de los paréntesis: '{leftover}'")

    segments = [(float(ang_s), float(dist_s)) for ang_s, dist_s in pairs]
    speed_g = float(raw_vg)
    speed_d = float(raw_vd)
    return segments, speed_g, speed_d


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

    print(f"[IMU] Lanzando driver mpu6050 (mantén el robot quieto para tare)...", flush=True)
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
    parser = argparse.ArgumentParser(description="CLI interactivo para calibrar Functions/move.py en Qupa real")
    parser.add_argument("--id", type=str, default=DEFAULT_ROBOT_ID, help=f"ID/namespace del robot (default: {DEFAULT_ROBOT_ID})")
    parser.add_argument("--tps", type=int, default=TICKS_PER_SECOND, help=f"Ticks por segundo (default: {TICKS_PER_SECOND})")
    parser.add_argument("--v_real", type=float, default=IMU_MEASURED_LINEAR_SPEED_CM_S, help="Velocidad lineal real medida en cm/s (0.0 = usa la pedida)")
    parser.add_argument("--no_imu", action="store_true", help="No lanzar el driver mpu6050 automáticamente")
    args = parser.parse_args()

    robot_id = normalize_robot_id(args.id)
    tps = max(1, int(args.tps))
    dt = 1.0 / float(tps)

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

    # Propagar configuración a shared_config antes de usar Move.move()
    shared_config.set_controller_vars({
        "ticks_per_second": tps,
        "FOOTBOT_INTERWHEEL_DISTANCE": FOOTBOT_INTERWHEEL_DISTANCE,
        "IMU_GYRO_Z_SIGN": IMU_GYRO_Z_SIGN,
        "IMU_ANGULAR_SPEED_SLOW_RAD_S": IMU_ANGULAR_SPEED_SLOW_RAD_S,
        "IMU_TURN_SLOW_ZONE_DEG": IMU_TURN_SLOW_ZONE_DEG,
        "IMU_TURN_EARLY_STOP_DEG": IMU_TURN_EARLY_STOP_DEG,
        "IMU_ADAPTIVE_EARLY_STOP": IMU_ADAPTIVE_EARLY_STOP,
        "IMU_RETARE_EACH_SEGMENT": IMU_RETARE_EACH_SEGMENT,
        "IMU_TURN_PAUSE_S": IMU_TURN_PAUSE_S,
        "IMU_SETTLE_S": IMU_SETTLE_S,
        "IMU_TARE_TIMEOUT_S": IMU_TARE_TIMEOUT_S,
        "IMU_TIMEOUT_FACTOR": IMU_TIMEOUT_FACTOR,
        "IMU_MEASURED_LINEAR_SPEED_CM_S": float(args.v_real),
    })

    imu_proc = None
    if AUTO_LAUNCH_IMU and not args.no_imu:
        imu_proc = launch_imu_process(robot_id, os.environ.copy())

    rclpy.init()

    class MoveTestNode(Node):
        def __init__(self):
            super().__init__("ks6_move_test_cli")
            self.cmd_vel_pub = self.create_publisher(Twist, f"/{robot_id}/cmd_vel", 10)
            self.rob = Utils.QupaRobotInstance(
                robot_id,
                0.0, 0.0, 0.0,
                "",
                "master",
                ticks_per_second=tps,
                interwheel_distance=FOOTBOT_INTERWHEEL_DISTANCE,
            )
            self.rob.odom_received = False
            self.imu_tare_client = self.create_client(Trigger, f"/{robot_id}/{IMU_TARE_SERVICE_SUFFIX}")
            self.rob.imu_tare_client = self.imu_tare_client
            self.create_subscription(Imu, f"/{robot_id}/{IMU_TOPIC_SUFFIX}", self._imu_cb, 50)

        def _imu_cb(self, msg):
            Move.update_imu(self.rob, msg)

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

    node = MoveTestNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print("\n===========================================================")
    print(f" CALIBRADOR INTERACTIVO DE Move.move() -> {robot_id}")
    print("===========================================================")
    print(f" Ticks/seg: {tps} Hz (dt={dt:.3f}s) | v_real_calibrada: {args.v_real:.2f} cm/s")
    print(f" Tópico cmd_vel: /{robot_id}/cmd_vel")
    print(f" Tópico IMU:     /{robot_id}/{IMU_TOPIC_SUFFIX}")

    # Esperar brevemente señal de la IMU (máximo 6 s)
    t_wait = time.monotonic()
    while not getattr(node.rob, "imu_received", False) and (time.monotonic() - t_wait) < 6.0:
        time.sleep(0.1)
    if getattr(node.rob, "imu_received", False):
        print(f" [OK] IMU conectada y publicando (yaw inicial = {math.degrees(node.rob.imu_yaw):.2f}°).")
    else:
        print(" [AVISO] No se recibieron datos de la IMU en 6s (verifica que mpu6050 esté activo).")

    print("\nFormato de entrada:")
    print("  ({ang1,dist1} , {ang2,dist2} , ...), vel_ang_deg_s, vel_lin_cm_s")
    print("  Ejemplo: ({90,30} , {90,20} , {30,45}), 40, 5")
    print("  Comandos: R = repetir último | N o Ctrl+C = salir\n")

    last_command = None
    global_tick = 0

    try:
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

            if upper == "R":
                if last_command is None:
                    print("[AVISO] Aún no hay ningún comando previo para repetir.")
                    continue
                segments, speed_g, speed_d = last_command
                print(f"[CLI] Repitiendo último comando: {segments} | vel_ang={speed_g} °/s | vel_lin={speed_d} cm/s")
            else:
                try:
                    segments, speed_g, speed_d = parse_move_command(user_in)
                    last_command = (segments, speed_g, speed_d)
                except ValueError as err:
                    print(f"[ERROR] {err}")
                    continue

            print(f"\n--- Ejecutando {len(segments)} segmento(s) | speed_g={speed_g:.2f} °/s | speed_d={speed_d:.2f} cm/s ---")

            for seg_idx, (ang_deg, dist_cm) in enumerate(segments, start=1):
                Move.r_move(node.rob)
                node.rob.c1 = 0
                node.rob.c2 = 0
                seg_t0 = time.monotonic()

                print(f"\n[SEG {seg_idx}/{len(segments)}] Objetivo -> ángulo={ang_deg:.2f}°, distancia={dist_cm:.2f} cm")
                print("  tick | fase     | ang_rec(°) | dist_rec(cm) | cmd_v(m/s) | cmd_w(rad/s) | imu_yaw(°)")
                print("  -----+----------+------------+--------------+------------+--------------+-----------")

                seg_tick = 0
                while node.rob.c1 != 2:
                    tick_start = time.monotonic()
                    global_tick += 1
                    seg_tick += 1

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

                    if c1_out == 0:
                        fase = f"GIRO:{getattr(node.rob, 'imu_turn_substate', 'TURN')}"
                    elif c1_out == 1:
                        fase = "AVANCE"
                    else:
                        fase = "FIN"

                    moved_ang_deg = math.degrees(moved_ang_rad)
                    imu_yaw_deg = math.degrees(float(getattr(node.rob, "imu_yaw", 0.0)))
                    print(
                        f"  {seg_tick:4d} | {fase:<8s} | {moved_ang_deg:+10.2f} | {moved_dist:12.2f} | "
                        f"{node.rob.cmd_linear_x:+10.4f} | {node.rob.cmd_angular_z:+12.4f} | {imu_yaw_deg:+9.2f}",
                        flush=True,
                    )

                    elapsed_tick = time.monotonic() - tick_start
                    sleep_s = dt - elapsed_tick
                    if sleep_s > 0.0:
                        time.sleep(sleep_s)

                node.stop_motors()
                seg_dur = time.monotonic() - seg_t0
                print(
                    f"[FIN SEG {seg_idx}/{len(segments)}] Tiempo={seg_dur:.2f}s | "
                    f"Objetivo=({ang_deg:+.2f}°, {dist_cm:.2f} cm) | "
                    f"Reportado=({math.degrees(node.rob.move_angle):+.2f}°, {node.rob.move_distance:.2f} cm)\n",
                    flush=True,
                )

            print("--- Secuencia completada. Ingresa otro comando, 'R' para repetir o 'N' para salir. ---\n")

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