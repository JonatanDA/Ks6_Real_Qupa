#!/usr/bin/env python3
# Archivo: scripts/test_apprentice_cli.py
# Ubicación: ~/sim_ws_qupa/Ks6_Real_Qupa/scripts/test_apprentice_cli.py
# Descripción:
#   Herramienta interactiva de calibración y diagnóstico del Aprendiz móvil en Qupa físico real.
#   Usa EXACTAMENTE los módulos del experimento real:
#     - Functions/move.py (Move.move, Move.update_imu, Move.r_move, Move.v_real)
#     - Controller/apprentice_entries.py (update_camera_detections, collect_omnidirectional_blobs,
#       tabla/calibración de cámara y compensación de odometría Fase 0 + Fase A.5)
#     - Functions/math_helper.py (polarACartesianoPAR, normalize_deg_180)
#
#   Escenario de prueba (simulación controlada de Active Learning):
#     - Maestro / baliza en un punto fijo con LEDs azules encendidos.
#     - Aprendiz (ej. qupa_AE) ejecuta secuencias de Move.move() a 4 ticks/seg (por defecto).
#     - En cada tick se registra la detección cruda y calibrada de cámara, la odometría
#       acumulada de Move.move() y la posición global compensada (glob_x, glob_y, glob_ang)
#       de la baliza, verificando si el punto fijo permanece quieto al girar/avanzar.
#     - Guarda todos los ticks automáticamente en un CSV para análisis posterior.
#
#   Uso típico en el robot aprendiz (qupa_AE):
#     - T1: ros2 launch qupa_hardware hardware.launch.py namespace:=qupa_AE
#     - T2: ros2 launch qupa_hardware camera.launch.py namespace:=qupa_AE
#           (o usar --launch_camera en este script)
#     - T3: python3 scripts/test_apprentice_cli.py --id AE
#
#   Formato de entrada en el prompt:
#     (ang1,dis1),(ang2,dis2),..., vel_ang, vel_lin, n
#     Ejemplo giro puro 45°:       (45,0), 44,7,1
#     Ejemplo giro 45° + 20 cm:    (45,20), 44,7,1
#     Comandos:
#       R = repetir última secuencia
#       S = observar quieto (8 ticks) sin moverse
#       Z = reiniciar origen de odometría (0,0,0°) y recapturar referencia de la baliza
#       T = /tare manual en la IMU
#       N o Ctrl+C = frenar motores, cerrar CSV y salir limpiamente

import argparse
import csv
import datetime
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
DEFAULT_ROBOT_ID               = "qupa_AE"
TICKS_PER_SECOND               = 4        # 4 Hz (misma frecuencia que Ks6_apprentice_controller.py)
FOOTBOT_INTERWHEEL_DISTANCE    = 8.6      # cm (wheel_base = 0.086 m)
MAX_FOV_RADIUS_CM              = 70.0     # cm (igual que self.max_fov_radius_cm en Ks6_apprentice_controller.py)

# Observación estática antes/después de moverse para separar retardo de cámara vs error real
PRE_MOVE_OBSERVE_TICKS         = 4        # 1.0 s a 4 Hz para promediar referencia inicial si aún no existe
POST_MOVE_OBSERVE_TICKS        = 6        # 1.5 s a 4 Hz quieto tras detenerse (permite llegar frames nuevos a ~2 Hz)
STATIONARY_CHECK_TICKS         = 8        # 2.0 s al ejecutar comando 'S'

# Cámara
AUTO_LAUNCH_CAMERA             = False    # False: asume camera.launch.py ya activo (o usa --launch_camera)
CAMERA_PACKAGE                 = "qupa_hardware"
CAMERA_LAUNCH_FILE             = "camera.launch.py"
WAIT_AFTER_CAMERA_SEC          = 5.0
CAMERA_TOPIC_SUFFIX            = "camera/detections"

# Parámetros IMU (mismos nombres que Ks6_apprentice_controller.py)
AUTO_LAUNCH_IMU                = True     # True: lanza qupa_imu_test/mpu6050 automáticamente
SHOW_IMU_LOGS                  = False    # True: muestra [INFO] de mpu6050 en consola
IMU_PACKAGE                    = "qupa_imu_test"
IMU_EXECUTABLE                 = "mpu6050"
IMU_SUBNAMESPACE               = "mpu6050"
IMU_TOPIC_SUFFIX               = "mpu6050/imu/data_raw"
IMU_TARE_SERVICE_SUFFIX        = "mpu6050/tare"
WAIT_AFTER_IMU_SEC             = 4.0

IMU_GYRO_Z_SIGN                = 1.0      # -1.0 si el eje Z apunta abajo
IMU_GYRO_SCALE                 = 1.0      # Factor de escala del giroscopio Z (default 1.0)
IMU_ANGULAR_SPEED_SLOW_RAD_S   = 0.72     # rad/s en zona lenta (> 0.70 rad/s deadband físico)
IMU_TURN_SLOW_ZONE_DEG         = 30.0     # ° finales a velocidad lenta
IMU_TURN_EARLY_STOP_DEG        = 15.0     # ° iniciales antes de la meta para frenar por inercia
IMU_ADAPTIVE_EARLY_STOP        = True     # Ajusta early_stop con el sobrepaso medido
IMU_RETARE_EACH_SEGMENT        = True     # True: re-encera ANTES de cada giro dentro de Move.move()
TARE_ON_FINISH                 = True     # True: ejecuta /tare al FINALIZAR toda la secuencia
IMU_TURN_PAUSE_S               = 0.5      # s de pausa de frenado tras cada giro
IMU_SETTLE_S                   = 0.3      # s tras el tare antes de girar
IMU_TARE_TIMEOUT_S             = 5.0      # s máximo de espera al servicio /tare
IMU_TIMEOUT_FACTOR             = 2.5      # Seguridad: aborta giro tras factor * tiempo esperado

# Ruido sintético desactivado (el robot físico ya tiene sensores reales)
noise_cfg = {
    "enabled": False,
    "distance_std_cm": 0.0,
    "angle_std_deg": 0.0,
}

# Carpeta de logs CSV (relativa a la raíz de Ks6_Real_Qupa)
DEFAULT_CSV_DIR                = os.path.join(_PROJECT_ROOT, "logs_test_apprentice")

# ============================================================================
#                            PARSER DE ENTRADA CLI
# ============================================================================
_PAIR_RE = re.compile(r"\(\s*([^(),]+?)\s*,\s*([^(),]+?)\s*\)")


def parse_sequence_command(raw_text: str):
    """
    Parsea una entrada con formato:
      (ang1,dis1),(ang2,dis2),...,(ang_i,dis_i), vel_ang, vel_lin, n
    Ejemplo:
      (45,0), 44,7,1
      (45,20),(-45,20), 44,7,1
    Retorna:
      (segments, speed_g_deg_s, speed_d_cm_s, repetitions)
    """
    text = raw_text.strip()
    matches = list(_PAIR_RE.finditer(text))
    if not matches:
        raise ValueError(
            "Formato inválido. Usa: (ang1,dis1),(ang2,dis2),..., vel_ang, vel_lin, n\n"
            "Ejemplo: (45,0), 44,7,1"
        )

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
            "Ejemplo: (45,0), 44,7,1"
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


def norm_rad(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def norm_deg_180(a: float) -> float:
    a = (float(a) + 180.0) % 360.0 - 180.0
    return a


def compute_compensated_pose_and_beacon(
    x_ant_r: float,
    y_ant_r: float,
    srt_ang_mark_ant: float,
    cur_move_dist: float,
    cur_move_ang_rad: float,
    blob_angle_deg: float,
    blob_dist_cm: float,
):
    """
    Replica exactamente:
      - apprentice_entries.py Fase A.5 (odometría del aprendiz desde Move.move)
      - apprentice_process.py Fase H   (proyección global del blob observado)
    Retorna:
      (obs_ang_offset_global_rad, x_mark_cm, y_mark_cm, glob_ang_deg, glob_x_cm, glob_y_cm)
    """
    obs_ang_offset_global = norm_rad(float(srt_ang_mark_ant) + float(cur_move_ang_rad))
    dx_mov_cm = float(cur_move_dist) * math.cos(obs_ang_offset_global)
    dy_mov_cm = float(cur_move_dist) * math.sin(obs_ang_offset_global)
    x_mark = float(x_ant_r) + dx_mov_cm
    y_mark = float(y_ant_r) + dy_mov_cm

    ang_total_rad = math.radians(float(blob_angle_deg)) + obs_ang_offset_global
    x_rel = float(blob_dist_cm) * math.cos(ang_total_rad)
    y_rel = float(blob_dist_cm) * math.sin(ang_total_rad)
    glob_x = x_rel + x_mark
    glob_y = y_rel + y_mark
    glob_ang_deg = norm_deg_180(math.degrees(ang_total_rad))

    return obs_ang_offset_global, x_mark, y_mark, glob_ang_deg, glob_x, glob_y


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


def launch_camera_process(robot_id: str, run_env: dict):
    cam_cmd = [
        "ros2", "launch", CAMERA_PACKAGE, CAMERA_LAUNCH_FILE,
        f"namespace:={robot_id}",
    ]
    print(f"[CAM] Lanzando {CAMERA_LAUNCH_FILE} para {robot_id}...", flush=True)
    proc = subprocess.Popen(cam_cmd, env=run_env, start_new_session=True)
    time.sleep(WAIT_AFTER_CAMERA_SEC)
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
#                            CSV LOGGER DE CALIBRACIÓN
# ============================================================================
CSV_COLUMNS = [
    "timestamp_s",
    "run_id",
    "rep",
    "seg",
    "global_tick",
    "seg_tick",
    "fase",
    "target_ang_deg",
    "target_dist_cm",
    "cur_move_ang_deg",
    "cur_move_dist_cm",
    "obs_ang_global_deg",
    "x_mark_cm",
    "y_mark_cm",
    "imu_yaw_deg",
    "avg_v_m_s",
    "avg_w_rad_s",
    "n_cmd",
    "cam_seq",
    "cam_age_ms",
    "n_blobs",
    "raw_color",
    "raw_det_px",
    "raw_det_ang_deg",
    "cam_dist_cm",
    "cam_ang_deg",
    "glob_ang_deg",
    "glob_x_cm",
    "glob_y_cm",
    "ref_x_cm",
    "ref_y_cm",
    "ref_ang_deg",
    "err_x_cm",
    "err_y_cm",
    "err_cm",
    "err_ang_deg",
]


class CalibrationCsvLogger:
    def __init__(self, enabled: bool, csv_dir: str, robot_id: str):
        self.enabled = enabled
        self.filepath = None
        self._file = None
        self._writer = None
        if enabled:
            os.makedirs(csv_dir, exist_ok=True)
            stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
            self.filepath = os.path.join(csv_dir, f"apprentice_calib_{robot_id}_{stamp}.csv")
            self._file = open(self.filepath, "w", newline="", encoding="utf-8")
            self._writer = csv.DictWriter(self._file, fieldnames=CSV_COLUMNS)
            self._writer.writeheader()
            self._file.flush()

    def write_row(self, row: dict):
        if not self.enabled or self._writer is None:
            return
        self._writer.writerow(row)
        self._file.flush()

    def close(self):
        if self._file is not None:
            try:
                self._file.flush()
                self._file.close()
            except Exception:
                pass
            self._file = None
            self._writer = None


# ============================================================================
#                     NODO ROS 2 DE PRUEBA DEL APRENDIZ
# ============================================================================
def run_cli():
    parser = argparse.ArgumentParser(
        description="CLI de calibración del Aprendiz móvil (Move.move + cámara + IMU) en Qupa real"
    )
    parser.add_argument("--id", type=str, default=DEFAULT_ROBOT_ID, help=f"ID/namespace del aprendiz (default: {DEFAULT_ROBOT_ID})")
    parser.add_argument("--tps", type=int, default=TICKS_PER_SECOND, help=f"Ticks por segundo (default: {TICKS_PER_SECOND})")
    parser.add_argument("--max_fov", type=float, default=MAX_FOV_RADIUS_CM, help=f"Radio máximo FOV cámara en cm (default: {MAX_FOV_RADIUS_CM})")
    parser.add_argument(
        "--v_real",
        type=float,
        default=None,
        help="Sobreescribe Move.v_real en cm/s (si se omite, usa v_real definido en Functions/move.py)",
    )
    parser.add_argument("--gyro_scale", type=float, default=IMU_GYRO_SCALE, help=f"Factor de escala del giroscopio Z (default: {IMU_GYRO_SCALE})")
    parser.add_argument("--cam_angle_sign", type=float, default=None, help="Sobreescribe apprentice_entries.CAMERA_ANGLE_SIGN (opcional)")
    parser.add_argument("--cam_angle_offset", type=float, default=None, help="Sobreescribe apprentice_entries.CAMERA_ANGLE_OFFSET_DEG en grados (opcional)")
    parser.add_argument("--launch_camera", action="store_true", help="Lanzar camera.launch.py automáticamente")
    parser.add_argument("--no_imu", action="store_true", help="No lanzar el driver mpu6050 automáticamente")
    parser.add_argument("--no_tare_on_finish", action="store_true", help="Desactivar /tare al finalizar toda la secuencia")
    parser.add_argument("--no_retare_start", action="store_true", help="Desactivar /tare automático al iniciar cada giro en Move.move()")
    parser.add_argument("--post_ticks", type=int, default=POST_MOVE_OBSERVE_TICKS, help=f"Ticks quietos al terminar secuencia (default: {POST_MOVE_OBSERVE_TICKS})")
    parser.add_argument("--csv_dir", type=str, default=DEFAULT_CSV_DIR, help=f"Carpeta para guardar CSV (default: {DEFAULT_CSV_DIR})")
    parser.add_argument("--no_csv", action="store_true", help="Desactivar guardado de CSV")
    args = parser.parse_args()

    robot_id = normalize_robot_id(args.id)
    tps = max(1, int(args.tps))
    dt = 1.0 / float(tps)
    max_fov_cm = float(args.max_fov)
    tare_on_finish = not bool(args.no_tare_on_finish) and bool(TARE_ON_FINISH)
    retare_each = not bool(args.no_retare_start) and bool(IMU_RETARE_EACH_SEGMENT)
    effective_gyro_sign = float(IMU_GYRO_Z_SIGN) * float(args.gyro_scale)
    post_observe_ticks = max(0, int(args.post_ticks))

    # Importar ROS 2 y mensajes del robot
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from geometry_msgs.msg import Twist
    from sensor_msgs.msg import Imu
    from std_srvs.srv import Trigger
    from qupa_msgs.msg import DetectionArray

    # Registrar este módulo como controlador fuente ANTES de importar apprentice_entries
    main_mod = sys.modules[__name__]
    sys.modules["Controller.Ks6_apprentice_controller"] = main_mod
    sys.modules["Ks6_apprentice_controller"] = main_mod
    sys.modules["Controller.Ks6_master_controller"] = main_mod

    from Functions import shared_config
    from Functions import math_helper
    from Functions import move as Move
    from Functions import utils as Utils
    from Controller import apprentice_entries
    from Controller import apprentice_process

    # Respetar Move.v_real por defecto salvo override explícito por CLI
    if args.v_real is not None:
        active_v_real = float(args.v_real)
        Move.v_real = active_v_real
        v_real_source = "CLI --v_real"
    else:
        active_v_real = float(getattr(Move, "v_real", 0.0))
        v_real_source = "Functions/move.py (v_real)"

    # Overrides opcionales de cámara para pruebas rápidas de calibración
    if args.cam_angle_sign is not None:
        apprentice_entries.CAMERA_ANGLE_SIGN = float(args.cam_angle_sign)
    if args.cam_angle_offset is not None:
        apprentice_entries.CAMERA_ANGLE_OFFSET_DEG = float(args.cam_angle_offset)

    # Propagar configuración al módulo activo, shared_config, Move y apprentice_process
    cfg_vars = {
        "ticks_per_second": tps,
        "FOOTBOT_INTERWHEEL_DISTANCE": FOOTBOT_INTERWHEEL_DISTANCE,
        "IMU_GYRO_Z_SIGN": effective_gyro_sign,
        "IMU_ANGULAR_SPEED_SLOW_RAD_S": IMU_ANGULAR_SPEED_SLOW_RAD_S,
        "IMU_TURN_SLOW_ZONE_DEG": IMU_TURN_SLOW_ZONE_DEG,
        "IMU_TURN_EARLY_STOP_DEG": IMU_TURN_EARLY_STOP_DEG,
        "IMU_ADAPTIVE_EARLY_STOP": IMU_ADAPTIVE_EARLY_STOP,
        "IMU_RETARE_EACH_SEGMENT": retare_each,
        "IMU_TURN_PAUSE_S": IMU_TURN_PAUSE_S,
        "IMU_SETTLE_S": IMU_SETTLE_S,
        "IMU_TARE_TIMEOUT_S": IMU_TARE_TIMEOUT_S,
        "IMU_TIMEOUT_FACTOR": IMU_TIMEOUT_FACTOR,
        "IMU_MEASURED_LINEAR_SPEED_CM_S": active_v_real,
        "noise_cfg": noise_cfg,
    }
    for _k, _v in cfg_vars.items():
        setattr(main_mod, _k, _v)
        setattr(shared_config, _k, _v)
        if hasattr(Move, _k):
            setattr(Move, _k, _v)
        if hasattr(apprentice_process, _k):
            setattr(apprentice_process, _k, _v)
    shared_config.sync_from_controller(main_mod, extra_targets=[Move, apprentice_entries, apprentice_process])
    apprentice_process.noise_cfg = noise_cfg

    cam_proc = None
    imu_proc = None
    if args.launch_camera or AUTO_LAUNCH_CAMERA:
        cam_proc = launch_camera_process(robot_id, os.environ.copy())
    if AUTO_LAUNCH_IMU and not args.no_imu:
        imu_proc = launch_imu_process(robot_id, os.environ.copy())

    csv_logger = CalibrationCsvLogger(enabled=not args.no_csv, csv_dir=args.csv_dir, robot_id=robot_id)

    rclpy.init()

    class ApprenticeTestNode(Node):
        def __init__(self):
            super().__init__("ks6_apprentice_test_cli")
            self.cmd_vel_pub = self.create_publisher(Twist, f"/{robot_id}/cmd_vel", 10)
            self._cmd_vel_lock = threading.Lock()
            self._tick_cmd_samples = []

            self._cam_lock = threading.Lock()
            self.cam_received = False
            self.cam_seq = 0
            self.last_cam_rx_monotonic = None
            self.last_raw_targets = []

            self.rob = Utils.QupaRobotInstance(
                robot_id,
                0.0, 0.0, 0.0,
                "",
                "apprentice",
                ticks_per_second=tps,
                interwheel_distance=FOOTBOT_INTERWHEEL_DISTANCE,
            )
            self.rob.odom_received = False
            self.rob.latest_detections = []
            self.imu_tare_client = self.create_client(Trigger, f"/{robot_id}/{IMU_TARE_SERVICE_SUFFIX}")
            self.rob.imu_tare_client = self.imu_tare_client

            self.create_subscription(Imu, f"/{robot_id}/{IMU_TOPIC_SUFFIX}", self._imu_cb, 50)
            self.create_subscription(Twist, f"/{robot_id}/cmd_vel", self._cmd_vel_cb, 50)
            self.create_subscription(
                DetectionArray,
                f"/{robot_id}/{CAMERA_TOPIC_SUFFIX}",
                self._detections_cb,
                10,
            )

        def _imu_cb(self, msg):
            Move.update_imu(self.rob, msg)

        def _detections_cb(self, msg: DetectionArray):
            now_m = time.monotonic()
            apprentice_entries.update_camera_detections(self.rob, msg)
            targets = list(getattr(msg, "targets", []) or [])
            with self._cam_lock:
                self.cam_received = True
                self.cam_seq += 1
                self.last_cam_rx_monotonic = now_m
                self.last_raw_targets = targets

        def get_cam_snapshot(self):
            now_m = time.monotonic()
            with self._cam_lock:
                seq = self.cam_seq
                rx_t = self.last_cam_rx_monotonic
                raw_targets = list(self.last_raw_targets)
            age_ms = ((now_m - rx_t) * 1000.0) if rx_t is not None else -1.0
            return seq, age_ms, raw_targets

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
                print(f"[IMU] Servicio /{robot_id}/{IMU_TARE_SERVICE_SUFFIX} no está listo aún.", flush=True)
                return False
            print(f"[IMU] Ejecutando /tare ({reason}) — mantén el robot quieto...", flush=True)
            fut = self.imu_tare_client.call_async(Trigger.Request())
            t0 = time.monotonic()
            while not fut.done() and (time.monotonic() - t0) < IMU_TARE_TIMEOUT_S:
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
            print(f"[IMU] Timeout ({IMU_TARE_TIMEOUT_S:.1f}s) esperando /tare.", flush=True)
            return False

    node = ApprenticeTestNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    print("\n=====================================================================")
    print(f" CALIBRADOR DE APRENDIZ MÓVIL (Move + Cámara + IMU) -> {robot_id}")
    print("=====================================================================")
    print(f" Ticks/seg: {tps} Hz (dt={dt:.3f}s) | v_real: {active_v_real:.2f} cm/s [{v_real_source}]")
    print(
        f" Cámara: mode={apprentice_entries.CAM_CALIB_MODE} | "
        f"ANGLE_SIGN={apprentice_entries.CAMERA_ANGLE_SIGN:+.1f} | "
        f"ANGLE_OFFSET={apprentice_entries.CAMERA_ANGLE_OFFSET_DEG:+.2f}° | FOV={max_fov_cm:.1f} cm"
    )
    print(f" IMU: gyro_z={effective_gyro_sign:+.3f} | tare_inicio={retare_each} | tare_fin={tare_on_finish}")
    print(f" Tópicos: /{robot_id}/cmd_vel | /{robot_id}/{CAMERA_TOPIC_SUFFIX} | /{robot_id}/{IMU_TOPIC_SUFFIX}")
    if csv_logger.filepath:
        print(f" CSV log: {csv_logger.filepath}")

    # Estado persistente de odometría (mismas variables que apprentice_entries.py)
    state = {
        "x_ant_r": 0.0,
        "y_ant_r": 0.0,
        "srt_ang_mark_ant": 0.0,  # rad
        "ref_x": None,
        "ref_y": None,
        "ref_ang_deg": None,
        "global_tick": 0,
        "run_id": 0,
        "t_start": time.monotonic(),
    }

    def pick_target_blob(yellow_blobs, raw_targets, obs_ang_rad, x_mark, y_mark):
        """
        Selecciona la detección BLUE/YELLOW correspondiente a la baliza fija.
        Si hay más de un blob, escoge el más cercano a la referencia (o el primero).
        """
        if not yellow_blobs:
            return None, None

        # Filtrar raw_targets azules/amarillos dentro de FOV para poder emparejar metadata cruda
        raw_candidates = []
        for det in raw_targets:
            c_up = str(getattr(det, "color", "")).strip().upper()
            if c_up in ("BLUE", "YELLOW"):
                d_cm = apprentice_entries.distance_px_to_cm(float(det.distance_px))
                if max_fov_cm <= 0.0 or d_cm <= max_fov_cm:
                    raw_candidates.append(det)

        best_idx = 0
        if len(yellow_blobs) > 1 and state["ref_x"] is not None:
            best_dist = float("inf")
            for idx, b in enumerate(yellow_blobs):
                ang_tot = math.radians(b["angle"]) + obs_ang_rad
                xr, yr = math_helper.polarACartesianoPAR(b["distance"], ang_tot)
                gx, gy = xr + x_mark, yr + y_mark
                d = math.hypot(gx - state["ref_x"], gy - state["ref_y"])
                if d < best_dist:
                    best_dist = d
                    best_idx = idx

        blob = yellow_blobs[best_idx]
        raw_det = raw_candidates[best_idx] if best_idx < len(raw_candidates) else (raw_candidates[0] if raw_candidates else None)
        return blob, raw_det

    def print_table_header():
        print(
            "  tick | fase        | giro(°)  | avanc(cm) | cam(°, cm)       | pos_correg(x, y cm) | ang_cor(°) | err_cm | age_ms"
        )
        print(
            "  -----+-------------+----------+-----------+------------------+---------------------+------------+--------+-------"
        )

    def process_and_log_tick(
        run_id: int,
        rep_idx: int,
        seg_idx: int,
        seg_tick: int,
        fase: str,
        target_ang_deg: float,
        target_dist_cm: float,
        cur_move_dist: float,
        cur_move_ang_rad: float,
        avg_v: float,
        avg_w: float,
        n_cmd: int,
    ):
        cam_seq, cam_age_ms, raw_targets = node.get_cam_snapshot()
        yellow_blobs, _ = apprentice_entries.collect_omnidirectional_blobs(node.rob, None, max_fov_cm, None)

        # Fase A.5 exacta de apprentice_entries.py
        obs_ang_offset_global = apprentice_entries._norm_rad(state["srt_ang_mark_ant"] + float(cur_move_ang_rad))
        dx_mov_cm, dy_mov_cm = math_helper.polarACartesianoPAR(float(cur_move_dist), obs_ang_offset_global)
        x_mark = state["x_ant_r"] + dx_mov_cm
        y_mark = state["y_ant_r"] + dy_mov_cm
        obs_ang_deg = math.degrees(obs_ang_offset_global)
        imu_yaw_deg = math.degrees(float(getattr(node.rob, "imu_yaw", 0.0)))

        blob, raw_det = pick_target_blob(yellow_blobs, raw_targets, obs_ang_offset_global, x_mark, y_mark)

        if blob is not None:
            cam_ang_deg = float(blob["angle"])
            cam_dist_cm = float(blob["distance"])
            # Fase H exacta de apprentice_process.py
            ang_total_rad = math.radians(cam_ang_deg) + obs_ang_offset_global
            x_rel, y_rel = math_helper.polarACartesianoPAR(cam_dist_cm, ang_total_rad)
            glob_x = x_rel + x_mark
            glob_y = y_rel + y_mark
            glob_ang_deg = math_helper.normalize_deg_180(math.degrees(ang_total_rad))

            if state["ref_x"] is not None:
                err_x = glob_x - state["ref_x"]
                err_y = glob_y - state["ref_y"]
                err_cm = math.hypot(err_x, err_y)
                err_ang_deg = math_helper.normalize_deg_180(glob_ang_deg - state["ref_ang_deg"])
            else:
                err_x = err_y = err_cm = err_ang_deg = None

            raw_color = str(getattr(raw_det, "color", "")) if raw_det is not None else ""
            raw_px = float(getattr(raw_det, "distance_px", 0.0)) if raw_det is not None else ""
            raw_ang = float(getattr(raw_det, "angle_deg", 0.0)) if raw_det is not None else ""

            err_str = f"{err_cm:6.2f}" if err_cm is not None else "   -- "
            cur_ang_deg = math.degrees(float(cur_move_ang_rad))
            print(
                f"  {seg_tick:4d} | {fase:<11s} | {cur_ang_deg:+8.2f} | {cur_move_dist:9.2f} | "
                f"{cam_ang_deg:+6.1f}°, {cam_dist_cm:5.1f} cm | "
                f"{glob_x:+8.2f}, {glob_y:+8.2f}  | {glob_ang_deg:+10.2f} | {err_str} | {cam_age_ms:6.0f}",
                flush=True,
            )
        else:
            cam_ang_deg = cam_dist_cm = glob_x = glob_y = glob_ang_deg = None
            err_x = err_y = err_cm = err_ang_deg = None
            raw_color = raw_px = raw_ang = ""
            cur_ang_deg = math.degrees(float(cur_move_ang_rad))
            print(
                f"  {seg_tick:4d} | {fase:<11s} | {cur_ang_deg:+8.2f} | {cur_move_dist:9.2f} | "
                f"     SIN BLOB    |         ---         |        --- |    --  | {cam_age_ms:6.0f}",
                flush=True,
            )

        csv_logger.write_row({
            "timestamp_s": f"{time.monotonic() - state['t_start']:.4f}",
            "run_id": run_id,
            "rep": rep_idx,
            "seg": seg_idx,
            "global_tick": state["global_tick"],
            "seg_tick": seg_tick,
            "fase": fase,
            "target_ang_deg": f"{target_ang_deg:.3f}",
            "target_dist_cm": f"{target_dist_cm:.3f}",
            "cur_move_ang_deg": f"{math.degrees(cur_move_ang_rad):.4f}",
            "cur_move_dist_cm": f"{cur_move_dist:.4f}",
            "obs_ang_global_deg": f"{obs_ang_deg:.4f}",
            "x_mark_cm": f"{x_mark:.4f}",
            "y_mark_cm": f"{y_mark:.4f}",
            "imu_yaw_deg": f"{imu_yaw_deg:.4f}",
            "avg_v_m_s": f"{avg_v:.5f}",
            "avg_w_rad_s": f"{avg_w:.5f}",
            "n_cmd": n_cmd,
            "cam_seq": cam_seq,
            "cam_age_ms": f"{cam_age_ms:.1f}",
            "n_blobs": len(yellow_blobs),
            "raw_color": raw_color,
            "raw_det_px": f"{raw_px:.3f}" if raw_px != "" else "",
            "raw_det_ang_deg": f"{raw_ang:.3f}" if raw_ang != "" else "",
            "cam_dist_cm": f"{cam_dist_cm:.4f}" if cam_dist_cm is not None else "",
            "cam_ang_deg": f"{cam_ang_deg:.4f}" if cam_ang_deg is not None else "",
            "glob_ang_deg": f"{glob_ang_deg:.4f}" if glob_ang_deg is not None else "",
            "glob_x_cm": f"{glob_x:.4f}" if glob_x is not None else "",
            "glob_y_cm": f"{glob_y:.4f}" if glob_y is not None else "",
            "ref_x_cm": f"{state['ref_x']:.4f}" if state["ref_x"] is not None else "",
            "ref_y_cm": f"{state['ref_y']:.4f}" if state["ref_y"] is not None else "",
            "ref_ang_deg": f"{state['ref_ang_deg']:.4f}" if state["ref_ang_deg"] is not None else "",
            "err_x_cm": f"{err_x:.4f}" if err_x is not None else "",
            "err_y_cm": f"{err_y:.4f}" if err_y is not None else "",
            "err_cm": f"{err_cm:.4f}" if err_cm is not None else "",
            "err_ang_deg": f"{err_ang_deg:.4f}" if err_ang_deg is not None else "",
        })

        return {
            "obs_ang_rad": obs_ang_offset_global,
            "x_mark": x_mark,
            "y_mark": y_mark,
            "glob_x": glob_x,
            "glob_y": glob_y,
            "glob_ang_deg": glob_ang_deg,
            "err_cm": err_cm,
            "err_ang_deg": err_ang_deg,
        }

    def observe_stationary(n_ticks: int, fase_label: str, set_as_reference: bool = False):
        samples = []
        print_table_header()
        for k in range(1, n_ticks + 1):
            t0 = time.monotonic()
            state["global_tick"] += 1
            node.publish_cmd(0.0, 0.0)
            res = process_and_log_tick(
                run_id=state["run_id"],
                rep_idx=0,
                seg_idx=0,
                seg_tick=k,
                fase=fase_label,
                target_ang_deg=0.0,
                target_dist_cm=0.0,
                cur_move_dist=0.0,
                cur_move_ang_rad=0.0,
                avg_v=0.0,
                avg_w=0.0,
                n_cmd=0,
            )
            if res["glob_x"] is not None:
                samples.append(res)
            sleep_s = dt - (time.monotonic() - t0)
            if sleep_s > 0.0:
                time.sleep(sleep_s)

        if set_as_reference and samples:
            avg_x = sum(s["glob_x"] for s in samples) / len(samples)
            avg_y = sum(s["glob_y"] for s in samples) / len(samples)
            # Promedio circular de ángulos
            sin_sum = sum(math.sin(math.radians(s["glob_ang_deg"])) for s in samples)
            cos_sum = sum(math.cos(math.radians(s["glob_ang_deg"])) for s in samples)
            avg_ang = math_helper.normalize_deg_180(math.degrees(math.atan2(sin_sum, cos_sum)))
            state["ref_x"] = avg_x
            state["ref_y"] = avg_y
            state["ref_ang_deg"] = avg_ang
            print(
                f"[REF] Baliza fija capturada ({len(samples)}/{n_ticks} muestras): "
                f"ref_x={avg_x:+.2f} cm, ref_y={avg_y:+.2f} cm, ref_ang={avg_ang:+.2f}°",
                flush=True,
            )
        elif set_as_reference and not samples:
            print(
                "[AVISO] No se detectó ningún blob BLUE/YELLOW al capturar referencia. "
                "Verifica los LEDs azules del maestro y ejecuta 'Z' cuando aparezca.",
                flush=True,
            )
        return samples

    last_command = None

    try:
        # 1) Esperar confirmación obligatoria de IMU y Cámara
        print(
            f"\n[INIT] Esperando IMU (/{robot_id}/{IMU_TOPIC_SUFFIX}) y Cámara (/{robot_id}/{CAMERA_TOPIC_SUFFIX})...",
            flush=True,
        )
        t_wait = time.monotonic()
        last_log_t = 0.0
        while True:
            imu_ok = bool(getattr(node.rob, "imu_received", False))
            tare_srv_ok = node.imu_tare_client.service_is_ready()
            cam_ok = bool(node.cam_received)
            if imu_ok and tare_srv_ok and cam_ok:
                break
            elapsed_w = time.monotonic() - t_wait
            if elapsed_w - last_log_t >= 2.0:
                print(
                    f"[INIT] Esperando ({elapsed_w:.0f}s)... "
                    f"IMU={'OK' if imu_ok else 'pendiente'} | "
                    f"/tare={'OK' if tare_srv_ok else 'pendiente'} | "
                    f"Cámara={'OK' if cam_ok else 'pendiente (lanza camera.launch.py o usa --launch_camera)'}",
                    flush=True,
                )
                last_log_t = elapsed_w
            time.sleep(0.1)

        print(
            f"[OK] IMU + Cámara confirmadas en {time.monotonic() - t_wait:.1f}s "
            f"(yaw inicial = {math.degrees(node.rob.imu_yaw):+.2f}°).",
            flush=True,
        )

        # 2) Capturar referencia inicial de la baliza fija (en reposo)
        print(f"\n[REF] Capturando posición inicial de la baliza fija ({PRE_MOVE_OBSERVE_TICKS} ticks quieto)...", flush=True)
        observe_stationary(PRE_MOVE_OBSERVE_TICKS, "REF_INI", set_as_reference=True)

        print("\nFormato de entrada:")
        print("  (ang1,dis1),(ang2,dis2),...,(ang_i,dis_i), vel_ang, vel_lin, n")
        print("  Ejemplos: (45,0), 44,7,1   |   (45,20), 44,7,1   |   (45,0),(-45,0), 44,7,1")
        print("  Comandos: R = repetir | S = observar quieto | Z = cero odometría + nueva ref | T = tare | N = salir\n")

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

            if upper == "Z":
                node.stop_motors()
                state["x_ant_r"] = 0.0
                state["y_ant_r"] = 0.0
                state["srt_ang_mark_ant"] = 0.0
                state["ref_x"] = None
                state["ref_y"] = None
                state["ref_ang_deg"] = None
                print("[ODOM] Origen reiniciado a (x=0.00, y=0.00, yaw=0.00°). Recapturando referencia...", flush=True)
                observe_stationary(PRE_MOVE_OBSERVE_TICKS, "REF_Z", set_as_reference=True)
                continue

            if upper == "S":
                node.stop_motors()
                state["run_id"] += 1
                print(f"\n[QUIETO] Observando baliza durante {STATIONARY_CHECK_TICKS} ticks ({STATIONARY_CHECK_TICKS * dt:.1f}s)...", flush=True)
                observe_stationary(STATIONARY_CHECK_TICKS, "QUIETO", set_as_reference=(state["ref_x"] is None))
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

            # Si aún no había referencia, intentar capturarla antes de moverse
            if state["ref_x"] is None:
                print("[REF] No había referencia previa; capturando ahora antes de mover...", flush=True)
                observe_stationary(PRE_MOVE_OBSERVE_TICKS, "REF_PRE", set_as_reference=True)

            state["run_id"] += 1
            run_id = state["run_id"]
            seq_t0 = time.monotonic()
            total_segs = len(segments)
            motion_err_cm = []
            motion_err_ang = []

            print(
                f"\n[RUN #{run_id}] {total_segs} segmento(s) x {repetitions} rep | "
                f"vel_ang={speed_g:.2f} °/s, vel_lin={speed_d:.2f} cm/s, v_real={active_v_real:.2f} cm/s | "
                f"Pose inicial=({state['x_ant_r']:+.2f}, {state['y_ant_r']:+.2f} cm, {math.degrees(state['srt_ang_mark_ant']):+.2f}°)",
                flush=True,
            )

            for rep_idx in range(1, repetitions + 1):
                for seg_idx, (ang_deg, dist_cm) in enumerate(segments, start=1):
                    Move.r_move(node.rob)
                    node.rob.c1 = 0
                    node.rob.c2 = 0

                    print(
                        f"\n[REP {rep_idx}/{repetitions} | SEG {seg_idx}/{total_segs}] "
                        f"Objetivo -> ang={ang_deg:+.2f}°, dis={dist_cm:.2f} cm",
                        flush=True,
                    )
                    print_table_header()

                    seg_tick = 0
                    last_tick_res = None
                    while node.rob.c1 != 2:
                        tick_start = time.monotonic()
                        state["global_tick"] += 1
                        seg_tick += 1

                        node.clear_tick_cmd_samples()

                        # Fase 0 exacta de apprentice_entries.py
                        c2_out, c1_out, cur_move_dist, cur_move_ang = Move.move(
                            node.rob,
                            dist_cm,
                            ang_deg,
                            speed_d,
                            speed_g,
                            state["global_tick"],
                            False,
                        )
                        node.publish_cmd(node.rob.cmd_linear_x, node.rob.cmd_angular_z)

                        if c1_out == 0:
                            fase = f"GIRO:{getattr(node.rob, 'imu_turn_substate', 'TURN')}"
                        elif c1_out == 1:
                            fase = "AVANCE"
                        else:
                            fase = "FIN"

                        # Esperar el resto del tick (igual que 4 Hz) para leer cámara + cmd_vel
                        elapsed_tick = time.monotonic() - tick_start
                        sleep_s = dt - elapsed_tick
                        if sleep_s > 0.0:
                            time.sleep(sleep_s)

                        avg_v, avg_w, n_cmd = node.consume_tick_cmd_averages()
                        last_tick_res = process_and_log_tick(
                            run_id=run_id,
                            rep_idx=rep_idx,
                            seg_idx=seg_idx,
                            seg_tick=seg_tick,
                            fase=fase,
                            target_ang_deg=ang_deg,
                            target_dist_cm=dist_cm,
                            cur_move_dist=float(cur_move_dist),
                            cur_move_ang_rad=float(cur_move_ang),
                            avg_v=avg_v,
                            avg_w=avg_w,
                            n_cmd=n_cmd,
                        )
                        if last_tick_res["err_cm"] is not None:
                            motion_err_cm.append(last_tick_res["err_cm"])
                            motion_err_ang.append(last_tick_res["err_ang_deg"])

                    # Commit exacto de apprentice_entries.py al terminar segmento (c1 == 2)
                    if last_tick_res is not None:
                        state["x_ant_r"] = last_tick_res["x_mark"]
                        state["y_ant_r"] = last_tick_res["y_mark"]
                        state["srt_ang_mark_ant"] = last_tick_res["obs_ang_rad"]
                    Move.r_move(node.rob)
                    node.stop_motors()

            # Observación post-movimiento en reposo (separa retardo de cámara ~2 Hz vs error real de calibración)
            post_samples = []
            if post_observe_ticks > 0:
                print(
                    f"\n[POST] Observando en reposo ({post_observe_ticks} ticks = {post_observe_ticks * dt:.1f}s) "
                    f"para verificar error estacionario sin retardo de cámara...",
                    flush=True,
                )
                post_samples = observe_stationary(post_observe_ticks, "POST_QUIETO", set_as_reference=False)

            # Resumen diagnóstico del Run
            print(
                f"\n==================== RESUMEN RUN #{run_id} ({time.monotonic() - seq_t0:.2f}s) ====================",
                flush=True,
            )
            print(
                f"  Pose acumulada aprendiz: x={state['x_ant_r']:+.2f} cm, y={state['y_ant_r']:+.2f} cm, "
                f"yaw={math.degrees(state['srt_ang_mark_ant']):+.2f}°",
                flush=True,
            )
            if state["ref_x"] is not None:
                print(
                    f"  Referencia baliza fija : ref_x={state['ref_x']:+.2f} cm, ref_y={state['ref_y']:+.2f} cm, "
                    f"ref_ang={state['ref_ang_deg']:+.2f}°",
                    flush=True,
                )
            if motion_err_cm:
                max_e_cm = max(motion_err_cm)
                max_e_ang = max(abs(a) for a in motion_err_ang)
                print(
                    f"  Durante movimiento     : max err_pos={max_e_cm:.2f} cm | max |err_ang|={max_e_ang:.2f}° "
                    f"(incluye latencia de cámara ~2 Hz)",
                    flush=True,
                )
            if post_samples and state["ref_x"] is not None:
                # Usar las últimas 2-3 muestras post-parada (cuando la cámara ya se estabilizó)
                tail_s = post_samples[-min(3, len(post_samples)):]
                fin_x = sum(s["glob_x"] for s in tail_s) / len(tail_s)
                fin_y = sum(s["glob_y"] for s in tail_s) / len(tail_s)
                sin_s = sum(math.sin(math.radians(s["glob_ang_deg"])) for s in tail_s)
                cos_s = sum(math.cos(math.radians(s["glob_ang_deg"])) for s in tail_s)
                fin_ang = math_helper.normalize_deg_180(math.degrees(math.atan2(sin_s, cos_s)))
                fin_err_cm = math.hypot(fin_x - state["ref_x"], fin_y - state["ref_y"])
                fin_err_ang = math_helper.normalize_deg_180(fin_ang - state["ref_ang_deg"])
                print(
                    f"  Final quieto (estable) : glob_x={fin_x:+.2f} cm, glob_y={fin_y:+.2f} cm, glob_ang={fin_ang:+.2f}°",
                    flush=True,
                )
                print(
                    f"  ERROR REAL (estacion.) : err_pos={fin_err_cm:.2f} cm | err_ang={fin_err_ang:+.2f}°",
                    flush=True,
                )
            if csv_logger.filepath:
                print(f"  Datos guardados en     : {csv_logger.filepath}", flush=True)
            print("=====================================================================\n", flush=True)

            if tare_on_finish:
                time.sleep(0.3)
                node.call_tare("fin de secuencia")

    except KeyboardInterrupt:
        print("\n[CLI] Interrumpido por Ctrl+C.")
    finally:
        print("[CLI] Deteniendo motores y cerrando recursos...", flush=True)
        try:
            node.stop_motors()
        except Exception:
            pass
        csv_logger.close()
        try:
            executor.shutdown()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass
        stop_process_group(imu_proc)
        stop_process_group(cam_proc)
        print("[CLI] Cerrado correctamente.", flush=True)


if __name__ == "__main__":
    run_cli()