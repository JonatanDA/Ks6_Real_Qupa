# Archivo: Ks6_Real_Qupa/Functions/move.py  [CASO 3: GIROS CON IMU (MPU6050) + AVANCE CALIBRABLE - Qupa Real]
# Ubicación: ~/sim_ws_qupa/Ks6_Real_Qupa/Functions/move.py
# Descripción:
#   Módulo compartido de movimiento y navegación para los Qupa en ROS 2 (Hardware Real con MPU6050):
#   move, r_move, redondear_resolucion, RandomWalking, navegar_a_coordenadas, Save_position y update_imu.
#   - Fase 1 (c1 == 0): Giro en lazo cerrado con giroscopio del MPU6050 (/{robot_id}/mpu6050/imu/data_raw),
#     zona lenta, frenado anticipado adaptativo (adaptive_early_stop) y re-encerado opcional por vértice.
#   - Fase 2 (c1 == 1): Avance lineal por tiempo calibrable (v_real / IMU_MEASURED_LINEAR_SPEED_CM_S o speed_d),
#     integrando (rob.x, rob.y) con el yaw real del giroscopio cuando no hay /odom.
#   Mantiene idéntica firma de entrada y salidas por tick:
#     move(rob, distance, target_angle, speed_d, speed_g, timer=1, stop=False)
#       -> (rob.c2, rob.c1, rob.move_distance, rob.move_angle)
#   Equivalente 1:1 de Ks6/Functions/move.lua.

import math
import random
import time

from Functions import shared_config
from Functions import sensors as sensors_mod
from Functions import utils as utils_mod

try:
    from std_srvs.srv import Trigger as _StdTrigger
except ImportError:
    _StdTrigger = None


# ============================================================================
# CALIBRACIÓN DE VELOCIDAD LINEAL REAL POR ROBOT (ROBOT_V_REAL / v_real)
# ============================================================================
# Fuente única de verdad de velocidad lineal real medida con regla [cm/s].
#   - Se busca primero el ID del robot en ROBOT_V_REAL (acepta "qupa_48", "48", etc.).
#   - Si el ID no está en ROBOT_V_REAL, se usa `v_real` como respaldo por defecto.
#   - Si el resultado es 0.0, se usa directamente la velocidad pedida (speed_d).
ROBOT_V_REAL = {
    "qupa_48": 15.3,   # @ vel_lin = 7.0 cm/s (0.07 m/s, 82% PWM, ~7.7V)
    "qupa_AE": 9.2,    # @ vel_lin = 8.0 cm/s (0.08 m/s, 90% PWM, ~7.8V)
    "qupa_D2": 15.3,   # pendiente de calibrar con regla
    "qupa_3A": 15.3,   # pendiente de calibrar con regla
    "qupa_F4": 15.3,   # pendiente de calibrar con regla
}
v_real = 15.3          # Respaldo por defecto si el robot_id no está en ROBOT_V_REAL


def get_v_real(robot_id=None) -> float:
    """Devuelve v_real [cm/s] para `robot_id` desde ROBOT_V_REAL, o el respaldo `v_real`."""
    if robot_id is not None:
        raw = str(robot_id).strip().lstrip("/")
        candidates = [raw, raw.lower(), raw.upper()]
        if not raw.lower().startswith("qupa"):
            candidates.extend([f"qupa_{raw}", f"qupa_{raw.lower()}", f"qupa_{raw.upper()}"])
        lookup = {k.lower(): float(val) for k, val in ROBOT_V_REAL.items()}
        for cand in candidates:
            if cand.lower() in lookup:
                return lookup[cand.lower()]
    return float(v_real)


# ============================================================================
# LÍMITES FÍSICOS DE SATURACIÓN DEL ROBOT QUPA (GAZEBO Y REAL)
# ============================================================================
MAX_LINEAR_SPEED_CM_S   = 8.0                   # 8.0 cm/s = 0.08 m/s
MAX_LINEAR_SPEED_M_S    = 0.08                  # m/s
MAX_ANGULAR_SPEED_RAD_S = 0.776                 # rad/s
MAX_ANGULAR_SPEED_DEG_S = math.degrees(0.776)   # ~44.4615 °/s

# Configuración centralizada de IMU (MPU6050) — fuente única de verdad para todos los controladores y scripts CLI.
IMU_TOPIC_SUFFIX               = "mpu6050/imu/data_raw"
IMU_TARE_SERVICE_SUFFIX        = "mpu6050/tare"
IMU_GYRO_Z_SIGN                = 1.0     # -1.0 si el eje Z del MPU6050 apunta hacia abajo
IMU_ANGULAR_SPEED_SLOW_RAD_S   = 0.72    # rad/s en zona lenta (> 0.70 rad/s deadband físico)
IMU_TURN_SLOW_ZONE_DEG         = 30.0    # ° finales a velocidad lenta
IMU_TURN_EARLY_STOP_DEG        = 15.0    # ° iniciales antes de la meta para absorber inercia
IMU_ADAPTIVE_EARLY_STOP        = True    # corrige early_stop con el sobrepaso medido en cada giro
IMU_RETARE_EACH_SEGMENT        = True    # True: re-encera en cada vértice + pausa de frenado; False: sin espera
IMU_TURN_PAUSE_S               = 0.5     # s quieto tras frenar el giro (solo si IMU_RETARE_EACH_SEGMENT=True)
IMU_SETTLE_S                   = 0.3     # s de estabilización tras cada tare (solo si IMU_RETARE_EACH_SEGMENT=True)
IMU_TARE_TIMEOUT_S             = 5.0     # s máximo de espera al servicio /tare
IMU_TIMEOUT_FACTOR             = 2.5     # seguridad: corta un giro tras factor × tiempo esperado
IMU_MEASURED_LINEAR_SPEED_CM_S = v_real  # respaldo sincronizado con v_real

# Corrección de avance en línea recta con IMU durante Fase 2 (AVANCE / reversa)
ENABLE_IMU_STRAIGHT_HOLD       = True    # True: corrige desvíos laterales usando imu_yaw al avanzar/retroceder
IMU_STRAIGHT_KP                = 2.2     # Ganancia proporcional [rad/s por rad de error de rumbo]
IMU_STRAIGHT_MAX_W_RAD_S       = 0.45    # Límite de corrección angular [rad/s] (suave, sin invertir ruedas)
IMU_STRAIGHT_DEADBAND_DEG      = 1.0     # Banda muerta [°] para ignorar ruido mínimo del giroscopio


def _get_max_linear_cm_s() -> float:
    return float(shared_config.get_controller_var("MAX_LINEAR_SPEED_CM_S", MAX_LINEAR_SPEED_CM_S))


def _get_max_angular_rad_s() -> float:
    return float(shared_config.get_controller_var("MAX_ANGULAR_SPEED_RAD_S", MAX_ANGULAR_SPEED_RAD_S))


def clamp_cmd_vel(lin_m_s: float, ang_rad_s: float):
    """Satura (linear.x [m/s], angular.z [rad/s]) a los límites reales del Qupa."""
    max_lin_m_s = _get_max_linear_cm_s() / 100.0
    max_ang_rad_s = _get_max_angular_rad_s()
    lin_clamped = max(-max_lin_m_s, min(max_lin_m_s, float(lin_m_s)))
    ang_clamped = max(-max_ang_rad_s, min(max_ang_rad_s, float(ang_rad_s)))
    return lin_clamped, ang_clamped


def _norm_rad(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def _ensure_imu_state(rob):
    """Inicializa de forma segura los atributos de estado del IMU sobre `rob`."""
    if not hasattr(rob, 'imu_received'):
        rob.imu_received = False
    if not hasattr(rob, 'imu_last_t'):
        rob.imu_last_t = None
    if not hasattr(rob, 'imu_gz'):
        rob.imu_gz = 0.0
    if not hasattr(rob, 'imu_yaw'):
        rob.imu_yaw = float(getattr(rob, 'yaw', 0.0))
    if not hasattr(rob, 'imu_seg_theta'):
        rob.imu_seg_theta = 0.0
    if not hasattr(rob, 'imu_integrating_turn'):
        rob.imu_integrating_turn = False
    if not hasattr(rob, 'imu_turn_substate'):
        rob.imu_turn_substate = 'INIT'
    if not hasattr(rob, 'imu_turn_start_t'):
        rob.imu_turn_start_t = 0.0
    if not hasattr(rob, 'imu_pause_start_t'):
        rob.imu_pause_start_t = 0.0
    if not hasattr(rob, 'imu_tare_future'):
        rob.imu_tare_future = None
    if not hasattr(rob, 'imu_tare_start_t'):
        rob.imu_tare_start_t = 0.0
    if not hasattr(rob, 'imu_settle_start_t'):
        rob.imu_settle_start_t = 0.0
    if not hasattr(rob, 'imu_turn_timeout'):
        rob.imu_turn_timeout = False
    if not hasattr(rob, 'imu_early_stop_rad'):
        init_early_deg = float(shared_config.get_controller_var("IMU_TURN_EARLY_STOP_DEG", IMU_TURN_EARLY_STOP_DEG))
        rob.imu_early_stop_rad = math.radians(max(0.0, init_early_deg))
    if not hasattr(rob, 'imu_straight_ref_yaw'):
        rob.imu_straight_ref_yaw = None


def update_imu(rob, msg, now_s: float = None):
    """
    Callback de integración a alta frecuencia (~50 Hz) para `/{robot_id}/mpu6050/imu/data_raw`.
    Integra `rob.imu_yaw` (y `rob.yaw` si no hay /odom) y acumula `rob.imu_seg_theta`
    durante las fases TURN y PAUSE_T para capturar el ángulo real y el sobrepaso de frenado.
    """
    _ensure_imu_state(rob)
    if now_s is None:
        now_s = time.monotonic()

    if not rob.imu_received:
        rob.imu_received = True
        rob.imu_yaw = float(getattr(rob, 'yaw', 0.0))
        rob.imu_last_t = float(now_s)
        return

    dt = float(now_s) - float(rob.imu_last_t) if rob.imu_last_t is not None else 0.0
    rob.imu_last_t = float(now_s)
    if dt <= 0.0 or dt > 0.2:
        return

    gz_sign = float(shared_config.get_controller_var("IMU_GYRO_Z_SIGN", IMU_GYRO_Z_SIGN))
    w = gz_sign * float(msg.angular_velocity.z)
    rob.imu_gz = w

    rob.imu_yaw = _norm_rad(rob.imu_yaw + w * dt)
    if not getattr(rob, 'odom_received', False):
        rob.yaw = rob.imu_yaw

    if rob.imu_integrating_turn:
        rob.imu_seg_theta += w * dt


def _reset_imu_segment_state(rob):
    """Limpia el estado temporal del segmento actual sin borrar el `imu_early_stop_rad` aprendido."""
    rob.imu_integrating_turn = False
    rob.imu_seg_theta = 0.0
    rob.imu_turn_substate = 'INIT'
    rob.imu_turn_start_t = 0.0
    rob.imu_pause_start_t = 0.0
    rob.imu_tare_future = None
    rob.imu_tare_start_t = 0.0
    rob.imu_settle_start_t = 0.0
    rob.imu_turn_timeout = False
    rob.imu_straight_ref_yaw = None


def _integrate_open_loop_pose(rob, dt: float):
    """
    Integra la pose (x, y, yaw) cuando no existe tópico /odom.
    Si el IMU está activo, mantiene `rob.yaw` desde el giroscopio real y solo integra (x, y).
    """
    if getattr(rob, 'odom_received', False):
        return
    if dt <= 0.0:
        return
    v = float(getattr(rob, 'cmd_linear_x', 0.0))
    w = float(getattr(rob, 'cmd_angular_z', 0.0))
    if not getattr(rob, 'imu_received', False):
        rob.yaw = _norm_rad(float(getattr(rob, 'yaw', 0.0)) + w * dt)
    rob.x = float(getattr(rob, 'x', 0.0)) + v * math.cos(rob.yaw) * dt
    rob.y = float(getattr(rob, 'y', 0.0)) + v * math.sin(rob.yaw) * dt


# ============================================================================
# 1. GUARDADO DE POSICIÓN
# ============================================================================
posiciones_guardadas = {}


def Save_position(rob, imprimir: bool = False, name: str = "Unknown", log_fn=print):
    if name not in rob.posiciones_guardadas:
        rob.posiciones_guardadas[name] = []
    if name not in posiciones_guardadas:
        posiciones_guardadas[name] = []

    posicion = [rob.x, rob.y]
    rob.posiciones_guardadas[name].append(posicion)
    posiciones_guardadas[name].append(posicion)

    if imprimir:
        log_fn(f"Guardando: [{name}] Posición en x: {posicion[0]}, Posición en y: {posicion[1]}")


# ============================================================================
# 2. CONTROLADOR CINEMÁTICO DE MOVIMIENTO (move, r_move, redondear_resolucion)
# ============================================================================
def redondear_resolucion(valor: float, min_speed: float, tps: float) -> float:
    res = min_speed / tps
    abs_val = abs(valor)
    rounded = math.floor((abs_val / res) + 0.5) * res
    return -rounded if (valor < 0) else rounded


c1                  = 0
c2                  = 0
move_controller     = 1
move_distance       = 0.0

ticks_needed        = 0
actual_dis_ticks    = 0

angle_ticks_needed  = 0
actual_ang_ticks    = 0

time_needed         = 0
speed               = shared_config.get_controller_var("d_speed", 30.0)

move_angle          = 0.0

wheel_speed         = 0.0
omega_rad           = 0.0

previous_speed_d    = None

full_ticks_a        = 0
has_rem_a           = False
rem_wheel_speed     = 0.0
rem_omega_rad       = 0.0

full_ticks_d        = 0
has_rem_d           = False
rem_speed_d         = 0.0

min_speed_d, min_speed_g = 5.0, 1.0  # Velocidades mínimas para redondeo de resolución (en cm/s y °/s)


def set_wheels_cm_s(rob, left_cm_s: float, right_cm_s: float, interwheel_distance_cm: float = None):
    if interwheel_distance_cm is None:
        interwheel_distance_cm = getattr(
            rob,
            'interwheel_distance',
            shared_config.get_controller_var("FOOTBOT_INTERWHEEL_DISTANCE", 14.0),
        )
    L = float(interwheel_distance_cm)
    if abs(left_cm_s + right_cm_s) < 1e-6:
        rob.cmd_linear_x = 0.0
        rob.cmd_angular_z = (right_cm_s - left_cm_s) / L
    else:
        v_avg_cm_s = 0.5 * (left_cm_s + right_cm_s)
        rob.cmd_linear_x = v_avg_cm_s / 100.0
        rob.cmd_angular_z = (right_cm_s - left_cm_s) / L

    rob.cmd_linear_x, rob.cmd_angular_z = clamp_cmd_vel(rob.cmd_linear_x, rob.cmd_angular_z)

    tps = float(getattr(rob, 'ticks_per_second', shared_config.get_controller_var("ticks_per_second", 4.0)))
    dt = 1.0 / tps if tps > 0.0 else 0.25
    _integrate_open_loop_pose(rob, dt)


def move(
    rob,
    distance: float,
    target_angle: float,
    speed_d: float,
    speed_g: float,
    timer: int = 1,
    stop: bool = False,
):
    """
    Controlador de movimiento con IMU (MPU6050) para el Qupa físico:
    - Fase 1 (c1 == 0): Gira en lazo cerrado usando el giroscopio del MPU6050 a ~50 Hz,
      con zona lenta, early_stop adaptativo y re-encerado opcional en cada vértice
      (controlado por `IMU_RETARE_EACH_SEGMENT`). Si `IMU_RETARE_EACH_SEGMENT` es False,
      no realiza tare ni pausas adicionales dentro de `move()`.
      Reporta en cada tick `rob.move_angle` (rad) medido por el giroscopio.
    - Fase 2 (c1 == 1): Avanza por tiempo usando `v_real` / `IMU_MEASURED_LINEAR_SPEED_CM_S`
      (o `speed_d`), reportando en cada tick `rob.move_distance` (cm) e integrando
      (rob.x, rob.y) con `rob.yaw` real.
    """
    global c1, c2, move_controller, move_distance, move_angle
    global ticks_needed, actual_dis_ticks, angle_ticks_needed, actual_ang_ticks
    global time_needed, wheel_speed, omega_rad

    _ensure_imu_state(rob)
    rob.cmd_linear_x, rob.cmd_angular_z = clamp_cmd_vel(rob.cmd_linear_x, rob.cmd_angular_z)

    tps = float(getattr(rob, 'ticks_per_second', shared_config.get_controller_var("ticks_per_second", 4.0)))
    dt = 1.0 / tps if tps > 0.0 else 0.25

    utils_mod.Temporizador_2(rob, 0.3, tps)
    if stop or rob.temp_2:
        rob.cmd_linear_x = 0.0
        rob.cmd_angular_z = 0.0
        c1, c2 = rob.c1, rob.c2
        move_distance, move_angle = rob.move_distance, rob.move_angle
        return rob.c2, rob.c1, rob.move_distance, rob.move_angle

    # Guardrail: exigir que el nodo mpu6050 esté publicando antes de girar
    if not getattr(rob, 'imu_received', False):
        rob.cmd_linear_x = 0.0
        rob.cmd_angular_z = 0.0
        if not getattr(rob, '_imu_wait_warned', False):
            rob._imu_wait_warned = True
            imu_suffix = shared_config.get_controller_var("IMU_TOPIC_SUFFIX", IMU_TOPIC_SUFFIX)
            shared_config.log_py(
                f"[MOVE_IMU] Esperando mensajes del MPU6050 en /{rob.id}/{imu_suffix}..."
            )
        c1, c2 = rob.c1, rob.c2
        move_distance, move_angle = rob.move_distance, rob.move_angle
        return rob.c2, rob.c1, rob.move_distance, rob.move_angle
    rob._imu_wait_warned = False

    while target_angle > 180.0:
        target_angle -= 360.0
    while target_angle <= -180.0:
        target_angle += 360.0

    rob.move_controller = -1 if distance < 0 else 1
    move_controller = rob.move_controller

    target_ang_deg = abs(float(target_angle))
    target_dist_cm = abs(float(distance))

    # Saturación física estricta del Qupa (configurable desde el controlador principal)
    max_lin_cm_s = _get_max_linear_cm_s()
    max_ang_rad_limit = _get_max_angular_rad_s()
    max_ang_deg_s = math.degrees(max_ang_rad_limit)
    abs_speed_g_deg_s = min(abs(float(speed_g)), max_ang_deg_s)
    abs_speed_d_cm_s = min(abs(float(speed_d)), max_lin_cm_s)

    max_lin_m_s = abs_speed_d_cm_s / 100.0
    max_ang_rad_s = math.radians(abs_speed_g_deg_s)

    # Lectura dinámica de parámetros IMU desde el controlador principal (fuente única de verdad)
    retare_each    = bool(shared_config.get_controller_var("IMU_RETARE_EACH_SEGMENT", IMU_RETARE_EACH_SEGMENT))
    adaptive_early = bool(shared_config.get_controller_var("IMU_ADAPTIVE_EARLY_STOP", IMU_ADAPTIVE_EARLY_STOP))
    w_slow_cfg     = float(shared_config.get_controller_var("IMU_ANGULAR_SPEED_SLOW_RAD_S", IMU_ANGULAR_SPEED_SLOW_RAD_S))
    slow_zone_deg  = float(shared_config.get_controller_var("IMU_TURN_SLOW_ZONE_DEG", IMU_TURN_SLOW_ZONE_DEG))
    turn_pause_s   = float(shared_config.get_controller_var("IMU_TURN_PAUSE_S", IMU_TURN_PAUSE_S))
    settle_s       = float(shared_config.get_controller_var("IMU_SETTLE_S", IMU_SETTLE_S))
    tare_timeout_s = float(shared_config.get_controller_var("IMU_TARE_TIMEOUT_S", IMU_TARE_TIMEOUT_S))
    timeout_factor = float(shared_config.get_controller_var("IMU_TIMEOUT_FACTOR", IMU_TIMEOUT_FACTOR))

    # Prioridad de calibración lineal:
    # 1) IMU_MEASURED_LINEAR_SPEED_CM_S > 0.0 si un CLI/override explícito lo fijó
    # 2) ROBOT_V_REAL[rob.id] o respaldo v_real definidos al inicio de Functions/move.py
    # 3) 0.0 -> usa directamente speed_d
    ctrl_measured_v = float(shared_config.get_controller_var("IMU_MEASURED_LINEAR_SPEED_CM_S", 0.0))
    if ctrl_measured_v > 0.0:
        measured_v_cm = ctrl_measured_v
    else:
        rob_id = getattr(rob, 'id', None) or getattr(rob, 'name', None) or getattr(rob, 'robot_id', None)
        robot_v = get_v_real(rob_id)
        measured_v_cm = robot_v if robot_v > 0.0 else 0.0

    if not hasattr(rob, 'elapsed_ang_s'):
        rob.elapsed_ang_s = 0.0
    if not hasattr(rob, 'elapsed_dis_s'):
        rob.elapsed_dis_s = 0.0
    if not hasattr(rob, 'actual_ang_ticks'):
        rob.actual_ang_ticks = 0
    if not hasattr(rob, 'actual_dis_ticks'):
        rob.actual_dis_ticks = 0

    now_s = time.monotonic()

    # ===============================
    # Fase 1: Giro en Lazo Cerrado con IMU (MPU6050)
    # ===============================
    if rob.c1 == 0:
        if target_ang_deg < 0.5:
            rob.cmd_linear_x = 0.0
            rob.cmd_angular_z = 0.0
            rob.move_angle = 0.0
            rob.elapsed_ang_s = 0.0
            rob.actual_ang_ticks = 0
            _reset_imu_segment_state(rob)
            rob.c1 = 1
        else:
            if abs_speed_g_deg_s <= 1e-6:
                rob.cmd_linear_x = 0.0
                rob.cmd_angular_z = 0.0
                c1, c2 = rob.c1, rob.c2
                move_distance, move_angle = rob.move_distance, rob.move_angle
                return rob.c2, rob.c1, rob.move_distance, rob.move_angle

            target_rad = math.radians(target_ang_deg)
            ang_sign = 1.0 if target_angle > 0.0 else -1.0
            w_nom = max_ang_rad_s
            w_slow = min(w_nom, max(0.0, w_slow_cfg))
            slow_zone_rad = math.radians(max(0.0, slow_zone_deg))
            expected_turn_s = target_rad / max(w_nom, 1e-3)
            angle_ticks_needed = int(math.ceil(expected_turn_s * tps))

            # --- Subestado INIT ---
            if rob.imu_turn_substate == 'INIT':
                rob.move_distance = 0.0
                rob.move_angle = 0.0
                rob.elapsed_ang_s = 0.0
                rob.actual_ang_ticks = 0
                rob.imu_seg_theta = 0.0
                rob.imu_turn_timeout = False

                if retare_each:
                    rob.imu_integrating_turn = False
                    rob.cmd_linear_x = 0.0
                    rob.cmd_angular_z = 0.0
                    tare_cli = getattr(rob, 'imu_tare_client', None)
                    if tare_cli is not None and _StdTrigger is not None and tare_cli.service_is_ready():
                        rob.imu_tare_future = tare_cli.call_async(_StdTrigger.Request())
                    else:
                        rob.imu_tare_future = None
                    rob.imu_tare_start_t = now_s
                    rob.imu_turn_substate = 'TARE'
                    c1, c2 = rob.c1, rob.c2
                    move_distance, move_angle = rob.move_distance, rob.move_angle
                    return rob.c2, rob.c1, rob.move_distance, rob.move_angle
                else:
                    # Sin re-encerado ni tiempos de espera adicionales: inicia el giro en este mismo tick
                    rob.imu_integrating_turn = True
                    rob.imu_turn_start_t = now_s
                    rob.imu_turn_substate = 'TURN'

            # --- Subestado TARE (solo si IMU_RETARE_EACH_SEGMENT == True) ---
            if rob.imu_turn_substate == 'TARE':
                rob.cmd_linear_x = 0.0
                rob.cmd_angular_z = 0.0
                tare_done = (rob.imu_tare_future is None) or rob.imu_tare_future.done()
                if tare_done or (now_s - rob.imu_tare_start_t) >= tare_timeout_s:
                    rob.imu_tare_future = None
                    rob.imu_settle_start_t = now_s
                    rob.imu_turn_substate = 'SETTLE'
                c1, c2 = rob.c1, rob.c2
                move_distance, move_angle = rob.move_distance, rob.move_angle
                return rob.c2, rob.c1, rob.move_distance, rob.move_angle

            # --- Subestado SETTLE (solo si IMU_RETARE_EACH_SEGMENT == True) ---
            if rob.imu_turn_substate == 'SETTLE':
                rob.cmd_linear_x = 0.0
                rob.cmd_angular_z = 0.0
                if (now_s - rob.imu_settle_start_t) >= max(0.0, settle_s):
                    rob.imu_seg_theta = 0.0
                    rob.imu_integrating_turn = True
                    rob.imu_turn_start_t = now_s
                    rob.imu_turn_substate = 'TURN'
                c1, c2 = rob.c1, rob.c2
                move_distance, move_angle = rob.move_distance, rob.move_angle
                return rob.c2, rob.c1, rob.move_distance, rob.move_angle

            # --- Subestado TURN ---
            if rob.imu_turn_substate == 'TURN':
                rob.move_angle = rob.imu_seg_theta
                early_eff = min(max(rob.imu_early_stop_rad, 0.0), 0.8 * target_rad)
                stop_at = max(0.0, target_rad - early_eff)
                turned_in_dir = rob.imu_seg_theta * ang_sign
                elapsed_turn_s = now_s - rob.imu_turn_start_t

                reached = turned_in_dir >= stop_at
                timeout = elapsed_turn_s >= (timeout_factor * expected_turn_s)

                if reached or timeout:
                    rob.cmd_linear_x = 0.0
                    rob.cmd_angular_z = 0.0
                    rob.imu_turn_timeout = bool(timeout and not reached)

                    if retare_each and turn_pause_s > 0.0:
                        # Mantener imu_integrating_turn=True para medir el sobrepaso de frenado
                        rob.imu_pause_start_t = now_s
                        rob.imu_turn_substate = 'PAUSE_T'
                    else:
                        # Sin espera adicional: cerrar el giro de inmediato
                        rob.imu_integrating_turn = False
                        rob.move_angle = rob.imu_seg_theta
                        if adaptive_early and not rob.imu_turn_timeout:
                            err = abs(rob.imu_seg_theta) - target_rad
                            rob.imu_early_stop_rad = min(max(rob.imu_early_stop_rad + err, 0.0), 0.8 * target_rad)
                        rob.imu_turn_substate = 'INIT'
                        rob.elapsed_ang_s = 0.0
                        rob.actual_ang_ticks = 0
                        rob.c1 = 1
                else:
                    w_cmd_mag = w_slow if max(0.0, turned_in_dir) >= (stop_at - slow_zone_rad) else w_nom
                    rob.cmd_linear_x = 0.0
                    rob.cmd_angular_z = w_cmd_mag * ang_sign
                    rob.elapsed_ang_s = elapsed_turn_s
                    rob.actual_ang_ticks += 1
                    actual_ang_ticks = rob.actual_ang_ticks

                c1, c2 = rob.c1, rob.c2
                move_distance, move_angle = rob.move_distance, rob.move_angle
                return rob.c2, rob.c1, rob.move_distance, rob.move_angle

            # --- Subestado PAUSE_T (solo si IMU_RETARE_EACH_SEGMENT == True) ---
            if rob.imu_turn_substate == 'PAUSE_T':
                rob.cmd_linear_x = 0.0
                rob.cmd_angular_z = 0.0
                rob.move_angle = rob.imu_seg_theta

                if (now_s - rob.imu_pause_start_t) >= turn_pause_s:
                    rob.imu_integrating_turn = False
                    if adaptive_early and not rob.imu_turn_timeout:
                        err = abs(rob.imu_seg_theta) - target_rad
                        rob.imu_early_stop_rad = min(max(rob.imu_early_stop_rad + err, 0.0), 0.8 * target_rad)
                    rob.imu_turn_substate = 'INIT'
                    rob.elapsed_ang_s = 0.0
                    rob.actual_ang_ticks = 0
                    rob.c1 = 1

                c1, c2 = rob.c1, rob.c2
                move_distance, move_angle = rob.move_distance, rob.move_angle
                return rob.c2, rob.c1, rob.move_distance, rob.move_angle

    # ===============================
    # Fase 2: Avance Lineal por Tiempo Calibrable (con yaw real del IMU)
    # ===============================
    if rob.c1 == 1:
        if target_dist_cm < 0.5:
            rob.cmd_linear_x = 0.0
            rob.cmd_angular_z = 0.0
            rob.move_distance = 0.0
            rob.elapsed_dis_s = 0.0
            rob.actual_dis_ticks = 0
            _reset_imu_segment_state(rob)
            rob.c1 = 2
            rob.c2 += 1
            utils_mod.r_Temporizador_2(rob)
        else:
            if abs_speed_d_cm_s <= 1e-6:
                rob.cmd_linear_x = 0.0
                rob.cmd_angular_z = 0.0
                c1, c2 = rob.c1, rob.c2
                move_distance, move_angle = rob.move_distance, rob.move_angle
                return rob.c2, rob.c1, rob.move_distance, rob.move_angle

            v_real_cm_s = measured_v_cm if measured_v_cm > 0.0 else abs_speed_d_cm_s
            dis_time_needed = target_dist_cm / v_real_cm_s
            time_needed = dis_time_needed
            ticks_needed = int(math.ceil(dis_time_needed * tps))

            if rob.elapsed_dis_s + 1e-9 >= dis_time_needed:
                rob.cmd_linear_x = 0.0
                rob.cmd_angular_z = 0.0
                rob.move_distance = float(distance)
                rob.elapsed_dis_s = 0.0
                rob.actual_dis_ticks = 0
                _reset_imu_segment_state(rob)
                rob.c1 = 2
                rob.c2 += 1
                utils_mod.r_Temporizador_2(rob)
            else:
                rem_time_s = dis_time_needed - rob.elapsed_dis_s
                v_cmd = max_lin_m_s * float(rob.move_controller)
                w_corr = 0.0
                straight_enabled = bool(
                    shared_config.get_controller_var("ENABLE_IMU_STRAIGHT_HOLD", ENABLE_IMU_STRAIGHT_HOLD)
                )
                if straight_enabled and bool(getattr(rob, 'imu_received', False)):
                    if getattr(rob, 'imu_straight_ref_yaw', None) is None:
                        rob.imu_straight_ref_yaw = float(rob.imu_yaw)
                    err_yaw_rad = _norm_rad(rob.imu_straight_ref_yaw - float(rob.imu_yaw))
                    db_rad = math.radians(
                        max(0.0, float(shared_config.get_controller_var("IMU_STRAIGHT_DEADBAND_DEG", IMU_STRAIGHT_DEADBAND_DEG)))
                    )
                    if abs(err_yaw_rad) > db_rad:
                        kp_straight = float(shared_config.get_controller_var("IMU_STRAIGHT_KP", IMU_STRAIGHT_KP))
                        max_w_straight = abs(
                            float(shared_config.get_controller_var("IMU_STRAIGHT_MAX_W_RAD_S", IMU_STRAIGHT_MAX_W_RAD_S))
                        )
                        w_corr = max(-max_w_straight, min(max_w_straight, kp_straight * err_yaw_rad))
                rob.cmd_linear_x = v_cmd
                rob.cmd_angular_z = w_corr

                step_dt = min(dt, rem_time_s)
                # Integra (rob.x, rob.y) con la velocidad real y el yaw real del giroscopio
                if not getattr(rob, 'odom_received', False) and step_dt > 0.0:
                    v_real_m_s = (v_real_cm_s / 100.0) * float(rob.move_controller)
                    rob.x = float(getattr(rob, 'x', 0.0)) + v_real_m_s * math.cos(rob.yaw) * step_dt
                    rob.y = float(getattr(rob, 'y', 0.0)) + v_real_m_s * math.sin(rob.yaw) * step_dt

                rob.elapsed_dis_s += dt
                rob.actual_dis_ticks += 1
                actual_dis_ticks = rob.actual_dis_ticks

                traveled_cm = min(target_dist_cm, rob.elapsed_dis_s * v_real_cm_s)
                rob.move_distance = traveled_cm * float(rob.move_controller)

                if rob.elapsed_dis_s + 1e-9 >= dis_time_needed:
                    rob.move_distance = float(distance)

    c1, c2 = rob.c1, rob.c2
    move_distance, move_angle = rob.move_distance, rob.move_angle
    return rob.c2, rob.c1, rob.move_distance, rob.move_angle


def r_move(rob):
    global c1, c2, move_controller, move_distance, move_angle
    global ticks_needed, actual_dis_ticks, angle_ticks_needed, actual_ang_ticks
    global time_needed, wheel_speed, omega_rad, previous_speed_d
    global full_ticks_a, has_rem_a, rem_wheel_speed, rem_omega_rad
    global rem_speed_d, full_ticks_d, has_rem_d

    c1                  = 0
    c2                  = 0
    move_controller     = 1
    move_distance       = 0.0
    ticks_needed        = 0
    actual_dis_ticks    = 0
    angle_ticks_needed  = 0
    actual_ang_ticks    = 0
    time_needed         = 0
    move_angle          = 0.0
    wheel_speed         = 0.0
    omega_rad           = 0.0
    previous_speed_d    = None
    full_ticks_a        = 0
    has_rem_a           = False
    rem_wheel_speed     = 0.0
    rem_omega_rad       = 0.0
    rem_speed_d         = 0.0
    full_ticks_d        = 0
    has_rem_d           = False

    rob.c1 = 0
    rob.c2 = 0
    rob.move_controller = 1
    rob.move_distance = 0.0
    rob.move_angle = 0.0
    rob.elapsed_ang_s = 0.0
    rob.elapsed_dis_s = 0.0
    rob.actual_ang_ticks = 0
    rob.actual_dis_ticks = 0
    rob.seg_start_x = None
    rob.seg_start_y = None
    rob.seg_start_yaw = None
    rob.seg_target_yaw = None
    _ensure_imu_state(rob)
    _reset_imu_segment_state(rob)


# ============================================================================
# 3. RANDOM WALKING
# ============================================================================
RW_THRESHOLD_FRONT  = 0.4  # Sensibilidad cono frontal (-45°, 0°, 45°)
RW_THRESHOLD_SIDE   = 0.8  # Sensibilidad lateral (-90°, 90°)
RW_THRESHOLD_CLEAR  = 0.3  # Umbral para considerar que el camino vuelve a estar libre

RW_b1               = 0
Rw_target_angle     = 0.0
RW_stuck_counter    = 0
RW_clear_path_timer = 0


def RandomWalking(rob, timer: int = 1):
    """Equivalente 1:1 de RandomWalking() en Functions/move.lua, respetando d_speed y g_speed del controlador."""
    global RW_b1, Rw_target_angle, RW_stuck_counter, RW_clear_path_timer
    datos_prox = sensors_mod.proximidad(rob, True)

    rw_d_speed = float(shared_config.get_controller_var("d_speed", getattr(rob, "d_speed", 30.0)))
    rw_g_speed = float(shared_config.get_controller_var("g_speed", getattr(rob, "g_speed", 80.0)))
    interwheel_dist = float(
        getattr(
            rob,
            'interwheel_distance',
            shared_config.get_controller_var("FOOTBOT_INTERWHEEL_DISTANCE", 14.0),
        )
    )

    if rob.RW_b1 == 0:
        rob.RW_clear_path_timer += 1

        if rob.RW_clear_path_timer > 30 and datos_prox['max_val']['valor'] < RW_THRESHOLD_CLEAR:
            rob.RW_stuck_counter = 0

        max_val = datos_prox['max_val']['valor']
        max_ang = datos_prox['max_val']['angulo']

        trigger_evasion = False

        if abs(max_ang) <= 45.0:
            if max_val > RW_THRESHOLD_FRONT:
                trigger_evasion = True
        elif abs(max_ang) == 90.0:
            if max_val > RW_THRESHOLD_SIDE:
                trigger_evasion = True

        if trigger_evasion:
            set_wheels_cm_s(rob, 0.0, 0.0, interwheel_dist)

            rob.RW_clear_path_timer = 0
            rob.RW_stuck_counter += 1

            if rob.RW_stuck_counter >= 3:
                ang = float(random.randint(160, 180))
                rob.Rw_target_angle = -ang if max_ang < 0 else ang
                rob.RW_stuck_counter = 0
            else:
                if max_ang == 0.0:
                    rob.Rw_target_angle = float(
                        random.randint(90, 160) if random.randint(0, 1) == 0 else random.randint(-160, -90)
                    )
                elif max_ang == 45.0:
                    rob.Rw_target_angle = float(random.randint(-150, -90))
                elif max_ang == -45.0:
                    rob.Rw_target_angle = float(random.randint(90, 150))
                elif max_ang == 90.0:
                    rob.Rw_target_angle = float(random.randint(-90, -45))
                elif max_ang == -90.0:
                    rob.Rw_target_angle = float(random.randint(45, 90))
                else:
                    rob.Rw_target_angle = float(random.randint(90, 180))

            r_move(rob)
            rob.RW_b1 = 1
        else:
            set_wheels_cm_s(rob, rw_d_speed, rw_d_speed, interwheel_dist)

    elif rob.RW_b1 == 1:
        move(rob, 0.0, rob.Rw_target_angle, rw_d_speed, rw_g_speed, timer)

        if rob.c1 == 2:
            rob.RW_b1 = 0
            rob.Rw_target_angle = 0.0
            r_move(rob)

    RW_b1 = rob.RW_b1
    Rw_target_angle = rob.Rw_target_angle
    RW_stuck_counter = rob.RW_stuck_counter
    RW_clear_path_timer = rob.RW_clear_path_timer


random_walking = RandomWalking


# ============================================================================
# 4. NAVEGACIÓN A COORDENADAS Y EVASIÓN (navegar_a_coordenadas)
# ============================================================================
UMBRAL_INTENSIDAD_EVASION = 0.1
DURACION_TICKS_ARCO       = 5 * 4

estado_evasion = {
    'modo'          : "NORMAL",
    'ultimo_modo'   : "",
    'angulo_giro'   : 90.0,
    'tiempo_arco'   : 0,
    'alternar_giro' : 1,
}


def sign(x: float) -> int:
    if x > 0:
        return 1
    elif x < 0:
        return -1
    else:
        return 0


def navegar_a_coordenadas(
    rob,
    target_x: float,
    target_y: float,
    stop_distance: float,
    timer: int = 1,
    ticks_per_second: float = 4.0,
    debug_evasion: int = 0,
    log_fn=print,
) -> bool:
    """Equivalente 1:1 de navegar_a_coordenadas en Functions/move.lua."""
    duracion_ticks_arco = int(5 * ticks_per_second)
    interwheel_dist = getattr(rob, 'interwheel_distance', 14.0)
    ctrl_d_speed = float(shared_config.get_controller_var("d_speed", getattr(rob, "d_speed", 30.0)))
    ctrl_g_speed = float(shared_config.get_controller_var("g_speed", getattr(rob, "g_speed", 80.0)))

    current_x = rob.x
    current_y = rob.y
    dx = target_x - current_x
    dy = target_y - current_y
    distance = math.hypot(dx, dy)

    if distance <= stop_distance:
        set_wheels_cm_s(rob, 0.0, 0.0, interwheel_dist)
        rob.estado_evasion['modo'] = "NORMAL"
        return True

    if abs(ctrl_d_speed) <= 1e-6:
        set_wheels_cm_s(rob, 0.0, 0.0, interwheel_dist)
        return False

    angle_to_center = math.atan2(dy, dx)
    angle_difference = _norm_rad(angle_to_center - rob.yaw)

    datos_prox = sensors_mod.proximidad(rob, False)
    prox_val = datos_prox[0]
    prox_angle = datos_prox[1]

    obstaculo_frontal = False
    if prox_val > UMBRAL_INTENSIDAD_EVASION and abs(prox_angle) <= 90.0:
        obstaculo_frontal = True

    if debug_evasion > 0:
        if rob.estado_evasion['modo'] != rob.estado_evasion['ultimo_modo']:
            if rob.estado_evasion['modo'] == "NORMAL":
                log_fn(f"[{rob.id}] AVANZANDO A COORDENADA: {target_x:.2f}, {target_y:.2f}")
            elif rob.estado_evasion['modo'] == "GIRANDO":
                log_fn(f"[{rob.id}] GIRANDO {int(rob.estado_evasion['angulo_giro'])}° para evadir")
            elif rob.estado_evasion['modo'] == "ARCO":
                log_fn(f"[{rob.id}] ESQUIVANDO (Haciendo Arco)")
            rob.estado_evasion['ultimo_modo'] = rob.estado_evasion['modo']

        if debug_evasion >= 2:
            log_fn(
                f"[{rob.id}] MODO: {rob.estado_evasion['modo']} | "
                f"ProxPromedio: {prox_val:.3f} a {prox_angle:.1f}° | "
                f"AngMeta: {math.degrees(angle_difference):.1f}°"
            )

    if rob.estado_evasion['modo'] == "NORMAL":
        if obstaculo_frontal:
            if prox_angle > 5.0:
                rob.estado_evasion['angulo_giro'] = -90.0
            elif prox_angle < -5.0:
                rob.estado_evasion['angulo_giro'] = 90.0
            else:
                rob.estado_evasion['angulo_giro'] = 90.0 * rob.estado_evasion['alternar_giro']
                rob.estado_evasion['alternar_giro'] *= -1
            r_move(rob)
            rob.estado_evasion['modo'] = "GIRANDO"

    elif rob.estado_evasion['modo'] == "GIRANDO":
        _, rc1, _, _ = move(rob, 0.0, rob.estado_evasion['angulo_giro'], 0.0, ctrl_g_speed, timer, False)
        if rc1 == 2:
            rob.estado_evasion['modo'] = "ARCO"
            rob.estado_evasion['tiempo_arco'] = duracion_ticks_arco
        return False

    elif rob.estado_evasion['modo'] == "ARCO":
        if obstaculo_frontal:
            if prox_angle > 5.0:
                rob.estado_evasion['angulo_giro'] = -90.0
            elif prox_angle < -5.0:
                rob.estado_evasion['angulo_giro'] = 90.0
            else:
                rob.estado_evasion['angulo_giro'] = 90.0 * rob.estado_evasion['alternar_giro']
                rob.estado_evasion['alternar_giro'] *= -1
            r_move(rob)
            rob.estado_evasion['modo'] = "GIRANDO"
            return False
        elif abs(angle_difference) < 0.35 and not obstaculo_frontal:
            rob.estado_evasion['modo'] = "NORMAL"
        else:
            rob.estado_evasion['tiempo_arco'] -= 1
            if rob.estado_evasion['tiempo_arco'] <= 0:
                rob.estado_evasion['modo'] = "NORMAL"

    if rob.estado_evasion['modo'] == "ARCO":
        velocidad_base = ctrl_d_speed
        diferencial = 0.5 * ctrl_d_speed
        if rob.estado_evasion['angulo_giro'] > 0:
            set_wheels_cm_s(rob, velocidad_base + diferencial, velocidad_base - diferencial, interwheel_dist)
        else:
            set_wheels_cm_s(rob, velocidad_base - diferencial, velocidad_base + diferencial, interwheel_dist)
        return False

    elif rob.estado_evasion['modo'] == "NORMAL":
        speed_straight = ctrl_d_speed
        speed_turn_factor = 10.0 * float(ticks_per_second)
        max_wheel_speed = max(abs(ctrl_d_speed), abs(ctrl_g_speed))

        if abs(angle_difference) < 0.1:
            set_wheels_cm_s(rob, speed_straight, speed_straight, interwheel_dist)
        else:
            turn_speed = speed_turn_factor * angle_difference
            v_l = -turn_speed
            v_r = turn_speed
            if abs(v_l) > max_wheel_speed:
                v_l = max_wheel_speed * sign(v_l)
            if abs(v_r) > max_wheel_speed:
                v_r = max_wheel_speed * sign(v_r)
            set_wheels_cm_s(rob, v_l, v_r, interwheel_dist)

    return False


class Move:
    Save_position         = staticmethod(Save_position)
    redondear_resolucion  = staticmethod(redondear_resolucion)
    move                  = staticmethod(move)
    r_move                = staticmethod(r_move)
    RandomWalking         = staticmethod(RandomWalking)
    random_walking        = staticmethod(random_walking)
    sign                  = staticmethod(sign)
    navegar_a_coordenadas = staticmethod(navegar_a_coordenadas)
    set_wheels_cm_s       = staticmethod(set_wheels_cm_s)
    clamp_cmd_vel         = staticmethod(clamp_cmd_vel)
    update_imu            = staticmethod(update_imu)