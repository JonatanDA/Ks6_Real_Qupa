# Archivo: Ks6_Real_Qupa/Functions/move_odometria_global.py  [CASO 2: LAZO CERRADO CON /{robot_id}/odom - Qupa Real]
# Ubicación: ~/sim_ws_qupa/Ks6/Functions/move.py
# Descripción:
#   Módulo compartido de movimiento y navegación para los Qupa en ROS 2 (Simulación y Hardware Real):
#   move, r_move, redondear_resolucion, RandomWalking, navegar_a_coordenadas y Save_position.
#   Ejecuta el control de movimiento en lazo abierto por velocidad enviada y tiempo transcurrido,
#   e integra cinemáticamente (x, y, yaw) cuando el robot opera sin tópico /odom (Qupa real).
#   Respeta fielmente las velocidades speed_d, speed_g, d_speed y g_speed definidas en el controlador.
#   Equivalente 1:1 de Ks6/Functions/move.lua.

import math
import random

from Functions import shared_config
from Functions import sensors as sensors_mod
from Functions import utils as utils_mod



# ============================================================================
# LÍMITES FÍSICOS DE SATURACIÓN DEL ROBOT QUPA (GAZEBO Y REAL)
# ============================================================================
MAX_LINEAR_SPEED_CM_S   = 8.0                   # 8.0 cm/s = 0.08 m/s
MAX_LINEAR_SPEED_M_S    = 0.08                  # m/s
MAX_ANGULAR_SPEED_RAD_S = 0.776                 # rad/s
MAX_ANGULAR_SPEED_DEG_S = math.degrees(0.776)   # ~44.4615 °/s


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


def _integrate_open_loop_pose(rob, dt: float):
    """
    Integra la pose (x, y, yaw) en lazo abierto a partir de las velocidades comandadas
    (cmd_linear_x, cmd_angular_z) y el paso de tiempo dt cuando no existe tópico /odom
    (robots físicos Qupa sin encoders).
    """
    if getattr(rob, 'odom_received', False):
        return
    if dt <= 0.0:
        return
    v = float(getattr(rob, 'cmd_linear_x', 0.0))
    w = float(getattr(rob, 'cmd_angular_z', 0.0))
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


# Parámetros de lazo cerrado para Qupa Real (respetan deadband de 0.03 m/s por rueda en motor_driver_node)
ODOM_ANG_TOL_DEG     = 3.0    # Tolerancia de llegada angular (°)
ODOM_DIST_TOL_CM     = 1.0    # Tolerancia de llegada lineal (cm)
ODOM_MIN_LIN_M_S     = 0.035  # Piso mínimo lineal (> 0.03 m/s deadband físico)
ODOM_MIN_ANG_RAD_S   = 0.72   # Piso mínimo angular (> 0.70 rad/s para 0.03 m/s por rueda con wheel_base=0.086 m)


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
    Controlador en lazo cerrado apoyado en el tópico /{robot_id}/odom (nav_msgs/msg/Odometry,
    misma estructura que Gazebo) para el robot físico Qupa.
    Fase 1 (c1 == 0): Gira usando rob.yaw hasta alcanzar target_angle y reporta el giro real en rob.move_angle (rad).
    Fase 2 (c1 == 1): Avanza usando (rob.x, rob.y) hasta alcanzar distance y reporta el avance real en rob.move_distance (cm).
    Si /{robot_id}/odom aún no publica datos (odom_received == False), detiene motores y avisa sin caer en lazo abierto.
    """
    global c1, c2, move_controller, move_distance, move_angle
    global ticks_needed, actual_dis_ticks, angle_ticks_needed, actual_ang_ticks
    global time_needed, wheel_speed, omega_rad

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

    # Guardrail: en modo odometría global, exigir que /{robot_id}/odom esté publicando
    if not getattr(rob, 'odom_received', False):
        rob.cmd_linear_x = 0.0
        rob.cmd_angular_z = 0.0
        if not getattr(rob, '_odom_wait_warned', False):
            rob._odom_wait_warned = True
            shared_config.log_py(
                f"[MOVE_ODOM] Esperando mensajes en /{rob.id}/odom (nav_msgs/msg/Odometry)..."
            )
        c1, c2 = rob.c1, rob.c2
        move_distance, move_angle = rob.move_distance, rob.move_angle
        return rob.c2, rob.c1, rob.move_distance, rob.move_angle
    rob._odom_wait_warned = False

    while target_angle > 180.0:
        target_angle -= 360.0
    while target_angle <= -180.0:
        target_angle += 360.0

    rob.move_controller = -1 if distance < 0 else 1
    move_controller = rob.move_controller

    target_ang_deg = abs(float(target_angle))
    target_dist_cm = abs(float(distance))

    # Saturación física estricta del Qupa (8.0 cm/s lineal y 0.776 rad/s ≈ 44.46 °/s angular)
    max_lin_cm_s = _get_max_linear_cm_s()
    max_ang_deg_s = math.degrees(_get_max_angular_rad_s())
    abs_speed_g_deg_s = min(abs(float(speed_g)), max_ang_deg_s)
    abs_speed_d_cm_s = min(abs(float(speed_d)), max_lin_cm_s)

    max_lin_m_s = abs_speed_d_cm_s / 100.0
    max_ang_rad_s = math.radians(abs_speed_g_deg_s)

    ang_tol_rad = math.radians(ODOM_ANG_TOL_DEG)
    dist_tol_cm = float(ODOM_DIST_TOL_CM)
    min_ang_rad_s = min(ODOM_MIN_ANG_RAD_S, max_ang_rad_s) if max_ang_rad_s > 0.0 else 0.0
    min_lin_m_s = min(ODOM_MIN_LIN_M_S, max_lin_m_s) if max_lin_m_s > 0.0 else 0.0

    if not hasattr(rob, 'seg_prev_yaw'):
        rob.seg_prev_yaw = None
    if not hasattr(rob, 'seg_turned_rad'):
        rob.seg_turned_rad = 0.0
    if not hasattr(rob, 'seg_heading_yaw'):
        rob.seg_heading_yaw = None

    cur_x = float(rob.x)
    cur_y = float(rob.y)
    cur_yaw = float(rob.yaw)

    # ===============================
    # Fase 1: Giro en Lazo Cerrado (/odom)
    # ===============================
    if rob.c1 == 0:
        if target_ang_deg < 0.5:
            rob.cmd_linear_x = 0.0
            rob.cmd_angular_z = 0.0
            rob.move_angle = 0.0
            rob.seg_turned_rad = 0.0
            rob.seg_start_x = cur_x
            rob.seg_start_y = cur_y
            rob.seg_heading_yaw = cur_yaw
            rob.c1 = 1
        else:
            if abs_speed_g_deg_s <= 1e-6:
                rob.cmd_linear_x = 0.0
                rob.cmd_angular_z = 0.0
                c1, c2 = rob.c1, rob.c2
                move_distance, move_angle = rob.move_distance, rob.move_angle
                return rob.c2, rob.c1, rob.move_distance, rob.move_angle

            if rob.seg_start_yaw is None:
                rob.seg_start_yaw = cur_yaw
                rob.seg_target_yaw = _norm_rad(cur_yaw + math.radians(target_angle))
                rob.seg_prev_yaw = cur_yaw
                rob.seg_turned_rad = 0.0
                rob.move_distance = 0.0
            else:
                delta_yaw = _norm_rad(cur_yaw - rob.seg_prev_yaw)
                rob.seg_turned_rad += delta_yaw
                rob.seg_prev_yaw = cur_yaw

            rob.move_angle = rob.seg_turned_rad
            err_rad = _norm_rad(rob.seg_target_yaw - cur_yaw)
            ang_sign = 1.0 if target_angle > 0.0 else -1.0
            reached_or_crossed = (
                abs(err_rad) <= ang_tol_rad
                or (rob.seg_turned_rad * ang_sign) >= (math.radians(target_ang_deg) - ang_tol_rad)
            )

            if reached_or_crossed:
                rob.cmd_linear_x = 0.0
                rob.cmd_angular_z = 0.0
                rob.seg_start_x = cur_x
                rob.seg_start_y = cur_y
                rob.seg_heading_yaw = cur_yaw
                rob.c1 = 1
            else:
                omega_mag = min(max_ang_rad_s, abs(err_rad) / max(dt, 1e-3))
                omega_mag = max(min_ang_rad_s, omega_mag)
                omega_cmd = omega_mag if err_rad > 0.0 else -omega_mag
                rob.cmd_linear_x = 0.0
                rob.cmd_angular_z = omega_cmd

                c1, c2 = rob.c1, rob.c2
                move_distance, move_angle = rob.move_distance, rob.move_angle
                return rob.c2, rob.c1, rob.move_distance, rob.move_angle

    # ===============================
    # Fase 2: Avance Lineal en Lazo Cerrado (/odom)
    # ===============================
    if rob.c1 == 1:
        if target_dist_cm < 0.5:
            rob.cmd_linear_x = 0.0
            rob.cmd_angular_z = 0.0
            rob.move_distance = 0.0
            rob.seg_start_x = None
            rob.seg_start_y = None
            rob.seg_start_yaw = None
            rob.seg_target_yaw = None
            rob.seg_prev_yaw = None
            rob.seg_heading_yaw = None
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

            if rob.seg_start_x is None or rob.seg_start_y is None or rob.seg_heading_yaw is None:
                rob.seg_start_x = cur_x
                rob.seg_start_y = cur_y
                rob.seg_heading_yaw = cur_yaw

            dx_m = cur_x - rob.seg_start_x
            dy_m = cur_y - rob.seg_start_y
            proj_cm = (
                dx_m * math.cos(rob.seg_heading_yaw) + dy_m * math.sin(rob.seg_heading_yaw)
            ) * 100.0
            rob.move_distance = proj_cm

            traveled_along_cm = proj_cm * float(rob.move_controller)
            rem_cm = target_dist_cm - traveled_along_cm

            if rem_cm <= dist_tol_cm:
                rob.cmd_linear_x = 0.0
                rob.cmd_angular_z = 0.0
                rob.seg_start_x = None
                rob.seg_start_y = None
                rob.seg_start_yaw = None
                rob.seg_target_yaw = None
                rob.seg_prev_yaw = None
                rob.seg_heading_yaw = None
                rob.c1 = 2
                rob.c2 += 1
                utils_mod.r_Temporizador_2(rob)
            else:
                v_mag = min(max_lin_m_s, (rem_cm / 100.0) / max(dt, 1e-3))
                v_mag = max(min_lin_m_s, v_mag)
                rob.cmd_linear_x = v_mag * float(rob.move_controller)
                rob.cmd_angular_z = 0.0

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
    rob.seg_prev_yaw = None
    rob.seg_turned_rad = 0.0
    rob.seg_heading_yaw = None


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