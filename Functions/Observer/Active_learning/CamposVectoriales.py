# ============================================================================
# VARIABLES DEL MÓDULO: CAMPOS VECTORIALES (FUERZAS FÍSICAS)
# ============================================================================
# AL_LOG_MODE              = 3
AL_SMOOTH_TICKS          = 3
AL_SMOOTH_METHOD         = "median"

M_MAX_DIST               = 140.0
PUSH_LIMIT               = 30.0
NEUTRAL_ZONE             = 5.0
N_CRIT_DIST              = 15.0
K_FORCE                  = 1.2   # Solo escala la fuerza de REPULSIÓN (d <= PUSH_LIMIT - NEUTRAL_ZONE)
MIN_FORCE_IGNORE         = 10.0

VECTOR_CHANGE_TOL        = 10.0
PAUSE_TICKS_ON_CHANGE    = 2

# Umbrales de Supervivencia (valores de respaldo si no se definen en el controlador)
FRONT_THRESHOLD          = 25.0
BACK_THRESHOLD           = 10.0
PROX_CRITICAL_THRESHOLD  = 0.90

import math
import sys

sys.modules["Functions.Observer.Active_learning.CamposVectoriales"] = sys.modules[__name__]

from Controller import apprentice_process
from Functions import math_helper

history_x = []
history_y = []
last_vector_mag = 0.0


def _get_ctrl_num(attr_name: str, default_val: float) -> float:
    val = getattr(apprentice_process, attr_name, None)
    return float(val) if val is not None else float(default_val)


def get_smoothed_values(new_x: float, new_y: float):
    history_x.append(new_x)
    history_y.append(new_y)
    while len(history_x) > AL_SMOOTH_TICKS:
        history_x.pop(0)
        history_y.pop(0)

    if AL_SMOOTH_TICKS == 1:
        return new_x, new_y

    if AL_SMOOTH_METHOD == "mean":
        return sum(history_x) / len(history_x), sum(history_y) / len(history_y)
    else:
        tx = sorted(history_x)
        ty = sorted(history_y)
        n = len(tx)
        mid = n // 2
        if n % 2 == 0:
            return (tx[mid - 1] + tx[mid]) / 2.0, (ty[mid - 1] + ty[mid]) / 2.0
        else:
            return tx[mid], ty[mid]


def process(obs_ang_offset_global: float, updated_anterior_indices: dict, app_x_m: float = 0.0, app_y_m: float = 0.0):
    global last_vector_mag

    active_count = 0
    sum_fx, sum_fy = 0.0, 0.0
    closest_master_dist = 999999.0
    angle_of_closest = 0.0

    for k, data in apprentice_process.srt_anterior_data.items():
        if updated_anterior_indices.get(k) and data.get('angle') is not None and data.get('distance') is not None:
            active_count += 1
            d = float(data['distance'])
            ang_rad = float(data['angle']) * (math.pi / 180.0)

            force_mag = 0.0
            if d >= (PUSH_LIMIT + NEUTRAL_ZONE):
                effective_d = min(d, M_MAX_DIST)
                f_ratio = (effective_d - (PUSH_LIMIT + NEUTRAL_ZONE)) / (
                    M_MAX_DIST - (PUSH_LIMIT + NEUTRAL_ZONE)
                )
                force_mag = f_ratio * 100.0
            elif d <= (PUSH_LIMIT - NEUTRAL_ZONE):
                effective_d = max(d, N_CRIT_DIST)
                f_ratio = ((PUSH_LIMIT - NEUTRAL_ZONE) - effective_d) / (
                    (PUSH_LIMIT - NEUTRAL_ZONE) - N_CRIT_DIST
                )
                force_mag = -(f_ratio * 100.0 * K_FORCE)

            sum_fx += force_mag * math.cos(ang_rad)
            sum_fy += force_mag * math.sin(ang_rad)

            if d < closest_master_dist:
                closest_master_dist = d
                angle_of_closest = float(data['angle'])

    enable_active_learning = getattr(apprentice_process, 'ENABLE_ACTIVE_LEARNING', True)

    if active_count > 0 and enable_active_learning:
        smooth_fx, smooth_fy = get_smoothed_values(sum_fx, sum_fy)
        resultant_force, resultant_angle_rad = math_helper.cartesianoAPolarPAR(smooth_fx, smooth_fy)
        resultant_angle = resultant_angle_rad * (180.0 / math.pi)

        tracker_tray_fn = getattr(apprentice_process, 'Print_tracker_trayectory_data', None)
        if callable(tracker_tray_fn):
            ang_total_res = resultant_angle_rad + obs_ang_offset_global
            res_gx, res_gy = math_helper.polarACartesianoPAR(resultant_force, ang_total_res)
            tracker_tray_fn("Active_learning", 4, "Qupa4", app_x_m, app_y_m, res_gx / 100.0, res_gy / 100.0)

        should_move = abs(resultant_force) > MIN_FORCE_IGNORE

        trigger_pause = False
        if abs(resultant_force - last_vector_mag) > VECTOR_CHANGE_TOL:
            trigger_pause = True
        last_vector_mag = resultant_force

        # Usa directamente d_speed y g_speed del controlador del aprendiz sin recortarlos por resultant_force/angle
        d_spd = _get_ctrl_num('d_speed', 8.0)
        g_spd = _get_ctrl_num('g_speed', 44.46)
        front_thr = _get_ctrl_num('front_threshold', FRONT_THRESHOLD)
        back_thr = _get_ctrl_num('back_threshold', BACK_THRESHOLD)

        dyn_speed_d = d_spd if d_spd > 0.0 else 0.0
        dyn_speed_g = g_spd if g_spd > 0.0 else 0.0

        if getattr(apprentice_process, 'AL_LOG_MODE', 0) == 2:
            apprentice_process.log_py(f"[ACTIVE LEARNING] Campos | Mag: {resultant_force:.1f}")

        return {
            'should_move'     : should_move,
            'pause_ticks'     : PAUSE_TICKS_ON_CHANGE if trigger_pause else 0,
            'target_x'        : smooth_fx,
            'target_y'        : smooth_fy,
            'target_dist'     : resultant_force,
            'target_angle'    : resultant_angle,
            'speed_d'         : dyn_speed_d,
            'speed_g'         : dyn_speed_g,
            'closest_dist'    : closest_master_dist,
            'closest_angle'   : angle_of_closest,
            'front_threshold' : front_thr,
            'back_threshold'  : back_thr,
            'prox_threshold'  : PROX_CRITICAL_THRESHOLD,
            'state_name'      : "SIGUIENDO_FUERZAS",
            'state_msg'       : f"FUERZA | Mag: {resultant_force:.1f} | Ang: {resultant_angle:.1f}°",
            'is_centroid'     : False,
        }
    else:
        history_x.clear()
        history_y.clear()
        return None


def reset_history():
    global last_vector_mag
    history_x.clear()
    history_y.clear()
    last_vector_mag = 0.0


class CamposVectoriales:
    AL_SMOOTH_TICKS = AL_SMOOTH_TICKS
    AL_SMOOTH_METHOD = AL_SMOOTH_METHOD
    M_MAX_DIST = M_MAX_DIST
    PUSH_LIMIT = PUSH_LIMIT
    NEUTRAL_ZONE = NEUTRAL_ZONE
    N_CRIT_DIST = N_CRIT_DIST
    K_FORCE = K_FORCE
    MIN_FORCE_IGNORE = MIN_FORCE_IGNORE
    VECTOR_CHANGE_TOL = VECTOR_CHANGE_TOL
    PAUSE_TICKS_ON_CHANGE = PAUSE_TICKS_ON_CHANGE
    FRONT_THRESHOLD = FRONT_THRESHOLD
    BACK_THRESHOLD = BACK_THRESHOLD
    PROX_CRITICAL_THRESHOLD = PROX_CRITICAL_THRESHOLD

    def __init__(self):
        pass

    def process(self, obs_ang_offset_global: float, updated_anterior_indices: dict, app_x_m: float = 0.0, app_y_m: float = 0.0):
        return process(obs_ang_offset_global, updated_anterior_indices, app_x_m, app_y_m)

    @staticmethod
    def reset_history():
        return reset_history()


ActiveLearning = CamposVectoriales
