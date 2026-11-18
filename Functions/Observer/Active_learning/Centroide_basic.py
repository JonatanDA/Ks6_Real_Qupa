# ============================================================================
# VARIABLES DEL MÓDULO: CENTROIDE BÁSICO
# ============================================================================
# AL_LOG_MODE              = 2         # 0: Sin logs, 1: Info, 2: Info Detallada, 3: Estados
AL_SMOOTH_TICKS          = 3         # Cantidad de ticks para suavizar la señal.
AL_SMOOTH_METHOD         = "median"  # Método matemático: "median" o "mean".
TARGET_REACH_TOLERANCE   = 15.0      # (cm) Radio del objetivo. Si el nuevo centroide cae aquí, mantenemos la meta actual.
PAUSE_TICKS_ON_CHANGE    = 1         # Ticks de pausa al cambiar de meta (Máx: AL_SMOOTH_TICKS - 1).

# Umbrales de Supervivencia
FRONT_THRESHOLD          = 25.0      # (cm) Distancia crítica visual (Cámara).
BACK_THRESHOLD           = 10.0      # (cm) Distancia a retroceder o avanzar en escapes.
PROX_CRITICAL_THRESHOLD  = 0.90      # (0 a 1) Intensidad crítica para el sensor infrarrojo.

import math
from Controller import apprentice_process
from Functions import math_helper

history_x = []
history_y = []
current_target_x = None
current_target_y = None
last_recalc_trigger = False


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
        mid = len(tx) // 2
        if len(tx) % 2 == 0:
            return (tx[mid - 1] + tx[mid]) / 2.0, (ty[mid - 1] + ty[mid]) / 2.0
        else:
            return tx[mid], ty[mid]


def process(obs_ang_offset_global: float, updated_anterior_indices: dict, app_x_m: float = 0.0, app_y_m: float = 0.0):
    global current_target_x, current_target_y, last_recalc_trigger

    active_count = 0
    sum_x, sum_y = 0.0, 0.0
    closest_master_dist = 999999.0
    angle_of_closest = 0.0

    for k, data in apprentice_process.srt_anterior_data.items():
        if updated_anterior_indices.get(k) and data.get('angle') is not None and data.get('distance') is not None:
            mx, my = math_helper.polarACartesianoPAR(float(data['distance']), float(data['angle']) * (math.pi / 180.0))
            sum_x += mx
            sum_y += my
            active_count += 1
            if float(data['distance']) < closest_master_dist:
                closest_master_dist = float(data['distance'])
                angle_of_closest = float(data['angle'])

    enable_active_learning = getattr(apprentice_process, 'ENABLE_ACTIVE_LEARNING', True)

    if active_count > 0 and enable_active_learning:
        if active_count == 1:
            active_count = 2

        raw_centroid_x = sum_x / active_count
        raw_centroid_y = sum_y / active_count
        smooth_x, smooth_y = get_smoothed_values(raw_centroid_x, raw_centroid_y)

        recalculate = True
        if current_target_x is not None and current_target_y is not None:
            dist_to_old = math.hypot(smooth_x - current_target_x, smooth_y - current_target_y)
            if dist_to_old <= TARGET_REACH_TOLERANCE:
                recalculate = False
                smooth_x = current_target_x
                smooth_y = current_target_y

        if recalculate:
            current_target_x = smooth_x
            current_target_y = smooth_y

        trigger_pause = (recalculate and not last_recalc_trigger)
        last_recalc_trigger = recalculate

        centroid_dist, centroid_angle_rad = math_helper.cartesianoAPolarPAR(smooth_x, smooth_y)
        centroid_angle = centroid_angle_rad * (180.0 / math.pi)

        tracker_tray_fn = getattr(apprentice_process, 'Print_tracker_trayectory_data', None)
        if callable(tracker_tray_fn):
            ang_total_cent = centroid_angle_rad + obs_ang_offset_global
            cgx, cgy = math_helper.polarACartesianoPAR(centroid_dist, ang_total_cent)
            tracker_tray_fn("Active_learning", 1, "Qupa4", app_x_m, app_y_m, cgx / 100.0, cgy / 100.0)

        if getattr(apprentice_process, 'AL_LOG_MODE', 0) == 2:
            apprentice_process.log_py(f"[ACTIVE LEARNING] Básico | Dist: {centroid_dist:.1f} cm")

        return {
            'should_move'     : True,
            'pause_ticks'     : PAUSE_TICKS_ON_CHANGE if trigger_pause else 0,
            'target_x'        : smooth_x,
            'target_y'        : smooth_y,
            'target_dist'     : centroid_dist,
            'target_angle'    : centroid_angle,
            'speed_d'         : _get_ctrl_num('d_speed', 30.0),
            'speed_g'         : _get_ctrl_num('g_speed', 80.0),
            'closest_dist'    : closest_master_dist,
            'closest_angle'   : angle_of_closest,
            'front_threshold' : _get_ctrl_num('front_threshold', FRONT_THRESHOLD),
            'back_threshold'  : _get_ctrl_num('back_threshold', BACK_THRESHOLD),
            'prox_threshold'  : PROX_CRITICAL_THRESHOLD,
            'state_name'      : "PERSIGUIENDO_CENTROIDE",
            'state_msg'       : f"BÁSICO | Dist: {centroid_dist:.1f} cm",
            'is_centroid'     : True,
        }
    else:
        history_x.clear()
        history_y.clear()
        current_target_x, current_target_y = None, None
        return None


def reset_history():
    global current_target_x, current_target_y, last_recalc_trigger
    history_x.clear()
    history_y.clear()
    current_target_x, current_target_y = None, None
    last_recalc_trigger = False


class ActiveLearning:
    process = staticmethod(process)
    reset_history = staticmethod(reset_history)
