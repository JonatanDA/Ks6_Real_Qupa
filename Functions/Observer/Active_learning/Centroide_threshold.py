# ============================================================================
# VARIABLES DEL MÓDULO: CENTROIDE THRESHOLD
# ============================================================================
# AL_LOG_MODE              = 2
AL_SMOOTH_TICKS          = 3
AL_SMOOTH_METHOD         = "median"
MAX_TRACKING_DISTANCE    = 35.0      # (cm) Si el centroide está dentro de este radio, NO se mueve.
PAUSE_TICKS_ON_CHANGE    = 1         # Ticks de pausa al reanudar movimiento.

# Umbrales de Supervivencia
FRONT_THRESHOLD          = 15.0
BACK_THRESHOLD           = 10.0
PROX_CRITICAL_THRESHOLD  = 0.80

import math
from Controller import apprentice_process
from Functions import math_helper

history_x = []
history_y = []
last_do_move_state = False


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
    global last_do_move_state

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

        centroid_dist, centroid_angle_rad = math_helper.cartesianoAPolarPAR(smooth_x, smooth_y)
        centroid_angle = centroid_angle_rad * (180.0 / math.pi)

        tracker_tray_fn = getattr(apprentice_process, 'Print_tracker_trayectory_data', None)
        if callable(tracker_tray_fn):
            ang_total_cent = centroid_angle_rad + obs_ang_offset_global
            cgx, cgy = math_helper.polarACartesianoPAR(centroid_dist, ang_total_cent)
            tracker_tray_fn("Active_learning", 2, "Qupa4", app_x_m, app_y_m, cgx / 100.0, cgy / 100.0)

        max_track_dist = _get_ctrl_num('max_tracking_distance', MAX_TRACKING_DISTANCE)
        do_move = (centroid_dist > max_track_dist)
        trigger_pause = False
        if do_move and not last_do_move_state:
            trigger_pause = True
        last_do_move_state = do_move

        if getattr(apprentice_process, 'AL_LOG_MODE', 0) == 2:
            apprentice_process.log_py(
                f"[ACTIVE LEARNING] Threshold({centroid_dist:.1f}) | Activos: {active_count} | Cerca: {closest_master_dist:.1f} cm"
            )

        return {
            'should_move'     : do_move,
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
            'state_msg'       : f"THRESHOLD | Dist: {centroid_dist:.1f} cm",
            'is_centroid'     : True,
        }
    else:
        history_x.clear()
        history_y.clear()
        return None


def reset_history():
    global last_do_move_state
    history_x.clear()
    history_y.clear()
    last_do_move_state = False


class ActiveLearning:
    process = staticmethod(process)
    reset_history = staticmethod(reset_history)
