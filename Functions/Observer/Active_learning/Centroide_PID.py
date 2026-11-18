# ============================================================================
# VARIABLES DEL MÓDULO: CENTROIDE PID
# ============================================================================
# AL_LOG_MODE              = 2
AL_SMOOTH_TICKS          = 3
AL_SMOOTH_METHOD         = "median"

Kp = 0.8
Ki = 0.01
Kd = 0.1
target_centroid_distance = 0.0
pid_tolerance            = 10.0
max_pid_output           = 100.0
min_pid_output           = 0.0
PAUSE_TICKS_ON_CHANGE    = 1

# Umbrales de Supervivencia
FRONT_THRESHOLD          = 25.0
BACK_THRESHOLD           = 10.0
PROX_CRITICAL_THRESHOLD  = 0.90

pid_error_integral = 0.0
pid_last_error     = 0.0

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
    global pid_error_integral, pid_last_error, last_do_move_state

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

        smooth_x, smooth_y = get_smoothed_values(sum_x / active_count, sum_y / active_count)
        centroid_dist, centroid_angle_rad = math_helper.cartesianoAPolarPAR(smooth_x, smooth_y)
        centroid_angle = centroid_angle_rad * (180.0 / math.pi)

        out_pid_dist, out_speed_d, out_speed_g = 0.0, 0.0, 0.0
        do_move = False

        front_thr = _get_ctrl_num('front_threshold', FRONT_THRESHOLD)
        back_thr = _get_ctrl_num('back_threshold', BACK_THRESHOLD)

        if closest_master_dist < front_thr:
            pid_error_integral = 0.0

        error_dist = centroid_dist - target_centroid_distance

        if abs(error_dist) > pid_tolerance:
            p_term = Kp * error_dist
            pid_error_integral += error_dist
            i_term = Ki * pid_error_integral
            d_term = Kd * (error_dist - pid_last_error)
            pid_last_error = error_dist

            out_pid_dist = p_term + i_term + d_term
            if out_pid_dist > max_pid_output:
                out_pid_dist = max_pid_output
            elif out_pid_dist < min_pid_output:
                out_pid_dist = min_pid_output

            global_d = _get_ctrl_num('d_speed', 30.0)
            global_g = _get_ctrl_num('g_speed', 80.0)
            out_speed_d = global_d if global_d > 0.0 else 0.0
            out_speed_g = global_g if global_g > 0.0 else 0.0
            do_move = True
        else:
            pid_error_integral = 0.0

        trigger_pause = False
        if do_move and not last_do_move_state:
            trigger_pause = True
        last_do_move_state = do_move

        tracker_tray_fn = getattr(apprentice_process, 'Print_tracker_trayectory_data', None)
        if callable(tracker_tray_fn):
            ang_total_cent = centroid_angle_rad + obs_ang_offset_global
            cgx, cgy = math_helper.polarACartesianoPAR(centroid_dist, ang_total_cent)
            tracker_tray_fn("Active_learning", 3, "Qupa4", app_x_m, app_y_m, cgx / 100.0, cgy / 100.0, out_pid_dist)

        if getattr(apprentice_process, 'AL_LOG_MODE', 0) == 2:
            apprentice_process.log_py(f"[ACTIVE LEARNING] PID | Dist: {centroid_dist:.1f} cm")

        return {
            'should_move'     : do_move,
            'pause_ticks'     : PAUSE_TICKS_ON_CHANGE if trigger_pause else 0,
            'target_x'        : smooth_x,
            'target_y'        : smooth_y,
            'target_dist'     : out_pid_dist,
            'target_angle'    : centroid_angle,
            'speed_d'         : out_speed_d,
            'speed_g'         : out_speed_g,
            'closest_dist'    : closest_master_dist,
            'closest_angle'   : angle_of_closest,
            'front_threshold' : front_thr,
            'back_threshold'  : back_thr,
            'prox_threshold'  : PROX_CRITICAL_THRESHOLD,
            'state_name'      : "PERSIGUIENDO_CENTROIDE",
            'state_msg'       : "PID ACTIVO",
            'is_centroid'     : True,
        }
    else:
        history_x.clear()
        history_y.clear()
        return None


def reset_history():
    global pid_error_integral, pid_last_error, last_do_move_state
    history_x.clear()
    history_y.clear()
    pid_error_integral = 0.0
    pid_last_error = 0.0
    last_do_move_state = False


class ActiveLearning:
    process = staticmethod(process)
    reset_history = staticmethod(reset_history)
