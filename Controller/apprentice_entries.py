# Archivo: Ks6/apprentice_entries.py
# Ubicación: ~/sim_ws_qupa/Ks6/Controller/apprentice_entries.py
# Descripción:
#   Submódulo de entradas físicas, cámara omnidireccional (/Qupa4/camera/detections),
#   odometría, supervivencia (SP) y control de movimiento por Active Learning del aprendiz
#   en ROS 2 + Gazebo Harmonic / Robot Real.
#   Equivalente 1:1 de Ks6/Controller/apprentice_entries.lua.

import importlib
import math
import sys

sys.modules["Controller.apprentice_entries"] = sys.modules[__name__]
sys.modules["apprentice_entries"] = sys.modules[__name__]


def _require_any(candidates):
    for name in candidates:
        try:
            return importlib.import_module(name)
        except ImportError:
            continue
    raise ImportError("No se pudo cargar ninguno de los módulos: " + ", ".join(candidates))


shared_config      = _require_any(["Functions.shared_config", "shared_config"])
math_helper        = _require_any(["Functions.math_helper", "math_helper"])
Sensors            = _require_any(["Functions.sensors", "sensors"])
Move               = _require_any(["Functions.move", "move"])
Utils              = _require_any(["Functions.utils", "utils"])
from Functions.Observer.noise_manager import NoiseManager
import Functions.Observer.Active_learning.CamposVectoriales as ActiveLearning
apprentice_process = _require_any(["Controller.apprentice_process", "apprentice_process"])

ENABLE_ACTIVE_LEARNING = shared_config.get_controller_var("ENABLE_ACTIVE_LEARNING", True)

# ============================================================================
# 1. VARIABLES DE ESTADO FÍSICO Y ACTIVE LEARNING DEL APRENDIZ
# ============================================================================
motion_active         = False
last_motion_marker    = [0.0, 0.0]
srt_ang_mark_ant      = 0.0
x_mark, y_mark        = 0.0, 0.0
x_ant_r, y_ant_r      = 0.0, 0.0
current_wait_ticks    = 0
obs_stop_flag         = False
MAX_WAIT_TICKS        = 2 * int(shared_config.get_controller_var("ticks_per_second", 4))

al_current_state      = "IDLE"
al_evasion_queue      = []
al_pause_timer        = 0
srt_c1                = 0
SP_CRITICAL_THRESHOLD = 0.10


def _norm_rad(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def change_al_state(app, new_state: str, log_msg: str = "", finish_reason: str = None):
    global al_current_state
    if app.al_current_state != new_state:
        if apprentice_process.AL_LOG_MODE == 3 and app.al_current_state != "IDLE":
            razon = finish_reason or "Razón desconocida"
            apprentice_process.log_py(f">>> [FIN] Estado '{app.al_current_state}' - Razón: {razon} \n")

        app.al_current_state = new_state
        al_current_state = new_state

        if apprentice_process.AL_LOG_MODE == 3 and new_state != "IDLE":
            apprentice_process.log_py(f"\n>>> [ALERTA] {log_msg}")


# ============================================================================
# 2. FASE A: RECOLECCIÓN DE BLOBS OMNIDIRECCIONALES Y RUIDO
# ============================================================================
# ----------------------------------------------------------------------------
# VARIABLES CONFIGURABLES DE CALIBRACIÓN DE CÁMARA OMNIDIRECCIONAL (px -> cm)
# La cámara apunta hacia un espejo superior de 360° (wideanglecamera / espejo hiperbólico).
# Modos disponibles en CAM_CALIB_MODE:
#   - "optical"   : Modelo inverso catadióptrico/ojo de pez (r_norm = c1*fun(theta/c2) -> d = dH*tan(theta))
#   - "polynomial": Polinomio empírico de grado 3 (a3*px^3 + a2*px^2 + a1*px + a0)
#   - "table"     : Interpolación lineal por tramos sobre puntos medidos (px, cm)
#   - "linear"    : Escala lineal directa (px * CAM_PX_TO_CM_LINEAR + CAM_RADIAL_OFFSET_CM)
# ----------------------------------------------------------------------------
CAM_CALIB_MODE              = "table"  # "optical" | "polynomial" | "table" | "linear"

# Geometría de imagen y anillo en píxeles (acorde a camera_sim.yaml / mirror.xacro)
CAM_IMG_WIDTH_PX            = 720.0
CAM_IMG_HEIGHT_PX           = 720.0
CAM_IMG_HALF_SIZE_PX        = 360.0   # Radio de normalización del shader (width / 2)
CAM_INNER_RADIUS_PX         = 80.0    # Radio interno de la zona muerta del espejo
CAM_OUTER_RADIUS_PX         = 300.0   # Radio externo útil del espejo

# Parámetros ópticos del lente/espejo 360° (mirror.xacro: r = c1 * fun(theta / c2))
CAM_LENS_C1                 = 1.0
CAM_LENS_C2                 = 2.0
CAM_LENS_F                  = 0.0
CAM_LENS_FUN                = "sin"   # "sin", "tan" o "id"
CAM_MAX_THETA_DEG           = 88.5    # Cota superior para evitar asíntota de tan(90°)

# Alturas físicas (en cm) desde el suelo/base_link para proyección trigonométrica
CAM_MIRROR_HEIGHT_CM        = 18.0    # Altura del punto nodal cámara-espejo (0.18 m en mirror.xacro)
CAM_TARGET_BEACON_HEIGHT_CM = 9.0     # Altura media del centroide visual del LED/domo del maestro (0.09 m)
# CAM_EFFECTIVE_DELTA_H_CM    = CAM_MIRROR_HEIGHT_CM - CAM_TARGET_BEACON_HEIGHT_CM
CAM_EFFECTIVE_DELTA_H_CM    = 4.88

# Factores de ajuste fino radial y angular
CAM_RADIAL_SCALE_FACTOR     = 1.0     # Multiplicador global de escala en cm
CAM_RADIAL_OFFSET_CM        = 0.0     # Offset aditivo en cm tras la conversión
CAM_PX_TO_CM_LINEAR         = 0.35    # Equivalente directo cm/px (usado si CAM_CALIB_MODE == "linear")
CAM_MIN_DISTANCE_CM         = 8.0     # Distancia mínima física entre centros de Qupa (cm)
CAM_MAX_DISTANCE_CM         = 350.0   # Distancia máxima admisible en la arena (cm)

# Coeficientes para ajuste polinomial [a3, a2, a1, a0] -> d_cm = a3*px^3 + a2*px^2 + a1*px + a0
CAM_POLY_COEFFS             = [0.0, 0.0022, -0.15, 5.0]

# Tabla empírica de pares (distance_px, distance_cm) para interpolación si CAM_CALIB_MODE == "table"
CAM_CALIB_POINTS_PX_CM      = [
    (77.77, 12.0),
    (84.83, 15.0),
    (88.67, 18.0),
    (91.73, 21.0),
    (92.73, 24.0),
    (94.72, 27.0),
    (95.79, 30.0),
    (96.72, 33.0),
    (97.71, 36.0),
    (98.70, 39.0),
    (98.65, 42.0),
    (99.65, 45.0),
    (100.65, 48.0),
    (100.71, 51.0),
    (101.77, 54.0),
    (101.70, 57.0),
    (102.55, 70.0),
]

# Convención angular entre camera_sim_node (atan2(vx, -vy), horario+) y marco ROS 2 del robot (antihorario+)
CAMERA_ANGLE_SIGN           = -1.0
CAMERA_ANGLE_OFFSET_DEG     = 0.0

latest_detections           = []


def update_camera_detections(app, detection_array_msg):
    """Callback auxiliar que almacena las detecciones recibidas desde /Qupa4/camera/detections."""
    global latest_detections
    targets = list(getattr(detection_array_msg, 'targets', []) or [])
    latest_detections = targets
    if app is not None:
        app.latest_detections = targets


def distance_px_to_cm(distance_px: float) -> float:
    """
    Convierte la distancia radial en píxeles (medida desde el centro del espejo 360°)
    a distancia horizontal real en centímetros en el plano de la arena.
    """
    px = float(distance_px)
    if px <= 0.0:
        return 0.0

    # ========================================================================
    # [ESPACIO RESERVADO PARA CALIBRACIÓN EMPÍRICA DE LA CÁMARA (px -> cm)]
    # Cuando midas los puntos reales (ej. a 20, 40, 60, 80, 100 cm) con
    # /Qupa4/camera/debug_image y /Qupa4/camera/detections:
    #   1) Puedes actualizar CAM_POLY_COEFFS = [a3, a2, a1, a0] y cambiar
    #      CAM_CALIB_MODE = "polynomial", o
    #   2) Actualizar CAM_CALIB_POINTS_PX_CM y cambiar CAM_CALIB_MODE = "table".
    # ========================================================================
    mode = str(CAM_CALIB_MODE).lower()

    if mode == "polynomial":
        a3, a2, a1, a0 = CAM_POLY_COEFFS
        dist_cm = ((a3 * px + a2) * px + a1) * px + a0
        dist_cm = dist_cm * float(CAM_RADIAL_SCALE_FACTOR) + float(CAM_RADIAL_OFFSET_CM)
        return max(float(CAM_MIN_DISTANCE_CM), min(float(CAM_MAX_DISTANCE_CM), dist_cm))

    if mode == "table" and len(CAM_CALIB_POINTS_PX_CM) >= 2:
        pts = sorted(CAM_CALIB_POINTS_PX_CM, key=lambda item: item[0])
        if px <= pts[0][0]:
            p0, p1 = pts[0], pts[1]
        elif px >= pts[-1][0]:
            p0, p1 = pts[-2], pts[-1]
        else:
            p0, p1 = pts[0], pts[1]
            for i in range(len(pts) - 1):
                if pts[i][0] <= px <= pts[i + 1][0]:
                    p0, p1 = pts[i], pts[i + 1]
                    break
        denom = (p1[0] - p0[0])
        t = ((px - p0[0]) / denom) if abs(denom) > 1e-6 else 0.0
        dist_cm = p0[1] + t * (p1[1] - p0[1])
        dist_cm = dist_cm * float(CAM_RADIAL_SCALE_FACTOR) + float(CAM_RADIAL_OFFSET_CM)
        return max(float(CAM_MIN_DISTANCE_CM), min(float(CAM_MAX_DISTANCE_CM), dist_cm))

    if mode == "linear":
        dist_cm = px * float(CAM_PX_TO_CM_LINEAR) * float(CAM_RADIAL_SCALE_FACTOR) + float(CAM_RADIAL_OFFSET_CM)
        return max(float(CAM_MIN_DISTANCE_CM), min(float(CAM_MAX_DISTANCE_CM), dist_cm))

    # Modo por defecto ("optical"): inversión del modelo de proyección del espejo superior 360°
    r_norm = px / max(1.0, float(CAM_IMG_HALF_SIZE_PX) * float(CAM_LENS_C1))
    fun_name = str(CAM_LENS_FUN).lower()
    if fun_name == "sin":
        r_clipped = max(-0.9999, min(0.9999, r_norm))
        theta_rad = (math.asin(r_clipped) - float(CAM_LENS_F)) * float(CAM_LENS_C2)
    elif fun_name == "tan":
        theta_rad = (math.atan(r_norm) - float(CAM_LENS_F)) * float(CAM_LENS_C2)
    else:
        theta_rad = (r_norm - float(CAM_LENS_F)) * float(CAM_LENS_C2)

    max_theta_rad = math.radians(float(CAM_MAX_THETA_DEG))
    theta_rad = max(0.01, min(max_theta_rad, theta_rad))

    delta_h_cm = float(CAM_EFFECTIVE_DELTA_H_CM)
    if delta_h_cm <= 0.1:
        delta_h_cm = max(1.0, float(CAM_MIRROR_HEIGHT_CM) - float(CAM_TARGET_BEACON_HEIGHT_CM))

    dist_cm = delta_h_cm * math.tan(theta_rad) * float(CAM_RADIAL_SCALE_FACTOR) + float(CAM_RADIAL_OFFSET_CM)
    return max(float(CAM_MIN_DISTANCE_CM), min(float(CAM_MAX_DISTANCE_CM), dist_cm))


def collect_omnidirectional_blobs(app, robots: dict = None, max_fov_radius_cm: float = 0.0, evaluator_ruido_fn=None):
    """
    Construye yellow_blobs y green_blobs exclusivamente a partir de las detecciones
    de la cámara omnidireccional (/Qupa4/camera/detections, qupa_msgs/msg/DetectionArray).
    """
    yellow_blobs = []
    green_blobs  = []

    dets = getattr(app, 'latest_detections', None)
    if dets is None:
        dets = latest_detections

    for det in (dets or []):
        raw_dist_cm = distance_px_to_cm(float(det.distance_px))
        raw_angle_deg = math_helper.normalize_deg_180(
            float(CAMERA_ANGLE_SIGN) * float(det.angle_deg) + float(CAMERA_ANGLE_OFFSET_DEG)
        )

        # Log de diagnóstico en consola (visible con NIVEL_LOG = 2)
        # apprentice_process.log_py(
        #     f"[CAM_BLOB] color={det.color} | px={det.distance_px:.1f} -> dist={raw_dist_cm:.2f} cm "
        #     f"| ang_raw={det.angle_deg:.1f}° -> ang={raw_angle_deg:.2f}° | area={det.area} "
        #     f"| fov_max={max_fov_radius_cm:.1f} cm"
        # )

        if max_fov_radius_cm > 0.0 and raw_dist_cm > max_fov_radius_cm:
            # apprentice_process.log_py(
            #     f"   [!] Descartado por FOV: {raw_dist_cm:.2f} cm > {max_fov_radius_cm:.2f} cm"
            # )
            continue

        noisy_dist, noisy_angle = NoiseManager.aplicar_ruido(
            raw_dist_cm, raw_angle_deg, apprentice_process.noise_cfg
        )

        if callable(evaluator_ruido_fn):
            evaluator_ruido_fn(raw_angle_deg, raw_dist_cm, noisy_angle, noisy_dist)

        color_upper = str(getattr(det, 'color', '')).strip().upper()
        if color_upper in ("BLUE", "YELLOW"):
            yellow_blobs.append({'angle': noisy_angle, 'distance': noisy_dist, 'type': 2})
        elif color_upper == "RED":
            yellow_blobs.append({'angle': noisy_angle, 'distance': noisy_dist, 'type': 1})
        elif color_upper == "GREEN":
            green_blobs.append({'angle': noisy_angle, 'distance': noisy_dist})

    return yellow_blobs, green_blobs


# ============================================================================
# 3. GESTIÓN FÍSICA, SENSORIAL Y ACTIVE LEARNING (store_relative_positions)
# ============================================================================
def store_relative_positions(
    app,
    robots: dict,
    timer_tick: int,
    max_fov_radius_cm: float,
    active_learning,
    set_led_fn=None,
    tracker_printer=None,
):
    global motion_active, last_motion_marker, srt_ang_mark_ant
    global x_mark, y_mark, x_ant_r, y_ant_r, current_wait_ticks, obs_stop_flag
    global al_evasion_queue, al_pause_timer, srt_c1, MAX_WAIT_TICKS

    MAX_WAIT_TICKS = int(2 * apprentice_process.ticks_per_second)

    if apprentice_process.AP_1 == 1:
        if apprentice_process.UP_LOG_MODE >= 1 or apprentice_process.SRP_LOG_MODE >= 1:
            apprentice_process.log_py("\n\n\n===========================================================")
            apprentice_process.log_py(" [INICIO CICLO DE APRENDIZAJE] ")
            apprentice_process.log_py("===========================================================")
        apprentice_process.AP_1 = 0

    if tracker_printer is not None:
        tracker_printer.Print_tracker_trayectory_data("trajectory", "Qupa4", 0, 0)

    # Lectura de proximidad y colisiones
    proximity = Sensors.proximidad(app, False)
    if proximity[0] < 1.0:
        app.srt_c1 = 0
        srt_c1 = 0
    else:
        if app.srt_c1 == 0:
            apprentice_process.log_py(" CHOQUE!!!!!!! ")
            app.srt_c1 += 1
            srt_c1 = app.srt_c1

    Utils.Temporizador_1(app, 20.0, apprentice_process.ticks_per_second)
    if not app.temp and callable(set_led_fn):
        set_led_fn(app, "black")

    # =========================================================================
    # FASE 0: EJECUCIÓN DE ODOMETRÍA, OBSTÁCULOS Y SUPERVIVENCIA (SP)
    # =========================================================================
    if app.motion_active:
        prox_data = Sensors.proximidad(app, False)
        app.obs_stop_flag = False

        is_object_critical    = (prox_data[0] > 0.9)
        evadiendo_hacia_atras = (app.last_motion_marker[0] < 0)

        if is_object_critical:
            if evadiendo_hacia_atras:
                if abs(prox_data[1]) >= 135.0:
                    app.obs_stop_flag = True
            else:
                if abs(prox_data[1]) <= 45.0:
                    app.obs_stop_flag = True

        if app.obs_stop_flag:
            app.current_wait_ticks += 1
            if app.current_wait_ticks > MAX_WAIT_TICKS:
                cur_ang_abort = _norm_rad(srt_ang_mark_ant + float(getattr(app, 'move_angle', 0.0)))
                dx_ab, dy_ab = math_helper.polarACartesianoPAR(
                    float(getattr(app, 'move_distance', 0.0)), cur_ang_abort
                )
                x_ant_r = x_ant_r + dx_ab
                y_ant_r = y_ant_r + dy_ab
                srt_ang_mark_ant = cur_ang_abort
                app.x_mark = x_ant_r
                app.y_mark = y_ant_r
                x_mark = app.x_mark
                y_mark = app.y_mark
                Move.r_move(app)
                Move.set_wheels_cm_s(app, 0.0, 0.0, apprentice_process.FOOTBOT_INTERWHEEL_DISTANCE)

                escape_dir = 1.0 if evadiendo_hacia_atras else -1.0
                app.last_motion_marker = [
                    escape_dir * float(apprentice_process.back_threshold) * float(apprentice_process.porcentaje),
                    0.0,
                    float(apprentice_process.d_speed),
                    float(apprentice_process.g_speed),
                ]
                app.current_wait_ticks = 0
                app.obs_stop_flag = False

                accion_str = "Avanzando" if escape_dir > 0 else "Retrocediendo"
                msg = f"EVADIENDO OBJETO FÍSICO | {accion_str} | Intensidad SP: {prox_data[0]:.2f}"
                change_al_state(app, "EVASION_SP", msg)
        else:
            app.current_wait_ticks = 0

        dyn_sd = (
            app.last_motion_marker[2]
            if len(app.last_motion_marker) > 2
            else float(apprentice_process.d_speed)
        )
        dyn_sg = (
            app.last_motion_marker[3]
            if len(app.last_motion_marker) > 3
            else float(apprentice_process.g_speed)
        )

        _, _, cur_move_dist, cur_move_ang = Move.move(
            app,
            app.last_motion_marker[0],
            app.last_motion_marker[1],
            dyn_sd,
            dyn_sg,
            timer_tick,
            app.obs_stop_flag,
        )
    else:
        app.current_wait_ticks = 0
        cur_move_dist = 0.0
        cur_move_ang = 0.0

    # =========================================================================
    # FASE A: Recolección y Ruido
    # =========================================================================
    eval_ruido_fn = (
        tracker_printer.evaluator_ruido
        if (tracker_printer is not None and tracker_printer.enable_eval_ruido)
        else None
    )
    yellow_blobs, green_blobs = collect_omnidirectional_blobs(
        app, robots, max_fov_radius_cm, eval_ruido_fn
    )

    # =========================================================================
    # FASE A.5: CORRECCIÓN DE ODOMETRÍA (EXCLUSIVAMENTE DESDE LO QUE DEVUELVE Move.move)
    # =========================================================================
    obs_ang_offset_global = _norm_rad(srt_ang_mark_ant + float(cur_move_ang))
    dx_mov_cm, dy_mov_cm = math_helper.polarACartesianoPAR(
        float(cur_move_dist), obs_ang_offset_global
    )
    app.x_mark = x_ant_r + dx_mov_cm
    app.y_mark = y_ant_r + dy_mov_cm
    x_mark = app.x_mark
    y_mark = app.y_mark

    if app.motion_active and app.c1 == 2:
        x_ant_r = app.x_mark
        y_ant_r = app.y_mark
        srt_ang_mark_ant = obs_ang_offset_global
        app.motion_active = False
        app.current_wait_ticks = 0
        Move.r_move(app)
        Move.set_wheels_cm_s(app, 0.0, 0.0, apprentice_process.FOOTBOT_INTERWHEEL_DISTANCE)
        change_al_state(app, "IDLE", "")

    motion_active = app.motion_active
    last_motion_marker = app.last_motion_marker
    current_wait_ticks = app.current_wait_ticks
    obs_stop_flag = app.obs_stop_flag

    # =========================================================================
    # EMPAQUETADO DE DATOS FÍSICOS Y LLAMADO A apprentice_process
    # =========================================================================
    datos_fisicos = {
        'yellow_blobs'          : yellow_blobs,
        'green_blobs'           : green_blobs,
        'obs_ang_offset_global' : obs_ang_offset_global,
        'x_mark'                : app.x_mark,
        'y_mark'                : app.y_mark,
    }

    updated_anterior_indices, timeout_occurred = (
        apprentice_process.procesar_datos_logicos_y_reconstruccion(datos_fisicos)
    )

    if timeout_occurred:
        return

    # =========================================================================
    # FASE E: CÁLCULO Y DECISIÓN DE MOVIMIENTO (ACTIVE LEARNING)
    # =========================================================================
    if active_learning is not None:
        al_result = active_learning.process(
            obs_ang_offset_global, updated_anterior_indices, app.x, app.y
        )

        if al_result:
            prox_data = Sensors.proximidad(app, False)
            is_prox_critical = (prox_data[0] > al_result['prox_threshold'])
            is_cam_critical  = (al_result['closest_dist'] < al_result['front_threshold'])

            is_evading = app.al_current_state in (
                "EVASION_VISUAL",
                "EVASION_CIEGA",
                "EVASION_LATERAL",
            )
            override_normal_move = False

            # 1. DETENER MOVIMIENTO SI HAY UNA EMERGENCIA NUEVA (Y no estamos ya evadiendo)
            if (is_prox_critical or is_cam_critical) and app.motion_active and not is_evading:
                x_ant_r = app.x_mark
                y_ant_r = app.y_mark
                srt_ang_mark_ant = obs_ang_offset_global
                app.motion_active = False
                Move.r_move(app)
                Move.set_wheels_cm_s(app, 0.0, 0.0, apprentice_process.FOOTBOT_INTERWHEEL_DISTANCE)
                app.al_evasion_queue.clear()
                app.al_pause_timer = 0

            # 2. GENERACIÓN DE RUTAS DE ESCAPE (Si estamos detenidos y no hay nada encolado)
            if (is_prox_critical or is_cam_critical) and len(app.al_evasion_queue) == 0 and not app.motion_active:
                override_normal_move = True

                is_sandwich = False
                if is_prox_critical and is_cam_critical:
                    diff = abs(al_result['closest_angle'] - prox_data[1])
                    if 90.0 < diff < 270.0:
                        is_sandwich = True

                if is_sandwich:
                    # REGLA C: Sándwich -> Evasión Lateral (90° respecto a la cámara)
                    escape_ang = al_result['closest_angle'] + 90.0
                    app.al_evasion_queue.append({
                        'dist': al_result['back_threshold'],
                        'ang': escape_ang,
                        'sp_d': float(apprentice_process.d_speed),
                        'sp_g': float(apprentice_process.g_speed),
                        'msg': "Evasión Lateral (Sándwich)",
                        'state': "EVASION_LATERAL",
                    })

                elif is_cam_critical:
                    # REGLA A: Solo Cámara o Mismo Lado -> Evasión Visual
                    escape_ang = (
                        (al_result['closest_angle'] - 180.0)
                        if al_result['closest_angle'] > 0
                        else (al_result['closest_angle'] + 180.0)
                    )
                    app.al_evasion_queue.append({
                        'dist': al_result['back_threshold'],
                        'ang': escape_ang,
                        'sp_d': float(apprentice_process.d_speed),
                        'sp_g': float(apprentice_process.g_speed),
                        'msg': "Evasión Visual (Cámara)",
                        'state': "EVASION_VISUAL",
                    })

                elif is_prox_critical:
                    # REGLA B: Solo Proximidad -> Evasión Ciega en 2 Pasos
                    sign_val = -1.0 if prox_data[1] >= 0.0 else 1.0
                    ang_1 = 45.0 * sign_val
                    ang_2 = -45.0 * sign_val
                    base_escape = 180.0 if abs(prox_data[1]) <= 90.0 else 0.0

                    app.al_evasion_queue.append({
                        'dist': al_result['back_threshold'],
                        'ang': base_escape + ang_1,
                        'sp_d': float(apprentice_process.d_speed),
                        'sp_g': float(apprentice_process.g_speed),
                        'msg': "Evasión Ciega P1",
                        'state': "EVASION_CIEGA",
                    })
                    app.al_evasion_queue.append({
                        'dist': al_result['back_threshold'],
                        'ang': ang_2,
                        'sp_d': float(apprentice_process.d_speed),
                        'sp_g': float(apprentice_process.g_speed),
                        'msg': "Evasión Ciega P2",
                        'state': "EVASION_CIEGA",
                    })

            # REGLA 4 / 3.a.2: Verificación de Solapamiento con el Centroide
            if not override_normal_move and al_result.get('is_centroid') and al_result.get('should_move'):
                is_overlapping = False
                for k, data in apprentice_process.srt_anterior_data.items():
                    if updated_anterior_indices.get(k) and data.get('distance') is not None:
                        r_x, r_y = math_helper.polarACartesianoPAR(
                            float(data['distance']), float(data['angle']) * (math.pi / 180.0)
                        )
                        dist_c_to_r = math.hypot(al_result['target_x'] - r_x, al_result['target_y'] - r_y)
                        if dist_c_to_r < 15.0:
                            is_overlapping = True
                            break

                if is_overlapping:
                    change_al_state(
                        app, "ESPERANDO_DESPEJE", "Centroide solapado con robot. Modo estático."
                    )
                    override_normal_move = True

            # 3. EJECUTOR DE COLA DE EVASIÓN Y PAUSAS
            if app.al_pause_timer > 0 and not app.motion_active:
                app.al_pause_timer -= 1

            elif not app.motion_active and len(app.al_evasion_queue) > 0:
                next_step = app.al_evasion_queue.pop(0)
                ev_ang  = float(next_step['ang'])
                ev_dist = float(next_step['dist'])

                while ev_ang > 180.0:
                    ev_ang -= 360.0
                while ev_ang < -180.0:
                    ev_ang += 360.0

                if ev_ang > 90.0:
                    ev_ang -= 180.0
                    ev_dist = -ev_dist
                elif ev_ang < -90.0:
                    ev_ang += 180.0
                    ev_dist = -ev_dist

                app.motion_active = True
                app.last_motion_marker = [ev_dist, ev_ang, next_step['sp_d'], next_step['sp_g']]
                change_al_state(app, next_step['state'], next_step['msg'])
                Move.r_move(app)

            # 4. MOVIMIENTO NORMAL Y DESVÍOS (Solo si no hay emergencias ni solapamientos)
            elif not override_normal_move and not app.motion_active and al_result.get('should_move'):
                if al_result.get('pause_ticks', 0) > 0:
                    app.al_pause_timer = int(al_result['pause_ticks'])
                else:
                    path_blocked = False
                    if al_result.get('is_centroid'):
                        t_len = float(al_result['target_dist'])
                        for k, data in apprentice_process.srt_anterior_data.items():
                            if (
                                updated_anterior_indices.get(k)
                                and data.get('distance') is not None
                                and float(data['distance']) < t_len
                            ):
                                r_ang_rad = float(data['angle']) * (math.pi / 180.0)
                                t_ang_rad = float(al_result['target_angle']) * (math.pi / 180.0)
                                angle_diff = r_ang_rad - t_ang_rad
                                p = float(data['distance']) * math.cos(angle_diff)
                                if 0.0 < p < t_len:
                                    dist_to_path = abs(float(data['distance']) * math.sin(angle_diff))
                                    if dist_to_path < (float(al_result['front_threshold']) * 0.8):
                                        path_blocked = True
                                        break

                    if path_blocked:
                        app.al_evasion_queue.append({
                            'dist': float(al_result['target_dist']) / 2.0,
                            'ang': float(al_result['target_angle']) + 45.0,
                            'sp_d': float(apprentice_process.d_speed),
                            'sp_g': float(apprentice_process.g_speed),
                            'msg': "Parada Extra: Rodeando bloqueo",
                            'state': "CALCULANDO_DESVIO",
                        })
                        change_al_state(app, "CALCULANDO_DESVIO", "Ruta bloqueada, encolando desvío.")
                    else:
                        opt_ang  = float(al_result['target_angle'])
                        opt_dist = float(al_result['target_dist']) * float(apprentice_process.porcentaje)

                        if opt_ang > 90.0:
                            opt_ang -= 180.0
                            opt_dist = -opt_dist
                        elif opt_ang < -90.0:
                            opt_ang += 180.0
                            opt_dist = -opt_dist

                        app.motion_active = True
                        app.last_motion_marker = [
                            opt_dist,
                            opt_ang,
                            float(al_result['speed_d']),
                            float(al_result['speed_g']),
                        ]
                        change_al_state(app, al_result['state_name'], al_result['state_msg'])
                        Move.r_move(app)

            motion_active = app.motion_active
            last_motion_marker = app.last_motion_marker
            al_evasion_queue = app.al_evasion_queue
            al_pause_timer = app.al_pause_timer


# ============================================================================
# 4. FUNCIONES DE RESET DEL OBSERVADOR (resetOBS, soft_reset_observer)
# ============================================================================
def resetOBS(app, set_led_fn=None):
    global srt_c1, srt_ang_mark_ant, x_mark, y_mark, x_ant_r, y_ant_r
    global last_motion_marker, motion_active, current_wait_ticks, obs_stop_flag
    global al_evasion_queue, al_pause_timer

    srt_c1 = 0
    srt_ang_mark_ant = 0.0
    x_mark, y_mark, x_ant_r, y_ant_r = 0.0, 0.0, 0.0, 0.0

    last_motion_marker  = [0.0, 0.0, float(apprentice_process.d_speed), float(apprentice_process.g_speed)]
    motion_active       = False
    current_wait_ticks  = 0
    obs_stop_flag       = False
    al_evasion_queue    = []
    al_pause_timer      = 0

    app.srt_c1 = 0
    app.x_mark = 0.0
    app.y_mark = 0.0
    app.last_motion_marker = list(last_motion_marker)
    app.motion_active = False
    app.current_wait_ticks = 0
    app.obs_stop_flag = False
    app.al_evasion_queue = []
    app.al_pause_timer = 0
    app.posiciones_guardadas = {}
    app.indice = 1
    app.PI_end = False
    app.PI_counter_seg = 0

    apprentice_process.c4 = 0

    if callable(set_led_fn):
        set_led_fn(app, "black")
    Utils.r_Temporizador_1(app)
    Utils.r_Temporizador_2(app)
    Move.r_move(app)


def soft_reset_observer(app, set_led_fn=None):
    resetOBS(app, set_led_fn)
    apprentice_process.soft_reset_observer_logic()
    app.cycle_origin_x = app.x
    app.cycle_origin_y = app.y
    app.cycle_origin_yaw = app.yaw
    if hasattr(ActiveLearning, "reset_history"):
        ActiveLearning.reset_history()
    change_al_state(app, "IDLE", "Fin del Random Walk, volviendo a observar")


class ApprenticeEntries:
    change_al_state               = staticmethod(change_al_state)
    distance_px_to_cm             = staticmethod(distance_px_to_cm)
    update_camera_detections      = staticmethod(update_camera_detections)
    collect_omnidirectional_blobs = staticmethod(collect_omnidirectional_blobs)
    store_relative_positions      = staticmethod(store_relative_positions)
    resetOBS                      = staticmethod(resetOBS)
    soft_reset_observer           = staticmethod(soft_reset_observer)