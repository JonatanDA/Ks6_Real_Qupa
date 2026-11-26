# Archivo: Ks6/apprentice_process.py
# Ubicación: ~/sim_ws_qupa/Ks6/Controller/apprentice_process.py
# Descripción:
#   Submódulo de procesamiento lógico del aprendiz (paciencia, segmentación,
#   parametrización, almacenamiento, similitud de comportamientos y reconstrucción global).
#   Toma sus parámetros desde Ks6_apprentice_controller.py (y solo usa valores por defecto
#   estilo `var = var or default` cuando se importa de forma aislada).
#   Equivalente 1:1 de Ks6/Controller/apprentice_process.lua.

from builtins import print
import math
import sys

sys.modules["Controller.apprentice_process"] = sys.modules[__name__]
sys.modules["apprentice_process"] = sys.modules[__name__]

# ============================================================================
# [SECCIÓN 1] IMPORTACIÓN DE SUBMÓDULOS DEL OBSERVADOR
# ============================================================================
from Functions import shared_config
from Functions import math_helper
from Functions.Observer.Point_Cloud_Segmenter import PointCloudSegmenter
from Functions.Observer.Segment_Parameterizer import SegmentParameterizer
from Functions.Observer.Parameter_Storage import ParameterStorage
from Functions.Observer.Global_Trajectory_Reconstructor import GlobalTrajectoryReconstructor
import Functions.Observer.BehaviorSimilarity as BehaviorSimilarity

param_store = ParameterStorage()

# ============================================================================
# [SECCIÓN 2] VARIABLES GLOBALES DE ESTADO Y MEMORIA
# (Heredadas del controlador principal Ks6_apprentice_controller si existe)
# ============================================================================
ticks_per_second                    = shared_config.get_controller_var("ticks_per_second", 4)
max_time                            = shared_config.get_controller_var("max_time", 5000 * ticks_per_second)
Distribucion_sensores_de_proximidad = shared_config.get_controller_var("Distribucion_sensores_de_proximidad", [0, -45, 45, -90, 90, 180])
Num_cantidad_leds                   = shared_config.get_controller_var("Num_cantidad_leds", 2)
FOOTBOT_INTERWHEEL_DISTANCE         = shared_config.get_controller_var("FOOTBOT_INTERWHEEL_DISTANCE", 8.6)

factor_vel                          = shared_config.get_controller_var("factor_vel", 1)
speed_dem                           = shared_config.get_controller_var("speed_dem", 6.0 * factor_vel)
speed_dem_g                         = shared_config.get_controller_var("speed_dem_g", 40.0)

d_speed                             = shared_config.get_controller_var("d_speed", 8.0)
g_speed                             = shared_config.get_controller_var("g_speed", 44.46)

distance_treshold                   = shared_config.get_controller_var("distance_treshold", 20)
MAX_PATIENCE                        = shared_config.get_controller_var("MAX_PATIENCE", 60 * ticks_per_second)
BASE_RESTORE                        = shared_config.get_controller_var("BASE_RESTORE", 12 * ticks_per_second)
PATIENCE_LVL2_MULTIPLIER            = float(shared_config.get_controller_var("PATIENCE_LVL2_MULTIPLIER", 2.0))
DECAY_RATES                         = dict(shared_config.get_controller_var("DECAY_RATES", {
    1: 0.30,  # 30% de pérdida por cada vez que ESE fragmento hace Nivel 1
    2: 0.25,  # 25% de pérdida por cada vez que ESE fragmento hace Nivel 2
    3: 0.0,   # 0% de pérdida para Nivel 3 (Totalmente nuevo, restaura full)
}))

back_threshold                      = shared_config.get_controller_var("back_threshold", 10)
front_threshold                     = shared_config.get_controller_var("front_threshold", 30)
max_tracking_distance               = shared_config.get_controller_var("max_tracking_distance", 10)
porcentaje                          = shared_config.get_controller_var("porcentaje", 0.75)

noise_cfg = shared_config.get_controller_var("noise_cfg", {
    'ENABLE_NOISE'           : False,
    'USE_DISTANCE_DEPENDENT' : True,
    'USE_OUTLIERS'           : False,
    'USE_BIAS'               : True,
    'NOISE_BASE_RADIUS'      : 0.000,
    'NOISE_SIGMAS'           : 2.0,
    'NOISE_FACTOR'           : 0.1,
    'OUTLIER_PROB'           : 0.01,
    'BIAS_OFFSET'            : 0.0,
})

seg_cfg = shared_config.get_controller_var("seg_cfg", {
    'distance_treshold'     : 15,
    'limite_maximo_ventana' : (30) / factor_vel,
    'tamano_subsampling'    : 15,
    'use_subsampling'       : False,
})

CONFIG = shared_config.get_controller_var("CONFIG", {
    'TOL_DIST': 15,
    'TOL_ANG' : 15,
})

SRP_LOG_MODE  = shared_config.get_controller_var("SRP_LOG_MODE", 0)
UP_LOG_MODE   = shared_config.get_controller_var("UP_LOG_MODE", 2)
AL_LOG_MODE   = shared_config.get_controller_var("AL_LOG_MODE", 0)
DEBUG_EVASION = shared_config.get_controller_var("DEBUG_EVASION", 0)

all_params           = []
parametros_elegidos  = []
c4                   = 0
AP_1                 = 1

N_MAX_INTENTOS       = int(shared_config.get_controller_var("N_MAX_INTENTOS", 3))
intentos_redundantes = N_MAX_INTENTOS
TIEMPO_RW_SEG        = float(shared_config.get_controller_var("TIEMPO_RW_SEG", 30))
TIEMPO_RW_TICKS      = int(TIEMPO_RW_SEG * ticks_per_second)
rw_timer             = 0
K_UMBRAL_SIMILITUD   = 0.85

current_patience     = MAX_PATIENCE
master_tracking      = {}
srt_anterior_data    = {}

ENABLE_ACTIVE_LEARNING               = False
is_master                            = 0
Print_tracker_python                 = None
Print_tracker_paciencia              = None
Print_tracker_trayectory_data        = None
Print_tracker_calidad_imitacion_data = None
Print_tracker_varianza               = None
evaluator_otsu                       = None


def log_py(msg):
    shared_config.log_py(msg)


log = log_py


def fmt_f(val, decimals=2):
    return math_helper.fmt_f(val, decimals)


def apply_injected_vars(vars_dict: dict):
    """
    Sincroniza primero las variables desde Ks6_apprentice_controller y aplica encima
    únicamente las variables explícitamente recibidas en `--vars` (vars_dict).
    """
    global speed_dem, speed_dem_g, MAX_PATIENCE, BASE_RESTORE, TIEMPO_RW_SEG, TIEMPO_RW_TICKS, current_patience
    global PATIENCE_LVL2_MULTIPLIER, DECAY_RATES, N_MAX_INTENTOS, intentos_redundantes
    global ENABLE_ACTIVE_LEARNING

    this_mod = sys.modules[__name__]
    ctrl_mod = shared_config.get_active_controller_module()
    shared_config.sync_from_controller(ctrl_mod, extra_targets=[this_mod])

    if ctrl_mod is not None:
        if hasattr(ctrl_mod, "DECAY_RATES"):
            DECAY_RATES = dict(getattr(ctrl_mod, "DECAY_RATES"))
        if hasattr(ctrl_mod, "PATIENCE_LVL2_MULTIPLIER"):
            PATIENCE_LVL2_MULTIPLIER = float(getattr(ctrl_mod, "PATIENCE_LVL2_MULTIPLIER"))
        if hasattr(ctrl_mod, "TIEMPO_RW_SEG"):
            TIEMPO_RW_SEG = float(getattr(ctrl_mod, "TIEMPO_RW_SEG"))
        if hasattr(ctrl_mod, "N_MAX_INTENTOS"):
            N_MAX_INTENTOS = int(getattr(ctrl_mod, "N_MAX_INTENTOS"))
            intentos_redundantes = N_MAX_INTENTOS

    if not vars_dict:
        TIEMPO_RW_TICKS = int(TIEMPO_RW_SEG * ticks_per_second)
        current_patience = int(MAX_PATIENCE)
        return

    try:
        import Functions.Observer.Active_learning.CamposVectoriales as CamposVectorialesMod
    except Exception:
        CamposVectorialesMod = None

    try:
        from Controller import apprentice_entries as ApprenticeEntriesMod
    except Exception:
        ApprenticeEntriesMod = None

    base_fvel = float(factor_vel) if float(factor_vel) != 0.0 else 1.0
    sdem_per_fvel = float(speed_dem) / base_fvel

    for k, v in vars_dict.items():
        if hasattr(this_mod, k):
            setattr(this_mod, k, v)
        if ctrl_mod is not None and hasattr(ctrl_mod, k):
            setattr(ctrl_mod, k, v)
        if k in noise_cfg:
            noise_cfg[k] = v
        if k in seg_cfg:
            seg_cfg[k] = v
        if k in CONFIG:
            CONFIG[k] = v
        if CamposVectorialesMod is not None and hasattr(CamposVectorialesMod, k):
            setattr(CamposVectorialesMod, k, v)
        if ApprenticeEntriesMod is not None and hasattr(ApprenticeEntriesMod, k):
            setattr(ApprenticeEntriesMod, k, v)
        if k == "ENABLE_ACTIVE_LEARNING":
            ENABLE_ACTIVE_LEARNING = bool(v)

    if "factor_vel" in vars_dict:
        if "speed_dem" not in vars_dict:
            speed_dem = sdem_per_fvel * float(factor_vel)
        if "speed_dem_g" not in vars_dict:
            speed_dem_g = float(speed_dem) / 2.0
        seg_cfg['limite_maximo_ventana'] = 30.0 / float(factor_vel)
    elif "speed_dem" in vars_dict and "speed_dem_g" not in vars_dict:
        speed_dem_g = float(speed_dem) / 2.0

    if "ticks_per_second" in vars_dict:
        if "MAX_PATIENCE" not in vars_dict:
            MAX_PATIENCE = int(160 * ticks_per_second)
        if "BASE_RESTORE" not in vars_dict:
            BASE_RESTORE = int(20 * ticks_per_second)

    TIEMPO_RW_TICKS = int(TIEMPO_RW_SEG * ticks_per_second)
    intentos_redundantes = int(N_MAX_INTENTOS)
    current_patience = int(MAX_PATIENCE)

    if ctrl_mod is not None:
        ctrl_mod.speed_dem = speed_dem
        ctrl_mod.speed_dem_g = speed_dem_g
        ctrl_mod.MAX_PATIENCE = MAX_PATIENCE
        ctrl_mod.BASE_RESTORE = BASE_RESTORE

    shared_config.sync_from_controller(ctrl_mod, extra_targets=[this_mod])


def soft_reset_observer_logic():
    global current_patience, master_tracking, srt_anterior_data, AP_1
    current_patience  = MAX_PATIENCE
    master_tracking.clear()
    srt_anterior_data.clear()
    AP_1              = 1


def ejecutar_limpieza_ciclo():
    global current_winning_ids
    current_cfg = {'CONFIG': CONFIG, 'LOG_MODE': SRP_LOG_MODE}
    param_store.clear(current_cfg, current_winning_ids)
    current_winning_ids = {}
    soft_reset_observer_logic()


# ============================================================================
# [SECCIÓN 3] GESTIÓN DE PACIENCIA Y RECONSTRUCCIÓN
# ============================================================================
# DECAY_RATES y PATIENCE_LVL2_MULTIPLIER se sincronizan desde Ks6_apprentice_controller.py

current_winning_ids = {}


def update_patience(restore_level, data_count, fragment_history_count):
    global current_patience
    if restore_level == 0:
        return

    if restore_level == 3:
        decay_factor = 1.0
        restore_amount = math.floor(MAX_PATIENCE * decay_factor)

        current_patience = min(MAX_PATIENCE, current_patience + restore_amount)

        if UP_LOG_MODE == 2 or UP_LOG_MODE == 3:
            log(
                f"[PACIENCIA] Restaurada (NIVEL 3). Eficacia 100%. "
                f"+{fmt_f(restore_amount / ticks_per_second, 2)} seg. "
                f"Actual: {fmt_f(current_patience / ticks_per_second, 2)} seg."
            )

    elif restore_level == 1 or restore_level == 2:
        decay_factor = max(0.0, 1.0 - (fragment_history_count * DECAY_RATES[restore_level]))
        base_efectiva = BASE_RESTORE * decay_factor
        restore_amount = math.floor(base_efectiva)

        if restore_level == 2:
            restore_amount = math.floor(base_efectiva * float(PATIENCE_LVL2_MULTIPLIER))

        current_patience = min(MAX_PATIENCE, current_patience + restore_amount)

        if UP_LOG_MODE == 2 or UP_LOG_MODE == 3:
            log(
                f"[PACIENCIA] Rest. Parcial (NIVEL {restore_level}). "
                f"Eficacia {decay_factor * 100:.0f}%. "
                f"+{fmt_f(restore_amount / ticks_per_second, 2)} seg. "
                f"Actual: {fmt_f(current_patience / ticks_per_second, 2)} seg."
            )


def check_patience_and_reconstruct(only_decrement=False):
    global current_patience, all_params, intentos_redundantes, current_winning_ids, c4

    if param_store.is_done():
        return

    def construir_linea_resumen(id_robot, lista_comportamientos):
        resumen = f"{id_robot} & Imitado&"
        for i, comp in enumerate(lista_comportamientos, start=1):
            resumen += f"p{i}"
            if isinstance(comp, (dict, list)):
                physical_segs = BehaviorSimilarity.translate_to_physical(comp)
                segs_str = [f"({fmt_f(seg[1], 2)},{fmt_f(seg[0], 2)})" for seg in physical_segs]
                resumen += ",".join(segs_str)
        return resumen

    def contar_segmentos_distancia(comp):
        if not comp:
            return 0
        if isinstance(comp, dict) and 'value' in comp and comp.get('type') is not None:
            d_count = 0
            for idx in range(len(comp['value'])):
                if math_helper.get_val_type(comp['type'], idx) == "D":
                    d_count = d_count + 1
            return d_count
        return len(comp)

    if only_decrement:
        current_patience = current_patience - 1
        return

    if current_patience <= 0:
        shared_config.emit_telemetry("[python];patience_timeout")

        datos_guardados = param_store.get_stored_data()
        current_cfg = {'CONFIG': CONFIG, 'LOG_MODE': SRP_LOG_MODE}

        num_previos = len(all_params)
        exito, linea_resumen, nuevos_comportamientos, winner_ids = GlobalTrajectoryReconstructor.run(
            "Qupa4", datos_guardados, current_cfg, num_previos
        )

        hay_comportamiento_nuevo = False
        logs_similitud = []
        comportamientos_ciclo = []

        if exito:
            param_store.set_done()
            if winner_ids:
                current_winning_ids = winner_ids
            else:
                current_winning_ids = {}

            # LÓGICA DE SIMILITUD Y REDUNDANCIA
            for idx_nuevo, nuevo_comportamiento in enumerate(nuevos_comportamientos, start=1):
                id_global_nuevo = num_previos + idx_nuevo
                num_seg_fisicos = contar_segmentos_distancia(nuevo_comportamiento)

                if num_seg_fisicos <= 2:
                    logs_similitud.append(
                        f" -> Comportamiento candidato [{id_global_nuevo}] ignorado: "
                        f"Eliminado debido a falta de información (Solo {num_seg_fisicos} segmentos)."
                    )
                else:
                    best_qi = -1.0
                    best_idx_conocido = None
                    best_info_alin = None

                    for idx_conocido, comportamiento_conocido in enumerate(all_params, start=1):
                        qi, info_alin = BehaviorSimilarity.evaluate(
                            comportamiento_conocido, nuevo_comportamiento, K_UMBRAL_SIMILITUD
                        )

                        if qi >= K_UMBRAL_SIMILITUD and qi > best_qi:
                            best_qi = qi
                            best_idx_conocido = idx_conocido
                            best_info_alin = info_alin

                    if best_idx_conocido is not None:
                        comportamiento_conocido = all_params[best_idx_conocido - 1]

                        BehaviorSimilarity.merge_behaviors(
                            comportamiento_conocido, nuevo_comportamiento, best_info_alin
                        )

                        match_type = (
                            "Alineación parcial (subsegmento)"
                            if best_info_alin.get('es_recorte')
                            else "Similitud estructural global"
                        )
                        method_name = BehaviorSimilarity.get_combine_method()

                        logs_similitud.append(
                            f" -> Historial del Comportamiento Conocido [{best_idx_conocido}] "
                            f"actualizado con el Candidato [{id_global_nuevo}]."
                        )
                        logs_similitud.append(
                            f"    Razón: {match_type} (Qi = {fmt_f(best_qi * 100, 2)}%). "
                            f"Fusión estadística mediante {method_name} =>"
                        )
                        logs_similitud.append("")
                        physical_merged = BehaviorSimilarity.translate_to_physical(comportamiento_conocido)
                        for seg_idx, seg in enumerate(physical_merged, start=1):
                            logs_similitud.append(
                                f"    Seg {seg_idx}: (Ang {fmt_f(seg[1], 2)}, Dist {fmt_f(seg[0], 2)})"
                            )
                        logs_similitud.append("")

                        comportamientos_ciclo.append(comportamiento_conocido)
                    else:
                        hay_comportamiento_nuevo = True

                        BehaviorSimilarity.init_history(nuevo_comportamiento)

                        all_params.append(nuevo_comportamiento)
                        id_diccionario = len(all_params)

                        logs_similitud.append(
                            f" -> COMPORTAMIENTO [{id_diccionario}]: NUEVO, AÑADIDO AL DICCIONARIO <<<"
                        )

                        comportamientos_ciclo.append(nuevo_comportamiento)

            es_fin_fase = (not hay_comportamiento_nuevo) and ((intentos_redundantes - 1) <= 0)
            if not es_fin_fase and callable(Print_tracker_calidad_imitacion_data) and len(comportamientos_ciclo) > 0:
                linea_resumen_actualizada = construir_linea_resumen("Qupa4", comportamientos_ciclo)
                Print_tracker_calidad_imitacion_data(linea_resumen_actualizada, is_master)
        else:
            current_winning_ids = {}

        # LÓGICA DE REDUCCIÓN DE INTENTOS Y FIN DE CICLO
        if hay_comportamiento_nuevo:
            if UP_LOG_MODE >= 1 or SRP_LOG_MODE >= 1:
                log("\n===========================================================")
                if UP_LOG_MODE == 2 and len(logs_similitud) > 0:
                    for msg in logs_similitud:
                        log(msg)
                    log("")
                    log("")
                log(" [FIN CICLO DE APRENDIZAJE] NUEVO COMPORTAMIENTO GUARDADO")
                log(" -> Intentos restantes N = " + str(intentos_redundantes))
                log(" Iniciando Random Walk para explorar nuevas áreas.")
                log("===========================================================\n\n\n")
            c4 = 1
        else:
            intentos_redundantes = intentos_redundantes - 1
            if intentos_redundantes > 0:
                if UP_LOG_MODE >= 1 or SRP_LOG_MODE >= 1:
                    log("\n===========================================================")
                    if UP_LOG_MODE == 2 and len(logs_similitud) > 0:
                        for msg in logs_similitud:
                            log(msg)
                        log("")
                        log("")
                    log(" [FIN CICLO DE APRENDIZAJE] REDUNDANCIA O FALLO")
                    if exito:
                        log(" -> Todo era redundante o rechazado. Intentos restantes N = " + str(intentos_redundantes))
                    else:
                        log(" -> Reconstrucción fallida (Pura basura). Intentos restantes N = " + str(intentos_redundantes))
                    log(" Iniciando Random Walk para buscar mejor perspectiva.")
                    log("===========================================================\n\n\n")
                c4 = 1
            else:
                if UP_LOG_MODE >= 1 or SRP_LOG_MODE >= 1:
                    log("\n\n\n===========================================================")
                    if UP_LOG_MODE == 2 and len(logs_similitud) > 0:
                        for msg in logs_similitud:
                            log(msg)
                        log("")
                        log("")
                    log(" [FIN FASE DE APRENDIZAJE] INTENTOS AGOTADOS (N=0)")
                    log(" -> Intentos restantes N = " + str(intentos_redundantes))
                    log("===========================================================")

                log("\n===========================================================")
                log(" RESUMEN GLOBAL: COMPORTAMIENTOS APRENDIDOS (DICCIONARIO)  ")
                log("===========================================================")
                for idx, comportamiento in enumerate(all_params, start=1):
                    log(f"\n--> Comportamiento [{idx}]:")
                    if isinstance(comportamiento, (dict, list)):
                        if isinstance(comportamiento, dict):
                            log("    Vector: " + math_helper.fmt_struct(comportamiento))
                        physical_params = BehaviorSimilarity.translate_to_physical(comportamiento)
                        params_str = [f"{{D:{fmt_f(p[0], 2)}, A:{fmt_f(p[1], 2)}}}" for p in physical_params]
                        log("    Parámetros: " + ", ".join(params_str))
                    else:
                        log("    Parámetros: [Formato desconocido]")
                log("===========================================================\n")

                if callable(Print_tracker_calidad_imitacion_data) and len(all_params) > 0:
                    linea_resumen_final = construir_linea_resumen("Qupa4", all_params)
                    Print_tracker_calidad_imitacion_data(linea_resumen_final, is_master)

                intentos_redundantes = N_MAX_INTENTOS
                c4 = 3


# ============================================================================
# [SECCIÓN 4] FUNCIONES AUXILIARES DEL OBSERVADOR (TRACKING CRUDO)
# ============================================================================
def initialize_master_tracking(k, angle_deg, distance, ty, obs_ang_offset_in=None, x_mark_in=None, y_mark_in=None):
    angle_rad = angle_deg * (math.pi / 180.0)

    obs_ang_offset = obs_ang_offset_in if obs_ang_offset_in is not None else 0.0
    xm = x_mark_in if x_mark_in is not None else 0.0
    ym = y_mark_in if y_mark_in is not None else 0.0

    ang_total = angle_rad + obs_ang_offset
    x_rel, y_rel = math_helper.polarACartesianoPAR(distance, ang_total)

    x = x_rel + xm
    y = y_rel + ym

    master_tracking[k] = {
        'raw_x': [x],
        'raw_y': [y],
        'raw_type': [ty],
        'Pattern_start_detected': (ty == 1),
        'End_of_pattern_detected': (ty == 3),
        'processed': False,
        'segmentos': {'movement': [], 'type': [], 'valid': []},
        'params': {},
    }


def process_master_segmentation(k, track, x_original, y_original, ty):
    track['raw_x'].append(x_original)
    track['raw_y'].append(y_original)
    track['raw_type'].append(ty)

    if ty == 1:
        track['Pattern_start_detected'] = True
    if ty == 3:
        track['End_of_pattern_detected'] = True


def normalize_angle_rad(ang):
    while ang > math.pi:
        ang = ang - 2 * math.pi
    while ang <= -math.pi:
        ang = ang + 2 * math.pi
    return ang


# ============================================================================
# [SECCIÓN 5] PIPELINE DE PROCESAMIENTO (OBSERVACIÓN INDIVIDUAL)
# ============================================================================
def process_single_observation(k, track):
    if SRP_LOG_MODE == 1 or SRP_LOG_MODE == 3:
        log(" ........................................................ ")
        log(f" PROCESANDO OBSERVACIÓN DEL MAESTRO {k} (Datos totales: {len(track['raw_x'])})")
        log(" ........................................................ ")

    PointCloudSegmenter.evaluator_otsu = evaluator_otsu
    PointCloudSegmenter.Print_tracker_varianza = Print_tracker_varianza
    clean_mov, clean_type, clean_valid, p_start, p_end = PointCloudSegmenter.process(
        k, track['raw_x'], track['raw_y'], track['raw_type'], seg_cfg
    )

    track['segmentos']['movement'] = clean_mov
    track['segmentos']['type']     = clean_type
    track['segmentos']['valid']    = clean_valid

    if p_start:
        track['Pattern_start_detected']  = True
    if p_end:
        track['End_of_pattern_detected'] = True

    if track['segmentos']['movement'] and len(track['segmentos']['movement']) > 1:
        track['params'] = SegmentParameterizer.process_segments(
            track['segmentos']['movement'], track['segmentos']['valid']
        )

        current_cfg = {'CONFIG': CONFIG}
        r_level, r_count, history_decay_level = param_store.package_and_add(
            track['params'], track['segmentos']['valid'], current_cfg
        )

        if r_level and r_level > 0:
            update_patience(r_level, r_count, history_decay_level)
    else:
        if SRP_LOG_MODE == 1 or SRP_LOG_MODE == 3:
            log("No hay segmentos suficientes para procesar tras la limpieza.")


# ============================================================================
# [SECCIÓN 6] BUCLE PRINCIPAL LÓGICO DEL OBSERVADOR (TICK A TICK)
# ============================================================================
def procesar_datos_logicos_y_reconstruccion(datos_fisicos):
    if param_store.is_done():
        return {}, True

    # 1. Extraer los datos de la tabla recibida
    yellow_blobs          = datos_fisicos['yellow_blobs']
    green_blobs           = datos_fisicos['green_blobs']
    obs_ang_offset_global = datos_fisicos['obs_ang_offset_global']
    xm                    = datos_fisicos.get('x_mark', 0.0)
    ym                    = datos_fisicos.get('y_mark', 0.0)

    if callable(Print_tracker_python):
        Print_tracker_python(obs_ang_offset_global, xm, ym, yellow_blobs, green_blobs)

    # 2. Actualizar el contador de paciencia del tick (la reconstrucción se evalúa tras las Fases B, C, D, G y H)
    check_patience_and_reconstruct(True)

    updated_anterior_indices = {}
    matched_new_indices = {}
    fallos_maestros = {}
    count = 0

    # =========================================================================
    # FASE B: ACTUALIZACIÓN DE MAESTROS EXISTENTES
    # =========================================================================
    candidatos = []
    for j, new_data in enumerate(yellow_blobs):
        ang_total_blob = (new_data['angle'] * (math.pi / 180.0)) + obs_ang_offset_global
        bx_rel, by_rel = math_helper.polarACartesianoPAR(new_data['distance'], ang_total_blob)
        bx_global = bx_rel + xm
        by_global = by_rel + ym

        for k, anterior_data in srt_anterior_data.items():
            if anterior_data.get('angle') is not None and k in master_tracking and len(master_tracking[k]['raw_x']) > 0:
                num_pts = len(master_tracking[k]['raw_x'])
                mx_global = master_tracking[k]['raw_x'][num_pts - 1]
                my_global = master_tracking[k]['raw_y'][num_pts - 1]
                error_dist = math.sqrt((bx_global - mx_global) ** 2 + (by_global - my_global) ** 2)
                if error_dist <= distance_treshold:
                    candidatos.append({'blob_idx': j, 'master_idx': k, 'error': error_dist})

    candidatos.sort(key=lambda a: (round(a['error'], 9), a['master_idx'], a['blob_idx']))

    for cand in candidatos:
        j = cand['blob_idx']
        k = cand['master_idx']
        if j not in matched_new_indices and k not in updated_anterior_indices:
            srt_anterior_data[k]['angle'] = yellow_blobs[j]['angle']
            srt_anterior_data[k]['distance'] = yellow_blobs[j]['distance']
            srt_anterior_data[k]['type'] = yellow_blobs[j]['type']
            fallos_maestros.pop(k, None)
            updated_anterior_indices[k] = True
            matched_new_indices[j] = True
            count = count + 1

    for k, anterior_data in srt_anterior_data.items():
        if anterior_data.get('angle') is not None and k not in updated_anterior_indices and k in master_tracking and len(master_tracking[k]['raw_x']) > 0:
            num_pts = len(master_tracking[k]['raw_x'])
            mx_global = master_tracking[k]['raw_x'][num_pts - 1]
            my_global = master_tracking[k]['raw_y'][num_pts - 1]
            distancias_sobrantes = []
            for j, new_data in enumerate(yellow_blobs):
                if j not in matched_new_indices:
                    ang_total_blob = (new_data['angle'] * (math.pi / 180.0)) + obs_ang_offset_global
                    bx_rel, by_rel = math_helper.polarACartesianoPAR(new_data['distance'], ang_total_blob)
                    bx_global = bx_rel + xm
                    by_global = by_rel + ym
                    error_dist = math.sqrt((bx_global - mx_global) ** 2 + (by_global - my_global) ** 2)
                    distancias_sobrantes.append(f"{fmt_f(error_dist, 2)} cm")

            msg_razon = f"Ningún blob válido a menos de {fmt_f(distance_treshold, 2)} cm."
            if len(distancias_sobrantes) > 0:
                msg_razon = msg_razon + " Blobs sobrantes a: " + " | ".join(distancias_sobrantes)
            else:
                msg_razon = msg_razon + " No se detectaron blobs sobrantes."
            fallos_maestros[k] = msg_razon

    # =========================================================================
    # FASE C: CREAR NUEVOS MAESTROS
    # =========================================================================
    for j, new_data in enumerate(yellow_blobs):
        if j not in matched_new_indices:
            new_k = len(srt_anterior_data) + 1
            srt_anterior_data[new_k] = {
                'angle': new_data['angle'],
                'distance': new_data['distance'],
                'type': new_data.get('type') or 2,
            }
            updated_anterior_indices[new_k] = True
            initialize_master_tracking(
                new_k,
                new_data['angle'],
                new_data['distance'],
                srt_anterior_data[new_k]['type'],
                obs_ang_offset_global,
                xm,
                ym,
            )

    # =========================================================================
    # FASE D: PROCESAR BLOBS VERDES
    # =========================================================================
    for green_blob in green_blobs:
        ang_total_green = (green_blob['angle'] * (math.pi / 180.0)) + obs_ang_offset_global
        gbx_rel, gby_rel = math_helper.polarACartesianoPAR(green_blob['distance'], ang_total_green)
        gbx = gbx_rel + xm
        gby = gby_rel + ym

        for k, master_data in srt_anterior_data.items():
            if master_data.get('angle') is not None and k not in updated_anterior_indices and k in master_tracking and len(master_tracking[k]['raw_x']) > 0:
                num_pts = len(master_tracking[k]['raw_x'])
                mx = master_tracking[k]['raw_x'][num_pts - 1]
                my = master_tracking[k]['raw_y'][num_pts - 1]
                dist_error = math.sqrt((gbx - mx) ** 2 + (gby - my) ** 2)
                if dist_error <= distance_treshold:
                    srt_anterior_data[k]['type'] = 3
                    updated_anterior_indices[k] = True
                    master_tracking[k]['End_of_pattern_detected'] = True
                    break

    # =========================================================================
    # FASE G: Gestión de Pérdidas y Procesamiento de Rutas
    # =========================================================================
    for k in range(1, len(srt_anterior_data) + 1):
        track = master_tracking.get(k)
        is_lost = (k in srt_anterior_data and srt_anterior_data[k].get('angle') is not None and k not in updated_anterior_indices)
        is_finished = track and track['End_of_pattern_detected']
        if is_lost:
            if SRP_LOG_MODE == 1 or SRP_LOG_MODE == 3:
                razon = fallos_maestros.get(k) or "Sin blobs amarillos cerca."
                log(f"\n[!] Maestro {k} PERDIDO. Razón: {razon}")
        if (is_lost or is_finished) and track and not track['processed']:
            process_single_observation(k, track)
            track['processed'] = True
            srt_anterior_data[k]['angle'] = None
            srt_anterior_data[k]['distance'] = None
            srt_anterior_data[k]['type'] = None

    # =========================================================================
    # FASE H: Grabación Continua
    # =========================================================================
    for k in sorted(updated_anterior_indices.keys()):
        master_data = srt_anterior_data.get(k)
        track = master_tracking.get(k)
        if master_data and track and not track['processed']:
            current_distance = master_data['distance']
            current_angle_rad = master_data['angle'] * (math.pi / 180.0)
            ty = master_data['type']

            ang_total = current_angle_rad + obs_ang_offset_global
            x_new, y_new = math_helper.polarACartesianoPAR(current_distance, ang_total)
            x_original = x_new + xm
            y_original = y_new + ym

            if callable(Print_tracker_trayectory_data):
                Print_tracker_trayectory_data("trajectory", k, x_original / 100.0, y_original / 100.0, False)
            process_master_segmentation(k, track, x_original, y_original, ty)

    if callable(Print_tracker_paciencia):
        Print_tracker_paciencia(current_patience / float(ticks_per_second))

    if current_patience <= 0:
        check_patience_and_reconstruct(False)
        return updated_anterior_indices, True

    return updated_anterior_indices, False


class ApprenticeProcess:
    soft_reset_observer_logic              = staticmethod(soft_reset_observer_logic)
    ejecutar_limpieza_ciclo                = staticmethod(ejecutar_limpieza_ciclo)
    update_patience                        = staticmethod(update_patience)
    check_patience_and_reconstruct         = staticmethod(check_patience_and_reconstruct)
    initialize_master_tracking             = staticmethod(initialize_master_tracking)
    process_master_segmentation            = staticmethod(process_master_segmentation)
    normalize_angle_rad                    = staticmethod(normalize_angle_rad)
    process_single_observation             = staticmethod(process_single_observation)
    procesar_datos_logicos_y_reconstruccion = staticmethod(procesar_datos_logicos_y_reconstruccion)