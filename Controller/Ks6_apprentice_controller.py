#!/usr/bin/env python3
# Archivo: Ks6/Ks6_apprentice_controller.py
# Ubicación: ~/sim_ws_qupa/Ks6/Controller/Ks6_apprentice_controller.py
# Descripción:
#   Controlador principal y autónomo del Aprendiz / Observador (Qupa4) en ROS 2 Jazzy + Gazebo Harmonic.
#   Es la fuente única de verdad de las variables del aprendiz; propaga sus valores a
#   apprentice_process, apprentice_entries, shared_config, Move y Active Learning sin ser sobrescrito.
#   Equivalente 1:1 de Ks6/Controller/Ks6_apprentice_controller.lua.

import importlib
import json
import math
import os
import random
import sys
import time

# Configurar ruta local para encontrar módulos tanto en Ks6/Controller/ como en Ks6/Functions/
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.dirname(_THIS_DIR) if os.path.basename(_THIS_DIR) == "Controller" else _THIS_DIR
for _p in (_THIS_DIR, _PROJECT_ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

sys.modules["Controller.Ks6_apprentice_controller"] = sys.modules[__name__]
sys.modules["Ks6_apprentice_controller"] = sys.modules[__name__]


def _require_any(candidates):
    for name in candidates:
        try:
            return importlib.import_module(name)
        except ImportError:
            continue
    raise ImportError("No se pudo cargar ninguno de los módulos: " + ", ".join(candidates))


# ============================================================================
#                               VARIABLES GLOBALES
# ============================================================================
random.seed(time.time())

ticks_per_second                    = 5
max_time                            = 5000 * ticks_per_second
Distribucion_sensores_de_proximidad = [0, -45, 45, -90, 90, 180]
Num_cantidad_leds                   = 2
FOOTBOT_INTERWHEEL_DISTANCE         = 8.6

factor_vel                          = 0.6
speed_dem                           = 10.0 * factor_vel
speed_dem_g                         = 40.0

d_speed                             = 7.0     # cm/s (7.0 coincide con v_real calibrado en qupa_48; en modo 'estatico' no se mueve)
g_speed                             = 44

# Métricas del modelo de observación y paciencia
# Límites físicos de saturación del robot Qupa (Gazebo y Hardware Real) y configuración IMU (MPU6050)
MAX_LINEAR_SPEED_CM_S               = 8.0     # Velocidad lineal máxima: 8.0 cm/s (0.08 m/s)
MAX_ANGULAR_SPEED_RAD_S             = 0.776   # Velocidad angular máxima: 0.776 rad/s (~44.46 °/s)

# Configuración de IMU y v_real por robot (ROBOT_V_REAL) centralizada en Functions/move.py

distance_treshold                   = 20

# Parámetros completos del modelo de paciencia y exploración (propagados a apprentice_process.py)
MAX_PATIENCE                        = 60  * ticks_per_second  # Techo de paciencia en ticks (antes 120s)
BASE_RESTORE                        = 12  * ticks_per_second  # Restauración base Nivel 1 (antes 30s)
PATIENCE_LVL2_MULTIPLIER            = 2.0                     # Multiplicador de BASE_RESTORE en Nivel 2
DECAY_RATES = {
    1: 0.30,  # 30% de pérdida por cada vez que ESE fragmento hace Nivel 1 (antes 0.15)
    2: 0.25,  # 25% de pérdida por cada vez que ESE fragmento hace Nivel 2 (antes 0.10)
    3: 0.0,   # 0% de pérdida para Nivel 3 (fragmento nuevo, restaura full)
}
ENABLE_RANDOM_WALK                  = False                   # False: espera quieto TIEMPO_RW_SEG s; True: ejecuta RandomWalking
TIEMPO_RW_SEG                       = 30                      # Duración de espera / Random Walk en segundos (antes 60s)
N_MAX_INTENTOS                      = 3                       # 0 = 1 solo ciclo (aprenda o no); >=1 = intentos extra de exploración

# Métricas del modelo de ActiveLearning
# Selector del método de Active Learning:
#   "estatico"            (o 0) -> Estatico.py (observación pura, no se mueve)
#   "centroide_basic"     (o 1) -> Centroide_basic.py (centroide básico)
#   "centroide_threshold" (o 2) -> Centroide_threshold.py (centroide con umbral)
#   "centroide_pid"       (o 3) -> Centroide_PID.py (centroide con PID)
#   "campos_vectoriales"  (o 4) -> CamposVectoriales.py (campos vectoriales / fuerzas físicas)
ACTIVE_LEARNING_METHOD              = "Centroide_threshold"
back_threshold                      = 10
front_threshold                     = 15
max_tracking_distance               = 25
porcentaje                          = 0.5

# Métricas del módulo de ruido en cámara de blobs del aprendiz
noise_cfg = {
    'ENABLE_NOISE'           : False,
    'USE_DISTANCE_DEPENDENT' : True,
    'USE_OUTLIERS'           : False,
    'USE_BIAS'               : True,
    'NOISE_BASE_RADIUS'      : 0.000,
    'NOISE_SIGMAS'           : 2.0,
    'NOISE_FACTOR'           : 0.1,
    'OUTLIER_PROB'           : 0.01,
    'BIAS_OFFSET'            : 0.0,
}

# Métricas del módulo: Point_Cloud_Segmenter.py y Segment_Parameterizer.py
seg_cfg = {
    'distance_treshold'     : 15,
    'limite_maximo_ventana' : (30) / factor_vel,
    'tamano_subsampling'    : 15,
    'use_subsampling'       : False,
}

# Métricas del módulo de reconstrucción: Global_Trajectory_Reconstructor.py y Parameter_Storage.py
CONFIG = {
    'TOL_DIST': 15,
    'TOL_ANG' : 15,
}

# Banderas de loggeo
SRP_LOG_MODE        = 0
UP_LOG_MODE         = 2
AL_LOG_MODE         = 0
DEBUG_EVASION       = 0

# Importación de módulos compartidos y submódulos del aprendiz (reciben automáticamente las variables de arriba)
shared_config      = _require_any(["Functions.shared_config", "shared_config"])
math_helper        = _require_any(["Functions.math_helper", "math_helper"])
Sensors            = _require_any(["Functions.sensors", "sensors"])
Utils              = _require_any(["Functions.utils", "utils"])
Move               = _require_any(["Functions.move", "move"])
apprentice_process = _require_any(["Controller.apprentice_process", "apprentice_process"])
apprentice_entries = _require_any(["Controller.apprentice_entries", "apprentice_entries"])
BehaviorSimilarity = _require_any(["Functions.Observer.BehaviorSimilarity", "BehaviorSimilarity"])
_ACTIVE_LEARNING_MODULES = {
    "0":                   "Functions.Observer.Active_learning.Estatico",
    "estatico":            "Functions.Observer.Active_learning.Estatico",
    "static":              "Functions.Observer.Active_learning.Estatico",
    "1":                   "Functions.Observer.Active_learning.Centroide_basic",
    "centroide_basic":     "Functions.Observer.Active_learning.Centroide_basic",
    "basic":               "Functions.Observer.Active_learning.Centroide_basic",
    "2":                   "Functions.Observer.Active_learning.Centroide_threshold",
    "centroide_threshold": "Functions.Observer.Active_learning.Centroide_threshold",
    "threshold":           "Functions.Observer.Active_learning.Centroide_threshold",
    "3":                   "Functions.Observer.Active_learning.Centroide_PID",
    "centroide_pid":       "Functions.Observer.Active_learning.Centroide_PID",
    "pid":                 "Functions.Observer.Active_learning.Centroide_PID",
    "4":                   "Functions.Observer.Active_learning.CamposVectoriales",
    "campos_vectoriales":  "Functions.Observer.Active_learning.CamposVectoriales",
    "campos":              "Functions.Observer.Active_learning.CamposVectoriales",
}


def load_active_learning_module(method_key=None):
    key = str(ACTIVE_LEARNING_METHOD if method_key is None else method_key).strip().lower()
    mod_path = _ACTIVE_LEARNING_MODULES.get(key)
    if mod_path is None:
        valid = "estatico, centroide_basic, centroide_threshold, centroide_pid, campos_vectoriales (o 0..4)"
        raise ValueError(f"Método de Active Learning desconocido: '{key}'. Opciones válidas: {valid}")
    return importlib.import_module(mod_path)

shared_config.sync_from_controller(sys.modules[__name__])

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger
from qupa_msgs.msg import DetectionArray

try:
    from qupa_msgs.srv import LEDCommand
    HAS_QUPA_LED_SRV = True
except ImportError:
    LEDCommand = None
    HAS_QUPA_LED_SRV = False

try:
    from gz.transport13 import Node as GzNode
    from gz.msgs10.material_color_pb2 import MaterialColor
    HAS_GZ_TRANSPORT = True
except ImportError:
    HAS_GZ_TRANSPORT = False

# Contexto activo del nodo ROS 2 para funciones globales espejo de Lua
_active_node = None
_active_app = None


def log(msg):
    apprentice_process.log_py(msg)


def setleds(color: str):
    if _active_node is not None and _active_app is not None:
        _active_node._set_led(_active_app, color)


# ============================================================================
#                                MAIN FUNCTIONS
# ============================================================================
# Funciones para los trackers (equivalentes 1:1 de Ks6_apprentice_controller.lua)

def Print_tracker_python(obs_ang_offset_global, x_mark, y_mark, yellow_blobs, green_blobs):
    if _active_node is not None and not _active_node.tracker_printer.enable_print_python:
        return
    yb_strs = []
    if yellow_blobs:
        for b in yellow_blobs:
            yb_strs.append(f"{b['angle']}:{b['distance']}:{b.get('type', 2)}")
    gb_strs = []
    if green_blobs:
        for b in green_blobs:
            gb_strs.append(f"{b['angle']}:{b['distance']}")
    py_str = (
        f"[python];observation;{obs_ang_offset_global};{x_mark};{y_mark};"
        + "|".join(yb_strs)
        + ";"
        + "|".join(gb_strs)
    )
    shared_config.emit_telemetry(py_str)


def Print_tracker_paciencia(current_patience_seg):
    if _active_node is not None and not _active_node.tracker_printer.enable_print_paciencia:
        return
    shared_config.emit_telemetry(f"[PACIENCIA]({current_patience_seg}) segundos.")


def Print_tracker_trayectory_data(p1, p2=None, p3=None, p4=None, p5=None, p6=None, p7=None, p8=None):
    if _active_node is not None and not _active_node.tracker_printer.enable_print_trayectory:
        return
    rx = _active_app.x if _active_app is not None else 0.0
    ry = _active_app.y if _active_app is not None else 0.0

    if not isinstance(p1, str) or (p1 != "trajectory" and p1 != "Active_learning"):
        k, x, y, logging_flag = p1, p2, p3, p4
        if isinstance(k, (int, float)):
            shared_config.emit_telemetry(f"Master{int(k)} & {x} , {y}")
            if logging_flag:
                log(f"Master{int(k)} & {x} , {y}")
        elif isinstance(k, str):
            if k == "Qupa4":
                shared_config.emit_telemetry(f"Apprentice & {rx} , {ry}")
            elif k == "App":
                shared_config.emit_telemetry(f"Patrón{master_indice or ''} & {rx} , {ry}")
            else:
                shared_config.emit_telemetry(f"{k} & {rx} , {ry}")
        return

    if p1 == "trajectory":
        k, x, y, logging_flag = p2, p3, p4, p5
        if isinstance(k, (int, float)):
            shared_config.emit_telemetry(f"[trajectory];Master{int(k)};{x};{y}")
        elif isinstance(k, str):
            if k == "Qupa4":
                shared_config.emit_telemetry(f"[trajectory];Apprentice;{rx};{ry}")
            elif k == "App":
                shared_config.emit_telemetry(f"[trajectory];Patrón{master_indice or ''};{rx};{ry}")
            else:
                shared_config.emit_telemetry(f"[trajectory];{k};{rx};{ry}")
    elif p1 == "Active_learning":
        al_mode, name, x, y, extra1, extra2, extra3 = p2, p3, p4, p5, p6, p7, p8
        if isinstance(name, str) and name == "Qupa4":
            x = rx
            y = ry
            name = "Apprentice"
        out = f"[active_learning];{al_mode};{name};{x};{y}"
        if extra1 is not None:
            out += f";{extra1}"
        if extra2 is not None:
            out += f";{extra2}"
        if extra3 is not None:
            out += f";{extra3}"
        shared_config.emit_telemetry(out)


def Print_tracker_AL_PID(target, actual, control_vars):
    if _active_node is not None and not _active_node.tracker_printer.enable_print_al_pid:
        return
    s = f"[PID];{target:.4f};{actual:.4f}"
    for name, value in (control_vars or {}).items():
        s += f";{name}={value:.4f}"
    shared_config.emit_telemetry(s)


def Print_tracker_calidad_imitacion_data(linea_resumen, is_master=0):
    if _active_node is not None and not _active_node.tracker_printer.enable_print_calidad:
        return
    if is_master == 0:
        shared_config.emit_telemetry(linea_resumen, is_init=False)


def Print_tracker_varianza(k, var):
    if _active_node is not None and not _active_node.tracker_printer.enable_print_varianza:
        return
    shared_config.emit_telemetry(f"Maestro{k} & {var}")


def evaluator_otsu(separacion, avance):
    if _active_node is not None and not _active_node.tracker_printer.enable_eval_otsu:
        return
    shared_config.emit_telemetry(f"Otsu & {separacion} , {avance}")


def evaluator_ruido(raw_angle, raw_dist, noisy_angle, noisy_dist):
    if _active_node is not None and not _active_node.tracker_printer.enable_eval_ruido:
        return
    shared_config.emit_telemetry(f"[EVALUATOR_RUIDO]Blob : angulo = {raw_angle}, distancia = {raw_dist}")
    shared_config.emit_telemetry(f"[EVALUATOR_RUIDO]Blob ruidoso : angulo = {noisy_angle}, distancia = {noisy_dist}")


# ============================================================================
#                EJECUCIÓN DE PATRONES APRENDIDOS TRAS FINALIZAR
# ============================================================================
d1              = 2
master_indice   = 0
indice          = 1
PI_end          = False
PI_counter_seg  = 0
parametros_elegidos = []


def Ejecutar_comportamiento_aprendido(
    params,
    app=None,
    timer_tick: int = 1,
    set_led_fn=None,
    tracker_printer=None,
):
    global indice, PI_end, PI_counter_seg
    rob = app if app is not None else _active_app
    if rob is None:
        return

    if not PI_end:
        if params and len(params) >= 1:
            if indice <= len(params):
                valor1, valor2 = params[indice - 1][0], params[indice - 1][1]
                Move.move(
                    rob,
                    valor1,
                    valor2,
                    float(speed_dem),
                    float(speed_dem_g),
                    timer_tick,
                    False,
                )

                if tracker_printer is not None:
                    tracker_printer.Print_tracker_trayectory_data("App", 0, 0)
                elif Print_tracker_trayectory_data is not None:
                    Print_tracker_trayectory_data("App", 0, 0)

                if callable(set_led_fn):
                    set_led_fn(rob, "yellow")
                else:
                    setleds("yellow")

                if rob.c1 == 2:
                    PI_counter_seg = PI_counter_seg + 1
                    rob.PI_counter_seg = PI_counter_seg
                    if indice == len(params):
                        indice = indice + 1
                        rob.indice = indice
                        PI_end = True
                        rob.PI_end = True
                        Utils.r_Temporizador_1(rob)
                    else:
                        indice = indice + 1
                        rob.indice = indice
                        rob.c1 = 0
        else:
            PI_end = True
            rob.PI_end = True
            if callable(set_led_fn):
                set_led_fn(rob, "black")
            else:
                setleds("black")
            log(" -------------------------------------------------------------------------")
            log(f"Patrón no detectado :( {rob.id}")
            log(" -------------------------------------------------------------------------")

    if PI_end:
        Move.set_wheels_cm_s(rob, 0.0, 0.0, FOOTBOT_INTERWHEEL_DISTANCE)
        if callable(set_led_fn):
            set_led_fn(rob, "black")
        else:
            setleds("black")


def demostrar_patrones_aprendidos(
    app=None,
    timer_tick: int = 1,
    set_led_fn=None,
    tracker_printer=None,
):
    global d1, master_indice, parametros_elegidos, indice, PI_end, PI_counter_seg
    rob = app if app is not None else _active_app
    if rob is None:
        return

    if d1 == 2:
        if len(apprentice_process.all_params) > 0:
            if master_indice == len(apprentice_process.all_params):
                d1 = 4
                rob.d1 = 4
                return
            master_indice = master_indice + 1
            rob.master_indice = master_indice
            parametros_elegidos = BehaviorSimilarity.translate_to_physical(
                apprentice_process.all_params[master_indice - 1]
            )
            rob.parametros_elegidos = parametros_elegidos
        else:
            log(f"No se detectaron comportamientos en la etapa de observación, {rob.id}")
            d1 = 4
            rob.d1 = 4
            return
        d1 = 3
        rob.d1 = 3

    elif d1 == 3:
        Ejecutar_comportamiento_aprendido(
            parametros_elegidos,
            app=rob,
            timer_tick=timer_tick,
            set_led_fn=set_led_fn,
            tracker_printer=tracker_printer,
        )
        if PI_end:
            rob.posiciones_guardadas = {}
            indice               = 1
            rob.indice           = 1
            PI_end               = False
            rob.PI_end           = False
            PI_counter_seg       = 0
            rob.PI_counter_seg   = 0
            Utils.r_Temporizador_1(rob)
            Utils.r_Temporizador_2(rob)
            Move.r_move(rob)
            d1 = 2
            rob.d1 = 2

    elif d1 == 4:
        Move.set_wheels_cm_s(rob, 0.0, 0.0, FOOTBOT_INTERWHEEL_DISTANCE)
        if callable(set_led_fn):
            set_led_fn(rob, "black")
        else:
            setleds("black")
        shared_config.emit_telemetry("End_simulation")


# ============================================================================
#                     MÁQUINA DE ESTADOS DEL APRENDIZ
# ============================================================================
def apprentice(
    app=None,
    robots: dict = None,
    timer_tick: int = 1,
    max_fov_radius_cm: float = 0.0,
    active_learning=None,
    set_led_fn=None,
    tracker_printer=None,
):
    """Equivalente 1:1 de apprentice() en Ks6_apprentice_controller.lua."""
    global d1, master_indice, indice, PI_end, PI_counter_seg
    rob = app if app is not None else _active_app

    # === ESTADO DE APRENDIZAJE ===
    if apprentice_process.c4 == 0:
        apprentice_entries.store_relative_positions(
            rob,
            robots,
            timer_tick,
            max_fov_radius_cm,
            active_learning,
            set_led_fn=set_led_fn,
            tracker_printer=tracker_printer,
        )

    # === ESTADOS DE EXPLORACIÓN (RANDOM WALK) ===
    elif apprentice_process.c4 == 1:
        apprentice_process.AP_1 = 1
        apprentice_process.rw_timer += 1
        rob.rw_timer = apprentice_process.rw_timer
        if tracker_printer is not None:
            tracker_printer.Print_tracker_trayectory_data("trajectory", "Qupa4", 0, 0)
        elif Print_tracker_trayectory_data is not None:
            Print_tracker_trayectory_data("trajectory", "Qupa4", 0, 0)

        if ENABLE_RANDOM_WALK:
            Move.RandomWalking(rob, timer_tick)
        else:
            Move.set_wheels_cm_s(rob, 0.0, 0.0, FOOTBOT_INTERWHEEL_DISTANCE)

        if apprentice_process.rw_timer >= apprentice_process.TIEMPO_RW_TICKS:
            Move.set_wheels_cm_s(rob, 0.0, 0.0, FOOTBOT_INTERWHEEL_DISTANCE)
            Move.r_move(rob)
            Utils.r_Temporizador_1(rob)
            Move.r_move(rob)
            apprentice_process.rw_timer = 0
            rob.rw_timer = 0
            apprentice_process.c4 = 2

    elif apprentice_process.c4 == 2:
        apprentice_process.ejecutar_limpieza_ciclo()
        apprentice_entries.soft_reset_observer(rob, set_led_fn=set_led_fn)

        current_x = rob.x
        current_y = rob.y
        ang       = rob.yaw

        if tracker_printer is not None and tracker_printer.enable_print_trayectory:
            shared_config.emit_telemetry(f"[trajectory];[offset];{current_x};{current_y};{ang}")

        apprentice_process.c4 = 0

    # === ESTADOS DE EJECUCIÓN DE COMPORTAMIENTOS APRENDIDOS ===
    elif apprentice_process.c4 == 3:
        NAC = Move.navegar_a_coordenadas(
            rob,
            0.0,
            0.0,
            0.10,
            timer_tick,
            ticks_per_second=ticks_per_second,
            debug_evasion=DEBUG_EVASION,
            log_fn=apprentice_process.log_py,
        )
        if NAC:
            apprentice_process.c4 = 4

    elif apprentice_process.c4 == 4:
        apprentice_entries.resetOBS(rob, set_led_fn=set_led_fn)
        d1             = 2
        master_indice  = 0
        indice         = 1
        PI_end         = False
        PI_counter_seg = 0
        rob.d1 = 2
        rob.master_indice = 0
        rob.indice = 1
        rob.PI_end = False
        rob.PI_counter_seg = 0
        apprentice_process.c4 = 5

    elif apprentice_process.c4 == 5:
        demostrar_patrones_aprendidos(
            app=rob,
            timer_tick=timer_tick,
            set_led_fn=set_led_fn,
            tracker_printer=tracker_printer,
        )


# ============================================================================
#                        CICLO DE VIDA ARGoS3 (init, step)
# ============================================================================
timer = 1


class Ks6ApprenticeControllerNode(Node):
    def __init__(self):
        global _active_node, _active_app
        cli_use_sim_time = False
        for idx, arg in enumerate(sys.argv):
            if arg.startswith('use_sim_time:='):
                cli_use_sim_time = arg.split(':=', 1)[1].strip().lower() in ('true', '1', 'yes')
            elif arg in ('-p', '--param') and idx + 1 < len(sys.argv) and sys.argv[idx + 1].startswith('use_sim_time:='):
                cli_use_sim_time = sys.argv[idx + 1].split(':=', 1)[1].strip().lower() in ('true', '1', 'yes')
        super().__init__(
            'ks6_apprentice_controller_node',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, cli_use_sim_time)],
        )
        self.use_sim_time_active = cli_use_sim_time
        _active_node = self

        self.declare_parameter('config_file', '/tmp/ks6_robots_config.json')
        self.declare_parameter('robot_id', 'Qupa4')
        self.declare_parameter('role', 'apprentice')
        self.declare_parameter('init_x', -1.20)
        self.declare_parameter('init_y', -1.20)
        self.declare_parameter('init_yaw', 0.0)
        self.declare_parameter('camera_aperture_deg', 72.1)
        self.declare_parameter('camera_height_m', 0.245)

        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        param_robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        param_role = self.get_parameter('role').get_parameter_value().string_value
        param_x = self.get_parameter('init_x').get_parameter_value().double_value
        param_y = self.get_parameter('init_y').get_parameter_value().double_value
        param_yaw = self.get_parameter('init_yaw').get_parameter_value().double_value
        aperture_deg = self.get_parameter('camera_aperture_deg').get_parameter_value().double_value
        cam_height_m = self.get_parameter('camera_height_m').get_parameter_value().double_value

        # self.max_fov_radius_cm = math.tan(math.radians(aperture_deg)) * cam_height_m * 100.0
        self.max_fov_radius_cm = 70

        injected_vars = {}
        active_funcs = None
        apprentice_entry = None

        if config_file and os.path.exists(config_file):
            try:
                with open(config_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                for r in data.get('robots', []):
                    r_id_str = str(r.get('id', ''))
                    r_role_str = str(r.get('role', ''))
                    if (param_robot_id and r_id_str == param_robot_id) or r_role_str.lower() == 'apprentice':
                        apprentice_entry = r
                        break
                injected_vars = data.get('vars', {}) or {}
                if 'active_functions' in data:
                    active_funcs = set(data.get('active_functions', []))
            except Exception:
                apprentice_entry = None

        # Primero empujamos las variables definidas en Ks6_apprentice_controller hacia los submódulos
        # y luego aplicamos únicamente las claves de `--vars` (injected_vars) si las hubiera.
        _push_globals_to_process()
        apprentice_process.apply_injected_vars(injected_vars)
        _sync_globals_from_process()

        if apprentice_entry is not None:
            r_id = str(apprentice_entry.get('id', param_robot_id or 'Qupa4'))
            rx = float(apprentice_entry.get('x', param_x))
            ry = float(apprentice_entry.get('y', param_y))
            ryaw = float(apprentice_entry.get('yaw', param_yaw))
            fig = str(apprentice_entry.get('figura', ''))
            role = str(apprentice_entry.get('role', param_role or 'apprentice'))
        else:
            r_id = param_robot_id or "Qupa4"
            rx = param_x
            ry = param_y
            ryaw = param_yaw
            fig = ""
            role = param_role or "apprentice"

        self.robot_id = r_id

        qos_init = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        qos_stream = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.telemetry_init_pub = self.create_publisher(
            String, shared_config.TELEMETRY_INIT_TOPIC, qos_init
        )
        self.telemetry_pub = self.create_publisher(
            String, shared_config.TELEMETRY_TOPIC, qos_stream
        )
        shared_config.set_telemetry_emitters(
            telemetry_fn=self._publish_telemetry,
            telemetry_init_fn=self._publish_telemetry_init,
        )

        self.gz_mat_pub = None
        if HAS_GZ_TRANSPORT:
            try:
                self.gz_node = GzNode()
                self.gz_mat_pub = self.gz_node.advertise("/world/qupa_world/material_color", MaterialColor)
            except Exception:
                self.gz_mat_pub = None

        self.rob = Utils.QupaRobotInstance(
            self.robot_id,
            rx,
            ry,
            ryaw,
            fig,
            role,
            dist_sens=Distribucion_sensores_de_proximidad,
            d_speed=d_speed,
            g_speed=g_speed,
            ticks_per_second=ticks_per_second,
            interwheel_distance=FOOTBOT_INTERWHEEL_DISTANCE,
        )
        _active_app = self.rob
        self.robots = {self.robot_id: self.rob}

        # --- CONFIGURACIÓN DE PUBLICADORES Y SUSCRIPCIONES DEL NODO APRENDIZ ---
        # Publicador de velocidad diferencial (Twist saturado a 0.08 m/s y 0.776 rad/s)
        self.cmd_vel_pub = self.create_publisher(Twist, f'/{self.robot_id}/cmd_vel', 10)
        self.led_pub = self.create_publisher(String, f'/{self.robot_id}/leds/color', 10)
        self.led_srv_client = None
        if HAS_QUPA_LED_SRV:
            self.led_srv_client = self.create_client(LEDCommand, f'/{self.robot_id}/set')

        self.create_subscription(
            Odometry,
            f'/{self.robot_id}/odom',
            self._odom_cb,
            10,
        )
        # 1) Suscripción estricta a proximidad por LaserScan (/{robot_id}/scan, sin ray-tracing)
        self.create_subscription(
            LaserScan,
            f'/{self.robot_id}/scan',
            self._scan_cb,
            10,
        )
        # 2) Suscripción estricta a cámara omnidireccional del aprendiz (/{robot_id}/camera/detections)
        self.create_subscription(
            DetectionArray,
            f'/{self.robot_id}/camera/detections',
            self._detections_cb,
            10,
        )
        # 3) Suscripción al IMU MPU6050 (/{robot_id}/mpu6050/imu/data_raw) y cliente de /tare
        self.create_subscription(
            Imu,
            f'/{self.robot_id}/{Move.IMU_TOPIC_SUFFIX}',
            self._imu_cb,
            50,
        )
        self.imu_tare_client = self.create_client(
            Trigger,
            f'/{self.robot_id}/{Move.IMU_TARE_SERVICE_SUFFIX}',
        )
        self.rob.imu_tare_client = self.imu_tare_client

        self.tracker_printer = Utils.TrackerPrinter(active_funcs, self.robots)
        apprentice_process.Print_tracker_python = (
            Print_tracker_python
            if self.tracker_printer.enable_print_python
            else None
        )
        apprentice_process.Print_tracker_paciencia = (
            Print_tracker_paciencia
            if self.tracker_printer.enable_print_paciencia
            else None
        )
        apprentice_process.Print_tracker_trayectory_data = (
            Print_tracker_trayectory_data
            if self.tracker_printer.enable_print_trayectory
            else None
        )
        apprentice_process.Print_tracker_calidad_imitacion_data = (
            Print_tracker_calidad_imitacion_data
            if self.tracker_printer.enable_print_calidad
            else None
        )
        apprentice_process.Print_tracker_varianza = (
            Print_tracker_varianza
            if self.tracker_printer.enable_print_varianza
            else None
        )
        apprentice_process.evaluator_otsu = (
            evaluator_otsu
            if self.tracker_printer.enable_eval_otsu
            else None
        )
        apprentice_process.is_master = 0

        al_mod = load_active_learning_module(ACTIVE_LEARNING_METHOD)
        self.active_learning = al_mod.ActiveLearning()
        apprentice_entries.ActiveLearning = al_mod
        self.get_logger().info(
            f"[ActiveLearning] Método activo: '{ACTIVE_LEARNING_METHOD}' ({al_mod.__name__})"
        )
        self.timer_tick = 1
        self.initialized = False
        self.start_wall_time = time.time()

        self.step_timer = self.create_timer(
            1.0 / float(ticks_per_second), self.step
        )

    def _publish_telemetry_init(self, text: str):
        msg = String()
        msg.data = str(text)
        self.telemetry_init_pub.publish(msg)

    def _publish_telemetry(self, text: str):
        msg = String()
        msg.data = str(text)
        self.telemetry_pub.publish(msg)

    def _set_led(self, rob, color: str, force_topic_pub: bool = False):
        Utils.setleds(
            rob,
            color,
            led_pub=self.led_pub,
            gz_mat_pub=self.gz_mat_pub,
            force_topic_pub=force_topic_pub,
            led_srv_client=self.led_srv_client,
        )

    def stop_hardware(self):
        """Detiene motores y apaga la tira LED APA102 al cerrar el controlador."""
        try:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            self._set_led(self.rob, "black", force_topic_pub=True)
        except Exception:
            pass

    def init(self):
        rob = self.rob
        rob.determinar_rol(apprentice_process.log_py)

        ang_deg = math.degrees(rob.yaw)
        if self.tracker_printer.enable_print_trayectory:
            shared_config.emit_telemetry(
                f"[trajectory];[offset];{rob.x};{rob.y};{ang_deg}", is_init=True
            )
        if self.tracker_printer.enable_print_paciencia:
            shared_config.emit_telemetry(
                f"[PACIENCIA]({apprentice_process.MAX_PATIENCE / apprentice_process.ticks_per_second})"
                f"&{apprentice_process.ticks_per_second}",
                is_init=True,
            )
        if self.tracker_printer.enable_print_varianza and apprentice_process.seg_cfg.get('use_subsampling'):
            shared_config.emit_telemetry(
                f"[varianza](tamano_subsampling={apprentice_process.seg_cfg.get('tamano_subsampling')}",
                is_init=True,
            )

    def step(self):
        global timer, _active_app
        if not self.initialized:
            if self.use_sim_time_active and not self.rob.odom_received and (time.time() - self.start_wall_time) < 8.0:
                return
            self.initialized = True
            self.init()

        timer = self.timer_tick
        if (
            self.tracker_printer.enable_eval_ruido
            or self.tracker_printer.enable_print_trayectory
            or self.tracker_printer.enable_print_paciencia
            or self.tracker_printer.enable_print_al_pid
            or self.tracker_printer.enable_print_python
        ):
            shared_config.emit_telemetry(f"Timer={self.timer_tick}")

        _active_app = self.rob
        apprentice(
            self.rob,
            None,
            self.timer_tick,
            self.max_fov_radius_cm,
            self.active_learning,
            set_led_fn=self._set_led,
            tracker_printer=self.tracker_printer,
        )

        twist = Twist()
        # Saturación estricta antes de publicar en /{robot_id}/cmd_vel (8 cm/s y 0.776 rad/s)
        lin_cmd, ang_cmd = Move.clamp_cmd_vel(self.rob.cmd_linear_x, self.rob.cmd_angular_z)
        twist.linear.x = float(lin_cmd)
        twist.angular.z = float(ang_cmd)
        self.cmd_vel_pub.publish(twist)

        if self.timer_tick >= max_time:
            shared_config.emit_telemetry("End_simulation")

        self.timer_tick += 1
        timer = self.timer_tick

    def reset(self):
        pass

    def destroy(self):
        self.stop_hardware()

    def _scan_cb(self, msg: LaserScan):
        Sensors.process_laser_scan(self.rob, msg)

    def _imu_cb(self, msg: Imu):
        Move.update_imu(self.rob, msg, self.get_clock().now().nanoseconds * 1e-9)

    def _detections_cb(self, msg: DetectionArray):
        apprentice_entries.update_camera_detections(self.rob, msg)

    def _odom_cb(self, msg: Odometry):
        rob = self.rob
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        raw_yaw = math.atan2(siny_cosp, cosy_cosp)

        if not rob.odom_received:
            rob.odom_received = True
            if abs(pos.x) < 0.05 and abs(pos.y) < 0.05 and (abs(rob.init_x) > 0.05 or abs(rob.init_y) > 0.05):
                rob.odom_origin_pos = (rob.init_x, rob.init_y)
                rob.odom_origin_yaw = rob.init_yaw - raw_yaw
            else:
                rob.odom_origin_pos = (0.0, 0.0)
                rob.odom_origin_yaw = 0.0

        cos_o = math.cos(rob.odom_origin_yaw)
        sin_o = math.sin(rob.odom_origin_yaw)
        rob.x = rob.odom_origin_pos[0] + (pos.x * cos_o - pos.y * sin_o)
        rob.y = rob.odom_origin_pos[1] + (pos.x * sin_o + pos.y * cos_o)
        rob.yaw = Utils.norm_rad(raw_yaw + rob.odom_origin_yaw)


Ks6ApprenticeControllerNode = Ks6ApprenticeControllerNode


def _push_globals_to_process():
    """Propaga las variables globales definidas en Ks6_apprentice_controller hacia los submódulos."""
    shared_config.sync_from_controller(sys.modules[__name__])
    apprentice_process.DECAY_RATES = dict(DECAY_RATES)
    apprentice_process.PATIENCE_LVL2_MULTIPLIER = float(PATIENCE_LVL2_MULTIPLIER)
    apprentice_process.ENABLE_RANDOM_WALK = bool(ENABLE_RANDOM_WALK)
    apprentice_process.TIEMPO_RW_SEG = float(TIEMPO_RW_SEG)
    apprentice_process.TIEMPO_RW_TICKS = int(TIEMPO_RW_SEG * ticks_per_second)
    apprentice_process.N_MAX_INTENTOS = int(N_MAX_INTENTOS)
    apprentice_process.intentos_redundantes = int(N_MAX_INTENTOS)
    apprentice_process.current_patience = int(MAX_PATIENCE)
    apprentice_entries.MAX_WAIT_TICKS = int(2 * ticks_per_second)


def _sync_globals_from_process():
    global ticks_per_second, max_time, Distribucion_sensores_de_proximidad
    global Num_cantidad_leds, FOOTBOT_INTERWHEEL_DISTANCE
    global factor_vel, speed_dem, speed_dem_g, d_speed, g_speed
    global distance_treshold, MAX_PATIENCE, BASE_RESTORE
    global PATIENCE_LVL2_MULTIPLIER, DECAY_RATES, ENABLE_RANDOM_WALK, TIEMPO_RW_SEG, N_MAX_INTENTOS
    global back_threshold, front_threshold, max_tracking_distance, porcentaje
    global SRP_LOG_MODE, UP_LOG_MODE, AL_LOG_MODE, DEBUG_EVASION

    ticks_per_second = apprentice_process.ticks_per_second
    max_time = apprentice_process.max_time
    Distribucion_sensores_de_proximidad = apprentice_process.Distribucion_sensores_de_proximidad
    Num_cantidad_leds = apprentice_process.Num_cantidad_leds
    FOOTBOT_INTERWHEEL_DISTANCE = apprentice_process.FOOTBOT_INTERWHEEL_DISTANCE
    factor_vel = apprentice_process.factor_vel
    speed_dem = apprentice_process.speed_dem
    speed_dem_g = apprentice_process.speed_dem_g
    d_speed = apprentice_process.d_speed
    g_speed = apprentice_process.g_speed
    distance_treshold = apprentice_process.distance_treshold
    MAX_PATIENCE = apprentice_process.MAX_PATIENCE
    BASE_RESTORE = apprentice_process.BASE_RESTORE
    PATIENCE_LVL2_MULTIPLIER = apprentice_process.PATIENCE_LVL2_MULTIPLIER
    DECAY_RATES = dict(apprentice_process.DECAY_RATES)
    ENABLE_RANDOM_WALK = bool(apprentice_process.ENABLE_RANDOM_WALK)
    TIEMPO_RW_SEG = apprentice_process.TIEMPO_RW_SEG
    N_MAX_INTENTOS = apprentice_process.N_MAX_INTENTOS
    back_threshold = apprentice_process.back_threshold
    front_threshold = apprentice_process.front_threshold
    max_tracking_distance = apprentice_process.max_tracking_distance
    porcentaje = apprentice_process.porcentaje
    SRP_LOG_MODE = apprentice_process.SRP_LOG_MODE
    UP_LOG_MODE = apprentice_process.UP_LOG_MODE
    AL_LOG_MODE = apprentice_process.AL_LOG_MODE
    DEBUG_EVASION = apprentice_process.DEBUG_EVASION


def init():
    if _active_node is not None:
        _active_node.init()


def step():
    if _active_node is not None:
        _active_node.step()


def reset():
    if _active_node is not None:
        _active_node.reset()


def destroy():
    if _active_node is not None:
        _active_node.destroy()


def main(args=None):
    rclpy.init(args=args)
    node = Ks6ApprenticeControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_hardware()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()