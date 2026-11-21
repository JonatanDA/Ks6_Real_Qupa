# Variante: Ks6 Real Qupa (sin Gazebo, sin /odom global, move lazo abierto por tiempo)
# Archivo: Ks6/Functions/shared_config.py
# Ubicación: ~/sim_ws_qupa/Ks6/Functions/shared_config.py
# Descripción:
#   Módulo de configuración y enlace de variables para Ks6.
#   Toma los valores definidos en el controlador principal activo (Ks6_master_controller
#   o Ks6_apprentice_controller) y solo usa valores de respaldo ("or default" estilo Lua)
#   cuando una variable no fue definida en el controlador principal.

import sys

sys.modules["Functions.shared_config"] = sys.modules[__name__]
sys.modules["shared_config"] = sys.modules[__name__]

_CONTROLLER_MODULE_NAMES = (
    "Ks6_apprentice_controller",
    "Controller.Ks6_apprentice_controller",
    "Ks6_master_controller",
    "Controller.Ks6_master_controller",
)


def get_active_controller_module():
    """Retorna el módulo del controlador principal activo en este proceso (Maestro o Aprendiz)."""
    for mod_name in _CONTROLLER_MODULE_NAMES:
        mod = sys.modules.get(mod_name)
        if mod is not None:
            return mod
    return None


def get_controller_var(name: str, default=None):
    """
    Equivalente en Python al patrón de Lua `var = var or default`, pero respetando
    valores numéricos cero (0, 0.0) y booleanos (False) definidos en el controlador.
    """
    ctrl = get_active_controller_module()
    if ctrl is not None and hasattr(ctrl, name):
        val = getattr(ctrl, name)
        if val is not None:
            return val
    return default


# ============================================================================
# 1. CONSTANTES FÍSICAS Y DE SIMULACIÓN (Heredadas del Controlador Principal)
# ============================================================================
ticks_per_second                    = get_controller_var("ticks_per_second", 4)
max_time                            = get_controller_var("max_time", 5000 * ticks_per_second)
Distribucion_sensores_de_proximidad = get_controller_var("Distribucion_sensores_de_proximidad", [0, -45, 45, -90, 90, 180])
Num_cantidad_leds                   = get_controller_var("Num_cantidad_leds", 2)
FOOTBOT_INTERWHEEL_DISTANCE         = get_controller_var("FOOTBOT_INTERWHEEL_DISTANCE", 8.6)

factor_vel                          = get_controller_var("factor_vel", 1)
speed_dem                           = get_controller_var("speed_dem", 6.0 * factor_vel)
speed_dem_g                         = get_controller_var("speed_dem_g", 40.0)

d_speed                             = get_controller_var("d_speed", 8.0)
g_speed                             = get_controller_var("g_speed", 44.46)


# Límites físicos de saturación del robot Qupa (Gazebo y Hardware Real)
MAX_LINEAR_SPEED_CM_S               = get_controller_var("MAX_LINEAR_SPEED_CM_S", 8.0)       # 0.08 m/s
MAX_ANGULAR_SPEED_RAD_S             = get_controller_var("MAX_ANGULAR_SPEED_RAD_S", 0.776)   # ~44.46 deg/s
distance_factor                     = get_controller_var("distance_factor", 1)
SMART_MASTERS                       = get_controller_var("SMART_MASTERS", True)

# ============================================================================
# 2. TÓPICOS ROS 2 DE TELEMETRÍA Y SINCRONIZACIÓN (Ks6)
# ============================================================================
TELEMETRY_INIT_TOPIC = "/ks6/telemetry_init"
TELEMETRY_TOPIC      = "/ks6/telemetry"
TRACKER_READY_TOPIC  = "/ks6/tracker_ready"
MASTER_READY_TOPIC   = "/ks6/master_ready"

_telemetry_emit_fn      = None
_telemetry_init_emit_fn = None

_SHARED_VAR_NAMES = (
    "ticks_per_second",
    "max_time",
    "Distribucion_sensores_de_proximidad",
    "Num_cantidad_leds",
    "FOOTBOT_INTERWHEEL_DISTANCE",
    "factor_vel",
    "speed_dem",
    "speed_dem_g",
    "d_speed",
    "g_speed",
    "MAX_LINEAR_SPEED_CM_S",
    "MAX_ANGULAR_SPEED_RAD_S",
    "distance_factor",
    "SMART_MASTERS",
    "distance_treshold",
    "MAX_PATIENCE",
    "BASE_RESTORE",
    "back_threshold",
    "front_threshold",
    "max_tracking_distance",
    "porcentaje",
    "noise_cfg",
    "seg_cfg",
    "CONFIG",
    "SRP_LOG_MODE",
    "UP_LOG_MODE",
    "AL_LOG_MODE",
    "DEBUG_EVASION",
)


def set_telemetry_emitters(telemetry_fn=None, telemetry_init_fn=None):
    """Registra los callbacks de publicación ROS 2 del nodo activo en este proceso."""
    global _telemetry_emit_fn, _telemetry_init_emit_fn
    _telemetry_emit_fn = telemetry_fn
    _telemetry_init_emit_fn = telemetry_init_fn


def emit_telemetry(line: str, is_init: bool = False):
    """
    Envía una línea de telemetría al tópico ROS 2 correspondiente (/ks6/telemetry_init
    o /ks6/telemetry) y la imprime en stdout para registro local.
    """
    if line is None:
        return
    text = str(line)
    print(text, flush=True)
    if is_init and callable(_telemetry_init_emit_fn):
        _telemetry_init_emit_fn(text)
    elif callable(_telemetry_emit_fn):
        _telemetry_emit_fn(text)


def log_py(msg):
    """Imprime y publica en telemetría mensajes con prefijo [PYTHON GESTOR]."""
    for subline in str(msg).split("\n"):
        emit_telemetry(f"[PYTHON GESTOR] {subline}", is_init=False)


log = log_py


def sync_from_controller(ctrl_module=None, extra_targets=None):
    """
    Propaga las variables definidas en el controlador principal hacia shared_config
    y hacia los submódulos cargados (move, apprentice_process, apprentice_entries, ActiveLearning),
    de modo que el controlador principal sea siempre la única fuente de verdad.
    """
    ctrl = ctrl_module or get_active_controller_module()
    if ctrl is None:
        return

    this_mod = sys.modules[__name__]
    targets = [this_mod]
    for mod_key in (
        "Functions.move",
        "move",
        "Controller.apprentice_process",
        "apprentice_process",
        "Controller.apprentice_entries",
        "apprentice_entries",
        "Functions.Observer.Active_learning.CamposVectoriales",
    ):
        m = sys.modules.get(mod_key)
        if m is not None and m not in targets:
            targets.append(m)

    for et in (extra_targets or []):
        if et is not None and et not in targets:
            targets.append(et)

    for var_name in _SHARED_VAR_NAMES:
        if hasattr(ctrl, var_name):
            val = getattr(ctrl, var_name)
            for tgt in targets:
                if tgt is not ctrl:
                    setattr(tgt, var_name, val)


def apply_injected_vars(vars_dict: dict, target_module=None):
    """
    Inyecta únicamente las variables explícitamente enviadas en `--vars` desde
    run_sim_ros2.py / multi_run_sim_ros2.py sobre el controlador principal y submódulos.
    Si `vars_dict` está vacío, conserva intactos todos los valores definidos en el controlador.
    """
    ctrl = target_module or get_active_controller_module()
    if not vars_dict:
        sync_from_controller(ctrl)
        return

    this_mod = sys.modules[__name__]
    base_fvel = float(getattr(ctrl, "factor_vel", getattr(this_mod, "factor_vel", 1.0)) or 1.0)
    base_sdem = float(getattr(ctrl, "speed_dem", getattr(this_mod, "speed_dem", 10.0)))
    sdem_per_fvel = (base_sdem / base_fvel) if abs(base_fvel) > 1e-9 else base_sdem

    for k, v in vars_dict.items():
        setattr(this_mod, k, v)
        if ctrl is not None:
            setattr(ctrl, k, v)

    target_ref = ctrl if ctrl is not None else this_mod
    new_fvel = float(getattr(target_ref, "factor_vel", 1.0))
    new_tps = float(getattr(target_ref, "ticks_per_second", 4.0))

    if "factor_vel" in vars_dict and "speed_dem" not in vars_dict:
        new_sdem = sdem_per_fvel * new_fvel
        setattr(target_ref, "speed_dem", new_sdem)
        if "speed_dem_g" not in vars_dict:
            setattr(target_ref, "speed_dem_g", new_sdem / 2.0)
    elif "speed_dem" in vars_dict and "speed_dem_g" not in vars_dict:
        setattr(target_ref, "speed_dem_g", float(vars_dict["speed_dem"]) / 2.0)

    if "ticks_per_second" in vars_dict and "max_time" not in vars_dict:
        setattr(target_ref, "max_time", int(5000 * new_tps))

    sync_from_controller(ctrl)
