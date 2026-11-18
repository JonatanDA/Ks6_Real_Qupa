#!/usr/bin/env python3
# Archivo: scripts/run_sim_ros2_p0.py
# Ubicación: ~/sim_ws_qupa/Ks7/scripts/run_sim_ros2_p0.py
# Descripción: Lanza el controlador individual (Maestro o Aprendiz) en un robot físico real.
# Uso: Ejecutar directamente en la Raspberry Pi del robot.
# Ejemplo: python3 scripts/run_sim_ros2_p0.py --id Qupa4 --role apprentice --x 0.0 --y 0.0

import os
import sys
import subprocess
import time
import json
import argparse

_THIS_FILE_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT_DIR = os.path.dirname(_THIS_FILE_DIR) if os.path.basename(_THIS_FILE_DIR) == "scripts" else _THIS_FILE_DIR
CONFIG_JSON_PATH = f"/tmp/ks7_robot_config_p0_{os.getpid()}.json"

# =============================================================================
# ---------- 1. DICCIONARIO DE EVALUADORES Y DEPENDENCIAS ----------
# =============================================================================
EVALUATOR_DEPENDENCIES = {
    "scripts/evaluator_qi_noise.py": {"lua_functions": ["evaluator_ruido", "Print_tracker_calidad_imitacion_data"], "lua_variables": [], "run_sim_trackers": ["scripts/tracker_calidad_imitacion.py"]},
    "scripts/evaluator_otsu.py": {"lua_functions": ["evaluator_otsu"], "lua_variables": [], "run_sim_trackers": []},
    "scripts/evaluator_qi_speed_noise.py": {"lua_functions": ["evaluator_ruido"], "lua_variables": [], "run_sim_trackers": ["scripts/tracker_calidad_imitacion.py"]}
}

# =============================================================================
# ---------- 2. DICCIONARIO DE TRACKERS Y DEPENDENCIAS LUA ----------
# =============================================================================
TRACKER_DEPENDENCIES = {
    "tracker_python_gestor.py": {"funciones_lua": ["Print_tracker_python"], "variables_lua": []},
    "tracker_trayectory.py": {"funciones_lua": ["Print_tracker_trayectory_data"], "variables_lua": ["displayed_data"]},
    "tracker_paciencia.py": {"funciones_lua": ["Print_tracker_paciencia"], "variables_lua": []},
    "tracker_AL_PID.py": {"funciones_lua": ["Print_tracker_AL_PID"], "variables_lua": []},
    "tracker_calidad_imitacion.py": {"funciones_lua": ["Print_tracker_calidad_imitacion_data"], "variables_lua": []},
    "tracker_varianza.py": {"funciones_lua": ["Print_tracker_varianza"], "variables_lua": []},
    "evaluator_otsu.py": {"funciones_lua": ["evaluator_otsu"], "variables_lua": []},
    "evaluator_qi_noise.py": {"funciones_lua": ["evaluator_ruido"], "variables_lua": []},
    "evaluator_qi_speed_noise.py": {"funciones_lua": ["evaluator_ruido"], "variables_lua": []},
}

# =============================================================================
# ---------- CONFIGURACIÓN DE RUTAS Y ARCHIVOS DE SIMULACIÓN ----------
# =============================================================================
DEFAULT_CONTROLLER_SCRIPTS = {
    "master":          "Controller/Ks7_master_controller.py",
    "apprentice":      "Controller/Ks7_apprentice_controller.py"
}

# =============================================================================
# ---------- CONFIGURACIÓN DE TRACKERS ACTIVOS (Señales a Emitir) ----------
# =============================================================================
TRACKERS = [
    {"script": "scripts/tracker_trayectory.py", "args": []},
    {"script": "scripts/tracker_paciencia.py", "args": []},
    {"script": "scripts/tracker_calidad_imitacion.py", "args": []},
    {"script": "scripts/tracker_varianza.py", "args": []},
]

# =============================================================================
# ---------- CONFIGURACIÓN DE VERBOSIDAD ----------
# =============================================================================
NIVEL_LOG = 2

def print_log(msg):
    if NIVEL_LOG >= 1:
        sys.stdout.write(msg + "\n")
        sys.stdout.flush()

def terminate_process(proc):
    if proc is None or proc.poll() is not None:
        return
    try:
        proc.terminate()
        proc.wait(timeout=3.0)
    except Exception:
        try:
            proc.kill()
        except Exception:
            pass

def main():
    parser = argparse.ArgumentParser(description="Lanzador individual para robot físico (P0)")
    parser.add_argument("--id", type=str, required=True, help="ID del robot (ej. Qupa4, Qupa1)")
    parser.add_argument("--role", type=str, required=True, choices=["master", "apprentice"], help="Rol del robot")
    parser.add_argument("--x", type=float, default=0.0, help="Posición inicial X")
    parser.add_argument("--y", type=float, default=0.0, help="Posición inicial Y")
    parser.add_argument("--yaw", type=float, default=0.0, help="Orientación inicial (radianes)")
    parser.add_argument("--figura", type=str, default="", help="Patrón a dibujar (solo si es master)")
    args = parser.parse_args()

    os.chdir(PROJECT_ROOT_DIR)

    # El robot físico solo necesita saber de sí mismo en su configuración local
    robot_config = [{
        "id": args.id,
        "role": args.role,
        "x": args.x,
        "y": args.y,
        "yaw": args.yaw,
        "figura": args.figura
    }]

    # Extraer funciones y variables necesarias según los trackers activados en P0
    req_lua_funcs = []
    req_lua_vars = []
    for t_conf in TRACKERS:
        script_key = os.path.basename(t_conf["script"])
        if script_key in TRACKER_DEPENDENCIES:
            req_lua_funcs.extend(TRACKER_DEPENDENCIES[script_key].get("funciones_lua", []))
            req_lua_vars.extend(TRACKER_DEPENDENCIES[script_key].get("variables_lua", []))
    req_lua_funcs = list(set(req_lua_funcs))
    req_lua_vars = list(set(req_lua_vars))

    with open(CONFIG_JSON_PATH, "w", encoding="utf-8") as f:
        json.dump({
            "robots": robot_config,
            "vars": {},
            "active_functions": req_lua_funcs,
            "active_variables": req_lua_vars
        }, f, indent=2)

    run_env = os.environ.copy()
    run_env["PYTHONUNBUFFERED"] = "1"
    existing_py_path = run_env.get("PYTHONPATH", "")
    extra_py_paths = f"{PROJECT_ROOT_DIR}:{os.path.join(PROJECT_ROOT_DIR, 'Controller')}:{os.path.join(PROJECT_ROOT_DIR, 'scripts')}"
    run_env["PYTHONPATH"] = f"{extra_py_paths}:{existing_py_path}" if existing_py_path else extra_py_paths

    script_path = DEFAULT_CONTROLLER_SCRIPTS[args.role]

    cmd = [
        sys.executable, "-u", script_path,
        "--ros-args", 
        "-p", f"config_file:={CONFIG_JSON_PATH}",
        "-p", f"robot_id:={args.id}",
        "-p", f"role:={args.role}",
        "-p", f"init_x:={args.x}",
        "-p", f"init_y:={args.y}",
        "-p", f"init_yaw:={args.yaw}"
    ]

    print_log(f"\n===========================================================")
    print_log(f" INICIANDO ROBOT FÍSICO (P0) - ID: {args.id} | Rol: {args.role}")
    print_log(f"===========================================================")
    print_log(f"[P0] Ejecutando controlador: {script_path}")
    print_log(f"[P0] Funciones de telemetría habilitadas: {', '.join(req_lua_funcs)}")
    print_log(f"[P0] Publicando en tópicos /ks7/telemetry de la red ROS 2...")

    proc = None
    try:
        proc = subprocess.Popen(cmd, env=run_env)
        while proc.poll() is None:
            time.sleep(1)
    except KeyboardInterrupt:
        print_log(f"\n[P0] Interrumpido (Ctrl+C). Apagando robot {args.id}...")
    finally:
        terminate_process(proc)
        if os.path.exists(CONFIG_JSON_PATH):
            os.remove(CONFIG_JSON_PATH)

if __name__ == '__main__':
    main()