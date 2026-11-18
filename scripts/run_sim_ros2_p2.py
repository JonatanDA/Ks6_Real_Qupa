#!/usr/bin/env python3
# Archivo: scripts/run_sim_ros2_p2.py
# Ubicación: ~/sim_ws_qupa/Ks7/scripts/run_sim_ros2_p2.py
# Descripción: Lanza únicamente el Gestor de Trackers que lee la telemetría de ROS 2.
# Uso: Ejecutar en Terminal 2.

import os
import sys
import subprocess
import json
import time

_THIS_FILE_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT_DIR = os.path.dirname(_THIS_FILE_DIR) if os.path.basename(_THIS_FILE_DIR) == "scripts" else _THIS_FILE_DIR
CONFIG_JSON_PATH = f"/tmp/ks7_trackers_config_p2_{os.getpid()}.json"

# =============================================================================
# ---------- CONFIGURACIÓN DE TRACKERS ----------
# =============================================================================
# Aquí defines qué gráficas o evaluadores se abren. Si el robot (P1) no está enviando
# la telemetría de uno de estos, la ventana simplemente se quedará esperando.
TRACKERS = [
    {"script": "scripts/tracker_trayectory.py", "args": []},
    {"script": "scripts/tracker_paciencia.py", "args": []},
    {"script": "scripts/tracker_calidad_imitacion.py", "args": []},
    {"script": "scripts/tracker_varianza.py", "args": []},
]

# 1 = Solo errores y avisos de trackers
# 2 = Muestra también qué datos crudos está pasando a los trackers (el eco de la telemetría)
NIVEL_LOG_P2 = 1

def print_log(msg):
    sys.stdout.write(msg + "\n")
    sys.stdout.flush()

def main():
    os.chdir(PROJECT_ROOT_DIR)
    
    with open(CONFIG_JSON_PATH, "w", encoding="utf-8") as f:
        json.dump({"trackers": TRACKERS}, f, indent=2)

    run_env = os.environ.copy()
    run_env["PYTHONUNBUFFERED"] = "1"
    existing_py_path = run_env.get("PYTHONPATH", "")
    extra_py_paths = f"{PROJECT_ROOT_DIR}:{os.path.join(PROJECT_ROOT_DIR, 'Controller')}:{os.path.join(PROJECT_ROOT_DIR, 'scripts')}"
    run_env["PYTHONPATH"] = f"{extra_py_paths}:{existing_py_path}" if existing_py_path else extra_py_paths

    tm_proc = None

    try:
        print_log("\n===========================================================")
        print_log(" INICIANDO TERMINAL 2: GESTOR DE TRACKERS (P2)")
        print_log("===========================================================")
        
        tm_cmd = [
            sys.executable, "-u", "Controller/tracker_manager_node.py",
            "--ros-args", 
            "-p", f"config_file:={CONFIG_JSON_PATH}",
            "-p", f"auto_stop_on_end:=true",
            "-p", f"nivel_log:={NIVEL_LOG_P2}"
        ]
        
        print_log("[P2] Lanzando tracker_manager_node.py...")
        print_log(f"[P2] Ejecutando {len(TRACKERS)} trackers.")
        if NIVEL_LOG_P2 == 1:
            print_log("[P2] Modo Silencioso: Ocultando flujo de datos crudos (Eco ROS 2).")
        print_log("[P2] Escuchando red ROS 2 (/ks7/telemetry)...\n")
        
        tm_proc = subprocess.Popen(tm_cmd, env=run_env)
        
        while tm_proc.poll() is None:
            time.sleep(1)

    except KeyboardInterrupt:
        print_log("\n[P2] Interrumpido (Ctrl+C). Cerrando P2...")
    finally:
        if tm_proc and tm_proc.poll() is None:
            tm_proc.terminate()
            tm_proc.wait(timeout=3.0)
            
        if os.path.exists(CONFIG_JSON_PATH):
            os.remove(CONFIG_JSON_PATH)

if __name__ == '__main__':
    main()