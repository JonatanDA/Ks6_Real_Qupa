#!/usr/bin/env python3
# Archivo: scripts/run_sim_ros2_p1.py
# Ubicación: ~/sim_ws_qupa/Ks7/scripts/run_sim_ros2_p1.py
# Descripción: Lanza Gazebo y los nodos ROS 2 de los robots (Maestros y Aprendiz).
# Uso: Ejecutar en Terminal 1.

import os
import sys
import signal
import subprocess
import threading
import time
import json
import argparse

# Determinar automáticamente la raíz del proyecto (~/sim_ws_qupa/Ks7)
_THIS_FILE_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT_DIR = os.path.dirname(_THIS_FILE_DIR) if os.path.basename(_THIS_FILE_DIR) == "scripts" else _THIS_FILE_DIR
SCRIPTS_DIR = os.path.join(PROJECT_ROOT_DIR, "scripts")
CONTROLLER_DIR = os.path.join(PROJECT_ROOT_DIR, "Controller")
FUNCTIONS_DIR = os.path.join(PROJECT_ROOT_DIR, "Functions")
GAZEBO_DIR = os.path.join(PROJECT_ROOT_DIR, "Gazebo")

# =============================================================================
# ---------- 1. DICCIONARIO DE EVALUADORES Y DEPENDENCIAS ----------
# =============================================================================
EVALUATOR_DEPENDENCIES = {
    "scripts/evaluator_qi_noise.py": {
        "lua_functions": [
            "evaluator_ruido",
            "Print_tracker_calidad_imitacion_data"
        ],
        "lua_variables": [],
        "run_sim_trackers": [
            "scripts/tracker_calidad_imitacion.py"
        ]
    },
    "scripts/evaluator_otsu.py": {
        "lua_functions": [
            "evaluator_otsu"
        ],
        "lua_variables": [],
        "run_sim_trackers": []
    },
    "scripts/evaluator_qi_speed_noise.py": {
        "lua_functions": [
            "evaluator_ruido"
        ],
        "lua_variables": [],
        "run_sim_trackers": [
            "scripts/tracker_calidad_imitacion.py"
        ]
    }
}

# =============================================================================
# ---------- 2. DICCIONARIO DE TRACKERS Y DEPENDENCIAS LUA ----------
# =============================================================================
TRACKER_DEPENDENCIES = {
    "tracker_python_gestor.py": {
        "descripcion": "",
        "funciones_lua": ["Print_tracker_python"],
        "variables_lua": []
    },
    "tracker_trayectory.py": {
        "descripcion": "Rastrea y grafica la trayectoria 2D (x,y) global del master y/o aprendiz.",
        "funciones_lua": ["Print_tracker_trayectory_data"],
        "variables_lua": ["displayed_data"]
    },
    "tracker_paciencia.py": {
        "descripcion": "Muestra la paciencia del aprendiz durante el aprendizaje",
        "funciones_lua": ["Print_tracker_paciencia"],
        "variables_lua": []
    },
    "tracker_AL_PID.py": {
        "descripcion": "",
        "funciones_lua": ["Print_tracker_AL_PID"],
        "variables_lua": []
    },
    "tracker_calidad_imitacion.py": {
        "descripcion": "Evalúa y compara la calidad geométrica del patrón dibujado frente al original (Heal Pattern, EMP, Qi).",
        "funciones_lua": ["Print_tracker_calidad_imitacion_data"],
        "variables_lua": []
    },
    "tracker_varianza.py": {
        "descripcion": "Analiza la varianza y convergencia en los datos obtenidos por el observador.",
        "funciones_lua": ["Print_tracker_varianza"],
        "variables_lua": []
    },
    "evaluator_otsu.py": {
        "descripcion": "Evalúa el método de Otsu a distintos valores de velocidad.",
        "funciones_lua": ["evaluator_otsu"],
        "variables_lua": []
    },
    "evaluator_qi_noise.py": {
        "descripcion": "Evalúa el comportamiento al inyectar ruido estadístico (ángulo y distancia).",
        "funciones_lua": ["evaluator_ruido"],
        "variables_lua": []
    },
    "evaluator_qi_speed_noise.py": {
        "descripcion": "Evalúa el comportamiento al inyectar ruido estadístico (ángulo y distancia).",
        "funciones_lua": ["evaluator_ruido"],
        "variables_lua": []
    },
}

# =============================================================================
# ---------- CONFIGURACIÓN DE RUTAS Y ARCHIVOS DE SIMULACIÓN ----------
# =============================================================================
DEFAULT_CONTROLLER_SCRIPTS      = {
    "master":          "Controller/Ks7_master_controller.py",
    "apprentice":      "Controller/Ks7_apprentice_controller.py",
    "tracker_manager": "Controller/tracker_manager_node.py",
}
DEFAULT_NODE_SCRIPT             = DEFAULT_CONTROLLER_SCRIPTS["apprentice"]
DEFAULT_MASTER_SCRIPT           = DEFAULT_CONTROLLER_SCRIPTS["master"]
DEFAULT_TRACKER_MANAGER_SCRIPT  = DEFAULT_CONTROLLER_SCRIPTS["tracker_manager"]
DEFAULT_LAUNCH_PACKAGE          = "qupa_simulation"
DEFAULT_LAUNCH_FILE             = "Gazebo/ks7_gazebo_experiment.launch.py"
AUTO_LAUNCH_GAZEBO              = True
CONFIG_JSON_PATH                = f"/tmp/ks7_robots_config_{os.getpid()}.json"

# Distribución de robots en la arena de 3x3m (equivalente a las entidades de Ks_A.argos)
# Figuras disponibles:
#   "Pentágono_irregular", "octagono", "triangulo_irregular_1", "triangulo_regular", "Heptágono"
ROBOTS_CONFIG = [
    # --- MAESTRO ACTIVO ---
    # {"id": "Qupa0", "role": "master",     "x": -1.00, "y": -0.50, "yaw": 0.0, "figura": "Pentágono_irregular"},
    {"id": "Qupa1", "role": "master",     "x": -0.50, "y": -1.00, "yaw": 0.0, "figura": "Pentágono_irregular"},
    # {"id": "Qupa2", "role": "master",     "x": -0.75, "y":  0.75, "yaw": 0.0, "figura": "octagono"},
    # {"id": "Qupa3", "role": "master",     "x":  0.00, "y":  0.00, "yaw": 0.0, "figura": "octagono"},
    # {"id": "Qupa5", "role": "master",     "x":  0.75, "y": -0.75, "yaw": 0.0, "figura": "octagono"},
    # {"id": "Qupa6", "role": "master",     "x":  0.50, "y":  1.00, "yaw": 0.0, "figura": "triangulo_irregular_1"},
    # {"id": "Qupa7", "role": "master",     "x":  1.00, "y":  0.50, "yaw": 0.0, "figura": "triangulo_regular"},

    # --- APRENDIZ / OBSERVADOR ---
    {"id": "Qupa4", "role": "apprentice", "x": -1.20, "y": -1.20, "yaw": 0.0, "figura": ""},
]

# =============================================================================
# ---------- CONFIGURACIÓN DE TRACKERS ACTIVOS ----------
# =============================================================================
TRACKERS = [

    {
        "script": "scripts/tracker_trayectory.py",
        "args": []
    },
    {
        "script": "scripts/tracker_paciencia.py",
        "args": []
    },
    # {
    #     "script": "scripts/tracker_AL_PID.py",
    #     "args": []
    # },
    {
        "script": "scripts/tracker_calidad_imitacion.py",
        "args": []
    },
    {
        "script": "scripts/tracker_varianza.py",
        "args": []
    },
    # {
    #     "script": "scripts/tracker_imitation_chain.py",
    #     "args": []
    # },
    # {
    #     "script": "scripts/tracker_imitated_pattern.py",
    #     "args": []
    # },
    # {
    #     "script": "scripts/tracker_clusters.py",
    #     "args": []
    # }
]

# =============================================================================
# ---------- CONFIGURACIÓN DE VISUALIZACIÓN E INTERFAZ ----------
# =============================================================================
ENABLE_VISUALIZATION            = True              # True: Muestra la ventana de Gazebo, False: Headless (sin GUI)
ENABLE_GAZEBO_GUI               = ENABLE_VISUALIZATION

# --- Control de Pantalla Virtual (Xvfb) ---
USE_XVFB_VIRTUAL_DISPLAY        = False
XVFB_DISPLAY_RESOLUTION         = "3840x2160x24"

# =============================================================================
# ---------- CONFIGURACIÓN DE EJECUCIÓN Y MOTOR DE ROS 2 ----------
# =============================================================================
USE_STDBUF_BUFFERING            = True

# =============================================================================
# ---------- CONFIGURACIÓN DE HISTORIAL Y ARCHIVOS DE SALIDA ----------
# =============================================================================
SAVE_OUTPUT_MODE                = "ALL"
OUTPUT_DATA_DIR                 = "Resultados/Historial_run_sim"
OUTPUT_LOG_FILENAME             = "j1.txt"

# =============================================================================
# ---------- CONTROL DE PROCESO Y FINALIZACIÓN ----------
# =============================================================================
END_SIMULATION_PATTERN          = "End_simulation"
AUTO_STOP_ON_END_PATTERN        = True
MAX_IDLE_TIMEOUT_SEC            = 240

# =============================================================================
# ---------- CONFIGURACIÓN DE VERBOSIDAD ----------
# =============================================================================
NIVEL_LOG = 3

tracker_activity = {}
print_lock = threading.Lock()

def print_log(mensaje, nivel_minimo=1, actual_nivel_log=1, end="\n"):
    if actual_nivel_log >= nivel_minimo:
        with print_lock:
            sys.stdout.write(mensaje + end)
            sys.stdout.flush()

def guardar_configuracion_robots(vars_dict=None, req_funcs=None, req_vars=None):
    data = {
        "enable_gazebo_gui": ENABLE_VISUALIZATION,
        "robots": ROBOTS_CONFIG,
        "trackers": TRACKERS,
        "vars": vars_dict or {},
        "active_functions": req_funcs if req_funcs is not None else [],
        "active_variables": req_vars if req_vars is not None else [],
    }
    with open(CONFIG_JSON_PATH, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)

def build_gazebo_launch_command(launch_pkg, launch_file, enable_gui):
    gui_str = "true" if enable_gui else "false"
    local_candidates = [
        launch_file,
        os.path.join(PROJECT_ROOT_DIR, launch_file),
        os.path.join(GAZEBO_DIR, os.path.basename(launch_file)),
    ]
    for cand in local_candidates:
        if cand and os.path.isfile(cand):
            return [
                "ros2", "launch", os.path.abspath(cand),
                f"gui:={gui_str}",
                f"config_file:={CONFIG_JSON_PATH}",
            ]
    return [
        "ros2", "launch", launch_pkg, os.path.basename(launch_file),
        f"gui:={gui_str}",
        f"config_file:={CONFIG_JSON_PATH}",
    ]

def stop_gazebo_process_group(gz_proc, actual_nivel_log=1):
    if gz_proc is None:
        return
    print_log("[Gestor] Cerrando Gazebo y ros_gz_bridge...", 1, actual_nivel_log)
    try:
        pgid = os.getpgid(gz_proc.pid)
        os.killpg(pgid, signal.SIGINT)
        try:
            gz_proc.wait(timeout=4.0)
        except subprocess.TimeoutExpired:
            os.killpg(pgid, signal.SIGTERM)
            try:
                gz_proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                os.killpg(pgid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    except Exception:
        try:
            gz_proc.terminate()
        except Exception:
            pass

def build_ros_command(node_script, extra_params=None):
    base = [
        sys.executable, "-u", node_script,
        "--ros-args", "-p", f"config_file:={CONFIG_JSON_PATH}"
    ]
    for k, v in (extra_params or {}).items():
        base.extend(["-p", f"{k}:={v}"])
    if USE_STDBUF_BUFFERING:
        return ["stdbuf", "-oL"] + base
    return base

def wait_for_ros_topics(required_topics, run_env, timeout_sec=20.0, actual_nivel_log=1):
    start_t = time.time()
    check_code = (
        "import sys, time, rclpy\n"
        "from rclpy.node import Node\n"
        "topics = sys.argv[1:]\n"
        "rclpy.init()\n"
        "node = Node('ks7_graph_waiter')\n"
        "t0 = time.time()\n"
        "ok = False\n"
        "while time.time() - t0 < float(topics[0]):\n"
        "    rclpy.spin_once(node, timeout_sec=0.1)\n"
        "    existing = dict(node.get_topic_names_and_types())\n"
        "    if all(t in existing and node.count_publishers(t) > 0 for t in topics[1:]):\n"
        "        ok = True\n"
        "        break\n"
        "    time.sleep(0.2)\n"
        "node.destroy_node()\n"
        "rclpy.shutdown()\n"
        "sys.exit(0 if ok else 1)\n"
    )
    try:
        cmd = [sys.executable, "-c", check_code, str(timeout_sec)] + list(required_topics)
        res = subprocess.run(cmd, env=run_env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=timeout_sec + 3.0)
        if res.returncode == 0:
            print_log(
                f"[Gestor] Etapa 1 lista: Tópicos {', '.join(required_topics)} activos ({time.time() - start_t:.1f}s).",
                1,
                actual_nivel_log,
            )
            return True
    except Exception:
        pass

    print_log(
        f"[Gestor] Aviso: Tiempo de espera agotado verificando {', '.join(required_topics)}; continuando...",
        1,
        actual_nivel_log,
    )
    return False

def monitor_robot_node_stdout(proc_stdout, ready_event=None, ready_token=None, actual_nivel_log=1, out_file=None, mode=None, end_sim_event=None, auto_stop=False):
    # En P1 filtramos la salida para no mostrar la data de trackers a menos que NIVEL_LOG sea 3
    mode_upper = (mode or "ALL").upper()
    for line in proc_stdout:
        line_clean = line.strip()
        if not line_clean:
            continue
            
        if ready_event is not None and ready_token and ready_token in line:
            ready_event.set()
            
        is_tracker_raw_data = False
        if line_clean.startswith("[trajectory]") or line_clean.startswith("[PACIENCIA]") or \
           line_clean.startswith("[active_learning]") or line_clean.startswith("[PID]") or \
           line_clean.startswith("[python]") or line_clean.startswith("[EVALUATOR_RUIDO]") or \
           line_clean.startswith("[varianza]") or line_clean.startswith("Otsu &") or \
           line_clean.startswith("End_simulation") or line_clean.startswith("Timer="):
            is_tracker_raw_data = True
            
        if " & " in line_clean and not ("[PYTHON GESTOR]" in line_clean or "[Gestor]" in line_clean):
            is_tracker_raw_data = True

        # Impresión a consola controlada por NIVEL_LOG
        if "Traceback" in line_clean or "[ERROR]" in line_clean or "[FATAL]" in line_clean:
            print_log(line, 1, actual_nivel_log, end="")
        elif not is_tracker_raw_data:
            print_log(line, 2, actual_nivel_log, end="")
        elif is_tracker_raw_data and actual_nivel_log >= 3:
            print_log(f"[RAW-TELEM] {line}", 3, actual_nivel_log, end="")

        # Guardado a archivo historial
        if out_file and mode and not out_file.closed:
            try:
                is_log = (
                    "[INFO]" in line
                    or "[WARNING]" in line
                    or "[ERR]" in line
                    or "[FATAL]" in line
                    or "[PYTHON GESTOR]" in line
                )
                if mode_upper == "ALL" or (mode_upper == "LOG" and is_log) or (mode_upper == "PRINT" and not is_log):
                    out_file.write(line)
                    out_file.flush()
            except ValueError:
                pass

        if auto_stop and end_sim_event and END_SIMULATION_PATTERN in line and not end_sim_event.is_set():
            print_log(f"\n[Gestor] Patrón de fin '{END_SIMULATION_PATTERN}' detectado. Cerrando...", 1, actual_nivel_log)
            end_sim_event.set()

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

def translate_return_code(code):
    if code == 0:
        return "Éxito/Normal"
    if code == -15 or code == 143:
        return "SIGTERM (Forzado a cerrarse - Esperado por Gestor)"
    if code == -9 or code == 137:
        return "SIGKILL (Asesinado abruptamente)"
    return "Desconocido"

def main():
    global ENABLE_VISUALIZATION, ENABLE_GAZEBO_GUI

    parser = argparse.ArgumentParser(description="Orquestador Secuencial Ks7 P1 (Mundo y Robots)")
    parser.add_argument("--node_script", type=str, help="Ruta a Ks7_apprentice_controller.py", default=None)
    parser.add_argument("--master_script", type=str, help="Ruta a Ks7_master_controller.py", default=None)
    parser.add_argument("--tracker_manager_script", type=str, help="Ruta a tracker_manager_node.py", default=None)
    parser.add_argument("--launch_pkg", type=str, help="Paquete ROS 2 del launch de Gazebo", default=None)
    parser.add_argument("--launch_file", type=str, help="Archivo launch de Gazebo", default=None)
    parser.add_argument("--no_launch_gazebo", action="store_true", help="No lanzar ni cerrar Gazebo automáticamente")
    parser.add_argument("--nivel_log", type=int, help="Sobrescribe NIVEL_LOG internamente", default=None)
    parser.add_argument("--vars", type=str, help="JSON con variables a inyectar", default="{}")
    parser.add_argument("--evaluators", type=str, help="Lista JSON de evaluadores a usar", default="[]")
    parser.add_argument("--visualize", type=str, help="True o False, de multi_run", default=None)
    args, _ = parser.parse_known_args()

    os.chdir(PROJECT_ROOT_DIR)

    actual_apprentice_script = args.node_script if args.node_script else DEFAULT_NODE_SCRIPT
    actual_master_script = args.master_script if args.master_script else DEFAULT_MASTER_SCRIPT
    actual_launch_pkg = args.launch_pkg if args.launch_pkg else DEFAULT_LAUNCH_PACKAGE
    actual_launch_file = args.launch_file if args.launch_file else DEFAULT_LAUNCH_FILE
    actual_nivel_log = args.nivel_log if args.nivel_log is not None else NIVEL_LOG
    should_launch_gazebo = AUTO_LAUNCH_GAZEBO and (not args.no_launch_gazebo)

    try:
        evaluators_list = json.loads(args.evaluators)
    except Exception:
        evaluators_list = []

    try:
        vars_dict = json.loads(args.vars)
    except Exception:
        vars_dict = {}

    if args.visualize is not None:
        ENABLE_VISUALIZATION = (args.visualize.lower() == "true")
        ENABLE_GAZEBO_GUI = ENABLE_VISUALIZATION

    cwd = os.getcwd()
    print_log(f"[Gestor] Directorio de trabajo: {cwd}", 1, actual_nivel_log)

    req_lua_funcs = []
    req_lua_vars = []

    for ev_script in evaluators_list:
        if ev_script in EVALUATOR_DEPENDENCIES:
            req_lua_funcs.extend(EVALUATOR_DEPENDENCIES[ev_script].get("lua_functions", []))
            req_lua_vars.extend(EVALUATOR_DEPENDENCIES[ev_script].get("lua_variables", []))
            req_trackers = EVALUATOR_DEPENDENCIES[ev_script].get("run_sim_trackers", [])
            for rt in req_trackers:
                if not any(t["script"] == rt for t in TRACKERS):
                    TRACKERS.append({"script": rt, "args": []})

    for t_conf in TRACKERS:
        script_key = os.path.basename(t_conf["script"])
        if script_key in TRACKER_DEPENDENCIES:
            req_lua_funcs.extend(TRACKER_DEPENDENCIES[script_key].get("funciones_lua", []))
            req_lua_vars.extend(TRACKER_DEPENDENCIES[script_key].get("variables_lua", []))

    req_lua_funcs = list(set(req_lua_funcs))
    req_lua_vars = list(set(req_lua_vars))

    if req_lua_funcs or req_lua_vars or vars_dict:
        print_log("[Gestor] Auto-configurando e inyectando dependencias y variables en configuración ROS 2...", 1, actual_nivel_log)

    guardar_configuracion_robots(vars_dict=vars_dict, req_funcs=req_lua_funcs, req_vars=req_lua_vars)

    output_file_handle = None
    if SAVE_OUTPUT_MODE and SAVE_OUTPUT_MODE != "None":
        os.makedirs(OUTPUT_DATA_DIR, exist_ok=True)
        full_output_path = os.path.join(OUTPUT_DATA_DIR, OUTPUT_LOG_FILENAME)
        output_file_handle = open(full_output_path, "w", encoding="utf-8")

    run_env = os.environ.copy()
    run_env["PYTHONUNBUFFERED"] = "1"
    existing_py_path = run_env.get("PYTHONPATH", "")
    extra_py_paths = f"{PROJECT_ROOT_DIR}:{CONTROLLER_DIR}:{SCRIPTS_DIR}"
    run_env["PYTHONPATH"] = f"{extra_py_paths}:{existing_py_path}" if existing_py_path else extra_py_paths

    if ENABLE_VISUALIZATION and USE_XVFB_VIRTUAL_DISPLAY:
        run_env["DISPLAY"] = ":99"

    gz_proc = None
    master_procs = []
    apprentice_proc = None
    end_sim_event = threading.Event()

    try:
        # =====================================================================
        # ETAPA 1: Lanzar Gazebo + ros_gz_bridge + camera_sim_node
        # =====================================================================
        if should_launch_gazebo:
            gz_cmd = build_gazebo_launch_command(actual_launch_pkg, actual_launch_file, ENABLE_VISUALIZATION)
            print_log(
                f"[Gestor] Etapa 1/4: Lanzando Gazebo + bridge + camera_sim_node ({actual_launch_file}, GUI={ENABLE_VISUALIZATION})...",
                1,
                actual_nivel_log,
            )
            gz_proc = subprocess.Popen(
                gz_cmd,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                env=run_env,
                start_new_session=True,
            )
            wait_for_ros_topics(["/clock", "/Qupa4/camera/detections"], run_env, timeout_sec=20.0, actual_nivel_log=actual_nivel_log)

        # ETAPA 2 ELIMINADA: EL GESTOR DE TRACKERS AHORA VIVE EN LA TERMINAL 2 (P2)
        print_log("[Gestor] Etapa 2/4: (Omitida) El Gestor de Trackers debe ejecutarse en otra terminal con run_sim_ros2_p2.py.", 1, actual_nivel_log)

        # =====================================================================
        # ETAPA 3: Lanzar N nodos Maestros
        # =====================================================================
        masters_cfg = [r for r in ROBOTS_CONFIG if str(r.get("role", "master")).lower() == "master"]
        master_ready_events = []
        print_log(
            f"[Gestor] Etapa 3/4: Lanzando {len(masters_cfg)} nodo(s) Maestro independientes ({actual_master_script})...",
            1,
            actual_nivel_log,
        )
        for m_cfg in masters_cfg:
            m_id = str(m_cfg["id"])
            m_fig = str(m_cfg.get("figura", ""))
            m_cmd = build_ros_command(
                actual_master_script,
                extra_params={
                    "robot_id": m_id,
                    "figura": m_fig,
                    "role": "master",
                    "init_x": float(m_cfg.get("x", 0.0)),
                    "init_y": float(m_cfg.get("y", 0.0)),
                    "init_yaw": float(m_cfg.get("yaw", 0.0)),
                },
            )
            m_ev = threading.Event()
            master_ready_events.append((m_id, m_ev))
            m_proc = subprocess.Popen(
                m_cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                env=run_env,
            )
            master_procs.append(m_proc)
            threading.Thread(
                target=monitor_robot_node_stdout,
                args=(m_proc.stdout, m_ev, f"[MASTER_READY] {m_id}", actual_nivel_log, output_file_handle, SAVE_OUTPUT_MODE, None, False),
                daemon=True,
            ).start()

        for m_id, m_ev in master_ready_events:
            m_ev.wait(timeout=10.0)

        # =====================================================================
        # ETAPA 4: Lanzar el nodo Aprendiz (Qupa4)
        # =====================================================================
        app_cfg_list = [r for r in ROBOTS_CONFIG if str(r.get("role", "")).lower() == "apprentice" or str(r.get("id", "")) == "Qupa4"]
        app_cfg = app_cfg_list[0] if app_cfg_list else {"id": "Qupa4", "role": "apprentice", "x": -1.20, "y": -1.20, "yaw": 0.0}
        app_id = str(app_cfg["id"])

        app_cmd = build_ros_command(
            actual_apprentice_script,
            extra_params={
                "robot_id": app_id,
                "role": "apprentice",
                "init_x": float(app_cfg.get("x", -1.20)),
                "init_y": float(app_cfg.get("y", -1.20)),
                "init_yaw": float(app_cfg.get("yaw", 0.0)),
            },
        )
        print_log(
            f"[Gestor] Etapa 4/4: Lanzando nodo Aprendiz ({app_id}): {actual_apprentice_script}...",
            1,
            actual_nivel_log,
        )
        apprentice_proc = subprocess.Popen(
            app_cmd,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
            env=run_env,
        )
        threading.Thread(
            target=monitor_robot_node_stdout,
            args=(apprentice_proc.stdout, None, None, actual_nivel_log, output_file_handle, SAVE_OUTPUT_MODE, end_sim_event, AUTO_STOP_ON_END_PATTERN),
            daemon=True,
        ).start()

        while apprentice_proc.poll() is None and not end_sim_event.is_set():
            time.sleep(0.2)

        terminate_process(apprentice_proc)
        for mp in master_procs:
            terminate_process(mp)

        ret = apprentice_proc.returncode if apprentice_proc.returncode is not None else 0
        desc_codigo = translate_return_code(ret)
        print_log(f"\n[Gestor] Nodo ROS 2 finalizó con código {ret} -> {desc_codigo}.", 1, actual_nivel_log)

        stop_gazebo_process_group(gz_proc, actual_nivel_log)
        gz_proc = None

    except KeyboardInterrupt:
        print_log("\n[Gestor] Interrumpido por usuario (Ctrl+C), terminando procesos...", 0, actual_nivel_log)
        terminate_process(apprentice_proc)
        for mp in master_procs:
            terminate_process(mp)
        stop_gazebo_process_group(gz_proc, actual_nivel_log)
        gz_proc = None
    finally:
        terminate_process(apprentice_proc)
        for mp in master_procs:
            terminate_process(mp)
        stop_gazebo_process_group(gz_proc, actual_nivel_log)
        if output_file_handle and not output_file_handle.closed:
            output_file_handle.close()
        if os.path.exists(CONFIG_JSON_PATH):
            try:
                os.remove(CONFIG_JSON_PATH)
            except Exception:
                pass

if __name__ == '__main__':
    main()