#!/usr/bin/env python3
# Archivo: scripts/run_sim_ros2_p0.py
# Ubicación: ~/sim_ws_qupa/Ks6/scripts/run_sim_ros2_p0.py
# Descripción:
#   Orquestador secuencial en el robot físico Qupa (Raspberry Pi Zero W2).
#   1) Opcionalmente lanza el stack de hardware (hardware.launch.py) si AUTO_LAUNCH_HARDWARE = True.
#      (Recomendado dejarlo en False si ya lanzaste hardware.launch.py en una terminal aparte).
#   2) Lanza opcionalmente la cámara (camera.launch.py) si es aprendiz o si ENABLE_MASTER_CAMERA = True.
#   3) Lanza finalmente el controlador seleccionado con use_sim_time:=false.
# Uso desde ~/sim_ws_qupa/Ks6:
#   python3 scripts/run_sim_ros2_p0.py --id qupa_D2 --role master --controller Controller/Ks6_master_controller.py

import os
import sys
import signal
import subprocess
import time
import json
import argparse

_THIS_FILE_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT_DIR = os.path.dirname(_THIS_FILE_DIR) if os.path.basename(_THIS_FILE_DIR) == "scripts" else _THIS_FILE_DIR
if PROJECT_ROOT_DIR not in sys.path:
    sys.path.insert(0, PROJECT_ROOT_DIR)

CONFIG_JSON_PATH = f"/tmp/ks6_robot_config_p0_{os.getpid()}.json"

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
# ---------- 3. SELECCIÓN DE CONTROLADOR Y CONFIGURACIÓN DEL ROBOT FÍSICO -----
# =============================================================================
DEFAULT_CONTROLLER_SCRIPTS = {
    "master":          "Controller/Ks6_master_controller.py",
    "apprentice":      "Controller/Ks6_apprentice_controller.py",
}

# Controlador y parámetros por defecto cuando se ejecuta `python3 scripts/run_sim_ros2_p0.py`
ACTIVE_ROLE_DEFAULT        = "master"
ACTIVE_CONTROLLER_SCRIPT   = DEFAULT_CONTROLLER_SCRIPTS[ACTIVE_ROLE_DEFAULT]
DEFAULT_ROBOT_ID           = "qupa_D2"
DEFAULT_FIGURA             = "Pentágono_irregular"
DEFAULT_INIT_X             = 0.0
DEFAULT_INIT_Y             = 0.0
DEFAULT_INIT_YAW           = 0.0

# Etapa 1: Drivers físicos (qupa_hardware).
# False = se asume que ya lanzaste `ros2 launch qupa_hardware hardware.launch.py namespace:=qupa_D2` una vez al inicio.
# True  = lanza hardware.launch.py automáticamente cada vez.
AUTO_LAUNCH_HARDWARE       = False

# Etapa 2: Cámara omnidireccional (camera.launch.py).
# "AUTO": activa cámara en apprentice y usa ENABLE_MASTER_CAMERA para master; True: siempre; False: nunca.
AUTO_LAUNCH_CAMERA         = "AUTO"
ENABLE_MASTER_CAMERA       = True   # True = activa cámara en el maestro | False = desactivada

HARDWARE_LAUNCH_PACKAGE    = "qupa_hardware"
HARDWARE_LAUNCH_FILE       = "hardware.launch.py"
# Modo de cámara real en qupa_hardware:
#   "detections"  -> camera.launch.py (publica /{id}/camera/detections, NECESARIO para que el aprendiz detecte maestros)
#   "calibration" -> camera_calibration.launch.py (publica /{id}/camera/image_calibration/compressed para ver/calibrar en RViz/rqt)
DEFAULT_CAMERA_MODE        = "detections"
CAMERA_LAUNCH_FILES        = {
    "detections":  "camera.launch.py",
    "calibration": "camera_calibration.launch.py",
}
CAMERA_LAUNCH_FILE         = CAMERA_LAUNCH_FILES[DEFAULT_CAMERA_MODE]

# Tiempos de espera secuencial (en segundos):
WAIT_AFTER_HARDWARE_SEC    = 12.0
WAIT_AFTER_CAMERA_SEC      = 5.0

# =============================================================================
# ---------- 4. CONFIGURACIÓN DE TRACKERS ACTIVOS (Señales a Emitir) ----------
# =============================================================================
TRACKERS = [
    {"script": "scripts/tracker_trayectory.py", "args": []},
    {"script": "scripts/tracker_paciencia.py", "args": []},
    {"script": "scripts/tracker_calidad_imitacion.py", "args": []},
    {"script": "scripts/tracker_varianza.py", "args": []},
]

# =============================================================================
# ---------- 5. CONFIGURACIÓN DE VERBOSIDAD ----------
# =============================================================================
NIVEL_LOG = 2


def print_log(msg):
    if NIVEL_LOG >= 1:
        sys.stdout.write(msg + "\n")
        sys.stdout.flush()


def terminate_process(proc, name="proceso"):
    if proc is None or proc.poll() is not None:
        return
    print_log(f"[P0] Deteniendo {name}...")
    try:
        proc.terminate()
        proc.wait(timeout=3.0)
    except Exception:
        try:
            proc.kill()
        except Exception:
            pass


def stop_process_group(proc, name="launch"):
    if proc is None or proc.poll() is not None:
        return
    print_log(f"[P0] Cerrando grupo de procesos de {name}...")
    try:
        pgid = os.getpgid(proc.pid)
        os.killpg(pgid, signal.SIGINT)
        try:
            proc.wait(timeout=4.0)
        except subprocess.TimeoutExpired:
            os.killpg(pgid, signal.SIGTERM)
            try:
                proc.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                os.killpg(pgid, signal.SIGKILL)
    except ProcessLookupError:
        pass
    except Exception:
        terminate_process(proc, name)


def wait_with_countdown(seconds: float, stage_label: str):
    total = max(0.0, float(seconds))
    if total <= 0.0:
        return
    print_log(f"[P0] Esperando {total:.1f}s para estabilizar {stage_label}...")
    end_t = time.time() + total
    while time.time() < end_t:
        time.sleep(min(0.5, end_t - time.time()))


def wait_for_ros_topics(required_topics, run_env, timeout_sec=10.0):
    if not required_topics:
        return True
    start_t = time.time()
    check_code = (
        "import sys, time, rclpy\n"
        "from rclpy.node import Node\n"
        "topics = sys.argv[1:]\n"
        "rclpy.init()\n"
        "node = Node('ks6_p0_graph_waiter')\n"
        "t0 = time.time()\n"
        "ok = False\n"
        "while time.time() - t0 < float(topics[0]):\n"
        "    rclpy.spin_once(node, timeout_sec=0.1)\n"
        "    existing = dict(node.get_topic_names_and_types())\n"
        "    if all(t in existing for t in topics[1:]):\n"
        "        ok = True\n"
        "        break\n"
        "    time.sleep(0.2)\n"
        "node.destroy_node()\n"
        "rclpy.shutdown()\n"
        "sys.exit(0 if ok else 1)\n"
    )
    try:
        cmd = [sys.executable, "-c", check_code, str(timeout_sec)] + list(required_topics)
        res = subprocess.run(
            cmd,
            env=run_env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
            timeout=timeout_sec + 3.0,
        )
        if res.returncode == 0:
            print_log(f"[P0] Tópicos verificados ({', '.join(required_topics)}) en {time.time() - start_t:.1f}s.")
            return True
    except Exception:
        pass
    print_log(f"[P0] Aviso: No se pudo confirmar {', '.join(required_topics)} antes del timeout; continuando...")
    return False


def main():
    parser = argparse.ArgumentParser(description="Orquestador secuencial para robot físico Qupa (P0)")
    parser.add_argument("--id", type=str, default=DEFAULT_ROBOT_ID, help=f"Namespace/ID del robot (default: {DEFAULT_ROBOT_ID})")
    parser.add_argument("--role", type=str, default=ACTIVE_ROLE_DEFAULT, choices=["master", "apprentice"], help=f"Rol del robot (default: {ACTIVE_ROLE_DEFAULT})")
    parser.add_argument("--controller", type=str, default=None, help="Ruta opcional al script controlador a cargar")
    parser.add_argument("--x", type=float, default=DEFAULT_INIT_X, help="Posición inicial X")
    parser.add_argument("--y", type=float, default=DEFAULT_INIT_Y, help="Posición inicial Y")
    parser.add_argument("--yaw", type=float, default=DEFAULT_INIT_YAW, help="Orientación inicial (grados)")
    parser.add_argument("--figura", type=str, default=DEFAULT_FIGURA, help=f"Patrón a dibujar si es master (default: {DEFAULT_FIGURA})")
    parser.add_argument("--launch_hardware", action="store_true", help="Forzar el lanzamiento de hardware.launch.py")
    parser.add_argument("--no_hardware", action="store_true", help="Omitir el lanzamiento de hardware.launch.py (si ya está corriendo)")
    parser.add_argument("--launch_camera", action="store_true", help="Forzar el lanzamiento de camera.launch.py")
    parser.add_argument("--no_camera", action="store_true", help="No lanzar cámara")
    parser.add_argument("--camera_mode", type=str, default=DEFAULT_CAMERA_MODE, choices=["detections", "calibration"], help="detections=camera.launch.py (/{id}/camera/detections) | calibration=camera_calibration.launch.py (video comprimido)")
    parser.add_argument("--wait_hw", type=float, default=WAIT_AFTER_HARDWARE_SEC, help="Segundos de espera tras lanzar hardware.launch.py")
    parser.add_argument("--wait_cam", type=float, default=WAIT_AFTER_CAMERA_SEC, help="Segundos de espera tras lanzar camera.launch.py")
    args = parser.parse_args()
    # Normalizar ID corto (ej. 'D2' -> 'qupa_D2') respetando namespaces que ya inicien con 'qupa_' o 'Qupa'
    raw_id = args.id.strip().lstrip('/')
    if not (raw_id.lower().startswith('qupa')):
        args.id = f'qupa_{raw_id}'
        print_log(f'[P0-Real] ID normalizado: "{raw_id}" -> "{args.id}"')
    else:
        args.id = raw_id
    selected_cam_launch = CAMERA_LAUNCH_FILES.get(args.camera_mode, CAMERA_LAUNCH_FILE)

    os.chdir(PROJECT_ROOT_DIR)

    if args.controller:
        script_path = args.controller
    elif args.role == ACTIVE_ROLE_DEFAULT and ACTIVE_CONTROLLER_SCRIPT:
        script_path = ACTIVE_CONTROLLER_SCRIPT
    else:
        script_path = DEFAULT_CONTROLLER_SCRIPTS[args.role]

    if args.no_hardware:
        should_launch_hw = False
    elif args.launch_hardware:
        should_launch_hw = True
    else:
        should_launch_hw = bool(AUTO_LAUNCH_HARDWARE)

    if args.no_camera:
        should_launch_cam = False
    elif args.launch_camera:
        should_launch_cam = True
    elif AUTO_LAUNCH_CAMERA == "AUTO":
        should_launch_cam = True if args.role.lower() == "apprentice" else bool(ENABLE_MASTER_CAMERA)
    else:
        should_launch_cam = bool(AUTO_LAUNCH_CAMERA)

    robot_config = [{
        "id": args.id,
        "role": args.role,
        "x": args.x,
        "y": args.y,
        "yaw": args.yaw,
        "figura": args.figura if args.role == "master" else "",
    }]

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
            "active_variables": req_lua_vars,
        }, f, indent=2, ensure_ascii=False)

    run_env = os.environ.copy()
    run_env["PYTHONUNBUFFERED"] = "1"
    run_env.setdefault("RMW_IMPLEMENTATION", "rmw_cyclonedds_cpp")
    run_env.setdefault("ROS_DOMAIN_ID", "0")
    run_env.setdefault("ROS_AUTOMATIC_DISCOVERY_RANGE", "SUBNET")

    default_cyclone_xml = os.path.join(PROJECT_ROOT_DIR, "cyclonedds_config.xml")
    if "CYCLONEDDS_URI" not in run_env and os.path.isfile(default_cyclone_xml):
        run_env["CYCLONEDDS_URI"] = f"file://{default_cyclone_xml}"

    existing_py_path = run_env.get("PYTHONPATH", "")
    extra_py_paths = f"{PROJECT_ROOT_DIR}:{os.path.join(PROJECT_ROOT_DIR, 'Controller')}:{os.path.join(PROJECT_ROOT_DIR, 'scripts')}"
    run_env["PYTHONPATH"] = f"{extra_py_paths}:{existing_py_path}" if existing_py_path else extra_py_paths

    print_log("\n===========================================================")
    print_log(f" INICIANDO ROBOT FÍSICO (P0) - ID: {args.id} | Rol: {args.role}")
    print_log("===========================================================")
    print_log(f"[P0] RMW_IMPLEMENTATION: {run_env.get('RMW_IMPLEMENTATION')}")
    if "CYCLONEDDS_URI" in run_env:
        print_log(f"[P0] CYCLONEDDS_URI:     {run_env.get('CYCLONEDDS_URI')}")
    print_log(f"[P0] Controlador final:  {script_path} (Figura: {args.figura or 'N/A'})")

    hw_proc = None
    cam_proc = None
    ctrl_proc = None

    try:
        # ETAPA 1: Drivers de hardware
        if should_launch_hw:
            hw_cmd = [
                "ros2", "launch", HARDWARE_LAUNCH_PACKAGE, HARDWARE_LAUNCH_FILE,
                f"namespace:={args.id}",
            ]
            print_log(f"\n[P0] Etapa 1/3: Lanzando drivers de hardware -> {' '.join(hw_cmd)}")
            hw_proc = subprocess.Popen(hw_cmd, env=run_env, start_new_session=True)
            wait_with_countdown(args.wait_hw, f"{HARDWARE_LAUNCH_FILE} ({args.id})")
            wait_for_ros_topics([f"/{args.id}/scan", f"/{args.id}/cmd_vel"], run_env, timeout_sec=8.0)
        else:
            print_log("\n[P0] Etapa 1/3: (Omitida) Se asume que hardware.launch.py ya está activo.")

        # ETAPA 2: Cámara omnidireccional
        if should_launch_cam:
            cam_cmd = [
                "ros2", "launch", HARDWARE_LAUNCH_PACKAGE, selected_cam_launch,
                f"namespace:={args.id}",
            ]
            print_log(f"\n[P0] Etapa 2/3: Lanzando nodo de cámara -> {' '.join(cam_cmd)}")
            cam_proc = subprocess.Popen(cam_cmd, env=run_env, start_new_session=True)
            wait_with_countdown(args.wait_cam, f"{CAMERA_LAUNCH_FILE} ({args.id})")
            expected_cam_topic = f"/{args.id}/camera/detections" if args.camera_mode == "detections" else f"/{args.id}/camera/image_calibration/compressed"
            print_log(f"[P0-Real] Modo cámara: {args.camera_mode} ({selected_cam_launch}) -> esperando {expected_cam_topic}")
            wait_for_ros_topics([expected_cam_topic], run_env, timeout_sec=35.0)
        else:
            print_log(f"\n[P0] Etapa 2/3: (Omitida) Cámara desactivada para rol '{args.role}'.")

        # ETAPA 3: Controlador
        ctrl_cmd = [
            sys.executable, "-u", script_path,
            "--ros-args",
            "-p", "use_sim_time:=false",
            "-p", f"config_file:={CONFIG_JSON_PATH}",
            "-p", f"robot_id:={args.id}",
            "-p", f"role:={args.role}",
            "-p", f"figura:={args.figura}",
            "-p", f"init_x:={args.x}",
            "-p", f"init_y:={args.y}",
            "-p", f"init_yaw:={args.yaw}",
        ]
        print_log(f"\n[P0] Etapa 3/3: Lanzando controlador {script_path}...")
        print_log(f"[P0] Telemetría habilitada: {', '.join(req_lua_funcs)}")
        ctrl_proc = subprocess.Popen(ctrl_cmd, env=run_env)

        while ctrl_proc.poll() is None:
            time.sleep(0.5)

    except KeyboardInterrupt:
        print_log(f"\n[P0] Interrumpido por el usuario (Ctrl+C). Apagando {args.id} en orden inverso...")
    finally:
        terminate_process(ctrl_proc, f"controlador ({script_path})")
        stop_process_group(cam_proc, CAMERA_LAUNCH_FILE)
        stop_process_group(hw_proc, HARDWARE_LAUNCH_FILE)
        if os.path.exists(CONFIG_JSON_PATH):
            try:
                os.remove(CONFIG_JSON_PATH)
            except Exception:
                pass
        print_log(f"[P0] Apagado completo de {args.id}.")


if __name__ == '__main__':
    main()