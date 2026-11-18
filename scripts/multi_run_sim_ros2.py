#!/usr/bin/env python3
# Archivo: scripts/multi_run_sim_ros2.py (Equivalente 1:1 de scripts/multi_run_sim.py)
# Ubicación exacta:
#   ~/sim_ws_qupa/Ks7/scripts/multi_run_sim_ros2.py

"""
multi_run_sim_ros2.py

Orquestador Maestro de Experimentos en ROS 2 + Gazebo (Auto-Configurable).
- Genera las combinaciones de variables a evaluar por grupos.
- Ejecuta scripts/run_sim_ros2.py enviándole el script del nodo, las variables modificadas y los evaluadores a usar.
- run_sim_ros2.py inyecta las variables en /tmp/ks7_robots_config.json y lanza/cierra Gazebo y los nodos ROS 2 en cada corrida.
"""

import os
import sys
import subprocess
import itertools
import time
import json
import threading

# Determinar automáticamente la raíz del proyecto (~/sim_ws_qupa/Ks7)
_THIS_FILE_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT_DIR = os.path.dirname(_THIS_FILE_DIR) if os.path.basename(_THIS_FILE_DIR) == "scripts" else _THIS_FILE_DIR

# =============================================================================
# ---------- 1. CONFIGURACIÓN DEL EXPERIMENTO (VARIABLES A BARRER) ----------
# =============================================================================

NUM_EXP_M        = 20   # Repeticiones por cada configuración (Simulaciones idénticas)

# Definición de grupos de variables.
# Formato de cada grupo: ( {variables}, NUM_PASOS_N, MODO_EVALUACION )
# MODO: 1 -> Simultáneo (Avanza las variables al mismo tiempo)
#       2 -> Combinatorio (Todas las mezclas posibles internamente en el grupo)
VARIABLES_A_EVALUAR = [
    # (
    #     {
    #         "NOISE_FACTOR"  : (0.0, 0.15),
    #     },
    #     7, # NUMERO DE PASOS
    #     1  # MODO A EVALUAR
    # ),

    (
        {  # Grupo 1
            "factor_vel": (1, 2)
        },
        3,  # NUMERO DE PASOS
        1   # MODO A EVALUAR
    ),

    (
        # { # Grupo 2
        #     "NOISE_LEVEL_DIST"  : (0.0, 0.05),
        #     "NOISE_LEVEL_ANGLE" : (0.0, 0.25),
        # },
        # 5, # NUMERO DE PASOS
        # 1  # MODO A EVALUAR

        {
            "NOISE_FACTOR": (0.0, 0.16),
        },
        5,  # NUMERO DE PASOS
        1   # MODO A EVALUAR
    ),
]

# =============================================================================
# ---------- 2. CONFIGURACIÓN DE MULTI EVALUATORS ----------
# =============================================================================
MULTI_EVALUATORS = [
    # {
    #     "script": "scripts/evaluator_qi_noise.py",
    #     "args": []
    # },
    # {
    #     "script": "scripts/evaluator_otsu.py",
    #     "args": []
    # },
    {
        "script": "scripts/evaluator_qi_speed_noise.py",
        "args": []
    },
]

# =============================================================================
# ---------- 3. CONFIGURACIÓN DE ARCHIVOS Y RUTAS ----------
# =============================================================================
NODE_SCRIPT        = "Controller/Ks7_apprentice_controller.py"
LAUNCH_PACKAGE     = "qupa_simulation"
LAUNCH_FILE        = "Gazebo/ks7_gazebo_experiment.launch.py"
RUN_SIM_SCRIPT     = "scripts/run_sim_ros2.py"
TIMEOUT_SIMULACION = 120

# =============================================================================
# ---------- CONFIGURACIÓN VISUAL Y VERBOSIDAD ----------
# =============================================================================
ENABLE_VISUALIZATION = False  # Se enviará a run_sim_ros2.py para sobrescribir su valor por defecto

# =============================================================================
# ---------- CONFIGURACIÓN DE VERBOSIDAD ----------
# =============================================================================
# 0: Silencioso total (solo info inicial y conteo de exps). Todo pasa oculto. Ideal para pruebas masivas.
# 1: Recomendado (Muestra eventos vitales: inicios, cierres, guardados de evaluadores).
# 2: Detallado (Modo 1 + Prints específicos etiquetados de los Trackers, sin mezclar salidas ruidosas).
# 3: Todo (Modo 2 + Datos puros de ROS 2 / Gazebo como variables continuas, timers, alertas internas, etc).
NIVEL_LOG = 2


def print_log(mensaje, nivel_minimo=0, end="\n"):
    if NIVEL_LOG >= nivel_minimo:
        sys.stdout.write(mensaje + end)
        sys.stdout.flush()


# =============================================================================
# ---------- GENERADOR DE COMBINACIONES ----------
# =============================================================================
def generar_configuraciones_grupo(variables, pasos, modo):
    valores_por_var = {}
    for var, rango in variables.items():
        if pasos <= 1:
            valores_por_var[var] = [rango[0]]
        else:
            paso_val = (rango[1] - rango[0]) / (pasos - 1)
            valores_por_var[var] = [rango[0] + (i * paso_val) for i in range(pasos)]

    configuraciones = []
    keys = list(valores_por_var.keys())
    if modo == 1:
        for i in range(pasos):
            configuraciones.append({k: valores_por_var[k][i] for k in keys})
    elif modo == 2:
        for combo in itertools.product(*[valores_por_var[k] for k in keys]):
            configuraciones.append({keys[i]: combo[i] for i in range(len(keys))})
    return configuraciones


def generar_configuraciones(grupos):
    listas_configs = []
    for g in grupos:
        listas_configs.append(generar_configuraciones_grupo(g[0], g[1], g[2]))

    # Invertimos el orden para que el último grupo en la lista varíe más lento (loop externo),
    # y el primer grupo varíe más rápido (loop interno), tal cual en multi_run_sim.py.
    listas_configs.reverse()

    configs_finales = []
    for combos in itertools.product(*listas_configs):
        merged = {}
        for c in reversed(combos):
            merged.update(c)
        configs_finales.append(merged)

    return configs_finales


# =============================================================================
# ---------- FUNCIONES DE EVALUADOR ----------
# =============================================================================
def enviar_a_evaluadores(procesos, mensaje):
    for p in procesos:
        if p.poll() is None:
            try:
                p.stdin.write(mensaje + "\n")
                p.stdin.flush()
            except OSError:
                pass


def monitor_evaluator_stdout(proc, script_name):
    for linea in iter(proc.stdout.readline, ''):
        if not linea:
            break
        print_log(f"[EVALUADOR: {script_name}] {linea}", 1, end="")


# =============================================================================
# ---------- BUCLE PRINCIPAL ----------
# =============================================================================
def main():
    os.chdir(PROJECT_ROOT_DIR)

    print_log("=" * 75, 0)
    print_log(" INICIANDO ORQUESTADOR DE EXPERIMENTOS ROS 2 + GAZEBO (AUTO-CONFIGURABLE)", 0)
    print_log("=" * 75, 0)

    configuraciones = generar_configuraciones(VARIABLES_A_EVALUAR)
    total_configs = len(configuraciones)
    total_sims = total_configs * NUM_EXP_M

    print_log(f"[INFO] Grupos de evaluación : {len(VARIABLES_A_EVALUAR)}", 0)
    print_log("-" * 75, 0)
    print_log(f"[>] Configuraciones totales : {total_configs}", 0)
    print_log(f"[>] Repeticiones por config : {NUM_EXP_M}", 0)
    print_log(f"[!] TOTAL DE SIMULACIONES   : {total_sims}", 0)
    print_log("-" * 75, 0)

    while True:
        resp = input(
            f"¿Desea iniciar la ejecución de {total_sims} simulaciones? (Enter para continuar / 'n' para cancelar): "
        ).strip().lower()
        if resp == 'n':
            print("Ejecución cancelada por el usuario.")
            return
        elif resp == '':
            break

    eval_procesos = []
    eval_threads = []
    print_log("\n[+] Levantando Evaluadores...", 1)

    for ev in MULTI_EVALUATORS:
        cmd = [sys.executable, "-u", ev["script"]] + ev["args"]
        nombre_script = os.path.basename(ev["script"])
        p = subprocess.Popen(
            cmd, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, text=True, bufsize=1
        )
        eval_procesos.append(p)
        print_log(f"    - Iniciado: {ev['script']} (PID: {p.pid})", 1)

        t = threading.Thread(target=monitor_evaluator_stdout, args=(p, nombre_script), daemon=True)
        t.start()
        eval_threads.append(t)

    evaluators_list = [ev["script"] for ev in MULTI_EVALUATORS]

    try:
        for i, config_actual in enumerate(configuraciones):
            str_vars = ", ".join([f"{k}: {v:.4f}" for k, v in config_actual.items()])
            print_log(f"\n[{'=' * 70}]", 1)
            print_log(f"[CONFIG {i + 1}/{total_configs}] Aplicando -> {str_vars}", 1)

            cfg_data = {"id": i + 1, "params": config_actual}
            enviar_a_evaluadores(eval_procesos, f"[MULTI_CONFIG] {json.dumps(cfg_data)}")

            for j in range(NUM_EXP_M):
                num_sim = (i * NUM_EXP_M) + (j + 1)
                print_log(f"  > Ejecutando exp {j + 1}/{NUM_EXP_M} (Simulación Global {num_sim}/{total_sims})", 0)

                enviar_a_evaluadores(eval_procesos, f"[SIM_START] {j + 1}")

                cmd = [
                    sys.executable, "-u", RUN_SIM_SCRIPT,
                    "--node_script", NODE_SCRIPT,
                    "--launch_pkg", LAUNCH_PACKAGE,
                    "--launch_file", LAUNCH_FILE,
                    # FORZADO A NIVEL 3. Multi_run capturará todo y filtrará su propia consola.
                    "--nivel_log", "3",
                    "--vars", json.dumps(config_actual),
                    "--evaluators", json.dumps(evaluators_list),
                    "--visualize", str(ENABLE_VISUALIZATION)
                ]
                proc_sim = subprocess.Popen(
                    cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True, bufsize=1
                )

                tiempo_inicio = time.time()
                while True:
                    if time.time() - tiempo_inicio > TIMEOUT_SIMULACION:
                        print_log("    [!] TIMEOUT de simulación. Forzando cierre...", 1)
                        proc_sim.terminate()
                        break

                    linea = proc_sim.stdout.readline()
                    if not linea and proc_sim.poll() is not None:
                        break

                    if linea:
                        enviar_a_evaluadores(eval_procesos, linea.strip())

                        if NIVEL_LOG == 3:
                            sys.stdout.write(linea)
                        elif NIVEL_LOG == 2:
                            if (
                                "[Gestor]" in linea
                                or "[+]" in linea
                                or "[Análisis Calidad]" in linea
                                or "Nodo ROS 2 finalizó" in linea
                                or "ARGoS finalizó" in linea
                            ):
                                sys.stdout.write(linea)
                        elif NIVEL_LOG == 1:
                            if (
                                "[Gestor]" in linea
                                or "[+] Esperando" in linea
                                or "[+] Todos los procesos" in linea
                                or "Nodo ROS 2 finalizó" in linea
                                or "ARGoS finalizó" in linea
                            ):
                                sys.stdout.write(linea)

                proc_sim.wait()
                enviar_a_evaluadores(eval_procesos, f"[SIM_END] {j + 1}/{NUM_EXP_M}")
                print_log(f"  < Fin exp {j + 1}/{NUM_EXP_M}", 0)

    except KeyboardInterrupt:
        print_log("\n[!] Ejecución interrumpida por el usuario.", 0)

    finally:
        print_log("\n[+] Enviando señal de finalización a los evaluadores...", 1)
        enviar_a_evaluadores(eval_procesos, "[MULTI_END_ALL]")

        for p in eval_procesos:
            if p.stdin:
                try:
                    p.stdin.close()
                except Exception:
                    pass
            p.wait()

        for t in eval_threads:
            t.join(timeout=3.0)

        print_log("\n" + "=" * 75, 0)
        print_log(" EXPERIMENTACIÓN FINALIZADA ", 0)
        print_log("=" * 75, 0)


if __name__ == '__main__':
    main()
