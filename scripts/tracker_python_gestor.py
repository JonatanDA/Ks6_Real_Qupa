#!/usr/bin/env python3
"""
tracker_python_gestor.py
Puente de comunicación entre ARGoS (Lua) y apprentice_process.py.
"""

import os
import re
import sys

_SCRIPTS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.dirname(_SCRIPTS_DIR)
_CONTROLLER_DIR = os.path.join(_PROJECT_ROOT, "Controller")
for _p in (_CONTROLLER_DIR, _PROJECT_ROOT, _SCRIPTS_DIR):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from Controller import apprentice_process as Ks6


def parse_observation_line(line):
    """Extrae los datos físicos de la trama [python];observation; enviada desde ARGoS."""
    clean = line.strip()
    parts = clean.split(';')

    obs_ang_offset_global = float(parts[2].replace(',', '.'))
    x_mark = float(parts[3].replace(',', '.'))
    y_mark = float(parts[4].replace(',', '.'))

    yellow_blobs = []
    if len(parts) > 5 and parts[5]:
        for item in parts[5].split('|'):
            if not item:
                continue
            b_parts = item.split(':')
            if len(b_parts) >= 2:
                ang = float(b_parts[0].replace(',', '.'))
                dist = float(b_parts[1].replace(',', '.'))
                ty = int(float(b_parts[2].replace(',', '.'))) if len(b_parts) >= 3 else 2
                yellow_blobs.append({'angle': ang, 'distance': dist, 'type': ty})

    green_blobs = []
    if len(parts) > 6 and parts[6]:
        for item in parts[6].split('|'):
            if not item:
                continue
            b_parts = item.split(':')
            if len(b_parts) >= 2:
                ang = float(b_parts[0].replace(',', '.'))
                dist = float(b_parts[1].replace(',', '.'))
                green_blobs.append({'angle': ang, 'distance': dist})

    return {
        'obs_ang_offset_global': obs_ang_offset_global,
        'x_mark': x_mark,
        'y_mark': y_mark,
        'yellow_blobs': yellow_blobs,
        'green_blobs': green_blobs,
    }


def main():
    Ks6.log_py("Iniciado. Escuchando datos de ARGoS en tiempo real...")
    pending_observation = None

    def flush_pending_observation():
        nonlocal pending_observation
        if pending_observation is not None:
            obs = pending_observation
            pending_observation = None
            Ks6.procesar_datos_logicos_y_reconstruccion(obs)

    for line in sys.stdin:
        if "Timer=" in line:
            flush_pending_observation()
            continue

        if "[DEPURACIÓN] INICIANDO TRANSFERENCIA A MEMORIA GLOBAL" in line:
            pending_observation = None
            Ks6.ejecutar_limpieza_ciclo()
            continue

        if "[python];patience_timeout" in line:
            pending_observation = None
            Ks6.current_patience = 0
            Ks6.check_patience_and_reconstruct(False)
            continue

        if "[PACIENCIA](" in line and "segundos" in line:
            m_pac = re.search(r"\[PACIENCIA\]\(([-+]?[0-9]*[.,]?[0-9]+)\)\s*segundos", line)
            if m_pac:
                pac_lua_val = float(m_pac.group(1).replace(',', '.'))
                if pac_lua_val <= 0.0:
                    pending_observation = None
                    Ks6.current_patience = 0
                    Ks6.check_patience_and_reconstruct(False)
                else:
                    flush_pending_observation()
                continue

        if "[python];observation;" in line:
            flush_pending_observation()
            try:
                pending_observation = parse_observation_line(line)
            except Exception as e:
                Ks6.log_py(f"[ERROR] Fallo procesando observación: {e}")
            continue

        flush_pending_observation()

        if "End_simulation" in line:
            break

    flush_pending_observation()


if __name__ == "__main__":
    main()