# Archivo: Ks6/Functions/sensors.py
# Ubicación: ~/sim_ws_qupa/Ks6/Functions/sensors.py
# Descripción:
#   Módulo compartido de lectura de sensores físicos para los robots Qupa en ROS 2 + Gazebo
#   (sensores de suelo base_ground / motor_ground y sensores de proximidad IR).
#   Equivalente 1:1 de Ks6/Functions/sensors.lua.

import math


def _norm_rad(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


# ============================================================================
# 1. LECTURA DE SENSORES DE SUELO (ground)
# ============================================================================
def ground(rob, opts: dict = None, log_fn=print) -> dict:
    """Equivalente 1:1 de ground(opts) en Functions/sensors.lua."""
    opts = opts or {}
    base_ground_active    = bool(opts.get('base_ground_active', False))
    print_all             = bool(opts.get('print_all_data', False))
    print_detected_ranges = bool(opts.get('print_detected_ranges', False))
    ranges                = opts.get('range', []) or []

    Ground = {
        'base'  : [],
        'motor' : [],
        'Gr'    : [],
    }

    base_readings = getattr(rob, 'base_ground', []) or []
    motor_readings = getattr(rob, 'motor_ground', []) or []

    # Sensores base_ground
    if base_ground_active and base_readings:
        if print_all:
            log_fn(f" ------------------ BASE GROUND ------------------ {rob.id}")
        for i, reading in enumerate(base_readings, start=1):
            val = float(reading.get('value', reading) if isinstance(reading, dict) else reading)
            Ground['base'].append(val)
            if print_all:
                log_fn(f"[Base_Ground {i}] valor={val:.3f}")
            for r_index, bounds in enumerate(ranges, start=1):
                if bounds[0] <= val <= bounds[1]:
                    Ground['Gr'].append([r_index, i])

    if print_all:
        log_fn(f" ------------------ MOTOR GROUND ----------------- {rob.id}")

    # Sensores motor_ground
    if motor_readings:
        base_count = len(base_readings)
        for i, reading in enumerate(motor_readings, start=1):
            val = float(reading.get('value', reading) if isinstance(reading, dict) else reading)
            Ground['motor'].append(val)
            if print_all:
                log_fn(f"[Motor_Ground {i}] valor={val:.3f}")
            for r_index, bounds in enumerate(ranges, start=1):
                if bounds[0] <= val <= bounds[1]:
                    Ground['Gr'].append([r_index, i + base_count])

    # Ordenar por número de rango (primer valor de cada par)
    Ground['Gr'].sort(key=lambda pair: pair[0])

    if print_detected_ranges:
        log_fn(f" -------------------------------------------------- {rob.id}")
        if Ground['Gr']:
            log_fn("Rangos detectados:")
            for rango, sensor in Ground['Gr']:
                log_fn(f"  - Rango Detectado: {rango} por el sensor #{sensor},")

    return Ground


# ============================================================================
# 2. LECTURA DE SENSORES DE PROXIMIDAD (obtener_angulo_sensor, proximidad)
# ============================================================================
proximity = [0.0, 0.0]


def obtener_angulo_sensor(i: int, n_sensores: int, dist_sens=None, log_fn=print):
    """Equivalente 1:1 de obtener_angulo_sensor(i, n_sensores) en Functions/sensors.lua."""
    if dist_sens == "uniform" or dist_sens is None:
        N = n_sensores / 2.0
        step = 180.0 / N
        if i <= N:
            return i * step
        else:
            return -180.0 + (i - N) * step

    elif isinstance(dist_sens, list):
        if len(dist_sens) != n_sensores:
            log_fn(
                f"[ERROR] La tabla de ángulos tiene {len(dist_sens)} elementos, "
                f"pero hay {n_sensores} sensores."
            )
            return None
        return float(dist_sens[i - 1])

    else:
        log_fn("[ERROR] 'Distribucion_sensores_de_proximidad' debe ser 'uniform' o una tabla de ángulos.")
        return None


def proximidad(rob, modo_detallado: bool = False):
    """Equivalente 1:1 de proximidad(modo_detallado) en Functions/sensors.lua."""
    global proximity
    sum_val = 0.0
    count = 0
    sum_x = 0.0
    sum_y = 0.0

    current_max_val = 0.0
    current_max_angle = 0.0

    # 1) Recorremos todos los sensores
    for s in rob.prox_sensors:
        v = float(s['valor'])
        angle = float(s['angulo'])

        if v > current_max_val:
            current_max_val = v
            current_max_angle = angle

        if v > 0.01:
            rad = math.radians(angle)
            sum_val += v
            count += 1
            sum_x += v * math.cos(rad)
            sum_y += v * math.sin(rad)

    # 2) Cálculo de promedios
    prom_v = (sum_val / count) if count > 0 else 0.0
    prom_a = math.degrees(math.atan2(sum_y, sum_x)) if (sum_x != 0.0 or sum_y != 0.0) else 0.0

    # 3) Salida según el parámetro recibido
    if modo_detallado:
        return {
            'lista': rob.prox_sensors,
            'max_val': {'valor': current_max_val, 'angulo': current_max_angle},
            'prom_val': {'valor': prom_v, 'angulo': prom_a},
        }
    else:
        proximity[0] = prom_v
        proximity[1] = prom_a
        return [prom_v, prom_a]


# ============================================================================
# 3. ACTUALIZACIÓN FÍSICA DE PROXIMIDAD EN ROS 2 / GAZEBO (LaserScan y Poses)
# ============================================================================
def process_laser_scan(rob, msg):
    """Actualiza rob.prox_sensors a partir de un mensaje sensor_msgs/LaserScan."""
    if not msg.ranges:
        return
    n = len(msg.ranges)
    r_min = max(0.02, msg.range_min if msg.range_min > 0.0 else 0.06)
    r_max = min(0.35, msg.range_max if msg.range_max > 0.0 else 0.35)
    two_pi = 2.0 * math.pi

    for sensor in rob.prox_sensors:
        target_rad = math.radians(sensor['angulo'])
        if msg.angle_increment != 0.0:
            rel_rad = (target_rad - msg.angle_min) % two_pi
            step_idx = int(round(rel_rad / abs(msg.angle_increment)))
            effective_n = (
                (n - 1)
                if (n > 1 and abs((n - 1) * abs(msg.angle_increment) - two_pi) < 1e-2)
                else n
            )
            idx = step_idx % effective_n
            r = msg.ranges[idx]
            if math.isfinite(r) and r > 0.0 and r < r_max:
                val = max(0.0, min(1.0, 1.0 - ((r - r_min) / (r_max - r_min))))
            else:
                val = 0.0
            sensor['valor'] = val


def update_proximity_from_poses(rob, robots_dict: dict):
    """Calcula lecturas de proximidad IR geométricas frente a paredes (3x3m) y otros Qupa."""
    max_range_m = 0.25
    min_range_m = 0.06
    for sensor in rob.prox_sensors:
        beam_ang = rob.yaw + math.radians(sensor['angulo'])
        cos_b = math.cos(beam_ang)
        sin_b = math.sin(beam_ang)
        min_dist = max_range_m

        if abs(cos_b) > 1e-4:
            for wall_x in (-1.5, 1.5):
                t = (wall_x - rob.x) / cos_b
                if 0.0 < t < min_dist:
                    min_dist = t
        if abs(sin_b) > 1e-4:
            for wall_y in (-1.5, 1.5):
                t = (wall_y - rob.y) / sin_b
                if 0.0 < t < min_dist:
                    min_dist = t

        for other_id, other in robots_dict.items():
            if other_id == rob.id:
                continue
            dx = other.x - rob.x
            dy = other.y - rob.y
            dist_centers = math.hypot(dx, dy) - 0.08
            if dist_centers < min_dist:
                ang_to_other = math.atan2(dy, dx)
                diff_ang = abs(math.degrees(_norm_rad(ang_to_other - beam_ang)))
                if diff_ang <= 25.0:
                    min_dist = max(min_range_m, dist_centers)

        if min_dist < max_range_m:
            sensor['valor'] = max(0.0, min(1.0, 1.0 - (min_dist - min_range_m) / (max_range_m - min_range_m)))
        else:
            sensor['valor'] = 0.0


class Sensors:
    ground                = staticmethod(ground)
    obtener_angulo_sensor = staticmethod(obtener_angulo_sensor)
    proximidad            = staticmethod(proximidad)
