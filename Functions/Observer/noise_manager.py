# Archivo: Functions/Observer/noise_manager.py
# Ubicación exacta:
#   ~/sim_ws_qupa/experimento_ks5/Functions/Observer/noise_manager.py

import math
import random


# ============================================================================
# FUNCIÓN PRIVADA: Generador de Distribución Normal (Algoritmo Box-Muller)
# ============================================================================
def gaussian(mean: float, desviacion_estandar: float) -> float:
    u1 = random.random()
    u2 = random.random()
    if u1 <= 0.0:
        u1 = 0.000001

    z0 = math.sqrt(-2.0 * math.log(u1)) * math.cos(2.0 * math.pi * u2)
    return z0 * desviacion_estandar + mean


# ============================================================================
# FUNCIÓN EXPORTADA: Aplica las perturbaciones configuradas a la lectura cruda
# ============================================================================
def aplicar_ruido(dist_real: float, angulo_real: float, cfg: dict):
    # 1. Si el ruido está apagado globalmente, devolver valores reales (Ground Truth)
    if not cfg.get('ENABLE_NOISE', True):
        return dist_real, angulo_real

    # 2. OUTLIERS (Fallos catastróficos aleatorios)
    if cfg.get('USE_OUTLIERS', False):
        if random.random() < cfg.get('OUTLIER_PROB', 0.01):
            return 1000.0, float(random.randint(-180, 180))

    dist_final = dist_real
    angulo_final = angulo_real

    # 3. RUIDO ESPACIAL 2D
    # Partimos del ruido constante mínimo del sensor
    radio_actual = cfg.get('NOISE_BASE_RADIUS', 0.0)

    # Si está activada la dependencia, el área de error crece con la distancia
    if cfg.get('USE_DISTANCE_DEPENDENT', True):
        radio_actual = radio_actual + (dist_real * cfg.get('NOISE_FACTOR', 0.1))

    # Obtener la desviación estándar a partir de las variaciones definidas
    sigma = radio_actual / cfg.get('NOISE_SIGMAS', 2.0)

    # Convertir a radianes para matemáticas cartesianas
    ang_rad = math.radians(angulo_real)

    # Proyectar la lectura polar a (X, Y)
    x_real = dist_real * math.cos(ang_rad)
    y_real = dist_real * math.sin(ang_rad)

    # Aplicar la campana de Gauss en ambos ejes con la sigma dinámica o estática
    x_ruido = x_real + gaussian(0.0, sigma)
    y_ruido = y_real + gaussian(0.0, sigma)

    # Volver a coordenadas polares
    dist_final = math.sqrt(x_ruido ** 2 + y_ruido ** 2)
    angulo_final = math.degrees(math.atan2(y_ruido, x_ruido))

    # 4. BIAS (Sesgo sistemático constante del sensor)
    if cfg.get('USE_BIAS', True):
        dist_final = dist_final + cfg.get('BIAS_OFFSET', 0.0)

    # Corrección física final: la distancia no puede ser negativa
    if dist_final < 0.0:
        dist_final = 0.0

    return dist_final, angulo_final


class NoiseManager:
    _gaussian = staticmethod(gaussian)
    gaussian = staticmethod(gaussian)
    aplicar_ruido = staticmethod(aplicar_ruido)