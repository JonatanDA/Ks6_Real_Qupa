# ============================================================================
# Archivo: Functions/Observer/BehaviorSimilarity.py
# Módulo de Similitud entre Comportamientos (Espejo 1:1 con BehaviorSimilarity.lua)
# ============================================================================

import math
from Functions import math_helper

# ============================================================================
# CONFIGURACIÓN DEL MÓDULO
# ============================================================================
CLOSURE_THRESHOLD_Q = 15.0  # Distancia máxima para considerar figura cerrada (cm)
WEIGHT_L = 1.0
WEIGHT_A = 1.0
WEIGHT_S = 1.0

# Método para fusionar comportamientos repetidos
COMBINE_METHOD = "median"  # Opciones: "median" (recomendado) o "mean"


# ============================================================================
# FUNCIONES ESTADÍSTICAS Y DE CONSULTA
# ============================================================================
def get_combine_method():
    return "mediana" if COMBINE_METHOD == "median" else "media"


def _calculate_stat(t):
    return math_helper.calculate_stat(t, COMBINE_METHOD)


get_val_type = math_helper.get_val_type


# ============================================================================
# FUNCIONES AUXILIARES (PREPARACIÓN VECTORIAL, CIERRE FÍSICO Y TRADUCCIÓN)
# ============================================================================

# Asegura que la entrada esté en formato vectorial { 'type': 0/1, 'value': [...], 'hist': [...] }
def ensure_vector_struct(input_data):
    if not input_data:
        return {'type': 0, 'value': [], 'hist': []}
    if isinstance(input_data, dict) and 'value' in input_data and input_data.get('type') is not None:
        copy_vals = [float(v) for v in input_data['value']]
        copy_hist = math_helper.clone_hist(input_data.get('hist'), copy_vals)
        return {
            'type': input_data['type'],
            'value': copy_vals,
            'hist': copy_hist
        }
    # Retrocompatibilidad si llega una lista de pares [[dist, ang], ...]
    vals = []
    if isinstance(input_data, (list, tuple)):
        for i, p in enumerate(input_data):
            if i > 0:
                vals.append(float(p[1]) if len(p) > 1 else 0.0)
            vals.append(float(p[0]) if len(p) > 0 else 0.0)
    vec = {'type': 0, 'value': vals}
    math_helper.init_vector_history(vec)
    return vec


# Evalúa si un vector cierra físicamente en el origen (<= CLOSURE_THRESHOLD_Q).
# Si es cíclico y tiene la misma apertura y cierre (ej. empieza en D y termina en D),
# inyecta el elemento de cierre h al final para que quede con cantidad par alternada (D,A,...,D,A) y pueda rotar modularmente.
def prepare_cyclic_vector(vec_struct):
    vec = ensure_vector_struct(vec_struct)
    n = len(vec['value'])
    if n == 0:
        return vec, False

    x, y, theta_deg, dist_count, _ = math_helper.integrate_vector_2d(vec)
    dist_to_origin = math.sqrt(x * x + y * y)
    is_cyclic = (dist_count >= 3) and (dist_to_origin <= CLOSURE_THRESHOLD_Q)

    if is_cyclic:
        first_type = get_val_type(vec['type'], 0)
        last_type = get_val_type(vec['type'], n - 1)

        if first_type == "D" and last_type == "D":
            # Empezó en distancia (orientación 0°) y terminó en distancia: falta el giro final h para reorientarse con d1
            h = math_helper.normalize_deg_180(-theta_deg)
            vec['value'].append(h)
            vec['hist'].append([h])
        elif first_type == "A" and last_type == "D":
            # Empezó con un ángulo a1 (theta inicial = a1 antes de d1) y terminó en distancia:
            # Para cerrar el ciclo orientándose hacia d1, el giro total del polígono cerrado (incluyendo a1) debe ser 0 mod 360
            h = math_helper.normalize_deg_180(-theta_deg)
            vec['value'][0] = h
            vec['hist'][0].append(h)

    return vec, is_cyclic


# Traduce un vector {'type': 0/1, 'value': [...]} a pares físicos [[dist, ang], ...] aplicando las 4 reglas
def translate_to_physical(vec_struct):
    return math_helper.translate_to_physical(vec_struct)


# Evalúa Ql y Qa entre una ventana alineada del vector largo y el vector corto (ambos de longitud Ns y misma paridad de tipos)
def compute_window_metrics(window_vals, short_vals, short_type, compare_last_angle=True):
    Ns = len(short_vals)
    sum_l_o, sum_diff_l = 0.0, 0.0
    num_dist_compared = 0

    sum_a_o, sum_diff_a = 0.0, 0.0
    num_ang_compared = 0

    for m in range(Ns):
        v_type = get_val_type(short_type, m)
        if v_type == "D":
            sum_l_o = sum_l_o + abs(window_vals[m])
            sum_diff_l = sum_diff_l + abs(window_vals[m] - short_vals[m])
            num_dist_compared = num_dist_compared + 1
        else:
            if m < (Ns - 1) or compare_last_angle:
                sum_a_o = sum_a_o + abs(window_vals[m])
                sum_diff_a = sum_diff_a + abs(window_vals[m] - short_vals[m])
                num_ang_compared = num_ang_compared + 1

    Ql = 1.0
    if num_dist_compared > 0:
        Ql = (1.0 - (sum_diff_l / sum_l_o)) if (sum_l_o > 0) else (1.0 if (sum_diff_l == 0) else 0.0)

    Qa = 1.0
    if num_ang_compared > 0:
        Qa = (1.0 - (sum_diff_a / sum_a_o)) if (sum_a_o > 0) else (1.0 if (sum_diff_a == 0) else 0.0)

    return Ql, Qa


# ============================================================================
# FUNCIONES PRINCIPALES
# ============================================================================
def evaluate(orig_raw, imit_raw, umbral_similitud=None):
    if not orig_raw or not imit_raw:
        return 0.0, {'offset': 1, 'es_recorte': False}

    # Obtenemos vectores preparados y su topología (abierta/cerrada con cierre h si aplica)
    orig, orig_cyclic = prepare_cyclic_vector(orig_raw)
    imit, imit_cyclic = prepare_cyclic_vector(imit_raw)

    No, Nc = len(orig['value']), len(imit['value'])
    if No == 0 or Nc == 0:
        return 0.0, {'offset': 1, 'es_recorte': False}

    umbral = umbral_similitud if umbral_similitud is not None else 0.80
    Qs = 1.0 - (abs(Nc - No) / max(No, Nc))

    # Definir qué secuencia desliza sobre cuál (siempre la corta sobre la larga)
    if Nc > No:
        seq_long, seq_short = imit, orig
        is_long_cyclic, is_short_cyclic = imit_cyclic, orig_cyclic
        Nl, Ns = Nc, No
    else:
        seq_long, seq_short = orig, imit
        is_long_cyclic, is_short_cyclic = orig_cyclic, imit_cyclic
        Nl, Ns = No, Nc

    # Paridad de tipo de dato (1-based como en Lua para mantener el mismo 'offset' en info_alin):
    # Si seq_long y seq_short empiezan con el mismo tipo (ambos Dist o ambos Ang), el offset empieza en 1 (1, 3, 5...).
    # Si empiezan con distinto tipo (uno Dist y otro Ang), el offset empieza en 2 (2, 4, 6...).
    start_offset = 1 if (seq_long['type'] == seq_short['type']) else 2

    # Si la larga es cíclica, la ventana puede empezar en cualquier posición de la misma paridad (1..Nl) y dar la vuelta.
    # Si NO es cíclica, la ventana solo puede deslizar hasta que el final de seq_short toque el borde derecho de seq_long.
    max_offset = Nl if is_long_cyclic else (Nl - Ns + 1)

    best_qi = -float('inf')
    best_geom_qi = -float('inf')
    best_offset = start_offset

    for offset in range(start_offset, max_offset + 1, 2):
        window_vals = []
        valid_window = True

        for m in range(1, Ns + 1):
            idx_1 = offset + m - 1  # Índice 1-based idéntico a Lua
            if idx_1 > Nl:
                if is_long_cyclic:
                    idx_1 = ((idx_1 - 1) % Nl) + 1
                else:
                    valid_window = False
                    break

            idx_0 = idx_1 - 1
            m_0 = m - 1

            # Verificación estricta de paridad elemento por elemento (jamás comparar Dist con Ang)
            if get_val_type(seq_long['type'], idx_0) != get_val_type(seq_short['type'], m_0):
                valid_window = False
                break

            window_vals.append(seq_long['value'][idx_0])

        if valid_window and len(window_vals) == Ns:
            compare_last_ang = True
            Ql, Qa = compute_window_metrics(window_vals, seq_short['value'], seq_short['type'], compare_last_ang)

            # Qi con penalización de tamaño Qs (para secuencias de igual tamaño o evaluación global)
            Qi_full = (WEIGHT_L * Ql + WEIGHT_A * Qa + WEIGHT_S * Qs) / (WEIGHT_L + WEIGHT_A + WEIGHT_S)

            # Qi de subsegmento alineado (con Qs_sub = 1.0, ya que comparamos ventana de tamaño Ns contra Ns)
            Qi_sub = (WEIGHT_L * Ql + WEIGHT_A * Qa + WEIGHT_S * 1.0) / (WEIGHT_L + WEIGHT_A + WEIGHT_S)

            if Qi_sub > best_geom_qi:
                best_geom_qi = Qi_sub
                best_qi = Qi_full
                best_offset = offset

    if best_qi == -float('inf'):
        return 0.0, {'offset': 1, 'es_recorte': False}

    es_recorte = False
    # Si tienen diferente longitud pero la ventana alineada supera el umbral geométrico, es un recorte válido (Qs = 1.0 en la ventana)
    if No != Nc:
        if best_geom_qi >= umbral:
            best_qi = best_geom_qi
            es_recorte = True

    return best_qi, {'offset': best_offset, 'es_recorte': es_recorte}


def init_history(comp):
    if not comp:
        return comp
    if isinstance(comp, dict) and 'value' in comp and comp.get('type') is not None:
        return math_helper.init_vector_history(comp)
    return comp


def merge_behaviors(conocido, nuevo, info_alin):
    if not conocido or not nuevo:
        return conocido

    # Si la figura más larga era cíclica y durante evaluate se extendió con el ángulo de cierre h,
    # aseguramos que la versión extendida sea la que conserve el cierre si aplica
    conocido_prep, con_cyclic = prepare_cyclic_vector(conocido)
    nuevo_prep, nue_cyclic = prepare_cyclic_vector(nuevo)

    No, Nc = len(conocido_prep['value']), len(nuevo_prep['value'])
    if No == 0 or Nc == 0:
        return conocido

    min_len = min(No, Nc)
    default_offset = 1 if (conocido_prep['type'] == nuevo_prep['type']) else 2
    offset = (info_alin.get('offset') if info_alin else None) or default_offset

    if No >= Nc:
        # 'conocido' es más largo o igual: actualizamos 'conocido' in-place con el historial de 'nuevo'
        conocido['type'] = conocido_prep['type']
        conocido['value'] = conocido_prep['value']
        conocido['hist'] = conocido_prep['hist']
        N_base = len(conocido['value'])

        for m in range(1, min_len + 1):
            idx_largo_1 = offset + m - 1  # Índice 1-based idéntico a Lua
            if con_cyclic and idx_largo_1 > N_base:
                idx_largo_1 = ((idx_largo_1 - 1) % N_base) + 1

            idx_largo_0 = idx_largo_1 - 1
            m_0 = m - 1

            if idx_largo_1 <= N_base and get_val_type(conocido['type'], idx_largo_0) == get_val_type(nuevo_prep['type'], m_0):
                for val in nuevo_prep['hist'][m_0]:
                    conocido['hist'][idx_largo_0].append(val)

        for i in range(N_base):
            conocido['value'][i] = _calculate_stat(conocido['hist'][i])
    else:
        # 'nuevo' es más largo que 'conocido': 'nuevo' pasa a ser la base y absorbe el historial de 'conocido'
        N_base = len(nuevo_prep['value'])

        for m in range(1, min_len + 1):
            idx_largo_1 = offset + m - 1  # Índice 1-based idéntico a Lua
            if nue_cyclic and idx_largo_1 > N_base:
                idx_largo_1 = ((idx_largo_1 - 1) % N_base) + 1

            idx_largo_0 = idx_largo_1 - 1
            m_0 = m - 1

            if idx_largo_1 <= N_base and get_val_type(nuevo_prep['type'], idx_largo_0) == get_val_type(conocido_prep['type'], m_0):
                for val in conocido_prep['hist'][m_0]:
                    nuevo_prep['hist'][idx_largo_0].append(val)

        for i in range(N_base):
            nuevo_prep['value'][i] = _calculate_stat(nuevo_prep['hist'][i])

        conocido['type'] = nuevo_prep['type']
        conocido['value'] = nuevo_prep['value']
        conocido['hist'] = nuevo_prep['hist']

    return conocido


class BehaviorSimilarity:
    CLOSURE_THRESHOLD_Q = CLOSURE_THRESHOLD_Q
    WEIGHT_L = WEIGHT_L
    WEIGHT_A = WEIGHT_A
    WEIGHT_S = WEIGHT_S
    COMBINE_METHOD = COMBINE_METHOD

    get_combine_method = staticmethod(get_combine_method)
    ensure_vector_struct = staticmethod(ensure_vector_struct)
    prepare_cyclic_vector = staticmethod(prepare_cyclic_vector)
    translate_to_physical = staticmethod(translate_to_physical)
    compute_window_metrics = staticmethod(compute_window_metrics)
    evaluate = staticmethod(evaluate)
    init_history = staticmethod(init_history)
    merge_behaviors = staticmethod(merge_behaviors)


# Alias de compatibilidad por si otra parte del proyecto aún llama a ImitationQuality
ImitationQuality = BehaviorSimilarity