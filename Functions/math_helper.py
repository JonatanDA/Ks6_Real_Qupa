# Archivo: Math_Functions/math_helper.py

import math

# ---------------------------------------------------------
# ESTADÍSTICAS
# ---------------------------------------------------------
def get_window_stats(start_idx, win_size, xx, yy):
    sum_x, sum_y = 0.0, 0.0
    n = 0

    # 1. Calcular Centroide (Promedio)
    for j in range(start_idx, start_idx + win_size):
        if 0 <= j < len(xx) and 0 <= j < len(yy) and xx[j] is not None and yy[j] is not None:
            sum_x = sum_x + xx[j]
            sum_y = sum_y + yy[j]
            n = n + 1

    if n == 0:
        return 0.0, 0.0, 0.0
    avg_x = sum_x / n
    avg_y = sum_y / n

    # 2. Calcular Dispersión (Desviación Estándar combinada)
    sum_sq_dist = 0.0
    for j in range(start_idx, start_idx + win_size):
        if 0 <= j < len(xx) and 0 <= j < len(yy) and xx[j] is not None and yy[j] is not None:
            dx = xx[j] - avg_x
            dy = yy[j] - avg_y
            sum_sq_dist = sum_sq_dist + (dx * dx + dy * dy)

    # Retornamos: Desviación Estándar, Centroide X, Centroide Y
    return math.sqrt(sum_sq_dist / n), avg_x, avg_y


# Función para encontrar el umbral óptimo usando el Método de Otsu 1D
def otsu_1d(varianzas):
    if not varianzas or len(varianzas) == 0:
        return 0.0, 0.0

    # Encontrar min y max para crear el histograma
    min_v, max_v = varianzas[0], varianzas[0]
    for i in range(len(varianzas)):
        if varianzas[i] < min_v:
            min_v = varianzas[i]
        if varianzas[i] > max_v:
            max_v = varianzas[i]

    if min_v == max_v:
        return min_v, 0.0

    # Crear histograma aproximado (50 bins)
    num_bins = 50
    hist = [0] * num_bins

    bin_size = (max_v - min_v) / num_bins
    for i in range(len(varianzas)):
        bin_idx = int(math.floor((varianzas[i] - min_v) / bin_size))
        if bin_idx >= num_bins:
            bin_idx = num_bins - 1
        hist[bin_idx] = hist[bin_idx] + 1

    total_datos = len(varianzas)
    sum_total = 0.0
    for i in range(1, num_bins + 1):
        bin_val = min_v + (i - 0.5) * bin_size
        sum_total = sum_total + (bin_val * hist[i - 1])

    weight_bg = 0
    sum_bg = 0.0
    max_varianza_interclase = -1.0
    umbral_optimo = 0.0

    # Evaluar cada posible umbral (Otsu)
    for i in range(1, num_bins + 1):
        weight_bg = weight_bg + hist[i - 1]

        if weight_bg > 0:
            weight_fg = total_datos - weight_bg
            if weight_fg == 0:
                break

            bin_val = min_v + (i - 0.5) * bin_size
            sum_bg = sum_bg + (bin_val * hist[i - 1])

            mean_bg = sum_bg / weight_bg
            mean_fg = (sum_total - sum_bg) / weight_fg

            # Varianza Inter-clase
            var_between = weight_bg * weight_fg * (mean_bg - mean_fg) * (mean_bg - mean_fg)

            if var_between > max_varianza_interclase:
                max_varianza_interclase = var_between
                umbral_optimo = bin_val

    return umbral_optimo, max_varianza_interclase


# ---------------------------------------------------------
# OPERACIONES Y CÁLCULOS
# ---------------------------------------------------------
def polarACartesianoPAR(r, theta):
    y = r * math.sin(theta)
    x = r * math.cos(theta)
    return x, y


def cartesianoAPolarPAR(x, y):
    r = math.sqrt(x ** 2 + y ** 2)
    theta = math.atan2(y, x)
    return r, theta


def vec2_angle(v):
    if isinstance(v, dict):
        w, x, y, z = v['w'], v['x'], v['y'], v['z']
    else:
        w, x, y, z = v.w, v.x, v.y, v.z
    ang = math.atan2(2 * (w * z + x * y), 1 - 2 * (y ** 2 + z ** 2))
    ang = ang * 180 / math.pi
    return ang


# ---------------------------------------------------------
# FUNCIONES COMPARTIDAS PARA VECTORES, ESTADÍSTICAS Y GEOMETRÍA 2D
# ---------------------------------------------------------

# Normaliza un ángulo en grados al rango (-180, 180]
def normalize_deg_180(ang):
    while ang > 180:
        ang = ang - 360
    while ang <= -180:
        ang = ang + 360
    return ang


# Calcula la mediana de una lista numérica
def get_median(t):
    if not t or len(t) == 0:
        return 0.0
    temp = [v for v in t]
    temp.sort()
    n = len(temp)
    if n % 2 == 0:
        return (temp[(n // 2) - 1] + temp[n // 2]) / 2.0
    else:
        return temp[n // 2]


# Calcula la media aritmética de una lista numérica
def get_mean(t):
    if not t or len(t) == 0:
        return 0.0
    s = 0.0
    for v in t:
        s = s + v
    return s / len(t)


# Calcula el estadístico configurado ("median" por defecto o "mean")
def calculate_stat(t, method=None):
    m = method or "median"
    if m == "median":
        return get_median(t)
    else:
        return get_mean(t)


# Compara dos IDs de forma determinista (numérico ascendente primero, luego lexicográfico)
def compare_ids(id_a, id_b):
    sa = str(id_a if id_a is not None else "")
    sb = str(id_b if id_b is not None else "")
    if sa == sb:
        return False
    try:
        na = float(sa)
        has_na = True
    except ValueError:
        na = 0.0
        has_na = False
    try:
        nb = float(sb)
        has_nb = True
    except ValueError:
        nb = 0.0
        has_nb = False

    if has_na and has_nb:
        if na != nb:
            return na < nb
        return sa < sb
    elif has_na and not has_nb:
        return True
    elif not has_na and has_nb:
        return False
    else:
        return sa < sb


def id_sort_key(frag_id):
    s = str(frag_id if frag_id is not None else "")
    try:
        return (0, float(s), s)
    except ValueError:
        return (1, 0.0, s)


# Extrae y ordena las claves activas de una tabla tipo conjunto/diccionario de forma determinista
def sorted_keys(tbl):
    if not tbl:
        return []
    if isinstance(tbl, dict):
        keys = [k for k, v in tbl.items() if v]
    else:
        keys = list(tbl)
    keys.sort(key=id_sort_key)
    return keys


# Determina el tipo de dato ("D" = Distancia, "A" = Ángulo) en la posición 'index' de un vector con 'struct_type' (0=Dist, 1=Ang)
def get_val_type(struct_type, index):
    is_odd = (index % 2 == 0)
    if struct_type == 1:
        return "A" if is_odd else "D"
    else:
        return "D" if is_odd else "A"


def fmt_f(val, decimals=2):
    return f"{val:.{decimals}f}".replace('.', ',')


# Formatea una estructura vectorial {type, value, count} para logs
def fmt_struct(s):
    if not s or 'value' not in s or s['value'] is None:
        return "nil"
    vals = [fmt_f(v, 2) for v in s['value']]
    t_str = "Ang" if (s.get('type') == 1) else "Dist"
    count = s.get('count')
    count_str = f" [x{count}]" if (count and count > 1) else ""
    return f"({t_str}){count_str} {{{', '.join(vals)}}}"


# Formatea una lista plana de valores numéricos separados por coma
def simple_fmt(val_table):
    if not val_table:
        return ""
    vals = [fmt_f(v, 2) for v in val_table]
    return ",".join(vals)


# Inicializa o asegura que un vector {type, value} tenga su tabla paralela de historial por elemento: hist = [ [v1], [v2], ... ]
def init_vector_history(vec_struct):
    if not vec_struct or 'value' not in vec_struct or vec_struct['value'] is None:
        return vec_struct
    if 'hist' not in vec_struct or vec_struct['hist'] is None:
        vec_struct['hist'] = []
    for i in range(len(vec_struct['value'])):
        if i >= len(vec_struct['hist']) or not vec_struct['hist'][i] or len(vec_struct['hist'][i]) == 0:
            if i < len(vec_struct['hist']):
                vec_struct['hist'][i] = [float(vec_struct['value'][i])]
            else:
                vec_struct['hist'].append([float(vec_struct['value'][i])])
    return vec_struct


# Clona profundamente una tabla de historial por elemento
def clone_hist(hist, fallback_values=None):
    new_hist = []
    if hist:
        for h_list in hist:
            copy_list = []
            if h_list:
                for val in h_list:
                    copy_list.append(float(val))
            new_hist.append(copy_list)
    if fallback_values:
        for i in range(len(fallback_values)):
            if i >= len(new_hist):
                new_hist.append([float(fallback_values[i])])
            elif not new_hist[i] or len(new_hist[i]) == 0:
                new_hist[i] = [float(fallback_values[i])]
    return new_hist


# Verifica si 'short_val' encaja dentro de 'long_val' a partir de 'start_idx' (0-based) respetando paridad de tipo y tolerancias
def check_vector_subset_match(long_val, short_val, start_idx, long_type, short_type, cfg, tol_mult=None):
    mult = tol_mult if tol_mult is not None else 1.0
    if get_val_type(long_type, start_idx) != get_val_type(short_type, 0):
        return False
    tol_cfg = cfg.get('CONFIG', {}) if isinstance(cfg, dict) else {}
    tol_a = tol_cfg.get('TOL_ANG', 15.0)
    tol_d = tol_cfg.get('TOL_DIST', 15.0)
    for k in range(len(short_val)):
        l_idx = start_idx + k
        v_type = get_val_type(short_type, k)
        diff = abs(long_val[l_idx] - short_val[k])
        tol = (tol_a if (v_type == "A") else tol_d) * mult
        if diff > tol:
            return False
    return True


# Integra directamente la trayectoria 2D (x, y, theta_deg) recorriendo un vector {type, value}
# Retorna: x_final, y_final, theta_deg_final, num_dist_segments, step_history
def integrate_vector_2d(vec_struct):
    if not vec_struct or 'value' not in vec_struct or vec_struct['value'] is None:
        return 0.0, 0.0, 0.0, 0, []
    values = vec_struct['value']
    s_type = vec_struct.get('type', 0)
    x, y = 0.0, 0.0
    theta_deg = 0.0
    dist_count = 0
    step_history = []

    for i in range(len(values)):
        v_type = get_val_type(s_type, i)
        val = values[i]
        if v_type == "A":
            theta_deg = theta_deg + val
        else:
            theta_rad = math.radians(theta_deg)
            x = x + val * math.cos(theta_rad)
            y = y + val * math.sin(theta_rad)
            dist_count = dist_count + 1
        dist_origin = math.sqrt(x * x + y * y)
        step_history.append({
            'idx': i,
            'v_type': v_type,
            'x': x,
            'y': y,
            'theta_deg': theta_deg,
            'dist_origin': dist_origin,
            'dist_count': dist_count
        })

    return x, y, theta_deg, dist_count, step_history


# Traduce un vector puro {type = 0/1, value = [...]} a la tabla de pares físicos [[dist, ang], ...]
def translate_to_physical(vec_struct):
    if not vec_struct:
        return []
    if not isinstance(vec_struct, dict) or 'value' not in vec_struct:
        if isinstance(vec_struct, list) and len(vec_struct) > 0 and isinstance(vec_struct[0], (list, tuple)):
            return vec_struct
        return []

    vals = vec_struct['value']
    n = len(vals)
    if n == 0:
        return []

    pairs_list = []
    if vec_struct.get('type') == 1:
        # Inicia con Ángulo (ang: ...)
        i = 0
        while i < n:
            ang = vals[i]
            dist = vals[i + 1] if (i + 1 < n) else 0.0
            pairs_list.append([dist, ang])
            i = i + 2
    else:
        # Inicia con Distancia (dis: ...)
        pairs_list.append([vals[0], 0.0])
        i = 1
        while i < n:
            ang = vals[i]
            dist = vals[i + 1] if (i + 1 < n) else 0.0
            pairs_list.append([dist, ang])
            i = i + 2

    return pairs_list