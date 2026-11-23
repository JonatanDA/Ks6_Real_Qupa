# Archivo: Ks6/Functions/utils.py
# Ubicación: ~/sim_ws_qupa/Ks6/Functions/utils.py
# Descripción:
#   Módulo compartido de utilidades para los Qupa en ROS 2 (Simulación y Hardware Real):
#   temporizadores (Temporizador_1, Temporizador_2), control de LEDs (setleds con soporte
#   para servicio físico APA102 /<robot_id>/set y Gazebo), determinación de rol,
#   diccionario de figuras geométricas (original_params) y funciones auxiliares de impresión.
#   Equivalente 1:1 de Ks6/Functions/utils.lua.

import json
import math
import random

from Functions import shared_config

try:
    from std_msgs.msg import String
except ImportError:
    String = None

try:
    from qupa_msgs.srv import LEDCommand
    HAS_QUPA_LED_SRV = True
except ImportError:
    LEDCommand = None
    HAS_QUPA_LED_SRV = False

try:
    from gz.msgs10.material_color_pb2 import MaterialColor
    from gz.msgs10.entity_pb2 import Entity
    HAS_GZ_MESSAGES = True
except ImportError:
    MaterialColor = None
    Entity = None
    HAS_GZ_MESSAGES = False


def norm_rad(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


# ============================================================================
# 1. TEMPORIZADORES
# ============================================================================
temp  = True
cr    = 1
crono = 0


def Temporizador_1(rob, c_seconds: float, ticks_per_second: float = None):
    global temp, cr, crono
    if ticks_per_second is None:
        ticks_per_second = float(shared_config.get_controller_var("ticks_per_second", getattr(rob, "ticks_per_second", 4.0)))
    c_ticks = c_seconds * ticks_per_second
    rob.crono = c_ticks - rob.cr
    rob.cr += 1
    if rob.crono <= 0:
        rob.temp = False
        rob.cr = 1
    temp, cr, crono = rob.temp, rob.cr, rob.crono


def r_Temporizador_1(rob):
    global temp, cr, crono
    rob.temp  = True
    rob.cr    = 1
    rob.crono = 0
    temp, cr, crono = True, 1, 0


temp_2  = True
cr_2    = 1
crono_2 = 0


def Temporizador_2(rob, d_seconds: float, ticks_per_second: float = None):
    global temp_2, cr_2, crono_2
    if ticks_per_second is None:
        ticks_per_second = float(shared_config.get_controller_var("ticks_per_second", getattr(rob, "ticks_per_second", 4.0)))
    d_ticks = d_seconds * ticks_per_second
    rob.crono_2 = d_ticks - rob.cr_2
    rob.cr_2 += 1
    if rob.crono_2 <= 0:
        rob.temp_2 = False
        rob.cr_2 = 1
    temp_2, cr_2, crono_2 = rob.temp_2, rob.cr_2, rob.crono_2


def r_Temporizador_2(rob):
    global temp_2, cr_2, crono_2
    rob.temp_2  = True
    rob.cr_2    = 1
    rob.crono_2 = 0
    temp_2, cr_2, crono_2 = True, 1, 0


# ============================================================================
# 2. CONTROL DE LEDS Y ROL
# ============================================================================
def setleds(
    rob,
    color: str,
    led_pub=None,
    gz_mat_pub=None,
    force_topic_pub: bool = False,
    led_srv_client=None,
):
    changed = (rob.led_color != color)
    rob.led_color = color

    # 1) Tópico de simulación (/leds/color)
    if (changed or force_topic_pub) and led_pub is not None and String is not None:
        msg = String()
        msg.data = json.dumps({"color": color})
        led_pub.publish(msg)

    # 2) Servicio físico del Qupa real (/<robot_id>/set -> qupa_msgs/srv/LEDCommand)
    # Se invoca únicamente cuando el color cambia para no saturar el bus SPI0 de la Raspberry Pi.
    # Nota: "yellow" (color lógico de demostración del Maestro en ARGoS3) se mapea a RGB Azul [0, 0, 255]
    # porque camera_node.py del Qupa real detecta el rango HSV BLUE y apprentice_entries lo traduce a "yellow".
    if changed and led_srv_client is not None and HAS_QUPA_LED_SRV:
        try:
            hw_rgb_map = {
                "yellow": [0, 0, 255],
                "blue":   [0, 0, 255],
                "green":  [0, 255, 0],
                "red":    [255, 0, 0],
            }
            req = LEDCommand.Request()
            if color in hw_rgb_map:
                req.command = json.dumps({"mode": "set_all", "rgb": hw_rgb_map[color]})
            else:
                req.command = json.dumps({"mode": "clear"})
            if led_srv_client.service_is_ready():
                led_srv_client.call_async(req)
        except Exception:
            pass

    # 3) MaterialColor para visualización en Gazebo
    if (changed or force_topic_pub) and gz_mat_pub is not None and HAS_GZ_MESSAGES:
        try:
            palette = {
                "yellow": (0.0, 0.35, 1.0, 1.0),
                "blue":   (0.0, 0.35, 1.0, 1.0),
                "red":    (1.0, 0.02, 0.02, 1.0),
                "green":  (0.0, 0.95, 0.10, 1.0),
                "black":  (0.18, 0.18, 0.18, 1.0),
            }
            r, g, b, a = palette.get(color, (0.18, 0.18, 0.18, 1.0))
            visual_targets = [
                f"{rob.id}::led_beacon_visual",
                f"{rob.id}/led_beacon_link_visual",
                f"{rob.id}/acrylic_dome_link_visual",
            ]
            for v_name in visual_targets:
                mc = MaterialColor()
                mc.entity_match = getattr(MaterialColor.EntityMatch, 'ALL', 1)
                if Entity is not None and hasattr(Entity, 'VISUAL'):
                    mc.entity.type = Entity.VISUAL
                mc.entity.name = v_name
                mc.ambient.r, mc.ambient.g, mc.ambient.b, mc.ambient.a = r, g, b, a
                mc.diffuse.r, mc.diffuse.g, mc.diffuse.b, mc.diffuse.a = r, g, b, a
                mc.emissive.r, mc.emissive.g, mc.emissive.b, mc.emissive.a = r, g, b, a
                gz_mat_pub.publish(mc)
        except Exception:
            pass


is_master = 0


def determinar_rol(rob, log_fn=print) -> int:
    global is_master
    randoms = random.randint(0, 2)
    if getattr(rob, 'role_cfg', ''):
        rob.is_master = 0 if rob.role_cfg.lower() == "apprentice" else 1
    else:
        rob.is_master = 1 if rob.id != "Qupa4" else 0

    is_master = rob.is_master
    if rob.is_master == 1:
        log_fn(f"The {rob.id} is master = {randoms}")
    else:
        log_fn(f"The {rob.id} is Observador= {randoms}")
    return rob.is_master


# ============================================================================
# 3. PATRONES PREDEFINIDOS DE LOS MAESTROS (original_params)
# ============================================================================
def asignar_figura_por_id(robot_id: str, figura_actual: str = "", distance_factor: float = 1.0):
    if figura_actual:
        return figura_actual, distance_factor
    if robot_id == "Qupa4":
        return "", 1.0
    if robot_id == "Qupa6":
        return "triangulo_irregular_1", 1.0
    if robot_id == "Qupa7":
        return "triangulo_regular", 1.0
    if robot_id in ("Qupa2", "Qupa3", "Qupa5"):
        return "octagono", 1.0
    if robot_id in ("Qupa0", "Qupa1"):
        return "Pentágono_irregular", 1.0
    # Respaldo para IDs de robots físicos reales (ej. qupa_D2, qupa_3A, etc.) cuando actúan como Maestro
    return "Pentágono_irregular", distance_factor


def original_params(figura: str, distance_factor: float = 1.0) -> dict:
    if figura == "triangulo_regular":
        return {
            'name': "triangulo",
            'params': [
                [distance_factor * 10.0, 60.0],
                [distance_factor * 10.0, 120.0],
                [distance_factor * 10.0, 120.0],
                [0.0, 60.0],
            ],
        }
    if figura == "linea":
        return {
            'name': "triangulo",
            'params': [
                [distance_factor * 20.0, 0.0],
                [0.0, 180.0],
            ],
        }
    if figura == "octagono":
        return {
            'name': "octagono",
            'params': [
                [distance_factor * 20.0, -45.0],
                [distance_factor * 20.0, -45.0],
                [distance_factor * 20.0, -45.0],
                [distance_factor * 20.0, -45.0],
                [distance_factor * 20.0, -45.0],
                [distance_factor * 20.0, -45.0],
                [distance_factor * 20.0, -45.0],
                [distance_factor * 20.0, -45.0],
                [0.0, 20.0],
                [0.0, -20.0],
            ],
        }
    if figura == "triangulo_irregular_1":
        return {
            'name': "Triangulo irregular 1",
            'params': [
                [40.0, 70.0],
                [30.0, 90.0],
                [50.0, 127.0],
                [0.0, 73.0],
            ],
        }
    if figura == "Pentágono_irregular":
        return {
            'name': "Pentágono_irregular",
            'params': [
                [0.0, -30.0],
                [0.0, 30.0],
                [distance_factor * 33.33, 0.0],
                [distance_factor * 33.33, 90.0],
                [distance_factor * 33.33, 53.0],
                [distance_factor * 33.33, 74.0],
                [distance_factor * 38.867, 84.0],
                [0.0, 59.0],
            ],
        }
    if figura == "Heptágono":
        return {
            'name': "Heptágono",
            'params': [
                [0.0, -45.0],
                [0.0, 45.0],
                [50.0, 0.0],
                [40.0, 90.0],
                [28.0, 45.0],
                [30.0, 45.0],
                [28.0, 45.0],
                [20.0, 45.0],
                [28.0, 45.0],
                [0.0, 45.0],
            ],
        }
    if figura == "trayectoria_1":
        return {
            'name': "Trayectoria 1",
            'params': [
                [40.0, 95.0],
                [50.0, -45.0],
                [30.0, -90.0],
                [20.0, 110.0],
                [20.0, -30.0],
                [0.0, -180.0],
            ],
        }
    return {'name': "Desconocido", 'params': []}


obtener_patron_original = original_params


def formatear_patron_original_tracker(robot_id: str, figura: str, distance_factor: float = 1.0) -> str:
    def split_num(val: float):
        sign = "-" if val < 0 else ""
        abs_val = abs(val)
        int_part = int(math.floor(abs_val))
        dec_part = int(math.floor((abs_val - int_part) * 100.0 + 0.5))
        if dec_part >= 100:
            int_part += 1
            dec_part -= 100
        return f"{sign}{int_part}", f"{dec_part:02d}"

    data = original_params(figura, distance_factor)
    params = data.get('params', [])
    if not params:
        return f"{robot_id} & Error: No se encontró un patrón asignado."

    nombre_patron = data.get('name', 'Desconocido')
    resultado = f"{robot_id} & Original&{nombre_patron}"
    pasos = []
    for dist, ang in params:
        dist_i, dist_d = split_num(dist)
        ang_i, ang_d = split_num(ang)
        pasos.append(f"({ang_i},{ang_d},{dist_i},{dist_d})")
    return resultado + ",".join(pasos)


# ============================================================================
# 4. IMPRESIÓN DE TABLAS Y POSICIONES
# ============================================================================
def print_table(tbl: dict, log_fn=print):
    for key, value in tbl.items():
        if isinstance(value, dict):
            log_fn(f"{key}:")
            print_table(value, log_fn)
        else:
            log_fn(f"{key} = {value}")


def imprimir_posicion(tabla: dict, full_pattern_params: list = None, log_fn=print):
    if full_pattern_params:
        for idx, seg in enumerate(full_pattern_params, start=1):
            log_fn(f"Segmento {idx} -> Distancia: {seg[0]:.2f} cm, ÁnguloRel: {seg[1]:.2f}°")
    for nombre, posiciones in tabla.items():
        log_fn(f"{nombre} -> ")
        for i, pos in enumerate(posiciones, start=1):
            log_fn(f"  Posición {i}: x = {pos[0]}, y = {pos[1]}")


# ============================================================================
# 5. GESTOR DE FUNCIONES DE IMPRESIÓN DE TRACKERS Y EVALUADORES
# ============================================================================
class TrackerPrinter:
    """Centraliza las funciones Print_tracker_* y evaluator_* compartidas entre controladores."""

    def __init__(self, active_funcs=None, robots_dict=None):
        self.robots = robots_dict if robots_dict is not None else {}
        if active_funcs is None:
            self.enable_print_python = True
            self.enable_print_paciencia = True
            self.enable_print_trayectory = True
            self.enable_print_al_pid = False
            self.enable_print_calidad = True
            self.enable_print_varianza = False
            self.enable_eval_otsu = False
            self.enable_eval_ruido = False
        else:
            active_set = set(active_funcs)
            self.enable_print_python = ("Print_tracker_python" in active_set)
            self.enable_print_paciencia = ("Print_tracker_paciencia" in active_set)
            self.enable_print_trayectory = ("Print_tracker_trayectory_data" in active_set)
            self.enable_print_al_pid = ("Print_tracker_AL_PID" in active_set)
            self.enable_print_calidad = ("Print_tracker_calidad_imitacion_data" in active_set)
            self.enable_print_varianza = ("Print_tracker_varianza" in active_set)
            self.enable_eval_otsu = ("evaluator_otsu" in active_set)
            self.enable_eval_ruido = ("evaluator_ruido" in active_set)

    def _emit(self, line: str, is_init: bool = False):
        shared_config.emit_telemetry(line, is_init=is_init)

    def Print_tracker_python(self, obs_ang_offset_global, x_mark, y_mark, yellow_blobs, green_blobs):
        if not self.enable_print_python:
            return
        yb_str = "|".join(f"{b['angle']}:{b['distance']}:{b.get('type', 2)}" for b in (yellow_blobs or []))
        gb_str = "|".join(f"{b['angle']}:{b['distance']}" for b in (green_blobs or []))
        self._emit(
            f"[python];observation;{obs_ang_offset_global};{x_mark};{y_mark};{yb_str};{gb_str}"
        )

    def Print_tracker_paciencia(self, current_patience_seg: float):
        if not self.enable_print_paciencia:
            return
        self._emit(f"[PACIENCIA]({current_patience_seg}) segundos.")

    def Print_tracker_trayectory_data(self, p1, p2=None, p3=None, p4=None, p5=None, p6=None, p7=None, p8=None):
        if not self.enable_print_trayectory:
            return
        app = self.robots.get("Qupa4")
        if app is None and len(self.robots) == 1:
            app = next(iter(self.robots.values()))
        rx = app.x if app is not None else 0.0
        ry = app.y if app is not None else 0.0
        m_idx = app.master_indice if app is not None else ""

        if not isinstance(p1, str) or (p1 != "trajectory" and p1 != "Active_learning"):
            k, x, y = p1, p2, p3
            if isinstance(k, (int, float)):
                self._emit(f"Master{int(k)} & {x} , {y}")
            elif isinstance(k, str):
                if k == "Qupa4":
                    self._emit(f"Apprentice & {rx} , {ry}")
                elif k == "App":
                    self._emit(f"Patrón{m_idx} & {rx} , {ry}")
                else:
                    self._emit(f"{k} & {rx} , {ry}")
            return

        if p1 == "trajectory":
            k, x, y = p2, p3, p4
            if isinstance(k, (int, float)):
                self._emit(f"[trajectory];Master{int(k)};{x};{y}")
            elif isinstance(k, str):
                if k == "Qupa4":
                    self._emit(f"[trajectory];Apprentice;{rx};{ry}")
                elif k == "App":
                    self._emit(f"[trajectory];Patrón{m_idx};{rx};{ry}")
                else:
                    self._emit(f"[trajectory];{k};{rx};{ry}")

        elif p1 == "Active_learning":
            al_mode, name, x, y, extra1, extra2, extra3 = p2, p3, p4, p5, p6, p7, p8
            if isinstance(name, str) and name == "Qupa4":
                x = rx
                y = ry
                name = "Apprentice"
            out = f"[active_learning];{al_mode};{name};{x};{y}"
            if extra1 is not None:
                out += f";{extra1}"
            if extra2 is not None:
                out += f";{extra2}"
            if extra3 is not None:
                out += f";{extra3}"
            self._emit(out)

    def Print_tracker_AL_PID(self, target: float, actual: float, control_vars: dict):
        if not self.enable_print_al_pid:
            return
        s = f"[PID];{target:.4f};{actual:.4f}"
        for name, value in (control_vars or {}).items():
            s += f";{name}={value:.4f}"
        self._emit(s)

    def Print_tracker_calidad_imitacion_data(self, linea_resumen: str, is_master: int = 0):
        if not self.enable_print_calidad:
            return
        if is_master == 0:
            self._emit(linea_resumen, is_init=False)
        else:
            rob = self.robots.get(linea_resumen)
            if rob is not None:
                self._emit(
                    formatear_patron_original_tracker(rob.id, rob.figura, rob.distance_factor),
                    is_init=True,
                )
            else:
                self._emit(linea_resumen, is_init=True)

    def Print_tracker_varianza(self, k: int, var: float):
        if not self.enable_print_varianza:
            return
        self._emit(f"Maestro{k} & {var}")

    def evaluator_otsu(self, separacion: float, avance: float):
        if not self.enable_eval_otsu:
            return
        self._emit(f"Otsu & {separacion} , {avance}")

    def evaluator_ruido(self, raw_angle: float, raw_dist: float, noisy_angle: float, noisy_dist: float):
        if not self.enable_eval_ruido:
            return
        self._emit(f"[EVALUATOR_RUIDO]Blob : angulo = {raw_angle}, distancia = {raw_dist}")
        self._emit(f"[EVALUATOR_RUIDO]Blob ruidoso : angulo = {noisy_angle}, distancia = {noisy_dist}")


# ============================================================================
# 6. ESTRUCTURA DE ESTADO FÍSICO DEL ROBOT QUPA
# ============================================================================
class QupaRobotInstance:
    """Representa el estado físico de cada robot Qupa y enlaza los métodos compartidos de Functions/."""

    def __init__(
        self,
        robot_id: str,
        init_x: float,
        init_y: float,
        init_yaw_deg: float,
        figura: str = "",
        role: str = "",
        dist_sens=None,
        d_speed: float = None,
        g_speed: float = None,
        ticks_per_second: float = None,
        interwheel_distance: float = None,
    ):
        from Functions import move as move_mod
        from Functions import sensors as sensors_mod

        self._move_mod = move_mod
        self._sensors_mod = sensors_mod

        if dist_sens is None:
            dist_sens = shared_config.get_controller_var(
                "Distribucion_sensores_de_proximidad", [0, -45, 45, -90, 90, 180]
            )
        if d_speed is None:
            d_speed = shared_config.get_controller_var("d_speed", 30.0)
        if g_speed is None:
            g_speed = shared_config.get_controller_var("g_speed", 80.0)
        if ticks_per_second is None:
            ticks_per_second = shared_config.get_controller_var("ticks_per_second", 4.0)
        if interwheel_distance is None:
            interwheel_distance = shared_config.get_controller_var("FOOTBOT_INTERWHEEL_DISTANCE", 14.0)

        self.id = robot_id
        self.role_cfg = role
        if role:
            self.is_master = 0 if role.lower() == "apprentice" else 1
        else:
            self.is_master = 0 if robot_id == "Qupa4" else 1

        df_cfg = float(shared_config.get_controller_var("distance_factor", 1.0))
        self.figura, self.distance_factor = asignar_figura_por_id(robot_id, figura, df_cfg)
        self.ticks_per_second = float(ticks_per_second)
        self.interwheel_distance = float(interwheel_distance)
        self.d_speed = float(d_speed)
        self.g_speed = float(g_speed)

        self.init_x = init_x
        self.init_y = init_y
        self.init_yaw = math.radians(init_yaw_deg)
        self.x = init_x
        self.y = init_y
        self.yaw = self.init_yaw
        self.odom_received = False
        self.odom_origin_pos = (0.0, 0.0)
        self.odom_origin_yaw = 0.0

        self.cycle_origin_x = init_x
        self.cycle_origin_y = init_y
        self.cycle_origin_yaw = self.init_yaw

        self.led_color = "black"
        self.base_ground = []
        self.motor_ground = []
        self.latest_detections = []

        if isinstance(dist_sens, list) and len(dist_sens) > 0:
            self.prox_sensors = [
                {'id': idx + 1, 'valor': 0.0, 'angulo': float(ang)}
                for idx, ang in enumerate(dist_sens)
            ]
        else:
            self.prox_sensors = [
                {'id': 1, 'valor': 0.0, 'angulo': 0.0},
                {'id': 2, 'valor': 0.0, 'angulo': -45.0},
                {'id': 3, 'valor': 0.0, 'angulo': 45.0},
                {'id': 4, 'valor': 0.0, 'angulo': -90.0},
                {'id': 5, 'valor': 0.0, 'angulo': 90.0},
                {'id': 6, 'valor': 0.0, 'angulo': 180.0},
            ]

        self.r_move()
        self.r_Temporizador_1()
        self.r_Temporizador_2()

        self.RW_b1 = 0
        self.Rw_target_angle = 0.0
        self.RW_stuck_counter = 0
        self.RW_clear_path_timer = 0

        self.d1 = 2
        self.master_avg_dist = 0.0
        self.EAEP_c1 = 0
        self.indice = 1
        self.PI_end = False
        self.PI_counter_seg = 0
        self.master_indice = 0
        self.parametros_elegidos = []
        self.posiciones_guardadas = {}

        self.srt_c1 = 0
        self.motion_active = False
        self.last_motion_marker = [0.0, 0.0, float(d_speed), float(g_speed)]
        self.x_mark = 0.0
        self.y_mark = 0.0
        self.current_wait_ticks = 0
        self.obs_stop_flag = False
        self.al_current_state = "IDLE"
        self.al_evasion_queue = []
        self.al_pause_timer = 0
        self.rw_timer = 0
        self.estado_evasion = {
            'modo': "NORMAL",
            'ultimo_modo': "",
            'angulo_giro': 90.0,
            'tiempo_arco': 0,
            'alternar_giro': 1,
        }

        self.cmd_linear_x = 0.0
        self.cmd_angular_z = 0.0

    def determinar_rol(self, log_fn=print):
        return determinar_rol(self, log_fn)

    def ground(self, opts: dict = None, log_fn=print):
        return self._sensors_mod.ground(self, opts, log_fn)

    def proximidad(self, modo_detallado: bool = False):
        return self._sensors_mod.proximidad(self, modo_detallado)

    def random_walking(self, timer: int):
        return self._move_mod.RandomWalking(self, timer)

    def move(self, distance_cm: float, target_angle_deg: float, speed_d_cm_s: float, speed_g_deg_s: float, timer: int, stop: bool = False):
        return self._move_mod.move(self, distance_cm, target_angle_deg, speed_d_cm_s, speed_g_deg_s, timer, stop)

    def r_move(self):
        return self._move_mod.r_move(self)

    def set_wheels_cm_s(self, left_cm_s: float, right_cm_s: float):
        return self._move_mod.set_wheels_cm_s(self, left_cm_s, right_cm_s, self.interwheel_distance)

    def r_Temporizador_1(self):
        return r_Temporizador_1(self)

    def Temporizador_1(self, c_seconds: float):
        return Temporizador_1(self, c_seconds, self.ticks_per_second)

    def r_Temporizador_2(self):
        return r_Temporizador_2(self)

    def Temporizador_2(self, d_seconds: float):
        return Temporizador_2(self, d_seconds, self.ticks_per_second)


class Utils:
    Temporizador_1        = staticmethod(Temporizador_1)
    r_Temporizador_1      = staticmethod(r_Temporizador_1)
    Temporizador_2        = staticmethod(Temporizador_2)
    r_Temporizador_2      = staticmethod(r_Temporizador_2)
    setleds               = staticmethod(setleds)
    determinar_rol        = staticmethod(determinar_rol)
    asignar_figura_por_id = staticmethod(asignar_figura_por_id)
    original_params       = staticmethod(original_params)
    print_table           = staticmethod(print_table)
    imprimir_posicion     = staticmethod(imprimir_posicion)