#!/usr/bin/env python3
# Archivo: Ks6/Ks6_master_controller.py
# Ubicación: ~/sim_ws_qupa/Ks6/Controller/Ks6_master_controller.py
# Descripción:
#   Controlador dedicado y autónomo de un robot Maestro (1 nodo = 1 robot, ej. Qupa0, Qupa1, qupa_D2)
#   en ROS 2 Jazzy (Gazebo Harmonic y Hardware Físico Real Qupa). Define las variables globales de los
#   maestros en un único lugar y las propaga a los submódulos (shared_config, Move, Utils, Sensors).
#   Equivalente 1:1 de Ks6/Controller/Ks6_master_controller.lua.

import json
import math
import os
import random
import sys
import time

# Configurar ruta local para encontrar módulos tanto en Ks6/Controller/ como en Ks6/Functions/
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.dirname(_THIS_DIR) if os.path.basename(_THIS_DIR) == "Controller" else _THIS_DIR
for _p in (_THIS_DIR, _PROJECT_ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

sys.modules["Controller.Ks6_master_controller"] = sys.modules[__name__]
sys.modules["Ks6_master_controller"] = sys.modules[__name__]
sys.modules["ks6_master_controller"] = sys.modules[__name__]

# ============================================================================
#                               VARIABLES GLOBALES
# ============================================================================
random.seed(time.time())

# Nota Hardware Real Qupa:
# - motor_driver_node tiene un watchdog de 0.3s (requiere ticks_per_second >= 4, recomendable 8-10 en físico)
#   y un deadband de 0.03 m/s por rueda con wheel_base = 0.086 m (para girar en su propio eje en el
#   robot físico se requiere velocidad angular >= 42 °/s, ej. speed_dem_g = 45, g_speed                             = 44.46).
ticks_per_second                    = 8
max_time                            = 5000 * ticks_per_second
Distribucion_sensores_de_proximidad = [0, -45, 45, -90, 90, 180]
Num_cantidad_leds                   = 2
FOOTBOT_INTERWHEEL_DISTANCE         = 8.6

factor_vel                          = 1.2
speed_dem                           = 10.0 * factor_vel
# speed_dem_g                         = (speed_dem / 2)
speed_dem_g                         = 40.0

d_speed                             = 8.0
g_speed                             = 44.46

# Límites físicos y configuración del IMU (MPU6050) — fuente única de verdad
MAX_LINEAR_SPEED_CM_S               = 16.0     # Velocidad lineal máxima: 16.0 cm/s (0.16 m/s)
MAX_ANGULAR_SPEED_RAD_S             = 0.776   # Velocidad angular máxima: 0.776 rad/s (~44.46 °/s)
IMU_TOPIC_SUFFIX                    = "mpu6050/imu/data_raw"
IMU_TARE_SERVICE_SUFFIX             = "mpu6050/tare"
IMU_GYRO_Z_SIGN                     = 1.0     # -1.0 si el eje Z del MPU6050 apunta hacia abajo
IMU_ANGULAR_SPEED_SLOW_RAD_S        = 0.72    # rad/s en zona lenta (> 0.70 rad/s deadband físico)
IMU_TURN_SLOW_ZONE_DEG              = 30.0    # ° finales a velocidad lenta
IMU_TURN_EARLY_STOP_DEG             = 15.0    # ° iniciales antes de la meta para frenar por inercia
IMU_ADAPTIVE_EARLY_STOP             = True    # Ajusta early_stop con el sobrepaso medido en cada giro
IMU_RETARE_EACH_SEGMENT             = True    # True: re-encera en cada vértice; False: sin tare ni esperas en move()
IMU_TURN_PAUSE_S                    = 0.5     # s de pausa de frenado (solo activo si IMU_RETARE_EACH_SEGMENT=True)
IMU_SETTLE_S                        = 0.3     # s tras el tare antes de girar (solo si IMU_RETARE_EACH_SEGMENT=True)
IMU_TARE_TIMEOUT_S                  = 5.0     # s máximo de espera al servicio /tare
IMU_TIMEOUT_FACTOR                  = 2.5     # Seguridad: aborta giro tras factor × tiempo esperado
IMU_MEASURED_LINEAR_SPEED_CM_S      = 0.0     # cm/s real medido con regla (0.0 → usa speed_dem / d_speed)

# Asignación de figura según ID del robot Maestro
distance_factor = 1

# Importación de módulos compartidos desde Functions/ (reciben automáticamente las variables de arriba)
from Functions import shared_config
from Functions import math_helper
from Functions import sensors as Sensors
from Functions import utils as Utils
from Functions import move as Move
import Functions.Observer.BehaviorSimilarity as BehaviorSimilarity

shared_config.sync_from_controller(sys.modules[__name__])

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger

try:
    from qupa_msgs.srv import LEDCommand
    HAS_QUPA_LED_SRV = True
except ImportError:
    LEDCommand = None
    HAS_QUPA_LED_SRV = False

try:
    from gz.transport13 import Node as GzNode
    from gz.msgs10.material_color_pb2 import MaterialColor
    HAS_GZ_TRANSPORT = True
except ImportError:
    HAS_GZ_TRANSPORT = False

_active_master_node = None
_active_master_rob = None


def log(msg):
    shared_config.log_py(msg)


# ============================================================================
#                                MAIN FUNCTIONS
# ============================================================================
# Funciones para los trackers (equivalentes 1:1 de Ks6_master_controller.lua)

def Print_tracker_trayectory_data(p1, p2=None, p3=None, p4=None, p5=None, p6=None, p7=None, p8=None):
    if _active_master_node is not None and not _active_master_node.tracker_printer.enable_print_trayectory:
        return
    rx = _active_master_rob.x if _active_master_rob is not None else 0.0
    ry = _active_master_rob.y if _active_master_rob is not None else 0.0

    if not isinstance(p1, str) or (p1 != "trajectory" and p1 != "Active_learning"):
        k, x, y, logging_flag = p1, p2, p3, p4
        if isinstance(k, (int, float)):
            shared_config.emit_telemetry(f"Master{int(k)} & {x} , {y}")
            if logging_flag:
                log(f"Master{int(k)} & {x} , {y}")
        elif isinstance(k, str):
            if k == "Qupa4":
                shared_config.emit_telemetry(f"Apprentice & {rx} , {ry}")
            elif k == "App":
                shared_config.emit_telemetry(f"Patrón{master_indice or ''} & {rx} , {ry}")
            else:
                shared_config.emit_telemetry(f"{k} & {rx} , {ry}")
        return

    if p1 == "trajectory":
        k, x, y = p2, p3, p4
        if isinstance(k, (int, float)):
            shared_config.emit_telemetry(f"[trajectory];Master{int(k)};{x};{y}")
        elif isinstance(k, str):
            if k == "Qupa4":
                shared_config.emit_telemetry(f"[trajectory];Apprentice;{rx};{ry}")
            elif k == "App":
                shared_config.emit_telemetry(f"[trajectory];Patrón{master_indice or ''};{rx};{ry}")
            else:
                shared_config.emit_telemetry(f"[trajectory];{k};{rx};{ry}")


def Print_tracker_calidad_imitacion_data(linea_resumen, is_master=1):
    if _active_master_node is not None and not _active_master_node.tracker_printer.enable_print_calidad:
        return
    if is_master == 0:
        shared_config.emit_telemetry(linea_resumen, is_init=False)
    else:
        rob = _active_master_rob
        if rob is not None and linea_resumen == rob.id:
            shared_config.emit_telemetry(
                Utils.formatear_patron_original_tracker(rob.id, rob.figura, rob.distance_factor),
                is_init=True,
            )
        else:
            shared_config.emit_telemetry(linea_resumen, is_init=True)


def evaluator_otsu(separacion, avance):
    if _active_master_node is not None and not _active_master_node.tracker_printer.enable_eval_otsu:
        return
    shared_config.emit_telemetry(f"Otsu & {separacion} , {avance}")


# ============================================================================
#                               MÓDULO DEL MAESTRO
# ============================================================================
all_params          = []
parametros_elegidos = []

d1                  = 2
master_avg_dist     = 0
master_indice       = 0
EAEP_c1             = 0
SMART_MASTERS       = True
indice              = 1
PI_end              = False
PI_counter_seg      = 0

shared_config.sync_from_controller(sys.modules[__name__])


def detectar_zona_demostracion(rob, timer_tick: int = 1, set_led_fn=None):
    """Equivalente 1:1 de detectar_zona_demostracion() en Ks6_master_controller.lua."""
    Move.RandomWalking(rob, timer_tick)
    if callable(set_led_fn):
        set_led_fn(rob, "black")
    Utils.Temporizador_1(rob, 4.0, ticks_per_second)
    if not rob.temp:
        rob.d1 = 1
        Move.set_wheels_cm_s(rob, 0.0, 0.0, FOOTBOT_INTERWHEEL_DISTANCE)
        Utils.r_Temporizador_1(rob)
        Move.r_move(rob)


def espera_antes_de_ejecutar_patron(rob):
    """Equivalente 1:1 de espera_antes_de_ejecutar_patron() en Ks6_master_controller.lua."""
    if rob.EAEP_c1 == 0:
        rob.master_avg_dist = 0.0
        Utils.Temporizador_1(rob, 15.0, ticks_per_second)
        if not rob.temp:
            rob.EAEP_c1 = 1
    else:
        rob.d1 = 2
        Move.r_move(rob)
        Utils.r_Temporizador_1(rob)


def Ejecutar_comportamiento(
    rob,
    params_P: list,
    timer_tick: int = 1,
    set_led_fn=None,
    tracker_printer=None,
    smart_masters: bool = True,
):
    """Equivalente 1:1 de Ejecutar_comportamiento(params_P) en Ks6_master_controller.lua."""
    params = params_P
    prox = Sensors.proximidad(rob, False)
    master_paused = False

    # LÓGICA DE PAUSA INTELIGENTE (usando exclusivamente el sensor /scan local)
    if smart_masters and rob.id != "Qupa4":
        if prox[0] >= 0.85 and abs(prox[1]) <= 45.0:
            master_paused = True

    if not rob.PI_end:
        if params and len(params) >= 1:
            if rob.indice <= len(params):
                valor1, valor2 = params[rob.indice - 1][0], params[rob.indice - 1][1]
                Move.move(
                    rob,
                    valor1,
                    valor2,
                    float(speed_dem),
                    float(speed_dem_g),
                    timer_tick,
                    master_paused,
                )

                if not master_paused:
                    if rob.id == "Qupa4" and tracker_printer is not None:
                        tracker_printer.Print_tracker_trayectory_data("trajectory", "App", 0, 0)
                    if callable(set_led_fn):
                        set_led_fn(rob, "yellow")

                    if rob.c1 == 2:
                        rob.PI_counter_seg += 1
                        if rob.indice == len(params):
                            rob.indice += 1
                            rob.PI_end = True
                            Utils.r_Temporizador_1(rob)
                        else:
                            rob.indice += 1
                            rob.c1 = 0
        else:
            rob.PI_end = True
            if callable(set_led_fn):
                set_led_fn(rob, "black")
            log(" -------------------------------------------------------------------------")
            log(f"Patrón no detectado :( {rob.id}")
            log(" -------------------------------------------------------------------------")

    if rob.PI_end:
        Move.set_wheels_cm_s(rob, 0.0, 0.0, FOOTBOT_INTERWHEEL_DISTANCE)
        if callable(set_led_fn):
            set_led_fn(rob, "black")


def resetDEM(rob):
    """Equivalente 1:1 de resetDEM() en Ks6_master_controller.lua."""
    rob.d1 = 0
    rob.EAEP_c1 = 0
    rob.posiciones_guardadas = {}
    rob.indice = 1
    rob.PI_end = False
    rob.PI_counter_seg = 0

    Utils.r_Temporizador_1(rob)
    Utils.r_Temporizador_2(rob)
    Move.r_move(rob)


def master(
    rob,
    timer_tick: int = 1,
    set_led_fn=None,
    tracker_printer=None,
    loop_masters: bool = True,
    smart_masters: bool = True,
):
    """Equivalente 1:1 de master() en Ks6_master_controller.lua."""
    if rob.d1 == 0:
        detectar_zona_demostracion(rob, timer_tick, set_led_fn)

    elif rob.d1 == 1:
        espera_antes_de_ejecutar_patron(rob)

    elif rob.d1 == 2:
        if len(all_params) > 0:
            if rob.master_indice == len(all_params):
                rob.d1 = 4
                return
            rob.master_indice += 1
            rob.parametros_elegidos = BehaviorSimilarity.translate_to_physical(
                all_params[rob.master_indice - 1]
            )
        else:
            if rob.id == "Qupa4":
                log(f"No se detectaron comportamientos en la etapa de observación, {rob.id}")
                rob.d1 = 4
                return
            orig = Utils.original_params(rob.figura, rob.distance_factor)
            rob.parametros_elegidos = orig.get('params', [])

        rob.d1 = 3

    elif rob.d1 == 3:
        Ejecutar_comportamiento(
            rob,
            rob.parametros_elegidos,
            timer_tick,
            set_led_fn=set_led_fn,
            tracker_printer=tracker_printer,
            smart_masters=smart_masters,
        )

        if rob.PI_end:
            resetDEM(rob)
            rob.d1 = 2 if loop_masters else 0

    elif rob.d1 == 4:
        Move.set_wheels_cm_s(rob, 0.0, 0.0, FOOTBOT_INTERWHEEL_DISTANCE)
        if callable(set_led_fn):
            set_led_fn(rob, "black")
        shared_config.emit_telemetry("End_simulation")


# ============================================================================
#                        CICLO DE VIDA ARGoS3 (init, step)
# ============================================================================
timer = 1


class Ks6MasterControllerNode(Node):
    def __init__(self):
        global _active_master_node, _active_master_rob

        cli_robot_id = ""
        cli_use_sim_time = False  # Robot real: reloj de pared (sin /clock de Gazebo)
        for idx, arg in enumerate(sys.argv):
            if arg.startswith("robot_id:="):
                cli_robot_id = arg.split(":=", 1)[1].strip()
            elif arg in ("-p", "--param") and idx + 1 < len(sys.argv) and sys.argv[idx + 1].startswith("robot_id:="):
                cli_robot_id = sys.argv[idx + 1].split(":=", 1)[1].strip()
            elif arg.startswith("use_sim_time:="):
                cli_use_sim_time = arg.split(":=", 1)[1].strip().lower() in ("true", "1", "yes")
            elif arg in ("-p", "--param") and idx + 1 < len(sys.argv) and sys.argv[idx + 1].startswith("use_sim_time:="):
                cli_use_sim_time = sys.argv[idx + 1].split(":=", 1)[1].strip().lower() in ("true", "1", "yes")

        safe_node_suffix = "".join(c if (c.isalnum() or c == "_") else "_" for c in cli_robot_id.lower())
        node_name = f"ks6_master_controller_{safe_node_suffix}" if safe_node_suffix else "ks6_master_controller_node"
        super().__init__(
            node_name,
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, cli_use_sim_time)],
        )
        _active_master_node = self
        self.use_sim_time_active = cli_use_sim_time

        self.declare_parameter('config_file', '/tmp/ks6_robots_config.json')
        self.declare_parameter('robot_id', cli_robot_id or '')
        self.declare_parameter('figura', '')
        self.declare_parameter('role', 'master')
        self.declare_parameter('init_x', 0.0)
        self.declare_parameter('init_y', 0.0)
        self.declare_parameter('init_yaw', 0.0)
        self.declare_parameter('loop_masters', True)

        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        param_robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        param_figura = self.get_parameter('figura').get_parameter_value().string_value
        param_role = self.get_parameter('role').get_parameter_value().string_value
        param_x = self.get_parameter('init_x').get_parameter_value().double_value
        param_y = self.get_parameter('init_y').get_parameter_value().double_value
        param_yaw = self.get_parameter('init_yaw').get_parameter_value().double_value
        self.loop_masters = self.get_parameter('loop_masters').get_parameter_value().bool_value

        injected_vars = {}
        active_funcs = None
        robot_entry = None

        if config_file and os.path.exists(config_file):
            try:
                with open(config_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                for r in data.get('robots', []):
                    r_id_str = str(r.get('id', ''))
                    r_role_str = str(r.get('role', 'master'))
                    if param_robot_id and r_id_str == param_robot_id:
                        robot_entry = r
                        break
                    elif not param_robot_id and r_role_str.lower() == 'master' and robot_entry is None:
                        robot_entry = r
                injected_vars = data.get('vars', {}) or {}
                if 'active_functions' in data:
                    active_funcs = set(data.get('active_functions', []))
            except Exception:
                robot_entry = None

        shared_config.apply_injected_vars(injected_vars, sys.modules[__name__])

        if robot_entry is not None:
            r_id = param_robot_id or str(robot_entry.get('id', 'Qupa0'))
            rx = float(robot_entry.get('x', param_x))
            ry = float(robot_entry.get('y', param_y))
            ryaw = float(robot_entry.get('yaw', param_yaw))
            fig = param_figura or str(robot_entry.get('figura', '')) or "Pentágono_irregular"
            role = param_role or str(robot_entry.get('role', 'master'))
        else:
            r_id = param_robot_id or "Qupa0"
            rx = param_x if param_x != 0.0 else -1.00
            ry = param_y if param_y != 0.0 else -0.50
            ryaw = param_yaw
            fig = param_figura or "Pentágono_irregular"
            role = param_role or "master"

        self.robot_id = r_id

        qos_init = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        qos_stream = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.telemetry_init_pub = self.create_publisher(
            String, shared_config.TELEMETRY_INIT_TOPIC, qos_init
        )
        self.telemetry_pub = self.create_publisher(
            String, shared_config.TELEMETRY_TOPIC, qos_stream
        )
        self.master_ready_pub = self.create_publisher(
            String, shared_config.MASTER_READY_TOPIC, qos_init
        )
        shared_config.set_telemetry_emitters(
            telemetry_fn=self._publish_telemetry,
            telemetry_init_fn=self._publish_telemetry_init,
        )

        self.gz_mat_pub = None
        if HAS_GZ_TRANSPORT and self.use_sim_time_active:
            try:
                self.gz_node = GzNode()
                self.gz_mat_pub = self.gz_node.advertise("/world/qupa_world/material_color", MaterialColor)
            except Exception:
                self.gz_mat_pub = None

        self.rob = Utils.QupaRobotInstance(
            self.robot_id,
            rx,
            ry,
            ryaw,
            fig,
            role,
            dist_sens=Distribucion_sensores_de_proximidad,
            d_speed=d_speed,
            g_speed=g_speed,
            ticks_per_second=ticks_per_second,
            interwheel_distance=FOOTBOT_INTERWHEEL_DISTANCE,
        )
        self.rob.distance_factor = float(distance_factor)
        _active_master_rob = self.rob
        self.robots = {self.robot_id: self.rob}

        # Tópicos y servicios de actuación/sensado (compatibles con Gazebo y qupa_hardware real)
        # --- CONFIGURACIÓN DE PUBLICADORES Y SUSCRIPCIONES DEL NODO MAESTRO ---
        # Publicador de velocidad diferencial (Twist saturado a 0.08 m/s y 0.776 rad/s)
        self.cmd_vel_pub = self.create_publisher(Twist, f'/{self.robot_id}/cmd_vel', 10)
        self.led_pub = self.create_publisher(String, f'/{self.robot_id}/leds/color', 10)
        self.led_srv_client = None
        if HAS_QUPA_LED_SRV:
            self.led_srv_client = self.create_client(LEDCommand, f'/{self.robot_id}/set')

        self.create_subscription(
            Odometry,
            f'/{self.robot_id}/odom',
            self._odom_cb,
            10,
        )
        # Suscripción estricta a proximidad por LaserScan (/{robot_id}/scan, sin ray-tracing)
        self.create_subscription(
            LaserScan,
            f'/{self.robot_id}/scan',
            self._scan_cb,
            10,
        )
        # Suscripción al IMU MPU6050 (/{robot_id}/mpu6050/imu/data_raw) y cliente de /tare
        self.create_subscription(
            Imu,
            f'/{self.robot_id}/{IMU_TOPIC_SUFFIX}',
            self._imu_cb,
            50,
        )
        self.imu_tare_client = self.create_client(
            Trigger,
            f'/{self.robot_id}/{IMU_TARE_SERVICE_SUFFIX}',
        )
        self.rob.imu_tare_client = self.imu_tare_client

        self.tracker_printer = Utils.TrackerPrinter(active_funcs, self.robots)
        self.timer_tick = 1
        self.initialized = False
        self.start_wall_time = time.time()

        self.step_timer = self.create_timer(
            1.0 / float(ticks_per_second), self.step
        )

    def _publish_telemetry_init(self, text: str):
        msg = String()
        msg.data = str(text)
        self.telemetry_init_pub.publish(msg)

    def _publish_telemetry(self, text: str):
        msg = String()
        msg.data = str(text)
        self.telemetry_pub.publish(msg)

    def _set_led(self, rob, color: str, force_topic_pub: bool = False):
        Utils.setleds(
            rob,
            color,
            led_pub=self.led_pub,
            gz_mat_pub=self.gz_mat_pub,
            force_topic_pub=force_topic_pub,
            led_srv_client=self.led_srv_client,
        )

    def stop_hardware(self):
        """Detiene motores y apaga la tira LED APA102 al cerrar el controlador."""
        try:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            self._set_led(self.rob, "black", force_topic_pub=True)
        except Exception:
            pass

    def init(self):
        self.rob.determinar_rol(shared_config.log_py)
        if self.tracker_printer.enable_print_calidad:
            self.tracker_printer.Print_tracker_calidad_imitacion_data(self.robot_id, self.rob.is_master)

        ready_msg = String()
        ready_msg.data = self.robot_id
        self.master_ready_pub.publish(ready_msg)
        print(f"[MASTER_READY] {self.robot_id}", flush=True)

    def step(self):
        global timer, _active_master_rob
        if not self.initialized:
            # En simulación (use_sim_time=True) espera hasta 8s por /odom; en robot físico arranca de inmediato en lazo abierto
            if self.use_sim_time_active and not self.rob.odom_received and (time.time() - self.start_wall_time) < 8.0:
                return
            self.initialized = True
            self.init()

        timer = self.timer_tick
        _active_master_rob = self.rob

        master(
            self.rob,
            self.timer_tick,
            set_led_fn=self._set_led,
            tracker_printer=self.tracker_printer,
            loop_masters=self.loop_masters,
            smart_masters=SMART_MASTERS,
        )

        self._set_led(self.rob, self.rob.led_color, force_topic_pub=True)

        twist = Twist()
        # Saturación estricta antes de publicar en /{robot_id}/cmd_vel (8 cm/s y 0.776 rad/s)
        lin_cmd, ang_cmd = Move.clamp_cmd_vel(self.rob.cmd_linear_x, self.rob.cmd_angular_z)
        twist.linear.x = float(lin_cmd)
        twist.angular.z = float(ang_cmd)
        self.cmd_vel_pub.publish(twist)

        self.timer_tick += 1
        timer = self.timer_tick

    def reset(self):
        pass

    def destroy(self):
        self.stop_hardware()

    def _scan_cb(self, msg: LaserScan):
        Sensors.process_laser_scan(self.rob, msg)

    def _imu_cb(self, msg: Imu):
        Move.update_imu(self.rob, msg, self.get_clock().now().nanoseconds * 1e-9)

    def _odom_cb(self, msg: Odometry):
        rob = self.rob
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        raw_yaw = math.atan2(siny_cosp, cosy_cosp)

        if not rob.odom_received:
            rob.odom_received = True
            if abs(pos.x) < 0.05 and abs(pos.y) < 0.05 and (abs(rob.init_x) > 0.05 or abs(rob.init_y) > 0.05):
                rob.odom_origin_pos = (rob.init_x, rob.init_y)
                rob.odom_origin_yaw = rob.init_yaw - raw_yaw
            else:
                rob.odom_origin_pos = (0.0, 0.0)
                rob.odom_origin_yaw = 0.0

        cos_o = math.cos(rob.odom_origin_yaw)
        sin_o = math.sin(rob.odom_origin_yaw)
        rob.x = rob.odom_origin_pos[0] + (pos.x * cos_o - pos.y * sin_o)
        rob.y = rob.odom_origin_pos[1] + (pos.x * sin_o + pos.y * cos_o)
        rob.yaw = Utils.norm_rad(raw_yaw + rob.odom_origin_yaw)


Ks6MasterControllerNode = Ks6MasterControllerNode


def init():
    if _active_master_node is not None:
        _active_master_node.init()


def step():
    if _active_master_node is not None:
        _active_master_node.step()


def reset():
    if _active_master_node is not None:
        _active_master_node.reset()


def destroy():
    if _active_master_node is not None:
        _active_master_node.destroy()


def main(args=None):
    rclpy.init(args=args)
    node = Ks6MasterControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_hardware()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()