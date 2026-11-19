#!/usr/bin/env python3
# Archivo: Ks7/Controller/Ks8_master_controller.py
# Ubicación: ~/sim_ws_qupa/Ks7/Controller/Ks8_master_controller.py
# Descripción:
#   Controlador interactivo de depuración (Ks8) para el robot físico Qupa.
#   - Pide por terminal pares "distancia,angulo" (ej: 10,45 -> gira 45° y avanza 10 cm).
#   - Tecla 'x' durante el movimiento: detiene el movimiento actual y pide un nuevo valor.
#   - Tecla 'n' (en prompt o en movimiento): reinicia la odometría estimada a (0, 0).
#   - Enciende LED azul una sola vez al iniciar movimiento y apaga al terminar.

import json
import math
import os
import queue
import random
import select
import sys
import termios
import threading
import time
import tty

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.dirname(_THIS_DIR) if os.path.basename(_THIS_DIR) == "Controller" else _THIS_DIR
for _p in (_THIS_DIR, _PROJECT_ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

sys.modules["Controller.Ks8_master_controller"] = sys.modules[__name__]
sys.modules["Ks8_master_controller"] = sys.modules[__name__]

# ============================================================================
#                               VARIABLES GLOBALES
# ============================================================================
random.seed(time.time())

ticks_per_second                    = 8
max_time                            = 5000 * ticks_per_second
Distribucion_sensores_de_proximidad = [0, -45, 45, -90, 90, 180]
Num_cantidad_leds                   = 2
FOOTBOT_INTERWHEEL_DISTANCE         = 8.6

factor_vel                          = 1
speed_dem                           = 5 * factor_vel   # cm/s
speed_dem_g                         = 50               # °/s
d_speed                             = 6
g_speed                             = 60
distance_factor                     = 1

from Functions import shared_config
from Functions import sensors as Sensors
from Functions import utils as Utils
from Functions import move as Move

shared_config.sync_from_controller(sys.modules[__name__])

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String

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


class Ks8MasterControllerNode(Node):
    def __init__(self):
        cli_robot_id = ""
        cli_use_sim_time = False
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
        node_name = f"ks8_master_controller_{safe_node_suffix}" if safe_node_suffix else "ks8_master_controller_node"
        super().__init__(
            node_name,
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, cli_use_sim_time)],
        )
        self.use_sim_time_active = cli_use_sim_time

        self.declare_parameter('config_file', '/tmp/ks7_robots_config.json')
        self.declare_parameter('robot_id', cli_robot_id or 'qupa_D2')
        self.declare_parameter('figura', 'Manual_Ks8')
        self.declare_parameter('role', 'master')
        self.declare_parameter('init_x', 0.0)
        self.declare_parameter('init_y', 0.0)
        self.declare_parameter('init_yaw', 0.0)

        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        param_robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        param_figura = self.get_parameter('figura').get_parameter_value().string_value
        param_role = self.get_parameter('role').get_parameter_value().string_value
        rx = self.get_parameter('init_x').get_parameter_value().double_value
        ry = self.get_parameter('init_y').get_parameter_value().double_value
        ryaw = math.radians(self.get_parameter('init_yaw').get_parameter_value().double_value)

        if config_file and os.path.exists(config_file):
            try:
                with open(config_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                shared_config.apply_injected_vars(data.get('vars', {}) or {}, sys.modules[__name__])
            except Exception:
                pass

        self.robot_id = param_robot_id or "qupa_D2"

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
        self.telemetry_init_pub = self.create_publisher(String, shared_config.TELEMETRY_INIT_TOPIC, qos_init)
        self.telemetry_pub = self.create_publisher(String, shared_config.TELEMETRY_TOPIC, qos_stream)
        self.master_ready_pub = self.create_publisher(String, shared_config.MASTER_READY_TOPIC, qos_init)

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
            param_figura,
            param_role,
            dist_sens=Distribucion_sensores_de_proximidad,
            d_speed=d_speed,
            g_speed=g_speed,
            ticks_per_second=ticks_per_second,
            interwheel_distance=FOOTBOT_INTERWHEEL_DISTANCE,
        )
        self.rob.distance_factor = float(distance_factor)

        self.cmd_vel_pub = self.create_publisher(Twist, f'/{self.robot_id}/cmd_vel', 10)
        self.led_pub = self.create_publisher(String, f'/{self.robot_id}/leds/color', 10)
        self.led_srv_client = self.create_client(LEDCommand, f'/{self.robot_id}/set') if HAS_QUPA_LED_SRV else None

        self.create_subscription(Odometry, f'/{self.robot_id}/odom', self._odom_cb, 10)
        self.create_subscription(LaserScan, f'/{self.robot_id}/scan', self._scan_cb, 10)

        # Estado interactivo Ks8
        self.cmd_queue = queue.Queue()
        self.move_done_event = threading.Event()
        self.cancel_move_event = threading.Event()
        self.stop_event = threading.Event()
        self.odom_lock = threading.Lock()
        self.current_cmd = None  # Tupla (distancia_cm, angulo_deg)
        self.move_tick_counter = 0

        # Referencia de odometría cruda para que el reset ('n') funcione también con /odom activo
        self.last_raw_odom = (0.0, 0.0, 0.0)
        self.odom_ref = (0.0, 0.0, 0.0)

        self.timer_tick = 1
        self.initialized = False
        self.step_timer = self.create_timer(1.0 / float(ticks_per_second), self.step)

        self.input_thread = threading.Thread(target=self._input_loop, daemon=True)

    def _publish_telemetry_init(self, text: str):
        msg = String()
        msg.data = str(text)
        self.telemetry_init_pub.publish(msg)

    def _publish_telemetry(self, text: str):
        msg = String()
        msg.data = str(text)
        self.telemetry_pub.publish(msg)

    def _set_led(self, color: str, force_topic_pub: bool = False):
        Utils.setleds(
            self.rob,
            color,
            led_pub=self.led_pub,
            gz_mat_pub=self.gz_mat_pub,
            force_topic_pub=force_topic_pub,
            led_srv_client=self.led_srv_client,
        )

    def _emit_position(self):
        """Envía coordenadas al tracker_trayectory.py."""
        self.move_tick_counter += 1
        with self.odom_lock:
            rx, ry = self.rob.x, self.rob.y
        shared_config.emit_telemetry(f"Timer={self.move_tick_counter}")
        shared_config.emit_telemetry(
            f"[trajectory];{self.robot_id};{rx:.4f};{ry:.4f}"
        )

    def _reset_odometry(self):
        """Reinicia la odometría estimada a (0, 0, 0°)."""
        with self.odom_lock:
            self.odom_ref = self.last_raw_odom
            self.rob.x = 0.0
            self.rob.y = 0.0
            self.rob.yaw = 0.0
        self._emit_position()
        print("\n[Ks8] Odometría reiniciada a x=0.000 m, y=0.000 m, yaw=0.0°", flush=True)

    def _wait_move_with_hotkeys(self):
        """Espera a que termine el movimiento escuchando 'x' (cancelar) y 'n' (reset odom)."""
        if not sys.stdin.isatty():
            while not self.move_done_event.is_set() and not self.stop_event.is_set():
                self.move_done_event.wait(timeout=0.2)
            return

        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while not self.move_done_event.is_set() and not self.stop_event.is_set():
                ready, _, _ = select.select([sys.stdin], [], [], 0.1)
                if ready:
                    ch = sys.stdin.read(1).lower()
                    if ch == 'x':
                        self.cancel_move_event.set()
                        print("\n[Ks8] Movimiento detenido por usuario ('x').", flush=True)
                        self.move_done_event.wait(timeout=1.0)
                        break
                    elif ch == 'n':
                        self._reset_odometry()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def _input_loop(self):
        """Hilo dedicado a leer 'distancia,angulo' desde la terminal."""
        print("\n===========================================================", flush=True)
        print(f" [Ks8] {self.robot_id} LISTO. Formato: distancia(cm),angulo(°) (ej: 10,45)", flush=True)
        print("       Tecla 'x' en movimiento = detener | 'n' = reset odom (0,0)", flush=True)
        print("       (Ctrl+C para salir)", flush=True)
        print("===========================================================\n", flush=True)

        while not self.stop_event.is_set():
            try:
                raw = input(f"[{self.robot_id}] distancia(cm),angulo(°) [n=reset] > ").strip()
            except (EOFError, KeyboardInterrupt):
                break

            if not raw:
                continue

            if raw.lower() == 'n':
                self._reset_odometry()
                continue

            parts = raw.replace(";", ",").split(",")
            if len(parts) != 2:
                print("[Ks8] Formato inválido. Usa: distancia,angulo (ej: 10,45) o 'n'", flush=True)
                continue

            try:
                distancia = float(parts[0].strip())
                angulo = float(parts[1].strip())
            except ValueError:
                print("[Ks8] Valores numéricos inválidos. Ej: 10,45 o 15,-90", flush=True)
                continue

            self.move_done_event.clear()
            self.cancel_move_event.clear()
            self.cmd_queue.put((distancia, angulo))
            print(
                f"[Ks8] Ejecutando: distancia={distancia} cm, angulo={angulo}°... "
                "(pulsa 'x' para detener, 'n' para reset odom)",
                flush=True,
            )

            self._wait_move_with_hotkeys()

            if not self.stop_event.is_set():
                with self.odom_lock:
                    rx, ry, ryaw = self.rob.x, self.rob.y, math.degrees(self.rob.yaw)
                print(
                    f"[Ks8] Listo. Posición estimada: x={rx:.3f} m, y={ry:.3f} m, yaw={ryaw:.1f}°\n",
                    flush=True,
                )

    def init(self):
        ready_msg = String()
        ready_msg.data = self.robot_id
        self.master_ready_pub.publish(ready_msg)
        print(f"[MASTER_READY] {self.robot_id}", flush=True)
        self._set_led("black", force_topic_pub=True)
        self._emit_position()
        self.input_thread.start()

    def _finish_current_move(self):
        Move.set_wheels_cm_s(self.rob, 0.0, 0.0, FOOTBOT_INTERWHEEL_DISTANCE)
        Move.r_move(self.rob)
        self.rob.cmd_linear_x = 0.0
        self.rob.cmd_angular_z = 0.0
        self._set_led("black", force_topic_pub=True)
        self.current_cmd = None
        self.cancel_move_event.clear()
        self.move_done_event.set()

    def step(self):
        if not self.initialized:
            self.initialized = True
            self.init()

        if self.current_cmd is None:
            try:
                self.current_cmd = self.cmd_queue.get_nowait()
                Move.r_move(self.rob)
                self.rob.c1 = 0
                # Encender LED una sola vez al comenzar el movimiento (evita saturar el servicio a 8 Hz)
                self._set_led("blue", force_topic_pub=True)
            except queue.Empty:
                pass

        if self.current_cmd is not None:
            if self.cancel_move_event.is_set():
                self._finish_current_move()
            else:
                distancia, angulo = self.current_cmd
                # Move.move ya integra (x, y, yaw) en lazo abierto internamente
                with self.odom_lock:
                    Move.move(
                        self.rob,
                        distancia,
                        angulo,
                        float(speed_dem),
                        float(speed_dem_g),
                        self.timer_tick,
                        False,
                    )
                self._emit_position()

                if self.rob.c1 == 2:
                    self._finish_current_move()
        else:
            self.rob.cmd_linear_x = 0.0
            self.rob.cmd_angular_z = 0.0

        twist = Twist()
        twist.linear.x = float(self.rob.cmd_linear_x)
        twist.angular.z = float(self.rob.cmd_angular_z)
        self.cmd_vel_pub.publish(twist)

        self.timer_tick += 1

    def stop_hardware(self):
        self.stop_event.set()
        self.cancel_move_event.set()
        self.move_done_event.set()
        try:
            twist = Twist()
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.cmd_vel_pub.publish(twist)
            self._set_led("black", force_topic_pub=True)
        except Exception:
            pass

    def _scan_cb(self, msg: LaserScan):
        Sensors.process_laser_scan(self.rob, msg)

    def _odom_cb(self, msg: Odometry):
        rob = self.rob
        pos = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        raw_yaw = math.atan2(siny_cosp, cosy_cosp)

        with self.odom_lock:
            self.last_raw_odom = (pos.x, pos.y, raw_yaw)
            if not rob.odom_received:
                rob.odom_received = True
                self.odom_ref = (pos.x, pos.y, raw_yaw)

            dx = pos.x - self.odom_ref[0]
            dy = pos.y - self.odom_ref[1]
            cos_r = math.cos(-self.odom_ref[2])
            sin_r = math.sin(-self.odom_ref[2])
            rob.x = dx * cos_r - dy * sin_r
            rob.y = dx * sin_r + dy * cos_r
            rob.yaw = Utils.norm_rad(raw_yaw - self.odom_ref[2])


def main(args=None):
    rclpy.init(args=args)
    node = Ks8MasterControllerNode()
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