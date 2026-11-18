#!/usr/bin/env python3
# Archivo: Ks7/Controller/tracker_manager_node.py
# Ubicación: ~/sim_ws_qupa/Ks7/Controller/tracker_manager_node.py
# Descripción:
#   Nodo central receptor de telemetría en ROS 2 (Fase 2 de Ks7).
#   Recibe la lista de TRACKERS desde el JSON de configuración, los levanta como
#   subprocesos con stdin=subprocess.PIPE y les reenvía por stdin cada línea recibida
#   en /ks7/telemetry_init (RELIABLE + TRANSIENT_LOCAL) y /ks7/telemetry (RELIABLE),
#   manteniendo los 9 scripts tracker_*.py y evaluator_*.py 100% intactos.

import json
import os
import subprocess
import sys
import threading
import time

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_PROJECT_ROOT = os.path.dirname(_THIS_DIR) if os.path.basename(_THIS_DIR) == "Controller" else _THIS_DIR
for _p in (_THIS_DIR, _PROJECT_ROOT):
    if _p not in sys.path:
        sys.path.insert(0, _p)

from Functions import shared_config

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class TrackerManagerNode(Node):
    def __init__(self):
        super().__init__(
            'ks7_tracker_manager_node',
            parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)],
        )

        self.declare_parameter('config_file', '/tmp/ks7_robots_config.json')
        self.declare_parameter('auto_stop_on_end', True)
        self.declare_parameter('max_idle_timeout_sec', 240.0)
        self.declare_parameter('nivel_log', 2)

        self.config_file = self.get_parameter('config_file').get_parameter_value().string_value
        self.auto_stop_on_end = self.get_parameter('auto_stop_on_end').get_parameter_value().bool_value
        self.max_idle_timeout_sec = self.get_parameter('max_idle_timeout_sec').get_parameter_value().double_value
        self.nivel_log = self.get_parameter('nivel_log').get_parameter_value().integer_value

        self.print_lock = threading.Lock()
        self.stdin_lock = threading.Lock()
        self.tracker_procs = []
        self.tracker_activity = {}
        self.end_received = False
        self.shutdown_complete = False

        trackers_cfg = []
        if self.config_file and os.path.exists(self.config_file):
            try:
                with open(self.config_file, 'r', encoding='utf-8') as f:
                    data = json.load(f)
                trackers_cfg = data.get('trackers', []) or []
            except Exception as e:
                self._log(f"[TrackerManager] Error leyendo {self.config_file}: {e}", 0)

        self._spawn_trackers(trackers_cfg)

        qos_init = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        qos_stream = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=200,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.sub_init = self.create_subscription(
            String,
            shared_config.TELEMETRY_INIT_TOPIC,
            self._telemetry_init_cb,
            qos_init,
        )
        self.sub_stream = self.create_subscription(
            String,
            shared_config.TELEMETRY_TOPIC,
            self._telemetry_cb,
            qos_stream,
        )

        self.ready_pub = self.create_publisher(
            String,
            shared_config.TRACKER_READY_TOPIC,
            qos_init,
        )
        ready_msg = String()
        ready_msg.data = "READY"
        self.ready_pub.publish(ready_msg)
        self._log("[TRACKER_MANAGER_READY]", 1)

    def _log(self, msg: str, min_level: int = 1, end: str = "\n"):
        if self.nivel_log >= min_level:
            with self.print_lock:
                sys.stdout.write(msg + end)
                sys.stdout.flush()

    def _monitor_tracker_stdout(self, p, p_pid):
        buffer_line = ""
        while True:
            char = p.stdout.read(1)
            if not char:
                if buffer_line and self.nivel_log >= 2:
                    with self.print_lock:
                        sys.stdout.write(buffer_line)
                        sys.stdout.flush()
                break

            buffer_line += char
            if char == '\n' or char == '\r':
                if self.nivel_log >= 2:
                    with self.print_lock:
                        sys.stdout.write(buffer_line)
                        sys.stdout.flush()
                buffer_line = ""

            self.tracker_activity[p_pid] = time.time()

    def _spawn_trackers(self, trackers_cfg: list):
        run_env = os.environ.copy()
        run_env["PYTHONUNBUFFERED"] = "1"

        for t_conf in trackers_cfg:
            script_rel = t_conf.get("script", "")
            script_args = t_conf.get("args", []) or []
            script_path = (
                script_rel
                if os.path.isabs(script_rel)
                else os.path.join(_PROJECT_ROOT, script_rel)
            )
            if not os.path.exists(script_path):
                self._log(f"[Gestor] Aviso: No se encontró {script_rel}, omitiendo...", 1)
                continue

            cmd = [sys.executable, "-u", script_path] + [str(a) for a in script_args]
            try:
                self._log(f"[Gestor] Lanzando tracker: {script_rel}...", 1)
                p = subprocess.Popen(
                    cmd,
                    stdin=subprocess.PIPE,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1,
                    env=run_env,
                    cwd=_PROJECT_ROOT,
                )
                self.tracker_procs.append(p)
                self.tracker_activity[p.pid] = time.time()

                t = threading.Thread(
                    target=self._monitor_tracker_stdout,
                    args=(p, p.pid),
                    daemon=True,
                )
                t.start()
            except FileNotFoundError as e:
                self._log(f"[Gestor] Error al lanzar {script_rel}: {e}", 0)

    def _forward_line(self, raw_data: str):
        if raw_data is None:
            return
        for subline in str(raw_data).splitlines():
            line_with_nl = subline + "\n"
            with self.print_lock:
                sys.stdout.write(line_with_nl)
                sys.stdout.flush()

            with self.stdin_lock:
                for p in self.tracker_procs:
                    if p.poll() is None and p.stdin and not p.stdin.closed:
                        try:
                            p.stdin.write(line_with_nl)
                            p.stdin.flush()
                        except OSError:
                            pass

            if "End_simulation" in subline and not self.end_received:
                self.end_received = True
                if self.auto_stop_on_end:
                    threading.Thread(target=self._finalize_trackers, daemon=True).start()

    def _telemetry_init_cb(self, msg: String):
        self._forward_line(msg.data)

    def _telemetry_cb(self, msg: String):
        self._forward_line(msg.data)

    def _finalize_trackers(self):
        with self.stdin_lock:
            for p in self.tracker_procs:
                if p.stdin and not p.stdin.closed:
                    try:
                        p.stdin.close()
                    except Exception:
                        pass

        self._log(
            f"[+] Esperando a que los trackers finalicen sus tareas (Timeout: {int(self.max_idle_timeout_sec)}s)...",
            1,
        )
        while self.tracker_procs:
            for p in list(self.tracker_procs):
                if p.poll() is not None:
                    self.tracker_procs.remove(p)
                    continue

                last_active = self.tracker_activity.get(p.pid, time.time())
                if self.auto_stop_on_end and (time.time() - last_active) > self.max_idle_timeout_sec:
                    self._log(f"\n[!] ERROR: Tracker PID:{p.pid} congelado. Matando...", 0)
                    p.terminate()
                    try:
                        p.wait(timeout=5)
                    except Exception:
                        p.kill()
                    self.tracker_procs.remove(p)
            time.sleep(0.5)

        self._log("\n[+] Todos los procesos y trackers han finalizado.", 1)
        self.shutdown_complete = True

    def terminate_all_trackers(self):
        with self.stdin_lock:
            for p in self.tracker_procs:
                try:
                    if p.stdin and not p.stdin.closed:
                        p.stdin.close()
                except Exception:
                    pass
                if p.poll() is None:
                    try:
                        p.terminate()
                    except Exception:
                        pass


def main(args=None):
    rclpy.init(args=args)
    node = TrackerManagerNode()
    try:
        while rclpy.ok() and not node.shutdown_complete:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        node.terminate_all_trackers()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
