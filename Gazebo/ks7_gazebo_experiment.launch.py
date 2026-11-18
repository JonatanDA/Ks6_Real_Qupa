#!/usr/bin/env python3
# ============================================================================
# Archivo: Gazebo/ks7_gazebo_experiment.launch.py
# Ubicación exacta:
#   ~/sim_ws_qupa/Ks7/Gazebo/ks7_gazebo_experiment.launch.py
# ============================================================================

import glob
import importlib.util
import json
import os
import re
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


ARENA_3X3_SDF = """<?xml version="1.0" ?>
<sdf version="1.8">
  <model name="arena">
    <static>true</static>
    <link name="arena_link">
      <visual name="floor_visual">
        <geometry><plane><normal>0 0 1</normal><size>3.0 3.0</size></plane></geometry>
        <material><ambient>0.8 0.8 0.8 1</ambient><diffuse>0.8 0.8 0.8 1</diffuse></material>
      </visual>
      <collision name="floor_collision">
        <geometry><plane><normal>0 0 1</normal><size>3.0 3.0</size></plane></geometry>
      </collision>
      <visual name="wall_north_visual">
        <pose>0 1.5 0.25 0 0 0</pose>
        <geometry><box><size>3.0 0.1 0.5</size></box></geometry>
        <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material>
      </visual>
      <collision name="wall_north_collision">
        <pose>0 1.5 0.25 0 0 0</pose>
        <geometry><box><size>3.0 0.1 0.5</size></box></geometry>
      </collision>
      <visual name="wall_south_visual">
        <pose>0 -1.5 0.25 0 0 0</pose>
        <geometry><box><size>3.0 0.1 0.5</size></box></geometry>
        <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material>
      </visual>
      <collision name="wall_south_collision">
        <pose>0 -1.5 0.25 0 0 0</pose>
        <geometry><box><size>3.0 0.1 0.5</size></box></geometry>
      </collision>
      <visual name="wall_east_visual">
        <pose>1.5 0 0.25 0 0 0</pose>
        <geometry><box><size>0.1 3.0 0.5</size></box></geometry>
        <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material>
      </visual>
      <collision name="wall_east_collision">
        <pose>1.5 0 0.25 0 0 0</pose>
        <geometry><box><size>0.1 3.0 0.5</size></box></geometry>
      </collision>
      <visual name="wall_west_visual">
        <pose>-1.5 0 0.25 0 0 0</pose>
        <geometry><box><size>0.1 3.0 0.5</size></box></geometry>
        <material><ambient>0.2 0.2 0.2 1</ambient><diffuse>0.2 0.2 0.2 1</diffuse></material>
      </visual>
      <collision name="wall_west_collision">
        <pose>-1.5 0 0.25 0 0 0</pose>
        <geometry><box><size>0.1 3.0 0.5</size></box></geometry>
      </collision>
    </link>
  </model>
</sdf>
"""

FAST_WORLD_SDF = """<?xml version="1.0" ?>
<sdf version="1.8">
  <world name="qupa_world">
    <physics name="4ms" type="dart">
      <max_step_size>0.004</max_step_size>
      <real_time_factor>10.0</real_time_factor>
      <real_time_update_rate>2500</real_time_update_rate>
    </physics>

    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>

    <scene>
      <ambient>0.9 0.9 0.9 1</ambient>
      <background>0.8 0.8 0.8 1</background>
      <shadows>false</shadows>
    </scene>

    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 10 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.1 -0.9</direction>
    </light>

    <include>
      <uri>model://arena</uri>
      <name>arena</name>
      <pose>0 0 0 0 0 0</pose>
    </include>
  </world>
</sdf>
"""


def _prepare_clean_gui_config() -> str:
    candidates = (
        glob.glob("/usr/share/gz/gz-sim*/gui/gui.config")
        + ["/root/.gz/sim/8/gui.config", "/root/.gz/sim/gui.config"]
        + [os.path.expanduser("~/.gz/sim/8/gui.config"), os.path.expanduser("~/.gz/sim/gui.config")]
    )
    target_cam = "<camera_pose>0.0 -2.1 2.2 0.0 0.82 1.5707963</camera_pose>"
    out_cfg = "/tmp/ks7_clean_gui.config"

    for cpath in candidates:
        if os.path.exists(cpath):
            try:
                with open(cpath, "r", encoding="utf-8") as f:
                    content = f.read()
                if "<camera_pose>" in content:
                    content = re.sub(
                        r"<camera_pose>[^<]+</camera_pose>",
                        target_cam,
                        content,
                        count=1,
                    )
                with open(out_cfg, "w", encoding="utf-8") as f:
                    f.write(content)
                return out_cfg
            except OSError:
                pass
    return ""


def _load_robots_from_run_sim(config_file: str):
    """Prioriza el archivo JSON generado por run_sim_ros2.py y usa las rutas del workspace como respaldo."""
    if config_file and os.path.exists(config_file):
        try:
            with open(config_file, "r", encoding="utf-8") as f:
                cfg = json.load(f)
            robots = cfg.get("robots", [])
            enable_gui = cfg.get("enable_gazebo_gui", True)
            if robots:
                return robots, enable_gui
        except Exception:
            pass

    _ks7_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    candidate_paths = [
        os.path.join(_ks7_root, "scripts", "run_sim_ros2.py"),
        os.path.expanduser("~/sim_ws_qupa/Ks7/scripts/run_sim_ros2.py"),
        "/root/sim_ws/Ks7/scripts/run_sim_ros2.py",
        os.path.expanduser("~/sim_ws_qupa/Ks6/scripts/run_sim_ros2.py"),
        "/root/sim_ws/Ks6/scripts/run_sim_ros2.py",
    ]
    for run_sim_path in candidate_paths:
        if os.path.exists(run_sim_path):
            try:
                spec = importlib.util.spec_from_file_location("run_sim_ros2_cfg", run_sim_path)
                mod = importlib.util.module_from_spec(spec)
                spec.loader.exec_module(mod)
                robots = getattr(mod, "ROBOTS_CONFIG", None)
                enable_gui = getattr(mod, "ENABLE_VISUALIZATION", getattr(mod, "ENABLE_GAZEBO_GUI", True))
                if robots:
                    with open(config_file, "w", encoding="utf-8") as f:
                        json.dump({"enable_gazebo_gui": enable_gui, "robots": robots}, f, indent=2)
                    return robots, enable_gui
            except Exception:
                pass

    return [
        {"id": "Qupa0", "role": "master",     "x": -1.00, "y": -0.50, "yaw": 0.0, "figura": "Pentágono_irregular"},
        {"id": "Qupa4", "role": "apprentice", "x": -1.20, "y": -1.20, "yaw": 0.0, "figura": ""},
    ], True


def _find_xacro_file(pkg_qupa_desc: str) -> str:
    for candidate in ['qupa.urdf.xacro', 'qupa.xacro']:
        p = os.path.join(pkg_qupa_desc, 'urdf', candidate)
        if os.path.exists(p):
            return p
    raise FileNotFoundError(f"No se encontró qupa.urdf.xacro en {pkg_qupa_desc}/urdf")


def _prepare_fast_urdf(xacro_file: str, tf_prefix: str, role: str, pkg_qupa_desc: str) -> str:
    is_apprentice = (role.lower() == "apprentice" or tf_prefix == "Qupa4")

    doc = xacro.process_file(
        xacro_file,
        mappings={
            'use_sim_plugins': 'true',
            'tf_prefix': tf_prefix,
            'high_res': 'true' if is_apprentice else 'false',
            'module': 'false',
        }
    )
    xml_str = doc.toxml()

    abs_mesh_prefix = f"file://{pkg_qupa_desc}"
    xml_str = xml_str.replace("package://qupa_description", abs_mesh_prefix)
    xml_str = xml_str.replace("model://qupa_description", abs_mesh_prefix)

    xml_str = re.sub(
        r"<left_joint>L_wheel_joint</left_joint>\s*<right_joint>R_wheel_joint</right_joint>",
        "<left_joint>R_wheel_joint</left_joint><right_joint>L_wheel_joint</right_joint>",
        xml_str,
    )

    # Desbloquear límites de velocidad y aceleración del plugin DiffDrive
    xml_str = re.sub(
        r"<max_velocity>[^<]+</max_velocity>", 
        "<max_velocity>0.3</max_velocity>", xml_str)
    xml_str = re.sub(
        r"<max_linear_acceleration>[^<]+</max_linear_acceleration>",
        "<max_linear_acceleration>0.1</max_linear_acceleration>",
        xml_str,
    )
    xml_str = re.sub(
        r"<max_angular_velocity>[^<]+</max_angular_velocity>",
        "<max_angular_velocity>2.5</max_angular_velocity>",
        xml_str,
    )
    xml_str = re.sub(
        r"<max_angular_acceleration>[^<]+</max_angular_acceleration>",
        "<max_angular_acceleration>10.0</max_angular_acceleration>",
        xml_str,
    )

    # En wheel.xacro las articulaciones de las ruedas tienen <limit velocity="1.0" .../>
    # (1.0 rad/s * 0.016 m de radio = 1.6 cm/s máximo!) y damping="0.7".
    # Elevamos el límite de velocidad articular a 200 rad/s (~320 cm/s) y reducimos el damping
    # para que DART obedezca cualquier velocidad lineal/angular enviada por cmd_vel.
    xml_str = re.sub(
        r'<limit\b[^>]*velocity="1\.0"[^>]*/>',
        '<limit effort="10.0" lower="-10000.0" upper="10000.0" velocity="20.0"/>',
        xml_str,
    )
    xml_str = re.sub(
        r'<dynamics\b[^>]*damping="[^"]*"[^>]*friction="[^"]*"[^>]*/>',
        '<dynamics damping="0.7" friction="0.5"/>',
        xml_str,
    )
    xml_str = re.sub(
        r"<max_vel>0\.1</max_vel>",
        "<max_vel>5.0</max_vel>",
        xml_str,
    )

    xml_str = xml_str.replace("<mu>0.1</mu>", "<mu>0.1</mu>")
    xml_str = xml_str.replace("<mu2>0.1</mu2>", "<mu2>0.1</mu2>")

    # Filtrado selectivo de sensores en Ks7:
    # - Conservar siempre el sensor gpu_lidar (/scan) en todos los robots.
    # - Conservar wideanglecamera únicamente cuando is_apprentice == True (Qupa4).
    def _filter_sensor(match):
        sensor_block = match.group(0)
        if 'type="gpu_lidar"' in sensor_block or '_infrared' in sensor_block:
            sensor_block = re.sub(
                r"<visualize>[^<]*</visualize>",
                "<visualize>false</visualize>",
                sensor_block,
            )
            sensor_block = re.sub(
                r"<pose>0 0 0 1\.5708 0 0</pose>",
                "<pose>0 0 0 -1.5708 0 0</pose>",
                sensor_block,
            )
            return sensor_block
        if is_apprentice and ('type="wideanglecamera"' in sensor_block or '_camera' in sensor_block):
            sensor_block = re.sub(r"<width>\d+</width>", "<width>720</width>", sensor_block)
            sensor_block = re.sub(r"<height>\d+</height>", "<height>720</height>", sensor_block)
            sensor_block = re.sub(r"<update_rate>\d+</update_rate>", "<update_rate>10</update_rate>", sensor_block)
            return sensor_block
        return ""

    xml_str = re.sub(r"<sensor\b[^>]*>.*?</sensor>", _filter_sensor, xml_str, flags=re.DOTALL)

    if is_apprentice:
        led_rgba = "0.18 0.18 0.18 1.0"
        beacon_xyz = "0 0 0.05"
        beacon_radius = "0.015"
        beacon_length = "0.010"
    else:
        led_rgba = "0.0 0.35 1.0 1.0"
        beacon_xyz = "0 0 0.09"
        beacon_radius = "0.046"
        beacon_length = "0.090"

    extra_gazebo_tags = f"""
  <link name="{tf_prefix}/led_beacon_link">
    <visual name="led_beacon_visual">
      <origin xyz="{beacon_xyz}" rpy="0 0 0"/>
      <geometry><cylinder radius="{beacon_radius}" length="{beacon_length}"/></geometry>
      <material name="led_mat"><color rgba="{led_rgba}"/></material>
    </visual>
  </link>
  <joint name="{tf_prefix}_led_beacon_joint" type="fixed">
    <parent link="{tf_prefix}/base_link"/>
    <child link="{tf_prefix}/led_beacon_link"/>
    <origin xyz="0 0 0" rpy="0 0 0"/>
  </joint>
  <gazebo reference="{tf_prefix}/led_beacon_link">
    <visual>
      <material>
        <ambient>{led_rgba}</ambient>
        <diffuse>{led_rgba}</diffuse>
        <emissive>{led_rgba}</emissive>
      </material>
    </visual>
  </gazebo>
</robot>"""

    xml_str = xml_str.replace("</robot>", extra_gazebo_tags)
    return xml_str


def launch_setup(context, *args, **kwargs):
    pkg_qupa_sim = get_package_share_directory('qupa_simulation')
    pkg_qupa_desc = get_package_share_directory('qupa_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    use_sim_time_str = LaunchConfiguration('use_sim_time').perform(context)
    use_sim_time_bool = use_sim_time_str.lower() in ['true', '1']
    gui_arg = LaunchConfiguration('gui').perform(context).lower()
    config_file = LaunchConfiguration('config_file').perform(context)

    robots_list, cfg_gui = _load_robots_from_run_sim(config_file)
    enable_gui = cfg_gui if gui_arg == 'auto' else (gui_arg in ['true', '1'])

    for arena_path in [
        os.path.join(pkg_qupa_sim, 'models', 'arena', 'model.sdf'),
        os.path.expanduser('~/sim_ws_qupa/src/qupa_simulation/models/arena/model.sdf'),
        '/root/sim_ws/src/qupa_simulation/models/arena/model.sdf',
    ]:
        try:
            if os.path.exists(os.path.dirname(arena_path)):
                with open(arena_path, 'w', encoding='utf-8') as f:
                    f.write(ARENA_3X3_SDF)
        except OSError:
            pass

    fast_world_path = "/tmp/ks7_fast_world.sdf"
    with open(fast_world_path, 'w', encoding='utf-8') as wf:
        wf.write(FAST_WORLD_SDF)

    arena_model_dir = os.path.join(pkg_qupa_sim, 'models')
    qupa_desc_share_parent = os.path.dirname(pkg_qupa_desc)
    host_qupa_src = os.path.expanduser('~/sim_ws_qupa/src/qupa')
    extra_paths = f"{arena_model_dir}:{qupa_desc_share_parent}:{host_qupa_src}:/root/sim_ws/src/qupa"
    if 'GZ_SIM_RESOURCE_PATH' in os.environ:
        os.environ['GZ_SIM_RESOURCE_PATH'] += f":{extra_paths}"
    else:
        os.environ['GZ_SIM_RESOURCE_PATH'] = extra_paths

    gui_cfg_path = _prepare_clean_gui_config()
    if enable_gui:
        if gui_cfg_path:
            gz_flags = f"-r --gui-config {gui_cfg_path} {fast_world_path}"
        else:
            gz_flags = f"-r {fast_world_path}"
    else:
        gz_flags = f"-s -r {fast_world_path}"

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': gz_flags}.items()
    )

    xacro_file = _find_xacro_file(pkg_qupa_desc)
    camera_sim_cfg = os.path.join(pkg_qupa_sim, 'config', 'camera_sim.yaml')
    actions = [gz_sim]
    bridge_args = ['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']

    for r_cfg in robots_list:
        r_name = str(r_cfg['id'])
        rx = str(r_cfg.get('x', 0.0))
        ry = str(r_cfg.get('y', 0.0))
        ryaw_deg = float(r_cfg.get('yaw', 0.0))
        ryaw_rad = str(ryaw_deg * 3.141592653589793 / 180.0)
        role = str(r_cfg.get('role', 'master'))
        is_apprentice = (role.lower() == 'apprentice' or r_name == 'Qupa4')

        robot_xml = _prepare_fast_urdf(xacro_file, r_name, role, pkg_qupa_desc)

        actions.append(
            Node(
                package='robot_state_publisher',
                executable='robot_state_publisher',
                name='robot_state_publisher',
                namespace=r_name,
                output='screen',
                parameters=[{
                    'robot_description': robot_xml,
                    'use_sim_time': use_sim_time_bool,
                }],
            )
        )

        actions.append(
            Node(
                package='ros_gz_sim',
                executable='create',
                name=f'spawn_{r_name}',
                output='screen',
                arguments=[
                    '-string', robot_xml,
                    '-name', r_name,
                    '-x', rx,
                    '-y', ry,
                    '-z', '0.03',
                    '-Y', ryaw_rad,
                ],
            )
        )

        bridge_args.extend([
            f'/{r_name}/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            f'/{r_name}/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            f'/{r_name}/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        ])

        if is_apprentice:
            cam_params = []
            if os.path.exists(camera_sim_cfg):
                cam_params.append(camera_sim_cfg)
            cam_params.append({
                'use_sim_time': use_sim_time_bool,
                'publish_debug_image': True,
                'publish_hz': 8.0,
                'min_area': 25,
            })
            actions.append(
                Node(
                    package='qupa_simulation',
                    executable='camera_sim',
                    name='camera_node',
                    namespace=r_name,
                    output='screen',
                    parameters=cam_params,
                )
            )

    actions.append(
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            name='ks7_swarm_bridge',
            output='screen',
            arguments=bridge_args,
            parameters=[{'use_sim_time': use_sim_time_bool}],
        )
    )

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('gui', default_value='auto'),
        DeclareLaunchArgument('config_file', default_value='/tmp/ks7_robots_config.json'),
        OpaqueFunction(function=launch_setup)
    ])
