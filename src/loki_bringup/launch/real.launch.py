"""
  ros2 launch loki_bringup real.launch.py

  note : need to double check with hans
"""

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description() -> LaunchDescription:

    config_dir = get_package_share_directory("loki_bringup")

    pid_params  = os.path.join(config_dir, "config", "pid_params.yaml")
    ekf_params  = os.path.join(config_dir, "config", "ekf.yaml")
    vesc_params = os.path.join(config_dir, "config", "vesc_params.yaml")
    urdf_file   = os.path.join(config_dir, "urdf", "Full_Assembly_URDF_Combined.urdf")

    with open(urdf_file) as f:
        robot_description = f.read()

    state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description}],
        respawn=True,
        respawn_delay=2.0,
    )

    imu_node = Node(
        package="loki_icm",
        executable="ahrs_orientation",
        name="ahrs_orientation_node",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
    )

    madgwick = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[{'use_mag': False, 'use_sim_time': False}],
        remappings=[
            ('/imu/data_raw', '/imu/data_raw'),
            ('/imu/data',     '/imu/data'),
        ],
        respawn=True,
        respawn_delay=2.0,
    )

    dvl_receiver = Node(
        package="loki_cerulean",
        executable="tracker650_receiver",
        name="tracker650_receiver",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
    )

    dvl_republisher = Node(
        package="loki_cerulean",
        executable="tracker650_republisher",
        name="tracker650_republisher",
        output="log",
        respawn=True,
        respawn_delay=2.0,
    )

    hw_bridge = Node(
        package="loki_actuators",
        executable="hw_bridge",
        name="hw_bridge",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
    )

    vesc_driver_1 = Node(
        package="loki_vesc",
        executable="vesc_node",
        name="vesc_driver_1",
        namespace="vesc1",
        output="screen",
        parameters=[vesc_params],
        remappings=[
            ('vesc/commands/duty_cycle', '/vesc/commands/duty_cycle'),
            ('vesc/commands/current',    '/vesc/commands/current'),
            ('vesc/commands/rpm',        '/vesc/commands/rpm'),
        ],
        respawn=True,
        respawn_delay=2.0,
    )

    vesc_driver_2 = Node(
        package="loki_vesc",
        executable="vesc_node",
        name="vesc_driver_2",
        namespace="vesc2",
        output="screen",
        parameters=[vesc_params],
        remappings=[
            ('vesc/commands/duty_cycle', '/vesc/commands/duty_cycle'),
            ('vesc/commands/current',    '/vesc/commands/current'),
            ('vesc/commands/rpm',        '/vesc/commands/rpm'),
        ],
        respawn=True,
        respawn_delay=2.0,
    )

    ekf_node = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter",
        output="screen",
        parameters=[ekf_params, {'use_sim_time': False}],
        respawn=True,
        respawn_delay=2.0,
    )

    controller = Node(
        package="loki_control",
        executable="loki_controller",
        name="loki_controller",
        output="screen",
        parameters=[pid_params],
        respawn=True,
        respawn_delay=2.0,
    )

    monitor = Node(
        package="loki_monitor",
        executable="monitor",
        name="loki_monitor",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
    )

    foxglove = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
        respawn=True,
        respawn_delay=2.0,
    )

    return LaunchDescription([
        foxglove,
        state_publisher,
        imu_node,
        madgwick,
        dvl_receiver,
        dvl_republisher,
        hw_bridge,
        vesc_driver_1,
        vesc_driver_2,
        TimerAction(period=2.0, actions=[ekf_node]),
        TimerAction(period=4.0, actions=[controller]),
        TimerAction(period=6.0, actions=[monitor]),
    ])