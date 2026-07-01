"""
  LokiGroundTruthPublisher.cs publishes to /ground_truth/odom
  LokiImuPublisher.cs publishes to /imu/data
  LokiDvlPublisher.cs publishes to /dvl/twist_stamped
  ActuatorBridge.cs subscribes to /cmd/thruster, /cmd/elevator, /cmd/rudder
  ros2 launch loki_bringup sim.launch.py
"""

from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description() -> LaunchDescription:

    config_dir = get_package_share_directory("loki_bringup")

    pid_params = os.path.join(config_dir, "config", "pid_params.yaml")
    ekf_config  = os.path.join(config_dir, "config", "ekf.yaml")

    ros_tcp_endpoint = Node(
        package="ros_tcp_endpoint",
        executable="default_server_endpoint",
        name="ros_tcp_endpoint",
        output="screen",
    )

    static_tf_imu = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_imu',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'imu_link']
                 #  x    y    z   yaw pitch roll   parent       child
    )
    
    static_tf_dvl = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_dvl',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'dvl_link']
                 #  x    y    z   yaw pitch roll   parent       child
    )

    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter",
        output="screen",
        parameters=[ekf_config],
    )

    controller = Node(
        package="loki_control",
        executable="loki_controller",
        name="loki_controller",
        output="screen",
        parameters=[pid_params],
    )

    monitor = Node(
        package="loki_monitor",
        executable="monitor",
        name="loki_monitor",
        output="screen",
    )

    foxglove = Node(
        package="foxglove_bridge",
        executable="foxglove_bridge",
        name="foxglove_bridge",
        output="screen",
    )

    return LaunchDescription([
        ros_tcp_endpoint,
        static_tf_imu,
        static_tf_dvl,
        TimerAction(period=2.0, actions=[ekf]),
        TimerAction(period=3.0, actions=[controller]),
        TimerAction(period=3.0, actions=[monitor]),
        TimerAction(period=3.0, actions=[foxglove]),
    ])