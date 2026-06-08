import os
import subprocess
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


_PKG = "caddy_ai2_ros2_control_system_steering_driver"


def _pkg_share() -> str:
    return get_package_share_directory(_PKG)


def _read_update_rate() -> int:
    config_path = os.path.join(_pkg_share(), "bringup", "config", "system_steering.yaml")
    with open(config_path) as f:
        cfg = yaml.safe_load(f)
    return int(cfg["steering"]["controller_manager"]["ros__parameters"]["update_rate"])


def generate_launch_description():

    pkg_share = _pkg_share()
    update_rate = _read_update_rate()

    system_steering_config = os.path.join(
        pkg_share, "bringup", "config", "system_steering.yaml"
    )

    xacro_path = os.path.join(
        pkg_share, "description", "urdf", "system_steering.urdf.xacro"
    )

    result = subprocess.run(
        ["xacro", xacro_path, f"update_rate:={update_rate}"],
        capture_output=True, text=True, check=True,
    )
    robot_description = {"robot_description": ParameterValue(result.stdout, value_type=str)}

    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        namespace="steering",
        parameters=[robot_description, system_steering_config],
        output="both",
    )

    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="steering",
        output="both",
        parameters=[robot_description],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="steering",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    system_steering_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        namespace="steering",
        arguments=["system_steering_controller"],
        output="screen",
    )

    return LaunchDescription([
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        system_steering_controller_spawner,
    ])
