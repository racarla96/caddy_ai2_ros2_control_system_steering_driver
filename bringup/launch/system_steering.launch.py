import yaml
from launch import LaunchContext
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

default_controller_manager_update_rate = 100


def generate_launch_description():

    # Declare arguments
    declared_arguments = []

    # Get controller configuration - CORREGIDO: añadir "bringup/"
    system_steering_config = PathJoinSubstitution(
        [
            FindPackageShare("caddy_ai2_ros2_control_system_steering_driver"),
            "bringup",
            "config",
            "system_steering.yaml",
        ]
    )

    # LEER EL YAML DE CONFIGURACIÓN DEL CONTROLADOR
    context = LaunchContext()
    resolved_system_steering_config_path = system_steering_config.perform(context)
    
    # Leer el contenido del archivo YAML
    with open(resolved_system_steering_config_path, 'r') as yaml_file:
        try:
            yaml_content = yaml.safe_load(yaml_file)
            print("[DEBUG LAUNCH] YAML Content:", yaml_content) 
            # Obtener el valor de update_rate o usar 100 como predeterminado
            update_rate = yaml_content.get("controller_manager", {}).get("ros__parameters", {}).get("update_rate", default_controller_manager_update_rate)
        except yaml.YAMLError as exc:
            print(f"[ERROR LAUNCH] Error al cargar el YAML: {exc}")
            update_rate = default_controller_manager_update_rate

    # Get URDF via xacro - CORREGIDO: añadir "description/"
    system_steering_urdf_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [
                    FindPackageShare("caddy_ai2_ros2_control_system_steering_driver"), 
                    "description",
                    "urdf", 
                    "system_steering.urdf.xacro"
                ]
            ),
            " ",
            f"controller_manager_frequency_hz:={update_rate}",
        ]
    )

    # MOSTRAR EL CONTENIDO DEL COMANDO
    resolved_system_steering_urdf_content = system_steering_urdf_content.perform(context)
    print(f"[DEBUG LAUNCH] system_steering_urdf_content: {resolved_system_steering_urdf_content}")

    robot_description = {"robot_description": system_steering_urdf_content}

    # ROS2 Control node
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, system_steering_config],
        output="both",
    )

    # Robot state publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Joint state broadcaster spawner
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # Steering controller spawner
    system_steering_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["steering_controller"],
        output="screen",
    )

    nodes = [
        control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        system_steering_controller_spawner,
    ]

    return LaunchDescription(declared_arguments + nodes)