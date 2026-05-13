import os.path
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def generate_launch_description():
    server_config_file = os.path.join(
        get_package_share_directory("phone_sensors_bridge_examples"),
        "config",
        "server_params_for_arduino.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="phone_sensors_bridge",
                executable="server",
                name="phone_sensors_bridge_server",
                output="screen",
                emulate_tty=True,
                parameters=[server_config_file],
            ),
            Node(
                package="phone_sensors_bridge_examples",
                executable="cmd_vel_to_arduino",
                name="cmd_vel_to_arduino",
                output="screen",
                emulate_tty=True,
                remappings=[("cmd_vel", "key_vel")],
            ),
        ]
    )
