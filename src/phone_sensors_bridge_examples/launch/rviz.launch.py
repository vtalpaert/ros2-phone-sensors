import os.path
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python import get_package_share_directory


def generate_launch_description():
    server_config_file = os.path.join(
        get_package_share_directory("phone_sensors_bridge_examples"),
        "config",
        "server_params_for_phone_sensors.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="phone_sensors_bridge",
                executable="server",
                name="phone_sensors_bridge_server",
                output="screen",
                emulate_tty=True,
                parameters=[
                    server_config_file,
                ],
                remappings=[
                    ("imu", "phone_sensors_bridge/imu"),
                    ("gnss", "phone_sensors_bridge/gps"),
                ],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="gnss_tf",
                arguments=["0", "0", "0", "0", "0", "0", "base_link", "gnss"],
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                name="imu_tf",
                arguments=["0", "0", "0", "0", "0", "0", "base_link", "imu"],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz",
                arguments=[
                    "--display-config",
                    os.path.join(
                        get_package_share_directory("phone_sensors_bridge_examples"),
                        "config",
                        "all_sensors.rviz",
                    ),
                ],
            ),
        ]
    )
