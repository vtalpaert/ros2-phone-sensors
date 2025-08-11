import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    server_config_file = os.path.join(
        get_package_share_directory("phone_sensors_bridge_examples"),
        "config",
        "phone_sensors_bridge_server.yaml",
    )

    localization_config_file = os.path.join(
        get_package_share_directory("phone_sensors_bridge_examples"),
        "config",
        "phone_sensors_bridge_localization.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="phone_sensors_bridge",
                executable="server",
                name="phone_sensors_bridge_server",
                output="screen",
                parameters=[server_config_file],
                remappings=[
                    ("imu", "phone_sensors_bridge/imu"),
                    ("gnss", "phone_sensors_bridge/gps"),
                ],
            ),
            Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform_node",
                output="screen",
                parameters=[localization_config_file],
                remappings=[
                    ("imu/data", "phone_sensors_bridge/imu"),
                    ("imu", "phone_sensors_bridge/imu"),
                    ("gps/fix", "phone_sensors_bridge/gps"),
                    ("odometry/gps", "phone_sensors_bridge/odometry"),
                    ("odometry/filtered", "odometry/global"),
                ],
            ),
            Node(
                package="robot_localization",
                executable="ukf_node",
                name="ukf_filter_node_map",
                output="screen",
                parameters=[localization_config_file],
                remappings=[
                    ("imu/data", "phone_sensors_bridge/imu"),
                    ("odometry/gps", "phone_sensors_bridge/odometry"),
                    ("odometry/filtered", "odometry/global"),
                ],
            ),
            Node(
                package="robot_localization",
                executable="ukf_node",
                name="ukf_filter_node_odom",
                output="screen",
                parameters=[localization_config_file],
                remappings=[
                    ("imu/data", "phone_sensors_bridge/imu"),
                    ("odometry/filtered", "odometry/local"),
                ],
            ),
        ]
    )
