import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory("phone_sensors_examples"),
        "config",
        "phone_sensors_localization.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="phone_sensors",
                executable="server",
                name="phone_sensors_server",
                output="screen",
                parameters=[config_file],
                remappings=[
                    ("imu", "phone_sensors/imu"),
                    ("gnss", "phone_sensors/gps"),
                ],
            ),
            Node(
                package="robot_localization",
                executable="navsat_transform_node",
                name="navsat_transform_node",
                output="screen",
                parameters=[config_file],
                remappings=[
                    ("imu", "phone_sensors/imu"),
                    ("gps/fix", "phone_sensors/gps"),
                    ("odometry/gps", "phone_sensors/odometry"),
                    ("odometry/filtered", "odometry/global"),
                ],
            ),
            Node(
                package="robot_localization",
                executable="ukf_node",
                name="ukf_filter_node_map",
                output="screen",
                parameters=[config_file],
                remappings=[
                    ("imu/data", "phone_sensors/imu"),
                    ("odometry/gps", "phone_sensors/odometry"),
                    ("odometry/filtered", "odometry/global"),
                ],
            ),
            Node(
                package="robot_localization",
                executable="ukf_node",
                name="ukf_filter_node_odom",
                output="screen",
                parameters=[config_file],
                remappings=[
                    ("imu/data", "phone_sensors/imu"),
                    ("odometry/filtered", "odometry/local"),
                ],
            ),
        ]
    )
