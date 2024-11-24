import os.path
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="rviz2",
                executable="rviz2",
                name="rviz",
                arguments=[
                    "--display-config",
                    os.path.join(
                        get_package_share_directory("phone_sensors_examples"),
                        "config",
                        "all_sensors.rviz",
                    )
                ]
            )
        ]
    )
