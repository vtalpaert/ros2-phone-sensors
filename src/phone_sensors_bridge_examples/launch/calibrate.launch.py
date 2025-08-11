import os.path
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory


def generate_launch_description():
    return LaunchDescription(
        [
            # ros2 run camera_calibration cameracalibrator --size 7x10 --square 0.02 --no-service-check --ros-args -r image:=/camera/image_raw
            Node(
                package="camera_calibration",
                executable="cameracalibrator",
                name="cameracalibrator",
                arguments=["--size", "7x10", "--square", "0.02", "--no-service-check"],
                remappings=[
                    ("image", "/camera/image_raw"),
                ],
            )
        ]
    )
