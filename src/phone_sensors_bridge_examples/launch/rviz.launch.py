import os.path
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python import get_package_share_directory


def generate_launch_description():
    server_config_file = os.path.join(
        get_package_share_directory("phone_sensors_bridge_examples"),
        "config",
        "phone_sensors_bridge_server.yaml",
    )
    camera_calibration_file = os.path.join(
        get_package_share_directory("phone_sensors_bridge_examples"),
        "config",
        "ost.yaml",
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
                    {
                        "camera_calibration_file": camera_calibration_file,
                    },
                ],
                remappings=[
                    ("imu", "phone_sensors_bridge/imu"),
                    ("gnss", "phone_sensors_bridge/gps"),
                ],
            ),
            ComposableNodeContainer(
                name='depth_image_proc_container',
                package='rclcpp_components',
                namespace="image_proc",
                executable='component_container',
                composable_node_descriptions=[
                    ComposableNode(
                        package='image_proc',
                        plugin='image_proc::RectifyNode',
                        name='rectify_node',
                        remappings=[
                            ('image', 'camera/image_raw'),
                            ('camera_info', 'camera/camera_info'),
                            ('image_rect', 'camera/image_rect'),
                        ],
                        parameters=[
                            {
                                "queue_size": 5,
                            },
                        ],
                    ),
                ]
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
