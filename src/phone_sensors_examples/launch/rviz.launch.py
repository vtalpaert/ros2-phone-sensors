import os.path
from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python import get_package_share_directory


def generate_launch_description():
    server_config_file = os.path.join(
        get_package_share_directory("phone_sensors_examples"),
        "config",
        "phone_sensors_server.yaml",
    )
    camera_calibration_file = os.path.join(
        get_package_share_directory("phone_sensors_examples"),
        "config",
        "ost.yaml",
    )

    return LaunchDescription(
        [
            Node(
                package="phone_sensors",
                executable="server",
                name="phone_sensors_server",
                output="screen",
                parameters=[
                    server_config_file,
                    {
                        "camera_calibration_file": camera_calibration_file,
                    },
                ],
                remappings=[
                    ("imu", "phone_sensors/imu"),
                    ("gnss", "phone_sensors/gps"),
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
                    ComposableNode(
                        package='depth_image_proc',
                        plugin='depth_image_proc::PointCloudXyzrgbNode',
                        name='point_cloud_xyzrgb_node',
                        remappings=[
                            ('rgb/image_rect_color', 'camera/image_rect'),
                            ('depth_registered/image_rect', 'camera/depth'),
                            ('points', 'camera/points'),
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
                        get_package_share_directory("phone_sensors_examples"),
                        "config",
                        "all_sensors.rviz",
                    ),
                ],
            ),
        ]
    )
