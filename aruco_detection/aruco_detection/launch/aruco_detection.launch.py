import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Declare launch arguments
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz for visualization'
    )

    # Get configuration file paths
    aruco_params = os.path.join(
        get_package_share_directory('aruco_detection'),
        'config',
        'aruco_parameters.yaml'
    )

    rviz_config = os.path.join(
        get_package_share_directory('aruco_detection'),
        'config',
        'aruco_rviz.rviz'
    )

    # Static transform: map -> camera_link
    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_map_to_camera',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_link']
    )

    # Webcam publisher node
    webcam_publisher_node = Node(
        package='aruco_detection',
        executable='webcam_publisher'
    )

    # ArUco detection node
    aruco_node = Node(
        package='aruco_detection',
        executable='aruco_node',
        parameters=[aruco_params]
    )

    # RViz node (conditional)
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        condition=IfCondition(LaunchConfiguration('use_rviz'))
    )

    return LaunchDescription([
        use_rviz_arg,
        static_transform_node,
        webcam_publisher_node,
        aruco_node,
        rviz_node
    ])
