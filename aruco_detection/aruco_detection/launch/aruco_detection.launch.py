import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    aruco_params = os.path.join(
        get_package_share_directory('aruco_detection'),
        'config',
        'aruco_parameters.yaml'
        )

    aruco_node = Node(
        package='aruco_detection',
        executable='aruco_node',
        parameters=[aruco_params]
    )

    webcam_publisher_node = Node(
        package='aruco_detection',
        executable='webcam_publisher'
    )

    return LaunchDescription([
        webcam_publisher_node,
        aruco_node
    ])
