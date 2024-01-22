# pointcloud_publisher_launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pointcloud_publisher',  # Replace with your actual package name
            executable='pointcloud_publisher_node',
            name='pointcloud_publisher',
            output='screen',
            parameters=[
                {'file_path': '/root/Downloads/LOAM/cloudCorner.pcd'},  # Replace with your actual file path
                {'publish_topic': '/pcl/cloud'}  # Replace with your desired topic
            ]
        )
    ])
