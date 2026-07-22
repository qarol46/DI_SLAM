from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('rgbd_imu_slam'),
        'config',
        'imu_params.yaml'
    )
    
    return LaunchDescription([
        Node(
            package='rgbd_imu_slams',
            executable='imu_preintegration_node',
            name='imu_preintegration_node',
            parameters=[config],
            output='screen'
        )
    ])