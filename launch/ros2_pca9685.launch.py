import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()
    share_dir = get_package_share_directory('ros2_pca9685')

    param_file = LaunchConfiguration('param_file')

    params_arg = DeclareLaunchArgument('param_file',
                                        default_value=os.path.join(share_dir, 'config', 'params.yaml'),
                                        description='Path to the ROS2 parameter file')

    pca9685_node = Node(
        package='ros2_pca9685',
        executable='ros2_pca9685_node',
        name='pca9685_node',
        output="screen",
        emulate_tty=True,
        parameters=[param_file]
    )

    return LaunchDescription([
        params_arg,
        pca9685_node
    ])
