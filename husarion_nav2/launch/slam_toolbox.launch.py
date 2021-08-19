import os
from ament_index_python.packages import get_package_share_directory
import launch
import launch_ros.actions
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    rosbot_description = get_package_share_directory('rosbot_description')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    slam_toolbox_conf = LaunchConfiguration('slam_toolbox_conf',
                                default=rosbot_description+'/config/slam_toolbox.yaml')
 
    slam_toolbox_node = launch_ros.actions.Node(
        	parameters=[
                slam_toolbox_conf
        	],
            package='slam_toolbox',
            executable='sync_slam_toolbox_node',
            name='slam_toolbox',
            output='screen'
        )
    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time',
            default_value=use_sim_time
        ),
        slam_toolbox_node,
    ])


if __name__ == '__main__':
    generate_launch_description()
