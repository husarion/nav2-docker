import launch
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    rosbot_description = get_package_share_directory('rosbot_description')

    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    use_navigation = LaunchConfiguration('use_navigation', default='true')
    use_slam_toolbox = LaunchConfiguration('use_slam_toolbox', default='true')

    slam_toolbox_conf = LaunchConfiguration('slam_toolbox_conf',
                            default=rosbot_description+'/config/slam_toolbox.yaml')
    nav2_conf = LaunchConfiguration('nav2_conf',
                            default=rosbot_description+'/config/nav2_params.yaml')

    nav2_launch = None
    if use_navigation:
        nav2_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rosbot_description, '/launch/navigation.launch.py']),
            launch_arguments = {'nav2_conf' : nav2_conf,
                                'use_sim_time' : use_sim_time}.items()
        )

    slam_toolbox_launch = None
    if use_slam_toolbox:
        slam_toolbox_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([rosbot_description, '/launch/slam_toolbox.launch.py']),
            launch_arguments = {'slam_toolbox_conf' : slam_toolbox_conf,
                                'use_sim_time' : use_sim_time}.items()
        )
    
    description = [nav2_launch, slam_toolbox_launch]
    # Remove all None values
    description = list(filter(None, description))

    return LaunchDescription(description)


if __name__ == '__main__':
    generate_launch_description()