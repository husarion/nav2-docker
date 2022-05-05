# Copyright (c) 2018 Intel Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""This is all-in-one launch script intended for use by nav2 developers."""

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Get the launch directory
    husarion_nav2_dir = get_package_share_directory('husarion_nav2')
    bringup_dir = get_package_share_directory('nav2_bringup')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_composition = LaunchConfiguration('use_composition')
    use_slam = LaunchConfiguration('use_slam')
    use_rviz = LaunchConfiguration('use_rviz')
    use_auto_localization = LaunchConfiguration('use_auto_localization')
    rviz_config_file_mapping = LaunchConfiguration('rviz_config_file_mapping')
    rviz_config_file_localization = LaunchConfiguration('rviz_config_file_localization')
    nav2_config_file_slam = LaunchConfiguration('nav2_config_file_slam')
    nav2_config_file_amcl = LaunchConfiguration('nav2_config_file_amcl')

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')

    declare_use_slam_cmd = DeclareLaunchArgument(
        'use_slam',
        default_value='True',
        description='Whether run a SLAM')

    declare_use_auto_localization_cmd = DeclareLaunchArgument(
        'use_auto_localization',
        default_value='False',
        description='Whether run a auto localization node')

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true')

    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(
            husarion_nav2_dir, 'config', 'map.yaml'),
        description='Full path to map file to load')

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack')

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='False',
        description='Whether to use composed bringup')

    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='False',
        description='Whether to start RVIZ')
    
    declare_rviz_config_file_mapping_cmd = DeclareLaunchArgument(
        'rviz_config_file_mapping',
        default_value=os.path.join(
            husarion_nav2_dir, 'config', 'rosbot_pro_mapping.rviz'),
        description='Full path to the RVIZ config file to use')

    declare_rviz_config_file_localization_cmd = DeclareLaunchArgument(
        'rviz_config_file_localization',
        default_value=os.path.join(
            husarion_nav2_dir, 'config', 'rosbot_pro_localization.rviz'),
        description='Full path to the RVIZ config file to use')

    declare_nav2_config_file_slam_cmd = DeclareLaunchArgument(
        'nav2_config_file_slam',
        default_value=os.path.join(husarion_nav2_dir, 'config', 'nav2_slam_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    declare_nav2_config_file_amcl_cmd = DeclareLaunchArgument(
        'nav2_config_file_amcl',
        default_value=os.path.join(husarion_nav2_dir, 'config', 'nav2_amcl_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes')

    # Specify the actions
    # Launch navigation2 stack with amcl or slam
    slam_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'bringup_launch.py')),
        condition=IfCondition(use_slam),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'slam': use_slam,
                          'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'params_file': nav2_config_file_slam,
                          'autostart': autostart,
                          'use_composition': use_composition}.items())
    
    amcl_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'bringup_launch.py')),
        condition=IfCondition(PythonExpression(['not ', use_slam])),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'slam': use_slam,
                          'map': map_yaml_file,
                          'use_sim_time': use_sim_time,
                          'params_file': nav2_config_file_amcl,
                          'autostart': autostart,
                          'use_composition': use_composition}.items())

    # Run rosbot initial auto localization node (it calls reinitialize_global_localization and
    # request_nomotion_update service)
    auto_localization_node_cmd = Node(
        package='amcl_auto_localization',
        executable='rosbot_auto_localizator',
        condition=IfCondition(PythonExpression([use_auto_localization, ' and not ', use_slam])),
        output='log',
    )

    # Launch rviz
    rviz_mapping_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rviz_launch.py')),
        condition=IfCondition(PythonExpression([use_rviz, ' and ', use_slam])),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'rviz_config': rviz_config_file_mapping}.items())
        

    rviz_localization_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'rviz_launch.py')),
        condition=IfCondition(PythonExpression([use_rviz, ' and not ', use_slam])),
        launch_arguments={'namespace': namespace,
                          'use_namespace': use_namespace,
                          'rviz_config': rviz_config_file_localization}.items())

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_slam_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_auto_localization_cmd)
    ld.add_action(declare_nav2_config_file_slam_cmd)
    ld.add_action(declare_nav2_config_file_amcl_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)

    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_rviz_config_file_mapping_cmd)
    ld.add_action(declare_rviz_config_file_localization_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(slam_bringup_cmd)
    ld.add_action(amcl_bringup_cmd)
    ld.add_action(auto_localization_node_cmd)
    ld.add_action(rviz_mapping_cmd)
    ld.add_action(rviz_localization_cmd)

    return ld