# husarion_nav2

This package purpose is to wrap launch files for [navigation2](https://github.com/ros-planning/navigation2) and [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox).

## Launch files
### navigation2_bringup.<span>launch</span>.py
Launches [bringup_launch.py](https://github.com/ros-planning/navigation2/blob/main/nav2_bringup/launch/bringup_launch.py) and [amcl_auto_localization node](https://github.com/husarion/amcl_auto_localization).

Depending on the `use_slam` parameter, the `bringup_launch.py` file may start navigation2 stack with `slam` or with `amcl`.


#### params
- `use_sim_time` *(default: 'false')* - enables using simulation time.

- `use_slam` *(default: 'True')* - enables using navigation2 stack with `slam_toolbox`. If it's *false*, runs navigation2 stack with `amcl`  

- `nav2_config_file_slam` *(default: '/ros2_ws/src/husarion_nav2/config/nav2_slam_params.yaml.yaml')* - path to navigation2 stack nodes parameters and `slam_toolbox`. This path refers to [this config file](./config/nav2_slam_params.yaml). It is only needed when running slam.

- `nav2_config_file_amcl` *(default: '/ros2_ws/src/husarion_nav2/config/nav2_amcl_params.yaml.yaml')* - path to navigation2 stack nodes parameters (including `amcl`). This path refers to [this config file](./config/nav2_amcl_params.yaml). It is only needed when running amcl node.

- `map` *(default: '/ros2_ws/src/husarion_nav2/config/map.yaml')* - path to map config file for `map_server`. This path refers to [this config file](./config/map.yaml).

- `use_rviz` *(default: 'False')* - enables using rviz.

- `rviz_config_file_mapping` *(default: '/ros2_ws/src/husarion_nav2/config/rosbot_pro_localization.rviz')* - amcl compatible configuration files. It is only needed when running amcl node.

- `rviz_config_file_localization` *(default: '/ros2_ws/src/husarion_nav2/config/rosbot_pro_localization.rviz')* - slam compatible configuration files. It is only needed when running slam node.
