# husarion_nav2

This package purpose is to wrap launch files for [navigation2](https://github.com/ros-planning/navigation2) and [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox).

## Launch files
### navigation2.<span>launch</span>.py
Launches only [navigation2](https://github.com/ros-planning/navigation2) node.
#### params
- `use_sim_time` *(default: 'false')* - enables using simulation time.
- `nav2_conf`*(default: '/ros2_ws/src/husarion_nav2/config/nav2_params.yaml')* - path to navigation2 parameters. This path refers to [this config file](./config/nav2_params.yaml).




### slam_toolbox.<span>launch</span>.py
Launches only [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) node.
#### params
- `use_sim_time` *(default: 'false')* - enables using simulation time.
- `slam_toolbox_conf`*(default: '/ros2_ws/src/husarion_nav2/config/slam_toolbox.yaml')* - path to slam_toolbox parameters. This path refers to [this config file](./config/slam_toolbox.yaml).


### bringup_both.<span>launch</span>.py
Launches both navigation2.<span>launch</span>.py and slam_toolbox.<span>launch</span>.py
#### params
- `use_sim_time` *(default: 'false')* - enables using simulation time.
- `nav2_conf`*(default: '/ros2_ws/src/husarion_nav2/config/nav2_params.yaml')* - path to navigation2 parameters. This path refers to [this config file](./config/nav2_params.yaml).
- `slam_toolbox_conf`*(default: '/ros2_ws/src/husarion_nav2/config/slam_toolbox.yaml')* - path to slam_toolbox parameters. This path refers to [this config file](./config/slam_toolbox.yaml).