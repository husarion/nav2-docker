# choose ROS distribudion based on build argument
FROM ros:galactic-ros-core

SHELL ["/bin/bash", "-c"]

WORKDIR /ros2_ws

COPY . .

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

RUN apt update && apt install -y \
        git \
        python3-pip \
        python3-colcon-common-extensions \
        ros-$ROS_DISTRO-rmw-fastrtps-cpp \
        ros-$ROS_DISTRO-slam-toolbox \
        ros-$ROS_DISTRO-navigation2 \
        ros-$ROS_DISTRO-nav2-bringup \
        ros-$ROS_DISTRO-nav2-rviz-plugins \
        ros-$ROS_DISTRO-rviz* && \
    git clone https://github.com/husarion/amcl_auto_localization.git src/amcl_auto_localization && \
    apt upgrade -y && \
    # apt-get remove -y ros-$ROS_DISTRO-rmw-cyclonedds-cpp && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --symlink-install && \
    # make the image smaller
    apt-get remove -y --purge \
        python3-pip \
        python3-colcon-common-extensions && \
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

ENTRYPOINT ["/ros2_ws/ros_entrypoint.sh"]
