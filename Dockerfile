# choose ROS distribudion based on build argument
FROM ros:galactic-ros-core

SHELL ["/bin/bash", "-c"]

WORKDIR /ros2_ws

COPY ./husarion_nav2 /ros2_ws/src/husarion_nav2

RUN apt update && apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-$ROS_DISTRO-slam-toolbox \
    ros-$ROS_DISTRO-navigation2 && \
    apt upgrade -y && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --symlink-install && \
    apt-get remove -y --purge \
    python3-pip \
    python3-colcon-common-extensions && \
    apt-get autoremove -y && apt clean && \
    rm -rf /var/lib/apt/lists/*

COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
