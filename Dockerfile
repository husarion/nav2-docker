# choose ROS distribudion based on build argument
ARG ROS_IMAGE=foxy
FROM ros:$ROS_DISTRO-ros-core

SHELL ["/bin/bash", "-c"]

# install dependencies
RUN apt update && apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-$ROS_DISTRO-slam-toolbox \
    ros-$ROS_DISTRO-navigation2

RUN apt upgrade -y 

# create ros2_ws, copy and build package
RUN mkdir -p /ros2_ws/src
COPY ./husarion_nav2 /ros2_ws/src/husarion_nav2
WORKDIR /ros2_ws 
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && colcon build --symlink-install
    
# clear ubuntu packages
RUN apt clean && \
    rm -rf /var/lib/apt/lists/*

COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
