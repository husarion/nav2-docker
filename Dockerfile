ARG ROS_DISTRO=eloquent

FROM ros:$ROS_DISTRO-ros-base

SHELL ["/bin/bash", "-c"]

RUN apt update && apt install -y \
    python3-pip \
    ros-$ROS_DISTRO-slam-toolbox \
    ros-$ROS_DISTRO-navigation2

RUN apt upgrade -y 

RUN mkdir -p /ros2_ws/src \ 
    && cd /ros2_ws \
    && colcon build --symlink-install --merge-install

COPY ./husarion_nav2 /ros2_ws/src/husarion_nav2

RUN cd /ros2_ws \
    && rosdep update \
    && source /opt/ros/$ROS_DISTRO/setup.bash \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release --cmake-args -DBUILD_TESTING=OFF --merge-install

COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
