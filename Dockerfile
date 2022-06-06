ARG ROS_DISTRO=humble

## ======================= STAGE 1 ==================================

FROM ros:$ROS_DISTRO AS amcl-builder

SHELL ["/bin/bash", "-c"]

WORKDIR /ros2_ws

COPY . .

RUN apt update && apt install -y \
        git \
        python3-pip \
        ros-$ROS_DISTRO-navigation2 \
        python3-colcon-common-extensions && \
    git clone https://github.com/husarion/amcl_auto_localization.git src/amcl_auto_localization && \
    apt upgrade -y && \
    source /opt/ros/$ROS_DISTRO/setup.bash && \
    colcon build --symlink-install && \
    rm -rf /var/lib/apt/lists/*

## ======================= STAGE 2 ==================================

FROM ros:$ROS_DISTRO-ros-core

SHELL ["/bin/bash", "-c"]

WORKDIR /ros2_ws

COPY . .

RUN apt update && apt install -y \
        ros-$ROS_DISTRO-navigation2 \
        ros-$ROS_DISTRO-nav2-bringup && \
    # make the image smaller
    apt-get autoremove -y && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/*

COPY --from=amcl-builder /ros2_ws /ros2_ws

ENTRYPOINT ["/ros2_ws/ros_entrypoint.sh"]
