# nav2-docker
Dockerized [Nav2](https://navigation.ros.org/) nodes with custom Husarion launch in order to simplify running [navigation2](https://github.com/ros-planning/navigation2) and [slam_toolbox](https://github.com/SteveMacenski/slam_toolbox) nodes. Main purpose of this docker is to bring out of the box solution of Navigation2 docker for all ROS platforms. This docker supports all *AMD64* *ARM64* and *ARM32* architectures.

For more details about our own launch file read it's [README.md](./husarion_nav2/README.md)

## Examples
### Rosbot 2.0 Pro stadlone with Rviz2 VNC
1. Start all containers:
```bash
cd examples/rosbot_pro_nav2_docker_foxy
docker-compose up
```

2. In the web browser opn url:  http://<rosbot<span> ip>:6080/

3. In the virtual desktop open LXTerminal from desktop shortcut

4. In the `LXTerminal` type:

```bash
ros2 run rviz2 rviz2
```

### Rosbot 2.0 Pro + Navigation2 and Rviz2 VNC on desktop
This configuration gives you ability to make use of your desktop's computation power combined with Rosbot 2.0 Pro hardware.

> Note: Both devices have to be in the same network

1. Start all container on Rosbot 2.0 Pro:
```bash
# On Rosbot 2.0 Pro
export HOST_IP=<rosbot ip>
cd examples/rosbot_pro_nav2_docker_foxy_desktop
docker-compose -f docker-compose-rosbot.yaml up
```

2. Start all container on your desktop:
```bash
# On desktop
cd examples/rosbot_pro_nav2_docker_foxy_desktop
docker-compose -f docker-compose-desktop.yaml up
```
3. In the web browser opn url:  http://localhost:6080/

4. In the virtual desktop open LXTerminal from desktop shortcut

5. In the `LXTerminal` type:

```bash
ros2 run rviz2 rviz2
```

### Rosbot 2.0 stadlone
> Note 2: Rosbot 2.0 uses ARM32 processor which is not suported by ROS Foxy dockers and above. That is why in this case we base our docker image on  `ros:eloquent-ros-core`.


1. Start all containers:
```bash
cd examples/rosbot_nav2_docker_eloquent
docker-compose up
```

2. Now you can log in to docker and control robot via terminal or add own nodes. Unfortunately due to hardware limitations Rosbot 2.0 is not supported by [ros2-desktop-vnc](https://hub.docker.com/r/husarion/ros2-desktop-vnc).

### Rosbot 2.0 + Navigation2 and Rviz2 VNC on desktop
Just like with Rosbot 2.0 Pro you can take advantage of Rosbot 2.0 hardware with your desktop's computation power.

> Note: Both devices have to be in the same network

1. Start all container on Rosbot 2.0:
```bash
# On Rosbot 2.0
export HOST_IP=<rosbot ip>
cd examples/rosbot_nav2_docker_eloquent_desktop
docker-compose -f docker-compose-rosbot.yaml up
```

2. Start all container on your desktop:
```bash
# On desktop
cd examples/rosbot_nav2_docker_eloquent_desktop
docker-compose -f docker-compose-desktop.yaml up
```
3. In the web browser opn url: http://localhost:6080/

4. In the virtual desktop open LXTerminal from desktop shortcut

5. In the `LXTerminal` type:

```bash
ros2 run rviz2 rviz2
```

## Using custom Nav2 and Slam Toolbox parameters

In order to pass your custom parameters you can refer to one of [examples](./examples/custom_parameters).
It shows the simplest way done by replacing original files by binding folder containing files with the same names to docker. Binding to *src* path instead to *install* path is possible thanks to colcon's `--symlink-install` flag.

The other way of passing custom parameters is to bind them whereever you want inside docker. You can also use custom names if you like. After binding you can pass paths to those files as parameters as it is shown in husarion_nav2 package [README.md](./husarion_nav2/README.md).
