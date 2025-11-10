# ArduPilot ROS 2 packages

This directory contains ROS 2 packages and configuration files for running
ROS 2 processes and nodes that communicate with the ArduPilot DDS client
library using the microROS agent. It contains the following packages:
 
#### `ardupilot_sitl`

This is a `colcon` package for building and running ArduPilot SITL using the ROS 2 CLI.
For example `ardurover` SITL may be launched with:

```bash
ros2 launch ardupilot_sitl sitl.launch.py command:=ardurover model:=rover
```

Other launch files are included with many arguments.
Some common arguments are exposed and forwarded to the underlying process.

For example, MAVProxy can be launched, and you can enable the `console` and `map`.
```bash
ros2 launch ardupilot_sitl sitl_mavproxy.launch.py map:=True console:=True 
```

ArduPilot SITL does not yet expose all arguments from the underlying binary.
See [#27714](https://github.com/ArduPilot/ardupilot/issues/27714) for context.

To see all current options, use the `-s` argument:
```bash
ros2 launch ardupilot_sitl sitl.launch.py -s
```

#### `ardupilot_dds_tests`

A `colcon` package for testing communication between `micro_ros_agent` and the
ArduPilot `AP_DDS` client library.

## Prerequisites

The packages depend on:

- [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)


## Install Ubuntu

#### 1. Create a workspace folder

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
```

The ROS 2 tutorials contain more details regarding [ROS 2 workspaces](https://docs.ros.org/en/humble/Tutorials/Workspace/Creating-A-Workspace.html).

#### 2. Get the `ros2.repos` file

```bash
cd ~/ros2_ws/src
wget https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos
vcs import --recursive < ros2.repos
```

#### 3. Update dependencies

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
sudo apt update
rosdep update
rosdep install --rosdistro ${ROS_DISTRO} --from-paths src
```

#### 4. Build

Check that the [ROS environment](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Configuring-ROS2-Environment.html#check-environment-variables) is configured correctly:

```bash
ROS_VERSION=2
ROS_PYTHON_VERSION=3
ROS_DISTRO=humble
```

```bash
cd ~/ros2_ws
colcon build --cmake-args -DBUILD_TESTING=ON
```

#### 5. Test

```bash
source ./install/setup.bash
colcon test --packages-select ardupilot_dds_tests
colcon test-result --all --verbose
```

To debug a specific test, you can do the following:
```
colcon --log-level DEBUG test --packages-select ardupilot_dds_tests --event-handlers=console_direct+ --pytest-args -k test_dds_udp_geopose_msg_recv -s
```

## Install macOS

The install procedure on macOS is similar, except that all dependencies
must be built from source and additional compiler flags are needed.

#### 1. Create a workspace folder

```bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws/src
```

#### 2. Get the `ros2_macos.repos` file

The `ros2_macos.repos` includes additional dependencies to build:

```bash
cd ~/ros2_ws/src
wget https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2_macos.repos
vcs import --recursive < ros2_macos.repos
```

#### 3. Update dependencies

```bash
cd ~/ros2_ws
source /{path_to_your_ros_distro_workspace}/install/setup.zsh
```

#### 4.1. Build microxrcedds_gen:

```bash
cd ~/ros2_ws/src/microxrcedds_gen
./gradlew assemble
export PATH=$PATH:$(pwd)/scripts
```

#### 4.2. Build colcon projects

```bash
colcon build --symlink-install --cmake-args \
-DBUILD_TESTING=ON \
-DCMAKE_BUILD_TYPE=RelWithDebInfo \
-DCMAKE_MACOSX_RPATH=FALSE \
-DUAGENT_SOCKETCAN_PROFILE=OFF \
-DUAGENT_LOGGER_PROFILE=OFF \
-DUAGENT_USE_SYSTEM_LOGGER=OFF \
-DUAGENT_USE_SYSTEM_FASTDDS=ON \
-DUAGENT_USE_SYSTEM_FASTCDR=ON \
--event-handlers=desktop_notification-
```

#### 5. Test

```bash
colcon test \
--pytest-args -s -v \
--event-handlers console_cohesion+ desktop_notification- \
--packages-select ardupilot_dds_tests
```

## Install Docker

#### 0. Build the image and run the container

Clone the ArduPilot docker project:

```bash
git clone https://github.com/ArduPilot/ardupilot_dev_docker.git
```

Build the container:

```bash
cd ~/ardupilot_dev_docker/docker
docker build -t ardupilot/ardupilot-dev-ros -f Dockerfile_dev-ros .
```

Start the container in interactive mode:

```bash
docker run -it --name ardupilot-dds ardupilot/ardupilot-dev-ros
```

Connect another bash process to the running container:

```bash
docker container exec -it ardupilot-dds /bin/bash
```

The remaining steps 1 - 5 are the same as for Ubuntu. You may need to
install MAVProxy if it is not available on the container.


```bash
python3 -m pip install -U MAVProxy
```


## Test details

The launch file replicates the following commands:

```bash
socat -d -d pty,raw,echo=0,link=./dev/ttyROS0 pty,raw,echo=0,link=./dev/ttyROS1
```

```bash
ros2 run micro_ros_agent micro_ros_agent serial --baudrate 115200 --dev ./dev/ttyROS0
```

```bash
arducopter --synthetic-clock --wipe --model quad --speedup 1 --slave 0 --instance 0 --serial1 uart:./dev/ttyROS1 --defaults $(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/copter.parm,$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/dds_serial.parm --sim-address 127.0.0.1
```

```bash
mavproxy.py --out 127.0.0.1:14550 --out 127.0.0.1:14551 --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501
```

Using individual launch files

```bash
ros2 launch ardupilot_sitl virtual_ports.launch.py tty0:=./dev/ttyROS0 tty1:=./dev/ttyROS1
```

```bash
ros2 launch ardupilot_sitl micro_ros_agent.launch.py transport:=serial baudrate:=115200 device:=./dev/ttyROS0
```

```bash
ros2 launch ardupilot_sitl sitl.launch.py synthetic_clock:=True wipe:=True model:=quad speedup:=1 slave:=0 instance:=0 serial1:=uart:./dev/ttyROS1 defaults:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/copter.parm,$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/dds_serial.parm sim_address:=127.0.0.1
```

```bash
ros2 launch ardupilot_sitl mavproxy.launch.py master:=tcp:127.0.0.1:5760 sitl:=127.0.0.1:5501
```

Using combined launch file

```bash
ros2 launch ardupilot_sitl sitl_dds_serial.launch.py \
\
tty0:=./dev/ttyROS0 \
tty1:=./dev/ttyROS1 \
\
transport:=serial \
baudrate:=115200 \
device:=./dev/ttyROS0 \
\
synthetic_clock:=True \
wipe:=True \
model:=quad \
speedup:=1 \
slave:=0 \
instance:=0 \
serial1:=uart:./dev/ttyROS1 \
defaults:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/copter.parm,$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/dds_serial.parm \
sim_address:=127.0.0.1 \
\
master:=tcp:127.0.0.1:5760 \
sitl:=127.0.0.1:5501
```

UDP version

```
ros2 launch ardupilot_sitl sitl_dds_udp.launch.py transport:=udp4 synthetic_clock:=True wipe:=False model:=quad speedup:=1 slave:=0 instance:=0 defaults:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/copter.parm,$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/dds_udp.parm sim_address:=127.0.0.1 master:=tcp:127.0.0.1:5760 sitl:=127.0.0.1:5501
```
