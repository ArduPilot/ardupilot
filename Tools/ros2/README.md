# ArduPilot ROS 2 packages

 This directory contains ROS 2 packages and configuration files for running
 ROS 2 processes and nodes that communicate with the ArduPilot DDS client
 library using the microROS agent. It contains the following packages:
 
#### `ardupilot_sitl`

A `colcon` package for building and running ArduPilot SITL using the ROS 2 CLI.
For example `ardurover` SITL may be launched with:

```bash
ros2 launch ardupilot_sitl sitl.launch.py command:=ardurover model:=rover
```

#### `ardupilot_dds_test`

A `colcon` package for testing communication between `micro_ros_agent` and the
ArduPilot `AP_DDS` client library.

## Prerequisites

The packages depend on:

- [ROS 2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)


## Install Ubuntu

#### 1. Create a workspace folder

```bash
mkdir -p ~/ros_ws/src && cd ~/ros_ws/src
```

The ROS 2 tutorials contain more details regarding [ROS 2 workspaces](https://docs.ros.org/en/humble/Tutorials/Workspace/Creating-A-Workspace.html).

#### 2. Get the `ros2.repos` file

```bash
cd ~/ros2_ws/src
wget https://raw.githubusercontent.com/srmainwaring/ardupilot/pr/pr-dds-launch-tests/Tools/ros2/ros2.repos
vcs import --recursive < ros2.repos
```

#### 3. Update dependencies

```bash
cd ~/ros_ws
source /opt/ros/humble/setup.bash
sudo apt update
rosdep update
rosdep install --rosdistro humble --from-paths src
```

#### 4. Build

```bash
cd ~/ros_ws
colcon build --packages-select micro_ros_agent
colcon build --packages-select ardupilot_sitl
colcon build --packages-select ardupilot_dds_tests
```

#### 5. Test

```bash
source ./install/setup.bash
colcon test --pytest-args -s -v --event-handlers console_cohesion+ --packages-select ardupilot_dds_tests
colcon test-result --all --verbose
```

## Install macOS

The install procedure on macOS is similar, except that all dependencies
must be built from source and additional compiler flags are needed.

#### 1. Create a workspace folder

```bash
mkdir -p ~/ros_ws/src && cd ~/ros_ws/src
```

#### 2. Get the `ros2.repos` file

The `ros2_macos.repos` includes additional dependencies to build:

```bash
cd ~/ros2_ws/src
wget https://raw.githubusercontent.com/srmainwaring/ardupilot/pr/pr-dds-launch-tests/Tools/ros2/ros2_macos.repos
vcs import --recursive < ros2_macos.repos
```

#### 3. Update dependencies

```bash
cd ~/ros_ws
source /<path_to_your_ros_humble_workspace>/install/setup.zsh
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
--event-handlers=desktop_notification- \
--packages-select \
micro_ros_msgs \
micro_ros_agent \
ardupilot_sitl \
ardupilot_dds_tests
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
pip install -U MAVProxy
```


## Test details

The launch file replicates the following commands:

```bash
socat -d -d pty,raw,echo=0,link=./dev/ttyROS0 pty,raw,echo=0,link=./dev/ttyROS1
```

```bash
ros2 run micro_ros_agent micro_ros_agent serial --baudrate 115200 --dev ./dev/ttyROS0 --refs $(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/dds_xrce_profile.xml
```

```bash
arducopter --synthetic-clock --wipe --model quad --speedup 1 --slave 0 --instance 0 --uartC uart:./dev/ttyROS1 --defaults $(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/copter.parm,$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/dds.parm --sim-address 127.0.0.1
```

```bash
mavproxy.py --out 127.0.0.1:14550 --out 127.0.0.1:14551 --master tcp:127.0.0.1:5760 --sitl 127.0.0.1:5501
```

Using individual launch files

```bash
ros2 launch ardupilot_sitl virtual_ports.launch.py tty0:=./dev/ttyROS0 tty1:=./dev/ttyROS1
```

```bash
ros2 launch ardupilot_sitl micro_ros_agent.launch.py refs:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/dds_xrce_profile.xml baudrate:=115200 device:=./dev/ttyROS0
```

```bash
ros2 launch ardupilot_sitl sitl.launch.py synthetic_clock:=True wipe:=True model:=quad speedup:=1 slave:=0 instance:=0 uartC:=uart:./dev/ttyROS1 defaults:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/copter.parm,$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/dds.parm sim_address:=127.0.0.1
```

```bash
ros2 launch ardupilot_sitl mavproxy.launch.py master:=tcp:127.0.0.1:5760 sitl:=127.0.0.1:5501
```

Using combined launch file

```bash
ros2 launch ardupilot_sitl sitl_dds.launch.py \
\
tty0:=./dev/ttyROS0 \
tty1:=./dev/ttyROS1 \
\
refs:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/dds_xrce_profile.xml \
baudrate:=115200 \
device:=./dev/ttyROS0 \
\
synthetic_clock:=True \
wipe:=True \
model:=quad \
speedup:=1 \
slave:=0 \
instance:=0 \
uartC:=uart:./dev/ttyROS1 \
defaults:=$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/copter.parm,$(ros2 pkg prefix ardupilot_sitl)/share/ardupilot_sitl/config/default_params/dds.parm \
sim_address:=127.0.0.1 \
\
master:=tcp:127.0.0.1:5760 \
sitl:=127.0.0.1:5501
```

## References

### Configuring linters and formatters for ROS 2 projects

- [ROS 2 Code style and language versions](https://docs.ros.org/en/humble/The-ROS2-Project/Contributing/Code-Style-Language-Versions.html).
- [Configuring Flake8](https://flake8.pycqa.org/en/latest/user/configuration.html).
- [ament_lint_auto](https://github.com/ament/ament_lint/blob/humble/ament_lint_auto/doc/index.rst).
- [How to configure ament python linters in CMakeLists?](https://answers.ros.org/question/351012/how-to-configure-ament-python-linters-in-cmakelists/).
- [Using black and flake8 in tandem](https://sbarnea.com/lint/black/).

