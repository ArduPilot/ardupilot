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

Follow the installation instructions in the [ArduPilot Wiki](https://ardupilot.org/dev/docs/ros2.html)


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
