# AP_UROS: micro-ROS client library

The `AP_UROS` library is an alternative to `AP_DDS` for direct ROS
integration in ArduPilot. It uses the micro-ROS client library rather
than the lower level XRCE-DDS-Client library.

The library implements the same features as `AP_DDS` and may be used as
drop in replacement.

## Build

Build the ArduPilot binaries for SITL:

```bash
./waf configure --board sitl --enable-uros
./waf build
```

Build as part of a `colcon` workspace:

```bash
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_CXX_STANDARD=17 --packages-select ardupilot_sitl
```

## Usage

The most convenient method to bringup a simulation session using the `AP_UROS`
library is to launch the runway world available in [`ardupilot_gz`](https://github.com/ArduPilot/ardupilot_gz):

```bash
ros2 launch ardupilot_gz_bringup iris_runway.launch.py rviz:=true use_gz_tf:=true  
```

Attach a MAVProxy session and enable `UROS`:

```bash
mavproxy.py --master udp:127.0.0.1:14550  --console --map
STABILIZE> param set UROS_ENABLE 1
STABILIZE> param show UROS*
STABILIZE> UROS_ENABLE      1.0
UROS_PORT        2019.0
```

The console should display various status messages prefixed with `AP: UROS: `
as the library is initialised.

### ROS 2 features

#### Nodes

```bash
$ ros2 node list
/ardupilot_uros
```

```bash
$ ros2 node info /ardupilot_uros
/ardupilot_uros
  Subscribers:
    /ap/cmd_vel: geometry_msgs/msg/TwistStamped
    /ap/joy: sensor_msgs/msg/Joy
    /ap/tf: tf2_msgs/msg/TFMessage
  Publishers:
    /ap/battery/battery0: sensor_msgs/msg/BatteryState
    /ap/clock: rosgraph_msgs/msg/Clock
    /ap/geopose/filtered: geographic_msgs/msg/GeoPoseStamped
    /ap/navsat/navsat0: sensor_msgs/msg/NavSatFix
    /ap/pose/filtered: geometry_msgs/msg/PoseStamped
    /ap/tf_static: tf2_msgs/msg/TFMessage
    /ap/time: builtin_interfaces/msg/Time
    /ap/twist/filtered: geometry_msgs/msg/TwistStamped
    /parameter_events: rcl_interfaces/msg/ParameterEvent
  Service Servers:
    /ap/arm_motors: ardupilot_msgs/srv/ArmMotors
    /ap/mode_switch: ardupilot_msgs/srv/ModeSwitch
    /ardupilot_uros/describe_parameters: rcl_interfaces/srv/DescribeParameters
    /ardupilot_uros/get_parameter_types: rcl_interfaces/srv/GetParameterTypes
    /ardupilot_uros/get_parameters: rcl_interfaces/srv/GetParameters
    /ardupilot_uros/list_parameters: rcl_interfaces/srv/ListParameters
    /ardupilot_uros/set_parameters: rcl_interfaces/srv/SetParameters
  Service Clients:

  Action Servers:

  Action Clients:
  ```

#### Topics

List topics published by `ardupilot_uros`:

```bash
% ros2 topic list | grep /ap
/ap/battery/battery0
/ap/clock
/ap/cmd_vel
/ap/geopose/filtered
/ap/joy
/ap/navsat/navsat0
/ap/pose/filtered
/ap/tf
/ap/tf_static
/ap/time
/ap/twist/filtered
```

Subscribe to a filtered pose:

```bash
% ros2 topic echo /ap/pose/filtered --once 
header:
  stamp:
    sec: 1696260581
    nanosec: 150658000
  frame_id: base_link
pose:
  position:
    x: -0.009078041650354862
    y: 0.011131884530186653
    z: 0.10999999940395355
  orientation:
    x: -0.0008264112402684987
    y: 6.37705234112218e-05
    z: 0.7135183215141296
    w: 0.700636088848114
---
```

Publish a `Joy` message:

```bash
$ ros2 topic pub /ap/joy sensor_msgs/msg/Joy "{axes: [1, 1, 1, 1]}" --once
publisher: beginning loop
publishing #1: sensor_msgs.msg.Joy(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id=''), axes=[1.0, 1.0, 1.0, 1.0], buttons=[])
```

The MAVProxy console should show:

```console
AP: UROS: sensor_msgs/Joy: 1.000000, 1.000000, 1.000000, 1.000000
```

#### Services

```bash
$ ros2 service list | grep /ap
/ap/arm_motors
/ap/mode_switch
```

Switch to `GUIDED` mode (`mode=4`):

```bash
$ ros2 service call /ap/mode_switch ardupilot_msgs/srv/ModeSwitch "{mode: 4}"
requester: making request: ardupilot_msgs.srv.ModeSwitch_Request(mode=4)

response:
ardupilot_msgs.srv.ModeSwitch_Response(status=True, curr_mode=4)
```

```console
Mode GUIDED
AP: UROS: Request for Mode Switch : SUCCESS
```

Arm motors:

```bash
ros2 service call /ap/arm_motors ardupilot_msgs/srv/ArmMotors "{arm: true}"
requester: making request: ardupilot_msgs.srv.ArmMotors_Request(arm=True)

response:
ardupilot_msgs.srv.ArmMotors_Response(result=True)
```

```console
AP: UROS: Request for arming received
AP: Arming motors
AP: UROS: Request for Arming/Disarming : SUCCESS
ARMED
```

#### Parameters

The parameter server is proof of concept only and is not integrated
with ArduPilot's parameter system.

Create:

```bash
ros2 param set /ardupilot_uros GPS_TYPE 11
```

Dump:

```bash
ros2 param dump /ardupilot_uros 
/ardupilot_uros:
  ros__parameters:
    GPS_TYPE: 11
```

Get:

```bash
$ ros2 param get /ardupilot_uros GPS_TYPE
Integer value is: 11
```

Set:

```bash
$ ros2 param get /ardupilot_uros GPS_TYPE
Integer value is: 11
```

```bash
$ ros2 param set /ardupilot_uros GPS_TYPE 1
Set parameter successful

```bash
$ ros2 param get /ardupilot_uros GPS_TYPE
Integer value is: 1
```


## Appendix A: ESP32

Notes for building the micro-ROS client library using the package
`micro_ros_espidf_component`.

### Config

- Update `app-colcon.meta` to allow more publishers and subscribers.
  - 1 node
  - 16 pub/sub/service/client
- Update `libmicroros.mk` to add message and service interface packages.
  - Added `ardupilot` for `ardupilot_msgs`
  - Added `geographic_info` for `geographic_msgs`
  - Added `geometry2` for `tf2_msgs`
  - Added `ardupilot` for  `ardupilot_msgs`

### Build (standalone example)

This refers to a standalone example used to test the micro-ROS client build.

```bash
cd ./firmware/toolchain
. ./esp-idf/export.sh
```

```bash
cd ./firmware/esp32_examples/ardupilot_uros
idf.py set-target esp32
idf.py build
idf.py -p /dev/cu.usbserial-0001 flash
```

### Run

```bash
cd ./firmware/esp32_examples/ardupilot_uros
idf.py monitor
```

```bash
cd ./firmware/esp32_examples/ardupilot_uros
idf.py monitor
```

```bash
ros2 run micro_ros_agent micro_ros_agent udp4 --port 2019 -v6
```