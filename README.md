# Ardupilot fork by Mecatron

This fork contains custom frame configs stored at [AP_Motors6DOF](libraries/AP_Motors/AP_Motors6DOF.cpp) for Mecatron use.

It also contains guide for running ArduSub SITL (Software In The Loop) so that you can run pixhawk package without the need of a physical pixhawk.
However, if you do not intend to run SITL (simulation), ignore all SITL related commands and just follow the firmware build and upload guide.

**Table of Contents**
- [Installation](#installation)
- [Build natively](#build-natively) (RECOMMENDED)
- [Build with Docker](#build-with-docker)
- [Uploading firmware](#uploading-firmware)
- [Uploading parameters](#uploading-parameters)
- [Running SITL](#running-sitl)
- [Available frames](#available-frames)

## Installation

```bash
cd ~/
git clone --recursive -b Sub-4.5 https://github.com/NTU-Mecatron/ardupilot.git
cd ardupilot
```

## Build natively

Instructions are extracted from [Setting up the Build Environment (Linux/Ubuntu)](https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux):

```
Tools/environment_install/install-prereqs-ubuntu.sh -y && . ~/.profile
```

Then, configure the build for Software-In-The-Loop:

```bash
./waf configure --board=sitl && ./waf sub
```

To run SITL:

## Build with Docker

### Setup for the first time build

For first time setup on Jetson, you may need to enable Docker access for your user:

```bash
sudo usermod -aG docker $USER
```

> Note: You may need to restart Jetson for the changes to take effect.

Build the Docker image:

```bash
docker build --rm -t ardupilot-dev .
```

After that, run **either** of the following commands to configure the build:
**

```bash
# If you are configuring on your own computer for SITL
docker run --rm -it -v $PWD:/ardupilot ardupilot-dev ./waf configure --board=sitl

# If you are working with a physical Pixhawk 6C (for example, to upload firmware)
docker run --rm -it -v $PWD:/ardupilot ardupilot-dev ./waf configure --board=Pixhawk6C
```

### Subsequent builds

Whenever you make changes to the code, you only need to run the following command to build the firmware (if you already followed the setup above):

```bash
# The above command is only for configuring the build
# You need to run this command to actually build it
docker run --rm -it -v $PWD:/ardupilot ardupilot-dev ./waf sub
```

## Uploading firmware

To upload the firmware to the Pixhawk 6C (which is usually at port `/dev/ttyACM0` and `/dev/ttyACM1`), run:

```bash
docker run --rm -it --privileged -v $PWD:/ardupilot ardupilot-dev ./waf --upload-port="/dev/ttyACM0" --upload sub
```

## Uploading parameters

We have created a parameter file [pix6c.parm](pix6c.parm) for custom use at Mecatron. This parameter file is working on the assumption that pin 1 for gimbal, pin 2 for marker, pin 3 for torpedo, pin 4 for gripper. Meaning pin 1 and 4 are pwm style, while pin 2 and 3 are relay style.

To upload the parameter file to the Pixhawk 6C, run:

```bash
mavproxy.py
param load <absolute_path_to_pix6c.parm>
reboot
```
If it throws an error "Unable to find parameter RELAY10_PIN", this is because it is a hidden parameter that only appears after you set the RELAY10_FUNCTION to something other than 0. To fix, you need to unplug and replug the Pixhawk (or reboot the Jetson) and try again.

## Running SITL

```bash
sim_vehicle.py -v ArduSub --out udp:0.0.0.0:14551 -L RATBeach 
```

or

```bash
docker run --rm -it -v $PWD:/ardupilot ardupilot-dev sim_vehicle.py -v ArduSub --out udp:0.0.0.0:14551 -L RATBeach
```

> Note: after following the above setup to build SITL, this is *the only command* you need to run everytime to start SITL.

`--out` flag is used to specify the IP address and port to send the MAVLink messages to. If you are running your pixhawk package in WSL2, you need to run `ifconfig` in WSL2 to find out its IP address and use that IP address.
If you are running the package in Docker, you might need to add `-p 14550` flag when running the container, or add the port manually, and use `127.0.0.1` as the IP address.

If you use WSL2 (meaning on Windows), you should clone this package and run docker in WSL2 to make it faster.

## Available frames

For Kevin bot:

```cpp
case SUB_FRAME_CUSTOM:
  //                 Motor #              Roll Factor     Pitch Factor    Yaw Factor      Throttle Factor     Forward Factor      Lateral Factor  Testing Order
  // For Primary bot
  _frame_class_string = "CUSTOM_PRIMARY";
  add_motor_raw_6dof(AP_MOTORS_MOT_1,     0,              0,              -1.0f,          0,                  1.0f,               0,              1);
  add_motor_raw_6dof(AP_MOTORS_MOT_2,     0,              0,              1.0f,           0,                  1.0f,               0,              2);
  add_motor_raw_6dof(AP_MOTORS_MOT_3,     1.0f,           -1.0f,          -0.5,           -1.0f,              0,                  -1.0f,          3);
  add_motor_raw_6dof(AP_MOTORS_MOT_4,     -1.0f,          -1.0f,          0.5,            -1.0f,              0,                  1.0f,           4);
  add_motor_raw_6dof(AP_MOTORS_MOT_5,     1.0f,           1.0f,           -0.5,           -1.0f,              0,                  1.0f,           5);
  add_motor_raw_6dof(AP_MOTORS_MOT_6,     -1.0f,          1.0f,           0.5,            -1.0f,              0,                  -1.0f,          6);
  break;
```

For Lucy frame:

```cpp
case SUB_FRAME_SIMPLEROV_5:
  //                 Motor #              Roll Factor     Pitch Factor    Yaw Factor      Throttle Factor     Forward Factor      Lateral Factor  Testing Order
  // For Secondary bot
  _frame_class_string = "CUSTOM_SECONDARY";
  add_motor_raw_6dof(AP_MOTORS_MOT_1,     0,              0,               -1.0f,          0,                  1.0f,               0,              1);
  add_motor_raw_6dof(AP_MOTORS_MOT_2,     0,              0,               1.0f,           0,                  1.0f,               0,              2);
  add_motor_raw_6dof(AP_MOTORS_MOT_3,     1.0f,           -1.0f,           0,              -1.0f,              0,                  0,              3);
  add_motor_raw_6dof(AP_MOTORS_MOT_4,     -1.0f,          -1.0f,           0,              -1.0f,              0,                  0,              4);
  add_motor_raw_6dof(AP_MOTORS_MOT_5,     0,              1.0f,            0,              -1.0f,              0,                  0,              5);
  break; 
```