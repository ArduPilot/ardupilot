# Testing with DDS/micro-Ros

## Architecture

Ardupilot contains the DDS Client library, which can run as SITL. Then, the DDS application runs a ROS2 node, an EProsima Integration Service, and the MicroXRCE Agent. The two systems communicate over serial, which is the only supported protocol in Ardupilot MicroXCE DDS at this time.

```mermaid
---
title: Hardware Serial Port Loopback
---
graph LR

  subgraph Linux Computer

    subgraph Ardupilot SITL
      veh[sim_vehicle.py] <--> xrceClient[EProsima Micro XRCE DDS Client]
      xrceClient <--> port1[devUSB1]
    end

    subgraph DDS Application
      ros[ROS2 Node] <--> agent[Micro ROS Agent]
      agent <--> port2[devUSB2]
    end

    port1 <--> port2

  end
```

Currently, serial is the only supported transport, but there are plans to add IP-based transport over ethernet. 

## Installing Build Dependencies

While DDS support in Ardupilot is mostly through git submodules, another tool needs to be available on your system: Micro XRCE DDS Gen.

- Go to a directory on your system to clone the repo (perhaps next to `ardupilot`)
- Install java
  ```console
  sudo apt install default-jre
  ````
- Follow instructions [here](https://micro-xrce-dds.docs.eprosima.com/en/latest/installation.html#installing-the-micro-xrce-dds-gen-tool) to install the latest version of the generator using Ardupilot's mirror
  ```console
  git clone --recurse-submodules https://github.com/ardupilot/Micro-XRCE-DDS-Gen.git
  cd Micro-XRCE-DDS-Gen
  ./gradlew assemble
  ```

- Add the generator directory to $PATH. 
  ```console
  # Add this to ~/.bashrc

  export PATH=$PATH:/your/path/to/Micro-XRCE-DDS-Gen/scripts
  ```
- Test it
  ```console
  cd /path/to/ardupilot
  microxrceddsgen -version
  # openjdk version "11.0.18" 2023-01-17
  # OpenJDK Runtime Environment (build 11.0.18+10-post-Ubuntu-0ubuntu122.04)
  # OpenJDK 64-Bit Server VM (build 11.0.18+10-post-Ubuntu-0ubuntu122.04, mixed mode, sharing)
  # microxrceddsgen version: 1.0.0beta2
  ```

> :warning: **If you have installed FastDDS or FastDDSGen globally on your system**:
eProsima's libraries and the packaging system in Ardupilot are not determistic in this scenario.
You may experience the wrong version of a library brought in, or runtime segfaults.
For now, avoid having simultaneous local and global installs.
If you followed the [global install](https://fast-dds.docs.eprosima.com/en/latest/installation/sources/sources_linux.html#global-installation)
section, you should remove it and switch to local install.

## Setup serial for SITL with DDS

On Linux, creating a virtual serial port will be necessary to use serial in SITL, because of that install socat.

```
sudo apt-get update
sudo apt-get install socat
```

## Setup ardupilot for SITL with DDS

Set up your [SITL](https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html).
Run the simulator with the following command. Take note how two parameters need adjusting from default to use DDS.
| Name | Description |
| - | - |
| SERIAL1_BAUD | The serial baud rate for DDS |
| SERIAL1_PROTOCOL | Set this to 45 to use DDS on the serial port |
```bash
# Wipe params till you see "AP: ArduPilot Ready"
# Select your favorite vehicle type
sim_vehicle.py -w -v ArduPlane

# Set params
param set SERIAL1_BAUD 115
# See libraries/AP_SerialManager/AP_SerialManager.h AP_SerialManager SerialProtocol_DDS_XRCE
param set SERIAL1_PROTOCOL 45
```
## Setup ROS 2 and micro-ROS

Follow the steps to use the microROS Agent

- Install ROS Humble (as described here)

  - https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

- Install and run the microROS agent (as descibed here). Make sure to use the `humble` branch.
  - Follow [the instructions](https://micro.ros.org/docs/tutorials/core/first_application_linux/) for the following:

    - Do "Installing ROS 2 and the micro-ROS build system"
      - Skip the docker run command, build it locally instead
    - Skip "Creating a new firmware workspace"
    - Skip "Building the firmware"
    - Do "Creating the micro-ROS agent"
    - Source your ROS workspace

Until this [PR](https://github.com/micro-ROS/micro-ROS.github.io/pull/401) is merged, ignore the notes about `foxy`. It works on `humble`.

## Using the ROS2 CLI to Read Ardupilot Data

After your setups are complete, do the following:

- Source the ros2 installation
  ```bash
  source /opt/ros/humble/setup.bash
  ```
- Start a virtual serial port with socat. Take note of the two `/dev/pts/*` ports. If yours are different, substitute as needed.
  ```bash
  socat -d -d pty,raw,echo=0 pty,raw,echo=0
  >>> 2023/02/21 05:26:06 socat[334] N PTY is /dev/pts/1
  >>> 2023/02/21 05:26:06 socat[334] N PTY is /dev/pts/2
  >>> 2023/02/21 05:26:06 socat[334] N starting data transfer loop with FDs [5,5] and [7,7]
  ```
- Run the microROS agent
  ```bash
  cd ardupilot/libraries/AP_DDS
  ros2 run micro_ros_agent micro_ros_agent serial -b 115200 -D /dev/pts/2  -r dds_xrce_profile.xml # (assuming we are using tty/pts/2 for DDS Application)
  ```
- Run SITL (remember to kill any terminals running ardupilot SITL beforehand)
  ```bash
  sim_vehicle.py -v ArduPlane -D --console --enable-dds -A "--uartC=uart:/dev/pts/1" # (assuming we are using /dev/pts/1 for Ardupilot SITL)
  ```
- You should be able to see the agent here and view the data output.
  ```bash
  $ ros2 node list
  /Ardupilot_DDS_XRCE_Client

  $ ros2 topic list  -v
  Published topics:
   * /ROS2_NavSatFix0 [sensor_msgs/msg/NavSatFix] 1 publisher
   * /ROS2_Time [builtin_interfaces/msg/Time] 1 publisher
   * /parameter_events [rcl_interfaces/msg/ParameterEvent] 1 publisher
   * /rosout [rcl_interfaces/msg/Log] 1 publisher
   * /tf [tf2_msgs/msg/TFMessage] 1 publisher

  Subscribed topics:

  $ ros2 topic hz /ROS2_Time
  average rate: 50.115
          min: 0.012s max: 0.024s std dev: 0.00328s window: 52

  $ ros2 topic echo /ROS2_Time 
  sec: 1678668735
  nanosec: 729410000
  ---
  ```

  The static transforms for enabled sensors are also published, and can be recieved like so:
  ```console
  ros2 topic echo /tf --qos-depth 1 --qos-history keep_last --qos-reliability reliable --qos-durability transient_local --once
  ```
  In order to consume the transforms, it's highly recommended to [create and run a transform broadcaster in ROS 2](https://docs.ros.org/en/humble/Concepts/About-Tf2.html#tutorials). 


## Contributing to AP_DDS library
### Adding DDS messages to Ardupilot

Unlike the use of ROS 2 `.msg` files, since Ardupilot supports native DDS, the message files follow [OMG IDL DDS v4.2](https://www.omg.org/spec/IDL/4.2/PDF).
This package is intended to work with any `.idl` file complying with those extensions.

Over time, these restrictions will ideally go away. 

To get a new IDL file from ROS2, follow this process:
```console
cd ardupilot
source /opt/ros/humble/setup.bash
# Find the IDL file
find /opt/ros/$ROS_DISTRO -type f -wholename \*builtin_interfaces/msg/Time.idl
# Create the directory in the source tree if it doesn't exist similar to the one found in the ros directory
mkdir -p libraries/AP_DDS/Idl/builtin_interfaces/msg/
# Copy the IDL
cp /opt/ros/humble/share/builtin_interfaces/msg/Time.idl libraries/AP_DDS/Idl/builtin_interfaces/msg/
# Build the code again with the `--enable-dds` flag as described above 
```

### Development Requirements

Astyle is used to format the C++ code in AP_DDS. This is required for CI to pass the build.
See [Tools/CodeStyle/ardupilot-astyle.sh](../../Tools/CodeStyle/ardupilot-astyle.sh).

```console
./Tools/CodeStyle/ardupilot-astyle.sh libraries/AP_DDS/*.h libraries/AP_DDS/*.cpp
```

Pre-commit is used for other things like formatting python and XML code. 
This will run the tools automatically when you commit. If there are changes, just add them back your staging index and commit again.

1. Install [pre-commit](https://pre-commit.com/#installation) python package.
1. Install ArduPilot's hooks in the root of the repo, then commit like normal
  ```console
  cd ardupilot
  pre-commit install
  git commit
  ```
