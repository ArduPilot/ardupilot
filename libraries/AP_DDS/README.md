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
- Follow instructions [here](https://micro-xrce-dds.docs.eprosima.com/en/latest/installation.html#installing-the-micro-xrce-dds-gen-tool) to install the generator, but use `develop` branch instead of `master` (for now).
  ```console
  git clone -b develop --recurse-submodules https://github.com/eProsima/Micro-XRCE-DDS-Gen.git
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

## Parameters for DDS

| Name | Description |
| - | - |
| SERIAL1_BAUD | The serial baud rate for DDS |
| SERIAL1_PROTOCOL | Set this to 45 to use DDS on the serial port |


## Testing with a UART

On Linux, first create a virtual serial port for use with SITL like [this](https://stackoverflow.com/questions/52187/virtual-serial-port-for-linux)

```
sudo apt-get update
sudo apt-get install socat
```

Then, start a virtual serial port with socat. Take note of the two `/dev/pts/*` ports. If yours are different, substitute as needed.
```
socat -d -d pty,raw,echo=0 pty,raw,echo=0
>>> 2023/02/21 05:26:06 socat[334] N PTY is /dev/pts/1
>>> 2023/02/21 05:26:06 socat[334] N PTY is /dev/pts/2
>>> 2023/02/21 05:26:06 socat[334] N starting data transfer loop with FDs [5,5] and [7,7]
```

Set up your [SITL](https://ardupilot.org/dev/docs/setting-up-sitl-on-linux.html).
Run the simulator with the following command (assuming we are using /dev/pts/1 for Ardupilot SITL). Take note how two parameters need adjusting from default to use DDS.
```
# Wipe params till you see "AP: ArduPilot Ready"
# Select your favorite vehicle type
sim_vehicle.py -w -v ArduPlane

# Set params
param set SERIAL1_BAUD 115
# See libraries/AP_SerialManager/AP_SerialManager.h AP_SerialManager SerialProtocol_DDS_XRCE
param set SERIAL1_PROTOCOL 45
```

## Starting with microROS Agent

Follow the steps to use the microROS Agent

- Install ROS Humble (as described here)

  - https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

- Install and run the microROS agent (as descibed here). Make sure to use the `humble` branch.

  - https://micro.ros.org/docs/tutorials/core/first_application_linux/

Until this [PR](https://github.com/micro-ROS/micro-ROS.github.io/pull/401) is merged, ignore the notes about `foxy`. It works on `humble`.

Follow the instructions for the following:

* Do "Installing ROS 2 and the micro-ROS build system"
  * Skip the docker run command, build it locally instead
* Skip "Creating a new firmware workspace"
* Skip "Building the firmware"
* Do "Creating the micro-ROS agent"
* Source your ROS workspace

Run microROS agent with the following command

```bash
cd ardupilot/libraries/AP_DDS
ros2 run micro_ros_agent micro_ros_agent serial -b 115200 -D /dev/pts/2  -r dds_xrce_profile.xml # (assuming we are using tty/pts/2 for Ardupilot)
```

## Tutorial

### Using the ROS2 CLI to Read Ardupilot Data

If you have installed the microROS agent and ROS 2 Humble

- Source the ros2 installation
  ```bash
  source /opt/ros/humble/setup.bash
  ```
- Run the microROS agent
  ```bash
  cd ardupilot/libraries/AP_DDS
  ros2 run micro_ros_agent micro_ros_agent serial -b 115200 -D /dev/pts/2  -r dds_xrce_profile.xml # (assuming we are using tty/pts/2 for Ardupilot)
  ```
- Run SITL
  ```bash
  sim_vehicle.py -v ArduPlane -D --console --enable-dds -A "--uartC=uart:/dev/pts/1"
  ```
- You should be able to see the agent here and view the data output.
  ```bash
  $ ros2 node list
  /Ardupilot_DDS_XRCE_Client

  $ ros2 topic list  -v
  Published topics:
  * /ROS2_Time [builtin_interfaces/msg/Time] 1 publisher
  * /parameter_events [rcl_interfaces/msg/ParameterEvent] 1 publisher
  * /rosout [rcl_interfaces/msg/Log] 1 publisher

  Subscribed topics:

  $ ros2 topic hz /ROS2_Time
  average rate: 50.115
          min: 0.012s max: 0.024s std dev: 0.00328s window: 52

  $ ros2 topic echo /ROS2_Time 
  sec: 1678668735
  nanosec: 729410000
  ---
  ```

## Adding DDS messages to Ardupilot

Unlike the use of ROS 2 `.msg` files, since Ardupilot supports native DDS, the message files follow [OMG IDL DDS v4.2](https://www.omg.org/spec/IDL/4.2/PDF).
This package is intended to work with any `.idl` file complying with those extensions, with some limitations. 

1. IDL files need to be in the same folder, and modified includes.
1. Topic types can't use alias types.
1. Arrays need manually edited type names. 

Over time, these restrictions will ideally go away. 

To get a new IDL file from ROS2, follow this process:
```console
cd ardupilot
source /opt/ros/humble/setup.bash
# Find the IDL file
find /opt/ros/$ROS_DISTRO -type f -wholename \*builtin_interfaces/msg/Time.idl
# Create the directory in the source tree if it doesn't exist
mkdir -p libraries/AP_DDS_Client/Idl/builtin_interfaces/msg/
# Copy the IDL
cp -r /opt/ros/humble/share/builtin_interfaces/msg/Time.idl libraries/AP_DDS_Client/Idl/builtin_interfaces/msg/
# Now, apply the mods manually to be compliant with MicroXRCEDDSGen limitations
# Create an output directory to test it
mkdir -p /tmp/xrce_out
# Run the generator
microxrceddsgen -replace -d /tmp/xrce_out libraries/AP_DDS_Client/Idl/builtin_interfaces/msg/Time.idl
# cat /tmp/xrce_out/
```
