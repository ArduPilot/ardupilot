# Ardupilot fork by Mecatron

> While `Sub-4.5` is the default branch, the other two active branches are `Rover-4.5` and `Copter-4.5`. Please refer to those branches for specific instructions.

**Table of Contents**
- [Installation](#installation)
- [Build](#build)
  - [Build natively (RECOMMENDED)](#build-natively-recommended)
  - [Build with Docker (only encouraged for Jetson use to upload firmware, not any other use cases)](#build-with-docker-only-encouraged-for-jetson-use-to-upload-firmware-not-any-other-use-cases)
    - [Setup for the first time build](#setup-for-the-first-time-build)
    - [Subsequent builds](#subsequent-builds)
    - [Uploading firmware](#uploading-firmware)
- [Running SITL](#running-sitl)
  - [Native SITL (No JSON backend)](#native-sitl-no-json-backend)
  - [JSON SITL (With JSON backend)](#json-sitl-with-json-backend-such-as-unitymds)
  - [JSON SITL multiple vehicles](#json-sitl-multiple-vehicles)
- [Setting parameters](#setting-parameters)
  - [Setting simple parameters](#setting-simple-parameters)
  - [Uploading entire parameter files](#uploading-entire-parameter-files)
- [Available frames](#available-frames)
- [Common issues](#common-issues)

## Installation

Create a general folder if you have not done so:
```bash
mkdir ~/ardupilot
```

Clone the repository:
```bash
cd ~/ardupilot
git clone --recursive -b Sub-4.5 https://github.com/NTU-Mecatron/ardupilot.git sub-4.5
cd sub-4.5
```

## Build
### Build natively (RECOMMENDED)

Instructions are extracted from [Setting up the Build Environment (Linux/Ubuntu)](https://ardupilot.org/dev/docs/building-setup-linux.html#building-setup-linux):

```
Tools/environment_install/install-prereqs-ubuntu.sh -y
```

Then, configure the build for Software-In-The-Loop:

```bash
./waf configure --board=sitl && ./waf sub
```

### Build with Docker (only encouraged for Jetson use to upload firmware, not any other use cases)

#### Setup for the first time build

For first time setup on Jetson, you may need to enable Docker access for your user:

```bash
sudo usermod -aG docker $USER
```

> Note: You may need to restart Jetson for the changes to take effect.

Go to the root of the repo:

```bash
cd ~/ardupilot/sub-4.5
```

Build the Docker image:

```bash
docker build --rm -t ardupilot-dev .
```

Configure the build:

```bash
docker run --rm -it -v $PWD:/ardupilot ardupilot-dev ./waf configure --board=Pixhawk6C
```

> Run `docker run --rm -it -v $PWD:/ardupilot ardupilot-dev ./waf list_boards` to see the list of supported boards.

#### Subsequent builds

Whenever you make changes to the code, you only need to run the following command to build the firmware (if you already followed the setup above):

```bash
# The above command is only for configuring the build
# You need to run this command to actually build it
docker run --rm -it -v $PWD:/ardupilot ardupilot-dev ./waf sub
```

#### Uploading firmware

To upload the firmware to the Pixhawk 6C (which is usually at port `/dev/ttyACM0` and `/dev/ttyACM1`), run:

```bash
docker run --rm -it --privileged -v $PWD:/ardupilot ardupilot-dev ./waf --upload-port="/dev/ttyACM0" --upload sub
```

## Running SITL

First, navigate to the root of the repo:
```bash
cd ~/ardupilot/sub-4.5
```

Running SITL script from the root of the repo is extremely important. As extracted from Ardupilot:

```
eeprom.bin in the starting directory contains the parameters for your simulated vehicle. Always start from the same directory. It is recommended that you start in the main vehicle directory for the vehicle you are simulating, for example, start in the ArduPlane directory to simulate ArduPlane
```

Please grant access to all the scripts below by running `chmod +x <script_path>` once. Feel free to edit the scripts to modify the IP address and port if needed.

### Native SITL (No JSON backend)

```bash
./run_sitl_native.sh
```

### JSON SITL (With JSON backend such as UnityMDS)

You will need to set the environment variable `AP_JSON_IP` to the IP address of the JSON backend server in the `.profile` file. This IP address is where the JSON backend is running (e.g. UnityMDS). If Linux, it should be `127.0.0.1`. If Windows, it should be the Windows WSL2 IP address (usually something like `172.x.x.x`).

Replace `<JSON_BACKEND_IP_ADDRESS>` with the actual IP address of your JSON backend server and run the following command:
```bash
echo 'export AP_JSON_IP=<JSON_BACKEND_IP_ADDRESS>' >> ~/.profile && source ~/.profile
```

Then run the JSON backend and the SITL instance: (order doesn't matter)
```bash
./run_sitl_json.sh
```

> Note: Remember to enable all firewall rules with Unity if using Windows.

### JSON SITL multiple vehicles

Ardupilot supports running multiple SITL instances, each with different SITL and GCS ports depending on instance id. When you execute `./run_sitl_json.sh`, the default instance id is `0`, corresponding to SITL port `9002` and GCS port `14550`. Port number auto-increments by `10` for each instance id increment. For example, instance id `1` corresponds to SITL port `9012` and GCS port `14560`.

To run the first vehicle, change the ports in UnityMDS to `9002` and:
```bash
cd ~/ardupilot/<your-vehicle-type>
./run_sitl_json.sh -I 0
```

To run the second vehicle, change the ports in UnityMDS to `9012` and:
```bash
cd ~/ardupilot/<your-vehicle-type>
./run_sitl_json.sh -I 1
```

You can monitor multiple vehicles with QGroundControl by adding multiple UDP links with ports `14550`, `14560`, etc. The steps are as follows:

1. Open QGroundControl. Click on the `logo` on the top left -> `Application Settings` -> `Comm Links`.
2. Under `Links`, add a new link with the following settings:
   - Link Type: UDP
   - Listening Port: `14550` (for the first vehicle), `14560` (for the second vehicle), etc.
3. Choose `autoconnect` and click save.

## Setting parameters

You can set parameters either by setting individual parameters or uploading entire parameter files.

### Setting simple parameters

To run the example `Kevin bot` inside `UnityMDS`, you need to set its frame config. Run this in the interactive terminal:

```bash
param set FRAME_CONFIG 7
reboot
```

### Uploading entire parameter files

You can refer to example parameter files in the `params/` folder and read up their definitions on official Ardupilot documentation.

To upload entire parameter files, run the following command in the interactive terminal. For example, to test with no-GPS underwater (using DVL):

```bash
param load params/no_gps_ext_nav.parm
reboot
```

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

## Common issues

### Arming checks

There are many arming checks in the ArduPilot ecosystem designed to keep you safe; you can read more about them at [Pre-Arm Safety Checks](https://ardupilot.org/copter/docs/common-prearm-safety-checks.html). These issues happen more frequently in simulation because we cannot calibrate the vehicles. However, this should only happen upon initialization when the FC (flight controller) expects data differently from what is sent by the simulator.

#### Recommended solution: Wait

That's right! Wait for a bit for everything to return back to normal. You should wait for up to 2 minutes.

#### Disable arming checks

You should go to `QGroundControl` and manually disable one by one, and only the issues that keep popping up. It is perhaps not recommended to disable everything because the vehicle may exhibit unexpected behaviors.

### Other issues

We continually update this documentation. Please let us know asap, cheers!