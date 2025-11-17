# ArduPilot Docker Setup for Remote Gazebo Connection

This document describes how to build and run ArduPilot SITL in a Docker container that connects to a remote Gazebo simulator on a separate Ubuntu system.

## Overview

- **ArduPilot SITL**: Runs in Docker container (headless, no Gazebo)
- **ROS2 + DDS**: Communication between ArduPilot and external systems
- **Remote Gazebo**: Runs on a separate Ubuntu machine

## Docker Configuration

### Available Dockerfile

The repository includes a production-ready Dockerfile at `/Dockerfile` that:
- Uses Ubuntu 22.04 base image
- Installs all ArduPilot dependencies via `install-prereqs-ubuntu.sh`
- Includes Micro-XRCE-DDS-Gen for ROS2 DDS communication
- Sets up non-root user for security
- Skips graphical dependencies (`SKIP_AP_GRAPHIC_ENV=1`)

### Building the Docker Image

```bash
# Build ArduPilot SITL image (headless)
docker build -t ardupilot-sitl:latest .

# Or with custom arguments
docker build \
  --build-arg BASE_IMAGE=ubuntu \
  --build-arg TAG=22.04 \
  --build-arg SKIP_AP_GRAPHIC_ENV=1 \
  -t ardupilot-sitl:latest .
```

## Running SITL with Remote Gazebo

### Key Parameters

The SITL binaries support connecting to external simulators via:

- `--sim-address=<IP>`: IP address of remote Gazebo simulator (default: 127.0.0.1)
- `--model=<model>`: Simulation model (e.g., `gazebo-iris`, `gazebo-rover`, `gazebo-zephyr`)

### Available Gazebo Models

| Vehicle | Model Name | Default Params |
|---------|-----------|----------------|
| Copter | `gazebo-iris` | `default_params/copter.parm`, `default_params/gazebo-iris.parm` |
| Plane | `gazebo-zephyr` | `default_params/gazebo-zephyr.parm` |
| Rover | `gazebo-rover` | `default_params/rover.parm`, `default_params/rover-skid.parm` |
| Sub | `gazebo-bluerov2` | `default_params/sub.parm` |

### Docker Run Command

```bash
# Run SITL container connecting to remote Gazebo
docker run -it --rm \
  --network host \
  --name ardupilot-sitl \
  -v $(pwd):/ardupilot \
  ardupilot-sitl:latest \
  bash -c "
    cd /ardupilot && \
    ./Tools/autotest/sim_vehicle.py \
      -v ArduCopter \
      --model gazebo-iris \
      --sim-address=<GAZEBO_HOST_IP> \
      --no-mavproxy \
      --out=udp:<YOUR_GCS_IP>:14550
  "
```

### Docker Compose Example

Create a `docker-compose.yml`:

```yaml
version: '3.8'

services:
  ardupilot-sitl:
    build:
      context: .
      dockerfile: Dockerfile
    container_name: ardupilot-sitl
    network_mode: host
    volumes:
      - .:/ardupilot
      - ./logs:/tmp/buildlogs
    environment:
      - SIM_ADDRESS=${GAZEBO_HOST_IP:-192.168.1.100}
      - VEHICLE_TYPE=${VEHICLE_TYPE:-ArduCopter}
      - MODEL=${MODEL:-gazebo-iris}
    command: >
      bash -c "
        source /home/ardupilot/.ardupilot_env &&
        cd /ardupilot &&
        ./Tools/autotest/sim_vehicle.py
          -v $${VEHICLE_TYPE}
          --model $${MODEL}
          --sim-address=$${SIM_ADDRESS}
          --no-mavproxy
          --add-param-file Tools/autotest/default_params/gazebo-iris.parm
      "
```

Run with:
```bash
GAZEBO_HOST_IP=192.168.1.100 docker-compose up
```

## ROS2 DDS Configuration

### DDS Parameters

The following parameters enable DDS communication:

```
DDS_ENABLE 1
DDS_UDP_PORT 2019
```

These are configured in `Tools/ros2/ardupilot_sitl/config/default_params/dds_udp.parm`.

### ROS2 Launch with Remote Connection

```bash
# From the ROS2 workspace
ros2 launch ardupilot_sitl sitl_dds_udp.launch.py \
  transport:=udp4 \
  synthetic_clock:=True \
  wipe:=False \
  model:=gazebo-iris \
  sim_address:=<GAZEBO_HOST_IP> \
  defaults:=/path/to/your/params.parm
```

### Micro ROS Agent Setup

The Dockerfile already includes Micro-XRCE-DDS-Gen. For the micro ROS agent:

```bash
# Run micro_ros_agent (in ROS2 environment)
ros2 run micro_ros_agent micro_ros_agent udp4 --port 2019
```

## Custom Parameter Configuration

### Creating Your Own Parameter File

1. Copy an existing parameter file:
```bash
cp Tools/autotest/default_params/copter.parm my_params.parm
```

2. Add DDS configuration:
```bash
cat Tools/ros2/ardupilot_sitl/config/default_params/dds_udp.parm >> my_params.parm
```

3. Customize your parameters:
```
# my_params.parm
DDS_ENABLE 1
DDS_UDP_PORT 2019

# Custom parameters
SCHED_LOOP_RATE 400
INS_LOG_BAT_CNT 1024
LOG_BITMASK 176126

# Your specific parameters here
# ...
```

4. Run with custom parameters:
```bash
docker run -it --rm \
  --network host \
  -v $(pwd):/ardupilot \
  ardupilot-sitl:latest \
  bash -c "
    cd /ardupilot && \
    ./Tools/autotest/sim_vehicle.py \
      -v ArduCopter \
      --model gazebo-iris \
      --sim-address=<GAZEBO_HOST_IP> \
      --add-param-file /ardupilot/my_params.parm
  "
```

## Network Configuration

### Required Ports

| Port | Protocol | Purpose |
|------|----------|---------|
| 5760 | TCP | MAVLink |
| 5501 | UDP | SITL simulation data |
| 5502 | UDP | RC input |
| 2019 | UDP | DDS communication |
| 14550 | UDP | MAVProxy/GCS |
| 14551 | UDP | Secondary GCS output |

### Firewall Rules (on Gazebo host)

```bash
# Allow SITL ports
sudo ufw allow 5501/udp
sudo ufw allow 5502/udp
sudo ufw allow 9002/udp  # Gazebo default port
```

## Alternative: ArduPilot Dev Docker (Official)

For a more complete ROS2 + ArduPilot setup, use the official dev docker:

```bash
# Clone official docker repo
git clone https://github.com/ArduPilot/ardupilot_dev_docker.git
cd ardupilot_dev_docker/docker

# Build ROS2-enabled image
docker build -t ardupilot/ardupilot-dev-ros -f Dockerfile_dev-ros .

# Run container
docker run -it --name ardupilot-dds ardupilot/ardupilot-dev-ros
```

## Building SITL Binary for Docker

```bash
# Inside Docker container or locally
./waf configure --board sitl
./waf copter

# Binary will be at build/sitl/bin/arducopter
```

## Troubleshooting

### Common Issues

1. **Connection refused to Gazebo**: Ensure Gazebo host firewall allows connections on ports 5501/9002
2. **No heartbeat**: Check `--sim-address` points to correct IP
3. **DDS not working**: Verify `DDS_ENABLE 1` in parameters and micro_ros_agent is running
4. **Permission denied**: Ensure volumes are mounted with correct permissions

### Verification Commands

```bash
# Inside container - check if SITL can reach Gazebo
ping <GAZEBO_HOST_IP>

# Check listening ports
netstat -tuln | grep -E '5501|5760|2019'

# Test MAVLink connection
mavproxy.py --master=tcp:127.0.0.1:5760
```

## Environment Variables

Set these in your Docker environment:

```bash
export BUILDLOGS=/tmp/buildlogs
export CCACHE_MAXSIZE=1G
export SIM_ADDRESS=<GAZEBO_HOST_IP>
```

## Files of Interest

- `/Dockerfile` - Main Docker configuration
- `Tools/autotest/sim_vehicle.py` - SITL launcher script
- `Tools/ros2/README.md` - ROS2 integration documentation
- `Tools/autotest/pysim/vehicleinfo.py` - Available models
- `Tools/ros2/ardupilot_sitl/` - ROS2 SITL package
- `Tools/autotest/default_params/` - Default parameter files

## Next Steps

1. Build the Docker image
2. Configure your Gazebo host IP
3. Set up parameter files for your specific use case
4. Start Gazebo on the remote Ubuntu machine
5. Launch ArduPilot SITL in Docker with `--sim-address` pointing to Gazebo host
6. Optionally enable DDS for ROS2 communication
