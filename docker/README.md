# ArduPilot SITL Docker Setup

This directory contains Docker configuration for running ArduPilot SITL with optional ROS2/DDS support and remote Gazebo connection.

## Quick Start

```bash
# 1. Copy environment template
cp .env.example .env

# 2. Edit configuration
nano .env

# 3. Build and run
docker-compose up --build

# Or run in background
docker-compose up --build -d
```

## Architecture

```
┌─────────────────────────┐      ┌─────────────────────────┐
│   Docker Container      │      │   Remote Ubuntu Host    │
│                         │      │                         │
│  ┌─────────────────┐   │      │  ┌─────────────────┐   │
│  │ ArduPilot SITL  │   │      │  │     Gazebo      │   │
│  │   (Headless)    │   │      │  │   Simulator     │   │
│  └────────┬────────┘   │      │  └────────┬────────┘   │
│           │            │      │           │            │
│  ┌────────▼────────┐   │      │  ┌────────▼────────┐   │
│  │   DDS Client    │───┼──UDP─┼──│  Micro ROS Agent│   │
│  └─────────────────┘   │      │  └────────┬────────┘   │
│                        │      │           │            │
└────────────────────────┘      │  ┌────────▼────────┐   │
                                │  │   ROS2 Nodes    │   │
                                │  └─────────────────┘   │
                                └─────────────────────────┘
```

## Configuration Files

### `.env.example`
Template for all configuration parameters. Copy to `.env` and modify:

```bash
cp .env.example .env
```

### `docker-compose.yml`
Main Docker Compose configuration that:
1. Builds ArduPilot from current source code
2. Generates parameter file from environment variables
3. Starts SITL with configured options

## Verified Parameters

All parameters are verified from ArduPilot source code:

### SITL Command-Line Options
Source: `Tools/autotest/sim_vehicle.py`

| Parameter | Description | Default | Source Line |
|-----------|-------------|---------|-------------|
| VEHICLE | Vehicle type (ArduCopter, ArduPlane, etc.) | ArduCopter | L1486 |
| MODEL | Simulation model | quad | L1303 |
| SPEEDUP | Simulation speed multiplier | 1 | L1313 |
| INSTANCE | SITL instance number | 0 | L1329 |
| SIM_ADDRESS | External simulator IP | 127.0.0.1 | L1368-1371 |
| ENABLE_DDS | Enable DDS client | 1 | L1372 |
| WIPE | Wipe EEPROM on start | 0 | L1270 |
| SYNTHETIC_CLOCK | Use synthetic clock | 0 | L1342 |
| SLAVE | Number of JSON slaves | 0 | L1360-1363 |
| SYSID | MAVLink system ID | - | L1346 |
| HOME_LOCATION | Start location | - | L1318 |

### DDS/ROS2 Parameters
Source: `libraries/AP_DDS/AP_DDS_Client.cpp` (lines 120-173)

| Parameter | Description | Default | Range |
|-----------|-------------|---------|-------|
| DDS_ENABLE | Enable DDS subsystem | 1 | 0-1 |
| DDS_UDP_PORT | UDP port for Micro ROS Agent | 2019 | 1-65535 |
| DDS_IP0 | Agent IP 1st octet | 127 | 0-255 |
| DDS_IP1 | Agent IP 2nd octet | 0 | 0-255 |
| DDS_IP2 | Agent IP 3rd octet | 0 | 0-255 |
| DDS_IP3 | Agent IP 4th octet | 1 | 0-255 |
| DDS_DOMAIN_ID | ROS_DOMAIN_ID | 0 | 0-232 |
| DDS_TIMEOUT_MS | Agent ping timeout (ms) | 1000 | 1-10000 |
| DDS_MAX_RETRY | Max ping retries | 10 | 0-100 |

Source: `libraries/AP_Networking/AP_Networking_address.cpp` (lines 13-40)

## Usage Examples

### 1. Basic Local SITL (No DDS)

```bash
# .env
VEHICLE=ArduCopter
MODEL=quad
ENABLE_DDS=0
MAVPROXY_ENABLED=1
SKIP_BUILD=0
```

```bash
docker-compose up --build
```

### 2. Remote Gazebo Connection

```bash
# .env
VEHICLE=ArduCopter
MODEL=gazebo-iris
SIM_ADDRESS=192.168.1.100  # Gazebo host IP
ENABLE_DDS=0
SKIP_BUILD=0
```

On Gazebo host:
```bash
# Start Gazebo with iris model
gz sim -r iris_runway.sdf
```

### 3. ROS2/DDS with Remote Agent

```bash
# .env
VEHICLE=ArduCopter
MODEL=quad
ENABLE_DDS=1
DDS_UDP_PORT=2019
DDS_IP0=192  # Agent host IP: 192.168.1.100
DDS_IP1=168
DDS_IP2=1
DDS_IP3=100
DDS_DOMAIN_ID=0
DDS_TIMEOUT_MS=2000
DDS_MAX_RETRY=0  # Unlimited retries
SKIP_BUILD=0
```

On ROS2 host (192.168.1.100):
```bash
source /opt/ros/humble/setup.bash
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019
```

### 4. Full Stack: Gazebo + ROS2

```bash
# .env
VEHICLE=ArduCopter
MODEL=gazebo-iris
SIM_ADDRESS=192.168.1.100    # Gazebo host
ENABLE_DDS=1
DDS_IP0=192                  # ROS2/Agent host
DDS_IP1=168
DDS_IP2=1
DDS_IP3=100
DDS_UDP_PORT=2019
SYNTHETIC_CLOCK=1
SPEEDUP=1
SKIP_BUILD=0
```

### 5. Pre-Built Binary (Skip Build)

```bash
# .env
SKIP_BUILD=1
VEHICLE=ArduCopter
MODEL=quad
```

### 6. Custom Build Options

```bash
# .env
BUILD_TARGET=sitl
WAF_CONFIGURE_OPTS=--debug --enable-math-check-indexes
BUILD_JOBS=8
SKIP_BUILD=0
```

### 7. Multiple Instances

```bash
# Instance 0 (default ports)
INSTANCE=0
# Ports: 5760, 5501, 2019

# Instance 1 (ports +10)
INSTANCE=1
# Ports: 5770, 5511, 2029
```

## Network Requirements

### Ports Used by SITL

| Port | Protocol | Purpose | Instance Offset |
|------|----------|---------|-----------------|
| 5760 | TCP | MAVLink telemetry | +10*instance |
| 5501 | UDP | SITL simulator data | +10*instance |
| 2019 | UDP | DDS/Micro ROS Agent | Uses DDS_UDP_PORT |

### Firewall Configuration

On remote hosts, ensure ports are open:

```bash
# On Gazebo host
sudo ufw allow 5501/udp
sudo ufw allow 9002/udp  # Gazebo default

# On ROS2/Agent host
sudo ufw allow 2019/udp
```

## Advanced Usage

### View Generated Parameters

The container generates a parameter file at runtime from environment variables. View it with:

```bash
docker-compose logs | grep -A 20 "Generated parameter file"
```

### Custom Parameter Files

Add custom parameters by mounting a file:

```bash
# .env
CUSTOM_DEFAULTS=/ardupilot/my_custom.parm
```

Then in docker-compose.yml volumes section:
```yaml
volumes:
  - ./my_custom.parm:/ardupilot/my_custom.parm
```

### Persisted EEPROM

The docker-compose.yml includes a volume for EEPROM persistence:

```yaml
volumes:
  sitl-eeprom:
    driver: local
```

This preserves parameters between container restarts.

### Rebuild from Scratch

```bash
# Remove build cache
docker-compose down
docker system prune -f
docker-compose build --no-cache

# Or clean ArduPilot build
docker-compose run --rm ardupilot-sitl bash -c "cd /ardupilot && ./waf distclean"
```

## Troubleshooting

### Build Failures

```bash
# Check build logs
docker-compose logs --tail=100

# Enter container for debugging
docker-compose run --rm ardupilot-sitl bash
```

### DDS Connection Issues

1. Verify agent is running:
```bash
# On ROS2 host
ros2 run micro_ros_agent micro_ros_agent udp4 -p 2019
```

2. Check connectivity:
```bash
# In container
ping <DDS_IP>
```

3. Verify parameters:
```bash
# Check generated parameter file
docker-compose logs | grep DDS
```

### Gazebo Connection Issues

1. Ensure Gazebo is running and listening
2. Check firewall rules
3. Verify SIM_ADDRESS is correct
4. Check Gazebo logs for connection attempts

### Container Won't Start

```bash
# Check for errors
docker-compose logs

# Validate compose file
docker-compose config

# Rebuild
docker-compose build --no-cache
```

## Files Reference

```
ardupilot/
├── Dockerfile                 # Base Docker image
├── docker-compose.yml         # Service configuration
├── .env.example              # Template environment file
├── .env                      # Your configuration (create this)
├── docker/
│   └── README.md             # This file
└── Claude.md                 # Additional documentation
```

## Related Documentation

- [ArduPilot ROS2 Packages](../Tools/ros2/README.md)
- [AP_DDS Library](../libraries/AP_DDS/README.md)
- [SITL Simulation](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html)
- [Building for SITL](../BUILD.md)
