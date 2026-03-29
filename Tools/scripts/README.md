# SITL MAVLink Vision Bridge

A lightweight Python utility for sending simulated vision data to ArduPilot SITL using MAVLink.

This script acts as a mock vision sensor, allowing developers to test external navigation and precision landing workflows without requiring real hardware such as cameras, VIO systems, or optical flow sensors.

---

## Overview

ArduPilot supports external vision-based inputs via MAVLink messages such as:

* `VISION_POSITION_ESTIMATE` — for external pose estimation (e.g., VIO / SLAM)
* `LANDING_TARGET` — for precision landing using visual targets

This script generates and sends these messages to a running SITL instance, enabling rapid testing and development of vision-based features.

---

## Features

* Supports two modes:

  * **vision mode**: sends `VISION_POSITION_ESTIMATE` messages
  * **landing mode**: sends `LANDING_TARGET` messages
* Configurable message rate
* Compatible with SITL over TCP or UDP
* Includes demo trajectories (replaceable with real estimator output)
* Simple CLI interface
* Unit-tested core functions

---

## Use Cases

* Testing EKF integration with external vision (GPS-denied navigation)
* Prototyping visual odometry pipelines
* Developing precision landing systems
* Validating MAVLink-based companion computer workflows

---

## Requirements

* Python 3.8+
* pymavlink

Install dependencies:

```bash
pip install pymavlink
```

---

## Running with SITL

Start SITL without MAVProxy:

```bash
Tools/autotest/sim_vehicle.py -v ArduCopter --console --map --no-mavproxy
```

---

## Usage

### Vision Mode (External Navigation)

```bash
python3 sitl_mavlink_vision_bridge.py --mode vision --connection tcp:127.0.0.1:5760
```

### Landing Mode (Precision Landing)

```bash
python3 sitl_mavlink_vision_bridge.py --mode landing --connection tcp:127.0.0.1:5760
```

Alternative UDP connection:

```bash
python3 sitl_mavlink_vision_bridge.py --connection udpin:127.0.0.1:14550
```

---

## Parameters (ArduPilot)

### For Vision / External Navigation

Set the following parameters:

```bash
VISO_TYPE = 1
EK3_SRC1_POSXY = 6
```

Optional (depending on setup):

```bash
EK3_SRC1_POSZ
EK3_SRC1_YAW
```

---

### For Precision Landing

```bash
PLND_ENABLED = 1
PLND_TYPE = 1
```

Use **LAND mode** to test behavior.

---

## Demo Behavior

### Vision Mode

Simulates a circular trajectory:

* x = 2 * sin(0.3t)
* y = 2 * cos(0.3t)

This mimics a moving vehicle estimate and is intended for EKF testing.

---

### Landing Mode

Simulates a fixed landing target:

* Slightly forward and right of the vehicle
* Constant distance (8 meters)

Replace these values with real detector output for practical use.

---

## Covariance Handling

The script provides a 21-element upper-triangular covariance matrix.

* Position variance: `0.05 m²`
* Orientation variance: `0.01 rad²`
* Remaining values: `NaN` (unknown correlations)

ArduPilot uses this to determine trust in external measurements.

---

## How It Works

```text
[Python Script / CV Model]
        ↓
[MAVLink Messages]
        ↓
[ArduPilot SITL]
        ↓
[EKF / Precision Landing Logic]
```

The script mimics a companion computer sending vision data into the flight stack.

---

## Integration Notes

* Designed for SITL testing; no hardware required
* Replace demo trajectories with real outputs from:

  * SLAM / VIO systems
  * Object detection models
  * Pose estimation pipelines
* Compatible with companion computer setups (Jetson, Raspberry Pi, etc.)

---

## Testing

Unit tests are provided for:

* Argument parsing
* Covariance generation
* MAVLink message dispatch

Run tests:

```bash
python -m unittest test_sitl_mavlink_vision_bridge.py
```

---

## Limitations

* Does not include a real vision model
* No synchronization with camera timestamps
* Simplified covariance assumptions
* Does not validate EKF acceptance internally

---

## Future Improvements

* Integration with real CV pipelines (OpenCV, ROS2, etc.)
* Time synchronization support
* Configurable noise models
* Example notebooks for ML integration

---

## Summary

This tool enables rapid prototyping of vision-based autonomy features in ArduPilot by simulating MAVLink-compatible sensors in a controlled SITL environment.

It is intended as a bridge between computer vision systems and the ArduPilot flight stack.
