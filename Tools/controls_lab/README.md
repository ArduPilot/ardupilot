# Controls Lab

A pip-installable Python framework for running control system experiments on ArduPilot SITL.

## Features

- **Step Response Testing** - Run standardized altitude step response tests
- **Automated Metrics** - Rise time, overshoot, settling time, steady-state error
- **Visualization** - Plot results with matplotlib
- **Simple API** - Clean `ControlsLab` interface
- **ArduCopter Support** - Plane/Rover/Sub planned for future

## Installation

```bash
cd Tools/controls_lab

# Basic installation
pip install -e .

# With plotting support
pip install -e ".[plot]"

# With development tools
pip install -e ".[dev]"
```

## Requirements

- Python >= 3.9
- ArduPilot SITL running

## Quick Start

```python
from controls_lab import ControlsLab

# Connect to SITL
lab = ControlsLab()
lab.connect()

# Run a step response test
results = lab.run_step_response(target_alt=20.0)

# View metrics
print(f"Rise time: {results.metrics.rise_time:.2f}s")
print(f"Overshoot: {results.metrics.overshoot:.1f}%")
print(f"Settling time: {results.metrics.settling_time:.2f}s")

# Plot results
lab.plot(results)
```

## Running SITL

Before running experiments, start ArduPilot SITL:

```bash
./Tools/autotest/sim_vehicle.py -v ArduCopter -f quad --console --map
```

## Example Output

<img width="1250" height="836" alt="Step Response Plot" src="https://github.com/user-attachments/assets/b37937dc-49c7-4b77-b70d-7cbefcd3a91f" />

## Running Tests

```bash
pip install -e ".[dev]"
python -m pytest tests/ -v
```

## Future Work

- Custom vehicle model loading (mass, inertia)
- Gymnasium environment wrapper for RL research
- Heuristic tuners (Ziegler-Nichols, etc.)
- Plane/Rover/Sub vehicle support

## License

GPL-3.0