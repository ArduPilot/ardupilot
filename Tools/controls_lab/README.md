# Controls Lab

A framework for control system experiments on ArduPilot SITL.

## Installation

```bash
# Basic installation
pip install -e .

# With plotting support
pip install -e ".[plot]"

# With development tools
pip install -e ".[dev]"

# All optional dependencies
pip install -e ".[plot,gym,dev]"
```

## Requirements

- Python >= 3.9
- ArduPilot SITL running (for hardware-in-loop tests)

## Quick Start

```python
from controls_lab import ControlsLab, __version__

print(f"Controls Lab v{__version__}")

# Connect to SITL
lab = ControlsLab()
lab.connect(host="127.0.0.1", port="14550")

# Run a step response test
result = lab.run_step_response(target_alt=20.0, duration=10.0)

# View metrics
print(f"Rise time: {result.metrics.rise_time:.2f}s")
print(f"Overshoot: {result.metrics.overshoot:.1f}%")
print(f"Settling time: {result.metrics.settling_time:.2f}s")

# Plot results (requires matplotlib)
lab.plot(result)
```

## Running SITL

Before running experiments, start ArduPilot SITL:

```bash
cd ardupilot
./Tools/autotest/sim_vehicle.py -v ArduCopter -f quad --console --map
```

## Running Tests

```bash
pip install -e ".[dev]"
pytest tests/ -v
```

## License

GPL-3.0
