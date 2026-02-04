#!/usr/bin/env python3
"""
Basic step response example.

Prerequisites:
    1. Start SITL: sim_vehicle.py -v ArduCopter --console --map
    2. Wait for GPS lock and EKF to initialize
    3. Run this script from the parent of controls_lab

Usage:
    cd Tools
    python -m controls_lab.examples.basic_step_response

AP_FLAKE8_CLEAN
"""
import logging

from controls_lab import ControlsLab

logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)


def main():
    lab = ControlsLab()

    lab.connect(host="127.0.0.1", port="14550")

    results = lab.run_step_response(
        target_alt=20.0,
        initial_alt=10.0,
        duration=15.0,
        sample_rate=10.0
    )

    print("\nResults:")
    if results.metrics.rise_time:
        print(f"  Rise Time:     {results.metrics.rise_time:.2f}s")
    else:
        print("  Rise Time:     N/A")
    print(f"  Overshoot:     {results.metrics.overshoot:.1f}%")
    if results.metrics.settling_time:
        print(f"  Settling Time: {results.metrics.settling_time:.2f}s")
    else:
        print("  Settling Time: N/A")
    print(f"  SS Error:      {results.metrics.steady_state_error:.3f}m")

    lab.plot(results)

    lab.vehicle.land()
    lab.disconnect()


if __name__ == "__main__":
    main()
