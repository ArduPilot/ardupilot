# Versal-Sim Linux Board Target (Experimental)

This target enables `AP_SIM` in a Linux/Versal build so SITL model data can be used
inside a `HAL_BOARD_LINUX` binary.

Use this for early telemetry/software-loop integration. It is not yet a replacement
for standard host SITL workflows.

## What It Sets

- Inherits `libraries/AP_HAL_Linux/hwdef/versal/hwdef.dat`
- Enables simulation path with `env SIM_ENABLED 1`
- Keeps Versal runtime directories under `/var/lib/ardupilot/*`

## Configure and Build

PetaLinux shell example:

```bash
source /opt/petalinux/2025.1/environment-setup-cortexa72-cortexa53-amd-linux
/usr/bin/python3 ./waf configure --board versal-sim --toolchain aarch64-amd-linux
/usr/bin/python3 ./waf copter -j"$(nproc)"
```

Vitis GNU toolchain example:

```bash
export PATH=/opt/Xilinx/2025.1/gnu/aarch64/lin/aarch64-linux/bin:$PATH
python3 ./waf configure --board versal-sim --toolchain aarch64-linux-gnu
python3 ./waf copter -j"$(nproc)"
```

Artifacts are under `build/versal-sim/bin/`.

## Runtime Notes

- This profile is experimental and intended for staged bring-up.
- Keep using host SITL (`--board sitl`) for high-fidelity simulation and regression.
- Use `versal-sim` to validate telemetry transport and integration behavior on target.
- The default `versal-sim.param` is tuned for telemetry-first stability:
  - disables physical baro/compass probing
  - disables arming checks for bench bring-up
  - disables file logging (`LOG_BACKEND_TYPE=0`) to reduce storage I/O stalls

## sim_vehicle.py-Style Launch On Versal

Use the helper script for a single-command launch pattern similar to `sim_vehicle.py`:

```bash
Tools/scripts/versal_sim_vehicle.sh --mavout 192.168.1.10:14550
```

With optional serial console and in-binary debug instrumentation:

```bash
Tools/scripts/versal_sim_vehicle.sh \
  --mavout 192.168.1.10:14550 \
  --serial0 /dev/ttyAMA0 \
  --debug
```

When telemetry is stable, re-enable features incrementally:

```bash
# re-enable file logging
param set LOG_BACKEND_TYPE 1

# re-enable arming checks once required sensors are real/healthy
param set ARMING_CHECK 1

# re-enable compass once hardware is present
param set COMPASS_ENABLE 1
```
