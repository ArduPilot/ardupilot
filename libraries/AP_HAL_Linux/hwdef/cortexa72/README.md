# Cortex-A72 Linux Board Target

This board target provides a generic ArduPilot Linux build profile for Cortex-A72 systems.
For Versal hardware, prefer `libraries/AP_HAL_Linux/hwdef/versal/README.md` and build with `--board versal`.

## What It Sets

- Inherits `libraries/AP_HAL_Linux/hwdef/linux/hwdef.dat`
- Uses cross-toolchain: `aarch64-linux-gnu`
- Adds CPU tuning flags: `-mcpu=cortex-a72 -mtune=cortex-a72`

## Configure and Build

Initialize submodules (required for `DroneCAN`, `lwip`, `mavlink`, etc.):

```bash
git submodule update --init --recursive
```

### Option 1: Xilinx GNU Toolchain (Vitis install)

```bash
export PATH=/opt/Xilinx/2025.1/gnu/aarch64/lin/aarch64-linux/bin:$PATH
python3 ./waf configure --board cortexa72 --toolchain aarch64-linux-gnu
python3 ./waf copter -j"$(nproc)"
```

### Option 2: PetaLinux Environment Setup

```bash
source /opt/petalinux/2025.1/environment-setup-cortexa72-cortexa53-amd-linux
/usr/bin/python3 -m pip install --user empy==3.3.4
/usr/bin/python3 ./waf configure --board cortexa72 --toolchain aarch64-amd-linux
/usr/bin/python3 ./waf copter -j"$(nproc)"
```

Build artifacts are created under `build/cortexa72/bin/`.

## Test and Next Steps

Compile unit-test binaries for this target:

```bash
python3 ./waf tests -j"$(nproc)"
```

If you are in a PetaLinux shell, use host Python explicitly:

```bash
/usr/bin/python3 ./waf tests -j"$(nproc)"
```

For cross-builds, `tests` compiles the test binaries, but running them should be done on the Cortex-A72 target.

If `tests` fails on a specific unit test link step, continue with vehicle bring-up using `copter` (or `plane`/`rover`/`sub`) and come back to the failing test target separately.

After tests build, continue with bring-up:

1. Build your vehicle binary (`copter`, `plane`, `rover`, or `sub`).
2. Verify architecture:

```bash
file build/cortexa72/bin/arducopter
```

3. Deploy to target (example):

```bash
rsync -av build/cortexa72/bin/arducopter root@<target-ip>:/opt/ardupilot/bin/
```

4. On the target, run a smoke check:

```bash
/opt/ardupilot/bin/arducopter --help
```

5. Add board-specific sensor/GPIO/SPIDEV definitions in `hwdef.dat`, then rebuild and repeat.

## Versal PetaLinux Bring-Up

1. Create runtime directories on target:

```bash
ssh root@<target-ip> 'mkdir -p /opt/ardupilot/bin /opt/ardupilot/etc /var/lib/ardupilot/{logs,terrain,storage}'
```

2. Copy binary and defaults to target:

```bash
scp build/cortexa72/bin/arducopter root@<target-ip>:/opt/ardupilot/bin/
scp Tools/autotest/default_params/copter.parm root@<target-ip>:/opt/ardupilot/etc/
```

3. Validate binary on target:

```bash
ssh root@<target-ip> '/opt/ardupilot/bin/arducopter --help'
ssh root@<target-ip> 'file /opt/ardupilot/bin/arducopter'
```

4. Run ArduCopter (example serial + UDP telemetry):

```bash
ssh root@<target-ip> '\
/opt/ardupilot/bin/arducopter \
  --serial0 /dev/ttyPS1 \
  --serial1 udp:192.168.1.50:14550 \
  --log-directory /var/lib/ardupilot/logs \
  --terrain-directory /var/lib/ardupilot/terrain \
  --storage-directory /var/lib/ardupilot/storage \
  --defaults /opt/ardupilot/etc/copter.parm'
```

5. After manual run is stable, create a `systemd` service for auto-start.

## Notes

- This is a generic starting point. Add board-specific sensors, GPIO, SPI, and storage paths to `hwdef.dat` as hardware bring-up progresses.
- If building directly on an AArch64 Cortex-A72 host, you can override the toolchain:

```bash
./waf configure --board cortexa72 --toolchain native
```
