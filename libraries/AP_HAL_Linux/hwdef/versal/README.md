# Versal Linux Board Target

This board target is the recommended profile for Versal hardware running PetaLinux.
It inherits `cortexa72` and keeps the same toolchain tuning while naming the board after the actual platform.

## What It Sets

- Inherits `libraries/AP_HAL_Linux/hwdef/cortexa72/hwdef.dat`
- Uses cross-toolchain: `aarch64-linux-gnu` (or `aarch64-amd-linux` in PetaLinux environment setup)
- Adds CPU tuning flags: `-mcpu=cortex-a72 -mtune=cortex-a72`
- Uses runtime directories:
  - Logs: `/var/lib/ardupilot/logs`
  - Terrain: `/var/lib/ardupilot/terrain`
  - Storage: `/var/lib/ardupilot/storage`

## Configure and Build

Initialize submodules (required for `DroneCAN`, `lwip`, `mavlink`, etc.):

```bash
git submodule update --init --recursive
```

### Option 1: Xilinx GNU Toolchain (Vitis install)

```bash
export PATH=/opt/Xilinx/2025.1/gnu/aarch64/lin/aarch64-linux/bin:$PATH
python3 ./waf configure --board versal --toolchain aarch64-linux-gnu
python3 ./waf copter -j"$(nproc)"
```

### Option 2: PetaLinux Environment Setup

```bash
source /opt/petalinux/2025.1/environment-setup-cortexa72-cortexa53-amd-linux
/usr/bin/python3 -m pip install --user empy==3.3.4
/usr/bin/python3 ./waf configure --board versal --toolchain aarch64-amd-linux
/usr/bin/python3 ./waf copter -j"$(nproc)"
```

Build artifacts are created under `build/versal/bin/`.

## Tests

Compile unit-test binaries:

```bash
python3 ./waf tests -j"$(nproc)"
```

If in a PetaLinux shell, use host Python explicitly:

```bash
/usr/bin/python3 ./waf tests -j"$(nproc)"
```

For cross-builds, this compiles test binaries; execute them on the Versal target.

## Deploy to Versal

1. Create runtime directories:

```bash
ssh root@<target-ip> 'mkdir -p /opt/ardupilot/bin /opt/ardupilot/etc /var/lib/ardupilot/{logs,terrain,storage}'
```

2. Copy binary and defaults:

```bash
scp build/versal/bin/arducopter root@<target-ip>:/opt/ardupilot/bin/
scp Tools/autotest/default_params/copter.parm root@<target-ip>:/opt/ardupilot/etc/
```

3. Validate binary:

```bash
ssh root@<target-ip> '/opt/ardupilot/bin/arducopter --help'
ssh root@<target-ip> 'file /opt/ardupilot/bin/arducopter'
```

4. Run ArduCopter:

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

5. After manual run is stable, create a `systemd` service.
