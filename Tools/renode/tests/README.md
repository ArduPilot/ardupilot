# Renode mission test

Run the same CubeOrange SITL-on-hardware mission used by CI from the repository
root:

```sh
Tools/renode/tests/fetch_renode.sh
Tools/renode/tests/test_mission.py
```

The fetch helper downloads the latest package for the host architecture from
`https://firmware.ardupilot.org/Tools/Renode/`. It verifies the package size
and SHA-256 against `latest.json` before extracting it. Set
`RENODE_PACKAGE_BASE_URL` to use another package mirror, or set
`RENODE_SOURCE_REVISION` to require a specific 40-character source revision as
CI does.

`run.py` downloads the SVD matching the exact MCU in the compiled `hwdef.dat`
from `https://firmware.ardupilot.org/Tools/Renode/data/SVD/`, verifies its size
and SHA-256, and caches it under `~/.cache/ardupilot/renode/data/SVD/`. Use
`RENODE_DATA_CACHE` to choose another cache or `RENODE_DATA_BASE_URL` to use a
mirror.

The test builds Copter with `Tools/scripts/sitl-on-hardware/sitl-on-hw.py`,
uploads and flies a four-waypoint mission, lands, downloads the DataFlash log
over MAVLink, and validates the recorded flight. Test state, Renode output, and
the downloaded `flight.BIN` are retained under `build/renode-test/`.

During development, reuse an existing firmware build with `--skip-build`, or
pass APJ, BIN, HEX, or ELF firmware with `--firmware`. Supplying firmware skips
the board firmware build but still builds the physics sidecar unless
`--skip-build` is also used.

To boot a scenario and attach a ground station yourself, use `--interactive`:

```sh
Tools/renode/tests/test_physics_flight.py quadplane --interactive --skip-build
```

The command prints the MAVLink TCP endpoint, runs at real-time pace, and keeps
Renode and the physics sidecar alive until Ctrl-C. Add `--usb` to export the
emulated flight-controller USB device over USB/IP and automatically attach it
to the Linux VHCI controller. Perform the one-time host setup first with
`sudo Tools/renode/usbip_attach.py --install-rules`; attachment status is saved
in `usbip.log` in the test artifact directory. Add `--gdb` to open an xterm
running GDB; execution is halted at reset until `continue` is entered. GDB
requires ELF firmware because APJ, BIN, and HEX images do not contain the ELF
symbols needed by the debugger. When the harness builds firmware, `--gdb`
automatically adds `-g` to the waf configure command. With `--skip-build`, the
selected ELF must already contain debug symbols.
