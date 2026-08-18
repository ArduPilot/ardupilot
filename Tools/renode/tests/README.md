# Renode mission test

Run the same CubeOrange SITL-on-hardware mission used by CI from the repository
root:

```sh
Tools/renode/tests/build_renode.sh
Tools/renode/tests/test_mission.py --renode build/renode/renode
```

The test builds Copter with `Tools/scripts/sitl-on-hardware/sitl-on-hw.py`,
uploads and flies a four-waypoint mission, lands, downloads the DataFlash log
over MAVLink, and validates the recorded flight. Test state, Renode output, and
the downloaded `flight.BIN` are retained under `build/renode-test/`.

During development, reuse an existing firmware build with `--skip-build`, or
pass another ELF with `--firmware`.
