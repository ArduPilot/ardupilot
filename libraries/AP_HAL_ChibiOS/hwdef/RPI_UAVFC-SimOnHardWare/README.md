# RPI_UAVFC Simulation-on-Hardware

Firmware for flashing into an [RPI_UAVFC](../RPI_UAVFC/README.md) to run
Simulation-on-Hardware: the flight code runs on the real board, at real loop
rates, against a physics model running on the same MCU instead of against the
real sensors. It is the same hardware, so the board documentation, flashing
procedure and pinout all apply unchanged.

What it is useful for on this port is timing. The scheduler, the EKF, the
dual-core handoff and the logging path all run on the real MCU at real rates,
so a change that costs core0 time shows up here without a propeller in the
room.

One difference to know before drawing conclusions from it: the fast rate
thread is **off**. `FSTRATE_ENABLE` compiles in as 0 and only
`RPI_UAVFC/defaults.parm` turns it on, which this board does not inherit (see
below), so nothing runs on core 1 at 2000 Hz here. Repeating the parent's
`FSTRATE_ENABLE 2` and `FSTRATE_DIV 2` would start it, but the sample source
is then the simulated IMU rather than the ICM42688P, so treat any rate-thread
number from this target as untested rather than as flight timing.

## Building and flashing

```bash
./waf configure --board RPI_UAVFC-SimOnHardWare
./waf copter
```

Flash it the same way as the normal firmware - see
[FLASHING.md](../RPI_UAVFC/FLASHING.md). Note the flash offset there is per
board; this target shares RPI_UAVFC's.

## What differs from the flight firmware

The hwdef is `RPI_UAVFC/hwdef.dat` plus `include/SimOnHW.inc`, so every pin,
clock and peripheral is inherited. The parameter defaults are not inherited:
a derived board's `defaults.parm` replaces the parent's wholesale rather than
merging, so anything RPI_UAVFC sets and this board still needs is repeated in
its own file. The ones that matter are the microSD log backend, the NeoPixel
function on servo 5, and the two BEC relay functions.

`SIM_RATE_HZ` and `SCHED_LOOP_RATE` are both 200 and want to stay equal: the
physics model steps once per main loop and `sync_frame_time()` sleeps to hold
`SIM_RATE_HZ`. Set `SIM_RATE_HZ` lower and the sleep throttles the loop below
its scheduled rate; set it higher and the loop keeps its rate but each
iteration advances less simulated time.

`AHRS_EKF_TYPE 10` selects the SITL AHRS, and `GPS1_TYPE 100` the simulated
GPS, so no real sensor is required - the board will run on the bench with
nothing attached but USB.
