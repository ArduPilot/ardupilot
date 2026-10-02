# SITL-on-HW notes

## Hardware Requirements

Running simulation on hardware (SimOnHW) executes both the full ArduPilot vehicle code and the 400Hz 6-DOF physics simulation engine directly on the microcontroller.

- **Recommended MCU:** STM32H7 series (e.g. STM32H743, STM32H753) with hardware double-precision FPU (FPv5-D16) and >= 1MB RAM/Flash (e.g., MatekH743, CubeOrange, NucleoH743).
- **Unsupported/Constrained MCUs:** STM32F4 / Cortex-M4 and STM32F7 / Cortex-M7 with single-precision FPUs are not recommended. They lack the computational power and double-precision hardware support required for the real-time simulation loops, causing CPU starvation, scheduling lag, and flight failure.

## Compiling and flashing

Run the `sitl-on-hw.py` script to compile and flash for MatekH743.  Adjust for your own board if required before running.  This script will configure a build ready for running SITL-on-hardware and attempt to upload it to a connected board.  It includes a set of embedded parameters to configure the simulated sensors appropriately.

::

    cd $HOME/ardupilot
    ./Tools/scripts/sitl-on-hardware/sitl-on-hw.py --board MatekH743 --vehicle copter

Plane can also be simulated:

::

    cd $HOME/ardupilot
    ./Tools/scripts/sitl-on-hardware/sitl-on-hw.py --board MatekH743 --vehicle plane

and quadplane:

::

    cd $HOME/ardupilot
    ./Tools/scripts/sitl-on-hardware/sitl-on-hw.py --board MatekH743 --vehicle plane --simclass QuadPlane

### Copter

Only the default quad frame is enable by default, to enable another frame type, you need to enable the right compile flag :
e.g. for octa-quad frame, AP_MOTORS_FRAME_OCTAQUAD_ENABLED 1 in the hwdef file. Compile flags list is in AP_Motors_class.h
Passing --frame parameter will enable the right compile flag for you.

## Configuring

Wipe the parameters on the board; this can be done with a mavlink command, or by setting the FORMAT_VERSION parameter to 0.

For example:

::

    STABILIZE> wipe_parameters IREALLYMEAANIT
    STABILIZE> Got COMMAND_ACK: PREFLIGHT_STORAGE: ACCEPTED
    AP: All parameters reset, reboot board
    reboot

You may need to power-cycle the board at this point.

::

    Device /dev/serial/by-id/usb-ArduPilot_MatekH743_3A0019001051393036353035-if00 reopened OK
    link 1 OK
    heartbeat OK
    disabling flow control on serial 2
    AP: Calibrating barometer
    AP: Barometer 1 calibration complete
    AP: Barometer 2 calibration complete
    Init Gyro**
    AP: ArduPilot Ready
    Suggested EK3_BCOEF_* = 16.288, EK3_MCOEF = 0.208
    Home: -35.36326 149.1652 alt=584.0000m hdg=353.0000
    Smoothing reset at 0.001
    AP: RCOut: PWM:1-13
    AP: GPS 1: detected as SITL at 115200 baud
    Time has wrapped
    Time has wrapped 5577 368458
    AP: EKF3 IMU0 initialised
    AP: EKF3 IMU1 initialised
    AP: EKF3 IMU0 tilt alignment complete
    AP: EKF3 IMU1 tilt alignment complete
    AP: EKF3 IMU1 MAG0 initial yaw alignment complete
    AP: EKF3 IMU0 MAG0 initial yaw alignment complete
    AP: PERF: 0/3999 [2653:2349] F=400Hz sd=39 Ex=0
    AP: EKF3 IMU1 forced reset
    AP: EKF3 IMU1 initialised
    AP: EKF3 IMU0 forced reset
    AP: EKF3 IMU0 initialised
    AP: EKF3 IMU1 tilt alignment complete
    AP: EKF3 IMU0 tilt alignment complete
    AP: EKF3 IMU1 MAG0 initial yaw alignment complete
    AP: EKF3 IMU0 MAG0 initial yaw alignment complete
    AP: PreArm: 3D Accel calibration needed
    AP: PERF: 0/4000 [2631:2369] F=400Hz sd=5 Ex=0
    AP: EKF3 IMU0 origin set
    AP: EKF3 IMU1 origin set
    AP: PERF: 0/4000 [2639:2362] F=400Hz sd=7 Ex=0

Force
