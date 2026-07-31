# SITL_Nexus

A SITL board that mirrors the inertial-sensor complement of the mRoNexus flight
controller.

Where the default `sitl` board uses the pure-maths `AP_InertialSensor_SITL`
backend, this board runs the real InvensenseV3 and ADIS IMU drivers against
their *simulated* SPI devices, so the actual driver code (register setup,
FIFO / burst reads, checksums) is exercised end to end under SITL.

The IMUs are declared with `IMU` lines in `hwdef.dat`; the SITL hwdef generator
turns these into the INS probe list, and the SPI device names resolve against
the SITL SPI device table in `libraries/AP_HAL_SITL/SPIDevice.cpp`.

Run it with the `nexus` ArduCopter frame:

```sh
sim_vehicle.py -v ArduCopter -f nexus
```

The `NexusIMUs` autotest brings the board up and checks that each simulated IMU
reports through the expected MAVLink telemetry.
