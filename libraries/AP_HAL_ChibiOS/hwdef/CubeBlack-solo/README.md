# CubeBlack-solo variant of the CubeBlack Flight Controller

The `CubeBlack-solo` build is identical to the CubeBlack build, but includes a large set of default parameters required by the Solo.

- For use in ArduCopter 3.7 and higher. Not compatible with any previous versions of ArduCopter or with other vehicle types.
- For data on the Hex CubeBlack flight controller, see the [Hex CubeBlack hwdef readme](https://github.com/ArduPilot/ardupilot/blob/master/libraries/AP_HAL_ChibiOS/hwdef/CubeBlack/README.md)
- For the parameter list used by this build, see [Tools/Fram_params/Solo_Copter-3.7_BlackCube.param](https://github.com/ArduPilot/ardupilot/blob/master/Tools/Frame_params/Solo_Copter-3.7_BlackCube.param)

### Using this build in waf

- `./waf configure --board CubeBlack-solo`
- `./waf copter`
- The completed firmware binary will be located in `/ardupilot/build/CubeBlack-solo/bin/arducopter.apj`
