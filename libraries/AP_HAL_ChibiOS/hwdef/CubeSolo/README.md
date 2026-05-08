# CubeSolo variant of the FMUv3

The `CubeSolo` build is based on FMUv3, but includes a large set of default parameters required by the Solo.  Among many other things, this does include the slew rate limiting for protecting the Solo's ESCs.

For the parameter list used by this build, see [Tools/Fram_params/Solo_Copter-3.7_BlackCube.param](https://github.com/ArduPilot/ardupilot/blob/master/Tools/Frame_params/Solo_Copter-3.7_BlackCube.param)








<!-- TODO: add Features content -->
<!-- TODO: add Pinout content -->
<!-- TODO: add UART Mapping content -->
<!-- TODO: add RC Input content -->
<!-- TODO: add PWM Output content -->
<!-- TODO: add Battery Monitoring content -->

## Using this build in waf

- `./waf configure --board CubeSolo`
- `./waf copter`
- The completed firmware binary will be located in `/ardupilot/build/CubeSolo/bin/arducopter.apj`
