# AP_Scripting

## Enabling Scripting Support in Builds

Scripting is automatically enabled on all boards with more than 1MB of flash space.
The following example enables scripting, builds the ArduPlane firmware for the Cube, and uploads it.

```
$ waf configure --board=CubeBlack

$ waf plane

$ waf plane --upload
```

To run SITL you can simply use the `sim_vehicle.py` script which will wrap the configuration, compilation,
and launching of the simulation into one command for you.


```
$ Tools/autotest/sim_vehicle.py -v ArduPlane
```

Once you have a vehicle flashed with scripting you need to set the `SCR_ENABLE` parameter to 1 to enable scripting and reboot.

## Adding Scripts

The vehicle will automatically look for and launch any scripts that are contained in the `scripts` folder when it starts.
On real hardware this should be inside of the `APM` folder of the SD card. In SITL this should be in the working directory (typically the main `ardupilot` directory).

An example script is given below:

```lua
function update () -- periodic function that will be called
  local current_pos = ahrs:get_location() -- fetch the current position of the vehicle
  local home = ahrs:get_home()            -- fetch the home position of the vehicle
  if current_pos and home then            -- check that both a vehicle location, and home location are available
    local distance = current_pos:get_distance(home) -- calculate the distance from home in meters
    if distance > 1000 then -- if more then 1000 meters away
      distance = 1000;      -- clamp the distance to 1000 meters
    end
    SRV_Channels:set_output_pwm(96, 1000 + distance) -- set the servo assigned function 96 (scripting3) to a proportional value
  end

  return update, 1000 -- request "update" to be rerun again 1000 milliseconds (1 second) from now
end

return update, 1000   -- request "update" to be the first time 1000 milliseconds (1 second) after script is loaded
```

## Examples
See the [code examples folder](https://github.com/ArduPilot/ardupilot/tree/master/libraries/AP_Scripting/examples)

## Working with bindings

Edit bindings.desc and rebuild. The waf build will automatically
re-run the code generator.

## Lua Source Code

The Lua 5.3.6 source code is vendored in `lua/`. This is a customized
version of the [official
distribution](https://www.lua.org/ftp/lua-5.3.6.tar.gz). Where possible,
differences have been marked of the code.

Lua (not including modifications) is distributed under the terms of the
MIT license.
