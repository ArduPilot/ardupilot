# AP_Scripting

## Enabling Scripting Support in Builds

Scripting is automatically enabled on all boards with at least 1MB of flash space.
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
  current_pos = ahrs:get_position()
  home = ahrs:get_home()
  if current_pos and home then
    distance = current_pos:get_distance(ahrs:get_home()) -- calculate the distance from home
    if distance > 1000 then -- if more then 1000 meters away
      distance = 1000;      -- clamp the distance to 1000 meters
    end
    servo.set_output_pwm(96, 1000 + distance) -- set the servo assigned function 96 (scripting3) to a proportional value
  end

  return update, 1000 -- request to be rerun again 1000 milliseconds (1 second) from now
end

return update, 1000 -- request to be rerun again 1000 milliseconds (1 second) from now
```

## Working with bindings

Edit bindings.desc

cd into the generator subdirectory and type `make run`
