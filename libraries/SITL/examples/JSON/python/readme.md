# Velocity controlled robot

A JSON backend for vehicles that ArduPilot commands by velocity rather than by
joint angle. It needs nothing outside the Python standard library.

Walking robots that are sold as complete products keep their gait controller on
board and do not expose their joints. The autopilot asks for a forward speed and
a turn rate, and the on-board controller decides how to walk. Unitree's Go2 is
one such vehicle: its SDK takes a body velocity and walks. The same shape of
interface appears on tracked and differential platforms that take a velocity
command over a serial or network link. This example models that vehicle so
Rover's navigation, mission and failsafe code can be exercised without a physics
engine.

The model here is a plain unicycle with a first order lag. It is not a model of
any particular robot, and no hardware has been driven from it.

`../pybullet/walking_robot.py` covers the other case, where ArduPilot drives the
leg joints itself through twelve servo outputs.

## Running

```sh
python3 velocity_controlled_robot.py
```

then, in another terminal:

```sh
sim_vehicle.py -v Rover --model JSON:127.0.0.1 --add-param-file=default.parm --map --console
```

Arm, upload a mission and switch to AUTO. The vehicle drives it.

Options:

```text
--max-speed       forward speed at full throttle, default 1.5 m/s
--max-turn-rate   turn rate at full steering, default 1.2 rad/s
--tau             first order lag of the locomotion controller, default 0.2 s
--port            UDP port to listen on, default 9002
```

## Parameters

`default.parm` holds the matching Rover parameters. Two of them are easy to get
wrong:

* `ATC_STR_RAT_FF` is the steering fraction required for one radian per second,
  so it is `1 / max turn rate`. Leaving it at the default while the model turns
  at 1.2 rad/s under states the gain by a factor of four; the vehicle then yaws
  back and forth about the target heading and makes almost no forward progress.
* `CRUISE_SPEED` and `CRUISE_THROTTLE` must describe the same vehicle as
  `--max-speed`, otherwise the speed controller fights its own feed forward.

## Notes for anyone writing a similar backend

* The skid mixer in `AP_MotorsUGV` computes `motor_left = throttle + steering`
  and `motor_right = throttle - steering`, so a positive turn rate means the
  left motor runs faster. If this sign is inverted the vehicle settles pointing
  180 degrees away from the target and yaws forever, which is hard to recognise
  as a sign error.
* ArduPilot sends 0 on servo channels it is not driving. Scaling that as if it
  were a PWM value gives full reverse, so values outside a sane band are treated
  as neutral.
