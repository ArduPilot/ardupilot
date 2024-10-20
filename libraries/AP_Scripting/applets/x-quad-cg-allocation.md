# Multicopter CoM compensation

This script allows for adjusting the control allocation matrix.

When the Center of Mass (CoM) of an airframe does not coincide with the center
of its thrusters, then there is a lever arm between the thrust vector and the
CoM. This often is the case in VTOL fixed-wing aircraft (quadplanes) where
typically the CoM is more forward than the center of thrust. As a result, the
thrust produces a pitch-down moment. This produces a disturbance in the pitch
control and requires significant wind-up in the pitch integrator.

To compensate for this issue, this script employs the scriptable control
allocation matrix to request asymmeterical front and back thrust.

WARNING: This script is applicable only to X-type quadrotors and quadplanes. Do
not use in any other frame configuration!

# Parameters

The script adds 1 parameter to control its behaviour.

## CGA_RATIO

This is the desired ratio between the front and back thrust. To have the front
motors produce more lift that the rear, increase higher than 1.

Reasonable extreme values are 2 (front works twice as hard as the rear) and 0.5
(the inverse case). Given an out-of-bounds parameter value, the script will
revert to the default 1.0. 

# Operation

## How To Use

First of all, place this script in the "scripts" directory.

To tune `CGA_RATIO` on the fly:

  1. Set `FRAME_CLASS` or `Q_FRAME_CLASS` (for quadplanes) to 17 to enable the
  dynamic scriptable mixer.
  2. Enable Lua scripting via the `SCR_ENABLE` parameter.
  3. Reboot.
  4. Fly the vehicle.
  5. Adjust the value of the `CGA_RATIO` parameter. A good indicator of a good
  tune is to monitor the telemetry value `PID_TUNE[2].I` (pitch rate controller
  integrator) until it reaches zero during a stable hover.

Once you are happy with the tuning, you can fall back to the static motor
matrix, which consumes no resources from the scripting engine:

  1. Set `FRAME_CLASS` or `Q_FRAME_CLASS` (for quadplanes) to 15 to enable the
  static scriptable mixer.
  2. Ensure Lua scripting is enabled via the `SCR_ENABLE` parameter.
  3. Reboot.
  
The aircraft is ready to fly.
Keep in mind that any further changes to `CGA_RATIO` will now require a reboot.

## How It Works

  1. The dynamic control allocation matrix is able to change the coefficients
  that convert the throttle command to individual PWM commands for every motor.
  These coefficients have a default value of 1.
  2. The parameter `CGA_RATIO` is used to alter these coefficients, so that the
  front and back thrust commands are not equal.