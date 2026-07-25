# Actuator Chirp

This script drives a single servo or ESC output with a frequency sweep
(a "chirp") while logging the response of the actuator. It is used to
measure the frequency response (bandwidth) of a servo or ESC on the
bench.

The sweep is an exponential chirp from a start frequency to a stop
frequency, with a short constant frequency dwell at the start and
smooth fade-in and fade-out of the amplitude to avoid step
transients. While running, the script sends the drive PWM, the
instantaneous chirp frequency and the actuator feedback (RPM for an
ESC, position for a servo) as NAMED_VALUE_FLOAT MAVLink messages for
live viewing, and writes them to the dataflash log for offline
analysis.

Note that the script does not stop the chirp when the vehicle is
armed. The output channel parameter ACHRP_CHAN is reset to zero on
boot, so a chirp can never start automatically, but you should only
use this tool on the bench with propellers removed or linkages
disconnected as appropriate.

## Parameters

The script adds the following parameters to control its behaviour.

## ACHRP_CHAN

The 1-based servo/ESC output channel to drive with the chirp. Setting
this to a channel number starts the chirp, and setting it to zero
stops it. It is reset to zero on boot, and automatically set back to
zero when the chirp completes.

## ACHRP_PWM_MIN

The lower PWM bound of the chirp waveform. Together with
ACHRP_PWM_MAX this sets the center point and amplitude of the sweep.

## ACHRP_PWM_MAX

The upper PWM bound of the chirp waveform.

## ACHRP_F_START

The chirp start frequency in Hz. The script dwells at this frequency
for a short time before the sweep begins.

## ACHRP_F_STOP

The chirp stop frequency in Hz. Must be greater than or equal to the
start frequency.

## ACHRP_F_FADE_IN

The time in seconds to ramp the chirp amplitude from zero to full at
the start of the sweep.

## ACHRP_F_FADE_OUT

The time in seconds to ramp the chirp amplitude back to zero at the
end of the sweep.

## ACHRP_TIME

The total sweep time in seconds, including the initial dwell and the
fade-in and fade-out periods.

## ACHRP_TYPE

The type of actuator being tested. Set to 0 for an ESC (feedback is
RPM from ESC telemetry) or 1 for a servo (feedback is measured
position from servo telemetry, for example from DroneCAN servos).

## Operation

Install the lua script in the APM/SCRIPTS directory on the flight
controller microSD card, then set SCR_ENABLE to 1 and reboot. You
should see "ActuatorChirp loaded" in the ground station messages.

Prepare the bench setup. For ESC tests make sure ESC telemetry is
working so RPM feedback is available for the target channel. For
servo tests the servo needs to provide measured position via servo
telemetry (for example DroneCAN servos). Note that many DroneCAN ESCs
will only run the motor when armed, so either arm for the test or
disable that requirement in the ESC settings.

Set ACHRP_PWM_MIN and ACHRP_PWM_MAX to bounds that are safe for the
device under test, choose the sweep with ACHRP_F_START, ACHRP_F_STOP
and ACHRP_TIME, and set ACHRP_TYPE for the actuator type.

Then set ACHRP_CHAN to the 1-based output channel number to start the
chirp. You can watch the FREQ, PWM and RPM (or POS) named float
values on the ground station while it runs. When the sweep completes
the script prints "Chirp finished" and sets ACHRP_CHAN back to zero.
You can stop early at any time by setting ACHRP_CHAN to zero.

## Data analysis

The response is logged in the ECRP log message for ESCs (PWM, Freq,
RPM) or the SCRP log message for servos (PWM, Freq, Pos). The Freq
field is the instantaneous excitation frequency in Hz. Plotting the
feedback against Freq gives an estimate of the magnitude response.
Keep the amplitude modest (a small PWM_MIN to PWM_MAX range) to stay
within the linear range of the actuator.
