# Forward flight motor shutdown script for tailsitters and tiltrotors

This allows to shutdown selected motors to be stopped once in forward flight for efficiency.

Set the motors to shutdown in with the `stop_motors` variable. Enable and disable the functionality with a RC switch with options 300 (Scripting1).

Motors will automatically be shutdown if forward throttle is lower than the value set in `throttle_off_threshold` (50% by default) the motors will then be re-enabled if the throttle goes above the value set in `throttle_on_threshold` (75% by default).

Time for stopped motors to go from throttle value to 0 and 0 back to throttle can be set with `slew_down_time` and `slew_up_time`.
