# Advance Waypoint

Advance Waypoint (`advance-wp.lua`) allows for advancing the current mission waypoint via an RC switch. When the RC switch state is high, the mission waypoint is advanced to the next waypoint (wraps back to WP 1 if the end of the mission is reached).

## How to Use

Install this script in the autopilot's SD card in the `APM/scripts` directory. Set SCR_ENABLE to 1 and reboot the autopilot.

* Set WAYPT_ADVANCE to an available aux function (300 by default).
* Set RCx_OPTION to the chosen aux function number. Preferably, set this to a channel assigned to a momentary switch.

Mission Planner's Aux Function tab can be used in lieu of dedicating an RC channel.

## Additional Features

If Yaapu telemetry (or similar) is in use on the RC transmitter, it may be useful to display bearing and distance to the currently selected waypoint in the messages view, regardless of flight mode or arming state. To enable this:

* Set WAYPT_ANNOUNCE to another available aux function (301 by default).
* Set RCx_OPTION to the chosen aux function.
* Set WAYPT_ANNOUNCE_S to desired interval (in seconds) between waypoint announcements (0 disables the feature).

As above, Mission Planner's Aux Function tab can be used in lieu of dedicating an RC channel.

When the announce switch is activated, the current waypoint index, bearing, and distance will be broadcast as a GCS message every WAYPT_ANNOUNCE_S seconds.

Additionally, there is an audio feedback feature:

* Set WAYPT_BUZ_ENABLE to 1 to enable buzzer feedback (0 to disable).

If WAYPT_BUZ_ENABLE is set, the buzzer will beep when the announce switch is activated, increasing in frequency and pitch as distance to the selected waypoint decreases (useful as a rangefinder for the selected waypoint if no telemetry source is readily available).

### Author's Note

I used this script to create a survey "prism pole" of sorts out of a spare autopilot, RTK GPS module/antenna, RC receiver, and telemetry radio. Using the existing SaveWP feature along with this script, I can save waypoints and subsequently relocate them precisely with the additional features above.