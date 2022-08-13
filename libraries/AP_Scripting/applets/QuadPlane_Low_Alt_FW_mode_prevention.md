# Quadplane low altitude forward flight mode prevention

This script prevents inadvertent switching into a fixed wing flight mode at low altitude. Its behaviour is controlled by Q_LOW_ALT_* parameters.

- Q_LOW_ALT_ENABLE: Enable script. 1: switch to Qland instead of fixed wing mode. 2: Switch back to previous mode instead of fixed wing mode.

- Q_LOW_ALT_ALT: Altitude threshold in meters above home. If vehicle is above this altitude the script will not take action

- Q_LOT_ALT_RADIUS: Radius threshold in meters from home. If vehicle is outside this radius the script wil not take action

Note: vehicle is always allowed to switch into auto mode or guided mode.
