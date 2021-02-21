# SmartAudio Lua Script

Allows the setting of Smart Audio 2.0 video transmitters (VTX) via RC transmitter and to set an initial power level on system boot.

Four power levels can be set by the value of an RC channel: Pit Mode, Low, Medium, High, and Maximum. What these power levels actually produce is dependent on transmitter used. The new power level is activated upon the channel's changing to the new power selection.

The power level upon boot can be selected to be unchanged, or set to one of those above values by setting parameter SCR_USER1 to -1,0,1,2,3 or 4. Unchanged is -1. 0-4 correspond to the above levels.

The RC channel used to control the power level has its RCx_OPTION set to "300".

Ground Control Station messages will confirm boot value and any changes made later.

The VTX SmartAudio input is connected to any free autopilot UART TX line. That UART's corresponding SERIALx Port should have its SERIALx_PROTOCOL set to 28 and SERIALx_OPTIONS to 4 for half-duplex.

Pit Mode is set by the RC channel being a RCx_MIN for that channel, and MAXIMUM is set at RCx_MAX for the channel. The other levels are linearly spread in the interval. If MIN is 1000 and MAX is 2000, the mapping would be:

PWM        | Power Level
-----------|------------
1000 - 1124| Pit Mode |
1125 - 1374| LOW |
1375-1624| MED |
1625-1874| HIGH |
1875-2000| MAX |

Note: Some SmartAudio 2.0 VTX cannot be placed in PitMode remotely, but they can exit it and have power changed with this script. SmartAudio 2.1 VTX have the abiity to be remotely commanded into PitMode
