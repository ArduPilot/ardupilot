# `leds_on_a_switch.lua`: change LED brightness using an RC switch

ArduPilot controls LED brightness via the parameter `NTF_LED_BRIGHT`. This script allows you to control the value of the parameter using an RC switch.

Configure an `RCx_OPTION` as 300 for scripting and load this script.
The low position will turn off the LEDs, the high position with give maximum brightness and the middle position will dim the LEDs