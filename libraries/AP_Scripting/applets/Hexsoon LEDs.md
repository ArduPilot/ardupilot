# Hexsoon EDU 450 LED script
LEDs will be fixed colour when disarmed, when armed LEDs will also strobe

# INSTRUCTIONS
LEDs have two servo headers, plug into AUX 5 and 6 on cube.

Note white is ground, black and brown are signal

black into AUX 5 and brown into AUX 6

Other aux pins could be used but 5 and 6 make best use of the available groups
    BRD_PWM_COUNT must be 6

Aux 5 and 6 are servo outs 13 and 14
```
SERVO13_FUNCITON 132 (Profi LED Clock)
SERVO14_FUNCTION 94 (Script 1)
```

-setup scripting:
```
SCR_ENABLE 1
SCR_HEAP_SIZE 88032
```
- Reboot

- Use MP config tab -> MAVFtp to place this script in 'APM/scripts' folder

- Reboot

- Check messages tab should see:
```
LEDs strip left: chan=14
RCOut: PWM:1-12 ProfiLED:13-14
```

Note that 1-12 might not be PWM, all than matters is: ProfiLED:13-14

If not check for scripting error messages.

LEDs should now work!, if not try swapping AUX 5 and 6, either by physically swapping or by swapping the servo functions and rebooting

To get colours to match either change the ordering in "local led_map ="  below or swap headers round on the LED distribution board

If using 6 les add two extra colours to "local led_map =" e.g:  "local led_map = {red, red, red, green, green, green}"
