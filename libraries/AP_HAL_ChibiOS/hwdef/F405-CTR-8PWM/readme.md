This hardware definition for Mateksys F405-CTR board is based on the MatekF405 hw definition. 
This definition reassigns the pads labeled RSSI and TX2 to enable two more PWM outputs which gives a total of eight PWM outputs.
With eight outputs the F405-CTR can be used for VTOL setups.

Pads:

RSSI (PB1) - Ardupilot PWM7 (TIM3_CH4)
TX2  (PA2) - Ardupilot PWM8 (TIM2_CH3)
