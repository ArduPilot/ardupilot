This hardware definition for Mateksys F405-CTR board is based on the MatekF405 hw definition. 
This definition reassigns the pads labeled RSSI and TX2 to enable two more PWM outputs which gives a total of eight PWM outputs.

Pads:
S1         - Ardupilot PWM1  TIMER3
S2         - Ardupilot PWM2  TIMER8
S3         - Ardupilot PWM3  TIMER8
S4         - Ardupilot PWM4  TIMER8
S5         - Ardupilot PWM5  TIMER2
S6         - Ardupilot PWM6  TIMER1
RSSI (PB1) - Ardupilot PWM7  TIMER3_CH4
TX2  (PA2) - Ardupilot PWM8  TIMER2_CH3


Motor ESC:s can’t share same TIMER with servo.

DSHOT capable.
