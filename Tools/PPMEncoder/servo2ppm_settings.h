
/*********************************************************************************************************
 Title  :   header file for the rc ppm encoder (servo2ppm_settings.h)
 Author:    Chris Efstathiou 
 E-mail:    hendrix at vivodinet dot gr
 Homepage:  ........................
 Date:      03/Aug/2009
 Compiler:  AVR-GCC with AVR-AS
 MCU type:  ATmega168 
 Comments:  This software is FREE. Use it at your own risk.
*********************************************************************************************************/

#ifndef SERVO2PPM_SETTINGS_H
#define SERVO2PPM_SETTINGS_H

/********************************************************************************************************/
/*                                  USER CONFIGURATION BLOCK START                                      */ 
/********************************************************************************************************/
/* 
The cpu frequency is defined in the makefile, the below definition is used only if the cpu frequency
is not defined in the makefile.
*/

#ifndef F_CPU
#define F_CPU                         8000000UL        /* CPU CLOCK FREQUENCY */
#endif
/*
Those values are the failsafe servo values and the values for the non used channels
If for example you leave unconnected channel 7, the servo pulse of channel 7 in the PPM train
will be the failsafe value set below as "RC_FAILSAFE_CHANNEL_7" thus 1000 microseconds.
*/ 

#define RC_FAILSAFE_CHANNEL_1         1500UL
#define RC_FAILSAFE_CHANNEL_2         1500UL
#define RC_FAILSAFE_CHANNEL_3         1000UL 
#define RC_FAILSAFE_CHANNEL_4         1500UL
#define RC_FAILSAFE_CHANNEL_5         1000UL
#define RC_FAILSAFE_CHANNEL_6         1000UL
#define RC_FAILSAFE_CHANNEL_7         1000UL 
#define RC_FAILSAFE_CHANNEL_8         1000UL 


/*
When signal is lost 1= ppm waveform remain on with failsafe values, 0= ppm waveform is off 
For use with Paparazzi use "0", the autopilot has it's own failsafe values when PPM signal is lost.
The above failsafe values will kick in only if the servo inputs are lost which cannot happen with
a receiver with dsp processing that provides "hold" or "failsafe" features.
If the receiver only provides "hold" on the last good servo signals received  it is not suitable
for use with the Paparazzi autopilot as it will prohibit the autopilot entering the "AUTO2" or "HOME" mode.
If you use a receiver with failsafe ability then remember to set the failsafe value of the receiver
to 2000 microseconds for the "MODE" channel so the autopilot can go to "AUTO2" or "HOME" mode when the receiver
loose the tx signal. The servo signals on receivers like PCM, IPD and any receiver with servo hold AND failsafe
will not stop outputing servo pulses thus the encoder will never stop producing a PPM pulse train
except if the throttle channel is used as an indication by setting the "RC_LOST_CHANNEL" to a value above 0.
If you use the throttle channel as an indication that the TX signal is lost then:
RC_USE_FAILSAFE set to 0 means that the ppm output will be shut down and if you set
RC_USE_FAILSAFE  to 1 the ppm output will NOT shut down but it will now output the failsafe values
defined above. 
*/
#define RC_USE_FAILSAFE               1

/*
The channel number (1,2,3...7,8) that will be used as a receiver ready indicator.
If set above 0 then this channel should be always connected otherwise
the PPM output will not be enabled as the ppm encoder will wait for ever for this channel
to produce a valid servo pulse. 
If 0 then the first connected channel going from channel 1 to 8 will be used, in other words
which ever connected channel comes on first it will indicate that the receiver is operational.
The detection of all connected channels is independent of this setting, this channel
is used as an indication that the receiver is up and running only. 
Valid only if greater than 0. 
*/
#define RC_RX_READY_CHANNEL           0     /* 0 = No channel will be exclusively checked. */

/* 
The default value for the rc signal lost indicator channel and it's threshold value in microseconds .
If the value is above 1500 microseconds then when the signal lost indicator channel servo pulse exceeds
RC_LOST_THRESHOLD then the ppm output will be shut down.
If the value is below 1500 microseconds then when the signal lost indicator channel servo pulse gets lower than
RC_LOST_THRESHOLD then the ppm output will be shut down.
If the RC_USE_FAILSAFE is set to 1 then the ppm output will not stop producing pulses but now
the ppm wavetrain will contain the failsafe values defined in the beginning of this file. 
Valid only if RC_LOST_CHANNEL > 0
*/
#define RC_LOST_CHANNEL               3      /* Defaul is the throttle channel. You can use any channel. */
#define RC_LOST_THRESHOLD             2025   /* Any value below 1300 or above 1700 microseconds. */


#define RC_PPM_GEN_CHANNELS           8      /* How many channels the PPM output frame will have. */

#define RC_PPM_OUTPUT_TYPE            0      /* 1 = POSITIVE PULSE, 0= NEGATIVE PULSE */


#define RC_MUX_CHANNEL                8     /*Jordi: Channel that will control the MUX */
#define RC_MUX_REVERSE                0     /*Jordi: Inverted the MUX output (NOT), 0 = normal, 1 = Rev*/
#define RC_MUX_MIN                    0  /*Jordi: the min point */
#define RC_MUX_MAX                    1250  /*Jordi: the max point */

/*
0 = the reset period is constant but the total PPM frame period is varying.
1 = the reset period is varying but the total PPM frame period is constant (always 23,5 ms for example).
*/
#define RC_CONSTANT_PPM_FRAME_TIME    0


/********************************************************************************************************/
/* 
                                      D A N G E R
The below settings are good for almost every situation and probably you don't need to change them.
Be carefull because you can cause a lot of problems if you change anything without knowing
exactly what you are doing.
*/
#define RC_SERVO_MIN_PW               800UL   /* in microseconds */
#define RC_SERVO_CENTER_PW            1500UL  /* in microseconds */
#define RC_SERVO_MAX_PW               2200UL  /* in microseconds */
#define RC_PPM_CHANNEL_SYNC_PW        300UL   /* the width of the PPM channel sync pulse in microseconds. */
#define RC_PPM_RESET_PW               0UL     /* Min reset time in microseconds(ie. 7500UL), 0=AUTO */
#define RC_PPM_FRAME_LENGTH_MS        0UL     /* Max PPM frame period in microseconds(ie. 23500UL), 0=AUTO */
#define RC_MAX_TIMEOUT                30000UL /* in microseconds, max ~65000 @ 8 mHZ, ~32000 @ 16 Mhz */
#define RC_MAX_BAD_PPM_FRAMES         4       /* Max consecutive bad servo pulse samples before failsafe */
#define RC_LED_FREQUENCY              5UL     /* In Hertz, not accurate, min=1, max=you will get a warning */
#define RC_THROTTLE_CH_OFFSET_PW      25      /* in microseconds. */

/********************************************************************************************************/
/*                                  USER CONFIGURATION BLOCK END                                        */ 
/********************************************************************************************************/

#endif //#ifndef SERVO2PPM_SETTINGS_H
