/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// updates the status of notify
// should be called at 50hz
static void update_notify()
{
    notify.update();
}

/////////////////////////////////////////////////////////////////////////////////////////////
//	Copter LEDS by Robert Lefebvre
//	Based on the work of U4eake, Bill Sanford, Max Levine, and Oliver
//	g.copter_leds_mode controls the copter leds function via bitmath
//	Zeroeth bit turns motor leds on and off:                                00000001
//	First bit turns GPS function on and off:                                00000010
//	Second bit turns Aux function on and off:                               00000100
//	Third bit turns on Beeper (legacy Piezo) function:                      00001000
//	Fourth bit toggles between Fast Flash or Oscillate on Low Battery:      00010000		(0) does Fast Flash, (1) does Oscillate
//	Fifth bit causes motor LEDs to Nav Blink:                               00100000
//	Sixth bit causes GPS LEDs to Nav Blink:                                 01000000
//	This code is written in order to be backwards compatible with the old Motor_LEDS code
//	I hope to include at least some of the Show_LEDS code in the future
//	copter_leds_GPS_blink controls the blinking of the GPS LEDS
//	copter_leds_motor_blink controls the blinking of the motor LEDS
//	Piezo Code and beeps once on Startup to verify operation
//	Piezo Enables Tone on reaching low battery or current alert
/////////////////////////////////////////////////////////////////////////////////////////////

#define COPTER_LEDS_BITMASK_ENABLED         0x01        // bit #0
#define COPTER_LEDS_BITMASK_GPS             0x02        // bit #1
#define COPTER_LEDS_BITMASK_AUX             0x04        // bit #2
#define COPTER_LEDS_BITMASK_BEEPER          0x08        // bit #3
#define COPTER_LEDS_BITMASK_BATT_OSCILLATE  0x10        // bit #4
#define COPTER_LEDS_BITMASK_MOTOR_NAV_BLINK 0x20        // bit #5
#define COPTER_LEDS_BITMASK_GPS_NAV_BLINK   0x40        // bit #6

#if COPTER_LEDS == ENABLED
static void copter_leds_init(void)
{
    pinMode(COPTER_LED_1, OUTPUT);              //Motor LED
    pinMode(COPTER_LED_2, OUTPUT);              //Motor LED
    pinMode(COPTER_LED_3, OUTPUT);              //Motor LED
    pinMode(COPTER_LED_4, OUTPUT);              //Motor LED
    pinMode(COPTER_LED_5, OUTPUT);              //Motor or Aux LED
    pinMode(COPTER_LED_6, OUTPUT);              //Motor or Aux LED
    pinMode(COPTER_LED_7, OUTPUT);              //Motor or GPS LED
    pinMode(COPTER_LED_8, OUTPUT);              //Motor or GPS LED

    if (!(g.copter_leds_mode & COPTER_LEDS_BITMASK_BEEPER)) {
        piezo_beep();
    }
}

static void update_copter_leds(void)
{
    if (g.copter_leds_mode == 0) {
        copter_leds_reset();                                        //method of reintializing LED state
    }

    // motor leds control
    if (g.copter_leds_mode & COPTER_LEDS_BITMASK_ENABLED) {
        if (motors.armed()) {
            if (failsafe.battery) {
                if (g.copter_leds_mode & COPTER_LEDS_BITMASK_BATT_OSCILLATE) {
                    copter_leds_oscillate();                        //if motors are armed, but battery level is low, motor leds fast blink
                } else {
                    copter_leds_fast_blink();                       //if motors are armed, but battery level is low, motor leds oscillate
                }
            } else {
                if (g.copter_leds_mode & COPTER_LEDS_BITMASK_MOTOR_NAV_BLINK) {
                    if (copter_leds_nav_blink > 0) {
                        copter_leds_slow_blink();                   //if nav command was seen, blink LEDs.
                    } else {
                        copter_leds_on();
                    }
                } else {
                    copter_leds_on();                               //if motors are armed, battery level OK, all motor leds ON
                }
            }
        } else {
            copter_leds_slow_blink();                               //if motors are not armed, blink motor leds
        }
    }

    // GPS led control
    if (g.copter_leds_mode & COPTER_LEDS_BITMASK_GPS) {

        // GPS LED on if we have a fix or Blink GPS LED if we are receiving data
        // ---------------------------------------------------------------------
        switch (g_gps->status()) {

        case GPS::NO_GPS:
            copter_leds_GPS_off();                                  //if no valid GPS signal, turn GPS LED off
            break;

        case GPS::NO_FIX:
            copter_leds_GPS_slow_blink();                           //if GPS has valid reads, but no fix, blink GPS LED slow
            break;

        case GPS::GPS_OK_FIX_2D:
        case GPS::GPS_OK_FIX_3D:
            if(ap.home_is_set) {
                if (g.copter_leds_mode & COPTER_LEDS_BITMASK_GPS_NAV_BLINK) {
                    if (copter_leds_nav_blink >0) {
                        copter_leds_GPS_slow_blink();               //if nav command was seen, blink LEDs.
                    } else {
                        copter_leds_GPS_on();
                    }
                } else {
                    copter_leds_GPS_on();							//Turn GPS LEDs on when gps has valid fix AND home is set
                }
            } else {
                copter_leds_GPS_fast_blink();                       //if GPS has fix, but home is not set, blink GPS LED fast
            }
            break;
        }
    }

    // AUX led control
    if (g.copter_leds_mode & COPTER_LEDS_BITMASK_AUX) {
        if (ap.CH7_flag) {
            copter_leds_aux_on();                                   //if sub-control of Ch7 is high, turn Aux LED on
        } else {
            copter_leds_aux_off();                                  //if sub-control of Ch7 is low, turn Aux LED off
        }
    }
}

static void copter_leds_reset(void) {
    digitalWrite(COPTER_LED_1, COPTER_LED_OFF);
    digitalWrite(COPTER_LED_2, COPTER_LED_OFF);
    digitalWrite(COPTER_LED_3, COPTER_LED_OFF);
    digitalWrite(COPTER_LED_4, COPTER_LED_OFF);
    digitalWrite(COPTER_LED_5, COPTER_LED_OFF);
    digitalWrite(COPTER_LED_6, COPTER_LED_OFF);
    digitalWrite(COPTER_LED_7, COPTER_LED_OFF);
    digitalWrite(COPTER_LED_8, COPTER_LED_OFF);
}

static void copter_leds_on(void) {
    if (!(g.copter_leds_mode & COPTER_LEDS_BITMASK_AUX)) {
        digitalWrite(COPTER_LED_1, COPTER_LED_ON);
    }
 #if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    if (!(g.copter_leds_mode & COPTER_LEDS_BITMASK_BEEPER)) {
        digitalWrite(COPTER_LED_2, COPTER_LED_ON);
    }
 #else
    digitalWrite(COPTER_LED_2, COPTER_LED_ON);
 #endif
    if (!(g.copter_leds_mode & COPTER_LEDS_BITMASK_GPS) ) {
        digitalWrite(COPTER_LED_3, COPTER_LED_ON);
    }
    digitalWrite(COPTER_LED_4, COPTER_LED_ON);
    digitalWrite(COPTER_LED_5, COPTER_LED_ON);
    digitalWrite(COPTER_LED_6, COPTER_LED_ON);
    digitalWrite(COPTER_LED_7, COPTER_LED_ON);
    digitalWrite(COPTER_LED_8, COPTER_LED_ON);
}

static void copter_leds_off(void) {
    if (!(g.copter_leds_mode & COPTER_LEDS_BITMASK_AUX)) {
        digitalWrite(COPTER_LED_1, COPTER_LED_OFF);
    }
 #if CONFIG_HAL_BOARD == HAL_BOARD_APM2
    if (!(g.copter_leds_mode & COPTER_LEDS_BITMASK_BEEPER)) {
        digitalWrite(COPTER_LED_2, COPTER_LED_OFF);
    }
 #else
    digitalWrite(COPTER_LED_2, COPTER_LED_OFF);
 #endif
    if (!(g.copter_leds_mode & COPTER_LEDS_BITMASK_GPS)) {
        digitalWrite(COPTER_LED_3, COPTER_LED_OFF);
    }
    digitalWrite(COPTER_LED_4, COPTER_LED_OFF);
    digitalWrite(COPTER_LED_5, COPTER_LED_OFF);
    digitalWrite(COPTER_LED_6, COPTER_LED_OFF);
    digitalWrite(COPTER_LED_7, COPTER_LED_OFF);
    digitalWrite(COPTER_LED_8, COPTER_LED_OFF);
}

static void copter_leds_slow_blink(void) {
    copter_leds_motor_blink++;                                                  // this increments once every 1/10 second because it is in the 10hz loop

    if ( 0 < copter_leds_motor_blink && copter_leds_motor_blink < 6 ) {         // when the counter reaches 5 (1/2 sec), then toggle the leds
        copter_leds_off();

        // if blinking is called by the Nav Blinker decrement the Nav Blink counter
        if ((g.copter_leds_mode & COPTER_LEDS_BITMASK_MOTOR_NAV_BLINK) && !(g.copter_leds_mode & COPTER_LEDS_BITMASK_GPS_NAV_BLINK) && copter_leds_nav_blink >0 ) {
            copter_leds_nav_blink--;
        }
    }else if (5 < copter_leds_motor_blink && copter_leds_motor_blink < 11) {
        copter_leds_on();
    }else{
        copter_leds_motor_blink = 0;                                            // start blink cycle again
    }
}

static void copter_leds_fast_blink(void) {    
    copter_leds_motor_blink++;                                                  // this increments once every 1/10 second because it is in the 10hz loop
    if ( 0 < copter_leds_motor_blink && copter_leds_motor_blink < 3 ) {         // when the counter reaches 3 (1/5 sec), then toggle the leds
        copter_leds_on();
    }else if (2 < copter_leds_motor_blink && copter_leds_motor_blink < 5) {
        copter_leds_off();
    }else{
        copter_leds_motor_blink = 0;                                            // start blink cycle again
    }
}

static void copter_leds_oscillate(void) {
    copter_leds_motor_blink++;                                                  // this increments once every 1/10 second because it is in the 10hz loop
    if ( 0 < copter_leds_motor_blink && copter_leds_motor_blink < 3 ) {         // when the counter reaches 3 (1/5 sec), then toggle the leds
        if ( !(g.copter_leds_mode & COPTER_LEDS_BITMASK_AUX)) {
            digitalWrite(COPTER_LED_1, COPTER_LED_ON);
        }
 #if CONFIG_HAL_BOARD == HAL_BOARD_APM2
        if ( !(g.copter_leds_mode & COPTER_LEDS_BITMASK_BEEPER)) {
            digitalWrite(COPTER_LED_2, COPTER_LED_ON);
        }
 #else
        digitalWrite(COPTER_LED_2, COPTER_LED_ON);
 #endif
        if ( !(g.copter_leds_mode & COPTER_LEDS_BITMASK_GPS)) {
            digitalWrite(COPTER_LED_3, COPTER_LED_OFF);
        }
        digitalWrite(COPTER_LED_4, COPTER_LED_OFF);
        digitalWrite(COPTER_LED_5, COPTER_LED_ON);
        digitalWrite(COPTER_LED_6, COPTER_LED_ON);
        digitalWrite(COPTER_LED_7, COPTER_LED_OFF);
        digitalWrite(COPTER_LED_8, COPTER_LED_OFF);
    }else if (2 < copter_leds_motor_blink && copter_leds_motor_blink < 5) {
        if ( !(g.copter_leds_mode & COPTER_LEDS_BITMASK_AUX)) {
            digitalWrite(COPTER_LED_1, COPTER_LED_OFF);
        }
 #if CONFIG_HAL_BOARD == HAL_BOARD_APM2
        if ( !(g.copter_leds_mode & COPTER_LEDS_BITMASK_BEEPER)) {
            digitalWrite(COPTER_LED_2, COPTER_LED_OFF);
        }
 #else
        digitalWrite(COPTER_LED_2, COPTER_LED_OFF);
 #endif
        if ( !(g.copter_leds_mode & COPTER_LEDS_BITMASK_GPS) ) {
            digitalWrite(COPTER_LED_3, COPTER_LED_ON);
        }
        digitalWrite(COPTER_LED_4, COPTER_LED_ON);
        digitalWrite(COPTER_LED_5, COPTER_LED_OFF);
        digitalWrite(COPTER_LED_6, COPTER_LED_OFF);
        digitalWrite(COPTER_LED_7, COPTER_LED_ON);
        digitalWrite(COPTER_LED_8, COPTER_LED_ON);
    }else{
        copter_leds_motor_blink = 0;                                            // start blink cycle again
    }
}

static void copter_leds_GPS_on(void) {
    digitalWrite(COPTER_LED_3, COPTER_LED_ON);
}

static void copter_leds_GPS_off(void) {
    digitalWrite(COPTER_LED_3, COPTER_LED_OFF);
}

static void copter_leds_GPS_slow_blink(void) {
    copter_leds_GPS_blink++;                                                    // this increments once every 1/10 second because it is in the 10hz loop
    if ( 0 < copter_leds_GPS_blink && copter_leds_GPS_blink < 6 ) {             // when the counter reaches 5 (1/2 sec), then toggle the leds
        copter_leds_GPS_off();
        if ((g.copter_leds_mode & COPTER_LEDS_BITMASK_GPS_NAV_BLINK) && copter_leds_nav_blink >0 ) {   // if blinking is called by the Nav Blinker...
            copter_leds_nav_blink--;                                                                                                                    // decrement the Nav Blink counter
        }
    }else if (5 < copter_leds_GPS_blink && copter_leds_GPS_blink < 11) {
        copter_leds_GPS_on();
    }
    else copter_leds_GPS_blink = 0;                                             // start blink cycle again
}

static void copter_leds_GPS_fast_blink(void) {
    copter_leds_GPS_blink++;                                                    // this increments once every 1/10 second because it is in the 10hz loop
    if ( 0 < copter_leds_GPS_blink && copter_leds_GPS_blink < 3 ) {             // when the counter reaches 3 (1/5 sec), then toggle the leds
        copter_leds_GPS_off();
    }else if (2 < copter_leds_GPS_blink && copter_leds_GPS_blink < 5) {
        copter_leds_GPS_on();
    }
    else copter_leds_GPS_blink = 0;                                             // start blink cycle again
}

static void copter_leds_aux_off(void){
    digitalWrite(COPTER_LED_1, COPTER_LED_OFF);
}

static void copter_leds_aux_on(void){
    digitalWrite(COPTER_LED_1, COPTER_LED_ON);
}

void piezo_on(){
    if (g.copter_leds_mode & COPTER_LEDS_BITMASK_BEEPER) {
        digitalWrite(PIEZO_PIN,HIGH);
    }
}

void piezo_off(){
    if (g.copter_leds_mode & COPTER_LEDS_BITMASK_BEEPER) {
        digitalWrite(PIEZO_PIN,LOW);
    }
}

void piezo_beep(){                                                              // Note! This command should not be used in time sensitive loops
    if (g.copter_leds_mode & COPTER_LEDS_BITMASK_BEEPER) {
        piezo_on();
        delay(100);
        piezo_off();
    }
}

void piezo_beep_twice(){                                                        // Note! This command should not be used in time sensitive loops
    if (g.copter_leds_mode & COPTER_LEDS_BITMASK_BEEPER) {
        piezo_beep();
        delay(50);
        piezo_beep();
    }
}

#endif                  //COPTER_LEDS
