/**
 * @file leds.pde
 *
 * @brief Board status LEDs and External LED logic
 */

/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/**
 * update_board_leds
 *
 * @access static
 * @return void
 *
 * @brief Status leds on APM board. Called at 10Hz - sets the lights to their respective state depending on copter state
 */
static void update_board_leds()
{
    // we need to slow down the calls to the GPS and dancing lights to 3.33hz
    static uint8_t counter = 0;
    if(++counter >= 3){
        counter = 0;
    }

    switch(led_mode) {
    case NORMAL_LEDS:
        update_arming_light();
        if (counter == 0) {
            update_GPS_light();
        }
        break;

    case SAVE_TRIM_LEDS:
        if (counter == 0) {
            dancing_light();
        }
        break;
    }
}

/**
 * update_GPS_light
 *
 * @brief Sets GPS light on APM board
 */
static void update_GPS_light(void)
{
    // GPS LED on if we have a fix or Blink GPS LED if we are receiving data
    // ---------------------------------------------------------------------
    switch (g_gps->status()) {
        case GPS::NO_FIX:
        case GPS::GPS_OK_FIX_2D:
            // check if we've blinked since the last gps update
            if (g_gps->valid_read) {
                g_gps->valid_read = false;
                ap_system.GPS_light = !ap_system.GPS_light;                     // Toggle light on and off to indicate gps messages being received, but no GPS fix lock
                if (ap_system.GPS_light) {
                    digitalWrite(C_LED_PIN, LED_OFF);
                }else{
                    digitalWrite(C_LED_PIN, LED_ON);
                }
            }
            break;

        case GPS::GPS_OK_FIX_3D:
            if(ap.home_is_set) {
                digitalWrite(C_LED_PIN, LED_ON);                  //Turn LED C on when gps has valid fix AND home is set.
            } else {
                digitalWrite(C_LED_PIN, LED_OFF);
            }
            break;

        default:
            digitalWrite(C_LED_PIN, LED_OFF);
            break;
    }
}

/**
 * update_arming_light
 *
 * @brief Sets armed led on APM board
 */
static void update_arming_light(void)
{
    // counter to control state
    static int8_t counter = 0;
    counter++;

    // disarmed
    if(!motors.armed()) {
        if(!ap.pre_arm_check) {
            // failed pre-arm checks so double flash
            switch(counter) {
                case 0:
                    ap_system.arming_light = true;
                    break;
                case 1:
                    ap_system.arming_light = false;
                    break;
                case 2:
                    ap_system.arming_light = true;
                    break;
                case 3:
                case 4:
                    ap_system.arming_light = false;
                    break;
                default:
                    // reset counter to restart the sequence
                    counter = -1;
                    break;
            }
        }else{
            // passed pre-arm checks so slower single flash
            switch(counter) {
                case 0:
                case 1:
                case 2:
                    ap_system.arming_light = true;
                    break;
                case 3:
                case 4:
                case 5:
                    ap_system.arming_light = false;
                    break;
                default:
                    // reset counter to restart the sequence
                    counter = -1;
                    break;
            }
        }
        // set arming led from arming_light flag
        if(ap_system.arming_light) {
            digitalWrite(A_LED_PIN, LED_ON);
        }else{
            digitalWrite(A_LED_PIN, LED_OFF);
        }
    }else{
        // armed
        if(!ap_system.arming_light) {
            ap_system.arming_light = true;
            digitalWrite(A_LED_PIN, LED_ON);
        }
    }
}

/**
 * dancing_light
 *
 * @access static
 * @return void
 *
 * @brief Dancing pattern on APM board leds - used principally for ESC calibration and save trim.
 */
static void dancing_light()
{
    static uint8_t step;

    if (step++ == 3)
        step = 0;

    switch(step)
    {
    case 0:
        digitalWrite(C_LED_PIN, LED_OFF);
        digitalWrite(A_LED_PIN, LED_ON);
        break;

    case 1:
        digitalWrite(A_LED_PIN, LED_OFF);
        digitalWrite(B_LED_PIN, LED_ON);
        break;

    case 2:
        digitalWrite(B_LED_PIN, LED_OFF);
        digitalWrite(C_LED_PIN, LED_ON);
        break;
    }
}

/**
 * clear_leds
 *
 * @brief APM board LEDS off
 */
static void clear_leds()
{
    digitalWrite(A_LED_PIN, LED_OFF);
    digitalWrite(B_LED_PIN, LED_OFF);
    digitalWrite(C_LED_PIN, LED_OFF);
    ap_system.arming_light = false;
    led_mode = NORMAL_LEDS;
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
/**
 * copter_leds_init
 *
 *
 * @brief Initialise external LEDs
 */
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

/**
 * update_copter_leds
 *
 * @brief Called repeatedly to update external LEDs.  
 */
static void update_copter_leds(void)
{
    if (g.copter_leds_mode == 0) {
        copter_leds_reset();                                        //method of reintializing LED state
    }

    // motor leds control
    if (g.copter_leds_mode & COPTER_LEDS_BITMASK_ENABLED) {
        if (motors.armed()) {
            if (ap.low_battery) {
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
        if (ap_system.CH7_flag) {
            copter_leds_aux_on();                                   //if sub-control of Ch7 is high, turn Aux LED on
        } else {
            copter_leds_aux_off();                                  //if sub-control of Ch7 is low, turn Aux LED off
        }
    }
}

/**
 * copter_leds_reset
 *
 * @brief External LEDs off
 */
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

/**
 * copter_leds_on
 *
 * @brief All external LEDs on
 */
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

/**
 * copter_leds_off
 *
 * @brief All external LEDs off
 */
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

/**
 * copter_leds_slow_blink
 *
 * @brief Slow blinks external LEDs
 */
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


/**
 * copter_leds_fast_blink
 *
 * @brief Fast blinks external leds
 */
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

/**
 * copter_leds_oscillate
 *
 * @brief Oscillates External LEDs.
 */
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

/**
 * copter_leds_GPS_on
 *
 * @brief External GPS led on
 */
static void copter_leds_GPS_on(void) {
    digitalWrite(COPTER_LED_3, COPTER_LED_ON);
}

/**
 * copter_leds_GPS_off
 *
 * @brief External GPS led off
 */
static void copter_leds_GPS_off(void) {
    digitalWrite(COPTER_LED_3, COPTER_LED_OFF);
}

/**
 * copter_leds_GPS_slow_blink
 *
 * @brief Slow blink external GPS led
 */
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

/**
 * copter_leds_GPS_fast_blink
 *
 * @brief Fast blink external GPS led
 */
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

/**
 * piezo_on
 *
 * @brief External optional buzzer on
 */
void piezo_on(){
    if (g.copter_leds_mode & COPTER_LEDS_BITMASK_BEEPER) {
        digitalWrite(PIEZO_PIN,HIGH);
    }
}

/**
 * piezo_off
 *
 * @brief External optional buzzer off
 */
void piezo_off(){
    if (g.copter_leds_mode & COPTER_LEDS_BITMASK_BEEPER) {
        digitalWrite(PIEZO_PIN,LOW);
    }
}

/**
 * piezo_beep
 *
 * @brief Beep external optional buzzer
 */
void piezo_beep(){                                                              // Note! This command should not be used in time sensitive loops
    if (g.copter_leds_mode & COPTER_LEDS_BITMASK_BEEPER) {
        piezo_on();
        delay(100);
        piezo_off();
    }
}

/**
 * piezo_beep_twice
 *
 * @brief Beep external optional buzzer twice
 */
void piezo_beep_twice(){                                                        // Note! This command should not be used in time sensitive loops
    if (g.copter_leds_mode & COPTER_LEDS_BITMASK_BEEPER) {
        piezo_beep();
        delay(50);
        piezo_beep();
    }
}

#endif                  //COPTER_LEDS
