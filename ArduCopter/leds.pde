static void update_lights()
{
    switch(led_mode) {
    case NORMAL_LEDS:
        update_motor_light();
        update_GPS_light();
        break;

    case SAVE_TRIM_LEDS:
        dancing_light();
        break;
    }
}

static void update_GPS_light(void)
{
    // GPS LED on if we have a fix or Blink GPS LED if we are receiving data
    // ---------------------------------------------------------------------
    switch (g_gps->status()) {

    case (2):
        if(ap.home_is_set) {                                      // JLN update
            digitalWrite(C_LED_PIN, LED_ON);                  //Turn LED C on when gps has valid fix AND home is set.
        } else {
            digitalWrite(C_LED_PIN, LED_OFF);
        }
        break;

    case (1):
        if (g_gps->valid_read == true) {
            ap_system.GPS_light = !ap_system.GPS_light;                     // Toggle light on and off to indicate gps messages being received, but no GPS fix lock
            if (ap_system.GPS_light) {
                digitalWrite(C_LED_PIN, LED_OFF);
            }else{
                digitalWrite(C_LED_PIN, LED_ON);
            }
            g_gps->valid_read = false;
        }
        break;

    default:
        digitalWrite(C_LED_PIN, LED_OFF);
        break;
    }
}

static void update_motor_light(void)
{
    if(motors.armed() == false) {
        ap_system.motor_light = !ap_system.motor_light;

        // blink
        if(ap_system.motor_light) {
            digitalWrite(A_LED_PIN, LED_ON);
        }else{
            digitalWrite(A_LED_PIN, LED_OFF);
        }
    }else{
        if(!ap_system.motor_light) {
            ap_system.motor_light = true;
            digitalWrite(A_LED_PIN, LED_ON);
        }
    }
}

static void dancing_light()
{
    static byte step;

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
static void clear_leds()
{
    digitalWrite(A_LED_PIN, LED_OFF);
    digitalWrite(B_LED_PIN, LED_OFF);
    digitalWrite(C_LED_PIN, LED_OFF);
    ap_system.motor_light = false;
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


#if COPTER_LEDS == ENABLED
static void update_copter_leds(void)
{
    if (g.copter_leds_mode == 0) {
        copter_leds_reset();                                        //method of reintializing LED state
    }

    if ( bitRead(g.copter_leds_mode, 0) ) {
        if (motors.armed() == true) {
            if (ap.low_battery == true) {
                if ( bitRead(g.copter_leds_mode, 4 )) {
                    copter_leds_oscillate();                        //if motors are armed, but battery level is low, motor leds fast blink
                } else {
                    copter_leds_fast_blink();                       //if motors are armed, but battery level is low, motor leds oscillate
                }
            } else if ( !bitRead(g.copter_leds_mode, 5 ) ) {
                copter_leds_on();                                   //if motors are armed, battery level OK, all motor leds ON
            } else if ( bitRead(g.copter_leds_mode, 5 ) ) {
                if ( copter_leds_nav_blink >0 ) {
                    copter_leds_slow_blink();                       //if nav command was seen, blink LEDs.
                } else {
                    copter_leds_on();
                }
            }
        } else {
            copter_leds_slow_blink();                               //if motors are not armed, blink motor leds
        }
    }

    if ( bitRead(g.copter_leds_mode, 1) ) {

        // GPS LED on if we have a fix or Blink GPS LED if we are receiving data
        // ---------------------------------------------------------------------
        switch (g_gps->status()) {

        case (2):
            if(ap.home_is_set) {
                if ( !bitRead(g.copter_leds_mode, 6 ) ) {
                    copter_leds_GPS_on();							//Turn GPS LEDs on when gps has valid fix AND home is set
                } else if (bitRead(g.copter_leds_mode, 6 ) ) {
                    if ( copter_leds_nav_blink >0 ) {
                        copter_leds_GPS_slow_blink();               //if nav command was seen, blink LEDs.
                    } else {
                        copter_leds_GPS_on();
                    }
                }
            } else {
                copter_leds_GPS_fast_blink();                       //if GPS has fix, but home is not set, blink GPS LED fast
            }
            break;

        case (1):

            copter_leds_GPS_slow_blink();                           //if GPS has valid reads, but no fix, blink GPS LED slow

            break;

        default:
            copter_leds_GPS_off();                                  //if no valid GPS signal, turn GPS LED off
            break;
        }
    }

    if ( bitRead(g.copter_leds_mode, 2) ) {
        if (200 <= g.rc_7.control_in && g.rc_7.control_in < 400) {
            copter_leds_aux_on();                                   //if sub-control of Ch7 is high, turn Aux LED on
        } else if (g.rc_7.control_in < 200) {
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
    if ( !bitRead(g.copter_leds_mode, 2) ) {
        digitalWrite(COPTER_LED_1, COPTER_LED_ON);
    }
 #if CONFIG_APM_HARDWARE == APM_HARDWARE_APM2
    if ( !bitRead(g.copter_leds_mode, 3) ) {
        digitalWrite(COPTER_LED_2, COPTER_LED_ON);
    }
 #else
    digitalWrite(COPTER_LED_2, COPTER_LED_ON);
 #endif
    if ( !bitRead(g.copter_leds_mode, 1) ) {
        digitalWrite(COPTER_LED_3, COPTER_LED_ON);
    }
    digitalWrite(COPTER_LED_4, COPTER_LED_ON);
    digitalWrite(COPTER_LED_5, COPTER_LED_ON);
    digitalWrite(COPTER_LED_6, COPTER_LED_ON);
    digitalWrite(COPTER_LED_7, COPTER_LED_ON);
    digitalWrite(COPTER_LED_8, COPTER_LED_ON);
}

static void copter_leds_off(void) {
    if ( !bitRead(g.copter_leds_mode, 2) ) {
        digitalWrite(COPTER_LED_1, COPTER_LED_OFF);
    }
 #if CONFIG_APM_HARDWARE == APM_HARDWARE_APM2
    if ( !bitRead(g.copter_leds_mode, 3) ) {
        digitalWrite(COPTER_LED_2, COPTER_LED_OFF);
    }
 #else
    digitalWrite(COPTER_LED_2, COPTER_LED_OFF);
 #endif
    if ( !bitRead(g.copter_leds_mode, 1) ) {
        digitalWrite(COPTER_LED_3, COPTER_LED_OFF);
    }
    digitalWrite(COPTER_LED_4, COPTER_LED_OFF);
    digitalWrite(COPTER_LED_5, COPTER_LED_OFF);
    digitalWrite(COPTER_LED_6, COPTER_LED_OFF);
    digitalWrite(COPTER_LED_7, COPTER_LED_OFF);
    digitalWrite(COPTER_LED_8, COPTER_LED_OFF);
}

static void copter_leds_slow_blink(void) {
    copter_leds_motor_blink++;                                                                                                                                                  // this increments once every 1/10 second because it is in the 10hz loop
    if ( 0 < copter_leds_motor_blink && copter_leds_motor_blink < 6 ) {                                                                                                 // when the counter reaches 5 (1/2 sec), then toggle the leds
        copter_leds_off();
        if (  bitRead(g.copter_leds_mode, 5 ) && !bitRead(g.copter_leds_mode, 6 ) && copter_leds_nav_blink >0 ) {               // if blinking is called by the Nav Blinker...
            copter_leds_nav_blink--;                                                                                                                                                                            // decrement the Nav Blink counter
        }
    }else if (5 < copter_leds_motor_blink && copter_leds_motor_blink < 11) {
        copter_leds_on();
    }
    else copter_leds_motor_blink = 0;                                                                                                                                                   // start blink cycle again
}

static void copter_leds_fast_blink(void) {
    copter_leds_motor_blink++;                                                                                                                                                  // this increments once every 1/10 second because it is in the 10hz loop
    if ( 0 < copter_leds_motor_blink && copter_leds_motor_blink < 3 ) {                                                                                                 // when the counter reaches 3 (1/5 sec), then toggle the leds
        copter_leds_on();
    }else if (2 < copter_leds_motor_blink && copter_leds_motor_blink < 5) {
        copter_leds_off();
    }
    else copter_leds_motor_blink = 0;                                                                                                                                                   // start blink cycle again
}


static void copter_leds_oscillate(void) {
    copter_leds_motor_blink++;                                                                                                                                                  // this increments once every 1/10 second because it is in the 10hz loop
    if ( 0 < copter_leds_motor_blink && copter_leds_motor_blink < 3 ) {                                                                                                 // when the counter reaches 3 (1/5 sec), then toggle the leds
        if ( !bitRead(g.copter_leds_mode, 2) ) {
            digitalWrite(COPTER_LED_1, COPTER_LED_ON);
        }
 #if CONFIG_APM_HARDWARE == APM_HARDWARE_APM2
        if ( !bitRead(g.copter_leds_mode, 3) ) {
            digitalWrite(COPTER_LED_2, COPTER_LED_ON);
        }
 #else
        digitalWrite(COPTER_LED_2, COPTER_LED_ON);
 #endif
        if ( !bitRead(g.copter_leds_mode, 1) ) {
            digitalWrite(COPTER_LED_3, COPTER_LED_OFF);
        }
        digitalWrite(COPTER_LED_4, COPTER_LED_OFF);
        digitalWrite(COPTER_LED_5, COPTER_LED_ON);
        digitalWrite(COPTER_LED_6, COPTER_LED_ON);
        digitalWrite(COPTER_LED_7, COPTER_LED_OFF);
        digitalWrite(COPTER_LED_8, COPTER_LED_OFF);
    }else if (2 < copter_leds_motor_blink && copter_leds_motor_blink < 5) {
        if ( !bitRead(g.copter_leds_mode, 2) ) {
            digitalWrite(COPTER_LED_1, COPTER_LED_OFF);
        }
 #if CONFIG_APM_HARDWARE == APM_HARDWARE_APM2
        if ( !bitRead(g.copter_leds_mode, 3) ) {
            digitalWrite(COPTER_LED_2, COPTER_LED_OFF);
        }
 #else
        digitalWrite(COPTER_LED_2, COPTER_LED_OFF);
 #endif
        if ( !bitRead(g.copter_leds_mode, 1) ) {
            digitalWrite(COPTER_LED_3, COPTER_LED_ON);
        }
        digitalWrite(COPTER_LED_4, COPTER_LED_ON);
        digitalWrite(COPTER_LED_5, COPTER_LED_OFF);
        digitalWrite(COPTER_LED_6, COPTER_LED_OFF);
        digitalWrite(COPTER_LED_7, COPTER_LED_ON);
        digitalWrite(COPTER_LED_8, COPTER_LED_ON);
    }
    else copter_leds_motor_blink = 0;                                           // start blink cycle again
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
        if (  bitRead(g.copter_leds_mode, 6 ) && copter_leds_nav_blink >0 ) {   // if blinking is called by the Nav Blinker...
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
    digitalWrite(PIEZO_PIN,HIGH);
}

void piezo_off(){
    digitalWrite(PIEZO_PIN,LOW);
}

void piezo_beep(){                                                              // Note! This command should not be used in time sensitive loops
    piezo_on();
    delay(100);
    piezo_off();
}

#endif                  //COPTER_LEDS