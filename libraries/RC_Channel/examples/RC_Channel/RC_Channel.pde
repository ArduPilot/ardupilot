/*
 *       Example of RC_Channel library.
 *       Code by Jason Short. 2010
 *       DIYDrones.com
 *
 */

#include <FastSerial.h>
#include <AP_Common.h>          // ArduPilot Mega Common Library
#include <Arduino_Mega_ISR_Registry.h>
#include <AP_Math.h>        // ArduPilot Mega Vector/Matrix math Library
#include <APM_RC.h>             // ArduPilot Mega RC Library
#include <RC_Channel.h>         // ArduPilot Mega RC Library

// define APM1 or APM2
#define APM_HARDWARE_APM1       1
#define APM_HARDWARE_APM2       2

// set your hardware type here
#define CONFIG_APM_HARDWARE APM_HARDWARE_APM2

Arduino_Mega_ISR_Registry isr_registry;

////////////////////////////////////////////////////////////////////////////////
// Serial ports
////////////////////////////////////////////////////////////////////////////////
//
// Note that FastSerial port buffers are allocated at ::begin time,
// so there is not much of a penalty to defining ports that we don't
// use.
//
FastSerialPort0(Serial);        // FTDI/console
FastSerialPort1(Serial1);       // GPS port
FastSerialPort3(Serial3);       // Telemetry port

////////////////////////////////////////////
// RC Hardware
////////////////////////////////////////////
#if CONFIG_APM_HARDWARE == APM_HARDWARE_APM2
APM_RC_APM2 APM_RC;
#else
APM_RC_APM1 APM_RC;
#endif

RC_Channel rc_1;
RC_Channel rc_2;
RC_Channel rc_3;
RC_Channel rc_4;
RC_Channel rc_5;
RC_Channel rc_6;
RC_Channel rc_7;
RC_Channel rc_8;

#define CH_1 0
#define CH_2 1
#define CH_3 2
#define CH_4 3
#define CH_5 4
#define CH_6 5
#define CH_7 6
#define CH_8 7

void setup()
{
    Serial.begin(115200);
    Serial.println("ArduPilot RC Channel test");
    APM_RC.Init( &isr_registry );               // APM Radio initialization


    delay(500);

    // setup radio

    // read eepom or set manually
    /*
     *  rc_1.radio_min = 1100;
     *  rc_1.radio_max = 1900;
     *
     *  rc_2.radio_min = 1100;
     *  rc_2.radio_max = 1900;
     *
     *  rc_3.radio_min = 1100;
     *  rc_3.radio_max = 1900;
     *
     *  rc_4.radio_min = 1100;
     *  rc_4.radio_max = 1900;
     *
     *  // or
     *
     *  rc_1.load_eeprom();
     *  rc_2.load_eeprom();
     *  rc_3.load_eeprom();
     *  rc_4.load_eeprom();
     *  rc_5.load_eeprom();
     *  rc_6.load_eeprom();
     *  rc_7.load_eeprom();
     *  rc_8.load_eeprom();
     *
     */

    // interactive setup
    setup_radio();

    print_radio_values();

    // set type of output, symmetrical angles or a number range;
    rc_1.set_angle(4500);
    rc_1.set_dead_zone(80);
    rc_2.set_angle(4500);
    rc_2.set_dead_zone(80);
    rc_3.set_range(0,1000);
    rc_3.set_dead_zone(20);
    rc_3.scale_output = .8;     // gives more dynamic range to quads
    rc_4.set_angle(6000);
    rc_4.set_dead_zone(500);
    rc_5.set_range(0,1000);
    rc_5.set_filter(false);
    rc_6.set_range(200,800);
    rc_7.set_range(0,1000);
    rc_8.set_range(0,1000);

    for (byte i = 0; i < 30; i++) {
        read_radio();
    }
    rc_1.trim();
    rc_2.trim();
    rc_4.trim();
}

void loop()
{
    delay(20);
    read_radio();
    print_pwm();
}


void read_radio()
{
    rc_1.set_pwm(APM_RC.InputCh(CH_1));
    rc_2.set_pwm(APM_RC.InputCh(CH_2));
    rc_3.set_pwm(APM_RC.InputCh(CH_3));
    rc_4.set_pwm(APM_RC.InputCh(CH_4));
    rc_5.set_pwm(APM_RC.InputCh(CH_5));
    rc_6.set_pwm(APM_RC.InputCh(CH_6));
    rc_7.set_pwm(APM_RC.InputCh(CH_7));
    rc_8.set_pwm(APM_RC.InputCh(CH_8));
    //Serial.printf_P(PSTR("OUT 1: %d\t2: %d\t3: %d\t4: %d \n"), rc_1.control_in, rc_2.control_in, rc_3.control_in, rc_4.control_in);
}

void print_pwm()
{
    Serial.print("ch1 ");
    Serial.print(rc_1.control_in, DEC);
    Serial.print("\tch2: ");
    Serial.print(rc_2.control_in, DEC);
    Serial.print("\tch3 :");
    Serial.print(rc_3.control_in, DEC);
    Serial.print("\tch4 :");
    Serial.print(rc_4.control_in, DEC);
    Serial.print("\tch5 :");
    Serial.print(rc_5.control_in, DEC);
    Serial.print("\tch6 :");
    Serial.print(rc_6.control_in, DEC);
    Serial.print("\tch7 :");
    Serial.print(rc_7.control_in, DEC);
    Serial.print("\tch8 :");
    Serial.println(rc_8.control_in, DEC);
}

void
print_radio_values()
{
    Serial.print("CH1: ");
    Serial.print(rc_1.radio_min, DEC);
    Serial.print(" | ");
    Serial.println(rc_1.radio_max, DEC);

    Serial.print("CH2: ");
    Serial.print(rc_2.radio_min, DEC);
    Serial.print(" | ");
    Serial.println(rc_2.radio_max, DEC);

    Serial.print("CH3: ");
    Serial.print(rc_3.radio_min, DEC);
    Serial.print(" | ");
    Serial.println(rc_3.radio_max, DEC);

    Serial.print("CH4: ");
    Serial.print(rc_4.radio_min, DEC);
    Serial.print(" | ");
    Serial.println(rc_4.radio_max, DEC);

    Serial.print("CH5: ");
    Serial.print(rc_5.radio_min, DEC);
    Serial.print(" | ");
    Serial.println(rc_5.radio_max, DEC);

    Serial.print("CH6: ");
    Serial.print(rc_6.radio_min, DEC);
    Serial.print(" | ");
    Serial.println(rc_6.radio_max, DEC);

    Serial.print("CH7: ");
    Serial.print(rc_7.radio_min, DEC);
    Serial.print(" | ");
    Serial.println(rc_7.radio_max, DEC);

    Serial.print("CH8: ");
    Serial.print(rc_8.radio_min, DEC);
    Serial.print(" | ");
    Serial.println(rc_8.radio_max, DEC);
}


void
setup_radio()
{
    Serial.println("\n\nRadio Setup:");
    uint8_t i;

    for(i = 0; i < 100; i++) {
        delay(20);
        read_radio();
    }

    rc_1.radio_min = rc_1.radio_in;
    rc_2.radio_min = rc_2.radio_in;
    rc_3.radio_min = rc_3.radio_in;
    rc_4.radio_min = rc_4.radio_in;
    rc_5.radio_min = rc_5.radio_in;
    rc_6.radio_min = rc_6.radio_in;
    rc_7.radio_min = rc_7.radio_in;
    rc_8.radio_min = rc_8.radio_in;

    rc_1.radio_max = rc_1.radio_in;
    rc_2.radio_max = rc_2.radio_in;
    rc_3.radio_max = rc_3.radio_in;
    rc_4.radio_max = rc_4.radio_in;
    rc_5.radio_max = rc_5.radio_in;
    rc_6.radio_max = rc_6.radio_in;
    rc_7.radio_max = rc_7.radio_in;
    rc_8.radio_max = rc_8.radio_in;

    rc_1.radio_trim = rc_1.radio_in;
    rc_2.radio_trim = rc_2.radio_in;
    rc_4.radio_trim = rc_4.radio_in;
    // 3 is not trimed
    rc_5.radio_trim = 1500;
    rc_6.radio_trim = 1500;
    rc_7.radio_trim = 1500;
    rc_8.radio_trim = 1500;

    Serial.println("\nMove all controls to each extreme. Hit Enter to save:");
    while(1) {

        delay(20);
        // Filters radio input - adjust filters in the radio.pde file
        // ----------------------------------------------------------
        read_radio();

        rc_1.update_min_max();
        rc_2.update_min_max();
        rc_3.update_min_max();
        rc_4.update_min_max();
        rc_5.update_min_max();
        rc_6.update_min_max();
        rc_7.update_min_max();
        rc_8.update_min_max();

        if(Serial.available() > 0) {
            //rc_3.radio_max += 250;
            Serial.flush();

            Serial.println("Radio calibrated, Showing control values:");
            break;
        }
    }
    return;
}