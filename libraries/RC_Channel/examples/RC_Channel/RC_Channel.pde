/*
 *       Example of RC_Channel library.
 *       Code by Jason Short. 2010
 *       DIYDrones.com
 *
 */

#include <FastSerial.h>
#include <AP_Common.h>          // ArduPilot Mega Common Library
#include <AP_Param.h>
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

#define CH_1 0
#define CH_2 1
#define CH_3 2
#define CH_4 3
#define CH_5 4
#define CH_6 5
#define CH_7 6
#define CH_8 7


RC_Channel rc_1(CH_1);
RC_Channel rc_2(CH_2);
RC_Channel rc_3(CH_3);
RC_Channel rc_4(CH_4);
RC_Channel rc_5(CH_5);
RC_Channel rc_6(CH_6);
RC_Channel rc_7(CH_7);
RC_Channel rc_8(CH_8);
RC_Channel *rc = &rc_1;

void setup()
{
    Serial.begin(115200);
    Serial.println("ArduPilot RC Channel test");
    isr_registry.init();
    APM_RC.Init( &isr_registry );               // APM Radio initialization


    delay(500);

    // setup radio

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
    rc_4.set_angle(6000);
    rc_4.set_dead_zone(500);
    rc_5.set_range(0,1000);
    rc_6.set_range(200,800);
    rc_7.set_range(0,1000);
    rc_8.set_range(0,1000);

#if 0
    for (byte i=0; i<8; i++) {
	    rc[i].set_reverse(false);
    }
#endif

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
    for (byte i=0; i<8; i++) {
	    rc[i].set_pwm(APM_RC.InputCh(CH_1+i));
    }
}

void print_pwm()
{
    for (byte i=0; i<8; i++) {
	    Serial.printf("ch%u: %4d ", (unsigned)i+1, (int)rc[i].control_in);
    }
    Serial.printf("\n");
}

void
print_radio_values()
{
    for (byte i=0; i<8; i++) {
	    Serial.printf("CH%u: %u|%u\n",
			  (unsigned)i+1, 
			  (unsigned)rc[i].radio_min, 
			  (unsigned)rc[i].radio_max); 
    }
}


void
setup_radio()
{
	Serial.println("\n\nRadio Setup:");
	uint8_t i;
	
	for(i = 0; i < 100;i++){
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
	while(1){
		
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
		
		if(Serial.available() > 0){
			//rc_3.radio_max += 250;
			Serial.flush();
			
			Serial.println("Radio calibrated, Showing control values:");
			break;
		}
	}
	return;
}
