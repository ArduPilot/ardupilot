/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#define THISFIRMWARE "ArduCopter V2.9.1 rc-2"
/*
 *  ArduCopter Version 2.9
 *  Lead author:	Jason Short
 *  Based on code and ideas from the Arducopter team: Randy Mackay, Pat Hickey, Jose Julio, Jani Hirvinen, Andrew Tridgell, Justin Beech, Adam Rivera, Jean-Louis Naudin, Roberto Navoni
 *  Thanks to:	Chris Anderson, Mike Smith, Jordi Munoz, Doug Weibel, James Goppert, Benjamin Pelletier, Robert Lefebvre, Marco Robustini
 *
 *  This firmware is free software; you can redistribute it and/or
 *  modify it under the terms of the GNU Lesser General Public
 *  License as published by the Free Software Foundation; either
 *  version 2.1 of the License, or (at your option) any later version.
 *
 *  Special Thanks for Contributors (in alphabetical order by first name):
 *
 *  Adam M Rivera		:Auto Compass Declination
 *  Amilcar Lucas		:Camera mount library
 *  Andrew Tridgell		:General development, Mavlink Support
 *  Angel Fernandez		:Alpha testing
 *  Doug Weibel			:Libraries
 *  Christof Schmid		:Alpha testing
 *  Dani Saez           :V Octo Support
 *  Gregory Fletcher	:Camera mount orientation math
 *  Guntars				:Arming safety suggestion
 *  HappyKillmore		:Mavlink GCS
 *  Hein Hollander      :Octo Support
 *  Igor van Airde      :Control Law optimization
 *  Leonard Hall 		:Flight Dynamics, INAV throttle
 *  Jonathan Challinger :Inertial Navigation
 *  Jean-Louis Naudin   :Auto Landing
 *  Max Levine			:Tri Support, Graphics
 *  Jack Dunkle			:Alpha testing
 *  James Goppert		:Mavlink Support
 *  Jani Hiriven		:Testing feedback
 *  John Arne Birkeland	:PPM Encoder
 *  Jose Julio			:Stabilization Control laws
 *  Randy Mackay		:General development and release
 *  Marco Robustini		:Lead tester
 *  Michael Oborne		:Mission Planner GCS
 *  Mike Smith			:Libraries, Coding support
 *  Oliver				:Piezo support
 *  Olivier Adler       :PPM Encoder
 *  Robert Lefebvre		:Heli Support & LEDs
 *  Sandro Benigno      :Camera support
 *
 *  And much more so PLEASE PM me on DIYDRONES to add your contribution to the List
 *
 *  Requires modified "mrelax" version of Arduino, which can be found here:
 *  http://code.google.com/p/ardupilot-mega/downloads/list
 *
 */

////////////////////////////////////////////////////////////////////////////////
// Header includes
////////////////////////////////////////////////////////////////////////////////




#define MP32								//Multipilot32

#define SAVE_EEPROM_EXCLUDE_PARAMETERS		//Excludes the parameters are saved in EEPROM during operation



#include <arm_math.h>
#include <stdio.h>
#include <stdarg.h>

// Libraries
#include <EEPROM.h>
#include <FastSerial.h>

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Menu.h>

#include <GCS_MAVLink.h>        // MAVLink GCS definitions

#include <Arduino_Mega_ISR_Registry.h>
#include <APM_RC.h>             // ArduPilot Mega RC Library
#include <AP_GPS.h>             // ArduPilot GPS library
#include <HardwareI2C.h>    // Arduino Compatible I2C lib
#include <SPI.h>                // Arduino SPI lib

//#include <SPI3.h>               // SPI3 library
//#include <AP_Semaphore.h>       // for removing conflict between optical flow and dataflash on SPI3 bus
#include <DataFlash.h>          // ArduPilot Mega Flash Memory Library
#include <AP_ADC.h>             // ArduPilot Mega Analog to Digital Converter Library
#include <AP_AnalogSource.h>
#include <AP_Baro.h>
#include <AP_Compass.h>         // ArduPilot Mega Magnetometer Library
#include <AP_Math.h>            // ArduPilot Mega Vector/Matrix math Library
#include <AP_Curve.h>           // Curve used to linearlise throttle pwm to thrust
#include <AP_InertialSensor.h>  // ArduPilot Mega Inertial Sensor (accel & gyro) Library
#include <AP_PeriodicProcess.h> // Parent header of Timer
                                // (only included for makefile libpath to work)
#include <AP_TimerProcess.h>    // TimerProcess is the scheduler for MPU6000 reads.
#include <AP_AHRS.h>
#include <APM_PI.h>             // PI library
#include <AC_PID.h>             // PID library
#include <RC_Channel.h>         // RC Channel Library
#include <AP_Motors.h>          // AP Motors library
#include <AP_MotorsQuad.h>      // AP Motors library for Quad
#include <AP_MotorsTri.h>       // AP Motors library for Tri
#include <AP_MotorsHexa.h>      // AP Motors library for Hexa
#include <AP_MotorsY6.h>        // AP Motors library for Y6
#include <AP_MotorsOcta.h>      // AP Motors library for Octa
#include <AP_MotorsOctaQuad.h>  // AP Motors library for OctaQuad
#include <AP_MotorsHeli.h>      // AP Motors library for Heli
#include <AP_MotorsMatrix.h>    // AP Motors library for Heli
#include <AP_RangeFinder.h>     // Range finder library
//#include <AP_OpticalFlow.h> // Optical Flow library
#include <Filter.h>             // Filter library
#include <AP_Buffer.h>          // APM FIFO Buffer
#include <ModeFilter.h>         // Mode Filter from Filter library
#include <AverageFilter.h>      // Mode Filter from Filter library
#include <AP_LeadFilter.h>      // GPS Lead filter
#include <LowPassFilter.h>      // Low Pass Filter library
#include <AP_Relay.h>           // APM relay
#include <AP_Camera.h>          // Photo or video camera
#include <AP_Mount.h>           // Camera/Antenna mount
#include <AP_Airspeed.h>        // needed for AHRS build
#include <AP_InertialNav.h>     // ArduPilot Mega inertial navigation library
//#include <ThirdOrderCompFilter.h>   // Complementary filter for combining barometer altitude with accelerometers
#include <memcheck.h>
//#include <AP_TimeCheck.h>		// loop time checker library
#include <AP_Perfmon.h>

// Configuration
#include "defines.h"
#include "config.h"
//#include "config_channels.h"


// Local modules
//#include "Parameters.h"
//#include "GCS.h"

#include <AP_Declination.h>     // ArduPilot Mega Declination Helper Library

// Limits library - Puts limits on the vehicle, and takes recovery actions
#include <AP_Limits.h>
#include <AP_Limit_GPSLock.h>   // a limits library module
#include <AP_Limit_Geofence.h>  // a limits library module
#include <AP_Limit_Altitude.h>  // a limits library module
#include <stopwatch.h>
#include <i2c.h>

////////////////////////////////////////////////////////////////////////////////
// Serial ports
////////////////////////////////////////////////////////////////////////////////
//
// Note that FastSerial port buffers are allocated at ::begin time,
// so there is not much of a penalty to defining ports that we don't
// use.
FastSerialPort2(Serial);       // Telemetry port

HardwareI2C I2C2x(2);
EEPROMClass EEPROM(&I2C2x, &Serial);

static uint32_t cycles;
static uint32_t lat = 451234567;
static uint8_t byt = 8;
uint32_t lat_new = 0;
uint8_t byt_new = 0;

void setup() {

    I2C2x.begin();
    EEPROM.init(&I2C2x,&Serial);
    Serial.printf_P("starting test");
    Serial.println();
    stopwatch_init();
    if(i2c_is_busy()){
	Serial.printf_P("I2C busy!");
    }
    uintptr_t mem = WP_START_BYTE + (WP_SIZE);
    stopwatch_reset();
    EEPROM.eeprom_write_dword((uint32_t *) mem, lat);     // Lat is stored in decimal degrees * 10^7
    cycles = stopwatch_getus();
    Serial.printf_P("write byte took: %d us \r\n", cycles);
    Serial.println();

    mem += 4;
    stopwatch_reset();
    EEPROM.eeprom_write_byte((uint8_t *)   mem, byt);
    cycles = stopwatch_getus();
    Serial.printf_P("write byte took: %d us \r\n", cycles);
    Serial.println();

    mem = WP_START_BYTE + (WP_SIZE);
    stopwatch_reset();
    lat_new = EEPROM.eeprom_read_dword((uint32_t *)mem);
    cycles = stopwatch_getus();
    Serial.printf_P("read dword: %ld took: %ld us \r\n",lat_new, cycles);
    Serial.println();

    mem += 4;
    stopwatch_reset();
    byt_new = EEPROM.eeprom_read_byte((uint8_t *)mem);
    cycles = stopwatch_getus();
    Serial.printf_P("read byte: %d took: %ld us \r\n",byt_new, cycles);
    Serial.println();

    Serial.printf("Writing home location");
    Serial.println();
    stopwatch_reset();
        mem = WP_START_BYTE + (WP_SIZE);

        EEPROM.eeprom_write_byte((uint8_t *)   mem, (uint8_t)1);

        mem++;
        EEPROM.eeprom_write_byte((uint8_t *)   mem, (uint8_t)2);

        mem++;
        EEPROM.eeprom_write_byte((uint8_t *)   mem, (uint8_t)3);

        mem++;
        EEPROM.eeprom_write_dword((uint32_t *) mem, (uint32_t)1234567890);     // Alt is stored in CM!

        mem += 4;
        EEPROM.eeprom_write_dword((uint32_t *) mem, (uint32_t)987654321);     // Lat is stored in decimal degrees * 10^7

        mem += 4;
        EEPROM.eeprom_write_dword((uint32_t *) mem, (uint32_t)5432167890);
        cycles = stopwatch_getus();
        Serial.printf_P("write home location took: %ld us \r\n", cycles);
        Serial.println();

        Serial.printf("Reading home location");
            Serial.println();
            stopwatch_reset();
                mem = WP_START_BYTE + (WP_SIZE);

                EEPROM.eeprom_read_byte((uint8_t *)   mem);

                mem++;
                EEPROM.eeprom_read_byte((uint8_t *)   mem);

                mem++;
                EEPROM.eeprom_read_byte((uint8_t *)   mem);

                mem++;
                EEPROM.eeprom_read_dword((uint32_t *) mem);     // Alt is stored in CM!

                mem += 4;
                EEPROM.eeprom_read_dword((uint32_t *) mem);     // Lat is stored in decimal degrees * 10^7

                mem += 4;
                EEPROM.eeprom_read_dword((uint32_t *) mem);
                cycles = stopwatch_getus();
                Serial.printf_P("read home location took: %ld us \r\n", cycles);
                Serial.println();

}

void loop()
{

    
}

