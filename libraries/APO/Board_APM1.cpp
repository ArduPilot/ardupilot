/*
 * Board_APM1.cpp
 *
 *  Created on: Dec 7, 2011
 *
 */

#include <Wire.h>
#include <FastSerial.h>
#include <AP_Common.h>
#include <APM_RC.h>
#include <AP_RangeFinder.h>
#include <GCS_MAVLink.h>
#include <AP_ADC.h>
#include <AP_DCM.h>
#include <AP_Compass.h>
#include <AP_GPS.h>
#include <AP_IMU.h>
#include <APM_BMP085.h>
#include <ModeFilter.h>
#include <APO.h>
#include <AP_AnalogSource.h>
#include <AP_InertialSensor.h>
#include <DataFlash.h>


#include "Board_APM1.h"

namespace apo {

Board_APM1::Board_APM1(mode_e mode, MAV_TYPE vehicle, options_t options) : AP_Board(mode,vehicle,options) {

    const uint32_t debugBaud = 57600;
    const uint32_t telemBaud = 57600;
    const uint32_t gpsBaud = 38400;
    const uint32_t hilBaud = 115200;
    const uint8_t batteryPin = 0;
    const float batteryVoltageDivRatio = 6;
    const float batteryMinVolt = 10.0;
    const float batteryMaxVolt = 12.4;
    Matrix3f compassOrientation = AP_COMPASS_COMPONENTS_UP_PINS_FORWARD;

    AP_Var::load_all();

    Wire.begin();

    // debug
    Serial.begin(debugBaud, 128, 128);
    debug = &Serial;
    debug->println_P(PSTR("initialized debug port"));

    // hil
    Serial1.begin(hilBaud, 128, 128);
    hilPort = &Serial1;
    hilPort->println_P(PSTR("initialized hil port"));

    slideSwitchPin = 40;
    pushButtonPin = 41;
    aLedPin = 37;
    bLedPin = 36;
    cLedPin = 35;

    eepromMaxAddr = 1024;
    pinMode(aLedPin, OUTPUT); //  extra led
    pinMode(bLedPin, OUTPUT); //  imu ledclass AP_CommLink;
    pinMode(cLedPin, OUTPUT); //  gps led
    pinMode(slideSwitchPin, INPUT);
    pinMode(pushButtonPin, INPUT);
    DDRL |= B00000100; // set port L, pint 2 to output for the relay
    isr_registry = new Arduino_Mega_ISR_Registry;
    radio = new APM_RC_APM1;
    radio->Init(isr_registry);
    dataFlash = new DataFlash_APM1;
    scheduler = new AP_TimerProcess;
    scheduler->init(isr_registry);
    adc = new AP_ADC_ADS7844;
    adc->Init(scheduler);

   /*
     * Sensor initialization
     */
    if (getMode() == MODE_LIVE) {

        if (_options & opt_batteryMonitor) {
            batteryMonitor = new AP_BatteryMonitor(batteryPin,batteryVoltageDivRatio,batteryMinVolt,batteryMaxVolt);
        }

        if (_options & opt_gps) {
            Serial1.begin(gpsBaud, 128, 16); // gps
            debug->println_P(PSTR("initializing gps"));
            AP_GPS_Auto gpsDriver(&Serial1, &(gps));
            gps = &gpsDriver;
            gps->callback = delay;
            gps->init();
        }

        if (_options & opt_baro) {
            debug->println_P(PSTR("initializing baro"));
            baro = new APM_BMP085_Class;
            baro->Init(0,false);
        }

        if (_options & opt_compass) {
            debug->println_P(PSTR("initializing compass"));
            compass = new AP_Compass_HMC5843;
            compass->set_orientation(compassOrientation);
            compass->set_offsets(0,0,0);
            compass->set_declination(0.0);
            compass->init();
        }
    }

    /**
     * Initialize ultrasonic sensors. If sensors are not plugged in, the navigator will not
     * initialize them and NULL will be assigned to those corresponding pointers.
     * On detecting NU/LL assigned to any ultrasonic sensor, its corresponding block of code
     * will not be executed by the navigator.
     * The coordinate system is assigned by the right hand rule with the thumb pointing down.
     * In set_orientation, it is defined as (front/back,left/right,down,up)
     */

    // XXX this isn't really that general, should be a better way

    if (_options & opt_rangeFinderFront) {
        debug->println_P(PSTR("initializing front range finder"));
        RangeFinder * rangeFinder = new AP_RangeFinder_MaxsonarXL(new AP_AnalogSource_Arduino(1),new ModeFilter);
        rangeFinder->set_orientation(1, 0, 0);
        rangeFinders.push_back(rangeFinder);
    }

    if (_options & opt_rangeFinderBack) {
        debug->println_P(PSTR("initializing back range finder"));
        RangeFinder * rangeFinder = new AP_RangeFinder_MaxsonarXL(new AP_AnalogSource_Arduino(2),new ModeFilter);
        rangeFinder->set_orientation(-1, 0, 0);
        rangeFinders.push_back(rangeFinder);
    }

    if (_options & opt_rangeFinderLeft) {
        debug->println_P(PSTR("initializing left range finder"));
        RangeFinder * rangeFinder = new AP_RangeFinder_MaxsonarXL(new AP_AnalogSource_Arduino(3),new ModeFilter);
        rangeFinder->set_orientation(0, -1, 0);
        rangeFinders.push_back(rangeFinder);
    }

    if (_options & opt_rangeFinderRight) {
        debug->println_P(PSTR("initializing right range finder"));
        RangeFinder * rangeFinder = new AP_RangeFinder_MaxsonarXL(new AP_AnalogSource_Arduino(4),new ModeFilter);
        rangeFinder->set_orientation(0, 1, 0);
        rangeFinders.push_back(rangeFinder);
    }

    if (_options & opt_rangeFinderUp) {
        debug->println_P(PSTR("initializing up range finder"));
        RangeFinder * rangeFinder = new AP_RangeFinder_MaxsonarXL(new AP_AnalogSource_Arduino(5),new ModeFilter);
        rangeFinder->set_orientation(0, 0, -1);
        rangeFinders.push_back(rangeFinder);
    }

    if (_options & opt_rangeFinderDown) {
        debug->println_P(PSTR("initializing down range finder"));
        RangeFinder * rangeFinder = new AP_RangeFinder_MaxsonarXL(new AP_AnalogSource_Arduino(6),new ModeFilter);
        rangeFinder->set_orientation(0, 0, 1);
        rangeFinders.push_back(rangeFinder);
    }

    /*
     * navigation sensors
     */
    debug->println_P(PSTR("initializing imu"));
    ins = new AP_InertialSensor_Oilpan(adc);
    ins->init(scheduler);
    //ins = new AP_InertialSensor_MPU6000(mpu6000SelectPin)
    debug->println_P(PSTR("initializing ins"));
    imu = new AP_IMU_INS(ins, k_sensorCalib); 
    imu->init(IMU::WARM_START,delay,scheduler);
    debug->println_P(PSTR("setup completed"));
	
	// gcs
    Serial3.begin(telemBaud, 128, 128);
    gcsPort = &Serial3;
    gcsPort->println_P(PSTR("initialized gcs port"));
}

} // namespace apo

// vim:ts=4:sw=4:expandtab
