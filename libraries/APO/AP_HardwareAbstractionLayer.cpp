/*
 * AP_HardwareAbstractionLayer.cpp
 *
 *  Created on: Apr 30, 2011
 *      Author: jgoppert
 */


// Libraries
#include <FastSerial.h>
#include <Arduino_Mega_ISR_Registry.h>
#include "AP_HardwareAbstractionLayer.h"
#include <AP_ADC.h>
#include <APM_RC.h>
#include <AP_AnalogSource.h>
#include <AP_PeriodicProcess.h>

namespace apo {

// default ctors on pointers called on pointers here, this
// allows NULL to be used as a boolean for if the device was
// initialized
AP_HardwareAbstractionLayer::AP_HardwareAbstractionLayer(halMode_t mode, board_t board, MAV_TYPE vehicle) :
    adc(), gps(), baro(), compass(), rangeFinders(), imu(), batteryMonitor(), 
    radio(), rc(), gcs(),
    hil(), debug(), load(), _mode(mode),
    _board(board), _vehicle(vehicle) {


    AP_Var::load_all();

    /*
     * Board specific hardware initialization
     */
    if (board == BOARD_ARDUPILOTMEGA_1280) {
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
        adc = new AP_ADC_ADS7844;
        radio = new APM_RC_APM1;
    } else if (board == BOARD_ARDUPILOTMEGA_2560) {
        slideSwitchPin = 40;
        pushButtonPin = 41;
        aLedPin = 37;
        bLedPin = 36;
        cLedPin = 35;
        eepromMaxAddr = 2048;
        pinMode(aLedPin, OUTPUT); //  extra led
        pinMode(bLedPin, OUTPUT); //  imu ledclass AP_CommLink;
        pinMode(cLedPin, OUTPUT); //  gps led
        pinMode(slideSwitchPin, INPUT);
        pinMode(pushButtonPin, INPUT);
        DDRL |= B00000100; // set port L, pint 2 to output for the relay
        adc = new AP_ADC_ADS7844;
        radio = new APM_RC_APM1;
    } else if (board == BOARD_ARDUPILOTMEGA_2) {
        slideSwitchPin = 40;
        pushButtonPin = 41;
        aLedPin = 37;
        bLedPin = 36;
        cLedPin = 35;
        eepromMaxAddr = 2048;
        pinMode(aLedPin, OUTPUT); //  extra led
        pinMode(bLedPin, OUTPUT); //  imu ledclass AP_CommLink;
        pinMode(cLedPin, OUTPUT); //  gps led
        pinMode(slideSwitchPin, INPUT);
        pinMode(pushButtonPin, INPUT);
        DDRL |= B00000100; // set port L, pint 2 to output for the relay
        /// FIXME adc = new ?
        adc = new AP_ADC_ADS7844;
        radio = new APM_RC_APM2;
    }

    // isr
    isr_registry = new Arduino_Mega_ISR_Registry;

    // initialize radio
    radio->Init(isr_registry);

    // initialize scheduler
    scheduler = new AP_TimerProcess;
    scheduler->init(isr_registry);

    // initialize the adc
    adc->Init(scheduler);
}

} // apo

// vim:ts=4:sw=4:expandtab
