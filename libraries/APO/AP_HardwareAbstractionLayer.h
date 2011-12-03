/*
 * AP_HardwareAbstractionLayer.h
 *
 *  Created on: Apr 4, 2011
 *
 */

#ifndef AP_HARDWAREABSTRACTIONLAYER_H_
#define AP_HARDWAREABSTRACTIONLAYER_H_

#include "../AP_Common/AP_Vector.h"
#include "../GCS_MAVLink/GCS_MAVLink.h"

class AP_ADC;
class IMU;
class GPS;
class APM_BMP085_Class;
class Compass;
class BetterStream;
class RangeFinder;
class FastSerial;
class AP_IMU_INS;
class AP_InertialSensor;
class APM_RC_Class;
class AP_TimerProcess;
class Arduino_Mega_ISR_Registry;

namespace apo {

class AP_RcChannel;
class AP_CommLink;
class AP_BatteryMonitor;

// enumerations
enum halMode_t {
    MODE_LIVE, MODE_HIL_CNTL,
    /*MODE_HIL_NAV*/
};
enum board_t {
    BOARD_ARDUPILOTMEGA_1280, BOARD_ARDUPILOTMEGA_2560, BOARD_ARDUPILOTMEGA_2
};

class AP_HardwareAbstractionLayer {

public:

    // default ctors on pointers called on pointers here, this
    // allows NULL to be used as a boolean for if the device was
    // initialized
    AP_HardwareAbstractionLayer(halMode_t mode, board_t board, MAV_TYPE vehicle) :
        adc(), gps(), baro(), compass(), rangeFinders(), imu(), batteryMonitor(), 
        radio(), rc(), gcs(),
        hil(), debug(), load(), _mode(mode),
        _board(board), _vehicle(vehicle) {

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
        }
    }

    /**
     * Sensors
     */
    AP_ADC * adc;
    GPS * gps;
    APM_BMP085_Class * baro;
    Compass * compass;
    Vector<RangeFinder *> rangeFinders;
    AP_BatteryMonitor * batteryMonitor;
    AP_IMU_INS * imu;
    AP_InertialSensor * ins;

    /**
     * Scheduler
     */
    AP_TimerProcess * scheduler;
    Arduino_Mega_ISR_Registry * isr_registry;

    /**
     * Actuators
     */
    APM_RC_Class * radio;

    /**
     * Radio Channels
     */
    Vector<AP_RcChannel *> rc;

    /**
     * Communication Channels
     */
    AP_CommLink * gcs;
    AP_CommLink * hil;
    FastSerial * debug;

    /**
     * data
     */
    uint8_t load;

    /**
     * settings
     */
    uint8_t slideSwitchPin;
    uint8_t pushButtonPin;
    uint8_t aLedPin;
    uint8_t bLedPin;
    uint8_t cLedPin;
    uint16_t eepromMaxAddr;

    // accessors
    halMode_t getMode() {
        return _mode;
    }
    board_t getBoard() {
        return _board;
    }
    MAV_TYPE getVehicle() {
        return _vehicle;
    }

private:

    // enumerations
    halMode_t _mode;
    board_t _board;
    MAV_TYPE _vehicle;
};

} // namespace apo

#endif /* AP_HARDWAREABSTRACTIONLAYER_H_ */
// vim:ts=4:sw=4:expandtab
