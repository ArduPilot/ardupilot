/*
 * AP_Board.h
 *
 *  Created on: Apr 4, 2011
 *
 */

#ifndef AP_BOARD_H_
#define AP_BOARD_H_

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
class DataFlash_Class;

namespace apo {

class AP_RcChannel;
class AP_CommLink;
class AP_BatteryMonitor;

class AP_Board {

public:

    typedef uint32_t options_t;
    options_t _options;

    // enumerations
    enum mode_e {
        MODE_LIVE, MODE_HIL_CNTL,
        /*MODE_HIL_NAV*/
    };


    enum options_e {
        opt_gps                 = 0<<1,
        opt_baro                = 1<<1,
        opt_compass             = 2<<1,
        opt_batteryMonitor      = 3<<1,
        opt_rangeFinderFront    = 4<<1,
        opt_rangeFinderBack     = 5<<1,
        opt_rangeFinderLeft     = 6<<1,
        opt_rangeFinderRight    = 7<<1,
        opt_rangeFinderUp       = 8<<1,
        opt_rangeFinderDown     = 9<<1,
    };

    // default ctors on pointers called on pointers here, this
    // allows NULL to be used as a boolean for if the device was
    // initialized
    AP_Board(mode_e mode, MAV_TYPE vehicle, options_t options): _mode(mode), _vehicle(vehicle), _options(options) {
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
    FastSerial * gcsPort;
    FastSerial * hilPort;

    /**
     * data
     */
    DataFlash_Class * dataFlash;
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
    mode_e getMode() {
        return _mode;
    }
    MAV_TYPE getVehicle() {
        return _vehicle;
    }

private:

    // enumerations
    mode_e _mode;
    MAV_TYPE _vehicle;
};

} // namespace apo

#endif /* AP_HARDWAREABSTRACTIONLAYER_H_ */
// vim:ts=4:sw=4:expandtab
