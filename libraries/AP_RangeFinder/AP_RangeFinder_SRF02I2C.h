#pragma once

#include "RangeFinder.h"
#include "RangeFinder_Backend.h"
#include <AP_HAL/I2CDevice.h>

class AP_RangeFinder_SRF02I2C : public AP_RangeFinder_Backend
{

public:
    enum SRFXX_I2C {
        SRF02_I2C_DEFAULT_ADDRESS = 0x70,
        SRF02_I2C_CHECK_CODE_DEFAULT = 0x80,
        SRF02_I2C_CHECK_CODE = 0x18,  // Not 0x80. This value is Software version 6.
        SRF02_I2C_RANGING_TIME = 70,  // 70msec
        //
        SRF02_I2C_REG_W_COMMAND  = 0,
        SRF02_I2C_REG_R_SOFTWARE_REVISION = 0,
        SRF02_I2C_REG_R_CHECK_CODE = 1,
        SRF02_I2C_REG_R_RANGE_HIGH_BYTE = 2,
        SRF02_I2C_REG_R_RANGE_LOW_BYTE  = 3,
        SRF02_I2C_REG_R_AUTOTUNE_MINIMUM_HIGH_BYTE = 4,
        SRF02_I2C_REG_R_AUTOTUNE_MINIMUM_LOW_BYTE  = 5,
        SRF02_I2C_CMD_REAL_RANGE_MODE_RESULT_IN_INCHES        = 0x50,
        SRF02_I2C_CMD_REAL_RANGE_MODE_RESULT_IN_CENTIMETERS   = 0x51,
        SRF02_I2C_CMD_REAL_RANGE_MODE_RESULT_IN_MICRO_SECONDS = 0x52,
        SRF02_I2C_CMD_FAKE_RANGING_MODE_RESULT_IN_INCHES        = 0x56,
        SRF02_I2C_CMD_FAKE_RANGING_MODE_RESULT_IN_CENTIMETERS   = 0x57,
        SRF02_I2C_CMD_FAKE_RANGING_MODE_RESULT_IN_MICRO_SECONDS = 0x58,
        SRF02_I2C_CMD_TRANSMIT_AN_8_CYCLE_40KHZ_BURST = 0x5c,
        SRF02_I2C_CMD_FORCE_AUTOTUNE_RESTART = 0x60,
        SRF02_I2C_CMD_1ST_IN_SEQUENCE_TO_CHANGE_I2C_ADDRESS = 0xa0,
        SRF02_I2C_CMD_3RD_IN_SEQUENCE_TO_CHANGE_I2C_ADDRESS = 0xa5,
        SRF02_I2C_CMD_2ND_IN_SEQUENCE_TO_CHANGE_I2C_ADDRESS = 0Xaa,
    };

    // static detection function
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state,
            AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    // update state
    void update(void);

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_ULTRASOUND;
    }

private:
    // constructor
    AP_RangeFinder_SRF02I2C(RangeFinder::RangeFinder_State &_state, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    bool _init();
    void _timer();

    uint16_t _distance;
    bool _new_distance;
    uint16_t _autotuneMinimum;

    // get a reading
    bool _getMeasuredDistance(uint16_t &outMeasuredDistance);
    bool _startMeasurement();
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
};
