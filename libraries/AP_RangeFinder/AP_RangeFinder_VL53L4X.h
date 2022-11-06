#pragma once

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"

#ifndef AP_RANGEFINDER_VL53L4X_ENABLED
#define AP_RANGEFINDER_VL53L4X_ENABLED AP_RANGEFINDER_BACKEND_DEFAULT_ENABLED
#endif

#if AP_RANGEFINDER_VL53L4X_ENABLED

#include <AP_HAL/I2CDevice.h>

class AP_RangeFinder_VL53L4X : public AP_RangeFinder_Backend
{

public:
    // static detection function
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev);

    // update state
    void update(void) override;

protected:

    virtual MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override
    {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:
    enum DeviceError : uint8_t {
        NOUPDATE                    = 0,
        VCSELCONTINUITYTESTFAILURE  = 1,
        VCSELWATCHDOGTESTFAILURE    = 2,
        NOVHVVALUEFOUND             = 3,
        MSRCNOTARGET                = 4,
        RANGEPHASECHECK             = 5,
        SIGMATHRESHOLDCHECK         = 6,
        PHASECONSISTENCY            = 7,
        MINCLIP                     = 8,
        RANGECOMPLETE               = 9,
        ALGOUNDERFLOW               = 10,
        ALGOOVERFLOW                = 11,
        RANGEIGNORETHRESHOLD        = 12,
        USERROICLIP                 = 13,
        REFSPADCHARNOTENOUGHDPADS   = 14,
        REFSPADCHARMORETHANTARGET   = 15,
        REFSPADCHARLESSTHANTARGET   = 16,
        MULTCLIPFAIL                = 17,
        GPHSTREAMCOUNT0READY        = 18,
        RANGECOMPLETE_NO_WRAP_CHECK = 19,
        EVENTCONSISTENCY            = 20,
        MINSIGNALEVENTCHECK         = 21,
        RANGECOMPLETE_MERGED_PULSE  = 22,
    };

    enum regAddr : uint16_t {
        SOFT_RESET = 0x0000,
        I2C_SLAVE_DEVICE_ADDRESS = 0x0001,
        VHV_CONFIG_TIMEOUT_MACROP_LOOP_BOUND = 0x0008,
        XTALK_PLANE_OFFSET_KCPS = 0x0016,
        XTALK_X_PLANE_GRADIENT_KCPS = 0x0018,
        XTALK_Y_PLANE_GRADIENT_KCPS = 0x001A,
        RANGE_OFFSET_MM = 0x001E,
        INNER_OFFSET_MM = 0x0020,
        OUTER_OFFSET_MM = 0x0022,
        I2C_FAST_MODE_PLUS = 0x002D,
        GPIO_HV_MUX_CTRL = 0x0030,
        GPIO_TIO_HV_STATUS = 0x0031,
        SYSTEM_INTERRUPT = 0x0046,
        RANGE_CONFIG_A = 0x005E,
        RANGE_CONFIG_B = 0x0061,
        RANGE_CONFIG_SIGMA_THRESH = 0x0064,
        MIN_COUNT_RATE_RTN_LIMIT_MCPS = 0x0066,
        INTERMEASUREMENT_MS = 0x006C,
        THRESH_HIGH = 0x0072,
        THRESH_LOW = 0x0074,
        SYSTEM_INTERRUPT_CLEAR = 0x0086,
        SYSTEM_START = 0x0087,
        RESULT_RANGE_STATUS = 0x0089,
        RESULT_SPAD_NB = 0x008C,
        RESULT_SIGNAL_RATE = 0x008E,
        RESULT_AMBIENT_RATE = 0x0090,
        RESULT_SIGMA = 0x0092,
        RESULT_DISTANCE = 0x0096,
        RESULT_OSC_CALIBRATE_VAL = 0x00DE,
        FIRMWARE_SYSTEM_STATUS = 0x00E5,
        IDENTIFICATION_MODEL_ID = 0x010F,
        DSS_CONFIG_TARGET_TOTAL_RATE_MCPS = 0x0024,
        OSC_MEASURED_FAST_OSC_FREQUENCY = 0x0006,
        VHV_CONFIG_INIT = 0x000B,
    };

    // constructor
    AP_RangeFinder_VL53L4X(RangeFinder::RangeFinder_State &_state, AP_RangeFinder_Params &_params, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    bool init();
    void timer();

    // check sensor ID
    bool check_id(void);

    // get a reading
    bool get_reading(uint16_t &reading_cm);
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev;

    uint16_t osc_calibrate_val;
    uint32_t sum_mm;
    uint32_t counter;

    bool read_register(uint16_t reg, uint8_t &value) WARN_IF_UNUSED;
    bool read_register16(uint16_t reg, uint16_t &value) WARN_IF_UNUSED;
    bool write_register(uint16_t reg, uint8_t value) WARN_IF_UNUSED;
    bool write_register16(uint16_t reg, uint16_t value) WARN_IF_UNUSED;
    bool write_register32(uint16_t reg, uint32_t value) WARN_IF_UNUSED;
    bool data_ready(void);
    bool reset(void) WARN_IF_UNUSED;
    bool set_timing_budget(uint32_t budget_ms) WARN_IF_UNUSED;
    bool start_continuous() WARN_IF_UNUSED;
    bool set_inter_measurement(uint32_t period_ms) WARN_IF_UNUSED;
    uint32_t calc_macro_period(uint16_t osc_freq) const;
    bool interrupt_polarity(uint8_t *value);
    bool wait_for_boot(void);
};

#endif  // AP_RANGEFINDER_VL53L4X_ENABLED
