#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_VL53L3CX_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend.h"
#include "AP_RangeFinder_VL53L3CX/vl53lx_class.h"

#include <AP_HAL/I2CDevice.h>

#define VL53L3CX_I2C_ADDR_DEFAULT 0x29

class AP_RangeFinder_VL53L3CX : public AP_RangeFinder_Backend
{
public:
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state,
                                          AP_RangeFinder_Params &_params,
                                          AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev,
                                          uint8_t address);

    static uint8_t probe_address(uint8_t address);

    void update(void) override;

protected:
    /**
     * Return the MAVLink distance sensor type reported by this backend.
     *
     * @return MAVLink laser distance sensor type.
     */
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:
    AP_RangeFinder_VL53L3CX(RangeFinder::RangeFinder_State &_state,
                            AP_RangeFinder_Params &_params,
                            AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    __INITFUNC__ bool init(uint8_t address);
    bool check_id(void);
    bool read_register(uint16_t reg, uint8_t &value) WARN_IF_UNUSED;
    bool write_register(uint16_t reg, uint8_t value) WARN_IF_UNUSED;
    bool reset(void) WARN_IF_UNUSED;
    bool get_reading(uint16_t &reading_mm) WARN_IF_UNUSED;
    void timer(void);

    static bool prepare_xshut(bool select_down_xshut);
    static bool range_status_ok(uint8_t status);

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev;
    VL53LX sensor;
    uint8_t i2c_address;
    uint32_t sum_mm;
    uint32_t counter;
};

#endif  // AP_RANGEFINDER_VL53L3CX_ENABLED
