#pragma once

#include "AP_RangeFinder_config.h"

#if AP_RANGEFINDER_LIGHTWARE_GRF_I2C_ENABLED

#include "AP_RangeFinder.h"
#include "AP_RangeFinder_Backend_I2C.h"
#include "AP_RangeFinder_LightWare_GRF_Common.h"

#define AP_RANGEFINDER_LIGHTWARE_GRF_I2C_DEFAULT_ADDR 0x66

class AP_RangeFinder_LightWare_GRF_I2C : public AP_RangeFinder_Backend_I2C
{
public:
    static AP_RangeFinder_Backend *detect(RangeFinder::RangeFinder_State &_state,
                                          AP_RangeFinder_Params &_params,
                                          class AP_HAL::I2CDevice &dev)
    {
        return configure(NEW_NOTHROW AP_RangeFinder_LightWare_GRF_I2C(_state, _params, dev));
    }

    static const struct AP_Param::GroupInfo var_info[];

    void update() override;

protected:
    MAV_DISTANCE_SENSOR _get_mav_distance_sensor_type() const override
    {
        return MAV_DISTANCE_SENSOR_LASER;
    }

private:
    AP_RangeFinder_LightWare_GRF_I2C(RangeFinder::RangeFinder_State &_state,
                                     AP_RangeFinder_Params &_params,
                                     AP_HAL::I2CDevice &_dev);

    using MessageID = AP_RangeFinder_LightWare_GRF_Common::MessageID;

    // Usual config steps
    enum class ConfigStep : uint8_t {
        WRITE_UPDATE_RATE,
        VERIFY_UPDATE_RATE,
        WRITE_DISTANCE_OUTPUT,
        VERIFY_DISTANCE_OUTPUT,
        DONE,
    };

    bool init() override;
    void timer();

    // Write/read a register on the sensor.
    bool write_command(MessageID msgid, const uint8_t *payload, uint16_t payload_len);
    bool read_payload(MessageID msgid, uint8_t *payload_buf, uint16_t payload_len);

    // Apply one step of the configuration.
    void run_config_step();

    AP_RangeFinder_LightWare_GRF_Common _common;

    struct {
        float sum_m;
        uint16_t count;
    } _accum;

    // config runs from the timer callback so init() stays quick.
    ConfigStep _config_step = ConfigStep::WRITE_UPDATE_RATE;
    uint8_t    _config_retry = 0;
};

#endif  // AP_RANGEFINDER_LIGHTWARE_GRF_I2C_ENABLED
