#pragma once

#include "OpticalFlow.h"
#include <AP_HAL/utility/OwnPtr.h>

class AP_OpticalFlow_Pixart : public OpticalFlow_backend
{
public:
    /// constructor
    AP_OpticalFlow_Pixart(OpticalFlow &_frontend);

    // init - initialise the sensor
    void init() override {}

    // update - read latest values from sensor and fill in x,y and totals.
    void update(void) override;

    // detect if the sensor is available
    static AP_OpticalFlow_Pixart *detect(OpticalFlow &_frontend);

private:
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev;

    struct RegData {
        uint8_t reg;
        uint8_t value;
    };

    struct PACKED MotionBurst {
        uint8_t motion;
        uint8_t observation;
        int16_t delta_x;
        int16_t delta_y;
        uint8_t squal;
        uint8_t rawdata_sum;
        uint8_t max_raw;
        uint8_t min_raw;
        uint8_t shutter_upper;
        uint8_t shutter_lower;
    } burst;

    struct {
        Vector2l sum;
        uint32_t last_frame_us;
        uint32_t sum_us;
        Vector2f gyro;
    } integral;
    
    static const uint8_t srom_data[];
    static const uint8_t srom_id;
    static const RegData init_data[];
    const float flow_pixel_scaling = 1.26e-3;

    // setup sensor
    bool setup_sensor(void);
    
    void reg_write(uint8_t reg, uint8_t value);
    uint8_t reg_read(uint8_t reg);
    int16_t reg_read16s(uint8_t reg);
    uint16_t reg_read16u(uint8_t reg);

    void srom_download(void);
    void load_configuration(void);

    void timer(void);
    void motion_burst(void);

    int32_t sum_x;
    int32_t sum_y;
    uint32_t last_print_ms;
    uint32_t last_burst_us;
    uint32_t last_update_ms;
};
