#pragma once

#include "AP_Baro_Backend.h"

#ifndef AP_BARO_ICP101XX_ENABLED
#define AP_BARO_ICP101XX_ENABLED AP_BARO_BACKEND_DEFAULT_ENABLED
#endif

#if AP_BARO_ICP101XX_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Device.h>

class AP_Baro_ICP101XX : public AP_Baro_Backend
{
public:
    void update() override;

    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);
    
private:
    AP_Baro_ICP101XX(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    bool init();
    bool send_cmd16(uint16_t cmd);
    bool read_measure_results(uint8_t *buf, uint8_t len);
    bool read_response(uint16_t cmd, uint8_t *buf, uint8_t len);
    bool send_command(uint16_t cmd);
    bool send_command(uint16_t cmd, uint8_t *data, uint8_t len);
    int8_t cal_crc(uint8_t seed, uint8_t data);
    bool start_measure(uint16_t mode);
    bool read_calibration_data(void);
    void convert_data(uint32_t Praw, uint32_t Traw);
    void calculate_conversion_constants(const float p_Pa[3], const float p_LUT[3],
                                        float &A, float &B, float &C);
    float get_pressure(uint32_t p_LSB, uint32_t T_LSB);
    void timer(void);
    
    // calibration data
    int16_t sensor_constants[4];

    uint8_t instance;

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev;

    // time last read command was sent
    uint32_t last_measure_us;

    // accumulation structure, protected by _sem
    struct {
        float tsum;
        float psum;
        uint32_t count;
    } accum;

    // conversion constants. Thanks to invensense for including python
    // sample code in the datasheet!
    const float p_Pa_calib[3] = {45000.0, 80000.0, 105000.0};
    const float LUT_lower = 3.5 * (1U<<20);
    const float LUT_upper = 11.5 * (1U<<20);
    const float quadr_factor = 1 / 16777216.0;
    const float offst_factor = 2048.0;
    uint32_t measure_interval = 0;
};

#endif  // AP_BARO_ICP101XX_ENABLED