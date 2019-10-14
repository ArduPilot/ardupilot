#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>
#include <AP_HAL/utility/OwnPtr.h>

#include "AP_Baro_Backend.h"

#ifndef HAL_BARO_DPS280_I2C_ADDR
 #define HAL_BARO_DPS280_I2C_ADDR  0x76
#endif
#ifndef HAL_BARO_DPS280_I2C_ADDR2
 #define HAL_BARO_DPS280_I2C_ADDR2 0x77
#endif

class AP_Baro_DPS280 : public AP_Baro_Backend {
public:
    AP_Baro_DPS280(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);

    /* AP_Baro public interface: */
    void update() override;

    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);

private:
    bool init(void);
    bool read_calibration(void);
    void timer(void);
    void calculate_PT(int32_t UT, int32_t UP, float &pressure, float &temperature);

    void fix_config_bits16(int16_t &v, uint8_t bits) const;
    void fix_config_bits32(int32_t &v, uint8_t bits) const;
    void set_config_registers(void);
    void check_health();

    AP_HAL::OwnPtr<AP_HAL::Device> dev;

    uint8_t instance;

    uint32_t count;
    uint8_t err_count;
    float pressure_sum;
    float temperature_sum;
    float last_temperature;
    bool pending_reset;

    struct dps280_cal {
        int16_t C0;  // 12bit
        int16_t C1;  // 12bit
        int32_t C00; // 20bit
        int32_t C10; // 20bit
        int16_t C01; // 16bit
        int16_t C11; // 16bit
        int16_t C20; // 16bit
        int16_t C21; // 16bit
        int16_t C30; // 16bit
        uint8_t temp_source;
    } calibration;
};
