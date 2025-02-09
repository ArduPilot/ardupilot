#pragma once

#include "AP_Baro_Backend.h"

#if AP_BARO_SPL06_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>
#include <AP_HAL/utility/OwnPtr.h>

#ifndef HAL_BARO_SPL06_I2C_ADDR
 #define HAL_BARO_SPL06_I2C_ADDR  (0x76)
#endif
#ifndef HAL_BARO_SPL06_I2C_ADDR2
 #define HAL_BARO_SPL06_I2C_ADDR2 (0x77)
#endif


#define SPL06_STATE_IDLE    0x0
#define SPL06_CHIP_READY    0x1
#define SPL06_COEFFS_READ   0x2
#define SPL06_TEMP_SENT     0x4
#define SPL06_PRESS_SENT    0x8
#define SPL06_TEMP_READ     0x16
#define SPL06_PRESS_READ    0x32


class AP_Baro_SPL06 : public AP_Baro_Backend
{
public:
	enum class Type {
		UNKNOWN,
		SPL06,
		SPA06,
	};
    AP_Baro_SPL06(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);

    /* AP_Baro public interface: */
    void update() override;

    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);

private:

    bool _init(void);
    void _timer(void);
    void _update_temperature(int32_t);
    void _update_pressure(int32_t);

    int32_t raw_value_scale_factor(uint8_t);
    bool get_calib_coeefs(void);

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    int16_t _timer_counter;
    uint8_t _instance;
    float _temp_raw;
    float _pressure_sum;
    uint32_t _pressure_count;
    float _temperature;
    
    uint8_t _state = 0; // SPL06 driver state register - not used yet

    // Internal calibration registers
    int32_t _c00, _c10;
    int16_t _c0, _c1, _c01, _c11, _c20, _c21, _c30, _c31, _c40;

    Type type;
};

#endif  // AP_BARO_SPL06_ENABLED
