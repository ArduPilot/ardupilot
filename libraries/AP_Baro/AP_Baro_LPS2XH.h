#pragma once

#include "AP_Baro_Backend.h"

#ifndef AP_BARO_LPS2XH_ENABLED
#define AP_BARO_LPS2XH_ENABLED AP_BARO_BACKEND_DEFAULT_ENABLED
#endif

#if AP_BARO_LPS2XH_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/Device.h>
#include <AP_HAL/utility/OwnPtr.h>

#define HAL_BARO_LPS25H_I2C_BUS 0

#ifndef HAL_BARO_LPS25H_I2C_ADDR
# define HAL_BARO_LPS25H_I2C_ADDR 0x5D
#endif


class AP_Baro_LPS2XH : public AP_Baro_Backend
{
public:
    enum LPS2XH_TYPE {
        BARO_LPS22H = 0,
        BARO_LPS25H = 1,
    };

    AP_Baro_LPS2XH(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);

    /* AP_Baro public interface: */
    void update() override;

    static AP_Baro_Backend *probe(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev);
    static AP_Baro_Backend *probe_InvensenseIMU(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev, uint8_t imu_address);

private:
    virtual ~AP_Baro_LPS2XH(void) {};

    bool _init(void);
    void _timer(void);
    void _update_temperature(void);
    void _update_pressure(void);
    bool _imu_i2c_init(uint8_t imu_address);

    bool _check_whoami(void);

    AP_HAL::OwnPtr<AP_HAL::Device> _dev;

    uint8_t _instance;
    float _pressure_sum;
    uint32_t _pressure_count;
    float _temperature;

    uint32_t CallTime = 0;

    enum LPS2XH_TYPE _lps2xh_type;
};

#endif  // AP_BARO_LPS2XH_ENABLED
