/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AP_Compass_AK8963_H
#define AP_Compass_AK8963_H

#include <AP_HAL.h>
#include "../AP_Common/AP_Common.h"
#include "../AP_Math/AP_Math.h"

#include "Compass.h"
#include "AP_Compass_Backend.h"

class AP_AK8963_SerialBus
{
public:
    virtual void register_read(uint8_t address, uint8_t *value, uint8_t count) = 0;
    uint8_t register_read(uint8_t address) {
        uint8_t reg;
        register_read(address, &reg, 1);
        return reg;
    }
    virtual void register_write(uint8_t address, uint8_t value) = 0;
    virtual AP_HAL::Semaphore* get_semaphore() = 0;
    virtual bool start_conversion() = 0;
    virtual bool configure() = 0;
    virtual bool read_raw(float &mag_x, float &mag_y, float &mag_z) = 0;
};

class AP_Compass_AK8963 : public AP_Compass_Backend
{
public:
    AP_Compass_AK8963(Compass &compass, AP_AK8963_SerialBus *bus);

    static AP_Compass_Backend *detect(Compass &compass);

    bool        init(void);
    void        read(void);
    void        accumulate(void);

private:
    typedef enum
    {
        STATE_UNKNOWN,
        STATE_CONVERSION,
        STATE_SAMPLE,
        STATE_ERROR
    } state_t;

    bool _reset();
    bool _configure();
    bool _check_id();
    bool _calibrate();

    void _update();
    bool _collect_samples();
    void _dump_registers();
    bool _sem_take_blocking();
    bool _sem_take_nonblocking();
    bool _sem_give();

    state_t             _state;

    float               _magnetometer_ASA[3] {0, 0, 0};
    float               _mag_x;
    float               _mag_y;
    float               _mag_z;
    uint8_t             _compass_instance;

    float               _mag_x_accum;
    float               _mag_y_accum;
    float               _mag_z_accum;
    uint32_t            _accum_count;

    bool                _initialized;
    uint8_t             _magnetometer_adc_resolution;
    uint32_t            _last_update_timestamp;
    uint32_t            _last_accum_time;

    AP_AK8963_SerialBus *_bus;
    AP_HAL::Semaphore *_bus_sem;
};

class AP_AK8963_SerialBus_MPU9250: public AP_AK8963_SerialBus
{
public:
    AP_AK8963_SerialBus_MPU9250();
    void register_read(uint8_t address, uint8_t *value, uint8_t count);
    void register_write(uint8_t address, uint8_t value);
    AP_HAL::Semaphore* get_semaphore();
    bool start_conversion();
    bool configure();
    bool read_raw(float &mag_x, float &mag_y, float &mag_z);
private:
    void _read(uint8_t address, uint8_t *value, uint32_t count);
    void _write(uint8_t address, const uint8_t *value,  uint32_t count);
    void _write(uint8_t address, const uint8_t value) {
        _write(address, &value, 1);
    }
    AP_HAL::SPIDeviceDriver *_spi;
    AP_HAL::Semaphore *_spi_sem;
};

#endif
