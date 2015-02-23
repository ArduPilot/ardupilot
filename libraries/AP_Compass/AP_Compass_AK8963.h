/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#ifndef AP_Compass_AK8963_H
#define AP_Compass_AK8963_H

#include <AP_HAL.h>
#include "../AP_Common/AP_Common.h"
#include "../AP_Math/AP_Math.h"

#include "Compass.h"
#include "AP_Compass_Backend.h"

class AK8963_Backend
{
    public:
        virtual ~AK8963_Backend() {}
        virtual void read(uint8_t address, uint8_t *buf, uint32_t count) = 0;
        virtual void write(uint8_t address, const uint8_t *buf, uint32_t count) = 0;
        virtual bool sem_take_nonblocking() = 0;
        virtual bool sem_take_blocking() = 0;
        virtual bool sem_give() = 0;
        virtual uint8_t read(uint8_t address) 
        {
            uint8_t value;
            read(address, &value, 1);
            return value;
        }

        virtual void write(uint8_t address, uint8_t value)
        {
            write(address, &value, 1);
        }
};

class AP_Compass_AK8963 : public AP_Compass_Backend
{
public:
    AP_Compass_AK8963(Compass &compass);

    bool        init(void);
    void        read(void);
    void        accumulate(void);

protected:
    AK8963_Backend      *_backend;  // Not to be confused with Compass (frontend) "_backends" attribute.
    float               magnetometer_ASA[3];
    float               _mag_x;
    float               _mag_y;
    float               _mag_z;
    uint8_t             _compass_instance;

    virtual bool read_raw() = 0;

private:
    typedef enum 
    {
        CONVERSION,
        SAMPLE,
        ERROR
    } state_t;

    virtual bool        _backend_init() = 0;
    virtual void        _register_read(uint8_t address, uint8_t count, uint8_t *value) = 0;
    virtual void        _register_write(uint8_t address, uint8_t value) = 0;
    virtual void        _backend_reset() = 0;
    virtual uint8_t     _read_id() = 0;
    virtual void        _dump_registers() {}

    bool                _register_read(uint8_t address, uint8_t *value);
    bool                _calibrate();
    bool                _self_test();
    void                _update();
    void                _start_conversion();
    void                _collect_samples();

    float               _mag_x_accum;
    float               _mag_y_accum;
    float               _mag_z_accum;
    uint32_t            _accum_count;

    bool                _initialised;
    state_t             _state;
    uint8_t             _magnetometer_adc_resolution;
    uint32_t            _last_update_timestamp;
    uint32_t            _last_accum_time;
};

class AK8963_MPU9250_SPI_Backend: public AK8963_Backend
{
    public:
        AK8963_MPU9250_SPI_Backend();
        void read(uint8_t address, uint8_t *buf, uint32_t count);
        void write(uint8_t address, const uint8_t *buf, uint32_t count);
        bool sem_take_nonblocking();
        bool sem_take_blocking();
        bool sem_give();
        ~AK8963_MPU9250_SPI_Backend() {}

    private:
        AP_HAL::SPIDeviceDriver *_spi;
        AP_HAL::Semaphore *_spi_sem;
};

class AP_Compass_AK8963_MPU9250: public AP_Compass_AK8963
{
    public:
        AP_Compass_AK8963_MPU9250(Compass &compass);
        ~AP_Compass_AK8963_MPU9250() {}
        bool init();

    // detect the sensor
    static AP_Compass_Backend *detect(Compass &compass);

    private:
        bool       _backend_init();
        void       _backend_reset();
        void       _register_read(uint8_t address, uint8_t count, uint8_t *value);
        void       _register_write(uint8_t address, uint8_t value);
        void       _dump_registers();
        bool       read_raw();
        uint8_t    _read_id();
};

#endif
