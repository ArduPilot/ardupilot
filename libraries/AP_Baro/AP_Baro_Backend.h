#pragma once

#include "AP_Baro.h"

class AP_Baro_Backend
{
public:
    AP_Baro_Backend(AP_Baro &baro);
    virtual ~AP_Baro_Backend(void) {};

    // each driver must provide an update method to copy accumulated
    // data to the frontend
    virtual void update() = 0;

    // accumulate function. This is used for backends that don't use a
    // timer, and need to be called regularly by the main code to
    // trigger them to read the sensor
    virtual void accumulate(void) {}

    void backend_update(uint8_t instance);

    //  Check that the baro valid by using a mean filter.
    // If the value further that filtrer_range from mean value, it is rejected.
    bool pressure_ok(float press);
    uint32_t get_error_count() const { return _error_count; }

#if HAL_MSP_BARO_ENABLED
    virtual void handle_msp(const MSP::msp_baro_data_message_t &pkt) {}
#endif 

#if HAL_EXTERNAL_AHRS_ENABLED
    virtual void handle_external(const AP_ExternalAHRS::baro_data_message_t &pkt) {}
#endif 
    
    /*
      device driver IDs. These are used to fill in the devtype field
      of the device ID, which shows up as BARO_DEVID* parameters to
      users.
     */
    enum DevTypes {
        DEVTYPE_BARO_SITL     = 0x01,
        DEVTYPE_BARO_BMP085   = 0x02,
        DEVTYPE_BARO_BMP280   = 0x03,
        DEVTYPE_BARO_BMP388   = 0x04,
        DEVTYPE_BARO_DPS280   = 0x05,
        DEVTYPE_BARO_DPS310   = 0x06,
        DEVTYPE_BARO_FBM320   = 0x07,
        DEVTYPE_BARO_ICM20789 = 0x08,
        DEVTYPE_BARO_KELLERLD = 0x09,
        DEVTYPE_BARO_LPS2XH   = 0x0A,
        DEVTYPE_BARO_MS5611   = 0x0B,
        DEVTYPE_BARO_SPL06    = 0x0C,
        DEVTYPE_BARO_UAVCAN   = 0x0D,
        DEVTYPE_BARO_MSP      = 0x0E,
        DEVTYPE_BARO_ICP101XX = 0x0F,
    };
    
protected:
    // reference to frontend object
    AP_Baro &_frontend;

    void _copy_to_frontend(uint8_t instance, float pressure, float temperature);

    // semaphore for access to shared frontend data
    HAL_Semaphore _sem;

    virtual void update_healthy_flag(uint8_t instance);

    // mean pressure for range filter
    float _mean_pressure; 
    // number of dropped samples. Not used for now, but can be usable to choose more reliable sensor
    uint32_t _error_count;

    // set bus ID of this instance, for BARO_DEVID parameters
    void set_bus_id(uint8_t instance, uint32_t id) {
        _frontend.sensors[instance].bus_id.set(int32_t(id));
    }
};
