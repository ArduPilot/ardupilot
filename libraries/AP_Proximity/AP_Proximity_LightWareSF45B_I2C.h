#pragma once

#include "AP_Proximity_config.h"

#if AP_PROXIMITY_LIGHTWARE_SF45B_I2C_ENABLED

#include "AP_Proximity_LightWareSF45B.h"

#include <AP_HAL/I2CDevice.h>

static const uint8_t PROXIMITY_SF45B_I2C_ADDRESS = 0x66;

class AP_Proximity_LightWareSF45B_I2C : public AP_Proximity_Backend, public AP_Proximity_LightWareSF45B
{
public:

    // static detection function
    static AP_Proximity_Backend *detect(AP_Proximity &_frontend,
                                          AP_Proximity::Proximity_State &_state,
                                          AP_Proximity_Params& _params,
                                          AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    // update state
    void update(void) override;

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const override { return 50.0f; }
    float distance_min() const override { return 0.20f; }

private:

    // constructor
    AP_Proximity_LightWareSF45B_I2C(AP_Proximity &_frontend, AP_Proximity::Proximity_State &_state, AP_Proximity_Params& _params, AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev);

    // initialise sensor
    bool initialise();

    // timer function
    void timer(void);

    // configure sensor to send distances and servo angle only
    bool configure_sensor();

    // get configuration from sensor
    bool get_configuration();

    // trigger a distance reading
    bool get_reading(float &reading_m, float &ang_deg);

    // process the latest message held in the msg structure
    void process_message(float distance_m, float angle_deg);

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;

    float distance_m;
    float yaw_deg;
    bool new_distance; // true if we have a new distance
};

#endif // AP_PROXIMITY_LIGHTWARE_SF45B_I2C_ENABLED
