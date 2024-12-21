#pragma once

#include "AP_Proximity_config.h"

#if AP_PROXIMITY_LIGHTWARE_SF45B_SERIAL_ENABLED

#include "AP_Proximity_LightWareSerial.h"
#include "AP_Proximity_LightWareSF45B.h"

class AP_Proximity_LightWareSF45B_Serial : public AP_Proximity_LightWareSerial, public AP_Proximity_LightWareSF45B
{

public:
    // constructor
    AP_Proximity_LightWareSF45B_Serial(AP_Proximity &_frontend,
                                AP_Proximity::Proximity_State &_state,
                                AP_Proximity_Params& _params,
                                uint8_t serial_instance) :
            AP_Proximity_LightWareSerial(_frontend, _state, _params, serial_instance) {}

    uint16_t rxspace() const override {
        return 1280;
    };

    // update state
    void update(void) override;

    // get maximum and minimum distances (in meters) of sensor
    float distance_max() const override { return 50.0f; }
    float distance_min() const override { return 0.20f; }

private:

    // initialise sensor
    void initialise();

    // request start of streaming of distances
    void request_stream_start();

    // check and process replies from sensor
    void process_replies();

    // process the latest message held in the msg structure
    void process_message();

    // state of sensor
    struct {
        uint8_t update_rate;        // sensor reported update rate enum from UPDATE_RATE message
        uint32_t streaming_fields;  // sensor reported bitmask of fields sent in DISTANCE_DATA_CM message
        uint32_t stream_data_type;  // sensor reported stream value.  5 if DISTANCE_DATA_CM messages are being streamed
    } _sensor_state;

};

#endif // AP_PROXIMITY_LIGHTWARE_SF45B_SERIAL_ENABLED
