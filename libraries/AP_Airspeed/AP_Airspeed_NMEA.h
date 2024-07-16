#pragma once

#include "AP_Airspeed_config.h"

#if AP_AIRSPEED_NMEA_ENABLED

#include "AP_Airspeed_Backend.h"
#include <AP_NMEA_Input/AP_NMEA_Input.h>
#include <AP_HAL/AP_HAL.h>

class AP_Airspeed_NMEA : public AP_Airspeed_Backend, AP_NMEA_Input
{
public:

    using AP_Airspeed_Backend::AP_Airspeed_Backend;

    // probe and initialise the sensor
    bool init(void) override;

    // this reads airspeed directly
    bool has_airspeed() override {return true;}

    // read the from the sensor
    bool get_airspeed(float &airspeed) override;

    // return the current temperature in degrees C
    bool get_temperature(float &temperature) override;


private:
    // methods required to be a AP_NMEA_Input
    void handle_decode_success() override;
    bool start_sentence_type(const char *term_type) override;
    bool handle_term(uint8_t term_number, const char *term) override;

    const char * sentence_mtw = "MTW";
    const char * sentence_vhw = "VHW";
    const char *_current_sentence_type;

    // variables for the sentence handlers:
    float _sum = 0.0f;
    uint16_t _count = 0;

    float _speed;                           // speed in m/s
    float _temp;                            // temp in deg c

    // Store the temp ready for a temp request
    float _temp_sum;
    uint16_t _temp_count;

    // store last sent speed and temp as update rate is slow
    float _last_temp;
    float _last_speed;

    // time last message was received
    uint32_t _last_update_ms;
};

#endif  // AP_AIRSPEED_NMEA_ENABLED
