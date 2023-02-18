#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

// note additional vehicle restrictions are made in the .cpp file!
#ifndef AP_AIRSPEED_NMEA_ENABLED
#define AP_AIRSPEED_NMEA_ENABLED AP_AIRSPEED_BACKEND_DEFAULT_ENABLED
#endif

#if AP_AIRSPEED_NMEA_ENABLED

#include "AP_Airspeed_Backend.h"
#include <AP_HAL/AP_HAL.h>

class AP_Airspeed_NMEA : public AP_Airspeed_Backend
{
public:
    AP_Airspeed_NMEA(AP_Airspeed &frontend, uint8_t _instance);

    // probe and initialise the sensor
    bool init(void) override;

    // this reads airspeed directly
    bool has_airspeed() override {return true;}

    // read the from the sensor
    bool get_airspeed(float &airspeed) override;

    // return the current temperature in degrees C
    bool get_temperature(float &temperature) override;


private:
    // pointer to serial uart
    AP_HAL::UARTDriver *_uart = nullptr; 

    // add a single character to the buffer and attempt to decode
    // returns true if a complete sentence was successfully decoded
    // distance should be pulled directly from _distance_m member
    bool decode(char c);

    // decode the just-completed term
    // returns true if new sentence has just passed checksum test and is validated
    bool decode_latest_term();

    // enum for handled messages
    enum sentence_types : uint8_t {
        TPYE_MTW = 0,
        TYPE_VHW,
    };


    // message decoding related members
    char _term[15];                         // buffer for the current term within the current sentence
    uint8_t _term_offset;                   // offset within the _term buffer where the next character should be placed
    uint8_t _term_number;                   // term index within the current sentence
    float _speed;                           // speed in m/s
    float _temp;                            // temp in deg c
    uint8_t _checksum;                      // checksum accumulator
    bool _term_is_checksum;                 // current term is the checksum
    bool _sentence_done;                    // has the current term already been decoded
    bool _sentence_valid;                   // is the decodeing valid so far
    sentence_types _sentence_type;          // the sentence type currently being processed

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
