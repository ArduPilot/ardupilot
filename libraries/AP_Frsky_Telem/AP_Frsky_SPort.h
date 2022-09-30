#pragma once

#include "AP_Frsky_Backend.h"

class AP_Frsky_SPort : public AP_Frsky_Backend
{

public:

    AP_Frsky_SPort(AP_HAL::UARTDriver *port) : AP_Frsky_Backend(port) {
        singleton = this;
    }

    /* Do not allow copies */
    CLASS_NO_COPY(AP_Frsky_SPort);

    void send() override;
    // send an sport packet by responding to the specified polled sensor
    bool sport_telemetry_push(const uint8_t sensor, const uint8_t frame, const uint16_t appid, const int32_t data);
    // utility method to pack numbers in a compact size
    uint16_t prep_number(int32_t const number, const uint8_t digits, const uint8_t power);

    static AP_Frsky_SPort *get_singleton(void) {
        return singleton;
    }

protected:

    void send_sport_frame(uint8_t frame, uint16_t appid, uint32_t data);

    struct {
        bool send_latitude;
        bool send_airspeed;     // toggles 0x5005 between airspeed and groundspeed
        uint32_t gps_lng_sample;
        uint8_t new_byte;
    } _passthrough;

    uint32_t calc_gps_latlng(bool &send_latitude);
    static uint8_t calc_sensor_id(const uint8_t physical_id);

    struct {
        sport_packet_t packet;
        bool pending = false;
        HAL_Semaphore sem;
    } _sport_push_buffer;

private:

    struct {
        bool sport_status;
        bool gps_refresh;
        bool vario_refresh;
        uint8_t fas_call;
        uint8_t gps_call;
        uint8_t vario_call;
        uint8_t various_call;
        uint8_t rpm_call;
    } _SPort;

    static AP_Frsky_SPort *singleton;

};

namespace AP {
    AP_Frsky_SPort *frsky_sport();
};
