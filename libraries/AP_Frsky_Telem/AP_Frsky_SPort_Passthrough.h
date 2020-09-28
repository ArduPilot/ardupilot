#pragma once

#include "AP_Frsky_SPort.h"
#include <AP_RCTelemetry/AP_RCTelemetry.h>

// for fair scheduler
#define TIME_SLOT_MAX               11

class AP_Frsky_SPort_Passthrough : public AP_Frsky_SPort, public AP_RCTelemetry
{
public:

    AP_Frsky_SPort_Passthrough(AP_HAL::UARTDriver *port, bool use_external_data) :
        AP_Frsky_SPort(port),
        AP_RCTelemetry(TIME_SLOT_MAX),
        _use_external_data(use_external_data)
    { }

    bool init() override;
    bool init_serial_port() override;

    // passthrough WFQ scheduler
    bool is_packet_ready(uint8_t idx, bool queue_empty) override;
    void process_packet(uint8_t idx) override;
    void adjust_packet_weight(bool queue_empty) override;
    // setup ready for passthrough operation
    void setup_wfq_scheduler(void) override;

    bool get_next_msg_chunk(void) override;

    bool get_telem_data(uint8_t &frame, uint16_t &appid, uint32_t &data) override;

    void queue_text_message(MAV_SEVERITY severity, const char *text) override
    {
        AP_RCTelemetry::queue_message(severity, text);
    }


protected:

    void send() override;

private:

    enum PassthroughParam : uint8_t {
        FRAME_TYPE =          1,
        BATT_FS_VOLTAGE =     2,
        BATT_FS_CAPACITY =    3,
        BATT_CAPACITY_1 =     4,
        BATT_CAPACITY_2 =     5
    };

    enum PassthroughPacketType : uint8_t {
        TEXT =          0,  // 0x5000 status text (dynamic)
        ATTITUDE =      1,  // 0x5006 Attitude and range (dynamic)
        GPS_LAT =       2,  // 0x800 GPS lat
        GPS_LON =       3,  // 0x800 GPS lon
        VEL_YAW =       4,  // 0x5005 Vel and Yaw
        AP_STATUS =     5,  // 0x5001 AP status
        GPS_STATUS =    6,  // 0x5002 GPS status
        HOME =          7,  // 0x5004 Home
        BATT_2 =        8,  // 0x5008 Battery 2 status
        BATT_1 =        9,  // 0x5008 Battery 1 status
        PARAM =         10  // 0x5007 parameters
    };

    // methods to convert flight controller data to FrSky SPort Passthrough (OpenTX) format
    uint32_t calc_param(void);
    uint32_t calc_batt(uint8_t instance);
    uint32_t calc_ap_status(void);
    uint32_t calc_home(void);
    uint32_t calc_velandyaw(void);
    uint32_t calc_attiandrng(void);

    // use_external_data is set when this library will
    // be providing data to another transport, such as FPort
    bool _use_external_data;
    struct {
        uint8_t frame;
        uint16_t appid;
        uint32_t data;
        bool pending;
    } external_data;

    struct {
        uint32_t chunk; // a "chunk" (four characters/bytes) at a time of the queued message to be sent
        uint8_t repeats; // send each message "chunk" 3 times to make sure the entire messsage gets through without getting cut
        uint8_t char_index; // index of which character to get in the message
    } _msg_chunk;

    void send_sport_frame(uint8_t frame, uint16_t appid, uint32_t data);

    uint8_t _paramID;

    uint32_t calc_gps_status(void);
    uint16_t prep_number(int32_t number, uint8_t digits, uint8_t power);
};
