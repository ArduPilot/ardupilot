#pragma once

#include "AP_Frsky_SPort.h"
#include <AP_RCTelemetry/AP_RCTelemetry.h>

#include "AP_Frsky_SPortParser.h"
#include "AP_Frsky_MAVlite.h"

#include "AP_Frsky_Telem.h"

#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
#include "AP_Frsky_MAVlite_SPortToMAVlite.h"
#include "AP_Frsky_MAVlite_MAVliteToSPort.h"
#include "AP_Frsky_MAVliteMsgHandler.h"

#define SPORT_TX_PACKET_DUPLICATES          1   // number of duplicates packets we send (fport only)
#endif

class AP_Frsky_SPort_Passthrough : public AP_Frsky_SPort, public AP_RCTelemetry
{
public:

    AP_Frsky_SPort_Passthrough(AP_HAL::UARTDriver *port, bool use_external_data, AP_Frsky_Parameters *&frsky_parameters) :
        AP_Frsky_SPort(port),
        AP_RCTelemetry(WFQ_LAST_ITEM),
        _use_external_data(use_external_data),
        _frsky_parameters(frsky_parameters)
    {
        singleton = this;
    }

    static AP_Frsky_SPort_Passthrough *get_singleton(void) {
        return singleton;
    }

    bool init() override;
    bool init_serial_port() override;

    // passthrough WFQ scheduler
    bool is_packet_ready(uint8_t idx, bool queue_empty) override;
    void process_packet(uint8_t idx) override;
    void adjust_packet_weight(bool queue_empty) override;
    // setup ready for passthrough operation
    void setup_wfq_scheduler(void) override;

    bool get_next_msg_chunk(void) override;

    bool get_telem_data(sport_packet_t* packet_array, uint8_t &packet_count, const uint8_t max_size) override;
#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    bool set_telem_data(const uint8_t frame, const uint16_t appid, const uint32_t data) override;
#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL

    void queue_text_message(MAV_SEVERITY severity, const char *text) override
    {
        AP_RCTelemetry::queue_message(severity, text);
    }

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
        PARAM =         10, // 0x5007 parameters
        RPM =           11, // 0x500A rpm sensors 1 and 2
        UDATA =         12, // user data
#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
        MAV =           13,  // mavlite
#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
        TERRAIN =       14, // 0x500B terrain data
        WIND =          15, // 0x500C wind data
        WAYPOINT =      16, // 0x500D waypoint data
        WFQ_LAST_ITEM       // must be last
    };

protected:

    void send() override;

private:

    AP_Frsky_Parameters *&_frsky_parameters;

    // methods to convert flight controller data to FrSky SPort Passthrough (OpenTX) format
    uint32_t calc_param(void);
    uint32_t calc_velandyaw(void);

    // use_external_data is set when this library will
    // be providing data to another transport, such as FPort
    bool _use_external_data;

    struct {
        sport_packet_t packet;
        bool pending;
    } external_data;

    struct {
        uint32_t chunk; // a "chunk" (four characters/bytes) at a time of the queued message to be sent
        uint8_t repeats; // send each message "chunk" 3 times to make sure the entire messsage gets through without getting cut
        uint8_t char_index; // index of which character to get in the message
    } _msg_chunk;

    // passthrough default sensor id
    uint8_t downlink_sensor_id = 0x1B;

#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    // bidirectional sport telemetry
    struct {
        uint8_t uplink_sensor_id = 0x0D;
        uint8_t downlink1_sensor_id = 0x34;
        uint8_t downlink2_sensor_id = 0x67;
        uint8_t tx_packet_duplicates;
        ObjectBuffer_TS<AP_Frsky_SPort::sport_packet_t> rx_packet_queue{SPORT_PACKET_QUEUE_LENGTH};
        ObjectBuffer_TS<AP_Frsky_SPort::sport_packet_t> tx_packet_queue{SPORT_PACKET_QUEUE_LENGTH};
    } _SPort_bidir;

    AP_Frsky_SPortParser _sport_handler;
    AP_Frsky_MAVlite_SPortToMAVlite sport_to_mavlite;
    AP_Frsky_MAVlite_MAVliteToSPort mavlite_to_sport;

    void set_sensor_id(AP_Int8 idx, uint8_t &sensor);
    // tx/rx sport packet processing
    void queue_rx_packet(const AP_Frsky_SPort::sport_packet_t sp);
    void process_rx_queue(void);
    void process_tx_queue(void);

    // create an object to handle incoming mavlite messages; a
    // callback method is provided to allow the handler to send responses
    bool send_message(const AP_Frsky_MAVlite_Message &txmsg);
    AP_Frsky_MAVliteMsgHandler mavlite{FUNCTOR_BIND_MEMBER(&AP_Frsky_SPort_Passthrough::send_message, bool, const AP_Frsky_MAVlite_Message &)};
#endif

    void send_sport_frame(uint8_t frame, uint16_t appid, uint32_t data);

    // true if we need to respond to the last polling byte
    bool is_passthrough_byte(const uint8_t byte) const;

    uint8_t _paramID;

    uint32_t calc_gps_status(void);

    static AP_Frsky_SPort_Passthrough *singleton;
};

namespace AP {
    AP_Frsky_SPort_Passthrough *frsky_passthrough_telem();
};
