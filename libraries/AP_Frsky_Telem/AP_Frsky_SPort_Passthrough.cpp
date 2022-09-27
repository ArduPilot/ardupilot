#include "AP_Frsky_SPort_Passthrough.h"

#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
#include "AP_Frsky_MAVlite.h"
#include "AP_Frsky_Parameters.h"
#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL

extern const AP_HAL::HAL& hal;

AP_Frsky_SPort_Passthrough *AP_Frsky_SPort_Passthrough::singleton;

bool AP_Frsky_SPort_Passthrough::init()
{
    if (!AP_RCTelemetry::init()) {
        return false;
    }
    return AP_Frsky_SPort::init();
}

bool AP_Frsky_SPort_Passthrough::init_serial_port()
{
    if (_use_external_data) {
        return true;
    }
    return AP_Frsky_SPort::init_serial_port();
}

void  AP_Frsky_SPort_Passthrough::send_sport_frame(uint8_t frame, uint16_t appid, uint32_t data)
{
    if (_use_external_data) {
        external_data.packet.frame = frame;
        external_data.packet.appid = appid;
        external_data.packet.data = data;
        external_data.pending = true;
        return;
    }

    return AP_Frsky_SPort::send_sport_frame(frame, appid, data);
}

/*
  setup ready for passthrough telem
 */
void AP_Frsky_SPort_Passthrough::setup_wfq_scheduler(void)
{
    // initialize packet weights for the WFQ scheduler
    // priority[i] = 1/_scheduler.packet_weight[i]
    // rate[i] = LinkRate * ( priority[i] / (sum(priority[1-n])) )
    set_scheduler_entry(TEXT, 35, 28);          // 0x5000 status text (dynamic)
    set_scheduler_entry(ATTITUDE, 50, 38);      // 0x5006 Attitude and range (dynamic)
    set_scheduler_entry(GPS_LAT, 550, 280);     // 0x800 GPS lat
    set_scheduler_entry(GPS_LON, 550, 280);     // 0x800 GPS lon
    set_scheduler_entry(VEL_YAW, 400, 250);     // 0x5005 Vel and Yaw
    set_scheduler_entry(AP_STATUS, 700, 500);   // 0x5001 AP status
    set_scheduler_entry(GPS_STATUS, 700, 500);  // 0x5002 GPS status
    set_scheduler_entry(HOME, 400, 500);        // 0x5004 Home
    set_scheduler_entry(BATT_2, 1300, 500);     // 0x5008 Battery 2 status
    set_scheduler_entry(BATT_1, 1300, 500);     // 0x5003 Battery 1 status
    set_scheduler_entry(PARAM, 1700, 1000);     // 0x5007 parameters
    set_scheduler_entry(RPM, 300, 330);         // 0x500A rpm sensors 1 and 2
    set_scheduler_entry(TERRAIN, 700, 500);     // 0x500B terrain data
    set_scheduler_entry(WIND, 700, 500);        // 0x500C wind data
    set_scheduler_entry(WAYPOINT, 750, 500);    // 0x500D waypoint data
    set_scheduler_entry(UDATA, 5000, 200);      // user data

    // initialize default sport sensor ID
    set_sensor_id(_frsky_parameters->_dnlink_id, downlink_sensor_id);
#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    set_scheduler_entry(MAV, 35, 25);           // mavlite
    // initialize sport sensor IDs
    set_sensor_id(_frsky_parameters->_uplink_id, _SPort_bidir.uplink_sensor_id);
    set_sensor_id(_frsky_parameters->_dnlink1_id, _SPort_bidir.downlink1_sensor_id);
    set_sensor_id(_frsky_parameters->_dnlink2_id, _SPort_bidir.downlink2_sensor_id);
    // initialize sport
    hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_Frsky_SPort_Passthrough::process_rx_queue, void));
#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
}

/*
  dynamically change scheduler priorities based on queue sizes
*/
void AP_Frsky_SPort_Passthrough::adjust_packet_weight(bool queue_empty)
{
    /*
        When queues are empty set a low priority (high weight), when queues
        are not empty set a higher priority (low weight) based on the following
        relative priority order: mavlite > status text > attitude.
     */
#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    if (!_SPort_bidir.tx_packet_queue.is_empty()) {
        _scheduler.packet_weight[MAV] = 30;             // mavlite
        if (!queue_empty) {
            _scheduler.packet_weight[TEXT] = 45;        // messages
            _scheduler.packet_weight[ATTITUDE] = 80;    // attitude
        } else {
            _scheduler.packet_weight[TEXT] = 5000;      // messages
            _scheduler.packet_weight[ATTITUDE] = 80;    // attitude
        }
    } else {
        _scheduler.packet_weight[MAV] = 5000;           // mavlite
        if (!queue_empty) {
            _scheduler.packet_weight[TEXT] = 45;        // messages
            _scheduler.packet_weight[ATTITUDE] = 80;    // attitude
        } else {
            _scheduler.packet_weight[TEXT] = 5000;      // messages
            _scheduler.packet_weight[ATTITUDE] = 45;    // attitude
        }
    }
#else   //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    if (!queue_empty) {
        _scheduler.packet_weight[TEXT] = 45;     // messages
        _scheduler.packet_weight[ATTITUDE] = 80;     // attitude
    } else {
        _scheduler.packet_weight[TEXT] = 5000;   // messages
        _scheduler.packet_weight[ATTITUDE] = 45;     // attitude
    }
#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    // when using fport raise user data priority if any packets are pending
    if (_use_external_data && _sport_push_buffer.pending) {
        _scheduler.packet_weight[UDATA] = 250;
    } else {
        _scheduler.packet_weight[UDATA] = 5000;   // user data
    }
}

// WFQ scheduler
bool AP_Frsky_SPort_Passthrough::is_packet_ready(uint8_t idx, bool queue_empty)
{
    bool packet_ready = false;
    switch (idx) {
    case TEXT:
        packet_ready = !queue_empty;
        break;
    case GPS_LAT:
    case GPS_LON:
        // force gps coords to use default sensor ID, always send when used with external data
        packet_ready = _use_external_data || (_passthrough.new_byte == downlink_sensor_id);
        break;
    case AP_STATUS:
        packet_ready = _protocol->is_available_ap_status(); //OW
        break;
    case BATT_2:
        packet_ready = _protocol->is_available_batt(1); //OW
        break;
    case RPM:
        packet_ready = _protocol->is_available_rpm(); //OW
        break;
    case TERRAIN:
        packet_ready = _protocol->is_available_terrain(); //OW
        break;
    case WIND:
        packet_ready = _protocol->is_available_wind(); //OW
        break;
    case WAYPOINT:
        packet_ready = _protocol->is_available_waypoint(); //OW
        break;
    case UDATA:
        // when using fport user data is sent by scheduler
        // when using sport user data is sent responding to custom polling
        packet_ready = _use_external_data && _sport_push_buffer.pending;
        break;
#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    case MAV:
        packet_ready = !_SPort_bidir.tx_packet_queue.is_empty();
        break;
#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    default:
        packet_ready = true;
        break;
    }

    return packet_ready;
}

/*
 * WFQ scheduler
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
void AP_Frsky_SPort_Passthrough::process_packet(uint8_t idx)
{
    // send packet
    switch (idx) {
    case TEXT: // 0x5000 status text
        if (get_next_msg_chunk()) {
            send_sport_frame(SPORT_DATA_FRAME, DIY_FIRST_ID, _msg_chunk.chunk);
        }
        break;
    case ATTITUDE: // 0x5006 Attitude and range
        send_sport_frame(SPORT_DATA_FRAME, DIY_FIRST_ID+6, _protocol->calc_attiandrng());
        break;
    case GPS_LAT: // 0x800 GPS lat
        // sample both lat and lon at the same time
        // send gps latitude or longitude
        send_sport_frame(SPORT_DATA_FRAME, GPS_LONG_LATI_FIRST_ID, _protocol->calc_gps_latlng(_passthrough.send_latitude));
        _passthrough.gps_lng_sample = _protocol->calc_gps_latlng(_passthrough.send_latitude);
        // force the scheduler to select GPS lon as packet that's been waiting the most
        // this guarantees that lat and lon are sent as consecutive packets
        _scheduler.packet_timer[GPS_LON] = _scheduler.packet_timer[GPS_LAT] - 10000;
        break;
    case GPS_LON: // 0x800 GPS lon
        send_sport_frame(SPORT_DATA_FRAME, GPS_LONG_LATI_FIRST_ID, _passthrough.gps_lng_sample); // gps longitude
        break;
    case VEL_YAW: // 0x5005 Vel and Yaw
        send_sport_frame(SPORT_DATA_FRAME, DIY_FIRST_ID+5, calc_velandyaw());
        break;
    case AP_STATUS: // 0x5001 AP status
        send_sport_frame(SPORT_DATA_FRAME, DIY_FIRST_ID+1, _protocol->calc_ap_status());
        break;
    case GPS_STATUS: // 0x5002 GPS Status
        send_sport_frame(SPORT_DATA_FRAME, DIY_FIRST_ID+2, _protocol->calc_gps_status());
        break;
    case HOME: // 0x5004 Home
        send_sport_frame(SPORT_DATA_FRAME, DIY_FIRST_ID+4, _protocol->calc_home());
        break;
    case BATT_2: // 0x5008 Battery 2 status
        send_sport_frame(SPORT_DATA_FRAME, DIY_FIRST_ID+8, _protocol->calc_batt(1));
        break;
    case BATT_1: // 0x5003 Battery 1 status
        send_sport_frame(SPORT_DATA_FRAME, DIY_FIRST_ID+3, _protocol->calc_batt(0));
        break;
    case PARAM: // 0x5007 parameters
        send_sport_frame(SPORT_DATA_FRAME, DIY_FIRST_ID+7, calc_param());
        break;
    case RPM: // 0x500A rpm sensors 1 and 2
        send_sport_frame(SPORT_DATA_FRAME, DIY_FIRST_ID+0x0A, _protocol->calc_rpm());
        break;
    case TERRAIN: // 0x500B terrain data
        send_sport_frame(SPORT_DATA_FRAME, DIY_FIRST_ID+0x0B, _protocol->calc_terrain());
        break;
    case WIND: // 0x500C terrain data
        send_sport_frame(SPORT_DATA_FRAME, DIY_FIRST_ID+0x0C, _protocol->calc_wind());
        break;
    case WAYPOINT: // 0x500D waypoint data
        send_sport_frame(SPORT_DATA_FRAME, DIY_FIRST_ID+0x0D, _protocol->calc_waypoint());
        break;
    case UDATA: // user data
        {
            WITH_SEMAPHORE(_sport_push_buffer.sem);
            if (_use_external_data && _sport_push_buffer.pending) {
                send_sport_frame(_sport_push_buffer.packet.frame, _sport_push_buffer.packet.appid, _sport_push_buffer.packet.data);
                _sport_push_buffer.pending = false;
            }
        }
        break;
#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    case MAV: // mavlite
        process_tx_queue();
        break;
#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    }
}

/*
 * send telemetry data
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
void AP_Frsky_SPort_Passthrough::send(void)
{
    const uint16_t numc = MIN(_port->available(), 1024U);

    // this is the constant for hub data frame
    if (_port->txspace() < 19) {
        return;
    }
    // keep only the last two bytes of the data found in the serial buffer, as we shouldn't respond to old poll requests
    uint8_t prev_byte = 0;
    for (uint16_t i = 0; i < numc; i++) {
        prev_byte = _passthrough.new_byte;
        _passthrough.new_byte = _port->read();
#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
        AP_Frsky_SPort::sport_packet_t sp;

        if (_sport_handler.process_byte(sp, _passthrough.new_byte)) {
            queue_rx_packet(sp);
        }
#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    }
    // check if we should respond to this polling byte
    if (prev_byte == FRAME_HEAD) {
        if (is_passthrough_byte(_passthrough.new_byte)) {
            run_wfq_scheduler();
        } else {
            // respond to custom user data polling
            WITH_SEMAPHORE(_sport_push_buffer.sem);
            if (_sport_push_buffer.pending && _passthrough.new_byte == _sport_push_buffer.packet.sensor) {
                send_sport_frame(_sport_push_buffer.packet.frame, _sport_push_buffer.packet.appid, _sport_push_buffer.packet.data);
                _sport_push_buffer.pending = false;
            }
        }
    }
}

/*
 * grabs one "chunk" (4 bytes) of the queued message to be transmitted
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
bool AP_Frsky_SPort_Passthrough::get_next_msg_chunk(void)
{
    if (!_statustext.available) {
        WITH_SEMAPHORE(_statustext.sem);
        if (!_statustext.queue.pop(_statustext.next)) {
            return false;
        }
        _statustext.available = true;
    }

    if (_msg_chunk.repeats == 0) { // if it's the first time get_next_msg_chunk is called for a given chunk
        uint8_t character = 0;
        _msg_chunk.chunk = 0; // clear the 4 bytes of the chunk buffer

        for (uint8_t i = 0; i < 4 && _msg_chunk.char_index < sizeof(_statustext.next.text); i++) {
            character = _statustext.next.text[_msg_chunk.char_index++];

            if (!character) {
                break;
            }

            _msg_chunk.chunk |= character << (3-i) * 8;
        }

        if (!character || (_msg_chunk.char_index == sizeof(_statustext.next.text))) { // we've reached the end of the message (string terminated by '\0' or last character of the string has been processed)
            _msg_chunk.char_index = 0; // reset index to get ready to process the next message
            // add severity which is sent as the MSB of the last three bytes of the last chunk (bits 24, 16, and 8) since a character is on 7 bits
            _msg_chunk.chunk |= (_statustext.next.severity & 0x4)<<21;
            _msg_chunk.chunk |= (_statustext.next.severity & 0x2)<<14;
            _msg_chunk.chunk |= (_statustext.next.severity & 0x1)<<7;
        }
    }

    // repeat each message chunk 3 times to ensure transmission
    // on slow links reduce the number of duplicate chunks
    uint8_t extra_chunks = 2;

    if (_scheduler.avg_packet_rate < 20) {
        // with 3 or more extra frsky sensors on the bus
        // send messages only once
        extra_chunks = 0;
    } else if (_scheduler.avg_packet_rate < 30) {
        // with 1 or 2 extra frsky sensors on the bus
        // send messages twice
        extra_chunks = 1;
    }

    if (_msg_chunk.repeats++ > extra_chunks ) {
        _msg_chunk.repeats = 0;
        if (_msg_chunk.char_index == 0) {
            // we're ready for the next message
            _statustext.available = false;
        }
    }
    return true;
}

/*
 * true if we need to respond to the last polling byte
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
bool AP_Frsky_SPort_Passthrough::is_passthrough_byte(const uint8_t byte) const
{
#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
    if( byte == _SPort_bidir.downlink1_sensor_id || byte == _SPort_bidir.downlink2_sensor_id ) {
        return true;
    }
#endif
    return byte == downlink_sensor_id;
}

/*
 * prepare parameter data
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
uint32_t AP_Frsky_SPort_Passthrough::calc_param(void)
{
    return _protocol->calc_param(&_paramID);
}

/*
 * prepare velocity and yaw data
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
uint32_t AP_Frsky_SPort_Passthrough::calc_velandyaw(void)
{
    bool option_airspeed_enabled = (_frsky_parameters->_options & frsky_options_e::OPTION_AIRSPEED_AND_GROUNDSPEED) != 0;
    uint32_t velandyaw = _protocol->calc_velandyaw(option_airspeed_enabled, _passthrough.send_airspeed);
    // toggle air/ground speed selector
    _passthrough.send_airspeed = !_passthrough.send_airspeed;
    return velandyaw;
}

/*
  fetch Sport data for an external transport, such as FPort or crossfire
  Note: we need to create a packet array with unique packet types
  For very big frames we might have to relax the "unique packet type per frame"
  constraint in order to maximize bandwidth usage
 */
bool AP_Frsky_SPort_Passthrough::get_telem_data(sport_packet_t* packet_array, uint8_t &packet_count, const uint8_t max_size)
{
    if (!_use_external_data) {
        return false;
    }

    uint8_t idx = 0;

    // max_size >= WFQ_LAST_ITEM
    // get a packet per enabled type
    if (max_size >= WFQ_LAST_ITEM) {
        for (uint8_t i=0; i<WFQ_LAST_ITEM; i++) {
            if (process_scheduler_entry(i)) {
                if (external_data.pending) {
                    packet_array[idx].frame = external_data.packet.frame;
                    packet_array[idx].appid = external_data.packet.appid;
                    packet_array[idx].data = external_data.packet.data;
                    idx++;
                    external_data.pending = false;
                }
            }
        }
    } else {
        // max_size < WFQ_LAST_ITEM
        // call run_wfq_scheduler(false) enough times to create a packet of up to max_size unique elements
        uint32_t item_mask = 0;
        for (uint8_t i=0; i<max_size; i++) {
            // call the scheduler with the shaper "disabled"
            const uint8_t item = run_wfq_scheduler(false);
            if (!BIT_IS_SET(item_mask, item) && external_data.pending) {
                // ok got some data, flip the bitmask bit to prevent adding the same packet type more than once
                BIT_SET(item_mask, item);
                packet_array[idx].frame = external_data.packet.frame;
                packet_array[idx].appid = external_data.packet.appid;
                packet_array[idx].data = external_data.packet.data;
                idx++;
                external_data.pending = false;
            }
        }
    }
    packet_count = idx;
    return idx > 0;
}

#if HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL
/*
  allow external transports (e.g. FPort), to supply telemetry data
 */
bool AP_Frsky_SPort_Passthrough::set_telem_data(const uint8_t frame, const uint16_t appid, const uint32_t data)
{
    // queue only Uplink packets
    if (frame == SPORT_UPLINK_FRAME || frame == SPORT_UPLINK_FRAME_RW) {
        const AP_Frsky_SPort::sport_packet_t sp {
            { 0x00,   // this is ignored by process_sport_rx_queue() so no need for a real sensor ID
            frame,
            appid,
            data }
        };

        _SPort_bidir.rx_packet_queue.push_force(sp);
        return true;
    }
    return false;
}

/*
 * Queue uplink packets in the sport rx queue
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
void AP_Frsky_SPort_Passthrough::queue_rx_packet(const AP_Frsky_SPort::sport_packet_t packet)
{
    // queue only Uplink packets
    if (packet.sensor == _SPort_bidir.uplink_sensor_id && packet.frame == SPORT_UPLINK_FRAME) {
        _SPort_bidir.rx_packet_queue.push_force(packet);
    }
}

/*
 * Extract up to 1 mavlite message from the sport rx packet queue
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
void AP_Frsky_SPort_Passthrough::process_rx_queue()
{
    AP_Frsky_SPort::sport_packet_t packet;
    uint8_t loop_count = 0; // prevent looping forever
    while (_SPort_bidir.rx_packet_queue.pop(packet) && loop_count++ < MAVLITE_MSG_SPORT_PACKETS_COUNT(MAVLITE_MAX_PAYLOAD_LEN)) {
        AP_Frsky_MAVlite_Message rxmsg;

        if (sport_to_mavlite.process(rxmsg, packet)) {
            mavlite.process_message(rxmsg);
            break; // process only 1 mavlite message each call
        }
    }
}

/*
 * Process the sport tx queue
 * pop and send 1 sport packet
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
void AP_Frsky_SPort_Passthrough::process_tx_queue()
{
    AP_Frsky_SPort::sport_packet_t packet;

    if (!_SPort_bidir.tx_packet_queue.peek(packet)) {
        return;
    }

    // when using fport repeat each packet to account for
    // fport packet loss (around 15%)
    if (!_use_external_data || _SPort_bidir.tx_packet_duplicates++ == SPORT_TX_PACKET_DUPLICATES) {
        _SPort_bidir.tx_packet_queue.pop();
        _SPort_bidir.tx_packet_duplicates = 0;
    }

    send_sport_frame(SPORT_DOWNLINK_FRAME, packet.appid, packet.data);
}

/*
 * Utility method to apply constraints in changing sensor id values
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
void AP_Frsky_SPort_Passthrough::set_sensor_id(AP_Int8 param_idx, uint8_t &sensor)
{
    int8_t idx = param_idx.get();

    if (idx == -1) {
        // disable this sensor
        sensor = 0xFF;
        return;
    }
    sensor = calc_sensor_id(idx);
}

/*
 * Send a mavlite message
 * Message is chunked in sport packets pushed in the tx queue
 * for FrSky SPort Passthrough (OpenTX) protocol (X-receivers)
 */
bool AP_Frsky_SPort_Passthrough::send_message(const AP_Frsky_MAVlite_Message &txmsg)
{
    return mavlite_to_sport.process(_SPort_bidir.tx_packet_queue, txmsg);
}
#endif //HAL_WITH_FRSKY_TELEM_BIDIRECTIONAL

namespace AP
{
AP_Frsky_SPort_Passthrough *frsky_passthrough_telem()
{
    return AP_Frsky_SPort_Passthrough::get_singleton();
}
};
