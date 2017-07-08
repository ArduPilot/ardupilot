/*
  Common GCS MAVLink functions for all vehicle types

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <AP_AHRS/AP_AHRS.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_OpticalFlow/AP_OpticalFlow.h>
#include <AP_Vehicle/AP_Vehicle.h>

#include "ap_version.h"
#include "GCS.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#include <drivers/drv_pwm_output.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#endif

extern const AP_HAL::HAL& hal;

uint32_t GCS_MAVLINK::last_radio_status_remrssi_ms;
uint8_t GCS_MAVLINK::mavlink_active = 0;
uint8_t GCS_MAVLINK::chan_is_streaming = 0;
uint32_t GCS_MAVLINK::reserve_param_space_start_ms;

AP_HAL::Util::perf_counter_t GCS_MAVLINK::_perf_packet;
AP_HAL::Util::perf_counter_t GCS_MAVLINK::_perf_update;

GCS *GCS::_singleton = nullptr;

GCS_MAVLINK::GCS_MAVLINK()
{
    AP_Param::setup_object_defaults(this, var_info);
}

void
GCS_MAVLINK::init(AP_HAL::UARTDriver *port, mavlink_channel_t mav_chan)
{
    if (!valid_channel(mav_chan)) {
        return;
    }

    _port = port;
    chan = mav_chan;

    mavlink_comm_port[chan] = _port;
    initialised = true;
    _queued_parameter = nullptr;
    reset_cli_timeout();

    if (!_perf_packet) {
        _perf_packet = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "GCS_Packet");
    }
    if (!_perf_update) {
        _perf_update = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, "GCS_Update");
    }
}


/*
  setup a UART, handling begin() and init()
 */
void
GCS_MAVLINK::setup_uart(const AP_SerialManager& serial_manager, AP_SerialManager::SerialProtocol protocol, uint8_t instance)
{
    serialmanager_p = &serial_manager;

    // search for serial port
    
    AP_HAL::UARTDriver *uart;
    uart = serial_manager.find_serial(protocol, instance);
    if (uart == nullptr) {
        // return immediately if not found
        return;
    }

    // get associated mavlink channel
    mavlink_channel_t mav_chan;
    if (!serial_manager.get_mavlink_channel(protocol, instance, mav_chan)) {
        // return immediately in unlikely case mavlink channel cannot be found
        return;
    }

    /*
      Now try to cope with SiK radios that may be stuck in bootloader
      mode because CTS was held while powering on. This tells the
      bootloader to wait for a firmware. It affects any SiK radio with
      CTS connected that is externally powered. To cope we send 0x30
      0x20 at 115200 on startup, which tells the bootloader to reset
      and boot normally
     */
    uart->begin(115200);
    AP_HAL::UARTDriver::flow_control old_flow_control = uart->get_flow_control();
    uart->set_flow_control(AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE);
    for (uint8_t i=0; i<3; i++) {
        hal.scheduler->delay(1);
        uart->write(0x30);
        uart->write(0x20);
    }
    // since tcdrain() and TCSADRAIN may not be implemented...
    hal.scheduler->delay(1);
    
    uart->set_flow_control(old_flow_control);

    // now change back to desired baudrate
    uart->begin(serial_manager.find_baudrate(protocol, instance));

    // and init the gcs instance
    init(uart, mav_chan);

    AP_SerialManager::SerialProtocol mavlink_protocol = serialmanager_p->get_mavlink_protocol(mav_chan);
    mavlink_status_t *status = mavlink_get_channel_status(chan);
    if (status == nullptr) {
        return;
    }
    
    if (mavlink_protocol == AP_SerialManager::SerialProtocol_MAVLink2) {
        // load signing key
        load_signing_key();

        if (status->signing == nullptr) {
            // if signing is off start by sending MAVLink1.
            status->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
        }
        // announce that we are MAVLink2 capable
        hal.util->set_capabilities(MAV_PROTOCOL_CAPABILITY_MAVLINK2);
    } else if (status) {
        // user has asked to only send MAVLink1
        status->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    }

    if (chan == MAVLINK_COMM_0) {
        // Always start with MAVLink1 on first port for now, to allow for recovery
        // after experiments with MAVLink2
        status->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    }
}


/**
 * @brief Send the next pending waypoint, called from deferred message
 * handling code
 */
void
GCS_MAVLINK::queued_waypoint_send()
{
    if (initialised &&
        waypoint_receiving &&
        waypoint_request_i <= waypoint_request_last) {
        mavlink_msg_mission_request_send(
            chan,
            waypoint_dest_sysid,
            waypoint_dest_compid,
            waypoint_request_i,
            MAV_MISSION_TYPE_MISSION);
    }
}

void GCS_MAVLINK::reset_cli_timeout() {
    _cli_timeout = AP_HAL::millis();
}

void GCS_MAVLINK::send_meminfo(void)
{
    unsigned __brkval = 0;
    uint32_t memory = hal.util->available_memory();
    mavlink_msg_meminfo_send(chan, __brkval, memory & 0xFFFF, memory);
}

// report power supply status
void GCS_MAVLINK::send_power_status(void)
{
    mavlink_msg_power_status_send(chan,
                                  hal.analogin->board_voltage() * 1000,
                                  hal.analogin->servorail_voltage() * 1000,
                                  hal.analogin->power_status_flags());
}

void GCS_MAVLINK::send_battery_status(const AP_BattMonitor &battery, const uint8_t instance) const
{
    // catch the battery backend not supporting the required number of cells
    static_assert(sizeof(AP_BattMonitor::cells) >= (sizeof(uint16_t) * MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_LEN),
                  "Not enough battery cells for the MAVLink message");

    float temp;
    bool got_temperature = battery.get_temperature(temp, instance);
    mavlink_msg_battery_status_send(chan,
                                    instance, // id
                                    MAV_BATTERY_FUNCTION_UNKNOWN, // function
                                    MAV_BATTERY_TYPE_UNKNOWN, // type
                                    got_temperature ? ((int16_t) (temp * 100)) : INT16_MAX, // temperature. INT16_MAX if unknown
                                    battery.get_cell_voltages(instance).cells, // cell voltages
                                    battery.has_current(instance) ? battery.current_amps(instance) * 100 : -1, // current
                                    battery.has_current(instance) ? battery.current_total_mah(instance) : -1, // total current
                                    -1, // joules used
                                    battery.capacity_remaining_pct(instance));
}

// returns true if all battery instances were reported
bool GCS_MAVLINK::send_battery_status(const AP_BattMonitor &battery) const
{
    for(uint8_t i = 0; i < battery.num_instances(); i++) {
        CHECK_PAYLOAD_SIZE(BATTERY_STATUS);
        send_battery_status(battery, i);
    }
    return true;
}

void GCS_MAVLINK::send_distance_sensor(const RangeFinder &rangefinder, const uint8_t instance) const
{
    if (rangefinder.status(instance) != RangeFinder::RangeFinder_NotConnected &&
        rangefinder.status(instance) != RangeFinder::RangeFinder_NoData) {
        mavlink_msg_distance_sensor_send(
                chan,
                AP_HAL::millis(),                       // time since system boot TODO: take time of measurement
                rangefinder.min_distance_cm(instance),  // minimum distance the sensor can measure in centimeters
                rangefinder.max_distance_cm(instance),  // maximum distance the sensor can measure in centimeters
                rangefinder.distance_cm(instance),      // current distance reading (in cm?)
                rangefinder.get_sensor_type(instance),  // type from MAV_DISTANCE_SENSOR enum
                instance,                               // onboard ID of the sensor == instance
                rangefinder.get_orientation(instance),  // direction the sensor faces from MAV_SENSOR_ORIENTATION enum
                0);                                     // Measurement covariance in centimeters, 0 for unknown / invalid readings
    }
}

bool GCS_MAVLINK::send_distance_sensor(const RangeFinder &rangefinder) const
{
    for (uint8_t i = 0; i < RANGEFINDER_MAX_INSTANCES; i++) {
        CHECK_PAYLOAD_SIZE(DISTANCE_SENSOR);
        send_distance_sensor(rangefinder, i);
    }
    return true;
}

void GCS_MAVLINK::send_distance_sensor_downward(const RangeFinder &rangefinder) const
{
    // exit immediately if rangefinder is disabled or not downward looking
    if (!rangefinder.has_data_orient(ROTATION_PITCH_270)) {
        return;
    }
    uint8_t instance;
    rangefinder.find_instance(ROTATION_PITCH_270, instance);
    send_distance_sensor(rangefinder, instance);
}

void GCS_MAVLINK::send_rangefinder_downward(const RangeFinder &rangefinder) const
{
    // exit immediately if rangefinder is disabled or not downward looking
    if (!rangefinder.has_data_orient(ROTATION_PITCH_270)) {
        // no sonar to report
        return;
    }
    mavlink_msg_rangefinder_send(
            chan,
            rangefinder.distance_cm_orient(ROTATION_PITCH_270) * 0.01f,
            rangefinder.voltage_mv_orient(ROTATION_PITCH_270) * 0.001f);
}

// report AHRS2 state
void GCS_MAVLINK::send_ahrs2(AP_AHRS &ahrs)
{
#if AP_AHRS_NAVEKF_AVAILABLE
    Vector3f euler;
    struct Location loc {};
    if (ahrs.get_secondary_attitude(euler)) {
        mavlink_msg_ahrs2_send(chan,
                               euler.x,
                               euler.y,
                               euler.z,
                               loc.alt*1.0e-2f,
                               loc.lat,
                               loc.lng);
    }
    AP_AHRS_NavEKF &_ahrs = reinterpret_cast<AP_AHRS_NavEKF&>(ahrs);
    if (_ahrs.get_NavEKF2().activeCores() > 0 &&
        HAVE_PAYLOAD_SPACE(chan, AHRS3)) {
        _ahrs.get_NavEKF2().getLLH(loc);
        _ahrs.get_NavEKF2().getEulerAngles(-1,euler);
        mavlink_msg_ahrs3_send(chan,
                               euler.x,
                               euler.y,
                               euler.z,
                               loc.alt*1.0e-2f,
                               loc.lat,
                               loc.lng,
                               0, 0, 0, 0);
    }
#endif
}

/*
  handle a MISSION_REQUEST_LIST mavlink packet
 */
void GCS_MAVLINK::handle_mission_request_list(AP_Mission &mission, mavlink_message_t *msg)
{
    // decode
    mavlink_mission_request_list_t packet;
    mavlink_msg_mission_request_list_decode(msg, &packet);

    // reply with number of commands in the mission.  The GCS will then request each command separately
    mavlink_msg_mission_count_send(chan,msg->sysid, msg->compid, mission.num_commands(),
                                   MAV_MISSION_TYPE_MISSION);

    // set variables to help handle the expected sending of commands to the GCS
    waypoint_receiving = false;             // record that we are sending commands (i.e. not receiving)
    waypoint_dest_sysid = msg->sysid;       // record system id of GCS who has requested the commands
    waypoint_dest_compid = msg->compid;     // record component id of GCS who has requested the commands
}

/*
  handle a MISSION_REQUEST mavlink packet
 */
void GCS_MAVLINK::handle_mission_request(AP_Mission &mission, mavlink_message_t *msg)
{
    AP_Mission::Mission_Command cmd;

    if (msg->msgid == MAVLINK_MSG_ID_MISSION_REQUEST_INT) {  
        // decode
        mavlink_mission_request_int_t packet;
        mavlink_msg_mission_request_int_decode(msg, &packet);

        // retrieve mission from eeprom
        if (!mission.read_cmd_from_storage(packet.seq, cmd)) {
            goto mission_item_send_failed;
        }

        mavlink_mission_item_int_t ret_packet;
        memset(&ret_packet, 0, sizeof(ret_packet));
        if (!AP_Mission::mission_cmd_to_mavlink_int(cmd, ret_packet)) {
            goto mission_item_send_failed;
        }

        // set packet's current field to 1 if this is the command being executed
        if (cmd.id == (uint16_t)mission.get_current_nav_cmd().index) {
            ret_packet.current = 1;
        } else {
            ret_packet.current = 0;
        }

        // set auto continue to 1
        ret_packet.autocontinue = 1;     // 1 (true), 0 (false)

        /*
          avoid the _send() function to save memory, as it avoids
          the stack usage of the _send() function by using the already
          declared ret_packet above
         */
        ret_packet.target_system = msg->sysid;
        ret_packet.target_component = msg->compid;
        ret_packet.seq = packet.seq;
        ret_packet.command = cmd.id;

        _mav_finalize_message_chan_send(chan, 
                                        MAVLINK_MSG_ID_MISSION_ITEM_INT,
                                        (const char *)&ret_packet,
                                        MAVLINK_MSG_ID_MISSION_ITEM_MIN_LEN,
                                        MAVLINK_MSG_ID_MISSION_ITEM_INT_LEN,
                                        MAVLINK_MSG_ID_MISSION_ITEM_INT_CRC);
    } else {
        // decode
        mavlink_mission_request_t packet;
        mavlink_msg_mission_request_decode(msg, &packet);

        if (packet.seq != 0 && // always allow HOME to be read
            packet.seq >= mission.num_commands()) {
            // try to educate the GCS on the actual size of the mission:
            mavlink_msg_mission_count_send(chan,msg->sysid, msg->compid, mission.num_commands(),
                                           MAV_MISSION_TYPE_MISSION);
            goto mission_item_send_failed;
        }

        // retrieve mission from eeprom
        if (!mission.read_cmd_from_storage(packet.seq, cmd)) {
            goto mission_item_send_failed;
        }

        mavlink_mission_item_t ret_packet;
        memset(&ret_packet, 0, sizeof(ret_packet));
        if (!AP_Mission::mission_cmd_to_mavlink(cmd, ret_packet)) {
            goto mission_item_send_failed;
        }
            
        // set packet's current field to 1 if this is the command being executed
        if (cmd.id == (uint16_t)mission.get_current_nav_cmd().index) {
            ret_packet.current = 1;
        } else {
            ret_packet.current = 0;
        }

        // set auto continue to 1
        ret_packet.autocontinue = 1;     // 1 (true), 0 (false)

        /*
          avoid the _send() function to save memory, as it avoids
          the stack usage of the _send() function by using the already
          declared ret_packet above
         */
        ret_packet.target_system = msg->sysid;
        ret_packet.target_component = msg->compid;
        ret_packet.seq = packet.seq;
        ret_packet.command = cmd.id;

        _mav_finalize_message_chan_send(chan, 
                                        MAVLINK_MSG_ID_MISSION_ITEM,
                                        (const char *)&ret_packet,
                                        MAVLINK_MSG_ID_MISSION_ITEM_MIN_LEN,
                                        MAVLINK_MSG_ID_MISSION_ITEM_LEN,
                                        MAVLINK_MSG_ID_MISSION_ITEM_CRC);
    }

    return;

mission_item_send_failed:
    // send failure message
    mavlink_msg_mission_ack_send(chan, msg->sysid, msg->compid, MAV_MISSION_ERROR,
                                 MAV_MISSION_TYPE_MISSION);
}

/*
  handle a MISSION_SET_CURRENT mavlink packet
 */
void GCS_MAVLINK::handle_mission_set_current(AP_Mission &mission, mavlink_message_t *msg)
{
    // decode
    mavlink_mission_set_current_t packet;
    mavlink_msg_mission_set_current_decode(msg, &packet);

    // set current command
    if (mission.set_current_cmd(packet.seq)) {
        mavlink_msg_mission_current_send(chan, packet.seq);
    }
}

/*
  handle a MISSION_COUNT mavlink packet
 */
void GCS_MAVLINK::handle_mission_count(AP_Mission &mission, mavlink_message_t *msg)
{
    // decode
    mavlink_mission_count_t packet;
    mavlink_msg_mission_count_decode(msg, &packet);

    // start waypoint receiving
    if (packet.count > mission.num_commands_max()) {
        // send NAK
        mavlink_msg_mission_ack_send(chan, msg->sysid, msg->compid, MAV_MISSION_NO_SPACE,
                                     MAV_MISSION_TYPE_MISSION);
        return;
    }

    // new mission arriving, truncate mission to be the same length
    mission.truncate(packet.count);

    // set variables to help handle the expected receiving of commands from the GCS
    waypoint_timelast_receive = AP_HAL::millis();    // set time we last received commands to now
    waypoint_receiving = true;              // record that we expect to receive commands
    waypoint_request_i = 0;                 // reset the next expected command number to zero
    waypoint_request_last = packet.count;   // record how many commands we expect to receive
    waypoint_timelast_request = 0;          // set time we last requested commands to zero
}

/*
  handle a MISSION_CLEAR_ALL mavlink packet
 */
void GCS_MAVLINK::handle_mission_clear_all(AP_Mission &mission, mavlink_message_t *msg)
{
    // decode
    mavlink_mission_clear_all_t packet;
    mavlink_msg_mission_clear_all_decode(msg, &packet);

    // clear all waypoints
    if (mission.clear()) {
        // send ack
        mavlink_msg_mission_ack_send(chan, msg->sysid, msg->compid, MAV_RESULT_ACCEPTED,
                                     MAV_MISSION_TYPE_MISSION);
    }else{
        // send nack
        mavlink_msg_mission_ack_send(chan, msg->sysid, msg->compid, MAV_MISSION_ERROR,
                                     MAV_MISSION_TYPE_MISSION);
    }
}

/*
  handle a MISSION_WRITE_PARTIAL_LIST mavlink packet
 */
void GCS_MAVLINK::handle_mission_write_partial_list(AP_Mission &mission, mavlink_message_t *msg)
{
    // decode
    mavlink_mission_write_partial_list_t packet;
    mavlink_msg_mission_write_partial_list_decode(msg, &packet);

    // start waypoint receiving
    if ((unsigned)packet.start_index > mission.num_commands() ||
        (unsigned)packet.end_index > mission.num_commands() ||
        packet.end_index < packet.start_index) {
        send_text(MAV_SEVERITY_WARNING,"Flight plan update rejected");
        return;
    }

    waypoint_timelast_receive = AP_HAL::millis();
    waypoint_timelast_request = 0;
    waypoint_receiving   = true;
    waypoint_request_i   = packet.start_index;
    waypoint_request_last= packet.end_index;
}


/*
  handle a GIMBAL_REPORT mavlink packet
 */
void GCS_MAVLINK::handle_gimbal_report(AP_Mount &mount, mavlink_message_t *msg) const
{
    mount.handle_gimbal_report(chan, msg);
}


void
GCS_MAVLINK::send_text(MAV_SEVERITY severity, const char *str)
{
    GCS_MAVLINK::send_statustext_chan(severity, chan, str);
}

void GCS_MAVLINK::handle_radio_status(mavlink_message_t *msg, DataFlash_Class &dataflash, bool log_radio)
{
    mavlink_radio_t packet;
    mavlink_msg_radio_decode(msg, &packet);

    // record if the GCS has been receiving radio messages from
    // the aircraft
    if (packet.remrssi != 0) {
        last_radio_status_remrssi_ms = AP_HAL::millis();
    }

    // use the state of the transmit buffer in the radio to
    // control the stream rate, giving us adaptive software
    // flow control
    if (packet.txbuf < 20 && stream_slowdown < 100) {
        // we are very low on space - slow down a lot
        stream_slowdown += 3;
    } else if (packet.txbuf < 50 && stream_slowdown < 100) {
        // we are a bit low on space, slow down slightly
        stream_slowdown += 1;
    } else if (packet.txbuf > 95 && stream_slowdown > 10) {
        // the buffer has plenty of space, speed up a lot
        stream_slowdown -= 2;
    } else if (packet.txbuf > 90 && stream_slowdown != 0) {
        // the buffer has enough space, speed up a bit
        stream_slowdown--;
    }

    //log rssi, noise, etc if logging Performance monitoring data
    if (log_radio) {
        dataflash.Log_Write_Radio(packet);
    }
}

/*
  handle an incoming mission item
  return true if this is the last mission item, otherwise false
 */
bool GCS_MAVLINK::handle_mission_item(mavlink_message_t *msg, AP_Mission &mission)
{
    MAV_MISSION_RESULT result = MAV_MISSION_ACCEPTED;
    struct AP_Mission::Mission_Command cmd = {};
    bool mission_is_complete = false;
    uint16_t seq=0;
    uint16_t current = 0;
    
    if (msg->msgid == MAVLINK_MSG_ID_MISSION_ITEM) {      
        mavlink_mission_item_t packet;    
        mavlink_msg_mission_item_decode(msg, &packet);
        
        // convert mavlink packet to mission command
        result = AP_Mission::mavlink_to_mission_cmd(packet, cmd);
        if (result != MAV_MISSION_ACCEPTED) {
            goto mission_ack;
        }
        
        seq = packet.seq;
        current = packet.current;
    } else {
        mavlink_mission_item_int_t packet;
        mavlink_msg_mission_item_int_decode(msg, &packet);
        
        // convert mavlink packet to mission command
        result = AP_Mission::mavlink_int_to_mission_cmd(packet, cmd);
        if (result != MAV_MISSION_ACCEPTED) {
            goto mission_ack;
        }
        
        seq = packet.seq;
        current = packet.current;
    }

    if (current == 2) {                                               
        // current = 2 is a flag to tell us this is a "guided mode"
        // waypoint and not for the mission
        result = (handle_guided_request(cmd) ? MAV_MISSION_ACCEPTED
                                             : MAV_MISSION_ERROR) ;

        // verify we received the command
        goto mission_ack;
    }

    if (current == 3) {
        //current = 3 is a flag to tell us this is a alt change only
        // add home alt if needed
        handle_change_alt_request(cmd);

        // verify we recevied the command
        result = MAV_MISSION_ACCEPTED;
        goto mission_ack;
    }

    // Check if receiving waypoints (mission upload expected)
    if (!waypoint_receiving) {
        result = MAV_MISSION_ERROR;
        goto mission_ack;
    }

    // check if this is the requested waypoint
    if (seq != waypoint_request_i) {
        result = MAV_MISSION_INVALID_SEQUENCE;
        goto mission_ack;
    }

    // sanity check for DO_JUMP command
    if (cmd.id == MAV_CMD_DO_JUMP) {
        if ((cmd.content.jump.target >= mission.num_commands() && cmd.content.jump.target >= waypoint_request_last) || cmd.content.jump.target == 0) {
            result = MAV_MISSION_ERROR;
            goto mission_ack;
        }
    }
    
    // if command index is within the existing list, replace the command
    if (seq < mission.num_commands()) {
        if (mission.replace_cmd(seq,cmd)) {
            result = MAV_MISSION_ACCEPTED;
        }else{
            result = MAV_MISSION_ERROR;
            goto mission_ack;
        }
        // if command is at the end of command list, add the command
    } else if (seq == mission.num_commands()) {
        if (mission.add_cmd(cmd)) {
            result = MAV_MISSION_ACCEPTED;
        }else{
            result = MAV_MISSION_ERROR;
            goto mission_ack;
        }
        // if beyond the end of the command list, return an error
    } else {
        result = MAV_MISSION_ERROR;
        goto mission_ack;
    }
    
    // update waypoint receiving state machine
    waypoint_timelast_receive = AP_HAL::millis();
    waypoint_request_i++;
    
    if (waypoint_request_i >= waypoint_request_last) {
        mavlink_msg_mission_ack_send_buf(
            msg,
            chan,
            msg->sysid,
            msg->compid,
            MAV_MISSION_ACCEPTED,
            MAV_MISSION_TYPE_MISSION);
        
        send_text(MAV_SEVERITY_INFO,"Flight plan received");
        waypoint_receiving = false;
        mission_is_complete = true;
        // XXX ignores waypoint radius for individual waypoints, can
        // only set WP_RADIUS parameter
    } else {
        waypoint_timelast_request = AP_HAL::millis();
        // if we have enough space, then send the next WP immediately
        if (HAVE_PAYLOAD_SPACE(chan, MISSION_ITEM)) {
            queued_waypoint_send();
        } else {
            send_message(MSG_NEXT_WAYPOINT);
        }
    }
    return mission_is_complete;

mission_ack:
    // we are rejecting the mission/waypoint
    mavlink_msg_mission_ack_send_buf(
        msg,
        chan,
        msg->sysid,
        msg->compid,
        result,
        MAV_MISSION_TYPE_MISSION);

    return mission_is_complete;
}

void 
GCS_MAVLINK::handle_gps_inject(const mavlink_message_t *msg, AP_GPS &gps)
{
    mavlink_gps_inject_data_t packet;
    mavlink_msg_gps_inject_data_decode(msg, &packet);
    //TODO: check target

    gps.inject_data(packet.data, packet.len);

}

// send a message using mavlink, handling message queueing
void GCS_MAVLINK::send_message(enum ap_message id)
{
    uint8_t i, nextid;

    if (id == MSG_HEARTBEAT) {
        save_signing_timestamp(false);
    }
    
    // see if we can send the deferred messages, if any
    while (num_deferred_messages != 0) {
        if (!try_send_message(deferred_messages[next_deferred_message])) {
            break;
        }
        next_deferred_message++;
        if (next_deferred_message == MSG_RETRY_DEFERRED) {
            next_deferred_message = 0;
        }
        num_deferred_messages--;
    }

    if (id == MSG_RETRY_DEFERRED) {
        return;
    }

    // this message id might already be deferred
    for (i=0, nextid = next_deferred_message; i < num_deferred_messages; i++) {
        if (deferred_messages[nextid] == id) {
            // it's already deferred, discard
            return;
        }
        nextid++;
        if (nextid == MSG_RETRY_DEFERRED) {
            nextid = 0;
        }
    }

    if (num_deferred_messages != 0 ||
        !try_send_message(id)) {
        // can't send it now, so defer it
        if (num_deferred_messages == MSG_RETRY_DEFERRED) {
            // the defer buffer is full, discard
            return;
        }
        nextid = next_deferred_message + num_deferred_messages;
        if (nextid >= MSG_RETRY_DEFERRED) {
            nextid -= MSG_RETRY_DEFERRED;
        }
        deferred_messages[nextid] = id;
        num_deferred_messages++;
    }
}

void GCS_MAVLINK::packetReceived(const mavlink_status_t &status,
                                 mavlink_message_t &msg)
{
    // we exclude radio packets to make it possible to use the
    // CLI over the radio
    if (msg.msgid != MAVLINK_MSG_ID_RADIO && msg.msgid != MAVLINK_MSG_ID_RADIO_STATUS) {
        mavlink_active |= (1U<<(chan-MAVLINK_COMM_0));
    }
    if (!(status.flags & MAVLINK_STATUS_FLAG_IN_MAVLINK1) &&
        (status.flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) &&
        serialmanager_p &&
        serialmanager_p->get_mavlink_protocol(chan) == AP_SerialManager::SerialProtocol_MAVLink2) {
        // if we receive any MAVLink2 packets on a connection
        // currently sending MAVLink1 then switch to sending
        // MAVLink2
        mavlink_status_t *cstatus = mavlink_get_channel_status(chan);
        if (cstatus != nullptr) {
            cstatus->flags &= ~MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
        }
    }
    // if a snoop handler has been setup then use it
    if (msg_snoop != nullptr) {
        msg_snoop(&msg);
    }
    if (routing.check_and_forward(chan, &msg) &&
        accept_packet(status, msg)) {
        handleMessage(&msg);
    }
}

void
GCS_MAVLINK::update(run_cli_fn run_cli, uint32_t max_time_us)
{
    // receive new packets
    mavlink_message_t msg;
    mavlink_status_t status;
    uint32_t tstart_us = AP_HAL::micros();

    hal.util->perf_begin(_perf_update);

    status.packet_rx_drop_count = 0;

    // process received bytes
    uint16_t nbytes = comm_get_available(chan);
    for (uint16_t i=0; i<nbytes; i++)
    {
        uint8_t c = comm_receive_ch(chan);

        if (run_cli) {
            /* allow CLI to be started by hitting enter 3 times, if no
             *  heartbeat packets have been received */
            if ((mavlink_active==0) && (AP_HAL::millis() - _cli_timeout) < 20000 && 
                comm_is_idle(chan)) {
                if (c == '\n' || c == '\r') {
                    crlf_count++;
                } else {
                    crlf_count = 0;
                }
                if (crlf_count == 3) {
                    run_cli(_port);
                }
            }
        }

        bool parsed_packet = false;
        
        // Try to get a new message
        if (mavlink_parse_char(chan, c, &msg, &status)) {
            hal.util->perf_begin(_perf_packet);
            packetReceived(status, msg);
            hal.util->perf_end(_perf_packet);
            parsed_packet = true;
        }

        if (parsed_packet || i % 100 == 0) {
            // make sure we don't spend too much time parsing mavlink messages
            if (AP_HAL::micros() - tstart_us > max_time_us) {
                break;
            }
        }
    }

    if (!waypoint_receiving) {
        hal.util->perf_end(_perf_update);    
        return;
    }

    uint32_t tnow = AP_HAL::millis();
    uint32_t wp_recv_time = 1000U + (stream_slowdown*20);

    // stop waypoint receiving if timeout
    if (waypoint_receiving && (tnow - waypoint_timelast_receive) > wp_recv_time+waypoint_receive_timeout) {
        waypoint_receiving = false;
    } else if (waypoint_receiving &&
               (tnow - waypoint_timelast_request) > wp_recv_time) {
        waypoint_timelast_request = tnow;
        send_message(MSG_NEXT_WAYPOINT);
    }

    hal.util->perf_end(_perf_update);    
}


/*
  send raw GPS position information (GPS_RAW_INT, GPS2_RAW, GPS_RTK and GPS2_RTK).
  returns true if messages fit into transmit buffer, false otherwise.
 */
bool GCS_MAVLINK::send_gps_raw(AP_GPS &gps)
{
    if (HAVE_PAYLOAD_SPACE(chan, GPS_RAW_INT)) {
        gps.send_mavlink_gps_raw(chan);
    } else {
        return false;
    }

    if (gps.highest_supported_status(0) > AP_GPS::GPS_OK_FIX_3D) {
        if (HAVE_PAYLOAD_SPACE(chan, GPS_RTK)) {
            gps.send_mavlink_gps_rtk(chan);
        }

    }

    if (gps.num_sensors() > 1 && gps.status(1) > AP_GPS::NO_GPS) {

        if (HAVE_PAYLOAD_SPACE(chan, GPS2_RAW)) {
            gps.send_mavlink_gps2_raw(chan);
        }

        if (gps.highest_supported_status(1) > AP_GPS::GPS_OK_FIX_3D) {
            if (HAVE_PAYLOAD_SPACE(chan, GPS2_RTK)) {
                gps.send_mavlink_gps2_rtk(chan);
            }
        }
    }

    //TODO: Should check what else managed to get through...
    return true;

}


/*
  send the SYSTEM_TIME message
 */
void GCS_MAVLINK::send_system_time(AP_GPS &gps)
{
    mavlink_msg_system_time_send(
        chan,
        gps.time_epoch_usec(),
        AP_HAL::millis());
}


/*
  send RC_CHANNELS messages
 */
void GCS_MAVLINK::send_radio_in(uint8_t receiver_rssi)
{
    uint32_t now = AP_HAL::millis();
    mavlink_status_t *status = mavlink_get_channel_status(chan);

    uint16_t values[18];
    memset(values, 0, sizeof(values));
    hal.rcin->read(values, 18);

    if (status && (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1)) {
        // for mavlink1 send RC_CHANNELS_RAW, for compatibility with OSD implementations
        mavlink_msg_rc_channels_raw_send(
            chan,
            now,
            0,
            values[0],
            values[1],
            values[2],
            values[3],
            values[4],
            values[5],
            values[6],
            values[7],
            receiver_rssi);
        if (!HAVE_PAYLOAD_SPACE(chan, RC_CHANNELS)) {
            // can't fit RC_CHANNELS
            return;
        }
    }
    mavlink_msg_rc_channels_send(
        chan,
        now,
        hal.rcin->num_channels(),
        values[0],
        values[1],
        values[2],
        values[3],
        values[4],
        values[5],
        values[6],
        values[7],
        values[8],
        values[9],
        values[10],
        values[11],
        values[12],
        values[13],
        values[14],
        values[15],
        values[16],
        values[17],
        receiver_rssi);        
}

void GCS_MAVLINK::send_raw_imu(const AP_InertialSensor &ins, const Compass &compass)
{
    const Vector3f &accel = ins.get_accel(0);
    const Vector3f &gyro = ins.get_gyro(0);
    Vector3f mag;
    if (compass.get_count() >= 1) {
        mag = compass.get_field(0);
    } else {
        mag.zero();
    }

    mavlink_msg_raw_imu_send(
        chan,
        AP_HAL::micros(),
        accel.x * 1000.0f / GRAVITY_MSS,
        accel.y * 1000.0f / GRAVITY_MSS,
        accel.z * 1000.0f / GRAVITY_MSS,
        gyro.x * 1000.0f,
        gyro.y * 1000.0f,
        gyro.z * 1000.0f,
        mag.x,
        mag.y,
        mag.z);

    if (ins.get_gyro_count() <= 1 &&
        ins.get_accel_count() <= 1 &&
        compass.get_count() <= 1) {
        return;
    }
    if (!HAVE_PAYLOAD_SPACE(chan, SCALED_IMU2)) {
        return;
    }
    const Vector3f &accel2 = ins.get_accel(1);
    const Vector3f &gyro2 = ins.get_gyro(1);
    if (compass.get_count() >= 2) {
        mag = compass.get_field(1);
    } else {
        mag.zero();
    }
    mavlink_msg_scaled_imu2_send(
        chan,
        AP_HAL::millis(),
        accel2.x * 1000.0f / GRAVITY_MSS,
        accel2.y * 1000.0f / GRAVITY_MSS,
        accel2.z * 1000.0f / GRAVITY_MSS,
        gyro2.x * 1000.0f,
        gyro2.y * 1000.0f,
        gyro2.z * 1000.0f,
        mag.x,
        mag.y,
        mag.z);        

    if (ins.get_gyro_count() <= 2 &&
        ins.get_accel_count() <= 2 &&
        compass.get_count() <= 2) {
        return;
    }
    if (!HAVE_PAYLOAD_SPACE(chan, SCALED_IMU3)) {
        return;
    }
    const Vector3f &accel3 = ins.get_accel(2);
    const Vector3f &gyro3 = ins.get_gyro(2);
    if (compass.get_count() >= 3) {
        mag = compass.get_field(2);
    } else {
        mag.zero();
    }
    mavlink_msg_scaled_imu3_send(
        chan,
        AP_HAL::millis(),
        accel3.x * 1000.0f / GRAVITY_MSS,
        accel3.y * 1000.0f / GRAVITY_MSS,
        accel3.z * 1000.0f / GRAVITY_MSS,
        gyro3.x * 1000.0f,
        gyro3.y * 1000.0f,
        gyro3.z * 1000.0f,
        mag.x,
        mag.y,
        mag.z);        
}

void GCS_MAVLINK::send_scaled_pressure(AP_Baro &barometer)
{
    uint32_t now = AP_HAL::millis();
    float pressure = barometer.get_pressure(0);
    mavlink_msg_scaled_pressure_send(
        chan,
        now,
        pressure*0.01f, // hectopascal
        (pressure - barometer.get_ground_pressure(0))*0.01f, // hectopascal
        barometer.get_temperature(0)*100); // 0.01 degrees C

    if (barometer.num_instances() > 1 &&
        HAVE_PAYLOAD_SPACE(chan, SCALED_PRESSURE2)) {
        pressure = barometer.get_pressure(1);
        mavlink_msg_scaled_pressure2_send(
            chan,
            now,
            pressure*0.01f, // hectopascal
            (pressure - barometer.get_ground_pressure(1))*0.01f, // hectopascal
            barometer.get_temperature(1)*100); // 0.01 degrees C        
    }

    if (barometer.num_instances() > 2 &&
        HAVE_PAYLOAD_SPACE(chan, SCALED_PRESSURE3)) {
        pressure = barometer.get_pressure(2);
        mavlink_msg_scaled_pressure3_send(
            chan,
            now,
            pressure*0.01f, // hectopascal
            (pressure - barometer.get_ground_pressure(2))*0.01f, // hectopascal
            barometer.get_temperature(2)*100); // 0.01 degrees C        
    }
}

void GCS_MAVLINK::send_sensor_offsets(const AP_InertialSensor &ins, const Compass &compass, AP_Baro &barometer)
{
    // run this message at a much lower rate - otherwise it
    // pointlessly wastes quite a lot of bandwidth
    static uint8_t counter;
    if (counter++ < 10) {
        return;
    }
    counter = 0;

    const Vector3f &mag_offsets = compass.get_offsets(0);
    const Vector3f &accel_offsets = ins.get_accel_offsets(0);
    const Vector3f &gyro_offsets = ins.get_gyro_offsets(0);

    mavlink_msg_sensor_offsets_send(chan,
                                    mag_offsets.x,
                                    mag_offsets.y,
                                    mag_offsets.z,
                                    compass.get_declination(),
                                    barometer.get_pressure(),
                                    barometer.get_temperature()*100,
                                    gyro_offsets.x,
                                    gyro_offsets.y,
                                    gyro_offsets.z,
                                    accel_offsets.x,
                                    accel_offsets.y,
                                    accel_offsets.z);
}

void GCS_MAVLINK::send_ahrs(AP_AHRS &ahrs)
{
    const Vector3f &omega_I = ahrs.get_gyro_drift();
    mavlink_msg_ahrs_send(
        chan,
        omega_I.x,
        omega_I.y,
        omega_I.z,
        0,
        0,
        ahrs.get_error_rp(),
        ahrs.get_error_yaw());
}

/*
  send a statustext message to all active MAVLink connections
 */
void GCS_MAVLINK::send_statustext_all(MAV_SEVERITY severity, const char *fmt, ...)
{
    char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1] {};
    va_list arg_list;
    va_start(arg_list, fmt);
    hal.util->vsnprintf((char *)text, sizeof(text)-1, fmt, arg_list);
    va_end(arg_list);
    text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] = 0;
    gcs().send_statustext(severity, mavlink_active | chan_is_streaming, text);
}

/*
    send a statustext message to specific MAVLink channel, zero indexed
*/
void GCS_MAVLINK::send_statustext_chan(MAV_SEVERITY severity, uint8_t dest_chan, const char *fmt, ...)
{
    char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN] {};
    va_list arg_list;
    va_start(arg_list, fmt);
    hal.util->vsnprintf((char *)text, sizeof(text), fmt, arg_list);
    va_end(arg_list);
    gcs().send_statustext(severity, (1<<dest_chan), text);
}


/*
    send a statustext text string to specific MAVLink bitmask
*/
void GCS::send_statustext(MAV_SEVERITY severity, uint8_t dest_bitmask, const char *text)
{
    if (dataflash_p != nullptr) {
        dataflash_p->Log_Write_Message(text);
    }

    // add statustext message to FrSky lib queue
    if (frsky_telemetry_p != NULL) {
        frsky_telemetry_p->queue_message(severity, text);
    }

    // filter destination ports to only allow active ports.
    statustext_t statustext{};
    statustext.bitmask = (GCS_MAVLINK::active_channel_mask()  | GCS_MAVLINK::streaming_channel_mask() ) & dest_bitmask;
    if (!statustext.bitmask) {
        // nowhere to send
        return;
    }

    statustext.msg.severity = severity;
    strncpy(statustext.msg.text, text, sizeof(statustext.msg.text));

    // The force push will ensure comm links do not block other comm links forever if they fail.
    // If we push to a full buffer then we overwrite the oldest entry, effectively removing the
    // block but not until the buffer fills up.
    _statustext_queue.push_force(statustext);

    // try and send immediately if possible
    service_statustext();

    AP_Notify *notify = AP_Notify::instance();
    if (notify) {
        notify->send_text(text);
    }
}

/*
    send a statustext message to specific MAVLink connections in a bitmask
 */
void GCS::service_statustext(void)
{
    // create bitmask of what mavlink ports we should send this text to.
    // note, if sending to all ports, we only need to store the bitmask for each and the string only once.
    // once we send over a link, clear the port but other busy ports bit may stay allowing for faster links
    // to clear the bit and send quickly but slower links to still store the string. Regardless of mixed
    // bitrates of ports, a maximum of _status_capacity strings can be buffered. Downside
    // is if you have a super slow link mixed with a faster port, if there are _status_capacity
    // strings in the slow queue then the next item can not be queued for the faster link

    if (_statustext_queue.empty()) {
        // nothing to do
        return;
    }

    for (uint8_t idx=0; idx<_status_capacity; ) {
        statustext_t *statustext = _statustext_queue[idx];
        if (statustext == nullptr) {
            break;
        }

        // try and send to all active mavlink ports listed in the statustext.bitmask
        for (uint8_t i=0; i<MAVLINK_COMM_NUM_BUFFERS; i++) {
            uint8_t chan_bit = (1U<<i);
            // logical AND (&) to mask them together
            if (statustext->bitmask & chan_bit) {
                // something is queued on a port and that's the port index we're looped at
                mavlink_channel_t chan_index = (mavlink_channel_t)(MAVLINK_COMM_0+i);
                if (HAVE_PAYLOAD_SPACE(chan_index, STATUSTEXT)) {
                    // we have space so send then clear that channel bit on the mask
                    mavlink_msg_statustext_send(chan_index, statustext->msg.severity, statustext->msg.text);
                    statustext->bitmask &= ~chan_bit;
                }
            }
        }

        if (statustext->bitmask == 0) {
            _statustext_queue.remove(idx);
        } else {
            // move to next index
            idx++;
        }
    }
}

void GCS::reset_cli_timeout()
{
    for (uint8_t i=0; i<num_gcs(); i++) {
        chan(i).reset_cli_timeout();
    }
}

void GCS::send_message(enum ap_message id)
{
    for (uint8_t i=0; i<num_gcs(); i++) {
        if (chan(i).initialised) {
            chan(i).send_message(id);
        }
    }
}

void GCS::data_stream_send()
{
    for (uint8_t i=0; i<num_gcs(); i++) {
        if (chan(i).initialised) {
            chan(i).data_stream_send();
        }
    }
}

void GCS::update(void)
{
    for (uint8_t i=0; i<num_gcs(); i++) {
        if (chan(i).initialised) {
            chan(i).update(_run_cli);
        }
    }
}

void GCS::send_mission_item_reached_message(uint16_t mission_index)
{
    for (uint8_t i=0; i<num_gcs(); i++) {
        if (chan(i).initialised) {
            chan(i).mission_item_reached_index = mission_index;
            chan(i).send_message(MSG_MISSION_ITEM_REACHED);
        }
    }
}

void GCS::setup_uarts(AP_SerialManager &serial_manager)
{
    for (uint8_t i = 1; i < MAVLINK_COMM_NUM_BUFFERS; i++) {
        chan(i).setup_uart(serial_manager, AP_SerialManager::SerialProtocol_MAVLink, i);
    }
}

void GCS::handle_interactive_setup()
{
    if (cli_enabled()) {
        AP_HAL::BetterStream* _cliSerial = cliSerial();
        const char *msg = "\nPress ENTER 3 times to start interactive setup\n";
        _cliSerial->printf("%s\n", msg);
        if (chan(1).initialised && (chan(1).get_uart() != NULL)) {
            chan(1).get_uart()->printf("%s\n", msg);
        }
        if (num_gcs() > 2 && chan(2).initialised && (chan(2).get_uart() != NULL)) {
            chan(2).get_uart()->printf("%s\n", msg);
        }
    }
}

// report battery2 state
void GCS_MAVLINK::send_battery2(const AP_BattMonitor &battery)
{
    if (battery.num_instances() > 1) {
        int16_t current;
        if (battery.has_current(1)) {
            current = battery.current_amps(1) * 100; // 10*mA
        } else {
            current = -1;
        }
        mavlink_msg_battery2_send(chan, battery.voltage(1)*1000, current);
    }
}

/*
  handle a SET_MODE MAVLink message
 */
void GCS_MAVLINK::handle_set_mode(mavlink_message_t* msg, set_mode_fn set_mode)
{
    uint8_t result = MAV_RESULT_FAILED;
    mavlink_set_mode_t packet;
    mavlink_msg_set_mode_decode(msg, &packet);

    // only accept custom modes because there is no easy mapping from Mavlink flight modes to AC flight modes
    if (packet.base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) {
        if (set_mode(packet.custom_mode)) {
            result = MAV_RESULT_ACCEPTED;
        }
    } else if (packet.base_mode == MAV_MODE_FLAG_DECODE_POSITION_SAFETY) {
        // set the safety switch position. Must be in a command by itself
        if (packet.custom_mode == 0) {
            // turn safety off (pwm outputs flow to the motors)
            hal.rcout->force_safety_off();
            result = MAV_RESULT_ACCEPTED;
        } else if (packet.custom_mode == 1) {
            // turn safety on (no pwm outputs to the motors)
            if (hal.rcout->force_safety_on()) {
                result = MAV_RESULT_ACCEPTED;
            }
        }
    }

    // send ACK or NAK
    mavlink_msg_command_ack_send_buf(msg, chan, MAVLINK_MSG_ID_SET_MODE, result);
}

#if AP_AHRS_NAVEKF_AVAILABLE
/*
  send OPTICAL_FLOW message
 */
void GCS_MAVLINK::send_opticalflow(AP_AHRS_NavEKF &ahrs, const OpticalFlow &optflow)
{
    // exit immediately if no optical flow sensor or not healthy
    if (!optflow.healthy()) {
        return;
    }

    // get rates from sensor
    const Vector2f &flowRate = optflow.flowRate();
    const Vector2f &bodyRate = optflow.bodyRate();
    float hagl = 0;

    if (ahrs.have_inertial_nav()) {

        ahrs.get_hagl(hagl);
    }

    // populate and send message
    mavlink_msg_optical_flow_send(
        chan,
        AP_HAL::millis(),
        0, // sensor id is zero
        flowRate.x,
        flowRate.y,
        bodyRate.x,
        bodyRate.y,
        optflow.quality(),
        hagl,  // ground distance (in meters) set to zero
        flowRate.x,
        flowRate.y);
}
#endif

/*
  send AUTOPILOT_VERSION packet
 */
void GCS_MAVLINK::send_autopilot_version(uint8_t major_version, uint8_t minor_version, uint8_t patch_version, uint8_t version_type) const
{
    uint32_t flight_sw_version = 0;
    uint32_t middleware_sw_version = 0;
    uint32_t os_sw_version = 0;
    uint32_t board_version = 0;
    uint8_t flight_custom_version[8];
    uint8_t middleware_custom_version[8];
    uint8_t os_custom_version[8];
    uint16_t vendor_id = 0;
    uint16_t product_id = 0;
    uint64_t uid = 0;
    
    flight_sw_version = major_version << (8*3) | \
                        minor_version << (8*2) | \
                        patch_version << (8*1) | \
                        version_type  << (8*0);

#if defined(GIT_VERSION)
    strncpy((char *)flight_custom_version, GIT_VERSION, 8);
#else
    memset(middleware_custom_version,0,8);
#endif

#if defined(PX4_GIT_VERSION)
    strncpy((char *)middleware_custom_version, PX4_GIT_VERSION, 8);
#else
    memset(middleware_custom_version,0,8);
#endif
    
#if defined(NUTTX_GIT_VERSION)
    strncpy((char *)os_custom_version, NUTTX_GIT_VERSION, 8);
#else
    memset(os_custom_version,0,8);
#endif
    
    mavlink_msg_autopilot_version_send(
        chan,
        hal.util->get_capabilities(),
        flight_sw_version,
        middleware_sw_version,
        os_sw_version,
        board_version,
        flight_custom_version,
        middleware_custom_version,
        os_custom_version,
        vendor_id,
        product_id,
        uid
    );
}


/*
  send LOCAL_POSITION_NED message
 */
void GCS_MAVLINK::send_local_position(const AP_AHRS &ahrs) const
{
    Vector3f local_position, velocity;
    if (!ahrs.get_relative_position_NED_home(local_position) ||
        !ahrs.get_velocity_NED(velocity)) {
        // we don't know the position and velocity
        return;
    }

    mavlink_msg_local_position_ned_send(
        chan,
        AP_HAL::millis(),
        local_position.x,
        local_position.y,
        local_position.z,
        velocity.x,
        velocity.y,
        velocity.z);
}

/*
  send VIBRATION message
 */
void GCS_MAVLINK::send_vibration(const AP_InertialSensor &ins) const
{
    Vector3f vibration = ins.get_vibration_levels();

    mavlink_msg_vibration_send(
        chan,
        AP_HAL::micros64(),
        vibration.x,
        vibration.y,
        vibration.z,
        ins.get_accel_clip_count(0),
        ins.get_accel_clip_count(1),
        ins.get_accel_clip_count(2));
}

void GCS_MAVLINK::send_home(const Location &home) const
{
    if (HAVE_PAYLOAD_SPACE(chan, HOME_POSITION)) {
        const float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
        mavlink_msg_home_position_send(
            chan,
            home.lat,
            home.lng,
            home.alt * 10,
            0.0f, 0.0f, 0.0f,
            q,
            0.0f, 0.0f, 0.0f);
    }
}

void GCS_MAVLINK::send_home_all(const Location &home)
{
    const float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    for (uint8_t i=0; i<MAVLINK_COMM_NUM_BUFFERS; i++) {
        if ((1U<<i) & mavlink_active) {
            mavlink_channel_t chan = (mavlink_channel_t)(MAVLINK_COMM_0+i);
            if (HAVE_PAYLOAD_SPACE(chan, HOME_POSITION)) {
                mavlink_msg_home_position_send(
                    chan,
                    home.lat,
                    home.lng,
                    home.alt * 10,
                    0.0f, 0.0f, 0.0f,
                    q,
                    0.0f, 0.0f, 0.0f);
            }
        }
    }
}

/*
  wrapper for sending heartbeat
 */
void GCS_MAVLINK::send_heartbeat(uint8_t type, uint8_t base_mode, uint32_t custom_mode, uint8_t system_status)
{
    mavlink_msg_heartbeat_send(
        chan,
        type,
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode,
        custom_mode,
        system_status);
}

float GCS_MAVLINK::adjust_rate_for_stream_trigger(enum streams stream_num)
{
    // send at a much lower rate while handling waypoints and
    // parameter sends
    if ((stream_num != STREAM_PARAMS) && 
        (waypoint_receiving || _queued_parameter != nullptr)) {
        return 0.25f;
    }

    return 1.0f;
}

// are we still delaying telemetry to try to avoid Xbee bricking?
bool GCS_MAVLINK::telemetry_delayed(mavlink_channel_t _chan)
{
    uint32_t tnow = AP_HAL::millis() >> 10;
    if (tnow > telem_delay()) {
        return false;
    }
    if (_chan == MAVLINK_COMM_0 && hal.gpio->usb_connected()) {
        // this is USB telemetry, so won't be an Xbee
        return false;
    }
    // we're either on the 2nd UART, or no USB cable is connected
    // we need to delay telemetry by the TELEM_DELAY time
    return true;
}


/*
  send SERVO_OUTPUT_RAW
 */
void GCS_MAVLINK::send_servo_output_raw(bool hil)
{
    uint16_t values[16] {};
    if (hil) {
        for (uint8_t i=0; i<16; i++) {
            values[i] = SRV_Channels::srv_channel(i)->get_output_pwm();
        }
    } else {
        hal.rcout->read(values, 16);
    }
    for (uint8_t i=0; i<16; i++) {
        if (values[i] == 65535) {
            values[i] = 0;
        }
    }    
    mavlink_msg_servo_output_raw_send(
            chan,
            AP_HAL::micros(),
            0,     // port
            values[0],  values[1],  values[2],  values[3],
            values[4],  values[5],  values[6],  values[7],
            values[8],  values[9],  values[10], values[11],
            values[12], values[13], values[14], values[15]);
}


void GCS_MAVLINK::send_collision_all(const AP_Avoidance::Obstacle &threat, MAV_COLLISION_ACTION behaviour)
{
    for (uint8_t i=0; i<MAVLINK_COMM_NUM_BUFFERS; i++) {
        if ((1U<<i) & mavlink_active) {
            mavlink_channel_t chan = (mavlink_channel_t)(MAVLINK_COMM_0+i);
            if (comm_get_txspace(chan) >= MAVLINK_NUM_NON_PAYLOAD_BYTES + MAVLINK_MSG_ID_COLLISION) {
                mavlink_msg_collision_send(
                    chan,
                    MAV_COLLISION_SRC_ADSB,
                    threat.src_id,
                    behaviour,
                    threat.threat_level,
                    threat.time_to_closest_approach,
                    threat.closest_approach_z,
                    threat.closest_approach_xy
                    );
            }
        }
    }
}

void GCS_MAVLINK::send_accelcal_vehicle_position(uint32_t position)
{
    if (HAVE_PAYLOAD_SPACE(chan, COMMAND_LONG)) {
        mavlink_msg_command_long_send(
            chan,
            0,
            0,
            MAV_CMD_ACCELCAL_VEHICLE_POS,
            0,
            (float) position,
            0, 0, 0, 0, 0, 0);
    }
}

/*
  handle a MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN command 

  Optionally disable PX4IO overrides. This is done for quadplanes to
  prevent the mixer running while rebooting which can start the VTOL
  motors. That can be dangerous when a preflight reboot is done with
  the pilot close to the aircraft and can also damage the aircraft
 */
uint8_t GCS_MAVLINK::handle_preflight_reboot(const mavlink_command_long_t &packet, bool disable_overrides)
{
    if (is_equal(packet.param1,1.0f) || is_equal(packet.param1,3.0f)) {
        if (disable_overrides) {
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
            // disable overrides while rebooting
            int px4io_fd = open("/dev/px4io", 0);
            if (px4io_fd >= 0) {
                // disable OVERRIDES so we don't run the mixer while
                // rebooting
                if (ioctl(px4io_fd, PWM_SERVO_SET_OVERRIDE_OK, 0) != 0) {
                    hal.console->printf("SET_OVERRIDE_OK failed\n");
                }
                if (ioctl(px4io_fd, PWM_SERVO_SET_OVERRIDE_IMMEDIATE, 0) != 0) {
                    hal.console->printf("SET_OVERRIDE_IMMEDIATE failed\n");
                }
                close(px4io_fd);
            }
#endif
        }

        // force safety on 
        hal.rcout->force_safety_on();
        hal.rcout->force_safety_no_wait();
        hal.scheduler->delay(200);

        // when packet.param1 == 3 we reboot to hold in bootloader
        bool hold_in_bootloader = is_equal(packet.param1,3.0f);
        hal.scheduler->reboot(hold_in_bootloader);
        return MAV_RESULT_ACCEPTED;
    }
    return MAV_RESULT_UNSUPPORTED;
}


/*
  handle a R/C bind request (for spektrum)
 */
uint8_t GCS_MAVLINK::handle_rc_bind(const mavlink_command_long_t &packet)
{
    // initiate bind procedure. We accept the DSM type from either
    // param1 or param2 due to a past mixup with what parameter is the
    // right one
    if (!hal.rcin->rc_bind(packet.param2>0?packet.param2:packet.param1)) {
        return MAV_RESULT_FAILED;
    }
    return MAV_RESULT_ACCEPTED;
}

/*
  return a timesync request
  Sends back ts1 as received, and tc1 is the local timestamp in usec
 */
void GCS_MAVLINK::handle_timesync(mavlink_message_t *msg)
{
    // decode incoming timesync message
    mavlink_timesync_t tsync;
    mavlink_msg_timesync_decode(msg, &tsync);

    // ignore messages in which tc1 field (timestamp 1) has already been filled in
    if (tsync.tc1 != 0) {
        return;
    }

    // create new timesync struct with tc1 field as system time in nanoseconds
    mavlink_timesync_t rsync;
    rsync.tc1 = AP_HAL::micros64() * 1000;
    rsync.ts1 = tsync.ts1;

    // respond with a timesync message
    mavlink_msg_timesync_send(
        chan,
        rsync.tc1,
        rsync.ts1
        );
}


/*
  handle messages which don't require vehicle specific data
 */
void GCS_MAVLINK::handle_common_message(mavlink_message_t *msg)
{
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_SETUP_SIGNING:
        handle_setup_signing(msg);
        break;
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
        handle_param_request_read(msg);
        break;
    case MAVLINK_MSG_ID_DEVICE_OP_READ:
        handle_device_op_read(msg);
        break;
    case MAVLINK_MSG_ID_DEVICE_OP_WRITE:
        handle_device_op_write(msg);
        break;
    case MAVLINK_MSG_ID_TIMESYNC:
        handle_timesync(msg);
        break;
    case MAVLINK_MSG_ID_LOG_REQUEST_LIST:
        /* fall through */
    case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
        /* fall through */
    case MAVLINK_MSG_ID_LOG_ERASE:
        /* fall through */
    case MAVLINK_MSG_ID_LOG_REQUEST_END:
        /* fall through */
    case MAVLINK_MSG_ID_REMOTE_LOG_BLOCK_STATUS:
        DataFlash_Class::instance()->handle_mavlink_msg(*this, msg);
        break;
    }
}

GCS &gcs()
{
    return *GCS::instance();
}
