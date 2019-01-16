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
#include <AP_RangeFinder/RangeFinder_Backend.h>
#include <AP_Airspeed/AP_Airspeed.h>
#include <AP_Gripper/AP_Gripper.h>
#include <AP_BLHeli/AP_BLHeli.h>
#include <AP_Common/Semaphore.h>
#include <AP_Scheduler/AP_Scheduler.h>

#include "GCS.h"

#include <stdio.h>
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
#include <drivers/drv_pwm_output.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#endif

#if HAL_RCINPUT_WITH_AP_RADIO
#include <AP_Radio/AP_Radio.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <SITL/SITL.h>
#endif

extern const AP_HAL::HAL& hal;

uint32_t GCS_MAVLINK::last_radio_status_remrssi_ms;
uint8_t GCS_MAVLINK::mavlink_active = 0;
uint8_t GCS_MAVLINK::chan_is_streaming = 0;
uint32_t GCS_MAVLINK::reserve_param_space_start_ms;

// private channels are ones used for point-to-point protocols, and
// don't get broadcasts or fwded packets
uint8_t GCS_MAVLINK::mavlink_private = 0;

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
    _queued_parameter = nullptr;

    snprintf(_perf_packet_name, sizeof(_perf_packet_name), "GCS_Packet_%u", chan);
    _perf_packet = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, _perf_packet_name);

    snprintf(_perf_update_name, sizeof(_perf_update_name), "GCS_Update_%u", chan);
    _perf_update = hal.util->perf_alloc(AP_HAL::Util::PC_ELAPSED, _perf_update_name);

    initialised = true;
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
GCS_MAVLINK::queued_mission_request_send()
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

void GCS_MAVLINK::send_meminfo(void)
{
    unsigned __brkval = 0;
    uint32_t memory = hal.util->available_memory();
    mavlink_msg_meminfo_send(chan, __brkval, MIN(memory, 0xFFFFU), memory);
}

// report power supply status
void GCS_MAVLINK::send_power_status(void)
{
    if (!vehicle_initialised()) {
        // avoid unnecessary errors being reported to user
        return;
    }
    mavlink_msg_power_status_send(chan,
                                  hal.analogin->board_voltage() * 1000,
                                  hal.analogin->servorail_voltage() * 1000,
                                  hal.analogin->power_status_flags());
}

void GCS_MAVLINK::send_battery_status(const AP_BattMonitor &battery,
                                      const uint8_t instance) const
{
    // catch the battery backend not supporting the required number of cells
    static_assert(sizeof(AP_BattMonitor::cells) >= (sizeof(uint16_t) * MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_LEN),
                  "Not enough battery cells for the MAVLink message");

    float temp;
    bool got_temperature = battery.get_temperature(temp, instance);

    // ensure we always send a voltage estimate to the GCS, because not all battery monitors monitor individual cells
    // as a work around for this we create a set of fake cells to be used if the backend doesn't provide direct monitoring
    // the GCS can then recover the pack voltage by summing all non ignored cell values. Because this is looped we can
    // report a pack up to 655.34 V with this method
    AP_BattMonitor::cells fake_cells;
    if (!battery.has_cell_voltages(instance)) {
        float voltage = battery.voltage(instance) * 1e3f;
        for (uint8_t i = 0; i < MAVLINK_MSG_BATTERY_STATUS_FIELD_VOLTAGES_LEN; i++) {
          if (voltage < 0.001f) {
              // too small to send to the GCS, set it to the no cell value
              fake_cells.cells[i] = UINT16_MAX;
          } else {
              fake_cells.cells[i] = MIN(voltage, 65534.0f); // Can't send more then UINT16_MAX - 1 in a cell
              voltage -= 65534.0f;
          }
        }
    }

    mavlink_msg_battery_status_send(chan,
                                    instance, // id
                                    MAV_BATTERY_FUNCTION_UNKNOWN, // function
                                    MAV_BATTERY_TYPE_UNKNOWN, // type
                                    got_temperature ? ((int16_t) (temp * 100)) : INT16_MAX, // temperature. INT16_MAX if unknown
                                    battery.has_cell_voltages(instance) ? battery.get_cell_voltages(instance).cells : fake_cells.cells, // cell voltages
                                    battery.has_current(instance) ? battery.current_amps(instance) * 100 : -1, // current in centiampere
                                    battery.has_current(instance) ? battery.consumed_mah(instance) : -1,       // total consumed current in milliampere.hour
                                    battery.has_consumed_energy(instance) ? battery.consumed_wh(instance) * 36 : -1, // consumed energy in hJ (hecto-Joules)
                                    battery.capacity_remaining_pct(instance),
                                    0, // time remaining, seconds (not provided)
                                    MAV_BATTERY_CHARGE_STATE_UNDEFINED);
}

// returns true if all battery instances were reported
bool GCS_MAVLINK::send_battery_status() const
{
    const AP_BattMonitor &battery = AP::battery();

    for(uint8_t i = 0; i < battery.num_instances(); i++) {
        CHECK_PAYLOAD_SIZE(BATTERY_STATUS);
        send_battery_status(battery, i);
    }
    return true;
}

void GCS_MAVLINK::send_distance_sensor(const AP_RangeFinder_Backend *sensor, const uint8_t instance) const
{
    if (!sensor->has_data()) {
        return;
    }

    mavlink_msg_distance_sensor_send(
        chan,
        AP_HAL::millis(),                        // time since system boot TODO: take time of measurement
        sensor->min_distance_cm(),               // minimum distance the sensor can measure in centimeters
        sensor->max_distance_cm(),               // maximum distance the sensor can measure in centimeters
        sensor->distance_cm(),                   // current distance reading
        sensor->get_mav_distance_sensor_type(),  // type from MAV_DISTANCE_SENSOR enum
        instance,                                // onboard ID of the sensor == instance
        sensor->orientation(),                   // direction the sensor faces from MAV_SENSOR_ORIENTATION enum
        0);                                      // Measurement covariance in centimeters, 0 for unknown / invalid readings
}
// send any and all distance_sensor messages.  This starts by sending
// any distance sensors not used by a Proximity sensor, then sends the
// proximity sensor ones.
void GCS_MAVLINK::send_distance_sensor() const
{
    RangeFinder *rangefinder = RangeFinder::get_singleton();
    if (rangefinder == nullptr) {
        return;
    }

    // if we have a proximity backend that utilizes rangefinders cull
    // sending them here, and allow the later proximity code to manage
    // them
    bool filter_possible_proximity_sensors = false;
    AP_Proximity *proximity = AP_Proximity::get_singleton();
    if (proximity != nullptr) {
        for (uint8_t i = 0; i < proximity->num_sensors(); i++) {
            if (proximity->get_type(i) == AP_Proximity::Proximity_Type_RangeFinder) {
                filter_possible_proximity_sensors = true;
            }
        }
    }

    for (uint8_t i = 0; i < RANGEFINDER_MAX_INSTANCES; i++) {
        if (!HAVE_PAYLOAD_SPACE(chan, DISTANCE_SENSOR)) {
            return;
        }
        AP_RangeFinder_Backend *sensor = rangefinder->get_backend(i);
        if (sensor == nullptr) {
            continue;
        }
        enum Rotation orient = sensor->orientation();
        if (!filter_possible_proximity_sensors ||
            (orient > ROTATION_YAW_315 && orient != ROTATION_PITCH_90)) {
            send_distance_sensor(sensor, i);
        }
    }

    send_proximity();
}

void GCS_MAVLINK::send_rangefinder() const
{
    RangeFinder *rangefinder = RangeFinder::get_singleton();
    if (rangefinder == nullptr) {
        return;
    }
    AP_RangeFinder_Backend *s = rangefinder->find_instance(ROTATION_PITCH_270);
    if (s == nullptr) {
        return;
    }
    mavlink_msg_rangefinder_send(
            chan,
            s->distance_cm() * 0.01f,
            s->voltage_mv() * 0.001f);
}

void GCS_MAVLINK::send_proximity() const
{
    AP_Proximity *proximity = AP_Proximity::get_singleton();
    if (proximity == nullptr || proximity->get_status() == AP_Proximity::Proximity_NotConnected) {
        return; // this is wrong, but pretend we sent data and don't requeue
    }

    const uint16_t dist_min = (uint16_t)(proximity->distance_min() * 100.0f); // minimum distance the sensor can measure in centimeters
    const uint16_t dist_max = (uint16_t)(proximity->distance_max() * 100.0f); // maximum distance the sensor can measure in centimeters
    // send horizontal distances
    AP_Proximity::Proximity_Distance_Array dist_array;
    if (proximity->get_horizontal_distances(dist_array)) {
        for (uint8_t i = 0; i < PROXIMITY_MAX_DIRECTION; i++) {
            if (!HAVE_PAYLOAD_SPACE(chan, DISTANCE_SENSOR)) {
                return;
            }
            mavlink_msg_distance_sensor_send(
                    chan,
                    AP_HAL::millis(),                               // time since system boot
                    dist_min,                                       // minimum distance the sensor can measure in centimeters
                    dist_max,                                       // maximum distance the sensor can measure in centimeters
                    (uint16_t)(dist_array.distance[i] * 100.0f),    // current distance reading
                    MAV_DISTANCE_SENSOR_LASER,                      // type from MAV_DISTANCE_SENSOR enum
                    PROXIMITY_SENSOR_ID_START + i,                  // onboard ID of the sensor
                    dist_array.orientation[i],                      // direction the sensor faces from MAV_SENSOR_ORIENTATION enum
                    0);                                             // Measurement covariance in centimeters, 0 for unknown / invalid readings
        }
    }

    // send upward distance
    float dist_up;
    if (proximity->get_upward_distance(dist_up)) {
        if (!HAVE_PAYLOAD_SPACE(chan, DISTANCE_SENSOR)) {
            return;
        }
        mavlink_msg_distance_sensor_send(
                chan,
                AP_HAL::millis(),                                         // time since system boot
                dist_min,                                                 // minimum distance the sensor can measure in centimeters
                dist_max,                                                 // maximum distance the sensor can measure in centimeters
                (uint16_t)(dist_up * 100.0f),                             // current distance reading
                MAV_DISTANCE_SENSOR_LASER,                                // type from MAV_DISTANCE_SENSOR enum
                PROXIMITY_SENSOR_ID_START + PROXIMITY_MAX_DIRECTION + 1,  // onboard ID of the sensor
                MAV_SENSOR_ROTATION_PITCH_90,                             // direction upwards
                0);                                                       // Measurement covariance in centimeters, 0 for unknown / invalid readings
    }
}

// report AHRS2 state
void GCS_MAVLINK::send_ahrs2()
{
#if AP_AHRS_NAVEKF_AVAILABLE
    const AP_AHRS &ahrs = AP::ahrs();
    Vector3f euler;
    struct Location loc {};
    if (ahrs.get_secondary_attitude(euler) ||
        ahrs.get_secondary_position(loc)) {
        mavlink_msg_ahrs2_send(chan,
                               euler.x,
                               euler.y,
                               euler.z,
                               loc.alt*1.0e-2f,
                               loc.lat,
                               loc.lng);
    }
#endif
}

void GCS_MAVLINK::send_ahrs3()
{
#if AP_AHRS_NAVEKF_AVAILABLE
    const NavEKF2 &ekf2 = AP::ahrs_navekf().get_NavEKF2_const();
    if (ekf2.activeCores() > 0 &&
        HAVE_PAYLOAD_SPACE(chan, AHRS3)) {
        struct Location loc {};
        ekf2.getLLH(loc);
        Vector3f euler;
        ekf2.getEulerAngles(-1,euler);
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

    waypoint_dest_sysid = msg->sysid;       // record system id of GCS who wants to upload the mission
    waypoint_dest_compid = msg->compid;     // record component id of GCS who wants to upload the mission
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
        mavlink_msg_mission_ack_send(chan, msg->sysid, msg->compid, MAV_MISSION_ACCEPTED,
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

    waypoint_dest_sysid = msg->sysid;       // record system id of GCS who wants to partially update the mission
    waypoint_dest_compid = msg->compid;     // record component id of GCS who wants to partially update the mission
}


/*
  pass mavlink messages to the AP_Mount singleton
 */
void GCS_MAVLINK::handle_mount_message(const mavlink_message_t *msg)
{
    AP_Mount *mount = AP::mount();
    if (mount == nullptr) {
        return;
    }
    mount->handle_message(chan, msg);
}

/*
  pass parameter value messages through to mount library
 */
void GCS_MAVLINK::handle_param_value(mavlink_message_t *msg)
{
    AP_Mount *mount = AP::mount();
    if (mount == nullptr) {
        return;
    }
    mount->handle_param_value(msg);
}

void GCS_MAVLINK::send_textv(MAV_SEVERITY severity, const char *fmt, va_list arg_list)
{
    char text[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1];
    hal.util->vsnprintf(text, sizeof(text), fmt, arg_list);
    gcs().send_statustext(severity, (1<<chan), text);
}
void GCS_MAVLINK::send_text(MAV_SEVERITY severity, const char *fmt, ...)
{
    va_list arg_list;
    va_start(arg_list, fmt);
    send_textv(severity, fmt, arg_list);
    va_end(arg_list);
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

#if GCS_DEBUG_SEND_MESSAGE_TIMINGS
    if (stream_slowdown > max_slowdown) {
        max_slowdown = stream_slowdown;
    }
#endif

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
        // if we have enough space, then send the next WP request immediately
        if (HAVE_PAYLOAD_SPACE(chan, MISSION_ITEM)) {
            queued_mission_request_send();
        } else {
            send_message(MSG_NEXT_MISSION_REQUEST);
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

ap_message GCS_MAVLINK::mavlink_id_to_ap_message_id(const uint32_t mavlink_id) const
{
    // MSG_NEXT_MISSION_REQUEST doesn't correspond to a mavlink message directly.
    // It is used to request the next waypoint after receiving one.

    // MSG_NEXT_PARAM doesn't correspond to a mavlink message directly.
    // It is used to send the next parameter in a stream after sending one

    // MSG_NAMED_FLOAT messages can't really be "streamed"...

    // this list is ordered by AP_MESSAGE ID - the value being returned:
    static const struct {
        uint32_t mavlink_id;
        ap_message msg_id;
    } map[] {
        { MAVLINK_MSG_ID_HEARTBEAT,             MSG_HEARTBEAT},
        { MAVLINK_MSG_ID_ATTITUDE,              MSG_ATTITUDE},
        { MAVLINK_MSG_ID_GLOBAL_POSITION_INT,   MSG_LOCATION},
        { MAVLINK_MSG_ID_SYS_STATUS,            MSG_SYS_STATUS},
        { MAVLINK_MSG_ID_POWER_STATUS,          MSG_POWER_STATUS},
        { MAVLINK_MSG_ID_MEMINFO,               MSG_MEMINFO},
        { MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT, MSG_NAV_CONTROLLER_OUTPUT},
        { MAVLINK_MSG_ID_MISSION_CURRENT,       MSG_CURRENT_WAYPOINT},
        { MAVLINK_MSG_ID_VFR_HUD,               MSG_VFR_HUD},
        { MAVLINK_MSG_ID_SERVO_OUTPUT_RAW,      MSG_SERVO_OUTPUT_RAW},
        { MAVLINK_MSG_ID_RC_CHANNELS,           MSG_RADIO_IN},
        { MAVLINK_MSG_ID_RAW_IMU,               MSG_RAW_IMU},
        { MAVLINK_MSG_ID_SCALED_IMU,            MSG_SCALED_IMU},
        { MAVLINK_MSG_ID_SCALED_IMU2,           MSG_SCALED_IMU2},
        { MAVLINK_MSG_ID_SCALED_IMU3,           MSG_SCALED_IMU3},
        { MAVLINK_MSG_ID_SCALED_PRESSURE,       MSG_SCALED_PRESSURE},
        { MAVLINK_MSG_ID_SCALED_PRESSURE2,      MSG_SCALED_PRESSURE2},
        { MAVLINK_MSG_ID_SCALED_PRESSURE3,      MSG_SCALED_PRESSURE3},
        { MAVLINK_MSG_ID_SENSOR_OFFSETS,        MSG_SENSOR_OFFSETS},
        { MAVLINK_MSG_ID_GPS_RAW_INT,           MSG_GPS_RAW},
        { MAVLINK_MSG_ID_GPS_RTK,               MSG_GPS_RTK},
        { MAVLINK_MSG_ID_GPS2_RAW,              MSG_GPS2_RAW},
        { MAVLINK_MSG_ID_GPS2_RTK,              MSG_GPS2_RTK},
        { MAVLINK_MSG_ID_SYSTEM_TIME,           MSG_SYSTEM_TIME},
        { MAVLINK_MSG_ID_RC_CHANNELS_SCALED,    MSG_SERVO_OUT},
        { MAVLINK_MSG_ID_PARAM_VALUE,           MSG_NEXT_PARAM},
        { MAVLINK_MSG_ID_FENCE_STATUS,          MSG_FENCE_STATUS},
        { MAVLINK_MSG_ID_AHRS,                  MSG_AHRS},
        { MAVLINK_MSG_ID_SIMSTATE,              MSG_SIMSTATE},
        { MAVLINK_MSG_ID_AHRS2,                 MSG_AHRS2},
        { MAVLINK_MSG_ID_AHRS3,                 MSG_AHRS3},
        { MAVLINK_MSG_ID_HWSTATUS,              MSG_HWSTATUS},
        { MAVLINK_MSG_ID_WIND,                  MSG_WIND},
        { MAVLINK_MSG_ID_RANGEFINDER,           MSG_RANGEFINDER},
        { MAVLINK_MSG_ID_DISTANCE_SENSOR,       MSG_DISTANCE_SENSOR},
            // request also does report:
        { MAVLINK_MSG_ID_TERRAIN_REQUEST,       MSG_TERRAIN},
        { MAVLINK_MSG_ID_BATTERY2,              MSG_BATTERY2},
        { MAVLINK_MSG_ID_CAMERA_FEEDBACK,       MSG_CAMERA_FEEDBACK},
        { MAVLINK_MSG_ID_MOUNT_STATUS,          MSG_MOUNT_STATUS},
        { MAVLINK_MSG_ID_OPTICAL_FLOW,          MSG_OPTICAL_FLOW},
        { MAVLINK_MSG_ID_GIMBAL_REPORT,         MSG_GIMBAL_REPORT},
        { MAVLINK_MSG_ID_MAG_CAL_PROGRESS,      MSG_MAG_CAL_PROGRESS},
        { MAVLINK_MSG_ID_MAG_CAL_REPORT,        MSG_MAG_CAL_REPORT},
        { MAVLINK_MSG_ID_EKF_STATUS_REPORT,     MSG_EKF_STATUS_REPORT},
        { MAVLINK_MSG_ID_LOCAL_POSITION_NED,    MSG_LOCAL_POSITION},
        { MAVLINK_MSG_ID_PID_TUNING,            MSG_PID_TUNING},
        { MAVLINK_MSG_ID_VIBRATION,             MSG_VIBRATION},
        { MAVLINK_MSG_ID_RPM,                   MSG_RPM},
        { MAVLINK_MSG_ID_MISSION_ITEM_REACHED,  MSG_MISSION_ITEM_REACHED},
        { MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT,  MSG_POSITION_TARGET_GLOBAL_INT},
        { MAVLINK_MSG_ID_ADSB_VEHICLE,          MSG_ADSB_VEHICLE},
        { MAVLINK_MSG_ID_BATTERY_STATUS,        MSG_BATTERY_STATUS},
        { MAVLINK_MSG_ID_AOA_SSA,               MSG_AOA_SSA},
        { MAVLINK_MSG_ID_DEEPSTALL,             MSG_LANDING},
            };

    for (uint8_t i=0; i<ARRAY_SIZE(map); i++) {
        if (map[i].mavlink_id == mavlink_id) {
            return map[i].msg_id;
        }
    }
    return MSG_LAST;
}

bool GCS_MAVLINK::set_mavlink_message_id_interval(const uint32_t mavlink_id,
                                                  const uint16_t interval_ms)
{
    const ap_message id = mavlink_id_to_ap_message_id(mavlink_id);
    if (id == MSG_LAST) {
        gcs().send_text(MAV_SEVERITY_INFO, "No ap_message for mavlink id (%u)", mavlink_id);
        return false;
    }
    return set_ap_message_interval(id, interval_ms);
}

bool GCS_MAVLINK::should_send_message_in_delay_callback(const ap_message id) const
{
    // No ID we return true for may take more than a few hundred
    // microseconds to return!

    if (id == MSG_HEARTBEAT || id == MSG_NEXT_PARAM) {
        return true;
    }

    if (in_hil_mode()) {
        // in HIL we need to keep sending servo values to ensure
        // the simulator doesn't pause, otherwise our sensor
        // calibration could stall
        if (id == MSG_SERVO_OUT ||
            id == MSG_SERVO_OUTPUT_RAW) {
            return true;
        }
    }

    return false;
}

uint16_t GCS_MAVLINK::get_reschedule_interval_ms(const deferred_message_bucket_t &deferred) const
{
    uint32_t interval_ms = deferred.interval_ms;

    // add in adjustments for streamrate-slowdown (e.g. based
    // on feedback from telemetry radio on its state).
    // slowdown is basically 50ths of a second
    interval_ms += stream_slowdown * 20;

    // slow most messages down if we're transfering parameters or
    // waypoints:
    if (_queued_parameter) {
        // we are sending parameters, penalize streams:
        interval_ms *= 4;
    }
    if (waypoint_receiving) {
        // we are sending requests for waypoints, penalize streams:
        interval_ms *= 4;
    }

    if (interval_ms > 60000) {
        return 60000;
    }

    return interval_ms;
}

// typical runtime on fmuv3: 5 microseconds for 3 buckets
void GCS_MAVLINK::find_next_bucket_to_send()
{
#if GCS_DEBUG_SEND_MESSAGE_TIMINGS
    void *data = hal.scheduler->disable_interrupts_save();
    uint32_t start_us = AP_HAL::micros();
#endif

    const uint16_t now16_ms{AP_HAL::millis16()};

    // all done sending this bucket... find another bucket...
    sending_bucket_id = no_bucket_to_send;
    uint16_t ms_before_send_next_bucket_to_send = UINT16_MAX;
    for (uint8_t i=0; i<ARRAY_SIZE(deferred_message_bucket); i++) {
        if (deferred_message_bucket[i].ap_message_ids.count() == 0) {
            // no entries
            continue;
        }
        const uint16_t interval = get_reschedule_interval_ms(deferred_message_bucket[i]);
        const uint16_t ms_since_last_sent = now16_ms - deferred_message_bucket[i].last_sent_ms;
        uint16_t ms_before_send_this_bucket;
        if (ms_since_last_sent > interval) {
            // should already have sent this bucket!
            ms_before_send_this_bucket = 0;
        } else {
            ms_before_send_this_bucket = interval - ms_since_last_sent;
        }
        if (ms_before_send_this_bucket < ms_before_send_next_bucket_to_send) {
            sending_bucket_id = i;
            ms_before_send_next_bucket_to_send = ms_before_send_this_bucket;
        }
    }
    if (sending_bucket_id != no_bucket_to_send) {
        bucket_message_ids_to_send = deferred_message_bucket[sending_bucket_id].ap_message_ids;
    } else {
        bucket_message_ids_to_send.clearall();
    }

#if GCS_DEBUG_SEND_MESSAGE_TIMINGS
    uint32_t delta_us = AP_HAL::micros() - start_us;
    hal.scheduler->restore_interrupts(data);
    if (delta_us > try_send_message_stats.fnbts_maxtime) {
        try_send_message_stats.fnbts_maxtime = delta_us;
    }
#endif
}

ap_message GCS_MAVLINK::next_deferred_bucket_message_to_send()
{
    if (sending_bucket_id == no_bucket_to_send) {
        // could happen if all streamrates are zero?
        return no_message_to_send;
    }

    const uint16_t now16_ms = AP_HAL::millis16();
    const uint16_t ms_since_last_sent = now16_ms - deferred_message_bucket[sending_bucket_id].last_sent_ms;
    if (ms_since_last_sent < get_reschedule_interval_ms(deferred_message_bucket[sending_bucket_id])) {
        // not time to send this bucket
        return no_message_to_send;
    }

    const int16_t next = bucket_message_ids_to_send.first_set();
    if (next == -1) {
        // should not happen
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("next_deferred_bucket_message_to_send called on empty bucket");
#endif
        find_next_bucket_to_send();
        return no_message_to_send;
    }
    return (ap_message)next;
}

// call try_send_message if appropriate.  Incorporates debug code to
// record how long it takes to send a message.  try_send_message is
// expected to be overridden, not this function.
bool GCS_MAVLINK::do_try_send_message(const ap_message id)
{
    const bool in_delay_callback = hal.scheduler->in_delay_callback();
    if (in_delay_callback && !should_send_message_in_delay_callback(id)) {
        return true;
    }
    if (telemetry_delayed()) {
        return false;
    }
#if GCS_DEBUG_SEND_MESSAGE_TIMINGS
    void *data = hal.scheduler->disable_interrupts_save();
    uint32_t start_send_message_us = AP_HAL::micros();
#endif
    if (!try_send_message(id)) {
        // didn't fit in buffer...
#if GCS_DEBUG_SEND_MESSAGE_TIMINGS
        try_send_message_stats.no_space_for_message++;
        hal.scheduler->restore_interrupts(data);
#endif
        return false;
    }
#if GCS_DEBUG_SEND_MESSAGE_TIMINGS
    const uint32_t delta_us = AP_HAL::micros() - start_send_message_us;
    hal.scheduler->restore_interrupts(data);
    if (delta_us > try_send_message_stats.longest_time_us) {
        try_send_message_stats.longest_time_us = delta_us;
        try_send_message_stats.longest_id = id;
    }
#endif
    return true;
}

int8_t GCS_MAVLINK::get_deferred_message_index(const ap_message id) const
{
    for (uint8_t i=0; i<ARRAY_SIZE(deferred_message); i++) {
        if (deferred_message[i].id == id) {
            return i;
        }
    }
    return -1;
}

int8_t GCS_MAVLINK::deferred_message_to_send_index()
{
    const uint16_t now16_ms = AP_HAL::millis16();

    if (next_deferred_message_to_send_cache == -1) {
        uint16_t ms_before_next_message_to_send = UINT16_MAX;
        for (uint8_t i=0; i<ARRAY_SIZE(deferred_message); i++) {
            const uint16_t interval_ms = deferred_message[i].interval_ms;
            if (interval_ms == 0) {
                continue;
            }
            const uint16_t ms_since_last_sent = now16_ms - deferred_message[i].last_sent_ms;
            uint16_t ms_before_send_this_message;
            if (ms_since_last_sent > interval_ms) {
                // should already have sent this one!
                ms_before_send_this_message = 0;
            } else {
                ms_before_send_this_message = interval_ms - ms_since_last_sent;
            }
            if (ms_before_send_this_message < ms_before_next_message_to_send) {
                next_deferred_message_to_send_cache = i;
                ms_before_next_message_to_send = ms_before_send_this_message;
            }
        }
    }

    if (next_deferred_message_to_send_cache == -1) {
        // this really shouldn't happen; we force parameter rates, for example.
        return -1;
    }

    const uint16_t ms_since_last_sent = now16_ms - deferred_message[next_deferred_message_to_send_cache].last_sent_ms;
    if (ms_since_last_sent < deferred_message[next_deferred_message_to_send_cache].interval_ms) {
        return -1;
    }

    return next_deferred_message_to_send_cache;
}

void GCS_MAVLINK::update_send()
{
    if (!hal.scheduler->in_delay_callback()) {
        // DataFlash_Class will not send log data if we are armed.
        DataFlash_Class::instance()->handle_log_send();
    }

    if (!deferred_messages_initialised) {
        initialise_message_intervals_from_streamrates();
        deferred_messages_initialised = true;
    }

#if GCS_DEBUG_SEND_MESSAGE_TIMINGS
    uint32_t retry_deferred_body_start = 0;
#endif

    const uint32_t start = AP_HAL::millis();
    gcs().set_out_of_time(false);
    while (AP_HAL::millis() - start < 5) { // spend a max of 5ms sending messages.  This should never trigger - out_of_time() should become true
        if (gcs().out_of_time()) {
            break;
        }

#if GCS_DEBUG_SEND_MESSAGE_TIMINGS
        retry_deferred_body_start = AP_HAL::micros();
#endif

        // check if any "specially handled" messages should be sent out
        {
            const int8_t next = deferred_message_to_send_index();
            if (next != -1) {
                if (!do_try_send_message(deferred_message[next].id)) {
                    break;
                }
                deferred_message[next].last_sent_ms += deferred_message[next].interval_ms;
                next_deferred_message_to_send_cache = -1; // deferred_message_to_send will recalculate
#if GCS_DEBUG_SEND_MESSAGE_TIMINGS
                const uint32_t stop = AP_HAL::micros();
                const uint32_t delta = stop - retry_deferred_body_start;
                if (delta > try_send_message_stats.max_retry_deferred_body_us) {
                    try_send_message_stats.max_retry_deferred_body_us = delta;
                    try_send_message_stats.max_retry_deferred_body_type = 1;
                }
#endif
                continue;
            }
        }

        // check for any messages that the code has explicitly sent
        const int16_t fs = pushed_ap_message_ids.first_set();
        if (fs != -1) {
            ap_message next = (ap_message)fs;
            if (!do_try_send_message(next)) {
                break;
            }
            pushed_ap_message_ids.clear(next);
#if GCS_DEBUG_SEND_MESSAGE_TIMINGS
            const uint32_t stop = AP_HAL::micros();
            const uint32_t delta = stop - retry_deferred_body_start;
            if (delta > try_send_message_stats.max_retry_deferred_body_us) {
                try_send_message_stats.max_retry_deferred_body_us = delta;
                try_send_message_stats.max_retry_deferred_body_type = 2;
            }
#endif
            continue;
        }

        ap_message next = next_deferred_bucket_message_to_send();
        if (next != no_message_to_send) {
            if (!do_try_send_message(next)) {
                break;
            }
            bucket_message_ids_to_send.clear(next);
            if (bucket_message_ids_to_send.count() == 0) {
                // we sent everything in the bucket.  Reschedule it.
                deferred_message_bucket[sending_bucket_id].last_sent_ms +=
                    get_reschedule_interval_ms(deferred_message_bucket[sending_bucket_id]);
                find_next_bucket_to_send();
            }
#if GCS_DEBUG_SEND_MESSAGE_TIMINGS
                const uint32_t stop = AP_HAL::micros();
                const uint32_t delta = stop - retry_deferred_body_start;
                if (delta > try_send_message_stats.max_retry_deferred_body_us) {
                    try_send_message_stats.max_retry_deferred_body_us = delta;
                    try_send_message_stats.max_retry_deferred_body_type = 3;
                }
#endif
            continue;
        }
        break;
    }
#if GCS_DEBUG_SEND_MESSAGE_TIMINGS
    const uint32_t stop = AP_HAL::micros();
    const uint32_t delta = stop - retry_deferred_body_start;
    if (delta > try_send_message_stats.max_retry_deferred_body_us) {
        try_send_message_stats.max_retry_deferred_body_us = delta;
        try_send_message_stats.max_retry_deferred_body_type = 4;
    }
#endif
}

void GCS_MAVLINK::remove_message_from_bucket(int8_t bucket, ap_message id)
{
    deferred_message_bucket[bucket].ap_message_ids.clear(id);

    if (bucket == sending_bucket_id) {
        bucket_message_ids_to_send.clear(id);
    }

    if (deferred_message_bucket[bucket].ap_message_ids.count() == 0) {
        // bucket empty.  Free it:
        deferred_message_bucket[bucket].interval_ms = 0;
        deferred_message_bucket[bucket].last_sent_ms = 0;
        if (sending_bucket_id == bucket) {
            find_next_bucket_to_send();
        }
    }
}

bool GCS_MAVLINK::set_ap_message_interval(enum ap_message id, uint16_t interval_ms)
{
    if (id == MSG_NEXT_PARAM) {
        // force parameters to *always* get streamed so a vehicle is
        // recoverable from bad configuration:
        if (interval_ms == 0) {
            interval_ms = 100;
        } else if (interval_ms > 1000) {
            interval_ms = 1000;
        }
    }

    // send messages out at most 80% of main loop rate
    if (interval_ms != 0 &&
        interval_ms*800 < AP::scheduler().get_loop_period_us()) {
        interval_ms = AP::scheduler().get_loop_period_us()/800.0;
    }

    // check if it's a specially-handled message:
    const int8_t deferred_offset = get_deferred_message_index(id);
    if (deferred_offset != -1) {
        deferred_message[deferred_offset].interval_ms = interval_ms;
        deferred_message[deferred_offset].last_sent_ms = AP_HAL::millis16();
        return true;
    }

    // see which bucket has the closest interval:
    int8_t closest_bucket = -1;
    uint16_t closest_bucket_interval_delta = UINT16_MAX;
    int8_t in_bucket = -1;
    int8_t empty_bucket_id = -1;
    for (uint8_t i=0; i<ARRAY_SIZE(deferred_message_bucket); i++) {
        const deferred_message_bucket_t &bucket = deferred_message_bucket[i];
        if (bucket.interval_ms == 0) {
            // unused bucket
            if (empty_bucket_id == -1) {
                empty_bucket_id = i;
            }
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
            if (bucket.ap_message_ids.count() != 0) {
                AP_HAL::panic("Bucket %u has zero interval but with ids set", i);
            }
#endif
            continue;
        }
        if (bucket.ap_message_ids.get(id)) {
            in_bucket = i;
        }
        const uint16_t interval_delta = abs(bucket.interval_ms - interval_ms);
        if (interval_delta < closest_bucket_interval_delta) {
            closest_bucket = i;
            closest_bucket_interval_delta = interval_delta;
        }
    }

    if (in_bucket == -1 && interval_ms == 0) {
        // not in a bucket and told to remove from scheduling
        return true;
    }

    if (in_bucket != -1) {
        if (interval_ms == 0) {
            // remove it
            remove_message_from_bucket(in_bucket, id);
            return true;
        }
        if (closest_bucket_interval_delta == 0 &&
            in_bucket == closest_bucket) {
            // don't need to move it
            return true;
        }
        // remove from existing bucket
        remove_message_from_bucket(in_bucket, id);
        if (empty_bucket_id == -1 &&
            deferred_message_bucket[in_bucket].ap_message_ids.count() == 0) {
            empty_bucket_id = in_bucket;
        }
    }

    if (closest_bucket == -1 && empty_bucket_id == -1) {
        // gah?!
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        ::fprintf(stderr, "no buckets?!\n");
        abort();
#endif
        return false;
    }

    if (closest_bucket_interval_delta != 0 &&
        empty_bucket_id != -1) {
        // allocate a bucket for this interval
        deferred_message_bucket[empty_bucket_id].interval_ms = interval_ms;
        deferred_message_bucket[empty_bucket_id].last_sent_ms = AP_HAL::millis16();
        closest_bucket = empty_bucket_id;
        closest_bucket_interval_delta = 0;
    }

    deferred_message_bucket[closest_bucket].ap_message_ids.set(id);

    if (sending_bucket_id == no_bucket_to_send) {
        sending_bucket_id = closest_bucket;
        bucket_message_ids_to_send = deferred_message_bucket[closest_bucket].ap_message_ids;
    }

    return true;
}

// queue a message to be sent (try_send_message does the *actual*
// mavlink work!)
void GCS_MAVLINK::send_message(enum ap_message id)
{
    if (id == MSG_HEARTBEAT) {
        save_signing_timestamp(false);
    }

    pushed_ap_message_ids.set(id);
}

void GCS_MAVLINK::packetReceived(const mavlink_status_t &status,
                                 mavlink_message_t &msg)
{
    // we exclude radio packets because we historically used this to
    // make it possible to use the CLI over the radio
    if (msg.msgid != MAVLINK_MSG_ID_RADIO && msg.msgid != MAVLINK_MSG_ID_RADIO_STATUS) {
        const uint8_t mask = (1U<<(chan-MAVLINK_COMM_0));
        if (!(mask & mavlink_private)) {
            mavlink_active |= mask;
        }
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
    if (routing.check_and_forward(chan, &msg) &&
        accept_packet(status, msg)) {
        handleMessage(&msg);
    }
}

void
GCS_MAVLINK::update_receive(uint32_t max_time_us)
{
    // receive new packets
    mavlink_message_t msg;
    mavlink_status_t status;
    uint32_t tstart_us = AP_HAL::micros();
    uint32_t now_ms = AP_HAL::millis();

    hal.util->perf_begin(_perf_update);

    status.packet_rx_drop_count = 0;

    // process received bytes
    uint16_t nbytes = comm_get_available(chan);
    for (uint16_t i=0; i<nbytes; i++)
    {
        const uint8_t c = (uint8_t)_port->read();
        const uint32_t protocol_timeout = 4000;
        
        if (alternative.handler &&
            now_ms - alternative.last_mavlink_ms > protocol_timeout) {
            /*
              we have an alternative protocol handler installed and we
              haven't parsed a MAVLink packet for 4 seconds. Try
              parsing using alternative handler
             */
            if (alternative.handler(c, mavlink_comm_port[chan])) {
                alternative.last_alternate_ms = now_ms;
                gcs_alternative_active[chan] = true;
            }
            
            /*
              we may also try parsing as MAVLink if we haven't had a
              successful parse on the alternative protocol for 4s
             */
            if (now_ms - alternative.last_alternate_ms <= protocol_timeout) {
                continue;
            }
        }

        bool parsed_packet = false;

        // Try to get a new message
        if (mavlink_parse_char(chan, c, &msg, &status)) {
            hal.util->perf_begin(_perf_packet);
            packetReceived(status, msg);
            hal.util->perf_end(_perf_packet);
            parsed_packet = true;
            gcs_alternative_active[chan] = false;
            alternative.last_mavlink_ms = now_ms;
        }

        if (parsed_packet || i % 100 == 0) {
            // make sure we don't spend too much time parsing mavlink messages
            if (AP_HAL::micros() - tstart_us > max_time_us) {
                break;
            }
        }
    }

    const uint32_t tnow = AP_HAL::millis();

    // send a timesync message every 10 seconds; this is for data
    // collection purposes
    if (tnow - _timesync_request.last_sent_ms > _timesync_request.interval_ms && !is_private()) {
        if (HAVE_PAYLOAD_SPACE(chan, TIMESYNC)) {
            send_timesync();
            _timesync_request.last_sent_ms = tnow;
        }
    }

#if GCS_DEBUG_SEND_MESSAGE_TIMINGS

    const uint16_t now16_ms{AP_HAL::millis16()};

    if (uint16_t(now16_ms - try_send_message_stats.statustext_last_sent_ms) > 10000U) {
        if (try_send_message_stats.longest_time_us) {
            gcs().send_text(MAV_SEVERITY_INFO,
                            "GCS.chan(%u): ap_msg=%u took %uus to send",
                            chan,
                            try_send_message_stats.longest_id,
                            try_send_message_stats.longest_time_us);
            try_send_message_stats.longest_time_us = 0;
        }
        if (try_send_message_stats.no_space_for_message &&
            (is_active() || is_streaming())) {
            gcs().send_text(MAV_SEVERITY_INFO,
                            "GCS.chan(%u): out-of-space: %u",
                            chan,
                            try_send_message_stats.no_space_for_message);
            try_send_message_stats.no_space_for_message = 0;
        }
        if (max_slowdown) {
            gcs().send_text(MAV_SEVERITY_INFO,
                            "GCS.chan(%u): max slowdown=%u",
                            chan,
                            max_slowdown);
            max_slowdown = 0;
        }
        if (try_send_message_stats.behind) {
            gcs().send_text(MAV_SEVERITY_INFO,
                            "GCS.chan(%u): behind=%u",
                            chan,
                            try_send_message_stats.behind);
            try_send_message_stats.behind = 0;
        }
        if (try_send_message_stats.fnbts_maxtime) {
            gcs().send_text(MAV_SEVERITY_INFO,
                            "GCS.chan(%u): fnbts_maxtime=%uus",
                            chan,
                            try_send_message_stats.fnbts_maxtime);
            try_send_message_stats.fnbts_maxtime = 0;
        }
        if (try_send_message_stats.max_retry_deferred_body_us) {
            gcs().send_text(MAV_SEVERITY_INFO,
                            "GCS.chan(%u): retry_body_maxtime=%uus (%u)",
                            chan,
                            try_send_message_stats.max_retry_deferred_body_us,
                            try_send_message_stats.max_retry_deferred_body_type
                );
            try_send_message_stats.max_retry_deferred_body_us = 0;
        }

        for (uint8_t i=0; i<ARRAY_SIZE(deferred_message_bucket); i++) {
            gcs().send_text(MAV_SEVERITY_INFO,
                            "B. intvl. (%u): %u %u %u %u %u",
                            chan,
                            deferred_message_bucket[0].interval_ms,
                            deferred_message_bucket[1].interval_ms,
                            deferred_message_bucket[2].interval_ms,
                            deferred_message_bucket[3].interval_ms,
                            deferred_message_bucket[4].interval_ms);
        }

        try_send_message_stats.statustext_last_sent_ms = now16_ms;
    }
#endif

    if (waypoint_receiving) {
        const uint32_t wp_recv_time = 1000U + (stream_slowdown*20);

        // stop waypoint receiving if timeout
        if (tnow - waypoint_timelast_receive > wp_recv_time+waypoint_receive_timeout) {
            waypoint_receiving = false;
            gcs().send_text(MAV_SEVERITY_WARNING, "Mission upload timeout");
        } else if (tnow - waypoint_timelast_request > wp_recv_time) {
            waypoint_timelast_request = tnow;
            send_message(MSG_NEXT_MISSION_REQUEST);
        }
    }

    hal.util->perf_end(_perf_update);    
}


/*
  send the SYSTEM_TIME message
 */
void GCS_MAVLINK::send_system_time()
{
    uint64_t time_unix = 0;
    AP::rtc().get_utc_usec(time_unix); // may fail, leaving time_unix at 0

    mavlink_msg_system_time_send(
        chan,
        time_unix,
        AP_HAL::millis());
}


/*
  send RC_CHANNELS messages
 */
void GCS_MAVLINK::send_radio_in()
{
    AP_RSSI *rssi = AP::rssi();
    uint8_t receiver_rssi = 0;
    if (rssi != nullptr) {
        receiver_rssi = rssi->read_receiver_rssi_uint8();
    }

    uint32_t now = AP_HAL::millis();
    mavlink_status_t *status = mavlink_get_channel_status(chan);

    uint16_t values[18] = {};
    rc().get_radio_in(values, ARRAY_SIZE(values));

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
    }
    if (!HAVE_PAYLOAD_SPACE(chan, RC_CHANNELS)) {
        // can't fit RC_CHANNELS
        return;
    }
    mavlink_msg_rc_channels_send(
        chan,
        now,
        RC_Channels::get_valid_channel_count(),
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

void GCS_MAVLINK::send_raw_imu()
{
    const AP_InertialSensor &ins = AP::ins();
    const Compass &compass = AP::compass();

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
}

void GCS_MAVLINK::send_scaled_imu(uint8_t instance, void (*send_fn)(mavlink_channel_t chan, uint32_t time_ms, int16_t xacc, int16_t yacc, int16_t zacc, int16_t xgyro, int16_t ygyro, int16_t zgyro, int16_t xmag, int16_t ymag, int16_t zmag))
{
    const AP_InertialSensor &ins = AP::ins();
    const Compass &compass = AP::compass();

    bool have_data = false;
    Vector3f accel{};
    if (ins.get_accel_count() > instance) {
        accel = ins.get_accel(instance);
        have_data = true;
    }
    Vector3f gyro{};
    if (ins.get_accel_count() > instance) {
        gyro = ins.get_gyro(instance);
        have_data = true;
    }
    Vector3f mag{};
    if (compass.get_count() > instance) {
        mag = compass.get_field(instance);
        have_data = true;
    }
    if (!have_data) {
        return;
    }
    send_fn(
        chan,
        AP_HAL::millis(),
        accel.x * 1000.0f / GRAVITY_MSS,
        accel.y * 1000.0f / GRAVITY_MSS,
        accel.z * 1000.0f / GRAVITY_MSS,
        gyro.x * 1000.0f,
        gyro.y * 1000.0f,
        gyro.z * 1000.0f,
        mag.x,
        mag.y,
        mag.z);
}


// send data for barometer and airspeed sensors instances.  In the
// case that we run out of instances of one before the other we send
// the relevant fields as 0.
void GCS_MAVLINK::send_scaled_pressure_instance(uint8_t instance, void (*send_fn)(mavlink_channel_t chan, uint32_t time_boot_ms, float press_abs, float press_diff, int16_t temperature))
{
    const AP_Baro &barometer = AP::baro();

    bool have_data = false;

    float press_abs = 0.0f;
    float temperature = 0.0f;
    if (instance < barometer.num_instances()) {
        press_abs = barometer.get_pressure(instance) * 0.01f;
        temperature = barometer.get_temperature(instance)*100;
        have_data = true;
    }

    float press_diff = 0; // pascal
    AP_Airspeed *airspeed = AP_Airspeed::get_singleton();
    if (airspeed != nullptr &&
        instance < AIRSPEED_MAX_SENSORS) {
        press_diff = airspeed->get_differential_pressure(instance) * 0.01f;
        have_data = true;
    }

    if (!have_data) {
        return;
    }

    send_fn(
        chan,
        AP_HAL::millis(),
        press_abs, // hectopascal
        press_diff, // hectopascal
        temperature); // 0.01 degrees C
}

void GCS_MAVLINK::send_scaled_pressure()
{
    send_scaled_pressure_instance(0, mavlink_msg_scaled_pressure_send);
}

void GCS_MAVLINK::send_scaled_pressure2()
{
    send_scaled_pressure_instance(1, mavlink_msg_scaled_pressure2_send);
}

void GCS_MAVLINK::send_scaled_pressure3()
{
    send_scaled_pressure_instance(2, mavlink_msg_scaled_pressure3_send);
}

void GCS_MAVLINK::send_sensor_offsets()
{
    const AP_InertialSensor &ins = AP::ins();
    const Compass &compass = AP::compass();

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

    const AP_Baro &barometer = AP::baro();

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

void GCS_MAVLINK::send_ahrs()
{
    const AP_AHRS &ahrs = AP::ahrs();
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
    send a statustext text string to specific MAVLink bitmask
*/
void GCS::send_statustext(MAV_SEVERITY severity, uint8_t dest_bitmask, const char *text)
{
    DataFlash_Class *dataflash = DataFlash_Class::instance();
    if (dataflash != nullptr) {
        dataflash->Log_Write_Message(text);
    }

    // add statustext message to FrSky lib queue
    if (frsky_telemetry_p != NULL) {
        frsky_telemetry_p->queue_message(severity, text);
    }

    AP_Notify *notify = AP_Notify::instance();
    if (notify) {
        notify->send_text(text);
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

    WITH_SEMAPHORE(_statustext_sem);
    
    // The force push will ensure comm links do not block other comm links forever if they fail.
    // If we push to a full buffer then we overwrite the oldest entry, effectively removing the
    // block but not until the buffer fills up.
    _statustext_queue.push_force(statustext);

    // try and send immediately if possible
    service_statustext();
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

void GCS::send_message(enum ap_message id)
{
    for (uint8_t i=0; i<num_gcs(); i++) {
        if (chan(i).initialised) {
            chan(i).send_message(id);
        }
    }
}

void GCS::update_send()
{
    for (uint8_t i=0; i<num_gcs(); i++) {
        if (chan(i).initialised) {
            chan(i).update_send();
        }
    }
    WITH_SEMAPHORE(_statustext_sem);
    service_statustext();
}

void GCS::update_receive(void)
{
    for (uint8_t i=0; i<num_gcs(); i++) {
        if (chan(i).initialised) {
            chan(i).update_receive();
        }
    }
    // also update UART pass-thru, if enabled
    update_passthru();
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

// report battery2 state
void GCS_MAVLINK::send_battery2()
{
    const AP_BattMonitor &battery = AP::battery();

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
void GCS_MAVLINK::handle_set_mode(mavlink_message_t* msg)
{
    mavlink_set_mode_t packet;
    mavlink_msg_set_mode_decode(msg, &packet);

    const MAV_MODE _base_mode = (MAV_MODE)packet.base_mode;
    const uint32_t _custom_mode = packet.custom_mode;

    const MAV_RESULT result = _set_mode_common(_base_mode, _custom_mode);

    // send ACK or NAK
    mavlink_msg_command_ack_send_buf(msg, chan, MAVLINK_MSG_ID_SET_MODE, result);
}

/*
  code common to both SET_MODE mavlink message and command long set_mode msg
*/
MAV_RESULT GCS_MAVLINK::_set_mode_common(const MAV_MODE _base_mode, const uint32_t _custom_mode)
{
    MAV_RESULT result = MAV_RESULT_UNSUPPORTED;
    // only accept custom modes because there is no easy mapping from Mavlink flight modes to AC flight modes
    if (_base_mode & MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) {
        if (set_mode(_custom_mode)) {
            result = MAV_RESULT_ACCEPTED;
        }
    } else if (_base_mode == (MAV_MODE)MAV_MODE_FLAG_DECODE_POSITION_SAFETY) {
        // set the safety switch position. Must be in a command by itself
        if (_custom_mode == 0) {
            // turn safety off (pwm outputs flow to the motors)
            hal.rcout->force_safety_off();
            result = MAV_RESULT_ACCEPTED;
        } else if (_custom_mode == 1) {
            // turn safety on (no pwm outputs to the motors)
            if (hal.rcout->force_safety_on()) {
                result = MAV_RESULT_ACCEPTED;
            }
        }
    }

    return result;
}

#if AP_AHRS_NAVEKF_AVAILABLE
/*
  send OPTICAL_FLOW message
 */
void GCS_MAVLINK::send_opticalflow()
{
    const OpticalFlow *optflow = AP::opticalflow();

    // exit immediately if no optical flow sensor or not healthy
    if (optflow == nullptr ||
        !optflow->healthy()) {
        return;
    }

    // get rates from sensor
    const Vector2f &flowRate = optflow->flowRate();
    const Vector2f &bodyRate = optflow->bodyRate();

    const AP_AHRS &ahrs = AP::ahrs();
    float hagl = 0;
    if (ahrs.have_inertial_nav()) {
        if (!ahrs.get_hagl(hagl)) {
            return;
        }
    }

    // populate and send message
    mavlink_msg_optical_flow_send(
        chan,
        AP_HAL::millis(),
        0, // sensor id is zero
        flowRate.x,
        flowRate.y,
        flowRate.x - bodyRate.x,
        flowRate.y - bodyRate.y,
        optflow->quality(),
        hagl,  // ground distance (in meters) set to zero
        flowRate.x,
        flowRate.y);
}
#endif

/*
  send AUTOPILOT_VERSION packet
 */
void GCS_MAVLINK::send_autopilot_version() const
{
    uint32_t flight_sw_version;
    uint32_t middleware_sw_version = 0;
    uint32_t os_sw_version = 0;
    uint32_t board_version = 0;
    char flight_custom_version[MAVLINK_MSG_AUTOPILOT_VERSION_FIELD_FLIGHT_CUSTOM_VERSION_LEN]{};
    char middleware_custom_version[MAVLINK_MSG_AUTOPILOT_VERSION_FIELD_MIDDLEWARE_CUSTOM_VERSION_LEN]{};
    char os_custom_version[MAVLINK_MSG_AUTOPILOT_VERSION_FIELD_OS_CUSTOM_VERSION_LEN]{};
    uint16_t vendor_id = 0;
    uint16_t product_id = 0;
    uint64_t uid = 0;
    uint8_t  uid2[MAVLINK_MSG_AUTOPILOT_VERSION_FIELD_UID2_LEN] = {0};
    const AP_FWVersion &version = AP::fwversion();

    flight_sw_version = version.major << (8 * 3) | \
                        version.minor << (8 * 2) | \
                        version.patch << (8 * 1) | \
                        (uint32_t)(version.fw_type) << (8 * 0);

    if (version.fw_hash_str) {
        strncpy(flight_custom_version, version.fw_hash_str, sizeof(flight_custom_version) - 1);
        flight_custom_version[sizeof(flight_custom_version) - 1] = '\0';
    }

    if (version.middleware_hash_str) {
        strncpy(middleware_custom_version, version.middleware_hash_str, sizeof(middleware_custom_version) - 1);
        middleware_custom_version[sizeof(middleware_custom_version) - 1] = '\0';
    }

    if (version.os_hash_str) {
        strncpy(os_custom_version, version.os_hash_str, sizeof(os_custom_version) - 1);
        os_custom_version[sizeof(os_custom_version) - 1] = '\0';
    }

    mavlink_msg_autopilot_version_send(
        chan,
        hal.util->get_capabilities(),
        flight_sw_version,
        middleware_sw_version,
        os_sw_version,
        board_version,
        (uint8_t *)flight_custom_version,
        (uint8_t *)middleware_custom_version,
        (uint8_t *)os_custom_version,
        vendor_id,
        product_id,
        uid,
        uid2
    );
}


/*
  send LOCAL_POSITION_NED message
 */
void GCS_MAVLINK::send_local_position() const
{
    const AP_AHRS &ahrs = AP::ahrs();

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
void GCS_MAVLINK::send_vibration() const
{
    const AP_InertialSensor &ins = AP::ins();

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

void GCS_MAVLINK::send_named_float(const char *name, float value) const
{
    char float_name[MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN+1] {};
    strncpy(float_name, name, MAVLINK_MSG_NAMED_VALUE_FLOAT_FIELD_NAME_LEN);
    mavlink_msg_named_value_float_send(chan, AP_HAL::millis(), float_name, value);
}

void GCS_MAVLINK::send_home() const
{
    if (!HAVE_PAYLOAD_SPACE(chan, HOME_POSITION)) {
        return;
    }
    if (!AP::ahrs().home_is_set()) {
        return;
    }

    Location home = AP::ahrs().get_home();

    const float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};
    mavlink_msg_home_position_send(
        chan,
        home.lat,
        home.lng,
        home.alt * 10,
        0.0f, 0.0f, 0.0f,
        q,
        0.0f, 0.0f, 0.0f,
        AP_HAL::micros64());
}

void GCS_MAVLINK::send_ekf_origin() const
{
    if (!HAVE_PAYLOAD_SPACE(chan, GPS_GLOBAL_ORIGIN)) {
        return;
    }
    Location ekf_origin;
    if (!AP::ahrs().get_origin(ekf_origin)) {
        return;
    }
    mavlink_msg_gps_global_origin_send(
        chan,
        ekf_origin.lat,
        ekf_origin.lng,
        ekf_origin.alt * 10,
        AP_HAL::micros64());
}

/*
  Send MAVLink heartbeat
 */
void GCS_MAVLINK::send_heartbeat() const
{
    mavlink_msg_heartbeat_send(
        chan,
        frame_type(),
        MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode(),
        custom_mode(),
        system_status());
}

MAV_RESULT GCS_MAVLINK::handle_command_set_message_interval(const mavlink_command_long_t &packet)
{
    const uint32_t msg_id = (uint32_t)packet.param1;
    const int32_t interval_us = (int32_t)packet.param2;

    uint16_t interval_ms;
    if (interval_us == 0) {
        // zero is "reset to default rate"
        if (!get_default_interval_for_mavlink_message_id(msg_id, interval_ms)) {
            return MAV_RESULT_FAILED;
        }
    } else if (interval_us == -1) {
        // minus-one is "stop sending"
        interval_ms = 0;
    } else if (interval_us < 1000) {
        // don't squash sub-ms times to zero
        interval_ms = 1;
    } else if (interval_us > 60000000) {
        interval_ms = 60000;
    } else {
        interval_ms = interval_us / 1000;
    }
    if (set_mavlink_message_id_interval(msg_id, interval_ms)) {
        return MAV_RESULT_ACCEPTED;
    }

    return MAV_RESULT_FAILED;
}

// are we still delaying telemetry to try to avoid Xbee bricking?
bool GCS_MAVLINK::telemetry_delayed() const
{
    uint32_t tnow = AP_HAL::millis() >> 10;
    if (tnow > telem_delay()) {
        return false;
    }
    if (chan == MAVLINK_COMM_0 && hal.gpio->usb_connected()) {
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
void GCS_MAVLINK::send_servo_output_raw()
{
    uint16_t values[16] {};
    if (in_hil_mode()) {
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


float GCS_MAVLINK::vfr_hud_airspeed() const
{
    AP_Airspeed *airspeed = AP_Airspeed::get_singleton();
    if (airspeed != nullptr && airspeed->healthy()) {
        return airspeed->get_airspeed();
    }
    // because most vehicles don't have airspeed sensors, we return a
    // different sort of speed estimate in the relevant field for
    // comparison's sake.
    return AP::gps().ground_speed();
}

float GCS_MAVLINK::vfr_hud_climbrate() const
{
    return -vfr_hud_velned.z;
}

float GCS_MAVLINK::vfr_hud_alt() const
{
    return global_position_current_loc.alt * 0.01f; // cm -> m
}

void GCS_MAVLINK::send_vfr_hud()
{
    AP_AHRS &ahrs = AP::ahrs();

    // return values ignored; we send stale data
    ahrs.get_position(global_position_current_loc);
    ahrs.get_velocity_NED(vfr_hud_velned);

    mavlink_msg_vfr_hud_send(
        chan,
        vfr_hud_airspeed(),
        ahrs.groundspeed(),
        (ahrs.yaw_sensor / 100) % 360,
        vfr_hud_throttle(),
        vfr_hud_alt(),
        vfr_hud_climbrate());
}

void GCS_MAVLINK::zero_rc_outputs()
{
    // Send an invalid signal to the motors to prevent spinning due to neutral (1500) pwm pulse being cut short
    // For that matter, send an invalid signal to all channels to prevent undesired/unexpected behavior
    SRV_Channels::cork();
    for (int i=0; i<NUM_RC_CHANNELS; i++) {
        hal.rcout->write(i, 0);
    }
    SRV_Channels::push();
}

void GCS_MAVLINK::disable_overrides()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 || CONFIG_HAL_BOARD == HAL_BOARD_VRBRAIN
    int px4io_fd = open("/dev/px4io", 0);
    if (px4io_fd < 0) {
        return;
    }
    // disable OVERRIDES so we don't run the mixer while
    // rebooting
    if (ioctl(px4io_fd, PWM_SERVO_SET_OVERRIDE_OK, 0) != 0) {
        hal.console->printf("SET_OVERRIDE_OK failed\n");
    }
    if (ioctl(px4io_fd, PWM_SERVO_SET_OVERRIDE_IMMEDIATE, 0) != 0) {
        hal.console->printf("SET_OVERRIDE_IMMEDIATE failed\n");
    }
    close(px4io_fd);
#endif
}

/*
  handle a MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN command 

  Optionally disable PX4IO overrides. This is done for quadplanes to
  prevent the mixer running while rebooting which can start the VTOL
  motors. That can be dangerous when a preflight reboot is done with
  the pilot close to the aircraft and can also damage the aircraft
 */
MAV_RESULT GCS_MAVLINK::handle_preflight_reboot(const mavlink_command_long_t &packet)
{
    if (!(is_equal(packet.param1, 1.0f) || is_equal(packet.param1, 3.0f))) {
        // param1 must be 1 or 3 - 1 being reboot, 3 being reboot-to-bootloader
        return MAV_RESULT_UNSUPPORTED;
    }

    if (should_disable_overrides_on_reboot()) {
        // disable overrides while rebooting
        disable_overrides();
    }
    if (should_zero_rc_outputs_on_reboot()) {
        zero_rc_outputs();
    }

    // send ack before we reboot
    mavlink_msg_command_ack_send(chan, packet.command, MAV_RESULT_ACCEPTED);
    // Notify might want to blink some LEDs:
    AP_Notify *notify = AP_Notify::instance();
    if (notify) {
        AP_Notify::flags.firmware_update = 1;
        notify->update();
    }
    // force safety on
    hal.rcout->force_safety_on();
    hal.rcout->force_safety_no_wait();

    // flush pending parameter writes
    AP_Param::flush();

    hal.scheduler->delay(200);
    
    // when packet.param1 == 3 we reboot to hold in bootloader
    const bool hold_in_bootloader = is_equal(packet.param1, 3.0f);
    hal.scheduler->reboot(hold_in_bootloader);

    return MAV_RESULT_FAILED;
}

/*
  handle a flight termination request
 */
MAV_RESULT GCS_MAVLINK::handle_flight_termination(const mavlink_command_long_t &packet)
{
    AP_AdvancedFailsafe *failsafe = get_advanced_failsafe();
    if (failsafe == nullptr) {
        return MAV_RESULT_UNSUPPORTED;
    }

    bool should_terminate = packet.param1 > 0.5f;

    if (failsafe->gcs_terminate(should_terminate, "GCS request")) {
        return MAV_RESULT_ACCEPTED;
    }
    return MAV_RESULT_FAILED;
}

/*
  handle a R/C bind request (for spektrum)
 */
MAV_RESULT GCS_MAVLINK::handle_rc_bind(const mavlink_command_long_t &packet)
{
    // initiate bind procedure. We accept the DSM type from either
    // param1 or param2 due to a past mixup with what parameter is the
    // right one
    if (!RC_Channels::receiver_bind(packet.param2>0?packet.param2:packet.param1)) {
        return MAV_RESULT_FAILED;
    }
    return MAV_RESULT_ACCEPTED;
}

uint64_t GCS_MAVLINK::timesync_receive_timestamp_ns() const
{
    uint64_t ret = _port->receive_time_constraint_us(PAYLOAD_SIZE(chan, TIMESYNC));
    if (ret == 0) {
        ret = AP_HAL::micros64();
    }
    return ret*1000LL;
}

uint64_t GCS_MAVLINK::timesync_timestamp_ns() const
{
    // we add in our own system id try to ensure we only consider
    // responses to our own timesync request messages
    return AP_HAL::micros64()*1000LL + mavlink_system.sysid;
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

    if (tsync.tc1 != 0) {
        // this is a response to a timesync request
        if (tsync.ts1 != _timesync_request.sent_ts1) {
            // we didn't actually send the request.... or it's a
            // response to an ancient request...
            return;
        }
        const uint64_t round_trip_time_us = (timesync_receive_timestamp_ns() - _timesync_request.sent_ts1)*0.001f;
#if 0
        gcs().send_text(MAV_SEVERITY_INFO,
                        "timesync response sysid=%u (latency=%fms)",
                        msg->sysid,
                        round_trip_time_us*0.001f);
#endif
        DataFlash_Class *df = DataFlash_Class::instance();
        if (df != nullptr) {
            DataFlash_Class::instance()->Log_Write(
                "TSYN",
                "TimeUS,SysID,RTT",
                "s-s",
                "F-F",
                "QBQ",
                AP_HAL::micros64(),
                msg->sysid,
                round_trip_time_us
                );
        }
        return;
    }

    if (!HAVE_PAYLOAD_SPACE(chan, TIMESYNC)) {
        // drop this timesync request entirely
        return;
    }

    // create new timesync struct with tc1 field as system time in
    // nanoseconds.  The client timestamp is as close as possible to
    // the time we received the TIMESYNC message.
    mavlink_timesync_t rsync;
    rsync.tc1 = timesync_receive_timestamp_ns();
    rsync.ts1 = tsync.ts1;

    // respond with a timesync message
    mavlink_msg_timesync_send(
        chan,
        rsync.tc1,
        rsync.ts1
        );
}

/*
 * broadcast a timesync message.  We may get multiple responses to this request.
 */
void GCS_MAVLINK::send_timesync()
{
    _timesync_request.sent_ts1 = timesync_timestamp_ns();
    mavlink_msg_timesync_send(
        chan,
        0,
        _timesync_request.sent_ts1
        );
}

void GCS_MAVLINK::handle_statustext(mavlink_message_t *msg)
{
    DataFlash_Class *df = DataFlash_Class::instance();
    if (df == nullptr) {
        return;
    }

    mavlink_statustext_t packet;
    mavlink_msg_statustext_decode(msg, &packet);
    const uint8_t max_prefix_len = 20;
    const uint8_t text_len = MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1+max_prefix_len;
    char text[text_len] = { 'G','C','S',':'};
    uint8_t offset = strlen(text);

    if (msg->sysid != sysid_my_gcs()) {
        offset = hal.util->snprintf(text,
                                    max_prefix_len,
                                    "SRC=%u/%u:",
                                    msg->sysid,
                                    msg->compid);
    }

    memcpy(&text[offset], packet.text, MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN);

    df->Log_Write_Message(text);
}


void GCS_MAVLINK::handle_system_time_message(const mavlink_message_t *msg)
{
    mavlink_system_time_t packet;
    mavlink_msg_system_time_decode(msg, &packet);

    AP::rtc().set_utc_usec(packet.time_unix_usec, AP_RTC::SOURCE_MAVLINK_SYSTEM_TIME);
}

MAV_RESULT GCS_MAVLINK::handle_command_camera(const mavlink_command_long_t &packet)
{
    AP_Camera *camera = AP::camera();
    if (camera == nullptr) {
        return MAV_RESULT_UNSUPPORTED;
    }

    MAV_RESULT result = MAV_RESULT_FAILED;
    switch (packet.command) {
    case MAV_CMD_DO_DIGICAM_CONFIGURE:
        camera->configure(packet.param1,
                          packet.param2,
                          packet.param3,
                          packet.param4,
                          packet.param5,
                          packet.param6,
                          packet.param7);
        result = MAV_RESULT_ACCEPTED;
        break;
    case MAV_CMD_DO_DIGICAM_CONTROL:
        camera->control(packet.param1,
                        packet.param2,
                        packet.param3,
                        packet.param4,
                        packet.param5,
                        packet.param6);
        result = MAV_RESULT_ACCEPTED;
        break;
    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
        camera->set_trigger_distance(packet.param1);
        result = MAV_RESULT_ACCEPTED;
        break;
    default:
        result = MAV_RESULT_UNSUPPORTED;
        break;
    }
    return result;
}


// sets ekf_origin if it has not been set.
//  should only be used when there is no GPS to provide an absolute position
void GCS_MAVLINK::set_ekf_origin(const Location& loc)
{
    // check location is valid
    if (!check_latlng(loc)) {
        return;
    }

    AP_AHRS &ahrs = AP::ahrs();

    // check if EKF origin has already been set
    Location ekf_origin;
    if (ahrs.get_origin(ekf_origin)) {
        return;
    }

    if (!ahrs.set_origin(loc)) {
        return;
    }

    // log ahrs home and ekf origin dataflash
    ahrs.Log_Write_Home_And_Origin();

    // send ekf origin to GCS
    send_ekf_origin();
}

void GCS_MAVLINK::handle_set_gps_global_origin(const mavlink_message_t *msg)
{
    mavlink_set_gps_global_origin_t packet;
    mavlink_msg_set_gps_global_origin_decode(msg, &packet);

    // sanity check location
    if (!check_latlng(packet.latitude, packet.longitude)) {
        // silently drop the request
        return;
    }

    Location ekf_origin {};
    ekf_origin.lat = packet.latitude;
    ekf_origin.lng = packet.longitude;
    ekf_origin.alt = packet.altitude / 10;
    set_ekf_origin(ekf_origin);
}

/*
  handle a DATA96 message
 */
void GCS_MAVLINK::handle_data_packet(mavlink_message_t *msg)
{
#if HAL_RCINPUT_WITH_AP_RADIO
    mavlink_data96_t m;
    mavlink_msg_data96_decode(msg, &m);
    switch (m.type) {
    case 42:
    case 43: {
        // pass to AP_Radio (for firmware upload and playing test tunes)
        AP_Radio *radio = AP_Radio::instance();
        if (radio != nullptr) {
            radio->handle_data_packet(chan, m);
        }
        break;
    }
    default:
        // unknown
        break;
    }
#endif
}

void GCS_MAVLINK::handle_vision_position_delta(mavlink_message_t *msg)
{
    AP_VisualOdom *visual_odom = get_visual_odom();
    if (visual_odom == nullptr) {
        return;
    }
    visual_odom->handle_msg(msg);
}

void GCS_MAVLINK::handle_vision_position_estimate(mavlink_message_t *msg)
{
    mavlink_vision_position_estimate_t m;
    mavlink_msg_vision_position_estimate_decode(msg, &m);

    handle_common_vision_position_estimate_data(m.usec, m.x, m.y, m.z, m.roll, m.pitch, m.yaw,
                                                PAYLOAD_SIZE(chan, VISION_POSITION_ESTIMATE));
}

void GCS_MAVLINK::handle_global_vision_position_estimate(mavlink_message_t *msg)
{
    mavlink_global_vision_position_estimate_t m;
    mavlink_msg_global_vision_position_estimate_decode(msg, &m);

    handle_common_vision_position_estimate_data(m.usec, m.x, m.y, m.z, m.roll, m.pitch, m.yaw,
                                                PAYLOAD_SIZE(chan, GLOBAL_VISION_POSITION_ESTIMATE));
}

void GCS_MAVLINK::handle_vicon_position_estimate(mavlink_message_t *msg)
{
    mavlink_vicon_position_estimate_t m;
    mavlink_msg_vicon_position_estimate_decode(msg, &m);

    handle_common_vision_position_estimate_data(m.usec, m.x, m.y, m.z, m.roll, m.pitch, m.yaw,
                                                PAYLOAD_SIZE(chan, VICON_POSITION_ESTIMATE));
}

// there are several messages which all have identical fields in them.
// This function provides common handling for the data contained in
// these packets
void GCS_MAVLINK::handle_common_vision_position_estimate_data(const uint64_t usec,
                                                              const float x,
                                                              const float y,
                                                              const float z,
                                                              const float roll,
                                                              const float pitch,
                                                              const float yaw,
                                                              const uint16_t payload_size)
{
    // correct offboard timestamp to be in local ms since boot
    uint32_t timestamp_ms = correct_offboard_timestamp_usec_to_ms(usec, payload_size);
    
    // sensor assumed to be at 0,0,0 body-frame; need parameters for this?
    // or a new message 
    const Vector3f sensor_offset = {};
    const Vector3f pos = {
        x,
        y,
        z
    };
    Quaternion attitude;
    attitude.from_euler(roll, pitch, yaw); // from_vector312?
    const float posErr = 0; // parameter required?
    const float angErr = 0; // parameter required?
    const uint32_t reset_timestamp_ms = 0; // no data available

    AP::ahrs().writeExtNavData(sensor_offset,
                               pos,
                               attitude,
                               posErr,
                               angErr,
                               timestamp_ms,
                               reset_timestamp_ms);

    log_vision_position_estimate_data(usec, x, y, z, roll, pitch, yaw);
}

void GCS_MAVLINK::log_vision_position_estimate_data(const uint64_t usec,
                                                    const float x,
                                                    const float y,
                                                    const float z,
                                                    const float roll,
                                                    const float pitch,
                                                    const float yaw)
{
    DataFlash_Class::instance()->Log_Write("VISP", "TimeUS,RemTimeUS,PX,PY,PZ,Roll,Pitch,Yaw",
                                           "ssmmmddh", "FF000000", "QQffffff",
                                           (uint64_t)AP_HAL::micros64(),
                                           (uint64_t)usec,
                                           (double)x,
                                           (double)y,
                                           (double)z,
                                           (double)(roll * RAD_TO_DEG),
                                           (double)(pitch * RAD_TO_DEG),
                                           (double)(yaw * RAD_TO_DEG));
}

void GCS_MAVLINK::handle_att_pos_mocap(mavlink_message_t *msg)
{
    mavlink_att_pos_mocap_t m;
    mavlink_msg_att_pos_mocap_decode(msg, &m);

    // sensor assumed to be at 0,0,0 body-frame; need parameters for this?
    const Vector3f sensor_offset = {};
    const Vector3f pos = {
        m.x,
        m.y,
        m.z
    };
    Quaternion attitude = Quaternion(m.q);
    const float posErr = 0; // parameter required?
    const float angErr = 0; // parameter required?
    // correct offboard timestamp to be in local ms since boot
    uint32_t timestamp_ms = correct_offboard_timestamp_usec_to_ms(m.time_usec, PAYLOAD_SIZE(chan, ATT_POS_MOCAP));
    const uint32_t reset_timestamp_ms = 0; // no data available

    AP::ahrs().writeExtNavData(sensor_offset,
                               pos,
                               attitude,
                               posErr,
                               angErr,
                               timestamp_ms,
                               reset_timestamp_ms);
   
    // calculate euler orientation for logging
    float roll;
    float pitch;
    float yaw;
    attitude.to_euler(roll, pitch, yaw);

    log_vision_position_estimate_data(m.time_usec, m.x, m.y, m.z, roll, pitch, yaw);
}

void GCS_MAVLINK::handle_command_ack(const mavlink_message_t* msg)
{
    AP_AccelCal *accelcal = AP::ins().get_acal();
    if (accelcal != nullptr) {
        accelcal->handleMessage(msg);
    }
}

/*
  handle messages which don't require vehicle specific data
 */
void GCS_MAVLINK::handle_common_message(mavlink_message_t *msg)
{
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_COMMAND_ACK: {
        handle_command_ack(msg);
        break;
    }

    case MAVLINK_MSG_ID_SETUP_SIGNING:
        handle_setup_signing(msg);
        break;

    case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    case MAVLINK_MSG_ID_PARAM_SET:
    case MAVLINK_MSG_ID_PARAM_REQUEST_READ:
        handle_common_param_message(msg);
        break;

    case MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN:
        handle_set_gps_global_origin(msg);
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
    case MAVLINK_MSG_ID_LOG_REQUEST_DATA:
    case MAVLINK_MSG_ID_LOG_ERASE:
    case MAVLINK_MSG_ID_LOG_REQUEST_END:
    case MAVLINK_MSG_ID_REMOTE_LOG_BLOCK_STATUS:
        DataFlash_Class::instance()->handle_mavlink_msg(*this, msg);
        break;


    case MAVLINK_MSG_ID_DIGICAM_CONTROL:
        {
            AP_Camera *camera = AP::camera();
            if (camera == nullptr) {
                return;
            }
            camera->control_msg(msg);
        }
        break;

    case MAVLINK_MSG_ID_SET_MODE:
        handle_set_mode(msg);
        break;

    case MAVLINK_MSG_ID_AUTOPILOT_VERSION_REQUEST:
        handle_send_autopilot_version(msg);
        break;

    case MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST:
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
    case MAVLINK_MSG_ID_MISSION_COUNT:
    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:
    case MAVLINK_MSG_ID_MISSION_ITEM:
    case MAVLINK_MSG_ID_MISSION_ITEM_INT:
    case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
    case MAVLINK_MSG_ID_MISSION_REQUEST:
    case MAVLINK_MSG_ID_MISSION_ACK:
    case MAVLINK_MSG_ID_MISSION_SET_CURRENT:
        handle_common_mission_message(msg);
        break;

    case MAVLINK_MSG_ID_COMMAND_LONG:
        handle_command_long(msg);
        break;

    case MAVLINK_MSG_ID_COMMAND_INT:
        handle_command_int(msg);
        break;

    case MAVLINK_MSG_ID_GIMBAL_REPORT:
        handle_mount_message(msg);
        break;

    case MAVLINK_MSG_ID_PARAM_VALUE:
        handle_param_value(msg);
        break;

    case MAVLINK_MSG_ID_SERIAL_CONTROL:
        handle_serial_control(msg);
        break;

    case MAVLINK_MSG_ID_GPS_RTCM_DATA:
    case MAVLINK_MSG_ID_GPS_INPUT:
    case MAVLINK_MSG_ID_HIL_GPS:
    case MAVLINK_MSG_ID_GPS_INJECT_DATA:
        AP::gps().handle_msg(msg);
        break;

    case MAVLINK_MSG_ID_STATUSTEXT:
        handle_statustext(msg);
        break;

    case MAVLINK_MSG_ID_LED_CONTROL:
        // send message to Notify
        AP_Notify::handle_led_control(msg);
        break;

    case MAVLINK_MSG_ID_MOUNT_CONFIGURE: // deprecated. Use MAV_CMD_DO_MOUNT_CONFIGURE
    case MAVLINK_MSG_ID_MOUNT_CONTROL: // deprecated. Use MAV_CMD_DO_MOUNT_CONTROL
        handle_mount_message(msg);
        break;

    case MAVLINK_MSG_ID_PLAY_TUNE:
        // send message to Notify
        AP_Notify::handle_play_tune(msg);
        break;

    case MAVLINK_MSG_ID_RALLY_POINT:
    case MAVLINK_MSG_ID_RALLY_FETCH_POINT:
        handle_common_rally_message(msg);
        break;

    case MAVLINK_MSG_ID_REQUEST_DATA_STREAM:
        handle_request_data_stream(msg);
        break;

    case MAVLINK_MSG_ID_DATA96:
        handle_data_packet(msg);
        break;        

    case MAVLINK_MSG_ID_VISION_POSITION_DELTA:
        handle_vision_position_delta(msg);
        break;

    case MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE:
        handle_vision_position_estimate(msg);
        break;

    case MAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE:
        handle_global_vision_position_estimate(msg);
        break;

    case MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE:
        handle_vicon_position_estimate(msg);
        break;

    case MAVLINK_MSG_ID_ATT_POS_MOCAP:
        handle_att_pos_mocap(msg);
        break;

    case MAVLINK_MSG_ID_SYSTEM_TIME:
        handle_system_time_message(msg);
        break;
    }

}

void GCS_MAVLINK::handle_common_mission_message(mavlink_message_t *msg)
{
    AP_Mission *_mission = AP::mission();
    if (_mission == nullptr) {
        return;
    }
    switch (msg->msgid) {
    case MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST: // MAV ID: 38
    {
        handle_mission_write_partial_list(*_mission, msg);
        break;
    }

    // GCS has sent us a mission item, store to EEPROM
    case MAVLINK_MSG_ID_MISSION_ITEM:           // MAV ID: 39
    case MAVLINK_MSG_ID_MISSION_ITEM_INT:
    {
        if (handle_mission_item(msg, *_mission)) {
            DataFlash_Class::instance()->Log_Write_EntireMission(*_mission);
        }
        break;
    }

    // read an individual command from EEPROM and send it to the GCS
    case MAVLINK_MSG_ID_MISSION_REQUEST_INT:
    case MAVLINK_MSG_ID_MISSION_REQUEST:     // MAV ID: 40, 51
    {
        handle_mission_request(*_mission, msg);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_SET_CURRENT:    // MAV ID: 41
    {
        handle_mission_set_current(*_mission, msg);
        break;
    }

    // GCS request the full list of commands, we return just the number and leave the GCS to then request each command individually
    case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:       // MAV ID: 43
    {
        handle_mission_request_list(*_mission, msg);
        break;
    }

    // GCS provides the full number of commands it wishes to upload
    //  individual commands will then be sent from the GCS using the MAVLINK_MSG_ID_MISSION_ITEM message
    case MAVLINK_MSG_ID_MISSION_COUNT:          // MAV ID: 44
    {
        handle_mission_count(*_mission, msg);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_CLEAR_ALL:      // MAV ID: 45
    {
        handle_mission_clear_all(*_mission, msg);
        break;
    }

    case MAVLINK_MSG_ID_MISSION_ACK:
        /* not used */
        break;
    }
}

void GCS_MAVLINK::handle_send_autopilot_version(const mavlink_message_t *msg)
{
    send_autopilot_version();
}

void GCS_MAVLINK::send_banner()
{
    // mark the firmware version in the tlog
    const AP_FWVersion &fwver = AP::fwversion();

    send_text(MAV_SEVERITY_INFO, fwver.fw_string);

    if (fwver.middleware_name && fwver.os_name) {
        send_text(MAV_SEVERITY_INFO, "%s: %s %s: %s",
                  fwver.middleware_name, fwver.middleware_hash_str,
                  fwver.os_name, fwver.os_hash_str);
    } else if (fwver.os_name) {
        send_text(MAV_SEVERITY_INFO, "%s: %s",
                  fwver.os_name, fwver.os_hash_str);
    }

    // send system ID if we can
    char sysid[40];
    if (hal.util->get_system_id(sysid)) {
        send_text(MAV_SEVERITY_INFO, sysid);
    }
}


void GCS_MAVLINK::send_simstate() const
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    SITL::SITL *sitl = AP::sitl();
    if (sitl == nullptr) {
        return;
    }
    sitl->simstate_send(get_chan());
#endif
}

MAV_RESULT GCS_MAVLINK::handle_command_flash_bootloader(const mavlink_command_long_t &packet)
{
    if (uint32_t(packet.param5) != 290876) {
        gcs().send_text(MAV_SEVERITY_INFO, "Magic not set");
        return MAV_RESULT_FAILED;
    }

    if (!hal.util->flash_bootloader()) {
        return MAV_RESULT_FAILED;
    }

    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT GCS_MAVLINK::handle_command_preflight_set_sensor_offsets(const mavlink_command_long_t &packet)
{
    Compass &compass = AP::compass();

    uint8_t compassNumber = -1;
    if (is_equal(packet.param1, 2.0f)) {
        compassNumber = 0;
    } else if (is_equal(packet.param1, 5.0f)) {
        compassNumber = 1;
    } else if (is_equal(packet.param1, 6.0f)) {
        compassNumber = 2;
    }
    if (compassNumber == (uint8_t) -1) {
        return MAV_RESULT_FAILED;
    }
    compass.set_and_save_offsets(compassNumber, packet.param2, packet.param3, packet.param4);
    return MAV_RESULT_ACCEPTED;
}

bool GCS_MAVLINK::calibrate_gyros()
{
    AP::ins().init_gyro();
    if (!AP::ins().gyro_calibrated_ok_all()) {
        return false;
    }
    AP::ahrs().reset_gyro_drift();
    return true;
}

MAV_RESULT GCS_MAVLINK::_handle_command_preflight_calibration_baro()
{
    // fast barometer calibration
    gcs().send_text(MAV_SEVERITY_INFO, "Updating barometer calibration");
    AP::baro().update_calibration();
    gcs().send_text(MAV_SEVERITY_INFO, "Barometer calibration complete");

    AP_Airspeed *airspeed = AP_Airspeed::get_singleton();
    if (airspeed != nullptr) {
        airspeed->calibrate(false);
    }

    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT GCS_MAVLINK::_handle_command_preflight_calibration(const mavlink_command_long_t &packet)
{
    if (is_equal(packet.param1,1.0f)) {
        if (!calibrate_gyros()) {
            return MAV_RESULT_FAILED;
        }
        return MAV_RESULT_ACCEPTED;
    }

    if (is_equal(packet.param3,1.0f)) {
        return _handle_command_preflight_calibration_baro();
    }

    if (is_equal(packet.param5,1.0f)) {
        // start with gyro calibration
        if (!calibrate_gyros()) {
            return MAV_RESULT_FAILED;
        }
        // start accel cal
        AP::ins().acal_init();
        AP::ins().get_acal()->start(this);
        return MAV_RESULT_ACCEPTED;
    }

    if (is_equal(packet.param5,2.0f)) {
        if (!calibrate_gyros()) {
            return MAV_RESULT_FAILED;
        }
        float trim_roll, trim_pitch;
        if (!AP::ins().calibrate_trim(trim_roll, trim_pitch)) {
            return MAV_RESULT_FAILED;
        }
        // reset ahrs's trim to suggested values from calibration routine
        AP::ahrs().set_trim(Vector3f(trim_roll, trim_pitch, 0));
        return MAV_RESULT_ACCEPTED;
    }

    if (is_equal(packet.param5,4.0f)) {
        // simple accel calibration
        return AP::ins().simple_accel_cal();
    }

    return MAV_RESULT_UNSUPPORTED;
}

MAV_RESULT GCS_MAVLINK::handle_command_preflight_calibration(const mavlink_command_long_t &packet)
{
    if (hal.util->get_soft_armed()) {
        // *preflight*, remember?
        return MAV_RESULT_FAILED;
    }
    // now call subclass methods:
    return _handle_command_preflight_calibration(packet);
}

MAV_RESULT GCS_MAVLINK::handle_command_mag_cal(const mavlink_command_long_t &packet)
{
    return AP::compass().handle_mag_cal_command(packet);
}

MAV_RESULT GCS_MAVLINK::handle_command_request_autopilot_capabilities(const mavlink_command_long_t &packet)
{
    if (!is_equal(packet.param1,1.0f)) {
        return MAV_RESULT_FAILED;
    }

    send_autopilot_version();

    return MAV_RESULT_ACCEPTED;
}


MAV_RESULT GCS_MAVLINK::handle_command_do_send_banner(const mavlink_command_long_t &packet)
{
    send_banner();
    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT GCS_MAVLINK::handle_command_do_set_mode(const mavlink_command_long_t &packet)
{
    const MAV_MODE _base_mode = (MAV_MODE)packet.param1;
    const uint32_t _custom_mode = (uint32_t)packet.param2;

    return _set_mode_common(_base_mode, _custom_mode);
}

MAV_RESULT GCS_MAVLINK::handle_command_get_home_position(const mavlink_command_long_t &packet)
{
    if (!AP::ahrs().home_is_set()) {
        return MAV_RESULT_FAILED;
    }
    send_home();
    send_ekf_origin();

    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT GCS_MAVLINK::handle_command_do_gripper(const mavlink_command_long_t &packet)
{
    AP_Gripper *gripper = AP::gripper();
    if (gripper == nullptr) {
        return MAV_RESULT_FAILED;
    }

    // param1 : gripper number (ignored)
    // param2 : action (0=release, 1=grab). See GRIPPER_ACTIONS enum.
    if(!gripper->enabled()) {
        return MAV_RESULT_FAILED;
    }

    MAV_RESULT result = MAV_RESULT_ACCEPTED;

    switch ((uint8_t)packet.param2) {
    case GRIPPER_ACTION_RELEASE:
        gripper->release();
        break;
    case GRIPPER_ACTION_GRAB:
        gripper->grab();
        break;
    default:
        result = MAV_RESULT_FAILED;
        break;
    }

    return result;
}

MAV_RESULT GCS_MAVLINK::handle_command_accelcal_vehicle_pos(const mavlink_command_long_t &packet)
{
    if (!AP::ins().get_acal()->gcs_vehicle_position(packet.param1)) {
        return MAV_RESULT_FAILED;
    }
    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT GCS_MAVLINK::handle_command_mount(const mavlink_command_long_t &packet)
{
    AP_Mount *mount = AP::mount();
    if (mount == nullptr) {
        return MAV_RESULT_UNSUPPORTED;
    }
    return mount->handle_command_long(packet);
}

MAV_RESULT GCS_MAVLINK::handle_command_long_packet(const mavlink_command_long_t &packet)
{
    MAV_RESULT result = MAV_RESULT_FAILED;

    switch (packet.command) {

    case MAV_CMD_ACCELCAL_VEHICLE_POS:
        result = handle_command_accelcal_vehicle_pos(packet);
        break;

    case MAV_CMD_DO_SET_MODE:
        result = handle_command_do_set_mode(packet);
        break;

    case MAV_CMD_DO_SEND_BANNER:
        result = handle_command_do_send_banner(packet);
        break;

    case MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN:
        result = handle_preflight_reboot(packet);
        break;

    case MAV_CMD_DO_START_MAG_CAL:
    case MAV_CMD_DO_ACCEPT_MAG_CAL:
    case MAV_CMD_DO_CANCEL_MAG_CAL: {
        result = handle_command_mag_cal(packet);
        break;
    }

    case MAV_CMD_START_RX_PAIR:
        result = handle_rc_bind(packet);
        break;

    case MAV_CMD_DO_DIGICAM_CONFIGURE:
    case MAV_CMD_DO_DIGICAM_CONTROL:
    case MAV_CMD_DO_SET_CAM_TRIGG_DIST:
        result = handle_command_camera(packet);
        break;

    case MAV_CMD_DO_GRIPPER:
        result = handle_command_do_gripper(packet);
        break;

    case MAV_CMD_DO_MOUNT_CONFIGURE:
    case MAV_CMD_DO_MOUNT_CONTROL:
        result = handle_command_mount(packet);
        break;

    case MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES: {
        result = handle_command_request_autopilot_capabilities(packet);
        break;
    }

    case MAV_CMD_DO_SET_ROI_LOCATION:
    case MAV_CMD_DO_SET_ROI:
        result = handle_command_do_set_roi(packet);
        break;

    case MAV_CMD_PREFLIGHT_CALIBRATION:
        result = handle_command_preflight_calibration(packet);
        break;

    case MAV_CMD_FLASH_BOOTLOADER:
        result = handle_command_flash_bootloader(packet);
        break;

    case MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS: {
        result = handle_command_preflight_set_sensor_offsets(packet);
        break;
    }

    case MAV_CMD_GET_HOME_POSITION:
        result = handle_command_get_home_position(packet);
        break;

    case MAV_CMD_PREFLIGHT_STORAGE:
        if (is_equal(packet.param1, 2.0f)) {
            AP_Param::erase_all();
            send_text(MAV_SEVERITY_WARNING, "All parameters reset, reboot board");
            result= MAV_RESULT_ACCEPTED;
        }
        break;

    case MAV_CMD_SET_MESSAGE_INTERVAL:
        result = handle_command_set_message_interval(packet);
        break;

    case MAV_CMD_DO_SET_SERVO:
    case MAV_CMD_DO_REPEAT_SERVO:
    case MAV_CMD_DO_SET_RELAY:
    case MAV_CMD_DO_REPEAT_RELAY:
        result = handle_servorelay_message(packet);
        break;

    case MAV_CMD_DO_FLIGHTTERMINATION:
        result = handle_flight_termination(packet);
        break;

    default:
        result = MAV_RESULT_UNSUPPORTED;
        break;
    }

    return result;
}

void GCS_MAVLINK::handle_command_long(mavlink_message_t *msg)
{
    // decode packet
    mavlink_command_long_t packet;
    mavlink_msg_command_long_decode(msg, &packet);

    const MAV_RESULT result = handle_command_long_packet(packet);

    // send ACK or NAK
    mavlink_msg_command_ack_send_buf(msg, chan, packet.command, result);
}

MAV_RESULT GCS_MAVLINK::handle_command_do_set_roi(const Location &roi_loc)
{
    AP_Mount *mount = AP::mount();
    if (mount == nullptr) {
        return MAV_RESULT_UNSUPPORTED;
    }

    // sanity check location
    if (!check_latlng(roi_loc)) {
        return MAV_RESULT_FAILED;
    }

    if (roi_loc.lat == 0 && roi_loc.lng == 0 && roi_loc.alt == 0) {
        // switch off the camera tracking if enabled
        if (mount->get_mode() == MAV_MOUNT_MODE_GPS_POINT) {
            mount->set_mode_to_default();
        }
    } else {
        // send the command to the camera mount
        mount->set_roi_target(roi_loc);
    }
    return MAV_RESULT_ACCEPTED;
}

MAV_RESULT GCS_MAVLINK::handle_command_do_set_roi(const mavlink_command_int_t &packet)
{
    // be aware that this method is called for both MAV_CMD_DO_SET_ROI
    // and MAV_CMD_DO_SET_ROI_LOCATION.  If you intend to support any
    // of the extra fields in the former then you will need to split
    // off support for MAV_CMD_DO_SET_ROI_LOCATION (which doesn't
    // support the extra fields).

    // param1 : /* Region of interest mode (not used)*/
    // param2 : /* MISSION index/ target ID (not used)*/
    // param3 : /* ROI index (not used)*/
    // param4 : /* empty */
    // x : lat
    // y : lon
    // z : alt
    Location roi_loc;
    roi_loc.lat = packet.x;
    roi_loc.lng = packet.y;
    roi_loc.alt = (int32_t)(packet.z * 100.0f);
    return handle_command_do_set_roi(roi_loc);
}

MAV_RESULT GCS_MAVLINK::handle_command_do_set_roi(const mavlink_command_long_t &packet)
{
    // be aware that this method is called for both MAV_CMD_DO_SET_ROI
    // and MAV_CMD_DO_SET_ROI_LOCATION.  If you intend to support any
    // of the extra fields in the former then you will need to split
    // off support for MAV_CMD_DO_SET_ROI_LOCATION (which doesn't
    // support the extra fields).

    Location roi_loc;
    roi_loc.lat = (int32_t)(packet.param5 * 1.0e7f);
    roi_loc.lng = (int32_t)(packet.param6 * 1.0e7f);
    roi_loc.alt = (int32_t)(packet.param7 * 100.0f);
    return handle_command_do_set_roi(roi_loc);
}

MAV_RESULT GCS_MAVLINK::handle_command_int_packet(const mavlink_command_int_t &packet)
{
    switch (packet.command) {
    case MAV_CMD_DO_SET_ROI:
    case MAV_CMD_DO_SET_ROI_LOCATION:
        return handle_command_do_set_roi(packet);
    default:
        break;
    }

    return MAV_RESULT_UNSUPPORTED;
}

void GCS_MAVLINK::handle_command_int(mavlink_message_t *msg)
{
    // decode packet
    mavlink_command_int_t packet;
    mavlink_msg_command_int_decode(msg, &packet);

    const MAV_RESULT result = handle_command_int_packet(packet);

    // send ACK or NAK
    mavlink_msg_command_ack_send_buf(msg, chan, packet.command, result);
}

bool GCS_MAVLINK::try_send_compass_message(const enum ap_message id)
{
    Compass &compass = AP::compass();
    bool ret = true;
    switch (id) {
    case MSG_MAG_CAL_PROGRESS:
        compass.send_mag_cal_progress(chan);
        ret = true;;
        break;
    case MSG_MAG_CAL_REPORT:
        compass.send_mag_cal_report(chan);
        ret = true;
        break;
    default:
        ret = true;
        break;
    }
    return ret;
}

bool GCS_MAVLINK::try_send_mission_message(const enum ap_message id)
{
    AP_Mission *mission = AP::mission();
    if (mission == nullptr) {
        return true;
    }

    bool ret = true;
    switch (id) {
    case MSG_CURRENT_WAYPOINT:
        CHECK_PAYLOAD_SIZE(MISSION_CURRENT);
        mavlink_msg_mission_current_send(chan, mission->get_current_nav_index());
        ret = true;
        break;
    case MSG_MISSION_ITEM_REACHED:
        CHECK_PAYLOAD_SIZE(MISSION_ITEM_REACHED);
        mavlink_msg_mission_item_reached_send(chan, mission_item_reached_index);
        ret = true;
        break;
    case MSG_NEXT_MISSION_REQUEST:
        CHECK_PAYLOAD_SIZE(MISSION_REQUEST);
        queued_mission_request_send();
        ret = true;
        break;
    default:
        ret = true;
        break;
    }
    return ret;
}

void GCS_MAVLINK::send_hwstatus()
{
    mavlink_msg_hwstatus_send(
        chan,
        hal.analogin->board_voltage()*1000,
        0);
}

void GCS_MAVLINK::send_attitude() const
{
    const AP_AHRS &ahrs = AP::ahrs();
    const Vector3f omega = ahrs.get_gyro();
    mavlink_msg_attitude_send(
        chan,
        AP_HAL::millis(),
        ahrs.roll,
        ahrs.pitch,
        ahrs.yaw,
        omega.x,
        omega.y,
        omega.z);
}

int32_t GCS_MAVLINK::global_position_int_alt() const {
    return global_position_current_loc.alt * 10UL;
}
int32_t GCS_MAVLINK::global_position_int_relative_alt() const {
    float posD;
    AP::ahrs().get_relative_position_D_home(posD);
    posD *= -1000.0f; // change from down to up and metres to millimeters
    return posD;
}
void GCS_MAVLINK::send_global_position_int()
{
    AP_AHRS &ahrs = AP::ahrs();

    ahrs.get_position(global_position_current_loc); // return value ignored; we send stale data

    Vector3f vel;
    ahrs.get_velocity_NED(vel);

    mavlink_msg_global_position_int_send(
        chan,
        AP_HAL::millis(),
        global_position_current_loc.lat, // in 1E7 degrees
        global_position_current_loc.lng, // in 1E7 degrees
        global_position_int_alt(),       // millimeters above ground/sea level
        global_position_int_relative_alt(), // millimeters above home
        vel.x * 100,                     // X speed cm/s (+ve North)
        vel.y * 100,                     // Y speed cm/s (+ve East)
        vel.z * 100,                     // Z speed cm/s (+ve Down)
        ahrs.yaw_sensor);                // compass heading in 1/100 degree
}

void GCS_MAVLINK::send_gimbal_report() const
{
    AP_Mount *mount = AP::mount();
    if (mount == nullptr) {
        return;
    }
    mount->send_gimbal_report(chan);
}

void GCS_MAVLINK::send_mount_status() const
{
    AP_Mount *mount = AP::mount();
    if (mount == nullptr) {
        return;
    }
    mount->send_mount_status(chan);
}

bool GCS_MAVLINK::try_send_message(const enum ap_message id)
{
    bool ret = true;

    switch(id) {

    case MSG_ATTITUDE:
        CHECK_PAYLOAD_SIZE(ATTITUDE);
        send_attitude();
        break;

    case MSG_NEXT_PARAM:
        CHECK_PAYLOAD_SIZE(PARAM_VALUE);
        queued_param_send();
        break;

    case MSG_GIMBAL_REPORT:
        CHECK_PAYLOAD_SIZE(GIMBAL_REPORT);
        send_gimbal_report();
        break;

    case MSG_HEARTBEAT:
        CHECK_PAYLOAD_SIZE(HEARTBEAT);
        last_heartbeat_time = AP_HAL::millis();
        send_heartbeat();
        break;

    case MSG_HWSTATUS:
        CHECK_PAYLOAD_SIZE(HWSTATUS);
        send_hwstatus();
        break;

    case MSG_LOCATION:
        CHECK_PAYLOAD_SIZE(GLOBAL_POSITION_INT);
        send_global_position_int();
        break;

    case MSG_CURRENT_WAYPOINT:
    case MSG_MISSION_ITEM_REACHED:
    case MSG_NEXT_MISSION_REQUEST:
        ret = try_send_mission_message(id);
        break;

    case MSG_MAG_CAL_PROGRESS:
    case MSG_MAG_CAL_REPORT:
        ret = try_send_compass_message(id);
        break;

    case MSG_BATTERY_STATUS:
        send_battery_status();
        break;

    case MSG_BATTERY2:
        CHECK_PAYLOAD_SIZE(BATTERY2);
        send_battery2();
        break;

    case MSG_EKF_STATUS_REPORT:
#if AP_AHRS_NAVEKF_AVAILABLE
        CHECK_PAYLOAD_SIZE(EKF_STATUS_REPORT);
        AP::ahrs_navekf().send_ekf_status_report(chan);
#endif
        break;

    case MSG_MEMINFO:
        CHECK_PAYLOAD_SIZE(MEMINFO);
        send_meminfo();
        break;

    case MSG_RANGEFINDER:
        CHECK_PAYLOAD_SIZE(RANGEFINDER);
        send_rangefinder();
        break;

    case MSG_DISTANCE_SENSOR:
        send_distance_sensor();
        break;

    case MSG_CAMERA_FEEDBACK:
        {
            AP_Camera *camera = AP::camera();
            if (camera == nullptr) {
                break;
            }
            CHECK_PAYLOAD_SIZE(CAMERA_FEEDBACK);
            camera->send_feedback(chan);
        }
        break;

    case MSG_SYSTEM_TIME:
        CHECK_PAYLOAD_SIZE(SYSTEM_TIME);
        send_system_time();
        break;
    case MSG_GPS_RAW:
        CHECK_PAYLOAD_SIZE(GPS_RAW_INT);
        AP::gps().send_mavlink_gps_raw(chan);
        break;
    case MSG_GPS_RTK:
        CHECK_PAYLOAD_SIZE(GPS_RTK);
        AP::gps().send_mavlink_gps_rtk(chan, 0);
        break;
    case MSG_GPS2_RAW:
        CHECK_PAYLOAD_SIZE(GPS2_RAW);
        AP::gps().send_mavlink_gps2_raw(chan);
        break;
    case MSG_GPS2_RTK:
        CHECK_PAYLOAD_SIZE(GPS2_RTK);
        AP::gps().send_mavlink_gps_rtk(chan, 1);
        break;

    case MSG_LOCAL_POSITION:
        CHECK_PAYLOAD_SIZE(LOCAL_POSITION_NED);
        send_local_position();
        break;

    case MSG_MOUNT_STATUS:
        CHECK_PAYLOAD_SIZE(MOUNT_STATUS);
        send_mount_status();
        break;

    case MSG_OPTICAL_FLOW:
        CHECK_PAYLOAD_SIZE(OPTICAL_FLOW);
        send_opticalflow();
        break;

    case MSG_POSITION_TARGET_GLOBAL_INT:
        CHECK_PAYLOAD_SIZE(POSITION_TARGET_GLOBAL_INT);
        send_position_target_global_int();
        break;

    case MSG_POWER_STATUS:
        CHECK_PAYLOAD_SIZE(POWER_STATUS);
        send_power_status();
        break;

    case MSG_RADIO_IN:
        CHECK_PAYLOAD_SIZE(RC_CHANNELS_RAW);
        send_radio_in();
        break;

    case MSG_RAW_IMU:
        CHECK_PAYLOAD_SIZE(RAW_IMU);
        send_raw_imu();
        break;

    case MSG_SCALED_IMU:
        CHECK_PAYLOAD_SIZE(SCALED_IMU);
        send_scaled_imu(0, mavlink_msg_scaled_imu_send);
        break;

    case MSG_SCALED_IMU2:
        CHECK_PAYLOAD_SIZE(SCALED_IMU2);
        send_scaled_imu(1, mavlink_msg_scaled_imu2_send);
        break;

    case MSG_SCALED_IMU3:
        CHECK_PAYLOAD_SIZE(SCALED_IMU3);
        send_scaled_imu(2, mavlink_msg_scaled_imu3_send);
        break;

    case MSG_SCALED_PRESSURE:
        CHECK_PAYLOAD_SIZE(SCALED_PRESSURE);
        send_scaled_pressure();
        break;

    case MSG_SCALED_PRESSURE2:
        CHECK_PAYLOAD_SIZE(SCALED_PRESSURE2);
        send_scaled_pressure2();
        break;

    case MSG_SCALED_PRESSURE3:
        CHECK_PAYLOAD_SIZE(SCALED_PRESSURE3);
        send_scaled_pressure3();
        break;

    case MSG_SENSOR_OFFSETS:
        CHECK_PAYLOAD_SIZE(SENSOR_OFFSETS);
        send_sensor_offsets();
        break;

    case MSG_SERVO_OUTPUT_RAW:
        CHECK_PAYLOAD_SIZE(SERVO_OUTPUT_RAW);
        send_servo_output_raw();
        break;

    case MSG_SIMSTATE:
        CHECK_PAYLOAD_SIZE(SIMSTATE);
        send_simstate();
        break;

    case MSG_AHRS2:
        CHECK_PAYLOAD_SIZE(AHRS2);
        send_ahrs2();
        break;

    case MSG_AHRS3:
        CHECK_PAYLOAD_SIZE(AHRS3);
        send_ahrs3();
        break;

    case MSG_AHRS:
        CHECK_PAYLOAD_SIZE(AHRS);
        send_ahrs();
        break;

    case MSG_VFR_HUD:
        CHECK_PAYLOAD_SIZE(VFR_HUD);
        send_vfr_hud();
        break;

    case MSG_VIBRATION:
        CHECK_PAYLOAD_SIZE(VIBRATION);
        send_vibration();
        break;

    case MSG_ESC_TELEMETRY: {
#ifdef HAVE_AP_BLHELI_SUPPORT
        CHECK_PAYLOAD_SIZE(ESC_TELEMETRY_1_TO_4);
        AP_BLHeli *blheli = AP_BLHeli::get_singleton();
        if (blheli) {
            blheli->send_esc_telemetry_mavlink(uint8_t(chan));
        }
#endif
        break;
    }

    default:
        // try_send_message must always at some stage return true for
        // a message, or we will attempt to infinitely retry the
        // message as part of send_message.
        // This message will be sent out at the same rate as the
        // unknown message, so should be safe.
        gcs().send_text(MAV_SEVERITY_DEBUG, "Sending unknown message (%u)", id);
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
        AP_HAL::panic("Sending unknown ap_message %u", id);
#endif
        break;
    }

    return ret;
}

uint16_t GCS_MAVLINK::get_interval_for_stream(GCS_MAVLINK::streams id) const
{
    const int16_t frate = streamRates[id].get();
    if (frate == 0) {
        return 0;
    }
    const uint32_t ret = 1000/frate;
    if (ret > 60000) {
        return 60000;
    }
    return ret;
}

void GCS_MAVLINK::initialise_message_intervals_for_stream(GCS_MAVLINK::streams id)
{
    for (uint8_t i=0; all_stream_entries[i].ap_message_ids != nullptr; i++) {
        const GCS_MAVLINK::stream_entries &entries = all_stream_entries[i];
        if (entries.stream_id != id) {
            continue;
        }
        // found it!
        const uint16_t interval_ms = get_interval_for_stream(id);
        for (uint8_t j=0; j<entries.num_ap_message_ids; j++) {
            set_ap_message_interval(entries.ap_message_ids[j], interval_ms);
        }
        break;
    }
}

void GCS_MAVLINK::initialise_message_intervals_from_streamrates()
{
    // this is O(n^2), but it's once at boot and across a 10-entry list...
    for (uint8_t i=0; all_stream_entries[i].ap_message_ids != nullptr; i++) {
        initialise_message_intervals_for_stream(all_stream_entries[i].stream_id);
    }
}

bool GCS_MAVLINK::get_default_interval_for_ap_message(const ap_message id, uint16_t &interval) const
{
    // find which stream this ap_message is in
    for (uint8_t i=0; all_stream_entries[i].ap_message_ids != nullptr; i++) {
        const GCS_MAVLINK::stream_entries &entries = all_stream_entries[i];
        for (uint8_t j=0; j<entries.num_ap_message_ids; j++) {
            if (entries.ap_message_ids[j] == id) {
                interval = get_interval_for_stream(all_stream_entries[i].stream_id);
                return true;
            }
        }
    }
    return false;
}

bool GCS_MAVLINK::get_default_interval_for_mavlink_message_id(const uint32_t mavlink_message_id, uint16_t &interval) const
{
    const ap_message id = mavlink_id_to_ap_message_id(mavlink_message_id);
    if (id == MSG_LAST) {
        return false;
    }

    return get_default_interval_for_ap_message(id, interval);
}

/*
  correct an offboard timestamp in microseconds into a local timestamp
  since boot in milliseconds. See the JitterCorrection code for details

  Return a value in milliseconds since boot (for use by the EKF)
 */
uint32_t GCS_MAVLINK::correct_offboard_timestamp_usec_to_ms(uint64_t offboard_usec, uint16_t payload_size)
{
    uint64_t local_us;
    // if the HAL supports it then constrain the latest possible time
    // the packet could have been sent by the uart receive time and
    // the baudrate and packet size.
    uint64_t uart_receive_time = _port->receive_time_constraint_us(payload_size);
    if (uart_receive_time != 0) {
        local_us = uart_receive_time;
    } else {
        local_us = AP_HAL::micros64();
    }
    uint64_t corrected_us = lag_correction.correct_offboard_timestamp_usec(offboard_usec, local_us);

    return corrected_us / 1000U;
}

/*
  return true if we will accept this packet. Used to implement SYSID_ENFORCE
 */
bool GCS_MAVLINK::accept_packet(const mavlink_status_t &status,
                                mavlink_message_t &msg)
{
    if (msg.sysid == mavlink_system.sysid) {
        // accept packets from our own components
        // (e.g. mavlink-connected companion computers)
        return true;
    }

    if (msg.sysid == sysid_my_gcs()) {
        return true;
    }

    if (msg.msgid == MAVLINK_MSG_ID_RADIO ||
        msg.msgid == MAVLINK_MSG_ID_RADIO_STATUS) {
        return true;
    }

    if (!sysid_enforce()) {
        return true;
    }

    return false;
}

/*
  update UART pass-thru, if enabled
 */
void GCS::update_passthru(void)
{
    WITH_SEMAPHORE(_passthru.sem);
    uint32_t now = AP_HAL::millis();

    bool enabled = AP::serialmanager().get_passthru(_passthru.port1, _passthru.port2, _passthru.timeout_s);
    if (enabled && !_passthru.enabled) {
        _passthru.start_ms = now;
        _passthru.last_ms = 0;
        _passthru.enabled = true;
        _passthru.last_port1_data_ms = now;
        gcs().send_text(MAV_SEVERITY_INFO, "Passthru enabled");
        if (!_passthru.timer_installed) {
            _passthru.timer_installed = true;
            hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&GCS::passthru_timer, void));
        }
    } else if (!enabled && _passthru.enabled) {
        _passthru.enabled = false;
        _passthru.port1->lock_port(0, 0);
        _passthru.port2->lock_port(0, 0);
        gcs().send_text(MAV_SEVERITY_INFO, "Passthru disabled");
    } else if (enabled &&
               _passthru.timeout_s &&
               now - _passthru.last_port1_data_ms > uint32_t(_passthru.timeout_s)*1000U) {
        // timed out, disable
        _passthru.enabled = false;
        _passthru.port1->lock_port(0, 0);
        _passthru.port2->lock_port(0, 0);
        AP::serialmanager().disable_passthru();
        gcs().send_text(MAV_SEVERITY_INFO, "Passthru timed out");
    }
}

/*
  called at 1kHz to handle pass-thru between SERIA0_PASSTHRU port and hal.console
 */
void GCS::passthru_timer(void)
{
    WITH_SEMAPHORE(_passthru.sem);

    if (!_passthru.enabled) {
        // it has been disabled after starting
        return;
    }
    if (_passthru.start_ms != 0) {
        uint32_t now = AP_HAL::millis();
        if (now - _passthru.start_ms < 1000) {
            // delay for 1s so the reply for the SERIAL0_PASSTHRU param set can be seen by GCS
            return;
        }
        _passthru.start_ms = 0;
    }

    // while pass-thru is enabled lock both ports. They remain
    // locked until disabled again, or reboot
    const uint32_t lock_key = 0x3256AB9F;
    _passthru.port1->lock_port(lock_key, lock_key);
    _passthru.port2->lock_port(lock_key, lock_key);

    int16_t b;
    uint8_t buf[64];
    uint8_t nbytes = 0;

    // read from port1, and write to port2
    while (nbytes < sizeof(buf) && (b = _passthru.port1->read_locked(lock_key)) >= 0) {
        buf[nbytes++] = b;
    }
    if (nbytes > 0) {
        _passthru.last_port1_data_ms = AP_HAL::millis();
        _passthru.port2->write_locked(buf, nbytes, lock_key);
    }

    // read from port2, and write to port1
    nbytes = 0;
    while (nbytes < sizeof(buf) && (b = _passthru.port2->read_locked(lock_key)) >= 0) {
        buf[nbytes++] = b;
    }
    if (nbytes > 0) {
        _passthru.port1->write_locked(buf, nbytes, lock_key);
    }
}


GCS &gcs()
{
    return *GCS::instance();
}
