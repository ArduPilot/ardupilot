/*
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
/*
  Scripting MAVLink class, to generate MAVLink packets and decode incoming
  MAvLink data. It is designed for High Latency MAVLink connections, in most cases
  users should use a regular MAVLink serial port.
 */
#include "AP_Scripting_MAVLink.h"

#if HAL_HIGH_LATENCY2_ENABLED
ScriptingMAVLink::ScriptingMAVLink(bool use_mavlink1)
{
    // Instantiate a ScriptingMAVLink and set the MAVLink version
    // true to use MAVLink1, false to use MAVLink2

    useMAVLink1 = use_mavlink1;

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    if (_singleton != nullptr) {
        AP_HAL::panic("ScriptingMAVLink must be singleton");
    }
#endif
    _singleton = this;
};

bool ScriptingMAVLink::is_high_latency_enabled()
{
    // Return true if high latency mode is enabled
    GCS_MAVLINK *link = gcs().chan(0);
    return link->high_latency_link_enabled;
}

void ScriptingMAVLink::set_high_latency_enabled(bool hl_enabled)
{
    //enable or disable high latency mode
    GCS_MAVLINK *link = gcs().chan(0);
    mavlink_command_long_t pkt;
    pkt.param1 = hl_enabled;

    link->handle_control_high_latency(pkt);
}

uint16_t ScriptingMAVLink::create_high_latency2(uint8_t* msgbuf)
{
    // Create a High_Latency2 MAvLink packet and save into msgbuf
    // returns the length of the msgbuf

    AP_AHRS &ahrs = AP::ahrs();
    GCS_MAVLINK *link = gcs().chan(0);

    Location global_position_current;
    UNUSED_RESULT(ahrs.get_location(global_position_current));
#if !defined(HAL_BUILD_AP_PERIPH) || defined(HAL_PERIPH_ENABLE_BATTERY)
    const int8_t battery_remaining = link->battery_remaining_pct(AP_BATT_PRIMARY_INSTANCE);
#endif

    AP_Mission *mission = AP::mission();
    uint16_t current_waypoint = 0;
    if (mission != nullptr) {
        current_waypoint = mission->get_current_nav_index();
    }

    uint32_t present;
    uint32_t enabled;
    uint32_t health;
    gcs().get_sensor_status_flags(present, enabled, health);
    // Remap HL_FAILURE_FLAG from system status flags
    static const struct PACKED status_map_t {
        MAV_SYS_STATUS_SENSOR sensor;
        HL_FAILURE_FLAG failure_flag;
    } status_map[] {
        { MAV_SYS_STATUS_SENSOR_GPS, HL_FAILURE_FLAG_GPS },
        { MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE, HL_FAILURE_FLAG_DIFFERENTIAL_PRESSURE },
        { MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE, HL_FAILURE_FLAG_ABSOLUTE_PRESSURE },
        { MAV_SYS_STATUS_SENSOR_3D_ACCEL, HL_FAILURE_FLAG_3D_ACCEL },
        { MAV_SYS_STATUS_SENSOR_3D_GYRO, HL_FAILURE_FLAG_3D_GYRO },
        { MAV_SYS_STATUS_SENSOR_3D_MAG, HL_FAILURE_FLAG_3D_MAG },
        { MAV_SYS_STATUS_TERRAIN, HL_FAILURE_FLAG_TERRAIN },
        { MAV_SYS_STATUS_SENSOR_BATTERY, HL_FAILURE_FLAG_BATTERY },
        { MAV_SYS_STATUS_SENSOR_RC_RECEIVER, HL_FAILURE_FLAG_RC_RECEIVER },
        { MAV_SYS_STATUS_GEOFENCE, HL_FAILURE_FLAG_GEOFENCE },
        { MAV_SYS_STATUS_AHRS, HL_FAILURE_FLAG_ESTIMATOR },
    };

    uint16_t failure_flags = 0;
    for (auto &map_entry : status_map) {
        if ((health & map_entry.sensor) == 0) {
            failure_flags |= map_entry.failure_flag;
        }
    }

    mavlink_message_t msg;
    mavlink_high_latency2_t hl2_message {};
    
    hl2_message.timestamp = AP_HAL::millis();
    hl2_message.latitude = global_position_current.lat;
    hl2_message.longitude = global_position_current.lng;
    hl2_message.custom_mode = gcs().custom_mode();
    hl2_message.altitude = global_position_current.alt * 0.01f;
    hl2_message.target_altitude = link->high_latency_target_altitude();
    hl2_message.target_distance = link->high_latency_tgt_dist();
    hl2_message.wp_num = current_waypoint;
    hl2_message.failure_flags = failure_flags;
    hl2_message.type = gcs().frame_type();
    hl2_message.autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
    hl2_message.heading = (((uint16_t)ahrs.yaw_sensor / 100) % 360) / 2;
    hl2_message.target_heading = link->high_latency_tgt_heading();
    hl2_message.throttle = abs(link->vfr_hud_throttle());
    hl2_message.airspeed = MIN(link->vfr_hud_airspeed() * 5, UINT8_MAX);
    hl2_message.airspeed_sp = link->high_latency_tgt_airspeed();
    hl2_message.groundspeed = MIN(ahrs.groundspeed() * 5, UINT8_MAX);
    hl2_message.windspeed = link->high_latency_wind_speed();
    hl2_message.wind_heading = link->high_latency_wind_direction();
    hl2_message.eph = 0;
    hl2_message.epv = 0;
    hl2_message.temperature_air = link->high_latency_air_temperature();
    hl2_message.climb_rate = 0;
#if !defined(HAL_BUILD_AP_PERIPH) || defined(HAL_PERIPH_ENABLE_BATTERY)
        hl2_message.battery = battery_remaining; // [%] Battery level (-1 if field not provided).
#else
        hl2_message.battery = -1;
#endif
    hl2_message.custom0 = link->base_mode();
    hl2_message.custom1 = 0;
    hl2_message.custom2 = 0;

    uint16_t len;
    
    // borrow chan0 to encode the packet
    mavlink_status_t *chan0_status = mavlink_get_channel_status(MAVLINK_COMM_0);
    uint8_t saved_seq = chan0_status->current_tx_seq;
    uint8_t saved_flags = chan0_status->flags;
    if (useMAVLink1) {
        chan0_status->flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
    }
    else {
        chan0_status->flags &= ~(MAVLINK_STATUS_FLAG_OUT_MAVLINK1);
    }
    chan0_status->current_tx_seq = mavlink.seq;
    len = mavlink_msg_high_latency2_encode(mavlink_system.sysid,
                                            mavlink_system.compid,
                                            &msg, &hl2_message);

    mavlink.seq = chan0_status->current_tx_seq;
    chan0_status->current_tx_seq = saved_seq;
    chan0_status->flags = saved_flags;
    
    len = mavlink_msg_to_send_buffer(msgbuf, &msg);

    return len;

}

void ScriptingMAVLink::handle_rx_byte(uint8_t rx_byte) {
    // Handle an incoming byte and parse. If a valid packet (command_long
    // or command_int) is detected, send on to GCS_MAVLINK for execution
    mavlink_message_t msg;
    mavlink_status_t status;
    if (mavlink_frame_char_buffer(&mavlink.rxmsg, &mavlink.status,
                                    rx_byte,
                                    &msg, &status) == MAVLINK_FRAMING_OK) {
        GCS_MAVLINK &link = *gcs().chan(0);
        // only allow commands to be passed on
        if(msg.msgid == MAVLINK_MSG_ID_COMMAND_LONG || msg.msgid == MAVLINK_MSG_ID_COMMAND_INT) {
            link.packetReceived(status, msg);
        }
    }
}
#endif