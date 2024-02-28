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
  AP_Periph ADSB support. This is designed to talk to a Ping ADSB
  module over the UART
 */

#include <AP_HAL/AP_HAL_Boards.h>
#include "AP_Periph.h"

#ifdef HAL_PERIPH_ENABLE_ADSB

#include <AP_SerialManager/AP_SerialManager.h>
#include <dronecan_msgs.h>

extern const AP_HAL::HAL &hal;

/*
  init ADSB support
 */
void AP_Periph_FW::adsb_init(void)
{
    if (g.adsb_baudrate > 0) {
        auto *uart = hal.serial(g.adsb_port);
        if (uart == nullptr) {
            return;
        }
        uart->begin(AP_SerialManager::map_baudrate(g.adsb_baudrate), 256, 256);
    }
}

/*
  update ADSB subsystem
 */
void AP_Periph_FW::adsb_update(void)
{
    if (g.adsb_baudrate <= 0) {
        return;
    }

    auto *uart = hal.serial(g.adsb_port);
    if (uart == nullptr) {
        return;
    }

    // look for incoming MAVLink ADSB_VEHICLE packets
    const uint16_t nbytes = uart->available();
    for (uint16_t i=0; i<nbytes; i++) {
        const uint8_t c = (uint8_t)uart->read();

        // Try to get a new message
        if (mavlink_frame_char_buffer(&adsb.msg, &adsb.status, c, &adsb.msg, &adsb.status) == MAVLINK_FRAMING_OK) {
            if (adsb.msg.msgid == MAVLINK_MSG_ID_ADSB_VEHICLE) {
                // decode and send as UAVCAN TrafficReport
                static mavlink_adsb_vehicle_t msg;
                mavlink_msg_adsb_vehicle_decode(&adsb.msg, &msg);
                can_send_ADSB(msg);
            }
        }
    }

    /*
      some ADSB devices need a heartbeat to get the system ID
     */
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - adsb.last_heartbeat_ms >= 1000) {
        adsb.last_heartbeat_ms = now_ms;
        mavlink_heartbeat_t heartbeat {};
        mavlink_message_t msg;
        heartbeat.type = MAV_TYPE_GENERIC;
        heartbeat.autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
        auto len = mavlink_msg_heartbeat_encode_status(1,
                                                       MAV_COMP_ID_PERIPHERAL,
                                                       &adsb.status,
                                                       &msg, &heartbeat);

        uart->write((uint8_t*)&msg.magic, len);
    }
}

/*
  map an ADSB_VEHICLE MAVLink message to a UAVCAN TrafficReport message
 */
void AP_Periph_FW::can_send_ADSB(struct __mavlink_adsb_vehicle_t &msg)
{
    ardupilot_equipment_trafficmonitor_TrafficReport pkt {};
    pkt.timestamp.usec = 0;
    pkt.icao_address = msg.ICAO_address;
    pkt.tslc = msg.tslc;
    pkt.latitude_deg_1e7 = msg.lat;
    pkt.longitude_deg_1e7 = msg.lon;
    pkt.alt_m = msg.altitude * 1e-3;

    pkt.heading = radians(msg.heading * 1e-2);

    pkt.velocity[0] = cosf(pkt.heading) * msg.hor_velocity * 1e-2;
    pkt.velocity[1] = sinf(pkt.heading) * msg.hor_velocity * 1e-2;
    pkt.velocity[2] = -msg.ver_velocity * 1e-2;

    pkt.squawk = msg.squawk;
    memcpy(pkt.callsign, msg.callsign, MIN(sizeof(msg.callsign),sizeof(pkt.callsign)));
    if (msg.flags & 0x8000) {
        pkt.source = ARDUPILOT_EQUIPMENT_TRAFFICMONITOR_TRAFFICREPORT_SOURCE_ADSB_UAT;
    } else {
        pkt.source = ARDUPILOT_EQUIPMENT_TRAFFICMONITOR_TRAFFICREPORT_SOURCE_ADSB;
    }

    pkt.traffic_type = msg.emitter_type;

    if ((msg.flags & ADSB_FLAGS_VALID_ALTITUDE) != 0 && msg.altitude_type == 0) {
        pkt.alt_type = ARDUPILOT_EQUIPMENT_TRAFFICMONITOR_TRAFFICREPORT_ALT_TYPE_PRESSURE_AMSL;
    } else if ((msg.flags & ADSB_FLAGS_VALID_ALTITUDE) != 0 && msg.altitude_type == 1) {
        pkt.alt_type = ARDUPILOT_EQUIPMENT_TRAFFICMONITOR_TRAFFICREPORT_ALT_TYPE_WGS84;
    } else {
        pkt.alt_type = ARDUPILOT_EQUIPMENT_TRAFFICMONITOR_TRAFFICREPORT_ALT_TYPE_ALT_UNKNOWN;
    }

    pkt.lat_lon_valid = (msg.flags & ADSB_FLAGS_VALID_COORDS) != 0;
    pkt.heading_valid = (msg.flags & ADSB_FLAGS_VALID_HEADING) != 0;
    pkt.velocity_valid = (msg.flags & ADSB_FLAGS_VALID_VELOCITY) != 0;
    pkt.callsign_valid = (msg.flags & ADSB_FLAGS_VALID_CALLSIGN) != 0;
    pkt.ident_valid = (msg.flags & ADSB_FLAGS_VALID_SQUAWK) != 0;
    pkt.simulated_report = (msg.flags & ADSB_FLAGS_SIMULATED) != 0;

    // these flags are not in common.xml
    pkt.vertical_velocity_valid = (msg.flags & 0x0080) != 0;
    pkt.baro_valid = (msg.flags & 0x0100) != 0;

    uint8_t buffer[ARDUPILOT_EQUIPMENT_TRAFFICMONITOR_TRAFFICREPORT_MAX_SIZE] {};
    uint16_t total_size = ardupilot_equipment_trafficmonitor_TrafficReport_encode(&pkt, buffer, !periph.canfdout());

    canard_broadcast(ARDUPILOT_EQUIPMENT_TRAFFICMONITOR_TRAFFICREPORT_SIGNATURE,
                    ARDUPILOT_EQUIPMENT_TRAFFICMONITOR_TRAFFICREPORT_ID,
                    CANARD_TRANSFER_PRIORITY_LOWEST,
                    &buffer[0],
                    total_size);
}

#endif // HAL_PERIPH_ENABLE_ADSB
