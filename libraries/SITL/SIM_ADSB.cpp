/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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
  ADSB simulator class for MAVLink ADSB peripheral
*/

#include "SIM_ADSB.h"

#include <stdio.h>

#include "SIM_Aircraft.h"

namespace SITL {

ADSB::ADSB(const struct sitl_fdm &_fdm, const char *_home_str) :
    fdm(_fdm)
{
    float yaw_degrees;
    Aircraft::parse_home(_home_str, home, yaw_degrees);
}


/*
  update a simulated vehicle
 */
void ADSB_Vehicle::update(float delta_t)
{
    if (!initialised) {
        initialised = true;
        ICAO_address = (uint32_t)(rand() % 10000);
        snprintf(callsign, sizeof(callsign), "SIM%u", ICAO_address);
        position.x = Aircraft::rand_normal(0, 1000);
        position.y = Aircraft::rand_normal(0, 1000);
        position.z = -fabsf(Aircraft::rand_normal(3000, 1000));
        velocity_ef.x = Aircraft::rand_normal(5, 20);
        velocity_ef.y = Aircraft::rand_normal(5, 20);
        velocity_ef.z = Aircraft::rand_normal(0, 3);
    }

    position += velocity_ef * delta_t;
    if (position.z > 0) {
        // it has crashed! reset
        initialised = false;
    }
}

/*
  update the ADSB peripheral state
*/
void ADSB::update(void)
{
    // calculate delta time in seconds
    uint32_t now_us = AP_HAL::micros();

    float delta_t = (now_us - last_update_us) * 1.0e-6f;
    last_update_us = now_us;

    for (uint8_t i=0; i<num_vehicles; i++) {
        vehicles[i].update(delta_t);
    }
    
    // see if we should do a report
    send_report();
}

/*
  send a report to the vehicle control code over MAVLink
*/
void ADSB::send_report(void)
{
    if (!mavlink.connected && mav_socket.connect(target_address, target_port)) {
        ::printf("ADSB connected to %s:%u\n", target_address, (unsigned)target_port);
        mavlink.connected = true;
    }
    if (!mavlink.connected) {
        return;
    }

    // check for incoming MAVLink messages
    uint8_t buf[100];
    ssize_t ret;

    while ((ret=mav_socket.recv(buf, sizeof(buf), 0)) > 0) {
        for (uint8_t i=0; i<ret; i++) {
            mavlink_message_t msg;
            mavlink_status_t status;
            if (mavlink_frame_char_buffer(&mavlink.rxmsg, &mavlink.status,
                                          buf[i],
                                          &msg, &status) == MAVLINK_FRAMING_OK) {
                switch (msg.msgid) {
                case MAVLINK_MSG_ID_HEARTBEAT: {
                    if (!seen_heartbeat) {
                        seen_heartbeat = true;
                        vehicle_component_id = msg.compid;
                        vehicle_system_id = msg.sysid;
                        ::printf("ADSB using srcSystem %u\n", (unsigned)vehicle_system_id);
                    }
                    break;
                }
                }
            }
        }
    }

    if (!seen_heartbeat) {
        return;
    }

    uint32_t now = AP_HAL::millis();
    mavlink_message_t msg;
    uint16_t len;

    if (now - last_heartbeat_ms >= 1000) {
        mavlink_heartbeat_t heartbeat;
        heartbeat.type = MAV_TYPE_ADSB;
        heartbeat.autopilot = MAV_AUTOPILOT_ARDUPILOTMEGA;
        heartbeat.base_mode = 0;
        heartbeat.system_status = 0;
        heartbeat.mavlink_version = 0;
        heartbeat.custom_mode = 0;

        /*
          save and restore sequence number for chan0, as it is used by
          generated encode functions
         */
        mavlink_status_t *chan0_status = mavlink_get_channel_status(MAVLINK_COMM_0);
        uint8_t saved_seq = chan0_status->current_tx_seq;
        chan0_status->current_tx_seq = mavlink.seq;
        len = mavlink_msg_heartbeat_encode(vehicle_system_id,
                                           vehicle_component_id,
                                           &msg, &heartbeat);
        chan0_status->current_tx_seq = saved_seq;

        mav_socket.send(&msg.magic, len);

        last_heartbeat_ms = now;
    }


    /*
      send a ADSB_VEHICLE messages
     */
    uint32_t now_us = AP_HAL::micros();
    if (now_us - last_report_us > reporting_period_ms*1000UL) {
        for (uint8_t i=0; i<num_vehicles; i++) {
            ADSB_Vehicle &vehicle = vehicles[i];
            Location loc = home;

            location_offset(loc, vehicle.position.x, vehicle.position.y);

            // re-init when over 50km from home
            if (get_distance(home, loc) > 1000) {
                vehicle.initialised = false;
            }
            
            mavlink_adsb_vehicle_t adsb_vehicle {};
            last_report_us = now_us;

            adsb_vehicle.ICAO_address = vehicle.ICAO_address;
            adsb_vehicle.lat = loc.lat;
            adsb_vehicle.lon = loc.lng;
            adsb_vehicle.altitude_type = ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
            adsb_vehicle.altitude = -vehicle.position.z * 1000;
            adsb_vehicle.heading = wrap_360_cd(100*degrees(atan2f(vehicle.velocity_ef.y, vehicle.velocity_ef.x))) / 100;
            adsb_vehicle.hor_velocity = pythagorous2(vehicle.velocity_ef.x, vehicle.velocity_ef.y) * 100;
            adsb_vehicle.ver_velocity = -vehicle.velocity_ef.z * 100;
            memcpy(adsb_vehicle.callsign, vehicle.callsign, sizeof(adsb_vehicle.callsign));
            adsb_vehicle.emitter_type = ADSB_EMITTER_TYPE_LARGE;
            adsb_vehicle.tslc = 1;
            adsb_vehicle.flags =
                ADSB_FLAGS_VALID_COORDS |
                ADSB_FLAGS_VALID_ALTITUDE |
                ADSB_FLAGS_VALID_HEADING |
                ADSB_FLAGS_VALID_VELOCITY |
                ADSB_FLAGS_VALID_CALLSIGN |
                ADSB_FLAGS_SIMULATED;
            adsb_vehicle.squawk = 0; // NOTE: ADSB_FLAGS_VALID_SQUAWK bit is not set

            mavlink_status_t *chan0_status = mavlink_get_channel_status(MAVLINK_COMM_0);
            uint8_t saved_seq = chan0_status->current_tx_seq;
            chan0_status->current_tx_seq = mavlink.seq;
            len = mavlink_msg_adsb_vehicle_encode(vehicle_system_id,
                                                  MAV_COMP_ID_ADSB,
                                                  &msg, &adsb_vehicle);
            chan0_status->current_tx_seq = saved_seq;
            
            mav_socket.send(&msg.magic, len);
        }
    }
    
}

} // namespace SITL
