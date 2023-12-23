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

#include "SIM_config.h"

#if HAL_SIM_ADSB_ENABLED

#include "SIM_ADSB.h"

#include "SITL.h"

#include <stdio.h>

#include "SIM_Aircraft.h"
#include <AP_HAL_SITL/SITL_State.h>
#include <AP_AHRS/AP_AHRS.h>

namespace SITL {

/*
  update a simulated vehicle
 */
void ADSB_Vehicle::update(const class Aircraft &aircraft, float delta_t)
{
    const SIM *_sitl = AP::sitl();
    if (_sitl == nullptr) {
        return;
    }

    const Location &origin { aircraft.get_origin() };

    if (!initialised) {
        // spawn another aircraft
        initialised = true;
        ICAO_address = (uint32_t)(rand() % 10000);
        snprintf(callsign, sizeof(callsign), "SIM%u", ICAO_address);
        Location aircraft_location = aircraft.get_location();
        const Vector2f aircraft_offset_ne = aircraft_location.get_distance_NE(origin);
        position.x = aircraft_offset_ne[1];
        position.y = aircraft_offset_ne[0];
        position.x += Aircraft::rand_normal(0, _sitl->adsb_radius_m);
        position.y += Aircraft::rand_normal(0, _sitl->adsb_radius_m);
        position.z = -fabsf(_sitl->adsb_altitude_m);

        double vel_min = 5, vel_max = 20;
        if (position.length() > 500) {
            vel_min *= 3;
            vel_max *= 3;
        } else if (position.length() > 10000) {
            vel_min *= 10;
            vel_max *= 10;
        }
        type = (ADSB_EMITTER_TYPE)(rand() % (ADSB_EMITTER_TYPE_POINT_OBSTACLE + 1));
        // don't allow surface emitters to move
        if (type == ADSB_EMITTER_TYPE_POINT_OBSTACLE) {
            stationary_object_created_ms = AP_HAL::millis64();
            velocity_ef.zero();
        } else {
            stationary_object_created_ms = 0;
            velocity_ef.x = Aircraft::rand_normal(vel_min, vel_max);
            velocity_ef.y = Aircraft::rand_normal(vel_min, vel_max);
            if (type < ADSB_EMITTER_TYPE_EMERGENCY_SURFACE) {
                velocity_ef.z = Aircraft::rand_normal(-3, 3);
            }
        }
    } else if (stationary_object_created_ms > 0 && AP_HAL::millis64() - stationary_object_created_ms > AP_MSEC_PER_HOUR) {
        // regenerate stationary objects so we don't randomly fill up the screen with them over time
        initialised = false;
    }

    position += velocity_ef * delta_t;
    if (position.z > 0) {
        // it has crashed! reset
        initialised = false;
    }

    Location ret = origin;
    ret.offset(position.x, position.y);

    location = ret;
}

const Location &ADSB_Vehicle::get_location() const
{
    return location;
}

/*
  update the ADSB peripheral state
*/
void ADSB::update_simulated_vehicles(const class Aircraft &aircraft)
{
    if (_sitl == nullptr) {
        _sitl = AP::sitl();
        return;
    } else if (_sitl->adsb_plane_count <= 0) {
        return;
    } else if (_sitl->adsb_plane_count >= num_vehicles_MAX) {
        _sitl->adsb_plane_count.set_and_save(0);
        num_vehicles = 0;
        return;
    } else if (num_vehicles != _sitl->adsb_plane_count) {
        num_vehicles = _sitl->adsb_plane_count;
        for (uint8_t i=0; i<num_vehicles_MAX; i++) {
            vehicles[i].initialised = false;
        }
    }

    // calculate delta time in seconds
    uint32_t now_us = AP_HAL::micros();

    float delta_t = (now_us - last_update_us) * 1.0e-6f;
    last_update_us = now_us;

    // prune any aircraft which get too far away from our simulated vehicle:
    const Location &aircraft_loc = aircraft.get_location();

    for (uint8_t i=0; i<num_vehicles; i++) {
        auto &vehicle = vehicles[i];
        vehicle.update(aircraft, delta_t);

        // re-init when exceeding radius range
        if (aircraft_loc.get_distance(vehicle.get_location()) > _sitl->adsb_radius_m) {
            vehicle.initialised = false;
        }
    }
}

void ADSB::update(const class Aircraft &aircraft)
{
    update_simulated_vehicles(aircraft);

    // see if we should do a report.
    if ((_sitl->adsb_types & (1U << (uint8_t)SIM::ADSBType::Shortcut)) == 0) {
        // some other simulated device is in use (e.g. MXS)
        return;
    }

    // bakwards compatability; the parameters specify ADSB simulation,
    // but we are not configured to use a simulated ADSB driver.
    // Pretend to be a uAvionix mavlink device:
    send_report(aircraft);
}

/*
  send a report to the vehicle control code over MAVLink
*/
void ADSB::send_report(const class Aircraft &aircraft)
{
    if (AP_HAL::millis() < 10000) {
        // simulated aircraft don't appear until 10s after startup. This avoids a windows
        // threading issue with non-blocking sockets and the initial wait on SERIAL0
        return;
    }

    // check for incoming MAVLink messages
    uint8_t buf[100];
    ssize_t ret;

    while ((ret=read_from_autopilot((char*)buf, sizeof(buf))) > 0) {
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

        len = mavlink_msg_heartbeat_encode_status(vehicle_system_id,
                                                  vehicle_component_id,
                                                  &mavlink.status,
                                                  &msg, &heartbeat);

        write_to_autopilot((char*)&msg.magic, len);

        last_heartbeat_ms = now;
    }


    /*
      send a ADSB_VEHICLE messages
     */
    uint32_t now_us = AP_HAL::micros();
    if (now_us - last_report_us >= reporting_period_ms*1000UL) {
        for (uint8_t i=0; i<num_vehicles; i++) {
            const ADSB_Vehicle &vehicle = vehicles[i];
            if (!vehicle.initialised) {
                continue;
            }

            const Location &loc { vehicle.get_location() };

            mavlink_adsb_vehicle_t adsb_vehicle {};
            last_report_us = now_us;

            adsb_vehicle.ICAO_address = vehicle.ICAO_address;
            adsb_vehicle.lat = loc.lat;
            adsb_vehicle.lon = loc.lng;
            adsb_vehicle.altitude_type = ADSB_ALTITUDE_TYPE_PRESSURE_QNH;
            adsb_vehicle.altitude = -vehicle.position.z * 1000;
            adsb_vehicle.heading = wrap_360_cd(100*degrees(atan2f(vehicle.velocity_ef.y, vehicle.velocity_ef.x)));
            adsb_vehicle.hor_velocity = norm(vehicle.velocity_ef.x, vehicle.velocity_ef.y) * 100;
            adsb_vehicle.ver_velocity = -vehicle.velocity_ef.z * 100;
            memcpy(adsb_vehicle.callsign, vehicle.callsign, sizeof(adsb_vehicle.callsign));
            adsb_vehicle.emitter_type = vehicle.type;
            adsb_vehicle.tslc = 1;
            adsb_vehicle.flags =
                ADSB_FLAGS_VALID_COORDS |
                ADSB_FLAGS_VALID_ALTITUDE |
                ADSB_FLAGS_VALID_HEADING |
                ADSB_FLAGS_VALID_VELOCITY |
                ADSB_FLAGS_VALID_CALLSIGN |
                ADSB_FLAGS_VALID_SQUAWK |
                ADSB_FLAGS_SIMULATED |
                ADSB_FLAGS_VERTICAL_VELOCITY_VALID |
                ADSB_FLAGS_BARO_VALID;
            // all flags set except ADSB_FLAGS_SOURCE_UAT

            adsb_vehicle.squawk = 1200;

            len = mavlink_msg_adsb_vehicle_encode_status(vehicle_system_id,
                                                  MAV_COMP_ID_ADSB,
                                                  &mavlink.status,
                                                  &msg, &adsb_vehicle);

            uint8_t msgbuf[len];
            len = mavlink_msg_to_send_buffer(msgbuf, &msg);
            if (len > 0) {
                write_to_autopilot((char*)msgbuf, len);
            }
        }
    }
    
    // ADSB_transceiever is enabled, send the status report.
    if (_sitl->adsb_tx && now - last_tx_report_ms > 1000) {
        last_tx_report_ms = now;

        const mavlink_uavionix_adsb_transceiver_health_report_t health_report = {UAVIONIX_ADSB_RF_HEALTH_OK};
        len = mavlink_msg_uavionix_adsb_transceiver_health_report_encode_status(vehicle_system_id,
                                                                                MAV_COMP_ID_ADSB,
                                                                                &mavlink.status,
                                                                                &msg, &health_report);

        uint8_t msgbuf[len];
        len = mavlink_msg_to_send_buffer(msgbuf, &msg);
        if (len > 0) {
            write_to_autopilot((char*)msgbuf, len);
            ::printf("ADSBsim send tx health packet\n");
        }
    }

}

} // namespace SITL

#endif // HAL_SIM_ADSB_ENABLED
