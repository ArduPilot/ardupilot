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

#include <AP_HAL/AP_HAL.h>
#include "AP_ADSB_uAvionix.h"

#include <stdio.h>  // for sprintf
#include <limits.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_AHRS/AP_AHRS.h>

extern const AP_HAL::HAL& hal;

#define ADSB_CHAN_TIMEOUT_MS            15000

// constructor
AP_ADSB_uAvionix::AP_ADSB_uAvionix(AP_ADSB &adsb) :
        AP_ADSB_Backend(adsb)
{
}

void AP_ADSB_uAvionix::update()
{
    const uint32_t now_ms = AP_HAL::millis();

    // send static configuration data to transceiver, every 5s
    if (_chan_last_ms > 0 && now_ms - _chan_last_ms > ADSB_CHAN_TIMEOUT_MS) {
        // haven't gotten a heartbeat health status packet in a while, assume hardware failure
        // TODO: reset out_state.chan
        _chan = (mavlink_channel_t)-1;
        gcs().send_text(MAV_SEVERITY_ERROR, "ADSB: Transceiver heartbeat timed out");
    } else if (_chan >= 0 && _chan < MAVLINK_COMM_NUM_BUFFERS) {
        if (now_ms - frontend.out_state.last_config_ms >= 5000 && HAVE_PAYLOAD_SPACE(_chan, UAVIONIX_ADSB_OUT_CFG)) {
            frontend.out_state.last_config_ms = now_ms;
            send_configure();
        } // last_config_ms

        // send dynamic data to transceiver at 5Hz
        if (now_ms - frontend.out_state.last_report_ms >= 200 && HAVE_PAYLOAD_SPACE(_chan, UAVIONIX_ADSB_OUT_DYNAMIC)) {
            frontend.out_state.last_report_ms = now_ms;
            send_dynamic_out();
        } // last_report_ms
    } // chan_last_ms
}


void AP_ADSB_uAvionix::handle_msg(const mavlink_channel_t chan, const mavlink_message_t &msg) {
    switch (msg.msgid) {
    case MAVLINK_MSG_ID_UAVIONIX_ADSB_TRANSCEIVER_HEALTH_REPORT:
        {
            _chan = chan;
            mavlink_uavionix_adsb_transceiver_health_report_t packet {};
            mavlink_msg_uavionix_adsb_transceiver_health_report_decode(&msg, &packet);
            handle_transceiver_report(packet);
        }
        break;

    case MAVLINK_MSG_ID_UAVIONIX_ADSB_OUT_CFG:
        {
            mavlink_uavionix_adsb_out_cfg_t packet {};
            mavlink_msg_uavionix_adsb_out_cfg_decode(&msg, &packet);
            handle_out_cfg(packet);
        }
        break;

    default:
        break;
    }
}

/*
 * this is a message from the transceiver reporting it's health. Using this packet
 * we determine which channel is on so we don't have to send out_state to all channels
 */

void AP_ADSB_uAvionix::handle_transceiver_report(const mavlink_uavionix_adsb_transceiver_health_report_t &packet)
{
    _chan_last_ms = AP_HAL::millis();
    frontend.out_state.status = (UAVIONIX_ADSB_RF_HEALTH)packet.rfHealth;
}



/*
 * handle incoming packet UAVIONIX_ADSB_OUT_CFG.
 * This allows a GCS to send cfg info through the autopilot to the ADSB hardware.
 * This is done indirectly by reading and storing the packet and then another mechanism sends it out periodically
 */
void AP_ADSB_uAvionix::handle_out_cfg(const mavlink_uavionix_adsb_out_cfg_t &packet)
{
    frontend.out_state.cfg.was_set_externally = true;

    frontend.out_state.cfg.ICAO_id = packet.ICAO;
    frontend.out_state.cfg.ICAO_id_param = frontend.out_state.cfg.ICAO_id_param_prev = packet.ICAO & 0x00FFFFFFFF;

    // May contain a non-null value at the end so accept it as-is with memcpy instead of strcpy
    memcpy(frontend.out_state.cfg.callsign, packet.callsign, sizeof(frontend.out_state.cfg.callsign));

    frontend.out_state.cfg.emitterType = packet.emitterType;
    frontend.out_state.cfg.lengthWidth = packet.aircraftSize;
    frontend.out_state.cfg.gpsLatOffset = packet.gpsOffsetLat;
    frontend.out_state.cfg.gpsLonOffset = packet.gpsOffsetLon;
    frontend.out_state.cfg.rfSelect = packet.rfSelect;
    frontend.out_state.cfg.stall_speed_cm = packet.stallSpeed;

    // guard against string with non-null end char
    const char c = frontend.out_state.cfg.callsign[MAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_FIELD_CALLSIGN_LEN-1];
    frontend.out_state.cfg.callsign[MAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_FIELD_CALLSIGN_LEN-1] = 0;
    gcs().send_text(MAV_SEVERITY_INFO, "ADSB: Using ICAO_id %d and Callsign %s", (int)frontend.out_state.cfg.ICAO_id, frontend.out_state.cfg.callsign);
    frontend.out_state.cfg.callsign[MAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_FIELD_CALLSIGN_LEN-1] = c;

    // send now
    frontend.out_state.last_config_ms = 0;
}




/*
 * populate and send MAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG
 */
void AP_ADSB_uAvionix::send_configure()
{
    // MAVLink spec says the 9 byte callsign field is 8 byte string with 9th byte as null.
    // Here we temporarily set some flags in that null char to signify the callsign
    // may be a flightplanID instead
    int8_t callsign[sizeof(frontend.out_state.cfg.callsign)];
    uint32_t icao;

    memcpy(callsign, frontend.out_state.cfg.callsign, sizeof(frontend.out_state.cfg.callsign));

    if (frontend.out_state.cfg.was_set_externally) {
        // take values as-is
        icao = frontend.out_state.cfg.ICAO_id;
    } else {
        callsign[MAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_FIELD_CALLSIGN_LEN-1] = (int8_t)get_encoded_callsign_null_char();
        icao = get_encoded_icao();
    }

    mavlink_msg_uavionix_adsb_out_cfg_send(
            _chan,
            icao,
            (const char*)callsign,
            (uint8_t)frontend.out_state.cfg.emitterType,
            (uint8_t)frontend.out_state.cfg.lengthWidth,
            (uint8_t)frontend.out_state.cfg.gpsLatOffset,
            (uint8_t)frontend.out_state.cfg.gpsLonOffset,
            frontend.out_state.cfg.stall_speed_cm,
            (uint8_t)frontend.out_state.cfg.rfSelect);
}



void AP_ADSB_uAvionix::send_dynamic_out()
{
    const AP_GPS &gps = AP::gps();
    const Vector3f &gps_velocity = gps.velocity();

    const int32_t latitude = frontend._my_loc.lat;
    const int32_t longitude = frontend._my_loc.lng;
    const int32_t altGNSS = frontend._my_loc.alt * 10; // convert cm to mm
    const int16_t velVert = gps_velocity.z * 1E2; // convert m/s to cm/s
    const int16_t nsVog = gps_velocity.x * 1E2; // convert m/s to cm/s
    const int16_t ewVog = gps_velocity.y * 1E2; // convert m/s to cm/s
    const uint8_t fixType = gps.status(); // this lines up perfectly with our enum
    const uint8_t emStatus = 0; // TODO: implement this ENUM. no emergency = 0
    const uint8_t numSats = gps.num_sats();
    const uint16_t squawk = frontend.out_state.cfg.squawk_octal;

    uint32_t accHoriz = UINT_MAX;
    float accHoriz_f;
    if (gps.horizontal_accuracy(accHoriz_f)) {
        accHoriz = accHoriz_f * 1E3; // convert m to mm
    }

    uint16_t accVert = USHRT_MAX;
    float accVert_f;
    if (gps.vertical_accuracy(accVert_f)) {
        accVert = accVert_f * 1E2; // convert m to cm
    }

    uint16_t accVel = USHRT_MAX;
    float accVel_f;
    if (gps.speed_accuracy(accVel_f)) {
        accVel = accVel_f * 1E3; // convert m/s to mm/s
    }

    uint16_t state = 0;
    if (frontend.out_state._is_in_auto_mode) {
        state |= UAVIONIX_ADSB_OUT_DYNAMIC_STATE_AUTOPILOT_ENABLED;
    }
    if (!frontend.out_state.is_flying) {
        state |= UAVIONIX_ADSB_OUT_DYNAMIC_STATE_ON_GROUND;
    }

    // TODO: confirm this sets utcTime correctly
    const uint64_t gps_time = gps.time_epoch_usec();
    const uint32_t utcTime = gps_time / 1000000ULL;

    const AP_Baro &baro = AP::baro();
    int32_t altPres = INT_MAX;
    if (baro.healthy()) {
        // Altitude difference between sea level pressure and current pressure. Result in millimeters
        altPres = baro.get_altitude_difference(SSL_AIR_PRESSURE, baro.get_pressure()) * 1E3; // convert m to mm;
    }



    mavlink_msg_uavionix_adsb_out_dynamic_send(
            _chan,
            utcTime,
            latitude,
            longitude,
            altGNSS,
            fixType,
            numSats,
            altPres,
            accHoriz,
            accVert,
            accVel,
            velVert,
            nsVog,
            ewVog,
            emStatus,
            state,
            squawk);
}



/*
 * To expand functionality in their HW, uAvionix has extended a few of the unused MAVLink bits to pack in more new features
 * This function will override the usually-null ending char of the callsign. It always encodes the last byte [8], even if
 * the callsign string is less than 9 chars and there are other zero-padded nulls.
 */
uint8_t AP_ADSB_uAvionix::get_encoded_callsign_null_char()
{
//  Encoding of the 8bit null char
//  (LSB) - knots
//  bit.1 - knots
//  bit.2 - knots
//  bit.3 - (unused)
//  bit.4 - flag - ADSB_BITBASK_RF_CAPABILITIES_1090ES_IN
//  bit.5 - flag - ADSB_BITBASK_RF_CAPABILITIES_UAT_IN
//  bit.6 - flag - 0 = callsign is treated as callsign, 1 = callsign is treated as flightPlanID/Squawk
//  (MSB) - (unused)

    uint8_t encoded_null = 0;

    if (frontend.out_state.cfg.maxAircraftSpeed_knots <= 0) {
        // not set or unknown. no bits set
    } else if (frontend.out_state.cfg.maxAircraftSpeed_knots <= 75) {
        encoded_null |= 0x01;
    } else if (frontend.out_state.cfg.maxAircraftSpeed_knots <= 150) {
        encoded_null |= 0x02;
    } else if (frontend.out_state.cfg.maxAircraftSpeed_knots <= 300) {
        encoded_null |= 0x03;
    } else if (frontend.out_state.cfg.maxAircraftSpeed_knots <= 600) {
        encoded_null |= 0x04;
    } else if (frontend.out_state.cfg.maxAircraftSpeed_knots <= 1200) {
        encoded_null |= 0x05;
    } else {
        encoded_null |= 0x06;
    }


    if (frontend.out_state.cfg.rf_capable & ADSB_BITBASK_RF_CAPABILITIES_1090ES_IN) {
        encoded_null |= 0x10;
    }
    if (frontend.out_state.cfg.rf_capable & ADSB_BITBASK_RF_CAPABILITIES_UAT_IN) {
        encoded_null |= 0x20;
    }


    /*
    If the user has an 8 digit flightPlanID assigned from a filed flight plan, this should be assigned to FlightPlanID, (assigned by remote app)
    else if the user has an assigned squawk code from ATC this should be converted from 4 digit octal to 4 character alpha string and assigned to FlightPlanID,
    else if a tail number is known it should be set to the tail number of the aircraft, (assigned by remote app)
    else it should be left blank (all 0's)
     */

    // using the above logic, we must always assign the squawk. once we get configured
    // externally then get_encoded_callsign_null_char() stops getting called
    snprintf(frontend.out_state.cfg.callsign, 5, "%04d", unsigned(frontend.out_state.cfg.squawk_octal) & 0x1FFF);
    memset(&frontend.out_state.cfg.callsign[4], 0, 5); // clear remaining 5 chars
    encoded_null |= 0x40;

    return encoded_null;
}

/*
 * To expand functionality in their HW, uAvionix has extended a few of the unused MAVLink bits to pack in more new features
 * This function will override the MSB byte of the 24bit ICAO address. To ensure an invalid >24bit ICAO is never broadcasted,
 * this function is used to create the encoded verison without ever writing to the actual ICAO number. It's created on-demand
 */
uint32_t AP_ADSB_uAvionix::get_encoded_icao(void)
{
    // utilize the upper unused 8bits of the icao with special flags.
    // This encoding is required for uAvionix devices that break the MAVLink spec.

    // ensure the user assignable icao is 24 bits
    uint32_t encoded_icao = (uint32_t)frontend.out_state.cfg.ICAO_id & 0x00FFFFFF;

    encoded_icao &= ~0x20000000;    // useGnssAltitude should always be FALSE
    encoded_icao |=  0x10000000;    // csidLogic       should always be TRUE

    //SIL/SDA are special fields that should be set to 0 with only expert user adjustment
    encoded_icao &= ~0x03000000;    // SDA should always be FALSE
    encoded_icao &= ~0x0C000000;    // SIL should always be FALSE

    return encoded_icao;
}


