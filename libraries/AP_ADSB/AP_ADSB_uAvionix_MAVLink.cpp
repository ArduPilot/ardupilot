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

#include "AP_ADSB_uAvionix_MAVLink.h"

#if HAL_ADSB_UAVIONIX_MAVLINK_ENABLED
#include <stdio.h>  // for sprintf
#include <limits.h>
#include <GCS_MAVLink/GCS.h>

#define ADSB_CHAN_TIMEOUT_MS            15000


extern const AP_HAL::HAL& hal;

// detect if an port is configured as MAVLink
bool AP_ADSB_uAvionix_MAVLink::detect()
{
    // this actually requires SerialProtocol_MAVLink or SerialProtocol_MAVLink2 but
    // we can't have a running system with that, so its safe to assume it's already defined
    return true;
}

void AP_ADSB_uAvionix_MAVLink::update()
{
    const uint32_t now = AP_HAL::millis();

    // send static configuration data to transceiver, every 5s
    if (_frontend.out_state.chan_last_ms > 0 && now - _frontend.out_state.chan_last_ms > ADSB_CHAN_TIMEOUT_MS) {
        // haven't gotten a heartbeat health status packet in a while, assume hardware failure
        _frontend.out_state.chan = -1;
        _frontend.out_state.chan_last_ms = 0; // if the time isn't reset we spam the message
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "ADSB: Transceiver heartbeat timed out");
    } else if (_frontend.out_state.chan >= 0 && !_frontend._my_loc.is_zero() && _frontend.out_state.chan < MAVLINK_COMM_NUM_BUFFERS) {
        const mavlink_channel_t chan = (mavlink_channel_t)(MAVLINK_COMM_0 + _frontend.out_state.chan);
        if (now - _frontend.out_state.last_config_ms >= 5000 && HAVE_PAYLOAD_SPACE(chan, UAVIONIX_ADSB_OUT_CFG)) {
            _frontend.out_state.last_config_ms = now;
            send_configure(chan);
        } // last_config_ms

        // send dynamic data to transceiver at 5Hz
        if (now - _frontend.out_state.last_report_ms >= 200 && HAVE_PAYLOAD_SPACE(chan, UAVIONIX_ADSB_OUT_DYNAMIC)) {
            _frontend.out_state.last_report_ms = now;
            send_dynamic_out(chan);
        } // last_report_ms
    } // chan_last_ms
}

void AP_ADSB_uAvionix_MAVLink::send_dynamic_out(const mavlink_channel_t chan) const
{
    const auto &_my_loc = _frontend._my_loc;
    const auto &gps = _my_loc;  // avoid churn

    const Vector3f &gps_velocity = gps.velocity();

    const int32_t latitude = _frontend._my_loc.lat;
    const int32_t longitude = _frontend._my_loc.lng;
    const int32_t altGNSS = _frontend._my_loc.alt * 10; // convert cm to mm
    const int16_t velVert = -1.0f * gps_velocity.z * 1E2; // convert m/s to cm/s
    const int16_t nsVog = gps_velocity.x * 1E2; // convert m/s to cm/s
    const int16_t ewVog = gps_velocity.y * 1E2; // convert m/s to cm/s
    const AP_GPS_FixType fixType = gps.status(); // this lines up perfectly with our enum
    const uint8_t emStatus = 0; // TODO: implement this ENUM. no emergency = 0
    const uint8_t numSats = gps.num_sats();
    const uint16_t squawk = _frontend.out_state.cfg.squawk_octal;

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
    if (_frontend.out_state.is_in_auto_mode) {
        state |= UAVIONIX_ADSB_OUT_DYNAMIC_STATE_AUTOPILOT_ENABLED;
    }
    if (!_frontend.out_state.is_flying) {
        state |= UAVIONIX_ADSB_OUT_DYNAMIC_STATE_ON_GROUND;
    }

    // TODO: confirm this sets utcTime correctly
    const uint64_t gps_time = gps.time_epoch_usec();
    const uint32_t utcTime = gps_time / 1000000ULL;

    int32_t altPres = INT_MAX;
    if (_my_loc.baro_is_healthy) {
        // Altitude difference between sea level pressure and current pressure. Result in millimeters
        altPres = _my_loc.baro_alt_press_diff_sea_level * 1E3; // convert m to mm;
    }



    mavlink_msg_uavionix_adsb_out_dynamic_send(
            chan,
            utcTime,
            latitude,
            longitude,
            altGNSS,
            uint8_t(fixType),
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
 * This function will override the MSB byte of the 24bit ICAO address. To ensure an invalid >24bit ICAO is never broadcasted,
 * this function is used to create the encoded version without ever writing to the actual ICAO number. It's created on-demand
 */
uint32_t AP_ADSB_uAvionix_MAVLink::encode_icao(const uint32_t icao_id) const
{
    // utilize the upper unused 8bits of the icao with special flags.
    // This encoding is required for uAvionix devices that break the MAVLink spec.

    // ensure the user assignable icao is 24 bits
    uint32_t encoded_icao = icao_id & 0x00FFFFFF;

    encoded_icao &= ~0x20000000;    // useGnssAltitude should always be FALSE
    encoded_icao |=  0x10000000;    // csidLogic       should always be TRUE

    //SIL/SDA are special fields that should be set to 0 with only expert user adjustment
    encoded_icao &= ~0x03000000;    // SDA should always be FALSE
    encoded_icao &= ~0x0C000000;    // SIL should always be FALSE

    return encoded_icao;
}

/*
 * To expand functionality in their HW, uAvionix has extended a few of the unused MAVLink bits to pack in more new features
 * This function will override the usually-null ending char of the callsign. It always encodes the last byte [8], even if
 * the callsign string is less than 9 chars and there are other zero-padded nulls.
 */
uint8_t AP_ADSB_uAvionix_MAVLink::get_encoded_callsign_null_char()
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

    if (_frontend.out_state.cfg.maxAircraftSpeed_knots <= 0) {
        // not set or unknown. no bits set
    } else if (_frontend.out_state.cfg.maxAircraftSpeed_knots <= 75) {
        encoded_null |= 0x01;
    } else if (_frontend.out_state.cfg.maxAircraftSpeed_knots <= 150) {
        encoded_null |= 0x02;
    } else if (_frontend.out_state.cfg.maxAircraftSpeed_knots <= 300) {
        encoded_null |= 0x03;
    } else if (_frontend.out_state.cfg.maxAircraftSpeed_knots <= 600) {
        encoded_null |= 0x04;
    } else if (_frontend.out_state.cfg.maxAircraftSpeed_knots <= 1200) {
        encoded_null |= 0x05;
    } else {
        encoded_null |= 0x06;
    }


    if (_frontend.out_state.cfg.rf_capable & ADSB_BITBASK_RF_CAPABILITIES_1090ES_IN) {
        encoded_null |= 0x10;
    }
    if (_frontend.out_state.cfg.rf_capable & ADSB_BITBASK_RF_CAPABILITIES_UAT_IN) {
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
    snprintf(_frontend.out_state.cfg.callsign, 5, "%04d", unsigned(_frontend.out_state.cfg.squawk_octal) & 0x1FFF);
    memset(&_frontend.out_state.cfg.callsign[4], 0, 5); // clear remaining 5 chars
    encoded_null |= 0x40;

    return encoded_null;
}

/*
 * populate and send MAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG
 */
void AP_ADSB_uAvionix_MAVLink::send_configure(const mavlink_channel_t chan)
{
    // MAVLink spec says the 9 byte callsign field is 8 byte string with 9th byte as null.
    // Here we temporarily set some flags in that null char to signify the callsign
    // may be a flightplanID instead
    int8_t callsign[sizeof(_frontend.out_state.cfg.callsign)];
    uint32_t icao;

    memcpy(callsign, _frontend.out_state.cfg.callsign, sizeof(_frontend.out_state.cfg.callsign));

    if (_frontend.out_state.cfg.was_set_externally) {
        // take values as-is
        icao = _frontend.out_state.cfg.ICAO_id;
    } else {
        callsign[MAVLINK_MSG_UAVIONIX_ADSB_OUT_CFG_FIELD_CALLSIGN_LEN-1] = (int8_t)get_encoded_callsign_null_char();
        icao = encode_icao((uint32_t)_frontend.out_state.cfg.ICAO_id);
    }

    mavlink_msg_uavionix_adsb_out_cfg_send(
            chan,
            icao,
            (const char*)callsign,
            (uint8_t)_frontend.out_state.cfg.emitterType,
            (uint8_t)_frontend.out_state.cfg.lengthWidth,
            (uint8_t)_frontend.out_state.cfg.gpsOffsetLat,
            (uint8_t)_frontend.out_state.cfg.gpsOffsetLon,
            _frontend.out_state.cfg.stall_speed_cm,
            (uint8_t)_frontend.out_state.cfg.rfSelect);
}

#endif // HAL_ADSB_UAVIONIX_MAVLINK_ENABLED
