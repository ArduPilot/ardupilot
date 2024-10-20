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

//
//  Trimble GPS driver for ArduPilot.
//  Code by Michael Oborne
//
//  Usage in SITL with hardware for debugging: 
//    sim_vehicle.py -v Plane -A "--serial3=uart:/dev/ttyUSB0" --console --map -DG
//    param set GPS1_TYPE 11 // GSOF
//    param set SERIAL3_PROTOCOL 5 // GPS
//
//  Pure SITL usage
//     param set SIM_GPS_TYPE 11 // GSOF
#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include "AP_GPS.h"
#include "AP_GPS_GSOF.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <GCS_MAVLink/GCS.h>

#if AP_GPS_GSOF_ENABLED

extern const AP_HAL::HAL& hal;

#define gsof_DEBUGGING 0

#if gsof_DEBUGGING
# define Debug(fmt, args ...)                  \
do {                                            \
    hal.console->printf("%s:%d: " fmt "\n",     \
                        __FUNCTION__, __LINE__, \
                        ## args);               \
    hal.scheduler->delay(1);                    \
} while(0)
#else
# define Debug(fmt, args ...)
#endif

AP_GPS_GSOF::AP_GPS_GSOF(AP_GPS &_gps,
                         AP_GPS::Params &_params,
                         AP_GPS::GPS_State &_state,
                         AP_HAL::UARTDriver *_port) :
    AP_GPS_Backend(_gps, _params, _state, _port)
{
    // https://receiverhelp.trimble.com/oem-gnss/index.html#GSOFmessages_Overview.html?TocPath=Output%2520Messages%257CGSOF%2520Messages%257COverview%257C_____0
    static_assert(ARRAY_SIZE(gsofmsgreq) <= 10, "The maximum number of outputs allowed with GSOF is 10.");

    constexpr uint8_t default_com_port = static_cast<uint8_t>(HW_Port::COM2);
    params.com_port.set_default(default_com_port);
    const auto com_port = params.com_port;
    if (!validate_com_port(com_port)) {
        // The user parameter for COM port is not a valid GSOF port
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "GSOF instance %d has invalid COM port setting of %d", state.instance, (unsigned)com_port);
        return;
    }
    requestBaud(static_cast<HW_Port>(unsigned(com_port)));

    const uint32_t now = AP_HAL::millis();
    gsofmsg_time = now + 110;
}

// Process all bytes available from the stream
//
bool
AP_GPS_GSOF::read(void)
{
    const uint32_t now = AP_HAL::millis();

    if (gsofmsgreq_index < (sizeof(gsofmsgreq))) {
        const auto com_port = params.com_port.get();
        if (!validate_com_port(com_port)) {
            // The user parameter for COM port is not a valid GSOF port
            return false;
        }
        if (now > gsofmsg_time) {
            requestGSOF(gsofmsgreq[gsofmsgreq_index], static_cast<HW_Port>(com_port), Output_Rate::FREQ_10_HZ);
            gsofmsg_time = now + 110;
            gsofmsgreq_index++;
        }
    }

    bool ret = false;
    while (port->available() > 0) {
        const uint8_t temp = port->read();
#if AP_GPS_DEBUG_LOGGING_ENABLED
        log_data(&temp, 1);
#endif
        const int n_gsof_received = parse(temp, ARRAY_SIZE(gsofmsgreq));
        if(n_gsof_received == UNEXPECTED_NUM_GSOF_PACKETS) {
            state.status = AP_GPS::NO_FIX;
            continue;
        }
        const bool got_expected_packets = n_gsof_received == ARRAY_SIZE(gsofmsgreq);
        ret |= got_expected_packets;
    }
    if (ret) {
        pack_state_data();
    }

    return ret;
}

void
AP_GPS_GSOF::requestBaud(const HW_Port portindex)
{
    uint8_t buffer[19] = {0x02,0x00,0x64,0x0d,0x00,0x00,0x00, // application file record
                          0x03, 0x00, 0x01, 0x00, // file control information block
                          0x02, 0x04, static_cast<uint8_t>(portindex), 0x07, 0x00,0x00, // serial port baud format
                          0x00,0x03
                         }; // checksum

    buffer[4] = packetcount++;

    uint8_t checksum = 0;
    for (uint8_t a = 1; a < (sizeof(buffer) - 1); a++) {
        checksum += buffer[a];
    }

    buffer[17] = checksum;

    port->write((const uint8_t*)buffer, sizeof(buffer));
}

void
AP_GPS_GSOF::requestGSOF(const uint8_t messageType, const HW_Port portIndex, const Output_Rate rateHz)
{
    uint8_t buffer[21] = {0x02,0x00,0x64,0x0f,0x00,0x00,0x00, // application file record
                          0x03,0x00,0x01,0x00, // file control information block
                          0x07,0x06,0x0a,static_cast<uint8_t>(portIndex),static_cast<uint8_t>(rateHz),0x00,messageType,0x00, // output message record
                          0x00,0x03
                         }; // checksum

    buffer[4] = packetcount++;

    uint8_t checksum = 0;
    for (uint8_t a = 1; a < (sizeof(buffer) - 1); a++) {
        checksum += buffer[a];
    }

    buffer[19] = checksum;

    port->write((const uint8_t*)buffer, sizeof(buffer));
}

bool
AP_GPS_GSOF::validate_com_port(const uint8_t com_port) const
{
    switch(com_port) {
        case static_cast<uint8_t>(HW_Port::COM1):
        case static_cast<uint8_t>(HW_Port::COM2):
            return true;
        default:
            return false;
    }
}

void
AP_GPS_GSOF::pack_state_data() 
{
    // TODO should we pack time data if there is no fix?
    state.time_week_ms = pos_time.time_week_ms;
    state.time_week = pos_time.time_week;
    state.num_sats = pos_time.num_sats;

    if ((pos_time.pos_flags1 & 1)) { // New position
        state.status = AP_GPS::GPS_OK_FIX_3D;
        if ((pos_time.pos_flags2 & 1)) { // Differential position 
            state.status = AP_GPS::GPS_OK_FIX_3D_DGPS;
            if (pos_time.pos_flags2 & 2) { // Differential position method
                if (pos_time.pos_flags2 & 4) {// Differential position method
                    state.status = AP_GPS::GPS_OK_FIX_3D_RTK_FIXED;
                } else {
                    state.status = AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT;
                }
            }
        }
    } else {
        state.status = AP_GPS::NO_FIX;
    }

    state.location.lat = (int32_t)(RAD_TO_DEG_DOUBLE * position.latitude_rad * (double)1e7);
    state.location.lng = (int32_t)(RAD_TO_DEG_DOUBLE * position.longitude_rad * (double)1e7);
    state.location.alt = (int32_t)(position.altitude * 100);
    state.last_gps_time_ms = AP_HAL::millis();

    if ((vel.velocity_flags & 1) == 1) {
        state.ground_speed = vel.horizontal_velocity;
        state.ground_course = wrap_360(degrees(vel.heading));
        fill_3d_velocity();
        state.velocity.z = -vel.vertical_velocity;
        state.have_vertical_velocity = true;
    }

    state.hdop = (uint16_t)(dop.hdop * 100);
    state.vdop = (uint16_t)(dop.vdop * 100);

    state.horizontal_accuracy = (pos_sigma.sigma_east + pos_sigma.sigma_north) / 2;
    state.vertical_accuracy = pos_sigma.sigma_up;
    state.have_horizontal_accuracy = true;
    state.have_vertical_accuracy = true;
}
#endif
