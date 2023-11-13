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
//    param set GPS_TYPE 11 // GSOF
//    param set SERIAL3_PROTOCOL 5 // GPS
//

#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include "AP_GPS.h"
#include "AP_GPS_GSOF.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <GCS_MAVLink/GCS.h>

#if AP_GPS_GSOF_ENABLED

extern const AP_HAL::HAL& hal;

// Set this to 1 to enable debug messages
#define gsof_DEBUGGING 0

#if gsof_DEBUGGING
# define Debug(fmt, args ...)                  \
do {                                            \
    hal.console->printf("%u %s:%s:%d: " fmt "\n",     \
                        AP_HAL::millis(),        \
                        __FILE__,                \
                        __FUNCTION__, __LINE__, \
                        ## args);               \
    hal.scheduler->delay(1);                    \
} while(0)
#else
# define Debug(fmt, args ...)
#endif

AP_GPS_GSOF::AP_GPS_GSOF(AP_GPS &_gps, AP_GPS::GPS_State &_state,
                         AP_HAL::UARTDriver *_port) :
    AP_GPS_Backend(_gps, _state, _port)
{
    // https://receiverhelp.trimble.com/oem-gnss/index.html#GSOFmessages_Overview.html?TocPath=Output%2520Messages%257CGSOF%2520Messages%257COverview%257C_____0
    static_assert(ARRAY_SIZE(gsofmsgreq) <= 10, "The maximum number of outputs allowed with GSOF is 10.");
    
    msg.state = Msg_Parser::State::STARTTX;

    constexpr uint8_t default_com_port = static_cast<uint8_t>(HW_Port::COM2);
    gps._com_port[state.instance].set_default(default_com_port);
    const auto com_port = gps._com_port[state.instance].get();
    if (!validate_com_port(com_port)) {
        // The user parameter for COM port is not a valid GSOF port
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "GSOF instance %d has invalid COM port setting of %d", state.instance, com_port);
        return;
    }
    const HW_Baud baud = HW_Baud::BAUD115K;
    // Start by disabling output during config
    [[maybe_unused]] const auto output_disabled = disableOutput(static_cast<HW_Port>(com_port));
    is_baud_configured = requestBaud(static_cast<HW_Port>(com_port), baud);
}

// Process all bytes available from the stream
//
bool
AP_GPS_GSOF::read(void)
{
    if (!is_baud_configured) {
        // Debug("Baud not configured, not ready to read()");
        return false;
    }

    bool gsof_configured = true;

    static_assert(sizeof(gsofmsgreq) != 0, "gsofmsgreq is not empty");
    while (gsofmsgreq_index < (sizeof(gsofmsgreq))) {
        const auto com_port = gps._com_port[state.instance].get();
        if (!validate_com_port(com_port)) {
            // The user parameter for COM port is not a valid GSOF port
            return false;
        }

        gsof_configured &= requestGSOF(gsofmsgreq[gsofmsgreq_index], static_cast<HW_Port>(com_port), Output_Rate::FREQ_10_HZ);
        gsofmsgreq_index++;
    }
    if (!gsof_configured) {
       Debug("Failed to requestGSOF()");
       return false;
    }

    bool ret = false;
    while (port->available() > 0) {
        const uint8_t temp = port->read();
#if AP_GPS_DEBUG_LOGGING_ENABLED
        log_data(&temp, 1);
#endif
        ret |= parse(temp);
    }

    return ret;
}

bool
AP_GPS_GSOF::parse(const uint8_t temp)
{
    // https://receiverhelp.trimble.com/oem-gnss/index.html#API_DataCollectorFormatPacketStructure.html
    switch (msg.state)
    {
    default:
    case Msg_Parser::State::STARTTX:
        if (temp == STX)
        {
            msg.state = Msg_Parser::State::STATUS;
            msg.read = 0;
            msg.checksumcalc = 0;
        }
        break;
    case Msg_Parser::State::STATUS:
        msg.status = temp;
        msg.state = Msg_Parser::State::PACKETTYPE;
        msg.checksumcalc += temp;
        break;
    case Msg_Parser::State::PACKETTYPE:
        msg.packettype = temp;
        msg.state = Msg_Parser::State::LENGTH;
        msg.checksumcalc += temp;
        break;
    case Msg_Parser::State::LENGTH:
        msg.length = temp;
        msg.state = Msg_Parser::State::DATA;
        msg.checksumcalc += temp;
        break;
    case Msg_Parser::State::DATA:
        msg.data[msg.read] = temp;
        msg.read++;
        msg.checksumcalc += temp;
        if (msg.read >= msg.length)
        {
            msg.state = Msg_Parser::State::CHECKSUM;
        }
        break;
    case Msg_Parser::State::CHECKSUM:
        msg.checksum = temp;
        msg.state = Msg_Parser::State::ENDTX;
        if (msg.checksum == msg.checksumcalc)
        {
            return process_message();
        }
        break;
    case Msg_Parser::State::ENDTX:
        msg.endtx = temp;
        msg.state = Msg_Parser::State::STARTTX;
        break;
    }

    return false;
}

bool
AP_GPS_GSOF::requestBaud(const HW_Port portIndex, const HW_Baud baudRate)
{
    Debug("Requesting baud on port %u", (uint8_t)portIndex);
    // GSOF is supported on the following bauds:
    // 2400, 4800, 9600, 19200, 38400, 115200, 230400

    // This packet is not documented in the API.
    uint8_t buffer[19] = {0x02,0x00,0x64,0x0d,0x00,0x00,0x00, // application file record
                          0x03, 0x00, 0x01, 0x00, // file control information block
                          0x02, 0x04, static_cast<uint8_t>(portIndex), static_cast<uint8_t>(baudRate), 0x00,0x00, // serial port baud format
                          0x00,0x03
                         };

    populateOutgoingTransNumber(buffer);
    populateChecksum(buffer, sizeof(buffer));

    return requestResponse(buffer, sizeof(buffer));
}

bool
AP_GPS_GSOF::requestGSOF(const uint8_t messageType, const HW_Port portIndex, const Output_Rate rateHz)
{
    Debug("Requesting gsof #%u on port %u", messageType, (uint8_t)portIndex);

    // This packet is not documented in the API.
    uint8_t buffer[21] = {0x02,0x00,0x64,0x0f,0x00,0x00,0x00, // application file record
                          0x03,0x00,0x01,0x00, // file control information block
                          0x07,0x06,0x0a,static_cast<uint8_t>(portIndex),static_cast<uint8_t>(rateHz),0x00,messageType,0x00, // output message record
                          0x00,0x03
                         };

    populateOutgoingTransNumber(buffer);
    populateChecksum(buffer, sizeof(buffer));

    return requestResponse(buffer, sizeof(buffer));
}

bool
AP_GPS_GSOF::disableOutput(const HW_Port portIndex) {
    Debug("Disabling output on on port %u", (uint8_t)portIndex);
    switch(portIndex) {
    case HW_Port::COM1:
    {
        constexpr uint8_t cmd[8] = {
            STX,
            0x00,
            0x51,
            0x02,
            0x0A,
            0x01,
            0x5E,
            ETX
        };
        // The checksum is hard-coded.
        // No need to calculate it.
        return requestResponse(cmd, sizeof(cmd));
    }

    case HW_Port::COM2:
    {
        constexpr uint8_t cmd[8] = {
            STX,
            0x00,
            0x51,
            0x02,
            0x0A,
            0x02,
            0x5F,
            ETX
        };
        return requestResponse(cmd, sizeof(cmd));
    }
    }
    return false;
}

bool
AP_GPS_GSOF::requestResponse(const uint8_t* buf, const uint8_t len) {
    if (!port->is_initialized()) {
        Debug("Port not initialized");
        return false;
    }

    for (int attempt = 0; attempt <= configuration_attempts; attempt++) {
        // Clear input buffer before doing configuration
        if (!port->discard_input()) {
            Debug("Failed to discard input");
            return false;
        };

        port->write((const uint8_t*)buf, len);

        constexpr uint16_t expected_bytes = 1;
        // TODO use wait_timeout instead once HAL_SITL has it implemented.
        const auto start_wait = AP_HAL::millis();
        auto now = start_wait;
        while (now  - start_wait <= configuration_wait_time_ms) {
            if (port->available() >= expected_bytes) {
                break;
            }
            constexpr uint16_t delay_us = 100;
            hal.scheduler->delay_microseconds(delay_us);
            now = AP_HAL::millis();
        }

        const auto available_bytes = port->available();
        if (available_bytes != expected_bytes) {
            Debug("Didn't get expected bytes, got %u bytes back", available_bytes);
            return false;
        }

        uint8_t resp_code;
        if(port->read(resp_code) && resp_code == ACK) {
            Debug("Got ack");
            return true;
        } else {
            Debug("Didn't get ACK, got 0x%02x", resp_code);
        }
    }
    return false;
}

void
AP_GPS_GSOF::populateChecksum(uint8_t* buf, const uint8_t len)
{
    // The buffer is of size len.
    // If ETX is not element buf[len-1], the buf/len numbers are wrong.
    // Same problem if the len is too small for a valid packet.
    if (len <= 3 || buf[len - 1] != ETX) {
        return;
    }

    // The checksum field is at buf[len - 2]
    uint8_t checksum = 0;
    const uint8_t payload_size = len - 2;
    for (uint8_t i = 1; i < payload_size; i++) {
        checksum += buf[i];
    }

    buf[len -2] = checksum;
}

void
AP_GPS_GSOF::populateOutgoingTransNumber(uint8_t* buf) {
   buf[4] = packetOutboundTransNumber++; 
}

double
AP_GPS_GSOF::SwapDouble(const uint8_t* src, const uint32_t pos) const
{
    union {
        double d;
        char bytes[sizeof(double)];
    } doubleu;
    doubleu.bytes[0] = src[pos + 7];
    doubleu.bytes[1] = src[pos + 6];
    doubleu.bytes[2] = src[pos + 5];
    doubleu.bytes[3] = src[pos + 4];
    doubleu.bytes[4] = src[pos + 3];
    doubleu.bytes[5] = src[pos + 2];
    doubleu.bytes[6] = src[pos + 1];
    doubleu.bytes[7] = src[pos + 0];

    return doubleu.d;
}

float
AP_GPS_GSOF::SwapFloat(const uint8_t* src, const uint32_t pos) const
{
    union {
        float f;
        char bytes[sizeof(float)];
    } floatu;
    floatu.bytes[0] = src[pos + 3];
    floatu.bytes[1] = src[pos + 2];
    floatu.bytes[2] = src[pos + 1];
    floatu.bytes[3] = src[pos + 0];

    return floatu.f;
}

uint32_t
AP_GPS_GSOF::SwapUint32(const uint8_t* src, const uint32_t pos) const
{
    uint32_t u;
    memcpy(&u, &src[pos], sizeof(u));
    return be32toh(u);
}

uint16_t
AP_GPS_GSOF::SwapUint16(const uint8_t* src, const uint32_t pos) const
{
    uint16_t u;
    memcpy(&u, &src[pos], sizeof(u));
    return be16toh(u);
}

bool
AP_GPS_GSOF::process_message(void)
{
    if (msg.packettype == 0x40) { // GSOF
        // https://receiverhelp.trimble.com/oem-gnss/index.html#GSOFmessages_TIME.html?TocPath=Output%2520Messages%257CGSOF%2520Messages%257C_____25
#if gsof_DEBUGGING
        //trans_number is functionally the sequence number.
        const uint8_t trans_number = msg.data[0];
        const uint8_t pageidx = msg.data[1];
        const uint8_t maxpageidx = msg.data[2];

        Debug("GSOF page: %u of %u (trans_number=%u)",
              pageidx, maxpageidx, trans_number);
#endif

        int valid = 0;

        // want 1 2 8 9 12
        for (uint32_t a = 3; a < msg.length; a++)
        {
            const uint8_t output_type = msg.data[a];
            a++;
            const uint8_t output_length = msg.data[a];
            a++;
            //Debug("GSOF type: " + output_type + " len: " + output_length);

            if (output_type == 1) // pos time
            {
                // https://receiverhelp.trimble.com/oem-gnss/index.html#GSOFmessages_TIME.html?TocPath=Output%2520Messages%257CGSOF%2520Messages%257C_____25
                state.time_week_ms = SwapUint32(msg.data, a);
                state.time_week = SwapUint16(msg.data, a + 4);
                state.num_sats = msg.data[a + 6];
                const uint8_t posf1 = msg.data[a + 7];
                const uint8_t posf2 = msg.data[a + 8];

                //Debug("POSTIME: " + posf1 + " " + posf2);
                
                if ((posf1 & 1)) { // New position
                    state.status = AP_GPS::GPS_OK_FIX_3D;
                    if ((posf2 & 1)) { // Differential position 
                        state.status = AP_GPS::GPS_OK_FIX_3D_DGPS;
                        if (posf2 & 2) { // Differential position method
                            if (posf2 & 4) {// Differential position method
                                state.status = AP_GPS::GPS_OK_FIX_3D_RTK_FIXED;
                            } else {
                                state.status = AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT;
                            }
                        }
                    }
                } else {
                    state.status = AP_GPS::NO_FIX;
                }
                valid++;
            }
            else if (output_type == 2) // position
            {
                // This packet is not documented in Trimble's receiver help as of May 18, 2023
                state.location.lat = (int32_t)(RAD_TO_DEG_DOUBLE * (SwapDouble(msg.data, a)) * (double)1e7);
                state.location.lng = (int32_t)(RAD_TO_DEG_DOUBLE * (SwapDouble(msg.data, a + 8)) * (double)1e7);
                state.location.alt = (int32_t)(SwapDouble(msg.data, a + 16) * 100);

                state.last_gps_time_ms = AP_HAL::millis();

                valid++;
            }
            else if (output_type == 8) // velocity
            {
                // https://receiverhelp.trimble.com/oem-gnss/index.html#GSOFmessages_Velocity.html?TocPath=Output%2520Messages%257CGSOF%2520Messages%257C_____32
                const uint8_t vflag = msg.data[a];
                if ((vflag & 1) == 1)
                {
                    state.ground_speed = SwapFloat(msg.data, a + 1);
                    state.ground_course = degrees(SwapFloat(msg.data, a + 5));
                    fill_3d_velocity();
                    state.velocity.z = -SwapFloat(msg.data, a + 9);
                    state.have_vertical_velocity = true;
                }
                valid++;
            }
            else if (output_type == 9) //dop
            {
                // https://receiverhelp.trimble.com/oem-gnss/index.html#GSOFmessages_PDOP.html?TocPath=Output%2520Messages%257CGSOF%2520Messages%257C_____12
                state.hdop = (uint16_t)(SwapFloat(msg.data, a + 4) * 100);
                valid++;
            }
            else if (output_type == 12) // position sigma
            {
                // https://receiverhelp.trimble.com/oem-gnss/index.html#GSOFmessages_SIGMA.html?TocPath=Output%2520Messages%257CGSOF%2520Messages%257C_____24
                state.horizontal_accuracy = (SwapFloat(msg.data, a + 4) + SwapFloat(msg.data, a + 8)) / 2;
                state.vertical_accuracy = SwapFloat(msg.data, a + 16);
                state.have_horizontal_accuracy = true;
                state.have_vertical_accuracy = true;
                valid++;
            }

            a += output_length-1u;
        }

        if (valid == 5) {
            return true;
        } else {
            state.status = AP_GPS::NO_FIX;
        }
    }

    return false;
}

bool
AP_GPS_GSOF::validate_baud(const uint8_t baud) const {
    switch(baud) {
        case static_cast<uint8_t>(HW_Baud::BAUD230K):
        case static_cast<uint8_t>(HW_Baud::BAUD115K):
            return true;
        default:
            return false;
    }
}
bool
AP_GPS_GSOF::validate_com_port(const uint8_t com_port) const {
    switch(com_port) {
        case static_cast<uint8_t>(HW_Port::COM1):
        case static_cast<uint8_t>(HW_Port::COM2):
            return true;
        default:
            return false;
    }
}

#endif
