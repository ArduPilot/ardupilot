#include <AP_HAL/AP_HAL.h>

// Only support ELRS simulation in SITL (not Sim on Hardware)
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "SIM_ELRS.h"
#include <SITL/SITL.h>

#include <AP_HAL_SITL/AP_HAL_SITL.h>
#include <AP_HAL_SITL/UARTDriver.h>

#include "include/mavlink/v2.0/all/mavlink.h"

// Example command: -A --serial2=sim:ELRS
// TCP connection will be started on normal AP port eg 5763 for serial 2

// Baud rate must be set correctly
// param set SERIAL2_BAUD 460

using namespace SITL;

ELRS::ELRS(const uint8_t portNumber, HALSITL::SITL_State_Common *sitl_state) :
    // Mirror typical ELRS UART buffer sizes
    SerialDevice::SerialDevice(64, 128),
    // Mirror MAVLink buffer sizes
    mavlinkInputBuffer(2048),
    mavlinkOutputBuffer(2048),
    // 255 is typically used by the GCS, for RC override to work in ArduPilot `MAV_GCS_SYSID` must be set to this value (255 is the default)
    this_system_id(255),
    // Strictly this is not a valid source component ID
    this_component_id(MAV_COMPONENT::MAV_COMP_ID_ALL),
    // Typical setup is about 500 B /s
    input_data_rate(500),
    output_data_rate(500),
    target_address("127.0.0.1"),
    target_port(5761 + portNumber)
{

    // Setup TCP server
    listener.reuseaddress();
    listener.bind(target_address, target_port);
    listener.listen(1);
    listener.set_blocking(false);

}

void ELRS::update()
{
    // Connect to incoming TCP
    if (sock == nullptr) {
        sock = listener.accept(0);
        if (sock != nullptr) {
            sock->set_blocking(false);
            sock->reuseaddress();
            ::printf("ELRS connected to %s:%u\n", target_address, (unsigned)target_port);
        }
    }
    if (sock == nullptr) {
        return;
    }

    // Read from AP into radio
    const uint32_t input_space = mavlinkInputBuffer.space();
    if (input_space > 0) {
        uint8_t buf[input_space];
        ssize_t len = read_from_autopilot((char*)buf, input_space);
        mavlinkInputBuffer.write(buf, len);
    }

    // Send from radio to GCS
    const uint32_t send_bytes = input_limit.max_bytes(input_data_rate);
    if (send_bytes > 0) {
        uint8_t buf[send_bytes];
        const uint32_t len = mavlinkInputBuffer.read(buf, send_bytes);
        if (len > 0) {
            sock->send(buf, len);
        }
    }

    // Incoming data from GCS to radio
    const uint32_t receive_bytes = output_limit.max_bytes(output_data_rate);
    if (receive_bytes > 0) {
        uint8_t buf[receive_bytes];
        const ssize_t len = sock->recv(buf, receive_bytes, 1);
        if (len > 0) {
            mavlinkOutputBuffer.write(buf, len);
        } else if (len == 0) {
            // EOF, go back to waiting for a new connection
            delete sock;
            sock = nullptr;
        }
    }

    // Write from radio to AP
    sendQueuedData();
}

// Function to behave like MAVLink libs `mavlink_parse_char` but use local buffer
uint8_t ELRS::mavlink_parse_char_helper(uint8_t c, mavlink_message_t* r_message, mavlink_status_t* r_mavlink_status)
{
    uint8_t msg_received = mavlink_frame_char_buffer(&mavlink.rxmsg, &mavlink.status, c, r_message, r_mavlink_status);
    if ((msg_received == MAVLINK_FRAMING_BAD_CRC) || (msg_received == MAVLINK_FRAMING_BAD_SIGNATURE)) {
        return 0;
    }
    return msg_received;
}

// Send incoming data to AP, this is a re-implementation of the ELRS function found here:
// https://github.com/ExpressLRS/ExpressLRS/blob/0d31863f34ca16a036e94a9c2a56038ae56c7f9e/src/src/rx-serial/SerialMavlink.cpp#L78
void ELRS::sendQueuedData()
{

    // Send radio messages at 100Hz
    const uint32_t now = AP_HAL::millis();
    if ((now - lastSentFlowCtrl) > 10) {
        lastSentFlowCtrl = now; 

        // Space remaining as a percentage.
        const uint8_t percentage_remaining = (mavlinkInputBuffer.space() * 100) / mavlinkInputBuffer.get_size();

        // Populate radio status packet
        const mavlink_radio_status_t radio_status {
            rxerrors: 0,
            fixed: 0,
            rssi: UINT8_MAX, // Unknown
            remrssi: UINT8_MAX, // Unknown
            txbuf: percentage_remaining,
            noise: UINT8_MAX, // Unknown
            remnoise: UINT8_MAX, // Unknown
        };

        uint8_t buf[MAVLINK_MSG_ID_RADIO_STATUS_LEN + MAVLINK_NUM_NON_PAYLOAD_BYTES];
        mavlink_message_t msg;
        mavlink_msg_radio_status_encode(this_system_id, this_component_id, &msg, &radio_status);
        uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
        write_to_autopilot((char*)buf, len);
    }

    // Read one byte at a time until were done
    while (true) {
        uint8_t c;
        if (!mavlinkOutputBuffer.read_byte(&c)) {
            break;
        }

        mavlink_message_t msg;
        mavlink_status_t status;

        // Try parse a mavlink message
        if (mavlink_parse_char_helper(c, &msg, &status)) {
            // Message decoded successfully

            // Forward message to the UART
            uint8_t buf[MAVLINK_MAX_PACKET_LEN];
            uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
            uint16_t written = write_to_autopilot((char*)buf, len);
            if ((written != uint16_t(-1)) && (len != written)) {
                ::fprintf(stderr, "Failed to write full msg, wanted %u achieved %u (msg id: %u)\n", len, written, msg.msgid);
            }
        }
    }

}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
