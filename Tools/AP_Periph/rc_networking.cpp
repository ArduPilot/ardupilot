/*
  Class to handle RC over network and convert it to CAN
 */

#include "rc_networking.h"
#include <AP_HAL/AP_HAL_Boards.h>
#include "AP_Periph.h"
#include <utility>
#include <cstdio>
#include <cstring>
#include <AP_Param/AP_Param.h>
#include <AP_Common/AP_FWVersion.h>
#include <AP_RCProtocol/AP_RCProtocol.h>
#include <dronecan_msgs.h>

#define SBUS_FLAGS_BYTE		23
#define SBUS_FAILSAFE_BIT	3
#define SBUS_FRAMELOST_BIT	2

/* define range mapping here, -+100% -> 1000..2000 */
#define SBUS_RANGE_MIN 200
#define SBUS_RANGE_MAX 1800
#define SBUS_RANGE_RANGE (SBUS_RANGE_MAX - SBUS_RANGE_MIN)

#define SBUS_TARGET_MIN 1000
#define SBUS_TARGET_MAX 2000
#define SBUS_TARGET_RANGE (SBUS_TARGET_MAX - SBUS_TARGET_MIN)

// this is 875
#define SBUS_SCALE_OFFSET (SBUS_TARGET_MIN - ((SBUS_TARGET_RANGE * SBUS_RANGE_MIN / SBUS_RANGE_RANGE)))

#ifndef HAL_SBUS_FRAME_GAP
#define HAL_SBUS_FRAME_GAP 2000U
#endif

extern const AP_HAL::HAL& hal;

bool UDPChannel::open_client(const char* ip_str, uint16_t port)
{
    close();

    auto s = NEW_NOTHROW SocketAPM(true);
    if (s == nullptr) {
        return false;
    }

    if (!s->connect(ip_str, port)) {
        delete s;
        return false;
    }

    s->set_blocking(false);
    sock = s;
    connected = true;
    return true;
}

bool UDPChannel::open_server(uint16_t port)
{
    close();

    auto s = NEW_NOTHROW SocketAPM(true);
    if (s == nullptr) {
        return false;
    }

    if (!s->bind("0.0.0.0", port)) {
        delete s;
        return false;
    }

    s->set_blocking(false);
    sock = s;
    connected = true;
    return true;
}

void UDPChannel::close()
{
    if (sock != nullptr) {
        delete sock;
        sock = nullptr;
    }
    connected = false;
}

size_t UDPChannel::write(const uint8_t* buf, size_t len, uint8_t* error)
{
    if (sock == nullptr) {
        if (error) *error = EINVAL;
        return 0;
    }

    const ssize_t ret = sock->send(buf, len);
    if (ret <= 0) {
        if (error) *error = errno;
        connected = false;
        return 0;
    }

    tx_bytes_total += ret;
    if (error) *error = 0;
    return ret;
}

size_t UDPChannel::read(uint8_t* buf, size_t len, int timeout_ms, uint8_t* error)
{
    if (sock == nullptr) {
        if (error) *error = EINVAL;
        return 0;
    }

    const ssize_t ret = sock->recv(buf, len, timeout_ms);
    if (ret <= 0) {
        if (error) *error = errno;
        return 0;
    }

    rx_bytes_total += ret;
    if (error) *error = 0;
    return ret;
}

AirSide_RC_Networking::AirSide_RC_Networking() {}

bool AirSide_RC_Networking::init()
{
    // RC input (joystick -> UAV)
    if (!rc_in.open_server(14551)) {
        hal.console->printf("RC_IN open failed\n");
        return false;
    }

    hal.console->printf("AirBoss_Networking init OK\n");
    return true;
}

void AirSide_RC_Networking::read_sbus_packet()
{
    uint8_t buf[120];
    uint8_t err = 0;

    const size_t nbytes = rc_in.read(buf, sizeof(buf), 0, &err);
    if (nbytes == 0) {
        return;
    }

    // can_printf("RC IN read %u bytes, err=%u\n", nbytes, err);
    process_sbus_buffer(buf, nbytes);
}

void AirSide_RC_Networking::update()
{
    read_sbus_packet();
    if (new_sbus_frame) {
        // Process the latest SBUS frame
        new_sbus_frame = false;
        if (sbus_decode(latest_sbus_frame, decoded_rc_values, &rc_num_values,
            _sbus_failsafe, SBUS_INPUT_CHANNELS) &&
            rc_num_values >= MIN_RCIN_CHANNELS) {
                new_sbus_can_frame = true;
            }
    }
}

void AirSide_RC_Networking::process_sbus_buffer(const uint8_t *buf, uint32_t nbytes)
{
    const uint32_t SBUS_FRAME_SIZE = 25;

    // Append new data to carry buffer
    uint32_t total_len = carry_buffer_len + nbytes;
    if (total_len > sizeof(carry_buffer)) {
        total_len = sizeof(carry_buffer);  // prevent overflow
    }

    // Copy new data to carry buffer
    memcpy(&carry_buffer[carry_buffer_len], buf, total_len - carry_buffer_len);
    carry_buffer_len = total_len;

    uint32_t offset = 0;

    // Parse complete SBUS frames
    while (carry_buffer_len - offset >= SBUS_FRAME_SIZE) {
        const uint8_t *packet = &carry_buffer[offset];

        // Look for SBUS header
        if (packet[0] == 0x0F) {
            // Found header, check footer
            bool is_valid = (packet[24] == 0x00 || packet[24] == 0x04);
            // Check CRC over bytes 0–22
            // uint8_t computed_crc = sbus_crc8(packet, 23);  // Exclude byte 23 (CRC) and 24 (footer)
            // can_printf("CRC: %02X %02X\n", computed_crc, packet[23]);
            if (is_valid ) {
                // Valid SBUS packet
                valid_sbus_packets++;
                total_sbus_bytes += SBUS_FRAME_SIZE;

                // Check for timing/jitter
                uint32_t now = AP_HAL::millis();
                uint32_t frame_interval = now - last_sbus_timestamp;
                if (last_sbus_timestamp != 0 && (frame_interval > 30)) {
                    uint32_t missed_frames = (frame_interval / 20) - 1; // compute how many were missed
                    lost_frames += missed_frames;
                }
                last_sbus_timestamp = now;
                // Copy the valid SBUS frame to latest_sbus_frame
                memcpy(latest_sbus_frame, packet, SBUS_FRAME_SIZE);
                latest_sbus_frame[23] = 0x00;  // Set the footer to 0x00
                // Set the flag to indicate a new SBUS frame is available
                new_sbus_frame = true;
            } else {
                // Invalid SBUS packet (bad footer)
                invalid_sbus_packets++;
            }

            // Move forward by one full frame
            offset += SBUS_FRAME_SIZE;
        } else {
            // Not a valid SBUS header, move forward 1 byte to search again
            offset += 1;
        }
    }

    // Keep leftover bytes for next round
    carry_buffer_len -= offset;
    if (carry_buffer_len > 0 && offset > 0) {
        memmove(carry_buffer, &carry_buffer[offset], carry_buffer_len);
    }
}

// decode a full SBUS frame
bool AirSide_RC_Networking::sbus_decode(const uint8_t frame[25], uint16_t *values, uint16_t *num_values,
                                     bool &sbus_failsafe, uint16_t max_values)
{
    /* check frame boundary markers to avoid out-of-sync cases */
    if ((frame[0] != 0x0f)) {
        return false;
    }

    uint16_t chancount = SBUS_INPUT_CHANNELS;

    decode_11bit_channels((const uint8_t*)(&frame[1]), max_values, values,
        SBUS_TARGET_RANGE, SBUS_RANGE_RANGE, SBUS_SCALE_OFFSET);

    /* decode switch channels if data fields are wide enough */
    if (max_values > 17 && SBUS_INPUT_CHANNELS > 15) {
        chancount = 18;

        /* channel 17 (index 16) */
        values[16] = (frame[SBUS_FLAGS_BYTE] & (1 << 0))?1998:998;
        /* channel 18 (index 17) */
        values[17] = (frame[SBUS_FLAGS_BYTE] & (1 << 1))?1998:998;
    }

    /* note the number of channels decoded */
    *num_values = chancount;

    /*
      as SBUS is such a weak protocol we additionally check if any of
      the first 4 channels are at or below the minimum value of
      875. We consider the frame as a failsafe in that case, which
      means we log the data but won't use it
     */
    bool invalid_data = false;
    for (uint8_t i=0; i<4; i++) {
        if (values[i] <= SBUS_SCALE_OFFSET) {
            invalid_data = true;
        }
    }

    if (invalid_data) {
        sbus_failsafe = true;
    }

    /* decode and handle failsafe and frame-lost flags */
    // if (frame[SBUS_FLAGS_BYTE] & (1 << SBUS_FAILSAFE_BIT)) { /* failsafe */
    //     /* report that we failed to read anything valid off the receiver */
    //     sbus_failsafe = true;
    // } else if (invalid_data) {
    //     sbus_failsafe = true;
    // } else if (frame[SBUS_FLAGS_BYTE] & (1 << SBUS_FRAMELOST_BIT)) { /* a frame was lost */
    //     /* set a special warning flag
    //      *
    //      * Attention! This flag indicates a skipped frame only, not a total link loss! Handling this
    //      * condition as fail-safe greatly reduces the reliability and range of the radio link,
    //      * e.g. by prematurely issuing return-to-launch!!! */

    //     sbus_failsafe = false;
    // } else {
    //     sbus_failsafe = false;
    // }

    return true;
}

/*
  decode channels from the standard 11bit format (used by CRSF, SBUS, FPort and FPort2)
  must be used on multiples of 8 channels
*/
void AirSide_RC_Networking::decode_11bit_channels(const uint8_t* data, uint8_t nchannels, uint16_t *values, uint16_t mult, uint16_t div, uint16_t offset)
{
#define CHANNEL_SCALE(x) ((int32_t(x) * mult) / div + offset)
    while (nchannels >= 8) {
        const Channels11Bit_8Chan* channels = (const Channels11Bit_8Chan*)data;
        values[0] = CHANNEL_SCALE(channels->ch0);
        values[1] = CHANNEL_SCALE(channels->ch1);
        values[2] = CHANNEL_SCALE(channels->ch2);
        values[3] = CHANNEL_SCALE(channels->ch3);
        values[4] = CHANNEL_SCALE(channels->ch4);
        values[5] = CHANNEL_SCALE(channels->ch5);
        values[6] = CHANNEL_SCALE(channels->ch6);
        values[7] = CHANNEL_SCALE(channels->ch7);

        nchannels -= 8;
        data += sizeof(*channels);
        values += 8;
    }
}

/*
  send an RCInput CAN message
 */
void AP_Periph_FW::can_send_RCInput(uint8_t quality, uint16_t *values, uint8_t nvalues, bool in_failsafe, bool quality_valid, uint8_t id)
{
   uint16_t status = 0;
   if (quality_valid) {
       status |= DRONECAN_SENSORS_RC_RCINPUT_STATUS_QUALITY_VALID;
   }
   if (in_failsafe) {
       status |= DRONECAN_SENSORS_RC_RCINPUT_STATUS_FAILSAFE;
   }

   // assemble packet
   dronecan_sensors_rc_RCInput pkt {};
   pkt.quality = quality;
   pkt.status = status;
   pkt.rcin.len = nvalues;
   for (uint8_t i=0; i<nvalues; i++) {
       pkt.rcin.data[i] = values[i];
   }
   pkt.id = id;

   // encode and send message:
   uint8_t buffer[DRONECAN_SENSORS_RC_RCINPUT_MAX_SIZE];

   uint16_t total_size = dronecan_sensors_rc_RCInput_encode(&pkt, buffer, !periph.canfdout());

   canard_broadcast(DRONECAN_SENSORS_RC_RCINPUT_SIGNATURE,
                    DRONECAN_SENSORS_RC_RCINPUT_ID,
                    CANARD_TRANSFER_PRIORITY_HIGH,
                    buffer,
                    total_size);
}

