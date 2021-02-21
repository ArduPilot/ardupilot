#include <AP_Math/AP_Math.h>
#include <AP_SerialManager/AP_SerialManager.h>

#include "msp.h"

#include <stdint.h>

#if HAL_MSP_ENABLED

/*
 ported from betaflight/src/main/msp/msp_serial.c
 */
uint8_t MSP::msp_serial_checksum_buf(uint8_t checksum, const uint8_t *data, uint32_t len)
{
    while (len-- > 0) {
        checksum ^= *data++;
    }
    return checksum;
}

/*
 ported from betaflight/src/main/msp/msp_serial.c
 */
uint32_t MSP::msp_serial_send_frame(msp_port_t *msp, const uint8_t * hdr, uint32_t hdr_len, const uint8_t * data, uint32_t data_len, const uint8_t * crc, uint32_t crc_len)
{
    if (msp->uart == nullptr) {
        return 0;
    }

    // We are allowed to send out the response if
    //  a) TX buffer is completely empty (we are talking to well-behaving party that follows request-response scheduling;
    //     this allows us to transmit jumbo frames bigger than TX buffer (serialWriteBuf will block, but for jumbo frames we don't care)
    //  b) Response fits into TX buffer
    const uint32_t total_frame_length = hdr_len + data_len + crc_len;

    if (!msp->uart->tx_pending() && (msp->uart->txspace() < total_frame_length)) {
        return 0;
    }

    // Transmit frame
    msp->uart->write(hdr, hdr_len);
    msp->uart->write(data, data_len);
    msp->uart->write(crc, crc_len);

    return total_frame_length;
}

/*
 ported from betaflight/src/main/msp/msp_serial.c
 */
uint32_t MSP::msp_serial_encode(msp_port_t *msp, msp_packet_t *packet, msp_version_e msp_version, bool is_request)
{
    static const uint8_t msp_magic[MSP_VERSION_COUNT] = MSP_VERSION_MAGIC_INITIALIZER;
    /*
        Note: after calling sbuf_switch_to_reader() sbuf_bytes_remaining() returns the size of the packet
    */
    const uint16_t data_len = sbuf_bytes_remaining(&packet->buf);
    uint8_t code;
    if (is_request) {
        code = '<';
    } else if (packet->result == MSP_RESULT_ERROR) {
        code = '!';
    } else {
        code = '>';
    }
    const uint8_t hdr_buf[16] = { '$', msp_magic[msp_version], code };
    uint8_t crc_buf[2];
    uint32_t hdr_len = 3;
    uint32_t crc_len = 0;

#define V1_CHECKSUM_STARTPOS 3
    if (msp_version == MSP_V1) {
        msp_header_v1_t * hdr_v1 = (msp_header_v1_t *)&hdr_buf[hdr_len];
        hdr_len += sizeof(msp_header_v1_t);
        hdr_v1->cmd = packet->cmd;

        // Add JUMBO-frame header if necessary
        if (data_len >= JUMBO_FRAME_SIZE_LIMIT) {
            msp_header_jumbo_t * hdr_jumbo = (msp_header_jumbo_t *)&hdr_buf[hdr_len];
            hdr_len += sizeof(msp_header_jumbo_t);

            hdr_v1->size = JUMBO_FRAME_SIZE_LIMIT;
            hdr_jumbo->size = data_len;
        } else {
            hdr_v1->size = data_len;
        }

        // Pre-calculate CRC
        crc_buf[crc_len] = msp_serial_checksum_buf(0, hdr_buf + V1_CHECKSUM_STARTPOS, hdr_len - V1_CHECKSUM_STARTPOS);
        crc_buf[crc_len] = msp_serial_checksum_buf(crc_buf[crc_len], sbuf_ptr(&packet->buf), data_len);
        crc_len++;
    } else if (msp_version == MSP_V2_OVER_V1) {
        msp_header_v1_t * hdr_v1 = (msp_header_v1_t *)&hdr_buf[hdr_len];

        hdr_len += sizeof(msp_header_v1_t);

        msp_header_v2_t * hdr_v2 = (msp_header_v2_t *)&hdr_buf[hdr_len];
        hdr_len += sizeof(msp_header_v2_t);

        const uint32_t v1_payload_size = sizeof(msp_header_v2_t) + data_len + 1;  // MSPv2 header + data payload + MSPv2 checksum
        hdr_v1->cmd = MSP_V2_FRAME_ID;

        // Add JUMBO-frame header if necessary
        if (v1_payload_size >= JUMBO_FRAME_SIZE_LIMIT) {
            msp_header_jumbo_t * hdr_jumbo = (msp_header_jumbo_t *)&hdr_buf[hdr_len];
            hdr_len += sizeof(msp_header_jumbo_t);

            hdr_v1->size = JUMBO_FRAME_SIZE_LIMIT;
            hdr_jumbo->size = v1_payload_size;
        } else {
            hdr_v1->size = v1_payload_size;
        }

        // Fill V2 header
        hdr_v2->flags = packet->flags;
        hdr_v2->cmd = packet->cmd;
        hdr_v2->size = data_len;

        // V2 CRC: only V2 header + data payload
        crc_buf[crc_len] = crc8_dvb_s2_update(0, (uint8_t *)hdr_v2, sizeof(msp_header_v2_t));
        crc_buf[crc_len] = crc8_dvb_s2_update(crc_buf[crc_len], sbuf_ptr(&packet->buf), data_len);
        crc_len++;

        // V1 CRC: All headers + data payload + V2 CRC byte
        crc_buf[crc_len] = msp_serial_checksum_buf(0, hdr_buf + V1_CHECKSUM_STARTPOS, hdr_len - V1_CHECKSUM_STARTPOS);
        crc_buf[crc_len] = msp_serial_checksum_buf(crc_buf[crc_len], sbuf_ptr(&packet->buf), data_len);
        crc_buf[crc_len] = msp_serial_checksum_buf(crc_buf[crc_len], crc_buf, crc_len);
        crc_len++;
    } else if (msp_version == MSP_V2_NATIVE) {
        msp_header_v2_t * hdr_v2 = (msp_header_v2_t *)&hdr_buf[hdr_len];
        hdr_len += sizeof(msp_header_v2_t);

        hdr_v2->flags = packet->flags;
        hdr_v2->cmd = packet->cmd;
        hdr_v2->size = data_len;

        crc_buf[crc_len] = crc8_dvb_s2_update(0, (uint8_t *)hdr_v2, sizeof(msp_header_v2_t));
        crc_buf[crc_len] = crc8_dvb_s2_update(crc_buf[crc_len], sbuf_ptr(&packet->buf), data_len);
        crc_len++;
    } else {
        // Shouldn't get here
        return 0;
    }

    // Send the frame
    return msp_serial_send_frame(msp, hdr_buf, hdr_len, sbuf_ptr(&packet->buf), data_len, crc_buf, crc_len);
}

/*
 ported from betaflight/src/main/msp/msp_serial.c
 */
bool MSP::msp_parse_received_data(msp_port_t *msp, uint8_t c)
{
    switch (msp->c_state) {
    default:
    case MSP_IDLE:      // Waiting for '$' character
        if (c == '$') {
            msp->msp_version = MSP_V1;
            msp->c_state = MSP_HEADER_START;
        } else {
            return false;
        }
        break;

    case MSP_HEADER_START:  // Waiting for 'M' (MSPv1 / MSPv2_over_v1) or 'X' (MSPv2 native)
        switch (c) {
        case 'M':
            msp->c_state = MSP_HEADER_M;
            break;
        case 'X':
            msp->c_state = MSP_HEADER_X;
            break;
        default:
            msp->c_state = MSP_IDLE;
            break;
        }
        break;

    case MSP_HEADER_M:      // Waiting for '<'
        if (c == '<') {
            msp->offset = 0;
            msp->checksum1 = 0;
            msp->checksum2 = 0;
            msp->c_state = MSP_HEADER_V1;
        } else {
            msp->c_state = MSP_IDLE;
        }
        break;

    case MSP_HEADER_X:
        if (c == '<') {
            msp->offset = 0;
            msp->checksum2 = 0;
            msp->msp_version = MSP_V2_NATIVE;
            msp->c_state = MSP_HEADER_V2_NATIVE;
        } else {
            msp->c_state = MSP_IDLE;
        }
        break;

    case MSP_HEADER_V1:     // Now receive v1 header (size/cmd), this is already checksummable
        msp->in_buf[msp->offset++] = c;
        msp->checksum1 ^= c;
        if (msp->offset == sizeof(msp_header_v1_t)) {
            msp_header_v1_t * hdr = (msp_header_v1_t *)&msp->in_buf[0];
            // Check incoming buffer size limit
            if (hdr->size > MSP_PORT_INBUF_SIZE) {
                msp->c_state = MSP_IDLE;
            } else if (hdr->cmd == MSP_V2_FRAME_ID) {
                // MSPv1 payload must be big enough to hold V2 header + extra checksum
                if (hdr->size >= sizeof(msp_header_v2_t) + 1) {
                    msp->msp_version = MSP_V2_OVER_V1;
                    msp->c_state = MSP_HEADER_V2_OVER_V1;
                } else {
                    msp->c_state = MSP_IDLE;
                }
            } else {
                msp->data_size = hdr->size;
                msp->cmd_msp = hdr->cmd;
                msp->cmd_flags = 0;
                msp->offset = 0;                // re-use buffer
                msp->c_state = msp->data_size > 0 ? MSP_PAYLOAD_V1 : MSP_CHECKSUM_V1;    // If no payload - jump to checksum byte
            }
        }
        break;

    case MSP_PAYLOAD_V1:
        msp->in_buf[msp->offset++] = c;
        msp->checksum1 ^= c;
        if (msp->offset == msp->data_size) {
            msp->c_state = MSP_CHECKSUM_V1;
        }
        break;

    case MSP_CHECKSUM_V1:
        if (msp->checksum1 == c) {
            msp->c_state = MSP_COMMAND_RECEIVED;
        } else {
            msp->c_state = MSP_IDLE;
        }
        break;

    case MSP_HEADER_V2_OVER_V1:     // V2 header is part of V1 payload - we need to calculate both checksums now
        msp->in_buf[msp->offset++] = c;
        msp->checksum1 ^= c;
        msp->checksum2 = crc8_dvb_s2(msp->checksum2, c);
        if (msp->offset == (sizeof(msp_header_v2_t) + sizeof(msp_header_v1_t))) {
            msp_header_v2_t * hdrv2 = (msp_header_v2_t *)&msp->in_buf[sizeof(msp_header_v1_t)];
            msp->data_size = hdrv2->size;

            // Check for potential buffer overflow
            if (hdrv2->size > MSP_PORT_INBUF_SIZE) {
                msp->c_state = MSP_IDLE;
            } else {
                msp->cmd_msp = hdrv2->cmd;
                msp->cmd_flags = hdrv2->flags;
                msp->offset = 0;                // re-use buffer
                msp->c_state = msp->data_size > 0 ? MSP_PAYLOAD_V2_OVER_V1 : MSP_CHECKSUM_V2_OVER_V1;
            }
        }
        break;

    case MSP_PAYLOAD_V2_OVER_V1:
        msp->checksum2 = crc8_dvb_s2(msp->checksum2, c);
        msp->checksum1 ^= c;
        msp->in_buf[msp->offset++] = c;

        if (msp->offset == msp->data_size) {
            msp->c_state = MSP_CHECKSUM_V2_OVER_V1;
        }
        break;

    case MSP_CHECKSUM_V2_OVER_V1:
        msp->checksum1 ^= c;
        if (msp->checksum2 == c) {
            msp->c_state = MSP_CHECKSUM_V1; // Checksum 2 correct - verify v1 checksum
        } else {
            msp->c_state = MSP_IDLE;
        }
        break;

    case MSP_HEADER_V2_NATIVE:
        msp->in_buf[msp->offset++] = c;
        msp->checksum2 = crc8_dvb_s2(msp->checksum2, c);
        if (msp->offset == sizeof(msp_header_v2_t)) {
            msp_header_v2_t * hdrv2 = (msp_header_v2_t *)&msp->in_buf[0];

            // Check for potential buffer overflow
            if (hdrv2->size > MSP_PORT_INBUF_SIZE) {
                msp->c_state = MSP_IDLE;
            } else {
                msp->data_size = hdrv2->size;
                msp->cmd_msp = hdrv2->cmd;
                msp->cmd_flags = hdrv2->flags;
                msp->offset = 0;                // re-use buffer
                msp->c_state = msp->data_size > 0 ? MSP_PAYLOAD_V2_NATIVE : MSP_CHECKSUM_V2_NATIVE;
            }
        }
        break;

    case MSP_PAYLOAD_V2_NATIVE:
        msp->checksum2 = crc8_dvb_s2(msp->checksum2, c);
        msp->in_buf[msp->offset++] = c;

        if (msp->offset == msp->data_size) {
            msp->c_state = MSP_CHECKSUM_V2_NATIVE;
        }
        break;

    case MSP_CHECKSUM_V2_NATIVE:
        if (msp->checksum2 == c) {
            msp->c_state = MSP_COMMAND_RECEIVED;
        } else {
            msp->c_state = MSP_IDLE;
        }
        break;
    }
    return true;
}

#endif //HAL_MSP_ENABLED
