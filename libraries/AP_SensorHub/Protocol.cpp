#include "Protocol.h"

#if HAL_SENSORHUB_ENABLED

using namespace SensorHub;

Packet::seq_t Packet::sequence_counter = 0;

extern const AP_HAL::HAL& hal;

template <class T>
void Packet::encode(packet_t *p)
{
    p->hdr.magic = MAGIC;
    p->hdr.ver   = VERSION;
    p->hdr.id    = T::ID;
    p->hdr.len   = sizeof(typename T::data_t);
    p->hdr.seq   = sequence_counter++;
#if SENSORHUB_DEBUG_TIMESTAMP
    p->hdr.timestamp = AP_HAL::raw_micros();
#endif

    p->crc = crc16_ccitt(reinterpret_cast<uint8_t *>(&p->hdr),
                          sizeof(hdr_t), 0);
    p->crc = crc16_ccitt(p->data, p->hdr.len, p->crc);
}

bool Packet::headerCheck(hdr_t *hdr)
{
    bool ver_check = false;
    bool id_check = false;
    bool len_check = false;

    ver_check = hdr->ver == VERSION;

    if (hdr->id >= MSGID_START && hdr->id <= MSGID_END) {
        id_check = true;
    }

    if (hdr->len == GyroMessage::DATA_LENGTH
        || hdr->len == AccelMessage::DATA_LENGTH
        || hdr->len == CompassMessage::DATA_LENGTH
        || hdr->len == BaroMessage::DATA_LENGTH
        || hdr->len == GPSMessage::DATA_LENGTH) {

        len_check = true;
    }

    return id_check && len_check && ver_check;
}

bool Packet::verify(packet_t *p)
{
    crc_t calc_crc = 0;

    // NOTE: In case of collision we handle whether data is valid in the
    // individual message's handler.
    calc_crc = crc16_ccitt(reinterpret_cast<uint8_t *>(&p->hdr),
                            sizeof(hdr_t), 0);
    calc_crc = crc16_ccitt(p->data, p->hdr.len, calc_crc);

    return calc_crc == p->crc;
}

int Packet::decode(packet_t *p, uint8_t *buf, size_t len, size_t &decoded_len)
{

    // Assumes packet & data in buf. (eg. HDR | DATA | CRC)

    bool verified = false;
    size_t remainingBytes = len;

    for (int i = 0; i < static_cast<int>(len); ) {
        // Find first instance of MAGIC & assume it is the beginning of a packet.
        i = findMagic(buf, i, remainingBytes);
        if (i < 0) {
            // MAGIC not found. It is safe for the caller to advance the buffer
            // (or flush).
            decoded_len = len;
            return static_cast<int>(decode_t::FAIL_ADV);
        }

        // The number of remaining bytes from the offset where MAGIC is to the
        // end of the buffer.
        remainingBytes = len - i;

        if (remainingBytes < EMPTY_PACKET_LEN) {
            // The caller should continue adding bytes to the buffer. The packet
            // is likely incomplete.
            return static_cast<int>(decode_t::FAIL_CON);
        }

        uint8_t *packetStart = buf + i;

        // We have enough bytes for at least an empty packet. We assume MAGIC marks the
        // beginning of a packet.
        memcpy(&p->hdr, packetStart, sizeof(hdr_t));

        if (!headerCheck(&p->hdr)) {
            // If the header doesn't seem to contain valid data then the
            // packetStart is likely wrong. Search for another MAGIC.
            i++;
            continue;
        }

        if (remainingBytes < EMPTY_PACKET_LEN + p->hdr.len) {
            // We need to wait for more data to be put into the buffer.
            return static_cast<int>(decode_t::FAIL_CON);
        }

        // The buffer contains enough data for the decoded length in header.
        p->data        = packetStart + sizeof(hdr_t); //Don't copy data yet.
        memcpy(&p->crc, packetStart + sizeof(hdr_t) +
               p->hdr.len, sizeof(crc_t));

        // Verify it is a valid packet.
        verified = Packet::verify(p);
        if (verified) {
            // A packet is successfully decoded.
            // We can advance by offset to the packetStart + packet length.
            decoded_len = i + Packet::length(p);
            return static_cast<int>(decode_t::SUCCESS);
        }

        i++;
    }

    // We didn't find anything. It is safe for the caller to advance the buffer
    // (or flush).
    decoded_len = len;
    return static_cast<int>(decode_t::FAIL_ADV);
}

int Packet::findMagic(uint8_t *buf, size_t offset, size_t len)
{

    // Don't bother searching
    if (len < EMPTY_PACKET_LEN) {
        return -1;
    }

    uint8_t byte = 0;

    for (size_t i = offset; i < len; i++) {
        byte = *(buf + i);
        if (byte == MAGIC) {
            return i;
        }
    }

    return -1;
}


/* Template Specializations */
template void Packet::encode<BaroMessage>(packet_t *p);
template void Packet::encode<GyroMessage>(packet_t *p);
template void Packet::encode<AccelMessage>(packet_t *p);
template void Packet::encode<CompassMessage>(packet_t *p);
template void Packet::encode<GPSMessage>(packet_t *p);

#endif