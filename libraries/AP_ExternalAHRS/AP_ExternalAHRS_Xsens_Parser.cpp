#include "AP_ExternalAHRS_Xsens_Parser.h"

#include <cmath>
#include <cstring>

namespace {
uint16_t be16(const uint8_t *data)
{
    return (uint16_t(data[0]) << 8) | data[1];
}

float be_float(const uint8_t *data)
{
    const uint32_t bits = (uint32_t(data[0]) << 24) | (uint32_t(data[1]) << 16) |
                          (uint32_t(data[2]) << 8) | data[3];
    float value;
    static_assert(sizeof(value) == sizeof(bits), "float32 required");
    memcpy(&value, &bits, sizeof(value));
    return value;
}
}

bool AP_ExternalAHRS_Xsens_Parser::feed(uint8_t byte, Sample &sample)
{
    if (count == 0) {
        if (byte == 0xFA) {
            packet[count++] = byte;
        }
        return false;
    }
    packet[count++] = byte;
    if (count == 2 && byte != 0xFF) {
        count = byte == 0xFA ? 1 : 0;
        return false;
    }
    if (count == 4) {
        // Our configured MTData2 sample is smaller than 255 bytes. Extended
        // packets are discarded rather than accepting an unbounded payload.
        if (byte == 0xFF) {
            count = 0;
            return false;
        }
        expected = uint16_t(byte) + 5;
    }
    if (expected == 0 || count != expected) {
        return false;
    }
    count = 0;
    expected = 0;
    uint8_t checksum = 0;
    for (uint16_t i = 1; i < uint16_t(packet[3]) + 5; i++) {
        checksum += packet[i];
    }
    return checksum == 0 && packet[2] == 0x36 && decode(sample);
}

bool AP_ExternalAHRS_Xsens_Parser::decode(Sample &sample) const
{
    const uint16_t end = uint16_t(packet[3]) + 4;
    uint16_t pos = 4;
    bool have_quat = false;
    bool have_accel = false;
    bool have_gyro = false;
    bool have_counter = false;
    bool have_mag = false;
    Sample parsed {};
    while (pos < end) {
        if (end - pos < 3) {
            return false;
        }
        const uint16_t id = be16(packet + pos);
        const uint8_t length = packet[pos + 2];
        pos += 3;
        if (length > end - pos) {
            return false;
        }
        const uint8_t *data = packet + pos;
        switch (id) {
        case 0x2014: // NED quaternion, float32
            if (have_quat || length != 16) {
                return false;
            }
            for (uint8_t i = 0; i < 4; i++) {
                parsed.quaternion_wxyz[i] = be_float(data + 4*i);
            }
            have_quat = true;
            break;
        case 0x4020: // calibrated acceleration, m/s^2
        case 0x8020: // calibrated rate of turn, rad/s
            if (length != 12 || (id == 0x4020 ? have_accel : have_gyro)) {
                return false;
            }
            for (uint8_t i = 0; i < 3; i++) {
                (id == 0x4020 ? parsed.accel_mps2 : parsed.gyro_rad_s)[i] = be_float(data + 4*i);
            }
            (id == 0x4020 ? have_accel : have_gyro) = true;
            break;
        case 0xC020: // calibrated magnetic field, arbitrary units
            if (have_mag || length != 12) {
                return false;
            }
            for (uint8_t i = 0; i < 3; i++) {
                parsed.mag_au[i] = be_float(data + 4*i);
            }
            have_mag = true;
            break;
        case 0x1020: // packet counter
            if (have_counter || length != 2) {
                return false;
            }
            parsed.counter = be16(data);
            have_counter = true;
            break;
        default:
            break;
        }
        pos += length;
    }
    if (!(have_quat && have_accel && have_gyro && have_counter)) {
        return false;
    }
    if (have_mag) {
        for (float value : parsed.mag_au) {
            if (!std::isfinite(value)) {
                return false;
            }
        }
        parsed.have_mag = true;
    }
    float norm2 = 0;
    for (float value : parsed.quaternion_wxyz) {
        if (!std::isfinite(value)) {
            return false;
        }
        norm2 += value * value;
    }
    if (norm2 < 0.81f || norm2 > 1.21f) {
        return false;
    }
    for (float value : parsed.accel_mps2) {
        if (!std::isfinite(value)) {
            return false;
        }
    }
    for (float value : parsed.gyro_rad_s) {
        if (!std::isfinite(value)) {
            return false;
        }
    }
    sample = parsed;
    return true;
}
