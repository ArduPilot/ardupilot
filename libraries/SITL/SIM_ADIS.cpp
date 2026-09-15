#include "SIM_ADIS.h"

#if AP_SIM_ADIS_ENABLED

#include "SIM_ADIS16470.h"
#include "SIM_ADIS16507.h"
#include "SIM_ADIS16547.h"

#include <SITL/SITL.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <AP_HAL/utility/sparse-endian.h>

#include <stddef.h>
#include <string.h>

using namespace SITL;

void ADIS::update(const class Aircraft &aircraft)
{
    const auto *sitl = AP::sitl();
    if (sitl == nullptr) {
        return;
    }

    gyro_radps[0] = radians(sitl->state.rollRate);
    gyro_radps[1] = radians(sitl->state.pitchRate);
    gyro_radps[2] = radians(sitl->state.yawRate);

    accel_ms2[0] = sitl->state.xAccel;
    accel_ms2[1] = sitl->state.yAccel;
    accel_ms2[2] = sitl->state.zAccel;

    sample_dt = 1.0 / nominal_rate_hz();
    sample_counter++;
}

/*
  apply a single 16-bit write word: bit15 set, addr in [14:8], data byte in [7:0]
 */
void ADIS::handle_word_write(uint16_t word)
{
    const uint8_t addr = (word >> 8) & 0x7f;
    const uint8_t byte = word & 0xff;

    if (is_paged() && addr == 0x00) {
        // PAGE_ID low byte selects the active register page
        current_page = byte & (MAX_PAGES-1);
        return;
    }

    uint16_t &r = regs[current_page][addr>>1];
    if (addr & 1) {
        r = (r & 0x00ff) | (uint16_t(byte) << 8);
    } else {
        r = (r & 0xff00) | byte;
    }
}

int ADIS::rdwr(uint8_t count, SPI::spi_ioc_transfer *&tfrs)
{
    for (uint8_t i=0; i<count; i++) {
        const auto &tfr = tfrs[i];
        const uint8_t *tx = (const uint8_t *)tfr.tx_buf;
        uint8_t *rx = (uint8_t *)tfr.rx_buf;

        if (tx != nullptr && rx != nullptr) {
            // full-duplex transfer: only used for burst reads
            if (tfr.len >= 4 && ((tx[0]<<8) | tx[1]) == burst_command()) {
                uint8_t frame[80];
                uint8_t n = build_burst(frame);
                if (n > tfr.len) {
                    n = tfr.len;
                }
                memcpy(rx, frame, n);
            }
            continue;
        }

        if (tx != nullptr) {
            // a write word, or a read-address latch
            const uint16_t word = (tx[0]<<8) | tx[1];
            if (word & 0x8000) {
                handle_word_write(word);
            } else {
                pending_read_addr = (word >> 8) & 0x7f;
            }
            continue;
        }

        if (rx != nullptr && tfr.len >= 2) {
            // deliver the previously-latched register, big-endian
            const uint16_t v = regs[current_page][pending_read_addr>>1];
            rx[0] = v >> 8;
            rx[1] = v & 0xff;
        }
    }
    return 0;
}

/*
  ADIS16470: 16-bit gyro/accel burst, 8-bit checksum.  Matches the packet in
  AP_InertialSensor_ADIS1647x::read_sensor16().
 */
uint8_t ADIS16470::build_burst(uint8_t *out)
{
    // driver decode: gyro rad/s = raw*radians(0.1); accel m/s^2 = raw*1.25*g*0.001
    const double gyro_lsb  = 1.0 / (0.1 * DEG_TO_RAD);
    const double accel_lsb = 1.0 / (1.25 * GRAVITY_MSS * 0.001);

    const int16_t gx = gyro_radps[0]  * gyro_lsb;
    const int16_t gy = gyro_radps[1]  * gyro_lsb;
    const int16_t gz = gyro_radps[2]  * gyro_lsb;
    const int16_t ax = accel_ms2[0]   * accel_lsb;
    const int16_t ay = accel_ms2[1]   * accel_lsb;
    const int16_t az = accel_ms2[2]   * accel_lsb;
    const int16_t temp = temperature_degc / 0.1;   // driver: degC = raw*0.1

    // 16-bit "basic" burst frame, big-endian on the wire
    struct PACKED Burst {
        uint16_t cmd_echo;   // dummy word echoed for the burst command
        uint16_t diag_stat;
        uint16_t gx, gy, gz;
        uint16_t ax, ay, az;
        uint16_t temp;
        uint16_t counter;    // DATA_CNT
        uint8_t  pad;
        uint8_t  checksum;
    } frame {
        .cmd_echo = 0,
        .diag_stat = 0,
        .gx = htobe16(uint16_t(gx)),
        .gy = htobe16(uint16_t(gy)),
        .gz = htobe16(uint16_t(gz)),
        .ax = htobe16(uint16_t(ax)),
        .ay = htobe16(uint16_t(ay)),
        .az = htobe16(uint16_t(az)),
        .temp = htobe16(uint16_t(temp)),
        .counter = htobe16(sample_counter),
        .pad = 0,
        .checksum = 0,
    };
    // 8-bit additive checksum over DIAG_STAT..DATA_CNT
    frame.checksum = sum_bytes((const uint8_t *)&frame.diag_stat,
                               offsetof(Burst, pad) - offsetof(Burst, diag_stat));

    memcpy(out, &frame, sizeof(frame));
    return sizeof(frame);
}

/*
  ADIS16507: 32-bit delta-angle/delta-velocity burst, 8-bit checksum.  Matches
  AP_InertialSensor_ADIS1647x::read_sensor32_delta() (which negates the y and z
  axes on decode, so we negate them here).
 */
uint8_t ADIS16507::build_burst(uint8_t *out)
{
    // driver scales for the +/-125 deg/s (RANG_MDL==0) variant
    const double dangle_scale = 360.0 * DEG_TO_RAD / 2147483647.0;   // rad per LSB
    const double dvel_scale   = 400.0 / 2147483647.0;               // m/s per LSB

    const int32_t dax =  int32_t((gyro_radps[0] * sample_dt) / dangle_scale);
    const int32_t day = -int32_t((gyro_radps[1] * sample_dt) / dangle_scale);
    const int32_t daz = -int32_t((gyro_radps[2] * sample_dt) / dangle_scale);
    const int32_t dvx =  int32_t((accel_ms2[0]  * sample_dt) / dvel_scale);
    const int32_t dvy = -int32_t((accel_ms2[1]  * sample_dt) / dvel_scale);
    const int32_t dvz = -int32_t((accel_ms2[2]  * sample_dt) / dvel_scale);
    const int16_t temp = temperature_degc / 0.1;

    // 32-bit delta burst frame; each 32-bit value is sent low word then high
    // word, all big-endian, matching read_sensor32_delta()
    struct PACKED Burst {
        uint16_t cmd_echo;   // dummy word
        uint16_t diag_stat;
        uint16_t dax_low, dax_high;
        uint16_t day_low, day_high;
        uint16_t daz_low, daz_high;
        uint16_t dvx_low, dvx_high;
        uint16_t dvy_low, dvy_high;
        uint16_t dvz_low, dvz_high;
        uint16_t temp;
        uint16_t counter;    // DATA_CNT
        uint8_t  pad;
        uint8_t  checksum;
    } frame {
        .cmd_echo = 0,
        .diag_stat = 0,
        .dax_low = htobe16(uint16_t(dax)), .dax_high = htobe16(uint16_t(dax >> 16)),
        .day_low = htobe16(uint16_t(day)), .day_high = htobe16(uint16_t(day >> 16)),
        .daz_low = htobe16(uint16_t(daz)), .daz_high = htobe16(uint16_t(daz >> 16)),
        .dvx_low = htobe16(uint16_t(dvx)), .dvx_high = htobe16(uint16_t(dvx >> 16)),
        .dvy_low = htobe16(uint16_t(dvy)), .dvy_high = htobe16(uint16_t(dvy >> 16)),
        .dvz_low = htobe16(uint16_t(dvz)), .dvz_high = htobe16(uint16_t(dvz >> 16)),
        .temp = htobe16(uint16_t(temp)),
        .counter = htobe16(sample_counter),
        .pad = 0,
        .checksum = 0,
    };
    // 8-bit additive checksum over DIAG_STAT..DATA_CNT
    frame.checksum = sum_bytes((const uint8_t *)&frame.diag_stat,
                               offsetof(Burst, pad) - offsetof(Burst, diag_stat));

    memcpy(out, &frame, sizeof(frame));
    return sizeof(frame);
}

/*
  CRC-32 exactly as specified in the ADIS16545/16547 datasheet "CRC-32 Code
  Example": the IEEE-802.3 reflected polynomial (matching AP_Math crc_crc32),
  computed byte-by-byte low-then-high for each 16-bit word, initialised to
  0xFFFFFFFF and finished with a 0xFFFFFFFF XOR.  The BURST_ID words are not
  part of the CRC; it covers STATUS..DATA_CNT.
 */
static uint32_t adis_burst_crc32(const uint16_t *words, uint8_t n)
{
    uint32_t crc = 0xFFFFFFFFUL;
    for (uint8_t i=0; i<n; i++) {
        const uint8_t lo = words[i] & 0xff;
        const uint8_t hi = words[i] >> 8;
        crc = crc_crc32(crc, &lo, 1);
        crc = crc_crc32(crc, &hi, 1);
    }
    return crc ^ 0xFFFFFFFFUL;
}

/*
  ADIS16547: paged, 32-bit delta burst prefixed by BURST_ID (0xC3C3),
  protected by CRC-32 over STATUS..DATA_CNT.  Layout is datasheet Table 16
  (CONFIG bit 8 = 1); it is decoded by
  AP_InertialSensor_ADIS1654x::read_sensor32_delta_crc().
 */
uint8_t ADIS16547::build_burst(uint8_t *out)
{
    // RANG_MDL bits[3:0]==0x3 => model -1: +/-125 deg/s, +/-360 deg delta angle
    const double dangle_scale = 360.0 * DEG_TO_RAD / 2147483647.0;
    const double dvel_scale   = 400.0 / 2147483647.0;              // ADIS16547: +/-400 m/s

    const int32_t dax =  int32_t((gyro_radps[0] * sample_dt) / dangle_scale);
    const int32_t day = -int32_t((gyro_radps[1] * sample_dt) / dangle_scale);
    const int32_t daz = -int32_t((gyro_radps[2] * sample_dt) / dangle_scale);
    const int32_t dvx =  int32_t((accel_ms2[0]  * sample_dt) / dvel_scale);
    const int32_t dvy = -int32_t((accel_ms2[1]  * sample_dt) / dvel_scale);
    const int32_t dvz = -int32_t((accel_ms2[2]  * sample_dt) / dvel_scale);
    // TEMP_OUT: 0x0000 at 25 degC, 140 LSB/degC
    const int16_t temp = int16_t((temperature_degc - 25.0f) * 140.0f);

    // the 15 words the CRC-32 covers (STATUS..DATA_CNT), in wire order;
    // 32-bit quantities are low word first (LR then UR)
    const uint16_t words[15] = {
        0x0000,                                                 // STATUS
        uint16_t(temp),                                         // TEMP_OUT
        uint16_t(dax & 0xffff), uint16_t((dax >> 16) & 0xffff), // X_DELTANG_LR/UR
        uint16_t(day & 0xffff), uint16_t((day >> 16) & 0xffff), // Y_DELTANG_LR/UR
        uint16_t(daz & 0xffff), uint16_t((daz >> 16) & 0xffff), // Z_DELTANG_LR/UR
        uint16_t(dvx & 0xffff), uint16_t((dvx >> 16) & 0xffff), // X_DELTVEL_LR/UR
        uint16_t(dvy & 0xffff), uint16_t((dvy >> 16) & 0xffff), // Y_DELTVEL_LR/UR
        uint16_t(dvz & 0xffff), uint16_t((dvz >> 16) & 0xffff), // Z_DELTVEL_LR/UR
        sample_counter,                                         // DATA_CNT
    };

    uint8_t *p = out;
    p = put_be16(p, 0x0000);          // dummy word (echo of BURST_CMD)
    p = put_be16(p, BURST_ID);
    p = put_be16(p, BURST_ID);
    for (uint8_t i=0; i<ARRAY_SIZE(words); i++) {
        p = put_be16(p, words[i]);
    }
    const uint32_t crc = adis_burst_crc32(words, ARRAY_SIZE(words));
    p = put_be16(p, crc & 0xffff);         // CRC_LWR
    p = put_be16(p, (crc >> 16) & 0xffff); // CRC_UPR
    return p - out;
}

#endif  // AP_SIM_ADIS_ENABLED
