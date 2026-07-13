/*
  Common base for simulated Analog Devices ADIS16xxx SPI IMUs.

  These parts all use a 16-bit-word SPI protocol (read request latches an
  address, the contents come back on the next transfer; writes carry a byte
  per word) and deliver inertial data via a "burst read" full-duplex transfer.
  The register maps, burst framing and data-integrity (8-bit checksum vs
  CRC-32) differ per part and are supplied by subclasses.
*/
#pragma once

#include "SIM_config.h"

#if AP_SIM_ADIS_ENABLED

#include "SIM_SPIDevice.h"

namespace SITL {

class ADIS : public SPIDevice
{
public:
    void update(const class Aircraft &aircraft) override;

    int rdwr(uint8_t count, SPI::spi_ioc_transfer *&data) override;

protected:
    // ---- part-specific behaviour ----

    // first 16-bit word the driver clocks out to trigger a burst read
    virtual uint16_t burst_command() const = 0;

    // assemble a burst response as big-endian wire bytes into out[],
    // returning the number of bytes written
    virtual uint8_t build_burst(uint8_t *out) = 0;

    // paged parts (e.g. ADIS16547) select a register page by writing PAGE_ID (0x00)
    virtual bool is_paged() const { return false; }

    // nominal output data rate; used to size delta-angle/velocity increments
    virtual float nominal_rate_hz() const { return 2000; }

    // ---- helpers for subclasses ----
    void set_reg(uint8_t page, uint8_t addr, uint16_t v) { regs[page][addr>>1] = v; }
    uint16_t get_reg(uint8_t page, uint8_t addr) const { return regs[page][addr>>1]; }

    static uint8_t *put_be16(uint8_t *p, uint16_t v) {
        *p++ = v >> 8;
        *p++ = v & 0xff;
        return p;
    }
    // low word then high word, matching the ADIS LOW/OUT register ordering
    static uint8_t *put_be32_lowhigh(uint8_t *p, uint32_t v) {
        p = put_be16(p, v & 0xffff);
        p = put_be16(p, (v >> 16) & 0xffff);
        return p;
    }
    static uint8_t sum_bytes(const uint8_t *p, uint8_t n) {
        uint8_t s = 0;
        while (n--) {
            s += *p++;
        }
        return s;
    }

    // latest simulated sensor state (SI units), refreshed each update()
    double gyro_radps[3];
    double accel_ms2[3];
    float  temperature_degc = 25;  // reads 25 degC until update() runs
    uint16_t sample_counter;
    float  sample_dt;    // nominal sample period (s), for delta-angle/velocity parts

private:
    static const uint8_t MAX_PAGES = 4;
    uint16_t regs[MAX_PAGES][64];
    uint8_t  current_page;
    uint8_t  pending_read_addr;

    void handle_word_write(uint16_t word);
};

} // namespace SITL

#endif  // AP_SIM_ADIS_ENABLED
