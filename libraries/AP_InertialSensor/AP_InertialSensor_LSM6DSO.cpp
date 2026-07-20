/*
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
/*
 *  driver for ST LSM6DSO IMU
 *
 *  The LSM6DSO shares its register map with the ASM330LHH, so this derives
 *  from that driver and changes only what differs: the WHO_AM_I value, an
 *  ODR selected from INS_FAST_SAMPLE / INS_GYRO_RATE rather than fixed at
 *  3333 Hz, and a burst FIFO read in place of one transfer per sample.
*/

#include "AP_InertialSensor_LSM6DSO.h"
#include "AP_InertialSensor_ASM330_registers.h"

#include <AP_HAL/AP_HAL.h>

#include <stdio.h>
#include <utility>

extern const AP_HAL::HAL& hal;

#define LSM6DSO_WHOAMI                  0x6C

// CTRL9_XL bit1. ST recommend disabling I3C when it is not in use.
#define LSM6DSO_CTRL9_XL_I3C_DISABLE    (1U << 1)

// slowest rate the sensor is run at, and the step of the ODR ladder below
#define LSM6DSO_BASE_RATE_HZ            833

// bytes per FIFO word: one tag byte plus three 16-bit axes
#define LSM6DSO_FIFO_WORD_SIZE          7

// Words per burst transfer. Reading more than one word at a time relies on the
// address auto-increment wrapping from FIFO_DATA_OUT_Z_H (7Eh) back to
// FIFO_DATA_OUT_TAG (78h). The datasheet does not spell that out, but it holds
// on hardware. Setting this to 1 falls back to a transfer per word.
#define LSM6DSO_FIFO_BURST_WORDS        16

// most words drained in a single poll
#define LSM6DSO_FIFO_MAX_DRAIN_WORDS    64

#define LSM6DSO_FIFO_BUFFER_SIZE        (LSM6DSO_FIFO_BURST_WORDS * LSM6DSO_FIFO_WORD_SIZE + 1)

/*
  ODR ladder, stepping in powers of two from the 833 Hz base rate. The
  sensor's rates are not exact multiples of the base, so the true ODR is
  what gets reported to the frontend - the harmonic notch is tuned off it.
 */
static const struct {
    uint16_t rate_hz;
    uint8_t odr;      // CTRL1_XL / CTRL2_G ODR field, pre-shifted
    uint8_t bdr_gy;
    uint8_t bdr_xl;
} odr_table[] = {
    {  833, ASM330_REG_CTRL1_XL_ODR_XL_833Hz,  ASM330_REG_FIFO_CTRL3_BDR_GY_833Hz,  ASM330_REG_FIFO_CTRL3_BDR_XL_833Hz  },
    { 1667, ASM330_REG_CTRL1_XL_ODR_XL_1667Hz, ASM330_REG_FIFO_CTRL3_BDR_GY_1667Hz, ASM330_REG_FIFO_CTRL3_BDR_XL_1667Hz },
    { 3333, ASM330_REG_CTRL1_XL_ODR_XL_3333Hz, ASM330_REG_FIFO_CTRL3_BDR_GY_3333Hz, ASM330_REG_FIFO_CTRL3_BDR_XL_3333Hz },
    { 6667, ASM330_REG_CTRL1_XL_ODR_XL_6667Hz, ASM330_REG_FIFO_CTRL3_BDR_GY_6667Hz, ASM330_REG_FIFO_CTRL3_BDR_XL_6667Hz },
};

AP_InertialSensor_LSM6DSO::AP_InertialSensor_LSM6DSO(AP_InertialSensor &imu,
        AP_HAL::OwnPtr<AP_HAL::Device> device,
        enum Rotation rotation)
    : AP_InertialSensor_ASM330(imu, std::move(device), rotation)
{
    // probe runs at the top of the ladder; start() re-programs the ODR once
    // the fast sampling parameters are known
    odr_index = ARRAY_SIZE(odr_table) - 1;
    backend_rate_hz = odr_table[odr_index].rate_hz;
    backend_period_us = 1000000UL / backend_rate_hz;
}

AP_InertialSensor_LSM6DSO::~AP_InertialSensor_LSM6DSO()
{
    if (fifo_buffer != nullptr) {
        hal.util->free_type(fifo_buffer, LSM6DSO_FIFO_BUFFER_SIZE, AP_HAL::Util::MEM_DMA_SAFE);
    }
}

AP_InertialSensor_Backend *AP_InertialSensor_LSM6DSO::probe(AP_InertialSensor &imu,
        AP_HAL::OwnPtr<AP_HAL::Device> device,
        enum Rotation rotation)
{
    if (!device) {
        return nullptr;
    }

    AP_InertialSensor_LSM6DSO *sensor =
        NEW_NOTHROW AP_InertialSensor_LSM6DSO(imu, std::move(device), rotation);
    if (!sensor || !sensor->init_sensor()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

uint8_t AP_InertialSensor_LSM6DSO::expected_whoami() const
{
    return LSM6DSO_WHOAMI;
}

AP_InertialSensor_Backend::DevTypes AP_InertialSensor_LSM6DSO::devtype() const
{
    return DEVTYPE_INS_LSM6DSO;
}

const char *AP_InertialSensor_LSM6DSO::sensor_name() const
{
    return "LSM6DSO";
}

uint8_t AP_InertialSensor_LSM6DSO::accel_fs_bits() const
{
    return ASM330_REG_CTRL1_XL_FS_XL_16G;
}

uint8_t AP_InertialSensor_LSM6DSO::temperature_decimation() const
{
    return uint8_t(MAX(1U, backend_rate_hz / 100U));
}

/*
  select the ODR from the fast sampling parameters
 */
void AP_InertialSensor_LSM6DSO::set_backend_rate()
{
    uint8_t mult = 1;

    fast_sampling = (dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI) &&
                    enable_fast_sampling(accel_instance) &&
                    get_fast_sampling_rate() > 1;
    if (fast_sampling) {
        mult = get_fast_sampling_rate();
    }

    // never run the sensor slower than the loop rate
    while (LSM6DSO_BASE_RATE_HZ * mult < get_loop_rate_hz() && mult < 8) {
        mult *= 2;
    }

    odr_index = 0;
    while (odr_index + 1U < ARRAY_SIZE(odr_table) && (1U << (odr_index + 1)) <= mult) {
        odr_index++;
    }

    backend_rate_hz = odr_table[odr_index].rate_hz;
    backend_period_us = 1000000UL / backend_rate_hz;
}

void AP_InertialSensor_LSM6DSO::gyro_init()
{
    // CTRL7_G(16h) : 00h - HPF disabled, offset correction bypassed
    register_write(ASM330_REG_CTRL7_G, ASM330_REG_CTRL7_G_HP_EN_G_DISABLE |
                   ASM330_REG_CTRL7_G_HPM_G_16mHz |
                   ASM330_REG_CTRL7_G_USR_OFF_ON_OUT_BYPASS, true);

    // CTRL2_G(11h) : ODR from the ladder, FS = 1100b (2000dps)
    register_write(ASM330_REG_CTRL2_G, odr_table[odr_index].odr |
                   ASM330_REG_CTRL2_G_FS_G_2000DPS, true);
}

void AP_InertialSensor_LSM6DSO::accel_init()
{
    // CTRL8_XL(17h) : 00h - LPF2 and HPF bypassed, XL_FS_MODE = 0 so that
    // FS_XL selects the full +/-16 g range
    register_write(ASM330_REG_CTRL8_XL, ASM330_REG_CTRL8_XL_HPCF_XL_ODR_PER_4 |
                   ASM330_REG_CTRL8_XL_HP_REF_MODE_XL_DISABLE |
                   ASM330_REG_CTRL8_XL_FASTSETTL_MODE_XL_DISABLE |
                   ASM330_REG_CTRL8_XL_HP_SLOPE_XL_EN_DISABLE |
                   ASM330_REG_CTRL8_XL_LOW_PASS_ON_6D_LPF1, true);

    // CTRL9_XL(18h) : E2h - reset default plus I3C disabled
    register_write(ASM330_REG_CTRL9_XL, ASM330_REG_CTRL9_XL_DEN_X_ENABLE |
                   ASM330_REG_CTRL9_XL_DEN_Y_ENABLE |
                   ASM330_REG_CTRL9_XL_DEN_Z_ENABLE |
                   ASM330_REG_CTRL9_XL_DEN_XL_G_GYRO |
                   ASM330_REG_CTRL9_XL_DEN_XL_EN_EXT_DISABLE |
                   ASM330_REG_CTRL9_XL_DEN_LH_ACTIVE_LOW |
                   LSM6DSO_CTRL9_XL_I3C_DISABLE, true);

    // CTRL1_XL(10h) : ODR from the ladder, +/-16g, LPF2 disabled
    register_write(ASM330_REG_CTRL1_XL, odr_table[odr_index].odr |
                   accel_fs_bits() |
                   ASM330_REG_CTRL1_XL_LPF2_XL_EN_DISABLE, true);
}

void AP_InertialSensor_LSM6DSO::fifo_init()
{
    // FIFO_CTRL1(07h), FIFO_CTRL2(08h) : 00h - no watermark
    register_write(ASM330_REG_FIFO_CTRL1, 0x00, true);
    register_write(ASM330_REG_FIFO_CTRL2, 0x00, true);

    // FIFO_CTRL3(09h) : batch both sensors at the selected ODR
    register_write(ASM330_REG_FIFO_CTRL3, odr_table[odr_index].bdr_gy |
                   odr_table[odr_index].bdr_xl, true);

    // FIFO_CTRL4(0Ah) : 06h - no timestamp or temperature batching, continuous mode
    register_write(ASM330_REG_FIFO_CTRL4, ASM330_REG_FIFO_CTRL4_DEC_TS_BATCH_NOT_BATCH |
                   ASM330_REG_FIFO_CTRL4_ODR_T_BATCH_NOT_BATCH |
                   ASM330_REG_FIFO_CTRL4_FIFO_MODE_CONT, true);
}

void AP_InertialSensor_LSM6DSO::start(void)
{
    // the fast sampling parameters are per-instance, so the ODR cannot be
    // chosen until the instance numbers are known
    if (!_imu.get_gyro_instance(gyro_instance) ||
        !_imu.get_accel_instance(accel_instance)) {
        return;
    }

    set_backend_rate();

    if (!_imu.register_gyro(gyro_instance, backend_rate_hz, dev->get_bus_id_devtype(devtype())) ||
        !_imu.register_accel(accel_instance, backend_rate_hz, dev->get_bus_id_devtype(devtype()))) {
        return;
    }

    set_accel_orientation(accel_instance, rot);
    set_gyro_orientation(gyro_instance, rot);

    fifo_buffer = (uint8_t *)hal.util->malloc_type(LSM6DSO_FIFO_BUFFER_SIZE, AP_HAL::Util::MEM_DMA_SAFE);
    if (fifo_buffer == nullptr) {
        AP_HAL::panic("LSM6DSO: unable to allocate FIFO buffer");
    }

    {
        WITH_SEMAPHORE(dev->get_semaphore());
        // re-program the ODR, which hardware_init() could only set to the default
        gyro_init();
        accel_init();
        fifo_init();
        fifo_reset();
    }

    periodic_handle = dev->register_periodic_callback(backend_period_us,
                      FUNCTOR_BIND_MEMBER(&AP_InertialSensor_LSM6DSO::poll_data, void));
}

bool AP_InertialSensor_LSM6DSO::get_output_banner(char* banner, uint8_t banner_len)
{
    snprintf(banner, banner_len, "IMU%u: LSM6DSO %s sampling %.1fkHz",
             gyro_instance,
             fast_sampling ? "fast" : "normal",
             backend_rate_hz * 0.001f);
    return true;
}

/*
  read n_words FIFO words into fifo_buffer. The buffer is passed whole to
  the SPI layer so that the DMA-safe allocation is used all the way down.
 */
bool AP_InertialSensor_LSM6DSO::read_fifo_block(uint16_t n_words)
{
    fifo_buffer[0] = ASM330_REG_FIFO_DATA_OUT_TAG | 0x80;
    // the transfer sends as well as receives, make sure it sends zeros
    memset(fifo_buffer + 1, 0, n_words * LSM6DSO_FIFO_WORD_SIZE);

    return dev->transfer_fullduplex(fifo_buffer, n_words * LSM6DSO_FIFO_WORD_SIZE + 1);
}

void AP_InertialSensor_LSM6DSO::poll_data()
{
    uint16_t samples = get_count_fifo_unread_data();

    if (samples > 0) {
        // keep the callback in step with the incoming data
        dev->adjust_periodic_callback(periodic_handle, backend_period_us);
    }
    if (samples > LSM6DSO_FIFO_MAX_DRAIN_WORDS) {
        samples = LSM6DSO_FIFO_MAX_DRAIN_WORDS;
    }

    while (samples > 0) {
        const uint16_t n = MIN(samples, uint16_t(LSM6DSO_FIFO_BURST_WORDS));

        if (!read_fifo_block(n)) {
            DEV_PRINTF("LSM6DSO: error reading fifo data\n");
            _inc_accel_error_count(accel_instance);
            _inc_gyro_error_count(gyro_instance);
            break;
        }

        const uint8_t *word = fifo_buffer + 1;
        for (uint16_t i = 0; i < n; i++, word += LSM6DSO_FIFO_WORD_SIZE) {
            process_fifo_word(word);
        }
        samples -= n;
    }

    poll_housekeeping();
}
