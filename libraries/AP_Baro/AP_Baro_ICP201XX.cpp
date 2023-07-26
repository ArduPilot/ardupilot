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

#include "AP_Baro_ICP201XX.h"

#if AP_BARO_ICP201XX_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <utility>

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

#include <utility>
#include <stdio.h>

#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>

#include <AP_InertialSensor/AP_InertialSensor_Invensense_registers.h>

extern const AP_HAL::HAL &hal;

#define ICP201XX_ID             0x63

#define CONVERSION_INTERVAL     25000

#define REG_EMPTY               0x00
#define REG_TRIM1_MSB           0x05
#define REG_TRIM2_LSB           0x06
#define REG_TRIM2_MSB           0x07
#define REG_DEVICE_ID           0x0C
#define REG_OTP_MTP_OTP_CFG1    0xAC
#define REG_OTP_MTP_MR_LSB      0xAD
#define REG_OTP_MTP_MR_MSB      0xAE
#define REG_OTP_MTP_MRA_LSB     0xAF
#define REG_OTP_MTP_MRA_MSB     0xB0
#define REG_OTP_MTP_MRB_LSB     0xB1
#define REG_OTP_MTP_MRB_MSB     0xB2
#define REG_OTP_MTP_OTP_ADDR    0xB5
#define REG_OTP_MTP_OTP_CMD     0xB6
#define REG_OTP_MTP_RD_DATA     0xB8
#define REG_OTP_MTP_OTP_STATUS  0xB9
#define REG_OTP_DEBUG2          0xBC
#define REG_MASTER_LOCK         0xBE
#define REG_OTP_MTP_OTP_STATUS2 0xBF
#define REG_MODE_SELECT         0xC0
#define REG_INTERRUPT_STATUS    0xC1
#define REG_INTERRUPT_MASK      0xC2
#define REG_FIFO_CONFIG         0xC3
#define REG_FIFO_FILL           0xC4
#define REG_SPI_MODE            0xC5
#define REG_PRESS_ABS_LSB       0xC7
#define REG_PRESS_ABS_MSB       0xC8
#define REG_PRESS_DELTA_LSB     0xC9
#define REG_PRESS_DELTA_MSB     0xCA
#define REG_DEVICE_STATUS       0xCD
#define REG_I3C_INFO            0xCE
#define REG_VERSION             0xD3
#define REG_FIFO_BASE           0xFA

/*
  constructor
 */
AP_Baro_ICP201XX::AP_Baro_ICP201XX(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev)
    : AP_Baro_Backend(baro)
    , dev(std::move(_dev))
{
}

AP_Baro_Backend *AP_Baro_ICP201XX::probe(AP_Baro &baro,
                                         AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
{
    if (!dev) {
        return nullptr;
    }
    AP_Baro_ICP201XX *sensor = new AP_Baro_ICP201XX(baro, std::move(dev));
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

bool AP_Baro_ICP201XX::init()
{
    if (!dev) {
        return false;
    }

    dev->get_semaphore()->take_blocking();

    uint8_t id = 0xFF;
    uint8_t ver = 0xFF;
    read_reg(REG_DEVICE_ID, &id);
    read_reg(REG_VERSION, &ver);

    if (id != ICP201XX_ID) {
        goto failed;
    }

    if (ver != 0x00 && ver != 0xB2) {
        goto failed;
    }

    hal.scheduler->delay(10);

    soft_reset();

    if (!boot_sequence()) {
        goto failed;
    }

    if (!configure()) {
        goto failed;
    }

    wait_read();

    dev->set_retries(0);

    instance = _frontend.register_sensor();

    dev->set_device_type(DEVTYPE_BARO_ICP201XX);
    set_bus_id(instance, dev->get_bus_id());

    dev->get_semaphore()->give();

    dev->register_periodic_callback(CONVERSION_INTERVAL/2, FUNCTOR_BIND_MEMBER(&AP_Baro_ICP201XX::timer, void));
    return true;

 failed:
    dev->get_semaphore()->give();
    return false;
}


void AP_Baro_ICP201XX::dummy_reg()
{
    do {
        uint8_t reg = REG_EMPTY;
        uint8_t val = 0;
        dev->transfer(&reg, 1, &val, 1);
    } while (0);
}

bool AP_Baro_ICP201XX::read_reg(uint8_t reg, uint8_t *buf, uint8_t len)
{
    bool ret;
    ret = dev->transfer(&reg, 1, buf, len);
    dummy_reg();
    return ret;
}

bool AP_Baro_ICP201XX::read_reg(uint8_t reg, uint8_t *val)
{
    return read_reg(reg, val, 1);
}

bool AP_Baro_ICP201XX::write_reg(uint8_t reg, uint8_t val)
{
    bool ret;
    uint8_t data[2] = { reg, val };
    ret = dev->transfer(data, sizeof(data), nullptr, 0);
    dummy_reg();
    return ret;
}

void AP_Baro_ICP201XX::soft_reset()
{
    /* Stop the measurement */
    mode_select(0x00);

    hal.scheduler->delay(2);

    /* Flush FIFO */
    flush_fifo();

    /* Mask all interrupts */
    write_reg(REG_FIFO_CONFIG, 0x00);
    write_reg(REG_INTERRUPT_MASK, 0xFF);
}

bool AP_Baro_ICP201XX::mode_select(uint8_t mode)
{
    uint8_t mode_sync_status = 0;

    do {
        read_reg(REG_DEVICE_STATUS, &mode_sync_status, 1);

        if (mode_sync_status & 0x01) {
            break;
        }

        hal.scheduler->delay(1);
    } while (1);

    return write_reg(REG_MODE_SELECT, mode);
}

bool AP_Baro_ICP201XX::read_otp_data(uint8_t addr, uint8_t cmd, uint8_t *val)
{
    uint8_t otp_status = 0xFF;

    /* Write the address content and read command */
    if (!write_reg(REG_OTP_MTP_OTP_ADDR, addr)) {
        return false;
    }

    if (!write_reg(REG_OTP_MTP_OTP_CMD, cmd)) {
        return false;
    }

    /* Wait for the OTP read to finish Monitor otp_status */
    do     {
        read_reg(REG_OTP_MTP_OTP_STATUS, &otp_status);

        if (otp_status == 0) {
            break;
        }

        hal.scheduler->delay_microseconds(1);
    } while (1);

    /* Read the data from register */
    if (!read_reg(REG_OTP_MTP_RD_DATA, val)) {
        return false;
    }

    return true;
}

bool AP_Baro_ICP201XX::get_sensor_data(float *pressure, float *temperature)
{
    uint8_t fifo_data[96] {0};
    uint8_t fifo_packets = 0;
    int32_t data_temp = 0;
    int32_t data_press = 0;
    *pressure = 0;
    *temperature = 0;

    if (read_reg(REG_FIFO_FILL, &fifo_packets)) {
        fifo_packets  = (uint8_t)(fifo_packets & 0x1F);
        if (fifo_packets > 16) {
            flush_fifo();
            return false;
        }
        if (fifo_packets > 0 && fifo_packets <= 16 && read_reg(REG_FIFO_BASE, fifo_data, fifo_packets * 2 * 3)) {
            uint8_t offset = 0;

            for (uint8_t i = 0; i < fifo_packets; i++) {
                data_press = (int32_t)(((fifo_data[offset + 2] & 0x0f) << 16) | (fifo_data[offset + 1] << 8) | fifo_data[offset]);
                if (data_press & 0x080000) {
                    data_press |= 0xFFF00000;
                }
                /* P = (POUT/2^17)*40kPa + 70kPa */
                *pressure += ((float)(data_press) * 40 / 131072) + 70;
                offset += 3;

                data_temp = (int32_t)(((fifo_data[offset + 2] & 0x0f) << 16) | (fifo_data[offset + 1] << 8) | fifo_data[offset]);
                if (data_temp & 0x080000) {
                    data_temp |= 0xFFF00000;
                }
                /* T = (TOUT/2^18)*65C + 25C */
                *temperature += ((float)(data_temp) * 65 / 262144) + 25;
                offset += 3;
            }

            *pressure = *pressure * 1000 / fifo_packets;
            *temperature = *temperature / fifo_packets;
            return true;
        }
    }

    return false;
}

bool AP_Baro_ICP201XX::boot_sequence()
{
    uint8_t reg_value = 0;
    uint8_t offset = 0, gain = 0, Hfosc = 0;
    uint8_t version = 0;
    uint8_t bootup_status = 0;
    int ret = 1;

    /*  read version register */
    if (!read_reg(REG_VERSION, &version)) {
        return false;
    }

    if (version == 0xB2) {
        /* B2 version Asic is detected. Boot up sequence is not required for B2 Asic, so returning */
        return true;
    }

    /* Read boot up status and avoid re running boot up sequence if it is already done */
    if (!read_reg(REG_OTP_MTP_OTP_STATUS2, &bootup_status)) {
        return false;
    }

    if (bootup_status & 0x01) {
        /* Boot up sequence is already done, not required to repeat boot up sequence */
        return true;
    }

    /* Bring the ASIC in power mode to activate the OTP power domain and get access to the main registers */
    mode_select(0x04);
    hal.scheduler->delay(4);

    /* Unlock the main registers */
    write_reg(REG_MASTER_LOCK, 0x1F);

    /* Enable the OTP and the write switch */
    read_reg(REG_OTP_MTP_OTP_CFG1, &reg_value);
    reg_value |= 0x03;
    write_reg(REG_OTP_MTP_OTP_CFG1, reg_value);
    hal.scheduler->delay_microseconds(10);

    /* Toggle the OTP reset pin */
    read_reg(REG_OTP_DEBUG2, &reg_value);
    reg_value |= 1 << 7;
    write_reg(REG_OTP_DEBUG2, reg_value);
    hal.scheduler->delay_microseconds(10);

    read_reg(REG_OTP_DEBUG2, &reg_value);
    reg_value &= ~(1 << 7);
    write_reg(REG_OTP_DEBUG2, reg_value);
    hal.scheduler->delay_microseconds(10);

    /* Program redundant read */
    write_reg(REG_OTP_MTP_MRA_LSB, 0x04);
    write_reg(REG_OTP_MTP_MRA_MSB, 0x04);
    write_reg(REG_OTP_MTP_MRB_LSB, 0x21);
    write_reg(REG_OTP_MTP_MRB_MSB, 0x20);
    write_reg(REG_OTP_MTP_MR_LSB, 0x10);
    write_reg(REG_OTP_MTP_MR_MSB, 0x80);

    /* Read the data from register */
    ret &= read_otp_data(0xF8, 0x10, &offset);
    ret &= read_otp_data(0xF9, 0x10, &gain);
    ret &= read_otp_data(0xFA, 0x10, &Hfosc);
    hal.scheduler->delay_microseconds(10);

    /* Write OTP values to main registers */
    ret &= read_reg(REG_TRIM1_MSB, &reg_value);
    if (ret) {
        reg_value = (reg_value & (~0x3F)) | (offset & 0x3F);
        ret &= write_reg(REG_TRIM1_MSB, reg_value);
    }

    ret &= read_reg(REG_TRIM2_MSB, &reg_value);
    if (ret) {
        reg_value = (reg_value & (~0x70)) | ((gain & 0x07) << 4);
        ret &= write_reg(REG_TRIM2_MSB, reg_value);
    }

    ret &= read_reg(REG_TRIM2_LSB, &reg_value);
    if (ret) {
        reg_value = (reg_value & (~0x7F)) | (Hfosc & 0x7F);
        ret &= write_reg(REG_TRIM2_LSB, reg_value);
    }

    hal.scheduler->delay_microseconds(10);

    /* Update boot up status to 1 */
    if (ret) {
        ret &= read_reg(REG_OTP_MTP_OTP_STATUS2, &reg_value);
        if (!ret) {
            reg_value |= 0x01;
            ret &= write_reg(REG_OTP_MTP_OTP_STATUS2, reg_value);
        }
    }

    /* Disable OTP and write switch */
    read_reg(REG_OTP_MTP_OTP_CFG1, &reg_value);
    reg_value &= ~0x03;
    write_reg(REG_OTP_MTP_OTP_CFG1, reg_value);

    /* Lock the main register */
    write_reg(REG_MASTER_LOCK, 0x00);

    /* Move to standby */
    mode_select(0x00);

    return ret;
}

bool AP_Baro_ICP201XX::configure()
{
    uint8_t reg_value = 0;

    /* Initiate Triggered Operation: Stay in Standby mode */
    reg_value |= (reg_value & (~0x10)) | ((uint8_t)_forced_meas_trigger << 4);

    /* Power Mode Selection: Normal Mode */
    reg_value |= (reg_value & (~0x04)) | ((uint8_t)_power_mode << 2);

    /* FIFO Readout Mode Selection: Pressure first. */
    reg_value |= (reg_value & (~0x03)) | ((uint8_t)(_fifo_readout_mode));

    /* Measurement Configuration: Mode2*/
    reg_value |= (reg_value & (~0xE0)) | (((uint8_t)_op_mode) << 5);

    /* Measurement Mode Selection: Continuous Measurements (duty cycled) */
    reg_value |= (reg_value & (~0x08)) | ((uint8_t)_meas_mode << 3);

    return mode_select(reg_value);
}

void AP_Baro_ICP201XX::wait_read()
{
    /*
    * If FIR filter is enabled, it will cause a settling effect on the first 14 pressure values.
    * Therefore the first 14 pressure output values are discarded.
    **/
    uint8_t fifo_packets = 0;
    uint8_t fifo_packets_to_skip = 14;

    do {
        hal.scheduler->delay(10);
        read_reg(REG_FIFO_FILL, &fifo_packets);
        fifo_packets = (uint8_t)(fifo_packets & 0x1F);
    } while (fifo_packets >= fifo_packets_to_skip);

    flush_fifo();
    fifo_packets = 0;

    do {
        hal.scheduler->delay(10);
        read_reg(REG_FIFO_FILL, &fifo_packets);
        fifo_packets = (uint8_t)(fifo_packets & 0x1F);
    } while (fifo_packets == 0);
}

bool AP_Baro_ICP201XX::flush_fifo()
{
    uint8_t reg_value;

    if (!read_reg(REG_FIFO_FILL, &reg_value)) {
        return false;
    }

    reg_value |= 0x80;

    if (!write_reg(REG_FIFO_FILL, reg_value)) {
        return false;
    }

    return true;
}

void AP_Baro_ICP201XX::timer()
{
    float p = 0;
    float t = 0;

    if (get_sensor_data(&p, &t)) {
        WITH_SEMAPHORE(_sem);

        accum.psum += p;
        accum.tsum += t;
        accum.count++;
        last_measure_us = AP_HAL::micros();
    } else {
        if (AP_HAL::micros() - last_measure_us > CONVERSION_INTERVAL*3) {
            flush_fifo();
            last_measure_us = AP_HAL::micros();
        }
    }
}

void AP_Baro_ICP201XX::update()
{
    WITH_SEMAPHORE(_sem);

    if (accum.count > 0) {
        _copy_to_frontend(instance, accum.psum/accum.count, accum.tsum/accum.count);
        accum.psum = accum.tsum = 0;
        accum.count = 0;
    }
}

#endif  // AP_BARO_ICP201XX_ENABLED 
