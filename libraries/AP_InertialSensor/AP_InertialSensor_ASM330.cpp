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
 *  driver for ASM330 IMUs
 *    Supported:
 *      ASM330LHH
*/

#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor_ASM330.h"
#include "AP_InertialSensor_ASM330_registers.h"

#include <utility>

#include <AP_HAL/GPIO.h>

extern const AP_HAL::HAL& hal;

#define GYRO_SAMPLE_RATE    3333    // 3333Hz
#define ACCEL_SAMPLE_RATE   3333    // 3333Hz

AP_InertialSensor_ASM330::AP_InertialSensor_ASM330(AP_InertialSensor &imu,
                                                   AP_HAL::OwnPtr<AP_HAL::Device> device,
                                                   enum Rotation rotation)
    : AP_InertialSensor_Backend(imu)
    , dev(std::move(device))
    , rot(rotation)
{
}

AP_InertialSensor_Backend *AP_InertialSensor_ASM330::probe(AP_InertialSensor &imu,
                                                           AP_HAL::OwnPtr<AP_HAL::Device> device,
                                                           enum Rotation rotation)
{
    if (!device) {
        return nullptr;
    }

    AP_InertialSensor_ASM330 *sensor =
        NEW_NOTHROW AP_InertialSensor_ASM330(imu,std::move(device),
                                             rotation);
    if (!sensor || !sensor->init_sensor()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

bool AP_InertialSensor_ASM330::init_sensor()
{
    bool success = hardware_init();

#if AP_INERTIALSENSOR_AMS330_DEBUG_ENABLED
    dump_registers();
#endif
    return success;
}

bool AP_InertialSensor_ASM330::hardware_init()
{
    WITH_SEMAPHORE(dev->get_semaphore());

    // set flag for reading registers
    dev->set_read_flag(0x80);

    const uint8_t whoami = register_read(ASM330_REG_WHO_AM_I);
    if (whoami != WHO_AM_I) {
        DEV_PRINTF("ASM330: unexpected acc/gyro WHOAMI 0x%x\n", whoami);
        return false;
    }

    dev->set_speed(AP_HAL::Device::SPEED_LOW);

    bool reset_success = false;
    for (uint8_t tries = 0; tries < 5; tries++) {
        if (chip_reset()) {
            common_init();
            fifo_init();
            gyro_init();
            accel_init();

            hal.scheduler->delay(50);

            // if samples == 0 -> FIFO empty
            if (get_count_fifo_unread_data() > 0) {
                reset_success = true;
                break;
            }

#if AP_INERTIALSENSOR_AMS330_DEBUG_ENABLED
            dump_registers();
#endif
        }
    }

    dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    if (!reset_success) {
        DEV_PRINTF("ASM330: Failed to boot ASM330\n");
        return false;
    }

    return true;
}

/*
  start the sensor going
 */
void AP_InertialSensor_ASM330::start(void)
{
    if (!_imu.register_gyro(gyro_instance, GYRO_SAMPLE_RATE, dev->get_bus_id_devtype(DEVTYPE_INS_ASM330)) ||
        !_imu.register_accel(accel_instance, ACCEL_SAMPLE_RATE, dev->get_bus_id_devtype(DEVTYPE_INS_ASM330))) {
        return;
    }

    set_accel_orientation(accel_instance, rot);
    set_gyro_orientation(gyro_instance, rot);

    fifo_reset();

    // start the timer process to read samples
    dev->register_periodic_callback(1000, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_ASM330::poll_data, void));
}

uint8_t AP_InertialSensor_ASM330::register_read(uint8_t reg)
{
    uint8_t val = 0;
    dev->read_registers(reg, &val, 1);
    return val;
}

void AP_InertialSensor_ASM330::register_write(uint8_t reg, uint8_t val, bool checked)
{
    dev->write_register(reg, val, checked);
}

bool AP_InertialSensor_ASM330::chip_reset()
{
    // CTRL3_C(12h) : 01h
    //      SW_RESET = 1b (reset device)
    register_write(ASM330_REG_CTRL3_C, ASM330_REG_CTRL3_C_SW_RESET_RESET);

    for (uint8_t tries = 0; tries < 5; tries++) {
        const uint8_t ctrl3_c = register_read(ASM330_REG_CTRL3_C);
        if ((ctrl3_c & 0x01) == ASM330_REG_CTRL3_C_SW_RESET_NORMAL) {
            return true;
        }

        hal.scheduler->delay(2);
    }

    return false;
}

void AP_InertialSensor_ASM330::fifo_reset()
{
    const uint8_t fifo_ctrl4 = register_read(ASM330_REG_FIFO_CTRL4);

    // FIFO_MODE is Bypass mode
    register_write(ASM330_REG_FIFO_CTRL4, fifo_ctrl4 |
                                           ASM330_REG_FIFO_CTRL4_FIFO_MODE_BYPASS, true);

    // Revert FIFO_MODE
    register_write(ASM330_REG_FIFO_CTRL4, fifo_ctrl4, true);

    notify_accel_fifo_reset(accel_instance);
    notify_gyro_fifo_reset(gyro_instance);
}

void AP_InertialSensor_ASM330::common_init()
{
    // CTRL3_C(12h) : 44h
    //      BOOT      = 0b (Reboots memory content is normal mode)
    //      BDU       = 1b (Block data update Enable)
    //      H_LACTIVE = 0b (interrupt output pins active high)
    //      PP_OD     = 0b (INT1 and INT2 pins push-pull mode)
    //      SIM       = 0b (SPI is 4-wire interface)
    //      IF_INC    = 1b (Register address automatically incremented Enable)
    //      SW_RESET  = 0b (Software reset is normal mode)
    register_write(ASM330_REG_CTRL3_C, ASM330_REG_CTRL3_C_BOOT_NORMAL |
                                       ASM330_REG_CTRL3_C_BDU_ENABLE |
                                       ASM330_REG_CTRL3_C_H_LACTIVE_ACTIVE_HIGH |
                                       ASM330_REG_CTRL3_C_PP_OD_PP |
                                       ASM330_REG_CTRL3_C_SIM_4_WIRE |
                                       ASM330_REG_CTRL3_C_IF_INC_ENABLE |
                                       ASM330_REG_CTRL3_C_SW_RESET_NORMAL, true);

    // CTRL4_C(13h) : 00h
    register_write(ASM330_REG_CTRL4_C, 0x00, true);

    // CTRL5_C(14h) : 00h
    register_write(ASM330_REG_CTRL5_C, 0x00, true);

    // CTRL6_C(15h) : 00h
    register_write(ASM330_REG_CTRL6_C, 0x00, true);

    // CTRL10_C(19h) : 00h
    register_write(ASM330_REG_CTRL10_C, 0x00, true);
}

void AP_InertialSensor_ASM330::fifo_init()
{
    // FIFO_CTRL1(07h) : 00h
    register_write(ASM330_REG_FIFO_CTRL1, 0x00, true);

    // FIFO_CTRL2(08h) : 00h
    register_write(ASM330_REG_FIFO_CTRL2, 0x00, true);

    // FIFO_CTRL3(09h) : 99h
    //      BDR_GY = 1001b (3333Hz)
    //      BDR_XL = 1001b (3333Hz)
    register_write(ASM330_REG_FIFO_CTRL3, ASM330_REG_FIFO_CTRL3_BDR_GY_3333Hz |
                                          ASM330_REG_FIFO_CTRL3_BDR_XL_3333Hz, true);

    // FIFO_CTRL4(0Ah) : 06h
    //      DEC_TS_BATCH = 00b (timestamp not batched)
    //      ODR_T_BATCH  = 00b (temperature not batched)
    //      FIFO_MODE    = 110b (Continuous mode)
    register_write(ASM330_REG_FIFO_CTRL4, ASM330_REG_FIFO_CTRL4_DEC_TS_BATCH_NOT_BATCH |
                                          ASM330_REG_FIFO_CTRL4_ODR_T_BATCH_NOT_BATCH |
                                          ASM330_REG_FIFO_CTRL4_FIFO_MODE_CONT, true);
}

void AP_InertialSensor_ASM330::gyro_init()
{
    // CTRL7_G(16h) : 00h
    //      HP_EN_G        = 0b (HPF disabled)
    //      HPM_G          = 00b (HPF cutoff 16mHz)
    //      USR_OFF_ON_OUT = 0b (accelerometer user offset correction block bypassed)
    register_write(ASM330_REG_CTRL7_G, ASM330_REG_CTRL7_G_HP_EN_G_DISABLE |
                                       ASM330_REG_CTRL7_G_HPM_G_16mHz |
                                       ASM330_REG_CTRL7_G_USR_OFF_ON_OUT_BYPASS, true);

    // CTRL2_G(11h) : 9Ch
    //      ODR_XL  = 1001b (3333Hz (high performance))
    //      FS      = 1100b (2000dps)
    register_write(ASM330_REG_CTRL2_G, ASM330_REG_CTRL2_G_ODR_G_3333Hz |
                                       ASM330_REG_CTRL2_G_FS_G_2000DPS, true);
}

void AP_InertialSensor_ASM330::accel_init()
{
    // CTRL8_XL(17h) : 00h
    //      HPCF_XL           = 000b (ODR / 4)
    //      HP_REF_MODE_XL    = 0b (HPF Reference Mode Disable)
    //      FASTSETTL_MODE_XL = 0b (LPF2 and HPF fast-setting Mode Disable)
    //      HP_SLOPE_XL_EN    = 0b (Slope filter / HPF Disable)
    //      LOW_PASS_ON_6D    = 0b (ODR/2 low-pass filtered data sent to 6D interrupt function)
    register_write(ASM330_REG_CTRL8_XL, ASM330_REG_CTRL8_XL_HPCF_XL_ODR_PER_4 |
                                        ASM330_REG_CTRL8_XL_HP_REF_MODE_XL_DISABLE |
                                        ASM330_REG_CTRL8_XL_FASTSETTL_MODE_XL_DISABLE |
                                        ASM330_REG_CTRL8_XL_HP_SLOPE_XL_EN_DISABLE |
                                        ASM330_REG_CTRL8_XL_LOW_PASS_ON_6D_LPF1, true);

    // CTRL9_XL(18h) : E0h
    //      DEN_X       = 1b (DEN stored in X-axis LSB)
    //      DEN_Y       = 1b (DEN stored in Y-axis LSB)
    //      DEN_Z       = 1b (DEN stored in Z-axis LSB)
    //      DEN_XL_G    = 0b (DEN pin info stamped in the gyroscope axis)
    //      DEN_XL_EN   = 0b (Extends DEN functionality to accelerometer sensor Disabled)
    //      DEN_LH      = 0b (active low)
    register_write(ASM330_REG_CTRL9_XL, ASM330_REG_CTRL9_XL_DEN_X_ENABLE |
                                        ASM330_REG_CTRL9_XL_DEN_Y_ENABLE |
                                        ASM330_REG_CTRL9_XL_DEN_Z_ENABLE |
                                        ASM330_REG_CTRL9_XL_DEN_XL_G_GYRO |
                                        ASM330_REG_CTRL9_XL_DEN_XL_EN_EXT_DISABLE |
                                        ASM330_REG_CTRL9_XL_DEN_LH_ACTIVE_LOW, true);

    // CTRL1_XL(10h) :  94h
    //      ODR_XL      = 1001b (3333Hz (high performance))
    //      FS_XL       = 01b (16g)
    //      LPF2_XL_EN  = 0b (LPF2 Disable)
    register_write(ASM330_REG_CTRL1_XL, ASM330_REG_CTRL1_XL_ODR_XL_3333Hz |
                                        ASM330_REG_CTRL1_XL_FS_XL_16G |
                                        ASM330_REG_CTRL1_XL_LPF2_XL_EN_DISABLE, true);
}

uint16_t AP_InertialSensor_ASM330::get_count_fifo_unread_data()
{
    const uint8_t reg = ASM330_REG_FIFO_STATUS1 | 0x80;
    uint16_t tmp;
    if (!dev->transfer(&reg, 1, (uint8_t *)&tmp, sizeof(tmp))) {
        DEV_PRINTF("ASM330: error reading fifo status\n");
        return 0;
    }
    // sample count is in bottom n bits
    return (tmp & 0x03FF);
}

/**
 * Timer process to poll for new data from the ASM330.
 */
void AP_InertialSensor_ASM330::poll_data()
{
    const uint16_t samples = get_count_fifo_unread_data();
    for (uint16_t i = 0; i < samples; i++) {
        const uint8_t fifo_reg = ASM330_REG_FIFO_DATA_OUT_TAG | 0x80;
        uint8_t fifo_tmp[7] = {0, 0, 0, 0, 0, 0, 0};

        if (!dev->transfer(&fifo_reg, 1, (uint8_t *)&fifo_tmp, sizeof(fifo_tmp))) {
            DEV_PRINTF("ASM330: error reading fifo data\n");
            return;
        }

        struct sensor_raw_data raw_data;

        const uint8_t tag = (uint8_t)((fifo_tmp[0] & 0xF8) >> 3);
        raw_data.x = (int16_t)(fifo_tmp[1] + (fifo_tmp[2] << 8));
        raw_data.y = (int16_t)(fifo_tmp[3] + (fifo_tmp[4] << 8));
        raw_data.z = (int16_t)(fifo_tmp[5] + (fifo_tmp[6] << 8));

        switch (tag) {
        case 0x01:
            update_transaction_g(raw_data);
            break;
        case 0x02:
            update_transaction_x(raw_data);
            break;
        default:
            // unused fifo data
            ;
            break;
        }
    }

    if (temperature_counter++ >= 10) {
        const uint8_t temperature_reg = ASM330_REG_OUT_TEMP_L | 0x80;
        int16_t temperature_tmp;
        if (dev->transfer(&temperature_reg, 1, (uint8_t *)&temperature_tmp, sizeof(temperature_tmp))) {
            temperature_degc = temperature_filter.apply((float)temperature_tmp / 256.0f + 25.0f);
        } else {
            DEV_PRINTF("ASM330: error reading temperature data\n");
        }
        temperature_counter = 0;
    }

    // check next register value for correctness
    AP_HAL::Device::checkreg reg;
    if (!dev->check_next_register(reg)) {
        log_register_change(dev->get_bus_id(), reg);
        _inc_accel_error_count(accel_instance);
    }
}

/*
 *  update raw data
 */
void AP_InertialSensor_ASM330::update_transaction_g(struct sensor_raw_data raw_data)
{
    Vector3f gyro_data(raw_data.x, -raw_data.y, -raw_data.z);
    gyro_data *= gyro_scale;

    _rotate_and_correct_gyro(gyro_instance, gyro_data);
    _notify_new_gyro_raw_sample(gyro_instance, gyro_data);
}

void AP_InertialSensor_ASM330::update_transaction_x(struct sensor_raw_data raw_data)
{
    Vector3f accel_data(raw_data.x, -raw_data.y, -raw_data.z);
    accel_data *= accel_scale;

    _rotate_and_correct_accel(accel_instance, accel_data);
    _notify_new_accel_raw_sample(accel_instance, accel_data);
}

bool AP_InertialSensor_ASM330::update()
{
    update_gyro(gyro_instance);
    update_accel(accel_instance);

    _publish_temperature(accel_instance, temperature_degc);
    return true;
}

#if AP_INERTIALSENSOR_AMS330_DEBUG_ENABLED
/*
 *  dump all config registers - used for debug
*/
void AP_InertialSensor_ASM330::dump_registers(void)
{
    hal.console->println("ASM330 registers:");

    const uint8_t first = ASM330_REG_01_RESERVED;
    const uint8_t last = ASM330_REG_FIFO_DATA_OUT_Z_H;
    for (uint8_t reg=first; reg<=last; reg++) {
        uint8_t v = register_read(reg);
        hal.console->printf("%02x:%02x ", reg, v);
        if ((reg - (first-1)) % 16 == 0) {
            hal.console->println("");
        }
    }
    hal.console->println("");
}
#endif
