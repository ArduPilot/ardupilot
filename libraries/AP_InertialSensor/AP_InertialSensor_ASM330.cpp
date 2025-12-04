/*
 *  This program is free software
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

#define GYRO_SCALE      (G_SCALE_2000DPS)
#define ACCEL_SCALE     (A_SCALE_16G)

AP_InertialSensor_ASM330::AP_InertialSensor_ASM330(AP_InertialSensor &imu,
                                                   AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                                   enum Rotation rotation)
    : AP_InertialSensor_Backend(imu)
    , _dev(std::move(dev))
    , _rotation(rotation)
    , _temp_filter(1000, 1)
{
    _temp_degc = 0.0F;
}

AP_InertialSensor_Backend *AP_InertialSensor_ASM330::probe(AP_InertialSensor &_imu,
                                                           AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                                           enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }

    AP_InertialSensor_ASM330 *sensor =
        NEW_NOTHROW AP_InertialSensor_ASM330(_imu,std::move(dev),
                                                rotation);
    if (!sensor || !sensor->_init_sensor()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

bool AP_InertialSensor_ASM330::_init_sensor()
{
    _spi_sem = _dev->get_semaphore();

    bool success = _hardware_init();

#if ASM330_DEBUG
    _dump_registers();
#endif
    return success;
}

bool AP_InertialSensor_ASM330::_hardware_init()
{
    _spi_sem->take_blocking();

    uint8_t tries;
    uint8_t whoami;

    // set flag for reading registers
    _dev->set_read_flag(0x80);

    whoami = _register_read(ASM330_REG_WHO_AM_I);
    if (whoami != WHO_AM_I) {
        DEV_PRINTF("ASM330: unexpected acc/gyro WHOAMI 0x%x\n", whoami);
        goto fail_whoami;
    }

    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    for (tries = 0; tries < 5; tries++) {
        if (_chip_reset()) {
            _common_init();
            _fifo_init();
            _gyro_init(GYRO_SCALE);
            _accel_init(ACCEL_SCALE);

            _set_gyro_scale(GYRO_SCALE);
            _set_accel_scale(ACCEL_SCALE);

            hal.scheduler->delay(50);

            // if samples == 0 -> FIFO empty
            if (_get_count_fifo_unread_data() > 0) {
                break;
            }

#if ASM330_DEBUG
            _dump_registers();
#endif
        }
    }

    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    if (tries == 5) {
        DEV_PRINTF("Failed to boot ASM330 5 times\n\n");
        goto fail_tries;
    }

    _spi_sem->give();
    return true;

fail_tries:
fail_whoami:
    _spi_sem->give();
    return false;
}

/*
  start the sensor going
 */
void AP_InertialSensor_ASM330::start(void)
{
    if (!_imu.register_gyro(gyro_instance, 3333, _dev->get_bus_id_devtype(DEVTYPE_INS_ASM330)) ||
        !_imu.register_accel(accel_instance, 3333, _dev->get_bus_id_devtype(DEVTYPE_INS_ASM330))) {
        return;
    }

    set_accel_orientation(accel_instance, _rotation);
    set_gyro_orientation(gyro_instance, _rotation);

    _fifo_reset();

    // start the timer process to read samples
    _dev->register_periodic_callback(1000, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_ASM330::_poll_data, void));
}

uint8_t AP_InertialSensor_ASM330::_register_read(uint8_t reg)
{
    uint8_t val = 0;
    _dev->read_registers(reg, &val, 1);
    return val;
}

void AP_InertialSensor_ASM330::_register_write(uint8_t reg, uint8_t val, bool checked)
{
    _dev->write_register(reg, val, checked);
}

bool AP_InertialSensor_ASM330::_chip_reset()
{
    // CTRL3_C(12h) : 01h
    //      SW_RESET = 1b (reset device)
    _register_write(ASM330_REG_CTRL3_C, ASM330_REG_CTRL3_C_SW_RESET_RESET);
    hal.scheduler->delay(1);

    for (int tries = 0; tries < 5; tries++) {
        uint8_t ctrl3_c = _register_read(ASM330_REG_CTRL3_C);
        if ((ctrl3_c & 0x01) == ASM330_REG_CTRL3_C_SW_RESET_NORMAL) {
            return true;
        }

        hal.scheduler->delay(2);
    }

    return false;
}

void AP_InertialSensor_ASM330::_fifo_reset()
{
    uint8_t fifo_ctrl4 = _register_read(ASM330_REG_FIFO_CTRL4);

    // FIFO_MODE is Bypass mode
    _register_write(ASM330_REG_FIFO_CTRL4, fifo_ctrl4 |
                                           ASM330_REG_FIFO_CTRL4_FIFO_MODE_BYPASS);
    hal.scheduler->delay(1);

    // Revert FIFO_MODE
    _register_write(ASM330_REG_FIFO_CTRL4, fifo_ctrl4);
    hal.scheduler->delay(1);

    notify_accel_fifo_reset(accel_instance);
    notify_gyro_fifo_reset(gyro_instance);
}

void AP_InertialSensor_ASM330::_common_init()
{
    // CTRL3_C(12h) : 44h
    //      BOOT      = 0b (Reboots memory content is normal mode)
    //      BDU       = 1b (Block data update Enable)
    //      H_LACTIVE = 0b (interrupt output pins active high)
    //      PP_OD     = 0b (INT1 and INT2 pins push-pull mode)
    //      SIM       = 0b (SPI is 4-wire interface)
    //      IF_INC    = 1b (Register address automatically incremented Enable)
    //      SW_RESET  = 0b (Software reset is normal mode)
    _register_write(ASM330_REG_CTRL3_C, ASM330_REG_CTRL3_C_BOOT_NORMAL |
                                        ASM330_REG_CTRL3_C_BDU_ENABLE |
                                        ASM330_REG_CTRL3_C_H_LACTIVE_ACTIVE_HIGH |
                                        ASM330_REG_CTRL3_C_PP_OD_PP |
                                        ASM330_REG_CTRL3_C_SIM_4_WIRE |
                                        ASM330_REG_CTRL3_C_IF_INC_ENABLE |
                                        ASM330_REG_CTRL3_C_SW_RESET_NORMAL);
    hal.scheduler->delay(1);

    // CTRL4_C(13h) : 00h
    _register_write(ASM330_REG_CTRL4_C, 0x00);
    hal.scheduler->delay(1);

    // CTRL5_C(14h) : 00h
    _register_write(ASM330_REG_CTRL5_C, 0x00);
    hal.scheduler->delay(1);

    // CTRL6_C(15h) : 00h
    _register_write(ASM330_REG_CTRL6_C, 0x00);
    hal.scheduler->delay(1);

    // CTRL10_C(19h) : 00h
    _register_write(ASM330_REG_CTRL10_C, 0x00);
    hal.scheduler->delay(1);
}

void AP_InertialSensor_ASM330::_fifo_init()
{
    // FIFO_CTRL1(07h) : 00h
    _register_write(ASM330_REG_FIFO_CTRL1, 0x00);
    hal.scheduler->delay(1);

    // FIFO_CTRL2(08h) : 00h
    _register_write(ASM330_REG_FIFO_CTRL2, 0x00);
    hal.scheduler->delay(1);

    // FIFO_CTRL3(09h) : 99h
    //      BDR_GY = 1001b (3333Hz)
    //      BDR_XL = 1001b (3333Hz)
    _register_write(ASM330_REG_FIFO_CTRL3, ASM330_REG_FIFO_CTRL3_BDR_GY_3333Hz |
                                           ASM330_REG_FIFO_CTRL3_BDR_XL_3333Hz);
    hal.scheduler->delay(1);

    // FIFO_CTRL4(0Ah) : 06h
    //      DEC_TS_BATCH = 00b (timestamp not batched)
    //      ODR_T_BATCH  = 00b (temperature not batched)
    //      FIFO_MODE    = 110b (Continuous mode)
    _register_write(ASM330_REG_FIFO_CTRL4, ASM330_REG_FIFO_CTRL4_DEC_TS_BATCH_NOT_BATCH |
                                           ASM330_REG_FIFO_CTRL4_ODR_T_BATCH_NOT_BATCH |
                                           ASM330_REG_FIFO_CTRL4_FIFO_MODE_CONT);
    hal.scheduler->delay(1);
}

void AP_InertialSensor_ASM330::_gyro_init(gyro_scale scale)
{
    // CTRL7_G(16h) : 00h
    //      HP_EN_G        = 0b (HPF disabled)
    //      HPM_G          = 00b (HPF cutoff 16mHz)
    //      USR_OFF_ON_OUT = 0b (accelerometer user offset correction block bypassed)
    _register_write(ASM330_REG_CTRL7_G, ASM330_REG_CTRL7_G_HP_EN_G_DISABLE |
                                        ASM330_REG_CTRL7_G_HPM_G_16mHz |
                                        ASM330_REG_CTRL7_G_USR_OFF_ON_OUT_BYPASS);
    hal.scheduler->delay(1);

    // CTRL2_G(11h) : 1001XXX0b
    //      ODR  = 1001b (3333Hz (high performance))
    uint8_t fs_g;
    switch (scale) {
    case G_SCALE_125DPS:
        fs_g = ASM330_REG_CTRL2_G_FS_G_125PS;
        break;
    case G_SCALE_250DPS:
        fs_g = ASM330_REG_CTRL2_G_FS_G_250DPS;
        break;
    case G_SCALE_500DPS:
        fs_g = ASM330_REG_CTRL2_G_FS_G_500DPS;
        break;
    case G_SCALE_1000DPS:
        fs_g = ASM330_REG_CTRL2_G_FS_G_1000DPS;
        break;
    case G_SCALE_2000DPS:
        fs_g = ASM330_REG_CTRL2_G_FS_G_2000DPS;
        break;
    default:
        fs_g = ASM330_REG_CTRL2_G_FS_G_2000DPS;
        break;
    }

    _register_write(ASM330_REG_CTRL2_G, ASM330_REG_CTRL2_G_ODR_G_3333Hz |
                                        fs_g);
    hal.scheduler->delay(1);
}

void AP_InertialSensor_ASM330::_accel_init(accel_scale scale)
{
    // CTRL8_XL(17h) : 02h
    //      HPCF_XL           = 000b (ODR / 4)
    //      HP_REF_MODE_XL    = 0b (HPF Reference Mode Disable)
    //      FASTSETTL_MODE_XL = 0b (LPF2 and HPF fast-setting Mode Disable)
    //      HP_SLOPE_XL_EN    = 0b (Slope filter / HPF Disable)
    //      LOW_PASS_ON_6D    = 0b (ODR/2 low-pass filtered data sent to 6D interrupt function)
    _register_write(ASM330_REG_CTRL8_XL, ASM330_REG_CTRL8_XL_HPCF_XL_ODR_PER_4 |
                                         ASM330_REG_CTRL8_XL_HP_REF_MODE_XL_DISABLE |
                                         ASM330_REG_CTRL8_XL_FASTSETTL_MODE_XL_DISABLE |
                                         ASM330_REG_CTRL8_XL_HP_SLOPE_XL_EN_DISABLE |
                                         ASM330_REG_CTRL8_XL_LOW_PASS_ON_6D_LPF1);
    hal.scheduler->delay(1);

    // CTRL9_XL(18h) : E0h
    //      DEN_X       = 1b (DEN stored in X-axis LSB)
    //      DEN_Y       = 1b (DEN stored in Y-axis LSB)
    //      DEN_Z       = 1b (DEN stored in Z-axis LSB)
    //      DEN_XL_G    = 0b (DEN pin info stamped in the gyroscope axis)
    //      DEN_XL_EN   = 0b (Extends DEN functionality to accelerometer sensor Disabled)
    //      DEN_LH      = 0b (active low)
    _register_write(ASM330_REG_CTRL9_XL, ASM330_REG_CTRL9_XL_DEN_X_ENABLE |
                                         ASM330_REG_CTRL9_XL_DEN_Y_ENABLE |
                                         ASM330_REG_CTRL9_XL_DEN_Z_ENABLE |
                                         ASM330_REG_CTRL9_XL_DEN_XL_G_GYRO |
                                         ASM330_REG_CTRL9_XL_DEN_XL_EN_EXT_DISABLE |
                                         ASM330_REG_CTRL9_XL_DEN_LH_ACTIVE_LOW);
    hal.scheduler->delay(1);

    // CTRL1_XL(10h) :  1001XX00b
    //      ODR        = 1001b (3333Hz (high performance))
    //      LPF2_XL_EN = 0b (LPF2 Disable)
    uint8_t fs_xl;
    switch (scale) {
    case A_SCALE_2G:
        fs_xl = ASM330_REG_CTRL1_XL_FS_XL_2G;
        break;
    case A_SCALE_4G:
        fs_xl = ASM330_REG_CTRL1_XL_FS_XL_4G;
        break;
    case A_SCALE_8G:
        fs_xl = ASM330_REG_CTRL1_XL_FS_XL_8G;
        break;
    case A_SCALE_16G:
        fs_xl = ASM330_REG_CTRL1_XL_FS_XL_16G;
        break;
    default:
        fs_xl = ASM330_REG_CTRL1_XL_FS_XL_16G;
        break;
    }

    _register_write(ASM330_REG_CTRL1_XL, ASM330_REG_CTRL1_XL_ODR_XL_3333Hz |
                                         fs_xl |
                                         ASM330_REG_CTRL1_XL_LPF2_XL_EN_DISABLE);
    hal.scheduler->delay(1);
}

uint16_t AP_InertialSensor_ASM330::_get_count_fifo_unread_data()
{
    const uint8_t _reg = ASM330_REG_FIFO_STATUS1 | 0x80;
    uint16_t tmp;
    uint16_t samples;

    if (_dev->transfer(&_reg, 1, (uint8_t *)&tmp, sizeof(tmp))) {
        samples = (uint16_t)(tmp & 0x03FF);
    } else {
        DEV_PRINTF("ASM330: error reading fifo status\n");
        samples = 0;
    }

    return samples;
}

void AP_InertialSensor_ASM330::_set_gyro_scale(gyro_scale scale)
{
    float scale_val = 0.0f;

    /* scales values from datasheet in mdps/digit */
    switch (scale) {
    case G_SCALE_125DPS:
        scale_val = 4.375f;
        break;
    case G_SCALE_250DPS:
        scale_val = 8.75f;
        break;
    case G_SCALE_500DPS:
        scale_val = 17.5f;
        break;
    case G_SCALE_1000DPS:
        scale_val = 35.0f;
        break;
    case G_SCALE_2000DPS:
        scale_val = 70.0f;
        break;
    default:
        scale_val = 70.0f;
        break;
    }

    /* convert mdps/digit to dps/digit */
    _gyro_scale = scale_val / 1000.0f;

    /* convert dps/digit to (rad/s)/digit */
    _gyro_scale *= DEG_TO_RAD;
}

void AP_InertialSensor_ASM330::_set_accel_scale(accel_scale scale)
{
    float scale_val = 0.0f;

    switch (scale) {
    case A_SCALE_2G:
        scale_val = 2.0f;
        break;
    case A_SCALE_4G:
        scale_val = 4.0f;
        break;
    case A_SCALE_8G:
        scale_val = 8.0f;
        break;
    case A_SCALE_16G:
        scale_val = 16.0f;
        break;
    default:
        scale_val = 16.0f;
        break;
    }

    _accel_scale = scale_val / 32768.0f;

    /* convert to G/LSB to (m/s/s)/LSB */
    _accel_scale *= GRAVITY_MSS;
}

/**
 * Timer process to poll for new data from the ASM330.
 */
void AP_InertialSensor_ASM330::_poll_data()
{
    const uint16_t samples = _get_count_fifo_unread_data();
    for (uint16_t i = 0; i < samples; i++) {
        const uint8_t fifo_reg = ASM330_REG_FIFO_DATA_OUT_TAG | 0x80;
        uint8_t fifo_tmp[7] = {0, 0, 0, 0, 0, 0, 0};

        if (!_dev->transfer(&fifo_reg, 1, (uint8_t *)&fifo_tmp, sizeof(fifo_tmp))) {
            DEV_PRINTF("ASM330: error reading fifo data\n");
            return;
        }

        uint8_t tag;
        struct sensor_raw_data raw_data;

        tag = (uint8_t)((fifo_tmp[0] & 0xF8) >> 3);
        raw_data.x = (int16_t)(fifo_tmp[1] + (fifo_tmp[2] << 8));
        raw_data.y = (int16_t)(fifo_tmp[3] + (fifo_tmp[4] << 8));
        raw_data.z = (int16_t)(fifo_tmp[5] + (fifo_tmp[6] << 8));

        switch (tag) {
        case 0x01:
            _update_transaction_g(raw_data);
            break;
        case 0x02:
            _update_transaction_x(raw_data);
            break;
        default:
            // unused fifo data
            ;
            break;
        }
    }

    const uint8_t temp_reg = ASM330_REG_OUT_TEMP_L | 0x80;
    int16_t temp_tmp;
    if (_dev->transfer(&temp_reg, 1, (uint8_t *)&temp_tmp, sizeof(temp_tmp))) {
        _temp_degc = _temp_filter.apply((float)temp_tmp / 256.0f + 25.0f);
    } else {
        DEV_PRINTF("ASM330: error reading temp data\n");
    }


    // check next register value for correctness
    AP_HAL::Device::checkreg reg;
    if (!_dev->check_next_register(reg)) {
        log_register_change(_dev->get_bus_id(), reg);
        _inc_accel_error_count(accel_instance);
    }
}

/*
 *  update raw data
 */
void AP_InertialSensor_ASM330::_update_transaction_g(struct sensor_raw_data raw_data)
{
    Vector3f gyro_data(raw_data.x, -raw_data.y, -raw_data.z);
    gyro_data *= _gyro_scale;

    _rotate_and_correct_gyro(gyro_instance, gyro_data);
    _notify_new_gyro_raw_sample(gyro_instance, gyro_data);
}

void AP_InertialSensor_ASM330::_update_transaction_x(struct sensor_raw_data raw_data)
{
    Vector3f accel_data(raw_data.x, -raw_data.y, -raw_data.z);
    accel_data *= _accel_scale;

    _rotate_and_correct_accel(accel_instance, accel_data);
    _notify_new_accel_raw_sample(accel_instance, accel_data);
}

bool AP_InertialSensor_ASM330::update()
{
    update_gyro(gyro_instance);
    update_accel(accel_instance);

    _publish_temperature(accel_instance, _temp_degc);
    return true;
}

#if ASM330_DEBUG
/*
 *  dump all config registers - used for debug
*/
void AP_InertialSensor_ASM330::_dump_registers(void)
{
    hal.console->println("ASM330 registers:");

    const uint8_t first = ASM330_REG_FUNC_CFG_ACCESS;
    const uint8_t last = ASM330_REG_FIFO_DATA_OUT_Z_H;
    for (uint8_t reg=first; reg<=last; reg++) {
        uint8_t v = _register_read(reg);
        hal.console->printf("%02x:%02x ", reg, v);
        if ((reg - (first-1)) % 16 == 0) {
            hal.console->println("");
        }
    }
    hal.console->println("");
}
#endif
