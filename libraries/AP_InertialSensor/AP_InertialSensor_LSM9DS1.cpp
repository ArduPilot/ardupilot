/*
 *  This program is free software
*/
#include <AP_HAL/AP_HAL.h>

#include "AP_InertialSensor_LSM9DS1.h"

#include <utility>

#include <AP_HAL/GPIO.h>

extern const AP_HAL::HAL& hal;

#define WHO_AM_I     0x68
#define WHO_AM_I_M   0x3D
#define LSM9DS1_DRY_XG_PIN -1

/*
 *  Accelerometer and Gyroscope registers
*/
#define LSM9DS1XG_ACT_THS                               0x04
#   define LSM9DS1XG_ACT_THS_SLEEP_ON             (0x1 << 7)
#define LSM9DS1XG_ACT_DUR                               0x05
#define LSM9DS1XG_INT_GEN_CFG_XL                        0x06
#   define LSM9DS1XG_INT_GEN_CFG_XL_AOI_XL        (0x1 << 7)
#   define LSM9DS1XG_INT_GEN_CFG_XL_6D            (0x1 << 6)
#   define LSM9DS1XG_INT_GEN_CFG_XL_ZHIE_XL       (0x1 << 5)
#   define LSM9DS1XG_INT_GEN_CFG_XL_ZLIE_XL       (0x1 << 4)
#   define LSM9DS1XG_INT_GEN_CFG_XL_YHIE_XL       (0x1 << 3)
#   define LSM9DS1XG_INT_GEN_CFG_XL_YLIE_XL       (0x1 << 2)
#   define LSM9DS1XG_INT_GEN_CFG_XL_XHIE_XL       (0x1 << 1)
#   define LSM9DS1XG_INT_GEN_CFG_XL_XLIE_XL       (0x1 << 0)
#define LSM9DS1XG_INT_GEN_THS_X_XL                      0x07
#define LSM9DS1XG_INT_GEN_THS_Y_XL                      0x08
#define LSM9DS1XG_INT_GEN_THS_Z_XL                      0x09
#define LSM9DS1XG_INT_GEN_DUR_XL                        0x0A
#   define LSM9DS1XG_INT_GEN_DUR_XL_WAIT_XL       (0x1 << 7)
#define LSM9DS1XG_REFERENCE_G                           0x0B
#define LSM9DS1XG_INT1_CTRL                             0x0C
#   define LSM9DS1XG_INT1_CTRL_INT1_IG_G          (0x1 << 7)
#   define LSM9DS1XG_INT1_CTRL_INT_IG_XL          (0x1 << 6)
#   define LSM9DS1XG_INT1_CTRL_INT_FSS5           (0x1 << 5)
#   define LSM9DS1XG_INT1_CTRL_INT_OVR            (0x1 << 4)
#   define LSM9DS1XG_INT1_CTRL_INT_FTH            (0x1 << 3)
#   define LSM9DS1XG_INT1_CTRL_INT_Boot           (0x1 << 2)
#   define LSM9DS1XG_INT1_CTRL_INT_DRDY_G         (0x1 << 1)
#   define LSM9DS1XG_INT1_CTRL_INT_DRDY_XL        (0x1 << 0)
#define LSM9DS1XG_INT2_CTRL                             0x0D
#   define LSM9DS1XG_INT2_CTRL_INT2_INACT         (0x1 << 7)
#   define LSM9DS1XG_INT2_CTRL_INT2_FSS5          (0x1 << 5)
#   define LSM9DS1XG_INT2_CTRL_INT2_OVR           (0x1 << 4)
#   define LSM9DS1XG_INT2_CTRL_INT2_FTH           (0x1 << 3)
#   define LSM9DS1XG_INT2_CTRL_INT2_DRDY_TEMP     (0x1 << 2)
#   define LSM9DS1XG_INT2_CTRL_INT2_DRDY_G        (0x1 << 1)
#   define LSM9DS1XG_INT2_CTRL_INT2_DRDY_XL       (0x1 << 0)
#define LSM9DS1XG_WHO_AM_I                              0x0F
#define LSM9DS1XG_CTRL_REG1_G                           0x10
#   define LSM9DS1XG_CTRL_REG1_G_ODR_G_14900mHz   (0x1 << 5)
#   define LSM9DS1XG_CTRL_REG1_G_ODR_G_59500mHz   (0x2 << 5)
#   define LSM9DS1XG_CTRL_REG1_G_ODR_G_119Hz      (0x3 << 5)
#   define LSM9DS1XG_CTRL_REG1_G_ODR_G_238Hz      (0x4 << 5)
#   define LSM9DS1XG_CTRL_REG1_G_ODR_G_476Hz      (0x5 << 5)
#   define LSM9DS1XG_CTRL_REG1_G_ODR_G_952Hz      (0x6 << 5)
#   define LSM9DS1XG_CTRL_REG1_FS_G_245DPS        (0x0 << 3)
#   define LSM9DS1XG_CTRL_REG1_FS_G_500DPS        (0x1 << 3)
#   define LSM9DS1XG_CTRL_REG1_FS_G_2000DPS       (0x3 << 3)
#define LSM9DS1XG_CTRL_REG2_G                           0x11
#   define LSM9DS1XG_CTRL_REG2_G_INT_SEL_00       (0x0 << 2)
#   define LSM9DS1XG_CTRL_REG2_G_INT_SEL_01       (0x1 << 2)
#   define LSM9DS1XG_CTRL_REG2_G_INT_SEL_10       (0x2 << 2)
#   define LSM9DS1XG_CTRL_REG2_G_INT_SEL_11       (0x3 << 2)
#   define LSM9DS1XG_CTRL_REG2_G_OUT_SEL_00       (0x0 << 0)
#   define LSM9DS1XG_CTRL_REG2_G_OUT_SEL_01       (0x1 << 0)
#   define LSM9DS1XG_CTRL_REG2_G_OUT_SEL_10       (0x2 << 0)
#   define LSM9DS1XG_CTRL_REG2_G_OUT_SEL_11       (0x3 << 0)
#define LSM9DS1XG_CTRL_REG3_G                           0x12
#   define LSM9DS1XG_CTRL_REG3_G_LP_MODE          (0x1 << 7)
#   define LSM9DS1XG_CTRL_REG3_G_HP_EN            (0x1 << 6)
#define LSM9DS1XG_ORIENT_CFG_G                          0x13
#define LSM9DS1XG_INT_GEN_SRC_G                         0x14
#   define LSM9DS1XG_INT_GEN_SRC_G_IA_G           (0x1 << 6)
#   define LSM9DS1XG_INT_GEN_SRC_G_ZH_G           (0x1 << 5)
#   define LSM9DS1XG_INT_GEN_SRC_G_ZL_G           (0x1 << 4)
#   define LSM9DS1XG_INT_GEN_SRC_G_YH_G           (0x1 << 3)
#   define LSM9DS1XG_INT_GEN_SRC_G_YL_G           (0x1 << 2)
#   define LSM9DS1XG_INT_GEN_SRC_G_XH_G           (0x1 << 1)
#   define LSM9DS1XG_INT_GEN_SRC_G_XL_G           (0x1 << 0)
#define LSM9DS1XG_OUT_TEMP_L                            0x15
#define LSM9DS1XG_OUT_TEMP_H                            0x16
#define LSM9DS1XG_STATUS_REG                            0x17
#   define LSM9DS1XG_STATUS_REG_IG_XL             (0x1 << 6)
#   define LSM9DS1XG_STATUS_REG_IG_G              (0x1 << 5)
#   define LSM9DS1XG_STATUS_REG_INACT             (0x1 << 4)
#   define LSM9DS1XG_STATUS_REG_BOOT_STATUS       (0x1 << 3)
#   define LSM9DS1XG_STATUS_REG_TDA               (0x1 << 2)
#   define LSM9DS1XG_STATUS_REG_GDA               (0x1 << 1)
#   define LSM9DS1XG_STATUS_REG_XLDA              (0x1 << 0)
#define LSM9DS1XG_OUT_X_L_G                             0x18
#define LSM9DS1XG_OUT_X_H_G                             0x19
#define LSM9DS1XG_OUT_Y_L_G                             0x1A
#define LSM9DS1XG_OUT_Y_H_G                             0x1B
#define LSM9DS1XG_OUT_Z_L_G                             0x1C
#define LSM9DS1XG_OUT_Z_H_G                             0x1D
#define LSM9DS1XG_CTRL_REG4                             0x1E
#   define LSM9DS1XG_CTRL_REG4_Zen_G              (0x1 << 5)
#   define LSM9DS1XG_CTRL_REG4_Yen_G              (0x1 << 4)
#   define LSM9DS1XG_CTRL_REG4_Xen_G              (0x1 << 3)
#   define LSM9DS1XG_CTRL_REG4_LIR_XL1            (0x1 << 1)
#   define LSM9DS1XG_CTRL_REG4_4D_XL1             (0x1 << 0)
#define LSM9DS1XG_CTRL_REG5_XL                          0x1F
#   define LSM9DS1XG_CTRL_REG5_XL_Zen_XL          (0x1 << 5)
#   define LSM9DS1XG_CTRL_REG5_XL_Yen_XL          (0x1 << 4)
#   define LSM9DS1XG_CTRL_REG5_XL_Xen_XL          (0x1 << 3)
#define LSM9DS1XG_CTRL_REG6_XL                          0x20
#   define LSM9DS1XG_CTRL_REG6_XL_ODR_XL_10Hz     (0x1 << 5)
#   define LSM9DS1XG_CTRL_REG6_XL_ODR_XL_50Hz     (0x2 << 5)
#   define LSM9DS1XG_CTRL_REG6_XL_ODR_XL_119Hz    (0x3 << 5)
#   define LSM9DS1XG_CTRL_REG6_XL_ODR_XL_238Hz    (0x4 << 5)
#   define LSM9DS1XG_CTRL_REG6_XL_ODR_XL_476Hz    (0x5 << 5)
#   define LSM9DS1XG_CTRL_REG6_XL_ODR_XL_952Hz    (0x6 << 5)
#   define LSM9DS1XG_CTRL_REG6_XL_FS_XL_2G        (0x0 << 3)
#   define LSM9DS1XG_CTRL_REG6_XL_FS_XL_16G       (0x1 << 3)
#   define LSM9DS1XG_CTRL_REG6_XL_FS_XL_4G        (0x2 << 3)
#   define LSM9DS1XG_CTRL_REG6_XL_FS_XL_8G        (0x3 << 3)
#   define LSM9DS1XG_CTRL_REG6_XL_BW_SCAL_ODR     (0x1 << 2)
#   define LSM9DS1XG_CTRL_REG6_XL_BW_XL_408Hz     (0x0 << 0)
#   define LSM9DS1XG_CTRL_REG6_XL_BW_XL_211Hz     (0x1 << 0)
#   define LSM9DS1XG_CTRL_REG6_XL_BW_XL_105Hz     (0x2 << 0)
#   define LSM9DS1XG_CTRL_REG6_XL_BW_XL_50Hz      (0x3 << 0)
#define LSM9DS1XG_CTRL_REG7_XL                          0x21
#   define LSM9DS1XG_CTRL_REG7_XL_FDS             (0x1 << 2)
#   define LSM9DS1XG_CTRL_REG7_XL_HPIS1           (0x1 << 0)
#define LSM9DS1XG_CTRL_REG8                             0x22
#   define LSM9DS1XG_CTRL_REG8_BOOT               (0x1 << 7)
#   define LSM9DS1XG_CTRL_REG8_BDU                (0x1 << 6)
#   define LSM9DS1XG_CTRL_REG8_H_LACTIVE          (0x1 << 5)
#   define LSM9DS1XG_CTRL_REG8_PP_OD              (0x1 << 4)
#   define LSM9DS1XG_CTRL_REG8_SIM                (0x1 << 3)
#   define LSM9DS1XG_CTRL_REG8_IF_ADD_INC         (0x1 << 2)
#   define LSM9DS1XG_CTRL_REG8_BLE                (0x1 << 1)
#   define LSM9DS1XG_CTRL_REG8_SW_RESET           (0x1 << 0)
#define LSM9DS1XG_CTRL_REG9                             0x23
#   define LSM9DS1XG_CTRL_REG9_SLEEP_G            (0x1 << 6)
#   define LSM9DS1XG_CTRL_REG9_FIFO_TEMP_EN       (0x1 << 4)
#   define LSM9DS1XG_CTRL_REG9_DRDY_mask_bit      (0x1 << 3)
#   define LSM9DS1XG_CTRL_REG9_I2C_DISABLE        (0x1 << 2)
#   define LSM9DS1XG_CTRL_REG9_FIFO_EN            (0x1 << 1)
#   define LSM9DS1XG_CTRL_REG9_STOP_ON_FTH        (0x1 << 0)
#define LSM9DS1XG_CTRL_REG10                            0x24
#   define LSM9DS1XG_CTRL_REG10_ST_G              (0x1 << 2)
#   define LSM9DS1XG_CTRL_REG10_ST_XL             (0x1 << 0)
#define LSM9DS1XG_INT_GEN_SRC_XL                        0x26
#   define LSM9DS1XG_INT_GEN_SRC_XL_IA_XL         (0x1 << 6)
#   define LSM9DS1XG_INT_GEN_SRC_XL_ZH_XL         (0x1 << 5)
#   define LSM9DS1XG_INT_GEN_SRC_XL_ZL_XL         (0x1 << 4)
#   define LSM9DS1XG_INT_GEN_SRC_XL_YH_XL         (0x1 << 3)
#   define LSM9DS1XG_INT_GEN_SRC_XL_YL_XL         (0x1 << 2)
#   define LSM9DS1XG_INT_GEN_SRC_XL_XH_XL         (0x1 << 1)
#   define LSM9DS1XG_INT_GEN_SRC_XL_XL_XL         (0x1 << 0)
//#define LSM9DS1XG_STATUS_REG                            0x27  //repeat
#define LSM9DS1XG_OUT_X_L_XL                            0x28
#define LSM9DS1XG_OUT_X_H_XL                            0x29
#define LSM9DS1XG_OUT_Y_L_XL                            0x2A
#define LSM9DS1XG_OUT_Y_H_XL                            0x2B
#define LSM9DS1XG_OUT_Z_L_XL                            0x2C
#define LSM9DS1XG_OUT_Z_H_XL                            0x2D
#define LSM9DS1XG_FIFO_CTRL                             0x2E
#   define LSM9DS1XG_FIFO_CTRL_FMODE_BYPASS       (0x0 << 5)
#   define LSM9DS1XG_FIFO_CTRL_FMODE_FIFO         (0x1 << 5)
#   define LSM9DS1XG_FIFO_CTRL_FMODE_STREAM       (0x3 << 5)
#   define LSM9DS1XG_FIFO_CTRL_FMODE_B_TO_S       (0x4 << 5)
#   define LSM9DS1XG_FIFO_CTRL_FMODE_CON          (0x5 << 5)
#define LSM9DS1XG_FIFO_SRC                              0x2F
#   define LSM9DS1XG_FIFO_SRC_FTH                 (0x1 << 7)
#   define LSM9DS1XG_FIFO_SRC_OVRN                (0x1 << 6)
#   define LSM9DS1XG_FIFO_SRC_UNREAD_SAMPLES            0x3F
#define LSM9DS1XG_INT_GEN_CFG_G                         0x30
#   define LSM9DS1XG_INT_GEN_CFG_G_AOI_G          (0x1 << 7)
#   define LSM9DS1XG_INT_GEN_CFG_G_LIR_G          (0x1 << 6)
#   define LSM9DS1XG_INT_GEN_CFG_G_ZHIE_G         (0x1 << 5)
#   define LSM9DS1XG_INT_GEN_CFG_G_ZLIE_G         (0x1 << 4)
#   define LSM9DS1XG_INT_GEN_CFG_G_YHIE_G         (0x1 << 3)
#   define LSM9DS1XG_INT_GEN_CFG_G_YLIE_G         (0x1 << 2)
#   define LSM9DS1XG_INT_GEN_CFG_G_XHIE_G         (0x1 << 1)
#   define LSM9DS1XG_INT_GEN_CFG_G_XLIE_G         (0x1 << 0)
#define LSM9DS1XG_INT_GEN_THS_XH_G                      0x31
#   define LSM9DS1XG_INT_GEN_THS_XH_G_DCRM_G      (0x1 << 7)
#define LSM9DS1XG_INT_GEN_THS_XL_G                      0x32
#define LSM9DS1XG_INT_GEN_THS_YH_G                      0x33
#define LSM9DS1XG_INT_GEN_THS_YL_G                      0x34
#define LSM9DS1XG_INT_GEN_THS_ZH_G                      0x35
#define LSM9DS1XG_INT_GEN_THS_ZL_G                      0x36
#define LSM9DS1XG_INT_GEN_DUR_G                         0x37
#   define LSM9DS1XG_INT_GEN_DUR_G_WAIT_G         (0x1 << 7)

AP_InertialSensor_LSM9DS1::AP_InertialSensor_LSM9DS1(AP_InertialSensor &imu,
                                                     AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                                     int drdy_pin_num_xg,
                                                     enum Rotation rotation)
    : AP_InertialSensor_Backend(imu)
    , _dev(std::move(dev))
    , _drdy_pin_num_xg(drdy_pin_num_xg)
    , _rotation(rotation)

{
}

AP_InertialSensor_Backend *AP_InertialSensor_LSM9DS1::probe(AP_InertialSensor &_imu,
                                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                                            enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }

    AP_InertialSensor_LSM9DS1 *sensor =
        new AP_InertialSensor_LSM9DS1(_imu,std::move(dev),
                                      LSM9DS1_DRY_XG_PIN,
                                      rotation);
    if (!sensor || !sensor->_init_sensor()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

bool AP_InertialSensor_LSM9DS1::_init_sensor()
{
    _spi_sem = _dev->get_semaphore();

    if (_drdy_pin_num_xg >= 0) {
        _drdy_pin_xg = hal.gpio->channel(_drdy_pin_num_xg);
        if (_drdy_pin_xg == nullptr) {
            AP_HAL::panic("LSM9DS1: null accel data-ready GPIO channel\n");
        }
        _drdy_pin_xg->mode(HAL_GPIO_INPUT);
    }

    bool success = _hardware_init();

#if LSM9DS1_DEBUG
    _dump_registers();
#endif
    return success;
}

bool AP_InertialSensor_LSM9DS1::_hardware_init()
{
    _spi_sem->take_blocking();

    uint8_t tries, whoami;

    // set flag for reading registers
    _dev->set_read_flag(0x80);

    whoami = _register_read(LSM9DS1XG_WHO_AM_I);
    if (whoami != WHO_AM_I) {
        hal.console->printf("LSM9DS1: unexpected acc/gyro WHOAMI 0x%x\n", whoami);
        goto fail_whoami;
    }

    _fifo_reset();

    for (tries = 0; tries < 5; tries++) {

        _dev->set_speed(AP_HAL::Device::SPEED_LOW);

        _gyro_init();
        _accel_init();

        _dev->set_speed(AP_HAL::Device::SPEED_HIGH);

        hal.scheduler->delay(10);

        int samples = _register_read(LSM9DS1XG_FIFO_SRC);

        //if samples == 0 -> FIFO empty
        if (samples > 1) {
            break;
        }

#if LSM9DS1_DEBUG
        _dump_registers();
#endif
    }
    if (tries == 5) {
        hal.console->printf("Failed to boot LSM9DS1 5 times\n\n");
        goto fail_tries;
    }

    _spi_sem->give();
    return true;

fail_tries:
fail_whoami:
    _spi_sem->give();
    return false;
}

void AP_InertialSensor_LSM9DS1::_fifo_reset()
{
    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    //Disable FIFO
    int reg_9 = _register_read(LSM9DS1XG_CTRL_REG9);
    _dev->write_register(LSM9DS1XG_CTRL_REG9, reg_9 & ~0x02);

    //Enable Bypass for reset FIFO
    _dev->write_register(LSM9DS1XG_FIFO_CTRL, LSM9DS1XG_FIFO_CTRL_FMODE_BYPASS);

    //Enable FIFO and Disable I2C
    _dev->write_register(LSM9DS1XG_CTRL_REG9,LSM9DS1XG_CTRL_REG9_FIFO_EN | LSM9DS1XG_CTRL_REG9_I2C_DISABLE);

    //Enable block data update, allow auto-increment during multiple byte read
    _dev->write_register(LSM9DS1XG_CTRL_REG8, LSM9DS1XG_CTRL_REG8_BDU | LSM9DS1XG_CTRL_REG8_IF_ADD_INC);
    hal.scheduler->delay_microseconds(1);

    //Enable FIFO stream mode and set watermark at 32 samples
    _dev->write_register(LSM9DS1XG_FIFO_CTRL, 0x1F | LSM9DS1XG_FIFO_CTRL_FMODE_FIFO);
    hal.scheduler->delay_microseconds(1);

    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    notify_accel_fifo_reset(_accel_instance);
    notify_gyro_fifo_reset(_gyro_instance);
}

/*
  start the sensor going
 */
void AP_InertialSensor_LSM9DS1::start(void)
{
    _gyro_instance = _imu.register_gyro(952, _dev->get_bus_id_devtype(DEVTYPE_GYR_LSM9DS1));
    _accel_instance = _imu.register_accel(952, _dev->get_bus_id_devtype(DEVTYPE_ACC_LSM9DS1));

    set_accel_orientation(_accel_instance, _rotation);
    set_gyro_orientation(_gyro_instance, _rotation);

    _set_accel_max_abs_offset(_accel_instance, 5.0f);

    /* start the timer process to read samples */
    _dev->register_periodic_callback(1000, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_LSM9DS1::_poll_data, void));
}

uint8_t AP_InertialSensor_LSM9DS1::_register_read(uint8_t reg)
{
    uint8_t val = 0;
    _dev->read_registers(reg, &val, 1);
    return val;
}


void AP_InertialSensor_LSM9DS1::_register_write(uint8_t reg, uint8_t val, bool checked)
{
    _dev->write_register(reg, val, checked);
}

void AP_InertialSensor_LSM9DS1::_gyro_init()
{
    _register_write(LSM9DS1XG_CTRL_REG1_G, LSM9DS1XG_CTRL_REG1_G_ODR_G_952Hz |
                                              LSM9DS1XG_CTRL_REG1_FS_G_2000DPS);
    hal.scheduler->delay(1);

    _register_write(LSM9DS1XG_CTRL_REG4, LSM9DS1XG_CTRL_REG4_Zen_G |
                                            LSM9DS1XG_CTRL_REG4_Yen_G |
                                            LSM9DS1XG_CTRL_REG4_Xen_G);
    _set_gyro_scale();
    hal.scheduler->delay(1);
}

void AP_InertialSensor_LSM9DS1::_accel_init()
{
    _register_write(LSM9DS1XG_CTRL_REG6_XL, LSM9DS1XG_CTRL_REG6_XL_ODR_XL_952Hz |
                                               LSM9DS1XG_CTRL_REG6_XL_FS_XL_16G);
    hal.scheduler->delay(1);

    _register_write(LSM9DS1XG_CTRL_REG5_XL, LSM9DS1XG_CTRL_REG5_XL_Zen_XL |
                                               LSM9DS1XG_CTRL_REG5_XL_Yen_XL |
                                               LSM9DS1XG_CTRL_REG5_XL_Xen_XL);
    _set_accel_scale(A_SCALE_16G);
    hal.scheduler->delay(1);
}

void AP_InertialSensor_LSM9DS1::_set_gyro_scale()
{
    /* scale value from datasheet 2000 mdps/digit */
    _gyro_scale = 70;
    /* convert mdps/digit to dps/digit */
    _gyro_scale /= 1000;
    /* convert dps/digit to (rad/s)/digit */
    _gyro_scale *= DEG_TO_RAD;
}

void AP_InertialSensor_LSM9DS1::_set_accel_scale(accel_scale scale)
{
    /*
     * Possible accelerometer scales (and their register bit settings) are:
     * 2 g (000), 4g (001), 6g (010) 8g (011), 16g (100). Here's a bit of an
     * algorithm to calculate g/(ADC tick) based on that 3-bit value:
     */
    _accel_scale = (((float) scale + 1.0f) * 2.0f) / 32768.0f;
    if (scale == A_SCALE_16G) {
        /* the datasheet shows an exception for +-16G */
        _accel_scale = 0.000732f;
    }
    /* convert to G/LSB to (m/s/s)/LSB */
    _accel_scale *= GRAVITY_MSS;
}

/**
 * Timer process to poll for new data from the LSM9DS1.
 */
void AP_InertialSensor_LSM9DS1::_poll_data()
{
    uint16_t samples = _register_read(LSM9DS1XG_FIFO_SRC);


    samples = samples & LSM9DS1XG_FIFO_SRC_UNREAD_SAMPLES;
    if (samples > 1) {
        _read_data_transaction_g(samples);
        _read_data_transaction_x(samples);
    }

    if (samples == 32) {
        _fifo_reset();
    }

    // check next register value for correctness
    if (!_dev->check_next_register()) {
        _inc_accel_error_count(_accel_instance);
    }
}

void AP_InertialSensor_LSM9DS1::_read_data_transaction_x(uint16_t samples)
{
    struct sensor_raw_data raw_data = { };
    int32_t _accel_bias[3] = {0, 0, 0};

    const uint8_t _reg = LSM9DS1XG_OUT_X_L_XL | 0x80;

    // Read the accel data stored in the FIFO
    for (int i = 0; i < samples; i++)
    {
        struct sensor_raw_data raw_data_temp = { };

        if (!_dev->transfer(&_reg, 1, (uint8_t *) &raw_data_temp, sizeof(raw_data_temp))) {
            hal.console->printf("LSM9DS1: error reading accelerometer\n");
            return;
        }

        _accel_bias[0] += (int32_t) raw_data_temp.x; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        _accel_bias[1] += (int32_t) raw_data_temp.y;
        _accel_bias[2] += (int32_t) raw_data_temp.z;
    }

    raw_data.x = _accel_bias[0] / samples; // average the data
    raw_data.y = _accel_bias[1] / samples;
    raw_data.z = _accel_bias[2] / samples;


    Vector3f accel_data(raw_data.x, raw_data.y, -raw_data.z);
    accel_data *= _accel_scale;

    _rotate_and_correct_accel(_accel_instance, accel_data);
    _notify_new_accel_raw_sample(_accel_instance, accel_data);
}

/*
 *  read from the data registers and update filtered data
 */
void AP_InertialSensor_LSM9DS1::_read_data_transaction_g(uint16_t samples)
{
    struct sensor_raw_data raw_data = { };
    int32_t _gyro_bias[3] = {0, 0, 0};

    const uint8_t _reg = LSM9DS1XG_OUT_X_L_G |  0x80;

    // Read the gyro data stored in the FIFO
    for (int i = 0; i < samples; i++)
    {
        struct sensor_raw_data raw_data_temp = { };

        if (!_dev->transfer(&_reg, 1, (uint8_t *) &raw_data_temp, sizeof(raw_data_temp))) {
            hal.console->printf("LSM9DS1: error reading gyroscope\n");
            return;
        }

        _gyro_bias[0] += (int32_t) raw_data_temp.x; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
        _gyro_bias[1] += (int32_t) raw_data_temp.y;
        _gyro_bias[2] += (int32_t) raw_data_temp.z;
    }

    raw_data.x = _gyro_bias[0] / samples; // average the data
    raw_data.y = _gyro_bias[1] / samples;
    raw_data.z = _gyro_bias[2] / samples;

    Vector3f gyro_data(raw_data.x, raw_data.y, -raw_data.z);
    gyro_data *= _gyro_scale;

    _rotate_and_correct_gyro(_gyro_instance, gyro_data);
    _notify_new_gyro_raw_sample(_gyro_instance, gyro_data);
}

bool AP_InertialSensor_LSM9DS1::update()
{
    update_gyro(_gyro_instance);
    update_accel(_accel_instance);

    return true;
}

#if LSM9DS1_DEBUG
/*
 *  dump all config registers - used for debug
*/
void AP_InertialSensor_LSM9DS1::_dump_registers(void)
{
    hal.console->println("LSM9DS1 registers:");

    const uint8_t first = LSM9DS1XG_ACT_THS;
    const uint8_t last = LSM9DS1XG_INT_GEN_DUR_G;
    for (uint8_t reg=first; reg<=last; reg++) {
        uint8_t v = _register_read(reg);
        hal.console->printf("%02x:%02x ", reg, v);
        if ((reg - (first-1)) % 16 == 0) {
            hal.console->println();
        }
    }
    hal.console->println();
}
#endif
