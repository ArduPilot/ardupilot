/*
  minimal test program for ICM20789 baro with IMU on SPI and baro on I2C
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <stdio.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#ifdef HAL_INS_MPU60x0_NAME
static AP_HAL::Device *i2c_dev;
static AP_HAL::Device *spi_dev;
static AP_HAL::Device *dev;
#endif  // defined(HAL_INS_MPU60x0_NAME)

// SPI registers
#define MPUREG_WHOAMI                           0x75
#define MPUREG_USER_CTRL                        0x6A
#define MPUREG_PWR_MGMT_1                       0x6B

#define MPUREG_INT_PIN_CFG                      0x37
#       define BIT_BYPASS_EN                        0x02
#       define BIT_INT_RD_CLEAR                     0x10    // clear the interrupt when any read occurs
#       define BIT_LATCH_INT_EN                     0x20    // latch data ready pin

#define BIT_USER_CTRL_I2C_MST_EN             0x20 // Enable MPU to act as the I2C Master to external slave sensors
#define BIT_PWR_MGMT_1_DEVICE_RESET          0x80 // reset entire device
#define BIT_USER_CTRL_I2C_IF_DIS             0x10 // Disable primary I2C interface and enable hal.spi->interface
#define BIT_PWR_MGMT_1_CLK_XGYRO             0x01 // PLL with X axis gyroscope reference
#define BIT_PWR_MGMT_1_CLK_ZGYRO             0x03 // PLL with Z axis gyroscope reference

// baro commands
#define CMD_SOFT_RESET     0x805D
#define CMD_READ_ID        0xEFC8

void setup(void);
void loop(void);


#ifdef HAL_INS_MPU60x0_NAME
static void spi_init()
{
    // SPI reads have flag 0x80 set
    spi_dev->set_read_flag(0x80);

    // run the SPI bus at low speed
    spi_dev->set_speed(AP_HAL::Device::SPEED_LOW);

    uint8_t whoami = 0;
    uint8_t v;

    spi_dev->write_register(0x6B, 0x01);
    spi_dev->write_register(0x6B, 0x01);

    hal.scheduler->delay(1);
    spi_dev->write_register(0x6A, 0x10);
    spi_dev->write_register(0x6B, 0x41);

    hal.scheduler->delay(1);
    spi_dev->write_register(0x6C, 0x3f);
    spi_dev->write_register(0xF5, 0x00);
    spi_dev->write_register(0x19, 0x09);
    spi_dev->write_register(0xEA, 0x00);
    spi_dev->write_register(0x6B, 0x01);

    hal.scheduler->delay(1);
    spi_dev->write_register(0x6A, 0x10);
    spi_dev->write_register(0x6B, 0x41);

    hal.scheduler->delay(1);
    spi_dev->write_register(0x6B, 0x01);

    hal.scheduler->delay(1);
    spi_dev->write_register(0x23, 0x00);
    spi_dev->write_register(0x6B, 0x41);

    hal.scheduler->delay(1);
    spi_dev->write_register(0x1D, 0xC0);
    spi_dev->write_register(0x6B, 0x01);

    hal.scheduler->delay(1);
    spi_dev->write_register(0x1A, 0xC0);
    spi_dev->write_register(0x6B, 0x41);

    hal.scheduler->delay(1);
    spi_dev->write_register(0x38, 0x01);

    spi_dev->read_registers(0x6A, &v, 1);
    printf("reg 0x6A=0x%02x\n", v);
    spi_dev->read_registers(0x6B, &v, 1);
    printf("reg 0x6B=0x%02x\n", v);

    hal.scheduler->delay(1);
    spi_dev->write_register(0x6A, 0x10);
    spi_dev->write_register(0x6B, 0x41);

    hal.scheduler->delay(1);
    spi_dev->write_register(0x6B, 0x01);

    hal.scheduler->delay(1);
    spi_dev->write_register(0x23, 0x00);
    spi_dev->write_register(0x6B, 0x41);

    hal.scheduler->delay(1);
    spi_dev->write_register(0x6B, 0x41);
    spi_dev->write_register(0x6C, 0x3f);
    spi_dev->write_register(0x6B, 0x41);

    spi_dev->read_registers(0x6A, &v, 1);
    printf("reg 0x6A=0x%02x\n", v);
    spi_dev->write_register(0x6B, 0x01);

    hal.scheduler->delay(1);
    spi_dev->write_register(0x6A, 0x10);
    spi_dev->write_register(0x6B, 0x41);

    hal.scheduler->delay(1);
    spi_dev->write_register(0x6B, 0x01);

    hal.scheduler->delay(1);
    spi_dev->write_register(0x23, 0x00);
    spi_dev->write_register(0x6B, 0x41);

    hal.scheduler->delay(1);
    spi_dev->read_registers(0x6A, &v, 1);
    printf("reg 0x6A=0x%02x\n", v);
    spi_dev->write_register(0x6B, 0x01);

    hal.scheduler->delay(1);
    spi_dev->write_register(0x6A, 0x10);
    spi_dev->write_register(0x6B, 0x41);

    hal.scheduler->delay(1);
    spi_dev->write_register(0x6B, 0x01);

    hal.scheduler->delay(1);
    spi_dev->write_register(0x23, 0x00);
    spi_dev->write_register(0x6B, 0x41);

    hal.scheduler->delay(1);
    spi_dev->write_register(0x6B, 0x41);
    spi_dev->write_register(0x6C, 0x3f);
    spi_dev->write_register(0x6B, 0x41);

    spi_dev->read_registers(0x6A, &v, 1);
    printf("reg 0x6A=0x%02x\n", v);
    spi_dev->write_register(0x6B, 0x01);

    hal.scheduler->delay(1);
    spi_dev->write_register(0x6A, 0x10);
    spi_dev->write_register(0x6B, 0x41);

    hal.scheduler->delay(1);
    spi_dev->write_register(0x6B, 0x01);

    hal.scheduler->delay(1);
    spi_dev->write_register(0x23, 0x00);
    spi_dev->write_register(0x6B, 0x41);

    // print the WHOAMI
    spi_dev->read_registers(MPUREG_WHOAMI, &whoami, 1);
    printf("20789 SPI whoami: 0x%02x\n", whoami);

    // wait for sensor to settle
    hal.scheduler->delay(100);

    // dump registers
    printf("ICM20789 registers\n");
#if 0
    for (uint8_t reg = 0; reg<0x80; reg++) {
        v=0;
        spi_dev->read_registers(reg, &v, 1);
        printf("%02x:%02x ", (unsigned)reg, (unsigned)v);
        if ((reg+1) % 16 == 0) {
            printf("\n");
        }
    }
    printf("\n");
#endif

    spi_dev->get_semaphore()->give();
}
#endif

#ifdef HAL_INS_MPU60x0_NAME
/*
  send a 16 bit command to the baro
 */
static bool send_cmd16(uint16_t cmd)
{
    uint8_t cmd_b[2] = { uint8_t(cmd >> 8), uint8_t(cmd & 0xFF) };
    return i2c_dev->transfer(cmd_b, 2, nullptr, 0);
}

/*
  read baro calibration data
 */
static bool read_calibration_data(void)
{
    // setup for OTP read
    const uint8_t cmd[5] = { 0xC5, 0x95, 0x00, 0x66, 0x9C };
    if (!i2c_dev->transfer(cmd, sizeof(cmd), nullptr, 0)) {
        return false;
    }
    for (uint8_t i=0; i<4; i++) {
        if (!send_cmd16(0xC7F7)) {
            return false;
        }
        uint8_t d[3];
        if (!i2c_dev->transfer(nullptr, 0, d, sizeof(d))) {
            return false;
        }
        uint16_t c = int16_t((d[0]<<8) | d[1]);
        printf("sensor_constants[%u]=%d\n", i, c);
    }
    return true;
}

// initialise baro on i2c
static void i2c_init(void)
{
    i2c_dev->get_semaphore()->take_blocking();

    if (send_cmd16(CMD_READ_ID)) {
        printf("ICM20789: read ID ******\n");
        uint8_t id[3] {};
        if (!i2c_dev->transfer(nullptr, 0, id, 3)) {
            printf("ICM20789: failed to read ID\n");
        }
        printf("ICM20789: ID %02x %02x %02x\n", id[0], id[1], id[2]);
    } else {
        printf("ICM20789 read ID failed\n");
    }

    if (send_cmd16(CMD_SOFT_RESET)) {
        printf("ICM20789: reset OK ************###########*********!!!!!!!!\n");
    } else {
        printf("ICM20789 baro reset failed\n");
    }
    hal.scheduler->delay(1);


    read_calibration_data();

    i2c_dev->get_semaphore()->give();

    printf("checking baro\n");
    dev->get_semaphore()->take_blocking();

    uint8_t regs[3] = { 0xC0, 0xC1, 0xC2 };
    for (uint8_t i=0; i<ARRAY_SIZE(regs); i++) {
        uint8_t v = 0;
        dev->read_registers(regs[i], &v, 1);
        printf("0x%02x : 0x%02x\n", regs[i], v);
    }
    dev->get_semaphore()->give();
}
#endif

void setup()
{
    printf("ICM20789 test\n");

    AP_BoardConfig{}.init();

    hal.scheduler->delay(1000);

#ifdef HAL_INS_MPU60x0_NAME
    spi_dev = hal.spi->get_device_ptr(HAL_INS_MPU60x0_NAME);

    spi_dev->get_semaphore()->take_blocking();

    i2c_dev = hal.i2c_mgr->get_device_ptr(1, 0x63);
    dev = hal.i2c_mgr->get_device_ptr(1, 0x29);

    while (true) {
        spi_init();
        i2c_init();
        hal.scheduler->delay(1000);
    }
#else
    while (true) {
        printf("HAL_INS_MPU60x0_NAME not defined for this board\n");
        hal.scheduler->delay(1000);
    }
#endif
}


void loop()
{
}

AP_HAL_MAIN();
