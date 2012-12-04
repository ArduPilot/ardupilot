/*******************************************
*   Sample sketch that configures an MPU6000
*   and reads back the three axis of accel,
*   temperature, three axis of gyro data
*******************************************/

#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

/* register #defines */
#include "MPU6000.h"

// debug only:
#include <avr/io.h>

const AP_HAL_AVR::HAL_AVR& hal = AP_HAL_AVR_APM2;

const uint8_t _baro_cs_pin = 40;

AP_HAL::SPIDeviceDriver* spidev;

static void register_write(uint8_t reg, uint8_t val) {
    hal.console->printf_P(PSTR("write reg %d val %d\r\n"),
            (int) reg, (int) val);
    spidev->cs_assert();
    spidev->transfer(reg);
    spidev->transfer(val);
    spidev->cs_release();
}

static uint8_t register_read(uint8_t reg) {
    /* set most significant bit to read register */
    uint8_t regaddr = reg | 0x80;
    spidev->cs_assert();
    spidev->transfer(regaddr);
    uint8_t val = spidev->transfer(0);
    spidev->cs_release();
    return val;
}

static uint16_t spi_read_16(void) {
    uint8_t byte_h, byte_l;
    byte_h = spidev->transfer(0);
    byte_l = spidev->transfer(0);
    return (((int16_t) byte_h)<< 8) | byte_l;
}


static void mpu6k_init(void) {
    // chip reset
    register_write(MPUREG_PWR_MGMT_1, BIT_H_RESET);
    hal.scheduler->delay(100);
    // Wake up device and select GyroZ clock (better performance)
    register_write(MPUREG_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
    hal.scheduler->delay(1);
    register_write(MPUREG_PWR_MGMT_2, 0);
    hal.scheduler->delay(1);
    // Disable I2C bus (recommended on datasheet)
    register_write(MPUREG_USER_CTRL, BIT_I2C_IF_DIS);
    hal.scheduler->delay(1);
    // SAMPLE RATE
    //// Sample rate = 200Hz    Fsample= 1Khz/(4+1) = 200Hz
    register_write(MPUREG_SMPLRT_DIV,0x04);
    hal.scheduler->delay(1);
    // FS & DLPF   FS=2000º/s, DLPF = 98Hz (low pass filter)
    register_write(MPUREG_CONFIG, BITS_DLPF_CFG_98HZ);
    hal.scheduler->delay(1);
    register_write(MPUREG_GYRO_CONFIG,BITS_FS_2000DPS);  // Gyro scale 2000º/s
    hal.scheduler->delay(1);

     // read the product ID rev c has 1/2 the sensitivity of rev d
    uint8_t _product_id = register_read(MPUREG_PRODUCT_ID);

    //Serial.printf("Product_ID= 0x%x\n", (unsigned) _product_id);

    if ((_product_id == MPU6000ES_REV_C4) || (_product_id == MPU6000ES_REV_C5)
      ||(_product_id == MPU6000_REV_C4)   || (_product_id == MPU6000_REV_C5)){
        // Accel scale 8g (4096 LSB/g)
        // Rev C has different scaling than rev D
        register_write(MPUREG_ACCEL_CONFIG,1<<3);
    } else {
        // Accel scale 8g (4096 LSB/g)
        register_write(MPUREG_ACCEL_CONFIG,2<<3);
    }
    hal.scheduler->delay(1);

    // INT CFG => Interrupt on Data Ready
    //// INT: Raw data ready
    register_write(MPUREG_INT_ENABLE,BIT_RAW_RDY_EN);
    hal.scheduler->delay(1);
    // INT: Clear on any read
    register_write(MPUREG_INT_PIN_CFG,BIT_INT_ANYRD_2CLEAR); 
    hal.scheduler->delay(1);

}

static void mpu6k_read(int16_t* data) {

    spidev->cs_assert();
    spidev->transfer( MPUREG_ACCEL_XOUT_H | 0x80 );
    for (int i = 0; i < 7; i++) {
        data[i] = spi_read_16();
    }
    spidev->cs_release();
}

static void setup() {
    hal.console->printf_P(PSTR("Initializing MPU6000\r\n"));
    spidev = hal.spi->device(AP_HAL::SPIDevice_MPU6000);

#if 0
    /* Setup the barometer cs to stop it from holding the bus */
    hal.gpio->pinMode(_baro_cs_pin, GPIO_OUTPUT);
    hal.gpio->write(_baro_cs_pin, 1);
#endif

    spidev->cs_assert();
    uint8_t spcr = SPCR;
    uint8_t spsr = SPSR;
    spidev->cs_release();

    mpu6k_init();
}

static void loop() {
    int16_t sensors[7];
    mpu6k_read(sensors);
    hal.console->printf_P(PSTR("mpu6k: %d %d %d %d %d %d %d\r\n"),
        sensors[0], sensors[1], sensors[2],
        sensors[3], sensors[4], sensors[5], sensors[6]);
    hal.scheduler->delay(10);
}

AP_HAL_MAIN();
