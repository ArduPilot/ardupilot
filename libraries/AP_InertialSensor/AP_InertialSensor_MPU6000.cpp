/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
#include <FastSerial.h>

#include "AP_InertialSensor_MPU6000.h"

#include <wiring.h>
#include <SPI.h>

// MPU 6000 registers
#define MPUREG_WHOAMI 0x75 //
#define MPUREG_SMPLRT_DIV 0x19 //
#define MPUREG_CONFIG 0x1A //
#define MPUREG_GYRO_CONFIG 0x1B
#define MPUREG_ACCEL_CONFIG 0x1C
#define MPUREG_FIFO_EN 0x23
#define MPUREG_INT_PIN_CFG 0x37
#define MPUREG_INT_ENABLE 0x38
#define MPUREG_INT_STATUS 0x3A
#define MPUREG_ACCEL_XOUT_H 0x3B //
#define MPUREG_ACCEL_XOUT_L 0x3C //
#define MPUREG_ACCEL_YOUT_H 0x3D //
#define MPUREG_ACCEL_YOUT_L 0x3E //
#define MPUREG_ACCEL_ZOUT_H 0x3F //
#define MPUREG_ACCEL_ZOUT_L 0x40 //
#define MPUREG_TEMP_OUT_H 0x41//
#define MPUREG_TEMP_OUT_L 0x42//
#define MPUREG_GYRO_XOUT_H 0x43 //
#define MPUREG_GYRO_XOUT_L 0x44 //
#define MPUREG_GYRO_YOUT_H 0x45 //
#define MPUREG_GYRO_YOUT_L 0x46 //
#define MPUREG_GYRO_ZOUT_H 0x47 //
#define MPUREG_GYRO_ZOUT_L 0x48 //
#define MPUREG_USER_CTRL 0x6A //
#define MPUREG_PWR_MGMT_1 0x6B //
#define MPUREG_PWR_MGMT_2 0x6C //
#define MPUREG_FIFO_COUNTH 0x72
#define MPUREG_FIFO_COUNTL 0x73
#define MPUREG_FIFO_R_W 0x74


// Configuration bits MPU 3000 and MPU 6000 (not revised)?
#define BIT_SLEEP 0x40
#define BIT_H_RESET 0x80
#define BITS_CLKSEL 0x07
#define MPU_CLK_SEL_PLLGYROX 0x01
#define MPU_CLK_SEL_PLLGYROZ 0x03
#define MPU_EXT_SYNC_GYROX 0x02
#define BITS_FS_250DPS              0x00
#define BITS_FS_500DPS              0x08
#define BITS_FS_1000DPS             0x10
#define BITS_FS_2000DPS             0x18
#define BITS_FS_MASK                0x18
#define BITS_DLPF_CFG_256HZ_NOLPF2  0x00
#define BITS_DLPF_CFG_188HZ         0x01
#define BITS_DLPF_CFG_98HZ          0x02
#define BITS_DLPF_CFG_42HZ          0x03
#define BITS_DLPF_CFG_20HZ          0x04
#define BITS_DLPF_CFG_10HZ          0x05
#define BITS_DLPF_CFG_5HZ           0x06
#define BITS_DLPF_CFG_2100HZ_NOLPF  0x07
#define BITS_DLPF_CFG_MASK          0x07
#define BIT_INT_ANYRD_2CLEAR      0x10
#define BIT_RAW_RDY_EN        0x01
#define BIT_I2C_IF_DIS              0x10
#define BIT_INT_STATUS_DATA   0x01

uint8_t AP_InertialSensor_MPU6000::_cs_pin;

/* pch: by the data sheet, the gyro scale should be 16.4LSB per DPS
 *      Given the radians conversion factor (0.174532), the gyro scale factor
 *      is waaaay off - output values are way too sensitive.
 *      Previously a divisor of 128 was appropriate.
 *      After tridge's changes to ::read, 50.0 seems about right based
 *      on making some 360 deg rotations on my desk.
 *      This issue requires more investigation.
 */
const float AP_InertialSensor_MPU6000::_gyro_scale = (0.0174532 / 16.4);
const float AP_InertialSensor_MPU6000::_accel_scale = 9.81 / 4096.0;

/* pch: I believe the accel and gyro indicies are correct
 *      but somone else should please confirm.
 */
const uint8_t AP_InertialSensor_MPU6000::_gyro_data_index[3]  = { 5, 4, 6 };
const int8_t  AP_InertialSensor_MPU6000::_gyro_data_sign[3]   = { 1, 1, -1 };

const uint8_t AP_InertialSensor_MPU6000::_accel_data_index[3] = { 1, 0, 2 };
const int8_t  AP_InertialSensor_MPU6000::_accel_data_sign[3]  = { 1, 1, -1 };

const uint8_t AP_InertialSensor_MPU6000::_temp_data_index = 3;

static volatile uint8_t _new_data;

AP_InertialSensor_MPU6000::AP_InertialSensor_MPU6000( uint8_t cs_pin )
{
  _cs_pin = cs_pin; /* can't use initializer list,  is static */
  _gyro.x = 0;
  _gyro.y = 0;
  _gyro.z = 0;
  _accel.x = 0;
  _accel.y = 0;
  _accel.z = 0;
  _temp = 0;
  _initialised = 0;
}

void AP_InertialSensor_MPU6000::init( AP_PeriodicProcess * scheduler )
{
    if (_initialised) return;
    _initialised = 1;
    hardware_init();
    scheduler->register_process( &AP_InertialSensor_MPU6000::read );
}

// accumulation in ISR - must be read with interrupts disabled
// the sum of the values since last read
static volatile int32_t _sum[7];

// how many values we've accumulated since last read
static volatile uint16_t _count;


/*================ AP_INERTIALSENSOR PUBLIC INTERFACE ==================== */

bool AP_InertialSensor_MPU6000::update( void )
{
	int32_t sum[7];
	uint16_t count;
	float count_scale;

	// wait for at least 1 sample
	while (_count == 0) /* nop */;

	// disable interrupts for mininum time
	cli();
	for (int i=0; i<7; i++) {
		sum[i] = _sum[i];
		_sum[i] = 0;
	}
	count = _count;
	_count = 0;
	sei();

	count_scale = 1.0 / count;

	_gyro.x = _gyro_scale * _gyro_data_sign[0] * sum[_gyro_data_index[0]] * count_scale;
	_gyro.y = _gyro_scale * _gyro_data_sign[1] * sum[_gyro_data_index[1]] * count_scale;
	_gyro.z = _gyro_scale * _gyro_data_sign[2] * sum[_gyro_data_index[2]] * count_scale;

	_accel.x = _accel_scale * _accel_data_sign[0] * sum[_accel_data_index[0]] * count_scale;
	_accel.y = _accel_scale * _accel_data_sign[1] * sum[_accel_data_index[1]] * count_scale;
	_accel.z = _accel_scale * _accel_data_sign[2] * sum[_accel_data_index[2]] * count_scale;

	_temp    = _temp_to_celsius(sum[_temp_data_index] * count_scale);

	return true;
}

float AP_InertialSensor_MPU6000::gx() { return _gyro.x; }
float AP_InertialSensor_MPU6000::gy() { return _gyro.y; }
float AP_InertialSensor_MPU6000::gz() { return _gyro.z; }

void AP_InertialSensor_MPU6000::get_gyros( float * g )
{
  g[0] = _gyro.x;
  g[1] = _gyro.y;
  g[2] = _gyro.z;
}

float AP_InertialSensor_MPU6000::ax() { return _accel.x; }
float AP_InertialSensor_MPU6000::ay() { return _accel.y; }
float AP_InertialSensor_MPU6000::az() { return _accel.z; }

void AP_InertialSensor_MPU6000::get_accels( float * a )
{
  a[0] = _accel.x;
  a[1] = _accel.y;
  a[2] = _accel.z;
}

void AP_InertialSensor_MPU6000::get_sensors( float * sensors )
{
  sensors[0] = _gyro.x;
  sensors[1] = _gyro.y;
  sensors[2] = _gyro.z;
  sensors[3] = _accel.x;
  sensors[4] = _accel.y;
  sensors[5] = _accel.z;
}

float AP_InertialSensor_MPU6000::temperature() { return _temp; }

uint32_t AP_InertialSensor_MPU6000::sample_time()
{
  uint32_t us = micros();
  uint32_t delta = us - _last_sample_micros;
  reset_sample_time();
  return delta;
}

void AP_InertialSensor_MPU6000::reset_sample_time()
{
    _last_sample_micros = micros();
}

/*================ HARDWARE FUNCTIONS ==================== */

static int16_t spi_transfer_16(void)
{
	uint8_t byte_H, byte_L;
	byte_H = SPI.transfer(0);
	byte_L = SPI.transfer(0);
	return (((int16_t)byte_H)<<8) | byte_L;
}

/*
  this is called from a timer interrupt to read data from the MPU6000
  and add it to _sum[]
 */
void AP_InertialSensor_MPU6000::read(uint32_t )
{
    if (_new_data == 0) {
        // no new data is ready from the MPU6000
        return;
    }
    _new_data = 0;

    // now read the data
    digitalWrite(_cs_pin, LOW);
    byte addr = MPUREG_ACCEL_XOUT_H | 0x80;
    SPI.transfer(addr);
    for (uint8_t i=0; i<7; i++) {
        _sum[i] += spi_transfer_16();
    }

    _count++;
    if (_count == 0) {
        // rollover - v unlikely
        memset((void*)_sum, 0, sizeof(_sum));
    }

    digitalWrite(_cs_pin, HIGH);
}

uint8_t AP_InertialSensor_MPU6000::register_read( uint8_t reg )
{
  uint8_t dump;
  uint8_t return_value;
  uint8_t addr = reg | 0x80; // Set most significant bit

  digitalWrite(_cs_pin, LOW);

  dump = SPI.transfer(addr);
  return_value = SPI.transfer(0);

  digitalWrite(_cs_pin, HIGH);

  return return_value;
}

void AP_InertialSensor_MPU6000::register_write(uint8_t reg, uint8_t val)
{
  uint8_t dump;
  digitalWrite(_cs_pin, LOW);
  dump = SPI.transfer(reg);
  dump = SPI.transfer(val);
  digitalWrite(_cs_pin, HIGH);
}

// MPU6000 new data interrupt on INT6
void AP_InertialSensor_MPU6000::data_interrupt(void)
{
    // tell the timer routine that there is data to be read
    _new_data = 1;
}

void AP_InertialSensor_MPU6000::hardware_init()
{
    // MPU6000 chip select setup
    pinMode(_cs_pin, OUTPUT);
    digitalWrite(_cs_pin, HIGH);
    delay(1);

    // Chip reset
    register_write(MPUREG_PWR_MGMT_1, BIT_H_RESET);
    delay(100);
    // Wake up device and select GyroZ clock (better performance)
    register_write(MPUREG_PWR_MGMT_1, MPU_CLK_SEL_PLLGYROZ);
    delay(1);
    // Disable I2C bus (recommended on datasheet)
    register_write(MPUREG_USER_CTRL, BIT_I2C_IF_DIS);
    delay(1);
    // SAMPLE RATE
    register_write(MPUREG_SMPLRT_DIV,0x04);     // Sample rate = 200Hz    Fsample= 1Khz/(4+1) = 200Hz
    delay(1);
    // FS & DLPF   FS=2000º/s, DLPF = 98Hz (low pass filter)
    register_write(MPUREG_CONFIG, BITS_DLPF_CFG_98HZ);
    delay(1);
    register_write(MPUREG_GYRO_CONFIG,BITS_FS_2000DPS);  // Gyro scale 2000º/s
    delay(1);
    register_write(MPUREG_ACCEL_CONFIG,0x08);           // Accel scele 4g (4096LSB/g)
    delay(1);

    // INT CFG => Interrupt on Data Ready
    register_write(MPUREG_INT_ENABLE,BIT_RAW_RDY_EN);         // INT: Raw data ready
    delay(1);
    register_write(MPUREG_INT_PIN_CFG,BIT_INT_ANYRD_2CLEAR);  // INT: Clear on any read
    delay(1);
    // Oscillator set
    // register_write(MPUREG_PWR_MGMT_1,MPU_CLK_SEL_PLLGYROZ);
    delay(1);

    attachInterrupt(6,data_interrupt,RISING);
}

float AP_InertialSensor_MPU6000::_temp_to_celsius ( uint16_t regval )
{
    /* TODO */
    return 20.0;
}
