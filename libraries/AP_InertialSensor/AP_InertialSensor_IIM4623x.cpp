/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>
#include <GCS_MAVLink/GCS.h>

#include "AP_InertialSensor_IIM4623x.h"

/*
  device registers
 */
#define REG_WHO_AM_I 0
#define REG_SAMPLE_RATE_DIV 0x1A
#define REG_ACCEL_CONFIG0 0x33
#define REG_GYRO_CONFIG0 0x34
#define REG_SELECT_OUT_DATA 0x1c

#define BYTE_HEADER_CMD	0x24
#define BYTE_HEADER_REP	0x23
#define BYTE_RESERVED	0x00
#define BYTE_FOOTER_1	0x0D
#define BYTE_FOOTER_2	0x0A
#define BYTE_PADDING	0x00

#define CMD_TYPE_SWITCH_TO_BOOTLOADER  0xF4
#define CMD_TYPE_UPGRADE_FIRMWARE      0xF5
#define CMD_TYPE_CLEAR_UPGRADE_FLAG    0xF6
#define CMD_TYPE_LENGTH_INFO           0xDA
#define CMD_TYPE_IMAGE_DATA            0xDA

#define CMD_TYPE_GET_VERSION		0x20
#define CMD_TYPE_GET_SERIAL_NUM		0x26
#define CMD_TYPE_READ_REG			0x11
#define CMD_TYPE_WRITE_REG			0x12
#define CMD_TYPE_SELF_TEST			0x2B
#define CMD_TYPE_SET_UTC_TIME		0x2D
#define CMD_TYPE_START_STREAMING	0x27
#define CMD_TYPE_STOP_STREAMING		0x28
#define CMD_TYPE_ENABLE_SENSORFT	0x2E
#define CMD_TYPE_DISABLE_SENSORFT	0x2F

#define IIM46230_WHO_AM_I	0xE6
#define IIM46234_WHO_AM_I	0xEA

#define IIM4623x_DEBUG 0

// IIM4623x uses gravity of 9.8 (can be configured in CUSTOM_GRAVITY register)
#define IIM4623x_GRAVITY 9.8

#if IIM4623x_DEBUG
#define Debug(fmt, args ...)  do { gcs().send_text(MAV_SEVERITY_DEBUG, fmt, ## args); } while (0)
#else
#define Debug(fmt, args ...)
#endif

#define TIMING_DEBUG 0
#if TIMING_DEBUG
#define DEBUG_SET_PIN(n,v) hal.gpio->write(52+n, v)
#define DEBUG_TOGGLE_PIN(n) hal.gpio->toggle(52+n)
#else
#define DEBUG_SET_PIN(n,v)
#define DEBUG_TOGGLE_PIN(n)
#endif

enum IIM4623x_Intf {
	INTF_UART = 1,
	INTF_SPI
};

enum IIM4623x_Mode {
	COMMAND = 0,
	STREAMING
};

enum IIM4623x_OutDataForm {
	FLOATING = 0,	// 32-Bit IEEE 754 single-precision floating point (default)
	FIXED			// 32-Bit Fixed point 2's Complement representation
};

enum IIM4623x_DataOutPut {
	ACCEL = 0,
	GYRO,
	TEMP,
	DELTA_VEL,
	DELTA_ANG
};

enum IIM4623x_SampleRateDiv {
	ODR_1KHZ = 1,
	ODR_500HZ = 2,
	ODR_250HZ = 4,
	ODR_200HZ = 5,
	ODR_125HZ = 8,
	ODR_100HZ = 10,
	ODR_50HZ = 20,
	ODR_25HZ = 40,
	ODR_20HZ = 50,
	ODR_10HZ = 100 // 0x64
};

enum IIM4623x_UartBaudRate {
	BAUD_921600 = 0,
	BAUD_1500000 = 1,
	BAUD_3000000 = 3
};

enum IIM4623x_SyncConfig {
	DISABLE_SYNC = 0,
	SYNC_WITH_PPS = 1
};

enum IIM4623x_AccBwConfig {
	ACC_LPF_BW4 = 0x40,
	ACC_LPF_BW5 = 0x50,
	ACC_LPF_BW6 = 0x60,
	ACC_LPF_BW7 = 0x70
};

enum IIM4623x_GyroBwConfig {
	GYRO_LPF_BW4 = 0x4,
	GYRO_LPF_BW5 = 0x5,
	GYRO_LPF_BW6 = 0x6,
	GYRO_LPF_BW7 = 0x7
};

enum IIM4623x_AccelConfig0 {
    ACC_FSR_8G = 0x10,
    ACC_FSR_4G = 0x20,
    ACC_FSR_2G = 0x40
};

enum IIM4623x_GyroConfig0 {
	GYRO_FSR_2000DPS = 0x00,
	GYRO_FSR_1000DPS = 0x20,
	GYRO_FSR_500DPS = 0x40,
	GYRO_FSR_480DPS = 0x40,
	GYRO_FSR_250DPS = 0x60
};

enum IIM4623x_Axis {
	ACCEL_X = 0,
	ACCEL_Y,
	ACCEL_Z,
	GYRO_X,
	GYRO_Y,
	GYRO_Z
};

enum IIM4623x_CalibConfig {
	ACCEL_BIAS = 0,
	GYRO_BIAS,
	ACCEL_SENS,
	GYRO_SENS
};


extern const AP_HAL::HAL& hal;

uint16_t AP_InertialSensor_IIM4623x::calc_checksum(uint8_t *buff, uint32_t length) const
{
	uint16_t sum = 0;
    for (uint8_t i = 0; i < length; i++) {
        sum += (uint16_t)buff[i];
    }
	return sum;
}

AP_InertialSensor_IIM4623x::AP_InertialSensor_IIM4623x(AP_InertialSensor &imu,
                                                         AP_HAL::OwnPtr<AP_HAL::Device> _dev,
                                                         enum Rotation _rotation,
                                                         int8_t _drdy_pin)
    : AP_InertialSensor_Backend(imu)
    , dev(std::move(_dev))
    , rotation(_rotation)
    , drdy_pin(_drdy_pin)
{
}

AP_InertialSensor_Backend *AP_InertialSensor_IIM4623x::probe(AP_InertialSensor &imu,
                                                             AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                                             enum Rotation rotation,
                                                             int8_t drdy_pin)
{
    if (!dev) {
        return nullptr;
    }

    AP_InertialSensor_IIM4623x *sensor = new AP_InertialSensor_IIM4623x(imu, std::move(dev), rotation, drdy_pin);

    if (sensor == nullptr) {
        return nullptr;
    }

    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}


void AP_InertialSensor_IIM4623x::start()
{
    if (!_imu.register_accel(accel_instance, expected_sample_rate_hz, dev->get_bus_id_devtype(dev_type)) ||
        !_imu.register_gyro(gyro_instance, expected_sample_rate_hz,   dev->get_bus_id_devtype(dev_type))) {
        return;
    }

    // setup sensor rotations from probe()
    set_gyro_orientation(gyro_instance, rotation);
    set_accel_orientation(accel_instance, rotation);

    /*
      as the sensor does not have a FIFO we need to jump through some
      hoops to ensure we don't lose any samples. This creates a thread
      to do the capture, running at very high priority
     */
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_InertialSensor_IIM4623x::loop, void),
                                      "IIM4623X",
                                      1024, AP_HAL::Scheduler::PRIORITY_BOOST, 1)) {
        AP_HAL::panic("Failed to create IIM4623X thread");
    }
}

static float swap_float(const float f)
{
    return be32tofloat_ptr((const uint8_t *)&f);
}

/*
  read one sample of sensor data
 */
void AP_InertialSensor_IIM4623x::read_sensor()
{
    WITH_SEMAPHORE(dev->get_semaphore());
    uint8_t status;
    uint8_t counter;
    uint64_t timestamp;
    struct {
        float accel[3];
        float gyro[3];
        float temp;
    } data;

    if (!read_streaming((uint8_t*)&data, sizeof(data), status, counter, timestamp)) {
        return;
    }

    if (first_timestamp_us == 0) {
        first_timestamp_us = AP_HAL::micros64();
        first_imu_timestamp_us = timestamp;
    }

    Vector3f accel{
        swap_float(data.accel[0])*IIM4623x_GRAVITY,
        swap_float(data.accel[1])*IIM4623x_GRAVITY,
        swap_float(data.accel[2])*IIM4623x_GRAVITY};
    Vector3f gyro{
        swap_float(data.gyro[0])*DEG_TO_RAD,
        swap_float(data.gyro[1])*DEG_TO_RAD,
        swap_float(data.gyro[2])*DEG_TO_RAD};

    const uint64_t sample_us = (timestamp - first_imu_timestamp_us) + first_timestamp_us;
    _rotate_and_correct_accel(accel_instance, accel);
    _notify_new_accel_raw_sample(accel_instance, accel, sample_us);

    _rotate_and_correct_gyro(gyro_instance, gyro);
    _notify_new_gyro_raw_sample(gyro_instance, gyro, sample_us);

    /*
      publish average temperature at 20Hz
     */
    temp_sum += swap_float(data.temp);
    temp_count++;

    if (temp_count == 100) {
        _publish_temperature(accel_instance, temp_sum/temp_count);
        temp_sum = 0;
        temp_count = 0;
    }
}

/*
  wait for data ready with timeout
 */
bool AP_InertialSensor_IIM4623x::wait_dready(uint32_t timeout_us)
{
    if (drdy_pin < 0) {
        hal.scheduler->delay_microseconds(timeout_us);
        return false;
    }
    return hal.gpio->wait_pin(drdy_pin, AP_HAL::GPIO::INTERRUPT_RISING, timeout_us);
}

/*
  send a command packet
 */
void AP_InertialSensor_IIM4623x::send_command(uint8_t command_type, uint8_t *data, uint8_t length)
{
    hal.scheduler->delay(1);
    WITH_SEMAPHORE(dev->get_semaphore());
    uint8_t pkt[20] {};
    pkt[0] = BYTE_HEADER_CMD;
    pkt[1] = BYTE_HEADER_CMD;
    pkt[2] = 8+length;
    pkt[3] = command_type;
    memcpy(&pkt[4], data, length);
    const uint16_t checksum = calc_checksum(&pkt[3], 1+length);
    put_be16_ptr(&pkt[4+length], checksum);
    pkt[6+length] = BYTE_FOOTER_1;
    pkt[7+length] = BYTE_FOOTER_2;

    dev->transfer(pkt, sizeof(pkt), nullptr, 0);
}

/*
  read a command reply
 */
bool AP_InertialSensor_IIM4623x::read_reply(uint8_t *data, uint8_t length)
{
    WITH_SEMAPHORE(dev->get_semaphore());
    uint8_t reply[8+length];
    memset(reply, 0, sizeof(reply));

    dev->transfer(nullptr, 0, reply, sizeof(reply));

    if (reply[0] != BYTE_HEADER_REP || reply[1] != BYTE_HEADER_REP) {
        Debug("fail %u %02x %02x len=%u", __LINE__, unsigned(reply[0]), unsigned(reply[1]), unsigned(sizeof(reply)));
        return false;
    }
    if (reply[2] != sizeof(reply)) {
        Debug("fail %u", __LINE__);
        return false;
    }
    if (reply[sizeof(reply)-2] != BYTE_FOOTER_1 || reply[sizeof(reply)-1] != BYTE_FOOTER_2) {
        Debug("fail %u", __LINE__);
        return false;
    }
    const uint16_t expected_checksum = calc_checksum(&reply[3], sizeof(reply)-(3+4));
    const uint16_t got_checksum = be16toh_ptr(&reply[sizeof(reply)-4]);
    if (expected_checksum != got_checksum) {
        Debug("fail %u", __LINE__);
        return false;
    }
    memcpy(data, &reply[4], length);
    return true;
}

/*
  read from a register
 */
bool AP_InertialSensor_IIM4623x::read_register(uint8_t address, uint8_t *data, uint8_t length)
{
    uint8_t cmd[] { 0, length, address, 0 };
    send_command(CMD_TYPE_READ_REG, cmd, sizeof(cmd));

    wait_dready(500);

    uint8_t reply[8+length];
    memset(reply, 0, sizeof(reply));
    if (!read_reply(reply, sizeof(reply))) {
        return false;
    }
    memcpy(data, &reply[8], length);
    return true;
}

/*
  read version data
 */
bool AP_InertialSensor_IIM4623x::read_version(uint8_t &major, uint8_t &minor)
{
    send_command(CMD_TYPE_GET_VERSION, nullptr, 0);

    wait_dready(500);

    uint8_t reply[12] {};
    if (!read_reply(reply, sizeof(reply))) {
        return false;
    }
    major = reply[2];
    minor = reply[3];
    return true;
}

/*
  read streaming data
 */
bool AP_InertialSensor_IIM4623x::read_streaming(uint8_t *data, uint8_t length, uint8_t &status, uint8_t &counter, uint64_t &timestamp_us)
{
    WITH_SEMAPHORE(dev->get_semaphore());
    uint8_t pkt[18+length];
    memset(pkt, 0, sizeof(pkt));

    dev->transfer(nullptr, 0, pkt, sizeof(pkt));

    if (pkt[0] != BYTE_HEADER_REP || pkt[1] != BYTE_HEADER_REP) {
        return false;
    }
    if (pkt[2] != sizeof(pkt)) {
        return false;
    }
    if (pkt[sizeof(pkt)-2] != BYTE_FOOTER_1 || pkt[sizeof(pkt)-1] != BYTE_FOOTER_2) {
        return false;
    }
    const uint16_t expected_checksum = calc_checksum(&pkt[3], sizeof(pkt)-(3+4));
    const uint16_t got_checksum = be16toh_ptr(&pkt[sizeof(pkt)-4]);
    if (expected_checksum != got_checksum) {
        return false;
    }
    status = pkt[4];
    counter = pkt[5];
    timestamp_us = be64toh_ptr(&pkt[6]);
    memcpy(data, &pkt[14], length);
    return true;
}

/*
  write to a register
 */
bool AP_InertialSensor_IIM4623x::write_registers(uint8_t address, const uint8_t *data, uint8_t length)
{
    if (length > 8) {
        Debug("fail %u", __LINE__);
        return false;
    }
    hal.scheduler->delay(1);
    WITH_SEMAPHORE(dev->get_semaphore());
    uint8_t pkt[20] {};
    pkt[0] = BYTE_HEADER_CMD;
    pkt[1] = BYTE_HEADER_CMD;
    pkt[2] = 12+length;
    pkt[3] = CMD_TYPE_WRITE_REG;
    pkt[4] = 0; // reserved
    pkt[5] = length;
    pkt[6] = address;
    pkt[7] = 0; // page ID
    memcpy(&pkt[8], data, length);
    const uint16_t checksum = calc_checksum(&pkt[3], 5+length);
    put_be16_ptr(&pkt[8+length], checksum);
    pkt[10+length] = BYTE_FOOTER_1;
    pkt[11+length] = BYTE_FOOTER_2;

    dev->transfer(pkt, sizeof(pkt), nullptr, 0);

    wait_dready(500);

    uint8_t reply[10] {};
    dev->transfer(nullptr, 0, reply, sizeof(reply));

    if (reply[0] != BYTE_HEADER_REP || reply[1] != BYTE_HEADER_REP) {
        Debug("fail %u", __LINE__);
        return false;
    }
    if (reply[2] != sizeof(reply)) {
        Debug("fail %u", __LINE__);
        return false;
    }
    if (reply[sizeof(reply)-2] != BYTE_FOOTER_1 || reply[sizeof(reply)-1] != BYTE_FOOTER_2) {
        Debug("fail %u", __LINE__);
        return false;
    }
    const uint16_t expected_checksum = calc_checksum(&reply[3], sizeof(reply)-(3+4));
    const uint16_t got_checksum = be16toh_ptr(&reply[sizeof(reply)-4]);
    if (expected_checksum != got_checksum) {
        Debug("fail %u", __LINE__);
        return false;
    }
    if (reply[4] != 0) {
        Debug("fail %u 0x%x 0x%02x", __LINE__, unsigned(address), unsigned(reply[4]));
        return false;
    }
    return true;
}

bool AP_InertialSensor_IIM4623x::write_register8(uint8_t address, uint8_t value)
{
    return write_registers(address, &value, 1);
}

bool AP_InertialSensor_IIM4623x::write_register16(uint8_t address, uint16_t value)
{
    value = htobe16(value);
    return write_registers(address, (uint8_t*)&value, 2);
}

/*
  start streaming
 */
void AP_InertialSensor_IIM4623x::start_streaming(void)
{
    hal.scheduler->delay(1);

    WITH_SEMAPHORE(dev->get_semaphore());
    uint8_t pkt[20] {};
    pkt[0] = BYTE_HEADER_CMD;
    pkt[1] = BYTE_HEADER_CMD;
    pkt[2] = 8;
    pkt[3] = CMD_TYPE_START_STREAMING;
    pkt[4] = 0;
    pkt[5] = CMD_TYPE_START_STREAMING;
    pkt[6] = BYTE_FOOTER_1;
    pkt[7] = BYTE_FOOTER_2;

    dev->transfer(pkt, sizeof(pkt), nullptr, 0);
}

// uint8_t drdy_state;
bool AP_InertialSensor_IIM4623x::init()
{
    if (drdy_pin >= 0) {
        hal.gpio->pinMode(drdy_pin, HAL_GPIO_INPUT);
    }

    _clip_limit = 7.9f * GRAVITY_MSS;
    expected_sample_rate_hz = 1000;

    uint8_t whoami;
    if (!read_register(REG_WHO_AM_I, &whoami, 1)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IIM4623x: failed to read whoami");
        return false;
    }

    switch (whoami) {
    case IIM46230_WHO_AM_I: {
        dev_type = DEVTYPE_INS_IIM46230;
        dev_name = "IIM46230";
        break;
    }
    case IIM46234_WHO_AM_I: {
        dev_type = DEVTYPE_INS_IIM46234;
        dev_name = "IIM46234";
        break;
    }
    default:
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "IIM4623x: unknown whoami 0x%02x", unsigned(whoami));
        return false;
    }

#if HAL_GCS_ENABLED
    uint8_t major, minor;
    if (!read_version(major, minor)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s: failed to read version", dev_name);
        return false;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "%s: version %u.%u", dev_name, unsigned(major), unsigned(minor));
#endif

    // select accel, gyro and temperature
    write_register8(REG_SELECT_OUT_DATA, 0x7);

    dev->set_device_type(dev_type);

    return true;
}

/*
  sensor read loop
 */
void AP_InertialSensor_IIM4623x::loop(void)
{
    start_streaming();

    while (true) {
        wait_dready(1005);
        read_sensor();
    }
}

bool AP_InertialSensor_IIM4623x::update()
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}

