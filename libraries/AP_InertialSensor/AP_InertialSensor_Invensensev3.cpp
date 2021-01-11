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
/*
  driver for Invensensev3 IMUs

  Supported:
   ICM-40609
   ICM-42688
   ICM-42605

  Note that this sensor includes 32kHz internal sampling and an
  anti-aliasing filter, which means this driver can be a lot simpler
  than the Invensense and Invensensev2 drivers which need to handle
  8kHz sample rates to achieve decent aliasing protection

  The sensor is a multi-bank design (4 banks) but as this driver only
  needs access to the first bank and the default bank is the first
  bank we can treat it as a single bank design
 */

#include <AP_HAL/AP_HAL.h>
#include "AP_InertialSensor_Invensensev3.h"
#include <utility>

extern const AP_HAL::HAL& hal;

/*
  gyro as 16.4 LSB/DPS at scale factor of +/- 2000dps (FS_SEL==0)
 */
static const float GYRO_SCALE = (0.0174532f / 16.4f);

// set bit 0x80 in register ID for read on SPI
#define BIT_READ_FLAG                           0x80

// registers we use
#define INV3REG_WHOAMI        0x75
#define INV3REG_INT_CONFIG    0x14
#define INV3REG_FIFO_CONFIG   0x16
#define INV3REG_PWR_MGMT0     0x4e
#define INV3REG_GYRO_CONFIG0  0x4f
#define INV3REG_ACCEL_CONFIG0 0x50
#define INV3REG_FIFO_CONFIG1  0x5f
#define INV3REG_FIFO_CONFIG2  0x60
#define INV3REG_FIFO_CONFIG3  0x61
#define INV3REG_INT_SOURCE0   0x65
#define INV3REG_SIGNAL_PATH_RESET 0x4b
#define INV3REG_INTF_CONFIG0  0x4c
#define INV3REG_FIFO_COUNTH   0x2e
#define INV3REG_FIFO_DATA     0x30
#define INV3REG_BANK_SEL      0x76

// WHOAMI values
#define INV3_ID_ICM40609      0x3b
#define INV3_ID_ICM42605      0x42
#define INV3_ID_ICM42688      0x47

// run output data at 2kHz
#define INV3_ODR 2000

/*
  really nice that this sensor has an option to request little-endian
  data
 */
struct PACKED FIFOData {
    uint8_t header;
    int16_t accel[3];
    int16_t gyro[3];
    int8_t temperature;
    uint16_t timestamp;
};

#define INV3_SAMPLE_SIZE sizeof(FIFOData)
#define INV3_FIFO_BUFFER_LEN 8

AP_InertialSensor_Invensensev3::AP_InertialSensor_Invensensev3(AP_InertialSensor &imu,
                                                               AP_HAL::OwnPtr<AP_HAL::Device> _dev,
                                                               enum Rotation _rotation)
    : AP_InertialSensor_Backend(imu)
    , rotation(_rotation)
    , dev(std::move(_dev))
{
}

AP_InertialSensor_Invensensev3::~AP_InertialSensor_Invensensev3()
{
    if (fifo_buffer != nullptr) {
        hal.util->free_type((void*)fifo_buffer, INV3_FIFO_BUFFER_LEN * INV3_SAMPLE_SIZE, AP_HAL::Util::MEM_DMA_SAFE);
    }
}

AP_InertialSensor_Backend *AP_InertialSensor_Invensensev3::probe(AP_InertialSensor &imu,
                                                                 AP_HAL::OwnPtr<AP_HAL::Device> _dev,
                                                                 enum Rotation _rotation)
{
    if (!_dev) {
        return nullptr;
    }

    if (_dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI) {
        _dev->set_read_flag(BIT_READ_FLAG);
    }

    AP_InertialSensor_Invensensev3 *sensor =
        new AP_InertialSensor_Invensensev3(imu, std::move(_dev), _rotation);
    if (!sensor || !sensor->hardware_init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

void AP_InertialSensor_Invensensev3::fifo_reset()
{
    // FIFO_MODE stop-on-full
    register_write(INV3REG_FIFO_CONFIG, 0x80);
    // FIFO partial disable, enable accel, gyro, temperature
    register_write(INV3REG_FIFO_CONFIG1, 0x07);
    // little-endian, fifo count in records, last data hold for ODR mismatch
    register_write(INV3REG_INTF_CONFIG0, 0xC0);
    register_write(INV3REG_SIGNAL_PATH_RESET, 2);

    notify_accel_fifo_reset(accel_instance);
    notify_gyro_fifo_reset(gyro_instance);
}

void AP_InertialSensor_Invensensev3::start()
{
    WITH_SEMAPHORE(dev->get_semaphore());

    // initially run the bus at low speed
    dev->set_speed(AP_HAL::Device::SPEED_LOW);

    // always use FIFO
    fifo_reset();

    // grab the used instances
    enum DevTypes devtype;
    switch (inv3_type) {
    case Invensensev3_Type::ICM42688:
        devtype = DEVTYPE_INS_ICM42688;
        break;
    case Invensensev3_Type::ICM42605:
        devtype = DEVTYPE_INS_ICM42605;
        break;
    case Invensensev3_Type::ICM40609:
    default:
        devtype = DEVTYPE_INS_ICM40609;
        break;
    }

    gyro_instance = _imu.register_gyro(INV3_ODR, dev->get_bus_id_devtype(devtype));
    accel_instance = _imu.register_accel(INV3_ODR, dev->get_bus_id_devtype(devtype));

    // setup on-sensor filtering and scaling
    set_filter_and_scaling();

    // update backend sample rate
    _set_accel_raw_sample_rate(accel_instance, INV3_ODR);
    _set_gyro_raw_sample_rate(gyro_instance, INV3_ODR);

    // indicate what multiplier is appropriate for the sensors'
    // readings to fit them into an int16_t:
    _set_raw_sample_accel_multiplier(accel_instance, multiplier_accel);

    // now that we have initialised, we set the bus speed to high
    dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    // setup sensor rotations from probe()
    set_gyro_orientation(gyro_instance, rotation);
    set_accel_orientation(accel_instance, rotation);

    // allocate fifo buffer
    fifo_buffer = (FIFOData *)hal.util->malloc_type(INV3_FIFO_BUFFER_LEN * INV3_SAMPLE_SIZE, AP_HAL::Util::MEM_DMA_SAFE);
    if (fifo_buffer == nullptr) {
        AP_HAL::panic("Invensensev3: Unable to allocate FIFO buffer");
    }

    // start the timer process to read samples
    dev->register_periodic_callback(1e6 / INV3_ODR, FUNCTOR_BIND_MEMBER(&AP_InertialSensor_Invensensev3::read_fifo, void));
}

/*
  publish any pending data
 */
bool AP_InertialSensor_Invensensev3::update()
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    _publish_temperature(accel_instance, temp_filtered);

    return true;
}

/*
  accumulate new samples
 */
void AP_InertialSensor_Invensensev3::accumulate()
{
    // nothing to do
}

bool AP_InertialSensor_Invensensev3::accumulate_samples(const FIFOData *data, uint8_t n_samples)
{
    for (uint8_t i = 0; i < n_samples; i++) {
        const FIFOData &d = data[i];

        // we have a header to confirm we don't have FIFO corruption! no more mucking
        // about with the temperature registers
        if ((d.header & 0xF8) != 0x68) {
            // no or bad data
            return false;
        }

        Vector3f accel{float(d.accel[0]), float(d.accel[1]), float(d.accel[2])};
        Vector3f gyro{float(d.gyro[0]), float(d.gyro[1]), float(d.gyro[2])};

        accel *= accel_scale;
        gyro *= GYRO_SCALE;

        const float temp = d.temperature * temp_sensitivity + temp_zero;

        _rotate_and_correct_accel(accel_instance, accel);
        _rotate_and_correct_gyro(gyro_instance, gyro);

        _notify_new_accel_raw_sample(accel_instance, accel, 0);
        _notify_new_gyro_raw_sample(gyro_instance, gyro);

        temp_filtered = temp_filter.apply(temp);
    }
    return true;
}

/*
  timer function called at ODR rate
 */
void AP_InertialSensor_Invensensev3::read_fifo()
{
    bool need_reset = false;
    uint16_t n_samples;

    if (!block_read(INV3REG_FIFO_COUNTH, (uint8_t*)&n_samples, 2)) {
        goto check_registers;
    }

    if (n_samples == 0) {
        /* Not enough data in FIFO */
        goto check_registers;
    }

    while (n_samples > 0) {
        uint8_t n = MIN(n_samples, INV3_FIFO_BUFFER_LEN);
        if (!block_read(INV3REG_FIFO_DATA, (uint8_t*)fifo_buffer, n * INV3_SAMPLE_SIZE)) {
            goto check_registers;
        }

        if (!accumulate_samples(fifo_buffer, n)) {
            need_reset = true;
            break;
        }
        n_samples -= n;
    }

    if (need_reset) {
        fifo_reset();
    }
    
check_registers:
    // check next register value for correctness
    dev->set_speed(AP_HAL::Device::SPEED_LOW);
    if (!dev->check_next_register()) {
        _inc_gyro_error_count(gyro_instance);
        _inc_accel_error_count(accel_instance);
    }
    dev->set_speed(AP_HAL::Device::SPEED_HIGH);
}

bool AP_InertialSensor_Invensensev3::block_read(uint8_t reg, uint8_t *buf, uint32_t size)
{
    return dev->read_registers(reg, buf, size);
}

uint8_t AP_InertialSensor_Invensensev3::register_read(uint8_t reg)
{
    uint8_t val = 0;
    dev->read_registers(reg, &val, 1);
    return val;
}

void AP_InertialSensor_Invensensev3::register_write(uint8_t reg, uint8_t val, bool checked)
{
    dev->write_register(reg, val, checked);
}

/*
  set the filter frequencies and scaling
 */
void AP_InertialSensor_Invensensev3::set_filter_and_scaling(void)
{
    // enable gyro and accel in low-noise modes
    register_write(INV3REG_PWR_MGMT0, 0x0F);
    hal.scheduler->delay_microseconds(300);

    // setup gyro for 2kHz
    register_write(INV3REG_GYRO_CONFIG0, 0x05);

    // setup accel for 2kHz
    register_write(INV3REG_ACCEL_CONFIG0, 0x05);
}

/*
  check whoami for sensor type
 */
bool AP_InertialSensor_Invensensev3::check_whoami(void)
{
    uint8_t whoami = register_read(INV3REG_WHOAMI);
    switch (whoami) {
    case INV3_ID_ICM40609:
        inv3_type = Invensensev3_Type::ICM40609;
        accel_scale = (GRAVITY_MSS / 1024);
        return true;
    case INV3_ID_ICM42688:
        inv3_type = Invensensev3_Type::ICM42688;
        accel_scale = (GRAVITY_MSS / 2048);
        return true;
    case INV3_ID_ICM42605:
        inv3_type = Invensensev3_Type::ICM42605;
        accel_scale = (GRAVITY_MSS / 2048);
        return true;
    }
    // not a value WHOAMI result
    return false;
}

bool AP_InertialSensor_Invensensev3::hardware_init(void)
{
    WITH_SEMAPHORE(dev->get_semaphore());

    dev->setup_checked_registers(7, dev->bus_type() == AP_HAL::Device::BUS_TYPE_I2C?200:20);

    // initially run the bus at low speed
    dev->set_speed(AP_HAL::Device::SPEED_LOW);

    if (!check_whoami()) {
        return false;
    }

    dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    switch (inv3_type) {
    case Invensensev3_Type::ICM40609:
        _clip_limit = 29.5f * GRAVITY_MSS;
        break;
    case Invensensev3_Type::ICM42688:
    case Invensensev3_Type::ICM42605:
        _clip_limit = 15.5f * GRAVITY_MSS;
        break;
    }

    return true;
}
