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
 *       AP_Compass_HMC5843.cpp - Arduino Library for HMC5843 I2C magnetometer
 *       Code by Jordi Muñoz and Jose Julio. DIYDrones.com
 *
 *       Sensor is connected to I2C port
 *       Sensor is initialized in Continuos mode (10Hz)
 *
 */
#include <AP_HAL/AP_HAL.h>

#ifdef HAL_COMPASS_HMC5843_I2C_ADDR

#include <assert.h>
#include <utility>
#include <stdio.h>

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/sparse-endian.h>

#include "AP_Compass_HMC5843.h"
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_InertialSensor/AuxiliaryBus.h>

extern const AP_HAL::HAL& hal;

/*
 * Defaul address: 0x1E
 */

#define HMC5843_REG_CONFIG_A 0x00
// Valid sample averaging for 5883L
#define HMC5843_SAMPLE_AVERAGING_1 (0x00 << 5)
#define HMC5843_SAMPLE_AVERAGING_2 (0x01 << 5)
#define HMC5843_SAMPLE_AVERAGING_4 (0x02 << 5)
#define HMC5843_SAMPLE_AVERAGING_8 (0x03 << 5)

#define HMC5843_CONF_TEMP_ENABLE   (0x80)

// Valid data output rates for 5883L
#define HMC5843_OSR_0_75HZ (0x00 << 2)
#define HMC5843_OSR_1_5HZ  (0x01 << 2)
#define HMC5843_OSR_3HZ    (0x02 << 2)
#define HMC5843_OSR_7_5HZ  (0x03 << 2)
#define HMC5843_OSR_15HZ   (0x04 << 2)
#define HMC5843_OSR_30HZ   (0x05 << 2)
#define HMC5843_OSR_75HZ   (0x06 << 2)
// Sensor operation modes
#define HMC5843_OPMODE_NORMAL 0x00
#define HMC5843_OPMODE_POSITIVE_BIAS 0x01
#define HMC5843_OPMODE_NEGATIVE_BIAS 0x02
#define HMC5843_OPMODE_MASK 0x03

#define HMC5843_REG_CONFIG_B 0x01
#define HMC5883L_GAIN_0_88_GA (0x00 << 5)
#define HMC5883L_GAIN_1_30_GA (0x01 << 5)
#define HMC5883L_GAIN_1_90_GA (0x02 << 5)
#define HMC5883L_GAIN_2_50_GA (0x03 << 5)
#define HMC5883L_GAIN_4_00_GA (0x04 << 5)
#define HMC5883L_GAIN_4_70_GA (0x05 << 5)
#define HMC5883L_GAIN_5_60_GA (0x06 << 5)
#define HMC5883L_GAIN_8_10_GA (0x07 << 5)

#define HMC5843_GAIN_0_70_GA (0x00 << 5)
#define HMC5843_GAIN_1_00_GA (0x01 << 5)
#define HMC5843_GAIN_1_50_GA (0x02 << 5)
#define HMC5843_GAIN_2_00_GA (0x03 << 5)
#define HMC5843_GAIN_3_20_GA (0x04 << 5)
#define HMC5843_GAIN_3_80_GA (0x05 << 5)
#define HMC5843_GAIN_4_50_GA (0x06 << 5)
#define HMC5843_GAIN_6_50_GA (0x07 << 5)

#define HMC5843_REG_MODE 0x02
#define HMC5843_MODE_CONTINUOUS 0x00
#define HMC5843_MODE_SINGLE     0x01

#define HMC5843_REG_DATA_OUTPUT_X_MSB 0x03

#define HMC5843_REG_ID_A 0x0A


AP_Compass_HMC5843::AP_Compass_HMC5843(AP_HMC5843_BusDriver *bus,
                                       bool force_external, enum Rotation rotation)
    : _bus(bus)
    , _rotation(rotation)
    , _force_external(force_external)
{
}

AP_Compass_HMC5843::~AP_Compass_HMC5843()
{
    delete _bus;
}

AP_Compass_Backend *AP_Compass_HMC5843::probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                              bool force_external,
                                              enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    AP_HMC5843_BusDriver *bus = new AP_HMC5843_BusDriver_HALDevice(std::move(dev));
    if (!bus) {
        return nullptr;
    }

    AP_Compass_HMC5843 *sensor = new AP_Compass_HMC5843(bus, force_external, rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

AP_Compass_Backend *AP_Compass_HMC5843::probe_mpu6000(enum Rotation rotation)
{
    AP_InertialSensor &ins = *AP_InertialSensor::get_singleton();

    AP_HMC5843_BusDriver *bus =
        new AP_HMC5843_BusDriver_Auxiliary(ins, HAL_INS_MPU60XX_SPI,
                                           HAL_COMPASS_HMC5843_I2C_ADDR);
    if (!bus) {
        return nullptr;
    }

    AP_Compass_HMC5843 *sensor = new AP_Compass_HMC5843(bus, false, rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

bool AP_Compass_HMC5843::init()
{
    AP_HAL::Semaphore *bus_sem = _bus->get_semaphore();

    if (!bus_sem) {
        hal.console->printf("HMC5843: Unable to get bus semaphore\n");
        return false;
    }
    bus_sem->take_blocking();

    // high retries for init
    _bus->set_retries(10);
    
    if (!_bus->configure()) {
        hal.console->printf("HMC5843: Could not configure the bus\n");
        goto errout;
    }

    if (!_check_whoami()) {
        goto errout;
    }

    if (!_calibrate()) {
        hal.console->printf("HMC5843: Could not calibrate sensor\n");
        goto errout;
    }

    if (!_setup_sampling_mode()) {
        goto errout;
    }

    if (!_bus->start_measurements()) {
        hal.console->printf("HMC5843: Could not start measurements on bus\n");
        goto errout;
    }

    _initialised = true;

    // lower retries for run
    _bus->set_retries(3);
    
    bus_sem->give();

    // perform an initial read
    read();

    _compass_instance = register_compass();

    set_rotation(_compass_instance, _rotation);
    
    _bus->set_device_type(DEVTYPE_HMC5883);
    set_dev_id(_compass_instance, _bus->get_bus_id());

    if (_force_external) {
        set_external(_compass_instance, true);
    }

    // read from sensor at 75Hz
    _bus->register_periodic_callback(13333,
                                     FUNCTOR_BIND_MEMBER(&AP_Compass_HMC5843::_timer, void));

    hal.console->printf("HMC5843 found on bus 0x%x\n", (unsigned)_bus->get_bus_id());
    
    return true;

errout:
    bus_sem->give();
    return false;
}

/*
 * take a reading from the magnetometer
 *
 * bus semaphore has been taken already by HAL
 */
void AP_Compass_HMC5843::_timer()
{
    bool result = _read_sample();

    // always ask for a new sample
    _take_sample();
    
    if (!result) {
        return;
    }

    // get raw_field - sensor frame, uncorrected
    Vector3f raw_field = Vector3f(_mag_x, _mag_y, _mag_z);
    raw_field *= _gain_scale;

    // rotate to the desired orientation
    if (is_external(_compass_instance)) {
        raw_field.rotate(ROTATION_YAW_90);
    }

    // We expect to do reads at 10Hz, and  we get new data at most 75Hz, so we
    // don't expect to accumulate more than 8 before a read; let's make it
    // 14 to give more room for the initialization phase
    accumulate_sample(raw_field, _compass_instance, 14);
}

/*
 * Take accumulated reads from the magnetometer or try to read once if no
 * valid data
 *
 * bus semaphore must not be locked
 */
void AP_Compass_HMC5843::read()
{
    if (!_initialised) {
        // someone has tried to enable a compass for the first time
        // mid-flight .... we can't do that yet (especially as we won't
        // have the right orientation!)
        return;
    }

    drain_accumulated_samples(_compass_instance, &_scaling);
}

bool AP_Compass_HMC5843::_setup_sampling_mode()
{
    _gain_scale = (1.0f / 1090) * 1000;
    if (!_bus->register_write(HMC5843_REG_CONFIG_A,
                              HMC5843_CONF_TEMP_ENABLE |
                              HMC5843_OSR_75HZ |
                              HMC5843_SAMPLE_AVERAGING_1) ||
        !_bus->register_write(HMC5843_REG_CONFIG_B,
                              HMC5883L_GAIN_1_30_GA) ||
        !_bus->register_write(HMC5843_REG_MODE,
                              HMC5843_MODE_SINGLE)) {
        return false;
    }
    return true;
}

/*
 * Read Sensor data - bus semaphore must be taken
 */
bool AP_Compass_HMC5843::_read_sample()
{
    struct PACKED {
        be16_t rx;
        be16_t ry;
        be16_t rz;
    } val;
    int16_t rx, ry, rz;

    if (!_bus->block_read(HMC5843_REG_DATA_OUTPUT_X_MSB, (uint8_t *) &val, sizeof(val))){
        return false;
    }

    rx = be16toh(val.rx);
    ry = be16toh(val.rz);
    rz = be16toh(val.ry);

    if (rx == -4096 || ry == -4096 || rz == -4096) {
        // no valid data available
        return false;
    }

    _mag_x = -rx;
    _mag_y =  ry;
    _mag_z = -rz;

    return true;
}


/*
  ask for a new oneshot sample
 */
void AP_Compass_HMC5843::_take_sample()
{
    _bus->register_write(HMC5843_REG_MODE,
                         HMC5843_MODE_SINGLE);
}

bool AP_Compass_HMC5843::_check_whoami()
{
    uint8_t id[3];
    if (!_bus->block_read(HMC5843_REG_ID_A, id, 3)) {
        // can't talk on bus
        return false;        
    }
    if (id[0] != 'H' ||
        id[1] != '4' ||
        id[2] != '3') {
        // not a HMC5x83 device
        return false;
    }

    return true;
}

bool AP_Compass_HMC5843::_calibrate()
{
    uint8_t calibration_gain;
    int numAttempts = 0, good_count = 0;
    bool success = false;

    calibration_gain = HMC5883L_GAIN_2_50_GA;

    /*
     * the expected values are based on observation of real sensors
     */
	float expected[3] = { 1.16*600, 1.08*600, 1.16*600 };

    uint8_t base_config = HMC5843_OSR_15HZ;
    uint8_t num_samples = 0;
    
    while (success == 0 && numAttempts < 25 && good_count < 5) {
        numAttempts++;

        // force positiveBias (compass should return 715 for all channels)
        if (!_bus->register_write(HMC5843_REG_CONFIG_A,
                                  base_config | HMC5843_OPMODE_POSITIVE_BIAS)) {
            // compass not responding on the bus
            continue;
        }

        hal.scheduler->delay(50);

        // set gains
        if (!_bus->register_write(HMC5843_REG_CONFIG_B, calibration_gain) ||
            !_bus->register_write(HMC5843_REG_MODE, HMC5843_MODE_SINGLE)) {
            continue;
        }

        // read values from the compass
        hal.scheduler->delay(50);
        if (!_read_sample()) {
            // we didn't read valid values
            continue;
        }

        num_samples++;

        float cal[3];

        // hal.console->printf("mag %d %d %d\n", _mag_x, _mag_y, _mag_z);

        cal[0] = fabsf(expected[0] / _mag_x);
        cal[1] = fabsf(expected[1] / _mag_y);
        cal[2] = fabsf(expected[2] / _mag_z);

        // hal.console->printf("cal=%.2f %.2f %.2f\n", cal[0], cal[1], cal[2]);

        // we throw away the first two samples as the compass may
        // still be changing its state from the application of the
        // strap excitation. After that we accept values in a
        // reasonable range
        if (numAttempts <= 2) {
            continue;
        }

#define IS_CALIBRATION_VALUE_VALID(val) (val > 0.7f && val < 1.35f)

        if (IS_CALIBRATION_VALUE_VALID(cal[0]) &&
            IS_CALIBRATION_VALUE_VALID(cal[1]) &&
            IS_CALIBRATION_VALUE_VALID(cal[2])) {
            // hal.console->printf("car=%.2f %.2f %.2f good\n", cal[0], cal[1], cal[2]);
            good_count++;

            _scaling[0] += cal[0];
            _scaling[1] += cal[1];
            _scaling[2] += cal[2];
        }

#undef IS_CALIBRATION_VALUE_VALID

#if 0
        /* useful for debugging */
        hal.console->printf("MagX: %d MagY: %d MagZ: %d\n", (int)_mag_x, (int)_mag_y, (int)_mag_z);
        hal.console->printf("CalX: %.2f CalY: %.2f CalZ: %.2f\n", cal[0], cal[1], cal[2]);
#endif
    }

    _bus->register_write(HMC5843_REG_CONFIG_A, base_config);
    
    if (good_count >= 5) {
        _scaling[0] = _scaling[0] / good_count;
        _scaling[1] = _scaling[1] / good_count;
        _scaling[2] = _scaling[2] / good_count;
        success = true;
    } else {
        /* best guess */
        _scaling[0] = 1.0;
        _scaling[1] = 1.0;
        _scaling[2] = 1.0;
        if (num_samples > 5) {
            // a sensor can be broken for calibration but still
            // otherwise workable, accept it if we are reading samples
            success = true;
        }
    }

#if 0
    printf("scaling: %.2f %.2f %.2f\n",
           _scaling[0], _scaling[1], _scaling[2]);
#endif
    
    return success;
}

/* AP_HAL::Device implementation of the HMC5843 */
AP_HMC5843_BusDriver_HALDevice::AP_HMC5843_BusDriver_HALDevice(AP_HAL::OwnPtr<AP_HAL::Device> dev)
    : _dev(std::move(dev))
{
    // set read and auto-increment flags on SPI
    if (_dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI) {
        _dev->set_read_flag(0xC0);
    }
}

bool AP_HMC5843_BusDriver_HALDevice::block_read(uint8_t reg, uint8_t *buf, uint32_t size)
{
    return _dev->read_registers(reg, buf, size);
}

bool AP_HMC5843_BusDriver_HALDevice::register_read(uint8_t reg, uint8_t *val)
{
    return _dev->read_registers(reg, val, 1);
}

bool AP_HMC5843_BusDriver_HALDevice::register_write(uint8_t reg, uint8_t val)
{
    return _dev->write_register(reg, val);
}

AP_HAL::Semaphore *AP_HMC5843_BusDriver_HALDevice::get_semaphore()
{
    return _dev->get_semaphore();
}

AP_HAL::Device::PeriodicHandle AP_HMC5843_BusDriver_HALDevice::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    return _dev->register_periodic_callback(period_usec, cb);
}


/* HMC5843 on an auxiliary bus of IMU driver */
AP_HMC5843_BusDriver_Auxiliary::AP_HMC5843_BusDriver_Auxiliary(AP_InertialSensor &ins, uint8_t backend_id,
                                                               uint8_t addr)
{
    /*
     * Only initialize members. Fails are handled by configure or while
     * getting the semaphore
     */
    _bus = ins.get_auxiliary_bus(backend_id);
    if (!_bus) {
        return;
    }

    _slave = _bus->request_next_slave(addr);
}

AP_HMC5843_BusDriver_Auxiliary::~AP_HMC5843_BusDriver_Auxiliary()
{
    /* After started it's owned by AuxiliaryBus */
    if (!_started) {
        delete _slave;
    }
}

bool AP_HMC5843_BusDriver_Auxiliary::block_read(uint8_t reg, uint8_t *buf, uint32_t size)
{
    if (_started) {
        /*
         * We can only read a block when reading the block of sample values -
         * calling with any other value is a mistake
         */
        assert(reg == HMC5843_REG_DATA_OUTPUT_X_MSB);

        int n = _slave->read(buf);
        return n == static_cast<int>(size);
    }

    int r = _slave->passthrough_read(reg, buf, size);

    return r > 0 && static_cast<uint32_t>(r) == size;
}

bool AP_HMC5843_BusDriver_Auxiliary::register_read(uint8_t reg, uint8_t *val)
{
    return _slave->passthrough_read(reg, val, 1) == 1;
}

bool AP_HMC5843_BusDriver_Auxiliary::register_write(uint8_t reg, uint8_t val)
{
    return _slave->passthrough_write(reg, val) == 1;
}

AP_HAL::Semaphore *AP_HMC5843_BusDriver_Auxiliary::get_semaphore()
{
    return _bus->get_semaphore();
}


bool AP_HMC5843_BusDriver_Auxiliary::configure()
{
    if (!_bus || !_slave) {
        return false;
    }
    return true;
}

bool AP_HMC5843_BusDriver_Auxiliary::start_measurements()
{
    if (_bus->register_periodic_read(_slave, HMC5843_REG_DATA_OUTPUT_X_MSB, 6) < 0) {
        return false;
    }

    _started = true;

    return true;
}

AP_HAL::Device::PeriodicHandle AP_HMC5843_BusDriver_Auxiliary::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    return _bus->register_periodic_callback(period_usec, cb);
}

// set device type within a device class
void AP_HMC5843_BusDriver_Auxiliary::set_device_type(uint8_t devtype)
{
    _bus->set_device_type(devtype);
}

// return 24 bit bus identifier
uint32_t AP_HMC5843_BusDriver_Auxiliary::get_bus_id(void) const
{
    return _bus->get_bus_id();
}

#endif
