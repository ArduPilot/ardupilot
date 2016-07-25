/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

AP_Compass_HMC5843::AP_Compass_HMC5843(Compass &compass, AP_HMC5843_BusDriver *bus,
                                       bool force_external)
    : AP_Compass_Backend(compass)
    , _bus(bus)
    , _force_external(force_external)
{
}

AP_Compass_HMC5843::~AP_Compass_HMC5843()
{
    delete _bus;
}

AP_Compass_Backend *AP_Compass_HMC5843::probe(Compass &compass,
                                              AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                              bool force_external)
{
    AP_HMC5843_BusDriver *bus = new AP_HMC5843_BusDriver_HALDevice(std::move(dev));
    if (!bus) {
        return nullptr;
    }

    AP_Compass_HMC5843 *sensor = new AP_Compass_HMC5843(compass, bus, force_external);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

AP_Compass_Backend *AP_Compass_HMC5843::probe_mpu6000(Compass &compass)
{
    AP_InertialSensor &ins = *AP_InertialSensor::get_instance();

    AP_HMC5843_BusDriver *bus =
        new AP_HMC5843_BusDriver_Auxiliary(ins, HAL_INS_MPU60XX_SPI,
                                           HAL_COMPASS_HMC5843_I2C_ADDR);
    if (!bus) {
        return nullptr;
    }

    AP_Compass_HMC5843 *sensor = new AP_Compass_HMC5843(compass, bus, false);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

bool AP_Compass_HMC5843::init()
{
    hal.scheduler->suspend_timer_procs();
    AP_HAL::Semaphore *bus_sem = _bus->get_semaphore();

    if (!bus_sem || !bus_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        hal.console->printf("HMC5843: Unable to get bus semaphore\n");
        goto fail_sem;
    }

    if (!_bus->configure()) {
        hal.console->printf("HMC5843: Could not configure the bus\n");
        goto errout;
    }

    if (!_detect_version()) {
        hal.console->printf("HMC5843: Could not detect version\n");
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

    bus_sem->give();
    hal.scheduler->resume_timer_procs();

    // perform an initial read
    read();

    _compass_instance = register_compass();
    set_dev_id(_compass_instance, _product_id);

    if (_force_external) {
        set_external(_compass_instance, true);
    }

    return true;

errout:
    bus_sem->give();

fail_sem:
    hal.scheduler->resume_timer_procs();

    return false;
}

/*
 * Accumulate a reading from the magnetometer
 *
 * bus semaphore must not be taken
 */
void AP_Compass_HMC5843::accumulate()
{
    if (!_initialised) {
        // someone has tried to enable a compass for the first time
        // mid-flight .... we can't do that yet (especially as we won't
        // have the right orientation!)
        return;
    }

   uint32_t tnow = AP_HAL::micros();
   if (_accum_count != 0 && (tnow - _last_accum_time) < 13333) {
	  // the compass gets new data at 75Hz
	  return;
   }

   if (!_bus->get_semaphore()->take(1)) {
       // the bus is busy - try again later
       return;
   }

   bool result = _read_sample();

   _bus->get_semaphore()->give();

   if (!result) {
       return;
   }

   // the _mag_N values are in the range -2048 to 2047, so we can
   // accumulate up to 15 of them in an int16_t. Let's make it 14
   // for ease of calculation. We expect to do reads at 10Hz, and
   // we get new data at most 75Hz, so we don't expect to
   // accumulate more than 8 before a read
   // get raw_field - sensor frame, uncorrected
   Vector3f raw_field = Vector3f(_mag_x, _mag_y, _mag_z);
   raw_field *= _gain_scale;

   // rotate to the desired orientation
   if (is_external(_compass_instance) &&
       _product_id == AP_COMPASS_TYPE_HMC5883L) {
       raw_field.rotate(ROTATION_YAW_90);
   }

   // rotate raw_field from sensor frame to body frame
   rotate_field(raw_field, _compass_instance);

   // publish raw_field (uncorrected point sample) for calibration use
   publish_raw_field(raw_field, tnow, _compass_instance);

   // correct raw_field for known errors
   correct_field(raw_field, _compass_instance);

   _mag_x_accum += raw_field.x;
   _mag_y_accum += raw_field.y;
   _mag_z_accum += raw_field.z;
   _accum_count++;
   if (_accum_count == 14) {
       _mag_x_accum /= 2;
       _mag_y_accum /= 2;
       _mag_z_accum /= 2;
       _accum_count = 7;
   }
   _last_accum_time = tnow;
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

    if (_accum_count == 0) {
       accumulate();
       if (_retry_time != 0) {
          return;
       }
    }

    Vector3f field(_mag_x_accum * _scaling[0],
                   _mag_y_accum * _scaling[1],
                   _mag_z_accum * _scaling[2]);
    field /= _accum_count;

    _accum_count = 0;
    _mag_x_accum = _mag_y_accum = _mag_z_accum = 0;

    publish_filtered_field(field, _compass_instance);
}

bool AP_Compass_HMC5843::_setup_sampling_mode()
{
    if (!_bus->register_write(HMC5843_REG_CONFIG_A, _base_config) ||
        !_bus->register_write(HMC5843_REG_CONFIG_B, _gain_config) ||
        !_bus->register_write(HMC5843_REG_MODE, HMC5843_MODE_CONTINUOUS)) {
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

    if (_retry_time > AP_HAL::millis()) {
        return false;
    }

    if (!_bus->block_read(HMC5843_REG_DATA_OUTPUT_X_MSB, (uint8_t *) &val, sizeof(val))){
        _retry_time = AP_HAL::millis() + 1000;
        return false;
    }

    rx = be16toh(val.rx);
    ry = be16toh(val.ry);
    rz = be16toh(val.rz);

    if (_product_id == AP_COMPASS_TYPE_HMC5883L) {
        std::swap(ry, rz);
    }

    if (rx == -4096 || ry == -4096 || rz == -4096) {
        // no valid data available
        return false;
    }

    _mag_x = -rx;
    _mag_y =  ry;
    _mag_z = -rz;

    _retry_time = 0;

    return true;
}

bool AP_Compass_HMC5843::_detect_version()
{
    _base_config = 0x0;

    uint8_t try_config = HMC5843_SAMPLE_AVERAGING_8 | HMC5843_OSR_75HZ | HMC5843_OPMODE_NORMAL;
    if (!_bus->register_write(HMC5843_REG_CONFIG_A, try_config) ||
        !_bus->register_read(HMC5843_REG_CONFIG_A, &_base_config)) {
        return false;
    }

    if (_base_config == try_config) {
        /* a 5883L supports the sample averaging config */
        _product_id = AP_COMPASS_TYPE_HMC5883L;
        _gain_config = HMC5883L_GAIN_1_30_GA;
        _gain_scale = (1.0f / 1090) * 1000;
    } else if (_base_config == (HMC5843_OPMODE_NORMAL | HMC5843_OSR_75HZ)) {
        _product_id = AP_COMPASS_TYPE_HMC5843;
        _gain_config = HMC5843_GAIN_1_00_GA;
        _gain_scale = (1.0f / 1300) * 1000;
    } else {
        /* not behaving like either supported compass type */
        return false;
    }

    return true;
}

bool AP_Compass_HMC5843::_calibrate()
{
    uint8_t calibration_gain;
    uint16_t expected_x;
    uint16_t expected_yz;
    int numAttempts = 0, good_count = 0;
    bool success = false;

    if (_product_id == AP_COMPASS_TYPE_HMC5883L) {
        calibration_gain = HMC5883L_GAIN_2_50_GA;
        /*
         * note that the HMC5883 datasheet gives the x and y expected
         * values as 766 and the z as 713. Experiments have shown the x
         * axis is around 766, and the y and z closer to 713.
         */
        expected_x = 766;
        expected_yz  = 713;
    } else {
        calibration_gain = HMC5843_GAIN_1_00_GA;
        expected_x = 715;
        expected_yz = 715;
    }

    uint8_t old_config = _base_config & ~(HMC5843_OPMODE_MASK);

    while (success == 0 && numAttempts < 25 && good_count < 5) {
        numAttempts++;

        // force positiveBias (compass should return 715 for all channels)
        if (!_bus->register_write(HMC5843_REG_CONFIG_A,
                                  old_config | HMC5843_OPMODE_POSITIVE_BIAS)) {
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

        float cal[3];

        // hal.console->printf("mag %d %d %d\n", _mag_x, _mag_y, _mag_z);

        cal[0] = fabsf(expected_x / (float)_mag_x);
        cal[1] = fabsf(expected_yz / (float)_mag_y);
        cal[2] = fabsf(expected_yz / (float)_mag_z);

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
    }

    return success;
}

/* AP_HAL::I2CDevice implementation of the HMC5843 */
AP_HMC5843_BusDriver_HALDevice::AP_HMC5843_BusDriver_HALDevice(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : _dev(std::move(dev))
{
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

#endif
