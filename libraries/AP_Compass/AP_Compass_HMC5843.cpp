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
 *       Sensor is conected to I2C port
 *       Sensor is initialized in Continuos mode (10Hz)
 *
 */

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>

#include "AP_Compass_HMC5843.h"
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_InertialSensor/AuxiliaryBus.h>

extern const AP_HAL::HAL& hal;

#define HMC5843_I2C_ADDR     0x1E
#define ConfigRegA           0x00
#define ConfigRegB           0x01
#define magGain              0x20
#define PositiveBiasConfig   0x11
#define NegativeBiasConfig   0x12
#define NormalOperation      0x10
#define ModeRegister         0x02
#define ContinuousConversion 0x00
#define SingleConversion     0x01

// ConfigRegA valid sample averaging for 5883L
#define SampleAveraging_1    0x00
#define SampleAveraging_2    0x01
#define SampleAveraging_4    0x02
#define SampleAveraging_8    0x03

// ConfigRegA valid data output rates for 5883L
#define DataOutputRate_0_75HZ 0x00
#define DataOutputRate_1_5HZ  0x01
#define DataOutputRate_3HZ    0x02
#define DataOutputRate_7_5HZ  0x03
#define DataOutputRate_15HZ   0x04
#define DataOutputRate_30HZ   0x05
#define DataOutputRate_75HZ   0x06

// constructor
AP_Compass_HMC5843::AP_Compass_HMC5843(Compass &compass, AP_HMC5843_SerialBus *bus) :
    AP_Compass_Backend(compass),
    _bus(bus),
    _retry_time(0),
    _mag_x(0),
    _mag_y(0),
    _mag_z(0),
    _mag_x_accum(0),
    _mag_y_accum(0),
    _mag_z_accum(0),
    _accum_count(0),
    _last_accum_time(0),
    _compass_instance(0),
    _product_id(0)
{}

AP_Compass_HMC5843::~AP_Compass_HMC5843()
{
    delete _bus;
}

// detect the sensor
AP_Compass_Backend *AP_Compass_HMC5843::detect_i2c(Compass &compass,
                                                   AP_HAL::I2CDriver *i2c)
{
    AP_HMC5843_SerialBus *bus = new AP_HMC5843_SerialBus_I2C(i2c, HMC5843_I2C_ADDR);
    if (!bus)
        return nullptr;
    return _detect(compass, bus);
}

AP_Compass_Backend *AP_Compass_HMC5843::detect_mpu6000(Compass &compass)
{
    AP_InertialSensor &ins = *AP_InertialSensor::get_instance();
    AP_HMC5843_SerialBus *bus = new AP_HMC5843_SerialBus_MPU6000(ins, HMC5843_I2C_ADDR);
    if (!bus)
        return nullptr;
    return _detect(compass, bus);
}

AP_Compass_Backend *AP_Compass_HMC5843::_detect(Compass &compass,
                                                AP_HMC5843_SerialBus *bus)
{
    AP_Compass_HMC5843 *sensor = new AP_Compass_HMC5843(compass, bus);
    if (!sensor) {
        delete bus;
        return nullptr;
    }
    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

// read_register - read a register value
bool AP_Compass_HMC5843::read_register(uint8_t address, uint8_t *value)
{
    if (_bus->register_read(address, value) != 0) {
        _retry_time = AP_HAL::millis() + 1000;
        return false;
    }
    return true;
}

// write_register - update a register value
bool AP_Compass_HMC5843::write_register(uint8_t address, uint8_t value)
{
    if (_bus->register_write(address, value) != 0) {
        _retry_time = AP_HAL::millis() + 1000;
        return false;
    }
    return true;
}

// Read Sensor data
bool AP_Compass_HMC5843::read_raw()
{
    struct AP_HMC5843_SerialBus::raw_value rv;

    if (_bus->read_raw(&rv) != 0) {
        _bus->set_high_speed(false);
        _retry_time = AP_HAL::millis() + 1000;
        return false;
    }

    int16_t rx, ry, rz;
    rx = (((int16_t)rv.val[0]) << 8) | rv.val[1];
    if (_product_id == AP_COMPASS_TYPE_HMC5883L) {
        rz = (((int16_t)rv.val[2]) << 8) | rv.val[3];
        ry = (((int16_t)rv.val[4]) << 8) | rv.val[5];
    } else {
        ry = (((int16_t)rv.val[2]) << 8) | rv.val[3];
        rz = (((int16_t)rv.val[4]) << 8) | rv.val[5];
    }
    if (rx == -4096 || ry == -4096 || rz == -4096) {
        // no valid data available
        return false;
    }

    _mag_x = -rx;
    _mag_y =  ry;
    _mag_z = -rz;

    return true;
}


// accumulate a reading from the magnetometer
void AP_Compass_HMC5843::accumulate(void)
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

   if (!_bus_sem->take(1)) {
       // the bus is busy - try again later
       return;
   }
   bool result = read_raw();
   _bus_sem->give();

   if (result) {
	  // the _mag_N values are in the range -2048 to 2047, so we can
	  // accumulate up to 15 of them in an int16_t. Let's make it 14
	  // for ease of calculation. We expect to do reads at 10Hz, and
	  // we get new data at most 75Hz, so we don't expect to
	  // accumulate more than 8 before a read
       // get raw_field - sensor frame, uncorrected
       Vector3f raw_field = Vector3f(_mag_x, _mag_y, _mag_z);
       raw_field *= _gain_multiple;

       // rotate raw_field from sensor frame to body frame
       rotate_field(raw_field, _compass_instance);

       // publish raw_field (uncorrected point sample) for calibration use
       publish_raw_field(raw_field, tnow, _compass_instance);

       // correct raw_field for known errors
       correct_field(raw_field, _compass_instance);

       // publish raw_field (corrected point sample) for EKF use
       publish_unfiltered_field(raw_field, tnow, _compass_instance);

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
}


/*
 *  re-initialise after a IO error
 */
bool AP_Compass_HMC5843::re_initialise()
{
    if (!write_register(ConfigRegA, _base_config) ||
        !write_register(ConfigRegB, magGain) ||
        !write_register(ModeRegister, ContinuousConversion))
        return false;
    return true;
}


bool AP_Compass_HMC5843::_detect_version()
{
    _base_config = 0x0;

    if (!write_register(ConfigRegA, SampleAveraging_8<<5 | DataOutputRate_75HZ<<2 | NormalOperation) ||
        !read_register(ConfigRegA, &_base_config)) {
        return false;
    }
    if (_base_config == (SampleAveraging_8<<5 | DataOutputRate_75HZ<<2 | NormalOperation)) {
        /* a 5883L supports the sample averaging config */
        _product_id = AP_COMPASS_TYPE_HMC5883L;
        return true;
    } else if (_base_config == (NormalOperation | DataOutputRate_75HZ<<2)) {
        _product_id = AP_COMPASS_TYPE_HMC5843;
        return true;
    } else {
        /* not behaving like either supported compass type */
        return false;
    }
}

// Public Methods //////////////////////////////////////////////////////////////
bool
AP_Compass_HMC5843::init()
{
    uint8_t calibration_gain = 0x20;
    uint16_t expected_x = 715;
    uint16_t expected_yz = 715;
    _gain_multiple = (1.0f / 1300) * 1000;

    _bus_sem = _bus->get_semaphore();
    hal.scheduler->suspend_timer_procs();

    if (!_bus_sem || !_bus_sem->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
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

    if (_product_id == AP_COMPASS_TYPE_HMC5883L) {
        calibration_gain = 0x60;
        /*
          note that the HMC5883 datasheet gives the x and y expected
          values as 766 and the z as 713. Experiments have shown the x
          axis is around 766, and the y and z closer to 713.
         */
        expected_x = 766;
        expected_yz  = 713;
        _gain_multiple = (1.0f / 1090) * 1000;
    }

    if (!_calibrate(calibration_gain, expected_x, expected_yz)) {
        hal.console->printf("HMC5843: Could not calibrate sensor\n");
        goto errout;
    }

    // leave test mode
    if (!re_initialise()) {
        goto errout;
    }

    if (!_bus->start_measurements()) {
        hal.console->printf("HMC5843: Could not start measurements on bus\n");
        goto errout;
    }
    _initialised = true;

    _bus_sem->give();
    hal.scheduler->resume_timer_procs();

    // perform an initial read
    read();

#if 0
    hal.console->printf("CalX: %.2f CalY: %.2f CalZ: %.2f\n",
                          _scaling[0], _scaling[1], _scaling[2]);
#endif

    _compass_instance = register_compass();
    set_dev_id(_compass_instance, _product_id);

#if CONFIG_HAL_BOARD == HAL_BOARD_LINUX && CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_RASPILOT
    set_external(_compass_instance, true);
#endif

    return true;

errout:
    _bus_sem->give();
fail_sem:
    hal.scheduler->resume_timer_procs();
    return false;
}

bool AP_Compass_HMC5843::_calibrate(uint8_t calibration_gain,
        uint16_t expected_x,
        uint16_t expected_yz)
{
    int numAttempts = 0, good_count = 0;
    bool success = false;

    while (success == 0 && numAttempts < 25 && good_count < 5)
    {
        numAttempts++;

        // force positiveBias (compass should return 715 for all channels)
        if (!write_register(ConfigRegA, PositiveBiasConfig))
            continue;   // compass not responding on the bus

        hal.scheduler->delay(50);

        // set gains
        if (!write_register(ConfigRegB, calibration_gain) ||
            !write_register(ModeRegister, SingleConversion))
            continue;

        // read values from the compass
        hal.scheduler->delay(50);
        if (!read_raw())
            continue;      // we didn't read valid values

        hal.scheduler->delay(10);

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

// Read Sensor data
void AP_Compass_HMC5843::read()
{
    if (!_initialised) {
        // someone has tried to enable a compass for the first time
        // mid-flight .... we can't do that yet (especially as we won't
        // have the right orientation!)
        return;
    }
    if (_retry_time != 0) {
        if (AP_HAL::millis() < _retry_time) {
            return;
        }
        if (!re_initialise()) {
            _retry_time = AP_HAL::millis() + 1000;
            _bus->set_high_speed(false);
            return;
        }
    }

    if (_accum_count == 0) {
       accumulate();
       if (_retry_time != 0) {
          _bus->set_high_speed(false);
          return;
       }
    }

    Vector3f field(_mag_x_accum * _scaling[0],
                   _mag_y_accum * _scaling[1],
                   _mag_z_accum * _scaling[2]);
    field /= _accum_count;

    _accum_count = 0;
    _mag_x_accum = _mag_y_accum = _mag_z_accum = 0;

    // rotate to the desired orientation
    if (_product_id == AP_COMPASS_TYPE_HMC5883L) {
        field.rotate(ROTATION_YAW_90);
    }

    publish_filtered_field(field, _compass_instance);
    _retry_time = 0;
}

/* I2C implementation of the HMC5843 */
AP_HMC5843_SerialBus_I2C::AP_HMC5843_SerialBus_I2C(AP_HAL::I2CDriver *i2c, uint8_t addr)
    : _i2c(i2c)
    , _addr(addr)
{
}

void AP_HMC5843_SerialBus_I2C::set_high_speed(bool val)
{
    _i2c->setHighSpeed(val);
}

uint8_t AP_HMC5843_SerialBus_I2C::register_read(uint8_t reg, uint8_t *buf, uint8_t size)
{
    return _i2c->readRegisters(_addr, reg, size, buf);
}

uint8_t AP_HMC5843_SerialBus_I2C::register_write(uint8_t reg, uint8_t val)
{
    return _i2c->writeRegister(_addr, reg, val);
}

AP_HAL::Semaphore* AP_HMC5843_SerialBus_I2C::get_semaphore()
{
    return _i2c->get_semaphore();
}

uint8_t AP_HMC5843_SerialBus_I2C::read_raw(struct raw_value *rv)
{
    return register_read(0x03, (uint8_t*)rv, sizeof(*rv));
}


/* MPU6000 implementation of the HMC5843 */
AP_HMC5843_SerialBus_MPU6000::AP_HMC5843_SerialBus_MPU6000(AP_InertialSensor &ins,
                                                           uint8_t addr)
{
    // Only initialize members. Fails are handled by configure or while
    // getting the semaphore
    _bus = ins.get_auxiliary_bus(HAL_INS_MPU60XX_SPI);
    if (!_bus)
        return;
    _slave = _bus->request_next_slave(addr);
}

AP_HMC5843_SerialBus_MPU6000::~AP_HMC5843_SerialBus_MPU6000()
{
    /* After started it's owned by AuxiliaryBus */
    if (!_started)
        delete _slave;
}

bool AP_HMC5843_SerialBus_MPU6000::configure()
{
    if (!_bus || !_slave)
        return false;
    return true;
}

void AP_HMC5843_SerialBus_MPU6000::set_high_speed(bool val)
{
}

uint8_t AP_HMC5843_SerialBus_MPU6000::register_read(uint8_t reg, uint8_t *buf, uint8_t size)
{
    return _slave->passthrough_read(reg, buf, size) == size ? 0 : 1;
}

uint8_t AP_HMC5843_SerialBus_MPU6000::register_write(uint8_t reg, uint8_t val)
{
    return _slave->passthrough_write(reg, val) >= 0 ? 0 : 1;
}

AP_HAL::Semaphore* AP_HMC5843_SerialBus_MPU6000::get_semaphore()
{
    return _bus ? _bus->get_semaphore() : nullptr;
}

uint8_t AP_HMC5843_SerialBus_MPU6000::read_raw(struct raw_value *rv)
{
    if (_started)
        return _slave->read((uint8_t*)rv) >= 0 ? 0 : 1;

    return _slave->passthrough_read(0x03, (uint8_t*)rv, sizeof(*rv)) >= 0 ? 0 : 1;
}

bool AP_HMC5843_SerialBus_MPU6000::start_measurements()
{
    if (_bus->register_periodic_read(_slave, 0x03, sizeof(struct raw_value)) < 0)
        return false;

    _started = true;

    return true;
}
