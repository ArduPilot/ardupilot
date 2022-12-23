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
#include <assert.h>
#include <utility>

#include <AP_Math/AP_Math.h>
#include <AP_HAL/AP_HAL.h>

#include "AP_Compass_AK8963.h"
#include <AP_InertialSensor/AP_InertialSensor_Invensense.h>

#define AK8963_I2C_ADDR                                 0x0c

#define AK8963_WIA                                      0x00
#        define AK8963_Device_ID                        0x48

#define AK8963_HXL                                      0x03

/* bit definitions for AK8963 CNTL1 */
#define AK8963_CNTL1                                    0x0A
#        define    AK8963_CONTINUOUS_MODE1              0x02
#        define    AK8963_CONTINUOUS_MODE2              0x06
#        define    AK8963_SELFTEST_MODE                 0x08
#        define    AK8963_POWERDOWN_MODE                0x00
#        define    AK8963_FUSE_MODE                     0x0f
#        define    AK8963_16BIT_ADC                     0x10
#        define    AK8963_14BIT_ADC                     0x00

#define AK8963_CNTL2                                    0x0B
#        define AK8963_RESET                            0x01

#define AK8963_ASAX                                     0x10

#define AK8963_MILLIGAUSS_SCALE 10.0f

struct PACKED sample_regs {
    int16_t val[3];
    uint8_t st2;
};

extern const AP_HAL::HAL &hal;

AP_Compass_AK8963::AP_Compass_AK8963(AP_AK8963_BusDriver *bus,
                                     enum Rotation rotation)
    : _bus(bus)
    , _rotation(rotation)
{
}

AP_Compass_AK8963::~AP_Compass_AK8963()
{
    delete _bus;
}

AP_Compass_Backend *AP_Compass_AK8963::probe(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                             enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    AP_AK8963_BusDriver *bus = new AP_AK8963_BusDriver_HALDevice(std::move(dev));
    if (!bus) {
        return nullptr;
    }

    AP_Compass_AK8963 *sensor = new AP_Compass_AK8963(bus, rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

AP_Compass_Backend *AP_Compass_AK8963::probe_mpu9250(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                                     enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
#if AP_INERTIALSENSOR_ENABLED
    AP_InertialSensor &ins = *AP_InertialSensor::get_singleton();

    /* Allow MPU9250 to shortcut auxiliary bus and host bus */
    ins.detect_backends();
#endif

    return probe(std::move(dev), rotation);
}

AP_Compass_Backend *AP_Compass_AK8963::probe_mpu9250(uint8_t mpu9250_instance,
                                                     enum Rotation rotation)
{
#if AP_INERTIALSENSOR_ENABLED
    AP_InertialSensor &ins = *AP_InertialSensor::get_singleton();

    AP_AK8963_BusDriver *bus =
        new AP_AK8963_BusDriver_Auxiliary(ins, HAL_INS_MPU9250_SPI, mpu9250_instance, AK8963_I2C_ADDR);
    if (!bus) {
        return nullptr;
    }

    AP_Compass_AK8963 *sensor = new AP_Compass_AK8963(bus, rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
#else
    return nullptr;
#endif

}

bool AP_Compass_AK8963::init()
{
    AP_HAL::Semaphore *bus_sem = _bus->get_semaphore();

    if (!bus_sem) {
        return false;
    }
    _bus->get_semaphore()->take_blocking();

    if (!_bus->configure()) {
        DEV_PRINTF("AK8963: Could not configure the bus\n");
        goto fail;
    }

    if (!_check_id()) {
        DEV_PRINTF("AK8963: Wrong id\n");
        goto fail;
    }

    if (!_calibrate()) {
        DEV_PRINTF("AK8963: Could not read calibration data\n");
        goto fail;
    }

    if (!_setup_mode()) {
        DEV_PRINTF("AK8963: Could not setup mode\n");
        goto fail;
    }

    if (!_bus->start_measurements()) {
        DEV_PRINTF("AK8963: Could not start measurements\n");
        goto fail;
    }

    _initialized = true;

    /* register the compass instance in the frontend */
    _bus->set_device_type(DEVTYPE_AK8963);
    if (!register_compass(_bus->get_bus_id(), _compass_instance)) {
        goto fail;
    }
    set_dev_id(_compass_instance, _bus->get_bus_id());

    set_rotation(_compass_instance, _rotation);
    bus_sem->give();

    _bus->register_periodic_callback(10000, FUNCTOR_BIND_MEMBER(&AP_Compass_AK8963::_update, void));

    return true;

fail:
    bus_sem->give();
    return false;
}

void AP_Compass_AK8963::read()
{
    if (!_initialized) {
        return;
    }

    drain_accumulated_samples(_compass_instance);
}

void AP_Compass_AK8963::_make_adc_sensitivity_adjustment(Vector3f& field) const
{
    static const float ADC_16BIT_RESOLUTION = 0.15f;

    field *= ADC_16BIT_RESOLUTION;
}

void AP_Compass_AK8963::_make_factory_sensitivity_adjustment(Vector3f& field) const
{
    field.x *= _magnetometer_ASA[0];
    field.y *= _magnetometer_ASA[1];
    field.z *= _magnetometer_ASA[2];
}

void AP_Compass_AK8963::_update()
{
    struct sample_regs regs;
    Vector3f raw_field;

    if (!_bus->block_read(AK8963_HXL, (uint8_t *) &regs, sizeof(regs))) {
        return;
    }

    /* Check for overflow. See AK8963's datasheet, section
     * 6.4.3.6 - Magnetic Sensor Overflow. */
    if ((regs.st2 & 0x08)) {
        return;
    }

    raw_field = Vector3f(regs.val[0], regs.val[1], regs.val[2]);

    if (is_zero(raw_field.x) && is_zero(raw_field.y) && is_zero(raw_field.z)) {
        return;
    }

    _make_factory_sensitivity_adjustment(raw_field);
    _make_adc_sensitivity_adjustment(raw_field);
    raw_field *= AK8963_MILLIGAUSS_SCALE;

    accumulate_sample(raw_field, _compass_instance, 10);
}

bool AP_Compass_AK8963::_check_id()
{
    for (int i = 0; i < 5; i++) {
        uint8_t deviceid = 0;

        /* Read AK8963's id */
        if (_bus->register_read(AK8963_WIA, &deviceid) &&
            deviceid == AK8963_Device_ID) {
            return true;
        }
    }

    return false;
}

bool AP_Compass_AK8963::_setup_mode() {
    return _bus->register_write(AK8963_CNTL1, AK8963_CONTINUOUS_MODE2 | AK8963_16BIT_ADC);
}

bool AP_Compass_AK8963::_reset()
{
    return _bus->register_write(AK8963_CNTL2, AK8963_RESET);
}


bool AP_Compass_AK8963::_calibrate()
{
    /* Enable FUSE-mode in order to be able to read calibration data */
    _bus->register_write(AK8963_CNTL1, AK8963_FUSE_MODE | AK8963_16BIT_ADC);

    uint8_t response[3];

    _bus->block_read(AK8963_ASAX, response, 3);

    for (int i = 0; i < 3; i++) {
        float data = response[i];
        _magnetometer_ASA[i] = ((data - 128) / 256 + 1);
    }

    return true;
}

/* AP_HAL::I2CDevice implementation of the AK8963 */
AP_AK8963_BusDriver_HALDevice::AP_AK8963_BusDriver_HALDevice(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : _dev(std::move(dev))
{
}

bool AP_AK8963_BusDriver_HALDevice::block_read(uint8_t reg, uint8_t *buf, uint32_t size)
{
    return _dev->read_registers(reg, buf, size);
}

bool AP_AK8963_BusDriver_HALDevice::register_read(uint8_t reg, uint8_t *val)
{
    return _dev->read_registers(reg, val, 1);
}

bool AP_AK8963_BusDriver_HALDevice::register_write(uint8_t reg, uint8_t val)
{
    return _dev->write_register(reg, val);
}

AP_HAL::Semaphore *AP_AK8963_BusDriver_HALDevice::get_semaphore()
{
    return _dev->get_semaphore();
}

AP_HAL::Device::PeriodicHandle AP_AK8963_BusDriver_HALDevice::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    return _dev->register_periodic_callback(period_usec, cb);
}

/* AK8963 on an auxiliary bus of IMU driver */
AP_AK8963_BusDriver_Auxiliary::AP_AK8963_BusDriver_Auxiliary(AP_InertialSensor &ins, uint8_t backend_id,
                                                             uint8_t backend_instance, uint8_t addr)
{
    /*
     * Only initialize members. Fails are handled by configure or while
     * getting the semaphore
     */
#if AP_INERTIALSENSOR_ENABLED
    _bus = ins.get_auxiliary_bus(backend_id, backend_instance);
    if (!_bus) {
        return;
    }

    _slave = _bus->request_next_slave(addr);
#endif
}

AP_AK8963_BusDriver_Auxiliary::~AP_AK8963_BusDriver_Auxiliary()
{
    /* After started it's owned by AuxiliaryBus */
    if (!_started) {
        delete _slave;
    }
}

bool AP_AK8963_BusDriver_Auxiliary::block_read(uint8_t reg, uint8_t *buf, uint32_t size)
{
    if (_started) {
        /*
         * We can only read a block when reading the block of sample values -
         * calling with any other value is a mistake
         */
        if (reg != AK8963_HXL) {
            return false;
        }

        int n = _slave->read(buf);
        return n == static_cast<int>(size);
    }

    int r = _slave->passthrough_read(reg, buf, size);

    return r > 0 && static_cast<uint32_t>(r) == size;
}

bool AP_AK8963_BusDriver_Auxiliary::register_read(uint8_t reg, uint8_t *val)
{
    return _slave->passthrough_read(reg, val, 1) == 1;
}

bool AP_AK8963_BusDriver_Auxiliary::register_write(uint8_t reg, uint8_t val)
{
    return _slave->passthrough_write(reg, val) == 1;
}

AP_HAL::Semaphore *AP_AK8963_BusDriver_Auxiliary::get_semaphore()
{
    return _bus ? _bus->get_semaphore() : nullptr;
}

bool AP_AK8963_BusDriver_Auxiliary::configure()
{
    if (!_bus || !_slave) {
        return false;
    }
    return true;
}

bool AP_AK8963_BusDriver_Auxiliary::start_measurements()
{
    if (_bus->register_periodic_read(_slave, AK8963_HXL, sizeof(sample_regs)) < 0) {
        return false;
    }

    _started = true;

    return true;
}

AP_HAL::Device::PeriodicHandle AP_AK8963_BusDriver_Auxiliary::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    return _bus->register_periodic_callback(period_usec, cb);
}

// set device type within a device class
void AP_AK8963_BusDriver_Auxiliary::set_device_type(uint8_t devtype)
{
    _bus->set_device_type(devtype);
}

// return 24 bit bus identifier
uint32_t AP_AK8963_BusDriver_Auxiliary::get_bus_id(void) const
{
    return _bus->get_bus_id();
}
