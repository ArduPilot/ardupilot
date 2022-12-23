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
/*
  Driver by Andrew Tridgell, Nov 2016
 */
#include "AP_Compass_AK09916.h"

#include <assert.h>
#include <AP_HAL/AP_HAL.h>
#include <utility>
#include <AP_Math/AP_Math.h>
#include <stdio.h>
#include <AP_InertialSensor/AP_InertialSensor_Invensensev2.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_BoardConfig/AP_BoardConfig.h>

extern const AP_HAL::HAL &hal;

#define REG_COMPANY_ID      0x00
#define REG_DEVICE_ID       0x01
#define REG_ST1             0x10
#define REG_HXL             0x11
#define REG_HXH             0x12
#define REG_HYL             0x13
#define REG_HYH             0x14
#define REG_HZL             0x15
#define REG_HZH             0x16
#define REG_TMPS            0x17
#define REG_ST2             0x18
#define REG_CNTL1           0x30
#define REG_CNTL2           0x31
#define REG_CNTL3           0x32

#define REG_ICM_WHOAMI      0x00
#define REG_ICM_PWR_MGMT_1  0x06
#define REG_ICM_INT_PIN_CFG 0x0f

#define ICM_WHOAMI_VAL      0xEA

#define AK09915_Device_ID   0x10
#define AK09916_Device_ID   0x09
#define AK09918_Device_ID   0x0c
#define AK09916_MILLIGAUSS_SCALE 10.0f

extern const AP_HAL::HAL &hal;

struct PACKED sample_regs {
    uint8_t st1;
    int16_t val[3];
    uint8_t tmps;
    uint8_t st2;
};

AP_Compass_AK09916::AP_Compass_AK09916(AP_AK09916_BusDriver *bus,
                                        bool force_external,
                                        enum Rotation rotation)
    : _bus(bus)
    , _force_external(force_external)
    , _rotation(rotation)
{
}

AP_Compass_AK09916::~AP_Compass_AK09916()
{
    delete _bus;
}

AP_Compass_Backend *AP_Compass_AK09916::probe(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                             bool force_external,
                                             enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    AP_AK09916_BusDriver *bus = new AP_AK09916_BusDriver_HALDevice(std::move(dev));
    if (!bus) {
        return nullptr;
    }

    AP_Compass_AK09916 *sensor = new AP_Compass_AK09916(bus, force_external, rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

AP_Compass_Backend *AP_Compass_AK09916::probe_ICM20948(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev,
                                                     AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev_icm,
                                                     bool force_external,
                                                     enum Rotation rotation)
{
    if (!dev || !dev_icm) {
        return nullptr;
    }

    dev->get_semaphore()->take_blocking();

    /* Allow ICM20x48 to shortcut auxiliary bus and host bus */
    uint8_t rval;
    uint16_t whoami;
    uint8_t retries = 5;
    if (!dev_icm->read_registers(REG_ICM_WHOAMI, &rval, 1) ||
        rval != ICM_WHOAMI_VAL) {
        // not an ICM_WHOAMI
        goto fail;
    }
    do {
        // reset then bring sensor out of sleep mode
        if (!dev_icm->write_register(REG_ICM_PWR_MGMT_1, 0x80)) {
            goto fail;
        }
        hal.scheduler->delay(10);

        if (!dev_icm->write_register(REG_ICM_PWR_MGMT_1, 0x00)) {
            goto fail;
        }
        hal.scheduler->delay(10);
        
        // see if ICM20948 is sleeping
        if (!dev_icm->read_registers(REG_ICM_PWR_MGMT_1, &rval, 1)) {
            goto fail;
        }
        if ((rval & 0x40) == 0) {
            break;
        }
    } while (retries--);
    
    if (rval & 0x40) {
        // it didn't come out of sleep
        goto fail;
    }

    // initially force i2c bypass off
    dev_icm->write_register(REG_ICM_INT_PIN_CFG, 0x00);
    hal.scheduler->delay(1);

    // now check if a AK09916 shows up on the bus. If it does then
    // we have both a real AK09916 and a ICM20948 with an embedded
    // AK09916. In that case we will fail the driver load and use
    // the main AK09916 driver
    if (dev->read_registers(REG_COMPANY_ID, (uint8_t *)&whoami, 2)) {
        // a device is replying on the AK09916 I2C address, don't
        // load the ICM20948
        DEV_PRINTF("ICM20948: AK09916 bus conflict\n");
        goto fail;
    }

    // now force bypass on
    dev_icm->write_register(REG_ICM_INT_PIN_CFG, 0x02);
    hal.scheduler->delay(1);
    dev->get_semaphore()->give();
    return probe(std::move(dev), force_external, rotation);
fail:
    dev->get_semaphore()->give();
    return nullptr;
}

// un-named, assume SPI for compat
AP_Compass_Backend *AP_Compass_AK09916::probe_ICM20948(uint8_t inv2_instance,
                                                     enum Rotation rotation)
{  
    return probe_ICM20948_SPI(inv2_instance,rotation);
}

AP_Compass_Backend *AP_Compass_AK09916::probe_ICM20948_SPI(uint8_t inv2_instance,
                                                     enum Rotation rotation)
{
#if AP_INERTIALSENSOR_ENABLED
    AP_InertialSensor &ins = AP::ins();

    AP_AK09916_BusDriver *bus =
        new AP_AK09916_BusDriver_Auxiliary(ins, HAL_INS_INV2_SPI, inv2_instance, HAL_COMPASS_AK09916_I2C_ADDR);
    if (!bus) {
        return nullptr;
    }

    AP_Compass_AK09916 *sensor = new AP_Compass_AK09916(bus, false, rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
#else
    return nullptr;
#endif
}

AP_Compass_Backend *AP_Compass_AK09916::probe_ICM20948_I2C(uint8_t inv2_instance,
                                                     enum Rotation rotation)
{
    AP_InertialSensor &ins = AP::ins();

    AP_AK09916_BusDriver *bus =
        new AP_AK09916_BusDriver_Auxiliary(ins, HAL_INS_INV2_I2C, inv2_instance, HAL_COMPASS_AK09916_I2C_ADDR);
    if (!bus) {
        return nullptr;
    }

    AP_Compass_AK09916 *sensor = new AP_Compass_AK09916(bus, false, rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

bool AP_Compass_AK09916::init()
{
    AP_HAL::Semaphore *bus_sem = _bus->get_semaphore();

    if (!bus_sem) {
        return false;
    }
    _bus->get_semaphore()->take_blocking();

    if (!_bus->configure()) {
        DEV_PRINTF("AK09916: Could not configure the bus\n");
        goto fail;
    }

    if (!_reset()) {
        goto fail;
    }

    if (!_check_id()) {
        goto fail;
    }

    // one checked register for mode
    _bus->setup_checked_registers(1);

    if (!_setup_mode()) {
        DEV_PRINTF("AK09916: Could not setup mode\n");
        goto fail;
    }

    if (!_bus->start_measurements()) {
        DEV_PRINTF("AK09916: Could not start measurements\n");
        goto fail;
    }

    _initialized = true;

    /* register the compass instance in the frontend */
    _bus->set_device_type(_devtype);
    if (!register_compass(_bus->get_bus_id(), _compass_instance)) {
        goto fail;
    }
    set_dev_id(_compass_instance, _bus->get_bus_id());

    if (_force_external) {
        set_external(_compass_instance, true);
    }

    set_rotation(_compass_instance, _rotation);
    
    bus_sem->give();

    _bus->register_periodic_callback(10000, FUNCTOR_BIND_MEMBER(&AP_Compass_AK09916::_update, void));

    return true;

fail:
    bus_sem->give();
    return false;
}

void AP_Compass_AK09916::read()
{
    if (!_initialized) {
        return;
    }

    drain_accumulated_samples(_compass_instance);
}

void AP_Compass_AK09916::_make_adc_sensitivity_adjustment(Vector3f& field) const
{
    static const float ADC_16BIT_RESOLUTION = 0.15f;

    field *= ADC_16BIT_RESOLUTION;
}

void AP_Compass_AK09916::_update()
{
    struct sample_regs regs = {0};
    Vector3f raw_field;

    if (!_bus->block_read(REG_ST1, (uint8_t *) &regs, sizeof(regs))) {
        goto check_registers;
    }

    if (!(regs.st1 & 0x01)) {
        goto check_registers;
    }

    /* Check for overflow. See AK09916's datasheet*/
    if ((regs.st2 & 0x08)) {
        goto check_registers;
    }

    raw_field = Vector3f(regs.val[0], regs.val[1], regs.val[2]);

    if (is_zero(raw_field.x) && is_zero(raw_field.y) && is_zero(raw_field.z)) {
        goto check_registers;
    }

    _make_adc_sensitivity_adjustment(raw_field);
    raw_field *= AK09916_MILLIGAUSS_SCALE;

    accumulate_sample(raw_field, _compass_instance, 10);

check_registers:
    _bus->check_next_register();
}

bool AP_Compass_AK09916::_check_id()
{
    for (int i = 0; i < 5; i++) {
        uint8_t deviceid = 0;

        /* Read AK09916's id */
        if (_bus->register_read(REG_DEVICE_ID, &deviceid)) {
            switch (deviceid) {
            case AK09915_Device_ID:
                _devtype = DEVTYPE_AK09915;
                return true;
            case AK09916_Device_ID:
                _devtype = DEVTYPE_AK09916;
                return true;
            case AK09918_Device_ID:
                _devtype = DEVTYPE_AK09918;
                return true;
            }
        }
    }

    return false;
}

bool AP_Compass_AK09916::_setup_mode() {
    return _bus->register_write(REG_CNTL2, 0x08, true); //Continuous Mode 2
}

bool AP_Compass_AK09916::_reset()
{
    return _bus->register_write(REG_CNTL3, 0x01); //Soft Reset
}

/* AP_HAL::I2CDevice implementation of the AK09916 */
AP_AK09916_BusDriver_HALDevice::AP_AK09916_BusDriver_HALDevice(AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev)
    : _dev(std::move(dev))
{
}

bool AP_AK09916_BusDriver_HALDevice::block_read(uint8_t reg, uint8_t *buf, uint32_t size)
{
    return _dev->read_registers(reg, buf, size);
}

bool AP_AK09916_BusDriver_HALDevice::register_read(uint8_t reg, uint8_t *val)
{
    return _dev->read_registers(reg, val, 1);
}

bool AP_AK09916_BusDriver_HALDevice::register_write(uint8_t reg, uint8_t val, bool checked)
{
    return _dev->write_register(reg, val, checked);
}

AP_HAL::Semaphore *AP_AK09916_BusDriver_HALDevice::get_semaphore()
{
    return _dev->get_semaphore();
}

AP_HAL::Device::PeriodicHandle AP_AK09916_BusDriver_HALDevice::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    return _dev->register_periodic_callback(period_usec, cb);
}

/* AK09916 on an auxiliary bus of IMU driver */
AP_AK09916_BusDriver_Auxiliary::AP_AK09916_BusDriver_Auxiliary(AP_InertialSensor &ins, uint8_t backend_id,
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

AP_AK09916_BusDriver_Auxiliary::~AP_AK09916_BusDriver_Auxiliary()
{
    /* After started it's owned by AuxiliaryBus */
    if (!_started) {
        delete _slave;
    }
}

bool AP_AK09916_BusDriver_Auxiliary::block_read(uint8_t reg, uint8_t *buf, uint32_t size)
{
    if (_started) {
        /*
         * We can only read a block when reading the block of sample values -
         * calling with any other value is a mistake
         */
        if (reg != REG_ST1) {
            return false;
        }

        int n = _slave->read(buf);
        return n == static_cast<int>(size);
    }

    int r = _slave->passthrough_read(reg, buf, size);

    return r > 0 && static_cast<uint32_t>(r) == size;
}

bool AP_AK09916_BusDriver_Auxiliary::register_read(uint8_t reg, uint8_t *val)
{
    return _slave->passthrough_read(reg, val, 1) == 1;
}

bool AP_AK09916_BusDriver_Auxiliary::register_write(uint8_t reg, uint8_t val, bool checked)
{
    (void)checked;
    return _slave->passthrough_write(reg, val) == 1;
}

AP_HAL::Semaphore *AP_AK09916_BusDriver_Auxiliary::get_semaphore()
{
    return _bus ? _bus->get_semaphore() : nullptr;
}

bool AP_AK09916_BusDriver_Auxiliary::configure()
{
    if (!_bus || !_slave) {
        return false;
    }
    return true;
}

bool AP_AK09916_BusDriver_Auxiliary::start_measurements()
{
    if (_bus->register_periodic_read(_slave, REG_ST1, sizeof(sample_regs)) < 0) {
        return false;
    }

    _started = true;

    return true;
}

AP_HAL::Device::PeriodicHandle AP_AK09916_BusDriver_Auxiliary::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    return _bus->register_periodic_callback(period_usec, cb);
}

// set device type within a device class
void AP_AK09916_BusDriver_Auxiliary::set_device_type(uint8_t devtype)
{
    _bus->set_device_type(devtype);
}

// return 24 bit bus identifier
uint32_t AP_AK09916_BusDriver_Auxiliary::get_bus_id(void) const
{
    return _bus->get_bus_id();
}
