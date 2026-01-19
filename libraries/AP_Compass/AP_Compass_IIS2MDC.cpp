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
 *
 * https://www.st.com/resource/en/datasheet/iis2mdc.pdf
 *
 */
#include "AP_Compass_config.h"

#if AP_COMPASS_IIS2MDC_ENABLED

#include "AP_Compass_IIS2MDC.h"

// IIS2MDC Registers
#define IIS2MDC_ADDR_CFG_REG_A  0x60
#define IIS2MDC_ADDR_CFG_REG_B  0x61
#define IIS2MDC_ADDR_CFG_REG_C  0x62
#define IIS2MDC_ADDR_STATUS_REG 0x67
#define IIS2MDC_ADDR_OUTX_L_REG 0x68
#define IIS2MDC_ADDR_WHO_AM_I   0x4F

// IIS2MDC Definitions
#define IIS2MDC_WHO_AM_I         0b01000000
#define IIS2MDC_STATUS_REG_READY 0b00001111
// CFG_REG_A
#define COMP_TEMP_EN    (1 << 7)
#define MD_CONTINUOUS   (0 << 0)
#define ODR_100         ((1 << 3) | (1 << 2))
// CFG_REG_B
#define OFF_CANC        (1 << 1)
// CFG_REG_C
#define BDU             (1 << 4)

extern const AP_HAL::HAL &hal;

AP_Compass_Backend *AP_Compass_IIS2MDC::probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
        bool force_external,
        enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }

    AP_Compass_IIS2MDC *sensor = NEW_NOTHROW AP_Compass_IIS2MDC(std::move(dev),force_external,rotation);

    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

AP_Compass_IIS2MDC::AP_Compass_IIS2MDC(AP_HAL::OwnPtr<AP_HAL::Device> dev,
        bool force_external,
        enum Rotation rotation)
    : _dev(std::move(dev))
    , _rotation(rotation)
    , _force_external(force_external)
{
}

bool AP_Compass_IIS2MDC::init()
{
    WITH_SEMAPHORE(_dev->get_semaphore());

    _dev->set_retries(10);

    if (!check_whoami()) {
        return false;
    }

    if (!_dev->write_register(IIS2MDC_ADDR_CFG_REG_A, MD_CONTINUOUS | ODR_100 | COMP_TEMP_EN)) {
        return false;
    }

    if (!_dev->write_register(IIS2MDC_ADDR_CFG_REG_B, OFF_CANC)) {
        return false;
    }

    if (!_dev->write_register(IIS2MDC_ADDR_CFG_REG_C, BDU)) {
        return false;
    }

    // lower retries for run
    _dev->set_retries(3);

    // register compass instance
    _dev->set_device_type(DEVTYPE_IIS2MDC);

    if (!register_compass(_dev->get_bus_id())) {
        return false;
    }

    set_rotation(_rotation);

    if (_force_external) {
        set_external(true);
    }

    // Enable 100HZ
    _dev->register_periodic_callback(10000, FUNCTOR_BIND_MEMBER(&AP_Compass_IIS2MDC::timer, void));

    return true;
}

bool AP_Compass_IIS2MDC::check_whoami()
{
    uint8_t whoami = 0;
    if (!_dev->read_registers(IIS2MDC_ADDR_WHO_AM_I, &whoami, 1)){
        return false;
    }

    return whoami == IIS2MDC_WHO_AM_I;
}

void AP_Compass_IIS2MDC::timer()
{
    struct PACKED {
        uint8_t xout0;
        uint8_t xout1;
        uint8_t yout0;
        uint8_t yout1;
        uint8_t zout0;
        uint8_t zout1;
        uint8_t tout0;
        uint8_t tout1;
    } buffer;

    const float range_scale = 100.f / 65.535f; // +/- 50,000 milligauss, 16bit

    uint8_t status = 0;
    if (!_dev->read_registers(IIS2MDC_ADDR_STATUS_REG, &status, 1)) {
        return;
    }

    if (!(status & IIS2MDC_STATUS_REG_READY)) {
        return;
    }

    if (!_dev->read_registers(IIS2MDC_ADDR_OUTX_L_REG, (uint8_t *) &buffer, sizeof(buffer))) {
        return;
    }

    const int16_t x = ((buffer.xout1 << 8) | buffer.xout0);
    const int16_t y = ((buffer.yout1 << 8) | buffer.yout0);
    const int16_t z = -1 * ((buffer.zout1 << 8) | buffer.zout0);

    Vector3f field{ x * range_scale, y * range_scale, z * range_scale };

    accumulate_sample(field);
}

void AP_Compass_IIS2MDC::read()
{
    drain_accumulated_samples();
}

#endif //AP_COMPASS_IIS2MDC_ENABLED
