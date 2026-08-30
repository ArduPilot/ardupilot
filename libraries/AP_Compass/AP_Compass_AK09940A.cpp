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

#include "AP_Compass_config.h"

#if AP_COMPASS_AK09940A_ENABLED

#include "AP_Compass_AK09940A.h"
#include <AP_HAL/AP_HAL.h>
#include <utility>
#include <AP_Math/AP_Math.h>

#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define AK09940A_WIA1                                   0x00
#       define      AK09940A_MFG_ID                     0x48
#define AK09940A_WIA2                                   0x01
#       define      AK09940A_DEV_ID                     0xA3

#define AK09940A_ST                                      0x0F
#   define      AK09940A_ST_DOR                         0x2
#   define      AK09940A_ST_DRDY                        0x1

#define AK09940A_ST1                                     0x10

#define AK09940A_HXL                                    0x11
#define AK09940A_TMPS                                   0x1A
#define AK09940A_ST2                                    0x1B

/* bit definitions for AK09940A CNTL1 */
#define AK09940A_CNTL1                                  0x0A
#       define    AK09940A_DTSET                        0x20 // Set for external trigger

#define AK09940A_CNTL3                                  0x32
#        define    AK09940A_POWERDOWN_MODE              0x00
#        define    AK09940A_CONTINUOUS_MODE1            0x02 // 10 Hz
#        define    AK09940A_CONTINUOUS_MODE2            0x04 // 20 Hz
#        define    AK09940A_CONTINUOUS_MODE3            0x06 // 50 Hz
#        define    AK09940A_CONTINUOUS_MODE4            0x08 // 100 Hz
#        define    AK09940A_CONTINUOUS_MODE5            0x0A // 200 Hz
#        define    AK09940A_CONTINUOUS_MODE6            0x0C // 400 Hz
#        define    AK09940A_CONTINUOUS_MODE7            0x0E // 1 kHz
#        define    AK09940A_CONTINUOUS_MODE8            0x0F // 2.5 kHz
#        define    AK09940A_SELFTEST_MODE               0x10 
#        define    AK09940A_EXT_TRIGGER_MODE            0x18
#        define    AK09940A_SENSORDRIVE_LP1             (0x00)
#        define    AK09940A_SENSORDRIVE_LP2             (0x01)
#        define    AK09940A_SENSORDRIVE_LN1             (0x10)
#        define    AK09940A_SENSORDRIVE_LN2             (0x11)
#        define    AK09940A_FIFO                        0x80

#define AK09940A_CNTL4                                  0x33
#        define AK09940A_RESET                          0x01

#define AK09940A_LSB_TO_MILLIGAUSS_SCALE                0.1f

AP_Compass_AK09940A::AP_Compass_AK09940A(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                          bool force_external,
                                          enum Rotation rotation)
    : _dev(std::move(dev))
    , _force_external(force_external)
    , _rotation(rotation)
{
}

AP_Compass_Backend *AP_Compass_AK09940A::probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                             bool force_external,
                                             enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    
    AP_Compass_AK09940A *sensor = NEW_NOTHROW AP_Compass_AK09940A(std::move(dev), force_external, rotation);
    if (!sensor || !sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

bool AP_Compass_AK09940A::init()
{
    WITH_SEMAPHORE(_dev->get_semaphore());
    _dev->set_retries(5);

    if (!_reset()) {
        return false;
    }

    if (!_check_id()) {
        return false;
    }
    
    _set_sensordrive(AK09940A_SENSORDRIVE_LN1);
    _setup_mode(AK09940A_CONTINUOUS_MODE4); // Set continuous mode 100Hz

    /* register the compass instance in the frontend */
    _dev->set_device_type(DEVTYPE_AK09940A);
    if (!register_compass(_dev->get_bus_id())) {
        return false;
    }
    
    set_rotation(_rotation);
    
    DEV_PRINTF("Found  AK09940A on 0x%x as compass %u \n",
           unsigned(_dev->get_bus_id()), instance);
    
    _dev->register_periodic_callback(1000000U/100U, FUNCTOR_BIND_MEMBER(&AP_Compass_AK09940A::_update, void));

    return true;
}

int32_t AP_Compass_AK09940A::combine(uint8_t hl, uint8_t hm, uint8_t hh) 
{
    int32_t raw = hl | (hm << 8) | (hh << 16);
    raw &= 0x3FFFF; // keep lower 18 bits
    // sign-extend from 18 bits
    if (raw & (1 << 17)) {
        raw -= (1 << 18);
    }
    return raw;
}

void AP_Compass_AK09940A::read()
{
    drain_accumulated_samples();
}

void AP_Compass_AK09940A::_update()
{
    struct PACKED 
    {
        uint8_t st1;
        uint8_t magx[3];
        uint8_t magy[3];
        uint8_t magz[3];
        uint8_t tmps;
        uint8_t st2;
    } data;
    
    if (!_dev->read_registers(AK09940A_ST1, (uint8_t *)&data, sizeof(data))) {
        return;
    }
    
    Vector3f raw_field{
        (float) -combine(data.magx[0], data.magx[1], data.magx[2]) * AK09940A_LSB_TO_MILLIGAUSS_SCALE,
        (float) -combine(data.magy[0], data.magy[1], data.magy[2]) * AK09940A_LSB_TO_MILLIGAUSS_SCALE,
        (float) combine(data.magz[0], data.magz[1], data.magz[2]) * AK09940A_LSB_TO_MILLIGAUSS_SCALE
    };
    
    accumulate_sample(raw_field, _compass_instance);
}

bool AP_Compass_AK09940A::_check_id()
{
    for (int i = 0; i < 5; i++) {
        uint8_t deviceid = 0;
        uint8_t mfgid = 0;
        _dev->read_registers(AK09940A_WIA1, &mfgid, 1);
        _dev->read_registers(AK09940A_WIA2, &deviceid, 1);
        if (deviceid == AK09940A_DEV_ID && mfgid == AK09940A_MFG_ID) {
            return true;
        }
    }
    return false;
}

bool AP_Compass_AK09940A::_reset()
{
    return _dev->write_register(AK09940A_CNTL4, AK09940A_RESET);
}

bool AP_Compass_AK09940A::_setup_mode(uint8_t mode)
{
    return _dev->write_register(AK09940A_CNTL3, mode);
}

bool AP_Compass_AK09940A::_set_sensordrive(uint8_t sensordrive)
{
    uint8_t reg_ctrl3 = 0;
    _dev->read_registers(AK09940A_CNTL3, &reg_ctrl3, 1);
    reg_ctrl3 |= (sensordrive << 5);
    return _dev->write_register(AK09940A_CNTL3, reg_ctrl3);
}

#endif  // AP_COMPASS_AK09940A_ENABLED
