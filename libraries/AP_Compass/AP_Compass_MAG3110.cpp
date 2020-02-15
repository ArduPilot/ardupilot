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
#include <utility>

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <stdio.h>

#include "AP_Compass_MAG3110.h"

extern const AP_HAL::HAL &hal;


/*
EN:
    at first glance, the magnetometer MAG3110 consists only of flaws:
    * noisy, with a bad characteristic, with very large difference on axes
    * it can't be calibrated in any way, you just have to believe what he has been measured
    * There are no adjustments and settings, it just sends data with some unknown sensitivity. Or does not sends :)
    * One and a half setup registers, in which only the frequency of operation and the number of averagings are specified
    
    This is a device, wooden to the waist. But of all these shortcomings, its sole and basic virtue arises:
    * He will never comes buggy or clevers.
    
    And since we do not need much from a magnetometer and it is required to calibrate the Ardupilot itself, the device
    appears in a completely new light - as a reliable info "north is there." What we really need.

RUS:
    на первый взгляд, магнитометр MAG3110 состоит из одних лишь недостатков:
    * шумный, с кривой характеристикой, 
    * никак не калибруется, приходится просто верить тому что он намерял
    * нет никаких регулировок и настроек, он просто выдает данные с некой неизвестной чувствительностью. Или не выдает :)
    * полтора настроечных регистра, в которых задается только частота работы и количество усреднений
    
    Такой вот девайс, по пояс деревянный. Но из всех этих недостатков проистекает его единственное и основное достоинство:
    * он никогда не глючит и не умничает.
    
    А так как нам от магнитометра особо много и не требуется, а калибровать Ардупилот и сам умеет, то девайс
    предстает в совсем новом свете - как надежный указатель "север там". Что нам собственно и надо.

*/

/*
  the vector length filter can help with noise on the bus, but may
  interfere with higher level processing. It should really be moved
  into the AP_Compass_backend code, with a parameter to enable it.
 */
#ifndef MAG3110_ENABLE_LEN_FILTER
#define MAG3110_ENABLE_LEN_FILTER 0
#endif


// Registers
#define MAG3110_MAG_REG_STATUS       0x00
#define MAG3110_MAG_REG_HXL          0x01
#define MAG3110_MAG_REG_HXH          0x02
#define MAG3110_MAG_REG_HYL          0x03
#define MAG3110_MAG_REG_HYH          0x04
#define MAG3110_MAG_REG_HZL          0x05
#define MAG3110_MAG_REG_HZH          0x06
#define MAG3110_MAG_REG_WHO_AM_I     0x07
#define MAG3110_MAG_REG_SYSMODE      0x08
#define MAG3110_MAG_REG_CTRL_REG1    0x10
#define MAG3110_MAG_REG_CTRL_REG2    0x11

#define BIT_STATUS_REG_DATA_READY    (1 << 3)




AP_Compass_MAG3110::AP_Compass_MAG3110(AP_HAL::OwnPtr<AP_HAL::Device> dev)
    : _dev(std::move(dev))
{
}

AP_Compass_Backend *AP_Compass_MAG3110::probe(AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                              enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    AP_Compass_MAG3110 *sensor = new AP_Compass_MAG3110(std::move(dev));
    if (!sensor || !sensor->init(rotation)) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}


bool AP_Compass_MAG3110::init(enum Rotation rotation)
{

    bool success = _hardware_init();

    if (!success) {
        return false;
    }

    _initialised = true;

    // perform an initial read
    read();
    
    /* register the compass instance in the frontend */
    _compass_instance = register_compass();

    set_rotation(_compass_instance, rotation);

    _dev->set_device_type(DEVTYPE_MAG3110);
    set_dev_id(_compass_instance, _dev->get_bus_id());

    set_external(_compass_instance, true);

    // read at 75Hz
    _dev->register_periodic_callback(13333, FUNCTOR_BIND_MEMBER(&AP_Compass_MAG3110::_update, void)); 

    return true;
}

bool AP_Compass_MAG3110::_hardware_init()
{

    AP_HAL::Semaphore *bus_sem = _dev->get_semaphore();
    bus_sem->take_blocking();

    // initially run the bus at low speed
    _dev->set_speed(AP_HAL::Device::SPEED_LOW);

    bool ret=false;
    
    _dev->set_retries(5);
    
    uint8_t sig = 0;
    bool ack = _dev->read_registers(MAG3110_MAG_REG_WHO_AM_I, &sig, 1);    
    if (!ack || sig != 0xC4) goto exit;

    ack = _dev->write_register(MAG3110_MAG_REG_CTRL_REG1, 0x01); //  active mode 80 Hz ODR with OSR = 1
    if (!ack) goto exit;

    hal.scheduler->delay(20);
    
    ack = _dev->write_register(MAG3110_MAG_REG_CTRL_REG2, 0xA0); // AUTO_MRST_EN + RAW
    if (!ack) goto exit;

    ret = true;

    _dev->set_retries(3);
    
    printf("MAG3110 found on bus 0x%x\n", (uint16_t)_dev->get_bus_id());

exit:
    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);
    bus_sem->give();
    return ret;
}


// Read Sensor data
bool AP_Compass_MAG3110::_read_sample()
{
    {
        uint8_t status;
        bool ack = _dev->read_registers(MAG3110_MAG_REG_STATUS, &status, 1);
    
        if (!ack || (status & BIT_STATUS_REG_DATA_READY) == 0) {
            return false;
        }
    }

    uint8_t buf[6];
    bool ack = _dev->read_registers(MAG3110_MAG_REG_HXL, buf, 6);
    if (!ack) {
        return false;
    }

    _mag_x = (int16_t)(buf[0] << 8 | buf[1]);
    _mag_y = (int16_t)(buf[2] << 8 | buf[3]);
    _mag_z = (int16_t)(buf[4] << 8 | buf[5]);

    return true;
}


#define MAG_SCALE (1.0f/10000 / 0.0001f * 1000)  // 1 Tesla full scale of +-10000, 1 Gauss = 0,0001 Tesla, library needs milliGauss

void AP_Compass_MAG3110::_update()
{
    if (!_read_sample()) {
        return;
    }

    Vector3f raw_field = Vector3f((float)_mag_x, (float)_mag_y, (float)_mag_z) * MAG_SCALE;

    accumulate_sample(raw_field, _compass_instance);
}


// Read Sensor data
void AP_Compass_MAG3110::read()
{
    if (!_initialised) {
        return;
    }

    drain_accumulated_samples(_compass_instance);
}
