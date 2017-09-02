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
#include "AP_Baro_LPS25H.h"

#include <unistd.h>
#include <utility>
#include <stdio.h>
extern const AP_HAL::HAL &hal;


#define LPS25H_ID            	  0xBD
#define LPS25H_REG_ID		      0x0F
#define LPS25H_CTRL_REG1_ADDR     0x20
#define LPS25H_CTRL_REG2_ADDR     0x21
#define LPS25H_CTRL_REG3_ADDR     0x22
#define LPS25H_CTRL_REG4_ADDR     0x23
#define LPS25H_FIFO_CTRL          0x2E
#define LPS25H_TEMP_OUT_ADDR      0xAB	//Regsiter address is 0x2B in 2's compliment.
#define PRESS_OUT_XL_ADDR		  0xA8	//Regsiter address is 0x28 in 2's compliment.
//putting 1 in the MSB of those two registers turns on Auto increment for faster reading.

AP_Baro_LPS25H::AP_Baro_LPS25H(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev)
    : AP_Baro_Backend(baro)
    , _dev(std::move(dev))
{
}

AP_Baro_Backend *AP_Baro_LPS25H::probe(AP_Baro &baro,
                                       AP_HAL::OwnPtr<AP_HAL::Device> dev)
{
    if (!dev) {
        return nullptr;
    }
    AP_Baro_LPS25H *sensor = new AP_Baro_LPS25H(baro, std::move(dev));
    if (!sensor || !sensor->_init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

bool AP_Baro_LPS25H::_init()
{
    if (!_dev || !_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }
    _has_sample = false;

    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    uint8_t whoami;
    if (!_dev->read_registers(LPS25H_REG_ID, &whoami, 1)  ||
        whoami != LPS25H_ID) {
        // not a LPS25H
        _dev->get_semaphore()->give();
        return false;
    }

    //init control registers.
    _dev->write_register(LPS25H_CTRL_REG1_ADDR,0x00); // turn off for config
    _dev->write_register(LPS25H_CTRL_REG2_ADDR,0x00); //FIFO Disabled
    _dev->write_register(LPS25H_FIFO_CTRL, 0x01);
    _dev->write_register(LPS25H_CTRL_REG1_ADDR,0xc0);

    _instance = _frontend.register_sensor();

    _dev->get_semaphore()->give();

    // request 25Hz update (maximum refresh Rate according to datasheet)
    _dev->register_periodic_callback(40 * AP_USEC_PER_MSEC, FUNCTOR_BIND_MEMBER(&AP_Baro_LPS25H::_timer, void));

    return true;
}



//  acumulate a new sensor reading
void AP_Baro_LPS25H::_timer(void)
{
    _update_temperature();
    _update_pressure();
    _has_sample = true;
}

// transfer data to the frontend
void AP_Baro_LPS25H::update(void)
{
    if (_sem->take_nonblocking()) {
        if (!_has_sample) {
            _sem->give();
            return;
        }

        _copy_to_frontend(_instance, _pressure, _temperature);
        _has_sample = false;
        _sem->give();
    }
}

// calculate temperature
void AP_Baro_LPS25H::_update_temperature(void)
{
    uint8_t pu8[2];
    _dev->read_registers(LPS25H_TEMP_OUT_ADDR, pu8, 2);
    int16_t Temp_Reg_s16 = (uint16_t)(pu8[1]<<8) | pu8[0];
    if (_sem->take_nonblocking()) {
        _temperature=((float)(Temp_Reg_s16/480)+42.5);
        _sem->give();
    }

}

// calculate pressure
void AP_Baro_LPS25H::_update_pressure(void)
{
    uint8_t pressure[3];
    _dev->read_registers(PRESS_OUT_XL_ADDR, pressure, 3);
    int32_t Pressure_Reg_s32 = ((uint32_t)pressure[2]<<16)|((uint32_t)pressure[1]<<8)|(uint32_t)pressure[0];
    int32_t Pressure_mb = Pressure_Reg_s32 / 4096; // scale
    if (_sem->take_nonblocking()) {
        _pressure=Pressure_mb;
        _sem->give();
    }
}
