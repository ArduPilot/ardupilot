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
#include <stdio.h>

#include "AP_Baro_LPS2XH.h"

#include <AP_InertialSensor/AP_InertialSensor_Invensense_registers.h>

extern const AP_HAL::HAL &hal;

// WHOAMI values
#define LPS22HB_WHOAMI 0xB1
#define LPS25HB_WHOAMI 0xBD

#define REG_ID		      					0x0F

#define LPS22H_ID       					0xB1
#define LPS22H_CTRL_REG1		0x10
#define LPS22H_CTRL_REG2		0x11
#define LPS22H_CTRL_REG3		0x12

#define LPS22H_CTRL_REG1_SIM				(1 << 0)
#define LPS22H_CTRL_REG1_BDU				(1 << 1)
#define LPS22H_CTRL_REG1_LPFP_CFG	(1 << 2)
#define LPS22H_CTRL_REG1_EN_LPFP	(1 << 3)
#define LPS22H_CTRL_REG1_PD				(0 << 4)
#define LPS22H_CTRL_REG1_ODR_1HZ	(1 << 4)
#define LPS22H_CTRL_REG1_ODR_10HZ	(2 << 4)
#define LPS22H_CTRL_REG1_ODR_25HZ	(3 << 4)
#define LPS22H_CTRL_REG1_ODR_50HZ	(4 << 4)
#define LPS22H_CTRL_REG1_ODR_75HZ	(5 << 4)


#define LPS25H_CTRL_REG1_ADDR      0x20
#define LPS25H_CTRL_REG2_ADDR      0x21
#define LPS25H_CTRL_REG3_ADDR      0x22
#define LPS25H_CTRL_REG4_ADDR     	0x23
#define LPS25H_FIFO_CTRL          			0x2E
#define TEMP_OUT_ADDR      					0x2B
#define PRESS_OUT_XL_ADDR		  		0x28
//putting 1 in the MSB of those two registers turns on Auto increment for faster reading.

AP_Baro_LPS2XH::AP_Baro_LPS2XH(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev)
    : AP_Baro_Backend(baro)
    , _dev(std::move(dev))
{
}

AP_Baro_Backend *AP_Baro_LPS2XH::probe(AP_Baro &baro,
                                       AP_HAL::OwnPtr<AP_HAL::Device> dev)
{
    if (!dev) {
        return nullptr;
    }
    AP_Baro_LPS2XH *sensor = new AP_Baro_LPS2XH(baro, std::move(dev));
    if (!sensor || !sensor->_init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

AP_Baro_Backend *AP_Baro_LPS2XH::probe_InvensenseIMU(AP_Baro &baro,
                                                     AP_HAL::OwnPtr<AP_HAL::Device> dev,
                                                     uint8_t imu_address)
{
    if (!dev) {
        return nullptr;
    }
    AP_Baro_LPS2XH *sensor = new AP_Baro_LPS2XH(baro, std::move(dev));
    if (sensor) {
        if (!sensor->_imu_i2c_init(imu_address)) {
            delete sensor;
            return nullptr;
        }
    }
    if (!sensor || !sensor->_init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

/*
  setup invensense IMU to enable barometer, assuming both IMU and baro
  on the same i2c bus
*/
bool AP_Baro_LPS2XH::_imu_i2c_init(uint8_t imu_address)
{
    if (!_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }
    // as the baro device is already locked we need to re-use it,
    // changing its address to match the IMU address
    uint8_t old_address = _dev->get_bus_address();
    _dev->set_address(imu_address);
    
    _dev->set_retries(4);

    uint8_t whoami=0;
    _dev->read_registers(MPUREG_WHOAMI, &whoami, 1);
    hal.console->printf("IMU: whoami 0x%02x old_address=%02x\n", whoami, old_address);

    _dev->write_register(MPUREG_FIFO_EN, 0x00);
    _dev->write_register(MPUREG_PWR_MGMT_1, BIT_PWR_MGMT_1_CLK_XGYRO);
    
    // wait for sensor to settle
    hal.scheduler->delay(10);

    _dev->write_register(MPUREG_INT_PIN_CFG, BIT_BYPASS_EN);

    _dev->set_address(old_address);

    _dev->get_semaphore()->give();

    return true;
}

bool AP_Baro_LPS2XH::_init()
{
    if (!_dev || !_dev->get_semaphore()->take(HAL_SEMAPHORE_BLOCK_FOREVER)) {
        return false;
    }
    _has_sample = false;

    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    // top bit is for read on SPI
    _dev->set_read_flag(0x80);
    
    if(!_check_whoami()){
        _dev->get_semaphore()->give();
        return false;
    }

    //init control registers.
    if(_lps2xh_type == BARO_LPS25H){
    	_dev->write_register(LPS25H_CTRL_REG1_ADDR,0x00); // turn off for config
    	_dev->write_register(LPS25H_CTRL_REG2_ADDR,0x00); //FIFO Disabled
    	_dev->write_register(LPS25H_FIFO_CTRL, 0x01);
    	_dev->write_register(LPS25H_CTRL_REG1_ADDR,0xc0);

    	// request 25Hz update (maximum refresh Rate according to datasheet)
    	CallTime = 40 * AP_USEC_PER_MSEC;
    }
    if(_lps2xh_type == BARO_LPS22H){
    	_dev->write_register(LPS22H_CTRL_REG1,LPS22H_CTRL_REG1_ODR_75HZ|LPS22H_CTRL_REG1_BDU|LPS22H_CTRL_REG1_EN_LPFP|LPS22H_CTRL_REG1_LPFP_CFG);
		_dev->write_register(LPS22H_CTRL_REG2,0x18);

		// request 75Hz update
		CallTime = 1000000/75;
    }

    _instance = _frontend.register_sensor();

    _dev->get_semaphore()->give();

    _dev->register_periodic_callback(CallTime, FUNCTOR_BIND_MEMBER(&AP_Baro_LPS2XH::_timer, void));

    return true;
}

//check ID
bool AP_Baro_LPS2XH::_check_whoami(void)
{
    uint8_t whoami;
    if (! _dev->read_registers(REG_ID, &whoami, 1)) {
	   return false;
    }
    hal.console->printf("LPS2XH whoami 0x%02x\n", whoami);

    switch(whoami){
    case	LPS22HB_WHOAMI:
    	_lps2xh_type = BARO_LPS22H;
    	return true;
    case LPS25HB_WHOAMI:
    	_lps2xh_type = BARO_LPS25H;
    	return true;
    }
    return false;
}


//  acumulate a new sensor reading
void AP_Baro_LPS2XH::_timer(void)
{
    _update_temperature();
    _update_pressure();
    _has_sample = true;
}

// transfer data to the frontend
void AP_Baro_LPS2XH::update(void)
{
    if (_sem.take_nonblocking()) {
        if (!_has_sample) {
            _sem.give();
            return;
        }

        _copy_to_frontend(_instance, _pressure, _temperature);
        _has_sample = false;
        _sem.give();
    }
}

// calculate temperature
void AP_Baro_LPS2XH::_update_temperature(void)
{
    uint8_t pu8[2];
    _dev->read_registers(TEMP_OUT_ADDR, pu8, 2);
    int16_t Temp_Reg_s16 = (uint16_t)(pu8[1]<<8) | pu8[0];
    
    if (_sem.take_nonblocking()) {
    	if(_lps2xh_type == BARO_LPS25H){
    		_temperature=((float)(Temp_Reg_s16/480)+42.5);
    	}
    	if(_lps2xh_type == BARO_LPS22H){
    		_temperature=(float)(Temp_Reg_s16/100);
    	}
        _sem.give();
    }
}

// calculate pressure
void AP_Baro_LPS2XH::_update_pressure(void)
{
    uint8_t pressure[3];
    _dev->read_registers(PRESS_OUT_XL_ADDR, pressure, 3);
    int32_t Pressure_Reg_s32 = ((uint32_t)pressure[2]<<16)|((uint32_t)pressure[1]<<8)|(uint32_t)pressure[0];
    int32_t Pressure_mb = Pressure_Reg_s32 * (100.0 / 4096); // scale for pa
    if (_sem.take_nonblocking()) {
        _pressure=Pressure_mb;
        _sem.give();
    }
}
