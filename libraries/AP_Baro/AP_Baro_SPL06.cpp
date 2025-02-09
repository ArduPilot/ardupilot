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
#include "AP_Baro_SPL06.h"

#if AP_BARO_SPL06_ENABLED

#include <GCS_MAVLink/GCS.h>

#include <utility>
#include <strings.h>
#include <AP_Math/definitions.h>

extern const AP_HAL::HAL &hal;

#define SPL06_CHIP_ID                          0x10
#define SPA06_CHIP_ID                          0x11

#define SPL06_REG_PRESSURE_B2                  0x00    // Pressure MSB Register
#define SPL06_REG_PRESSURE_B1                  0x01    // Pressure middle byte Register
#define SPL06_REG_PRESSURE_B0                  0x02    // Pressure LSB Register
#define SPL06_REG_PRESSURE_START               SPL06_REG_PRESSURE_B2
#define SPL06_PRESSURE_LEN                     3       // 24 bits, 3 bytes
#define SPL06_REG_TEMPERATURE_B2               0x03    // Temperature MSB Register
#define SPL06_REG_TEMPERATURE_B1               0x04    // Temperature middle byte Register
#define SPL06_REG_TEMPERATURE_B0               0x05    // Temperature LSB Register
#define SPL06_REG_TEMPERATURE_START            SPL06_REG_TEMPERATURE_B2
#define SPL06_TEMPERATURE_LEN                  3       // 24 bits, 3 bytes
#define SPL06_REG_PRESSURE_CFG                 0x06    // Pressure config
#define SPL06_REG_TEMPERATURE_CFG              0x07    // Temperature config
#define SPL06_REG_MODE_AND_STATUS              0x08    // Mode and status
#define SPL06_REG_INT_AND_FIFO_CFG             0x09    // Interrupt and FIFO config
#define SPL06_REG_INT_STATUS                   0x0A    // Interrupt and FIFO config
#define SPL06_REG_FIFO_STATUS                  0x0B    // Interrupt and FIFO config
#define SPL06_REG_SOFT_RESET                   0x0C    // Softreset Register
#define SPL06_REG_CHIP_ID                      0x0D    // Chip ID Register
#define SPL06_REG_CALIB_COEFFS_START           0x10
#define SPL06_REG_CALIB_COEFFS_END             0x21
#define SPA06_REG_CALIB_COEFFS_END             0x24

#define SPL06_OVERSAMPLING_TO_REG_VALUE(n)     (ffs(n)-1)
#define SPL06_MESSURE_RATE_TO_REG_VALUE(n)     (ffs(n)-1)

// TEMPERATURE_CFG_REG
#define SPL06_TEMP_USE_EXT_SENSOR              (1<<7)

// MODE_AND_STATUS_REG
#define SPL06_MEAS_PRESSURE                    (1<<0)  // measure pressure
#define SPL06_MEAS_TEMPERATURE                 (1<<1)  // measure temperature
#define SPL06_MEAS_CON_PRE_TEM				   0x07

// FIFO & RESET Register
#define SPL06_CMD_SOFT_RESET                   0x09
#define SPL06_CMD_FLASH_FIFO                   (1<<7)

#define SPL06_MEAS_CFG_CONTINUOUS              (1<<2)
#define SPL06_MEAS_CFG_PRESSURE_RDY            (1<<4)
#define SPL06_MEAS_CFG_TEMPERATURE_RDY         (1<<5)
#define SPL06_MEAS_CFG_SENSOR_RDY              (1<<6)
#define SPL06_MEAS_CFG_COEFFS_RDY              (1<<7)

// INT_AND_FIFO_CFG_REG
#define SPL06_PRESSURE_RESULT_BIT_SHIFT        (1<<2)  // necessary for pressure oversampling > 8
#define SPL06_TEMPERATURE_RESULT_BIT_SHIFT     (1<<3)  // necessary for temperature oversampling > 8


// Here comes the main configuration for the sensor

// Take care of measurement time, it may be longer than the MODE_POLLING_RATE, so not all measurements will fit/be done in one cycle
// Advised cfg register values are: pressure 0x26, temp 0xA0 for high precision and rate in background mode
#define SPL06_PRESSURE_OVERSAMPLING            32     
#define SPL06_TEMPERATURE_OVERSAMPLING         32

// per second while in background mode
#define SPL06_PRESSURE_MEASURE_RATE            8   
#define SPL06_TEMPERATURE_MEASURE_RATE         4

#define SPL06_BACKGROUND_MODE_POLLING_RATE	   20 * AP_USEC_PER_MSEC
#define SPL06_COMMAND_MODE_POLLING_RATE	       20 * AP_USEC_PER_MSEC

// Enable Background Mode for continuous measurement - default mode
#ifndef AP_BARO_SPL06_BACKGROUND_ENABLE
#define AP_BARO_SPL06_BACKGROUND_ENABLE 1
#endif

// Sensor driver configuration ends here

AP_Baro_SPL06::AP_Baro_SPL06(AP_Baro &baro, AP_HAL::OwnPtr<AP_HAL::Device> dev)
    : AP_Baro_Backend(baro)
    , _dev(std::move(dev))
{
}

AP_Baro_Backend *AP_Baro_SPL06::probe(AP_Baro &baro,
                                       AP_HAL::OwnPtr<AP_HAL::Device> dev)
{
    if (!dev) {
        return nullptr;
    }

    if (dev->bus_type() == AP_HAL::Device::BUS_TYPE_SPI) {
        dev->set_read_flag(0x80);
    }

    AP_Baro_SPL06 *sensor = NEW_NOTHROW AP_Baro_SPL06(baro, std::move(dev));
    if (!sensor || !sensor->_init()) {
        delete sensor;
        return nullptr;
    }
    return sensor;
}

bool AP_Baro_SPL06::_init()
{
    if (!_dev) {
        return false;
    }
    WITH_SEMAPHORE(_dev->get_semaphore());

    _dev->set_speed(AP_HAL::Device::SPEED_HIGH);

    // chip reset
    _dev->write_register(SPL06_REG_SOFT_RESET, SPL06_CMD_SOFT_RESET, true);    

    uint8_t meas_cfg;
    uint32_t time = AP_HAL::millis();

    do {    
        _dev->read_registers(SPL06_REG_MODE_AND_STATUS,&meas_cfg, sizeof(meas_cfg));          
    } while ((AP_HAL::millis() - time < 150) && !(meas_cfg & (SPL06_MEAS_CFG_COEFFS_RDY)));

    if (!(meas_cfg & SPL06_MEAS_CFG_COEFFS_RDY)){
        GCS_SEND_TEXT(MAV_SEVERITY_ALERT, "Baro Failure: Calibration data not ready");      
        return false; // calibration data not ready for to long time, quit init()
    }
    
    if (!get_calib_coeefs())
    {
        GCS_SEND_TEXT(MAV_SEVERITY_ALERT, "Baro Failure: Calibration data read failed");
        return false; // chip id failed?  
    }

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Baro Init: SPL06");

    // setup temperature and pressure measurements
    _dev->setup_checked_registers(3, 20);

#if AP_BARO_SPL06_BACKGROUND_ENABLE
    //set rate and oversampling
	_dev->write_register(SPL06_REG_TEMPERATURE_CFG, SPL06_MESSURE_RATE_TO_REG_VALUE(SPL06_TEMPERATURE_MEASURE_RATE) |  SPL06_OVERSAMPLING_TO_REG_VALUE(SPL06_TEMPERATURE_OVERSAMPLING) | SPL06_TEMP_USE_EXT_SENSOR , true);    
	_dev->write_register(SPL06_REG_PRESSURE_CFG, SPL06_MESSURE_RATE_TO_REG_VALUE(SPL06_PRESSURE_MEASURE_RATE) | SPL06_OVERSAMPLING_TO_REG_VALUE(SPL06_PRESSURE_OVERSAMPLING), true);

	//  Manufacturer advised cfg register values for high precision and rate in background mode
    // _dev->write_register(SPL06_REG_TEMPERATURE_CFG, 0xA0, true);    
	// _dev->write_register(SPL06_REG_PRESSURE_CFG, 0x26, true);

	//enable background mode
	_dev->write_register(SPL06_REG_MODE_AND_STATUS, SPL06_MEAS_CON_PRE_TEM, true);
#else
    _dev->write_register(SPL06_REG_TEMPERATURE_CFG, SPL06_TEMP_USE_EXT_SENSOR | SPL06_OVERSAMPLING_TO_REG_VALUE(SPL06_TEMPERATURE_OVERSAMPLING), true);
    _dev->write_register(SPL06_REG_PRESSURE_CFG, SPL06_OVERSAMPLING_TO_REG_VALUE(SPL06_PRESSURE_OVERSAMPLING), true);
#endif //AP_BARO_SPL06_BACKGROUND_ENABLE

    uint8_t int_and_fifo_reg_value = 0;
    if (SPL06_TEMPERATURE_OVERSAMPLING > 8) {
        int_and_fifo_reg_value |= SPL06_TEMPERATURE_RESULT_BIT_SHIFT;
    }
    if (SPL06_PRESSURE_OVERSAMPLING > 8) {
        int_and_fifo_reg_value |= SPL06_PRESSURE_RESULT_BIT_SHIFT;
    }
    _dev->write_register(SPL06_REG_INT_AND_FIFO_CFG, int_and_fifo_reg_value, true);

    _instance = _frontend.register_sensor();

    _dev->set_device_type(DEVTYPE_BARO_SPL06);
    set_bus_id(_instance, _dev->get_bus_id());
    
    // Request start temp messurement
    _timer_counter = -1;
#if AP_BARO_SPL06_BACKGROUND_ENABLE
    _dev->register_periodic_callback(SPL06_BACKGROUND_MODE_POLLING_RATE, FUNCTOR_BIND_MEMBER(&AP_Baro_SPL06::_timer, void));
#else
    //Start measurements
    _dev->write_register(SPL06_REG_MODE_AND_STATUS, SPL06_MEAS_TEMPERATURE, false);                      
    // _dev->write_register(SPL06_REG_MODE_AND_STATUS, SPL06_MEAS_PRESSURE, false);                                  
    _dev->register_periodic_callback(SPL06_COMMAND_MODE_POLLING_RATE, FUNCTOR_BIND_MEMBER(&AP_Baro_SPL06::_timer, void));    
#endif //AP_BARO_SPL06_BACKGROUND_ENABLE

    return true;
}

int32_t AP_Baro_SPL06::raw_value_scale_factor(uint8_t oversampling)
{
    // From the datasheet page 13
    switch(oversampling)
    {
        case 1: return 524288;
        case 2: return 1572864;
        case 4: return 3670016;
        case 8: return 7864320;
        case 16: return 253952;
        case 32: return 516096;
        case 64: return 1040384;
        case 128: return 2088960;
        default: return -1; // invalid
    }
}

static int32_t get_twos_complement(uint32_t raw, uint8_t length)
{
    if (raw & ((int)1 << (length - 1))) {
        return ((int32_t)raw) - ((int32_t)1 << length);
    }
    else {
        return raw;
    }
}

bool AP_Baro_SPL06::get_calib_coeefs(void)
{

    uint8_t whoami;

// Sometimes SPL06 has init problems, that's due to failure of reading using SPI for the first time. The SPL06 is a dual
// protocol sensor(I2C and SPI), sometimes it takes one SPI operation to convert it to SPI mode after it starts up.

    for (uint8_t i=0; i<5; i++) {
        if (_dev->read_registers(SPL06_REG_CHIP_ID, &whoami, 1)) {
            // whoami = SPL06_CHIP_ID;
            // whoami = SPA06_CHIP_ID;
            switch(whoami) {
			case SPL06_CHIP_ID:
				type = Type::SPL06;
				break;
			case SPA06_CHIP_ID:
				type = Type::SPA06;
				break;
			default:
				type = Type::UNKNOWN;
				break;
			}
        }

        if (type != Type::UNKNOWN)
			break;
    }
    
    if (type == Type::UNKNOWN) {
        return false;
    }



    // read the calibration data
    uint8_t SPL06_CALIB_COEFFS_LEN = 18;
	switch(type) {
	case Type::SPL06:
		SPL06_CALIB_COEFFS_LEN = SPL06_REG_CALIB_COEFFS_END - SPL06_REG_CALIB_COEFFS_START + 1;
		break;
	case Type::SPA06:
		SPL06_CALIB_COEFFS_LEN = SPA06_REG_CALIB_COEFFS_END - SPL06_REG_CALIB_COEFFS_START + 1;
		break;
	default:
		break;
	}


    // bool ready = false;
    // for (uint8_t i=0; i<5; i++) {
    //     uint8_t status = 0;
    //     if (_dev->read_registers(SPL06_REG_MODE_AND_STATUS, &status, 1)) {
    //         if ((status & 1<<7U) && (status & 1<<6U)) {
    //             ready = true;
    //             break;
    //         }
    //     }
    //     hal.scheduler->delay_microseconds(100);
    // }

    // if (!ready) {
    //     return false;
    // }

    // uint8_t buf[SPL06_CALIB_COEFFS_LEN];

    // #define READ_LENGTH 9

    // for (uint8_t i = 0; i < SPL06_CALIB_COEFFS_LEN; ) {
    //     ssize_t chunk = MIN(READ_LENGTH, SPL06_CALIB_COEFFS_LEN - i);
    //     if (!_dev->read_registers(SPL06_REG_CALIB_COEFFS_START + i, buf + i, chunk)) {
    //         return false;
    //     }
    //     i += chunk;
    // }

    // // 0x11 c0 [3:0] + 0x10 c0 [11:4]
    // _c0 = get_twos_complement(((uint32_t)buf[0] << 4) | (((uint32_t)buf[1] >> 4) & 0x0F), 12);
    // // 0x11 c1 [11:8] + 0x12 c1 [7:0]
    // _c1 = get_twos_complement((((uint32_t)buf[1] & 0x0F) << 8) | (uint32_t)buf[2], 12);
    // // 0x13 c00 [19:12] + 0x14 c00 [11:4] + 0x15 c00 [3:0]
    // _c00 = get_twos_complement(((uint32_t)buf[3] << 12) | ((uint32_t)buf[4] << 4) | (((uint32_t)buf[5] >> 4) & 0x0F), 20);
    // // 0x15 c10 [19:16] + 0x16 c10 [15:8] + 0x17 c10 [7:0]
    // _c10 = get_twos_complement((((uint32_t)buf[5] & 0x0F) << 16) | ((uint32_t)buf[6] << 8) | (uint32_t)buf[7], 20);
    // // 0x18 c01 [15:8] + 0x19 c01 [7:0]
    // _c01 = get_twos_complement(((uint32_t)buf[8] << 8) | (uint32_t)buf[9], 16);
    // // 0x1A c11 [15:8] + 0x1B c11 [7:0]
    // _c11 = get_twos_complement(((uint32_t)buf[10] << 8) | (uint32_t)buf[11], 16);
    // // 0x1C c20 [15:8] + 0x1D c20 [7:0]
    // _c20 = get_twos_complement(((uint32_t)buf[12] << 8) | (uint32_t)buf[13], 16);
    // // 0x1E c21 [15:8] + 0x1F c21 [7:0]
    // _c21 = get_twos_complement(((uint32_t)buf[14] << 8) | (uint32_t)buf[15], 16);
    // // 0x20 c30 [15:8] + 0x21 c30 [7:0]
    // _c30 = get_twos_complement(((uint32_t)buf[16] << 8) | (uint32_t)buf[17], 16);

    // if(type == Type::SPA06) {
    //     // 0x23 c31 [3:0] + 0x22 c31 [11:4]
    //     _c31 = get_twos_complement(((uint32_t)buf[18] << 4) | (((uint32_t)buf[19] >> 4) & 0x0F), 12);
    //     // 0x23 c40 [11:8] + 0x24 c40 [7:0]
    //     _c40 = get_twos_complement((((uint32_t)buf[19] & 0x0F) << 8) | (uint32_t)buf[20], 12);
    // }


    uint8_t buf[SPL06_CALIB_COEFFS_LEN];
    
    _dev->read_registers(SPL06_REG_CALIB_COEFFS_START, buf, sizeof(buf));

    _c0 = (buf[0] & 0x80 ? 0xF000 : 0) | ((uint16_t)buf[0] << 4) | (((uint16_t)buf[1] & 0xF0) >> 4);
    _c1 = ((buf[1] & 0x08 ? 0xF000 : 0) | ((uint16_t)buf[1] & 0x0F) << 8) | (uint16_t)buf[2];
    _c00 = (buf[3] & 0x80 ? 0xFFF00000 : 0) | ((uint32_t)buf[3] << 12) | ((uint32_t)buf[4] << 4) | (((uint32_t)buf[5] & 0xF0) >> 4);
    _c10 = (buf[5] & 0x08 ? 0xFFF00000 : 0) | (((uint32_t)buf[5] & 0x0F) << 16) | ((uint32_t)buf[6] << 8) | (uint32_t)buf[7];
    _c01 = ((uint16_t)buf[8] << 8) | ((uint16_t)buf[9]);
    _c11 = ((uint16_t)buf[10] << 8) | (uint16_t)buf[11];
    _c20 = ((uint16_t)buf[12] << 8) | (uint16_t)buf[13];
    _c21 = ((uint16_t)buf[14] << 8) | (uint16_t)buf[15];
    _c30 = ((uint16_t)buf[16] << 8) | (uint16_t)buf[17];
    if(type == Type::SPA06) {
		_c31 = (buf[18] & 0x80 ? 0xF000 : 0) | ((uint16_t)buf[18] << 4) | (((uint16_t)buf[19] & 0xF0) >> 4);
		_c40 = ((buf[19] & 0x8 ? 0xF000 : 0) | ((uint16_t)buf[19] & 0x0F) << 8) | (uint16_t)buf[20];
	}
    
    return true;   

}

// accumulate a new sensor reading
void AP_Baro_SPL06::_timer(void)
{
    uint8_t buf[3],meas_cfg;

    _dev->read_registers(SPL06_REG_MODE_AND_STATUS,&meas_cfg, sizeof(meas_cfg));

    //Quit if not ready
    if (!((meas_cfg & SPL06_MEAS_CFG_SENSOR_RDY) && (meas_cfg & SPL06_MEAS_CFG_COEFFS_RDY))) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Baro: Not ready");
        return;
    }
   
    if (meas_cfg & SPL06_MEAS_CFG_TEMPERATURE_RDY) {
        _dev->read_registers(SPL06_REG_TEMPERATURE_START, buf, sizeof(buf));
	    _update_temperature((int32_t)((buf[0] & 0x80 ? 0xFF000000 : 0) | ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2]));
        
        #if !AP_BARO_SPL06_BACKGROUND_ENABLE    
        _dev->write_register(SPL06_REG_MODE_AND_STATUS, SPL06_MEAS_PRESSURE, false);                         
        #endif

    }        
    
    if (meas_cfg & SPL06_MEAS_CFG_PRESSURE_RDY) {
	    _dev->read_registers(SPL06_REG_PRESSURE_START, buf, sizeof(buf));
	    _update_pressure((int32_t)((buf[0] & 0x80 ? 0xFF000000 : 0) | ((uint32_t)buf[0] << 16) | ((uint32_t)buf[1] << 8) | buf[2]));
        
        #if !AP_BARO_SPL06_BACKGROUND_ENABLE            
        _dev->write_register(SPL06_REG_MODE_AND_STATUS, SPL06_MEAS_TEMPERATURE, false);                               
        #endif

    }

    _dev->check_next_register();
}

// transfer data to the frontend
void AP_Baro_SPL06::update(void)
{
    WITH_SEMAPHORE(_sem);

    if (_pressure_count == 0) {
        return;
    }

    _copy_to_frontend(_instance, _pressure_sum/_pressure_count, _temperature);

    _pressure_sum = 0;
    _pressure_count = 0;
}

// calculate temperature
void AP_Baro_SPL06::_update_temperature(int32_t temp_raw)
{
    _temp_raw = (float)temp_raw / raw_value_scale_factor(SPL06_TEMPERATURE_OVERSAMPLING);
    const float temp_comp = (float)_c0 / 2 + _temp_raw * _c1;

    WITH_SEMAPHORE(_sem);

    _temperature = temp_comp;
}

// calculate pressure
void AP_Baro_SPL06::_update_pressure(int32_t press_raw)
{
    const float press_raw_sc = (float)press_raw / raw_value_scale_factor(SPL06_PRESSURE_OVERSAMPLING);
    float pressure_cal = 0;
    float press_temp_comp = 0;

    switch(type) {
	case Type::SPL06:
		pressure_cal = (float)_c00 + press_raw_sc * ((float)_c10 + press_raw_sc * ((float)_c20 + press_raw_sc * _c30));
		press_temp_comp = _temp_raw * ((float)_c01 + press_raw_sc * ((float)_c11 + press_raw_sc * _c21));
		break;
	case Type::SPA06:
		pressure_cal = (float)_c00 + press_raw_sc * ((float)_c10 + press_raw_sc * ((float)_c20 + press_raw_sc * ((float)_c30 + press_raw_sc * _c40)));
		press_temp_comp = _temp_raw * ((float)_c01 + press_raw_sc * ((float)_c11 + press_raw_sc * ((float)_c21) + press_raw_sc * _c31));
		break;
	default:
		break;
	}

    const float press_comp = pressure_cal + press_temp_comp;

    if (!pressure_ok(press_comp)) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Baro: Pressure out of range");
        return;
    }

    WITH_SEMAPHORE(_sem);

    _pressure_sum += press_comp;
    _pressure_count++;
}

#endif  // AP_BARO_SPL06_ENABLED
