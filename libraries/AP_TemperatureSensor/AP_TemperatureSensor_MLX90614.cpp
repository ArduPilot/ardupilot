#include "AP_TemperatureSensor_config.h"

#if AP_TEMPERATURE_SENSOR_MLX90614_ENABLED

#include "AP_TemperatureSensor_MLX90614.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>
#include <AP_Math/AP_Math.h>


extern const AP_HAL::HAL &hal;

#define MLX90614_I2CDEFAULTADDR 0x5A    // Device default slave address 
#define MLX90614_BROADCASTADDR  0       // Device broadcast slave address 

//  RAM addresses
#define MLX90614_RAWIR1         0x04    //  RAM reg - Raw temperature, source #1 
#define MLX90614_RAWIR2         0x05    //  RAM reg - Raw temperature, source #2 
#define MLX90614_TA             0x06    //  RAM reg - Linearized temperature, ambient 
#define MLX90614_TOBJ1          0x07    //  RAM reg - Linearized temperature, source #1 
#define MLX90614_TOBJ2          0x08    //  RAM reg - Linearized temperature, source #2 

void AP_TemperatureSensor_MLX90614::init()
{
    _params.bus_address.set_default(MLX90614_I2CDEFAULTADDR);
    
    _dev = hal.i2c_mgr->get_device_ptr(_params.bus, _params.bus_address);
    if (!_dev) {
        return;
    }

    WITH_SEMAPHORE(_dev->get_semaphore());

    _dev->register_periodic_callback(50 * AP_USEC_PER_MSEC,
                                     FUNCTOR_BIND_MEMBER(&AP_TemperatureSensor_MLX90614::_timer, void));
}


void AP_TemperatureSensor_MLX90614::_timer()
{
    // read the temperature of the object the sensor is pointed at,
    // not the sensor's own (ambient) temperature
    uint16_t _crude_value;
    if (!read_data(MLX90614_TOBJ1, _crude_value)) {
        return;
    }

    // the top bit is set if the sensor has an error
    if ((_crude_value & 0x8000) != 0) {
        return;
    }

    WITH_SEMAPHORE(_dev->get_semaphore());
        
    // temp * 0.02 - 273.15 = degrees, temp * 0.02 is temperature in kelvin
    const float tmp = KELVIN_TO_C(_crude_value * 0.02);
    set_temperature(tmp);
}



bool AP_TemperatureSensor_MLX90614::read_data(uint8_t cmd, uint16_t &value)
{
    uint8_t val[3];
  
    if (!_dev->transfer(&cmd, 1, val, 3)) {
        return false;
    }

    // the third byte is a CRC-8 (polynomial 0x07) over the write
    // address, command, read address and the two data bytes
    const uint8_t addr = _dev->get_bus_address() << 1;
    const uint8_t pec_data[] { addr, cmd, uint8_t(addr | 1), val[0], val[1] };
    if (crc8_generic(pec_data, sizeof(pec_data), 0x07) != val[2]) {
        return false;
    }

    value = UINT16_VALUE(val[1],val[0]);
    return true;
}
#endif // AP_TEMPERATURE_SENSOR_MLX90614_ENABLED
