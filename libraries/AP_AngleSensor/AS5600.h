#include "AP_AngleSensor_config.h"

#if AP_ANGLESENSOR_AS5600_ENABLED

#include <AP_HAL/I2CDevice.h>
#include <unistd.h>

class AP_AngleSensor_AS5600 {

public:

    AP_AngleSensor_AS5600(uint8_t _bus, uint8_t _address) :
        bus{_bus},
        address{_address}
        { }

    void init(void);
    void update(void);

private:

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev;

    uint8_t bus;
    uint8_t address;

    struct {
        HAL_Semaphore sem;
        uint16_t angle;
    } readings;

    struct {
        uint32_t last_reading_ms;
        uint16_t angle;
    } state;

    void timer();
};

#endif  // AP_ANGLESENSOR_AS5600_ENABLED
