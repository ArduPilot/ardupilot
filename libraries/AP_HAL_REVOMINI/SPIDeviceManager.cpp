#include <AP_HAL/AP_HAL.h>

#include <AP_HAL/AP_HAL.h>
#include "SPIDriver.h"
#include "SPIDevices.h"

using namespace REVOMINI;

extern const AP_HAL::HAL& hal;

void REVOMINISPIDeviceManager::init() {

    /* Note that the order of the init() of the MS5611 and MPU6k is
     * critical for the APM2. If you initialise in the wrong order
     * then the MS5611 doesn't initialise itself correctly. This
     * indicates an electrical fault in the APM2 which needs to be
     * investigated. Meanwhile, initialising the MPU6k CS pin before
     * the MS5611 CS pin works around the problem
     */

    _mpu6k = new REVOMINISPI1DeviceDriver(51);  // PA4
    _mpu6k->init();

    _dataflash  = new REVOMINISPI3DeviceDriver(104); // PB3
    _dataflash ->init();

}

AP_HAL::SPIDeviceDriver* REVOMINISPIDeviceManager::device(enum AP_HAL::SPIDeviceType d, uint8_t index)
{
    switch (d) {
        case AP_HAL::SPIDevice_Dataflash:
            return _dataflash;
        case AP_HAL::SPIDevice_MPU6000:
            return _mpu6k;
        default:
            return NULL;
    };
}
