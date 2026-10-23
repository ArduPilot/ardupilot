#include "AP_Mount_config.h"

#if HAL_MOUNT_ENABLED

#include "AP_Mount_Backend_Serial.h"

#include <AP_SerialManager/AP_SerialManager.h>

// Default init function for every mount
void AP_Mount_Backend_Serial::init()
{
    const AP_SerialManager& serial_manager = AP::serialmanager();

    // search for serial port.  hild classes should check that uart is not nullptr
    _uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_Gimbal, _serial_instance);
    if (_uart == nullptr) {
        return;
    }

    // initialised successfully if uart is found
    _initialised = true;

    // call the parent class init
    AP_Mount_Backend::init();
}

#endif // HAL_MOUNT_ENABLED
