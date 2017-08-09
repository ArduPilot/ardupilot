#include <AP_HAL/I2CDevice.h>
#include <AP_HAL_Empty/I2CDevice.h>

namespace HALSITL {

class I2CDeviceManager : public Empty::I2CDeviceManager {
public:
    uint32_t bus_detect_mask() const override { return 0; }
};

}
