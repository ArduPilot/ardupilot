#include "SIM_I2CDevice.h"

namespace SITL {

class ToshibaLED : public I2CDevice
{
public:
    int rdwr(I2C::i2c_rdwr_ioctl_data *&data) override;
    // void update(const struct sitl_input input) override;
private:
    bool _enabled;
    bool _pwm0; // FIXME: just an array of register values?!
    bool _pwm1;
    bool _pwm2;
    bool _pwm3;
    uint32_t last_internal_clock_update_ms;
};

} // namespace SITL
