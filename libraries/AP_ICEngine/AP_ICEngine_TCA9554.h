/*
  optional control of starter via a TCA9554 I2C
 */

#include "AP_ICEngine_config.h"

#if AP_ICENGINE_TCA9554_STARTER_ENABLED
#include "AP_ICEngine.h"

class AP_ICEngine_TCA9554 {
public:
    void set_starter(bool on, bool crank_dir_reverse);

private:
    AP_HAL::OwnPtr<AP_HAL::I2CDevice> dev_TCA9554;

    enum TCA9554_state_t {
        STARTER_OFF = 0x30, // output register - 0011 0000
        STARTER_FORWARD = 0x11, // output register - 0001 0001 - Forward direction
        STARTER_REVERSE = 0x01, // output register - 0000 0001 - Reverse direction
    };
    TCA9554_state_t last_state;

    bool initialised;

    bool TCA9554_init();
    void TCA9554_set(TCA9554_state_t value);
    uint32_t last_reg_check_ms;
};

#endif // AP_ICENGINE_TCA9554_STARTER_ENABLED
