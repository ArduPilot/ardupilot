/*

  Simulator for GPIO-based "Board LEDs"

  ./Tools/autotest/sim_vehicle.py -v ArduCopter --gdb --debug --rgbled

  reboot

 */


#include "SIM_config.h"

#if AP_SIM_GPIO_LED_1_ENABLED

#include "SIM_LED_n.h"

namespace SITL {

class GPIO_LED_1
{
public:

    GPIO_LED_1(uint8_t _LED_A_PIN) :
        LED_A_PIN{_LED_A_PIN}
        { }

    void update(const class Aircraft &aircraft);

private:

    void init();
    bool init_done;

    SIM_LED_n<1>::LEDColour colours[1] {
        SIM_LED_n<1>::LEDColour::RED,
    };

    SIM_LED_n<1> leds{"GPIO_LED_1", colours};

    uint8_t LED_A_PIN;
};

} // namespace SITL

#endif  // AP_SIM_GPIO_LED_2_ENABLED
