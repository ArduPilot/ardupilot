/*

  Simulator for GPIO-based "Board LEDs"

  ./Tools/autotest/sim_vehicle.py -v ArduCopter --gdb --debug --rgbled

  reboot

 */


#include "SIM_config.h"

#if AP_SIM_GPIO_LED_3_ENABLED

#include "SIM_LED_n.h"

namespace SITL {

class GPIO_LED_3
{
public:

    GPIO_LED_3(uint8_t _LED_A_PIN, uint8_t _LED_B_PIN, uint8_t _LED_C_PIN) :
        LED_A_PIN{_LED_A_PIN},
        LED_B_PIN{_LED_B_PIN},
        LED_C_PIN{_LED_C_PIN}
        { }

    void update(const class Aircraft &aircraft);

private:

    void init();
    bool init_done;

    SIM_LED_n<3>::LEDColour colours[3] {
        SIM_LED_n<3>::LEDColour::RED,
        SIM_LED_n<3>::LEDColour::BLUE,
        SIM_LED_n<3>::LEDColour::YELLOW,
    };

    SIM_LED_n<3> leds{"GPIO_LED_3", colours};

    uint8_t LED_A_PIN;
    uint8_t LED_B_PIN;
    uint8_t LED_C_PIN;
};

} // namespace SITL

#endif  // AP_SIM_GPIO_LED_3_ENABLED
