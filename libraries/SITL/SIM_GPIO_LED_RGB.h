/*

  Simulator for GPIO-based "Board LEDs"

  ./Tools/autotest/sim_vehicle.py -v ArduCopter --gdb --debug --rgbled

  reboot

 */


#include "SIM_config.h"

#if AP_SIM_GPIO_LED_RGB_ENABLED

#include "SIM_LED_n.h"
#include "SIM_RGBLED.h"

namespace SITL {

class GPIO_LED_RGB
{
public:

    GPIO_LED_RGB(uint8_t _LED_RED_PIN, uint8_t _LED_GREEN_PIN, uint8_t _LED_BLUE_PIN) :
        LED_RED_PIN{_LED_RED_PIN},
        LED_GREEN_PIN{_LED_GREEN_PIN},
        LED_BLUE_PIN{_LED_BLUE_PIN}
        { }

    void update(const class Aircraft &aircraft);

private:

    void init();
    bool init_done;

    SIM_LED_n<3>::LEDColour colours[3] {
        SIM_LED_n<3>::LEDColour::RED,
        SIM_LED_n<3>::LEDColour::GREEN,
        SIM_LED_n<3>::LEDColour::BLUE,
    };

    SIM_LED_n<3> leds{"GPIO_LED_RGB", colours};
    SIM_RGBLED rgbled{"GPIO_LED_RGB Mixed"};

    uint8_t LED_RED_PIN;
    uint8_t LED_GREEN_PIN;
    uint8_t LED_BLUE_PIN;
};

} // namespace SITL

#endif  // AP_SIM_GPIO_LED_RGB_ENABLED
