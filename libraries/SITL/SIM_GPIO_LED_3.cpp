#include "SIM_config.h"

#if AP_SIM_GPIO_LED_3_ENABLED

#include "SIM_GPIO_LED_3.h"

#include <SITL/SITL.h>

using namespace SITL;

void GPIO_LED_3::init()
{
    leds.init();
}

void GPIO_LED_3::update(const class Aircraft &aircraft)
{
    if (!init_done) {
        init();
        init_done = true;
    }

    const uint16_t pin_mask = AP::sitl()->pin_mask.get();
    const bool new_led_states[3] {
        ((pin_mask & uint16_t((1U<<LED_A_PIN))) != 0),
        ((pin_mask & uint16_t((1U<<LED_B_PIN))) != 0),
        ((pin_mask & uint16_t((1U<<LED_C_PIN))) != 0)
    };

    leds.set_state(new_led_states);
}

#endif
