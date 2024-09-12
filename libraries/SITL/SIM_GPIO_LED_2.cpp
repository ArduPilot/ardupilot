#include "SIM_config.h"

#if AP_SIM_GPIO_LED_2_ENABLED

#include "SIM_GPIO_LED_2.h"

#include <SITL/SITL.h>

using namespace SITL;

void GPIO_LED_2::init()
{
    leds.init();
}

void GPIO_LED_2::update(const class Aircraft &aircraft)
{
    if (!init_done) {
        init();
        init_done = true;
    }

    const uint16_t pin_mask = AP::sitl()->pin_mask.get();
    const bool new_led_states[2] {
        ((pin_mask & uint16_t((1U<<LED_A_PIN))) != 0),
        ((pin_mask & uint16_t((1U<<LED_B_PIN))) != 0)
    };

    leds.set_state(new_led_states);
}

#endif
