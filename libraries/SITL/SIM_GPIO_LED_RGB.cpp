#include "SIM_config.h"

#if AP_SIM_GPIO_LED_RGB_ENABLED

#include "SIM_GPIO_LED_RGB.h"

#include <SITL/SITL.h>

using namespace SITL;

void GPIO_LED_RGB::init()
{
    leds.init();
    rgbled.init();
}

void GPIO_LED_RGB::update(const class Aircraft &aircraft)
{
    if (!init_done) {
        init();
        init_done = true;
    }

    const uint16_t pin_mask = AP::sitl()->pin_mask.get();

    const bool red = ((pin_mask & uint16_t((1U<<LED_RED_PIN))) != 0);
    const bool green = ((pin_mask & uint16_t((1U<<LED_GREEN_PIN))) != 0);
    const bool blue = ((pin_mask & uint16_t((1U<<LED_BLUE_PIN))) != 0);

    const bool new_led_states[3] { red, green, blue };
    leds.set_state(new_led_states);

    // FIXME:  check why  we need  to  "!" here;  do we  need to  move
    // ON_VALUE into here?
    rgbled.set_colours((!red)*255, (!green)*255, (!blue)*255);
}

#endif
