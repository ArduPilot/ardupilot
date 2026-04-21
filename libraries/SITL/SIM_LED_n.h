#pragma once

#include "SIM_config.h"

#if AP_SIM_LED_N_ENABLED

#include <stdint.h>
#include <pthread.h>

/*
  A class to create output of some description or another for a group
  of N LEDs.

  Hopefully something visual, but perhaps just text
*/

template <uint8_t NUM_LEDS>
class SIM_LED_n
{
public:
    enum class LEDColour : uint8_t {
        RED,
        GREEN,
        BLUE,
        YELLOW,
    };

    SIM_LED_n<NUM_LEDS>(const char *_name, const LEDColour _led_colours[NUM_LEDS]) :
        name{_name}
        {
            memcpy(led_colours, _led_colours, sizeof(led_colours));
        }

    void set_state(const bool state[NUM_LEDS]) {
        memcpy(new_state, state, sizeof(new_state));
    }

    void init();

private:

    const char *name;
    LEDColour led_colours[NUM_LEDS];

    static constexpr uint8_t height = 50;
    static constexpr uint8_t width = height;
    static constexpr uint8_t total_width = width * NUM_LEDS;

    pthread_t thread;
    static void *update_thread_start(void *obj);
    void update_thread(void);

    // state to be written to LEDs; note lack of thread protection.
    bool new_state[NUM_LEDS];

    // avoid too-frequent display updates:
    bool last_state[NUM_LEDS];

    const bool ON_VALUE = 0;  // so if the pin is low we are lit
};

#endif  // AP_SIM_LED_N_ENABLED
