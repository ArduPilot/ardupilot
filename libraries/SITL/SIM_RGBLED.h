#pragma once

#include <stdint.h>
#ifdef WITH_SITL_RGBLED
#include <pthread.h>
#endif

/*
  A class to create output of some description or another for RGB LEDs.

  Hopefully something visual, but perhaps just text
*/

class SIM_RGBLED
{
public:
    SIM_RGBLED(const char *_name) :
        name{_name}
        { }

    void init();

    void set_colours(uint8_t _red, uint8_t _green, uint8_t _blue) {
        red = _red;
        green = _green;
        blue = _blue;
    }

private:

    const char *name;

    uint8_t red;
    uint8_t green;
    uint8_t blue;

    static constexpr uint8_t height = 50;
    static constexpr uint8_t width = height;

#ifdef WITH_SITL_RGBLED
    pthread_t thread;
    static void *update_thread_start(void *obj);
    void update_thread(void);
#endif

    uint32_t last_colour;
};
