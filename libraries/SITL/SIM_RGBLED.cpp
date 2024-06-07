#include "SIM_RGBLED.h"

#ifdef WITH_SITL_RGBLED

#include <AP_HAL/HAL.h>

#ifdef HAVE_SFML_GRAPHICS_H
#include <SFML/Graphics.h>
#else
#include <SFML/Graphics.hpp>
#endif

#include <AP_Notify/AP_Notify.h>

void SIM_RGBLED::update_thread(void)
{
    sf::RenderWindow *w = nullptr;
    {
        WITH_SEMAPHORE(AP::notify().sf_window_mutex);
        w = NEW_NOTHROW sf::RenderWindow(sf::VideoMode(width, height), name);
    }

    if (w == nullptr) {
        AP_HAL::panic("Unable to create SIM_RGBLED window");
    }

    while (true) {
        {
            WITH_SEMAPHORE(AP::notify().sf_window_mutex);
            sf::Event event;
            while (w->pollEvent(event)) {
                if (event.type == sf::Event::Closed) {
                    w->close();
                    break;
                }
            }
            if (!w->isOpen()) {
                break;
            }
            const uint32_t colour = red<<16 | green<<8 | blue;
            if (colour != last_colour) {
                last_colour = colour;
                w->clear(sf::Color(red, green, blue, 255));
                w->display();
            }
        }
        usleep(10000);
    }
}

// trampoline for update thread
void *SIM_RGBLED::update_thread_start(void *obj)
{
    ((SIM_RGBLED *)obj)->update_thread();
    return nullptr;
}

#endif  // WITH_SITL_RGBLED

void SIM_RGBLED::init()
{
#ifdef WITH_SITL_RGBLED
    pthread_create(&thread, NULL, update_thread_start, this);
#endif
}
