/*
   Copyright (C) 2019 Peter Barker. All rights reserved.

   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
#include <AP_HAL/AP_HAL.h>
#include <AP_Notify/AP_Notify.h>

#include <stdio.h>
#include <unistd.h>

#ifdef WITH_SITL_RGBLED

#include "SITL_SFML_LED.h"

SITL_SFML_LED::SITL_SFML_LED():
    RGBLed((uint8_t)brightness::LED_OFF,
           (uint8_t)brightness::LED_HIGH,
           (uint8_t)brightness::LED_MEDIUM,
           (uint8_t)brightness::LED_LOW)
{
}

void SITL_SFML_LED::update_thread(void)
{
    sf::RenderWindow *w;
    {
        WITH_SEMAPHORE(AP::notify().sf_window_mutex);
        w = new sf::RenderWindow(sf::VideoMode(width, height), "LED");
    }

    if (!w) {
        AP_HAL::panic("Unable to create RGBLed window");
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
void *SITL_SFML_LED::update_thread_start(void *obj)
{
    ((SITL_SFML_LED *)obj)->update_thread();
    return nullptr;
}

bool SITL_SFML_LED::hw_init()
{
    pthread_create(&thread, NULL, update_thread_start, this);

    return true;
}

bool SITL_SFML_LED::hw_set_rgb(uint8_t _red, uint8_t _green, uint8_t _blue)
{
    red = _red;
    green = _green;
    blue = _blue;

    return true;
}
#endif
