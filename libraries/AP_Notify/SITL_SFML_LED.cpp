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
#include <SITL/SITL.h>

#include "SITL_SFML_LED.h"

/*
  return layout size for a LED layout scheme
 */
bool SITL_SFML_LED::layout_size(SITL::LedLayout layout, uint16_t &xsize, uint16_t &ysize)
{
    switch (layout) {
    case SITL::LedLayout::ROWS:
        xsize = MAX_LEDS;
        ysize = 16;
        break;

    case SITL::LedLayout::LUMINOUSBEE:
        xsize = 5 + 3 + 5;
        ysize = 5 + 3 + 5;
        break;
    default:
        return false;
    }
    return true;
}

/*
  return layout position for a LED layout scheme
 */
bool SITL_SFML_LED::layout_pos(SITL::LedLayout layout, uint8_t chan, uint8_t led, uint16_t &xpos, uint16_t &ypos)
{
    switch (layout) {
    case SITL::LedLayout::ROWS:
        xpos = led;
        ypos = chan;
        break;

    case SITL::LedLayout::LUMINOUSBEE:
        /*
          Luminousbee has 60 LEDs, 15 on each arm, connected on PWM7
         */
        if (chan != 6) {
            return false;
        }
        if (led < 15) {
            xpos = 5 + led / 5;
            ypos = led % 5;
            if (((led/5) & 1) == 0) {
                ypos = 4 - ypos;
            }
        } else if (led < 30) {
            led -= 15;
            xpos = led % 5;
            ypos = 5 + led / 5;
            if (((led/5) & 1) == 1) {
                xpos = 4 - xpos;
            }
            xpos += 8;
        } else if (led < 45) {
            led -= 30;
            xpos = 5 + led / 5;
            ypos = led % 5;
            if (((led/5) & 1) == 0) {
                ypos = 4 - ypos;
            }
            ypos += 8;
        } else if (led < 60) {
            led -= 45;
            xpos = led % 5;
            ypos = 5 + led / 5;
            if (((led/5) & 1) == 0) {
                xpos = 4 - xpos;
            }
        } else {
            return false;
        }
        break;
    default:
        return false;
    }
    return true;
}

/*
  update simulation of WS2812 (NeoPixel) serial LEDs
 */
void SITL_SFML_LED::update_serial_LEDs()
{
    static sf::RenderWindow *w;
    static sf::RectangleShape *leds[16][MAX_LEDS];

    SITL::SIM *sitl = AP::sitl();
    if (sitl == nullptr || sitl->led.send_counter == 0) {
        // no SerialLEDs set
        return;
    }
    SITL::LedLayout layout = SITL::LedLayout(sitl->led_layout.get());
    if (w == nullptr) {
        uint8_t max_leds = 0;
        for (uint8_t i=0; i<16; i++) {
            max_leds = MAX(max_leds, sitl->led.num_leds[i]);
        }
        uint16_t xsize=0, ysize=0;
        if (!layout_size(layout, xsize, ysize)) {
            return;
        }
        w = NEW_NOTHROW sf::RenderWindow(sf::VideoMode(xsize*(serialLED_size+1), ysize*(serialLED_size+1)), "SerialLED");
        if (!w) {
            return;
        }
        w->clear(sf::Color(0, 0, 0, 255));
    }

    WITH_SEMAPHORE(AP::notify().sf_window_mutex);
    sf::Event event;
    while (w->pollEvent(event)) {
        if (event.type == sf::Event::Closed) {
            w->close();
            break;
        }
    }
    if (!w->isOpen()) {
        return;
    }

    for (uint8_t chan=0; chan<16; chan++) {
        for (uint8_t led=0; led<sitl->led.num_leds[chan]; led++) {
            uint8_t *rgb = &sitl->led.rgb[chan][led].rgb[0];

            if (leds[chan][led] == nullptr) {
                leds[chan][led] = NEW_NOTHROW sf::RectangleShape(sf::Vector2f(serialLED_size, serialLED_size));
                if (!leds[chan][led]) {
                    return;
                }
                uint16_t xpos, ypos;
                if (layout_pos(layout, chan, led, xpos, ypos)) {
                    leds[chan][led]->setPosition(xpos*(serialLED_size+1), ypos*(serialLED_size+1));
                }
            }
            leds[chan][led]->setFillColor(sf::Color(rgb[0], rgb[1], rgb[2], 255));
            w->draw(*leds[chan][led]);
        }
    }
    w->display();
}

void SITL_SFML_LED::update_thread(void)
{
    while (true) {
        {
            WITH_SEMAPHORE(AP::notify().sf_window_mutex);
            update_serial_LEDs();
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

bool SITL_SFML_LED::init()
{
    pthread_create(&thread, NULL, update_thread_start, this);

    return true;
}

void SITL_SFML_LED::update()
{
    // all updates are done in the thread
}

#endif
