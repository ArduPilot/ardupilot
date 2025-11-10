/*
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
#ifdef WITH_SITL_OSD

#include "Display_SITL.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Notify/AP_Notify.h>

#include <stdio.h>
#include <unistd.h>

// constructor
Display_SITL::Display_SITL()
{
}

Display_SITL::~Display_SITL()
{
}

Display_SITL *Display_SITL::probe()
{
    Display_SITL *driver = NEW_NOTHROW Display_SITL();
    if (!driver || !driver->hw_init()) {
        delete driver;
        return nullptr;
    }
    return driver;
}

// main loop of graphics thread
void Display_SITL::update_thread(void)
{
    {
        WITH_SEMAPHORE(AP::notify().sf_window_mutex);
        w = NEW_NOTHROW sf::RenderWindow(sf::VideoMode(COLUMNS*SCALE, ROWS*SCALE), "Display");
    }
    if (!w) {
        AP_HAL::panic("Unable to create Display_SITL window");
    }

    const sf::Color color_black = sf::Color(0,0,0);
    const sf::Color color_white = sf::Color(255,255,255);

    const sf::Uint8 pixels[ROWS*COLUMNS*4]{};
    sf::Image image;
    image.create(COLUMNS, ROWS, pixels);

    while (true) {
        {
            WITH_SEMAPHORE(AP::notify().sf_window_mutex);
            sf::Event event;
            while (w->pollEvent(event)) {
                if (event.type == sf::Event::Closed) {
                    w->close();
                }
            }
            if (!w->isOpen()) {
                break;
            }
            if (_need_hw_update) {
                _need_hw_update = false;

                uint8_t buffer2[ROWS*COLUMNS];
                {
                    WITH_SEMAPHORE(mutex);
                    memcpy(buffer2, _displaybuffer, sizeof(buffer2));
                }
                w->clear();

                for (uint16_t y=0; y<ROWS; y++) {
                    for (uint16_t x=0; x<COLUMNS; x++) {
                        if (buffer2[x+y/8*COLUMNS] & 1<<y%8) {
                            image.setPixel(x, y, color_white);
                        } else {
                            image.setPixel(x, y, color_black);
                        }
                    }
                }

                sf::Texture texture;
                texture.loadFromImage(image);
                sf::Sprite sprite;
                sprite.setTexture(texture, true);
                sprite.setScale(SCALE, SCALE);
                w->draw(sprite);

                w->display();
            }
        }
        usleep(10000);
    }
}

// trampoline for update thread
void *Display_SITL::update_thread_start(void *obj)
{
    ((Display_SITL *)obj)->update_thread();
    return nullptr;
}

bool Display_SITL::hw_init()
{
    pthread_create(&thread, NULL, update_thread_start, this);
    _need_hw_update = true;

    return true;
}

void Display_SITL::hw_update()
{
    _need_hw_update = true;
}

void Display_SITL::set_pixel(uint16_t x, uint16_t y)
{
    // check x, y range
    if ((x >= COLUMNS) || (y >= ROWS)) {
        return;
    }
    // set pixel in buffer
    WITH_SEMAPHORE(mutex);
    _displaybuffer[x + (y / 8 * COLUMNS)] |= 1 << (y % 8);
    _need_hw_update = true;
}

void Display_SITL::clear_pixel(uint16_t x, uint16_t y)
{
    // check x, y range
    if ((x >= COLUMNS) || (y >= ROWS)) {
        return;
    }
    // clear pixel in buffer
    WITH_SEMAPHORE(mutex);
    _displaybuffer[x + (y / 8 * COLUMNS)] &= ~(1 << (y % 8));
    _need_hw_update = true;
}

void Display_SITL::clear_screen()
{
    WITH_SEMAPHORE(mutex);
    memset(_displaybuffer, 0, sizeof(_displaybuffer));
    _need_hw_update = true;
}

#endif // WITH_SITL_OSD
