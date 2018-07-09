/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */
/*
  OSD backend for SITL. Uses SFML media library. See
  https://www.sfml-dev.org/index.php

  To use install SFML libraries, and run sim_vehicle.py with --osd
  option. Then set OSD_TYPE to 2
 */
#ifdef WITH_SITL_OSD

#include "AP_OSD_SITL.h"
#include <AP_HAL/Util.h>
#include <AP_HAL/Semaphores.h>
#include <AP_HAL/Scheduler.h>
#include <AP_ROMFS/AP_ROMFS.h>
#include <utility>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <unistd.h>
#include "pthread.h"

extern const AP_HAL::HAL &hal;

/*
  load *.bin font file, in MAX7456 format
 */
void AP_OSD_SITL::load_font(void)
{
    uint32_t font_size;
    uint8_t *font_data = AP_ROMFS::find_decompress("font0.bin", font_size);
    if (font_data == nullptr || font_size != 54 * 256) {
        AP_HAL::panic("Bad font file");
    }
    for (uint16_t i=0; i<256; i++) {
        const uint8_t *c = &font_data[i*54];
        // each pixel is 4 bytes, RGBA
        sf::Uint8 *pixels = new sf::Uint8[char_width * char_height * 4];
        if (!font[i].create(char_width, char_height)) {
            AP_HAL::panic("Failed to create texture");
        }
        for (uint16_t y=0; y<char_height; y++) {
            for (uint16_t x=0; x<char_width; x++) {
                // 2 bits per pixel
                uint16_t bitoffset = (y*char_width+x)*2;
                uint8_t byteoffset = bitoffset / 8;
                uint8_t bitshift = 6-(bitoffset % 8);
                uint8_t v = (c[byteoffset] >> bitshift) & 3;
                sf::Uint8 *p = &pixels[(y*char_width+x)*4];
                switch (v) {
                case 0:
                    p[0] = 0;
                    p[1] = 0;
                    p[2] = 0;
                    p[3] = 255;
                    break;
                case 1:
                case 3:
                    p[0] = 0;
                    p[1] = 0;
                    p[2] = 0;
                    p[3] = 0;
                    break;
                case 2:
                    p[0] = 255;
                    p[1] = 255;
                    p[2] = 255;
                    p[3] = 255;
                    break;
                }
                    
            }
        }
        font[i].update(pixels);
    }
    free(font_data);
}

void AP_OSD_SITL::write(uint8_t x, uint8_t y, const char* text, uint8_t char_attr)
{
    if (y >= video_lines || text == nullptr) {
        return;
    }
    mutex->take_blocking();
    while ((x < video_cols) && (*text != 0)) {
        buffer[y][x] = *text;
        attr[y][x] = char_attr;
        ++text;
        ++x;
    }
    mutex->give();
}

void AP_OSD_SITL::clear(void)
{
    mutex->take_blocking();
    memset(buffer, 0, sizeof(buffer));
    mutex->give();
}

void AP_OSD_SITL::flush(void)
{
    counter++;
}

// main loop of graphics thread
void AP_OSD_SITL::update_thread(void)
{
    load_font();
    w = new sf::RenderWindow(sf::VideoMode(video_cols*(char_width+char_spacing)*char_scale,
                                           video_lines*(char_height+char_spacing)*char_scale),
                             "OSD");
    if (!w) {
        AP_HAL::panic("Unable to create OSD window");
    }

    uint8_t blink = 0;
    while (w->isOpen())
    {
        sf::Event event;
        while (w->pollEvent(event))
        {
            if (event.type == sf::Event::Closed)
                w->close();
        }
        if (counter == last_counter) {
            usleep(10000);
            continue;
        }
        last_counter = counter;

        uint8_t buffer2[video_lines][video_cols];
        uint8_t attr2[video_lines][video_cols];
        mutex->take_blocking();
        memcpy(buffer2, buffer, sizeof(buffer2));
        memcpy(attr2, attr, sizeof(attr2));
        mutex->give();
        w->clear();

        for (uint8_t y=0; y<video_lines; y++) {
            for (uint8_t x=0; x<video_cols; x++) {
                uint16_t px = x * (char_width+char_spacing) * char_scale;
                uint16_t py = y * (char_height+char_spacing) * char_scale;
                sf::Sprite s;
                uint8_t c = buffer2[y][x];
                if (attr2[y][x] & BLINK) {
                    if (blink >= 2) {
                        c = ' ';
                    }
                }
                s.setTexture(font[c]);
                s.setPosition(sf::Vector2f(px, py));
                s.scale(sf::Vector2f(char_scale,char_scale));
                w->draw(s);
            }
        }
        
        blink = (blink+1) % 4;
        w->display();
        usleep(10000);
    }
}

// trampoline for update thread
void *AP_OSD_SITL::update_thread_start(void *obj)
{
    ((AP_OSD_SITL *)obj)->update_thread();
    return nullptr;
}

// initialise backend
bool AP_OSD_SITL::init(void)
{
    mutex = hal.util->new_semaphore();
    pthread_create(&thread, NULL, update_thread_start, this);
    return true;
}

AP_OSD_Backend *AP_OSD_SITL::probe(AP_OSD &osd)
{
    AP_OSD_SITL *backend = new AP_OSD_SITL(osd);
    if (!backend) {
        return nullptr;
    }
    if (!backend->init()) {
        delete backend;
        return nullptr;
    }
    return backend;
}

AP_OSD_SITL::AP_OSD_SITL(AP_OSD &osd):
    AP_OSD_Backend(osd)
{
}

#endif // WITH_SITL_OSD
