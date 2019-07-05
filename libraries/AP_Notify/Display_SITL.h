#pragma once

#ifdef WITH_SITL_OSD

#include "Display.h"
#include "Display_Backend.h"

#ifdef HAVE_SFML_GRAPHICS_H
#include <SFML/Graphics.h>
#else
#include <SFML/Graphics.hpp>
#endif


class Display_SITL: public Display_Backend {

public:

    static Display_SITL *probe();

    void hw_update() override;
    void set_pixel(uint16_t x, uint16_t y) override;
    void clear_pixel(uint16_t x, uint16_t y) override;
    void clear_screen() override;

protected:

    Display_SITL();
    ~Display_SITL() override;

private:

    static constexpr const uint16_t COLUMNS = 132;
    static constexpr const uint8_t ROWS = 64;
    static constexpr const uint8_t SCALE = 4; // make it more readable

    bool hw_init() override;

    void _timer();

    uint8_t _displaybuffer[COLUMNS * ROWS];
    bool _need_hw_update;

    static void *update_thread_start(void *obj);
    void update_thread(void);
    sf::RenderWindow *w;
    pthread_t thread;
    HAL_Semaphore mutex;
};

#endif // WITH_SITL_OSD
