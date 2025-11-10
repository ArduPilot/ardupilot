#include "SIM_config.h"

#if AP_SIM_LED_N_ENABLED

#include "SIM_LED_n.h"

#include <AP_HAL/HAL.h>

#ifdef HAVE_SFML_GRAPHICS_H
#include <SFML/Graphics.h>
#else
#include <SFML/Graphics.hpp>
#endif

#include <AP_Notify/AP_Notify.h>

template <uint8_t NUM_LEDS>
void SIM_LED_n<NUM_LEDS>::update_thread(void)
{
    sf::RenderWindow *w = nullptr;
    {
        WITH_SEMAPHORE(AP::notify().sf_window_mutex);
        w = NEW_NOTHROW sf::RenderWindow(sf::VideoMode(total_width, height), name);
        w->clear(sf::Color(0, 0, 0, 255));  // blacken
    }

    if (w == nullptr) {
        AP_HAL::panic("Unable to create SIM_LED_n window");
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
            if (memcmp(new_state, last_state, sizeof(new_state)) != 0) {
                memcpy(last_state, new_state, sizeof(last_state));
                w->clear(sf::Color(0, 0, 0, 255));  // blacken
                sf::RectangleShape rectangle(sf::Vector2f(width, height));
                for (uint8_t i=0; i<NUM_LEDS; i++) {
                    rectangle.setPosition(i*width, 0);
                    sf::Color led_colour;
                    if (new_state[i] == ON_VALUE) {
                        switch (led_colours[i]) {
                        case LEDColour::RED:
                            led_colour = sf::Color::Red;
                            break;
                        case LEDColour::GREEN:
                            led_colour = sf::Color::Green;
                            break;
                        case LEDColour::BLUE:
                            led_colour = sf::Color::Blue;
                            break;
                        case LEDColour::YELLOW:
                            led_colour = sf::Color::Yellow;
                            break;
                        }
                    } else {
                        led_colour = sf::Color::Black;
                    }
                    rectangle.setFillColor(led_colour);
                    w->draw(rectangle);
                }
                w->display();
            }
        }
        usleep(10000);
    }
}

// trampoline for update thread
template <uint8_t NUM_LEDS>
void *SIM_LED_n<NUM_LEDS>::update_thread_start(void *obj)
{
    ((SIM_LED_n *)obj)->update_thread();
    return nullptr;
}

template <uint8_t NUM_LEDS>
void SIM_LED_n<NUM_LEDS>::init()
{
    pthread_create(&thread, NULL, update_thread_start, this);
}

template class SIM_LED_n<1>;
template class SIM_LED_n<2>;
template class SIM_LED_n<3>;

#endif  // AP_SIM_LED_N_ENABLED
