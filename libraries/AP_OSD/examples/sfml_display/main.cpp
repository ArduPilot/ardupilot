#include <SFML/Graphics.hpp>
#include <SFML/Window.hpp>

#include <iostream>
#include <chrono>
#include <thread>
#include <mutex>
#include <memory>
#include <string>

class Display
{
public:
    Display(const std::string& title)
      : title(title)
    {
    }

    ~Display()
    {
        stop_render_thread();
    }

    /// \note call on main thread
    void init() {
        window = std::make_unique<sf::Window>();
        window->create(sf::VideoMode(400, 400), title);

        start_render_thread();
    }

    /// \note call on main thread
    void update() {
      poll_events();
    }

    /// \note call on any thread
    bool is_open() {
      std::lock_guard<std::mutex> lock(window_mutex);

      return window->isOpen();
    }

private:

    /// \note call on main thread
    void start_render_thread() {
        render_thread = std::thread(&Display::render, this);
    }

    /// \note run on main thread
    void stop_render_thread() {

        // shutdown window
        {
            std::lock_guard<std::mutex> lock(window_mutex);
            window->setVisible(false);
            is_closing = true;
        }

        poll_events();

        // joint thread
        render_thread.join();

        std::cout << "stopped render thread: " << title << "\n";
    }

    /// \note call on render thread
    void render() {
        while (true) {
            {
                std::lock_guard<std::mutex> lock(window_mutex);
                if (window->isOpen()) {
                    std::cout << "rendering: " << title << "\n";
                }
                else {
                    std::cout << "stopped rendering: " << title << "\n";
                    break;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
    }

    /// \note call on main thread
    void poll_events() {
        std::lock_guard<std::mutex> lock(window_mutex);

        if (window->isOpen()) {
            if (is_closing) {
                window->close();
            }
            sf::Event event;
            while (window->pollEvent(event))
            {
                if (event.type == sf::Event::Closed) {
                    window->setVisible(false);
                    is_closing = true;
                }
            }
        }
    }

    std::string title;
    std::unique_ptr<sf::Window> window;
    bool is_closing {false};
    std::mutex window_mutex;
    std::thread render_thread;
};

int main()
{
    {
        Display display1("Display 1");
        Display display2("Display 2");

        display1.init();
        display2.init();

        // main loop
        while (display1.is_open() || display2.is_open())
        {
            display1.update();
            display2.update();

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    // exit
    std::cout << "Displays closed\n";
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    std::cout << "Exiting\n";
    return 0;
  }
