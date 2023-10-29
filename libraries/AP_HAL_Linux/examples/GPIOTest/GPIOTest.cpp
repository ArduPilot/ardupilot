#include <stdio.h>
#include <unistd.h>

#include <AP_Common/AP_Common.h>
#include <AP_HAL_Linux/AP_HAL_Linux.h>
#include <AP_Menu/AP_Menu.h>

void setup();
void loop();
int parse_gpio_pin_number(uint8_t argc, const Menu::arg *argv);

#define MENU_FUNC(func) FUNCTOR_BIND(&commands, &MenuCommands::func, int8_t, uint8_t, const Menu::arg *)

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

int parse_gpio_pin_number(uint8_t argc, const Menu::arg *argv) {
    if (argc != 2) {
        fprintf(stderr, "Input and output commands take only one argument, which is the GPIO pin number\n");
        return -1;
    }

    long int pin = argv[1].i;
    if (pin <= 0) {
        fprintf(stderr, "Invalid pin number: %ld\n", pin);
        return -1;
    }

    return pin;
}

static int8_t test_gpio_input(uint8_t argc, const Menu::arg *argv, bool use_channel) {
    AP_HAL::DigitalSource *ch = nullptr;
    int pin = parse_gpio_pin_number(argc, argv);

    if (pin <= 0) return -1;

    if (use_channel) {
        ch = hal.gpio->channel(pin);
        ch->mode(HAL_GPIO_INPUT);
    } else {
        hal.gpio->pinMode(pin, HAL_GPIO_INPUT);
    }

    hal.console->printf("Ok, I'll start reading pin number %d  and printing the value read on intervals of 1 second.", pin);
    while (1) {
        hal.console->printf("%u ", use_channel ? ch->read() : hal.gpio->read(pin));
        sleep(1);
    }
    return 0;
}

static int8_t test_gpio_output(uint8_t argc, const Menu::arg *argv, bool use_channel) {
    AP_HAL::DigitalSource *ch = nullptr;
    int pin = parse_gpio_pin_number(argc, argv);

    if (pin <= 0) return -1;

    if (use_channel) {
        ch = hal.gpio->channel(pin);
        ch->mode(HAL_GPIO_OUTPUT);
    } else {
        hal.gpio->pinMode(pin, HAL_GPIO_OUTPUT);
    }

    hal.console->printf("Now I'll start toggling the signal on the pin number %d on intervals of 1 second."
            " It's recommended to verify that the signal is reaching it (e.g. by using a LED)\n", pin);
    uint8_t signal = 0;
    while (1) {
        signal ^= 1;
        if (use_channel) {
            ch->write(signal);
        } else {
            hal.gpio->write(pin, signal);
        }
        sleep(1);
    }
    return 0;
}

class MenuCommands {
public:
    int8_t gpio_input(uint8_t argc, const Menu::arg *argv) {
        return ::test_gpio_input(argc, argv, false);
    }

    int8_t gpio_output(uint8_t argc, const Menu::arg *argv) {
        return ::test_gpio_output(argc, argv, false);
    }

    int8_t gpio_input_channel(uint8_t argc, const Menu::arg *argv) {
        hal.console->printf("GPIO Input using digital source\n");
        return test_gpio_input(argc, argv, true);
    }

    int8_t gpio_output_channel(uint8_t argc, const Menu::arg *argv) {
        hal.console->printf("GPIO Output using digital source\n");
        return test_gpio_output(argc, argv, true);
    }

};

MenuCommands commands;

const struct Menu::command test_menu_commands[] = {
    {"input",          MENU_FUNC(gpio_input)},
    {"output",         MENU_FUNC(gpio_output)},
    {"input_ch",       MENU_FUNC(gpio_input_channel)},
    {"output_ch",      MENU_FUNC(gpio_output_channel)}
};

MENU(main_menu, "GPIOTest: Please select one of the operations followed by the GPIO pin number", test_menu_commands);

void setup(void)
{
    Menu::set_port(hal.console);

    while (1) {
        main_menu.run();
    }
}

void loop(void)
{
}

AP_HAL_MAIN();
