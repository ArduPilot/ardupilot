
#include <AP_Common.h>
#include <AP_Math.h>
#include <AP_Param.h>
#include <AP_Progmem.h>

#include <AP_Menu.h>
#include <AP_HAL.h>
#include <AP_HAL_AVR.h>

const AP_HAL::HAL& hal = AP_HAL_BOARD_DRIVER;

int8_t
menu_test(uint8_t argc, const Menu::arg *argv)
{
    int i;

    hal.console->printf("This is a test with %d arguments\n", argc);
    for (i = 1; i < argc; i++) {
        hal.console->printf("%d: int %ld  float ", i, argv[i].i);
        hal.console->println(argv[i].f, 6);    // gross
    }
    return 0;
}

int8_t
menu_auto(uint8_t argc, const Menu::arg *argv)
{
    hal.console->println("auto text");
    return 0;
}

const struct Menu::command top_menu_commands[] PROGMEM = {
    {"*",               menu_auto},
    {"test",                    menu_test},
};

MENU(top, "menu", top_menu_commands);

void
setup(void)
{
    hal.console->println_P(PSTR("AP_Menu unit test"));
    top.run();
}

void
loop(void)
{
}

AP_HAL_MAIN();
