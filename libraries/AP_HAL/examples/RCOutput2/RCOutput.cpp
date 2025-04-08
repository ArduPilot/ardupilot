/*
  Simple test of RC output interface with Menu
  Attention: If your board has safety switch,
  don't forget to push it to enable the PWM output.
*/

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Menu/AP_Menu.h>

// we need a boardconfig created so that the io processor's enable
// parameter is available
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_IOMCU/AP_IOMCU.h>
AP_BoardConfig BoardConfig;
#endif

void setup();
void loop();
void drive(uint16_t hz_speed);

#define MENU_FUNC(func) FUNCTOR_BIND(&commands, &Menu_Commands::func, int8_t, uint8_t, const Menu::arg *)

#define ESC_HZ     490
#define SERVO_HZ    50

class Menu_Commands {
public:
    /* Menu commands to drive a SERVO type with
     * respective PWM output freq defined by SERVO_HZ
     */
    int8_t menu_servo(uint8_t argc, const Menu::arg *argv);

    /* Menu commands to drive a ESC type with
     * respective PWM output freq defined by ESC_HZ
     */
    int8_t menu_esc(uint8_t argc, const Menu::arg *argv);
};

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

Menu_Commands commands;

static uint16_t pwm = 1500;
static int8_t delta = 1;

/* Function to drive a RC output TYPE specified */
void drive(uint16_t hz_speed) {
    hal.rcout->set_freq(0xFF, hz_speed);

    while (1) {
        for (uint8_t i = 0; i < 14; i++) {
            hal.rcout->write(i, pwm);
            pwm += delta;
            if (delta > 0 && pwm >= 2000) {
                delta = -1;
                hal.console->printf("reversing\n");
            } else if (delta < 0 && pwm <= 1000) {
                delta = 1;
                hal.console->printf("normalizing\n");
            }
        }
        hal.scheduler->delay(5);
        if (hal.console->available()) {
            break;
        }
    }
}

int8_t Menu_Commands::menu_servo(uint8_t argc, const Menu::arg *argv) {
    drive(SERVO_HZ);
    return 0;
}

int8_t Menu_Commands::menu_esc(uint8_t argc, const Menu::arg *argv) {
    drive(ESC_HZ);
    return 0;
}

const struct Menu::command rcoutput_menu_commands[] = {
    { "servo",          MENU_FUNC(menu_servo) },
    { "esc",            MENU_FUNC(menu_esc) },
};

MENU(menu, "Menu: ", rcoutput_menu_commands);

void setup(void) {
    hal.console->printf("Starting AP_HAL::RCOutput test\n");

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    BoardConfig.init();
#endif
    for (uint8_t i = 0; i < 14; i++) {
        hal.rcout->enable_ch(i);
    }
}

void loop(void) {
    /* We call and run the menu, you can type help into menu to show commands
     * available */
    menu.run();
}

AP_HAL_MAIN();
