/*
  simple test of RC output interface
 */

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Menu/AP_Menu.h>

#define MENU_FUNC(func) FUNCTOR_BIND(&commands, &MenuCommands::func, int8_t, uint8_t, const Menu::arg *)

#define ESC_HZ    490
#define SERVO_HZ   50

class MenuCommands {
  public:

    int8_t menu_servo(uint8_t argc, const Menu::arg *argv);
    int8_t menu_esc(uint8_t argc, const Menu::arg *argv);

};

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

MenuCommands commands;

static uint16_t pwm = 1500;
static int8_t delta = 1;

void drive(uint16_t hz_speed)
{
    hal.rcout->set_freq(0xFF, hz_speed);
    while(1){
        uint8_t i;
        for (i=0; i<14; i++) {
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
        if (hal.console->available()){
           break;
        }
    }
}

int8_t MenuCommands::menu_servo(uint8_t argc, const Menu::arg *argv)
{
    drive(SERVO_HZ);
    return 0;
}

int8_t MenuCommands::menu_esc(uint8_t argc, const Menu::arg *argv)
{
    drive(ESC_HZ);
    return 0;
}

const struct Menu::command rcoutput_menu_commands[] = {
    {"servo",          MENU_FUNC(menu_servo)},
    {"esc",              MENU_FUNC(menu_esc)},
};

MENU(menu, "Menu: ", rcoutput_menu_commands);

void setup (void) 
{

    hal.console->println("Starting AP_HAL::RCOutput test");
    for (uint8_t i=0; i<14; i++) {
        hal.rcout->enable_ch(i);
    }
    
}

void loop (void) 
{
    menu.run();
}

AP_HAL_MAIN();
