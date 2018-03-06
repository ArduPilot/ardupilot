/*
  JoyPad init example:
  Hiroshi Takey, December 2017.
*/

#include <AP_HAL/AP_HAL.h>
#include <AP_Joypad/AP_Joypad.h>

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

AP_Joypad joypad;

void setup()
{
    joypad.init();
}

void loop()
{
}

AP_HAL_MAIN();
