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
#include "ExternalLED.h"

#include "AP_Notify.h"

#include <AP_HAL/AP_HAL.h>

#if (defined(EXTERNAL_LED_ARMED) && defined(EXTERNAL_LED_GPS) && \
    defined(EXTERNAL_LED_MOTOR1) && defined(EXTERNAL_LED_MOTOR2))

extern const AP_HAL::HAL& hal;

bool ExternalLED::init(void)
{
    // setup the main LEDs as outputs
    hal.gpio->pinMode(EXTERNAL_LED_ARMED, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(EXTERNAL_LED_GPS, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(EXTERNAL_LED_MOTOR1, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(EXTERNAL_LED_MOTOR2, HAL_GPIO_OUTPUT);

    // turn leds off
    hal.gpio->write(EXTERNAL_LED_ARMED, HAL_GPIO_LED_OFF);
    hal.gpio->write(EXTERNAL_LED_GPS, HAL_GPIO_LED_OFF);
    hal.gpio->write(EXTERNAL_LED_MOTOR1, HAL_GPIO_LED_OFF);
    hal.gpio->write(EXTERNAL_LED_MOTOR2, HAL_GPIO_LED_OFF);
    return true;
}

/*
  main update function called at 50Hz
 */
void ExternalLED::update(void)
{
    // reduce update rate from 50hz to 10hz
    _counter++;
    if (_counter < 5) {
        return;
    }
    _counter = 0;

    // internal counter used to control step of armed and gps led
    _counter2++;
    if (_counter2 >= 10) {
        _counter2 = 0;
    }

    // initialising
    if (AP_Notify::flags.initialising) {
        // blink arming and gps leds at 5hz
        switch(_counter2) {
            case 0:
            case 2:
            case 4:
            case 6:
            case 8:
                armed_led(true);
                gps_led(false);
                break;
            case 1:
            case 3:
            case 5:
            case 7:
            case 9:
                armed_led(false);
                gps_led(true);
                break;
        }
        return;
    }

    // arming led control
    if (AP_Notify::flags.armed) {
        armed_led(true);
    }else{
        // blink arming led at 2hz
        switch(_counter2) {
            case 0:
            case 1:
            case 2:
            case 5:
            case 6:
            case 7:
                armed_led(false);
                break;
            case 3:
            case 4:
            case 8:
            case 9:
                armed_led(true);
                break;
        }
    }

    // GPS led control
    switch (AP_Notify::flags.gps_status) {
        case 0:
            // no GPS attached
            gps_led(false);
            break;
        case 1:
        case 2:
            // GPS attached but no lock, blink at 4Hz
            switch(_counter2) {                             // Pattern: 3(off), 2(on), 3(off), 2(on), repeat
                case 0:
                case 1:
                case 2:
                case 5:
                case 6:
                case 7:
                    gps_led(false);
                    break;
                case 3:
                case 4:
                case 8:
                case 9:
                    gps_led(true);
                    break;
            }
            break;
        case 3:
            // solid blue on gps lock
            gps_led(true);
            break;
    }

    // motor led control
    // if we are displaying a pattern complete it
    if (_pattern != NONE) {
        _pattern_counter++;
        switch(_pattern) {
            case NONE:
                // do nothing
                break;
            case FAST_FLASH:
                switch(_pattern_counter) {
                    case 1:
                    case 3:
                    case 5:
                    case 7:
                    case 9:
                        motor_led1(true);
                        motor_led2(true);
                        break;
                    case 2:
                    case 4:
                    case 6:
                    case 8:
                        motor_led1(false);
                        motor_led2(false);
                        break;
                    case 10:
                        motor_led1(false);
                        motor_led2(false);
                        set_pattern(NONE);
                        break;
                }
                break;
            case OSCILLATE:
                switch(_pattern_counter) {
                    case 1:
                        motor_led1(true);
                        motor_led2(false);
                        break;
                    case 4:
                        motor_led1(false);
                        motor_led2(true);
                        break;
                    case 6:
                        set_pattern(NONE);
                        break;
                }
                break;
        }
    }else{
        if (AP_Notify::flags.failsafe_battery || AP_Notify::flags.failsafe_radio || AP_Notify::flags.failsafe_gcs) {
            // radio or battery failsafe indicated by fast flashing
            set_pattern(FAST_FLASH);
        } else {
            // otherwise do whatever the armed led is doing
            motor_led1(_flags.armedled_on);
            motor_led2(_flags.armedled_on);
        }
    }
}

// set_pattern - sets pattern for motor leds
void ExternalLED::set_pattern(LEDPattern pattern_id)
{
    _pattern = pattern_id;
    _pattern_counter = 0;
}

// armed_led - set armed light on or off
void ExternalLED::armed_led(bool on_off)
{
    if (_flags.armedled_on != on_off) {
        _flags.armedled_on = on_off;
        hal.gpio->write(EXTERNAL_LED_ARMED, _flags.armedled_on);
    }
}

// gps_led - set gps light on or off
void ExternalLED::gps_led(bool on_off)
{
    if (_flags.gpsled_on != on_off) {
        _flags.gpsled_on = on_off;
        hal.gpio->write(EXTERNAL_LED_GPS, _flags.gpsled_on);
    }
}

// motor_led - set motor light on or off
void ExternalLED::motor_led1(bool on_off)
{
    if (_flags.motorled1_on != on_off) {
        _flags.motorled1_on = on_off;
        hal.gpio->write(EXTERNAL_LED_MOTOR1, _flags.motorled1_on);
    }
}

// motor_led - set motor light on or off
void ExternalLED::motor_led2(bool on_off)
{
    if (_flags.motorled2_on != on_off) {
        _flags.motorled2_on = on_off;
        hal.gpio->write(EXTERNAL_LED_MOTOR2, _flags.motorled2_on);
    }
}
#else
bool ExternalLED::init(void) {return true;}
void ExternalLED::update(void) {return;}
#endif
