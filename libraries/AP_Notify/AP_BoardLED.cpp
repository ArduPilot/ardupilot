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
#include "AP_BoardLED.h"

#include "AP_Notify.h"

#if (defined(HAL_GPIO_A_LED_PIN) && defined(HAL_GPIO_B_LED_PIN) && \
     defined(HAL_GPIO_C_LED_PIN))

extern const AP_HAL::HAL& hal;

bool AP_BoardLED::init(void)
{
    // setup the main LEDs as outputs
    hal.gpio->pinMode(HAL_GPIO_A_LED_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(HAL_GPIO_B_LED_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(HAL_GPIO_C_LED_PIN, HAL_GPIO_OUTPUT);

    // turn all lights off
    hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_OFF);
    hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_OFF);
    hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_OFF);
    return true;
}

/*
  main update function called at 50Hz
 */
void AP_BoardLED::update(void)
{
    _counter++;

    // we never want to update LEDs at a higher than 16Hz rate
    if (_counter % 3 != 0) {
        return;
    }

    // counter2 used to drop frequency down to 16hz
    uint8_t counter2 = _counter / 3;

    // initialising
    if (AP_Notify::flags.initialising) {
        // blink LEDs A and C at 8Hz (full cycle) during initialisation
        hal.gpio->write(HAL_GPIO_A_LED_PIN, (counter2 & 1) ? HAL_GPIO_LED_ON : HAL_GPIO_LED_OFF);
        hal.gpio->write(HAL_GPIO_C_LED_PIN, (counter2 & 1) ? HAL_GPIO_LED_OFF : HAL_GPIO_LED_ON);
        return;
	}

    // save trim and ESC calibration
    if (AP_Notify::flags.save_trim || AP_Notify::flags.esc_calibration) {
        static uint8_t save_trim_counter = 0;
        if ((counter2 & 0x2) == 0) {
            save_trim_counter++;
        }
        switch(save_trim_counter) {
            case 0:
                hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_OFF);
                hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON);
                break;

            case 1:
                hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_OFF);
                hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_ON);
                break;

            case 2:
                hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_OFF);
                hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_ON);
                break;

            default:
                save_trim_counter = -1;
                break;
        }
        return;
    }

    // arming light
    static uint8_t arm_counter = 0;
	if (AP_Notify::flags.armed) {
        // red led solid
        hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON);
    }else{
        if ((counter2 & 0x2) == 0) {
            arm_counter++;
        }
        if (AP_Notify::flags.pre_arm_check) {
            // passed pre-arm checks so slower single flash
            switch(arm_counter) {
                case 0:
                case 1:
                case 2:
                    hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON);
                    break;
                case 3:
                case 4:
                case 5:
                    hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_OFF);
                    break;
                default:
                    // reset counter to restart the sequence
                    arm_counter = -1;
                    break;
            }
        }else{
            // failed pre-arm checks so double flash
            switch(arm_counter) {
                case 0:
                case 1:
                case 3:
                case 4:
                    hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON);
                    break;

                case 2:
                case 5:
                case 6:
                    hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_OFF);
                    break;

                default:
                    arm_counter = -1;
                    break;
            }
        }
    }

    // gps light
    switch (AP_Notify::flags.gps_status) {
        case 0:
            // no GPS attached
            hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_OFF);
            break;

        case 1:
            // GPS attached but no lock, blink at 4Hz
            if ((counter2 & 0x3) == 0) {
                hal.gpio->toggle(HAL_GPIO_C_LED_PIN);
            }
            break;

        case 2:
            // GPS attached but 2D lock, blink more slowly (around 2Hz)
            if ((counter2 & 0x7) == 0) {
                hal.gpio->toggle(HAL_GPIO_C_LED_PIN);
            }
            break;

        default:
            // solid blue on gps lock
            hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_ON);
            break;        
    }
}
#else
bool AP_BoardLED::init(void) {return true;}
void AP_BoardLED::update(void) {return;}
#endif
