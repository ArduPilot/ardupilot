/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

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
#if CONFIG_HAL_BOARD == HAL_BOARD_PX4

#include "PixRacerLED.h"
#include <AP_GPS/AP_GPS.h>
#include "AP_Notify.h"

extern const AP_HAL::HAL& hal;

bool PixRacerLED::init(void)
{

    // turn all lights off
    hal.gpio->write(HAL_GPIO_A_LED_PIN, LED_OFF);
    hal.gpio->write(HAL_GPIO_B_LED_PIN, LED_OFF);
    hal.gpio->write(HAL_GPIO_C_LED_PIN, LED_OFF);
    return true;
}

/*
  main update function called at 50Hz
 */
void PixRacerLED::update(void)
{
    _counter++;

    // we never want to update LEDs at a higher than 16Hz rate
    if (_counter < 5) {
        return;
    }
    _counter = 0;

    // move forward one step
    step++;
    if (step >= 10) {
        step = 0;
    }

    // initialising
    if (AP_Notify::flags.initialising) {
    	// do red blue at startup
    	if (step & 1) {
            hal.gpio->write(HAL_GPIO_A_LED_PIN, LED_ON);
            hal.gpio->write(HAL_GPIO_B_LED_PIN, LED_OFF);
            hal.gpio->write(HAL_GPIO_C_LED_PIN, LED_OFF);
    	} else {
            hal.gpio->write(HAL_GPIO_A_LED_PIN, LED_OFF);
            hal.gpio->write(HAL_GPIO_B_LED_PIN, LED_OFF);
            hal.gpio->write(HAL_GPIO_C_LED_PIN, LED_ON);
    	}
        return;
	}

    // save trim and ESC calibration
    if ((AP_Notify::flags.save_trim) || (AP_Notify::flags.esc_calibration)) {
        switch (step) {
        case 0:
        case 3:
        case 6:
        	// red on
            hal.gpio->write(HAL_GPIO_A_LED_PIN, LED_ON);
            hal.gpio->write(HAL_GPIO_B_LED_PIN, LED_OFF);
            hal.gpio->write(HAL_GPIO_C_LED_PIN, LED_OFF);
            break;
        case 1:
        case 4:
        case 7:
            // blue on
            hal.gpio->write(HAL_GPIO_A_LED_PIN, LED_OFF);
            hal.gpio->write(HAL_GPIO_B_LED_PIN, LED_OFF);
            hal.gpio->write(HAL_GPIO_C_LED_PIN, LED_ON);
            break;
        case 2:
        case 5:
        case 8:
        	// green on
            hal.gpio->write(HAL_GPIO_A_LED_PIN, LED_OFF);
            hal.gpio->write(HAL_GPIO_B_LED_PIN, LED_ON);
            hal.gpio->write(HAL_GPIO_C_LED_PIN, LED_OFF);
            break;
        case 9:
            // all off
            hal.gpio->write(HAL_GPIO_A_LED_PIN, LED_OFF);
            hal.gpio->write(HAL_GPIO_B_LED_PIN, LED_OFF);
            hal.gpio->write(HAL_GPIO_C_LED_PIN, LED_OFF);
        }
        return;
    }

    // radio and battery failsafe patter: flash yellow
    // gps failsafe pattern : flashing yellow and blue
    // ekf_bad pattern : flashing yellow and red
    if (AP_Notify::flags.failsafe_radio || AP_Notify::flags.failsafe_battery ||
            AP_Notify::flags.ekf_bad) {
        switch(step) {
            case 0:
            case 1:
            case 2:
            case 3:
            case 4:
                // yellow on
                hal.gpio->write(HAL_GPIO_A_LED_PIN, LED_ON);
                hal.gpio->write(HAL_GPIO_B_LED_PIN, LED_ON);
                hal.gpio->write(HAL_GPIO_C_LED_PIN, LED_OFF);
                break;
            case 5:
            case 6:
            case 7:
            case 8:
            case 9:
                if (AP_Notify::flags.ekf_bad) {
                    // red on if ekf bad
                    hal.gpio->write(HAL_GPIO_A_LED_PIN, LED_ON);
                    hal.gpio->write(HAL_GPIO_B_LED_PIN, LED_OFF);
                    hal.gpio->write(HAL_GPIO_C_LED_PIN, LED_OFF);
                }else{
                    // all off for radio or battery failsafe
                    hal.gpio->write(HAL_GPIO_A_LED_PIN, LED_OFF);
                    hal.gpio->write(HAL_GPIO_B_LED_PIN, LED_OFF);
                    hal.gpio->write(HAL_GPIO_C_LED_PIN, LED_OFF);
                }
                break;
        }
        // exit so no other status modify this pattern
        return;
    }

    // solid green or blue if armed
    if (AP_Notify::flags.armed) {
        // solid green if armed with GPS 3d lock
        if (AP_Notify::flags.gps_status >= AP_GPS::GPS_OK_FIX_3D) {
            hal.gpio->write(HAL_GPIO_A_LED_PIN, LED_OFF);
            hal.gpio->write(HAL_GPIO_B_LED_PIN, LED_ON);
            hal.gpio->write(HAL_GPIO_C_LED_PIN, LED_OFF);
        }else{
            // solid blue if armed with no GPS lock
            hal.gpio->write(HAL_GPIO_A_LED_PIN, LED_OFF);
            hal.gpio->write(HAL_GPIO_B_LED_PIN, LED_OFF);
            hal.gpio->write(HAL_GPIO_C_LED_PIN, LED_ON);
        }
        return;
    }else{
        // double flash yellow if failing pre-arm checks
        if (!AP_Notify::flags.pre_arm_check) {
            switch(step) {
                case 0:
                case 1:
                case 4:
                case 5:
                    // yellow on
                    hal.gpio->write(HAL_GPIO_A_LED_PIN, LED_ON);
                    hal.gpio->write(HAL_GPIO_B_LED_PIN, LED_ON);
                    hal.gpio->write(HAL_GPIO_C_LED_PIN, LED_OFF);
                    break;
                case 2:
                case 3:
                case 6:
                case 7:
                case 8:
                case 9:
                    // all off
                    hal.gpio->write(HAL_GPIO_A_LED_PIN, LED_OFF);
                    hal.gpio->write(HAL_GPIO_B_LED_PIN, LED_OFF);
                    hal.gpio->write(HAL_GPIO_C_LED_PIN, LED_OFF);
                    break;
            }
        }else{
            // fast flashing green if disarmed with GPS 3D lock and DGPS
            // slow flashing green if disarmed with GPS 3d lock (and no DGPS)
            // flashing blue if disarmed with no gps lock or gps pre_arm checks have failed
            bool fast_green = AP_Notify::flags.gps_status >= AP_GPS::GPS_OK_FIX_3D_DGPS && AP_Notify::flags.pre_arm_gps_check;
            switch(step) {
                case 0:
                    if (fast_green) {
                        hal.gpio->write(HAL_GPIO_A_LED_PIN, LED_OFF);
                        hal.gpio->write(HAL_GPIO_B_LED_PIN, LED_ON);
                        hal.gpio->write(HAL_GPIO_C_LED_PIN, LED_OFF);
                    }
                    break;
                case 1:
                    if (fast_green) {
                        hal.gpio->write(HAL_GPIO_A_LED_PIN, LED_OFF);
                        hal.gpio->write(HAL_GPIO_B_LED_PIN, LED_OFF);
                        hal.gpio->write(HAL_GPIO_C_LED_PIN, LED_OFF);
                    }
                    break;
                case 2:
                    if (fast_green) {
                        hal.gpio->write(HAL_GPIO_A_LED_PIN, LED_OFF);
                        hal.gpio->write(HAL_GPIO_B_LED_PIN, LED_ON);
                        hal.gpio->write(HAL_GPIO_C_LED_PIN, LED_OFF);
                    }
                    break;
                case 3:
                    if (fast_green) {
                        hal.gpio->write(HAL_GPIO_A_LED_PIN, LED_OFF);
                        hal.gpio->write(HAL_GPIO_B_LED_PIN, LED_OFF);
                        hal.gpio->write(HAL_GPIO_C_LED_PIN, LED_OFF);
                    }
                    break;
                case 4:
                    hal.gpio->write(HAL_GPIO_A_LED_PIN, LED_OFF);
                    if (AP_Notify::flags.gps_status >= AP_GPS::GPS_OK_FIX_3D && AP_Notify::flags.pre_arm_gps_check) {
                        // flashing green if disarmed with GPS 3d lock
                        hal.gpio->write(HAL_GPIO_B_LED_PIN, LED_ON);
                        hal.gpio->write(HAL_GPIO_C_LED_PIN, LED_OFF);
                    }else{
                        // flashing blue if disarmed with no gps lock
                        hal.gpio->write(HAL_GPIO_B_LED_PIN, LED_OFF);
                        hal.gpio->write(HAL_GPIO_C_LED_PIN, LED_ON);
                    }
                    break;
                case 5:
                    if (fast_green) {
                        hal.gpio->write(HAL_GPIO_B_LED_PIN, LED_OFF);
                        hal.gpio->write(HAL_GPIO_C_LED_PIN, LED_OFF);
                    }
                    break;

                case 6:
                    if (fast_green) {
                        hal.gpio->write(HAL_GPIO_B_LED_PIN, LED_ON);
                        hal.gpio->write(HAL_GPIO_C_LED_PIN, LED_OFF);
                    }
                    break;

                case 7:
                    if (fast_green) {
                        hal.gpio->write(HAL_GPIO_B_LED_PIN, LED_OFF);
                        hal.gpio->write(HAL_GPIO_C_LED_PIN, LED_OFF);
                    }
                    break;
                case 8:
                    if (fast_green) {
                        hal.gpio->write(HAL_GPIO_B_LED_PIN, LED_ON);
                        hal.gpio->write(HAL_GPIO_C_LED_PIN, LED_OFF);
                    }
                    break;
                case 9:
                    // all off
                    hal.gpio->write(HAL_GPIO_A_LED_PIN, LED_OFF);
                    hal.gpio->write(HAL_GPIO_B_LED_PIN, LED_OFF);
                    hal.gpio->write(HAL_GPIO_C_LED_PIN, LED_OFF);
                    break;
            }
        }
    }
}

#endif // CONFIG_HAL_BOARD
