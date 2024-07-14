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


   show all status on only 2 leds
 */

#include "AP_Notify_config.h"

#if AP_NOTIFY_GPIO_LED_2_ENABLED

#include "AP_BoardLED2.h"

#include "AP_Notify.h"

static_assert((HAL_GPIO_A_LED_PIN != HAL_GPIO_B_LED_PIN), "Duplicate LED assignments detected");

extern const AP_HAL::HAL& hal;

bool AP_BoardLED2::init(void)
{
    // setup the main LEDs as outputs
    hal.gpio->pinMode(HAL_GPIO_A_LED_PIN, HAL_GPIO_OUTPUT);
    hal.gpio->pinMode(HAL_GPIO_B_LED_PIN, HAL_GPIO_OUTPUT);

    // turn all lights off
    hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_OFF);
    hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_OFF);
    
    _sat_cnt=0;
    save_trim_counter = 0;
    arm_counter = 0;
    return true;
}

/*
  main update function called at 50Hz
 */
void AP_BoardLED2::update(void)
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
        // blink LEDs A at 8Hz (full cycle) during initialisation
        hal.gpio->write(HAL_GPIO_A_LED_PIN, (counter2 & 1) ? HAL_GPIO_LED_ON : HAL_GPIO_LED_OFF);
        return;
    }

    if ((counter2 & 0x2) == 0) {
        save_trim_counter++;
    }

    bool led_a_used=false;

    // save trim and ESC calibration
    if (AP_Notify::flags.save_trim || AP_Notify::flags.esc_calibration) {
        switch(save_trim_counter) {
            case 0:
                hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_OFF);
                hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON);
                break;

            case 1:
                hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_OFF);
                hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_ON);
                break;

            default:
                save_trim_counter = -1;
                break;
        }
        return;
    }

    if(AP_Notify::flags.compass_cal_running ||
       AP_Notify::flags.temp_cal_running){
        // compass calibration or IMU temperature calibration
        switch(save_trim_counter) {
        case 0:
            hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON); // short blinks by both LEDs
            hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_ON);
            break;
        case 1:
        case 2:
            hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_OFF);
            hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_OFF);
            break;
        case 3:
        case 4:
        case 5:
        case 6:
        case 7:
            break;
        default:
            // reset counter to restart the sequence
            save_trim_counter = -1;
            break;
        }
        return; 
        
    }

    if(AP_Notify::events.autotune_complete){
        switch(save_trim_counter) {
        case 0:
            hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON); // short darkening
            break;
        case 1:
        case 2:
        case 3:
        case 4:
        case 5:
        case 6:
            hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_OFF);
            break;
        case 7:
            break;
        default:
            // reset counter to restart the sequence
            save_trim_counter = -1;
            break;
        }
        led_a_used=true;
    }

    if(AP_Notify::events.autotune_failed){
        switch(save_trim_counter) {
        case 0:
            hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON); // short double darkening
            break;
        case 1:
            hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_OFF);
            break;
        case 2:
        case 3:
            hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON); 
            break;
        case 4:
        case 5:
        case 6:
        case 7:
            hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_OFF);
            break;
        default:
            // reset counter to restart the sequence
            save_trim_counter = -1;
            break;
        }
        led_a_used=true;
    }

    // arming light
    if(!led_a_used) {
        if (AP_Notify::flags.armed) {
            if(AP_Notify::flags.failsafe_battery){//   blink slowly (around 2Hz)
                if ((counter2 & 0x7) == 0) {
                    hal.gpio->toggle(HAL_GPIO_A_LED_PIN);
                }
            }else if(AP_Notify::flags.failsafe_radio || AP_Notify::flags.failsafe_gcs){//   blink fast (around 4Hz)
                if ((counter2 & 0x3) == 0) {
                    hal.gpio->toggle(HAL_GPIO_A_LED_PIN);
                }
            } else {
                // ARM led solid
                hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON);
            }
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
                    case 3:
                        hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON);
                        break;
                    case 4:
                    case 5:
                    case 6:
                    case 7:
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
                        hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON);
                        break;
    
                    case 2:
                        hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_OFF);
                        break;
                    
                    
                    case 3:
                    case 4:
                        hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON);
                        break;
    
                    case 5:
                    case 6:
                    case 7: // add one tick to do period be a multiple of the second
                        hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_OFF);
                        break;
    
                    default:
                        arm_counter = -1;
                        break;
                }
            }
        }
    }
    // gps light
    switch (AP_Notify::flags.gps_status) {
        case 0:
        case 1:
            // no GPS attached or no lock - be dark
            hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_OFF);
            break;

        case 2: // 2d lock
        case 3://  3d lock
        default: // show number of sats 
            if ((counter2 & 0x2) == 0) {
                _sat_cnt++;
            }
            uint16_t sats = AP_Notify::flags.gps_num_sats;
    
            if(_sat_cnt<8) { // pause between pulses
                hal.gpio->write(HAL_GPIO_B_LED_PIN, HAL_GPIO_LED_OFF);
            } else if(_sat_cnt< (8 + (sats-6)*2) ) {
                hal.gpio->toggle(HAL_GPIO_B_LED_PIN); // 2Hz
            } else {
                _sat_cnt=-1;
            }            
            break;        
    }
}

#endif  // AP_NOTIFY_GPIO_LED_2_ENABLED
