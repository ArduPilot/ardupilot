/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include <AP_BoardLED.h>

extern const AP_HAL::HAL& hal;

// static private variable instantiation
uint16_t AP_BoardLED::_counter;

void AP_BoardLED::init(void)
{
    // update LEDs as often as needed
    hal.scheduler->register_timer_process( AP_BoardLED::_update );	

    // setup the main LEDs as outputs
    hal.gpio->pinMode(HAL_GPIO_A_LED_PIN, GPIO_OUTPUT);
    hal.gpio->pinMode(HAL_GPIO_B_LED_PIN, GPIO_OUTPUT);
    hal.gpio->pinMode(HAL_GPIO_C_LED_PIN, GPIO_OUTPUT);
}

void AP_BoardLED::_update(uint32_t now)
{
    _counter++;

    // we never want to update LEDs at a higher than 16Hz rate
    if (_counter & 0x3F) {
        return;
    }

    // counter2 used to drop frequency down to 16hz
    uint8_t counter2 = _counter >> 6;

    // initialising
    if (notify.flags.initialising) {
        // blink LEDs A and C at 8Hz (full cycle) during initialisation
        if (counter2 & 1) {
            hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON);
            hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_OFF);
        } else {
            hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_OFF);
            hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_ON);
        }
        return;
	}

    // save trim
    if (notify.flags.save_trim) {
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
        }
        return;
    }

    // arming light
    static uint8_t arm_counter = 0;
	if (notify.flags.armed) {
        // red led solid
        hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_ON);
    }else{
        if ((counter2 & 0x2) == 0) {
            arm_counter++;
        }
        if (notify.flags.pre_arm_check) {
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
            // disarmed and passing pre-arm checks, blink at about 2hz
            //if ((counter2 & 0x7) == 0) {
            //    hal.gpio->toggle(HAL_GPIO_A_LED_PIN);
            //}
        }else{
            // disarmed and failing pre-arm checks, double blink
            //if (counter2 & 0x4) {
            //    hal.gpio->toggle(HAL_GPIO_A_LED_PIN);
            //}
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
                    hal.gpio->write(HAL_GPIO_A_LED_PIN, HAL_GPIO_LED_OFF);
                    break;
                default:
                    arm_counter = -1;
                    break;
            }
        }
    }

    // gps light
    switch (notify.flags.gps_status) {
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

        case 3:
            // GPS attached but 2D lock, blink more slowly (around 2Hz)
            hal.gpio->write(HAL_GPIO_C_LED_PIN, HAL_GPIO_LED_ON);
            break;        
    }
}
