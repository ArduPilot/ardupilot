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
/*
  battery BMS includes a button which, when pressed, shows the state of charge percentage using LEDs
 */
#include "AP_Periph.h"

#if AP_PERIPH_BATTERY_BMS_ENABLED
#include "stdio.h"
#include "battery_bms.h"

extern const AP_HAL::HAL &hal;
extern AP_Periph_FW periph;

// GPIO pins used for BMS LEDs
const uint8_t BatteryBMS::led_gpios[] = {AP_PERIPH_BMS_LED_PINS};

// configure gpio pins. returns true once configured
bool BatteryBMS::configured()
{
    if (config_complete) {
        return true;
    }

    // when HAL_GPIO_LED_ON is 0 then we must not use pinMode()
    // as it could remove the OPENDRAIN attribute on the pin
    // configure LED pins as outputs
#if HAL_GPIO_LED_ON != 0
    for (uint8_t i = 0; i < ARRAY_SIZE(led_gpios); i++) {
        hal.gpio->pinMode(led_gpios[i], HAL_GPIO_OUTPUT);
    }
#endif

    // configure button as input with pullup
#ifdef HAL_GPIO_PIN_BMS_BTN1
    hal.gpio->pinMode(HAL_GPIO_PIN_BMS_BTN1, HAL_GPIO_INPUT);
    hal.gpio->write(HAL_GPIO_PIN_BMS_BTN1, 1);
#endif

    // mark configuration as complete
    config_complete = true;
    return true;
}

// main update function
void BatteryBMS::update(void)
{
    // exit immediately if no batteries are enabled
    if (periph.battery_lib.num_instances() == 0) {
        return;
    }

    // configure
    if (!configured()) {
        return;
    }

#ifdef HAL_GPIO_PIN_BMS_BTN1
    // check and handle button press events
    handle_button_press();
#endif

    // update BMS state machine
    update_bms_state();

    // update LED state machine
    update_led_state();
}

// check and handle button press events
void BatteryBMS::handle_button_press(void)
{
    // we take action in these cases
    //   1. button is released after being pressed between 10ms an 1 sec, we display the battery percentage (short press)
    //   2. button is pressed for at least 1 second (long press), we start the power-up or power-down counter and animation
    //   3. button is released after case 2 but before the counter and animation has completed, we cancel the power-up or power-down action
    //   4. button is released after the power-up or power-down counter and animation have completed, no further action is taken

    // ignore button presses during startup to avoid false detections from GPIO initialization
    uint32_t now_ms = AP_HAL::millis();
    if (!button.startup_complete && (now_ms < BUTTON_STARTUP_DELAY_MS)) {
        return;
    }
    button.startup_complete = true;

    // read current button state (assuming active low - pressed = 0)
    bool button_pressed = (hal.gpio->read(HAL_GPIO_PIN_BMS_BTN1) == 0);

    // check if button has just been pressed (transition from released to pressed)
    if (button_pressed && !button.pressed_prev) {
        // record time button was pressed
        button.pressed_start_ms = now_ms;

        // record button pressed state before returning
        button.pressed_prev = button_pressed;
        button.long_press_handled = false;
        return;
    }

    // calculate how long the button was pressed including when button has just been released
    const uint32_t press_duration_ms = (button_pressed || button.pressed_prev) ? (now_ms - button.pressed_start_ms) : 0;
    const bool short_press = (press_duration_ms > BUTTON_SHORT_PRESS_THRESHOLD_MS) && (press_duration_ms < BUTTON_LONG_PRESS_THRESHOLD_MS);
    const bool long_press = (press_duration_ms >= BUTTON_LONG_PRESS_THRESHOLD_MS);

    // check if button has just been released (transition from pressed to released)
    bool button_released = false;
    if (!button_pressed && button.pressed_prev) {
        button.pressed_start_ms = 0;
        button_released = true;
    }

    // update button state for next iteration
    button.pressed_prev = button_pressed;

    // handle release after short press
    // displays battery SOC percentage on LEDs and prints voltages to the console
    if (short_press && button_released) {
        request_display_percentage();
        return;
    }

    // handle long press event
    // starts power on/off sequence
    if (long_press && !button.long_press_handled) {

        // ensure we only handle long press once
        button.long_press_handled = true;

        switch (bms_state) {
        case BmsState::IDLE:
        case BmsState::POWERING_OFF:
        case BmsState::POWERED_OFF:
            // if battery is off, request power on
            request_bms_state(BmsState::POWERED_ON);
            break;
        case BmsState::POWERING_ON:
        case BmsState::POWERED_ON:
            // if battery if on, request power off
            request_bms_state(BmsState::POWERED_OFF);
            break;
        }
        return;
    }

    // handle release after long press
    if (long_press && button_released) {
        // if the BMS is transitioning to powered on or off, abort the transition
        switch (bms_state) {
        case BmsState::POWERING_ON:
            // request power off
            request_bms_state(BmsState::POWERED_OFF);
            break;
        case BmsState::POWERING_OFF:
            request_bms_state(BmsState::POWERED_ON);
            break;
        case BmsState::IDLE:
        case BmsState::POWERED_ON:
        case BmsState::POWERED_OFF:
            // BMS/battery have already completed the power on/off action -- do nothing
            break;
        }
        return;
    }
}

// request display of battery percentage using LEDs
void BatteryBMS::request_display_percentage()
{
    led_display_soc_start_ms = AP_HAL::millis();
}

// display battery SOC percentage using LEDs
void BatteryBMS::display_percentage()
{
    // get battery percentage
    uint8_t batt_soc_pct;
    if (!get_percentage(batt_soc_pct)) {
        return;
    }

    // calculate how many LEDs to light up based on battery percentage
    // uses ceiling division to round up: 0% = 0 LEDs, 1-12% = 1 LED, etc
    const uint8_t num_leds = ARRAY_SIZE(led_gpios);
    const uint8_t num_leds_on = MIN(num_leds, (batt_soc_pct * num_leds + 99) / 100);

    // build bitmask for LEDs (e.g., num_leds=3 gives 0b00000111)
    const uint8_t pattern = (1U << num_leds_on) - 1;

    // set the LED pattern and start display timer
    set_led_pattern(pattern);
}

// get battery percentage (0-100). returns true on success
bool BatteryBMS::get_percentage(uint8_t &percentage)
{
    percentage = 0;

    // try to get capacity remaining percentage first
    if (periph.battery_lib.capacity_remaining_pct(percentage, 0)) {
        return true;
    }

    // fallback: calculate percentage from average cell voltage
    // Li-ion/LiPo typical range: 3.0V (0%) to 4.2V (100%)
    if (!periph.battery_lib.has_cell_voltages()) {
        return false;
    }
    const AP_BattMonitor::cells &cell_voltages = periph.battery_lib.get_cell_voltages();
    uint32_t total_voltage_mv = 0;
    uint8_t cell_count = 0;

    for (uint8_t i = 0; i < ARRAY_SIZE(cell_voltages.cells); i++) {
        if (cell_voltages.cells[i] == UINT16_MAX) {
            break;
        }
        total_voltage_mv += cell_voltages.cells[i];
        cell_count++;
    }

    if (cell_count > 0) {
        uint16_t avg_cell_voltage_mv = total_voltage_mv / cell_count;
        // map 3000mV-4200mV to 0-100%
        if (avg_cell_voltage_mv <= 3000) {
            percentage = 0;
        } else if (avg_cell_voltage_mv >= 4200) {
            percentage = 100;
        } else {
            percentage = constrain_uint16((avg_cell_voltage_mv - 3000) * 100 / 1200, 0, 100);
        }
        return true;
    }

    return false;
}

// set LED pattern based on 8-bit bitmask
void BatteryBMS::set_led_pattern(uint8_t pattern)
{
    // configure LED pins as outputs
    for (uint8_t i = 0; i < ARRAY_SIZE(led_gpios); i++) {
        hal.gpio->write(led_gpios[i], BIT_IS_SET(pattern, i) ? HAL_GPIO_LED_ON : HAL_GPIO_LED_OFF);
    }
}

// LED state machine - manages LED display based on battery state
void BatteryBMS::update_led_state(void)
{
    // slow down LED updates
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - led_last_update_ms < LED_UPDATE_INTERVAL_MS) {
        return;
    }
    led_last_update_ms = now_ms;

    // display state-of-charge (SOC) percentage
    if (led_display_soc_start_ms > 0) {
        // display SOC percentage
        display_percentage();

        // turn off SOC display after 1 second
        if (now_ms - led_display_soc_start_ms >= LED_DISPLAY_SOC_DURATION_MS) {
            led_display_soc_start_ms = 0;
        }
        return;
    }

    // handle POWERING_ON / OFF transition
    if (bms_state == BmsState::POWERING_ON || bms_state == BmsState::POWERING_OFF) {
        // turn on LEDs sequentially
        set_led_pattern((1 << bms_transition_counter) - 1);
        return;
    }

    // run animation while charging
    auto battery_charging_state = periph.battery_lib.get_charging_state();
    if (battery_charging_state == AP_BattMonitor::ChargingState::CHARGING) {
        led_charging_animation_step = (led_charging_animation_step + 1) % 8;

        // charging: chase forward (bit 0 -> 7)
        uint8_t pattern = 1 << led_charging_animation_step;
        set_led_pattern(pattern);
        return;
    }

    // handle POWERED_ON state - display SOC percentage
    if (bms_state == BmsState::POWERED_ON) {
        display_percentage();
        return;
    }

    // if we get this far, turn all LEDs off
    set_led_pattern(0x00);
}

// set bms state. the only valid inputs are POWERED_ON and POWERING_OFF
// return true on success
bool BatteryBMS::request_bms_state(BmsState new_state)
{
    // sanity check inputs
    if (new_state != BmsState::POWERED_ON && new_state != BmsState::POWERED_OFF) {
        return false;
    }

    // if request is already in progress then return true
    if (req_bms_state == new_state) {
        return true;
    }

    // handle request to power on the battery
    if (new_state == BmsState::POWERED_ON) {
        // allow cancelling a power-off transition
        if (bms_state == BmsState::POWERING_OFF) {
            req_bms_state = new_state;
            return true;
        }
        // allow power on when battery is in IDLE state
        if (periph.battery_lib.get_charging_state() == AP_BattMonitor::ChargingState::IDLE) {
            req_bms_state = new_state;
            return true;
        }
        return false;
    }

    // always accept request to power off the battery
    req_bms_state = new_state;
    return true;
}

// update bms state.  transitions bms_state to req_bms_state
void BatteryBMS::update_bms_state()
{
    // return immediately if current state matches requested state or requested state is IDLE
    if (bms_state == req_bms_state || req_bms_state == BmsState::IDLE) {
        return;
    }

    // rate limit state transitions to match LED update interval (~1.2 seconds for full animation)
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - bms_last_update_ms < LED_UPDATE_INTERVAL_MS) {
        return;
    }
    bms_last_update_ms = now_ms;

    // handle request to power on
    if (req_bms_state == BmsState::POWERED_ON) {
        switch (bms_state) {
        case BmsState::IDLE:
        case BmsState::POWERING_OFF:
        case BmsState::POWERED_OFF:
            // move to powering on
            bms_state = BmsState::POWERING_ON;
            bms_transition_counter = 0;
            break;
        case BmsState::POWERING_ON:
            bms_transition_counter++;
            if (bms_transition_counter >= 8) {
                bms_state = BmsState::POWERED_ON;
                bms_transition_counter = 0;
                periph.battery_lib.set_powered_state(0, true);
            }
            break;
        case BmsState::POWERED_ON:
            // already powered on -- do nothing
            break;
        }
        return;
    }

    // handle request to power off
    if (req_bms_state == BmsState::POWERED_OFF) {
        switch (bms_state) {
        case BmsState::IDLE:
        case BmsState::POWERED_OFF:
            // already powered off -- do nothing
            break;
        case BmsState::POWERED_ON:
            // move to powering off state
            bms_transition_counter = 8;
            bms_state = BmsState::POWERING_OFF;
            break;
        case BmsState::POWERING_ON:
            // move to powering off state.  transition counter should already be between 0 and 8
            bms_state = BmsState::POWERING_OFF;
            break;
        case BmsState::POWERING_OFF:
            if (bms_transition_counter > 0) {
                bms_transition_counter--;
            }
            if (bms_transition_counter == 0) {
                bms_state = BmsState::POWERED_OFF;
                periph.battery_lib.set_powered_state(0, false);
            }
            break;
        }
        return;
    }
}

#endif  // AP_PERIPH_BATTERY_BMS_ENABLED
