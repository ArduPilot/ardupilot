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

#include "AP_Periph.h"

#ifdef HAL_PERIPH_ENABLE_FSO_POWER_STACK

#include <dronecan_msgs.h>

#define FSO_OUT_VOLTS_DIFF_MAX              2.0     // Maximum difference between output and batteries before turn on
#define FSO_BATT_VOLTS_DIFF_MAX             1.0     // Maximum difference between batteries before turn on
#define FSO_PRECHARGE_TIMEOUT_MS            500     // Maximum pre-charge time before turn on is aborted
#define FSO_SWITCH_ON_TIME_MS               500     // Minimum press time to turn on
#define FSO_SWITCH_OFF_TIME_MS              1000    // Minimum press time to turn off
#define FSO_LOOP_TIME_MS                    100     // Loop time in ms
#define FSO_PAYLOAD_HV_CURRENT_MAX          25.0    // Maximum current of the HV payload output
#define FSO_PAYLOAD_BEC_CURRENT_MAX         10.0    // Maximum current of the payload BEC's
#define FSO_PAYLOAD_BEC_TEMPERATURE_MAX     100.0   // Maximum temperature of the payload BEC's
#define FSO_INTERNAL_BEC_HC_CURRENT_MAX     10.0    // Maximum current of the high current internal BEC
#define FSO_INTERNAL_BEC_CURRENT_MAX        1.0     // Maximum current of the low current internal BEC's
#define FSO_INTERNAL_BEC_TEMPERATURE_MAX    100.0   // Maximum temperature of the payload BEC's

extern const AP_HAL::HAL &hal;

const AP_Param::GroupInfo FSOPowerStack::var_info[] {
    // @Param: _OPTIONS
    // @DisplayName: FSO Options
    // @Description: FSO Options
    // @Bitmask: 0:Debug
    AP_GROUPINFO("_OPTIONS", 1, FSOPowerStack, options, 0),

    // @Param: _HV_AMP_LIM
    // @DisplayName: Payload HV amp limit
    // @Description: Payload HV amp limit
    // @Range: 0 25
    AP_GROUPINFO("_HV_AMP_LIM", 2, FSOPowerStack, payload_HV_current_max, FSO_PAYLOAD_HV_CURRENT_MAX),

    // @Param: _BEC1_VOLT
    // @DisplayName: Payload 1 voltage
    // @Description: Payload 1 voltage
    // @Range: 0 100
    AP_GROUPINFO("_BEC1_VOLT", 3, FSOPowerStack, payload_1_voltage, 5.0),

    // @Param: _BEC2_VOLT
    // @DisplayName: Payload 2 voltage
    // @Description: Payload 2 voltage
    // @Range: 0 100
    AP_GROUPINFO("_BEC2_VOLT", 4, FSOPowerStack, payload_2_voltage, 12.0),

    // @Param: _BEC1_AMP_LIM
    // @DisplayName: Payload 1 amp limit
    // @Description: Payload 1 amp limit
    // @Range: 0 10
    AP_GROUPINFO("_BEC1_AMP_LIM", 5, FSOPowerStack, payload_1_current_max, FSO_PAYLOAD_BEC_CURRENT_MAX),

    // @Param: _BEC2_AMP_LIM
    // @DisplayName: Payload 2 amp limit
    // @Description: Payload 2 amp limit
    // @Range: 0 10
    AP_GROUPINFO("_BEC2_AMP_LIM", 6, FSOPowerStack, payload_2_current_max, FSO_PAYLOAD_BEC_CURRENT_MAX),

    // @Param: _BEC_TEMP_LIM
    // @DisplayName: Maximum temperature of BEC circuits before shutdown or warning
    // @Description: Maximum temperature of BEC circuits before shutdown or warning
    // @Range: 0 100
    AP_GROUPINFO("_BEC_TEMP_LIM", 7, FSOPowerStack, bec_temperature_max, FSO_PAYLOAD_BEC_TEMPERATURE_MAX),

    // @Param: _FAN_1_MIN
    // @DisplayName: Alarm threshold for minimum fan 1 tachometer in Hz
    // @Description: Alarm threshold for minimum fan 1 tachometer in Hz
    // @Range: 0 200
    AP_GROUPINFO("_FAN_1_MIN", 8, FSOPowerStack, fan_1_min_Hz, 0.0),

    // @Param: _FAN_2_MIN
    // @DisplayName: Alarm threshold for minimum fan 2 tachometer in Hz
    // @Description: Alarm threshold for minimum fan 2 tachometer in Hz
    // @Range: 0 200
    AP_GROUPINFO("_FAN_2_MIN", 9, FSOPowerStack, fan_2_min_Hz, 0.0),

    // @Param: _FAN_3_MIN
    // @DisplayName: Alarm threshold for minimum fan 2 tachometer in Hz
    // @Description: Alarm threshold for minimum fan 2 tachometer in Hz
    // @Range: 0 200
    AP_GROUPINFO("_FAN_3_MIN", 10, FSOPowerStack, fan_3_min_Hz, 0.0),

    // @Param: _FAN_4_MIN
    // @DisplayName: Alarm threshold for minimum fan 2 tachometer in Hz
    // @Description: Alarm threshold for minimum fan 2 tachometer in Hz
    // @Range: 0 200
    AP_GROUPINFO("_FAN_4_MIN", 11, FSOPowerStack, fan_4_min_Hz, 0.0),

    // @Group: DAC
    // @Path: ../../libraries/AP_DAC/AP_DAC.cpp
    AP_SUBGROUPINFO(dac, "_DAC", 12, FSOPowerStack, AP_DAC),

    AP_GROUPEND
};

FSOPowerStack::FSOPowerStack(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}

/*
  handle interrupt for a fan
 */
void FSOPowerStack::fan_handler(uint8_t pin, bool pin_state, uint32_t timestamp)
{
    for (uint8_t i=0; i<ARRAY_SIZE(fans); i++) {
        auto &fan = fans[i];
        if (pin == fan.pin) {
            if (fan.last_pulse_us != 0) {
                const uint32_t dt = timestamp - fan.last_pulse_us;
                fan.dt_sum += dt;
                fan.dt_count++;
            }
            fan.last_pulse_us = timestamp;
        }
    }
}

/*
  initialise a fan interrupt
 */
void FSOPowerStack::init_fan(uint8_t pin, FAN &fan)
{
    fan.pin = pin;
    hal.gpio->pinMode(pin, HAL_GPIO_INPUT);
    hal.gpio->attach_interrupt(pin,
                               FUNCTOR_BIND_MEMBER(&FSOPowerStack::fan_handler, void, uint8_t, bool, uint32_t),
                               AP_HAL::GPIO::INTERRUPT_RISING);
}

void FSOPowerStack::init()
{
    init_fan(FSO_FAN_TACH1_PIN, fans[0]);
    init_fan(FSO_FAN_TACH2_PIN, fans[1]);
    init_fan(FSO_FAN_TACH3_PIN, fans[2]);
    init_fan(FSO_FAN_TACH4_PIN, fans[3]);

    float sample_freq = 1.0 / FSO_LOOP_TIME_MS;
    float over_current_tc = 1.0;
    payload_HV_current_filter.set_cutoff_frequency(sample_freq, 1.0/over_current_tc);
    payload_1_current_filter.set_cutoff_frequency(sample_freq, 1.0/over_current_tc);
    payload_2_current_filter.set_cutoff_frequency(sample_freq, 1.0/over_current_tc);
}

/*
  init called after CAN init
 */
void FSOPowerStack::late_init()
{
    dac.init();
}

/*
  update fan frequency reading
 */
void FSOPowerStack::update_fans(void)
{
    uint32_t now_ms = AP_HAL::millis();
    // update fans at 1Hz
    if (now_ms - last_fan_ms < 1000) {
        return;
    }
    last_fan_ms = now_ms;

    for (auto &fan : fans) {
        if (fan.dt_count == 0) {
            fan.freq_hz = 0;
            continue;
        }
        void *irqstate = hal.scheduler->disable_interrupts_save();
        const float dt_avg = float(fan.dt_sum) / fan.dt_count;
        fan.dt_sum = 0;
        fan.dt_count = 0;
        hal.scheduler->restore_interrupts(irqstate);
        fan.freq_hz = 1.0/(dt_avg*1.0e-6);
    }

    if (fans[0].freq_hz < fan_1_min_Hz) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Fan 1 failure");
    }
    if (fans[1].freq_hz < fan_2_min_Hz){
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Fan 2 failure");
    }
    if (fans[2].freq_hz < fan_3_min_Hz){
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Fan 3 failure");
    }
    if (fans[3].freq_hz < fan_4_min_Hz){
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Fan 4 failure");
    }
}

void FSOPowerStack::report(void)
{
    if (!option_is_set(Option::DEBUG)) {
        return;
    }

    uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_report_ms < 5000) {
        return;
    }
    last_report_ms = now_ms;

    auto &batt = AP::battery();

    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Volt - B1:%.1f, B2:%.1f, PHV:%.1f, P1:%.2f, P2:%.2f, IHC:%.2f, I1:%.2f, I2:%.2f, O:%.2f",
                  batt.voltage(0), batt.voltage(1), batt.voltage(2), batt.voltage(3),
                  batt.voltage(4), batt.voltage(5), batt.voltage(6), batt.voltage(7), batt.voltage(8));

    float B1_C;
    float B2_C;
    float PHV_C;
    float P1_C;
    float P2_C;
    float IHC_C;
    float I1_C;
    float I2_C;
    if (batt.current_amps(B1_C, 0) || batt.current_amps(B2_C, 1) || batt.current_amps(PHV_C, 2) || batt.current_amps(P1_C, 3) || batt.current_amps(P2_C, 4) || batt.current_amps(IHC_C, 5) || batt.current_amps(I1_C, 6) || batt.current_amps(I2_C, 7)) {
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Current - B1:%.4f, B2:%.4f, PHV:%.2f, P1:%.2f, P2:%.2f, IHC:%.2f, I1:%.2f, I2:%.2f",
                  B1_C, B2_C, PHV_C, P1_C, P2_C, IHC_C, I1_C, I2_C);
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Current Limit - PHV:%.2f, P1:%.2f, P2:%.2f,  IHC:%.2f, I1:%.2f, I2:%.2f",
                  payload_HV_current_filter.get(), payload_1_current_filter.get(), payload_2_current_filter.get(),
                  internal_HC_current_filter.get(), internal_1_current_filter.get(), internal_2_current_filter.get());
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Temp Limit - P1:%.1f, P2:%.1f, IHC:%.1f, I1:%.1f, I2:%.1f",
                   payload_1_temp, payload_2_temp, internal_HC_temp, internal_1_temp, internal_2_temp);
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Fans - F1: %.0f, F2: %.0f, F3: %.0f, F4: %.0f",
                    fans[0].freq_hz, fans[1].freq_hz, fans[2].freq_hz, fans[3].freq_hz);
}


void FSOPowerStack::update_switches()
{
    uint32_t now_ms = AP_HAL::millis();
    bool switch_1_pressed = hal.gpio->read(FSO_SWITCH_MAIN_PIN);
    bool switch_2_pressed = hal.gpio->read(FSO_SWITCH_PAYLOAD_PIN);

    if (!switch_1_pressed) {
        switch_1_press_time_ms = now_ms;
        switch_1_switch_released = true;
    }
    
    if (switch_1_on == false){
        if (switch_1_switch_released && (now_ms - switch_1_press_time_ms > FSO_SWITCH_ON_TIME_MS)) {
            switch_1_on = true;
            switch_1_switch_released = false;
        }
    } else {
        if (switch_1_switch_released && (now_ms - switch_1_press_time_ms > FSO_SWITCH_OFF_TIME_MS)) {
            switch_1_on = false;
            switch_1_switch_released = false;
        }
    }

    if (!switch_2_pressed) {
        switch_2_press_time_ms = now_ms;
        switch_2_switch_released = true;
    }
    
    if (switch_2_on == false){
        if (switch_2_switch_released && (now_ms - switch_2_press_time_ms > FSO_SWITCH_ON_TIME_MS)) {
            switch_2_on = true;
            switch_2_switch_released = false;
        }
    } else {
        if (switch_2_switch_released && (now_ms - switch_2_press_time_ms > FSO_SWITCH_OFF_TIME_MS)) {
            switch_2_on = false;
            switch_2_switch_released = false;
        }
    }
}

void FSOPowerStack::update_main_power()
{
    uint32_t now_ms = AP_HAL::millis();
    auto &batt = AP::battery();
    
    // Main Turn On State Machine
    switch (main_turn_on_state) {

    case Off:
        if (switch_1_on == true) {
            main_turn_on_state = PreChargeStart;
        }
        break;

    case PreChargeStart:
        if (fabsf(batt.voltage(0) - batt.voltage(1)) < FSO_BATT_VOLTS_DIFF_MAX) {
            hal.gpio->write(FSO_LED_DEBUG_PIN, 1);
            hal.gpio->write(FSO_MAIN_PC_PIN, 1);
            start_main_precharge_ms = now_ms;
            main_turn_on_state = PreCharge;
            break;
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Unbalanced Battery Voltage");
            main_turn_on_state = ShutDown;
        }
        break;

    case PreCharge:
        if (MAX(batt.voltage(0), batt.voltage(1)) - batt.voltage(8) < FSO_OUT_VOLTS_DIFF_MAX) {
            // Turn off Pre-Charge
            hal.gpio->write(FSO_LED_DEBUG_PIN, 0);
            hal.gpio->write(FSO_MAIN_PC_PIN, 0);

            // Turn on Main Power and Led indicator
            hal.gpio->write(FSO_LED_MAIN_PIN, 1);
            hal.gpio->write(FSO_BAT_1_EN_PIN, 1);
            hal.gpio->write(FSO_BAT_2_EN_PIN, 1);
            main_turn_on_state = On;
        } else if (now_ms - start_main_precharge_ms > FSO_PRECHARGE_TIMEOUT_MS) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Main Pre-Charge Failure Diff Voltage: %.2f", MAX(batt.voltage(0), batt.voltage(1)) - batt.voltage(9));
            main_turn_on_state = ShutDown;
        }
        break;

    case On:
        if (switch_1_on == false) {
            // Check if armed
            // If armed override switch state
            // switch_1_on == true;
            // else
            main_turn_on_state = ShutDown;
        }
        break;

    case ShutDown:
        // Turn off Pre-Charge
        hal.gpio->write(FSO_LED_DEBUG_PIN, 0);
        hal.gpio->write(FSO_MAIN_PC_PIN, 0);

        // Turn off Main Power and Led indicator
        hal.gpio->write(FSO_LED_MAIN_PIN, 0);
        hal.gpio->write(FSO_BAT_1_EN_PIN, 0);
        hal.gpio->write(FSO_BAT_2_EN_PIN, 0);
        main_turn_on_state = Off;

        // Override swich state
        switch_1_on = false;
        break;
    }
}

void FSOPowerStack::update_payload_HV_power()
{
    uint32_t now_ms = AP_HAL::millis();
    auto &batt = AP::battery();
    
    float payload_HV_current;
    if (batt.current_amps(payload_HV_current, 3)) {
        payload_HV_current_filter.apply(payload_HV_current, 0.001 * FSO_LOOP_TIME_MS);
        if (payload_HV_current_filter.get() > MIN(payload_HV_current_max, FSO_PAYLOAD_HV_CURRENT_MAX)) {
            payload_HV_turn_on_state = ShutDown;
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Payload HV over current shutdown");
        }
    }
    
    // Payload Turn On State Machine
    switch (payload_HV_turn_on_state) {

    case Off:
        if (switch_1_on == true || switch_2_on == true) {
            payload_HV_turn_on_state = PreChargeStart;
        }
        break;

    case PreChargeStart:
        // Turn on Payload BEC
        hal.gpio->write(FSO_PAYLOAD_1_EN_PIN, 1);
        hal.gpio->write(FSO_PAYLOAD_2_EN_PIN, 1);

        // Turn on Payload HV pre-charge
        hal.gpio->write(FSO_PAYLOAD_HV_PC_PIN, 1);
        start_payload_HV_precharge_ms = now_ms;
        payload_HV_turn_on_state = PreCharge;
        break;

    case PreCharge:
        if (MAX(batt.voltage(0), batt.voltage(1)) - batt.voltage(2) < FSO_OUT_VOLTS_DIFF_MAX) {
            // Turn off Pre-Charge
            hal.gpio->write(FSO_LED_DEBUG_PIN, 0);
            hal.gpio->write(FSO_PAYLOAD_HV_PC_PIN, 0);

            // Turn on Main Power and Led indicator
            hal.gpio->write(FSO_LED_PAYLOAD_PIN, 1);
            hal.gpio->write(FSO_PAYLOAD_HV_EN_PIN, 1);
            payload_HV_turn_on_state = On;
        } else if (now_ms - start_payload_HV_precharge_ms > FSO_PRECHARGE_TIMEOUT_MS) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Payload Pre-Charge Failure Diff Voltage: %.2f", MAX(batt.voltage(0), batt.voltage(1)) - batt.voltage(2));
            payload_HV_turn_on_state = ShutDown;
        }
        break;

    case On:
        if (switch_1_on == false && switch_2_on == false) {
            payload_HV_turn_on_state = ShutDown;
        }
        break;

    case ShutDown:
        // Turn off Payload BEC
        hal.gpio->write(FSO_PAYLOAD_1_EN_PIN, 0);
        hal.gpio->write(FSO_PAYLOAD_2_EN_PIN, 0);
        
        // Turn off Pre-Charge
        hal.gpio->write(FSO_LED_DEBUG_PIN, 0);
        hal.gpio->write(FSO_PAYLOAD_HV_PC_PIN, 0);

        // Turn off Main Power and Led indicator
        hal.gpio->write(FSO_LED_PAYLOAD_PIN, 0);
        hal.gpio->write(FSO_PAYLOAD_HV_EN_PIN, 0);
        payload_HV_turn_on_state = Off;

        // Override swich state
        // This needs to be improved as it will turn on again due to switch 1.
        switch_2_on = false;
        break;
    }
}

void FSOPowerStack::update_payload_BEC()
{
    auto &batt = AP::battery();

    float payload_1_current;
    if (batt.current_amps(payload_1_current, 3)) {
        payload_1_current_filter.apply(payload_1_current, 0.001 * FSO_LOOP_TIME_MS);
        if (payload_1_current_filter.get() > MIN(payload_1_current_max, FSO_PAYLOAD_BEC_CURRENT_MAX)) {
            hal.gpio->write(FSO_PAYLOAD_1_EN_PIN, 0);
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Payload 1 over current shutdown");
        }
    }
    
    float payload_2_current;
    if (batt.current_amps(payload_2_current, 4)) {
        payload_2_current_filter.apply(payload_2_current, 0.001 * FSO_LOOP_TIME_MS);
        if (payload_2_current_filter.get() > MIN(payload_2_current_max, FSO_PAYLOAD_BEC_CURRENT_MAX)) {
            hal.gpio->write(FSO_PAYLOAD_2_EN_PIN, 0);
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Payload 2 over current shutdown");
        }
    }
    
    if (batt.get_temperature(payload_1_temp, 3)) {
        if (payload_1_temp > MIN(bec_temperature_max, FSO_PAYLOAD_BEC_TEMPERATURE_MAX)) {
            hal.gpio->write(FSO_PAYLOAD_1_EN_PIN, 0);
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Payload 1 over temperature shutdown");
        }
    }
    
    if (batt.get_temperature(payload_2_temp, 4)) {
        if (payload_2_temp > MIN(bec_temperature_max, FSO_PAYLOAD_BEC_TEMPERATURE_MAX)) {
            hal.gpio->write(FSO_PAYLOAD_2_EN_PIN, 0);
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Payload 2 over temperature shutdown");
        }
    }
}

void FSOPowerStack::update_internal_BEC()
{
    auto &batt = AP::battery();

    float internal_HC_current;
    if (batt.current_amps(internal_HC_current, 5)) {
        internal_HC_current_filter.apply(internal_HC_current, 0.001 * FSO_LOOP_TIME_MS);
        if (internal_HC_current_filter.get() > FSO_INTERNAL_BEC_HC_CURRENT_MAX) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Internal BEC HC Over Current Fault");
        }
    }
    
    float internal_1_current;
    if (batt.current_amps(internal_1_current, 6)) {
        internal_1_current_filter.apply(internal_1_current, 0.001 * FSO_LOOP_TIME_MS);
        if (internal_1_current_filter.get() > FSO_INTERNAL_BEC_CURRENT_MAX) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Internal BEC 1 Over Current Fault");
        }
    }
    
    float internal_2_current;
    if (batt.current_amps(internal_2_current, 7)) {
        internal_2_current_filter.apply(internal_2_current, 0.001 * FSO_LOOP_TIME_MS);
        if (internal_2_current_filter.get() > FSO_INTERNAL_BEC_CURRENT_MAX) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Internal BEC 2 Over Current Fault");
        }
    }
    
    if (batt.get_temperature(internal_HC_temp, 5)) {
        if (internal_HC_temp > FSO_INTERNAL_BEC_TEMPERATURE_MAX) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Internal BEC HC Over Temperature Fault");
        }
    }
    
    if (batt.get_temperature(internal_1_temp, 6)) {
        if (internal_1_temp > FSO_INTERNAL_BEC_TEMPERATURE_MAX) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Internal BEC 1 Over Temperature Fault");
        }
    }
    
    if (batt.get_temperature(internal_2_temp, 7)) {
        if (internal_2_temp > FSO_INTERNAL_BEC_TEMPERATURE_MAX) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Internal BEC 2 Over Temperature Fault");
        }
    }
}

/*
  update DACs
 */
void FSOPowerStack::update_DAC()
{
    const float t = AP_HAL::micros() * 1.0e-6;
    for (uint8_t i=0; i<4; i++) {
        const float freq = 0.2;
        const float v = 1 + sinf(freq * (i+1) * t * 2 * M_PI);
        if (!dac.set_voltage(0, i, v)) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "set voltage %u %.3f failed", unsigned(i), v);
        }
    }
}

/*
  update FSOPowerStack
 */
void FSOPowerStack::update()
{
    const uint32_t now_ms = AP_HAL::millis();

    if (!done_late_init) {
        done_late_init = true;
        late_init();
    }

    // update DAC fast to get smooth sine wave
    update_DAC();
    
    if (now_ms - last_update_ms < FSO_LOOP_TIME_MS) {
        // run at 10Hz
        return;
    }
    last_update_ms = now_ms;

    update_switches();

    update_fans();

    report();

    update_main_power();

    update_payload_HV_power();

    update_payload_BEC();

    update_internal_BEC();
}

#endif  // HAL_PERIPH_ENABLE_FSO_POWER_STACK

