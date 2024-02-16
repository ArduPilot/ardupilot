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

    // @Param: _CAL_MV
    // @DisplayName: Main voltage reference
    // @Description: Main voltage reference
    // @Range: 0 50
    AP_GROUPINFO("_CAL_MV", 13, FSOPowerStack, cal_main_voltage, 50.0),

    // @Param: _CAL_MC
    // @DisplayName: Main current reference
    // @Description: Main current reference
    // @Range: 0 200
    AP_GROUPINFO("_CAL_MC", 14, FSOPowerStack, cal_main_current, 10.0),

    // @Param: _CAL_HVC
    // @DisplayName: High voltage payload current reference
    // @Description: High voltage payload current reference
    // @Range: 0 10
    AP_GROUPINFO("_CAL_HVC", 15, FSOPowerStack, cal_HV_current, 2.5),

    // @Param: _CAL_HCC
    // @DisplayName: High current BEC current reference
    // @Description: High current BEC current reference
    // @Range: 0 10
    AP_GROUPINFO("_CAL_HCC", 16, FSOPowerStack, cal_HCB_current, 5.0),

    // @Param: _CAL_LCC
    // @DisplayName: Low current BEC current reference
    // @Description: Low current BEC current reference
    // @Range: 0 10
    AP_GROUPINFO("_CAL_LCC", 17, FSOPowerStack, cal_LCB_current, 1.0),

    // @Param: _CAL_P1C1
    // @DisplayName: Payload 1 Coefficient 1
    // @Description: Payload 1 Coefficient 1
    // @Range: 0 10
    AP_GROUPINFO("_CAL_P1C1", 18, FSOPowerStack, cal_payload_P1c1, 25.4948),

    // @Param: _CAL_P1C2
    // @DisplayName: Payload 1 Coefficient 2
    // @Description: Payload 1 Coefficient 2
    // @Range: 0 10
    AP_GROUPINFO("_CAL_P1C2", 19, FSOPowerStack, cal_payload_P1c2, 0.2007),

    // @Param: _CAL_P2C1
    // @DisplayName: Payload 2 Coefficient 1
    // @Description: Payload 2 Coefficient 1
    // @Range: 0 10
    AP_GROUPINFO("_CAL_P2C1", 20, FSOPowerStack, cal_payload_P2c1, 25.4948),

    // @Param: _CAL_P2C2
    // @DisplayName: Payload 2 Coefficient 2
    // @Description: Payload 2 Coefficient 2
    // @Range: 0 10
    AP_GROUPINFO("_CAL_P2C2", 21, FSOPowerStack, cal_payload_P2c2, 0.2007),

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
    
    if (switch_1_is_off()){
        if (switch_1_switch_released && (now_ms - switch_1_press_time_ms > FSO_SWITCH_ON_TIME_MS)) {
            set_switch_1_on();
            switch_1_switch_released = false;
        }
    } else {
        if (switch_1_switch_released && (now_ms - switch_1_press_time_ms > FSO_SWITCH_OFF_TIME_MS)) {
            set_switch_1_off();
            switch_1_switch_released = false;
        }
    }

    if (!switch_2_pressed) {
        switch_2_press_time_ms = now_ms;
        switch_2_switch_released = true;
    }
    
    if (switch_2_on == false){
        if (switch_2_switch_released && (now_ms - switch_2_press_time_ms > FSO_SWITCH_ON_TIME_MS)) {
            set_switch_2_on();
            switch_2_switch_released = false;
        }
    } else {
        if (switch_2_switch_released && (now_ms - switch_2_press_time_ms > FSO_SWITCH_OFF_TIME_MS)) {
            set_switch_2_off();
            switch_2_switch_released = false;
        }
    }
}

void FSOPowerStack::update_main_power()
{
    uint32_t now_ms = AP_HAL::millis();
    auto &batt = AP::battery();
    
    // Main Turn On State Machine
    switch (main_state) {

    case TurnOnState::Off:
        if (main_on == true) {
            main_state = TurnOnState::PreChargeStart;
        }
        break;

    case TurnOnState::PreChargeStart:
        if (fabsf(batt.voltage(0) - batt.voltage(1)) < FSO_BATT_VOLTS_DIFF_MAX) {
            set_main_PC_on();
            start_main_precharge_ms = now_ms;
            main_state = TurnOnState::PreCharge;
            break;
        } else {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Unbalanced Battery Voltage");
            main_state = TurnOnState::ShutDown;
        }
        break;

    case TurnOnState::PreCharge:
        if (MAX(batt.voltage(0), batt.voltage(1)) - batt.voltage(8) < FSO_OUT_VOLTS_DIFF_MAX) {
            // Turn off Pre-Charge
            set_main_PC_off();

            // Turn on main battery switches
            set_bat_1_SW_on();
            set_bat_2_SW_on();
            main_state = TurnOnState::On;
        } else if (now_ms - start_main_precharge_ms > FSO_PRECHARGE_TIMEOUT_MS) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Main Pre-Charge Failure Diff Voltage: %.2f", MAX(batt.voltage(0), batt.voltage(1)) - batt.voltage(9));
            main_state = ShutDown;
        }
        break;

    case TurnOnState::On:
        if (main_on == false) {
            main_state = TurnOnState::ShutDown;
        }
        break;

    case TurnOnState::ShutDown:
        // Turn off Pre-Charge
        set_main_PC_off();
        set_bat_1_SW_off();
        set_bat_2_SW_off();
        main_state = TurnOnState::Off;
        break;
    }
}

void FSOPowerStack::update_payload_HV_power()
{
    uint32_t now_ms = AP_HAL::millis();
    auto &batt = AP::battery();
    
    float payload_HV_current;
    if (batt.current_amps(payload_HV_current, 2)) {
        payload_HV_current_filter.apply(payload_HV_current, 0.001 * FSO_LOOP_TIME_MS);
        if (payload_HV_current_filter.get() > MIN(payload_HV_current_max, FSO_PAYLOAD_HV_CURRENT_MAX)) {
            payload_HV_state = ShutDown;
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Payload HV over current shutdown");
        }
    }
    
    // Payload Turn On State Machine
    switch (payload_HV_state) {

    case TurnOnState::Off:
        if (payload_HV_on == true) {
            payload_HV_state = TurnOnState::PreChargeStart;
        }
        break;

    case TurnOnState::PreChargeStart:
        // Turn on Payload HV pre-charge
        set_payload_HV_PC_on();
        start_payload_HV_precharge_ms = now_ms;
        payload_HV_state = TurnOnState::PreCharge;
        break;

    case TurnOnState::PreCharge:
        if (MAX(batt.voltage(0), batt.voltage(1)) - batt.voltage(2) < FSO_OUT_VOLTS_DIFF_MAX) {
            // Turn off payload HV pre-charge
            set_payload_HV_PC_off();

            // Turn on payload HV switch
            set_payload_HV_SW_on();

            payload_HV_state = TurnOnState::On;
        } else if (now_ms - start_payload_HV_precharge_ms > FSO_PRECHARGE_TIMEOUT_MS) {
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Payload Pre-Charge Failure Diff Voltage: %.2f", MAX(batt.voltage(0), batt.voltage(1)) - batt.voltage(2));
            payload_HV_state = ShutDown;
        }
        break;

    case TurnOnState::On:
        if (payload_HV_on == false) {
            payload_HV_state = TurnOnState::ShutDown;
        }
        break;

    case TurnOnState::ShutDown:
        // Turn off payload HV pre-charge
        set_payload_HV_PC_off();

        // Turn off payload HV switch
        set_payload_HV_SW_off();

        payload_HV_state = TurnOnState::Off;
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
            set_payload_BEC_1_off();
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Payload 1 over current shutdown");
        }
    }
    
    float payload_2_current;
    if (batt.current_amps(payload_2_current, 4)) {
        payload_2_current_filter.apply(payload_2_current, 0.001 * FSO_LOOP_TIME_MS);
        if (payload_2_current_filter.get() > MIN(payload_2_current_max, FSO_PAYLOAD_BEC_CURRENT_MAX)) {
            set_payload_BEC_2_off();
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Payload 2 over current shutdown");
        }
    }
    
    if (batt.get_temperature(payload_1_temp, 3)) {
        if (payload_1_temp > MIN(bec_temperature_max, FSO_PAYLOAD_BEC_TEMPERATURE_MAX)) {
            set_payload_BEC_1_off();
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Payload 1 over temperature shutdown");
        }
    }
    
    if (batt.get_temperature(payload_2_temp, 4)) {
        if (payload_2_temp > MIN(bec_temperature_max, FSO_PAYLOAD_BEC_TEMPERATURE_MAX)) {
            set_payload_BEC_2_off();
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
    const float v1 = (cal_payload_P1c1 - payload_1_voltage) * cal_payload_P1c2;
    if (!dac.set_voltage(0, 0, v1)) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "set voltage %u %.3f failed", unsigned(0), v1);
    }
    const float v2 = (cal_payload_P2c1 - payload_2_voltage) * cal_payload_P2c2;
    if (!dac.set_voltage(0, 3, v2)) {
        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "set voltage %u %.3f failed", unsigned(3), v2);
    }
}

/*
  Calibrate Power Stack
 */
void FSOPowerStack::calibrate()
{
    const uint32_t now_ms = AP_HAL::millis();
    auto &batt = AP::battery();
    float payload_current;
    // Calibration State Machine
    switch (cal_state) {

    case  CalibrateState::Begin:
        switch (cal_sub_state) {

        case CalibrateSubState::Setup:
            set_main_off();
            set_HV_payload_off();
            set_payload_BEC_1_off();
            set_payload_BEC_2_off();
            set_switch_1_off();
            set_LED_1_off();
            set_LED_debug_on();
            last_update_ms = now_ms;
            cal_sub_state = CalibrateSubState::Start;
            break;

        default:
            if (now_ms - last_update_ms > 2000) {
                cal_state = CalibrateState::Payload_BEC_C1;
                cal_sub_state = CalibrateSubState::Setup;
            }
            break;
        }
        break;

    case  CalibrateState::Payload_BEC_C1:
        switch (cal_sub_state) {

        case CalibrateSubState::Setup:
            set_switch_1_off();
            set_LED_1_off();
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Calibrating BEC control: C1");

            // Setup for Payload_BEC_C1
            if (!dac.set_voltage(0, 0, 0)) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "set voltage %u %.3f failed", unsigned(0), 0.0);
            }
            if (!dac.set_voltage(0, 3, 0)) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "set voltage %u %.3f failed", unsigned(3), 0.0);
            }
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Press Main Button to start");
            cal_sub_state = CalibrateSubState::Start;
            break;

        case CalibrateSubState::Start:
            if (switch_1_is_on()) {
                set_LED_1_on();
                // Turn on Payload BEC
                set_payload_BEC_1_on();
                set_payload_BEC_2_on();
                cal_measurement_count = 0;
                cal_measurement_1 = 0.0;
                cal_measurement_2 = 0.0;
                last_update_ms = now_ms;
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Measuring C1");
                cal_sub_state = CalibrateSubState::Measure;
            }
            break;

        case CalibrateSubState::Measure:
            if (now_ms - last_update_ms > 5000) {
                cal_sub_state = CalibrateSubState::Write;
            } else if (now_ms - last_update_ms > 1000) {
                cal_measurement_count += 1;
                cal_measurement_1 += batt.voltage(3);
                cal_measurement_2 += batt.voltage(4);
            }
            break;

        case CalibrateSubState::Write:
            cal_payload_P1c1.set_and_save(cal_measurement_1 / cal_measurement_count);
            cal_payload_P2c1.set_and_save(cal_measurement_2 / cal_measurement_count);
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Payload_BEC_C1 done. P1c1: %.4f, P2c1: %.4f", (float)cal_payload_P1c1, (float)cal_payload_P2c1);
            set_payload_BEC_1_off();
            set_payload_BEC_2_off();
            update_DAC();

            cal_state = CalibrateState::Payload_BEC_C2;
            cal_sub_state = CalibrateSubState::Setup;
            break;
        }
        break;

    case  CalibrateState::Payload_BEC_C2:
        switch (cal_sub_state) {

        case CalibrateSubState::Setup:
            set_switch_1_off();
            set_LED_1_off();
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Calibrating BEC control: C2");
            if (!dac.set_voltage(0, 0, 4.0)) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "set voltage %u %.3f failed", unsigned(0), 4.0);
            }
            if (!dac.set_voltage(0, 3, 4.0)) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "set voltage %u %.3f failed", unsigned(3), 4.0);
            }
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Press Main Button to start");
            cal_sub_state = CalibrateSubState::Start;
            break;

        case CalibrateSubState::Start:
            if (switch_1_is_on()) {
                set_LED_1_on();
                set_payload_BEC_1_on();
                set_payload_BEC_2_on();
                cal_measurement_count = 0;
                cal_measurement_1 = 0.0;
                cal_measurement_2 = 0.0;
                last_update_ms = now_ms;
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Measuring C2");
                cal_sub_state = CalibrateSubState::Measure;
            }
            break;

        case CalibrateSubState::Measure:
            if (now_ms - last_update_ms > 5000) {
                cal_sub_state = CalibrateSubState::Write;
            } else if (now_ms - last_update_ms > 1000) {
                cal_measurement_count += 1;
                cal_measurement_1 += batt.voltage(3);
                cal_measurement_2 += batt.voltage(4);
            }
            break;

        case CalibrateSubState::Write:
            cal_payload_P1c2.set_and_save(4.0 / (cal_payload_P1c1 - cal_measurement_1 / cal_measurement_count));
            cal_payload_P2c2.set_and_save(4.0 / (cal_payload_P2c1 - cal_measurement_2 / cal_measurement_count));
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Payload_BEC_C2 done. P1c2: %.4f, P2c2: %.4f", (float)cal_payload_P1c2, (float)cal_payload_P2c2);
            set_payload_BEC_1_off();
            set_payload_BEC_2_off();
            update_DAC();

            cal_state = CalibrateState::Payload_BEC_1_Shunt;
            cal_sub_state = CalibrateSubState::Setup;
            break;
        }
        break;

    case  CalibrateState::Payload_BEC_1_Shunt:
        switch (cal_sub_state) {

        case CalibrateSubState::Setup: {
            set_switch_1_off();
            set_LED_1_off();
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Calibrating Payload BEC 1 shunt");
            power_on = false;
            const float v1 = (cal_payload_P1c1 - 5.0) * cal_payload_P1c2;
            if (!dac.set_voltage(0, 0, v1)) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "set voltage %u %.3f failed", unsigned(0), v1);
            }
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Connect P1 and set Load to %.1f A", (float)cal_HCB_current);
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Press Main Button to start");
            cal_sub_state = CalibrateSubState::Start;
            break;
        }

        case CalibrateSubState::Start:
            if (switch_1_is_on() && !power_on) {
                set_LED_1_on();
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Turn on load");

                // Turn on Payload BEC
                set_payload_BEC_1_on();
                power_on = true;

                cal_measurement_count = 0;
                cal_measurement_1 = 0.0;
                last_update_ms = now_ms;
            } else if (switch_1_is_on() && power_on) {
                if (batt.current_amps(payload_current, 3)) {
                    if (payload_current > 0.75 * cal_HCB_current) {
                        last_update_ms = now_ms;
                        cal_sub_state = CalibrateSubState::Measure;
                        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Measuring BATT4_SHUNT");
                    }
                }
            }
            break;

        case CalibrateSubState::Measure:
            if (now_ms - last_update_ms > 5000) {
                cal_sub_state = CalibrateSubState::Write;
            } else if (now_ms - last_update_ms > 1000) {
                if (batt.current_amps(payload_current, 3)) {
                    cal_measurement_count += 1;
                    cal_measurement_1 += payload_current;
                }
            }
            break;

        case CalibrateSubState::Write:
                float shunt;
                if (!AP_Param::get("BATT4_SHUNT", shunt)) {
                    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Could not get BATT4_SHUNT");
                }
                if (!AP_Param::set_and_save_by_name("BATT4_SHUNT", shunt * cal_HCB_current / (cal_measurement_1/cal_measurement_count))) {
                    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Could not write to BATT4_SHUNT");
                }
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Payload_BEC_1_Shunt done. Shunt correction factor: %.4f", cal_HCB_current / (cal_measurement_1/cal_measurement_count));
                set_payload_BEC_1_off();
                set_payload_BEC_2_off();
                update_DAC();
                cal_state = CalibrateState::Payload_BEC_2_Shunt;
                cal_sub_state = CalibrateSubState::Setup;
            break;
        }
        break;

    case  CalibrateState::Payload_BEC_2_Shunt:
        switch (cal_sub_state) {

        case CalibrateSubState::Setup: {
            set_switch_1_off();
            set_LED_1_off();
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Calibrating Payload BEC 2 shunt");
            power_on = false;
            const float v2 = (cal_payload_P2c1 - 5.0) * cal_payload_P2c2;
            if (!dac.set_voltage(0, 3, v2)) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "set voltage %u %.3f failed", unsigned(3), v2);
            }
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Connect P2 and set Load to %.1f A", (float)cal_HCB_current);
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Press Main Button to start");
            cal_sub_state = CalibrateSubState::Start;
            break;
        }

        case CalibrateSubState::Start: 
            if (switch_1_is_on() && !power_on) {
                set_LED_1_on();
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Turn on load");

                // Turn on Payload BEC
                set_payload_BEC_2_on();
                power_on = true;

                cal_measurement_count = 0;
                cal_measurement_1 = 0.0;
                last_update_ms = now_ms;
            } else if (switch_1_is_on() && power_on) {
                if (batt.current_amps(payload_current, 4)) {
                    if (payload_current > 0.75 * cal_HCB_current) {
                        last_update_ms = now_ms;
                        cal_sub_state = CalibrateSubState::Measure;
                        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Measuring BATT5_SHUNT");
                    }
                }
            }
            break;

        case CalibrateSubState::Measure:
            if (now_ms - last_update_ms > 5000) {
                cal_sub_state = CalibrateSubState::Write;
            } else if (now_ms - last_update_ms > 1000) {
                if (batt.current_amps(payload_current, 4)) {
                    cal_measurement_count += 1;
                    cal_measurement_1 += payload_current;
                }
            }
            break;

        case CalibrateSubState::Write:
            float shunt;
            if (!AP_Param::get("BATT5_SHUNT", shunt)) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Could not read BATT5_SHUNT");
            }
            if (!AP_Param::set_and_save_by_name("BATT5_SHUNT", shunt * cal_HCB_current / (cal_measurement_1/cal_measurement_count))) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Could not write to BATT5_SHUNT");
            }
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Payload_BEC_2_Shunt done. Shunt correction factor: %.4f", cal_HCB_current / (cal_measurement_1/cal_measurement_count));
            set_payload_BEC_1_off();
            set_payload_BEC_2_off();
            update_DAC();
            cal_state = CalibrateState::Internal_BEC_HC_Shunt;
            cal_sub_state = CalibrateSubState::Setup;
            break;
        }
        break;

    case  CalibrateState::Internal_BEC_HC_Shunt:
        switch (cal_sub_state) {

        case CalibrateSubState::Setup: {
            set_switch_1_off();
            set_LED_1_off();
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Calibrating Internal BEC HC shunt");
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Connect IHC and set Load to %.1f A", (float)cal_HCB_current);
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Press Main Button to start");
            cal_sub_state = CalibrateSubState::Start;
            break;
        }

        case CalibrateSubState::Start: 
            if (switch_1_is_on()) {
                set_LED_1_on();
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Turn on load");
                cal_measurement_count = 0;
                cal_measurement_1 = 0.0;
                last_update_ms = now_ms;
                cal_sub_state = CalibrateSubState::Measure;
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Measuring BATT6_SHUNT");
            }
            break;

        case CalibrateSubState::Measure:
            if (batt.current_amps(payload_current, 5)) {
                if (payload_current < 0.75 * cal_HCB_current) {
                    last_update_ms = now_ms;
                }
                if (now_ms - last_update_ms > 5000) {
                    cal_sub_state = CalibrateSubState::Write;
                } else if (now_ms - last_update_ms > 1000) {
                    cal_measurement_count += 1;
                    cal_measurement_1 += payload_current;
                }
            }
            break;

        case CalibrateSubState::Write:
            float shunt;
            if (!AP_Param::get("BATT6_SHUNT", shunt)) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Could not read BATT6_SHUNT");
            }
            if (!AP_Param::set_and_save_by_name("BATT6_SHUNT", shunt * cal_HCB_current / (cal_measurement_1/cal_measurement_count))) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Could not write to BATT6_SHUNT");
            }
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Internal_BEC_HC_Shunt done. Shunt correction factor: %.4f", cal_HCB_current / (cal_measurement_1/cal_measurement_count));

            cal_state = CalibrateState::Internal_BEC_1_Shunt;
            cal_sub_state = CalibrateSubState::Setup;
            break;
        }
        break;

    case  CalibrateState::Internal_BEC_1_Shunt:
        switch (cal_sub_state) {

        case CalibrateSubState::Setup: {
            set_switch_1_off();
            set_LED_1_off();
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Calibrating Internal BEC 1 shunt");
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Connect I1 and set Load to %.1f A", (float)cal_LCB_current);
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Press Main Button to start");
            cal_sub_state = CalibrateSubState::Start;
            break;
        }

        case CalibrateSubState::Start: 
            if (switch_1_is_on()) {
                set_LED_1_on();
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Turn on load");
                cal_measurement_count = 0;
                cal_measurement_1 = 0.0;
                last_update_ms = now_ms;
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Measuring BATT7_SHUNT");
                cal_sub_state = CalibrateSubState::Measure;
            }
            break;

        case CalibrateSubState::Measure:
            if (batt.current_amps(payload_current, 6)) {
                if (payload_current < 0.75 * cal_LCB_current) {
                    last_update_ms = now_ms;
                }
                if (now_ms - last_update_ms > 5000) {
                    cal_sub_state = CalibrateSubState::Write;
                } else if (now_ms - last_update_ms > 1000) {
                    cal_measurement_count += 1;
                    cal_measurement_1 += payload_current;
                }
            }
            break;

        case CalibrateSubState::Write:
            float shunt;
            if (!AP_Param::get("BATT7_SHUNT", shunt)) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Could not read BATT7_SHUNT");
            }
            if (!AP_Param::set_and_save_by_name("BATT7_SHUNT", shunt * cal_LCB_current / (cal_measurement_1/cal_measurement_count))) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Could not write to BATT7_SHUNT");
            }
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Internal_BEC_1_Shunt done. Shunt correction factor: %.4f", cal_LCB_current / (cal_measurement_1/cal_measurement_count));

            cal_state = CalibrateState::Internal_BEC_2_Shunt;
            cal_sub_state = CalibrateSubState::Setup;
            break;
        }
        break;

    case  CalibrateState::Internal_BEC_2_Shunt:
        switch (cal_sub_state) {

        case CalibrateSubState::Setup: {
            set_switch_1_off();
            set_LED_1_off();
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Calibrating Internal BEC 2 shunt");
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Connect I2 and set Load to %.1f A", (float)cal_LCB_current);
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Press Main Button to start");
            cal_sub_state = CalibrateSubState::Start;
            break;
        }

        case CalibrateSubState::Start: 
            if (switch_1_is_on()) {
                set_LED_1_on();
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Turn on load");
                cal_measurement_count = 0;
                cal_measurement_1 = 0.0;
                last_update_ms = now_ms;
                cal_sub_state = CalibrateSubState::Measure;
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Measuring BATT8_SHUNT");
            }
            break;

        case CalibrateSubState::Measure:
            if (batt.current_amps(payload_current, 7)) {
                if (payload_current < 0.75 * cal_LCB_current) {
                    last_update_ms = now_ms;
                }
                if (now_ms - last_update_ms > 5000) {
                    cal_sub_state = CalibrateSubState::Write;
                } else if (now_ms - last_update_ms > 1000) {
                    cal_measurement_count += 1;
                    cal_measurement_1 += payload_current;
                }
            }
            break;

        case CalibrateSubState::Write:
            float shunt;
            if (!AP_Param::get("BATT8_SHUNT", shunt)) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Could not read BATT8_SHUNT");
            }
            if (!AP_Param::set_and_save_by_name("BATT8_SHUNT", shunt * cal_LCB_current / (cal_measurement_1/cal_measurement_count))) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Could not write to BATT8_SHUNT");
            }
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Internal_BEC_2_Shunt done. Shunt correction factor: %.4f", cal_LCB_current / (cal_measurement_1/cal_measurement_count));

            cal_state = CalibrateState::Payload_HV_Shunt;
            cal_sub_state = CalibrateSubState::Setup;
            break;
        }
        break;

    case  CalibrateState::Payload_HV_Shunt:
        switch (cal_sub_state) {

        case CalibrateSubState::Setup:
            set_switch_1_off();
            set_LED_1_off();
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Calibrating Payload HV shunt");
            power_on = false;
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Connect Phv and set Load to %.1f A", (float)cal_HV_current);
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Press Main Button to start");
            cal_sub_state = CalibrateSubState::Start;
            break;

        case CalibrateSubState::Start:
            if (switch_1_is_on() && HV_payload_is_off()) {
                // Turn on Payload HV
                set_HV_payload_on();
            } else if (switch_1_is_on() && HV_payload_is_on() && !power_on) {
                set_LED_1_on();
                power_on = true;
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Turn on load");
            } else if (switch_1_is_on() && HV_payload_is_on() && power_on) {
                if (batt.current_amps(payload_current, 2)) {
                    if (payload_current > 0.75 * cal_HV_current) {
                        last_update_ms = now_ms;
                        cal_measurement_count = 0;
                        cal_measurement_1 = 0.0;
                        cal_sub_state = CalibrateSubState::Measure;
                        GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Measuring BATT3_SHUNT");
                    }
                }
            }
            break;

        case CalibrateSubState::Measure:
            if (now_ms - last_update_ms > 5000) {
                set_HV_payload_off();
                if (HV_payload_is_off()){
                    cal_sub_state = CalibrateSubState::Write;
                }
            } else if (now_ms - last_update_ms > 1000) {
                if (batt.current_amps(payload_current, 2)) {
                    cal_measurement_count += 1;
                    cal_measurement_1 += payload_current;
                }
            }

            break;

        case CalibrateSubState::Write:
            float shunt;
            if (!AP_Param::get("BATT3_SHUNT", shunt)) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Could not read BATT3_SHUNT");
            }
            if (!AP_Param::set_and_save_by_name("BATT3_SHUNT", shunt * cal_HV_current / (cal_measurement_1/cal_measurement_count))) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Could not write to BATT3_SHUNT");
            }
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Payload_HV_Shunt done. Shunt correction factor: %.4f", cal_HV_current / (cal_measurement_1/cal_measurement_count));
            cal_state = CalibrateState::Main_V_Divider;
            cal_sub_state = CalibrateSubState::Setup;
            break;
        }
        break;

    case  CalibrateState::Main_V_Divider:
        switch (cal_sub_state) {

        case CalibrateSubState::Setup:
            set_switch_1_off();
            set_LED_1_off();
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Calibrating Main voltage dividers");
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Set Main voltage to %.1f V", (float)cal_main_voltage);
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Press Main Button to start");
            cal_sub_state = CalibrateSubState::Start;
            break;

        case CalibrateSubState::Start:
            if (switch_1_is_on() && main_is_off()) {
                // Turn on Main
                set_main_on();
            } else if (switch_1_is_on() && main_is_on()) {
                set_LED_1_on();
                cal_measurement_count = 0;
                cal_measurement_1 = 0.0;
                cal_measurement_2 = 0.0;
                cal_measurement_3 = 0.0;
                last_update_ms = now_ms;
                cal_sub_state = CalibrateSubState::Measure;
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Measuring BATT_VOLT_MULT");
            }
            break;

        case CalibrateSubState::Measure:
            if (now_ms - last_update_ms > 5000) {
                set_main_off();
                if (main_is_off()){
                    cal_sub_state = CalibrateSubState::Write;
                }
            } else if (now_ms - last_update_ms > 1000) {
                cal_measurement_count += 1;
                cal_measurement_1 += batt.voltage(0);
                cal_measurement_2 += batt.voltage(1);
                cal_measurement_3 += batt.voltage(8);
            }
            break;

        case CalibrateSubState::Write: {
                float mult;
                if (!AP_Param::get("BATT_VOLT_MULT", mult)) {
                    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Could not read BATT_VOLT_MULT");
                }
                if (!AP_Param::set_and_save_by_name("BATT_VOLT_MULT", mult * cal_main_voltage / (cal_measurement_1/cal_measurement_count))) {
                    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Could not write to BATT_VOLT_MULT");
                }
                if (!AP_Param::get("BATT2_VOLT_MULT", mult)) {
                    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Could not read BATT2_VOLT_MULT");
                }
                if (!AP_Param::set_and_save_by_name("BATT2_VOLT_MULT", mult * cal_main_voltage / (cal_measurement_2/cal_measurement_count))) {
                    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Could not write to BATT2_VOLT_MULT");
                }
                if (!AP_Param::get("BATT9_VOLT_MULT", mult)) {
                    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Could not read BATT9_VOLT_MULT");
                }
                if (!AP_Param::set_and_save_by_name("BATT9_VOLT_MULT", mult * cal_main_voltage / (cal_measurement_3/cal_measurement_count))) {
                    GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Could not write to BATT9_VOLT_MULT");
                }
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Main_V_Divider done. VOLT_MULT correction factor - B1: %.4f B2: %.4f Out: %.4f", cal_main_voltage / (cal_measurement_1/cal_measurement_count), cal_main_voltage / (cal_measurement_2/cal_measurement_count), cal_main_voltage / (cal_measurement_3/cal_measurement_count));

                cal_state = CalibrateState::Bat1_Amp_Offset;
                cal_sub_state = CalibrateSubState::Setup;
            break;
        }
        }
        break;

    case  CalibrateState::Bat1_Amp_Offset:
        switch (cal_sub_state) {

        case CalibrateSubState::Setup:
            set_switch_1_off();
            set_LED_1_off();
            if (!AP_Param::set_and_save_by_name("BATT_AMP_OFFSET", 0.0)) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Could not write to BATT2_AMP_OFFSET");
            }
            // Turn on Main
            set_main_on();
            if (main_is_on()) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Calibrating Battery 1 current offset");
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Disconnect B1 and fan");
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Press Main Button to start");
                cal_sub_state = CalibrateSubState::Start;
            }
            break;

        case CalibrateSubState::Start:
            if (switch_1_is_on()) {
                set_LED_1_on();
                cal_measurement_count = 0;
                cal_measurement_1 = 0.0;
                last_update_ms = now_ms;
                cal_sub_state = CalibrateSubState::Measure;
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Measuring BATT_AMP_OFFSET");
            }
            break;

        case CalibrateSubState::Measure:
            if (now_ms - last_update_ms > 5000) {
                set_main_off();
                if (main_is_off()){
                    cal_sub_state = CalibrateSubState::Write;
                }
            } else if (now_ms - last_update_ms > 1000) {
                if (batt.current_amps(payload_current, 0)) {
                    cal_measurement_count += 1;
                    cal_measurement_1 += payload_current;
                }
            }
            break;

        case CalibrateSubState::Write:
            float mult = 0.0;
            if (!AP_Param::get("BATT_AMP_PERVLT", mult)) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Could not read BATT_AMP_PERVLT");
            }
            if (!AP_Param::set_and_save_by_name("BATT_AMP_OFFSET", (cal_measurement_1/cal_measurement_count)/mult)) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Could not write to BATT_AMP_OFFSET");
            }
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Bat1_Amp_Offset done. Current Offset: %.4f", (cal_measurement_1/cal_measurement_count)/mult);
            cal_state = CalibrateState::Bat2_Amp_Offset;
            cal_sub_state = CalibrateSubState::Setup;
            break;
        }
        break;

    case  CalibrateState::Bat2_Amp_Offset:
        switch (cal_sub_state) {

        case CalibrateSubState::Setup:
            set_switch_1_off();
            set_LED_1_off();
            // set BATT2_AMP_OFFSET = 0.0
            if (!AP_Param::set_and_save_by_name("BATT2_AMP_OFFSET", 0.0)) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Could not write to BATT2_AMP_OFFSET");
            }
            set_main_on();
            if (main_is_on()) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Calibrating Battery 2 current offset");
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Connect B1 and Disconnect B2");
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Press Main Button to start");
                cal_sub_state = CalibrateSubState::Start;
            }
            break;

        case CalibrateSubState::Start:
            if (switch_1_is_on()) {
                set_LED_1_on();
                cal_measurement_count = 0;
                cal_measurement_1 = 0.0;
                last_update_ms = now_ms;
                cal_sub_state = CalibrateSubState::Measure;
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Measuring BATT2_AMP_OFFSET");
            }
            break;

        case CalibrateSubState::Measure:
            if (now_ms - last_update_ms > 5000) {
                set_main_off();
                if (main_is_off()){
                    cal_sub_state = CalibrateSubState::Write;
                }
            } else if (now_ms - last_update_ms > 1000) {
                if (batt.current_amps(payload_current, 1)) {
                    cal_measurement_count += 1;
                    cal_measurement_1 += payload_current;
                }
            }
            break;

        case CalibrateSubState::Write:
            float mult = 0.0;
            if (!AP_Param::get("BATT2_AMP_PERVLT", mult)) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Could not read BATT2_AMP_PERVLT");
            }
            if (!AP_Param::set_and_save_by_name("BATT2_AMP_OFFSET", (cal_measurement_1/cal_measurement_count)/mult)) {
                GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Could not write to BATT2_AMP_OFFSET");
            }
            GCS_SEND_TEXT(MAV_SEVERITY_CRITICAL, "Bat2_Amp_Offset done. Current Offset: %.4f", (cal_measurement_1/cal_measurement_count)/mult);
            cal_state = CalibrateState::Finished;
            cal_sub_state = CalibrateSubState::Setup;

            // I would like to move this to CalibrateState::Finished:
            set_main_off();
            set_HV_payload_off();
            set_payload_BEC_1_off();
            set_payload_BEC_2_off();
            set_switch_1_off();
            set_LED_1_off();
            set_LED_debug_off(); 
            // I would like to unset the options bit to stop this running
            break;
        }
        break;

    case  CalibrateState::Finished:
        break;
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

    update_switches();

    update_main_power();

    update_payload_HV_power();

    if (option_is_set(Option::CAL) && cal_state != CalibrateState::Finished) {
        calibrate();
        return;
    }    
    
    if (now_ms - last_update_ms < FSO_LOOP_TIME_MS) {
        // run at 10Hz
        return;
    }
    last_update_ms = now_ms;

    if (switch_1_is_on()) {
        set_LED_1_on();
        set_main_on();
    } else {
        set_LED_1_off();
        set_main_off();
    }

    if (switch_2_is_on()) {
        set_LED_2_on();
        set_HV_payload_on();
        set_payload_BEC_1_on();
        set_payload_BEC_2_on();
    } else {
        set_LED_2_off();
        set_HV_payload_off();
        set_payload_BEC_1_off();
        set_payload_BEC_2_off();
    }

    update_DAC();

    update_fans();

    update_payload_BEC();

    update_internal_BEC();

    report();
}

#endif  // HAL_PERIPH_ENABLE_FSO_POWER_STACK

