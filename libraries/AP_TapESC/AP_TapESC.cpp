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
  driver for TAP UART ESCs, based on AP_HAL/utility/RCOutput_Tap.cpp
 */

#include "AP_TapESC.h"

#if AP_TAPESC_ENABLED

#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

#define TAP_ESC_BAUD 250000
#define TAP_ESC_PACKET_HEAD 0xFE
#define TAP_ESC_CRC_POLYNOMIAL 0xE7

// the ESCs ignore commands sent sooner than this after power on
#define TAP_ESC_MIN_BOOT_TIME_MS 550

// the ESCs need this many all-zero run packets to unlock
#define TAP_ESC_UNLOCK_PACKETS 10

// minimum packet-to-packet time is 1ms
#define TAP_ESC_MIN_PACKET_INTERVAL_US 2000

#define RPM_MAX 1900
#define RPM_MIN 1200
#define RPM_STOPPED (RPM_MIN - 10)

#define RUN_CHANNEL_VALUE_MASK 0x07ff
#define RUN_LED_ON_MASK 0x3800

#define CHANNEL_MAP_CHANNEL 0x0f
#define CHANNEL_MAP_RUNNING_DIRECTION 0xf0

// ESC ids, circular from back right in CCW direction
static const uint8_t device_mux_map[] = {0, 1, 4, 3, 2, 5, 7, 8};
// 0 is CW, 1 is CCW
static const uint8_t device_dir_map[] = {0, 1, 0, 1, 0, 1, 0, 1};

// ESC slot used for each of Motor1 to Motor4
static const uint8_t motor_mapping[] = {2, 1, 0, 3};

void AP_TapESC::init()
{
    AP_SerialManager &serial_manager = AP::serialmanager();
    uart = serial_manager.find_serial(AP_SerialManager::SerialProtocol_TapESC, 0);
    if (uart == nullptr) {
        state = State::NO_PORT;
        return;
    }
    // update baud param in case user looks at it
    serial_manager.set_and_default_baud(AP_SerialManager::SerialProtocol_TapESC, 0, TAP_ESC_BAUD);
    uart->begin(TAP_ESC_BAUD);
    state = State::WAIT_BOOT;
}

void AP_TapESC::send_packet(MsgID msg_id, const void *data, uint8_t len)
{
    uint8_t buf[3 + sizeof(ConfigBasic) + 1];
    if (len > sizeof(buf) - 4) {
        return;
    }
    buf[0] = TAP_ESC_PACKET_HEAD;
    buf[1] = len;
    buf[2] = uint8_t(msg_id);
    memcpy(&buf[3], data, len);
    // CRC covers the length, message ID and data
    buf[3 + len] = crc8_generic(&buf[1], len + 2, TAP_ESC_CRC_POLYNOMIAL);

    const uint8_t packet_len = len + 4;
    if (uart->txspace() < packet_len) {
        return;
    }
    uart->write(buf, packet_len);
    last_send_us = AP_HAL::micros();
}

void AP_TapESC::send_config()
{
    ConfigBasic config {};
    config.max_channel_in_use = NUM_MOTORS;
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        config.channel_map[i] = (device_mux_map[i] & CHANNEL_MAP_CHANNEL) |
            ((device_dir_map[i] << 4) & CHANNEL_MAP_RUNNING_DIRECTION);
    }
    config.control_mode = 1;
    config.min_channel_value = RPM_MIN;
    config.max_channel_value = RPM_MAX;
    send_packet(MsgID::CONFIG_BASIC, &config, sizeof(config));
}

void AP_TapESC::send_run(const uint16_t rpm[NUM_MOTORS])
{
    Run run;
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        run.value[i] = rpm[i];
    }
    send_packet(MsgID::RUN, &run, sizeof(run));
}

/*
  map a motor output onto the ESC's RPM range. Map to [RPM_STOPPED,
  RPM_MAX] rather than [RPM_MIN, RPM_MAX] because AP_Motors sends the
  minimum ESC PWM when it is disarmed
 */
uint16_t AP_TapESC::rpm_for_motor(uint8_t motor) const
{
    if (!hal.util->get_soft_armed()) {
        return RPM_STOPPED;
    }
    uint16_t pwm;
    if (!SRV_Channels::get_output_pwm(SRV_Channels::get_motor_function(motor), pwm)) {
        return RPM_STOPPED;
    }
    const float thrust = 0.5 * (hal.rcout->scale_esc_to_unity(pwm) + 1.0);
    if (thrust <= 0) {
        return RPM_STOPPED;
    }
    if (thrust >= 1) {
        return RPM_MAX;
    }
    return RPM_STOPPED + thrust * (RPM_MAX - RPM_STOPPED);
}

void AP_TapESC::update()
{
    if (state == State::UNINITIALISED) {
        init();
    }
    if (state == State::NO_PORT) {
        return;
    }

    // we do not use the ESC feedback
    uart->discard_input();

    if (AP_HAL::micros() - last_send_us < TAP_ESC_MIN_PACKET_INTERVAL_US) {
        return;
    }

    switch (state) {
    case State::UNINITIALISED:
    case State::NO_PORT:
        return;

    case State::WAIT_BOOT:
        if (AP_HAL::millis() < TAP_ESC_MIN_BOOT_TIME_MS) {
            return;
        }
        send_config();
        state = State::UNLOCKING;
        return;

    case State::UNLOCKING: {
        const uint16_t zero[NUM_MOTORS] {};
        send_run(zero);
        if (++unlock_count >= TAP_ESC_UNLOCK_PACKETS) {
            state = State::RUNNING;
        }
        return;
    }

    case State::RUNNING:
        break;
    }

    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms - last_led_toggle_ms > 250) {
        led_on = !led_on;
        last_led_toggle_ms = now_ms;
    }

    uint16_t rpm[NUM_MOTORS];
    for (uint8_t i = 0; i < NUM_MOTORS; i++) {
        uint16_t value = rpm_for_motor(i) & RUN_CHANNEL_VALUE_MASK;
        if (led_on) {
            value |= RUN_LED_ON_MASK;
        }
        rpm[motor_mapping[i]] = value;
    }
    send_run(rpm);
}

#endif  // AP_TAPESC_ENABLED
