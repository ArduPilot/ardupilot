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

#include "AP_RangeFinder_HI50.h"

#include <AP_HAL/AP_HAL.h>

#include <stdio.h>

extern const AP_HAL::HAL& hal;

void AP_RangeFinder_HI50::update(void)
{
    if (uart == nullptr) {
        return;
    }

    // experiments show this sensor can do ~3Hz:
    if (AP_HAL::millis() - state.last_reading_ms > 500) {
        set_status(RangeFinder::Status::NoData);
    }

    switch (hi50_state) {
    case HI50State::ERROR:
        return;
    case HI50State::CLOSED:
        zero_linebuf();
        if (!uart->discard_input()) {
            set_state(HI50State::ERROR);
            return;
        }
        set_state(HI50State::OPENING_SEND_OPEN);
        break;
    case HI50State::OPENING_SEND_OPEN:
        if (uart->write("O") != 1) {
            return;
        }
        set_state(HI50State::OPENING_WAIT_OK);
        break;
    case HI50State::OPENING_WAIT_OK:
        handle_state_opening_wait_ok();
        break;
    case HI50State::GET_VERSION_START:
        // we sscanf linebuf below, so null-termination is critical:
        zero_linebuf();
        if (uart->write("V") != 1) {
            return;
        }
        set_state(HI50State::WAITING_FOR_VERSION);
        break;
    case HI50State::WAITING_FOR_VERSION:
        handle_state_waiting_for_version();
        break;
    case HI50State::WORK:
        handle_state_work();
        break;
    }
}

void AP_RangeFinder_HI50::handle_state_opening_wait_ok()
{
    if (!get_line_into_linebuf()) {
        return;
    }

    if (!strncmp((char*)linebuf, "O,OK!\r\n", linebuf_len)) {
        set_state(HI50State::GET_VERSION_START);
        return;
    }
    // TODO: redo/recovery rather than this:
    set_state(HI50State::ERROR);
}

void AP_RangeFinder_HI50::handle_state_waiting_for_version()
{
    if (!get_line_into_linebuf()) {
        return;
    }

    uint32_t hwver;
    uint32_t swver;
    // V:19062506866,37953
    if (sscanf((char*)linebuf, "V:%u,%u\r\n", &hwver, &swver) == 2) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RangeFinder: HI50 (%u/%u)", hwver, swver);
        set_state(HI50State::WORK);
        set_workstate(WorkState::START);
    }
}

void AP_RangeFinder_HI50::handle_state_work()
{
    switch (workstate) {
    case WorkState::START:
        set_workstate(WorkState::REQUEST);
        FALLTHROUGH;
    case WorkState::REQUEST:
        set_workstate(WorkState::REQUEST_FAST);
        return;
    case WorkState::REQUEST_AUTO:
        // device automatically selects best type of reading to take
        zero_linebuf();
        if (uart->write("D") != 1) {
            return;
        }
        set_workstate(WorkState::EXPECT_READING);
        return;
    case WorkState::REQUEST_FAST:
        zero_linebuf();
        if (uart->write("F") != 1) {
            return;
        }
        set_workstate(WorkState::EXPECT_READING);
        return;
    case WorkState::REQUEST_SLOW:
        zero_linebuf();
        if (uart->write("M") != 1) {
            return;
        }
        set_workstate(WorkState::EXPECT_READING);
        return;
    case WorkState::EXPECT_READING:
        handle_workstate_expect_reading();
        break;
    }
}

bool AP_RangeFinder_HI50::get_reading(uint16_t &reading_cm)
{
    // not actually called as we override update()
    return false;
}

bool AP_RangeFinder_HI50::fill_linebuf()
{
    // the additional -1 here is to ensure the buffer is always null-terminated
    const uint8_t to_read = MIN(uart->available(), ARRAY_SIZE(linebuf) - 1 - linebuf_len);
    const ssize_t n = uart->read(&linebuf[linebuf_len], to_read);
    if (n == -1) {
        set_state(HI50State::ERROR);
        return false;
    }
    if (n == 0) {
        return false;
    }
    // for (ssize_t i=0; i<n; i++) {
    //     ::fprintf(stderr, "read from device: %02X (%c)\n", linebuf[linebuf_len+i],linebuf[linebuf_len+i]);
    // }
    linebuf_len += n;
    return true;
}

bool AP_RangeFinder_HI50::get_line_into_linebuf()
{
    if (!fill_linebuf()) {
        return false;
    }

    return strstr((char*)linebuf, "\r\n");
}

void AP_RangeFinder_HI50::handle_workstate_expect_reading()
{
    if (!get_line_into_linebuf()) {
        return;
    }

// F: 0.039m,1199
// D: 0.039m,1199
    float distance;
    unsigned quality;
    char result_type;
    if (sscanf((char*)linebuf, "%c: %fm,%u\r\n", &result_type, &distance, &quality) == 3) {
        // GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RangeFinder: HI50 (dist=%fm)", distance);
        if (distance < 32.0f) {
            // very short distances can glitch to >32m
            state.distance_cm = distance * 100.0f;
            state.last_reading_ms = AP_HAL::millis();
        }
        set_workstate(WorkState::REQUEST);
        return;
    }
    unsigned error_code;
    if (sscanf((char*)linebuf, "%c:Er%u!\r\n", &result_type, &error_code) == 2) {
        if (error_code != 6) { // these happen relatively frequently
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "RangeFinder: HI50 (error=%02u)", error_code);
        }
        set_workstate(WorkState::REQUEST);
        return;
    }
    if (linebuf_len >= ARRAY_SIZE(linebuf)-1) {
        // full buffer + no reading....
        set_state(HI50State::ERROR);
        return;
    }
}
