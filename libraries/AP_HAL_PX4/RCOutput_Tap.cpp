/*
 * Copyright (c) 2016  PX4 Development Team. All rights reserved.
 * Copyright (C) 2017  Intel Corporation. All rights reserved.
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

/*
 * Implementation of TAP UART ESCs. Used the implementation from PX4 as a base
 * which is BSD-licensed:
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_PX4 && CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_PX4_AEROFC_V1

#include "RCOutput_Tap.h"

#include <fcntl.h>
#include <stdio.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h>
#include <unistd.h>

#include <drivers/drv_hrt.h>

#include <AP_Math/AP_Math.h>

#define DEBUG 0
#if DEBUG
#define debug(fmt, args...) ::printf(fmt "\n", ##args)
#else
#define debug(fmt, args...)
#endif

#define UART_DEVICE_PATH "/dev/ttyS0"

extern const AP_HAL::HAL &hal;

/****** ESC data types ******/

#define ESC_HAVE_CURRENT_SENSOR
#define MIN_BOOT_TIME_USEC (550 * USEC_PER_MSEC)

static const uint8_t crcTable[256] = {
    0x00, 0xE7, 0x29, 0xCE, 0x52, 0xB5, 0x7B, 0x9C, 0xA4, 0x43, 0x8D, 0x6A,
    0xF6, 0x11, 0xDF, 0x38, 0xAF, 0x48, 0x86, 0x61, 0xFD, 0x1A, 0xD4, 0x33,
    0x0B, 0xEC, 0x22, 0xC5, 0x59, 0xBE, 0x70, 0x97, 0xB9, 0x5E, 0x90, 0x77,
    0xEB, 0x0C, 0xC2, 0x25, 0x1D, 0xFA, 0x34, 0xD3, 0x4F, 0xA8, 0x66, 0x81,
    0x16, 0xF1, 0x3F, 0xD8, 0x44, 0xA3, 0x6D, 0x8A, 0xB2, 0x55, 0x9B, 0x7C,
    0xE0, 0x07, 0xC9, 0x2E, 0x95, 0x72, 0xBC, 0x5B, 0xC7, 0x20, 0xEE, 0x09,
    0x31, 0xD6, 0x18, 0xFF, 0x63, 0x84, 0x4A, 0xAD, 0x3A, 0xDD, 0x13, 0xF4,
    0x68, 0x8F, 0x41, 0xA6, 0x9E, 0x79, 0xB7, 0x50, 0xCC, 0x2B, 0xE5, 0x02,
    0x2C, 0xCB, 0x05, 0xE2, 0x7E, 0x99, 0x57, 0xB0, 0x88, 0x6F, 0xA1, 0x46,
    0xDA, 0x3D, 0xF3, 0x14, 0x83, 0x64, 0xAA, 0x4D, 0xD1, 0x36, 0xF8, 0x1F,
    0x27, 0xC0, 0x0E, 0xE9, 0x75, 0x92, 0x5C, 0xBB, 0xCD, 0x2A, 0xE4, 0x03,
    0x9F, 0x78, 0xB6, 0x51, 0x69, 0x8E, 0x40, 0xA7, 0x3B, 0xDC, 0x12, 0xF5,
    0x62, 0x85, 0x4B, 0xAC, 0x30, 0xD7, 0x19, 0xFE, 0xC6, 0x21, 0xEF, 0x08,
    0x94, 0x73, 0xBD, 0x5A, 0x74, 0x93, 0x5D, 0xBA, 0x26, 0xC1, 0x0F, 0xE8,
    0xD0, 0x37, 0xF9, 0x1E, 0x82, 0x65, 0xAB, 0x4C, 0xDB, 0x3C, 0xF2, 0x15,
    0x89, 0x6E, 0xA0, 0x47, 0x7F, 0x98, 0x56, 0xB1, 0x2D, 0xCA, 0x04, 0xE3,
    0x58, 0xBF, 0x71, 0x96, 0x0A, 0xED, 0x23, 0xC4, 0xFC, 0x1B, 0xD5, 0x32,
    0xAE, 0x49, 0x87, 0x60, 0xF7, 0x10, 0xDE, 0x39, 0xA5, 0x42, 0x8C, 0x6B,
    0x53, 0xB4, 0x7A, 0x9D, 0x01, 0xE6, 0x28, 0xCF, 0xE1, 0x06, 0xC8, 0x2F,
    0xB3, 0x54, 0x9A, 0x7D, 0x45, 0xA2, 0x6C, 0x8B, 0x17, 0xF0, 0x3E, 0xD9,
    0x4E, 0xA9, 0x67, 0x80, 0x1C, 0xFB, 0x35, 0xD2, 0xEA, 0x0D, 0xC3, 0x24,
    0xB8, 0x5F, 0x91, 0x76
};

// Circular from back right in CCW direction
static const uint8_t device_mux_map[] = {0, 1, 4, 3, 2, 5, 7, 8};
// 0 is CW, 1 is CCW
static const uint8_t device_dir_map[] = {0, 1, 0, 1, 0, 1, 0, 1};

#define TAP_ESC_MAX_PACKET_LEN 20
#define TAP_ESC_MAX_MOTOR_NUM 8

/*
 * ESC_POS maps the values stored in the channelMapTable to reorder the ESC's
 * id so that that match the mux setting, so that the ressonder's data
 * will be read.
 * The index on channelMapTable[p] p is the physical ESC
 * The value it is set to is the logical value from ESC_POS[p]
 *  Phy Log
 *  0   0
 *  1   1
 *  2   4
 *  3   3
 *  4   2
 *  5   5
 *   ....
 *
 */

#define RPMMAX 1900
#define RPMMIN 1200
#define RPMSTOPPED (RPMMIN - 10)

#define MIN_BOOT_TIME_MSEC (550) // Minimum time to wait after Power on before sending commands

namespace PX4 {

/****** Run ***********/

#define RUN_CHANNEL_VALUE_MASK (uint16_t)0x07ff
#define RUN_RED_LED_ON_MASK (uint16_t)0x0800
#define RUN_GREEN_LED_ON_MASK (uint16_t)0x1000
#define RUN_BLUE_LED_ON_MASK (uint16_t)0x2000
#define RUN_LED_ON_MASK (uint16_t)0x3800
#define RUN_FEEDBACK_ENABLE_MASK (uint16_t)0x4000
#define RUN_REVERSE_MASK (uint16_t)0x8000

struct PACKED RunReq {
    uint16_t value[TAP_ESC_MAX_MOTOR_NUM];
};

struct PACKED RunInfoRepsonse {
    uint8_t channelID;
    uint8_t ESCStatus;
    int16_t speed; // -32767 - 32768
#if defined(ESC_HAVE_VOLTAGE_SENSOR)
    uint16_t voltage; // 0.00 - 100.00 V
#endif
#if defined(ESC_HAVE_CURRENT_SENSOR)
    uint16_t current; // 0.0 - 200.0 A
#endif
#if defined(ESC_HAVE_TEMPERATURE_SENSOR)
    uint8_t temperature; // 0 - 256 degree celsius
#endif
};
/****** Run ***********/

/****** ConFigInfoBasic ***********/
struct PACKED ConfigInfoBasicRequest {
    uint8_t maxChannelInUse;
    uint8_t channelMapTable[TAP_ESC_MAX_MOTOR_NUM];
    uint8_t monitorMsgType;
    uint8_t controlMode;
    uint16_t minChannelValue;
    uint16_t maxChannelValue;
};

struct PACKED ConfigInfoBasicResponse {
    uint8_t channelID;
    ConfigInfoBasicRequest resp;
};

#define ESC_CHANNEL_MAP_CHANNEL 0x0f
#define ESC_CHANNEL_MAP_RUNNING_DIRECTION 0xf0
/****** ConFigInfoBasicResponse ***********/

/****** InfoRequest  ***********/
enum InfoTypes {
    REQUEST_INFO_BASIC = 0,
    REQUEST_INFO_FUll,
    REQUEST_INFO_RUN,
    REQUEST_INFO_STUDY,
    REQUEST_INFO_COMM,
    REQUEST_INFO_DEVICE,
};

struct PACKED InfoRequest {
    uint8_t channelID;
    uint8_t requestInfoType;
};

/****** InfoRequest ***********/

struct PACKED EscPacket {
    uint8_t head;
    uint8_t len;
    uint8_t msg_id;
    union {
        InfoRequest reqInfo;
        ConfigInfoBasicRequest reqConfigInfoBasic;
        RunReq reqRun;

        ConfigInfoBasicResponse rspConfigInfoBasic;
        RunInfoRepsonse rspRunInfo;
        uint8_t bytes[100];
    } d;
    uint8_t crc_data;
};

//static const unsigned ESC_PACKET_DATA_OFFSET = offsetof(EscPacket, d);
static const unsigned ESC_PACKET_DATA_OFFSET = 3;

/******************************************************************************************
 * ESCBUS_MSG_ID_RUN_INFO packet
 *
 * Monitor message of ESCs while motor is running
 *
 * channelID: assigned channel number
 *
 * ESCStatus: status of ESC
 *  Num		Health status
 *  0		HEALTHY
 *  1		WARNING_LOW_VOLTAGE
 *  2		WARNING_OVER_CURRENT
 *  3		WARNING_OVER_HEAT
 *  4		ERROR_MOTOR_LOW_SPEED_LOSE_STEP
 *  5		ERROR_MOTOR_STALL
 *  6		ERROR_HARDWARE
 *  7		ERROR_LOSE_PROPELLER
 *  8		ERROR_OVER_CURRENT
 *
 * speed: -32767 - 32767 rpm
 *
 * temperature: 0 - 256 celsius degree (if available)
 * voltage: 0.00 - 100.00 V (if available)
 * current: 0.0 - 200.0 A (if available)
 */

enum ESCBUS_ENUM_ESC_STATUS {
    ESC_STATUS_HEALTHY,
    ESC_STATUS_WARNING_LOW_VOLTAGE,
    ESC_STATUS_WARNING_OVER_HEAT,
    ESC_STATUS_ERROR_MOTOR_LOW_SPEED_LOSE_STEP,
    ESC_STATUS_ERROR_MOTOR_STALL,
    ESC_STATUS_ERROR_HARDWARE,
    ESC_STATUS_ERROR_LOSE_PROPELLER,
    ESC_STATUS_ERROR_OVER_CURRENT,
    ESC_STATUS_ERROR_MOTOR_HIGH_SPEED_LOSE_STEP,
    ESC_STATUS_ERROR_LOSE_CMD,
};

enum ESCBUS_ENUM_MESSAGE_ID {
    // messages or command to ESC
    ESCBUS_MSG_ID_CONFIG_BASIC = 0,
    ESCBUS_MSG_ID_CONFIG_FULL,
    ESCBUS_MSG_ID_RUN,
    ESCBUS_MSG_ID_TUNE,
    ESCBUS_MSG_ID_DO_CMD,
    // messages from ESC
    ESCBUS_MSG_ID_REQUEST_INFO,
    ESCBUS_MSG_ID_CONFIG_INFO_BASIC, // simple configuration info for request from flight controller
    ESCBUS_MSG_ID_CONFIG_INFO_FULL, // full configuration info for request from host such as computer
    ESCBUS_MSG_ID_RUN_INFO, // feedback message in RUN mode
    ESCBUS_MSG_ID_STUDY_INFO, // studied parameters in STUDY mode
    ESCBUS_MSG_ID_COMM_INFO, // communication method info
    ESCBUS_MSG_ID_DEVICE_INFO, // ESC device info
    ESCBUS_MSG_ID_ASSIGNED_ID, // never touch ESCBUS_MSG_ID_MAX_NUM
    //boot loader used
    PROTO_OK = 0x10, // INSYNC/OK - 'ok' response
    PROTO_FAILED = 0x11, // INSYNC/FAILED - 'fail' response

    ESCBUS_MSG_ID_BOOT_SYNC = 0x21, // boot loader used
    PROTO_GET_DEVICE = 0x22, // get device ID bytes
    PROTO_CHIP_ERASE = 0x23, // erase program area and reset program address
    PROTO_PROG_MULTI = 0x27, // write bytes at program address and increment
    PROTO_GET_CRC = 0x29, // compute & return a CRC
    PROTO_BOOT = 0x30, // boot the application
    PROTO_GET_SOFTWARE_VERSION = 0x40,
    ESCBUS_MSG_ID_MAX_NUM,
};

enum PARSR_ESC_STATE {
    HEAD,
    LEN,
    ID,
    DATA,
    CRC,
};

/****************************/
}

using namespace PX4;

void RCOutput_Tap::_uart_close()
{
    if (_uart_fd < 0) {
        return;
    }
    ::close(_uart_fd);
    _uart_fd = -1;
}

bool RCOutput_Tap::_uart_open()
{
    // open uart
    _uart_fd = open(UART_DEVICE_PATH, O_RDWR | O_NOCTTY | O_NONBLOCK);
    int termios_state = -1;

    if (_uart_fd < 0) {
        PX4_ERR("failed to open uart device!");
        return -1;
    }

    // set baud rate
    int speed = 250000;
    struct termios uart_config;
    tcgetattr(_uart_fd, &uart_config);

    // clear ONLCR flag (which appends a CR for every LF)
    uart_config.c_oflag &= ~ONLCR;

    // set baud rate
    if (cfsetispeed(&uart_config, speed) < 0 || cfsetospeed(&uart_config, speed) < 0) {
        ::fprintf(stderr, "failed to set baudrate for %s: %d\n",
                  UART_DEVICE_PATH, termios_state);
        _uart_close();
        return false;
    }

    if ((termios_state = tcsetattr(_uart_fd, TCSANOW, &uart_config)) < 0) {
        fprintf(stderr, "tcsetattr failed for %s\n", UART_DEVICE_PATH);
        _uart_close();
        return false;
    }

    return true;
}

void RCOutput_Tap::init()
{
    _perf_rcout = perf_alloc(PC_ELAPSED, "APM_rcout");

    if (!_uart_open()) {
        AP_HAL::panic("Unable to open " UART_DEVICE_PATH);
        return;
    }

    hrt_abstime uptime_usec = hrt_absolute_time();
    if (uptime_usec < MIN_BOOT_TIME_USEC) {
        hal.scheduler->delay((MIN_BOOT_TIME_USEC - uptime_usec) / USEC_PER_MSEC);
    }

    /* Issue Basic Config */

    EscPacket packet = {0xfe, sizeof(ConfigInfoBasicRequest), ESCBUS_MSG_ID_CONFIG_BASIC};
    ConfigInfoBasicRequest &config = packet.d.reqConfigInfoBasic;
    memset(&config, 0, sizeof(ConfigInfoBasicRequest));
    config.maxChannelInUse = _channels_count;
    config.controlMode = 1;

    /* Assign the id's to the ESCs to match the mux */
    for (uint8_t phy_chan_index = 0; phy_chan_index < _channels_count; phy_chan_index++) {
        config.channelMapTable[phy_chan_index] = device_mux_map[phy_chan_index] & ESC_CHANNEL_MAP_CHANNEL;
        config.channelMapTable[phy_chan_index] |= (device_dir_map[phy_chan_index] << 4) & ESC_CHANNEL_MAP_RUNNING_DIRECTION;
    }

    config.maxChannelValue = RPMMAX;
    config.minChannelValue = RPMMIN;

    int ret = _send_packet(packet);
    if (ret < 0) {
        _uart_close();
        AP_HAL::panic("Unable to send configuration to " UART_DEVICE_PATH);
        return;
    }

    /*
     * To Unlock the ESC from the Power up state we need to issue 10
	 * ESCBUS_MSG_ID_RUN request with all the values 0;
	 */
    EscPacket unlock_packet = {0xfe, _channels_count, ESCBUS_MSG_ID_RUN};
    unlock_packet.len *= sizeof(unlock_packet.d.reqRun.value[0]);
    memset(unlock_packet.d.bytes, 0, sizeof(packet.d.bytes));

    for (uint8_t i = 0; i < 10; i++) {
        _send_packet(unlock_packet);
        /* Min Packet to Packet time is 1 Ms so use 2 */
        hal.scheduler->delay(2);
    }
}

int RCOutput_Tap::_send_packet(EscPacket &packet)
{
    int packet_len = _crc_packet(packet);
    int ret = ::write(_uart_fd, &packet.head, packet_len);

    if (ret != packet_len) {
        debug("TX ERROR: ret: %d, errno: %d", ret, errno);
    }

    return ret;
}

uint8_t RCOutput_Tap::_crc8_esc(uint8_t *p, uint8_t len)
{
    uint8_t crc = 0;

    for (uint8_t i = 0; i < len; i++) {
        crc = crcTable[crc ^ *p++];
    }

    return crc;
}

uint8_t RCOutput_Tap::_crc_packet(EscPacket &p)
{
    /* Calculate the crc over Len,ID,data */
    p.d.bytes[p.len] = _crc8_esc(&p.len, p.len + 2);
    return p.len + ESC_PACKET_DATA_OFFSET + 1;
}

/*
  set output frequency
 */
void RCOutput_Tap::set_freq(uint32_t chmask, uint16_t freq_hz)
{
}

uint16_t RCOutput_Tap::get_freq(uint8_t ch)
{
    return 400;
}

void RCOutput_Tap::enable_ch(uint8_t ch)
{
    if (ch >= MAX_MOTORS) {
        return;
    }
    _enabled_channels |= (1U << ch);
}

void RCOutput_Tap::disable_ch(uint8_t ch)
{
    if (ch >= MAX_MOTORS) {
        return;
    }

    _enabled_channels &= ~(1U << ch);
}

void RCOutput_Tap::write(uint8_t ch, uint16_t period_us)
{
    if (ch >= MAX_MOTORS) {
        return;
    }
    if (!(_enabled_channels & (1U << ch))) {
        // not enabled
        return;
    }

    _period[ch] = period_us;

    if (!_corking) {
        push();
    }
}

uint16_t RCOutput_Tap::read(uint8_t ch)
{
    if (ch >= MAX_MOTORS) {
        return 0;
    }
    return _period[ch];
}

void RCOutput_Tap::read(uint16_t *period_us, uint8_t len)
{
    for (uint8_t i = 0; i < len; i++) {
        period_us[i] = read(i);
    }
}

void RCOutput_Tap::cork()
{
    _corking = true;
}

void RCOutput_Tap::push()
{
    _corking = false;

    perf_begin(_perf_rcout);

    uint16_t out[TAP_ESC_MAX_MOTOR_NUM];
    uint8_t motor_cnt = _channels_count;

    uint8_t motor_mapping[] = {
            [0] = 2,
            [1] = 1,
            [2] = 0,
            [3] = 3,
    };

    // map from the RPM range to 0 - 100% duty cycle for the ESCs
    for (uint8_t i = 0; i < motor_cnt; i++) {
        uint16_t *val = &out[motor_mapping[i]];

        if (!(_enabled_channels & (1U << i))) {
            *val = RPMSTOPPED;
        } else if (_period[i] < _esc_pwm_min) {
            *val = RPMSTOPPED;
        } else if (_period[i] >= _esc_pwm_max) {
            *val = RPMMAX;
        } else {
            float period_us = constrain_int16(_period[i], _esc_pwm_min, _esc_pwm_max);

            /*
             * Map to [ RPMSTOPPED, RPMMAX ] range rather than
             * [ RPMMIN, RPMMAX ] because AP_Motors will send us _esc_pwm_min
             * when it's disarmed
             */
            float rpm = (period_us - _esc_pwm_min)/(_esc_pwm_max - _esc_pwm_min)
                        * (RPMMAX - RPMSTOPPED) + RPMSTOPPED;

            *val = (uint16_t) rpm;
        }
    }
    for (uint8_t i = motor_cnt; i < TAP_ESC_MAX_MOTOR_NUM; i++) {
        out[i] = RPMSTOPPED;
    }

/*
     * Value packet format, little endian
     *
     * | 15 | 14 | 13 | 12 | 11  | 10  | 11 | 12 | 11 | 10 ...... 0 |
     * --------------------------------------------------------------
     * | -- | -- | -- | -- | REV | FEN | BL | GL | RL |  RPM value  |
     *
     * RPM value: [ RPMMIN, RPMMAX ]
     * RL: LED1
     * GL: LED2  (ESC may have only one LED that works, independent of the color)
     * BL: LED3
     * FEN: Feedback enable
     * REV: Reverse direction
     */

// TODO: enable feedback from 1 ESC and read data back
#if 0
	out[_next_channel_reply] |= RUN_FEEDBACK_ENABLE_MASK;
#endif

    EscPacket packet = {0xfe, _channels_count, ESCBUS_MSG_ID_RUN};
    packet.len *= sizeof(packet.d.reqRun.value[0]);

    for (uint8_t i = 0; i < _channels_count; i++) {
        packet.d.reqRun.value[i] = out[i] & RUN_CHANNEL_VALUE_MASK;
    }

    int ret = _send_packet(packet);
    if (ret < 1) {
        debug("TX ERROR: ret: %d, errno: %d", ret, errno);
    }

    _next_channel_reply = (_next_channel_reply + 1) % _channels_count;

    perf_end(_perf_rcout);
}

#endif
