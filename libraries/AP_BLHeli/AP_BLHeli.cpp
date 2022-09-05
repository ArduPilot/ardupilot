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
  implementation of MSP and BLHeli-4way protocols for pass-through ESC
  calibration and firmware update

  With thanks to betaflight for a great reference
  implementation. Several of the functions below are based on
  betaflight equivalent functions
 */

#include "AP_BLHeli.h"

#ifdef HAVE_AP_BLHELI_SUPPORT

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <hal.h>
#endif

#include <AP_Math/crc.h>
#include <AP_Vehicle/AP_Vehicle.h>
#if APM_BUILD_TYPE(APM_BUILD_Rover)
#include <AR_Motors/AP_MotorsUGV.h>
#else
#include <AP_Motors/AP_Motors_Class.h>
#endif
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

#define debug(fmt, args ...) do { if (debug_level) { GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ESC: " fmt, ## args); } } while (0)

// key for locking UART for exclusive use. This prevents any other writes from corrupting
// the MSP protocol on hal.console
#define BLHELI_UART_LOCK_KEY 0x20180402

// if no packets are received for this time and motor control is active BLH will disconect (stoping motors)
#define MOTOR_ACTIVE_TIMEOUT 1000

const AP_Param::GroupInfo AP_BLHeli::var_info[] = {
    // @Param: MASK
    // @DisplayName: BLHeli Channel Bitmask
    // @Description: Enable of BLHeli pass-thru servo protocol support to specific channels. This mask is in addition to motors enabled using SERVO_BLH_AUTO (if any)
    // @Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16, 16:Channel 17, 17: Channel 18, 18: Channel 19, 19: Channel 20, 20: Channel 21, 21: Channel 22, 22: Channel 23, 23: Channel 24, 24: Channel 25, 25: Channel 26, 26: Channel 27, 27: Channel 28, 28: Channel 29, 29: Channel 30, 30: Channel 31, 31: Channel 32
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("MASK",  1, AP_BLHeli, channel_mask, 0),

#if APM_BUILD_COPTER_OR_HELI || APM_BUILD_TYPE(APM_BUILD_ArduPlane) || APM_BUILD_TYPE(APM_BUILD_Rover)
    // @Param: AUTO
    // @DisplayName: BLHeli pass-thru auto-enable for multicopter motors
    // @Description: If set to 1 this auto-enables BLHeli pass-thru support for all multicopter motors
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("AUTO",  2, AP_BLHeli, channel_auto, 0),
#endif

    // @Param: TEST
    // @DisplayName: BLHeli internal interface test
    // @Description: Setting SERVO_BLH_TEST to a motor number enables an internal test of the BLHeli ESC protocol to the corresponding ESC. The debug output is displayed on the USB console.
    // @Values: 0:Disabled,1:TestMotor1,2:TestMotor2,3:TestMotor3,4:TestMotor4,5:TestMotor5,6:TestMotor6,7:TestMotor7,8:TestMotor8
    // @User: Advanced
    AP_GROUPINFO("TEST",  3, AP_BLHeli, run_test, 0),

    // @Param: TMOUT
    // @DisplayName: BLHeli protocol timeout
    // @Description: This sets the inactivity timeout for the BLHeli protocol in seconds. If no packets are received in this time normal MAVLink operations are resumed. A value of 0 means no timeout
    // @Units: s
    // @Range: 0 300
    // @User: Standard
    AP_GROUPINFO("TMOUT",  4, AP_BLHeli, timeout_sec, 0),

    // @Param: TRATE
    // @DisplayName: BLHeli telemetry rate
    // @Description: This sets the rate in Hz for requesting telemetry from ESCs. It is the rate per ESC. Setting to zero disables telemetry requests
    // @Units: Hz
    // @Range: 0 500
    // @User: Standard
    AP_GROUPINFO("TRATE",  5, AP_BLHeli, telem_rate, 10),

    // @Param: DEBUG
    // @DisplayName: BLHeli debug level
    // @Description: When set to 1 this enabled verbose debugging output over MAVLink when the blheli protocol is active. This can be used to diagnose failures.
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("DEBUG",  6, AP_BLHeli, debug_level, 0),

    // @Param: OTYPE
    // @DisplayName: BLHeli output type override
    // @Description: When set to a non-zero value this overrides the output type for the output channels given by SERVO_BLH_MASK. This can be used to enable DShot on outputs that are not part of the multicopter motors group.
    // @Values: 0:None,1:OneShot,2:OneShot125,3:Brushed,4:DShot150,5:DShot300,6:DShot600,7:DShot1200
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("OTYPE",  7, AP_BLHeli, output_type, 0),

    // @Param: PORT
    // @DisplayName: Control port
    // @Description: This sets the mavlink channel to use for blheli pass-thru. The channel number is determined by the number of serial ports configured to use mavlink. So 0 is always the console, 1 is the next serial port using mavlink, 2 the next after that and so on.
    // @Values: 0:Console,1:Mavlink Serial Channel1,2:Mavlink Serial Channel2,3:Mavlink Serial Channel3,4:Mavlink Serial Channel4,5:Mavlink Serial Channel5
    // @User: Advanced
    AP_GROUPINFO("PORT",  8, AP_BLHeli, control_port, 0),

    // @Param: POLES
    // @DisplayName: BLHeli Motor Poles
    // @Description: This allows calculation of true RPM from ESC's eRPM. The default is 14.
    // @Range: 1 127
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("POLES",  9, AP_BLHeli, motor_poles, 14),

    // @Param: 3DMASK
    // @DisplayName: BLHeli bitmask of 3D channels
    // @Description: Mask of channels which are dynamically reversible. This is used to configure ESCs in '3D' mode, allowing for the motor to spin in either direction
    // @Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16, 16:Channel 17, 17: Channel 18, 18: Channel 19, 19: Channel 20, 20: Channel 21, 21: Channel 22, 22: Channel 23, 23: Channel 24, 24: Channel 25, 25: Channel 26, 26: Channel 27, 27: Channel 28, 28: Channel 29, 29: Channel 30, 30: Channel 31, 31: Channel 32
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("3DMASK",  10, AP_BLHeli, channel_reversible_mask, 0),

#ifdef HAL_WITH_BIDIR_DSHOT
    // @Param: BDMASK
    // @DisplayName: BLHeli bitmask of bi-directional dshot channels
    // @Description: Mask of channels which support bi-directional dshot. This is used for ESCs which have firmware that supports bi-directional dshot allowing fast rpm telemetry values to be returned for the harmonic notch.
    // @Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16, 16:Channel 17, 17: Channel 18, 18: Channel 19, 19: Channel 20, 20: Channel 21, 21: Channel 22, 22: Channel 23, 23: Channel 24, 24: Channel 25, 25: Channel 26, 26: Channel 27, 27: Channel 28, 28: Channel 29, 29: Channel 30, 30: Channel 31, 31: Channel 32
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("BDMASK",  11, AP_BLHeli, channel_bidir_dshot_mask, 0),
#endif
    // @Param: RVMASK
    // @DisplayName: BLHeli bitmask of reversed channels
    // @Description: Mask of channels which are reversed. This is used to configure ESCs in reversed mode
    // @Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16, 16:Channel 17, 17: Channel 18, 18: Channel 19, 19: Channel 20, 20: Channel 21, 21: Channel 22, 22: Channel 23, 23: Channel 24, 24: Channel 25, 25: Channel 26, 26: Channel 27, 27: Channel 28, 28: Channel 29, 29: Channel 30, 30: Channel 31, 31: Channel 32
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("RVMASK",  12, AP_BLHeli, channel_reversed_mask, 0),

    AP_GROUPEND
};

#define RPM_SLEW_RATE 50

AP_BLHeli *AP_BLHeli::_singleton;

// constructor
AP_BLHeli::AP_BLHeli(void)
{
    // set defaults from the parameter table
    AP_Param::setup_object_defaults(this, var_info);
    _singleton = this;
    last_control_port = -1;
}

/*
  process one byte of serial input for MSP protocol
 */
bool AP_BLHeli::msp_process_byte(uint8_t c)
{
    if (msp.state == MSP_IDLE) {
        msp.escMode = PROTOCOL_NONE;
        if (c == '$') {
            msp.state = MSP_HEADER_START;
        } else {
            return false;
        }
    } else if (msp.state == MSP_HEADER_START) {
        msp.state = (c == 'M') ? MSP_HEADER_M : MSP_IDLE;
    } else if (msp.state == MSP_HEADER_M) {
        msp.state = MSP_IDLE;
        switch (c) {
            case '<': // COMMAND
                msp.packetType = MSP_PACKET_COMMAND;
                msp.state = MSP_HEADER_ARROW;
                break;
            case '>': // REPLY
                msp.packetType = MSP_PACKET_REPLY;
                msp.state = MSP_HEADER_ARROW;
                break;
            default:
                break;
        }
    } else if (msp.state == MSP_HEADER_ARROW) {
        if (c > sizeof(msp.buf)) {
            msp.state = MSP_IDLE;
        } else {
            msp.dataSize = c;
            msp.offset = 0;
            msp.checksum = 0;
            msp.checksum ^= c;
            msp.state = MSP_HEADER_SIZE;
        }
    } else if (msp.state == MSP_HEADER_SIZE) {
        msp.cmdMSP = c;
        msp.checksum ^= c;
        msp.state = MSP_HEADER_CMD;
    } else if (msp.state == MSP_HEADER_CMD && msp.offset < msp.dataSize) {
        msp.checksum ^= c;
        msp.buf[msp.offset++] = c;
    } else if (msp.state == MSP_HEADER_CMD && msp.offset >= msp.dataSize) {
        if (msp.checksum == c) {
            msp.state = MSP_COMMAND_RECEIVED;
        } else {
            msp.state = MSP_IDLE;
        }
    }
    return true;
}

/*
  update CRC state for blheli protocol
 */
void AP_BLHeli::blheli_crc_update(uint8_t c)
{
    blheli.crc = crc_xmodem_update(blheli.crc, c);
}

/*
  process one byte of serial input for blheli 4way protocol
 */
bool AP_BLHeli::blheli_4way_process_byte(uint8_t c)
{
    if (blheli.state == BLHELI_IDLE) {
        if (c == cmd_Local_Escape) {
            blheli.state = BLHELI_HEADER_START;
            blheli.crc = 0;
            blheli_crc_update(c);
        } else {
            return false;
        }
    } else if (blheli.state == BLHELI_HEADER_START) {
        blheli.command = c;
        blheli_crc_update(c);
        blheli.state = BLHELI_HEADER_CMD;
    } else if (blheli.state == BLHELI_HEADER_CMD) {
        blheli.address = c<<8;
        blheli.state = BLHELI_HEADER_ADDR_HIGH;
        blheli_crc_update(c);
    } else if (blheli.state == BLHELI_HEADER_ADDR_HIGH) {
        blheli.address |= c;
        blheli.state = BLHELI_HEADER_ADDR_LOW;
        blheli_crc_update(c);
    } else if (blheli.state == BLHELI_HEADER_ADDR_LOW) {
        blheli.state = BLHELI_HEADER_LEN;
        blheli.param_len = c?c:256;
        blheli.offset = 0;
        blheli_crc_update(c);
    } else if (blheli.state == BLHELI_HEADER_LEN) {
        blheli.buf[blheli.offset++] = c;
        blheli_crc_update(c);
        if (blheli.offset == blheli.param_len) {
            blheli.state = BLHELI_CRC1;
        }
    } else if (blheli.state == BLHELI_CRC1) {
        blheli.crc1 = c;
        blheli.state = BLHELI_CRC2;
    } else if (blheli.state == BLHELI_CRC2) {
        uint16_t crc = blheli.crc1<<8 | c;
        if (crc == blheli.crc) {
            blheli.state = BLHELI_COMMAND_RECEIVED;
        } else {
            blheli.state = BLHELI_IDLE;
        }
    }
    return true;
}


/*
  send a MSP protocol ack
 */
void AP_BLHeli::msp_send_ack(uint8_t cmd)
{
    msp_send_reply(cmd, 0, 0);
}

/*
  send a MSP protocol reply
 */
void AP_BLHeli::msp_send_reply(uint8_t cmd, const uint8_t *buf, uint8_t len)
{
    uint8_t *b = &msp.buf[0];
    *b++ = '$';
    *b++ = 'M';
    *b++ = '>';
    *b++ = len;
    *b++ = cmd;
    // acks do not have a payload
    if (len > 0) {
        memcpy(b, buf, len);
    }
    b += len;
    uint8_t c = 0;
    for (uint8_t i=0; i<len+2; i++) {
        c ^= msp.buf[i+3];
    }
    *b++ = c;
    uart->write_locked(&msp.buf[0], len+6, BLHELI_UART_LOCK_KEY);
}

void AP_BLHeli::putU16(uint8_t *b, uint16_t v)
{
    b[0] = v;
    b[1] = v >> 8;
}

uint16_t AP_BLHeli::getU16(const uint8_t *b)
{
    return b[0] | (b[1]<<8);
}

void AP_BLHeli::putU32(uint8_t *b, uint32_t v)
{
    b[0] = v;
    b[1] = v >> 8;
    b[2] = v >> 16;
    b[3] = v >> 24;
}

void AP_BLHeli::putU16_BE(uint8_t *b, uint16_t v)
{
    b[0] = v >> 8;
    b[1] = v;
}

/*
  process a MSP command from GCS
 */
void AP_BLHeli::msp_process_command(void)
{
    debug("MSP cmd %u len=%u", msp.cmdMSP, msp.dataSize);
    switch (msp.cmdMSP) {
    case MSP_API_VERSION: {
        debug("MSP_API_VERSION");
        uint8_t buf[3] = { MSP_PROTOCOL_VERSION, API_VERSION_MAJOR, API_VERSION_MINOR };
        msp_send_reply(msp.cmdMSP, buf, sizeof(buf));
        break;
    }

    case MSP_FC_VARIANT:
        debug("MSP_FC_VARIANT");
        msp_send_reply(msp.cmdMSP, (const uint8_t *)ARDUPILOT_IDENTIFIER, FLIGHT_CONTROLLER_IDENTIFIER_LENGTH);
        break;

    /*
      Notes:
        version 3.3.1 adds a reply to MSP_SET_MOTOR which was missing
        version 3.3.0 requires a workaround in blheli suite to handle MSP_SET_MOTOR without an ack
    */
    case MSP_FC_VERSION: {
        debug("MSP_FC_VERSION");
        uint8_t version[3] = { 3, 3, 1 };
        msp_send_reply(msp.cmdMSP, version, sizeof(version));
        break;
    }
    case MSP_BOARD_INFO: {
        debug("MSP_BOARD_INFO");
        // send a generic 'ArduPilot ChibiOS' board type
        uint8_t buf[7] = { 'A', 'R', 'C', 'H', 0, 0, 0 };
        msp_send_reply(msp.cmdMSP, buf, sizeof(buf));
        break;
    }

    case MSP_BUILD_INFO: {
        debug("MSP_BUILD_INFO");
         // build date, build time, git version
        uint8_t buf[26] {
                0x4d, 0x61, 0x72, 0x20, 0x31, 0x36, 0x20, 0x32, 0x30,
                0x31, 0x38, 0x30, 0x38, 0x3A, 0x34, 0x32, 0x3a, 0x32, 0x39,
                0x62, 0x30, 0x66, 0x66, 0x39, 0x32, 0x38};
        msp_send_reply(msp.cmdMSP, buf, sizeof(buf));
        break;
    }

    case MSP_REBOOT:
        debug("MSP: ignoring reboot command, end serial comms");
        hal.rcout->serial_end();
        blheli.connected[blheli.chan] = false;
        serial_start_ms = 0;
        break;

    case MSP_UID:
        // MCU identifer
        debug("MSP_UID");
        msp_send_reply(msp.cmdMSP, (const uint8_t *)UDID_START, 12);
        break;

    case MSP_ADVANCED_CONFIG: {
        debug("MSP_ADVANCED_CONFIG");
        uint8_t buf[10];
        buf[0] = 1; // gyro sync denom
        buf[1] = 4; // pid process denom
        buf[2] = 0; // use unsynced pwm
        buf[3] = (uint8_t)PWM_TYPE_DSHOT150; // motor PWM protocol
        putU16(&buf[4], 480); // motor PWM Rate
        putU16(&buf[6], 450); // idle offset value
        buf[8] = 0; // use 32kHz
        buf[9] = 0; // motor PWM inversion
        msp_send_reply(msp.cmdMSP, buf, sizeof(buf));
        break;
    }

    case MSP_FEATURE_CONFIG: {
        debug("MSP_FEATURE_CONFIG");
        uint8_t buf[4];
        putU32(buf, (channel_reversible_mask.get() != 0) ? FEATURE_3D : 0); // from MSPFeatures enum
        msp_send_reply(msp.cmdMSP, buf, sizeof(buf));
        break;
    }

    case MSP_STATUS: {
        debug("MSP_STATUS");
        uint8_t buf[21];
        putU16(&buf[0], 1000); // loop time usec
        putU16(&buf[2], 0);    // i2c error count
        putU16(&buf[4], 0x27); // available sensors
        putU32(&buf[6], 0);    // flight modes
        buf[10] = 0;           // pid profile index
        putU16(&buf[11], 5);   // system load percent
        putU16(&buf[13], 0);   // gyro cycle time
        buf[15] = 0;           // flight mode flags length
        buf[16] = 18;          // arming disable flags count
        putU32(&buf[17], 0);   // arming disable flags
        msp_send_reply(msp.cmdMSP, buf, sizeof(buf));
        break;
    }

    case MSP_MOTOR_3D_CONFIG: {
        debug("MSP_MOTOR_3D_CONFIG");
        uint8_t buf[6];
        putU16(&buf[0], 1406); // 3D deadband low
        putU16(&buf[2], 1514); // 3D deadband high
        putU16(&buf[4], 1460); // 3D neutral
        msp_send_reply(msp.cmdMSP, buf, sizeof(buf));
        break;
    }

    case MSP_MOTOR_CONFIG: {
        debug("MSP_MOTOR_CONFIG");
        uint8_t buf[10];
        putU16(&buf[0], 1030); // min throttle
        putU16(&buf[2], 2000); // max throttle
        putU16(&buf[4], 1000); // min command
        // API 1.42
        buf[6] = num_motors; // motorCount
        buf[7] = motor_poles; // motorPoleCount
        buf[8] = 0; // useDshotTelemetry
        buf[9] = 0; // FEATURE_ESC_SENSOR
        msp_send_reply(msp.cmdMSP, buf, sizeof(buf));
        break;
    }

    case MSP_MOTOR: {
        debug("MSP_MOTOR");
        // get the output going to each motor
        uint8_t buf[16] {};
        for (uint8_t i = 0; i < num_motors; i++) {
            // if we have a mix of reversible and normal report a PWM of zero, this allows BLHeliSuite to conect
            uint16_t v = mixed_type ? 0 : hal.rcout->read(motor_map[i]);
            putU16(&buf[2*i], v);
            debug("MOTOR %u val: %u",i,v);
        }
        msp_send_reply(msp.cmdMSP, buf, sizeof(buf));
        break;
    }

    case MSP_SET_MOTOR: {
        debug("MSP_SET_MOTOR");
        if (!mixed_type) {
            // set the output to each motor
            uint8_t nmotors = msp.dataSize / 2;
            debug("MSP_SET_MOTOR %u", nmotors);
            motors_disabled_mask = SRV_Channels::get_disabled_channel_mask();
            SRV_Channels::set_disabled_channel_mask(0xFFFF);
            motors_disabled = true;
            EXPECT_DELAY_MS(1000);
            hal.rcout->cork();
            for (uint8_t i = 0; i < nmotors; i++) {
                if (i >= num_motors) {
                    break;
                }
                uint16_t v = getU16(&msp.buf[i*2]);
                debug("MSP_SET_MOTOR %u %u", i, v);
                // map from a MSP value to a value in the range 1000 to 2000
                uint16_t pwm = (v < 1000)?0:v;
                hal.rcout->write(motor_map[i], pwm);
            }
            hal.rcout->push();
        } else {
            debug("mixed type, Motors Disabled");
        }
        msp_send_ack(msp.cmdMSP);
        break;
    }

    case MSP_SET_PASSTHROUGH: {
        debug("MSP_SET_PASSTHROUGH");
        if (msp.dataSize == 0) {
            msp.escMode = PROTOCOL_4WAY;
        } else if (msp.dataSize == 2) {
            msp.escMode = (enum escProtocol)msp.buf[0];
            msp.portIndex = msp.buf[1];
        }
        debug("escMode=%u portIndex=%u num_motors=%u", msp.escMode, msp.portIndex, num_motors);
        uint8_t n = num_motors;
        switch (msp.escMode) {
        case PROTOCOL_4WAY:
            break;
        default:
            n = 0;
            hal.rcout->serial_end();
            serial_start_ms = 0;
            break;
        }
        // doing the serial setup here avoids delays when doing it on demand and makes
        // BLHeliSuite considerably more reliable
        EXPECT_DELAY_MS(1000);
        if (!hal.rcout->serial_setup_output(motor_map[0], 19200, motor_mask)) {
            msp_send_ack(ACK_D_GENERAL_ERROR);
            break;
        } else {
            msp_send_reply(msp.cmdMSP, &n, 1);
        }
        break;
    }
    default:
        debug("Unknown MSP command %u", msp.cmdMSP);
        break;
    }
}

/*
  send a blheli 4way protocol reply
 */
void AP_BLHeli::blheli_send_reply(const uint8_t *buf, uint16_t len)
{
    uint8_t *b = &blheli.buf[0];
    *b++ = cmd_Remote_Escape;
    *b++ = blheli.command;
    putU16_BE(b, blheli.address); b += 2;
    *b++ = len==256?0:len;
    memcpy(b, buf, len);
    b += len;
    *b++ = blheli.ack;
    putU16_BE(b, crc_xmodem(&blheli.buf[0], len+6));
    uart->write_locked(&blheli.buf[0], len+8, BLHELI_UART_LOCK_KEY);
    debug("OutB(%u) 0x%02x ack=0x%02x", len+8, (unsigned)blheli.command, blheli.ack);
}

/*
  CRC used when talking to ESCs
 */
uint16_t AP_BLHeli::BL_CRC(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0;
    while (len--) {
        uint8_t xb = *buf++;
        for (uint8_t i = 0; i < 8; i++) {
            if (((xb & 0x01) ^ (crc & 0x0001)) !=0 ) {
                crc = crc >> 1;
                crc = crc ^ 0xA001;
            } else {
                crc = crc >> 1;
            }
            xb = xb >> 1;
        }
    }
    return crc;
}

bool AP_BLHeli::isMcuConnected(void)
{
    return blheli.connected[blheli.chan];
}

void AP_BLHeli::setDisconnected(void)
{
    blheli.connected[blheli.chan] = false;
    blheli.deviceInfo[blheli.chan][0] = 0;
    blheli.deviceInfo[blheli.chan][1] = 0;
}

/*
  send a set of bytes to an RC output channel
 */
bool AP_BLHeli::BL_SendBuf(const uint8_t *buf, uint16_t len)
{
    bool send_crc = isMcuConnected();
    if (blheli.chan >= num_motors) {
        return false;
    }
    EXPECT_DELAY_MS(1000);
    if (!hal.rcout->serial_setup_output(motor_map[blheli.chan], 19200, motor_mask)) {
        blheli.ack = ACK_D_GENERAL_ERROR;
        return false;
    }
    if (serial_start_ms == 0) {
        serial_start_ms = AP_HAL::millis();
    }
    uint32_t now = AP_HAL::millis();
    if (serial_start_ms == 0 || now - serial_start_ms < 1000) {
        /*
          we've just started the interface. We want it idle for at
          least 1 second before we start sending serial data. 
         */
        hal.scheduler->delay(1100);
    }
    memcpy(blheli.buf, buf, len);
    uint16_t crc = BL_CRC(buf, len);
    blheli.buf[len] = crc;
    blheli.buf[len+1] = crc>>8;
    if (!hal.rcout->serial_write_bytes(blheli.buf, len+(send_crc?2:0))) {
        blheli.ack = ACK_D_GENERAL_ERROR;
        return false;
    }
    // 19200 baud is 52us per bit - wait for half a bit between sending and receiving to avoid reading
    // the end of the last sent bit by accident
    hal.scheduler->delay_microseconds(26);
    return true;
}

/*
  read bytes from the ESC connection
 */
bool AP_BLHeli::BL_ReadBuf(uint8_t *buf, uint16_t len)
{
    bool check_crc = isMcuConnected() && len > 0;
    uint16_t req_bytes = len+(check_crc?3:1);
    EXPECT_DELAY_MS(1000);
    uint16_t n = hal.rcout->serial_read_bytes(blheli.buf, req_bytes);
    debug("BL_ReadBuf %u -> %u", len, n);
    if (req_bytes != n) {
        debug("short read");
        blheli.ack = ACK_D_GENERAL_ERROR;
        return false;
    }
    if (check_crc) {
        uint16_t crc = BL_CRC(blheli.buf, len);
        if ((crc & 0xff) != blheli.buf[len] ||
            (crc >> 8) != blheli.buf[len+1]) {
            debug("bad CRC");
            blheli.ack = ACK_D_GENERAL_ERROR;
            return false;
        }
        if (blheli.buf[len+2] != brSUCCESS) {
            debug("bad ACK 0x%02x", blheli.buf[len+2]);
            blheli.ack = ACK_D_GENERAL_ERROR;
            return false;
        }
    } else {
        if (blheli.buf[len] != brSUCCESS) {
            debug("bad ACK1 0x%02x", blheli.buf[len]);
            blheli.ack = ACK_D_GENERAL_ERROR;
            return false;
        }
    }
    if (len > 0) {
        memcpy(buf, blheli.buf, len);
    }
    return true;
}

uint8_t AP_BLHeli::BL_GetACK(uint16_t timeout_ms)
{
    uint8_t ack;
    uint32_t start_ms = AP_HAL::millis();
    EXPECT_DELAY_MS(1000);
    while (AP_HAL::millis() - start_ms < timeout_ms) {
        if (hal.rcout->serial_read_bytes(&ack, 1) == 1) {
            return ack;
        }
    }
    // return brNONE, meaning no ACK received in the timeout
    return brNONE;
}

bool AP_BLHeli::BL_SendCMDSetAddress()
{
    // skip if adr == 0xFFFF
    if (blheli.address == 0xFFFF) {
        return true;
    }
    debug("BL_SendCMDSetAddress 0x%04x", blheli.address);
    uint8_t sCMD[] = {CMD_SET_ADDRESS, 0, uint8_t(blheli.address>>8), uint8_t(blheli.address)};
    if (!BL_SendBuf(sCMD, 4)) {
        return false;
    }
    return BL_GetACK() == brSUCCESS;
}

bool AP_BLHeli::BL_ReadA(uint8_t cmd, uint8_t *buf, uint16_t n)
{
    if (BL_SendCMDSetAddress()) {
        uint8_t sCMD[] = {cmd, uint8_t(n==256?0:n)};
        if (!BL_SendBuf(sCMD, 2)) {
            return false;
        }
        bool ret = BL_ReadBuf(buf, n);
        if (ret && n == sizeof(esc_status) && blheli.address == esc_status_addr) {
            // display esc_status structure if we see it
            struct esc_status status;
            memcpy(&status, buf, n);
            debug("Prot %u Good %u Bad %u %x %x %x x%x\n",
                  (unsigned)status.protocol,
                  (unsigned)status.good_frames,
                  (unsigned)status.bad_frames,
                  (unsigned)status.unknown[0],
                  (unsigned)status.unknown[1],
                  (unsigned)status.unknown[2],
                  (unsigned)status.unknown2);
        }
        return ret;
    }
    return false;
}

/*
  connect to a blheli ESC
 */
bool AP_BLHeli::BL_ConnectEx(void)
{
    if (blheli.connected[blheli.chan] != 0) {
        debug("Using cached interface 0x%x for %u", blheli.interface_mode[blheli.chan], blheli.chan);
        return true;
    }
    debug("BL_ConnectEx %u/%u at %u", blheli.chan, num_motors, motor_map[blheli.chan]);
    setDisconnected();
    const uint8_t BootInit[] = {0,0,0,0,0,0,0,0,0,0,0,0,0x0D,'B','L','H','e','l','i',0xF4,0x7D};
    if (!BL_SendBuf(BootInit, 21)) {
        return false;
    }

    uint8_t BootInfo[8];
    if (!BL_ReadBuf(BootInfo, 8)) {
        return false;
    }

    // reply must start with 471
    if (strncmp((const char *)BootInfo, "471", 3) != 0) {
        blheli.ack = ACK_D_GENERAL_ERROR;        
        return false;
    }

    // extract device information
    blheli.deviceInfo[blheli.chan][2] = BootInfo[3];
    blheli.deviceInfo[blheli.chan][1] = BootInfo[4];
    blheli.deviceInfo[blheli.chan][0] = BootInfo[5];

    blheli.interface_mode[blheli.chan] = 0;

    uint16_t devword;
    memcpy(&devword, blheli.deviceInfo[blheli.chan], sizeof(devword));
    switch (devword) {
    case 0x9307:
    case 0x930A:
    case 0x930F:
    case 0x940B:
        blheli.interface_mode[blheli.chan] = imATM_BLB;
        debug("Interface type imATM_BLB");
        break;
    case 0xF310:
    case 0xF330:
    case 0xF410:
    case 0xF390:
    case 0xF850:
    case 0xE8B1:
    case 0xE8B2:
        blheli.interface_mode[blheli.chan] = imSIL_BLB;
        debug("Interface type imSIL_BLB");
        break;
    default:
        // BLHeli_32 MCU ID hi > 0x00 and < 0x90 / lo always = 0x06
        if ((blheli.deviceInfo[blheli.chan][1] > 0x00) && (blheli.deviceInfo[blheli.chan][1] < 0x90) && (blheli.deviceInfo[blheli.chan][0] == 0x06)) {
            blheli.interface_mode[blheli.chan] = imARM_BLB;
            debug("Interface type imARM_BLB");
        } else {
            blheli.ack = ACK_D_GENERAL_ERROR;
            debug("Unknown interface type 0x%04x", devword);
            break;
        }
    }
    blheli.deviceInfo[blheli.chan][3] = blheli.interface_mode[blheli.chan];
    if (blheli.interface_mode[blheli.chan] != 0) {
        blheli.connected[blheli.chan] = true;
    }
    return true;
}

bool AP_BLHeli::BL_SendCMDKeepAlive(void)
{
    uint8_t sCMD[] = {CMD_KEEP_ALIVE, 0};
    if (!BL_SendBuf(sCMD, 2)) {
        return false;
    }
    if (BL_GetACK() != brERRORCOMMAND) {
        return false;
    }
    return true;
}

bool AP_BLHeli::BL_PageErase(void)
{
    if (BL_SendCMDSetAddress()) {
        uint8_t sCMD[] = {CMD_ERASE_FLASH, 0x01};
        if (!BL_SendBuf(sCMD, 2)) {
            return false;
        }
        return BL_GetACK(3000) == brSUCCESS;
    }
    return false;
}

void AP_BLHeli::BL_SendCMDRunRestartBootloader(void)
{
    uint8_t sCMD[] = {RestartBootloader, 0};
    blheli.deviceInfo[blheli.chan][0] = 1;
    BL_SendBuf(sCMD, 2);
}

uint8_t AP_BLHeli::BL_SendCMDSetBuffer(const uint8_t *buf, uint16_t nbytes)
{
    uint8_t sCMD[] = {CMD_SET_BUFFER, 0, uint8_t(nbytes>>8), uint8_t(nbytes&0xff)};
    if (!BL_SendBuf(sCMD, 4)) {
        return false;
    }
    uint8_t ack;
    if ((ack = BL_GetACK()) != brNONE) {
        debug("BL_SendCMDSetBuffer ack failed 0x%02x", ack);
        blheli.ack = ACK_D_GENERAL_ERROR;
        return false;
    }
    if (!BL_SendBuf(buf, nbytes)) {
        debug("BL_SendCMDSetBuffer send failed");
        blheli.ack = ACK_D_GENERAL_ERROR;
        return false;
    }
    return (BL_GetACK(40) == brSUCCESS);
}

bool AP_BLHeli::BL_WriteA(uint8_t cmd, const uint8_t *buf, uint16_t nbytes, uint32_t timeout_ms)
{
    if (BL_SendCMDSetAddress()) {
        if (!BL_SendCMDSetBuffer(buf, nbytes)) {
            blheli.ack = ACK_D_GENERAL_ERROR;
            return false;
        }
        uint8_t sCMD[] = {cmd, 0x01};
        if (!BL_SendBuf(sCMD, 2)) {
            return false;
        }
        return (BL_GetACK(timeout_ms) == brSUCCESS);
    }
    blheli.ack = ACK_D_GENERAL_ERROR;
    return false;
}

uint8_t AP_BLHeli::BL_WriteFlash(const uint8_t *buf, uint16_t n)
{
    return BL_WriteA(CMD_PROG_FLASH, buf, n, 500);
}

bool AP_BLHeli::BL_VerifyFlash(const uint8_t *buf, uint16_t n)
{
    if (BL_SendCMDSetAddress()) {
        if (!BL_SendCMDSetBuffer(buf, n)) {
            return false;
        }
        uint8_t sCMD[] = {CMD_VERIFY_FLASH_ARM, 0x01};
        if (!BL_SendBuf(sCMD, 2)) {
            return false;
        }
        uint8_t ack = BL_GetACK(40);
        switch (ack) {
        case brSUCCESS:
            blheli.ack = ACK_OK;
            break;
        case brERRORVERIFY:
            blheli.ack = ACK_I_VERIFY_ERROR;
            break;
        default:
            blheli.ack = ACK_D_GENERAL_ERROR;
            break;
        }
        return true;
    }
    return false;
}

/*
  process a blheli 4way command from GCS
 */
void AP_BLHeli::blheli_process_command(void)
{
    debug("BLHeli cmd 0x%02x len=%u", blheli.command, blheli.param_len);
    blheli.ack = ACK_OK;
    switch (blheli.command) {
    case cmd_InterfaceTestAlive: {
        debug("cmd_InterfaceTestAlive");
        BL_SendCMDKeepAlive();
        if (blheli.ack != ACK_OK) {
            setDisconnected();
        }
        uint8_t b = 0;
        blheli_send_reply(&b, 1);
        break;
    }
    case cmd_ProtocolGetVersion: {
        debug("cmd_ProtocolGetVersion");
        uint8_t buf[1];
        buf[0] = SERIAL_4WAY_PROTOCOL_VER;
        blheli_send_reply(buf, sizeof(buf));
        break;
    }
    case cmd_InterfaceGetName: {
        debug("cmd_InterfaceGetName");
        uint8_t buf[5] = { 4, 'A', 'R', 'D', 'U' };
        blheli_send_reply(buf, sizeof(buf));
        break;
    }
    case cmd_InterfaceGetVersion: {
        debug("cmd_InterfaceGetVersion");
        uint8_t buf[2] = { SERIAL_4WAY_VERSION_HI, SERIAL_4WAY_VERSION_LO };
        blheli_send_reply(buf, sizeof(buf));
        break;
    }
    case cmd_InterfaceExit: {
        debug("cmd_InterfaceExit");
        msp.escMode = PROTOCOL_NONE;
        uint8_t b = 0;
        blheli_send_reply(&b, 1);
        hal.rcout->serial_end();
        serial_start_ms = 0;
        if (motors_disabled) {
            motors_disabled = false;
            SRV_Channels::set_disabled_channel_mask(motors_disabled_mask);
        }
        if (uart_locked) {
            debug("Unlocked UART");
            uart->lock_port(0, 0);
            uart_locked = false;
        }
        memset(blheli.connected, 0, sizeof(blheli.connected));
        break;
    }
    case cmd_DeviceReset: {
        debug("cmd_DeviceReset(%u)", unsigned(blheli.buf[0]));
        if (blheli.buf[0] >= num_motors) {
            debug("bad reset channel %u", blheli.buf[0]);
            blheli.ack = ACK_I_INVALID_CHANNEL;
            blheli_send_reply(&blheli.buf[0], 1);            
            break;
        }
        blheli.chan = blheli.buf[0];
        switch (blheli.interface_mode[blheli.chan]) {
        case imSIL_BLB:
        case imATM_BLB:
        case imARM_BLB:
            BL_SendCMDRunRestartBootloader();
            break;
        case imSK:
            break;
        }
        blheli_send_reply(&blheli.chan, 1);
        setDisconnected();
        break;
    }

    case cmd_DeviceInitFlash: {
        debug("cmd_DeviceInitFlash(%u)", unsigned(blheli.buf[0]));
        if (blheli.buf[0] >= num_motors) {
            debug("bad channel %u", blheli.buf[0]);
            blheli.ack = ACK_I_INVALID_CHANNEL;
            blheli_send_reply(&blheli.buf[0], 1);
            break;
        }
        blheli.chan = blheli.buf[0];
        blheli.ack = ACK_OK;
        BL_ConnectEx();
        uint8_t buf[4] = {blheli.deviceInfo[blheli.chan][0],
                          blheli.deviceInfo[blheli.chan][1],
                          blheli.deviceInfo[blheli.chan][2],
                          blheli.deviceInfo[blheli.chan][3]};  // device ID
        blheli_send_reply(buf, sizeof(buf));
        break;
    }

    case cmd_InterfaceSetMode: {
        debug("cmd_InterfaceSetMode(%u)", unsigned(blheli.buf[0]));
        blheli.interface_mode[blheli.chan] = blheli.buf[0];
        blheli_send_reply(&blheli.interface_mode[blheli.chan], 1);
        break;
    }

    case cmd_DeviceRead: {
        uint16_t nbytes = blheli.buf[0]?blheli.buf[0]:256;
        debug("cmd_DeviceRead(%u) n=%u", blheli.chan, nbytes);
        uint8_t buf[nbytes];
        uint8_t cmd = blheli.interface_mode[blheli.chan]==imATM_BLB?CMD_READ_FLASH_ATM:CMD_READ_FLASH_SIL;
        if (!BL_ReadA(cmd, buf, nbytes)) {
            nbytes = 1;
        }
        blheli_send_reply(buf, nbytes);
        break;
    }

    case cmd_DevicePageErase: {
        uint8_t page = blheli.buf[0];
        debug("cmd_DevicePageErase(%u) im=%u", page, blheli.interface_mode[blheli.chan]);
        switch (blheli.interface_mode[blheli.chan]) {
        case imSIL_BLB:
        case imARM_BLB: {
            if  (blheli.interface_mode[blheli.chan] == imARM_BLB) {
                // Address =Page * 1024
                blheli.address = page << 10;
            } else {
                // Address =Page * 512
                blheli.address = page << 9;
            }
            debug("ARM PageErase 0x%04x", blheli.address);
            BL_PageErase();
            blheli.address = 0;
            blheli_send_reply(&page, 1);
            break;
        }
        default:
            blheli.ack = ACK_I_INVALID_CMD;
            blheli_send_reply(&page, 1);
            break;
        }
        break;
    }

    case cmd_DeviceWrite: {
        uint16_t nbytes = blheli.param_len;
        debug("cmd_DeviceWrite n=%u im=%u", nbytes, blheli.interface_mode[blheli.chan]);
        uint8_t buf[nbytes];
        memcpy(buf, blheli.buf, nbytes);
        switch (blheli.interface_mode[blheli.chan]) {
        case imSIL_BLB:
        case imATM_BLB:
        case imARM_BLB: {
            BL_WriteFlash(buf, nbytes);
            break;
        }
        case imSK: {
            debug("Unsupported flash mode imSK");
            break;
        }
        }
        uint8_t b=0;
        blheli_send_reply(&b, 1);        
        break;
    }

    case cmd_DeviceVerify: {
        uint16_t nbytes = blheli.param_len;
        debug("cmd_DeviceWrite n=%u im=%u", nbytes, blheli.interface_mode[blheli.chan]);
        switch (blheli.interface_mode[blheli.chan]) {
        case imARM_BLB: {
            uint8_t buf[nbytes];
            memcpy(buf, blheli.buf, nbytes);            
            BL_VerifyFlash(buf, nbytes);
            break;
        }
        default:
            blheli.ack = ACK_I_INVALID_CMD;
            break;
        }
        uint8_t b=0;
        blheli_send_reply(&b, 1);        
        break;
    }

    case cmd_DeviceReadEEprom: {
        uint16_t nbytes = blheli.buf[0]?blheli.buf[0]:256;
        uint8_t buf[nbytes];
        debug("cmd_DeviceReadEEprom n=%u im=%u", nbytes, blheli.interface_mode[blheli.chan]);
        switch (blheli.interface_mode[blheli.chan]) {
        case imATM_BLB: {
            if (!BL_ReadA(CMD_READ_EEPROM, buf, nbytes)) {
                blheli.ack = ACK_D_GENERAL_ERROR;
            }
            break;
        }
        default:
            blheli.ack = ACK_I_INVALID_CMD;
            break;
        }
        if (blheli.ack != ACK_OK) {
            nbytes = 1;
            buf[0] = 0;
        }
        blheli_send_reply(buf, nbytes);
        break;
    }

    case cmd_DeviceWriteEEprom: {
        uint16_t nbytes = blheli.param_len;
        uint8_t buf[nbytes];
        memcpy(buf, blheli.buf, nbytes);
        debug("cmd_DeviceWriteEEprom n=%u im=%u", nbytes, blheli.interface_mode[blheli.chan]);
        switch (blheli.interface_mode[blheli.chan]) {
        case imATM_BLB:
            BL_WriteA(CMD_PROG_EEPROM, buf, nbytes, 3000);
            break;
        default:
            blheli.ack = ACK_D_GENERAL_ERROR;
            break;
        }
        uint8_t b = 0;
        blheli_send_reply(&b, 1);
        break;
    }

    case cmd_DeviceEraseAll:
    case cmd_DeviceC2CK_LOW:
    default:
        // ack=unknown command
        blheli.ack = ACK_I_INVALID_CMD;
        debug("Unknown BLHeli protocol 0x%02x", blheli.command);
        uint8_t b = 0;
        blheli_send_reply(&b, 1);
        break;
    }
}

/*
  process an input byte, return true if we have received a whole
  packet with correct CRC
 */
bool AP_BLHeli::process_input(uint8_t b)
{
    bool valid_packet = false;

    if (msp.escMode == PROTOCOL_4WAY && blheli.state == BLHELI_IDLE && b == '$') {
        debug("Change to MSP mode");
        msp.escMode = PROTOCOL_NONE;
        hal.rcout->serial_end();
        serial_start_ms = 0;
    }
    if (msp.escMode != PROTOCOL_4WAY && msp.state == MSP_IDLE && b == '/') {
        debug("Change to BLHeli mode");
        memset(blheli.connected, 0, sizeof(blheli.connected));
        msp.escMode = PROTOCOL_4WAY;
    }
    if (msp.escMode == PROTOCOL_4WAY) {
        blheli_4way_process_byte(b);
    } else {
        msp_process_byte(b);
    }
    if (msp.escMode == PROTOCOL_4WAY) {
        if (blheli.state == BLHELI_COMMAND_RECEIVED) {
            valid_packet = true;
            last_valid_ms = AP_HAL::millis();
            if (uart->lock_port(BLHELI_UART_LOCK_KEY, 0)) {
                uart_locked = true;
            }
            blheli_process_command();
            blheli.state = BLHELI_IDLE;
            msp.state = MSP_IDLE;
        }
    } else if (msp.state == MSP_COMMAND_RECEIVED) {
        if (msp.packetType == MSP_PACKET_COMMAND) {
            valid_packet = true;
            if (uart->lock_port(BLHELI_UART_LOCK_KEY, 0)) {
                uart_locked = true;
            }
            last_valid_ms = AP_HAL::millis();
            msp_process_command();
        }
        msp.state = MSP_IDLE;
        blheli.state = BLHELI_IDLE;
    }

    return valid_packet;
}

/*
  protocol handler for detecting BLHeli input
 */
bool AP_BLHeli::protocol_handler(uint8_t b, AP_HAL::UARTDriver *_uart)
{
    uart = _uart;
    if (hal.util->get_soft_armed()) {
        // don't allow MSP control when armed
        return false;
    }
    return process_input(b);
}

/*
  run a connection test to the ESCs. This is used to test the
  operation of the BLHeli ESC protocol
*/
void AP_BLHeli::run_connection_test(uint8_t chan)
{
    run_test.set_and_notify(0);
    debug_uart = hal.console;
    uint8_t saved_chan = blheli.chan;
    if (chan >= num_motors) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ESC: bad channel %u", chan);
        return;
    }
    blheli.chan = chan;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ESC: Running test on channel %u",  blheli.chan);
    bool passed = false;
    for (uint8_t tries=0; tries<5; tries++) {
        EXPECT_DELAY_MS(3000);
        blheli.ack = ACK_OK;
        setDisconnected();
        if (BL_ConnectEx()) {
            uint8_t buf[256];
            uint8_t cmd = blheli.interface_mode[blheli.chan]==imATM_BLB?CMD_READ_FLASH_ATM:CMD_READ_FLASH_SIL;
            passed = true;
            blheli.address = blheli.interface_mode[blheli.chan]==imATM_BLB?0:0x7c00;
            passed &= BL_ReadA(cmd, buf, sizeof(buf));
            if (blheli.interface_mode[blheli.chan]==imARM_BLB) {
                if (passed) {
                    // read status structure
                    blheli.address = esc_status_addr;
                    passed &= BL_SendCMDSetAddress();
                }
                if (passed) {
                    struct esc_status status;
                    passed &= BL_ReadA(CMD_READ_FLASH_SIL, (uint8_t *)&status, sizeof(status));
                }
            }
            BL_SendCMDRunRestartBootloader();
            break;
        }
    }
    hal.rcout->serial_end();
    SRV_Channels::set_disabled_channel_mask(motors_disabled_mask);
    motors_disabled = false;
    serial_start_ms = 0;
    blheli.chan = saved_chan;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ESC: Test %s", passed?"PASSED":"FAILED");
    debug_uart = nullptr;
}

/*
  update BLHeli
 */
void AP_BLHeli::update(void)
{
    bool motor_control_active = false;
    for (uint8_t i = 0; i < num_motors; i++) {
        bool reversed = ((1U<< motor_map[i]) & channel_reversible_mask.get()) != 0;
        if (hal.rcout->read( motor_map[i]) != (reversed ? 1500 : 1000)) {
            motor_control_active = true;
        }
    }

    uint32_t now = AP_HAL::millis();
    if (initialised && uart_locked &&
        ((timeout_sec && now - last_valid_ms > uint32_t(timeout_sec.get())*1000U) || 
        (motor_control_active && now - last_valid_ms > MOTOR_ACTIVE_TIMEOUT))) {
        // we're not processing requests any more, shutdown serial
        // output
        if (serial_start_ms) {
            hal.rcout->serial_end();
            serial_start_ms = 0;
        }
        if (motors_disabled) {
            motors_disabled = false;
            SRV_Channels::set_disabled_channel_mask(motors_disabled_mask);
        }
        if (uart != nullptr) {
            debug("Unlocked UART");
            uart->lock_port(0, 0);
            uart_locked = false;
        }
        if (motor_control_active) {
            for (uint8_t i = 0; i < num_motors; i++) {
                bool reversed = ((1U<<motor_map[i]) & channel_reversible_mask.get()) != 0;
                hal.rcout->write(motor_map[i], reversed ? 1500 : 1000);
            }
        }
    }

    if (initialised || (channel_mask.get() == 0 && channel_auto.get() == 0)) {
        if (initialised && run_test.get() > 0) {
            run_connection_test(run_test.get() - 1);
        }
    }
}

/*
  Initialize BLHeli, called by SRV_Channels::init()
  Used to install protocol handler
  The motor mask of enabled motors can be passed in
 */
void AP_BLHeli::init(uint32_t mask, AP_HAL::RCOutput::output_mode otype)
{
    initialised = true;

    run_test.set_and_notify(0);

#if HAL_GCS_ENABLED
    // only install pass-thru protocol handler if either auto or the motor mask are set
    if (channel_mask.get() != 0 || channel_auto.get() != 0) {
        if (last_control_port > 0 && last_control_port != control_port) {
            gcs().install_alternative_protocol((mavlink_channel_t)(MAVLINK_COMM_0+last_control_port), nullptr);
            last_control_port = -1;
        }
        if (gcs().install_alternative_protocol((mavlink_channel_t)(MAVLINK_COMM_0+control_port),
                                            FUNCTOR_BIND_MEMBER(&AP_BLHeli::protocol_handler,
                                                                bool, uint8_t, AP_HAL::UARTDriver *))) {
            debug("BLHeli installed on port %u", (unsigned)control_port);
            last_control_port = control_port;
        }
    }
#endif // HAL_GCS_ENABLED

#if HAL_WITH_IO_MCU
    if (AP_BoardConfig::io_enabled()) {
        // with IOMCU the local (FMU) channels start at 8
        chan_offset = 8;
    }
#endif

    mask |= uint32_t(channel_mask.get());

    /*
      allow mode override - this makes it possible to use DShot for
      rovers and subs, plus for quadplane fwd motors
     */
    // +1 converts from AP_Motors::pwm_type to AP_HAL::RCOutput::output_mode and saves doing a param conversion
    // this is the only use of the param, but this is still a bit of a hack
    const int16_t type = output_type.get() + 1;
    if (otype == AP_HAL::RCOutput::MODE_PWM_NONE) {
        otype = ((type > AP_HAL::RCOutput::MODE_PWM_NONE) && (type < AP_HAL::RCOutput::MODE_NEOPIXEL)) ? AP_HAL::RCOutput::output_mode(type) : AP_HAL::RCOutput::MODE_PWM_NONE;
    }
    switch (otype) {
    case AP_HAL::RCOutput::MODE_PWM_ONESHOT:
    case AP_HAL::RCOutput::MODE_PWM_ONESHOT125:
    case AP_HAL::RCOutput::MODE_PWM_BRUSHED:
    case AP_HAL::RCOutput::MODE_PWM_DSHOT150:
    case AP_HAL::RCOutput::MODE_PWM_DSHOT300:
    case AP_HAL::RCOutput::MODE_PWM_DSHOT600:
    case AP_HAL::RCOutput::MODE_PWM_DSHOT1200:
        if (mask) {
            hal.rcout->set_output_mode(mask, otype);
        }
        break;
    default:
        break;
    }

    uint32_t digital_mask = 0;
    // setting the digital mask changes the min/max PWM values
    // it's important that this is NOT done for non-digital channels as otherwise
    // PWM min can result in motors turning. set for individual overrides first
    if (mask && hal.rcout->is_dshot_protocol(otype)) {
        digital_mask = mask;
    }

#if APM_BUILD_COPTER_OR_HELI || APM_BUILD_TYPE(APM_BUILD_ArduPlane) || APM_BUILD_TYPE(APM_BUILD_Rover)
    /*
      plane and copter can use AP_Motors to get an automatic mask
     */
#if APM_BUILD_TYPE(APM_BUILD_Rover)
    AP_MotorsUGV *motors = AP::motors_ugv();
#else
    AP_Motors *motors = AP::motors();
#endif
    if (motors) {
        uint32_t motormask = motors->get_motor_mask();
        // set the rest of the digital channels
        if (motors->is_digital_pwm_type()) {
            digital_mask |= motormask;
        }
        mask |= motormask;
    }
#endif
    // tell SRV_Channels about ESC capabilities
    SRV_Channels::set_digital_outputs(digital_mask, uint32_t(channel_reversible_mask.get()) & digital_mask);
    // the dshot ESC type is required in order to send the reversed/reversible dshot command correctly
    hal.rcout->set_dshot_esc_type(SRV_Channels::get_dshot_esc_type());
    hal.rcout->set_reversible_mask(uint32_t(channel_reversible_mask.get()) & digital_mask);
    hal.rcout->set_reversed_mask(uint32_t(channel_reversed_mask.get()) & digital_mask);
#ifdef HAL_WITH_BIDIR_DSHOT
    // possibly enable bi-directional dshot
    hal.rcout->set_motor_poles(motor_poles);
    hal.rcout->set_bidir_dshot_mask(uint32_t(channel_bidir_dshot_mask.get()) & digital_mask);
#endif
    // add motors from channel mask
    for (uint8_t i=0; i<16 && num_motors < max_motors; i++) {
        if (mask & (1U<<i)) {
            motor_map[num_motors] = i;
            num_motors++;
        }
    }
    motor_mask = mask;
    debug("ESC: %u motors mask=0x%08lx", num_motors, mask);

    // check if we have a combination of reversable and normal
    mixed_type = (mask != (mask & channel_reversible_mask.get())) && (channel_reversible_mask.get() != 0);

    if (num_motors != 0 && telem_rate > 0) {
        AP_SerialManager *serial_manager = AP_SerialManager::get_singleton();
        if (serial_manager) {
            telem_uart = serial_manager->find_serial(AP_SerialManager::SerialProtocol_ESCTelemetry,0);
        }
    }
}

/*
  read an ESC telemetry packet
 */
void AP_BLHeli::read_telemetry_packet(void)
{
#if HAL_WITH_ESC_TELEM
    uint8_t buf[telem_packet_size];
    if (telem_uart->read(buf, telem_packet_size) < telem_packet_size) {
        // short read, we should have 10 bytes ready when this function is called
        return;
    }

    // calculate crc
    uint8_t crc = 0;
    for (uint8_t i=0; i<telem_packet_size-1; i++) {    
        crc = crc8_dvb(buf[i], crc, 0x07);
    }

    if (buf[telem_packet_size-1] != crc) {
        // bad crc
        debug("Bad CRC on %u", last_telem_esc);
        return;
    }
    // record the previous rpm so that we can slew to the new one
    uint16_t new_rpm = ((buf[7]<<8) | buf[8]) * 200 / motor_poles;
    const uint8_t motor_idx = motor_map[last_telem_esc];
    // we have received valid data, mark the ESC as now active
    hal.rcout->set_active_escs_mask(1<<motor_idx);
    update_rpm(motor_idx - chan_offset, new_rpm);

    TelemetryData t {
        .temperature_cdeg = int16_t(buf[0] * 100),
        .voltage = float(uint16_t((buf[1]<<8) | buf[2])) * 0.01,
        .current = float(uint16_t((buf[3]<<8) | buf[4])) * 0.01,
        .consumption_mah = float(uint16_t((buf[5]<<8) | buf[6])),
    };

    update_telem_data(motor_idx - chan_offset, t,
        AP_ESC_Telem_Backend::TelemetryType::CURRENT
            | AP_ESC_Telem_Backend::TelemetryType::VOLTAGE
            | AP_ESC_Telem_Backend::TelemetryType::CONSUMPTION
            | AP_ESC_Telem_Backend::TelemetryType::TEMPERATURE);

    if (debug_level >= 2) {
        uint16_t trpm = new_rpm;
        if (has_bidir_dshot(last_telem_esc)) {
            trpm = hal.rcout->get_erpm(motor_idx);
            if (trpm != 0xFFFF) {
                trpm = trpm * 200 / motor_poles;
            }
        }
        DEV_PRINTF("ESC[%u] T=%u V=%f C=%f con=%f RPM=%u e=%.1f t=%u\n",
                            last_telem_esc,
                            t.temperature_cdeg,
                            t.voltage,
                            t.current,
                            t.consumption_mah,
                            trpm, hal.rcout->get_erpm_error_rate(motor_idx), (unsigned)AP_HAL::millis());
    }
#endif // HAL_WITH_ESC_TELEM
}

/*
  log bidir telemetry - only called if BLH telemetry is not active
 */
void AP_BLHeli::log_bidir_telemetry(void)
{
    uint32_t now = AP_HAL::millis();

    if (debug_level >= 2 && now - last_log_ms[last_telem_esc] > 100) {
        if (has_bidir_dshot(last_telem_esc)) {
            const uint8_t motor_idx = motor_map[last_telem_esc];
            uint16_t trpm = hal.rcout->get_erpm(motor_idx);
            if (trpm != 0xFFFF) {    // don't log invalid values as they are never used
                trpm = trpm * 200 / motor_poles;
            }

            if (trpm > 0) {
                last_log_ms[last_telem_esc] = now;
                DEV_PRINTF("ESC[%u] RPM=%u e=%.1f t=%u\n", last_telem_esc, trpm, hal.rcout->get_erpm_error_rate(motor_idx), (unsigned)AP_HAL::millis());
            }
        }
    }

    if (!SRV_Channels::have_digital_outputs()) {
        return;
    }

    // ask the next ESC for telemetry
    uint8_t idx_pos = last_telem_esc;
    uint8_t idx = (idx_pos + 1) % num_motors;
    for (; idx != idx_pos; idx = (idx + 1) % num_motors) {
        if (SRV_Channels::have_digital_outputs(1U << motor_map[idx])) {
            break;
        }
    }
    if (SRV_Channels::have_digital_outputs(1U << motor_map[idx])) {
        last_telem_esc = idx;
    }
}

/*
  update BLHeli telemetry handling
  This is called on push() in SRV_Channels
 */
void AP_BLHeli::update_telemetry(void)
{
#ifdef HAL_WITH_BIDIR_DSHOT
    // we might only have bi-dir dshot
    if (channel_bidir_dshot_mask.get() != 0 && !telem_uart) {
        log_bidir_telemetry();
    }
#endif
    if (!telem_uart || !SRV_Channels::have_digital_outputs()) {
        return;
    }
    uint32_t now = AP_HAL::micros();
    uint32_t telem_rate_us = 1000000U / uint32_t(telem_rate.get() * num_motors);
    if (telem_rate_us < 2000) {
        // make sure we have a gap between frames
        telem_rate_us = 2000;
    }
    if (!telem_uart_started) {
        // we need to use begin() here to ensure the correct thread owns the uart
        telem_uart->begin(115200);
        telem_uart_started = true;
    }

    uint32_t nbytes = telem_uart->available();

    if (nbytes > telem_packet_size) {
        // if we have more than 10 bytes then we don't know which ESC
        // they are from. Throw them all away
        telem_uart->discard_input();
        return;
    }
    if (nbytes > 0 &&
        nbytes < telem_packet_size &&
        (last_telem_byte_read_us == 0 ||
         now - last_telem_byte_read_us < 1000)) {
        // wait a bit longer, we don't have enough bytes yet
        if (last_telem_byte_read_us == 0) {
            last_telem_byte_read_us = now;
        }
        return;
    }
    if (nbytes > 0 && nbytes < telem_packet_size) {
        // we've waited long enough, discard bytes if we don't have 10 yet
        telem_uart->discard_input();
        return;
    }
    if (nbytes == telem_packet_size) {
        // we have a full packet ready to parse
        read_telemetry_packet();
        last_telem_byte_read_us = 0;
    }
    if (now - last_telem_request_us >= telem_rate_us) {
        // ask the next ESC for telemetry
        uint8_t idx_pos = last_telem_esc;
        uint8_t idx = (idx_pos + 1) % num_motors;
        for (; idx != idx_pos; idx = (idx + 1) % num_motors) {
            if (SRV_Channels::have_digital_outputs(1U << motor_map[idx])) {
                break;
            }
        }
        uint32_t mask = 1U << motor_map[idx];
        if (SRV_Channels::have_digital_outputs(mask)) {
            hal.rcout->set_telem_request_mask(mask);
            last_telem_esc = idx;
            last_telem_request_us = now;
        }
    }
}

#endif // HAVE_AP_BLHELI_SUPPORT
