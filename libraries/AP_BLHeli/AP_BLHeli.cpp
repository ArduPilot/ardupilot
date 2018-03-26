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

  With thanks to betaflight for a great reference implementation
 */

#include "AP_BLHeli.h"
#include <AP_Math/crc.h>
#include <AP_Motors/AP_Motors_Class.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#if 0
#define debug(fmt, args ...) do { printf("ESC: " fmt "\n", ## args); } while (0)
#else
#define debug(fmt, args ...)
#endif

const AP_Param::GroupInfo AP_BLHeli::var_info[] = {
    // @Param: MASK
    // @DisplayName: Channel Bitmask
    // @Description: Enable of BLHeli pass-thru servo protocol support to specific channels. This mask is in addition to motors enabled using SERVO_BLH_AUTO (if any)
    // @Bitmask: 0:Channel1,1:Channel2,2:Channel3,3:Channel4,4:Channel5,5:Channel6,6:Channel7,7:Channel8,8:Channel9,9:Channel10,10:Channel11,11:Channel12,12:Channel13,13:Channel14,14:Channel15,15:Channel16
    // @User: Advanced
    AP_GROUPINFO("MASK",  1, AP_BLHeli, channel_mask, 0),

    // @Param: AUTO
    // @DisplayName: auto-enable for multicopter motors
    // @Description: If set to 1 this auto-enables BLHeli pass-thru support for all multicopter motors
    // @Values: 0:Disabled,1:Enabled
    // @User: Standard
    AP_GROUPINFO("AUTO",  2, AP_BLHeli, channel_auto, 0),
    
    AP_GROUPEND
};

// constructor
AP_BLHeli::AP_BLHeli(void)
{
    // set defaults from the parameter table
    AP_Param::setup_object_defaults(this, var_info);
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
    memcpy(b, buf, len);
    b += len;
    uint8_t c = 0;
    for (uint8_t i=0; i<len+2; i++) {
        c ^= msp.buf[i+3];
    }
    *b++ = c;
    uart->write(&msp.buf[0], len+6);
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
        uint8_t buf[3] = { MSP_PROTOCOL_VERSION, API_VERSION_MAJOR, API_VERSION_MINOR };
        msp_send_reply(msp.cmdMSP, buf, sizeof(buf));
        break;
    }
        
    case MSP_FC_VARIANT:
        msp_send_reply(msp.cmdMSP, (const uint8_t *)ARDUPILOT_IDENTIFIER, FLIGHT_CONTROLLER_IDENTIFIER_LENGTH);
        break;
        
    case MSP_FC_VERSION: {
        uint8_t version[3] = { 3, 3, 0 };
        msp_send_reply(msp.cmdMSP, version, sizeof(version));
        break;
    }
    case MSP_BOARD_INFO: {
        // send a generic 'ArduPilot ChibiOS' board type
        uint8_t buf[7] = { 'A', 'R', 'C', 'H', 0, 0, 0 };
        msp_send_reply(msp.cmdMSP, buf, sizeof(buf));
        break;
    }

    case MSP_BUILD_INFO: {
         // build date, build time, git version
        uint8_t buf[26] {
                0x4d, 0x61, 0x72, 0x20, 0x31, 0x36, 0x20, 0x32, 0x30,
                0x31, 0x38, 0x30, 0x38, 0x3A, 0x34, 0x32, 0x3a, 0x32, 0x39,
                0x62, 0x30, 0x66, 0x66, 0x39, 0x32, 0x38};
        msp_send_reply(msp.cmdMSP, buf, sizeof(buf));
        break;
    }

    case MSP_REBOOT:
        debug("MSP: rebooting");
        hal.scheduler->reboot(false);
        break;

    case MSP_UID:
        // MCU identifer
        msp_send_reply(msp.cmdMSP, (const uint8_t *)UDID_START, 12);
        break;

    case MSP_ADVANCED_CONFIG: {
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
        uint8_t buf[4];
        putU32(buf, 0); // from MSPFeatures enum
        msp_send_reply(msp.cmdMSP, buf, sizeof(buf));
        break;
    }

    case MSP_STATUS: {
        uint8_t buf[21];
        putU16(&buf[0], 2500); // loop time usec
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
        uint8_t buf[6];
        putU16(&buf[0], 1406); // 3D deadband low
        putU16(&buf[2], 1514); // 3D deadband high
        putU16(&buf[4], 1460); // 3D neutral
        msp_send_reply(msp.cmdMSP, buf, sizeof(buf));
        break;
    }

    case MSP_MOTOR_CONFIG: {
        uint8_t buf[6];
        putU16(&buf[0], 1070); // min throttle
        putU16(&buf[2], 2000); // max throttle
        putU16(&buf[4], 1000); // min command
        msp_send_reply(msp.cmdMSP, buf, sizeof(buf));
        break;
    }

    case MSP_MOTOR: {
        // get the output going to each motor
        uint8_t buf[16];
        for (uint8_t i = 0; i < 8; i++) {
            putU16(&buf[2*i], hal.rcout->read(i));
        }
        msp_send_reply(msp.cmdMSP, buf, sizeof(buf));
        break;
    }

    case MSP_SET_MOTOR: {
        // set the output to each motor
        uint8_t nmotors = msp.dataSize / 2;
        debug("MSP_SET_MOTOR %u", nmotors);
        hal.rcout->cork();
        for (uint8_t i = 0; i < nmotors; i++) {
            if (i >= num_motors) {
                break;
            }
            uint16_t v = getU16(&msp.buf[i*2]);
            debug("MSP_SET_MOTOR %u %u", i, v);
            hal.rcout->write(motor_map[i], v);
        }
        hal.rcout->push();
        break;
    }
    
    case MSP_SET_4WAY_IF: {
        if (msp.dataSize == 0) {
            msp.escMode = PROTOCOL_4WAY;
        } else if (msp.dataSize == 2) {
            msp.escMode = (enum escProtocol)msp.buf[0];
            msp.portIndex = msp.buf[1];
        }
        debug("escMode=%u portIndex=%u", msp.escMode, msp.portIndex);
        uint8_t n = num_motors;
        switch (msp.escMode) {
        case PROTOCOL_4WAY:
            break;
        default:
            n = 0;
            hal.rcout->serial_end();
            serial_started = false;
            break;
        }
        msp_send_reply(msp.cmdMSP, &n, 1);
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
    uart->write(&blheli.buf[0], len+8);
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
    return blheli.deviceInfo[0] > 0;
}

void AP_BLHeli::setDisconnected(void)
{
    blheli.deviceInfo[0] = 0;
    blheli.deviceInfo[1] = 0;
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
    hal.scheduler->delay(10);
    if (!hal.rcout->serial_setup_output(motor_map[blheli.chan], 19200)) {
        blheli.ack = ACK_D_GENERAL_ERROR;
        return false;
    }
    serial_started = true;
    memcpy(blheli.buf, buf, len);
    uint16_t crc = BL_CRC(buf, len);
    blheli.buf[len] = crc;
    blheli.buf[len+1] = crc>>8;
    if (!hal.rcout->serial_write_bytes(blheli.buf, len+(send_crc?2:0))) {
        blheli.ack = ACK_D_GENERAL_ERROR;
        return false;
    }
    return true;
}

/*
  read bytes from the ESC connection
 */
bool AP_BLHeli::BL_ReadBuf(uint8_t *buf, uint16_t len)
{
    bool check_crc = isMcuConnected() && len > 0;
    uint16_t req_bytes = len+(check_crc?3:1);
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
        return BL_ReadBuf(buf, n);
    }
    return false;
}

/*
  connect to a blheli ESC
 */
bool AP_BLHeli::BL_ConnectEx(void)
{
    debug("BL_ConnectEx start");
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
    blheli.deviceInfo[2] = BootInfo[3];
    blheli.deviceInfo[1] = BootInfo[4];
    blheli.deviceInfo[0] = BootInfo[5];

    blheli.interface_mode = 0;
    
    uint16_t *devword = (uint16_t *)blheli.deviceInfo;
    switch (*devword) {
    case 0x9307:
    case 0x930A:
    case 0x930F:
    case 0x940B:
        blheli.interface_mode = imATM_BLB;
        debug("Interface type imATM_BLB");
        break;
    case 0xF310:
    case 0xF330:
    case 0xF410:
    case 0xF390:
    case 0xF850:
    case 0xE8B1:
    case 0xE8B2:
        blheli.interface_mode = imSIL_BLB;
        debug("Interface type imSIL_BLB");
        break;
    case 0x1F06:
    case 0x3306:
    case 0x3406:
    case 0x3506:
        blheli.interface_mode = imARM_BLB;
        debug("Interface type imARM_BLB");
        break;
    default:
        blheli.ack = ACK_D_GENERAL_ERROR;        
        debug("Unknown interface type 0x%04x", *devword);
        break;
    }
    blheli.deviceInfo[3] = blheli.interface_mode;
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
        return BL_GetACK(1000) == brSUCCESS;
    }
    return false;
}

void AP_BLHeli::BL_SendCMDRunRestartBootloader(void)
{
    uint8_t sCMD[] = {RestartBootloader, 0};
    blheli.deviceInfo[0] = 1;
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
    return BL_WriteA(CMD_PROG_FLASH, buf, n, 250);
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
        serial_started = false;
        break;
    }
    case cmd_DeviceReset: {
        debug("cmd_DeviceReset(%u)", unsigned(blheli.buf[0]));
        blheli.chan = blheli.buf[0];
        switch (blheli.interface_mode) {
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
        blheli.chan = blheli.buf[0];
        for (uint8_t tries=0; tries<5; tries++) {
            blheli.ack = ACK_OK;
            setDisconnected();
            if (BL_ConnectEx()) {
                break;
            }
        }
        uint8_t buf[4] = {blheli.deviceInfo[0],
                          blheli.deviceInfo[1],
                          blheli.deviceInfo[2],
                          blheli.deviceInfo[3]};  // device ID
        blheli_send_reply(buf, sizeof(buf));
        break;
    }

    case cmd_InterfaceSetMode: {
        debug("cmd_InterfaceSetMode(%u)", unsigned(blheli.buf[0]));
        blheli.interface_mode = blheli.buf[0];
        blheli_send_reply(&blheli.interface_mode, 1);
        break;
    }

    case cmd_DeviceRead: {
        uint16_t nbytes = blheli.buf[0]?blheli.buf[0]:256;
        debug("cmd_DeviceRead(%u) n=%u", blheli.chan, nbytes);
        uint8_t buf[nbytes];
        uint8_t cmd = blheli.interface_mode==imATM_BLB?CMD_READ_FLASH_ATM:CMD_READ_FLASH_SIL;
        if (!BL_ReadA(cmd, buf, nbytes)) {
            nbytes = 1;
        }
        blheli_send_reply(buf, nbytes);
        break;
    }

    case cmd_DevicePageErase: {
        uint8_t page = blheli.buf[0];
        debug("cmd_DevicePageErase(%u) im=%u", page, blheli.interface_mode);
        switch (blheli.interface_mode) {
        case imSIL_BLB:
        case imARM_BLB: {
            if  (blheli.interface_mode == imARM_BLB) {
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
        debug("cmd_DeviceWrite n=%u im=%u", nbytes, blheli.interface_mode);
        uint8_t buf[nbytes];
        memcpy(buf, blheli.buf, nbytes);
        switch (blheli.interface_mode) {
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
        debug("cmd_DeviceWrite n=%u im=%u", nbytes, blheli.interface_mode);
        switch (blheli.interface_mode) {
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
        debug("cmd_DeviceReadEEprom n=%u im=%u", nbytes, blheli.interface_mode);
        switch (blheli.interface_mode) {
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
        debug("cmd_DeviceWriteEEprom n=%u im=%u", nbytes, blheli.interface_mode);
        switch (blheli.interface_mode) {
        case imATM_BLB:
            BL_WriteA(CMD_PROG_EEPROM, buf, nbytes, 1000);
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
        serial_started = false;
    }
    if (msp.escMode != PROTOCOL_4WAY && msp.state == MSP_IDLE && b == '/') {
        debug("Change to BLHeli mode");
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
            blheli_process_command();
            blheli.state = BLHELI_IDLE;
            msp.state = MSP_IDLE;
        }
    } else if (msp.state == MSP_COMMAND_RECEIVED) {
        if (msp.packetType == MSP_PACKET_COMMAND) {
            valid_packet = true;
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
    return process_input(b);
}

/*
  update BLHeli
  Used to install protocol handler
 */
void AP_BLHeli::update(void)
{
    if (initialised && serial_started && AP_HAL::millis() - last_valid_ms > 4000) {
        // we're not processing requests any more, shutdown serial
        // output
        hal.rcout->serial_end();
        serial_started = false;
    }
    if (initialised || (channel_mask.get() == 0 && channel_auto.get() == 0)) {
        return;
    }
    initialised = true;
    
    if (gcs().install_alternative_protocol(MAVLINK_COMM_0,
                                           FUNCTOR_BIND_MEMBER(&AP_BLHeli::protocol_handler,
                                                               bool, uint8_t, AP_HAL::UARTDriver *))) {
        debug("BLHeli installed");
    }

    AP_Motors *motors = AP_Motors::get_instance();
    uint16_t motors_mask = 0;
    if (motors) {
        motors_mask = motors->get_motor_mask();
    }
    
    // add motors from channel mask
    uint16_t mask = uint16_t(channel_mask.get()) | motors_mask;
    for (uint8_t i=0; i<16 && num_motors < max_motors; i++) {
        if (mask & (1U<<i)) {
            motor_map[num_motors] = i;
            num_motors++;
        }
    }
    debug("ESC: mapped %u motors with mask 0x%04x", num_motors, mask);
}
