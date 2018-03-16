/*
  example implementing MSP and BLHeli passthrough protocol in ArduPilot

  With thanks to betaflight for a great reference implementation
 */

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "ch.h"
#include "hal.h"
#include "hwdef.h"
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_Math/crc.h>
#include "msp_protocol.h"
#include "blheli_4way_protocol.h"

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

void setup();
void loop();

//#pragma GCC optimize("Og")

/*
  implementation of MSP protocol based on betaflight
 */

enum mspState {
    MSP_IDLE=0,
    MSP_HEADER_START,
    MSP_HEADER_M,
    MSP_HEADER_ARROW,
    MSP_HEADER_SIZE,
    MSP_HEADER_CMD,
    MSP_COMMAND_RECEIVED
};

enum mspPacketType {
    MSP_PACKET_COMMAND,
    MSP_PACKET_REPLY
};

enum escProtocol {
    PROTOCOL_SIMONK = 0,
    PROTOCOL_BLHELI = 1,
    PROTOCOL_KISS = 2,
    PROTOCOL_KISSALL = 3,
    PROTOCOL_CASTLE = 4,
    PROTOCOL_MAX = 5,
    PROTOCOL_NONE = 0xfe,
    PROTOCOL_4WAY = 0xff
};

enum motorPwmProtocol {
    PWM_TYPE_STANDARD = 0,
    PWM_TYPE_ONESHOT125,
    PWM_TYPE_ONESHOT42,
    PWM_TYPE_MULTISHOT,
    PWM_TYPE_BRUSHED,
    PWM_TYPE_DSHOT150,
    PWM_TYPE_DSHOT300,
    PWM_TYPE_DSHOT600,
    PWM_TYPE_DSHOT1200,
    PWM_TYPE_PROSHOT1000,
};

enum MSPFeatures {
    FEATURE_RX_PPM = 1 << 0,
    FEATURE_INFLIGHT_ACC_CAL = 1 << 2,
    FEATURE_RX_SERIAL = 1 << 3,
    FEATURE_MOTOR_STOP = 1 << 4,
    FEATURE_SERVO_TILT = 1 << 5,
    FEATURE_SOFTSERIAL = 1 << 6,
    FEATURE_GPS = 1 << 7,
    FEATURE_RANGEFINDER = 1 << 9,
    FEATURE_TELEMETRY = 1 << 10,
    FEATURE_3D = 1 << 12,
    FEATURE_RX_PARALLEL_PWM = 1 << 13,
    FEATURE_RX_MSP = 1 << 14,
    FEATURE_RSSI_ADC = 1 << 15,
    FEATURE_LED_STRIP = 1 << 16,
    FEATURE_DASHBOARD = 1 << 17,
    FEATURE_OSD = 1 << 18,
    FEATURE_CHANNEL_FORWARDING = 1 << 20,
    FEATURE_TRANSPONDER = 1 << 21,
    FEATURE_AIRMODE = 1 << 22,
    FEATURE_RX_SPI = 1 << 25,
    FEATURE_SOFTSPI = 1 << 26,
    FEATURE_ESC_SENSOR = 1 << 27,
    FEATURE_ANTI_GRAVITY = 1 << 28,
    FEATURE_DYNAMIC_FILTER = 1 << 29,
};


/*
  state of MSP command processing
 */
static struct {
    enum mspState state;
    enum mspPacketType packetType;
    uint8_t offset;
    uint8_t dataSize;
    uint8_t checksum;
    uint8_t buf[192];
    uint8_t cmdMSP;
    enum escProtocol escMode;
    uint8_t portIndex;
} msp;

#define MSP_PORT_INBUF_SIZE sizeof(msp.buf)


enum blheliState {
    BLHELI_IDLE=0,
    BLHELI_HEADER_START,
    BLHELI_HEADER_CMD,
    BLHELI_HEADER_ADDR_LOW,
    BLHELI_HEADER_ADDR_HIGH,
    BLHELI_HEADER_LEN,
    BLHELI_CRC1,
    BLHELI_CRC2,
    BLHELI_COMMAND_RECEIVED
};

/*
  state of blheli 4way protocol handling
 */
static struct {
    enum blheliState state;
    uint8_t command;
    uint16_t address;
    uint16_t param_len;
    uint16_t offset;
    uint8_t buf[256+3+8];
    uint8_t crc1;
    uint16_t crc;
    uint8_t interface_mode;
    uint8_t deviceInfo[4];
    uint8_t chan;
    uint8_t ack;
} blheli;


// start of 12 byte CPU ID
#ifndef UDID_START
#define UDID_START	0x1FFF7A10
#endif

// fixed number of channels for now
#define NUM_ESC_CHANNELS 4

/*
  process one byte of serial input for MSP protocol
 */
static bool msp_process_byte(uint8_t c)
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
        if (c > MSP_PORT_INBUF_SIZE) {
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
static void blheli_crc_update(uint8_t c)
{
    blheli.crc = crc_xmodem_update(blheli.crc, c);
}

/*
  process one byte of serial input for blheli 4way protocol
 */
static bool blheli_4way_process_byte(uint8_t c)
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
static void msp_send_reply(uint8_t cmd, const uint8_t *buf, uint8_t len)
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
    hal.uartC->write(&msp.buf[0], len+6);
}

static void putU16(uint8_t *b, uint16_t v)
{
    b[0] = v;
    b[1] = v >> 8;
}

static uint16_t getU16(const uint8_t *b)
{
    return b[0] | (b[1]<<8);
}

static void putU32(uint8_t *b, uint32_t v)
{
    b[0] = v;
    b[1] = v >> 8;
    b[2] = v >> 16;
    b[3] = v >> 24;
}

static void putU16_BE(uint8_t *b, uint16_t v)
{
    b[0] = v >> 8;
    b[1] = v;
}

/*
  process a MSP command from GCS
 */
static void msp_process_command(void)
{
    hal.console->printf("MSP cmd %u len=%u\n", msp.cmdMSP, msp.dataSize);
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
        hal.console->printf("MSP: rebooting\n");
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
        hal.console->printf("MSP_SET_MOTOR %u\n", nmotors);
        hal.rcout->cork();
        for (uint8_t i = 0; i < nmotors; i++) {
            uint16_t v = getU16(&msp.buf[i*2]);
            hal.console->printf("MSP_SET_MOTOR %u %u\n", i, v);
            hal.rcout->write(i, v);
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
        hal.console->printf("escMode=%u portIndex=%u\n", msp.escMode, msp.portIndex);
        uint8_t n = NUM_ESC_CHANNELS;
        switch (msp.escMode) {
        case PROTOCOL_4WAY:
            break;
        default:
            n = 0;
            hal.rcout->serial_end();
            break;
        }
        msp_send_reply(msp.cmdMSP, &n, 1);
        break;
    }
    default:
        hal.console->printf("Unknown MSP command %u\n", msp.cmdMSP);
        break;
    }
}


/*
  send a blheli 4way protocol reply
 */
static void blheli_send_reply(const uint8_t *buf, uint16_t len)
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
    hal.uartC->write(&blheli.buf[0], len+8);
    hal.console->printf("OutB(%u) 0x%02x ack=0x%02x\n", len+8, (unsigned)blheli.command, blheli.ack);
}

/*
  CRC used when talking to ESCs
 */
static uint16_t BL_CRC(const uint8_t *buf, uint16_t len)
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

static bool isMcuConnected(void)
{
    return blheli.deviceInfo[0] > 0;
}

static void setDisconnected(void)
{
    blheli.deviceInfo[0] = 0;
    blheli.deviceInfo[1] = 0;
}

/*
  send a set of bytes to an RC output channel
 */
static bool BL_SendBuf(const uint8_t *buf, uint16_t len)
{
    bool send_crc = isMcuConnected();
    if (!hal.rcout->serial_setup_output(blheli.chan, 19200)) {
        blheli.ack = ACK_D_GENERAL_ERROR;
        return false;
    }
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

static bool BL_ReadBuf(uint8_t *buf, uint16_t len)
{
    bool check_crc = isMcuConnected() && len > 0;
    uint16_t req_bytes = len+(check_crc?3:1);
    uint16_t n = hal.rcout->serial_read_bytes(blheli.buf, req_bytes);
    hal.console->printf("BL_ReadBuf %u -> %u\n", len, n);
    if (req_bytes != n) {
        hal.console->printf("short read\n");
        blheli.ack = ACK_D_GENERAL_ERROR;
        return false;
    }
    if (check_crc) {
        uint16_t crc = BL_CRC(blheli.buf, len);
        if ((crc & 0xff) != blheli.buf[len] ||
            (crc >> 8) != blheli.buf[len+1]) {
            hal.console->printf("bad CRC\n");
            blheli.ack = ACK_D_GENERAL_ERROR;
            return false;
        }
        if (blheli.buf[len+2] != brSUCCESS) {
            hal.console->printf("bad ACK\n");
            blheli.ack = ACK_D_GENERAL_ERROR;
            return false;
        }
    } else {
        if (blheli.buf[len] != brSUCCESS) {
            hal.console->printf("bad ACK1\n");
            blheli.ack = ACK_D_GENERAL_ERROR;
            return false;
        }
    }
    if (len > 0) {
        memcpy(buf, blheli.buf, len);
    }
    return true;
}

static uint8_t BL_GetACK(uint16_t timeout_ms=2)
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

static bool BL_SendCMDSetAddress()
{
    // skip if adr == 0xFFFF
    if (blheli.address == 0xFFFF) {
        return true;
    }
    hal.console->printf("BL_SendCMDSetAddress 0x%04x\n", blheli.address);
    uint8_t sCMD[] = {CMD_SET_ADDRESS, 0, uint8_t(blheli.address>>8), uint8_t(blheli.address)};
    BL_SendBuf(sCMD, 4);
    return BL_GetACK() == brSUCCESS;
}

static bool BL_ReadA(uint8_t cmd, uint8_t *buf, uint16_t n)
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
static bool BL_ConnectEx(void)
{
    hal.console->printf("BL_ConnectEx start\n");
    setDisconnected();
    const uint8_t BootInit[] = {0,0,0,0,0,0,0,0,0,0,0,0,0x0D,'B','L','H','e','l','i',0xF4,0x7D};
    BL_SendBuf(BootInit, 21);

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
        hal.console->printf("Interface type imATM_BLB\n");
        break;
    case 0xF310:
    case 0xF330:
    case 0xF410:
    case 0xF390:
    case 0xF850:
    case 0xE8B1:
    case 0xE8B2:
        blheli.interface_mode = imSIL_BLB;
        hal.console->printf("Interface type imSIL_BLB\n");
        break;
    case 0x1F06:
    case 0x3306:
    case 0x3406:
    case 0x3506:
        blheli.interface_mode = imARM_BLB;
        hal.console->printf("Interface type imARM_BLB\n");
        break;
    default:
        blheli.ack = ACK_D_GENERAL_ERROR;        
        hal.console->printf("Unknown interface type 0x%04x\n", *devword);
        break;
    }
    blheli.deviceInfo[3] = blheli.interface_mode;
    return true;
}

static bool BL_SendCMDKeepAlive(void)
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

static bool BL_PageErase(void)
{
    if (BL_SendCMDSetAddress()) {
        uint8_t sCMD[] = {CMD_ERASE_FLASH, 0x01};
        BL_SendBuf(sCMD, 2);
        return BL_GetACK(1000) == brSUCCESS;
    }
    return false;
}

static void BL_SendCMDRunRestartBootloader(void)
{
    uint8_t sCMD[] = {RestartBootloader, 0};
    blheli.deviceInfo[0] = 1;
    BL_SendBuf(sCMD, 2);
}

static uint8_t BL_SendCMDSetBuffer(const uint8_t *buf, uint16_t nbytes)
{
    uint8_t sCMD[] = {CMD_SET_BUFFER, 0, uint8_t(nbytes>>8), uint8_t(nbytes&0xff)};
    if (!BL_SendBuf(sCMD, 4)) {
        return false;
    }
    uint8_t ack;
    if ((ack = BL_GetACK()) != brNONE) {
        hal.console->printf("BL_SendCMDSetBuffer ack failed 0x%02x\n", ack);
        blheli.ack = ACK_D_GENERAL_ERROR;
        return false;
    }
    if (!BL_SendBuf(buf, nbytes)) {
        hal.console->printf("BL_SendCMDSetBuffer send failed\n");
        blheli.ack = ACK_D_GENERAL_ERROR;
        return false;
    }
    return (BL_GetACK(40) == brSUCCESS);
}

static bool BL_WriteA(uint8_t cmd, const uint8_t *buf, uint16_t nbytes, uint32_t timeout)
{
    if (BL_SendCMDSetAddress()) {
        if (!BL_SendCMDSetBuffer(buf, nbytes)) {
            blheli.ack = ACK_D_GENERAL_ERROR;
            return false;
        }
        uint8_t sCMD[] = {cmd, 0x01};
        BL_SendBuf(sCMD, 2);
        return (BL_GetACK(timeout) == brSUCCESS);
    }
    blheli.ack = ACK_D_GENERAL_ERROR;
    return false;
}

static uint8_t BL_WriteFlash(const uint8_t *buf, uint16_t n)
{
    return BL_WriteA(CMD_PROG_FLASH, buf, n, 250);
}

static bool BL_VerifyFlash(const uint8_t *buf, uint16_t n)
{
    if (BL_SendCMDSetAddress()) {
        if (!BL_SendCMDSetBuffer(buf, n)) {
            return false;
        }
        uint8_t sCMD[] = {CMD_VERIFY_FLASH_ARM, 0x01};
        BL_SendBuf(sCMD, 2);
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
static void blheli_process_command(void)
{
    hal.console->printf("BLHeli cmd 0x%02x len=%u\n", blheli.command, blheli.param_len);
    blheli.ack = ACK_OK;
    switch (blheli.command) {
    case cmd_InterfaceTestAlive: {
        hal.console->printf("cmd_InterfaceTestAlive\n");
        BL_SendCMDKeepAlive();
        if (blheli.ack != ACK_OK) {
            setDisconnected();
        }
        uint8_t b = 0;
        blheli_send_reply(&b, 1);
        break;
    }
    case cmd_ProtocolGetVersion: {
        hal.console->printf("cmd_ProtocolGetVersion\n");
        uint8_t buf[1];
        buf[0] = SERIAL_4WAY_PROTOCOL_VER;
        blheli_send_reply(buf, sizeof(buf));
        break;
    }
    case cmd_InterfaceGetName: {
        hal.console->printf("cmd_InterfaceGetName\n");
        uint8_t buf[5] = { 4, 'A', 'R', 'D', 'U' };
        blheli_send_reply(buf, sizeof(buf));
        break;
    }
    case cmd_InterfaceGetVersion: {
        hal.console->printf("cmd_InterfaceGetVersion\n");
        uint8_t buf[2] = { SERIAL_4WAY_VERSION_HI, SERIAL_4WAY_VERSION_LO };
        blheli_send_reply(buf, sizeof(buf));
        break;
    }
    case cmd_InterfaceExit: {
        hal.console->printf("cmd_InterfaceExit\n");
        msp.escMode = PROTOCOL_NONE;
        uint8_t b = 0;
        blheli_send_reply(&b, 1);
        hal.rcout->serial_end();
        break;
    }
    case cmd_DeviceReset: {
        hal.console->printf("cmd_DeviceReset(%u)\n", unsigned(blheli.buf[0]));
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
        hal.console->printf("cmd_DeviceInitFlash(%u)\n", unsigned(blheli.buf[0]));
        blheli.chan = blheli.buf[0];
        for (uint8_t tries=0; tries<5; tries++) {
            blheli.ack = ACK_OK;
            setDisconnected();
            if (BL_ConnectEx()) {
                break;
            }
            hal.scheduler->delay(5);
        }
        uint8_t buf[4] = {blheli.deviceInfo[0],
                          blheli.deviceInfo[1],
                          blheli.deviceInfo[2],
                          blheli.deviceInfo[3]};  // device ID
        blheli_send_reply(buf, sizeof(buf));
        break;
    }

    case cmd_InterfaceSetMode: {
        hal.console->printf("cmd_InterfaceSetMode(%u)\n", unsigned(blheli.buf[0]));
        blheli.interface_mode = blheli.buf[0];
        blheli_send_reply(&blheli.interface_mode, 1);
        break;
    }

    case cmd_DeviceRead: {
        uint16_t nbytes = blheli.buf[0]?blheli.buf[0]:256;
        hal.console->printf("cmd_DeviceRead(%u) n=%u\n", blheli.chan, nbytes);
        uint8_t buf[nbytes];
        if (!BL_ReadA(CMD_READ_FLASH_SIL, buf, nbytes)) {
            nbytes = 1;
        }
        blheli_send_reply(buf, nbytes);
        break;
    }

    case cmd_DevicePageErase: {
        uint8_t page = blheli.buf[0];
        hal.console->printf("cmd_DevicePageErase(%u) im=%u\n", page, blheli.interface_mode);
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
            hal.console->printf("ARM PageErase 0x%04x\n", blheli.address);
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
        hal.console->printf("cmd_DeviceWrite n=%u im=%u\n", nbytes, blheli.interface_mode);
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
            hal.console->printf("Unsupported flash mode imSK\n");
            break;
        }
        }
        uint8_t b=0;
        blheli_send_reply(&b, 1);        
        break;
    }

    case cmd_DeviceVerify: {
        uint16_t nbytes = blheli.param_len;
        hal.console->printf("cmd_DeviceWrite n=%u im=%u\n", nbytes, blheli.interface_mode);
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
        
    case cmd_DeviceEraseAll:
    case cmd_DeviceC2CK_LOW:
    case cmd_DeviceReadEEprom:
    case cmd_DeviceWriteEEprom:
    default:
        // ack=unknown command
        blheli.ack = ACK_I_INVALID_CMD;
        hal.console->printf("Unknown BLHeli protocol 0x%02x\n", blheli.command);
        uint8_t b = 0;
        blheli_send_reply(&b, 1);
        break;
    }
}

static void test_dshot_out(void)
{
    hal.rcout->cork();
    for (uint8_t i=0; i<6; i++) {
        hal.rcout->enable_ch(i);
        hal.rcout->write(i, 1100+i*100);
    }
    hal.rcout->push();
}


static void MSP_protocol_update(void)
{
    uint32_t n = hal.uartC->available();
    if (n > 0) {
        for (uint32_t i=0; i<n; i++) {
            int16_t b = hal.uartC->read();
            if (b < 0) {
                break;
            }
            if (msp.escMode == PROTOCOL_4WAY && blheli.state == BLHELI_IDLE && b == '$') {
                hal.console->printf("Change to MSP mode\n");
                msp.escMode = PROTOCOL_NONE;
                hal.rcout->serial_end();
            }
            if (msp.escMode != PROTOCOL_4WAY && msp.state == MSP_IDLE && b == '/') {
                hal.console->printf("Change to BLHeli mode\n");
                msp.escMode = PROTOCOL_4WAY;
            }
            if (msp.escMode == PROTOCOL_4WAY) {
                blheli_4way_process_byte(b);
            } else {
                msp_process_byte(b);
            }
        }
    }
    if (msp.escMode == PROTOCOL_4WAY) {
        if (blheli.state == BLHELI_COMMAND_RECEIVED) {
            blheli_process_command();
            blheli.state = BLHELI_IDLE;
            msp.state = MSP_IDLE;
        }
    } else if (msp.state == MSP_COMMAND_RECEIVED) {
        if (msp.packetType == MSP_PACKET_COMMAND) {
            msp_process_command();
        }
        msp.state = MSP_IDLE;
        blheli.state = BLHELI_IDLE;
    }

    static uint32_t last_tick_ms;
    uint32_t now = AP_HAL::millis();
    if (now - last_tick_ms > 1000) {
        hal.console->printf("tick\n");
        last_tick_ms = now;
        while (hal.console->available() > 0) {
            hal.console->read();
        }
#if 0
        BL_ConnectEx();
        hal.scheduler->delay(5);
        blheli.chan = 0;
        blheli.address = 0x7c00;
        BL_ReadA(CMD_READ_FLASH_SIL, blheli.buf, 256);
#endif
    }
}


static void test_output_modes(void)
{
    static uint8_t mode;
    
    hal.console->printf("Test mode %u\n", mode);

    switch (mode) {
    case 0:
        // test normal
        hal.rcout->set_output_mode(0xF, AP_HAL::RCOutput::MODE_PWM_NORMAL);
        hal.rcout->set_freq(0xF, 250);
        break;

    case 1:
        // test oneshot
        hal.rcout->set_output_mode(0xF, AP_HAL::RCOutput::MODE_PWM_ONESHOT);
        hal.rcout->set_freq(0xF, 400);
        break;

    case 2:
        // test brushed
        hal.rcout->set_output_mode(0xF, AP_HAL::RCOutput::MODE_PWM_BRUSHED);
        hal.rcout->set_freq(0xF, 16000);
        break;

    case 3:
        // test dshot150
        hal.rcout->set_output_mode(0xF, AP_HAL::RCOutput::MODE_PWM_DSHOT150);
        break;

    case 4:
        // test dshot300
        hal.rcout->set_output_mode(0xF, AP_HAL::RCOutput::MODE_PWM_DSHOT300);
        break;

    case 5:
        // test dshot600
        hal.rcout->set_output_mode(0xF, AP_HAL::RCOutput::MODE_PWM_DSHOT600);
        break;

    case 6:
        // test dshot1200
        hal.rcout->set_output_mode(0xF, AP_HAL::RCOutput::MODE_PWM_DSHOT1200);
        break;
        
    case 7:
        // test serial
        hal.rcout->serial_setup_output(0, 19200);
        hal.scheduler->delay(10);
        hal.rcout->serial_write_bytes((const uint8_t *)"Hello World", 12);
        hal.scheduler->delay(10);
        hal.rcout->serial_end();
        hal.scheduler->delay(10);
        break;

    case 8:
        hal.rcout->set_output_mode(0xF, AP_HAL::RCOutput::MODE_PWM_NONE);
        break;

    default:
        break;
    }
    
    hal.rcout->cork();
    hal.rcout->enable_ch(0);
    hal.rcout->enable_ch(4);
    hal.rcout->enable_ch(5);
    hal.rcout->write(0, 1100 + mode*100);
    hal.rcout->write(4, 1000 + mode*100);
    hal.rcout->write(5, 1000 + mode*100);
    hal.rcout->push();
    
    mode = (mode+1) % 9;
}

void setup(void) {
    hal.console->begin(115200);
    hal.scheduler->delay(1000);
    hal.console->printf("Starting\n");
    hal.uartC->begin(115200, 256, 256);
    hal.rcout->init();
    hal.rcout->set_esc_scaling(1000, 2000);
    hal.rcout->cork();
    for (uint8_t i=0; i<NUM_ESC_CHANNELS; i++) {
        hal.rcout->enable_ch(i);
        hal.rcout->write(i, 1000);
    }
    hal.rcout->push();
}

void loop(void)
{
    hal.scheduler->delay(200);
    //test_dshot_out();
    //MSP_protocol_update();
    test_output_modes();
    // allow for firmware upload without power cycling for rapid
    // development
    if (hal.console->available() > 10) {
        hal.console->printf("rebooting\n");
        hal.scheduler->delay(1000);
        hal.scheduler->reboot(false);
    }
}

AP_HAL_MAIN();
