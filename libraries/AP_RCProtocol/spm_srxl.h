/*
MIT License

Copyright (c) 2019 Horizon Hobby, LLC

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

#ifndef __SRXL_H__
#define __SRXL_H__

#ifdef __cplusplus
extern "C"
{
#endif

// Standard C Libraries
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>


//      7.1 General Overview
#define SPEKTRUM_SRXL_ID        (0xA6)
#define SRXL_MAX_BUFFER_SIZE    (80)
#define SRXL_MAX_DEVICES        (16)

// Supported SRXL device types (upper nibble of device ID)
typedef enum
{
    SrxlDevType_None                = 0,
    SrxlDevType_RemoteReceiver      = 1,
    SrxlDevType_Receiver            = 2,
    SrxlDevType_FlightController    = 3,
    SrxlDevType_ESC                 = 4,
    SrxlDevType_SRXLServo1          = 6,
    SrxlDevType_SRXLServo2          = 7,
    SrxlDevType_VTX                 = 8,
    SrxlDevType_Broadcast           = 15
} SrxlDevType;

// Set SRXL_CRC_OPTIMIZE_MODE in spm_srxl_config.h to one of the following values
#define SRXL_CRC_OPTIMIZE_EXTERNAL  (0)  // Uses an external function defined by SRXL_CRC_EXTERNAL_FN for CRC
#define SRXL_CRC_OPTIMIZE_SPEED     (1)  // Uses table lookup for CRC computation (requires 512 const bytes for CRC table)
#define SRXL_CRC_OPTIMIZE_SIZE      (2)  // Uses bitwise operations
#define SRXL_CRC_OPTIMIZE_STM_HW    (3)  // Uses STM32 register-level hardware acceleration (only available on STM32F30x devices for now)
#define SRXL_CRC_OPTIMIZE_STM_HAL   (4)  // Uses STM32Cube HAL driver for hardware acceleration (only available on STM32F3/F7) -- see srxlCrc16() for details on HAL config

// Set SRXL_STM_TARGET_FAMILY in spm_srxl_config.h to one of the following values when using one of the STM HW-optimized modes
#define SRXL_STM_TARGET_F3          (3)
#define SRXL_STM_TARGET_F7          (7)

//      7.2 Handshake Packet
#define SRXL_HANDSHAKE_ID       (0x21)

// Supported additional baud rates besides default 115200
// NOTE: Treated as bitmask, ANDed with baud rates from slaves
#define SRXL_BAUD_115200        (0x00)
#define SRXL_BAUD_400000        (0x01)
//#define SRXL_BAUD_NEXT_RATE   (0x02)
//#define SRXL_BAUD_ANOTHER     (0x04)

// Bit masks for Device Info byte sent via Handshake
#define SRXL_DEVINFO_NO_RF              (0x00)  // This is the base for non-RF devices
#define SRXL_DEVINFO_TELEM_TX_ENABLED   (0x01)  // This bit is set if the device is actively configured to transmit telemetry over RF
#define SRXL_DEVINFO_TELEM_FULL_RANGE   (0x02)  // This bit is set if the device can send full-range telemetry over RF
#define SRXL_DEVINFO_FWD_PROG_SUPPORT   (0x04)  // This bit is set if the device supports Forward Programming via RF or SRXL

//      7.3 Bind Info Packet
#define SRXL_BIND_ID                    (0x41)
#define SRXL_BIND_REQ_ENTER             (0xEB)
#define SRXL_BIND_REQ_STATUS            (0xB5)
#define SRXL_BIND_REQ_BOUND_DATA        (0xDB)
#define SRXL_BIND_REQ_SET_BIND          (0x5B)

// Bit masks for Options byte
#define SRXL_BIND_OPT_NONE              (0x00)
#define SRXL_BIND_OPT_TELEM_TX_ENABLE   (0x01)  // Set if this device should be enabled as the current telemetry device to tx over RF
#define SRXL_BIND_OPT_BIND_TX_ENABLE    (0x02)  // Set if this device should reply to a bind request with a Discover packet over RF
#define SRXL_BIND_OPT_US_POWER          (0x04)  // Set if this device should request US transmit power levels instead of EU

// Current Bind Status
typedef enum
{
    NOT_BOUND           = 0x00,
    // Air types
    DSM2_1024_22MS      = 0x01,
    DSM2_1024_MC24      = 0x02,
    DSM2_2048_11MS      = 0x12,
    DSMX_22MS           = 0xA2,
    DSMX_11MS           = 0xB2,
    // Surface types (corresponding Air type bitwise OR'd with 0x40)
    SURFACE_DSM1        = 0x40,
    SURFACE_DSM2_16p5MS = 0x63,
    DSMR_11MS_22MS      = 0xE2,
    DSMR_5p5MS          = 0xE4,
} BIND_STATUS;

//      7.4 Parameter Configuration
#define SRXL_PARAM_ID           (0x50)
#define SRXL_PARAM_REQ_QUERY    (0x50)
#define SRXL_PARAM_REQ_WRITE    (0x57)

//      7.5 Signal Quality Packet
#define SRXL_RSSI_ID            (0x55)
#define SRXL_RSSI_REQ_REQUEST   (0x52)
#define SRXL_RSSI_REQ_SEND      (0x53)

//      7.6 Telemetry Sensor Data Packet
#define SRXL_TELEM_ID           (0x80)

//      7.7 Control Data Packet
#define SRXL_CTRL_ID                (0xCD)
#define SRXL_CTRL_BASE_LENGTH       (3 + 2 + 2) // header + cmd/replyID + crc
#define SRXL_CTRL_CMD_CHANNEL       (0x00)
#define SRXL_CTRL_CMD_CHANNEL_FS    (0x01)
#define SRXL_CTRL_CMD_VTX           (0x02)
#define SRXL_CTRL_CMD_FWDPGM        (0x03)

typedef enum
{
    SRXL_CMD_NONE,
    SRXL_CMD_CHANNEL,
    SRXL_CMD_CHANNEL_FS,
    SRXL_CMD_VTX,
    SRXL_CMD_FWDPGM,
    SRXL_CMD_RSSI,
    SRXL_CMD_HANDSHAKE,
    SRXL_CMD_TELEMETRY,
    SRXL_CMD_ENTER_BIND,
    SRXL_CMD_REQ_BIND,
    SRXL_CMD_SET_BIND,
    SRXL_CMD_BIND_INFO,
} SRXL_CMD;

// VTX Band
#define VTX_BAND_FATSHARK   (0)
#define VTX_BAND_RACEBAND   (1)
#define VTX_BAND_E_BAND     (2)
#define VTX_BAND_B_BAND     (3)
#define VTX_BAND_A_BAND     (4)

// VTX Pit Mode
#define VTX_MODE_RACE   (0)
#define VTX_MODE_PIT    (1)

// VTX Power
#define VTX_POWER_OFF           (0)
#define VTX_POWER_1MW_14MW      (1)
#define VTX_POWER_15MW_25MW     (2)
#define VTX_POWER_26MW_99MW     (3)
#define VTX_POWER_100MW_299MW   (4)
#define VTX_POWER_300MW_600MW   (5)
#define VTX_POWER_601_PLUS      (6)
#define VTX_POWER_MANUAL        (7)

// VTX Region
#define VTX_REGION_US   (0)
#define VTX_REGION_EU   (1)

// Forward Programming Pass-Thru
#define FWD_PGM_MAX_DATA_SIZE   (64)


// Enable byte packing for all structs defined here!
#ifdef PACKED
#define SRXL_EXTERNAL_PACKED
#elif defined(__GNUC__)
#define PACKED __attribute__((packed))
#else
#pragma pack(push, 1)
#define PACKED
#endif

// Spektrum SRXL header
typedef struct SrxlHeader
{
    uint8_t srxlID;     // Always 0xA6 for SRXL2
    uint8_t packetType;
    uint8_t length;
} PACKED SrxlHeader;

// Handshake
typedef struct SrxlHandshakeData
{
    uint8_t     srcDevID;
    uint8_t     destDevID;
    uint8_t     priority;
    uint8_t     baudSupported;  // 0 = 115200, 1 = 400000 (See SRXL_BAUD_xxx definitions above)
    uint8_t     info;           // See SRXL_DEVINFO_xxx definitions above for defined bits
    uint32_t    uid;            // Unique/random id to allow detection of two devices on bus with same deviceID
} PACKED SrxlHandshakeData;

typedef struct SrxlHandshakePacket
{
    SrxlHeader          hdr;
    SrxlHandshakeData   payload;
    uint16_t            crc;
} PACKED SrxlHandshakePacket;

// Bind
typedef struct SrxlBindData
{
    uint8_t     type;
    uint8_t     options;
    uint64_t    guid;
    uint32_t    uid;
} PACKED SrxlBindData;

typedef struct SrxlBindPacket
{
    SrxlHeader      hdr;
    uint8_t         request;
    uint8_t         deviceID;
    SrxlBindData    data;
    uint16_t        crc;
} PACKED SrxlBindPacket;

// Telemetry
typedef struct SrxlTelemetryData
{
    union
    {
        struct
        {
            uint8_t sensorID;
            uint8_t secondaryID;
            uint8_t data[14];
        };
        uint8_t raw[16];
    };
} PACKED SrxlTelemetryData;

typedef struct SrxlTelemetryPacket
{
    SrxlHeader          hdr;
    uint8_t             destDevID;
    SrxlTelemetryData   payload;
    uint16_t            crc;
} PACKED SrxlTelemetryPacket;

// Signal Quality
typedef struct SrxlRssiPacket
{
    SrxlHeader  hdr;
    uint8_t     request;
    int8_t      antennaA;
    int8_t      antennaB;
    int8_t      antennaC;
    int8_t      antennaD;
    uint16_t    crc;
} PACKED SrxlRssiPacket;

// Parameter Config
typedef struct SrxlParamPacket
{
    SrxlHeader  hdr;
    uint8_t     request;
    uint8_t     destDevID;
    uint32_t    paramID;
    uint32_t    paramVal;
    uint16_t    crc;
} PACKED SrxlParamPacket;

// VTX Data
typedef struct SrxlVtxData
{
    uint8_t band;       // VTX Band (0 = Fatshark, 1 = Raceband, 2 = E, 3 = B, 4 = A)
    uint8_t channel;    // VTX Channel (0-7)
    uint8_t pit;        // Pit/Race mode (0 = Race, 1 = Pit). Race = normal power, Pit = reduced power
    uint8_t power;      // VTX Power (0 = Off, 1 = 1mw to 14mW, 2 = 15mW to 25mW, 3 = 26mW to 99mW,
                        // 4 = 100mW to 299mW, 5 = 300mW to 600mW, 6 = 601mW+, 7 = manual control)
    uint16_t powerDec;  // VTX Power as a decimal 1mw/unit
    uint8_t region;     // Region (0 = USA, 1 = EU)
} PACKED SrxlVtxData;

// Forward Programming Data
typedef struct SrxlFwdPgmData
{
    uint8_t rfu[3];     // 0 for now -- used to word-align data
    uint8_t data[FWD_PGM_MAX_DATA_SIZE];
} PACKED SrxlFwdPgmData;

// Channel Data
typedef struct SrxlChannelData
{
    int8_t    rssi;         // Best RSSI when sending channel data, or dropout RSSI when sending failsafe data
    uint16_t  frameLosses;  // Total lost frames (or fade count when sent from Remote Rx to main Receiver)
    uint32_t  mask;         // Set bits indicate that channel data with the corresponding index is present
    uint16_t  values[32];   // Channel values, shifted to full 16-bit range (32768 = mid-scale); lowest 2 bits RFU
} PACKED SrxlChannelData;

// Control Data
typedef struct SrxlControlData
{
    uint8_t cmd;
    uint8_t replyID;
    union
    {
        SrxlChannelData channelData;    // Used for Channel Data and Failsafe Channel Data commands
        SrxlVtxData     vtxData;        // Used for VTX commands
        SrxlFwdPgmData  fpData;         // Used to pass forward programming data to an SRXL device
    };
} PACKED SrxlControlData;

typedef struct SrxlControlPacket
{
    SrxlHeader      hdr;
    SrxlControlData payload;
//  uint16_t        crc;    // NOTE: Since this packet is variable-length, we can't use this value anyway
} PACKED SrxlControlPacket;

// SRXL Packets
typedef union
{
    SrxlHeader          header;
    SrxlBindPacket      bind;
    SrxlHandshakePacket handshake;
    SrxlTelemetryPacket telemetry;
    SrxlRssiPacket      rssi;
    SrxlParamPacket     parameter;
    SrxlControlPacket   control;
    uint8_t             raw[SRXL_MAX_BUFFER_SIZE];
} SrxlPacket;

// SRXL full device identifier -- SRXL Device ID with bus number
typedef union
{
    struct
    {
        uint8_t deviceID;
        uint8_t busIndex;
    };
    uint16_t word;
} PACKED SrxlFullID;

// Restore packing back to default
#ifndef SRXL_EXTERNAL_PACKED
#undef PACKED
#ifndef __GNUC__
#pragma pack(pop)
#endif
#endif

// Global vars
extern SrxlChannelData srxlChData;
extern SrxlTelemetryData srxlTelemData;
extern SrxlVtxData srxlVtxData;

// Include config here, after all typedefs that might be needed within it
#include "spm_srxl_config.h"

#ifndef FALLTHROUGH
#define FALLTHROUGH
#endif

#if !defined(SRXL_NUM_OF_BUSES)
#error "SRXL_NUM_OF_BUSES must be defined in spm_srxl_config.h!"
#elif SRXL_NUM_OF_BUSES <= 0
#error "SRXL_NUM_OF_BUSES must be defined in spm_srxl_config.h!"
#elif SRXL_NUM_OF_BUSES > 1
#define SRXL_IS_HUB
#endif
#define SRXL_ALL_BUSES          ((1u << SRXL_NUM_OF_BUSES) - 1)
#define SRXL_MAX_RCVRS          (2 * SRXL_NUM_OF_BUSES)
#ifndef SRXL_CRC_OPTIMIZE_MODE  // NOTE: This should be set in spm_srxl_config.h
#define SRXL_CRC_OPTIMIZE_MODE  SRXL_CRC_OPTIMIZE_SPEED
#endif

#define RSSI_RCVD_NONE  (0)
#define RSSI_RCVD_DBM   (1)
#define RSSI_RCVD_PCT   (2)
#define RSSI_RCVD_BOTH  (3)

// Internal types
typedef enum
{
    SrxlState_Disabled,             // Default state before initialized or if bus is subsequently disabled
    SrxlState_ListenOnStartup,      // Wait 50ms to see if anything is already talking (i.e. we probably browned out)
    SrxlState_SendHandshake,        // Call when handshake should be sent every 50ms
    SrxlState_ListenForHandshake,   // Wait at least 150ms more for handshake request
    SrxlState_Running,              // Normal run state
    SrxlState_SendTelemetry,        // Send telemetry reply when requested
    SrxlState_SendVTX,              // Send VTX packet when needed
    SrxlState_SendEnterBind,
    SrxlState_SendBoundDataReport,
    SrxlState_SendSetBindInfo,
} SrxlState;

//#ifdef SRXL_IS_HUB
typedef struct SrxlRcvrEntry
{
    uint8_t     deviceID;       // SRXL device ID of the receiver
    uint8_t     busBits;        // Supports 8 buses, with each bit corresponding to busIndex (bit 0 = bus 0, bit 7 = bus 7)
    uint8_t     info;           // Info bits reported during handshake - See SRXL_DEVINFO_XXX mask bits in header
    uint8_t     rssiRcvd;       // 0 = none, 1 = dBm, 2 = percent, 3 = both dBm and percent
    int8_t      rssi_dBm;       // Latest RSSI dBm value reported by receiver (negative, varies with receiver type)
    int8_t      rssi_Pct;       // Latest RSSI percent range estimate reported by receiver (0-100)
    uint16_t    fades;          // Latest number of fades reported for a given receiver
    uint32_t    channelMask;    // Latest channel mask for channels provided in channel data packet (0 during fade)
} SrxlRcvrEntry;

typedef struct SrxlReceiverInfo
{
    SrxlRcvrEntry   rcvr[SRXL_MAX_RCVRS];       // Stats for each receiver, filled when ch data is received
    SrxlRcvrEntry*  rcvrSorted[SRXL_MAX_RCVRS]; // Pointers to receiver entries sorted in telemetry range order
    uint8_t         rcvrSortInsert;             // Index into rcvrSorted where full-range telem rcvrs should be inserted
    uint8_t         rcvrCount;                  // Number of entries in rcvr[] and rcvrSorted[]
    uint8_t         rxBusBits;
    int8_t          bestRssi_dBm;
    int8_t          bestRssi_Pct;
    uint8_t         lossCountdown;  // Reset to lossHoldCount when frame is good, and decrement for each consecutive
                                    // frame loss -- when we get to 0, convert lossHoldCount frame losses to a hold
    uint8_t         lossHoldCount;  // Consecutive frame losses required to count as hold
    uint16_t        frameLosses;    // Increment each time all receivers are in frame loss -- if 45
                                    // consecutive, subtract those and increment holds
    uint16_t        holds;          // Increment each time 45 or more consecutive frames are lost (but don't keep
                                    // incrementing once in that state)
    SrxlRcvrEntry*  pTelemRcvr;     // Pointer to current assigned telemetry receiver (used for checking
                                    // for fade to know when to switch)
    SrxlRcvrEntry*  pBindRcvr;      // Pointer to receiver that we told to Enter Bind Mode (used to
                                    // process Bound Data Report and send Set Bind Info)
} SrxlReceiverStats;
//#endif

typedef struct SrxlDevEntry
{
    uint8_t deviceID;
    uint8_t priority;   // Requested telemetry priority of this device
    uint8_t info;       // Refer to SRXL_DEVINFO_XXX mask bits in header
    uint8_t rfu;
} SrxlDevEntry;

typedef struct SrxlTxFlags
{
    unsigned int enterBind : 1;
    unsigned int getBindInfo : 1;
    unsigned int setBindInfo : 1;
    unsigned int broadcastBindInfo : 1;
    unsigned int reportBindInfo : 1;
    unsigned int sendVtxData : 1;
    unsigned int sendFwdPgmData : 1;
} SrxlTxFlags;

typedef struct SrxlBus
{
    SrxlPacket      srxlOut;            // Transmit packet buffer
    SrxlPacket      srxlIn;             // Receive packet buffer

    SrxlState       state;              // Current state of SRXL state machine
    SrxlFullID      fullID;             // Device ID and Bus Index of this device, set during init
    uint8_t         rxDevCount;         // Number of other SRXL devices discovered via handshake
    SrxlDevEntry    rxDev[SRXL_MAX_DEVICES];    // Device entries for tracking SRXL telemetry priorities
#ifdef SRXL_INCLUDE_MASTER_CODE
    uint16_t        rxDevAge[SRXL_MAX_DEVICES]; // Telemetry age value for the associated device
#endif
    uint16_t        rxDevPrioritySum;   // Sum of priorities requested for each discovered SRXL device
    uint16_t        timeoutCount_ms;    // Milliseconds since SRXL packet was received (incremented in srxlRun)
    uint8_t         requestID;          // Device ID to poll
    uint8_t         baudSupported;      // Baud rates this device can do: 0 = 115200, 1 = 400000
    uint8_t         baudRate;           // Current baud rate: 0 = 115200, 1 = 400000
    uint8_t         frameErrCount;      // Number of consecutive missed frames
    SrxlTxFlags     txFlags;            // Pending outgoing packet types
    uint8_t         uart;               // Index number of UART tied to this SRXL bus
    SrxlRcvrEntry*  pMasterRcvr;        // Receiver entry for the bus master, if one exists
    bool            master;             // True if this device is the bus master on this bus
    bool            initialized;        // True when this SRXL bus is initialized
} SrxlBus;

typedef struct SrxlDevice
{
    SrxlDevEntry    devEntry;   // Device info for this local device, shared across all buses.
    uint32_t        uid;        // ID statistically likely to be unique (Random, hash of serial, etc.)
    SrxlRcvrEntry*  pRcvr;      // Pointer to our receiver entry, if we're a receiver (don't set for
                                // flight controller acting as hub -- only true receiver)
    bool vtxProxy;              // Set true if this device can and should respond to VTX commands
} SrxlDevice;


// Function prototypes
bool srxlInitDevice(uint8_t deviceID, uint8_t priority, uint8_t info, uint32_t uid);
bool srxlInitBus(uint8_t busIndex, uint8_t uart, uint8_t baudSupported);
bool srxlIsBusMaster(uint8_t busIndex);
uint16_t srxlGetTimeoutCount_ms(uint8_t busIndex);
uint8_t srxlGetDeviceID(uint8_t busIndex);
bool srxlParsePacket(uint8_t busIndex, uint8_t *packet, uint8_t length);
void srxlRun(uint8_t busIndex, int16_t timeoutDelta_ms);
bool srxlEnterBind(uint8_t bindType, bool broadcast);
bool srxlSetBindInfo(uint8_t bindType, uint64_t guid, uint32_t uid);
void srxlOnFrameError(uint8_t busIndex);
SrxlFullID srxlGetTelemetryEndpoint(void);
bool srxlSetVtxData(SrxlVtxData *pVtxData);
bool srxlPassThruFwdPgm(uint8_t *pData, uint8_t length);
void srxlSetHoldThreshold(uint8_t countdownReset);
void srxlClearCommStats(void);
bool srxlUpdateCommStats(bool isFade);

#ifdef __cplusplus
} // extern "C"
#endif

#ifdef SRXL_INCLUDE_MASTER_CODE
// NOTE: Most user applications should not be an SRXL2 bus master, so master-specific code is not open.
// If your application requires this functionality, please inquire about this from Spektrum RC.
#include "spm_srxl_master.h"
#endif

#endif //__SRXL_H__
