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

#include <string.h>
#include <stdint.h>

#include "spm_srxl.h"

/// LOCAL TYPES AND CONSTANTS ///

#if(SRXL_CRC_OPTIMIZE_MODE == SRXL_CRC_OPTIMIZE_SPEED)
const uint16_t srxlCRCTable[] =
{
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,

    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,

    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,

    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};
#endif

#define SRXL_TELEM_SUPPRESS_MAX (100)

/// PUBLIC VARIABLES ///

SrxlChannelData srxlChData;
SrxlTelemetryData srxlTelemData;
SrxlVtxData srxlVtxData = {0, 0, 1, 0, 0, 1};

/// LOCAL VARIABLES ///

static SrxlDevice srxlThisDev;
static SrxlBus srxlBus[SRXL_NUM_OF_BUSES];
static bool srxlChDataIsFailsafe = false;
static bool srxlTelemetryPhase = false;
#ifdef SRXL_INCLUDE_MASTER_CODE
static uint32_t srxlFailsafeChMask;  // Tracks all active channels for use during failsafe transmission
#endif
static SrxlBindData srxlBindInfo;
static SrxlReceiverStats srxlRx;
static uint16_t srxlTelemSuppressCount;

#ifdef SRXL_INCLUDE_FWD_PGM_CODE
static SrxlFullID srxlFwdPgmDevice;  // Device that should accept Forward Programming connection by default
static uint8_t srxlFwdPgmBuffer[FWD_PGM_MAX_DATA_SIZE];
static uint8_t srxlFwdPgmBufferLength;
#endif

// Include additional header and externs if using STM32 hardware acceleration
#if(SRXL_CRC_OPTIMIZE_MODE > SRXL_CRC_OPTIMIZE_SIZE)
#ifdef __cplusplus
extern "C"
{
#endif
#if(SRXL_CRC_OPTIMIZE_MODE == SRXL_CRC_OPTIMIZE_STM_HAL)
#if(SRXL_STM_TARGET_FAMILY == SRXL_STM_TARGET_F7)
#include "stm32f7xx_hal.h"
#else
#include "stm32f3xx_hal.h"  // Default to F3 if not given
#endif
    extern CRC_HandleTypeDef hcrc;
#else
#if(SRXL_STM_TARGET_FAMILY == SRXL_STM_TARGET_F7)
#error "STM32F7 targets not yet supported for register-based STM HW optimization"
#else
#include "stm32f30x.h"  // Default to F3 if not given
#endif
#endif
#ifdef __cplusplus
}
#endif
#endif

/// LOCAL HELPER FUNCTIONS ///

// Compute SRXL CRC over packet buffer (assumes length is correctly set)
static uint16_t srxlCrc16(uint8_t* packet)
{
    uint16_t crc = 0;                // Seed with 0
    uint8_t length = packet[2] - 2;  // Exclude 2 CRC bytes at end of packet from the length

    if(length <= SRXL_MAX_BUFFER_SIZE - 2)
    {
#if(SRXL_CRC_OPTIMIZE_MODE == SRXL_CRC_OPTIMIZE_SIZE)
        // Use bitwise method
        for(uint8_t i = 0; i < length; ++i)
        {
            crc = crc ^ ((uint16_t)packet[i] << 8);
            for(int b = 0; b < 8; b++)
            {
                if(crc & 0x8000)
                    crc = (crc << 1) ^ 0x1021;
                else
                    crc = crc << 1;
            }
        }
#elif(SRXL_CRC_OPTIMIZE_MODE == SRXL_CRC_OPTIMIZE_STM_HW)
        // Use direct STM32 HW CRC register access
        uint8_t* pEnd = &packet[length];
        srxlEnterCriticalSection();
#ifdef SRXL_SAVE_HW_CRC_CONTEXT
        uint32 savedPOL = CRC->POL;
        uint32 savedINIT = CRC->INIT;
        uint32 savedCR = CRC->CR;
        uint32 savedDR = CRC->DR;
#endif
        CRC->POL = 0x1021;
        CRC->INIT = 0;
        CRC->CR = 0x09;  // 16-bit polynomial, no input or output reversal, reset to init of 0
        while(packet < pEnd)
            *(uint8_t*)(CRC_BASE) = *(packet++);
        crc = (uint16_t)CRC->DR;
#ifdef SRXL_SAVE_HW_CRC_CONTEXT
        // We have to use this convoluted method to restore things because writing INIT sets DR too
        CRC->CR = 0;
        CRC->POL = 0x04C11DB7;
        CRC->INIT = savedINIT;
        CRC->DR = savedINIT;
        CRC->POL = savedDR;
        CRC->DR = 1;
        CRC->CR = savedCR;
        CRC->POL = savedPOL;
#endif
        srxlExitCriticalSection();
#elif(SRXL_CRC_OPTIMIZE_MODE == SRXL_CRC_OPTIMIZE_STM_HAL)
        // STM32f3/f7 hardware optimization using the STM32Cube libraries in HAL mode requires the following
        // configuration, set in the STM32CubeMX "Pinout & Configuration" tab under "Computing > CRC":
        //  Basic Parameters
        //    Default Polynomial State        = Disable
        //    CRC Length                      = 16-bit
        //    CRC Generating Polynomial       = X12+X5+X0
        //    Default Init Value State        = Disable
        //    Init Value For CRC Computation  = 0
        //  Advanced Parameters
        //    Input Data Inversion Mode       = None
        //    Output Data Inversion Mode      = Disable
        //    Input Data Format               = Bytes
        crc = (uint16_t)HAL_CRC_Calculate(&hcrc, (uint32_t*)packet, length);
#elif(SRXL_CRC_OPTIMIZE_MODE == SRXL_CRC_OPTIMIZE_EXTERNAL)
        crc = SRXL_CRC_CALCULATE(packet, length, crc);
#else
        // Default to table-lookup method
        uint8_t i;
        for(i = 0; i < length; ++i)
        {
            // Get indexed position in lookup table using XOR of current CRC hi byte
            uint8_t pos = (uint8_t)((crc >> 8) ^ packet[i]);
            // Shift LSB up and XOR with the resulting lookup table entry
            crc = (uint16_t)((crc << 8) ^ (uint16_t)(srxlCRCTable[pos]));
        }
#endif
    }
    return crc;
}

// Get the receiver entry for the requested bus and device ID
static inline SrxlRcvrEntry* srxlGetReceiverEntry(uint8_t busIndex, uint8_t deviceID)
{
    SrxlRcvrEntry* pRcvr = 0;
    uint8_t i;
    for(i = 0; i < srxlRx.rcvrCount; ++i)
    {
        if((srxlRx.rcvr[i].busBits & (1u << busIndex)) && (srxlRx.rcvr[i].deviceID == deviceID))
        {
            pRcvr = &srxlRx.rcvr[i];
            break;
        }
    }
    return pRcvr;
}

// Add a new receiver entry for the given device
static inline SrxlRcvrEntry* srxlAddReceiverEntry(SrxlBus* pBus, SrxlDevEntry devEntry)
{
    // Only allow receivers (or flight controllers in certain circumstances) to be added
    if(!pBus || devEntry.deviceID < 0x10 || devEntry.deviceID >= 0x40)
        return 0;

    // If we didn't previously add this receiver, add it now if we have room
    SrxlRcvrEntry* pRcvr = srxlGetReceiverEntry(pBus->fullID.busIndex, devEntry.deviceID);
    if(!pRcvr)
    {
        if(srxlRx.rcvrCount >= SRXL_MAX_RCVRS)
            return 0;

        uint8_t i = srxlRx.rcvrCount++;
        pRcvr = &srxlRx.rcvr[i];
        pRcvr->deviceID = devEntry.deviceID;
        pRcvr->busBits = (1u << pBus->fullID.busIndex);
        pRcvr->info = devEntry.info;

        // If this receiver is full-range, insert into our sorted list after the other full-range telemetry receivers
        if(pRcvr->info & SRXL_DEVINFO_TELEM_FULL_RANGE)
        {
            uint8_t n;
            for(n = i; n > srxlRx.rcvrSortInsert; --n)
                srxlRx.rcvrSorted[n] = srxlRx.rcvrSorted[n - 1];
            srxlRx.rcvrSorted[(srxlRx.rcvrSortInsert)++] = pRcvr;
        }
        // Else just tack onto the end
        else
        {
            srxlRx.rcvrSorted[i] = pRcvr;
        }
    }

    // If this new receiver is a base receiver that supports telemetry or we haven't set a default active telemetry receiver, set it
    if(!srxlRx.pTelemRcvr || (pRcvr->deviceID >= 0x20 && pRcvr->deviceID < 0x30 && (pRcvr->info & SRXL_DEVINFO_TELEM_TX_ENABLED)))
    {
        srxlRx.pTelemRcvr = pRcvr;
    }

    return pRcvr;
}

// Pick the best receiver to send telemetry on
static inline SrxlRcvrEntry* srxlChooseTelemRcvr(void)
{
    // If we only know about one receiver, set it to that
    if(srxlRx.rcvrCount == 1)
        return srxlRx.rcvrSorted[0];

    // If we were previously sending telemetry
    if(srxlRx.pTelemRcvr && srxlRx.pTelemRcvr->channelMask)
    {
        // If the current choice is not full-range
        if((srxlRx.pTelemRcvr->info & SRXL_DEVINFO_TELEM_FULL_RANGE) == 0)
        {
            // Then see if there is a full-range choice that received channel data to switch to
            uint8_t i;
            for(i = 0; i < srxlRx.rcvrSortInsert; ++i)
            {
                if(srxlRx.rcvrSorted[i]->channelMask)
                    return srxlRx.rcvrSorted[i];
            }
        }

        // Else keep using the current receiver
        return srxlRx.pTelemRcvr;
    }
    // Else just pick the first one that got channel data this past frame
    else
    {
        uint8_t i;
        for(i = 0; i < srxlRx.rcvrCount; ++i)
        {
            if(srxlRx.rcvrSorted[i]->channelMask)
                return srxlRx.rcvrSorted[i];
        }
    }

    return 0;
}

// Return pointer to device entry matching the given ID, or NULL if not found
static SrxlDevEntry* srxlGetDeviceEntry(SrxlBus* pBus, uint8_t deviceID)
{
    if(pBus)
    {
        uint8_t i;
        for(i = 0; i < pBus->rxDevCount; ++i)
        {
            if(pBus->rxDev[i].deviceID == deviceID)
                return &(pBus->rxDev[i]);
        }
    }
    return 0;
}

// Add an entry to our list of devices found on the SRXL bus (or update an entry if it already exists)
static SrxlDevEntry* srxlAddDeviceEntry(SrxlBus* pBus, SrxlDevEntry devEntry)
{
    // Don't allow broadcast or unknown device types to be added
    if(!pBus || devEntry.deviceID < 0x10 || devEntry.deviceID > 0xEF)
        return 0;

    // Limit device priority
    if(devEntry.priority > 100)
        devEntry.priority = 100;

    // Update device entry if it already exists
    SrxlDevEntry* retVal = srxlGetDeviceEntry(pBus, devEntry.deviceID);
    if(retVal)
    {
        pBus->rxDevPrioritySum -= retVal->priority;
        *retVal = devEntry;
        pBus->rxDevPrioritySum += retVal->priority;
    }
    // Else add to the list if we have room
    else if(pBus->rxDevCount < SRXL_MAX_DEVICES)
    {
        retVal = &(pBus->rxDev[pBus->rxDevCount++]);
        *retVal = devEntry;
        pBus->rxDevPrioritySum += retVal->priority;

#ifdef SRXL_INCLUDE_FWD_PGM_CODE
        // If the new device supports Forward Programming and is a base receiver or flight controller, choose as the default
        if(devEntry.info & SRXL_DEVINFO_FWD_PROG_SUPPORT)
        {
            uint8_t devType = devEntry.deviceID >> 4;
            if(devType > (srxlFwdPgmDevice.deviceID >> 4) && devType < SrxlDevType_ESC)
            {
                srxlFwdPgmDevice.deviceID = devEntry.deviceID;
                srxlFwdPgmDevice.busIndex = pBus->fullID.busIndex;
            }
        }
#endif

        // If the new device is a receiver, add to our receiver list
        if(retVal->deviceID < 0x30)
        {
            srxlAddReceiverEntry(pBus, *retVal);
        }
    }

    return retVal;
}

/// PUBLIC FUNCTIONS ///

/**
    @brief  Initialize common SRXL info for this device

    @param  deviceID:   SRXL Device ID (see section 7.1.1 of SRXL2 Spec)
    @param  priority:   Requested telemetry priority (1-100; typical is 10 per unique message type)
    @param  info:       Device info bits (see SRXL_DEVINFO_XXX bits in spm_srxl.h)
    @param  uid:        Unique 32-bit id to avoid device ID collision during handshake
    @return bool:       True if device info was successfully initialized
*/
bool srxlInitDevice(uint8_t deviceID, uint8_t priority, uint8_t info, uint32_t uid)
{
    if(deviceID < 0x10 || deviceID > 0xEF)
        return false;

    srxlThisDev.devEntry.deviceID = deviceID;
    srxlThisDev.devEntry.info = info;
    srxlThisDev.devEntry.priority = priority;
    srxlThisDev.devEntry.rfu = 0;
    srxlThisDev.uid = uid;
    srxlThisDev.vtxProxy = false;

#ifdef SRXL_INCLUDE_MASTER_CODE
    // If this device is a receiver, add to our receiver info
    if(deviceID < 0x30)
    {
        srxlInitReceiver(deviceID, info);
    }
#endif

#ifdef SRXL_INCLUDE_FWD_PGM_CODE
    // If this device is a receiver or flight controller that supports Forward Programming, set as default
    if((info & SRXL_DEVINFO_FWD_PROG_SUPPORT) && deviceID < 0x40)
    {
        srxlFwdPgmDevice.deviceID = deviceID;
        srxlFwdPgmDevice.busIndex = 0;
    }
#endif

#if(SRXL_CRC_OPTIMIZE_MODE == SRXL_CRC_OPTIMIZE_STM_HW)
    // Enable the peripheral clock for the HW CRC engine
    RCC->AHBENR |= RCC_AHBENR_CRCEN;
#endif

    return true;
}

/**
    @brief  Initialize bus settings for the given SRXL bus

    @param  busIndex:       Index into srxlBus array of bus entries
    @param  uart:           Number to identify UART to which this SRXL bus should be connected
    @param  baudSupported:  0 = 115200 baud, 1 = 400000 baud
    @return bool:           True if SRXL bus was successfully initialized
*/
bool srxlInitBus(uint8_t busIndex, uint8_t uart, uint8_t baudSupported)
{
    if(busIndex >= SRXL_NUM_OF_BUSES || !srxlThisDev.devEntry.deviceID)
        return false;

    SrxlBus* pBus = &srxlBus[busIndex];
    pBus->state = SrxlState_ListenOnStartup;
    pBus->fullID.deviceID = srxlThisDev.devEntry.deviceID;
    pBus->fullID.busIndex = busIndex;
    pBus->rxDevCount = 0;
    pBus->rxDevPrioritySum = 0;
    pBus->requestID = (srxlThisDev.devEntry.deviceID == 0x10) ? 0x11 : 0;
    pBus->baudSupported = baudSupported;
    pBus->baudRate = SRXL_BAUD_115200;
    pBus->frameErrCount = 0;
    pBus->uart = uart;
    // Default remote receiver is automatically master -- everyone else figures it out during handshake
    pBus->master = (srxlThisDev.devEntry.deviceID == 0x10);
    pBus->pMasterRcvr = (srxlThisDev.devEntry.deviceID == 0x10) ? &srxlRx.rcvr[0] : 0;
    pBus->initialized = true;

    return true;
}

/**
    @brief  See if this device is the bus master on the given bus

    @param  busIndex:   Index into srxlBus array for the desired SRXL bus
    @return bool:       True if this device is the bus master on the given SRXL bus
*/
bool srxlIsBusMaster(uint8_t busIndex)
{
    return (busIndex < SRXL_NUM_OF_BUSES && srxlBus[busIndex].master);
}

/**
    @brief  Get the current SRXL state machine timeout count for the given bus

    @param  busIndex:   Index into srxlBus array for the desired SRXL bus
    @return uint16_t:    Timeout count in ms for the given SRXL bus
*/
uint16_t srxlGetTimeoutCount_ms(uint8_t busIndex)
{
    return (busIndex < SRXL_NUM_OF_BUSES) ? srxlBus[busIndex].timeoutCount_ms : 0;
}

/**
    @brief  Get the Device ID of this device on the given bus

    @param  busIndex:   Index into srxlBus array for the desired SRXL bus
    @return uint8_t:    Device ID of this device on the given SRXL bus
*/
uint8_t srxlGetDeviceID(uint8_t busIndex)
{
    return (busIndex < SRXL_NUM_OF_BUSES) ? srxlBus[busIndex].fullID.deviceID : 0;
}

/**
    @brief  Internal send function called by srxlRun() -- do not call in user code

    @param  pBus:       Pointer to SRXL bus entry for the desired SRXL bus
    @param  srxlCmd:    Specific type of packet to send
    @param  replyID:    Device ID of the device this Send command is targeting
*/
static void srxlSend(SrxlBus* pBus, SRXL_CMD srxlCmd, uint8_t replyID)
{
    if(!pBus || !pBus->initialized)
        return;

    memset(pBus->srxlOut.raw, 0, SRXL_MAX_BUFFER_SIZE);
    pBus->srxlOut.header.srxlID = SPEKTRUM_SRXL_ID;

    // VTX Data
    if(srxlCmd == SRXL_CMD_VTX)
    {
        pBus->srxlOut.header.packetType = SRXL_CTRL_ID;
        pBus->srxlOut.header.length = SRXL_CTRL_BASE_LENGTH + sizeof(SrxlVtxData);
        pBus->srxlOut.control.payload.cmd = SRXL_CTRL_CMD_VTX;
        pBus->srxlOut.control.payload.replyID = replyID;
        pBus->srxlOut.control.payload.vtxData = srxlVtxData;
    }
#ifdef SRXL_INCLUDE_MASTER_CODE
    // Channel Data
    else if(srxlCmd == SRXL_CMD_CHANNEL || srxlCmd == SRXL_CMD_CHANNEL_FS)
    {
        pBus->srxlOut.header.packetType = SRXL_CTRL_ID;
        uint32_t channelMask;
        if(srxlCmd == SRXL_CMD_CHANNEL)
        {
            pBus->srxlOut.control.payload.cmd = SRXL_CTRL_CMD_CHANNEL;
            pBus->srxlOut.control.payload.replyID = replyID;

            channelMask = srxlChData.mask;
        }
        else // == SRXL_CMD_CHANNEL_FS
        {
            // In failsafe mode, we dont want a telemetry reply
            pBus->srxlOut.control.payload.cmd = SRXL_CTRL_CMD_CHANNEL_FS;
            pBus->srxlOut.control.payload.replyID = 0;

            channelMask = srxlFailsafeChMask;
        }

        // Set signal quality info (only a bus master sends this, so assume srxlChData contains the latest values)
        pBus->srxlOut.control.payload.channelData.rssi = srxlChData.rssi;
#ifdef SRXL_IS_HUB
        pBus->srxlOut.control.payload.channelData.frameLosses = srxlRx.frameLosses;
#else
        pBus->srxlOut.control.payload.channelData.frameLosses = srxlRx.rcvr[0].fades;
#endif

        uint8_t channelIndex = 0;
        uint32_t channelMaskBit = 1;
        for(uint8_t i = 0; i < 32; ++i, channelMaskBit <<= 1)
        {
            if(channelMask & channelMaskBit)
            {
                pBus->srxlOut.control.payload.channelData.values[channelIndex++] = srxlChData.values[i];
            }
        }

        // Set bits in packet for channels we populated, and clear those mask bits if it was part of a normal channel data command
        pBus->srxlOut.control.payload.channelData.mask = channelMask;
        if(srxlCmd == SRXL_CMD_CHANNEL)
            srxlChData.mask &= ~channelMask;

        pBus->srxlOut.header.length = SRXL_CTRL_BASE_LENGTH + 7 + (2 * channelIndex);
    }
#ifdef SRXL_INCLUDE_FWD_PGM_CODE
    // Forward Programming Pass-thru
    else if(srxlCmd == SRXL_CMD_FWDPGM)
    {
        pBus->srxlOut.header.packetType = SRXL_CTRL_ID;
        pBus->srxlOut.header.length = SRXL_CTRL_BASE_LENGTH + 3 + srxlFwdPgmBufferLength;
        pBus->srxlOut.control.payload.cmd = SRXL_CTRL_CMD_FWDPGM;
        pBus->srxlOut.control.payload.replyID = replyID;
        memcpy(pBus->srxlOut.control.payload.fpData.data, srxlFwdPgmBuffer, srxlFwdPgmBufferLength);
    }
#endif  // SRXL_INCLUDE_FWD_PGM_CODE
#endif  // SRXL_INCLUDE_MASTER_CODE
    // RSSI Data
    else if(srxlCmd == SRXL_CMD_RSSI)
    {
        pBus->srxlOut.header.packetType = SRXL_RSSI_ID;
        pBus->srxlOut.header.length = sizeof(SrxlRssiPacket);
        pBus->srxlOut.rssi.request = SRXL_RSSI_REQ_REQUEST;  // TODO: Needs to handle both directions!
        pBus->srxlOut.rssi.antennaA = 0;                     // TODO: Fill in actual data later
        pBus->srxlOut.rssi.antennaB = 0;
        pBus->srxlOut.rssi.antennaC = 0;
        pBus->srxlOut.rssi.antennaD = 0;
    }
    else if(srxlCmd == SRXL_CMD_HANDSHAKE)
    {
        pBus->srxlOut.header.packetType = SRXL_HANDSHAKE_ID;
        pBus->srxlOut.header.length = sizeof(SrxlHandshakePacket);
        pBus->srxlOut.handshake.payload.srcDevID = srxlThisDev.devEntry.deviceID;
        pBus->srxlOut.handshake.payload.destDevID = replyID;
        pBus->srxlOut.handshake.payload.priority = srxlThisDev.devEntry.priority;
        pBus->srxlOut.handshake.payload.baudSupported = pBus->baudSupported;
        pBus->srxlOut.handshake.payload.info = srxlThisDev.devEntry.info;
        pBus->srxlOut.handshake.payload.uid = srxlThisDev.uid;
    }
    else if(srxlCmd == SRXL_CMD_TELEMETRY)
    {
        srxlFillTelemetry(&pBus->srxlOut.telemetry.payload);
        pBus->srxlOut.header.packetType = SRXL_TELEM_ID;
        pBus->srxlOut.header.length = sizeof(SrxlTelemetryPacket);
        // If we successfully received a handshake from the bus master
        if(pBus->pMasterRcvr)
        {
            // If we know that a device on this bus should send it, then target that device
            if(srxlRx.pTelemRcvr && (srxlRx.pTelemRcvr->busBits & (1u << pBus->fullID.busIndex)))
                pBus->srxlOut.telemetry.destDevID = srxlRx.pTelemRcvr->deviceID;
            else
                pBus->srxlOut.telemetry.destDevID = 0;
        }
        else
        {
            // Send 0xFF to tell bus master to re-send the handshake so we know where to direct telemetry in the future
            pBus->srxlOut.telemetry.destDevID = 0xFF;
        }

#ifdef SRXL_INCLUDE_MASTER_CODE
        if(srxlRx.pTelemRcvr && (pBus->srxlOut.telemetry.destDevID == srxlRx.pTelemRcvr->deviceID))
        {
            // Don't mark telemetry as having been sent if we are sending it ourself over RF
            if(pBus->srxlOut.telemetry.destDevID != srxlThisDev.pRcvr->deviceID)
            {
                srxlTelemetrySent();
                // Clear telemetry buffer after sending so we don't repeatedly display old data
//                srxlTelemData.sensorID = srxlTelemData.secondaryID = 0;
            }
        }
#endif
    }
    else if(srxlCmd == SRXL_CMD_ENTER_BIND)
    {
        pBus->srxlOut.header.packetType = SRXL_BIND_ID;
        pBus->srxlOut.header.length = sizeof(SrxlBindPacket);
        pBus->srxlOut.bind.request = SRXL_BIND_REQ_ENTER;
        pBus->srxlOut.bind.deviceID = replyID;
        pBus->srxlOut.bind.data.type = DSMX_11MS;
        pBus->srxlOut.bind.data.options = (replyID != 0xFF) ? SRXL_BIND_OPT_TELEM_TX_ENABLE | SRXL_BIND_OPT_BIND_TX_ENABLE : 0;
        pBus->srxlOut.bind.data.guid = 0;
        pBus->srxlOut.bind.data.uid = 0;
    }
    else if(srxlCmd == SRXL_CMD_REQ_BIND)
    {
        pBus->srxlOut.header.packetType = SRXL_BIND_ID;
        pBus->srxlOut.header.length = sizeof(SrxlBindPacket);
        pBus->srxlOut.bind.request = SRXL_BIND_REQ_STATUS;
        pBus->srxlOut.bind.deviceID = replyID;
        memset(&(pBus->srxlOut.bind.data), 0, sizeof(SrxlBindData));
    }
    else if(srxlCmd == SRXL_CMD_SET_BIND)
    {
        pBus->srxlOut.header.packetType = SRXL_BIND_ID;
        pBus->srxlOut.header.length = sizeof(SrxlBindPacket);
        pBus->srxlOut.bind.request = SRXL_BIND_REQ_SET_BIND;
        pBus->srxlOut.bind.deviceID = replyID;
        pBus->srxlOut.bind.data = srxlBindInfo;
    }
    else if(srxlCmd == SRXL_CMD_BIND_INFO)
    {
        pBus->srxlOut.header.packetType = SRXL_BIND_ID;
        pBus->srxlOut.header.length = sizeof(SrxlBindPacket);
        pBus->srxlOut.bind.request = SRXL_BIND_REQ_BOUND_DATA;
        pBus->srxlOut.bind.deviceID = replyID;
        pBus->srxlOut.bind.data = srxlBindInfo;
    }

    // Compute CRC over entire SRXL packet (excluding the 2 CRC bytes at the end)
    uint16_t crc = srxlCrc16(pBus->srxlOut.raw);

    // Add CRC to packet in big-endian byte order
    pBus->srxlOut.raw[pBus->srxlOut.header.length - 2] = (crc >> 8) & 0xFF;
    pBus->srxlOut.raw[pBus->srxlOut.header.length - 1] = crc & 0xFF;

    // Send the packet out over the assigned UART
    srxlSendOnUart(pBus->uart, pBus->srxlOut.raw, pBus->srxlOut.header.length);
}

/**
    @brief  Parse an SRXL packet received on the given SRXL bus UART

    @param  busIndex:   Index of SRXL bus state information entry in the srxlBus array
    @param  packet:     Pointer to received packet data
    @param  length:     Length in bytes of received packet data
    @return bool:       True if a valid packet was received, else false
*/
bool srxlParsePacket(uint8_t busIndex, uint8_t* packet, uint8_t length)
{
    // Validate parameters
    if(busIndex >= SRXL_NUM_OF_BUSES || !packet || length < 5 || length > SRXL_MAX_BUFFER_SIZE)
        return false;

    // Validate SRXL ID and length
    if(packet[0] != SPEKTRUM_SRXL_ID || packet[2] != length)
        return false;

    // Validate checksum
    uint16_t crc = srxlCrc16(packet);
    if((((uint16_t)packet[length - 2] << 8) | packet[length - 1]) != crc)
        return false;

    // Copy packet into our unioned buffer to avoid "strict aliasing" violations
    SrxlBus* pBus = &srxlBus[busIndex];
    SrxlPacket* pRx = &(pBus->srxlIn);
    memcpy(pRx, packet, length);

    // Handle restart with ongoing communications -- bump to run state
    pBus->timeoutCount_ms = 0;  // TODO: Should we clear this even if packet isn't valid?
    if(pBus->state < SrxlState_Running && pRx->header.packetType != SRXL_HANDSHAKE_ID)
        pBus->state = SrxlState_Running;

    // Parse the specific data
    switch(pRx->header.packetType)
    {
    case SRXL_CTRL_ID:  // 0xCD
    {
        SrxlControlData* pCtrlData = &(pRx->control.payload);

        // Validate command
        if(pCtrlData->cmd > SRXL_CTRL_CMD_FWDPGM)
            break;

        // VTX
        if(pCtrlData->cmd == SRXL_CTRL_CMD_VTX)
        {
            if(srxlSetVtxData(&pCtrlData->vtxData))
                srxlOnVtx(&srxlVtxData);
            if(pCtrlData->replyID == pBus->fullID.deviceID || pCtrlData->replyID == 0xFF || srxlThisDev.vtxProxy)
            {
                // TODO: Should we ack this somehow
            }
        }
#ifdef SRXL_INCLUDE_FWD_PGM_CODE
        // Forward Programming
        else if(pCtrlData->cmd == SRXL_CTRL_CMD_FWDPGM)
        {
            if(pCtrlData->replyID == pBus->fullID.deviceID)
            {
                memcpy(srxlFwdPgmBuffer, pCtrlData->fpData.data, pRx->header.length - 3 - SRXL_CTRL_BASE_LENGTH);
                srxlFwdPgmBufferLength = length;
                if(pCtrlData->replyID == srxlFwdPgmDevice.deviceID)
                {
                    // Handle Forward Programming command locally
                    srxlOnFwdPgm(srxlFwdPgmBuffer, srxlFwdPgmBufferLength);
                }
                else if(srxlFwdPgmDevice.deviceID && srxlBus[srxlFwdPgmDevice.busIndex].master)
                {
                    // Pass it on through to the next target
                    srxlBus[srxlFwdPgmDevice.busIndex].txFlags.sendFwdPgmData = 1;
                }
            }
        }
#endif
        // Channel Data or Failsafe Data
        else
        {
            bool isFailsafe = (pCtrlData->cmd == SRXL_CTRL_CMD_CHANNEL_FS);
            srxlChData.rssi = pCtrlData->channelData.rssi;
            srxlChData.frameLosses = pCtrlData->channelData.frameLosses;
            if(pBus->pMasterRcvr)
            {
                if(pCtrlData->channelData.rssi < 0)
                {
                    pBus->pMasterRcvr->rssi_dBm = pCtrlData->channelData.rssi;
                    pBus->pMasterRcvr->rssiRcvd |= RSSI_RCVD_DBM;
                }
                else
                {
                    pBus->pMasterRcvr->rssi_Pct = pCtrlData->channelData.rssi;
                    pBus->pMasterRcvr->rssiRcvd |= RSSI_RCVD_PCT;
                }
                // If the receiver is sending alternating dBm/%, then use that as phase
                if(pBus->pMasterRcvr->rssiRcvd == RSSI_RCVD_BOTH)
                {
                    srxlTelemetryPhase = pCtrlData->channelData.rssi >= 0;
                }
                pBus->pMasterRcvr->fades = pCtrlData->channelData.frameLosses;
                pBus->pMasterRcvr->channelMask = isFailsafe ? 0 : pCtrlData->channelData.mask;
                srxlRx.rxBusBits |= pBus->pMasterRcvr->busBits;
            }

            // Only save received channel values to srxlChData if it's normal channel data or we're in a hold condition
            if(!isFailsafe || !srxlRx.lossCountdown)
            {
                uint8_t channelIndex = 0;
                uint32_t channelMaskBit = 1;
                uint8_t i;
                for(i = 0; i < 32 && channelMaskBit <= pCtrlData->channelData.mask; ++i, channelMaskBit <<= 1)
                {
                    if(pCtrlData->channelData.mask & channelMaskBit)
                    {
                        srxlChData.values[i] = pCtrlData->channelData.values[channelIndex++];
                        srxlChData.mask |= channelMaskBit;
                    }
                }
            }

            srxlChDataIsFailsafe = isFailsafe;  // TODO: Can we still assume this???
            srxlReceivedChannelData(&(pCtrlData->channelData), isFailsafe);

            // Figure out what type of reply packet to send, if any
            if(pCtrlData->replyID == 0)
            {
                if(pBus->txFlags.enterBind)
                {
                    pBus->state = SrxlState_SendEnterBind;
                    pBus->txFlags.enterBind = 0;
                }
                else if(pBus->txFlags.setBindInfo)
                {
                    if(srxlRx.pBindRcvr)  // TODO: Double-check this logic
                    {
                        pBus->requestID = srxlRx.pBindRcvr->deviceID;
                        pBus->state = SrxlState_SendSetBindInfo;
                    }
                    pBus->txFlags.setBindInfo = 0;
                }
                else if(pBus->txFlags.broadcastBindInfo)
                {
                    pBus->requestID = 0xFF;
                    pBus->state = SrxlState_SendSetBindInfo;
                    pBus->txFlags.broadcastBindInfo = 0;
                }
            }
            else if(pCtrlData->replyID == pBus->fullID.deviceID)
            {
                pBus->state = SrxlState_SendTelemetry;
            }
        }
        break;
    }
    case SRXL_HANDSHAKE_ID:  // 0x21
    {
        if(length < sizeof(SrxlHandshakePacket))
            return false;

        // If this is an unprompted handshake (dest == 0) from a higher device ID, then we're the master
        SrxlHandshakeData* pHandshake = &(pRx->handshake.payload);
        if((pHandshake->destDevID == 0) && (pHandshake->srcDevID > pBus->fullID.deviceID))
        {
            // Send a reply immediately to get the slave to shut up
            pBus->state = SrxlState_SendHandshake;
            pBus->requestID = pHandshake->srcDevID;
            pBus->baudSupported = SRXL_SUPPORTED_BAUD_RATES;
            srxlRun(busIndex, 0);
            pBus->requestID = pBus->fullID.deviceID;
            pBus->state = SrxlState_SendHandshake;
            pBus->master = true;
            pBus->pMasterRcvr = &srxlRx.rcvr[0];
        }

        // Add this device to our list of discovered devices
        SrxlDevEntry newDev = {.deviceID = pHandshake->srcDevID, .priority = pHandshake->priority, .info = pHandshake->info};
        srxlAddDeviceEntry(pBus, newDev);

        // Bus master needs to track responses and poll next device
        if(pBus->master)
        {
            // Keep track of baud rates supported
            pBus->baudSupported &= pHandshake->baudSupported;

            // Make sure the state machine is set to poll -- this will eventually reach broadcast address (0xFF)
            pBus->state = SrxlState_SendHandshake;
        }
        // Broadcast handshake sets the agreed upon baud rate for this bus
        else if(pHandshake->destDevID == 0xFF)
        {
            // Get bus master receiver entry (if it's a flight controller, add it now as a receiver on this bus)
            if(newDev.deviceID >= 0x30 && newDev.deviceID < 0x40)
            {
                pBus->pMasterRcvr = srxlAddReceiverEntry(pBus, newDev);
            }
            else
            {
                pBus->pMasterRcvr = srxlGetReceiverEntry(busIndex, newDev.deviceID);
            }

            // Set baud rate and advance to run state
            pBus->baudRate = pHandshake->baudSupported;
            if(pBus->baudRate & SRXL_BAUD_400000)
            {
                srxlChangeBaudRate(pBus->uart, 400000);  // Only alternate rate supported for now...
            }
            pBus->state = SrxlState_Running;
        }
        // Normal Handshake destined for this device should be replied to -- else ignore
        else
        {
            if(pHandshake->destDevID == pBus->fullID.deviceID && pBus->state != SrxlState_SendHandshake)
            {
                pBus->requestID = pHandshake->srcDevID;
                pBus->state = SrxlState_SendHandshake;
            }
            else
            {
                pBus->state = SrxlState_ListenForHandshake;
            }
        }

        break;
    }
    case SRXL_PARAM_ID:  // 0x50
    {
        // TODO: Add later
        break;
    }
    case SRXL_RSSI_ID:  // 0x55
    {
        // TODO: Add later
        break;
    }
    case SRXL_BIND_ID:  // 0x41
    {
        if(length < sizeof(SrxlBindPacket))
            return false;

        SrxlBindPacket* pBindInfo = &(pRx->bind);

        // If this is a bound data report
        if(pBindInfo->request == SRXL_BIND_REQ_BOUND_DATA)
        {
            // Call the user-defined callback -- if returns true, bind all other receivers
            SrxlFullID boundID;
            boundID.deviceID = pBindInfo->deviceID;
            boundID.busIndex = busIndex;
            if(srxlOnBind(boundID, pBindInfo->data))
            {
                // Update the bind info
                srxlBindInfo.type = pBindInfo->data.type;
                if(pBindInfo->data.options & SRXL_BIND_OPT_TELEM_TX_ENABLE)
                {
                    SrxlRcvrEntry* pNewTelem = srxlGetReceiverEntry(busIndex, pBindInfo->deviceID);
                    if(pNewTelem || !srxlRx.pTelemRcvr)
                        srxlRx.pTelemRcvr = pNewTelem;
                }
                srxlBindInfo.options = 0;  // Disable telemetry and Discovery reply when setting other receivers
                srxlBindInfo.guid = pBindInfo->data.guid;
                srxlBindInfo.uid = pBindInfo->data.uid;

                // Try to set bind info for all other receivers on other buses to match it
                uint8_t b;
                for(b = 0; b < SRXL_NUM_OF_BUSES; ++b)
                {
                    if(b == busIndex)
                        continue;
                    srxlBus[b].txFlags.broadcastBindInfo = 1;
                }
            }
        }
        // If this bind packet is directed at us
        else if(pBindInfo->deviceID == pBus->fullID.deviceID || pBindInfo->deviceID == 0xFF)
        {
            // Check for Enter Bind Mode (only valid if sent to a specific receiver)
            if(pBindInfo->request == SRXL_BIND_REQ_ENTER)
            {
#ifdef SRXL_INCLUDE_MASTER_CODE
                srxlBindInfo.type = pBindInfo->data.type;
                srxlBindInfo.options = pBindInfo->data.options;
                srxlBindInfo.guid = 0;
                srxlBindInfo.uid = 0;
                srxlTryToBind(srxlBindInfo);
#endif
            }
            else if(pBindInfo->request == SRXL_BIND_REQ_STATUS && srxlThisDev.pRcvr)
            {
                // TODO: Fill in data if we didn't just bind?
                pBus->txFlags.reportBindInfo = 1;
            }
            // Handle set bind info request
            else if(pBindInfo->request == SRXL_BIND_REQ_SET_BIND)
            {
                srxlBindInfo = pBindInfo->data;
#ifdef SRXL_INCLUDE_MASTER_CODE
                if(pBus->fullID.deviceID < 0x30)
                    srxlTryToBind(srxlBindInfo);
#endif
            }
        }

        break;
    }
    case SRXL_TELEM_ID:  // 0x80
    {
        if(length < sizeof(SrxlTelemetryPacket))
            return false;

        // NOTE: This data should be sent by exactly one telemetry device in response to a bus master request,
        //       so it is safe to update the global pTelemRcvr here even though this is a bus-specific function.

        SrxlTelemetryPacket* pTelem = &(pRx->telemetry);
        memcpy(&srxlTelemData, &pTelem->payload, sizeof(srxlTelemData));
        // If the telemetry destination is set to broadcast, that indicates a request to re-handshake
        if(pBus->master && pTelem->destDevID == 0xFF)
        {
            // If the master only found one device, don't poll again -- just tell the requesting device who we are
            pBus->requestID = pBus->rxDevCount > 1 ? pBus->fullID.deviceID : 0xFF;
            pBus->state = SrxlState_SendHandshake;
        }
        // If the incoming telemetry is destined for us, then we need to figure out who should send it over RF
        else if(pTelem->destDevID == pBus->fullID.deviceID)
        {
            // This needs different logic for hubs versus endpoints
#ifdef SRXL_IS_HUB
            if(srxlRx.pTelemRcvr == 0)
            {
                srxlRx.pTelemRcvr = srxlChooseTelemRcvr();
            }
#else
            srxlRx.pTelemRcvr = srxlThisDev.pRcvr;
#endif
            // Enable this device's telemetry tx based on whether we are the chosen telemetry receiver
#ifdef SRXL_INCLUDE_MASTER_CODE
            srxlSetTelemetryTxEnable(srxlRx.pTelemRcvr && (srxlRx.pTelemRcvr == srxlThisDev.pRcvr));
#endif
        }
        // Else turn off our telemetry and that of any receivers we might reply to via our own telemetry
        else
        {
            srxlRx.pTelemRcvr = 0;
#ifdef SRXL_INCLUDE_MASTER_CODE
            srxlSetTelemetryTxEnable(false);
#endif
        }

        srxlTelemSuppressCount = 0;
#ifdef SRXL_INCLUDE_MASTER_CODE
        srxlSuppressInternalTelemetry(&pTelem->payload);
#endif
        break;
    }
    default:
    {
        break;
    }
    }

    // Run state machine for slave devices after each received packet
    if(!pBus->master)
    {
        srxlRun(busIndex, 0);
    }

    return true;
}

/**
    @brief  Run the SRXL state machine after each receive or rx timeout

    @param  busIndex:           Index of SRXL bus state information entry in the srxlBus array
    @param  timeoutDelta_ms:    Number of milliseconds to increment receive timeout if a timeout
                                occured, or <= 0 to clear timeout count upon packet receive.
*/
void srxlRun(uint8_t busIndex, int16_t timeoutDelta_ms)
{
    SrxlBus* pBus = &srxlBus[busIndex];
    if(busIndex >= SRXL_NUM_OF_BUSES || !pBus->initialized || pBus->state == SrxlState_Disabled)
        return;

    // Check receive timeout and advance state if needed
    if(timeoutDelta_ms > 0)
    {
        pBus->timeoutCount_ms += timeoutDelta_ms;
        if(pBus->timeoutCount_ms >= 30000)
            pBus->timeoutCount_ms = 30000;

        if(pBus->timeoutCount_ms >= 50)
        {
            // After startup delay of 50ms, switch to handshake send or listen based on device unit ID
            if(pBus->state == SrxlState_ListenOnStartup)
            {
                pBus->state = (pBus->fullID.deviceID & 0x0F) ? SrxlState_ListenForHandshake : SrxlState_SendHandshake;
            }
            // Reset non-master device back to startup conditions if 50ms elapses with no communications
            else if(!pBus->master && pBus->state >= SrxlState_Running)
            {
                srxlChangeBaudRate(pBus->uart, 115200);
                pBus->baudRate = SRXL_BAUD_115200;
                pBus->timeoutCount_ms = 0;
                pBus->requestID = 0;  // Change back to 0 to indicate unprompted handshake from slave device
                if(pBus->pMasterRcvr)
                {
                    pBus->pMasterRcvr->channelMask = 0;
                    pBus->pMasterRcvr->fades = 0xFFFF;
                    pBus->pMasterRcvr->rssi_Pct = 0;
                    pBus->pMasterRcvr->rssi_dBm = -1;
                }
                pBus->state = SrxlState_ListenOnStartup;
            }
        }
    }
    else
    {
        pBus->timeoutCount_ms = 0;
    }

    if(!pBus->master)
    {
        // Non-master actions for the given state
        switch(pBus->state)
        {
        case SrxlState_SendHandshake:
        {
            srxlSend(pBus, SRXL_CMD_HANDSHAKE, pBus->requestID);
            break;
        }
        case SrxlState_SendTelemetry:
        {
            srxlSend(pBus, SRXL_CMD_TELEMETRY, pBus->requestID);
            pBus->state = SrxlState_Running;
            break;
        }
        case SrxlState_SendVTX:
        {
            srxlSend(pBus, SRXL_CMD_VTX, pBus->requestID);
            pBus->state = SrxlState_Running;
            break;
        }
        case SrxlState_SendEnterBind:
        {
            if(srxlRx.pBindRcvr && (srxlRx.pBindRcvr != srxlThisDev.pRcvr))
            {
                srxlSend(pBus, SRXL_CMD_ENTER_BIND, srxlRx.pBindRcvr->deviceID);
            }
            else
            {
                srxlBindInfo.options = 0;
                srxlSend(pBus, SRXL_CMD_ENTER_BIND, 0xFF);
            }
            pBus->state = SrxlState_Running;
            break;
        }
        case SrxlState_SendSetBindInfo:
        {
            srxlSend(pBus, SRXL_CMD_SET_BIND, pBus->requestID);
            pBus->state = SrxlState_Running;
            break;
        }
        case SrxlState_SendBoundDataReport:
        {
            srxlSend(pBus, SRXL_CMD_BIND_INFO, pBus->fullID.deviceID);
            pBus->state = SrxlState_Running;
            break;
        }
        case SrxlState_Running:
        default:
        {
            return;
        }
        }
    }
#ifdef SRXL_INCLUDE_MASTER_CODE
    else
    {
        srxlRunMaster(pBus);
    }
#endif  // SRXL_INCLUDE_MASTER_CODE
}

/**
    @brief  Tell the "best" receiver to enter bind mode, either locally or via SRXL command

    @param  bindType: One of the possible bind status types to use when binding -- NOTE: The transmitter may ignore this
    @param  broadcast: True if this is a local request that should tell all connected receivers to enter bind
    @return bool: True if a receiver was told to enter bind mode; else false
*/
bool srxlEnterBind(uint8_t bindType, bool broadcast)
{
    srxlRx.pBindRcvr = 0;
    if(broadcast && srxlThisDev.pRcvr)
    {
        srxlRx.pBindRcvr = srxlThisDev.pRcvr;
    }
    else if(srxlRx.pTelemRcvr)
    {
        srxlRx.pBindRcvr = srxlRx.pTelemRcvr;
    }
    else if(srxlRx.rcvrCount > 0 && srxlRx.rcvrSorted[0]->deviceID < 0x30)
    {
        srxlRx.pBindRcvr = srxlRx.rcvrSorted[0];
    }

    if(srxlRx.pBindRcvr)
    {
#ifdef SRXL_INCLUDE_MASTER_CODE
        // Local bind
        if(srxlRx.pBindRcvr == srxlThisDev.pRcvr)
        {
            srxlBindInfo.type = bindType;
            srxlBindInfo.options = SRXL_BIND_OPT_BIND_TX_ENABLE;
            srxlBindInfo.guid = 0;
            srxlBindInfo.uid = 0;
            if(srxlRx.pBindRcvr == srxlRx.pTelemRcvr)
                srxlBindInfo.options |= SRXL_BIND_OPT_TELEM_TX_ENABLE;
            srxlTryToBind(srxlBindInfo);
            if(broadcast)
            {
                uint8_t b;
                for(b = 0; b < SRXL_NUM_OF_BUSES; ++b)
                {
                    srxlBus[b].txFlags.enterBind = 1;
                }
            }
            return true;
        }
#endif // SRXL_INCLUDE_MASTER_CODE

        // Remote bind
        uint8_t i;
        for(i = 0; i < SRXL_NUM_OF_BUSES; ++i)
        {
            if((1u << i) & srxlRx.pBindRcvr->busBits)
            {
                srxlBus[i].txFlags.enterBind = 1;
                return true;
            }
        }
    }

    return false;
}

/**
    @brief  Public function to set bind info for the system, either locally or via SRXL commands

    @param  bindType: Type of bind requested for this receiver or all receivers
    @param  guid: Transmitter GUID to bind the receiver to
    @param  uid: Unique ID provided by transmitter upon initial bind (can be 0 if unknown)
    @return bool: True if bind info was successfully set for the destination device; else false
*/
bool srxlSetBindInfo(uint8_t bindType, uint64_t guid, uint32_t uid)
{
    if(guid == 0)
        return false;

    // Set bind info, with options defaulted to 0
    srxlBindInfo.type = bindType;
    srxlBindInfo.options = SRXL_BIND_OPT_US_POWER; // Request US power, with no guarantee it's supported
    srxlBindInfo.guid = guid;
    srxlBindInfo.uid = uid ? uid : srxlThisDev.uid;

#ifdef SRXL_INCLUDE_MASTER_CODE
    // If we are a receiver
    if(srxlThisDev.pRcvr)
    {
        // Bind locally, which will result in no further packets since options == 0
        srxlTryToBind(srxlBindInfo);
    }
#endif

    // Broadcast this bind info on all SRXL buses
    uint8_t b;
    for(b = 0; b < SRXL_NUM_OF_BUSES; ++b)
    {
        srxlBus[b].txFlags.broadcastBindInfo = 1;
    }

    return true;
}

/**
    @brief  Public function to call from the user UART code when a UART frame error occurs

    @param  busIndex: Index of SRXL bus state information entry in the srxlBus array
*/
void srxlOnFrameError(uint8_t busIndex)
{
    SrxlBus* pBus = &srxlBus[busIndex];
    if(busIndex >= SRXL_NUM_OF_BUSES || !pBus->initialized)
        return;

    ++(pBus->frameErrCount);
    if(pBus->master)
    {
        // TODO: If master (i.e. remote receiver 0x10), cause break condition to force reset?
        return;
    }

    if(pBus->state == SrxlState_ListenOnStartup || pBus->state == SrxlState_ListenForHandshake)
    {
        // Wait for multiple frame breaks before trying to change baud rate
        if(pBus->frameErrCount < 3)
            return;

        // Try the next higher baud rate
        switch(pBus->baudRate)
        {
        case SRXL_BAUD_115200:
        {
            if(pBus->baudSupported & SRXL_BAUD_400000)
            {
                srxlChangeBaudRate(pBus->uart, 400000);
                pBus->baudRate = SRXL_BAUD_400000;
                break;
            }
            FALLTHROUGH;
            // else fall thru...
        }
        case SRXL_BAUD_400000:
        default:
        {
            // TODO: Cause break condition to force reset of everyone? Or just keep cycling?

            srxlChangeBaudRate(pBus->uart, 115200);
            pBus->baudRate = SRXL_BAUD_115200;
            break;
        }
        }
        pBus->frameErrCount = 0;
    }
    else
    {
        // TODO: Handle frame error during normal comm -- probably caused by collision on bus?
    }
}

SrxlFullID srxlGetTelemetryEndpoint(void)
{
    SrxlFullID retVal = {{0}};
    if(srxlRx.pTelemRcvr)
    {
        retVal.deviceID = srxlRx.pTelemRcvr->deviceID;
        retVal.busIndex = srxlRx.pTelemRcvr->busBits;
    }
    return retVal;
}

bool srxlSetVtxData(SrxlVtxData* pVtxData)
{
    if(!pVtxData)
        return false;

    // Update VTX data, ignoring values marked as unchanged
    if(pVtxData->band != 0xFF)
        srxlVtxData.band = pVtxData->band;
    if(pVtxData->channel != 0xFF)
        srxlVtxData.channel = pVtxData->channel;
    if(pVtxData->pit != 0xFF)
        srxlVtxData.pit = pVtxData->pit;
    if(pVtxData->power != 0xFF || pVtxData->powerDec != 0xFFFF)
        srxlVtxData.power = pVtxData->power;
    if(pVtxData->powerDec != 0xFFFF)
        srxlVtxData.powerDec = pVtxData->powerDec;
    if(pVtxData->region != 0xFF)
        srxlVtxData.region = pVtxData->region;

    uint8_t b;
    for(b = 0; b < SRXL_NUM_OF_BUSES; ++b)
    {
        srxlBus[b].txFlags.sendVtxData = 1;
    }

    return true;
}

void srxlSetHoldThreshold(uint8_t countdownReset)
{
    srxlRx.lossHoldCount = (countdownReset > 1) ? countdownReset : 45;
}

void srxlClearCommStats(void)
{
    srxlRx.holds = 0;
    srxlRx.frameLosses = 0;
    srxlRx.lossCountdown = srxlRx.lossHoldCount + 1;
}

// Return true on failsafe hold
bool srxlUpdateCommStats(bool isFade)
{
    srxlRx.rxBusBits = 0;
    if(srxlTelemetryPhase)
    {
        srxlRx.bestRssi_dBm = -128;
        srxlRx.bestRssi_Pct = 0;
    }

    uint8_t i;
    for(i = 0; i < srxlRx.rcvrCount; ++i)
    {
        if(srxlRx.rcvr[i].channelMask)
        {
            srxlRx.lossCountdown = srxlRx.lossHoldCount + 1;

            if((srxlRx.rcvr[i].rssiRcvd & RSSI_RCVD_DBM) && srxlRx.bestRssi_dBm < srxlRx.rcvr[i].rssi_dBm)
                srxlRx.bestRssi_dBm = srxlRx.rcvr[i].rssi_dBm;
            if((srxlRx.rcvr[i].rssiRcvd & RSSI_RCVD_PCT) && srxlRx.bestRssi_dBm < srxlRx.rcvr[i].rssi_Pct)
                srxlRx.bestRssi_Pct = srxlRx.rcvr[i].rssi_Pct;
        }
    }

    // Set RSSI based on telemetry phase and type of telemetry received
    srxlChData.rssi = (srxlTelemetryPhase || srxlChDataIsFailsafe) ? srxlRx.bestRssi_Pct : srxlRx.bestRssi_dBm;

    // Update flight log frame losses and holds
    if(isFade && srxlRx.lossCountdown)
    {
        if(--srxlRx.lossCountdown == 0)
        {
            ++srxlRx.holds;
            srxlRx.frameLosses -= srxlRx.lossHoldCount;
        }
        else
        {
            ++srxlRx.frameLosses;
        }
    }

    static uint8_t telemFadeCount = 0;
    // If we are allowed to send telemetry by the device downstream (i.e. any slave device)
    if(srxlRx.pTelemRcvr)
    {
        if(srxlRx.pTelemRcvr->channelMask == 0 || (srxlRx.pTelemRcvr->info & SRXL_DEVINFO_TELEM_FULL_RANGE) == 0)
        {
            // If our telemetry receiver missed channel data 3 frames in a row, switch
            if(++telemFadeCount > 3)
            {
                srxlRx.pTelemRcvr = srxlChooseTelemRcvr();
#ifdef SRXL_INCLUDE_MASTER_CODE
                srxlSetTelemetryTxEnable(srxlRx.pTelemRcvr && (srxlRx.pTelemRcvr == srxlThisDev.pRcvr));
#endif
                telemFadeCount = 0;
            }
        }
        else
        {
            telemFadeCount = 0;
        }
    }
#ifdef SRXL_INCLUDE_MASTER_CODE
    // Else check to make sure we're still supposed to suppress telemetry (reset countdown when slave tells us not to send again)
    else if(++srxlTelemSuppressCount > SRXL_TELEM_SUPPRESS_MAX)
    {
        // Enable this device's telemetry tx since we stopped being told not to
        srxlRx.pTelemRcvr = srxlThisDev.pRcvr;
        srxlSetTelemetryTxEnable(srxlRx.pTelemRcvr);
    }
#endif

    // Return true while we're in hold condition (failsafe)
    return srxlRx.lossCountdown == 0;
}
