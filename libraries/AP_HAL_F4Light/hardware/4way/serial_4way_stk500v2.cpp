/*
(c) 2017 night_ghost@ykoctpa.ru
 
based on: Cleanflight

 * have a look at https://github.com/sim-/tgy/blob/master/boot.inc
 * for info about the stk500v2 implementation
 */

#include <stdbool.h>
#include <stdint.h>
#include "serial_4way.h"


#ifdef  USE_SERIAL_4WAY_BLHELI_INTERFACE

#include <hal.h>
#include "serial_4way_impl.h"
#include "serial_4way_stk500v2.h"

#ifdef USE_SERIAL_4WAY_SK_BOOTLOADER

#define BIT_LO_US (32) //32uS
#define BIT_HI_US (2*BIT_LO_US)

static uint8_t StkInBuf[16];

#define STK_BIT_TIMEOUT 250 // micro seconds
#define STK_WAIT_TICKS (1000 / STK_BIT_TIMEOUT)    // per ms
#define STK_WAITCYLCES (STK_WAIT_TICKS * 35)       // 35ms
#define STK_WAITCYLCES_START (STK_WAIT_TICKS / 2)  // 0.5 ms
#define STK_WAITCYLCES_EXT (STK_WAIT_TICKS * 5000) //5 s

#define  WaitPinLo  while (ESC_IS_HI) {if (micros() > timeout_timer) goto timeout;}
#define  WaitPinHi  while (ESC_IS_LO) {if (micros() > timeout_timer) goto timeout;}

static uint32_t LastBitTime;
static uint32_t HiLoTsh;

static uint8_t SeqNumber;
static uint8_t StkCmd;
static uint8_t ckSumIn;
static uint8_t ckSumOut;

// used STK message constants from ATMEL AVR - Application note
#define MESSAGE_START           0x1B
#define TOKEN                   0x0E

#define CMD_SIGN_ON             0x01
#define CMD_LOAD_ADDRESS        0x06
#define CMD_CHIP_ERASE_ISP      0x12
#define CMD_PROGRAM_FLASH_ISP   0x13
#define CMD_READ_FLASH_ISP      0x14
#define CMD_PROGRAM_EEPROM_ISP  0x15
#define CMD_READ_EEPROM_ISP     0x16
#define CMD_READ_SIGNATURE_ISP  0x1B
#define CMD_SPI_MULTI           0x1D

#define STATUS_CMD_OK           0x00

#define CmdFlashEepromRead 0xA0
#define EnterIspCmd1 0xAC
#define EnterIspCmd2 0x53
#define signature_r  0x30

#define IRQ_OFF // dummy
#define IRQ_ON  // dummy

static void StkSendByte(uint8_t dat)
{
    ckSumOut ^= dat;
    for (uint8_t i = 0; i < 8; i++)    { // TODO каналы моторов на таймере так что время нужно формировать через PWM
        if (dat & 0x01) {
            // 1-bits are encoded as 64.0us high, 72.8us low (135.8us total).
            ESC_SET_HI;
            delay_us(BIT_HI_US);
            ESC_SET_LO;
            delay_us(BIT_HI_US);
        } else {
            // 0-bits are encoded as 27.8us high, 34.5us low, 34.4us high, 37.9 low (134.6us total)
            ESC_SET_HI;
            delay_us(BIT_LO_US);
            ESC_SET_LO;
            delay_us(BIT_LO_US);
            ESC_SET_HI;
            delay_us(BIT_LO_US);
            ESC_SET_LO;
            delay_us(BIT_LO_US);
        }
        dat >>= 1;
    }
}

static void StkSendPacketHeader(void)
{
    IRQ_OFF;
    ESC_OUTPUT;
    StkSendByte(0xFF);
    StkSendByte(0xFF);
    StkSendByte(0x7F);
    ckSumOut = 0;
    StkSendByte(MESSAGE_START);
    StkSendByte(++SeqNumber);
}

static void StkSendPacketFooter(void)
{
    StkSendByte(ckSumOut);
    ESC_SET_HI;
    delay_us(BIT_LO_US);
    ESC_INPUT;
    IRQ_ON;
}



static int8_t ReadBit(void)
{
    uint32_t btimer = micros();
    uint32_t timeout_timer = btimer + STK_BIT_TIMEOUT;
    WaitPinLo;
    WaitPinHi;
    LastBitTime = micros() - btimer;
    if (LastBitTime <= HiLoTsh) {
        timeout_timer = timeout_timer + STK_BIT_TIMEOUT;
        WaitPinLo;
        WaitPinHi;
        //lo-bit
        return 0;
    } else {
        return 1;
    }
timeout:
    return -1;
}

static uint8_t ReadByte(uint8_t *bt)
{
    *bt = 0;
    for (uint8_t i = 0; i < 8; i++) {
        int8_t bit = ReadBit();
        if (bit == -1) goto timeout;
        if (bit == 1) {
            *bt |= (1 << i);
        }
    }
    ckSumIn ^=*bt;
    return 1;
timeout:
    return 0;
}

static uint8_t StkReadLeader(void)
{

    // Reset learned timing
    HiLoTsh = BIT_HI_US + BIT_LO_US;

    // Wait for the first bit
    uint32_t waitcycl; //250uS each

    if((StkCmd == CMD_PROGRAM_EEPROM_ISP) || (StkCmd == CMD_CHIP_ERASE_ISP)) {
         waitcycl = STK_WAITCYLCES_EXT;
    } else if(StkCmd == CMD_SIGN_ON) {
        waitcycl = STK_WAITCYLCES_START;
    } else {
        waitcycl= STK_WAITCYLCES;
    }
    for ( ; waitcycl >0 ; waitcycl--) {
        //check is not timeout
        if (ReadBit() >- 1) break;
    }

    //Skip the first bits
    if (waitcycl == 0){
        goto timeout;
    }

    for (uint8_t i = 0; i < 10; i++) {
        if (ReadBit() == -1) goto timeout;
    }

    // learn timing
    HiLoTsh = (LastBitTime >> 1) + (LastBitTime >> 2);

    // Read until we get a 0 bit
    int8_t bit;
    do {
        bit = ReadBit();
        if (bit == -1) goto timeout;
    } while (bit > 0);
    return 1;
timeout:
    return 0;
}

static uint8_t StkRcvPacket(uint8_t *pstring)
{
    uint8_t bt = 0;
    uint8_16_u Len;

    IRQ_OFF;
    if (!StkReadLeader()) goto Err;
    ckSumIn=0;
    if (!ReadByte(&bt) || (bt != MESSAGE_START)) goto Err;
    if (!ReadByte(&bt) || (bt != SeqNumber)) goto Err;
    ReadByte(&Len.bytes[1]);
    if (Len.bytes[1] > 1) goto Err;
    ReadByte(&Len.bytes[0]);
    if (Len.bytes[0] < 1) goto Err;
    if (!ReadByte(&bt) || (bt != TOKEN)) goto Err;
    if (!ReadByte(&bt) || (bt != StkCmd)) goto Err;
    if (!ReadByte(&bt) || (bt != STATUS_CMD_OK)) goto Err;
    for (uint16_t i = 0; i < (Len.word - 2); i++)
    {
         if (!ReadByte(pstring)) goto Err;
         pstring++;
    }
    ReadByte(&bt);
    if (ckSumIn != 0) goto Err;
    IRQ_ON;
    return 1;
Err:
    IRQ_ON;
    return 0;
}

static uint8_t _CMD_SPI_MULTI_EX(volatile uint8_t * ResByte,uint8_t Cmd,uint8_t AdrHi,uint8_t AdrLo)
{
    StkCmd= CMD_SPI_MULTI;
    StkSendPacketHeader();
    StkSendByte(0); // hi byte Msg len
    StkSendByte(8); // lo byte Msg len
    StkSendByte(TOKEN);
    StkSendByte(CMD_SPI_MULTI);
    StkSendByte(4); // NumTX
    StkSendByte(4); // NumRX
    StkSendByte(0); // RxStartAdr
    StkSendByte(Cmd);    // {TxData} Cmd
    StkSendByte(AdrHi); // {TxData} AdrHi
    StkSendByte(AdrLo);    // {TxData} AdrLoch
    StkSendByte(0); // {TxData} 0
    StkSendPacketFooter();
    if (StkRcvPacket(StkInBuf)) { // NumRX + 3
         if ((StkInBuf[0] == 0x00) && ((StkInBuf[1] == Cmd)||(StkInBuf[1] == 0x00)/* ignore  zero returns */) &&(StkInBuf[2] == 0x00)) {
            *ResByte = StkInBuf[3];
         }
         return 1;
    }
    return 0;
}

static uint8_t _CMD_LOAD_ADDRESS(ioMem_t *pMem)
{
    // ignore 0xFFFF
    // assume address is set before and we read or write the immediately following package
    if((pMem->D_FLASH_ADDR_H == 0xFF) && (pMem->D_FLASH_ADDR_L == 0xFF)) return 1;
    StkCmd = CMD_LOAD_ADDRESS;
    StkSendPacketHeader();
    StkSendByte(0); // hi byte Msg len
    StkSendByte(5); // lo byte Msg len
    StkSendByte(TOKEN);
    StkSendByte(CMD_LOAD_ADDRESS);
    StkSendByte(0);
    StkSendByte(0);
    StkSendByte(pMem->D_FLASH_ADDR_H);
    StkSendByte(pMem->D_FLASH_ADDR_L);
    StkSendPacketFooter();
    return StkRcvPacket(StkInBuf);
}

static uint8_t _CMD_READ_MEM_ISP(ioMem_t *pMem)
{
    uint8_t LenHi;
    if (pMem->D_NUM_BYTES>0) {
        LenHi=0;
    } else {
        LenHi=1;
    }
    StkSendPacketHeader();
    StkSendByte(0); // hi byte Msg len
    StkSendByte(4); // lo byte Msg len
    StkSendByte(TOKEN);
    StkSendByte(StkCmd);
    StkSendByte(LenHi);
    StkSendByte(pMem->D_NUM_BYTES);
    StkSendByte(CmdFlashEepromRead);
    StkSendPacketFooter();
    return (StkRcvPacket(pMem->D_PTR_I));
}

static uint8_t _CMD_PROGRAM_MEM_ISP(ioMem_t *pMem)
{
    uint8_16_u Len;
    uint8_t LenLo = pMem->D_NUM_BYTES;
    uint8_t LenHi;
    if (LenLo) {
        LenHi = 0;
        Len.word = LenLo + 10;
    } else {
        LenHi = 1;
        Len.word = 256 + 10;
    }
    StkSendPacketHeader();
    StkSendByte(Len.bytes[1]); // high byte Msg len
    StkSendByte(Len.bytes[0]); // low byte Msg len
    StkSendByte(TOKEN);
    StkSendByte(StkCmd);
    StkSendByte(LenHi);
    StkSendByte(LenLo);
    StkSendByte(0); // mode
    StkSendByte(0); // delay
    StkSendByte(0); // cmd1
    StkSendByte(0); // cmd2
    StkSendByte(0); // cmd3
    StkSendByte(0); // poll1
    StkSendByte(0); // poll2
    do {
        StkSendByte(*pMem->D_PTR_I);
        pMem->D_PTR_I++;
        LenLo--;
    } while (LenLo);
    StkSendPacketFooter();
    return StkRcvPacket(StkInBuf);
}

uint8_t Stk_SignOn(void)
{
    StkCmd=CMD_SIGN_ON;
    StkSendPacketHeader();
    StkSendByte(0); // hi byte Msg len
    StkSendByte(1); // lo byte Msg len
    StkSendByte(TOKEN);
    StkSendByte(CMD_SIGN_ON);
    StkSendPacketFooter();
    return StkRcvPacket(StkInBuf);
}

uint8_t Stk_ConnectEx(uint8_32_u *pDeviceInfo)
{
    if (Stk_SignOn()) {
        if (_CMD_SPI_MULTI_EX(&pDeviceInfo->bytes[1], signature_r,0,1)) {
            if (_CMD_SPI_MULTI_EX(&pDeviceInfo->bytes[0], signature_r,0,2)) {
                return 1;
            }
        }
    }
    return 0;
}

uint8_t Stk_Chip_Erase(void)
{
    StkCmd = CMD_CHIP_ERASE_ISP;
    StkSendPacketHeader();
    StkSendByte(0); // high byte Msg len
    StkSendByte(7); // low byte Msg len
    StkSendByte(TOKEN);
    StkSendByte(CMD_CHIP_ERASE_ISP);
    StkSendByte(20); // ChipErase_eraseDelay atmega8
    StkSendByte(0); // ChipErase_pollMethod atmega8
    StkSendByte(0xAC);
    StkSendByte(0x88);
    StkSendByte(0x13);
    StkSendByte(0x76);
    StkSendPacketFooter();
    return StkRcvPacket(StkInBuf);
}

uint8_t Stk_ReadFlash(ioMem_t *pMem)
{
    if (_CMD_LOAD_ADDRESS(pMem)) {
        StkCmd = CMD_READ_FLASH_ISP;
        return (_CMD_READ_MEM_ISP(pMem));
    }
    return 0;
}


uint8_t Stk_ReadEEprom(ioMem_t *pMem)
{
    if (_CMD_LOAD_ADDRESS(pMem)) {
        StkCmd = CMD_READ_EEPROM_ISP;
        return (_CMD_READ_MEM_ISP(pMem));
    }
    return 0;
}

uint8_t Stk_WriteFlash(ioMem_t *pMem)
{
    if (_CMD_LOAD_ADDRESS(pMem)) {
        StkCmd = CMD_PROGRAM_FLASH_ISP;
        return (_CMD_PROGRAM_MEM_ISP(pMem));
    }
    return 0;
}

uint8_t Stk_WriteEEprom(ioMem_t *pMem)
{
    if (_CMD_LOAD_ADDRESS(pMem)) {
        StkCmd = CMD_PROGRAM_EEPROM_ISP;
        return (_CMD_PROGRAM_MEM_ISP(pMem));
    }
    return 0;
}
#endif
#endif
