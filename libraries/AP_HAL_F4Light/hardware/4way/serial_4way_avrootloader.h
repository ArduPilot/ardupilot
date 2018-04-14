/*
 * This file got from Cleanflight.
 *
 * for info about Hagens AVRootloader:
 * http://www.mikrocontroller.net/topic/avr-bootloader-mit-verschluesselung
*/

#pragma once

// Bootloader result codes
#define brSUCCESS           0x30
#define brERRORVERIFY       0xC0
#define brERRORCOMMAND      0xC1
#define brERRORCRC          0xC2
#define brNONE              0xFF

void BL_SendBootInit(void);
uint8_t BL_ConnectEx(uint8_32_u *pDeviceInfo);
uint8_t BL_SendCMDKeepAlive(void);
uint8_t BL_PageErase(ioMem_t *pMem);
uint8_t BL_ReadEEprom(ioMem_t *pMem);
uint8_t BL_WriteEEprom(ioMem_t *pMem);
uint8_t BL_WriteFlash(ioMem_t *pMem);
uint8_t BL_ReadFlash(uint8_t interface_mode, ioMem_t *pMem);
uint8_t BL_VerifyFlash(ioMem_t *pMem);
void BL_SendCMDRunRestartBootloader(uint8_32_u *pDeviceInfo);
