#ifndef __FLASH_STM32_H
#define __FLASH_STM32_H

#ifdef __cplusplus
 extern "C" {
#endif

typedef enum
	{
	FLASH_BUSY = 1,
	FLASH_ERROR_PG,
	FLASH_ERROR_WRP,
	FLASH_ERROR_OPT,
	FLASH_COMPLETE,
	FLASH_TIMEOUT,
	FLASH_BAD_ADDRESS,
    FLASH_ERROR_PGS,
    FLASH_ERROR_PGP,
    FLASH_ERROR_PGA,
    FLASH_ERROR_PROGRAM,
    FLASH_ERROR_OPERATION
} FLASH_Status;

#define IS_FLASH_ADDRESS(ADDRESS) (((ADDRESS) >= 0x08000000) && ((ADDRESS) < 0x0807FFFF))

FLASH_Status FLASH_WaitForLastOperation(uint32 Timeout);
FLASH_Status FLASH_ErasePage(uint32 Page_Address);
FLASH_Status FLASH_ProgramHalfWord(uint32 Address, uint16 Data);
void FLASH_Unlock(void);
void FLASH_Lock(void);

#ifdef __cplusplus
}
#endif

#endif /* __FLASH_STM32_H */
