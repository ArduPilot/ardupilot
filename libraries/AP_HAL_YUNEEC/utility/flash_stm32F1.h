#include <AP_HAL_Boards.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_FLYMAPLE

#include "libmaple.h"
#include "util.h"
#include "flash.h"
#include "flash_stm32.h"

#ifndef __get_bits	
// add macros missing in maple 0.0.12
#define __set_bits(addr, mask)   (*(volatile uint32*)(addr) |= (uint32)(mask))
#define __clear_bits(addr, mask) (*(volatile uint32*)(addr) &= (uint32)~(mask))
#define __get_bits(addr, mask)   (*(volatile uint32*)(addr) & (uint32)(mask))

#define __read(reg)              (*(volatile uint32*)(reg))
#define __write(reg, value)      (*(volatile uint32*)(reg) = (value))
#endif

#undef FLASH_BASE
#define FLASH_BASE		0x40022000
#define FLASH_SR		(FLASH_BASE + 0x0C)
	//	 FLASH_ACR		+ 0x00
#define FLASH_KEYR		(FLASH_BASE + 0x04)
	//	 FLASH_OPTKEYR	+ 0x08
#define FLASH_CR		(FLASH_BASE + 0x10)
#define FLASH_AR		(FLASH_BASE + 0x14)
	//	 FLASH_RESERVED	+ 0x18
	//	 FLASH_OBR		+ 0x1C
	//	 FLASH_WRPR		+ 0x20

#define FLASH_FLAG_BSY		((uint32)0x00000001)  /*!< FLASH Busy flag */
#define FLASH_FLAG_PGERR	((uint32)0x00000004)  /*!< FLASH Program error flag */
#define FLASH_FLAG_WRPRTERR	((uint32)0x00000010)  /*!< FLASH Write protected error flag */
#define FLASH_FLAG_EOP		((uint32)0x00000020)  /*!< FLASH End of Operation flag */
#define FLASH_FLAG_OPTERR	((uint32)0x00000001)  /*!< FLASH Option Byte error flag */

/* Flash Control Register bits */
#define CR_PG_Set			((uint32)0x00000001)
#define CR_PG_Reset			((uint32)0x00001FFE)
#define CR_PER_Set			((uint32)0x00000002)
#define CR_PER_Reset		((uint32)0x00001FFD)
//#define CR_MER_Set			((uint32)0x00000004)
//#define CR_MER_Reset		((uint32)0x00001FFB)
//#define CR_OPTPG_Set		((uint32)0x00000010)
//#define CR_OPTPG_Reset		((uint32)0x00001FEF)
//#define CR_OPTER_Set		((uint32)0x00000020)
//#define CR_OPTER_Reset		((uint32)0x00001FDF)
#define CR_STRT_Set			((uint32)0x00000040)
#define CR_LOCK_Set			((uint32)0x00000080)

#define FLASH_KEY1			((uint32)0x45670123)
#define FLASH_KEY2			((uint32)0xCDEF89AB)

/* Delay definition */
#define EraseTimeout		((uint32)0x00000FFF)
#define ProgramTimeout		((uint32)0x0000001F)

/**
  * @brief  Inserts a time delay.
  * @param  None
  * @retval None
  */
static void delay(void)
{
	__io uint32 i = 0;
	for(i = 0xFF; i != 0; i--) { }
}

/**
  * @brief  Returns the FLASH Status.
  * @param  None
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PG,
  *   FLASH_ERROR_WRP or FLASH_COMPLETE
  */
FLASH_Status FLASH_GetStatus(void)
{
	if(__get_bits(FLASH_SR, FLASH_FLAG_BSY) == FLASH_FLAG_BSY)
		return FLASH_BUSY;

	if(__get_bits(FLASH_SR, FLASH_FLAG_PGERR) != 0)
		return FLASH_ERROR_PG;

	if(__get_bits(FLASH_SR, FLASH_FLAG_WRPRTERR) != 0 )
		return FLASH_ERROR_WRP;

	if(__get_bits(FLASH_SR, FLASH_FLAG_OPTERR) != 0 )
		return FLASH_ERROR_OPT;

	return FLASH_COMPLETE;
}

/**
  * @brief  Waits for a Flash operation to complete or a TIMEOUT to occur.
  * @param  Timeout: FLASH progamming Timeout
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *   FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_WaitForLastOperation(uint32 Timeout)
{ 
	FLASH_Status status;

	/* Check for the Flash Status */
	status = FLASH_GetStatus();
	/* Wait for a Flash operation to complete or a TIMEOUT to occur */
	while((status == FLASH_BUSY) && (Timeout != 0x00))
	{
		delay();
		status = FLASH_GetStatus();
		Timeout--;
	}
	if (Timeout == 0)
		status = FLASH_TIMEOUT;
	/* Return the operation status */
	return status;
}

/**
  * @brief  Erases a specified FLASH page.
  * @param  Page_Address: The page address to be erased.
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PG,
  *   FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status FLASH_ErasePage(uint32 Page_Address)
{
	FLASH_Status status = FLASH_COMPLETE;
	/* Check the parameters */
	ASSERT(IS_FLASH_ADDRESS(Page_Address));
	/* Wait for last operation to be completed */
	status = FLASH_WaitForLastOperation(EraseTimeout);
  
	if(status == FLASH_COMPLETE)
	{
		/* if the previous operation is completed, proceed to erase the page */
		__set_bits(FLASH_CR, CR_PER_Set);
		__write(FLASH_AR, Page_Address);
		__set_bits(FLASH_CR, CR_STRT_Set);

		/* Wait for last operation to be completed */
		status = FLASH_WaitForLastOperation(EraseTimeout);
		if(status != FLASH_TIMEOUT)
		{
			/* if the erase operation is completed, disable the PER Bit */
			__clear_bits(FLASH_CR, ~CR_PER_Reset);
		}
		__write(FLASH_SR, (FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR));
	}
	/* Return the Erase Status */
	return status;
}

/**
  * @brief  Programs a half word at a specified address.
  * @param  Address: specifies the address to be programmed.
  * @param  Data: specifies the data to be programmed.
  * @retval FLASH Status: The returned value can be: FLASH_ERROR_PG,
  *   FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT. 
  */
FLASH_Status FLASH_ProgramHalfWord(uint32 Address, uint16 Data)
{
	FLASH_Status status = FLASH_BAD_ADDRESS;

	if (IS_FLASH_ADDRESS(Address))
	{
		/* Wait for last operation to be completed */
		status = FLASH_WaitForLastOperation(ProgramTimeout);
		if(status == FLASH_COMPLETE)
		{
			/* if the previous operation is completed, proceed to program the new data */
			__set_bits(FLASH_CR, CR_PG_Set);
			*(__io uint16*)Address = Data;
			/* Wait for last operation to be completed */
			status = FLASH_WaitForLastOperation(ProgramTimeout);
			if(status != FLASH_TIMEOUT)
			{
				/* if the program operation is completed, disable the PG Bit */
				__clear_bits(FLASH_CR, ~CR_PG_Reset);
			}
			__write(FLASH_SR, (FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR));
		}
	}
	return status;
}

/**
  * @brief  Unlocks the FLASH Program Erase Controller.
  * @param  None
  * @retval None
  */
void FLASH_Unlock(void)
{
  /* Authorize the FPEC Access */
  __write(FLASH_KEYR, FLASH_KEY1);
  __write(FLASH_KEYR, FLASH_KEY2);
}

/**
  * @brief  Locks the FLASH Program Erase Controller.
  * @param  None
  * @retval None
  */
void FLASH_Lock(void)
{
  /* Set the Lock Bit to lock the FPEC and the FCR */
	__set_bits(FLASH_CR, CR_LOCK_Set);
}

#endif
