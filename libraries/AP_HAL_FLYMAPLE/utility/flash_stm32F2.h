#include "libmaple.h"
#include "util.h"
//#include "flash.h"
#include "flash_stm32.h"



typedef struct
{
  __io uint32 ACR;      /*!< FLASH access control register, Address offset: 0x00 */
  __io uint32 KEYR;     /*!< FLASH key register,            Address offset: 0x04 */
  __io uint32 OPTKEYR;  /*!< FLASH option key register,     Address offset: 0x08 */
  __io uint32 SR;       /*!< FLASH status register,         Address offset: 0x0C */
  __io uint32 CR;       /*!< FLASH control register,        Address offset: 0x10 */
  __io uint32 OPTCR;    /*!< FLASH option control register, Address offset: 0x14 */
} FLASH_TypeDef;

#define PERIPH_BASE           ((uint32)0x40000000) /*!< Peripheral base address in the alias region */
#define AHB1PERIPH_BASE       (PERIPH_BASE + 0x00020000)
#define FLASH_R_BASE          (AHB1PERIPH_BASE + 0x3C00)
#define FLASH                 ((FLASH_TypeDef *) FLASH_R_BASE)

#define FLASH_FLAG_EOP                 ((uint32)0x00000001)  /*!< FLASH End of Operation flag */
#define FLASH_FLAG_OPERR               ((uint32)0x00000002)  /*!< FLASH operation Error flag */
#define FLASH_FLAG_WRPERR              ((uint32)0x00000010)  /*!< FLASH Write protected error flag */
#define FLASH_FLAG_PGAERR              ((uint32)0x00000020)  /*!< FLASH Programming Alignment error flag */
#define FLASH_FLAG_PGPERR              ((uint32)0x00000040)  /*!< FLASH Programming Parallelism error flag  */
#define FLASH_FLAG_PGSERR              ((uint32)0x00000080)  /*!< FLASH Programming Sequence error flag  */
#define FLASH_FLAG_BSY                 ((uint32)0x00010000)  /*!< FLASH Busy flag */ 

#define FLASH_PSIZE_BYTE           ((uint32)0x00000000)
#define FLASH_PSIZE_HALF_WORD      ((uint32)0x00000100)
#define FLASH_PSIZE_WORD           ((uint32)0x00000200)
#define FLASH_PSIZE_DOUBLE_WORD    ((uint32)0x00000300)
#define CR_PSIZE_MASK              ((uint32)0xFFFFFCFF)

#define SECTOR_MASK                ((uint32)0xFFFFFF07)

/*******************  Bits definition for FLASH_CR register  ******************/
#define FLASH_CR_PG                          ((uint32)0x00000001)
#define FLASH_CR_SER                         ((uint32)0x00000002)
#define FLASH_CR_MER                         ((uint32)0x00000004)
#define FLASH_CR_SNB_0                       ((uint32)0x00000008)
#define FLASH_CR_SNB_1                       ((uint32)0x00000010)
#define FLASH_CR_SNB_2                       ((uint32)0x00000020)
#define FLASH_CR_SNB_3                       ((uint32)0x00000040)
#define FLASH_CR_PSIZE_0                     ((uint32)0x00000100)
#define FLASH_CR_PSIZE_1                     ((uint32)0x00000200)
#define FLASH_CR_STRT                        ((uint32)0x00010000)
#define FLASH_CR_EOPIE                       ((uint32)0x01000000)
#define FLASH_CR_LOCK                        ((uint32)0x80000000)

#define FLASH_KEY1			((uint32)0x45670123)
#define FLASH_KEY2			((uint32)0xCDEF89AB)

/* Delay definition */
#define EraseTimeout		((uint32)0x00000FFF)
#define ProgramTimeout		((uint32)0x0000001F)

#define VoltageRange_1        ((uint8)0x00)  /*!< Device operating range: 1.8V to 2.1V */
#define VoltageRange_2        ((uint8)0x01)  /*!<Device operating range: 2.1V to 2.7V */
#define VoltageRange_3        ((uint8)0x02)  /*!<Device operating range: 2.7V to 3.6V */
#define VoltageRange_4        ((uint8)0x03)  /*!<Device operating range: 2.7V to 3.6V + External Vpp */

#define IS_VOLTAGERANGE(RANGE)(((RANGE) == VoltageRange_1) || \
                               ((RANGE) == VoltageRange_2) || \
                               ((RANGE) == VoltageRange_3) || \
                               ((RANGE) == VoltageRange_4))                                                                                                               

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
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PROGRAM,
  *                       FLASH_ERROR_WRP, FLASH_ERROR_OPERATION or FLASH_COMPLETE.
  */
FLASH_Status FLASH_GetStatus(void)
{
  FLASH_Status flashstatus = FLASH_COMPLETE;

  if((FLASH->SR & FLASH_FLAG_BSY) == FLASH_FLAG_BSY)
  {
    flashstatus = FLASH_BUSY;
  }
  else
  {
    if((FLASH->SR & FLASH_FLAG_WRPERR) != 0)
    {
      flashstatus = FLASH_ERROR_WRP;
    }
    else
    {
      if((FLASH->SR & 0xEF) != 0)
      {
        flashstatus = FLASH_ERROR_PROGRAM;
      }
      else
      {
        if((FLASH->SR & FLASH_FLAG_OPERR) != 0)
        {
          flashstatus = FLASH_ERROR_OPERATION;
        }
        else
        {
          flashstatus = FLASH_COMPLETE;
        }
      }
    }
  }
  /* Return the FLASH Status */
  return flashstatus;
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
  * @brief  Erases a specified FLASH Sector.
  *
  * @param  FLASH_Sector: The Sector number to be erased.
  *          This parameter can be a value between FLASH_Sector_0 and FLASH_Sector_11
  *
  * @param  VoltageRange: The device voltage range which defines the erase parallelism.
  *          This parameter can be one of the following values:
  *            @arg VoltageRange_1: when the device voltage range is 1.8V to 2.1V,
  *                                  the operation will be done by byte (8-bit)
  *            @arg VoltageRange_2: when the device voltage range is 2.1V to 2.7V,
  *                                  the operation will be done by half word (16-bit)
  *            @arg VoltageRange_3: when the device voltage range is 2.7V to 3.6V,
  *                                  the operation will be done by word (32-bit)
  *            @arg VoltageRange_4: when the device voltage range is 2.7V to 3.6V + External Vpp,
  *                                  the operation will be done by double word (64-bit)
  *
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PROGRAM,
  *                       FLASH_ERROR_WRP, FLASH_ERROR_OPERATION or FLASH_COMPLETE.
  */
FLASH_Status FLASH_EraseSector(uint32 FLASH_Sector, uint8 VoltageRange)
{
  uint32 tmp_psize = 0x0;
  FLASH_Status status = FLASH_COMPLETE;

  /* Check the parameters */
  //assert_param(IS_FLASH_SECTOR(FLASH_Sector));
  //assert_param(IS_VOLTAGERANGE(VoltageRange));

  if(VoltageRange == VoltageRange_1)
  {
     tmp_psize = FLASH_PSIZE_BYTE;
  }
  else if(VoltageRange == VoltageRange_2)
  {
    tmp_psize = FLASH_PSIZE_HALF_WORD;
  }
  else if(VoltageRange == VoltageRange_3)
  {
    tmp_psize = FLASH_PSIZE_WORD;
  }
  else
  {
    tmp_psize = FLASH_PSIZE_DOUBLE_WORD;
  }
  /* Wait for last operation to be completed */
  status = FLASH_WaitForLastOperation(EraseTimeout);

  if(status == FLASH_COMPLETE)
  {
    /* if the previous operation is completed, proceed to erase the sector */
    FLASH->CR &= CR_PSIZE_MASK;
    FLASH->CR |= tmp_psize;
    FLASH->CR &= SECTOR_MASK;
    FLASH->CR |= FLASH_CR_SER | FLASH_Sector;
    FLASH->CR |= FLASH_CR_STRT;

    /* Wait for last operation to be completed */
    status = FLASH_WaitForLastOperation(EraseTimeout);

    /* if the erase operation is completed, disable the SER Bit */
    FLASH->CR &= (~FLASH_CR_SER);
    FLASH->CR &= SECTOR_MASK;
  }
  /* Return the Erase Status */
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
	int Page_Offset = Page_Address - 0x08000000;
	uint32 FLASH_Sector;

	if(Page_Offset < 0x10000) {
		FLASH_Sector = Page_Offset / 0x4000;
	} else if(Page_Offset < 0x20000) {
		FLASH_Sector = 4;
	} else {
		FLASH_Sector = 4 + Page_Offset / 0x20000;
	}
	
	return FLASH_EraseSector(8 * FLASH_Sector, VoltageRange_4);
}



/**
  * @brief  Programs a half word (16-bit) at a specified address.
  * @note   This function must be used when the device voltage range is from 2.1V to 3.6V.
  * @param  Address: specifies the address to be programmed.
  *         This parameter can be any address in Program memory zone or in OTP zone.
  * @param  Data: specifies the data to be programmed.
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PROGRAM,
  *                       FLASH_ERROR_WRP, FLASH_ERROR_OPERATION or FLASH_COMPLETE.
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
			FLASH->CR &= CR_PSIZE_MASK;
			FLASH->CR |= FLASH_PSIZE_HALF_WORD;
			FLASH->CR |= FLASH_CR_PG;

			*(__io uint16*)Address = Data;

			/* Wait for last operation to be completed */
  		status = FLASH_WaitForLastOperation(ProgramTimeout);

			if(status != FLASH_TIMEOUT)
			{
			  /* if the program operation is completed, disable the PG Bit */
			  FLASH->CR &= (~FLASH_CR_PG);
			}
		}
	}

  /* Return the Program Status */
  return status;
}


/**
  * @brief  Unlocks the FLASH control register access
  * @param  None
  * @retval None
  */
void FLASH_Unlock(void)
{
  if((FLASH->CR & FLASH_CR_LOCK) != 0)
  {
    /* Authorize the FLASH Registers access */
    FLASH->KEYR = FLASH_KEY1;
    FLASH->KEYR = FLASH_KEY2;
  }  
}

/**
  * @brief  Locks the FLASH control register access
  * @param  None
  * @retval None
  */
void FLASH_Lock(void)
{
  /* Set the LOCK Bit to lock the FLASH Registers access */
  FLASH->CR |= FLASH_CR_LOCK;
}
