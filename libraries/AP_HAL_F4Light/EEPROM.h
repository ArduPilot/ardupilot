#pragma once

#include <stdio.h>

typedef enum
{
  FLASH_BUSY = 1,
  FLASH_ERROR_PGS,
  FLASH_ERROR_PGP,
  FLASH_ERROR_PGA,
  FLASH_ERROR_WRP,
  FLASH_ERROR_PROGRAM,
  FLASH_ERROR_OPERATION,
  FLASH_COMPLETE
} FLASH_Status;

//unlock keys
#define RDP_KEY                  (0x00A5)
#define FLASH_KEY1               (0x45670123L)
#define FLASH_KEY2               (0xCDEF89ABL)
#define FLASH_OPT_KEY1           (0x08192A3BL)
#define FLASH_OPT_KEY2           (0x4C5D6E7FL)

#define FLASH_PRG_SIZE_BYTE           (0L<<8)
#define FLASH_PRG_SIZE_HALF_WORD      (1L<<8)
#define FLASH_PRG_SIZE_WORD           (2L<<8)
#define FLASH_PRG_SIZE_DOUBLE_WORD    (3L<<8)
#define FLASH_PRG_SIZE_MASK           (~FLASH_PRG_SIZE_DOUBLE_WORD)

#define OPTCR_BYTES                   ((volatile uint8_t *)&FLASH->OPTCR)
#define OPTCR_WORDS                   ((volatile uint16_t *)&FLASH->OPTCR)

#define FLASH_BIT_EOP                 (1L<<0)  // End of Operation
#define FLASH_BIT_OPERR               (1L<<1)  // operation Error
#define FLASH_BIT_WRPERR              (1L<<4)  // Write protected error
#define FLASH_BIT_PGAERR              (1L<<5)  // Programming Alignment error
#define FLASH_BIT_PGPERR              (1L<<6)  // Programming Parallelism error
#define FLASH_BIT_PGSERR              (1L<<7)  // Programming Sequence error
#define FLASH_BIT_BSY                 (1L<<16)  // Busy               

#define OB_WRP_Sector_0       (1L<<0)
#define OB_WRP_Sector_1       (1L<<1)
#define OB_WRP_Sector_2       (1L<<2)
#define OB_WRP_Sector_3       (1L<<3)
#define OB_WRP_Sector_4       (1L<<4)
#define OB_WRP_Sector_5       (1L<<5)
#define OB_WRP_Sector_6       (1L<<6)
#define OB_WRP_Sector_7       (1L<<7)
#define OB_WRP_Sector_8       (1L<<8)
#define OB_WRP_Sector_9       (1L<<9)
#define OB_WRP_Sector_10      (1L<<10)
#define OB_WRP_Sector_11      (1L<<11)
#define OB_WRP_Sector_All     (0x00000FFFL)


/* Page status definitions */
#define EEPROM_ERASED			((uint16_t)0xFFFF)	/* PAGE is empty */
#define EEPROM_RECEIVE_DATA		((uint16_t)0xEEEE)	/* PAGE is marked to receive data */
#define EEPROM_VALID_PAGE		((uint16_t)0xAAAA)	/* 1st PAGE containing valid data */

#define ADDRESS_MASK 0x3fff          // valid address always below it - 16K of EEPROM max
#define FLAGS_MASK   (~ADDRESS_MASK) // if this bits are set then we have partially written slot

/* Page full define */
enum {
	EEPROM_OK            = 0x00,
	EEPROM_OUT_SIZE      = 0x81,
	EEPROM_BAD_ADDRESS   = 0x82,
	EEPROM_BAD_FLASH     = 0x83,
	EEPROM_NOT_INIT      = 0x84,
	EEPROM_WRITE_FAILED  = 0x96,
	EEPROM_NO_VALID_PAGE = 0xAB
};

#define EEPROM_DEFAULT_DATA		0xFFFF

#define FLASH_CR_ERRIE (0x02000000) // not in stm32f4xx.h somehow

class EEPROMClass
{
public:
        typedef void (*func_t)(uint8_t page);
        
	EEPROMClass(void);

	uint16_t init(uint32_t, uint32_t, uint32_t);

        /**
          * @brief  Erases PAGE0 and PAGE1 and writes EEPROM_VALID_PAGE / 0 header to PAGE0
          * @param  PAGE0 and PAGE1 base addresses
          * @retval _status of the last operation (Flash write or erase) done during EEPROM formating
          */
	uint16_t format(void);

        /**
          * @brief  Returns the erase counter for current page
          * @param  Data: Global variable contains the read variable value
          * @retval Success or error status:
          *                     - EEPROM_OK: if erases counter return.
          *                     - EEPROM_NO_VALID_PAGE: if no valid page was found.
          */
	uint16_t erases(uint16_t *);
        /**
          * @brief      Returns the last stored variable data, if found,
          *                     which correspond to the passed virtual address
          * @param  Address: Variable virtual address
          * @param  Data: Pointer to data variable
          * @retval Success or error status:
          *           - EEPROM_OK: if variable was found
          *           - EEPROM_BAD_ADDRESS: if the variable was not found
          *           - EEPROM_NO_VALID_PAGE: if no valid page was found.
          */
	uint16_t read (uint16_t address, uint16_t *data);
	/**
          * @brief      Returns the last stored variable data, if found,
          *                     which correspond to the passed virtual address
          * @param  Address: Variable virtual address
          * @retval Data for variable or EEPROM_DEFAULT_DATA, if any errors
         */
        inline uint16_t read (uint16_t address) { 
            uint16_t data;
            read(address, &data);
            return data;
        }
        /**
          * @brief  Writes/upadtes variable data in EEPROM.
          * @param  VirtAddress: Variable virtual address
          * @param  Data: 16 bit data to be written
          * @retval Success or error status:
          *                     - FLASH_COMPLETE: on success
          *                     - EEPROM_BAD_ADDRESS: if address = 0xFFFF
          *                     - EEPROM_PAGE_FULL: if valid page is full
          *                     - EEPROM_NO_VALID_PAGE: if no valid page was found
          *                     - EEPROM_OUT_SIZE: if no empty EEPROM variables
          *                     - Flash error code: on write Flash error
          */
	uint16_t write(uint16_t address, uint16_t data);
	/**
         * @brief  Return number of variable
         * @retval Number of variables
        */
	uint16_t count(uint16_t *data);
	inline uint16_t maxcount(void) {   return (PageSize / 4)-1; }

        static FLASH_Status write_16(uint32_t addr, uint16_t data);
        static FLASH_Status write_8(uint32_t addr, uint8_t data);
        static void FLASH_Lock_check();
        static void FLASH_Unlock_dis();
        
        static inline uint32_t read_16(uint32_t addr){
            return *(__IO uint16_t*)addr;
        }

        static inline uint32_t read_32(uint32_t addr){
            return *(__IO uint32_t*)addr;
        }

	uint16_t _CheckErasePage(uint32_t, uint16_t);
        static FLASH_Status _ErasePageByAddress(uint32_t Page_Address);

        static void OB_WRPConfig(uint16_t OB_WRP, bool v);

private:
	uint32_t PageBase0; // uses 2 flash pages
	uint32_t PageBase1;
	uint32_t PageSize;
	uint16_t _status;

	uint16_t _init(void);
	uint16_t _format(void);
	static void reset_flash_errors();
	static FLASH_Status GetStatus(void);
        static FLASH_Status WaitForLastOperation(void);
        static FLASH_Status ProgramHalfWord(uint32_t Address, uint16_t Data);
        static FLASH_Status ProgramByte(uint32_t Address, uint8_t Data);
        static FLASH_Status EraseSector(uint32_t FLASH_Sector);
        static inline void Lock(void){   FLASH->CR |= FLASH_CR_LOCK; }
        static inline void Unlock(void){  if((FLASH->CR & FLASH_CR_LOCK) )  {  FLASH->KEYR = FLASH_KEY1;   FLASH->KEYR = FLASH_KEY2;  } }
        

	FLASH_Status _ErasePage(uint32_t);

	uint16_t _CheckPage(uint32_t, uint16_t);
	uint16_t _Format(void);
	uint32_t _FindValidPage(void);
	uint16_t _GetVariablesCount(uint32_t, uint16_t);
	uint16_t _PageTransfer(uint32_t, uint32_t, uint16_t);
	uint16_t _VerifyPageFullWriteVariable(uint16_t, uint16_t);
};


extern EEPROMClass EEPROM;
