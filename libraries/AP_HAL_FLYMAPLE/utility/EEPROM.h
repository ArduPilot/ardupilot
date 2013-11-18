#ifndef __EEPROM_H
#define __EEPROM_H

#define EEPROM_USES_16BIT_WORDS

#include "wirish.h"
#include "flash_stm32.h"

#ifndef EEPROM_PAGE_SIZE
	#if defined (MCU_STM32F103RB) || defined (MCU_STM32F103CB)
		#define EEPROM_PAGE_SIZE	(uint16)0x400  /* Page size = 1KByte */
	#elif defined (MCU_STM32F103ZE) || defined (MCU_STM32F103VE) || defined (MCU_STM32F103RE)
		#define EEPROM_PAGE_SIZE	(uint16)0x800  /* Page size = 2KByte */
	#elif defined (MCU_STM32F205VE) || defined (MCU_STM32F406VG)
		#define EEPROM_PAGE_SIZE	(uint16)0x4000  /* Page size = 16KByte */
	#else
		#error	"No MCU type specified. Add something like -DMCU_STM32F103RB " \
			"to your compiler arguments (probably in a Makefile)."
	#endif
#endif

#ifndef EEPROM_PAGE_SIZE
#endif

#ifndef EEPROM_START_ADDRESS
	#if defined BOARD_freeflight
		#define EEPROM_START_ADDRESS	((uint32)(0x8020000 - 2 * EEPROM_PAGE_SIZE))
	#elif defined (MCU_STM32F103RB) || defined (MCU_STM32F103CB)
		#define EEPROM_START_ADDRESS	((uint32)(0x8005000 - 2 * EEPROM_PAGE_SIZE))
	#elif defined (MCU_STM32F103ZE) || defined (MCU_STM32F103VE) || defined (MCU_STM32F103RE)
		//#define EEPROM_START_ADDRESS	((uint32)(0x8080000 - 2 * EEPROM_PAGE_SIZE))
		#define EEPROM_START_ADDRESS	((uint32)(0x8005000 - 2 * EEPROM_PAGE_SIZE))
	#elif defined (MCU_STM32F205VE) || defined (MCU_STM32F406VG)
		#define EEPROM_START_ADDRESS	((uint32)(0x8010000 - 2 * EEPROM_PAGE_SIZE))
	#else
		#error	"No MCU type specified. Add something like -DMCU_STM32F103RB " \
			"to your compiler arguments (probably in a Makefile)."
	#endif
#endif

/* Pages 0 and 1 base and end addresses */
#define EEPROM_PAGE0_BASE		((uint32)(EEPROM_START_ADDRESS))
#define EEPROM_PAGE1_BASE		((uint32)(EEPROM_START_ADDRESS + EEPROM_PAGE_SIZE))

/* Page status definitions */
#define EEPROM_ERASED			((uint16)0xFFFF)	/* PAGE is empty */
#define EEPROM_RECEIVE_DATA		((uint16)0xEEEE)	/* PAGE is marked to receive data */
#define EEPROM_VALID_PAGE		((uint16)0x0000)	/* PAGE containing valid data */

/* Page full define */
enum //: uint16
{
	EEPROM_OK            = ((uint16)0x0000),
	EEPROM_OUT_SIZE      = ((uint16)0x0081),
	EEPROM_BAD_ADDRESS   = ((uint16)0x0082),
	EEPROM_BAD_FLASH     = ((uint16)0x0083),
	EEPROM_NOT_INIT      = ((uint16)0x0084),
	EEPROM_NO_VALID_PAGE = ((uint16)0x00AB)
};

#define EEPROM_DEFAULT_DATA		0xFFFF


class EEPROMClass
{
public:
	EEPROMClass(void);

	uint16 init(void);
	uint16 init(uint32, uint32, uint32);

	uint16 format(void);

	uint16 erases(uint16 *);
	uint16 read (uint16 address);
	uint16 read (uint16 address, uint16 *data);
	uint16 write(uint16 address, uint16 data);
	uint16 count(uint16 *);
	uint16 maxcount(void);

	uint32 PageBase0;
	uint32 PageBase1;
	uint32 PageSize;
	uint16 Status;
private:
	FLASH_Status EE_ErasePage(uint32);

	uint16 EE_CheckPage(uint32, uint16);
	uint16 EE_CheckErasePage(uint32, uint16);
	uint16 EE_Format(void);
	uint32 EE_FindValidPage(void);
	uint16 EE_GetVariablesCount(uint32, uint16);
	uint16 EE_PageTransfer(uint32, uint32, uint16);
	uint16 EE_VerifyPageFullWriteVariable(uint16, uint16);
};

extern EEPROMClass EEPROM;

#endif	/* __EEPROM_H */
