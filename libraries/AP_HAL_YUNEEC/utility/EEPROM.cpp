#include <AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_YUNEEC

#include "EEPROM.h"

/**
  * @brief  Check page for blank
  * @param  page base address
  * @retval Success or error
  *		EEPROM_BAD_FLASH:	page not empty after erase
  *		EEPROM_OK:			page blank
  */
uint16_t EEPROMClass::EE_CheckPage(uint32_t pageBase, uint16_t status)
{
	uint32_t pageEnd = pageBase + (uint32_t)PageSize;

	// Page Status not EEPROM_ERASED and not a "state"
	if ((*(__IO uint16_t*)pageBase) != EEPROM_ERASED && (*(__IO uint16_t*)pageBase) != status)
		return EEPROM_BAD_FLASH;
	for(pageBase += 4; pageBase < pageEnd; pageBase += 4)
		if ((*(__IO uint32_t*)pageBase) != 0xFFFFFFFF)	// Verify if slot is empty
			return EEPROM_BAD_FLASH;
	return EEPROM_OK;
}

/**
  * @brief  Erase page with increment erase counter (page + 2)
  * @param  page base address
  * @retval Success or error
  *			FLASH_COMPLETE: success erase
  *			- Flash error code: on write Flash error
  */
FLASH_Status EEPROMClass::EE_ErasePage(uint32_t pageBase)
{
	FLASH_Status FlashStatus;
	uint16_t data = (*(__IO uint16_t*)(pageBase));
	if ((data == EEPROM_ERASED) || (data == EEPROM_VALID_PAGE) || (data == EEPROM_RECEIVE_DATA))
		data = (*(__IO uint16_t*)(pageBase + 2)) + 1;
	else
		data = 0;

	FlashStatus = FLASH_ErasePage(pageBase);
	if (FlashStatus == FLASH_COMPLETE)
		FlashStatus = FLASH_ProgramHalfWord(pageBase + 2, data);

	return FlashStatus;
}

/**
  * @brief  Check page for blank and erase it
  * @param  page base address
  * @retval Success or error
  *			- Flash error code: on write Flash error
  *			- EEPROM_BAD_FLASH:	page not empty after erase
  *			- EEPROM_OK:			page blank
  */
uint16_t EEPROMClass::EE_CheckErasePage(uint32_t pageBase, uint16_t status)
{
	uint16_t FlashStatus;
	if (EE_CheckPage(pageBase, status) != EEPROM_OK)
	{
		FlashStatus = EE_ErasePage(pageBase);
		if (FlashStatus != FLASH_COMPLETE)
			return FlashStatus;
		return EE_CheckPage(pageBase, status);
	}
	return EEPROM_OK;
}

/**
  * @brief  Find valid Page for write or read operation
  * @param	Page0: Page0 base address
  *			Page1: Page1 base address
  * @retval Valid page address (PAGE0 or PAGE1) or NULL in case of no valid page was found
  */
uint32_t EEPROMClass::EE_FindValidPage(void)
{
	uint16_t status0 = (*(__IO uint16_t*)PageBase0);		// Get Page0 actual status
	uint16_t status1 = (*(__IO uint16_t*)PageBase1);		// Get Page1 actual status

	if (status0 == EEPROM_VALID_PAGE && status1 == EEPROM_ERASED)
		return PageBase0;
	if (status1 == EEPROM_VALID_PAGE && status0 == EEPROM_ERASED)
		return PageBase1;

	return 0;
}

/**
  * @brief  Calculate unique variables in EEPROM
  * @param  start: address of first slot to check (page + 4)
  * @param	end: page end address
  * @param	address: 16 bit virtual address of the variable to excluse (or 0XFFFF)
  * @retval count of variables
  */
uint16_t EEPROMClass::EE_GetVariablesCount(uint32_t pageBase, uint16_t skipAddress)
{
	uint16_t varAddress, nextAddress;
	uint32_t idx;
	uint32_t pageEnd = pageBase + (uint32_t)PageSize;
	uint16_t mycount = 0;

	for (pageBase += 6; pageBase < pageEnd; pageBase += 4)
	{
		varAddress = (*(__IO uint16_t*)pageBase);
		if (varAddress == 0xFFFF || varAddress == skipAddress)
			continue;

		mycount++;
		for(idx = pageBase + 4; idx < pageEnd; idx += 4)
		{
			nextAddress = (*(__IO uint16_t*)idx);
			if (nextAddress == varAddress)
			{
				mycount--;
				break;
			}
		}
	}
	return mycount;
}

/**
  * @brief  Transfers last updated variables data from the full Page to an empty one.
  * @param  newPage: new page base address
  * @param	oldPage: old page base address
  *	@param	SkipAddress: 16 bit virtual address of the variable (or 0xFFFF)
  * @retval Success or error status:
  *           - FLASH_COMPLETE: on success
  *           - EEPROM_OUT_SIZE: if valid new page is full
  *           - Flash error code: on write Flash error
  */
uint16_t EEPROMClass::EE_PageTransfer(uint32_t newPage, uint32_t oldPage, uint16_t SkipAddress)
{
	uint32_t oldEnd, newEnd;
	uint32_t oldIdx, newIdx, idx;
	uint16_t address, data, found;
	FLASH_Status FlashStatus;

	// Transfer process: transfer variables from old to the new active page
	newEnd = newPage + ((uint32_t)PageSize);

	// Find first free element in new page
	for (newIdx = newPage + 4; newIdx < newEnd; newIdx += 4)
		if ((*(__IO uint32_t*)newIdx) == 0xFFFFFFFF)	// Verify if element
			break;									//  contents are 0xFFFFFFFF
	if (newIdx >= newEnd)
		return EEPROM_OUT_SIZE;

	oldEnd = oldPage + 4;
	oldIdx = oldPage + (uint32_t)(PageSize - 2);

	for (; oldIdx > oldEnd; oldIdx -= 4)
	{
		address = *(__IO uint16_t*)oldIdx;
		if (address == 0xFFFF || address == SkipAddress)
			continue;						// it's means that power off after write data

		found = 0;
		for (idx = newPage + 6; idx < newIdx; idx += 4)
			if ((*(__IO uint16_t*)(idx)) == address)
			{
				found = 1;
				break;
			}

		if (found)
			continue;

		if (newIdx < newEnd)
		{
			data = (*(__IO uint16_t*)(oldIdx - 2));

			FlashStatus = FLASH_ProgramHalfWord(newIdx, data);
			if (FlashStatus != FLASH_COMPLETE)
				return FlashStatus;

			FlashStatus = FLASH_ProgramHalfWord(newIdx + 2, address);
			if (FlashStatus != FLASH_COMPLETE)
				return FlashStatus;

			newIdx += 4;
		}
		else
			return EEPROM_OUT_SIZE;
	}

	// Erase the old Page: Set old Page status to EEPROM_EEPROM_ERASED status
	data = EE_CheckErasePage(oldPage, EEPROM_ERASED);
	if (data != EEPROM_OK)
		return data;

	// Set new Page status
	FlashStatus = FLASH_ProgramHalfWord(newPage, EEPROM_VALID_PAGE);
	if (FlashStatus != FLASH_COMPLETE)
		return FlashStatus;

	return EEPROM_OK;
}

/**
  * @brief  Verify if active page is full and Writes variable in EEPROM.
  * @param  Address: 16 bit virtual address of the variable
  * @param  Data: 16 bit data to be written as variable value
  * @retval Success or error status:
  *           - FLASH_COMPLETE: on success
  *           - EEPROM_PAGE_FULL: if valid page is full (need page transfer)
  *           - EEPROM_NO_VALID_PAGE: if no valid page was found
  *           - EEPROM_OUT_SIZE: if EEPROM size exceeded
  *           - Flash error code: on write Flash error
  */
uint16_t EEPROMClass::EE_VerifyPageFullWriteVariable(uint16_t Address, uint16_t Data)
{
	FLASH_Status FlashStatus;
	uint32_t idx, pageBase, pageEnd, newPage;
	uint16_t mycount;

	// Get valid Page for write operation
	pageBase = EE_FindValidPage();
	if (pageBase == 0)
		return  EEPROM_NO_VALID_PAGE;

	// Get the valid Page end Address
	pageEnd = pageBase + PageSize;			// Set end of page

	for (idx = pageEnd - 2; idx > pageBase; idx -= 4)
	{
		if ((*(__IO uint16_t*)idx) == Address)		// Find last value for address
		{
			mycount = (*(__IO uint16_t*)(idx - 2));	// Read last data
			if (mycount == Data)
				return EEPROM_OK;
			if (mycount == 0xFFFF)
			{
				FlashStatus = FLASH_ProgramHalfWord(idx - 2, Data);	// Set variable data
				if (FlashStatus == FLASH_COMPLETE)
					return EEPROM_OK;
			}
			break;
		}
	}

	// Check each active page address starting from begining
	for (idx = pageBase + 4; idx < pageEnd; idx += 4)
		if ((*(__IO uint32_t*)idx) == 0xFFFFFFFF)				// Verify if element
		{													//  contents are 0xFFFFFFFF
			FlashStatus = FLASH_ProgramHalfWord(idx, Data);	// Set variable data
			if (FlashStatus != FLASH_COMPLETE)
				return FlashStatus;
			FlashStatus = FLASH_ProgramHalfWord(idx + 2, Address);	// Set variable virtual address
			if (FlashStatus != FLASH_COMPLETE)
				return FlashStatus;
			return EEPROM_OK;
		}

	// Empty slot not found, need page transfer
	// Calculate unique variables in page
	mycount = EE_GetVariablesCount(pageBase, Address) + 1;
	if (mycount >= (PageSize / 4 - 1))
		return EEPROM_OUT_SIZE;

	if (pageBase == PageBase1)
		newPage = PageBase0;		// New page address where variable will be moved to
	else
		newPage = PageBase1;

	// Set the new Page status to RECEIVE_DATA status
	FlashStatus = FLASH_ProgramHalfWord(newPage, EEPROM_RECEIVE_DATA);
	if (FlashStatus != FLASH_COMPLETE)
		return FlashStatus;

	// Write the variable passed as parameter in the new active page
	FlashStatus = FLASH_ProgramHalfWord(newPage + 4, Data);
	if (FlashStatus != FLASH_COMPLETE)
		return FlashStatus;

	FlashStatus = FLASH_ProgramHalfWord(newPage + 6, Address);
	if (FlashStatus != FLASH_COMPLETE)
		return FlashStatus;

	return EE_PageTransfer(newPage, pageBase, Address);
}

EEPROMClass::EEPROMClass(void)
{
	PageBase0 = EEPROM_PAGE0_BASE;
	PageBase1 = EEPROM_PAGE1_BASE;
	PageSize = EEPROM_PAGE_SIZE;
	Status = EEPROM_NOT_INIT;
}

uint16_t EEPROMClass::init(uint32_t pageBase0, uint32_t pageBase1, uint32_t pageSize)
{
	PageBase0 = pageBase0;
	PageBase1 = pageBase1;
	PageSize = pageSize;
	return init();
}

uint16_t EEPROMClass::init(void)
{
	uint16_t status0, status1;
	FLASH_Status FlashStatus;

	FLASH_Unlock();
	Status = EEPROM_NO_VALID_PAGE;

	status0 = (*(__IO uint16_t *)PageBase0);
	status1 = (*(__IO uint16_t *)PageBase1);

	switch (status0)
	{
/*
		Page0				Page1
		-----				-----
	EEPROM_ERASED		EEPROM_VALID_PAGE			Page1 valid, Page0 erased
						EEPROM_RECEIVE_DATA			Page1 need set to valid, Page0 erased
						EEPROM_ERASED				make EE_Format
						any							Error: EEPROM_NO_VALID_PAGE
*/
	case EEPROM_ERASED:
		if (status1 == EEPROM_VALID_PAGE)			// Page0 erased, Page1 valid
			Status = EE_CheckErasePage(PageBase0, EEPROM_ERASED);
		else if (status1 == EEPROM_RECEIVE_DATA)	// Page0 erased, Page1 receive
		{
			FlashStatus = FLASH_ProgramHalfWord(PageBase1, EEPROM_VALID_PAGE);
			if (FlashStatus != FLASH_COMPLETE)
				Status = FlashStatus;
			else
				Status = EE_CheckErasePage(PageBase0, EEPROM_ERASED);
		}
		else if (status1 == EEPROM_ERASED)			// Both in erased state so format EEPROM
			Status = format();
		break;
/*
		Page0				Page1
		-----				-----
	EEPROM_RECEIVE_DATA	EEPROM_VALID_PAGE			Transfer Page1 to Page0
						EEPROM_ERASED				Page0 need set to valid, Page1 erased
						any							EEPROM_NO_VALID_PAGE
*/
    case EEPROM_RECEIVE_DATA:
		if (status1 == EEPROM_VALID_PAGE)			// Page0 receive, Page1 valid
			Status = EE_PageTransfer(PageBase0, PageBase1, 0xFFFF);
		else if (status1 == EEPROM_ERASED)			// Page0 receive, Page1 erased
		{
			Status = EE_CheckErasePage(PageBase1, EEPROM_ERASED);
			if (Status == EEPROM_OK)
			{
				FlashStatus = FLASH_ProgramHalfWord(PageBase0, EEPROM_VALID_PAGE);
				if (FlashStatus != FLASH_COMPLETE)
					Status = FlashStatus;
				else
					Status = EEPROM_OK;
			}
		}
		break;
/*
		Page0				Page1
		-----				-----
	EEPROM_VALID_PAGE	EEPROM_VALID_PAGE			Error: EEPROM_NO_VALID_PAGE
						EEPROM_RECEIVE_DATA			Transfer Page0 to Page1
						any							Page0 valid, Page1 erased
*/
	case EEPROM_VALID_PAGE:
		if (status1 == EEPROM_VALID_PAGE)			// Both pages valid
			Status = EEPROM_NO_VALID_PAGE;
		else if (status1 == EEPROM_RECEIVE_DATA)
			Status = EE_PageTransfer(PageBase1, PageBase0, 0xFFFF);
		else
			Status = EE_CheckErasePage(PageBase1, EEPROM_ERASED);
		break;
/*
		Page0				Page1
		-----				-----
		any				EEPROM_VALID_PAGE			Page1 valid, Page0 erased
						EEPROM_RECEIVE_DATA			Page1 valid, Page0 erased
						any							EEPROM_NO_VALID_PAGE
*/
	default:
		if (status1 == EEPROM_VALID_PAGE)
			Status = EE_CheckErasePage(PageBase0, EEPROM_ERASED);	// Check/Erase Page0
		else if (status1 == EEPROM_RECEIVE_DATA)
		{
			FlashStatus = FLASH_ProgramHalfWord(PageBase1, EEPROM_VALID_PAGE);
			if (FlashStatus != FLASH_COMPLETE)
				Status = FlashStatus;
			else
				Status = EE_CheckErasePage(PageBase0, EEPROM_ERASED);
		}
		break;
	}
	return Status;
}

/**
  * @brief  Erases PAGE0 and PAGE1 and writes EEPROM_VALID_PAGE / 0 header to PAGE0
  * @param  PAGE0 and PAGE1 base addresses
  * @retval Status of the last operation (Flash write or erase) done during EEPROM formating
  */
uint16_t EEPROMClass::format(void)
{
	uint16_t status;
	FLASH_Status FlashStatus;

	FLASH_Unlock();

	// Erase Page0
	status = EE_CheckErasePage(PageBase0, EEPROM_VALID_PAGE);
	if (status != EEPROM_OK)
		return status;
	if ((*(__IO uint16_t*)PageBase0) == EEPROM_ERASED)
	{
		// Set Page0 as valid page: Write VALID_PAGE at Page0 base address
		FlashStatus = FLASH_ProgramHalfWord(PageBase0, EEPROM_VALID_PAGE);
		if (FlashStatus != FLASH_COMPLETE)
			return FlashStatus;
	}
	// Erase Page1
	return EE_CheckErasePage(PageBase1, EEPROM_ERASED);
}

/**
  * @brief  Returns the erase counter for current page
  * @param  Data: Global variable contains the read variable value
  * @retval Success or error status:
  *			- EEPROM_OK: if erases counter return.
  *			- EEPROM_NO_VALID_PAGE: if no valid page was found.
  */
uint16_t EEPROMClass::erases(uint16_t *Erases)
{
	uint32_t pageBase;
	if (Status != EEPROM_OK)
		if (init() != EEPROM_OK)
			return Status;

	// Get active Page for read operation
	pageBase = EE_FindValidPage();
	if (pageBase == 0)
		return  EEPROM_NO_VALID_PAGE;

	*Erases = (*(__IO uint16_t*)pageBase+2);
	return EEPROM_OK;
}

/**
  * @brief	Returns the last stored variable data, if found,
  *			which correspond to the passed virtual address
  * @param  Address: Variable virtual address
  * @retval Data for variable or EEPROM_DEFAULT_DATA, if any errors
  */
uint16_t EEPROMClass::read (uint16_t Address)
{
	uint16_t data;
	read(Address, &data);
	return data;
}

/**
  * @brief	Returns the last stored variable data, if found,
  *			which correspond to the passed virtual address
  * @param  Address: Variable virtual address
  * @param  Data: Pointer to data variable
  * @retval Success or error status:
  *           - EEPROM_OK: if variable was found
  *           - EEPROM_BAD_ADDRESS: if the variable was not found
  *           - EEPROM_NO_VALID_PAGE: if no valid page was found.
  */
uint16_t EEPROMClass::read(uint16_t Address, uint16_t *Data)
{
	uint32_t pageBase, pageEnd;

	// Set default data (empty EEPROM)
	*Data = EEPROM_DEFAULT_DATA;

	if (Status == EEPROM_NOT_INIT)
		if (init() != EEPROM_OK)
			return Status;

	// Get active Page for read operation
	pageBase = EE_FindValidPage();
	if (pageBase == 0)
		return  EEPROM_NO_VALID_PAGE;

	// Get the valid Page end Address
	pageEnd = pageBase + ((uint32_t)(PageSize - 2));
	
	// Check each active page address starting from end
	for (pageBase += 6; pageEnd >= pageBase; pageEnd -= 4)
		if ((*(__IO uint16_t*)pageEnd) == Address)		// Compare the read address with the virtual address
		{
			*Data = (*(__IO uint16_t*)(pageEnd - 2));		// Get content of Address-2 which is variable value
			return EEPROM_OK;
		}

	// Return ReadStatus value: (0: variable exist, 1: variable doesn't exist)
	return EEPROM_BAD_ADDRESS;
}

/**
  * @brief  Writes/upadtes variable data in EEPROM.
  * @param  VirtAddress: Variable virtual address
  * @param  Data: 16 bit data to be written
  * @retval Success or error status:
  *			- FLASH_COMPLETE: on success
  *			- EEPROM_BAD_ADDRESS: if address = 0xFFFF
  *			- EEPROM_PAGE_FULL: if valid page is full
  *			- EEPROM_NO_VALID_PAGE: if no valid page was found
  *			- EEPROM_OUT_SIZE: if no empty EEPROM variables
  *			- Flash error code: on write Flash error
  */
uint16_t EEPROMClass::write(uint16_t Address, uint16_t Data)
{
	if (Status == EEPROM_NOT_INIT)
		if (init() != EEPROM_OK)
			return Status;

	if (Address == 0xFFFF)
		return EEPROM_BAD_ADDRESS;

	// Write the variable virtual address and value in the EEPROM
	uint16_t status = EE_VerifyPageFullWriteVariable(Address, Data);
	return status;
}

/**
  * @brief  Return number of variable
  * @retval Number of variables
  */
uint16_t EEPROMClass::count(uint16_t *Count)
{
	if (Status == EEPROM_NOT_INIT)
		if (init() != EEPROM_OK)
			return Status;

	// Get valid Page for write operation
	uint32_t pageBase = EE_FindValidPage();
	if (pageBase == 0)
		return EEPROM_NO_VALID_PAGE;	// No valid page, return max. numbers

	*Count = EE_GetVariablesCount(pageBase, 0xFFFF);
	return EEPROM_OK;
}

uint16_t EEPROMClass::maxcount(void)
{
	return ((PageSize / 4)-1);
}

EEPROMClass EEPROM;
#endif
