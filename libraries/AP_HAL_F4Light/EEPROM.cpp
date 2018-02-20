/*
(c) 2017 night_ghost@ykoctpa.ru
 

*/
#pragma GCC optimize ("O2")

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_F4LIGHT


#include <string.h>
#include "stm32f4xx.h"
#include "EEPROM.h"
#include <hal.h>

#include <GCS_MAVLink/GCS.h>

/*
    address not uses 2 high bits so we will use them as flags of right written slot - if address has high bit then it written wrong
*/

EEPROMClass::EEPROMClass(void) 
: PageSize(0)
, _status(EEPROM_NOT_INIT)
{
}

static void reset_flash_errors(){
    if(FLASH->SR & 0xE0) FLASH->SR = 0xE0; // reset Programming Sequence, Parallelism and Alignment errors
    if(FLASH->SR & FLASH_FLAG_WRPERR) {  // write protection detected
        FLASH_OB_Unlock();
        
        FLASH_OB_WRPConfig(OB_WRP_Sector_All, DISABLE); // remove protection            
        FLASH->SR |= FLASH_FLAG_WRPERR; // reset flag
    }
}

// библиотечная версия содержит ошибку и не разблокирует память
void EEPROMClass::FLASH_OB_WRPConfig(uint32_t OB_WRP, FunctionalState NewState)
{ 
  
  /* Check the parameters */
  assert_param(IS_OB_WRP(OB_WRP));
  assert_param(IS_FUNCTIONAL_STATE(NewState));
    
    FLASH_WaitForLastOperation();

//  if(status == FLASH_COMPLETE) {  тут может быть любая ошибка - оттого мы и вызываем разблокировку!
    if(NewState != DISABLE)
    {
      *(__IO uint16_t*)OPTCR_BYTE2_ADDRESS &= (~OB_WRP);
    }
    else
    {
      *(__IO uint16_t*)OPTCR_BYTE2_ADDRESS |= (uint16_t)OB_WRP;
    }
//  }
}



FLASH_Status EEPROMClass::write_16(uint32_t addr, uint16_t data){
    uint16_t n_try=16;
again:
    FLASH_Status  sts = FLASH_ProgramHalfWord(addr, data);
    
    if(sts != FLASH_COMPLETE ) {
        reset_flash_errors();
        if(n_try-- > 0) goto again;
    }
    
    return sts;
}

FLASH_Status EEPROMClass::write_8(uint32_t addr, uint8_t data){ 
    uint16_t n_try=16;
again:

    FLASH_Status  sts = FLASH_ProgramByte(addr, data);
    
    if(sts != FLASH_COMPLETE ) {
        reset_flash_errors();

        if(n_try-- > 0) goto again;
    }
    
    return sts;
}

void EEPROMClass::FLASH_Lock_check(){
    FLASH_Lock();
    FLASH->CR |= FLASH_CR_ERRIE;
    FLASH->ACR |= FLASH_ACR_DCEN; // enable data cache again
}

void EEPROMClass::FLASH_Unlock_dis(){
    FLASH->ACR &= ~FLASH_ACR_DCEN; // disable data cache
    FLASH_Unlock();
}

/**
  * @brief  Check page for blank
  * @param  page base address
  * @retval Success or error
  *		EEPROM_BAD_FLASH:	page not empty after erase
  *		EEPROM_OK:			page blank
  */
uint16_t EEPROMClass::_CheckPage(uint32_t pageBase, uint16_t status)
{
	uint32_t pageEnd = pageBase + PageSize;

	// Page Status not EEPROM_ERASED and not a "state"
	if (read_16(pageBase) != EEPROM_ERASED && read_16(pageBase) != status)
		return EEPROM_BAD_FLASH;

	for(pageBase += 4; pageBase < pageEnd; pageBase += 4)
		if (read_32(pageBase) != 0xFFFFFFFF)	// Verify if slot is empty
			return EEPROM_BAD_FLASH;
	return EEPROM_OK;
}

/**
  * @brief  Erases a specified FLASH page by address.
  * @param  Page_Address: The page address to be erased.
  * @retval FLASH Status: The returned value can be: FLASH_BUSY, FLASH_ERROR_PG,
  *   FLASH_ERROR_WRP, FLASH_COMPLETE or FLASH_TIMEOUT.
  */
FLASH_Status EEPROMClass::_ErasePageByAddress(uint32_t Page_Address)
{

    int Page_Offset = Page_Address - 0x08000000; // calculates sector by address
    uint32_t FLASH_Sector;

    if(Page_Offset < 0x10000) {
	FLASH_Sector = Page_Offset / 0x4000; // 4 * 16K pages
    } else if(Page_Offset < 0x20000) {
	FLASH_Sector = 4;                    // 1 * 64K page
    } else {
	FLASH_Sector = 4 + Page_Offset / 0x20000; // all another pages of 128K
    }

    uint8_t n_try = 16;
again:
    FLASH_Status ret = FLASH_EraseSector(8 * FLASH_Sector, VoltageRange_3);

    if(ret != FLASH_COMPLETE ) {
        reset_flash_errors();
        if(n_try-- > 0) goto again;
    }
	
    return ret;
}

/**
  * @brief  Erase page with increment erase counter (page + 2)
  * @param  page base address
  * @retval Success or error
  *			FLASH_COMPLETE: success erase
  *			- Flash error code: on write Flash error
  */
FLASH_Status EEPROMClass::_ErasePage(uint32_t pageBase)
{

	FLASH_Status status;
	uint16_t data = read_16(pageBase);
	if ((data == EEPROM_ERASED) || (data == EEPROM_VALID_PAGE) || (data == EEPROM_RECEIVE_DATA))
		data = read_16(pageBase + 2) + 1; // erase count +1
	else
		data = 0;
#ifdef DEBUG_BUILD
        printf("\nEEprom erase page %d\n ", (uint16_t)((pageBase & 0x00ffffff) / 0x4000) ); // clear high byte of address and count 16K blocks
#endif
        gcs().send_text(MAV_SEVERITY_INFO, "EEprom erase page %d", (uint16_t)((pageBase & 0x00ffffff) / 0x4000) );

	status = _ErasePageByAddress(pageBase);
	
	if (status == FLASH_COMPLETE)
		status = write_16(pageBase + 2, data); // write count back

	return status;
}

/**
  * @brief  Check page for blank and erase it
  * @param  page base address
  * @retval Success or error
  *			- Flash error code: on write Flash error
  *			- EEPROM_BAD_FLASH:	page not empty after erase
  *			- EEPROM_OK:			page blank
  */
uint16_t EEPROMClass::_CheckErasePage(uint32_t pageBase, uint16_t req)
{
	uint16_t status;
	if (_CheckPage(pageBase, req) != EEPROM_OK) {
		status = _ErasePage(pageBase);
		if (status != FLASH_COMPLETE)
			return status;
		return _CheckPage(pageBase, req);
	}
	return EEPROM_OK;
}

/**
  * @brief  Find valid Page for write or read operation
  * @param	Page0: Page0 base address
  *		Page1: Page1 base address
  * @retval Valid page address (PAGE0 or PAGE1) or NULL in case of no valid page was found
  */
uint32_t EEPROMClass::_FindValidPage(void)
{

        if (_status == EEPROM_NOT_INIT) {
            if (_init() != EEPROM_OK) return 0;
        }
again:
	uint16_t status0 = read_16(PageBase0);		// Get Page0 actual status
	uint16_t status1 = read_16(PageBase1);		// Get Page1 actual status

	if (status0 == EEPROM_VALID_PAGE && status1 == EEPROM_ERASED)
		return PageBase0;
	if (status1 == EEPROM_VALID_PAGE && status0 == EEPROM_ERASED)
		return PageBase1;
// something went wrong, try to recover
        if(_init() == EEPROM_OK) goto again;

// all bad - -_init() fails. TODO: Panic()?
        return 0;
}

/**
  * @brief  Calculate unique variables in EEPROM
  * @param  start: address of first slot to check (page + 4)
  * @param	end: page end address
  * @param	address: 16 bit virtual address of the variable to excluse (or 0XFFFF)
  * @retval count of variables
  */
uint16_t EEPROMClass::_GetVariablesCount(uint32_t pageBase, uint16_t skipAddress)
{
	uint16_t varAddress, nextAddress;
	uint32_t idx;
	uint32_t pageEnd = pageBase + PageSize;
	uint16_t mycount = 0;

	for (pageBase += 6; pageBase < pageEnd; pageBase += 4) {
		varAddress = read_16(pageBase);
		if (varAddress == 0xFFFF || (varAddress & ADDRESS_MASK)== skipAddress || /* partially written */ (varAddress & FLAGS_MASK)!=0 )
			continue;

		mycount++;

		for(idx = pageBase + 4; idx < pageEnd; idx += 4) {
			nextAddress = read_16(idx);
			if ((nextAddress & ADDRESS_MASK) == (varAddress & ADDRESS_MASK)) {
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
  * @param	SkipAddress: 16 bit virtual address of the variable (or 0xFFFF)
  * @retval Success or error status:
  *           - FLASH_COMPLETE: on success
  *           - EEPROM_OUT_SIZE: if valid new page is full
  *           - Flash error code: on write Flash error
  */
uint16_t EEPROMClass::_PageTransfer(uint32_t newPage, uint32_t oldPage, uint16_t SkipAddress)
{
	uint32_t oldEnd, newEnd;
	uint32_t oldIdx, newIdx, idx;
	uint16_t address, data, found;
	FLASH_Status status;

	// Transfer process: transfer variables from old to the new active page
	newEnd = newPage + PageSize;

	// Find first free element in new page
	for (newIdx = newPage + 4; newIdx < newEnd; newIdx += 4)
		if (read_32(newIdx) == 0xFFFFFFFF)	// Verify if element contents are 0xFFFFFFFF
		    break;
	if (newIdx >= newEnd)
		return EEPROM_OUT_SIZE;

	oldEnd = oldPage + 4;
	oldIdx = oldPage + (PageSize - 2);

	for (; oldIdx > oldEnd; oldIdx -= 4) {
		address = read_16(oldIdx);
		if ( address == SkipAddress || (address & FLAGS_MASK)!=0)
			continue;						// it's means that power off after write data

		found = 0;

		for (idx = newPage + 6; idx < newIdx; idx += 4){
			if (read_16(idx) == address) {
				found = 1;
				break;
			}
                }
		if (found)
			continue;       // There is more recent data with this address

		if (newIdx < newEnd) {
			data = read_16(oldIdx - 2);

			status = write_16(newIdx, data);
			if (status != FLASH_COMPLETE)
				return status;

			status = write_16(newIdx + 2, address & ADDRESS_MASK);
			if (status != FLASH_COMPLETE)
				return status;

			newIdx += 4;
		}
		else
			return EEPROM_OUT_SIZE;
	}

	// Erase the old Page: Set old Page status to EEPROM_EEPROM_ERASED status
	data = _CheckErasePage(oldPage, EEPROM_ERASED);
	if (data != EEPROM_OK)
	    return data;

	// Set new Page status
	status = write_16(newPage, EEPROM_VALID_PAGE);
	if (status != FLASH_COMPLETE)
	    return status;

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
uint16_t EEPROMClass::_VerifyPageFullWriteVariable(uint16_t Address, uint16_t Data)
{
	FLASH_Status status;
	uint32_t idx, pageBase, pageEnd, newPage;
	uint16_t mycount;
	uint16_t old_data;

	// Get valid Page for write operation
	pageBase = _FindValidPage();
	if (pageBase == 0)
		return  EEPROM_NO_VALID_PAGE;

	// Get the valid Page end Address
	pageEnd = pageBase + PageSize;			// Set end of page
	
// read from end to begin
	for (idx = pageEnd - 2; idx > pageBase; idx -= 4) { 
		if (read_16(idx) == Address){		// Find last value for address, will stop loop if found
			old_data = read_16(idx - 2);	// Read last data
			if (old_data == Data){
			    return EEPROM_OK;   //      data already OK
			}
			if (old_data == 0xFFFF || /* we can write - there is no '0' where we need '1' */ (~old_data & Data)==0 ) { 
				status = write_16(idx - 2, Data);	// Set variable data
				if (status == FLASH_COMPLETE && read_16(idx - 2) == Data) // check if writen
					return EEPROM_OK;
			}
			break;
		}
	}

	// Check each active page address starting from begining
	for (idx = pageBase + 4; idx < pageEnd; idx += 4){
		if (read_32(idx) == 0xFFFFFFFF){		// Verify if element contents are 0xFFFFFFFF
			status = write_16(idx, Data);	// Set variable data
			if (status != FLASH_COMPLETE)
				return status;
			status = write_16(idx + 2, Address & ADDRESS_MASK);	// Set variable virtual address
			if (status != FLASH_COMPLETE)
				return 0x90 + status;
			return EEPROM_OK;
		}
        }

	// Empty slot not found, need page transfer
	// Calculate unique variables in page
	mycount = _GetVariablesCount(pageBase, Address) + 1;
	if (mycount >= maxcount())
		return EEPROM_OUT_SIZE;

	if (pageBase == PageBase1)
		newPage = PageBase0;		// New page address where variable will be moved to
	else
		newPage = PageBase1;

	// Set the new Page status to RECEIVE_DATA status
	status = write_16(newPage, EEPROM_RECEIVE_DATA);
	if (status != FLASH_COMPLETE)
		return status;

	// Write the variable passed as parameter in the new active page
	status = write_16(newPage + 4, Data);
	if (status != FLASH_COMPLETE)
		return status;

	status = write_16(newPage + 6, Address);
	if (status != FLASH_COMPLETE)
		return status;

	return _PageTransfer(newPage, pageBase, Address);
}


uint16_t EEPROMClass::init(uint32_t pageBase0, uint32_t pageBase1, uint32_t pageSize)
{
	PageBase0 = pageBase0;
	PageBase1 = pageBase1;
	PageSize = pageSize;

	return _init();
}

uint16_t EEPROMClass::_init(void) // 
{

	uint16_t status0, status1, erased0;
	FLASH_Status status;

	_status = EEPROM_NO_VALID_PAGE;

        if(PageSize == 0) return _status; // no real Init call

	FLASH_Unlock();

	erased0 = read_16(PageBase0 + 2);
	if (erased0 == 0xffff) erased0 = 0;
	// Print number of EEprom write cycles - but it cleared each reflash
#ifdef DEBUG_BUILD
	printf("\nEEprom write cycles %d\n ", erased0);
#endif

	status0 = read_16(PageBase0);
	status1 = read_16(PageBase1);

	// Check if EEprom is formatted
        if (       status0 != EEPROM_VALID_PAGE && status0 != EEPROM_RECEIVE_DATA && status0 != EEPROM_ERASED){
    	    // _status = _format();  как-то жестко форматировать ВСЕ по одиночной ошибке. Если ВТОРАЯ страница валидная то достаточно стереть пострадавшую
    	    if(status1 == EEPROM_VALID_PAGE) _status = _CheckErasePage(PageBase0, EEPROM_ERASED);
    	    else                             _status = _format();
    	    status0 = read_16(PageBase0);
            status1 = read_16(PageBase1);
        }else  if (status1 != EEPROM_VALID_PAGE && status1 != EEPROM_RECEIVE_DATA && status1 != EEPROM_ERASED){
        //    _status = _format();  тут мы первую страницу уже проверили - валидная, отставить вредительство!
            _status = _CheckErasePage(PageBase1, EEPROM_ERASED);
	    status0 = read_16(PageBase0);
	    status1 = read_16(PageBase1);            
        }


	switch (status0) {
/*
		Page0				Page1
		-----				-----
		EEPROM_ERASED			EEPROM_VALID_PAGE			Page1 valid, Page0 erased
						EEPROM_RECEIVE_DATA			Page1 need set to valid, Page0 erased
						EEPROM_ERASED				make _Format
						any					Error: EEPROM_NO_VALID_PAGE
*/
	case EEPROM_ERASED:
		if (status1 == EEPROM_VALID_PAGE)		// Page0 erased, Page1 valid
			_status = _CheckErasePage(PageBase0, EEPROM_ERASED);
		else if (status1 == EEPROM_RECEIVE_DATA) {	// Page0 erased, Page1 receive		
	    // Page Transfer failed! we can't be sure if it finished OK so should restart transfer - but page is erased. This can be if write "valid" mark fails
			status = write_16(PageBase1, EEPROM_VALID_PAGE); // so just mark it as valid
			if (status != FLASH_COMPLETE)
				_status = status;
			else
				_status = _CheckErasePage(PageBase0, EEPROM_ERASED);

		}
		else /* if (status1 == EEPROM_ERASED)*/		// Both in erased OR 2nd in unknown state so format EEPROM
			_status = _format();
		break;
/*
		Page0				Page1
		-----				-----
		EEPROM_RECEIVE_DATA		EEPROM_VALID_PAGE			Transfer Page1 to Page0
						EEPROM_ERASED				Page0 need set to valid, Page1 erased
						any					EEPROM_NO_VALID_PAGE
*/
	case EEPROM_RECEIVE_DATA:
		if (status1 == EEPROM_VALID_PAGE)			// Page0 receive, Page1 valid
		// transfer failed and we have good data - restart transfer
			_status = _PageTransfer(PageBase0, PageBase1, 0xFFFF);
		else if (status1 == EEPROM_ERASED){			// Page0 receive, Page1 erased		
    	                // setting "valid" mark failed
			_status = _CheckErasePage(PageBase1, EEPROM_ERASED);
			if (_status == EEPROM_OK){			
				status = write_16(PageBase0, EEPROM_VALID_PAGE); // mark as valid again
				if (status != FLASH_COMPLETE)
					_status = status;
				else
					_status = EEPROM_OK;
			}
		}
		else _status = _format(); // all bad
		break;
/*
		Page0				Page1
		-----				-----
		EEPROM_VALID_PAGE		EEPROM_VALID_PAGE			Error: EEPROM_NO_VALID_PAGE
						EEPROM_RECEIVE_DATA			Transfer Page0 to Page1
						any					Page0 valid, Page1 erased
*/
	case EEPROM_VALID_PAGE:
		if (status1 == EEPROM_VALID_PAGE){			// Both pages valid
// just check amount and correctness of data
                    uint16_t cnt0 = _GetVariablesCount(PageBase0, 0xFFFF);
                    uint16_t cnt1 = _GetVariablesCount(PageBase1, 0xFFFF);
                    if(cnt0>cnt1){
                        _status = _CheckErasePage(PageBase1, EEPROM_ERASED);
                    }else if(cnt0<cnt1){
                        _status = _CheckErasePage(PageBase0, EEPROM_ERASED);
                    } else { // ну такого совсем не может быть ибо марку "валид" мы делаем только после стирания.
		        // _status = EEPROM_NO_VALID_PAGE;
		        _status = _CheckErasePage(PageBase1, EEPROM_ERASED); // сотрем вторую - я монетку бросил
		    }
		}else if (status1 == EEPROM_RECEIVE_DATA) {		
		    _status = _PageTransfer(PageBase1, PageBase0, 0xFFFF); // restart transfer
		} else {
		    _status = _CheckErasePage(PageBase1, EEPROM_ERASED);
		}
		break;
/*
		Page0				Page1
		-----				-----
		any				EEPROM_VALID_PAGE			Page1 valid, Page0 erased
						EEPROM_RECEIVE_DATA			Page1 valid, Page0 erased
						any					EEPROM_NO_VALID_PAGE
*/
	default:
		if (status1 == EEPROM_VALID_PAGE)
			_status = _CheckErasePage(PageBase0, EEPROM_ERASED);	// Check/Erase Page0
		else if (status1 == EEPROM_RECEIVE_DATA) {
			status = write_16(PageBase1, EEPROM_VALID_PAGE);
			if (status != FLASH_COMPLETE)
				_status = status;
			else
				_status = _CheckErasePage(PageBase0, EEPROM_ERASED);
		}
		else _status = _format(); // all bad
		break;
	}
	
	FLASH_Lock_check(); // lock after all writes
	return _status;
}

/**
  * @brief  Erases PAGE0 and PAGE1 and writes EEPROM_VALID_PAGE / 0 header to PAGE0
  * @param  PAGE0 and PAGE1 base addresses
  * @retval _status of the last operation (Flash write or erase) done during EEPROM formating
  */
uint16_t EEPROMClass::_format(void)
{
	uint16_t status;
	uint16_t n_try=16;

again:

	// Erase Page0
	status = _CheckErasePage(PageBase0, EEPROM_VALID_PAGE);
	if (status != EEPROM_OK) goto error;
	
	if (read_16(PageBase0) == EEPROM_ERASED) {
		// Set Page0 as valid page: Write VALID_PAGE at Page0 base address
		FLASH_Status fs = write_16(PageBase0, EEPROM_VALID_PAGE);
		if (fs != FLASH_COMPLETE) goto error;
	}
	// Erase Page1
	status = _CheckErasePage(PageBase1, EEPROM_ERASED);
	if (status == EEPROM_OK) return status;
error:
        // something went wrong
        reset_flash_errors();

        if(n_try-- > 0) goto again;
        
	return status;
}

uint16_t EEPROMClass::format(void){
    uint16_t status;
    FLASH_Unlock();
    status = _format();
    FLASH_Lock_check();
    return status;
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

	// Get active Page for read operation
	pageBase = _FindValidPage();
	if (pageBase == 0)
		return  EEPROM_NO_VALID_PAGE;

	*Erases = read_16(pageBase+2);
	return EEPROM_OK;
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

	*Data = EEPROM_DEFAULT_DATA; // Set default data (empty EEPROM)

	// Get active Page for read operation
	pageBase = _FindValidPage();
	if (pageBase == 0)      return  EEPROM_NO_VALID_PAGE;

	// Get the valid Page end Address
	pageEnd = pageBase + (PageSize - 2);
	
	Address &= ADDRESS_MASK;
	
	uint32_t ptr = pageEnd;
	
	// Check each active page address  starting from end - the last value written
	for (pageBase += 6; ptr >= pageBase; ptr -= 4){
	    if (read_16(ptr) == Address){// Compare the read address with the virtual address		
		*Data = read_16(ptr - 2);		// Get content of Address-2 which is variable value
		return EEPROM_OK;
	    }
	}

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
	if (_status == EEPROM_NOT_INIT)
		if (_init() != EEPROM_OK)
			return _status;

	if (Address == 0xFFFF)
		return EEPROM_BAD_ADDRESS;

        FLASH_Unlock();

	// Write the variable virtual address and value in the EEPROM
	uint16_t status = _VerifyPageFullWriteVariable(Address & ADDRESS_MASK, Data);
	if(status == EEPROM_NO_VALID_PAGE) _status = EEPROM_NOT_INIT;

	FLASH_Lock_check();
	return status;
}

/**
  * @brief  Return number of variable
  * @retval Number of variables
  */
uint16_t EEPROMClass::count(uint16_t *cnt)
{
	// Get valid Page for write operation
	uint32_t pageBase = _FindValidPage();
	if (pageBase == 0)
		return EEPROM_NO_VALID_PAGE;	// No valid page, return max. numbers

	*cnt = _GetVariablesCount(pageBase, 0xFFFF);
	return EEPROM_OK;
}


EEPROMClass EEPROM;
#endif
