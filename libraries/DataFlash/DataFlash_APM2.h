/* ************************************************************ */
/* DataFlash_APM2 Log library                                 */
/* ************************************************************ */
#ifndef __DATAFLASH_APM2_H__
#define __DATAFLASH_APM2_H__

#include "DataFlash.h"

class DataFlash_APM2 : public DataFlash_Class
{
  private:
	//Methods
	unsigned char BufferRead (unsigned char BufferNum, uint16_t IntPageAdr);
	void BufferWrite (unsigned char BufferNum, uint16_t IntPageAdr, unsigned char Data);
	void BufferToPage (unsigned char BufferNum, uint16_t PageAdr, unsigned char wait);
	void PageToBuffer(unsigned char BufferNum, uint16_t PageAdr);
	void WaitReady();
	unsigned char ReadStatusReg();
	unsigned char ReadStatus();
	uint16_t PageSize();

	unsigned char SPI_transfer(unsigned char data);
	void CS_inactive();
	void CS_active();


  public:
	DataFlash_APM2(); // Constructor
	void Init();
	void ReadManufacturerID();
	bool CardInserted();
	void PageErase (uint16_t PageAdr);
	void ChipErase ();
};

#endif
