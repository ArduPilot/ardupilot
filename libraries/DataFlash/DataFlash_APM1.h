/* ************************************************************ */
/* Test for DataFlash Log library                               */
/* ************************************************************ */
#ifndef __DATAFLASH_APM1_H__
#define __DATAFLASH_APM1_H__

#include "DataFlash.h"

class DataFlash_APM1 : public DataFlash_Class
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

  public:

	DataFlash_APM1(); // Constructor
	void Init();
	void ReadManufacturerID();
	void PageErase (uint16_t PageAdr);
	void ChipErase ();
};

#endif // __DATAFLASH_APM1_H__
