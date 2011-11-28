/* ************************************************************ */
/* Test for DataFlash Log library                               */
/* ************************************************************ */
#ifndef DataFlash_h
#define DataFlash_h

#include <stdint.h>

class DataFlash_Class
{
  public:
	DataFlash_Class() {} // Constructor

  virtual void Init() = 0;
	virtual void ReadManufacturerID() = 0;
	virtual int16_t GetPage() = 0;
	virtual int16_t GetWritePage() = 0;
	virtual void PageErase (uint16_t PageAdr) = 0;
	virtual void ChipErase () = 0;

  // Write methods
	virtual void StartWrite(int16_t PageAdr) = 0;
	virtual void FinishWrite() = 0;
	virtual void WriteByte(unsigned char data) = 0;
	virtual void WriteInt(int16_t data) = 0;
	virtual void WriteLong(int32_t data) = 0;

	// Read methods
	virtual void StartRead(int16_t PageAdr) = 0;
	virtual unsigned char ReadByte() = 0;
	virtual int16_t ReadInt() = 0;
	virtual int32_t ReadLong() = 0;

	void SetFileNumber(uint16_t FileNumber);
	uint16_t GetFileNumber();
	uint16_t GetFilePage();
};

#include "DataFlash_APM1.h"
#include "DataFlash_APM2.h"

#endif
