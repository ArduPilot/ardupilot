/* ************************************************************ */
/* DataFlash_APM2 Log library                                 */
/* ************************************************************ */
#ifndef __DATAFLASH_APM2_H__
#define __DATAFLASH_APM2_H__

#include "DataFlash.h"

class DataFlash_APM2 : public DataFlash_Class
{
  private:
	// DataFlash Log variables...
	unsigned char df_BufferNum;
	unsigned char df_Read_BufferNum;
	unsigned int df_BufferIdx;
	unsigned int df_Read_BufferIdx;
	unsigned int df_PageAdr;
	unsigned int df_Read_PageAdr;
	unsigned char df_Read_END;
	unsigned char df_Stop_Write;

	uint16_t df_FileNumber;
	uint16_t df_FilePage;

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
	unsigned char df_manufacturer;
	unsigned char df_device_0;
	unsigned char df_device_1;
	uint16_t df_PageSize;

	DataFlash_APM2(); // Constructor
	void Init();
	void ReadManufacturerID();
	bool CardInserted();
	int16_t GetPage();
	int16_t GetWritePage();
	void PageErase (uint16_t PageAdr);
	void ChipErase ();
	// Write methods
	void StartWrite(int16_t PageAdr);
	void FinishWrite();
	void WriteByte(unsigned char data);
	void WriteInt(int16_t data);
	void WriteLong(int32_t data);

	// Read methods
	void StartRead(int16_t PageAdr);
	unsigned char ReadByte();
	int16_t ReadInt();
	int32_t ReadLong();

	void SetFileNumber(uint16_t FileNumber);
	uint16_t GetFileNumber();
	uint16_t GetFilePage();
};

#endif
