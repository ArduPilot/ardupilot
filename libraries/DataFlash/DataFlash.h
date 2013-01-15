/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/* ************************************************************ */
/* Test for DataFlash Log library                               */
/* ************************************************************ */
#ifndef DataFlash_h
#define DataFlash_h

#include <stdint.h>

// the last page holds the log format in first 4 bytes. Please change
// this if (and only if!) the low level format changes
#define DF_LOGGING_FORMAT    0x28122013

// we use an invalie logging format to test the chip erase
#define DF_LOGGING_FORMAT_INVALID   0x28122012

class DataFlash_Class
{
private:
    struct PageHeader {
        uint16_t FileNumber;
        uint16_t FilePage;
    };

    // DataFlash Log variables...
    uint8_t df_BufferNum;
    uint8_t df_Read_BufferNum;
    uint16_t df_BufferIdx;
    uint16_t df_Read_BufferIdx;
    uint16_t df_PageAdr;
    uint16_t df_Read_PageAdr;
    uint16_t df_FileNumber;
    uint16_t df_FilePage;

    virtual void WaitReady() = 0;
    virtual void BufferToPage (uint8_t BufferNum, uint16_t PageAdr, uint8_t wait) = 0;
    virtual void PageToBuffer(uint8_t BufferNum, uint16_t PageAdr) = 0;
    virtual void PageErase(uint16_t PageAdr) = 0;
    virtual void BlockErase(uint16_t BlockAdr) = 0;
    virtual void ChipErase() = 0;

    // write size bytes of data to a page. The caller must ensure that
    // the data fits within the page, otherwise it will wrap to the
    // start of the page
    virtual void BlockWrite(uint8_t BufferNum, uint16_t IntPageAdr, 
                            const void *pHeader, uint8_t hdr_size,
                            const void *pBuffer, uint16_t size) = 0;
    
    // read size bytes of data to a page. The caller must ensure that
    // the data fits within the page, otherwise it will wrap to the
    // start of the page
    virtual bool BlockRead(uint8_t BufferNum, uint16_t IntPageAdr, void *pBuffer, uint16_t size) = 0;

    // internal high level functions
    int16_t find_last_page(void);
    int16_t find_last_page_of_log(uint16_t log_number);
    bool check_wrapped(void);

public:
    uint8_t df_manufacturer;
    uint16_t df_device;
    uint16_t df_PageSize;

    virtual void Init(void) = 0;
    virtual void ReadManufacturerID() = 0;
    virtual bool CardInserted(void) = 0;

    int16_t GetPage(void);
    int16_t GetWritePage(void);

    // erase handling
    void EraseAll();
    bool NeedErase(void);

    // Write methods
    void StartWrite(int16_t PageAdr);
    void FinishWrite(void);
    void WriteBlock(const void *pBuffer, uint16_t size);
    void WriteByte(uint8_t data);
    void WriteInt(int16_t data);
    void WriteLong(int32_t data);


    // Read methods
    void StartRead(int16_t PageAdr);
    void ReadBlock(void *pBuffer, uint16_t size);
    uint8_t ReadByte();
    int16_t ReadInt();
    int32_t ReadLong();

    // file numbers
    void SetFileNumber(uint16_t FileNumber);
    uint16_t GetFileNumber();
    uint16_t GetFilePage();

    // page handling
    uint16_t df_NumPages;

    /*
      read a packet, stripping off the header bytes
    */
    void ReadPacket(void *pkt, uint16_t size) {
        ReadBlock((void *)(sizeof(struct log_Header)+(uintptr_t)pkt), size - sizeof(struct log_Header));
    }

    // high level interface
    int16_t find_last_log(void);
    void get_log_boundaries(uint8_t log_num, int16_t & start_page, int16_t & end_page);
    uint8_t get_num_logs(void);
    void start_new_log(void);
    uint16_t log_read_process(uint16_t start_page, uint16_t end_page, 
                              void (*callback)(uint8_t msgid));

	/*
      every logged packet starts with 3 bytes
    */
    struct log_Header {
        uint8_t head1, head2, msgid;
    };
};

/*
  unfortunately these need to be macros because of a limitation of
  named member structure initialisation in g++
 */
#define LOG_PACKET_HEADER	       uint8_t head1, head2, msgid;
#define LOG_PACKET_HEADER_INIT(id) head1 : HEAD_BYTE1, head2 : HEAD_BYTE2, msgid : id

// once the logging code is all converted we will remove these from
// this header
#define HEAD_BYTE1  0xA3    // Decimal 163
#define HEAD_BYTE2  0x95    // Decimal 149



#include "DataFlash_APM1.h"
#include "DataFlash_APM2.h"
#include "DataFlash_SITL.h"
#include "DataFlash_Empty.h"

#endif
