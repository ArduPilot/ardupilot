/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/* 
   DataFlash logging - file oriented variant

   This uses posix file IO to create log files called logNN.dat in the
   given directory
 */

#ifndef DataFlash_File_h
#define DataFlash_File_h

class DataFlash_File : DataFlash_Class
{
public:
    // constructor
    DataFlash_File(const char *log_directory);

    // initialisation
    void Init(void);
    bool CardInserted(void);

    // erase handling
    bool NeedErase(void);
    void EraseAll();

    /* Write a block of data at current offset */
    void WriteBlock(const void *pBuffer, uint16_t size);

    /*
      read a packet. The header byte have already been read.
    */
    void ReadPacket(void *pkt, uint16_t size);

    // high level interface
    uint16_t find_last_log(void);
    void get_log_boundaries(uint8_t log_num, uint16_t & start_page, uint16_t & end_page);
    uint8_t get_num_logs(void);
    void start_new_log(void);
    uint16_t log_read_process(uint8_t log_num,
                              uint16_t start_page, uint16_t end_page, 
                              void (*callback)(uint8_t msgid));
    void DumpPageInfo(AP_HAL::BetterStream *port);
    void ShowDeviceInfo(AP_HAL::BetterStream *port);

private:
    int _write_fd;
    int _read_fd;
    bool _initialised;
    const char *_log_directory;

    // write buffer
    uint8_t *_writebuf;
    const uint16_t _writebuf_size;
    volatile uint16_t _writebuf_head;
    volatile uint16_t _writebuf_tail;

    /* construct a file name given a log number. Caller must free. */
    char *_log_file_name(uint8_t log_num);
};


#endif // DataFlash_File_h

