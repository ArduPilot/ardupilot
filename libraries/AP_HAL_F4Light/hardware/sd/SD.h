/*
   night_ghost@ykoctpa.ru 2017
    
    a port of SparkFun's SD class to FatFs (c) Chan
     because it much better and faster than sdfatlib
    
    also it was rewritten to:
* distinguish between flie at directory by stat(), not by try to open
* provide last error code and its text description
* added tracking of opened files for global sync()
* some new functions added
    
  original SparkFun readme below
----------------------------------------------

 SD - a slightly more friendly wrapper for sdfatlib

 This library aims to expose a subset of SD card functionality
 in the form of a higher level "wrapper" object.

 License: GNU General Public License V3
          (Because sdfatlib is licensed with this.)

 (C) Copyright 2010 SparkFun Electronics

 */

#ifndef __SD_H__
#define __SD_H__


#include <stdio.h>
#include <util.h>
#include <AP_HAL/AP_HAL.h>

#if defined(BOARD_SDCARD_CS_PIN) || defined(BOARD_DATAFLASH_FATFS)

#include "Sd2Card.h"
#include "SdFatFs.h"


// replace FatFs defines to standard ones
/*
#define FA_READ                         0x01
#define FA_OPEN_EXISTING        0x00

#if !_FS_READONLY
#define FA_WRITE                        0x02
#define FA_CREATE_NEW           0x04
#define FA_CREATE_ALWAYS        0x08
#define FA_OPEN_ALWAYS          0x10
*/
#define O_RDONLY FA_READ
#define O_WRITE  FA_WRITE
#define O_RDWR   FA_WRITE
#define O_CREAT  FA_CREATE_NEW    // создать новый, если существует то ошибка
#define O_TRUNC  FA_CREATE_ALWAYS // переписать старый
#define O_APPEND 0x20
#define O_CLOEXEC 0x40

#undef EEXIST
#undef EACCES
#undef EISDIR
#undef ENOENT

#define EEXIST FR_EXIST
#define EACCES FR_DENIED
#define EISDIR FR_IS_DIR
#define ENOENT FR_NO_FILE

// flags for ls()
/** ls() flag to print modify date */
uint8_t const LS_DATE = 1;
/** ls() flag to print file size */
uint8_t const LS_SIZE = 2;
/** ls() flag for recursive list of subdirectories */
uint8_t const LS_R = 4;

typedef void (*cb_putc)(char c);

class File {
    friend class SDClass;
public:
    File(void);
    File(const char* name);
    size_t write(uint8_t);
    size_t write(const uint8_t *buf, size_t size);
    size_t write(const char *buf, size_t size);

    int read();
    UINT gets(char* buf, size_t len);
    int peek();
    int available();
    void flush();
    int read(void* buf, size_t len);
    uint8_t seek(uint32_t pos);
    uint32_t position();
    uint32_t size();
    void close();
    inline operator bool() const { return  (_name == NULL)? FALSE : TRUE; } // is open?

    char* name(void);
    char* fullname(void) {return _name;};
    uint8_t isDirectory();
    File openNextFile(uint8_t mode = FILE_READ);
    void rewindDirectory(void);

    size_t print(const char* data);
    size_t println();
    size_t println(const char* data);

    // Print to Serial line
    void ls(cb_putc cb, uint8_t flags, uint8_t indent = 0);
    static void printFatDate(uint16_t fatDate, cb_putc cb);
    static void printFatTime(uint16_t fatTime, cb_putc cb);
    static void printTwoDigits(uint8_t v, cb_putc cb);
    static void printNumber(int16_t n, cb_putc cb);
    static void printStr(const char *s, cb_putc cb);

    void inline sync() { if(!is_dir)  f_sync(&_d.fil); };

    inline uint32_t firstCluster(){ return is_dir ? _d.dir.obj.sclust :  _d.fil.obj.sclust; }

    static void syncAll();
    static void addOpenFile(FIL *f);
    static void removeOpenFile(FIL *f);

protected:
// should be private?
    char *_name = NULL; //file or dir name

#if 0
    FIL _fil;  // each struct contains sector buffer so this casue twice of memory use
    DIR _dir;
#else
    union {
        FIL fil;  // each struct contains sector buffer so this casue twice of memory use
        DIR dir;
    } _d;

    bool is_dir;
#endif



// static list of all open files
    static FIL* openFiles[16];
    static uint8_t num_openFiles;

};

class SDClass {

public:

  /* Initialize the SD peripheral */
    static uint8_t begin(AP_HAL::OwnPtr<F4Light::SPIDevice> spi);
    static File open(const char *filepath, uint8_t mode);
    static File open(const char *filepath);
    static uint8_t exists(const char *filepath);
    static uint8_t mkdir(const char *filepath);
    static uint8_t remove(const char *filepath);
    static uint8_t rmdir(const char *filepath);

    static uint32_t getfree(const char *filepath, uint32_t * fssize); 
    static uint8_t stat(const char *filepath, FILINFO* fno);
    static uint8_t format(const char *filepath);

    File openRoot(void);

    void inline sync() {  File::syncAll(); };

    friend class File;

    static inline Sd2Card *getCard() { return &_card; }
    static inline SdFatFs *getVolume() { return &_fatFs; }

    static FRESULT lastError;

    static const char * strError(FRESULT err){ return SdFatFs::strError(err); }
    
private:
    static Sd2Card _card;
    static SdFatFs _fatFs;

};

extern SDClass SD;

#endif
#endif
