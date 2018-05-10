/*
    night_ghost@ykoctpa.ru 2017
    
    a port of SparkFun's SD class to FatFs (c) Chan
     because it much better and faster than sdfatlib
    
    also it was rewritten to:
* distinguish between flie at directory by stat(), not by try to open
* provide last error code and its text description
* added tracking of opened files for global sync()
* some new functions added
* reduced memory usage by half
    
    
  original SparkFun readme below
----------------------------------------------

 SD - a slightly more friendly wrapper for sdfatlib

 This library aims to expose a subset of SD card functionality
 in the form of a higher level "wrapper" object.

 License: GNU General Public License V3
          (Because sdfatlib is licensed with this.)

 (C) Copyright 2010 SparkFun Electronics


 This library provides four key benefits:

   * Including `SD.h` automatically creates a global
     `SD` object which can be interacted with in a similar
     manner to other standard global objects like `Serial` and `Ethernet`.

   * Boilerplate initialisation code is contained in one method named
     `begin` and no further objects need to be created in order to access
     the SD card.

   * Calls to `open` can supply a full path name including parent
     directories which simplifies interacting with files in subdirectories.

   * Utility methods are provided to determine whether a file exists
     and to create a directory heirarchy.


  Note however that not all functionality provided by the underlying
  sdfatlib library is exposed.

 */


extern "C" {
  #include <stdlib.h>
  #include <string.h>
  #include <inttypes.h>
}

#include "assert.h"
#include <utility>
#include "SD.h"

#if defined(BOARD_SDCARD_CS_PIN) || defined(BOARD_DATAFLASH_FATFS)

SDClass SD;

Sd2Card SDClass::_card;
SdFatFs SDClass::_fatFs;

FRESULT SDClass::lastError = FR_OK;

/**
  * @brief  Link SD, register the file system object to the FatFs mode and configure
  *         relatives SD IOs 
  * @param  None
  * @retval TRUE or FALSE
  */
uint8_t SDClass::begin(AP_HAL::OwnPtr<F4Light::SPIDevice> spi)
{
    if (_card.init(std::move(spi))) {        
        lastError=_fatFs.init(&_card);
	if(lastError == FR_OK) return true;
        printf("\nSD card error: %s\n", strError(lastError));	
	return false;
    } 
    printf("\nSD card init error!\n");
    return FALSE;
}


/**
  * @brief  Check if a file or folder exist on the SD disk
  * @param  filename: File name
  * @retval TRUE or FALSE
  */
uint8_t SDClass::exists(const char *filepath)
{
    FILINFO fno;

    lastError=f_stat(filepath, &fno);

                                // FatFs gives such error for root directory
    return lastError == FR_OK || lastError == FR_INVALID_NAME;
}

/**
  * @brief  Create directory on the SD disk
  * @param  filename: File name
  * @retval TRUE or FALSE
  */
uint8_t SDClass::mkdir(const char *filepath)
{
    lastError = f_mkdir(filepath);
    return lastError == FR_OK;
}

/**
  * @brief  Remove directory on the SD disk
  * @param  filename: File name
  * @retval TRUE or FALSE
  */
uint8_t SDClass::rmdir(const char *filepath)
{
    lastError = f_unlink(filepath);
    return lastError == FR_OK;
}

/**
  * @brief  Open a file on the SD disk, if not existing it's created
  * @param  filename: File name
  * @retval File object referring to the opened file
  */
File SDClass::open(const char *filepath)
{
    File file = File(filepath);

    FILINFO fno;

    lastError=f_stat(filepath, &fno);

    if(lastError!=FR_OK){ // no file
        return file;
    }

    if(fno.fattrib & AM_DIR) {
	lastError = f_opendir(&file._d.dir, filepath);
	file.is_dir=true;
        if( lastError != FR_OK) {
            file.close();
	}
    } else {
        lastError = f_open(&file._d.fil, filepath, FA_READ);
	file.is_dir=false;

        if( lastError == FR_OK) {
            File::addOpenFile(&file._d.fil);
        } else {
            file.close();
	}
    }
    return file;
}

/**
  * @brief  Open a file on the SD disk, if not existing it's created
  * @param  filename: File name
  * @param  mode: the mode in which to open the file
  * @retval File object referring to the opened file
  */
File SDClass::open(const char *filepath, uint8_t mode)
{
    File file = File(filepath);

    FILINFO fno;

    lastError=f_stat(filepath, &fno);

    if(lastError == FR_OK && fno.fattrib & AM_DIR) { // exists and is dir
        if(!(mode & FILE_WRITE)){
            lastError = f_opendir(&file._d.dir, filepath);
            file.is_dir=true;
        } else {
            lastError = FR_IS_DIR;
        }
        if( lastError != FR_OK) file.close();
    } else { // dir not exists - regular file
    
        if((mode & FILE_WRITE) && lastError==FR_OK) { // the modes of opening the file are different. if a file exists
            mode &= ~FA_CREATE_NEW;                  //  then remove the creation flag - or we will got error "file exists"
        }

        lastError = f_open(&file._d.fil, filepath, mode);
        file.is_dir=false;

        if( lastError == FR_OK){
            if(mode & O_APPEND){
                f_lseek(&file._d.fil, f_size(&file._d.fil));            
            }
            File::addOpenFile(&file._d.fil);
        } else {
            file.close();
        }
    }
    return file;
}

/**
  * @brief  Remove a file on the SD disk
  * @param  filename: File name
  * @retval TRUE or FALSE
  */
uint8_t SDClass::remove(const char *filepath)
{
    lastError = f_unlink(filepath);
    return lastError == FR_OK;
}

File SDClass::openRoot(void)
{
    File file = File(_fatFs.getRoot());

    lastError = f_opendir(&file._d.dir, _fatFs.getRoot());
    file.is_dir = true;
    if(lastError != FR_OK) {
	file._d.dir.obj.fs = 0;
    }
    return file;
}



uint32_t SDClass::getfree(const char *filepath, uint32_t * fssize){
    FATFS *fs;
    DWORD fre_clust, fre_sect;


    /* Get volume information and free clusters of drive */
    lastError = f_getfree(filepath, &fre_clust, &fs);
    if (lastError != FR_OK) return -1;

    /* Get total sectors and free sectors */
    if(fssize) *fssize = (fs->n_fatent - 2) * fs->csize; // tot_sect
    fre_sect = fre_clust * fs->csize;
    
    return fre_sect;
}


//f_stat (        const TCHAR* path,      /* Pointer to the file path */         FILINFO* fno )
uint8_t SDClass::stat(const char *filepath, FILINFO* fno){
    lastError = f_stat(filepath, fno);
    if(lastError != FR_OK) return -1;
    return 0;
}

uint8_t SDClass::format(const char *filepath){
    
    lastError = _fatFs.format(filepath, &_card);
    
    return lastError == FR_OK;
}



//* *************************************
File::File()
{
    _name = NULL;
     _d.fil.obj.fs = 0;
     _d.dir.obj.fs = 0;
}

File::File(const char* fname)
{
    _name = (char*)malloc(strlen(fname) +1);

    //assert(_name  != NULL );
    if(_name == NULL) return;  // no HardFault, just not opened
    
    strcpy(_name, fname);
    _d.fil.obj.fs = 0;
    _d.dir.obj.fs = 0;
}

/** List directory contents to given callback
 *
 * \param[in] flags The inclusive OR of
 *
 * LS_DATE - %Print file modification date
 *
 * LS_SIZE - %Print file size.
 *
 * LS_R - Recursive list of subdirectories.
 *
 * \param[in] indent Amount of space before file name. Used for recursive
 * list to indicate subdirectory level.
 */
void File::ls(cb_putc cb, uint8_t flags, uint8_t indent) {
  FRESULT res = FR_OK;
  FILINFO fno;
  char *fn;

    if(!is_dir) return;

#if _USE_LFN
  static char lfn[_MAX_LFN];
  fno.lfname = lfn;
  fno.lfsize = sizeof(lfn);
#endif

  while(1) {
    res = f_readdir(&_d.dir, &fno);
    if(res != FR_OK || fno.fname[0] == 0) {
      break;
    }
    if(fno.fname[0] == '.') {
      continue;
    }
#if _USE_LFN
    fn = *fno.lfname ? fno.lfname : fno.fname;
#else
    fn = fno.fname;
#endif
	//print any indent spaces
    for (int8_t i = 0; i < indent; i++) cb(' ');
    printStr(fn, cb);

    if((fno.fattrib & AM_DIR) == 0)  {
	  // print modify date/time if requested
	  if (flags & LS_DATE) {
		cb(' ');
		printFatDate(fno.fdate,cb);
		cb(' ');
		printFatTime(fno.ftime,cb);
	  }
	  // print size if requested
	  if (flags & LS_SIZE) {
	    cb(' ');
	    printNumber(fno.fsize, cb);
	  }
	  cb('\r'); cb('\n');
    }
	else
	{
	  // list subdirectory content if requested
	  if (flags & LS_R)
	  {
		char *fullPath;
		fullPath = (char*)malloc(strlen(_name) + 1 + strlen(fn) +1);
		if (fullPath != NULL) {
		  sprintf(fullPath, "%s/%s", _name, fn);
		  File filtmp = SD.open(fullPath);

		  if (filtmp._name != NULL) {
                	cb('\r'); cb('\n');
			filtmp.ls(cb,flags, indent+2);
			filtmp.close();
		  } else {
			printStr(fn, cb);
			cb('\r'); cb('\n');

                        static const char err_s[] = "Error to open dir: ";
			printStr(err_s, cb);
			printStr(fn, cb);
		  }
		  free(fullPath);
		} else {
		  cb('\r'); cb('\n');
		  static const char err_s[] = "Error to allocate memory!";
		  printStr(err_s, cb);
		}
	  }
	}
  }
}
//------------------------------------------------------------------------------
/** %Print a directory date field to Serial.
 *
 *  Format is yyyy-mm-dd.
 *
 * \param[in] fatDate The date field from a directory entry.
 */
void File::printFatDate(uint16_t fatDate, cb_putc cb) {
  printNumber(FAT_YEAR(fatDate), cb);
  cb('-');
  printTwoDigits(FAT_MONTH(fatDate),cb);
  cb('-');
  printTwoDigits(FAT_DAY(fatDate),cb);
}
//------------------------------------------------------------------------------
/** %Print a directory time field to Serial.
 *
 * Format is hh:mm:ss.
 *
 * \param[in] fatTime The time field from a directory entry.
 */
void File::printFatTime(uint16_t fatTime, cb_putc cb) {
  printTwoDigits(FAT_HOUR(fatTime), cb);
  cb(':');
  printTwoDigits(FAT_MINUTE(fatTime), cb);
  cb(':');
  printTwoDigits(FAT_SECOND(fatTime), cb);
}
//------------------------------------------------------------------------------
/** %Print a value as two digits to Serial.
 *
 * \param[in] v Value to be printed, 0 <= \a v <= 99
 */
void File::printTwoDigits(uint8_t v, cb_putc cb) {
  cb('0' + v/10);
  cb('0' + v % 10);
}

void File::printNumber(int16_t n, cb_putc cb) {
    const uint8_t base = 10;

    if (n < 0) {
      cb('-');
      n = -n;
    }

    char buf[8 * sizeof(long) + 1]; // Assumes 8-bit chars plus zero byte.
    char *str = &buf[sizeof(buf) - 1];

    *str = '\0';
    
    do {
        char c = n % base;
        n /= base;

        *--str = c < 10 ? c + '0' : c + 'A' - 10;
    } while(n);

    printStr(str, cb);
}

void File::printStr(const char *s, cb_putc cb) {
    while(*s) cb(*s++);
}


/**
  * @brief  Read byte from the file
  * @retval Byte read
  */
int File::read()
{
    UINT byteread;
    int8_t data;
    SD.lastError = f_read(&_d.fil, (void *)&data, 1, &byteread);
    return data;
}

/**
  * @brief  Read an amount of data from the file
  * @param  buf: an array to store the read data from the file
  * @param  len: the number of elements to read
  * @retval Number of bytes read
  */
int File::read(void* buf, size_t len)
{
    UINT bytesread;

    SD.lastError = f_read(&_d.fil, buf, len, &bytesread);
    return bytesread;

}

UINT File::gets(char* buf, size_t len)
{
    UINT bytesread=0;

    while(len--){

        uint8_t c;
        uint8_t ret = read(&c,1);
        if(!ret) break; // EOF
        if(c=='\n') break;
        if(c=='\r') continue;
        *buf++=c;
        *buf=0;   // close string
        bytesread++;
    }        
    return bytesread;

}

/**
  * @brief  Close a file on the SD disk
  * @param  None
  * @retval None
  */
void File::close()
{
    if(_name){
        if(is_dir) {
            if(_d.dir.obj.fs != 0) {
	        SD.lastError = f_closedir(&_d.dir);
	    }
        } else {
            if(_d.fil.obj.fs != 0) {
	        /* Flush the file before close */
	        f_sync(&_d.fil);

                /* Close the file */
	        SD.lastError = f_close(&_d.fil);
            }

            removeOpenFile(&_d.fil);
            free(_name);
	    _name=NULL;
	}
    }
}


/**
  * @brief  Ensures that any bytes written to the file are physically saved to the SD card
  * @param  None
  * @retval None
  */
void File::flush()
{
    if(!is_dir) {
        SD.lastError = f_sync(&_d.fil);
    }
}

/**
  * @brief  Read a byte from the file without advancing to the next one
  * @param  None
  * @retval read byte
  */
int File::peek()
{
    int data;
    data = read();
    seek(position() -1);
    return data;
}

/**
  * @brief  Get the current position within the file
  * @param  None
  * @retval position within file
  */
uint32_t File::position()
{
    return f_tell(&_d.fil);
}

/**
  * @brief  Seek to a new position in the file
  * @param  pos: The position to which to seek
  * @retval TRUE or FALSE
  */
uint8_t File::seek(uint32_t pos)
{
    if(is_dir) return false;

    if(pos > size()) {
        return FALSE;
    } else {
        SD.lastError = f_lseek(&_d.fil, pos);
        return SD.lastError == FR_OK;
    }
}

/**
  * @brief  Get the size of the file
  * @param  None
  * @retval file's size
  */
uint32_t File::size()
{
    if(is_dir) return 0;
    return f_size(&_d.fil);
}


/**
  * @brief  Write data to the file
  * @param  data: Data to write to the file
  * @retval Number of data written (1)
  */
size_t File::write(uint8_t data)
{
  return write(&data, 1);
}

/**
  * @brief  Write an array of data to the file
  * @param  buf: an array of characters or bytes to write to the file
  * @param  len: the number of elements in buf
  * @retval Number of data written
  */
size_t File::write(const char *buf, size_t sz)
{
    size_t byteswritten;
    if(is_dir) return 0;
    
    SD.lastError = f_write(&_d.fil, (const void *)buf, sz, (UINT *)&byteswritten);
    return byteswritten;
}

size_t File::write(const uint8_t *buf, size_t sz)
{
    return write((const char *)buf, sz);
}

/**
  * @brief  Print data to the file
  * @param  data: Data to write to the file
  * @retval Number of data written (1)
  */
size_t File::print(const char* data)
{
    return write(data, strlen(data));
}

/**
  * @brief  Print data to the file
  * @retval Number of data written (1)
  */
size_t File::println()
{
    return write("\r\n", 2);
}

/**
  * @brief  Print data to the file
  * @param  data: Data to write to the file
  * @retval Number of data written (1)
  */
size_t File::println(const char* data)
{
	size_t bytewritten = write(data, strlen(data));
	bytewritten += println();
	return bytewritten;
}


/**
  * @brief  Check if there are any bytes available for reading from the file
  * @retval Number of bytes available
  */
int File::available()
{
    uint32_t n = size() - position();
    return n > 0x7FFF ? 0x7FFF : n;
}


char* File::name()
{
	char *fname = strrchr(_name, '/');
	if (fname && fname[0] == '/')
		fname++;
	return fname;
}

/**
  * @brief  Check if the file is directory or normal file
  * @retval TRUE if directory else FALSE
  */
uint8_t File::isDirectory()
{
    //assert(_name  != NULL );
    
    if(_name == NULL) return false;
    
    
    if (is_dir){
        if(_d.dir.obj.fs != 0) return TRUE;
    } else {
        if(_d.fil.obj.fs != 0) return FALSE;
    }

    // if not init get info
    FILINFO fno;
    
    SD.lastError = f_stat(_name, &fno);
    if (SD.lastError == FR_OK) {
	if(fno.fattrib & AM_DIR){
	    is_dir = true;
	    return TRUE;
	} else {
	    is_dir=false;
	    return FALSE;
	}
    }

    return FALSE;
}


// TODO: some strange and not works at all
File File::openNextFile(uint8_t mode)
{
    if(!is_dir) return File();
    
  FRESULT res = FR_OK;
  FILINFO fno;
  char *fn;
  char *fullPath = NULL;
  size_t name_len= strlen(_name);
  size_t len = name_len;
#if _USE_LFN
  static char lfn[_MAX_LFN];
  fno.lfname = lfn;
  fno.lfsize = sizeof(lfn);
#endif
  while(1) {
    res = f_readdir(&_d.dir, &fno);
    if(res != FR_OK || fno.fname[0] == 0) {
      return File();
    }
    if(fno.fname[0] == '.') {
      continue;
    }
#if _USE_LFN
    fn = *fno.lfname ? fno.lfname : fno.fname;
#else
    fn = fno.fname;
#endif
	len += strlen(fn) +2;
	fullPath = (char*)malloc(len);
	if (fullPath != NULL) {
	    // Avoid twice '/'
	    if ((name_len > 0)  && (_name[name_len-1] == '/'))  {
                sprintf(fullPath, "%s%s", _name, fn);
	    } else {
	        sprintf(fullPath, "%s/%s", _name, fn);
	    }
	    File filtmp = SD.open(fullPath, mode);
	    free(fullPath);
	    return filtmp;
	} else {
	    return File();
	}
  }
}

void File::rewindDirectory(void)
{
    if(isDirectory()) {
	if(_d.dir.obj.fs != 0) {
	    f_closedir(&_d.dir);
	}
	f_opendir(&_d.dir, _name);
    }
}

#define MAX_OPEN_FILES 16
FIL* File::openFiles[MAX_OPEN_FILES]= {0};
uint8_t File::num_openFiles=0;

void File::syncAll(){
    for(uint8_t i=0; i<num_openFiles;i++){
        if(openFiles[i]) 
            f_sync(openFiles[i]);
    }
}

void File::addOpenFile(FIL *f){
    for(uint8_t i=0; i<num_openFiles;i++){
        if(!openFiles[i]) {
            openFiles[i] = f;
            return;
        }
    }
    if(num_openFiles<MAX_OPEN_FILES){
        openFiles[num_openFiles++] = f;
    }
}

void File::removeOpenFile(FIL *f){
    for(uint8_t i=0; i<num_openFiles;i++){
        if(openFiles[i] == f) {
            openFiles[i] = NULL;
            return;
        }
    }
}

#endif
