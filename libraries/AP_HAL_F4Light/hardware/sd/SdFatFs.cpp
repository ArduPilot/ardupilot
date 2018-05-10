/*
(c) 2017 night_ghost@ykoctpa.ru
 

    intermediate layer for FatFs 

 */



#include "SdFatFs.h"

#if defined(BOARD_SDCARD_CS_PIN) || defined(BOARD_DATAFLASH_FATFS)

FRESULT SdFatFs::init(Sd2Card *card) {

    _card=card;

    _SDPath[0] = '0';
    _SDPath[1] = ':';
    _SDPath[2] = '/';
    _SDPath[3] = 0;

    FRESULT res = f_mount(&_SDFatFs, (TCHAR const*)_SDPath, 1);
    
    /*##-2- Register the file system object to the FatFs module ##############*/
    if(res == FR_OK) {
	/* FatFs Initialization done */
	return res;
    }
    
#if defined(BOARD_DATAFLASH_FATFS) // in DataFlash
    // always reformat internal flash

    printf("Formatting DataFlash to FAT..."); 

#else
    // reformat SD card in case of filesystem error
    if(res!=FR_NO_FILESYSTEM) return res;


    printf("Formatting SD to FAT..."); 
#endif

    res = format((TCHAR const*)_SDPath, card);

    if( res == FR_OK){
        printf(" OK!\n");
    } else {
        printf(" Error: %s!\n", strError(res));
    }


    return res;
}


FRESULT SdFatFs::format(const char *filepath, Sd2Card *card){
#if defined(BOARD_DATAFLASH_FATFS) // in DataFlash

    _card->ioctl(CTRL_FORMAT,0); // clear chip

#endif
/*
    const TCHAR* path,      Logical drive number 
    BYTE opt,               Format option 
    DWORD au,               Size of allocation unit (cluster) [byte] 
    void* work,             Pointer to working buffer (null: use heap memory) 
    UINT len                Size of working buffer [byte] 
*/
    
    char buf[FF_MAX_SS];

    FRESULT res = f_mkfs(filepath, 1 /* unpartitioned */, card->blockSize() /* cluster in sectors */, buf, FF_MAX_SS);

    if(res == FR_OK){
        res = f_mount(&_SDFatFs, filepath, 1);
    }

    return res;
}

uint8_t SdFatFs::fatType(void)
{
	switch (_SDFatFs.fs_type)
	{
	case FS_FAT12:
		return 12;
	case FS_FAT16:
		return 16;
	case FS_FAT32:
		return 32;
	default:
		return 0;
	}
}

const char *SdFatFs::strError(FRESULT err){
    switch(err){
    case    FR_OK:                          /* (0) Succeeded */
        return "no error";
    case    FR_DISK_ERR:                    /* (1) A hard error occurred in the low level disk I/O layer */
        return "Disk error";
    case    FR_INT_ERR:                     /* (2) Assertion failed */
        return "internal error";
    case    FR_NOT_READY:                   /* (3) The physical drive cannot work */
        return "drive not ready";
    case    FR_NO_FILE:                     /* (4) Could not find the file */
        return "no file";
    case    FR_NO_PATH:                     /* (5) Could not find the path */
        return "no path";
    case    FR_INVALID_NAME:                /* (6) The path name format is invalid */
        return "invalid name";
    case    FR_DENIED:                      /* (7) Access denied due to prohibited access or directory full */
        return "access denied";
    case    FR_EXIST:                       /* (8) Access denied due to prohibited access */
        return "file exists";
    case    FR_INVALID_OBJECT:              /* (9) The file/directory object is invalid */
        return "invalid object";
    case    FR_WRITE_PROTECTED:             /* (10) The physical drive is write protected */
        return "write protected";
    case    FR_INVALID_DRIVE:               /* (11) The logical drive number is invalid */
        return "invalid drive";
    case    FR_NOT_ENABLED:                 /* (12) The volume has no work area */
        return "not enabled";
    case    FR_NO_FILESYSTEM:               /* (13) There is no valid FAT volume */
        return "no filesystem";
    case    FR_MKFS_ABORTED:                /* (14) The f_mkfs() aborted due to any parameter error */
        return "MKFS aborted";
    case    FR_TIMEOUT:                     /* (15) Could not get a grant to access the volume within defined period */
        return "timeout";
    case    FR_LOCKED:                      /* (16) The operation is rejected according to the file sharing policy */
        return "locked";
    case    FR_NOT_ENOUGH_CORE:             /* (17) LFN working buffer could not be allocated */
        return "not enough memory";
    case    FR_TOO_MANY_OPEN_FILES: /* (18) Number of open files > _FS_SHARE */
        return "too many files";
    case    FR_INVALID_PARAMETER:   /* (19) Given parameter is invalid */
        return "invalid parameter";
    case    FR_IS_DIR:
        return "is directory";
    }
    return "";
}

#endif

