/*
  ArduPilot wrapper for ChibiOS FatFS disk I/O bindings.

  This is a copy of modules/ChibiOS/os/various/fatfs_bindings/fatfs_diskio.c
  with get_fattime() removed.  ArduPilot provides a better GPS-based
  get_fattime() in hwdef/common/stm32_util.c; keeping both strong definitions
  in libch.a causes a linker duplicate-symbol error with the updated ChibiOS
  submodule.  chibios_board.mk filters ChibiOS's fatfs_diskio.c out of
  ALLCSRC and adds this file instead.
*/

/*
 * ----------------------------------------------------------------------- Low level disk I/O module skeleton for FatFs (C)ChaN, 2007 ----------------------------------------------------------------------- This is a stub disk I/O module that acts as front end of the existing disk I/O modules and attach it to FatFs module with common interface.
 * -----------------------------------------------------------------------
 */

#include "hal.h"
#include "ffconf.h"
#include "ff.h"
#include "diskio.h"

#if !defined(FATFS_HAL_DEVICE)
#if HAL_USE_SDC
#define FATFS_HAL_DEVICE SDCD1
#else
#define FATFS_HAL_DEVICE MMCD1
#endif
#endif

#if HAL_USE_MMC_SPI
extern MMCDriver FATFS_HAL_DEVICE;
#elif HAL_USE_SDC
extern SDCDriver FATFS_HAL_DEVICE;
#else
#error "MMC_SPI or SDC driver must be specified"
#endif

/*-----------------------------------------------------------------------*/
/* Inidialize a Drive                                                    */

DSTATUS disk_initialize (
    BYTE pdrv         /* Physical drive number (0..) */
)
{
  DSTATUS stat;

  switch (pdrv) {
  case 0:
    stat = 0;
    /* It is initialized externally, just reads the status.*/
    if (blkGetDriverState(&FATFS_HAL_DEVICE) != BLK_READY)
      stat |= STA_NOINIT;
    if (blkIsWriteProtected(&FATFS_HAL_DEVICE))
      stat |= STA_PROTECT;
    return stat;
  }
  return STA_NOINIT;
}

/*-----------------------------------------------------------------------*/
/* Return Disk Status                                                    */

DSTATUS disk_status (
    BYTE pdrv         /* Physical drive number (0..) */
)
{
  DSTATUS stat;

  switch (pdrv) {
  case 0:
    stat = 0;
    /* It is initialized externally, just reads the status.*/
    if (blkGetDriverState(&FATFS_HAL_DEVICE) != BLK_READY)
      stat |= STA_NOINIT;
    if (blkIsWriteProtected(&FATFS_HAL_DEVICE))
      stat |= STA_PROTECT;
    return stat;
  }
  return STA_NOINIT;
}

/*-----------------------------------------------------------------------*/
/* Read Sector(s)                                                        */

DRESULT disk_read (
    BYTE pdrv,        /* Physical drive number (0..) */
    BYTE *buff,       /* Data buffer to store read data */
    DWORD sector,     /* Sector address (LBA) */
    UINT count        /* Number of sectors to read (1..255) */
)
{
  switch (pdrv) {
  case 0:
    if (blkGetDriverState(&FATFS_HAL_DEVICE) != BLK_READY)
      return RES_NOTRDY;
    if (blkRead(&FATFS_HAL_DEVICE, sector, buff, count))
      return RES_ERROR;
    return RES_OK;
  }
  return RES_PARERR;
}

/*-----------------------------------------------------------------------*/
/* Write Sector(s)                                                       */

#if !FF_FS_READONLY
DRESULT disk_write (
    BYTE pdrv,        /* Physical drive number (0..) */
    const BYTE *buff, /* Data to be written */
    DWORD sector,     /* Sector address (LBA) */
    UINT count        /* Number of sectors to write (1..255) */
)
{
  switch (pdrv) {
  case 0:
    if (blkGetDriverState(&FATFS_HAL_DEVICE) != BLK_READY)
      return RES_NOTRDY;
    if (blkWrite(&FATFS_HAL_DEVICE, sector, buff, count))
      return RES_ERROR;
    return RES_OK;
  }
  return RES_PARERR;
}
#endif /* _FS_READONLY */

/*-----------------------------------------------------------------------*/
/* Miscellaneous Functions                                               */

DRESULT disk_ioctl (
    BYTE pdrv,        /* Physical drive number (0..) */
    BYTE cmd,         /* Control code */
    void *buff        /* Buffer to send/receive control data */
)
{
  BlockDeviceInfo bdi;

  (void)buff;

  switch (pdrv) {
  case 0:
    switch (cmd) {
    case CTRL_SYNC:
      return RES_OK;
    case GET_SECTOR_COUNT:
      if (blkGetInfo(&FATFS_HAL_DEVICE, &bdi)) {
        return RES_ERROR;
      }
      *((DWORD *)buff) = bdi.blk_num;
      return RES_OK;
#if FF_MAX_SS > FF_MIN_SS
    case GET_SECTOR_SIZE:
      if (blkGetInfo(&FATFS_HAL_DEVICE, &bdi)) {
        return RES_ERROR;
      }
      *((WORD *)buff) = bdi.blk_size;
      return RES_OK;
#endif
#if FF_USE_TRIM
    case GET_BLOCK_SIZE:
      /* unsupported */
      break;
    case CTRL_TRIM:
      /* unsupported */
      break;
#endif
    default:
      return RES_PARERR;
    }
  }
  return RES_PARERR;
}

/*
  get_fattime() is intentionally NOT defined here.
  ArduPilot's hwdef/common/stm32_util.c provides a GPS-based implementation
  that returns accurate UTC timestamps once a GPS fix is acquired.
*/
