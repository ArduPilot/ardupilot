/*-----------------------------------------------------------------------
  Low level SD card and DataFlash include file
/-----------------------------------------------------------------------*/

#ifndef _SD_DEFINED
#define _SD_DEFINED

#include "../diskio.h"

#include <hal.h>
#include <timer.h>
#include <string.h>

#define FAT_SECTOR_SIZE 512


#ifdef __cplusplus
extern "C" {
#endif

#include "../integer.h"


/*---------------------------------------*/
/* Prototypes for disk control functions */
/*---------------------------------------*/

void sd_timerproc();

DSTATUS sd_initialize ();
DSTATUS sd_status ();
DRESULT sd_read (uint8_t* buff, uint32_t sector, uint16_t count);
DRESULT sd_write (const uint8_t* buff, uint32_t sector, uint16_t count);
DRESULT sd_ioctl (uint8_t cmd, void* buff);
uint8_t sd_get_type();
uint8_t sd_get_state();
uint8_t sd_getSectorCount(uint32_t *ptr);


/* MMC card type flags (MMC_GET_TYPE) */
#define CT_MMC          0x01            /* MMC ver 3 */
#define CT_SD1          0x02            /* SD ver 1 */
#define CT_SD2          0x04            /* SD ver 2 */
#define CT_SDC          (CT_SD1|CT_SD2) /* SD */
#define CT_BLOCK        0x08            /* Block addressing */


#define MMCSD_R1_OUTOFRANGE         ((uint32_t)1 << 31)    /* Bad argument */
#define MMCSD_R1_ADDRESSERROR       ((uint32_t)1 << 30)    /* Bad address */
#define MMCSD_R1_BLOCKLENERROR      ((uint32_t)1 << 29)    /* Bad block length */
#define MMCSD_R1_ERASESEQERROR      ((uint32_t)1 << 28)    /* Erase cmd error */
#define MMCSD_R1_ERASEPARAM         ((uint32_t)1 << 27)    /* Bad write blocks */
#define MMCSD_R1_WPVIOLATION        ((uint32_t)1 << 26)    /* Erase access failure */
#define MMCSD_R1_CARDISLOCKED       ((uint32_t)1 << 25)    /* Card is locked */
#define MMCSD_R1_LOCKUNLOCKFAILED   ((uint32_t)1 << 24)    /* Password error */
#define MMCSD_R1_COMCRCERROR        ((uint32_t)1 << 23)    /* CRC error */
#define MMCSD_R1_ILLEGALCOMMAND     ((uint32_t)1 << 22)    /* Bad command */
#define MMCSD_R1_CARDECCFAILED      ((uint32_t)1 << 21)    /* Failed to correct data */
#define MMCSD_R1_CCERROR            ((uint32_t)1 << 20)    /* Card controller error */
#define MMCSD_R1_ERROR              ((uint32_t)1 << 19)    /* General error */
#define MMCSD_R1_UNDERRUN           ((uint32_t)1 << 18)    /* Underrun (MMC only) */
#define MMCSD_R1_OVERRRUN           ((uint32_t)1 << 17)    /* Overrun (MMC only) */
#define MMCSD_R1_CIDCSDOVERWRITE    ((uint32_t)1 << 16)    /* CID/CSD error */
#define MMCSD_R1_WPERASESKIP        ((uint32_t)1 << 15)    /* Not all erased */
#define MMCSD_R1_CARDECCDISABLED    ((uint32_t)1 << 14)    /* Internal ECC not used */
#define MMCSD_R1_ERASERESET         ((uint32_t)1 << 13)    /* Reset sequence cleared */
#define MMCSD_R1_STATE_SHIFT        (9)                    /* Current card state */
#define MMCSD_R1_STATE_MASK         ((uint32_t)15 << MMCSD_R1_STATE_SHIFT)
                                                           /* Card identification mode states */
#  define MMCSD_R1_STATE_IDLE       ((uint32_t)0 << MMCSD_R1_STATE_SHIFT) /* 0=Idle state */
#  define MMCSD_R1_STATE_READY      ((uint32_t)1 << MMCSD_R1_STATE_SHIFT) /* 1=Ready state */
#  define MMCSD_R1_STATE_IDENT      ((uint32_t)2 << MMCSD_R1_STATE_SHIFT) /* 2=Identification state */
                                                           /* Data transfer states */
#  define MMCSD_R1_STATE_STBY       ((uint32_t)3 << MMCSD_R1_STATE_SHIFT) /* 3=Standby state */
#  define MMCSD_R1_STATE_TRAN       ((uint32_t)4 << MMCSD_R1_STATE_SHIFT) /* 4=Transfer state */
#  define MMCSD_R1_STATE_DATA       ((uint32_t)5 << MMCSD_R1_STATE_SHIFT) /* 5=Sending data state */
#  define MMCSD_R1_STATE_RCV        ((uint32_t)6 << MMCSD_R1_STATE_SHIFT) /* 6=Receiving data state */
#  define MMCSD_R1_STATE_PRG        ((uint32_t)7 << MMCSD_R1_STATE_SHIFT) /* 7=Programming state */
#  define MMCSD_R1_STATE_DIS        ((uint32_t)8 << MMCSD_R1_STATE_SHIFT) /* 8=Disconnect state */
#define MMCSD_R1_READYFORDATA       ((uint32_t)1 << 8)     /* Buffer empty */
#define MMCSD_R1_APPCMD             ((uint32_t)1 << 5)     /* Next CMD is ACMD */
#define MMCSD_R1_AKESEQERROR        ((uint32_t)1 << 3)     /* Authentication error */
#define MMCSD_R1_ERRORMASK          ((uint32_t)0xfdffe008) /* Error mask */


#ifdef __cplusplus
}
#endif

#endif
