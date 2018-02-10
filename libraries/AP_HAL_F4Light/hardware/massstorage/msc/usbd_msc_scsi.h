/**
  ******************************************************************************
  * @file    usbd_msc_scsi.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    19-March-2012
  * @brief   header for the usbd_msc_scsi.c file
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USBD_MSC_SCSI_H
#define __USBD_MSC_SCSI_H

/* Includes ------------------------------------------------------------------*/
#include "usbd_def.h"

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */
  
/** @defgroup USBD_SCSI
  * @brief header file for the storage disk file
  * @{
  */ 

/** @defgroup USBD_SCSI_Exported_Defines
  * @{
  */ 

#define SENSE_LIST_DEEPTH                          4

/* SCSI Commands */
enum SCSI_Commands {
    SCSI_FORMAT_UNIT =                           0x04,
    SCSI_INQUIRY       =                         0x12,
    SCSI_MODE_SELECT6  =                         0x15,
    SCSI_MODE_SELECT10 =                         0x55,
    SCSI_MODE_SENSE6   =                         0x1A,
    SCSI_MODE_SENSE10  =                         0x5A,
    SCSI_ALLOW_MEDIUM_REMOVAL =                  0x1E,
    SCSI_READ6         =                         0x08,
    SCSI_READ10        =                         0x28,
    SCSI_READ12        =                         0xA8,
    SCSI_READ16        =                         0x88,

    SCSI_READ_CAPACITY10=                        0x25,
    SCSI_READ_CAPACITY16=                        0x9E,

    SCSI_REQUEST_SENSE  =                        0x03,
    SCSI_START_STOP_UNIT=                        0x1B,
    SCSI_TEST_UNIT_READY=                        0x00,
    SCSI_WRITE6         =                        0x0A,
    SCSI_WRITE10        =                        0x2A,
    SCSI_WRITE12        =                        0xAA,
    SCSI_WRITE16        =                        0x8A,

    SCSI_VERIFY10       =                        0x2F,
    SCSI_VERIFY12       =                        0xAF,
    SCSI_VERIFY16       =                        0x8F,

    SCSI_SEND_DIAGNOSTIC=                        0x1D,
    SCSI_READ_FORMAT_CAPACITIES=                 0x23,
};

enum SCSI_Status {
     NO_SENSE   =                                 0,
     RECOVERED_ERROR =                            1,
     NOT_READY        =                           2,
     MEDIUM_ERROR     =                           3,
     HARDWARE_ERROR   =                           4,
     ILLEGAL_REQUEST  =                           5,
     UNIT_ATTENTION   =                           6,
     DATA_PROTECT     =                           7,
     BLANK_CHECK      =                           8,
     VENDOR_SPECIFIC  =                           9,
     COPY_ABORTED     =                          10,
     ABORTED_COMMAND  =                          11,
     VOLUME_OVERFLOW  =                          13,
     MISCOMPARE       =                          14,
};

enum SCSI_ExtStatus {
    INVALID_CDB                      =           0x20,
    INVALID_FIELED_IN_COMMAND        =           0x24,
    PARAMETER_LIST_LENGTH_ERROR      =           0x1A,
    INVALID_FIELD_IN_PARAMETER_LIST  =           0x26,
    ADDRESS_OUT_OF_RANGE             =           0x21,
    MEDIUM_NOT_PRESENT               =           0x3A,
    MEDIUM_HAVE_CHANGED              =           0x28,
    WRITE_PROTECTED                  =           0x27,
    UNRECOVERED_READ_ERROR	     =           0x11,
    WRITE_FAULT			     =           0x03, 

    READ_FORMAT_CAPACITY_DATA_LEN    =           0x0C,
    READ_CAPACITY10_DATA_LEN         =           0x08,
    MODE_SENSE10_DATA_LEN            =           0x08,
    MODE_SENSE6_DATA_LEN             =           0x04,
    REQUEST_SENSE_DATA_LEN           =           0x12,
    STANDARD_INQUIRY_DATA_LEN        =           0x24,
    BLKVFY                           =           0x04,
};

extern  uint8_t Page00_Inquiry_Data[];
extern  uint8_t Standard_Inquiry_Data[];
extern  uint8_t Standard_Inquiry_Data2[];
extern  uint8_t Mode_Sense6_data[];
extern  uint8_t Mode_Sense10_data[];
extern  uint8_t Scsi_Sense_Data[];
extern  uint8_t ReadCapacity10_Data[];
extern  uint8_t ReadFormatCapacity_Data [];
/**
  * @}
  */ 


/** @defgroup USBD_SCSI_Exported_TypesDefinitions
  * @{
  */

typedef struct _SENSE_ITEM {                
  char Skey;
  union {
    struct _ASCs {
      char ASC;
      char ASCQ;
    }b;
    unsigned int	ASC;
    char *pData;
  } w;
} SCSI_Sense_TypeDef; 
/**
  * @}
  */ 

/** @defgroup USBD_SCSI_Exported_Macros
  * @{
  */ 

/**
  * @}
  */ 

/** @defgroup USBD_SCSI_Exported_Variables
  * @{
  */ 
extern SCSI_Sense_TypeDef     SCSI_Sense [SENSE_LIST_DEEPTH]; 
extern uint8_t   SCSI_Sense_Head;
extern uint8_t   SCSI_Sense_Tail;

/**
  * @}
  */ 
/** @defgroup USBD_SCSI_Exported_FunctionsPrototype
  * @{
  */ 
int8_t SCSI_ProcessCmd(USB_OTG_CORE_HANDLE  *pdev,
                           uint8_t lun, 
                           uint8_t *cmd);

void   SCSI_SenseCode(uint8_t lun, 
                    uint8_t sKey, 
                    uint8_t ASC);


void SCSI_Init(); // needed to start usb IO process
/**
  * @}
  */ 

#endif /* __USBD_MSC_SCSI_H */
/**
  * @}
  */ 

/**
  * @}
  */ 

/**
* @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

