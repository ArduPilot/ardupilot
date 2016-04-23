/**
  ******************************************************************************
  * @file    stm32f4xx_hash_sha1.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    11-January-2013
  * @brief   This file provides high level functions to compute the HASH SHA1 and
  *          HMAC SHA1 Digest of an input message.
  *          It uses the stm32f4xx_hash.c/.h drivers to access the STM32F4xx HASH
  *          peripheral.
  *
@verbatim
 ===================================================================
                 ##### How to use this driver #####
 ===================================================================
 [..]
   (#) Enable The HASH controller clock using 
       RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_HASH, ENABLE); function.
  
   (#) Calculate the HASH SHA1 Digest using HASH_SHA1() function.
  
   (#) Calculate the HMAC SHA1 Digest using HMAC_SHA1() function.
  
@endverbatim
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2013 STMicroelectronics</center></h2>
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

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hash.h"

/** @addtogroup STM32F4xx_StdPeriph_Driver
  * @{
  */

/** @defgroup HASH 
  * @brief HASH driver modules
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define SHA1BUSY_TIMEOUT    ((uint32_t) 0x00010000)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup HASH_Private_Functions
  * @{
  */ 

/** @defgroup HASH_Group6 High Level SHA1 functions
 *  @brief   High Level SHA1 Hash and HMAC functions 
 *
@verbatim   
 ===============================================================================
               ##### High Level SHA1 Hash and HMAC functions #####
 ===============================================================================


@endverbatim
  * @{
  */

/**
  * @brief  Compute the HASH SHA1 digest.
  * @param  Input: pointer to the Input buffer to be treated.
  * @param  Ilen: length of the Input buffer.
  * @param  Output: the returned digest
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: digest computation done
  *          - ERROR: digest computation failed
  */
ErrorStatus HASH_SHA1(uint8_t *Input, uint32_t Ilen, uint8_t Output[20])
{
  HASH_InitTypeDef SHA1_HASH_InitStructure;
  HASH_MsgDigest SHA1_MessageDigest;
  __IO uint16_t nbvalidbitsdata = 0;
  uint32_t i = 0;
  __IO uint32_t counter = 0;
  uint32_t busystatus = 0;
  ErrorStatus status = SUCCESS;
  uint32_t inputaddr  = (uint32_t)Input;
  uint32_t outputaddr = (uint32_t)Output;

  /* Number of valid bits in last word of the Input data */
  nbvalidbitsdata = 8 * (Ilen % 4);

  /* HASH peripheral initialization */
  HASH_DeInit();

  /* HASH Configuration */
  SHA1_HASH_InitStructure.HASH_AlgoSelection = HASH_AlgoSelection_SHA1;
  SHA1_HASH_InitStructure.HASH_AlgoMode = HASH_AlgoMode_HASH;
  SHA1_HASH_InitStructure.HASH_DataType = HASH_DataType_8b;
  HASH_Init(&SHA1_HASH_InitStructure);

  /* Configure the number of valid bits in last word of the data */
  HASH_SetLastWordValidBitsNbr(nbvalidbitsdata);

  /* Write the Input block in the IN FIFO */
  for(i=0; i<Ilen; i+=4)
  {
    HASH_DataIn(*(uint32_t*)inputaddr);
    inputaddr+=4;
  }

  /* Start the HASH processor */
  HASH_StartDigest();

  /* wait until the Busy flag is RESET */
  do
  {
    busystatus = HASH_GetFlagStatus(HASH_FLAG_BUSY);
    counter++;
  }while ((counter != SHA1BUSY_TIMEOUT) && (busystatus != RESET));

  if (busystatus != RESET)
  {
     status = ERROR;
  }
  else
  {
    /* Read the message digest */
    HASH_GetDigest(&SHA1_MessageDigest);
    *(uint32_t*)(outputaddr)  = __REV(SHA1_MessageDigest.Data[0]);
    outputaddr+=4;
    *(uint32_t*)(outputaddr)  = __REV(SHA1_MessageDigest.Data[1]);
    outputaddr+=4;
    *(uint32_t*)(outputaddr)  = __REV(SHA1_MessageDigest.Data[2]);
    outputaddr+=4;
    *(uint32_t*)(outputaddr)  = __REV(SHA1_MessageDigest.Data[3]);
    outputaddr+=4;
    *(uint32_t*)(outputaddr)  = __REV(SHA1_MessageDigest.Data[4]);
  }
  return status;
}

/**
  * @brief  Compute the HMAC SHA1 digest.
  * @param  Key: pointer to the Key used for HMAC.
  * @param  Keylen: length of the Key used for HMAC.  
  * @param  Input: pointer to the Input buffer to be treated.
  * @param  Ilen: length of the Input buffer.
  * @param  Output: the returned digest
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: digest computation done
  *          - ERROR: digest computation failed
  */
ErrorStatus HMAC_SHA1(uint8_t *Key, uint32_t Keylen, uint8_t *Input,
                      uint32_t Ilen, uint8_t Output[20])
{
  HASH_InitTypeDef SHA1_HASH_InitStructure;
  HASH_MsgDigest SHA1_MessageDigest;
  __IO uint16_t nbvalidbitsdata = 0;
  __IO uint16_t nbvalidbitskey = 0;
  uint32_t i = 0;
  __IO uint32_t counter = 0;
  uint32_t busystatus = 0;
  ErrorStatus status = SUCCESS;
  uint32_t keyaddr    = (uint32_t)Key;
  uint32_t inputaddr  = (uint32_t)Input;
  uint32_t outputaddr = (uint32_t)Output;

  /* Number of valid bits in last word of the Input data */
  nbvalidbitsdata = 8 * (Ilen % 4);

  /* Number of valid bits in last word of the Key */
  nbvalidbitskey = 8 * (Keylen % 4);

  /* HASH peripheral initialization */
  HASH_DeInit();

  /* HASH Configuration */
  SHA1_HASH_InitStructure.HASH_AlgoSelection = HASH_AlgoSelection_SHA1;
  SHA1_HASH_InitStructure.HASH_AlgoMode = HASH_AlgoMode_HMAC;
  SHA1_HASH_InitStructure.HASH_DataType = HASH_DataType_8b;
  if(Keylen > 64)
  {
    /* HMAC long Key */
    SHA1_HASH_InitStructure.HASH_HMACKeyType = HASH_HMACKeyType_LongKey;
  }
  else
  {
    /* HMAC short Key */
    SHA1_HASH_InitStructure.HASH_HMACKeyType = HASH_HMACKeyType_ShortKey;
  }
  HASH_Init(&SHA1_HASH_InitStructure);

  /* Configure the number of valid bits in last word of the Key */
  HASH_SetLastWordValidBitsNbr(nbvalidbitskey);

  /* Write the Key */
  for(i=0; i<Keylen; i+=4)
  {
    HASH_DataIn(*(uint32_t*)keyaddr);
    keyaddr+=4;
  }

  /* Start the HASH processor */
  HASH_StartDigest();

  /* wait until the Busy flag is RESET */
  do
  {
    busystatus = HASH_GetFlagStatus(HASH_FLAG_BUSY);
    counter++;
  }while ((counter != SHA1BUSY_TIMEOUT) && (busystatus != RESET));

  if (busystatus != RESET)
  {
     status = ERROR;
  }
  else
  {
    /* Configure the number of valid bits in last word of the Input data */
    HASH_SetLastWordValidBitsNbr(nbvalidbitsdata);

    /* Write the Input block in the IN FIFO */
    for(i=0; i<Ilen; i+=4)
    {
      HASH_DataIn(*(uint32_t*)inputaddr);
      inputaddr+=4;
    }

    /* Start the HASH processor */
    HASH_StartDigest();


    /* wait until the Busy flag is RESET */
    counter =0;
    do
    {
      busystatus = HASH_GetFlagStatus(HASH_FLAG_BUSY);
      counter++;
    }while ((counter != SHA1BUSY_TIMEOUT) && (busystatus != RESET));

    if (busystatus != RESET)
    {
      status = ERROR;
    }
    else
    {  
      /* Configure the number of valid bits in last word of the Key */
      HASH_SetLastWordValidBitsNbr(nbvalidbitskey);

      /* Write the Key */
      keyaddr = (uint32_t)Key;
      for(i=0; i<Keylen; i+=4)
      {
        HASH_DataIn(*(uint32_t*)keyaddr);
        keyaddr+=4;
      }

      /* Start the HASH processor */
      HASH_StartDigest();

      /* wait until the Busy flag is RESET */
      counter =0;
      do
      {
        busystatus = HASH_GetFlagStatus(HASH_FLAG_BUSY);
        counter++;
      }while ((counter != SHA1BUSY_TIMEOUT) && (busystatus != RESET));

      if (busystatus != RESET)
      {
        status = ERROR;
      }
      else
      {
        /* Read the message digest */
        HASH_GetDigest(&SHA1_MessageDigest);
        *(uint32_t*)(outputaddr)  = __REV(SHA1_MessageDigest.Data[0]);
        outputaddr+=4;
        *(uint32_t*)(outputaddr)  = __REV(SHA1_MessageDigest.Data[1]);
        outputaddr+=4;
        *(uint32_t*)(outputaddr)  = __REV(SHA1_MessageDigest.Data[2]);
        outputaddr+=4;
        *(uint32_t*)(outputaddr)  = __REV(SHA1_MessageDigest.Data[3]);
        outputaddr+=4;
        *(uint32_t*)(outputaddr)  = __REV(SHA1_MessageDigest.Data[4]);
      }
    }  
  }
  return status;  
}
/**
  * @}
  */ 

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
