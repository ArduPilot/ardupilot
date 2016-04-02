/**
  ******************************************************************************
  * @file    stm32f4xx_cryp_aes.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    11-January-2013
  * @brief   This file provides high level functions to encrypt and decrypt an 
  *          input message using AES in ECB/CBC/CTR/GCM/CCM modes.
  *          It uses the stm32f4xx_cryp.c/.h drivers to access the STM32F4xx CRYP
  *          peripheral.
  *          AES-ECB/CBC/CTR/GCM/CCM modes are available on STM32F437x Devices.
  *          For STM32F41xx Devices, only AES-ECB/CBC/CTR modes are available.
  *
@verbatim
 ===================================================================
                  ##### How to use this driver #####
 ===================================================================
 [..]
   (#) Enable The CRYP controller clock using 
      RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_CRYP, ENABLE); function.
  
   (#) Encrypt and decrypt using AES in ECB Mode using CRYP_AES_ECB() function.
  
   (#) Encrypt and decrypt using AES in CBC Mode using CRYP_AES_CBC() function.
  
   (#) Encrypt and decrypt using AES in CTR Mode using CRYP_AES_CTR() function.

   (#) Encrypt and decrypt using AES in GCM Mode using CRYP_AES_GCM() function.
   
   (#) Encrypt and decrypt using AES in CCM Mode using CRYP_AES_CCM() function.
     
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
#include "stm32f4xx_cryp.h"

/** @addtogroup STM32F4xx_StdPeriph_Driver
  * @{
  */

/** @defgroup CRYP 
  * @brief CRYP driver modules
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define AESBUSY_TIMEOUT    ((uint32_t) 0x00010000)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/** @defgroup CRYP_Private_Functions
  * @{
  */ 

/** @defgroup CRYP_Group6 High Level AES functions
 *  @brief   High Level AES functions 
 *
@verbatim   
 ===============================================================================
                       ##### High Level AES functions #####
 ===============================================================================

@endverbatim
  * @{
  */

/**
  * @brief  Encrypt and decrypt using AES in ECB Mode
  * @param  Mode: encryption or decryption Mode.
  *          This parameter can be one of the following values:
  *            @arg MODE_ENCRYPT: Encryption
  *            @arg MODE_DECRYPT: Decryption
  * @param  Key: Key used for AES algorithm.
  * @param  Keysize: length of the Key, must be a 128, 192 or 256.
  * @param  Input: pointer to the Input buffer.
  * @param  Ilength: length of the Input buffer, must be a multiple of 16.
  * @param  Output: pointer to the returned buffer.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: Operation done
  *          - ERROR: Operation failed
  */
ErrorStatus CRYP_AES_ECB(uint8_t Mode, uint8_t* Key, uint16_t Keysize,
                         uint8_t* Input, uint32_t Ilength, uint8_t* Output)
{
  CRYP_InitTypeDef AES_CRYP_InitStructure;
  CRYP_KeyInitTypeDef AES_CRYP_KeyInitStructure;
  __IO uint32_t counter = 0;
  uint32_t busystatus = 0;
  ErrorStatus status = SUCCESS;
  uint32_t keyaddr    = (uint32_t)Key;
  uint32_t inputaddr  = (uint32_t)Input;
  uint32_t outputaddr = (uint32_t)Output;
  uint32_t i = 0;

  /* Crypto structures initialisation*/
  CRYP_KeyStructInit(&AES_CRYP_KeyInitStructure);

  switch(Keysize)
  {
    case 128:
    AES_CRYP_InitStructure.CRYP_KeySize = CRYP_KeySize_128b;
    AES_CRYP_KeyInitStructure.CRYP_Key2Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key2Right= __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key3Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key3Right= __REV(*(uint32_t*)(keyaddr));
    break;
    case 192:
    AES_CRYP_InitStructure.CRYP_KeySize  = CRYP_KeySize_192b;
    AES_CRYP_KeyInitStructure.CRYP_Key1Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key1Right= __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key2Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key2Right= __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key3Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key3Right= __REV(*(uint32_t*)(keyaddr));
    break;
    case 256:
    AES_CRYP_InitStructure.CRYP_KeySize  = CRYP_KeySize_256b;
    AES_CRYP_KeyInitStructure.CRYP_Key0Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key0Right= __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key1Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key1Right= __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key2Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key2Right= __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key3Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key3Right= __REV(*(uint32_t*)(keyaddr));
    break;
    default:
    break;
  }

  /*------------------ AES Decryption ------------------*/
  if(Mode == MODE_DECRYPT) /* AES decryption */
  {
    /* Flush IN/OUT FIFOs */
    CRYP_FIFOFlush();

    /* Crypto Init for Key preparation for decryption process */
    AES_CRYP_InitStructure.CRYP_AlgoDir = CRYP_AlgoDir_Decrypt;
    AES_CRYP_InitStructure.CRYP_AlgoMode = CRYP_AlgoMode_AES_Key;
    AES_CRYP_InitStructure.CRYP_DataType = CRYP_DataType_32b;
    CRYP_Init(&AES_CRYP_InitStructure);

    /* Key Initialisation */
    CRYP_KeyInit(&AES_CRYP_KeyInitStructure);

    /* Enable Crypto processor */
    CRYP_Cmd(ENABLE);

    /* wait until the Busy flag is RESET */
    do
    {
      busystatus = CRYP_GetFlagStatus(CRYP_FLAG_BUSY);
      counter++;
    }while ((counter != AESBUSY_TIMEOUT) && (busystatus != RESET));

    if (busystatus != RESET)
   {
       status = ERROR;
    }
    else
    {
      /* Crypto Init for decryption process */  
      AES_CRYP_InitStructure.CRYP_AlgoDir = CRYP_AlgoDir_Decrypt;
    }
  }
  /*------------------ AES Encryption ------------------*/
  else /* AES encryption */
  {

    CRYP_KeyInit(&AES_CRYP_KeyInitStructure);

    /* Crypto Init for Encryption process */
    AES_CRYP_InitStructure.CRYP_AlgoDir  = CRYP_AlgoDir_Encrypt;
  }

  AES_CRYP_InitStructure.CRYP_AlgoMode = CRYP_AlgoMode_AES_ECB;
  AES_CRYP_InitStructure.CRYP_DataType = CRYP_DataType_8b;
  CRYP_Init(&AES_CRYP_InitStructure);

  /* Flush IN/OUT FIFOs */
  CRYP_FIFOFlush();

  /* Enable Crypto processor */
  CRYP_Cmd(ENABLE);

  if(CRYP_GetCmdStatus() == DISABLE)
  {
    /* The CRYP peripheral clock is not enabled or the device doesn't embedd 
       the CRYP peripheral (please check the device sales type. */
    return(ERROR);
  }
  
  for(i=0; ((i<Ilength) && (status != ERROR)); i+=16)
  {

    /* Write the Input block in the IN FIFO */
    CRYP_DataIn(*(uint32_t*)(inputaddr));
    inputaddr+=4;
    CRYP_DataIn(*(uint32_t*)(inputaddr));
    inputaddr+=4;
    CRYP_DataIn(*(uint32_t*)(inputaddr));
    inputaddr+=4;
    CRYP_DataIn(*(uint32_t*)(inputaddr));
    inputaddr+=4;

    /* Wait until the complete message has been processed */
    counter = 0;
    do
    {
      busystatus = CRYP_GetFlagStatus(CRYP_FLAG_BUSY);
      counter++;
    }while ((counter != AESBUSY_TIMEOUT) && (busystatus != RESET));

    if (busystatus != RESET)
   {
       status = ERROR;
    }
    else
    {

      /* Read the Output block from the Output FIFO */
      *(uint32_t*)(outputaddr) = CRYP_DataOut();
      outputaddr+=4;
      *(uint32_t*)(outputaddr) = CRYP_DataOut();
      outputaddr+=4;
      *(uint32_t*)(outputaddr) = CRYP_DataOut();
      outputaddr+=4;
      *(uint32_t*)(outputaddr) = CRYP_DataOut(); 
      outputaddr+=4;
    }
  }

  /* Disable Crypto */
  CRYP_Cmd(DISABLE);

  return status; 
}

/**
  * @brief  Encrypt and decrypt using AES in CBC Mode
  * @param  Mode: encryption or decryption Mode.
  *          This parameter can be one of the following values:
  *            @arg MODE_ENCRYPT: Encryption
  *            @arg MODE_DECRYPT: Decryption
  * @param  InitVectors: Initialisation Vectors used for AES algorithm.
  * @param  Key: Key used for AES algorithm.
  * @param  Keysize: length of the Key, must be a 128, 192 or 256.
  * @param  Input: pointer to the Input buffer.
  * @param  Ilength: length of the Input buffer, must be a multiple of 16.
  * @param  Output: pointer to the returned buffer.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: Operation done
  *          - ERROR: Operation failed
  */
ErrorStatus CRYP_AES_CBC(uint8_t Mode, uint8_t InitVectors[16], uint8_t *Key,
                         uint16_t Keysize, uint8_t *Input, uint32_t Ilength,
                         uint8_t *Output)
{
  CRYP_InitTypeDef AES_CRYP_InitStructure;
  CRYP_KeyInitTypeDef AES_CRYP_KeyInitStructure;
  CRYP_IVInitTypeDef AES_CRYP_IVInitStructure;
  __IO uint32_t counter = 0;
  uint32_t busystatus = 0;
  ErrorStatus status = SUCCESS;
  uint32_t keyaddr    = (uint32_t)Key;
  uint32_t inputaddr  = (uint32_t)Input;
  uint32_t outputaddr = (uint32_t)Output;
  uint32_t ivaddr = (uint32_t)InitVectors;
  uint32_t i = 0;

  /* Crypto structures initialisation*/
  CRYP_KeyStructInit(&AES_CRYP_KeyInitStructure);

  switch(Keysize)
  {
    case 128:
    AES_CRYP_InitStructure.CRYP_KeySize = CRYP_KeySize_128b;
    AES_CRYP_KeyInitStructure.CRYP_Key2Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key2Right= __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key3Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key3Right= __REV(*(uint32_t*)(keyaddr));
    break;
    case 192:
    AES_CRYP_InitStructure.CRYP_KeySize  = CRYP_KeySize_192b;
    AES_CRYP_KeyInitStructure.CRYP_Key1Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key1Right= __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key2Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key2Right= __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key3Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key3Right= __REV(*(uint32_t*)(keyaddr));
    break;
    case 256:
    AES_CRYP_InitStructure.CRYP_KeySize  = CRYP_KeySize_256b;
    AES_CRYP_KeyInitStructure.CRYP_Key0Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key0Right= __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key1Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key1Right= __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key2Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key2Right= __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key3Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key3Right= __REV(*(uint32_t*)(keyaddr));
    break;
    default:
    break;
  }

  /* CRYP Initialization Vectors */
  AES_CRYP_IVInitStructure.CRYP_IV0Left = __REV(*(uint32_t*)(ivaddr));
  ivaddr+=4;
  AES_CRYP_IVInitStructure.CRYP_IV0Right= __REV(*(uint32_t*)(ivaddr));
  ivaddr+=4;
  AES_CRYP_IVInitStructure.CRYP_IV1Left = __REV(*(uint32_t*)(ivaddr));
  ivaddr+=4;
  AES_CRYP_IVInitStructure.CRYP_IV1Right= __REV(*(uint32_t*)(ivaddr));


  /*------------------ AES Decryption ------------------*/
  if(Mode == MODE_DECRYPT) /* AES decryption */
  {
    /* Flush IN/OUT FIFOs */
    CRYP_FIFOFlush();

    /* Crypto Init for Key preparation for decryption process */
    AES_CRYP_InitStructure.CRYP_AlgoDir = CRYP_AlgoDir_Decrypt;
    AES_CRYP_InitStructure.CRYP_AlgoMode = CRYP_AlgoMode_AES_Key;
    AES_CRYP_InitStructure.CRYP_DataType = CRYP_DataType_32b;

    CRYP_Init(&AES_CRYP_InitStructure);

    /* Key Initialisation */
    CRYP_KeyInit(&AES_CRYP_KeyInitStructure);

    /* Enable Crypto processor */
    CRYP_Cmd(ENABLE);

    /* wait until the Busy flag is RESET */
    do
    {
      busystatus = CRYP_GetFlagStatus(CRYP_FLAG_BUSY);
      counter++;
    }while ((counter != AESBUSY_TIMEOUT) && (busystatus != RESET));

    if (busystatus != RESET)
   {
       status = ERROR;
    }
    else
    {
      /* Crypto Init for decryption process */  
      AES_CRYP_InitStructure.CRYP_AlgoDir = CRYP_AlgoDir_Decrypt;
    }
  }
  /*------------------ AES Encryption ------------------*/
  else /* AES encryption */
  {
    CRYP_KeyInit(&AES_CRYP_KeyInitStructure);

    /* Crypto Init for Encryption process */
    AES_CRYP_InitStructure.CRYP_AlgoDir  = CRYP_AlgoDir_Encrypt;
  }
  AES_CRYP_InitStructure.CRYP_AlgoMode = CRYP_AlgoMode_AES_CBC;
  AES_CRYP_InitStructure.CRYP_DataType = CRYP_DataType_8b;
  CRYP_Init(&AES_CRYP_InitStructure);

  /* CRYP Initialization Vectors */
  CRYP_IVInit(&AES_CRYP_IVInitStructure);

  /* Flush IN/OUT FIFOs */
  CRYP_FIFOFlush();

  /* Enable Crypto processor */
  CRYP_Cmd(ENABLE);

  if(CRYP_GetCmdStatus() == DISABLE)
  {
    /* The CRYP peripheral clock is not enabled or the device doesn't embedd 
       the CRYP peripheral (please check the device sales type. */
    return(ERROR);
  }
  
  for(i=0; ((i<Ilength) && (status != ERROR)); i+=16)
  {

    /* Write the Input block in the IN FIFO */
    CRYP_DataIn(*(uint32_t*)(inputaddr));
    inputaddr+=4;
    CRYP_DataIn(*(uint32_t*)(inputaddr));
    inputaddr+=4;
    CRYP_DataIn(*(uint32_t*)(inputaddr));
    inputaddr+=4;
    CRYP_DataIn(*(uint32_t*)(inputaddr));
    inputaddr+=4;
    /* Wait until the complete message has been processed */
    counter = 0;
    do
    {
      busystatus = CRYP_GetFlagStatus(CRYP_FLAG_BUSY);
      counter++;
    }while ((counter != AESBUSY_TIMEOUT) && (busystatus != RESET));

    if (busystatus != RESET)
   {
       status = ERROR;
    }
    else
    {

      /* Read the Output block from the Output FIFO */
      *(uint32_t*)(outputaddr) = CRYP_DataOut();
      outputaddr+=4;
      *(uint32_t*)(outputaddr) = CRYP_DataOut();
      outputaddr+=4;
      *(uint32_t*)(outputaddr) = CRYP_DataOut();
      outputaddr+=4;
      *(uint32_t*)(outputaddr) = CRYP_DataOut();
      outputaddr+=4;
    }
  }

  /* Disable Crypto */
  CRYP_Cmd(DISABLE);

  return status;
}

/**
  * @brief  Encrypt and decrypt using AES in CTR Mode
  * @param  Mode: encryption or decryption Mode.
  *           This parameter can be one of the following values:
  *            @arg MODE_ENCRYPT: Encryption
  *            @arg MODE_DECRYPT: Decryption
  * @param  InitVectors: Initialisation Vectors used for AES algorithm.
  * @param  Key: Key used for AES algorithm.
  * @param  Keysize: length of the Key, must be a 128, 192 or 256.
  * @param  Input: pointer to the Input buffer.
  * @param  Ilength: length of the Input buffer, must be a multiple of 16.
  * @param  Output: pointer to the returned buffer.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: Operation done
  *          - ERROR: Operation failed
  */
ErrorStatus CRYP_AES_CTR(uint8_t Mode, uint8_t InitVectors[16], uint8_t *Key, 
                         uint16_t Keysize, uint8_t *Input, uint32_t Ilength,
                         uint8_t *Output)
{
  CRYP_InitTypeDef AES_CRYP_InitStructure;
  CRYP_KeyInitTypeDef AES_CRYP_KeyInitStructure;
  CRYP_IVInitTypeDef AES_CRYP_IVInitStructure;
  __IO uint32_t counter = 0;
  uint32_t busystatus = 0;
  ErrorStatus status = SUCCESS;
  uint32_t keyaddr    = (uint32_t)Key;
  uint32_t inputaddr  = (uint32_t)Input;
  uint32_t outputaddr = (uint32_t)Output;
  uint32_t ivaddr     = (uint32_t)InitVectors;
  uint32_t i = 0;

  /* Crypto structures initialisation*/
  CRYP_KeyStructInit(&AES_CRYP_KeyInitStructure);

  switch(Keysize)
  {
    case 128:
    AES_CRYP_InitStructure.CRYP_KeySize = CRYP_KeySize_128b;
    AES_CRYP_KeyInitStructure.CRYP_Key2Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key2Right= __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key3Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key3Right= __REV(*(uint32_t*)(keyaddr));
    break;
    case 192:
    AES_CRYP_InitStructure.CRYP_KeySize  = CRYP_KeySize_192b;
    AES_CRYP_KeyInitStructure.CRYP_Key1Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key1Right= __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key2Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key2Right= __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key3Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key3Right= __REV(*(uint32_t*)(keyaddr));
    break;
    case 256:
    AES_CRYP_InitStructure.CRYP_KeySize  = CRYP_KeySize_256b;
    AES_CRYP_KeyInitStructure.CRYP_Key0Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key0Right= __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key1Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key1Right= __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key2Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key2Right= __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key3Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key3Right= __REV(*(uint32_t*)(keyaddr));
    break;
    default:
    break;
  }
  /* CRYP Initialization Vectors */
  AES_CRYP_IVInitStructure.CRYP_IV0Left = __REV(*(uint32_t*)(ivaddr));
  ivaddr+=4;
  AES_CRYP_IVInitStructure.CRYP_IV0Right= __REV(*(uint32_t*)(ivaddr));
  ivaddr+=4;
  AES_CRYP_IVInitStructure.CRYP_IV1Left = __REV(*(uint32_t*)(ivaddr));
  ivaddr+=4;
  AES_CRYP_IVInitStructure.CRYP_IV1Right= __REV(*(uint32_t*)(ivaddr));

  /* Key Initialisation */
  CRYP_KeyInit(&AES_CRYP_KeyInitStructure);

  /*------------------ AES Decryption ------------------*/
  if(Mode == MODE_DECRYPT) /* AES decryption */
  {
    /* Crypto Init for decryption process */
    AES_CRYP_InitStructure.CRYP_AlgoDir = CRYP_AlgoDir_Decrypt;
  }
  /*------------------ AES Encryption ------------------*/
  else /* AES encryption */
  {
    /* Crypto Init for Encryption process */
    AES_CRYP_InitStructure.CRYP_AlgoDir = CRYP_AlgoDir_Encrypt;
  }
  AES_CRYP_InitStructure.CRYP_AlgoMode = CRYP_AlgoMode_AES_CTR;
  AES_CRYP_InitStructure.CRYP_DataType = CRYP_DataType_8b;
  CRYP_Init(&AES_CRYP_InitStructure);

  /* CRYP Initialization Vectors */
  CRYP_IVInit(&AES_CRYP_IVInitStructure);

  /* Flush IN/OUT FIFOs */
  CRYP_FIFOFlush();

  /* Enable Crypto processor */
  CRYP_Cmd(ENABLE);

  if(CRYP_GetCmdStatus() == DISABLE)
  {
    /* The CRYP peripheral clock is not enabled or the device doesn't embedd 
       the CRYP peripheral (please check the device sales type. */
    return(ERROR);
  }
  
  for(i=0; ((i<Ilength) && (status != ERROR)); i+=16)
  {

    /* Write the Input block in the IN FIFO */
    CRYP_DataIn(*(uint32_t*)(inputaddr));
    inputaddr+=4;
    CRYP_DataIn(*(uint32_t*)(inputaddr));
    inputaddr+=4;
    CRYP_DataIn(*(uint32_t*)(inputaddr));
    inputaddr+=4;
    CRYP_DataIn(*(uint32_t*)(inputaddr));
    inputaddr+=4;
    /* Wait until the complete message has been processed */
    counter = 0;
    do
    {
      busystatus = CRYP_GetFlagStatus(CRYP_FLAG_BUSY);
      counter++;
    }while ((counter != AESBUSY_TIMEOUT) && (busystatus != RESET));

    if (busystatus != RESET)
   {
       status = ERROR;
    }
    else
    {

      /* Read the Output block from the Output FIFO */
      *(uint32_t*)(outputaddr) = CRYP_DataOut();
      outputaddr+=4;
      *(uint32_t*)(outputaddr) = CRYP_DataOut();
      outputaddr+=4;
      *(uint32_t*)(outputaddr) = CRYP_DataOut();
      outputaddr+=4;
      *(uint32_t*)(outputaddr) = CRYP_DataOut();
      outputaddr+=4;
    }
  }
  /* Disable Crypto */
  CRYP_Cmd(DISABLE);

  return status;
}

/**
  * @brief  Encrypt and decrypt using AES in GCM Mode. The GCM and CCM modes
  *         are available only on STM32F437x Devices.
  * @param  Mode: encryption or decryption Mode.
  *          This parameter can be one of the following values:
  *            @arg MODE_ENCRYPT: Encryption
  *            @arg MODE_DECRYPT: Decryption
  * @param  InitVectors: Initialisation Vectors used for AES algorithm.
  * @param  Key: Key used for AES algorithm.
  * @param  Keysize: length of the Key, must be a 128, 192 or 256.
  * @param  Input: pointer to the Input buffer.
  * @param  Ilength: length of the Input buffer in bytes, must be a multiple of 16.
  * @param  Header: pointer to the header buffer.
  * @param  Hlength: length of the header buffer in bytes, must be a multiple of 16.  
  * @param  Output: pointer to the returned buffer.
  * @param  AuthTAG: pointer to the authentication TAG buffer.
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: Operation done
  *          - ERROR: Operation failed
  */
ErrorStatus CRYP_AES_GCM(uint8_t Mode, uint8_t InitVectors[16],
                         uint8_t *Key, uint16_t Keysize,
                         uint8_t *Input, uint32_t ILength,
                         uint8_t *Header, uint32_t HLength,
                         uint8_t *Output, uint8_t *AuthTAG)
{
  CRYP_InitTypeDef AES_CRYP_InitStructure;
  CRYP_KeyInitTypeDef AES_CRYP_KeyInitStructure;
  CRYP_IVInitTypeDef AES_CRYP_IVInitStructure;
  __IO uint32_t counter = 0;
  uint32_t busystatus = 0;
  ErrorStatus status = SUCCESS;
  uint32_t keyaddr    = (uint32_t)Key;
  uint32_t inputaddr  = (uint32_t)Input;
  uint32_t outputaddr = (uint32_t)Output;
  uint32_t ivaddr     = (uint32_t)InitVectors;
  uint32_t headeraddr = (uint32_t)Header;
  uint32_t tagaddr = (uint32_t)AuthTAG;
  uint64_t headerlength = HLength * 8;/* header length in bits */
  uint64_t inputlength = ILength * 8;/* input length in bits */
  uint32_t loopcounter = 0;

  /* Crypto structures initialisation*/
  CRYP_KeyStructInit(&AES_CRYP_KeyInitStructure);

  switch(Keysize)
  {
    case 128:
    AES_CRYP_InitStructure.CRYP_KeySize = CRYP_KeySize_128b;
    AES_CRYP_KeyInitStructure.CRYP_Key2Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key2Right= __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key3Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key3Right= __REV(*(uint32_t*)(keyaddr));
    break;
    case 192:
    AES_CRYP_InitStructure.CRYP_KeySize  = CRYP_KeySize_192b;
    AES_CRYP_KeyInitStructure.CRYP_Key1Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key1Right= __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key2Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key2Right= __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key3Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key3Right= __REV(*(uint32_t*)(keyaddr));
    break;
    case 256:
    AES_CRYP_InitStructure.CRYP_KeySize  = CRYP_KeySize_256b;
    AES_CRYP_KeyInitStructure.CRYP_Key0Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key0Right= __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key1Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key1Right= __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key2Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key2Right= __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key3Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key3Right= __REV(*(uint32_t*)(keyaddr));
    break;
    default:
    break;
  }
  
  /* CRYP Initialization Vectors */
  AES_CRYP_IVInitStructure.CRYP_IV0Left = __REV(*(uint32_t*)(ivaddr));
  ivaddr+=4;
  AES_CRYP_IVInitStructure.CRYP_IV0Right= __REV(*(uint32_t*)(ivaddr));
  ivaddr+=4;
  AES_CRYP_IVInitStructure.CRYP_IV1Left = __REV(*(uint32_t*)(ivaddr));
  ivaddr+=4;
  AES_CRYP_IVInitStructure.CRYP_IV1Right= __REV(*(uint32_t*)(ivaddr));
  
  /*------------------ AES Encryption ------------------*/
  if(Mode == MODE_ENCRYPT) /* AES encryption */
  {
    /* Flush IN/OUT FIFOs */
    CRYP_FIFOFlush();
    
    /* Key Initialisation */
    CRYP_KeyInit(&AES_CRYP_KeyInitStructure);
    
    /* CRYP Initialization Vectors */
    CRYP_IVInit(&AES_CRYP_IVInitStructure);
    
    /* Crypto Init for Key preparation for decryption process */
    AES_CRYP_InitStructure.CRYP_AlgoDir = CRYP_AlgoDir_Encrypt;
    AES_CRYP_InitStructure.CRYP_AlgoMode = CRYP_AlgoMode_AES_GCM;
    AES_CRYP_InitStructure.CRYP_DataType = CRYP_DataType_8b;
    CRYP_Init(&AES_CRYP_InitStructure);
    
    /***************************** Init phase *********************************/
    /* Select init phase */
    CRYP_PhaseConfig(CRYP_Phase_Init);
    
    /* Enable Crypto processor */
    CRYP_Cmd(ENABLE);
    
    /* Wait for CRYPEN bit to be 0 */
    while(CRYP_GetCmdStatus() == ENABLE)
    {
    }
    
    /***************************** header phase *******************************/
    if(HLength != 0)
    {
      /* Select header phase */
      CRYP_PhaseConfig(CRYP_Phase_Header);
      
      /* Enable Crypto processor */
      CRYP_Cmd(ENABLE);
      
      if(CRYP_GetCmdStatus() == DISABLE)
      {
         /* The CRYP peripheral clock is not enabled or the device doesn't embedd 
            the CRYP peripheral (please check the device sales type. */
         return(ERROR);
      }
      
      for(loopcounter = 0; (loopcounter < HLength); loopcounter+=16)
      {
        /* Wait until the IFEM flag is reset */
        while(CRYP_GetFlagStatus(CRYP_FLAG_IFEM) == RESET)
        {
        }
        
        /* Write the Input block in the IN FIFO */
        CRYP_DataIn(*(uint32_t*)(headeraddr));
        headeraddr+=4;
        CRYP_DataIn(*(uint32_t*)(headeraddr));
        headeraddr+=4;
        CRYP_DataIn(*(uint32_t*)(headeraddr));
        headeraddr+=4;
        CRYP_DataIn(*(uint32_t*)(headeraddr));
        headeraddr+=4;
      }
      
      /* Wait until the complete message has been processed */
      counter = 0;
      do
      {
        busystatus = CRYP_GetFlagStatus(CRYP_FLAG_BUSY);
        counter++;
      }while ((counter != AESBUSY_TIMEOUT) && (busystatus != RESET));

      if (busystatus != RESET)
      {
        status = ERROR;
      }
    }
    
    /**************************** payload phase *******************************/
    if(ILength != 0)
    {
      /* Select payload phase */
      CRYP_PhaseConfig(CRYP_Phase_Payload);
      
      /* Enable Crypto processor */
      CRYP_Cmd(ENABLE);
      
      if(CRYP_GetCmdStatus() == DISABLE)
      {
        /* The CRYP peripheral clock is not enabled or the device doesn't embedd 
           the CRYP peripheral (please check the device sales type. */
        return(ERROR);
      }
      
      for(loopcounter = 0; ((loopcounter < ILength) && (status != ERROR)); loopcounter+=16)
      {
        /* Wait until the IFEM flag is reset */
        while(CRYP_GetFlagStatus(CRYP_FLAG_IFEM) == RESET)
        {
        }
        /* Write the Input block in the IN FIFO */
        CRYP_DataIn(*(uint32_t*)(inputaddr));
        inputaddr+=4;
        CRYP_DataIn(*(uint32_t*)(inputaddr));
        inputaddr+=4;
        CRYP_DataIn(*(uint32_t*)(inputaddr));
        inputaddr+=4;
        CRYP_DataIn(*(uint32_t*)(inputaddr));
        inputaddr+=4;
        
        /* Wait until the complete message has been processed */
        counter = 0;
        do
        {
          busystatus = CRYP_GetFlagStatus(CRYP_FLAG_BUSY);
          counter++;
        }while ((counter != AESBUSY_TIMEOUT) && (busystatus != RESET));

        if (busystatus != RESET)
        {
          status = ERROR;
        }
        else
        {
          /* Wait until the OFNE flag is reset */
          while(CRYP_GetFlagStatus(CRYP_FLAG_OFNE) == RESET)
          {
          }
          
          /* Read the Output block from the Output FIFO */
          *(uint32_t*)(outputaddr) = CRYP_DataOut();
          outputaddr+=4;
          *(uint32_t*)(outputaddr) = CRYP_DataOut();
          outputaddr+=4;
          *(uint32_t*)(outputaddr) = CRYP_DataOut();
          outputaddr+=4;
          *(uint32_t*)(outputaddr) = CRYP_DataOut();
          outputaddr+=4;
        }
      }
    }
    
    /***************************** final phase ********************************/
    /* Select final phase */
    CRYP_PhaseConfig(CRYP_Phase_Final);
    
    /* Enable Crypto processor */
    CRYP_Cmd(ENABLE);
    
    if(CRYP_GetCmdStatus() == DISABLE)
    {
      /* The CRYP peripheral clock is not enabled or the device doesn't embedd 
         the CRYP peripheral (please check the device sales type. */
      return(ERROR);
    }
    
    /* Write number of bits concatenated with header in the IN FIFO */
    CRYP_DataIn(__REV(headerlength>>32));
    CRYP_DataIn(__REV(headerlength));
    CRYP_DataIn(__REV(inputlength>>32));
    CRYP_DataIn(__REV(inputlength));
    /* Wait until the OFNE flag is reset */
    while(CRYP_GetFlagStatus(CRYP_FLAG_OFNE) == RESET)
    {
    }
    
    tagaddr = (uint32_t)AuthTAG;
    /* Read the Auth TAG in the IN FIFO */
    *(uint32_t*)(tagaddr) = CRYP_DataOut();
    tagaddr+=4;
    *(uint32_t*)(tagaddr) = CRYP_DataOut();
    tagaddr+=4;
    *(uint32_t*)(tagaddr) = CRYP_DataOut();
    tagaddr+=4;
    *(uint32_t*)(tagaddr) = CRYP_DataOut();
    tagaddr+=4;
  }
  /*------------------ AES Decryption ------------------*/
  else /* AES decryption */
  {
    /* Flush IN/OUT FIFOs */
    CRYP_FIFOFlush();
    
    /* Key Initialisation */
    CRYP_KeyInit(&AES_CRYP_KeyInitStructure);
    
    /* CRYP Initialization Vectors */
    CRYP_IVInit(&AES_CRYP_IVInitStructure);
    
    /* Crypto Init for Key preparation for decryption process */
    AES_CRYP_InitStructure.CRYP_AlgoDir = CRYP_AlgoDir_Decrypt;
    AES_CRYP_InitStructure.CRYP_AlgoMode = CRYP_AlgoMode_AES_GCM;
    AES_CRYP_InitStructure.CRYP_DataType = CRYP_DataType_8b;
    CRYP_Init(&AES_CRYP_InitStructure);
    
    /***************************** Init phase *********************************/
    /* Select init phase */
    CRYP_PhaseConfig(CRYP_Phase_Init);
    
    /* Enable Crypto processor */
    CRYP_Cmd(ENABLE);
    
    /* Wait for CRYPEN bit to be 0 */
    while(CRYP_GetCmdStatus() == ENABLE);
    
    /***************************** header phase *******************************/
    if(HLength != 0)
    {
      /* Select header phase */
      CRYP_PhaseConfig(CRYP_Phase_Header);
      
      /* Enable Crypto processor */
      CRYP_Cmd(ENABLE);
      
      if(CRYP_GetCmdStatus() == DISABLE)
      {
        /* The CRYP peripheral clock is not enabled or the device doesn't embedd 
           the CRYP peripheral (please check the device sales type. */
        return(ERROR);
      }
      
      for(loopcounter = 0; (loopcounter < HLength); loopcounter+=16)
      {
        /* Wait until the IFEM flag is reset */
        while(CRYP_GetFlagStatus(CRYP_FLAG_IFEM) == RESET);
        
        /* Write the Input block in the IN FIFO */
        CRYP_DataIn(*(uint32_t*)(headeraddr));
        headeraddr+=4;
        CRYP_DataIn(*(uint32_t*)(headeraddr));
        headeraddr+=4;
        CRYP_DataIn(*(uint32_t*)(headeraddr));
        headeraddr+=4;
        CRYP_DataIn(*(uint32_t*)(headeraddr));
        headeraddr+=4;
      }
      
      /* Wait until the complete message has been processed */
      counter = 0;
      do
      {
        busystatus = CRYP_GetFlagStatus(CRYP_FLAG_BUSY);
        counter++;
      }while ((counter != AESBUSY_TIMEOUT) && (busystatus != RESET));

      if (busystatus != RESET)
      {
        status = ERROR;
      }
    }
    
    /**************************** payload phase *******************************/
    if(ILength != 0)
    {
      /* Select payload phase */
      CRYP_PhaseConfig(CRYP_Phase_Payload);

      /* Enable Crypto processor */
      CRYP_Cmd(ENABLE);
      
      if(CRYP_GetCmdStatus() == DISABLE)
      {
        /* The CRYP peripheral clock is not enabled or the device doesn't embedd 
           the CRYP peripheral (please check the device sales type. */
        return(ERROR);
      }
      
      for(loopcounter = 0; ((loopcounter < ILength) && (status != ERROR)); loopcounter+=16)
      {
        /* Wait until the IFEM flag is reset */
        while(CRYP_GetFlagStatus(CRYP_FLAG_IFEM) == RESET);
        /* Write the Input block in the IN FIFO */
        CRYP_DataIn(*(uint32_t*)(inputaddr));
        inputaddr+=4;
        CRYP_DataIn(*(uint32_t*)(inputaddr));
        inputaddr+=4;
        CRYP_DataIn(*(uint32_t*)(inputaddr));
        inputaddr+=4;
        CRYP_DataIn(*(uint32_t*)(inputaddr));
        inputaddr+=4;
        
        /* Wait until the complete message has been processed */
        counter = 0;
        do
        {
          busystatus = CRYP_GetFlagStatus(CRYP_FLAG_BUSY);
          counter++;
        }while ((counter != AESBUSY_TIMEOUT) && (busystatus != RESET));

        if (busystatus != RESET)
        {
          status = ERROR;
        }
        else
        {
          /* Wait until the OFNE flag is reset */
          while(CRYP_GetFlagStatus(CRYP_FLAG_OFNE) == RESET);
          
          /* Read the Output block from the Output FIFO */
          *(uint32_t*)(outputaddr) = CRYP_DataOut();
          outputaddr+=4;
          *(uint32_t*)(outputaddr) = CRYP_DataOut();
          outputaddr+=4;
          *(uint32_t*)(outputaddr) = CRYP_DataOut();
          outputaddr+=4;
          *(uint32_t*)(outputaddr) = CRYP_DataOut();
          outputaddr+=4;
        }
      }
    }
    
    /***************************** final phase ********************************/
    /* Select final phase */
    CRYP_PhaseConfig(CRYP_Phase_Final);

    /* Enable Crypto processor */
    CRYP_Cmd(ENABLE);
    
    if(CRYP_GetCmdStatus() == DISABLE)
    {
      /* The CRYP peripheral clock is not enabled or the device doesn't embedd 
         the CRYP peripheral (please check the device sales type. */
      return(ERROR);
    }
    
    /* Write number of bits concatenated with header in the IN FIFO */
    CRYP_DataIn(__REV(headerlength>>32));
    CRYP_DataIn(__REV(headerlength));
    CRYP_DataIn(__REV(inputlength>>32));
    CRYP_DataIn(__REV(inputlength));
    /* Wait until the OFNE flag is reset */
    while(CRYP_GetFlagStatus(CRYP_FLAG_OFNE) == RESET);
    
    tagaddr = (uint32_t)AuthTAG;
    /* Read the Auth TAG in the IN FIFO */
    *(uint32_t*)(tagaddr) = CRYP_DataOut();
    tagaddr+=4;
    *(uint32_t*)(tagaddr) = CRYP_DataOut();
    tagaddr+=4;
    *(uint32_t*)(tagaddr) = CRYP_DataOut();
    tagaddr+=4;
    *(uint32_t*)(tagaddr) = CRYP_DataOut();
    tagaddr+=4;
  }
  /* Disable Crypto */
  CRYP_Cmd(DISABLE);

  return status;
}

/**
  * @brief  Encrypt and decrypt using AES in CCM Mode. The GCM and CCM modes
  *         are available only on STM32F437x Devices.
  * @param  Mode: encryption or decryption Mode.
  *          This parameter can be one of the following values:
  *            @arg MODE_ENCRYPT: Encryption
  *            @arg MODE_DECRYPT: Decryption
  * @param  Nonce: the nounce used for AES algorithm. It shall be unique for each processing.
  * @param  Key: Key used for AES algorithm.
  * @param  Keysize: length of the Key, must be a 128, 192 or 256.
  * @param  Input: pointer to the Input buffer.
  * @param  Ilength: length of the Input buffer in bytes, must be a multiple of 16.
  * @param  Header: pointer to the header buffer.
  * @param  Hlength: length of the header buffer in bytes.
  * @param  HBuffer: pointer to temporary buffer used to append the header
  *         HBuffer size must be equal to Hlength + 21
  * @param  Output: pointer to the returned buffer.
  * @param  AuthTAG: pointer to the authentication TAG buffer.
  * @param  TAGSize: the size of the TAG (called also MAC).
  * @retval An ErrorStatus enumeration value:
  *          - SUCCESS: Operation done
  *          - ERROR: Operation failed
  */
ErrorStatus CRYP_AES_CCM(uint8_t Mode, 
                         uint8_t* Nonce, uint32_t NonceSize,
                         uint8_t *Key, uint16_t Keysize,
                         uint8_t *Input, uint32_t ILength,
                         uint8_t *Header, uint32_t HLength, uint8_t *HBuffer,
                         uint8_t *Output,
                         uint8_t *AuthTAG, uint32_t TAGSize)
{
  CRYP_InitTypeDef AES_CRYP_InitStructure;
  CRYP_KeyInitTypeDef AES_CRYP_KeyInitStructure;
  CRYP_IVInitTypeDef AES_CRYP_IVInitStructure;
  __IO uint32_t counter = 0;
  uint32_t busystatus = 0;
  ErrorStatus status = SUCCESS;
  uint32_t keyaddr    = (uint32_t)Key;
  uint32_t inputaddr  = (uint32_t)Input;
  uint32_t outputaddr = (uint32_t)Output;
  uint32_t headeraddr = (uint32_t)Header;
  uint32_t tagaddr = (uint32_t)AuthTAG;
  uint32_t headersize = HLength;
  uint32_t loopcounter = 0;
  uint32_t bufferidx = 0;
  uint8_t blockb0[16] = {0};/* Block B0 */
  uint8_t ctr[16] = {0}; /* Counter */
  uint32_t temptag[4] = {0}; /* temporary TAG (MAC) */
  uint32_t ctraddr = (uint32_t)ctr;
  uint32_t b0addr = (uint32_t)blockb0;
  
  /************************ Formatting the header block ***********************/
  if(headersize != 0)
  {
    /* Check that the associated data (or header) length is lower than 2^16 - 2^8 = 65536 - 256 = 65280 */
    if(headersize < 65280)
    {
      HBuffer[bufferidx++] = (uint8_t) ((headersize >> 8) & 0xFF);
      HBuffer[bufferidx++] = (uint8_t) ((headersize) & 0xFF);
      headersize += 2;
    }
    else
    {
      /* header is encoded as 0xff || 0xfe || [headersize]32, i.e., six octets */
      HBuffer[bufferidx++] = 0xFF;
      HBuffer[bufferidx++] = 0xFE;
      HBuffer[bufferidx++] = headersize & 0xff000000;
      HBuffer[bufferidx++] = headersize & 0x00ff0000;
      HBuffer[bufferidx++] = headersize & 0x0000ff00;
      HBuffer[bufferidx++] = headersize & 0x000000ff;
      headersize += 6;
    }
    /* Copy the header buffer in internal buffer "HBuffer" */
    for(loopcounter = 0; loopcounter < headersize; loopcounter++)
    {
      HBuffer[bufferidx++] = Header[loopcounter];
    }
    /* Check if the header size is modulo 16 */
    if ((headersize % 16) != 0)
    {
      /* Padd the header buffer with 0s till the HBuffer length is modulo 16 */
      for(loopcounter = headersize; loopcounter <= ((headersize/16) + 1) * 16; loopcounter++)
      {
        HBuffer[loopcounter] = 0;
      }
      /* Set the header size to modulo 16 */
      headersize = ((headersize/16) + 1) * 16;
    }
    /* set the pointer headeraddr to HBuffer */
    headeraddr = (uint32_t)HBuffer;
  }
  /************************* Formatting the block B0 **************************/
  if(headersize != 0)
  {
    blockb0[0] = 0x40;
  }
  /* Flags byte */
  blockb0[0] |= 0u | (((( (uint8_t) TAGSize - 2) / 2) & 0x07 ) << 3 ) | ( ( (uint8_t) (15 - NonceSize) - 1) & 0x07);
  
  for (loopcounter = 0; loopcounter < NonceSize; loopcounter++)
  {
    blockb0[loopcounter+1] = Nonce[loopcounter];
  }
  for ( ; loopcounter < 13; loopcounter++)
  {
    blockb0[loopcounter+1] = 0;
  }
  
  blockb0[14] = ((ILength >> 8) & 0xFF);
  blockb0[15] = (ILength & 0xFF);
  
  /************************* Formatting the initial counter *******************/
  /* Byte 0:
     Bits 7 and 6 are reserved and shall be set to 0
     Bits 3, 4, and 5 shall also be set to 0, to ensure that all the counter blocks
     are distinct from B0
     Bits 0, 1, and 2 contain the same encoding of q as in B0
  */
  ctr[0] = blockb0[0] & 0x07;
  /* byte 1 to NonceSize is the IV (Nonce) */
  for(loopcounter = 1; loopcounter < NonceSize + 1; loopcounter++)
  {
    ctr[loopcounter] = blockb0[loopcounter];
  }
  /* Set the LSB to 1 */
  ctr[15] |= 0x01;
  
  /* Crypto structures initialisation*/
  CRYP_KeyStructInit(&AES_CRYP_KeyInitStructure);
  
  switch(Keysize)
  {
    case 128:
    AES_CRYP_InitStructure.CRYP_KeySize = CRYP_KeySize_128b;
    AES_CRYP_KeyInitStructure.CRYP_Key2Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key2Right= __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key3Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key3Right= __REV(*(uint32_t*)(keyaddr));
    break;
    case 192:
    AES_CRYP_InitStructure.CRYP_KeySize  = CRYP_KeySize_192b;
    AES_CRYP_KeyInitStructure.CRYP_Key1Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key1Right= __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key2Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key2Right= __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key3Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key3Right= __REV(*(uint32_t*)(keyaddr));
    break;
    case 256:
    AES_CRYP_InitStructure.CRYP_KeySize  = CRYP_KeySize_256b;
    AES_CRYP_KeyInitStructure.CRYP_Key0Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key0Right= __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key1Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key1Right= __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key2Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key2Right= __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key3Left = __REV(*(uint32_t*)(keyaddr));
    keyaddr+=4;
    AES_CRYP_KeyInitStructure.CRYP_Key3Right= __REV(*(uint32_t*)(keyaddr));
    break;
    default:
    break;
  }
  
  /* CRYP Initialization Vectors */
  AES_CRYP_IVInitStructure.CRYP_IV0Left = (__REV(*(uint32_t*)(ctraddr)));
  ctraddr+=4;
  AES_CRYP_IVInitStructure.CRYP_IV0Right= (__REV(*(uint32_t*)(ctraddr)));
  ctraddr+=4;
  AES_CRYP_IVInitStructure.CRYP_IV1Left = (__REV(*(uint32_t*)(ctraddr)));
  ctraddr+=4;
  AES_CRYP_IVInitStructure.CRYP_IV1Right= (__REV(*(uint32_t*)(ctraddr)));
  
  /*------------------ AES Encryption ------------------*/
  if(Mode == MODE_ENCRYPT) /* AES encryption */
  {
    /* Flush IN/OUT FIFOs */
    CRYP_FIFOFlush();
    
    /* Key Initialisation */
    CRYP_KeyInit(&AES_CRYP_KeyInitStructure);
    
    /* CRYP Initialization Vectors */
    CRYP_IVInit(&AES_CRYP_IVInitStructure);
    
    /* Crypto Init for Key preparation for decryption process */
    AES_CRYP_InitStructure.CRYP_AlgoDir = CRYP_AlgoDir_Encrypt;
    AES_CRYP_InitStructure.CRYP_AlgoMode = CRYP_AlgoMode_AES_CCM;
    AES_CRYP_InitStructure.CRYP_DataType = CRYP_DataType_8b;
    CRYP_Init(&AES_CRYP_InitStructure);
    
    /***************************** Init phase *********************************/
    /* Select init phase */
    CRYP_PhaseConfig(CRYP_Phase_Init);
    
    b0addr = (uint32_t)blockb0;
    /* Write the blockb0 block in the IN FIFO */
    CRYP_DataIn((*(uint32_t*)(b0addr)));
    b0addr+=4;
    CRYP_DataIn((*(uint32_t*)(b0addr)));
    b0addr+=4;
    CRYP_DataIn((*(uint32_t*)(b0addr)));
    b0addr+=4;
    CRYP_DataIn((*(uint32_t*)(b0addr)));
    
    /* Enable Crypto processor */
    CRYP_Cmd(ENABLE);
    
    /* Wait for CRYPEN bit to be 0 */
    while(CRYP_GetCmdStatus() == ENABLE);
    
    /***************************** header phase *******************************/
    if(headersize != 0)
    {
      /* Select header phase */
      CRYP_PhaseConfig(CRYP_Phase_Header);
      
      /* Enable Crypto processor */
      CRYP_Cmd(ENABLE);
      
      if(CRYP_GetCmdStatus() == DISABLE)
      {
         /* The CRYP peripheral clock is not enabled or the device doesn't embedd 
            the CRYP peripheral (please check the device sales type. */
         return(ERROR);
      }
      
      for(loopcounter = 0; (loopcounter < headersize); loopcounter+=16)
      {
        /* Wait until the IFEM flag is reset */
        while(CRYP_GetFlagStatus(CRYP_FLAG_IFEM) == RESET);
        
        /* Write the Input block in the IN FIFO */
        CRYP_DataIn(*(uint32_t*)(headeraddr));
        headeraddr+=4;
        CRYP_DataIn(*(uint32_t*)(headeraddr));
        headeraddr+=4;
        CRYP_DataIn(*(uint32_t*)(headeraddr));
        headeraddr+=4;
        CRYP_DataIn(*(uint32_t*)(headeraddr));
        headeraddr+=4;
      }
      
      /* Wait until the complete message has been processed */
      counter = 0;
      do
      {
        busystatus = CRYP_GetFlagStatus(CRYP_FLAG_BUSY);
        counter++;
      }while ((counter != AESBUSY_TIMEOUT) && (busystatus != RESET));

      if (busystatus != RESET)
      {
        status = ERROR;
      }
    }
    
    /**************************** payload phase *******************************/
    if(ILength != 0)
    {
      /* Select payload phase */
      CRYP_PhaseConfig(CRYP_Phase_Payload);
      
      /* Enable Crypto processor */
      CRYP_Cmd(ENABLE);
      
      if(CRYP_GetCmdStatus() == DISABLE)
      {
        /* The CRYP peripheral clock is not enabled or the device doesn't embedd 
           the CRYP peripheral (please check the device sales type. */
        return(ERROR);
      }
      
      for(loopcounter = 0; ((loopcounter < ILength) && (status != ERROR)); loopcounter+=16)
      {
        /* Wait until the IFEM flag is reset */
        while(CRYP_GetFlagStatus(CRYP_FLAG_IFEM) == RESET);
        /* Write the Input block in the IN FIFO */
        CRYP_DataIn(*(uint32_t*)(inputaddr));
        inputaddr+=4;
        CRYP_DataIn(*(uint32_t*)(inputaddr));
        inputaddr+=4;
        CRYP_DataIn(*(uint32_t*)(inputaddr));
        inputaddr+=4;
        CRYP_DataIn(*(uint32_t*)(inputaddr));
        inputaddr+=4;
        
        /* Wait until the complete message has been processed */
        counter = 0;
        do
        {
          busystatus = CRYP_GetFlagStatus(CRYP_FLAG_BUSY);
          counter++;
        }while ((counter != AESBUSY_TIMEOUT) && (busystatus != RESET));

        if (busystatus != RESET)
        {
          status = ERROR;
        }
        else
        {
          /* Wait until the OFNE flag is reset */
          while(CRYP_GetFlagStatus(CRYP_FLAG_OFNE) == RESET);
          
          /* Read the Output block from the Output FIFO */
          *(uint32_t*)(outputaddr) = CRYP_DataOut();
          outputaddr+=4;
          *(uint32_t*)(outputaddr) = CRYP_DataOut();
          outputaddr+=4;
          *(uint32_t*)(outputaddr) = CRYP_DataOut();
          outputaddr+=4;
          *(uint32_t*)(outputaddr) = CRYP_DataOut();
          outputaddr+=4;
        }
      }
    }
    
    /***************************** final phase ********************************/
    /* Select final phase */
    CRYP_PhaseConfig(CRYP_Phase_Final);
    
    /* Enable Crypto processor */
    CRYP_Cmd(ENABLE);
    
    if(CRYP_GetCmdStatus() == DISABLE)
    {
      /* The CRYP peripheral clock is not enabled or the device doesn't embedd 
         the CRYP peripheral (please check the device sales type. */
      return(ERROR);
    }
    
    ctraddr = (uint32_t)ctr;
    /* Write the counter block in the IN FIFO */
    CRYP_DataIn(*(uint32_t*)(ctraddr));
    ctraddr+=4;
    CRYP_DataIn(*(uint32_t*)(ctraddr));
    ctraddr+=4;
    CRYP_DataIn(*(uint32_t*)(ctraddr));
    ctraddr+=4;
    /* Reset bit 0 (after 8-bit swap) is equivalent to reset bit 24 (before 8-bit swap) */
    CRYP_DataIn(*(uint32_t*)(ctraddr) & 0xfeffffff);
    
    /* Wait until the OFNE flag is reset */
    while(CRYP_GetFlagStatus(CRYP_FLAG_OFNE) == RESET);
    
    /* Read the Auth TAG in the IN FIFO */
    temptag[0] = CRYP_DataOut();
    temptag[1] = CRYP_DataOut();
    temptag[2] = CRYP_DataOut();
    temptag[3] = CRYP_DataOut();
  }
  /*------------------ AES Decryption ------------------*/
  else /* AES decryption */
  {
    /* Flush IN/OUT FIFOs */
    CRYP_FIFOFlush();
    
    /* Key Initialisation */
    CRYP_KeyInit(&AES_CRYP_KeyInitStructure);
    
    /* CRYP Initialization Vectors */
    CRYP_IVInit(&AES_CRYP_IVInitStructure);
    
    /* Crypto Init for Key preparation for decryption process */
    AES_CRYP_InitStructure.CRYP_AlgoDir = CRYP_AlgoDir_Decrypt;
    AES_CRYP_InitStructure.CRYP_AlgoMode = CRYP_AlgoMode_AES_CCM;
    AES_CRYP_InitStructure.CRYP_DataType = CRYP_DataType_8b;
    CRYP_Init(&AES_CRYP_InitStructure);
    
    /***************************** Init phase *********************************/
    /* Select init phase */
    CRYP_PhaseConfig(CRYP_Phase_Init);
    
    b0addr = (uint32_t)blockb0;
    /* Write the blockb0 block in the IN FIFO */
    CRYP_DataIn((*(uint32_t*)(b0addr)));
    b0addr+=4;
    CRYP_DataIn((*(uint32_t*)(b0addr)));
    b0addr+=4;
    CRYP_DataIn((*(uint32_t*)(b0addr)));
    b0addr+=4;
    CRYP_DataIn((*(uint32_t*)(b0addr)));
    
    /* Enable Crypto processor */
    CRYP_Cmd(ENABLE);
    
    /* Wait for CRYPEN bit to be 0 */
    while(CRYP_GetCmdStatus() == ENABLE);
    
    /***************************** header phase *******************************/
    if(headersize != 0)
    {
      /* Select header phase */
      CRYP_PhaseConfig(CRYP_Phase_Header);
      
      /* Enable Crypto processor */
      CRYP_Cmd(ENABLE);
      
      if(CRYP_GetCmdStatus() == DISABLE)
      {
        /* The CRYP peripheral clock is not enabled or the device doesn't embedd 
           the CRYP peripheral (please check the device sales type. */
        return(ERROR);
      }
      
      for(loopcounter = 0; (loopcounter < headersize); loopcounter+=16)
      {
        /* Wait until the IFEM flag is reset */
        while(CRYP_GetFlagStatus(CRYP_FLAG_IFEM) == RESET);
        
        /* Write the Input block in the IN FIFO */
        CRYP_DataIn(*(uint32_t*)(headeraddr));
        headeraddr+=4;
        CRYP_DataIn(*(uint32_t*)(headeraddr));
        headeraddr+=4;
        CRYP_DataIn(*(uint32_t*)(headeraddr));
        headeraddr+=4;
        CRYP_DataIn(*(uint32_t*)(headeraddr));
        headeraddr+=4;
      }
      
      /* Wait until the complete message has been processed */
      counter = 0;
      do
      {
        busystatus = CRYP_GetFlagStatus(CRYP_FLAG_BUSY);
        counter++;
      }while ((counter != AESBUSY_TIMEOUT) && (busystatus != RESET));

      if (busystatus != RESET)
      {
        status = ERROR;
      }
    }
    
    /**************************** payload phase *******************************/
    if(ILength != 0)
    {
      /* Select payload phase */
      CRYP_PhaseConfig(CRYP_Phase_Payload);

      /* Enable Crypto processor */
      CRYP_Cmd(ENABLE);
      
      if(CRYP_GetCmdStatus() == DISABLE)
      {
        /* The CRYP peripheral clock is not enabled or the device doesn't embedd 
           the CRYP peripheral (please check the device sales type. */
        return(ERROR);
      }
      
      for(loopcounter = 0; ((loopcounter < ILength) && (status != ERROR)); loopcounter+=16)
      {
        /* Wait until the IFEM flag is reset */
        while(CRYP_GetFlagStatus(CRYP_FLAG_IFEM) == RESET);
        /* Write the Input block in the IN FIFO */
        CRYP_DataIn(*(uint32_t*)(inputaddr));
        inputaddr+=4;
        CRYP_DataIn(*(uint32_t*)(inputaddr));
        inputaddr+=4;
        CRYP_DataIn(*(uint32_t*)(inputaddr));
        inputaddr+=4;
        CRYP_DataIn(*(uint32_t*)(inputaddr));
        inputaddr+=4;
        
        /* Wait until the complete message has been processed */
        counter = 0;
        do
        {
          busystatus = CRYP_GetFlagStatus(CRYP_FLAG_BUSY);
          counter++;
        }while ((counter != AESBUSY_TIMEOUT) && (busystatus != RESET));

        if (busystatus != RESET)
        {
          status = ERROR;
        }
        else
        {
          /* Wait until the OFNE flag is reset */
          while(CRYP_GetFlagStatus(CRYP_FLAG_OFNE) == RESET);
          
          /* Read the Output block from the Output FIFO */
          *(uint32_t*)(outputaddr) = CRYP_DataOut();
          outputaddr+=4;
          *(uint32_t*)(outputaddr) = CRYP_DataOut();
          outputaddr+=4;
          *(uint32_t*)(outputaddr) = CRYP_DataOut();
          outputaddr+=4;
          *(uint32_t*)(outputaddr) = CRYP_DataOut();
          outputaddr+=4;
        }
      }
    }
    
    /***************************** final phase ********************************/
    /* Select final phase */
    CRYP_PhaseConfig(CRYP_Phase_Final);
    
    /* Enable Crypto processor */
    CRYP_Cmd(ENABLE);
    
    if(CRYP_GetCmdStatus() == DISABLE)
    {
      /* The CRYP peripheral clock is not enabled or the device doesn't embedd 
         the CRYP peripheral (please check the device sales type. */
      return(ERROR);
    }
    
    ctraddr = (uint32_t)ctr;
    /* Write the counter block in the IN FIFO */
    CRYP_DataIn(*(uint32_t*)(ctraddr));
    ctraddr+=4;
    CRYP_DataIn(*(uint32_t*)(ctraddr));
    ctraddr+=4;
    CRYP_DataIn(*(uint32_t*)(ctraddr));
    ctraddr+=4;
    /* Reset bit 0 (after 8-bit swap) is equivalent to reset bit 24 (before 8-bit swap) */
    CRYP_DataIn(*(uint32_t*)(ctraddr) & 0xfeffffff);
    
    /* Wait until the OFNE flag is reset */
    while(CRYP_GetFlagStatus(CRYP_FLAG_OFNE) == RESET);
    
    /* Read the Authentaication TAG (MAC) in the IN FIFO */
    temptag[0] = CRYP_DataOut();
    temptag[1] = CRYP_DataOut();
    temptag[2] = CRYP_DataOut();
    temptag[3] = CRYP_DataOut();
  }
  
  /* Copy temporary authentication TAG in user TAG buffer */
  for(loopcounter = 0; (loopcounter < TAGSize); loopcounter++)
  {
    /* Set the authentication TAG buffer */
    *((uint8_t*)tagaddr+loopcounter) = *((uint8_t*)temptag+loopcounter);
  }
  
  /* Disable Crypto */
  CRYP_Cmd(DISABLE);

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

