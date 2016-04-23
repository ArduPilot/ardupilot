/**
  ******************************************************************************
  * @file    stm32f4xx_hash.h
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    11-January-2013
  * @brief   This file contains all the functions prototypes for the HASH 
  *          firmware library.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __STM32F4xx_HASH_H
#define __STM32F4xx_HASH_H

#ifdef __cplusplus
 extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"

/** @addtogroup STM32F4xx_StdPeriph_Driver
  * @{
  */

/** @addtogroup HASH
  * @{
  */ 

/* Exported types ------------------------------------------------------------*/

/** 
  * @brief   HASH Init structure definition
  */ 
typedef struct
{
  uint32_t HASH_AlgoSelection; /*!< SHA-1, SHA-224, SHA-256 or MD5. This parameter
                                    can be a value of @ref HASH_Algo_Selection */
  uint32_t HASH_AlgoMode;      /*!< HASH or HMAC. This parameter can be a value 
                                    of @ref HASH_processor_Algorithm_Mode */
  uint32_t HASH_DataType;      /*!< 32-bit data, 16-bit data, 8-bit data or 
                                    bit string. This parameter can be a value of
                                    @ref HASH_Data_Type */
  uint32_t HASH_HMACKeyType;   /*!< HMAC Short key or HMAC Long Key. This parameter
                                    can be a value of @ref HASH_HMAC_Long_key_only_for_HMAC_mode */
}HASH_InitTypeDef;

/** 
  * @brief  HASH message digest result structure definition  
  */ 
typedef struct
{
  uint32_t Data[8];      /*!< Message digest result : 8x 32bit wors for SHA-256,
                                                      7x 32bit wors for SHA-224,
                                                      5x 32bit words for SHA-1 or
                                                      4x 32bit words for MD5  */
} HASH_MsgDigest; 

/** 
  * @brief  HASH context swapping structure definition  
  */ 
typedef struct
{
  uint32_t HASH_IMR; 
  uint32_t HASH_STR;      
  uint32_t HASH_CR;     
  uint32_t HASH_CSR[54];       
}HASH_Context;

/* Exported constants --------------------------------------------------------*/

/** @defgroup HASH_Exported_Constants
  * @{
  */ 

/** @defgroup HASH_Algo_Selection 
  * @{
  */ 
#define HASH_AlgoSelection_SHA1      ((uint32_t)0x0000) /*!< HASH function is SHA1   */
#define HASH_AlgoSelection_SHA224    HASH_CR_ALGO_1     /*!< HASH function is SHA224 */
#define HASH_AlgoSelection_SHA256    HASH_CR_ALGO       /*!< HASH function is SHA256 */
#define HASH_AlgoSelection_MD5       HASH_CR_ALGO_0     /*!< HASH function is MD5    */

#define IS_HASH_ALGOSELECTION(ALGOSELECTION) (((ALGOSELECTION) == HASH_AlgoSelection_SHA1) || \
                                              ((ALGOSELECTION) == HASH_AlgoSelection_SHA224) || \
                                              ((ALGOSELECTION) == HASH_AlgoSelection_SHA256) || \
                                              ((ALGOSELECTION) == HASH_AlgoSelection_MD5))
/**
  * @}
  */

/** @defgroup HASH_processor_Algorithm_Mode 
  * @{
  */ 
#define HASH_AlgoMode_HASH         ((uint32_t)0x00000000) /*!< Algorithm is HASH */ 
#define HASH_AlgoMode_HMAC         HASH_CR_MODE           /*!< Algorithm is HMAC */

#define IS_HASH_ALGOMODE(ALGOMODE) (((ALGOMODE) == HASH_AlgoMode_HASH) || \
                                    ((ALGOMODE) == HASH_AlgoMode_HMAC))
/**
  * @}
  */

/** @defgroup HASH_Data_Type  
  * @{
  */  
#define HASH_DataType_32b          ((uint32_t)0x0000) /*!< 32-bit data. No swapping                     */
#define HASH_DataType_16b          HASH_CR_DATATYPE_0 /*!< 16-bit data. Each half word is swapped       */
#define HASH_DataType_8b           HASH_CR_DATATYPE_1 /*!< 8-bit data. All bytes are swapped            */
#define HASH_DataType_1b           HASH_CR_DATATYPE   /*!< 1-bit data. In the word all bits are swapped */

#define IS_HASH_DATATYPE(DATATYPE) (((DATATYPE) == HASH_DataType_32b)|| \
                                    ((DATATYPE) == HASH_DataType_16b)|| \
                                    ((DATATYPE) == HASH_DataType_8b) || \
                                    ((DATATYPE) == HASH_DataType_1b))
/**
  * @}
  */

/** @defgroup HASH_HMAC_Long_key_only_for_HMAC_mode  
  * @{
  */ 
#define HASH_HMACKeyType_ShortKey      ((uint32_t)0x00000000) /*!< HMAC Key is <= 64 bytes */
#define HASH_HMACKeyType_LongKey       HASH_CR_LKEY           /*!< HMAC Key is > 64 bytes  */

#define IS_HASH_HMAC_KEYTYPE(KEYTYPE) (((KEYTYPE) == HASH_HMACKeyType_ShortKey) || \
                                       ((KEYTYPE) == HASH_HMACKeyType_LongKey))
/**
  * @}
  */

/** @defgroup Number_of_valid_bits_in_last_word_of_the_message   
  * @{
  */  
#define IS_HASH_VALIDBITSNUMBER(VALIDBITS) ((VALIDBITS) <= 0x1F)

/**
  * @}
  */

/** @defgroup HASH_interrupts_definition   
  * @{
  */  
#define HASH_IT_DINI               HASH_IMR_DINIM  /*!< A new block can be entered into the input buffer (DIN) */
#define HASH_IT_DCI                HASH_IMR_DCIM   /*!< Digest calculation complete                            */

#define IS_HASH_IT(IT) ((((IT) & (uint32_t)0xFFFFFFFC) == 0x00000000) && ((IT) != 0x00000000))
#define IS_HASH_GET_IT(IT) (((IT) == HASH_IT_DINI) || ((IT) == HASH_IT_DCI))
				   
/**
  * @}
  */

/** @defgroup HASH_flags_definition   
  * @{
  */  
#define HASH_FLAG_DINIS            HASH_SR_DINIS  /*!< 16 locations are free in the DIN : A new block can be entered into the input buffer */
#define HASH_FLAG_DCIS             HASH_SR_DCIS   /*!< Digest calculation complete                                                         */
#define HASH_FLAG_DMAS             HASH_SR_DMAS   /*!< DMA interface is enabled (DMAE=1) or a transfer is ongoing                          */
#define HASH_FLAG_BUSY             HASH_SR_BUSY   /*!< The hash core is Busy : processing a block of data                                  */
#define HASH_FLAG_DINNE            HASH_CR_DINNE  /*!< DIN not empty : The input buffer contains at least one word of data                 */

#define IS_HASH_GET_FLAG(FLAG) (((FLAG) == HASH_FLAG_DINIS) || \
                                ((FLAG) == HASH_FLAG_DCIS)  || \
                                ((FLAG) == HASH_FLAG_DMAS)  || \
                                ((FLAG) == HASH_FLAG_BUSY)  || \
                                ((FLAG) == HASH_FLAG_DINNE)) 

#define IS_HASH_CLEAR_FLAG(FLAG)(((FLAG) == HASH_FLAG_DINIS) || \
                                 ((FLAG) == HASH_FLAG_DCIS))                                 

/**
  * @}
  */ 

/**
  * @}
  */ 

/* Exported macro ------------------------------------------------------------*/
/* Exported functions --------------------------------------------------------*/ 
  
/*  Function used to set the HASH configuration to the default reset state ****/
void HASH_DeInit(void);

/* HASH Configuration function ************************************************/
void HASH_Init(HASH_InitTypeDef* HASH_InitStruct);
void HASH_StructInit(HASH_InitTypeDef* HASH_InitStruct);
void HASH_Reset(void);

/* HASH Message Digest generation functions ***********************************/
void HASH_DataIn(uint32_t Data);
uint8_t HASH_GetInFIFOWordsNbr(void);
void HASH_SetLastWordValidBitsNbr(uint16_t ValidNumber);
void HASH_StartDigest(void);
void HASH_AutoStartDigest(FunctionalState NewState);
void HASH_GetDigest(HASH_MsgDigest* HASH_MessageDigest);

/* HASH Context swapping functions ********************************************/
void HASH_SaveContext(HASH_Context* HASH_ContextSave);
void HASH_RestoreContext(HASH_Context* HASH_ContextRestore);

/* HASH DMA interface function ************************************************/
void HASH_DMACmd(FunctionalState NewState);

/* HASH Interrupts and flags management functions *****************************/
void HASH_ITConfig(uint32_t HASH_IT, FunctionalState NewState);
FlagStatus HASH_GetFlagStatus(uint32_t HASH_FLAG);
void HASH_ClearFlag(uint32_t HASH_FLAG);
ITStatus HASH_GetITStatus(uint32_t HASH_IT);
void HASH_ClearITPendingBit(uint32_t HASH_IT);

/* High Level SHA1 functions **************************************************/
ErrorStatus HASH_SHA1(uint8_t *Input, uint32_t Ilen, uint8_t Output[20]);
ErrorStatus HMAC_SHA1(uint8_t *Key, uint32_t Keylen,
                      uint8_t *Input, uint32_t Ilen,
                      uint8_t Output[20]);

/* High Level MD5 functions ***************************************************/
ErrorStatus HASH_MD5(uint8_t *Input, uint32_t Ilen, uint8_t Output[16]);
ErrorStatus HMAC_MD5(uint8_t *Key, uint32_t Keylen,
                     uint8_t *Input, uint32_t Ilen,
                     uint8_t Output[16]);

#ifdef __cplusplus
}
#endif

#endif /*__STM32F4xx_HASH_H */

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
