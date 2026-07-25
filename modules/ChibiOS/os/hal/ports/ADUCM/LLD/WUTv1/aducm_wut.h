/*
    ChibiOS - Copyright (C) 2006..2020 Rocco Marco Guglielmi

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

/**
 * @file    ADUCM/aducm_wut.h
 * @brief   ADUCM WUT units common header.
 * @note    This file requires definitions from the ADI ADUCM header file.
 *
 * @{
 */

#ifndef ADUCM_WUT_H
#define ADUCM_WUT_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    WUT Configuration register definitions
 * @{
 */
#define ADUCM_WUT_CON_PRE_DIV1                            (0 << 0)
#define ADUCM_WUT_CON_PRE_DIV4                            (0 << 0)
#define ADUCM_WUT_CON_PRE_DIV16                           (1 << 0)
#define ADUCM_WUT_CON_PRE_DIV256                          (2 << 0)
#define ADUCM_WUT_CON_PRE_DIV32768                        (3 << 0)
#define ADUCM_WUT_CON_FREEZE                              (1 << 3)
#define ADUCM_WUT_CON_TMODE_PERIODIC                      (0 << 6)
#define ADUCM_WUT_CON_TMODE_FREERUN                       (1 << 6)
#define ADUCM_WUT_CON_ENABLE                              (1 << 7)
#define ADUCM_WUT_CON_WUEN                                (1 << 8)
#define ADUCM_WUT_CON_CLK_PCLK                            (0 << 9)
#define ADUCM_WUT_CON_CLK_LFOSC                           (1 << 9)
#define ADUCM_WUT_CON_CLK_LFOSC1                          (2 << 9)
#define ADUCM_WUT_CON_CLK_ECLKIN                          (3 << 9)
#define ADUCM_WUT_CON_STOP_WUFA                           (1 << 11)
/** @} */

/**
 * @name    WUT Interrupt Enable register definitions
 * @{
 */
#define ADUCM_WUT_IER_WUFA                                (1 << 0)
#define ADUCM_WUT_IER_WUFB                                (1 << 1)
#define ADUCM_WUT_IER_WUFC                                (1 << 2)
#define ADUCM_WUT_IER_WUFD                                (1 << 3)
#define ADUCM_WUT_IER_ROLL                                (1 << 4)
/** @} */

/**
 * @name    WUT Status register definitions
 * @{
 */
#define ADUCM_WUT_STA_WUFA                                (1 << 0)
#define ADUCM_WUT_STA_WUFB                                (1 << 1)
#define ADUCM_WUT_STA_WUFC                                (1 << 2)
#define ADUCM_WUT_STA_WUFD                                (1 << 3)
#define ADUCM_WUT_STA_ROLL                                (1 << 4)
#define ADUCM_WUT_STA_IRQCRY                              (1 << 6)
#define ADUCM_WUT_STA_FREEZE                              (1 << 7)
#define ADUCM_WUT_STA_PDOK                                (1 << 8)
/** @} */

/**
 * @name    WUT Clear Interrupt register definitions
 * @{
 */
#define ADUCM_WUT_CLRI_WUFA                               (1 << 0)
#define ADUCM_WUT_CLRI_WUFB                               (1 << 1)
#define ADUCM_WUT_CLRI_WUFC                               (1 << 2)
#define ADUCM_WUT_CLRI_WUFD                               (1 << 3)
#define ADUCM_WUT_CLRI_ROLL                               (1 << 4)
/** @} */

/**
 * @name    WUT ports definitions
 * @{
 */
#define ADUCM_WUT                       ((aducm_wut_t *)pADI_WUT)
/** @} */

/*===========================================================================*/
/* Driver pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Derived constants and error checks.                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Driver data structures and types.                                         */
/*===========================================================================*/

/**
 * @brief   ADUCM41x WUT registers block.
 */
typedef struct {

  volatile uint32_t     VAL0;
  volatile uint32_t     VAL1;
  volatile uint32_t     CON;
  volatile uint32_t     INC;
  volatile uint32_t     WUFB0;
  volatile uint32_t     WUFB1;
  volatile uint32_t     WUFC0;
  volatile uint32_t     WUFC1;
  volatile uint32_t     WUFD0;
  volatile uint32_t     WUFD1;
  volatile uint32_t     IEN;
  volatile uint32_t     STA;
  volatile uint32_t     CLRI;
  volatile uint32_t     RESERVED[2];
  volatile uint32_t     WUFA0;
  volatile uint32_t     WUFA1;
} aducm_wut_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/**
 * @brief   Enable the Wake-Up timer.
 *
 * @param[in] wutp      Wake-up timer identifier
 *
 * @notapi
 */
#define ADCUM_WUT_ENABLE(wutp) ((wutp)->CON |= ADUCM_WUT_CON_ENABLE)


/**
 * @brief   Enable the Wake-Up timer.
 *
 * @param[in] wutp      Wake-up timer identifier
 *
 * @notapi
 */
#define ADCUM_WUT_DISABLE(wutp) ((wutp)->CON &= ~ADUCM_WUT_CON_ENABLE)

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#endif /* ADUCM_WUT_H */

/** @} */
