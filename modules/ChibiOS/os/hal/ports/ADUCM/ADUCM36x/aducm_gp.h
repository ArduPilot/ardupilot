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
 * @file    ADUCM36x/aducm_gp.h
 * @brief   ADUCM GP units common header.
 * @note    This file requires definitions from the ADI ADUCM header file.
 *
 * @{
 */

#ifndef ADUCM_GP_H
#define ADUCM_GP_H

/*===========================================================================*/
/* Driver constants.                                                         */
/*===========================================================================*/

/**
 * @name    GP ports definitions
 * @{
 */
#define GP0                             ((aducm_gp_t *)pADI_GP0)
#define GP1                             ((aducm_gp_t *)pADI_GP1)
#define GP2                             ((aducm_gp_t *)pADI_GP2)
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
 * @brief   ADUCM36x GPIO registers block.
 */
typedef struct {

  volatile uint32_t     CON;
  volatile uint32_t     OEN;
  volatile uint32_t     PUL;
  volatile uint32_t     OCE;
  volatile uint32_t     RES0;
  volatile uint32_t     IN;
  volatile uint32_t     OUT;
  volatile uint32_t     SET;
  volatile uint32_t     CLR;
  volatile uint32_t     TGL;
} aducm_gp_t;

/*===========================================================================*/
/* Driver macros.                                                            */
/*===========================================================================*/

/*===========================================================================*/
/* External declarations.                                                    */
/*===========================================================================*/

#endif /* ADUCM_GP_H */

/** @} */
