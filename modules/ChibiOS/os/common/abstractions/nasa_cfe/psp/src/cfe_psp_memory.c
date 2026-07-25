/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

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
 * @file    cfe_psp_memory.c
 * @brief   CFE PSP memory module code.
 *
 * @addtogroup nasa_cfe_psp_memory
 * @{
 */

#include "ch.h"

#include "common_types.h"
#include "osapi.h"
#include "cfe_psp.h"

/*===========================================================================*/
/* Module local definitions.                                                 */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported variables.                                                */
/*===========================================================================*/

/*===========================================================================*/
/* Module local types.                                                       */
/*===========================================================================*/

/*===========================================================================*/
/* Module local variables.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module local functions.                                                   */
/*===========================================================================*/

/*===========================================================================*/
/* Module exported functions.                                                */
/*===========================================================================*/

int32 CFE_PSP_GetCDSSize(uint32 *SizeOfCDS) {

  (void)SizeOfCDS;

  return 0;
}

int32 CFE_PSP_WriteToCDS(void *PtrToDataToWrite,
                         uint32 CDSOffset,
                         uint32 NumBytes) {

  (void)PtrToDataToWrite;
  (void)CDSOffset;
  (void)NumBytes;

  return 0;
}

int32 CFE_PSP_ReadFromCDS(void *PtrToDataToRead,
                          uint32 CDSOffset,
                          uint32 NumBytes) {

  (void)PtrToDataToRead;
  (void)CDSOffset;
  (void)NumBytes;

  return 0;
}

int32 CFE_PSP_GetResetArea(void *PtrToResetArea,
                           uint32 *SizeOfResetArea) {

  (void)PtrToResetArea;
  (void)SizeOfResetArea;

  return 0;
}

int32 CFE_PSP_GetUserReservedArea(void *PtrToUserArea,
                                  uint32 *SizeOfUserArea) {

  (void)PtrToUserArea;
  (void)SizeOfUserArea;

  return 0;
}

int32 CFE_PSP_GetVolatileDiskMem(void *PtrToVolDisk,
                                 uint32 *SizeOfVolDisk) {

  (void)PtrToVolDisk;
  (void)SizeOfVolDisk;

  return 0;
}

int32 CFE_PSP_InitProcessorReservedMemory(uint32 RestartType) {

  (void)RestartType;

  return 0;
}

int32 CFE_PSP_GetKernelTextSegmentInfo(void *PtrToKernelSegment,
                                       uint32 *SizeOfKernelSegment) {

  (void)PtrToKernelSegment;
  (void)SizeOfKernelSegment;

  return 0;
}

int32 CFE_PSP_GetCFETextSegmentInfo(void *PtrToCFESegment,
                                    uint32 *SizeOfCFESegment) {

  (void)PtrToCFESegment;
  (void)SizeOfCFESegment;

  return 0;
}

/** @} */
