/* ----------------------------------------------------------------------
 * Project:      CMSIS DSP Library
 * Title:        arm_common_tables.h
 * Description:  Extern declaration for common tables
 *
 * $Date:        27. January 2017
 * $Revision:    V.1.5.1
 *
 * Target Processor: Cortex-M cores
 * -------------------------------------------------------------------- */
/*
 * Copyright (C) 2010-2017 ARM Limited or its affiliates. All rights reserved.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _FFT_COMMON_TABLES_H
#define _FFT_COMMON_TABLES_H
#include <inttypes.h>


extern const uint16_t armBitRevTable[1024];
extern const float twiddleCoef_4096[8192];
#define twiddleCoef twiddleCoef_4096


#endif /*  FFT_COMMON_TABLES_H */
