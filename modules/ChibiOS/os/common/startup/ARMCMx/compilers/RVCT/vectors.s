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
 * @file    ARMCMx/compilers/RVCT/vectors.c
 * @brief   Interrupt vectors for Cortex-Mx devices.
 *
 * @defgroup ARMCMx_RVCT_VECTORS Cortex-Mx Interrupt Vectors
 * @{
 */

#define _FROM_ASM_
#include "cmparams.h"

#if !defined(__DOXYGEN__)

#if (CORTEX_NUM_VECTORS & 7) != 0
#error "the constant CORTEX_NUM_VECTORS must be a multiple of 8"
#endif

#if (CORTEX_NUM_VECTORS < 8) || (CORTEX_NUM_VECTORS > 240)
#error "the constant CORTEX_NUM_VECTORS must be between 8 and 240 inclusive"
#endif

                PRESERVE8

                AREA    RESET, DATA, READONLY

                IMPORT  __initial_msp
                IMPORT  Reset_Handler
                EXPORT  __Vectors

__Vectors
                DCD     __initial_msp
                DCD     Reset_Handler
                DCD     NMI_Handler
                DCD     HardFault_Handler
                DCD     MemManage_Handler
                DCD     BusFault_Handler
                DCD     UsageFault_Handler
                DCD     Vector1C
                DCD     Vector20
                DCD     Vector24
                DCD     Vector28
                DCD     SVC_Handler
                DCD     DebugMon_Handler
                DCD     Vector34
                DCD     PendSV_Handler
                DCD     SysTick_Handler
                DCD     Vector40
                DCD     Vector44
                DCD     Vector48
                DCD     Vector4C
                DCD     Vector50
                DCD     Vector54
                DCD     Vector58
                DCD     Vector5C
#if CORTEX_NUM_VECTORS > 8
                DCD     Vector60
                DCD     Vector64
                DCD     Vector68
                DCD     Vector6C
                DCD     Vector70
                DCD     Vector74
                DCD     Vector78
                DCD     Vector7C
#endif
#if CORTEX_NUM_VECTORS > 16
                DCD     Vector80
                DCD     Vector84
                DCD     Vector88
                DCD     Vector8C
                DCD     Vector90
                DCD     Vector94
                DCD     Vector98
                DCD     Vector9C
#endif
#if CORTEX_NUM_VECTORS > 24
                DCD     VectorA0
                DCD     VectorA4
                DCD     VectorA8
                DCD     VectorAC
                DCD     VectorB0
                DCD     VectorB4
                DCD     VectorB8
                DCD     VectorBC
#endif
#if CORTEX_NUM_VECTORS > 32
                DCD     VectorC0
                DCD     VectorC4
                DCD     VectorC8
                DCD     VectorCC
                DCD     VectorD0
                DCD     VectorD4
                DCD     VectorD8
                DCD     VectorDC
#endif
#if CORTEX_NUM_VECTORS > 40
                DCD     VectorE0
                DCD     VectorE4
                DCD     VectorE8
                DCD     VectorEC
                DCD     VectorF0
                DCD     VectorF4
                DCD     VectorF8
                DCD     VectorFC
#endif
#if CORTEX_NUM_VECTORS > 48
                DCD     Vector100
                DCD     Vector104
                DCD     Vector108
                DCD     Vector10C
                DCD     Vector110
                DCD     Vector114
                DCD     Vector118
                DCD     Vector11C
#endif
#if CORTEX_NUM_VECTORS > 56
                DCD     Vector120
                DCD     Vector124
                DCD     Vector128
                DCD     Vector12C
                DCD     Vector130
                DCD     Vector134
                DCD     Vector138
                DCD     Vector13C
#endif
#if CORTEX_NUM_VECTORS > 64
                DCD     Vector140
                DCD     Vector144
                DCD     Vector148
                DCD     Vector14C
                DCD     Vector150
                DCD     Vector154
                DCD     Vector158
                DCD     Vector15C
#endif
#if CORTEX_NUM_VECTORS > 72
                DCD     Vector160
                DCD     Vector164
                DCD     Vector168
                DCD     Vector16C
                DCD     Vector170
                DCD     Vector174
                DCD     Vector178
                DCD     Vector17C
#endif
#if CORTEX_NUM_VECTORS > 80
                DCD     Vector180
                DCD     Vector184
                DCD     Vector188
                DCD     Vector18C
                DCD     Vector190
                DCD     Vector194
                DCD     Vector198
                DCD     Vector19C
#endif
#if CORTEX_NUM_VECTORS > 88
                DCD     Vector1A0
                DCD     Vector1A4
                DCD     Vector1A8
                DCD     Vector1AC
                DCD     Vector1B0
                DCD     Vector1B4
                DCD     Vector1B8
                DCD     Vector1BC
#endif
#if CORTEX_NUM_VECTORS > 96
                DCD     Vector1C0
                DCD     Vector1C4
                DCD     Vector1C8
                DCD     Vector1CC
                DCD     Vector1D0
                DCD     Vector1D4
                DCD     Vector1D8
                DCD     Vector1DC
#endif
#if CORTEX_NUM_VECTORS > 104
                DCD     Vector1E0
                DCD     Vector1E4
                DCD     Vector1E8
                DCD     Vector1EC
                DCD     Vector1F0
                DCD     Vector1F4
                DCD     Vector1F8
                DCD     Vector1FC
#endif
#if CORTEX_NUM_VECTORS > 112
                DCD     Vector200
                DCD     Vector204
                DCD     Vector208
                DCD     Vector20C
                DCD     Vector210
                DCD     Vector214
                DCD     Vector218
                DCD     Vector21C
#endif
#if CORTEX_NUM_VECTORS > 120
                DCD     Vector220
                DCD     Vector224
                DCD     Vector228
                DCD     Vector22C
                DCD     Vector230
                DCD     Vector234
                DCD     Vector238
                DCD     Vector23C
#endif
#if CORTEX_NUM_VECTORS > 128
                DCD     Vector240
                DCD     Vector244
                DCD     Vector248
                DCD     Vector24C
                DCD     Vector250
                DCD     Vector254
                DCD     Vector258
                DCD     Vector25C
#endif
#if CORTEX_NUM_VECTORS > 136
                DCD     Vector260
                DCD     Vector264
                DCD     Vector268
                DCD     Vector26C
                DCD     Vector270
                DCD     Vector274
                DCD     Vector278
                DCD     Vector27C
#endif
#if CORTEX_NUM_VECTORS > 144
                DCD     Vector280
                DCD     Vector284
                DCD     Vector288
                DCD     Vector28C
                DCD     Vector290
                DCD     Vector294
                DCD     Vector298
                DCD     Vector29C
#endif
#if CORTEX_NUM_VECTORS > 152
                DCD     Vector2A0
                DCD     Vector2A4
                DCD     Vector2A8
                DCD     Vector2AC
                DCD     Vector2B0
                DCD     Vector2B4
                DCD     Vector2B8
                DCD     Vector2BC
#endif
#if CORTEX_NUM_VECTORS > 160
                DCD     Vector2C0
                DCD     Vector2C4
                DCD     Vector2C8
                DCD     Vector2CC
                DCD     Vector2D0
                DCD     Vector2D4
                DCD     Vector2D8
                DCD     Vector2DC
#endif
#if CORTEX_NUM_VECTORS > 168
                DCD     Vector2E0
                DCD     Vector2E4
                DCD     Vector2E8
                DCD     Vector2EC
                DCD     Vector2F0
                DCD     Vector2F4
                DCD     Vector2F8
                DCD     Vector2FC
#endif
#if CORTEX_NUM_VECTORS > 176
                DCD     Vector300
                DCD     Vector304
                DCD     Vector308
                DCD     Vector30C
                DCD     Vector310
                DCD     Vector314
                DCD     Vector318
                DCD     Vector31C
#endif
#if CORTEX_NUM_VECTORS > 184
                DCD     Vector320
                DCD     Vector324
                DCD     Vector328
                DCD     Vector32C
                DCD     Vector330
                DCD     Vector334
                DCD     Vector338
                DCD     Vector33C
#endif
#if CORTEX_NUM_VECTORS > 192
                DCD     Vector340
                DCD     Vector344
                DCD     Vector348
                DCD     Vector34C
                DCD     Vector350
                DCD     Vector354
                DCD     Vector358
                DCD     Vector35C
#endif
#if CORTEX_NUM_VECTORS > 200
                DCD     Vector360
                DCD     Vector364
                DCD     Vector368
                DCD     Vector36C
                DCD     Vector370
                DCD     Vector374
                DCD     Vector378
                DCD     Vector37C
#endif
#if CORTEX_NUM_VECTORS > 208
                DCD     Vector380
                DCD     Vector384
                DCD     Vector388
                DCD     Vector38C
                DCD     Vector390
                DCD     Vector394
                DCD     Vector398
                DCD     Vector39C
#endif
#if CORTEX_NUM_VECTORS > 216
                DCD     Vector3A0
                DCD     Vector3A4
                DCD     Vector3A8
                DCD     Vector3AC
                DCD     Vector3B0
                DCD     Vector3B4
                DCD     Vector3B8
                DCD     Vector3BC
#endif
#if CORTEX_NUM_VECTORS > 224
                DCD     Vector3C0
                DCD     Vector3C4
                DCD     Vector3C8
                DCD     Vector3CC
                DCD     Vector3D0
                DCD     Vector3D4
                DCD     Vector3D8
                DCD     Vector3DC
#endif
#if CORTEX_NUM_VECTORS > 232
                DCD     Vector3E0
                DCD     Vector3E4
                DCD     Vector3E8
                DCD     Vector3EC
                DCD     Vector3F0
                DCD     Vector3F4
                DCD     Vector3F8
                DCD     Vector3FC
#endif

                AREA    |.text|, CODE, READONLY
                THUMB

/*
 * Default interrupt handlers.
 */
                EXPORT  _unhandled_exception
_unhandled_exception PROC 
                EXPORT  NMI_Handler             [WEAK]
                EXPORT  HardFault_Handler       [WEAK]
                EXPORT  MemManage_Handler       [WEAK]
                EXPORT  BusFault_Handler        [WEAK]
                EXPORT  UsageFault_Handler      [WEAK]
                EXPORT  Vector1C                [WEAK]
                EXPORT  Vector20                [WEAK]
                EXPORT  Vector24                [WEAK]
                EXPORT  Vector28                [WEAK]
                EXPORT  SVC_Handler             [WEAK]
                EXPORT  DebugMon_Handler        [WEAK]
                EXPORT  Vector34                [WEAK]
                EXPORT  PendSV_Handler          [WEAK]
                EXPORT  SysTick_Handler         [WEAK]
                EXPORT  Vector40                [WEAK]
                EXPORT  Vector44                [WEAK]
                EXPORT  Vector48                [WEAK]
                EXPORT  Vector4C                [WEAK]
                EXPORT  Vector50                [WEAK]
                EXPORT  Vector54                [WEAK]
                EXPORT  Vector58                [WEAK]
                EXPORT  Vector5C                [WEAK]
#if CORTEX_NUM_VECTORS > 8
                EXPORT  Vector60                [WEAK]
                EXPORT  Vector64                [WEAK]
                EXPORT  Vector68                [WEAK]
                EXPORT  Vector6C                [WEAK]
                EXPORT  Vector70                [WEAK]
                EXPORT  Vector74                [WEAK]
                EXPORT  Vector78                [WEAK]
                EXPORT  Vector7C                [WEAK]
#endif
#if CORTEX_NUM_VECTORS > 16
                EXPORT  Vector80                [WEAK]
                EXPORT  Vector84                [WEAK]
                EXPORT  Vector88                [WEAK]
                EXPORT  Vector8C                [WEAK]
                EXPORT  Vector90                [WEAK]
                EXPORT  Vector94                [WEAK]
                EXPORT  Vector98                [WEAK]
                EXPORT  Vector9C                [WEAK]
#endif
#if CORTEX_NUM_VECTORS > 24
                EXPORT  VectorA0                [WEAK]
                EXPORT  VectorA4                [WEAK]
                EXPORT  VectorA8                [WEAK]
                EXPORT  VectorAC                [WEAK]
                EXPORT  VectorB0                [WEAK]
                EXPORT  VectorB4                [WEAK]
                EXPORT  VectorB8                [WEAK]
                EXPORT  VectorBC                [WEAK]
#endif
#if CORTEX_NUM_VECTORS > 32
                EXPORT  VectorC0                [WEAK]
                EXPORT  VectorC4                [WEAK]
                EXPORT  VectorC8                [WEAK]
                EXPORT  VectorCC                [WEAK]
                EXPORT  VectorD0                [WEAK]
                EXPORT  VectorD4                [WEAK]
                EXPORT  VectorD8                [WEAK]
                EXPORT  VectorDC                [WEAK]
#endif
#if CORTEX_NUM_VECTORS > 40
                EXPORT  VectorE0                [WEAK]
                EXPORT  VectorE4                [WEAK]
                EXPORT  VectorE8                [WEAK]
                EXPORT  VectorEC                [WEAK]
                EXPORT  VectorF0                [WEAK]
                EXPORT  VectorF4                [WEAK]
                EXPORT  VectorF8                [WEAK]
                EXPORT  VectorFC                [WEAK]
#endif
#if CORTEX_NUM_VECTORS > 48
                EXPORT  Vector100               [WEAK]
                EXPORT  Vector104               [WEAK]
                EXPORT  Vector108               [WEAK]
                EXPORT  Vector10C               [WEAK]
                EXPORT  Vector110               [WEAK]
                EXPORT  Vector114               [WEAK]
                EXPORT  Vector118               [WEAK]
                EXPORT  Vector11C               [WEAK]
#endif
#if CORTEX_NUM_VECTORS > 56
                EXPORT  Vector120               [WEAK]
                EXPORT  Vector124               [WEAK]
                EXPORT  Vector128               [WEAK]
                EXPORT  Vector12C               [WEAK]
                EXPORT  Vector130               [WEAK]
                EXPORT  Vector134               [WEAK]
                EXPORT  Vector138               [WEAK]
                EXPORT  Vector13C               [WEAK]
#endif
#if CORTEX_NUM_VECTORS > 64
                EXPORT  Vector140               [WEAK]
                EXPORT  Vector144               [WEAK]
                EXPORT  Vector148               [WEAK]
                EXPORT  Vector14C               [WEAK]
                EXPORT  Vector150               [WEAK]
                EXPORT  Vector154               [WEAK]
                EXPORT  Vector158               [WEAK]
                EXPORT  Vector15C               [WEAK]
#endif
#if CORTEX_NUM_VECTORS > 72
                EXPORT  Vector160               [WEAK]
                EXPORT  Vector164               [WEAK]
                EXPORT  Vector168               [WEAK]
                EXPORT  Vector16C               [WEAK]
                EXPORT  Vector170               [WEAK]
                EXPORT  Vector174               [WEAK]
                EXPORT  Vector178               [WEAK]
                EXPORT  Vector17C               [WEAK]
#endif
#if CORTEX_NUM_VECTORS > 80
                EXPORT  Vector180               [WEAK]
                EXPORT  Vector184               [WEAK]
                EXPORT  Vector188               [WEAK]
                EXPORT  Vector18C               [WEAK]
                EXPORT  Vector190               [WEAK]
                EXPORT  Vector194               [WEAK]
                EXPORT  Vector198               [WEAK]
                EXPORT  Vector19C               [WEAK]
#endif
#if CORTEX_NUM_VECTORS > 88
                EXPORT  Vector1A0               [WEAK]
                EXPORT  Vector1A4               [WEAK]
                EXPORT  Vector1A8               [WEAK]
                EXPORT  Vector1AC               [WEAK]
                EXPORT  Vector1B0               [WEAK]
                EXPORT  Vector1B4               [WEAK]
                EXPORT  Vector1B8               [WEAK]
                EXPORT  Vector1BC               [WEAK]
#endif
#if CORTEX_NUM_VECTORS > 96
                EXPORT  Vector1C0               [WEAK]
                EXPORT  Vector1C4               [WEAK]
                EXPORT  Vector1C8               [WEAK]
                EXPORT  Vector1CC               [WEAK]
                EXPORT  Vector1D0               [WEAK]
                EXPORT  Vector1D4               [WEAK]
                EXPORT  Vector1D8               [WEAK]
                EXPORT  Vector1DC               [WEAK]
#endif
#if CORTEX_NUM_VECTORS > 104
                EXPORT  Vector1E0               [WEAK]
                EXPORT  Vector1E4               [WEAK]
                EXPORT  Vector1E8               [WEAK]
                EXPORT  Vector1EC               [WEAK]
                EXPORT  Vector1F0               [WEAK]
                EXPORT  Vector1F4               [WEAK]
                EXPORT  Vector1F8               [WEAK]
                EXPORT  Vector1FC               [WEAK]
#endif
#if CORTEX_NUM_VECTORS > 112
                EXPORT  Vector200               [WEAK]
                EXPORT  Vector204               [WEAK]
                EXPORT  Vector208               [WEAK]
                EXPORT  Vector20C               [WEAK]
                EXPORT  Vector210               [WEAK]
                EXPORT  Vector214               [WEAK]
                EXPORT  Vector218               [WEAK]
                EXPORT  Vector21C               [WEAK]
#endif
#if CORTEX_NUM_VECTORS > 120
                EXPORT  Vector220               [WEAK]
                EXPORT  Vector224               [WEAK]
                EXPORT  Vector228               [WEAK]
                EXPORT  Vector22C               [WEAK]
                EXPORT  Vector230               [WEAK]
                EXPORT  Vector234               [WEAK]
                EXPORT  Vector238               [WEAK]
                EXPORT  Vector23C               [WEAK]
#endif
#if CORTEX_NUM_VECTORS > 128
                EXPORT  Vector240               [WEAK]
                EXPORT  Vector244               [WEAK]
                EXPORT  Vector248               [WEAK]
                EXPORT  Vector24C               [WEAK]
                EXPORT  Vector250               [WEAK]
                EXPORT  Vector254               [WEAK]
                EXPORT  Vector258               [WEAK]
                EXPORT  Vector25C               [WEAK]
#endif
#if CORTEX_NUM_VECTORS > 136
                EXPORT  Vector260               [WEAK]
                EXPORT  Vector264               [WEAK]
                EXPORT  Vector268               [WEAK]
                EXPORT  Vector26C               [WEAK]
                EXPORT  Vector270               [WEAK]
                EXPORT  Vector274               [WEAK]
                EXPORT  Vector278               [WEAK]
                EXPORT  Vector27C               [WEAK]
#endif
#if CORTEX_NUM_VECTORS > 144
                EXPORT  Vector280               [WEAK]
                EXPORT  Vector284               [WEAK]
                EXPORT  Vector288               [WEAK]
                EXPORT  Vector28C               [WEAK]
                EXPORT  Vector290               [WEAK]
                EXPORT  Vector294               [WEAK]
                EXPORT  Vector298               [WEAK]
                EXPORT  Vector29C               [WEAK]
#endif
#if CORTEX_NUM_VECTORS > 152
                EXPORT  Vector2A0               [WEAK]
                EXPORT  Vector2A4               [WEAK]
                EXPORT  Vector2A8               [WEAK]
                EXPORT  Vector2AC               [WEAK]
                EXPORT  Vector2B0               [WEAK]
                EXPORT  Vector2B4               [WEAK]
                EXPORT  Vector2B8               [WEAK]
                EXPORT  Vector2BC               [WEAK]
#endif
#if CORTEX_NUM_VECTORS > 160
                EXPORT  Vector2C0               [WEAK]
                EXPORT  Vector2C4               [WEAK]
                EXPORT  Vector2C8               [WEAK]
                EXPORT  Vector2CC               [WEAK]
                EXPORT  Vector2D0               [WEAK]
                EXPORT  Vector2D4               [WEAK]
                EXPORT  Vector2D8               [WEAK]
                EXPORT  Vector2DC               [WEAK]
#endif
#if CORTEX_NUM_VECTORS > 168
                EXPORT  Vector2E0               [WEAK]
                EXPORT  Vector2E4               [WEAK]
                EXPORT  Vector2E8               [WEAK]
                EXPORT  Vector2EC               [WEAK]
                EXPORT  Vector2F0               [WEAK]
                EXPORT  Vector2F4               [WEAK]
                EXPORT  Vector2F8               [WEAK]
                EXPORT  Vector2FC               [WEAK]
#endif
#if CORTEX_NUM_VECTORS > 176
                EXPORT  Vector300               [WEAK]
                EXPORT  Vector304               [WEAK]
                EXPORT  Vector308               [WEAK]
                EXPORT  Vector30C               [WEAK]
                EXPORT  Vector310               [WEAK]
                EXPORT  Vector314               [WEAK]
                EXPORT  Vector318               [WEAK]
                EXPORT  Vector31C               [WEAK]
#endif
#if CORTEX_NUM_VECTORS > 184
                EXPORT  Vector320               [WEAK]
                EXPORT  Vector324               [WEAK]
                EXPORT  Vector328               [WEAK]
                EXPORT  Vector32C               [WEAK]
                EXPORT  Vector330               [WEAK]
                EXPORT  Vector334               [WEAK]
                EXPORT  Vector338               [WEAK]
                EXPORT  Vector33C               [WEAK]
#endif
#if CORTEX_NUM_VECTORS > 192
                EXPORT  Vector340               [WEAK]
                EXPORT  Vector344               [WEAK]
                EXPORT  Vector348               [WEAK]
                EXPORT  Vector34C               [WEAK]
                EXPORT  Vector350               [WEAK]
                EXPORT  Vector354               [WEAK]
                EXPORT  Vector358               [WEAK]
                EXPORT  Vector35C               [WEAK]
#endif
#if CORTEX_NUM_VECTORS > 200
                EXPORT  Vector360               [WEAK]
                EXPORT  Vector364               [WEAK]
                EXPORT  Vector368               [WEAK]
                EXPORT  Vector36C               [WEAK]
                EXPORT  Vector370               [WEAK]
                EXPORT  Vector374               [WEAK]
                EXPORT  Vector378               [WEAK]
                EXPORT  Vector37C               [WEAK]
#endif
#if CORTEX_NUM_VECTORS > 208
                EXPORT  Vector380               [WEAK]
                EXPORT  Vector384               [WEAK]
                EXPORT  Vector388               [WEAK]
                EXPORT  Vector38C               [WEAK]
                EXPORT  Vector390               [WEAK]
                EXPORT  Vector394               [WEAK]
                EXPORT  Vector398               [WEAK]
                EXPORT  Vector39C               [WEAK]
#endif
#if CORTEX_NUM_VECTORS > 216
                EXPORT  Vector3A0               [WEAK]
                EXPORT  Vector3A4               [WEAK]
                EXPORT  Vector3A8               [WEAK]
                EXPORT  Vector3AC               [WEAK]
                EXPORT  Vector3B0               [WEAK]
                EXPORT  Vector3B4               [WEAK]
                EXPORT  Vector3B8               [WEAK]
                EXPORT  Vector3BC               [WEAK]
#endif
#if CORTEX_NUM_VECTORS > 224
                EXPORT  Vector3C0               [WEAK]
                EXPORT  Vector3C4               [WEAK]
                EXPORT  Vector3C8               [WEAK]
                EXPORT  Vector3CC               [WEAK]
                EXPORT  Vector3D0               [WEAK]
                EXPORT  Vector3D4               [WEAK]
                EXPORT  Vector3D8               [WEAK]
                EXPORT  Vector3DC               [WEAK]
#endif
#if CORTEX_NUM_VECTORS > 232
                EXPORT  Vector3E0               [WEAK]
                EXPORT  Vector3E4               [WEAK]
                EXPORT  Vector3E8               [WEAK]
                EXPORT  Vector3EC               [WEAK]
                EXPORT  Vector3F0               [WEAK]
                EXPORT  Vector3F4               [WEAK]
                EXPORT  Vector3F8               [WEAK]
                EXPORT  Vector3FC               [WEAK]
#endif

NMI_Handler
HardFault_Handler
MemManage_Handler
BusFault_Handler
UsageFault_Handler
Vector1C
Vector20
Vector24
Vector28
SVC_Handler
DebugMon_Handler
Vector34
PendSV_Handler
SysTick_Handler
Vector40
Vector44
Vector48
Vector4C
Vector50
Vector54
Vector58
Vector5C
#if CORTEX_NUM_VECTORS > 8
Vector60
Vector64
Vector68
Vector6C
Vector70
Vector74
Vector78
Vector7C
#endif
#if CORTEX_NUM_VECTORS > 16
Vector80
Vector84
Vector88
Vector8C
Vector90
Vector94
Vector98
Vector9C
#endif
#if CORTEX_NUM_VECTORS > 24
VectorA0
VectorA4
VectorA8
VectorAC
VectorB0
VectorB4
VectorB8
VectorBC
#endif
#if CORTEX_NUM_VECTORS > 32
VectorC0
VectorC4
VectorC8
VectorCC
VectorD0
VectorD4
VectorD8
VectorDC
#endif
#if CORTEX_NUM_VECTORS > 40
VectorE0
VectorE4
VectorE8
VectorEC
VectorF0
VectorF4
VectorF8
VectorFC
#endif
#if CORTEX_NUM_VECTORS > 48
Vector100
Vector104
Vector108
Vector10C
Vector110
Vector114
Vector118
Vector11C
#endif
#if CORTEX_NUM_VECTORS > 56
Vector120
Vector124
Vector128
Vector12C
Vector130
Vector134
Vector138
Vector13C
#endif
#if CORTEX_NUM_VECTORS > 64
Vector140
Vector144
Vector148
Vector14C
Vector150
Vector154
Vector158
Vector15C
#endif
#if CORTEX_NUM_VECTORS > 72
Vector160
Vector164
Vector168
Vector16C
Vector170
Vector174
Vector178
Vector17C
#endif
#if CORTEX_NUM_VECTORS > 80
Vector180
Vector184
Vector188
Vector18C
Vector190
Vector194
Vector198
Vector19C
#endif
#if CORTEX_NUM_VECTORS > 88
Vector1A0
Vector1A4
Vector1A8
Vector1AC
Vector1B0
Vector1B4
Vector1B8
Vector1BC
#endif
#if CORTEX_NUM_VECTORS > 96
Vector1C0
Vector1C4
Vector1C8
Vector1CC
Vector1D0
Vector1D4
Vector1D8
Vector1DC
#endif
#if CORTEX_NUM_VECTORS > 104
Vector1E0
Vector1E4
Vector1E8
Vector1EC
Vector1F0
Vector1F4
Vector1F8
Vector1FC
#endif
#if CORTEX_NUM_VECTORS > 112
Vector200
Vector204
Vector208
Vector20C
Vector210
Vector214
Vector218
Vector21C
#endif
#if CORTEX_NUM_VECTORS > 120
Vector220
Vector224
Vector228
Vector22C
Vector230
Vector234
Vector238
Vector23C
#endif
#if CORTEX_NUM_VECTORS > 128
Vector240
Vector244
Vector248
Vector24C
Vector250
Vector254
Vector258
Vector25C
#endif
#if CORTEX_NUM_VECTORS > 136
Vector260
Vector264
Vector268
Vector26C
Vector270
Vector274
Vector278
Vector27C
#endif
#if CORTEX_NUM_VECTORS > 144
Vector280
Vector284
Vector288
Vector28C
Vector290
Vector294
Vector298
Vector29C
#endif
#if CORTEX_NUM_VECTORS > 152
Vector2A0
Vector2A4
Vector2A8
Vector2AC
Vector2B0
Vector2B4
Vector2B8
Vector2BC
#endif
#if CORTEX_NUM_VECTORS > 160
Vector2C0
Vector2C4
Vector2C8
Vector2CC
Vector2D0
Vector2D4
Vector2D8
Vector2DC
#endif
#if CORTEX_NUM_VECTORS > 168
Vector2E0
Vector2E4
Vector2E8
Vector2EC
Vector2F0
Vector2F4
Vector2F8
Vector2FC
#endif
#if CORTEX_NUM_VECTORS > 176
Vector300
Vector304
Vector308
Vector30C
Vector310
Vector314
Vector318
Vector31C
#endif
#if CORTEX_NUM_VECTORS > 184
Vector320
Vector324
Vector328
Vector32C
Vector330
Vector334
Vector338
Vector33C
#endif
#if CORTEX_NUM_VECTORS > 192
Vector340
Vector344
Vector348
Vector34C
Vector350
Vector354
Vector358
Vector35C
#endif
#if CORTEX_NUM_VECTORS > 200
Vector360
Vector364
Vector368
Vector36C
Vector370
Vector374
Vector378
Vector37C
#endif
#if CORTEX_NUM_VECTORS > 208
Vector380
Vector384
Vector388
Vector38C
Vector390
Vector394
Vector398
Vector39C
#endif
#if CORTEX_NUM_VECTORS > 216
Vector3A0
Vector3A4
Vector3A8
Vector3AC
Vector3B0
Vector3B4
Vector3B8
Vector3BC
#endif
#if CORTEX_NUM_VECTORS > 224
Vector3C0
Vector3C4
Vector3C8
Vector3CC
Vector3D0
Vector3D4
Vector3D8
Vector3DC
#endif
#if CORTEX_NUM_VECTORS > 232
Vector3E0
Vector3E4
Vector3E8
Vector3EC
Vector3F0
Vector3F4
Vector3F8
Vector3FC
#endif
                b       _unhandled_exception
                ENDP

                END

#endif /* !defined(__DOXYGEN__) */

/**< @} */
