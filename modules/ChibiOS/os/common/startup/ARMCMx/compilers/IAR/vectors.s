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
 * @file    ARMCMx/compilers/IAR/vectors.s
 * @brief   Interrupt vectors for Cortex-Mx devices.
 *
 * @defgroup ARMCMx_IAR_VECTORS Cortex-Mx Interrupt Vectors
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

        MODULE  ?vectors

        AAPCS INTERWORK, VFP_COMPATIBLE, RWPI_COMPATIBLE
        PRESERVE8

        SECTION IRQSTACK:DATA:NOROOT(3)
        SECTION .intvec:CODE:NOROOT(3)

        EXTERN  __iar_program_start
        PUBLIC  __vector_table

        DATA

__vector_table:
        DCD     SFE(IRQSTACK)
        DCD     __iar_program_start 
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

/*
 * Default interrupt handlers.
 */
        PUBWEAK NMI_Handler
        PUBWEAK HardFault_Handler
        PUBWEAK MemManage_Handler
        PUBWEAK BusFault_Handler
        PUBWEAK UsageFault_Handler
        PUBWEAK Vector1C
        PUBWEAK Vector20
        PUBWEAK Vector24
        PUBWEAK Vector28
        PUBWEAK SVC_Handler
        PUBWEAK DebugMon_Handler
        PUBWEAK Vector34
        PUBWEAK PendSV_Handler
        PUBWEAK SysTick_Handler
        PUBWEAK Vector40
        PUBWEAK Vector44
        PUBWEAK Vector48
        PUBWEAK Vector4C
        PUBWEAK Vector50
        PUBWEAK Vector54
        PUBWEAK Vector58
        PUBWEAK Vector5C
#if CORTEX_NUM_VECTORS > 8
        PUBWEAK Vector60
        PUBWEAK Vector64
        PUBWEAK Vector68
        PUBWEAK Vector6C
        PUBWEAK Vector70
        PUBWEAK Vector74
        PUBWEAK Vector78
        PUBWEAK Vector7C
#endif
#if CORTEX_NUM_VECTORS > 16
        PUBWEAK Vector80
        PUBWEAK Vector84
        PUBWEAK Vector88
        PUBWEAK Vector8C
        PUBWEAK Vector90
        PUBWEAK Vector94
        PUBWEAK Vector98
        PUBWEAK Vector9C
#endif
#if CORTEX_NUM_VECTORS > 24
        PUBWEAK VectorA0
        PUBWEAK VectorA4
        PUBWEAK VectorA8
        PUBWEAK VectorAC
        PUBWEAK VectorB0
        PUBWEAK VectorB4
        PUBWEAK VectorB8
        PUBWEAK VectorBC
#endif
#if CORTEX_NUM_VECTORS > 32
        PUBWEAK VectorC0
        PUBWEAK VectorC4
        PUBWEAK VectorC8
        PUBWEAK VectorCC
        PUBWEAK VectorD0
        PUBWEAK VectorD4
        PUBWEAK VectorD8
        PUBWEAK VectorDC
#endif
#if CORTEX_NUM_VECTORS > 40
        PUBWEAK VectorE0
        PUBWEAK VectorE4
        PUBWEAK VectorE8
        PUBWEAK VectorEC
        PUBWEAK VectorF0
        PUBWEAK VectorF4
        PUBWEAK VectorF8
        PUBWEAK VectorFC
#endif
#if CORTEX_NUM_VECTORS > 48
        PUBWEAK Vector100
        PUBWEAK Vector104
        PUBWEAK Vector108
        PUBWEAK Vector10C
        PUBWEAK Vector110
        PUBWEAK Vector114
        PUBWEAK Vector118
        PUBWEAK Vector11C
#endif
#if CORTEX_NUM_VECTORS > 56
        PUBWEAK Vector120
        PUBWEAK Vector124
        PUBWEAK Vector128
        PUBWEAK Vector12C
        PUBWEAK Vector130
        PUBWEAK Vector134
        PUBWEAK Vector138
        PUBWEAK Vector13C
#endif
#if CORTEX_NUM_VECTORS > 64
        PUBWEAK Vector140
        PUBWEAK Vector144
        PUBWEAK Vector148
        PUBWEAK Vector14C
        PUBWEAK Vector150
        PUBWEAK Vector154
        PUBWEAK Vector158
        PUBWEAK Vector15C
#endif
#if CORTEX_NUM_VECTORS > 72
        PUBWEAK Vector160
        PUBWEAK Vector164
        PUBWEAK Vector168
        PUBWEAK Vector16C
        PUBWEAK Vector170
        PUBWEAK Vector174
        PUBWEAK Vector178
        PUBWEAK Vector17C
#endif
#if CORTEX_NUM_VECTORS > 80
        PUBWEAK Vector180
        PUBWEAK Vector184
        PUBWEAK Vector188
        PUBWEAK Vector18C
        PUBWEAK Vector190
        PUBWEAK Vector194
        PUBWEAK Vector198
        PUBWEAK Vector19C
#endif
#if CORTEX_NUM_VECTORS > 88
        PUBWEAK Vector1A0
        PUBWEAK Vector1A4
        PUBWEAK Vector1A8
        PUBWEAK Vector1AC
        PUBWEAK Vector1B0
        PUBWEAK Vector1B4
        PUBWEAK Vector1B8
        PUBWEAK Vector1BC
#endif
#if CORTEX_NUM_VECTORS > 96
        PUBWEAK Vector1C0
        PUBWEAK Vector1C4
        PUBWEAK Vector1C8
        PUBWEAK Vector1CC
        PUBWEAK Vector1D0
        PUBWEAK Vector1D4
        PUBWEAK Vector1D8
        PUBWEAK Vector1DC
#endif
#if CORTEX_NUM_VECTORS > 104
        PUBWEAK Vector1E0
        PUBWEAK Vector1E4
        PUBWEAK Vector1E8
        PUBWEAK Vector1EC
        PUBWEAK Vector1F0
        PUBWEAK Vector1F4
        PUBWEAK Vector1F8
        PUBWEAK Vector1FC
#endif
#if CORTEX_NUM_VECTORS > 112
        PUBWEAK Vector200
        PUBWEAK Vector204
        PUBWEAK Vector208
        PUBWEAK Vector20C
        PUBWEAK Vector210
        PUBWEAK Vector214
        PUBWEAK Vector218
        PUBWEAK Vector21C
#endif
#if CORTEX_NUM_VECTORS > 120
        PUBWEAK Vector220
        PUBWEAK Vector224
        PUBWEAK Vector228
        PUBWEAK Vector22C
        PUBWEAK Vector230
        PUBWEAK Vector234
        PUBWEAK Vector238
        PUBWEAK Vector23C
#endif
#if CORTEX_NUM_VECTORS > 128
        PUBWEAK Vector240
        PUBWEAK Vector244
        PUBWEAK Vector248
        PUBWEAK Vector24C
        PUBWEAK Vector250
        PUBWEAK Vector254
        PUBWEAK Vector258
        PUBWEAK Vector25C
#endif
#if CORTEX_NUM_VECTORS > 136
        PUBWEAK Vector260
        PUBWEAK Vector264
        PUBWEAK Vector268
        PUBWEAK Vector26C
        PUBWEAK Vector270
        PUBWEAK Vector274
        PUBWEAK Vector278
        PUBWEAK Vector27C
#endif
#if CORTEX_NUM_VECTORS > 144
        PUBWEAK Vector280
        PUBWEAK Vector284
        PUBWEAK Vector288
        PUBWEAK Vector28C
        PUBWEAK Vector290
        PUBWEAK Vector294
        PUBWEAK Vector298
        PUBWEAK Vector29C
#endif
#if CORTEX_NUM_VECTORS > 152
        PUBWEAK Vector2A0
        PUBWEAK Vector2A4
        PUBWEAK Vector2A8
        PUBWEAK Vector2AC
        PUBWEAK Vector2B0
        PUBWEAK Vector2B4
        PUBWEAK Vector2B8
        PUBWEAK Vector2BC
#endif
#if CORTEX_NUM_VECTORS > 160
        PUBWEAK Vector2C0
        PUBWEAK Vector2C4
        PUBWEAK Vector2C8
        PUBWEAK Vector2CC
        PUBWEAK Vector2D0
        PUBWEAK Vector2D4
        PUBWEAK Vector2D8
        PUBWEAK Vector2DC
#endif
#if CORTEX_NUM_VECTORS > 168
        PUBWEAK Vector2E0
        PUBWEAK Vector2E4
        PUBWEAK Vector2E8
        PUBWEAK Vector2EC
        PUBWEAK Vector2F0
        PUBWEAK Vector2F4
        PUBWEAK Vector2F8
        PUBWEAK Vector2FC
#endif
#if CORTEX_NUM_VECTORS > 176
        PUBWEAK Vector300
        PUBWEAK Vector304
        PUBWEAK Vector308
        PUBWEAK Vector30C
        PUBWEAK Vector310
        PUBWEAK Vector314
        PUBWEAK Vector318
        PUBWEAK Vector31C
#endif
#if CORTEX_NUM_VECTORS > 184
        PUBWEAK Vector320
        PUBWEAK Vector324
        PUBWEAK Vector328
        PUBWEAK Vector32C
        PUBWEAK Vector330
        PUBWEAK Vector334
        PUBWEAK Vector338
        PUBWEAK Vector33C
#endif
#if CORTEX_NUM_VECTORS > 192
        PUBWEAK Vector340
        PUBWEAK Vector344
        PUBWEAK Vector348
        PUBWEAK Vector34C
        PUBWEAK Vector350
        PUBWEAK Vector354
        PUBWEAK Vector358
        PUBWEAK Vector35C
#endif
#if CORTEX_NUM_VECTORS > 200
        PUBWEAK Vector360
        PUBWEAK Vector364
        PUBWEAK Vector368
        PUBWEAK Vector36C
        PUBWEAK Vector370
        PUBWEAK Vector374
        PUBWEAK Vector378
        PUBWEAK Vector37C
#endif
#if CORTEX_NUM_VECTORS > 208
        PUBWEAK Vector380
        PUBWEAK Vector384
        PUBWEAK Vector388
        PUBWEAK Vector38C
        PUBWEAK Vector390
        PUBWEAK Vector394
        PUBWEAK Vector398
        PUBWEAK Vector39C
#endif
#if CORTEX_NUM_VECTORS > 216
        PUBWEAK Vector3A0
        PUBWEAK Vector3A4
        PUBWEAK Vector3A8
        PUBWEAK Vector3AC
        PUBWEAK Vector3B0
        PUBWEAK Vector3B4
        PUBWEAK Vector3B8
        PUBWEAK Vector3BC
#endif
#if CORTEX_NUM_VECTORS > 224
        PUBWEAK Vector3C0
        PUBWEAK Vector3C4
        PUBWEAK Vector3C8
        PUBWEAK Vector3CC
        PUBWEAK Vector3D0
        PUBWEAK Vector3D4
        PUBWEAK Vector3D8
        PUBWEAK Vector3DC
#endif
#if CORTEX_NUM_VECTORS > 232
        PUBWEAK Vector3E0
        PUBWEAK Vector3E4
        PUBWEAK Vector3E8
        PUBWEAK Vector3EC
        PUBWEAK Vector3F0
        PUBWEAK Vector3F4
        PUBWEAK Vector3F8
        PUBWEAK Vector3FC
#endif
        PUBLIC  _unhandled_exception

        SECTION .text:CODE:NOROOT:REORDER(1)
        THUMB

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
_unhandled_exception
        b       _unhandled_exception

        END

#endif /* !defined(__DOXYGEN__) */

/**< @} */
