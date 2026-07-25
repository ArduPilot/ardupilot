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
 * @file    ARMCMx/compilers/GCC/vectors.S
 * @brief   Interrupt vectors for Cortex-Mx devices.
 *
 * @defgroup ARMCMx_GCC_VECTORS Cortex-Mx Interrupt Vectors
 * @{
 */

#define _FROM_ASM_
#include "cmparams.h"

#if (CORTEX_NUM_VECTORS % 8) != 0
#error "the constant CORTEX_NUM_VECTORS must be a multiple of 8"
#endif

#if (CORTEX_NUM_VECTORS < 8) || (CORTEX_NUM_VECTORS > 240)
#error "the constant CORTEX_NUM_VECTORS must be between 8 and 240 inclusive"
#endif

/*===========================================================================*/
/* Module constants.                                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Module pre-compile time settings.                                         */
/*===========================================================================*/

/*===========================================================================*/
/* Code section.                                                             */
/*===========================================================================*/

#if !defined(__DOXYGEN__)

        .syntax unified
        .cpu    cortex-m0
        .thumb

        .section    .vectors, "ax"
        .align      4
        .globl      _vectors
_vectors:
        .long       __main_stack_end__
        .long       Reset_Handler
        .long       NMI_Handler
        .long       HardFault_Handler
        .long       MemManage_Handler
        .long       BusFault_Handler
        .long       UsageFault_Handler
        .long       SecureFault_Handler
        .long       Vector20
        .long       Vector24
        .long       Vector28
        .long       SVC_Handler
        .long       DebugMon_Handler
        .long       Vector34
        .long       PendSV_Handler
        .long       SysTick_Handler
        .long       Vector40,   Vector44,   Vector48,   Vector4C
#if CORTEX_NUM_VECTORS > 4
        .long       Vector50,   Vector54,   Vector58,   Vector5C
#endif
#if CORTEX_NUM_VECTORS > 8
        .long       Vector60,   Vector64,   Vector68,   Vector6C
#endif
#if CORTEX_NUM_VECTORS > 12
        .long       Vector70,   Vector74,   Vector78,   Vector7C
#endif
#if CORTEX_NUM_VECTORS > 16
        .long       Vector80,   Vector84,   Vector88,   Vector8C
#endif
#if CORTEX_NUM_VECTORS > 20
        .long       Vector90,   Vector94,   Vector98,   Vector9C
#endif
#if CORTEX_NUM_VECTORS > 24
        .long       VectorA0,   VectorA4,   VectorA8,   VectorAC
#endif
#if CORTEX_NUM_VECTORS > 28
        .long       VectorB0,   VectorB4,   VectorB8,   VectorBC
#endif
#if CORTEX_NUM_VECTORS > 32
        .long       VectorC0,   VectorC4,   VectorC8,   VectorCC
#endif
#if CORTEX_NUM_VECTORS > 36
        .long       VectorD0,   VectorD4,   VectorD8,   VectorDC
#endif
#if CORTEX_NUM_VECTORS > 40
        .long       VectorE0,   VectorE4,   VectorE8,   VectorEC
#endif
#if CORTEX_NUM_VECTORS > 44
        .long       VectorF0,   VectorF4,   VectorF8,   VectorFC
#endif
#if CORTEX_NUM_VECTORS > 48
        .long       Vector100,  Vector104,  Vector108,  Vector10C
#endif
#if CORTEX_NUM_VECTORS > 52
        .long       Vector110,  Vector114,  Vector118,  Vector11C
#endif
#if CORTEX_NUM_VECTORS > 56
        .long       Vector120,  Vector124,  Vector128,  Vector12C
#endif
#if CORTEX_NUM_VECTORS > 60
        .long       Vector130,  Vector134,  Vector138,  Vector13C
#endif
#if CORTEX_NUM_VECTORS > 64
        .long       Vector140,  Vector144,  Vector148,  Vector14C
#endif
#if CORTEX_NUM_VECTORS > 68
        .long       Vector150,  Vector154,  Vector158,  Vector15C
#endif
#if CORTEX_NUM_VECTORS > 72
        .long       Vector160,  Vector164,  Vector168,  Vector16C
#endif
#if CORTEX_NUM_VECTORS > 76
        .long       Vector170,  Vector174,  Vector178,  Vector17C
#endif
#if CORTEX_NUM_VECTORS > 80
        .long       Vector180,  Vector184,  Vector188,  Vector18C
#endif
#if CORTEX_NUM_VECTORS > 84
        .long       Vector190,  Vector194,  Vector198,  Vector19C
#endif
#if CORTEX_NUM_VECTORS > 88
        .long       Vector1A0,  Vector1A4,  Vector1A8,  Vector1AC
#endif
#if CORTEX_NUM_VECTORS > 92
        .long       Vector1B0,  Vector1B4,  Vector1B8,  Vector1BC
#endif
#if CORTEX_NUM_VECTORS > 96
        .long       Vector1C0,  Vector1C4,  Vector1C8,  Vector1CC
#endif
#if CORTEX_NUM_VECTORS > 100
        .long       Vector1D0,  Vector1D4,  Vector1D8,  Vector1DC
#endif
#if CORTEX_NUM_VECTORS > 104
        .long       Vector1E0,  Vector1E4,  Vector1E8,  Vector1EC
#endif
#if CORTEX_NUM_VECTORS > 108
        .long       Vector1F0,  Vector1F4,  Vector1F8,  Vector1FC
#endif
#if CORTEX_NUM_VECTORS > 112
        .long       Vector200,  Vector204,  Vector208,  Vector20C
#endif
#if CORTEX_NUM_VECTORS > 116
        .long       Vector210,  Vector214,  Vector218,  Vector21C
#endif
#if CORTEX_NUM_VECTORS > 120
        .long       Vector220,  Vector224,  Vector228,  Vector22C
#endif
#if CORTEX_NUM_VECTORS > 124
        .long       Vector230,  Vector234,  Vector238,  Vector23C
#endif
#if CORTEX_NUM_VECTORS > 128
        .long       Vector240,  Vector244,  Vector248,  Vector24C
#endif
#if CORTEX_NUM_VECTORS > 132
        .long       Vector250,  Vector254,  Vector258,  Vector25C
#endif
#if CORTEX_NUM_VECTORS > 136
        .long       Vector260,  Vector264,  Vector268,  Vector26C
#endif
#if CORTEX_NUM_VECTORS > 140
        .long       Vector270,  Vector274,  Vector278,  Vector27C
#endif
#if CORTEX_NUM_VECTORS > 144
        .long       Vector280,  Vector284,  Vector288,  Vector28C
#endif
#if CORTEX_NUM_VECTORS > 148
        .long       Vector290,  Vector294,  Vector298,  Vector29C
#endif
#if CORTEX_NUM_VECTORS > 152
        .long       Vector2A0,  Vector2A4,  Vector2A8,  Vector2AC
#endif
#if CORTEX_NUM_VECTORS > 156
        .long       Vector2B0,  Vector2B4,  Vector2B8,  Vector2BC
#endif
#if CORTEX_NUM_VECTORS > 160
        .long       Vector2C0,  Vector2C4,  Vector2C8,  Vector2CC
#endif
#if CORTEX_NUM_VECTORS > 164
        .long       Vector2D0,  Vector2D4,  Vector2D8,  Vector2DC
#endif
#if CORTEX_NUM_VECTORS > 168
        .long       Vector2E0,  Vector2E4,  Vector2E8,  Vector2EC
#endif
#if CORTEX_NUM_VECTORS > 172
        .long       Vector2F0,  Vector2F4,  Vector2F8,  Vector2FC
#endif
#if CORTEX_NUM_VECTORS > 176
        .long       Vector300,  Vector304,  Vector308,  Vector30C
#endif
#if CORTEX_NUM_VECTORS > 180
        .long       Vector310,  Vector314,  Vector318,  Vector31C
#endif
#if CORTEX_NUM_VECTORS > 184
        .long       Vector320,  Vector324,  Vector328,  Vector32C
#endif
#if CORTEX_NUM_VECTORS > 188
        .long       Vector330,  Vector334,  Vector338,  Vector33C
#endif
#if CORTEX_NUM_VECTORS > 192
        .long       Vector340,  Vector344,  Vector348,  Vector34C
#endif
#if CORTEX_NUM_VECTORS > 196
        .long       Vector350,  Vector354,  Vector358,  Vector35C
#endif
#if CORTEX_NUM_VECTORS > 200
        .long       Vector360,  Vector364,  Vector368,  Vector36C
#endif
#if CORTEX_NUM_VECTORS > 204
        .long       Vector370,  Vector374,  Vector378,  Vector37C
#endif
#if CORTEX_NUM_VECTORS > 208
        .long       Vector380,  Vector384,  Vector388,  Vector38C
#endif
#if CORTEX_NUM_VECTORS > 212
        .long       Vector390,  Vector394,  Vector398,  Vector39C
#endif
#if CORTEX_NUM_VECTORS > 216
        .long       Vector3A0,  Vector3A4,  Vector3A8,  Vector3AC
#endif
#if CORTEX_NUM_VECTORS > 220
        .long       Vector3B0,  Vector3B4,  Vector3B8,  Vector3BC
#endif
#if CORTEX_NUM_VECTORS > 224
        .long       Vector3C0,  Vector3C4,  Vector3C8,  Vector3CC
#endif
#if CORTEX_NUM_VECTORS > 228
        .long       Vector3D0,  Vector3D4,  Vector3D8,  Vector3DC
#endif
#if CORTEX_NUM_VECTORS > 232
        .long       Vector3E0,  Vector3E4,  Vector3E8,  Vector3EC
#endif
#if CORTEX_NUM_VECTORS > 236
        .long       Vector3F0,  Vector3F4,  Vector3F8,  Vector3FC
#endif

        .text

        .align      2
        .thumb_func
        .weak       Reset_Handler
Reset_Handler:
         b          _crt0_entry

        .thumb_func
        .weak       NMI_Handler
        .weak       HardFault_Handler
        .weak       MemManage_Handler
        .weak       BusFault_Handler
        .weak       UsageFault_Handler
        .weak       SecureFault_Handler
        .weak       Vector20
        .weak       Vector24
        .weak       Vector28
        .weak       SVC_Handler
        .weak       DebugMon_Handler
        .weak       Vector34
        .weak       PendSV_Handler
        .weak       SysTick_Handler
        .weak       Vector40,   Vector44,   Vector48,   Vector4C
#if CORTEX_NUM_VECTORS > 4
        .weak       Vector50,   Vector54,   Vector58,   Vector5C
#endif
#if CORTEX_NUM_VECTORS > 8
        .weak       Vector60,   Vector64,   Vector68,   Vector6C
#endif
#if CORTEX_NUM_VECTORS > 12
        .weak       Vector70,   Vector74,   Vector78,   Vector7C
#endif
#if CORTEX_NUM_VECTORS > 16
        .weak       Vector80,   Vector84,   Vector88,   Vector8C
#endif
#if CORTEX_NUM_VECTORS > 20
        .weak       Vector90,   Vector94,   Vector98,   Vector9C
#endif
#if CORTEX_NUM_VECTORS > 24
        .weak       VectorA0,   VectorA4,   VectorA8,   VectorAC
#endif
#if CORTEX_NUM_VECTORS > 28
        .weak       VectorB0,   VectorB4,   VectorB8,   VectorBC
#endif
#if CORTEX_NUM_VECTORS > 32
        .weak       VectorC0,   VectorC4,   VectorC8,   VectorCC
#endif
#if CORTEX_NUM_VECTORS > 36
        .weak       VectorD0,   VectorD4,   VectorD8,   VectorDC
#endif
#if CORTEX_NUM_VECTORS > 40
        .weak       VectorE0,   VectorE4,   VectorE8,   VectorEC
#endif
#if CORTEX_NUM_VECTORS > 44
        .weak       VectorF0,   VectorF4,   VectorF8,   VectorFC
#endif
#if CORTEX_NUM_VECTORS > 48
        .weak       Vector100,  Vector104,  Vector108,  Vector10C
#endif
#if CORTEX_NUM_VECTORS > 52
        .weak       Vector110,  Vector114,  Vector118,  Vector11C
#endif
#if CORTEX_NUM_VECTORS > 56
        .weak       Vector120,  Vector124,  Vector128,  Vector12C
#endif
#if CORTEX_NUM_VECTORS > 60
        .weak       Vector130,  Vector134,  Vector138,  Vector13C
#endif
#if CORTEX_NUM_VECTORS > 64
        .weak       Vector140,  Vector144,  Vector148,  Vector14C
#endif
#if CORTEX_NUM_VECTORS > 68
        .weak       Vector150,  Vector154,  Vector158,  Vector15C
#endif
#if CORTEX_NUM_VECTORS > 72
        .weak       Vector160,  Vector164,  Vector168,  Vector16C
#endif
#if CORTEX_NUM_VECTORS > 76
        .weak       Vector170,  Vector174,  Vector178,  Vector17C
#endif
#if CORTEX_NUM_VECTORS > 80
        .weak       Vector180,  Vector184,  Vector188,  Vector18C
#endif
#if CORTEX_NUM_VECTORS > 84
        .weak       Vector190,  Vector194,  Vector198,  Vector19C
#endif
#if CORTEX_NUM_VECTORS > 88
        .weak       Vector1A0,  Vector1A4,  Vector1A8,  Vector1AC
#endif
#if CORTEX_NUM_VECTORS > 92
        .weak       Vector1B0,  Vector1B4,  Vector1B8,  Vector1BC
#endif
#if CORTEX_NUM_VECTORS > 96
        .weak       Vector1C0,  Vector1C4,  Vector1C8,  Vector1CC
#endif
#if CORTEX_NUM_VECTORS > 100
        .weak       Vector1D0,  Vector1D4,  Vector1D8,  Vector1DC
#endif
#if CORTEX_NUM_VECTORS > 104
        .weak       Vector1E0,  Vector1E4,  Vector1E8,  Vector1EC
#endif
#if CORTEX_NUM_VECTORS > 108
        .weak       Vector1F0,  Vector1F4,  Vector1F8,  Vector1FC
#endif
#if CORTEX_NUM_VECTORS > 112
        .weak       Vector200,  Vector204,  Vector208,  Vector20C
#endif
#if CORTEX_NUM_VECTORS > 116
        .weak       Vector210,  Vector214,  Vector218,  Vector21C
#endif
#if CORTEX_NUM_VECTORS > 120
        .weak       Vector220,  Vector224,  Vector228,  Vector22C
#endif
#if CORTEX_NUM_VECTORS > 124
        .weak       Vector230,  Vector234,  Vector238,  Vector23C
#endif
#if CORTEX_NUM_VECTORS > 128
        .weak       Vector240,  Vector244,  Vector248,  Vector24C
#endif
#if CORTEX_NUM_VECTORS > 132
        .weak       Vector250,  Vector254,  Vector258,  Vector25C
#endif
#if CORTEX_NUM_VECTORS > 136
        .weak       Vector260,  Vector264,  Vector268,  Vector26C
#endif
#if CORTEX_NUM_VECTORS > 140
        .weak       Vector270,  Vector274,  Vector278,  Vector27C
#endif
#if CORTEX_NUM_VECTORS > 144
        .weak       Vector280,  Vector284,  Vector288,  Vector28C
#endif
#if CORTEX_NUM_VECTORS > 148
        .weak       Vector290,  Vector294,  Vector298,  Vector29C
#endif
#if CORTEX_NUM_VECTORS > 152
        .weak       Vector2A0,  Vector2A4,  Vector2A8,  Vector2AC
#endif
#if CORTEX_NUM_VECTORS > 156
        .weak       Vector2B0,  Vector2B4,  Vector2B8,  Vector2BC
#endif
#if CORTEX_NUM_VECTORS > 160
        .weak       Vector2C0,  Vector2C4,  Vector2C8,  Vector2CC
#endif
#if CORTEX_NUM_VECTORS > 164
        .weak       Vector2D0,  Vector2D4,  Vector2D8,  Vector2DC
#endif
#if CORTEX_NUM_VECTORS > 168
        .weak       Vector2E0,  Vector2E4,  Vector2E8,  Vector2EC
#endif
#if CORTEX_NUM_VECTORS > 172
        .weak       Vector2F0,  Vector2F4,  Vector2F8,  Vector2FC
#endif
#if CORTEX_NUM_VECTORS > 176
        .weak       Vector300,  Vector304,  Vector308,  Vector30C
#endif
#if CORTEX_NUM_VECTORS > 180
        .weak       Vector310,  Vector314,  Vector318,  Vector31C
#endif
#if CORTEX_NUM_VECTORS > 184
        .weak       Vector320,  Vector324,  Vector328,  Vector32C
#endif
#if CORTEX_NUM_VECTORS > 188
        .weak       Vector330,  Vector334,  Vector338,  Vector33C
#endif
#if CORTEX_NUM_VECTORS > 192
        .weak       Vector340,  Vector344,  Vector348,  Vector34C
#endif
#if CORTEX_NUM_VECTORS > 196
        .weak       Vector350,  Vector354,  Vector358,  Vector35C
#endif
#if CORTEX_NUM_VECTORS > 200
        .weak       Vector360,  Vector364,  Vector368,  Vector36C
#endif
#if CORTEX_NUM_VECTORS > 204
        .weak       Vector370,  Vector374,  Vector378,  Vector37C
#endif
#if CORTEX_NUM_VECTORS > 208
        .weak       Vector380,  Vector384,  Vector388,  Vector38C
#endif
#if CORTEX_NUM_VECTORS > 212
        .weak       Vector390,  Vector394,  Vector398,  Vector39C
#endif
#if CORTEX_NUM_VECTORS > 216
        .weak       Vector3A0,  Vector3A4,  Vector3A8,  Vector3AC
#endif
#if CORTEX_NUM_VECTORS > 220
        .weak       Vector3B0,  Vector3B4,  Vector3B8,  Vector3BC
#endif
#if CORTEX_NUM_VECTORS > 224
        .weak       Vector3C0,  Vector3C4,  Vector3C8,  Vector3CC
#endif
#if CORTEX_NUM_VECTORS > 228
        .weak       Vector3D0,  Vector3D4,  Vector3D8,  Vector3DC
#endif
#if CORTEX_NUM_VECTORS > 232
        .weak       Vector3E0,  Vector3E4,  Vector3E8,  Vector3EC
#endif
#if CORTEX_NUM_VECTORS > 236
        .weak       Vector3F0,  Vector3F4,  Vector3F8,  Vector3FC
#endif

        .thumb_func
NMI_Handler:
        .thumb_func
HardFault_Handler:
        .thumb_func
MemManage_Handler:
        .thumb_func
BusFault_Handler:
        .thumb_func
UsageFault_Handler:
        .thumb_func
SecureFault_Handler:
        .thumb_func
Vector20:
        .thumb_func
Vector24:
        .thumb_func
Vector28:
        .thumb_func
SVC_Handler:
        .thumb_func
DebugMon_Handler:
        .thumb_func
Vector34:
        .thumb_func
PendSV_Handler:
        .thumb_func
SysTick_Handler:
        .thumb_func
Vector40:
        .thumb_func
Vector44:
        .thumb_func
Vector48:
        .thumb_func
Vector4C:
        .thumb_func
Vector50:
        .thumb_func
Vector54:
        .thumb_func
Vector58:
        .thumb_func
Vector5C:
#if CORTEX_NUM_VECTORS > 8
        .thumb_func
Vector60:
        .thumb_func
Vector64:
        .thumb_func
Vector68:
        .thumb_func
Vector6C:
        .thumb_func
Vector70:
        .thumb_func
Vector74:
        .thumb_func
Vector78:
        .thumb_func
Vector7C:
#endif
#if CORTEX_NUM_VECTORS > 16
        .thumb_func
Vector80:
        .thumb_func
Vector84:
        .thumb_func
Vector88:
        .thumb_func
Vector8C:
        .thumb_func
Vector90:
        .thumb_func
Vector94:
        .thumb_func
Vector98:
        .thumb_func
Vector9C:
#endif
#if CORTEX_NUM_VECTORS > 24
        .thumb_func
VectorA0:
        .thumb_func
VectorA4:
        .thumb_func
VectorA8:
        .thumb_func
VectorAC:
        .thumb_func
VectorB0:
        .thumb_func
VectorB4:
        .thumb_func
VectorB8:
        .thumb_func
VectorBC:
#endif
#if CORTEX_NUM_VECTORS > 32
        .thumb_func
VectorC0:
        .thumb_func
VectorC4:
        .thumb_func
VectorC8:
        .thumb_func
VectorCC:
        .thumb_func
VectorD0:
        .thumb_func
VectorD4:
        .thumb_func
VectorD8:
        .thumb_func
VectorDC:
#endif
#if CORTEX_NUM_VECTORS > 40
        .thumb_func
VectorE0:
        .thumb_func
VectorE4:
        .thumb_func
VectorE8:
        .thumb_func
VectorEC:
        .thumb_func
VectorF0:
        .thumb_func
VectorF4:
        .thumb_func
VectorF8:
        .thumb_func
VectorFC:
#endif
#if CORTEX_NUM_VECTORS > 48
        .thumb_func
Vector100:
        .thumb_func
Vector104:
        .thumb_func
Vector108:
        .thumb_func
Vector10C:
        .thumb_func
Vector110:
        .thumb_func
Vector114:
        .thumb_func
Vector118:
        .thumb_func
Vector11C:
#endif
#if CORTEX_NUM_VECTORS > 56
        .thumb_func
Vector120:
        .thumb_func
Vector124:
        .thumb_func
Vector128:
        .thumb_func
Vector12C:
        .thumb_func
Vector130:
        .thumb_func
Vector134:
        .thumb_func
Vector138:
        .thumb_func
Vector13C:
#endif
#if CORTEX_NUM_VECTORS > 64
        .thumb_func
Vector140:
        .thumb_func
Vector144:
        .thumb_func
Vector148:
        .thumb_func
Vector14C:
        .thumb_func
Vector150:
        .thumb_func
Vector154:
        .thumb_func
Vector158:
        .thumb_func
Vector15C:
#endif
#if CORTEX_NUM_VECTORS > 72
        .thumb_func
Vector160:
        .thumb_func
Vector164:
        .thumb_func
Vector168:
        .thumb_func
Vector16C:
        .thumb_func
Vector170:
        .thumb_func
Vector174:
        .thumb_func
Vector178:
        .thumb_func
Vector17C:
#endif
#if CORTEX_NUM_VECTORS > 80
        .thumb_func
Vector180:
        .thumb_func
Vector184:
        .thumb_func
Vector188:
        .thumb_func
Vector18C:
        .thumb_func
Vector190:
        .thumb_func
Vector194:
        .thumb_func
Vector198:
        .thumb_func
Vector19C:
#endif
#if CORTEX_NUM_VECTORS > 88
        .thumb_func
Vector1A0:
        .thumb_func
Vector1A4:
        .thumb_func
Vector1A8:
        .thumb_func
Vector1AC:
        .thumb_func
Vector1B0:
        .thumb_func
Vector1B4:
        .thumb_func
Vector1B8:
        .thumb_func
Vector1BC:
#endif
#if CORTEX_NUM_VECTORS > 96
        .thumb_func
Vector1C0:
        .thumb_func
Vector1C4:
        .thumb_func
Vector1C8:
        .thumb_func
Vector1CC:
        .thumb_func
Vector1D0:
        .thumb_func
Vector1D4:
        .thumb_func
Vector1D8:
        .thumb_func
Vector1DC:
#endif
#if CORTEX_NUM_VECTORS > 104
        .thumb_func
Vector1E0:
        .thumb_func
Vector1E4:
        .thumb_func
Vector1E8:
        .thumb_func
Vector1EC:
        .thumb_func
Vector1F0:
        .thumb_func
Vector1F4:
        .thumb_func
Vector1F8:
        .thumb_func
Vector1FC:
#endif
#if CORTEX_NUM_VECTORS > 112
        .thumb_func
Vector200:
        .thumb_func
Vector204:
        .thumb_func
Vector208:
        .thumb_func
Vector20C:
        .thumb_func
Vector210:
        .thumb_func
Vector214:
        .thumb_func
Vector218:
        .thumb_func
Vector21C:
#endif
#if CORTEX_NUM_VECTORS > 120
        .thumb_func
Vector220:
        .thumb_func
Vector224:
        .thumb_func
Vector228:
        .thumb_func
Vector22C:
        .thumb_func
Vector230:
        .thumb_func
Vector234:
        .thumb_func
Vector238:
        .thumb_func
Vector23C:
#endif
#if CORTEX_NUM_VECTORS > 128
        .thumb_func
Vector240:
        .thumb_func
Vector244:
        .thumb_func
Vector248:
        .thumb_func
Vector24C:
        .thumb_func
Vector250:
        .thumb_func
Vector254:
        .thumb_func
Vector258:
        .thumb_func
Vector25C:
#endif
#if CORTEX_NUM_VECTORS > 136
        .thumb_func
Vector260:
        .thumb_func
Vector264:
        .thumb_func
Vector268:
        .thumb_func
Vector26C:
        .thumb_func
Vector270:
        .thumb_func
Vector274:
        .thumb_func
Vector278:
        .thumb_func
Vector27C:
#endif
#if CORTEX_NUM_VECTORS > 144
        .thumb_func
Vector280:
        .thumb_func
Vector284:
        .thumb_func
Vector288:
        .thumb_func
Vector28C:
        .thumb_func
Vector290:
        .thumb_func
Vector294:
        .thumb_func
Vector298:
        .thumb_func
Vector29C:
#endif
#if CORTEX_NUM_VECTORS > 152
        .thumb_func
Vector2A0:
        .thumb_func
Vector2A4:
        .thumb_func
Vector2A8:
        .thumb_func
Vector2AC:
        .thumb_func
Vector2B0:
        .thumb_func
Vector2B4:
        .thumb_func
Vector2B8:
        .thumb_func
Vector2BC:
#endif
#if CORTEX_NUM_VECTORS > 160
        .thumb_func
Vector2C0:
        .thumb_func
Vector2C4:
        .thumb_func
Vector2C8:
        .thumb_func
Vector2CC:
        .thumb_func
Vector2D0:
        .thumb_func
Vector2D4:
        .thumb_func
Vector2D8:
        .thumb_func
Vector2DC:
#endif
#if CORTEX_NUM_VECTORS > 168
        .thumb_func
Vector2E0:
        .thumb_func
Vector2E4:
        .thumb_func
Vector2E8:
        .thumb_func
Vector2EC:
        .thumb_func
Vector2F0:
        .thumb_func
Vector2F4:
        .thumb_func
Vector2F8:
        .thumb_func
Vector2FC:
#endif
#if CORTEX_NUM_VECTORS > 176
        .thumb_func
Vector300:
        .thumb_func
Vector304:
        .thumb_func
Vector308:
        .thumb_func
Vector30C:
        .thumb_func
Vector310:
        .thumb_func
Vector314:
        .thumb_func
Vector318:
        .thumb_func
Vector31C:
#endif
#if CORTEX_NUM_VECTORS > 184
        .thumb_func
Vector320:
        .thumb_func
Vector324:
        .thumb_func
Vector328:
        .thumb_func
Vector32C:
        .thumb_func
Vector330:
        .thumb_func
Vector334:
        .thumb_func
Vector338:
        .thumb_func
Vector33C:
#endif
#if CORTEX_NUM_VECTORS > 192
        .thumb_func
Vector340:
        .thumb_func
Vector344:
        .thumb_func
Vector348:
        .thumb_func
Vector34C:
        .thumb_func
Vector350:
        .thumb_func
Vector354:
        .thumb_func
Vector358:
        .thumb_func
Vector35C:
#endif
#if CORTEX_NUM_VECTORS > 200
        .thumb_func
Vector360:
        .thumb_func
Vector364:
        .thumb_func
Vector368:
        .thumb_func
Vector36C:
        .thumb_func
Vector370:
        .thumb_func
Vector374:
        .thumb_func
Vector378:
        .thumb_func
Vector37C:
#endif
#if CORTEX_NUM_VECTORS > 208
        .thumb_func
Vector380:
        .thumb_func
Vector384:
        .thumb_func
Vector388:
        .thumb_func
Vector38C:
        .thumb_func
Vector390:
        .thumb_func
Vector394:
        .thumb_func
Vector398:
        .thumb_func
Vector39C:
#endif
#if CORTEX_NUM_VECTORS > 216
        .thumb_func
Vector3A0:
        .thumb_func
Vector3A4:
        .thumb_func
Vector3A8:
        .thumb_func
Vector3AC:
        .thumb_func
Vector3B0:
        .thumb_func
Vector3B4:
        .thumb_func
Vector3B8:
        .thumb_func
Vector3BC:
#endif
#if CORTEX_NUM_VECTORS > 224
        .thumb_func
Vector3C0:
        .thumb_func
Vector3C4:
        .thumb_func
Vector3C8:
        .thumb_func
Vector3CC:
        .thumb_func
Vector3D0:
        .thumb_func
Vector3D4:
        .thumb_func
Vector3D8:
        .thumb_func
Vector3DC:
#endif
#if CORTEX_NUM_VECTORS > 232
        .thumb_func
Vector3E0:
        .thumb_func
Vector3E4:
        .thumb_func
Vector3E8:
        .thumb_func
Vector3EC:
        .thumb_func
Vector3F0:
        .thumb_func
Vector3F4:
        .thumb_func
Vector3F8:
        .thumb_func
Vector3FC:
#endif
        bl          _unhandled_exception

        .thumb_func
        .weak       _unhandled_exception
_unhandled_exception:
.stay:
        b           .stay

#endif /* !defined(__DOXYGEN__) */

/** @} */
