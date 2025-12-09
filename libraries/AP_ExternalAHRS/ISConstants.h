/*
MIT LICENSE

Copyright (c) 2014-2025 Inertial Sense, Inc. - http://inertialsense.com

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files(the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions :

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#pragma once

#include "AP_ExternalAHRS_config.h"

#if AP_EXTERNAL_AHRS_INERTIALSENSE_ENABLED

#ifdef __cplusplus
extern "C" {
#endif

#undef _MATH_DEFINES_DEFINED
#define _MATH_DEFINES_DEFINED
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include <inttypes.h>
#include <time.h>
#include <limits.h>

#define ECEF2LLA_METHOD 5  // Method to compute LLA from ECEF position (0 through 5)

#define ENABLE_RTK_PROCESSING       1
#define RTK_ENGINE_NFREQ            NFREQ   // Set to 1 to disable L5 in RTK without changing NFREQ and data size
#define CONVERT_RAW_GPS_V2_TO_V1    0       // Allow conversion of raw GPS format from v1 to v2 used for RTK

#if defined(WIN32) || defined(__WIN32__) || defined(_WIN32)
    #define PLATFORM_IS_WINDOWS 1
    #define PLATFORM_IS_EMBEDDED 0
    #ifndef _CRT_SECURE_NO_DEPRECATE
    #define _CRT_SECURE_NO_DEPRECATE
    #endif

    // If you are getting winsock compile errors, make sure to include ISConstants.h as the first file in your header or c/cpp file
    #include <winsock2.h>
    #define _WINSOCKAPI_        // THIS shouldn't be necessary - winsock2.h should define it.
    #include <WS2tcpip.h>
    #define WIN32_LEAN_AND_MEAN
    #include <windows.h>

    // #define is_socket_t SOCKET
    typedef SOCKET is_socket_t;

    #define CPU_IS_LITTLE_ENDIAN (REG_DWORD == REG_DWORD_LITTLE_ENDIAN)
    #define CPU_IS_BIG_ENDIAN (REG_DWORD == REG_DWORD_BIG_ENDIAN)
    #define S_ISREG(m) (((m) & S_IFMT) == S_IFREG)
    #define S_ISDIR(m) (((m) & S_IFMT) == S_IFDIR)
#elif defined(__APPLE__)
    typedef int is_socket_t;

    #define PLATFORM_IS_APPLE 1
    #define PLATFORM_IS_EMBEDDED 0

    #if defined(__LITTLE_ENDIAN__)
        #define CPU_IS_LITTLE_ENDIAN 1
        #define CPU_IS_BIG_ENDIAN 0
    #elif defined(__BIG_ENDIAN__)
        #define CPU_IS_LITTLE_ENDIAN 0
        #define CPU_IS_BIG_ENDIAN 1
    #endif
#elif defined(__linux__) || defined(__unix__) || defined(__CYGWIN__)
    #include <endian.h>

    #ifndef __BYTE_ORDER
        #error __BYTE_ORDER not defined, must be __LITTLE_ENDIAN or __BIG_ENDIAN
    #endif

    #define PLATFORM_IS_LINUX 1
    #define PLATFORM_IS_EMBEDDED 0
    typedef int is_socket_t;
    #define CPU_IS_LITTLE_ENDIAN (__BYTE_ORDER == __LITTLE_ENDIAN)
    #define CPU_IS_BIG_ENDIAN (__BYTE_ORDER == __BIG_ENDIAN)
    // Detect if we're being compiled for ARM
    #ifdef __ARM_ARCH
        //#define STR_HELPER(x) #x
        //#define STR(x) STR_HELPER(x)
        //#pragma message "Compiling for ARMv" STR(__ARM_ARCH)
        #define CPU_IS_ARM 1                    // NOTE that is is different than "PLATFORM_IS_ARM" since this still applied is LINUX operating systems running on ARM, like Raspberry Pi, etc
        #if defined(__ARM_ARCH)                 // GCC compiler for ARM/version
            #define CPU_ARM_VERSION (__ARM_ARCH)
        #elif defined(__TARGET_ARCH_ARM)        // ARM compiler for ARM/version
            #define CPU_ARM_VERSION (__TARGET_ARCH_ARM)
        #endif
    #endif
#elif defined(__INERTIAL_SENSE_EVB_2__)
    #define PLATFORM_IS_EMBEDDED 1
    #define PLATFORM_IS_ARM 1
    #define PLATFORM_IS_EVB_2 1
    #define CPU_IS_LITTLE_ENDIAN 1
    #define CPU_IS_BIG_ENDIAN 0
#elif defined(ARM) || defined(__SAM3X8E__)
    #define PLATFORM_IS_EMBEDDED 1
    #define PLATFORM_IS_ARM 1
    #define CPU_IS_LITTLE_ENDIAN 1
    #define CPU_IS_BIG_ENDIAN 0
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__) || defined(__AVR_ATmega168__) || defined(__AVR_ATmega168P__) || defined(__AVR_ATmega328P__)
    #define PLATFORM_IS_EMBEDDED 1
    #define PLATFORM_IS_ARM 0
    #define CPU_IS_LITTLE_ENDIAN 1
    #define CPU_IS_BIG_ENDIAN 0
#elif defined(ARDUINO_SAMD_ZERO)
    #define PLATFORM_IS_EMBEDDED 1
    #define PLATFORM_IS_ARM 0
    #define CPU_IS_LITTLE_ENDIAN 1
    #define CPU_IS_BIG_ENDIAN 0
#elif defined(__MK20DX128__) || defined(__MK20DX256__) || defined(__MK64FX512__) || defined(__MK66FX1M0__)
    #define PLATFORM_IS_EMBEDDED 1
    #define PLATFORM_IS_ARM 1
    #define CPU_IS_LITTLE_ENDIAN 1
    #define CPU_IS_BIG_ENDIAN 0
#else
    // #error Unknown platform not supported, be sure to set it up here, defining CPU_IS_LITTLE_ENDIAN and CPU_IS_BIG_ENDIAN
    #define PLATFORM_IS_EMBEDDED 1
    #define PLATFORM_IS_ARM 0
    #define CPU_IS_LITTLE_ENDIAN 1
    #define CPU_IS_BIG_ENDIAN 0
#endif // platform defines

#if !defined(CPU_IS_LITTLE_ENDIAN) || !defined(CPU_IS_BIG_ENDIAN) || CPU_IS_LITTLE_ENDIAN == CPU_IS_BIG_ENDIAN
    #error Unsupported / unknown CPU architecture
#endif // Invalid CPU endianess


// "PLATFORM_IS_EMBEDDED" must be defined
#if !defined(PLATFORM_IS_EMBEDDED) 
    #error "Missing PLATFORM_IS_EMBEDDED macro!!!"
#endif


#if PLATFORM_IS_EMBEDDED
    extern void* pvPortMalloc(size_t xWantedSize);
    extern void vPortFree(void* pv);
    #define MALLOC(m) pvPortMalloc(m)
    #define REALLOC(m, size) 0 // not supported
    #define FREE(m) vPortFree(m)
#else
    #define MALLOC(m) malloc(m)
    #define REALLOC(m, size) realloc(m, size)
    #define FREE(m) free(m)
#endif 

#if defined(__ZEPHYR__)
    // #define SNPRINTF snprintfcb
    #define SNPRINTF snprintf
    #define VSNPRINTF vsnprintf
#elif PLATFORM_IS_EMBEDDED
    #ifndef SAMD_ZERO
        #ifndef ARDUINO_SAMD_ZERO
            // #include "printf.h" // Use embedded-safe SNPRINTF
        #endif
    #endif
    #define SNPRINTF snprintf_
    #define VSNPRINTF vsnprintf_
#else
    #define SNPRINTF snprintf
    #define VSNPRINTF vsnprintf
#endif


#if defined(_MSC_VER)
    #ifndef INLINE
#define INLINE __inline 
#endif

#ifndef SSCANF
#define SSCANF(src, fmt, ...) sscanf_s(src, fmt, __VA_ARGS__);
#endif

#ifndef STRNCPY
#define STRNCPY(dst, src, maxlen) strncpy_s(dst, maxlen, src, maxlen);
#endif

#define strncasecmp _strnicmp 
#else
    #ifndef INLINE
        #define INLINE inline
    #endif

    #ifndef SSCANF
        #define SSCANF sscanf
    #endif

    #ifndef STRNCPY
        #define STRNCPY(dst, src, maxlen) strncpy((char*)(dst), (char*)(src), (maxlen))
    #endif
#endif // defined(_MSC_VER)

#if defined(__ZEPHYR__)
    #include <zephyr/irq.h>
    #define BEGIN_CRITICAL_SECTION  irq_lock();
    #define END_CRITICAL_SECTION    irq_unlock(0);
#elif !PLATFORM_IS_EMBEDDED
    #define BEGIN_CRITICAL_SECTION
    #define END_CRITICAL_SECTION
    #define DBGPIO_ENABLE(pin)
    #define DBGPIO_TOGGLE(pin)
#endif

#if defined(PLATFORM_IS_EVB_2)
    #define _MKDIR(dir) f_mkdir(dir)
    #define _RMDIR(dir) f_unlink(dir)
    #define _GETCWD(buf, len) f_getcwd(buf, len)
#elif !PLATFORM_IS_EMBEDDED
    #if defined(PLATFORM_IS_WINDOWS)
        #include <direct.h>
        #include <sys/utime.h>
        #define _MKDIR(dir) _mkdir(dir)
        #define _RMDIR(dir) _rmdir(dir)
        #define _GETCWD(buf, len) _getcwd((buf), (len))
        #define _UTIME _utime
        #define _UTIMEBUF struct _utimbuf
    #else // POSIX
        #include <unistd.h>
        #include <dirent.h>
        #include <errno.h>
        #include <utime.h>
        #include <sys/stat.h>
        //#define _MKDIR(dir) mkdir(dir, S_IRWXU) // 777 owner access only
        #define _MKDIR(dir) mkdir((dir), ACCESSPERMS) // 0777 access for all
        #define _RMDIR(dir) rmdir((dir))
        #define _GETCWD(buf, len) getcwd((buf), (len))
        #define _UTIME utime
        #define _UTIMEBUF struct utimbuf
    #endif
#endif

// with this you can tell the compiler not to insert padding
#if defined(_MSC_VER)
    #define PUSH_PACK_1 __pragma(pack(push, 1))
    #define PUSH_PACK_4 __pragma(pack(push, 4))
    #define PUSH_PACK_8 __pragma(pack(push, 8))
    #define POP_PACK __pragma(pack(pop))
    #undef PACKED
    #define PACKED
#else
    #define PUSH_PACK_1 _Pragma("pack(push, 1)")
    #define PUSH_PACK_4 _Pragma("pack(push, 4)")
    #define PUSH_PACK_8 _Pragma("pack(push, 8)")
    #define POP_PACK _Pragma("pack(pop)")
    #undef PACKED
    #define PACKED
#endif

#define NO_FUNC_OPTIMIZATION __attribute__((optimize(0)))       // Place this before the function name, i.e. void NO_FUNC_OPTIMIZATION my_func(){ ... }

// #define PACKED_STRUCT typedef struct PACKED
// #define PACKED_UNION typedef union PACKED

#ifndef RAMFUNC

/* Define RAMFUNC attribute */
#if defined   (__CC_ARM ) /* Keil uVision 4 */
#   define RAMFUNC __attribute__ ((section(".ramfunc")))
#elif defined (__ICCARM__) /* IAR Ewarm 5.41+ */
#   define RAMFUNC __ramfunc
#elif defined (__GNUC__) /* GCC CS3 2009q3-68 */
#   define RAMFUNC __attribute__ ((section(".ramfunc")))
#else
#    define RAMFUNC
#endif

#endif

#ifndef UNMASK
#define UNMASK(_word, _prefix) (((_word) & (_prefix##_MASK)) >> (_prefix##_SHIFT))
#endif

#ifndef MASK
#define MASK(_prefix, _val) ((_prefix##_MASK) & ((_val) << (_prefix##_SHIFT)))
#endif

#ifndef SWAP16
#define SWAP16(v) ((uint16_t)(((uint16_t)(v) >> 8) | ((uint16_t)(v) << 8)))
#endif

#ifndef SWAP32
#if defined(__GNUC__)
#define SWAP32 __builtin_bswap32
#define SWAP64 __builtin_bswap64
#elif defined(__ICCAVR32__)
#define SWAP32 __swap_bytes
#elif defined(_MSC_VER)
#include "intrin.h"
#define SWAP32 _byteswap_ulong
#define SWAP64 _byteswap_uint64
#endif
#endif

#ifndef SWAP32
#define SWAP32(v) ((uint32_t)(((uint32_t)SWAP16((uint32_t)(v) >> 16)) | ((uint32_t)SWAP16((uint32_t)(v)) << 16)))
#endif

#ifndef SWAP64
#define SWAP64(v) ((uint64_t)(((uint64_t)SWAP32((uint64_t)(v) >> 32)) | ((uint64_t)SWAP32((uint64_t)(v)) << 32)))
#endif

#ifndef _ABS
#define _ABS(a) ((a) < 0 ? -(a) : (a))
#endif

#ifndef _MAX 
#define _MAX(a,b) (((a) > (b)) ? (a) : (b))
#endif

#ifndef _MIN
#define _MIN(a,b) (((a) < (b)) ? (a) : (b))
#endif

#ifndef _SQR
#define _SQR(a) ((a) * (a))
#endif

#ifndef _CLAMP
#define _CLAMP(v, minV, maxV) ((v) < (minV) ? (minV) : ((v) > (maxV) ? (maxV) : (v)))
#endif

#ifndef _SATURATE
#define _SATURATE(v) _CLAMP((v), 0, 1)
#endif

#ifndef _LIMIT
#define _LIMIT(x, lim) {if (!((x)>(-lim))){(x)=(-lim);}else{if (!((x)<(lim))){(x)=(lim);}}} // Works w/ NAN
#endif

#ifndef _LIMIT2
#define _LIMIT2(x, xmin, xmax) { if ((x) < (xmin)) { (x) = (xmin); } else { if ((x) > (xmax)) { (x) = (xmax); } } }
#endif

#ifndef _ROUND_NEAREST
#define _ROUND_NEAREST(number, multiple) ((((number) + ((multiple)/2)) / (multiple)) * (multiple))
#endif

#ifndef _ROUND_CLOSEST
#define _ROUND_CLOSEST(number, multiple) (((number) + ((multiple)/2)) / (multiple))
#endif

#ifndef _ROUNDUP
#define _ROUNDUP(numToRound, multiple) ((((numToRound) + (multiple) - 1) / (multiple)) * (multiple))
#endif

#ifndef _ISNAN
#define _ISNAN(a) ((a)!=(a))
#endif

#ifndef _ARRAY_BYTE_COUNT
#define _ARRAY_BYTE_COUNT(a) sizeof(a)
#endif

#ifndef _ARRAY_ELEMENT_COUNT
#define _ARRAY_ELEMENT_COUNT(a) (sizeof(a) / sizeof(a[0]))
#endif

#ifndef _MEMBER_ARRAY_ELEMENT_COUNT
#define _MEMBER_ARRAY_ELEMENT_COUNT(type, member) _ARRAY_ELEMENT_COUNT(((type*)0)->member)
#endif

#ifndef MEMBERSIZE
#define MEMBERSIZE(type, member) sizeof(((type*)0)->member)
#endif

#ifndef MEMBER_ITEM_SIZE
#define MEMBER_ITEM_SIZE(type, member) sizeof(((type*)NULL)->member[0])
#endif

/* equivalent to `offsetof(type, member[i])`, but does not require that `i` is a constant expression.*/
#ifndef OFFSET_OF_MEMBER_INDEX
#define OFFSET_OF_MEMBER_INDEX(type, member, i) (offsetof(type, member) + (i) * MEMBER_ITEM_SIZE(type, member))
#endif

/* equivalent to `offsetof(type, member[i].submember)`, but does not require that `i` is a constant expression. */
#ifndef OFFSET_OF_MEMBER_INDEX_SUBMEMBER
#define OFFSET_OF_MEMBER_INDEX_SUBMEMBER(type, member, i, submember) (offsetof(type, member[0].submember) + (i) * MEMBER_ITEM_SIZE(type, member))
#endif

#ifndef __ZEPHYR__
#ifndef STRINGIFY
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#endif
#endif

#define ATanH(x)        (0.5 * log((1 + (x)) / (1 - (x))))

#if defined(__cplusplus)

#define PURE_VIRTUAL = 0

#endif

#if ((defined(_MSC_VER) && _MSC_VER >= 1900) || (defined(__cplusplus) && ((__cplusplus >= 201103L || (__cplusplus < 200000 && __cplusplus > 199711L)))))

#ifndef CPP11_IS_ENABLED
#define CPP11_IS_ENABLED 1
#endif
#ifndef CONST_EXPRESSION
#define CONST_EXPRESSION constexpr static
#endif
#ifndef STATIC_ASSERT
#define STATIC_ASSERT(exp) static_assert((exp), #exp)
#endif
#ifndef OVERRIDE
#define OVERRIDE override
#endif
#ifndef NULLPTR
#define NULLPTR nullptr
#endif

#else

// #if defined(__GNUC__)
// #pragma GCC diagnostic push
// #pragma GCC diagnostic ignored "-Wunused-local-typedefs"
// #endif

#ifndef CPP11_IS_ENABLED
#define CPP11_IS_ENABLED 0
#endif
#ifndef CONST_EXPRESSION
#define CONST_EXPRESSION static const
#endif
#define PRE_PROC_COMBINE(X, Y) X##Y
#ifndef STATIC_ASSERT
// #define STATIC_ASSERT_MSG(exp, msg) typedef char PRE_PROC_COMBINE(msg, __LINE__)[(exp) ? 1 : -1]
#ifdef __cplusplus
#define STATIC_ASSERT(exp) static_assert(exp, #exp)
#else
#define STATIC_ASSERT(exp) _Static_assert(exp, #exp)
#endif
#endif
#ifndef OVERRIDE
#define OVERRIDE
#endif
#ifndef NULLPTR
#define NULLPTR 0
#endif

// #if defined(__GNUC__)
// #pragma GCC diagnostic pop
// #endif

#endif


//////////////////////////////////////////////////////////////////////////
// Time
// diff two times as uint32_t, handling wrap-around
#define UINT32_TIME_DIFF(current, prev) ((uint32_t)(current) - (uint32_t)(prev))

#define DT_TO_HZ(dt)                (((dt) == (0.0)) ? (0) : (1.0/(dt)))

#define C_WEEKS_TO_SECONDS          604800.0                            // Seconds per week
#define C_WEEKS_TO_MILLISECONDS     604800000                           // Milliseconds per week

#define MS2SEC_D(ms)                ((static_cast<double>(ms))*1.0E-3)  // Convert milliseconds to seconds

//////////////////////////////////////////////////////////////////////////
// Distance

#define C_IN2M                      0.0254                              // inches to meters
#define C_FT2M                      0.3048                              // (C_IN2M*12) feet to meters
#define C_M2FT                      3.2808398950131233595800524934383   // (1.0/C_FT2M)

#define C_YD2M                      0.9144                              // (C_FT2M*3) yards to meters

#define C_IN2M_F                    0.0254f                             // inches to meters
#define C_FT2M_F                    0.3048f                             // (C_IN2M*12) feet to meters
#define C_M2FT_F                    3.2808398950131233595800524934383f  // (1.0/C_FT2M)
#define C_YD2M_F                    0.9144f                             // (C_FT2M*3) yards to meters

#define C_NMI2M                     1852.0                              // nautical miles to meters
#define C_MI2M                      1609.344                            // (C_FT2M*5280) miles to meters

#define C_NMI2M_F                   1852.0f                             // nautical miles to meters
#define C_MI2M_F                    1609.344f                           // (C_FT2M*5280) miles to meters

#define C_METERS_KNOTS              1.943844                            // Meters/sec squared to knots
#define C_METERS_KNOTS_F            1.943844f                           // Meters/sec squared to knots

#define C_KNOTS_METERS_F            0.514444444f                        // Knots to meters/sec

//////////////////////////////////////////////////////////////////////////
// Acceleration / Force

// Local gravity available as = gravity_igf80(lattitude_rad, altitude_m);
#if 1

// Standard Gravity (at sea level)
#define C_G_TO_MPS2     9.80665             // (m/s^2) standard gravity
#define C_MPS2_TO_G     0.101971621297793
#define C_G_TO_FTPS2    32.17404856         // (ft/s^2) standard gravity

#define C_G_TO_MPS2_F   9.80665f            // (m/s^2) standard gravity
#define C_MPS2_TO_G_F   0.101971621297793f
#define C_G_TO_FTPS2_F  32.17404856f        // (ft/s^2) standard gravity
#endif

//////////////////////////////////////////////////////////////////////////
// Pressure

// Standard Atmospheric Pressure (at sea level)
                                                    // standard atmospheric pressure
#define C_P0N_M2        1.01325e5               // p0 in N/m^2
#define C_P0N_M2_F      1.01325e5f              // p0 in N/m^2
#define C_P0            14.692125               // (C_P0N_M2*1.450e-4)
#define C_P0_F          14.692125f              // (C_P0N_M2*1.450e-4)

#define C_P0_PA_F       101325f                 // (Pa) Sea level standard atmospheric pressure
#define C_P0_KPA_F      101.325f                // (kPa) Sea level standard atmospheric pressure
#define C_P0_MBAR_F     1013.25f                // (mbar) Sea level standard atmospheric pressure

#define C_INV_P0_KPA_F  9.8692326671601283e-3f  // (1/kPa) Inverse of Sea level standard atmospheric pressure

#define C_MBAR_TO_KPA_F 0.1f                    // (kpa/mbar) absolute pressure
#define C_KPA_TO_MBAR_F 10.0f                   // (mbar/kpa) absolute pressure
#define C_MBAR_TO_PA_F  100.0f                  // (kpa/mbar) absolute pressure

//////////////////////////////////////////////////////////////////////////
// Temperature

#define C_T0_F          288.15f                 // (K) Sea level standard temperature

//////////////////////////////////////////////////////////////////////////
// Mass

#define C_LBM2KG        0.45359237              // lb mass
#define C_KG2LBM        2.204622622             // (1.0/C_LBM2KG)
#define C_LBF2N         4.448221615             // (C_LBM2KG*C_G0MPERSEC2) lb force
#define C_SLUG2KG       14.59390294             // (C_LBM2KG*C_G0) slugs

#define C_LBM2KG_F      0.45359237f             // lb mass
#define C_KG2LBM_F      2.204622622f            // (1.0/C_LBM2KG)
#define C_LBF2N_F       4.448221615f            // (C_LBM2KG*C_G0MPERSEC2) lb force
#define C_SLUG2KG_F     14.59390294f            // (C_LBM2KG*C_G0) slugs

// Math constants 
// from CRC Standard Mathematical Tables, 27th edition, 1984 

#define C_ROOT2         1.41421356237309504880168872420969807856967187537695
#define C_ROOT3         1.73205080756887729352744634150587236694280525381039
#define C_E             2.71828182845904523536028747135266249775724709369996
#define C_DIVROOT2      0.70710678118654752440084436210485

#define C_ROOT2_F       1.41421356237309504880168872420969807856967187537695f
#define C_ROOT3_F       1.73205080756887729352744634150587236694280525381039f
#define C_E_F           2.71828182845904523536028747135266249775724709369996f
#define C_DIVROOT_F     0.70710678118654752440084436210485f

#define C_PIDIV16       0.19634954084936207740391521145497                      // 11.25 deg
#define C_PIDIV8        0.39269908169872415480783042290994                      // 22.5 deg
#define C_PIDIV4        0.78539816339744830961566084581988
#define C_PIDIV2        1.5707963267948966192313216916398
#define C_PI            3.14159265358979323846264338327950288419716939937511
#define C_TWOPI         6.283185307179586476925286766559
#define C_DIVTWOPI      0.15915494309189533576888376337251
#define C_DIVPI         0.31830988618379067153776752674503

#define C_PIDIV16_F     0.19634954084936207740391521145497f                     // 11.25 deg
#define C_PIDIV8_F      0.39269908169872415480783042290994f                     // 22.5 deg
#define C_PIDIV4_F      0.78539816339744830961566084581988f                     // 45 deg
#define C_PIDIV2_F      1.5707963267948966192313216916398f                      // 90 deg
#define C_PI_F          3.14159265358979323846264338327950288419716939937511f   // 180 deg
#define C_TWOPI_F       6.283185307179586476925286766559f                       // 360 deg
#define C_DIVTWOPI_F    0.15915494309189533576888376337251f
#define C_DIVPI_F       0.31830988618379067153776752674503f

#define C_DEG2RAD       0.017453292519943295769236907684886
#define C_DEG2RAD_F     0.017453292519943295769236907684886f
#define C_RAD2DEG       57.295779513082320876798154814105
#define C_RAD2DEG_F     57.295779513082320876798154814105f

#define C_KMPH2MPS      0.277777777777777777
#define C_KMPH2MPS_F    0.277777777777777777f
#define C_MPS2KMPH      3.6
#define C_MPS2KMPH_F    3.6f

#define C_MM2M              0.001
#define C_MM2M_F            0.001f
#define C_M2MM              1000.0
#define C_M2MM_F            1000.0f

#define C_CM2M              0.01
#define C_CM2M_F            0.01f

#define C_IN2M              0.0254
#define C_IN2M_F            0.0254f
#define C_M2IN              39.37007874
#define C_M2IN_F            39.37007874f

#define C_IN2MM             25.4
#define C_IN2MM_F           25.4f
#define C_MM2IN             0.03937007874
#define C_MM2IN_F           0.03937007874f

#define C_FPS2KT            0.5924838
#define C_FPS2KT_F          0.5924838f
#define C_KT2FPS            1.68780986
#define C_KT2FPS_F          1.68780986f

#define C_SQIN2SQFT         0.00694444444444444444444
#define C_SQIN2SQFT_F       0.00694444444444444444444f
#define C_SQFT2SQIN         144.0
#define C_SQFT2SQIN_F       144.0f

#define C_GPM2CFS           0.0022280093
#define C_GPM2CFS_F         0.0022280093f
#define C_CFS2GPM           448.83117
#define C_CFS2GPM_F         448.83117f

#define C_DEGF0_R           459.69
#define C_DEGC0_T           273.16
#define C_DEGC0_DEGF        32.0
#define C_DEGF_PER_DEGC     1.8

#define C_LN_2              0.69314718055994530941723212145818
#define C_LN_2_F            0.69314718055994530941723212145818f

#define C_C2K               273.16
#define C_C2K_F             273.16f
#define C_F2R               459.69
#define C_F2R_F             459.69f

#define C_G_CONST           1.068944098e-09     // 6.6732e-11*CUBE(C_M2FT)/C_KG2LBM
#define C_EARTH_MASS        1.317041554e25      // 5.974e24*C_KG2LBM
#define C_N0_AVOGADRO       6.02205e23
#define C_R_IDEAL_SU        8.31434
#define C_K_BOLTZMANN       1.380622e-23
#define C_C_LIGHT           983571194.2         // 2.9979250e+8*C_M2FT
#define C_ECHARGE           1.6021917e-19

#define C_DOFA              0.080719353         // 1.293*C_KG2LBM/CUBE(C_M2FT)
#define C_DOFH2O            62.427960576        // 1.000e3*C_KG2LBM/CUBE(C_M2FT)
#define C_STOFH2O           75.6
#define C_VOFH2O            1.787e-3
#define C_SOUND0VEL         1087.598425         // 331.5*C_M2FT
#define C_SOUND20VEL        1126.64042          // 343.4*C_M2FT

#define C_G_CONST_F         1.068944098e-09f    // 6.6732e-11*CUBE(C_M2FT)/C_KG2LBM
#define C_EARTH_MASS_F      1.317041554e25f     // 5.974e24*C_KG2LBM
#define C_N0_AVOGADRO_F     6.02205e23f
#define C_R_IDEAL_SU_F      8.31434f
#define C_K_BOLTZMANN_F     1.380622e-23f
#define C_C_LIGHT_F         983571194.2f        // 2.9979250e+8*C_M2FT
#define C_ECHARGE_F         1.6021917e-19f

#define C_DOFA_F            0.080719353f        // 1.293*C_KG2LBM/CUBE(C_M2FT)
#define C_DOFH2O_F          62.427960576f       // 1.000e3*C_KG2LBM/CUBE(C_M2FT)
#define C_STOFH2O_F         75.6f
#define C_VOFH2O_F          1.787e-3f
#define C_SOUND0VEL_F       1087.598425f        // 331.5*C_M2FT
#define C_SOUND20VEL_F      1126.64042f         // 343.4*C_M2FT

#define C_WGS84_a           6378137.0          // WGS-84 semimajor axis (m)
#define C_WGS84_a_F         6378137.0f
#define C_WGS84_b           6356752.3142       // WGS-84 semiminor axis (m)
#define C_WGS84_b_F         6356752.3142f
#define C_WIE               7.2321151467e-05   // WGS-84 earth rotation rate (rad/s)
#define C_WIE_F             7.2321151467e-05f

#define C_TESLA2GAUSS_F     10000.0
#define C_UTESLA2GAUSS_F    0.01

#define C_0p1_DEG2RAD_F     0.00174532925199433f        // = 0.1f * C_0pDEG2RAD_F
#define C_0p11_DEG2RAD_F    0.00191986217719376f
#define C_0p12_DEG2RAD_F    0.00209439510239320f
#define C_0p13_DEG2RAD_F    0.00226892802759263f
#define C_0p14_DEG2RAD_F    0.00244346095279206f
#define C_0p15_DEG2RAD_F    0.00261799387799149f
#define C_0p16_DEG2RAD_F    0.00279252680319093f
#define C_0p17_DEG2RAD_F    0.00296705972839036f
#define C_0p18_DEG2RAD_F    0.00314159265358979f
#define C_0p19_DEG2RAD_F    0.00331612557878923f
#define C_0p2_DEG2RAD_F     0.00349065850398866f
#define C_0p21_DEG2RAD_F    0.00366519142918809f
#define C_0p22_DEG2RAD_F    0.00383972435438753f
#define C_0p23_DEG2RAD_F    0.00401425727958696f
#define C_0p24_DEG2RAD_F    0.00418879020478639f
#define C_0p25_DEG2RAD_F    0.00436332312998582f
#define C_0p26_DEG2RAD_F    0.00453785605518526f
#define C_0p27_DEG2RAD_F    0.00471238898038469f
#define C_0p28_DEG2RAD_F    0.00488692190558412f
#define C_0p29_DEG2RAD_F    0.00506145483078356f
#define C_0p3_DEG2RAD_F     0.00523598775598299f
#define C_0p31_DEG2RAD_F    0.00541052068118242f
#define C_0p32_DEG2RAD_F    0.00558505360638185f
#define C_0p33_DEG2RAD_F    0.00575958653158129f
#define C_0p34_DEG2RAD_F    0.00593411945678072f
#define C_0p35_DEG2RAD_F    0.00610865238198015f
#define C_0p36_DEG2RAD_F    0.00628318530717959f
#define C_0p37_DEG2RAD_F    0.00645771823237902f
#define C_0p38_DEG2RAD_F    0.00663225115757845f
#define C_0p39_DEG2RAD_F    0.00680678408277788f
#define C_0p4_DEG2RAD_F     0.00698131700797732f
#define C_0p41_DEG2RAD_F    0.00715584993317675f
#define C_0p42_DEG2RAD_F    0.00733038285837618f
#define C_0p43_DEG2RAD_F    0.00750491578357562f
#define C_0p44_DEG2RAD_F    0.00767944870877505f
#define C_0p45_DEG2RAD_F    0.00785398163397448f
#define C_0p46_DEG2RAD_F    0.00802851455917391f
#define C_0p47_DEG2RAD_F    0.00820304748437335f
#define C_0p48_DEG2RAD_F    0.00837758040957278f
#define C_0p49_DEG2RAD_F    0.00855211333477222f
#define C_0p5_DEG2RAD_F     0.00872664625997165f        // = 5.0f * C_0pDEG2RAD_F
#define C_0p6_DEG2RAD_F     0.01047197551196600f
#define C_0p7_DEG2RAD_F     0.01221730476396030f
#define C_0p8_DEG2RAD_F     0.01396263401595460f
#define C_0p9_DEG2RAD_F     0.01570796326794900f
#define C_1p0_DEG2RAD_F     0.0174532925199433f        // = 1.0f * C_DEG2RAD_F
#define C_1p1_DEG2RAD_F     0.0191986217719376f
#define C_1p2_DEG2RAD_F     0.0209439510239320f
#define C_1p3_DEG2RAD_F     0.0226892802759263f
#define C_1p4_DEG2RAD_F     0.0244346095279206f
#define C_1p5_DEG2RAD_F     0.0261799387799149f
#define C_1p6_DEG2RAD_F     0.0279252680319093f
#define C_1p7_DEG2RAD_F     0.0296705972839036f
#define C_1p8_DEG2RAD_F     0.0314159265358979f
#define C_1p9_DEG2RAD_F     0.0331612557878923f
#define C_2p0_DEG2RAD_F     0.0349065850398866f
#define C_2p1_DEG2RAD_F     0.0366519142918809f
#define C_2p2_DEG2RAD_F     0.0383972435438753f
#define C_2p3_DEG2RAD_F     0.0401425727958696f
#define C_2p4_DEG2RAD_F     0.0418879020478639f
#define C_2p5_DEG2RAD_F     0.0436332312998582f
#define C_2p6_DEG2RAD_F     0.0453785605518526f
#define C_2p7_DEG2RAD_F     0.0471238898038469f
#define C_2p8_DEG2RAD_F     0.0488692190558412f
#define C_2p9_DEG2RAD_F     0.0506145483078356f
#define C_3p0_DEG2RAD_F     0.0523598775598299f
#define C_3p1_DEG2RAD_F     0.0541052068118242f
#define C_3p2_DEG2RAD_F     0.0558505360638185f
#define C_3p3_DEG2RAD_F     0.0575958653158129f
#define C_3p4_DEG2RAD_F     0.0593411945678072f
#define C_3p5_DEG2RAD_F     0.0610865238198015f
#define C_3p6_DEG2RAD_F     0.0628318530717959f
#define C_3p7_DEG2RAD_F     0.0645771823237902f
#define C_3p8_DEG2RAD_F     0.0663225115757845f
#define C_3p9_DEG2RAD_F     0.0680678408277788f
#define C_4p0_DEG2RAD_F     0.0698131700797732f
#define C_4p1_DEG2RAD_F     0.0715584993317675f
#define C_4p2_DEG2RAD_F     0.0733038285837618f
#define C_4p3_DEG2RAD_F     0.0750491578357562f
#define C_4p4_DEG2RAD_F     0.0767944870877505f
#define C_4p5_DEG2RAD_F     0.0785398163397448f
#define C_4p6_DEG2RAD_F     0.0802851455917391f
#define C_4p7_DEG2RAD_F     0.0820304748437335f
#define C_4p8_DEG2RAD_F     0.0837758040957278f
#define C_4p9_DEG2RAD_F     0.0855211333477222f
#define C_5p0_DEG2RAD_F     0.0872664625997165f        // = 5.0f * C_DEG2RAD_F
#define C_6p0_DEG2RAD_F     0.1047197551196600f
#define C_7p0_DEG2RAD_F     0.1221730476396030f
#define C_8p0_DEG2RAD_F     0.1396263401595460f
#define C_9p0_DEG2RAD_F     0.1570796326794900f
#define C_10p0_DEG2RAD_F    0.1745329251994330f
#define C_15p0_DEG2RAD_F    0.2617993877991490f
#define C_20p0_DEG2RAD_F    0.3490658503988660f        // = 20.0f * C_DEG2RAD_F
#define C_25p0_DEG2RAD_F    0.4363323129985820f
#define C_30p0_DEG2RAD_F    0.5235987755982990f
#define C_35p0_DEG2RAD_F    0.6108652381980150f
#define C_40p0_DEG2RAD_F    0.6981317007977320f
#define C_45p0_DEG2RAD_F    0.7853981633974480f
#define C_50p0_DEG2RAD_F    0.8726646259971650f
#define C_55p0_DEG2RAD_F    0.9599310885968810f
#define C_60p0_DEG2RAD_F    1.0471975511966000f
#define C_65p0_DEG2RAD_F    1.1344640137963100f
#define C_70p0_DEG2RAD_F    1.2217304763960300f
#define C_75p0_DEG2RAD_F    1.3089969389957500f
#define C_80p0_DEG2RAD_F    1.3962634015954600f
#define C_85p0_DEG2RAD_F    1.4835298641951800f
#define C_90p0_DEG2RAD_F    1.5707963267949000f
#define C_135p0_DEG2RAD_F   2.3561944901923400f
#define C_180p0_DEG2RAD_F   3.1415926535897900f

// Angle Unwrap
#define UNWRAP_DEG_F64(x)           { if ((x) < (-180.0))  { (x) += (360.0);     } if ((x) > (180.0))     { (x) -= (360.0);     } }    // unwrap to +- 180
#define UNWRAP_DEG_F32(x)           { if ((x) < (-180.0f)) { (x) += (360.0f);    } if ((x) > (180.0f))    { (x) -= (360.0f);    } }    // unwrap to +- 180
#define UNWRAP_90DEG_F64(x)         { if ((x) < (-90.0))   { (x) += (180.0);     } if ((x) > (90.0))      { (x) -= (180.0);     } }    // unwrap to +- 90
#define UNWRAP_90DEG_F32(x)         { if ((x) < (-90.0f))  { (x) += (180.0f);    } if ((x) > (90.0f))     { (x) -= (180.0f);    } }    // unwrap to +- 90
#define UNWRAP_F64(x)               { if ((x) < (-C_PI))   { (x) += (C_TWOPI);   } if ((x) > (C_PI))      { (x) -= (C_TWOPI);   } }    // unwrap to +- PI
#define UNWRAP_F32(x)               { if ((x) < (-C_PI_F)) { (x) += (C_TWOPI_F); } if ((x) > (C_PI_F))    { (x) -= (C_TWOPI_F); } }    // unwrap to +- PI
#define UNWRAP_ZERO_TWOPI_F64(x)    { if ((x) < (0.0))     { (x) += (C_TWOPI);   } if ((x) > (C_TWOPI))   { (x) -= (C_TWOPI);   } }    // unwrap to 0 to TWOPI
#define UNWRAP_ZERO_TWOPI_F32(x)    { if ((x) < (0.f))     { (x) += (C_TWOPI_F); } if ((x) > (C_TWOPI_F)) { (x) -= (C_TWOPI_F); } }    // unwrap to 0 to TWOPI
#define UNWRAP_PIDIV4_F32(x)        { while ((x) < (-C_PIDIV4_F)) { (x) += (C_PIDIV2_F); } while ((x) > (C_PIDIV4_F)) { (x) -= (C_PIDIV2_F); } }    // unwrap to +- PI/4 (45 degrees)

#define _SIN        sinf
#define _COS        cosf
#define _TAN        tanf
#define _ASIN       asinf
#define _ACOS       acosf
#define _ATAN2      atan2f
#define _SQRT       sqrtf
#define _FABS       fabsf
#define _LOG        logf

#define _DEG2RAD    C_DEG2RAD_F
#define _RAD2DEG    C_RAD2DEG_F
#define _ZERO        0.0f

#define FLOAT2DOUBLE (double) // Used to prevent warning when compiling with -Wdouble-promotion in Linux

#define REF_INS_SERIAL_NUMBER            99999                                        // 10101 was prior value

#define C_EARTH_RADIUS_EQUATORIAL       6378137.0       // (m) Equatorial radius
#define C_EARTH_RADIUS_POLAR            6356752.0       // (m) Polar radius
#define C_EARTH_RADIUS_MEAN             6371000.0       // (m) Mean radius (used by the IUGG, WGS-84, etc.)

// IMX Operational Limits
#define INS_MAX_VELOCITY                500.0f          // (m/s) INS operation limit - velocity.  Limited by GPS.
#define INS_MAX_LATITUDE                C_PIDIV2        // (rad) INS operation limit - latitude
#define INS_MAX_LATITUDE_DEG            90.0            // (deg) INS operation limit - latitude
#define INS_MAX_LONGITUDE               C_PI            // (rad) INS operation limit - longitude
#define INS_MAX_LONGITUDE_DEG           180.0           // (deg) INS operation limit - longitude
#define INS_MAX_ALTITUDE                50000.0         // (m)   INS operation limit - altitude.  Limited by GPS.  50 km = 164,042 ft, 15 km = 49,212 ft
#define INS_MAX_ECEF                    (INS_MAX_ALTITUDE+C_EARTH_RADIUS_EQUATORIAL)     // (m)   INS operation limit - ECEF

#define C_GPS_LEAP_SECONDS              18

typedef float       f_t;
typedef int            i_t;
typedef double      ixVector2d[2];        // V = | 0 1 |
typedef f_t         ixVector2[2];         // V = | 0 1 |
typedef double      ixVector3d[3];        // V = | 0 1 2 |
typedef f_t         ixVector3[3];         // V = | 0 1 2 |
typedef double      ixVector4d[4];        // V = | 0 1 2 3 |
typedef f_t         ixVector4[4];         // V = | 0 1 2 3 |
typedef f_t         ixVector5[5];         // V = | 0 1 2 3 4 |
typedef f_t         ixVector6[6];         // V = | 0 1 2 3 4 5 |
typedef ixVector4   ixQuat;                // w,x,y,z
typedef ixVector3   ixEuler;            // roll,pitch,yaw
typedef f_t         ixMatrix2[4];
typedef f_t         ixMatrix3[9];
typedef f_t         ixMatrix4[16];
typedef f_t         ixMatrix5[25];
typedef double      ixMatrix3d[9];

typedef enum {
    IS_OP_OK = 0,
    IS_OP_ERROR = -1,
    IS_OP_CANCELLED = -2,
    IS_OP_INCOMPATIBLE = -3,
    IS_OP_RETRY = -4,
    IS_OP_CLOSED = -5,
} is_operation_result;

PUSH_PACK_8

typedef struct
{
    /** time (s) expressed by standard time_t */
    int64_t time;

    /** fraction of second under 1 s */
    double sec;
} gtime_t;

POP_PACK

#ifdef __cplusplus
} // extern "C"
#endif

#endif // AP_EXTERNAL_AHRS_INERTIALSENSE_ENABLED
