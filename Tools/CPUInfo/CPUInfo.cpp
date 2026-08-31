/*
  test CPU speed
  Andrew Tridgell September 2011
*/

#define AP_MATH_ALLOW_DOUBLE_FUNCTIONS 1

#include <cmath>

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_Math/div1000.h>
#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include "EKF_Maths.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#if HAL_WITH_DSP
#include <arm_math.h>
#endif
#include <hrt.h>
#include <ch.h>
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <fenv.h>
#endif  // HAL_BOARD_CHIBIOS

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// On H750 we want to measure external flash to ram performance
#if defined(EXT_FLASH_SIZE_MB) && EXT_FLASH_SIZE_MB>0 && defined(STM32H7)
#include "ch.h"
#define DISABLE_CACHES
#endif

#ifdef STM32_SYS_CK
static uint32_t sysclk = STM32_SYS_CK;
#elif defined(STM32_SYSCLK)
static uint32_t sysclk = STM32_SYSCLK;
#else
static uint32_t sysclk = 0;
#endif

static EKF_Maths ekf;

HAL_Semaphore sem;
#if HAL_WITH_ESC_TELEM
AP_ESC_Telem telem;
#endif

void setup() {
#ifdef DISABLE_CACHES
#if !HAL_XIP_ENABLED // can't disable DCache in memory-mapped mode
    SCB_DisableDCache();
#endif
    SCB_DisableICache();
#endif
    ekf.init();
}

static void show_sizes(void)
{
    hal.console->printf("SYSCLK %uMHz\n", unsigned(sysclk/1000000U));

    hal.console->printf("Type sizes:\n");
    hal.console->printf("char      : %lu\n", (unsigned long)sizeof(char));
    hal.console->printf("short     : %lu\n", (unsigned long)sizeof(short));
    hal.console->printf("int       : %lu\n", (unsigned long)sizeof(int));
    hal.console->printf("long      : %lu\n", (unsigned long)sizeof(long));
    hal.console->printf("long long : %lu\n", (unsigned long)sizeof(long long));
    hal.console->printf("bool      : %lu\n", (unsigned long)sizeof(bool));
    hal.console->printf("void*     : %lu\n", (unsigned long)sizeof(void *));

    hal.console->printf("printing NaN: %f\n", (double)sqrtf(-1.0f));
    hal.console->printf("printing +Inf: %f\n", (double)(1.0f/0.0f));
    hal.console->printf("printing -Inf: %f\n", (double)(-1.0f/0.0f));
}

#define TENTIMES(x) do { x; x; x; x; x; x; x; x; x; x; } while (0)
#define FIFTYTIMES(x) do { TENTIMES(x); TENTIMES(x); TENTIMES(x); TENTIMES(x); TENTIMES(x); } while (0)

#define TIMEIT(name, op, count) do { \
    uint16_t us_end, us_start; \
    us_start = AP_HAL::micros16(); \
    for (uint8_t i = 0; i < count; i++) { \
        FIFTYTIMES(op); \
    } \
    us_end = AP_HAL::micros16(); \
    uint16_t dt_us = us_end - us_start; \
    hal.console->printf("%-10s %7.4f usec/call\n", name, double(dt_us) / double(count * 50.0)); \
    hal.scheduler->delay(10); \
} while (0)

volatile float v_f = 1.0;
volatile float v_out;
volatile double v_d = 1.0;
volatile double v_out_d;
volatile uint32_t v_32 = 1;
volatile uint32_t v_out_32 = 1;
volatile uint16_t v_16 = 1;
volatile uint16_t v_out_16 = 1;
volatile uint8_t v_8 = 1;
volatile uint8_t v_out_8 = 1;
volatile uint8_t mbuf1[128], mbuf2[128];
volatile uint64_t v_64 = 1;
volatile uint64_t v_out_64 = 1;

//Main loop where the action takes place
#if defined(__clang_major__)
// clang doesn't understand -Wframe-larger-than=
#else
#pragma GCC diagnostic error "-Wframe-larger-than=2000"
#endif
static void show_timings(void)
{

    v_f = 1+(AP_HAL::micros() % 5);
    v_out = 1+(AP_HAL::micros() % 3);

    v_32 = AP_HAL::millis();
    v_32 = 1+(AP_HAL::micros() % 5);
    v_out_32 = 1+(AP_HAL::micros() % 3);

    v_16 = 1+(AP_HAL::micros() % 5);
    v_out_16 = 1+(AP_HAL::micros() % 3);

    v_8 = 1+(AP_HAL::micros() % 5);
    v_out_8 = 1+(AP_HAL::micros() % 3);


    hal.console->printf("Operation timings:\n");
    hal.console->printf("Note: timings for some operations are very data dependent\n");

    TIMEIT("nop", asm volatile("nop"::), 255);

    TIMEIT("micros()", AP_HAL::micros(), 200);
    TIMEIT("micros16()", AP_HAL::micros16(), 200);
    TIMEIT("millis()", AP_HAL::millis(), 200);
    TIMEIT("millis16()", AP_HAL::millis16(), 200);
    TIMEIT("micros64()", AP_HAL::micros64(), 200);

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    TIMEIT("hrt_micros32()", hrt_micros32(), 200);
    TIMEIT("hrt_micros64()", hrt_micros64(), 200);
    TIMEIT("hrt_millis32()", hrt_millis32(), 200);
    TIMEIT("hrt_millis64()", hrt_millis64(), 200);
#endif
    
    TIMEIT("fadd", v_out += v_f, 100);
    TIMEIT("fsub", v_out -= v_f, 100);
    TIMEIT("fmul", v_out *= v_f, 100);
    TIMEIT("fdiv /=", v_out /= v_f, 100);
    TIMEIT("fdiv 2/x", v_out = 2.0f/v_f, 100);

    TIMEIT("dadd", v_out_d += v_d, 100);
    TIMEIT("dsub", v_out_d -= v_d, 100);
    TIMEIT("dmul", v_out_d *= v_d, 100);
    TIMEIT("ddiv", v_out_d /= v_d, 100);

    TIMEIT("sinf()", v_out = sinf(v_f), 100);
    TIMEIT("cosf()", v_out = cosf(v_f), 100);
    #if HAL_WITH_DSP && CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    TIMEIT("arm_sin_f32()", v_out = arm_sin_f32(v_f), 100);
    TIMEIT("arm_cos_f32()", v_out = arm_cos_f32(v_f), 100);
    #endif
    TIMEIT("tanf()", v_out = tanf(v_f), 100);
    TIMEIT("acosf()", v_out = acosf(v_f * 0.2), 100);
    TIMEIT("asinf()", v_out = asinf(v_f * 0.2), 100);
    TIMEIT("atan2f()", v_out = atan2f(v_f * 0.2, v_f * 0.3), 100);
    TIMEIT("sqrtf()",v_out = sqrtf(v_f), 100);

    TIMEIT("sin()", v_out = sin(v_f), 100);
    TIMEIT("cos()", v_out = cos(v_f), 100);
    TIMEIT("tan()", v_out = tan(v_f), 100);
    TIMEIT("acos()", v_out = acos(v_f * 0.2), 100);
    TIMEIT("asin()", v_out = asin(v_f * 0.2), 100);
    TIMEIT("atan2()", v_out = atan2(v_f * 0.2, v_f * 0.3), 100);
    TIMEIT("sqrt()",v_out = sqrt(v_f), 100);
    #if HAL_WITH_DSP && CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
	TIMEIT("arm_sqrt_f32()", arm_sqrt_f32(v_f, (float32_t*)&v_out), 100);
    #endif
    TIMEIT("sq()",v_out = sq(v_f), 100);
    TIMEIT("powf(v,2)",v_out = powf(v_f, 2), 100);
    TIMEIT("powf(v,3.1)",v_out = powf(v_f, 3.1), 100);
    TIMEIT("EKF",v_out = ekf.test(), 5);

    TIMEIT("iadd8", v_out_8 += v_8, 100);
    TIMEIT("isub8", v_out_8 -= v_8, 100);
    TIMEIT("imul8", v_out_8 *= v_8, 100);
    TIMEIT("idiv8", v_out_8 /= v_8, 100);

    TIMEIT("iadd16", v_out_16 += v_16, 100);
    TIMEIT("isub16", v_out_16 -= v_16, 100);
    TIMEIT("imul16", v_out_16 *= v_16, 100);
    TIMEIT("idiv16", v_out_16 /= v_16, 100);

    TIMEIT("iadd32", v_out_32 += v_32, 100);
    TIMEIT("isub32", v_out_32 -= v_32, 100);
    TIMEIT("imul32", v_out_32 *= v_32, 100);
    TIMEIT("idiv32", v_out_32 /= v_32, 100);

    TIMEIT("iadd64", v_out_64 += v_64, 100);
    TIMEIT("isub64", v_out_64 -= v_64, 100);
    TIMEIT("imul64", v_out_64 *= v_64, 100);
    TIMEIT("idiv64", v_out_64 /= v_64, 100);

    TIMEIT("memcpy128", memcpy((void*)mbuf1, (const void *)mbuf2, sizeof(mbuf1)); v_out_8 += mbuf1[0], 200);
    TIMEIT("memset128", memset((void*)mbuf1, 1, sizeof(mbuf1)); v_out_8 += mbuf1[0], 200);
    TIMEIT("delay(1)", hal.scheduler->delay(1), 5);

    TIMEIT("SEM", { WITH_SEMAPHORE(sem); v_out_32 += v_32;}, 100);
}

static void div1000_check(uint64_t v)
{
    const uint64_t v1 = v / 1000ULL;
    const uint64_t v2 = uint64_div1000(v);
    if (v1 != v2) {
        AP_HAL::panic("ERROR: 0x%llx v1=0x%llx v2=0x%llx",
                      (unsigned long long)v, (unsigned long long)v1, (unsigned long long)v2);
    }
}

/*
  Uniform random 64-bit values are all enormous - millions of draws
  produce nothing below 2^35 - so on their own they never exercise the
  range this board's clock actually runs in, which stays under 2^35
  for the first 9.5 hours of uptime.  Cover that range and the
  algorithm's boundaries explicitly before the random sweep below.
  MathTest.div1000_structured makes the same argument at length.
 */
static void test_div1000_structured(void)
{
    // the sub-second range, densely: covers zero and every
    // 1000-boundary and pre-shift window within it
    for (uint64_t v = 0; v < 200000ULL; v++) {
        div1000_check(v);
    }

    // top of the range
    for (uint64_t i = 0; i < 20000ULL; i++) {
        div1000_check(UINT64_MAX - i);
    }

    // powers of two and their neighbourhoods: 2^32 is where a_lo
    // overflows, 2^35 where a_hi stops being zero
    for (uint8_t bit = 0; bit < 64; bit++) {
        const uint64_t p = 1ULL << bit;
        for (int8_t d = -9; d <= 9; d++) {
            if (d < 0 && p < (uint64_t)(-d)) {
                continue;
            }
            div1000_check(p + d);
        }
    }

    // instants this board's microsecond clock passes through
    static const uint64_t instants[] = {
        1000ULL,              // 1 ms
        1000000ULL,           // 1 s
        3600000000ULL,        // 1 hour
        34359738368ULL,       // 2^35 us, ~9.5 hours
        86400000000ULL,       // 1 day
        4294967296000ULL,     // 2^32 ms, where a 32-bit millis wraps
        31536000000000ULL,    // 1 year
    };
    for (uint8_t i = 0; i < ARRAY_SIZE(instants); i++) {
        for (int8_t d = -8; d <= 8; d++) {
            div1000_check(instants[i] + d);
        }
    }
}

static void test_div1000(void)
{
    hal.console->printf("Testing div1000\n");
    test_div1000_structured();
    for (uint32_t i=0; i<2000000; i++) {
        uint64_t v = 0;
        if (!hal.util->get_random_vals((uint8_t*)&v, sizeof(v))) {
            AP_HAL::panic("ERROR: div1000 no random");
            break;
        }
        div1000_check(v);
    }
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    // test from locked context
    for (uint32_t i=0; i<2000000; i++) {
        uint64_t v = 0;
        if (!hal.util->get_random_vals((uint8_t*)&v, sizeof(v))) {
            AP_HAL::panic("ERROR: div1000 no random");
            break;
        }
        chSysLock();
        uint64_t v1 = v / 1000ULL;
        uint64_t v2 = uint64_div1000(v);
        chSysUnlock();
        if (v1 != v2) {
            AP_HAL::panic("ERROR: 0x%llx v1=0x%llx v2=0x%llx",
                          (unsigned long long)v, (unsigned long long)v1, (unsigned long long)v2);
            return;
        }
    }
#endif
    hal.console->printf("div1000 OK\n");
}

void loop()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    // pretend we are embedded so that 1.0/0 "works"
    fedisableexcept(FE_ALL_EXCEPT);
#endif
    show_sizes();
    hal.console->printf("\n");
    show_timings();
    test_div1000();
    hal.console->printf("\n");
    hal.scheduler->delay(3000);
}

AP_HAL_MAIN();
