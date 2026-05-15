/*
  test CPU speed
  Andrew Tridgell September 2011
*/

#define AP_MATH_ALLOW_DOUBLE_FUNCTIONS 1

#ifndef CPUINFO_ENABLE_EKF_TEST
#define CPUINFO_ENABLE_EKF_TEST 0
#endif

#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <stdio.h>

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
#endif // HAL_BOARD_CHIBIOS

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
#elif defined(RP2350)
//static uint32_t sysclk = 150000000U;// stock
static uint32_t sysclk = 375000000U;  // over clocked
#else
static uint32_t sysclk = 0;
#endif

static EKF_Maths ekf;

HAL_Semaphore sem;
#if HAL_WITH_ESC_TELEM
AP_ESC_Telem telem;
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
static mutex_t s_raw_mtx;
#endif

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && defined(RP2350)
// Section marker — written before each major operation.
// After a crash, read this via OpenOCD to see which section faulted.
// Also read CFSR (0xE000ED28) and HFSR (0xE000ED2C) via OpenOCD mdw command.
char g_fault_section[64];
#define SET_SECTION(s) do { strncpy(g_fault_section, s, sizeof(g_fault_section)-1); } while(0)
#else
#define SET_SECTION(s) do {} while(0)
#endif

// Debugger-visible mirror of console output for boards where USB console
// output is hard to capture during early bring-up.
char g_cpuinfo_debug_log[32768];
uint32_t g_cpuinfo_debug_log_len;
uint32_t g_cpuinfo_debug_log_overflow_count;
uint32_t g_cpuinfo_cycle_count;

static void cpuinfo_debug_log_reset(void)
{
    g_cpuinfo_debug_log_len = 0;
    g_cpuinfo_debug_log[0] = '\0';
}


static void cpuinfo_printf(const char *fmt, ...)
{
    va_list ap_console;
    va_start(ap_console, fmt);
    hal.console->vprintf(fmt, ap_console);
    va_end(ap_console);

    if (g_cpuinfo_debug_log_len >= sizeof(g_cpuinfo_debug_log)) {
        g_cpuinfo_debug_log_overflow_count++;
        return;
    }

    va_list ap_buf;
    va_start(ap_buf, fmt);
    const int n = ::vsnprintf(&g_cpuinfo_debug_log[g_cpuinfo_debug_log_len],
                              sizeof(g_cpuinfo_debug_log) - g_cpuinfo_debug_log_len,
                              fmt,
                              ap_buf);
    va_end(ap_buf);

    if (n <= 0) {
        return;
    }

    const uint32_t appended = MIN<uint32_t>(n, sizeof(g_cpuinfo_debug_log) - g_cpuinfo_debug_log_len - 1);
    g_cpuinfo_debug_log_len += appended;

    if (appended != uint32_t(n)) {
        g_cpuinfo_debug_log_overflow_count++;
    }
}

void setup() {
    cpuinfo_debug_log_reset();
    cpuinfo_printf("CPUInfo setup reached\n");
#ifdef DISABLE_CACHES
#if !HAL_XIP_ENABLED // can't disable DCache in memory-mapped mode
    SCB_DisableDCache();
#endif
    SCB_DisableICache();
#endif
    ekf.init();
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    chMtxObjectInit(&s_raw_mtx);
#endif
}

static void show_sizes(void)
{
    cpuinfo_printf("SYSCLK %uMHz\n", unsigned(sysclk/1000000U));

    cpuinfo_printf("Type sizes:\n");
    cpuinfo_printf("char      : %lu\n", (unsigned long)sizeof(char));
    cpuinfo_printf("short     : %lu\n", (unsigned long)sizeof(short));
    cpuinfo_printf("int       : %lu\n", (unsigned long)sizeof(int));
    cpuinfo_printf("long      : %lu\n", (unsigned long)sizeof(long));
    cpuinfo_printf("long long : %lu\n", (unsigned long)sizeof(long long));
    cpuinfo_printf("bool      : %lu\n", (unsigned long)sizeof(bool));
    cpuinfo_printf("void*     : %lu\n", (unsigned long)sizeof(void *));

    SET_SECTION("print_NaN");
    cpuinfo_printf("printing NaN: %f\n", (double)sqrtf(-1.0f));
    hal.scheduler->delay(50);
    SET_SECTION("print_PosInf");
    cpuinfo_printf("printing +Inf: %f\n", (double)(1.0f/0.0f));
    hal.scheduler->delay(50);
    SET_SECTION("print_NegInf");
    cpuinfo_printf("printing -Inf: %f\n", (double)(-1.0f/0.0f));
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
    cpuinfo_printf("%-10s %7.4f usec/call\n", name, double(dt_us) / double(count * 50.0)); \
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
volatile float v_fn_out_f = 0;
volatile double v_fn_out_d = 0;

static float fp_call_f(float a, float b)
{
    return a * 1.000123f + b;
}

static double fp_call_d(double a, double b)
{
    return a * 1.000123 + b;
}

static float bench_float_call_us(void)
{
    uint64_t us_start = AP_HAL::micros64();
    for (uint16_t i = 0; i < 20000; i++) {
        FIFTYTIMES(v_fn_out_f = fp_call_f(v_f, v_out));
    }
    uint64_t us_end = AP_HAL::micros64();
    return float(us_end - us_start) / float(20000.0f * 50.0f);
}

static float bench_double_call_us(void)
{
    uint64_t us_start = AP_HAL::micros64();
    for (uint16_t i = 0; i < 20000; i++) {
        FIFTYTIMES(v_fn_out_d = fp_call_d(v_d, v_out_d));
    }
    uint64_t us_end = AP_HAL::micros64();
    return float(us_end - us_start) / float(20000.0f * 50.0f);
}

static void show_fp_build_and_precision_summary(void)
{
    const char *float_abi = "unknown";
#if defined(__ARM_PCS_VFP)
    float_abi = "hard";
#elif defined(__SOFTFP__)
    float_abi = "softfp";
#elif defined(__SOFTFP)
    float_abi = "softfp";
#endif

    const char *fpu_mode = "none";
#if defined(__ARM_FP) && (__ARM_FP & 0x4)
    fpu_mode = "sp";
#endif
#if defined(__ARM_FP) && (__ARM_FP & 0x8)
    fpu_mode = "dp";
#endif

    cpuinfo_printf("FP build mode:\n");
    cpuinfo_printf("  float ABI      : %s\n", float_abi);
    cpuinfo_printf("  FPU precision  : %s\n", fpu_mode);
    cpuinfo_printf("  __ARM_FP       : 0x%lx\n", (unsigned long)__ARM_FP);
    cpuinfo_printf("  sizeof(float)  : %lu\n", (unsigned long)sizeof(float));
    cpuinfo_printf("  sizeof(double) : %lu\n", (unsigned long)sizeof(double));

    const float f_call_us = bench_float_call_us();
    const float d_call_us = bench_double_call_us();
    cpuinfo_printf("SP/DP microbench:\n");
    cpuinfo_printf("  float call     : %.5f usec/call\n", (double)f_call_us);
    cpuinfo_printf("  double call    : %.5f usec/call\n", (double)d_call_us);
    if (f_call_us > 0.0f) {
        cpuinfo_printf("  dp/sp ratio    : %.2fx\n", (double)(d_call_us / f_call_us));
    }
    cpuinfo_printf("  note           : hard vs softfp requires comparing separate builds\n");
}

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


    cpuinfo_printf("Operation timings:\n");
    cpuinfo_printf("Note: timings for some operations are very data dependent\n");
    show_fp_build_and_precision_summary();
    cpuinfo_printf("\n");

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
#if CPUINFO_ENABLE_EKF_TEST
    TIMEIT("EKF", v_out = ekf.test(), 5);
#else
    cpuinfo_printf("EKF test skipped (CPUINFO_ENABLE_EKF_TEST=0)\n");
#endif

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

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    // These three tests isolate ChibiOS OS-primitive overhead.
    // chSysLock is where the SMP spinlock sits in port_rp2.mk builds —
    // single-core (port.mk) is just BASEPRI write; SMP adds LDREX/STREX
    // on the hardware spinlock register.
    TIMEIT("chSysLock", { chSysLock(); chSysUnlock(); }, 200);
    TIMEIT("chMtxLock", { chMtxLock(&s_raw_mtx); chMtxUnlock(&s_raw_mtx); }, 100);
    TIMEIT("chThdYield", chThdYield(), 50);
#endif
}

static void test_div1000(void)
{
    // Use a fast xorshift64 PRNG — div1000 tests the math, not the RNG.
    // TRNG reads on RP2350 are slow enough to trigger the watchdog at 2M iters.
    cpuinfo_printf("Testing div1000\n");
    SET_SECTION("div1000_pass1");
    uint64_t prng = 0x123456789abcdefULL;
    for (uint32_t i=0; i<2000000; i++) {
        if ((i % 50000) == 0) {
            hal.scheduler->delay(1); // pat the watchdog
        }
        prng ^= prng << 13; prng ^= prng >> 7; prng ^= prng << 17;
        uint64_t v1 = prng / 1000ULL;
        uint64_t v2 = uint64_div1000(prng);
        if (v1 != v2) {
            AP_HAL::panic("ERROR: 0x%llx v1=0x%llx v2=0x%llx",
                          (unsigned long long)prng, (unsigned long long)v1, (unsigned long long)v2);
            return;
        }
    }
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
    SET_SECTION("div1000_pass2");
    // repeat from locked context
    prng = 0x123456789abcdefULL;
    for (uint32_t i=0; i<2000000; i++) {
        if ((i % 50000) == 0) {
            hal.scheduler->delay(1); // pat the watchdog
        }
        prng ^= prng << 13; prng ^= prng >> 7; prng ^= prng << 17;
        chSysLock();
        uint64_t v1 = prng / 1000ULL;
        uint64_t v2 = uint64_div1000(prng);
        chSysUnlock();
        if (v1 != v2) {
            AP_HAL::panic("ERROR: 0x%llx v1=0x%llx v2=0x%llx",
                          (unsigned long long)prng, (unsigned long long)v1, (unsigned long long)v2);
            return;
        }
    }
#endif
    cpuinfo_printf("div1000 OK\n");
}

void loop()
{
    hal.scheduler->delay(50); // careful of watchdog

    cpuinfo_debug_log_reset();
    hal.scheduler->delay(50); // careful of watchdog
    g_cpuinfo_cycle_count++;
    hal.scheduler->delay(50); // careful of watchdog
    cpuinfo_printf("CPUInfo cycle %lu\n", (unsigned long)g_cpuinfo_cycle_count);
    hal.scheduler->delay(50); // careful of watchdog

    SET_SECTION("show_sizes");
    show_sizes();
    hal.scheduler->delay(50);
    cpuinfo_printf("\n");
    hal.scheduler->delay(50);
    SET_SECTION("show_timings");
    show_timings();
    hal.scheduler->delay(50);
    SET_SECTION("test_div1000");
    test_div1000();
    SET_SECTION("loop_delay");
    cpuinfo_printf("\n");
    hal.scheduler->delay(500);
}

AP_HAL_MAIN();
