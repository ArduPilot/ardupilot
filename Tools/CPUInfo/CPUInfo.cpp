/*
  test CPU speed
  Andrew Tridgell September 2011
*/

#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include <cmath>

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_ESC_Telem/AP_ESC_Telem.h>
#include "EKF_Maths.h"

#if HAL_WITH_DSP && CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
#include <arm_math.h>
#endif

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

#if CONFIG_HAL_BOARD != HAL_BOARD_LINUX

// On H750 we want to measure external flash to ram performance
#if defined(EXT_FLASH_SIZE_MB) && EXT_FLASH_SIZE_MB>0 && defined(STM32H7)
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
AP_ESC_Telem telem;

void setup() {
#ifdef DISABLE_CACHES
    SCB_DisableDCache();
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

void loop()
{
    show_sizes();
    hal.console->printf("\n");
    show_timings();
    hal.console->printf("\n");
    hal.scheduler->delay(3000);
}

#else
void loop() {}
void setup() {}
#endif

AP_HAL_MAIN();
