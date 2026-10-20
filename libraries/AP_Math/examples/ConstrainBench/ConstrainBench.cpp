/*
  benchmark constrain_float() using the DWT cycle counter

  warm: calls run back to back
  btac: one call per measurement, after evicting the branch predictor
  cold: as btac, but with the instruction cache invalidated as well
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>

#if CONFIG_HAL_BOARD != HAL_BOARD_CHIBIOS
#error "ConstrainBench needs the Cortex-M cycle counter"
#endif

#include <ch.h>
#include <hal.h>

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static const uint16_t num_values = 64;
static const uint16_t num_calls = 1024;
static const uint16_t num_batches = 201;
static const uint16_t num_single = 1001;

static float values[num_values];
static uint32_t cycles[num_single];
static volatile float sink;

void setup()
{
    hal.scheduler->delay(2000);

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
#if defined(STM32H7) || defined(STM32F7)
    DWT->LAR = 0xC5ACCE55;
#endif
    DWT->CYCCNT = 0;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    // a mix of values below, inside and above the [-1,1] range
    for (uint16_t i=0; i<num_values; i++) {
        values[i] = 3 * sinf(i * 0.7);
    }
}

// run 1024 taken branches at distinct addresses to evict the branch
// predictor's entries for the code under test
static void __attribute__((noinline)) pollute_btac()
{
    asm volatile(".rept 1024\n\tb 1f\n1:\n\t.endr\n" ::: "memory");
}

static int cmp_u32(const void *a, const void *b)
{
    const uint32_t x = *(const uint32_t *)a, y = *(const uint32_t *)b;
    return x < y ? -1 : (x > y ? 1 : 0);
}

static void print_result(const char *mode, uint16_t n, float scale)
{
    qsort(cycles, n, sizeof(cycles[0]), cmp_u32);
    hal.console->printf("BENCH constrain %s med=%.2f min=%.2f\n", mode, cycles[n/2] / scale, cycles[0] / scale);
}

void loop()
{
    // 'B' on the console reboots into the bootloader for reflashing
    while (hal.console->available() > 0) {
        if (hal.console->read() == 'B') {
            hal.scheduler->reboot(true);
        }
    }

    for (uint16_t b=0; b<num_batches; b++) {
        chSysLock();
        const uint32_t start = DWT->CYCCNT;
        float sum = 0;
        for (uint16_t i=0; i<num_calls; i++) {
            sum += constrain_float(values[i % num_values], -1, 1);
        }
        cycles[b] = DWT->CYCCNT - start;
        chSysUnlock();
        sink = sum;
    }
    print_result("warm", num_batches, num_calls);

    for (uint8_t cold=0; cold<2; cold++) {
        for (uint16_t i=0; i<num_single; i++) {
            chSysLock();
            if (cold) {
                SCB_InvalidateICache();
            }
            pollute_btac();
            __DSB();
            __ISB();
            const uint32_t start = DWT->CYCCNT;
            sink = constrain_float(values[i % num_values], -1, 1);
            cycles[i] = DWT->CYCCNT - start;
            chSysUnlock();
        }
        print_result(cold ? "cold" : "btac", num_single, 1);
    }
    hal.scheduler->delay(500);
}

AP_HAL_MAIN();
