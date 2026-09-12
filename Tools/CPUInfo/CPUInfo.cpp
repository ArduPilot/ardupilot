/*
  test CPU speed
  Andrew Tridgell September 2011
*/

#define AP_MATH_ALLOW_DOUBLE_FUNCTIONS 1

#include <cmath>

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/I2CDevice.h>   /* bus masks for the Busses section */
#include <AP_HAL/SPIDevice.h>
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
#elif CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR
#include <zephyr/kernel.h>
#include <zephyr/sys/time_units.h>
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
#elif CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR && defined(CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC)
static uint32_t sysclk = CONFIG_SYS_CLOCK_HW_CYCLES_PER_SEC;
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

/*
  Report the kernel tick TWO independent ways, because they can disagree: the
  constant says what the build asked for, the measurement says what the
  scheduler actually does. A mismatch means the hardware timer is not set up
  for the tick the kernel believes it has.

  Reading the measured number:
    ~1-3 us/call     -> a 1 MHz (1 us) tick
    ~1000 us/call    -> a 1 kHz (1 ms) tick, every request rounded up to a tick
  Those are ~1000x apart, so the result cannot be read ambiguously.

  Uses the 32-bit AP_HAL::micros(), NOT the micros16() that TIMEIT uses.
  micros16() wraps every 65.536 ms and this loop runs for a full second on a
  1 kHz tick. That wrap is precisely why the committed "delay(1)" numbers in
  Tools/CPUInfo/output-*.txt are wrong: 250 x delay(1) is 250 ms, which wraps
  ~4x, and 250000 mod 65536 / 250 = 213.6 us/call - which is what those files
  report as "216.7520", not a real timing.
*/
static void show_tick_info(void)
{
    hal.console->printf("\nKernel tick:\n");
#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS && defined(CH_CFG_ST_FREQUENCY)
    hal.console->printf("  CH_CFG_ST_FREQUENCY     %u Hz (compiled in)\n",
                        unsigned(CH_CFG_ST_FREQUENCY));
#elif CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR && defined(CONFIG_SYS_CLOCK_TICKS_PER_SEC)
    hal.console->printf("  SYS_CLOCK_TICKS_PER_SEC %u Hz (compiled in)\n",
                        unsigned(CONFIG_SYS_CLOCK_TICKS_PER_SEC));
#endif

    const uint32_t n = 1000;
    const uint32_t t0 = AP_HAL::micros();
    for (uint32_t i = 0; i < n; i++) {
        hal.scheduler->delay_microseconds(1);
    }
    const uint32_t dt = AP_HAL::micros() - t0;
    const double per = double(dt) / double(n);
    hal.console->printf("  delay_microseconds(1) x%u: %lu us total, %.3f us/call\n",
                        unsigned(n), (unsigned long)dt, per);
    /*
      Report the cost in TICKS of the configured rate, not a guessed tick rate.
      The old text inferred "COARSE tick (~1 kHz)" from per > 100 us, which
      flatly contradicted the SYS_CLOCK_TICKS_PER_SEC printed two lines above:
      measured 200 us at 10 kHz is 2 ticks of a 100 us tick, not evidence of a
      1 kHz tick. k_sleep() guarantees AT LEAST the request and does not align
      to the tick the caller happens to be in, so a sub-tick sleep costing 2
      ticks is the expected worst case rather than an anomaly.
    */
#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR && defined(CONFIG_SYS_CLOCK_TICKS_PER_SEC)
    const double tick_us = 1e6 / double(CONFIG_SYS_CLOCK_TICKS_PER_SEC);
    hal.console->printf("  => %.2f ticks of the configured %.1f us tick%s\n",
                        per / tick_us, tick_us,
                        per / tick_us > 2.5 ? "  <-- MORE than the 2-tick worst case" : "");
#endif

    /*
      delay(ms) SCALING TEST - diagnostic for an unexplained result.

      delay(1) measures ~14.5 us for a 1000 us request, yet delay(5000) plainly
      takes ~5 s of wall clock (CPUInfo's runs are 5.75 s apart and delay(5000)
      is the only thing in loop() that could account for it). Both are long runs
      of consecutive 1 ms sleeps, so "the sleep returns early" does not explain
      the difference.

      Two candidates remain and this separates them:
        - if the per-call cost is FLAT across ms, the sleep is genuinely short
          and something else accounts for the 5 s;
        - if it RISES with ms, the cost is count-dependent;
        - if micros() disagrees with k_uptime_get(), the CLOCK is the liar and
          every timing in this file is suspect.

      Each row is timed twice, by AP_HAL::micros() and by an independent
      millisecond source, so the two clocks can be compared directly.
    */
#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR && defined(CONFIG_SOC_SERIES_STM32H7X)
    /*
      Is the kernel's clock scaled correctly? TIM5 is explicitly programmed to
      1 MHz in AP_HAL_Zephyr/system.cpp, so it is an INDEPENDENT reference for
      k_uptime_get(). If they disagree, the kernel tick is mis-scaled and every
      timeout in the system is wrong by that ratio - not just delay().
      CTRL bit2 CLKSOURCE: 1 = processor clock, 0 = external reference (CPU/8
      on many STM32), which alone would mis-scale ticks by 8x.
    */
    {
        const uint32_t ctrl = SysTick->CTRL, load = SysTick->LOAD;
        const int64_t  k0 = k_uptime_get();
        const uint32_t t0 = AP_HAL::micros();
        while ((AP_HAL::micros() - t0) < 500000U) { }   /* 0.5 s by TIM5 */
        const int64_t  dk = k_uptime_get() - k0;
        hal.console->printf("\nClock cross-check (TIM5 vs kernel tick):\n");
        hal.console->printf("  SysTick CTRL=0x%08x LOAD=%lu CLKSOURCE=%s\n",
                            (unsigned)ctrl, (unsigned long)load,
                            (ctrl & 4U) ? "processor" : "external/div8");
        hal.console->printf("  TIM5 says 500000 us elapsed; k_uptime says %ld ms\n",
                            (long)dk);
        hal.console->printf("  => kernel tick runs %.2fx real time (1.00 = correct)\n",
                            double(dk) / 500.0);
    }
#endif

    hal.console->printf("\ndelay(ms) scaling  (us/call by micros() vs by uptime):\n");
    const uint16_t steps[] = { 1, 2, 5, 10, 50, 100 };
    for (uint8_t si = 0; si < ARRAY_SIZE(steps); si++) {
        const uint16_t ms = steps[si];
        const uint32_t reps = ms >= 50 ? 4 : (ms >= 10 ? 20 : 100);
        /* Tell the HAL the main thread is about to be busy. A monitor thread
           that watchdogs the main loop otherwise reboots the board part-way
           through this test - on AP_HAL_Zephyr at 1800 ms, which this loop
           exceeds by design. Nothing happens on HALs without one. */
        hal.scheduler->expect_delay_ms(ms * reps + 500);
        const uint32_t m0 = AP_HAL::micros();
#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR
        /* k_uptime_get() DIRECT, not AP_HAL::millis64(). On STM32H7
           millis64() is uint64_div1000(micros64()), i.e. the SAME TIM5 source
           as micros() - so using it here compared a clock with itself and an
           earlier version of this test wrongly concluded the clock was sound. */
        const int64_t u0 = k_uptime_get();
#else
        const uint64_t u0 = AP_HAL::millis64();
#endif
        for (uint32_t i = 0; i < reps; i++) {
            hal.scheduler->delay(ms);
        }
        const uint32_t dm = AP_HAL::micros() - m0;
#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR
        const int64_t du = k_uptime_get() - u0;
#else
        const uint64_t du = AP_HAL::millis64() - u0;
#endif
        hal.scheduler->expect_delay_ms(0);
        hal.console->printf("  delay(%-4u) x%-4lu : micros %8.1f us/call | k_uptime %8.1f us/call\n",
                            (unsigned)ms, (unsigned long)reps,
                            double(dm) / double(reps),
                            (double(du) * 1000.0) / double(reps));
    }
}


/*
  SPI DMA value-add metric.

  WHAT THIS ANSWERS: is DMA actually worth it, and above what transfer size?

  WHY NOT "interrupts per transfer": that is a mechanism, not the objective. It
  explains why CPU is consumed but not whether you won.

  WHY NOT "cycles in the SPI path": measured alone it MISLEADS. On
  mr_vmu_rt1176, 2026-08-02, enabling SPI DMA cut spi_xfer occupancy from 84% to
  15.1% of the bus thread - a 5x win by that metric - while total throughput
  COLLAPSED from 3095 to 849 transfers/s and the main loop fell 41.6 -> 8.3 Hz.
  Optimising that number alone declares victory on a regression.

  DMA moves two quantities in OPPOSITE directions:
      CPU cost per transfer  - DMA lowers it (no per-word interrupt)
      wall-clock latency     - DMA raises it (descriptor setup, completion
                               interrupt, two cache maintenance operations)
  so both must be reported, AS A FUNCTION OF SIZE, because the crossover is the
  number you actually need (e.g. to set CONFIG_SPI_NXP_LPSPI_RTIO_DMA_THRESHOLD).

  HOW CPU COST IS MEASURED: a low-priority thread spins incrementing a counter.
  Whatever it manages to count is CPU nobody else took. Compare its rate while
  SPI is idle against its rate during transfers, and the difference is cycles
  the SPI path stole - which is exactly the currency, the CPU the main
  loop does not get. No per-thread accounting needed; just measure what is left.

  Run this on a CPU-driver build and a DMA build and diff the tables.
*/
/* ISR entries, from AP_HAL_Zephyr's override of sys_trace_isr_enter_user().
   Returns 0 when CONFIG_AP_ISR_COUNT is off, so the column reads 0.0. */
#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR && defined(CONFIG_AP_ISR_COUNT)
extern "C" volatile uint32_t g_ap_isr_count;
extern "C" volatile uint32_t g_ap_isr_cycles;
static inline uint32_t ap_isr_count(void) { return g_ap_isr_count; }
static inline uint32_t ap_isr_cycles(void) { return g_ap_isr_cycles; }
#else
static inline uint32_t ap_isr_count(void) { return 0; }
static inline uint32_t ap_isr_cycles(void) { return 0; }
#endif

/* Cycle counter rate behind the ISR-cycle column. Zephyr-only; elsewhere the
   column has nothing to divide by, so return 1 and let the count read as-is. */
#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR
static inline double ap_cyc_per_sec(void) { return double(sys_clock_hw_cycles_per_sec()); }
#else
static inline double ap_cyc_per_sec(void) { return 1.0; }
#endif

static volatile uint32_t spare_counter;
static volatile bool spare_run;
static volatile bool load_run;
static volatile uint16_t load_size;      // 0 = idle, so the baseline runs the same code path
static volatile uint32_t load_xfers;
static volatile uint32_t load_deadline_ms;   // 0 = no bound; see SpiLoad::thread
static AP_HAL::SPIDevice *load_dev;

/* thread_create() takes an AP_HAL::MemberProc, so these have to be member
   functions - there is no FUNCTOR_ macro that binds a free one. */
static struct SpareCycles {
    void thread(void)
    {
        while (spare_run) {
            spare_counter++;
        }
    }
} spare_cycles;

/*
  SPI load generator. This MUST run on its own thread rather than on main:
  the first version drove the sweep from main and compared it against a
  baseline taken during delay(), so it measured "main is awake" (spare/s
  pinned to 0, cyc_stolen a flat 100.0%) instead of what SPI costs.

  Running the load here at APM_SPI_PRIORITY - where the real bus threads sit -
  lets main sleep during BOTH the baseline and the measurement, so the spare
  thread's count difference is attributable to the SPI work itself.
*/
static struct SpiLoad {
    void thread(void)
    {
        static uint8_t buf[256];
        uint32_t since_check = 0;
        while (load_run) {
            const uint16_t sz = load_size;
            if (sz == 0) {
                hal.scheduler->delay(1);
                since_check = 0;
                continue;
            }
            load_dev->read_registers(0x75, buf, sz);   // WHOAMI region, read-only
            load_xfers++;
            /*
              Self-imposed deadline. If this thread is ever mis-prioritised
              above main again it will starve it and hang the board, costing a
              reflash - but it is the thread still running, so it can rescue
              itself. millis() is 0.20 us (CPUInfo "Operation timings"),
              negligible amortised over 256 transfers.
            */
            if (++since_check >= 256) {
                since_check = 0;
                if (load_deadline_ms && AP_HAL::millis() > load_deadline_ms) {
                    load_size = 0;
                }
            }
        }
    }
} spi_load;

static void show_spi_dma_metric(void)
{
    // read-only register access, so this cannot disturb sensor configuration.
    //
    // Asks the HAL which devices the board actually has rather than naming
    // them. The previous list - imu_sensor1/2/3 - is the mr_vmu_rt1176 hwdef's
    // naming, so this metric ran there and silently skipped itself everywhere
    // else; CubeOrange calls the same parts ms5611, icm20948 and so on.
    AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev;
    const char *used = nullptr;
    const uint8_t ndev = hal.spi->get_count();
    for (uint8_t i = 0; i < ndev; i++) {
        const char *n = hal.spi->get_device_name(i);
        if (n == nullptr) {
            continue;
        }
        dev = hal.spi->get_device(n);
        if (dev) { used = n; break; }
    }
    if (!dev) {
        hal.console->printf("\nSPI metric: no SPI device found, skipped (HAL reports %u)\n",
                            (unsigned)ndev);
        return;
    }
    load_dev = dev.get();

    spare_run = true;
    load_run = true;
    load_size = 0;
    if (!hal.scheduler->thread_create(
            FUNCTOR_BIND(&spare_cycles, &SpareCycles::thread, void),
            /*
              MUST be the least urgent thread in the system. The loop never
              yields, so at any priority that ties or beats another thread it
              starves it. PRIORITY_IO with offset -1 gave K_PRIO_PREEMPT(8) -
              exactly APM_MAIN_PRIORITY - and main and this thread then
              ping-ponged in CONFIG_TIMESLICE_SIZE (20 ms) quanta, which looks
              identical to a hang from the console.

              thread_create computes prio = base_prio - offset, and
              PRIORITY_SCRIPTING has base_prio 13 (Scheduler.cpp), so offset -1
              gives 14 = the largest valid value with
              CONFIG_NUM_PREEMPT_PRIORITIES=15.
            */
            "spare", 1024, AP_HAL::Scheduler::PRIORITY_SCRIPTING, -1)) {
        hal.console->printf("\nSPI metric: could not start spare-cycle thread\n");
        return;
    }
    if (!hal.scheduler->thread_create(
            FUNCTOR_BIND(&spi_load, &SpiLoad::thread, void),
            /*
              Ordering is what matters, NOT the label: this thread must sit
              BELOW main and ABOVE spare.

              PRIORITY_SPI (the real bus threads' priority) maps to 6 against
              main's 8, and lower wins - so a load loop there is permanently
              runnable and main NEVER returns from delay(300). Timeslicing does
              not save it: that only rotates threads of EQUAL priority. The
              previous run had already shown read_registers does not block below
              64 bytes (spare/s was 0 for every size <= 32), so the loop never
              yields of its own accord.

              PRIORITY_STORAGE maps to 12: under main (8), over spare (14).
              This does not distort the measurement - main is blocked for the
              whole window regardless, so the load thread has the CPU either
              way, and interrupt accounting is priority-independent.
            */
            "spiload", 2048, AP_HAL::Scheduler::PRIORITY_STORAGE, 0)) {
        hal.console->printf("\nSPI metric: could not start SPI load thread\n");
        spare_run = false;
        return;
    }
    hal.scheduler->delay(50);       // let both get going

    const double cyc_per_sec = ap_cyc_per_sec();

    /*
      Baseline with load_size == 0: main asleep, load thread idling, spare thread
      free-running. The measurement below is taken in the SAME state apart from
      the SPI traffic, which is what makes the difference attributable.
    */
    const uint32_t base_t0 = spare_counter;
    const uint32_t base_i0 = ap_isr_count();
    const uint64_t base_us0 = AP_HAL::micros64();
    hal.scheduler->delay(300);
    const double base_secs = double(AP_HAL::micros64() - base_us0) * 1e-6;
    const double base_rate = double(spare_counter - base_t0) / base_secs;
    const double idle_irq_rate = double(ap_isr_count() - base_i0) / base_secs;

    hal.console->printf("\nSPI DMA metric (device %s)\n", used);
    hal.console->printf("  baseline: spare %.3g counts/s, %.0f irq/s with no SPI traffic\n",
                        base_rate, idle_irq_rate);
    hal.console->printf("  %5s %10s %12s %11s %10s %10s\n",
                        "bytes", "us/xfer", "xfers/s", "cpu_used%", "isr_cpu%", "irq/xfer");

    const uint16_t sizes[] = { 1, 2, 4, 8, 16, 32, 64, 128, 256 };
    for (uint8_t i = 0; i < ARRAY_SIZE(sizes); i++) {
        load_xfers = 0;
        load_deadline_ms = AP_HAL::millis() + 2000;   // window is 320ms; slack for a slow size
        load_size = sizes[i];
        hal.scheduler->delay(20);           // discard the ramp-up
        const uint32_t c0 = spare_counter;
        const uint32_t i0 = ap_isr_count();
        const uint32_t y0 = ap_isr_cycles();
        const uint32_t x0 = load_xfers;
        const uint64_t t0 = AP_HAL::micros64();     // micros64: micros16 wraps at 65.5ms
        hal.scheduler->delay(300);                  // main sleeps - same state as baseline
        const uint64_t dt = AP_HAL::micros64() - t0;
        const uint32_t dc = spare_counter - c0;
        const uint32_t di = ap_isr_count() - i0;
        const uint32_t dy = ap_isr_cycles() - y0;
        const uint32_t dx = load_xfers - x0;

        const double secs = double(dt) * 1e-6;
        const double obs_rate = double(dc) / secs;
        // spare cycles the SPI work consumed, as a fraction of what was free
        const double cpu_used = base_rate > 0 ? (1.0 - obs_rate / base_rate) * 100.0 : 0.0;
        // cycles actually spent inside ISRs, as a fraction of wall time
        const double isr_cpu = double(dy) / (secs * cyc_per_sec) * 100.0;
        // subtract the idle interrupt rate so what remains is SPI's own
        const double irq_per_xfer = dx ? (double(di) - idle_irq_rate * secs) / double(dx) : 0.0;

        hal.console->printf("  %5u %10.1f %12.0f %11.1f %10.2f %10.1f\n",
                            (unsigned)sizes[i],
                            dx ? secs * 1e6 / double(dx) : 0.0,
                            double(dx) / secs,
                            cpu_used, isr_cpu,
                            irq_per_xfer > 0 ? irq_per_xfer : 0.0);
    }
    load_size = 0;
    load_run = false;
    spare_run = false;
    hal.scheduler->delay(50);
    hal.console->printf("  cpu_used%% = total CPU the SPI path consumed (thread + ISR).\n"
                        "  isr_cpu%%  = the part of it spent INSIDE interrupt handlers.\n"
                        "  irq/xfer counts events; isr_cpu%% is what they COST - a high\n"
                        "  irq/xfer with a low isr_cpu%% is not what is binding the CPU.\n"
                        "  DMA should cut irq/xfer and isr_cpu%% hard, and cpu_used%% is\n"
                        "  the bottom line; us/xfer may rise. Run this on a CPU-driver\n"
                        "  build and a DMA build and diff the tables.\n");
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
#elif defined(__riscv)
/* RV32 has no register-window/ldm-stm equivalents and spills far more of
   the TIMEIT locals; the same function that fits in 2000 bytes on ARM
   measures 3216 here. Thread stacks on the RISC-V Zephyr targets are
   8 KB, so allow it rather than restructure the benchmark. */
#pragma GCC diagnostic error "-Wframe-larger-than=3500"
#else
#pragma GCC diagnostic error "-Wframe-larger-than=2000"
#endif
/*
  Bus speeds, one line per peripheral.

  The number in hwdef.dat is a CEILING, not a setting: the driver picks the
  largest divider that stays under it, so what the silicon clocks out can be
  well below what was asked for and nothing normally reports it. On
  CubeOrangeZephyr the SPI buses do not even share a source - spi1/spi2 take
  PLL1_Q while the board devicetree points spi4 at PLL3_Q.

  Derived from the dividers and the selected kernel clock, NOT timed. A timed
  transfer measures the driver as much as the wire: on a board doing programmed
  I/O, asking for 20 MHz instead of 2 MHz moved measured throughput by 0.1%.

  The same binary runs on ChibiOS and Zephyr, so the two reports diff
  column-for-column on one board - which is the comparison that matters when a
  peripheral behaves differently under one HAL.
 */
#if defined(STM32H7) || (CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR && defined(CONFIG_SOC_SERIES_STM32H7X))
#define AP_CPUINFO_BUS_CLOCKS_H7 1

/* The crystal, from wherever this HAL records it: ChibiOS puts STM32_HSECLK in
   the generated hwdef.h, Zephyr has it as the clk_hse node's clock-frequency. */
#if defined(STM32_HSECLK)
#define AP_H7_HSE_HZ STM32_HSECLK
#elif DT_NODE_HAS_PROP(DT_NODELABEL(clk_hse), clock_frequency)
#define AP_H7_HSE_HZ DT_PROP(DT_NODELABEL(clk_hse), clock_frequency)
#else
#define AP_H7_HSE_HZ 24000000U
#endif

#define AP_H7_RCC_BASE 0x58024400UL
static inline uint32_t h7_rcc(uint32_t off)
{
    return *(volatile uint32_t *)(AP_H7_RCC_BASE + off);
}

/* Reference for PLL1/PLL3: HSE when selected, else the 64 MHz HSI. Both PLLs
   divide it by their own DIVMx before the VCO. */
static uint32_t h7_pll_ref_hz(uint8_t which)
{
    const uint32_t sel = h7_rcc(0x28);                 /* PLLCKSELR */
    uint32_t src;
    switch (sel & 3U) {
    case 2:  src = AP_H7_HSE_HZ;      break;           /* HSE */
    case 1:  src = 4000000U;          break;           /* CSI */
    default: src = 64000000U;         break;           /* HSI */
    }
    const uint32_t divm = (sel >> (which == 1 ? 4 : (which == 2 ? 12 : 20))) & 0x3FU;
    return divm ? src / divm : 0;
}

/* Output n of PLL x, in Hz. out: 0=P 1=Q 2=R. */
static uint32_t h7_pll_out_hz(uint8_t which, uint8_t out)
{
    const uint32_t divr = h7_rcc(which == 1 ? 0x30 : (which == 2 ? 0x38 : 0x40));
    const uint32_t n = (divr & 0x1FFU) + 1U;
    /* RCC_PLLnDIVR field positions, from stm32h743xx.h: N at 0 (9 bits), P at
       9, Q at 16, R at 24, each 7 bits. They are NOT evenly spaced - getting
       Q and R one bit high reports PLL3_Q as 96 MHz instead of 48 and PLL3_R
       as 480 instead of 240, which is exactly wrong enough to look plausible. */
    uint32_t d;
    switch (out) {
    case 0:  d = ((divr >> 9)  & 0x7FU) + 1U; break;   /* P */
    case 1:  d = ((divr >> 16) & 0x7FU) + 1U; break;   /* Q */
    default: d = ((divr >> 24) & 0x7FU) + 1U; break;   /* R */
    }
    const uint64_t vco = (uint64_t)h7_pll_ref_hz(which) * n;
    return d ? (uint32_t)(vco / d) : 0;
}

/* APB1 or APB2 (D2 domain) peripheral clock, for peripherals fed from PCLK. */
static uint32_t h7_pclk_hz(uint8_t apb)
{
    const uint32_t d1 = h7_rcc(0x18), d2 = h7_rcc(0x1C);
    static const uint16_t hpre_div[] = { 1,1,1,1,1,1,1,1, 2,4,8,16,64,128,256,512 };
    const uint32_t ahb = h7_pll_out_hz(1, 0) / hpre_div[d1 & 0xFU];
    const uint32_t ppre = (d2 >> (apb == 1 ? 4 : 8)) & 0x7U;
    return (ppre & 4U) ? ahb / (1U << ((ppre & 3U) + 1U)) : ahb;
}

/* SPI1/2/3 take D2CCIP1R[14:12]; SPI4/5 take [18:16]. */
static uint32_t h7_spi_kernel_hz(uint8_t spi)
{
    const uint32_t r = h7_rcc(0x50);                   /* D2CCIP1R */
    const uint32_t sel = (spi <= 3) ? ((r >> 12) & 7U) : ((r >> 16) & 7U);
    if (spi <= 3) {
        switch (sel) {
        case 0: return h7_pll_out_hz(1, 1);            /* PLL1_Q */
        case 1: return h7_pll_out_hz(2, 0);            /* PLL2_P */
        case 2: return h7_pll_out_hz(3, 0);            /* PLL3_P */
        default: return 0;                             /* PIN/HSI/CSI/HSE */
        }
    }
    switch (sel) {
    case 0: return h7_pclk_hz(2);
    case 1: return h7_pll_out_hz(2, 1);                /* PLL2_Q */
    case 2: return h7_pll_out_hz(3, 1);                /* PLL3_Q */
    default: return 0;
    }
}

/* I2C1/2/3 take D2CCIP2R[13:12]; I2C4 takes D3CCIPR[9:8]. Same encoding. */
static uint32_t h7_i2c_kernel_hz(uint8_t i2c)
{
    const uint32_t sel = (i2c <= 3) ? ((h7_rcc(0x54) >> 12) & 3U)
                                    : ((h7_rcc(0x58) >> 8)  & 3U);
    switch (sel) {
    case 0:  return h7_pclk_hz(1);
    case 1:  return h7_pll_out_hz(3, 2);               /* PLL3_R */
    case 2:  return 64000000U;                         /* HSI */
    default: return 4000000U;                          /* CSI */
    }
}

/* USART1/6 take D2CCIP2R[2:0]; the rest take [5:3]. USART1/6 are on APB2. */
static uint32_t h7_usart_kernel_hz(uint8_t n)
{
    const bool apb2 = (n == 1 || n == 6);
    const uint32_t sel = (h7_rcc(0x54) >> (apb2 ? 0 : 3)) & 7U;
    switch (sel) {
    case 0:  return h7_pclk_hz(apb2 ? 2 : 1);
    case 1:  return h7_pll_out_hz(2, 1);               /* PLL2_Q */
    case 2:  return h7_pll_out_hz(3, 1);               /* PLL3_Q */
    case 3:  return 64000000U;                         /* HSI */
    case 4:  return 4000000U;                          /* CSI */
    default: return 32768U;                            /* LSE */
    }
}
#endif  /* STM32H7 */

#if CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR && defined(CONFIG_SOC_SERIES_IMXRT11XX)
#define AP_CPUINFO_BUS_CLOCKS_RT11XX 1
#include <fsl_clock.h>

/*
  Same idea as the H7 block, different silicon. The RT1176 has no PLL-per-
  peripheral mux: every peripheral hangs off a CCM "clock root" with its own
  source mux and divider, so the NXP SDK's own CLOCK_GetRootClockFreq() is the
  honest way to read it - it decodes the root the driver actually programmed
  rather than re-deriving the CCM by hand.

  LPSPI divides that root again: SCK = root / (SCKDIV + 2), SCKDIV being
  CCR[7:0] at offset 0x40.
 */
static void rt11xx_show_bus_clocks(void)
{
    /* All six. LPSPI5/6 live in the CM4/wakeup domain at 0x40C2xxxx, not with
       1-4 at 0x4011xxxx. */
    static const struct { const char *name; uint32_t base; clock_root_t root; } lpspis[] = {
        { "LPSPI1", 0x40114000UL, kCLOCK_Root_Lpspi1 },
        { "LPSPI2", 0x40118000UL, kCLOCK_Root_Lpspi2 },
        { "LPSPI3", 0x4011C000UL, kCLOCK_Root_Lpspi3 },
        { "LPSPI4", 0x40120000UL, kCLOCK_Root_Lpspi4 },
        { "LPSPI5", 0x40C2C000UL, kCLOCK_Root_Lpspi5 },
        { "LPSPI6", 0x40C30000UL, kCLOCK_Root_Lpspi6 },
    };
    for (uint8_t i = 0; i < ARRAY_SIZE(lpspis); i++) {
        const uint32_t root = CLOCK_GetRootClockFreq(lpspis[i].root);
        if (root == 0) {
            continue;
        }
        const uint32_t ccr = *(volatile uint32_t *)(lpspis[i].base + 0x40);
        const uint32_t sckdiv = ccr & 0xFFU;
        hal.console->printf("  %s  root %8lu Hz  /%-3u  SCK %8lu Hz\n",
                            lpspis[i].name, (unsigned long)root,
                            (unsigned)(sckdiv + 2U),
                            (unsigned long)(root / (sckdiv + 2U)));
    }

    /* LPI2C and LPUART: the root is most of the answer and is what a wrong
       clock tree gets wrong. The per-peripheral divider (LPI2C CLKLO/CLKHI,
       LPUART OSR/SBR) is not decoded yet - see ZEPHYR_TODO 2.4/2.12. */
    /* All six LPI2C and all twelve LPUART the RT1176 has. */
    static const struct { const char *name; clock_root_t root; } others[] = {
        { "LPI2C1", kCLOCK_Root_Lpi2c1 },   { "LPI2C2", kCLOCK_Root_Lpi2c2 },
        { "LPI2C3", kCLOCK_Root_Lpi2c3 },   { "LPI2C4", kCLOCK_Root_Lpi2c4 },
        { "LPI2C5", kCLOCK_Root_Lpi2c5 },   { "LPI2C6", kCLOCK_Root_Lpi2c6 },
        { "LPUART1", kCLOCK_Root_Lpuart1 }, { "LPUART2", kCLOCK_Root_Lpuart2 },
        { "LPUART3", kCLOCK_Root_Lpuart3 }, { "LPUART4", kCLOCK_Root_Lpuart4 },
        { "LPUART5", kCLOCK_Root_Lpuart5 }, { "LPUART6", kCLOCK_Root_Lpuart6 },
        { "LPUART7", kCLOCK_Root_Lpuart7 }, { "LPUART8", kCLOCK_Root_Lpuart8 },
        { "LPUART9", kCLOCK_Root_Lpuart9 }, { "LPUART10", kCLOCK_Root_Lpuart10 },
        { "LPUART11", kCLOCK_Root_Lpuart11 }, { "LPUART12", kCLOCK_Root_Lpuart12 },
    };
    for (uint8_t i = 0; i < ARRAY_SIZE(others); i++) {
        const uint32_t root = CLOCK_GetRootClockFreq(others[i].root);
        if (root == 0) {
            continue;
        }
        hal.console->printf("  %-7s root %8lu Hz\n", others[i].name, (unsigned long)root);
    }
}
#endif  /* IMXRT11XX */

/*
  Touch every bus once so the register decode in show_busses() reads a
  peripheral that has actually been configured.

  ChibiOS ungates and programs an SPI or I2C block only when a driver opens
  it, and CPUInfo opens none, so without this every SPI and I2C register in
  the decode reads back as zero and the whole listing comes out empty. Zephyr
  initialises every enabled devicetree node at boot, which is why its listing
  was full and the ChibiOS one was not.

  Both transfers are chosen so they cannot disturb anything on the bus. The
  SPI one clocks a single byte: an SPI sensor takes the first byte of a
  transaction as a register address and needs a second byte before it writes
  anything, so a one-byte transaction has no effect on any device. The I2C one
  addresses 0x7F, inside the reserved 0x78-0x7F block that no device may
  claim, so it always ends in a NACK.
*/
static void exercise_busses(void)
{
    const uint8_t nspi = hal.spi->get_count();
    for (uint8_t i = 0; i < nspi; i++) {
        const char *name = hal.spi->get_device_name(i);
        if (name == nullptr) {
            continue;
        }
        auto dev = hal.spi->get_device(name);
        if (!dev) {
            continue;
        }
        WITH_SEMAPHORE(dev->get_semaphore());
        uint8_t rx = 0;
        dev->transfer(nullptr, 0, &rx, 1);
    }

    const uint32_t i2c_mask = hal.i2c_mgr->get_bus_mask();
    for (uint8_t bus = 0; bus < 32; bus++) {
        if ((i2c_mask & (1UL << bus)) == 0) {
            continue;
        }
        auto dev = hal.i2c_mgr->get_device(bus, 0x7F);
        if (!dev) {
            continue;
        }
        WITH_SEMAPHORE(dev->get_semaphore());
        uint8_t rx = 0;
        dev->transfer(nullptr, 0, &rx, 1);
    }
}

/* Every other board reaches show_busses() with neither decode selected, and
   ArduPilot builds with -Werror=undef, so an unset guard is a build failure
   rather than a quietly-false branch. Default them here instead of testing
   with defined() at each use. */
#ifndef AP_CPUINFO_BUS_CLOCKS_H7
#define AP_CPUINFO_BUS_CLOCKS_H7 0
#endif
#ifndef AP_CPUINFO_BUS_CLOCKS_RT11XX
#define AP_CPUINFO_BUS_CLOCKS_RT11XX 0
#endif

static void show_busses(void)
{
    exercise_busses();

    hal.console->printf("\nBusses:\n");

#if AP_CPUINFO_BUS_CLOCKS_H7
    hal.console->printf("  clock tree: SYSCLK %lu Hz  PCLK1 %lu Hz  PLL1_Q %lu  PLL3_Q %lu\n",
                        (unsigned long)h7_pll_out_hz(1, 0), (unsigned long)h7_pclk_hz(1),
                        (unsigned long)h7_pll_out_hz(1, 1), (unsigned long)h7_pll_out_hz(3, 1));

    /* Whether a block is clocked is a fact about RCC, not something to infer
       from one of its registers reading zero - a gated peripheral and one
       sitting at its reset value read back identically. RCC_APB1LENR 0xE8,
       RCC_APB2ENR 0xF0, RCC_APB4ENR 0xF4. */
    const uint32_t apb1lenr = h7_rcc(0xE8);
    const uint32_t apb2enr  = h7_rcc(0xF0);
    const uint32_t apb4enr  = h7_rcc(0xF4);

    /* SCK = kernel / 2^(MBR+1). CFG1 is at +0x08 on the H7 SPI block. */
    static const struct {
        const char *name; uint32_t base; uint8_t n; uint8_t enr; uint8_t bit;
    } spis[] = {
        { "SPI1", 0x40013000UL, 1, 2, 12 }, { "SPI2", 0x40003800UL, 2, 1, 14 },
        { "SPI3", 0x40003C00UL, 3, 1, 15 }, { "SPI4", 0x40013400UL, 4, 2, 13 },
        { "SPI5", 0x40015000UL, 5, 2, 20 }, { "SPI6", 0x58001400UL, 6, 4,  5 },
    };
    for (uint8_t i = 0; i < ARRAY_SIZE(spis); i++) {
        const uint32_t enr = spis[i].enr == 1 ? apb1lenr : (spis[i].enr == 2 ? apb2enr : apb4enr);
        const bool on = (enr & (1UL << spis[i].bit)) != 0;
        const uint32_t kern = h7_spi_kernel_hz(spis[i].n);
        if (!on) {
            hal.console->printf("  %-6s kernel %8lu Hz   (not clocked)\n",
                                spis[i].name, (unsigned long)kern);
            continue;
        }
        /* MBR is per-transaction state, not a property of the bus: the driver
           rewrites it for each device's requested speed, so read at an
           arbitrary moment it reports whatever the last transfer left behind.
           Labelled as such rather than presented as "the" SCK - the kernel
           clock is the number that means something here. */
        const uint32_t cfg1 = *(volatile uint32_t *)(spis[i].base + 0x08);
        const uint32_t mbr = (cfg1 >> 28) & 7U;
        hal.console->printf("  %-6s kernel %8lu Hz   last xfer /%-3u = %8lu Hz\n",
                            spis[i].name, (unsigned long)kern,
                            (unsigned)(1U << (mbr + 1U)),
                            (unsigned long)(kern >> (mbr + 1U)));
    }

    /* All four I2C the H743 has. SCL ~= kernel / ((PRESC+1) * (SCLL+SCLH+2)),
       TIMINGR at +0x10. Ignores the rise/fall padding, so it reads a little
       high against a scope - close enough to catch a wrong kernel clock. */
    static const struct {
        const char *name; uint32_t base; uint8_t n; uint8_t enr; uint8_t bit;
    } i2cs[] = {
        { "I2C1", 0x40005400UL, 1, 1, 21 }, { "I2C2", 0x40005800UL, 2, 1, 22 },
        { "I2C3", 0x40005C00UL, 3, 1, 23 }, { "I2C4", 0x58001C00UL, 4, 4,  7 },
    };
    for (uint8_t i = 0; i < ARRAY_SIZE(i2cs); i++) {
        const uint32_t enr = i2cs[i].enr == 1 ? apb1lenr : apb4enr;
        const bool on = (enr & (1UL << i2cs[i].bit)) != 0;
        const uint32_t kern = h7_i2c_kernel_hz(i2cs[i].n);
        if (!on) {
            hal.console->printf("  %-6s kernel %8lu Hz   (not clocked)\n",
                                i2cs[i].name, (unsigned long)kern);
            continue;
        }
        const uint32_t tim = *(volatile uint32_t *)(i2cs[i].base + 0x10);
        const uint32_t presc = ((tim >> 28) & 0xFU) + 1U;
        const uint32_t scll = (tim & 0xFFU) + 1U;
        const uint32_t sclh = ((tim >> 8) & 0xFFU) + 1U;
        const uint32_t denom = presc * (scll + sclh);
        hal.console->printf("  %-6s kernel %8lu Hz   SCL ~%8lu Hz\n",
                            i2cs[i].name, (unsigned long)kern,
                            (unsigned long)(denom ? kern / denom : 0));
    }

    /* All eight the H743 has. BRR at +0x0C is the whole divider at OVER8=0. */
    static const struct {
        const char *name; uint32_t base; uint8_t n; uint8_t enr; uint8_t bit;
    } uarts[] = {
        { "USART1", 0x40011000UL, 1, 2,  4 }, { "USART2", 0x40004400UL, 2, 1, 17 },
        { "USART3", 0x40004800UL, 3, 1, 18 }, { "UART4",  0x40004C00UL, 4, 1, 19 },
        { "UART5",  0x40005000UL, 5, 1, 20 }, { "USART6", 0x40011400UL, 6, 2,  5 },
        { "UART7",  0x40007800UL, 7, 1, 30 }, { "UART8",  0x40007C00UL, 8, 1, 31 },
    };
    for (uint8_t i = 0; i < ARRAY_SIZE(uarts); i++) {
        const uint32_t enr = uarts[i].enr == 1 ? apb1lenr : apb2enr;
        const bool on = (enr & (1UL << uarts[i].bit)) != 0;
        const uint32_t kern = h7_usart_kernel_hz(uarts[i].n);
        if (!on) {
            hal.console->printf("  %-6s kernel %8lu Hz   (not clocked)\n",
                                uarts[i].name, (unsigned long)kern);
            continue;
        }
        const uint32_t brr = *(volatile uint32_t *)(uarts[i].base + 0x0C) & 0xFFFFU;
        hal.console->printf("  %-6s kernel %8lu Hz   BRR %5lu  baud %8lu\n",
                            uarts[i].name, (unsigned long)kern,
                            (unsigned long)brr, (unsigned long)(brr ? kern / brr : 0));
    }

#elif AP_CPUINFO_BUS_CLOCKS_RT11XX
    rt11xx_show_bus_clocks();
#else
    hal.console->printf("  (no bus-clock decode for this SoC yet)\n");
#endif

    /* Portable half: what the HAL believes, on any board. Device names come
       from hwdef; AP_HAL exposes no speed getter, which is why the decode
       above exists at all. */
    const uint8_t nspi = hal.spi->get_count();
    hal.console->printf("  SPI devices (%u):", (unsigned)nspi);
    for (uint8_t i = 0; i < nspi; i++) {
        const char *n = hal.spi->get_device_name(i);
        hal.console->printf(" %s", n ? n : "?");
    }
    hal.console->printf("\n");

    hal.console->printf("  I2C buses: mask 0x%lx  internal 0x%lx  external 0x%lx\n",
                        (unsigned long)hal.i2c_mgr->get_bus_mask(),
                        (unsigned long)hal.i2c_mgr->get_bus_mask_internal(),
                        (unsigned long)hal.i2c_mgr->get_bus_mask_external());

    for (uint8_t i = 0; i < AP_HAL::HAL::num_serial; i++) {
        AP_HAL::UARTDriver *u = hal.serial(i);
        if (u == nullptr) {
            continue;
        }
        const uint32_t baud = u->get_baud_rate();
        if (baud == 0) {
            continue;   /* not begun, or a driver with no baud concept (USB) */
        }
        hal.console->printf("  SERIAL%u  baud %lu\n", (unsigned)i, (unsigned long)baud);
    }
}

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
#elif CONFIG_HAL_BOARD == HAL_BOARD_ZEPHYR
    // Zephyr's raw cycle-counter/uptime reads - same role as ChibiOS's hrt_*
    // above (timer cost independent of the AP_HAL::micros/millis wrapper).
    TIMEIT("k_cycle_get_32()", k_cycle_get_32(), 200);
    TIMEIT("k_cyc_to_us32()", k_cyc_to_us_floor32(k_cycle_get_32()), 200);
    TIMEIT("k_cyc_to_us64()", k_cyc_to_us_floor64(k_cycle_get_32()), 200);
    TIMEIT("k_uptime_get_32()", k_uptime_get_32(), 200);
    TIMEIT("k_uptime_get()", k_uptime_get(), 200);
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
    /*
      NOT TIMEIT: this one must use 32-bit micros(). TIMEIT times with
      micros16(), which wraps every 65.536 ms, and 250 x delay(1) is ~250 ms -
      so the committed "delay(1) 216.7520 usec/call" in the output-*.txt files
      is a wrap artifact (250000 mod 65536 / 250 = 213.6), not a measurement.
      A 1 ms delay obviously cannot average 216 us.
    */
    {
        const uint32_t dn = 250;
        const uint32_t dt0 = AP_HAL::micros();
        for (uint32_t i = 0; i < dn; i++) {
            hal.scheduler->delay(1);
        }
        const uint32_t ddt = AP_HAL::micros() - dt0;
        hal.console->printf("%-10s %7.4f usec/call\n", "delay(1)",
                            double(ddt) / double(dn));
    }

    TIMEIT("SEM", { WITH_SEMAPHORE(sem); v_out_32 += v_32;}, 100);
}

static void test_div1000(void)
{
    hal.console->printf("Testing div1000\n");
    for (uint32_t i=0; i<2000000; i++) {
        uint64_t v = 0;
        if (!hal.util->get_random_vals((uint8_t*)&v, sizeof(v))) {
            AP_HAL::panic("ERROR: div1000 no random");
            break;
        }
        uint64_t v1 = v / 1000ULL;
        uint64_t v2 = uint64_div1000(v);
        if (v1 != v2) {
            AP_HAL::panic("ERROR: 0x%llx v1=0x%llx v2=0x%llx",
                          (unsigned long long)v, (unsigned long long)v1, (unsigned long long)v2);
            return;
        }
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
    /* One declaration covering the whole pass. The main loop only pats the
       software watchdog after loop() RETURNS, and this whole report runs inside
       a single call, so an unbracketed section longer than the monitor's warn
       threshold reports a stall that is really just measurement in progress -
       the 500 ms clock cross-check and the timing sweeps both do. Those
       warnings are not merely noise: on a USB CDC console they consume the same
       TX ring buffer as the report, which discards when full. */
    hal.scheduler->expect_delay_ms(60000);

    show_sizes();
    show_tick_info();
    show_spi_dma_metric();
    hal.console->printf("\n");
    show_timings();
    show_busses();
#if CONFIG_HAL_BOARD != HAL_BOARD_ZEPHYR
    test_div1000();
#endif
    hal.console->printf("\n");
    /* The pause between report passes is far longer than a monitor thread
       that watchdogs the main loop will tolerate, so keep it declared too. */
    hal.scheduler->expect_delay_ms(5500);
    hal.scheduler->delay(5000);
    hal.scheduler->expect_delay_ms(0);   /* closes the 5500 above */

    hal.scheduler->expect_delay_ms(0);   /* closes the pass-wide declaration */
}

AP_HAL_MAIN();
