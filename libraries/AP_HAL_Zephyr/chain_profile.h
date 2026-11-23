/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by @davidbuzz and Claude
 */
/* Phase markers for the sensor->EKF chain, sampled over SWD without halting the
 * core, so the profile reflects a running vehicle rather than a stopped one. */
#pragma once
#include <stdint.h>

/* AP_PHASE_SCHED_TASKS added 2026-08-05: AP_PHASE_OTHER was 42.9% of the profile,
 * which is a bucket too coarse to act on. */
enum ap_phase_t {
    AP_PHASE_OTHER = 0,
    AP_PHASE_BUS_CB,        /* DeviceBus periodic callback                  */
    AP_PHASE_SPI_XFER,      /* one SPIDevice transfer                       */
    AP_PHASE_READ_FIFO,     /* Invensensev3::read_fifo - drain the IMU FIFO */
    AP_PHASE_WAIT_SAMPLE,   /* wait_for_sample - main loop BLOCKS here      */
    AP_PHASE_INS_UPDATE,    /* AP_InertialSensor::update                    */
    AP_PHASE_EKF3_UPDATE,   /* AP_NavEKF3::UpdateFilter                     */
    AP_PHASE_AHRS_UPDATE,   /* AP_AHRS::update (Copter::read_AHRS wraps it) */
    AP_PHASE_SCHED_TASKS,   /* AP_Scheduler::run - the scheduled task list  */
    AP_PHASE_COUNT
};

#define AP_PROF_MAIN_PHASE  0
#define AP_PROF_LOOP_COUNT  1
#define AP_PROF_BUSCB_COUNT 2   /* total, all buses */
#define AP_PROF_MAX_BUSES   6
/* per-bus phase words: [3 .. 3+MAX_BUSES-1] */
#define AP_PROF_BUS_PHASE0  3
#define AP_PROF_BUSCNT0     (AP_PROF_BUS_PHASE0 + AP_PROF_MAX_BUSES)
/* Transfer SIZE accounting, added 2026-08-05, to test whether small transfers
 * dominated the bus. */
#define AP_PROF_XFER_COUNT  (AP_PROF_BUSCNT0 + AP_PROF_MAX_BUSES)
#define AP_PROF_XFER_BYTES  (AP_PROF_XFER_COUNT + 1)

/* Transfer DURATION distribution, added 2026-08-05: a histogram, because the mean
 * hid a long tail that the mean alone could not distinguish from jitter. */
#define AP_PROF_XFER_CYC_LO   (AP_PROF_XFER_BYTES + 1)
#define AP_PROF_XFER_CYC_HI   (AP_PROF_XFER_CYC_LO + 1)
#define AP_PROF_XFER_CYC_MIN  (AP_PROF_XFER_CYC_HI + 1)
#define AP_PROF_XFER_CYC_MAX  (AP_PROF_XFER_CYC_MIN + 1)
#define AP_PROF_XFER_HZ       (AP_PROF_XFER_CYC_MAX + 1)
#define AP_PROF_XFER_HIST0    (AP_PROF_XFER_HZ + 1)
/* 24 buckets = 1 cycle .. 16.7M cycles, i.e. up to ~16 ms at this SoC's 1 GHz
   cycle counter. Anything slower than that saturates the top bucket. */
#define AP_PROF_XFER_HIST_N   24

/* PER-PHASE DURATIONS, added 2026-08-05. */
#define AP_PROF_PH_COUNT0     (AP_PROF_XFER_HIST0 + AP_PROF_XFER_HIST_N)
#define AP_PROF_PH_CYC0       (AP_PROF_PH_COUNT0 + AP_PHASE_COUNT)

/* PER-SCHEDULER-TASK accounting, added 2026-08-05. */
#define AP_PROF_MAX_TASKS     64
#define AP_PROF_TASK0         (AP_PROF_PH_CYC0 + 2 * AP_PHASE_COUNT)
/* per task, 3 words: [0] call count, [1] total microseconds, [2] name pointer */
#define AP_PROF_WORDS         (AP_PROF_TASK0 + 3 * AP_PROF_MAX_TASKS)

#ifdef CONFIG_AP_CHAIN_PROFILE
extern "C" volatile uint32_t g_ap_prof[AP_PROF_WORDS];
/* k_cycle_get_32(), out of line so <zephyr/kernel.h> does not have to be pulled
   into AP_NavEKF3 / AP_AHRS / AP_Scheduler, which include this header directly.
   ~310 calls/s at the current loop rate - the call overhead is irrelevant. */
extern "C" uint32_t ap_prof_cycles(void);

/* save/restore so nesting (spi_xfer inside read_fifo inside bus_cb) works */
struct APPhaseScope {
    uint8_t  _slot;
    uint8_t  _phase;
    uint32_t _prev;
    uint32_t _t0;
    APPhaseScope(uint8_t slot, uint32_t p) : _slot(slot), _phase((uint8_t)p) {
        _prev = g_ap_prof[slot];
        g_ap_prof[slot] = p;
        _t0 = ap_prof_cycles();
    }
    ~APPhaseScope() {
        const uint32_t dt = ap_prof_cycles() - _t0;
        g_ap_prof[_slot] = _prev;
        if (_phase < AP_PHASE_COUNT) {
            g_ap_prof[AP_PROF_PH_COUNT0 + _phase]++;
            /* 64-bit accumulate: a 32-bit total of a 1 GHz counter wraps every
               4.29 s, the fault that made micros() unusable on this board. */
            const uint8_t ci = AP_PROF_PH_CYC0 + 2 * _phase;
            const uint32_t prev = g_ap_prof[ci];
            const uint32_t sum = prev + dt;
            g_ap_prof[ci] = sum;
            if (sum < prev) {
                g_ap_prof[ci + 1]++;
            }
        }
    }
};
#define AP_PHASE_MAIN(p) APPhaseScope _ap_ph##__LINE__(AP_PROF_MAIN_PHASE, (p))
#define AP_PHASE_BUSN(bus,p) APPhaseScope _ap_ph##__LINE__(AP_PROF_BUS_PHASE0+((bus)%AP_PROF_MAX_BUSES), (p))
#define AP_PHASE_BUS(p)  APPhaseScope _ap_ph##__LINE__(AP_PROF_BUS_PHASE0, (p))
#define AP_PROF_TICK(i)  do { g_ap_prof[i]++; } while (0)
#define AP_PROF_ADD(i,n) do { g_ap_prof[i] += (n); } while (0)

/*
  Record one transfer duration. `cycles` is a k_cycle_get_32() delta; the caller
  does the timestamping so this header stays free of <zephyr/kernel.h>.
  `hz` is sys_clock_hw_cycles_per_sec(), published once for the reader.
 */
static inline void ap_prof_xfer_record(uint32_t cycles, uint32_t hz)
{
    /* 64-bit accumulate. A 32-bit total of a 1 GHz counter wraps every 4.29 s,
       which is exactly the fault that made micros() unusable here and cost a
       session to find - so carry explicitly rather than letting it wrap. */
    const uint32_t prev = g_ap_prof[AP_PROF_XFER_CYC_LO];
    const uint32_t sum = prev + cycles;
    g_ap_prof[AP_PROF_XFER_CYC_LO] = sum;
    if (sum < prev) {
        g_ap_prof[AP_PROF_XFER_CYC_HI]++;
    }

    /* 0 means "no sample yet" for MIN. Deliberately not a `= 0xffffffff`
       initialiser: these live in .bss and are zeroed at startup. */
    const uint32_t cur_min = g_ap_prof[AP_PROF_XFER_CYC_MIN];
    if (cur_min == 0 || cycles < cur_min) {
        g_ap_prof[AP_PROF_XFER_CYC_MIN] = cycles;
    }
    if (cycles > g_ap_prof[AP_PROF_XFER_CYC_MAX]) {
        g_ap_prof[AP_PROF_XFER_CYC_MAX] = cycles;
    }
    if (g_ap_prof[AP_PROF_XFER_HZ] == 0) {
        g_ap_prof[AP_PROF_XFER_HZ] = hz;
    }

    /* bucket = floor(log2(cycles)); __builtin_clz(0) is UNDEFINED, hence the
       explicit zero case. */
    uint32_t bucket = 0;
    if (cycles != 0) {
        bucket = 31u - (uint32_t)__builtin_clz(cycles);
    }
    if (bucket >= AP_PROF_XFER_HIST_N) {
        bucket = AP_PROF_XFER_HIST_N - 1;
    }
    g_ap_prof[AP_PROF_XFER_HIST0 + bucket]++;
}

/*
  Record one scheduler task execution. `us` is AP_Scheduler's own already
  computed time_taken (microseconds); `name` is the task's static name string,
  stored as a pointer for the reader to follow.
 */
static inline void ap_prof_task_record(uint8_t idx, uint32_t us, const char *name)
{
    if (idx >= AP_PROF_MAX_TASKS) {
        return;
    }
    /* the index must be 16-bit: AP_PROF_TASK0 + 3*63 exceeds 255 */
    const uint16_t base = (uint16_t)(AP_PROF_TASK0 + 3 * (uint16_t)idx);
    g_ap_prof[base]++;
    g_ap_prof[base + 1] += us;   /* us total wraps after 71 min; windows are seconds */
    g_ap_prof[base + 2] = (uint32_t)(uintptr_t)name;
}
#else
#define AP_PHASE_MAIN(p) do {} while (0)
#define AP_PHASE_BUS(p)  do {} while (0)
#define AP_PHASE_BUSN(bus,p) do {} while (0)
#define AP_PROF_TICK(i)  do {} while (0)
#define AP_PROF_ADD(i,n) do {} while (0)
static inline void ap_prof_xfer_record(uint32_t cycles, uint32_t hz)
{
    (void)cycles;
    (void)hz;
}
static inline void ap_prof_task_record(uint8_t idx, uint32_t us, const char *name)
{
    (void)idx;
    (void)us;
    (void)name;
}
#endif
