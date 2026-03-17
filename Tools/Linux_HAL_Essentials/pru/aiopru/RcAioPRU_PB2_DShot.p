// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.
// You should have received a copy of the GNU General Public License
// along with this program. If not, see <http://www.gnu.org/licenses/>.

// ---------- Timing (IEP @ ~250 MHz => 4 ns/tick) ----------
#define DSHOT_PERIOD_TICKS   417     // 1.668 us
#define DSHOT_T0H_TICKS      156     // 0.624 us
#define DSHOT_T1H_TICKS      313     // 1.252 us
#define DSHOT_FRAME_TICKS    250000  // 1 ms (1 kHz update)

// ---------- FAILSAFE --------
#define TICK_PER_MS     250000
#define FAILSAFE_PERIOD (1000 * TICK_PER_MS)

// While waiting for DSHOT edges, we can service ECAP as long as we are not close to the deadline.
// 1000 ticks = 4.0 us guard (conservative for SBUS).
#define ECAP_GUARD_TICKS     1000

// ---------- PRU Constant Table ----------
#define ECAP C3
#define RAM  C24
#define IEP  C26

// ---------- IEP ----------
#define IEP_TMR_GLB_CFG 0x0
#define COUNT_REG0      0x10
#define COUNT_REG1      0x14
#define CMP_CFG_REG     0x70

// ---------- ECAP (only what we use) ----------
#define ECAP_CAP1   0x8
#define ECAP_ECCTL1 0x28
#define ECAP_ECEINT 0x2c
#define ECAP_ECCLR  0x30

// ECCTL1 bit positions (AM62x TRM, eCAP)
#define ECAP_CAP1POL     0
#define ECAP_CTRRST1     1
#define ECAP_CAP2POL     2
#define ECAP_CTRRST2     3
#define ECAP_CAPLDEN     8
#define ECAP_STOP_WRAP   17
#define ECAP_TSCTRSTOP   20
#define ECAP_SYNCO_SEL   22

// ECCLR/ECEINT bits
#define ECAP_CEVT1 1
#define ECAP_CEVT2 2

// ---------- Shared RAM Layout (must match host) ----------
#define CH_ENABLE_RAM_OFFSET (0 * 4)

// ---------- Failsafe memory space ---------
#define FAILSAFE_RAM_OFFSET  (1 * 4)

// For each channel, host provides:
//   bitpos   (unused by this PRU version)
//   ch_frame (lower 16 bits used)
#define CH_1_CURRENT_BIT_POSITION_RAM_OFFSET (2 * 4)
#define CH_1_FRAME_RAM_OFFSET                (3 * 4)
#define CH_2_CURRENT_BIT_POSITION_RAM_OFFSET (4 * 4)
#define CH_2_FRAME_RAM_OFFSET                (5 * 4)
#define CH_3_CURRENT_BIT_POSITION_RAM_OFFSET (6 * 4)
#define CH_3_FRAME_RAM_OFFSET                (7 * 4)
#define CH_4_CURRENT_BIT_POSITION_RAM_OFFSET (8 * 4)
#define CH_4_FRAME_RAM_OFFSET                (9 * 4)
#define CH_5_CURRENT_BIT_POSITION_RAM_OFFSET (10 * 4)
#define CH_5_FRAME_RAM_OFFSET                (11 * 4)
#define CH_6_CURRENT_BIT_POSITION_RAM_OFFSET (12 * 4)
#define CH_6_FRAME_RAM_OFFSET                (13 * 4)

// Ringbuffer (SBUS capture durations)
#define RCIN_RING_HEAD_OFFSET 0x1000
#define RCIN_RING_TAIL_OFFSET 0x1002
#define RCIN_RINGBUFFER_RAM_OFFSET 0x1004
#define RCIN_RINGBUFFERSIZE 300

// ---------- RCOut pins ----------
#define RC_CH_1_PIN r30.t7
#define RC_CH_2_PIN r30.t4
#define RC_CH_3_PIN r30.t1
#define RC_CH_4_PIN r30.t5
#define RC_CH_5_PIN r30.t2
#define RC_CH_6_PIN r30.t6

// ---------- Register struct (keep small; occupies R4.. ) ----------
.struct RegisterStruct
   .u32 ch_enable
   .u32 ch_1_next_time
   .u32 ch_2_next_time
   .u32 ch_3_next_time
   .u32 ch_4_next_time
   .u32 ch_5_next_time
   .u32 ch_6_next_time
   .u32 next_failsafe
   .u32 time
   .u32 rcin_ram_pointer
   .u32 rcin_ram_pointer_index
   .u32 rcin_ram_pointer_index_max
   .u32 rcin_ram_pointer_head
   .u32 rcin_ram_pointer_tail
   .u32 temp
   .u32 temp1
 .ends
 .assign RegisterStruct, R4, *, register

// ---------- Helpers ----------

// Load 32-bit immediate (hi16:lo16)
.macro LDI32
.mparam DST, IMM32
    ldi DST, (IMM32 & 0xFFFF)
    ldi register.temp, ((IMM32 >> 16) & 0xFFFF)
    lsl register.temp, register.temp, 16
    or  DST, DST, register.temp
.endm

.macro RCIN_WRITE_HEAD
.mparam RCIN_HEAD
   sbbo RCIN_HEAD, register.rcin_ram_pointer_head, 0, 2
.endm

.macro RCIN_WRITE_TAIL
.mparam RCIN_TAIL
   sbbo RCIN_TAIL, register.rcin_ram_pointer_tail, 0, 2
.endm

// ---------- ECAP init for "difference mode": CAP1=low, CAP2=high durations ----------
.macro RCIN_ECAP_INIT
   // CAP1: falling edge, reset counter
   // CAP2: rising edge, reset counter
   // CAPLDEN enabled, STOP_WRAP=wrap after event2, TSCTRSTOP=run, SYNCO disabled
   mov register.temp, (1 << ECAP_CTRRST1) | (1 << ECAP_CAP1POL) | (1 << ECAP_CTRRST2) | (1 << ECAP_CAPLDEN) | (1 << ECAP_STOP_WRAP) | (1 << ECAP_TSCTRSTOP) | (2 << ECAP_SYNCO_SEL)
   sbco register.temp, ECAP, ECAP_ECCTL1, 8
.endm

// Fast ECAP service (for SBUS edge bursts)
.macro RCIN_ECAP_FAST
   lbco register.temp, ECAP, ECAP_ECEINT, 8
   qbbc rcin_ecap_fast_end, register.temp.t18

      // CAP1/CAP2 into temp/temp1
      lbco register.temp, ECAP, ECAP_CAP1, 8

      // store 8 bytes (low/high durations)
      sbbo register.temp, register.rcin_ram_pointer, 0, 8

      // clear flags
      mov register.temp, (1 << ECAP_CEVT1) | (1 << ECAP_CEVT2)
      sbco register.temp, ECAP, ECAP_ECCLR, 4

      // advance ringbuffer
      RCIN_WRITE_TAIL register.rcin_ram_pointer_index
      add register.rcin_ram_pointer_index, register.rcin_ram_pointer_index, 1
      add register.rcin_ram_pointer, register.rcin_ram_pointer, 8

      qblt rcin_ecap_fast_end, register.rcin_ram_pointer_index_max, register.rcin_ram_pointer_index
         mov register.rcin_ram_pointer, RCIN_RINGBUFFER_RAM_OFFSET
         mov register.rcin_ram_pointer_index, 0

rcin_ecap_fast_end:
.endm

// Optional: once-per-loop service (kept for readability)
.macro RCIN_ECAP
   RCIN_ECAP_FAST
.endm

// Wait until absolute time in r29, servicing ECAP when there's time margin.
.macro WAIT_UNTIL_ECAP_R29
wait_until_loop:
    lbco register.temp, IEP, COUNT_REG0, 4            // now
    sub  register.temp1, r29, register.temp           // remaining = target - now
    qbbs wait_reached, register.temp1.t31             // if now >= target

    ldi  register.temp, ECAP_GUARD_TICKS
    qbgt wait_until_loop, register.temp, register.temp1 // if remaining < guard -> spin

      RCIN_ECAP_FAST

    jmp wait_until_loop
wait_reached:
.endm

// ---------- DSHOT600 (full anchored start/high/end) ----------
// Assumes:
//   r25 = period ticks
//   r26 = T0H ticks
//   r27 = T1H ticks
// Uses: r0 (frame), r23 (t_start), r24 (count), r29 (target)
.macro SEND_DSHOT16_IEP_FULL_ANCHORED
.mparam PIN_T

    lbco r23, IEP, COUNT_REG0, 4
    ldi  r24, 16

send_bit_loop:
    // Anchor bit start
    mov r29, r23
    WAIT_UNTIL_ECAP_R29

    set PIN_T

    // TH selection (stored in register.temp1)
    mov register.temp1, r26                 // default T0H
    qbbc th_done, r0.t15
        mov register.temp1, r27             // T1H
        mov r23, r23                        // filler
th_done:
    mov r23, r23                            // filler

    // High end: t_start + TH
    add r29, r23, register.temp1
    WAIT_UNTIL_ECAP_R29

    clr PIN_T

    // Bit end: t_start + PERIOD
    add r29, r23, r25
    WAIT_UNTIL_ECAP_R29

    // Next bit start
    add r23, r23, r25

    // MSB-first
    lsl r0, r0, 1

    sub r24, r24, 1
    qbne send_bit_loop, r24, 0
.endm

// Per-channel frame send @ 1kHz (r28 holds DSHOT_FRAME_TICKS)
.macro RCOUT_DSHOT_FRAME
.mparam PIN_T, CH_X_NEXT_TIME, CH_X_ENABLE, CH_X_FRAME_RAM_OFFSET

    qbbc dshotf_end, CH_X_ENABLE

    // time fresh
    lbco register.time, IEP, COUNT_REG0, 4

    // due? dt = time - next_time ; if underflow => not due
    sub register.temp, register.time, CH_X_NEXT_TIME
    qbbs dshotf_end, register.temp.t31

    // schedule next frame
    add CH_X_NEXT_TIME, register.time, r28

    // load frame (lower 16 bits)
    lbco r0, RAM, CH_X_FRAME_RAM_OFFSET, 4
    ldi  register.temp, 0xFFFF
    and  r0, r0, register.temp

    SEND_DSHOT16_IEP_FULL_ANCHORED PIN_T

dshotf_end:
.endm

// ---------- INIT ----------
.macro INIT
    // clear outputs
    mov r30, 0

    // clear register struct
    zero &register, SIZE(register)

    // ringbuffer init
    mov register.rcin_ram_pointer, RCIN_RINGBUFFER_RAM_OFFSET
    mov register.rcin_ram_pointer_index_max, RCIN_RINGBUFFERSIZE
    mov register.rcin_ram_pointer_head, RCIN_RING_HEAD_OFFSET
    mov register.rcin_ram_pointer_tail, RCIN_RING_TAIL_OFFSET
    mov register.temp, 0
    RCIN_WRITE_HEAD register.temp
    RCIN_WRITE_TAIL register.temp

    // stagger channel frame times
    mov register.ch_1_next_time, 1000
    mov register.ch_2_next_time, 2000
    mov register.ch_3_next_time, 3000
    mov register.ch_4_next_time, 4000
    mov register.ch_5_next_time, 5000
    mov register.ch_6_next_time, 6000

    // disable channels
    mov register.ch_enable, 0
    sbco register.ch_enable, RAM, CH_ENABLE_RAM_OFFSET, 4

    // initialize frames to DSHOT "stop" (value=48, telem=0)
    ldi register.temp, 0x0606
    sbco register.temp, RAM, CH_1_FRAME_RAM_OFFSET, 4
    sbco register.temp, RAM, CH_2_FRAME_RAM_OFFSET, 4
    sbco register.temp, RAM, CH_3_FRAME_RAM_OFFSET, 4
    sbco register.temp, RAM, CH_4_FRAME_RAM_OFFSET, 4
    sbco register.temp, RAM, CH_5_FRAME_RAM_OFFSET, 4
    sbco register.temp, RAM, CH_6_FRAME_RAM_OFFSET, 4

    // also clear bitpos words (host still writes them; PRU ignores them)
    mov register.temp, 0
    sbco register.temp, RAM, CH_1_CURRENT_BIT_POSITION_RAM_OFFSET, 4
    sbco register.temp, RAM, CH_2_CURRENT_BIT_POSITION_RAM_OFFSET, 4
    sbco register.temp, RAM, CH_3_CURRENT_BIT_POSITION_RAM_OFFSET, 4
    sbco register.temp, RAM, CH_4_CURRENT_BIT_POSITION_RAM_OFFSET, 4
    sbco register.temp, RAM, CH_5_CURRENT_BIT_POSITION_RAM_OFFSET, 4
    sbco register.temp, RAM, CH_6_CURRENT_BIT_POSITION_RAM_OFFSET, 4

    // IEP timer: disable, reset, enable
    lbco register.temp, IEP, IEP_TMR_GLB_CFG, 4
    clr register.temp.t0
    sbco register.temp, IEP, IEP_TMR_GLB_CFG, 4

    lbco register.temp, IEP, CMP_CFG_REG, 4
    mov register.temp, 0x20000
    sbco register.temp, IEP, CMP_CFG_REG, 4

    mov register.temp, 0xffffffff
    sbco register.temp, IEP, COUNT_REG0, 4
    sbco register.temp, IEP, COUNT_REG1, 4

    mov register.temp, 0x0111
    sbco register.temp, IEP, IEP_TMR_GLB_CFG, 2

    // preload constants
    ldi r25, DSHOT_PERIOD_TICKS
    ldi r26, DSHOT_T0H_TICKS
    ldi r27, DSHOT_T1H_TICKS
    LDI32 r28, DSHOT_FRAME_TICKS
.endm

// -------------- FAILSAFE MACRO 
// Ensure if ardupilot die the pru will stop
.macro FAILSAFE_HANDLING
   sub register.temp, register.next_failsafe, register.time
   mov register.temp1, 0xF0000000
   qbgt failsafeend, register.temp, register.temp1
   mov register.temp, FAILSAFE_PERIOD
   add register.next_failsafe, register.time, register.temp
   lbco register.temp, RAM, FAILSAFE_RAM_OFFSET, 4
   qbbs failsafe_succesful, register.temp.t0
   sbco register.temp, RAM, CH_ENABLE_RAM_OFFSET, 4
failsafe_succesful:
   ldi register.temp, 0
   sbco register.temp, RAM, FAILSAFE_RAM_OFFSET, 4
failsafeend:
.endm

.origin 0
start:
    INIT
    RCIN_ECAP_INIT

mainloop:
    RCIN_ECAP
    FAILSAFE_HANDLING

    // load enables (shared RAM)
    lbco register.ch_enable, RAM, CH_ENABLE_RAM_OFFSET, 4

    // send frames
    RCOUT_DSHOT_FRAME RC_CH_1_PIN, register.ch_1_next_time, register.ch_enable.t0, CH_1_FRAME_RAM_OFFSET
    RCOUT_DSHOT_FRAME RC_CH_2_PIN, register.ch_2_next_time, register.ch_enable.t1, CH_2_FRAME_RAM_OFFSET
    RCOUT_DSHOT_FRAME RC_CH_3_PIN, register.ch_3_next_time, register.ch_enable.t2, CH_3_FRAME_RAM_OFFSET
    RCOUT_DSHOT_FRAME RC_CH_4_PIN, register.ch_4_next_time, register.ch_enable.t3, CH_4_FRAME_RAM_OFFSET
    RCOUT_DSHOT_FRAME RC_CH_5_PIN, register.ch_5_next_time, register.ch_enable.t4, CH_5_FRAME_RAM_OFFSET
    RCOUT_DSHOT_FRAME RC_CH_6_PIN, register.ch_6_next_time, register.ch_enable.t5, CH_6_FRAME_RAM_OFFSET

    jmp mainloop
