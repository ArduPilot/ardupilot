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


// RC AllInOnePRU
//
// 1 channel RCInput with 5ns accuracy
// 6 channel RCOutput with 1us accuracy

// Timer
#define TICK_PER_US 250
#define TICK_PER_MS 250000

// Period within which the failsafe bit has to
// be set.
#define FAILSAFE_PERIOD (1000 * TICK_PER_MS)

// PWM
// 0 us
#define PWM_PULSE_DEFAULT (0 * TICK_PER_US)

// 50 Hz
#define PWM_FREQ_DEFAULT (20 * TICK_PER_MS)

// Ringbuffer size
#define RCIN_RINGBUFFERSIZE 300

// PRU Constants Table
// -> 4.4.1.1 Constants Table in TRM
#define ECAP C3
#define RAM C24
#define IEP C26

// IEP
// -> 4.5.4 PRU_ICSS_IEP Registers in TRM
#define IEP_TMR_GLB_CFG 0x0
#define COUNT_REG0      0x10
#define COUNT_REG1      0x14
#define CMP_CFG_REG     0x70

// ECAP
// -> 15.3.4.1 ECAP Registers in TRM
#define ECAP_TSCTR 0x0
#define ECAP_CTRPHS 0x4
#define ECAP_CAP1 0x8
#define ECAP_CAP2 0xc
#define ECAP_CAP3 0x10
#define ECAP_CAP4 0x14
#define ECAP_ECCTL1 0x28
#define ECAP_ECEINT 0x2c
#define ECAP_ECCLR 0x30
#define ECAP_REVID 0x5c

// ECCTL1
// -> 15.3.4.1.7 ECCTL1 Register in TRM
#define ECAP_CAP1POL 0
#define ECAP_CTRRST1 1
#define ECAP_CAP2POL 2
#define ECAP_CTRRST2 3
#define ECAP_CAP3POL 4
#define ECAP_CTRRST3 5
#define ECAP_CAP4POL 6
#define ECAP_CTRRST4 7
#define ECAP_CAPLDEN 8
#define ECAP_PRESCALE 9
#define ECAP_FREE_SOFT 14
#define ECAP_CONT_ONESHT 16
#define ECAP_STOP_WRAP 17
#define ECAP_RE_ARM 19
#define ECAP_TSCTRSTOP 20
#define ECAP_SYNCI_EN 21
#define ECAP_SYNCO_SEL 22
#define ECAP_SWSYNC 24
#define ECAP_CAP_APWM 25
#define ECAP_APWMPOL 26

// ECEINT, ECFLG
// -> 15.3.4.1.9 ECEINT Register in TRM
// -> 15.3.4.1.10 ECFLG Register in TRM
#define ECAP_INT 0
#define ECAP_CEVT1 1
#define ECAP_CEVT2 2
#define ECAP_CEVT3 3
#define ECAP_CEVT4 4
#define ECAP_CNTOVF 5
#define ECAP_PRDEQ 6
#define ECAP_CMPEQ 7

// RAM
#define CH_ENABLE_RAM_OFFSET (0 * 4)
#define FAILSAFE_RAM_OFFSET  (1 * 4)
#define CH_1_PULSE_TIME_RAM_OFFSET (2 * 4)
#define CH_1_T_TIME_RAM_OFFSET  (3 * 4)
#define CH_2_PULSE_TIME_RAM_OFFSET (4 * 4)
#define CH_2_T_TIME_RAM_OFFSET  (5 * 4)
#define CH_3_PULSE_TIME_RAM_OFFSET (6 * 4)
#define CH_3_T_TIME_RAM_OFFSET  (7 * 4)
#define CH_4_PULSE_TIME_RAM_OFFSET (8 * 4)
#define CH_4_T_TIME_RAM_OFFSET  (9 * 4)
#define CH_5_PULSE_TIME_RAM_OFFSET (10 * 4)
#define CH_5_T_TIME_RAM_OFFSET  (11 * 4)
#define CH_6_PULSE_TIME_RAM_OFFSET (12 * 4)
#define CH_6_T_TIME_RAM_OFFSET  (13 * 4)

#define RCIN_RING_HEAD_OFFSET 0x1000
#define RCIN_RING_TAIL_OFFSET 0x1002
#define RCIN_RINGBUFFER_RAM_OFFSET 0x1004

#define RC_CH_1_PIN r30.t7
#define RC_CH_2_PIN r30.t4
#define RC_CH_3_PIN r30.t1
#define RC_CH_4_PIN r30.t5
#define RC_CH_5_PIN r30.t2
#define RC_CH_6_PIN r30.t6

// RCOut enable bits
#define RC_CH_1_ENABLE register.ch_enable.t0
#define RC_CH_2_ENABLE register.ch_enable.t1
#define RC_CH_3_ENABLE register.ch_enable.t2
#define RC_CH_4_ENABLE register.ch_enable.t3
#define RC_CH_5_ENABLE register.ch_enable.t4
#define RC_CH_6_ENABLE register.ch_enable.t5

// Register struct
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
   .u32 time_max
   .u32 time_cycle
   .u32 rcin_ram_pointer
   .u32 rcin_ram_pointer_index
   .u32 rcin_ram_pointer_index_max
   .u32 rcin_ram_pointer_head
   .u32 rcin_ram_pointer_tail
   .u32 temp
   .u32 temp1
   .u32 test
 .ends
 .assign RegisterStruct, R4, *, register

.macro RCOUT_PWM
.mparam RC_CH_X_PIN, CH_X_NEXT_TIME, CH_X_ENABLE, CH_X_PULSE_TIME_RAM_OFFSET, CH_X_T_TIME_RAM_OFFSET
pwm:
   // Handle arithmetic and counter overflow, check if there is something to do
   sub register.temp, CH_X_NEXT_TIME, register.time
   mov register.temp1, 0xF0000000
   qbgt pwmend, register.temp, register.temp1

      mov CH_X_NEXT_TIME, register.time

      // Set pin or clear pin?
      qbbs pwmclear, RC_CH_X_PIN

         // Load pulse duration
         lbco register.temp, RAM, CH_X_PULSE_TIME_RAM_OFFSET, 4

         // Calculate time to next event
         add CH_X_NEXT_TIME, CH_X_NEXT_TIME, register.temp

         // Do not set pin if pulse time is 0
         qbeq pwmend, register.temp, 0

         // Check if channel is enabled
         qbbc pwmend, CH_X_ENABLE

            // Set pin
            set RC_CH_X_PIN
         jmp pwmend

      pwmclear:
         // Load pulse time
         lbco register.temp1, RAM, CH_X_PULSE_TIME_RAM_OFFSET, 4

         // Load T time
         lbco register.temp, RAM, CH_X_T_TIME_RAM_OFFSET, 4

         // Calculate time to next event (T - pulse duration)
         sub register.temp, register.temp, register.temp1
         add CH_X_NEXT_TIME, CH_X_NEXT_TIME, register.temp

         // Clear pin
         clr RC_CH_X_PIN
pwmend:
.endm

.macro RCIN_ECAP_INIT
   // Initialize ECAP
   // -> 15.3.4.1.7 ECCTL1 Register in TRM
   // ECAP_CTRRST1 = 1 : Reset counter after Event 1 time-stamp has been captured (used in difference mode operation)
   // ECAP_CAP1POL = 1 : Capture Event 1 triggered on a falling edge (FE)
   // ECAP_CAP2POL = 0 : Capture Event 2 triggered on a rising edge (RE) - default
   // ECAP_CTRRST2 = 1 : Reset counter after Event 2 time-stamp has been captured (used in difference mode operation)
   // ECAP_CAPLDEN = 1 : Enable CAP1-4 register loads at capture event time
   mov register.temp, (1 << ECAP_CTRRST1) | (1 << ECAP_CAP1POL) | (1 << ECAP_CTRRST2) | (1 << ECAP_CAPLDEN) | (1 << ECAP_STOP_WRAP) | (1 << ECAP_TSCTRSTOP) | (2 << ECAP_SYNCO_SEL)
   sbco register.temp, ECAP, ECAP_ECCTL1, 8

   // -> 15.3.4.1.8 ECCTL2 Register in TRM
   // ECAP_CONT_ONESHT = 0 : Operate in continuous mode - default
   // ECAP_STOP_WRAP = 1 : Wrap after Capture Event 2 in continuous mode
   // ECAP_TSCTRSTOP = 1 : TSCTR free-running
   // ECAP_SYNCO_SEL = 2 : Disable sync out signal
   // ECAP_CAP_APWM = 0 : ECAP module operates in capture mode - default
   //mov register.temp, (1 << ECAP_STOP_WRAP) | (1 << ECAP_TSCTRSTOP) | (2 << ECAP_SYNCO_SEL)
   //sbco register.temp, ECAP, ECAP_ECCTL1, 4
.endm

.macro RCIN_ECAP
   // New value?
   //lbco register.temp, ECAP, ECAP_ECFLG, 4
   lbco register.temp, ECAP, ECAP_ECEINT, 8
   qbbc rcin_ecap_end, register.temp.t18

      // Copy S0 and S1 duration to temp and temp1
      lbco register.temp, ECAP, ECAP_CAP1, 8

      // Copy S0 and S1 duration to RAM
      sbbo register.temp, register.rcin_ram_pointer, 0, 8

      // Clear event flags
      mov register.temp, (1 << ECAP_CEVT1) | (1 << ECAP_CEVT2)
      sbco register.temp, ECAP, ECAP_ECCLR, 4

      // Set new tail value
      RCIN_WRITE_TAIL register.rcin_ram_pointer_index

      // Update pointer
      add register.rcin_ram_pointer_index, register.rcin_ram_pointer_index, 1
      add register.rcin_ram_pointer, register.rcin_ram_pointer, 8

      // Check end of ringbuffer
      qblt rcin_ecap_end, register.rcin_ram_pointer_index_max, register.rcin_ram_pointer_index
         mov register.rcin_ram_pointer, RCIN_RINGBUFFER_RAM_OFFSET
         mov register.rcin_ram_pointer_index, 0
rcin_ecap_end:
.endm

.macro RCIN_WRITE_HEAD
.mparam RCIN_HEAD
   sbbo RCIN_HEAD, register.rcin_ram_pointer_head, 0, 2
.endm

.macro RCIN_WRITE_TAIL
.mparam RCIN_TAIL
   sbbo RCIN_TAIL, register.rcin_ram_pointer_tail, 0, 2
.endm

.macro INIT
   // Reset PWM pins
   mov r30, 0x0

   // Clear register
   zero &register, SIZE(register)

   // Initialize ringbuffer
   mov register.rcin_ram_pointer, RCIN_RINGBUFFER_RAM_OFFSET
   mov register.rcin_ram_pointer_index_max, RCIN_RINGBUFFERSIZE
   mov register.rcin_ram_pointer_head, RCIN_RING_HEAD_OFFSET
   mov register.rcin_ram_pointer_tail, RCIN_RING_TAIL_OFFSET
   mov register.temp, 0
   RCIN_WRITE_HEAD register.temp
   RCIN_WRITE_TAIL register.temp

   // Load balancing
   mov register.ch_1_next_time, 1000
   mov register.ch_2_next_time, 2000
   mov register.ch_3_next_time, 3000
   mov register.ch_4_next_time, 4000
   mov register.ch_5_next_time, 5000
   mov register.ch_6_next_time, 6000

   // Disable all PWMs
   mov register.ch_enable, 0x0
   sbco register.ch_enable, RAM, CH_ENABLE_RAM_OFFSET, 4

   // Initialize PWM pulse (0us)
   mov register.temp, PWM_PULSE_DEFAULT
   sbco register.temp, RAM, CH_1_PULSE_TIME_RAM_OFFSET, 4
   sbco register.temp, RAM, CH_2_PULSE_TIME_RAM_OFFSET, 4
   sbco register.temp, RAM, CH_3_PULSE_TIME_RAM_OFFSET, 4
   sbco register.temp, RAM, CH_4_PULSE_TIME_RAM_OFFSET, 4
   sbco register.temp, RAM, CH_5_PULSE_TIME_RAM_OFFSET, 4
   sbco register.temp, RAM, CH_6_PULSE_TIME_RAM_OFFSET, 4

   // Initialize PWM frequency (50Hz)
   mov register.temp, PWM_FREQ_DEFAULT
   sbco register.temp, RAM, CH_1_T_TIME_RAM_OFFSET, 4
   sbco register.temp, RAM, CH_2_T_TIME_RAM_OFFSET, 4
   sbco register.temp, RAM, CH_3_T_TIME_RAM_OFFSET, 4
   sbco register.temp, RAM, CH_4_T_TIME_RAM_OFFSET, 4
   sbco register.temp, RAM, CH_5_T_TIME_RAM_OFFSET, 4
   sbco register.temp, RAM, CH_6_T_TIME_RAM_OFFSET, 4

   // Disables the counter of IEP timer
   // -> 4.5.4.1 IEP_TMR_GLB_CFG Register in TRM
   // CNT_ENABLE = 0
   lbco register.temp, IEP, IEP_TMR_GLB_CFG, 4
   clr register.temp.t0
   sbco register.temp, IEP, IEP_TMR_GLB_CFG, 4
   
   //SHADOW enabled
   lbco register.temp, IEP, CMP_CFG_REG, 4
   mov register.temp, 0x20000
   sbco register.temp, IEP, CMP_CFG_REG, 4

   // Resets the counter of IEP timer
   // Reset Count Register (CNT) by writing 0xFFFFFFFF to clear
   // -> 7.4.9.2.4 Basic Programming Model in TRM
   mov register.temp, 0xffffffff
   sbco register.temp, IEP, COUNT_REG0, 4
   sbco register.temp, IEP, COUNT_REG1, 4

   // Configures the IEP counter and enables it
   // -> 4.5.4.1 IEP_TMR_GLB_CFG Register in TRM
   // -> Reference: https://github.com/beagleboard/am335x_pru_package/blob/master/pru_sw/example_apps/PRU_industrialEthernetTimer/PRU_industrialEthernetTimer.p
	// CMP_INC     = 1
	// DEFAULT_INC = 1
	// CNT_ENABLE  = 1
	mov  register.temp, 0x0111
	sbco register.temp, IEP, IEP_TMR_GLB_CFG, 2
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
init:
   INIT
   RCIN_ECAP_INIT
mainloop:
   
   FAILSAFE_HANDLING

   lbco register.ch_enable, RAM, CH_ENABLE_RAM_OFFSET, 4
   lbco register.time, IEP, COUNT_REG0, 4

   RCOUT_PWM RC_CH_1_PIN, register.ch_1_next_time, RC_CH_1_ENABLE, CH_1_PULSE_TIME_RAM_OFFSET, CH_1_T_TIME_RAM_OFFSET
   RCOUT_PWM RC_CH_2_PIN, register.ch_2_next_time, RC_CH_2_ENABLE, CH_2_PULSE_TIME_RAM_OFFSET, CH_2_T_TIME_RAM_OFFSET
   RCOUT_PWM RC_CH_3_PIN, register.ch_3_next_time, RC_CH_3_ENABLE, CH_3_PULSE_TIME_RAM_OFFSET, CH_3_T_TIME_RAM_OFFSET
   RCOUT_PWM RC_CH_4_PIN, register.ch_4_next_time, RC_CH_4_ENABLE, CH_4_PULSE_TIME_RAM_OFFSET, CH_4_T_TIME_RAM_OFFSET
   RCOUT_PWM RC_CH_5_PIN, register.ch_5_next_time, RC_CH_5_ENABLE, CH_5_PULSE_TIME_RAM_OFFSET, CH_5_T_TIME_RAM_OFFSET
   RCOUT_PWM RC_CH_6_PIN, register.ch_6_next_time, RC_CH_6_ENABLE, CH_6_PULSE_TIME_RAM_OFFSET, CH_6_T_TIME_RAM_OFFSET

   RCIN_ECAP
   
jmp mainloop