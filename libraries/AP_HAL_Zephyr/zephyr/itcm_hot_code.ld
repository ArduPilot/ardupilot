/*
 * Hot ArduPilot code placed in ITCM (instruction tightly-coupled memory).
 * Everything here otherwise executes XIP from external FlexSPI NOR; ITCM is
 * zero-wait-state at core speed. Measured 2026-08-01: ITCM was 0.00% used.
 */

/* EKF3 - the dominant math consumer; the reason this file exists.
 *
 * EVICTED AND RESTORED 2026-09-26, within the hour. The eviction freed 107,532
 * bytes and paid for the position controller, the navigation math, the GPS
 * parser, AP_Param, CRSF telemetry and GCS_MAVLINK. The result on silicon:
 * MAIN LOOP 190 Hz -> 83 Hz, followed by a reboot loop with fault 25
 * (K_ERR_ARM_BUS_PRECISE_DATA_BUS) in the main thread. Whatever those additions
 * were worth, they were worth less than a fifth of the region spent on EKF3.
 * Do not evict it again without being ready to measure the loop rate inside a
 * minute and revert. */
*libArduCopter_libs.a:AP_NavEKF3*.o(.text .text.*)

/* AP_Math - vector/matrix/quaternion helpers called from EKF3 and the IMU path. */
*libArduCopter_libs.a:AP_Math*.o(.text .text.*)
*libArduCopter_libs.a:matrix*.o(.text .text.*)
*libArduCopter_libs.a:vector*.o(.text .text.*)
*libArduCopter_libs.a:quaternion*.o(.text .text.*)

/* Filter - notch/LPF, runs at IMU sample rate. */
*libArduCopter_libs.a:*Filter*.o(.text .text.*)

    /* SPI transaction hot path (2026-08-01). */

/* Zephyr's LPSPI driver - spi_transceive() and the transfer state machine. */
*libdrivers__spi.a:*(.text .text.*)
*spi_nxp_lpspi*.obj(.text .text.*)

/* AP HAL: SPIDevice::transfer(), DeviceBus bus threads, I2CDevice. */
*libArduCopter_libs.a:SPIDevice*.o(.text .text.*)
*libArduCopter_libs.a:DeviceBus*.o(.text .text.*)
*libArduCopter_libs.a:I2CDevice*.o(.text .text.*)

    /* I2C driver + MCUX SDK. Asymmetry found 2026-08-05: the SPI equivalents were
       already relocated but the I2C ones were not. */
*libdrivers__i2c.a:*(.text .text.*)
*fsl_lpi2c.c.obj(.text .text.*)
*libArduCopter_libs.a:Scheduler*.o(.text .text.*)

/* IMU drivers - the callers, running at sample rate. */
*libArduCopter_libs.a:AP_InertialSensor*.o(.text .text.*)

    /* The flight-path callee chain (2026-08-06): the functions the hot ISRs call.
       Useless to relocate without their callers, and vice versa. */
*libArduCopter_libs.a:AC_AttitudeControl*.o(.text .text.*)
*libArduCopter_libs.a:AP_AHRS*.o(.text .text.*)
*libArduCopter_libs.a:AC_PID*.o(.text .text.*)

    /* System timer driver - tested 2026-08-06/07. Verdict: NO CLEAR EFFECT, and
       the first apparent regression was a measurement error. Recorded so the
       next person does not spend the day re-testing it. */


    /* Zephyr kernel (2026-08-01). */
*libkernel.a:*(.text .text.*)

*libdrivers__timer.a:*(.text .text.*)

    /* ARM arch context-switch path (2026-08-08). libkernel was already in ITCM but
       the ARCH archive never was, so the switch path itself still ran XIP. */
*libarch__arm__core__cortex_m.a:swap_helper.S.obj(.text .text.*)
*libarch__arm__core__cortex_m.a:exc_exit.c.obj(.text .text.*)
*libarch__arm__core__cortex_m.a:isr_wrapper.c.obj(.text .text.*)
*libarch__arm__core__cortex_m.a:irq_manage.c.obj(.text .text.*)
*libarch__arm__core__cortex_m.a:svc.S.obj(.text .text.*)

    /* 64-bit division helpers (2026-08-08), from the same PCSR run. */
*libgcc.a:_udivmoddi4.o(.text .text.*)
*libgcc.a:_aeabi_uldivmod.o(.text .text.*)

    /* Second PCSR round (2026-08-08), after the arch switch path went in. */
*libArduCopter_libs.a:BatchSampler*.o(.text .text.*)
*libArduCopter_libs.a:Semaphores*.o(.text .text.*)
*libArduCopter_libs.a:UARTDriver*.o(.text .text.*)
*libArduCopter_libs.a:AP_Scheduler*.o(.text .text.*)
*libArduCopter_libs.a:AP_DAL*.o(.text .text.*)
*libArduCopter_libs.a:system*.o(.text .text.*)
*libarch__arm__core__cortex_m.a:__aeabi_read_tp.S.obj(.text .text.*)

    /* Third PCSR round (2026-08-08): loop 118.6 -> 195.0 Hz. */
*libArduCopter_libs.a:RingBuffer*.o(.text .text.*)
*libArduCopter_libs.a:SlewLimiter*.o(.text .text.*)
/* AP_Vehicle dropped 2026-08-08 to make room for the libm hot set below -
   it earned 1.62% of PC samples when added; the libm set measures ~13%. */
/* AP_Logger dropped 2026-08-08 when the DMA arm made the eDMA objects real
   and ITCM overflowed by 1488 B; RESTORED the same day after the DMA re-park
   freed ~23 KB (337,208 B used of 352 KB without it). io_thread measured
   2.69% of PC samples in XIP when first added. */
*libArduCopter_libs.a:AP_Logger*.o(.text .text.*)
*libdrivers__serial.a:*(.text .text.*)

    /* Fourth PCSR round (2026-08-08): loop 195.0 -> 242.1 Hz. */
*libArduCopter_libs.a:Device*.o(.text .text.*)
*libArduCopter_libs.a:control*.o(.text .text.*)

    /* eDMA driver (2026-08-07). Same asymmetry as the I2C case above: the ISR was
       relocated, the callee chain it enters was not. */
*libdrivers__dma.a:*(.text .text.*)
*dma_mcux_edma*.obj(.text .text.*)
*fsl_edma*.c.obj(.text .text.*)

    /* The whole vehicle archive (2026-08-07), once the FlexRAM rebalance freed
       the space for it. */
*libarducopter.a:*(.text .text.*)

    /* libm, correctly this time (2026-08-08): the earlier rounds' *libm.a pattern
       matched nothing, so the math functions were never actually relocated. */
*libc.a:libm_math_sf_pow.c.o(.text .text.*)
*libc.a:libm_math_sf_sin.c.o(.text .text.*)
*libc.a:libm_math_sf_cos.c.o(.text .text.*)
*libc.a:libm_math_sf_atan.c.o(.text .text.*)
*libc.a:libm_math_sf_atan2.c.o(.text .text.*)
*libc.a:libm_math_sf_fmod.c.o(.text .text.*)
*libc.a:libm_math_sf_rem_pio2.c.o(.text .text.*)
*libc.a:libm_math_kf_sin.c.o(.text .text.*)
*libc.a:libm_math_kf_cos.c.o(.text .text.*)
*libc.a:libm_math_kf_rem_pio2.c.o(.text .text.*)
*libc.a:libm_math_s_sin.c.o(.text .text.*)
*libc.a:libm_math_s_cos.c.o(.text .text.*)
*libc.a:libm_math_s_rem_pio2.c.o(.text .text.*)
*libc.a:libm_math_k_sin.c.o(.text .text.*)
*libc.a:libm_math_k_cos.c.o(.text .text.*)
*libc.a:libm_math_k_rem_pio2.c.o(.text .text.*)

    /* Motors hot path (2026-08-08, PCSR at 370 Hz). */
    /* Thrust_Linearization tried and reverted the same day: p50 365.8 Hz against
       379.3 without it. */
/* AP_MotorsMulticopter also dropped: still overflowed without Matrix. Only
   the 1.1 KB powf caller fits until the next FlexRAM step frees headroom. */
/* AP_MotorsMatrix dropped same day: overflowed the region as predicted. */

    /* 480 KB era (2026-08-08): the AP_Motors + SRV call graph, whole. */
*libArduCopter_libs.a:AP_Motors*.o(.text .text.*)
*libArduCopter_libs.a:SRV_Channel*.o(.text .text.*)
*libArduCopter_libs.a:RC_Channel*.o(.text .text.*)
*libArduCopter_libs.a:AP_Vehicle*.o(.text .text.*)
*libArduCopter_libs.a:RCOutput*.o(.text .text.*)
*libc.a:libm_math_s_atan.c.o(.text .text.*)

    /* Moved out of .ramfunc (OCRAM) 2026-09-26 to shrink that region's
     * power-of-two slot and give the OCRAM malloc arena its 64 KB back. Both
     * are hot in their own right: SCurve runs every loop under AC_PosControl,
     * AP_Param is every parameter read plus the io thread's save queue. */
*libArduCopter_libs.a:SCurve*.o(.text .text.*)
*libArduCopter_libs.a:AP_Param.cpp*.o(.text .text.*)
*libArduCopter_libs.a:AP_ParamT*.o(.text .text.*)
*libc.a:libm_math_s_atan2.c.o(.text .text.*)

    /* GyroFFT / CMSIS-DSP used to live here and held ~53 KB of ITCM. The
     * board now builds with HAL_GYROFFT_ENABLED 0 (see hwdef.dat), so there is
     * nothing to place and the space goes to code that runs every loop.
     * If FFT is ever turned back on it MUST come back here with it: with the
     * FFT enabled and its code left in XIP the loop ran 57-72 Hz, and putting
     * it in ITCM was what restored 190 Hz. */

    /* Fifth PCSR round (2026-08-12): CAN/DroneCAN, RC protocol scan, PWM. */
*libArduCopter_libs.a:CANIface.cpp.1.o(.text .text.* .rodata .rodata.*)
*can_mcux_flexcan.c.obj(.text .text.* .rodata .rodata.*)
*libArduCopter_libs.a:AP_Canard_iface.cpp.1.o(.text .text.* .rodata .rodata.*)
*libArduCopter_libs.a:AP_DroneCAN.cpp.1.o(.text .text.* .rodata .rodata.*)
*libArduCopter_libs.a:canard.c.1.o(.text .text.* .rodata .rodata.*)
*fsl_flexcan.c.obj(.text .text.* .rodata .rodata.*)
*can_common.c.obj(.text .text.* .rodata .rodata.*)
*libArduCopter_libs.a:AP_RCProtocol.cpp.2.o(.text .text.* .rodata .rodata.*)
*pwm_mcux.c.obj(.text .text.* .rodata .rodata.*)
*ring_buffer.c.obj(.text .text.* .rodata .rodata.*)
*libArduCopter_libs.a:EKFGSF_yaw.cpp.1.o(.text .text.* .rodata .rodata.*)
*libArduCopter_libs.a:LogFile.cpp.1.o(.text .text.* .rodata .rodata.*)
*libc.a:libm_math_sf_acos.c.o(.text .text.* .rodata .rodata.*)
*libc.a:libm_math_sf_asin.c.o(.text .text.* .rodata .rodata.*)
/* AC_PosControl DROPPED 2026-08-13 (the round-5 notes' own nominated
   first-drop: 1.13% of PC for 9.7KB, worst value in that batch) to make
   room for the RC QTMR hardware-capture code growth in RCInput.cpp -
   the link overflowed ITCM by 572 B with it in. */

    /* Sixth PCSR round (2026-08-12): the last few bytes of the region. */
*libArduCopter_libs.a:LoggerMessageWriter.cpp.2.o(.text .text.* .rodata .rodata.*)
*libArduCopter_libs.a:AP_Baro_atmosphere.cpp.1.o(.text .text.* .rodata .rodata.*)
*libArduCopter_libs.a:GCS.cpp.2.o(.text .text.* .rodata .rodata.*)
/* AP_DroneCAN_serial.cpp.1.o DROPPED 2026-08-13 (the drop order above named
 * it first, worst %/byte of the round-6 batch, 2240 B for 1.02%): the UART
 * port-options work (generalized RXINV/TXINV + _ap_lpuart_base lookup in
 * UARTDriver.cpp, an ITCM resident) grew the region 544 B past the ceiling. */

    /* PPM-SUM edge-capture ISR path (2026-08-13). Interrupt-latency jitter on the
       RC timestamp, not loop throughput. */
*libdrivers__gpio.a:gpio_mcux_igpio.c.obj(.text .text.*)
*libArduCopter_libs.a:RCInput.cpp*.o(.text .text.*)

    /* THE FAULT PATH (2026-09-26). Not for speed - for being fetchable at all.
       A fault whose cause is the FlexSPI controller leaves the CPU unable to
       read XIP flash, and every one of these objects was IN that flash:

         30098fb0 z_arm_fault        30099188 z_arm_usage_fault
         30097e48 k_sys_fatal_error_handler

       So the core took a UsageFault, could not fetch the handler for it,
       escalated to HardFault, could not fetch that either, and locked up until
       the 2 s watchdog reset it - losing the fault record every time. Observed
       on silicon as "WDG: ... FT0 FA0 FTP0 FLR0 FICSR0": a watchdog reset with
       nothing recorded, next to otherwise identical resets that DID record
       reason 35, the difference being only whether flash happened to read back
       in time.

       The chain is vectors -> z_arm_usage_fault (fault_s.S) -> z_arm_fault
       (fault.c) -> z_arm_fatal_error (fatal.c) -> z_fatal_error (libkernel,
       already here) -> k_sys_fatal_error_handler (ap_fault_handler.c). All of
       it has to be resident, and so does its rodata: the dump reads its own
       strings. */
*libarch__arm__core__cortex_m.a:fault_s.S.obj(.text .text.* .rodata .rodata.*)
*libarch__arm__core__cortex_m.a:fault.c.obj(.text .text.* .rodata .rodata.*)
*libarch__arm__core.a:fatal.c.obj(.text .text.* .rodata .rodata.*)
*libapp.a:ap_fault_handler.c.obj(.text .text.* .rodata .rodata.*)

    /* USB into ITCM (2026-09-26). Not for speed. The USB device interrupt is the
       one that fires constantly - the host polls at 1 kHz - so it is the
       interrupt most likely to be taken the instant rt1176_flash_erase()
       releases its lock between chunks. Fetching that handler from the NOR that
       was just being erased is the worst possible moment to need the FlexSPI,
       and unlike a thread an ISR cannot be deferred. ~16 KB for all three. */
*libdrivers__usb__udc.a:*(.text .text.* .rodata .rodata.*)
*libsubsys__usb__device_next.a:*(.text .text.* .rodata .rodata.*)
*libdrivers__usb__common__buf.a:*(.text .text.* .rodata .rodata.*)

    /* FastRateBuffer (2026-09-26). The AP_InertialSensor*.o glob above does NOT
       match FastRateBuffer.cpp.3.o, so all seven fast-rate hand-off functions
       were left executing XIP from external NOR. The rate loop enters them four
       times per iteration and the IMU bus thread three times per gyro sample at
       1012 Hz, and they sat in two XIP regions 421 KB apart, so in separate
       I-cache line sets. Verified against the ELF: of 8552 functions in XIP
       text, these seven were the ONLY rate-loop symbols still there. Same class
       of pattern miss this file already records for libm. ~460 bytes. */
*libArduCopter_libs.a:FastRateBuffer*.o(.text .text.*)

    /* RC-IN HOT PATH (2026-09-26). The rcin thread measured 17-19% of the CPU.
       AP_RCProtocol.cpp and RCInput.cpp were already resident, but the BACKEND
       that actually decodes was not: every received byte made two virtual calls
       into XIP - process_byte() and new_input() on the locked protocol - at
       ~13 kB/s from a 500 Hz CRSF link, so ~26,000 XIP entries per second.
       Verified: all 26 AP_RCProtocol_CRSF functions were in XIP, and
       process_byte (0x300c98ec) sat 360 KB from _process_byte (0x30121242), in
       separate I-cache line sets. Only ~2.2 KB of ITCM for the pair.
       Measure the result - XIP placement misses in this file have repeatedly
       come out larger than predicted, and 19% is far more than call overhead
       alone explains, so this may well be only part of it. */
*libArduCopter_libs.a:AP_RCProtocol_CRSF*.o(.text .text.*)
*libArduCopter_libs.a:AP_RCProtocol_Backend*.o(.text .text.*)

    /* REVERTED with the EKF3 eviction (2026-09-26): AC_PosControl, SCurve,
       AC_WPNav, AC_Loiter, AC_Circle, AP_GPS, AP_GPS_UBLOX, GPS_Backend,
       AP_Param, AP_CRSF_Telem, GCS_MAVLINK and GCS_Common all went in here when
       EKF3 came out, and all came back out when that halved the loop rate. They
       are still XIP. If ITCM headroom ever appears, AC_PosControl is the best
       candidate of them - it is genuinely per-loop code.

       WHAT WAS KEPT, because it was measured and cheap: the RC-in decode chain
       below. rcin went from 17% of CPU to 2% for about 3.4 KB. */

    /* Fourth PCSR/PC-sample round (2026-09-27). Measured with the in-firmware PC
       profiler at ~1 kHz over 10052 samples: itcm=42% xip=57%, and the single
       HOTTEST 64-byte bucket in the whole system was 0x300b4740 at 523/10052 =
       5.2%, which resolves to the AP_InternalError region (errors_as_string at
       0x300b4718+100, plus AP::internalerror() at 0x300b477c).

       That cannot be algorithmic work: errors_as_string() is called from exactly
       one place, AP_Arming.cpp:1203, at roughly 1 Hz. A 100-byte function called
       once a second is not 5% of a 996 MHz CPU. The samples are INSTRUCTION FETCH
       STALLS - the CPU parked at that PC waiting on FlexSPI. The two veneers that
       also showed up hot (__k_heap_init_veneer, __k_queue_append_veneer) are the
       corroboration: veneers exist only because ITCM-resident code branches to
       XIP targets.

       AP_InternalError.cpp is 681 bytes of text, and ITCM had 30.8 KB free, so
       this is the cheapest possible test of that reading. The Zephyr ARM cache
       ops go in with it: arch_dcache_flush_range is called on every SPI DMA
       transfer (~850/s from AP_HAL_Zephyr/SPIDevice.cpp:45) and its bucket was
       ~2%, for 464 bytes total across all arch_dcache/arch_icache entries. */
*libArduCopter_libs.a:AP_InternalError*.o(.text .text.*)
*libarch__arm__core__cortex_m.a:cache.c.obj(.text .text.*)
