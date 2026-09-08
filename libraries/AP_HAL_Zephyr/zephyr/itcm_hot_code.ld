/*
 * Hot ArduPilot code placed in ITCM (instruction tightly-coupled memory).
 * Everything here otherwise executes XIP from external FlexSPI NOR; ITCM is
 * zero-wait-state at core speed. Measured 2026-08-01: ITCM was 0.00% used.
 */

/* EKF3 - the dominant math consumer; the reason this file exists. */
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
*libc.a:libm_math_s_atan2.c.o(.text .text.*)

    /* GyroFFT / CMSIS-DSP hot set (2026-08-11). */
*libmodules__cmsis-dsp.a:arm_cfft_f32.c.obj(.text .text.* .rodata .rodata.*)
*libmodules__cmsis-dsp.a:arm_rfft_fast_f32.c.obj(.text .text.* .rodata .rodata.*)
*libmodules__cmsis-dsp.a:arm_rfft_fast_init_f32.c.obj(.text .text.* .rodata .rodata.*)
*libmodules__cmsis-dsp.a:arm_bitreversal2.c.obj(.text .text.* .rodata .rodata.*)
*libmodules__cmsis-dsp.a:arm_cmplx_mag_squared_f32.c.obj(.text .text.*)
*libmodules__cmsis-dsp.a:arm_mult_f32.c.obj(.text .text.*)
*libmodules__cmsis-dsp.a:arm_add_f32.c.obj(.text .text.*)
*libmodules__cmsis-dsp.a:arm_scale_f32.c.obj(.text .text.*)
*libmodules__cmsis-dsp.a:arm_max_f32.c.obj(.text .text.*)
*libmodules__cmsis-dsp.a:arm_mean_f32.c.obj(.text .text.*)
*libmodules__cmsis-dsp.a:arm_const_structs.c.obj(.text .text.* .rodata .rodata.*)
*libmodules__cmsis-dsp.a:arm_common_tables.c.obj(.text .text.* .rodata .rodata.*)

/* AP-side FFT drivers: Zephyr::DSP's step functions and AP_GyroFFT's
 * per-sample window pushes + analysis loop (10.6 KB, called at gyro rate
 * from sample_gyros even between analysis cycles). */
*libArduCopter_libs.a:DSP.cpp*.o(.text .text.*)
*libArduCopter_libs.a:AP_GyroFFT*.o(.text .text.*)

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
