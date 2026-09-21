# FMUv3 with bi-directional dshot.
#
# This includes the common FMUv3 configuration and overrides only the
# parts which differ for bi-directional dshot: the motor timers gain
# BIDIR/NODMA, SPI4 (and its devices) is freed up to release the DMA
# streams that dshot needs, and the DMA priorities are adjusted.

include ../fmuv3/fmuv3.inc

# the motor outputs use bi-directional dshot:
undef PE13 PE11 PD13 PD14
PE13 TIM1_CH3 TIM1 PWM(2) GPIO(51) BIDIR
PE11 TIM1_CH2 TIM1 PWM(3) GPIO(52) BIDIR
PD13 TIM4_CH2 TIM4 PWM(5) GPIO(54) NODMA
PD14 TIM4_CH3 TIM4 PWM(6) GPIO(55) NODMA

# free up SPI4 and its devices to release DMA streams for dshot:
undef SPI4
undef ms5611_ext mpu9250_ext icm20948_ext lsm9ds0_ext_g lsm9ds0_ext_am icm20602_ext
undef external0m0 external0m1 external0m2 external0m3 pixartPC15

# DMA priorities for bi-directional dshot:
DMA_PRIORITY TIM1_CH3 TIM1_CH2 TIM1_UP USART6* ADC1* SDIO* SPI*

# The standalone fmuv3-bdshot definition never carried the Solo-specific,
# 1M-flash-check or external IST8310 probing defines that fmuv3 has, so
# remove them to keep behaviour identical now that we inherit fmuv3:

# Solo-specific:
undef AP_NOTIFY_OREOLED_ENABLED
undef HAL_SOLO_GIMBAL_ENABLED
undef AP_BATTERY_SMBUS_SOLO_ENABLED

# 1M-flash check:
undef BOARD_CHECK_F427_USE_1M

# external IST8310 compass probing:
undef AP_COMPASS_IST8310_EXTERNAL_BUS_PROBING_ENABLED
