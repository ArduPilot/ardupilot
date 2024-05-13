# crystal frequency
OSCILLATOR_HZ 24000000

FLASH_SIZE_KB 2048

# with 2M flash we can afford to optimize for speed
env OPTIMIZE -O2

# the H743 has 128k sectors
FLASH_RESERVE_START_KB 128

define HAL_STORAGE_SIZE 32768

# order of I2C buses
I2C_ORDER I2C2 I2C1

# order of UARTs (and USB)
SERIAL_ORDER OTG1 USART2 USART3 UART4 UART8 UART7 USART1

# UART for IOMCU
IOMCU_UART USART6

# If the board has an IOMCU connected via a UART then this defines the
# UART to talk to that MCU. Leave it out for boards with no IOMCU.
PC6 USART6_TX USART6
PC7 USART6_RX USART6

include serial.inc

# these are the pins for SWD debugging with a STlinkv2 or black-magic probe
PA13 JTMS-SWDIO SWD
PA14 JTCK-SWCLK SWD
PB2 BOOT1 INPUT

include spi.inc

PA2 BATT_VOLTAGE_SENS ADC1 SCALE(1)
PA3 BATT_CURRENT_SENS ADC1 SCALE(1)
PC4 AUX_ADC1 ADC1 SCALE(1)
PC5 PRESSURE_SENS ADC1 SCALE(2)

PB5 LED_SAFETY OUTPUT HIGH OPENDRAIN

PD10 FRAM_CS CS SPEED_VERYLOW

PC13 GYRO_CS CS

include sdcard.inc

# now we define the pins that USB is connected on
PA11 OTG_FS_DM OTG1
PA12 OTG_FS_DP OTG1

# this is the pin that senses USB being connected. It is an input pin
# setup as OPENDRAIN
PA9 VBUS INPUT OPENDRAIN

include i2c_gpio.inc

PD7 BARO_CS CS
PE4 MPU_EXT_CS CS

SPIDEV ramtron        SPI2 DEVID10 FRAM_CS      MODE3  8*MHZ  8*MHZ
SPIDEV bmi270_ext     SPI4 DEVID4  MPU_EXT_CS   MODE3  1*MHZ  8*MHZ 
SPIDEV imu_ext        SPI4 DEVID5  MPU_EXT_CS   MODE3  2*MHZ  8*MHZ
SPIDEV bmi270_onboard SPI1 DEVID7  GYRO_CS      MODE3  1*MHZ  8*MHZ 
SPIDEV imu_onboard    SPI1 DEVID8  GYRO_CS      MODE3  2*MHZ  8*MHZ
SPIDEV SPL06_onboard  SPI1 DEVID9  BARO_CS      MODE3  1*MHZ  1*MHZ

IMU BMI270 SPI:bmi270_ext ROTATION_YAW_90
IMU Invensensev3 SPI:imu_ext ROTATION_NONE
BARO SPL06 SPI:SPL06_onboard
IMU BMI270 SPI:bmi270_onboard ROTATION_YAW_90
IMU Invensensev3 SPI:imu_onboard ROTATION_NONE

define HAL_DEFAULT_INS_FAST_SAMPLE 3
define HAL_EKF_IMU_MASK_DEFAULT 3

define HAL_PROBE_EXTERNAL_I2C_COMPASSES

# Nnow some defines for logging and terrain data files.
define HAL_BOARD_LOG_DIRECTORY "/APM/LOGS"
define HAL_BOARD_TERRAIN_DIRECTORY "/APM/TERRAIN"
# Enable FAT filesystem support (needs a microSD defined via SDIO).
define HAL_OS_FATFS_IO 1

# allow to have have a dedicated safety switch pin
define HAL_HAVE_SAFETY_SWITCH 1

# This enables the use of a ramtron device for storage, if one is
# found on SPI. You must have a ramtron entry in the SPI device table.
# Enable RAMTROM parameter storage.
define HAL_WITH_RAMTRON 1

# This is analog air speed adc pin
define HAL_ARSPD_PIN 8

define HAL_HAVE_PIXRACER_LED
define HAL_GPIO_LED_ON 0
define HAL_GPIO_A_LED_PIN 90
define HAL_GPIO_B_LED_PIN 91
define HAL_GPIO_C_LED_PIN 92

# We can't share the IO UART (USART6).
DMA_NOSHARE SPI4* SPI1* USART6*
