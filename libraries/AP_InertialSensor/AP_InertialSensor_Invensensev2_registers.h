
#pragma once

#define REG_BANK0 0x00U
#define REG_BANK1 0x01U
#define REG_BANK2 0x02U
#define REG_BANK3 0x03U


#define INV2REG(b, r)      ((((uint16_t)b) << 8)|(r))
#define GET_BANK(r)         ((r) >> 8)
#define GET_REG(r)          ((r) & 0xFFU)

#define BIT_READ_FLAG                           0x80
#define BIT_I2C_SLVX_EN                         0x80

//Register Map
#define INV2REG_WHO_AM_I               INV2REG(REG_BANK0,0x00U)
#define INV2REG_USER_CTRL              INV2REG(REG_BANK0,0x03U)
#       define BIT_USER_CTRL_I2C_MST_RESET          0x02 // reset I2C Master (only applicable if I2C_MST_EN bit is set)
#       define BIT_USER_CTRL_SRAM_RESET             0x04 // Reset (i.e. clear) FIFO buffer
#       define BIT_USER_CTRL_DMP_RESET              0x08 // Reset DMP
#       define BIT_USER_CTRL_I2C_IF_DIS             0x10 // Disable primary I2C interface and enable hal.spi->interface
#       define BIT_USER_CTRL_I2C_MST_EN             0x20 // Enable MPU to act as the I2C Master to external slave sensors
#       define BIT_USER_CTRL_FIFO_EN                0x40 // Enable FIFO operations
#       define BIT_USER_CTRL_DMP_EN                 0x80     // Enable DMP operations
#define INV2REG_LP_CONFIG              INV2REG(REG_BANK0,0x05U)
#define INV2REG_PWR_MGMT_1             INV2REG(REG_BANK0,0x06U)
#       define BIT_PWR_MGMT_1_CLK_INTERNAL          0x00 // clock set to internal 8Mhz oscillator
#       define BIT_PWR_MGMT_1_CLK_AUTO              0x01 // PLL with X axis gyroscope reference
#       define BIT_PWR_MGMT_1_CLK_STOP              0x07 // Stops the clock and keeps the timing generator in reset
#       define BIT_PWR_MGMT_1_TEMP_DIS              0x08 // disable temperature sensor
#       define BIT_PWR_MGMT_1_SLEEP                 0x40 // put sensor into low power sleep mode
#       define BIT_PWR_MGMT_1_DEVICE_RESET          0x80 // reset entire device
#define INV2REG_PWR_MGMT_2             INV2REG(REG_BANK0,0x07U)
#define INV2REG_INT_PIN_CFG            INV2REG(REG_BANK0,0x0FU)
#       define BIT_BYPASS_EN                        0x02
#       define BIT_INT_RD_CLEAR                     0x10    // clear the interrupt when any read occurs
#       define BIT_LATCH_INT_EN                     0x20    // latch data ready pin
#define INV2REG_INT_ENABLE             INV2REG(REG_BANK0,0x10U)
#       define BIT_PLL_RDY_EN                       0x04
#define INV2REG_INT_ENABLE_1           INV2REG(REG_BANK0,0x11U)
#define INV2REG_INT_ENABLE_2           INV2REG(REG_BANK0,0x12U)
#define INV2REG_INT_ENABLE_3           INV2REG(REG_BANK0,0x13U)
#define INV2REG_I2C_MST_STATUS         INV2REG(REG_BANK0,0x17U)
#define INV2REG_INT_STATUS             INV2REG(REG_BANK0,0x19U)

#define INV2REG_INT_STATUS_1           INV2REG(REG_BANK0,0x1AU)
#define INV2REG_INT_STATUS_2           INV2REG(REG_BANK0,0x1BU)
#define INV2REG_INT_STATUS_3           INV2REG(REG_BANK0,0x1CU)
#define INV2REG_DELAY_TIMEH            INV2REG(REG_BANK0,0x28U)
#define INV2REG_DELAY_TIMEL            INV2REG(REG_BANK0,0x29U)
#define INV2REG_ACCEL_XOUT_H           INV2REG(REG_BANK0,0x2DU)
#define INV2REG_ACCEL_XOUT_L           INV2REG(REG_BANK0,0x2EU)
#define INV2REG_ACCEL_YOUT_H           INV2REG(REG_BANK0,0x2FU)
#define INV2REG_ACCEL_YOUT_L           INV2REG(REG_BANK0,0x30U)
#define INV2REG_ACCEL_ZOUT_H           INV2REG(REG_BANK0,0x31U)
#define INV2REG_ACCEL_ZOUT_L           INV2REG(REG_BANK0,0x32U)
#define INV2REG_GYRO_XOUT_H            INV2REG(REG_BANK0,0x33U)
#define INV2REG_GYRO_XOUT_L            INV2REG(REG_BANK0,0x34U)
#define INV2REG_GYRO_YOUT_H            INV2REG(REG_BANK0,0x35U)
#define INV2REG_GYRO_YOUT_L            INV2REG(REG_BANK0,0x36U)
#define INV2REG_GYRO_ZOUT_H            INV2REG(REG_BANK0,0x37U)
#define INV2REG_GYRO_ZOUT_L            INV2REG(REG_BANK0,0x38U)
#define INV2REG_TEMP_OUT_H             INV2REG(REG_BANK0,0x39U)
#define INV2REG_TEMP_OUT_L             INV2REG(REG_BANK0,0x3AU)
#define INV2REG_EXT_SLV_SENS_DATA_00   INV2REG(REG_BANK0,0x3BU)
#define INV2REG_EXT_SLV_SENS_DATA_01   INV2REG(REG_BANK0,0x3CU)
#define INV2REG_EXT_SLV_SENS_DATA_02   INV2REG(REG_BANK0,0x3DU)
#define INV2REG_EXT_SLV_SENS_DATA_03   INV2REG(REG_BANK0,0x3EU)
#define INV2REG_EXT_SLV_SENS_DATA_04   INV2REG(REG_BANK0,0x3FU)
#define INV2REG_EXT_SLV_SENS_DATA_05   INV2REG(REG_BANK0,0x40U)
#define INV2REG_EXT_SLV_SENS_DATA_06   INV2REG(REG_BANK0,0x41U)
#define INV2REG_EXT_SLV_SENS_DATA_07   INV2REG(REG_BANK0,0x42U)
#define INV2REG_EXT_SLV_SENS_DATA_08   INV2REG(REG_BANK0,0x43U)
#define INV2REG_EXT_SLV_SENS_DATA_09   INV2REG(REG_BANK0,0x44U)
#define INV2REG_EXT_SLV_SENS_DATA_10   INV2REG(REG_BANK0,0x45U)
#define INV2REG_EXT_SLV_SENS_DATA_11   INV2REG(REG_BANK0,0x46U)
#define INV2REG_EXT_SLV_SENS_DATA_12   INV2REG(REG_BANK0,0x47U)
#define INV2REG_EXT_SLV_SENS_DATA_13   INV2REG(REG_BANK0,0x48U)
#define INV2REG_EXT_SLV_SENS_DATA_14   INV2REG(REG_BANK0,0x49U)
#define INV2REG_EXT_SLV_SENS_DATA_15   INV2REG(REG_BANK0,0x4AU)
#define INV2REG_EXT_SLV_SENS_DATA_16   INV2REG(REG_BANK0,0x4BU)
#define INV2REG_EXT_SLV_SENS_DATA_17   INV2REG(REG_BANK0,0x4CU)
#define INV2REG_EXT_SLV_SENS_DATA_18   INV2REG(REG_BANK0,0x4DU)
#define INV2REG_EXT_SLV_SENS_DATA_19   INV2REG(REG_BANK0,0x4EU)
#define INV2REG_EXT_SLV_SENS_DATA_20   INV2REG(REG_BANK0,0x4FU)
#define INV2REG_EXT_SLV_SENS_DATA_21   INV2REG(REG_BANK0,0x50U)
#define INV2REG_EXT_SLV_SENS_DATA_22   INV2REG(REG_BANK0,0x51U)
#define INV2REG_EXT_SLV_SENS_DATA_23   INV2REG(REG_BANK0,0x52U)
#define INV2REG_FIFO_EN_1              INV2REG(REG_BANK0,0x66U)
#       define BIT_SLV3_FIFO_EN                     0x08
#       define BIT_SLV2_FIFO_EN                     0x04
#       define BIT_SLV1_FIFO_EN                     0x02
#       define BIT_SLV0_FIFI_EN0                    0x01
#define INV2REG_FIFO_EN_2              INV2REG(REG_BANK0,0x67U)
#       define BIT_ACCEL_FIFO_EN                    0x10
#       define BIT_ZG_FIFO_EN                       0x08
#       define BIT_YG_FIFO_EN                       0x04
#       define BIT_XG_FIFO_EN                       0x02
#       define BIT_TEMP_FIFO_EN                     0x01
#define INV2REG_FIFO_RST               INV2REG(REG_BANK0,0x68U)
#define INV2REG_FIFO_MODE              INV2REG(REG_BANK0,0x69U)
#define INV2REG_FIFO_COUNTH            INV2REG(REG_BANK0,0x70U)
#define INV2REG_FIFO_COUNTL            INV2REG(REG_BANK0,0x71U)
#define INV2REG_FIFO_R_W               INV2REG(REG_BANK0,0x72U)
#define INV2REG_DATA_RDY_STATUS        INV2REG(REG_BANK0,0x74U)
#define INV2REG_FIFO_CFG               INV2REG(REG_BANK0,0x76U)

#define INV2REG_SELF_TEST_X_GYRO       INV2REG(REG_BANK1,0x02U)
#define INV2REG_SELF_TEST_Y_GYRO       INV2REG(REG_BANK1,0x03U)
#define INV2REG_SELF_TEST_Z_GYRO       INV2REG(REG_BANK1,0x04U)
#define INV2REG_SELF_TEST_X_ACCEL      INV2REG(REG_BANK1,0x0EU)
#define INV2REG_SELF_TEST_Y_ACCEL      INV2REG(REG_BANK1,0x0FU)
#define INV2REG_SELF_TEST_Z_ACCEL      INV2REG(REG_BANK1,0x10U)
#define INV2REG_XA_OFFS_H              INV2REG(REG_BANK1,0x14U)
#define INV2REG_XA_OFFS_L              INV2REG(REG_BANK1,0x15U)
#define INV2REG_YA_OFFS_H              INV2REG(REG_BANK1,0x17U)
#define INV2REG_YA_OFFS_L              INV2REG(REG_BANK1,0x18U)
#define INV2REG_ZA_OFFS_H              INV2REG(REG_BANK1,0x1AU)
#define INV2REG_ZA_OFFS_L              INV2REG(REG_BANK1,0x1BU)
#define INV2REG_TIMEBASE_CORRECTIO     INV2REG(REG_BANK1,0x28U)

#define INV2REG_GYRO_SMPLRT_DIV        INV2REG(REG_BANK2,0x00U)
#define INV2REG_GYRO_CONFIG_1          INV2REG(REG_BANK2,0x01U)
#       define BIT_GYRO_NODLPF_9KHZ                 0x00
#       define BIT_GYRO_DLPF_ENABLE                 0x01
#       define GYRO_DLPF_CFG_229HZ                  0x00
#       define GYRO_DLPF_CFG_188HZ                  0x01
#       define GYRO_DLPF_CFG_154HZ                  0x02
#       define GYRO_DLPF_CFG_73HZ                   0x03
#       define GYRO_DLPF_CFG_35HZ                   0x04
#       define GYRO_DLPF_CFG_17HZ                   0x05
#       define GYRO_DLPF_CFG_9HZ                    0x06
#       define GYRO_DLPF_CFG_376HZ                  0x07
#       define GYRO_DLPF_CFG_SHIFT                  0x03
#       define BITS_GYRO_FS_250DPS                  0x00
#       define BITS_GYRO_FS_500DPS                  0x02
#       define BITS_GYRO_FS_1000DPS                 0x04
#       define BITS_GYRO_FS_2000DPS                 0x06
#       define BITS_GYRO_FS_MASK                    0x06 // only bits 1 and 2 are used for gyro full scale so use this to mask off other bits
#define INV2REG_GYRO_CONFIG_2          INV2REG(REG_BANK2,0x02U)
#define INV2REG_XG_OFFS_USRH           INV2REG(REG_BANK2,0x03U)
#define INV2REG_XG_OFFS_USRL           INV2REG(REG_BANK2,0x04U)
#define INV2REG_YG_OFFS_USRH           INV2REG(REG_BANK2,0x05U)
#define INV2REG_YG_OFFS_USRL           INV2REG(REG_BANK2,0x06U)
#define INV2REG_ZG_OFFS_USRH           INV2REG(REG_BANK2,0x07U)
#define INV2REG_ZG_OFFS_USRL           INV2REG(REG_BANK2,0x08U)
#define INV2REG_ODR_ALIGN_EN           INV2REG(REG_BANK2,0x09U)
#define INV2REG_ACCEL_SMPLRT_DIV_1     INV2REG(REG_BANK2,0x10U)
#define INV2REG_ACCEL_SMPLRT_DIV_2     INV2REG(REG_BANK2,0x11U)
#define INV2REG_ACCEL_INTEL_CTRL       INV2REG(REG_BANK2,0x12U)
#define INV2REG_ACCEL_WOM_THR          INV2REG(REG_BANK2,0x13U)
#define INV2REG_ACCEL_CONFIG           INV2REG(REG_BANK2,0x14U)
#       define BIT_ACCEL_NODLPF_4_5KHZ               0x00
#       define BIT_ACCEL_DLPF_ENABLE                 0x01
#       define ACCEL_DLPF_CFG_265HZ                  0x00
#       define ACCEL_DLPF_CFG_136HZ                  0x02
#       define ACCEL_DLPF_CFG_68HZ                   0x03
#       define ACCEL_DLPF_CFG_34HZ                   0x04
#       define ACCEL_DLPF_CFG_17HZ                   0x05
#       define ACCEL_DLPF_CFG_8HZ                    0x06
#       define ACCEL_DLPF_CFG_499HZ                  0x07
#       define ACCEL_DLPF_CFG_SHIFT                  0x03
#       define BITS_ACCEL_FS_2G                      0x00
#       define BITS_ACCEL_FS_4G                      0x02
#       define BITS_ACCEL_FS_8G                      0x04
#       define BITS_ACCEL_FS_16G                     0x06
#       define BITS_ACCEL_FS_MASK                    0x06 // only bits 1 and 2 are used for gyro full scale so use this to mask off other bits
#define INV2REG_FSYNC_CONFIG           INV2REG(REG_BANK2,0x52U)
#       define FSYNC_CONFIG_EXT_SYNC_TEMP          0x01
#       define FSYNC_CONFIG_EXT_SYNC_GX            0x02
#       define FSYNC_CONFIG_EXT_SYNC_GY            0x03
#       define FSYNC_CONFIG_EXT_SYNC_GZ            0x04
#       define FSYNC_CONFIG_EXT_SYNC_AX            0x05
#       define FSYNC_CONFIG_EXT_SYNC_AY            0x06
#       define FSYNC_CONFIG_EXT_SYNC_AZ            0x07
#define INV2REG_TEMP_CONFIG            INV2REG(REG_BANK2,0x53U)
#define INV2REG_MOD_CTRL_USR           INV2REG(REG_BANK2,0x54U)

#define INV2REG_I2C_MST_ODR_CONFIG     INV2REG(REG_BANK3,0x00U)
#define INV2REG_I2C_MST_CTRL           INV2REG(REG_BANK3,0x01U)
#       define BIT_I2C_MST_P_NSR                    0x10
#       define BIT_I2C_MST_CLK_400KHZ               0x0D
#define INV2REG_I2C_MST_DELAY_CTRL     INV2REG(REG_BANK3,0x02U)
#       define BIT_I2C_SLV0_DLY_EN              0x01
#       define BIT_I2C_SLV1_DLY_EN              0x02
#       define BIT_I2C_SLV2_DLY_EN              0x04
#       define BIT_I2C_SLV3_DLY_EN              0x08
#define INV2REG_I2C_SLV0_ADDR          INV2REG(REG_BANK3,0x03U)
#define INV2REG_I2C_SLV0_REG           INV2REG(REG_BANK3,0x04U)
#define INV2REG_I2C_SLV0_CTRL          INV2REG(REG_BANK3,0x05U)
#define INV2REG_I2C_SLV0_DO            INV2REG(REG_BANK3,0x06U)
#define INV2REG_I2C_SLV1_ADDR          INV2REG(REG_BANK3,0x07U)
#define INV2REG_I2C_SLV1_REG           INV2REG(REG_BANK3,0x08U)
#define INV2REG_I2C_SLV1_CTRL          INV2REG(REG_BANK3,0x09U)
#define INV2REG_I2C_SLV1_DO            INV2REG(REG_BANK3,0x0AU)
#define INV2REG_I2C_SLV2_ADDR          INV2REG(REG_BANK3,0x0BU)
#define INV2REG_I2C_SLV2_REG           INV2REG(REG_BANK3,0x0CU)
#define INV2REG_I2C_SLV2_CTRL          INV2REG(REG_BANK3,0x0DU)
#define INV2REG_I2C_SLV2_DO            INV2REG(REG_BANK3,0x0EU)
#define INV2REG_I2C_SLV3_ADDR          INV2REG(REG_BANK3,0x0FU)
#define INV2REG_I2C_SLV3_REG           INV2REG(REG_BANK3,0x10U)
#define INV2REG_I2C_SLV3_CTRL          INV2REG(REG_BANK3,0x11U)
#define INV2REG_I2C_SLV3_DO            INV2REG(REG_BANK3,0x12U)
#define INV2REG_I2C_SLV4_ADDR          INV2REG(REG_BANK3,0x13U)
#define INV2REG_I2C_SLV4_REG           INV2REG(REG_BANK3,0x14U)
#define INV2REG_I2C_SLV4_CTRL          INV2REG(REG_BANK3,0x15U)
#define INV2REG_I2C_SLV4_DO            INV2REG(REG_BANK3,0x16U)
#define INV2REG_I2C_SLV4_DI            INV2REG(REG_BANK3,0x17U)

#define INV2REG_BANK_SEL               0x7F

// WHOAMI values
#define INV2_WHOAMI_ICM20648		0xe0
#define INV2_WHOAMI_ICM20948		0xea
#define INV2_WHOAMI_ICM20649        0xe1
