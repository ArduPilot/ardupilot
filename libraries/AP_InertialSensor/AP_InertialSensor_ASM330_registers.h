
#pragma once

// WHOAMI values
#define WHO_AM_I    0x6B

// ODR
#define ODR_POWER_DOWN  (0x0 << 4)
#define ODR_1600mHz     (0xB << 4)
#define ODR_12500mHz    (0x1 << 4)
#define ODR_26Hz        (0x2 << 4)
#define ODR_52mHz       (0x3 << 4)
#define ODR_104Hz       (0x4 << 4)
#define ODR_208Hz       (0x5 << 4)
#define ODR_416Hz       (0x6 << 4)
#define ODR_833Hz       (0x7 << 4)
#define ODR_1666Hz      (0x8 << 4)
#define ODR_3333Hz      (0x9 << 4)
#define ODR_6667Hz      (0xA << 4)

/*
 *  Accelerometer and Gyroscope registers
*/
#define ASM330_REG_01_RESERVED                              0x01
#define ASM330_REG_PIN_CTRL                                 0x02
#define ASM330_REG_03_RESERVED                              0x03
#define ASM330_REG_04_RESERVED                              0x04
#define ASM330_REG_05_RESERVED                              0x05
#define ASM330_REG_06_RESERVED                              0x06
#define ASM330_REG_FIFO_CTRL1                               0x07
#define ASM330_REG_FIFO_CTRL2                               0x08
#define ASM330_REG_FIFO_CTRL3                               0x09
// BDR_GY
#   define ASM330_REG_FIFO_CTRL3_BDR_GY_NOT_BATCH           (0x0 << 4)
#   define ASM330_REG_FIFO_CTRL3_BDR_GY_6500mHz             (0xB << 4)
#   define ASM330_REG_FIFO_CTRL3_BDR_GY_12500mHz            (0x1 << 4)
#   define ASM330_REG_FIFO_CTRL3_BDR_GY_26Hz                (0x2 << 4)
#   define ASM330_REG_FIFO_CTRL3_BDR_GY_52Hz                (0x3 << 4)
#   define ASM330_REG_FIFO_CTRL3_BDR_GY_104Hz               (0x4 << 4)
#   define ASM330_REG_FIFO_CTRL3_BDR_GY_208Hz               (0x5 << 4)
#   define ASM330_REG_FIFO_CTRL3_BDR_GY_417Hz               (0x6 << 4)
#   define ASM330_REG_FIFO_CTRL3_BDR_GY_833Hz               (0x7 << 4)
#   define ASM330_REG_FIFO_CTRL3_BDR_GY_1667Hz              (0x8 << 4)
#   define ASM330_REG_FIFO_CTRL3_BDR_GY_3333Hz              (0x9 << 4)
#   define ASM330_REG_FIFO_CTRL3_BDR_GY_6667Hz              (0xA << 4)
// BDR_XL
#   define ASM330_REG_FIFO_CTRL3_BDR_XL_NOT_BATCH           (0x0)
#   define ASM330_REG_FIFO_CTRL3_BDR_XL_12500mHz            (0x1)
#   define ASM330_REG_FIFO_CTRL3_BDR_XL_26Hz                (0x2)
#   define ASM330_REG_FIFO_CTRL3_BDR_XL_52Hz                (0x3)
#   define ASM330_REG_FIFO_CTRL3_BDR_XL_104Hz               (0x4)
#   define ASM330_REG_FIFO_CTRL3_BDR_XL_208Hz               (0x5)
#   define ASM330_REG_FIFO_CTRL3_BDR_XL_417Hz               (0x6)
#   define ASM330_REG_FIFO_CTRL3_BDR_XL_833Hz               (0x7)
#   define ASM330_REG_FIFO_CTRL3_BDR_XL_1667Hz              (0x8)
#   define ASM330_REG_FIFO_CTRL3_BDR_XL_3333Hz              (0x9)
#   define ASM330_REG_FIFO_CTRL3_BDR_XL_6667Hz              (0xA)

#define ASM330_REG_FIFO_CTRL4                               0x0A
// DEC_TS_BATCH
#   define ASM330_REG_FIFO_CTRL4_DEC_TS_BATCH_NOT_BATCH     (0x0 << 4)
#   define ASM330_REG_FIFO_CTRL4_DEC_TS_BATCH_DEC_1         (0x1 << 4)
#   define ASM330_REG_FIFO_CTRL4_DEC_TS_BATCH_DEC_8         (0x2 << 4)
#   define ASM330_REG_FIFO_CTRL4_DEC_TS_BATCH_DEC_32        (0x3 << 4)
// ODR_T_BATCH
#   define ASM330_REG_FIFO_CTRL4_ODR_T_BATCH_NOT_BATCH      (0x0 << 4)
#   define ASM330_REG_FIFO_CTRL4_ODR_T_BATCH_1600mHz        (0x1 << 4)
#   define ASM330_REG_FIFO_CTRL4_ODR_T_BATCH_12500mHz       (0x2 << 4)
#   define ASM330_REG_FIFO_CTRL4_ODR_T_BATCH_52Hz           (0x3 << 4)
// FIFO_MODE
#   define ASM330_REG_FIFO_CTRL4_FIFO_MODE_BYPASS           (0x0)
#   define ASM330_REG_FIFO_CTRL4_FIFO_MODE_FIFO             (0x1)
#   define ASM330_REG_FIFO_CTRL4_FIFO_MODE_CONT_TO_FIFO     (0x3)
#   define ASM330_REG_FIFO_CTRL4_FIFO_MODE_BYPASS_TO_CONT   (0x4)
#   define ASM330_REG_FIFO_CTRL4_FIFO_MODE_CONT             (0x6)
#   define ASM330_REG_FIFO_CTRL4_FIFO_MODE_BYPASS_TO_FIFO   (0x7)

#define ASM330_REG_COUNTER_BDR_REG1                         0x0B
#define ASM330_REG_COUNTER_BDR_REG2                         0x0C
#define ASM330_REG_INT1_CTRL                                0x0D
#define ASM330_REG_INT2_CTRL                                0x0E
#define ASM330_REG_WHO_AM_I                                 0x0F
#define ASM330_REG_CTRL1_XL                                 0x10
// ODR_XL
#   define ASM330_REG_CTRL1_XL_ODR_XL_POWER_DOWN            ODR_POWER_DOWN
#   define ASM330_REG_CTRL1_XL_ODR_XL_12500mHz              ODR_12500mHz
#   define ASM330_REG_CTRL1_XL_ODR_XL_26Hz                  ODR_26Hz
#   define ASM330_REG_CTRL1_XL_ODR_XL_52mHz                 ODR_52mHz
#   define ASM330_REG_CTRL1_XL_ODR_XL_104Hz                 ODR_104Hz
#   define ASM330_REG_CTRL1_XL_ODR_XL_208Hz                 ODR_208Hz
#   define ASM330_REG_CTRL1_XL_ODR_XL_416Hz                 ODR_416Hz
#   define ASM330_REG_CTRL1_XL_ODR_XL_833Hz                 ODR_833Hz
#   define ASM330_REG_CTRL1_XL_ODR_XL_1667Hz                ODR_1667Hz
#   define ASM330_REG_CTRL1_XL_ODR_XL_3333Hz                ODR_3333Hz
#   define ASM330_REG_CTRL1_XL_ODR_XL_6667Hz                ODR_6667Hz
// FS1_XL/FS2_XL
#   define ASM330_REG_CTRL1_XL_FS_XL_2G                     (0x0 << 2)
#   define ASM330_REG_CTRL1_XL_FS_XL_4G                     (0x2 << 2)
#   define ASM330_REG_CTRL1_XL_FS_XL_8G                     (0x3 << 2)
#   define ASM330_REG_CTRL1_XL_FS_XL_16G                    (0x1 << 2)
// LPF2_XL_EN
#   define ASM330_REG_CTRL1_XL_LPF2_XL_EN_DISABLE           (0x0 << 1)
#   define ASM330_REG_CTRL1_XL_LPF2_XL_EN_ENABLE            (0x1 << 1)

#define ASM330_REG_CTRL2_G                                  0x11
// ODR_G
#   define ASM330_REG_CTRL2_G_ODR_G_POWER_DOWN              ODR_POWER_DOWN
#   define ASM330_REG_CTRL2_G_ODR_G_12500mHz                ODR_12500mHz
#   define ASM330_REG_CTRL2_G_ODR_G_26Hz                    ODR_26Hz
#   define ASM330_REG_CTRL2_G_ODR_G_52mHz                   ODR_52mHz
#   define ASM330_REG_CTRL2_G_ODR_G_104Hz                   ODR_104Hz
#   define ASM330_REG_CTRL2_G_ODR_G_208Hz                   ODR_208Hz
#   define ASM330_REG_CTRL2_G_ODR_G_416Hz                   ODR_416Hz
#   define ASM330_REG_CTRL2_G_ODR_G_833Hz                   ODR_833Hz
#   define ASM330_REG_CTRL2_G_ODR_G_1667Hz                  ODR_1667Hz
#   define ASM330_REG_CTRL2_G_ODR_G_3333Hz                  ODR_3333Hz
#   define ASM330_REG_CTRL2_G_ODR_G_6667Hz                  ODR_6667Hz
// FS1_G/FS2_G & FS_125
#   define ASM330_REG_CTRL2_G_FS_G_125PS                    (0x1)   // 0010b
#   define ASM330_REG_CTRL2_G_FS_G_250DPS                   (0x0)   // 0000b
#   define ASM330_REG_CTRL2_G_FS_G_500DPS                   (0x4)   // 0100b
#   define ASM330_REG_CTRL2_G_FS_G_1000DPS                  (0x8)   // 1000b
#   define ASM330_REG_CTRL2_G_FS_G_2000DPS                  (0xC)   // 1100b
#   define ASM330_REG_CTRL2_G_FS_G_4000DPS                  (0x1)   // 0001b

#define ASM330_REG_CTRL3_C                                  0x12
// BOOT
#   define ASM330_REG_CTRL3_C_BOOT_NORMAL                   (0x0 << 7)
#   define ASM330_REG_CTRL3_C_BOOT_REBOOT_MEM_CONT          (0x1 << 7)
// BDU
#   define ASM330_REG_CTRL3_C_BDU_DISABLE                   (0x0 << 6)
#   define ASM330_REG_CTRL3_C_BDU_ENABLE                    (0x1 << 6)
// H_LACTIVE
#   define ASM330_REG_CTRL3_C_H_LACTIVE_ACTIVE_HIGH         (0x0 << 5)
#   define ASM330_REG_CTRL3_C_H_LACTIVE_ACTIVE_LOW          (0x1 << 5)
// PP_OD
#   define ASM330_REG_CTRL3_C_PP_OD_PP                      (0x0 << 4)
#   define ASM330_REG_CTRL3_C_PP_OD_OD                      (0x1 << 4)
// SIM
#   define ASM330_REG_CTRL3_C_SIM_4_WIRE                    (0x0 << 3)
#   define ASM330_REG_CTRL3_C_SIM_3_WIRE                    (0x1 << 3)
// IF_INC
#   define ASM330_REG_CTRL3_C_IF_INC_DISABLE                (0x0 << 2)
#   define ASM330_REG_CTRL3_C_IF_INC_ENABLE                 (0x1 << 2)
// SW_RESET
#   define ASM330_REG_CTRL3_C_SW_RESET_NORMAL               (0x0)
#   define ASM330_REG_CTRL3_C_SW_RESET_RESET                (0x1)

#define ASM330_REG_CTRL4_C                                  0x13
#define ASM330_REG_CTRL5_C                                  0x14
#define ASM330_REG_CTRL6_C                                  0x15
#define ASM330_REG_CTRL7_G                                  0x16
// HP_EN_G
#   define ASM330_REG_CTRL7_G_HP_EN_G_DISABLE               (0x0 << 6)
#   define ASM330_REG_CTRL7_G_HP_EN_G_ENABLE                (0x1 << 6)
// HPM1_G/HPM2_G
#   define ASM330_REG_CTRL7_G_HPM_G_16mHz                   (0x0 << 4)
#   define ASM330_REG_CTRL7_G_HPM_G_65mHz                   (0x1 << 4)
#   define ASM330_REG_CTRL7_G_HPM_G_260mHz                  (0x2 << 4)
#   define ASM330_REG_CTRL7_G_HPM_G_1040mHz                 (0x3 << 4)
// USR_OFF_ON_OUT
#   define ASM330_REG_CTRL7_G_USR_OFF_ON_OUT_BYPASS         (0x0 << 1)
#   define ASM330_REG_CTRL7_G_USR_OFF_ON_OUT_ENABLE         (0x1 << 1)

#define ASM330_REG_CTRL8_XL                                 0x17
// HPCF_XL
#   define ASM330_REG_CTRL8_XL_HPCF_XL_ODR_PER_4            (0x0 << 5)
#   define ASM330_REG_CTRL8_XL_HPCF_XL_ODR_PER_10           (0x1 << 5)
#   define ASM330_REG_CTRL8_XL_HPCF_XL_ODR_PER_20           (0x2 << 5)
#   define ASM330_REG_CTRL8_XL_HPCF_XL_ODR_PER_45           (0x3 << 5)
#   define ASM330_REG_CTRL8_XL_HPCF_XL_ODR_PER_100          (0x4 << 5)
#   define ASM330_REG_CTRL8_XL_HPCF_XL_ODR_PER_200          (0x5 << 5)
#   define ASM330_REG_CTRL8_XL_HPCF_XL_ODR_PER_400          (0x6 << 5)
#   define ASM330_REG_CTRL8_XL_HPCF_XL_ODR_PER_800          (0x7 << 5)
// HP_REF_MODE_XL
#   define ASM330_REG_CTRL8_XL_HP_REF_MODE_XL_DISABLE       (0x0 << 4)
#   define ASM330_REG_CTRL8_XL_HP_REF_MODE_XL_ENABLE        (0x1 << 4)
// FASTSETTL_MODE_XL
#   define ASM330_REG_CTRL8_XL_FASTSETTL_MODE_XL_DISABLE    (0x0 << 3)
#   define ASM330_REG_CTRL8_XL_FASTSETTL_MODE_XL_ENABLE     (0x1 << 3)
// HP_SLOPE_XL_EN
#   define ASM330_REG_CTRL8_XL_HP_SLOPE_XL_EN_DISABLE       (0x0 << 2)
#   define ASM330_REG_CTRL8_XL_HP_SLOPE_XL_EN_ENABLE        (0x1 << 2)
// LOW_PASS_ON_6D
#   define ASM330_REG_CTRL8_XL_LOW_PASS_ON_6D_LPF1          (0x0)
#   define ASM330_REG_CTRL8_XL_LOW_PASS_ON_6D_LPF2          (0x1)

#define ASM330_REG_CTRL9_XL                                 0x18
// DEN_X
#   define ASM330_REG_CTRL9_XL_DEN_X_DISABLE                (0x0 << 7)
#   define ASM330_REG_CTRL9_XL_DEN_X_ENABLE                 (0x1 << 7)
// DEN_Y
#   define ASM330_REG_CTRL9_XL_DEN_Y_DISABLE                (0x0 << 6)
#   define ASM330_REG_CTRL9_XL_DEN_Y_ENABLE                 (0x1 << 6)
// DEN_Z
#   define ASM330_REG_CTRL9_XL_DEN_Z_DISABLE                (0x0 << 5)
#   define ASM330_REG_CTRL9_XL_DEN_Z_ENABLE                 (0x1 << 5)
// DEN_XL_G
#   define ASM330_REG_CTRL9_XL_DEN_XL_G_GYRO                (0x0 << 4)
#   define ASM330_REG_CTRL9_XL_DEN_XL_G_ACCEL               (0x1 << 4)
// DEN_XL_EN
#   define ASM330_REG_CTRL9_XL_DEN_XL_EN_EXT_DISABLE        (0x0 << 3)
#   define ASM330_REG_CTRL9_XL_DEN_XL_EN_EXT_ENABLE         (0x1 << 3)
// DEN_LH
#   define ASM330_REG_CTRL9_XL_DEN_LH_ACTIVE_LOW            (0x0 << 2)
#   define ASM330_REG_CTRL9_XL_DEN_LH_ACTIVE_HIGH           (0x1 << 2)
#define ASM330_REG_CTRL10_C                                 0x19
#define ASM330_REG_ALL_INT_SRC                              0x1A
#define ASM330_REG_WAKE_UP_SRC                              0x1B
#define ASM330_REG_1C_RESERVED                              0x1C
#define ASM330_REG_D6D_SRC                                  0x1D
#define ASM330_REG_STATUS_REG                               0x1E
#define ASM330_REG_STATUS_SPIAUX                            0x1E
#define ASM330_REG_1F_RESERVED                              0x1F
#define ASM330_REG_OUT_TEMP_L                               0x20
#define ASM330_REG_OUT_TEMP_H                               0x21
#define ASM330_REG_OUTX_L_G                                 0x22
#define ASM330_REG_OUTX_H_G                                 0x23
#define ASM330_REG_OUTY_L_G                                 0x24
#define ASM330_REG_OUTY_H_G                                 0x25
#define ASM330_REG_OUTZ_L_G                                 0x26
#define ASM330_REG_OUTZ_H_G                                 0x27
#define ASM330_REG_OUTX_L_A                                 0x28
#define ASM330_REG_OUTX_H_A                                 0x29
#define ASM330_REG_OUTY_L_A                                 0x2A
#define ASM330_REG_OUTY_H_A                                 0x2B
#define ASM330_REG_OUTZ_L_A                                 0x2C
#define ASM330_REG_OUTZ_H_A                                 0x2D
#define ASM330_REG_2E_RESERVED                              0x2E
#define ASM330_REG_2F_RESERVED                              0x2F
#define ASM330_REG_30_RESERVED                              0x30
#define ASM330_REG_31_RESERVED                              0x31
#define ASM330_REG_32_RESERVED                              0x32
#define ASM330_REG_33_RESERVED                              0x33
#define ASM330_REG_34_RESERVED                              0x34
#define ASM330_REG_35_RESERVED                              0x35
#define ASM330_REG_36_RESERVED                              0x36
#define ASM330_REG_37_RESERVED                              0x37
#define ASM330_REG_38_RESERVED                              0x38
#define ASM330_REG_39_RESERVED                              0x39
#define ASM330_REG_FIFO_STATUS1                             0x3A
#define ASM330_REG_FIFO_STATUS2                             0x3B
#define ASM330_REG_3C_RESERVED                              0x3C
#define ASM330_REG_3D_RESERVED                              0x3D
#define ASM330_REG_3E_RESERVED                              0x3E
#define ASM330_REG_3F_RESERVED                              0x3F
#define ASM330_REG_TIMESTAMP0                               0x40
#define ASM330_REG_TIMESTAMP1                               0x41
#define ASM330_REG_TIMESTAMP2                               0x42
#define ASM330_REG_TIMESTAMP3                               0x43
#define ASM330_REG_44_RESERVED                              0x44
#define ASM330_REG_45_RESERVED                              0x45
#define ASM330_REG_46_RESERVED                              0x46
#define ASM330_REG_47_RESERVED                              0x47
#define ASM330_REG_48_RESERVED                              0x48
#define ASM330_REG_49_RESERVED                              0x49
#define ASM330_REG_4A_RESERVED                              0x4A
#define ASM330_REG_4B_RESERVED                              0x4B
#define ASM330_REG_4C_RESERVED                              0x4C
#define ASM330_REG_4D_RESERVED                              0x4D
#define ASM330_REG_4E_RESERVED                              0x4E
#define ASM330_REG_4F_RESERVED                              0x4F
#define ASM330_REG_50_RESERVED                              0x50
#define ASM330_REG_51_RESERVED                              0x51
#define ASM330_REG_52_RESERVED                              0x52
#define ASM330_REG_53_RESERVED                              0x53
#define ASM330_REG_54_RESERVED                              0x54
#define ASM330_REG_55_RESERVED                              0x55
#define ASM330_REG_INT_CFG0                                 0x56
#define ASM330_REG_57_RESERVED                              0x57
#define ASM330_REG_INT_CFG2                                 0x58
#define ASM330_REG_THS_6D                                   0x59
#define ASM330_REG_5A_RESERVED                              0x5A
#define ASM330_REG_WAKE_UP_THS                              0x5B
#define ASM330_REG_WAKE_UP_DUR                              0x5C
#define ASM330_REG_FREE_FALL                                0x5D
#define ASM330_REG_MD1_CFG                                  0x5E
#define ASM330_REG_MD2_CFG                                  0x5F
#define ASM330_REG_60_RESERVED                              0x60
#define ASM330_REG_61_RESERVED                              0x61
#define ASM330_REG_62_RESERVED                              0x62
#define ASM330_REG_INTERNAL_FREQ_FINE                       0x63
#define ASM330_REG_64_RESERVED                              0x64
#define ASM330_REG_65_RESERVED                              0x65
#define ASM330_REG_66_RESERVED                              0x66
#define ASM330_REG_67_RESERVED                              0x67
#define ASM330_REG_68_RESERVED                              0x68
#define ASM330_REG_69_RESERVED                              0x69
#define ASM330_REG_6A_RESERVED                              0x6A
#define ASM330_REG_6B_RESERVED                              0x6B
#define ASM330_REG_6C_RESERVED                              0x6C
#define ASM330_REG_6D_RESERVED                              0x6D
#define ASM330_REG_6E_RESERVED                              0x6E
#define ASM330_REG_6F_RESERVED                              0x6F
#define ASM330_REG_70_RESERVED                              0x70
#define ASM330_REG_71_RESERVED                              0x71
#define ASM330_REG_72_RESERVED                              0x72
#define ASM330_REG_X_OFS_USR                                0x73
#define ASM330_REG_Y_OFS_USR                                0x74
#define ASM330_REG_Z_OFS_USR                                0x75
#define ASM330_REG_76_RESERVED                              0x76
#define ASM330_REG_77_RESERVED                              0x77
#define ASM330_REG_FIFO_DATA_OUT_TAG                        0x78
#define ASM330_REG_FIFO_DATA_OUT_X_L                        0x79
#define ASM330_REG_FIFO_DATA_OUT_X_H                        0x7A
#define ASM330_REG_FIFO_DATA_OUT_Y_L                        0x7B
#define ASM330_REG_FIFO_DATA_OUT_Y_H                        0x7C
#define ASM330_REG_FIFO_DATA_OUT_Z_L                        0x7D
#define ASM330_REG_FIFO_DATA_OUT_Z_H                        0x7E

