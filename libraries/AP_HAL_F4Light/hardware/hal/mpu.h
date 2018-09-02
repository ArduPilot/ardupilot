/*
    MPU defines

    see ST AN4838

(c) 2017 night_ghost@ykoctpa.ru
 
based on: datasheet

*/

#pragma once


#define MPU_TYPE_SEPARATED                  (1U << 0U)
#define MPU_TYPE_DREGION(n)                 (((n) >> 8U) & 255U)
#define MPU_TYPE_IREGION(n)                 (((n) >> 16U) & 255U)

#define MPU_CTRL_ENABLE                     MPU_CTRL_ENABLE_Msk
#define MPU_CTRL_HFNMIENA                   MPU_CTRL_HFNMIENA_Msk
#define MPU_CTRL_PRIVDEFENA                 MPU_CTRL_PRIVDEFENA_Msk

#define MPU_RNR_REGION                      MPU_RNR_REGION_Msk

#define MPU_RBAR_REGION_MASK                MPU_RBAR_REGION_Msk
#define MPU_RBAR_VALID                      MPU_RBAR_VALID_Msk
#define MPU_RBAR_ADDR_MASK                  MPU_RBAR_ADDR_Msk

#define MPU_RASR_ENABLE                     MPU_RASR_ENABLE_Msk
#define MPU_RASR_SIZE_MASK                  MPU_RASR_SIZE_Msk
#define MPU_RASR_SIZE(n)                    ((n) << MPU_RASR_SIZE_Pos)
#define MPU_RASR_SIZE_32                    MPU_RASR_SIZE(4)
#define MPU_RASR_SIZE_64                    MPU_RASR_SIZE(5)
#define MPU_RASR_SIZE_128                   MPU_RASR_SIZE(6)
#define MPU_RASR_SIZE_256                   MPU_RASR_SIZE(7)
#define MPU_RASR_SIZE_512                   MPU_RASR_SIZE(8)
#define MPU_RASR_SIZE_1K                    MPU_RASR_SIZE(9)
#define MPU_RASR_SIZE_2K                    MPU_RASR_SIZE(10)
#define MPU_RASR_SIZE_4K                    MPU_RASR_SIZE(11)
#define MPU_RASR_SIZE_8K                    MPU_RASR_SIZE(12)
#define MPU_RASR_SIZE_16K                   MPU_RASR_SIZE(13)
#define MPU_RASR_SIZE_32K                   MPU_RASR_SIZE(14)
#define MPU_RASR_SIZE_64K                   MPU_RASR_SIZE(15)
#define MPU_RASR_SIZE_128K                  MPU_RASR_SIZE(16)
#define MPU_RASR_SIZE_256K                  MPU_RASR_SIZE(17)
#define MPU_RASR_SIZE_512K                  MPU_RASR_SIZE(18)
#define MPU_RASR_SIZE_1M                    MPU_RASR_SIZE(19)
#define MPU_RASR_SIZE_2M                    MPU_RASR_SIZE(20)
#define MPU_RASR_SIZE_4M                    MPU_RASR_SIZE(21)
#define MPU_RASR_SIZE_8M                    MPU_RASR_SIZE(22)
#define MPU_RASR_SIZE_16M                   MPU_RASR_SIZE(23)
#define MPU_RASR_SIZE_32M                   MPU_RASR_SIZE(24)
#define MPU_RASR_SIZE_64M                   MPU_RASR_SIZE(25)
#define MPU_RASR_SIZE_128M                  MPU_RASR_SIZE(26)
#define MPU_RASR_SIZE_256M                  MPU_RASR_SIZE(27)
#define MPU_RASR_SIZE_512M                  MPU_RASR_SIZE(28)
#define MPU_RASR_SIZE_1G                    MPU_RASR_SIZE(29)
#define MPU_RASR_SIZE_2G                    MPU_RASR_SIZE(30)
#define MPU_RASR_SIZE_4G                    MPU_RASR_SIZE(31)
#define MPU_RASR_SRD_MASK                   MPU_RASR_SRD_Msk
#define MPU_RASR_SRD(n)                     ((n)  << MPU_RASR_SRD_Pos)
#define MPU_RASR_SRD_ALL                    (0U   << MPU_RASR_SRD_Pos)
#define MPU_RASR_SRD_DISABLE_SUB0           (1U   << MPU_RASR_SRD_Pos)
#define MPU_RASR_SRD_DISABLE_SUB1           (2U   << MPU_RASR_SRD_Pos)
#define MPU_RASR_SRD_DISABLE_SUB2           (4U   << MPU_RASR_SRD_Pos)
#define MPU_RASR_SRD_DISABLE_SUB3           (8U   << MPU_RASR_SRD_Pos)
#define MPU_RASR_SRD_DISABLE_SUB4           (16U  << MPU_RASR_SRD_Pos)
#define MPU_RASR_SRD_DISABLE_SUB5           (32U  << MPU_RASR_SRD_Pos)
#define MPU_RASR_SRD_DISABLE_SUB6           (64U  << MPU_RASR_SRD_Pos)
#define MPU_RASR_SRD_DISABLE_SUB7           (128U << MPU_RASR_SRD_Pos)

#define MPU_RASR_ATTR_B                     MPU_RASR_B_Msk
#define MPU_RASR_ATTR_C                     MPU_RASR_C_Msk
#define MPU_RASR_ATTR_S                     MPU_RASR_S_Msk
#define MPU_RASR_ATTR_TEX_MASK              MPU_RASR_TEX_Msk
#define MPU_RASR_ATTR_TEX(n)                ((n) << MPU_RASR_TEX_Pos)
#define MPU_RASR_ATTR_AP_MASK               MPU_RASR_AP_Msk
#define MPU_RASR_ATTR_AP(n)                 ((n) << MPU_RASR_AP_Pos)
#define MPU_RASR_ATTR_XN                    MPU_RASR_XN_Msk

// Human Readable region attributes
#define MPU_RASR_ATTR_AP_NA_NA              (0U  << MPU_RASR_AP_Pos)
#define MPU_RASR_ATTR_AP_RW_NA              (1U  << MPU_RASR_AP_Pos)
#define MPU_RASR_ATTR_AP_RW_RO              (2U  << MPU_RASR_AP_Pos)
#define MPU_RASR_ATTR_AP_RW_RW              (3U  << MPU_RASR_AP_Pos)
#define MPU_RASR_ATTR_AP_RO_NA              (5U  << MPU_RASR_AP_Pos)
#define MPU_RASR_ATTR_AP_RO_RO              (6U  << MPU_RASR_AP_Pos)

#define MPU_RASR_ATTR_STRONGLY_ORDERED      (MPU_RASR_ATTR_TEX(0))
#define MPU_RASR_ATTR_SHARED_DEVICE         (MPU_RASR_ATTR_TEX(0) | MPU_RASR_ATTR_B)
#define MPU_RASR_ATTR_CACHEABLE_WT_NWA      (MPU_RASR_ATTR_TEX(0) | MPU_RASR_ATTR_C)
#define MPU_RASR_ATTR_CACHEABLE_WB_NWA      (MPU_RASR_ATTR_TEX(0) | MPU_RASR_ATTR_B | MPU_RASR_ATTR_C)
#define MPU_RASR_ATTR_NON_CACHEABLE         (MPU_RASR_ATTR_TEX(1))
#define MPU_RASR_ATTR_CACHEABLE_WB_WA       (MPU_RASR_ATTR_TEX(1) | MPU_RASR_ATTR_B | MPU_RASR_ATTR_C)
#define MPU_RASR_ATTR_NON_SHARED_DEVICE     (MPU_RASR_ATTR_TEX(2))

#define MPU_REGION_0                        0
#define MPU_REGION_1                        1
#define MPU_REGION_2                        2
#define MPU_REGION_3                        3
#define MPU_REGION_4                        4
#define MPU_REGION_5                        5
#define MPU_REGION_6                        6
#define MPU_REGION_7                        7


#ifdef __cplusplus
extern "C" {
#endif

static inline void mpu_enable(uint32_t ctrl) {
  MPU->CTRL = ctrl | MPU_CTRL_ENABLE;     // flags + Enable
  SCB->SHCSR |= SCB_SHCSR_MEMFAULTENA_Msk;      // enable MemManage fault
}

static inline void mpu_disable() {
//  SCB->SHCSR &= ~SCB_SHCSR_MEMFAULTENA_Msk;  // disable MemManage fault
  MPU->CTRL = 0;                     // disable
  asm volatile("DSB"); // see PM0214 (en.DM00046982.pdf) page 196
}

static inline void mpu_configure_region(uint8_t region, uint32_t addr, uint32_t attribs) {
  MPU->RASR = 0; // disable region first
  MPU->RBAR = (addr & MPU_RBAR_ADDR_MASK) | region | MPU_RBAR_VALID;    // set region number and address
  MPU->RASR = attribs | MPU_RASR_ENABLE;                                // set flags and enable region
}

#ifdef __cplusplus
}
#endif
