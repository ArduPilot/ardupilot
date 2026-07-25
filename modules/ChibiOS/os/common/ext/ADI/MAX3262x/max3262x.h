#ifndef __MAX3262x_H__
#define __MAX3262x_H__

/* To add here support for 620 and 626 */
#if defined(MAX32625)

#define  MXC_ADC_REV                        0
#define  MXC_ADC_REV                        0
#define  MXC_SPIM_REV                       0
#define  MXC_SPIX_REV                       0
#define  MXC_UART_REV                       0
#define  MXC_UART_REV                       0
#define  MXC_UART_REV                       0

#include "MAX32625/adc_regs.h"
#include "MAX32625/aes_regs.h"
#include "MAX32625/clkman_regs.h"
#include "MAX32625/crc_regs.h"
#include "MAX32625/flc_regs.h"
#include "MAX32625/gpio_regs.h"
#include "MAX32625/i2cm_regs.h"
#include "MAX32625/i2cs_regs.h"
#include "MAX32625/icc_regs.h"
#include "MAX32625/ioman_regs.h"
#include "MAX32625/maa_regs.h"
#include "MAX32625/max32625.h"
//#include "MAX32625/mxc_device.h"
#include "MAX32625/owm_regs.h"
#include "MAX32625/pmu_regs.h"
#include "MAX32625/prng_regs.h"
#include "MAX32625/pt_regs.h"
#include "MAX32625/pwrman_regs.h"
#include "MAX32625/pwrseq_regs.h"
#include "MAX32625/rtc_regs.h"
#include "MAX32625/spim_regs.h"
#include "MAX32625/spis_regs.h"
#include "MAX32625/spix_regs.h"
#include "MAX32625/sysman_regs.h"
#include "MAX32625/system_max32625.h"
#include "MAX32625/tmr_regs.h"
#include "MAX32625/tpu_regs.h"
#include "MAX32625/trim_regs.h"
#include "MAX32625/uart_regs.h"
#include "MAX32625/usb_regs.h"
#include "MAX32625/wdt2_regs.h"
#include "MAX32625/wdt_regs.h"
#else
#include "max32625.h"
#endif

#endif /* __MAX3262x_H__ */
