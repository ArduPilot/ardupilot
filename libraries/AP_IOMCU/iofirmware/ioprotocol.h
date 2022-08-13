#pragma once

#include <stdint.h>
#include <AP_Common/AP_Common.h>
/*
  common protocol definitions between AP_IOMCU and iofirmware
 */

// 22 is enough for the rc_input page in one transfer
#define PKT_MAX_REGS 22
#define IOMCU_MAX_CHANNELS 16

//#define IOMCU_DEBUG

struct PACKED IOPacket {
    uint8_t 	count:6;
    uint8_t 	code:2;
    uint8_t 	crc;
    uint8_t 	page;
    uint8_t 	offset;
    uint16_t	regs[PKT_MAX_REGS];

    // get packet size in bytes
    uint8_t get_size(void) const
    {
        return count*2 + 4;
    }
};

/*
  values for pkt.code
 */
enum iocode {
    // read types
    CODE_READ = 0,
    CODE_WRITE = 1,

    // reply codes
    CODE_SUCCESS = 0,
    CODE_CORRUPT = 1,
    CODE_ERROR = 2
};

// IO pages
enum iopage {
    PAGE_CONFIG = 0,
    PAGE_STATUS = 1,
    PAGE_ACTUATORS = 2,
    PAGE_SERVOS = 3,
    PAGE_RAW_RCIN = 4,
    PAGE_RCIN = 5,
    PAGE_RAW_ADC = 6,
    PAGE_PWM_INFO = 7,
    PAGE_SETUP = 50,
    PAGE_DIRECT_PWM = 54,
    PAGE_FAILSAFE_PWM = 55,
    PAGE_GPIO = 201,
};

// setup page registers
#define PAGE_REG_SETUP_FEATURES	0
#define P_SETUP_FEATURES_SBUS1_OUT	1
#define P_SETUP_FEATURES_SBUS2_OUT	2
#define P_SETUP_FEATURES_PWM_RSSI   4
#define P_SETUP_FEATURES_ADC_RSSI   8
#define P_SETUP_FEATURES_ONESHOT   16
#define P_SETUP_FEATURES_BRUSHED   32

#define PAGE_REG_SETUP_ARMING 1
#define P_SETUP_ARMING_IO_ARM_OK (1<<0)
#define P_SETUP_ARMING_FMU_ARMED (1<<1)
#define P_SETUP_ARMING_RC_HANDLING_DISABLED (1<<6)
#define P_SETUP_ARMING_SAFETY_DISABLE_ON	(1 << 11) // disable use of safety button for safety off->on
#define P_SETUP_ARMING_SAFETY_DISABLE_OFF	(1 << 12) // disable use of safety button for safety on->off

#define PAGE_REG_SETUP_PWM_RATE_MASK 2
#define PAGE_REG_SETUP_DEFAULTRATE   3
#define PAGE_REG_SETUP_ALTRATE       4
#define PAGE_REG_SETUP_REBOOT_BL    10
#define PAGE_REG_SETUP_CRC			11
#define PAGE_REG_SETUP_SBUS_RATE    19
#define PAGE_REG_SETUP_IGNORE_SAFETY 20 /* bitmask of surfaces to ignore the safety status */
#define PAGE_REG_SETUP_HEATER_DUTY_CYCLE 21
#define PAGE_REG_SETUP_DSM_BIND     22
#define PAGE_REG_SETUP_RC_PROTOCOLS 23 // uses 2 slots, 23 and 24

// config page registers
#define PAGE_CONFIG_PROTOCOL_VERSION  0
#define PAGE_CONFIG_PROTOCOL_VERSION2 1
#define IOMCU_PROTOCOL_VERSION       4
#define IOMCU_PROTOCOL_VERSION2     10

// magic value for rebooting to bootloader
#define REBOOT_BL_MAGIC 14662

#define PAGE_REG_SETUP_FORCE_SAFETY_OFF 12
#define PAGE_REG_SETUP_FORCE_SAFETY_ON  14
#define FORCE_SAFETY_MAGIC 22027

struct page_config {
    uint16_t protocol_version;
    uint16_t protocol_version2;
};

struct page_reg_status {
    uint16_t freemem;
    uint32_t timestamp_ms;
    uint16_t vservo;
    uint16_t vrssi;
    uint32_t num_errors;
    uint32_t total_pkts;
    uint8_t flag_safety_off;
    uint8_t err_crc;
    uint8_t err_bad_opcode;
    uint8_t err_read;
    uint8_t err_write;
    uint8_t err_uart;
};

struct page_rc_input {
    uint8_t count;
    uint8_t flags_failsafe:1;
    uint8_t flags_rc_ok:1;
    uint8_t rc_protocol;
    uint16_t pwm[IOMCU_MAX_CHANNELS];
    int16_t rssi;
};

struct __attribute__((packed, aligned(2))) page_GPIO {
    uint8_t channel_mask;
    uint8_t output_mask;
};
