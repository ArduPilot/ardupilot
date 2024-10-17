#pragma once

#include <stdint.h>
#include <AP_Common/AP_Common.h>

/*
  common protocol definitions between AP_IOMCU and iofirmware
 */

#ifndef AP_IOMCU_PROFILED_SUPPORT_ENABLED
#define AP_IOMCU_PROFILED_SUPPORT_ENABLED 0
#endif

// 22 is enough for the rc_input page in one transfer
#define PKT_MAX_REGS 22
// The number of channels that can be propagated - due to SBUS_OUT is higher than the physical channels
#define IOMCU_MAX_RC_CHANNELS 16
// The actual number of output channels
#define IOMCU_MAX_CHANNELS 8
#define IOMCU_MAX_TELEM_CHANNELS 4

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
    CODE_NOOP = 2,

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
    PAGE_MIXING = 200,
    PAGE_GPIO = 201,
    PAGE_DSHOT = 202,
    PAGE_RAW_DSHOT_ERPM = 203,
    PAGE_RAW_DSHOT_TELEM_1_4 = 204,
    PAGE_RAW_DSHOT_TELEM_5_8 = 205,
    PAGE_RAW_DSHOT_TELEM_9_12 = 206,
    PAGE_RAW_DSHOT_TELEM_13_16 = 207,
#if AP_IOMCU_PROFILED_SUPPORT_ENABLED
    PAGE_PROFILED = 208,
#endif
};

// setup page registers
#define PAGE_REG_SETUP_FEATURES	0
#define P_SETUP_FEATURES_SBUS1_OUT	1
#define P_SETUP_FEATURES_SBUS2_OUT	2
#define P_SETUP_FEATURES_PWM_RSSI   4
#define P_SETUP_FEATURES_ADC_RSSI   8
#define P_SETUP_FEATURES_ONESHOT   16
#define P_SETUP_FEATURES_BRUSHED   32
#define P_SETUP_FEATURES_HEATER    64

#define PAGE_REG_SETUP_ARMING 1
#define P_SETUP_ARMING_IO_ARM_OK (1<<0)
#define P_SETUP_ARMING_FMU_ARMED (1<<1)
#define P_SETUP_ARMING_RC_HANDLING_DISABLED (1<<6)
#define P_SETUP_ARMING_SAFETY_DISABLE_ON	(1 << 11) // disable use of safety button for safety off->on
#define P_SETUP_ARMING_SAFETY_DISABLE_OFF	(1 << 12) // disable use of safety button for safety on->off

#define PAGE_REG_SETUP_PWM_RATE_MASK 2
#define PAGE_REG_SETUP_DEFAULTRATE   3
#define PAGE_REG_SETUP_ALTRATE       4
#define PAGE_REG_SETUP_OUTPUT_MODE   5
#define PAGE_REG_SETUP_REBOOT_BL    10
#define PAGE_REG_SETUP_CRC			11
#define PAGE_REG_SETUP_SBUS_RATE    19
#define PAGE_REG_SETUP_IGNORE_SAFETY 20 /* bitmask of surfaces to ignore the safety status */
#define PAGE_REG_SETUP_HEATER_DUTY_CYCLE 21
#define PAGE_REG_SETUP_DSM_BIND     22
#define PAGE_REG_SETUP_RC_PROTOCOLS 23 // uses 2 slots, 23 and 24
#define PAGE_REG_SETUP_DSHOT_PERIOD 25
#define PAGE_REG_SETUP_CHANNEL_MASK 27

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

#define PROFILED_ENABLE_MAGIC 123

struct page_config {
    uint16_t protocol_version;
    uint16_t protocol_version2;
    uint32_t mcuid;
    uint32_t cpuid;
};

struct page_reg_status {
    uint16_t freemem;
    uint16_t freemstack;
    uint16_t freepstack;
    uint32_t timestamp_ms;
    uint16_t vservo;
    uint16_t vrssi;
    uint32_t num_errors;
    uint32_t total_pkts;
    uint32_t total_ticks;
    uint32_t total_events;
    uint8_t flag_safety_off;
    uint8_t rcout_mask;
    uint8_t rcout_mode;
    uint8_t err_crc;
    uint8_t err_bad_opcode;
    uint8_t err_read;
    uint8_t err_write;
    uint8_t err_uart;
    uint8_t err_lock;
    uint8_t spare;
};

struct page_rc_input {
    uint8_t count;
    uint8_t flags_failsafe:1;
    uint8_t flags_rc_ok:1;
    uint8_t rc_protocol;
    uint16_t pwm[IOMCU_MAX_RC_CHANNELS];
    int16_t rssi;
};

/*
  data for mixing on FMU failsafe
 */
struct page_mixing {
    uint16_t servo_min[IOMCU_MAX_RC_CHANNELS];
    uint16_t servo_max[IOMCU_MAX_RC_CHANNELS];
    uint16_t servo_trim[IOMCU_MAX_RC_CHANNELS];
    uint8_t servo_function[IOMCU_MAX_RC_CHANNELS];
    uint8_t servo_reversed[IOMCU_MAX_RC_CHANNELS];

    // RC input arrays are in AETR order
    uint16_t rc_min[4];
    uint16_t rc_max[4];
    uint16_t rc_trim[4];
    uint8_t rc_reversed[IOMCU_MAX_RC_CHANNELS];
    uint8_t rc_channel[4];

    // gain for elevon and vtail mixing, x1000
    uint16_t mixing_gain;

    // channel which when high forces mixer
    int8_t rc_chan_override;

    // is the throttle an angle input?
    uint8_t throttle_is_angle;

    // mask of channels which are pure manual in override
    uint16_t manual_rc_mask;

    // enabled needs to be 1 to enable mixing
    uint8_t enabled;

    uint8_t pad;
};

static_assert(sizeof(struct page_mixing) % 2 == 0, "page_mixing must be even size");

struct __attribute__((packed, aligned(2))) page_GPIO {
    uint8_t channel_mask;
    uint8_t output_mask;
};

struct page_mode_out {
    uint16_t mask;
    uint16_t mode;
    uint16_t bdmask;
    uint16_t esc_type;
    uint16_t reversible_mask;
};

struct page_dshot {
    uint16_t telem_mask;
    uint8_t command;
    uint8_t chan;
    uint32_t command_timeout_ms;
    uint8_t repeat_count;
    uint8_t priority;
};

struct page_dshot_erpm {
    uint16_t erpm[IOMCU_MAX_TELEM_CHANNELS];
    uint32_t update_mask;
};

// separate telemetry packet because (a) it's too big otherwise and (b) slower update rate
struct page_dshot_telem {
    uint16_t  error_rate[4]; // as a centi-percentage
    uint16_t  voltage_cvolts[4];
    uint16_t  current_camps[4];
    uint16_t  temperature_cdeg[4];
    uint16_t  types[4];
// if EDTv2 needs to be disabled, IOMCU firmware should be recompiled too, this is the reason
#if AP_EXTENDED_DSHOT_TELEM_V2_ENABLED
    uint8_t   edt2_status[4];
    uint8_t   edt2_stress[4];
#endif
};

#if AP_IOMCU_PROFILED_SUPPORT_ENABLED
struct __attribute__((packed, aligned(2))) page_profiled {
    uint8_t magic;
    uint8_t blue;
    uint8_t red;
    uint8_t green;
};
#endif
