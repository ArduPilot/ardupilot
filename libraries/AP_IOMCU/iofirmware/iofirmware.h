#include <AP_HAL/AP_HAL.h>


#include "ch.h"

#define IOMCU_MAX_CHANNELS 16
#define PKT_MAX_REGS 32
#define PWM_IGNORE_THIS_CHANNEL UINT16_MAX
#define SERVO_COUNT 8
class AP_IOMCU_FW {
public:
    void process_io_packet();

    struct PACKED IOPacket {
        uint8_t 	count:6;
        uint8_t 	code:2;
        uint8_t 	crc;
        uint8_t 	page;
        uint8_t 	offset;
        uint16_t	regs[PKT_MAX_REGS];

        // get packet size in bytes
        uint8_t get_size(void) const {
            return count*2 + 4;
        }
    } rx_io_packet = {0}, tx_io_packet = {0};

    void init();
    void update();
    void calculate_fw_crc(void);

private:
    void pwm_out_update();
    void heater_update();
    void rcin_update();

    bool handle_code_write();
    bool handle_code_read();
    void schedule_reboot(uint32_t time_ms);
    struct PACKED {
        /* default to RSSI ADC functionality */
        uint16_t features = 0;
        uint16_t arming = 0;
        uint16_t pwm_rates = 0;
        uint16_t pwm_defaultrate = 50;
        uint16_t pwm_altrate = 200;
        uint16_t relays_pad = 0;
        uint16_t vbatt_scale = 10000;
        uint16_t reserved1;
        uint16_t reserved2;
        uint16_t set_debug = 0;
        uint16_t reboot_bl = 0;
        uint16_t crc[2] = {0};
        uint16_t rc_thr_failsafe_us;
        uint16_t reserved3;
        uint16_t pwm_reverse = 0;
        uint16_t trim_roll = 0;
        uint16_t trim_pitch = 0;
        uint16_t trim_yaw = 0;
        uint16_t sbus_rate = 72;
        uint16_t ignore_safety = 0;
        uint16_t heater_duty_cycle = 0xFFFFU;
        uint16_t pwm_altclock = 1;
    } reg_setup;

    // PAGE_STATUS values
    struct PACKED {
        uint16_t freemem;
        uint16_t cpuload;
        
        // status flags
        uint16_t flag_outputs_armed:1;
        uint16_t flag_override:1;
        uint16_t flag_rc_ok:1;
        uint16_t flag_rc_ppm:1;
        uint16_t flag_rc_dsm:1;
        uint16_t flag_rc_sbus:1;
        uint16_t flag_fmu_ok:1;
        uint16_t flag_raw_pwm:1;
        uint16_t flag_mixer_ok:1;
        uint16_t flag_arm_sync:1;
        uint16_t flag_init_ok:1;
        uint16_t flag_failsafe:1;
        uint16_t flag_safety_off:1;
        uint16_t flag_fmu_initialised:1;
        uint16_t flag_rc_st24:1;
        uint16_t flag_rc_sumd_srxl:1;
        
        uint16_t alarms;
        uint16_t vbatt;
        uint16_t ibatt;
        uint16_t vservo;
        uint16_t vrssi;
        uint16_t prssi;
    } reg_status = {0};

    // PAGE_RAW_RCIN values
    struct PACKED {
        uint16_t count;
        uint16_t flags_frame_drop:1;
        uint16_t flags_failsafe:1;
        uint16_t flags_dsm11:1;
        uint16_t flags_mapping_ok:1;
        uint16_t flags_rc_ok:1;
        uint16_t flags_unused:11;
        uint16_t nrssi;
        uint16_t data;
        uint16_t frame_count;
        uint16_t lost_frame_count;
        uint16_t pwm[IOMCU_MAX_CHANNELS];
        uint16_t last_frame_count;
        uint32_t last_input_us;
    } rc_input = {0};

    // PAGE_SERVO values
    struct {
        uint16_t pwm[IOMCU_MAX_CHANNELS] = {0};
    } reg_servo;

    // PAGE_SERVO values
    struct {
        uint16_t pwm[IOMCU_MAX_CHANNELS]= {0};
    } reg_direct_pwm;

    // output rates
    struct {
        uint16_t freq;
        uint16_t chmask;
        uint16_t default_freq = 50;
        uint16_t sbus_rate_hz;
    } rate;

    uint8_t last_page;
    uint8_t last_offset;
    uint32_t fmu_data_received_time;
    uint32_t last_heater_ms;
    uint32_t reboot_time;
    bool do_reboot;
    bool update_default_rate;
    bool update_rcout_freq;
    bool has_heater;
    uint32_t last_blue_led_ms;
};

