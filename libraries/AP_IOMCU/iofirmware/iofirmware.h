#include <AP_HAL/AP_HAL.h>


#include "ch.h"
#include "ioprotocol.h"

#define IOMCU_MAX_CHANNELS 16
#define PWM_IGNORE_THIS_CHANNEL UINT16_MAX
#define SERVO_COUNT 8

class AP_IOMCU_FW {
public:
    void process_io_packet();

    struct IOPacket rx_io_packet, tx_io_packet;

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
    void safety_update();
    void rcout_mode_update();

    struct PACKED {
        /* default to RSSI ADC functionality */
        uint16_t features;
        uint16_t arming;
        uint16_t pwm_rates;
        uint16_t pwm_defaultrate = 50;
        uint16_t pwm_altrate = 200;
        uint16_t relays_pad;
        uint16_t vbatt_scale = 10000;
        uint16_t reserved1;
        uint16_t reserved2;
        uint16_t set_debug;
        uint16_t reboot_bl;
        uint16_t crc[2];
        uint16_t rc_thr_failsafe_us;
        uint16_t reserved3;
        uint16_t pwm_reverse;
        uint16_t trim_roll;
        uint16_t trim_pitch;
        uint16_t trim_yaw;
        uint16_t sbus_rate = 72;
        uint16_t ignore_safety;
        uint16_t heater_duty_cycle = 0xFFFFU;
        uint16_t pwm_altclock = 1;
    } reg_setup;

    // PAGE_STATUS values
    struct page_reg_status reg_status;

    // PAGE_RAW_RCIN values
    struct page_rc_input rc_input;

    // PAGE_SERVO values
    struct {
        uint16_t pwm[IOMCU_MAX_CHANNELS];
    } reg_servo;

    // PAGE_SERVO values
    struct {
        uint16_t pwm[IOMCU_MAX_CHANNELS];
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
    uint32_t safety_update_ms;
    uint32_t safety_button_counter;
    uint8_t led_counter;
    uint32_t last_loop_ms;
    bool oneshot_enabled;
    thread_t *thread_ctx;
};

