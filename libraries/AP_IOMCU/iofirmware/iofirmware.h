#pragma once

#include <stdint.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_RCProtocol/AP_RCProtocol.h>
#include <AP_ESC_Telem/AP_ESC_Telem.h>

#include "hal.h"
#include "ch.h"
#include "ioprotocol.h"

#if AP_HAL_SHARED_DMA_ENABLED
#include <AP_HAL_ChibiOS/shared_dma.h>
#endif

#define PWM_IGNORE_THIS_CHANNEL UINT16_MAX
#define SERVO_COUNT 8

class AP_IOMCU_FW {
public:
    void process_io_packet();

    struct IOPacket rx_io_packet, tx_io_packet;

    void init();
    void update();
    void calculate_fw_crc(void);

    void pwm_out_update();
    void heater_update();
    void rcin_update();
    void erpm_update();
    void telem_update();

    bool handle_code_write();
    bool handle_code_read();
    void schedule_reboot(uint32_t time_ms);
    void safety_update();
    void rcout_config_update();
    void rcin_serial_init();
    void rcin_serial_update();
    void page_status_update(void);
    void fill_failsafe_pwm(void);
    void run_mixer(void);
    int16_t mix_input_angle(uint8_t channel, uint16_t radio_in) const;
    int16_t mix_input_range(uint8_t channel, uint16_t radio_in) const;
    uint16_t mix_output_angle(uint8_t channel, int16_t angle) const;
    uint16_t mix_output_range(uint8_t channel, int16_t value) const;
    int16_t mix_elevon_vtail(int16_t angle1, int16_t angle2, bool first_output) const;
    void dsm_bind_step(void);

    struct {
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
        uint16_t dshot_period_us;
        uint16_t dshot_rate;
        uint16_t channel_mask;
    } reg_setup;

    uint16_t last_channel_mask;

    // CONFIG values
    struct page_config config;

    // PAGE_STATUS values
    struct page_reg_status reg_status;

    // PAGE_RAW_RCIN values
    struct page_rc_input rc_input;
    uint32_t rc_last_input_ms;

    // PAGE_SERVO values
    struct {
        uint16_t pwm[IOMCU_MAX_RC_CHANNELS];    // size has to account for virtual channels via SBUS_OUT
    } reg_servo;

    // PAGE_DIRECT_PWM values
    struct {
        uint16_t pwm[IOMCU_MAX_RC_CHANNELS];
    } reg_direct_pwm;

    // PAGE_FAILSAFE_PWM
    struct {
        uint16_t pwm[IOMCU_MAX_RC_CHANNELS];
    } reg_failsafe_pwm;

    // output rates
    struct {
        uint16_t freq;
        uint16_t chmask;
        uint16_t default_freq = 50;
        uint16_t sbus_rate_hz;
    } rate;

    // output mode values
    struct page_mode_out mode_out;

    uint16_t last_output_mode_mask;
    uint16_t last_output_bdmask;
    uint16_t last_output_esc_type;

    // MIXER values
    struct page_mixing mixing;

    // GPIO masks
    struct page_GPIO GPIO;
    uint8_t last_GPIO_channel_mask;
    void GPIO_write();

    // DSHOT runtime
    struct page_dshot dshot;

#if AP_HAL_SHARED_DMA_ENABLED
    void tx_dma_allocate(ChibiOS::Shared_DMA *ctx);
    void tx_dma_deallocate(ChibiOS::Shared_DMA *ctx);

    ChibiOS::Shared_DMA* tx_dma_handle;
#endif
#ifdef HAL_WITH_BIDIR_DSHOT
    struct page_dshot_erpm dshot_erpm;
    uint32_t last_erpm_us;
    struct page_dshot_telem dshot_telem[IOMCU_MAX_TELEM_CHANNELS/4];
    uint32_t last_telem_ms;
#if HAL_WITH_ESC_TELEM
    AP_ESC_Telem esc_telem;
#endif
#endif

    // true when override channel active
    bool override_active;

    // sbus rate handling
    uint32_t sbus_last_ms;
    uint32_t sbus_interval_ms;

    uint32_t fmu_data_received_time;

    bool pwm_update_pending;
    uint32_t last_heater_ms;
    uint32_t reboot_time;
    bool do_reboot;
    bool update_default_rate;
    bool update_rcout_freq;
    bool has_heater;
    const bool heater_pwm_polarity = IOMCU_IMU_HEATER_POLARITY;
    uint32_t last_blue_led_ms;
    uint32_t safety_update_ms;
    uint32_t safety_button_counter;
    uint8_t led_counter;
    uint32_t last_slow_loop_ms;
    uint32_t last_fast_loop_us;
    thread_t *thread_ctx;
    bool last_safety_off;
    uint32_t last_status_ms;
    uint32_t last_ms;
    uint32_t loop_counter;
    uint8_t dsm_bind_state;
    uint32_t last_dsm_bind_ms;
    uint32_t last_failsafe_ms;
};

// GPIO macros
#define HEATER_SET(on) palWriteLine(HAL_GPIO_PIN_HEATER, (on));
#define BLUE_TOGGLE() palToggleLine(HAL_GPIO_PIN_HEATER);
#define AMBER_SET(on) palWriteLine(HAL_GPIO_PIN_AMBER_LED, !(on));
#define SPEKTRUM_POWER(on) palWriteLine(HAL_GPIO_PIN_SPEKTRUM_PWR_EN, on);
#define SPEKTRUM_SET(on) palWriteLine(HAL_GPIO_PIN_SPEKTRUM_OUT, on);
