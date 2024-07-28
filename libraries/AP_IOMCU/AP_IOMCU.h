/*
  implement protocol for controlling an IO microcontroller

  For bootstrapping this will initially implement the px4io protocol,
  but will later move to an ArduPilot specific protocol
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_IO_MCU

#include "iofirmware/ioprotocol.h"
#include <AP_RCMapper/AP_RCMapper.h>
#include <AP_HAL/RCOutput.h>
#include <AP_ESC_Telem/AP_ESC_Telem_Backend.h>

typedef uint32_t eventmask_t;
typedef struct ch_thread thread_t;

class AP_IOMCU
#ifdef HAL_WITH_ESC_TELEM
  : public AP_ESC_Telem_Backend
#endif
{
public:
    AP_IOMCU(AP_HAL::UARTDriver &uart);

    void init(void);

    // write to one channel
    void write_channel(uint8_t chan, uint16_t pwm);

    // read from one channel
    uint16_t read_channel(uint8_t chan);

    // cork output
    void cork(void);

    // push output
    void push(void);

    // set output frequency
    void set_freq(uint16_t chmask, uint16_t freq);

    // get output frequency
    uint16_t get_freq(uint16_t chan);

    // get state of safety switch
    AP_HAL::Util::safety_state get_safety_switch_state(void) const;

    // force safety on
    bool force_safety_on(void);

    // force safety off
    void force_safety_off(void);

    // set mask of channels that ignore safety state
    void set_safety_mask(uint16_t chmask);

    // set PWM of channels when in FMU failsafe
    void set_failsafe_pwm(uint16_t chmask, uint16_t period_us);

    /*
      enable sbus output
    */
    bool enable_sbus_out(uint16_t rate_hz);

    /*
      check for new RC input
     */
    bool check_rcinput(uint32_t &last_frame_us, uint8_t &num_channels, uint16_t *channels, uint8_t max_channels);

    // Do DSM receiver binding
    void bind_dsm(uint8_t mode);

    // get the name of the RC protocol
    const char *get_rc_protocol(void);

    // get receiver RSSI
    int16_t get_RSSI(void) const {
        return rc_input.rssi;
    }
    
    /*
      get servo rail voltage adc counts
     */
    uint16_t get_vservo_adc_count(void) const { return reg_status.vservo; }

    /*
      get rssi voltage adc counts
     */
    uint16_t get_vrssi_adc_count(void) const { return reg_status.vrssi; }

    // set target for IMU heater
    void set_heater_duty_cycle(uint8_t duty_cycle);

    // set default output rate
    void set_default_rate(uint16_t rate_hz);

    // set to oneshot mode
    void set_oneshot_mode(void);

    // set to brushed mode
    void set_brushed_mode(void);

    // set output mode
    void set_output_mode(uint16_t mask, uint16_t mode);

    // set bi-directional mask
    void set_bidir_dshot_mask(uint16_t mask);

    // get output mode
    AP_HAL::RCOutput::output_mode get_output_mode(uint8_t& mask) const;

    // MCUID
    uint32_t get_mcu_id() const { return config.mcuid; }

    // CPUID
    uint32_t get_cpu_id() const { return config.cpuid; }

#if HAL_DSHOT_ENABLED
    // set dshot output period
    void set_dshot_period(uint16_t period_us, uint8_t drate);

    // set telem request mask
    void set_telem_request_mask(uint32_t mask);

    // set the dshot esc_type
    void set_dshot_esc_type(AP_HAL::RCOutput::DshotEscType dshot_esc_type);

    // send a dshot command
    void send_dshot_command(uint8_t command, uint8_t chan, uint32_t command_timeout_ms, uint16_t repeat_count, bool priority);
#endif
    // setup channels
    void     enable_ch(uint8_t ch);
    void     disable_ch(uint8_t ch);

    // check if IO is healthy
    bool healthy(void);

    // shutdown IO protocol (for reboot)
    void shutdown();

    void soft_reboot();

    // setup for FMU failsafe mixing
    bool setup_mixing(RCMapper *rcmap, int8_t override_chan,
                      float mixing_gain, uint16_t manual_rc_mask);

    // Check if pin number is valid and configured for GPIO
    bool valid_GPIO_pin(uint8_t pin) const;

    // convert external pin numbers 101 to 108 to internal 0 to 7
    bool convert_pin_number(uint8_t& pin) const;

    // set GPIO mask of channels setup for output
    void set_GPIO_mask(uint8_t mask);

    // Get GPIO mask of channels setup for output
    uint8_t get_GPIO_mask() const;

    // write to a output pin
    void write_GPIO(uint8_t pin, bool value);

    // Read the last output value send to the GPIO pin
    // This is not a real read of the actual pin
    // This allows callers to check for state change
    uint8_t read_virtual_GPIO(uint8_t pin) const;

    // toggle a output pin
    void toggle_GPIO(uint8_t pin);

    // channel group masks
    const uint8_t ch_masks[3] = { 0x03,0x0C,0xF0 };

    static AP_IOMCU *get_singleton(void) {
        return singleton;
    }

private:
    AP_HAL::UARTDriver &uart;

    void thread_main(void);

    // read count 16 bit registers
    bool read_registers(uint8_t page, uint8_t offset, uint8_t count, uint16_t *regs);

    // write count 16 bit registers
    bool write_registers(uint8_t page, uint8_t offset, uint8_t count, const uint16_t *regs);

    // write a single register
    bool write_register(uint8_t page, uint8_t offset, uint16_t v) {
        return write_registers(page, offset, 1, &v);
    }

    // modify a single register
    bool modify_register(uint8_t page, uint8_t offset, uint16_t clearbits, uint16_t setbits);

    // trigger an ioevent
    void trigger_event(uint8_t event);

    // IOMCU thread
    thread_t *thread_ctx;

    eventmask_t initial_event_mask;

    // time when we last read various pages
    uint32_t last_status_read_ms;
    uint32_t last_rc_read_ms;
    uint32_t last_servo_read_ms;
    uint32_t last_safety_option_check_ms;
    uint32_t last_reg_access_ms;
    uint32_t last_erpm_read_ms;
    uint32_t last_telem_read_ms;

    // last value of safety options
    uint16_t last_safety_options = 0xFFFF;

    // have we forced the safety off?
    bool safety_forced_off;

    // was safety off on last status?
    bool last_safety_off;

    void send_servo_out(void);
    void read_rc_input(void);
    void read_erpm(void);
    void read_telem(void);
    void read_servo(void);
    void read_status(void);
    void discard_input(void);
    void event_failed(uint32_t event_mask);
    void update_safety_options(void);
    void send_rc_protocols(void);

    // CONFIG page
    struct page_config config;

    // PAGE_STATUS values
    struct page_reg_status reg_status;
    uint32_t last_log_ms;

    // PAGE_RAW_RCIN values
    struct page_rc_input rc_input;
    uint32_t rc_last_input_ms;

    // MIXER values
    struct page_mixing mixing;

    // output pwm values
    struct {
        uint8_t num_channels;
        uint16_t pwm[IOMCU_MAX_RC_CHANNELS];
        uint16_t safety_mask;
        uint16_t failsafe_pwm[IOMCU_MAX_RC_CHANNELS];
        uint8_t failsafe_pwm_set;
        uint8_t failsafe_pwm_sent;
        uint16_t channel_mask;
    } pwm_out;

    // read back pwm values
    struct {
        uint16_t pwm[IOMCU_MAX_RC_CHANNELS];
    } pwm_in;

    // output rates
    struct {
        uint16_t freq;
        uint16_t chmask;
        uint16_t default_freq = 50;
        uint16_t sbus_rate_hz;
        bool oneshot_enabled;
        bool brushed_enabled;
    } rate;

    struct {
        uint16_t period_us;
        uint16_t rate;
    } dshot_rate;

#if HAL_WITH_IO_MCU_BIDIR_DSHOT
    // bi-directional dshot erpm values
    struct page_dshot_erpm dshot_erpm;
    struct page_dshot_telem dshot_telem[IOMCU_MAX_TELEM_CHANNELS/4];
    uint8_t esc_group;
#endif
    // queue of dshot commands that need sending
    ObjectBuffer<page_dshot> dshot_command_queue{8};

    struct page_GPIO GPIO;
    // output mode values
    struct page_mode_out mode_out;

    // IMU heater duty cycle
    uint8_t heater_duty_cycle;

    uint32_t last_servo_out_us;

    bool corked;
    bool do_shutdown;
    bool done_shutdown;

    bool crc_is_ok;
    bool detected_io_reset;
    bool initialised;
    bool is_chibios_backend;

    uint32_t protocol_fail_count;
    uint32_t protocol_count;
    uint32_t total_errors;
    uint32_t num_delayed;
    uint32_t last_iocmu_timestamp_ms;
    uint32_t read_status_errors;
    uint32_t read_status_ok;
    uint32_t last_rc_protocols;

    // firmware upload
    const char *fw_name = "io_firmware.bin";
    const char *dshot_fw_name = "io_firmware_dshot.bin";
    const uint8_t *fw;
    uint32_t fw_size;

    size_t write_wait(const uint8_t *pkt, uint8_t len);
    bool upload_fw(void);
    bool recv_byte_with_timeout(uint8_t *c, uint32_t timeout_ms);
    bool recv_bytes(uint8_t *p, uint32_t count);
    void drain(void);
    bool send(uint8_t c);
    bool send(const uint8_t *p, uint32_t count);
    bool get_sync(uint32_t timeout = 40);
    bool sync();
    bool get_info(uint8_t param, uint32_t &val);
    bool erase();
    bool program(uint32_t fw_size);
    bool verify_rev2(uint32_t fw_size);
    bool verify_rev3(uint32_t fw_size_local);
    bool reboot();

    bool check_crc(void);
    void handle_repeated_failures();
    void check_iomcu_reset();

    void write_log();  // handle onboard logging

    static AP_IOMCU *singleton;

    enum {
        PROTO_NOP               = 0x00,
        PROTO_OK                = 0x10,
        PROTO_FAILED            = 0x11,
        PROTO_INSYNC            = 0x12,
        PROTO_INVALID           = 0x13,
        PROTO_BAD_SILICON_REV   = 0x14,
        PROTO_EOC               = 0x20,
        PROTO_GET_SYNC          = 0x21,
        PROTO_GET_DEVICE        = 0x22,
        PROTO_CHIP_ERASE        = 0x23,
        PROTO_CHIP_VERIFY       = 0x24,
        PROTO_PROG_MULTI        = 0x27,
        PROTO_READ_MULTI        = 0x28,
        PROTO_GET_CRC           = 0x29,
        PROTO_GET_OTP           = 0x2a,
        PROTO_GET_SN            = 0x2b,
        PROTO_GET_CHIP          = 0x2c,
        PROTO_SET_DELAY         = 0x2d,
        PROTO_GET_CHIP_DES      = 0x2e,
        PROTO_REBOOT            = 0x30,

        INFO_BL_REV       = 1,        /**< bootloader protocol revision */
        BL_REV            = 5,        /**< supported bootloader protocol  */
        INFO_BOARD_ID     = 2,        /**< board type */
        INFO_BOARD_REV    = 3,        /**< board revision */
        INFO_FLASH_SIZE   = 4,        /**< max firmware size in bytes */

        PROG_MULTI_MAX    = 248,      /**< protocol max is 255, must be multiple of 4 */
    };
};

namespace AP {
    AP_IOMCU *iomcu(void);
};

#endif // HAL_WITH_IO_MCU
