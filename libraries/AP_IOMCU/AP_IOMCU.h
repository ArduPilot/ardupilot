/*
  implement protocol for controlling an IO microcontroller

  For bootstrapping this will initially implement the px4io protocol,
  but will later move to an ArduPilot specific protocol
 */

#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_IO_MCU

#include "ch.h"
#include "iofirmware/ioprotocol.h"
#include <AP_RCMapper/AP_RCMapper.h>

class AP_IOMCU {
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

    // set PWM of channels when safety is on
    void set_safety_pwm(uint16_t chmask, uint16_t period_us);

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

    /*
      get servo rail voltage
     */
    float get_vservo(void) const { return reg_status.vservo * 0.001; }

    /*
      get rssi voltage
     */
    float get_vrssi(void) const { return reg_status.vrssi * 0.001; }

    // set target for IMU heater
    void set_heater_duty_cycle(uint8_t duty_cycle);

    // set default output rate
    void set_default_rate(uint16_t rate_hz);

    // set to oneshot mode
    void set_oneshot_mode(void);

    // set to brushed mode
    void set_brushed_mode(void);

    // check if IO is healthy
    bool healthy(void);

    // shutdown IO protocol (for reboot)
    void shutdown();

    // setup for FMU failsafe mixing
    bool setup_mixing(RCMapper *rcmap, int8_t override_chan,
                      float mixing_gain, uint16_t manual_rc_mask);

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

    // last value of safety options
    uint16_t last_safety_options = 0xFFFF;

    // have we forced the safety off?
    bool safety_forced_off;

    void send_servo_out(void);
    void read_rc_input(void);
    void read_servo(void);
    void read_status(void);
    void discard_input(void);
    void event_failed(uint8_t event);
    void update_safety_options(void);

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
        uint16_t pwm[IOMCU_MAX_CHANNELS];
        uint8_t safety_pwm_set;
        uint8_t safety_pwm_sent;
        uint16_t safety_pwm[IOMCU_MAX_CHANNELS];
        uint16_t safety_mask;
        uint16_t failsafe_pwm[IOMCU_MAX_CHANNELS];
        uint8_t failsafe_pwm_set;
        uint8_t failsafe_pwm_sent;
    } pwm_out;

    // read back pwm values
    struct {
        uint16_t pwm[IOMCU_MAX_CHANNELS];
    } pwm_in;

    // output rates
    struct {
        uint16_t freq;
        uint16_t chmask;
        uint16_t default_freq = 50;
        uint16_t sbus_rate_hz;
    } rate;

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

    // firmware upload
    const char *fw_name = "io_firmware.bin";
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
