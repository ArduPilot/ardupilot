/*
  implement protocol for controlling an IO microcontroller

  For bootstrapping this will initially implement the px4io protocol,
  but will later move to an ArduPilot specific protocol
 */

#include <AP_HAL/AP_HAL.h>

#if HAL_WITH_IO_MCU

#include "ch.h"

#define IOMCU_MAX_CHANNELS 16

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

    /*
      enable sbus output
    */
    bool enable_sbus_out(uint16_t rate_hz);

    /*
      check for new RC input
     */
    bool check_rcinput(uint32_t &last_frame_us, uint8_t &num_channels, uint16_t *channels, uint8_t max_channels);

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

private:
    AP_HAL::UARTDriver &uart;

    static void thread_start(void *ctx);
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

    // time when we last read various pages
    uint32_t last_status_read_ms;
    uint32_t last_rc_read_ms;
    uint32_t last_servo_read_ms;
    uint32_t last_debug_ms;

    void send_servo_out(void);
    void read_rc_input(void);
    void read_servo(void);
    void read_status(void);
    void print_debug(void);
    void discard_input(void);
    void event_failed(uint8_t event);
    
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
    } reg_status;

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
    } rc_input;
    
    // output pwm values
    struct {
        uint8_t num_channels;
        uint16_t pwm[IOMCU_MAX_CHANNELS];
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
};

#endif // HAL_WITH_IO_MCU
