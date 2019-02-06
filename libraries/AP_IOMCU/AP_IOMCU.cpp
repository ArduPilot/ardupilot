/*
  implement protocol for controlling an IO microcontroller

  For bootstrapping this will initially implement the px4io protocol,
  but will later move to an ArduPilot specific protocol
 */

#include "AP_IOMCU.h"

#if HAL_WITH_IO_MCU

#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <AP_BoardConfig/AP_BoardConfig.h>
#include <AP_ROMFS/AP_ROMFS.h>
#include <AP_Math/crc.h>
#include <SRV_Channel/SRV_Channel.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_RCProtocol/AP_RCProtocol.h>

extern const AP_HAL::HAL &hal;

// pending IO events to send, used as an event mask
enum ioevents {
    IOEVENT_INIT=1,
    IOEVENT_SEND_PWM_OUT,
    IOEVENT_FORCE_SAFETY_OFF,
    IOEVENT_FORCE_SAFETY_ON,
    IOEVENT_SET_ONESHOT_ON,
    IOEVENT_SET_BRUSHED_ON,
    IOEVENT_SET_RATES,
    IOEVENT_ENABLE_SBUS,
    IOEVENT_SET_HEATER_TARGET,
    IOEVENT_SET_DEFAULT_RATE,
    IOEVENT_SET_SAFETY_MASK,
    IOEVENT_MIXING
};

AP_IOMCU::AP_IOMCU(AP_HAL::UARTDriver &_uart) :
    uart(_uart)
{}

#if 0
#define debug(fmt, args ...)  do {printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)
#endif

/*
  initialise library, starting thread
 */
void AP_IOMCU::init(void)
{
    // uart runs at 1.5MBit
    uart.begin(1500*1000, 256, 256);
    uart.set_blocking_writes(false);
    uart.set_unbuffered_writes(true);

    // check IO firmware CRC
    hal.scheduler->delay(2000);
    
    AP_BoardConfig *boardconfig = AP_BoardConfig::get_singleton();
    if (!boardconfig || boardconfig->io_enabled() == 1) {
        check_crc();
    } else {
        crc_is_ok = true;
    }

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_IOMCU::thread_main, void), "IOMCU",
                                      1024, AP_HAL::Scheduler::PRIORITY_BOOST, 1)) {
        AP_HAL::panic("Unable to allocate IOMCU thread");
    }
    initialised = true;
}

/*
  handle event failure
 */
void AP_IOMCU::event_failed(uint8_t event)
{
    // wait 0.5ms then retry
    hal.scheduler->delay_microseconds(500);
    trigger_event(event);
}

/*
  main IO thread loop
 */
void AP_IOMCU::thread_main(void)
{
    thread_ctx = chThdGetSelfX();
    chEvtSignal(thread_ctx, initial_event_mask);

    uart.begin(1500*1000, 256, 256);
    uart.set_blocking_writes(false);
    uart.set_unbuffered_writes(true);
    
    trigger_event(IOEVENT_INIT);
    
    while (!do_shutdown) {
        eventmask_t mask = chEvtWaitAnyTimeout(~0, chTimeMS2I(10));

        // check for pending IO events
        if (mask & EVENT_MASK(IOEVENT_SEND_PWM_OUT)) {
            send_servo_out();
        }

        if (mask & EVENT_MASK(IOEVENT_INIT)) {
            // get protocol version
            if (!read_registers(PAGE_CONFIG, 0, sizeof(config)/2, (uint16_t *)&config)) {
                event_failed(IOEVENT_INIT);
                continue;
            }
            is_chibios_backend = (config.protocol_version == IOMCU_PROTOCOL_VERSION &&
                                  config.protocol_version2 == IOMCU_PROTOCOL_VERSION2);

            // set IO_ARM_OK and FMU_ARMED
            if (!modify_register(PAGE_SETUP, PAGE_REG_SETUP_ARMING, 0,
                                 P_SETUP_ARMING_IO_ARM_OK |
                                 P_SETUP_ARMING_FMU_ARMED |
                                 P_SETUP_ARMING_RC_HANDLING_DISABLED)) {
                event_failed(IOEVENT_INIT);
                continue;
            }
        }

        if (mask & EVENT_MASK(IOEVENT_MIXING)) {
            if (!write_registers(PAGE_MIXING, 0, sizeof(mixing)/2, (const uint16_t *)&mixing)) {
                event_failed(IOEVENT_MIXING);
                continue;
            }
        }

        if (mask & EVENT_MASK(IOEVENT_FORCE_SAFETY_OFF)) {
            if (!write_register(PAGE_SETUP, PAGE_REG_SETUP_FORCE_SAFETY_OFF, FORCE_SAFETY_MAGIC)) {
                event_failed(IOEVENT_FORCE_SAFETY_OFF);
                continue;
            }
        }

        if (mask & EVENT_MASK(IOEVENT_FORCE_SAFETY_ON)) {
            if (!write_register(PAGE_SETUP, PAGE_REG_SETUP_FORCE_SAFETY_ON, FORCE_SAFETY_MAGIC)) {
                event_failed(IOEVENT_FORCE_SAFETY_ON);
                continue;
            }
        }

        
        if (mask & EVENT_MASK(IOEVENT_SET_RATES)) {
            if (!write_register(PAGE_SETUP, PAGE_REG_SETUP_ALTRATE, rate.freq) ||
                !write_register(PAGE_SETUP, PAGE_REG_SETUP_PWM_RATE_MASK, rate.chmask)) {
                event_failed(IOEVENT_SET_RATES);
                continue;
            }
        }

        if (mask & EVENT_MASK(IOEVENT_ENABLE_SBUS)) {
            if (!write_register(PAGE_SETUP, PAGE_REG_SETUP_SBUS_RATE, rate.sbus_rate_hz) ||
                !modify_register(PAGE_SETUP, PAGE_REG_SETUP_FEATURES, 0,
                                 P_SETUP_FEATURES_SBUS1_OUT)) {
                event_failed(IOEVENT_ENABLE_SBUS);
                continue;                
            }
        }

        if (mask & EVENT_MASK(IOEVENT_SET_HEATER_TARGET)) {
            if (!write_register(PAGE_SETUP, PAGE_REG_SETUP_HEATER_DUTY_CYCLE, heater_duty_cycle)) {
                event_failed(IOEVENT_SET_HEATER_TARGET);
                continue;
            }
        }

        if (mask & EVENT_MASK(IOEVENT_SET_DEFAULT_RATE)) {
            if (!write_register(PAGE_SETUP, PAGE_REG_SETUP_DEFAULTRATE, rate.default_freq)) {
                event_failed(IOEVENT_SET_DEFAULT_RATE);
                continue;
            }
        }

        if (mask & EVENT_MASK(IOEVENT_SET_ONESHOT_ON)) {
            if (!modify_register(PAGE_SETUP, PAGE_REG_SETUP_FEATURES, 0, P_SETUP_FEATURES_ONESHOT)) {
                event_failed(IOEVENT_SET_ONESHOT_ON);
                continue;
            }
        }

        if (mask & EVENT_MASK(IOEVENT_SET_BRUSHED_ON)) {
            if (!modify_register(PAGE_SETUP, PAGE_REG_SETUP_FEATURES, 0, P_SETUP_FEATURES_BRUSHED)) {
                event_failed(IOEVENT_SET_BRUSHED_ON);
                continue;
            }
        }
        
        if (mask & EVENT_MASK(IOEVENT_SET_SAFETY_MASK)) {
            if (!write_register(PAGE_SETUP, PAGE_REG_SETUP_IGNORE_SAFETY, pwm_out.safety_mask)) {
                event_failed(IOEVENT_SET_SAFETY_MASK);
                continue;
            }
        }

        // check for regular timed events
        uint32_t now = AP_HAL::millis();
        if (now - last_rc_read_ms > 20) {
            // read RC input at 50Hz
            read_rc_input();
            last_rc_read_ms = AP_HAL::millis();
        }
        
        if (now - last_status_read_ms > 50) {
            // read status at 20Hz
            read_status();
            last_status_read_ms = AP_HAL::millis();
        }

        if (now - last_servo_read_ms > 50) {
            // read servo out at 20Hz
            read_servo();
            last_servo_read_ms = AP_HAL::millis();
        }

#ifdef IOMCU_DEBUG
        if (now - last_debug_ms > 1000) {
            print_debug();
            last_debug_ms = AP_HAL::millis();
        }
#endif // IOMCU_DEBUG

        if (now - last_safety_option_check_ms > 1000) {
            update_safety_options();
            last_safety_option_check_ms = now;
        }

        // update safety pwm
        if (pwm_out.safety_pwm_set != pwm_out.safety_pwm_sent) {
            uint8_t set = pwm_out.safety_pwm_set;
            if (write_registers(PAGE_SAFETY_PWM, 0, IOMCU_MAX_CHANNELS, pwm_out.safety_pwm)) {
                pwm_out.safety_pwm_sent = set;
            }
        }

        // update failsafe pwm
        if (pwm_out.failsafe_pwm_set != pwm_out.failsafe_pwm_sent) {
            uint8_t set = pwm_out.failsafe_pwm_set;
            if (write_registers(PAGE_FAILSAFE_PWM, 0, IOMCU_MAX_CHANNELS, pwm_out.failsafe_pwm)) {
                pwm_out.failsafe_pwm_sent = set;
            }
        }
    }
    done_shutdown = true;
}

/*
  send servo output data
 */
void AP_IOMCU::send_servo_out()
{
#if 0
    // simple method to test IO failsafe
    if (AP_HAL::millis() > 30000) {
        return;
    }
#endif
    if (pwm_out.num_channels > 0) {
        uint8_t n = pwm_out.num_channels;
        if (rate.sbus_rate_hz == 0) {
            n = MIN(n, 8);
        } else {
            n = MIN(n, IOMCU_MAX_CHANNELS);
        }
        uint32_t now = AP_HAL::micros();
        if (now - last_servo_out_us >= 2000) {
            // don't send data at more than 500Hz
            if (write_registers(PAGE_DIRECT_PWM, 0, n, pwm_out.pwm)) {
                last_servo_out_us = now;
            }
        }
    }    
}

/*
  read RC input
 */
void AP_IOMCU::read_rc_input()
{
    // read a min of 9 channels and max of IOMCU_MAX_CHANNELS
    uint8_t n = MIN(MAX(9, rc_input.count), IOMCU_MAX_CHANNELS);
    read_registers(PAGE_RAW_RCIN, 0, 6+n, (uint16_t *)&rc_input);
    if (rc_input.flags_rc_ok && !rc_input.flags_failsafe) {
        rc_input.last_input_ms = AP_HAL::millis();
    }
}

/*
  read status registers
 */
void AP_IOMCU::read_status()
{
    uint16_t *r = (uint16_t *)&reg_status;
    read_registers(PAGE_STATUS, 0, sizeof(reg_status)/2, r);

    if (reg_status.flag_safety_off == 0) {
        // if the IOMCU is indicating that safety is on, then force a
        // re-check of the safety options. This copes with a IOMCU reset
        last_safety_options = 0xFFFF;

        // also check if the safety should be definately off.
        AP_BoardConfig *boardconfig = AP_BoardConfig::get_singleton();
        if (!boardconfig) {
            return;
        }
        uint16_t options = boardconfig->get_safety_button_options();
        if (safety_forced_off && (options & AP_BoardConfig::BOARD_SAFETY_OPTION_BUTTON_ACTIVE_SAFETY_ON) == 0) {
            // the safety has been forced off, and the user has asked
            // that the button can never be used, so there should be
            // no way for the safety to be on except a IOMCU
            // reboot. Force safety off again
            force_safety_off();
        }
    }
}

/*
  read servo output values
 */
void AP_IOMCU::read_servo()
{
    if (pwm_out.num_channels > 0) {
        read_registers(PAGE_SERVOS, 0, pwm_out.num_channels, pwm_in.pwm);
    }
}


/*
  discard any pending input
 */
void AP_IOMCU::discard_input(void)
{
    uint32_t n = uart.available();
    while (n--) {
        uart.read();
    }
}


/*
  read count 16 bit registers
*/
bool AP_IOMCU::read_registers(uint8_t page, uint8_t offset, uint8_t count, uint16_t *regs)
{
    while (count > PKT_MAX_REGS) {
        if (!read_registers(page, offset, PKT_MAX_REGS, regs)) {
            return false;
        }
        offset += PKT_MAX_REGS;
        count -= PKT_MAX_REGS;
        regs += PKT_MAX_REGS;
    }

    IOPacket pkt;

    discard_input();
    
    memset(&pkt.regs[0], 0, count*2);

    pkt.code = CODE_READ;
    pkt.count = count;
    pkt.page = page;
    pkt.offset = offset;
    pkt.crc = 0;

    uint8_t pkt_size = pkt.get_size();
    if (is_chibios_backend) {
        // save bandwidth on reads
        pkt_size = 4;
    }
    
    /*
      the protocol is a bit strange, as it unnecessarily sends the
      same size packet that it expects to receive. This means reading
      a large number of registers wastes a lot of serial bandwidth
     */
    pkt.crc = crc_crc8((const uint8_t *)&pkt, pkt_size);
    if (uart.write((uint8_t *)&pkt, pkt_size) != pkt_size) {
        protocol_fail_count++;
        return false;
    }

    // wait for the expected number of reply bytes or timeout
    if (!uart.wait_timeout(count*2+4, 10)) {
        return false;
    }
    
    uint8_t *b = (uint8_t *)&pkt;
    uint8_t n = uart.available();
    for (uint8_t i=0; i<n; i++) {
        if (i < sizeof(pkt)) {
            b[i] = uart.read();
        }
    }

    uint8_t got_crc = pkt.crc;
    pkt.crc = 0;
    uint8_t expected_crc = crc_crc8((const uint8_t *)&pkt, pkt.get_size());
    if (got_crc != expected_crc) {
        debug("bad crc %02x should be %02x n=%u %u/%u/%u\n",
              got_crc, expected_crc,
              n, page, offset, count);
        protocol_fail_count++;
        return false;
    }

    if (pkt.code != CODE_SUCCESS) {
        debug("bad code %02x read %u/%u/%u\n", pkt.code, page, offset, count);
        protocol_fail_count++;
        return false;
    }
    if (pkt.count < count) {
        debug("bad count %u read %u/%u/%u n=%u\n", pkt.count, page, offset, count, n);
        protocol_fail_count++;
        return false;
    }
    memcpy(regs, pkt.regs, count*2);
    protocol_fail_count = 0;
    return true;
}

/*
  write count 16 bit registers
*/
bool AP_IOMCU::write_registers(uint8_t page, uint8_t offset, uint8_t count, const uint16_t *regs)
{
    while (count > PKT_MAX_REGS) {
        if (!write_registers(page, offset, PKT_MAX_REGS, regs)) {
            return false;
        }
        offset += PKT_MAX_REGS;
        count -= PKT_MAX_REGS;
        regs += PKT_MAX_REGS;
    }
    IOPacket pkt;
    
    discard_input();

    memset(&pkt.regs[0], 0, count*2);

    pkt.code = CODE_WRITE;
    pkt.count = count;
    pkt.page = page;
    pkt.offset = offset;
    pkt.crc = 0;
    memcpy(pkt.regs, regs, 2*count);
    pkt.crc = crc_crc8((const uint8_t *)&pkt, pkt.get_size());
    if (uart.write((uint8_t *)&pkt, pkt.get_size()) != pkt.get_size()) {
        protocol_fail_count++;
        return false;
    }

    // wait for the expected number of reply bytes or timeout
    if (!uart.wait_timeout(4, 10)) {
        //debug("no reply for %u/%u/%u\n", page, offset, count);
        protocol_fail_count++;
        return false;
    }
    
    uint8_t *b = (uint8_t *)&pkt;
    uint8_t n = uart.available();
    for (uint8_t i=0; i<n; i++) {
        if (i < sizeof(pkt)) {
            b[i] = uart.read();
        }
    }

    if (pkt.code != CODE_SUCCESS) {
        debug("bad code %02x write %u/%u/%u %02x/%02x n=%u\n",
              pkt.code, page, offset, count,
              pkt.page, pkt.offset, n);
        protocol_fail_count++;
        return false;
    }
    uint8_t got_crc = pkt.crc;
    pkt.crc = 0;
    uint8_t expected_crc = crc_crc8((const uint8_t *)&pkt, pkt.get_size());
    if (got_crc != expected_crc) {
        debug("bad crc %02x should be %02x\n", got_crc, expected_crc);
        protocol_fail_count++;
        return false;
    }
    protocol_fail_count = 0;
    return true;
}

// modify a single register
bool AP_IOMCU::modify_register(uint8_t page, uint8_t offset, uint16_t clearbits, uint16_t setbits)
{
    uint16_t v = 0;
    if (!read_registers(page, offset, 1, &v)) {
        return false;
    }
    uint16_t v2 = (v & ~clearbits) | setbits;
    if (v2 == v) {
        return true;
    }
    return write_registers(page, offset, 1, &v2);
}

void AP_IOMCU::write_channel(uint8_t chan, uint16_t pwm)
{
    if (chan >= IOMCU_MAX_CHANNELS) {
        return;
    }
    if (chan >= pwm_out.num_channels) {
        pwm_out.num_channels = chan+1;
    }
    pwm_out.pwm[chan] = pwm;
    if (!corked) {
        push();
    }
}

void AP_IOMCU::print_debug(void)
{
#ifdef IOMCU_DEBUG
    const uint16_t *r = (const uint16_t *)&reg_status;
    for (uint8_t i=0; i<sizeof(reg_status)/2; i++) {
        hal.console->printf("%04x ", r[i]);
    }
    hal.console->printf("\n");
#endif // IOMCU_DEBUG
}

// trigger an ioevent
void AP_IOMCU::trigger_event(uint8_t event)
{
    if (thread_ctx != nullptr) {
        chEvtSignal(thread_ctx, EVENT_MASK(event));
    } else {
        // thread isn't started yet, trigger this event once it is started
        initial_event_mask |= EVENT_MASK(event);
    }
}

// get state of safety switch
AP_HAL::Util::safety_state AP_IOMCU::get_safety_switch_state(void) const
{
    return reg_status.flag_safety_off?AP_HAL::Util::SAFETY_ARMED:AP_HAL::Util::SAFETY_DISARMED;
}

// force safety on
bool AP_IOMCU::force_safety_on(void)
{
    trigger_event(IOEVENT_FORCE_SAFETY_ON);
    safety_forced_off = false;
    return true;
}

// force safety off
void AP_IOMCU::force_safety_off(void)
{
    trigger_event(IOEVENT_FORCE_SAFETY_OFF);
    safety_forced_off = true;
}

// read from one channel
uint16_t AP_IOMCU::read_channel(uint8_t chan)
{
    return pwm_in.pwm[chan];
}

// cork output
void AP_IOMCU::cork(void)
{
    corked = true;
}

// push output
void AP_IOMCU::push(void)
{
    trigger_event(IOEVENT_SEND_PWM_OUT);
    corked = false;
}

// set output frequency
void AP_IOMCU::set_freq(uint16_t chmask, uint16_t freq)
{
    const uint8_t masks[] = { 0x03,0x0C,0xF0 };
    // ensure mask is legal for the timer layout
    for (uint8_t i=0; i<ARRAY_SIZE(masks); i++) {
        if (chmask & masks[i]) {
            chmask |= masks[i];
        }
    }
    rate.freq = freq;
    rate.chmask |= chmask;
    trigger_event(IOEVENT_SET_RATES);
}

// get output frequency
uint16_t AP_IOMCU::get_freq(uint16_t chan)
{
    if ((1U<<chan) & rate.chmask) {
        return rate.freq;
    }
    return rate.default_freq;
}

// enable SBUS out
bool AP_IOMCU::enable_sbus_out(uint16_t rate_hz)
{
    rate.sbus_rate_hz = rate_hz;
    trigger_event(IOEVENT_ENABLE_SBUS);
    return true;
}

/*
  check for new RC input
*/
bool AP_IOMCU::check_rcinput(uint32_t &last_frame_us, uint8_t &num_channels, uint16_t *channels, uint8_t max_chan)
{
    if (last_frame_us != uint32_t(rc_input.last_input_ms * 1000U)) {
        num_channels = MIN(MIN(rc_input.count, IOMCU_MAX_CHANNELS), max_chan);
        memcpy(channels, rc_input.pwm, num_channels*2);
        last_frame_us = uint32_t(rc_input.last_input_ms * 1000U);
        return true;
    }
    return false;
}

// set IMU heater target
void AP_IOMCU::set_heater_duty_cycle(uint8_t duty_cycle)
{
    heater_duty_cycle = duty_cycle;
    trigger_event(IOEVENT_SET_HEATER_TARGET);
}

// set default output rate
void AP_IOMCU::set_default_rate(uint16_t rate_hz)
{
    if (rate.default_freq != rate_hz) {
        rate.default_freq = rate_hz;
        trigger_event(IOEVENT_SET_DEFAULT_RATE);
    }
}

// setup for oneshot mode
void AP_IOMCU::set_oneshot_mode(void)
{
    trigger_event(IOEVENT_SET_ONESHOT_ON);
}

// setup for brushed mode
void AP_IOMCU::set_brushed_mode(void)
{
    trigger_event(IOEVENT_SET_BRUSHED_ON);
}

// handling of BRD_SAFETYOPTION parameter
void AP_IOMCU::update_safety_options(void)
{
    AP_BoardConfig *boardconfig = AP_BoardConfig::get_singleton();
    if (!boardconfig) {
        return;
    }
    uint16_t desired_options = 0;
    uint16_t options = boardconfig->get_safety_button_options();
    if (!(options & AP_BoardConfig::BOARD_SAFETY_OPTION_BUTTON_ACTIVE_SAFETY_OFF)) {
        desired_options |= P_SETUP_ARMING_SAFETY_DISABLE_OFF;
    }
    if (!(options & AP_BoardConfig::BOARD_SAFETY_OPTION_BUTTON_ACTIVE_SAFETY_ON)) {
        desired_options |= P_SETUP_ARMING_SAFETY_DISABLE_ON;
    }
    if (!(options & AP_BoardConfig::BOARD_SAFETY_OPTION_BUTTON_ACTIVE_ARMED) && hal.util->get_soft_armed()) {
        desired_options |= (P_SETUP_ARMING_SAFETY_DISABLE_ON | P_SETUP_ARMING_SAFETY_DISABLE_OFF);
    }
    if (last_safety_options != desired_options) {
        uint16_t mask = (P_SETUP_ARMING_SAFETY_DISABLE_ON | P_SETUP_ARMING_SAFETY_DISABLE_OFF);
        uint32_t bits_to_set = desired_options & mask;
        uint32_t bits_to_clear = (~desired_options) & mask;
        if (modify_register(PAGE_SETUP, PAGE_REG_SETUP_ARMING, bits_to_clear, bits_to_set)) {
            last_safety_options = desired_options;
        }
    }
}

/*
  check ROMFS firmware against CRC on IOMCU, and if incorrect then upload new firmware
 */
bool AP_IOMCU::check_crc(void)
{
    // flash size minus 4k bootloader
	const uint32_t flash_size = 0x10000 - 0x1000;
    
    fw = AP_ROMFS::find_decompress(fw_name, fw_size);
    if (!fw) {
        hal.console->printf("failed to find %s\n", fw_name);
        return false;
    }
    uint32_t crc = crc_crc32(0, fw, fw_size);

    // pad CRC to max size
	for (uint32_t i=0; i<flash_size-fw_size; i++) {
		uint8_t b = 0xff;
		crc = crc_crc32(crc, &b, 1);
	}

    uint32_t io_crc = 0;
    uint8_t tries = 32;
    while (tries--) {
        if (read_registers(PAGE_SETUP, PAGE_REG_SETUP_CRC, 2, (uint16_t *)&io_crc)) {
            break;
        }
    }
    if (io_crc == crc) {
        hal.console->printf("IOMCU: CRC ok\n");
        crc_is_ok = true;
        free(fw);
        fw = nullptr;
        return true;
    } else {
        hal.console->printf("IOMCU: CRC mismatch expected: 0x%X got: 0x%X\n", (unsigned)crc, (unsigned)io_crc);
    }

    const uint16_t magic = REBOOT_BL_MAGIC;
    write_registers(PAGE_SETUP, PAGE_REG_SETUP_REBOOT_BL, 1, &magic);

    if (!upload_fw()) {
        free(fw);
        fw = nullptr;
        AP_BoardConfig::sensor_config_error("Failed to update IO firmware");
    }
    
    free(fw);
    fw = nullptr;
    return false;
}

/*
  set the pwm to use when safety is on
 */
void AP_IOMCU::set_safety_pwm(uint16_t chmask, uint16_t period_us)
{
    bool changed = false;
    for (uint8_t i=0; i<IOMCU_MAX_CHANNELS; i++) {
        if (chmask & (1U<<i)) {
            if (pwm_out.safety_pwm[i] != period_us) {
                pwm_out.safety_pwm[i] = period_us;
                changed = true;
            }
        }
    }
    if (changed) {
        pwm_out.safety_pwm_set++;
    }
}

/*
  set the pwm to use when in FMU failsafe
 */
void AP_IOMCU::set_failsafe_pwm(uint16_t chmask, uint16_t period_us)
{
    bool changed = false;
    for (uint8_t i=0; i<IOMCU_MAX_CHANNELS; i++) {
        if (chmask & (1U<<i)) {
            if (pwm_out.failsafe_pwm[i] != period_us) {
                pwm_out.failsafe_pwm[i] = period_us;
                changed = true;
            }
        }
    }
    if (changed) {
        pwm_out.failsafe_pwm_set++;
    }
}


// set mask of channels that ignore safety state
void AP_IOMCU::set_safety_mask(uint16_t chmask)
{
    if (pwm_out.safety_mask != chmask) {
        pwm_out.safety_mask = chmask;
        trigger_event(IOEVENT_SET_SAFETY_MASK);        
    }
}

/*
  check that IO is healthy. This should be used in arming checks
 */
bool AP_IOMCU::healthy(void)
{
    return crc_is_ok && protocol_fail_count == 0;
}

/*
  shutdown protocol, ready for reboot
 */
void AP_IOMCU::shutdown(void)
{
    do_shutdown = true;
    while (!done_shutdown) {
        hal.scheduler->delay(1);
    }
}

/*
  request bind on a DSM radio
 */
void AP_IOMCU::bind_dsm(uint8_t mode)
{
    if (!is_chibios_backend || hal.util->get_soft_armed()) {
        // only with ChibiOS IO firmware, and disarmed
        return;
    }
    uint16_t reg = mode;
    write_registers(PAGE_SETUP, PAGE_REG_SETUP_DSM_BIND, 1, &reg);
}

/*
  setup for mixing. This allows fixed wing aircraft to fly in manual
  mode if the FMU dies
 */
bool AP_IOMCU::setup_mixing(RCMapper *rcmap, int8_t override_chan,
                            float mixing_gain, uint16_t manual_rc_mask)
{
    if (!is_chibios_backend) {
        return false;
    }
    bool changed = false;
#define MIX_UPDATE(a,b) do { if ((a) != (b)) { a = b; changed = true; }} while (0)

    // update mixing structure, checking for changes
    for (uint8_t i=0; i<IOMCU_MAX_CHANNELS; i++) {
        const SRV_Channel *c = SRV_Channels::srv_channel(i);
        if (!c) {
            continue;
        }
        MIX_UPDATE(mixing.servo_trim[i], c->get_trim());
        MIX_UPDATE(mixing.servo_min[i], c->get_output_min());
        MIX_UPDATE(mixing.servo_max[i], c->get_output_max());
        MIX_UPDATE(mixing.servo_function[i], c->get_function());
        MIX_UPDATE(mixing.servo_reversed[i], c->get_reversed());
    }
    // update RCMap
    MIX_UPDATE(mixing.rc_channel[0], rcmap->roll());
    MIX_UPDATE(mixing.rc_channel[1], rcmap->pitch());
    MIX_UPDATE(mixing.rc_channel[2], rcmap->throttle());
    MIX_UPDATE(mixing.rc_channel[3], rcmap->yaw());
    for (uint8_t i=0; i<4; i++) {
        const RC_Channel *c = RC_Channels::rc_channel(mixing.rc_channel[i]-1);
        if (!c) {
            continue;
        }
        MIX_UPDATE(mixing.rc_min[i], c->get_radio_min());
        MIX_UPDATE(mixing.rc_max[i], c->get_radio_max());
        MIX_UPDATE(mixing.rc_trim[i], c->get_radio_trim());
        MIX_UPDATE(mixing.rc_reversed[i], c->get_reverse());

        // cope with reversible throttle
        if (i == 2 && c->get_type() == RC_Channel::RC_CHANNEL_TYPE_ANGLE) {
            MIX_UPDATE(mixing.throttle_is_angle, 1);
        } else {
            MIX_UPDATE(mixing.throttle_is_angle, 0);
        }
    }

    MIX_UPDATE(mixing.rc_chan_override, override_chan);
    MIX_UPDATE(mixing.mixing_gain, (uint16_t)(mixing_gain*1000));
    MIX_UPDATE(mixing.manual_rc_mask, manual_rc_mask);

    // and enable
    MIX_UPDATE(mixing.enabled, 1);
    if (changed) {
        trigger_event(IOEVENT_MIXING);
    }
    return true;
}

/*
  return the RC protocol name
 */
const char *AP_IOMCU::get_rc_protocol(void)
{
    if (!is_chibios_backend) {
        return nullptr;
    }
    return AP_RCProtocol::protocol_name_from_protocol((AP_RCProtocol::rcprotocol_t)rc_input.data);
}

#endif // HAL_WITH_IO_MCU
