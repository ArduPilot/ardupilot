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
#include <SRV_Channel/SRV_Channel.h>
#include <RC_Channel/RC_Channel.h>
#include <AP_RCProtocol/AP_RCProtocol.h>
#include <AP_InternalError/AP_InternalError.h>
#include <AP_Logger/AP_Logger.h>
#include <ch.h>

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
    IOEVENT_MIXING,
    IOEVENT_GPIO,
};

// max number of consecutve protocol failures we accept before raising
// an error
#define IOMCU_MAX_REPEATED_FAILURES 20

AP_IOMCU::AP_IOMCU(AP_HAL::UARTDriver &_uart) :
    uart(_uart)
{
    singleton = this;
}

#define IOMCU_DEBUG_ENABLE 0

#if IOMCU_DEBUG_ENABLE
#include <stdio.h>
#define debug(fmt, args ...)  do {printf("%s:%d: " fmt "\n", __FUNCTION__, __LINE__, ## args); } while(0)
#else
#define debug(fmt, args ...)
#endif

AP_IOMCU *AP_IOMCU::singleton;

/*
  initialise library, starting thread
 */
void AP_IOMCU::init(void)
{
    // uart runs at 1.5MBit
    uart.begin(1500*1000, 128, 128);
    uart.set_blocking_writes(true);
    uart.set_unbuffered_writes(true);

    AP_BoardConfig *boardconfig = AP_BoardConfig::get_singleton();
    if ((!boardconfig || boardconfig->io_enabled() == 1) && !hal.util->was_watchdog_reset()) {
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
void AP_IOMCU::event_failed(uint32_t event_mask)
{
    // wait 0.5ms then retry
    hal.scheduler->delay_microseconds(500);
    chEvtSignal(thread_ctx, event_mask);
}

/*
  main IO thread loop
 */
void AP_IOMCU::thread_main(void)
{
    thread_ctx = chThdGetSelfX();
    chEvtSignal(thread_ctx, initial_event_mask);

    uart.begin(1500*1000, 128, 128);
    uart.set_blocking_writes(true);
    uart.set_unbuffered_writes(true);

    trigger_event(IOEVENT_INIT);

    while (!do_shutdown) {
        eventmask_t mask = chEvtWaitAnyTimeout(~0, chTimeMS2I(10));

        // check for pending IO events
        if (mask & EVENT_MASK(IOEVENT_SEND_PWM_OUT)) {
            send_servo_out();
        }
        mask &= ~EVENT_MASK(IOEVENT_SEND_PWM_OUT);

        if (mask & EVENT_MASK(IOEVENT_INIT)) {
            // get protocol version
            if (!read_registers(PAGE_CONFIG, 0, sizeof(config)/2, (uint16_t *)&config)) {
                event_failed(mask);
                continue;
            }
            is_chibios_backend = (config.protocol_version == IOMCU_PROTOCOL_VERSION &&
                                  config.protocol_version2 == IOMCU_PROTOCOL_VERSION2);

            // set IO_ARM_OK and FMU_ARMED
            if (!modify_register(PAGE_SETUP, PAGE_REG_SETUP_ARMING, 0,
                                 P_SETUP_ARMING_IO_ARM_OK |
                                 P_SETUP_ARMING_FMU_ARMED |
                                 P_SETUP_ARMING_RC_HANDLING_DISABLED)) {
                event_failed(mask);
                continue;
            }
        }
        mask &= ~EVENT_MASK(IOEVENT_INIT);

        if (mask & EVENT_MASK(IOEVENT_MIXING)) {
            if (!write_registers(PAGE_MIXING, 0, sizeof(mixing)/2, (const uint16_t *)&mixing)) {
                event_failed(mask);
                continue;
            }
        }
        mask &= ~EVENT_MASK(IOEVENT_MIXING);

        if (mask & EVENT_MASK(IOEVENT_FORCE_SAFETY_OFF)) {
            if (!write_register(PAGE_SETUP, PAGE_REG_SETUP_FORCE_SAFETY_OFF, FORCE_SAFETY_MAGIC)) {
                event_failed(mask);
                continue;
            }
        }
        mask &= ~EVENT_MASK(IOEVENT_FORCE_SAFETY_OFF);

        if (mask & EVENT_MASK(IOEVENT_FORCE_SAFETY_ON)) {
            if (!write_register(PAGE_SETUP, PAGE_REG_SETUP_FORCE_SAFETY_ON, FORCE_SAFETY_MAGIC)) {
                event_failed(mask);
                continue;
            }
        }
        mask &= ~EVENT_MASK(IOEVENT_FORCE_SAFETY_ON);

        if (mask & EVENT_MASK(IOEVENT_SET_RATES)) {
            if (!write_register(PAGE_SETUP, PAGE_REG_SETUP_ALTRATE, rate.freq) ||
                !write_register(PAGE_SETUP, PAGE_REG_SETUP_PWM_RATE_MASK, rate.chmask)) {
                event_failed(mask);
                continue;
            }
        }
        mask &= ~EVENT_MASK(IOEVENT_SET_RATES);

        if (mask & EVENT_MASK(IOEVENT_ENABLE_SBUS)) {
            if (!write_register(PAGE_SETUP, PAGE_REG_SETUP_SBUS_RATE, rate.sbus_rate_hz) ||
                !modify_register(PAGE_SETUP, PAGE_REG_SETUP_FEATURES, 0,
                                 P_SETUP_FEATURES_SBUS1_OUT)) {
                event_failed(mask);
                continue;
            }
        }
        mask &= ~EVENT_MASK(IOEVENT_ENABLE_SBUS);

        if (mask & EVENT_MASK(IOEVENT_SET_HEATER_TARGET)) {
            if (!write_register(PAGE_SETUP, PAGE_REG_SETUP_HEATER_DUTY_CYCLE, heater_duty_cycle)) {
                event_failed(mask);
                continue;
            }
        }
        mask &= ~EVENT_MASK(IOEVENT_SET_HEATER_TARGET);

        if (mask & EVENT_MASK(IOEVENT_SET_DEFAULT_RATE)) {
            if (!write_register(PAGE_SETUP, PAGE_REG_SETUP_DEFAULTRATE, rate.default_freq)) {
                event_failed(mask);
                continue;
            }
        }
        mask &= ~EVENT_MASK(IOEVENT_SET_DEFAULT_RATE);

        if (mask & EVENT_MASK(IOEVENT_SET_ONESHOT_ON)) {
            if (!modify_register(PAGE_SETUP, PAGE_REG_SETUP_FEATURES, 0, P_SETUP_FEATURES_ONESHOT)) {
                event_failed(mask);
                continue;
            }
        }
        mask &= ~EVENT_MASK(IOEVENT_SET_ONESHOT_ON);

        if (mask & EVENT_MASK(IOEVENT_SET_BRUSHED_ON)) {
            if (!modify_register(PAGE_SETUP, PAGE_REG_SETUP_FEATURES, 0, P_SETUP_FEATURES_BRUSHED)) {
                event_failed(mask);
                continue;
            }
        }
        mask &= ~EVENT_MASK(IOEVENT_SET_BRUSHED_ON);

        if (mask & EVENT_MASK(IOEVENT_SET_SAFETY_MASK)) {
            if (!write_register(PAGE_SETUP, PAGE_REG_SETUP_IGNORE_SAFETY, pwm_out.safety_mask)) {
                event_failed(mask);
                continue;
            }
        }
        mask &= ~EVENT_MASK(IOEVENT_SET_SAFETY_MASK);

        if (is_chibios_backend) {
            if (mask & EVENT_MASK(IOEVENT_GPIO)) {
                if (!write_registers(PAGE_GPIO, 0, sizeof(GPIO)/sizeof(uint16_t), (const uint16_t*)&GPIO)) {
                    event_failed(mask);
                    continue;
                }
            }
            mask &= ~EVENT_MASK(IOEVENT_GPIO);
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
            write_log();
        }

        if (now - last_servo_read_ms > 50) {
            // read servo out at 20Hz
            read_servo();
            last_servo_read_ms = AP_HAL::millis();
        }

        if (now - last_safety_option_check_ms > 1000) {
            update_safety_options();
            last_safety_option_check_ms = now;
        }

        // update failsafe pwm
        if (pwm_out.failsafe_pwm_set != pwm_out.failsafe_pwm_sent) {
            uint8_t set = pwm_out.failsafe_pwm_set;
            if (write_registers(PAGE_FAILSAFE_PWM, 0, IOMCU_MAX_CHANNELS, pwm_out.failsafe_pwm)) {
                pwm_out.failsafe_pwm_sent = set;
            }
        }

        send_rc_protocols();
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
    uint16_t *r = (uint16_t *)&rc_input;
    if (!read_registers(PAGE_RAW_RCIN, 0, sizeof(rc_input)/2, r)) {
        return;
    }
    if (rc_input.flags_failsafe && rc().ignore_rc_failsafe()) {
        rc_input.flags_failsafe = false;
    }
    if (rc_input.flags_rc_ok && !rc_input.flags_failsafe) {
        rc_last_input_ms = AP_HAL::millis();
    }
}

/*
  read status registers
 */
void AP_IOMCU::read_status()
{
    uint16_t *r = (uint16_t *)&reg_status;
    if (!read_registers(PAGE_STATUS, 0, sizeof(reg_status)/2, r)) {
        read_status_errors++;
        return;
    }
    if (read_status_ok == 0) {
        // reset error count on first good read
        read_status_errors = 0;
    }
    read_status_ok++;

    check_iomcu_reset();

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

void AP_IOMCU::write_log()
{
    uint32_t now = AP_HAL::millis();
    if (now - last_log_ms >= 1000U) {
        last_log_ms = now;
        if (AP_Logger::get_singleton()) {
// @LoggerMessage: IOMC
// @Description: IOMCU diagnostic information
// @Field: TimeUS: Time since system startup
// @Field: RSErr: Status Read error count (zeroed on successful read)
// @Field: Mem: Free memory
// @Field: TS: IOMCU uptime
// @Field: NPkt: Number of packets received by IOMCU
// @Field: Nerr: Protocol failures on MCU side
// @Field: Nerr2: Reported number of failures on IOMCU side
// @Field: NDel: Number of delayed packets received by MCU
            AP::logger().WriteStreaming("IOMC", "TimeUS,RSErr,Mem,TS,NPkt,Nerr,Nerr2,NDel", "QHHIIIII",
                               AP_HAL::micros64(),
                               read_status_errors,
                               reg_status.freemem,
                               reg_status.timestamp_ms,
                               reg_status.total_pkts,
                               total_errors,
                               reg_status.num_errors,
                               num_delayed);
        }
#if IOMCU_DEBUG_ENABLE
        static uint32_t last_io_print;
        if (now - last_io_print >= 5000) {
            last_io_print = now;
            debug("t=%u num=%u mem=%u terr=%u nerr=%u crc=%u opcode=%u rd=%u wr=%u ur=%u ndel=%u\n",
                  now,
                  reg_status.total_pkts,
                  reg_status.freemem,
                  total_errors,
                  reg_status.num_errors,
                  reg_status.err_crc,
                  reg_status.err_bad_opcode,
                  reg_status.err_read,
                  reg_status.err_write,
                  reg_status.err_uart,
                  num_delayed);
        }
#endif // IOMCU_DEBUG_ENABLE
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
    uart.discard_input();
}

/*
  write a packet, retrying as needed
 */
size_t AP_IOMCU::write_wait(const uint8_t *pkt, uint8_t len)
{
    uint8_t wait_count = 5;
    size_t ret;
    do {
        ret = uart.write(pkt, len);
        if (ret == 0) {
            hal.scheduler->delay_microseconds(100);
            num_delayed++;
        }
    } while (ret == 0 && wait_count--);
    return ret;
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
        /*
          the original read protocol is a bit strange, as it
          unnecessarily sends the same size packet that it expects to
          receive. This means reading a large number of registers
          wastes a lot of serial bandwidth. We avoid this overhead
          when we know we are talking to a ChibiOS backend
        */
        pkt_size = 4;
    }

    pkt.crc = crc_crc8((const uint8_t *)&pkt, pkt_size);

    size_t ret = write_wait((uint8_t *)&pkt, pkt_size);

    if (ret != pkt_size) {
        debug("write failed1 %u %u %u\n", unsigned(pkt_size), page, offset);
        protocol_fail_count++;
        return false;
    }

    // wait for the expected number of reply bytes or timeout
    if (!uart.wait_timeout(count*2+4, 10)) {
        debug("t=%u timeout read page=%u offset=%u count=%u\n",
              AP_HAL::millis(), page, offset, count);
        protocol_fail_count++;
        return false;
    }

    uint8_t *b = (uint8_t *)&pkt;
    uint8_t n = uart.available();
    if (n < offsetof(struct IOPacket, regs)) {
        debug("t=%u small pkt %u\n", AP_HAL::millis(), n);
        protocol_fail_count++;
        return false;
    }
    if (pkt.get_size() != n) {
        debug("t=%u bad len %u %u\n", AP_HAL::millis(), n, pkt.get_size());
        protocol_fail_count++;
        return false;
    }
    for (uint8_t i=0; i<n; i++) {
        if (i < sizeof(pkt)) {
            b[i] = uart.read();
        }
    }

    uint8_t got_crc = pkt.crc;
    pkt.crc = 0;
    uint8_t expected_crc = crc_crc8((const uint8_t *)&pkt, pkt.get_size());
    if (got_crc != expected_crc) {
        debug("t=%u bad crc %02x should be %02x n=%u %u/%u/%u\n",
              AP_HAL::millis(), got_crc, expected_crc,
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
    if (protocol_fail_count > IOMCU_MAX_REPEATED_FAILURES) {
        handle_repeated_failures();
    }
    total_errors += protocol_fail_count;
    protocol_fail_count = 0;
    protocol_count++;
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

    const uint8_t pkt_size = pkt.get_size();
    size_t ret = write_wait((uint8_t *)&pkt, pkt_size);

    if (ret != pkt_size) {
        debug("write failed2 %u %u %u %u\n", pkt_size, page, offset, ret);
        protocol_fail_count++;
        return false;
    }

    // wait for the expected number of reply bytes or timeout
    if (!uart.wait_timeout(4, 10)) {
        debug("no reply for %u/%u/%u\n", page, offset, count);
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
    if (protocol_fail_count > IOMCU_MAX_REPEATED_FAILURES) {
        handle_repeated_failures();
    }
    total_errors += protocol_fail_count;
    protocol_fail_count = 0;
    protocol_count++;
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
    // ensure mask is legal for the timer layout
    for (uint8_t i=0; i<ARRAY_SIZE(ch_masks); i++) {
        if (chmask & ch_masks[i]) {
            chmask |= ch_masks[i];
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
    if (last_frame_us != uint32_t(rc_last_input_ms * 1000U)) {
        num_channels = MIN(MIN(rc_input.count, IOMCU_MAX_CHANNELS), max_chan);
        memcpy(channels, rc_input.pwm, num_channels*2);
        last_frame_us = uint32_t(rc_last_input_ms * 1000U);
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
    rate.oneshot_enabled = true;
}

// setup for brushed mode
void AP_IOMCU::set_brushed_mode(void)
{
    trigger_event(IOEVENT_SET_BRUSHED_ON);
    rate.brushed_enabled = true;
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

// update enabled RC protocols mask
void AP_IOMCU::send_rc_protocols()
{
    const uint32_t v = rc().enabled_protocols();
    if (last_rc_protocols == v) {
        return;
    }
    if (write_registers(PAGE_SETUP, PAGE_REG_SETUP_RC_PROTOCOLS, 2, (uint16_t *)&v)) {
        last_rc_protocols = v;
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
        DEV_PRINTF("failed to find %s\n", fw_name);
        return false;
    }
    uint32_t crc = crc32_small(0, fw, fw_size);

    // pad CRC to max size
	for (uint32_t i=0; i<flash_size-fw_size; i++) {
		uint8_t b = 0xff;
        crc = crc32_small(crc, &b, 1);
	}

    uint32_t io_crc = 0;
    uint8_t tries = 32;
    while (tries--) {
        if (read_registers(PAGE_SETUP, PAGE_REG_SETUP_CRC, 2, (uint16_t *)&io_crc)) {
            break;
        }
    }
    if (io_crc == crc) {
        DEV_PRINTF("IOMCU: CRC ok\n");
        crc_is_ok = true;
        AP_ROMFS::free(fw);
        fw = nullptr;
        return true;
    } else {
        DEV_PRINTF("IOMCU: CRC mismatch expected: 0x%X got: 0x%X\n", (unsigned)crc, (unsigned)io_crc);
    }

    const uint16_t magic = REBOOT_BL_MAGIC;
    write_registers(PAGE_SETUP, PAGE_REG_SETUP_REBOOT_BL, 1, &magic);

    if (!upload_fw()) {
        AP_ROMFS::free(fw);
        fw = nullptr;
        AP_BoardConfig::config_error("Failed to update IO firmware");
    }

    AP_ROMFS::free(fw);
    fw = nullptr;
    return false;
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
    return crc_is_ok && protocol_fail_count == 0 && !detected_io_reset && read_status_errors < read_status_ok/128U;
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
        if (i == 2 && c->get_type() == RC_Channel::ControlType::ANGLE) {
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
    return AP_RCProtocol::protocol_name_from_protocol((AP_RCProtocol::rcprotocol_t)rc_input.rc_protocol);
}

/*
  we have had a series of repeated protocol failures to the
  IOMCU. This may indicate that the IOMCU has been reset (possibly due
  to a watchdog).
 */
void AP_IOMCU::handle_repeated_failures(void)
{
    if (protocol_count < 100) {
        // we're just starting up, ignore initial failures caused by
        // initial sync with IOMCU
        return;
    }
    INTERNAL_ERROR(AP_InternalError::error_t::iomcu_fail);
}

/*
  check for IOMCU reset (possibly due to a watchdog).
 */
void AP_IOMCU::check_iomcu_reset(void)
{
    if (last_iocmu_timestamp_ms == 0) {
        // initialisation
        last_iocmu_timestamp_ms = reg_status.timestamp_ms;
        DEV_PRINTF("IOMCU startup\n");
        return;
    }
    uint32_t dt_ms = reg_status.timestamp_ms - last_iocmu_timestamp_ms;
    uint32_t ts1 = last_iocmu_timestamp_ms;
    // when we are in an expected delay allow for a larger time
    // delta. This copes with flash erase, such as bootloader update
    const uint32_t max_delay = hal.scheduler->in_expected_delay()?5000:500;
    last_iocmu_timestamp_ms = reg_status.timestamp_ms;

    if (dt_ms < max_delay) {
        // all OK
        last_safety_off = reg_status.flag_safety_off;
        return;
    }
    detected_io_reset = true;
    INTERNAL_ERROR(AP_InternalError::error_t::iomcu_reset);
    hal.console->printf("IOMCU reset t=%u %u %u dt=%u\n",
                        unsigned(AP_HAL::millis()), unsigned(ts1), unsigned(reg_status.timestamp_ms), unsigned(dt_ms));

    if (last_safety_off && !reg_status.flag_safety_off && hal.util->get_soft_armed()) {
        AP_BoardConfig *boardconfig = AP_BoardConfig::get_singleton();
        uint16_t options = boardconfig?boardconfig->get_safety_button_options():0;
        if (safety_forced_off || (options & AP_BoardConfig::BOARD_SAFETY_OPTION_BUTTON_ACTIVE_ARMED) == 0) {
            // IOMCU has reset while armed with safety off - force it off
            // again so we can keep flying
            force_safety_off();
        }
    }
    last_safety_off = reg_status.flag_safety_off;

    // we need to ensure the mixer data and the rates are sent over to
    // the IOMCU
    if (mixing.enabled) {
        trigger_event(IOEVENT_MIXING);
    }
    trigger_event(IOEVENT_SET_RATES);
    trigger_event(IOEVENT_SET_DEFAULT_RATE);
    if (rate.oneshot_enabled) {
        trigger_event(IOEVENT_SET_ONESHOT_ON);
    }
    if (rate.brushed_enabled) {
        trigger_event(IOEVENT_SET_BRUSHED_ON);
    }
    if (rate.sbus_rate_hz) {
        trigger_event(IOEVENT_ENABLE_SBUS);
    }
    if (pwm_out.safety_mask) {
        trigger_event(IOEVENT_SET_SAFETY_MASK);
    }
    last_rc_protocols = 0;
}

// Check if pin number is valid and configured for GPIO
bool AP_IOMCU::valid_GPIO_pin(uint8_t pin) const
{
    // sanity check pin number
    if (!convert_pin_number(pin)) {
        return false;
    }

    // check pin is enabled as GPIO
    return ((GPIO.channel_mask & (1U << pin)) != 0);
}

// convert external pin numbers 101 to 108 to internal 0 to 7
bool AP_IOMCU::convert_pin_number(uint8_t& pin) const
{
    if (pin < 101 || pin > 108) {
        return false;
    }
    pin -= 101;
    return true;
}

// set GPIO mask of channels setup for output
void AP_IOMCU::set_GPIO_mask(uint8_t mask)
{
    if (mask == GPIO.channel_mask) {
        return;
    }
    GPIO.channel_mask = mask;
    trigger_event(IOEVENT_GPIO);
}

// write to a output pin
void AP_IOMCU::write_GPIO(uint8_t pin, bool value)
{
    if (!convert_pin_number(pin)) {
        return;
    }
    if (value == ((GPIO.output_mask & (1U << pin)) != 0)) {
        return;
    }
    if (value) {
        GPIO.output_mask |= (1U << pin);
    } else {
        GPIO.output_mask &= ~(1U << pin);
    }
    trigger_event(IOEVENT_GPIO);
}

// toggle a output pin
void AP_IOMCU::toggle_GPIO(uint8_t pin)
{
    if (!convert_pin_number(pin)) {
        return;
    }
    GPIO.output_mask ^= (1U << pin);
    trigger_event(IOEVENT_GPIO);
}


namespace AP {
    AP_IOMCU *iomcu(void) {
        return AP_IOMCU::get_singleton();
    }
};

#endif // HAL_WITH_IO_MCU
