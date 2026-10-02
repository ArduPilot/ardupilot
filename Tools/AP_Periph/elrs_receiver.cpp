#include "AP_Periph.h"
#include "elrs_receiver.h"

#if AP_PERIPH_ELRS_ENABLED

#include <stdio.h>

extern const AP_HAL::HAL &hal;

#define DEBUG 0

// Never print from the radio worker.
#if DEBUG
#define Debug(fmt, ...) printf(fmt "\n", ##__VA_ARGS__)
#else
#define Debug(fmt, ...) do {} while (0)
#endif

void AP_ELRSReceiver::init(int32_t uid1, int32_t uid2, int16_t model_id, bool bind)
{
    if (uid1 < 0 || uid1 > 0xFFFFFF || uid2 < 0 || uid2 > 0xFFFFFF ||
        model_id < 0 || model_id > 255) {
        fail(Error::CONFIGURATION);
        Debug("ELRS: invalid UID/model settings");
        return;
    }
    const uint8_t uid[ELRS::UID_LENGTH] {
        uint8_t(uid1 >> 16), uint8_t(uid1 >> 8), uint8_t(uid1),
        uint8_t(uid2 >> 16), uint8_t(uid2 >> 8), uint8_t(uid2)
    };
    _binding = bind;
    if (!bind && !_tracker.start(uid, model_id)) {
        fail(Error::UNBOUND);
        Debug("ELRS: configure a bound UID first");
        return;
    }
    if (!_radio.begin()) {
        fail(Error::RADIO_INIT);
        Debug("ELRS: SPI probe failed, rev=%04x stage=%u busy=%u reset=%u", _radio.last_firmware_rev,
              unsigned(_radio.init_stage), unsigned(hal.gpio->read(HAL_SX1280_BUSY_PIN)),
              unsigned(hal.gpio->read(HAL_SX1280_RESET_PIN)));
        return;
    }
    if (!_radio.enable_irq_timestamps()) {
        fail(Error::IRQ);
        Debug("ELRS: DIO1 interrupt unavailable");
        return;
    }
    _radio.irq_callback = FUNCTOR_BIND_MEMBER(&AP_ELRSReceiver::packet_received, void, const uint8_t *);
    _invert_iq = bind || (uid[5] & 1);
    _sync_word = ELRS::uid_seed(uid);
    _crc_seed = ELRS::crc_initializer(uid);
    configure_radio();
    _radio.set_output_power(0); // 1 mW telemetry, no external PA
    if (!_radio.healthy()) {
        fail(Error::RADIO_CONFIG);
        Debug("ELRS: radio configuration failed");
        return;
    }
    publish_report(Error::NONE);
    _started = hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ELRSReceiver::worker, void),
                                            "ELRS", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0);
    if (!_started) {
        _radio.standby();
        fail(Error::THREAD);
        Debug("ELRS: receiver thread allocation failed");
        return;
    }
    Debug("ELRS: auto SX1280 all-rates RX worker started, rev=%04x", _radio.last_firmware_rev);
    if (bind) {
        Debug("ELRS: binding active; start TX Bind, reboot RX to cancel");
    }
}

void AP_ELRSReceiver::fail(Error error)
{
    _tracker.stop();
    publish_report(error);
}

const char *AP_ELRSReceiver::error_message()
{
    Error error;
    {
        WITH_SEMAPHORE(_report_sem);
        error = _report.error;
    }
    switch (error) {
    case Error::NONE:
        return nullptr;
    case Error::CONFIGURATION:
        return "ELRS: invalid UID/model configuration";
    case Error::UNBOUND:
        return "ELRS: no bound UID; request binding and reboot";
    case Error::RADIO_INIT:
        return "ELRS: SX1280 initialization failed";
    case Error::IRQ:
        return "ELRS: SX1280 DIO1 interrupt unavailable";
    case Error::RADIO_CONFIG:
        return "ELRS: SX1280 configuration failed";
    case Error::THREAD:
        return "ELRS: receiver thread allocation failed";
    case Error::RADIO_IO:
        return "ELRS: SX1280 I/O failure; reboot required";
    }
    return "ELRS: unknown receiver error";
}

void AP_ELRSReceiver::configure_radio()
{
    const auto &rate = _tracker.rate();
    _configured_rate = _tracker.rate_index();
    _frequency = _tracker.frequency();
    _radio.config(SX1280_LORA_BW_0800, uint8_t(rate.spreading_factor << 4),
                  rate.coding_rate, _frequency, rate.preamble, _invert_iq, rate.payload_size,
                  rate.flrc, _sync_word, _crc_seed);
    _radio.rx();
}

void AP_ELRSReceiver::packet_received(const uint8_t *packet)
{
    if (packet == nullptr) {
        if (_transmitting) {
            _transmitting = false;
            _tx_done++;
            _radio.rx();
        }
        return;
    }
    const uint32_t overruns = _radio.irq_overruns();
    if (overruns != _overruns) {
        _overruns = overruns;
        _tracker.lose();
        return;
    }
    if (_binding) {
        if (!_binding_received && AP_HAL::micros() - _radio.last_irq_us <= ELRS::Tracker::SERVICE_LIMIT_US) {
            _binding_received = ELRS::parse_binding_uid(packet, ELRS::STANDARD_SIZE, _binding_uid);
        }
        return;
    }
    _tracker.receive(packet, _radio.last_irq_us, AP_HAL::micros());
}

void AP_ELRSReceiver::publish_report(Error error)
{
    // Neither owner holds this lock across SPI, waiting, or CAN logging.
    WITH_SEMAPHORE(_report_sem);
#if DEBUG
    _report.stats = _tracker.stats();
    _report.radio = _radio.diagnostics();
    for (uint8_t i = 0; i < 4; i++) {
        _report.primary_channels[i] = _tracker.channels().crsf[i];
    }
    _report.rate = _tracker.rate().wire;
    _report.mode = _tracker.switch_mode();
    _report.overruns = _overruns;
    _report.tx_count = _tx_count;
    _report.tx_done = _tx_done;
    _report.tx_timeout = _tx_timeout;
    _report.telemetry_denom = _tracker.telemetry_denom();
#endif
    _report.state = _tracker.state();
    _report.seen = _tracker.seen_channels();
    _report.last_input_us = _tracker.last_input_us();
    const uint32_t now_us = AP_HAL::micros();
    uint32_t valid_for_us = 0;
    _report.output_count = _tracker.read_channels(now_us, _report.output_channels, valid_for_us);
    _report.output_expires_us = now_us + valid_for_us;
    _report.frame_sequence = _tracker.frame_sequence();
    _report.lq = _tracker.link_quality();
    _report.error = error;
    _report.binding_complete = _binding_received && error == Error::NONE;
    for (uint8_t i = 0; i < ELRS::UID_LENGTH; i++) {
        _report.binding_uid[i] = _binding_uid[i];
    }
}

void AP_ELRSReceiver::worker()
{
    // Only this thread owns the radio and tracker after successful creation.
#if DEBUG
    uint32_t last_sample_ms = 0;
#endif
    while (true) {
        _radio.update();
        if (_binding && _binding_received) {
            _radio.standby();
            if (!_radio.healthy()) {
                fail(Error::RADIO_IO);
            } else {
                publish_report(Error::NONE);
            }
            return; // main thread saves the UID and reboots
        }
        if (_transmitting && AP_HAL::micros() - _tx_started_us >= _tracker.rate().airtime_us + 500U) {
            // Bound TX recovery by this profile's airtime; never wait 15 ms at F1000.
            _radio.standby();
            _radio.rx();
            _transmitting = false;
            _tx_timeout++;
            _tracker.lose();
        }
        if (!_binding) {
            _tracker.advance(AP_HAL::micros());
            if (_configured_rate != _tracker.rate_index()) {
                if (ELRS::Tracker::rate_at(_configured_rate).flrc && _tracker.rate().flrc) {
                    _configured_rate = _tracker.rate_index();
                } else {
                    configure_radio();
                }
            }
        }
        const uint32_t frequency = _tracker.frequency();
        if (frequency != _frequency) {
            _radio.set_frequency_reg(frequency);
            _frequency = frequency;
        }
        uint8_t telemetry[ELRS::FULL_SIZE];
        if (!_transmitting && _tracker.take_telemetry(AP_HAL::micros(), telemetry,
                _radio.last_packet_rssi, _radio.last_packet_snr_raw)) {
            _tx_started_us = AP_HAL::micros();
            _transmitting = true;
            _tx_count++;
            _radio.tx(telemetry);
        }
#if DEBUG
        // Sample only while searching, so diagnostic SPI cannot delay tracked slots.
        const uint32_t now_ms = AP_HAL::millis();
        if (_tracker.state() == ELRS::Tracker::State::SEARCHING && now_ms - last_sample_ms >= 1000) {
            last_sample_ms = now_ms;
            _radio.sample_diagnostics();
        }
#endif
        if (!_radio.healthy()) {
            fail(Error::RADIO_IO);
            return;
        }
        publish_report(Error::NONE);
        // A pending IRQ remains signalled even if it arrives before wait().
        const uint32_t now_us = AP_HAL::micros();
        uint32_t wait_us = _tracker.time_until_slot(now_us);
        if (_transmitting) {
            const uint32_t elapsed = now_us - _tx_started_us;
            const uint32_t timeout = _tracker.rate().airtime_us + 500U;
            const uint32_t tx_wait = elapsed >= timeout ? 0 : timeout - elapsed;
            wait_us = MIN(wait_us, tx_wait);
        }
        (void)_radio.wait_for_irq(wait_us);
    }
}

bool AP_ELRSReceiver::read_bound_uid(uint8_t uid[ELRS::UID_LENGTH])
{
    {
        WITH_SEMAPHORE(_report_sem);
        if (!_started || _binding_consumed || !_report.binding_complete || _report.error != Error::NONE) {
            return false;
        }
        for (uint8_t i = 0; i < ELRS::UID_LENGTH; i++) {
            uid[i] = _report.binding_uid[i];
        }
        _binding_consumed = true;
    }
    Debug("ELRS: binding complete");
    return true;
}

bool AP_ELRSReceiver::read_input(uint16_t values[ELRS::Tracker::OUTPUT_CHANNELS], uint8_t &count,
                                 uint8_t &quality, bool &failsafe)
{
    WITH_SEMAPHORE(_report_sem);
    const uint32_t now_us = AP_HAL::micros();
    count = 0;
    quality = 0;
    failsafe = false;
    if (_started && _report.error == Error::NONE && _report.output_count > 0 &&
        _report.frame_sequence != _last_output_rc &&
        int32_t(now_us - _report.output_expires_us) <= 0) {
        for (uint8_t i = 0; i < _report.output_count; i++) {
            values[i] = _report.output_channels[i];
        }
        // DroneCAN quality is 0..255; use the same snapshot as the channels.
        quality = (uint16_t(_report.lq) * 255U + 50U) / 100U;
        count = _report.output_count;
        _last_output_rc = _report.frame_sequence;
        _input_status.valid(_report.last_input_us);
        return true;
    }
    // Initial search/binding stays silent. After a working link, report loss
    // even if the radio worker has stopped; no stale channel values are sent.
    failsafe = _input_status.failsafe_due(now_us);
    return failsafe;
}

void AP_ELRSReceiver::update()
{
    const uint32_t now_ms = AP_HAL::millis();
    Report report;
    {
        WITH_SEMAPHORE(_report_sem);
        report = _report;
    }
#ifdef HAL_ELRS_LED_PIN
    auto state = ELRS::StatusLED::State::OFF;
    if (report.error != Error::NONE) {
        state = ELRS::StatusLED::State::FAILED;
    } else if (_started) {
        if (_binding) {
            state = ELRS::StatusLED::State::BINDING;
        } else if (report.state == ELRS::Tracker::State::TRACKING &&
                   (report.seen & 0x000F) == 0x000F &&
                   AP_HAL::micros() - report.last_input_us <= ELRS::Tracker::TIMEOUT_US) {
            state = ELRS::StatusLED::State::CONNECTED;
        } else {
            state = ELRS::StatusLED::State::SEARCHING;
        }
    }
    const bool on = _led.update(state, now_ms);
    if (on != _led_on) {
        _led_on = on;
        hal.gpio->write(HAL_ELRS_LED_PIN, HAL_ELRS_LED_ACTIVE_HIGH ? on : !on);
    }
#endif
    if (!_started || now_ms - _last_report_ms < 1000) {
        return;
    }
    _last_report_ms = now_ms;
    if (report.error != Error::NONE) {
        _started = false;
        Debug("ELRS: SPI failure, receiver worker stopped");
        return;
    }
#if DEBUG
    if (_binding) {
        Debug("ELRS: binding waiting; irq=%lu rx=%lu fifo=%lu",
              (unsigned long)report.radio.serviced_irqs, (unsigned long)report.radio.rx_done,
              (unsigned long)report.radio.fifo_rejected);
        return;
    }
    if (report.state == ELRS::Tracker::State::TRACKING && (report.seen & 0x000F) == 0x000F) {
        Debug("ELRS last CRSF ch1=%u ch2=%u ch3=%u ch4=%u",
              unsigned(report.primary_channels[0]), unsigned(report.primary_channels[1]),
              unsigned(report.primary_channels[2]), unsigned(report.primary_channels[3]));
    }
    Debug("ELRS rate=%u TX mode=%s channels=%u", report.rate, report.mode == ELRS::SwitchMode::WIDE ? "Wide" : "Hybrid",
          unsigned(report.output_count));
    Debug("ELRS TLM denom=%u tx=%lu done=%lu timeout=%lu lq=%u",
          unsigned(report.telemetry_denom), (unsigned long)report.tx_count,
          (unsigned long)report.tx_done, (unsigned long)report.tx_timeout, unsigned(report.lq));
    const auto &radio = report.radio;
    Debug("ELRS radio st=%02x rssi=%d dio=%u flags=%04x irq=%lu rx=%lu fifo=%lu",
          unsigned(radio.status), int(radio.rssi), unsigned(radio.dio1), unsigned(radio.irq_flags),
          (unsigned long)radio.serviced_irqs, (unsigned long)radio.rx_done,
          (unsigned long)radio.fifo_rejected);
    Debug("ELRS fifo status=%02x len=%u", unsigned(radio.fifo_status), unsigned(radio.fifo_length));
    const auto &stats = report.stats;
    Debug("ELRS uplink data=%lu complete=%lu", (unsigned long)stats.data,
          (unsigned long)stats.transfers);
    Debug("ELRS %s sync=%lu rc=%lu frames=%lu bad=%lu late=%lu",
          report.state == ELRS::Tracker::State::TRACKING ? "track" : "search",
          (unsigned long)stats.sync, (unsigned long)stats.rc, (unsigned long)report.frame_sequence,
          (unsigned long)stats.rejected, (unsigned long)stats.late);
    Debug("ELRS seen=%04x irq=%luus slot=%luus lost=%lu ov=%lu",
          report.seen, (unsigned long)stats.max_service_us,
          (unsigned long)stats.max_slot_late_us, (unsigned long)stats.lost,
          (unsigned long)report.overruns);
#endif
}

#endif // AP_PERIPH_ELRS_ENABLED
