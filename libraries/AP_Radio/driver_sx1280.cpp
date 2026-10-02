/*
  Single-radio SX1280 LoRa/FLRC driver - see driver_sx1280.h for provenance and scope.
*/
#include "driver_sx1280.h"

#if AP_RADIO_SX1280_ENABLED

#include <string.h>
#include <AP_Math/AP_Math.h>

// datasheet register holding the firmware version, used as an SPI comms sanity check
extern const AP_HAL::HAL &hal;

#define REG_LR_FIRMWARE_VERSION_MSB 0x0153

bool AP_SX1280::begin(void)
{
    initialised = false;
    last_firmware_rev = 0;
    init_stage = 1;
    if (!_radio_hal.init()) {
        return false;
    }
    init_stage = 2;
    _radio_hal.reset();
    if (!_radio_hal.healthy()) {
        return false;
    }

    init_stage = 3;
    set_mode(SX1280_MODE_STDBY_RC); // must be in STDBY_RC for SET_REGULATORMODE below

    if (!_radio_hal.healthy()) {
        return false;
    }
    init_stage = 4;
    const uint16_t firmware_rev = ((uint16_t)_radio_hal.read_register(REG_LR_FIRMWARE_VERSION_MSB) << 8) |
                                  _radio_hal.read_register(REG_LR_FIRMWARE_VERSION_MSB + 1);
    last_firmware_rev = firmware_rev;
    if (firmware_rev == 0 || firmware_rev == 0xFFFF) {
        // SPI communication failed
        return false;
    }

    // switch from the default low power RX mode to high sensitivity mode
    _radio_hal.write_register(0x0891, _radio_hal.read_register(0x0891) | 0xC0);

    pwr_current = PWRPENDING_NONE;
    set_output_power(SX1280_POWER_MIN);
    commit_output_power();

    _radio_hal.write_command(SX1280_RADIO_SET_AUTOFS, uint8_t(1));
    initialised = _radio_hal.healthy();
    if (initialised) {
        init_stage = 5;
    }
    return initialised;
}

void AP_SX1280::set_mode(SX1280_RadioOperatingModes_t mode)
{
    uint8_t buf[3];
    switch (mode) {
    case SX1280_MODE_SLEEP:
        _radio_hal.write_command(SX1280_RADIO_SET_SLEEP, (uint8_t)0x01);
        break;
    case SX1280_MODE_STDBY_RC:
        _radio_hal.write_command(SX1280_RADIO_SET_STANDBY, (uint8_t)SX1280_STDBY_RC, 1500);
        break;
    case SX1280_MODE_STDBY_XOSC:
        _radio_hal.write_command(SX1280_RADIO_SET_STANDBY, (uint8_t)SX1280_STDBY_XOSC, 50);
        break;
    case SX1280_MODE_FS:
        _radio_hal.write_command(SX1280_RADIO_SET_FS, (uint8_t)0x00, 70);
        break;
    case SX1280_MODE_RX_CONT:
        buf[0] = SX1280_RADIO_TICK_SIZE_0015_US;
        buf[1] = 0xFF;
        buf[2] = 0xFF; // continuous RX, no timeout
        _radio_hal.write_command(SX1280_RADIO_SET_RX, buf, sizeof(buf), 100);
        break;
    case SX1280_MODE_TX:
        buf[0] = SX1280_RADIO_TICK_SIZE_0015_US;
        buf[1] = 0xFF;
        buf[2] = 0xFF; // TODO: dynamic timeout based on expected on-air-time
        _radio_hal.write_command(SX1280_RADIO_SET_TX, buf, sizeof(buf), 100);
        break;
    default:
        break;
    }
    curr_opmode = mode;
}

void AP_SX1280::config_mod_params_lora(uint8_t bw, uint8_t sf, uint8_t cr)
{
    // packet type must already be set to LoRa before modulation params (datasheet)
    const uint8_t rfparams[3] = { sf, bw, cr };
    _radio_hal.write_command(SX1280_RADIO_SET_MODULATIONPARAMS, rfparams, sizeof(rfparams), 25);

    // datasheet-recommended SF-dependent tweak to the internal detection optimisation register
    switch (sf) {
    case SX1280_LORA_SF5:
    case SX1280_LORA_SF6:
        _radio_hal.write_register(0x0925, 0x1E);
        break;
    case SX1280_LORA_SF7:
    case SX1280_LORA_SF8:
        _radio_hal.write_register(0x0925, 0x37);
        break;
    default:
        _radio_hal.write_register(0x0925, 0x32);
        break;
    }
}

void AP_SX1280::set_packet_params_lora(uint8_t preamble_len, bool invert_iq)
{
    const uint8_t buf[7] = {
        preamble_len,
        SX1280_LORA_PACKET_FIXED_LENGTH,
        payload_length,
        SX1280_LORA_CRC_OFF,
        (uint8_t)(invert_iq ? SX1280_LORA_IQ_INVERTED : SX1280_LORA_IQ_NORMAL),
        0x00,
        0x00,
    };
    _radio_hal.write_command(SX1280_RADIO_SET_PACKETPARAMS, buf, sizeof(buf), 20);
}

void AP_SX1280::set_dio_irq_params(void)
{
    const uint16_t dio1_mask = SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_DONE;
    const uint16_t irq_mask = SX1280_IRQ_TX_DONE | SX1280_IRQ_RX_DONE | SX1280_IRQ_CRC_ERROR |
                              SX1280_IRQ_SYNCWORD_VALID | SX1280_IRQ_SYNCWORD_ERROR;
    const uint8_t buf[8] = {
        (uint8_t)(irq_mask >> 8), (uint8_t)irq_mask,
        (uint8_t)(dio1_mask >> 8), (uint8_t)dio1_mask,
        0, 0, // dio2 unused
        0, 0, // dio3 unused
    };
    _radio_hal.write_command(SX1280_RADIO_SET_DIOIRQPARAMS, buf, sizeof(buf));
}

void AP_SX1280::set_fifo_addr(void)
{
    const uint8_t buf[2] = { 0x00, 0x80 }; // TX base 0x00, RX base 0x80
    _radio_hal.write_command(SX1280_RADIO_SET_BUFFERBASEADDRESS, buf, sizeof(buf));
}

void AP_SX1280::config(uint8_t bw, uint8_t sf, uint8_t cr, uint32_t regfreq,
                       uint8_t preamble_len, bool invert_iq, uint8_t _payload_length,
                       bool flrc, uint32_t sync_word, uint16_t crc_seed)
{
    payload_length = MIN(_payload_length, PAYLOAD_LENGTH_MAX);
    iq_inverted = invert_iq;
    _flrc = flrc;

    set_mode(SX1280_MODE_STDBY_RC);
    _radio_hal.write_command(SX1280_RADIO_SET_PACKETTYPE,
                      uint8_t(flrc ? SX1280_PACKET_TYPE_FLRC : SX1280_PACKET_TYPE_LORA), 20);
    if (flrc) {
        config_flrc(preamble_len, sync_word, crc_seed);
    } else {
        config_mod_params_lora(bw, sf, cr);
        set_packet_params_lora(preamble_len, invert_iq);
    }
    set_frequency_reg(regfreq);
    set_dio_irq_params();
    set_fifo_addr();
}

void AP_SX1280::config_flrc(uint8_t preamble, uint32_t sync_word, uint16_t crc_seed)
{
    const uint8_t modulation[3] {SX1280_FLRC_BR_0_650_BW_0_6, 0, SX1280_FLRC_BT_1};
    _radio_hal.write_command(SX1280_RADIO_SET_MODULATIONPARAMS, modulation, sizeof(modulation), 110);
    const uint8_t params[7] {
        uint8_t((((preamble < 8 ? 8 : preamble) / 4) - 1) << 4),
        SX1280_FLRC_SYNC_WORD_LEN_P32S, SX1280_FLRC_RX_MATCH_SYNC_WORD_1,
        0, payload_length, SX1280_FLRC_CRC_3_BYTE, 0x08
    };
    _radio_hal.write_command(SX1280_RADIO_SET_PACKETPARAMS, params, sizeof(params), 30);
    const uint8_t seed[2] {uint8_t(crc_seed >> 8), uint8_t(crc_seed)};
    _radio_hal.write_register(SX1280_REG_FLRC_CRC_SEED, seed, sizeof(seed));
    uint8_t word[4] {uint8_t(sync_word >> 24), uint8_t(sync_word >> 16),
                     uint8_t(sync_word >> 8), uint8_t(sync_word)
                    };
    // Semtech FLRC sync-word erratum, same correction as upstream for CR 1/2.
    if ((word[0] == 0x8C && word[1] == 0x38) || (word[0] == 0x63 && word[1] == 0x0E)) {
        const uint8_t first = word[0];
        word[0] = word[1];
        word[1] = first;
    }
    _radio_hal.write_register(SX1280_REG_FLRC_SYNC_WORD, word, sizeof(word));
}

void AP_SX1280::set_frequency_reg(uint32_t regfreq)
{
    const uint8_t buf[3] = {
        (uint8_t)((regfreq >> 16) & 0xFF),
        (uint8_t)((regfreq >> 8) & 0xFF),
        (uint8_t)(regfreq & 0xFF),
    };
    _radio_hal.write_command(SX1280_RADIO_SET_RFFREQUENCY, buf, sizeof(buf));
}

void AP_SX1280::set_output_power(int8_t power_dbm)
{
    const uint8_t pwr_new = (uint8_t)(constrain_int16(power_dbm, SX1280_POWER_MIN, SX1280_POWER_MAX) - SX1280_POWER_MIN);
    if ((pwr_pending == PWRPENDING_NONE && pwr_current != pwr_new) || pwr_pending != pwr_new) {
        pwr_pending = pwr_new;
    }
}

void AP_SX1280::commit_output_power(void)
{
    if (pwr_pending == PWRPENDING_NONE) {
        return;
    }
    pwr_current = pwr_pending;
    pwr_pending = PWRPENDING_NONE;
    const uint8_t buf[2] = { pwr_current, (uint8_t)SX1280_RADIO_RAMP_04_US };
    _radio_hal.write_command(SX1280_RADIO_SET_TXPARAMS, buf, sizeof(buf));
}

void AP_SX1280::tx(const uint8_t *data)
{
    // Keep the oscillator running: a 1.5 ms RC standby delay exceeds F1000's slot.
    set_mode(SX1280_MODE_FS);
    commit_output_power();
    _radio_hal.write_buffer(0x00, data, payload_length);
    set_mode(SX1280_MODE_TX);
}

void AP_SX1280::rx(void)
{
    set_mode(SX1280_MODE_RX_CONT);
}

bool AP_SX1280::get_rx_buffer_addr(uint8_t *addr)
{
    uint8_t status[2];
    const uint8_t chip_status = _radio_hal.read_command(SX1280_RADIO_GET_RXBUFFERSTATUS, status, sizeof(status));
    *addr = status[1];
    _diagnostics.fifo_status = chip_status;
    _diagnostics.fifo_length = status[0];
    // Implicit-header LoRa uses the configured payload length. On the bench
    // SX1280 returns zero in the buffer-status length field, as tolerated by
    // upstream ELRS; OTA CRC validation still checks every received frame.
    if (!_radio_hal.healthy()) {
        return false;
    }
    return chip_status == (SX1280_STATUS_CIRCUIT_MODE_RX | SX1280_STATUS_COMMAND_DATA_AVAILABLE);
}

int8_t AP_SX1280::get_rssi_inst(void)
{
    uint8_t status;
    _radio_hal.read_command(SX1280_RADIO_GET_RSSIINST, &status, 1);
    return -(int8_t)(status / 2);
}

void AP_SX1280::sample_diagnostics()
{
    uint8_t rssi = 0;
    _diagnostics.status = _radio_hal.read_command(SX1280_RADIO_GET_RSSIINST, &rssi, 1);
    _diagnostics.rssi = -int16_t(rssi / 2);
    uint8_t flags[2] {};
    _radio_hal.read_command(SX1280_RADIO_GET_IRQSTATUS, flags, sizeof(flags));
    _diagnostics.irq_flags = (uint16_t(flags[0]) << 8) | flags[1];
    _diagnostics.dio1 = _radio_hal.irq_pending();
}

void AP_SX1280::get_last_packet_stats(void)
{
    uint8_t status[2];
    _radio_hal.read_command(SX1280_RADIO_GET_PACKETSTATUS, status, sizeof(status));
    if (_flrc) {
        last_packet_rssi = -int8_t(status[1] / 2);
        last_packet_snr_raw = 0;
        return;
    }
    // DS_SX1280 p84: RSSI/SNR, must subtract SNR from RSSI when SNR is negative
    last_packet_snr_raw = (int8_t)status[1];
    int8_t rssi = -(int8_t)(status[0] / 2);
    if (last_packet_snr_raw < 0) {
        rssi += last_packet_snr_raw / 4; // RADIO_SNR_SCALE
    }
    last_packet_rssi = rssi;
}

bool AP_SX1280::enable_irq_timestamps()
{
    _use_irq_timestamps = hal.gpio->attach_interrupt(HAL_SX1280_DIO1_PIN,
                          FUNCTOR_BIND_MEMBER(&AP_SX1280::dio1_irq, void, uint8_t, bool, uint32_t),
                          AP_HAL::GPIO::INTERRUPT_RISING);
    return _use_irq_timestamps;
}

void AP_SX1280::dio1_irq(uint8_t pin, bool state, uint32_t timestamp)
{
    // Record the event and wake the worker; all SPI stays in thread context.
    if (_irq_pending) {
        _irq_overruns++;
    } else {
        _irq_timestamp = timestamp;
        _irq_pending = true;
    }
    _irq_event.signal_ISR();
}

void AP_SX1280::update(void)
{
    if (!healthy()) {
        return;
    }
    if (_use_irq_timestamps) {
        void *irq_state = hal.scheduler->disable_interrupts_save();
        const bool pending = _irq_pending;
        last_irq_us = _irq_timestamp;
        _irq_pending = false;
        hal.scheduler->restore_interrupts(irq_state);
        if (pending) {
            handle_irq();
        }
    } else if (_radio_hal.irq_pending()) {
        last_irq_us = AP_HAL::micros();
        handle_irq();
    }
}

void AP_SX1280::handle_irq(void)
{
    _diagnostics.serviced_irqs++;
    uint8_t irq_status_buf[2];
    _radio_hal.read_command(SX1280_RADIO_GET_IRQSTATUS, irq_status_buf, sizeof(irq_status_buf));
    const uint16_t irq_status = ((uint16_t)irq_status_buf[0] << 8) | irq_status_buf[1];

    if (!_radio_hal.healthy() || irq_status == SX1280_IRQ_RADIO_NONE) {
        return;
    }
    const bool tx_done = irq_status & SX1280_IRQ_TX_DONE;
    bool packet_ready = false;
    const bool sync_valid = !_flrc || ((irq_status & SX1280_IRQ_SYNCWORD_VALID) &&
                                       !(irq_status & SX1280_IRQ_SYNCWORD_ERROR));
    if (!tx_done && sync_valid && (irq_status & SX1280_IRQ_RX_DONE) && !(irq_status & SX1280_IRQ_CRC_ERROR)) {
        _diagnostics.rx_done++;
        uint8_t fifo_addr = 0;
        if (get_rx_buffer_addr(&fifo_addr)) {
            _radio_hal.read_buffer(fifo_addr, rx_data, payload_length);
            get_last_packet_stats();
            packet_ready = _radio_hal.healthy();
        } else {
            _diagnostics.fifo_rejected++;
        }
    }
    // Preserve DATA_AVAILABLE until the FIFO has been read. Acknowledge before
    // the callback, which may start another RX/TX operation.
    _radio_hal.write_command(SX1280_RADIO_CLR_IRQSTATUS, irq_status_buf, sizeof(irq_status_buf));
    if (!_radio_hal.healthy()) {
        return;
    }
    if (tx_done) {
        curr_opmode = SX1280_MODE_FS;
        commit_output_power();
        if (irq_callback) {
            irq_callback(nullptr);
        }
    } else if (packet_ready && irq_callback) {
        irq_callback(rx_data);
    }
}

#endif  // AP_RADIO_SX1280_ENABLED
