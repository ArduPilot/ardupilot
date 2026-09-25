/*
  Single-radio SX1280 LoRa/FLRC driver (Config/TX/RX/IRQ), sitting on top of
  AP_SX1280_HAL.

  Ported from the ExpressLRS project (lib/SX1280Driver/SX1280.cpp/.h,
  https://github.com/ExpressLRS/ExpressLRS, GPLv3). Simplified from ELRS's
  original class: no dual-radio diversity/gemini TX, no ranging/BLE
  modes, no external PA (RFAMP) control - this board wires a single SX1280
  directly. OTA/FHSS primitives live in elrs_protocol.h; the experimental
  receiver worker owns binding and timed link-statistic transmission.
*/
#pragma once

#include <AP_HAL/AP_HAL.h>
#include <AP_HAL/utility/functor.h>
#include "driver_sx1280_hal.h"

#if AP_RADIO_SX1280_ENABLED

class AP_SX1280
{
public:
    static constexpr uint8_t PAYLOAD_LENGTH_MAX = 16;

    AP_SX1280() = default;

    CLASS_NO_COPY(AP_SX1280);

    // reset + bring up the radio in STDBY_RC, return false if SPI/firmware ID read fails
    bool begin(void);

    // raw REG_LR_FIRMWARE_VERSION_MSB value read during begin(), 0 or 0xFFFF means no SPI response
    uint8_t init_stage = 0; // 1=device, 2=reset, 3=standby, 4=version, 5=ready
    uint16_t last_firmware_rev = 0;

    // configure LoRa modulation/packet params and RF frequency (regfreq units, see sx1280_freq_reg())
    void config(uint8_t bw, uint8_t sf, uint8_t cr, uint32_t regfreq,
                uint8_t preamble_len, bool invert_iq, uint8_t payload_length,
                bool flrc = false, uint32_t sync_word = 0, uint16_t crc_seed = 0);

    void set_frequency_reg(uint32_t regfreq);
    void set_output_power(int8_t power_dbm);

    // start a non-blocking transmit of payload_length bytes (see config())
    void tx(const uint8_t *data);

    // switch to continuous RX
    void rx(void);

    int8_t get_rssi_inst(void);

    struct Diagnostics {
        uint32_t serviced_irqs = 0;
        uint32_t rx_done = 0;
        uint32_t fifo_rejected = 0;
        uint16_t irq_flags = 0;
        uint8_t status = 0;
        uint8_t fifo_status = 0;
        uint8_t fifo_length = 0;
        int8_t rssi = 0;
        bool dio1 = false;
    };
    void sample_diagnostics();
    const Diagnostics &diagnostics() const
    {
        return _diagnostics;
    }

    // last successfully received packet's RSSI (dBm) and raw SNR (see RADIO_SNR_SCALE)
    int8_t last_packet_rssi = 0;
    int8_t last_packet_snr_raw = 0;

    bool enable_irq_timestamps();
    bool wait_for_irq(uint32_t timeout_us)
    {
        return _irq_event.wait(timeout_us);
    }
    void standby()
    {
        set_mode(SX1280_MODE_STDBY_RC);
    }
    bool healthy() const
    {
        return initialised && _radio_hal.healthy();
    }
    uint32_t last_irq_us = 0;
    uint32_t irq_overruns() const
    {
        return _irq_overruns;
    }

    // Poll from thread context only; SPI transactions take a semaphore.
    void update(void);

    // called (from update()) with the received payload on RX done,
    // or with nullptr on TX done. Set by the owner before calling begin().
    FUNCTOR_TYPEDEF(irq_callback_t, void, const uint8_t *);
    irq_callback_t irq_callback;

private:
    static constexpr uint8_t PWRPENDING_NONE = 0x7f;


    Diagnostics _diagnostics;
    AP_SX1280_HAL _radio_hal;
    SX1280_RadioOperatingModes_t curr_opmode = SX1280_MODE_SLEEP;
    uint8_t payload_length = 0;
    bool _flrc = false;
    void config_flrc(uint8_t preamble, uint32_t sync_word, uint16_t crc_seed);
    bool iq_inverted = false;
    bool initialised = false;
    uint8_t pwr_current = PWRPENDING_NONE;
    uint8_t pwr_pending = PWRPENDING_NONE;
    uint8_t rx_data[PAYLOAD_LENGTH_MAX];

    void set_mode(SX1280_RadioOperatingModes_t mode);
    void config_mod_params_lora(uint8_t bw, uint8_t sf, uint8_t cr);
    void set_packet_params_lora(uint8_t preamble_len, bool invert_iq);
    void set_dio_irq_params(void);
    void set_fifo_addr(void);
    bool get_rx_buffer_addr(uint8_t *addr);
    void get_last_packet_stats(void);
    void commit_output_power(void);

    void handle_irq(void);
    void dio1_irq(uint8_t pin, bool state, uint32_t timestamp);
    HAL_BinarySemaphore _irq_event;
    volatile uint32_t _irq_timestamp = 0;
    volatile uint32_t _irq_overruns = 0;
    volatile bool _irq_pending = false;
    bool _use_irq_timestamps = false;
};

#endif  // AP_RADIO_SX1280_ENABLED
