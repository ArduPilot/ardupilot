#include "RCOutput.h"
#include <AP_Math/AP_Math.h>
#include "pwm_multi.pio.h"
//#include <pico-sdk_build/pwm_multi.pio.h>
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"

using namespace RP;

extern const AP_HAL::HAL& hal;

void RCOutput::init() {
    // Initializing internal arrays
    for (uint8_t i = 0; i < MAX_CHANNELS; i++) {
        _periods[i] = 1000; // Safe default value (1ms)
    }
    _enabled_mask = 0;
    _need_update = true;

    // PIO settings
    _pio_offset = pio_add_program(_pio, &pwm_multi_program);
    _sm = pio_claim_unused_sm(_pio, true);

    // Pin configuration
    for (uint i = 0; i < MAX_CHANNELS; i++) {
        pio_gpio_init(_pio, RCOUT_GPIO_BASE + i);
    }
    pio_sm_set_consecutive_pindirs(_pio, _sm, 0, MAX_CHANNELS, true);

    // State Machine Configuration
    pio_sm_config c = pwm_multi_program_get_default_config(_pio_offset);
    sm_config_set_out_pins(&c, 0, MAX_CHANNELS);

    // FIFO settings: autopull every 32 bits
    sm_config_set_out_shift(&c, true, true, 32);

    // Set the PIO frequency. For 1µs resolution at 150MHz sys_clk:
    // divider = 150.0
    float div = (float)clock_get_hz(clk_sys) / 1000000.0f;
    sm_config_set_clkdiv(&c, div);

    pio_sm_init(_pio, _sm, _pio_offset, &c);
    pio_sm_set_enabled(_pio, _sm, true);

    // DMA settings (Double Buffering is not needed if the update is fast)
    _dma_chan = dma_claim_unused_channel(true);
    dma_channel_config dma_cfg = dma_channel_get_default_config(_dma_chan);
    
    channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_cfg, true);
    channel_config_set_write_increment(&dma_cfg, false);
    
    // DMA synchronization with PIO rate (DREQ)
    channel_config_set_dreq(&dma_cfg, pio_get_dreq(_pio, _sm, true));

    // Setting up a ring buffer or restart
    // For ArduPilot simplicity: DMA just chains the _bit_buffer
    int dma_timer = dma_claim_unused_channel(true);
    dma_channel_config timer_cfg = dma_channel_get_default_config(dma_timer);
    channel_config_set_chain_to(&timer_cfg, _dma_chan); // Restarts the main channel

    dma_channel_configure(
        _dma_chan, &dma_cfg,
        &_pio->txf[_sm], // Where: in FIFO PIO
        _bit_buffer,     // From: our buffer
        _pwm_steps,      // How much: the entire period
        true             // Launch immediately
    );

    // Initial buffer formation
    update_bit_buffer();
}

void RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz) {
    if (freq_hz == 0) return;
    // Since we are using one DMA buffer for all 32 channels,
    // the frequency freq_hz will be global for the entire PIO module.
    _current_freq = freq_hz;
    // Calculate the number of PIO ticks per PWM period
    // For example, at 150MHz system frequency and a divider of 150, 1 tick = 1 µs.
    // For 50Hz, the period will be 20,000 ticks (20ms).
    uint32_t clock_hz = clock_get_hz(clk_sys);
    float div = 150.0f; // Fixed divider for 1μs resolution
    uint32_t ticks_per_period = (clock_hz / div) / freq_hz;
    _pwm_steps = (ticks_per_period < PWM_RESOLUTION) ? ticks_per_period : PWM_RESOLUTION;
    _need_update = true;
}

uint16_t RCOutput::get_freq(uint8_t chan) {
    return _current_freq;
}

void RCOutput::enable_ch(uint8_t chan)
{
    if (chan >= MAX_CHANNELS) return;
    _enabled_mask |= (1u << chan);
    _need_update = true;
}

void RCOutput::disable_ch(uint8_t chan)
{
    if (chan >= MAX_CHANNELS) return;
    _enabled_mask &= ~(1u << chan);
    _need_update = true;
}

void RCOutput::write(uint8_t chan, uint16_t period_us)
{
    if (chan >= MAX_CHANNELS) return;
    if (_periods[chan] == period_us) return;

    _periods[chan] = period_us;
    update_bit_buffer();
}

uint16_t RCOutput::read(uint8_t chan)
{
    // Check if the channel number does not exceed the available 32 channels
    if (chan >= MAX_CHANNELS) {
        return 0;
    }
    // Return the stored value from our _periods array.
    // This is the value that was set via write() or initialized in init().
    return _periods[chan];
}

void RCOutput::read(uint16_t* period_us, uint8_t len)
{
    // Determine how many channels we can read
    uint8_t count = (len < MAX_CHANNELS) ? len : MAX_CHANNELS;
    for (uint8_t i = 0; i < count; i++) {
        // Return values from our internal _periods array
        period_us[i] = _periods[i];
    }
    // If more channels are requested than we have, we will reset the rest to zero
    for (uint8_t i = count; i < len; i++) {
        period_us[i] = 0;
    }
}

void RCOutput::update_bit_buffer() {
    if (!_need_update) return;
    // We go through each time step (tick) of our period
    for (uint16_t t = 0; t < _pwm_steps; t++) {
        uint32_t mask = 0;
        // We check the state of each of the MAX_CHANNELS channels for a given time t
        for (uint8_t ch = 0; ch < MAX_CHANNELS; ch++) {
            if ((_enabled_mask & (1u << ch)) && (t < _periods[ch])) {
                mask |= (1u << ch);
            }
        }
        _bit_buffer[t] = mask;
    }
    // Fill the rest of the buffer with zeros if the period has decreased
    if (_pwm_steps < PWM_RESOLUTION) {
        memset(&_bit_buffer[_pwm_steps], 0, (PWM_RESOLUTION - _pwm_steps) * sizeof(uint32_t));
    }
    _need_update = false;
    // Update the number of transfers for the DMA channel
    dma_channel_set_trans_count(_dma_chan, _pwm_steps, false);
}
