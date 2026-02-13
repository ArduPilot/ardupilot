#pragma once

#include "AP_HAL_RP.h"

#if defined(HAL_RCOUT_DRIVER_ENABLED) && HAL_RCOUT_DRIVER_ENABLED == 1

#include "pwm_multi.pio.h"

class RP::RCOutput : public AP_HAL::RCOutput {
public:
    RCOutput() : _periods{0}, _dma_chan{-1}, _dma_timer{-1}, _pio{PIO_PWM_MULTI},
     _sm{0}, _pio_offset{0}, _current_freq{50}, _pwm_steps{PWM_RESOLUTION}, _enabled_mask{0}, _need_update{false}, _corked{false}
    {}
    void     init() override;
    void     set_freq(uint32_t chmask, uint16_t freq_hz) override;
    uint16_t get_freq(uint8_t ch) override;
    void     enable_ch(uint8_t ch) override;
    void     disable_ch(uint8_t ch) override;
    void     write(uint8_t ch, uint16_t period_us) override;
    uint16_t read(uint8_t ch) override;
    void     read(uint16_t* period_us, uint8_t len) override;
    void     cork(void) override { _corked = true; }
    void     push(void) override;

private:
    void update_bit_buffer();
    void handle_dma_irq();

    static void handle_dma_irq(void *obj);
    static void dma_handler();

    static const uint8_t MAX_CHANNELS = RCOUT_MAX_CHANNELS;
    static const uint16_t PWM_RESOLUTION = RCOUT_PWM_RESOLUTION; // Resolution (e.g. 1 tick = 1 µs)
    
    uint16_t _periods[MAX_CHANNELS];
    
    int _dma_chan;
    int _dma_timer;
    PIO _pio;
    uint _sm;
    uint _pio_offset;

    // Current refresh rate (Hz)
    uint16_t _current_freq;
    // Number of steps in the current period (depends on frequency)
    uint16_t _pwm_steps;
    // Bitmask of allowed channels (only enabled channels generate a signal)
    uint32_t _enabled_mask;
    // A flag indicating whether _bit_buffer recalculation is necessary
    bool _need_update;
    bool _corked;

    // DMA buffer: 2000 steps of 32 bits (each bit is a pin state)
    //static uint32_t _bit_buffer[2][PWM_RESOLUTION]; // Two buffers
    //static uint8_t _write_idx;                      // Write buffer index
    //static const uint32_t* _buffer_to_dma_ptr;
    struct DMA_ControlBlock {
        uint32_t count;
        const uint32_t *addr;
    };

    static uint32_t _bit_buffer[2][RCOUT_PWM_RESOLUTION] __attribute__((aligned(16)));
    static uint8_t _write_idx;
    static struct DMA_ControlBlock _dma_blocks[2] __attribute__((aligned(8))); // Two buffers

    static volatile uint8_t _dma_busy_idx;
    static RCOutput* _instance;
};
#endif
