#include "RCOutput.h"
#include <AP_Math/AP_Math.h>

#if defined(HAL_RCOUT_DRIVER_ENABLED) && HAL_RCOUT_DRIVER_ENABLED == 1

#include "pwm_multi.pio.h"
//#include <pico-sdk_build/pwm_multi.pio.h>
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/pio.h"
#include "hardware/sync.h"
#include "pico/multicore.h"
#include "pico/platform.h"

using namespace RP;

extern const AP_HAL::HAL& hal;

uint32_t RCOutput::_bit_buffer[2][RCOUT_PWM_RESOLUTION] __attribute__((aligned(16)));
uint8_t RCOutput::_write_idx = 0;
struct RCOutput::DMA_ControlBlock RCOutput::_dma_blocks[2] __attribute__((aligned(8)));

RCOutput* RCOutput::_instance = nullptr;
volatile uint8_t RCOutput::_dma_busy_idx = 0;

//const uint32_t* RP::RCOutput::_buffer_to_dma_ptr = nullptr;
//uint8_t RP::RCOutput::_write_idx = 0;
//uint32_t RP::RCOutput::_bit_buffer[2][PWM_RESOLUTION];

//static const uint32_t* _buffer_to_dma_ptr = nullptr;
//static uint8_t __attribute__((section(".uninitialized_data"))) _write_idx = 0;
//static uint32_t _bit_buffer[2][RCOUT_PWM_RESOLUTION];

void RCOutput::init() {
    _instance = this;

    // 1. PIO: Програма та SM
    _pio_offset = pio_add_program(_pio, &pwm_multi_program);
    _sm = pio_claim_unused_sm(_pio, true);

    // 2. Налаштування GPIO (те, що я помилково прибирав)
    for (uint i = 0; i < MAX_CHANNELS; i++) {
        pio_gpio_init(_pio, RCOUT_GPIO_BASE + i);
    }
    pio_sm_set_consecutive_pindirs(_pio, _sm, RCOUT_GPIO_BASE, MAX_CHANNELS, true);

    // 3. Конфігурація PIO State Machine
    pio_sm_config c = pwm_multi_program_get_default_config(_pio_offset);
    sm_config_set_out_pins(&c, RCOUT_GPIO_BASE, MAX_CHANNELS);
    sm_config_set_out_shift(&c, true, false, 32);
    
    // Розрахунок div для 400 Гц (3 такти циклу + 1 такт mov)
    float div = (float)clock_get_hz(clk_sys) / (400.0f * (3.0f * RCOUT_PWM_RESOLUTION + 1.0f));
    sm_config_set_clkdiv(&c, div);
    pio_sm_init(_pio, _sm, _pio_offset, &c);

    // 4. Налаштування ДВОХ каналів DMA для апаратного Ping-Pong
    _dma_chan = dma_claim_unused_channel(true);  // Канал A (Ping)
    _dma_timer = dma_claim_unused_channel(true); // Канал B (Pong) - перейменуємо логічно

    // Конфігурація Каналу A
    dma_channel_config cfg_a = dma_channel_get_default_config(_dma_chan);
    channel_config_set_transfer_data_size(&cfg_a, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg_a, true);
    channel_config_set_dreq(&cfg_a, pio_get_dreq(_pio, _sm, true));
    channel_config_set_chain_to(&cfg_a, _dma_timer); // Після А стартує Б

    dma_channel_configure(_dma_chan, &cfg_a, &_pio->txf[_sm], _bit_buffer[0], RCOUT_PWM_RESOLUTION, false);

    // Конфігурація Каналу B
    dma_channel_config cfg_b = dma_channel_get_default_config(_dma_timer);
    channel_config_set_transfer_data_size(&cfg_b, DMA_SIZE_32);
    channel_config_set_read_increment(&cfg_b, true);
    channel_config_set_dreq(&cfg_b, pio_get_dreq(_pio, _sm, true));
    channel_config_set_chain_to(&cfg_b, _dma_chan); // Після Б стартує А

    dma_channel_configure(_dma_timer, &cfg_b, &_pio->txf[_sm], _bit_buffer[1], RCOUT_PWM_RESOLUTION, false);

    // 5. Переривання на обидва канали
    dma_channel_set_irq0_enabled(_dma_chan, true);
    dma_channel_set_irq0_enabled(_dma_timer, true);
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    irq_set_enabled(DMA_IRQ_0, true);

    // 6. Підготовка PIO ISR (лічильник періоду)
    pio_sm_set_enabled(_pio, _sm, false);
    pio_sm_put_blocking(_pio, _sm, RCOUT_PWM_RESOLUTION - 1);
    pio_sm_exec(_pio, _sm, pio_encode_pull(false, false));
    pio_sm_exec(_pio, _sm, pio_encode_mov(pio_isr, pio_osr));
    
    // 7. СТАРТ
    pio_sm_set_enabled(_pio, _sm, true);
    dma_channel_start(_dma_chan);

    DEV_PRINTF("init: 2-channel ping-pong started, res=%u\n", RCOUT_PWM_RESOLUTION);
}


#if 0
void RCOutput::init() {
    _instance = this;

    _pio_offset = pio_add_program(_pio, &pwm_multi_program);
    _sm = pio_claim_unused_sm(_pio, true);

    // Pin configuration
    for (uint i = 0; i < MAX_CHANNELS; i++) {
        pio_gpio_init(_pio, RCOUT_GPIO_BASE + i);
    }
    pio_sm_set_consecutive_pindirs(_pio, _sm, RCOUT_GPIO_BASE, MAX_CHANNELS, true);

    // Конфігурація PIO
    pio_sm_config c = pwm_multi_program_get_default_config(_pio_offset);
    sm_config_set_out_pins(&c, RCOUT_GPIO_BASE, MAX_CHANNELS);
    sm_config_set_out_shift(&c, true, false, 32);
    
    // Розрахунок div (враховуємо 3 такти + 1 такт на запуск циклу)
    float div = (float)clock_get_hz(clk_sys) / (400.0f * (3.0f * RCOUT_PWM_RESOLUTION + 1.0f));
    sm_config_set_clkdiv(&c, div);
    pio_sm_init(_pio, _sm, _pio_offset, &c);

    // Ініціалізація Контрольних Блоків (Ping-Pong)
    _dma_blocks[0].count = RCOUT_PWM_RESOLUTION;
    _dma_blocks[0].addr = _bit_buffer[0];
    _dma_blocks[1].count = RCOUT_PWM_RESOLUTION;
    _dma_blocks[1].addr = _bit_buffer[1];

    _dma_chan = dma_claim_unused_channel(true);
    _dma_timer = dma_claim_unused_channel(true); // Використовуємо як Ctrl Channel

    // 1. DATA CHANNEL: передає маски в PIO
    dma_channel_config data_cfg = dma_channel_get_default_config(_dma_chan);
    channel_config_set_transfer_data_size(&data_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&data_cfg, true);
    channel_config_set_dreq(&data_cfg, pio_get_dreq(_pio, _sm, true));
    // Коли закінчив буфер -> штовхає CTRL CHANNEL
    channel_config_set_chain_to(&data_cfg, _dma_timer);

    dma_channel_configure(_dma_chan, &data_cfg, &_pio->txf[_sm], NULL, 0, false);

    // 2. CTRL CHANNEL: переписує READ_ADDR та TRANS_COUNT основного каналу
    dma_channel_config ctrl_cfg = dma_channel_get_default_config(_dma_timer);
    channel_config_set_transfer_data_size(&ctrl_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&ctrl_cfg, true);
    channel_config_set_write_increment(&ctrl_cfg, true);
    channel_config_set_ring(&ctrl_cfg, false, 4); 

    dma_channel_configure(
        _dma_timer, &ctrl_cfg,
        &dma_hw->ch[_dma_chan].al2_transfer_count, // Пишемо в Alias 3 (Count + Addr + Trigger)
        &_dma_blocks[0],
        2,    // Переносимо 2 слова (Addr та Count)
        false
    );

    // ЗАПУСК СИСТЕМИ
    pio_sm_set_enabled(_pio, _sm, false);
    // Завантажуємо 2499 в ISR (для jmp y--)
    pio_sm_put_blocking(_pio, _sm, RCOUT_PWM_RESOLUTION - 1);
    pio_sm_exec(_pio, _sm, pio_encode_pull(false, false));
    pio_sm_exec(_pio, _sm, pio_encode_mov(pio_isr, pio_osr));

    pio_sm_set_enabled(_pio, _sm, true);

    // 1. Дозволяємо каналу даних генерувати переривання IRQ0 після завершення блоку
    dma_channel_set_irq0_enabled(_dma_chan, true);

    // 2. Реєструємо обробник у системі
    irq_set_exclusive_handler(DMA_IRQ_0, dma_handler);
    
    // 3. Увімкнення переривання в контролері (NVIC)
    irq_set_enabled(DMA_IRQ_0, true);

    //for (int i=0; i<4; i++) {
    //    pio_sm_put(_pio, _sm, 0); // Закидаємо кілька нулів у FIFO
    //}
    // Стартуємо через CTRL канал — він сам завантажить перший блок у DATA канал
    dma_channel_start(_dma_timer);
}
#endif

void RCOutput::handle_dma_irq() {
    uint32_t ints = dma_hw->ints0;
    if (ints & (1u << _dma_chan)) {
        dma_hw->ints0 = 1u << _dma_chan;
        _dma_busy_idx = 0; // DMA тільки що закінчив 0-й буфер
    } 
    if (ints & (1u << _dma_timer)) {
        dma_hw->ints0 = 1u << _dma_timer;
        _dma_busy_idx = 1; // DMA тільки що закінчив 1-й буфер
    }
}

#if 0
void RCOutput::handle_dma_irq() {
    dma_hw->ints0 = 1u << _dma_chan; // Тепер тут є доступ до закритих членів
    _dma_busy_idx = 1 - _dma_busy_idx;
}
#endif

void RCOutput::handle_dma_irq(void *obj) {
    if (obj) {
        ((RCOutput *)obj)->handle_dma_irq();
    }
}

void RCOutput::dma_handler() {
    handle_dma_irq(_instance);
}

#if 0
void RCOutput::init() {
    __dmb();
    _write_idx = 0;
    _buffer_to_dma_ptr = (const uint32_t*)_bit_buffer[_write_idx];
    memset(_bit_buffer, 0, sizeof(_bit_buffer));

    //update_bit_buffer();

    for (uint8_t i = 0; i < MAX_CHANNELS; i++) {
        _periods[i] = 1000; 
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
    pio_sm_set_consecutive_pindirs(_pio, _sm, RCOUT_GPIO_BASE, MAX_CHANNELS, true);

    // State Machine Configuration
    pio_sm_config c = pwm_multi_program_get_default_config(_pio_offset);
    sm_config_set_out_pins(&c, RCOUT_GPIO_BASE, MAX_CHANNELS);

    // FIFO settings: autopull OFF
    sm_config_set_out_shift(&c, true, false, 32);

    float div = (float)clock_get_hz(clk_sys) / (1000000.0f * 2.0f);
    sm_config_set_clkdiv(&c, div);

    pio_sm_init(_pio, _sm, _pio_offset, &c);

    _dma_chan = dma_claim_unused_channel(true);
    _dma_timer = dma_claim_unused_channel(true);

    // Tunning the main channel (Data Channel)
    dma_channel_config dma_cfg = dma_channel_get_default_config(_dma_chan);
    channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_cfg, true);
    channel_config_set_write_increment(&dma_cfg, false);
    channel_config_set_dreq(&dma_cfg, pio_get_dreq(_pio, _sm, true));
    // After the buffer is full, we call the timer channel
    channel_config_set_chain_to(&dma_cfg, _dma_timer); 

    // Tunning the timer channel (Reload Channel)
    dma_channel_config timer_cfg = dma_channel_get_default_config(_dma_timer);
    channel_config_set_transfer_data_size(&timer_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&timer_cfg, false);
    channel_config_set_write_increment(&timer_cfg, false);
    // The timer channel returns the chain back to the main one
    channel_config_set_chain_to(&timer_cfg, _dma_chan);

    // Configure the timer: it writes the address buffer_start_ptr to the READ_ADDR register of the main channel
    dma_channel_configure(
        _dma_timer, &timer_cfg,
        &dma_hw->ch[_dma_chan].al1_read_addr, // Trigger register to reload
        &_buffer_to_dma_ptr,
        1,
        true
    );

    uint32_t *data_ptr = _bit_buffer[_write_idx];

    _pwm_steps = RCOUT_PWM_RESOLUTION;
    // Configure and launch the main channel
    dma_channel_configure(
        _dma_chan, &dma_cfg,
        &_pio->txf[_sm],
        data_ptr,
        _pwm_steps,
        false
    );

    _write_idx = 1 - _write_idx;

    pio_sm_set_enabled(_pio, _sm, false);
    pio_sm_clear_fifos(_pio, _sm);

    // Запускаємо DMA, який буде крутитися вічно
    dma_channel_start(_dma_chan);
    pio_sm_set_enabled(_pio, _sm, true);
    /*
    set_freq(0, 400);
    pio_sm_set_enabled(_pio, _sm, true);
    pio_sm_put_blocking(_pio, _sm, 999);
    dma_channel_start(_dma_chan);
    */
    DEV_PRINTF("init: _pwm_steps=%u, _current_freq=%u\n", _pwm_steps, _current_freq);
}
#endif

#if 0
void RCOutput::init() {
    __dmb();
    _write_idx = 0;
    _buffer_to_dma_ptr = (const uint32_t*)_bit_buffer[_write_idx];
    memset(_bit_buffer, 0, sizeof(_bit_buffer));

    update_bit_buffer();

    for (uint8_t i = 0; i < MAX_CHANNELS; i++) {
        _periods[i] = 1000;
    }
    _enabled_mask = 0;
    _need_update = true;

    // PIO settings
    _pio_offset = pio_add_program(_pio, &pwm_multi_program);
    _sm = pio_claim_unused_sm(_pio, true);

    // State Machine Configuration
    pio_sm_config c = pwm_multi_program_get_default_config(_pio_offset);
    sm_config_set_out_pins(&c, RCOUT_GPIO_BASE, MAX_CHANNELS);
    // FIFO settings: autopull OFF
    sm_config_set_out_shift(&c, true, false, 32);

    float div = (float)clock_get_hz(clk_sys) / (1000000.0f * 3.0f);
    sm_config_set_clkdiv(&c, div);
    DEV_PRINTF("init REAL DIV: %f\n", div);

    pio_sm_init(_pio, _sm, _pio_offset, &c);

    // Pin configuration
    for (uint i = 0; i < MAX_CHANNELS; i++) {
        pio_gpio_init(_pio, RCOUT_GPIO_BASE + i);
    }
    pio_sm_set_consecutive_pindirs(_pio, _sm, RCOUT_GPIO_BASE, MAX_CHANNELS, true);

    _dma_chan = dma_claim_unused_channel(true);
    _dma_timer = dma_claim_unused_channel(true);

    // Tunning the main channel (Data Channel)
    dma_channel_config dma_cfg = dma_channel_get_default_config(_dma_chan);
    channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&dma_cfg, true);
    channel_config_set_write_increment(&dma_cfg, false);
    channel_config_set_dreq(&dma_cfg, pio_get_dreq(_pio, _sm, true));
    // After the buffer is full, we call the timer channel
    channel_config_set_chain_to(&dma_cfg, _dma_timer); 

    // Tunning the timer channel (Reload Channel)
    dma_channel_config timer_cfg = dma_channel_get_default_config(_dma_timer);
    channel_config_set_transfer_data_size(&timer_cfg, DMA_SIZE_32);
    channel_config_set_read_increment(&timer_cfg, false);
    channel_config_set_write_increment(&timer_cfg, false);
    // The timer channel returns the chain back to the main one
    channel_config_set_chain_to(&timer_cfg, _dma_chan);

    // Configure the timer: it writes the address buffer_start_ptr to the READ_ADDR register of the main channel
    dma_channel_configure(
        _dma_timer, &timer_cfg,
        &dma_hw->ch[_dma_chan].al1_read_addr, // Trigger register to reload
        &_buffer_to_dma_ptr,
        1,
        false
    );

    uint32_t *data_ptr = _bit_buffer[_write_idx];

    _pwm_steps = RCOUT_PWM_RESOLUTION;
    // Configure and launch the main channel
    dma_channel_configure(
        _dma_chan, &dma_cfg,
        &_pio->txf[_sm],
        data_ptr,
        _pwm_steps,
        false
    );

    set_freq(0, 400);

    pio_sm_restart(_pio, _sm);
    pio_sm_clkdiv_restart(_pio, _sm);
    pio_sm_clear_fifos(_pio, _sm);

    pio_sm_set_enabled(_pio, _sm, false);
    pio_sm_put_blocking(_pio, _sm, _pwm_steps - 1);
    pio_sm_exec(_pio, _sm, pio_encode_pull(false, false));
    pio_sm_exec(_pio, _sm, pio_encode_mov(pio_isr, pio_osr));
    pio_sm_set_enabled(_pio, _sm, true);

    dma_channel_start(_dma_chan);
    _write_idx = 1 - _write_idx;
    DEV_PRINTF("init: _pwm_steps=%u, _current_freq=%u, _write_idx=%u\n", 
               _pwm_steps, _current_freq, _write_idx);
}
#endif

void RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz) {
    if (freq_hz == 0) return;
    _current_freq = freq_hz;

    float div = (float)clock_get_hz(clk_sys) / ((float)freq_hz * 3.0f * (float)RCOUT_PWM_RESOLUTION);

    pio_sm_set_clkdiv(_pio, _sm, div);
}

#if 0
void RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz) {
    if (freq_hz == 0) return;
    // Since we are using one DMA buffer for all 32 channels,
    // the frequency freq_hz will be global for the entire PIO module.
    _current_freq = freq_hz;
    uint32_t clock_hz = clock_get_hz(clk_sys);

    // Set PIO so that 1 iteration (3 clocks) lasts exactly 1 µs
    // For 150MHz div will be 50.0 (150 / (1 * 3))
    float div = (float)clock_hz / (1000000.0f * 3.0f);
    pio_sm_set_clkdiv(_pio, _sm, div);

    _pwm_steps = 1000000 / freq_hz;

    // CRITICAL: Update ISR in PIO so that cycle Y matches DMA length
    pio_sm_set_enabled(_pio, _sm, false);
    pio_sm_put_blocking(_pio, _sm, _pwm_steps - 1); // Y counter

    // Restart DMA (Critical for changing period length)
    // If we just change _pwm_steps, DMA will continue to spin the old cycle
    dma_channel_abort(_dma_chan);
    dma_channel_transfer_from_buffer_now(_dma_chan, _bit_buffer[_write_idx], _pwm_steps);

    pio_sm_set_enabled(_pio, _sm, true);
    _need_update = true;
}
#endif
#if 0
void RCOutput::set_freq(uint32_t chmask, uint16_t freq_hz) {
    if (freq_hz == 0) return;
    // Since we are using one DMA buffer for all 32 channels,
    // the frequency freq_hz will be global for the entire PIO module.
    _current_freq = freq_hz;
    uint32_t clock_hz = clock_get_hz(clk_sys);

    // Set PIO so that 1 iteration (3 clocks) lasts exactly 1 µs
    // For 150MHz div will be 50.0 (150 / (1 * 3))
    float div = (float)clock_hz / (1000000.0f * 3.0f);

    pio_sm_set_clkdiv(_pio, _sm, div);
    DEV_PRINTF("set_freq REAL DIV: %f\n", div);

    _pwm_steps = 1000000 / freq_hz;
    DEV_PRINTF("DEBUG: freq_hz=%u, _pwm_steps=%u\n", freq_hz, _pwm_steps);

    // CRITICAL: Update ISR in PIO so that cycle Y matches DMA length
    pio_sm_set_enabled(_pio, _sm, false);
    pio_sm_put_blocking(_pio, _sm, _pwm_steps - 1); // Y counter
    pio_sm_exec(_pio, _sm, pio_encode_pull(false, false)); // pull to OSR
    pio_sm_exec(_pio, _sm, pio_encode_out(pio_isr, 32));   // move to ISR

    // Restart DMA (Critical for changing period length)
    // If we just change _pwm_steps, DMA will continue to spin the old cycle
    dma_channel_abort(_dma_chan);
    dma_channel_transfer_from_buffer_now(_dma_chan, _bit_buffer[_write_idx], _pwm_steps);

    pio_sm_set_enabled(_pio, _sm, true);
    _need_update = true;
}
#endif

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
    _need_update = true;
    if (!_corked) push();
}

void RCOutput::push() {
    if (!_need_update || _corked) return;

    // ВАЖЛИВО: Ми заповнюємо той буфер, який НЕ зайнятий DMA
    // dma_busy_idx оновлюється в перериванні автоматично
    _write_idx = 1 - _dma_busy_idx;

    update_bit_buffer(); 
    
    _need_update = false;
}

#if 0
void RCOutput::push() {
    if (!_need_update || _corked) return;
    static uint32_t push_count = 0;
    push_count++;

    if (push_count % 33 == 0) {
        //DEV_PRINTF("DEBUG push #%lu: START, _write_idx=%d\n", push_count, _write_idx);
    }

    uint32_t write_idx_before = _write_idx;
    
    //update_bit_buffer(); 
    if (push_count % 33 == 0) {
        //DEV_PRINTF("DEBUG push #%lu: after update, _write_idx=%d\n", push_count, _write_idx);
    }
    __dmb();
    _buffer_to_dma_ptr = _bit_buffer[_write_idx];
    __dmb();

    _write_idx = 1 - _write_idx;
    uint32_t write_idx_after = _write_idx;
    _need_update = false;
    if (push_count % 33 == 0) {
        DEV_PRINTF("DEBUG push #%lu: BEFORE=%lu, AFTER=%lu, buffer_ptr=0x%lx\n", 
               push_count, write_idx_before, write_idx_after, 
               (uint32_t)_buffer_to_dma_ptr);
    }
}
#endif

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
#if 1
void RCOutput::update_bit_buffer() {
    uint32_t* target_buffer = _bit_buffer[_write_idx];
    //memset(target_buffer, 0, _pwm_steps * 4);

    for (uint8_t ch = 0; ch < MAX_CHANNELS; ch++) {
        if (!(_enabled_mask & (1u << ch))) continue;
        uint16_t length = _periods[ch];
        if (length > _pwm_steps) length = _pwm_steps;
        for (uint16_t t = 0; t < length; t++) {
            if (t >= RCOUT_PWM_RESOLUTION) break;
            target_buffer[t] |= (1u << ch);
        }
    }
    __dmb();
}
#endif

#if 0
void RCOutput::update_bit_buffer() {
    uint32_t temp_buffer[RCOUT_PWM_RESOLUTION];
    memset(temp_buffer, 0, _pwm_steps * sizeof(uint32_t));

    for (uint8_t ch = 0; ch < MAX_CHANNELS; ch++) {
        if (!(_enabled_mask & (1u << ch))) continue;
        uint16_t length = _periods[ch];
        if (length > _pwm_steps) length = _pwm_steps;
        for (uint16_t t = 0; t < length; t++) {
            temp_buffer[t] |= (1u << ch);
        }
    }

    __dmb();
    memcpy(_bit_buffer[_write_idx], temp_buffer, _pwm_steps * sizeof(uint32_t));
    __dmb();
}
#endif
#endif // HAL_RCOUT_DRIVER_ENABLED
