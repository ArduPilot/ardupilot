/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  IOMCU main firmware
 */
#include <AP_HAL/AP_HAL.h>

#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include "iofirmware.h"
#include <AP_HAL_ChibiOS/RCInput.h>
#include <AP_HAL_ChibiOS/RCOutput.h>
#include "analog.h"
#include "rc.h"
#include <AP_HAL_ChibiOS/hwdef/common/watchdog.h>

extern const AP_HAL::HAL &hal;

// we build this file with optimisation to lower the interrupt
// latency. This helps reduce the chance of losing an RC input byte
// due to missing a UART interrupt
#pragma GCC optimize("O2")

static AP_IOMCU_FW iomcu;

void setup();
void loop();

#undef CH_DBG_ENABLE_STACK_CHECK
#define CH_DBG_ENABLE_STACK_CHECK FALSE

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

/*
 enable testing of IOMCU reset using safety switch
 a value of 0 means normal operation
 a value of 1 means test with watchdog
 a value of 2 means test with reboot
*/
#define IOMCU_ENABLE_RESET_TEST 0

//#define IOMCU_LOOP_TIMING_DEBUG
// enable timing GPIO pings
#ifdef IOMCU_LOOP_TIMING_DEBUG
#undef TOGGLE_PIN_DEBUG
#define TOGGLE_PIN_DEBUG(pin) do { palToggleLine(HAL_GPIO_LINE_GPIO ## pin); } while (0)
#endif

// pending events on the main thread
enum ioevents {
    IOEVENT_PWM = EVENT_MASK(1),
    IOEVENT_TX_BEGIN = EVENT_MASK(2),
    IOEVENT_TX_END = EVENT_MASK(3),
};

// see https://github.com/MaJerle/stm32-usart-uart-dma-rx-tx for a discussion of how to run
// separate tx and rx streams
static void setup_rx_dma(hal_uart_driver* uart)
{
    uart->usart->CR3 &= ~USART_CR3_DMAR;
    dmaStreamDisable(uart->dmarx);
    dmaStreamSetMemory0(uart->dmarx, &iomcu.rx_io_packet);
    dmaStreamSetTransactionSize(uart->dmarx, sizeof(iomcu.rx_io_packet));
    dmaStreamSetPeripheral(uart->dmarx, &(uart->usart->DR));
    dmaStreamSetMode(uart->dmarx, uart->dmarxmode    | STM32_DMA_CR_DIR_P2M |
                     STM32_DMA_CR_MINC | STM32_DMA_CR_TCIE);
    dmaStreamEnable(uart->dmarx);
    uart->usart->CR3 |= USART_CR3_DMAR;
}

static void setup_tx_dma(hal_uart_driver* uart)
{
    uart->usart->CR3 &= ~USART_CR3_DMAT;
    dmaStreamDisable(uart->dmatx);
    dmaStreamSetMemory0(uart->dmatx, &iomcu.tx_io_packet);
    dmaStreamSetTransactionSize(uart->dmatx, iomcu.tx_io_packet.get_size());
    // starting the UART allocates the peripheral statically, so we need to reinstate it after swapping
    dmaStreamSetPeripheral(uart->dmatx, &(uart->usart->DR));
    dmaStreamSetMode(uart->dmatx, uart->dmatxmode    | STM32_DMA_CR_DIR_M2P |
                     STM32_DMA_CR_MINC | STM32_DMA_CR_TCIE);
    // enable transmission complete interrupt
    uart->usart->SR &= ~USART_SR_TC;
    uart->usart->CR1 |= USART_CR1_TCIE;

    dmaStreamEnable(uart->dmatx);

    uart->usart->CR3 |= USART_CR3_DMAT;
}

static void dma_rx_end_cb(hal_uart_driver *uart)
{
    chSysLockFromISR();
    uart->usart->CR3 &= ~USART_CR3_DMAR;

    dmaStreamDisable(uart->dmarx);

    iomcu.process_io_packet();

    setup_rx_dma(uart);

#if AP_HAL_SHARED_DMA_ENABLED
    // indicate that a response needs to be sent
    uint32_t mask = chEvtGetAndClearEventsI(IOEVENT_TX_BEGIN);
    if (mask) {
        iomcu.reg_status.err_lock++;
    }
    // the FMU code waits 10ms for a reply so this should be easily fast enough
    chEvtSignalI(iomcu.thread_ctx, IOEVENT_TX_BEGIN);
#else
    setup_tx_dma(uart);
#endif
    chSysUnlockFromISR();
}

static void dma_tx_end_cb(hal_uart_driver *uart)
{
    // DMA stream has already been disabled at this point
    uart->usart->CR3 &= ~USART_CR3_DMAT;

    (void)uart->usart->SR;
    (void)uart->usart->DR;
    (void)uart->usart->DR;

#ifdef HAL_GPIO_LINE_GPIO108
    TOGGLE_PIN_DEBUG(108);
    TOGGLE_PIN_DEBUG(108);
#endif
#if AP_HAL_SHARED_DMA_ENABLED
    chSysLockFromISR();
    chEvtSignalI(iomcu.thread_ctx, IOEVENT_TX_END);
    chSysUnlockFromISR();
#endif
}

/* replacement for ChibiOS uart_lld_serve_interrupt() */
static void idle_rx_handler(hal_uart_driver *uart)
{
    volatile uint16_t sr;
    sr = uart->usart->SR; /* SR reset step 1.*/
    uint32_t cr1 = uart->usart->CR1;

    if (sr & (USART_SR_LBD | USART_SR_ORE |	/* overrun error - packet was too big for DMA or DMA was too slow */
              USART_SR_NE |		/* noise error - we have lost a byte due to noise */
              USART_SR_FE |
              USART_SR_PE)) {		/* framing error - start/stop bit lost or line break */

        (void)uart->usart->DR;  /* SR reset step 2 - clear ORE | FE.*/

        /* send a line break - this will abort transmission/reception on the other end */
        chSysLockFromISR();
        uart->usart->SR = ~USART_SR_LBD;
        uart->usart->CR1 = cr1 | USART_CR1_SBK;

        iomcu.reg_status.num_errors++;
        iomcu.reg_status.err_uart++;

        /* disable RX DMA */
        uart->usart->CR3 &= ~USART_CR3_DMAR;

        setup_rx_dma(uart);

        chSysUnlockFromISR();
    }

    if ((sr & USART_SR_TC) && (cr1 & USART_CR1_TCIE)) {
        /* TC interrupt cleared and disabled.*/
        uart->usart->SR &= ~USART_SR_TC;
        uart->usart->CR1 = cr1 & ~USART_CR1_TCIE;
#ifdef HAL_GPIO_LINE_GPIO105
        TOGGLE_PIN_DEBUG(105);
        TOGGLE_PIN_DEBUG(105);
#endif
        /* End of transmission, a callback is generated.*/
        dma_tx_end_cb(uart);
    }

    if ((sr & USART_SR_IDLE) && (cr1 & USART_CR1_IDLEIE)) {
        (void)uart->usart->DR;  /* SR reset step 2 - clear IDLE.*/

        /* the DMA size is the maximum packet size, but smaller packets are perfectly possible leading to 
           an IDLE ISR. The data still must be processed. */

        /* End of receive, a callback is generated.*/
        dma_rx_end_cb(uart);
    }
}

using namespace ChibiOS;

#if AP_HAL_SHARED_DMA_ENABLED
/*
 copy of uart_lld_serve_tx_end_irq() from ChibiOS hal_uart_lld
 that is re-instated upon switching the DMA channel
 */
static void uart_lld_serve_tx_end_irq(hal_uart_driver *uart, uint32_t flags)
{
    dmaStreamDisable(uart->dmatx);

    /* A callback is generated, if enabled, after a completed transfer.*/
    _uart_tx1_isr_code(uart);
}

void AP_IOMCU_FW::tx_dma_allocate(Shared_DMA *ctx)
{
    hal_uart_driver *uart = &UARTD2;
    chSysLock();
    if (uart->dmatx == nullptr) {
        uart->dmatx = dmaStreamAllocI(STM32_UART_USART2_TX_DMA_STREAM,
                                        STM32_UART_USART2_IRQ_PRIORITY,
                                        (stm32_dmaisr_t)uart_lld_serve_tx_end_irq,
                                        (void *)uart);
    }
    chSysUnlock();
}

/*
  deallocate DMA channel
 */
void AP_IOMCU_FW::tx_dma_deallocate(Shared_DMA *ctx)
{
    hal_uart_driver *uart = &UARTD2;
    chSysLock();
    if (uart->dmatx != nullptr) {
        // defensively make sure the DMA is fully shutdown before swapping
        uart->usart->CR3 &= ~USART_CR3_DMAT;
        dmaStreamDisable(uart->dmatx);
        dmaStreamSetPeripheral(uart->dmatx, nullptr);
        dmaStreamFreeI(uart->dmatx);
        uart->dmatx = nullptr;
    }
    chSysUnlock();
}
#endif // AP_HAL_SHARED_DMA_ENABLED

/*
 * UART driver configuration structure.
 */
static UARTConfig uart_cfg = {
    nullptr,
    dma_tx_end_cb,
    dma_rx_end_cb,
    nullptr,
    nullptr,            // error
    idle_rx_handler,    // global irq
    nullptr,            // idle
    1500000,      //1.5MBit
    USART_CR1_IDLEIE,
    0,
    0
};

void setup(void)
{
    hal.rcin->init();
    hal.rcout->init();
    iomcu.init();

    iomcu.calculate_fw_crc();

    uartStart(&UARTD2, &uart_cfg);
    uartStartReceive(&UARTD2, sizeof(iomcu.rx_io_packet), &iomcu.rx_io_packet);
#if AP_HAL_SHARED_DMA_ENABLED
    iomcu.tx_dma_handle->unlock();
#endif
    // disable the pieces from the UART which will get enabled later
    chSysLock();
    UARTD2.usart->CR3 &= ~USART_CR3_DMAT;
    chSysUnlock();
}

void loop(void)
{
    iomcu.update();
}

void AP_IOMCU_FW::init()
{
    // the first protocol version must be 4 to allow downgrade to
    // old NuttX based firmwares
    config.protocol_version = IOMCU_PROTOCOL_VERSION;
    config.protocol_version2 = IOMCU_PROTOCOL_VERSION2;
    config.mcuid = (*(uint32_t *)DBGMCU_BASE);
#if defined(STM32F103xB) || defined(STM32F103x8)
    if (config.mcuid == 0) {
        // Errata 2.2.2 - Debug registers cannot be read by user software
        config.mcuid = 0x20036410;  // STM32F10x (Medium Density) rev Y
    }
#endif
    config.cpuid = SCB->CPUID;

    thread_ctx = chThdGetSelfX();

#if AP_HAL_SHARED_DMA_ENABLED
    tx_dma_handle = NEW_NOTHROW ChibiOS::Shared_DMA(STM32_UART_USART2_TX_DMA_STREAM, SHARED_DMA_NONE,
                        FUNCTOR_BIND_MEMBER(&AP_IOMCU_FW::tx_dma_allocate, void, Shared_DMA *),
                        FUNCTOR_BIND_MEMBER(&AP_IOMCU_FW::tx_dma_deallocate, void, Shared_DMA *));
    tx_dma_handle->lock();
    // deallocate so that the uart initializes correctly
    tx_dma_deallocate(tx_dma_handle);
#endif

    if (palReadLine(HAL_GPIO_PIN_IO_HW_DETECT1) == 1 && palReadLine(HAL_GPIO_PIN_IO_HW_DETECT2) == 0) {
        has_heater = true;
    }

    //Set Heater pin mode
    if (heater_pwm_polarity) {
        palSetLineMode(HAL_GPIO_PIN_HEATER, PAL_MODE_OUTPUT_PUSHPULL);
    } else {
        palSetLineMode(HAL_GPIO_PIN_HEATER, PAL_MODE_OUTPUT_OPENDRAIN);
    }

    adc_init();
    rcin_serial_init();

    // power on spektrum port
    palSetLineMode(HAL_GPIO_PIN_SPEKTRUM_PWR_EN, PAL_MODE_OUTPUT_PUSHPULL);
    SPEKTRUM_POWER(1);

    // we generally do no allocations after setup completes
    reg_status.freemem = hal.util->available_memory();

    if (hal.util->was_watchdog_safety_off()) {
        hal.rcout->force_safety_off();
        reg_status.flag_safety_off = true;
    }
}


#if CH_DBG_ENABLE_STACK_CHECK == TRUE
static void stackCheck(uint16_t& mstack, uint16_t& pstack) {
    extern stkalign_t __main_stack_base__[];
    extern stkalign_t __main_stack_end__[];
    uint32_t stklimit = (uint32_t)__main_stack_end__;
    uint32_t stkbase  = (uint32_t)__main_stack_base__;
    uint32_t *crawl   = (uint32_t *)stkbase;

    while (*crawl == 0x55555555 && crawl < (uint32_t *)stklimit) {
        crawl++;
    }
    uint32_t free = (uint32_t)crawl - stkbase;
    chDbgAssert(free > 0, "mstack exhausted");
    mstack = (uint16_t)free;

    extern stkalign_t __main_thread_stack_base__[];
    extern stkalign_t __main_thread_stack_end__[];
    stklimit = (uint32_t)__main_thread_stack_end__;
    stkbase  = (uint32_t)__main_thread_stack_base__;
    crawl   = (uint32_t *)stkbase;

    while (*crawl == 0x55555555 && crawl < (uint32_t *)stklimit) {
        crawl++;
    }
    free = (uint32_t)crawl - stkbase;
    chDbgAssert(free > 0, "pstack exhausted");
    pstack = (uint16_t)free;
}
#endif /* CH_DBG_ENABLE_STACK_CHECK == TRUE */

/*
 Update loop design.

 Considerations - the F100 is quite slow and so processing time needs to be used effectively.
 The CPU time slices required by dshot are generally faster than those required for other processing.
 Dshot requires even updates at at least 1Khz and generally faster if SERVO_DSHOT_RATE is used.
 The two most time sensitive regular functions are (1) PWM updates which run at loop rate triggered from the FMU
 (and thus require efficient code page write) and (2) rcin updates which run at a fixed 1Khz cycle (a speed
 which is assumed by the rcin protocol handlers) and require efficient code read. The FMU sends code page
 requests which require a response within 10ms in order to prevent the IOMCU being considered to have failed,
 however code page requests are always initiated by the FMU and so the IOMCU only ever needs to be ready
 to read requests - writing responses are always in response to a request. Finally, PWM channels 3-4 share a DMA
 channel with UART TX and so access needs to be mediated.

 Design -
 1. requests are read using circular DMA. In other words the RX side of the UART is always ready. Once
 a request has been processed DMA is immediately set up for a new request.
 2. responses are only ever sent in response to a request. As soon as a request is received the ISR only
 ever requests that a response be sent - it never actually sends a response.
 3. The update loop waits for four different events:
    3a - a request has been received and should be processed. This does not require the TX DMA lock.
    3b - a response needs to be sent. This requires the TX DMA lock.
    3c - a response has been sent. This allows the TX DMA lock to be released.
    3d - an out of band PWM request, usually triggered by a failsafe needs to be processed.
 Since requests are processed continuously it is possible for 3b and 3c to occur simultaneously. Since the
 TX lock is already held to send the previous response, there is no need to do anything with the lock in order
 to process the next response.

 Profiling shows that sending a response takes very little time - 10s of microseconds - and so a response is sent
 if required at the beginning of the update. This means that by the end of the update there is a very high chance
 that the response will have already been sent and this is therefore checked. If the response has been sent the
 lock is released. If for some reason the response has not gone out, as soon as it does an event will be posted
 and the update loop will run again.

 This design means that on average the update loop is idle with the TX DMA channel unlocked. This maximises the
 time that dshot can run uninterrupted leading to very efficient and even output.

 Finally the update loop has a timeout which forces updates to progress even in the absence of requests from the
 FMU. Since responses will always be triggered in a timely fashion, regardlesss of the timeout, this can be
 set relatively long.

 If compiled without sharing, DMA - and thus dshot - is not used on channels 3-4, there are no locks and responses
 are always setup in the request ISR handler.
*/
void AP_IOMCU_FW::update()
{
    eventmask_t mask = chEvtWaitAnyTimeout(IOEVENT_PWM | IOEVENT_TX_END | IOEVENT_TX_BEGIN, TIME_US2I(1000));
#ifdef HAL_GPIO_LINE_GPIO107
    TOGGLE_PIN_DEBUG(107);
#endif

    iomcu.reg_status.total_ticks++;
    if (mask) {
        iomcu.reg_status.total_events++;
    }

#if AP_HAL_SHARED_DMA_ENABLED
    // See discussion above
    if ((mask & IOEVENT_TX_BEGIN) && !(mask & IOEVENT_TX_END)) {        // 3b - lock required to send response
        tx_dma_handle->lock();
    } else if (!(mask & IOEVENT_TX_BEGIN) && (mask & IOEVENT_TX_END)) { // 3c - response sent, lock can be released
        tx_dma_handle->unlock();
    }   // else 3b and 3c   - current lock required for new response

    // send a response if required
    if (mask & IOEVENT_TX_BEGIN) {
        chSysLock();
        setup_tx_dma(&UARTD2);
        chSysUnlock();
    }
#endif

    // we get the timestamp once here, and avoid fetching it
    // within the DMA callbacks
    last_ms = AP_HAL::millis();
    loop_counter++;

    if (do_reboot && (last_ms > reboot_time)) {
        hal.scheduler->reboot(true);
        while (true) {}
    }
    if ((mask & IOEVENT_PWM) ||
        (last_safety_off != reg_status.flag_safety_off)) {
        last_safety_off = reg_status.flag_safety_off;
        pwm_out_update();
    }

    uint32_t now = last_ms;
    uint32_t now_us = AP_HAL::micros();

    reg_status.timestamp_ms = last_ms;
    // output SBUS if enabled
    if ((reg_setup.features & P_SETUP_FEATURES_SBUS1_OUT) &&
        reg_status.flag_safety_off &&
        now - sbus_last_ms >= sbus_interval_ms) {
        // output a new SBUS frame
        sbus_last_ms = now;
        sbus_out_write(reg_servo.pwm, IOMCU_MAX_RC_CHANNELS);
    }
    // handle FMU failsafe
    if (now - fmu_data_received_time > 200) {
        // we are not getting input from the FMU. Fill in failsafe values at 100Hz
        if (now - last_failsafe_ms > 10) {
            fill_failsafe_pwm();
            chEvtSignal(thread_ctx, IOEVENT_PWM);
            last_failsafe_ms = now;
        }
        // turn amber on
        AMBER_SET(1);
    } else {
        last_failsafe_ms = now;
        // turn amber off
        AMBER_SET(0);
    }

    // update status page at 20Hz
    if (now - last_status_ms > 50) {
        last_status_ms = now;
        page_status_update();
    }
#ifdef HAL_WITH_BIDIR_DSHOT
    // EDT updates are semt at ~1Hz per ESC, but we want to make sure
    // that we don't delay updates unduly so sample at 5Hz
    if (now - last_telem_ms > 200) {
        last_telem_ms = now;
        telem_update();
    }
#endif
    // run fast loop functions at 1Khz
    if (now_us - last_fast_loop_us >= 1000)
    {
        last_fast_loop_us = now_us;
        heater_update();
        rcin_update();
        rcin_serial_update();
#ifdef HAL_WITH_BIDIR_DSHOT
        erpm_update();
#endif
    }

    // run remaining functions at 100Hz
    // these are all relatively expensive and take ~10ms to complete
    // so there is no way they can effectively be run faster than 100Hz
    if (now - last_slow_loop_ms > 10) {
        last_slow_loop_ms = now;
        safety_update();
        rcout_config_update();
        hal.rcout->timer_tick();
        if (dsm_bind_state) {
            dsm_bind_step();
        }
        GPIO_write();
#if CH_DBG_ENABLE_STACK_CHECK == TRUE
        stackCheck(reg_status.freemstack, reg_status.freepstack);
#endif
    }
#if AP_HAL_SHARED_DMA_ENABLED
    // check whether a response has now been sent
    mask = chEvtGetAndClearEvents(IOEVENT_TX_END);

    if (mask) {
        tx_dma_handle->unlock();
    }
#endif
#ifdef HAL_GPIO_LINE_GPIO107
    TOGGLE_PIN_DEBUG(107);
#endif
}

void AP_IOMCU_FW::pwm_out_update()
{
    memcpy(reg_servo.pwm, reg_direct_pwm.pwm, sizeof(reg_direct_pwm));
    hal.rcout->cork();
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        if (reg_status.flag_safety_off || (reg_setup.ignore_safety & (1U<<i))) {
            hal.rcout->write(i, reg_servo.pwm[i]);
        } else {
            hal.rcout->write(i, 0);
        }
    }
    hal.rcout->push();
}

void AP_IOMCU_FW::heater_update()
{
    uint32_t now = last_ms;
    if (!has_heater) {
        // use blue LED as heartbeat, run it 4x faster when override active
        if (now - last_blue_led_ms > (override_active?125:500)) {
            BLUE_TOGGLE();
            last_blue_led_ms = now;
        }
    } else if (reg_setup.heater_duty_cycle == 0 || (now - last_heater_ms > 3000UL)) {
        // turn off the heater
        HEATER_SET(!heater_pwm_polarity);
    } else {
        // we use a pseudo random sequence to dither the cycling as
        // the heater has a significant effect on the internal
        // magnetometers. The random generator dithers this so we don't get a 1Hz cycly in the magnetometer.
        // The impact on the mags is about 25 mGauss.
        bool heater_on = (get_random16() < uint32_t(reg_setup.heater_duty_cycle) * 0xFFFFU / 100U);
        HEATER_SET(heater_on? heater_pwm_polarity : !heater_pwm_polarity);
    }
}

void AP_IOMCU_FW::rcin_update()
{
    ((ChibiOS::RCInput *)hal.rcin)->_timer_tick();
    if (hal.rcin->new_input()) {
        const auto &rc = AP::RC();
        rc_input.count = hal.rcin->num_channels();
        rc_input.flags_rc_ok = true;
        hal.rcin->read(rc_input.pwm, IOMCU_MAX_RC_CHANNELS);
        rc_last_input_ms = last_ms;
        rc_input.rc_protocol = (uint16_t)rc.protocol_detected();
        rc_input.rssi = rc.get_RSSI();
        rc_input.flags_failsafe = rc.failsafe_active();
    } else if (last_ms - rc_last_input_ms > 200U) {
        rc_input.flags_rc_ok = false;
    }
    if (update_rcout_freq) {
        hal.rcout->set_freq(reg_setup.pwm_rates, reg_setup.pwm_altrate);
        update_rcout_freq = false;
    }
    if (update_default_rate) {
        hal.rcout->set_default_rate(reg_setup.pwm_defaultrate);
        update_default_rate = false;
    }

    bool old_override = override_active;

    // check for active override channel
    if (mixing.enabled &&
        mixing.rc_chan_override > 0 &&
        rc_input.flags_rc_ok &&
        mixing.rc_chan_override <= IOMCU_MAX_RC_CHANNELS) {
        override_active = (rc_input.pwm[mixing.rc_chan_override-1] >= 1750);
    } else {
        override_active = false;
    }
    if (old_override != override_active) {
        if (override_active) {
            fill_failsafe_pwm();
        }
        chEvtSignal(thread_ctx, IOEVENT_PWM);
    }
}

#ifdef HAL_WITH_BIDIR_DSHOT
void AP_IOMCU_FW::erpm_update()
{
    uint32_t now_us = AP_HAL::micros();

    if (hal.rcout->new_erpm()) {
        dshot_erpm.update_mask |= hal.rcout->read_erpm(dshot_erpm.erpm, IOMCU_MAX_TELEM_CHANNELS);
        last_erpm_us = now_us;
    } else if (now_us - last_erpm_us > ESC_RPM_DATA_TIMEOUT_US) {
        dshot_erpm.update_mask = 0;
    }
}

void AP_IOMCU_FW::telem_update()
{
    uint32_t now_ms = AP_HAL::millis();

    for (uint8_t i = 0; i < IOMCU_MAX_TELEM_CHANNELS/4; i++) {
        struct page_dshot_telem &dshot_i = dshot_telem[i];
        for (uint8_t j = 0; j < 4; j++) {
            const uint8_t esc_id = (i * 4 + j);
            if (esc_id >= IOMCU_MAX_TELEM_CHANNELS) {
                continue;
            }
            dshot_i.error_rate[j] = uint16_t(roundf(hal.rcout->get_erpm_error_rate(esc_id) * 100.0));
#if HAL_WITH_ESC_TELEM
            const volatile AP_ESC_Telem_Backend::TelemetryData& telem = esc_telem.get_telem_data(esc_id);
            // if data is stale then set to zero to avoid phantom data appearing in mavlink
            if (now_ms - telem.last_update_ms > ESC_TELEM_DATA_TIMEOUT_MS) {
                dshot_i.voltage_cvolts[j] = 0;
                dshot_i.current_camps[j] = 0;
                dshot_i.temperature_cdeg[j] = 0;
#if AP_EXTENDED_DSHOT_TELEM_V2_ENABLED
                dshot_i.edt2_status[j] = 0;
                dshot_i.edt2_stress[j] = 0;
#endif
                continue;
            }
            dshot_i.voltage_cvolts[j] = uint16_t(roundf(telem.voltage * 100));
            dshot_i.current_camps[j] = uint16_t(roundf(telem.current * 100));
            dshot_i.temperature_cdeg[j] = telem.temperature_cdeg;
#if AP_EXTENDED_DSHOT_TELEM_V2_ENABLED
            dshot_i.edt2_status[j] = uint8_t(telem.edt2_status);
            dshot_i.edt2_stress[j] = uint8_t(telem.edt2_stress);
#endif
            dshot_i.types[j] = telem.types;
#endif
        }
    }
}
#endif

void AP_IOMCU_FW::process_io_packet()
{
    iomcu.reg_status.total_pkts++;

    if (rx_io_packet.code == CODE_NOOP) {
        iomcu.reg_status.num_errors++;
        iomcu.reg_status.err_bad_opcode++;
        return;
    }

    uint8_t rx_crc = rx_io_packet.crc;
    uint8_t calc_crc;
    rx_io_packet.crc = 0;
    uint8_t pkt_size = rx_io_packet.get_size();
    if (rx_io_packet.code == CODE_READ) {
        // allow for more bandwidth efficient read packets
        calc_crc = crc_crc8((const uint8_t *)&rx_io_packet, 4);
        if (calc_crc != rx_crc) {
            calc_crc = crc_crc8((const uint8_t *)&rx_io_packet, pkt_size);
        }
    } else {
        calc_crc = crc_crc8((const uint8_t *)&rx_io_packet, pkt_size);
    }
    if (rx_crc != calc_crc || rx_io_packet.count > PKT_MAX_REGS) {
        tx_io_packet.count = 0;
        tx_io_packet.code = CODE_CORRUPT;
        tx_io_packet.crc = 0;
        tx_io_packet.page = 0;
        tx_io_packet.offset = 0;
        tx_io_packet.crc =  crc_crc8((const uint8_t *)&tx_io_packet, tx_io_packet.get_size());
        iomcu.reg_status.num_errors++;
        iomcu.reg_status.err_crc++;
        return;
    }
    switch (rx_io_packet.code) {
    case CODE_READ: {
        if (!handle_code_read()) {
            tx_io_packet.count = 0;
            tx_io_packet.code = CODE_ERROR;
            tx_io_packet.crc = 0;
            tx_io_packet.page = 0;
            tx_io_packet.offset = 0;
            tx_io_packet.crc =  crc_crc8((const uint8_t *)&tx_io_packet, tx_io_packet.get_size());
            iomcu.reg_status.num_errors++;
            iomcu.reg_status.err_read++;
        }
    }
    break;
    case CODE_WRITE: {
        if (!handle_code_write()) {
            tx_io_packet.count = 0;
            tx_io_packet.code = CODE_ERROR;
            tx_io_packet.crc = 0;
            tx_io_packet.page = 0;
            tx_io_packet.offset = 0;
            tx_io_packet.crc =  crc_crc8((const uint8_t *)&tx_io_packet, tx_io_packet.get_size());
            iomcu.reg_status.num_errors++;
            iomcu.reg_status.err_write++;
        }
    }
    break;
    default: {
        iomcu.reg_status.num_errors++;
        iomcu.reg_status.err_bad_opcode++;
        rx_io_packet.code = CODE_NOOP;
        rx_io_packet.count = 0;
        return;
    }
    break;
    }

    // prevent a spurious DMA callback from doing anything bad
    rx_io_packet.code = CODE_NOOP;
    rx_io_packet.count = 0;

    return;
}

/*
  update dynamic elements of status page
 */
void AP_IOMCU_FW::page_status_update(void)
{
    adc_sample_channels();

    if ((reg_setup.features & P_SETUP_FEATURES_SBUS1_OUT) == 0) {
        // we can only get VRSSI when sbus is disabled
        reg_status.vrssi = adc_vrssi();
    } else {
        reg_status.vrssi = 0;
    }
    reg_status.vservo = adc_vservo();
}

bool AP_IOMCU_FW::handle_code_read()
{
    uint16_t *values = nullptr;
#define COPY_PAGE(_page_name)							\
	do {									\
		values = (uint16_t *)&_page_name;				\
		tx_io_packet.count = sizeof(_page_name) / sizeof(uint16_t);	\
	} while(0);

    switch (rx_io_packet.page) {
    case PAGE_CONFIG:
        COPY_PAGE(config);
        break;
    case PAGE_SETUP:
        COPY_PAGE(reg_setup);
        break;
    case PAGE_RAW_RCIN:
        COPY_PAGE(rc_input);
        break;
#ifdef HAL_WITH_BIDIR_DSHOT
    case PAGE_RAW_DSHOT_ERPM:
        COPY_PAGE(dshot_erpm);
        break;
    case PAGE_RAW_DSHOT_TELEM_1_4:
        COPY_PAGE(dshot_telem[0]);
        break;
#if IOMCU_MAX_TELEM_CHANNELS > 4
    case PAGE_RAW_DSHOT_TELEM_5_8:
        COPY_PAGE(dshot_telem[1]);
        break;
#endif
#endif
    case PAGE_STATUS:
        COPY_PAGE(reg_status);
        break;
    case PAGE_SERVOS:
        COPY_PAGE(reg_servo);
        break;
    default:
        return false;
    }

    /* if the offset is at or beyond the end of the page, we have no data */
    if (rx_io_packet.offset + rx_io_packet.count > tx_io_packet.count) {
        return false;
    }

    /* correct the data pointer and count for the offset */
    values += rx_io_packet.offset;
    tx_io_packet.page = rx_io_packet.page;
    tx_io_packet.offset = rx_io_packet.offset;
    tx_io_packet.count -= rx_io_packet.offset;
    tx_io_packet.count = MIN(tx_io_packet.count, rx_io_packet.count);
    tx_io_packet.count = MIN(tx_io_packet.count, PKT_MAX_REGS);
    tx_io_packet.code = CODE_SUCCESS;
    memcpy(tx_io_packet.regs, values, sizeof(uint16_t)*tx_io_packet.count);
    tx_io_packet.crc = 0;
    tx_io_packet.crc =  crc_crc8((const uint8_t *)&tx_io_packet, tx_io_packet.get_size());

#ifdef HAL_WITH_BIDIR_DSHOT
    switch (rx_io_packet.page) {
    case PAGE_RAW_DSHOT_ERPM:
        memset(&dshot_erpm, 0, sizeof(dshot_erpm));
        break;
    default:
        break;
    }
#endif
    return true;
}

bool AP_IOMCU_FW::handle_code_write()
{
    switch (rx_io_packet.page) {
    case PAGE_SETUP:
        switch (rx_io_packet.offset) {
        case PAGE_REG_SETUP_ARMING:
            reg_setup.arming = rx_io_packet.regs[0];
            break;
        case PAGE_REG_SETUP_FORCE_SAFETY_OFF:
            if (rx_io_packet.regs[0] == FORCE_SAFETY_MAGIC) {
                hal.rcout->force_safety_off();
                reg_status.flag_safety_off = true;
            } else {
                return false;
            }
            break;
        case PAGE_REG_SETUP_FORCE_SAFETY_ON:
            if (rx_io_packet.regs[0] == FORCE_SAFETY_MAGIC) {
                hal.rcout->force_safety_on();
                reg_status.flag_safety_off = false;
            } else {
                return false;
            }
            break;
        case PAGE_REG_SETUP_ALTRATE:
            reg_setup.pwm_altrate = rx_io_packet.regs[0];
            update_rcout_freq = true;
            break;
        case PAGE_REG_SETUP_PWM_RATE_MASK:
            reg_setup.pwm_rates = rx_io_packet.regs[0];
            update_rcout_freq = true;
            break;
        case PAGE_REG_SETUP_DEFAULTRATE:
            if (rx_io_packet.regs[0] < 25 && reg_setup.pwm_altclock == 1) {
                rx_io_packet.regs[0] = 25;
            }

            if (rx_io_packet.regs[0] > 400 && reg_setup.pwm_altclock == 1) {
                rx_io_packet.regs[0] = 400;
            }
            reg_setup.pwm_defaultrate = rx_io_packet.regs[0];
            update_default_rate = true;
            break;
        case PAGE_REG_SETUP_DSHOT_PERIOD:
            reg_setup.dshot_period_us = rx_io_packet.regs[0];
            reg_setup.dshot_rate = rx_io_packet.regs[1];
            hal.rcout->set_dshot_period(reg_setup.dshot_period_us, reg_setup.dshot_rate);
            break;
        case PAGE_REG_SETUP_CHANNEL_MASK:
            reg_setup.channel_mask = rx_io_packet.regs[0];
            break;
        case PAGE_REG_SETUP_SBUS_RATE:
            reg_setup.sbus_rate = rx_io_packet.regs[0];
            sbus_interval_ms = MAX(1000U / reg_setup.sbus_rate,3);
            break;
        case PAGE_REG_SETUP_FEATURES:
            reg_setup.features = rx_io_packet.regs[0];
            /* disable the conflicting options with SBUS 1 */
            if (reg_setup.features & (P_SETUP_FEATURES_SBUS1_OUT)) {
                reg_setup.features &= ~(P_SETUP_FEATURES_PWM_RSSI |
                                        P_SETUP_FEATURES_ADC_RSSI |
                                        P_SETUP_FEATURES_SBUS2_OUT);

                // enable SBUS output at specified rate
                sbus_interval_ms = MAX(1000U / reg_setup.sbus_rate,3);

                // we need to release the JTAG reset pin to be used as a GPIO, otherwise we can't enable
                // or disable SBUS out
                AFIO->MAPR = AFIO_MAPR_SWJ_CFG_NOJNTRST;

                adc_disable_vrssi();
                palClearLine(HAL_GPIO_PIN_SBUS_OUT_EN);
            } else {
                adc_enable_vrssi();
                palSetLine(HAL_GPIO_PIN_SBUS_OUT_EN);
            }
            if (reg_setup.features & P_SETUP_FEATURES_HEATER) {
                has_heater = true;
            }
            break;

        case PAGE_REG_SETUP_OUTPUT_MODE:
            mode_out.mask = rx_io_packet.regs[0];
            mode_out.mode = rx_io_packet.regs[1];
            mode_out.bdmask = rx_io_packet.regs[2];
            mode_out.esc_type = rx_io_packet.regs[3];
            mode_out.reversible_mask = rx_io_packet.regs[4];
            break;

        case PAGE_REG_SETUP_HEATER_DUTY_CYCLE:
            reg_setup.heater_duty_cycle = rx_io_packet.regs[0];
            last_heater_ms = last_ms;
            break;

        case PAGE_REG_SETUP_REBOOT_BL:
            if (reg_status.flag_safety_off) {
                // don't allow reboot while armed
                return false;
            }

            // check the magic value
            if (rx_io_packet.regs[0] != REBOOT_BL_MAGIC) {
                return false;
            }
            schedule_reboot(100);
            break;

        case PAGE_REG_SETUP_IGNORE_SAFETY:
            reg_setup.ignore_safety = rx_io_packet.regs[0];
            ((ChibiOS::RCOutput *)hal.rcout)->set_safety_mask(reg_setup.ignore_safety);
            break;

        case PAGE_REG_SETUP_DSM_BIND:
            if (dsm_bind_state == 0) {
                dsm_bind_state = 1;
            }
            break;

        case PAGE_REG_SETUP_RC_PROTOCOLS: {
            if (rx_io_packet.count == 2) {
                uint32_t v;
                memcpy(&v, &rx_io_packet.regs[0], 4);
                AP::RC().set_rc_protocols(v);
            }
            break;
        }

        default:
            break;
        }
        break;

    case PAGE_DIRECT_PWM: {
        if (override_active) {
            // no input when override is active
            break;
        }
        if (rx_io_packet.count > sizeof(reg_direct_pwm.pwm)/2) {
            return false;
        }
        /* copy channel data */
        uint16_t i = 0, num_values = rx_io_packet.count;
        while ((i < IOMCU_MAX_RC_CHANNELS) && (num_values > 0)) {
            /* XXX range-check value? */
            if (rx_io_packet.regs[i] != PWM_IGNORE_THIS_CHANNEL) {
                reg_direct_pwm.pwm[i] = rx_io_packet.regs[i];
            }

            num_values--;
            i++;
        }
        fmu_data_received_time = last_ms;
        chEvtSignalI(thread_ctx, IOEVENT_PWM);
        break;
    }

    case PAGE_MIXING: { // multi-packet message
        uint16_t offset = rx_io_packet.offset, num_values = rx_io_packet.count;
        if (offset + num_values > sizeof(mixing)/2) {
            return false;
        }
        memcpy(((uint16_t *)&mixing)+offset, &rx_io_packet.regs[0], num_values*2);
        break;
    }

    case PAGE_FAILSAFE_PWM: {
        if (rx_io_packet.count != sizeof(reg_failsafe_pwm.pwm)/2) {
            return false;
        }
        memcpy((&reg_failsafe_pwm.pwm[0]), &rx_io_packet.regs[0], rx_io_packet.count*2);
        break;
    }

    case PAGE_GPIO:
        if (rx_io_packet.count != 1) {
            return false;
        }
        memcpy(&GPIO, &rx_io_packet.regs[0] + rx_io_packet.offset, sizeof(GPIO));
        break;

    case PAGE_DSHOT: {
        if (rx_io_packet.count != sizeof(dshot)/2) {
            return false;
        }
        memcpy(((uint16_t *)&dshot)+rx_io_packet.offset, &rx_io_packet.regs[0], rx_io_packet.count*2);
        if(dshot.telem_mask) {
            hal.rcout->set_telem_request_mask(dshot.telem_mask);
        }
        if (dshot.command) {
            hal.rcout->send_dshot_command(dshot.command, dshot.chan, dshot.command_timeout_ms, dshot.repeat_count, dshot.priority);
        }

        break;
    }

    default:
        break;
    }
    tx_io_packet.count = 0;
    tx_io_packet.code = CODE_SUCCESS;
    tx_io_packet.crc = 0;
    tx_io_packet.page = 0;
    tx_io_packet.offset = 0;
    tx_io_packet.crc =  crc_crc8((const uint8_t *)&tx_io_packet, tx_io_packet.get_size());
    return true;
}

void AP_IOMCU_FW::schedule_reboot(uint32_t time_ms)
{
    do_reboot = true;
    reboot_time = last_ms + time_ms;
}

void AP_IOMCU_FW::calculate_fw_crc(void)
{
#define APP_SIZE_MAX 0xf000
#define APP_LOAD_ADDRESS 0x08001000
    // compute CRC of the current firmware
    uint32_t sum = 0;

    for (unsigned p = 0; p < APP_SIZE_MAX; p += 4) {
        uint32_t bytes = *(uint32_t *)(p + APP_LOAD_ADDRESS);
        sum = crc32_small(sum, (const uint8_t *)&bytes, sizeof(bytes));
    }

    reg_setup.crc[0] = sum & 0xFFFF;
    reg_setup.crc[1] = sum >> 16;
}


/*
  update safety state
 */
void AP_IOMCU_FW::safety_update(void)
{
    uint32_t now = last_ms;
    if (now - safety_update_ms < 100) {
        // update safety at 10Hz
        return;
    }
    safety_update_ms = now;

    bool safety_pressed = palReadLine(HAL_GPIO_PIN_SAFETY_INPUT);
    if (safety_pressed) {
        if (reg_status.flag_safety_off && (reg_setup.arming & P_SETUP_ARMING_SAFETY_DISABLE_ON)) {
            safety_pressed = false;
        } else if ((!reg_status.flag_safety_off) && (reg_setup.arming & P_SETUP_ARMING_SAFETY_DISABLE_OFF)) {
            safety_pressed = false;
        }
    }
    if (safety_pressed) {
        safety_button_counter++;
    } else {
        safety_button_counter = 0;
    }
    if (safety_button_counter == 10) {
        // safety has been pressed for 1 second, change state
        reg_status.flag_safety_off = !reg_status.flag_safety_off;
        if (reg_status.flag_safety_off) {
            hal.rcout->force_safety_off();
        } else {
            hal.rcout->force_safety_on();
        }
    }
    // update the armed state
    hal.util->set_soft_armed((reg_setup.arming & P_SETUP_ARMING_FMU_ARMED) != 0);

#if IOMCU_ENABLE_RESET_TEST
    {
        // deliberate lockup of IOMCU on 5s button press, for testing
        // watchdog
        static uint32_t safety_test_counter;
        static bool should_lockup;
        if (palReadLine(HAL_GPIO_PIN_SAFETY_INPUT)) {
            safety_test_counter++;
        } else {
            safety_test_counter = 0;
        }
        if (safety_test_counter == 50) {
            should_lockup = true;
        }
        // wait for lockup for safety to be released so we don't end
        // up in the bootloader
        if (should_lockup && palReadLine(HAL_GPIO_PIN_SAFETY_INPUT) == 0) {
#if IOMCU_ENABLE_RESET_TEST == 1
            // lockup with watchdog
            while (true) {
                hal.scheduler->delay(50);
                palToggleLine(HAL_GPIO_PIN_SAFETY_LED);
            }
#else
            // hard fault to simulate power reset or software fault
            void *foo = (void*)0xE000ED38;
            typedef void (*fptr)();
            fptr gptr = (fptr) (void *) foo;
            gptr();
            while (true) {}
#endif
        }
    }
#endif // IOMCU_ENABLE_RESET_TEST

    led_counter = (led_counter+1) % 16;
    const uint16_t led_pattern = reg_status.flag_safety_off?0xFFFF:0x5500;
    palWriteLine(HAL_GPIO_PIN_SAFETY_LED, (led_pattern & (1U << led_counter))?0:1);
}

/*
  update hal.rcout mode if needed
 */
void AP_IOMCU_FW::rcout_config_update(void)
{
    // channels cannot be changed from within a lock zone
    // so needs to be done here
    if (GPIO.channel_mask != last_GPIO_channel_mask) {
        for (uint8_t i=0; i<8; i++) {
            if ((GPIO.channel_mask & (1U << i)) != 0) {
                hal.rcout->disable_ch(i);
                hal.gpio->pinMode(101+i, HAL_GPIO_OUTPUT);
            } else {
                hal.rcout->enable_ch(i);
            }
        }
        last_GPIO_channel_mask = GPIO.channel_mask;
    }

    if (last_channel_mask != reg_setup.channel_mask) {
        for (uint8_t i=0; i<IOMCU_MAX_CHANNELS; i++) {
            if (reg_setup.channel_mask & 1U << i) {
                hal.rcout->enable_ch(i);
            } else {
                hal.rcout->disable_ch(i);
            }
        }
        last_channel_mask = reg_setup.channel_mask;
        // channel enablement will affect the reported output mode
        uint32_t output_mask = 0;
        reg_status.rcout_mode = hal.rcout->get_output_mode(output_mask);
        reg_status.rcout_mask = uint8_t(0xFF & output_mask);
    }

    // see if there is anything to do, we only support setting the mode for a particular channel once
    if ((last_output_mode_mask & mode_out.mask) == mode_out.mask
        && (last_output_bdmask & mode_out.bdmask) == mode_out.bdmask
        && (last_output_reversible_mask & mode_out.reversible_mask) == mode_out.reversible_mask
        && last_output_esc_type == mode_out.esc_type) {
        return;
    }

    switch (mode_out.mode) {
    case AP_HAL::RCOutput::MODE_PWM_DSHOT150:
    case AP_HAL::RCOutput::MODE_PWM_DSHOT300:
#if defined(STM32F103xB) || defined(STM32F103x8)
    case AP_HAL::RCOutput::MODE_PWM_DSHOT600:
#endif
#ifdef HAL_WITH_BIDIR_DSHOT
        hal.rcout->set_bidir_dshot_mask(mode_out.bdmask);
#endif
        hal.rcout->set_reversible_mask(mode_out.reversible_mask);
        hal.rcout->set_dshot_esc_type(AP_HAL::RCOutput::DshotEscType(mode_out.esc_type));
        hal.rcout->set_output_mode(mode_out.mask, (AP_HAL::RCOutput::output_mode)mode_out.mode);
        // enabling dshot changes the memory allocation
        reg_status.freemem = hal.util->available_memory();
        last_output_mode_mask |= mode_out.mask;
        last_output_bdmask |= mode_out.bdmask;
        last_output_reversible_mask |= mode_out.reversible_mask;
        last_output_esc_type = mode_out.esc_type;
        break;
    case AP_HAL::RCOutput::MODE_PWM_ONESHOT:
    case AP_HAL::RCOutput::MODE_PWM_ONESHOT125:
        // setup to use a 1Hz frequency, so we only get output when we trigger
        hal.rcout->set_freq(mode_out.mask, 1);
        hal.rcout->set_output_mode(mode_out.mask, (AP_HAL::RCOutput::output_mode)mode_out.mode);
        last_output_mode_mask |= mode_out.mask;
        break;
    case AP_HAL::RCOutput::MODE_PWM_BRUSHED:
        // default to 2kHz for all channels for brushed output
        hal.rcout->set_freq(mode_out.mask, 2000);
        hal.rcout->set_esc_scaling(1000, 2000);
        hal.rcout->set_output_mode(mode_out.mask, AP_HAL::RCOutput::MODE_PWM_BRUSHED);
        hal.rcout->set_freq(mode_out.mask, reg_setup.pwm_altrate);
        last_output_mode_mask |= mode_out.mask;
        break;
    default:
        break;
    }

    uint32_t output_mask = 0;
    reg_status.rcout_mode = hal.rcout->get_output_mode(output_mask);
    reg_status.rcout_mask = uint8_t(0xFF & output_mask);
}

/*
  fill in failsafe PWM values
 */
void AP_IOMCU_FW::fill_failsafe_pwm(void)
{
    for (uint8_t i=0; i<IOMCU_MAX_RC_CHANNELS; i++) {
        if (reg_status.flag_safety_off) {
            reg_direct_pwm.pwm[i] = reg_failsafe_pwm.pwm[i];
        } else {
            reg_direct_pwm.pwm[i] = 0;
        }
    }
    if (mixing.enabled) {
        run_mixer();
    }
}

void AP_IOMCU_FW::GPIO_write()
{
    for (uint8_t i=0; i<8; i++) {
        if ((GPIO.channel_mask & (1U << i)) != 0) {
            hal.gpio->write(101+i, (GPIO.output_mask & (1U << i)) != 0);
        }
    }
}

AP_HAL_MAIN();
