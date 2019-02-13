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
#include "hal.h"
#include <AP_HAL_ChibiOS/RCInput.h>
#include <AP_HAL_ChibiOS/RCOutput.h>
#include "analog.h"
#include "rc.h"

extern const AP_HAL::HAL &hal;

// we build this file with optimisation to lower the interrupt
// latency. This helps reduce the chance of losing an RC input byte
// due to missing a UART interrupt
#pragma GCC optimize("O3")

static AP_IOMCU_FW iomcu;

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// pending events on the main thread
enum ioevents {
    IOEVENT_PWM=1,
};

static struct {
    uint32_t num_code_read, num_bad_crc, num_write_pkt, num_unknown_pkt;
    uint32_t num_idle_rx, num_dma_complete_rx, num_total_rx, num_rx_error;
} stats;

static void dma_rx_end_cb(UARTDriver *uart)
{
    osalSysLockFromISR();
    uart->usart->CR3 &= ~(USART_CR3_DMAT | USART_CR3_DMAR);

    (void)uart->usart->SR;
    (void)uart->usart->DR;
    (void)uart->usart->DR;
    dmaStreamDisable(uart->dmarx);
    dmaStreamDisable(uart->dmatx);

    iomcu.process_io_packet();
    stats.num_total_rx++;
    stats.num_dma_complete_rx = stats.num_total_rx - stats.num_idle_rx;

    dmaStreamSetMemory0(uart->dmarx, &iomcu.rx_io_packet);
    dmaStreamSetTransactionSize(uart->dmarx, sizeof(iomcu.rx_io_packet));
    dmaStreamSetMode(uart->dmarx, uart->dmamode    | STM32_DMA_CR_DIR_P2M |
                     STM32_DMA_CR_MINC | STM32_DMA_CR_TCIE);
    dmaStreamEnable(uart->dmarx);
    uart->usart->CR3 |= USART_CR3_DMAR;

    dmaStreamSetMemory0(uart->dmatx, &iomcu.tx_io_packet);
    dmaStreamSetTransactionSize(uart->dmatx, iomcu.tx_io_packet.get_size());
    dmaStreamSetMode(uart->dmatx, uart->dmamode    | STM32_DMA_CR_DIR_M2P |
                     STM32_DMA_CR_MINC | STM32_DMA_CR_TCIE);
    dmaStreamEnable(uart->dmatx);
    uart->usart->CR3 |= USART_CR3_DMAT;
    osalSysUnlockFromISR();
}

static void idle_rx_handler(UARTDriver *uart)
{
    volatile uint16_t sr = uart->usart->SR;

    if (sr & (USART_SR_LBD | USART_SR_ORE |	/* overrun error - packet was too big for DMA or DMA was too slow */
              USART_SR_NE |		/* noise error - we have lost a byte due to noise */
              USART_SR_FE |
              USART_SR_PE)) {		/* framing error - start/stop bit lost or line break */
        /* send a line break - this will abort transmission/reception on the other end */
        osalSysLockFromISR();
        uart->usart->SR = ~USART_SR_LBD;
        uart->usart->CR1 |= USART_CR1_SBK;
        stats.num_rx_error++;
        uart->usart->CR3 &= ~(USART_CR3_DMAT | USART_CR3_DMAR);
        (void)uart->usart->SR;
        (void)uart->usart->DR;
        (void)uart->usart->DR;
        dmaStreamDisable(uart->dmarx);
        dmaStreamDisable(uart->dmatx);

        dmaStreamSetMemory0(uart->dmarx, &iomcu.rx_io_packet);
        dmaStreamSetTransactionSize(uart->dmarx, sizeof(iomcu.rx_io_packet));
        dmaStreamSetMode(uart->dmarx, uart->dmamode    | STM32_DMA_CR_DIR_P2M |
                         STM32_DMA_CR_MINC | STM32_DMA_CR_TCIE);
        dmaStreamEnable(uart->dmarx);
        uart->usart->CR3 |= USART_CR3_DMAR;
        osalSysUnlockFromISR();
        return;
    }

    if (sr & USART_SR_IDLE) {
        dma_rx_end_cb(uart);
        stats.num_idle_rx++;
    }
}

/*
 * UART driver configuration structure.
 */
static UARTConfig uart_cfg = {
    nullptr,
    nullptr,
    dma_rx_end_cb,
    nullptr,
    nullptr,
    idle_rx_handler,
    1500000,      //1.5MBit
    USART_CR1_IDLEIE,
    0,
    0
};

void setup(void)
{
    hal.rcin->init();
    hal.rcout->init();

    for (uint8_t i = 0; i< 14; i++) {
        hal.rcout->enable_ch(i);
    }

    iomcu.init();

    iomcu.calculate_fw_crc();
    uartStart(&UARTD2, &uart_cfg);
    uartStartReceive(&UARTD2, sizeof(iomcu.rx_io_packet), &iomcu.rx_io_packet);
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

    thread_ctx = chThdGetSelfX();

    if (palReadLine(HAL_GPIO_PIN_IO_HW_DETECT1) == 1 && palReadLine(HAL_GPIO_PIN_IO_HW_DETECT2) == 0) {
        has_heater = true;
    }

    adc_init();
    rcin_serial_init();

    // power on spektrum port
    palSetLineMode(HAL_GPIO_PIN_SPEKTRUM_PWR_EN, PAL_MODE_OUTPUT_PUSHPULL);
    SPEKTRUM_POWER(1);

    // we do no allocations after setup completes
    reg_status.freemem = hal.util->available_memory();
}


void AP_IOMCU_FW::update()
{
    // we are not running any other threads, so we can use an
    // immediate timeout here for lowest latency
    eventmask_t mask = chEvtWaitAnyTimeout(~0, TIME_IMMEDIATE);

    // we get the timestamp once here, and avoid fetching it
    // within the DMA callbacks
    last_ms = AP_HAL::millis();
    loop_counter++;

    if (do_reboot && (last_ms > reboot_time)) {
        hal.scheduler->reboot(true);
        while (true) {}
    }

    if ((mask & EVENT_MASK(IOEVENT_PWM)) ||
        (last_safety_off != reg_status.flag_safety_off)) {
        last_safety_off = reg_status.flag_safety_off;
        pwm_out_update();
    }

    uint32_t now = last_ms;

    // output SBUS if enabled
    if ((reg_setup.features & P_SETUP_FEATURES_SBUS1_OUT) &&
        reg_status.flag_safety_off &&
        now - sbus_last_ms >= sbus_interval_ms) {
        // output a new SBUS frame
        sbus_last_ms = now;
        sbus_out_write(reg_servo.pwm, IOMCU_MAX_CHANNELS);
    }

    // handle FMU failsafe
    if (now - fmu_data_received_time > 200) {
        // we are not getting input from the FMU. Fill in failsafe values at 100Hz
        if (now - last_failsafe_ms > 10) {
            fill_failsafe_pwm();
            chEvtSignal(thread_ctx, EVENT_MASK(IOEVENT_PWM));
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

    // run remaining functions at 1kHz
    if (now != last_loop_ms) {
        last_loop_ms = now;
        heater_update();
        rcin_update();
        safety_update();
        rcout_mode_update();
        rcin_serial_update();
        hal.rcout->timer_tick();
        if (dsm_bind_state) {
            dsm_bind_step();
        }
    }
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
        HEATER_SET(0);
    } else {
        uint8_t cycle = ((now / 10UL) % 100U);
        HEATER_SET(!(cycle >= reg_setup.heater_duty_cycle));
    }
}

void AP_IOMCU_FW::rcin_update()
{
    ((ChibiOS::RCInput *)hal.rcin)->_timer_tick();
    if (hal.rcin->new_input()) {
        rc_input.count = hal.rcin->num_channels();
        rc_input.flags_rc_ok = true;
        for (uint8_t i = 0; i < IOMCU_MAX_CHANNELS; i++) {
            rc_input.pwm[i] = hal.rcin->read(i);
        }
        rc_input.last_input_ms = last_ms;
        rc_input.data = (uint16_t)rcprotocol->protocol_detected();
    } else if (last_ms - rc_input.last_input_ms > 200U) {
        rc_input.flags_rc_ok = false;
    }
    if (update_rcout_freq) {
        hal.rcout->set_freq(reg_setup.pwm_rates, reg_setup.pwm_altrate);
        update_rcout_freq = false;
    }
    if (update_default_rate) {
        hal.rcout->set_default_rate(reg_setup.pwm_defaultrate);
    }

    bool old_override = override_active;

    // check for active override channel
    if (mixing.enabled &&
        mixing.rc_chan_override > 0 &&
        rc_input.flags_rc_ok &&
        mixing.rc_chan_override <= IOMCU_MAX_CHANNELS) {
        override_active = (rc_input.pwm[mixing.rc_chan_override-1] >= 1750);
    } else {
        override_active = false;
    }
    if (old_override != override_active) {
        if (override_active) {
            fill_failsafe_pwm();
        }
        chEvtSignal(thread_ctx, EVENT_MASK(IOEVENT_PWM));
    }
}

void AP_IOMCU_FW::process_io_packet()
{
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
        stats.num_bad_crc++;
        return;
    }
    switch (rx_io_packet.code) {
    case CODE_READ: {
        stats.num_code_read++;
        if (!handle_code_read()) {
            tx_io_packet.count = 0;
            tx_io_packet.code = CODE_ERROR;
            tx_io_packet.crc = 0;
            tx_io_packet.page = 0;
            tx_io_packet.offset = 0;
            tx_io_packet.crc =  crc_crc8((const uint8_t *)&tx_io_packet, tx_io_packet.get_size());
        }
    }
    break;
    case CODE_WRITE: {
        stats.num_write_pkt++;
        if (!handle_code_write()) {
            tx_io_packet.count = 0;
            tx_io_packet.code = CODE_ERROR;
            tx_io_packet.crc = 0;
            tx_io_packet.page = 0;
            tx_io_packet.offset = 0;
            tx_io_packet.crc =  crc_crc8((const uint8_t *)&tx_io_packet, tx_io_packet.get_size());
        }
    }
    break;
    default: {
        stats.num_unknown_pkt++;
    }
    break;
    }
}

/*
  update dynamic elements of status page
 */
void AP_IOMCU_FW::page_status_update(void)
{
    if ((reg_setup.features & P_SETUP_FEATURES_SBUS1_OUT) == 0) {
        // we can only get VRSSI when sbus is disabled
        reg_status.vrssi = adc_sample_vrssi();
    } else {
        reg_status.vrssi = 0;
    }
    reg_status.vservo = adc_sample_vservo();
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

                palClearLine(HAL_GPIO_PIN_SBUS_OUT_EN);
            } else {
                palSetLine(HAL_GPIO_PIN_SBUS_OUT_EN);
            }
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
            
        default:
            break;
        }
        break;

    case PAGE_DIRECT_PWM: {
        if (override_active) {
            // no input when override is active
            break;
        }
        /* copy channel data */
        uint16_t i = 0, offset = rx_io_packet.offset, num_values = rx_io_packet.count;
        if (offset + num_values > sizeof(reg_direct_pwm.pwm)/2) {
            return false;
        }
        while ((offset < IOMCU_MAX_CHANNELS) && (num_values > 0)) {
            /* XXX range-check value? */
            if (rx_io_packet.regs[i] != PWM_IGNORE_THIS_CHANNEL) {
                reg_direct_pwm.pwm[offset] = rx_io_packet.regs[i];
            }

            offset++;
            num_values--;
            i++;
        }
        fmu_data_received_time = last_ms;
        reg_status.flag_fmu_ok = true;
        reg_status.flag_raw_pwm = true;
        chEvtSignalI(thread_ctx, EVENT_MASK(IOEVENT_PWM));
        break;
    }

    case PAGE_MIXING: {
        uint16_t offset = rx_io_packet.offset, num_values = rx_io_packet.count;
        if (offset + num_values > sizeof(mixing)/2) {
            return false;
        }
        memcpy(((uint16_t *)&mixing)+offset, &rx_io_packet.regs[0], num_values*2);
        break;
    }

    case PAGE_SAFETY_PWM: {
        uint16_t offset = rx_io_packet.offset, num_values = rx_io_packet.count;
        if (offset + num_values > sizeof(reg_safety_pwm.pwm)/2) {
            return false;
        }
        memcpy((&reg_safety_pwm.pwm[0])+offset, &rx_io_packet.regs[0], num_values*2);
        break;
    }

    case PAGE_FAILSAFE_PWM: {
        uint16_t offset = rx_io_packet.offset, num_values = rx_io_packet.count;
        if (offset + num_values > sizeof(reg_failsafe_pwm.pwm)/2) {
            return false;
        }
        memcpy((&reg_failsafe_pwm.pwm[0])+offset, &rx_io_packet.regs[0], num_values*2);
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
        sum = crc_crc32(sum, (const uint8_t *)&bytes, sizeof(bytes));
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
    }

    led_counter = (led_counter+1) % 16;
    const uint16_t led_pattern = reg_status.flag_safety_off?0xFFFF:0x5500;
    palWriteLine(HAL_GPIO_PIN_SAFETY_LED, (led_pattern & (1U << led_counter))?0:1);
}

/*
  update hal.rcout mode if needed
 */
void AP_IOMCU_FW::rcout_mode_update(void)
{
    bool use_oneshot = (reg_setup.features & P_SETUP_FEATURES_ONESHOT) != 0;
    if (use_oneshot && !oneshot_enabled) {
        oneshot_enabled = true;
        hal.rcout->set_output_mode(reg_setup.pwm_rates, AP_HAL::RCOutput::MODE_PWM_ONESHOT);
    }
    bool use_brushed = (reg_setup.features & P_SETUP_FEATURES_BRUSHED) != 0;
    if (use_brushed && !brushed_enabled) {
        brushed_enabled = true;
        if (reg_setup.pwm_rates == 0) {
            // default to 2kHz for all channels for brushed output
            reg_setup.pwm_rates = 0xFF;
            reg_setup.pwm_altrate = 2000;
            hal.rcout->set_freq(reg_setup.pwm_rates, reg_setup.pwm_altrate);
        }
        hal.rcout->set_esc_scaling(1000, 2000);
        hal.rcout->set_output_mode(reg_setup.pwm_rates, AP_HAL::RCOutput::MODE_PWM_BRUSHED);
        hal.rcout->set_freq(reg_setup.pwm_rates, reg_setup.pwm_altrate);
    }
}

/*
  fill in failsafe PWM values
 */
void AP_IOMCU_FW::fill_failsafe_pwm(void)
{
    for (uint8_t i=0; i<IOMCU_MAX_CHANNELS; i++) {
        if (reg_status.flag_safety_off) {
            reg_direct_pwm.pwm[i] = reg_failsafe_pwm.pwm[i];
        } else {
            reg_direct_pwm.pwm[i] = reg_safety_pwm.pwm[i];
        }
    }
    if (mixing.enabled) {
        run_mixer();
    }
}

AP_HAL_MAIN();



