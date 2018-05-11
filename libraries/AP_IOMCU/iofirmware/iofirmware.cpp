//IO Controller Firmware

#include <AP_HAL/AP_HAL.h>

#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include "iofirmware.h"
#include "hal.h"
extern const AP_HAL::HAL &hal;

//#pragma GCC optimize("Og")

static AP_IOMCU_FW iomcu;

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// pending events on the main thread
enum ioevents {
    IOEVENT_PWM=1,
};

static uint32_t num_code_read, num_bad_crc, num_write_pkt, num_unknown_pkt; 
static uint32_t num_idle_rx, num_dma_complete_rx, num_total_rx, num_rx_error;

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
    num_total_rx++;
    num_dma_complete_rx = num_total_rx - num_idle_rx;

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
        num_rx_error++;
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

    if(sr & USART_SR_IDLE) {
        dma_rx_end_cb(uart);
        num_idle_rx++;
    }
}

/*
 * UART driver configuration structure.
 */
static UARTConfig uart_cfg = {
  NULL,
  NULL,
  dma_rx_end_cb,
  NULL,
  NULL,
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
    thread_ctx = chThdGetSelfX();

    if (palReadLine(HAL_GPIO_PIN_IO_HW_DETECT1) == 1 && palReadLine(HAL_GPIO_PIN_IO_HW_DETECT2) == 0) {
        has_heater = true;
    }
}
    

void AP_IOMCU_FW::update()
{
    chEvtWaitAnyTimeout(~0, MS2ST(1));

    if (do_reboot && (AP_HAL::millis() > reboot_time)) {
        hal.scheduler->reboot(true);
        while(true){}
    }

    // always do pwm update
    pwm_out_update();

    // run remaining functions at 1kHz
    uint32_t now = AP_HAL::millis();
    if (now != last_loop_ms) {
        last_loop_ms = now;
        heater_update();
        rcin_update();
        safety_update();
        rcout_mode_update();
        hal.rcout->timer_tick();
    }
}

void AP_IOMCU_FW::pwm_out_update()
{
    //TODO: PWM mixing
    memcpy(reg_servo.pwm, reg_direct_pwm.pwm, sizeof(reg_direct_pwm));
    hal.rcout->cork();
    for(uint8_t i = 0; i < SERVO_COUNT; i++) {
        if (reg_servo.pwm[i] != 0) {
            hal.rcout->write(i, reg_status.flag_safety_off?reg_servo.pwm[i]:0);
        }
    }
    hal.rcout->push();
}

void AP_IOMCU_FW::heater_update()
{
    uint32_t now = AP_HAL::millis();
    if (!has_heater) {
        // use blue LED as heartbeat
        if (now - last_blue_led_ms > 500) {
            palToggleLine(HAL_GPIO_PIN_HEATER);
            last_blue_led_ms = now;
        }
    } else if (reg_setup.heater_duty_cycle == 0 || (now - last_heater_ms > 3000UL)) {
        palWriteLine(HAL_GPIO_PIN_HEATER, 0);
	} else {
		uint8_t cycle = ((now / 10UL) % 100U);
        palWriteLine(HAL_GPIO_PIN_HEATER, !(cycle >= reg_setup.heater_duty_cycle));
	}
}

void AP_IOMCU_FW::rcin_update()
{
    if (hal.rcin->new_input()) {
        rc_input.count = hal.rcin->num_channels();
        rc_input.flags_rc_ok = true;
        for (uint8_t i = 0; i < IOMCU_MAX_CHANNELS; i++) {
            rc_input.pwm[i] = hal.rcin->read(i);
        }
        rc_input.last_input_us = AP_HAL::micros();
    }
    if (update_rcout_freq) {
        hal.rcout->set_freq(reg_setup.pwm_rates, reg_setup.pwm_altrate);
        update_rcout_freq = false;
    }
    if (update_default_rate) {
        hal.rcout->set_default_rate(reg_setup.pwm_defaultrate);
    }

}

void AP_IOMCU_FW::process_io_packet()
{
    uint8_t rx_crc = rx_io_packet.crc;
    rx_io_packet.crc = 0;
    uint8_t calc_crc = crc_crc8((const uint8_t *)&rx_io_packet, rx_io_packet.get_size());
    if (rx_crc != calc_crc) {
        memset(&tx_io_packet, 0xFF, sizeof(tx_io_packet));
        tx_io_packet.count = 0;
        tx_io_packet.code = CODE_CORRUPT;
        tx_io_packet.crc = 0;
        tx_io_packet.crc =  crc_crc8((const uint8_t *)&tx_io_packet, tx_io_packet.get_size());
        num_bad_crc++;
        return;
    }
    switch(rx_io_packet.code) {
        case CODE_READ:
        {
            num_code_read++;
            if(!handle_code_read()) {
                memset(&tx_io_packet, 0xFF, sizeof(tx_io_packet));
                tx_io_packet.count = 0;
                tx_io_packet.code = CODE_ERROR;
                tx_io_packet.crc = 0;
                tx_io_packet.crc =  crc_crc8((const uint8_t *)&tx_io_packet, tx_io_packet.get_size());
            }
        }
        break;
        case CODE_WRITE:
        {
            num_write_pkt++;
            if(!handle_code_write()) {
                memset(&tx_io_packet, 0xFF, sizeof(tx_io_packet));
                tx_io_packet.count = 0;
                tx_io_packet.code = CODE_ERROR;
                tx_io_packet.crc = 0;
                tx_io_packet.crc =  crc_crc8((const uint8_t *)&tx_io_packet, tx_io_packet.get_size());
            }
        }
        break;
        default:
        {
            num_unknown_pkt++;
        }
        break;
    }
}

bool AP_IOMCU_FW::handle_code_read()
{
    uint16_t *values = NULL;
    #define COPY_PAGE(_page_name)							\
	do {									\
		values = (uint16_t *)&_page_name;				\
		tx_io_packet.count = sizeof(_page_name) / sizeof(uint16_t);	\
	} while(0);

    switch(rx_io_packet.page) {
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
    last_page = rx_io_packet.page;
	last_offset = rx_io_packet.offset;

	/* if the offset is at or beyond the end of the page, we have no data */
	if (rx_io_packet.offset >= tx_io_packet.count) {
		return false;
	}

	/* correct the data pointer and count for the offset */
	values += rx_io_packet.offset;
	tx_io_packet.count -= rx_io_packet.offset;
    memcpy(tx_io_packet.regs, values, sizeof(uint16_t)*tx_io_packet.count);
    tx_io_packet.crc = 0;
    tx_io_packet.crc =  crc_crc8((const uint8_t *)&tx_io_packet, tx_io_packet.get_size());
    return true;
}

bool AP_IOMCU_FW::handle_code_write()
{
    switch(rx_io_packet.page) {
        case PAGE_SETUP:
            switch(rx_io_packet.offset) {
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
                    if (rx_io_packet.regs[0] < 25 && reg_setup.pwm_altclock == 1) {
                        rx_io_packet.regs[0] = 25;
                    }

                    if (rx_io_packet.regs[0] > 400 && reg_setup.pwm_altclock == 1) {
                        rx_io_packet.regs[0] = 400;
                    }
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
                    break;
                case PAGE_REG_SETUP_FEATURES:
                    reg_setup.features = rx_io_packet.regs[0];
                    /* disable the conflicting options with SBUS 1 */
                    if (reg_setup.features & (P_SETUP_FEATURES_SBUS1_OUT)) {
                        reg_setup.features &= ~(P_SETUP_FEATURES_PWM_RSSI |
                                                P_SETUP_FEATURES_ADC_RSSI |
                                                P_SETUP_FEATURES_SBUS2_OUT);
                    }
                    break;
                case PAGE_REG_SETUP_HEATER_DUTY_CYCLE:
                    reg_setup.heater_duty_cycle = rx_io_packet.regs[0];
                    last_heater_ms = AP_HAL::millis();
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

                default:
                    break;
            }
        break;
        case PAGE_DIRECT_PWM:
            {
                /* copy channel data */
                uint8_t i = 0, offset = rx_io_packet.offset, num_values = rx_io_packet.count;
                while ((offset < IOMCU_MAX_CHANNELS) && (num_values > 0)) {
                    /* XXX range-check value? */
                    if (rx_io_packet.regs[i] != PWM_IGNORE_THIS_CHANNEL) {
                        reg_direct_pwm.pwm[offset] = rx_io_packet.regs[i];
                    }

                    offset++;
                    num_values--;
                    i++;
                }
                fmu_data_received_time = AP_HAL::millis();
                reg_status.flag_fmu_ok = true;
                reg_status.flag_raw_pwm = true;
                chEvtSignalI(thread_ctx, EVENT_MASK(IOEVENT_PWM));
                break;
            }
        
        default:
            break;
    }
    memset(&tx_io_packet, 0xFF, sizeof(tx_io_packet));
    tx_io_packet.count = 0;
    tx_io_packet.code = CODE_SUCCESS;
    tx_io_packet.crc = 0;
    tx_io_packet.crc =  crc_crc8((const uint8_t *)&tx_io_packet, tx_io_packet.get_size());
    return true;
}

void AP_IOMCU_FW::schedule_reboot(uint32_t time_ms)
{
    do_reboot = true;
    reboot_time = AP_HAL::millis() + time_ms;
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
    uint32_t now = AP_HAL::millis();
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
}

AP_HAL_MAIN();
