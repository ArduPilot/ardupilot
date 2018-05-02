//
// Simple test for the AP_AHRS interface
//

#include <AP_HAL/AP_HAL.h>

#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include "iofirmware.h"
#include "hal.h"
extern const AP_HAL::HAL &hal;
#pragma GCC optimize("O0")
#define PKT_MAX_REGS 32
AP_IOMCU_FW iomcu;
/*
  values for pkt.code
 */
enum iocode {
    // read types
    CODE_READ = 0,
    CODE_WRITE = 1,

    // reply codes
    CODE_SUCCESS = 0,
    CODE_CORRUPT = 1,
    CODE_ERROR = 2
};

// IO pages
enum iopage {
    PAGE_CONFIG = 0,
    PAGE_STATUS = 1,
    PAGE_ACTUATORS = 2,
    PAGE_SERVOS = 3,
    PAGE_RAW_RCIN = 4,
    PAGE_RCIN = 5,
    PAGE_RAW_ADC = 6,
    PAGE_PWM_INFO = 7,
    PAGE_SETUP = 50,
    PAGE_DIRECT_PWM = 54,
};

// setup page registers
#define PAGE_REG_SETUP_FEATURES	0
#define P_SETUP_FEATURES_SBUS1_OUT	1
#define P_SETUP_FEATURES_SBUS2_OUT	2
#define P_SETUP_FEATURES_PWM_RSSI   4
#define P_SETUP_FEATURES_ADC_RSSI   8
#define P_SETUP_FEATURES_ONESHOT   16

#define PAGE_REG_SETUP_ARMING 1
#define P_SETUP_ARMING_IO_ARM_OK (1<<0)
#define P_SETUP_ARMING_FMU_ARMED (1<<1)
#define P_SETUP_ARMING_RC_HANDLING_DISABLED (1<<6)

#define PAGE_REG_SETUP_PWM_RATE_MASK 2
#define PAGE_REG_SETUP_DEFAULTRATE   3
#define PAGE_REG_SETUP_ALTRATE       4
#define PAGE_REG_SETUP_SBUS_RATE    19
#define PAGE_REG_SETUP_HEATER_DUTY_CYCLE 21

#define PAGE_REG_SETUP_FORCE_SAFETY_OFF 12
#define PAGE_REG_SETUP_FORCE_SAFETY_ON  14
#define FORCE_SAFETY_MAGIC 22027

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

static uint32_t num_code_read = 0, num_bad_crc = 0, num_write_pkt = 0, num_unknown_pkt = 0; 
static uint32_t num_idle_rx = 0, num_dma_complete_rx = 0, num_total_rx = 0, num_rx_error = 0;

void dma_rx_end_cb(UARTDriver *uart)
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

void idle_rx_handler(UARTDriver *uart) {
    volatile uint16_t sr = uart->usart->SR;
    if (sr & (USART_SR_ORE |	/* overrun error - packet was too big for DMA or DMA was too slow */
		  USART_SR_NE |		/* noise error - we have lost a byte due to noise */
		  USART_SR_FE)) {		/* framing error - start/stop bit lost or line break */
		/* send a line break - this will abort transmission/reception on the other end */
        osalSysLockFromISR();
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
    hal.gpio->init();
    hal.rcin->init();
    hal.rcout->init();

    for (uint8_t i = 0; i< 14; i++) {
        hal.rcout->enable_ch(i);
    }
    (void)uart_cfg;
    uartStart(&UARTD2, &uart_cfg);
    uartStartReceive(&UARTD2, sizeof(iomcu.rx_io_packet), &iomcu.rx_io_packet);
}

void loop(void)
{
    iomcu.pwm_out_update();
    iomcu.heater_update();
    iomcu.rcin_update();
}

void AP_IOMCU_FW::pwm_out_update()
{
    //TODO: PWM mixing
    memcpy(reg_servo.pwm, reg_direct_pwm.pwm, sizeof(reg_direct_pwm));
    for(uint8_t i = 0; i < SERVO_COUNT; i++) {
        if (reg_servo.pwm[i] != 0) {
            hal.rcout->write(i, reg_servo.pwm[i]);
        }
    }
}

void AP_IOMCU_FW::heater_update()
{
	if (reg_setup.heater_duty_cycle == 0 || (AP_HAL::millis() - last_heater_ms > 3000UL)) {
		hal.gpio->write(0, 0);
	} else {
		uint8_t cycle = ((AP_HAL::millis() / 10UL) % 100U);
		hal.gpio->write(0, !(cycle >= reg_setup.heater_duty_cycle));
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
            handle_code_read();
        }
        break;
        case CODE_WRITE:
        {
            num_write_pkt++;
            handle_code_write();
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
                        reg_status.flag_safety_off = true;
                    } else {
                        return false;
                    }
                    break;
                case PAGE_REG_SETUP_FORCE_SAFETY_ON:
                    if (rx_io_packet.regs[0] == FORCE_SAFETY_MAGIC) {
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
                    hal.rcout->set_freq(reg_setup.pwm_rates, reg_setup.pwm_altrate);
                    break;
                case PAGE_REG_SETUP_PWM_RATE_MASK:
                    reg_setup.pwm_rates = rx_io_packet.regs[0];
                    hal.rcout->set_freq(reg_setup.pwm_rates, reg_setup.pwm_altrate);
                    break;
                case PAGE_REG_SETUP_DEFAULTRATE:
                    if (rx_io_packet.regs[0] < 25 && reg_setup.pwm_altclock == 1) {
                        rx_io_packet.regs[0] = 25;
                    }

                    if (rx_io_packet.regs[0] > 400 && reg_setup.pwm_altclock == 1) {
                        rx_io_packet.regs[0] = 400;
                    }
                    reg_setup.pwm_defaultrate = rx_io_packet.regs[0];
                    hal.rcout->set_default_rate(reg_setup.pwm_defaultrate);
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
                //TODO: Oneshot support
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

AP_HAL_MAIN();
