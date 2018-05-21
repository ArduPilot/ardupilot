/*
    (c) 2017 night_ghost@ykoctpa.ru
 
based on: datasheet

*/

#pragma GCC optimize ("O2")

#include "i2c.h"
#include "dma.h"
#include "systick.h"
#include <boards.h>


#if defined(I2C1_SDA) && defined(I2C1_SCL)
static i2c_state i2c1_state IN_CCM;

static const i2c_dev i2c_dev1 = {
    .regs         = I2C1,
    .sda_pin      = I2C1_SDA,
    .scl_pin      = I2C1_SCL, // 101 PB8
    .clk       	  = RCC_APB1_bit_I2C1,
    .gpio_af	  = GPIO_AF_I2C1,
    .ev_nvic_line = I2C1_EV_IRQn,
    .er_nvic_line = I2C1_ER_IRQn,
//    .dma          = { DMA_CR_CH1, DMA1_STREAM0, DMA1_STREAM6 }, // I2C1
    .state        = &i2c1_state,
};
// I2C1 device 
const i2c_dev* const _I2C1 = &i2c_dev1;
#endif

#if defined(I2C2_SDA) && defined(I2C2_SCL)
static i2c_state i2c2_state IN_CCM;

static const i2c_dev i2c_dev2 = {
    .regs         = I2C2,
    .sda_pin      = I2C2_SDA,
    .scl_pin      = I2C2_SCL,
    .clk       	  = RCC_APB1_bit_I2C2,
    .gpio_af	  = GPIO_AF_I2C2,
    .ev_nvic_line = I2C2_EV_IRQn,
    .er_nvic_line = I2C2_ER_IRQn,
//    .dma          = { DMA_CR_CH7, DMA1_STREAM3 /* intersects with spi2_tx */ , DMA1_STREAM7 }, // I2C2
    .state        = &i2c2_state,
};

/** I2C2 device */
const i2c_dev* const _I2C2 = &i2c_dev2;
#endif

#if defined(I2C3_SDA) && defined(I2C3_SCL)
static i2c_state i2c3_state IN_CCM;

static const i2c_dev i2c_dev3 = {
    .regs         = I2C3,
    .sda_pin      = I2C3_SDA,
    .scl_pin      = I2C3_SCL,
    .clk       	  = RCC_APB1_bit_I2C3,
    .gpio_af	  = GPIO_AF_I2C3,
    .ev_nvic_line = I2C3_EV_IRQn,
    .er_nvic_line = I2C3_ER_IRQn,
    .state        = &i2c3_state,
};

/** I2C3 device */
const i2c_dev* const _I2C3 = &i2c_dev3;
#endif


typedef enum {TX = 0, RX = 1, TXREG = 2} I2C_Dir;

static void delay_10us(){
    hal_delay_microseconds(10);
}

uint32_t i2c_bit_time=4;

void i2c_lowLevel_deinit(const i2c_dev *dev){
    i2c_master_release_bus(dev);
    i2c_peripheral_disable(dev);
    RCC_doAPB1_reset(dev->clk);
}

/**
 *   Initializes hw used by the I2C driver.
 */
static inline void i2c_lowLevel_init(const i2c_dev *dev)  {
    memset(dev->state,0,sizeof(i2c_state));
    
    RCC_doAPB1_reset(dev->clk); // Enable the i2c and Reset it

    { /* Configure SCL */
        const stm32_pin_info *p = &PIN_MAP[dev->scl_pin];

        /* Enable the GPIOs for the SCL/SDA Pins */
        RCC_enableAHB1_clk(p->gpio_device->clk);

        gpio_set_mode(   p->gpio_device, p->gpio_bit, GPIO_AF_OUTPUT_OD_PU);
        gpio_set_af_mode(p->gpio_device, p->gpio_bit, dev->gpio_af);
        gpio_set_speed(  p->gpio_device, p->gpio_bit, GPIO_speed_25MHz);

        gpio_set_af_mode(p->gpio_device, p->gpio_bit, dev->gpio_af); // Connect GPIO pins to peripheral, SCL must be first!
    }
    
    { /* Configure SDA */
        const stm32_pin_info *p = &PIN_MAP[dev->sda_pin];

        /* Enable the GPIOs for the SCL/SDA Pins */
        RCC_enableAHB1_clk(p->gpio_device->clk);

        gpio_set_mode(   p->gpio_device, p->gpio_bit, GPIO_AF_OUTPUT_OD_PU);
        gpio_set_af_mode(p->gpio_device, p->gpio_bit, dev->gpio_af);
        gpio_set_speed(  p->gpio_device, p->gpio_bit, GPIO_speed_25MHz);

        gpio_set_af_mode(p->gpio_device, p->gpio_bit, dev->gpio_af);
    }
}


void i2c_init(const i2c_dev *dev, uint16_t address, uint32_t speed)
{
    i2c_lowLevel_init(dev); // init GPIO hardware

    i2c_bit_time = 1000000l / speed;

    // Disable all I2C interrupts 
    i2c_disable_irq(dev, I2C_CR2_ITERREN | I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN);

    (void)dev->regs->DR; 

    i2c_peripheral_enable(dev);
  
    RCC_Clocks_t clock;
    RCC_GetClocksFreq(&clock);         // Get pclk1 frequency value
    uint32_t  pclk1 = clock.PCLK1_Frequency;
  
    uint16_t freq = (uint16_t)(pclk1 / 1000000);

    uint16_t  reg = dev->regs->CR2 & (uint16_t)~I2C_CR2_FREQ; // Clear FREQ[5:0]
    dev->regs->CR2 = reg | freq; // set calculated frequency

    // Disable  I2C hw to setup TRISE
    i2c_peripheral_disable(dev);    // clears F/S, DUTY and CCR[11:0] bits
    
    if (speed <= 100000) {    // standard mode speed calculate
        uint16_t ccr = (uint16_t)(pclk1 / (speed * 2));
    
        if (ccr < 0x04) { //  CCR should not be under 0x4
            ccr = 0x04;   
        }
        dev->regs->CCR = ccr;
        dev->regs->TRISE = freq + 1; // set Maximum Rise time
    }  else {     // fast mode
                // (datasheet) To use the I2C at 400 KHz (in fast mode), the PCLK1 frequency (I2C input clock) must be a multiple of 10 MHz 
        uint16_t ccr = (uint16_t)(pclk1 / (speed * 3));
    
        if ((ccr & I2C_CCR_CCR) == 0) { // CCR should not be under 0x1
            ccr |= 0x01;
        }
        
        dev->regs->CCR = (uint16_t)(ccr | I2C_CCR_FS); // set speed value and F/S bit
        dev->regs->TRISE = (uint16_t)(freq * 300 / 1000 + 1); // set Maximum Rise time
    }
    i2c_peripheral_enable(dev);

    reg = dev->regs->CR1 & ~(I2C_CR1_SMBUS | I2C_CR1_SMBTYPE); // clear SMBTYPE and SMBUS bits
    dev->regs->CR1 = reg | I2C_CR1_ACK; // enable acknowledgement 
  
    dev->regs->OAR1 = I2C_OAR_7bitMode | address;  
}

/**
 * DeInitializes peripherals used by the I2C driver.
 */
void i2c_deinit(const i2c_dev *dev)
{
    i2c_lowLevel_deinit(dev);
}

static void ev_handler(const i2c_dev *dev, bool err){
    if(dev->state->handler) revo_call_handler(dev->state->handler, err);
    else {
        i2c_disable_irq(dev, I2C_CR2_ITBUFEN|I2C_CR2_ITEVTEN|I2C_CR2_ITERREN);    // Disable interrupts
    }
}

void I2C1_EV_IRQHandler(); // to avoid warnings
void I2C1_ER_IRQHandler();
void I2C2_EV_IRQHandler();
void I2C2_ER_IRQHandler();
void I2C3_EV_IRQHandler();
void I2C3_ER_IRQHandler();

#if defined(I2C1_SDA) && defined(I2C1_SCL)
void I2C1_EV_IRQHandler(){                // I2C1 Event                   
    ev_handler(_I2C1, false);
}
  
void I2C1_ER_IRQHandler(){                // I2C1 Error                   
    ev_handler(_I2C1, true);
}
#endif

#if defined(I2C2_SDA) && defined(I2C2_SCL)
void I2C2_EV_IRQHandler(){                // I2C2 Event                 
    ev_handler(_I2C2, false);
}

void I2C2_ER_IRQHandler(){                // I2C2 Error
    ev_handler(_I2C2, true);
}
#endif

#if defined(I2C3_SDA) && defined(I2C3_SCL)
void I2C3_EV_IRQHandler(){                // I2C2 Event                 
    ev_handler(_I2C3, false);
}

void I2C3_ER_IRQHandler(){                // I2C2 Error
    ev_handler(_I2C3, true);
}
#endif

void i2c_master_release_bus(const i2c_dev *dev) {
    const stm32_pin_info *p_sda = &PIN_MAP[dev->sda_pin];
    const stm32_pin_info *p_scl = &PIN_MAP[dev->scl_pin];

    gpio_write_bit(p_scl->gpio_device, p_scl->gpio_bit, 1);
    gpio_write_bit(p_sda->gpio_device, p_sda->gpio_bit, 1);
    gpio_set_mode(p_scl->gpio_device, p_scl->gpio_bit, GPIO_OUTPUT_OD_PU);
    gpio_set_mode(p_sda->gpio_device, p_sda->gpio_bit, GPIO_OUTPUT_OD_PU);
}


/**
 * @brief Reset an I2C bus.
 *
 * Reset is accomplished by clocking out pulses until any hung slaves
 * release SDA and SCL, then generating a START condition, then a STOP
 * condition.
 *
 * @param dev I2C device
 */
 
#define MAX_I2C_TIME 300 // 300ms before device turn off

bool i2c_bus_reset(const i2c_dev *dev) {

    /* Release both lines */
    i2c_master_release_bus(dev);

    uint32_t t=systick_uptime();

    const stm32_pin_info *p_sda = &PIN_MAP[dev->sda_pin];
    const stm32_pin_info *p_scl = &PIN_MAP[dev->scl_pin];

    /*
     * Make sure the bus is free by clocking it until any slaves release the
     * bus.
     */

again:
    /* Wait for any clock stretching to finish */
    while (!gpio_read_bit(p_scl->gpio_device, p_scl->gpio_bit)) {// device can output 1 so check clock first
        if(systick_uptime()-t > MAX_I2C_TIME) return false;
        hal_yield(10);
    }
    delay_10us();	// 50kHz

    while (!gpio_read_bit(p_sda->gpio_device, p_sda->gpio_bit)) {
        /* Wait for any clock stretching to finish */
        while (!gpio_read_bit(p_scl->gpio_device, p_scl->gpio_bit)){
            if(systick_uptime()-t > MAX_I2C_TIME) return false;
            hal_yield(10);
        }
        delay_10us();	// 50kHz

        /* Pull low */
        gpio_write_bit(p_scl->gpio_device, p_scl->gpio_bit, 0);
        delay_10us();

        /* Release high again */
        gpio_write_bit(p_scl->gpio_device, p_scl->gpio_bit, 1);
        delay_10us();
    }

    /* Generate start then stop condition */
    gpio_write_bit(p_sda->gpio_device, p_sda->gpio_bit, 0);
    delay_10us();
    gpio_write_bit(p_scl->gpio_device, p_scl->gpio_bit, 0);
    delay_10us();
    gpio_write_bit(p_scl->gpio_device, p_scl->gpio_bit, 1);
    delay_10us();
    gpio_write_bit(p_sda->gpio_device, p_sda->gpio_bit, 1);
    
    uint32_t rtime = stopwatch_getticks();
    uint32_t dt    = us_ticks * 50; // 50uS

    while ((stopwatch_getticks() - rtime) < dt) {
        if (!gpio_read_bit(p_scl->gpio_device, p_scl->gpio_bit))  goto again; // any SCL activity after STOP
    }

// we was generating signals on I2C bus, but BUSY flag senses it even when hardware is off
// datasheet: It indicates a communication in progress on the bus. This information is still updated when the interface is disabled (PE=0).

    dev->regs->CR1 |= I2C_CR1_SWRST;           // set SoftReset for some time 
    hal_yield(0);
    dev->regs->CR1 &= (uint16_t)(~I2C_CR1_SWRST); // clear SoftReset flag 
    return true;
}

