#ifndef _GPIO_H
#define _GPIO_H

#include "hal_types.h"
#include "exti.h"


/**
 * @brief GPIO Pin modes.
 *
 * These only allow for 50MHZ max output speeds; if you want slower,
 * use direct register access.
 */
 
/*
    we should define modes to be compatible with HAL_GPIO_ defines from HAL.h
#define HAL_GPIO_INPUT  0
#define HAL_GPIO_OUTPUT 1
#define HAL_GPIO_ALT    2

*/
 
typedef enum gpio_pin_mode {
    GPIO_INPUT_FLOATING, 	/**< Input floating. */
    GPIO_OUTPUT_PP, 		/**< Output push-pull. */
    GPIO_AF_OUTPUT_PP, 		/**< Alternate function output push-pull. */
// more complex modes
    GPIO_INPUT_ANALOG, 		/**< Analog input. */
    GPIO_INPUT_PD, 		/**< Input pull-down. */
    GPIO_INPUT_PU, 		/**< Input pull-up. */
    /* GPIO_INPUT_PU treated as a special case, for ODR twiddling */
    GPIO_OUTPUT_OD, 		/**< Output open-drain. */
    GPIO_OUTPUT_OD_PU, 		/**< Output open-drain with pullUp */
    GPIO_AF_OUTPUT_OD, 		/**< Alternate function output open drain. */
    GPIO_AF_OUTPUT_OD_PU, 	/**< Alternate function output open drain with pullup */
    GPIO_PIN_MODE_LAST
} gpio_pin_mode;

#define  Bit_RESET 0
#define  Bit_SET   1

typedef enum
{
  GPIO_speed_2MHz   = 0x00, /*!< Low speed */
  GPIO_speed_25MHz  = 0x01, /*!< Medium speed */
  GPIO_speed_50MHz  = 0x02, /*!< Fast speed */
  GPIO_speed_100MHz = 0x03  /*!< High speed on 30 pF (80 MHz Output max speed on 15 pF) */
} GPIOSpeed_t;



/** GPIO device type */
typedef struct gpio_dev {
    GPIO_TypeDef *GPIOx;      /**< Register map */
    uint32_t clk; 	      /**< RCC clock information */
    afio_exti_port exti_port; /**< AFIO external interrupt port value */
} gpio_dev;

#ifdef __cplusplus
  extern "C" {
#endif

extern const gpio_dev gpioa;
extern const gpio_dev* const _GPIOA;
extern const gpio_dev gpiob;
extern const gpio_dev* const _GPIOB;
extern const gpio_dev gpioc;
extern const gpio_dev* const _GPIOC;
extern const gpio_dev gpiod;
extern const gpio_dev* const _GPIOD;
extern const gpio_dev gpioe;
extern const gpio_dev* const _GPIOE;
extern const gpio_dev gpiof;
extern const gpio_dev* const _GPIOF;
extern const gpio_dev gpiog;
extern const gpio_dev* const _GPIOG;

/**
 * Initialize a GPIO device. 
 */
extern void gpio_init(const gpio_dev* const dev);

/**
 * Initialize and reset all available GPIO devices. 
 */
extern void gpio_init_all(void);

/**
 * Set the mode of a GPIO pin. 
 */
extern void gpio_set_mode(const gpio_dev* const dev, uint8_t pin, gpio_pin_mode mode);


/**
 * Set the alternate function mode of a GPIO pin.
 *
 * @param dev GPIO device.
 * @param pin Pin on the device whose mode to set, 0--15.
 * @param mode alternate function mode to set the pin to.
 * @see gpio_pin_mode
 */
static inline void gpio_set_af_mode(const gpio_dev* const dev, uint8_t pin, uint8_t mode)
{
        /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(dev->GPIOx));
    assert_param(IS_GPIO_PIN_SOURCE(pin));
    assert_param(IS_GPIO_AF(mode));
        
//    GPIO_PinAFConfig(dev->GPIOx, pin, mode);
    uint32_t temp = dev->GPIOx->AFR[pin >> 0x03] & ~((uint32_t)0xF << ((uint32_t)((uint32_t)pin & (uint32_t)0x07) * 4));
    dev->GPIOx->AFR[pin >> 0x03] = temp | ((uint32_t)(mode) << ((uint32_t)((uint32_t)pin & (uint32_t)0x07) * 4));
}
    

static INLINE void gpio_write_bit(const gpio_dev* const dev, uint8_t pin, uint8_t val)
{
	/* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(dev->GPIOx));
    assert_param(IS_GPIO_PIN_SOURCE(pin));

    uint16_t bv = BIT(pin);
    
    if (val) {
	dev->GPIOx->BSRRL = bv;
    } else {
	dev->GPIOx->BSRRH = bv;
    }    
}

static INLINE uint8_t gpio_read_bit(const gpio_dev* const dev, uint8_t pin)
{
    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(dev->GPIOx));
    assert_param(IS_GPIO_PIN_SOURCE(pin));
 
    if ((dev->GPIOx->IDR & BIT(pin)) != Bit_RESET){
	return  (uint8_t)Bit_SET;
    } 

    return (uint8_t)Bit_RESET;

	
}

static inline void gpio_toggle_bit(const gpio_dev* const dev, uint8_t pin)
{
	/* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(dev->GPIOx));
    assert_param(IS_GPIO_PIN_SOURCE(pin));
    dev->GPIOx->ODR ^= BIT(pin);	
}

static inline afio_exti_port gpio_exti_port(const gpio_dev* const dev)
{
	/* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(dev->GPIOx));
    return dev->exti_port;
}


static inline void gpio_set_speed(const gpio_dev* const dev, uint8_t pin, GPIOSpeed_t gpio_speed){
/* Speed mode configuration */
    dev->GPIOx->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << (pin * 2));
    dev->GPIOx->OSPEEDR |=  ((uint32_t)(gpio_speed) << (pin * 2));
}


static inline void afio_exti_select(afio_exti_port gpio_port, afio_exti_num pin)
{
  uint32_t tmp = ((uint32_t)0x0F) << (0x04 * (pin & (uint8_t)0x03));
  SYSCFG->EXTICR[pin >> 0x02] &= ~tmp;
  SYSCFG->EXTICR[pin >> 0x02] |= (((uint32_t)gpio_port) << (0x04 * (pin & (uint8_t)0x03)));

}


#ifdef __cplusplus
  }
#endif
 
#endif

