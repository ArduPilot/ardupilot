/*
(c) 2017 night_ghost@ykoctpa.ru
 
based on: LeafLabs

*/

#pragma GCC optimize ("O2")

#include <gpio_hal.h>


/*
 * GPIO devices
 */

const gpio_dev gpioa = {
    .GPIOx     = GPIOA,
    .clk       = RCC_AHB1Periph_GPIOA,
    .exti_port = AFIO_EXTI_PA,
};
/** GPIO port A device. */
const gpio_dev* const _GPIOA = &gpioa;

const gpio_dev gpiob = {
    .GPIOx      = GPIOB,
    .clk       = RCC_AHB1Periph_GPIOB,
    .exti_port = AFIO_EXTI_PB,
};
/** GPIO port B device. */
const gpio_dev* const _GPIOB = &gpiob;

const gpio_dev gpioc = {
    .GPIOx      = GPIOC,
    .clk       = RCC_AHB1Periph_GPIOC,
    .exti_port = AFIO_EXTI_PC,
};
/** GPIO port C device. */
const gpio_dev* const _GPIOC = &gpioc;

const gpio_dev gpiod = {
    .GPIOx      = GPIOD,
    .clk       = RCC_AHB1Periph_GPIOD,
    .exti_port = AFIO_EXTI_PD,
};
/** GPIO port D device. */
const gpio_dev* const _GPIOD = &gpiod;

const gpio_dev gpioe = {
    .GPIOx      = GPIOE,
    .clk       = RCC_AHB1Periph_GPIOE,
    .exti_port = AFIO_EXTI_PE,
};
/** GPIO port E device. */
const gpio_dev* const _GPIOE = &gpioe;

const gpio_dev gpiof = {
    .GPIOx      = GPIOF,
    .clk       = RCC_AHB1Periph_GPIOF,
    .exti_port = AFIO_EXTI_PF,
};
/** GPIO port F device. */
const gpio_dev* const _GPIOF = &gpiof;

const gpio_dev gpiog = {
    .GPIOx      = GPIOG,
    .clk       = RCC_AHB1Periph_GPIOG,
    .exti_port = AFIO_EXTI_PG,
};
/** GPIO port G device. */
const gpio_dev* const _GPIOG = &gpiog;



static const gpio_dev* _gpios[] =  { &gpioa, &gpiob, &gpioc, &gpiod, &gpioe, &gpiof, &gpiog };


#if 0 // unused

void gpio_init(const gpio_dev* const dev) 
{
	/* Check the parameters */
	assert_param(IS_GPIO_ALL_PERIPH(dev->GPIOx));
	GPIO_DeInit(dev->GPIOx);
	/* Enable the GPIO Clock  */
	RCC_AHB1PeriphClockCmd(dev->clk, ENABLE);
}
#endif

void gpio_init_all(void)
{
    RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_GPIOA, DISABLE);

    RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_GPIOB, DISABLE);

    RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_GPIOC, DISABLE);

    RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_GPIOD, DISABLE);

    RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_GPIOE, DISABLE);

    RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_GPIOF, ENABLE);
    RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_GPIOF, DISABLE);

    RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_GPIOG, ENABLE);
    RCC_AHB1PeriphResetCmd(RCC_AHB1Periph_GPIOG, DISABLE);

    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
}



void gpio_set_mode(const gpio_dev* const dev, uint8_t pin, gpio_pin_mode mode)
{
    /* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(dev->GPIOx));
    assert_param(IS_GPIO_PIN_SOURCE(pin));

    GPIO_InitTypeDef config;

    /* Enable the GPIO Clock  */
    RCC_AHB1PeriphClockCmd(dev->clk, ENABLE);
  
    /* Configure the pin */
    GPIO_StructInit(&config);
    config.GPIO_Speed = GPIO_Speed_2MHz; // low noise by default
	
    switch(mode) {
    case GPIO_OUTPUT_PP:
	config.GPIO_Mode = GPIO_Mode_OUT;
	config.GPIO_PuPd = GPIO_PuPd_NOPULL;
	config.GPIO_OType = GPIO_OType_PP;
	break;
    case GPIO_OUTPUT_OD:
	config.GPIO_Mode = GPIO_Mode_OUT;
	config.GPIO_PuPd = GPIO_PuPd_NOPULL;
	config.GPIO_OType = GPIO_OType_OD;
	break;
    case GPIO_OUTPUT_OD_PU:
	config.GPIO_Mode = GPIO_Mode_OUT;
	config.GPIO_PuPd = GPIO_PuPd_UP;
	config.GPIO_OType = GPIO_OType_OD;
	break;
    case GPIO_INPUT_FLOATING:
	config.GPIO_Mode = GPIO_Mode_IN;
	config.GPIO_PuPd = GPIO_PuPd_NOPULL;
	config.GPIO_OType = GPIO_OType_PP;
	break;
    case GPIO_INPUT_ANALOG:
	config.GPIO_Mode = GPIO_Mode_AN;
	config.GPIO_PuPd = GPIO_PuPd_NOPULL;
	config.GPIO_OType = GPIO_OType_PP;
	break;
    case GPIO_INPUT_PU:
	config.GPIO_Mode = GPIO_Mode_IN;
	config.GPIO_PuPd = GPIO_PuPd_UP;
	config.GPIO_OType = GPIO_OType_PP;
	break;
    case GPIO_INPUT_PD:
	config.GPIO_Mode = GPIO_Mode_IN;
	config.GPIO_PuPd = GPIO_PuPd_DOWN;
	config.GPIO_OType = GPIO_OType_PP;
	break;
    case GPIO_AF_OUTPUT_PP:
	config.GPIO_Mode = GPIO_Mode_AF;
	config.GPIO_PuPd = GPIO_PuPd_NOPULL;
	config.GPIO_OType = GPIO_OType_PP;
	break;
    case GPIO_AF_OUTPUT_OD:
	config.GPIO_Mode = GPIO_Mode_AF;
	config.GPIO_PuPd = GPIO_PuPd_NOPULL;
	config.GPIO_OType = GPIO_OType_OD;
	break;
    case GPIO_AF_OUTPUT_OD_PU:
	config.GPIO_Mode = GPIO_Mode_AF;
	config.GPIO_PuPd = GPIO_PuPd_UP;
	config.GPIO_OType = GPIO_OType_OD;
	break;
    default:
	return;
    }

    config.GPIO_Pin = BIT(pin);
    GPIO_Init(dev->GPIOx, &config);
}


