#include <gpio_hal.h>
#include <hal.h>

/*
 * GPIO devices
 */

gpio_dev gpioa = {
    .GPIOx     = GPIOA,
    .clk       = RCC_AHB1Periph_GPIOA,
    .exti_port = AFIO_EXTI_PA,
};
/** GPIO port A device. */
gpio_dev* const _GPIOA = &gpioa;

gpio_dev gpiob = {
    .GPIOx      = GPIOB,
    .clk       = RCC_AHB1Periph_GPIOB,
    .exti_port = AFIO_EXTI_PB,
};
/** GPIO port B device. */
gpio_dev* const _GPIOB = &gpiob;

gpio_dev gpioc = {
    .GPIOx      = GPIOC,
    .clk       = RCC_AHB1Periph_GPIOC,
    .exti_port = AFIO_EXTI_PC,
};
/** GPIO port C device. */
gpio_dev* const _GPIOC = &gpioc;

gpio_dev gpiod = {
    .GPIOx      = GPIOD,
    .clk       = RCC_AHB1Periph_GPIOD,
    .exti_port = AFIO_EXTI_PD,
};
/** GPIO port D device. */
gpio_dev* const _GPIOD = &gpiod;

gpio_dev gpioe = {
    .GPIOx      = GPIOE,
    .clk       = RCC_AHB1Periph_GPIOE,
    .exti_port = AFIO_EXTI_PE,
};
/** GPIO port E device. */
gpio_dev* const _GPIOE = &gpioe;

gpio_dev gpiof = {
    .GPIOx      = GPIOF,
    .clk       = RCC_AHB1Periph_GPIOF,
    .exti_port = AFIO_EXTI_PF,
};
/** GPIO port F device. */
gpio_dev* const _GPIOF = &gpiof;

gpio_dev gpiog = {
    .GPIOx      = GPIOG,
    .clk       = RCC_AHB1Periph_GPIOG,
    .exti_port = AFIO_EXTI_PG,
};
/** GPIO port G device. */
gpio_dev* const _GPIOG = &gpiog;










void gpio_init(gpio_dev* dev) 
{
	/* Check the parameters */
	assert_param(IS_GPIO_ALL_PERIPH(dev->GPIOx));
	GPIO_DeInit(dev->GPIOx);
	/* Enable the GPIO Clock  */
	RCC_AHB1PeriphClockCmd(dev->clk, ENABLE);	
}

void gpio_init_all(void)
{
	GPIO_DeInit(GPIOA);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);	
	GPIO_DeInit(GPIOB);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);	
	GPIO_DeInit(GPIOC);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);	
	GPIO_DeInit(GPIOD);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);	
	GPIO_DeInit(GPIOE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOE, ENABLE);	
}

gpio_dev * gpio_get_gpio_dev(uint8_t port)
{
	// Check the parameters 
	assert_param(num >= 0 && num <= 4);
	
	gpio_dev *dev;
	  	
	switch(port) {
		case 0: dev = _GPIOA;
				break;
		case 1: dev = _GPIOB;
				break;
		case 2: dev = _GPIOC;
				break;
		case 3: dev = _GPIOD;
				break;
		case 4: dev = _GPIOE;
				break;
		default:
				assert_param(0);
				//errno_r = EINVAL;
				dev = NULL;				
	}		
	return dev;
}


void gpio_set_mode(gpio_dev* dev, uint8_t pin, gpio_pin_mode mode)
{
	/* Check the parameters */
	assert_param(IS_GPIO_ALL_PERIPH(dev->GPIOx));
	assert_param(IS_GPIO_PIN_SOURCE(pin));
	
	GPIO_InitTypeDef config;
	
	/* Enable the GPIO Clock  */
	RCC_AHB1PeriphClockCmd(dev->clk, ENABLE);
  
	/* Configure the pin */
	GPIO_StructInit(&config);
	config.GPIO_Speed = GPIO_Speed_50MHz;
    switch(mode) 
    {
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
			config.GPIO_PuPd = GPIO_PuPd_UP;
			config.GPIO_OType = GPIO_OType_PP;
			break;
		case GPIO_AF_OUTPUT_OD:
			config.GPIO_Mode = GPIO_Mode_AF;
			config.GPIO_PuPd = GPIO_PuPd_NOPULL;
			config.GPIO_OType = GPIO_OType_OD;
			break;
		default:
			//errno_r = EINVAL;
			return;
    }

	config.GPIO_Pin = BIT(pin);
	GPIO_Init(dev->GPIOx, &config);      	
}

void gpio_write_bit(gpio_dev* dev, uint8_t pin, uint8_t val)
{
	/* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(dev->GPIOx));
    assert_param(IS_GPIO_PIN_SOURCE(pin));
    
    if (val) 
    {
		dev->GPIOx->BSRRL = BIT(pin);
	}
    else
    {
		dev->GPIOx->BSRRH = BIT(pin);
    }    
}

uint8_t gpio_read_bit(gpio_dev* dev, uint8_t pin)
{
	uint8_t bitstatus = 0x00;

	/* Check the parameters */
	assert_param(IS_GPIO_ALL_PERIPH(dev->GPIOx));
	assert_param(IS_GPIO_PIN_SOURCE(pin));
 
	if ((dev->GPIOx->IDR & BIT(pin)) != (uint32_t)Bit_RESET)
	{
		bitstatus = (uint8_t)Bit_SET;
	}
	else
	{
		bitstatus = (uint8_t)Bit_RESET;
	}
   return bitstatus;

	
}

void gpio_toggle_bit(gpio_dev* dev, uint8_t pin)
{
	/* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(dev->GPIOx));
    assert_param(IS_GPIO_PIN_SOURCE(pin));
    dev->GPIOx->ODR ^= BIT(pin);	
}

afio_exti_port gpio_exti_port(gpio_dev* dev)
{
	/* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(dev->GPIOx));
    return dev->exti_port;
}

void gpio_set_af_mode(gpio_dev* dev, uint8_t pin, int mode)
{
	/* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(dev->GPIOx));
    assert_param(IS_GPIO_PIN_SOURCE(pin));
    assert_param(IS_GPIO_AF(mode));
    
	/* Enable the GPIO Clock  */
	RCC_AHB1PeriphClockCmd(dev->clk, ENABLE);    
	GPIO_PinAFConfig(dev->GPIOx, pin, mode);
}


void afio_exti_select(afio_exti_num exti, afio_exti_port gpio_port)
{
	/* Check the parameters */
	assert_param(IS_EXTI_PIN_SOURCE(exti));
	assert_param(IS_EXTI_PORT_SOURCE(gpio_port));
		
	SYSCFG_EXTILineConfig(gpio_port, exti);
}


void afio_remap(gpio_dev* dev, uint8_t pin, afio_remap_peripheral remapping)
{
	/* Check the parameters */
    assert_param(IS_GPIO_ALL_PERIPH(dev->GPIOx));
    assert_param(IS_GPIO_PIN_SOURCE(pin));
    assert_param(IS_GPIO_AF(remapping));
    	
	/* Enable the GPIO Clock  */
	RCC_AHB1PeriphClockCmd(dev->clk, ENABLE);    	
	GPIO_PinAFConfig(dev->GPIOx, BIT(pin), remapping);
}

void afio_cfg_debug_ports(afio_debug_cfg config)
{
	GPIO_InitTypeDef GPIO_InitStructure;


				
	switch(config)
	{
		case AFIO_DEBUG_NONE:
			/* Enable GPIOA and GPIOB clocks */
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_GPIOB , ENABLE);		
			
			/* Configure PA.13 (JTMS/SWDIO), PA.14 (JTCK/SWCLK) and PA.15 (JTDI) as output push-pull */
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_Init(GPIOA, &GPIO_InitStructure);

			/* Configure PB.03 (JTDO) and PB.04 (JTRST) as output push-pull */
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_4;
			GPIO_Init(GPIOB, &GPIO_InitStructure);			
		
			break;
		case AFIO_DEBUG_SW_ONLY:
			/* Enable GPIOA clocks */
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA , ENABLE);
			
			/* Configure PA.13 (JTMS/SWDIO), PA.14 (JTCK/SWCLK) as output push-pull */
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_14;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_Init(GPIOA, &GPIO_InitStructure);		
			break;
		case AFIO_DEBUG_FULL_SWJ_NO_NJRST:
			/* Enable GPIOB clocks */
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB , ENABLE);
					
			/* Configure PB.04 (JTRST) as output push-pull */
			GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
			GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
			GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
			GPIO_Init(GPIOB, &GPIO_InitStructure);				
			break;
		case AFIO_DEBUG_FULL_SWJ:
			break;
		default:
			//errno_r = EINVAL;
			return;			
	}
}
