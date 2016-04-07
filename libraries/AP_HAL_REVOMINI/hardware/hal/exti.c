#include "exti.h"
#include "hal.h"
#include "util.h"

typedef struct exti_channel {
    void (*handler)(void);
    uint32_t irq_line;
    IRQn_Type irq_type;
} exti_channel;

static exti_channel exti_channels[] = {
    { .handler = NULL, .irq_line = EXTI_Line0,		.irq_type = EXTI0_IRQn 		},  // EXTI0
    { .handler = NULL, .irq_line = EXTI_Line1,		.irq_type = EXTI1_IRQn      },  // EXTI1
    { .handler = NULL, .irq_line = EXTI_Line2,		.irq_type = EXTI2_IRQn      },  // EXTI2
    { .handler = NULL, .irq_line = EXTI_Line3,		.irq_type = EXTI3_IRQn      },  // EXTI3
    { .handler = NULL, .irq_line = EXTI_Line4,		.irq_type = EXTI4_IRQn      },  // EXTI4
    { .handler = NULL, .irq_line = EXTI_Line5,		.irq_type = EXTI9_5_IRQn    },  // EXTI5
    { .handler = NULL, .irq_line = EXTI_Line6,		.irq_type = EXTI9_5_IRQn    },  // EXTI6
    { .handler = NULL, .irq_line = EXTI_Line7,		.irq_type = EXTI9_5_IRQn    },  // EXTI7
    { .handler = NULL, .irq_line = EXTI_Line8,		.irq_type = EXTI9_5_IRQn    },  // EXTI8
    { .handler = NULL, .irq_line = EXTI_Line9,		.irq_type = EXTI9_5_IRQn    },  // EXTI9
    { .handler = NULL, .irq_line = EXTI_Line10,		.irq_type = EXTI15_10_IRQn	},  // EXTI10
    { .handler = NULL, .irq_line = EXTI_Line11,		.irq_type = EXTI15_10_IRQn	},  // EXTI11
    { .handler = NULL, .irq_line = EXTI_Line12,		.irq_type = EXTI15_10_IRQn	},  // EXTI12
    { .handler = NULL, .irq_line = EXTI_Line13,		.irq_type = EXTI15_10_IRQn	},  // EXTI13
    { .handler = NULL, .irq_line = EXTI_Line14,		.irq_type = EXTI15_10_IRQn	},  // EXTI14
    { .handler = NULL, .irq_line = EXTI_Line15,		.irq_type = EXTI15_10_IRQn	},  // EXTI15
};

static inline EXTITrigger_TypeDef get_exti_mode(exti_trigger_mode mode) {
    switch (mode) {
    case EXTI_RISING:
        return EXTI_Trigger_Rising;
    case EXTI_FALLING:
        return EXTI_Trigger_Falling;
    case EXTI_RISING_FALLING:
        return EXTI_Trigger_Rising_Falling;
    }
    // Can't happen
    assert_param(0);
    return (EXTITrigger_TypeDef)0;
}

void exti_attach_interrupt(afio_exti_num num,
                           afio_exti_port port,
                           voidFuncPtr handler,
                           exti_trigger_mode mode)
{
	/* Check the parameters */
	assert_param(handler);
	assert_param(IS_EXTI_PIN_SOURCE(num));
	assert_param(IS_EXTI_PORT_SOURCE(num));
	assert_param(port >= 0 && num <= 4);
	
	EXTI_InitTypeDef   EXTI_InitStructure;
	GPIO_InitTypeDef   GPIO_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
  	
	/* Register the handler */
	exti_channels[num].handler = handler;

	gpio_dev *dev = gpio_get_gpio_dev(port);
	
	/* Enable SYSCFG clock */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
 
	/* Enable GPIOx clock */	
	RCC_AHB1PeriphClockCmd(dev->clk, ENABLE);
  
	/* Configure pin as input floating */
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Pin = BIT(num);
	GPIO_Init(dev->GPIOx, &GPIO_InitStructure);

	SYSCFG_EXTILineConfig(port, num);

	/* Configure EXTI Line */
	EXTI_InitStructure.EXTI_Line = exti_channels[num].irq_line;
	EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	EXTI_InitStructure.EXTI_Trigger = get_exti_mode(mode);  
	EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&EXTI_InitStructure);

	/* Enable and set EXTI Line Interrupt to the lowest priority */
	NVIC_InitStructure.NVIC_IRQChannel = exti_channels[num].irq_type;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);  	
}

void exti_detach_interrupt(afio_exti_num num)
{
	/* Check the parameters */
	assert_param(IS_EXTI_PORT_SOURCE(num));
	
	EXTI_InitTypeDef   EXTI_InitStructure;
	NVIC_InitTypeDef   NVIC_InitStructure;
   
	EXTI_StructInit(&EXTI_InitStructure);
	EXTI_InitStructure.EXTI_Line = exti_channels[num].irq_line;
	EXTI_InitStructure.EXTI_LineCmd = DISABLE;
	EXTI_Init(&EXTI_InitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = exti_channels[num].irq_type;
	NVIC_InitStructure.NVIC_IRQChannelCmd = DISABLE;
	NVIC_Init(&NVIC_InitStructure);
  
	/* Finally, unregister the user's handler */
	exti_channels[num].handler = NULL;	
}

/*
 * Interrupt handlers
 */


/*
 * Interrupt handlers
 */


void exti_serv(uint32_t extline, uint8_t num)
{
	if(EXTI_GetITStatus(extline) != RESET)
	{
		voidFuncPtr handler = exti_channels[num].handler;

		if (!handler) {
			EXTI_ClearITPendingBit(extline);
			return;
		}
		
		handler();
        
		/* Clear the EXTI line pending bit */
		EXTI_ClearITPendingBit(extline);
	}

}
    
void EXTI0_IRQHandler(void)
{
	exti_serv(EXTI_Line0, 0);
}

void EXTI1_IRQHandler(void) 
{
	exti_serv(EXTI_Line1, 1);
}

void EXTI2_IRQHandler(void) 
{
	exti_serv(EXTI_Line2, 2);
}

void EXTI3_IRQHandler(void) 
{
	exti_serv(EXTI_Line3, 3);
}

void EXTI4_IRQHandler(void) 
{
	exti_serv(EXTI_Line4, 4);
}

void EXTI9_5_IRQHandler(void) 
{
	exti_serv(EXTI_Line5, 5);
	exti_serv(EXTI_Line6, 6);
	exti_serv(EXTI_Line7, 7);
	exti_serv(EXTI_Line8, 8);
	exti_serv(EXTI_Line9, 9);
}

void EXTI15_10_IRQHandler(void)
{
	exti_serv(EXTI_Line10, 10);
	exti_serv(EXTI_Line11, 11);
	exti_serv(EXTI_Line12, 12);
	exti_serv(EXTI_Line13, 13);
	exti_serv(EXTI_Line14, 14);
	exti_serv(EXTI_Line15, 15);
}

