#ifndef _GPIO_H
#define _GPIO_H

#include "hal_types.h"
#include "exti.h"


// AF 0 (default) selection  
#define GPIO_AF_RTC_50Hz      (0)  // RTC_50Hz 
#define GPIO_AF_MCO           (0)  // MCO (MCO1 and MCO2) 
#define GPIO_AF_TAMPER        (0)  // TAMPER (TAMPER_1 and TAMPER_2) 
#define GPIO_AF_SWJ           (0)  // SWJ (SWD and JTAG) 
#define GPIO_AF_TRACE         (0)  // TRACE 


#define GPIO_AF_TIM1          (1)  // TIM1 
#define GPIO_AF_TIM2          (1)  // TIM2 

// AF 2 selection  
#define GPIO_AF_TIM3          (2)  // TIM3 
#define GPIO_AF_TIM4          (2)  // TIM4 
#define GPIO_AF_TIM5          (2)  // TIM5 

// AF 3 selection  
#define GPIO_AF_TIM8          (3)  // TIM8 
#define GPIO_AF_TIM9          (3)  // TIM9 
#define GPIO_AF_TIM10         (3)  // TIM10 
#define GPIO_AF_TIM11         (3)  // TIM11 

// AF 4 selection  
#define GPIO_AF_I2C1          (4)  // I2C1 
#define GPIO_AF_I2C2          (4)  // I2C2 
#define GPIO_AF_I2C3          (4)  // I2C3 

// AF 5 selection  
#define GPIO_AF_SPI1          (5)  // SPI1      
#define GPIO_AF_SPI2          (5)  // SPI2/I2S2 
#define GPIO_AF_SPI4          (5)  // SPI4      
#define GPIO_AF_SPI5          (5)  // SPI5      
#define GPIO_AF_SPI6          (5)  // SPI6      

// AF 6 selection  
#define GPIO_AF_SPI3          (6)  // SPI3/I2S3 

// AF 7 selection  
#define GPIO_AF_USART1        (7)  // USART1  
#define GPIO_AF_USART2        (7)  // USART2  
#define GPIO_AF_USART3        (7)  // USART3  
#define GPIO_AF_I2S3ext       (7)  // I2S3ext 

// AF 8 selection  
#define GPIO_AF_UART4         (8)  // UART4  
#define GPIO_AF_UART5         (8)  // UART5  
#define GPIO_AF_USART6        (8)  // USART6 
#define GPIO_AF_UART7         (8)  // UART7  
#define GPIO_AF_UART8         (8)  // UART8  

//   AF 9 selection 
#define GPIO_AF_CAN1          (9)  // CAN1  
#define GPIO_AF_CAN2          (9)  // CAN2  
#define GPIO_AF_TIM12         (9)  // TIM12 
#define GPIO_AF_TIM13         (9)  // TIM13 
#define GPIO_AF_TIM14         (9)  // TIM14 

// AF 10 selection  
#define GPIO_AF_OTG_FS         (0xA)  // OTG_FS 
#define GPIO_AF_OTG_HS         (0xA)  // OTG_HS 

//  AF 11 selection  
#define GPIO_AF_ETH             (0xB)  // ETHERNET 

// AF 12 selection  
#define GPIO_AF_FSMC             (0xC)  // FSMC                     

#define GPIO_AF_OTG_HS_FS        (0xC)  // OTG HS configured in FS, 
#define GPIO_AF_SDIO             (0xC)  // SDIO                     

//     AF 13 selection  
#define GPIO_AF_DCMI          (0xD)  // DCMI


#define GPIO_AF_OTG1_FS         GPIO_AF_OTG_FS
#define GPIO_AF_OTG2_HS         GPIO_AF_OTG_HS
#define GPIO_AF_OTG2_FS         GPIO_AF_OTG_HS_FS
 
typedef enum gpio_pin_mode {
    GPIO_INPUT_FLOATING, 	// Input floating. 
    GPIO_OUTPUT_PP, 		// Output push-pull. 
    GPIO_AF_OUTPUT_PP, 		// Alternate function output push-pull. 
// more complex modes
    GPIO_INPUT_ANALOG, 		// Analog input. 
    GPIO_INPUT_PD, 		// Input pull-down. 
    GPIO_INPUT_PU, 		// Input pull-up. 
    /* GPIO_INPUT_PU treated as a special case, for ODR twiddling */
    GPIO_OUTPUT_OD, 		// Output open-drain. 
    GPIO_OUTPUT_OD_PU, 		// Output open-drain with pullUp 
    GPIO_AF_OUTPUT_OD, 		// Alternate function output open drain. 
    GPIO_AF_OUTPUT_OD_PU, 	// Alternate function output open drain with pullup 
    GPIO_PIN_MODE_LAST
} gpio_pin_mode;

#define  Bit_RESET 0
#define  Bit_SET   1

typedef enum
{
  GPIO_speed_2MHz   = 0, // Low speed 
  GPIO_speed_25MHz  = 1, // Medium speed 
  GPIO_speed_50MHz  = 2, // Fast speed 
  GPIO_speed_100MHz = 3  // High speed on 30 pF (80 MHz Output max speed on 15 pF) 
} GPIOSpeed_t;

typedef enum
{
  GPIO_Mode_IN   = 0, // GPIO Input Mode 
  GPIO_Mode_OUT  = 1, // GPIO Output Mode 
  GPIO_Mode_AF   = 2, // GPIO Alternate function Mode 
  GPIO_Mode_AN   = 3  // GPIO Analog Mode 
}GPIOMode_t;

typedef enum
{
  GPIO_OType_PP = 0,
  GPIO_OType_OD = 1
}GPIOOType_t;

typedef enum
{
  GPIO_PuPd_NOPULL = 0,
  GPIO_PuPd_UP     = 1,
  GPIO_PuPd_DOWN   = 2
}GPIOPuPd_t;

/** GPIO device type */
typedef struct gpio_dev {
    GPIO_TypeDef *regs;      // Register map 
    uint32_t clk; 	      // RCC clock information 
    afio_exti_port exti_port; // AFIO external interrupt port value 
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
    
    uint32_t reg = pin >> 3;
    uint32_t pos = (uint32_t)(pin & 7) * 4;
    
    uint32_t tmp = dev->regs->AFR[reg] & ~((uint32_t)0xF  << pos);
    dev->regs->AFR[reg] = tmp |           ((uint32_t)mode << pos);
}
    

static INLINE void gpio_write_bit(const gpio_dev* const dev, uint8_t pin, uint8_t val)
{
    uint16_t bv = BIT(pin);
    
    if (val) {
	dev->regs->BSRRL = bv;
    } else {
	dev->regs->BSRRH = bv;
    }    
}

static INLINE uint8_t gpio_read_bit(const gpio_dev* const dev, uint8_t pin)
{
    if ((dev->regs->IDR & BIT(pin)) != 0){
	return  1;
    } 

    return 0;
}

static inline void gpio_toggle_bit(const gpio_dev* const dev, uint8_t pin)
{
    dev->regs->ODR ^= BIT(pin);	
}

static inline afio_exti_port gpio_exti_port(const gpio_dev* const dev)
{
    return dev->exti_port;
}


static inline void gpio_set_speed(const gpio_dev* const dev, uint8_t pin, GPIOSpeed_t gpio_speed){ /* Speed mode configuration */
    dev->regs->OSPEEDR &= ~(GPIO_OSPEEDER_OSPEEDR0 << pin * 2);
    dev->regs->OSPEEDR |=     (uint32_t)gpio_speed << pin * 2;
}


static inline void afio_exti_select(afio_exti_port gpio_port, afio_exti_num pin)
{
    uint32_t reg = pin >> 0x02;
    uint32_t pos = 4 * (pin & 3); // 4 bit per port
    SYSCFG->EXTICR[reg] &= ~(((uint32_t)0x0F)      << pos);
    SYSCFG->EXTICR[reg] |=  (((uint32_t)gpio_port) << pos);
}


#ifdef __cplusplus
  }
#endif
 
#endif


