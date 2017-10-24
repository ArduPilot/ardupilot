// Collection of register definitions for the TI AM335X
// credit to Ethan Hayon for writing the majority of this


#ifndef MMAP_GPIO_ADC_DEFS
#define MMAP_GPIO_ADC_DEFS

#include <stdint.h>

#define MMAP_OFFSET (0x44C00000)
#define MMAP_SIZE   (0x481AEFFF-MMAP_OFFSET)




/* GPIO Memory Registers */
#define GPIO_REGISTER_SIZE (4)

#define GPIO0 	(0x44E07000)
#define GPIO1		(0x4804C000)
#define GPIO2		(0x481AC000)
#define GPIO3		(0x481AE000)

#define GPIO_CLEARDATAOUT (0x190)
#define GPIO_SETDATAOUT   (0x194)
#define GPIO_OE			      (0x134)
#define GPIO_DATAOUT      (0x13C)
#define GPIO_DATAIN       (0x138)

/* Analog Digital Converter Memory Registers */
#define ADC_TSC (0x44E0D000)

#define ADC_CTRL (ADC_TSC+0x40)
#define ADC_STEPCONFIG_WRITE_PROTECT_OFF (0x01<<2)
#define ADC_STEPENABLE (ADC_TSC+0x54)

#define ADCSTEPCONFIG1 (ADC_TSC+0x64)
#define ADCSTEPDELAY1  (ADC_TSC+0x68)
#define ADCSTEPCONFIG2 (ADC_TSC+0x6C)
#define ADCSTEPDELAY2  (ADC_TSC+0x70)
#define ADCSTEPCONFIG3 (ADC_TSC+0x74)
#define ADCSTEPDELAY3  (ADC_TSC+0x78)
#define ADCSTEPCONFIG4 (ADC_TSC+0x7C)
#define ADCSTEPDELAY4  (ADC_TSC+0x80)
#define ADCSTEPCONFIG5 (ADC_TSC+0x84)
#define ADCSTEPDELAY5  (ADC_TSC+0x88)
#define ADCSTEPCONFIG6 (ADC_TSC+0x8C)
#define ADCSTEPDELAY6  (ADC_TSC+0x90)
#define ADCSTEPCONFIG7 (ADC_TSC+0x94)
#define ADCSTEPDELAY7  (ADC_TSC+0x98)
#define ADCSTEPCONFIG8 (ADC_TSC+0x9C)
#define ADCSTEPDELAY8  (ADC_TSC+0xA0)

#define ADC_AVG1 (0b000 << 2)
#define ADC_AVG2 (0b001 << 2)
#define ADC_AVG4 (0b010 << 2)
#define ADC_AVG8 (0b011 << 2)
#define ADC_AVG16 (0b100 << 2)

#define ADC_SW_ONESHOT 0b00
#define FIFO0COUNT (ADC_TSC+0xE4)
#define FIFO_COUNT_MASK 0b01111111

#define ADC_FIFO0DATA (ADC_TSC+0x100)
#define ADC_FIFO_MASK (0xFFF)

#define TRUE 1
#define FALSE 0


#define INPUT    ((unsigned char)(1))
#define OUTPUT   ((unsigned char)(0))
#define PULLUP   ((unsigned char)(1))
#define PULLDOWN ((unsigned char)(0))
#define PULL_DISABLED ((unsigned char)(2))

/*********************************
* clock control registers
*********************************/
#ifndef CM_PER
	#define CM_PER 0x44E00000 //base of Clock Module Peripheral control
	#define CM_PER_PAGE_SIZE 1024 //1kb
#endif


#define CM_PER_GPIO1_CLKCTRL 0xAC
#define CM_PER_GPIO2_CLKCTRL 0xB0
#define CM_PER_GPIO3_CLKCTRL 0xB4

/* Clock Module Memory Registers */
#define CM_WKUP 0x44E00400

#define MODULEMODE_DISABLED 0x0
#define MODULEMODE_ENABLE 	0x2

#define CM_WKUP_ADC_TSC_CLKCTRL 0xBC


#endif
