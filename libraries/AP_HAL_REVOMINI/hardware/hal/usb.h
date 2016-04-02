#ifndef USB_STM32_H
#define USB_STM32_H

#include <stm32f4xx.h>
#include "ring_buffer.h"
#include <gpio_hal.h>
#include <usart.h>

#define USB_IRQ		OTG_FS_IRQn
#define USB_AF		GPIO_AF_OTG1_FS
#define USB_CLOCK	RCC_AHB2Periph_OTG_FS


/* USB D-/D+ (DM/DP) */
#define DM_PIN_PORT   		_GPIOA  //PA11
#define DM_PIN_PIN   		11
#define DP_PIN_PORT   		_GPIOA  //PA12
#define DP_PIN_PIN   		12  //PA12


/* Exported typef ------------------------------------------------------------*/
/* The following structures groups all needed parameters to be configured for the 
   ComPort. These parameters can modified on the fly by the host through CDC class
   command class requests. */
typedef struct
{
  uint32_t bitrate;
  uint8_t  format;
  uint8_t  paritytype;
  uint8_t  datatype;
} LINE_CODING;

/* USB Attribute Data Structure */
typedef struct {
	uint8_t	preempt_prio;				/* pre-emption priority for the IRQ channel */
	uint8_t	sub_prio;						/* subpriority level for the IRQ channel  */
	uint8_t  use_present_pin;
	gpio_dev *present_port;
	uint16_t present_pin;
	char *description;
	char *manufacturer;
	char *serial_number;
	char *configuration;
	char *interface;
} usb_attr_t;

#define I_USB_CLEAR			1
#define I_USB_CONNECTED		2
#define I_USB_GETATTR		3
#define I_USB_SETATTR		4

#ifdef __cplusplus
  extern "C" {
#endif

/* functions */
int usb_open(void);
int usb_close(void);
int usb_ioctl(int request, void * ctl);
int usb_write(uint8_t *buf, unsigned int nbytes);
int usb_read(void  * buf, unsigned int nbytes);
void usb_default_attr(usb_attr_t *attr);
int usb_configure(usb_attr_t * attr);
int8_t usb_getc(void);
uint32_t usb_data_available(void);
uint16_t usb_tx_pending(void);
void usb_putc(uint8_t byte);
void usb_reset_rx(void);
void usb_reset_tx(void);

#ifdef __cplusplus
  }
#endif

#endif
/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
