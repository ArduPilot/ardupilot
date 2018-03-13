#ifndef USB_STM32_H
#define USB_STM32_H

#include <hal_types.h>
#include <gpio_hal.h>
#include <usart.h>

#include <stdbool.h>

#ifdef __cplusplus
  extern "C" {
#endif


#include "../STM32_USB_Driver/usb_bsp.h"
#include "../STM32_USB_Driver/usb_regs.h"
#include "../STM32_USB_Driver/usbd_conf.h"
#include "../STM32_USB_Driver/usbd_usr.h"
#include "../STM32_USB_Driver/usbd_desc.h"
#include "../STM32_USB_Driver/usbd_cdc_core.h"
#include "../STM32_USB_Driver/usbd_ioreq.h"
#include "../STM32_USB_Driver/usbd_req.h"
#include "../STM32_USB_Driver/usbd_core.h"

#include "../STM32_USB_Driver/ring_buff.h"
#include "../STM32_USB_Driver/min_max.h"

#ifdef __cplusplus
  }
#endif



#define USBD_MANUFACTURER_STRING        "RevoMini"
#define USBD_PRODUCT_FS_STRING          "3DR Virtual COM"
#define USBD_SERIALNUMBER_FS_STRING     "00000000050C"
#define USBD_CONFIGURATION_FS_STRING    "VCP Config"
#define USBD_INTERFACE_FS_STRING        "VCP Interface"
  
#define USBD_VID                        0x26ac
#define USBD_PID                        0x0011
#define USBD_LANGID_STRING              0x409
    
#define USB_RXFIFO_SIZE 256
#define USB_TXFIFO_SIZE 256



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
	const gpio_dev *present_port;
	uint16_t present_pin;
//	char *description;
//	char *manufacturer;
//	char *serial_number;
//	char *configuration;
//	char *interface;
} usb_attr_t;

#define I_USB_CLEAR		1
#define I_USB_CONNECTED		2
#define I_USB_GETATTR		3
#define I_USB_SETATTR		4

#ifdef __cplusplus
  extern "C" {
#endif


extern USB_OTG_CORE_HANDLE           USB_OTG_dev;

/* functions */
int usb_open(void);
int usb_close(void);
int usb_ioctl(int request, void * ctl);
int usb_read(void  * buf, unsigned int nbytes);
void usb_default_attr(usb_attr_t *attr);
int usb_configure(usb_attr_t * attr);
void usb_setParams(usb_attr_t * attr);

void USB_OTG_BSP_DisableInterrupt();

void USB_OTG_BSP_mDelay (const uint32_t msec);

extern uint8_t is_usb_connected(usb_attr_t *attr);
uint8_t is_usb_opened();
void reset_usb_opened();

int usb_write(const uint8_t *buf, unsigned int nbytes);
int usb_read(void  * buf, unsigned int nbytes);
uint32_t usb_data_available(void);
uint16_t usb_tx_space(void);

void usb_reset_rx();
void usb_reset_tx();
bool usb_get_dtr();

static inline uint8_t usb_putc(uint8_t b){  return usb_write(&b, 1); }
//uint8_t usb_getc(void);
static inline uint8_t usb_getc(void){ uint8_t c;  usb_read(&c, 1);  return c; }


uint16_t usb_tx_pending(void);


static inline void usb_disconnect(){
//    (__IO USB_OTG_GREGS *)(USB_OTG_FS_BASE_ADDR + USB_OTG_CORE_GLOBAL_REGS_OFFSET)->GCCFG = 0;
//    (__IO USB_OTG_GREGS *)(USB_OTG_HS_BASE_ADDR + USB_OTG_CORE_GLOBAL_REGS_OFFSET)->GCCFG = 0;
    
//    USB_OTG_dev.regs.GREGS->GCCFG = 0;


    USB_OTG_DCTL_TypeDef dctl, dctl0;
 
//  dctl.d32 = USB_OTG_READ_REG32(&USB_OTG_FS_regs.DEV->DCTL);
    dctl.d32 = USB_OTG_dev.regs.DREGS->DCTL;

    dctl0=dctl;

    /* Disconnect device for 20ms */
    dctl.b.sftdiscon  = 1;
//  USB_OTG_WRITE_REG32(&USB_OTG_FS_regs.DEV->DCTL, dctl.d32);
    USB_OTG_dev.regs.DREGS->DCTL = dctl.d32;
    
    USB_OTG_BSP_mDelay(250);

    USB_OTG_dev.regs.DREGS->DCTL = dctl0.d32;
}

#define DEFAULT_CONFIG                  0
#define OTHER_CONFIG                    1

#ifdef __cplusplus
  }
#endif

void USB_OTG_BSP_uDelay (const uint32_t usec);


/* Get the total number of data/space bytes available */
unsigned VCP_DataAvail(void);
unsigned VCP_SpaceAvail(void);

/* Get the number of contiguous data/space bytes available */
unsigned VCP_DataAvailContig(void);
unsigned VCP_SpaceAvailContig(void);

/* Get/put data from/to contiguous area */
unsigned VCP_GetContig(void* buff, unsigned max_len);
unsigned VCP_PutContig(void const* buff, unsigned len);

/* Get/put as much as possible */
unsigned VCP_Get(void* buff, unsigned max_len);
unsigned VCP_Put(void const* buff, unsigned len);

/* Returns pointer to contiguous input data area */
static inline uint8_t const* VCP_DataPtr(void){    return USB_Rx_Buffer + USB_Rx_buff_tail; }

/* Returns pointer to contiguous output free space area */
static inline uint8_t* VCP_SpacePtr(void) {        return USB_Tx_Buffer + USB_Tx_buff_head; }

/* Mark data as read */
void VCP_MarkRead(unsigned sz);

/* Mark space as written */
void VCP_MarkWritten(unsigned sz);

int usb_periphcfg(FunctionalState state);


void VCP_SetUSBTxBlocking(uint8_t Mode);
void OTG_FS_IRQHandler(void);


/*
 * Descriptors and other paraphernalia
 */

/* Descriptor types */

#define USB_DESCRIPTOR_TYPE_DEVICE        0x01
#define USB_DESCRIPTOR_TYPE_CONFIGURATION 0x02
#define USB_DESCRIPTOR_TYPE_STRING        0x03
#define USB_DESCRIPTOR_TYPE_INTERFACE     0x04
#define USB_DESCRIPTOR_TYPE_ENDPOINT      0x05

/* Descriptor structs and declaration helpers */

#define USB_DESCRIPTOR_STRING_LEN(x) (2 + (x << 1))

#define USB_DESCRIPTOR_STRING(len)              \
  struct {                                      \
      uint8_t bLength;                            \
      uint8_t bDescriptorType;                    \
      uint16_t bString[len];                      \
  } __packed

typedef struct usb_descriptor_device {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t bcdUSB;
    uint8_t  bDeviceClass;
    uint8_t  bDeviceSubClass;
    uint8_t  bDeviceProtocol;
    uint8_t  bMaxPacketSize0;
    uint16_t idVendor;
    uint16_t idProduct;
    uint16_t bcdDevice;
    uint8_t  iManufacturer;
    uint8_t  iProduct;
    uint8_t  iSerialNumber;
    uint8_t  bNumConfigurations;
} __packed usb_descriptor_device;

typedef struct usb_descriptor_config_header {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint16_t wTotalLength;
    uint8_t  bNumInterfaces;
    uint8_t  bConfigurationValue;
    uint8_t  iConfiguration;
    uint8_t  bmAttributes;
    uint8_t  bMaxPower;
} __packed usb_descriptor_config_header;

typedef struct usb_descriptor_interface {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bInterfaceNumber;
    uint8_t bAlternateSetting;
    uint8_t bNumEndpoints;
    uint8_t bInterfaceClass;
    uint8_t bInterfaceSubClass;
    uint8_t bInterfaceProtocol;
    uint8_t iInterface;
} __packed usb_descriptor_interface;

typedef struct usb_descriptor_endpoint {
    uint8_t  bLength;
    uint8_t  bDescriptorType;
    uint8_t  bEndpointAddress;
    uint8_t  bmAttributes;
    uint16_t wMaxPacketSize;
    uint8_t  bInterval;
} __packed usb_descriptor_endpoint;

typedef struct usb_descriptor_string {
    uint8_t bLength;
    uint8_t bDescriptorType;
    uint8_t bString[];
} usb_descriptor_string;

/* Common values that go inside descriptors */

#define USB_CONFIG_ATTR_BUSPOWERED        0b10000000
#define USB_CONFIG_ATTR_SELF_POWERED      0b11000000

#define USB_EP_TYPE_INTERRUPT             0x03
#define USB_EP_TYPE_BULK                  0x02

#define USB_DESCRIPTOR_ENDPOINT_IN        0x80
#define USB_DESCRIPTOR_ENDPOINT_OUT       0x00

/*
 * USB module core
 */

#ifndef USB_ISR_MSK
/* Handle CTRM, WKUPM, SUSPM, ERRM, SOFM, ESOFM, RESETM */
#define USB_ISR_MSK 0xBF00
#endif


typedef enum usb_dev_state {
    USB_UNCONNECTED,
    USB_ATTACHED,
    USB_POWERED,
    USB_SUSPENDED,
    USB_ADDRESSED,
    USB_CONFIGURED
} usb_dev_state;
    
    
// Encapsulates global state formerly handled by usb_lib/ 
/*
typedef struct usblib_dev {
    uint32_t irq_mask;
    void (**ep_int_in)(void);
    void (**ep_int_out)(void);
    usb_dev_state state;
    usb_dev_state prevState;
    rcc_clk_id clk_id;
} usblib_dev;
*/

#endif
