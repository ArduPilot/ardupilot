#include <usb.h>
#include <hal.h>
#include <systick.h>
#include "../STM32_USB_Driver/usb_bsp.h"
#include "../STM32_USB_Driver/usb_regs.h"
#include "../STM32_USB_Driver/usbd_conf.h"
#include "../STM32_USB_Driver/usbd_usr.h"
#include "../STM32_USB_Driver/usbd_desc.h"
#include "../STM32_USB_Driver/usbd_cdc_core.h"
#include "../STM32_USB_Driver/usbd_ioreq.h"
#include "../STM32_USB_Driver/usbd_req.h"
#include "../STM32_USB_Driver/usbd_core.h"

//#define USB_DEBUG

typedef uint8_t U8;
typedef uint32_t U32;
typedef uint16_t U16;

#define USBD_MANUFACTURER_STRING        "IBEGlab"
#define USBD_PRODUCT_FS_STRING          "IBEG Virtual COM"
#define USBD_SERIALNUMBER_FS_STRING     "00000000050C"
#define USBD_CONFIGURATION_FS_STRING    "VCP Config"
#define USBD_INTERFACE_FS_STRING        "VCP Interface"

#define USBD_VID                        0x0483
#define USBD_PID                        0x5740
#define USBD_LANGID_STRING              0x409

#define USB_RXFIFO_SIZE 256
#define USB_TXFIFO_SIZE 256

extern USB_OTG_CORE_HANDLE           USB_OTG_dev;
extern uint32_t USBD_OTG_ISR_Handler (USB_OTG_CORE_HANDLE *pdev);

__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;

static usb_attr_t *usb_attr;
static U8 usb_connected;

static const U16 rxfifo_size = USB_RXFIFO_SIZE;
static const U16 txfifo_size = USB_TXFIFO_SIZE;

static ring_buffer _rxfifo;
static ring_buffer _txfifo;

static ring_buffer *rxfifo = &_rxfifo;                /* Rx FIFO */
static ring_buffer *txfifo = &_txfifo;                /* Rx FIFO */

static uint8_t rx_buf[USB_RXFIFO_SIZE];
static uint8_t tx_buf[USB_TXFIFO_SIZE];

static U8 preempt_prio, sub_prio;
static U8 usb_ready;

LINE_CODING linecoding =
  {
    115200, /* baud rate*/
    0x00,   /* stop bits-1*/
    0x00,   /* parity - none*/
    0x08    /* nb. of bits 8*/
  };
	
/* These are external variables imported from CDC core to be used for IN 
   transfer management. */
extern U8  APP_Rx_Buffer []; /* Write CDC received data in this buffer.
                                These data will be sent over USB IN endpoint
                                in the CDC core functions. */
extern U32 APP_Rx_ptr_in;    /* Increment this pointer or roll it back to
                                start address when writing received data
                                in the buffer APP_Rx_Buffer. */

/* Private function prototypes -----------------------------------------------*/
static U16 VCP_Init     (void);
static U16 VCP_DeInit   (void);
static U16 VCP_Ctrl     (uint32_t Cmd, uint8_t* Buf, uint32_t Len);
static U16 VCP_DataTx   (uint8_t* Buf, uint32_t Len);
static U16 VCP_DataRx   (uint8_t* Buf, uint32_t Len);

CDC_IF_Prop_TypeDef VCP_fops = 
{
  VCP_Init,
  VCP_DeInit,
  VCP_Ctrl,
  VCP_DataTx,
  VCP_DataRx
};

USBD_Usr_cb_TypeDef USR_cb =
{
  USBD_USR_Init,
  USBD_USR_DeviceReset,
  USBD_USR_DeviceConfigured,
  USBD_USR_DeviceSuspended,
  USBD_USR_DeviceResumed,
  USBD_USR_DeviceConnected,
  USBD_USR_DeviceDisconnected,
};


USBD_DEVICE USR_desc =
{
  USBD_USR_DeviceDescriptor,
  USBD_USR_LangIDStrDescriptor, 
  USBD_USR_ManufacturerStrDescriptor,
  USBD_USR_ProductStrDescriptor,
  USBD_USR_SerialStrDescriptor,
  USBD_USR_ConfigStrDescriptor,
  USBD_USR_InterfaceStrDescriptor,
  
};

/* USB Standard Device Descriptor */
__ALIGN_BEGIN U8 USBD_DeviceDesc[USB_SIZ_DEVICE_DESC] __ALIGN_END =
  {
    0x12,                       /*bLength */
    USB_DEVICE_DESCRIPTOR_TYPE, /*bDescriptorType*/
    0x00,                       /*bcdUSB */
    0x02,
    0x00,                       /*bDeviceClass*/
    0x00,                       /*bDeviceSubClass*/
    0x00,                       /*bDeviceProtocol*/
    USB_OTG_MAX_EP0_SIZE,      /*bMaxPacketSize*/
    LOBYTE(USBD_VID),           /*idVendor*/
    HIBYTE(USBD_VID),           /*idVendor*/
    LOBYTE(USBD_PID),           /*idVendor*/
    HIBYTE(USBD_PID),           /*idVendor*/
    0x00,                       /*bcdDevice rel. 2.00*/
    0x02,
    USBD_IDX_MFC_STR,           /*Index of manufacturer  string*/
    USBD_IDX_PRODUCT_STR,       /*Index of product string*/
    USBD_IDX_SERIAL_STR,        /*Index of serial number string*/
    USBD_CFG_MAX_NUM            /*bNumConfigurations*/
  } ; /* USB_DeviceDescriptor */

/* USB Standard Device Descriptor */
__ALIGN_BEGIN U8 USBD_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
{
  USB_LEN_DEV_QUALIFIER_DESC,
  USB_DESC_TYPE_DEVICE_QUALIFIER,
  0x00,
  0x02,
  0x00,
  0x00,
  0x00,
  0x40,
  0x01,
  0x00,
};

/* USB Standard Device Descriptor */
__ALIGN_BEGIN U8 USBD_LangIDDesc[USB_SIZ_STRING_LANGID] __ALIGN_END =
{
     USB_SIZ_STRING_LANGID,         
     USB_DESC_TYPE_STRING,       
     LOBYTE(USBD_LANGID_STRING),
     HIBYTE(USBD_LANGID_STRING), 
};


/* Return the device descriptor */
U8 * USBD_USR_DeviceDescriptor(U8 speed, U16 *length)
{
  *length = sizeof(USBD_DeviceDesc);
  return USBD_DeviceDesc;
}

/* return the LangID string descriptor */
U8 * USBD_USR_LangIDStrDescriptor(U8 speed, U16 *length)
{
  *length =  sizeof(USBD_LangIDDesc);  
  return USBD_LangIDDesc;
}



/* return the product string descriptor */
U8 * USBD_USR_ProductStrDescriptor(U8 speed, U16 *length)
{
#ifdef USB_DEBUG
	usart_putstr(_USART1, "desc\n");
#endif
	if (usb_attr && usb_attr->description)
		USBD_GetString ((U8 *)usb_attr->description, USBD_StrDesc, length);
	else
		USBD_GetString ((U8 *)USBD_PRODUCT_FS_STRING, USBD_StrDesc, length);
#ifdef USB_DEBUG
	usart_putstr(_USART1, "desc ok\n");
#endif

return USBD_StrDesc;
}

/* return the manufacturer string descriptor */
U8 * USBD_USR_ManufacturerStrDescriptor(U8 speed, U16 *length)
{
#ifdef USB_DEBUG
	usart_putstr(_USART1, "manu\n");
#endif
	if (usb_attr && usb_attr->manufacturer)
		USBD_GetString ((U8 *)usb_attr->manufacturer, USBD_StrDesc, length);
	else
		USBD_GetString ((U8 *)USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
#ifdef USB_DEBUG
	usart_putstr(_USART1, "manu ok\n");
#endif
return USBD_StrDesc;
}

/* return the serial number string descriptor */
U8 *  USBD_USR_SerialStrDescriptor(U8 speed, U16 *length)
{
#ifdef USB_DEBUG
	usart_putstr(_USART1, "serial\n");
#endif
	if (usb_attr && usb_attr->serial_number)
		USBD_GetString ((U8 *)usb_attr->serial_number, USBD_StrDesc, length);
	else
		USBD_GetString ((U8 *)USBD_SERIALNUMBER_FS_STRING, USBD_StrDesc, length);
#ifdef USB_DEBUG
	usart_putstr(_USART1, "serial ok\n");
#endif
return USBD_StrDesc;
}

/* return the configuration string descriptor */
U8 * USBD_USR_ConfigStrDescriptor(U8 speed , U16 *length)
{
#ifdef USB_DEBUG
    usart_putstr(_USART1, "configuration\n");
#endif
    if (usb_attr && usb_attr->configuration)
		USBD_GetString ((U8 *)usb_attr->configuration, USBD_StrDesc, length);
	else	
		USBD_GetString ((U8 *)USBD_CONFIGURATION_FS_STRING, USBD_StrDesc, length); 
#ifdef USB_DEBUG
    usart_putstr(_USART1, "configuration ok\n");
#endif
    return USBD_StrDesc;
}


/* return the interface string descriptor */
U8 * USBD_USR_InterfaceStrDescriptor( U8 speed , U16 *length)
{
#ifdef USB_DEBUG
	usart_putstr(_USART1, "interface\n");
#endif
	if (usb_attr && usb_attr->interface)
		USBD_GetString ((U8 *)usb_attr->interface, USBD_StrDesc, length);
	else	
		USBD_GetString ((U8 *)USBD_INTERFACE_FS_STRING, USBD_StrDesc, length);
#ifdef USB_DEBUG
	usart_putstr(_USART1, "interface ok\n");
#endif
return USBD_StrDesc;
}

void USBD_USR_Init(void)
{  
#ifdef USB_DEBUG
    usart_putstr(_USART1, "USBD_USR_Init(void)\n");
#endif
    usb_connected = 0;
}

void USBD_USR_DeviceReset(uint8_t speed )
{
	 switch (speed)
	 {
		 case USB_OTG_SPEED_HIGH: 
			 break;
		 case USB_OTG_SPEED_FULL: 
			 break;
		 default:
			 break;
	 }
#ifdef USB_DEBUG
	 usart_putstr(_USART1, "USBD_USR_DeviceReset\n");
#endif
}

void USBD_USR_DeviceConfigured (void)
{
#ifdef USB_DEBUG
	usart_putstr(_USART1, "USBD_USR_DeviceConfigured\n");
#endif
	usb_connected = 1;
}

void USBD_USR_DeviceSuspended(void)
{
#ifdef USB_DEBUG
	usart_putstr(_USART1, "USBD_USR_DeviceSuspended\n");
#endif
	usb_connected = 0;
}


void USBD_USR_DeviceResumed(void)
{
    usb_connected = 1;
}


void USBD_USR_DeviceConnected (void)
{
    usb_connected = 1;
}


void USBD_USR_DeviceDisconnected (void)
{
    usb_connected = 0;
}

/*------------------------- usb_default_attr -------------------------------*/

void usb_default_attr(usb_attr_t *attr)
{
    attr->preempt_prio = 0;
    attr->sub_prio = 0;
	attr->use_present_pin = 0;
	attr->description = NULL;
	attr->manufacturer = NULL;
	attr->serial_number = NULL;
	attr->configuration = NULL;
	attr->interface = NULL;
}

/*--------------------------- usb_periphcfg -------------------------------*/

int usb_periphcfg(FunctionalState state)
{
	if (state == ENABLE)
	{

		RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA , ENABLE);

		/* Configure USB D-/D+ (DM/DP) pins */
		GPIO_InitTypeDef GPIO_InitStructure;
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
		GPIO_Init(GPIOA, &GPIO_InitStructure);

		GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_OTG1_FS);
		GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_OTG1_FS);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
		RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_OTG_FS, ENABLE) ;

		/*
		gpio_set_af_mode(DM_PIN_PORT, DM_PIN_PIN, USB_AF);
		gpio_set_mode(DM_PIN_PORT, DM_PIN_PIN, GPIO_AF_OUTPUT_PP);
		gpio_set_af_mode(DP_PIN_PORT, DP_PIN_PIN, USB_AF);
		gpio_set_mode(DP_PIN_PORT, DP_PIN_PIN, GPIO_AF_OUTPUT_PP);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
		*/
	}
	else
	{
		gpio_set_mode(DM_PIN_PORT, DM_PIN_PIN, GPIO_INPUT_FLOATING);
		gpio_set_mode(DP_PIN_PORT, DP_PIN_PIN, GPIO_INPUT_FLOATING);
		// we should not disable sysfg clock..
	}
 
	RCC_AHB2PeriphClockCmd(USB_CLOCK, state);
	
	return 1;
}

/*--------------------------- usb_configure -------------------------------*/

int usb_configure(usb_attr_t * attr)
{
	if (attr == NULL)
	{
		return 0;
	}

	if (attr->use_present_pin)
	{
			if (!IS_GPIO_ALL_PERIPH(attr->present_port->GPIOx) || !IS_GPIO_PIN_SOURCE(attr->present_pin))
			{
				return 0;
			}
			gpio_set_mode(attr->present_port, attr->present_pin, GPIO_INPUT_FLOATING);
	}

	preempt_prio = attr->preempt_prio;
	sub_prio = attr->sub_prio;

	// USB Device Initialize
	USBD_Init(&USB_OTG_dev,
            USB_OTG_FS_CORE_ID,
            &USR_desc, 
            &USBD_CDC_cb, 
            &USR_cb);

	usb_ready = 1;
	
	return 1;
}

void USB_OTG_BSP_Init(USB_OTG_CORE_HANDLE *pdev)
{
	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA , ENABLE);

	/* Configure USB D-/D+ (DM/DP) pins */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);


	GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_OTG1_FS);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_OTG1_FS);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_OTG_FS, ENABLE) ;
/*
	usart_putstr(_USART1, "usb_periphcfg\n");
 	 // Configure USB D-/D+ (DM/DP) pins 
	 int ret = usb_periphcfg(ENABLE);

	 if (ret != 1)
	 {
		 assert_param(ret == 1);
		 usart_putstr(_USART1, "usb_periphcfg failed\n");
	 }
	 usart_putstr(_USART1, "usb_periphcfg ok\n");
*/
}

void USB_OTG_BSP_uDelay (const uint32_t usec)
{
	delay_us(usec);
}


static void delay_ms(unsigned long ms) {
    uint32 i;
    for (i = 0; i < ms; i++) {
    	delay_us(1000);
    }
}

 void USB_OTG_BSP_mDelay (const uint32_t msec)
 {
	 delay_ms(msec);
 }

/*
 void USB_OTG_BSP_uDelay (const uint32_t usec)
 {

 	uint32_t count = 0;
 	const uint32_t utime = (120 * usec / 7);
 	do
 	{
 		if ( ++count > utime )
 		{
 			return ;
 		}
 	}
 	while (1);
  }


  void USB_OTG_BSP_mDelay (const uint32_t msec)
  {
 	 USB_OTG_BSP_uDelay(msec * 1000);
  }
*/

void USB_OTG_BSP_EnableInterrupt(USB_OTG_CORE_HANDLE *pdev)
{
	NVIC_InitTypeDef NVIC_InitStructure;
	NVIC_InitStructure.NVIC_IRQChannel = OTG_FS_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = preempt_prio;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = sub_prio;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/* VCP_Init
 * Initializes the Media on the STM32
 * @param  None
 * @retval Result of the opeartion (USBD_OK in all cases)
 */
static U16 VCP_Init(void)
{
	rb_init(rxfifo, rxfifo_size, rx_buf);
	rb_init(txfifo, txfifo_size, tx_buf);
	return USBD_OK;
}

/* VCP_DeInit
 * DeInitializes the Media on the STM32
 * @param  None
 * @retval Result of the opeartion (USBD_OK in all cases)
 */
static U16 VCP_DeInit(void)
{
	return USBD_OK;
}


/* VCP_Ctrl
 * Manage the CDC class requests
 * @param  Cmd: Command code            
 * @param  Buf: Buffer containing command data (request parameters)
 * @param  Len: Number of data to be sent (in bytes)
 * @retval Result of the opeartion (USBD_OK in all cases)
 */
static U16 VCP_Ctrl (U32 Cmd, U8 *Buf, U32 Len)
{ 
  switch (Cmd)
  {
  case SEND_ENCAPSULATED_COMMAND:
    /* Not  needed for this driver */
    break;

  case GET_ENCAPSULATED_RESPONSE:
    /* Not  needed for this driver */
    break;

  case SET_COMM_FEATURE:
    /* Not  needed for this driver */
    break;

  case GET_COMM_FEATURE:
    /* Not  needed for this driver */
    break;

  case CLEAR_COMM_FEATURE:
    /* Not  needed for this driver */
    break;

  case SET_LINE_CODING:
     /* Not  needed for this driver */
    break;

  case GET_LINE_CODING:
		Buf[0] = (uint8_t)(linecoding.bitrate);
		Buf[1] = (uint8_t)(linecoding.bitrate >> 8);
		Buf[2] = (uint8_t)(linecoding.bitrate >> 16);
		Buf[3] = (uint8_t)(linecoding.bitrate >> 24);
		Buf[4] = linecoding.format;
		Buf[5] = linecoding.paritytype;
		Buf[6] = linecoding.datatype;
		break;

  case SET_CONTROL_LINE_STATE:
    /* Not  needed for this driver */
    break;

  case SEND_BREAK:
    /* Not  needed for this driver */
    break;    
    
  default:
    break;
  }

  return USBD_OK;
}

/* VCP_DataTx
 *  CDC received data to be send over USB IN endpoint are managed in 
 *  this function.
 * @param  Buf: Buffer of data to be sent
 * @param  Len: Number of data to be sent (in bytes)
 * @retval Result of the opeartion: USBD_OK if all operations are OK else VCP_FAIL
 */
U16 VCP_DataTx(U8 *buffer, U32 nbytes)
{
	U32 sent = 0;

	while(sent < nbytes)
	{	
			APP_Rx_Buffer[APP_Rx_ptr_in] = *(buffer + sent);
			APP_Rx_ptr_in++;
			sent++;
			// To avoid buffer overflow 
			if(APP_Rx_ptr_in == APP_RX_DATA_SIZE)
			{
				APP_Rx_ptr_in = 0;
				// If the buffer is full set OVERRUN line event and ignore new bytes.
			}		
	}
	return sent;
}

/* VCP_DataRx
 * Data received over USB OUT endpoint are sent over CDC interface 
 * through this function.
 *           
 * Note
 * This function will block any OUT packet reception on USB endpoint 
 * untill exiting this function. If you exit this function before transfer
 * is complete on CDC interface (ie. using DMA controller) it will result 
 * in receiving more data while previous ones are still not sent.
 *                 
 * @param  Buf: Buffer of data to be received
 * @param  Len: Number of data received (in bytes)
 * @retval Result of the opeartion: USBD_OK if all operations are OK else VCP_FAIL
  */
static U16 VCP_DataRx(U8 *buffer, U32 nbytes)
{
	if (!rxfifo) return 0;
	U32 sent = 0;
	U32 tosend = nbytes;
	while(tosend)
	{
		if (rb_is_full(rxfifo))
			return sent;
		rb_insert(rxfifo, *buffer++);
		sent++;
		tosend--;
	}
	return sent;
}

void OTG_FS_IRQHandler(void)
{
	USBD_OTG_ISR_Handler(&USB_OTG_dev);
}


/*---------------------------- usb_open ---------------------------------*/

int usb_open(void)
{
	usb_connected = 0;
	preempt_prio = 0;
	sub_prio = 0;
	usb_ready = 0;
	
	return 1;
}

/*---------------------------- usb_close --------------------------------*/

int usb_close(void)
{	
	usb_periphcfg(DISABLE);	
	
	if (usb_ready == 1)
	{
		DCD_DevDisconnect(&USB_OTG_dev);
		USBD_DeInit(&USB_OTG_dev);
		USB_OTG_StopDevice(&USB_OTG_dev);
		usb_ready = 0;
	}
	
	if (usb_attr->use_present_pin)
		gpio_set_mode(usb_attr->present_port, usb_attr->present_pin, GPIO_INPUT_FLOATING);
	
	return 1;
}

uint8_t is_usb_connected(usb_attr_t *attr)
{
	if (usb_attr == NULL || (usb_attr && usb_attr->use_present_pin == 0))
		return usb_connected;
	else
	{
		return gpio_read_bit(usb_attr->present_port, usb_attr->present_pin);
	}
}


/*---------------------------- usb_ioctl --------------------------------*/

int usb_ioctl(int request, void *ctl)
{
	switch(request)
	{
		case I_USB_CLEAR:
			if (!rxfifo)
			{
				return 0;
			}
			rb_reset(rxfifo);
			return 1;
		case I_USB_CONNECTED:
			*((U8 *)ctl) = is_usb_connected(usb_attr);
			break;
		case I_USB_GETATTR:
			if (usb_attr)
			{
				*((usb_attr_t **)ctl) = usb_attr;
			}
			else
			{
				return 0;
			}
			break;
		case I_USB_SETATTR:
			if (ctl == NULL)
			{
				return 0;
			}
			usb_attr = (usb_attr_t *)ctl;
			return usb_configure(usb_attr);
		default:
			return 0;
	}
	return 1;
}


/*---------------------------- usb_write --------------------------------*/

int usb_write(uint8_t *buf, unsigned int nbytes)
{
    U16 tosend = nbytes;
	U16 sent = 0;
	U8 *buffer8 = (U8 *)buf;
	int bytesput;
	
	if (usb_ready == 0)
	{
		return 0;
	}
		
	sent = 0;

	//if (!txfifo)
	//    {
	//	return 0;
	//    }

	//char *str = (char *)buf;
	//usart_putstr(_USART1, str);
	// blocking mode
	while(tosend)
	{					
		bytesput = VCP_DataTx(buffer8,nbytes);
		tosend -= bytesput;
		buffer8 += bytesput;
		sent += bytesput;
	}
	return sent;
}

/*---------------------------- usb_read ---------------------------------*/

int usb_read(void  * buf, unsigned int nbytes)
{
	U16 toread = nbytes;
	U16 received;
	U8 *buffer8 = (U8 *)buf;

	if (usb_ready == 0)
	{
		return 0;
	}
	
	received = 0;
	
	if (!rxfifo) 
	{
		return 0;
	}
		
	// blocking mode
	while(toread)
	{
		/* get characters from ring buffer */
		if (!rb_is_empty(rxfifo))
		{
			*buffer8 = rb_remove(rxfifo);
			received++;
			buffer8++;
			toread--;
		}
	}

	return received;
}

void usb_reset_rx()
{
	rb_reset(rxfifo);
}

void usb_reset_tx()
{
	APP_Rx_ptr_in = 0;
	rb_reset(txfifo);
}

void usb_putc(uint8_t byte)
{
	usb_write(&byte, 1);
}

int8_t usb_getc(void)
{
    return rb_remove(rxfifo);
}

uint32_t usb_data_available(void)
{
    return rb_full_count(rxfifo);
}
uint16_t usb_tx_pending(void)
{
    return rb_full_count(txfifo);
}
/*----------------------------------------------------------------------------
 * end of file
 *---------------------------------------------------------------------------*/
