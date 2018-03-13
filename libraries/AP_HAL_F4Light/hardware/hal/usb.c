/*
(c) 2017 night_ghost@ykoctpa.ru
 
*/

#include <usb.h>
#include <hal.h>
#include <systick.h>
#include <exti.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <boards.h>

// bugfixes to ST USB stack - https://github.com/olegv142/stm32tivc_usb_cdc
// register description - http://mcu.goodboard.ru/viewtopic.php?id=40


/*
    на поржать - https://kincajou.livejournal.com/4206484.html
    товарисч копает USB :)

*/


//#define USB_DEBUG

typedef uint8_t U8;
typedef uint32_t U32;
typedef uint16_t U16;


extern USB_OTG_CORE_HANDLE           USB_OTG_dev;
extern uint32_t USBD_OTG_ISR_Handler (const USB_OTG_CORE_HANDLE *pdev);

__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END;

static usb_attr_t *usb_attr;
static volatile U8 VCP_DTRHIGH = 0;
static volatile U8 VCP_RTSHIGH = 0;

enum reset_state_t {
    DTR_UNSET,
    DTR_HIGH,
    DTR_NEGEDGE,
    DTR_LOW
};

static enum reset_state_t reset_state = DTR_UNSET;

static const U16 rxfifo_size = USB_RXFIFO_SIZE;
//static const U16 txfifo_size = USB_TXFIFO_SIZE;

static ring_buffer _rxfifo IN_CCM;
//static ring_buffer _txfifo;

static ring_buffer *rxfifo = &_rxfifo;                /* Rx FIFO */
//static ring_buffer *txfifo = &_txfifo;                /* Rx FIFO */

static uint8_t rx_buf[USB_RXFIFO_SIZE]; // USB can work via DMA so no CCM!
//static uint8_t tx_buf[USB_TXFIFO_SIZE];

static U8 preempt_prio, sub_prio;
static U8 usb_ready;

static U8 UsbTXBlock = 1;


LINE_CODING linecoding = // not const!
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
static U16 VCP_DataTx   (const uint8_t* Buf, uint32_t Len);
static U16 VCP_DataRx   (uint8_t* Buf, uint32_t Len);

// direct usage from usbd_cdc_core.c
const CDC_IF_Prop_TypeDef VCP_fops = 
{
  VCP_Init,
  VCP_DeInit,
  VCP_Ctrl,
  VCP_DataTx,
  VCP_DataRx
};

const USBD_Usr_cb_TypeDef USR_cb =
{
  USBD_USR_Init,
  USBD_USR_DeviceReset,
  USBD_USR_DeviceConfigured,
  USBD_USR_DeviceSuspended,
  USBD_USR_DeviceResumed,
  USBD_USR_DeviceConnected,
  USBD_USR_DeviceDisconnected,
};



static const USBD_DEVICE USR_CDC_desc =
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
__ALIGN_BEGIN U8 const USBD_DeviceDesc[USB_SIZ_DEVICE_DESC] __ALIGN_END =
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
const __ALIGN_BEGIN U8 USBD_DeviceQualifierDesc[USB_LEN_DEV_QUALIFIER_DESC] __ALIGN_END =
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


// handler for hardware ISR
void OTG_FS_IRQHandler(void)
{
  USBD_OTG_ISR_Handler(&USB_OTG_dev);
}


/* USB Standard Device Descriptor */
const __ALIGN_BEGIN U8 USBD_LangIDDesc[USB_SIZ_STRING_LANGID] __ALIGN_END =
{
     USB_SIZ_STRING_LANGID,         
     USB_DESC_TYPE_STRING,       
     LOBYTE(USBD_LANGID_STRING),
     HIBYTE(USBD_LANGID_STRING), 
};


static U8 usb_connected =0;
static U8 usb_opened =0;

/* Return the device descriptor */
U8 * USBD_USR_DeviceDescriptor(U8 speed, U16 *length)
{
  *length = sizeof(USBD_DeviceDesc);
  return (U8 *)USBD_DeviceDesc;
}

/* return the LangID string descriptor */
U8 * USBD_USR_LangIDStrDescriptor(U8 speed, U16 *length)
{
  *length =  sizeof(USBD_LangIDDesc);  
  return (U8 *)USBD_LangIDDesc;
}



/* return the product string descriptor */
U8 * USBD_USR_ProductStrDescriptor(U8 speed, U16 *length)
{
    USBD_GetString ((U8 *)USBD_PRODUCT_FS_STRING, USBD_StrDesc, length);

    return USBD_StrDesc;
}

/* return the manufacturer string descriptor */
U8 * USBD_USR_ManufacturerStrDescriptor(U8 speed, U16 *length)
{
    USBD_GetString ((U8 *)USBD_MANUFACTURER_STRING, USBD_StrDesc, length);
    return USBD_StrDesc;
}

/* return the serial number string descriptor */
U8 *  USBD_USR_SerialStrDescriptor(U8 speed, U16 *length)
{
    USBD_GetString ((U8 *)USBD_SERIALNUMBER_FS_STRING, USBD_StrDesc, length);
    return USBD_StrDesc;
}

/* return the configuration string descriptor */
U8 * USBD_USR_ConfigStrDescriptor(U8 speed , U16 *length)
{
    USBD_GetString ((U8 *)USBD_CONFIGURATION_FS_STRING, USBD_StrDesc, length); 
    return USBD_StrDesc;
}


/* return the interface string descriptor */
U8 * USBD_USR_InterfaceStrDescriptor( U8 speed , U16 *length)
{
    USBD_GetString ((U8 *)USBD_INTERFACE_FS_STRING, USBD_StrDesc, length);
    return USBD_StrDesc;
}

void USBD_USR_Init(void)
{  
    usb_connected = 0;
    usb_opened = 0;
}

void USBD_USR_DeviceReset(uint8_t speed )
{
     switch (speed) {
     case USB_OTG_SPEED_HIGH: 
	 break;
     case USB_OTG_SPEED_FULL: 
	 break;
     default:
	 break;
     }
}

void USBD_USR_DeviceConfigured (void)
{
    usb_connected = 1;
    usb_opened = 1; // just for case
}

void USBD_USR_DeviceSuspended(void)
{
    usb_connected = 0;
    usb_opened = 0;
}


void USBD_USR_DeviceResumed(void)
{
    usb_connected = 1;
    usb_opened = 1; // just for case
}


void USBD_USR_DeviceConnected (void)
{
    usb_connected = 1;
    usb_opened = 1; // just for case
}


void USBD_USR_DeviceDisconnected (void)
{
    usb_connected = 0;
    usb_opened = 0;

    extern void usb_mass_mal_USBdisconnect();
    usb_mass_mal_USBdisconnect(); //reset connection state for mass-storage
}

/*------------------------- usb_default_attr -------------------------------*/

void usb_default_attr(usb_attr_t *attr)
{
    attr->preempt_prio = 0;
    attr->sub_prio = 0;
    attr->use_present_pin = 0;
//    attr->description = NULL;
//    attr->manufacturer = NULL;
//    attr->serial_number = NULL;
//    attr->configuration = NULL;
//    attr->interface = NULL;
}

/*--------------------------- usb_periphcfg -------------------------------*/

void USB_OTG_BSP_Init(USB_OTG_CORE_HANDLE *pdev){ // callback for hardware init
    return;
}

int usb_periphcfg(FunctionalState state)
{
    USB_OTG_BSP_DisableInterrupt(); // disable IRQ
    
    if (state == ENABLE) {

	RCC_AHB1PeriphClockCmd( RCC_AHB1Periph_GPIOA , ENABLE); // USB on GPIO_A

	/* Configure USB D-/D+ (DM/DP) pins */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12; // sometimes touching of GPIO_Pin_12 causes interrupt which will not be served
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	gpio_set_af_mode(_GPIOA, GPIO_PinSource11, GPIO_AF_OTG1_FS);
	gpio_set_af_mode(_GPIOA, GPIO_PinSource12, GPIO_AF_OTG1_FS);


	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
	RCC_AHB2PeriphClockCmd(RCC_AHB2Periph_OTG_FS, ENABLE) ;
    } else {
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
	return 0;

    usb_setParams(attr);

    // USB Device Initialize
    USBD_Init(&USB_OTG_dev, USB_OTG_FS_CORE_ID, &USR_CDC_desc, &USBD_CDC_cb, &USR_cb);

    usb_ready = 1;
	
    return 1;
}

void usb_setParams(usb_attr_t * attr)
{
    if (attr == NULL)
	return;

    if (attr->use_present_pin) {
	if (!IS_GPIO_ALL_PERIPH(attr->present_port->GPIOx) || !IS_GPIO_PIN_SOURCE(attr->present_pin))
		return;
	gpio_set_mode(attr->present_port, attr->present_pin, GPIO_INPUT_FLOATING);
    }

    preempt_prio = attr->preempt_prio;
    sub_prio = attr->sub_prio;
}




void USB_OTG_BSP_EnableInterrupt(USB_OTG_CORE_HANDLE *pdev)
{
    enable_nvic_irq(OTG_FS_IRQn, preempt_prio);
}

void USB_OTG_BSP_DisableInterrupt()
{
    NVIC_DisableIRQ(OTG_FS_IRQn);
}

void USB_OTG_BSP_mDelay (const uint32_t msec)
{
    uint32_t start = hal_micros(), ms = msec;

    while (ms > 0) {
        while ((hal_micros() - start) >= 1000) {
            ms--;
            if (ms == 0) break;
            start += 1000;
        }
    }
}









/* VCP_Init
 * Initializes the Media on the STM32
 * @param  None
 * @retval Result of the opeartion (USBD_OK in all cases)
 */
static U16 VCP_Init(void)
{
	rb_init(rxfifo, rxfifo_size, rx_buf);
//	rb_init(txfifo, txfifo_size, tx_buf);
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
  switch (Cmd) {
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
    linecoding.bitrate = (uint32_t)(Buf[0] | (Buf[1] << 8) | (Buf[2] << 16) | (Buf[3] << 24));
    linecoding.format = Buf[4];
    linecoding.paritytype = Buf[5];
    linecoding.datatype = Buf[6];
    /* Set the new configuration */
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


#define USB_CDCACM_CONTROL_LINE_DTR       (0x01)
#define USB_CDCACM_CONTROL_LINE_RTS       (0x02)


  case SET_CONTROL_LINE_STATE:
        linecoding.bitrate = (uint32_t)(Buf[0] | (Buf[1] << 8));
        
        VCP_DTRHIGH = (Buf[0] & USB_CDCACM_CONTROL_LINE_DTR);
        VCP_RTSHIGH = (Buf[0] & USB_CDCACM_CONTROL_LINE_RTS);

#ifdef DEBUG_BUILD
// { from Arduino32
// We need to see a negative edge on DTR before we start looking
    // for the in-band magic reset byte sequence.
        uint8_t dtr = VCP_DTRHIGH;
        switch (reset_state) {
        case DTR_UNSET:
            reset_state = dtr ? DTR_HIGH : DTR_LOW;
            break;
        case DTR_HIGH:
            reset_state = dtr ? DTR_HIGH : DTR_NEGEDGE;
            break;
        case DTR_NEGEDGE:
            reset_state = dtr ? DTR_HIGH : DTR_LOW;
            break;
        case DTR_LOW:
            reset_state = dtr ? DTR_HIGH : DTR_LOW;
            break;
        }
        
        // as Arduino - reset on DTR negative edge
        if ((linecoding.bitrate == 1200) && (reset_state == DTR_NEGEDGE)) {
            //    iwdg_init(IWDG_PRE_4, 10);
            NVIC_SystemReset();
            while (1);
        }
//}    
#endif
        
    break;

  case SEND_BREAK:
    /* Not  needed for this driver */
    break;    
    
  default:
    break;
  }

  usb_opened = 1; // set active state only on VCP opening
  usb_connected = 1; // just for case

  return USBD_OK;
}

/* VCP_DataTx
 *  CDC received data to be send over USB IN endpoint are managed in 
 *  this function.
 * @param  Buf: Buffer of data to be sent
 * @param  Len: Number of data to be sent (in bytes)
 * @retval Result of the opeartion: USBD_OK if all operations are OK else VCP_FAIL
 */

void VCP_MarkWritten(unsigned sz)
{
        noInterrupts();
        USB_Tx_buff_head = ring_wrap(USB_TX_BUFF_SIZE, USB_Tx_buff_head + sz);
        interrupts();
}

unsigned VCP_SpaceAvailContig(void)
{
        noInterrupts();
        unsigned sz = ring_space_contig(USB_TX_BUFF_SIZE, USB_Tx_buff_head, USB_Tx_buff_tail);
        interrupts();
        return sz;
}

unsigned VCP_PutContig(void const* buff, unsigned len)
{
        unsigned avail = VCP_SpaceAvailContig();
        unsigned sz = MIN_(avail, len);
        if (sz) {
            memcpy(VCP_SpacePtr(), buff, sz);
            VCP_MarkWritten(sz);
        }
        return sz;
}

unsigned VCP_SpaceAvail(void)
{
        noInterrupts();
        unsigned sz = ring_space_avail(USB_TX_BUFF_SIZE, USB_Tx_buff_head, USB_Tx_buff_tail);
        interrupts();
        return sz;
}

U16 VCP_DataTx(const U8 *buffer, U32 nbytes)
{

        unsigned sz = VCP_PutContig(buffer, nbytes);
        if (sz && (nbytes -= sz))
                sz += VCP_PutContig((uint8_t*)buffer + sz, nbytes);
        return sz;
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

unsigned VCP_DataAvail(void)
{
        noInterrupts();
        unsigned sz = ring_data_avail(USB_Rx_buff_size, USB_Rx_buff_head, USB_Rx_buff_tail);
        interrupts();
        return sz;
}
   

void VCP_MarkRead(unsigned sz)
{
        noInterrupts();
        USB_Rx_buff_tail = ring_wrap(USB_Rx_buff_size, USB_Rx_buff_tail + sz);
        interrupts();
}

unsigned VCP_GetContig(void* buff, unsigned max_len)
{
        unsigned avail = VCP_DataAvailContig();
        unsigned sz = MIN_(avail, max_len);
        if (sz) {
                memcpy(buff, VCP_DataPtr(), sz);
                VCP_MarkRead(sz);
        }
        return sz;
}

unsigned VCP_DataAvailContig(void)
{
        noInterrupts();
        unsigned sz = ring_data_contig(USB_Rx_buff_size, USB_Rx_buff_head, USB_Rx_buff_tail);
        interrupts();
        return sz;
}

static U16 VCP_DataRx(U8 *buffer, U32 nbytes)
{
    usb_opened = 1; // data from other side

#ifdef DEBUG_BUILD
    if(VCP_DTRHIGH) {
        if(nbytes >= 4) {
            if(buffer[0] == '1' && buffer[1] == 'E' && buffer[2] == 'A' && buffer[3] == 'F') {
                    nbytes = 0;
                    if(is_bare_metal())  // bare metal build without bootloader should reboot to DFU on this reset
                        board_set_rtc_register(DFU_RTC_SIGNATURE, RTC_SIGNATURE_REG);
                        
                    NVIC_SystemReset();
            }
        }
    }
    VCP_DTRHIGH =0;
#endif
    unsigned sz = VCP_GetContig(buffer, nbytes);
    if (sz && (nbytes -= sz))
            sz += VCP_GetContig((uint8_t*)buffer + sz, nbytes);
    return sz;
}





/*---------------------------- usb_open ---------------------------------*/

int usb_open(void)
{
	usb_periphcfg(ENABLE);	

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
	
    if (usb_ready == 1) {
        DCD_DevDisconnect(&USB_OTG_dev);
	USBD_DeInit(&USB_OTG_dev);
	USB_OTG_StopDevice(&USB_OTG_dev);
	usb_ready = 0;
    }
	
    if (usb_attr->use_present_pin){
	gpio_set_mode(usb_attr->present_port, usb_attr->present_pin, GPIO_INPUT_FLOATING);
    }
    return 1;
}

uint8_t is_usb_connected(usb_attr_t *attr)
{
	if (usb_attr == NULL || (usb_attr && usb_attr->use_present_pin == 0))
	    return usb_connected;
	else
	    return gpio_read_bit(usb_attr->present_port, usb_attr->present_pin);
}


/*---------------------------- usb_ioctl --------------------------------*/

int usb_ioctl(int request, void *ctl)
{
	switch(request) {
	case I_USB_CLEAR:
		if (!rxfifo) {
			return 0;
		}
		rb_reset(rxfifo);
		return 1;
		
	case I_USB_CONNECTED:
		*((U8 *)ctl) = is_usb_connected(usb_attr);
		break;
		
	case I_USB_GETATTR:
		if (usb_attr) {
			*((usb_attr_t **)ctl) = usb_attr;
		} else {
			return 0;
		}
		break;
	case I_USB_SETATTR:
		if (ctl == NULL) {
			return 0;
		}
		usb_attr = (usb_attr_t *)ctl;
		return usb_configure(usb_attr);
	default:
		return 0;
	}
	return 1;
}


// following functions can't be inline!
void USB_OTG_BSP_uDelay (const uint32_t usec) {   
    //stopwatch_delay_us(usec); 
    hal_delay_microseconds(usec);
}

int usb_write(const uint8_t *buf, unsigned int nbytes) {
    return VCP_DataTx(buf, nbytes); 
}

int usb_read(void  * buf, unsigned int nbytes){
     return VCP_DataRx(buf, nbytes); 
}

uint32_t usb_data_available(void){  
    return VCP_DataAvail(); 
}

bool usb_get_dtr(){
    return VCP_DTRHIGH;
}

void usb_reset_rx(){  
      rb_reset(rxfifo); 
}

void usb_reset_tx(){  
      APP_Rx_ptr_in = 0;    
}

uint16_t usb_tx_pending(void){     
    return USB_TX_BUFF_SIZE-VCP_SpaceAvail(); 
}

uint16_t usb_tx_space(void){     
    return VCP_SpaceAvail(); 
}


void VCP_SetUSBTxBlocking(uint8_t Mode) {    
    UsbTXBlock = Mode; 
}
//----------------------------------------------------------------------------

uint8_t is_usb_opened() { return usb_opened; }
void reset_usb_opened() { usb_opened=0; }
