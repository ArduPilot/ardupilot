/**
  ******************************************************************************
  * @file    usbd_cdc_core.c
  * @author  MCD Application Team
  * @version V1.1.0
  * @date    19-March-2012
  * @brief   This file provides the high layer firmware functions to manage the 
  *          following functionalities of the USB CDC Class:
  *           - Initialization and Configuration of high and low layer
  *           - Enumeration as CDC Device (and enumeration for each implemented memory interface)
  *           - OUT/IN data transfer
  *           - Command IN transfer (class requests management)
  *           - Error management
  *           
  *  @verbatim
  *      
  *          ===================================================================      
  *                                CDC Class Driver Description
  *          =================================================================== 
  *           This driver manages the "Universal Serial Bus Class Definitions for Communications Devices
  *           Revision 1.2 November 16, 2007" and the sub-protocol specification of "Universal Serial Bus 
  *           Communications Class Subclass Specification for PSTN Devices Revision 1.2 February 9, 2007"
  *           This driver implements the following aspects of the specification:
  *             - Device descriptor management
  *             - Configuration descriptor management
  *             - Enumeration as CDC device with 2 data endpoints (IN and OUT) and 1 command endpoint (IN)
  *             - Requests management (as described in section 6.2 in specification)
  *             - Abstract Control Model compliant
  *             - Union Functional collection (using 1 IN endpoint for control)
  *             - Data interface class

  *           @note
  *             For the Abstract Control Model, this core allows only transmitting the requests to
  *             lower layer dispatcher (ie. usbd_cdc_vcp.c/.h) which should manage each request and
  *             perform relative actions.
  * 
  *           These aspects may be enriched or modified for a specific user application.
  *          
  *            This driver doesn't implement the following aspects of the specification 
  *            (but it is possible to manage these features with some modifications on this driver):
  *             - Any class-specific aspect relative to communication classes should be managed by user application.
  *             - All communication classes other than PSTN are not managed
  *      
  *  @endverbatim
  *                                  
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2012 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */ 

/* Includes ------------------------------------------------------------------*/
#include "usbd_cdc_core.h"
#include "usbd_desc.h"
#include "usbd_req.h"
#include "ring_buff.h"
#include "min_max.h"

/** @addtogroup STM32_USB_OTG_DEVICE_LIBRARY
  * @{
  */


/** @defgroup usbd_cdc 
  * @brief usbd core module
  * @{
  */ 

/** @defgroup usbd_cdc_Private_TypesDefinitions
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup usbd_cdc_Private_Defines
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup usbd_cdc_Private_Macros
  * @{
  */ 
/**
  * @}
  */ 


/** @defgroup usbd_cdc_Private_FunctionPrototypes
  * @{
  */

/*********************************************
   CDC Device library callbacks
 *********************************************/
static uint8_t  usbd_cdc_Init        (void *pdev, uint8_t cfgidx);
static uint8_t  usbd_cdc_DeInit      (void *pdev, uint8_t cfgidx);
static uint8_t  usbd_cdc_Setup       (void *pdev, USB_SETUP_REQ *req);
static uint8_t  usbd_cdc_EP0_RxReady (void *pdev);
static uint8_t  usbd_cdc_DataIn      (void *pdev, uint8_t epnum);
static uint8_t  usbd_cdc_DataOut     (void *pdev, uint8_t epnum);
static uint8_t  usbd_cdc_SOF         (void *pdev);

/*********************************************
   CDC specific management functions
 *********************************************/
static uint8_t  *USBD_cdc_GetCfgDesc (uint8_t speed, uint16_t *length);
#ifdef USE_USB_OTG_HS  
static uint8_t  *USBD_cdc_GetOtherCfgDesc (uint8_t speed, uint16_t *length);
#endif
/**
  * @}
  */ 

/** @defgroup usbd_cdc_Private_Variables
  * @{
  */ 
extern CDC_IF_Prop_TypeDef  APP_FOPS;
extern const uint8_t USBD_DeviceDesc   [USB_SIZ_DEVICE_DESC];

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN uint8_t const usbd_cdc_CfgDesc  [USB_CDC_CONFIG_DESC_SIZ] __ALIGN_END ;

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN uint8_t usbd_cdc_OtherCfgDesc  [USB_CDC_CONFIG_DESC_SIZ] __ALIGN_END ;

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN static __IO uint32_t  usbd_cdc_AltSet  __ALIGN_END = 0;

/* OUT ring buffer */

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN uint8_t USB_Rx_Buffer   [USB_RX_BUFF_SIZE] __ALIGN_END ;

uint32_t USB_Rx_buff_head = 0;
uint32_t USB_Rx_buff_tail = 0;
uint32_t USB_Rx_buff_size = USB_RX_BUFF_SIZE;
uint32_t USB_Rx_total_bytes = 0;

/* IN ring buffer */

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN uint8_t USB_Tx_Buffer   [USB_TX_BUFF_SIZE] __ALIGN_END ; 

uint32_t USB_Tx_buff_head = 0;
uint32_t USB_Tx_buff_tail = 0;
uint32_t USB_Tx_total_bytes = 0;

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
__ALIGN_BEGIN uint8_t CmdBuff[CDC_CMD_PACKET_SZE] __ALIGN_END ;

uint8_t USB_Tx_Active = 0; /* IN  endpoint is transmitting */
uint8_t USB_Rx_Active = 0; /* OUT endpoint is receiving */

static uint32_t cdcCmd = 0xFF;
static uint32_t cdcLen = 0;

/* CDC interface class callbacks structure */
USBD_Class_cb_TypeDef const  USBD_CDC_cb = 
{
  usbd_cdc_Init,
  usbd_cdc_DeInit,
  usbd_cdc_Setup,
  NULL,                 /* EP0_TxSent, */
  usbd_cdc_EP0_RxReady,
  usbd_cdc_DataIn,
  usbd_cdc_DataOut,
  usbd_cdc_SOF,
  NULL,
  NULL,     
  USBD_cdc_GetCfgDesc,
#ifdef USE_USB_OTG_HS   
  USBD_cdc_GetOtherCfgDesc, /* use same config as per FS */
#endif /* USE_USB_OTG_HS  */
};

#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */
/* USB CDC device Configuration Descriptor */
__ALIGN_BEGIN uint8_t const  usbd_cdc_CfgDesc[USB_CDC_CONFIG_DESC_SIZ]  __ALIGN_END =
{
  /*Configuration Descriptor*/
  0x09,   /* bLength: Configuration Descriptor size */
  USB_CONFIGURATION_DESCRIPTOR_TYPE,      /* bDescriptorType: Configuration */
  USB_CDC_CONFIG_DESC_SIZ,                /* wTotalLength:no of returned bytes */
  0x00,
  0x02,   /* bNumInterfaces: 2 interface */
  0x01,   /* bConfigurationValue: Configuration value */
  0x00,   /* iConfiguration: Index of string descriptor describing the configuration */
  0xC0,   /* bmAttributes: self powered */
  0x32,   /* MaxPower 0 mA */
  
  /*---------------------------------------------------------------------------*/
  
  /*Interface Descriptor */
  0x09,   /* bLength: Interface Descriptor size */
  USB_INTERFACE_DESCRIPTOR_TYPE,  /* bDescriptorType: Interface */
  /* Interface descriptor type */
  0x00,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x01,   /* bNumEndpoints: One endpoints used */
  0x02,   /* bInterfaceClass: Communication Interface Class */
  0x02,   /* bInterfaceSubClass: Abstract Control Model */
  0x01,   /* bInterfaceProtocol: Common AT commands */
  0x00,   /* iInterface: */
  
  /*Header Functional Descriptor*/
  0x05,   /* bLength: Endpoint Descriptor size */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x00,   /* bDescriptorSubtype: Header Func Desc */
  0x10,   /* bcdCDC: spec release number */
  0x01,
  
  /*Call Management Functional Descriptor*/
  0x05,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x01,   /* bDescriptorSubtype: Call Management Func Desc */
  0x00,   /* bmCapabilities: D0+D1 */
  0x01,   /* bDataInterface: 1 */
  
  /*ACM Functional Descriptor*/
  0x04,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
  0x02,   /* bmCapabilities */
  
  /*Union Functional Descriptor*/
  0x05,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x06,   /* bDescriptorSubtype: Union func desc */
  0x00,   /* bMasterInterface: Communication class interface */
  0x01,   /* bSlaveInterface0: Data Class Interface */
  
  /*Endpoint 2 Descriptor*/
  0x07,                           /* bLength: Endpoint Descriptor size */
  USB_ENDPOINT_DESCRIPTOR_TYPE,   /* bDescriptorType: Endpoint */
  CDC_CMD_EP,                     /* bEndpointAddress */
  0x03,                           /* bmAttributes: Interrupt */
  LOBYTE(CDC_CMD_PACKET_SZE),     /* wMaxPacketSize: */
  HIBYTE(CDC_CMD_PACKET_SZE),
#ifdef USE_USB_OTG_HS
  0x10,                           /* bInterval: */
#else
  0xFF,                           /* bInterval: */
#endif /* USE_USB_OTG_HS */
  
  /*---------------------------------------------------------------------------*/
  
  /*Data class interface descriptor*/
  0x09,   /* bLength: Endpoint Descriptor size */
  USB_INTERFACE_DESCRIPTOR_TYPE,  /* bDescriptorType: */
  0x01,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x02,   /* bNumEndpoints: Two endpoints used */
  0x0A,   /* bInterfaceClass: CDC */
  0x00,   /* bInterfaceSubClass: */
  0x00,   /* bInterfaceProtocol: */
  0x00,   /* iInterface: */
  
  /*Endpoint OUT Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_ENDPOINT_DESCRIPTOR_TYPE,      /* bDescriptorType: Endpoint */
  CDC_OUT_EP,                        /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_MAX_PACKET_SIZE),
  0x00,                              /* bInterval: ignore for Bulk transfer */
  
  /*Endpoint IN Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_ENDPOINT_DESCRIPTOR_TYPE,      /* bDescriptorType: Endpoint */
  CDC_IN_EP,                         /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  LOBYTE(CDC_DATA_MAX_PACKET_SIZE),  /* wMaxPacketSize: */
  HIBYTE(CDC_DATA_MAX_PACKET_SIZE),
  0x00                               /* bInterval: ignore for Bulk transfer */
} ;

#ifdef USE_USB_OTG_HS
#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
  #if defined ( __ICCARM__ ) /*!< IAR Compiler */
    #pragma data_alignment=4   
  #endif
#endif /* USB_OTG_HS_INTERNAL_DMA_ENABLED */ 
__ALIGN_BEGIN uint8_t const  usbd_cdc_OtherCfgDesc[USB_CDC_CONFIG_DESC_SIZ]  __ALIGN_END =
{ 
  0x09,   /* bLength: Configuation Descriptor size */
  USB_DESC_TYPE_OTHER_SPEED_CONFIGURATION,   
  USB_CDC_CONFIG_DESC_SIZ,
  0x00,
  0x02,   /* bNumInterfaces: 2 interfaces */
  0x01,   /* bConfigurationValue: */
  0x04,   /* iConfiguration: */
  0xC0,   /* bmAttributes: */
  0x32,   /* MaxPower 100 mA */  
  
  /*Interface Descriptor */
  0x09,   /* bLength: Interface Descriptor size */
  USB_INTERFACE_DESCRIPTOR_TYPE,  /* bDescriptorType: Interface */
  /* Interface descriptor type */
  0x00,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x01,   /* bNumEndpoints: One endpoints used */
  0x02,   /* bInterfaceClass: Communication Interface Class */
  0x02,   /* bInterfaceSubClass: Abstract Control Model */
  0x01,   /* bInterfaceProtocol: Common AT commands */
  0x00,   /* iInterface: */
  
  /*Header Functional Descriptor*/
  0x05,   /* bLength: Endpoint Descriptor size */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x00,   /* bDescriptorSubtype: Header Func Desc */
  0x10,   /* bcdCDC: spec release number */
  0x01,
  
  /*Call Management Functional Descriptor*/
  0x05,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x01,   /* bDescriptorSubtype: Call Management Func Desc */
  0x00,   /* bmCapabilities: D0+D1 */
  0x01,   /* bDataInterface: 1 */
  
  /*ACM Functional Descriptor*/
  0x04,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x02,   /* bDescriptorSubtype: Abstract Control Management desc */
  0x02,   /* bmCapabilities */
  
  /*Union Functional Descriptor*/
  0x05,   /* bFunctionLength */
  0x24,   /* bDescriptorType: CS_INTERFACE */
  0x06,   /* bDescriptorSubtype: Union func desc */
  0x00,   /* bMasterInterface: Communication class interface */
  0x01,   /* bSlaveInterface0: Data Class Interface */
  
  /*Endpoint 2 Descriptor*/
  0x07,                           /* bLength: Endpoint Descriptor size */
  USB_ENDPOINT_DESCRIPTOR_TYPE,   /* bDescriptorType: Endpoint */
  CDC_CMD_EP,                     /* bEndpointAddress */
  0x03,                           /* bmAttributes: Interrupt */
  LOBYTE(CDC_CMD_PACKET_SZE),     /* wMaxPacketSize: */
  HIBYTE(CDC_CMD_PACKET_SZE),
  0xFF,                           /* bInterval: */
  
  /*---------------------------------------------------------------------------*/
  
  /*Data class interface descriptor*/
  0x09,   /* bLength: Endpoint Descriptor size */
  USB_INTERFACE_DESCRIPTOR_TYPE,  /* bDescriptorType: */
  0x01,   /* bInterfaceNumber: Number of Interface */
  0x00,   /* bAlternateSetting: Alternate setting */
  0x02,   /* bNumEndpoints: Two endpoints used */
  0x0A,   /* bInterfaceClass: CDC */
  0x00,   /* bInterfaceSubClass: */
  0x00,   /* bInterfaceProtocol: */
  0x00,   /* iInterface: */
  
  /*Endpoint OUT Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_ENDPOINT_DESCRIPTOR_TYPE,      /* bDescriptorType: Endpoint */
  CDC_OUT_EP,                        /* bEndpointAddress */
  0x02,                              /* bmAttributes: Bulk */
  0x40,                              /* wMaxPacketSize: */
  0x00,
  0x00,                              /* bInterval: ignore for Bulk transfer */
  
  /*Endpoint IN Descriptor*/
  0x07,   /* bLength: Endpoint Descriptor size */
  USB_ENDPOINT_DESCRIPTOR_TYPE,     /* bDescriptorType: Endpoint */
  CDC_IN_EP,                        /* bEndpointAddress */
  0x02,                             /* bmAttributes: Bulk */
  0x40,                             /* wMaxPacketSize: */
  0x00,
  0x00                              /* bInterval */
};
#endif /* USE_USB_OTG_HS  */

/**
  * @}
  */ 

/** @defgroup usbd_cdc_Private_Functions
  * @{
  */ 

/**
  * @brief  usbd_cdc_Init
  *         Initilaize the CDC interface
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  usbd_cdc_Init (void  *pdev, 
                               uint8_t cfgidx)
{
  uint8_t *pbuf;

  /* Open EP IN */
  DCD_EP_Open(pdev,
              CDC_IN_EP,
              CDC_DATA_IN_PACKET_SIZE,
              USB_OTG_EP_BULK);
  
  /* Open EP OUT */
  DCD_EP_Open(pdev,
              CDC_OUT_EP,
              CDC_DATA_OUT_PACKET_SIZE,
              USB_OTG_EP_BULK);
  
  /* Open Command IN EP */
  DCD_EP_Open(pdev,
              CDC_CMD_EP,
              CDC_CMD_PACKET_SZE,
              USB_OTG_EP_INT);
  
  pbuf = (uint8_t *)USBD_DeviceDesc;
  pbuf[4] = DEVICE_CLASS_CDC;
  pbuf[5] = DEVICE_SUBCLASS_CDC;
  
  /* Initialize the Interface physical components */
  APP_FOPS.pIf_Init();

  USB_Rx_Active = 1;

  /* Prepare Out endpoint to receive next packet */
  DCD_EP_PrepareRx(pdev,
                   CDC_OUT_EP,
                   USB_Rx_Buffer,
                   CDC_DATA_OUT_PACKET_SIZE);
  
  return USBD_OK;
}

/**
  * @brief  usbd_cdc_Init
  *         DeInitialize the CDC layer
  * @param  pdev: device instance
  * @param  cfgidx: Configuration index
  * @retval status
  */
static uint8_t  usbd_cdc_DeInit (void  *pdev, 
                                 uint8_t cfgidx)
{
  /* Open EP IN */
  DCD_EP_Close(pdev,
              CDC_IN_EP);
  
  /* Open EP OUT */
  DCD_EP_Close(pdev,
              CDC_OUT_EP);
  
  /* Open Command IN EP */
  DCD_EP_Close(pdev,
              CDC_CMD_EP);

  /* Restore default state of the Interface physical components */
  APP_FOPS.pIf_DeInit();
  
  return USBD_OK;
}

/**
  * @brief  usbd_cdc_Setup
  *         Handle the CDC specific requests
  * @param  pdev: instance
  * @param  req: usb requests
  * @retval status
  */
static uint8_t  usbd_cdc_Setup (void  *pdev, 
                                USB_SETUP_REQ *req)
{
  uint16_t len=USB_CDC_DESC_SIZ;
  uint8_t const  *pbuf=usbd_cdc_CfgDesc + 9;
  
  switch (req->bmRequest & USB_REQ_TYPE_MASK)
  {
    /* CDC Class Requests -------------------------------*/
  case USB_REQ_TYPE_CLASS :
      /* Check if the request is a data setup packet */
      if (req->wLength)
      {
        /* Check if the request is Device-to-Host */
        if (req->bmRequest & 0x80)
        {
          /* Get the data to be sent to Host from interface layer */
          APP_FOPS.pIf_Ctrl(req->bRequest, CmdBuff, req->wLength);
          
          /* Send the data to the host */
          USBD_CtlSendData (pdev, 
                            CmdBuff,
                            req->wLength);          
        }
        else /* Host-to-Device requeset */
        {
          /* Set the value of the current command to be processed */
          cdcCmd = req->bRequest;
          cdcLen = req->wLength;
          
          /* Prepare the reception of the buffer over EP0
          Next step: the received data will be managed in usbd_cdc_EP0_TxSent() 
          function. */
          USBD_CtlPrepareRx (pdev,
                             CmdBuff,
                             req->wLength);          
        }
      }
      else /* No Data request */
      {
        /* Transfer the command to the interface layer */
        APP_FOPS.pIf_Ctrl(req->bRequest, NULL, 0);
      }
      
      return USBD_OK;
      
    default:
      USBD_CtlError (pdev, req);
      return USBD_FAIL;
    
      
      
    /* Standard Requests -------------------------------*/
  case USB_REQ_TYPE_STANDARD:
    switch (req->bRequest)
    {
    case USB_REQ_GET_DESCRIPTOR: 
      if( (req->wValue >> 8) == CDC_DESCRIPTOR_TYPE)
      {
#ifdef USB_OTG_HS_INTERNAL_DMA_ENABLED
        pbuf = usbd_cdc_Desc;   
#else
        pbuf = usbd_cdc_CfgDesc + 9 + (9 * USBD_ITF_MAX_NUM);
#endif 
        len = MIN(USB_CDC_DESC_SIZ , req->wLength);
      }
      
      USBD_CtlSendData (pdev, 
                        pbuf,
                        len);
      break;
      
    case USB_REQ_GET_INTERFACE :
      USBD_CtlSendData (pdev,
                        (uint8_t *)&usbd_cdc_AltSet,
                        1);
      break;
      
    case USB_REQ_SET_INTERFACE :
      if ((uint8_t)(req->wValue) < USBD_ITF_MAX_NUM)
      {
        usbd_cdc_AltSet = (uint8_t)(req->wValue);
      }
      else
      {
        /* Call the error management function (command will be nacked */
        USBD_CtlError (pdev, req);
      }
      break;
    }
  }
  return USBD_OK;
}

/**
  * @brief  usbd_cdc_EP0_RxReady
  *         Data received on control endpoint
  * @param  pdev: device device instance
  * @retval status
  */
static uint8_t  usbd_cdc_EP0_RxReady (void *pdev)
{ 
  if (cdcCmd != NO_CMD)
  {
    /* Process the data */
    APP_FOPS.pIf_Ctrl(cdcCmd, CmdBuff, cdcLen);
    
    /* Reset the command variable to default value */
    cdcCmd = NO_CMD;
  }
  
  return USBD_OK;
}

static inline uint32_t last_tx_packet_size(void *pdev, uint8_t epnum)
{
	return ((USB_OTG_CORE_HANDLE*)pdev)->dev.in_ep[epnum].xfer_len;
}

static inline uint32_t last_rx_data_size(void *pdev, uint8_t epnum)
{
	return ((USB_OTG_CORE_HANDLE*)pdev)->dev.out_ep[epnum].xfer_count;
}

/**
  * @brief  usbd_audio_DataIn
  *         Data sent on non-control IN endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  usbd_cdc_DataIn (void *pdev, uint8_t epnum)
{
	uint16_t USB_Tx_length;
	if (!USB_Tx_Active)
		return USBD_OK;

	USB_Tx_length = ring_data_contig(USB_TX_BUFF_SIZE, USB_Tx_buff_head, USB_Tx_buff_tail);
	if (USB_Tx_length) {
		USB_Tx_length = MIN_(USB_Tx_length, CDC_DATA_IN_PACKET_SIZE);
	} else if (last_tx_packet_size(pdev, epnum) != CDC_DATA_IN_PACKET_SIZE) {
		USB_Tx_Active = 0;
		return USBD_OK;
	}
	/* Send the available data buffer on IN endpoint or ZLP to indicate the end of data stream */
	DCD_EP_Tx(
		pdev,
		CDC_IN_EP,
		(uint8_t*)&USB_Tx_Buffer[USB_Tx_buff_tail],
		USB_Tx_length
	);
	USB_Tx_buff_tail = ring_wrap(USB_TX_BUFF_SIZE, USB_Tx_buff_tail + USB_Tx_length);
	USB_Tx_total_bytes += USB_Tx_length;
	return USBD_OK;
}

static inline int try_start_rx(void *pdev)
{
	uint32_t space_avail;
	if (USB_Rx_buff_head >= USB_Rx_buff_tail)
		USB_Rx_buff_size = USB_RX_BUFF_SIZE;

	space_avail= ring_space_contig(USB_Rx_buff_size, USB_Rx_buff_head, USB_Rx_buff_tail);
	if (space_avail < CDC_DATA_OUT_PACKET_SIZE) {
		space_avail = ring_space_wrapped(USB_Rx_buff_size, USB_Rx_buff_head, USB_Rx_buff_tail);
		if (space_avail < CDC_DATA_OUT_PACKET_SIZE) {
			if (USB_Rx_Active) {
				USB_Rx_Active = 0;
				DCD_SetEPStatus(pdev, CDC_OUT_EP, USB_OTG_EP_TX_NAK);
			}
			return 0;
		}
		USB_Rx_buff_size = USB_Rx_buff_head;
		USB_Rx_buff_head = 0;
		if (USB_Rx_buff_tail == USB_Rx_buff_size)
			USB_Rx_buff_tail = 0;
	}
	if (!USB_Rx_Active) {
		USB_Rx_Active = 1;
		DCD_SetEPStatus(pdev, CDC_OUT_EP, USB_OTG_EP_TX_VALID);
	}
	/* Prepare Out endpoint to receive next packet */
	DCD_EP_PrepareRx(
		pdev,
		CDC_OUT_EP,
		USB_Rx_Buffer + USB_Rx_buff_head,
		CDC_DATA_OUT_PACKET_SIZE
	);
	return 1;
}

/**
  * @brief  usbd_cdc_DataOut
  *         Data received on non-control Out endpoint
  * @param  pdev: device instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  usbd_cdc_DataOut (void *pdev, uint8_t epnum)
{
	uint32_t USB_Rx_Cnt = last_rx_data_size(pdev, epnum);
	USB_Rx_buff_head = ring_wrap(USB_Rx_buff_size, USB_Rx_buff_head + USB_Rx_Cnt);
	USB_Rx_total_bytes += USB_Rx_Cnt;
	try_start_rx(pdev);
	return USBD_OK;
}

static void schedule_cdc_out(void *pdev)
{
	if (!USB_Rx_Active)
		try_start_rx(pdev);
}

static void schedule_cdc_in(void *pdev)
{
	uint16_t USB_Tx_length;
	if (USB_Tx_Active)
		return;
 
	USB_Tx_length = ring_data_contig(USB_TX_BUFF_SIZE, USB_Tx_buff_head, USB_Tx_buff_tail);
	if (!USB_Tx_length)
		return;

	USB_Tx_Active = 1;
	USB_Tx_length = MIN_(USB_Tx_length, CDC_DATA_IN_PACKET_SIZE);
	/* Send the available data buffer on IN endpoint */
	DCD_EP_Tx(
		pdev,
		CDC_IN_EP,
		(uint8_t*)&USB_Tx_Buffer[USB_Tx_buff_tail],
		USB_Tx_length
	);
	USB_Tx_buff_tail = ring_wrap(USB_TX_BUFF_SIZE, USB_Tx_buff_tail + USB_Tx_length);
	USB_Tx_total_bytes += USB_Tx_length;
}

/**
  * @brief  usbd_audio_SOF
  *         Start Of Frame event management
  * @param  pdev: instance
  * @param  epnum: endpoint number
  * @retval status
  */
static uint8_t  usbd_cdc_SOF (void *pdev)
{      
  static uint32_t FrameCount = 0;
  
  if (FrameCount++ == CDC_IN_FRAME_INTERVAL)
  {
    /* Reset the frame counter */
    FrameCount = 0;
    /* Check the data to be sent through IN pipe */
    schedule_cdc_in(pdev);
    /* Check we can start receiving through OUT pipe */
    schedule_cdc_out(pdev);
  }
  
  return USBD_OK;
}

/**
  * @brief  USBD_cdc_GetCfgDesc 
  *         Return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
static uint8_t  *USBD_cdc_GetCfgDesc (uint8_t speed, uint16_t *length)
{
  *length = sizeof (usbd_cdc_CfgDesc);
  return usbd_cdc_CfgDesc;
}

/**
  * @brief  USBD_cdc_GetCfgDesc 
  *         Return configuration descriptor
  * @param  speed : current device speed
  * @param  length : pointer data length
  * @retval pointer to descriptor buffer
  */
#ifdef USE_USB_OTG_HS 
static uint8_t  *USBD_cdc_GetOtherCfgDesc (uint8_t speed, uint16_t *length)
{
  *length = sizeof (usbd_cdc_OtherCfgDesc);
  return usbd_cdc_OtherCfgDesc;
}
#endif
/**
  * @}
  */ 

/**
  * @}
  */ 

/**
  * @}
  */ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
