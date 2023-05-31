/*
    ChibiOS - Copyright (C) 2006..2016 Giovanni Di Sirio

    Licensed under the Apache License, Version 2.0 (the "License");
    you may not use this file except in compliance with the License.
    You may obtain a copy of the License at

        http://www.apache.org/licenses/LICENSE-2.0

    Unless required by applicable law or agreed to in writing, software
    distributed under the License is distributed on an "AS IS" BASIS,
    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
    See the License for the specific language governing permissions and
    limitations under the License.
*/

#include "hal.h"
#include "hwdef.h"

#include <stdlib.h>
#include <string.h>

#include "hal_usb_msd.h"
#include "usbcfg.h"

#if HAL_HAVE_USB_CDC_MSD
/*
 * must be 64 for full speed and 512 for high speed
 */
#define USB_MSD_EP_SIZE               512U
#define USB_DATA_REQUEST_EP           2
#define USB_DATA_AVAILABLE_EP         2
#define USB_INTERRUPT_REQUEST_EP      3

SerialUSBDriver SDU1;

static cdc_linecoding_t linecoding = {
    {0x00, 0x96, 0x00, 0x00},             /* 38400.                           */
    LC_STOP_1, LC_PARITY_NONE, 8
};

/*
 * USB Device Descriptor.
 */
static const uint8_t vcom_device_descriptor_data[18] = {
  USB_DESC_DEVICE       (0x0200,        /* bcdUSB (2.0).                    */
                         0xEF,          /* bDeviceClass (MISC).         ef  */
                         0x02,          /* bDeviceSubClass.             02  */
                         0x01,          /* bDeviceProtocol.             01  */
                         0x40,          /* bMaxPacketSize.                  */
                         HAL_USB_VENDOR_ID,      /* idVendor (ST).                   */
                         HAL_USB_PRODUCT_ID,      /* idProduct.                       */
                         0x0200,        /* bcdDevice.                       */
                         1,             /* iManufacturer.                   */
                         2,             /* iProduct.                        */
                         3,             /* iSerialNumber.                   */
                         1)             /* bNumConfigurations.              */
};

/*
 * Device Descriptor wrapper.
 */
static const USBDescriptor vcom_device_descriptor = {
    sizeof vcom_device_descriptor_data,
    vcom_device_descriptor_data
};

/* Configuration Descriptor tree for a CDC.*/
static const uint8_t vcom_configuration_descriptor_data[] = {
    /* Configuration Descriptor.*/
    USB_DESC_CONFIGURATION(0x0062,        /* wTotalLength.                    */
                           0x03,          /* bNumInterfaces.                  */
                           0x01,          /* bConfigurationValue.             */
                           0,             /* iConfiguration.                  */
                           0xC0,          /* bmAttributes (self powered).     */
                           250),         /* bMaxPower (100mA).               */
    /* Interface Descriptor.*/
    USB_DESC_INTERFACE    (0x00,          /* bInterfaceNumber.                */
                           0x00,          /* bAlternateSetting.               */
                           0x02,          /* bNumEndpoints.                   */
                           0x08,          /* bInterfaceClass (Mass Storage)   */
                           0x06,          /* bInterfaceSubClass (SCSI
                                             Transparent storage class)       */
                           0x50,          /* bInterfaceProtocol (Bulk Only)   */
                           0),            /* iInterface. (none)               */
    /* Mass Storage Data In Endpoint Descriptor.*/
    USB_DESC_ENDPOINT     (USB_MSD_DATA_EP | 0x80,
                           0x02,          /* bmAttributes (Bulk).             */
                           USB_MSD_EP_SIZE,            /* wMaxPacketSize.                  */
                           0x00),         /* bInterval. 1ms                   */
    /* Mass Storage Data Out Endpoint Descriptor.*/
    USB_DESC_ENDPOINT     (USB_MSD_DATA_EP,
                           0x02,          /* bmAttributes (Bulk).             */
                           USB_MSD_EP_SIZE,            /* wMaxPacketSize.                  */
                           0x00),         /* bInterval. 1ms                   */
    // CDC
    /* IAD Descriptor */
    USB_DESC_INTERFACE_ASSOCIATION(0x01, /* bFirstInterface.                  */
                                   0x02, /* bInterfaceCount.                  */
                                   0x02, /* bFunctionClass (CDC).             */
                                   0x02, /* bFunctionSubClass.  (2)           */
                                   0x01, /* bFunctionProtocol (1)             */
                                   2),   /* iInterface.                       */
    /* Interface Descriptor.*/
    USB_DESC_INTERFACE    (0x01,          /* bInterfaceNumber.                */
                           0x00,          /* bAlternateSetting.               */
                           0x01,          /* bNumEndpoints.                   */
                           0x02,          /* bInterfaceClass (Communications
                                           Interface Class, CDC section
                                           4.2).                            */
                           0x02,          /* bInterfaceSubClass (Abstract
                                         Control Model, CDC section 4.3).   */
                           0x01,          /* bInterfaceProtocol (AT commands,
                                           CDC section 4.4).                */
                           0),            /* iInterface.                      */
    /* Header Functional Descriptor (CDC section 5.2.3).*/
    USB_DESC_BYTE         (5),            /* bLength.                         */
    USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
    USB_DESC_BYTE         (0x00),         /* bDescriptorSubtype (Header
                                           Functional Descriptor.           */
    USB_DESC_BCD          (0x0110),       /* bcdCDC.                          */
    /* Call Management Functional Descriptor. */
    USB_DESC_BYTE         (5),            /* bFunctionLength.                 */
    USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
    USB_DESC_BYTE         (0x01),         /* bDescriptorSubtype (Call Management
                                           Functional Descriptor).          */
    USB_DESC_BYTE         (0x00),         /* bmCapabilities (D0+D1).          */
    USB_DESC_BYTE         (0x02),         /* bDataInterface.                */
    /* ACM Functional Descriptor.*/
    USB_DESC_BYTE         (4),            /* bFunctionLength.                 */
    USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
    USB_DESC_BYTE         (0x02),         /* bDescriptorSubtype (Abstract
                                           Control Management Descriptor).  */
    USB_DESC_BYTE         (0x02),         /* bmCapabilities.                  */
    /* Union Functional Descriptor.*/
    USB_DESC_BYTE         (5),            /* bFunctionLength.                 */
    USB_DESC_BYTE         (0x24),         /* bDescriptorType (CS_INTERFACE).  */
    USB_DESC_BYTE         (0x06),         /* bDescriptorSubtype (Union
                                           Functional Descriptor).          */
    USB_DESC_BYTE         (0x01),         /* bMasterInterface (Communication
                                           Class Interface).                */
    USB_DESC_BYTE         (0x02),         /* bSlaveInterface0 (Data Class
                                           Interface).                      */
    /* Endpoint 5 Descriptor.*/
    USB_DESC_ENDPOINT     (USB_INTERRUPT_REQUEST_EP|0x80,
                           0x03,          /* bmAttributes (Interrupt).        */
                           0x0008,        /* wMaxPacketSize.                  */
                           0xFF),         /* bInterval.                       */
    /* Interface Descriptor.*/
    USB_DESC_INTERFACE    (0x02,          /* bInterfaceNumber.                */
                           0x00,          /* bAlternateSetting.               */
                           0x02,          /* bNumEndpoints.                   */
                           0x0A,          /* bInterfaceClass (Data Class
                                           Interface, CDC section 4.5).     */
                           0x00,          /* bInterfaceSubClass (CDC section
                                           4.6).                            */
                           0x00,          /* bInterfaceProtocol (CDC section
                                           4.7).                            */
                           0x00),         /* iInterface.                      */
    /* Endpoint 4 Descriptor.*/
    USB_DESC_ENDPOINT     (USB_DATA_AVAILABLE_EP,       /* bEndpointAddress.*/
                           0x02,          /* bmAttributes (Bulk).             */
                           0x0040,        /* wMaxPacketSize.                  */
                           0x00),         /* bInterval.                       */
    /* Endpoint 4 Descriptor.*/
    USB_DESC_ENDPOINT     (USB_DATA_REQUEST_EP|0x80,    /* bEndpointAddress.*/
                           0x02,          /* bmAttributes (Bulk).             */
                           0x0040,        /* wMaxPacketSize.                  */
                           0x00)          /* bInterval.                       */
};

/*
 * Configuration Descriptor wrapper.
 */
static const USBDescriptor vcom_configuration_descriptor = {
    sizeof vcom_configuration_descriptor_data,
    vcom_configuration_descriptor_data
};

/*
 * U.S. English language identifier.
 */
static const uint8_t vcom_string0[] = {
    USB_DESC_BYTE(4),                     /* bLength.                         */
    USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* bDescriptorType.                 */
    USB_DESC_WORD(0x0409)                 /* wLANGID (U.S. English).          */
};

/*
 * Strings wrappers array. The strings are created dynamically to
 * allow them to be setup with apj_tool
 */
static USBDescriptor vcom_strings[] = {
    {sizeof vcom_string0, vcom_string0},
    {0, NULL}, // manufacturer
    {0, NULL}, // product
    {0, NULL}, // version
};

static uint8_t vcom_buffers[3][2+2*USB_DESC_MAX_STRLEN];

/*
  dynamically allocate a USB descriptor string
 */
static void setup_usb_string(USBDescriptor *desc, const char *str, uint8_t *b)
{
    char str2[USB_DESC_MAX_STRLEN];
    string_substitute(str, str2);
    uint8_t len = strlen(str2);
    desc->ud_size = 2+2*len;
    desc->ud_string = (const uint8_t *)b;
    b[0] = USB_DESC_BYTE(desc->ud_size);
    b[1] = USB_DESC_BYTE(USB_DESCRIPTOR_STRING);
    uint8_t i;
    for (i=0; i<len; i++) {
        b[2+i*2] = str2[i];
        b[2+i*2+1] = 0;
    }
}

/*
  dynamically allocate a USB descriptor strings
 */
void setup_usb_strings(void)
{
    setup_usb_string(&vcom_strings[1], HAL_USB_STRING_MANUFACTURER, vcom_buffers[0]);
    setup_usb_string(&vcom_strings[2], HAL_USB_STRING_PRODUCT, vcom_buffers[1]);
    setup_usb_string(&vcom_strings[3], HAL_USB_STRING_SERIAL, vcom_buffers[2]);
}

/*
 * Handles the GET_DESCRIPTOR callback. All required descriptors must be
 * handled here.
 */
static const USBDescriptor *get_descriptor(USBDriver *usbp,
        uint8_t dtype,
        uint8_t dindex,
        uint16_t lang)
{

    (void)usbp;
    (void)lang;
    switch (dtype) {
    case USB_DESCRIPTOR_DEVICE:
        return &vcom_device_descriptor;
    case USB_DESCRIPTOR_CONFIGURATION:
        return &vcom_configuration_descriptor;
    case USB_DESCRIPTOR_STRING:
        if (dindex < 4) {
            return &vcom_strings[dindex];
        }
    }
    return NULL;
}


/*
    get the requested usb baudrate - 0 = none
*/
#if HAL_USE_SERIAL_USB
uint32_t get_usb_baud(uint16_t endpoint_id)
{
    if (endpoint_id == 0) {
        uint32_t rate;
        memcpy(&rate, &linecoding.dwDTERate[0], sizeof(rate));
        return rate;
    }
    return 0;
}
#endif

/**
 * @brief   IN EP1 state.
 */
static USBInEndpointState ep1instate;

/**
 * @brief   OUT EP1 state.
 */
static USBOutEndpointState ep1outstate;

/**
 * @brief   EP1 initialization structure (both IN and OUT).
 */
static const USBEndpointConfig ep1config = {
    USB_EP_MODE_TYPE_BULK,
    NULL,
    NULL,
    NULL,
    USB_MSD_EP_SIZE,
    USB_MSD_EP_SIZE,
    &ep1instate,
    &ep1outstate,
    2,
    NULL
};

/**
 * @brief   IN EP2 state.
 */

static USBInEndpointState ep2instate;

/**
 * @brief   OUT EP2 state.
 */

static USBOutEndpointState ep2outstate;

/**
 * @brief   EP2 initialization structure (both IN and OUT).
 */

static const USBEndpointConfig ep2config = {
    USB_EP_MODE_TYPE_BULK,
    NULL,
    sduDataTransmitted,
    sduDataReceived,
    0x0040,
    0x0040,
    &ep2instate,
    &ep2outstate,
    2,
    NULL
};

/**
 * @brief   IN EP3 state.
 */
static USBInEndpointState ep3instate;

/**
 * @brief   EP3 initialization structure (IN only).
 */
static const USBEndpointConfig ep3config = {
    USB_EP_MODE_TYPE_INTR,
    NULL,
    sduInterruptTransmitted,
    NULL,
    0x0010,
    0x0000,
    &ep3instate,
    NULL,
    1,
    NULL
};

/*
 * Handles the USB driver global events.
 */
static void usb_event(USBDriver *usbp, usbevent_t event)
{
    extern SerialUSBDriver SDU1;

    switch (event) {
    case USB_EVENT_ADDRESS:
        return;
    case USB_EVENT_CONFIGURED:
        chSysLockFromISR();

        /* Enables the endpoints specified into the configuration.
           Note, this callback is invoked from an ISR so I-Class functions
           must be used.*/

        usbInitEndpointI(usbp, USB_MSD_DATA_EP,          &ep1config);
        usbInitEndpointI(usbp, USB_DATA_REQUEST_EP,      &ep2config);
        usbInitEndpointI(usbp, USB_INTERRUPT_REQUEST_EP, &ep3config);

        /* Resetting the state of the CDC subsystem.*/
        sduConfigureHookI(&SDU1);

        chSysUnlockFromISR();
        return;
    case USB_EVENT_RESET:
    /* Falls into.*/
    case USB_EVENT_UNCONFIGURED:
    /* Falls into.*/
    case USB_EVENT_SUSPEND:
        chSysLockFromISR();

        /* Disconnection event on suspend.*/
        sduSuspendHookI(&SDU1);

        chSysUnlockFromISR();
        return;
    case USB_EVENT_WAKEUP:
        chSysLockFromISR();

        /* Disconnection event on suspend.*/
        sduWakeupHookI(&SDU1);

        chSysUnlockFromISR();
        return;
    case USB_EVENT_STALLED:
        return;
    }
    return;
}


#define MSD_SETUP_WORD(setup, index) (uint16_t)(((uint16_t)setup[index+1] << 8)\
                                                | (setup[index] & 0x00FF))
#define MSD_SETUP_VALUE(setup)  MSD_SETUP_WORD(setup, 2)
#define MSD_SETUP_INDEX(setup)  MSD_SETUP_WORD(setup, 4)
#define MSD_SETUP_LENGTH(setup) MSD_SETUP_WORD(setup, 6)

#define MSD_REQ_RESET                   0xFF
#define MSD_GET_MAX_LUN                 0xFE

static uint8_t zbuf = 0;
static bool hybridRequestHook(USBDriver *usbp)
{

    // handle MSD setup request -- we could change the interface here
#define USB_MSD_INTERFACE 0

    if (((usbp->setup[0] & USB_RTYPE_TYPE_MASK) == USB_RTYPE_TYPE_CLASS) &&
        ((usbp->setup[0] & USB_RTYPE_RECIPIENT_MASK) ==
         USB_RTYPE_RECIPIENT_INTERFACE)) {

        if (MSD_SETUP_INDEX(usbp->setup) == USB_MSD_INTERFACE) {

            switch (usbp->setup[1]) {
            case MSD_REQ_RESET:
                /* check that it is a HOST2DEV request */
                if (((usbp->setup[0] & USB_RTYPE_DIR_MASK) != USB_RTYPE_DIR_HOST2DEV) ||
                    (MSD_SETUP_LENGTH(usbp->setup) != 0) ||
                    (MSD_SETUP_VALUE(usbp->setup) != 0)) {
                    return false;
                }
                chSysLockFromISR();
                usbStallReceiveI(usbp, 1);
                usbStallTransmitI(usbp, 1);
                chSysUnlockFromISR();

                /* response to this request using EP0 */

                usbSetupTransfer(usbp, 0, 0, NULL);
                return true;

            case MSD_GET_MAX_LUN:
                /* check that it is a DEV2HOST request */
                if (((usbp->setup[0] & USB_RTYPE_DIR_MASK) != USB_RTYPE_DIR_DEV2HOST) ||
                    (MSD_SETUP_LENGTH(usbp->setup) != 1) ||
                    (MSD_SETUP_VALUE(usbp->setup) != 0)) {
                    return false;
                }
                // send 0 packet to indicate that we don't do LUN
                zbuf = 0;
                usbSetupTransfer(usbp, &zbuf, 1, NULL);
                return true;

            default:
                return false;
            }
        }
    }

    if ((usbp->setup[0] & USB_RTYPE_TYPE_MASK) == USB_RTYPE_TYPE_CLASS && usbp->setup[4] == 0x01 && usbp->setup[5] == 0x00) {
        switch (usbp->setup[1]) {
        case CDC_GET_LINE_CODING:
            usbSetupTransfer(usbp, (uint8_t *)&linecoding, sizeof(linecoding), NULL);
            return true;
        case CDC_SET_LINE_CODING:
            usbSetupTransfer(usbp, (uint8_t *)&linecoding, sizeof(linecoding), NULL);
            return true;
        case CDC_SET_CONTROL_LINE_STATE:
            /* Nothing to do, there are no control lines.*/
            usbSetupTransfer(usbp, NULL, 0, NULL);
            return true;
        }
    }

    return sduRequestsHook(usbp);
}


/*
 * Handles the USB driver global events.
 */
static void sof_handler(USBDriver *usbp)
{

    (void)usbp;

    osalSysLockFromISR();
    sduSOFHookI(&SDU1);
    osalSysUnlockFromISR();
}

// USB Driver configuration.

const USBConfig usbcfg = {
    usb_event,
    get_descriptor,
    hybridRequestHook,
    sof_handler
};


// Serial over USB driver configuration.

const SerialUSBConfig serusbcfg1 = {
#if STM32_USB_USE_OTG1
    &USBD1,
#else
    &USBD2,
#endif
    USB_DATA_REQUEST_EP,
    USB_DATA_AVAILABLE_EP,
    USB_INTERRUPT_REQUEST_EP
};
#endif // HAL_HAVE_USB_CDC_MSD
