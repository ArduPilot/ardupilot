/*
    ChibiOS - Copyright (C) 2006..2015 Giovanni Di Sirio
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
/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Modified for use in AP_HAL by Andrew Tridgell and Siddharth Bharat Purohit
 */
#include "hal.h"
#include "hwdef.h"

#include <stdlib.h>
#include <string.h>

#include "usbcfg.h"

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(_arr) (sizeof(_arr) / sizeof(_arr[0]))
#endif

#if defined(HAL_USB_PRODUCT_ID) && HAL_HAVE_DUAL_USB_CDC
/*
 * Virtual serial ports over USB.
 */
SerialUSBDriver SDU1;
SerialUSBDriver SDU2;

static cdc_linecoding_t linecoding[] = {
  {{0x00, 0x96, 0x00, 0x00},             /* 38400.                           */
  LC_STOP_1, LC_PARITY_NONE, 8},
  {{0x00, 0x96, 0x00, 0x00},             /* 38400.                           */
  LC_STOP_1, LC_PARITY_NONE, 8}
};

static uint8_t ep_index[] = {0, 2};

/*
 * Endpoints.
 */
#define USB_INTERRUPT_REQUEST_EP_A      1
#define USB_DATA_AVAILABLE_EP_A         2
#define USB_DATA_REQUEST_EP_A           2
#define USB_INTERRUPT_REQUEST_EP_B      3
#define USB_DATA_AVAILABLE_EP_B         4
#define USB_DATA_REQUEST_EP_B           4

#define USB_INTERRUPT_REQUEST_SIZE      0x10
#define USB_DATA_SIZE                   0x40

/*
 * Interfaces
 */
#define USB_NUM_INTERFACES              4
#define USB_CDC_CIF_NUM0                0
#define USB_CDC_DIF_NUM0                1
#define USB_CDC_CIF_NUM1                2
#define USB_CDC_DIF_NUM1                3

/*
 * USB Device Descriptor.
 */
static const uint8_t vcom_device_descriptor_data[] = {
  USB_DESC_DEVICE(
    0x0200,                                 /* bcdUSB (1.1).                */
    0xEF,                                   /* bDeviceClass (misc).         */
    0x02,                                   /* bDeviceSubClass (common).    */
    0x01,                                   /* bDeviceProtocol (IAD).       */
    USB_DATA_SIZE,                          /* bMaxPacketSize.              */
    HAL_USB_VENDOR_ID,                      /* idVendor.                    */
    HAL_USB_PRODUCT_ID,                     /* idProduct.                   */
    0x0200,                                 /* bcdDevice.                   */
    1,                                      /* iManufacturer.               */
    2,                                      /* iProduct.                    */
    3,                                      /* iSerialNumber.               */
    1)                                      /* bNumConfigurations.          */
};

/*
 * Device Descriptor wrapper.
 */
static const USBDescriptor vcom_device_descriptor = {
  sizeof vcom_device_descriptor_data,
  vcom_device_descriptor_data
};

#define CDC_IF_DESC_SET_SIZE                                                \
  (USB_DESC_INTERFACE_SIZE + 5 + 5 + 4 + 5 + USB_DESC_ENDPOINT_SIZE +       \
   USB_DESC_INTERFACE_SIZE + (USB_DESC_ENDPOINT_SIZE * 2))

#define CDC_IF_DESC_SET(comIfNum, datIfNum, comInEp, datOutEp, datInEp)     \
  /* Interface Descriptor.*/                                                \
  USB_DESC_INTERFACE(                                                       \
    comIfNum,                               /* bInterfaceNumber.        */  \
    0x00,                                   /* bAlternateSetting.       */  \
    0x01,                                   /* bNumEndpoints.           */  \
    CDC_COMMUNICATION_INTERFACE_CLASS,      /* bInterfaceClass.         */  \
    CDC_ABSTRACT_CONTROL_MODEL,             /* bInterfaceSubClass.      */  \
    0x01,                                   /* bInterfaceProtocol (AT
                                               commands, CDC section
                                               4.4).                    */  \
    0),                                     /* iInterface.              */  \
  /* Header Functional Descriptor (CDC section 5.2.3).*/                    \
  USB_DESC_BYTE     (5),                    /* bLength.                 */  \
  USB_DESC_BYTE     (CDC_CS_INTERFACE),     /* bDescriptorType.         */  \
  USB_DESC_BYTE     (CDC_HEADER),           /* bDescriptorSubtype.      */  \
  USB_DESC_BCD      (0x0110),               /* bcdCDC.                  */  \
  /* Call Management Functional Descriptor.*/                               \
  USB_DESC_BYTE     (5),                    /* bFunctionLength.         */  \
  USB_DESC_BYTE     (CDC_CS_INTERFACE),     /* bDescriptorType.         */  \
  USB_DESC_BYTE     (CDC_CALL_MANAGEMENT),  /* bDescriptorSubtype.      */  \
  USB_DESC_BYTE     (0x03),    /*******/    /* bmCapabilities.          */  \
  USB_DESC_BYTE     (datIfNum),             /* bDataInterface.          */  \
  /* Abstract Control Management Functional Descriptor.*/                   \
  USB_DESC_BYTE     (4),                    /* bFunctionLength.         */  \
  USB_DESC_BYTE     (CDC_CS_INTERFACE),     /* bDescriptorType.         */  \
  USB_DESC_BYTE     (CDC_ABSTRACT_CONTROL_MANAGEMENT),                      \
  USB_DESC_BYTE     (0x02),                 /* bmCapabilities.          */  \
  /* Union Functional Descriptor.*/                                         \
  USB_DESC_BYTE     (5),                    /* bFunctionLength.         */  \
  USB_DESC_BYTE     (CDC_CS_INTERFACE),     /* bDescriptorType.         */  \
  USB_DESC_BYTE     (CDC_UNION),            /* bDescriptorSubtype.      */  \
  USB_DESC_BYTE     (comIfNum),             /* bMasterInterface.        */  \
  USB_DESC_BYTE     (datIfNum),             /* bSlaveInterface.         */  \
  /* Endpoint, Interrupt IN.*/                                              \
  USB_DESC_ENDPOINT (                                                       \
    comInEp,                                                                \
    USB_EP_MODE_TYPE_INTR,                  /* bmAttributes.            */  \
    USB_INTERRUPT_REQUEST_SIZE,             /* wMaxPacketSize.          */  \
    0x01),                                  /* bInterval.               */  \
                                                                            \
  /* CDC Data Interface Descriptor.*/                                       \
  USB_DESC_INTERFACE(                                                       \
    datIfNum,                               /* bInterfaceNumber.        */  \
    0x00,                                   /* bAlternateSetting.       */  \
    0x02,                                   /* bNumEndpoints.           */  \
    CDC_DATA_INTERFACE_CLASS,               /* bInterfaceClass.         */  \
    0x00,                                   /* bInterfaceSubClass (CDC
                                               section 4.6).            */  \
    0x00,                                   /* bInterfaceProtocol (CDC
                                               section 4.7).            */  \
    0x00),                                  /* iInterface.              */  \
  /* Endpoint, Bulk OUT.*/                                                  \
  USB_DESC_ENDPOINT(                                                        \
    datOutEp,                               /* bEndpointAddress.        */  \
    USB_EP_MODE_TYPE_BULK,                  /* bmAttributes.            */  \
    USB_DATA_SIZE,                          /* wMaxPacketSize.          */  \
    0x00),                                  /* bInterval.               */  \
  /* Endpoint, Bulk IN.*/                                                   \
  USB_DESC_ENDPOINT(                                                        \
    datInEp,                                /* bEndpointAddress.        */  \
    USB_EP_MODE_TYPE_BULK,                  /* bmAttributes.            */  \
    USB_DATA_SIZE,                          /* wMaxPacketSize.          */  \
    0x00)                                   /* bInterval.               */

#define IAD_CDC_IF_DESC_SET_SIZE                                            \
  (USB_DESC_INTERFACE_ASSOCIATION_SIZE + CDC_IF_DESC_SET_SIZE)

#define IAD_CDC_IF_DESC_SET(comIfNum, datIfNum, comInEp, datOutEp, datInEp) \
  /* Interface Association Descriptor.*/                                    \
  USB_DESC_INTERFACE_ASSOCIATION(                                           \
    comIfNum,                               /* bFirstInterface.         */  \
    2,                                      /* bInterfaceCount.         */  \
    CDC_COMMUNICATION_INTERFACE_CLASS,      /* bFunctionClass.          */  \
    CDC_ABSTRACT_CONTROL_MODEL,             /* bFunctionSubClass.       */  \
    1,                                      /* bFunctionProcotol.       */  \
    0                                       /* iInterface.              */  \
  ),                                                                        \
  /* CDC Interface descriptor set */                                        \
  CDC_IF_DESC_SET(comIfNum, datIfNum, comInEp, datOutEp, datInEp)

/* Configuration Descriptor tree for a CDC.*/
static const uint8_t vcom_configuration_descriptor_data[] = {
  /* Configuration Descriptor.*/
  USB_DESC_CONFIGURATION(
    USB_DESC_CONFIGURATION_SIZE +
    (IAD_CDC_IF_DESC_SET_SIZE * 2),         /* wTotalLength.                */
    USB_NUM_INTERFACES,                     /* bNumInterfaces.              */
    0x01,                                   /* bConfigurationValue.         */
    0,                                      /* iConfiguration.              */
    0xC0,                                   /* bmAttributes (self powered). */
    50                                      /* bMaxPower (100mA).           */
  ),
  IAD_CDC_IF_DESC_SET(
    USB_CDC_CIF_NUM0,
    USB_CDC_DIF_NUM0,
    USB_ENDPOINT_IN(USB_INTERRUPT_REQUEST_EP_A),
    USB_ENDPOINT_OUT(USB_DATA_AVAILABLE_EP_A),
    USB_ENDPOINT_IN(USB_DATA_REQUEST_EP_A)
  ),
  IAD_CDC_IF_DESC_SET(
    USB_CDC_CIF_NUM1,
    USB_CDC_DIF_NUM1,
    USB_ENDPOINT_IN(USB_INTERRUPT_REQUEST_EP_B),
    USB_ENDPOINT_OUT(USB_DATA_AVAILABLE_EP_B),
    USB_ENDPOINT_IN(USB_DATA_REQUEST_EP_B)
  ),
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

#define USB_DESC_MAX_STRLEN 100
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
                                           uint16_t lang) {

  (void)usbp;
  (void)lang;
  switch (dtype) {
  case USB_DESCRIPTOR_DEVICE:
    return &vcom_device_descriptor;
  case USB_DESCRIPTOR_CONFIGURATION:
    return &vcom_configuration_descriptor;
  case USB_DESCRIPTOR_STRING:
    if (dindex < 4)
      return &vcom_strings[dindex];
  }
  return NULL;
}

/*
    get the requested usb baudrate - 0 = none
*/
#if HAL_USE_SERIAL_USB
uint32_t get_usb_baud(uint16_t endpoint_id)
{
  for (uint8_t i = 0; i < ARRAY_SIZE(linecoding); i++) {
      if (endpoint_id == ep_index[i]) {
          uint32_t rate;
          memcpy(&rate, &linecoding[i].dwDTERate[0], sizeof(rate));
          return rate;
      }
  }
  return 0;
}
#endif
/**
 * @brief   IN EP1 state.
 */
static USBInEndpointState ep1instate;

/**
 * @brief   EP1 initialization structure (IN only).
 */
static const USBEndpointConfig ep1config = {
  USB_EP_MODE_TYPE_INTR,
  NULL,
  sduInterruptTransmitted,
  NULL,
  USB_INTERRUPT_REQUEST_SIZE,
  0x0000,
  &ep1instate,
  NULL,
  1,
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
  USB_DATA_SIZE,
  USB_DATA_SIZE,
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
  USB_INTERRUPT_REQUEST_SIZE,
  0x0000,
  &ep3instate,
  NULL,
  1,
  NULL
};

/**
 * @brief   IN EP4 state.
 */
static USBInEndpointState ep4instate;

/**
 * @brief   OUT EP4 state.
 */
static USBOutEndpointState ep4outstate;

/**
 * @brief   EP4 initialization structure (both IN and OUT).
 */
static const USBEndpointConfig ep4config = {
  USB_EP_MODE_TYPE_BULK,
  NULL,
  sduDataTransmitted,
  sduDataReceived,
  USB_DATA_SIZE,
  USB_DATA_SIZE,
  &ep4instate,
  &ep4outstate,
  2,
  NULL
};

/*
 * Handles the USB driver global events.
 */
static void usb_event(USBDriver *usbp, usbevent_t event) {
  extern SerialUSBDriver SDU1;
  extern SerialUSBDriver SDU2;

  switch (event) {
  case USB_EVENT_ADDRESS:
    return;
  case USB_EVENT_CONFIGURED:
    chSysLockFromISR();

    if (usbp->state == USB_ACTIVE) {
      /* Enables the endpoints specified into the configuration.
         Note, this callback is invoked from an ISR so I-Class functions
         must be used.*/
      usbInitEndpointI(usbp, USB_INTERRUPT_REQUEST_EP_A, &ep1config);
      usbInitEndpointI(usbp, USB_DATA_REQUEST_EP_A, &ep2config);
      usbInitEndpointI(usbp, USB_INTERRUPT_REQUEST_EP_B, &ep3config);
      usbInitEndpointI(usbp, USB_DATA_REQUEST_EP_B, &ep4config);

      /* Resetting the state of the CDC subsystem.*/
      sduConfigureHookI(&SDU1);
      sduConfigureHookI(&SDU2);
    }
    else if (usbp->state == USB_SELECTED) {
      usbDisableEndpointsI(usbp);
    }

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
    sduSuspendHookI(&SDU2);

    chSysUnlockFromISR();
    return;
  case USB_EVENT_WAKEUP:
    chSysLockFromISR();

    /* Connection event on wakeup.*/
    sduWakeupHookI(&SDU1);
    sduWakeupHookI(&SDU2);

    chSysUnlockFromISR();
    return;
  case USB_EVENT_STALLED:
    return;
  }
  return;
}

/*
 * Handling messages not implemented in the default handler nor in the
 * SerialUSB handler.
 */
static bool requests_hook(USBDriver *usbp) {
  if (((usbp->setup[0] & USB_RTYPE_RECIPIENT_MASK) == USB_RTYPE_RECIPIENT_INTERFACE) &&
      (usbp->setup[1] == USB_REQ_SET_INTERFACE)) {
    usbSetupTransfer(usbp, NULL, 0, NULL);
    return true;
  }
  // process only index=2 commands here - ie the second interface - the first interface is handled via the default handler
  for (uint8_t i = 0; i < ARRAY_SIZE(linecoding); i++) {
    if ((usbp->setup[0] & USB_RTYPE_TYPE_MASK) == USB_RTYPE_TYPE_CLASS && usbp->setup[4] == ep_index[i] && usbp->setup[5] == 0x00) {
      switch (usbp->setup[1]) {
      case CDC_GET_LINE_CODING:
        usbSetupTransfer(usbp, (uint8_t *)&linecoding[i], sizeof(linecoding[i]), NULL);
        return true;
      case CDC_SET_LINE_CODING:
        usbSetupTransfer(usbp, (uint8_t *)&linecoding[i], sizeof(linecoding[i]), NULL);
        return true;
      case CDC_SET_CONTROL_LINE_STATE:
        /* Nothing to do, there are no control lines.*/
        usbSetupTransfer(usbp, NULL, 0, NULL);
        return true;
      }
    }
  }

  return sduRequestsHook(usbp);
}

/*
 * Handles the USB driver global events.
 */
static void sof_handler(USBDriver *usbp) {

  (void)usbp;

  osalSysLockFromISR();
  sduSOFHookI(&SDU1);
  sduSOFHookI(&SDU2);
  osalSysUnlockFromISR();
}

/*
 * USB driver configuration.
 */
const USBConfig usbcfg = {
  usb_event,
  get_descriptor,
  requests_hook,
  sof_handler
};

/*
 * Serial over USB driver configuration 1.
 */
const SerialUSBConfig serusbcfg1 = {
#if STM32_USB_USE_OTG1
  &USBD1,
#else
  &USBD2,
#endif
  USB_DATA_REQUEST_EP_A,
  USB_DATA_AVAILABLE_EP_A,
  USB_INTERRUPT_REQUEST_EP_A
};

/*
 * Serial over USB driver configuration 2.
 */
const SerialUSBConfig serusbcfg2 = {
#if STM32_USB_USE_OTG1
  &USBD1,
#else
  &USBD2,
#endif
  USB_DATA_REQUEST_EP_B,
  USB_DATA_AVAILABLE_EP_B,
  USB_INTERRUPT_REQUEST_EP_B
};
#endif
