/*
    ChibiOS - Copyright (C) 2006-2026 Giovanni Di Sirio.

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

#include "ch.h"
#include "hal.h"
#include "usbcfg.h"

/*
 * Endpoints to be used for USBD1.
 */
#define USBD1_DATA_REQUEST_EP           1
#define USBD1_DATA_AVAILABLE_EP         3
#define USBD1_INTERRUPT_REQUEST_EP      2

#if (AVR_USB_USE_NAMED_ADDRESS_SPACES == TRUE) && defined(__FLASH)
#  undef ROMCONST
#  define ROMCONST const __flash
#endif

/* Virtual serial port over USB.*/
SerialUSBDriver SDU1;

/*
 * USB Device Descriptor.
 */
static ROMCONST uint8_t vcom_device_descriptor_data[18] = {
  USB_DESC_DEVICE       (0x0110,        /* USB specification version (1.1). */
                         0x02,          /* Device class (CDC).              */
                         0x00,          /* Device sub class.                */
                         0x00,          /* Device protocol.                 */
                         0x40,          /* Max packet size.                 */
                         0x1b4f,        /* Vendor identifier (Sparkfun).    */
                         0x9206,        /* Product identifier.              */
                         0x0200,        /* Device version.                  */
                         1,             /* Device manufacturer.             */
                         2,             /* Product description.             */
                         3,             /* Device serial number.            */
                         1)             /* Device number of configurations. */
};

/*
 * Device Descriptor wrapper.
 */
static ROMCONST USBDescriptor vcom_device_descriptor = {
  sizeof vcom_device_descriptor_data,
  vcom_device_descriptor_data
};

/* Configuration Descriptor tree for a CDC.*/
static ROMCONST uint8_t vcom_configuration_descriptor_data[67] = {
  /* Configuration Descriptor.*/
  USB_DESC_CONFIGURATION(67,            /* Total Length.                    */
                         0x02,          /* Number of Interfaces.            */
                         0x01,          /* Configuration Value.             */
                         0,             /* Configuration.                   */
                         0xC0,          /* Attributes (self powered).       */
                         50),           /* Max Power (100mA).               */
  /* Interface Descriptor.*/
  USB_DESC_INTERFACE    (0x00,          /* Interface Number.                */
                         0x00,          /* Alternate Setting.               */
                         0x01,          /* Number of Endpoints.             */
                         0x02,          /* Interface Class (Communications
                                           Interface Class, CDC section
                                           4.2).                            */
                         0x02,          /* Interface Sub Class (Abstract
                                         Control Model, CDC section 4.3).   */
                         0x01,          /* Interface Protocol (AT commands,
                                           CDC section 4.4).                */
                         0),            /* Interface.                       */
  /* Header Functional Descriptor (CDC section 5.2.3).*/
  USB_DESC_BYTE         (5),            /* Length.                          */
  USB_DESC_BYTE         (0x24),         /* Descriptor Type (CS_INTERFACE).  */
  USB_DESC_BYTE         (0x00),         /* Descriptor Subtype (Header
                                           Functional Descriptor.           */
  USB_DESC_BCD          (0x0110),       /* CDC.                             */
  /* Call Management Functional Descriptor. */
  USB_DESC_BYTE         (5),            /* Function Length.                 */
  USB_DESC_BYTE         (0x24),         /* Descriptor Type (CS_INTERFACE).  */
  USB_DESC_BYTE         (0x01),         /* Descriptor Subtype (Call Management
                                           Functional Descriptor).          */
  USB_DESC_BYTE         (0x00),         /* Capabilities (D0+D1).            */
  USB_DESC_BYTE         (0x01),         /* Data Interface.                  */
  /* ACM Functional Descriptor.*/
  USB_DESC_BYTE         (4),            /* Function Length.                 */
  USB_DESC_BYTE         (0x24),         /* Descriptor Type (CS_INTERFACE).  */
  USB_DESC_BYTE         (0x02),         /* Descriptor Subtype (Abstract
                                           Control Management Descriptor).  */
  USB_DESC_BYTE         (0x02),         /* Capabilities.                    */
  /* Union Functional Descriptor.*/
  USB_DESC_BYTE         (5),            /* Function Length.                 */
  USB_DESC_BYTE         (0x24),         /* Descriptor Type (CS_INTERFACE).  */
  USB_DESC_BYTE         (0x06),         /* Descriptor Subtype (Union
                                           Functional Descriptor).          */
  USB_DESC_BYTE         (0x00),         /* Master Interface (Communication
                                           Class Interface).                */
  USB_DESC_BYTE         (0x01),         /* Slave Interface0 (Data Class
                                           Interface).                      */
  /* Endpoint 2 Descriptor.*/
  USB_DESC_ENDPOINT     (USBD1_INTERRUPT_REQUEST_EP|0x80,
                         0x03,          /* Attributes (Interrupt).          */
                         0x0008,        /* Max Packet Size.                 */
                         0xFF),         /* Interval.                        */
  /* Interface Descriptor.*/
  USB_DESC_INTERFACE    (0x01,          /* Interface Number.                */
                         0x00,          /* Alternate Setting.               */
                         0x02,          /* Number of Endpoints.             */
                         0x0A,          /* Interface Class (Data Class
                                           Interface, CDC section 4.5).     */
                         0x00,          /* Interface SubClass (CDC section
                                           4.6).                            */
                         0x00,          /* Interface Protocol (CDC section
                                           4.7).                            */
                         0x00),         /* Interface.                       */
  /* Endpoint 3 Descriptor.*/
  USB_DESC_ENDPOINT     (USBD1_DATA_AVAILABLE_EP,       /* Endpoint Address.*/
                         0x02,          /* Attributes (Bulk).               */
                         0x0040,        /* Max Packet Size.                 */
                         0x00),         /* Interval.                        */
  /* Endpoint 1 Descriptor.*/
  USB_DESC_ENDPOINT     (USBD1_DATA_REQUEST_EP|0x80,    /* Endpoint Address.*/
                         0x02,          /* Attributes (Bulk).               */
                         0x0040,        /* Max Packet Size.                 */
                         0x00)          /* Interval.                        */
};

/*
 * Configuration Descriptor wrapper.
 */
static ROMCONST USBDescriptor vcom_configuration_descriptor = {
  sizeof vcom_configuration_descriptor_data,
  vcom_configuration_descriptor_data
};

/*
 * U.S. English language identifier.
 */
static ROMCONST uint8_t vcom_string0[] = {
  USB_DESC_BYTE(4),                     /* Length.                          */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* Descriptor Type.                 */
  USB_DESC_WORD(0x0409)                 /* LANGID (U.S. English).           */
};

/*
 * Vendor string.
 */
static ROMCONST uint8_t vcom_string1[] = {
  USB_DESC_BYTE(18),                    /* Length .                         */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* Descriptor Type.                 */
  'S', 0, 'p', 0, 'a', 0, 'r', 0, 'k', 0, 'f', 0, 'u', 0, 'n', 0,
};

/*
 * Device Description string.
 */
static ROMCONST uint8_t vcom_string2[] = {
  USB_DESC_BYTE(56),                    /* Length.                          */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* Descriptor Type.                 */
  'C', 0, 'h', 0, 'i', 0, 'b', 0, 'i', 0, 'O', 0, 'S', 0, '/', 0,
  'R', 0, 'T', 0, ' ', 0, 'V', 0, 'i', 0, 'r', 0, 't', 0, 'u', 0,
  'a', 0, 'l', 0, ' ', 0, 'C', 0, 'O', 0, 'M', 0, ' ', 0, 'P', 0,
  'o', 0, 'r', 0, 't', 0
};

/*
 * Serial Number string.
 */
static ROMCONST uint8_t vcom_string3[] = {
  USB_DESC_BYTE(8),                     /* Length.                          */
  USB_DESC_BYTE(USB_DESCRIPTOR_STRING), /* Descriptor Type.                 */
  '0' + CH_KERNEL_MAJOR, 0,
  '0' + CH_KERNEL_MINOR, 0,
  '0' + CH_KERNEL_PATCH, 0
};

/*
 * Strings wrappers array.
 */
static ROMCONST USBDescriptor vcom_strings[] = {
  {sizeof vcom_string0, vcom_string0},
  {sizeof vcom_string1, vcom_string1},
  {sizeof vcom_string2, vcom_string2},
  {sizeof vcom_string3, vcom_string3}
};

/*
 * Handles the GET_DESCRIPTOR callback. All required descriptors must be
 * Handled here.
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

/**
 * @brief   IN EP1 state.
 */
static USBInEndpointState ep1state;

/**
 * @brief   EP1 initialization structure (IN).
 */
static const USBEndpointConfig ep1config = {
  USB_EP_MODE_TYPE_BULK,
  NULL,
  sduDataTransmitted,
  NULL,
  0x0040,
  0x0000,
  &ep1state,
  NULL,
};

/**
 * @brief   INTR EP2 state.
 */
static USBInEndpointState ep2state;

/**
 * @brief   EP2 initialization structure.
 */
static const USBEndpointConfig ep2config = {
  USB_EP_MODE_TYPE_INTR,
  NULL,
  sduInterruptTransmitted,
  NULL,
  0x0010,
  0x0000,
  &ep2state,
  NULL
};

/**
 * @brief   OUT EP3 state.
 */
static USBOutEndpointState ep3state;

/**
 * @brief   EP3 initialization structure (OUT).
 */
static const USBEndpointConfig ep3config = {
  USB_EP_MODE_TYPE_BULK,
  NULL,
  NULL,
  sduDataReceived,
  0x0000,
  0x0040,
  NULL,
  &ep3state,
};

/*
 * Handles the USB driver global events.
 */
static void usb_event(USBDriver *usbp, usbevent_t event) {
  extern SerialUSBDriver SDU1;

  switch (event) {
  case USB_EVENT_ADDRESS:
    return;
  case USB_EVENT_CONFIGURED:
    chSysLockFromISR();

    /* Enables the endpoints specified into the configuration.
       Note, this callback is invoked from an ISR so I-Class functions
       Must be used.*/
    usbInitEndpointI(usbp, USBD1_DATA_REQUEST_EP, &ep1config);
    usbInitEndpointI(usbp, USBD1_DATA_AVAILABLE_EP, &ep2config);
    usbInitEndpointI(usbp, USBD1_INTERRUPT_REQUEST_EP, &ep3config);

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

    /* Connection event on wakeup.*/
    sduWakeupHookI(&SDU1);

    chSysUnlockFromISR();
    return;
  case USB_EVENT_STALLED:
    return;
  }
  return;
}

bool usb_setup_hook(USBDriver *usbp) {
  /* Override GET_DESCRIPTOR requests to return data from program memory */
  if ((usbp->setup[0] & (USB_RTYPE_RECIPIENT_MASK | USB_RTYPE_TYPE_MASK)) ==
      (USB_RTYPE_RECIPIENT_DEVICE | USB_RTYPE_TYPE_STD) &&
      usbp->setup[1] == USB_REQ_GET_DESCRIPTOR) {
    const uint8_t dtype = usbp->setup[3];
    const uint8_t dindex = usbp->setup[2];
    const AVR_USB_TX_BUF_ADDRESS_SPACE uint8_t *ddata = NULL;
    size_t dsize = 0;
    switch (dtype) {
      case USB_DESCRIPTOR_DEVICE:
        dsize = sizeof(vcom_device_descriptor_data);
        ddata = vcom_device_descriptor_data;
        break;
      case USB_DESCRIPTOR_CONFIGURATION:
        dsize = sizeof(vcom_configuration_descriptor_data);
        ddata = vcom_configuration_descriptor_data;
        break;
      case USB_DESCRIPTOR_STRING:
        if (dindex == 0) {
          dsize = sizeof(vcom_string0);
          ddata = vcom_string0;
        } else if (dindex == 1) {
          dsize = sizeof(vcom_string1);
          ddata = vcom_string1;
        } else if (dindex == 2) {
          dsize = sizeof(vcom_string2);
          ddata = vcom_string2;
        } else if (dindex == 3) {
          dsize = sizeof(vcom_string3);
          ddata = vcom_string3;
        }
        break;
    }
    if (ddata == NULL) {
      return false;
    }

    usbSetupTransfer(usbp, ddata, dsize, NULL);
    return true;
  }
  return sduRequestsHook(usbp);
}

/*
 * Handles the USB driver start of frame event.
 */
static void sof_handler(USBDriver *usbp) {
  (void)usbp;

  osalSysLockFromISR();
  sduSOFHookI(&SDU1);
  osalSysUnlockFromISR();
}

/*
 * USB driver configuration.
 */
const USBConfig usbcfg = {
  usb_event,
  get_descriptor,
  usb_setup_hook,
  sof_handler
};

/*
 * Serial over USB driver configuration.
 */
const SerialUSBConfig serusbcfg = {
  &USBD1,
  USBD1_DATA_REQUEST_EP,
  USBD1_DATA_AVAILABLE_EP,
  USBD1_INTERRUPT_REQUEST_EP
};
