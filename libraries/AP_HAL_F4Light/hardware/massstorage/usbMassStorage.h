#ifndef USBMASSSTORAGE_H
#define	USBMASSSTORAGE_H

#include <boards.h>
#include "usb_mass_mal.h"


extern "C" {
#include <usbd_usr.h>
#include <usbd_desc.h>
#include <usb_conf.h>
#include <usbd_core.h>

#include "msc/usbd_msc_core.h"

#define USB_MASS_MAL_FAIL   -1
#define USB_MASS_MAL_SUCCESS 0


}
#endif	/* USBMASSSTORAGE_H */

