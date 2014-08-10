#ifndef __AP_HAL_LINUX_STORAGE_H__
#define __AP_HAL_LINUX_STORAGE_H__

#if HAL_STORAGE == USE_FRAM
#include "Storage_FRAM.h"
#else
#include "Storage_FS.h"
#endif

#endif
