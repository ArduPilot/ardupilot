
#ifndef __AP_PROGMEM_DESKTOP_BUILD_H__
#define __AP_PROGMEM_DESKTOP_BUILD_H__

#ifndef __AP_PROGMEM_H__
#error "Do not import AP_Progmem_DesktopBuild.h directly - use AP_Progmem.h"
#endif // __AP_PROGMEM_H__

/* Inherit AVR definitions for everything else */
#include "AP_Progmem_AVR.h"

#undef PROGMEM
#define PROGMEM __attribute__(())

#undef SITL_debug
#define SITL_debug(fmt, args ...)  fprintf(stdout, "%s:%u " fmt, __FUNCTION__, __LINE__, ## args)


#endif // __AP_PROGMEM_DESKTOP_BUILD_H__

