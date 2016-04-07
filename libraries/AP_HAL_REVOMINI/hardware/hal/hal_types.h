#ifndef _HAL_TYPES_H_
#define _HAL_TYPES_H_

#include <stdint.h>

typedef unsigned char uint8;
typedef unsigned short uint16;
typedef unsigned int uint32;
typedef unsigned long long uint64;

typedef signed char int8;
typedef short int16;
typedef int int32;
typedef long long int64;

typedef void (*voidFuncPtr)(void);

#define __attr_flash __attribute__((section (".USER_FLASH")))
#define __packed __attribute__((__packed__))

#ifndef NULL
#define NULL 0
#endif

#endif

