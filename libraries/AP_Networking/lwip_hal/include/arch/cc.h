#pragma once

#include <stdint.h>
#include <sys/select.h>

#ifdef __cplusplus
extern "C"
{
#endif

#define LWIP_ERRNO_STDINCLUDE	1

extern unsigned int lwip_port_rand(void);
#define LWIP_RAND() (lwip_port_rand())

typedef uint32_t sys_prot_t;

#ifdef __cplusplus
}
#endif

