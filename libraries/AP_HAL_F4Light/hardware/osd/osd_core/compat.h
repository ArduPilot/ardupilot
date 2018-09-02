#ifndef COMPAT_H
#define COMPAT_H


#include "binary.h"
#include <inttypes.h>

typedef uint8_t byte;

#ifndef PSTR
#define PSTR(x) (x)
#endif
#define PROGMEM

typedef uint8_t byte;
typedef uint8_t boolean;

typedef char prog_char;

#define INLINE __attribute__ ((always_inline)) inline
#define WEAK __attribute__((weak))

typedef const char * PGM_P;

#undef PSTR

#define PSTR(str) (str)

#define BYTE_OF(v,n) (((byte *)&(v))[n])

#define TO_STRING2(x) #x
#define TO_STRING(x) TO_STRING2(x)


static inline int max(int a, int b){
    if(a>b) return a;
    return b;
}

static inline float pgm_read_float(const void * v){
    const float * addr = (const float *)v;
    return *addr;
}

static inline uint8_t pgm_read_byte(const void * v){
    const byte * addr = (const byte *)v;
    return *addr;
}

// this can be used on Arduino's world to load an address so limiting to uint16_t cause HardFault
static inline uint32_t pgm_read_word(const void * v){
    const uint32_t * addr = (const uint32_t *)v;
    return *addr;
}


#include <AP_HAL_F4Light/AP_HAL_F4Light.h>
#include <AP_HAL_F4Light/GPIO.h>
#include <AP_HAL_F4Light/Scheduler.h>
#include <AP_HAL/utility/print_vprintf.h>

#ifndef NOINLINE
#define NOINLINE __attribute__ ((noinline))
#endif

using namespace F4Light;


#include "../osd_ns.h"

class OSDns::BetterStream : public AP_HAL::BetterStream {};

using namespace OSDns;


typedef uint32_t byte_32;
typedef int16_t byte_16;

extern void max7456_off();
extern void max7456_on();

static inline void delayMicroseconds(uint16_t us) { hal_delay_microseconds(us); }
//static inline void delay(uint16_t ms) { hal_delay(ms); } no calls to delay() allowed

#endif
