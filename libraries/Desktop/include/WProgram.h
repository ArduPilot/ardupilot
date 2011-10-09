#ifndef WProgram_h
#define WProgram_h

#include "wiring.h"
#include <stdarg.h>
#include "HardwareSerial.h"

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <stdint.h>

typedef uint8_t byte;

void delay(unsigned long msec);
char *itoa(int __val, char *__s, int __radix);
char *ltoa(long __val, char *__s, int __radix);
char *ultoa(unsigned long __val, char *__s, int __radix);

#ifdef __cplusplus
}
#endif

#endif
