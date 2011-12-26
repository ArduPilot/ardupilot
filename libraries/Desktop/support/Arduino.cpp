#include "WProgram.h"
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include "avr/pgmspace.h"
#include <BetterStream.h>
#include <sys/time.h>
#include "desktop.h"

extern "C" {

volatile uint8_t __iomem[1024];

unsigned __brkval = 0x2000;
unsigned __bss_end = 0x1000;

void pinMode(uint8_t pin, uint8_t mode)
{

}


long unsigned int millis(void)
{
	struct timeval tp;
	gettimeofday(&tp,NULL);
	return 1.0e3*((tp.tv_sec + (tp.tv_usec*1.0e-6)) - 
		      (desktop_state.sketch_start_time.tv_sec +
		       (desktop_state.sketch_start_time.tv_usec*1.0e-6)));
}

long unsigned int micros(void)
{
	struct timeval tp;
	gettimeofday(&tp,NULL);
	return 1.0e6*((tp.tv_sec + (tp.tv_usec*1.0e-6)) - 
		      (desktop_state.sketch_start_time.tv_sec +
		       (desktop_state.sketch_start_time.tv_usec*1.0e-6)));
}

void delayMicroseconds(unsigned usec)
{
	usleep(usec);
}

void delay(long unsigned msec)
{
	delayMicroseconds(msec*1000);
}

size_t strlcat_P(char *d, PGM_P s, size_t bufsize)
{
	size_t len1 = strlen(d);
	size_t len2 = strlen(s);
	size_t ret = len1 + len2;

	if (len1+len2 >= bufsize) {
		if (bufsize < (len1+1)) {
			return ret;
		}
		len2 = bufsize - (len1+1);
	}
	if (len2 > 0) {
		memcpy(d+len1, s, len2);
		d[len1+len2] = 0;
	}
	return ret;
}

size_t strnlen_P(PGM_P str, size_t size)
{
	return strnlen(str, size);
}

size_t strlen_P(PGM_P str)
{
	return strlen(str);
}

int strcasecmp_P(PGM_P str1, PGM_P str2)
{
	return strcasecmp(str1, str2);
}

int strcmp_P(PGM_P str1, PGM_P str2)
{
	return strcmp(str1, str2);
}

void *memcpy_P(void *dest, PGM_P src, size_t n)
{
	return memcpy(dest, src, n);
}


void digitalWrite(uint8_t pin, uint8_t val)
{
}

int analogRead(uint8_t pin)
{
	return 0;
}

}


char *itoa(int __val, char *__s, int __radix)
{
	switch (__radix) {
	case 8:
		sprintf(__s, "%o", __val);
		break;
	case 16:
		sprintf(__s, "%x", __val);
		break;
	case 10:
	default:
		sprintf(__s, "%d", __val);
		break;
	}
	return __s;
}

char *ultoa(unsigned long __val, char *__s, int __radix)
{
	switch (__radix) {
	case 8:
		sprintf(__s, "%lo", __val);
		break;
	case 16:
		sprintf(__s, "%lx", __val);
		break;
	case 10:
	default:
		sprintf(__s, "%lu", __val);
		break;
	}
	return __s;
}

char *ltoa(long __val, char *__s, int __radix)
{
	switch (__radix) {
	case 8:
		sprintf(__s, "%lo", __val);
		break;
	case 16:
		sprintf(__s, "%lx", __val);
		break;
	case 10:
	default:
		sprintf(__s, "%ld", __val);
		break;
	}
	return __s;
}

#define ARRAY_LENGTH(x) (sizeof((x))/sizeof((x)[0]))

static struct {
	void (*call)(void);
} interrupt_table[7];

void attachInterrupt(uint8_t inum, void (*call)(void), int mode)
{
	if (inum >= ARRAY_LENGTH(interrupt_table)) {
		fprintf(stderr, "Bad attachInterrupt to interrupt %u\n", inum);
		exit(1);
	}
	interrupt_table[inum].call = call;
}

void runInterrupt(uint8_t inum)
{
	if (inum >= ARRAY_LENGTH(interrupt_table)) {
		fprintf(stderr, "Bad runInterrupt to interrupt %u\n", inum);
		exit(1);
	}
	if (interrupt_table[inum].call != NULL) {
		interrupt_table[inum].call();
	}
}
