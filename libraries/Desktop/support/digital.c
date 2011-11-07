// support for digitalRead() and digitalWrite()
#include <stdio.h>
#include <stdbool.h>
#include <time.h>
#include <sys/time.h>
#include <stdint.h>
#include "desktop.h"


int digitalRead(uint8_t address)
{
	switch (address) {
	case 40:
		// CLI slider switch
		return desktop_state.slider?0:1;

	default:
		break;
	}
	printf("digitalRead(0x%x) unsupported\n", address);
	return 0;
}
