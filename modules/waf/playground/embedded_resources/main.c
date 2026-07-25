#include <stdio.h>
#include <stdint.h>
#include "x-resources.h"


int main()
{
	EXTLD(main_c)

	size_t length = LDLEN(main_c);
	uint8_t const * data = LDVAR(main_c);

	printf("Data at %p, len %zu\n", data, length);
	printf("%s\n", data);

	return 0;
}

