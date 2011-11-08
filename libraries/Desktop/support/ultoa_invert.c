#include <stdio.h>

char * __ultoa_invert(unsigned long val, char *s, int base)
{
	char tbuf[32];
	char *p;
	switch (base) {
	case 8:
		snprintf(tbuf, sizeof(tbuf), "%lo", val);
		break;
	case 16:
		snprintf(tbuf, sizeof(tbuf), "%lx", val);
		break;
	case 10:
	default:
		snprintf(tbuf, sizeof(tbuf), "%lu", val);
		break;
	}
	p = &tbuf[strlen(tbuf)-1];
	while (p >= &tbuf[0]) {
		*s++ = *p--;
	}
	return s;
}
