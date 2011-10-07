#include <stdio.h>

char * __ultoa_invert(unsigned long val, char *s, int base)
{
	switch (base) {
	case 8:
		return s+sprintf(s, "%lo", val);
	case 16:
		return s+sprintf(s, "%lx", val);
	}
	return s+sprintf(s, "%lu", val);
}
