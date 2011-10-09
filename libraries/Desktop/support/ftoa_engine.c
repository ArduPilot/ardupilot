#include <stdio.h>

int __ftoa_engine(double val, char *buf,
		  unsigned char prec, unsigned char maxdgs)
{
	return sprintf(buf, "%*.*lf", prec, maxdgs, val);
}
