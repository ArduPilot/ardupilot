#include <stdio.h>
#include <stdint.h>
#include <ftoa_engine.h>

/*
ftoa: val=0.000000 prec=3 maxdgs=7 -> ret=0 flags=0x2 '0000'
ftoa: val=inf prec=3 maxdgs=7 -> ret=38 flags=0x4 '3403'
ftoa: val=1.000000 prec=3 maxdgs=7 -> ret=0 flags=0x0 '1000'
ftoa: val=1.200000 prec=3 maxdgs=7 -> ret=0 flags=0x0 '1200'
ftoa: val=1.230000 prec=3 maxdgs=7 -> ret=0 flags=0x0 '1230'
ftoa: val=10.000000 prec=3 maxdgs=7 -> ret=1 flags=0x0 '1000'
ftoa: val=12.000000 prec=3 maxdgs=7 -> ret=1 flags=0x0 '1200'
ftoa: val=12.300000 prec=3 maxdgs=7 -> ret=1 flags=0x0 '1230'
ftoa: val=101.234570 prec=3 maxdgs=7 -> ret=2 flags=0x0 '1012'
ftoa: val=123.456790 prec=3 maxdgs=7 -> ret=2 flags=0x0 '1235'
ftoa: val=1234.567900 prec=3 maxdgs=7 -> ret=3 flags=0x0 '1235'
ftoa: val=1234.567900 prec=3 maxdgs=1 -> ret=3 flags=0x0 '1235'
ftoa: val=1234.567900 prec=1 maxdgs=3 -> ret=3 flags=0x0 '12'
ftoa: val=0.000000 prec=3 maxdgs=7 -> ret=0 flags=0x2 '0000'
ftoa: val=0.200000 prec=3 maxdgs=7 -> ret=-1 flags=0x0 '2000'
ftoa: val=0.230000 prec=3 maxdgs=7 -> ret=-1 flags=0x0 '2300'
ftoa: val=0.023457 prec=3 maxdgs=7 -> ret=-2 flags=0x0 '2346'
ftoa: val=0.002346 prec=3 maxdgs=7 -> ret=-3 flags=0x0 '2346'
ftoa: val=0.000235 prec=3 maxdgs=7 -> ret=-4 flags=0x0 '235'
ftoa: val=0.000023 prec=3 maxdgs=7 -> ret=-5 flags=0x0 '23'
ftoa: val=0.000002 prec=3 maxdgs=7 -> ret=-6 flags=0x0 '2'
ftoa: val=0.000000 prec=3 maxdgs=7 -> ret=-7 flags=0x0 '2'
ftoa: val=0.000000 prec=3 maxdgs=7 -> ret=-8 flags=0x0 '2'
ftoa: val=0.000000 prec=3 maxdgs=7 -> ret=-9 flags=0x0 '2'
ftoa: val=0.200000 prec=5 maxdgs=2 -> ret=-1 flags=0x0 '2'
ftoa: val=0.230000 prec=5 maxdgs=2 -> ret=-1 flags=0x0 '2'
*/

int __ftoa_engine(double val, char *buf,
		  unsigned char prec, unsigned char maxdgs)
{
	uint8_t flags = 0;
	char tbuf[32];
	int ret;

	if (isnan(val)) {
		buf[0] = FTOA_NAN;
		return 0;
	}
	if (isinf(val)) {
		buf[0] = FTOA_INF;
		return 0;
	}

	buf[0] = 0;

	if (val < 0.0) {
		buf[0] |= FTOA_MINUS;
		val = -val;
	}

	ret = -1;
	while (val >= 1.0) {
		val *= 0.1;
		ret++;
	}
	while (val < 0.1) {
		val *= 10.0;
		ret--;
		if (ret < -10) {
			buf[0] |= FTOA_ZERO;
			memset(&buf[1], '0', prec+1);
			buf[1+prec+1] = 0;
			return 0;
		}
	}
	if (maxdgs == 0) {
		snprintf(tbuf, sizeof(tbuf), "%lf", val);
	} else {
		snprintf(tbuf, sizeof(tbuf), "%.*lf", (int)maxdgs, val);
	}
	strncpy(buf+1, &tbuf[2], prec+1);
	return ret;
}
