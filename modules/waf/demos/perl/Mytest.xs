/* Shamelessy stolen from perlxstut */

#include <EXTERN.h>
#include <perl.h>
#include <XSUB.h>



MODULE = Mytest		PACKAGE = Mytest		

void
hello()
	CODE:
	printf("Hello, world\n");
