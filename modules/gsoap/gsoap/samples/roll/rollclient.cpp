#include "soapH.h"
#include "r.nsmap"
int main() { int d; struct soap *soap = soap_new(); soap_call_r__roll(soap, "http://www.cs.fsu.edu/~engelen/rolllitserver.cgi", "", d); printf("%d\n", d); soap_destroy(soap); soap_end(soap); soap_free(soap); return 0; }

