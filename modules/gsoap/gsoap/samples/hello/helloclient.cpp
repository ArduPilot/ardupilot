#include "soapH.h"
#include "h.nsmap"
int main() { char *s; struct soap *soap = soap_new(); soap_call_h__hello(soap, "http://www.cs.fsu.edu/~engelen/hellolitserver.cgi", "", s); printf("%s\n", s); soap_destroy(soap); soap_end(soap); soap_free(soap); return 0; }

