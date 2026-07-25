#include "soapH.h"
#include "h.nsmap"
int main() { return soap_serve(soap_new()); } int h__hello(struct soap *soap, char *&s) { s = soap_strdup(soap, "Hello World!"); return SOAP_OK; }
