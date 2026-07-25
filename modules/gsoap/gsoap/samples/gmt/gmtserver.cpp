#include "soapH.h"
#include "gmt.nsmap"
int main() { return soap_serve(soap_new()); } int t__gmt(struct soap *soap, time_t *t) { *t = time(0); return SOAP_OK; }
