#include "soapH.h"
#include "r.nsmap"
int main() { return soap_serve(soap_new()); } int r__roll(struct soap *soap, int &r) { srand(time(0)); r = rand()%6+1; return SOAP_OK; }

