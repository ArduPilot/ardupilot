//gsoap t service name: gmt Get current time client code: `#include "soapH.h" #include "gmt.nsmap" int main() { time_t t; struct soap *soap = soap_new(); soap_call_t__gmt(soap, "http://www.cs.fsu.edu/~engelen/gmtlitserver.cgi", "", &t); printf("The current time is %s\n", ctime(&t)); soap_destroy(soap); soap_end(soap); soap_free(soap); return 0; }`
t__gmt(time_t*);
