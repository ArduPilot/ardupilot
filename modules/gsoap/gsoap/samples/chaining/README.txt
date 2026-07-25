
How to combine multiple C services into one server executable and let the
services listen to the same port (or serve over CGI)?

Run wsdl2h on each WSDL, with "wsdl2h -c name.wsdl" to produce name.h

Run "soapcpp2 -c -S -np name name.h" to produce nameServerLib.c

Run "soapcpp2 -c -CS -penv env.h" on an empty env.h (or an env.h with shared
SOAP Header and SOAP Fault definitions) to produce envC.c for Header and Fault
processing.

Say we have name1 and name2 services, each with different methods ns1__method1
and ns2__method2. Implement service chaining as follows:

#include "name1.nsmap"
#include "name2.nsmap"
int main()
{
        struct soap *soap = soap_new();

        soap_set_namespaces(soap, name1_namespaces);
        /* serve over stdin/out, CGI style */
        if (soap_begin_serve(soap))
                soap_print_fault(soap, stderr);
        else if (name1_serve_request(soap) == SOAP_NO_METHOD)
        {
                soap_set_namespaces(soap, name2_namespaces);
                if (name2_serve_request(soap))
                        soap_print_fault(soap, stderr);
        }
        soap_destroy(soap);
        soap_end(soap);
        soap_free(soap);
        return 0;
}

int ns1__method1(struct soap *soap, ...)
{ ...
  return SOAP_OK;
}

int ns2__method2(struct soap *soap, ...)
{ ...
  return SOAP_OK;
}

