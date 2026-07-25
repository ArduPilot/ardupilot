/* mod_gsoap sample.
	please note: if you use classes in your service header file, you have to export soap_destroy also.
	Please uncomment the according line in the stdsoap2.def file in that case, otherwise you will have memory leaks.
 */
#include "soapH.h"
#include "calc.nsmap"


extern "C" {

/** This function is called by mod_gsoap after the dll was successfully loaded and before processing begins.
	You can do any one-time initialization here.
*/
int mod_gsoap_init() {
	// todo: add your initialization code here 
	return SOAP_OK;
}
/** This function is called after all processing was done before dll is unloaded.
	You can do any cleanup here.
*/
int mod_gsoap_terminate() {
	// todo: add your termination code here 
	return SOAP_OK;
}

}

int ns__add(struct soap *soap, double a, double b, double *result)
{ *result = a + b;
  return SOAP_OK;
} 

int ns__sub(struct soap *soap, double a, double b, double *result)
{ *result = a - b;
  return SOAP_OK;
} 

int ns__mul(struct soap *soap, double a, double b, double *result)
{ *result = a * b;
  return SOAP_OK;
} 

int ns__div(struct soap *soap, double a, double b, double *result)
{ if (b)
    *result = a / b;
  else
  { char *s = (char*)soap_malloc(soap, 1024);
    sprintf(s, "Can't divide %f by %f", a, b);
    return soap_receiver_fault(soap, "Division by zero", s);
  }
  return SOAP_OK;
} 

int ns__pow(struct soap *soap, double a, double b, double *result)
{ *result = pow(a, b);
  if (soap_errno == EDOM)	/* soap_errno is like errno, but compatible with Win32 */
  { char *s = (char*)soap_malloc(soap, 1024);
    sprintf(s, "Can't take the power of %f to %f", a, b);
    return soap_receiver_fault(soap, "Power function domain error", s);
  }
  return SOAP_OK;
} 
