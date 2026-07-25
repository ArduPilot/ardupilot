/** Demo of a calculator gsoap service implemented as a shared library that can be loaded from Apache Http Server.
 * @author Christian Aberger (http://www.aberger.at)
 * @file calculator.cpp
 */

#include <float.h>
#include <string.h>
#include "soapH.h"
#include "apache_gsoap.h"
#include "calc.nsmap" // link the namespace

IMPLEMENT_GSOAP_SERVER() ///< this macro defines the necessary exports from the dll.

int ns__add(struct soap *soap, int a, int b, int *result) {
	*result = a + b;
	return SOAP_OK;
}
int ns__subtract(struct soap *, int a, int b, int *result) {
	*result = a - b;
	return SOAP_OK;
}
int ns__mutiply(struct soap *, int a, int b, int *result) {
	*result = a * b;
	return SOAP_OK;
}
int ns__divide(struct soap * soap, int a, int b, int *result) {
	if (fabs(a) > DBL_EPSILON) {
		*result = a / b;
	} else {
	  return soap_receiver_fault(soap, "divide by zero in ns__divide", "make sure divisor is > 0");
	}
	return SOAP_OK;
}
/*
int main(void) {
	struct soap soap;
	soap_init(&soap);
	int master = soap_bind(&soap, "localhost", 8080, 100);
	if (master < 0) {
		soap_print_fault(&soap, stderr);
		exit(-1);
	}
	fprintf(stderr, "Socket connection success\n");
	while(1) {
		int client = soap_accept(&soap);
		if (client < 0) {
			soap_print_fault(&soap, stderr);
			exit(-2);
		}
		fprintf(stderr, "Client connected\n");
		soap_serve(&soap);
		soap_end(&soap);
	}
}
*/

