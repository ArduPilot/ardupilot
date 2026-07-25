/*
	ckdbtest.c

	Test client for HTTP cookie database manager.

	Copyright (C) 2000-2002 Robert A. van Engelen. All Rights Reserved.

        1. Compile ckdb.h:
           soapcpp2 -cnpckdb ckdb.h
        2. Compile ckdb.c:
           cc -DWITH_COOKIES -c ckdb.c
        3. Compile and link ckdbtest.c:
           soapcpp2 -c ckdbtest.h
           cc -DWITH_COOKIES ckdbtest.c ckdb.o stdsoap2.c soapC.c soapClient.c
        4. Execute
           Cookies will be stored in 'jar.xml'

*/

#include "soapH.h"
#include "ckdbtest.nsmap"

char ckserver[] = "http://www.cs.fsu.edu/~engelen/ck.cgi";

int main()
{
  struct soap soap;
  char *r;
  soap_init(&soap);
  if (soap_call_ck__demo(&soap, ckserver, NULL, &r))
  {
    soap_print_fault(&soap, stderr);
    soap_print_fault_location(&soap, stderr);
    exit(1);
  }
  printf("The server responded with: %s\n", r);
  if (soap_save_cookies(&soap, "jar.xml"))
    fprintf(stderr, "Cannot store cookies\n");
  soap_free_cookies(&soap);
  if (soap_load_cookies(&soap, "jar.xml"))
    fprintf(stderr, "Cannot restore cookies\n");
  else
    printf("Got cookies (%s=%s)\n", soap.cookies->name, soap.cookies->value);
  if (soap_call_ck__demo(&soap, ckserver, NULL, &r))
  {
    soap_print_fault(&soap, stderr);
    soap_print_fault_location(&soap, stderr);
    exit(1);
  }
  printf("The server responded with: %s\n", r);
  if (soap_save_cookies(&soap, "jar.xml"))
    fprintf(stderr, "Cannot store cookies\n");
  soap_end(&soap);	/* This will delete the deserialized cookies too! */
  soap.cookies = NULL;	/* so make sure this is NULL */
  return 0;
}
