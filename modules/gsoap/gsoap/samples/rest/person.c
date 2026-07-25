/*
   This example shows a REST PUT, GET, POST, DELETE client

   Uses the custom/struct_tm_date.c xsd:date serializer

   To run the client, first build and start the gSOAP webserver on port 8080
   and make sure person.xml (in samples/webserver) is in the webserver's folder
   for the GET and POST operations to work.
   See samples/webserver.

   Compile with:
   soapcpp2 -c -0 person.h
   cc -o person person.c soapC.c stdsoap2.c custom/struct_tm_date.c
*/

#include "soapH.h"
#include "soap.nsmap"

int main()
{
  struct soap *ctx = soap_new1(SOAP_XML_STRICT);
  struct Person p;

  p.name = "John Doe";
  p.dob.tm_year = 66; // 1966 (year since 1900
  p.dob.tm_mon = 0;   // month 0..11
  p.dob.tm_mday = 31 ;
  p.gender = MALE;

  printf("\nPUT\n");
  if (soap_PUT_Person(ctx, "http://localhost:8080/person.xml", &p))
    soap_print_fault(ctx, stderr);

  printf("\nGET\n");
  if (soap_GET_Person(ctx, "http://localhost:8080/person.xml", &p))
    soap_print_fault(ctx, stderr);
  else
    soap_write_Person(ctx, &p); // default stdout

  printf("\nPOST\n");
  if (soap_POST_send_Person(ctx, "http://localhost:8080/person.xml", &p)
   || soap_POST_recv_Person(ctx, &p))
    soap_print_fault(ctx, stderr);
  else
    soap_write_Person(ctx, &p); // default stdout

  printf("\nDELETE\n");
  if (soap_DELETE(ctx, "http://localhost:8080/person.xml"))
    soap_print_fault(ctx, stderr);

  soap_destroy(ctx);
  soap_end(ctx);
  soap_free(ctx);

  return 0;
}
