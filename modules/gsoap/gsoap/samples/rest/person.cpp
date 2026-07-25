/*
   This example shows a REST PUT, GET, POST, DELETE client

   Uses the custom/struct_tm_date.c xsd:date serializer

   To run the client, first build and start the gSOAP webserver on port 8080
   and make sure person.xml (in samples/webserver) is in the webserver's folder
   for the GET and POST operations to work.
   See samples/webserver.

   Compile with:
   soapcpp2 -0 person.h
   c++ -o person person.cpp soapC.cpp stdsoap2.cpp custom/struct_tm_date.c
*/

#include "soapH.h"
#include "soap.nsmap"

int main()
{
  soap *ctx = soap_new1(SOAP_XML_STRICT);
  Person p;

  p.name = "John Doe";
  p.dob.tm_year = 66; // 1966 (year since 1900
  p.dob.tm_mon = 0;   // month 0..11
  p.dob.tm_mday = 31 ;
  p.gender = MALE;

  std::cout << "\nPUT\n";
  if (soap_PUT_Person(ctx, "http://localhost:8080/person.xml", &p))
    soap_stream_fault(ctx, std::cerr);

  std::cout << "\nGET\n";
  if (soap_GET_Person(ctx, "http://localhost:8080/person.xml", &p))
    soap_stream_fault(ctx, std::cerr);
  else
    soap_write_Person(ctx, &p); // default stdout

  std::cout << "\nPOST\n";
  if (soap_POST_send_Person(ctx, "http://localhost:8080/person.xml", &p)
   || soap_POST_recv_Person(ctx, &p))
    soap_stream_fault(ctx, std::cerr);
  else
    soap_write_Person(ctx, &p); // default stdout

  std::cout << "\nDELETE\n";
  if (soap_DELETE(ctx, "http://localhost:8080/person.xml"))
    soap_stream_fault(ctx, std::cerr);

  soap_destroy(ctx);
  soap_end(ctx);
  soap_free(ctx);

  return 0;
}

