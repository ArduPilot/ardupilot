#include <string.h>
#include "soapH.h"
#include "calc.nsmap"

int main(int argc, char** argv)
{ 
  struct soap *soap = soap_new();
  int a, b, result;
  if(argc > 3 )
  { a = atoi(argv[1]);
    b = atoi(argv[3]);
  }
  else
      return -1;

  switch (*argv[2]) {
  case '+':
    if(soap_call_ns__add(soap, "http://localhost:8080/soap", "calculate", a, b, &result) == 0)
      printf("%d+%d=%d\n", a, b, result);
    else
      soap_print_fault(soap, stdout);
    break;
 }
  return 0;
}
