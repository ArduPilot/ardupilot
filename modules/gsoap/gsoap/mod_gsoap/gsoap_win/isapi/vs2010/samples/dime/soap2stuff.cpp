/** file to include stdsoap2.cpp here, to get around directory location problems etc.
*/

/* You must set the include path in Visual Studio under tools|options|directories|include anyway. 
   There also the file below can be found.
*/
#include "stdsoap2.cpp"

#include "soapH.h"

xsd__base64Binary::xsd__base64Binary() 
{ 
   __ptr = NULL; 
   __size = 0; 
   id = NULL;
   type = NULL;
   options = NULL;
} 
xsd__base64Binary::xsd__base64Binary(struct soap *soap, int n) 
{ 
   __ptr = (unsigned char*)soap_malloc(soap, n); 
   __size = n; 
   id = NULL;
   type = NULL;
   options = NULL;
} 
xsd__base64Binary::~xsd__base64Binary() 
{ } 
unsigned char *xsd__base64Binary::location() 
{ 
   return __ptr; 
} 
int xsd__base64Binary::size() 
{ 
   return __size; 
} 
