//gsoap ns service name: dime
//gsoap ns service namespace: http://www.aberger,at/SOAP/dime.wsdl
//gsoap ns service location: http://localhost/gsoap?mod_gsoap.dll?dime
//gsoap ns service executable: dime.dll
//gsoap ns schema  namespace: urn:dime
//gsoap service documentation: Send image files back and forth as dime attac


class xsd__base64Binary {
  unsigned char *__ptr;
  int __size;
  char *id;
  char *type;
  char *options;

  xsd__base64Binary(); // Constructor 
  xsd__base64Binary(struct soap *soap, int n); // Constructor 
  ~xsd__base64Binary(); // Destructor 
  unsigned char *location(); // returns the memory location 
  int size(); // returns the number of bytes 

};

//gsoap method-documentation getImage retrieve an image from the server.
int ns__getImage(char *name, xsd__base64Binary &image);

//gsoap method-documentation getImage send an image to the server.
int ns__putImage(char *name, xsd__base64Binary *image, int& status); 
