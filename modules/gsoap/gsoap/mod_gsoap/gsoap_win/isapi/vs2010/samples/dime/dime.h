//gsoap ns service name: dime
//gsoap ns service namespace: http://www.aberger,at/SOAP/dime.wsdl
//gsoap ns service location: http://localhost/gsoap/mod_gsoap.dll?gsoap/dime
//gsoap ns service executable: dime.dll
//gsoap ns schema  namespace: urn:dime

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

int ns__getImage(char *name, xsd__base64Binary &image);

int ns__putImage(char *name, xsd__base64Binary *image, int& status); 
