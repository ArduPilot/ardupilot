/*
	ser.h

	.NET DataContract Serialization Schema

	From type bindings in typemap.dat
*/

//gsoap ser schema import: http://schemas.microsoft.com/2003/10/Serialization/
//gsoap ser schema elementForm: qualified
//gsoap ser schema attributeForm: unqualified

struct __ser__anyType
{ int __type;		// set to SOAP_TYPE_X, where X is the type's name
  void *ser__anyType;	// points to objects of type X
};
