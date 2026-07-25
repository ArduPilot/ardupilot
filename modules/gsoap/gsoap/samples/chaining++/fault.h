/*	fault.h

	Example SOAP Fault detail definitions, to demonstrate the use of env.h.
	Defines optional SOAP Fault derail data structures

	Copyright (C) 2000-2004 Robert A. van Engelen. All Rights Reserved.
*/

/*

Add any data structure you want to serialize as part of the SOAP Fault detail
element. The detail element '__type' and 'value' fields should be set to
transmit data. The fields are set when data of corresponding types are received.

For example, we define an <element> of name <f:myData> with a string vector
(note the leading _ in the following declaration):

class _f__myData
{ public:
  std::vector<std::string*> *data;
};

To return a fault from your service application:

soap_sender_fault(soap, "An error occurred", NULL));	// set soap fault
soap->fault->detail = (struct SOAP_ENV__Detail*)soap_malloc(soap, sizeof(struct SOAP_ENV__Detail));
soap->fault->detail->__type = SOAP_TYPE__f__myData;
soap->fault->detail->value = soap_new__f__myData(soap, -1);
return SOAP_FAULT;

In addition, you can modify the SOAP_ENV__Detail struct and add your own set
of fields, as in:

struct SOAP_ENV__Detail
{ struct f__myDataType f__myData;
  int __type;
  void *value;
  char *__any;		// or use '_XML __any' to store literal XML content
};
 
*/
