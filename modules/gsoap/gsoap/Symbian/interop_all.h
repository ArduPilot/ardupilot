// Interop test base round 2

//gsoap ns service name:	interop2
//gsoap ns service style:	rpc
//gsoap ns service encoding:	encoded
//gsoap ns service namespace:	http://soapinterop.org/wsdl

//gsoap ns schema namespace: http://soapinterop.org/
//gsoap s schema namespace: http://soapinterop.org/xsd
//gsoap a schema namespace: http://xml.apache.org/xml-soap

typedef char *xsd__string;
ns__echoString(xsd__string inputString, xsd__string &_return);

struct ArrayOfstring { xsd__string *__ptr; int __size; int __offset; };
ns__echoStringArray(struct ArrayOfstring inputStringArray, struct ArrayOfstring &_return);

typedef long xsd__int;
ns__echoInteger(xsd__int inputInteger, xsd__int &_return);

struct ArrayOfint { xsd__int **__ptr; int __size; int __offset; };
ns__echoIntegerArray(struct ArrayOfint inputIntegerArray, struct ArrayOfint &_return);

typedef float xsd__float;
ns__echoFloat(xsd__float inputFloat, xsd__float &_return);

struct ArrayOffloat { xsd__float **__ptr; int __size; int __offset; };
ns__echoFloatArray(struct ArrayOffloat inputFloatArray, struct ArrayOffloat &_return);

struct s__SOAPStruct
{ xsd__string varString;
  xsd__int *varInt;
  xsd__float *varFloat;
};
ns__echoStruct(struct s__SOAPStruct _inputStruct, struct ns__echoStructResponse { struct s__SOAPStruct _return; } &result);

struct ArrayOfSOAPStruct { struct s__SOAPStruct **__ptr; int __size; int __offset; };
ns__echoStructArray(struct ArrayOfSOAPStruct inputStructArray, struct ArrayOfSOAPStruct &_return);

ns__echoVoid(struct ns__echoVoidResponse { } &result);

struct xsd__base64Binary { unsigned char *__ptr; int __size; };
ns__echoBase64(struct xsd__base64Binary inputBase64, struct xsd__base64Binary &_return);

typedef char *xsd__dateTime;
ns__echoDate(xsd__dateTime inputDate, xsd__dateTime &_return);

struct xsd__hexBinary { unsigned char *__ptr; int __size; };
ns__echoHexBinary(struct xsd__hexBinary inputHexBinary, struct xsd__hexBinary &_return);

typedef char *xsd__decimal;
ns__echoDecimal(xsd__decimal inputDecimal, xsd__decimal &_return);

typedef bool xsd__boolean;
ns__echoBoolean(xsd__boolean inputBoolean, xsd__boolean &_return);

// Interop test B round 2

ns__echoStructAsSimpleTypes
(	struct	s__SOAPStruct _inputStruct,
	struct	ns__echoStructAsSimpleTypesResponse
	{	xsd__string	_outputString;
		xsd__int	*_outputInteger;
		xsd__float	*_outputFloat;
	} &	result
);

ns__echoSimpleTypesAsStruct
(	xsd__string	inputString,
	xsd__int	*inputInteger,
	xsd__float	*inputFloat,
	struct	ns__echoSimpleTypesAsStructResponse
	{	struct	s__SOAPStruct	_return;
	} &	result
);

struct	ArrayOfstring2D
{	xsd__string *	__ptr;
	int		__size[2];
	int		__offset[2];
};

ns__echo2DStringArray(struct ArrayOfstring2D _input2DStringArray, struct ArrayOfstring2D &_return);

struct	s__SOAPStructStruct
{	xsd__string		varString;
	xsd__int *		varInt;
	xsd__float *		varFloat;
	struct	s__SOAPStruct *	varStruct;
};

ns__echoNestedStruct
(	struct	s__SOAPStructStruct	_inputStruct,
	struct	ns__echoNestedStructResponse
	{	struct	s__SOAPStructStruct	_return;
	} &	result
);

struct	s__SOAPArrayStruct
{	xsd__string		varString;
	xsd__int *		varInt;
	xsd__float *		varFloat;
	struct	ArrayOfstring	varArray;
};

ns__echoNestedArray
(	struct	s__SOAPArrayStruct	_inputStruct,
	struct	ns__echoNestedArrayResponse
	{	struct	s__SOAPArrayStruct	_return;
	} &	result
);

// Interop test C round 2

//gsoap h schema namespace: http://soapinterop.org/echoheader/
struct SOAP_ENV__Header
{	char *					h__echoMeStringRequest;
	mustUnderstand char *			h__echoMeStringRequest_;
	char *					h__echoMeStringResponse;
	struct s__SOAPStruct *			h__echoMeStructRequest;
	mustUnderstand struct s__SOAPStruct *	h__echoMeStructRequest_;
	struct s__SOAPStruct *			h__echoMeStructResponse;
	char *					h__someUnknownRequest;	// do not include in server
	mustUnderstand char *			h__someUnknownRequest_;	// do not include in server
};

//gsoap m service name: interop2C
//gsoap m service style: rpc
//gsoap m service encoding: encoded
//gsoap m service port: http://www.cs.fsu.edu/~engelen/interop2C.cgi
//gsoap m service namespace: http://soapinterop.org/
//gsoap m service method-input-header-part: echoVoid h__echoMeStringRequest
//gsoap m service method-input-header-part: echoVoid h__echoMeStructRequest
//gsoap m service method-output-header-part: echoVoid h__echoMeStringResponse
//gsoap m service method-output-header-part: echoVoid h__echoMeStructResponse
int m__echoVoid(struct m__echoVoidResponse { } &result);
