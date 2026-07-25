/*
        wsc2.h

	WS-SecureConversation 2005/02
        Generated with:
        wsdl2h -cex -o wsc.h -t WS/WS-typemap.dat WS/WS-SecureConversation.xsd

        - Removed //gsoapopt
        - Changed //gsoap wsc schema namespace directive to import directive
        - Changed wsc namespace URI to http://schemas.xmlsoap.org/ws/2005/02/sc
        - Changed #import "wsse2.h"
*/


/******************************************************************************\
 *                                                                            *
 * Definitions                                                                *
 *   http://schemas.xmlsoap.org/ws/2005/02/sc                                 *
 *                                                                            *
\******************************************************************************/


/******************************************************************************\
 *                                                                            *
 * Import                                                                     *
 *                                                                            *
\******************************************************************************/

#import "wsse2.h"
#import "wsu.h"	// wsu = <http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd>

/******************************************************************************\
 *                                                                            *
 * Schema Namespaces                                                          *
 *                                                                            *
\******************************************************************************/


#define SOAP_NAMESPACE_OF_wsc	"http://schemas.xmlsoap.org/ws/2005/02/sc"
//gsoap wsc   schema import:	http://schemas.xmlsoap.org/ws/2005/02/sc
//gsoap wsc   schema elementForm:	qualified
//gsoap wsc   schema attributeForm:	unqualified

/******************************************************************************\
 *                                                                            *
 * Built-in Schema Types and Top-Level Elements and Attributes                *
 *                                                                            *
\******************************************************************************/


/// Imported element "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-secext-1.0.xsd":SecurityTokenReference from typemap WS/WS-typemap.dat.


/******************************************************************************\
 *                                                                            *
 * Schema Types and Top-Level Elements and Attributes                         *
 *   http://schemas.xmlsoap.org/ws/2005/02/sc                                 *
 *                                                                            *
\******************************************************************************/


/// union of values "wsc:FaultCodeType xs:QName"
typedef char* wsc__FaultCodeOpenEnumType;

/// "http://schemas.xmlsoap.org/ws/2005/02/sc":FaultCodeType is a simpleType restriction of xs:QName.
enum wsc__FaultCodeType
{
	wsc__BadContextToken,	///< xs:QName value=""http://schemas.xmlsoap.org/ws/2005/02/sc":BadContextToken"
	wsc__UnsupportedContextToken,	///< xs:QName value=""http://schemas.xmlsoap.org/ws/2005/02/sc":UnsupportedContextToken"
	wsc__UnknownDerivationSource,	///< xs:QName value=""http://schemas.xmlsoap.org/ws/2005/02/sc":UnknownDerivationSource"
	wsc__RenewNeeded,	///< xs:QName value=""http://schemas.xmlsoap.org/ws/2005/02/sc":RenewNeeded"
	wsc__UnableToRenew,	///< xs:QName value=""http://schemas.xmlsoap.org/ws/2005/02/sc":UnableToRenew"
};

/******************************************************************************\
 *                                                                            *
 * Schema Complex Types and Top-Level Elements                                *
 *   http://schemas.xmlsoap.org/ws/2005/02/sc                                 *
 *                                                                            *
\******************************************************************************/


/// "http://schemas.xmlsoap.org/ws/2005/02/sc":SecurityContextTokenType is a complexType.
/// @brief Actual content model is non-deterministic, hence wildcard. The following shows intended content model: <xs:element ref='wsc:Identifier' minOccurs='1' /> <xs:element ref='wsc:Instance' minOccurs='0' /> <xs:any namespace='##any' processContents='lax' minOccurs='0' maxOccurs='unbounded' />
struct wsc__SecurityContextTokenType
{
/// TODO: <any namespace="##any" minOccurs="0" maxOccurs="unbounded">
/// TODO: Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element).
/// Imported attribute reference "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd":Id.
   @char*                                wsu__Id                        0;	///< Optional attribute.
/// <anyAttribute namespace="##other">
/// TODO: Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
///       Use wsdl2h option -d for xsd__anyAttribute DOM (soap_dom_attribute).
/// Member declared in WS/WS-typemap.dat
       char*                                Identifier;
/// Member declared in WS/WS-typemap.dat
       char*                                Instance;
};

/// "http://schemas.xmlsoap.org/ws/2005/02/sc":DerivedKeyTokenType is a complexType.
struct wsc__DerivedKeyTokenType
{
/// Imported element reference "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-secext-1.0.xsd":SecurityTokenReference.
    _wsse__SecurityTokenReference*       wsse__SecurityTokenReference   0;	///< Optional element.
/// Element Properties of type "http://schemas.xmlsoap.org/ws/2005/02/sc":PropertiesType.
    struct wsc__PropertiesType*          Properties                     0;	///< Optional element.
/// SEQUENCE OF ELEMENTS <xs:sequence minOccurs="0">
   $int                                  __size_DerivedKeyTokenType_sequence 0;
    struct __wsc__DerivedKeyTokenType_sequence
    {
/// CHOICE OF ELEMENTS <xs:choice>
   $int                                  __union_DerivedKeyTokenType   ;	///< Union _wsc__union_DerivedKeyTokenType selector: set to SOAP_UNION__wsc__union_DerivedKeyTokenType_<fieldname>
    union _wsc__union_DerivedKeyTokenType
    {
/// Element Generation of type xs:unsignedLong.
    ULONG64                              Generation                     1;	///< Required element.
/// Element Offset of type xs:unsignedLong.
    ULONG64                              Offset                         1;	///< Required element.
    }                                    union_DerivedKeyTokenType     ;
//  END OF CHOICE
/// Element Length of type xs:unsignedLong.
    ULONG64*                             Length                         0;	///< Optional element.
    }                                   *__DerivedKeyTokenType_sequence;
//  END OF SEQUENCE
/// Element reference "http://schemas.xmlsoap.org/ws/2005/02/sc":Label.
    char*                                Label                          0;	///< Optional element.
/// Element reference "http://schemas.xmlsoap.org/ws/2005/02/sc":Nonce.
    char*                                Nonce                          0;	///< Optional element.
/// Imported attribute reference "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd":Id.
   @char*                                wsu__Id                        0;	///< Optional attribute.
/// Attribute Algorithm of type xs:anyURI.
   @char*                                Algorithm                      0;	///< Optional attribute.
};

/// "http://schemas.xmlsoap.org/ws/2005/02/sc":PropertiesType is a complexType.
struct wsc__PropertiesType
{
/// TODO: <any namespace="##any" minOccurs="0" maxOccurs="unbounded">
/// TODO: Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element).
};

/******************************************************************************\
 *                                                                            *
 * Additional Top-Level Elements                                              *
 *   http://schemas.xmlsoap.org/ws/2005/02/sc                                 *
 *                                                                            *
\******************************************************************************/


/// Top-level root element "http://schemas.xmlsoap.org/ws/2005/02/sc":SecurityContextToken of type "http://schemas.xmlsoap.org/ws/2005/02/sc":SecurityContextTokenType.
/// Note: use wsdl2h option -g to auto-generate a top-level root element declaration and processing code.

/// Top-level root element "http://schemas.xmlsoap.org/ws/2005/02/sc":Identifier of type xs:anyURI.
// '_wsc__Identifier' element definition intentionally left blank.

/// Top-level root element "http://schemas.xmlsoap.org/ws/2005/02/sc":Instance of type xs:string.
// '_wsc__Instance' element definition intentionally left blank.

/// Top-level root element "http://schemas.xmlsoap.org/ws/2005/02/sc":DerivedKeyToken of type "http://schemas.xmlsoap.org/ws/2005/02/sc":DerivedKeyTokenType.
/// Note: use wsdl2h option -g to auto-generate a top-level root element declaration and processing code.

/// Top-level root element "http://schemas.xmlsoap.org/ws/2005/02/sc":Name of type xs:anyURI.
// '_wsc__Name' element definition intentionally left blank.

/// Top-level root element "http://schemas.xmlsoap.org/ws/2005/02/sc":Label of type xs:string.
// '_wsc__Label' element definition intentionally left blank.

/// Top-level root element "http://schemas.xmlsoap.org/ws/2005/02/sc":Nonce of type xs:base64Binary.
// '_wsc__Nonce' element definition intentionally left blank.

/******************************************************************************\
 *                                                                            *
 * Additional Top-Level Attributes                                            *
 *   http://schemas.xmlsoap.org/ws/2005/02/sc                                 *
 *                                                                            *
\******************************************************************************/


/// Top-level attribute "http://schemas.xmlsoap.org/ws/2005/02/sc":Instance of simpleType xs:string.
// '_wsc__Instance' attribute definition intentionally left blank.

/// Top-level attribute "http://schemas.xmlsoap.org/ws/2005/02/sc":Nonce of simpleType xs:base64Binary.
// '_wsc__Nonce' attribute definition intentionally left blank.

/******************************************************************************\
 *                                                                            *
 * XML Data Binding                                                           *
 *                                                                            *
\******************************************************************************/


/**

@page page_XMLDataBinding XML Data Binding

SOAP/XML services use data bindings contractually bound by WSDL and auto-
generated by wsdl2h and soapcpp2 (see Service Bindings). Plain data bindings
are adopted from XML schemas as part of the WSDL types section or when running
wsdl2h on a set of schemas to produce non-SOAP-based XML data bindings.

The following readers and writers are C/C++ data type (de)serializers auto-
generated by wsdl2h and soapcpp2. Run soapcpp2 on this file to generate the
(de)serialization code, which is stored in soapC.c[pp]. Include "soapH.h" in
your code to import these data type and function declarations. Only use the
soapcpp2-generated files in your project build. Do not include the wsdl2h-
generated .h file in your code.

XML content can be retrieved from:
  - a file descriptor, using soap->recvfd = fd
  - a socket, using soap->socket = ...
  - a C++ stream, using soap->is = ...
  - a buffer, using the soap->frecv() callback

XML content can be stored to:
  - a file descriptor, using soap->sendfd = fd
  - a socket, using soap->socket = ...
  - a C++ stream, using soap->os = ...
  - a buffer, using the soap->fsend() callback


@section wsc Top-level root elements of schema "http://schemas.xmlsoap.org/ws/2005/02/sc"

  - <wsc:SecurityContextToken> (use wsdl2h option -g to auto-generate)

  - <wsc:Identifier> @ref _wsc__Identifier
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wsc__Identifier(struct soap*, char*);
    // Writer (returns SOAP_OK on success):
    soap_write__wsc__Identifier(struct soap*, char*);
    @endcode

  - <wsc:Instance> @ref _wsc__Instance
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wsc__Instance(struct soap*, char*);
    // Writer (returns SOAP_OK on success):
    soap_write__wsc__Instance(struct soap*, char*);
    @endcode

  - <wsc:DerivedKeyToken> (use wsdl2h option -g to auto-generate)

  - <wsc:Name> @ref _wsc__Name
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wsc__Name(struct soap*, char*);
    // Writer (returns SOAP_OK on success):
    soap_write__wsc__Name(struct soap*, char*);
    @endcode

  - <wsc:Label> @ref _wsc__Label
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wsc__Label(struct soap*, char*);
    // Writer (returns SOAP_OK on success):
    soap_write__wsc__Label(struct soap*, char*);
    @endcode

  - <wsc:Nonce> @ref _wsc__Nonce
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wsc__Nonce(struct soap*, char*);
    // Writer (returns SOAP_OK on success):
    soap_write__wsc__Nonce(struct soap*, char*);
    @endcode

*/

/* End of wsc.h */
