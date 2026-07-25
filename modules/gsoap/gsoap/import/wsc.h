/*
        wsc.h

	WS-SecureConversation 2005/12, also accepts 2005/02
        Generated with:
        wsdl2h -cex -o wsc.h -t WS/WS-typemap.dat WS/WS-SecureConversation.xsd

        - Removed //gsoapopt
        - Removed #import "wsse.h"
        - Removed #import "wsu.h"
        - Changed //gsoap wsc schema namespace directive to import directive
        - Added //gsoap wsc schema namespace2 directive
*/

/******************************************************************************\
 *                                                                            *
 * Definitions                                                                *
 *   http://docs.oasis-open.org/ws-sx/ws-secureconversation/200512            *
 *                                                                            *
\******************************************************************************/


/******************************************************************************\
 *                                                                            *
 * Import                                                                     *
 *                                                                            *
\******************************************************************************/

/******************************************************************************\
 *                                                                            *
 * Schema Namespaces                                                          *
 *                                                                            *
\******************************************************************************/

#define SOAP_NAMESPACE_OF_wsc	"http://docs.oasis-open.org/ws-sx/ws-secureconversation/200512"
//gsoap wsc   schema import:	http://docs.oasis-open.org/ws-sx/ws-secureconversation/200512
//gsoap wsc   schema namespace2:	http://schemas.xmlsoap.org/ws/2005/02/sc
//gsoap wsc   schema elementForm:	qualified
//gsoap wsc   schema attributeForm:	unqualified

/******************************************************************************\
 *                                                                            *
 * Built-in Schema Types and Top-Level Elements and Attributes                *
 *                                                                            *
\******************************************************************************/

/// Imported element ""http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-secext-1.0.xsd":SecurityTokenReference" from typemap "WS/WS-typemap.dat".


/******************************************************************************\
 *                                                                            *
 * Schema Types and Top-Level Elements and Attributes                         *
 *   http://docs.oasis-open.org/ws-sx/ws-secureconversation/200512            *
 *                                                                            *
\******************************************************************************/

/// @brief Union of values from member types "wsc:FaultCodeType xs:QName".
typedef char* wsc__FaultCodeOpenEnumType;

/// @brief "http://docs.oasis-open.org/ws-sx/ws-secureconversation/200512":FaultCodeType is a simpleType restriction of XSD type xs:QName.
///
enum wsc__FaultCodeType
{
	wsc__BadContextToken,	///< xs:QName value=""http://docs.oasis-open.org/ws-sx/ws-secureconversation/200512":BadContextToken"
	wsc__UnsupportedContextToken,	///< xs:QName value=""http://docs.oasis-open.org/ws-sx/ws-secureconversation/200512":UnsupportedContextToken"
	wsc__UnknownDerivationSource,	///< xs:QName value=""http://docs.oasis-open.org/ws-sx/ws-secureconversation/200512":UnknownDerivationSource"
	wsc__RenewNeeded,	///< xs:QName value=""http://docs.oasis-open.org/ws-sx/ws-secureconversation/200512":RenewNeeded"
	wsc__UnableToRenew,	///< xs:QName value=""http://docs.oasis-open.org/ws-sx/ws-secureconversation/200512":UnableToRenew"
};


/******************************************************************************\
 *                                                                            *
 * Schema Complex Types and Top-Level Elements                                *
 *   http://docs.oasis-open.org/ws-sx/ws-secureconversation/200512            *
 *                                                                            *
\******************************************************************************/

/// @brief "http://docs.oasis-open.org/ws-sx/ws-secureconversation/200512":SecurityContextTokenType is a complexType.
///
/// <PRE><BLOCKQUOTE>
///   Actual content model is non-deterministic, hence wildcard. The following shows intended content model:
///   <xs:element ref='wsc:Identifier' minOccurs='1' />
///   <xs:element ref='wsc:Instance' minOccurs='0' />
///   <xs:any namespace='##any' processContents='lax' minOccurs='0' maxOccurs='unbounded' />
/// </BLOCKQUOTE></PRE>
struct wsc__SecurityContextTokenType
{
/// @todo <any namespace="##any" minOccurs="0" maxOccurs="unbounded">
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element).
/// Imported attribute reference "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd":Id.
   @char*                                wsu__Id                        0;	///< Optional attribute.
/// @todo <anyAttribute namespace="##other">.
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
///       Use wsdl2h option -d for xsd__anyAttribute DOM (soap_dom_attribute).
/// Member declared in WS/WS-typemap.dat
       char*                                Identifier;
/// Member declared in WS/WS-typemap.dat
       char*                                Instance;
};

/// @brief "http://docs.oasis-open.org/ws-sx/ws-secureconversation/200512":DerivedKeyTokenType is a complexType.
///
struct wsc__DerivedKeyTokenType
{
/// Imported element reference "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-secext-1.0.xsd":SecurityTokenReference.
    _wsse__SecurityTokenReference*       wsse__SecurityTokenReference   0;	///< Optional element.
/// Element "Properties" of XSD type "http://docs.oasis-open.org/ws-sx/ws-secureconversation/200512":PropertiesType.
    struct wsc__PropertiesType*          Properties                     0;	///< Optional element.
//  BEGIN SEQUENCE <xs:sequence minOccurs="0">
    struct __wsc__DerivedKeyTokenType_sequence
    {
//  BEGIN CHOICE <xs:choice>
   $int                                  __union_DerivedKeyTokenType   ;	///< Union _wsc__union_DerivedKeyTokenType selector: set to SOAP_UNION__wsc__union_DerivedKeyTokenType_<fieldname>
    union _wsc__union_DerivedKeyTokenType
    {
/// Element "Generation" of XSD type xs:unsignedLong.
    ULONG64                              Generation                     1;	///< Required element.
/// Element "Offset" of XSD type xs:unsignedLong.
    ULONG64                              Offset                         1;	///< Required element.
    }                                    union_DerivedKeyTokenType     ;
//  END OF CHOICE
/// Element "Length" of XSD type xs:unsignedLong.
    ULONG64*                             Length                         0;	///< Optional element.
    }                                   *__DerivedKeyTokenType_sequence 0;
//  END OF SEQUENCE
/// Element reference "http://docs.oasis-open.org/ws-sx/ws-secureconversation/200512:""http://docs.oasis-open.org/ws-sx/ws-secureconversation/200512":Label.
    char*                                Label                          0;	///< Optional element.
/// Element reference "http://docs.oasis-open.org/ws-sx/ws-secureconversation/200512:""http://docs.oasis-open.org/ws-sx/ws-secureconversation/200512":Nonce.
    char*                                Nonce                          0;	///< Optional element.
/// Imported attribute reference "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd":Id.
   @char*                                wsu__Id                        0;	///< Optional attribute.
/// Attribute "Algorithm" of XSD type xs:anyURI.
   @char*                                Algorithm                      0;	///< Optional attribute.
};

/// @brief "http://docs.oasis-open.org/ws-sx/ws-secureconversation/200512":PropertiesType is a complexType.
///
struct wsc__PropertiesType
{
/// @todo <any namespace="##any" minOccurs="0" maxOccurs="unbounded">
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element).
};


/******************************************************************************\
 *                                                                            *
 * Additional Top-Level Elements                                              *
 *   http://docs.oasis-open.org/ws-sx/ws-secureconversation/200512            *
 *                                                                            *
\******************************************************************************/

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-secureconversation/200512":SecurityContextToken of XSD type "http://docs.oasis-open.org/ws-sx/ws-secureconversation/200512":SecurityContextTokenType.
/// @note Use wsdl2h option -g to auto-generate a top-level root element declaration and processing code.

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-secureconversation/200512":Identifier of XSD type xs:anyURI.
// "_wsc__Identifier" element definition intentionally left blank.
/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-secureconversation/200512":Instance of XSD type xs:string.
// "_wsc__Instance" element definition intentionally left blank.
/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-secureconversation/200512":DerivedKeyToken of XSD type "http://docs.oasis-open.org/ws-sx/ws-secureconversation/200512":DerivedKeyTokenType.
/// @note Use wsdl2h option -g to auto-generate a top-level root element declaration and processing code.

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-secureconversation/200512":Name of XSD type xs:anyURI.
// "_wsc__Name" element definition intentionally left blank.
/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-secureconversation/200512":Label of XSD type xs:string.
// "_wsc__Label" element definition intentionally left blank.
/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-secureconversation/200512":Nonce of XSD type xs:base64Binary.
// "_wsc__Nonce" element definition intentionally left blank.

/******************************************************************************\
 *                                                                            *
 * Additional Top-Level Attributes                                            *
 *   http://docs.oasis-open.org/ws-sx/ws-secureconversation/200512            *
 *                                                                            *
\******************************************************************************/

/// @brief Top-level attribute "http://docs.oasis-open.org/ws-sx/ws-secureconversation/200512":Instance of simpleType xs:string.
// "_wsc__Instance" attribute definition intentionally left blank.
/// @brief Top-level attribute "http://docs.oasis-open.org/ws-sx/ws-secureconversation/200512":Nonce of simpleType xs:base64Binary.
// "_wsc__Nonce" attribute definition intentionally left blank.
/// @brief Top-level attribute "http://docs.oasis-open.org/ws-sx/ws-secureconversation/200512":Length of simpleType xs:unsignedLong.
/// @note Use wsdl2h option -g to auto-generate a top-level attribute declaration and processing code.


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

Data can be read in XML and deserialized from:
  - a file descriptor, using soap->recvfd = fd
  - a socket, using soap->socket = ...
  - a C++ stream, using soap->is = ...
  - a buffer, using the soap->frecv() callback

Data can be serialized in XML and written to:
  - a file descriptor, using soap->sendfd = fd
  - a socket, using soap->socket = ...
  - a C++ stream, using soap->os = ...
  - a buffer, using the soap->fsend() callback

The following options are available for (de)serialization control:
  - soap->encodingStyle = NULL; to remove SOAP 1.1/1.2 encodingStyle
  - soap_mode(soap, SOAP_XML_TREE); XML without id-ref (no cycles!)
  - soap_mode(soap, SOAP_XML_GRAPH); XML with id-ref (including cycles)
  - soap_set_namespaces(soap, struct Namespace *nsmap); to set xmlns bindings


@section wsc Top-level root elements of schema "http://docs.oasis-open.org/ws-sx/ws-secureconversation/200512"

  - <SecurityContextToken> (use wsdl2h option -g to auto-generate)

  - <Identifier> @ref _wsc__Identifier
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wsc__Identifier(struct soap*, char*);
    // Writer (returns SOAP_OK on success):
    soap_write__wsc__Identifier(struct soap*, char*);
    @endcode

  - <Instance> @ref _wsc__Instance
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wsc__Instance(struct soap*, char*);
    // Writer (returns SOAP_OK on success):
    soap_write__wsc__Instance(struct soap*, char*);
    @endcode

  - <DerivedKeyToken> (use wsdl2h option -g to auto-generate)

  - <Name> @ref _wsc__Name
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wsc__Name(struct soap*, char*);
    // Writer (returns SOAP_OK on success):
    soap_write__wsc__Name(struct soap*, char*);
    @endcode

  - <Label> @ref _wsc__Label
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wsc__Label(struct soap*, char*);
    // Writer (returns SOAP_OK on success):
    soap_write__wsc__Label(struct soap*, char*);
    @endcode

  - <Nonce> @ref _wsc__Nonce
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wsc__Nonce(struct soap*, char*);
    // Writer (returns SOAP_OK on success):
    soap_write__wsc__Nonce(struct soap*, char*);
    @endcode

*/

/* End of wsc.h */
