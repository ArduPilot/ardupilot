/*

	wsa3.h WS-Addressing 2003/03

	Usage: See plugin/wsaapi.c

	Generated with:
	wsdl2h -cgye -o wsa3.h -t WS/WS-typemap.dat WS/WS-Addressing03.xsd

        Requires:
        - plugin/wsaapi.h and plugin/wsaapi.c

	- Removed //gsoapopt
	- Added the following directive to import WS-Addressing namespace:
	  //gsoap wsa4 schema import: http://schemas.xmlsoap.org/ws/2003/03/addressing
	  This ensures that the WS-Addressing schemas are not copied into the generated
	  WSDL by soapcpp2 but are referenced with schema import in the generated WSDL.
	- Added #define SOAP_WSA_2003
	- Added mutable SOAP_ENV__Header struct
	- Added SOAP_ENV__Fault one-way operation

*/

#define SOAP_WSA_2003


/******************************************************************************\
 *                                                                            *
 * http://schemas.xmlsoap.org/ws/2003/03/addressing                           *
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

#define SOAP_NAMESPACE_OF_wsa3	"http://schemas.xmlsoap.org/ws/2003/03/addressing"
//gsoap wsa3  schema import:	http://schemas.xmlsoap.org/ws/2003/03/addressing
//gsoap wsa3  schema elementForm:	qualified
//gsoap wsa3  schema attributeForm:	unqualified

/******************************************************************************\
 *                                                                            *
 * Schema Types                                                               *
 *                                                                            *
\******************************************************************************/



/// Typedef synonym for struct wsa3__EndpointReferenceType.
typedef struct wsa3__EndpointReferenceType wsa3__EndpointReferenceType;

/// Typedef synonym for struct wsa3__ReferencePropertiesType.
typedef struct wsa3__ReferencePropertiesType wsa3__ReferencePropertiesType;

/// Typedef synonym for struct wsa3__ServiceNameType.
typedef struct wsa3__ServiceNameType wsa3__ServiceNameType;

/// Typedef synonym for struct wsa3__Relationship.
typedef struct wsa3__Relationship wsa3__Relationship;

/// Imported complexType "http://schemas.xmlsoap.org/ws/2003/03/addressing":AttributedQName from typemap WS/WS-typemap.dat.
// complexType definition intentionally left blank.

/// Imported complexType "http://schemas.xmlsoap.org/ws/2003/03/addressing":AttributedURI from typemap WS/WS-typemap.dat.
// complexType definition intentionally left blank.

/// "http://schemas.xmlsoap.org/ws/2003/03/addressing":RelationshipTypeValues is a simpleType restriction of xs:QName.
enum wsa3__RelationshipTypeValues
{
	wsa3__Response,	///< xs:QName value=""http://schemas.xmlsoap.org/ws/2003/03/addressing":Response"
};
/// Typedef synonym for enum wsa3__RelationshipTypeValues.
typedef enum wsa3__RelationshipTypeValues wsa3__RelationshipTypeValues;

/// "http://schemas.xmlsoap.org/ws/2003/03/addressing":EndpointReferenceType is a complexType.
struct wsa3__EndpointReferenceType
{
/// Element Address of type "http://schemas.xmlsoap.org/ws/2003/03/addressing":AttributedURI.
    char* /*URI*/                        Address                        1;	///< Required element.
/// Element ReferenceProperties of type "http://schemas.xmlsoap.org/ws/2003/03/addressing":ReferencePropertiesType.
    struct wsa3__ReferencePropertiesType*  ReferenceProperties            0;	///< Optional element.
/// Element PortType of type "http://schemas.xmlsoap.org/ws/2003/03/addressing":AttributedQName.
    _QName*                              PortType                       0;	///< Optional element.
/// Element ServiceName of type "http://schemas.xmlsoap.org/ws/2003/03/addressing":ServiceNameType.
    struct wsa3__ServiceNameType*        ServiceName                    0;	///< Optional element.
/// TODO: <any namespace="##other" minOccurs="0" maxOccurs="unbounded">
///       Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d to use xsd__anyType DOM.
/// Size of the dynamic array of XML is 0..unbounded
    int                                  __size                        ;
    _XML                                *__any                         ;	///< Catch any element content in XML string.
/// TODO: <anyAttribute namespace="##other">
///       Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
   @_XML                                 __anyAttribute                ;	///< Catch any attribute content in XML string.
};

/// "http://schemas.xmlsoap.org/ws/2003/03/addressing":ReferencePropertiesType is a complexType.
struct wsa3__ReferencePropertiesType
{
/// TODO: <any minOccurs="0" maxOccurs="unbounded">
///       Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d to use xsd__anyType DOM.
/// Size of the dynamic array of XML is 0..unbounded
    int                                  __size                        ;
    _XML                                *__any                         ;	///< Catch any element content in XML string.
};

/// "http://schemas.xmlsoap.org/ws/2003/03/addressing":ServiceNameType is a complexType with simpleContent.
struct wsa3__ServiceNameType
{
/// __item wraps 'xs:QName' simpleContent.
    _QName                               __item                        ;
/// Attribute PortName of type xs:NCName.
   @char* /*NCName*/                     PortName                       0;	///< Optional attribute.
/// TODO: <anyAttribute namespace="##other">
///       Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
   @_XML                                 __anyAttribute                ;	///< Catch any attribute content in XML string.
};

/// "http://schemas.xmlsoap.org/ws/2003/03/addressing":Relationship is a complexType with simpleContent.
struct wsa3__Relationship
{
/// __item wraps 'xs:anyURI' simpleContent.
    char* /*URI*/                        __item                        ;
/// Attribute RelationshipType of type xs:QName.
   @_QName                               RelationshipType               0;	///< Optional attribute.
/// TODO: <anyAttribute namespace="##other">
///       Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
   @_XML                                 __anyAttribute                ;	///< Catch any attribute content in XML string.
};

/// Element "http://schemas.xmlsoap.org/ws/2003/03/addressing":EndpointReference of type "http://schemas.xmlsoap.org/ws/2003/03/addressing":EndpointReferenceType.
typedef struct wsa3__EndpointReferenceType _wsa3__EndpointReference;

/// Element "http://schemas.xmlsoap.org/ws/2003/03/addressing":MessageID of type "http://schemas.xmlsoap.org/ws/2003/03/addressing":AttributedURI.
typedef char* /*URI*/ _wsa3__MessageID;

/// Element "http://schemas.xmlsoap.org/ws/2003/03/addressing":RelatesTo of type "http://schemas.xmlsoap.org/ws/2003/03/addressing":Relationship.
typedef struct wsa3__Relationship _wsa3__RelatesTo;

/// Element "http://schemas.xmlsoap.org/ws/2003/03/addressing":To of type "http://schemas.xmlsoap.org/ws/2003/03/addressing":AttributedURI.
typedef char* /*URI*/ _wsa3__To;

/// Element "http://schemas.xmlsoap.org/ws/2003/03/addressing":Action of type "http://schemas.xmlsoap.org/ws/2003/03/addressing":AttributedURI.
typedef char* /*URI*/ _wsa3__Action;

/// Element "http://schemas.xmlsoap.org/ws/2003/03/addressing":From of type "http://schemas.xmlsoap.org/ws/2003/03/addressing":EndpointReferenceType.
typedef struct wsa3__EndpointReferenceType _wsa3__From;

/// Element "http://schemas.xmlsoap.org/ws/2003/03/addressing":ReplyTo of type "http://schemas.xmlsoap.org/ws/2003/03/addressing":EndpointReferenceType.
typedef struct wsa3__EndpointReferenceType _wsa3__ReplyTo;

/// Element "http://schemas.xmlsoap.org/ws/2003/03/addressing":FaultTo of type "http://schemas.xmlsoap.org/ws/2003/03/addressing":EndpointReferenceType.
typedef struct wsa3__EndpointReferenceType _wsa3__FaultTo;

/// Element "http://schemas.xmlsoap.org/ws/2003/03/addressing":Recipient of type "http://schemas.xmlsoap.org/ws/2003/03/addressing":EndpointReferenceType.
typedef struct wsa3__EndpointReferenceType _wsa3__Recipient;

mutable struct SOAP_ENV__Header
{
                 _wsa3__MessageID  wsa3__MessageID 0;
                 _wsa3__RelatesTo *wsa3__RelatesTo 0;
                 _wsa3__From      *wsa3__From      0;
  mustUnderstand _wsa3__ReplyTo   *wsa3__ReplyTo   0;
  mustUnderstand _wsa3__FaultTo   *wsa3__FaultTo   0;
  mustUnderstand _wsa3__To         wsa3__To        0;
  mustUnderstand _wsa3__Action     wsa3__Action    0;
};

// Added
//gsoap SOAP_ENV service method-action: Fault http://schemas.xmlsoap.org/ws/2003/03/addressing/soap/fault
int SOAP_ENV__Fault
(       _QName			 faultcode,		// SOAP 1.1
        char			*faultstring,		// SOAP 1.1
        char			*faultactor,		// SOAP 1.1
        struct SOAP_ENV__Detail	*detail,		// SOAP 1.1
        struct SOAP_ENV__Code	*SOAP_ENV__Code,	// SOAP 1.2
        struct SOAP_ENV__Reason	*SOAP_ENV__Reason,	// SOAP 1.2
        char			*SOAP_ENV__Node,	// SOAP 1.2
        char			*SOAP_ENV__Role,	// SOAP 1.2
        struct SOAP_ENV__Detail	*SOAP_ENV__Detail,	// SOAP 1.2
	void
);

/* End of wsa3.h */
