/*

	wsa4.h WS-Addressing 2004/03

	Usage: See plugin/wsaapi.c

	Generated with:
	wsdl2h -cgye -o wsa4.h -t WS/WS-typemap.dat WS/WS-Addressing04.xsd

        Requires:
        - plugin/wsaapi.h and plugin/wsaapi.c

	- Removed //gsoapopt
	- Added the following directive to import WS-Addressing namespace:
	  //gsoap wsa4 schema import: http://schemas.xmlsoap.org/ws/2004/03/addressing
	  This ensures that the WS-Addressing schemas are not copied into the generated
	  WSDL by soapcpp2 but are referenced with schema import in the generated WSDL.
	- Added #define SOAP_WSA_2004
	- Added mutable SOAP_ENV__Header struct
	- Added SOAP_ENV__Fault one-way operation

*/

#define SOAP_WSA_2004

/******************************************************************************\
 *                                                                            *
 * http://schemas.xmlsoap.org/ws/2004/03/addressing                           *
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

#define SOAP_NAMESPACE_OF_wsa4	"http://schemas.xmlsoap.org/ws/2004/03/addressing"
//gsoap wsa4  schema import:	http://schemas.xmlsoap.org/ws/2004/03/addressing
//gsoap wsa4  schema elementForm:	qualified
//gsoap wsa4  schema attributeForm:	unqualified

/******************************************************************************\
 *                                                                            *
 * Schema Types                                                               *
 *                                                                            *
\******************************************************************************/



/// Typedef synonym for struct wsa4__EndpointReferenceType.
typedef struct wsa4__EndpointReferenceType wsa4__EndpointReferenceType;

/// Typedef synonym for struct wsa4__ReferencePropertiesType.
typedef struct wsa4__ReferencePropertiesType wsa4__ReferencePropertiesType;

/// Typedef synonym for struct wsa4__ServiceNameType.
typedef struct wsa4__ServiceNameType wsa4__ServiceNameType;

/// Typedef synonym for struct wsa4__Relationship.
typedef struct wsa4__Relationship wsa4__Relationship;

/// Imported complexType "http://schemas.xmlsoap.org/ws/2004/03/addressing":ReplyAfterType from typemap WS/WS-typemap.dat.
// complexType definition intentionally left blank.

/// Imported complexType "http://schemas.xmlsoap.org/ws/2004/03/addressing":AttributedQName from typemap WS/WS-typemap.dat.
// complexType definition intentionally left blank.

/// Imported complexType "http://schemas.xmlsoap.org/ws/2004/03/addressing":AttributedURI from typemap WS/WS-typemap.dat.
// complexType definition intentionally left blank.

/// Element "http://schemas.xmlsoap.org/ws/2004/03/addressing":ReplyAfter.

/// "http://schemas.xmlsoap.org/ws/2004/03/addressing":RelationshipTypeValues is a simpleType restriction of xs:QName.
enum wsa4__RelationshipTypeValues
{
	wsa4__Reply,	///< xs:QName value=""http://schemas.xmlsoap.org/ws/2004/03/addressing":Reply"
};
/// Typedef synonym for enum wsa4__RelationshipTypeValues.
typedef enum wsa4__RelationshipTypeValues wsa4__RelationshipTypeValues;

/// "http://schemas.xmlsoap.org/ws/2004/03/addressing":FaultSubcodeValues is a simpleType restriction of xs:QName.
enum wsa4__FaultSubcodeValues
{
	wsa4__InvalidMessageInformationHeader,	///< xs:QName value=""http://schemas.xmlsoap.org/ws/2004/03/addressing":InvalidMessageInformationHeader"
	wsa4__MessageInformationHeaderRequired,	///< xs:QName value=""http://schemas.xmlsoap.org/ws/2004/03/addressing":MessageInformationHeaderRequired"
	wsa4__DestinationUnreachable,	///< xs:QName value=""http://schemas.xmlsoap.org/ws/2004/03/addressing":DestinationUnreachable"
	wsa4__ActionNotSupported,	///< xs:QName value=""http://schemas.xmlsoap.org/ws/2004/03/addressing":ActionNotSupported"
	wsa4__EndpointUnavailable,	///< xs:QName value=""http://schemas.xmlsoap.org/ws/2004/03/addressing":EndpointUnavailable"
};
/// Typedef synonym for enum wsa4__FaultSubcodeValues.
typedef enum wsa4__FaultSubcodeValues wsa4__FaultSubcodeValues;

/// "http://schemas.xmlsoap.org/ws/2004/03/addressing":EndpointReferenceType is a complexType.
struct wsa4__EndpointReferenceType
{
/// Element Address of type "http://schemas.xmlsoap.org/ws/2004/03/addressing":AttributedURI.
    char* /*URI*/                        Address                        1;	///< Required element.
/// Element ReferenceProperties of type "http://schemas.xmlsoap.org/ws/2004/03/addressing":ReferencePropertiesType.
    struct wsa4__ReferencePropertiesType*  ReferenceProperties            0;	///< Optional element.
/// Element PortType of type "http://schemas.xmlsoap.org/ws/2004/03/addressing":AttributedQName.
    _QName*                              PortType                       0;	///< Optional element.
/// Element ServiceName of type "http://schemas.xmlsoap.org/ws/2004/03/addressing":ServiceNameType.
    struct wsa4__ServiceNameType*        ServiceName                    0;	///< Optional element.
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

/// "http://schemas.xmlsoap.org/ws/2004/03/addressing":ReferencePropertiesType is a complexType.
struct wsa4__ReferencePropertiesType
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

/// "http://schemas.xmlsoap.org/ws/2004/03/addressing":ServiceNameType is a complexType with simpleContent.
struct wsa4__ServiceNameType
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

/// "http://schemas.xmlsoap.org/ws/2004/03/addressing":Relationship is a complexType with simpleContent.
struct wsa4__Relationship
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

/// Element "http://schemas.xmlsoap.org/ws/2004/03/addressing":EndpointReference of type "http://schemas.xmlsoap.org/ws/2004/03/addressing":EndpointReferenceType.
typedef struct wsa4__EndpointReferenceType _wsa4__EndpointReference;

/// Element "http://schemas.xmlsoap.org/ws/2004/03/addressing":MessageID of type "http://schemas.xmlsoap.org/ws/2004/03/addressing":AttributedURI.
typedef char* /*URI*/ _wsa4__MessageID;

/// Element "http://schemas.xmlsoap.org/ws/2004/03/addressing":RelatesTo of type "http://schemas.xmlsoap.org/ws/2004/03/addressing":Relationship.
typedef struct wsa4__Relationship _wsa4__RelatesTo;

/// Element "http://schemas.xmlsoap.org/ws/2004/03/addressing":To of type "http://schemas.xmlsoap.org/ws/2004/03/addressing":AttributedURI.
typedef char* /*URI*/ _wsa4__To;

/// Element "http://schemas.xmlsoap.org/ws/2004/03/addressing":Action of type "http://schemas.xmlsoap.org/ws/2004/03/addressing":AttributedURI.
typedef char* /*URI*/ _wsa4__Action;

/// Element "http://schemas.xmlsoap.org/ws/2004/03/addressing":From of type "http://schemas.xmlsoap.org/ws/2004/03/addressing":EndpointReferenceType.
typedef struct wsa4__EndpointReferenceType _wsa4__From;

/// Element "http://schemas.xmlsoap.org/ws/2004/03/addressing":ReplyTo of type "http://schemas.xmlsoap.org/ws/2004/03/addressing":EndpointReferenceType.
typedef struct wsa4__EndpointReferenceType _wsa4__ReplyTo;

/// Element "http://schemas.xmlsoap.org/ws/2004/03/addressing":FaultTo of type "http://schemas.xmlsoap.org/ws/2004/03/addressing":EndpointReferenceType.
typedef struct wsa4__EndpointReferenceType _wsa4__FaultTo;

/// Attribute "http://schemas.xmlsoap.org/ws/2004/03/addressing":Action of simpleType xs:anyURI.
// '_wsa4__Action' attribute definition intentionally left blank.

mutable struct SOAP_ENV__Header
{
                 _wsa4__MessageID  wsa4__MessageID 0;
                 _wsa4__RelatesTo *wsa4__RelatesTo 0;
                 _wsa4__From      *wsa4__From      0;
  mustUnderstand _wsa4__ReplyTo   *wsa4__ReplyTo   0;
  mustUnderstand _wsa4__FaultTo   *wsa4__FaultTo   0;
  mustUnderstand _wsa4__To         wsa4__To        0;
  mustUnderstand _wsa4__Action     wsa4__Action    0;
};

// Added
//gsoap SOAP_ENV service method-action: Fault http://schemas.xmlsoap.org/ws/2004/03/addressing/soap/fault
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

/* End of wsa4.h */
