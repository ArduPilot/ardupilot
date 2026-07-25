/*
	wsa.h WS-Addressing 2004/08, accepts WS-Addressing 2005/08

	Usage: See plugin/wsaapi.c

	Generated with:
	wsdl2h -cgye -o wsa.h -t WS/WS-typemap.dat WS/WS-Addressing.xsd

        Requires:
        - plugin/wsaapi.h and plugin/wsaapi.c

	- Removed //gsoapopt
	- Added the following directive to import WS-Addressing namespace:
  	//gsoap wsa schema import: http://schemas.xmlsoap.org/ws/2004/08/addressing
	This ensures that the WS-Addressing schemas are not copied into the
	generated WSDL by soapcpp2 but are referenced with schema import in the
	generated WSDL.
        - Added the following directive to accept WS-Addressing 2005/08:
        //gsoap wsa schema namespace2: http://www.w3.org/2005/08/addressing
	- Added mutable SOAP_ENV__Header struct
	- Added SOAP_ENV__Fault one-way operation

Usage:

// header file for soapcpp2
#import "wsa.h"

// client-side source code
{ struct soap soap;
  struct SOAP_ENV__Header header;
  _wsa__ReplyTo replyTo;
  soap_init(&soap);
  soap_default_SOAP_ENV__Header(&soap, &header);
  soap.header = &header;
  soap_default_wsa__EndpointReferenceType(&soap, &replyTo);
  replyTo.Address = "http://schemas.xmlsoap.org/ws/2004/08/addressing/role/anony
mous";
  header.wsa__MessageID = "...";
  header.wsa__To = "...";
  header.wsa__Action = "...";
  header.wsa__ReplyTo = &replyTo;
  if (soap_call_ns__method(&soap, ...) != SOAP_OK)

// server-side source code
int soap_ns__method(struct soap *soap, ...)
{ if (!soap->header)
    return soap_sender_fault(soap, "No SOAP header, must send one", NULL);
  if (!soap->header->wsa__MessageID)
    return soap_sender_fault(soap, "No WS-Addressing MessageID", NULL);
  soap->header->wsa__RelatesTo = (struct wsa__Relationship*)soap_malloc(soap, sizeof(struct wsa__Relationship));
  soap_default_wsa__Relationship(soap, soap->header->wsa__RelatesTo);
  soap->header->wsa__RelatesTo->__item = soap->header->wsa__MessageID;
  if (!soap->header->wsa__ReplyTo || !soap->header->wsa__ReplyTo->Address)
    return soap_sender_fault(soap, "No WS-Addressing ReplyTo address", NULL);
  soap->header->wsa__To = soap->header->wsa__ReplyTo->Address;
  soap->header->wsa__MessageID = "...";
  soap->header->wsa__Action = "...";
  ...

*/

#define SOAP_WSA_200408

/******************************************************************************\
 *                                                                            *
 * http://schemas.xmlsoap.org/ws/2004/08/addressing                           *
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

#define SOAP_NAMESPACE_OF_wsa	"http://schemas.xmlsoap.org/ws/2004/08/addressing"
//gsoap wsa   schema import:	http://schemas.xmlsoap.org/ws/2004/08/addressing
//gsoap wsa   schema namespace2:	http://www.w3.org/2005/08/addressing
//gsoap wsa   schema elementForm:	qualified
//gsoap wsa   schema attributeForm:	unqualified

/******************************************************************************\
 *                                                                            *
 * Schema Types                                                               *
 *                                                                            *
\******************************************************************************/



/// Typedef synonym for struct wsa__EndpointReferenceType.
typedef struct wsa__EndpointReferenceType wsa__EndpointReferenceType;

/// Typedef synonym for struct wsa__ReferencePropertiesType.
typedef struct wsa__ReferencePropertiesType wsa__ReferencePropertiesType;

/// Typedef synonym for struct wsa__ReferenceParametersType.
typedef struct wsa__ReferenceParametersType wsa__ReferenceParametersType;

/// Typedef synonym for struct wsa__ServiceNameType.
typedef struct wsa__ServiceNameType wsa__ServiceNameType;

/// Typedef synonym for struct wsa__Relationship.
typedef struct wsa__Relationship wsa__Relationship;

/// Imported complexType "http://schemas.xmlsoap.org/ws/2004/08/addressing":ReplyAfterType from typemap WS/WS-typemap.dat.
// complexType definition intentionally left blank.

/// Imported complexType "http://schemas.xmlsoap.org/ws/2004/08/addressing":AttributedQName from typemap WS/WS-typemap.dat.
// complexType definition intentionally left blank.

/// Imported complexType "http://schemas.xmlsoap.org/ws/2004/08/addressing":AttributedURI from typemap WS/WS-typemap.dat.
// complexType definition intentionally left blank.

/// "http://schemas.xmlsoap.org/ws/2004/08/addressing":RelationshipTypeValues is a simpleType restriction of xs:QName.
enum wsa__RelationshipTypeValues
{
	wsa__Reply,	///< xs:QName value=""http://schemas.xmlsoap.org/ws/2004/08/addressing":Reply"
};
/// Typedef synonym for enum wsa__RelationshipTypeValues.
typedef enum wsa__RelationshipTypeValues wsa__RelationshipTypeValues;

/// "http://schemas.xmlsoap.org/ws/2004/08/addressing":FaultSubcodeValues is a simpleType restriction of xs:QName.
enum wsa__FaultSubcodeValues
{
	wsa__InvalidMessageInformationHeader,	///< xs:QName value=""http://schemas.xmlsoap.org/ws/2004/08/addressing":InvalidMessageInformationHeader"
	wsa__MessageInformationHeaderRequired,	///< xs:QName value=""http://schemas.xmlsoap.org/ws/2004/08/addressing":MessageInformationHeaderRequired"
	wsa__DestinationUnreachable,	///< xs:QName value=""http://schemas.xmlsoap.org/ws/2004/08/addressing":DestinationUnreachable"
	wsa__ActionNotSupported,	///< xs:QName value=""http://schemas.xmlsoap.org/ws/2004/08/addressing":ActionNotSupported"
	wsa__EndpointUnavailable,	///< xs:QName value=""http://schemas.xmlsoap.org/ws/2004/08/addressing":EndpointUnavailable"
};
/// Typedef synonym for enum wsa__FaultSubcodeValues.
typedef enum wsa__FaultSubcodeValues wsa__FaultSubcodeValues;

/// "http://schemas.xmlsoap.org/ws/2004/08/addressing":EndpointReferenceType is a complexType.
struct wsa__EndpointReferenceType
{
/// Element Address of type "http://schemas.xmlsoap.org/ws/2004/08/addressing":AttributedURI.
    char*                                Address                        1;	///< Required element.
/// Element ReferenceProperties of type "http://schemas.xmlsoap.org/ws/2004/08/addressing":ReferencePropertiesType.
    struct wsa__ReferencePropertiesType*  ReferenceProperties            0;	///< Optional element.
/// Element ReferenceParameters of type "http://schemas.xmlsoap.org/ws/2004/08/addressing":ReferenceParametersType.
    struct wsa__ReferenceParametersType*  ReferenceParameters            0;	///< Optional element.
/// Element PortType of type "http://schemas.xmlsoap.org/ws/2004/08/addressing":AttributedQName.
    _QName*                              PortType                       0;	///< Optional element.
/// Element ServiceName of type "http://schemas.xmlsoap.org/ws/2004/08/addressing":ServiceNameType.
    struct wsa__ServiceNameType*         ServiceName                    0;	///< Optional element.
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

/// "http://schemas.xmlsoap.org/ws/2004/08/addressing":ReferencePropertiesType is a complexType.
struct wsa__ReferencePropertiesType
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

/// "http://schemas.xmlsoap.org/ws/2004/08/addressing":ReferenceParametersType is a complexType.
struct wsa__ReferenceParametersType
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

/// "http://schemas.xmlsoap.org/ws/2004/08/addressing":ServiceNameType is a complexType with simpleContent.
struct wsa__ServiceNameType
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

/// "http://schemas.xmlsoap.org/ws/2004/08/addressing":Relationship is a complexType with simpleContent.
struct wsa__Relationship
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

/// Element "http://schemas.xmlsoap.org/ws/2004/08/addressing":EndpointReference of type "http://schemas.xmlsoap.org/ws/2004/08/addressing":EndpointReferenceType.
typedef struct wsa__EndpointReferenceType _wsa__EndpointReference;

/// Element "http://schemas.xmlsoap.org/ws/2004/08/addressing":MessageID of type "http://schemas.xmlsoap.org/ws/2004/08/addressing":AttributedURI.
typedef char* _wsa__MessageID;

/// Element "http://schemas.xmlsoap.org/ws/2004/08/addressing":RelatesTo of type "http://schemas.xmlsoap.org/ws/2004/08/addressing":Relationship.
typedef struct wsa__Relationship _wsa__RelatesTo;

/// Element "http://schemas.xmlsoap.org/ws/2004/08/addressing":To of type "http://schemas.xmlsoap.org/ws/2004/08/addressing":AttributedURI.
typedef char* _wsa__To;

/// Element "http://schemas.xmlsoap.org/ws/2004/08/addressing":Action of type "http://schemas.xmlsoap.org/ws/2004/08/addressing":AttributedURI.
typedef char* _wsa__Action;

/// Element "http://schemas.xmlsoap.org/ws/2004/08/addressing":From of type "http://schemas.xmlsoap.org/ws/2004/08/addressing":EndpointReferenceType.
typedef struct wsa__EndpointReferenceType _wsa__From;

/// Element "http://schemas.xmlsoap.org/ws/2004/08/addressing":ReplyTo of type "http://schemas.xmlsoap.org/ws/2004/08/addressing":EndpointReferenceType.
typedef struct wsa__EndpointReferenceType _wsa__ReplyTo;

/// Element "http://schemas.xmlsoap.org/ws/2004/08/addressing":FaultTo of type "http://schemas.xmlsoap.org/ws/2004/08/addressing":EndpointReferenceType.
typedef struct wsa__EndpointReferenceType _wsa__FaultTo;

/// Element "http://schemas.xmlsoap.org/ws/2004/08/addressing":ReplyAfter of type "http://schemas.xmlsoap.org/ws/2004/08/addressing":ReplyAfterType.
typedef unsigned int _wsa__ReplyAfter;

/// Attribute "http://schemas.xmlsoap.org/ws/2004/08/addressing":Action of simpleType xs:anyURI.
// '_wsa__Action' attribute definition intentionally left blank.

mutable struct SOAP_ENV__Header
{
                 _wsa__MessageID  wsa__MessageID 0;
                 _wsa__RelatesTo *wsa__RelatesTo 0;
                 _wsa__From      *wsa__From      0;
  mustUnderstand _wsa__ReplyTo   *wsa__ReplyTo   0;
  mustUnderstand _wsa__FaultTo   *wsa__FaultTo   0;
  mustUnderstand _wsa__To         wsa__To        0;
  mustUnderstand _wsa__Action     wsa__Action    0;
};

// Added
//gsoap SOAP_ENV service method-action: Fault http://schemas.xmlsoap.org/ws/2004/08/addressing/soap/fault
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

/* End of wsa.h */
