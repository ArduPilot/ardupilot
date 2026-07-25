/*
	wsa5.h WS-Addressing 2005/08, accepts WS-Addressing 2004/08

	Usage: See plugin/wsaapi.c

	Generated with:
	wsdl2h -cgye -o wsa5.h -t WS/WS-typemap.dat WS/WS-Addressing05.xsd

        Requires:
        - plugin/wsaapi.h and plugin/wsaapi.c

	- Removed //gsoapopt
	- Removed xsd__boolean declaration
	- Added the following directive to import WS-Addressing namespace:
  	//gsoap wsa5  schema import:	http://www.w3.org/2005/08/addressing
	This ensures that the WS-Addressing schemas are not copied into the
	generated WSDL by soapcpp2 but are referenced with schema import in the
	generated WSDL.
	- Added //gsoap wsa5  schema namespace2: http://schemas.xmlsoap.org/ws/2004/08/addressing
	- Added #define SOAP_WSA_2005
	- Added mutable SOAP_ENV__Header struct
	- Added SOAP_ENV__Fault one-way operation
	- Added //gsoap chan schema import: http://schemas.microsoft.com/ws/2005/02/duplex
	- Added chan__ChannelInstance to wsa5__ReferenceParametersType
	- Added chan__ChannelInstanceType and chan__ChannelInstance to Header

*/

#define SOAP_WSA_2005

/******************************************************************************\
 *                                                                            *
 * http://www.w3.org/2005/08/addressing                                       *
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

#define SOAP_NAMESPACE_OF_wsa5	"http://www.w3.org/2005/08/addressing"
//gsoap wsa5  schema import:		http://www.w3.org/2005/08/addressing
//gsoap wsa5  schema namespace2:	http://schemas.xmlsoap.org/ws/2004/08/addressing
//gsoap wsa5  schema elementForm:	qualified
//gsoap wsa5  schema attributeForm:	unqualified

/******************************************************************************\
 *                                                                            *
 * Schema Types                                                               *
 *                                                                            *
\******************************************************************************/


/// Typedef synonym for struct wsa5__EndpointReferenceType.
typedef struct wsa5__EndpointReferenceType wsa5__EndpointReferenceType;

/// Typedef synonym for struct wsa5__ReferenceParametersType.
typedef struct wsa5__ReferenceParametersType wsa5__ReferenceParametersType;

/// Typedef synonym for struct wsa5__MetadataType.
typedef struct wsa5__MetadataType wsa5__MetadataType;

/// Typedef synonym for struct wsa5__RelatesToType.
typedef struct wsa5__RelatesToType wsa5__RelatesToType;

/// Imported complexType "http://www.w3.org/2005/08/addressing":AttributedURIType from typemap WS/WS-typemap.dat.
// complexType definition intentionally left blank.

/// Imported complexType "http://www.w3.org/2005/08/addressing":AttributedUnsignedLongType from typemap WS/WS-typemap.dat.
// complexType definition intentionally left blank.

/// Imported complexType "http://www.w3.org/2005/08/addressing":AttributedQNameType from typemap WS/WS-typemap.dat.
// complexType definition intentionally left blank.

/// Typedef synonym for struct wsa5__ProblemActionType.
typedef struct wsa5__ProblemActionType wsa5__ProblemActionType;

/// union of values "tns:RelationshipType xs:anyURI"
typedef char* wsa5__RelationshipTypeOpenEnum;

/// union of values "tns:FaultCodesType xs:QName"
typedef char* wsa5__FaultCodesOpenEnumType;

/// "http://www.w3.org/2005/08/addressing":RelationshipType is a simpleType restriction of xs:anyURI.
enum wsa5__RelationshipType
{
	http_x003a_x002f_x002fwww_x002ew3_x002eorg_x002f2005_x002f08_x002faddressing_x002freply,	///< xs:anyURI value="http://www.w3.org/2005/08/addressing/reply"
};
/// Typedef synonym for enum wsa5__RelationshipType.
typedef enum wsa5__RelationshipType wsa5__RelationshipType;

/// "http://www.w3.org/2005/08/addressing":FaultCodesType is a simpleType restriction of xs:QName.
enum wsa5__FaultCodesType
{
	wsa5__InvalidAddressingHeader,	///< xs:QName value=""http://www.w3.org/2005/08/addressing":InvalidAddressingHeader"
	wsa5__InvalidAddress,	///< xs:QName value=""http://www.w3.org/2005/08/addressing":InvalidAddress"
	wsa5__InvalidEPR,	///< xs:QName value=""http://www.w3.org/2005/08/addressing":InvalidEPR"
	wsa5__InvalidCardinality,	///< xs:QName value=""http://www.w3.org/2005/08/addressing":InvalidCardinality"
	wsa5__MissingAddressInEPR,	///< xs:QName value=""http://www.w3.org/2005/08/addressing":MissingAddressInEPR"
	wsa5__DuplicateMessageID,	///< xs:QName value=""http://www.w3.org/2005/08/addressing":DuplicateMessageID"
	wsa5__ActionMismatch,	///< xs:QName value=""http://www.w3.org/2005/08/addressing":ActionMismatch"
	wsa5__MessageAddressingHeaderRequired,	///< xs:QName value=""http://www.w3.org/2005/08/addressing":MessageAddressingHeaderRequired"
	wsa5__DestinationUnreachable,	///< xs:QName value=""http://www.w3.org/2005/08/addressing":DestinationUnreachable"
	wsa5__ActionNotSupported,	///< xs:QName value=""http://www.w3.org/2005/08/addressing":ActionNotSupported"
	wsa5__EndpointUnavailable,	///< xs:QName value=""http://www.w3.org/2005/08/addressing":EndpointUnavailable"
};
/// Typedef synonym for enum wsa5__FaultCodesType.
typedef enum wsa5__FaultCodesType wsa5__FaultCodesType;

/// "http://www.w3.org/2005/08/addressing":EndpointReferenceType is a complexType.
struct wsa5__EndpointReferenceType
{
/// Element Address of type "http://www.w3.org/2005/08/addressing":AttributedURIType.
    char* /*URI*/                        Address                        1;	///< Required element.
/// Element reference "http://www.w3.org/2005/08/addressing":ReferenceParameters.
    struct wsa5__ReferenceParametersType*  ReferenceParameters            0;	///< Optional element.
/// Element reference "http://www.w3.org/2005/08/addressing":Metadata.
    struct wsa5__MetadataType*           Metadata                       0;	///< Optional element.
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

/// "http://www.w3.org/2005/08/addressing":ReferenceParametersType is a complexType.
struct wsa5__ReferenceParametersType
{
// Added
    int                                 *chan__ChannelInstance          0;
/// TODO: <any namespace="##any" minOccurs="0" maxOccurs="unbounded">
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

/// "http://www.w3.org/2005/08/addressing":MetadataType is a complexType.
struct wsa5__MetadataType
{
/// TODO: <any namespace="##any" minOccurs="0" maxOccurs="unbounded">
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

/// "http://www.w3.org/2005/08/addressing":ProblemActionType is a complexType.
struct wsa5__ProblemActionType
{
/// Element reference "http://www.w3.org/2005/08/addressing":Action.
    char* /*URI*/                        Action                         0;	///< Optional element.
/// Element SoapAction of type xs:anyURI.
    char* /*URI*/                        SoapAction                     0;	///< Optional element.
/// TODO: <anyAttribute namespace="##other">
///       Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
   @_XML                                 __anyAttribute                ;	///< Catch any attribute content in XML string.
};

/// "http://www.w3.org/2005/08/addressing":RelatesToType is a complexType with simpleContent.
struct wsa5__RelatesToType
{
/// __item wraps 'xs:anyURI' simpleContent.
    char* /*URI*/                        __item                        ;
/// Attribute RelationshipType of type "http://www.w3.org/2005/08/addressing":RelationshipTypeOpenEnum.
   @wsa5__RelationshipTypeOpenEnum       RelationshipType               0;	///< Optional attribute.
/// TODO: <anyAttribute namespace="##other">
///       Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
   @_XML                                 __anyAttribute                ;	///< Catch any attribute content in XML string.
};

/// Element "http://www.w3.org/2005/08/addressing":EndpointReference of type "http://www.w3.org/2005/08/addressing":EndpointReferenceType.
typedef struct wsa5__EndpointReferenceType _wsa5__EndpointReference;

/// Element "http://www.w3.org/2005/08/addressing":ReferenceParameters of type "http://www.w3.org/2005/08/addressing":ReferenceParametersType.
typedef struct wsa5__ReferenceParametersType _wsa5__ReferenceParameters;

/// Element "http://www.w3.org/2005/08/addressing":Metadata of type "http://www.w3.org/2005/08/addressing":MetadataType.
typedef struct wsa5__MetadataType _wsa5__Metadata;

/// Element "http://www.w3.org/2005/08/addressing":MessageID of type "http://www.w3.org/2005/08/addressing":AttributedURIType.
typedef char* /*URI*/ _wsa5__MessageID;

/// Element "http://www.w3.org/2005/08/addressing":RelatesTo of type "http://www.w3.org/2005/08/addressing":RelatesToType.
typedef struct wsa5__RelatesToType _wsa5__RelatesTo;

/// Element "http://www.w3.org/2005/08/addressing":ReplyTo of type "http://www.w3.org/2005/08/addressing":EndpointReferenceType.
typedef struct wsa5__EndpointReferenceType _wsa5__ReplyTo;

/// Element "http://www.w3.org/2005/08/addressing":From of type "http://www.w3.org/2005/08/addressing":EndpointReferenceType.
typedef struct wsa5__EndpointReferenceType _wsa5__From;

/// Element "http://www.w3.org/2005/08/addressing":FaultTo of type "http://www.w3.org/2005/08/addressing":EndpointReferenceType.
typedef struct wsa5__EndpointReferenceType _wsa5__FaultTo;

/// Element "http://www.w3.org/2005/08/addressing":To of type "http://www.w3.org/2005/08/addressing":AttributedURIType.
typedef char* /*URI*/ _wsa5__To;

/// Element "http://www.w3.org/2005/08/addressing":Action of type "http://www.w3.org/2005/08/addressing":AttributedURIType.
typedef char* /*URI*/ _wsa5__Action;

/// Element "http://www.w3.org/2005/08/addressing":RetryAfter of type "http://www.w3.org/2005/08/addressing":AttributedUnsignedLongType.
typedef ULONG64 _wsa5__RetryAfter;

/// Element "http://www.w3.org/2005/08/addressing":ProblemHeaderQName of type "http://www.w3.org/2005/08/addressing":AttributedQNameType.
typedef _QName _wsa5__ProblemHeaderQName;

/// Element "http://www.w3.org/2005/08/addressing":ProblemIRI of type "http://www.w3.org/2005/08/addressing":AttributedURIType.
typedef char* /*URI*/ _wsa5__ProblemIRI;

/// Element "http://www.w3.org/2005/08/addressing":ProblemAction of type "http://www.w3.org/2005/08/addressing":ProblemActionType.
typedef struct wsa5__ProblemActionType _wsa5__ProblemAction;

/// Attribute "http://www.w3.org/2005/08/addressing":IsReferenceParameter of simpleType xs:boolean.
/// Imported attribute _wsa5__IsReferenceParameter from typemap WS/WS-typemap.dat.
typedef enum _wsa5__IsReferenceParameter { _wsa5__IsReferenceParameter__false, _wsa5__IsReferenceParameter__true } _wsa5__IsReferenceParameter;

// Added
//gsoap chan schema import: http://schemas.microsoft.com/ws/2005/02/duplex
/// "http://schemas.microsoft.com/ws/2005/02/duplex":ChannelInstanceType is a complexType.
struct chan__ChannelInstanceType
{   int __item;
    @_wsa5__IsReferenceParameter wsa5__IsReferenceParameter = _wsa5__IsReferenceParameter__false;
};

/// Added
mutable struct SOAP_ENV__Header
{
                 _wsa5__MessageID  wsa5__MessageID 0;
                 _wsa5__RelatesTo *wsa5__RelatesTo 0;
                 _wsa5__From      *wsa5__From      0;
  mustUnderstand _wsa5__ReplyTo   *wsa5__ReplyTo   0;
  mustUnderstand _wsa5__FaultTo   *wsa5__FaultTo   0;
  mustUnderstand _wsa5__To         wsa5__To        0;
  mustUnderstand _wsa5__Action     wsa5__Action    0;
                 struct chan__ChannelInstanceType *chan__ChannelInstance 0;
};

// Added
//gsoap SOAP_ENV service method-action: Fault http://www.w3.org/2005/08/addressing/soap/fault
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

/* End of wsa5.h */
