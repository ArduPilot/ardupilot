/*
	wsrm4.h WS-ReliableMessaging 1.0 2004/06

THIS IS AN OBSOLETE SPECIFICATION superseded by WS-ReliableMessaging 2005/08.
Support for this version has ceased.

Generated with:

wsdl2h -cgex -o wsrm4.h -t WS/WS-typemap.dat WS/ws-reliability-1.1.xsd

Modified by Robert van Engelen:

- Removed //gsoapopt
- Changed the //gsoap schema namespace directives to imports

*/

/******************************************************************************\
 *                                                                            *
 * http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd             *
 *                                                                            *
\******************************************************************************/


/******************************************************************************\
 *                                                                            *
 * Import                                                                     *
 *                                                                            *
\******************************************************************************/

#import "ref.h"	// ref = <http://docs.oasis-open.org/wsrm/2004/06/reference-1.1.xsd>

/******************************************************************************\
 *                                                                            *
 * Schema Namespaces                                                          *
 *                                                                            *
\******************************************************************************/

//gsoap wsrm  schema import:	http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd
//gsoap wsrm  schema elementForm:	qualified
//gsoap wsrm  schema attributeForm:	unqualified

/******************************************************************************\
 *                                                                            *
 * Schema Types                                                               *
 *                                                                            *
\******************************************************************************/


// Imported type "http://docs.oasis-open.org/wsrm/2004/06/reference-1.1.xsd":ServiceRefType defined by ref__ServiceRefType

/// Built-in type "xs:boolean".
enum xsd__boolean_ { _false, _true };


/// "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":ReplyPatternTypeValues is a simpleType restriction of xs:string.
enum wsrm__ReplyPatternTypeValues
{
	Response,	///< xs:string value="Response"
	Callback,	///< xs:string value="Callback"
	Poll,	///< xs:string value="Poll"
};

/// "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":MIDType is a simpleType restriction of xs:anyURI.
typedef char* /*URI*/ wsrm__MIDType;

/// "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":FaultCodeEnum is a simpleType restriction of xs:QName.
enum wsrm__FaultCodeEnum
{
	wsrm__InvalidMessageId,	///< xs:QName value=""http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":InvalidMessageId"
	wsrm__InvalidMessageParameters,	///< xs:QName value=""http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":InvalidMessageParameters"
	wsrm__InvalidPollRequest,	///< xs:QName value=""http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":InvalidPollRequest"
	wsrm__InvalidRequest,	///< xs:QName value=""http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":InvalidRequest"
	wsrm__InvalidExpiryTime,	///< xs:QName value=""http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":InvalidExpiryTime"
	wsrm__InvalidReplyPattern,	///< xs:QName value=""http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":InvalidReplyPattern"
	wsrm__FeatureNotSupported,	///< xs:QName value=""http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":FeatureNotSupported"
	wsrm__PermanentProcessingFailure,	///< xs:QName value=""http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":PermanentProcessingFailure"
	wsrm__MessageProcessingFailure,	///< xs:QName value=""http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":MessageProcessingFailure"
	wsrm__GroupAborted,	///< xs:QName value=""http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":GroupAborted"
};

/// "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":BareURIType is a simpleType restriction of xs:anyURI.
typedef char* /*URI*/ wsrm__BareURIType;

/// "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":HeaderBaseType is a complexType.
struct wsrm__HeaderBaseType
{
/// TODO: <any namespace="##other" minOccurs="0" maxOccurs="unbounded">
/// TODO: Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element).
/// <anyAttribute namespace="##other">
/// TODO: Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
///       Use wsdl2h option -d for xsd__anyAttribute DOM (soap_dom_attribute).
};

/// "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":EmptyType is a complexType.
struct wsrm__EmptyType
{
};

/// "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":ExtensibleType is a complexType.
struct wsrm__ExtensibleType
{
/// TODO: <any namespace="##other" minOccurs="0" maxOccurs="unbounded">
/// TODO: Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element).
/// <anyAttribute namespace="##other">
/// TODO: Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
///       Use wsdl2h option -d for xsd__anyAttribute DOM (soap_dom_attribute).
};

/// "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":ReplyPatternType is a complexType.
struct wsrm__ReplyPatternType
{
/// Element Value of type "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":ReplyPatternTypeValues.
    enum wsrm__ReplyPatternTypeValues    Value                          1;	///< Required element.
/// Element ReplyTo of type "http://docs.oasis-open.org/wsrm/2004/06/reference-1.1.xsd":ServiceRefType.
    ref__ServiceRefType*                 ReplyTo                        0;	///< Optional element.
};

/// "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":RequestType is a complexType with complexContent extension of "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":HeaderBaseType.
struct wsrm__RequestType
{
/// INHERITED FROM wsrm__HeaderBaseType:
/// TODO: <any namespace="##other" minOccurs="0" maxOccurs="unbounded">
/// TODO: Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element).
/// <anyAttribute namespace="##other">
/// TODO: Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
///       Use wsdl2h option -d for xsd__anyAttribute DOM (soap_dom_attribute).
//  END OF INHERITED
/// Element MessageId of type "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":MessageIdType.
    struct wsrm__MessageIdType*          MessageId                      1;	///< Required element.
/// Element ExpiryTime of type xs:dateTime.
    time_t                               ExpiryTime                     1;	///< Required element.
/// Element ReplyPattern of type "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":ReplyPatternType.
    struct wsrm__ReplyPatternType*       ReplyPattern                   1;	///< Required element.
/// Element AckRequested of type "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":EmptyType.
    struct wsrm__EmptyType*              AckRequested                   0;	///< Optional element.
/// Element DuplicateElimination of type "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":EmptyType.
    struct wsrm__EmptyType*              DuplicateElimination           0;	///< Optional element.
/// Element MessageOrder of type "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":EmptyType.
    struct wsrm__EmptyType*              MessageOrder                   0;	///< Optional element.
};

/// "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":MessageIdType is a complexType with complexContent extension of "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":EmptyType.
struct wsrm__MessageIdType
{
/// INHERITED FROM wsrm__EmptyType:
//  END OF INHERITED
/// Element SequenceNum of type "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":SequenceNumType.
    struct wsrm__SequenceNumType*        SequenceNum                    0;	///< Optional element.
/// Attribute groupId of type "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":MIDType.
   @wsrm__MIDType                        groupId                        1;	///< Required attribute.
};

/// "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":SequenceNumType is a complexType with complexContent extension of "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":EmptyType.
struct wsrm__SequenceNumType
{
/// INHERITED FROM wsrm__EmptyType:
//  END OF INHERITED
/// Attribute number of type xs:unsignedLong.
   @ULONG64                              number                         1;	///< Required attribute.
/// Attribute last of type xs:boolean.
   @enum xsd__boolean_                   last                           0 = false_;	///< Default value="false".
/// Attribute groupExpiryTime of type xs:dateTime.
   @time_t*                              groupExpiryTime                0;	///< Optional attribute.
/// Attribute groupMaxIdleDuration of type xs:duration.
   @char* /*duration*/                   groupMaxIdleDuration           0;	///< Optional attribute.
};

/// "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":ResponseType is a complexType with complexContent extension of "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":HeaderBaseType.
struct wsrm__ResponseType
{
/// INHERITED FROM wsrm__HeaderBaseType:
/// TODO: <any namespace="##other" minOccurs="0" maxOccurs="unbounded">
/// TODO: Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element).
/// <anyAttribute namespace="##other">
/// TODO: Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
///       Use wsdl2h option -d for xsd__anyAttribute DOM (soap_dom_attribute).
//  END OF INHERITED
/// CHOICE OF ELEMENTS FOR choice maxOccurs="unbounded"
    int                                  __size_ResponseType           ;
    struct __wsrm__union_ResponseType
    {
    int                                  __union_ResponseType          ;	///< Union _wsrm__union_ResponseType selector: set to SOAP_UNION__wsrm__union_ResponseType_<fieldname>
/// Union for choice in type wsrm__ResponseType
    union _wsrm__union_ResponseType
    {
/// Element NonSequenceReply of type "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":NonSequenceReplyType.
    struct wsrm__NonSequenceReplyType*   NonSequenceReply               1;	///< Required element.
/// Element SequenceReplies of type "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":SequenceRepliesType.
    struct wsrm__SequenceRepliesType*    SequenceReplies                1;	///< Required element.
    }                                    union_ResponseType            ;
    }                                   *__union_ResponseType          ;
//  END OF CHOICE
};

/// "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":NonSequenceReplyType is a complexType with complexContent extension of "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":ExtensibleType.
struct wsrm__NonSequenceReplyType
{
/// INHERITED FROM wsrm__ExtensibleType:
/// TODO: <any namespace="##other" minOccurs="0" maxOccurs="unbounded">
/// TODO: Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element).
/// <anyAttribute namespace="##other">
/// TODO: Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
///       Use wsdl2h option -d for xsd__anyAttribute DOM (soap_dom_attribute).
//  END OF INHERITED
/// Attribute groupId of type "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":MIDType.
   @wsrm__MIDType                        groupId                        1;	///< Required attribute.
/// Attribute fault of type "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":FaultCodeEnum.
   @enum wsrm__FaultCodeEnum*            fault                          0;	///< Optional attribute.
};

/// "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":SequenceRepliesType is a complexType with complexContent extension of "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":ExtensibleType.
struct wsrm__SequenceRepliesType
{
/// INHERITED FROM wsrm__ExtensibleType:
/// TODO: <any namespace="##other" minOccurs="0" maxOccurs="unbounded">
/// TODO: Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element).
/// <anyAttribute namespace="##other">
/// TODO: Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
///       Use wsdl2h option -d for xsd__anyAttribute DOM (soap_dom_attribute).
//  END OF INHERITED
/// Size of array of struct wsrm__ReplyRangeType* is 1..unbounded
    int                                  __sizeReplyRange              ;
/// Array of length 1..unbounded
    struct wsrm__ReplyRangeType*         ReplyRange                     1;
/// Attribute groupId of type "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":MIDType.
   @wsrm__MIDType                        groupId                        1;	///< Required attribute.
};

/// "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":ReplyRangeType is a complexType with complexContent extension of "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":ExtensibleType.
struct wsrm__ReplyRangeType
{
/// INHERITED FROM wsrm__ExtensibleType:
/// TODO: <any namespace="##other" minOccurs="0" maxOccurs="unbounded">
/// TODO: Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element).
/// <anyAttribute namespace="##other">
/// TODO: Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
///       Use wsdl2h option -d for xsd__anyAttribute DOM (soap_dom_attribute).
//  END OF INHERITED
/// Attribute from of type xs:unsignedLong.
   @ULONG64                              from                           1;	///< Required attribute.
/// Attribute to of type xs:unsignedLong.
   @ULONG64                              to                             1;	///< Required attribute.
/// Attribute fault of type "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":FaultCodeEnum.
   @enum wsrm__FaultCodeEnum*            fault                          0;	///< Optional attribute.
};

/// "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":RefToMessageIdsType is a complexType with complexContent extension of "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":EmptyType.
struct wsrm__RefToMessageIdsType
{
/// INHERITED FROM wsrm__EmptyType:
//  END OF INHERITED
/// Size of array of struct wsrm__SequenceNumRangeType* is 0..unbounded
    int                                  __sizeSequenceNumRange        ;
/// Array of length 0..unbounded
    struct wsrm__SequenceNumRangeType*   SequenceNumRange               0;
/// Attribute groupId of type "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":MIDType.
   @wsrm__MIDType                        groupId                        1;	///< Required attribute.
};

/// "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":SequenceNumRangeType is a complexType with complexContent extension of "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":EmptyType.
struct wsrm__SequenceNumRangeType
{
/// INHERITED FROM wsrm__EmptyType:
//  END OF INHERITED
/// Attribute from of type xs:unsignedLong.
   @ULONG64                              from                           1;	///< Required attribute.
/// Attribute to of type xs:unsignedLong.
   @ULONG64                              to                             1;	///< Required attribute.
};

/// "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":PollRequestType is a complexType with complexContent extension of "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":HeaderBaseType.
struct wsrm__PollRequestType
{
/// INHERITED FROM wsrm__HeaderBaseType:
/// TODO: <any namespace="##other" minOccurs="0" maxOccurs="unbounded">
/// TODO: Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element).
/// <anyAttribute namespace="##other">
/// TODO: Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
///       Use wsdl2h option -d for xsd__anyAttribute DOM (soap_dom_attribute).
//  END OF INHERITED
/// Size of array of struct wsrm__RefToMessageIdsType* is 1..unbounded
    int                                  __sizeRefToMessageIds         ;
/// Array of length 1..unbounded
    struct wsrm__RefToMessageIdsType*    RefToMessageIds                1;
/// Element ReplyTo of type "http://docs.oasis-open.org/wsrm/2004/06/reference-1.1.xsd":ServiceRefType.
    ref__ServiceRefType*                 ReplyTo                        0;	///< Optional element.
};

/// Element "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":Request of type "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":RequestType.
typedef struct wsrm__RequestType _wsrm__Request;

/// Element "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":Response of type "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":ResponseType.
typedef struct wsrm__ResponseType _wsrm__Response;

/// Element "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":PollRequest of type "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":PollRequestType.
typedef struct wsrm__PollRequestType _wsrm__PollRequest;

/// Element "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":BareURI of type "http://docs.oasis-open.org/wsrm/2004/06/ws-reliability-1.1.xsd":BareURIType.
typedef wsrm__BareURIType _wsrm__BareURI;

/* End of wsrm1.h */
