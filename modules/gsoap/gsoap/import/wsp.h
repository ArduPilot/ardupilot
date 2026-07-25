/*
	wsp.h

	Generated with:
	wsdl2h -c -x -o wsp.h -t WS/WS-typemap.dat WS/WS-Policy.xsd

	- Removed //gsoapopt
	- Removed #import "wsu.h" since only wsu__Id is needed
	- Removed wsu__Id and replaced with char*, since this type is defined in wsse.h
	- Removed enum xsd__boolean_
	- Added //gsoap wsu schema import: http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd
	- Added //gsoap wsp schema import: http://schemas.xmlsoap.org/ws/2004/09/policy
	- Added #import "wsse.h"
	- Modified wsp__union_1/_2/_3 to include wsse elements
	- Added wsse elements to wsp__union_1 and _2

*/

/******************************************************************************\
 *                                                                            *
 * http://schemas.xmlsoap.org/ws/2004/09/policy                               *
 *                                                                            *
\******************************************************************************/

/******************************************************************************\
 *                                                                            *
 * Import                                                                     *
 *                                                                            *
\******************************************************************************/

#import "wsse.h"

/******************************************************************************\
 *                                                                            *
 * Schema Namespaces                                                          *
 *                                                                            *
\******************************************************************************/

//gsoap wsu schema import: http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd

//gsoap wsp   schema import:	http://schemas.xmlsoap.org/ws/2004/09/policy
//gsoap wsp   schema elementForm:	qualified
//gsoap wsp   schema attributeForm:	unqualified

/******************************************************************************\
 *                                                                            *
 * Schema Types                                                               *
 *                                                                            *
\******************************************************************************/


/// Built-in type "xs:base64Binary"
struct xsd__base64Binary_ { unsigned char *__ptr; int __size; };


/// "http://schemas.xmlsoap.org/ws/2004/09/policy":PolicyURIs is a simpleType containing a whitespace separated list of xs:anyURI.
typedef char* _wsp__PolicyURIs;

/// Modified wsp__union_1 to add wsse elements
union wsp__union_1
{
/// Element reference "http://schemas.xmlsoap.org/ws/2004/09/policy":Policy
    struct _wsp__Policy*                 Policy                         1;	///< Required element
/// Element reference "http://schemas.xmlsoap.org/ws/2004/09/policy":All
    struct wsp__OperatorContentType*     All                            1;	///< Required element
/// Element reference "http://schemas.xmlsoap.org/ws/2004/09/policy":ExactlyOne
    struct wsp__OperatorContentType*     ExactlyOne                     1;	///< Required element
/// Element reference "http://schemas.xmlsoap.org/ws/2004/09/policy":PolicyReference
    struct _wsp__PolicyReference*        PolicyReference                1;	///< Required element

/// Added wsse:Confidentiality element
    struct wsse__Confidentiality*        wsse__Confidentiality          1;
/// Added wsse:SecurityHeader element
    struct wsse__SecurityHeader*         wsse__SecurityHeader           1;
/// Added wsse:SecurityToken element
    struct wsse__SecurityToken*          wsse__SecurityToken            1;
};

/// Modified __wsp__union_1 by removing duplicate __wsp__union_3
struct __wsp__union_1
{
    int                                  __union_1                      0;	///< Union wsp__union_1 selector: set to SOAP_UNION_wsp__union_1_<fieldname> or 0
    union wsp__union_1                   union_1                       ;
};

/// "http://schemas.xmlsoap.org/ws/2004/09/policy":OperatorContentType is a complexType.
struct wsp__OperatorContentType
{
    int                                  __sizeunion_1                  0;
    struct __wsp__union_1               *__union_1                     ;
};

/// "http://schemas.xmlsoap.org/ws/2004/09/policy":PolicyReference is a complexType.
struct _wsp__PolicyReference
{
/// Attribute URI of type xs:anyURI
   @char*                                URI                            0;	///< Optional attribute
/// Attribute Digest of type xs:base64Binary
   @struct xsd__base64Binary_*           Digest                         0;	///< Optional attribute
/// Attribute DigestAlgorithm of type xs:anyURI
   @char*                                DigestAlgorithm                0;	///< Optional attribute
/// TODO: <anyAttribute namespace="##any">
///       Schema extensibility is user-definable.
///       Consult the protocol documentation to change and/or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
};

/// "http://schemas.xmlsoap.org/ws/2004/09/policy":UsingPolicy is a complexType.
struct _wsp__UsingPolicy
{
/// TODO: <anyAttribute namespace="##any">
///       Schema extensibility is user-definable.
///       Consult the protocol documentation to change and/or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
};

/// Modified wsp__union_2
union wsp__union_2
{
/// Element reference "http://schemas.xmlsoap.org/ws/2004/09/policy":Policy
    struct _wsp__Policy*                 Policy                         1;	///< Required element
/// Element reference "http://schemas.xmlsoap.org/ws/2004/09/policy":PolicyReference
    struct _wsp__PolicyReference*        PolicyReference                1;	///< Required element
};

// Modified __wsp__union_2
struct __wsp__union_2
{
    int                                  __union_2                     ;	///< Union wsp__union_2 selector: set to SOAP_UNION_wsp__union_2_<fieldname>
    union wsp__union_2                   union_2                       ;
};

/// "http://schemas.xmlsoap.org/ws/2004/09/policy":PolicyAttachment is a complexType.
struct _wsp__PolicyAttachment
{
/// Element reference "http://schemas.xmlsoap.org/ws/2004/09/policy":AppliesTo
    struct _wsp__AppliesTo*              AppliesTo                      1;	///< Required element
/// CHOICE OF ELEMENTS <choice maxOccurs="unbounded">
    int                                  __sizeunion_2                 ;
    struct __wsp__union_2               *__union_2                     ;
/// TODO: <any namespace="##other" minOccurs="0" maxOccurs="unbounded">
///       Schema extensibility is user-definable.
///       Consult the protocol documentation to change and/or insert declarations.
///       Use wsdl2h option -x to remove this element.
/// TODO: <anyAttribute namespace="##any">
///       Schema extensibility is user-definable.
///       Consult the protocol documentation to change and/or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
};

/// "http://schemas.xmlsoap.org/ws/2004/09/policy":AppliesTo is a complexType.
struct _wsp__AppliesTo
{
/// TODO: <any namespace="##any" maxOccurs="unbounded">
///       Schema extensibility is user-definable.
///       Consult the protocol documentation to change and/or insert declarations.
///       Use wsdl2h option -x to remove this element.
/// TODO: <anyAttribute namespace="##any">
///       Schema extensibility is user-definable.
///       Consult the protocol documentation to change and/or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
};

/// "http://schemas.xmlsoap.org/ws/2004/09/policy":Policy is a complexType with complexContent extension of "http://schemas.xmlsoap.org/ws/2004/09/policy":OperatorContentType.
struct _wsp__Policy
{
    int                                  __sizeunion_1                  0;
    struct __wsp__union_1               *__union_1                     ;
/// Attribute TargetNamespace of type xs:anyURI
   @char*                                TargetNamespace                0;	///< Optional attribute
/// Attribute reference "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd":Id
   @char*                                wsu__Id_                       0;	///< Optional attribute
/// TODO: <anyAttribute namespace="##any">
///       Schema extensibility is user-definable.
///       Consult the protocol documentation to change and/or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
};

/* End of wsp.h */
