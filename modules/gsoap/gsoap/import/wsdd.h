/*
	wsdd.h WS-Discovery 1.1 2009 with WS-Addressing 2005/08

	Generated with:
	wsdl2h -cgyex -o wsdd.h -t WS/WS-typemap.dat WS/discovery.xsd

        Requires:
        - plugin/wsddapi.h and plugin/wsddapi.c
        - plugin/wsaapi.h and plugin/wsaapi.c
        - custom/duration.c

        This file imports:
        - wsa5.h
        - custom/duration.h
        - wsdx.h

        - Removed //gsoapopt
        - Changed //gsoap wssd schema namespace directive to import directive
        - Added #import "wsdx.h" at the end of these definitions
	- Added #define SOAP_WSDD_2009
*/

#define SOAP_WSDD_2009

/******************************************************************************\
 *                                                                            *
 * Definitions                                                                *
 *   http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01                    *
 *                                                                            *
\******************************************************************************/


/******************************************************************************\
 *                                                                            *
 * Import                                                                     *
 *                                                                            *
\******************************************************************************/

#import "wsa5.h"	// wsa5 = <http://www.w3.org/2005/08/addressing>

/******************************************************************************\
 *                                                                            *
 * Schema Namespaces                                                          *
 *                                                                            *
\******************************************************************************/

#define SOAP_NAMESPACE_OF_wsdd	"http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01"
//gsoap wsdd  schema import:	http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01
//gsoap wsdd  schema elementForm:	qualified
//gsoap wsdd  schema attributeForm:	unqualified

/******************************************************************************\
 *                                                                            *
 * Built-in Schema Types and Top-Level Elements and Attributes                *
 *                                                                            *
\******************************************************************************/

// Imported element ""http://www.w3.org/2005/08/addressing":EndpointReference" declared as _wsa5__EndpointReference.

/// @brief Typedef synonym for struct wsdd__HelloType.
typedef struct wsdd__HelloType wsdd__HelloType;
/// @brief Typedef synonym for struct wsdd__ByeType.
typedef struct wsdd__ByeType wsdd__ByeType;
/// @brief Typedef synonym for struct wsdd__ProbeType.
typedef struct wsdd__ProbeType wsdd__ProbeType;
/// @brief Typedef synonym for struct wsdd__ProbeMatchesType.
typedef struct wsdd__ProbeMatchesType wsdd__ProbeMatchesType;
/// @brief Typedef synonym for struct wsdd__ProbeMatchType.
typedef struct wsdd__ProbeMatchType wsdd__ProbeMatchType;
/// @brief Typedef synonym for struct wsdd__ResolveType.
typedef struct wsdd__ResolveType wsdd__ResolveType;
/// @brief Typedef synonym for struct wsdd__ResolveMatchesType.
typedef struct wsdd__ResolveMatchesType wsdd__ResolveMatchesType;
/// @brief Typedef synonym for struct wsdd__ResolveMatchType.
typedef struct wsdd__ResolveMatchType wsdd__ResolveMatchType;
/// @brief Typedef synonym for struct wsdd__ScopesType.
typedef struct wsdd__ScopesType wsdd__ScopesType;
/// @brief Typedef synonym for struct wsdd__SecurityType.
typedef struct wsdd__SecurityType wsdd__SecurityType;
/// @brief Typedef synonym for struct wsdd__SigType.
typedef struct wsdd__SigType wsdd__SigType;
/// @brief Typedef synonym for struct wsdd__AppSequenceType.
typedef struct wsdd__AppSequenceType wsdd__AppSequenceType;

/******************************************************************************\
 *                                                                            *
 * Schema Types and Top-Level Elements and Attributes                         *
 *   http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01                    *
 *                                                                            *
\******************************************************************************/

/// @brief "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":QNameListType is a simpleType containing a whitespace separated list of xs:QName.
///
typedef _QName wsdd__QNameListType;

/// @brief "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":UriListType is a simpleType containing a whitespace separated list of xs:anyURI.
///
typedef char* wsdd__UriListType;

/// @brief Union of values from member types "tns:FaultCodeType xs:QName".
typedef char* wsdd__FaultCodeOpenType;

/// @brief "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":FaultCodeType is a simpleType restriction of XSD type xs:QName.
///
enum wsdd__FaultCodeType
{
	wsdd__MatchingRuleNotSupported,	///< xs:QName value=""http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":MatchingRuleNotSupported"
};

/// @brief Typedef synonym for enum wsdd__FaultCodeType.
typedef enum wsdd__FaultCodeType wsdd__FaultCodeType;

/******************************************************************************\
 *                                                                            *
 * Schema Complex Types and Top-Level Elements                                *
 *   http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01                    *
 *                                                                            *
\******************************************************************************/

/// @brief "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":HelloType is a complexType.
///
/// struct wsdd__HelloType operations:
/// - wsdd__HelloType* soap_new_wsdd__HelloType(struct soap*, int num) allocate and default initialize one or more values (array)
/// - soap_default_wsdd__HelloType(struct soap*, wsdd__HelloType*) default initialize members
/// - int soap_read_wsdd__HelloType(struct soap*, wsdd__HelloType*) deserialize from a source
/// - int soap_write_wsdd__HelloType(struct soap*, wsdd__HelloType*) serialize to a sink
/// - wsdd__HelloType* soap_dup_wsdd__HelloType(struct soap*, wsdd__HelloType* dst, wsdd__HelloType *src) returns deep copy of wsdd__HelloType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_wsdd__HelloType(wsdd__HelloType*) deep deletes wsdd__HelloType data members, use only on dst after soap_dup_wsdd__HelloType(NULL, wsdd__HelloType *dst, wsdd__HelloType *src) (use soapcpp2 -Ed)
struct wsdd__HelloType
{
/// Imported element reference "http://www.w3.org/2005/08/addressing":EndpointReference.
    _wsa5__EndpointReference             wsa5__EndpointReference        1;	///< Required element.
/// Element reference "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01:""http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":Types.
    wsdd__QNameListType                  Types                          0;	///< Optional element.
/// Element reference "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01:""http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":Scopes.
    struct wsdd__ScopesType*             Scopes                         0;	///< Optional element.
/// Element reference "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01:""http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":XAddrs.
    wsdd__UriListType                    XAddrs                         0;	///< Optional element.
/// Element reference "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01:""http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":MetadataVersion.
    unsigned int                         MetadataVersion                1;	///< Required element.
/// @todo <any namespace="##other" minOccurs="0" maxOccurs="unbounded">
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element):
///       wsdl2h maps xsd:any to xsd__anyType, use typemap.dat to remap.
/// @todo <anyAttribute namespace="##other">.
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
///       Use wsdl2h option -d for xsd__anyAttribute DOM (soap_dom_attribute).
};

/// @brief "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":ByeType is a complexType.
///
/// struct wsdd__ByeType operations:
/// - wsdd__ByeType* soap_new_wsdd__ByeType(struct soap*, int num) allocate and default initialize one or more values (array)
/// - soap_default_wsdd__ByeType(struct soap*, wsdd__ByeType*) default initialize members
/// - int soap_read_wsdd__ByeType(struct soap*, wsdd__ByeType*) deserialize from a source
/// - int soap_write_wsdd__ByeType(struct soap*, wsdd__ByeType*) serialize to a sink
/// - wsdd__ByeType* soap_dup_wsdd__ByeType(struct soap*, wsdd__ByeType* dst, wsdd__ByeType *src) returns deep copy of wsdd__ByeType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_wsdd__ByeType(wsdd__ByeType*) deep deletes wsdd__ByeType data members, use only on dst after soap_dup_wsdd__ByeType(NULL, wsdd__ByeType *dst, wsdd__ByeType *src) (use soapcpp2 -Ed)
struct wsdd__ByeType
{
/// Imported element reference "http://www.w3.org/2005/08/addressing":EndpointReference.
    _wsa5__EndpointReference             wsa5__EndpointReference        1;	///< Required element.
/// Element reference "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01:""http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":Types.
    wsdd__QNameListType                  Types                          0;	///< Optional element.
/// Element reference "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01:""http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":Scopes.
    struct wsdd__ScopesType*             Scopes                         0;	///< Optional element.
/// Element reference "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01:""http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":XAddrs.
    wsdd__UriListType                    XAddrs                         0;	///< Optional element.
/// Element reference "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01:""http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":MetadataVersion.
    unsigned int*                        MetadataVersion                0;	///< Optional element.
/// @todo <any namespace="##other" minOccurs="0" maxOccurs="unbounded">
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element):
///       wsdl2h maps xsd:any to xsd__anyType, use typemap.dat to remap.
/// @todo <anyAttribute namespace="##other">.
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
///       Use wsdl2h option -d for xsd__anyAttribute DOM (soap_dom_attribute).
};

/// @brief "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":ProbeType is a complexType.
///
/// struct wsdd__ProbeType operations:
/// - wsdd__ProbeType* soap_new_wsdd__ProbeType(struct soap*, int num) allocate and default initialize one or more values (array)
/// - soap_default_wsdd__ProbeType(struct soap*, wsdd__ProbeType*) default initialize members
/// - int soap_read_wsdd__ProbeType(struct soap*, wsdd__ProbeType*) deserialize from a source
/// - int soap_write_wsdd__ProbeType(struct soap*, wsdd__ProbeType*) serialize to a sink
/// - wsdd__ProbeType* soap_dup_wsdd__ProbeType(struct soap*, wsdd__ProbeType* dst, wsdd__ProbeType *src) returns deep copy of wsdd__ProbeType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_wsdd__ProbeType(wsdd__ProbeType*) deep deletes wsdd__ProbeType data members, use only on dst after soap_dup_wsdd__ProbeType(NULL, wsdd__ProbeType *dst, wsdd__ProbeType *src) (use soapcpp2 -Ed)
struct wsdd__ProbeType
{
/// Element reference "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01:""http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":Types.
    wsdd__QNameListType                  Types                          0;	///< Optional element.
/// Element reference "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01:""http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":Scopes.
    struct wsdd__ScopesType*             Scopes                         0;	///< Optional element.
/// @todo <any namespace="##other" minOccurs="0" maxOccurs="unbounded">
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element):
///       wsdl2h maps xsd:any to xsd__anyType, use typemap.dat to remap.
/// @todo <anyAttribute namespace="##other">.
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
///       Use wsdl2h option -d for xsd__anyAttribute DOM (soap_dom_attribute).
};

/// @brief "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":ProbeMatchesType is a complexType.
///
/// struct wsdd__ProbeMatchesType operations:
/// - wsdd__ProbeMatchesType* soap_new_wsdd__ProbeMatchesType(struct soap*, int num) allocate and default initialize one or more values (array)
/// - soap_default_wsdd__ProbeMatchesType(struct soap*, wsdd__ProbeMatchesType*) default initialize members
/// - int soap_read_wsdd__ProbeMatchesType(struct soap*, wsdd__ProbeMatchesType*) deserialize from a source
/// - int soap_write_wsdd__ProbeMatchesType(struct soap*, wsdd__ProbeMatchesType*) serialize to a sink
/// - wsdd__ProbeMatchesType* soap_dup_wsdd__ProbeMatchesType(struct soap*, wsdd__ProbeMatchesType* dst, wsdd__ProbeMatchesType *src) returns deep copy of wsdd__ProbeMatchesType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_wsdd__ProbeMatchesType(wsdd__ProbeMatchesType*) deep deletes wsdd__ProbeMatchesType data members, use only on dst after soap_dup_wsdd__ProbeMatchesType(NULL, wsdd__ProbeMatchesType *dst, wsdd__ProbeMatchesType *src) (use soapcpp2 -Ed)
struct wsdd__ProbeMatchesType
{
/// Size of array of struct wsdd__ProbeMatchType* is 0..unbounded.
   $int                                  __sizeProbeMatch               0;
/// Array struct wsdd__ProbeMatchType* of size 0..unbounded.
    struct wsdd__ProbeMatchType*         ProbeMatch                     0;
/// @todo <any namespace="##other" minOccurs="0" maxOccurs="unbounded">
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element):
///       wsdl2h maps xsd:any to xsd__anyType, use typemap.dat to remap.
/// @todo <anyAttribute namespace="##other">.
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
///       Use wsdl2h option -d for xsd__anyAttribute DOM (soap_dom_attribute).
};

/// @brief "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":ProbeMatchType is a complexType.
///
/// struct wsdd__ProbeMatchType operations:
/// - wsdd__ProbeMatchType* soap_new_wsdd__ProbeMatchType(struct soap*, int num) allocate and default initialize one or more values (array)
/// - soap_default_wsdd__ProbeMatchType(struct soap*, wsdd__ProbeMatchType*) default initialize members
/// - int soap_read_wsdd__ProbeMatchType(struct soap*, wsdd__ProbeMatchType*) deserialize from a source
/// - int soap_write_wsdd__ProbeMatchType(struct soap*, wsdd__ProbeMatchType*) serialize to a sink
/// - wsdd__ProbeMatchType* soap_dup_wsdd__ProbeMatchType(struct soap*, wsdd__ProbeMatchType* dst, wsdd__ProbeMatchType *src) returns deep copy of wsdd__ProbeMatchType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_wsdd__ProbeMatchType(wsdd__ProbeMatchType*) deep deletes wsdd__ProbeMatchType data members, use only on dst after soap_dup_wsdd__ProbeMatchType(NULL, wsdd__ProbeMatchType *dst, wsdd__ProbeMatchType *src) (use soapcpp2 -Ed)
struct wsdd__ProbeMatchType
{
/// Imported element reference "http://www.w3.org/2005/08/addressing":EndpointReference.
    _wsa5__EndpointReference             wsa5__EndpointReference        1;	///< Required element.
/// Element reference "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01:""http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":Types.
    wsdd__QNameListType                  Types                          0;	///< Optional element.
/// Element reference "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01:""http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":Scopes.
    struct wsdd__ScopesType*             Scopes                         0;	///< Optional element.
/// Element reference "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01:""http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":XAddrs.
    wsdd__UriListType                    XAddrs                         0;	///< Optional element.
/// Element reference "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01:""http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":MetadataVersion.
    unsigned int                         MetadataVersion                1;	///< Required element.
/// @todo <any namespace="##other" minOccurs="0" maxOccurs="unbounded">
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element):
///       wsdl2h maps xsd:any to xsd__anyType, use typemap.dat to remap.
/// @todo <anyAttribute namespace="##other">.
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
///       Use wsdl2h option -d for xsd__anyAttribute DOM (soap_dom_attribute).
};

/// @brief "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":ResolveType is a complexType.
///
/// struct wsdd__ResolveType operations:
/// - wsdd__ResolveType* soap_new_wsdd__ResolveType(struct soap*, int num) allocate and default initialize one or more values (array)
/// - soap_default_wsdd__ResolveType(struct soap*, wsdd__ResolveType*) default initialize members
/// - int soap_read_wsdd__ResolveType(struct soap*, wsdd__ResolveType*) deserialize from a source
/// - int soap_write_wsdd__ResolveType(struct soap*, wsdd__ResolveType*) serialize to a sink
/// - wsdd__ResolveType* soap_dup_wsdd__ResolveType(struct soap*, wsdd__ResolveType* dst, wsdd__ResolveType *src) returns deep copy of wsdd__ResolveType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_wsdd__ResolveType(wsdd__ResolveType*) deep deletes wsdd__ResolveType data members, use only on dst after soap_dup_wsdd__ResolveType(NULL, wsdd__ResolveType *dst, wsdd__ResolveType *src) (use soapcpp2 -Ed)
struct wsdd__ResolveType
{
/// Imported element reference "http://www.w3.org/2005/08/addressing":EndpointReference.
    _wsa5__EndpointReference             wsa5__EndpointReference        1;	///< Required element.
/// @todo <any namespace="##other" minOccurs="0" maxOccurs="unbounded">
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element):
///       wsdl2h maps xsd:any to xsd__anyType, use typemap.dat to remap.
/// @todo <anyAttribute namespace="##other">.
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
///       Use wsdl2h option -d for xsd__anyAttribute DOM (soap_dom_attribute).
};

/// @brief "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":ResolveMatchesType is a complexType.
///
/// struct wsdd__ResolveMatchesType operations:
/// - wsdd__ResolveMatchesType* soap_new_wsdd__ResolveMatchesType(struct soap*, int num) allocate and default initialize one or more values (array)
/// - soap_default_wsdd__ResolveMatchesType(struct soap*, wsdd__ResolveMatchesType*) default initialize members
/// - int soap_read_wsdd__ResolveMatchesType(struct soap*, wsdd__ResolveMatchesType*) deserialize from a source
/// - int soap_write_wsdd__ResolveMatchesType(struct soap*, wsdd__ResolveMatchesType*) serialize to a sink
/// - wsdd__ResolveMatchesType* soap_dup_wsdd__ResolveMatchesType(struct soap*, wsdd__ResolveMatchesType* dst, wsdd__ResolveMatchesType *src) returns deep copy of wsdd__ResolveMatchesType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_wsdd__ResolveMatchesType(wsdd__ResolveMatchesType*) deep deletes wsdd__ResolveMatchesType data members, use only on dst after soap_dup_wsdd__ResolveMatchesType(NULL, wsdd__ResolveMatchesType *dst, wsdd__ResolveMatchesType *src) (use soapcpp2 -Ed)
struct wsdd__ResolveMatchesType
{
/// Element "ResolveMatch" of XSD type "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":ResolveMatchType.
    struct wsdd__ResolveMatchType*       ResolveMatch                   0;	///< Optional element.
/// @todo <any namespace="##other" minOccurs="0" maxOccurs="unbounded">
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element):
///       wsdl2h maps xsd:any to xsd__anyType, use typemap.dat to remap.
/// @todo <anyAttribute namespace="##other">.
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
///       Use wsdl2h option -d for xsd__anyAttribute DOM (soap_dom_attribute).
};

/// @brief "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":ResolveMatchType is a complexType.
///
/// struct wsdd__ResolveMatchType operations:
/// - wsdd__ResolveMatchType* soap_new_wsdd__ResolveMatchType(struct soap*, int num) allocate and default initialize one or more values (array)
/// - soap_default_wsdd__ResolveMatchType(struct soap*, wsdd__ResolveMatchType*) default initialize members
/// - int soap_read_wsdd__ResolveMatchType(struct soap*, wsdd__ResolveMatchType*) deserialize from a source
/// - int soap_write_wsdd__ResolveMatchType(struct soap*, wsdd__ResolveMatchType*) serialize to a sink
/// - wsdd__ResolveMatchType* soap_dup_wsdd__ResolveMatchType(struct soap*, wsdd__ResolveMatchType* dst, wsdd__ResolveMatchType *src) returns deep copy of wsdd__ResolveMatchType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_wsdd__ResolveMatchType(wsdd__ResolveMatchType*) deep deletes wsdd__ResolveMatchType data members, use only on dst after soap_dup_wsdd__ResolveMatchType(NULL, wsdd__ResolveMatchType *dst, wsdd__ResolveMatchType *src) (use soapcpp2 -Ed)
struct wsdd__ResolveMatchType
{
/// Imported element reference "http://www.w3.org/2005/08/addressing":EndpointReference.
    _wsa5__EndpointReference             wsa5__EndpointReference        1;	///< Required element.
/// Element reference "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01:""http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":Types.
    wsdd__QNameListType                  Types                          0;	///< Optional element.
/// Element reference "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01:""http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":Scopes.
    struct wsdd__ScopesType*             Scopes                         0;	///< Optional element.
/// Element reference "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01:""http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":XAddrs.
    wsdd__UriListType                    XAddrs                         0;	///< Optional element.
/// Element reference "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01:""http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":MetadataVersion.
    unsigned int                         MetadataVersion                1;	///< Required element.
/// @todo <any namespace="##other" minOccurs="0" maxOccurs="unbounded">
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element):
///       wsdl2h maps xsd:any to xsd__anyType, use typemap.dat to remap.
/// @todo <anyAttribute namespace="##other">.
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
///       Use wsdl2h option -d for xsd__anyAttribute DOM (soap_dom_attribute).
};

/// @brief "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":SecurityType is a complexType.
///
/// struct wsdd__SecurityType operations:
/// - wsdd__SecurityType* soap_new_wsdd__SecurityType(struct soap*, int num) allocate and default initialize one or more values (array)
/// - soap_default_wsdd__SecurityType(struct soap*, wsdd__SecurityType*) default initialize members
/// - int soap_read_wsdd__SecurityType(struct soap*, wsdd__SecurityType*) deserialize from a source
/// - int soap_write_wsdd__SecurityType(struct soap*, wsdd__SecurityType*) serialize to a sink
/// - wsdd__SecurityType* soap_dup_wsdd__SecurityType(struct soap*, wsdd__SecurityType* dst, wsdd__SecurityType *src) returns deep copy of wsdd__SecurityType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_wsdd__SecurityType(wsdd__SecurityType*) deep deletes wsdd__SecurityType data members, use only on dst after soap_dup_wsdd__SecurityType(NULL, wsdd__SecurityType *dst, wsdd__SecurityType *src) (use soapcpp2 -Ed)
struct wsdd__SecurityType
{
/// Element reference "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01:""http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":Sig.
    struct wsdd__SigType*                Sig                            0;	///< Optional element.
/// @todo <anyAttribute namespace="##other">.
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
///       Use wsdl2h option -d for xsd__anyAttribute DOM (soap_dom_attribute).
};

/// @brief "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":SigType is a complexType.
///
/// struct wsdd__SigType operations:
/// - wsdd__SigType* soap_new_wsdd__SigType(struct soap*, int num) allocate and default initialize one or more values (array)
/// - soap_default_wsdd__SigType(struct soap*, wsdd__SigType*) default initialize members
/// - int soap_read_wsdd__SigType(struct soap*, wsdd__SigType*) deserialize from a source
/// - int soap_write_wsdd__SigType(struct soap*, wsdd__SigType*) serialize to a sink
/// - wsdd__SigType* soap_dup_wsdd__SigType(struct soap*, wsdd__SigType* dst, wsdd__SigType *src) returns deep copy of wsdd__SigType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_wsdd__SigType(wsdd__SigType*) deep deletes wsdd__SigType data members, use only on dst after soap_dup_wsdd__SigType(NULL, wsdd__SigType *dst, wsdd__SigType *src) (use soapcpp2 -Ed)
struct wsdd__SigType
{
/// @todo <any namespace="##other" minOccurs="0" maxOccurs="unbounded">
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element):
///       wsdl2h maps xsd:any to xsd__anyType, use typemap.dat to remap.
/// Attribute "Scheme" of XSD type xs:anyURI.
   @char*                                Scheme                         1;	///< Required attribute.
/// Attribute "KeyId" of XSD type xs:base64Binary.
   @char*                                KeyId                          0;	///< Optional attribute.
/// Attribute "Refs" of XSD type xs:IDREFS.
   @char*                                Refs                           1;	///< Required attribute.
/// Attribute "Sig" of XSD type xs:base64Binary.
   @char*                                Sig                            1;	///< Required attribute.
/// @todo <anyAttribute namespace="##other">.
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
///       Use wsdl2h option -d for xsd__anyAttribute DOM (soap_dom_attribute).
};

/// @brief "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":ScopesType is a complexType with simpleContent.
///
/// struct wsdd__ScopesType operations:
/// - wsdd__ScopesType* soap_new_wsdd__ScopesType(struct soap*, int num) allocate and default initialize one or more values (array)
/// - soap_default_wsdd__ScopesType(struct soap*, wsdd__ScopesType*) default initialize members
/// - int soap_read_wsdd__ScopesType(struct soap*, wsdd__ScopesType*) deserialize from a source
/// - int soap_write_wsdd__ScopesType(struct soap*, wsdd__ScopesType*) serialize to a sink
/// - wsdd__ScopesType* soap_dup_wsdd__ScopesType(struct soap*, wsdd__ScopesType* dst, wsdd__ScopesType *src) returns deep copy of wsdd__ScopesType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_wsdd__ScopesType(wsdd__ScopesType*) deep deletes wsdd__ScopesType data members, use only on dst after soap_dup_wsdd__ScopesType(NULL, wsdd__ScopesType *dst, wsdd__ScopesType *src) (use soapcpp2 -Ed)
struct wsdd__ScopesType
{
/// __item wraps ""http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":UriListType" simpleContent.
    wsdd__UriListType                    __item                        ;
/// Attribute "MatchBy" of XSD type xs:anyURI.
   @char*                                MatchBy                        0;	///< Optional attribute.
/// @todo <anyAttribute namespace="##other">.
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
///       Use wsdl2h option -d for xsd__anyAttribute DOM (soap_dom_attribute).
};

/// @brief "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":AppSequenceType is a complexType with complexContent restriction of XSD type xs:anyType.
///
/// struct wsdd__AppSequenceType operations:
/// - wsdd__AppSequenceType* soap_new_wsdd__AppSequenceType(struct soap*, int num) allocate and default initialize one or more values (array)
/// - soap_default_wsdd__AppSequenceType(struct soap*, wsdd__AppSequenceType*) default initialize members
/// - int soap_read_wsdd__AppSequenceType(struct soap*, wsdd__AppSequenceType*) deserialize from a source
/// - int soap_write_wsdd__AppSequenceType(struct soap*, wsdd__AppSequenceType*) serialize to a sink
/// - wsdd__AppSequenceType* soap_dup_wsdd__AppSequenceType(struct soap*, wsdd__AppSequenceType* dst, wsdd__AppSequenceType *src) returns deep copy of wsdd__AppSequenceType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_wsdd__AppSequenceType(wsdd__AppSequenceType*) deep deletes wsdd__AppSequenceType data members, use only on dst after soap_dup_wsdd__AppSequenceType(NULL, wsdd__AppSequenceType *dst, wsdd__AppSequenceType *src) (use soapcpp2 -Ed)
struct wsdd__AppSequenceType
{
/// Attribute "InstanceId" of XSD type xs:unsignedInt.
   @unsigned int                         InstanceId                     1;	///< Required attribute.
/// Attribute "SequenceId" of XSD type xs:anyURI.
   @char*                                SequenceId                     0;	///< Optional attribute.
/// Attribute "MessageNumber" of XSD type xs:unsignedInt.
   @unsigned int                         MessageNumber                  1;	///< Required attribute.
/// @todo <anyAttribute namespace="##other">.
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
///       Use wsdl2h option -d for xsd__anyAttribute DOM (soap_dom_attribute).
};


/******************************************************************************\
 *                                                                            *
 * Additional Top-Level Elements                                              *
 *   http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01                    *
 *                                                                            *
\******************************************************************************/

/// @brief Top-level root element "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":Hello of XSD type "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":HelloType.
typedef struct wsdd__HelloType _wsdd__Hello;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":Bye of XSD type "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":ByeType.
typedef struct wsdd__ByeType _wsdd__Bye;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":Probe of XSD type "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":ProbeType.
typedef struct wsdd__ProbeType _wsdd__Probe;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":ProbeMatches of XSD type "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":ProbeMatchesType.
typedef struct wsdd__ProbeMatchesType _wsdd__ProbeMatches;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":Resolve of XSD type "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":ResolveType.
typedef struct wsdd__ResolveType _wsdd__Resolve;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":ResolveMatches of XSD type "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":ResolveMatchesType.
typedef struct wsdd__ResolveMatchesType _wsdd__ResolveMatches;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":Types of XSD type "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":QNameListType.
typedef wsdd__QNameListType _wsdd__Types;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":Scopes of XSD type "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":ScopesType.
typedef struct wsdd__ScopesType _wsdd__Scopes;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":XAddrs of XSD type "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":UriListType.
typedef wsdd__UriListType _wsdd__XAddrs;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":MetadataVersion of XSD type xs:unsignedInt.
typedef unsigned int _wsdd__MetadataVersion;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":SupportedMatchingRules of XSD type "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":UriListType.
typedef wsdd__UriListType _wsdd__SupportedMatchingRules;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":Security of XSD type "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":SecurityType.
typedef struct wsdd__SecurityType _wsdd__Security;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":Sig of XSD type "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":SigType.
typedef struct wsdd__SigType _wsdd__Sig;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":AppSequence of XSD type "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":AppSequenceType.
typedef struct wsdd__AppSequenceType _wsdd__AppSequence;


/******************************************************************************\
 *                                                                            *
 * Additional Top-Level Attributes                                            *
 *   http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01                    *
 *                                                                            *
\******************************************************************************/

/// @brief Top-level attribute "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01":Id of simpleType xs:ID.
typedef char*  _wsdd__Id;


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

Data can be read and deserialized from:
  - an int file descriptor, using soap->recvfd = fd
  - a socket, using soap->socket = (int)...
  - a C++ stream (istream, stringstream), using soap->is = (istream*)...
  - a C string, using soap->is = (const char*)...
  - any input, using the soap->frecv() callback

Data can be serialized and written to:
  - an int file descriptor, using soap->sendfd = (int)...
  - a socket, using soap->socket = (int)...
  - a C++ stream (ostream, stringstream), using soap->os = (ostream*)...
  - a C string, using soap->os = (const char**)...
  - any output, using the soap->fsend() callback

The following options are available for (de)serialization control:
  - soap->encodingStyle = NULL; to remove SOAP 1.1/1.2 encodingStyle
  - soap_mode(soap, SOAP_XML_TREE); XML without id-ref (no cycles!)
  - soap_mode(soap, SOAP_XML_GRAPH); XML with id-ref (including cycles)
  - soap_set_namespaces(soap, struct Namespace *nsmap); to set xmlns bindings


@section wsdd Top-level root elements of schema "http://docs.oasis-open.org/ws-dd/ns/discovery/2009/01"

  - <wsdd:Hello> @ref _wsdd__Hello
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wsdd__Hello(struct soap*, _wsdd__Hello*);
    // Writer (returns SOAP_OK on success):
    soap_write__wsdd__Hello(struct soap*, _wsdd__Hello*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wsdd__Hello(struct soap*, const char *URL, _wsdd__Hello*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wsdd__Hello(struct soap*, const char *URL, _wsdd__Hello*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wsdd__Hello(struct soap*, const char *URL, _wsdd__Hello*);
    soap_POST_recv__wsdd__Hello(struct soap*, _wsdd__Hello*);
    @endcode

  - <wsdd:Bye> @ref _wsdd__Bye
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wsdd__Bye(struct soap*, _wsdd__Bye*);
    // Writer (returns SOAP_OK on success):
    soap_write__wsdd__Bye(struct soap*, _wsdd__Bye*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wsdd__Bye(struct soap*, const char *URL, _wsdd__Bye*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wsdd__Bye(struct soap*, const char *URL, _wsdd__Bye*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wsdd__Bye(struct soap*, const char *URL, _wsdd__Bye*);
    soap_POST_recv__wsdd__Bye(struct soap*, _wsdd__Bye*);
    @endcode

  - <wsdd:Probe> @ref _wsdd__Probe
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wsdd__Probe(struct soap*, _wsdd__Probe*);
    // Writer (returns SOAP_OK on success):
    soap_write__wsdd__Probe(struct soap*, _wsdd__Probe*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wsdd__Probe(struct soap*, const char *URL, _wsdd__Probe*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wsdd__Probe(struct soap*, const char *URL, _wsdd__Probe*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wsdd__Probe(struct soap*, const char *URL, _wsdd__Probe*);
    soap_POST_recv__wsdd__Probe(struct soap*, _wsdd__Probe*);
    @endcode

  - <wsdd:ProbeMatches> @ref _wsdd__ProbeMatches
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wsdd__ProbeMatches(struct soap*, _wsdd__ProbeMatches*);
    // Writer (returns SOAP_OK on success):
    soap_write__wsdd__ProbeMatches(struct soap*, _wsdd__ProbeMatches*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wsdd__ProbeMatches(struct soap*, const char *URL, _wsdd__ProbeMatches*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wsdd__ProbeMatches(struct soap*, const char *URL, _wsdd__ProbeMatches*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wsdd__ProbeMatches(struct soap*, const char *URL, _wsdd__ProbeMatches*);
    soap_POST_recv__wsdd__ProbeMatches(struct soap*, _wsdd__ProbeMatches*);
    @endcode

  - <wsdd:Resolve> @ref _wsdd__Resolve
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wsdd__Resolve(struct soap*, _wsdd__Resolve*);
    // Writer (returns SOAP_OK on success):
    soap_write__wsdd__Resolve(struct soap*, _wsdd__Resolve*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wsdd__Resolve(struct soap*, const char *URL, _wsdd__Resolve*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wsdd__Resolve(struct soap*, const char *URL, _wsdd__Resolve*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wsdd__Resolve(struct soap*, const char *URL, _wsdd__Resolve*);
    soap_POST_recv__wsdd__Resolve(struct soap*, _wsdd__Resolve*);
    @endcode

  - <wsdd:ResolveMatches> @ref _wsdd__ResolveMatches
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wsdd__ResolveMatches(struct soap*, _wsdd__ResolveMatches*);
    // Writer (returns SOAP_OK on success):
    soap_write__wsdd__ResolveMatches(struct soap*, _wsdd__ResolveMatches*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wsdd__ResolveMatches(struct soap*, const char *URL, _wsdd__ResolveMatches*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wsdd__ResolveMatches(struct soap*, const char *URL, _wsdd__ResolveMatches*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wsdd__ResolveMatches(struct soap*, const char *URL, _wsdd__ResolveMatches*);
    soap_POST_recv__wsdd__ResolveMatches(struct soap*, _wsdd__ResolveMatches*);
    @endcode

  - <wsdd:Types> @ref _wsdd__Types
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wsdd__Types(struct soap*, _wsdd__Types*);
    // Writer (returns SOAP_OK on success):
    soap_write__wsdd__Types(struct soap*, _wsdd__Types*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wsdd__Types(struct soap*, const char *URL, _wsdd__Types*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wsdd__Types(struct soap*, const char *URL, _wsdd__Types*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wsdd__Types(struct soap*, const char *URL, _wsdd__Types*);
    soap_POST_recv__wsdd__Types(struct soap*, _wsdd__Types*);
    @endcode

  - <wsdd:Scopes> @ref _wsdd__Scopes
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wsdd__Scopes(struct soap*, _wsdd__Scopes*);
    // Writer (returns SOAP_OK on success):
    soap_write__wsdd__Scopes(struct soap*, _wsdd__Scopes*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wsdd__Scopes(struct soap*, const char *URL, _wsdd__Scopes*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wsdd__Scopes(struct soap*, const char *URL, _wsdd__Scopes*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wsdd__Scopes(struct soap*, const char *URL, _wsdd__Scopes*);
    soap_POST_recv__wsdd__Scopes(struct soap*, _wsdd__Scopes*);
    @endcode

  - <wsdd:XAddrs> @ref _wsdd__XAddrs
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wsdd__XAddrs(struct soap*, _wsdd__XAddrs*);
    // Writer (returns SOAP_OK on success):
    soap_write__wsdd__XAddrs(struct soap*, _wsdd__XAddrs*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wsdd__XAddrs(struct soap*, const char *URL, _wsdd__XAddrs*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wsdd__XAddrs(struct soap*, const char *URL, _wsdd__XAddrs*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wsdd__XAddrs(struct soap*, const char *URL, _wsdd__XAddrs*);
    soap_POST_recv__wsdd__XAddrs(struct soap*, _wsdd__XAddrs*);
    @endcode

  - <wsdd:MetadataVersion> @ref _wsdd__MetadataVersion
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wsdd__MetadataVersion(struct soap*, _wsdd__MetadataVersion*);
    // Writer (returns SOAP_OK on success):
    soap_write__wsdd__MetadataVersion(struct soap*, _wsdd__MetadataVersion*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wsdd__MetadataVersion(struct soap*, const char *URL, _wsdd__MetadataVersion*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wsdd__MetadataVersion(struct soap*, const char *URL, _wsdd__MetadataVersion*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wsdd__MetadataVersion(struct soap*, const char *URL, _wsdd__MetadataVersion*);
    soap_POST_recv__wsdd__MetadataVersion(struct soap*, _wsdd__MetadataVersion*);
    @endcode

  - <wsdd:SupportedMatchingRules> @ref _wsdd__SupportedMatchingRules
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wsdd__SupportedMatchingRules(struct soap*, _wsdd__SupportedMatchingRules*);
    // Writer (returns SOAP_OK on success):
    soap_write__wsdd__SupportedMatchingRules(struct soap*, _wsdd__SupportedMatchingRules*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wsdd__SupportedMatchingRules(struct soap*, const char *URL, _wsdd__SupportedMatchingRules*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wsdd__SupportedMatchingRules(struct soap*, const char *URL, _wsdd__SupportedMatchingRules*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wsdd__SupportedMatchingRules(struct soap*, const char *URL, _wsdd__SupportedMatchingRules*);
    soap_POST_recv__wsdd__SupportedMatchingRules(struct soap*, _wsdd__SupportedMatchingRules*);
    @endcode

  - <wsdd:Security> @ref _wsdd__Security
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wsdd__Security(struct soap*, _wsdd__Security*);
    // Writer (returns SOAP_OK on success):
    soap_write__wsdd__Security(struct soap*, _wsdd__Security*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wsdd__Security(struct soap*, const char *URL, _wsdd__Security*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wsdd__Security(struct soap*, const char *URL, _wsdd__Security*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wsdd__Security(struct soap*, const char *URL, _wsdd__Security*);
    soap_POST_recv__wsdd__Security(struct soap*, _wsdd__Security*);
    @endcode

  - <wsdd:Sig> @ref _wsdd__Sig
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wsdd__Sig(struct soap*, _wsdd__Sig*);
    // Writer (returns SOAP_OK on success):
    soap_write__wsdd__Sig(struct soap*, _wsdd__Sig*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wsdd__Sig(struct soap*, const char *URL, _wsdd__Sig*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wsdd__Sig(struct soap*, const char *URL, _wsdd__Sig*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wsdd__Sig(struct soap*, const char *URL, _wsdd__Sig*);
    soap_POST_recv__wsdd__Sig(struct soap*, _wsdd__Sig*);
    @endcode

  - <wsdd:AppSequence> @ref _wsdd__AppSequence
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wsdd__AppSequence(struct soap*, _wsdd__AppSequence*);
    // Writer (returns SOAP_OK on success):
    soap_write__wsdd__AppSequence(struct soap*, _wsdd__AppSequence*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wsdd__AppSequence(struct soap*, const char *URL, _wsdd__AppSequence*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wsdd__AppSequence(struct soap*, const char *URL, _wsdd__AppSequence*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wsdd__AppSequence(struct soap*, const char *URL, _wsdd__AppSequence*);
    soap_POST_recv__wsdd__AppSequence(struct soap*, _wsdd__AppSequence*);
    @endcode

*/

#import "wsdx.h"

/* End of wsdd.h */
