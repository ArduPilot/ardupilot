/*
	wst.h

	WS-Trust 2005/12 with SAML 1.0/2.0, also accepts 2005/02
	Generated with:
	wsdl2h -cguxy -o wst.h -t WS/WS-typemap.dat WS/WS-Trust.xsd

        Requires:
        - plugin/wsseapi.h and plugin/wsseapi.c
        - plugin/mecevp.c
        - plugin/smdevp.c
        - plugin/wsaapi.h and plugin/wsaapi.c
        - plugin/wstapi.h and plugin/wstapi.c
        - custom/struct_timeval.c

        This file imports:
        - wsse.h
        - saml1.h
        - saml2.h
        - custom/struct_timeval.h
        - wsu.h
        - xenc.h
        - ds.h
        - c14n.h
        - wsa5.h
        - wsp_appliesto.h
        - wstx.h

	- Removed //gsoapopt
        - Added #import "wsse11.h" to choose between WS-Security 1.1 and 1.0
	- Changed http://docs.oasis-open.org/ws-sx/ws-trust/200512 to remove trailing /
	- Changed //gsoap wst schema namespace directive to import directive
        - Added //gsoap wst schema namespace2 directive
	- Added #import "wsp_appliesto.h"
	- Added #import "wstx.h" at the end of these definitions

*/

/******************************************************************************\
 *                                                                            *
 * Definitions                                                                *
 *   http://docs.oasis-open.org/ws-sx/ws-trust/200512/                        *
 *                                                                            *
\******************************************************************************/


/******************************************************************************\
 *                                                                            *
 * Import                                                                     *
 *                                                                            *
\******************************************************************************/

// Choose between WS-Security 1.1 or 1.0:
// #import "wsse11.h"	// wsse = <http://docs.oasis-open.org/wss/oasis-wss-wssecurity-secext-1.1.xsd>
#import "wsse.h"	// wsse = <http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-secext-1.0.xsd>

#import "wsu.h" 	// wsu = <http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd>
#import "wsa5.h"	// wsa5 = <http://www.w3.org/2005/08/addressing>
#import "wsp_appliesto.h"

/******************************************************************************\
 *                                                                            *
 * Schema Namespaces                                                          *
 *                                                                            *
\******************************************************************************/

#define SOAP_NAMESPACE_OF_wst	"http://docs.oasis-open.org/ws-sx/ws-trust/200512"
//gsoap wst   schema import:	http://docs.oasis-open.org/ws-sx/ws-trust/200512
//gsoap wst   schema namespace2:	http://schemas.xmlsoap.org/ws/2005/02/trust
//gsoap wst   schema elementForm:	qualified
//gsoap wst   schema attributeForm:	unqualified

/******************************************************************************\
 *                                                                            *
 * Built-in Schema Types and Top-Level Elements and Attributes                *
 *                                                                            *
\******************************************************************************/

// Imported XSD type ""http://www.w3.org/2005/08/addressing":EndpointReferenceType" defined by wsa5__EndpointReferenceType.
/// Imported element ""http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-secext-1.0.xsd":SecurityTokenReference" from typemap WS/WS-typemap.dat.

/// @brief Typedef synonym for struct wst__RequestSecurityTokenType.
typedef struct wst__RequestSecurityTokenType wst__RequestSecurityTokenType;
/// @brief Typedef synonym for struct wst__RequestSecurityTokenResponseType.
typedef struct wst__RequestSecurityTokenResponseType wst__RequestSecurityTokenResponseType;
/// Imported complexType "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":RequestedSecurityTokenType from typemap "WS/WS-typemap.dat".
#import "saml1.h"
#import "saml2.h"
typedef struct wst__RequestedSecurityTokenType
{	saml1__AssertionType *saml1__Assertion;
	saml2__AssertionType *saml2__Assertion;
        _wsse__SecurityTokenReference *wsse__SecurityTokenReference;
        struct wsc__SecurityContextTokenType *wsc__SecurityContextToken;
} wst__RequestedSecurityTokenType;

/// @brief Typedef synonym for struct wst__BinarySecretType.
typedef struct wst__BinarySecretType wst__BinarySecretType;
/// @brief Typedef synonym for struct wst__ClaimsType.
typedef struct wst__ClaimsType wst__ClaimsType;
/// Imported complexType "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":EntropyType from typemap "WS/WS-typemap.dat".
typedef struct wst__EntropyType
{	struct wst__BinarySecretType *BinarySecret;
} wst__EntropyType;

/// @brief Typedef synonym for struct wst__LifetimeType.
typedef struct wst__LifetimeType wst__LifetimeType;
/// @brief Typedef synonym for struct wst__RequestSecurityTokenCollectionType.
typedef struct wst__RequestSecurityTokenCollectionType wst__RequestSecurityTokenCollectionType;
/// @brief Typedef synonym for struct wst__RequestSecurityTokenResponseCollectionType.
typedef struct wst__RequestSecurityTokenResponseCollectionType wst__RequestSecurityTokenResponseCollectionType;
/// @brief Typedef synonym for struct wst__RequestedReferenceType.
typedef struct wst__RequestedReferenceType wst__RequestedReferenceType;
/// @brief Typedef synonym for struct wst__RequestedProofTokenType.
typedef struct wst__RequestedProofTokenType wst__RequestedProofTokenType;
/// @brief Typedef synonym for struct wst__RenewTargetType.
typedef struct wst__RenewTargetType wst__RenewTargetType;
/// @brief Typedef synonym for struct wst__AllowPostdatingType.
typedef struct wst__AllowPostdatingType wst__AllowPostdatingType;
/// @brief Typedef synonym for struct wst__RenewingType.
typedef struct wst__RenewingType wst__RenewingType;
/// @brief Typedef synonym for struct wst__CancelTargetType.
typedef struct wst__CancelTargetType wst__CancelTargetType;
/// @brief Typedef synonym for struct wst__RequestedTokenCancelledType.
typedef struct wst__RequestedTokenCancelledType wst__RequestedTokenCancelledType;
/// @brief Typedef synonym for struct wst__ValidateTargetType.
typedef struct wst__ValidateTargetType wst__ValidateTargetType;
/// @brief Typedef synonym for struct wst__StatusType.
typedef struct wst__StatusType wst__StatusType;
/// @brief Typedef synonym for struct wst__SignChallengeType.
typedef struct wst__SignChallengeType wst__SignChallengeType;
/// @brief Typedef synonym for struct wst__BinaryExchangeType.
typedef struct wst__BinaryExchangeType wst__BinaryExchangeType;
/// @brief Typedef synonym for struct wst__RequestKETType.
typedef struct wst__RequestKETType wst__RequestKETType;
/// @brief Typedef synonym for struct wst__KeyExchangeTokenType.
typedef struct wst__KeyExchangeTokenType wst__KeyExchangeTokenType;
/// Imported complexType "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":AuthenticatorType from typemap "WS/WS-typemap.dat".
typedef struct wst__AuthenticatorType
{	char *CombinedHash;
} wst__AuthenticatorType;

/// @brief Typedef synonym for struct wst__OnBehalfOfType.
typedef struct wst__OnBehalfOfType wst__OnBehalfOfType;
/// @brief Typedef synonym for struct wst__EncryptionType.
typedef struct wst__EncryptionType wst__EncryptionType;
/// @brief Typedef synonym for struct wst__ProofEncryptionType.
typedef struct wst__ProofEncryptionType wst__ProofEncryptionType;
/// @brief Typedef synonym for struct wst__UseKeyType.
typedef struct wst__UseKeyType wst__UseKeyType;
/// @brief Typedef synonym for struct wst__DelegateToType.
typedef struct wst__DelegateToType wst__DelegateToType;
/// @brief Typedef synonym for struct wst__ParticipantsType.
typedef struct wst__ParticipantsType wst__ParticipantsType;
/// @brief Typedef synonym for struct wst__ParticipantType.
typedef struct wst__ParticipantType wst__ParticipantType;

/******************************************************************************\
 *                                                                            *
 * Schema Types and Top-Level Elements and Attributes                         *
 *   http://docs.oasis-open.org/ws-sx/ws-trust/200512/                        *
 *                                                                            *
\******************************************************************************/

/// Imported simpleType "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":RequestTypeOpenEnum from typemap "WS/WS-typemap.dat".
// simpleType definition intentionally left blank.
/// Imported simpleType "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":BinarySecretTypeOpenEnum from typemap "WS/WS-typemap.dat".
// simpleType definition intentionally left blank.
/// Imported simpleType "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":ComputedKeyOpenEnum from typemap "WS/WS-typemap.dat".
// simpleType definition intentionally left blank.
/// Imported simpleType "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":StatusCodeOpenEnum from typemap "WS/WS-typemap.dat".
// simpleType definition intentionally left blank.
/// Imported simpleType "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":KeyTypeOpenEnum from typemap "WS/WS-typemap.dat".
// simpleType definition intentionally left blank.
/// Imported simpleType "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":RequestTypeEnum from typemap "WS/WS-typemap.dat".
// simpleType definition intentionally left blank.
/// Imported simpleType "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":BinarySecretTypeEnum from typemap "WS/WS-typemap.dat".
// simpleType definition intentionally left blank.
/// Imported simpleType "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":ComputedKeyEnum from typemap "WS/WS-typemap.dat".
// simpleType definition intentionally left blank.
/// Imported simpleType "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":StatusCodeEnum from typemap "WS/WS-typemap.dat".
// simpleType definition intentionally left blank.
/// Imported simpleType "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":KeyTypeEnum from typemap "WS/WS-typemap.dat".
// simpleType definition intentionally left blank.

/******************************************************************************\
 *                                                                            *
 * Schema Complex Types and Top-Level Elements                                *
 *   http://docs.oasis-open.org/ws-sx/ws-trust/200512/                        *
 *                                                                            *
\******************************************************************************/

/// @brief "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":RequestSecurityTokenType is a complexType.
///
/// <PRE><BLOCKQUOTE>
///   Actual content model is non-deterministic, hence wildcard. The following shows intended content model:
///   <xs:element ref='wst:TokenType' minOccurs='0' />
///   <xs:element ref='wst:RequestType' />
///   <xs:element ref='wsp:AppliesTo' minOccurs='0' />
///   <xs:element ref='wst:Claims' minOccurs='0' />
///   <xs:element ref='wst:Entropy' minOccurs='0' />
///   <xs:element ref='wst:Lifetime' minOccurs='0' />
///   <xs:element ref='wst:AllowPostdating' minOccurs='0' />
///   <xs:element ref='wst:Renewing' minOccurs='0' />
///   <xs:element ref='wst:OnBehalfOf' minOccurs='0' />
///   <xs:element ref='wst:Issuer' minOccurs='0' />
///   <xs:element ref='wst:AuthenticationType' minOccurs='0' />
///   <xs:element ref='wst:KeyType' minOccurs='0' />
///   <xs:element ref='wst:KeySize' minOccurs='0' />
///   <xs:element ref='wst:SignatureAlgorithm' minOccurs='0' />
///   <xs:element ref='wst:Encryption' minOccurs='0' />
///   <xs:element ref='wst:EncryptionAlgorithm' minOccurs='0' />
///   <xs:element ref='wst:CanonicalizationAlgorithm' minOccurs='0' />
///   <xs:element ref='wst:ProofEncryption' minOccurs='0' />
///   <xs:element ref='wst:UseKey' minOccurs='0' />
///   <xs:element ref='wst:SignWith' minOccurs='0' />
///   <xs:element ref='wst:EncryptWith' minOccurs='0' />
///   <xs:element ref='wst:DelegateTo' minOccurs='0' />
///   <xs:element ref='wst:Forwardable' minOccurs='0' />
///   <xs:element ref='wst:Delegatable' minOccurs='0' />
///   <xs:element ref='wsp:Policy' minOccurs='0' />
///   <xs:element ref='wsp:PolicyReference' minOccurs='0' />
///   <xs:any namespace='##other' processContents='lax' minOccurs='0' maxOccurs='unbounded' />
/// </BLOCKQUOTE></PRE>
/// struct wst__RequestSecurityTokenType operations:
/// - wst__RequestSecurityTokenType* soap_new_wst__RequestSecurityTokenType(struct soap*, int num) allocate and default initialize one or more values (array)
/// - soap_default_wst__RequestSecurityTokenType(struct soap*, wst__RequestSecurityTokenType*) default initialize members
/// - int soap_read_wst__RequestSecurityTokenType(struct soap*, wst__RequestSecurityTokenType*) deserialize from a source
/// - int soap_write_wst__RequestSecurityTokenType(struct soap*, wst__RequestSecurityTokenType*) serialize to a sink
/// - wst__RequestSecurityTokenType* soap_dup_wst__RequestSecurityTokenType(struct soap*, wst__RequestSecurityTokenType* dst, wst__RequestSecurityTokenType *src) returns deep copy of wst__RequestSecurityTokenType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_wst__RequestSecurityTokenType(wst__RequestSecurityTokenType*) deep deletes wst__RequestSecurityTokenType data members, use only on dst after soap_dup_wst__RequestSecurityTokenType(NULL, wst__RequestSecurityTokenType *dst, wst__RequestSecurityTokenType *src) (use soapcpp2 -Ed)
struct wst__RequestSecurityTokenType
{
/// @todo <any namespace="##any" minOccurs="0" maxOccurs="unbounded">
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element):
///       wsdl2h maps xsd:any to xsd__anyType, use typemap.dat to remap.
/// Attribute "Context" of XSD type xs:anyURI.
   @char*                                Context                        0;	///< Optional attribute.
/// @todo <anyAttribute namespace="##other">.
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
///       Use wsdl2h option -d for xsd__anyAttribute DOM (soap_dom_attribute).
/// Member declared in WS/WS-typemap.dat
       _wsp__AppliesTo_*                    wsp__AppliesTo;
/// Member declared in WS/WS-typemap.dat
       char*                                KeyType;
/// Member declared in WS/WS-typemap.dat
       char*                                RequestType;
/// Member declared in WS/WS-typemap.dat
       char*                                TokenType;
/// Member declared in WS/WS-typemap.dat
       wst__EntropyType*                    Entropy;
/// Member declared in WS/WS-typemap.dat
       char*                                ComputedKeyAlgorithm;
/// Member declared in WS/WS-typemap.dat
       unsigned int*                        KeySize;
/// Member declared in WS/WS-typemap.dat
       struct wst__CancelTargetType*        CancelTarget;
/// Member declared in WS/WS-typemap.dat
       struct wst__BinaryExchangeType*      BinaryExchange;
/// Member declared in WS/WS-typemap.dat
       struct wst__AuthenticatorType*       Authenticator;
};

/// @brief "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":RequestSecurityTokenResponseType is a complexType.
///
/// <PRE><BLOCKQUOTE>
///   Actual content model is non-deterministic, hence wildcard. The following shows intended content model:
///   <xs:element ref='wst:TokenType' minOccurs='0' />
///   <xs:element ref='wst:RequestType' />
///   <xs:element ref='wst:RequestedSecurityToken' minOccurs='0' />
///   <xs:element ref='wsp:AppliesTo' minOccurs='0' />
///   <xs:element ref='wst:RequestedAttachedReference' minOccurs='0' />
///   <xs:element ref='wst:RequestedUnattachedReference' minOccurs='0' />
///   <xs:element ref='wst:RequestedProofToken' minOccurs='0' />
///   <xs:element ref='wst:Entropy' minOccurs='0' />
///   <xs:element ref='wst:Lifetime' minOccurs='0' />
///   <xs:element ref='wst:Status' minOccurs='0' />
///   <xs:element ref='wst:AllowPostdating' minOccurs='0' />
///   <xs:element ref='wst:Renewing' minOccurs='0' />
///   <xs:element ref='wst:OnBehalfOf' minOccurs='0' />
///   <xs:element ref='wst:Issuer' minOccurs='0' />
///   <xs:element ref='wst:AuthenticationType' minOccurs='0' />
///   <xs:element ref='wst:Authenticator' minOccurs='0' />
///   <xs:element ref='wst:KeyType' minOccurs='0' />
///   <xs:element ref='wst:KeySize' minOccurs='0' />
///   <xs:element ref='wst:SignatureAlgorithm' minOccurs='0' />
///   <xs:element ref='wst:Encryption' minOccurs='0' />
///   <xs:element ref='wst:EncryptionAlgorithm' minOccurs='0' />
///   <xs:element ref='wst:CanonicalizationAlgorithm' minOccurs='0' />
///   <xs:element ref='wst:ProofEncryption' minOccurs='0' />
///   <xs:element ref='wst:UseKey' minOccurs='0' />
///   <xs:element ref='wst:SignWith' minOccurs='0' />
///   <xs:element ref='wst:EncryptWith' minOccurs='0' />
///   <xs:element ref='wst:DelegateTo' minOccurs='0' />
///   <xs:element ref='wst:Forwardable' minOccurs='0' />
///   <xs:element ref='wst:Delegatable' minOccurs='0' />
///   <xs:element ref='wsp:Policy' minOccurs='0' />
///   <xs:element ref='wsp:PolicyReference' minOccurs='0' />
///   <xs:any namespace='##other' processContents='lax' minOccurs='0' maxOccurs='unbounded' />
/// </BLOCKQUOTE></PRE>
/// struct wst__RequestSecurityTokenResponseType operations:
/// - wst__RequestSecurityTokenResponseType* soap_new_wst__RequestSecurityTokenResponseType(struct soap*, int num) allocate and default initialize one or more values (array)
/// - soap_default_wst__RequestSecurityTokenResponseType(struct soap*, wst__RequestSecurityTokenResponseType*) default initialize members
/// - int soap_read_wst__RequestSecurityTokenResponseType(struct soap*, wst__RequestSecurityTokenResponseType*) deserialize from a source
/// - int soap_write_wst__RequestSecurityTokenResponseType(struct soap*, wst__RequestSecurityTokenResponseType*) serialize to a sink
/// - wst__RequestSecurityTokenResponseType* soap_dup_wst__RequestSecurityTokenResponseType(struct soap*, wst__RequestSecurityTokenResponseType* dst, wst__RequestSecurityTokenResponseType *src) returns deep copy of wst__RequestSecurityTokenResponseType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_wst__RequestSecurityTokenResponseType(wst__RequestSecurityTokenResponseType*) deep deletes wst__RequestSecurityTokenResponseType data members, use only on dst after soap_dup_wst__RequestSecurityTokenResponseType(NULL, wst__RequestSecurityTokenResponseType *dst, wst__RequestSecurityTokenResponseType *src) (use soapcpp2 -Ed)
struct wst__RequestSecurityTokenResponseType
{
/// @todo <any namespace="##any" minOccurs="0" maxOccurs="unbounded">
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element):
///       wsdl2h maps xsd:any to xsd__anyType, use typemap.dat to remap.
/// Attribute "Context" of XSD type xs:anyURI.
   @char*                                Context                        0;	///< Optional attribute.
/// @todo <anyAttribute namespace="##other">.
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
///       Use wsdl2h option -d for xsd__anyAttribute DOM (soap_dom_attribute).
/// Member declared in WS/WS-typemap.dat
       struct wst__RequestedSecurityTokenType* RequestedSecurityToken;
/// Member declared in WS/WS-typemap.dat
       struct wst__RequestedReferenceType*  RequestedAttachedReference;
/// Member declared in WS/WS-typemap.dat
       struct wst__RequestedReferenceType*  RequestedUnattachedReference;
/// Member declared in WS/WS-typemap.dat
       struct wst__RequestedProofTokenType* RequestedProofToken;
/// Member declared in WS/WS-typemap.dat
       struct wst__RequestedTokenCancelledType* RequestedTokenCancelled;
/// Member declared in WS/WS-typemap.dat
       char*                                KeyType;
/// Member declared in WS/WS-typemap.dat
       char*                                RequestType;
/// Member declared in WS/WS-typemap.dat
       char*                                TokenType;
/// Member declared in WS/WS-typemap.dat
       wst__EntropyType*                    Entropy;
/// Member declared in WS/WS-typemap.dat
       struct wst__LifetimeType*            Lifetime;
/// Member declared in WS/WS-typemap.dat
       unsigned int*                        KeySize;
/// Member declared in WS/WS-typemap.dat
       struct wst__BinaryExchangeType*      BinaryExchange;
/// Member declared in WS/WS-typemap.dat
       struct wst__AuthenticatorType*       Authenticator;
};

/// @brief "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":ClaimsType is a complexType.
///
/// struct wst__ClaimsType operations:
/// - wst__ClaimsType* soap_new_wst__ClaimsType(struct soap*, int num) allocate and default initialize one or more values (array)
/// - soap_default_wst__ClaimsType(struct soap*, wst__ClaimsType*) default initialize members
/// - int soap_read_wst__ClaimsType(struct soap*, wst__ClaimsType*) deserialize from a source
/// - int soap_write_wst__ClaimsType(struct soap*, wst__ClaimsType*) serialize to a sink
/// - wst__ClaimsType* soap_dup_wst__ClaimsType(struct soap*, wst__ClaimsType* dst, wst__ClaimsType *src) returns deep copy of wst__ClaimsType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_wst__ClaimsType(wst__ClaimsType*) deep deletes wst__ClaimsType data members, use only on dst after soap_dup_wst__ClaimsType(NULL, wst__ClaimsType *dst, wst__ClaimsType *src) (use soapcpp2 -Ed)
struct wst__ClaimsType
{
/// @todo <any namespace="##any" minOccurs="0" maxOccurs="unbounded">
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element):
///       wsdl2h maps xsd:any to xsd__anyType, use typemap.dat to remap.
/// Attribute "Dialect" of XSD type xs:anyURI.
   @char*                                Dialect                        0;	///< Optional attribute.
/// @todo <anyAttribute namespace="##other">.
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
///       Use wsdl2h option -d for xsd__anyAttribute DOM (soap_dom_attribute).
};

/// @brief "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":LifetimeType is a complexType.
///
/// struct wst__LifetimeType operations:
/// - wst__LifetimeType* soap_new_wst__LifetimeType(struct soap*, int num) allocate and default initialize one or more values (array)
/// - soap_default_wst__LifetimeType(struct soap*, wst__LifetimeType*) default initialize members
/// - int soap_read_wst__LifetimeType(struct soap*, wst__LifetimeType*) deserialize from a source
/// - int soap_write_wst__LifetimeType(struct soap*, wst__LifetimeType*) serialize to a sink
/// - wst__LifetimeType* soap_dup_wst__LifetimeType(struct soap*, wst__LifetimeType* dst, wst__LifetimeType *src) returns deep copy of wst__LifetimeType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_wst__LifetimeType(wst__LifetimeType*) deep deletes wst__LifetimeType data members, use only on dst after soap_dup_wst__LifetimeType(NULL, wst__LifetimeType *dst, wst__LifetimeType *src) (use soapcpp2 -Ed)
struct wst__LifetimeType
{
/// Imported element reference "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd":Created.
    char*                                wsu__Created                   0;	///< Optional element.
/// Imported element reference "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd":Expires.
    char*                                wsu__Expires                   0;	///< Optional element.
};

/// @brief "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":RequestSecurityTokenCollectionType is a complexType.
///
/// <PRE><BLOCKQUOTE>
///   The RequestSecurityTokenCollection (RSTC) element is used to provide multiple RST requests. One or more RSTR elements in an RSTRC element are returned in the response to the RequestSecurityTokenCollection.
/// </BLOCKQUOTE></PRE>
/// struct wst__RequestSecurityTokenCollectionType operations:
/// - wst__RequestSecurityTokenCollectionType* soap_new_wst__RequestSecurityTokenCollectionType(struct soap*, int num) allocate and default initialize one or more values (array)
/// - soap_default_wst__RequestSecurityTokenCollectionType(struct soap*, wst__RequestSecurityTokenCollectionType*) default initialize members
/// - int soap_read_wst__RequestSecurityTokenCollectionType(struct soap*, wst__RequestSecurityTokenCollectionType*) deserialize from a source
/// - int soap_write_wst__RequestSecurityTokenCollectionType(struct soap*, wst__RequestSecurityTokenCollectionType*) serialize to a sink
/// - wst__RequestSecurityTokenCollectionType* soap_dup_wst__RequestSecurityTokenCollectionType(struct soap*, wst__RequestSecurityTokenCollectionType* dst, wst__RequestSecurityTokenCollectionType *src) returns deep copy of wst__RequestSecurityTokenCollectionType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_wst__RequestSecurityTokenCollectionType(wst__RequestSecurityTokenCollectionType*) deep deletes wst__RequestSecurityTokenCollectionType data members, use only on dst after soap_dup_wst__RequestSecurityTokenCollectionType(NULL, wst__RequestSecurityTokenCollectionType *dst, wst__RequestSecurityTokenCollectionType *src) (use soapcpp2 -Ed)
struct wst__RequestSecurityTokenCollectionType
{
/// Size of array of struct wst__RequestSecurityTokenType* is 2..unbounded.
   $int                                  __sizeRequestSecurityToken     2;
/// Array struct wst__RequestSecurityTokenType* of size 2..unbounded.
    struct wst__RequestSecurityTokenType*  RequestSecurityToken           2;
};

/// @brief "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":RequestSecurityTokenResponseCollectionType is a complexType.
///
/// <PRE><BLOCKQUOTE>
///   The <wst:RequestSecurityTokenResponseCollection> element (RSTRC) MUST be used to return a security token or response to a security token request on the final response.
/// </BLOCKQUOTE></PRE>
/// struct wst__RequestSecurityTokenResponseCollectionType operations:
/// - wst__RequestSecurityTokenResponseCollectionType* soap_new_wst__RequestSecurityTokenResponseCollectionType(struct soap*, int num) allocate and default initialize one or more values (array)
/// - soap_default_wst__RequestSecurityTokenResponseCollectionType(struct soap*, wst__RequestSecurityTokenResponseCollectionType*) default initialize members
/// - int soap_read_wst__RequestSecurityTokenResponseCollectionType(struct soap*, wst__RequestSecurityTokenResponseCollectionType*) deserialize from a source
/// - int soap_write_wst__RequestSecurityTokenResponseCollectionType(struct soap*, wst__RequestSecurityTokenResponseCollectionType*) serialize to a sink
/// - wst__RequestSecurityTokenResponseCollectionType* soap_dup_wst__RequestSecurityTokenResponseCollectionType(struct soap*, wst__RequestSecurityTokenResponseCollectionType* dst, wst__RequestSecurityTokenResponseCollectionType *src) returns deep copy of wst__RequestSecurityTokenResponseCollectionType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_wst__RequestSecurityTokenResponseCollectionType(wst__RequestSecurityTokenResponseCollectionType*) deep deletes wst__RequestSecurityTokenResponseCollectionType data members, use only on dst after soap_dup_wst__RequestSecurityTokenResponseCollectionType(NULL, wst__RequestSecurityTokenResponseCollectionType *dst, wst__RequestSecurityTokenResponseCollectionType *src) (use soapcpp2 -Ed)
struct wst__RequestSecurityTokenResponseCollectionType
{
/// Size of the dynamic array of struct wst__RequestSecurityTokenResponseType* is 1..unbounded.
   $int                                  __sizeRequestSecurityTokenResponse 1;
/// Array struct wst__RequestSecurityTokenResponseType* of size 1..unbounded.
    struct wst__RequestSecurityTokenResponseType*  RequestSecurityTokenResponse   1;
/// @todo <anyAttribute namespace="##other">.
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
///       Use wsdl2h option -d for xsd__anyAttribute DOM (soap_dom_attribute).
};

/// @brief "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":RequestedReferenceType is a complexType.
///
/// struct wst__RequestedReferenceType operations:
/// - wst__RequestedReferenceType* soap_new_wst__RequestedReferenceType(struct soap*, int num) allocate and default initialize one or more values (array)
/// - soap_default_wst__RequestedReferenceType(struct soap*, wst__RequestedReferenceType*) default initialize members
/// - int soap_read_wst__RequestedReferenceType(struct soap*, wst__RequestedReferenceType*) deserialize from a source
/// - int soap_write_wst__RequestedReferenceType(struct soap*, wst__RequestedReferenceType*) serialize to a sink
/// - wst__RequestedReferenceType* soap_dup_wst__RequestedReferenceType(struct soap*, wst__RequestedReferenceType* dst, wst__RequestedReferenceType *src) returns deep copy of wst__RequestedReferenceType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_wst__RequestedReferenceType(wst__RequestedReferenceType*) deep deletes wst__RequestedReferenceType data members, use only on dst after soap_dup_wst__RequestedReferenceType(NULL, wst__RequestedReferenceType *dst, wst__RequestedReferenceType *src) (use soapcpp2 -Ed)
struct wst__RequestedReferenceType
{
/// Imported element reference "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-secext-1.0.xsd":SecurityTokenReference.
    _wsse__SecurityTokenReference        wsse__SecurityTokenReference   1;	///< Required element.
};

/// @brief "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":RequestedProofTokenType is a complexType.
///
/// struct wst__RequestedProofTokenType operations:
/// - wst__RequestedProofTokenType* soap_new_wst__RequestedProofTokenType(struct soap*, int num) allocate and default initialize one or more values (array)
/// - soap_default_wst__RequestedProofTokenType(struct soap*, wst__RequestedProofTokenType*) default initialize members
/// - int soap_read_wst__RequestedProofTokenType(struct soap*, wst__RequestedProofTokenType*) deserialize from a source
/// - int soap_write_wst__RequestedProofTokenType(struct soap*, wst__RequestedProofTokenType*) serialize to a sink
/// - wst__RequestedProofTokenType* soap_dup_wst__RequestedProofTokenType(struct soap*, wst__RequestedProofTokenType* dst, wst__RequestedProofTokenType *src) returns deep copy of wst__RequestedProofTokenType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_wst__RequestedProofTokenType(wst__RequestedProofTokenType*) deep deletes wst__RequestedProofTokenType data members, use only on dst after soap_dup_wst__RequestedProofTokenType(NULL, wst__RequestedProofTokenType *dst, wst__RequestedProofTokenType *src) (use soapcpp2 -Ed)
struct wst__RequestedProofTokenType
{
/// @todo <any namespace="##any">
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element):
///       wsdl2h maps xsd:any to xsd__anyType, use typemap.dat to remap.
/// Member declared in WS/WS-typemap.dat
        struct xenc__EncryptedKeyType*       xenc__EncryptedKey;
};

/// @brief "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":RenewTargetType is a complexType.
///
/// struct wst__RenewTargetType operations:
/// - wst__RenewTargetType* soap_new_wst__RenewTargetType(struct soap*, int num) allocate and default initialize one or more values (array)
/// - soap_default_wst__RenewTargetType(struct soap*, wst__RenewTargetType*) default initialize members
/// - int soap_read_wst__RenewTargetType(struct soap*, wst__RenewTargetType*) deserialize from a source
/// - int soap_write_wst__RenewTargetType(struct soap*, wst__RenewTargetType*) serialize to a sink
/// - wst__RenewTargetType* soap_dup_wst__RenewTargetType(struct soap*, wst__RenewTargetType* dst, wst__RenewTargetType *src) returns deep copy of wst__RenewTargetType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_wst__RenewTargetType(wst__RenewTargetType*) deep deletes wst__RenewTargetType data members, use only on dst after soap_dup_wst__RenewTargetType(NULL, wst__RenewTargetType *dst, wst__RenewTargetType *src) (use soapcpp2 -Ed)
struct wst__RenewTargetType
{
/// @todo <any namespace="##other" minOccurs="1" maxOccurs="1">
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element):
///       wsdl2h maps xsd:any to xsd__anyType, use typemap.dat to remap.
};

/// @brief "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":AllowPostdatingType is a complexType.
///
/// struct wst__AllowPostdatingType operations:
/// - wst__AllowPostdatingType* soap_new_wst__AllowPostdatingType(struct soap*, int num) allocate and default initialize one or more values (array)
/// - soap_default_wst__AllowPostdatingType(struct soap*, wst__AllowPostdatingType*) default initialize members
/// - int soap_read_wst__AllowPostdatingType(struct soap*, wst__AllowPostdatingType*) deserialize from a source
/// - int soap_write_wst__AllowPostdatingType(struct soap*, wst__AllowPostdatingType*) serialize to a sink
/// - wst__AllowPostdatingType* soap_dup_wst__AllowPostdatingType(struct soap*, wst__AllowPostdatingType* dst, wst__AllowPostdatingType *src) returns deep copy of wst__AllowPostdatingType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_wst__AllowPostdatingType(wst__AllowPostdatingType*) deep deletes wst__AllowPostdatingType data members, use only on dst after soap_dup_wst__AllowPostdatingType(NULL, wst__AllowPostdatingType *dst, wst__AllowPostdatingType *src) (use soapcpp2 -Ed)
struct wst__AllowPostdatingType
{
};

/// @brief "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":RenewingType is a complexType.
///
/// struct wst__RenewingType operations:
/// - wst__RenewingType* soap_new_wst__RenewingType(struct soap*, int num) allocate and default initialize one or more values (array)
/// - soap_default_wst__RenewingType(struct soap*, wst__RenewingType*) default initialize members
/// - int soap_read_wst__RenewingType(struct soap*, wst__RenewingType*) deserialize from a source
/// - int soap_write_wst__RenewingType(struct soap*, wst__RenewingType*) serialize to a sink
/// - wst__RenewingType* soap_dup_wst__RenewingType(struct soap*, wst__RenewingType* dst, wst__RenewingType *src) returns deep copy of wst__RenewingType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_wst__RenewingType(wst__RenewingType*) deep deletes wst__RenewingType data members, use only on dst after soap_dup_wst__RenewingType(NULL, wst__RenewingType *dst, wst__RenewingType *src) (use soapcpp2 -Ed)
struct wst__RenewingType
{
/// Attribute "Allow" of XSD type xs:boolean.
   @char*                                Allow                          0;	///< Optional attribute.
/// Attribute "OK" of XSD type xs:boolean.
   @char*                                OK                             0;	///< Optional attribute.
};

/// @brief "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":CancelTargetType is a complexType.
///
/// struct wst__CancelTargetType operations:
/// - wst__CancelTargetType* soap_new_wst__CancelTargetType(struct soap*, int num) allocate and default initialize one or more values (array)
/// - soap_default_wst__CancelTargetType(struct soap*, wst__CancelTargetType*) default initialize members
/// - int soap_read_wst__CancelTargetType(struct soap*, wst__CancelTargetType*) deserialize from a source
/// - int soap_write_wst__CancelTargetType(struct soap*, wst__CancelTargetType*) serialize to a sink
/// - wst__CancelTargetType* soap_dup_wst__CancelTargetType(struct soap*, wst__CancelTargetType* dst, wst__CancelTargetType *src) returns deep copy of wst__CancelTargetType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_wst__CancelTargetType(wst__CancelTargetType*) deep deletes wst__CancelTargetType data members, use only on dst after soap_dup_wst__CancelTargetType(NULL, wst__CancelTargetType *dst, wst__CancelTargetType *src) (use soapcpp2 -Ed)
struct wst__CancelTargetType
{
/// @todo <any namespace="##other" minOccurs="1" maxOccurs="1">
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element):
///       wsdl2h maps xsd:any to xsd__anyType, use typemap.dat to remap.
/// Member declared in WS/WS-typemap.dat
    _wsse__SecurityTokenReference        wsse__SecurityTokenReference;
};

/// @brief "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":RequestedTokenCancelledType is a complexType.
///
/// struct wst__RequestedTokenCancelledType operations:
/// - wst__RequestedTokenCancelledType* soap_new_wst__RequestedTokenCancelledType(struct soap*, int num) allocate and default initialize one or more values (array)
/// - soap_default_wst__RequestedTokenCancelledType(struct soap*, wst__RequestedTokenCancelledType*) default initialize members
/// - int soap_read_wst__RequestedTokenCancelledType(struct soap*, wst__RequestedTokenCancelledType*) deserialize from a source
/// - int soap_write_wst__RequestedTokenCancelledType(struct soap*, wst__RequestedTokenCancelledType*) serialize to a sink
/// - wst__RequestedTokenCancelledType* soap_dup_wst__RequestedTokenCancelledType(struct soap*, wst__RequestedTokenCancelledType* dst, wst__RequestedTokenCancelledType *src) returns deep copy of wst__RequestedTokenCancelledType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_wst__RequestedTokenCancelledType(wst__RequestedTokenCancelledType*) deep deletes wst__RequestedTokenCancelledType data members, use only on dst after soap_dup_wst__RequestedTokenCancelledType(NULL, wst__RequestedTokenCancelledType *dst, wst__RequestedTokenCancelledType *src) (use soapcpp2 -Ed)
struct wst__RequestedTokenCancelledType
{
};

/// @brief "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":ValidateTargetType is a complexType.
///
/// struct wst__ValidateTargetType operations:
/// - wst__ValidateTargetType* soap_new_wst__ValidateTargetType(struct soap*, int num) allocate and default initialize one or more values (array)
/// - soap_default_wst__ValidateTargetType(struct soap*, wst__ValidateTargetType*) default initialize members
/// - int soap_read_wst__ValidateTargetType(struct soap*, wst__ValidateTargetType*) deserialize from a source
/// - int soap_write_wst__ValidateTargetType(struct soap*, wst__ValidateTargetType*) serialize to a sink
/// - wst__ValidateTargetType* soap_dup_wst__ValidateTargetType(struct soap*, wst__ValidateTargetType* dst, wst__ValidateTargetType *src) returns deep copy of wst__ValidateTargetType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_wst__ValidateTargetType(wst__ValidateTargetType*) deep deletes wst__ValidateTargetType data members, use only on dst after soap_dup_wst__ValidateTargetType(NULL, wst__ValidateTargetType *dst, wst__ValidateTargetType *src) (use soapcpp2 -Ed)
struct wst__ValidateTargetType
{
/// @todo <any namespace="##other" minOccurs="1" maxOccurs="1">
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element):
///       wsdl2h maps xsd:any to xsd__anyType, use typemap.dat to remap.
};

/// @brief "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":StatusType is a complexType.
///
/// struct wst__StatusType operations:
/// - wst__StatusType* soap_new_wst__StatusType(struct soap*, int num) allocate and default initialize one or more values (array)
/// - soap_default_wst__StatusType(struct soap*, wst__StatusType*) default initialize members
/// - int soap_read_wst__StatusType(struct soap*, wst__StatusType*) deserialize from a source
/// - int soap_write_wst__StatusType(struct soap*, wst__StatusType*) serialize to a sink
/// - wst__StatusType* soap_dup_wst__StatusType(struct soap*, wst__StatusType* dst, wst__StatusType *src) returns deep copy of wst__StatusType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_wst__StatusType(wst__StatusType*) deep deletes wst__StatusType data members, use only on dst after soap_dup_wst__StatusType(NULL, wst__StatusType *dst, wst__StatusType *src) (use soapcpp2 -Ed)
struct wst__StatusType
{
/// Element "Code" of XSD type "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":StatusCodeOpenEnum.
    char*                                Code                           1;	///< Required element.
/// Element "Reason" of XSD type xs:string.
    char*                                Reason                         0;	///< Optional element.
};

/// @brief "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":SignChallengeType is a complexType.
///
/// struct wst__SignChallengeType operations:
/// - wst__SignChallengeType* soap_new_wst__SignChallengeType(struct soap*, int num) allocate and default initialize one or more values (array)
/// - soap_default_wst__SignChallengeType(struct soap*, wst__SignChallengeType*) default initialize members
/// - int soap_read_wst__SignChallengeType(struct soap*, wst__SignChallengeType*) deserialize from a source
/// - int soap_write_wst__SignChallengeType(struct soap*, wst__SignChallengeType*) serialize to a sink
/// - wst__SignChallengeType* soap_dup_wst__SignChallengeType(struct soap*, wst__SignChallengeType* dst, wst__SignChallengeType *src) returns deep copy of wst__SignChallengeType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_wst__SignChallengeType(wst__SignChallengeType*) deep deletes wst__SignChallengeType data members, use only on dst after soap_dup_wst__SignChallengeType(NULL, wst__SignChallengeType *dst, wst__SignChallengeType *src) (use soapcpp2 -Ed)
struct wst__SignChallengeType
{
/// Element reference "http://docs.oasis-open.org/ws-sx/ws-trust/200512/:""http://docs.oasis-open.org/ws-sx/ws-trust/200512/":Challenge.
    char*                                Challenge                      1;	///< Required element.
/// @todo <any namespace="##any" minOccurs="0" maxOccurs="unbounded">
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element):
///       wsdl2h maps xsd:any to xsd__anyType, use typemap.dat to remap.
/// @todo <anyAttribute namespace="##any">.
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
///       Use wsdl2h option -d for xsd__anyAttribute DOM (soap_dom_attribute).
};

/// @brief "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":RequestKETType is a complexType.
///
/// struct wst__RequestKETType operations:
/// - wst__RequestKETType* soap_new_wst__RequestKETType(struct soap*, int num) allocate and default initialize one or more values (array)
/// - soap_default_wst__RequestKETType(struct soap*, wst__RequestKETType*) default initialize members
/// - int soap_read_wst__RequestKETType(struct soap*, wst__RequestKETType*) deserialize from a source
/// - int soap_write_wst__RequestKETType(struct soap*, wst__RequestKETType*) serialize to a sink
/// - wst__RequestKETType* soap_dup_wst__RequestKETType(struct soap*, wst__RequestKETType* dst, wst__RequestKETType *src) returns deep copy of wst__RequestKETType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_wst__RequestKETType(wst__RequestKETType*) deep deletes wst__RequestKETType data members, use only on dst after soap_dup_wst__RequestKETType(NULL, wst__RequestKETType *dst, wst__RequestKETType *src) (use soapcpp2 -Ed)
struct wst__RequestKETType
{
};

/// @brief "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":KeyExchangeTokenType is a complexType.
///
/// struct wst__KeyExchangeTokenType operations:
/// - wst__KeyExchangeTokenType* soap_new_wst__KeyExchangeTokenType(struct soap*, int num) allocate and default initialize one or more values (array)
/// - soap_default_wst__KeyExchangeTokenType(struct soap*, wst__KeyExchangeTokenType*) default initialize members
/// - int soap_read_wst__KeyExchangeTokenType(struct soap*, wst__KeyExchangeTokenType*) deserialize from a source
/// - int soap_write_wst__KeyExchangeTokenType(struct soap*, wst__KeyExchangeTokenType*) serialize to a sink
/// - wst__KeyExchangeTokenType* soap_dup_wst__KeyExchangeTokenType(struct soap*, wst__KeyExchangeTokenType* dst, wst__KeyExchangeTokenType *src) returns deep copy of wst__KeyExchangeTokenType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_wst__KeyExchangeTokenType(wst__KeyExchangeTokenType*) deep deletes wst__KeyExchangeTokenType data members, use only on dst after soap_dup_wst__KeyExchangeTokenType(NULL, wst__KeyExchangeTokenType *dst, wst__KeyExchangeTokenType *src) (use soapcpp2 -Ed)
struct wst__KeyExchangeTokenType
{
/// @todo <any namespace="##any" minOccurs="0" maxOccurs="unbounded">
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element):
///       wsdl2h maps xsd:any to xsd__anyType, use typemap.dat to remap.
};

/// @brief "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":OnBehalfOfType is a complexType.
///
/// struct wst__OnBehalfOfType operations:
/// - wst__OnBehalfOfType* soap_new_wst__OnBehalfOfType(struct soap*, int num) allocate and default initialize one or more values (array)
/// - soap_default_wst__OnBehalfOfType(struct soap*, wst__OnBehalfOfType*) default initialize members
/// - int soap_read_wst__OnBehalfOfType(struct soap*, wst__OnBehalfOfType*) deserialize from a source
/// - int soap_write_wst__OnBehalfOfType(struct soap*, wst__OnBehalfOfType*) serialize to a sink
/// - wst__OnBehalfOfType* soap_dup_wst__OnBehalfOfType(struct soap*, wst__OnBehalfOfType* dst, wst__OnBehalfOfType *src) returns deep copy of wst__OnBehalfOfType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_wst__OnBehalfOfType(wst__OnBehalfOfType*) deep deletes wst__OnBehalfOfType data members, use only on dst after soap_dup_wst__OnBehalfOfType(NULL, wst__OnBehalfOfType *dst, wst__OnBehalfOfType *src) (use soapcpp2 -Ed)
struct wst__OnBehalfOfType
{
/// @todo <any namespace="##any">
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element):
///       wsdl2h maps xsd:any to xsd__anyType, use typemap.dat to remap.
};

/// @brief "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":EncryptionType is a complexType.
///
/// struct wst__EncryptionType operations:
/// - wst__EncryptionType* soap_new_wst__EncryptionType(struct soap*, int num) allocate and default initialize one or more values (array)
/// - soap_default_wst__EncryptionType(struct soap*, wst__EncryptionType*) default initialize members
/// - int soap_read_wst__EncryptionType(struct soap*, wst__EncryptionType*) deserialize from a source
/// - int soap_write_wst__EncryptionType(struct soap*, wst__EncryptionType*) serialize to a sink
/// - wst__EncryptionType* soap_dup_wst__EncryptionType(struct soap*, wst__EncryptionType* dst, wst__EncryptionType *src) returns deep copy of wst__EncryptionType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_wst__EncryptionType(wst__EncryptionType*) deep deletes wst__EncryptionType data members, use only on dst after soap_dup_wst__EncryptionType(NULL, wst__EncryptionType *dst, wst__EncryptionType *src) (use soapcpp2 -Ed)
struct wst__EncryptionType
{
/// @todo <any namespace="##any">
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element):
///       wsdl2h maps xsd:any to xsd__anyType, use typemap.dat to remap.
};

/// @brief "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":ProofEncryptionType is a complexType.
///
/// struct wst__ProofEncryptionType operations:
/// - wst__ProofEncryptionType* soap_new_wst__ProofEncryptionType(struct soap*, int num) allocate and default initialize one or more values (array)
/// - soap_default_wst__ProofEncryptionType(struct soap*, wst__ProofEncryptionType*) default initialize members
/// - int soap_read_wst__ProofEncryptionType(struct soap*, wst__ProofEncryptionType*) deserialize from a source
/// - int soap_write_wst__ProofEncryptionType(struct soap*, wst__ProofEncryptionType*) serialize to a sink
/// - wst__ProofEncryptionType* soap_dup_wst__ProofEncryptionType(struct soap*, wst__ProofEncryptionType* dst, wst__ProofEncryptionType *src) returns deep copy of wst__ProofEncryptionType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_wst__ProofEncryptionType(wst__ProofEncryptionType*) deep deletes wst__ProofEncryptionType data members, use only on dst after soap_dup_wst__ProofEncryptionType(NULL, wst__ProofEncryptionType *dst, wst__ProofEncryptionType *src) (use soapcpp2 -Ed)
struct wst__ProofEncryptionType
{
/// @todo <any namespace="##any">
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element):
///       wsdl2h maps xsd:any to xsd__anyType, use typemap.dat to remap.
};

/// @brief "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":UseKeyType is a complexType.
///
/// struct wst__UseKeyType operations:
/// - wst__UseKeyType* soap_new_wst__UseKeyType(struct soap*, int num) allocate and default initialize one or more values (array)
/// - soap_default_wst__UseKeyType(struct soap*, wst__UseKeyType*) default initialize members
/// - int soap_read_wst__UseKeyType(struct soap*, wst__UseKeyType*) deserialize from a source
/// - int soap_write_wst__UseKeyType(struct soap*, wst__UseKeyType*) serialize to a sink
/// - wst__UseKeyType* soap_dup_wst__UseKeyType(struct soap*, wst__UseKeyType* dst, wst__UseKeyType *src) returns deep copy of wst__UseKeyType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_wst__UseKeyType(wst__UseKeyType*) deep deletes wst__UseKeyType data members, use only on dst after soap_dup_wst__UseKeyType(NULL, wst__UseKeyType *dst, wst__UseKeyType *src) (use soapcpp2 -Ed)
struct wst__UseKeyType
{
/// @todo <any namespace="##any" minOccurs="0">
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element):
///       wsdl2h maps xsd:any to xsd__anyType, use typemap.dat to remap.
/// Attribute "Sig" of XSD type xs:anyURI.
   @char*                                Sig                            0;	///< Optional attribute.
};

/// @brief "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":DelegateToType is a complexType.
///
/// struct wst__DelegateToType operations:
/// - wst__DelegateToType* soap_new_wst__DelegateToType(struct soap*, int num) allocate and default initialize one or more values (array)
/// - soap_default_wst__DelegateToType(struct soap*, wst__DelegateToType*) default initialize members
/// - int soap_read_wst__DelegateToType(struct soap*, wst__DelegateToType*) deserialize from a source
/// - int soap_write_wst__DelegateToType(struct soap*, wst__DelegateToType*) serialize to a sink
/// - wst__DelegateToType* soap_dup_wst__DelegateToType(struct soap*, wst__DelegateToType* dst, wst__DelegateToType *src) returns deep copy of wst__DelegateToType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_wst__DelegateToType(wst__DelegateToType*) deep deletes wst__DelegateToType data members, use only on dst after soap_dup_wst__DelegateToType(NULL, wst__DelegateToType *dst, wst__DelegateToType *src) (use soapcpp2 -Ed)
struct wst__DelegateToType
{
/// @todo <any namespace="##any">
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element):
///       wsdl2h maps xsd:any to xsd__anyType, use typemap.dat to remap.
};

/// @brief "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":ParticipantsType is a complexType.
///
/// struct wst__ParticipantsType operations:
/// - wst__ParticipantsType* soap_new_wst__ParticipantsType(struct soap*, int num) allocate and default initialize one or more values (array)
/// - soap_default_wst__ParticipantsType(struct soap*, wst__ParticipantsType*) default initialize members
/// - int soap_read_wst__ParticipantsType(struct soap*, wst__ParticipantsType*) deserialize from a source
/// - int soap_write_wst__ParticipantsType(struct soap*, wst__ParticipantsType*) serialize to a sink
/// - wst__ParticipantsType* soap_dup_wst__ParticipantsType(struct soap*, wst__ParticipantsType* dst, wst__ParticipantsType *src) returns deep copy of wst__ParticipantsType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_wst__ParticipantsType(wst__ParticipantsType*) deep deletes wst__ParticipantsType data members, use only on dst after soap_dup_wst__ParticipantsType(NULL, wst__ParticipantsType *dst, wst__ParticipantsType *src) (use soapcpp2 -Ed)
struct wst__ParticipantsType
{
/// Element "Primary" of XSD type "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":ParticipantType.
    struct wst__ParticipantType*         Primary                        0;	///< Optional element.
/// Size of array of struct wst__ParticipantType* is 0..unbounded.
   $int                                  __sizeParticipant              0;
/// Array struct wst__ParticipantType* of size 0..unbounded.
    struct wst__ParticipantType*         Participant                    0;
/// @todo <any namespace="##other" minOccurs="0" maxOccurs="unbounded">
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element):
///       wsdl2h maps xsd:any to xsd__anyType, use typemap.dat to remap.
};

/// @brief "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":ParticipantType is a complexType.
///
/// struct wst__ParticipantType operations:
/// - wst__ParticipantType* soap_new_wst__ParticipantType(struct soap*, int num) allocate and default initialize one or more values (array)
/// - soap_default_wst__ParticipantType(struct soap*, wst__ParticipantType*) default initialize members
/// - int soap_read_wst__ParticipantType(struct soap*, wst__ParticipantType*) deserialize from a source
/// - int soap_write_wst__ParticipantType(struct soap*, wst__ParticipantType*) serialize to a sink
/// - wst__ParticipantType* soap_dup_wst__ParticipantType(struct soap*, wst__ParticipantType* dst, wst__ParticipantType *src) returns deep copy of wst__ParticipantType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_wst__ParticipantType(wst__ParticipantType*) deep deletes wst__ParticipantType data members, use only on dst after soap_dup_wst__ParticipantType(NULL, wst__ParticipantType *dst, wst__ParticipantType *src) (use soapcpp2 -Ed)
struct wst__ParticipantType
{
/// @todo <any namespace="##any">
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element):
///       wsdl2h maps xsd:any to xsd__anyType, use typemap.dat to remap.
};

/// @brief "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":BinarySecretType is a complexType with simpleContent.
///
/// struct wst__BinarySecretType operations:
/// - wst__BinarySecretType* soap_new_wst__BinarySecretType(struct soap*, int num) allocate and default initialize one or more values (array)
/// - soap_default_wst__BinarySecretType(struct soap*, wst__BinarySecretType*) default initialize members
/// - int soap_read_wst__BinarySecretType(struct soap*, wst__BinarySecretType*) deserialize from a source
/// - int soap_write_wst__BinarySecretType(struct soap*, wst__BinarySecretType*) serialize to a sink
/// - wst__BinarySecretType* soap_dup_wst__BinarySecretType(struct soap*, wst__BinarySecretType* dst, wst__BinarySecretType *src) returns deep copy of wst__BinarySecretType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_wst__BinarySecretType(wst__BinarySecretType*) deep deletes wst__BinarySecretType data members, use only on dst after soap_dup_wst__BinarySecretType(NULL, wst__BinarySecretType *dst, wst__BinarySecretType *src) (use soapcpp2 -Ed)
struct wst__BinarySecretType
{
/// __item wraps "xs:base64Binary" simpleContent.
    char*                                __item                        ;
/// Attribute "Type" of XSD type "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":BinarySecretTypeOpenEnum.
   @char*                                Type                           0;	///< Optional attribute.
/// @todo <anyAttribute namespace="##other">.
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
///       Use wsdl2h option -d for xsd__anyAttribute DOM (soap_dom_attribute).
};

/// @brief "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":BinaryExchangeType is a complexType with simpleContent.
///
/// struct wst__BinaryExchangeType operations:
/// - wst__BinaryExchangeType* soap_new_wst__BinaryExchangeType(struct soap*, int num) allocate and default initialize one or more values (array)
/// - soap_default_wst__BinaryExchangeType(struct soap*, wst__BinaryExchangeType*) default initialize members
/// - int soap_read_wst__BinaryExchangeType(struct soap*, wst__BinaryExchangeType*) deserialize from a source
/// - int soap_write_wst__BinaryExchangeType(struct soap*, wst__BinaryExchangeType*) serialize to a sink
/// - wst__BinaryExchangeType* soap_dup_wst__BinaryExchangeType(struct soap*, wst__BinaryExchangeType* dst, wst__BinaryExchangeType *src) returns deep copy of wst__BinaryExchangeType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_wst__BinaryExchangeType(wst__BinaryExchangeType*) deep deletes wst__BinaryExchangeType data members, use only on dst after soap_dup_wst__BinaryExchangeType(NULL, wst__BinaryExchangeType *dst, wst__BinaryExchangeType *src) (use soapcpp2 -Ed)
struct wst__BinaryExchangeType
{
/// __item wraps "xs:string" simpleContent.
    char*                                __item                        ;
/// Attribute "ValueType" of XSD type xs:anyURI.
   @char*                                ValueType                      1;	///< Required attribute.
/// Attribute "EncodingType" of XSD type xs:anyURI.
   @char*                                EncodingType                   1;	///< Required attribute.
/// @todo <anyAttribute namespace="##other">.
/// @todo Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
///       Use wsdl2h option -d for xsd__anyAttribute DOM (soap_dom_attribute).
};


/******************************************************************************\
 *                                                                            *
 * Additional Top-Level Elements                                              *
 *   http://docs.oasis-open.org/ws-sx/ws-trust/200512/                        *
 *                                                                            *
\******************************************************************************/

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":RequestSecurityToken of XSD type "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":RequestSecurityTokenType.
typedef struct wst__RequestSecurityTokenType _wst__RequestSecurityToken;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":TokenType of XSD type xs:anyURI.
typedef char*  _wst__TokenType;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":RequestType of XSD type "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":RequestTypeOpenEnum.
typedef char* _wst__RequestType;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":RequestSecurityTokenResponse of XSD type "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":RequestSecurityTokenResponseType.
typedef struct wst__RequestSecurityTokenResponseType _wst__RequestSecurityTokenResponse;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":RequestedSecurityToken of XSD type "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":RequestedSecurityTokenType.
typedef wst__RequestedSecurityTokenType _wst__RequestedSecurityToken;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":BinarySecret of XSD type "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":BinarySecretType.
typedef struct wst__BinarySecretType _wst__BinarySecret;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":Claims of XSD type "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":ClaimsType.
typedef struct wst__ClaimsType _wst__Claims;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":Entropy of XSD type "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":EntropyType.
typedef wst__EntropyType _wst__Entropy;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":Lifetime of XSD type "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":LifetimeType.
typedef struct wst__LifetimeType _wst__Lifetime;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":RequestSecurityTokenCollection of XSD type "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":RequestSecurityTokenCollectionType.
typedef struct wst__RequestSecurityTokenCollectionType _wst__RequestSecurityTokenCollection;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":RequestSecurityTokenResponseCollection of XSD type "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":RequestSecurityTokenResponseCollectionType.
typedef struct wst__RequestSecurityTokenResponseCollectionType _wst__RequestSecurityTokenResponseCollection;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":ComputedKey of XSD type "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":ComputedKeyOpenEnum.
typedef char* _wst__ComputedKey;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":RequestedAttachedReference of XSD type "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":RequestedReferenceType.
typedef struct wst__RequestedReferenceType _wst__RequestedAttachedReference;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":RequestedUnattachedReference of XSD type "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":RequestedReferenceType.
typedef struct wst__RequestedReferenceType _wst__RequestedUnattachedReference;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":RequestedProofToken of XSD type "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":RequestedProofTokenType.
typedef struct wst__RequestedProofTokenType _wst__RequestedProofToken;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":IssuedTokens of XSD type "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":RequestSecurityTokenResponseCollectionType.
typedef struct wst__RequestSecurityTokenResponseCollectionType _wst__IssuedTokens;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":RenewTarget of XSD type "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":RenewTargetType.
typedef struct wst__RenewTargetType _wst__RenewTarget;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":AllowPostdating of XSD type "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":AllowPostdatingType.
typedef struct wst__AllowPostdatingType _wst__AllowPostdating;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":Renewing of XSD type "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":RenewingType.
typedef struct wst__RenewingType _wst__Renewing;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":CancelTarget of XSD type "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":CancelTargetType.
typedef struct wst__CancelTargetType _wst__CancelTarget;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":RequestedTokenCancelled of XSD type "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":RequestedTokenCancelledType.
typedef struct wst__RequestedTokenCancelledType _wst__RequestedTokenCancelled;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":ValidateTarget of XSD type "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":ValidateTargetType.
typedef struct wst__ValidateTargetType _wst__ValidateTarget;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":Status of XSD type "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":StatusType.
typedef struct wst__StatusType _wst__Status;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":SignChallenge of XSD type "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":SignChallengeType.
typedef struct wst__SignChallengeType _wst__SignChallenge;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":SignChallengeResponse of XSD type "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":SignChallengeType.
typedef struct wst__SignChallengeType _wst__SignChallengeResponse;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":Challenge of XSD type xs:string.
typedef char* _wst__Challenge;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":BinaryExchange of XSD type "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":BinaryExchangeType.
typedef struct wst__BinaryExchangeType _wst__BinaryExchange;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":RequestKET of XSD type "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":RequestKETType.
typedef struct wst__RequestKETType _wst__RequestKET;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":KeyExchangeToken of XSD type "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":KeyExchangeTokenType.
typedef struct wst__KeyExchangeTokenType _wst__KeyExchangeToken;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":Authenticator of XSD type "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":AuthenticatorType.
typedef wst__AuthenticatorType _wst__Authenticator;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":CombinedHash of XSD type xs:base64Binary.
typedef char*  _wst__CombinedHash;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":OnBehalfOf of XSD type "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":OnBehalfOfType.
typedef struct wst__OnBehalfOfType _wst__OnBehalfOf;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":Issuer of XSD type "http://www.w3.org/2005/08/addressing":EndpointReferenceType.
typedef wsa5__EndpointReferenceType _wst__Issuer;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":AuthenticationType of XSD type xs:anyURI.
typedef char*  _wst__AuthenticationType;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":KeyType of XSD type "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":KeyTypeOpenEnum.
typedef char* _wst__KeyType;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":KeySize of XSD type xs:unsignedInt.
typedef unsigned int _wst__KeySize;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":SignatureAlgorithm of XSD type xs:anyURI.
typedef char*  _wst__SignatureAlgorithm;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":EncryptionAlgorithm of XSD type xs:anyURI.
typedef char*  _wst__EncryptionAlgorithm;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":CanonicalizationAlgorithm of XSD type xs:anyURI.
typedef char*  _wst__CanonicalizationAlgorithm;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":ComputedKeyAlgorithm of XSD type xs:anyURI.
typedef char*  _wst__ComputedKeyAlgorithm;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":Encryption of XSD type "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":EncryptionType.
typedef struct wst__EncryptionType _wst__Encryption;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":ProofEncryption of XSD type "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":ProofEncryptionType.
typedef struct wst__ProofEncryptionType _wst__ProofEncryption;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":UseKey of XSD type "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":UseKeyType.
typedef struct wst__UseKeyType _wst__UseKey;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":KeyWrapAlgorithm of XSD type xs:anyURI.
typedef char*  _wst__KeyWrapAlgorithm;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":SignWith of XSD type xs:anyURI.
typedef char*  _wst__SignWith;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":EncryptWith of XSD type xs:anyURI.
typedef char*  _wst__EncryptWith;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":DelegateTo of XSD type "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":DelegateToType.
typedef struct wst__DelegateToType _wst__DelegateTo;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":Forwardable of XSD type xs:boolean.
typedef char* _wst__Forwardable;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":Delegatable of XSD type xs:boolean.
typedef char* _wst__Delegatable;

/// @brief Top-level root element "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":Participants of XSD type "http://docs.oasis-open.org/ws-sx/ws-trust/200512/":ParticipantsType.
typedef struct wst__ParticipantsType _wst__Participants;


/******************************************************************************\
 *                                                                            *
 * Additional Top-Level Attributes                                            *
 *   http://docs.oasis-open.org/ws-sx/ws-trust/200512/                        *
 *                                                                            *
\******************************************************************************/


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


@section wst Top-level root elements of schema "http://docs.oasis-open.org/ws-sx/ws-trust/200512/"

  - <wst:RequestSecurityToken> @ref _wst__RequestSecurityToken
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__RequestSecurityToken(struct soap*, _wst__RequestSecurityToken*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__RequestSecurityToken(struct soap*, _wst__RequestSecurityToken*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__RequestSecurityToken(struct soap*, const char *URL, _wst__RequestSecurityToken*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__RequestSecurityToken(struct soap*, const char *URL, _wst__RequestSecurityToken*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__RequestSecurityToken(struct soap*, const char *URL, _wst__RequestSecurityToken*);
    soap_POST_recv__wst__RequestSecurityToken(struct soap*, _wst__RequestSecurityToken*);
    @endcode

  - <wst:TokenType> @ref _wst__TokenType
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__TokenType(struct soap*, _wst__TokenType*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__TokenType(struct soap*, _wst__TokenType*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__TokenType(struct soap*, const char *URL, _wst__TokenType*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__TokenType(struct soap*, const char *URL, _wst__TokenType*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__TokenType(struct soap*, const char *URL, _wst__TokenType*);
    soap_POST_recv__wst__TokenType(struct soap*, _wst__TokenType*);
    @endcode

  - <wst:RequestType> @ref _wst__RequestType
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__RequestType(struct soap*, _wst__RequestType*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__RequestType(struct soap*, _wst__RequestType*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__RequestType(struct soap*, const char *URL, _wst__RequestType*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__RequestType(struct soap*, const char *URL, _wst__RequestType*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__RequestType(struct soap*, const char *URL, _wst__RequestType*);
    soap_POST_recv__wst__RequestType(struct soap*, _wst__RequestType*);
    @endcode

  - <wst:RequestSecurityTokenResponse> @ref _wst__RequestSecurityTokenResponse
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__RequestSecurityTokenResponse(struct soap*, _wst__RequestSecurityTokenResponse*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__RequestSecurityTokenResponse(struct soap*, _wst__RequestSecurityTokenResponse*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__RequestSecurityTokenResponse(struct soap*, const char *URL, _wst__RequestSecurityTokenResponse*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__RequestSecurityTokenResponse(struct soap*, const char *URL, _wst__RequestSecurityTokenResponse*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__RequestSecurityTokenResponse(struct soap*, const char *URL, _wst__RequestSecurityTokenResponse*);
    soap_POST_recv__wst__RequestSecurityTokenResponse(struct soap*, _wst__RequestSecurityTokenResponse*);
    @endcode

  - <wst:RequestedSecurityToken> @ref _wst__RequestedSecurityToken
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__RequestedSecurityToken(struct soap*, _wst__RequestedSecurityToken*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__RequestedSecurityToken(struct soap*, _wst__RequestedSecurityToken*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__RequestedSecurityToken(struct soap*, const char *URL, _wst__RequestedSecurityToken*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__RequestedSecurityToken(struct soap*, const char *URL, _wst__RequestedSecurityToken*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__RequestedSecurityToken(struct soap*, const char *URL, _wst__RequestedSecurityToken*);
    soap_POST_recv__wst__RequestedSecurityToken(struct soap*, _wst__RequestedSecurityToken*);
    @endcode

  - <wst:BinarySecret> @ref _wst__BinarySecret
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__BinarySecret(struct soap*, _wst__BinarySecret*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__BinarySecret(struct soap*, _wst__BinarySecret*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__BinarySecret(struct soap*, const char *URL, _wst__BinarySecret*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__BinarySecret(struct soap*, const char *URL, _wst__BinarySecret*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__BinarySecret(struct soap*, const char *URL, _wst__BinarySecret*);
    soap_POST_recv__wst__BinarySecret(struct soap*, _wst__BinarySecret*);
    @endcode

  - <wst:Claims> @ref _wst__Claims
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__Claims(struct soap*, _wst__Claims*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__Claims(struct soap*, _wst__Claims*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__Claims(struct soap*, const char *URL, _wst__Claims*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__Claims(struct soap*, const char *URL, _wst__Claims*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__Claims(struct soap*, const char *URL, _wst__Claims*);
    soap_POST_recv__wst__Claims(struct soap*, _wst__Claims*);
    @endcode

  - <wst:Entropy> @ref _wst__Entropy
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__Entropy(struct soap*, _wst__Entropy*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__Entropy(struct soap*, _wst__Entropy*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__Entropy(struct soap*, const char *URL, _wst__Entropy*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__Entropy(struct soap*, const char *URL, _wst__Entropy*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__Entropy(struct soap*, const char *URL, _wst__Entropy*);
    soap_POST_recv__wst__Entropy(struct soap*, _wst__Entropy*);
    @endcode

  - <wst:Lifetime> @ref _wst__Lifetime
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__Lifetime(struct soap*, _wst__Lifetime*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__Lifetime(struct soap*, _wst__Lifetime*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__Lifetime(struct soap*, const char *URL, _wst__Lifetime*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__Lifetime(struct soap*, const char *URL, _wst__Lifetime*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__Lifetime(struct soap*, const char *URL, _wst__Lifetime*);
    soap_POST_recv__wst__Lifetime(struct soap*, _wst__Lifetime*);
    @endcode

  - <wst:RequestSecurityTokenCollection> @ref _wst__RequestSecurityTokenCollection
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__RequestSecurityTokenCollection(struct soap*, _wst__RequestSecurityTokenCollection*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__RequestSecurityTokenCollection(struct soap*, _wst__RequestSecurityTokenCollection*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__RequestSecurityTokenCollection(struct soap*, const char *URL, _wst__RequestSecurityTokenCollection*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__RequestSecurityTokenCollection(struct soap*, const char *URL, _wst__RequestSecurityTokenCollection*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__RequestSecurityTokenCollection(struct soap*, const char *URL, _wst__RequestSecurityTokenCollection*);
    soap_POST_recv__wst__RequestSecurityTokenCollection(struct soap*, _wst__RequestSecurityTokenCollection*);
    @endcode

  - <wst:RequestSecurityTokenResponseCollection> @ref _wst__RequestSecurityTokenResponseCollection
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__RequestSecurityTokenResponseCollection(struct soap*, _wst__RequestSecurityTokenResponseCollection*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__RequestSecurityTokenResponseCollection(struct soap*, _wst__RequestSecurityTokenResponseCollection*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__RequestSecurityTokenResponseCollection(struct soap*, const char *URL, _wst__RequestSecurityTokenResponseCollection*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__RequestSecurityTokenResponseCollection(struct soap*, const char *URL, _wst__RequestSecurityTokenResponseCollection*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__RequestSecurityTokenResponseCollection(struct soap*, const char *URL, _wst__RequestSecurityTokenResponseCollection*);
    soap_POST_recv__wst__RequestSecurityTokenResponseCollection(struct soap*, _wst__RequestSecurityTokenResponseCollection*);
    @endcode

  - <wst:ComputedKey> @ref _wst__ComputedKey
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__ComputedKey(struct soap*, _wst__ComputedKey*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__ComputedKey(struct soap*, _wst__ComputedKey*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__ComputedKey(struct soap*, const char *URL, _wst__ComputedKey*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__ComputedKey(struct soap*, const char *URL, _wst__ComputedKey*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__ComputedKey(struct soap*, const char *URL, _wst__ComputedKey*);
    soap_POST_recv__wst__ComputedKey(struct soap*, _wst__ComputedKey*);
    @endcode

  - <wst:RequestedAttachedReference> @ref _wst__RequestedAttachedReference
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__RequestedAttachedReference(struct soap*, _wst__RequestedAttachedReference*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__RequestedAttachedReference(struct soap*, _wst__RequestedAttachedReference*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__RequestedAttachedReference(struct soap*, const char *URL, _wst__RequestedAttachedReference*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__RequestedAttachedReference(struct soap*, const char *URL, _wst__RequestedAttachedReference*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__RequestedAttachedReference(struct soap*, const char *URL, _wst__RequestedAttachedReference*);
    soap_POST_recv__wst__RequestedAttachedReference(struct soap*, _wst__RequestedAttachedReference*);
    @endcode

  - <wst:RequestedUnattachedReference> @ref _wst__RequestedUnattachedReference
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__RequestedUnattachedReference(struct soap*, _wst__RequestedUnattachedReference*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__RequestedUnattachedReference(struct soap*, _wst__RequestedUnattachedReference*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__RequestedUnattachedReference(struct soap*, const char *URL, _wst__RequestedUnattachedReference*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__RequestedUnattachedReference(struct soap*, const char *URL, _wst__RequestedUnattachedReference*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__RequestedUnattachedReference(struct soap*, const char *URL, _wst__RequestedUnattachedReference*);
    soap_POST_recv__wst__RequestedUnattachedReference(struct soap*, _wst__RequestedUnattachedReference*);
    @endcode

  - <wst:RequestedProofToken> @ref _wst__RequestedProofToken
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__RequestedProofToken(struct soap*, _wst__RequestedProofToken*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__RequestedProofToken(struct soap*, _wst__RequestedProofToken*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__RequestedProofToken(struct soap*, const char *URL, _wst__RequestedProofToken*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__RequestedProofToken(struct soap*, const char *URL, _wst__RequestedProofToken*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__RequestedProofToken(struct soap*, const char *URL, _wst__RequestedProofToken*);
    soap_POST_recv__wst__RequestedProofToken(struct soap*, _wst__RequestedProofToken*);
    @endcode

  - <wst:IssuedTokens> @ref _wst__IssuedTokens
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__IssuedTokens(struct soap*, _wst__IssuedTokens*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__IssuedTokens(struct soap*, _wst__IssuedTokens*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__IssuedTokens(struct soap*, const char *URL, _wst__IssuedTokens*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__IssuedTokens(struct soap*, const char *URL, _wst__IssuedTokens*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__IssuedTokens(struct soap*, const char *URL, _wst__IssuedTokens*);
    soap_POST_recv__wst__IssuedTokens(struct soap*, _wst__IssuedTokens*);
    @endcode

  - <wst:RenewTarget> @ref _wst__RenewTarget
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__RenewTarget(struct soap*, _wst__RenewTarget*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__RenewTarget(struct soap*, _wst__RenewTarget*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__RenewTarget(struct soap*, const char *URL, _wst__RenewTarget*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__RenewTarget(struct soap*, const char *URL, _wst__RenewTarget*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__RenewTarget(struct soap*, const char *URL, _wst__RenewTarget*);
    soap_POST_recv__wst__RenewTarget(struct soap*, _wst__RenewTarget*);
    @endcode

  - <wst:AllowPostdating> @ref _wst__AllowPostdating
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__AllowPostdating(struct soap*, _wst__AllowPostdating*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__AllowPostdating(struct soap*, _wst__AllowPostdating*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__AllowPostdating(struct soap*, const char *URL, _wst__AllowPostdating*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__AllowPostdating(struct soap*, const char *URL, _wst__AllowPostdating*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__AllowPostdating(struct soap*, const char *URL, _wst__AllowPostdating*);
    soap_POST_recv__wst__AllowPostdating(struct soap*, _wst__AllowPostdating*);
    @endcode

  - <wst:Renewing> @ref _wst__Renewing
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__Renewing(struct soap*, _wst__Renewing*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__Renewing(struct soap*, _wst__Renewing*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__Renewing(struct soap*, const char *URL, _wst__Renewing*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__Renewing(struct soap*, const char *URL, _wst__Renewing*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__Renewing(struct soap*, const char *URL, _wst__Renewing*);
    soap_POST_recv__wst__Renewing(struct soap*, _wst__Renewing*);
    @endcode

  - <wst:CancelTarget> @ref _wst__CancelTarget
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__CancelTarget(struct soap*, _wst__CancelTarget*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__CancelTarget(struct soap*, _wst__CancelTarget*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__CancelTarget(struct soap*, const char *URL, _wst__CancelTarget*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__CancelTarget(struct soap*, const char *URL, _wst__CancelTarget*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__CancelTarget(struct soap*, const char *URL, _wst__CancelTarget*);
    soap_POST_recv__wst__CancelTarget(struct soap*, _wst__CancelTarget*);
    @endcode

  - <wst:RequestedTokenCancelled> @ref _wst__RequestedTokenCancelled
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__RequestedTokenCancelled(struct soap*, _wst__RequestedTokenCancelled*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__RequestedTokenCancelled(struct soap*, _wst__RequestedTokenCancelled*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__RequestedTokenCancelled(struct soap*, const char *URL, _wst__RequestedTokenCancelled*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__RequestedTokenCancelled(struct soap*, const char *URL, _wst__RequestedTokenCancelled*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__RequestedTokenCancelled(struct soap*, const char *URL, _wst__RequestedTokenCancelled*);
    soap_POST_recv__wst__RequestedTokenCancelled(struct soap*, _wst__RequestedTokenCancelled*);
    @endcode

  - <wst:ValidateTarget> @ref _wst__ValidateTarget
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__ValidateTarget(struct soap*, _wst__ValidateTarget*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__ValidateTarget(struct soap*, _wst__ValidateTarget*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__ValidateTarget(struct soap*, const char *URL, _wst__ValidateTarget*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__ValidateTarget(struct soap*, const char *URL, _wst__ValidateTarget*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__ValidateTarget(struct soap*, const char *URL, _wst__ValidateTarget*);
    soap_POST_recv__wst__ValidateTarget(struct soap*, _wst__ValidateTarget*);
    @endcode

  - <wst:Status> @ref _wst__Status
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__Status(struct soap*, _wst__Status*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__Status(struct soap*, _wst__Status*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__Status(struct soap*, const char *URL, _wst__Status*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__Status(struct soap*, const char *URL, _wst__Status*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__Status(struct soap*, const char *URL, _wst__Status*);
    soap_POST_recv__wst__Status(struct soap*, _wst__Status*);
    @endcode

  - <wst:SignChallenge> @ref _wst__SignChallenge
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__SignChallenge(struct soap*, _wst__SignChallenge*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__SignChallenge(struct soap*, _wst__SignChallenge*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__SignChallenge(struct soap*, const char *URL, _wst__SignChallenge*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__SignChallenge(struct soap*, const char *URL, _wst__SignChallenge*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__SignChallenge(struct soap*, const char *URL, _wst__SignChallenge*);
    soap_POST_recv__wst__SignChallenge(struct soap*, _wst__SignChallenge*);
    @endcode

  - <wst:SignChallengeResponse> @ref _wst__SignChallengeResponse
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__SignChallengeResponse(struct soap*, _wst__SignChallengeResponse*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__SignChallengeResponse(struct soap*, _wst__SignChallengeResponse*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__SignChallengeResponse(struct soap*, const char *URL, _wst__SignChallengeResponse*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__SignChallengeResponse(struct soap*, const char *URL, _wst__SignChallengeResponse*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__SignChallengeResponse(struct soap*, const char *URL, _wst__SignChallengeResponse*);
    soap_POST_recv__wst__SignChallengeResponse(struct soap*, _wst__SignChallengeResponse*);
    @endcode

  - <wst:Challenge> @ref _wst__Challenge
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__Challenge(struct soap*, _wst__Challenge*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__Challenge(struct soap*, _wst__Challenge*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__Challenge(struct soap*, const char *URL, _wst__Challenge*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__Challenge(struct soap*, const char *URL, _wst__Challenge*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__Challenge(struct soap*, const char *URL, _wst__Challenge*);
    soap_POST_recv__wst__Challenge(struct soap*, _wst__Challenge*);
    @endcode

  - <wst:BinaryExchange> @ref _wst__BinaryExchange
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__BinaryExchange(struct soap*, _wst__BinaryExchange*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__BinaryExchange(struct soap*, _wst__BinaryExchange*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__BinaryExchange(struct soap*, const char *URL, _wst__BinaryExchange*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__BinaryExchange(struct soap*, const char *URL, _wst__BinaryExchange*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__BinaryExchange(struct soap*, const char *URL, _wst__BinaryExchange*);
    soap_POST_recv__wst__BinaryExchange(struct soap*, _wst__BinaryExchange*);
    @endcode

  - <wst:RequestKET> @ref _wst__RequestKET
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__RequestKET(struct soap*, _wst__RequestKET*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__RequestKET(struct soap*, _wst__RequestKET*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__RequestKET(struct soap*, const char *URL, _wst__RequestKET*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__RequestKET(struct soap*, const char *URL, _wst__RequestKET*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__RequestKET(struct soap*, const char *URL, _wst__RequestKET*);
    soap_POST_recv__wst__RequestKET(struct soap*, _wst__RequestKET*);
    @endcode

  - <wst:KeyExchangeToken> @ref _wst__KeyExchangeToken
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__KeyExchangeToken(struct soap*, _wst__KeyExchangeToken*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__KeyExchangeToken(struct soap*, _wst__KeyExchangeToken*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__KeyExchangeToken(struct soap*, const char *URL, _wst__KeyExchangeToken*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__KeyExchangeToken(struct soap*, const char *URL, _wst__KeyExchangeToken*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__KeyExchangeToken(struct soap*, const char *URL, _wst__KeyExchangeToken*);
    soap_POST_recv__wst__KeyExchangeToken(struct soap*, _wst__KeyExchangeToken*);
    @endcode

  - <wst:Authenticator> @ref _wst__Authenticator
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__Authenticator(struct soap*, _wst__Authenticator*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__Authenticator(struct soap*, _wst__Authenticator*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__Authenticator(struct soap*, const char *URL, _wst__Authenticator*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__Authenticator(struct soap*, const char *URL, _wst__Authenticator*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__Authenticator(struct soap*, const char *URL, _wst__Authenticator*);
    soap_POST_recv__wst__Authenticator(struct soap*, _wst__Authenticator*);
    @endcode

  - <wst:CombinedHash> @ref _wst__CombinedHash
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__CombinedHash(struct soap*, _wst__CombinedHash*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__CombinedHash(struct soap*, _wst__CombinedHash*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__CombinedHash(struct soap*, const char *URL, _wst__CombinedHash*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__CombinedHash(struct soap*, const char *URL, _wst__CombinedHash*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__CombinedHash(struct soap*, const char *URL, _wst__CombinedHash*);
    soap_POST_recv__wst__CombinedHash(struct soap*, _wst__CombinedHash*);
    @endcode

  - <wst:OnBehalfOf> @ref _wst__OnBehalfOf
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__OnBehalfOf(struct soap*, _wst__OnBehalfOf*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__OnBehalfOf(struct soap*, _wst__OnBehalfOf*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__OnBehalfOf(struct soap*, const char *URL, _wst__OnBehalfOf*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__OnBehalfOf(struct soap*, const char *URL, _wst__OnBehalfOf*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__OnBehalfOf(struct soap*, const char *URL, _wst__OnBehalfOf*);
    soap_POST_recv__wst__OnBehalfOf(struct soap*, _wst__OnBehalfOf*);
    @endcode

  - <wst:Issuer> @ref _wst__Issuer
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__Issuer(struct soap*, _wst__Issuer*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__Issuer(struct soap*, _wst__Issuer*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__Issuer(struct soap*, const char *URL, _wst__Issuer*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__Issuer(struct soap*, const char *URL, _wst__Issuer*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__Issuer(struct soap*, const char *URL, _wst__Issuer*);
    soap_POST_recv__wst__Issuer(struct soap*, _wst__Issuer*);
    @endcode

  - <wst:AuthenticationType> @ref _wst__AuthenticationType
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__AuthenticationType(struct soap*, _wst__AuthenticationType*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__AuthenticationType(struct soap*, _wst__AuthenticationType*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__AuthenticationType(struct soap*, const char *URL, _wst__AuthenticationType*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__AuthenticationType(struct soap*, const char *URL, _wst__AuthenticationType*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__AuthenticationType(struct soap*, const char *URL, _wst__AuthenticationType*);
    soap_POST_recv__wst__AuthenticationType(struct soap*, _wst__AuthenticationType*);
    @endcode

  - <wst:KeyType> @ref _wst__KeyType
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__KeyType(struct soap*, _wst__KeyType*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__KeyType(struct soap*, _wst__KeyType*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__KeyType(struct soap*, const char *URL, _wst__KeyType*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__KeyType(struct soap*, const char *URL, _wst__KeyType*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__KeyType(struct soap*, const char *URL, _wst__KeyType*);
    soap_POST_recv__wst__KeyType(struct soap*, _wst__KeyType*);
    @endcode

  - <wst:KeySize> @ref _wst__KeySize
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__KeySize(struct soap*, _wst__KeySize*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__KeySize(struct soap*, _wst__KeySize*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__KeySize(struct soap*, const char *URL, _wst__KeySize*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__KeySize(struct soap*, const char *URL, _wst__KeySize*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__KeySize(struct soap*, const char *URL, _wst__KeySize*);
    soap_POST_recv__wst__KeySize(struct soap*, _wst__KeySize*);
    @endcode

  - <wst:SignatureAlgorithm> @ref _wst__SignatureAlgorithm
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__SignatureAlgorithm(struct soap*, _wst__SignatureAlgorithm*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__SignatureAlgorithm(struct soap*, _wst__SignatureAlgorithm*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__SignatureAlgorithm(struct soap*, const char *URL, _wst__SignatureAlgorithm*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__SignatureAlgorithm(struct soap*, const char *URL, _wst__SignatureAlgorithm*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__SignatureAlgorithm(struct soap*, const char *URL, _wst__SignatureAlgorithm*);
    soap_POST_recv__wst__SignatureAlgorithm(struct soap*, _wst__SignatureAlgorithm*);
    @endcode

  - <wst:EncryptionAlgorithm> @ref _wst__EncryptionAlgorithm
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__EncryptionAlgorithm(struct soap*, _wst__EncryptionAlgorithm*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__EncryptionAlgorithm(struct soap*, _wst__EncryptionAlgorithm*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__EncryptionAlgorithm(struct soap*, const char *URL, _wst__EncryptionAlgorithm*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__EncryptionAlgorithm(struct soap*, const char *URL, _wst__EncryptionAlgorithm*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__EncryptionAlgorithm(struct soap*, const char *URL, _wst__EncryptionAlgorithm*);
    soap_POST_recv__wst__EncryptionAlgorithm(struct soap*, _wst__EncryptionAlgorithm*);
    @endcode

  - <wst:CanonicalizationAlgorithm> @ref _wst__CanonicalizationAlgorithm
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__CanonicalizationAlgorithm(struct soap*, _wst__CanonicalizationAlgorithm*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__CanonicalizationAlgorithm(struct soap*, _wst__CanonicalizationAlgorithm*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__CanonicalizationAlgorithm(struct soap*, const char *URL, _wst__CanonicalizationAlgorithm*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__CanonicalizationAlgorithm(struct soap*, const char *URL, _wst__CanonicalizationAlgorithm*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__CanonicalizationAlgorithm(struct soap*, const char *URL, _wst__CanonicalizationAlgorithm*);
    soap_POST_recv__wst__CanonicalizationAlgorithm(struct soap*, _wst__CanonicalizationAlgorithm*);
    @endcode

  - <wst:ComputedKeyAlgorithm> @ref _wst__ComputedKeyAlgorithm
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__ComputedKeyAlgorithm(struct soap*, _wst__ComputedKeyAlgorithm*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__ComputedKeyAlgorithm(struct soap*, _wst__ComputedKeyAlgorithm*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__ComputedKeyAlgorithm(struct soap*, const char *URL, _wst__ComputedKeyAlgorithm*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__ComputedKeyAlgorithm(struct soap*, const char *URL, _wst__ComputedKeyAlgorithm*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__ComputedKeyAlgorithm(struct soap*, const char *URL, _wst__ComputedKeyAlgorithm*);
    soap_POST_recv__wst__ComputedKeyAlgorithm(struct soap*, _wst__ComputedKeyAlgorithm*);
    @endcode

  - <wst:Encryption> @ref _wst__Encryption
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__Encryption(struct soap*, _wst__Encryption*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__Encryption(struct soap*, _wst__Encryption*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__Encryption(struct soap*, const char *URL, _wst__Encryption*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__Encryption(struct soap*, const char *URL, _wst__Encryption*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__Encryption(struct soap*, const char *URL, _wst__Encryption*);
    soap_POST_recv__wst__Encryption(struct soap*, _wst__Encryption*);
    @endcode

  - <wst:ProofEncryption> @ref _wst__ProofEncryption
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__ProofEncryption(struct soap*, _wst__ProofEncryption*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__ProofEncryption(struct soap*, _wst__ProofEncryption*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__ProofEncryption(struct soap*, const char *URL, _wst__ProofEncryption*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__ProofEncryption(struct soap*, const char *URL, _wst__ProofEncryption*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__ProofEncryption(struct soap*, const char *URL, _wst__ProofEncryption*);
    soap_POST_recv__wst__ProofEncryption(struct soap*, _wst__ProofEncryption*);
    @endcode

  - <wst:UseKey> @ref _wst__UseKey
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__UseKey(struct soap*, _wst__UseKey*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__UseKey(struct soap*, _wst__UseKey*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__UseKey(struct soap*, const char *URL, _wst__UseKey*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__UseKey(struct soap*, const char *URL, _wst__UseKey*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__UseKey(struct soap*, const char *URL, _wst__UseKey*);
    soap_POST_recv__wst__UseKey(struct soap*, _wst__UseKey*);
    @endcode

  - <wst:KeyWrapAlgorithm> @ref _wst__KeyWrapAlgorithm
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__KeyWrapAlgorithm(struct soap*, _wst__KeyWrapAlgorithm*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__KeyWrapAlgorithm(struct soap*, _wst__KeyWrapAlgorithm*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__KeyWrapAlgorithm(struct soap*, const char *URL, _wst__KeyWrapAlgorithm*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__KeyWrapAlgorithm(struct soap*, const char *URL, _wst__KeyWrapAlgorithm*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__KeyWrapAlgorithm(struct soap*, const char *URL, _wst__KeyWrapAlgorithm*);
    soap_POST_recv__wst__KeyWrapAlgorithm(struct soap*, _wst__KeyWrapAlgorithm*);
    @endcode

  - <wst:SignWith> @ref _wst__SignWith
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__SignWith(struct soap*, _wst__SignWith*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__SignWith(struct soap*, _wst__SignWith*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__SignWith(struct soap*, const char *URL, _wst__SignWith*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__SignWith(struct soap*, const char *URL, _wst__SignWith*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__SignWith(struct soap*, const char *URL, _wst__SignWith*);
    soap_POST_recv__wst__SignWith(struct soap*, _wst__SignWith*);
    @endcode

  - <wst:EncryptWith> @ref _wst__EncryptWith
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__EncryptWith(struct soap*, _wst__EncryptWith*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__EncryptWith(struct soap*, _wst__EncryptWith*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__EncryptWith(struct soap*, const char *URL, _wst__EncryptWith*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__EncryptWith(struct soap*, const char *URL, _wst__EncryptWith*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__EncryptWith(struct soap*, const char *URL, _wst__EncryptWith*);
    soap_POST_recv__wst__EncryptWith(struct soap*, _wst__EncryptWith*);
    @endcode

  - <wst:DelegateTo> @ref _wst__DelegateTo
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__DelegateTo(struct soap*, _wst__DelegateTo*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__DelegateTo(struct soap*, _wst__DelegateTo*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__DelegateTo(struct soap*, const char *URL, _wst__DelegateTo*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__DelegateTo(struct soap*, const char *URL, _wst__DelegateTo*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__DelegateTo(struct soap*, const char *URL, _wst__DelegateTo*);
    soap_POST_recv__wst__DelegateTo(struct soap*, _wst__DelegateTo*);
    @endcode

  - <wst:Forwardable> @ref _wst__Forwardable
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__Forwardable(struct soap*, _wst__Forwardable*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__Forwardable(struct soap*, _wst__Forwardable*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__Forwardable(struct soap*, const char *URL, _wst__Forwardable*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__Forwardable(struct soap*, const char *URL, _wst__Forwardable*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__Forwardable(struct soap*, const char *URL, _wst__Forwardable*);
    soap_POST_recv__wst__Forwardable(struct soap*, _wst__Forwardable*);
    @endcode

  - <wst:Delegatable> @ref _wst__Delegatable
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__Delegatable(struct soap*, _wst__Delegatable*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__Delegatable(struct soap*, _wst__Delegatable*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__Delegatable(struct soap*, const char *URL, _wst__Delegatable*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__Delegatable(struct soap*, const char *URL, _wst__Delegatable*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__Delegatable(struct soap*, const char *URL, _wst__Delegatable*);
    soap_POST_recv__wst__Delegatable(struct soap*, _wst__Delegatable*);
    @endcode

  - <wst:Participants> @ref _wst__Participants
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__wst__Participants(struct soap*, _wst__Participants*);
    // Writer (returns SOAP_OK on success):
    soap_write__wst__Participants(struct soap*, _wst__Participants*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__wst__Participants(struct soap*, const char *URL, _wst__Participants*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__wst__Participants(struct soap*, const char *URL, _wst__Participants*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__wst__Participants(struct soap*, const char *URL, _wst__Participants*);
    soap_POST_recv__wst__Participants(struct soap*, _wst__Participants*);
    @endcode

*/

#import "wstx.h"

/* End of wst.h */
