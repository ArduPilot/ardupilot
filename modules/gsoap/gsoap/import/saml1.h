/*
	saml1.h

	Generated with:
	wsdl2h -cguyx -z9 -o saml1.h -t WS/WS-typemap.dat WS/oasis-sstc-saml-schema-assertion-1.1.xsd

        This file imports:
        - custom/struct_timeval.h
        - xenc.h
        - ds.h
        - c14n.h

	- Removed //gsoapopt
	- Changed //gsoap saml1 schema namespace directive to import directive
	- Replaced #import "ds.h" with #import "xenc.h" (that imports "ds.h")
	- Commented out duplicate saml1__SubjectConfirmation member (due to choice/seq)
	  line 279

*/

/******************************************************************************\
 *                                                                            *
 * Definitions                                                                *
 *   urn:oasis:names:tc:SAML:1.0:assertion                                    *
 *                                                                            *
\******************************************************************************/


/******************************************************************************\
 *                                                                            *
 * Import                                                                     *
 *                                                                            *
\******************************************************************************/

#import "xenc.h"	// xenc = <http://www.w3.org/2001/04/xmlenc#>

/******************************************************************************\
 *                                                                            *
 * Schema Namespaces                                                          *
 *                                                                            *
\******************************************************************************/


/// <PRE><BLOCKQUOTE>
///   Document identifier: oasis-sstc-saml-schema-assertion-1.1
///   Location: http://www.oasis-open.org/committees/documents.php?wg_abbrev=security
///   Revision history:
///   V1.0 (November, 2002):
///   Initial standard schema.
///   V1.1 (September, 2003):
///   * Note that V1.1 of this schema has the same XML namespace as V1.0.
///   Rebased ID content directly on XML Schema types
///   Added DoNotCacheCondition element and DoNotCacheConditionType
/// </BLOCKQUOTE></PRE>
///
#define SOAP_NAMESPACE_OF_saml1	"urn:oasis:names:tc:SAML:1.0:assertion"
//gsoap saml1 schema import:	urn:oasis:names:tc:SAML:1.0:assertion
//gsoap saml1 schema form:	unqualified

/******************************************************************************\
 *                                                                            *
 * Built-in Schema Types and Top-Level Elements and Attributes                *
 *                                                                            *
\******************************************************************************/

/// Built-in type "xs:dateTime".
#import "custom/struct_timeval.h"

// Imported element ""http://www.w3.org/2000/09/xmldsig#":KeyInfo" declared as _ds__KeyInfo.

// Imported element ""http://www.w3.org/2000/09/xmldsig#":Signature" declared as _ds__Signature.


/// @brief Typedef synonym for struct saml1__AssertionType.
typedef struct saml1__AssertionType saml1__AssertionType;

/// @brief Typedef synonym for struct saml1__ConditionsType.
typedef struct saml1__ConditionsType saml1__ConditionsType;

/// @brief Typedef synonym for struct saml1__ConditionAbstractType.
typedef struct saml1__ConditionAbstractType saml1__ConditionAbstractType;

/// @brief Typedef synonym for struct saml1__AudienceRestrictionConditionType.
typedef struct saml1__AudienceRestrictionConditionType saml1__AudienceRestrictionConditionType;

/// @brief Typedef synonym for struct saml1__DoNotCacheConditionType.
typedef struct saml1__DoNotCacheConditionType saml1__DoNotCacheConditionType;

/// @brief Typedef synonym for struct saml1__AdviceType.
typedef struct saml1__AdviceType saml1__AdviceType;

/// @brief Typedef synonym for struct saml1__StatementAbstractType.
typedef struct saml1__StatementAbstractType saml1__StatementAbstractType;

/// @brief Typedef synonym for struct saml1__SubjectStatementAbstractType.
typedef struct saml1__SubjectStatementAbstractType saml1__SubjectStatementAbstractType;

/// @brief Typedef synonym for struct saml1__SubjectType.
typedef struct saml1__SubjectType saml1__SubjectType;

/// @brief Typedef synonym for struct saml1__NameIdentifierType.
typedef struct saml1__NameIdentifierType saml1__NameIdentifierType;

/// @brief Typedef synonym for struct saml1__SubjectConfirmationType.
typedef struct saml1__SubjectConfirmationType saml1__SubjectConfirmationType;

/// @brief Typedef synonym for struct saml1__AuthenticationStatementType.
typedef struct saml1__AuthenticationStatementType saml1__AuthenticationStatementType;

/// @brief Typedef synonym for struct saml1__SubjectLocalityType.
typedef struct saml1__SubjectLocalityType saml1__SubjectLocalityType;

/// @brief Typedef synonym for struct saml1__AuthorityBindingType.
typedef struct saml1__AuthorityBindingType saml1__AuthorityBindingType;

/// @brief Typedef synonym for struct saml1__AuthorizationDecisionStatementType.
typedef struct saml1__AuthorizationDecisionStatementType saml1__AuthorizationDecisionStatementType;

/// @brief Typedef synonym for struct saml1__ActionType.
typedef struct saml1__ActionType saml1__ActionType;

/// @brief Typedef synonym for struct saml1__EvidenceType.
typedef struct saml1__EvidenceType saml1__EvidenceType;

/// @brief Typedef synonym for struct saml1__AttributeStatementType.
typedef struct saml1__AttributeStatementType saml1__AttributeStatementType;

/// @brief Typedef synonym for struct saml1__AttributeDesignatorType.
typedef struct saml1__AttributeDesignatorType saml1__AttributeDesignatorType;

/// @brief Typedef synonym for struct saml1__AttributeType.
typedef struct saml1__AttributeType saml1__AttributeType;


/******************************************************************************\
 *                                                                            *
 * Schema Types and Top-Level Elements and Attributes                         *
 *   urn:oasis:names:tc:SAML:1.0:assertion                                    *
 *                                                                            *
\******************************************************************************/

/// @brief "urn:oasis:names:tc:SAML:1.0:assertion":DecisionType is a simpleType restriction of type xs:string.
///
/// @note The enum values are prefixed with "saml1__DecisionType__" to prevent name clashes: use wsdl2h option -e to omit this prefix or use option -c++11 for scoped enumerations
enum saml1__DecisionType
{
	saml1__DecisionType__Permit,	///< xs:string value="Permit"
	saml1__DecisionType__Deny,	///< xs:string value="Deny"
	saml1__DecisionType__Indeterminate,	///< xs:string value="Indeterminate"
};

/// @brief Typedef synonym for enum saml1__DecisionType.
typedef enum saml1__DecisionType saml1__DecisionType;


/******************************************************************************\
 *                                                                            *
 * Schema Complex Types and Top-Level Elements                                *
 *   urn:oasis:names:tc:SAML:1.0:assertion                                    *
 *                                                                            *
\******************************************************************************/

/// @brief "urn:oasis:names:tc:SAML:1.0:assertion":AssertionType is a complexType.
///
/// @note struct saml1__AssertionType operations:
/// - saml1__AssertionType* soap_new_saml1__AssertionType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml1__AssertionType(struct soap*, saml1__AssertionType*) default initialize members
/// - int soap_read_saml1__AssertionType(struct soap*, saml1__AssertionType*) deserialize from a source
/// - int soap_write_saml1__AssertionType(struct soap*, saml1__AssertionType*) serialize to a sink
/// - saml1__AssertionType* soap_dup_saml1__AssertionType(struct soap*, saml1__AssertionType* dst, saml1__AssertionType *src) returns deep copy of saml1__AssertionType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml1__AssertionType(saml1__AssertionType*) deep deletes saml1__AssertionType data members, use only on dst after soap_dup_saml1__AssertionType(NULL, saml1__AssertionType *dst, saml1__AssertionType *src) (use soapcpp2 -Ed)
struct saml1__AssertionType
{
/// Element reference "urn:oasis:names:tc:SAML:1.0:assertion:"Conditions.
    struct saml1__ConditionsType*        saml1__Conditions              0;	///< Optional element.
/// Element reference "urn:oasis:names:tc:SAML:1.0:assertion:"Advice.
    struct saml1__AdviceType*            saml1__Advice                  0;	///< Optional element.
//  BEGIN CHOICE <xs:choice maxOccurs="unbounded">
  $ int                                  __size_AssertionType           0;
    struct __saml1__union_AssertionType
    {
/// Element reference "urn:oasis:names:tc:SAML:1.0:assertion:"Statement.
    struct saml1__StatementAbstractType*  saml1__Statement              ;	///< Choice of element (one of multiple choices).
/// Element reference "urn:oasis:names:tc:SAML:1.0:assertion:"SubjectStatement.
    struct saml1__SubjectStatementAbstractType*  saml1__SubjectStatement       ;	///< Choice of element (one of multiple choices).
/// Element reference "urn:oasis:names:tc:SAML:1.0:assertion:"AuthenticationStatement.
    struct saml1__AuthenticationStatementType*  saml1__AuthenticationStatement;	///< Choice of element (one of multiple choices).
/// Element reference "urn:oasis:names:tc:SAML:1.0:assertion:"AuthorizationDecisionStatement.
    struct saml1__AuthorizationDecisionStatementType*  saml1__AuthorizationDecisionStatement;	///< Choice of element (one of multiple choices).
/// Element reference "urn:oasis:names:tc:SAML:1.0:assertion:"AttributeStatement.
    struct saml1__AttributeStatementType*  saml1__AttributeStatement     ;	///< Choice of element (one of multiple choices).
    }                                   *__union_AssertionType         ;
//  END OF CHOICE
/// Imported element reference "http://www.w3.org/2000/09/xmldsig#":Signature.
    _ds__Signature*                      ds__Signature                  0;	///< Optional element.
/// Attribute "MajorVersion" of type xs:integer.
  @ char*                                MajorVersion                   1;	///< Required attribute.
/// Attribute "MinorVersion" of type xs:integer.
  @ char*                                MinorVersion                   1;	///< Required attribute.
/// Attribute "AssertionID" of type xs:ID.
  @ char*                                AssertionID                    1;	///< Required attribute.
/// Attribute "Issuer" of type xs:string.
  @ char*                                Issuer                         1;	///< Required attribute.
/// Attribute "IssueInstant" of type xs:dateTime.
  @ xsd__dateTime                        IssueInstant                   1;	///< Required attribute.
/// Member declared in WS/WS-typemap.dat
   @char*                                wsu__Id                        0;	///< Optional attribute.
};

/// @brief "urn:oasis:names:tc:SAML:1.0:assertion":ConditionsType is a complexType.
///
/// @note struct saml1__ConditionsType operations:
/// - saml1__ConditionsType* soap_new_saml1__ConditionsType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml1__ConditionsType(struct soap*, saml1__ConditionsType*) default initialize members
/// - int soap_read_saml1__ConditionsType(struct soap*, saml1__ConditionsType*) deserialize from a source
/// - int soap_write_saml1__ConditionsType(struct soap*, saml1__ConditionsType*) serialize to a sink
/// - saml1__ConditionsType* soap_dup_saml1__ConditionsType(struct soap*, saml1__ConditionsType* dst, saml1__ConditionsType *src) returns deep copy of saml1__ConditionsType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml1__ConditionsType(saml1__ConditionsType*) deep deletes saml1__ConditionsType data members, use only on dst after soap_dup_saml1__ConditionsType(NULL, saml1__ConditionsType *dst, saml1__ConditionsType *src) (use soapcpp2 -Ed)
struct saml1__ConditionsType
{
//  BEGIN CHOICE <xs:choice minOccurs="0" maxOccurs="unbounded">
  $ int                                  __size_ConditionsType          0;
    struct __saml1__union_ConditionsType
    {
/// Element reference "urn:oasis:names:tc:SAML:1.0:assertion:"AudienceRestrictionCondition.
    struct saml1__AudienceRestrictionConditionType*  saml1__AudienceRestrictionCondition;	///< Choice of element (one of multiple choices).
/// Element reference "urn:oasis:names:tc:SAML:1.0:assertion:"DoNotCacheCondition.
    struct saml1__DoNotCacheConditionType*  saml1__DoNotCacheCondition    ;	///< Choice of element (one of multiple choices).
/// Element reference "urn:oasis:names:tc:SAML:1.0:assertion:"Condition.
    struct saml1__ConditionAbstractType*  saml1__Condition              ;	///< Choice of element (one of multiple choices).
    }                                   *__union_ConditionsType        ;
//  END OF CHOICE
/// Attribute "NotBefore" of type xs:dateTime.
  @ xsd__dateTime*                       NotBefore                      0;	///< Optional attribute.
/// Attribute "NotOnOrAfter" of type xs:dateTime.
  @ xsd__dateTime*                       NotOnOrAfter                   0;	///< Optional attribute.
};

/// @brief "urn:oasis:names:tc:SAML:1.0:assertion":ConditionAbstractType is an abstract complexType.
///
/// This type is extended by:
/// - "urn:oasis:names:tc:SAML:1.0:assertion":AudienceRestrictionConditionType as struct saml1__AudienceRestrictionConditionType
/// - "urn:oasis:names:tc:SAML:1.0:assertion":DoNotCacheConditionType as struct saml1__DoNotCacheConditionType
///
/// @note struct saml1__ConditionAbstractType operations:
/// - saml1__ConditionAbstractType* soap_new_saml1__ConditionAbstractType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml1__ConditionAbstractType(struct soap*, saml1__ConditionAbstractType*) default initialize members
/// - int soap_read_saml1__ConditionAbstractType(struct soap*, saml1__ConditionAbstractType*) deserialize from a source
/// - int soap_write_saml1__ConditionAbstractType(struct soap*, saml1__ConditionAbstractType*) serialize to a sink
/// - saml1__ConditionAbstractType* soap_dup_saml1__ConditionAbstractType(struct soap*, saml1__ConditionAbstractType* dst, saml1__ConditionAbstractType *src) returns deep copy of saml1__ConditionAbstractType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml1__ConditionAbstractType(saml1__ConditionAbstractType*) deep deletes saml1__ConditionAbstractType data members, use only on dst after soap_dup_saml1__ConditionAbstractType(NULL, saml1__ConditionAbstractType *dst, saml1__ConditionAbstractType *src) (use soapcpp2 -Ed)
struct saml1__ConditionAbstractType
{
};

/// @brief "urn:oasis:names:tc:SAML:1.0:assertion":AdviceType is a complexType.
///
/// @note struct saml1__AdviceType operations:
/// - saml1__AdviceType* soap_new_saml1__AdviceType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml1__AdviceType(struct soap*, saml1__AdviceType*) default initialize members
/// - int soap_read_saml1__AdviceType(struct soap*, saml1__AdviceType*) deserialize from a source
/// - int soap_write_saml1__AdviceType(struct soap*, saml1__AdviceType*) serialize to a sink
/// - saml1__AdviceType* soap_dup_saml1__AdviceType(struct soap*, saml1__AdviceType* dst, saml1__AdviceType *src) returns deep copy of saml1__AdviceType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml1__AdviceType(saml1__AdviceType*) deep deletes saml1__AdviceType data members, use only on dst after soap_dup_saml1__AdviceType(NULL, saml1__AdviceType *dst, saml1__AdviceType *src) (use soapcpp2 -Ed)
struct saml1__AdviceType
{
//  BEGIN CHOICE <xs:choice minOccurs="0" maxOccurs="unbounded">
  $ int                                  __size_AdviceType              0;
    struct __saml1__union_AdviceType
    {
/// Element reference "urn:oasis:names:tc:SAML:1.0:assertion:"AssertionIDReference.
    char*                                saml1__AssertionIDReference   ;	///< Choice of element (one of multiple choices).
/// Element reference "urn:oasis:names:tc:SAML:1.0:assertion:"Assertion.
    struct saml1__AssertionType*         saml1__Assertion              ;	///< Choice of element (one of multiple choices).
/// <any namespace="##other">
/// @note Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element):
///       wsdl2h maps xsd:any to xsd__anyType, use typemap.dat to remap.
    }                                   *__union_AdviceType            ;
//  END OF CHOICE
};

/// @brief "urn:oasis:names:tc:SAML:1.0:assertion":StatementAbstractType is an abstract complexType.
///
/// This type is extended by:
/// - "urn:oasis:names:tc:SAML:1.0:assertion":SubjectStatementAbstractType as struct saml1__SubjectStatementAbstractType
///
/// @note struct saml1__StatementAbstractType operations:
/// - saml1__StatementAbstractType* soap_new_saml1__StatementAbstractType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml1__StatementAbstractType(struct soap*, saml1__StatementAbstractType*) default initialize members
/// - int soap_read_saml1__StatementAbstractType(struct soap*, saml1__StatementAbstractType*) deserialize from a source
/// - int soap_write_saml1__StatementAbstractType(struct soap*, saml1__StatementAbstractType*) serialize to a sink
/// - saml1__StatementAbstractType* soap_dup_saml1__StatementAbstractType(struct soap*, saml1__StatementAbstractType* dst, saml1__StatementAbstractType *src) returns deep copy of saml1__StatementAbstractType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml1__StatementAbstractType(saml1__StatementAbstractType*) deep deletes saml1__StatementAbstractType data members, use only on dst after soap_dup_saml1__StatementAbstractType(NULL, saml1__StatementAbstractType *dst, saml1__StatementAbstractType *src) (use soapcpp2 -Ed)
struct saml1__StatementAbstractType
{
};

/// @brief "urn:oasis:names:tc:SAML:1.0:assertion":SubjectType is a complexType.
///
/// @note struct saml1__SubjectType operations:
/// - saml1__SubjectType* soap_new_saml1__SubjectType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml1__SubjectType(struct soap*, saml1__SubjectType*) default initialize members
/// - int soap_read_saml1__SubjectType(struct soap*, saml1__SubjectType*) deserialize from a source
/// - int soap_write_saml1__SubjectType(struct soap*, saml1__SubjectType*) serialize to a sink
/// - saml1__SubjectType* soap_dup_saml1__SubjectType(struct soap*, saml1__SubjectType* dst, saml1__SubjectType *src) returns deep copy of saml1__SubjectType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml1__SubjectType(saml1__SubjectType*) deep deletes saml1__SubjectType data members, use only on dst after soap_dup_saml1__SubjectType(NULL, saml1__SubjectType *dst, saml1__SubjectType *src) (use soapcpp2 -Ed)
struct saml1__SubjectType
{
//  BEGIN CHOICE <xs:choice>
//  BEGIN SEQUENCE <xs:sequence>
/// Element reference "urn:oasis:names:tc:SAML:1.0:assertion:"NameIdentifier.
    struct saml1__NameIdentifierType*    saml1__NameIdentifier         ;	///< Choice of element (one of multiple choices).
/// Element reference "urn:oasis:names:tc:SAML:1.0:assertion:"SubjectConfirmation.
    struct saml1__SubjectConfirmationType*  saml1__SubjectConfirmation    ;	///< Choice of optional element (one of multiple choices).
//  END OF SEQUENCE
/// Element reference "urn:oasis:names:tc:SAML:1.0:assertion:"SubjectConfirmation.
    struct saml1__SubjectConfirmationType*  saml1__SubjectConfirmation_   ;	///< Choice of element (one of multiple choices).
//  END OF CHOICE
};

/// @brief "urn:oasis:names:tc:SAML:1.0:assertion":SubjectConfirmationType is a complexType.
///
/// @note struct saml1__SubjectConfirmationType operations:
/// - saml1__SubjectConfirmationType* soap_new_saml1__SubjectConfirmationType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml1__SubjectConfirmationType(struct soap*, saml1__SubjectConfirmationType*) default initialize members
/// - int soap_read_saml1__SubjectConfirmationType(struct soap*, saml1__SubjectConfirmationType*) deserialize from a source
/// - int soap_write_saml1__SubjectConfirmationType(struct soap*, saml1__SubjectConfirmationType*) serialize to a sink
/// - saml1__SubjectConfirmationType* soap_dup_saml1__SubjectConfirmationType(struct soap*, saml1__SubjectConfirmationType* dst, saml1__SubjectConfirmationType *src) returns deep copy of saml1__SubjectConfirmationType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml1__SubjectConfirmationType(saml1__SubjectConfirmationType*) deep deletes saml1__SubjectConfirmationType data members, use only on dst after soap_dup_saml1__SubjectConfirmationType(NULL, saml1__SubjectConfirmationType *dst, saml1__SubjectConfirmationType *src) (use soapcpp2 -Ed)
struct saml1__SubjectConfirmationType
{
/// Size of the dynamic array of values of type char* * is 1..unbounded.
  $ int                                  __sizeConfirmationMethod       1;
/// Array char* * of size 1..unbounded.
    char* *                              saml1__ConfirmationMethod      1;	///< Multiple elements.
/// Element reference "urn:oasis:names:tc:SAML:1.0:assertion:"SubjectConfirmationData.
    _XML                                 saml1__SubjectConfirmationData 0;	///< Optional element.
/// Imported element reference "http://www.w3.org/2000/09/xmldsig#":KeyInfo.
    _ds__KeyInfo*                        ds__KeyInfo                    0;	///< Optional element.
};

/// @brief "urn:oasis:names:tc:SAML:1.0:assertion":SubjectLocalityType is a complexType.
///
/// @note struct saml1__SubjectLocalityType operations:
/// - saml1__SubjectLocalityType* soap_new_saml1__SubjectLocalityType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml1__SubjectLocalityType(struct soap*, saml1__SubjectLocalityType*) default initialize members
/// - int soap_read_saml1__SubjectLocalityType(struct soap*, saml1__SubjectLocalityType*) deserialize from a source
/// - int soap_write_saml1__SubjectLocalityType(struct soap*, saml1__SubjectLocalityType*) serialize to a sink
/// - saml1__SubjectLocalityType* soap_dup_saml1__SubjectLocalityType(struct soap*, saml1__SubjectLocalityType* dst, saml1__SubjectLocalityType *src) returns deep copy of saml1__SubjectLocalityType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml1__SubjectLocalityType(saml1__SubjectLocalityType*) deep deletes saml1__SubjectLocalityType data members, use only on dst after soap_dup_saml1__SubjectLocalityType(NULL, saml1__SubjectLocalityType *dst, saml1__SubjectLocalityType *src) (use soapcpp2 -Ed)
struct saml1__SubjectLocalityType
{
/// Attribute "IPAddress" of type xs:string.
  @ char*                                IPAddress                      0;	///< Optional attribute.
/// Attribute "DNSAddress" of type xs:string.
  @ char*                                DNSAddress                     0;	///< Optional attribute.
};

/// @brief "urn:oasis:names:tc:SAML:1.0:assertion":AuthorityBindingType is a complexType.
///
/// @note struct saml1__AuthorityBindingType operations:
/// - saml1__AuthorityBindingType* soap_new_saml1__AuthorityBindingType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml1__AuthorityBindingType(struct soap*, saml1__AuthorityBindingType*) default initialize members
/// - int soap_read_saml1__AuthorityBindingType(struct soap*, saml1__AuthorityBindingType*) deserialize from a source
/// - int soap_write_saml1__AuthorityBindingType(struct soap*, saml1__AuthorityBindingType*) serialize to a sink
/// - saml1__AuthorityBindingType* soap_dup_saml1__AuthorityBindingType(struct soap*, saml1__AuthorityBindingType* dst, saml1__AuthorityBindingType *src) returns deep copy of saml1__AuthorityBindingType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml1__AuthorityBindingType(saml1__AuthorityBindingType*) deep deletes saml1__AuthorityBindingType data members, use only on dst after soap_dup_saml1__AuthorityBindingType(NULL, saml1__AuthorityBindingType *dst, saml1__AuthorityBindingType *src) (use soapcpp2 -Ed)
struct saml1__AuthorityBindingType
{
/// Attribute "AuthorityKind" of type xs:QName.
  @ _QName                               AuthorityKind                  1;	///< Required attribute.
/// Attribute "Location" of type xs:anyURI.
  @ char*                                Location                       1;	///< Required attribute.
/// Attribute "Binding" of type xs:anyURI.
  @ char*                                Binding                        1;	///< Required attribute.
};

/// @brief "urn:oasis:names:tc:SAML:1.0:assertion":EvidenceType is a complexType.
///
/// @note struct saml1__EvidenceType operations:
/// - saml1__EvidenceType* soap_new_saml1__EvidenceType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml1__EvidenceType(struct soap*, saml1__EvidenceType*) default initialize members
/// - int soap_read_saml1__EvidenceType(struct soap*, saml1__EvidenceType*) deserialize from a source
/// - int soap_write_saml1__EvidenceType(struct soap*, saml1__EvidenceType*) serialize to a sink
/// - saml1__EvidenceType* soap_dup_saml1__EvidenceType(struct soap*, saml1__EvidenceType* dst, saml1__EvidenceType *src) returns deep copy of saml1__EvidenceType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml1__EvidenceType(saml1__EvidenceType*) deep deletes saml1__EvidenceType data members, use only on dst after soap_dup_saml1__EvidenceType(NULL, saml1__EvidenceType *dst, saml1__EvidenceType *src) (use soapcpp2 -Ed)
struct saml1__EvidenceType
{
//  BEGIN CHOICE <xs:choice maxOccurs="unbounded">
  $ int                                  __size_EvidenceType            0;
    struct __saml1__union_EvidenceType
    {
/// Element reference "urn:oasis:names:tc:SAML:1.0:assertion:"AssertionIDReference.
    char*                                saml1__AssertionIDReference   ;	///< Choice of element (one of multiple choices).
/// Element reference "urn:oasis:names:tc:SAML:1.0:assertion:"Assertion.
    struct saml1__AssertionType*         saml1__Assertion              ;	///< Choice of element (one of multiple choices).
    }                                   *__union_EvidenceType          ;
//  END OF CHOICE
};

/// @brief "urn:oasis:names:tc:SAML:1.0:assertion":AttributeDesignatorType is a complexType.
///
/// This type is extended by:
/// - "urn:oasis:names:tc:SAML:1.0:assertion":AttributeType as struct saml1__AttributeType
///
/// @note struct saml1__AttributeDesignatorType operations:
/// - saml1__AttributeDesignatorType* soap_new_saml1__AttributeDesignatorType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml1__AttributeDesignatorType(struct soap*, saml1__AttributeDesignatorType*) default initialize members
/// - int soap_read_saml1__AttributeDesignatorType(struct soap*, saml1__AttributeDesignatorType*) deserialize from a source
/// - int soap_write_saml1__AttributeDesignatorType(struct soap*, saml1__AttributeDesignatorType*) serialize to a sink
/// - saml1__AttributeDesignatorType* soap_dup_saml1__AttributeDesignatorType(struct soap*, saml1__AttributeDesignatorType* dst, saml1__AttributeDesignatorType *src) returns deep copy of saml1__AttributeDesignatorType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml1__AttributeDesignatorType(saml1__AttributeDesignatorType*) deep deletes saml1__AttributeDesignatorType data members, use only on dst after soap_dup_saml1__AttributeDesignatorType(NULL, saml1__AttributeDesignatorType *dst, saml1__AttributeDesignatorType *src) (use soapcpp2 -Ed)
struct saml1__AttributeDesignatorType
{
/// Attribute "AttributeName" of type xs:string.
  @ char*                                AttributeName                  1;	///< Required attribute.
/// Attribute "AttributeNamespace" of type xs:anyURI.
  @ char*                                AttributeNamespace             1;	///< Required attribute.
};

/// @brief "urn:oasis:names:tc:SAML:1.0:assertion":AudienceRestrictionConditionType is a complexType with complexContent extension of type "urn:oasis:names:tc:SAML:1.0:assertion":ConditionAbstractType.
///
/// @note struct saml1__AudienceRestrictionConditionType operations:
/// - saml1__AudienceRestrictionConditionType* soap_new_saml1__AudienceRestrictionConditionType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml1__AudienceRestrictionConditionType(struct soap*, saml1__AudienceRestrictionConditionType*) default initialize members
/// - int soap_read_saml1__AudienceRestrictionConditionType(struct soap*, saml1__AudienceRestrictionConditionType*) deserialize from a source
/// - int soap_write_saml1__AudienceRestrictionConditionType(struct soap*, saml1__AudienceRestrictionConditionType*) serialize to a sink
/// - saml1__AudienceRestrictionConditionType* soap_dup_saml1__AudienceRestrictionConditionType(struct soap*, saml1__AudienceRestrictionConditionType* dst, saml1__AudienceRestrictionConditionType *src) returns deep copy of saml1__AudienceRestrictionConditionType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml1__AudienceRestrictionConditionType(saml1__AudienceRestrictionConditionType*) deep deletes saml1__AudienceRestrictionConditionType data members, use only on dst after soap_dup_saml1__AudienceRestrictionConditionType(NULL, saml1__AudienceRestrictionConditionType *dst, saml1__AudienceRestrictionConditionType *src) (use soapcpp2 -Ed)
struct saml1__AudienceRestrictionConditionType
{
/// INHERITED FROM saml1__ConditionAbstractType:
//  END OF INHERITED FROM saml1__ConditionAbstractType
/// Size of the dynamic array of values of type char* * is 1..unbounded.
  $ int                                  __sizeAudience                 1;
/// Array char* * of size 1..unbounded.
    char* *                              saml1__Audience                1;	///< Multiple elements.
};

/// @brief "urn:oasis:names:tc:SAML:1.0:assertion":DoNotCacheConditionType is a complexType with complexContent extension of type "urn:oasis:names:tc:SAML:1.0:assertion":ConditionAbstractType.
///
/// @note struct saml1__DoNotCacheConditionType operations:
/// - saml1__DoNotCacheConditionType* soap_new_saml1__DoNotCacheConditionType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml1__DoNotCacheConditionType(struct soap*, saml1__DoNotCacheConditionType*) default initialize members
/// - int soap_read_saml1__DoNotCacheConditionType(struct soap*, saml1__DoNotCacheConditionType*) deserialize from a source
/// - int soap_write_saml1__DoNotCacheConditionType(struct soap*, saml1__DoNotCacheConditionType*) serialize to a sink
/// - saml1__DoNotCacheConditionType* soap_dup_saml1__DoNotCacheConditionType(struct soap*, saml1__DoNotCacheConditionType* dst, saml1__DoNotCacheConditionType *src) returns deep copy of saml1__DoNotCacheConditionType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml1__DoNotCacheConditionType(saml1__DoNotCacheConditionType*) deep deletes saml1__DoNotCacheConditionType data members, use only on dst after soap_dup_saml1__DoNotCacheConditionType(NULL, saml1__DoNotCacheConditionType *dst, saml1__DoNotCacheConditionType *src) (use soapcpp2 -Ed)
struct saml1__DoNotCacheConditionType
{
/// INHERITED FROM saml1__ConditionAbstractType:
//  END OF INHERITED FROM saml1__ConditionAbstractType
};

/// @brief "urn:oasis:names:tc:SAML:1.0:assertion":SubjectStatementAbstractType is an abstract complexType with complexContent extension of type "urn:oasis:names:tc:SAML:1.0:assertion":StatementAbstractType.
///
/// @note struct saml1__SubjectStatementAbstractType operations:
/// - saml1__SubjectStatementAbstractType* soap_new_saml1__SubjectStatementAbstractType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml1__SubjectStatementAbstractType(struct soap*, saml1__SubjectStatementAbstractType*) default initialize members
/// - int soap_read_saml1__SubjectStatementAbstractType(struct soap*, saml1__SubjectStatementAbstractType*) deserialize from a source
/// - int soap_write_saml1__SubjectStatementAbstractType(struct soap*, saml1__SubjectStatementAbstractType*) serialize to a sink
/// - saml1__SubjectStatementAbstractType* soap_dup_saml1__SubjectStatementAbstractType(struct soap*, saml1__SubjectStatementAbstractType* dst, saml1__SubjectStatementAbstractType *src) returns deep copy of saml1__SubjectStatementAbstractType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml1__SubjectStatementAbstractType(saml1__SubjectStatementAbstractType*) deep deletes saml1__SubjectStatementAbstractType data members, use only on dst after soap_dup_saml1__SubjectStatementAbstractType(NULL, saml1__SubjectStatementAbstractType *dst, saml1__SubjectStatementAbstractType *src) (use soapcpp2 -Ed)
struct saml1__SubjectStatementAbstractType
{
/// INHERITED FROM saml1__StatementAbstractType:
//  END OF INHERITED FROM saml1__StatementAbstractType
/// Element reference "urn:oasis:names:tc:SAML:1.0:assertion:"Subject.
    struct saml1__SubjectType*           saml1__Subject                 1;	///< Required element.
};

/// @brief "urn:oasis:names:tc:SAML:1.0:assertion":NameIdentifierType is a complexType with simpleContent extension of type xs:string.
///
/// @note struct saml1__NameIdentifierType operations:
/// - saml1__NameIdentifierType* soap_new_saml1__NameIdentifierType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml1__NameIdentifierType(struct soap*, saml1__NameIdentifierType*) default initialize members
/// - int soap_read_saml1__NameIdentifierType(struct soap*, saml1__NameIdentifierType*) deserialize from a source
/// - int soap_write_saml1__NameIdentifierType(struct soap*, saml1__NameIdentifierType*) serialize to a sink
/// - saml1__NameIdentifierType* soap_dup_saml1__NameIdentifierType(struct soap*, saml1__NameIdentifierType* dst, saml1__NameIdentifierType *src) returns deep copy of saml1__NameIdentifierType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml1__NameIdentifierType(saml1__NameIdentifierType*) deep deletes saml1__NameIdentifierType data members, use only on dst after soap_dup_saml1__NameIdentifierType(NULL, saml1__NameIdentifierType *dst, saml1__NameIdentifierType *src) (use soapcpp2 -Ed)
struct saml1__NameIdentifierType
{
/// INHERITED FROM saml1__NameIdentifierType:
/// __item wraps simpleContent of type xs:string.
    char*                                __item                        ;
/// Attribute "NameQualifier" of type xs:string.
  @ char*                                NameQualifier                  0;	///< Optional attribute.
/// Attribute "Format" of type xs:anyURI.
  @ char*                                Format                         0;	///< Optional attribute.
//  END OF INHERITED FROM saml1__NameIdentifierType
};

/// @brief "urn:oasis:names:tc:SAML:1.0:assertion":ActionType is a complexType with simpleContent extension of type xs:string.
///
/// @note struct saml1__ActionType operations:
/// - saml1__ActionType* soap_new_saml1__ActionType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml1__ActionType(struct soap*, saml1__ActionType*) default initialize members
/// - int soap_read_saml1__ActionType(struct soap*, saml1__ActionType*) deserialize from a source
/// - int soap_write_saml1__ActionType(struct soap*, saml1__ActionType*) serialize to a sink
/// - saml1__ActionType* soap_dup_saml1__ActionType(struct soap*, saml1__ActionType* dst, saml1__ActionType *src) returns deep copy of saml1__ActionType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml1__ActionType(saml1__ActionType*) deep deletes saml1__ActionType data members, use only on dst after soap_dup_saml1__ActionType(NULL, saml1__ActionType *dst, saml1__ActionType *src) (use soapcpp2 -Ed)
struct saml1__ActionType
{
/// INHERITED FROM saml1__ActionType:
/// __item wraps simpleContent of type xs:string.
    char*                                __item                        ;
/// Attribute "Namespace" of type xs:anyURI.
  @ char*                                Namespace                      0;	///< Optional attribute.
//  END OF INHERITED FROM saml1__ActionType
};

/// @brief "urn:oasis:names:tc:SAML:1.0:assertion":AttributeType is a complexType with complexContent extension of type "urn:oasis:names:tc:SAML:1.0:assertion":AttributeDesignatorType.
///
/// @note struct saml1__AttributeType operations:
/// - saml1__AttributeType* soap_new_saml1__AttributeType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml1__AttributeType(struct soap*, saml1__AttributeType*) default initialize members
/// - int soap_read_saml1__AttributeType(struct soap*, saml1__AttributeType*) deserialize from a source
/// - int soap_write_saml1__AttributeType(struct soap*, saml1__AttributeType*) serialize to a sink
/// - saml1__AttributeType* soap_dup_saml1__AttributeType(struct soap*, saml1__AttributeType* dst, saml1__AttributeType *src) returns deep copy of saml1__AttributeType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml1__AttributeType(saml1__AttributeType*) deep deletes saml1__AttributeType data members, use only on dst after soap_dup_saml1__AttributeType(NULL, saml1__AttributeType *dst, saml1__AttributeType *src) (use soapcpp2 -Ed)
struct saml1__AttributeType
{
/// INHERITED FROM saml1__AttributeDesignatorType:
/// Attribute "AttributeName" of type xs:string.
  @ char*                                AttributeName                  1;	///< Required attribute.
/// Attribute "AttributeNamespace" of type xs:anyURI.
  @ char*                                AttributeNamespace             1;	///< Required attribute.
//  END OF INHERITED FROM saml1__AttributeDesignatorType
/// Size of the dynamic array of values of type _XML* is 1..unbounded.
  $ int                                  __sizeAttributeValue           1;
/// Array _XML* of size 1..unbounded.
    _XML*                                saml1__AttributeValue          1;	///< Multiple elements.
};

/// @brief "urn:oasis:names:tc:SAML:1.0:assertion":AuthenticationStatementType is a complexType with complexContent extension of type "urn:oasis:names:tc:SAML:1.0:assertion":SubjectStatementAbstractType.
///
/// @note struct saml1__AuthenticationStatementType operations:
/// - saml1__AuthenticationStatementType* soap_new_saml1__AuthenticationStatementType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml1__AuthenticationStatementType(struct soap*, saml1__AuthenticationStatementType*) default initialize members
/// - int soap_read_saml1__AuthenticationStatementType(struct soap*, saml1__AuthenticationStatementType*) deserialize from a source
/// - int soap_write_saml1__AuthenticationStatementType(struct soap*, saml1__AuthenticationStatementType*) serialize to a sink
/// - saml1__AuthenticationStatementType* soap_dup_saml1__AuthenticationStatementType(struct soap*, saml1__AuthenticationStatementType* dst, saml1__AuthenticationStatementType *src) returns deep copy of saml1__AuthenticationStatementType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml1__AuthenticationStatementType(saml1__AuthenticationStatementType*) deep deletes saml1__AuthenticationStatementType data members, use only on dst after soap_dup_saml1__AuthenticationStatementType(NULL, saml1__AuthenticationStatementType *dst, saml1__AuthenticationStatementType *src) (use soapcpp2 -Ed)
struct saml1__AuthenticationStatementType
{
/// INHERITED FROM saml1__StatementAbstractType:
//  END OF INHERITED FROM saml1__StatementAbstractType
/// INHERITED FROM saml1__SubjectStatementAbstractType:
/// Element reference "urn:oasis:names:tc:SAML:1.0:assertion:"Subject.
    struct saml1__SubjectType*           saml1__Subject                 1;	///< Required element.
//  END OF INHERITED FROM saml1__SubjectStatementAbstractType
/// Element reference "urn:oasis:names:tc:SAML:1.0:assertion:"SubjectLocality.
    struct saml1__SubjectLocalityType*   saml1__SubjectLocality         0;	///< Optional element.
/// Size of the dynamic array of values of type struct saml1__AuthorityBindingType* is 0..unbounded.
  $ int                                  __sizeAuthorityBinding         0;
/// Array struct saml1__AuthorityBindingType* of size 0..unbounded.
    struct saml1__AuthorityBindingType*  saml1__AuthorityBinding        0;	///< Multiple elements.
/// Attribute "AuthenticationMethod" of type xs:anyURI.
  @ char*                                AuthenticationMethod           1;	///< Required attribute.
/// Attribute "AuthenticationInstant" of type xs:dateTime.
  @ xsd__dateTime                        AuthenticationInstant          1;	///< Required attribute.
};

/// @brief "urn:oasis:names:tc:SAML:1.0:assertion":AuthorizationDecisionStatementType is a complexType with complexContent extension of type "urn:oasis:names:tc:SAML:1.0:assertion":SubjectStatementAbstractType.
///
/// @note struct saml1__AuthorizationDecisionStatementType operations:
/// - saml1__AuthorizationDecisionStatementType* soap_new_saml1__AuthorizationDecisionStatementType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml1__AuthorizationDecisionStatementType(struct soap*, saml1__AuthorizationDecisionStatementType*) default initialize members
/// - int soap_read_saml1__AuthorizationDecisionStatementType(struct soap*, saml1__AuthorizationDecisionStatementType*) deserialize from a source
/// - int soap_write_saml1__AuthorizationDecisionStatementType(struct soap*, saml1__AuthorizationDecisionStatementType*) serialize to a sink
/// - saml1__AuthorizationDecisionStatementType* soap_dup_saml1__AuthorizationDecisionStatementType(struct soap*, saml1__AuthorizationDecisionStatementType* dst, saml1__AuthorizationDecisionStatementType *src) returns deep copy of saml1__AuthorizationDecisionStatementType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml1__AuthorizationDecisionStatementType(saml1__AuthorizationDecisionStatementType*) deep deletes saml1__AuthorizationDecisionStatementType data members, use only on dst after soap_dup_saml1__AuthorizationDecisionStatementType(NULL, saml1__AuthorizationDecisionStatementType *dst, saml1__AuthorizationDecisionStatementType *src) (use soapcpp2 -Ed)
struct saml1__AuthorizationDecisionStatementType
{
/// INHERITED FROM saml1__StatementAbstractType:
//  END OF INHERITED FROM saml1__StatementAbstractType
/// INHERITED FROM saml1__SubjectStatementAbstractType:
/// Element reference "urn:oasis:names:tc:SAML:1.0:assertion:"Subject.
    struct saml1__SubjectType*           saml1__Subject                 1;	///< Required element.
//  END OF INHERITED FROM saml1__SubjectStatementAbstractType
/// Size of the dynamic array of values of type struct saml1__ActionType* is 1..unbounded.
  $ int                                  __sizeAction                   1;
/// Array struct saml1__ActionType* of size 1..unbounded.
    struct saml1__ActionType*            saml1__Action                  1;	///< Multiple elements.
/// Element reference "urn:oasis:names:tc:SAML:1.0:assertion:"Evidence.
    struct saml1__EvidenceType*          saml1__Evidence                0;	///< Optional element.
/// Attribute "Resource" of type xs:anyURI.
  @ char*                                Resource                       1;	///< Required attribute.
/// Attribute "Decision" of type "urn:oasis:names:tc:SAML:1.0:assertion":DecisionType.
  @ enum saml1__DecisionType             Decision                       1;	///< Required attribute.
};

/// @brief "urn:oasis:names:tc:SAML:1.0:assertion":AttributeStatementType is a complexType with complexContent extension of type "urn:oasis:names:tc:SAML:1.0:assertion":SubjectStatementAbstractType.
///
/// @note struct saml1__AttributeStatementType operations:
/// - saml1__AttributeStatementType* soap_new_saml1__AttributeStatementType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml1__AttributeStatementType(struct soap*, saml1__AttributeStatementType*) default initialize members
/// - int soap_read_saml1__AttributeStatementType(struct soap*, saml1__AttributeStatementType*) deserialize from a source
/// - int soap_write_saml1__AttributeStatementType(struct soap*, saml1__AttributeStatementType*) serialize to a sink
/// - saml1__AttributeStatementType* soap_dup_saml1__AttributeStatementType(struct soap*, saml1__AttributeStatementType* dst, saml1__AttributeStatementType *src) returns deep copy of saml1__AttributeStatementType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml1__AttributeStatementType(saml1__AttributeStatementType*) deep deletes saml1__AttributeStatementType data members, use only on dst after soap_dup_saml1__AttributeStatementType(NULL, saml1__AttributeStatementType *dst, saml1__AttributeStatementType *src) (use soapcpp2 -Ed)
struct saml1__AttributeStatementType
{
/// INHERITED FROM saml1__StatementAbstractType:
//  END OF INHERITED FROM saml1__StatementAbstractType
/// INHERITED FROM saml1__SubjectStatementAbstractType:
/// Element reference "urn:oasis:names:tc:SAML:1.0:assertion:"Subject.
    struct saml1__SubjectType*           saml1__Subject                 1;	///< Required element.
//  END OF INHERITED FROM saml1__SubjectStatementAbstractType
/// Size of the dynamic array of values of type struct saml1__AttributeType* is 1..unbounded.
  $ int                                  __sizeAttribute                1;
/// Array struct saml1__AttributeType* of size 1..unbounded.
    struct saml1__AttributeType*         saml1__Attribute               1;	///< Multiple elements.
};


/******************************************************************************\
 *                                                                            *
 * Additional Top-Level Elements                                              *
 *   urn:oasis:names:tc:SAML:1.0:assertion                                    *
 *                                                                            *
\******************************************************************************/

/// @brief Top-level root element "urn:oasis:names:tc:SAML:1.0:assertion":AssertionIDReference of type xs:NCName.
typedef char*  _saml1__AssertionIDReference;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:1.0:assertion":Assertion of type "urn:oasis:names:tc:SAML:1.0:assertion":AssertionType.
typedef struct saml1__AssertionType _saml1__Assertion;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:1.0:assertion":Conditions of type "urn:oasis:names:tc:SAML:1.0:assertion":ConditionsType.
typedef struct saml1__ConditionsType _saml1__Conditions;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:1.0:assertion":Condition of type "urn:oasis:names:tc:SAML:1.0:assertion":ConditionAbstractType.
typedef struct saml1__ConditionAbstractType _saml1__Condition;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:1.0:assertion":AudienceRestrictionCondition of type "urn:oasis:names:tc:SAML:1.0:assertion":AudienceRestrictionConditionType.
typedef struct saml1__AudienceRestrictionConditionType _saml1__AudienceRestrictionCondition;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:1.0:assertion":Audience of type xs:anyURI.
typedef char*  _saml1__Audience;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:1.0:assertion":DoNotCacheCondition of type "urn:oasis:names:tc:SAML:1.0:assertion":DoNotCacheConditionType.
typedef struct saml1__DoNotCacheConditionType _saml1__DoNotCacheCondition;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:1.0:assertion":Advice of type "urn:oasis:names:tc:SAML:1.0:assertion":AdviceType.
typedef struct saml1__AdviceType _saml1__Advice;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:1.0:assertion":Statement of type "urn:oasis:names:tc:SAML:1.0:assertion":StatementAbstractType.
typedef struct saml1__StatementAbstractType _saml1__Statement;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:1.0:assertion":SubjectStatement of type "urn:oasis:names:tc:SAML:1.0:assertion":SubjectStatementAbstractType.
typedef struct saml1__SubjectStatementAbstractType _saml1__SubjectStatement;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:1.0:assertion":Subject of type "urn:oasis:names:tc:SAML:1.0:assertion":SubjectType.
typedef struct saml1__SubjectType _saml1__Subject;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:1.0:assertion":NameIdentifier of type "urn:oasis:names:tc:SAML:1.0:assertion":NameIdentifierType.
typedef struct saml1__NameIdentifierType _saml1__NameIdentifier;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:1.0:assertion":SubjectConfirmation of type "urn:oasis:names:tc:SAML:1.0:assertion":SubjectConfirmationType.
typedef struct saml1__SubjectConfirmationType _saml1__SubjectConfirmation;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:1.0:assertion":SubjectConfirmationData of type xs:anyType.
typedef _XML _saml1__SubjectConfirmationData;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:1.0:assertion":ConfirmationMethod of type xs:anyURI.
typedef char*  _saml1__ConfirmationMethod;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:1.0:assertion":AuthenticationStatement of type "urn:oasis:names:tc:SAML:1.0:assertion":AuthenticationStatementType.
typedef struct saml1__AuthenticationStatementType _saml1__AuthenticationStatement;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:1.0:assertion":SubjectLocality of type "urn:oasis:names:tc:SAML:1.0:assertion":SubjectLocalityType.
typedef struct saml1__SubjectLocalityType _saml1__SubjectLocality;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:1.0:assertion":AuthorityBinding of type "urn:oasis:names:tc:SAML:1.0:assertion":AuthorityBindingType.
typedef struct saml1__AuthorityBindingType _saml1__AuthorityBinding;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:1.0:assertion":AuthorizationDecisionStatement of type "urn:oasis:names:tc:SAML:1.0:assertion":AuthorizationDecisionStatementType.
typedef struct saml1__AuthorizationDecisionStatementType _saml1__AuthorizationDecisionStatement;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:1.0:assertion":Action of type "urn:oasis:names:tc:SAML:1.0:assertion":ActionType.
typedef struct saml1__ActionType _saml1__Action;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:1.0:assertion":Evidence of type "urn:oasis:names:tc:SAML:1.0:assertion":EvidenceType.
typedef struct saml1__EvidenceType _saml1__Evidence;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:1.0:assertion":AttributeStatement of type "urn:oasis:names:tc:SAML:1.0:assertion":AttributeStatementType.
typedef struct saml1__AttributeStatementType _saml1__AttributeStatement;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:1.0:assertion":AttributeDesignator of type "urn:oasis:names:tc:SAML:1.0:assertion":AttributeDesignatorType.
typedef struct saml1__AttributeDesignatorType _saml1__AttributeDesignator;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:1.0:assertion":Attribute of type "urn:oasis:names:tc:SAML:1.0:assertion":AttributeType.
typedef struct saml1__AttributeType _saml1__Attribute;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:1.0:assertion":AttributeValue of type xs:anyType.
typedef _XML _saml1__AttributeValue;


/******************************************************************************\
 *                                                                            *
 * Additional Top-Level Attributes                                            *
 *   urn:oasis:names:tc:SAML:1.0:assertion                                    *
 *                                                                            *
\******************************************************************************/


/******************************************************************************\
 *                                                                            *
 * XML Data Binding                                                           *
 *                                                                            *
\******************************************************************************/


/** @page page_XMLDataBinding XML Data Binding

SOAP/XML services use data bindings that are contractually bound by WSDLs and
are auto-generated by wsdl2h and soapcpp2 (see Service Bindings). Plain data
bindings are adopted from XML schemas as part of the WSDL types section or when
running wsdl2h on a set of schemas to produce non-SOAP-based XML data bindings.

@note The following readers and writers are C/C++ data type (de)serializers
auto-generated by wsdl2h and soapcpp2. Run soapcpp2 on this file to generate the
(de)serialization code, which is stored in soapC.c[pp]. Include "soapH.h" in
your code to import these data type and function declarations. Only use the
soapcpp2-generated files in your project build. Do not include the wsdl2h-
generated .h file in your code.

@note Data can be read and deserialized from:
  - an int file descriptor, using soap->recvfd = fd
  - a socket, using soap->socket = (int)...
  - a C++ stream (istream, stringstream), using soap->is = (istream*)...
  - a C string, using soap->is = (const char*)...
  - any input, using the soap->frecv() callback

@note Data can be serialized and written to:
  - an int file descriptor, using soap->sendfd = (int)...
  - a socket, using soap->socket = (int)...
  - a C++ stream (ostream, stringstream), using soap->os = (ostream*)...
  - a C string, using soap->os = (const char**)...
  - any output, using the soap->fsend() callback

@note The following options are available for (de)serialization control:
  - soap->encodingStyle = NULL; to remove SOAP 1.1/1.2 encodingStyle
  - soap_set_mode(soap, SOAP_XML_TREE); XML without id-ref (no cycles!)
  - soap_set_mode(soap, SOAP_XML_GRAPH); XML with id-ref (including cycles)
  - soap_set_namespaces(soap, struct Namespace *nsmap); to set xmlns bindings


*/

/**

@section saml1 Top-level root elements of schema "urn:oasis:names:tc:SAML:1.0:assertion"

  - <saml1:AssertionIDReference> @ref _saml1__AssertionIDReference
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml1__AssertionIDReference(struct soap*, _saml1__AssertionIDReference*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml1__AssertionIDReference(struct soap*, _saml1__AssertionIDReference*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml1__AssertionIDReference(struct soap*, const char *URL, _saml1__AssertionIDReference*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml1__AssertionIDReference(struct soap*, const char *URL, _saml1__AssertionIDReference*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml1__AssertionIDReference(struct soap*, const char *URL, _saml1__AssertionIDReference*);
    soap_POST_recv__saml1__AssertionIDReference(struct soap*, _saml1__AssertionIDReference*);
    @endcode

  - <saml1:Assertion> @ref _saml1__Assertion
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml1__Assertion(struct soap*, _saml1__Assertion*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml1__Assertion(struct soap*, _saml1__Assertion*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml1__Assertion(struct soap*, const char *URL, _saml1__Assertion*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml1__Assertion(struct soap*, const char *URL, _saml1__Assertion*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml1__Assertion(struct soap*, const char *URL, _saml1__Assertion*);
    soap_POST_recv__saml1__Assertion(struct soap*, _saml1__Assertion*);
    @endcode

  - <saml1:Conditions> @ref _saml1__Conditions
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml1__Conditions(struct soap*, _saml1__Conditions*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml1__Conditions(struct soap*, _saml1__Conditions*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml1__Conditions(struct soap*, const char *URL, _saml1__Conditions*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml1__Conditions(struct soap*, const char *URL, _saml1__Conditions*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml1__Conditions(struct soap*, const char *URL, _saml1__Conditions*);
    soap_POST_recv__saml1__Conditions(struct soap*, _saml1__Conditions*);
    @endcode

  - <saml1:Condition> @ref _saml1__Condition
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml1__Condition(struct soap*, _saml1__Condition*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml1__Condition(struct soap*, _saml1__Condition*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml1__Condition(struct soap*, const char *URL, _saml1__Condition*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml1__Condition(struct soap*, const char *URL, _saml1__Condition*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml1__Condition(struct soap*, const char *URL, _saml1__Condition*);
    soap_POST_recv__saml1__Condition(struct soap*, _saml1__Condition*);
    @endcode

  - <saml1:AudienceRestrictionCondition> @ref _saml1__AudienceRestrictionCondition
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml1__AudienceRestrictionCondition(struct soap*, _saml1__AudienceRestrictionCondition*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml1__AudienceRestrictionCondition(struct soap*, _saml1__AudienceRestrictionCondition*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml1__AudienceRestrictionCondition(struct soap*, const char *URL, _saml1__AudienceRestrictionCondition*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml1__AudienceRestrictionCondition(struct soap*, const char *URL, _saml1__AudienceRestrictionCondition*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml1__AudienceRestrictionCondition(struct soap*, const char *URL, _saml1__AudienceRestrictionCondition*);
    soap_POST_recv__saml1__AudienceRestrictionCondition(struct soap*, _saml1__AudienceRestrictionCondition*);
    @endcode

  - <saml1:Audience> @ref _saml1__Audience
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml1__Audience(struct soap*, _saml1__Audience*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml1__Audience(struct soap*, _saml1__Audience*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml1__Audience(struct soap*, const char *URL, _saml1__Audience*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml1__Audience(struct soap*, const char *URL, _saml1__Audience*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml1__Audience(struct soap*, const char *URL, _saml1__Audience*);
    soap_POST_recv__saml1__Audience(struct soap*, _saml1__Audience*);
    @endcode

  - <saml1:DoNotCacheCondition> @ref _saml1__DoNotCacheCondition
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml1__DoNotCacheCondition(struct soap*, _saml1__DoNotCacheCondition*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml1__DoNotCacheCondition(struct soap*, _saml1__DoNotCacheCondition*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml1__DoNotCacheCondition(struct soap*, const char *URL, _saml1__DoNotCacheCondition*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml1__DoNotCacheCondition(struct soap*, const char *URL, _saml1__DoNotCacheCondition*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml1__DoNotCacheCondition(struct soap*, const char *URL, _saml1__DoNotCacheCondition*);
    soap_POST_recv__saml1__DoNotCacheCondition(struct soap*, _saml1__DoNotCacheCondition*);
    @endcode

  - <saml1:Advice> @ref _saml1__Advice
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml1__Advice(struct soap*, _saml1__Advice*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml1__Advice(struct soap*, _saml1__Advice*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml1__Advice(struct soap*, const char *URL, _saml1__Advice*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml1__Advice(struct soap*, const char *URL, _saml1__Advice*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml1__Advice(struct soap*, const char *URL, _saml1__Advice*);
    soap_POST_recv__saml1__Advice(struct soap*, _saml1__Advice*);
    @endcode

  - <saml1:Statement> @ref _saml1__Statement
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml1__Statement(struct soap*, _saml1__Statement*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml1__Statement(struct soap*, _saml1__Statement*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml1__Statement(struct soap*, const char *URL, _saml1__Statement*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml1__Statement(struct soap*, const char *URL, _saml1__Statement*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml1__Statement(struct soap*, const char *URL, _saml1__Statement*);
    soap_POST_recv__saml1__Statement(struct soap*, _saml1__Statement*);
    @endcode

  - <saml1:SubjectStatement> @ref _saml1__SubjectStatement
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml1__SubjectStatement(struct soap*, _saml1__SubjectStatement*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml1__SubjectStatement(struct soap*, _saml1__SubjectStatement*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml1__SubjectStatement(struct soap*, const char *URL, _saml1__SubjectStatement*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml1__SubjectStatement(struct soap*, const char *URL, _saml1__SubjectStatement*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml1__SubjectStatement(struct soap*, const char *URL, _saml1__SubjectStatement*);
    soap_POST_recv__saml1__SubjectStatement(struct soap*, _saml1__SubjectStatement*);
    @endcode

  - <saml1:Subject> @ref _saml1__Subject
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml1__Subject(struct soap*, _saml1__Subject*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml1__Subject(struct soap*, _saml1__Subject*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml1__Subject(struct soap*, const char *URL, _saml1__Subject*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml1__Subject(struct soap*, const char *URL, _saml1__Subject*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml1__Subject(struct soap*, const char *URL, _saml1__Subject*);
    soap_POST_recv__saml1__Subject(struct soap*, _saml1__Subject*);
    @endcode

  - <saml1:NameIdentifier> @ref _saml1__NameIdentifier
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml1__NameIdentifier(struct soap*, _saml1__NameIdentifier*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml1__NameIdentifier(struct soap*, _saml1__NameIdentifier*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml1__NameIdentifier(struct soap*, const char *URL, _saml1__NameIdentifier*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml1__NameIdentifier(struct soap*, const char *URL, _saml1__NameIdentifier*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml1__NameIdentifier(struct soap*, const char *URL, _saml1__NameIdentifier*);
    soap_POST_recv__saml1__NameIdentifier(struct soap*, _saml1__NameIdentifier*);
    @endcode

  - <saml1:SubjectConfirmation> @ref _saml1__SubjectConfirmation
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml1__SubjectConfirmation(struct soap*, _saml1__SubjectConfirmation*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml1__SubjectConfirmation(struct soap*, _saml1__SubjectConfirmation*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml1__SubjectConfirmation(struct soap*, const char *URL, _saml1__SubjectConfirmation*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml1__SubjectConfirmation(struct soap*, const char *URL, _saml1__SubjectConfirmation*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml1__SubjectConfirmation(struct soap*, const char *URL, _saml1__SubjectConfirmation*);
    soap_POST_recv__saml1__SubjectConfirmation(struct soap*, _saml1__SubjectConfirmation*);
    @endcode

  - <saml1:SubjectConfirmationData> @ref _saml1__SubjectConfirmationData
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml1__SubjectConfirmationData(struct soap*, _saml1__SubjectConfirmationData*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml1__SubjectConfirmationData(struct soap*, _saml1__SubjectConfirmationData*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml1__SubjectConfirmationData(struct soap*, const char *URL, _saml1__SubjectConfirmationData*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml1__SubjectConfirmationData(struct soap*, const char *URL, _saml1__SubjectConfirmationData*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml1__SubjectConfirmationData(struct soap*, const char *URL, _saml1__SubjectConfirmationData*);
    soap_POST_recv__saml1__SubjectConfirmationData(struct soap*, _saml1__SubjectConfirmationData*);
    @endcode

  - <saml1:ConfirmationMethod> @ref _saml1__ConfirmationMethod
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml1__ConfirmationMethod(struct soap*, _saml1__ConfirmationMethod*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml1__ConfirmationMethod(struct soap*, _saml1__ConfirmationMethod*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml1__ConfirmationMethod(struct soap*, const char *URL, _saml1__ConfirmationMethod*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml1__ConfirmationMethod(struct soap*, const char *URL, _saml1__ConfirmationMethod*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml1__ConfirmationMethod(struct soap*, const char *URL, _saml1__ConfirmationMethod*);
    soap_POST_recv__saml1__ConfirmationMethod(struct soap*, _saml1__ConfirmationMethod*);
    @endcode

  - <saml1:AuthenticationStatement> @ref _saml1__AuthenticationStatement
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml1__AuthenticationStatement(struct soap*, _saml1__AuthenticationStatement*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml1__AuthenticationStatement(struct soap*, _saml1__AuthenticationStatement*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml1__AuthenticationStatement(struct soap*, const char *URL, _saml1__AuthenticationStatement*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml1__AuthenticationStatement(struct soap*, const char *URL, _saml1__AuthenticationStatement*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml1__AuthenticationStatement(struct soap*, const char *URL, _saml1__AuthenticationStatement*);
    soap_POST_recv__saml1__AuthenticationStatement(struct soap*, _saml1__AuthenticationStatement*);
    @endcode

  - <saml1:SubjectLocality> @ref _saml1__SubjectLocality
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml1__SubjectLocality(struct soap*, _saml1__SubjectLocality*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml1__SubjectLocality(struct soap*, _saml1__SubjectLocality*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml1__SubjectLocality(struct soap*, const char *URL, _saml1__SubjectLocality*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml1__SubjectLocality(struct soap*, const char *URL, _saml1__SubjectLocality*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml1__SubjectLocality(struct soap*, const char *URL, _saml1__SubjectLocality*);
    soap_POST_recv__saml1__SubjectLocality(struct soap*, _saml1__SubjectLocality*);
    @endcode

  - <saml1:AuthorityBinding> @ref _saml1__AuthorityBinding
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml1__AuthorityBinding(struct soap*, _saml1__AuthorityBinding*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml1__AuthorityBinding(struct soap*, _saml1__AuthorityBinding*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml1__AuthorityBinding(struct soap*, const char *URL, _saml1__AuthorityBinding*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml1__AuthorityBinding(struct soap*, const char *URL, _saml1__AuthorityBinding*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml1__AuthorityBinding(struct soap*, const char *URL, _saml1__AuthorityBinding*);
    soap_POST_recv__saml1__AuthorityBinding(struct soap*, _saml1__AuthorityBinding*);
    @endcode

  - <saml1:AuthorizationDecisionStatement> @ref _saml1__AuthorizationDecisionStatement
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml1__AuthorizationDecisionStatement(struct soap*, _saml1__AuthorizationDecisionStatement*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml1__AuthorizationDecisionStatement(struct soap*, _saml1__AuthorizationDecisionStatement*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml1__AuthorizationDecisionStatement(struct soap*, const char *URL, _saml1__AuthorizationDecisionStatement*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml1__AuthorizationDecisionStatement(struct soap*, const char *URL, _saml1__AuthorizationDecisionStatement*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml1__AuthorizationDecisionStatement(struct soap*, const char *URL, _saml1__AuthorizationDecisionStatement*);
    soap_POST_recv__saml1__AuthorizationDecisionStatement(struct soap*, _saml1__AuthorizationDecisionStatement*);
    @endcode

  - <saml1:Action> @ref _saml1__Action
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml1__Action(struct soap*, _saml1__Action*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml1__Action(struct soap*, _saml1__Action*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml1__Action(struct soap*, const char *URL, _saml1__Action*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml1__Action(struct soap*, const char *URL, _saml1__Action*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml1__Action(struct soap*, const char *URL, _saml1__Action*);
    soap_POST_recv__saml1__Action(struct soap*, _saml1__Action*);
    @endcode

  - <saml1:Evidence> @ref _saml1__Evidence
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml1__Evidence(struct soap*, _saml1__Evidence*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml1__Evidence(struct soap*, _saml1__Evidence*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml1__Evidence(struct soap*, const char *URL, _saml1__Evidence*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml1__Evidence(struct soap*, const char *URL, _saml1__Evidence*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml1__Evidence(struct soap*, const char *URL, _saml1__Evidence*);
    soap_POST_recv__saml1__Evidence(struct soap*, _saml1__Evidence*);
    @endcode

  - <saml1:AttributeStatement> @ref _saml1__AttributeStatement
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml1__AttributeStatement(struct soap*, _saml1__AttributeStatement*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml1__AttributeStatement(struct soap*, _saml1__AttributeStatement*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml1__AttributeStatement(struct soap*, const char *URL, _saml1__AttributeStatement*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml1__AttributeStatement(struct soap*, const char *URL, _saml1__AttributeStatement*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml1__AttributeStatement(struct soap*, const char *URL, _saml1__AttributeStatement*);
    soap_POST_recv__saml1__AttributeStatement(struct soap*, _saml1__AttributeStatement*);
    @endcode

  - <saml1:AttributeDesignator> @ref _saml1__AttributeDesignator
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml1__AttributeDesignator(struct soap*, _saml1__AttributeDesignator*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml1__AttributeDesignator(struct soap*, _saml1__AttributeDesignator*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml1__AttributeDesignator(struct soap*, const char *URL, _saml1__AttributeDesignator*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml1__AttributeDesignator(struct soap*, const char *URL, _saml1__AttributeDesignator*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml1__AttributeDesignator(struct soap*, const char *URL, _saml1__AttributeDesignator*);
    soap_POST_recv__saml1__AttributeDesignator(struct soap*, _saml1__AttributeDesignator*);
    @endcode

  - <saml1:Attribute> @ref _saml1__Attribute
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml1__Attribute(struct soap*, _saml1__Attribute*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml1__Attribute(struct soap*, _saml1__Attribute*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml1__Attribute(struct soap*, const char *URL, _saml1__Attribute*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml1__Attribute(struct soap*, const char *URL, _saml1__Attribute*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml1__Attribute(struct soap*, const char *URL, _saml1__Attribute*);
    soap_POST_recv__saml1__Attribute(struct soap*, _saml1__Attribute*);
    @endcode

  - <saml1:AttributeValue> @ref _saml1__AttributeValue
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml1__AttributeValue(struct soap*, _saml1__AttributeValue*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml1__AttributeValue(struct soap*, _saml1__AttributeValue*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml1__AttributeValue(struct soap*, const char *URL, _saml1__AttributeValue*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml1__AttributeValue(struct soap*, const char *URL, _saml1__AttributeValue*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml1__AttributeValue(struct soap*, const char *URL, _saml1__AttributeValue*);
    soap_POST_recv__saml1__AttributeValue(struct soap*, _saml1__AttributeValue*);
    @endcode

*/

/* End of saml1.h */
