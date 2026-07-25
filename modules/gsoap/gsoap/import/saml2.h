/*
	saml2.h

	Generated with:
	wsdl2h -cguyx -z9 -o saml2.h -t WS/WS-typemap.dat WS/saml-schema-assertion-2.0.xsd

        This file imports:
        - custom/struct_timeval.h
        - xenc.h
        - ds.h
        - c14n.h

	- Removed //gsoapopt
	- Changed //gsoap saml2 schema namespace directive to import directive
	- Removed #import "ds.h" since already imported in xenc.h
	- Commented out two duplicate member declarations (due to choice/seq)
	  lines 245, 247, 407, 409

*/

/******************************************************************************\
 *                                                                            *
 * Definitions                                                                *
 *   urn:oasis:names:tc:SAML:2.0:assertion                                    *
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
///   Document identifier: saml-schema-assertion-2.0
///   Location: http://docs.oasis-open.org/security/saml/v2.0/
///   Revision history:
///   V1.0 (November, 2002):
///   Initial Standard Schema.
///   V1.1 (September, 2003):
///   Updates within the same V1.0 namespace.
///   V2.0 (March, 2005):
///   New assertion schema for SAML V2.0 namespace.
/// </BLOCKQUOTE></PRE>
///
#define SOAP_NAMESPACE_OF_saml2	"urn:oasis:names:tc:SAML:2.0:assertion"
//gsoap saml2 schema import:	urn:oasis:names:tc:SAML:2.0:assertion
//gsoap saml2 schema form:	unqualified

/******************************************************************************\
 *                                                                            *
 * Built-in Schema Types and Top-Level Elements and Attributes                *
 *                                                                            *
\******************************************************************************/

/// Built-in type "xs:dateTime".
#import "custom/struct_timeval.h"

// Imported element ""http://www.w3.org/2000/09/xmldsig#":KeyInfo" declared as _ds__KeyInfo.

// Imported element ""http://www.w3.org/2000/09/xmldsig#":Signature" declared as _ds__Signature.


/// @brief Typedef synonym for struct saml2__BaseIDAbstractType.
typedef struct saml2__BaseIDAbstractType saml2__BaseIDAbstractType;

/// @brief Typedef synonym for struct saml2__NameIDType.
typedef struct saml2__NameIDType saml2__NameIDType;

/// @brief Typedef synonym for struct saml2__EncryptedElementType.
typedef struct saml2__EncryptedElementType saml2__EncryptedElementType;

/// @brief Typedef synonym for struct saml2__AssertionType.
typedef struct saml2__AssertionType saml2__AssertionType;

/// @brief Typedef synonym for struct saml2__SubjectType.
typedef struct saml2__SubjectType saml2__SubjectType;

/// @brief Typedef synonym for struct saml2__SubjectConfirmationType.
typedef struct saml2__SubjectConfirmationType saml2__SubjectConfirmationType;

/// @brief Typedef synonym for struct saml2__SubjectConfirmationDataType.
typedef struct saml2__SubjectConfirmationDataType saml2__SubjectConfirmationDataType;

/// @brief Typedef synonym for struct saml2__KeyInfoConfirmationDataType.
typedef struct saml2__KeyInfoConfirmationDataType saml2__KeyInfoConfirmationDataType;

/// @brief Typedef synonym for struct saml2__ConditionsType.
typedef struct saml2__ConditionsType saml2__ConditionsType;

/// @brief Typedef synonym for struct saml2__ConditionAbstractType.
typedef struct saml2__ConditionAbstractType saml2__ConditionAbstractType;

/// @brief Typedef synonym for struct saml2__AudienceRestrictionType.
typedef struct saml2__AudienceRestrictionType saml2__AudienceRestrictionType;

/// @brief Typedef synonym for struct saml2__OneTimeUseType.
typedef struct saml2__OneTimeUseType saml2__OneTimeUseType;

/// @brief Typedef synonym for struct saml2__ProxyRestrictionType.
typedef struct saml2__ProxyRestrictionType saml2__ProxyRestrictionType;

/// @brief Typedef synonym for struct saml2__AdviceType.
typedef struct saml2__AdviceType saml2__AdviceType;

/// @brief Typedef synonym for struct saml2__StatementAbstractType.
typedef struct saml2__StatementAbstractType saml2__StatementAbstractType;

/// @brief Typedef synonym for struct saml2__AuthnStatementType.
typedef struct saml2__AuthnStatementType saml2__AuthnStatementType;

/// @brief Typedef synonym for struct saml2__SubjectLocalityType.
typedef struct saml2__SubjectLocalityType saml2__SubjectLocalityType;

/// @brief Typedef synonym for struct saml2__AuthnContextType.
typedef struct saml2__AuthnContextType saml2__AuthnContextType;

/// @brief Typedef synonym for struct saml2__AuthzDecisionStatementType.
typedef struct saml2__AuthzDecisionStatementType saml2__AuthzDecisionStatementType;

/// @brief Typedef synonym for struct saml2__ActionType.
typedef struct saml2__ActionType saml2__ActionType;

/// @brief Typedef synonym for struct saml2__EvidenceType.
typedef struct saml2__EvidenceType saml2__EvidenceType;

/// @brief Typedef synonym for struct saml2__AttributeStatementType.
typedef struct saml2__AttributeStatementType saml2__AttributeStatementType;

/// @brief Typedef synonym for struct saml2__AttributeType.
typedef struct saml2__AttributeType saml2__AttributeType;


/******************************************************************************\
 *                                                                            *
 * Schema Types and Top-Level Elements and Attributes                         *
 *   urn:oasis:names:tc:SAML:2.0:assertion                                    *
 *                                                                            *
\******************************************************************************/

/// @brief "urn:oasis:names:tc:SAML:2.0:assertion":DecisionType is a simpleType restriction of type xs:string.
///
/// @note The enum values are prefixed with "saml2__DecisionType__" to prevent name clashes: use wsdl2h option -e to omit this prefix or use option -c++11 for scoped enumerations
enum saml2__DecisionType
{
	saml2__DecisionType__Permit,	///< xs:string value="Permit"
	saml2__DecisionType__Deny,	///< xs:string value="Deny"
	saml2__DecisionType__Indeterminate,	///< xs:string value="Indeterminate"
};

/// @brief Typedef synonym for enum saml2__DecisionType.
typedef enum saml2__DecisionType saml2__DecisionType;


/******************************************************************************\
 *                                                                            *
 * Schema Complex Types and Top-Level Elements                                *
 *   urn:oasis:names:tc:SAML:2.0:assertion                                    *
 *                                                                            *
\******************************************************************************/

/// @brief "urn:oasis:names:tc:SAML:2.0:assertion":BaseIDAbstractType is an abstract complexType.
///
/// @note struct saml2__BaseIDAbstractType operations:
/// - saml2__BaseIDAbstractType* soap_new_saml2__BaseIDAbstractType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml2__BaseIDAbstractType(struct soap*, saml2__BaseIDAbstractType*) default initialize members
/// - int soap_read_saml2__BaseIDAbstractType(struct soap*, saml2__BaseIDAbstractType*) deserialize from a source
/// - int soap_write_saml2__BaseIDAbstractType(struct soap*, saml2__BaseIDAbstractType*) serialize to a sink
/// - saml2__BaseIDAbstractType* soap_dup_saml2__BaseIDAbstractType(struct soap*, saml2__BaseIDAbstractType* dst, saml2__BaseIDAbstractType *src) returns deep copy of saml2__BaseIDAbstractType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml2__BaseIDAbstractType(saml2__BaseIDAbstractType*) deep deletes saml2__BaseIDAbstractType data members, use only on dst after soap_dup_saml2__BaseIDAbstractType(NULL, saml2__BaseIDAbstractType *dst, saml2__BaseIDAbstractType *src) (use soapcpp2 -Ed)
struct saml2__BaseIDAbstractType
{
//  BEGIN ATTRIBUTEGROUP <xs:attributeGroup ref=""urn:oasis:names:tc:SAML:2.0:assertion":IDNameQualifiers">.
/// Attribute "NameQualifier" of type xs:string.
  @ char*                                NameQualifier                  0;	///< Optional attribute.
/// Attribute "SPNameQualifier" of type xs:string.
  @ char*                                SPNameQualifier                0;	///< Optional attribute.
//  END OF ATTRIBUTEGROUP
};

/// @brief "urn:oasis:names:tc:SAML:2.0:assertion":EncryptedElementType is a complexType.
///
/// @note struct saml2__EncryptedElementType operations:
/// - saml2__EncryptedElementType* soap_new_saml2__EncryptedElementType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml2__EncryptedElementType(struct soap*, saml2__EncryptedElementType*) default initialize members
/// - int soap_read_saml2__EncryptedElementType(struct soap*, saml2__EncryptedElementType*) deserialize from a source
/// - int soap_write_saml2__EncryptedElementType(struct soap*, saml2__EncryptedElementType*) serialize to a sink
/// - saml2__EncryptedElementType* soap_dup_saml2__EncryptedElementType(struct soap*, saml2__EncryptedElementType* dst, saml2__EncryptedElementType *src) returns deep copy of saml2__EncryptedElementType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml2__EncryptedElementType(saml2__EncryptedElementType*) deep deletes saml2__EncryptedElementType data members, use only on dst after soap_dup_saml2__EncryptedElementType(NULL, saml2__EncryptedElementType *dst, saml2__EncryptedElementType *src) (use soapcpp2 -Ed)
struct saml2__EncryptedElementType
{
/// Imported element reference "http://www.w3.org/2001/04/xmlenc#":EncryptedData.
    xenc__EncryptedDataType              xenc__EncryptedData            1;	///< Required element.
/// Imported element reference "http://www.w3.org/2001/04/xmlenc#":EncryptedKey.
/// Size of "http://www.w3.org/2001/04/xmlenc#":EncryptedKey array is 0..unbounded.
  $ int                                  __sizexenc__EncryptedKey       0;
    xenc__EncryptedKeyType*             *xenc__EncryptedKey             0;	///< Multiple elements.
};

/// @brief "urn:oasis:names:tc:SAML:2.0:assertion":AssertionType is a complexType.
///
/// @note struct saml2__AssertionType operations:
/// - saml2__AssertionType* soap_new_saml2__AssertionType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml2__AssertionType(struct soap*, saml2__AssertionType*) default initialize members
/// - int soap_read_saml2__AssertionType(struct soap*, saml2__AssertionType*) deserialize from a source
/// - int soap_write_saml2__AssertionType(struct soap*, saml2__AssertionType*) serialize to a sink
/// - saml2__AssertionType* soap_dup_saml2__AssertionType(struct soap*, saml2__AssertionType* dst, saml2__AssertionType *src) returns deep copy of saml2__AssertionType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml2__AssertionType(saml2__AssertionType*) deep deletes saml2__AssertionType data members, use only on dst after soap_dup_saml2__AssertionType(NULL, saml2__AssertionType *dst, saml2__AssertionType *src) (use soapcpp2 -Ed)
struct saml2__AssertionType
{
/// Element reference "urn:oasis:names:tc:SAML:2.0:assertion:"Issuer.
    struct saml2__NameIDType*            saml2__Issuer                  1;	///< Required element.
/// Imported element reference "http://www.w3.org/2000/09/xmldsig#":Signature.
    _ds__Signature*                      ds__Signature                  0;	///< Optional element.
/// Element reference "urn:oasis:names:tc:SAML:2.0:assertion:"Subject.
    struct saml2__SubjectType*           saml2__Subject                 0;	///< Optional element.
/// Element reference "urn:oasis:names:tc:SAML:2.0:assertion:"Conditions.
    struct saml2__ConditionsType*        saml2__Conditions              0;	///< Optional element.
/// Element reference "urn:oasis:names:tc:SAML:2.0:assertion:"Advice.
    struct saml2__AdviceType*            saml2__Advice                  0;	///< Optional element.
//  BEGIN CHOICE <xs:choice minOccurs="0" maxOccurs="unbounded">
  $ int                                  __size_AssertionType           0;
    struct __saml2__union_AssertionType
    {
/// Element reference "urn:oasis:names:tc:SAML:2.0:assertion:"Statement.
    struct saml2__StatementAbstractType*  saml2__Statement              ;	///< Choice of element (one of multiple choices).
/// Element reference "urn:oasis:names:tc:SAML:2.0:assertion:"AuthnStatement.
    struct saml2__AuthnStatementType*    saml2__AuthnStatement         ;	///< Choice of element (one of multiple choices).
/// Element reference "urn:oasis:names:tc:SAML:2.0:assertion:"AuthzDecisionStatement.
    struct saml2__AuthzDecisionStatementType*  saml2__AuthzDecisionStatement ;	///< Choice of element (one of multiple choices).
/// Element reference "urn:oasis:names:tc:SAML:2.0:assertion:"AttributeStatement.
    struct saml2__AttributeStatementType*  saml2__AttributeStatement     ;	///< Choice of element (one of multiple choices).
    }                                   *__union_AssertionType         ;
//  END OF CHOICE
/// Attribute "Version" of type xs:string.
  @ char*                                Version                        1;	///< Required attribute.
/// Attribute "ID" of type xs:ID.
  @ char*                                ID                             1;	///< Required attribute.
/// Attribute "IssueInstant" of type xs:dateTime.
  @ xsd__dateTime                        IssueInstant                   1;	///< Required attribute.
/// Member declared in WS/WS-typemap.dat
   @char*                                wsu__Id                        0;	///< Optional attribute.
};

/// @brief "urn:oasis:names:tc:SAML:2.0:assertion":SubjectType is a complexType.
///
/// @note struct saml2__SubjectType operations:
/// - saml2__SubjectType* soap_new_saml2__SubjectType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml2__SubjectType(struct soap*, saml2__SubjectType*) default initialize members
/// - int soap_read_saml2__SubjectType(struct soap*, saml2__SubjectType*) deserialize from a source
/// - int soap_write_saml2__SubjectType(struct soap*, saml2__SubjectType*) serialize to a sink
/// - saml2__SubjectType* soap_dup_saml2__SubjectType(struct soap*, saml2__SubjectType* dst, saml2__SubjectType *src) returns deep copy of saml2__SubjectType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml2__SubjectType(saml2__SubjectType*) deep deletes saml2__SubjectType data members, use only on dst after soap_dup_saml2__SubjectType(NULL, saml2__SubjectType *dst, saml2__SubjectType *src) (use soapcpp2 -Ed)
struct saml2__SubjectType
{
//  BEGIN CHOICE <xs:choice>
//  BEGIN SEQUENCE <xs:sequence>
//  BEGIN CHOICE <xs:choice>
/// Element reference "urn:oasis:names:tc:SAML:2.0:assertion:"BaseID.
    struct saml2__BaseIDAbstractType*    saml2__BaseID                 ;	///< Choice of element (one of multiple choices).
/// Element reference "urn:oasis:names:tc:SAML:2.0:assertion:"NameID.
    struct saml2__NameIDType*            saml2__NameID                 ;	///< Choice of element (one of multiple choices).
/// Element reference "urn:oasis:names:tc:SAML:2.0:assertion:"EncryptedID.
    struct saml2__EncryptedElementType*  saml2__EncryptedID            ;	///< Choice of element (one of multiple choices).
//  END OF CHOICE
/// Size of the dynamic array of values of type struct saml2__SubjectConfirmationType* is 0..unbounded.
  $ int                                  __sizeSubjectConfirmation      0;
/// Array struct saml2__SubjectConfirmationType* of size 0..unbounded.
    struct saml2__SubjectConfirmationType*  saml2__SubjectConfirmation    ;	///< Choice of optional element (one of multiple choices).
//  END OF SEQUENCE
/// Size of the dynamic array of values of type struct saml2__SubjectConfirmationType* is 1..unbounded.
  $ int                                  __sizeSubjectConfirmation_     0;
/// Array struct saml2__SubjectConfirmationType* of size 1..unbounded.
    struct saml2__SubjectConfirmationType*  saml2__SubjectConfirmation_   ;	///< Choice of element (one of multiple choices).
//  END OF CHOICE
};

/// @brief "urn:oasis:names:tc:SAML:2.0:assertion":SubjectConfirmationType is a complexType.
///
/// @note struct saml2__SubjectConfirmationType operations:
/// - saml2__SubjectConfirmationType* soap_new_saml2__SubjectConfirmationType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml2__SubjectConfirmationType(struct soap*, saml2__SubjectConfirmationType*) default initialize members
/// - int soap_read_saml2__SubjectConfirmationType(struct soap*, saml2__SubjectConfirmationType*) deserialize from a source
/// - int soap_write_saml2__SubjectConfirmationType(struct soap*, saml2__SubjectConfirmationType*) serialize to a sink
/// - saml2__SubjectConfirmationType* soap_dup_saml2__SubjectConfirmationType(struct soap*, saml2__SubjectConfirmationType* dst, saml2__SubjectConfirmationType *src) returns deep copy of saml2__SubjectConfirmationType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml2__SubjectConfirmationType(saml2__SubjectConfirmationType*) deep deletes saml2__SubjectConfirmationType data members, use only on dst after soap_dup_saml2__SubjectConfirmationType(NULL, saml2__SubjectConfirmationType *dst, saml2__SubjectConfirmationType *src) (use soapcpp2 -Ed)
struct saml2__SubjectConfirmationType
{
//  BEGIN CHOICE <xs:choice minOccurs="0">
/// Element reference "urn:oasis:names:tc:SAML:2.0:assertion:"BaseID.
    struct saml2__BaseIDAbstractType*    saml2__BaseID                 ;	///< Choice of element (one of multiple choices).
/// Element reference "urn:oasis:names:tc:SAML:2.0:assertion:"NameID.
    struct saml2__NameIDType*            saml2__NameID                 ;	///< Choice of element (one of multiple choices).
/// Element reference "urn:oasis:names:tc:SAML:2.0:assertion:"EncryptedID.
    struct saml2__EncryptedElementType*  saml2__EncryptedID            ;	///< Choice of element (one of multiple choices).
//  END OF CHOICE
/// Element reference "urn:oasis:names:tc:SAML:2.0:assertion:"SubjectConfirmationData.
    struct saml2__SubjectConfirmationDataType*  saml2__SubjectConfirmationData 0;	///< Optional element.
/// Attribute "Method" of type xs:anyURI.
  @ char*                                Method                         1;	///< Required attribute.
};

/// @brief "urn:oasis:names:tc:SAML:2.0:assertion":ConditionsType is a complexType.
///
/// @note struct saml2__ConditionsType operations:
/// - saml2__ConditionsType* soap_new_saml2__ConditionsType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml2__ConditionsType(struct soap*, saml2__ConditionsType*) default initialize members
/// - int soap_read_saml2__ConditionsType(struct soap*, saml2__ConditionsType*) deserialize from a source
/// - int soap_write_saml2__ConditionsType(struct soap*, saml2__ConditionsType*) serialize to a sink
/// - saml2__ConditionsType* soap_dup_saml2__ConditionsType(struct soap*, saml2__ConditionsType* dst, saml2__ConditionsType *src) returns deep copy of saml2__ConditionsType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml2__ConditionsType(saml2__ConditionsType*) deep deletes saml2__ConditionsType data members, use only on dst after soap_dup_saml2__ConditionsType(NULL, saml2__ConditionsType *dst, saml2__ConditionsType *src) (use soapcpp2 -Ed)
struct saml2__ConditionsType
{
//  BEGIN CHOICE <xs:choice minOccurs="0" maxOccurs="unbounded">
  $ int                                  __size_ConditionsType          0;
    struct __saml2__union_ConditionsType
    {
/// Element reference "urn:oasis:names:tc:SAML:2.0:assertion:"Condition.
    struct saml2__ConditionAbstractType*  saml2__Condition              ;	///< Choice of element (one of multiple choices).
/// Element reference "urn:oasis:names:tc:SAML:2.0:assertion:"AudienceRestriction.
    struct saml2__AudienceRestrictionType*  saml2__AudienceRestriction    ;	///< Choice of element (one of multiple choices).
/// Element reference "urn:oasis:names:tc:SAML:2.0:assertion:"OneTimeUse.
    struct saml2__OneTimeUseType*        saml2__OneTimeUse             ;	///< Choice of element (one of multiple choices).
/// Element reference "urn:oasis:names:tc:SAML:2.0:assertion:"ProxyRestriction.
    struct saml2__ProxyRestrictionType*  saml2__ProxyRestriction       ;	///< Choice of element (one of multiple choices).
    }                                   *__union_ConditionsType        ;
//  END OF CHOICE
/// Attribute "NotBefore" of type xs:dateTime.
  @ xsd__dateTime*                       NotBefore                      0;	///< Optional attribute.
/// Attribute "NotOnOrAfter" of type xs:dateTime.
  @ xsd__dateTime*                       NotOnOrAfter                   0;	///< Optional attribute.
};

/// @brief "urn:oasis:names:tc:SAML:2.0:assertion":ConditionAbstractType is an abstract complexType.
///
/// This type is extended by:
/// - "urn:oasis:names:tc:SAML:2.0:assertion":AudienceRestrictionType as struct saml2__AudienceRestrictionType
/// - "urn:oasis:names:tc:SAML:2.0:assertion":OneTimeUseType as struct saml2__OneTimeUseType
/// - "urn:oasis:names:tc:SAML:2.0:assertion":ProxyRestrictionType as struct saml2__ProxyRestrictionType
///
/// @note struct saml2__ConditionAbstractType operations:
/// - saml2__ConditionAbstractType* soap_new_saml2__ConditionAbstractType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml2__ConditionAbstractType(struct soap*, saml2__ConditionAbstractType*) default initialize members
/// - int soap_read_saml2__ConditionAbstractType(struct soap*, saml2__ConditionAbstractType*) deserialize from a source
/// - int soap_write_saml2__ConditionAbstractType(struct soap*, saml2__ConditionAbstractType*) serialize to a sink
/// - saml2__ConditionAbstractType* soap_dup_saml2__ConditionAbstractType(struct soap*, saml2__ConditionAbstractType* dst, saml2__ConditionAbstractType *src) returns deep copy of saml2__ConditionAbstractType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml2__ConditionAbstractType(saml2__ConditionAbstractType*) deep deletes saml2__ConditionAbstractType data members, use only on dst after soap_dup_saml2__ConditionAbstractType(NULL, saml2__ConditionAbstractType *dst, saml2__ConditionAbstractType *src) (use soapcpp2 -Ed)
struct saml2__ConditionAbstractType
{
};

/// @brief "urn:oasis:names:tc:SAML:2.0:assertion":AdviceType is a complexType.
///
/// @note struct saml2__AdviceType operations:
/// - saml2__AdviceType* soap_new_saml2__AdviceType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml2__AdviceType(struct soap*, saml2__AdviceType*) default initialize members
/// - int soap_read_saml2__AdviceType(struct soap*, saml2__AdviceType*) deserialize from a source
/// - int soap_write_saml2__AdviceType(struct soap*, saml2__AdviceType*) serialize to a sink
/// - saml2__AdviceType* soap_dup_saml2__AdviceType(struct soap*, saml2__AdviceType* dst, saml2__AdviceType *src) returns deep copy of saml2__AdviceType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml2__AdviceType(saml2__AdviceType*) deep deletes saml2__AdviceType data members, use only on dst after soap_dup_saml2__AdviceType(NULL, saml2__AdviceType *dst, saml2__AdviceType *src) (use soapcpp2 -Ed)
struct saml2__AdviceType
{
//  BEGIN CHOICE <xs:choice minOccurs="0" maxOccurs="unbounded">
  $ int                                  __size_AdviceType              0;
    struct __saml2__union_AdviceType
    {
/// Element reference "urn:oasis:names:tc:SAML:2.0:assertion:"AssertionIDRef.
    char*                                saml2__AssertionIDRef         ;	///< Choice of element (one of multiple choices).
/// Element reference "urn:oasis:names:tc:SAML:2.0:assertion:"AssertionURIRef.
    char*                                saml2__AssertionURIRef        ;	///< Choice of element (one of multiple choices).
/// Element reference "urn:oasis:names:tc:SAML:2.0:assertion:"Assertion.
    struct saml2__AssertionType*         saml2__Assertion              ;	///< Choice of element (one of multiple choices).
/// Element reference "urn:oasis:names:tc:SAML:2.0:assertion:"EncryptedAssertion.
    struct saml2__EncryptedElementType*  saml2__EncryptedAssertion     ;	///< Choice of element (one of multiple choices).
/// <any namespace="##other">
/// @note Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element):
///       wsdl2h maps xsd:any to xsd__anyType, use typemap.dat to remap.
    }                                   *__union_AdviceType            ;
//  END OF CHOICE
};

/// @brief "urn:oasis:names:tc:SAML:2.0:assertion":StatementAbstractType is an abstract complexType.
///
/// This type is extended by:
/// - "urn:oasis:names:tc:SAML:2.0:assertion":AuthnStatementType as struct saml2__AuthnStatementType
/// - "urn:oasis:names:tc:SAML:2.0:assertion":AuthzDecisionStatementType as struct saml2__AuthzDecisionStatementType
/// - "urn:oasis:names:tc:SAML:2.0:assertion":AttributeStatementType as struct saml2__AttributeStatementType
///
/// @note struct saml2__StatementAbstractType operations:
/// - saml2__StatementAbstractType* soap_new_saml2__StatementAbstractType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml2__StatementAbstractType(struct soap*, saml2__StatementAbstractType*) default initialize members
/// - int soap_read_saml2__StatementAbstractType(struct soap*, saml2__StatementAbstractType*) deserialize from a source
/// - int soap_write_saml2__StatementAbstractType(struct soap*, saml2__StatementAbstractType*) serialize to a sink
/// - saml2__StatementAbstractType* soap_dup_saml2__StatementAbstractType(struct soap*, saml2__StatementAbstractType* dst, saml2__StatementAbstractType *src) returns deep copy of saml2__StatementAbstractType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml2__StatementAbstractType(saml2__StatementAbstractType*) deep deletes saml2__StatementAbstractType data members, use only on dst after soap_dup_saml2__StatementAbstractType(NULL, saml2__StatementAbstractType *dst, saml2__StatementAbstractType *src) (use soapcpp2 -Ed)
struct saml2__StatementAbstractType
{
};

/// @brief "urn:oasis:names:tc:SAML:2.0:assertion":SubjectLocalityType is a complexType.
///
/// @note struct saml2__SubjectLocalityType operations:
/// - saml2__SubjectLocalityType* soap_new_saml2__SubjectLocalityType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml2__SubjectLocalityType(struct soap*, saml2__SubjectLocalityType*) default initialize members
/// - int soap_read_saml2__SubjectLocalityType(struct soap*, saml2__SubjectLocalityType*) deserialize from a source
/// - int soap_write_saml2__SubjectLocalityType(struct soap*, saml2__SubjectLocalityType*) serialize to a sink
/// - saml2__SubjectLocalityType* soap_dup_saml2__SubjectLocalityType(struct soap*, saml2__SubjectLocalityType* dst, saml2__SubjectLocalityType *src) returns deep copy of saml2__SubjectLocalityType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml2__SubjectLocalityType(saml2__SubjectLocalityType*) deep deletes saml2__SubjectLocalityType data members, use only on dst after soap_dup_saml2__SubjectLocalityType(NULL, saml2__SubjectLocalityType *dst, saml2__SubjectLocalityType *src) (use soapcpp2 -Ed)
struct saml2__SubjectLocalityType
{
/// Attribute "Address" of type xs:string.
  @ char*                                Address                        0;	///< Optional attribute.
/// Attribute "DNSName" of type xs:string.
  @ char*                                DNSName                        0;	///< Optional attribute.
};

/// @brief "urn:oasis:names:tc:SAML:2.0:assertion":AuthnContextType is a complexType.
///
/// @note struct saml2__AuthnContextType operations:
/// - saml2__AuthnContextType* soap_new_saml2__AuthnContextType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml2__AuthnContextType(struct soap*, saml2__AuthnContextType*) default initialize members
/// - int soap_read_saml2__AuthnContextType(struct soap*, saml2__AuthnContextType*) deserialize from a source
/// - int soap_write_saml2__AuthnContextType(struct soap*, saml2__AuthnContextType*) serialize to a sink
/// - saml2__AuthnContextType* soap_dup_saml2__AuthnContextType(struct soap*, saml2__AuthnContextType* dst, saml2__AuthnContextType *src) returns deep copy of saml2__AuthnContextType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml2__AuthnContextType(saml2__AuthnContextType*) deep deletes saml2__AuthnContextType data members, use only on dst after soap_dup_saml2__AuthnContextType(NULL, saml2__AuthnContextType *dst, saml2__AuthnContextType *src) (use soapcpp2 -Ed)
struct saml2__AuthnContextType
{
//  BEGIN CHOICE <xs:choice>
//  BEGIN SEQUENCE <xs:sequence>
/// Element reference "urn:oasis:names:tc:SAML:2.0:assertion:"AuthnContextClassRef.
    char*                                saml2__AuthnContextClassRef   ;	///< Choice of element (one of multiple choices).
//  BEGIN CHOICE <xs:choice minOccurs="0">
/// Element reference "urn:oasis:names:tc:SAML:2.0:assertion:"AuthnContextDecl.
    _XML                                 saml2__AuthnContextDecl       ;	///< Choice of element (one of multiple choices).
/// Element reference "urn:oasis:names:tc:SAML:2.0:assertion:"AuthnContextDeclRef.
    char*                                saml2__AuthnContextDeclRef    ;	///< Choice of element (one of multiple choices).
//  END OF CHOICE
//  END OF SEQUENCE
//  BEGIN CHOICE <xs:choice>
/// Element reference "urn:oasis:names:tc:SAML:2.0:assertion:"AuthnContextDecl.
    _XML                                 saml2__AuthnContextDecl_      ;	///< Choice of element (one of multiple choices).
/// Element reference "urn:oasis:names:tc:SAML:2.0:assertion:"AuthnContextDeclRef.
    char*                                saml2__AuthnContextDeclRef_   ;	///< Choice of element (one of multiple choices).
//  END OF CHOICE
//  END OF CHOICE
/// Size of the dynamic array of values of type char* * is 0..unbounded.
  $ int                                  __sizeAuthenticatingAuthority  0;
/// Array char* * of size 0..unbounded.
    char* *                              saml2__AuthenticatingAuthority 0;	///< Multiple elements.
};

/// @brief "urn:oasis:names:tc:SAML:2.0:assertion":EvidenceType is a complexType.
///
/// @note struct saml2__EvidenceType operations:
/// - saml2__EvidenceType* soap_new_saml2__EvidenceType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml2__EvidenceType(struct soap*, saml2__EvidenceType*) default initialize members
/// - int soap_read_saml2__EvidenceType(struct soap*, saml2__EvidenceType*) deserialize from a source
/// - int soap_write_saml2__EvidenceType(struct soap*, saml2__EvidenceType*) serialize to a sink
/// - saml2__EvidenceType* soap_dup_saml2__EvidenceType(struct soap*, saml2__EvidenceType* dst, saml2__EvidenceType *src) returns deep copy of saml2__EvidenceType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml2__EvidenceType(saml2__EvidenceType*) deep deletes saml2__EvidenceType data members, use only on dst after soap_dup_saml2__EvidenceType(NULL, saml2__EvidenceType *dst, saml2__EvidenceType *src) (use soapcpp2 -Ed)
struct saml2__EvidenceType
{
//  BEGIN CHOICE <xs:choice maxOccurs="unbounded">
  $ int                                  __size_EvidenceType            0;
    struct __saml2__union_EvidenceType
    {
/// Element reference "urn:oasis:names:tc:SAML:2.0:assertion:"AssertionIDRef.
    char*                                saml2__AssertionIDRef         ;	///< Choice of element (one of multiple choices).
/// Element reference "urn:oasis:names:tc:SAML:2.0:assertion:"AssertionURIRef.
    char*                                saml2__AssertionURIRef        ;	///< Choice of element (one of multiple choices).
/// Element reference "urn:oasis:names:tc:SAML:2.0:assertion:"Assertion.
    struct saml2__AssertionType*         saml2__Assertion              ;	///< Choice of element (one of multiple choices).
/// Element reference "urn:oasis:names:tc:SAML:2.0:assertion:"EncryptedAssertion.
    struct saml2__EncryptedElementType*  saml2__EncryptedAssertion     ;	///< Choice of element (one of multiple choices).
    }                                   *__union_EvidenceType          ;
//  END OF CHOICE
};

/// @brief "urn:oasis:names:tc:SAML:2.0:assertion":AttributeType is a complexType.
///
/// @note struct saml2__AttributeType operations:
/// - saml2__AttributeType* soap_new_saml2__AttributeType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml2__AttributeType(struct soap*, saml2__AttributeType*) default initialize members
/// - int soap_read_saml2__AttributeType(struct soap*, saml2__AttributeType*) deserialize from a source
/// - int soap_write_saml2__AttributeType(struct soap*, saml2__AttributeType*) serialize to a sink
/// - saml2__AttributeType* soap_dup_saml2__AttributeType(struct soap*, saml2__AttributeType* dst, saml2__AttributeType *src) returns deep copy of saml2__AttributeType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml2__AttributeType(saml2__AttributeType*) deep deletes saml2__AttributeType data members, use only on dst after soap_dup_saml2__AttributeType(NULL, saml2__AttributeType *dst, saml2__AttributeType *src) (use soapcpp2 -Ed)
struct saml2__AttributeType
{
/// Size of the dynamic array of values of type _XML* is 0..unbounded.
  $ int                                  __sizeAttributeValue           0;
/// Array _XML* of size 0..unbounded.
    _XML*                                saml2__AttributeValue          0;	///< Multiple elements.
/// Attribute "Name" of type xs:string.
  @ char*                                Name                           1;	///< Required attribute.
/// Attribute "NameFormat" of type xs:anyURI.
  @ char*                                NameFormat                     0;	///< Optional attribute.
/// Attribute "FriendlyName" of type xs:string.
  @ char*                                FriendlyName                   0;	///< Optional attribute.
/// <anyAttribute namespace="##other">.
/// @note Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
///       Use wsdl2h option -d for xsd__anyAttribute DOM (soap_dom_attribute).
};

/// @brief "urn:oasis:names:tc:SAML:2.0:assertion":NameIDType is a complexType with simpleContent extension of type xs:string.
///
/// @note struct saml2__NameIDType operations:
/// - saml2__NameIDType* soap_new_saml2__NameIDType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml2__NameIDType(struct soap*, saml2__NameIDType*) default initialize members
/// - int soap_read_saml2__NameIDType(struct soap*, saml2__NameIDType*) deserialize from a source
/// - int soap_write_saml2__NameIDType(struct soap*, saml2__NameIDType*) serialize to a sink
/// - saml2__NameIDType* soap_dup_saml2__NameIDType(struct soap*, saml2__NameIDType* dst, saml2__NameIDType *src) returns deep copy of saml2__NameIDType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml2__NameIDType(saml2__NameIDType*) deep deletes saml2__NameIDType data members, use only on dst after soap_dup_saml2__NameIDType(NULL, saml2__NameIDType *dst, saml2__NameIDType *src) (use soapcpp2 -Ed)
struct saml2__NameIDType
{
/// INHERITED FROM saml2__NameIDType:
/// __item wraps simpleContent of type xs:string.
    char*                                __item                        ;
/// Attribute "Format" of type xs:anyURI.
  @ char*                                Format                         0;	///< Optional attribute.
/// Attribute "SPProvidedID" of type xs:string.
  @ char*                                SPProvidedID                   0;	///< Optional attribute.
//  BEGIN ATTRIBUTEGROUP <xs:attributeGroup ref=""urn:oasis:names:tc:SAML:2.0:assertion":IDNameQualifiers">.
/// Attribute "NameQualifier" of type xs:string.
  @ char*                                NameQualifier                  0;	///< Optional attribute.
/// Attribute "SPNameQualifier" of type xs:string.
  @ char*                                SPNameQualifier                0;	///< Optional attribute.
//  END OF ATTRIBUTEGROUP
//  END OF INHERITED FROM saml2__NameIDType
};

/// @brief "urn:oasis:names:tc:SAML:2.0:assertion":SubjectConfirmationDataType is a complexType with complexContent restriction of type xs:anyType.
///
/// This type is restricted by:
/// - "urn:oasis:names:tc:SAML:2.0:assertion":KeyInfoConfirmationDataType as struct saml2__KeyInfoConfirmationDataType
///
/// @note struct saml2__SubjectConfirmationDataType operations:
/// - saml2__SubjectConfirmationDataType* soap_new_saml2__SubjectConfirmationDataType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml2__SubjectConfirmationDataType(struct soap*, saml2__SubjectConfirmationDataType*) default initialize members
/// - int soap_read_saml2__SubjectConfirmationDataType(struct soap*, saml2__SubjectConfirmationDataType*) deserialize from a source
/// - int soap_write_saml2__SubjectConfirmationDataType(struct soap*, saml2__SubjectConfirmationDataType*) serialize to a sink
/// - saml2__SubjectConfirmationDataType* soap_dup_saml2__SubjectConfirmationDataType(struct soap*, saml2__SubjectConfirmationDataType* dst, saml2__SubjectConfirmationDataType *src) returns deep copy of saml2__SubjectConfirmationDataType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml2__SubjectConfirmationDataType(saml2__SubjectConfirmationDataType*) deep deletes saml2__SubjectConfirmationDataType data members, use only on dst after soap_dup_saml2__SubjectConfirmationDataType(NULL, saml2__SubjectConfirmationDataType *dst, saml2__SubjectConfirmationDataType *src) (use soapcpp2 -Ed)
struct saml2__SubjectConfirmationDataType
{
/// <any namespace="##any" minOccurs="0" maxOccurs="unbounded">
/// @note Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element):
///       wsdl2h maps xsd:any to xsd__anyType, use typemap.dat to remap.
/// Attribute "NotBefore" of type xs:dateTime.
  @ xsd__dateTime*                       NotBefore                      0;	///< Optional attribute.
/// Attribute "NotOnOrAfter" of type xs:dateTime.
  @ xsd__dateTime*                       NotOnOrAfter                   0;	///< Optional attribute.
/// Attribute "Recipient" of type xs:anyURI.
  @ char*                                Recipient                      0;	///< Optional attribute.
/// Attribute "InResponseTo" of type xs:NCName.
  @ char*                                InResponseTo                   0;	///< Optional attribute.
/// Attribute "Address" of type xs:string.
  @ char*                                Address                        0;	///< Optional attribute.
/// <anyAttribute namespace="##other">.
/// @note Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
///       Use wsdl2h option -d for xsd__anyAttribute DOM (soap_dom_attribute).
/// Mixed content.
/// @note Mixed content is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -d for DOM (soap_dom_element) to store mixed content.
    _XML                                 __mixed                       0;	///< Store mixed content as an xsd:any (an XML string by default).
};

/// @brief "urn:oasis:names:tc:SAML:2.0:assertion":AudienceRestrictionType is a complexType with complexContent extension of type "urn:oasis:names:tc:SAML:2.0:assertion":ConditionAbstractType.
///
/// @note struct saml2__AudienceRestrictionType operations:
/// - saml2__AudienceRestrictionType* soap_new_saml2__AudienceRestrictionType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml2__AudienceRestrictionType(struct soap*, saml2__AudienceRestrictionType*) default initialize members
/// - int soap_read_saml2__AudienceRestrictionType(struct soap*, saml2__AudienceRestrictionType*) deserialize from a source
/// - int soap_write_saml2__AudienceRestrictionType(struct soap*, saml2__AudienceRestrictionType*) serialize to a sink
/// - saml2__AudienceRestrictionType* soap_dup_saml2__AudienceRestrictionType(struct soap*, saml2__AudienceRestrictionType* dst, saml2__AudienceRestrictionType *src) returns deep copy of saml2__AudienceRestrictionType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml2__AudienceRestrictionType(saml2__AudienceRestrictionType*) deep deletes saml2__AudienceRestrictionType data members, use only on dst after soap_dup_saml2__AudienceRestrictionType(NULL, saml2__AudienceRestrictionType *dst, saml2__AudienceRestrictionType *src) (use soapcpp2 -Ed)
struct saml2__AudienceRestrictionType
{
/// INHERITED FROM saml2__ConditionAbstractType:
//  END OF INHERITED FROM saml2__ConditionAbstractType
/// Size of the dynamic array of values of type char* * is 1..unbounded.
  $ int                                  __sizeAudience                 1;
/// Array char* * of size 1..unbounded.
    char* *                              saml2__Audience                1;	///< Multiple elements.
};

/// @brief "urn:oasis:names:tc:SAML:2.0:assertion":OneTimeUseType is a complexType with complexContent extension of type "urn:oasis:names:tc:SAML:2.0:assertion":ConditionAbstractType.
///
/// @note struct saml2__OneTimeUseType operations:
/// - saml2__OneTimeUseType* soap_new_saml2__OneTimeUseType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml2__OneTimeUseType(struct soap*, saml2__OneTimeUseType*) default initialize members
/// - int soap_read_saml2__OneTimeUseType(struct soap*, saml2__OneTimeUseType*) deserialize from a source
/// - int soap_write_saml2__OneTimeUseType(struct soap*, saml2__OneTimeUseType*) serialize to a sink
/// - saml2__OneTimeUseType* soap_dup_saml2__OneTimeUseType(struct soap*, saml2__OneTimeUseType* dst, saml2__OneTimeUseType *src) returns deep copy of saml2__OneTimeUseType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml2__OneTimeUseType(saml2__OneTimeUseType*) deep deletes saml2__OneTimeUseType data members, use only on dst after soap_dup_saml2__OneTimeUseType(NULL, saml2__OneTimeUseType *dst, saml2__OneTimeUseType *src) (use soapcpp2 -Ed)
struct saml2__OneTimeUseType
{
/// INHERITED FROM saml2__ConditionAbstractType:
//  END OF INHERITED FROM saml2__ConditionAbstractType
};

/// @brief "urn:oasis:names:tc:SAML:2.0:assertion":ProxyRestrictionType is a complexType with complexContent extension of type "urn:oasis:names:tc:SAML:2.0:assertion":ConditionAbstractType.
///
/// @note struct saml2__ProxyRestrictionType operations:
/// - saml2__ProxyRestrictionType* soap_new_saml2__ProxyRestrictionType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml2__ProxyRestrictionType(struct soap*, saml2__ProxyRestrictionType*) default initialize members
/// - int soap_read_saml2__ProxyRestrictionType(struct soap*, saml2__ProxyRestrictionType*) deserialize from a source
/// - int soap_write_saml2__ProxyRestrictionType(struct soap*, saml2__ProxyRestrictionType*) serialize to a sink
/// - saml2__ProxyRestrictionType* soap_dup_saml2__ProxyRestrictionType(struct soap*, saml2__ProxyRestrictionType* dst, saml2__ProxyRestrictionType *src) returns deep copy of saml2__ProxyRestrictionType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml2__ProxyRestrictionType(saml2__ProxyRestrictionType*) deep deletes saml2__ProxyRestrictionType data members, use only on dst after soap_dup_saml2__ProxyRestrictionType(NULL, saml2__ProxyRestrictionType *dst, saml2__ProxyRestrictionType *src) (use soapcpp2 -Ed)
struct saml2__ProxyRestrictionType
{
/// INHERITED FROM saml2__ConditionAbstractType:
//  END OF INHERITED FROM saml2__ConditionAbstractType
/// Size of the dynamic array of values of type char* * is 0..unbounded.
  $ int                                  __sizeAudience                 0;
/// Array char* * of size 0..unbounded.
    char* *                              saml2__Audience                0;	///< Multiple elements.
/// Attribute "Count" of type xs:nonNegativeInteger.
  @ char*                                Count                          0;	///< Optional attribute.
};

/// @brief "urn:oasis:names:tc:SAML:2.0:assertion":AuthnStatementType is a complexType with complexContent extension of type "urn:oasis:names:tc:SAML:2.0:assertion":StatementAbstractType.
///
/// @note struct saml2__AuthnStatementType operations:
/// - saml2__AuthnStatementType* soap_new_saml2__AuthnStatementType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml2__AuthnStatementType(struct soap*, saml2__AuthnStatementType*) default initialize members
/// - int soap_read_saml2__AuthnStatementType(struct soap*, saml2__AuthnStatementType*) deserialize from a source
/// - int soap_write_saml2__AuthnStatementType(struct soap*, saml2__AuthnStatementType*) serialize to a sink
/// - saml2__AuthnStatementType* soap_dup_saml2__AuthnStatementType(struct soap*, saml2__AuthnStatementType* dst, saml2__AuthnStatementType *src) returns deep copy of saml2__AuthnStatementType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml2__AuthnStatementType(saml2__AuthnStatementType*) deep deletes saml2__AuthnStatementType data members, use only on dst after soap_dup_saml2__AuthnStatementType(NULL, saml2__AuthnStatementType *dst, saml2__AuthnStatementType *src) (use soapcpp2 -Ed)
struct saml2__AuthnStatementType
{
/// INHERITED FROM saml2__StatementAbstractType:
//  END OF INHERITED FROM saml2__StatementAbstractType
/// Element reference "urn:oasis:names:tc:SAML:2.0:assertion:"SubjectLocality.
    struct saml2__SubjectLocalityType*   saml2__SubjectLocality         0;	///< Optional element.
/// Element reference "urn:oasis:names:tc:SAML:2.0:assertion:"AuthnContext.
    struct saml2__AuthnContextType*      saml2__AuthnContext            1;	///< Required element.
/// Attribute "AuthnInstant" of type xs:dateTime.
  @ xsd__dateTime                        AuthnInstant                   1;	///< Required attribute.
/// Attribute "SessionIndex" of type xs:string.
  @ char*                                SessionIndex                   0;	///< Optional attribute.
/// Attribute "SessionNotOnOrAfter" of type xs:dateTime.
  @ xsd__dateTime*                       SessionNotOnOrAfter            0;	///< Optional attribute.
};

/// @brief "urn:oasis:names:tc:SAML:2.0:assertion":AuthzDecisionStatementType is a complexType with complexContent extension of type "urn:oasis:names:tc:SAML:2.0:assertion":StatementAbstractType.
///
/// @note struct saml2__AuthzDecisionStatementType operations:
/// - saml2__AuthzDecisionStatementType* soap_new_saml2__AuthzDecisionStatementType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml2__AuthzDecisionStatementType(struct soap*, saml2__AuthzDecisionStatementType*) default initialize members
/// - int soap_read_saml2__AuthzDecisionStatementType(struct soap*, saml2__AuthzDecisionStatementType*) deserialize from a source
/// - int soap_write_saml2__AuthzDecisionStatementType(struct soap*, saml2__AuthzDecisionStatementType*) serialize to a sink
/// - saml2__AuthzDecisionStatementType* soap_dup_saml2__AuthzDecisionStatementType(struct soap*, saml2__AuthzDecisionStatementType* dst, saml2__AuthzDecisionStatementType *src) returns deep copy of saml2__AuthzDecisionStatementType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml2__AuthzDecisionStatementType(saml2__AuthzDecisionStatementType*) deep deletes saml2__AuthzDecisionStatementType data members, use only on dst after soap_dup_saml2__AuthzDecisionStatementType(NULL, saml2__AuthzDecisionStatementType *dst, saml2__AuthzDecisionStatementType *src) (use soapcpp2 -Ed)
struct saml2__AuthzDecisionStatementType
{
/// INHERITED FROM saml2__StatementAbstractType:
//  END OF INHERITED FROM saml2__StatementAbstractType
/// Size of the dynamic array of values of type struct saml2__ActionType* is 1..unbounded.
  $ int                                  __sizeAction                   1;
/// Array struct saml2__ActionType* of size 1..unbounded.
    struct saml2__ActionType*            saml2__Action                  1;	///< Multiple elements.
/// Element reference "urn:oasis:names:tc:SAML:2.0:assertion:"Evidence.
    struct saml2__EvidenceType*          saml2__Evidence                0;	///< Optional element.
/// Attribute "Resource" of type xs:anyURI.
  @ char*                                Resource                       1;	///< Required attribute.
/// Attribute "Decision" of type "urn:oasis:names:tc:SAML:2.0:assertion":DecisionType.
  @ enum saml2__DecisionType             Decision                       1;	///< Required attribute.
};

/// @brief "urn:oasis:names:tc:SAML:2.0:assertion":ActionType is a complexType with simpleContent extension of type xs:string.
///
/// @note struct saml2__ActionType operations:
/// - saml2__ActionType* soap_new_saml2__ActionType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml2__ActionType(struct soap*, saml2__ActionType*) default initialize members
/// - int soap_read_saml2__ActionType(struct soap*, saml2__ActionType*) deserialize from a source
/// - int soap_write_saml2__ActionType(struct soap*, saml2__ActionType*) serialize to a sink
/// - saml2__ActionType* soap_dup_saml2__ActionType(struct soap*, saml2__ActionType* dst, saml2__ActionType *src) returns deep copy of saml2__ActionType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml2__ActionType(saml2__ActionType*) deep deletes saml2__ActionType data members, use only on dst after soap_dup_saml2__ActionType(NULL, saml2__ActionType *dst, saml2__ActionType *src) (use soapcpp2 -Ed)
struct saml2__ActionType
{
/// INHERITED FROM saml2__ActionType:
/// __item wraps simpleContent of type xs:string.
    char*                                __item                        ;
/// Attribute "Namespace" of type xs:anyURI.
  @ char*                                Namespace                      1;	///< Required attribute.
//  END OF INHERITED FROM saml2__ActionType
};

/// @brief "urn:oasis:names:tc:SAML:2.0:assertion":AttributeStatementType is a complexType with complexContent extension of type "urn:oasis:names:tc:SAML:2.0:assertion":StatementAbstractType.
///
/// @note struct saml2__AttributeStatementType operations:
/// - saml2__AttributeStatementType* soap_new_saml2__AttributeStatementType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml2__AttributeStatementType(struct soap*, saml2__AttributeStatementType*) default initialize members
/// - int soap_read_saml2__AttributeStatementType(struct soap*, saml2__AttributeStatementType*) deserialize from a source
/// - int soap_write_saml2__AttributeStatementType(struct soap*, saml2__AttributeStatementType*) serialize to a sink
/// - saml2__AttributeStatementType* soap_dup_saml2__AttributeStatementType(struct soap*, saml2__AttributeStatementType* dst, saml2__AttributeStatementType *src) returns deep copy of saml2__AttributeStatementType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml2__AttributeStatementType(saml2__AttributeStatementType*) deep deletes saml2__AttributeStatementType data members, use only on dst after soap_dup_saml2__AttributeStatementType(NULL, saml2__AttributeStatementType *dst, saml2__AttributeStatementType *src) (use soapcpp2 -Ed)
struct saml2__AttributeStatementType
{
/// INHERITED FROM saml2__StatementAbstractType:
//  END OF INHERITED FROM saml2__StatementAbstractType
//  BEGIN CHOICE <xs:choice maxOccurs="unbounded">
  $ int                                  __size_AttributeStatementType  0;
    struct __saml2__union_AttributeStatementType
    {
/// Element reference "urn:oasis:names:tc:SAML:2.0:assertion:"Attribute.
    struct saml2__AttributeType*         saml2__Attribute              ;	///< Choice of element (one of multiple choices).
/// Element reference "urn:oasis:names:tc:SAML:2.0:assertion:"EncryptedAttribute.
    struct saml2__EncryptedElementType*  saml2__EncryptedAttribute     ;	///< Choice of element (one of multiple choices).
    }                                   *__union_AttributeStatementType;
//  END OF CHOICE
};

/// @brief "urn:oasis:names:tc:SAML:2.0:assertion":KeyInfoConfirmationDataType is a complexType with complexContent restriction of type "urn:oasis:names:tc:SAML:2.0:assertion":SubjectConfirmationDataType.
///
/// @note struct saml2__KeyInfoConfirmationDataType operations:
/// - saml2__KeyInfoConfirmationDataType* soap_new_saml2__KeyInfoConfirmationDataType(struct soap*, int num) allocate and default initialize one or more values (an array)
/// - soap_default_saml2__KeyInfoConfirmationDataType(struct soap*, saml2__KeyInfoConfirmationDataType*) default initialize members
/// - int soap_read_saml2__KeyInfoConfirmationDataType(struct soap*, saml2__KeyInfoConfirmationDataType*) deserialize from a source
/// - int soap_write_saml2__KeyInfoConfirmationDataType(struct soap*, saml2__KeyInfoConfirmationDataType*) serialize to a sink
/// - saml2__KeyInfoConfirmationDataType* soap_dup_saml2__KeyInfoConfirmationDataType(struct soap*, saml2__KeyInfoConfirmationDataType* dst, saml2__KeyInfoConfirmationDataType *src) returns deep copy of saml2__KeyInfoConfirmationDataType src into dst, copies the (cyclic) graph structure when a context is provided, or (cycle-pruned) tree structure with soap_set_mode(soap, SOAP_XML_TREE) (use soapcpp2 -Ec)
/// - soap_del_saml2__KeyInfoConfirmationDataType(saml2__KeyInfoConfirmationDataType*) deep deletes saml2__KeyInfoConfirmationDataType data members, use only on dst after soap_dup_saml2__KeyInfoConfirmationDataType(NULL, saml2__KeyInfoConfirmationDataType *dst, saml2__KeyInfoConfirmationDataType *src) (use soapcpp2 -Ed)
struct saml2__KeyInfoConfirmationDataType
{
/// Imported element reference "http://www.w3.org/2000/09/xmldsig#":KeyInfo.
/// Size of "http://www.w3.org/2000/09/xmldsig#":KeyInfo array is 1..unbounded.
  $ int                                  __sizeds__KeyInfo              1;
    _ds__KeyInfo*                       *ds__KeyInfo                    1;	///< Multiple elements.
/*  RESTRICTED FROM saml2__SubjectConfirmationDataType:
/// Attribute "NotBefore" of type xs:dateTime.
  @ xsd__dateTime*                       NotBefore                      0;	///< Optional attribute.
/// Attribute "NotOnOrAfter" of type xs:dateTime.
  @ xsd__dateTime*                       NotOnOrAfter                   0;	///< Optional attribute.
/// Attribute "Recipient" of type xs:anyURI.
  @ char*                                Recipient                      0;	///< Optional attribute.
/// Attribute "InResponseTo" of type xs:NCName.
  @ char*                                InResponseTo                   0;	///< Optional attribute.
/// Attribute "Address" of type xs:string.
  @ char*                                Address                        0;	///< Optional attribute.
/// <anyAttribute namespace="##other">.
/// @note Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
///       Use wsdl2h option -d for xsd__anyAttribute DOM (soap_dom_attribute).
    END OF RESTRICTED FROM saml2__SubjectConfirmationDataType */
};


/******************************************************************************\
 *                                                                            *
 * Additional Top-Level Elements                                              *
 *   urn:oasis:names:tc:SAML:2.0:assertion                                    *
 *                                                                            *
\******************************************************************************/

/// @brief Top-level root element "urn:oasis:names:tc:SAML:2.0:assertion":BaseID of type "urn:oasis:names:tc:SAML:2.0:assertion":BaseIDAbstractType.
typedef struct saml2__BaseIDAbstractType _saml2__BaseID;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:2.0:assertion":NameID of type "urn:oasis:names:tc:SAML:2.0:assertion":NameIDType.
typedef struct saml2__NameIDType _saml2__NameID;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:2.0:assertion":EncryptedID of type "urn:oasis:names:tc:SAML:2.0:assertion":EncryptedElementType.
typedef struct saml2__EncryptedElementType _saml2__EncryptedID;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:2.0:assertion":Issuer of type "urn:oasis:names:tc:SAML:2.0:assertion":NameIDType.
typedef struct saml2__NameIDType _saml2__Issuer;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:2.0:assertion":AssertionIDRef of type xs:NCName.
typedef char*  _saml2__AssertionIDRef;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:2.0:assertion":AssertionURIRef of type xs:anyURI.
typedef char*  _saml2__AssertionURIRef;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:2.0:assertion":Assertion of type "urn:oasis:names:tc:SAML:2.0:assertion":AssertionType.
typedef struct saml2__AssertionType _saml2__Assertion;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:2.0:assertion":Subject of type "urn:oasis:names:tc:SAML:2.0:assertion":SubjectType.
typedef struct saml2__SubjectType _saml2__Subject;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:2.0:assertion":SubjectConfirmation of type "urn:oasis:names:tc:SAML:2.0:assertion":SubjectConfirmationType.
typedef struct saml2__SubjectConfirmationType _saml2__SubjectConfirmation;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:2.0:assertion":SubjectConfirmationData of type "urn:oasis:names:tc:SAML:2.0:assertion":SubjectConfirmationDataType.
typedef struct saml2__SubjectConfirmationDataType _saml2__SubjectConfirmationData;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:2.0:assertion":Conditions of type "urn:oasis:names:tc:SAML:2.0:assertion":ConditionsType.
typedef struct saml2__ConditionsType _saml2__Conditions;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:2.0:assertion":Condition of type "urn:oasis:names:tc:SAML:2.0:assertion":ConditionAbstractType.
typedef struct saml2__ConditionAbstractType _saml2__Condition;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:2.0:assertion":AudienceRestriction of type "urn:oasis:names:tc:SAML:2.0:assertion":AudienceRestrictionType.
typedef struct saml2__AudienceRestrictionType _saml2__AudienceRestriction;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:2.0:assertion":Audience of type xs:anyURI.
typedef char*  _saml2__Audience;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:2.0:assertion":OneTimeUse of type "urn:oasis:names:tc:SAML:2.0:assertion":OneTimeUseType.
typedef struct saml2__OneTimeUseType _saml2__OneTimeUse;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:2.0:assertion":ProxyRestriction of type "urn:oasis:names:tc:SAML:2.0:assertion":ProxyRestrictionType.
typedef struct saml2__ProxyRestrictionType _saml2__ProxyRestriction;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:2.0:assertion":Advice of type "urn:oasis:names:tc:SAML:2.0:assertion":AdviceType.
typedef struct saml2__AdviceType _saml2__Advice;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:2.0:assertion":EncryptedAssertion of type "urn:oasis:names:tc:SAML:2.0:assertion":EncryptedElementType.
typedef struct saml2__EncryptedElementType _saml2__EncryptedAssertion;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:2.0:assertion":Statement of type "urn:oasis:names:tc:SAML:2.0:assertion":StatementAbstractType.
typedef struct saml2__StatementAbstractType _saml2__Statement;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:2.0:assertion":AuthnStatement of type "urn:oasis:names:tc:SAML:2.0:assertion":AuthnStatementType.
typedef struct saml2__AuthnStatementType _saml2__AuthnStatement;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:2.0:assertion":SubjectLocality of type "urn:oasis:names:tc:SAML:2.0:assertion":SubjectLocalityType.
typedef struct saml2__SubjectLocalityType _saml2__SubjectLocality;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:2.0:assertion":AuthnContext of type "urn:oasis:names:tc:SAML:2.0:assertion":AuthnContextType.
typedef struct saml2__AuthnContextType _saml2__AuthnContext;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:2.0:assertion":AuthnContextClassRef of type xs:anyURI.
typedef char*  _saml2__AuthnContextClassRef;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:2.0:assertion":AuthnContextDeclRef of type xs:anyURI.
typedef char*  _saml2__AuthnContextDeclRef;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:2.0:assertion":AuthnContextDecl of type xs:anyType.
typedef _XML _saml2__AuthnContextDecl;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:2.0:assertion":AuthenticatingAuthority of type xs:anyURI.
typedef char*  _saml2__AuthenticatingAuthority;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:2.0:assertion":AuthzDecisionStatement of type "urn:oasis:names:tc:SAML:2.0:assertion":AuthzDecisionStatementType.
typedef struct saml2__AuthzDecisionStatementType _saml2__AuthzDecisionStatement;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:2.0:assertion":Action of type "urn:oasis:names:tc:SAML:2.0:assertion":ActionType.
typedef struct saml2__ActionType _saml2__Action;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:2.0:assertion":Evidence of type "urn:oasis:names:tc:SAML:2.0:assertion":EvidenceType.
typedef struct saml2__EvidenceType _saml2__Evidence;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:2.0:assertion":AttributeStatement of type "urn:oasis:names:tc:SAML:2.0:assertion":AttributeStatementType.
typedef struct saml2__AttributeStatementType _saml2__AttributeStatement;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:2.0:assertion":Attribute of type "urn:oasis:names:tc:SAML:2.0:assertion":AttributeType.
typedef struct saml2__AttributeType _saml2__Attribute;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:2.0:assertion":AttributeValue of type xs:anyType.
typedef _XML _saml2__AttributeValue;

/// @brief Top-level root element "urn:oasis:names:tc:SAML:2.0:assertion":EncryptedAttribute of type "urn:oasis:names:tc:SAML:2.0:assertion":EncryptedElementType.
typedef struct saml2__EncryptedElementType _saml2__EncryptedAttribute;


/******************************************************************************\
 *                                                                            *
 * Additional Top-Level Attributes                                            *
 *   urn:oasis:names:tc:SAML:2.0:assertion                                    *
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

@section saml2 Top-level root elements of schema "urn:oasis:names:tc:SAML:2.0:assertion"

  - <saml2:BaseID> @ref _saml2__BaseID
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml2__BaseID(struct soap*, _saml2__BaseID*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml2__BaseID(struct soap*, _saml2__BaseID*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml2__BaseID(struct soap*, const char *URL, _saml2__BaseID*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml2__BaseID(struct soap*, const char *URL, _saml2__BaseID*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml2__BaseID(struct soap*, const char *URL, _saml2__BaseID*);
    soap_POST_recv__saml2__BaseID(struct soap*, _saml2__BaseID*);
    @endcode

  - <saml2:NameID> @ref _saml2__NameID
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml2__NameID(struct soap*, _saml2__NameID*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml2__NameID(struct soap*, _saml2__NameID*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml2__NameID(struct soap*, const char *URL, _saml2__NameID*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml2__NameID(struct soap*, const char *URL, _saml2__NameID*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml2__NameID(struct soap*, const char *URL, _saml2__NameID*);
    soap_POST_recv__saml2__NameID(struct soap*, _saml2__NameID*);
    @endcode

  - <saml2:EncryptedID> @ref _saml2__EncryptedID
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml2__EncryptedID(struct soap*, _saml2__EncryptedID*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml2__EncryptedID(struct soap*, _saml2__EncryptedID*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml2__EncryptedID(struct soap*, const char *URL, _saml2__EncryptedID*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml2__EncryptedID(struct soap*, const char *URL, _saml2__EncryptedID*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml2__EncryptedID(struct soap*, const char *URL, _saml2__EncryptedID*);
    soap_POST_recv__saml2__EncryptedID(struct soap*, _saml2__EncryptedID*);
    @endcode

  - <saml2:Issuer> @ref _saml2__Issuer
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml2__Issuer(struct soap*, _saml2__Issuer*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml2__Issuer(struct soap*, _saml2__Issuer*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml2__Issuer(struct soap*, const char *URL, _saml2__Issuer*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml2__Issuer(struct soap*, const char *URL, _saml2__Issuer*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml2__Issuer(struct soap*, const char *URL, _saml2__Issuer*);
    soap_POST_recv__saml2__Issuer(struct soap*, _saml2__Issuer*);
    @endcode

  - <saml2:AssertionIDRef> @ref _saml2__AssertionIDRef
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml2__AssertionIDRef(struct soap*, _saml2__AssertionIDRef*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml2__AssertionIDRef(struct soap*, _saml2__AssertionIDRef*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml2__AssertionIDRef(struct soap*, const char *URL, _saml2__AssertionIDRef*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml2__AssertionIDRef(struct soap*, const char *URL, _saml2__AssertionIDRef*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml2__AssertionIDRef(struct soap*, const char *URL, _saml2__AssertionIDRef*);
    soap_POST_recv__saml2__AssertionIDRef(struct soap*, _saml2__AssertionIDRef*);
    @endcode

  - <saml2:AssertionURIRef> @ref _saml2__AssertionURIRef
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml2__AssertionURIRef(struct soap*, _saml2__AssertionURIRef*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml2__AssertionURIRef(struct soap*, _saml2__AssertionURIRef*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml2__AssertionURIRef(struct soap*, const char *URL, _saml2__AssertionURIRef*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml2__AssertionURIRef(struct soap*, const char *URL, _saml2__AssertionURIRef*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml2__AssertionURIRef(struct soap*, const char *URL, _saml2__AssertionURIRef*);
    soap_POST_recv__saml2__AssertionURIRef(struct soap*, _saml2__AssertionURIRef*);
    @endcode

  - <saml2:Assertion> @ref _saml2__Assertion
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml2__Assertion(struct soap*, _saml2__Assertion*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml2__Assertion(struct soap*, _saml2__Assertion*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml2__Assertion(struct soap*, const char *URL, _saml2__Assertion*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml2__Assertion(struct soap*, const char *URL, _saml2__Assertion*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml2__Assertion(struct soap*, const char *URL, _saml2__Assertion*);
    soap_POST_recv__saml2__Assertion(struct soap*, _saml2__Assertion*);
    @endcode

  - <saml2:Subject> @ref _saml2__Subject
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml2__Subject(struct soap*, _saml2__Subject*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml2__Subject(struct soap*, _saml2__Subject*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml2__Subject(struct soap*, const char *URL, _saml2__Subject*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml2__Subject(struct soap*, const char *URL, _saml2__Subject*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml2__Subject(struct soap*, const char *URL, _saml2__Subject*);
    soap_POST_recv__saml2__Subject(struct soap*, _saml2__Subject*);
    @endcode

  - <saml2:SubjectConfirmation> @ref _saml2__SubjectConfirmation
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml2__SubjectConfirmation(struct soap*, _saml2__SubjectConfirmation*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml2__SubjectConfirmation(struct soap*, _saml2__SubjectConfirmation*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml2__SubjectConfirmation(struct soap*, const char *URL, _saml2__SubjectConfirmation*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml2__SubjectConfirmation(struct soap*, const char *URL, _saml2__SubjectConfirmation*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml2__SubjectConfirmation(struct soap*, const char *URL, _saml2__SubjectConfirmation*);
    soap_POST_recv__saml2__SubjectConfirmation(struct soap*, _saml2__SubjectConfirmation*);
    @endcode

  - <saml2:SubjectConfirmationData> @ref _saml2__SubjectConfirmationData
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml2__SubjectConfirmationData(struct soap*, _saml2__SubjectConfirmationData*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml2__SubjectConfirmationData(struct soap*, _saml2__SubjectConfirmationData*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml2__SubjectConfirmationData(struct soap*, const char *URL, _saml2__SubjectConfirmationData*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml2__SubjectConfirmationData(struct soap*, const char *URL, _saml2__SubjectConfirmationData*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml2__SubjectConfirmationData(struct soap*, const char *URL, _saml2__SubjectConfirmationData*);
    soap_POST_recv__saml2__SubjectConfirmationData(struct soap*, _saml2__SubjectConfirmationData*);
    @endcode

  - <saml2:Conditions> @ref _saml2__Conditions
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml2__Conditions(struct soap*, _saml2__Conditions*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml2__Conditions(struct soap*, _saml2__Conditions*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml2__Conditions(struct soap*, const char *URL, _saml2__Conditions*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml2__Conditions(struct soap*, const char *URL, _saml2__Conditions*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml2__Conditions(struct soap*, const char *URL, _saml2__Conditions*);
    soap_POST_recv__saml2__Conditions(struct soap*, _saml2__Conditions*);
    @endcode

  - <saml2:Condition> @ref _saml2__Condition
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml2__Condition(struct soap*, _saml2__Condition*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml2__Condition(struct soap*, _saml2__Condition*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml2__Condition(struct soap*, const char *URL, _saml2__Condition*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml2__Condition(struct soap*, const char *URL, _saml2__Condition*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml2__Condition(struct soap*, const char *URL, _saml2__Condition*);
    soap_POST_recv__saml2__Condition(struct soap*, _saml2__Condition*);
    @endcode

  - <saml2:AudienceRestriction> @ref _saml2__AudienceRestriction
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml2__AudienceRestriction(struct soap*, _saml2__AudienceRestriction*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml2__AudienceRestriction(struct soap*, _saml2__AudienceRestriction*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml2__AudienceRestriction(struct soap*, const char *URL, _saml2__AudienceRestriction*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml2__AudienceRestriction(struct soap*, const char *URL, _saml2__AudienceRestriction*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml2__AudienceRestriction(struct soap*, const char *URL, _saml2__AudienceRestriction*);
    soap_POST_recv__saml2__AudienceRestriction(struct soap*, _saml2__AudienceRestriction*);
    @endcode

  - <saml2:Audience> @ref _saml2__Audience
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml2__Audience(struct soap*, _saml2__Audience*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml2__Audience(struct soap*, _saml2__Audience*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml2__Audience(struct soap*, const char *URL, _saml2__Audience*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml2__Audience(struct soap*, const char *URL, _saml2__Audience*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml2__Audience(struct soap*, const char *URL, _saml2__Audience*);
    soap_POST_recv__saml2__Audience(struct soap*, _saml2__Audience*);
    @endcode

  - <saml2:OneTimeUse> @ref _saml2__OneTimeUse
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml2__OneTimeUse(struct soap*, _saml2__OneTimeUse*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml2__OneTimeUse(struct soap*, _saml2__OneTimeUse*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml2__OneTimeUse(struct soap*, const char *URL, _saml2__OneTimeUse*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml2__OneTimeUse(struct soap*, const char *URL, _saml2__OneTimeUse*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml2__OneTimeUse(struct soap*, const char *URL, _saml2__OneTimeUse*);
    soap_POST_recv__saml2__OneTimeUse(struct soap*, _saml2__OneTimeUse*);
    @endcode

  - <saml2:ProxyRestriction> @ref _saml2__ProxyRestriction
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml2__ProxyRestriction(struct soap*, _saml2__ProxyRestriction*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml2__ProxyRestriction(struct soap*, _saml2__ProxyRestriction*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml2__ProxyRestriction(struct soap*, const char *URL, _saml2__ProxyRestriction*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml2__ProxyRestriction(struct soap*, const char *URL, _saml2__ProxyRestriction*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml2__ProxyRestriction(struct soap*, const char *URL, _saml2__ProxyRestriction*);
    soap_POST_recv__saml2__ProxyRestriction(struct soap*, _saml2__ProxyRestriction*);
    @endcode

  - <saml2:Advice> @ref _saml2__Advice
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml2__Advice(struct soap*, _saml2__Advice*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml2__Advice(struct soap*, _saml2__Advice*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml2__Advice(struct soap*, const char *URL, _saml2__Advice*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml2__Advice(struct soap*, const char *URL, _saml2__Advice*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml2__Advice(struct soap*, const char *URL, _saml2__Advice*);
    soap_POST_recv__saml2__Advice(struct soap*, _saml2__Advice*);
    @endcode

  - <saml2:EncryptedAssertion> @ref _saml2__EncryptedAssertion
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml2__EncryptedAssertion(struct soap*, _saml2__EncryptedAssertion*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml2__EncryptedAssertion(struct soap*, _saml2__EncryptedAssertion*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml2__EncryptedAssertion(struct soap*, const char *URL, _saml2__EncryptedAssertion*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml2__EncryptedAssertion(struct soap*, const char *URL, _saml2__EncryptedAssertion*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml2__EncryptedAssertion(struct soap*, const char *URL, _saml2__EncryptedAssertion*);
    soap_POST_recv__saml2__EncryptedAssertion(struct soap*, _saml2__EncryptedAssertion*);
    @endcode

  - <saml2:Statement> @ref _saml2__Statement
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml2__Statement(struct soap*, _saml2__Statement*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml2__Statement(struct soap*, _saml2__Statement*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml2__Statement(struct soap*, const char *URL, _saml2__Statement*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml2__Statement(struct soap*, const char *URL, _saml2__Statement*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml2__Statement(struct soap*, const char *URL, _saml2__Statement*);
    soap_POST_recv__saml2__Statement(struct soap*, _saml2__Statement*);
    @endcode

  - <saml2:AuthnStatement> @ref _saml2__AuthnStatement
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml2__AuthnStatement(struct soap*, _saml2__AuthnStatement*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml2__AuthnStatement(struct soap*, _saml2__AuthnStatement*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml2__AuthnStatement(struct soap*, const char *URL, _saml2__AuthnStatement*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml2__AuthnStatement(struct soap*, const char *URL, _saml2__AuthnStatement*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml2__AuthnStatement(struct soap*, const char *URL, _saml2__AuthnStatement*);
    soap_POST_recv__saml2__AuthnStatement(struct soap*, _saml2__AuthnStatement*);
    @endcode

  - <saml2:SubjectLocality> @ref _saml2__SubjectLocality
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml2__SubjectLocality(struct soap*, _saml2__SubjectLocality*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml2__SubjectLocality(struct soap*, _saml2__SubjectLocality*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml2__SubjectLocality(struct soap*, const char *URL, _saml2__SubjectLocality*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml2__SubjectLocality(struct soap*, const char *URL, _saml2__SubjectLocality*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml2__SubjectLocality(struct soap*, const char *URL, _saml2__SubjectLocality*);
    soap_POST_recv__saml2__SubjectLocality(struct soap*, _saml2__SubjectLocality*);
    @endcode

  - <saml2:AuthnContext> @ref _saml2__AuthnContext
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml2__AuthnContext(struct soap*, _saml2__AuthnContext*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml2__AuthnContext(struct soap*, _saml2__AuthnContext*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml2__AuthnContext(struct soap*, const char *URL, _saml2__AuthnContext*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml2__AuthnContext(struct soap*, const char *URL, _saml2__AuthnContext*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml2__AuthnContext(struct soap*, const char *URL, _saml2__AuthnContext*);
    soap_POST_recv__saml2__AuthnContext(struct soap*, _saml2__AuthnContext*);
    @endcode

  - <saml2:AuthnContextClassRef> @ref _saml2__AuthnContextClassRef
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml2__AuthnContextClassRef(struct soap*, _saml2__AuthnContextClassRef*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml2__AuthnContextClassRef(struct soap*, _saml2__AuthnContextClassRef*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml2__AuthnContextClassRef(struct soap*, const char *URL, _saml2__AuthnContextClassRef*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml2__AuthnContextClassRef(struct soap*, const char *URL, _saml2__AuthnContextClassRef*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml2__AuthnContextClassRef(struct soap*, const char *URL, _saml2__AuthnContextClassRef*);
    soap_POST_recv__saml2__AuthnContextClassRef(struct soap*, _saml2__AuthnContextClassRef*);
    @endcode

  - <saml2:AuthnContextDeclRef> @ref _saml2__AuthnContextDeclRef
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml2__AuthnContextDeclRef(struct soap*, _saml2__AuthnContextDeclRef*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml2__AuthnContextDeclRef(struct soap*, _saml2__AuthnContextDeclRef*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml2__AuthnContextDeclRef(struct soap*, const char *URL, _saml2__AuthnContextDeclRef*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml2__AuthnContextDeclRef(struct soap*, const char *URL, _saml2__AuthnContextDeclRef*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml2__AuthnContextDeclRef(struct soap*, const char *URL, _saml2__AuthnContextDeclRef*);
    soap_POST_recv__saml2__AuthnContextDeclRef(struct soap*, _saml2__AuthnContextDeclRef*);
    @endcode

  - <saml2:AuthnContextDecl> @ref _saml2__AuthnContextDecl
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml2__AuthnContextDecl(struct soap*, _saml2__AuthnContextDecl*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml2__AuthnContextDecl(struct soap*, _saml2__AuthnContextDecl*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml2__AuthnContextDecl(struct soap*, const char *URL, _saml2__AuthnContextDecl*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml2__AuthnContextDecl(struct soap*, const char *URL, _saml2__AuthnContextDecl*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml2__AuthnContextDecl(struct soap*, const char *URL, _saml2__AuthnContextDecl*);
    soap_POST_recv__saml2__AuthnContextDecl(struct soap*, _saml2__AuthnContextDecl*);
    @endcode

  - <saml2:AuthenticatingAuthority> @ref _saml2__AuthenticatingAuthority
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml2__AuthenticatingAuthority(struct soap*, _saml2__AuthenticatingAuthority*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml2__AuthenticatingAuthority(struct soap*, _saml2__AuthenticatingAuthority*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml2__AuthenticatingAuthority(struct soap*, const char *URL, _saml2__AuthenticatingAuthority*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml2__AuthenticatingAuthority(struct soap*, const char *URL, _saml2__AuthenticatingAuthority*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml2__AuthenticatingAuthority(struct soap*, const char *URL, _saml2__AuthenticatingAuthority*);
    soap_POST_recv__saml2__AuthenticatingAuthority(struct soap*, _saml2__AuthenticatingAuthority*);
    @endcode

  - <saml2:AuthzDecisionStatement> @ref _saml2__AuthzDecisionStatement
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml2__AuthzDecisionStatement(struct soap*, _saml2__AuthzDecisionStatement*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml2__AuthzDecisionStatement(struct soap*, _saml2__AuthzDecisionStatement*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml2__AuthzDecisionStatement(struct soap*, const char *URL, _saml2__AuthzDecisionStatement*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml2__AuthzDecisionStatement(struct soap*, const char *URL, _saml2__AuthzDecisionStatement*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml2__AuthzDecisionStatement(struct soap*, const char *URL, _saml2__AuthzDecisionStatement*);
    soap_POST_recv__saml2__AuthzDecisionStatement(struct soap*, _saml2__AuthzDecisionStatement*);
    @endcode

  - <saml2:Action> @ref _saml2__Action
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml2__Action(struct soap*, _saml2__Action*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml2__Action(struct soap*, _saml2__Action*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml2__Action(struct soap*, const char *URL, _saml2__Action*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml2__Action(struct soap*, const char *URL, _saml2__Action*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml2__Action(struct soap*, const char *URL, _saml2__Action*);
    soap_POST_recv__saml2__Action(struct soap*, _saml2__Action*);
    @endcode

  - <saml2:Evidence> @ref _saml2__Evidence
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml2__Evidence(struct soap*, _saml2__Evidence*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml2__Evidence(struct soap*, _saml2__Evidence*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml2__Evidence(struct soap*, const char *URL, _saml2__Evidence*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml2__Evidence(struct soap*, const char *URL, _saml2__Evidence*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml2__Evidence(struct soap*, const char *URL, _saml2__Evidence*);
    soap_POST_recv__saml2__Evidence(struct soap*, _saml2__Evidence*);
    @endcode

  - <saml2:AttributeStatement> @ref _saml2__AttributeStatement
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml2__AttributeStatement(struct soap*, _saml2__AttributeStatement*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml2__AttributeStatement(struct soap*, _saml2__AttributeStatement*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml2__AttributeStatement(struct soap*, const char *URL, _saml2__AttributeStatement*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml2__AttributeStatement(struct soap*, const char *URL, _saml2__AttributeStatement*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml2__AttributeStatement(struct soap*, const char *URL, _saml2__AttributeStatement*);
    soap_POST_recv__saml2__AttributeStatement(struct soap*, _saml2__AttributeStatement*);
    @endcode

  - <saml2:Attribute> @ref _saml2__Attribute
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml2__Attribute(struct soap*, _saml2__Attribute*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml2__Attribute(struct soap*, _saml2__Attribute*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml2__Attribute(struct soap*, const char *URL, _saml2__Attribute*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml2__Attribute(struct soap*, const char *URL, _saml2__Attribute*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml2__Attribute(struct soap*, const char *URL, _saml2__Attribute*);
    soap_POST_recv__saml2__Attribute(struct soap*, _saml2__Attribute*);
    @endcode

  - <saml2:AttributeValue> @ref _saml2__AttributeValue
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml2__AttributeValue(struct soap*, _saml2__AttributeValue*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml2__AttributeValue(struct soap*, _saml2__AttributeValue*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml2__AttributeValue(struct soap*, const char *URL, _saml2__AttributeValue*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml2__AttributeValue(struct soap*, const char *URL, _saml2__AttributeValue*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml2__AttributeValue(struct soap*, const char *URL, _saml2__AttributeValue*);
    soap_POST_recv__saml2__AttributeValue(struct soap*, _saml2__AttributeValue*);
    @endcode

  - <saml2:EncryptedAttribute> @ref _saml2__EncryptedAttribute
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__saml2__EncryptedAttribute(struct soap*, _saml2__EncryptedAttribute*);
    // Writer (returns SOAP_OK on success):
    soap_write__saml2__EncryptedAttribute(struct soap*, _saml2__EncryptedAttribute*);
    // REST GET (returns SOAP_OK on success):
    soap_GET__saml2__EncryptedAttribute(struct soap*, const char *URL, _saml2__EncryptedAttribute*);
    // REST PUT (returns SOAP_OK on success):
    soap_PUT__saml2__EncryptedAttribute(struct soap*, const char *URL, _saml2__EncryptedAttribute*);
    // REST POST (returns SOAP_OK on success):
    soap_POST_send__saml2__EncryptedAttribute(struct soap*, const char *URL, _saml2__EncryptedAttribute*);
    soap_POST_recv__saml2__EncryptedAttribute(struct soap*, _saml2__EncryptedAttribute*);
    @endcode

*/

/* End of saml2.h */
