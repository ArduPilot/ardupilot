/*
	ds.h

	Generated with:
	wsdl2h -cuxy -o ds.h -t WS/WS-typemap.dat WS/ds.xsd

        This file imports:
        - c14n.h

	- Removed //gsoapopt
	- Added //gsoap ds    schema import: http://www.w3.org/2000/09/xmldsig#

*/

/******************************************************************************\
 *                                                                            *
 * http://www.w3.org/2000/09/xmldsig#                                         *
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

#define SOAP_NAMESPACE_OF_ds	"http://www.w3.org/2000/09/xmldsig#"
//gsoap ds    schema import:	http://www.w3.org/2000/09/xmldsig#
//gsoap ds    schema elementForm:	qualified
//gsoap ds    schema attributeForm:	unqualified

/******************************************************************************\
 *                                                                            *
 * Schema Types                                                               *
 *                                                                            *
\******************************************************************************/


/// Imported complexType "http://www.w3.org/2000/09/xmldsig#":SignatureType from typemap WS/WS-typemap.dat.
typedef char *_ds__SignatureValue;
typedef struct ds__SignatureType
{	struct ds__SignedInfoType*		SignedInfo;
	_ds__SignatureValue			SignatureValue;
	struct ds__KeyInfoType*			KeyInfo;
	@char*					Id;
} ds__SignatureType, _ds__Signature;

/// Imported complexType "http://www.w3.org/2000/09/xmldsig#":SignatureValueType from typemap WS/WS-typemap.dat.
/// complexType definition intentionally left blank.

/// Typedef synonym for struct ds__SignedInfoType.
typedef struct ds__SignedInfoType ds__SignedInfoType;

/// Typedef synonym for struct ds__CanonicalizationMethodType.
typedef struct ds__CanonicalizationMethodType ds__CanonicalizationMethodType;

/// Typedef synonym for struct ds__SignatureMethodType.
typedef struct ds__SignatureMethodType ds__SignatureMethodType;

/// Typedef synonym for struct ds__ReferenceType.
typedef struct ds__ReferenceType ds__ReferenceType;

/// Typedef synonym for struct ds__TransformsType.
typedef struct ds__TransformsType ds__TransformsType;

/// Imported complexType "http://www.w3.org/2000/09/xmldsig#":TransformType from typemap WS/WS-typemap.dat.
#import "c14n.h"
typedef struct ds__TransformType
{	_c14n__InclusiveNamespaces*             c14n__InclusiveNamespaces;
	_XML					__any;
	@char*					Algorithm;
} ds__TransformType, _ds__Transform;

/// Typedef synonym for struct ds__DigestMethodType.
typedef struct ds__DigestMethodType ds__DigestMethodType;

/// Imported complexType "http://www.w3.org/2000/09/xmldsig#":KeyInfoType from typemap WS/WS-typemap.dat.
mutable struct ds__KeyInfoType
{	char*					KeyName;
	struct ds__KeyValueType*		KeyValue;
	struct ds__RetrievalMethodType*		RetrievalMethod;
	struct ds__X509DataType*		X509Data;
	struct _wsse__SecurityTokenReference*	wsse__SecurityTokenReference;
	@char*					Id;
};
typedef struct ds__KeyInfoType ds__KeyInfoType, _ds__KeyInfo;

/// Typedef synonym for struct ds__KeyValueType.
typedef struct ds__KeyValueType ds__KeyValueType;

/// Typedef synonym for struct ds__RetrievalMethodType.
typedef struct ds__RetrievalMethodType ds__RetrievalMethodType;

/// Typedef synonym for struct ds__X509DataType.
typedef struct ds__X509DataType ds__X509DataType;

/// Typedef synonym for struct ds__X509IssuerSerialType.
typedef struct ds__X509IssuerSerialType ds__X509IssuerSerialType;

/// Imported complexType "http://www.w3.org/2000/09/xmldsig#":PGPDataType from typemap WS/WS-typemap.dat.
/// complexType definition intentionally left blank.

/// Imported complexType "http://www.w3.org/2000/09/xmldsig#":SPKIDataType from typemap WS/WS-typemap.dat.
/// complexType definition intentionally left blank.

/// Imported complexType "http://www.w3.org/2000/09/xmldsig#":ObjectType from typemap WS/WS-typemap.dat.
/// complexType definition intentionally left blank.

/// Imported complexType "http://www.w3.org/2000/09/xmldsig#":ManifestType from typemap WS/WS-typemap.dat.
/// complexType definition intentionally left blank.

/// Imported complexType "http://www.w3.org/2000/09/xmldsig#":SignaturePropertiesType from typemap WS/WS-typemap.dat.
/// complexType definition intentionally left blank.

/// Imported complexType "http://www.w3.org/2000/09/xmldsig#":SignaturePropertyType from typemap WS/WS-typemap.dat.
/// complexType definition intentionally left blank.

/// Typedef synonym for struct ds__DSAKeyValueType.
typedef struct ds__DSAKeyValueType ds__DSAKeyValueType;

/// Typedef synonym for struct ds__RSAKeyValueType.
typedef struct ds__RSAKeyValueType ds__RSAKeyValueType;

/// Imported simpleType "http://www.w3.org/2000/09/xmldsig#":CryptoBinary from typemap WS/WS-typemap.dat.
/// simpleType definition intentionally left blank.

/// Imported simpleType "http://www.w3.org/2000/09/xmldsig#":DigestValueType from typemap WS/WS-typemap.dat.
/// simpleType definition intentionally left blank.

/// Imported simpleType "http://www.w3.org/2000/09/xmldsig#":HMACOutputLengthType from typemap WS/WS-typemap.dat.
/// simpleType definition intentionally left blank.

/// "http://www.w3.org/2000/09/xmldsig#":SignedInfoType is a complexType.
struct ds__SignedInfoType
{
/// Element reference "http://www.w3.org/2000/09/xmldsig#":CanonicalizationMethod.
    struct ds__CanonicalizationMethodType*  CanonicalizationMethod         1;	///< Required element.
/// Element reference "http://www.w3.org/2000/09/xmldsig#":SignatureMethod.
    struct ds__SignatureMethodType*      SignatureMethod                1;	///< Required element.
/// Size of the dynamic array of struct ds__ReferenceType* is 0..unbounded
    int                                  __sizeReference               ;
/// Pointer to array of struct ds__ReferenceType*.
    struct ds__ReferenceType*           *Reference                      1;
/// Attribute Id of type xs:ID.
   @char* /*ID*/                         Id                             0;	///< Optional attribute.
};

/// "http://www.w3.org/2000/09/xmldsig#":CanonicalizationMethodType is a complexType.
struct ds__CanonicalizationMethodType
{
/// TODO: <any namespace="##any" minOccurs="0" maxOccurs="unbounded">
///       Schema extensibility is user-definable.
///       Consult the protocol documentation to change and/or insert declarations.
///       Use wsdl2h option -x to remove this element.
/// Attribute Algorithm of type xs:anyURI.
   @char* /*URI*/                        Algorithm                      1;	///< Required attribute.
/// Member declared in WS/WS-typemap.dat
   _c14n__InclusiveNamespaces*		c14n__InclusiveNamespaces;
};

/// "http://www.w3.org/2000/09/xmldsig#":SignatureMethodType is a complexType.
struct ds__SignatureMethodType
{
/// Element HMACOutputLength of type "http://www.w3.org/2000/09/xmldsig#":HMACOutputLengthType.
    int*                                 HMACOutputLength               0;	///< Optional element.
/// TODO: <any namespace="##other" minOccurs="0" maxOccurs="unbounded">
///       Schema extensibility is user-definable.
///       Consult the protocol documentation to change and/or insert declarations.
///       Use wsdl2h option -x to remove this element.
/// Attribute Algorithm of type xs:anyURI.
   @char* /*URI*/                        Algorithm                      1;	///< Required attribute.
};

/// "http://www.w3.org/2000/09/xmldsig#":ReferenceType is a complexType.
struct ds__ReferenceType
{
/// Element reference "http://www.w3.org/2000/09/xmldsig#":Transforms.
    struct ds__TransformsType*           Transforms                     0;	///< Optional element.
/// Element reference "http://www.w3.org/2000/09/xmldsig#":DigestMethod.
    struct ds__DigestMethodType*         DigestMethod                   1;	///< Required element.
/// Element reference "http://www.w3.org/2000/09/xmldsig#":DigestValue.
    char* /*base64*/                     DigestValue                    1;	///< Required element.
/// Attribute Id of type xs:ID.
   @char* /*ID*/                         Id                             0;	///< Optional attribute.
/// Attribute URI of type xs:anyURI.
   @char* /*URI*/                        URI                            0;	///< Optional attribute.
/// Attribute Type of type xs:anyURI.
   @char* /*URI*/                        Type                           0;	///< Optional attribute.
};

/// "http://www.w3.org/2000/09/xmldsig#":TransformsType is a complexType.
struct ds__TransformsType
{
/// Size of the dynamic array of ds__TransformType is 0..unbounded
    int                                  __sizeTransform               ;
/// Pointer to array of ds__TransformType.
    ds__TransformType                   *Transform                      1;
};

/// "http://www.w3.org/2000/09/xmldsig#":DigestMethodType is a complexType.
struct ds__DigestMethodType
{
/// TODO: <any namespace="##other" minOccurs="0" maxOccurs="unbounded">
///       Schema extensibility is user-definable.
///       Consult the protocol documentation to change and/or insert declarations.
///       Use wsdl2h option -x to remove this element.
/// Attribute Algorithm of type xs:anyURI.
   @char* /*URI*/                        Algorithm                      1;	///< Required attribute.
};

/// "http://www.w3.org/2000/09/xmldsig#":KeyValueType is a complexType.
struct ds__KeyValueType
{
/// CHOICE OF ELEMENTS <choice>
/// Element reference "http://www.w3.org/2000/09/xmldsig#":DSAKeyValue.
    struct ds__DSAKeyValueType*          DSAKeyValue                    0;	///< Required element.
/// Element reference "http://www.w3.org/2000/09/xmldsig#":RSAKeyValue.
    struct ds__RSAKeyValueType*          RSAKeyValue                    0;	///< Required element.
/// TODO: <any namespace="##other">
///       Schema extensibility is user-definable.
///       Consult the protocol documentation to change and/or insert declarations.
///       Use wsdl2h option -x to remove this element.

//  END OF CHOICE
};

/// "http://www.w3.org/2000/09/xmldsig#":RetrievalMethodType is a complexType.
struct ds__RetrievalMethodType
{
/// Element reference "http://www.w3.org/2000/09/xmldsig#":Transforms.
    struct ds__TransformsType*           Transforms                     0;	///< Optional element.
/// Attribute URI of type xs:anyURI.
   @char* /*URI*/                        URI                            0;	///< Optional attribute.
/// Attribute Type of type xs:anyURI.
   @char* /*URI*/                        Type                           0;	///< Optional attribute.
};

/// "http://www.w3.org/2000/09/xmldsig#":X509DataType is a complexType.
struct ds__X509DataType
{
/// CHOICE OF ELEMENTS <choice>
/// Element X509IssuerSerial of type "http://www.w3.org/2000/09/xmldsig#":X509IssuerSerialType.
    struct ds__X509IssuerSerialType*     X509IssuerSerial               0;	///< Required element.
/// Element X509SKI of type xs:base64Binary.
    char* /*base64*/                     X509SKI                        0;	///< Required element.
/// Element X509SubjectName of type xs:string.
    char*                                X509SubjectName                0;	///< Required element.
/// Element X509Certificate of type xs:base64Binary.
    char* /*base64*/                     X509Certificate                0;	///< Required element.
/// Element X509CRL of type xs:base64Binary.
    char* /*base64*/                     X509CRL                        0;	///< Required element.
/// TODO: <any namespace="##other">
///       Schema extensibility is user-definable.
///       Consult the protocol documentation to change and/or insert declarations.
///       Use wsdl2h option -x to remove this element.

//  END OF CHOICE
};

/// "http://www.w3.org/2000/09/xmldsig#":X509IssuerSerialType is a complexType.
struct ds__X509IssuerSerialType
{
/// Element X509IssuerName of type xs:string.
    char*                                X509IssuerName                 1;	///< Required element.
/// Element X509SerialNumber of type xs:integer.
    char*                                X509SerialNumber               1;	///< Required element.
};

/// "http://www.w3.org/2000/09/xmldsig#":DSAKeyValueType is a complexType.
struct ds__DSAKeyValueType
{
/// Element G of type "http://www.w3.org/2000/09/xmldsig#":CryptoBinary.
    char* /*base64*/                     G                              0;	///< Optional element.
/// Element Y of type "http://www.w3.org/2000/09/xmldsig#":CryptoBinary.
    char* /*base64*/                     Y                              1;	///< Required element.
/// Element J of type "http://www.w3.org/2000/09/xmldsig#":CryptoBinary.
    char* /*base64*/                     J                              0;	///< Optional element.
/// Element P of type "http://www.w3.org/2000/09/xmldsig#":CryptoBinary.
    char* /*base64*/                     P                              1;	///< Required element.
/// Element Q of type "http://www.w3.org/2000/09/xmldsig#":CryptoBinary.
    char* /*base64*/                     Q                              1;	///< Required element.
/// Element Seed of type "http://www.w3.org/2000/09/xmldsig#":CryptoBinary.
    char* /*base64*/                     Seed                           1;	///< Required element.
/// Element PgenCounter of type "http://www.w3.org/2000/09/xmldsig#":CryptoBinary.
    char* /*base64*/                     PgenCounter                    1;	///< Required element.
};

/// "http://www.w3.org/2000/09/xmldsig#":RSAKeyValueType is a complexType.
struct ds__RSAKeyValueType
{
/// Element Modulus of type "http://www.w3.org/2000/09/xmldsig#":CryptoBinary.
    char* /*base64*/                     Modulus                        1;	///< Required element.
/// Element Exponent of type "http://www.w3.org/2000/09/xmldsig#":CryptoBinary.
    char* /*base64*/                     Exponent                       1;	///< Required element.
};

/// Element "http://www.w3.org/2000/09/xmldsig#":Signature of complexType "http://www.w3.org/2000/09/xmldsig#":SignatureType.
/// Note: use wsdl2h option -g to generate this global element declaration.

/// Element "http://www.w3.org/2000/09/xmldsig#":SignatureValue of complexType "http://www.w3.org/2000/09/xmldsig#":SignatureValueType.
/// Note: use wsdl2h option -g to generate this global element declaration.

/// Element "http://www.w3.org/2000/09/xmldsig#":SignedInfo of complexType "http://www.w3.org/2000/09/xmldsig#":SignedInfoType.
/// Note: use wsdl2h option -g to generate this global element declaration.

/// Element "http://www.w3.org/2000/09/xmldsig#":CanonicalizationMethod of complexType "http://www.w3.org/2000/09/xmldsig#":CanonicalizationMethodType.
/// Note: use wsdl2h option -g to generate this global element declaration.

/// Element "http://www.w3.org/2000/09/xmldsig#":SignatureMethod of complexType "http://www.w3.org/2000/09/xmldsig#":SignatureMethodType.
/// Note: use wsdl2h option -g to generate this global element declaration.

/// Element "http://www.w3.org/2000/09/xmldsig#":Reference of complexType "http://www.w3.org/2000/09/xmldsig#":ReferenceType.
/// Note: use wsdl2h option -g to generate this global element declaration.

/// Element "http://www.w3.org/2000/09/xmldsig#":Transforms of complexType "http://www.w3.org/2000/09/xmldsig#":TransformsType.
/// Note: use wsdl2h option -g to generate this global element declaration.

/// Element "http://www.w3.org/2000/09/xmldsig#":Transform of complexType "http://www.w3.org/2000/09/xmldsig#":TransformType.
/// Note: use wsdl2h option -g to generate this global element declaration.

/// Element "http://www.w3.org/2000/09/xmldsig#":DigestMethod of complexType "http://www.w3.org/2000/09/xmldsig#":DigestMethodType.
/// Note: use wsdl2h option -g to generate this global element declaration.

/// Element "http://www.w3.org/2000/09/xmldsig#":DigestValue of complexType "http://www.w3.org/2000/09/xmldsig#":DigestValueType.
/// Note: use wsdl2h option -g to generate this global element declaration.

/// Element "http://www.w3.org/2000/09/xmldsig#":KeyInfo of complexType "http://www.w3.org/2000/09/xmldsig#":KeyInfoType.
/// Note: use wsdl2h option -g to generate this global element declaration.

/// Element "http://www.w3.org/2000/09/xmldsig#":KeyName of complexType xs:string.
/// Note: use wsdl2h option -g to generate this global element declaration.

/// Element "http://www.w3.org/2000/09/xmldsig#":MgmtData of complexType xs:string.
/// Note: use wsdl2h option -g to generate this global element declaration.

/// Element "http://www.w3.org/2000/09/xmldsig#":KeyValue of complexType "http://www.w3.org/2000/09/xmldsig#":KeyValueType.
/// Note: use wsdl2h option -g to generate this global element declaration.

/// Element "http://www.w3.org/2000/09/xmldsig#":RetrievalMethod of complexType "http://www.w3.org/2000/09/xmldsig#":RetrievalMethodType.
/// Note: use wsdl2h option -g to generate this global element declaration.

/// Element "http://www.w3.org/2000/09/xmldsig#":X509Data of complexType "http://www.w3.org/2000/09/xmldsig#":X509DataType.
/// Note: use wsdl2h option -g to generate this global element declaration.

/// Element "http://www.w3.org/2000/09/xmldsig#":PGPData of complexType "http://www.w3.org/2000/09/xmldsig#":PGPDataType.
/// Note: use wsdl2h option -g to generate this global element declaration.

/// Element "http://www.w3.org/2000/09/xmldsig#":SPKIData of complexType "http://www.w3.org/2000/09/xmldsig#":SPKIDataType.
/// Note: use wsdl2h option -g to generate this global element declaration.

/// Element "http://www.w3.org/2000/09/xmldsig#":Object of complexType "http://www.w3.org/2000/09/xmldsig#":ObjectType.
/// Note: use wsdl2h option -g to generate this global element declaration.

/// Element "http://www.w3.org/2000/09/xmldsig#":Manifest of complexType "http://www.w3.org/2000/09/xmldsig#":ManifestType.
/// Note: use wsdl2h option -g to generate this global element declaration.

/// Element "http://www.w3.org/2000/09/xmldsig#":SignatureProperties of complexType "http://www.w3.org/2000/09/xmldsig#":SignaturePropertiesType.
/// Note: use wsdl2h option -g to generate this global element declaration.

/// Element "http://www.w3.org/2000/09/xmldsig#":SignatureProperty of complexType "http://www.w3.org/2000/09/xmldsig#":SignaturePropertyType.
/// Note: use wsdl2h option -g to generate this global element declaration.

/// Element "http://www.w3.org/2000/09/xmldsig#":DSAKeyValue of complexType "http://www.w3.org/2000/09/xmldsig#":DSAKeyValueType.
/// Note: use wsdl2h option -g to generate this global element declaration.

/// Element "http://www.w3.org/2000/09/xmldsig#":RSAKeyValue of complexType "http://www.w3.org/2000/09/xmldsig#":RSAKeyValueType.
/// Note: use wsdl2h option -g to generate this global element declaration.

/* End of ds.h */
