/*
	xenc.h

	Generated with:
	wsdl2h -cuxy -o xenc.h -t WS/WS-typemap.dat WS/xenc.xsd

	- Removed //gsoapopt
	- Added //gsoap xenc    schema import: http://www.w3.org/2001/04/xmlenc#

*/

/******************************************************************************\
 *                                                                            *
 * Definitions                                                                *
 *   http://www.w3.org/2001/04/xmlenc#                                        *
 *                                                                            *
\******************************************************************************/


/******************************************************************************\
 *                                                                            *
 * Import                                                                     *
 *                                                                            *
\******************************************************************************/

#import "ds2.h"	// ds = <http://www.w3.org/2000/09/xmldsig#>

/******************************************************************************\
 *                                                                            *
 * Schema Namespaces                                                          *
 *                                                                            *
\******************************************************************************/

//gsoap xenc  schema import:	http://www.w3.org/2001/04/xmlenc#
//gsoap xenc  schema elementForm:	qualified
//gsoap xenc  schema attributeForm:	unqualified

/******************************************************************************\
 *                                                                            *
 * Built-in Schema Types and Top-Level Elements and Attributes                *
 *                                                                            *
\******************************************************************************/


/// Imported type "http://www.w3.org/2000/09/xmldsig#":KeyInfoType from typemap WS/WS-typemap.dat.

// Imported element "http://www.w3.org/2000/09/xmldsig#":KeyInfo declared as _ds__KeyInfo

// Imported element "http://www.w3.org/2000/09/xmldsig#":Transform declared as _ds__Transform


/// Typedef synonym for struct xenc__EncryptedType.
typedef struct xenc__EncryptedType xenc__EncryptedType;

/// Typedef synonym for struct xenc__EncryptionMethodType.
typedef struct xenc__EncryptionMethodType xenc__EncryptionMethodType;

/// Typedef synonym for struct xenc__CipherDataType.
typedef struct xenc__CipherDataType xenc__CipherDataType;

/// Typedef synonym for struct xenc__CipherReferenceType.
typedef struct xenc__CipherReferenceType xenc__CipherReferenceType;

/// Typedef synonym for struct xenc__TransformsType.
typedef struct xenc__TransformsType xenc__TransformsType;

/// Typedef synonym for struct xenc__EncryptedDataType.
typedef struct xenc__EncryptedDataType xenc__EncryptedDataType;

/// Typedef synonym for struct xenc__EncryptedKeyType.
typedef struct xenc__EncryptedKeyType xenc__EncryptedKeyType;

/// Typedef synonym for struct xenc__AgreementMethodType.
typedef struct xenc__AgreementMethodType xenc__AgreementMethodType;

/// Typedef synonym for struct xenc__ReferenceType.
typedef struct xenc__ReferenceType xenc__ReferenceType;

/// Typedef synonym for struct xenc__EncryptionPropertiesType.
typedef struct xenc__EncryptionPropertiesType xenc__EncryptionPropertiesType;

/// Imported complexType "http://www.w3.org/2001/04/xmlenc#":EncryptionPropertyType from typemap WS/WS-typemap.dat.
typedef struct xenc__EncryptionPropertyType
{	@char*					Target;
	@char*					Id;
} xenc__EncryptionPropertyType;

/// Typedef synonym for struct _xenc__ReferenceList.
typedef struct _xenc__ReferenceList _xenc__ReferenceList;

/******************************************************************************\
 *                                                                            *
 * Schema Types and Top-Level Elements and Attributes                         *
 *   http://www.w3.org/2001/04/xmlenc#                                        *
 *                                                                            *
\******************************************************************************/


/// Imported simpleType "http://www.w3.org/2001/04/xmlenc#":KeySizeType from typemap WS/WS-typemap.dat.
// simpleType definition intentionally left blank.

/******************************************************************************\
 *                                                                            *
 * Schema Complex Types and Top-Level Elements                                *
 *   http://www.w3.org/2001/04/xmlenc#                                        *
 *                                                                            *
\******************************************************************************/


/// "http://www.w3.org/2001/04/xmlenc#":EncryptedType is an abstract complexType.
struct xenc__EncryptedType
{
/// Element EncryptionMethod of type "http://www.w3.org/2001/04/xmlenc#":EncryptionMethodType.
    struct xenc__EncryptionMethodType*   EncryptionMethod               0;	///< Optional element.
/// Imported element reference "http://www.w3.org/2000/09/xmldsig#":KeyInfo.
    _ds__KeyInfo*                        ds__KeyInfo                    0;	///< Optional element.
/// Element reference "http://www.w3.org/2001/04/xmlenc#":CipherData.
    struct xenc__CipherDataType*         CipherData                     1;	///< Required element.
/// Element reference "http://www.w3.org/2001/04/xmlenc#":EncryptionProperties.
    struct xenc__EncryptionPropertiesType*  EncryptionProperties           0;	///< Optional element.
/// Attribute Id of type xs:ID.
   @char*                                Id                             0;	///< Optional attribute.
/// Attribute Type of type xs:anyURI.
   @char*                                Type                           0;	///< Optional attribute.
/// Attribute MimeType of type xs:string.
   @char*                                MimeType                       0;	///< Optional attribute.
/// Attribute Encoding of type xs:anyURI.
   @char*                                Encoding                       0;	///< Optional attribute.
};

/// "http://www.w3.org/2001/04/xmlenc#":EncryptionMethodType is a complexType.
struct xenc__EncryptionMethodType
{
/// Element KeySize of type "http://www.w3.org/2001/04/xmlenc#":KeySizeType.
    int*                                 KeySize                        0;	///< Optional element.
/// Element OAEPparams of type xs:base64Binary.
    char*                                OAEPparams                     0;	///< Optional element.
/// TODO: <any namespace="##other" minOccurs="0" maxOccurs="unbounded">
/// TODO: Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element).
/// Attribute Algorithm of type xs:anyURI.
   @char*                                Algorithm                      1;	///< Required attribute.
/// Member declared in WS/WS-typemap.dat
       struct ds__DigestMethodType*         ds__DigestMethod;
/// TODO: this mixed complexType is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element).
    _XML                                 __mixed                       0;	///< Catch mixed content in XML string
};

/// "http://www.w3.org/2001/04/xmlenc#":CipherDataType is a complexType.
struct xenc__CipherDataType
{
/// CHOICE OF ELEMENTS <xs:choice>
/// Element CipherValue of type xs:base64Binary.
    char*                                CipherValue                   ;
/// Element reference "http://www.w3.org/2001/04/xmlenc#":CipherReference.
    struct xenc__CipherReferenceType*    CipherReference               ;
//  END OF CHOICE
};

/// "http://www.w3.org/2001/04/xmlenc#":CipherReferenceType is a complexType.
struct xenc__CipherReferenceType
{
/// CHOICE OF ELEMENTS <xs:choice>
/// Element Transforms of type "http://www.w3.org/2001/04/xmlenc#":TransformsType.
    struct xenc__TransformsType*         Transforms                    ;
//  END OF CHOICE
/// Attribute URI of type xs:anyURI.
   @char*                                URI                            1;	///< Required attribute.
};

/// "http://www.w3.org/2001/04/xmlenc#":TransformsType is a complexType.
struct xenc__TransformsType
{
/// Imported element reference "http://www.w3.org/2000/09/xmldsig#":Transform.
    _ds__Transform                       ds__Transform                  1;
};

/// "http://www.w3.org/2001/04/xmlenc#":AgreementMethodType is a complexType.
struct xenc__AgreementMethodType
{
/// Element KA-Nonce of type xs:base64Binary.
    char*                                KA_Nonce                       0;	///< Optional element.
/// TODO: <any namespace="##other" minOccurs="0" maxOccurs="unbounded">
/// TODO: Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element).
/// Element OriginatorKeyInfo of type "http://www.w3.org/2000/09/xmldsig#":KeyInfoType.
    ds__KeyInfoType*                     OriginatorKeyInfo              0;	///< Optional element.
/// Element RecipientKeyInfo of type "http://www.w3.org/2000/09/xmldsig#":KeyInfoType.
    ds__KeyInfoType*                     RecipientKeyInfo               0;	///< Optional element.
/// Attribute Algorithm of type xs:anyURI.
   @char*                                Algorithm                      1;	///< Required attribute.
/// TODO: this mixed complexType is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element).
    _XML                                 __mixed                       0;	///< Catch mixed content in XML string
};

/// "http://www.w3.org/2001/04/xmlenc#":ReferenceType is a complexType.
struct xenc__ReferenceType
{
/// TODO: <any namespace="##other" minOccurs="0" maxOccurs="unbounded">
/// TODO: Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element).
/// Attribute URI of type xs:anyURI.
   @char*                                URI                            1;	///< Required attribute.
};

/// "http://www.w3.org/2001/04/xmlenc#":EncryptionPropertiesType is a complexType.
struct xenc__EncryptionPropertiesType
{
/// Size of the dynamic array of xenc__EncryptionPropertyType* is 1..unbounded
   $int                                  __sizeEncryptionProperty       1;
/// Array xenc__EncryptionPropertyType* of length 1..unbounded
    xenc__EncryptionPropertyType*        EncryptionProperty             1;
/// Attribute Id of type xs:ID.
   @char*                                Id                             0;	///< Optional attribute.
};


/// Top-level root element "http://www.w3.org/2001/04/xmlenc#":ReferenceList

/// "http://www.w3.org/2001/04/xmlenc#":ReferenceList is a complexType.
struct _xenc__ReferenceList
{
/// CHOICE OF ELEMENTS <xs:choice minOccurs="1" maxOccurs="unbounded">
   $int                                  __size_ReferenceList           1;
    struct __xenc__union_ReferenceList
    {
/// Element DataReference of type "http://www.w3.org/2001/04/xmlenc#":ReferenceType.
    struct xenc__ReferenceType*          DataReference                 ;
/// Element KeyReference of type "http://www.w3.org/2001/04/xmlenc#":ReferenceType.
    struct xenc__ReferenceType*          KeyReference                  ;
    }                                   *__union_ReferenceList         ;
//  END OF CHOICE
};

/// "http://www.w3.org/2001/04/xmlenc#":EncryptedDataType is a complexType with complexContent extension of "http://www.w3.org/2001/04/xmlenc#":EncryptedType.
struct xenc__EncryptedDataType
{
/// INHERITED FROM xenc__EncryptedType:
/// Element EncryptionMethod of type "http://www.w3.org/2001/04/xmlenc#":EncryptionMethodType.
    struct xenc__EncryptionMethodType*   EncryptionMethod               0;	///< Optional element.
/// Imported element reference "http://www.w3.org/2000/09/xmldsig#":KeyInfo.
    _ds__KeyInfo*                        ds__KeyInfo                    0;	///< Optional element.
/// Element reference "http://www.w3.org/2001/04/xmlenc#":CipherData.
    struct xenc__CipherDataType*         CipherData                     1;	///< Required element.
/// Element reference "http://www.w3.org/2001/04/xmlenc#":EncryptionProperties.
    struct xenc__EncryptionPropertiesType*  EncryptionProperties           0;	///< Optional element.
/// Attribute Id of type xs:ID.
   @char*                                Id                             0;	///< Optional attribute.
/// Attribute Type of type xs:anyURI.
   @char*                                Type                           0;	///< Optional attribute.
/// Attribute MimeType of type xs:string.
   @char*                                MimeType                       0;	///< Optional attribute.
/// Attribute Encoding of type xs:anyURI.
   @char*                                Encoding                       0;	///< Optional attribute.
//  END OF INHERITED FROM xenc__EncryptedType
};

/// "http://www.w3.org/2001/04/xmlenc#":EncryptedKeyType is a complexType with complexContent extension of "http://www.w3.org/2001/04/xmlenc#":EncryptedType.
struct xenc__EncryptedKeyType
{
/// INHERITED FROM xenc__EncryptedType:
/// Element EncryptionMethod of type "http://www.w3.org/2001/04/xmlenc#":EncryptionMethodType.
    struct xenc__EncryptionMethodType*   EncryptionMethod               0;	///< Optional element.
/// Imported element reference "http://www.w3.org/2000/09/xmldsig#":KeyInfo.
    _ds__KeyInfo*                        ds__KeyInfo                    0;	///< Optional element.
/// Element reference "http://www.w3.org/2001/04/xmlenc#":CipherData.
    struct xenc__CipherDataType*         CipherData                     1;	///< Required element.
/// Element reference "http://www.w3.org/2001/04/xmlenc#":EncryptionProperties.
    struct xenc__EncryptionPropertiesType*  EncryptionProperties           0;	///< Optional element.
/// Attribute Id of type xs:ID.
   @char*                                Id                             0;	///< Optional attribute.
/// Attribute Type of type xs:anyURI.
   @char*                                Type                           0;	///< Optional attribute.
/// Attribute MimeType of type xs:string.
   @char*                                MimeType                       0;	///< Optional attribute.
/// Attribute Encoding of type xs:anyURI.
   @char*                                Encoding                       0;	///< Optional attribute.
//  END OF INHERITED FROM xenc__EncryptedType
/// Element reference "http://www.w3.org/2001/04/xmlenc#":ReferenceList.
    struct _xenc__ReferenceList*         ReferenceList                  0;	///< Optional element.
/// Element CarriedKeyName of type xs:string.
    char*                                CarriedKeyName                 0;	///< Optional element.
/// Attribute Recipient of type xs:string.
   @char*                                Recipient                      0;	///< Optional attribute.
};

/******************************************************************************\
 *                                                                            *
 * Additional Top-Level Elements                                              *
 *   http://www.w3.org/2001/04/xmlenc#                                        *
 *                                                                            *
\******************************************************************************/


/// Top-level root element "http://www.w3.org/2001/04/xmlenc#":CipherData of type "http://www.w3.org/2001/04/xmlenc#":CipherDataType.
/// Note: use wsdl2h option -g to auto-generate a top-level root element declaration and processing code.

/// Top-level root element "http://www.w3.org/2001/04/xmlenc#":CipherReference of type "http://www.w3.org/2001/04/xmlenc#":CipherReferenceType.
/// Note: use wsdl2h option -g to auto-generate a top-level root element declaration and processing code.

/// Top-level root element "http://www.w3.org/2001/04/xmlenc#":EncryptedData of type "http://www.w3.org/2001/04/xmlenc#":EncryptedDataType.
/// Note: use wsdl2h option -g to auto-generate a top-level root element declaration and processing code.

/// Top-level root element "http://www.w3.org/2001/04/xmlenc#":EncryptedKey of type "http://www.w3.org/2001/04/xmlenc#":EncryptedKeyType.
/// Note: use wsdl2h option -g to auto-generate a top-level root element declaration and processing code.

/// Top-level root element "http://www.w3.org/2001/04/xmlenc#":AgreementMethod of type "http://www.w3.org/2001/04/xmlenc#":AgreementMethodType.
/// Note: use wsdl2h option -g to auto-generate a top-level root element declaration and processing code.

/// Top-level root element "http://www.w3.org/2001/04/xmlenc#":EncryptionProperties of type "http://www.w3.org/2001/04/xmlenc#":EncryptionPropertiesType.
/// Note: use wsdl2h option -g to auto-generate a top-level root element declaration and processing code.

/// Top-level root element "http://www.w3.org/2001/04/xmlenc#":EncryptionProperty of type "http://www.w3.org/2001/04/xmlenc#":EncryptionPropertyType.
/// Note: use wsdl2h option -g to auto-generate a top-level root element declaration and processing code.

/******************************************************************************\
 *                                                                            *
 * Additional Top-Level Attributes                                            *
 *   http://www.w3.org/2001/04/xmlenc#                                        *
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

XML content can be retrieved from:
  - a file descriptor, using soap->recvfd = fd
  - a socket, using soap->socket = ...
  - a C++ stream, using soap->is = ...
  - a buffer, using the soap->frecv() callback

XML content can be stored to:
  - a file descriptor, using soap->sendfd = fd
  - a socket, using soap->socket = ...
  - a C++ stream, using soap->os = ...
  - a buffer, using the soap->fsend() callback


@section xenc Top-level root elements of schema "http://www.w3.org/2001/04/xmlenc#"

  - <xenc:CipherData> (use wsdl2h option -g to auto-generate)

  - <xenc:CipherReference> (use wsdl2h option -g to auto-generate)

  - <xenc:EncryptedData> (use wsdl2h option -g to auto-generate)

  - <xenc:EncryptedKey> (use wsdl2h option -g to auto-generate)

  - <xenc:AgreementMethod> (use wsdl2h option -g to auto-generate)

  - <xenc:ReferenceList> @ref _xenc__ReferenceList
    @code
    // Reader (returns SOAP_OK on success):
    soap_read__xenc__ReferenceList(struct soap*, struct _xenc__ReferenceList*);
    // Writer (returns SOAP_OK on success):
    soap_write__xenc__ReferenceList(struct soap*, struct _xenc__ReferenceList*);
    @endcode

  - <xenc:EncryptionProperties> (use wsdl2h option -g to auto-generate)

  - <xenc:EncryptionProperty> (use wsdl2h option -g to auto-generate)

*/

/* End of xenc.h */
