/*

wsu.h

Generated with:
wsdl2h -cegy -o wsu.h -t WS/WS-typemap.dat WS/wsu.xsd

Modified by Robert van Engelen:

- Removed //gsoapopt
- Added the following directive to import wsu namespace
  //gsoap wsu schema import: http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd
  This ensures that the WS-Addressing schemas are not copied into the generated
  WSDL by soapcpp2 but are referenced with schema import in the generated WSDL.

*/

/******************************************************************************\
 *                                                                            *
 * http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd*
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

#define SOAP_NAMESPACE_OF_wsu	"http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd"
//gsoap wsu   schema import:	http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd
//gsoap wsu   schema elementForm:	qualified
//gsoap wsu   schema attributeForm:	unqualified

/******************************************************************************\
 *                                                                            *
 * Schema Types                                                               *
 *                                                                            *
\******************************************************************************/



/// Imported complexType "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd":AttributedDateTime from typemap WS/WS-typemap.dat.
/// @brief This type is for elements whose [children] is a psuedo-dateTime and can have arbitrary attributes.
/// complexType definition intentionally left blank.

/// Imported complexType "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd":AttributedURI from typemap WS/WS-typemap.dat.
/// @brief This type is for elements whose [children] is an anyURI and can have arbitrary attributes.
/// complexType definition intentionally left blank.

/// Imported complexType "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd":TimestampType from typemap WS/WS-typemap.dat.
/// @brief This complex type ties together the timestamp related elements into a composite type.
/// complexType definition intentionally left blank.

/// "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd":tTimestampFault is a simpleType restriction of xs:QName.
/// @brief This type defines the fault code value for Timestamp message expiration.
/// Note: enum values are prefixed with 'wsu__tTimestampFault' to avoid name clashes, please use wsdl2h option -e to omit this prefix
enum wsu__tTimestampFault
{
	wsu__MessageExpired,	///< xs:QName value=""http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd":MessageExpired"
};
/// Typedef synonym for enum wsu__tTimestampFault.
typedef enum wsu__tTimestampFault wsu__tTimestampFault;

/// Element "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd":Timestamp of complexType "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd":TimestampType.
/// @brief This element allows Timestamps to be applied anywhere element wildcards are present, including as a SOAP header.
/// Imported element _wsu__Timestamp from typemap WS/WS-typemap.dat.
typedef struct _wsu__Timestamp
{	@char*	wsu__Id;	// use qualified form to enable signature
	char*	Created;
	char*	Expires;
} _wsu__Timestamp;

/// Element "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd":Expires of complexType "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd":AttributedDateTime.
/// @brief This element allows an expiration time to be applied anywhere element wildcards are present.
/// '_wsu__Expires' element definition intentionally left blank.

/// Element "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd":Created of complexType "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd":AttributedDateTime.
/// @brief This element allows a creation time to be applied anywhere element wildcards are present.
/// '_wsu__Created' element definition intentionally left blank.

/// Attribute "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd":Id of simpleType xs:ID.
/// @brief This global attribute supports annotating arbitrary elements with an ID.
/// '_wsu__Id' attribute definition intentionally left blank.

/* End of wsu.h */
