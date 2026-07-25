/*

ref.h

Generated with:

wsdl2h -cyx -o ref.h -t WS/WS-typemap.dat WS/reference-1.1.xsd

Modified by Robert van Engelen:

- Removed //gsoapopt
- Changed the //gsoap schema namespace directives to imports
*/

/******************************************************************************\
 *                                                                            *
 * http://docs.oasis-open.org/wsrm/2004/06/reference-1.1.xsd                  *
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

//gsoap ref   schema import:	http://docs.oasis-open.org/wsrm/2004/06/reference-1.1.xsd
//gsoap ref   schema elementForm:	qualified
//gsoap ref   schema attributeForm:	unqualified

/******************************************************************************\
 *                                                                            *
 * Schema Types                                                               *
 *                                                                            *
\******************************************************************************/



/// Typedef synonym for struct ref__ServiceRefType.
typedef struct ref__ServiceRefType ref__ServiceRefType;

/// "http://docs.oasis-open.org/wsrm/2004/06/reference-1.1.xsd":ServiceRefType is a complexType.
struct ref__ServiceRefType
{
/// TODO: <any namespace="##other">
/// TODO: Schema extensibility is user-definable.
///       Consult the protocol documentation to change or insert declarations.
///       Use wsdl2h option -x to remove this element.
///       Use wsdl2h option -d for xsd__anyType DOM (soap_dom_element).
/// Attribute reference-scheme of type xs:anyURI.
   @char* /*URI*/                        reference_scheme               0;	///< Optional attribute.
};

/* End of ref.h */
