/*

wsrp.h

Generated with:
wsdl2h -cgy -o wsrp.h -t WS/WS-typemap.dat WS/WS-Routing.xsd

Modified by Robert van Engelen:

- Removed //gsoapopt
- Added the following directive to import WS-Routing namespace:
  //gsoap wsa schema import: http://schemas.xmlsoap.org/rp/
  This ensures that the WS-Routing schemas are not copied into the generated
  WSDL by soapcpp2 but are referenced with schema import in the generated WSDL.

*/

/******************************************************************************\
 *                                                                            *
 * http://schemas.xmlsoap.org/rp/                                             *
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

//gsoap wsrp  schema import:	http://schemas.xmlsoap.org/rp/
//gsoap wsrp  schema form:	unqualified

/******************************************************************************\
 *                                                                            *
 * Schema Types                                                               *
 *                                                                            *
\******************************************************************************/



/// Typedef synonym for struct wsrp__path_USCOREt.
typedef struct wsrp__path_USCOREt wsrp__path_USCOREt;

/// Typedef synonym for struct wsrp__via_USCOREt.
typedef struct wsrp__via_USCOREt wsrp__via_USCOREt;

/// Typedef synonym for struct wsrp__fwd_USCOREt.
typedef struct wsrp__fwd_USCOREt wsrp__fwd_USCOREt;

/// Typedef synonym for struct wsrp__rev_USCOREt.
typedef struct wsrp__rev_USCOREt wsrp__rev_USCOREt;

/// Typedef synonym for struct wsrp__found_USCOREt.
typedef struct wsrp__found_USCOREt wsrp__found_USCOREt;

/// Typedef synonym for struct wsrp__fault_USCOREt.
typedef struct wsrp__fault_USCOREt wsrp__fault_USCOREt;

/// Imported simpleType "http://schemas.xmlsoap.org/rp/":action_t from typemap WS/WS-typemap.dat.


/// Element "http://schemas.xmlsoap.org/rp/":action of simpleType "http://schemas.xmlsoap.org/rp/":action_t.
typedef char _wsrp__action;

/// Imported simpleType "http://schemas.xmlsoap.org/rp/":to_t from typemap WS/WS-typemap.dat.


/// Element "http://schemas.xmlsoap.org/rp/":to of simpleType "http://schemas.xmlsoap.org/rp/":to_t.
typedef char _wsrp__to;

/// Imported simpleType "http://schemas.xmlsoap.org/rp/":from_t from typemap WS/WS-typemap.dat.


/// Element "http://schemas.xmlsoap.org/rp/":from of simpleType "http://schemas.xmlsoap.org/rp/":from_t.
typedef char _wsrp__from;

/// Imported simpleType "http://schemas.xmlsoap.org/rp/":id_t from typemap WS/WS-typemap.dat.


/// Element "http://schemas.xmlsoap.org/rp/":id of simpleType "http://schemas.xmlsoap.org/rp/":id_t.
typedef char _wsrp__id;

/// Imported simpleType "http://schemas.xmlsoap.org/rp/":relatesTo_t from typemap WS/WS-typemap.dat.


/// Element "http://schemas.xmlsoap.org/rp/":relatesTo of simpleType "http://schemas.xmlsoap.org/rp/":relatesTo_t.
typedef char _wsrp__relatesTo;

/// Imported simpleType "http://schemas.xmlsoap.org/rp/":faultcode_t from typemap WS/WS-typemap.dat.


/// Element "http://schemas.xmlsoap.org/rp/":faultcode of simpleType "http://schemas.xmlsoap.org/rp/":faultcode_t.
typedef char _wsrp__faultcode;

/// Imported simpleType "http://schemas.xmlsoap.org/rp/":faultreason_t from typemap WS/WS-typemap.dat.


/// Element "http://schemas.xmlsoap.org/rp/":faultreason of simpleType "http://schemas.xmlsoap.org/rp/":faultreason_t.
typedef char _wsrp__faultreason;

/// "http://schemas.xmlsoap.org/rp/":path_t is a complexType.
struct wsrp__path_USCOREt
{
/// Element reference "http://schemas.xmlsoap.org/rp/":action.
    char*                                wsrp__action                   1;	///< Required element.
/// Element reference "http://schemas.xmlsoap.org/rp/":to.
    char*                                wsrp__to                       0;	///< Optional element.
/// Element reference "http://schemas.xmlsoap.org/rp/":fwd.
    struct wsrp__fwd_USCOREt*            wsrp__fwd                      0;	///< Optional element.
/// Element reference "http://schemas.xmlsoap.org/rp/":rev.
    struct wsrp__rev_USCOREt*            wsrp__rev                      0;	///< Optional element.
/// Element reference "http://schemas.xmlsoap.org/rp/":from.
    char*                                wsrp__from                     0;	///< Optional element.
/// Element reference "http://schemas.xmlsoap.org/rp/":id.
    char*                                wsrp__id                       0;	///< Optional element.
/// Element reference "http://schemas.xmlsoap.org/rp/":relatesTo.
    char*                                wsrp__relatesTo                0;	///< Optional element.
/// Element reference "http://schemas.xmlsoap.org/rp/":fault.
    struct wsrp__fault_USCOREt*          wsrp__fault                    0;	///< Optional element.
/// TODO: <anyAttribute>
///       Schema extensibility is user-definable.
///       Consult the protocol documentation to change and/or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
   @_XML                                 __anyAttribute                ;	///< Catch any attribute content in XML string.
};

/// Element "http://schemas.xmlsoap.org/rp/":path of complexType "http://schemas.xmlsoap.org/rp/":path_t.
typedef struct wsrp__path_USCOREt _wsrp__path;

/// "http://schemas.xmlsoap.org/rp/":fwd_t is a complexType.
struct wsrp__fwd_USCOREt
{
/// Size of the dynamic array of struct wsrp__via_USCOREt* is 0..unbounded
    int                                  __sizevia                     ;
/// Pointer to array of struct wsrp__via_USCOREt*.
    struct wsrp__via_USCOREt*           *wsrp__via                      0;
/// TODO: <anyAttribute>
///       Schema extensibility is user-definable.
///       Consult the protocol documentation to change and/or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
   @_XML                                 __anyAttribute                ;	///< Catch any attribute content in XML string.
};

/// Element "http://schemas.xmlsoap.org/rp/":fwd of complexType "http://schemas.xmlsoap.org/rp/":fwd_t.
typedef struct wsrp__fwd_USCOREt _wsrp__fwd;

/// "http://schemas.xmlsoap.org/rp/":rev_t is a complexType.
struct wsrp__rev_USCOREt
{
/// Size of the dynamic array of struct wsrp__via_USCOREt* is 0..unbounded
    int                                  __sizevia                     ;
/// Pointer to array of struct wsrp__via_USCOREt*.
    struct wsrp__via_USCOREt*           *wsrp__via                      0;
/// TODO: <anyAttribute>
///       Schema extensibility is user-definable.
///       Consult the protocol documentation to change and/or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
   @_XML                                 __anyAttribute                ;	///< Catch any attribute content in XML string.
};

/// Element "http://schemas.xmlsoap.org/rp/":rev of complexType "http://schemas.xmlsoap.org/rp/":rev_t.
typedef struct wsrp__rev_USCOREt _wsrp__rev;

/// "http://schemas.xmlsoap.org/rp/":found_t is a complexType.
struct wsrp__found_USCOREt
{
/// Size of the dynamic array of char* is 0..unbounded
    int                                  __sizeat                      ;
/// Pointer to array of char*.
    char*                               *at                             1;
/// TODO: <anyAttribute>
///       Schema extensibility is user-definable.
///       Consult the protocol documentation to change and/or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
   @_XML                                 __anyAttribute                ;	///< Catch any attribute content in XML string.
};

/// Element "http://schemas.xmlsoap.org/rp/":found of complexType "http://schemas.xmlsoap.org/rp/":found_t.
typedef struct wsrp__found_USCOREt _wsrp__found;

/// "http://schemas.xmlsoap.org/rp/":fault_t is a complexType.
struct wsrp__fault_USCOREt
{
/// Element reference "http://schemas.xmlsoap.org/rp/":faultcode.
    char*                                wsrp__faultcode                1;	///< Required element.
/// Element reference "http://schemas.xmlsoap.org/rp/":faultreason.
    char*                                wsrp__faultreason              1;	///< Required element.
/// Element endpoint of type xs:anyURI.
    char*                                endpoint                       0;	///< Optional element.
/// Element found of type "http://schemas.xmlsoap.org/rp/":found_t.
    struct wsrp__found_USCOREt*          found                          0;	///< Optional element.
/// Element maxsize of type xs:integer.
    char*                                maxsize                        0;	///< Optional element.
/// Element maxtime of type xs:integer.
    char*                                maxtime                        0;	///< Optional element.
/// Element retryAfter of type xs:integer.
    char*                                retryAfter                     0;	///< Optional element.
/// TODO: <anyAttribute>
///       Schema extensibility is user-definable.
///       Consult the protocol documentation to change and/or insert declarations.
///       Use wsdl2h option -x to remove this attribute.
   @_XML                                 __anyAttribute                ;	///< Catch any attribute content in XML string.
};

/// Element "http://schemas.xmlsoap.org/rp/":fault of complexType "http://schemas.xmlsoap.org/rp/":fault_t.
typedef struct wsrp__fault_USCOREt _wsrp__fault;

/// "http://schemas.xmlsoap.org/rp/":via_t is a complexType with simpleContent.
struct wsrp__via_USCOREt
{
/// __item wraps simpleContent.
    char*                                __item                        ;
/// Attribute vid of type xs:anyURI.
   @char*                                vid                            0;	///< Optional attribute.
};

/// Element "http://schemas.xmlsoap.org/rp/":via of complexType "http://schemas.xmlsoap.org/rp/":via_t.
typedef struct wsrp__via_USCOREt _wsrp__via;

/* End of wsrp.h */
