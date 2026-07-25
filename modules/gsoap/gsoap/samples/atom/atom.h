/// Atom 1.0 specification for C++
///
/// [RFC4287] https://tools.ietf.org/html/rfc4287
///
/// Atom 1.0 is not defined in an XSD.  This specification accurately reflects
/// the Atom 1.0 XML structure, but not the XSD this specification generates!
///
/// Class a__feed is the element root of an Atom document
///
/// Read/write feeds by calling:
/// int soap_read_a__feed(struct soap *, a__feed *feed)
/// int soap_write_a__feed(struct soap *, const a__feed *feed)
///
/// GET feeds by calling:
/// int soap_GET_a__feed(struct soap *, const char *URL, a__feed *feed)
///
/// PUT feeds with:
/// soap->http_content = "application/atom+xml";
/// then call:
/// int soap_PUT_a__feed(struct soap *, const char *URL, const a__feed *feed)

/*

   Warning:
   XML content with type _XML (a char* type) in a__text and all __any members
   in several classes allow the delivery of HTML and XHTML.  Many elements in
   these languages are considered 'unsafe' in that they open clients to one or
   more types of attack.  Implementers of software that processes Atom should
   carefully consider their handling of every type of element when processing
   incoming (X)HTML in Atom Documents.  See the security sections of [RFC2854]
   and [HTML] for guidance.  See [RFC4287] https://tools.ietf.org/html/rfc4287
   for details.

   [RFC4287] 4.1.3.3.  Processing Model

   Atom Documents MUST conform to the following rules.  Atom Processors
   MUST interpret atom:content according to the first applicable rule.

   1.  If the value of "type" is "text", the content of atom:content
       MUST NOT contain child elements.  Such text is intended to be
       presented to humans in a readable fashion.  Thus, Atom Processors
       MAY collapse white space (including line breaks), and display the
       text using typographic techniques such as justification and
       proportional fonts.

   2.  If the value of "type" is "html", the content of atom:content
       MUST NOT contain child elements and SHOULD be suitable for
       handling as HTML [HTML].  The HTML markup MUST be escaped; for
       example, "<br>" as "&lt;br>".  The HTML markup SHOULD be such
       that it could validly appear directly within an HTML <DIV>
       element.  Atom Processors that display the content MAY use the
       markup to aid in displaying it.

   3.  If the value of "type" is "xhtml", the content of atom:content
       MUST be a single XHTML div element [XHTML] and SHOULD be suitable
       for handling as XHTML.  The XHTML div element itself MUST NOT be
       considered part of the content.  Atom Processors that display the
       content MAY use the markup to aid in displaying it.  The escaped
       versions of characters such as "&" and ">" represent those
       characters, not markup.

   4.  If the value of "type" is an XML media type [RFC3023] or ends
       with "+xml" or "/xml" (case insensitive), the content of
       atom:content MAY include child elements and SHOULD be suitable
       for handling as the indicated media type.  If the "src" attribute
       is not provided, this would normally mean that the "atom:content"
       element would contain a single child element that would serve as
       the root element of the XML document of the indicated type.

   5.  If the value of "type" begins with "text/" (case insensitive),
       the content of atom:content MUST NOT contain child elements.

   6.  For all other values of "type", the content of atom:content MUST
       be a valid Base64 encoding, as described in [RFC3548], section 3.
       When decoded, it SHOULD be suitable for handling as the indicated
       media type.  In this case, the characters in the Base64 encoding
       MAY be preceded and followed in the atom:content element by white
       space, and lines are separated by a single newline (U+000A)
       character.
*/

//gsoapopt w

//gsoap a schema namespace:     http://www.w3.org/2005/Atom
//gsoap a schema elementForm:   qualified
//gsoap a schema attributeForm: unqualified

#import "stlvector.h"

typedef std::string xsd__anyURI;

typedef std::string _xml__base;

typedef std::string _xml__lang;

enum a__textType
{
  a__textType__text,	///< xs:token value="text"
  a__textType__html,	///< xs:token value="html"
  a__textType__xhtml,	///< xs:token value="xhtml"
};

class a__uri
{ public:
    xsd__anyURI                          __item                        ;
   @_xml__base*                          xml__base                      0;	///< Optional attribute.
   @_xml__lang*                          xml__lang                      0;	///< Optional attribute.
};

class a__dateTime
{ public:
    time_t                               __item                        ;
   @_xml__base*                          xml__base                      0;	///< Optional attribute.
   @_xml__lang*                          xml__lang                      0;	///< Optional attribute.
};

class a__generator
{ public:
    std::string                          __item                        ;
   @xsd__anyURI*                         uri                            0;	///< Optional attribute.
   @std::string*                         version                        0;	///< Optional attribute.
   @_xml__base*                          xml__base                      0;	///< Optional attribute.
   @_xml__lang*                          xml__lang                      0;	///< Optional attribute.
};

class a__text
{ public:
    _XML                                 __item                        ;
   @enum a__textType*                    type                           0;	///< Optional attribute.
   @_xml__base*                          xml__base                      0;	///< Optional attribute.
   @_xml__lang*                          xml__lang                      0;	///< Optional attribute.
};

class a__person
{ public:
   @_xml__base*                          xml__base                      0;	///< Optional attribute.
   @_xml__lang*                          xml__lang                      0;	///< Optional attribute.
    std::string                          name                           1;	///< Required element.
    a__uri*                              uri                           ;	///< Optional element.
    std::string*                         email                         ;	///< Optional element.
    std::vector<_XML                   > __any                         0;	///< Catch any element content in XML string.
};

class a__content
{ public:
   @std::string*                         type                           0;	///< Optional attribute.
   @xsd__anyURI*                         src                            0;	///< Optional attribute.
   @_xml__base*                          xml__base                      0;	///< Optional attribute.
   @_xml__lang*                          xml__lang                      0;	///< Optional attribute.
    std::vector<_XML                   > __any                         0;	///< Catch any element content in XML string.
};

class a__category
{ public:
   @std::string                          term                           1;	///< Required attribute.
   @xsd__anyURI*                         scheme                         0;	///< Optional attribute.
   @std::string*                         label                          0;	///< Optional attribute.
   @_xml__base*                          xml__base                      0;	///< Optional attribute.
   @_xml__lang*                          xml__lang                      0;	///< Optional attribute.
    std::vector<_XML                   > __any                         0;	///< Catch any element content in XML string.
};

class a__link
{ public:
   @xsd__anyURI                          href                           1;	///< Required attribute.
   @std::string*                         rel                            0;	///< Optional attribute.
   @std::string*                         type                           0;	///< Optional attribute.
   @std::string*                         hreflang                       0;	///< Optional attribute.
   @std::string*                         title                          0;	///< Optional attribute.
   @unsigned int*                        length                         0;	///< Optional attribute.
   @_xml__base*                          xml__base                      0;	///< Optional attribute.
   @_xml__lang*                          xml__lang                      0;	///< Optional attribute.
    std::vector<_XML                   > __any                         0;	///< Catch any element content in XML string.
};

class a__source
{ public:
   @_xml__base*                          xml__base                      0;	///< Optional attribute.
   @_xml__lang*                          xml__lang                      0;	///< Optional attribute.
    std::vector<a__person              > author                         0;	///< Optional element.
    std::vector<a__category            > category                       0;	///< Optional element.
    std::vector<a__person              > contributor                    0;	///< Optional element.
    a__generator*                        generator                      0;	///< Optional element.
    a__uri*                              icon                           0;	///< Optional element.
    a__uri*                              id                             0;	///< Optional element.
    std::vector<a__link                > link                           0;	///< Optional element.
    a__uri*                              logo                           0;	///< Optional element.
    a__text*                             rights                         0;	///< Optional element.
    a__text*                             subtitle                       0;	///< Optional element.
    a__text*                             title                          0;	///< Optional element.
    a__dateTime*                         updated                        0;	///< Optional element.
    std::vector<_XML                   > __any                          0;	///< Catch any element content in XML string.
};

class a__entry
{ public:
   @_xml__base*                          xml__base                      0;	///< Optional attribute.
   @_xml__lang*                          xml__lang                      0;	///< Optional attribute.
    a__uri*                              id                             0;	///< Optional element.
    a__text*                             title                          0;	///< Optional element.
    a__dateTime*                         published                      0;	///< Optional element.
    a__dateTime*                         updated                        0;	///< Optional element.
    a__text*                             rights                         0;	///< Optional element.
    a__text*                             summary                        0;	///< Optional element.
    a__source*                           source                         0;	///< Optional element.
    a__content*                          content                        0;	///< Optional element.
    std::vector<a__category            > category                       0;	///< Optional element.
    std::vector<a__person              > author                         0;	///< Optional element.
    std::vector<a__person              > contributor                    0;	///< Optional element.
    std::vector<a__link                > link                           0;	///< Optional element.
    std::vector<_XML                   > __any                          0;	///< Catch any element content in XML string.
};

class a__feed    
{ public:
   @_xml__base*                          xml__base                      0;	///< Optional attribute.
   @_xml__lang*                          xml__lang                      0;	///< Optional attribute.
    std::vector<
    class __a__feed
    {
    a__uri*                              id                            ;	///< Optional element.
    a__text*                             title                         ;	///< Optional element.
    a__text*                             subtitle                      ;	///< Optional element.
    a__dateTime*                         updated                       ;	///< Optional element.
    a__generator*                        generator                     ;	///< Optional element.
    a__uri*                              icon                          ;	///< Optional element.
    a__uri*                              logo                          ;	///< Optional element.
    a__text*                             rights                        ;	///< Optional element.
    std::vector<a__link                > link                          ;	///< Optional element.
    std::vector<a__person              > author                        ;	///< Optional element.
    std::vector<a__category            > category                      ;	///< Optional element.
    std::vector<a__person              > contributor                   ;	///< Optional element.
    std::vector<a__entry               > entry                         ;	///< Optional element.
    }>                                   __feeds                       ;
    std::vector<_XML                   > __any                         0;	///< Catch any element content in XML string.
};
