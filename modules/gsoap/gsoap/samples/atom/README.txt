
Atom
====

The gSOAP Atom library supports Atom 1.0

Compile:
$ soapcpp2 -0 -CSL atom.h
$ c++ -o atom atom.cpp stdsoap2.cpp soapC.cpp

Try it out:
$ ./atom > feed.xml
$ ./atom feed.xml

Warning:
XML content with type _XML (a char* type) in a__text and in all __any members
in several classes allow the delivery of HTML and XHTML.  Many elements in
these languages are considered 'unsafe' in that they open clients to one or
more types of attack.  Implementers of software that processes Atom should
carefully consider their handling of every type of element when processing
incoming (X)HTML in Atom Documents.  See the security sections of [RFC2854] and
[HTML] for guidance.

See [RFC4287] https://tools.ietf.org/html/rfc4287 for details.

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
