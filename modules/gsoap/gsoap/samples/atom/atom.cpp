/*
        atom.cpp
        
        Atom 1.0 example to type-safe populate Atom XML data

        Compile:
        $ soapcpp2 -0 -CSL atom.h
        $ c++ -o atom atom.cpp stdsoap2.cpp soapC.cpp

        Try it out:
        $ ./atom > feed.xml
        $ ./atom feed.xml

        Warning:
        XML content with type _XML (a char* type) in a__text and in all __any
        members in several classes allow the delivery of HTML and XHTML.  Many
        elements in these languages are considered 'unsafe' in that they open
        clients to one or more types of attack.  Implementers of software that
        processes Atom should carefully consider their handling of every type
        of element when processing incoming (X)HTML in Atom Documents.  See the
        security sections of [RFC2854] and [HTML] for guidance.

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

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2001-2017, Robert van Engelen, Genivia, Inc. All Rights Reserved.
This software is released under one of the following two licenses:
GPL.
--------------------------------------------------------------------------------
GPL license.

This program is free software; you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program; if not, write to the Free Software Foundation, Inc., 59 Temple
Place, Suite 330, Boston, MA 02111-1307 USA

Author contact information:
engelen@genivia.com / engelen@acm.org
--------------------------------------------------------------------------------
A commercial use license is available from Genivia, Inc., contact@genivia.com
--------------------------------------------------------------------------------
*/

#include "soapH.h"
#include <fstream>

/// the namespace mapping table has NULL entries for SOAP and XSD namespaces that we do not need
struct Namespace namespaces[] = {
  {"SOAP-ENV", NULL, NULL, NULL},
  {"SOAP-ENC", NULL, NULL, NULL},
  {"xsi", NULL, NULL, NULL},
  {"xsd", NULL, NULL, NULL},
  {"a", "http://www.w3.org/2005/Atom", NULL, NULL},
  {NULL, NULL, NULL, NULL}
};

// Some useful templates and functions to populate Atom data.  More useful
// perhaps is to add a few more functions to populate and extract Atom data, or
// add methods to the a__feed class to populate and extract Atom data.

/// soap_make: populate primitive values, e.g. integers, returns pointer to managed heap-allocated data of type T
template<class T>
T * soap_make(struct soap *soap, T val)
{
  T *p = (T*)soap_malloc(soap, sizeof(T));
  if (p == NULL)
    throw std::bad_alloc();
  *p = val;
  return p;
}

/// soap_make_string: return managed heap-allocated and populated std::string
std::string *soap_make_string(struct soap *soap, const char *s)
{
  if (!s)
    return NULL;
  std::string *p = (std::string*)soap_new_std__string(soap);
  if (p == NULL)
    throw std::bad_alloc();
  *p = s;
  return p;
}

/// soap_make_text: return managed heap-allocated and populated a__text
a__text *soap_make_text(struct soap *soap, const char *s, const char *lang = NULL)
{
  if (!s)
    return NULL;
  a__text *p = (a__text*)soap_new_a__text(soap);
  if (p == NULL)
    throw std::bad_alloc();
  p->__item = soap_strdup(soap, s);
  p->xml__base = NULL;
  p->xml__lang = lang ? soap_make_string(soap, s) : NULL;
  return p;
}

/// soap_make_uri: return managed heap-allocated and populated a__uri
a__uri *soap_make_uri(struct soap *soap, const char *s, const char *lang = NULL)
{
  if (!s)
    return NULL;
  a__uri *p = (a__uri*)soap_new_a__uri(soap);
  if (p == NULL)
    throw std::bad_alloc();
  p->__item = s;
  p->xml__base = NULL;
  p->xml__lang = lang ? soap_make_string(soap, s) : NULL;
  return p;
}

/// soap_make_dateTime: return managed heap-allocated and populated a__dateTime
a__dateTime *soap_make_now(struct soap *soap)
{
  a__dateTime *p = (a__dateTime*)soap_new_a__dateTime(soap);
  if (p == NULL)
    throw std::bad_alloc();
  p->__item = time(NULL);
  p->xml__base = NULL;
  p->xml__lang = NULL;
  return p;
}

/// soap_make_gen: return managed heap-allocated and populated a__generator
a__generator *soap_make_gen(struct soap *soap, const char *s, const char *u = NULL, const char *v = NULL, const char *lang = NULL)
{
  if (!s)
    return NULL;
  a__generator *p = (a__generator*)soap_new_a__generator(soap);
  if (p == NULL)
    throw std::bad_alloc();
  p->__item = s;
  p->uri = u ? soap_make_string(soap, u) : NULL;
  p->version = v ? soap_make_string(soap, v) : NULL;
  p->xml__base = NULL;
  p->xml__lang = lang ? soap_make_string(soap, s) : NULL;
  return p;
}

/// display text
void show_text(const char *label, const a__text *text)
{
  if (text)
  {
    std::cout << label << text->__item;
    if (text->xml__lang)
      std::cout << " lang=" << *text->xml__lang;
    if (text->type)
    {
      // we could have used soap_a__textType2s(soap, *text->type) to display the type
      switch (*text->type)
      {
        case a__textType__text:
          std::cout << " (text)";
          break;
        case a__textType__html:
          std::cout << " (HTML)";
          break;
        case a__textType__xhtml:
          std::cout << " (XHTML)";
          break;
      }
    }
    std::cout << std::endl;
  }
}

/// display URI
void show_uri(const char *label, const a__uri *uri)
{
  if (uri)
  {
    std::cout << label << uri->__item;
    if (uri->xml__lang)
      std::cout << " lang=" << *uri->xml__lang;
    std::cout << std::endl;
  }
}

/// display dateTime (not MT-safe because of clib ctime, use ctime_r instead)
void show_date(const char *label, const a__dateTime *date)
{
  if (date)
    std::cout << label << ctime(&date->__item);
}

/// display generator
void show_gen(const char *label, const a__generator *gen)
{
  if (gen)
  {
    std::cout << label << gen->__item;
    if (gen->uri)
      std::cout << " URI=" << *gen->uri;
    if (gen->version)
      std::cout << " version=" << *gen->version;
    if (gen->xml__lang)
      std::cout << " lang=" << *gen->xml__lang;
    std::cout << std::endl;
  }
}

/// display person
void show_pers(const char *label, const a__person& pers)
{
  std::cout << label << pers.name;
  if (pers.email)
    std::cout << " email=" << *pers.email;
  if (pers.xml__lang)
    std::cout << " lang=" << *pers.xml__lang;
  show_uri(" uri=", pers.uri);
  for (std::vector<_XML>::const_iterator i = pers.__any.begin(); i != pers.__any.end(); ++i)
    std::cout << std::endl << *i;
  std::cout << std::endl;
}

/// display content
void show_cont(const char *label, const a__content *cont)
{
  if (cont)
  {
    std::cout << label;
    if (cont->type)
      std::cout << " type=" << *cont->type;
    if (cont->src)
      std::cout << " src=" << *cont->src;
    if (cont->xml__lang)
      std::cout << " lang=" << *cont->xml__lang;
    for (std::vector<_XML>::const_iterator i = cont->__any.begin(); i != cont->__any.end(); ++i)
      std::cout << std::endl << *i;
    std::cout << std::endl;
  }
}

/// display category
void show_caty(const char *label, const a__category& caty)
{
  std::cout << label << caty.term;
  if (caty.scheme)
    std::cout << " scheme=" << *caty.scheme;
  if (caty.label)
    std::cout << " label=" << *caty.label;
  if (caty.xml__lang)
    std::cout << " lang=" << *caty.xml__lang;
  for (std::vector<_XML>::const_iterator i = caty.__any.begin(); i != caty.__any.end(); ++i)
    std::cout << std::endl << *i;
  std::cout << std::endl;
}

/// display link
void show_link(const char *label, const a__link& link)
{
  std::cout << label << link.href;
  if (link.rel)
    std::cout << " rel=" << *link.rel;
  if (link.type)
    std::cout << " type=" << *link.type;
  if (link.hreflang)
    std::cout << " hreflang=" << *link.hreflang;
  if (link.title)
    std::cout << " title=" << *link.title;
  if (link.length)
    std::cout << " length=" << *link.length;
  if (link.xml__lang)
    std::cout << " lang=" << *link.xml__lang;
  std::cout << std::endl;
}

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    /*
       The example code below shows how to populate in a type-safe way the
       following Atom feed:

        <?xml version="1.0" encoding="utf-8"?>

        <feed xmlns="http://www.w3.org/2005/Atom">

                <title>Example Feed</title>
                <subtitle>A subtitle.</subtitle>
                <link href="http://example.org/feed/" rel="self" />
                <link href="http://example.org/" />
                <id>urn:uuid:60a76c80-d399-11d9-b91C-0003939e0af6</id>
                <updated>2003-12-13T18:30:02Z</updated>
                
                <entry>
                        <title>Atom-Powered Robots Run Amok</title>
                        <link href="http://example.org/2003/12/13/atom03" />
                        <link rel="alternate" type="text/html" href="http://example.org/2003/12/13/atom03.html"/>
                        <link rel="edit" href="http://example.org/2003/12/13/atom03/edit"/>
                        <id>urn:uuid:1225c695-cfb8-4ebb-aaaa-80da344efa6a</id>
                        <updated>2003-12-13T18:30:02Z</updated>
                        <summary>Some text.</summary>
                        <content type="xhtml">
                                <div xmlns="http://www.w3.org/1999/xhtml">
                                        <p>This is the entry content.</p>
                                </div>
                        </content>
                        <author>
                                <name>John Doe</name>
                                <email>johndoe@example.com</email>
                        </author>
                </entry>

        </feed>
    */
    struct soap *soap = soap_new1(SOAP_XML_INDENT | SOAP_XML_DEFAULTNS);
    a__feed    feeds;
    __a__feed  feed;
    a__link    link;
    a__entry   entry;
    a__person  person;

    /* create a feed */
    feed.id       = soap_make_uri(soap, soap_rand_uuid(soap, "urn:uuid:"));
    feed.title    = soap_make_text(soap, "Example Feed");
    feed.subtitle = soap_make_text(soap, "A subtitle.");
    feed.updated  = soap_make_now(soap);
    /* feed/link */
    link.href     = "http://example.org/feed/";
    link.rel      = soap_make_string(soap, "self");
    feed.link.push_back(link);
    /* feed/link */
    link.href     = "http://example.org/";
    link.rel      = NULL;
    feed.link.push_back(link);

    /* create an entry in the feed with three links */
    entry.id      = soap_make_uri(soap, soap_rand_uuid(soap, "urn:uuid:"));
    entry.title   = soap_make_text(soap, "Atom-Powered Robots Run Amok");
    entry.updated = soap_make_now(soap);
    entry.summary = soap_make_text(soap, "Some text.");
    link.href     = "http://example.org/2003/12/13/atom03";
    link.rel      = NULL;
    link.type     = NULL;
    entry.link.push_back(link);
    link.href     = "http://example.org/2003/12/13/atom03.html";
    link.rel      = soap_make_string(soap, "alternate");
    link.type     = soap_make_string(soap, "text/html");
    entry.link.push_back(link);
    link.href     = "http://example.org/2003/12/13/atom03/edit";
    link.rel      = soap_make_string(soap, "edit");
    link.type     = NULL;
    entry.link.push_back(link);

    /* create content for the entry */
    entry.content = soap_new_a__content(soap);
    entry.content->type = soap_make_string(soap, "xhtml");
    entry.content->__any.push_back((char*)"\n<div xmlns=\"http://www.w3.org/1999/xhtml\">\n  <p>This is the entry content.</p>\n</div>\n");

    /* create an author for the entry */
    person.name   = "John Doe";
    person.email  = soap_make_string(soap, "johndoe@example.com");
    entry.author.push_back(person);

    /* add entry to feed/entry */
    feed.entry.push_back(entry);

    /* add feed */
    feeds.__feeds.push_back(feed);

    /* display XML */
    soap_write_a__feed(soap, &feeds);

    /* demo is done */
    soap_destroy(soap);
    soap_end(soap);
    soap_free(soap);
  }
  else
  {
    /* parse the atom file given on the command line, deserialized to C++, and display contents */
    struct soap *soap = soap_new();

    a__feed    feeds;

    std::ifstream in(argv[1]);
    soap->is = &in;
    if (soap_read_a__feed(soap, &feeds))
    {
      soap_stream_fault(soap, std::cerr);
      exit(EXIT_FAILURE);
    }
    in.close();
    soap->is = NULL;

    for (std::vector<__a__feed>::const_iterator i = feeds.__feeds.begin(); i != feeds.__feeds.end(); ++i)
    {
      show_text("title:     ", i->title);
      show_text("subtitle:  ", i->subtitle);
      show_date("updated:   ", i->updated);
      show_gen ("generator: ", i->generator);
      show_uri ("icon:      ", i->icon);
      show_uri ("logo:      ", i->logo);
      show_text("rights:    ", i->rights);

      for (std::vector<a__link>::const_iterator j = i->link.begin(); j != i->link.end(); ++j)
        show_link("link:      ", *j);

      for (std::vector<a__entry>::const_iterator j = i->entry.begin(); j != i->entry.end(); ++j)
      {
        std::cout << std::endl;
        show_text("  title:       ", j->title);
        show_date("  published:   ", j->updated);
        show_date("  updated:     ", j->updated);
        show_text("  rights:      ", j->rights);
        show_text("  summary:     ", j->summary);
        show_cont("  content:    ",  j->content);
        // skipped source intentionally: you need to add code to display the source element

        for (std::vector<a__category>::const_iterator k = j->category.begin(); k != j->category.end(); ++k)
          show_caty("  category:    ", *k);

        for (std::vector<a__person>::const_iterator k = j->author.begin(); k != j->author.end(); ++k)
          show_pers("  author:      ", *k);

        for (std::vector<a__person>::const_iterator k = j->contributor.begin(); k != j->contributor.end(); ++k)
          show_pers("  contributor: ", *k);

        for (std::vector<a__link>::const_iterator k = j->link.begin(); k != j->link.end(); ++k)
          show_link("  link:        ", *k);
      }
    }

    soap_destroy(soap);
    soap_end(soap);
    soap_free(soap);
  }
  return 0;
}
