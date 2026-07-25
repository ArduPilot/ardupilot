/*
        schema.cpp

        XSD binding schema implementation

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2000-2020, Robert van Engelen, Genivia Inc. All Rights Reserved.
This software is released under one of the following licenses:
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

#include "wsdlH.h"              // cannot include "schemaH.h"
#include "includes.h"

extern struct Namespace namespaces[];

extern "C" {
extern int warn_ignore(struct soap*, const char*);
}

extern const char *qname_token(const char*, const char*);
extern int is_builtin_qname(const char*);
extern xsd__QName make_qname(xs__schema&, const char *);

////////////////////////////////////////////////////////////////////////////////
//
//      schema
//
////////////////////////////////////////////////////////////////////////////////

xs__schema::xs__schema()
{
  soap = soap_new1(SOAP_XML_TREE | SOAP_C_UTFSTRING);
#ifdef HTTPDA_H
  soap_register_plugin(soap, http_da);
#endif
#ifdef WITH_OPENSSL
  soap_ssl_client_context(soap, SOAP_SSL_NO_AUTHENTICATION | SOAP_SSLv3_TLSv1, NULL, NULL, NULL, NULL, NULL);
#endif
  soap_set_namespaces(soap, namespaces);
  soap_default(soap);
  soap->fignore = warn_ignore;
  soap->encodingStyle = NULL;
  soap->proxy_host = proxy_host;
  soap->proxy_port = proxy_port;
  soap->proxy_userid = proxy_userid;
  soap->proxy_passwd = proxy_passwd;
  targetNamespace = NULL;
  version = NULL;
  attributeGroupRef = NULL;
  updated = false;
  location = NULL;
  redirs = 0;
  used = false;
}

xs__schema::xs__schema(struct soap *copy)
{
  soap = soap_copy(copy);
  soap->socket = SOAP_INVALID_SOCKET;
  soap->recvfd = 0;
  soap->sendfd = 1;
  soap_default(soap);
  soap->fignore = warn_ignore;
  soap->encodingStyle = NULL;
  targetNamespace = NULL;
  version = NULL;
  attributeGroupRef = NULL;
  updated = false;
  location = NULL;
  redirs = 0;
}

xs__schema::xs__schema(struct soap *copy, const char *cwd, const char *loc, const char *relloc)
{
  soap = soap_copy(copy);
  soap->socket = SOAP_INVALID_SOCKET;
  soap->recvfd = 0;
  soap->sendfd = 1;
  soap_default(soap);
  soap->fignore = warn_ignore;
  soap->encodingStyle = NULL;
  targetNamespace = NULL;
  version = NULL;
  attributeGroupRef = NULL;
  updated = false;
  location = NULL;
  redirs = 0;
  read(cwd, loc, relloc);
}

xs__schema::~xs__schema()
{ }

int xs__schema::get(struct soap *soap)
{
  (void)soap;
  return preprocess();
}

int xs__schema::preprocess()
{
  for (std::vector<xs__import>::iterator im = import.begin(); im != import.end(); ++im)
    (*im).preprocess(*this); // read schema and recurse over <import>
  for (std::vector<xs__include>::iterator in = include.begin(); in != include.end(); ++in)
  {
    (*in).preprocess(*this); // read schema and recurse over <include>, <override> and <redefine>
    if ((*in).schemaPtr())
      insert(*(*in).schemaPtr());
  }
  for (std::vector<xs__override>::iterator ov = override_.begin(); ov != override_.end(); ++ov)
  {
    (*ov).preprocess(*this); // read schema and recurse over <include>, <override> and <redefine>
    if ((*ov).schemaPtr())
      insert(*(*ov).schemaPtr());
  }
  for (std::vector<xs__redefine>::iterator re = redefine.begin(); re != redefine.end(); ++re)
  {
    (*re).preprocess(*this); // read schema and recurse over <redefine>, <override> and <redefine>
    if ((*re).schemaPtr())
      insert(*(*re).schemaPtr());
  }
  return SOAP_OK;
}

int xs__schema::insert(xs__schema& schema)
{
  bool found;
  if (targetNamespace && (!schema.targetNamespace || strcmp(targetNamespace, schema.targetNamespace)))
  {
    if (!Wflag)
      fprintf(stderr, "\nWarning: attempt to include schema '%s' with mismatching targetNamespace '%s' into schema namespace '%s', assuming chameleon schema targetNamespace '%s'\n", schema.sourceLocation() ? schema.sourceLocation() : "", schema.targetNamespace ? schema.targetNamespace : "(undefined)", targetNamespace ? targetNamespace : "(undefined)", targetNamespace ? targetNamespace : "(undefined)");
    schema.targetNamespace = targetNamespace;
  }
  if (elementFormDefault != schema.elementFormDefault)
  {
    if (!Wflag)
      fprintf(stderr, "\nWarning: attempt to include schema '%s' with mismatching elementFormDefault into schema namespace '%s', assuming elementFormDefault '%squalified'\n", schema.sourceLocation() ? schema.sourceLocation() : "", targetNamespace ? targetNamespace : "(undefined)", elementFormDefault == qualified ? "" : "un");
    schema.elementFormDefault = elementFormDefault;
  }
  if (attributeFormDefault != schema.attributeFormDefault)
  {
    if (!Wflag)
      fprintf(stderr, "\nWarning: attempt to include schema '%s' with mismatching attributeFormDefault into schema namespace '%s', assuming attributeFormDefault '%squalified'\n", schema.sourceLocation() ? schema.sourceLocation() : "", targetNamespace ? targetNamespace : "(undefined)", attributeFormDefault == qualified ? "" : "un");
    schema.attributeFormDefault = attributeFormDefault;
  }
  // insert imports
  for (std::vector<xs__import>::const_iterator im = schema.import.begin(); im != schema.import.end(); ++im)
  {
    found = false;
    if ((*im).schemaLocation)
    {
      for (std::vector<xs__import>::const_iterator i = import.begin(); i != import.end(); ++i)
      {
        if ((*i).schemaLocation && !strcmp((*im).schemaLocation, (*i).schemaLocation))
        {
          found = true;
          break;
        }
      }
    }
    else if ((*im).namespace_)
    {
      for (std::vector<xs__import>::const_iterator i = import.begin(); i != import.end(); ++i)
      {
        if ((*i).namespace_ && !strcmp((*im).namespace_, (*i).namespace_))
        {
          found = true;
          break;
        }
      }
    }
    if (!found)
      import.push_back(*im);
  }
  // insert attributes, but only add attributes with new name (limited conflict check)
  for (std::vector<xs__attribute>::const_iterator at = schema.attribute.begin(); at != schema.attribute.end(); ++at)
  {
    found = false;
    if ((*at).name)
    {
      for (std::vector<xs__attribute>::const_iterator a = attribute.begin(); a != attribute.end(); ++a)
      {
        if ((*a).name && !strcmp((*at).name, (*a).name))
        {
          found = true;
          if ((*at).type && (*a).type && strcmp((*at).type, (*a).type))
            if (!Wflag)
              fprintf(stderr, "\nWarning: attempt to redefine attribute '%s' with type '%s' in schema '%s'\n", (*at).name, (*at).type, targetNamespace ? targetNamespace : "(undefined)");
          break;
        }
      }
    }
    if (!found)
    {
      attribute.push_back(*at);
      attribute.back().schemaPtr(this);
    }
  }
  // insert elements, but only add elements with new name (limited conflict check)
  for (std::vector<xs__element>::const_iterator el = schema.element.begin(); el != schema.element.end(); ++el)
  {
    found = false;
    if ((*el).name)
    {
      for (std::vector<xs__element>::const_iterator e = element.begin(); e != element.end(); ++e)
      {
        if ((*e).name && !strcmp((*el).name, (*e).name))
        {
          found = true;
          if ((*el).type && (*e).type && strcmp((*el).type, (*e).type))
            if (!Wflag)
              fprintf(stderr, "\nWarning: attempt to redefine element '%s' with type '%s' in schema '%s'\n", (*el).name, (*el).type, targetNamespace ? targetNamespace : "(undefined)");
          break;
        }
      }
    }
    if (!found)
    {
      element.push_back(*el);
      element.back().schemaPtr(this);
    }
  }
  // insert groups, but only add groups with new name (no conflict warning)
  for (std::vector<xs__group>::const_iterator gp = schema.group.begin(); gp != schema.group.end(); ++gp)
  {
    found = false;
    if ((*gp).name)
    {
      for (std::vector<xs__group>::const_iterator g = group.begin(); g != group.end(); ++g)
      {
        if ((*g).name && !strcmp((*gp).name, (*g).name))
        {
          found = true;
          break;
        }
      }
    }
    if (!found)
    {
      group.push_back(*gp);
      group.back().schemaPtr(this);
    }
  }
  // insert attributeGroups, but only add attributeGroups with new name (no conflict warning)
  for (std::vector<xs__attributeGroup>::const_iterator ag = schema.attributeGroup.begin(); ag != schema.attributeGroup.end(); ++ag)
  {
    found = false;
    if ((*ag).name)
    {
      for (std::vector<xs__attributeGroup>::const_iterator g = attributeGroup.begin(); g != attributeGroup.end(); ++g)
      {
        if ((*g).name && !strcmp((*ag).name, (*g).name))
        {
          found = true;
          break;
        }
      }
    }
    if (!found)
    {
      attributeGroup.push_back(*ag);
      attributeGroup.back().schemaPtr(this);
    }
  }
  // insert simpleTypes, but only add simpleTypes with new name (no conflict warning)
  for (std::vector<xs__simpleType>::const_iterator st = schema.simpleType.begin(); st != schema.simpleType.end(); ++st)
  {
    found = false;
    if ((*st).name)
    {
      for (std::vector<xs__simpleType>::const_iterator s = simpleType.begin(); s != simpleType.end(); ++s)
      {
        if ((*s).name && !strcmp((*st).name, (*s).name))
        {
          found = true;
          break;
        }
      }
    }
    if (!found)
    {
      simpleType.push_back(*st);
      simpleType.back().schemaPtr(this);
    }
  }
  // insert complexTypes, but only add complexTypes with new name (no conflict warning)
  for (std::vector<xs__complexType>::const_iterator ct = schema.complexType.begin(); ct != schema.complexType.end(); ++ct)
  {
    found = false;
    if ((*ct).name)
    {
      for (std::vector<xs__complexType>::const_iterator c = complexType.begin(); c != complexType.end(); ++c)
      {
        if ((*c).name && !strcmp((*ct).name, (*c).name))
        {
          found = true;
          break;
        }
      }
    }
    if (!found)
    {
      complexType.push_back(*ct);
      complexType.back().schemaPtr(this);
    }
  }
  return SOAP_OK;
}

int xs__schema::traverse()
{
  if (updated)
    return SOAP_OK;
  if (vflag)
    std::cerr << "  Analyzing schema '" << (targetNamespace ? targetNamespace : "(null)") << "' '" << (sourceLocation() ? sourceLocation() : "") << "'" << std::endl;
  updated = true;
  if (!targetNamespace)
  {
    if (vflag)
      fprintf(stderr, "\nWarning: Schema has no targetNamespace\n");
    targetNamespace = (char*)"";
  }
  else if (exturis.find(targetNamespace) != exturis.end())
  {
    if (vflag)
      fprintf(stderr, "\nWarning: Built-in schema '%s' content encountered\n", targetNamespace);
  }
  // process import
  for (std::vector<xs__import>::iterator im = import.begin(); im != import.end(); ++im)
    (*im).traverse(*this);
  // process attributes
  for (std::vector<xs__attribute>::iterator at = attribute.begin(); at != attribute.end(); ++at)
    (*at).traverse(*this);
  // process elements
  for (std::vector<xs__element>::iterator el = element.begin(); el != element.end(); 
++el)
    (*el).traverse(*this);
  // process simpleTypes, check conflicts with complexTypes
  for (std::vector<xs__simpleType>::iterator st = simpleType.begin(); st != simpleType.end(); ++st)
  {
    (*st).traverse(*this);
    if ((*st).name)
    {
      for (std::vector<xs__complexType>::iterator ct = complexType.begin(); ct != complexType.end(); ++ct)
      {
        if ((*ct).name && !strcmp((*st).name, (*ct).name))
        {
          if (!Wflag)
            fprintf(stderr, "\nWarning: top-level simpleType name and complexType name '%s' clash in schema '%s'\n", (*st).name, targetNamespace ? targetNamespace : "(undefined)");
        }
      }
    }
  }
  // process complexTypes
  for (std::vector<xs__complexType>::iterator ct = complexType.begin(); ct != complexType.end(); ++ct)
    (*ct).traverse(*this);
  // process groups
  for (std::vector<xs__group>::iterator gp = group.begin(); gp != group.end(); ++gp)
    (*gp).traverse(*this);
  // process attributeGroups
  for (std::vector<xs__attributeGroup>::iterator ag = attributeGroup.begin(); ag != attributeGroup.end(); ++ag)
    (*ag).traverse(*this);
  // XSD 1.1 defaultAttributes
  if (defaultAttributes)
  {
    for (std::vector<xs__attributeGroup>::iterator ag = attributeGroup.begin(); ag != attributeGroup.end(); ++ag)
    {
      if ((*ag).name && !strcmp((*ag).name, defaultAttributes))
      {
        attributeGroupRef = &*ag;
        break;
      }
    }
    if (!attributeGroupRef)
      if (!Wflag)
        std::cerr << "\nWarning: could not find defaultAttributes attributeGroup '" << defaultAttributes << "' in schema '" << (targetNamespace ? targetNamespace : "(undefined)") << "'" << std::endl;
  }
  if (vflag)
    std::cerr << "  End of schema '" << (targetNamespace ? targetNamespace : "(undefined)") << "'" << std::endl;
  return SOAP_OK;
}

int xs__schema::read(const char *cwd, const char *loc, const char *relloc)
{
  const char *cwd_temp;
  if (!cwd)
    cwd = cwd_path;
  if (vflag)
    fprintf(stderr, "\nOpening schema '%s' relative to '%s'\n", loc ? loc : "(stdin)", cwd ? cwd : "./");
  if (loc)
  {
    if (soap->recvfd > 2)
    {
      soap_end_recv(soap);
      close(soap->recvfd);
      soap->recvfd = -1;
    }
    else if (soap_valid_socket(soap->socket))
    {
      soap_end_recv(soap);
      soap_closesock(soap);
    }
#ifdef WITH_OPENSSL
    if (!strncmp(loc, "http://", 7) || !strncmp(loc, "https://", 8))
#else
    if (!strncmp(loc, "https://", 8))
    {
#ifdef WIN32
      fprintf(stderr, "\nCannot connect to https site: SSL/TLS support not enabled in this version. Visit https://www.genivia.com/downloads.html to download the secure version of wsdl2h.exe that supports SSL/TLS to connect to https sites.\n");
#else
      fprintf(stderr, "\nCannot connect to https site: SSL/TLS support not enabled, please rebuild wsdl2h with SSL/TLS enabled using 'make secure' or download the WSDL/WADL and XSD files and rerun wsdl2h on these files directly by specifying the file names on the command line.\n");
#endif
      exit(1);
    }
    else if (!strncmp(loc, "http://", 7))
#endif
    {
      fprintf(stderr, "%*sConnecting to '%s' to retrieve schema...", 2*openfiles, "", loc);
      location = soap_strdup(soap, loc);
      if (soap_connect_command(soap, SOAP_GET, location, NULL))
      {
        fprintf(stderr, "\n\nError: connection failed\n");
        soap_print_fault(soap, stderr);
        exit(1);
      }
      fprintf(stderr, " connected, receiving...\n");
      openfiles++;
    }
    else if (cwd && (!strncmp(cwd, "http://", 7) || !strncmp(cwd, "https://", 8)))
    {
      size_t l = strlen(cwd) + strlen(loc);
      location = (char*)soap_malloc(soap, l + 2);
      soap_strcpy(location, l + 2, cwd);
      char *s = strrchr(location, '/');
      if (s)
        *s = '\0';
      size_t n = strlen(location);
      soap_strcpy(location + n, l + 2 - n, "/");
      ++n;
      soap_strcpy(location + n, l + 2 - n, loc);
      fprintf(stderr, "%*sConnecting to '%s' to retrieve schema '%s'...", 2*openfiles, "", location, loc);
      if (soap_connect_command(soap, SOAP_GET, location, NULL))
      {
        fprintf(stderr, "\n\nError: failed to retrieve '%s'\n", loc);
        soap_print_fault(soap, stderr);
        exit(1);
      }
      fprintf(stderr, " connected, receiving...\n");
      openfiles++;
    }
    else
    {
      if (!strncmp(loc, "file://", 7))
        loc += 7;
      soap->recvfd = open(loc, O_RDONLY, 0);
      if (soap->recvfd < 0)
      {
        if (loc && cwd)
        {
          size_t l = strlen(cwd) + strlen(relloc);
          location = (char*)soap_malloc(soap, l + 2);
          soap_strcpy(location, l + 2, cwd);
          char *s = strrchr(location, '/');
#ifdef WIN32
          if (!s)
            s = strrchr(location, '\\');
#endif
          if (s)
          {
            *s = '\0';
            size_t n = strlen(location);
            soap_strcpy(location + n, l + 2 - n, "/");
            ++n;
            soap_strcpy(location + n, l + 2 - n, relloc);
            if (!strncmp(location, "file://", 7))
              location += 7;
            soap->recvfd = open(location, O_RDONLY, 0);
            if (vflag)
              std::cerr << "Opening file " << location << (soap->recvfd < 0 ? " failed" : " successful") << std::endl;
          }
        }
        if (soap->recvfd < 0 && import_path)
        {
          size_t l = strlen(import_path) + strlen(loc);
          location = (char*)soap_malloc(soap, l + 2);
          soap_strcpy(location, l + 2, import_path);
          size_t n = strlen(location);
          soap_strcpy(location + n, l + 2 - n, "/");
          ++n;
          soap_strcpy(location + n, l + 2 - n, loc);
          if (!strncmp(location, "file://", 7))
            location += 7;
          soap->recvfd = open(location, O_RDONLY, 0);
          if (vflag)
            std::cerr << "Opening file " << location << (soap->recvfd < 0 ? " failed" : " successful") << std::endl;
        }
        if (relloc && soap->recvfd < 0 && import_path)
        {
          size_t l = strlen(import_path) + strlen(relloc);
          location = (char*)soap_malloc(soap, l + 2);
          soap_strcpy(location, l + 2, import_path);
          size_t n = strlen(location);
          soap_strcpy(location + n, l + 2 - n, "/");
          ++n;
          soap_strcpy(location + n, l + 2 - n, relloc);
          if (!strncmp(location, "file://", 7))
            location += 7;
          soap->recvfd = open(location, O_RDONLY, 0);
          if (vflag)
            std::cerr << "Opening file " << location << (soap->recvfd < 0 ? " failed" : " successful") << std::endl;
        }
        if (soap->recvfd < 0)
        {
          fprintf(stderr, "\nCannot open '%s' to retrieve schema\n", loc);
          exit(1);
        }
      }
      else
      {
        location = soap_strdup(soap, loc);
      }
      fprintf(stderr, "%*sReading schema '%s'...\n", 2*openfiles, "", location);
      openfiles++;
    }
  }
  cwd_temp = cwd_path;
  cwd_path = location;
  if (!soap_begin_recv(soap))
    this->soap_in(soap, "xs:schema", NULL);
  if ((soap->error >= 301 && soap->error <= 303) || soap->error == 307) // HTTP redirect, socket was closed
  {
    int r = SOAP_ERR;
    fprintf(stderr, "Redirected to '%s'...\n", soap->endpoint);
    if (redirs++ < 10)
      r = read(cwd, soap->endpoint, NULL);
    else
      fprintf(stderr, "\nMax redirects exceeded\n");
    redirs--;
    return r;
  }
  else if (soap->error == 401)
  {
    int r = SOAP_ERR;
    fprintf(stderr, "Authenticating to '%s' realm '%s'...\n", loc, soap->authrealm);
    if (auth_userid && auth_passwd && redirs++ < 1)
    { 
#ifdef HTTPDA_H
      struct http_da_info info;
      http_da_save(soap, &info, soap->authrealm, auth_userid, auth_passwd);
#else
      soap->userid = auth_userid;
      soap->passwd = auth_passwd;
#endif
      r = read(cwd, loc, NULL);
#ifdef HTTPDA_H
      http_da_release(soap, &info);
#endif
      redirs--;
    }
    else
    {
      fprintf(stderr, "Authentication failed, use option -r:uid:pwd and (re)build with OpenSSL to enable digest authentication\n");
    }
    return r;
  }
  if (soap->error)
  {
    fprintf(stderr, "\nAn error occurred while parsing schema from '%s'\n", loc ? loc : "(stdin)");
    soap_print_fault(soap, stderr);
    if (soap->error < 200)
      soap_print_fault_location(soap, stderr);
    fprintf(stderr, "\nIf this schema namespace is considered \"built-in\", then add\n  namespaceprefix = <namespaceURI>\nto typemap.dat.\n");
    exit(1);
  }
  openfiles--;
  fprintf(stderr, "%*sDone reading '%s'\n", 2*openfiles, "", loc ? loc : "(stdin)");
  soap_end_recv(soap);
  if (soap->recvfd > 2)
  {
    close(soap->recvfd);
    soap->recvfd = -1;
  }
  else
  {
    soap_closesock(soap);
  }
  cwd_path = cwd_temp;
  return SOAP_OK;
}

void xs__schema::sourceLocation(const char *loc)
{
  location = soap_strdup(soap, loc);
}

const char *xs__schema::sourceLocation()
{
  return location;
}

char *xs__schema::absoluteLocation(const char *loc) const
{
  const char *base = location ? location : cwd_path;
  if (!base)
    return soap_strdup(soap, loc);
  if (!strncmp(loc, "http://", 7) || !strncmp(loc, "https://", 8))
    return soap_strdup(soap, loc);
  if (!strncmp(loc, "file://", 7))
    loc += 7;
  const char *s = strrchr(base, '/');
#ifdef WIN32
  while (!strncmp(loc, "./", 2) || !strncmp(loc, ".\\", 2))
    loc += 2;
  const char *t = strrchr(base, '\\');
  if (!s || s < t)
    s = t;
  if (!s)
    return soap_strdup(soap, loc);
  while (true)
  {
    if ((!strncmp(loc, "../", 3) || !strncmp(loc, "..\\", 3)) && s > base)
    {
      while (--s >= base)
      {
        if (*s == '/' || *s == '\\')
        {
          if (s[1] != '.')
            break;
          if (s[2] == '.' && (s[3] == '/' || s[3] == '\\'))
          {
            s += 3;
            break;
          }
        }
      }
      loc += 3;
    }
    else if (!strncmp(loc, "./", 2) || !strncmp(loc, ".\\", 2))
    {
      loc += 2;
    }
    else
    {
      break;
    }
  }
#else
  while (!strncmp(loc, "./", 2))
    loc += 2;
  if (!s)
    return soap_strdup(soap, loc);
  while (true)
  {
    if (!strncmp(loc, "../", 3) && s > base)
    {
      while (--s >= base)
      {
        if (*s == '/')
        {
          if (s[1] != '.')
            break;
          if (s[2] == '.' && s[3] == '/')
          {
            s += 3;
            break;
          }
        }
      }
      loc += 3;
    }
    else if (!strncmp(loc, "./", 2))
    {
      loc += 2;
    }
    else
    {
      break;
    }
  }
#endif
  size_t n = s - base + 1;
  size_t l = n + strlen(loc);
  char *abs = (char*)soap_malloc(soap, l + 1);
  soap_strncpy(abs, l + 1, base, n);
  soap_strcpy(abs + n, l + 1 - n, loc);
  return abs;
}

xs__attributeGroup *xs__schema::attributeGroupPtr() const
{
  return attributeGroupRef;
}

int xs__schema::error()
{
  return soap->error;
}

void xs__schema::print_fault()
{
  soap_print_fault(soap, stderr);
  if (soap->error < 200)
    soap_print_fault_location(soap, stderr);
}

void xs__schema::builtinType(const char *type)
{
  builtinTypeSet.insert(type);
}

void xs__schema::builtinTypeDerivation(xs__schema& schema, const char *base, const char *derived)
{
  builtinTypeMap[make_qname(schema, derived)] = base;
}

void xs__schema::builtinElement(const char *element)
{
  builtinElementSet.insert(element);
}

void xs__schema::builtinAttribute(const char *attribute)
{
  builtinAttributeSet.insert(attribute);
}

const SetOfString& xs__schema::builtinTypes() const
{
  return builtinTypeSet;
}

const MapOfStringToString& xs__schema::builtinTypeDerivations() const
{
  return builtinTypeMap;
}

const SetOfString& xs__schema::builtinElements() const
{
  return builtinElementSet;
}

const SetOfString& xs__schema::builtinAttributes() const
{
  return builtinAttributeSet;
}

bool xs__schema::empty() const
{
  return include.empty() && redefine.empty() && override_.empty() && attribute.empty() && element.empty() && group.empty() && attributeGroup.empty() && simpleType.empty() && complexType.empty(); // empty except for <xs:import>
}

void xs__schema::mark()
{
  if (Oflag > 1 && !used)
  {
    used = true;
    for (std::vector<xs__import>::iterator im = import.begin(); im != import.end(); ++im)
      (*im).mark();
    // -O2: start with root of usage: use top-level attributes
    if (Oflag < 3)
      for (std::vector<xs__attribute>::iterator at = attribute.begin(); at != attribute.end(); ++at)
        (*at).mark();
    // -O2 and -O3: start with root of usage: use top-level elements
    if (Oflag < 4)
      for (std::vector<xs__element>::iterator el = element.begin(); el != element.end(); ++el)
        (*el).mark();
  }
}

xs__include::xs__include()
{
  schemaLocation = NULL;
  schemaRef = NULL;
}

int xs__include::preprocess(xs__schema &schema)
{
  if (!schemaRef)
  {
    if (schemaLocation)
    {
      // only read from include locations not read already, uses static std::map
      static std::map<const char*, xs__schema*, ltstr> included;
      std::map<const char*, xs__schema*, ltstr>::iterator i = included.end();
      const char *relative_schemaLocation = soap_strdup(schema.soap, schemaLocation);
      schemaLocation = schema.absoluteLocation(schemaLocation);
      if (!zflag || zflag > 10)
      {
        for (i = included.begin(); i != included.end(); ++i)
          if (((schema.targetNamespace && (*i).second->targetNamespace && !strcmp(schema.targetNamespace, (*i).second->targetNamespace)) || (!schema.targetNamespace && !(*i).second->targetNamespace)) && !strcmp(schemaLocation, (*i).first))
            break;
      }
      else
      {
        if (schema.targetNamespace)
        {
          for (i = included.begin(); i != included.end(); ++i)
          {
            if ((*i).second->targetNamespace
             && !strcmp(schemaLocation, (*i).first)
             && !strcmp(schema.targetNamespace, (*i).second->targetNamespace))
              break;
          }
        }
      }
      if (i == included.end())
      {
        if (vflag)
          std::cerr << "Preprocessing schema include '" << (schemaLocation ? schemaLocation : "(null)") << "' into schema '" << (schema.targetNamespace ? schema.targetNamespace : "(null)") << "'" << std::endl;
        schemaRef = new xs__schema(schema.soap);
        if (!schemaRef)
          return SOAP_EOM;
        included[schemaLocation] = schemaRef;
        schemaRef->read(schema.sourceLocation(), schemaLocation, relative_schemaLocation);
        if (schema.targetNamespace && (!schemaRef->targetNamespace || strcmp(schema.targetNamespace, schemaRef->targetNamespace)))
        {
          if (!Wflag)
          {
            if (schemaRef->targetNamespace)
              fprintf(stderr, "\nWarning: attempt to include schema with mismatching targetNamespace '%s' into schema namespace '%s', assigning targetNamespace '%s'\n", schemaRef->targetNamespace, schema.targetNamespace, schema.targetNamespace);
            else
              fprintf(stderr, "\nWarning: attempt to include chameleon schema with no targetNamespace into schema namespace '%s', assigning targetNamespace '%s'\n", schema.targetNamespace, schema.targetNamespace);
          }
        }
      }
      else
      {
        if (vflag)
          std::cerr << "Schema '" << (schemaLocation ? schemaLocation : "(null)") << "' already included into schema '" << (schema.targetNamespace ? schema.targetNamespace : "(null)") << "'" << std::endl;
        schemaRef = (*i).second;
      }
    }
    else if (!Wflag)
    {
      fprintf(stderr, "\nWarning: no schemaLocation in <include> to load schema\n");
    }
  }
  return SOAP_OK;
}

int xs__include::traverse(xs__schema &schema)
{
  (void)schema;
  return SOAP_OK;
}

void xs__include::schemaPtr(xs__schema *schema)
{
  schemaRef = schema;
}

xs__schema *xs__include::schemaPtr() const
{
  return schemaRef;
}

xs__redefine::xs__redefine()
{
  schemaLocation = NULL;
  schemaRef = NULL;
}

int xs__redefine::preprocess(xs__schema &schema)
{
  if (vflag)
    std::cerr << "Preprocessing schema redefine '" << (schemaLocation ? schemaLocation : "(null)") << "' into schema '" << (schema.targetNamespace ? schema.targetNamespace : "(null)") << "'" << std::endl;
  if (!schemaRef)
  {
    if (schemaLocation)
    {
      const char *relative_schemaLocation = soap_strdup(schema.soap, schemaLocation);
      schemaLocation = schema.absoluteLocation(schemaLocation);
      schemaRef = new xs__schema(schema.soap, schema.sourceLocation(), schemaLocation, relative_schemaLocation);
      if (!schemaRef)
        return SOAP_EOM;
      // redefine xs:all, xs:choice, or xs:sequence in a group
      for (std::vector<xs__group>::iterator gp = schemaRef->group.begin(); gp != schemaRef->group.end(); ++gp)
      {
        if ((*gp).name)
        {
          for (std::vector<xs__group>::const_iterator g = group.begin(); g != group.end(); ++g)
          {
            if ((*g).name && !strcmp((*gp).name, (*g).name))
            {
              if ((*g).all)
              {
                (*gp).all = (*g).all;
              }
              else if ((*g).choice && (*gp).choice)
              {
                xs__seqchoice& s = *(*gp).choice;
                (*gp).choice = soap_new_xs__seqchoice(schema.soap);
                for (std::vector<xs__contents>::iterator c = (*g).choice->__contents.begin(); c != (*g).choice->__contents.end(); )
                {
                  if ((*c).__union == SOAP_UNION_xs__union_content_element)
                  {
                    (*gp).choice->__contents.push_back(*c);
                    ++c;
                  }
                  else if ((*c).__union == SOAP_UNION_xs__union_content_group)
                  {
                    if ((*c).__content.group->ref)
                    {
                      const char *token = qname_token((*c).__content.group->ref, schema.targetNamespace);
                      if (token && !strcmp((*gp).name, token))
                        for (std::vector<xs__contents>::const_iterator d = s.__contents.begin(); d != s.__contents.end(); ++d)
                          if ((*d).__union == SOAP_UNION_xs__union_content_element)
                            (*gp).choice->__contents.push_back(*d);
                      (*g).choice->__contents.erase(c);
                    }
                  }
                  else
                  {
                    ++c;
                  }
                }
              }
              else if ((*g).sequence && (*gp).sequence)
              {
                xs__seqchoice& s = *(*gp).sequence;
                (*gp).sequence = soap_new_xs__seqchoice(schema.soap);
                for (std::vector<xs__contents>::iterator c = (*g).sequence->__contents.begin(); c != (*g).sequence->__contents.end(); )
                {
                  if ((*c).__union == SOAP_UNION_xs__union_content_element)
                  {
                    (*gp).sequence->__contents.push_back(*c);
                    ++c;
                  }
                  else if ((*c).__union == SOAP_UNION_xs__union_content_group)
                  {
                    if ((*c).__content.group->ref)
                    {
                      const char *token = qname_token((*c).__content.group->ref, schema.targetNamespace);
                      if (token && !strcmp((*gp).name, token))
                        for (std::vector<xs__contents>::const_iterator d = s.__contents.begin(); d != s.__contents.end(); ++d)
                          if ((*d).__union == SOAP_UNION_xs__union_content_element)
                            (*gp).sequence->__contents.push_back(*d);
                      (*g).sequence->__contents.erase(c);
                    }
                  }
                  else
                  {
                    ++c;
                  }
                }
              }
              break;
            }
          }
        }
      }
      // redefine specified attributes in an attributeGroup
      for (std::vector<xs__attributeGroup>::iterator ag = schemaRef->attributeGroup.begin(); ag != schemaRef->attributeGroup.end(); ++ag)
      {
        if ((*ag).name)
        {
          for (std::vector<xs__attributeGroup>::const_iterator g = attributeGroup.begin(); g != attributeGroup.end(); ++g)
          {
            if ((*g).name && !strcmp((*ag).name, (*g).name))
            {
              for (std::vector<xs__attribute>::const_iterator ga = (*g).attribute.begin(); ga != (*g).attribute.end(); ++ga)
              {
                if ((*ga).name)
                {
                  bool found = false;
                  for (std::vector<xs__attribute>::iterator aga = (*ag).attribute.begin(); aga != (*ag).attribute.end(); ++aga)
                  {
                    if ((*aga).name && !strcmp((*aga).name, (*ga).name))
                    {
                      *aga = *ga;
                      found = true;
                      break;
                    }
                  }
                  if (!found)
                    (*ag).attribute.push_back(*ga);
                }
              }
            }
          }
        }
      }
      // redefine simpleType
      for (std::vector<xs__simpleType>::iterator st = schemaRef->simpleType.begin(); st != schemaRef->simpleType.end(); ++st)
      {
        if ((*st).name)
        {
          for (std::vector<xs__simpleType>::const_iterator s = simpleType.begin(); s != simpleType.end(); ++s)
          {
            if ((*s).name && !strcmp((*st).name, (*s).name))
            {
              char *base = (*st).restriction ? (*st).restriction->base : (char*)"xs:string";
              *st = *s;
              (*st).restriction->base = base;
              break;
            }
          }
        }
      }
      // redefine complexType by extension/restriction
      for (std::vector<xs__complexType>::iterator ct = schemaRef->complexType.begin(); ct != schemaRef->complexType.end(); ++ct)
      {
        if ((*ct).name)
        {
          for (std::vector<xs__complexType>::const_iterator c = complexType.begin(); c != complexType.end(); ++c)
          {
            if ((*c).name && !strcmp((*ct).name, (*c).name))
            {
              if ((*c).complexContent && (*c).complexContent->extension && qname_token((*c).complexContent->extension->base, schemaRef->targetNamespace))
              {
                if ((*ct).all && (*c).complexContent->extension->all)
                {
                  (*ct).all->element.insert((*ct).all->element.end(), (*c).complexContent->extension->all->element.begin(), (*c).complexContent->extension->all->element.end()); 
                }
                else if ((*ct).choice && (*c).complexContent->extension->choice)
                {
                  (*ct).choice->__contents.insert((*ct).choice->__contents.end(), (*c).complexContent->extension->choice->__contents.begin(), (*c).complexContent->extension->choice->__contents.end()); 
                }
                else if ((*ct).sequence && (*c).complexContent->extension->sequence)
                {
                  (*ct).sequence->__contents.insert((*ct).sequence->__contents.end(), (*c).complexContent->extension->sequence->__contents.begin(), (*c).complexContent->extension->sequence->__contents.end()); 
                }
                else if ((*ct).group && (*c).complexContent->extension->group)
                {
                  if ((*ct).group->sequence && (*c).complexContent->extension->group->sequence)
                    (*ct).group->sequence->__contents.insert((*ct).group->sequence->__contents.end(), (*c).complexContent->extension->group->sequence->__contents.begin(), (*c).complexContent->extension->group->sequence->__contents.end()); 
                }
                else if ((*ct).complexContent && (*ct).complexContent->extension)
                {
                  if ((*ct).complexContent->extension->all && (*c).complexContent->extension->all)
                  {
                    (*ct).complexContent->extension->all->element.insert((*ct).complexContent->extension->all->element.end(), (*c).complexContent->extension->all->element.begin(), (*c).complexContent->extension->all->element.end()); 
                  }
                  else if ((*ct).complexContent->extension->choice && (*c).complexContent->extension->choice)
                  {
                    (*ct).complexContent->extension->choice->__contents.insert((*ct).complexContent->extension->choice->__contents.end(), (*c).complexContent->extension->choice->__contents.begin(), (*c).complexContent->extension->choice->__contents.end()); 
                  }
                  else if ((*ct).complexContent->extension->sequence && (*c).complexContent->extension->sequence)
                  {
                    (*ct).complexContent->extension->sequence->__contents.insert((*ct).complexContent->extension->sequence->__contents.end(), (*c).complexContent->extension->sequence->__contents.begin(), (*c).complexContent->extension->sequence->__contents.end()); 
                  }
                  else if ((*ct).complexContent->extension->group && (*c).complexContent->extension->group)
                  {
                    if ((*ct).complexContent->extension->group->sequence && (*c).complexContent->extension->group->sequence)
                      (*ct).complexContent->extension->group->sequence->__contents.insert((*ct).complexContent->extension->group->sequence->__contents.end(), (*c).complexContent->extension->group->sequence->__contents.begin(), (*c).complexContent->extension->group->sequence->__contents.end()); 
                  }
                  else
                  {
                    (*ct).complexContent->extension->all = (*c).complexContent->extension->all;
                    (*ct).complexContent->extension->choice = (*c).complexContent->extension->choice;
                    (*ct).complexContent->extension->sequence = (*c).complexContent->extension->sequence;
                    (*ct).complexContent->extension->group = (*c).complexContent->extension->group;
                  }
                }
                else if ((*ct).complexContent && (*ct).complexContent->restriction)
                {
                  if ((*ct).complexContent->restriction->all && (*c).complexContent->extension->all)
                  {
                    (*ct).complexContent->restriction->all->element.insert((*ct).complexContent->restriction->all->element.end(), (*c).complexContent->extension->all->element.begin(), (*c).complexContent->extension->all->element.end()); 
                  }
                  else if ((*ct).complexContent->restriction->choice && (*c).complexContent->extension->choice)
                  {
                    (*ct).complexContent->restriction->choice->__contents.insert((*ct).complexContent->restriction->choice->__contents.end(), (*c).complexContent->extension->choice->__contents.begin(), (*c).complexContent->extension->choice->__contents.end()); 
                  }
                  else if ((*ct).complexContent->restriction->sequence && (*c).complexContent->extension->sequence)
                  {
                    (*ct).complexContent->restriction->sequence->__contents.insert((*ct).complexContent->restriction->sequence->__contents.end(), (*c).complexContent->extension->sequence->__contents.begin(), (*c).complexContent->extension->sequence->__contents.end()); 
                  }
                  else if ((*ct).complexContent->restriction->group && (*c).complexContent->extension->group)
                  {
                    if ((*ct).complexContent->restriction->group->sequence && (*c).complexContent->restriction->group->sequence)
                      (*ct).complexContent->restriction->group->sequence->__contents.insert((*ct).complexContent->restriction->group->sequence->__contents.end(), (*c).complexContent->extension->group->sequence->__contents.begin(), (*c).complexContent->extension->group->sequence->__contents.end()); 
                  }
                  else
                  {
                    (*ct).complexContent->restriction->all = (*c).complexContent->extension->all;
                    (*ct).complexContent->restriction->choice = (*c).complexContent->extension->choice;
                    (*ct).complexContent->restriction->sequence = (*c).complexContent->extension->sequence;
                    (*ct).complexContent->restriction->group = (*c).complexContent->extension->group;
                  }
                }
                else
                {
                  (*ct).all = (*c).complexContent->extension->all;
                  (*ct).choice = (*c).complexContent->extension->choice;
                  (*ct).sequence = (*c).complexContent->extension->sequence;
                  (*ct).group = (*c).complexContent->extension->group;
                }
                (*ct).attribute.insert((*ct).attribute.begin(), (*c).complexContent->extension->attribute.begin(), (*c).complexContent->extension->attribute.end());
              }
              else if ((*c).complexContent && (*c).complexContent->restriction && qname_token((*c).complexContent->restriction->base, schemaRef->targetNamespace))
              {
                (*ct).all = (*c).complexContent->restriction->all;
                (*ct).choice = (*c).complexContent->restriction->choice;
                (*ct).sequence = (*c).complexContent->restriction->sequence;
                (*ct).group = (*c).complexContent->restriction->group;
                (*ct).attribute = (*c).complexContent->restriction->attribute;
                (*ct).complexContent = NULL;
              }
              else
              {
                if (!Wflag)
                  fprintf(stderr, "\nWarning: redefining complexType \"%s\" in schema \"%s\" by extension requires an extension base\n", (*ct).name, schemaRef->targetNamespace ? schemaRef->targetNamespace : "(undefined)");
              }
              break;
            }
          }
        }
      }
    }
    else if (!Wflag)
    {
      fprintf(stderr, "\nWarning: no schemaLocation in <redefine> to load schema\n");
    }
  }
  return SOAP_OK;
}

int xs__redefine::traverse(xs__schema &schema)
{
  (void)schema;
  return SOAP_OK;
}

void xs__redefine::schemaPtr(xs__schema *schema)
{
  schemaRef = schema;
}

xs__schema *xs__redefine::schemaPtr() const
{
  return schemaRef;
}

xs__override::xs__override()
{
  schemaLocation = NULL;
  schemaRef = NULL;
}

int xs__override::preprocess(xs__schema &schema)
{
  if (vflag)
    std::cerr << "Preprocessing schema override '" << (schemaLocation ? schemaLocation : "(null)") << "' into schema '" << (schema.targetNamespace ? schema.targetNamespace : "(null)") << "'" << std::endl;
  if (!schemaRef)
  {
    if (schemaLocation)
    {
      const char *relative_schemaLocation = soap_strdup(schema.soap, schemaLocation);
      schemaLocation = schema.absoluteLocation(schemaLocation);
      schemaRef = new xs__schema(schema.soap, schema.sourceLocation(), schemaLocation, relative_schemaLocation);
      if (!schemaRef)
        return SOAP_EOM;
      for (std::vector<xs__element>::iterator el = schemaRef->element.begin(); el != schemaRef->element.end(); ++el)
      {
        if ((*el).name)
        {
          for (std::vector<xs__element>::const_iterator e = element.begin(); e != element.end(); ++e)
          {
            if ((*e).name && !strcmp((*el).name, (*e).name))
            {
              *el = *e;
              break;
            }
          }
        }
      }
      for (std::vector<xs__attribute>::iterator at = schemaRef->attribute.begin(); at != schemaRef->attribute.end(); ++at)
      {
        if ((*at).name)
        {
          for (std::vector<xs__attribute>::const_iterator a = attribute.begin(); a != attribute.end(); ++a)
          {
            if ((*a).name && !strcmp((*at).name, (*a).name))
            {
              *at = *a;
              break;
            }
          }
        }
      }
      for (std::vector<xs__group>::iterator gp = schemaRef->group.begin(); gp != schemaRef->group.end(); ++gp)
      {
        if ((*gp).name)
        {
          for (std::vector<xs__group>::const_iterator g = group.begin(); g != group.end(); ++g)
          {
            if ((*g).name && !strcmp((*gp).name, (*g).name))
            {
              *gp = *g;
              break;
            }
          }
        }
      }
      for (std::vector<xs__attributeGroup>::iterator ag = schemaRef->attributeGroup.begin(); ag != schemaRef->attributeGroup.end(); ++ag)
      {
        if ((*ag).name)
        {
          for (std::vector<xs__attributeGroup>::const_iterator g = attributeGroup.begin(); g != attributeGroup.end(); ++g)
          {
            if ((*g).name && !strcmp((*ag).name, (*g).name))
            {
              *ag = *g;
              break;
            }
          }
        }
      }
      for (std::vector<xs__simpleType>::iterator st = schemaRef->simpleType.begin(); st != schemaRef->simpleType.end(); ++st)
      {
        if ((*st).name)
        {
          for (std::vector<xs__simpleType>::const_iterator s = simpleType.begin(); s != simpleType.end(); ++s)
          {
            if ((*s).name && !strcmp((*st).name, (*s).name))
            {
              *st = *s;
              break;
            }
          }
        }
      }
      for (std::vector<xs__complexType>::iterator ct = schemaRef->complexType.begin(); ct != schemaRef->complexType.end(); ++ct)
      {
        if ((*ct).name)
        {
          for (std::vector<xs__complexType>::const_iterator c = complexType.begin(); c != complexType.end(); ++c)
          {
            if ((*c).name && !strcmp((*ct).name, (*c).name))
            {
              *ct = *c;
              break;
            }
          }
        }
      }
    }
    else if (!Wflag)
    {
      fprintf(stderr, "\nWarning: no schemaLocation in <override> to load schema\n");
    }
  }
  return SOAP_OK;
}

int xs__override::traverse(xs__schema &schema)
{
  (void)schema;
  return SOAP_OK;
}

void xs__override::schemaPtr(xs__schema *schema)
{
  schemaRef = schema;
}

xs__schema *xs__override::schemaPtr() const
{
  return schemaRef;
}

xs__import::xs__import()
{
  namespace_ = NULL;
  schemaLocation = NULL;
  location = NULL; // work around a Microsoft WSDL bug uses @location instead of @schemaLocation in WSDLs
  schemaRef = NULL;
}

int xs__import::preprocess(xs__schema &schema)
{
  // work around a Microsoft bug that uses @location instead of @schemaLocation in WSDLs
  if (!schemaLocation && location)
    schemaLocation = location;
  if (vflag)
    std::cerr << "   Preprocessing schema import '" << (namespace_ ? namespace_ : "(null)") << "' location '" << schemaLocation << "'" << std::endl;
  if (!schemaRef)
  {
    bool found = false;
    if (namespace_)
    {
      for (SetOfString::const_iterator i = exturis.begin(); i != exturis.end(); ++i)
      {
        if (!soap_tag_cmp(namespace_, *i))
        {
          found = true;
          break;
        }
      }
    }
    else
    {
      if (!zflag || zflag > 10)
        namespace_ = (char*)"";
      else if (!Wflag)
        fprintf(stderr, "\nWarning: no namespace in <import>\n");
    }
    if (!found && !iflag) // don't import any of the schemas in the .nsmap table (or when -i option is used)
    {
      if (schemaLocation)
      {
        schemaLocation = schema.absoluteLocation(schemaLocation);
        // only read from import locations not read already, uses static std::map
        static std::map<const char*, xs__schema*, ltstr> included;
        std::map<const char*, xs__schema*, ltstr>::iterator i = included.find(schemaLocation);
        const char *relative_schemaLocation = soap_strdup(schema.soap, schemaLocation);
        if (i == included.end())
        {
          included[schemaLocation] = schemaRef = new xs__schema(schema.soap);
          if (!schemaRef)
            return SOAP_EOM;
          schemaRef->read(schema.sourceLocation(), schemaLocation, relative_schemaLocation);
        }
        else
        {
          schemaRef = (*i).second;
        }
        if (schemaRef)
        {
          if (!schemaRef->targetNamespace || !*schemaRef->targetNamespace)
          {
            schemaRef->targetNamespace = namespace_;
          }
          else if (!namespace_ || strcmp(schemaRef->targetNamespace, namespace_))
          {
            if (!Wflag)
              fprintf(stderr, "\nWarning: schema import '%s' with schema targetNamespace '%s' mismatch\n", namespace_ ? namespace_ : "(undefined)", schemaRef->targetNamespace ? schemaRef->targetNamespace : "(undefined)");
          }
        }
      }
    }
  }
  return SOAP_OK;
}

int xs__import::traverse(xs__schema &schema)
{
  (void)schema;
  if (schemaRef)
    schemaRef->traverse();
  return SOAP_OK;
}

void xs__import::schemaPtr(xs__schema *schema)
{
  schemaRef = schema;
}

xs__schema *xs__import::schemaPtr() const
{
  return schemaRef;
}

void xs__import::mark()
{
  if (Oflag > 1)
    if (schemaPtr())
      schemaPtr()->mark();
}

xs__attribute::xs__attribute()
{
  schemaRef = NULL;
  attributeRef = NULL;
  simpleTypeRef = NULL;
  used = false;
}

int xs__attribute::traverse(xs__schema &schema)
{
  if (vflag)
    std::cerr << "   Analyzing schema attribute '" << (name ? name : "(null)") << "'" << std::endl;
  schemaRef = &schema;
  const char *token = qname_token(ref, schema.targetNamespace);
  attributeRef = NULL;
  if (token)
  {
    for (std::vector<xs__attribute>::iterator i = schema.attribute.begin(); i != schema.attribute.end(); ++i)
    {
      if ((*i).name && !strcmp((*i).name, token))
      {
        attributeRef = &(*i);
        if (vflag)
          std::cerr << "    Found attribute '" << (name ? name : "(null)") << "' ref '" << (token ? token : "(null)") << "'" << std::endl;
        break;
      }
    }
  }
  if (!attributeRef)
  {
    for (std::vector<xs__import>::iterator i = schema.import.begin(); i != schema.import.end(); ++i)
    {
      xs__schema *s = (*i).schemaPtr();
      if (s)
      {
        token = qname_token(ref, s->targetNamespace);
        if (token)
        {
          for (std::vector<xs__attribute>::iterator j = s->attribute.begin(); j != s->attribute.end(); ++j)
          {
            if ((*j).name && !strcmp((*j).name, token))
            {
              attributeRef = &(*j);
              if (vflag)
                std::cerr << "    Found attribute '" << (name ? name : "(null)") << "' ref '" << (token ? token : "(null)") << "'" << std::endl;
              break;
            }
          }
          if (attributeRef)
            break;
        }
      }
    }
  }
  if (simpleType)
  {
    simpleType->traverse(schema);
    simpleTypeRef = simpleType;
  }
  else
  {
    token = qname_token(type, schema.targetNamespace);
    simpleTypeRef = NULL;
    if (token)
    {
      for (std::vector<xs__simpleType>::iterator i = schema.simpleType.begin(); i != schema.simpleType.end(); ++i)
      {
        if ((*i).name && !strcmp((*i).name, token))
        {
          simpleTypeRef = &(*i);
          if (vflag)
            std::cerr << "    Found attribute '" << (name ? name : "(null)") << "' type '" << (token ? token : "(null)") << "'" << std::endl;
          break;
        }
      }
    }
    if (!simpleTypeRef)
    {
      for (std::vector<xs__import>::iterator i = schema.import.begin(); i != schema.import.end(); ++i)
      {
        xs__schema *s = (*i).schemaPtr();
        if (s)
        {
          token = qname_token(type, s->targetNamespace);
          if (token)
          {
            for (std::vector<xs__simpleType>::iterator j = s->simpleType.begin(); j != s->simpleType.end(); ++j)
            {
              if ((*j).name && !strcmp((*j).name, token))
              {
                simpleTypeRef = &(*j);
                if (vflag)
                  std::cerr << "    Found attribute '" << (name ? name : "(null)") << "' type '" << (token ? token : "(null)") << "'" << std::endl;
                break;
              }
            }
            if (simpleTypeRef)
              break;
          }
        }
      }
    }
  }
  if (!attributeRef && !simpleTypeRef)
  {
    if (ref)
    {
      if (is_builtin_qname(ref))
        schema.builtinAttribute(ref);
      else if (!Wflag)
        std::cerr << "\nWarning: could not find the referenced attribute '" << (name ? name : "") << "' ref '" << ref << "' in schema '" << (schema.targetNamespace ? schema.targetNamespace : "(undefined)") << "'" << std::endl;
      /* moved to restriction::traverse()
      if (wsdl__arrayType)
      {
        char *arrayType = soap_strdup(schema.soap, wsdl__arrayType);
        char *s = strchr(arrayType, '[');
        if (s)
          *s = '\0';
        if (is_builtin_qname(arrayType))
          schema.builtinType(arrayType);
      }
      */
    }
    else if (type)
    {
      if (is_builtin_qname(type))
        schema.builtinType(type);
      else if (!Wflag)
        std::cerr << "\nWarning: could not find the type for attribute '" << (name ? name : "(undefined)") << "' type '" << type << "' in schema '" << (schema.targetNamespace ? schema.targetNamespace : "(undefined)") << "'" << std::endl;
    }
  }
  return SOAP_OK;
}

void xs__attribute::schemaPtr(xs__schema *schema)
{
  schemaRef = schema;
}

xs__schema* xs__attribute::schemaPtr() const
{
  return schemaRef;
}

void xs__attribute::attributePtr(xs__attribute *attribute)
{
  attributeRef = attribute;
}

void xs__attribute::simpleTypePtr(xs__simpleType *simpleType)
{
  simpleTypeRef = simpleType;
}

xs__attribute *xs__attribute::attributePtr() const
{
  return attributeRef;
}

xs__simpleType *xs__attribute::simpleTypePtr() const
{
  return simpleTypeRef;
}

void xs__attribute::mark()
{
  if (Oflag > 1 && !used)
  {
    used = true;
    if (attributePtr())
      attributePtr()->mark();
    if (simpleTypePtr())
      simpleTypePtr()->mark();
  }
}

bool xs__attribute::is_used() const
{
  return used;
}

xs__element::xs__element()
{
  schemaRef = NULL;
  elementRef = NULL;
  simpleTypeRef = NULL;
  complexTypeRef = NULL;
  name = NULL;
  ref = NULL;
  type = NULL;
  default_ = NULL;
  default__ = NULL;
  fixed = NULL;
  fixed_ = NULL;
  form = NULL;
  nillable = false;
  abstract = false;
  substitutionGroup = NULL;
  minOccurs = NULL;
  maxOccurs = NULL;
  xmime__expectedContentTypes = NULL;
  annotation = NULL;
  simpleType = NULL;
  complexType = NULL;
  unique = NULL;
  used = false;
}

int xs__element::traverse(xs__schema &schema)
{
  if (vflag)
    std::cerr << "   Analyzing schema element '" << (name ? name : "(null)") << "'" << std::endl;
  schemaRef = &schema;
  const char *token = qname_token(ref, schema.targetNamespace);
  elementRef = NULL;
  if (token)
  {
    for (std::vector<xs__element>::iterator i = schema.element.begin(); i != schema.element.end(); ++i)
    {
      if ((*i).name && !strcmp((*i).name, token))
      {
        elementRef = &(*i);
        if (vflag)
          std::cerr << "    Found element '" << (name ? name : "(null)") << "' ref '" << (token ? token : "(null)") << "'" << std::endl;
        break;
      }
    }
  }
  if (!elementRef)
  {
    for (std::vector<xs__import>::const_iterator i = schema.import.begin(); i != schema.import.end(); ++i)
    {
      xs__schema *s = (*i).schemaPtr();
      if (s)
      {
        token = qname_token(ref, s->targetNamespace);
        if (token)
        {
          for (std::vector<xs__element>::iterator j = s->element.begin(); j != s->element.end(); ++j)
          {
            if ((*j).name && !strcmp((*j).name, token))
            {
              elementRef = &(*j);
              if (vflag)
                std::cerr << "    Found element '" << (name ? name : "(null)") << "' ref '" << (token ? token : "(null)") << "'" << std::endl;
              break;
            }
          }
          if (elementRef)
            break;
        }
      }
    }
  }
  if (simpleType)
  {
    simpleType->traverse(schema);
    simpleTypeRef = simpleType;
  }
  else
  {
    token = qname_token(type, schema.targetNamespace);
    simpleTypeRef = NULL;
    if (token)
    {
      for (std::vector<xs__simpleType>::iterator i = schema.simpleType.begin(); i != schema.simpleType.end(); ++i)
        if ((*i).name && !strcmp((*i).name, token))
        {
          simpleTypeRef = &(*i);
          if (vflag)
            std::cerr << "    Found element '" << (name ? name : "(null)") << "' simpleType '" << (token ? token : "(null)") << "'" << std::endl;
          break;
        }
    }
    if (!simpleTypeRef)
    {
      for (std::vector<xs__import>::const_iterator i = schema.import.begin(); i != schema.import.end(); ++i)
      {
        xs__schema *s = (*i).schemaPtr();
        if (s)
        {
          token = qname_token(type, s->targetNamespace);
          if (token)
          {
            for (std::vector<xs__simpleType>::iterator j = s->simpleType.begin(); j != s->simpleType.end(); ++j)
            {
              if ((*j).name && !strcmp((*j).name, token))
              {
                simpleTypeRef = &(*j);
                if (vflag)
                  std::cerr << "    Found element '" << (name ? name : "(null)") << "' simpleType '" << (token ? token : "(null)") << "'" << std::endl;
                break;
              }
            }
            if (simpleTypeRef)
              break;
          }
        }
      }
    }
  }
  if (complexType)
  {
    complexType->traverse(schema);
    complexTypeRef = complexType;
  }
  else
  {
    token = qname_token(type, schema.targetNamespace);
    complexTypeRef = NULL;
    if (token)
    {
      for (std::vector<xs__complexType>::iterator i = schema.complexType.begin(); i != schema.complexType.end(); ++i)
        if ((*i).name && !strcmp((*i).name, token))
        {
          complexTypeRef = &(*i);
          if (vflag)
            std::cerr << "    Found element '" << (name ? name : "(null)") << "' complexType '" << (token ? token : "(null)") << "'" << std::endl;
          break;
        }
    }
    if (!complexTypeRef)
    {
      for (std::vector<xs__import>::const_iterator i = schema.import.begin(); i != schema.import.end(); ++i)
      {
        xs__schema *s = (*i).schemaPtr();
        if (s)
        {
          token = qname_token(type, s->targetNamespace);
          if (token)
          {
            for (std::vector<xs__complexType>::iterator j = s->complexType.begin(); j != s->complexType.end(); ++j)
            {
              if ((*j).name && !strcmp((*j).name, token))
              {
                complexTypeRef = &(*j);
                if (vflag)
                  std::cerr << "    Found element '" << (name ? name : "(null)") << "' complexType '" << (token ? token : "(null)") << "'" << std::endl;
                break;
              }
            }
            if (complexTypeRef)
              break;
          }
        }
      }
    }
  }
  if (!abstract && substitutionGroup)
  {
    token = qname_token(substitutionGroup, schema.targetNamespace);
    if (token)
    {
      for (std::vector<xs__element>::iterator i = schema.element.begin(); i != schema.element.end(); ++i)
      {
        if ((*i).name && !strcmp((*i).name, token))
        {
          xs__element *elt = this;
          if (!elementPtr())
          {
            elt = soap_new_xs__element(schema.soap);
            elt->soap_default(schema.soap);
            elt->name = name;
            elt->form = form;
            elt->elementPtr(this); // create element ref in substitutionGroup
            elt->schemaPtr(schemaPtr());
            elt->targetNamespace = NULL;
          }
          std::vector<xs__element*>::iterator k = (*i).substitutions.begin();
          while (k != (*i).substitutions.end())
          {
            if ((*k)->schemaPtr() == elt->schemaPtr() && !strcmp((*k)->name, elt->name))
              break;
            ++k;
          }
          if (k == (*i).substitutions.end())
            (*i).substitutions.push_back(elt);
          if (vflag)
            std::cerr << "    Found substitutionGroup element '" << (name ? name : "(null)") << "' for element '" << (token ? token : "(null)") << "'" << std::endl;
          break;
        }
      }
    }
    for (std::vector<xs__import>::const_iterator i = schema.import.begin(); i != schema.import.end(); ++i)
    {
      xs__schema *s = (*i).schemaPtr();
      if (s)
      {
        token = qname_token(substitutionGroup, s->targetNamespace);
        if (token)
        {
          for (std::vector<xs__element>::iterator j = s->element.begin(); j != s->element.end(); ++j)
          {
            if ((*j).name && !strcmp((*j).name, token))
            {
              xs__element *elt = this;
              if (!elementPtr())
              {
                elt = soap_new_xs__element(schema.soap);
                elt->soap_default(schema.soap);
                elt->name = name;
                elt->form = form;
                elt->elementPtr(this); // create element ref in substitutionGroup
                elt->schemaPtr(schemaPtr());
                elt->targetNamespace = NULL;
              }
              std::vector<xs__element*>::iterator k = (*j).substitutions.begin();
              while (k != (*j).substitutions.end())
              {
                if ((*k)->schemaPtr() == elt->schemaPtr() && !strcmp((*k)->name, elt->name))
                  break;
                ++k;
              }
              if (k == (*j).substitutions.end())
                (*j).substitutions.push_back(elt);
              if (vflag)
                std::cerr << "    Found substitutionGroup element '" << (name ? name : "(null)") << "' for element '" << (token ? token : "(null)") << "' in '" << s->targetNamespace << "'" << std::endl;
              break;
            }
          }
        }
      }
    }
  }
  if (!elementRef && !simpleTypeRef && !complexTypeRef)
  {
    if (ref)
    {
      if (is_builtin_qname(ref))
        schema.builtinElement(ref);
      else if (!Wflag)
        std::cerr << "\nWarning: could not find the referenced element '" << (name ? name : "") << "' ref '" << ref << "' in schema '" << (schema.targetNamespace ? schema.targetNamespace : "(undefined)") << "'" << std::endl;
    }
    else if (type)
    {
      if (is_builtin_qname(type))
        schema.builtinType(type);
      else if (!Wflag)
        std::cerr << "\nWarning: could not find the type for element '" << (name ? name : "(undefined)") << "' type '" << type << "' in schema '" << (schema.targetNamespace ? schema.targetNamespace : "(undefined)") << "'" << std::endl;
    }
  }
  return SOAP_OK;
}

void xs__element::schemaPtr(xs__schema *schema)
{
  schemaRef = schema;
}

xs__schema* xs__element::schemaPtr() const
{
  return schemaRef;
}

void xs__element::elementPtr(xs__element *element)
{
  elementRef = element;
}

void xs__element::simpleTypePtr(xs__simpleType *simpleType)
{
  simpleTypeRef = simpleType;
}

void xs__element::complexTypePtr(xs__complexType *complexType)
{
  complexTypeRef = complexType;
}

xs__element *xs__element::elementPtr() const
{
  return elementRef;
}

const std::vector<xs__element*>* xs__element::substitutionsPtr() const
{
  return &substitutions;
}

xs__simpleType *xs__element::simpleTypePtr() const
{
  return simpleTypeRef;
}

xs__complexType *xs__element::complexTypePtr() const
{
  return complexTypeRef;
}

void xs__element::mark()
{
  if (Oflag > 1 && !used)
  {
    used = true;
    if (elementPtr())
      elementPtr()->mark();
    if (simpleTypePtr())
      simpleTypePtr()->mark();
    if (complexTypePtr())
      complexTypePtr()->mark();
    for (std::vector<xs__element*>::iterator i = substitutions.begin(); i != substitutions.end(); ++i)
      if ((*i)->elementPtr())
        (*i)->elementPtr()->mark();
  }
}

bool xs__element::is_used() const
{
  return used;
}

xs__simpleType::xs__simpleType()
{
  schemaRef = NULL;
  level = 0;
  used = false;
}

int xs__simpleType::traverse(xs__schema &schema)
{
  if (vflag)
    std::cerr << "   Analyzing schema simpleType '" << (name ? name : "(null)") << "'" << std::endl;
  schemaRef = &schema;
  if (list)
  {
    list->traverse(schema);
  }
  else if (restriction)
  {
    restriction->traverse(schema);
    if (name)
    {
      xs__simpleType *base = restriction->simpleTypePtr();
      if (base)
        base->add_restriction(schema, name);
      else if (is_builtin_qname(restriction->base))
        schema.builtinTypeDerivation(schema, restriction->base, name);
    }
  }
  else if (union_)
  {
    union_->traverse(schema);
  }
  return SOAP_OK;
}

void xs__simpleType::schemaPtr(xs__schema *schema)
{
  schemaRef = schema;
}

xs__schema *xs__simpleType::schemaPtr() const
{
  return schemaRef;
}

int xs__simpleType::baseLevel()
{
  if (!level)
  {
    if (restriction)
    {
      level = -1;
      if (restriction->simpleTypePtr())
        level = restriction->simpleTypePtr()->baseLevel() + 1;
      else
        level = 2;
    }
    else if (list && list->restriction)
    {
      level = -1;
      if (list->restriction->simpleTypePtr())
        level = list->restriction->simpleTypePtr()->baseLevel() + 1;
      else
        level = 2;
    }
    else
      level = 1;
  }
  else if (level < 0)
  {
    std::cerr << "Error: cyclic simpleType restriction/extension base dependency in '" << (name ? name : "(null)") << "'" << std::endl;
  }
  return level;
}

void xs__simpleType::mark()
{
  if (Oflag > 1 && !used)
  {
    used = true;
    if (restriction)
      restriction->mark();
    else if (list)
      list->mark();
    else if (union_)
      union_->mark();
    if (Owflag)
      for (std::vector<xs__complexType*>::const_iterator i = complextype_extensions.begin(); i != complextype_extensions.end(); ++i)
        (*i)->mark();
  }
}

bool xs__simpleType::is_used() const
{
  return used;
}

void xs__simpleType::add_extension(xs__complexType *complexType, xs__schema& schema, xsd__NCName name)
{
  complextype_extensions.push_back(complexType);
  extensions.push_back(make_qname(schema, name));
}

void xs__simpleType::add_restriction(xs__schema& schema, xsd__NCName name)
{
  restrictions.push_back(make_qname(schema, name));
}

const std::vector<xsd__QName>& xs__simpleType::get_extensions() const
{
  return extensions;
}

const std::vector<xsd__QName>& xs__simpleType::get_restrictions() const
{
  return restrictions;
}

xs__complexType::xs__complexType()
{
  schemaRef = NULL;
  level = 0;
  used = false;
}

int xs__complexType::traverse(xs__schema &schema)
{
  if (vflag)
    std::cerr << "   Analyzing schema complexType '" << (name ? name : "(null)") << "'" << std::endl;
  schemaRef = &schema;
  if (simpleContent)
  {
    simpleContent->traverse(schema);
    if (name)
    {
      if (simpleContent->extension)
      {
        xs__complexType *ct_base = simpleContent->extension->complexTypePtr();
        if (ct_base)
        {
          ct_base->add_extension(this, schema, name);
        }
        else
        {
          xs__simpleType *st_base = simpleContent->extension->simpleTypePtr();
          if (st_base)
            st_base->add_extension(this, schema, name);
          else if (is_builtin_qname(simpleContent->extension->base))
            schema.builtinTypeDerivation(schema, simpleContent->extension->base, name);
        }
      }
      else if (simpleContent->restriction)
      {
        xs__complexType *ct_base = simpleContent->restriction->complexTypePtr();
        if (ct_base)
        {
          ct_base->add_restriction(schema, name);
        }
        else
        {
          xs__simpleType *st_base = simpleContent->restriction->simpleTypePtr();
          if (st_base)
            st_base->add_restriction(schema, name);
          else if (is_builtin_qname(simpleContent->restriction->base))
            schema.builtinTypeDerivation(schema, simpleContent->restriction->base, name);
        }
      }
    }
  }
  else if (complexContent)
  {
    complexContent->traverse(schema);
    if (name)
    {
      if (complexContent->extension)
      {
        xs__complexType *ct_base = complexContent->extension->complexTypePtr();
        if (ct_base)
          ct_base->add_extension(this, schema, name);
      }
      else if (complexContent->restriction)
      {
        xs__complexType *ct_base = complexContent->restriction->complexTypePtr();
        if (ct_base)
          ct_base->add_restriction(schema, name);
      }
    }
  }
  else if (all)
  {
    all->traverse(schema);
  }
  else if (choice)
  {
    choice->traverse(schema);
  }
  else if (sequence)
  {
    sequence->traverse(schema);
  }
  else if (group)
  {
    group->traverse(schema);
  }
  else if (any)
  {
    any->traverse(schema);
  }
  for (std::vector<xs__attribute>::iterator at = attribute.begin(); at != attribute.end(); ++at)
    (*at).traverse(schema);
  for (std::vector<xs__attributeGroup>::iterator ag = attributeGroup.begin(); ag != attributeGroup.end(); ++ag)
    (*ag).traverse(schema);
  return SOAP_OK;
}

void xs__complexType::schemaPtr(xs__schema *schema)
{
  schemaRef = schema;
}

xs__schema *xs__complexType::schemaPtr() const
{
  return schemaRef;
}

int xs__complexType::baseLevel()
{
  if (!level)
  {
    if (simpleContent)
    {
      if (simpleContent->restriction)
      {
        level = -1;
        if (simpleContent->restriction->simpleTypePtr())
          level = simpleContent->restriction->simpleTypePtr()->baseLevel() + 1;
        else if (simpleContent->restriction->complexTypePtr())
          level = simpleContent->restriction->complexTypePtr()->baseLevel() + 1;
        else
          level = 2;
      }
      else if (simpleContent->extension)
      {
        level = -1;
        if (simpleContent->extension->simpleTypePtr())
          level = simpleContent->extension->simpleTypePtr()->baseLevel() + 1;
        else if (simpleContent->extension->complexTypePtr())
          level = simpleContent->extension->complexTypePtr()->baseLevel() + 1;
        else
          level = 2;
      }
    }
    else if (complexContent)
    {
      if (complexContent->restriction)
      {
        level = -1;
        if (complexContent->restriction->simpleTypePtr())
          level = complexContent->restriction->simpleTypePtr()->baseLevel() + 1;
        else if (complexContent->restriction->complexTypePtr())
          level = complexContent->restriction->complexTypePtr()->baseLevel() + 1;
        else
          level = 2;
      }
      else if (complexContent->extension)
      {
        level = -1;
        if (complexContent->extension->simpleTypePtr())
          level = complexContent->extension->simpleTypePtr()->baseLevel() + 1;
        else if (complexContent->extension->complexTypePtr())
          level = complexContent->extension->complexTypePtr()->baseLevel() + 1;
        else
          level = 2;
      }
    }
    else
      level = 1;
  }
  else if (level < 0)
  {
    std::cerr << "Error: cyclic complexType restriction/extension base dependency in '" << (name ? name : "(null)") << "'" << std::endl;
  }
  return level;
}

void xs__complexType::mark()
{
  if (Oflag > 1 && !used)
  {
    used = true;
    if (simpleContent)
      simpleContent->mark();
    else if (complexContent)
      complexContent->mark();
    else if (all)
      all->mark();
    else if (choice)
      choice->mark();
    else if (sequence)
      sequence->mark();
    else if (group)
      group->mark();
    else if (any)
      any->mark();
    for (std::vector<xs__attribute>::iterator at = attribute.begin(); at != attribute.end(); ++at)
      (*at).mark();
    for (std::vector<xs__attributeGroup>::iterator ag = attributeGroup.begin(); ag != attributeGroup.end(); ++ag)
      (*ag).mark();
    if (Owflag)
      for (std::vector<xs__complexType*>::const_iterator i = complextype_extensions.begin(); i != complextype_extensions.end(); ++i)
        (*i)->mark();
  }
}

bool xs__complexType::is_used() const
{
  return used;
}

void xs__complexType::add_extension(xs__complexType *complexType, xs__schema& schema, xsd__NCName name)
{
  complextype_extensions.push_back(complexType);
  extensions.push_back(make_qname(schema, name));
}

void xs__complexType::add_restriction(xs__schema& schema, xsd__NCName name)
{
  restrictions.push_back(make_qname(schema, name));
}

const std::vector<xsd__QName>& xs__complexType::get_extensions() const
{
  return extensions;
}

const std::vector<xsd__QName>& xs__complexType::get_restrictions() const
{
  return restrictions;
}

int xs__simpleContent::traverse(xs__schema &schema)
{
  if (vflag)
    std::cerr << "   Analyzing schema simpleContent" << std::endl;
  if (extension)
    extension->traverse(schema);
  else if (restriction)
    restriction->traverse(schema);
  return SOAP_OK;
}

void xs__simpleContent::mark()
{
  if (Oflag > 1)
  {
    if (extension)
      extension->mark();
    else if (restriction)
      restriction->mark();
  }
}

int xs__complexContent::traverse(xs__schema &schema)
{
  if (vflag)
    std::cerr << "   Analyzing schema complexContent" << std::endl;
  if (extension)
    extension->traverse(schema);
  else if (restriction)
    restriction->traverse(schema);
  return SOAP_OK;
}

void xs__complexContent::mark()
{
  if (Oflag > 1)
  {
    if (extension)
      extension->mark();
    else if (restriction)
      restriction->mark();
  }
}

xs__extension::xs__extension()
{
  simpleTypeRef = NULL;
  complexTypeRef = NULL;
}

int xs__extension::traverse(xs__schema &schema)
{
  if (vflag)
    std::cerr << "   Analyzing schema extension '" << (base ? base : "(null)") << "'" << std::endl;
  if (group)
    group->traverse(schema);
  else if (all)
    all->traverse(schema);
  else if (choice)
    choice->traverse(schema);
  else if (sequence)
    sequence->traverse(schema);
  for (std::vector<xs__attribute>::iterator at = attribute.begin(); at != attribute.end(); ++at)
    (*at).traverse(schema);
  for (std::vector<xs__attributeGroup>::iterator ag = attributeGroup.begin(); ag != attributeGroup.end(); ++ag)
    (*ag).traverse(schema);
  const char *token = qname_token(base, schema.targetNamespace);
  simpleTypeRef = NULL;
  if (token)
  {
    for (std::vector<xs__simpleType>::iterator i = schema.simpleType.begin(); i != schema.simpleType.end(); ++i)
      if ((*i).name && !strcmp((*i).name, token))
      {
        simpleTypeRef = &(*i);
        if (vflag)
          std::cerr << "    Found extension base type '" << (token ? token : "(null)") << "'" << std::endl;
        break;
      }
  }
  if (!simpleTypeRef)
  {
    for (std::vector<xs__import>::const_iterator i = schema.import.begin(); i != schema.import.end(); ++i)
    {
      xs__schema *s = (*i).schemaPtr();
      if (s)
      {
        token = qname_token(base, s->targetNamespace);
        if (token)
        {
          for (std::vector<xs__simpleType>::iterator j = s->simpleType.begin(); j != s->simpleType.end(); ++j)
          {
            if ((*j).name && !strcmp((*j).name, token))
            {
              simpleTypeRef = &(*j);
              if (vflag)
                std::cerr << "    Found extension base type '" << (token ? token : "(null)") << "'" << std::endl;
              break;
            }
          }
          if (simpleTypeRef)
            break;
        }
      }
    }
  }
  token = qname_token(base, schema.targetNamespace);
  complexTypeRef = NULL;
  if (token)
  {
    for (std::vector<xs__complexType>::iterator i = schema.complexType.begin(); i != schema.complexType.end(); ++i)
      if ((*i).name && !strcmp((*i).name, token))
      {
        complexTypeRef = &(*i);
        if (vflag)
          std::cerr << "    Found extension base type '" << (token ? token : "(null)") << "'" << std::endl;
        break;
      }
  }
  if (!complexTypeRef)
  {
    for (std::vector<xs__import>::const_iterator i = schema.import.begin(); i != schema.import.end(); ++i)
    {
      xs__schema *s = (*i).schemaPtr();
      if (s)
      {
        token = qname_token(base, s->targetNamespace);
        if (token)
        {
          for (std::vector<xs__complexType>::iterator j = s->complexType.begin(); j != s->complexType.end(); ++j)
          {
            if ((*j).name && !strcmp((*j).name, token))
            {
              complexTypeRef = &(*j);
              if (vflag)
                std::cerr << "    Found extension base type '" << (token ? token : "(null)") << "'" << std::endl;
              break;
            }
          }
          if (complexTypeRef)
            break;
        }
      }
    }
  }
  if (!simpleTypeRef && !complexTypeRef)
  {
    if (base)
    {
      if (is_builtin_qname(base))
        schema.builtinType(base);
      else if (!Wflag)
        std::cerr << "\nWarning: could not find extension base type '" << base << "' in schema '" << (schema.targetNamespace ? schema.targetNamespace : "(undefined)") << "'" << std::endl;
    }
    else
      std::cerr << "Extension has no base" << std::endl;
  }
  return SOAP_OK;
}

void xs__extension::simpleTypePtr(xs__simpleType *simpleType)
{
  simpleTypeRef = simpleType;
}

void xs__extension::complexTypePtr(xs__complexType *complexType)
{
  complexTypeRef = complexType;
}

void xs__extension::mark()
{
  if (Oflag > 1)
  {
    if (simpleTypePtr())
      simpleTypePtr()->mark();
    if (complexTypePtr())
      complexTypePtr()->mark();
    if (group)
      group->mark();
    else if (all)
      all->mark();
    else if (choice)
      choice->mark();
    else if (sequence)
      sequence->mark();
    for (std::vector<xs__attribute>::iterator at = attribute.begin(); at != attribute.end(); ++at)
      (*at).mark();
    for (std::vector<xs__attributeGroup>::iterator ag = attributeGroup.begin(); ag != attributeGroup.end(); ++ag)
      (*ag).mark();
  }
}

xs__simpleType *xs__extension::simpleTypePtr() const
{
  return simpleTypeRef;
}

xs__complexType *xs__extension::complexTypePtr() const
{
  return complexTypeRef;
}

xs__restriction::xs__restriction()
{
  simpleTypeRef = NULL;
  complexTypeRef = NULL;
  simpleArrayTypeRef = NULL;
  complexArrayTypeRef = NULL;
}

int xs__restriction::traverse(xs__schema &schema)
{
  if (vflag)
    std::cerr << "   Analyzing schema restriction '" << (base ? base : "(null)") << "'" << std::endl;
  if (simpleType)
    simpleType->traverse(schema);
  if (attributeGroup)
    attributeGroup->traverse(schema);
  if (group)
    group->traverse(schema);
  else if (all)
    all->traverse(schema);
  else if (choice)
    choice->traverse(schema);
  else if (sequence)
    sequence->traverse(schema);
  else
  {
    for (std::vector<xs__enumeration>::iterator en = enumeration.begin(); en != enumeration.end(); ++en)
      (*en).traverse(schema);
    for (std::vector<xs__pattern>::iterator pn = pattern.begin(); pn != pattern.end(); ++pn)
      (*pn).traverse(schema);
  }
  for (std::vector<xs__attribute>::iterator at = attribute.begin(); at != attribute.end(); ++at)
  {
    (*at).traverse(schema);
    if ((*at).wsdl__arrayType)
    {
      char *arrayType = soap_strdup(schema.soap, (*at).wsdl__arrayType);
      char *r = strchr(arrayType, '[');
      if (r)
        *r = '\0';
#if 0
      /* one work-around for incomplete schemas with SOAP arrays defined solely by wsdl:arrayType is to add sequence/element with type but this creates __ptritem */
      if (!sequence)
      {
        std::cerr << "ADDED sequence/element for array\n";
        sequence = soap_new_xs__seqchoice(schema.soap);
        sequence->soap_default(schema.soap);
        xs__contents ct;
        ct.__union = SOAP_UNION_xs__union_content_element;
        ct.__content.element = soap_new_xs__element(schema.soap);
        ct.__content.element->soap_default(schema.soap);
        ct.__content.element->name = soap_strdup(schema.soap, "item");
        ct.__content.element->type = arrayType;
        ct.__content.element->minOccurs = soap_strdup(schema.soap, "0");
        ct.__content.element->maxOccurs = soap_strdup(schema.soap, "unbounded");
        sequence->__contents.push_back(ct);
        sequence->traverse(schema);
      }
#else
      /* another way is to add simpleArrayTypeRef and complexArrayTypeRef */
      const char *token = qname_token(arrayType, schema.targetNamespace);
      simpleArrayTypeRef = NULL;
      if (token)
      {
        for (std::vector<xs__simpleType>::iterator i = schema.simpleType.begin(); i != schema.simpleType.end(); ++i)
        {
          if ((*i).name && !strcmp((*i).name, token))
          {
            simpleArrayTypeRef = &(*i);
            if (vflag)
              std::cerr << "    Found restriction array type '" << (token ? token : "(null)") << "'" << std::endl;
            break;
          }
        }
      }
      if (!simpleArrayTypeRef)
      {
        for (std::vector<xs__import>::const_iterator i = schema.import.begin(); i != schema.import.end(); ++i)
        {
          xs__schema *s = (*i).schemaPtr();
          if (s)
          {
            token = qname_token(arrayType, s->targetNamespace);
            if (token)
            {
              for (std::vector<xs__simpleType>::iterator j = s->simpleType.begin(); j != s->simpleType.end(); ++j)
              {
                if ((*j).name && !strcmp((*j).name, token))
                {
                  simpleArrayTypeRef = &(*j);
                  if (vflag)
                    std::cerr << "    Found restriction array type '" << (token ? token : "(null)") << "'" << std::endl;
                  break;
                }
              }
              if (simpleArrayTypeRef)
                break;
            }
          }
        }
      }
      token = qname_token(arrayType, schema.targetNamespace);
      complexArrayTypeRef = NULL;
      if (token)
      {
        for (std::vector<xs__complexType>::iterator i = schema.complexType.begin(); i != schema.complexType.end(); ++i)
        {
          if ((*i).name && !strcmp((*i).name, token))
          {
            complexArrayTypeRef = &(*i);
            if (vflag)
              std::cerr << "    Found restriction array type '" << (token ? token : "(null)") << "'" << std::endl;
            break;
          }
        }
      }
      if (!complexArrayTypeRef)
      {
        for (std::vector<xs__import>::const_iterator i = schema.import.begin(); i != schema.import.end(); ++i)
        {
          xs__schema *s = (*i).schemaPtr();
          if (s)
          {
            token = qname_token(arrayType, s->targetNamespace);
            if (token)
            {
              for (std::vector<xs__complexType>::iterator j = s->complexType.begin(); j != s->complexType.end(); ++j)
              {
                if ((*j).name && !strcmp((*j).name, token))
                {
                  complexArrayTypeRef = &(*j);
                  if (vflag)
                    std::cerr << "    Found restriction array type '" << (token ? token : "(null)") << "'" << std::endl;
                  break;
                }
              }
              if (complexArrayTypeRef)
                break;
            }
          }
        }
      }
      if (!simpleArrayTypeRef && !complexArrayTypeRef)
      {
        if (is_builtin_qname(arrayType))
          schema.builtinType(arrayType);
        else if (!Wflag)
          std::cerr << "\nWarning: could not find restriction array type '" << arrayType << "' in schema '" << (schema.targetNamespace ? schema.targetNamespace : "(undefined)") << "'" << std::endl;
      }
#endif
    }
  }
  const char *token = qname_token(base, schema.targetNamespace);
  simpleTypeRef = NULL;
  if (token)
  {
    for (std::vector<xs__simpleType>::iterator i = schema.simpleType.begin(); i != schema.simpleType.end(); ++i)
    {
      if ((*i).name && !strcmp((*i).name, token))
      {
        simpleTypeRef = &(*i);
        if (vflag)
          std::cerr << "    Found restriction base type '" << (token ? token : "(null)") << "'" << std::endl;
        break;
      }
    }
  }
  if (!simpleTypeRef)
  {
    for (std::vector<xs__import>::const_iterator i = schema.import.begin(); i != schema.import.end(); ++i)
    {
      xs__schema *s = (*i).schemaPtr();
      if (s)
      {
        token = qname_token(base, s->targetNamespace);
        if (token)
        {
          for (std::vector<xs__simpleType>::iterator j = s->simpleType.begin(); j != s->simpleType.end(); ++j)
          {
            if ((*j).name && !strcmp((*j).name, token))
            {
              simpleTypeRef = &(*j);
              if (vflag)
                std::cerr << "    Found restriction base type '" << (token ? token : "(null)") << "'" << std::endl;
              break;
            }
          }
          if (simpleTypeRef)
            break;
        }
      }
    }
  }
  token = qname_token(base, schema.targetNamespace);
  complexTypeRef = NULL;
  if (token)
  {
    for (std::vector<xs__complexType>::iterator i = schema.complexType.begin(); i != schema.complexType.end(); ++i)
    {
      if ((*i).name && !strcmp((*i).name, token))
      {
        complexTypeRef = &(*i);
        if (vflag)
          std::cerr << "    Found restriction base type '" << (token ? token : "(null)") << "'" << std::endl;
        break;
      }
    }
  }
  if (!complexTypeRef)
  {
    for (std::vector<xs__import>::const_iterator i = schema.import.begin(); i != schema.import.end(); ++i)
    {
      xs__schema *s = (*i).schemaPtr();
      if (s)
      {
        token = qname_token(base, s->targetNamespace);
        if (token)
        {
          for (std::vector<xs__complexType>::iterator j = s->complexType.begin(); j != s->complexType.end(); ++j)
          {
            if ((*j).name && !strcmp((*j).name, token))
            {
              complexTypeRef = &(*j);
              if (vflag)
                std::cerr << "    Found restriction base type '" << (token ? token : "(null)") << "'" << std::endl;
              break;
            }
          }
          if (complexTypeRef)
            break;
        }
      }
    }
  }
  if (!simpleTypeRef && !complexTypeRef)
  {
    if (base)
    {
      if (is_builtin_qname(base))
        schema.builtinType(base);
      else if (!Wflag)
        std::cerr << "\nWarning: could not find restriction base type '" << base << "' in schema '" << (schema.targetNamespace ? schema.targetNamespace : "(undefined)") << "'" << std::endl;
    }
    else if (!simpleType)
    {
      std::cerr << "Restriction has no base" << std::endl;
    }
  }
  return SOAP_OK;
}

void xs__restriction::simpleTypePtr(xs__simpleType *simpleType)
{
  simpleTypeRef = simpleType;
}

void xs__restriction::complexTypePtr(xs__complexType *complexType)
{
  complexTypeRef = complexType;
}

xs__simpleType *xs__restriction::simpleTypePtr() const
{
  return simpleTypeRef;
}

xs__complexType *xs__restriction::complexTypePtr() const
{
  return complexTypeRef;
}

xs__simpleType *xs__restriction::simpleArrayTypePtr() const
{
  return simpleArrayTypeRef;
}

xs__complexType *xs__restriction::complexArrayTypePtr() const
{
  return complexArrayTypeRef;
}

void xs__restriction::mark()
{
  if (Oflag > 1)
  {
    if (simpleTypePtr())
      simpleTypePtr()->mark();
    if (complexTypePtr())
      complexTypePtr()->mark();
    if (simpleArrayTypePtr())
      simpleArrayTypePtr()->mark();
    if (complexArrayTypePtr())
      complexArrayTypePtr()->mark();
    if (simpleType)
      simpleType->mark();
    if (attributeGroup)
      attributeGroup->mark();
    if (group)
      group->mark();
    else if (all)
      all->mark();
    else if (choice)
      choice->mark();
    else if (sequence)
      sequence->mark();
    for (std::vector<xs__attribute>::iterator at = attribute.begin(); at != attribute.end(); ++at)
      (*at).mark();
  }
}

xs__list::xs__list()
{
  itemTypeRef = NULL;
}

int xs__list::traverse(xs__schema &schema)
{
  if (vflag)
    std::cerr << "   Analyzing schema list" << std::endl;
  if (restriction)
    restriction->traverse(schema);
  for (std::vector<xs__simpleType>::iterator i = simpleType.begin(); i != simpleType.end(); ++i)
    (*i).traverse(schema);
  itemTypeRef = NULL;
  const char *token = qname_token(itemType, schema.targetNamespace);
  if (token)
  {
    for (std::vector<xs__simpleType>::iterator i = schema.simpleType.begin(); i != schema.simpleType.end(); ++i)
    {
      if ((*i).name && !strcmp((*i).name, token))
      {
        itemTypeRef = &(*i);
        if (vflag)
          std::cerr << "    Found list itemType '" << (token ? token : "(null)") << "'" << std::endl;
        break;
      }
    }
  }
  if (!itemTypeRef)
  {
    for (std::vector<xs__import>::const_iterator i = schema.import.begin(); i != schema.import.end(); ++i)
    {
      xs__schema *s = (*i).schemaPtr();
      if (s)
      {
        token = qname_token(itemType, s->targetNamespace);
        if (token)
        {
          for (std::vector<xs__simpleType>::iterator j = s->simpleType.begin(); j != s->simpleType.end(); ++j)
          {
            if ((*j).name && !strcmp((*j).name, token))
            {
              itemTypeRef = &(*j);
              if (vflag)
                std::cerr << "    Found list itemType '" << (token ? token : "(null)") << "'" << std::endl;
              break;
            }
          }
          if (itemTypeRef)
            break;
        }
      }
    }
  }
  if (itemType && !itemTypeRef)
  {
    if (is_builtin_qname(itemType))
      schema.builtinType(itemType);
    else if (!Wflag)
      std::cerr << "\nWarning: could not find list itemType '" << itemType << "' in schema '" << (schema.targetNamespace ? schema.targetNamespace : "(undefined)") << "'" << std::endl;
  }
  return SOAP_OK;
}

void xs__list::itemTypePtr(xs__simpleType *simpleType)
{
  itemTypeRef = simpleType;
}

xs__simpleType *xs__list::itemTypePtr() const
{
  return itemTypeRef;
}

void xs__list::mark()
{
  if (Oflag > 1)
  {
    if (restriction)
      restriction->mark();
    for (std::vector<xs__simpleType>::iterator i = simpleType.begin(); i != simpleType.end(); ++i)
      (*i).mark();
  }
}

int xs__union::traverse(xs__schema &schema)
{
  if (vflag)
    std::cerr << "   Analyzing schema union" << std::endl;
  for (std::vector<xs__simpleType>::iterator i = simpleType.begin(); i != simpleType.end(); ++i)
    (*i).traverse(schema);
  return SOAP_OK;
}

void xs__union::mark()
{
  if (Oflag > 1)
  {
    for (std::vector<xs__simpleType>::iterator i = simpleType.begin(); i != simpleType.end(); ++i)
      (*i).mark();
  }
}

int xs__all::traverse(xs__schema &schema)
{
  if (vflag)
    std::cerr << "   Analyzing schema all" << std::endl;
  for (std::vector<xs__element>::iterator i = element.begin(); i != element.end(); ++i)
    (*i).traverse(schema);
  return SOAP_OK;
}

void xs__all::mark()
{
  if (Oflag > 1)
    for (std::vector<xs__element>::iterator i = element.begin(); i != element.end(); ++i)
      (*i).mark();
}

int xs__contents::traverse(xs__schema &schema)
{
  switch (__union)
  {
    case SOAP_UNION_xs__union_content_element:
      if (__content.element)
        __content.element->traverse(schema);
      break;
    case SOAP_UNION_xs__union_content_group:
      if (__content.group)
        __content.group->traverse(schema);
      break;
    case SOAP_UNION_xs__union_content_choice:
      if (__content.choice)
        __content.choice->traverse(schema);
      break;
    case SOAP_UNION_xs__union_content_sequence:
      if (__content.sequence)
        __content.sequence->traverse(schema);
      break;
    case SOAP_UNION_xs__union_content_any:
      if (__content.any)
        __content.any->traverse(schema);
      break;
  }
  return SOAP_OK;
}

void xs__contents::mark()
{
  if (Oflag > 1)
  {
    switch (__union)
    {
      case SOAP_UNION_xs__union_content_element:
        if (__content.element)
          __content.element->mark();
        break;
      case SOAP_UNION_xs__union_content_group:
        if (__content.group)
          __content.group->mark();
        break;
      case SOAP_UNION_xs__union_content_choice:
        if (__content.choice)
          __content.choice->mark();
        break;
      case SOAP_UNION_xs__union_content_sequence:
        if (__content.sequence)
          __content.sequence->mark();
        break;
      case SOAP_UNION_xs__union_content_any:
        if (__content.any)
          __content.any->mark();
        break;
    }
  }
}

xs__seqchoice::xs__seqchoice()
{
  minOccurs = NULL;
  maxOccurs = NULL;
  annotation = NULL;
  schemaRef = NULL;
}

int xs__seqchoice::traverse(xs__schema &schema)
{
  if (vflag)
    std::cerr << "   Analyzing schema sequence/choice" << std::endl;
  schemaRef = &schema;
  for (std::vector<xs__contents>::iterator c = __contents.begin(); c != __contents.end(); ++c)
    (*c).traverse(schema);
  if (Oflag > 0)
  {
    SetOfString members;
    for (std::vector<xs__contents>::iterator c = __contents.begin(); c != __contents.end(); )
    {
      if ((*c).__union == SOAP_UNION_xs__union_content_sequence)
      {
        for (std::vector<xs__contents>::iterator c1 = (*c).__content.sequence->__contents.begin(); c1 != (*c).__content.sequence->__contents.end(); )
        {
          if ((*c1).__union == SOAP_UNION_xs__union_content_element)
          {
            xs__element *e = (*c1).__content.element;
            if (e->elementPtr())
            {
              e = e->elementPtr();
            }
            if (e->name)
            {
              if (members.find(e->name) != members.end())
              {
                (*c).__content.sequence->__contents.erase(c1);
                if (!Wflag)
                  std::cerr << "\nOptimization: removed duplicate element '" << e->name << "' from nested choice/sequence" << std::endl;
              }
              else
              {
                members.insert(e->name);
                ++c1;
              }
            }
            else
            {
              ++c1;
            }
          }
          else
          {
            ++c1;
          }
        }
        ++c;
      }
      else if ((*c).__union == SOAP_UNION_xs__union_content_element)
      {
        xs__element *e = (*c).__content.element;
        if (e->elementPtr())
        {
          e = e->elementPtr();
        }
        if (e->name)
        {
          if (members.find(e->name) != members.end())
          {
            __contents.erase(c);
            std::cerr << "\nOptimization: removed duplicate element '" << e->name << "' from nested xs:sequence/xs:choice" << std::endl;
          }
          else
          {
            members.insert(e->name);
            ++c;
          }
        }
        else
        {
          ++c;
        }
      }
      else
      {
        ++c;
      }
    }
  }
  return SOAP_OK;
}

void xs__seqchoice::schemaPtr(xs__schema *schema)
{
  schemaRef = schema;
}

xs__schema *xs__seqchoice::schemaPtr() const
{
  return schemaRef;
}

void xs__seqchoice::mark()
{
  if (Oflag > 1)
    for (std::vector<xs__contents>::iterator c = __contents.begin(); c != __contents.end(); ++c)
      (*c).mark();
}

xs__attributeGroup::xs__attributeGroup()
{
  schemaRef = NULL;
  attributeGroupRef = NULL;
  used = false;
}

int xs__attributeGroup::traverse(xs__schema& schema)
{
  if (vflag)
    std::cerr << "   Analyzing schema attributeGroup" << std::endl;
  schemaRef = &schema;
  for (std::vector<xs__attribute>::iterator at = attribute.begin(); at != attribute.end(); ++at)
    (*at).traverse(schema);
  for (std::vector<xs__attributeGroup>::iterator ag = attributeGroup.begin(); ag != attributeGroup.end(); ++ag)
    (*ag).traverse(schema);
  attributeGroupRef = NULL;
  if (ref)
  {
    const char *token = qname_token(ref, schema.targetNamespace);
    if (token)
    {
      for (std::vector<xs__attributeGroup>::iterator i = schema.attributeGroup.begin(); i != schema.attributeGroup.end(); ++i)
      {
        if ((*i).name && !strcmp((*i).name, token))
        {
          attributeGroupRef = &(*i);
          if (vflag)
            std::cerr << "    Found attributeGroup '" << (name ? name : "(null)") << "' ref '" << (token ? token : "(null)") << "'" << std::endl;
          break;
        }
      }
    }
    if (!attributeGroupRef)
    {
      for (std::vector<xs__import>::const_iterator i = schema.import.begin(); i != schema.import.end(); ++i)
      {
        xs__schema *s = (*i).schemaPtr();
        if (s)
        {
          token = qname_token(ref, s->targetNamespace);
          if (token)
          {
            for (std::vector<xs__attributeGroup>::iterator j = s->attributeGroup.begin(); j != s->attributeGroup.end(); ++j)
            {
              if ((*j).name && !strcmp((*j).name, token))
              {
                attributeGroupRef = &(*j);
                if (vflag)
                    std::cerr << "    Found attribute Group '" << (name ? name : "(null)") << "' ref '" << (token ? token : "(null)") << "'" << std::endl;
                break;
              }
            }
            if (attributeGroupRef)
              break;
          }
        }
      }
    }
    if (!attributeGroupRef)
      if (!Wflag)
        std::cerr << "\nWarning: could not find the referenced attributeGroup '" << (name ? name : "") << "' ref '" << (ref ? ref : "(undefined)") << "' in schema '" << (schema.targetNamespace ? schema.targetNamespace : "(undefined)") << "'" << std::endl;
  }
  return SOAP_OK;
}

void xs__attributeGroup::schemaPtr(xs__schema *schema)
{
  schemaRef = schema;
}

void xs__attributeGroup::attributeGroupPtr(xs__attributeGroup *attributeGroup)
{
  attributeGroupRef = attributeGroup;
}

xs__schema *xs__attributeGroup::schemaPtr() const
{
  return schemaRef;
}

xs__attributeGroup *xs__attributeGroup::attributeGroupPtr() const
{
  return attributeGroupRef;
}

void xs__attributeGroup::mark()
{
  if (Oflag > 1 && !used)
  {
    used = true;
    if (attributeGroupPtr())
      attributeGroupPtr()->mark();
    for (std::vector<xs__attribute>::iterator at = attribute.begin(); at != attribute.end(); ++at)
      (*at).mark();
    for (std::vector<xs__attributeGroup>::iterator ag = attributeGroup.begin(); ag != attributeGroup.end(); ++ag)
      (*ag).mark();
  }
}

int xs__any::traverse(xs__schema &schema)
{
  if (vflag)
    std::cerr << "   Analyzing schema any" << std::endl;
  for (std::vector<xs__element>::iterator i = element.begin(); i != element.end(); ++i)
    (*i).traverse(schema);
  return SOAP_OK;
}

void xs__any::mark()
{
  if (Oflag > 1)
    for (std::vector<xs__element>::iterator i = element.begin(); i != element.end(); ++i)
      (*i).mark();
}

xs__group::xs__group()
{
  schemaRef = NULL;
  groupRef = NULL;
  used = false;
}

int xs__group::traverse(xs__schema &schema)
{
  if (vflag)
    std::cerr << "   Analyzing schema group" << std::endl;
  schemaRef = &schema;
  if (all)
    all->traverse(schema);
  if (choice)
    choice->traverse(schema);
  if (sequence)
    sequence->traverse(schema);
  groupRef = NULL;
  if (ref)
  {
    const char *token = qname_token(ref, schema.targetNamespace);
    if (token)
    {
      for (std::vector<xs__group>::iterator i = schema.group.begin(); i != schema.group.end(); ++i)
      {
        if ((*i).name && !strcmp((*i).name, token))
        {
          groupRef = &(*i);
          if (vflag)
              std::cerr << "    Found group '" << (name ? name : "(null)") << "' ref '" << (token ? token : "(null)") << "'" << std::endl;
          break;
        }
      }
    }
    if (!groupRef)
    {
      for (std::vector<xs__import>::const_iterator i = schema.import.begin(); i != schema.import.end(); ++i)
      {
        xs__schema *s = (*i).schemaPtr();
        if (s)
        {
          token = qname_token(ref, s->targetNamespace);
          if (token)
          {
            for (std::vector<xs__group>::iterator j = s->group.begin(); j != s->group.end(); ++j)
            {
              if ((*j).name && !strcmp((*j).name, token))
              {
                groupRef = &(*j);
                if (vflag)
                    std::cerr << "    Found group '" << (name ? name : "(null)") << "' ref '" << (token ? token : "(null)") << "'" << std::endl;
                break;
              }
            }
            if (groupRef)
              break;
          }
        }
      }
    }
    if (!groupRef)
      if (!Wflag)
        std::cerr << "\nWarning: could not find the referenced group '" << (name ? name : "") << "' ref '" << (ref ? ref : "(undefined)") << "' in schema '" << (schema.targetNamespace ? schema.targetNamespace : "(undefined)") << "'" << std::endl;
  }
  return SOAP_OK;
}

void xs__group::schemaPtr(xs__schema *schema)
{
  schemaRef = schema;
}

xs__schema* xs__group::schemaPtr() const
{
  return schemaRef;
}

void xs__group::groupPtr(xs__group *group)
{
  groupRef = group;
}

xs__group* xs__group::groupPtr() const
{
  return groupRef;
}

void xs__group::mark()
{
  if (Oflag > 1 && !used)
  {
    used = true;
    if (groupPtr())
      groupPtr()->mark();
    if (all)
      all->mark();
    if (choice)
      choice->mark();
    if (sequence)
      sequence->mark();
  }
}

int xs__enumeration::traverse(xs__schema &schema)
{
  (void)schema;
  if (vflag)
    std::cerr << "   Analyzing schema enumeration '" << (value ? value : "(null)") << "'" << std::endl;
  return SOAP_OK;
}

int xs__pattern::traverse(xs__schema &schema)
{
  (void)schema;
  if (vflag)
    std::cerr << "   Analyzing schema pattern" << std::endl;
  return SOAP_OK;
}

////////////////////////////////////////////////////////////////////////////////
//
//      I/O
//
////////////////////////////////////////////////////////////////////////////////

std::ostream &operator<<(std::ostream &o, const xs__schema &e)
{
  if (!e.soap)
  {
    struct soap soap;
    soap_init2(&soap, SOAP_IO_DEFAULT, SOAP_XML_TREE | SOAP_C_UTFSTRING);
    soap_set_namespaces(&soap, namespaces);
    e.soap_serialize(&soap);
    soap_begin_send(&soap);
    e.soap_out(&soap, "xs:schema", 0, NULL);
    soap_end_send(&soap);
    soap_end(&soap);
    soap_done(&soap);
  }
  else
  {
    std::ostream *os = e.soap->os;
    e.soap->os = &o;
    e.soap_serialize(e.soap);
    soap_begin_send(e.soap);
    e.soap_out(e.soap, "xs:schema", 0, NULL);
    soap_end_send(e.soap);
    e.soap->os = os;
  }
  return o;
}

std::istream &operator>>(std::istream &i, xs__schema &e)
{
  if (!e.soap)
  {
    e.soap = soap_new();
    soap_set_namespaces(e.soap, namespaces);
  }
  std::istream *is = e.soap->is;
  e.soap->is = &i;
  if (soap_begin_recv(e.soap)
   || !e.soap_in(e.soap, "xs:schema", NULL)
   || soap_end_recv(e.soap))
  {
    // handle error? Note: e.soap->error is set and app should check
  }
  e.soap->is = is;
  return i;
}

