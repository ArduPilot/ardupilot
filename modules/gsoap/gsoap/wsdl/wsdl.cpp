/*
        wsdl.cpp

        WSDL 1.1 and WSDL 2.0 binding schema implementation

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

#include "wsdlH.h"
#include "includes.h"

extern struct Namespace namespaces[];

const char *qname_token(const char *QName, const char *URI)
{
  if (QName && QName[0] == '"' && QName[1] == '"' && QName[2] == ':')
    return QName + 3;
  if (QName && URI && *QName == '"') // QNames are stored in the format "URI":name, unless the URI is in the nsmap
  {
    size_t n = strlen(URI);
    if (!strncmp(QName + 1, URI, n) && QName[n + 1] == '"')
      return QName + n + 3;
  }
  return NULL;
}

int is_builtin_qname(const char *QName)
{
  if (iflag)
    return 1;
  if (QName)
  {
    if (*QName == '#') // reserved QNames
      return 0;
    if (*QName != '"')
      return 1; // if the QName does not start with a ", it must be in the nsmap
    const char *s = strchr(QName + 1, '"');
    if (s)
    {
      size_t n = s - QName - 1;
      for (SetOfString::const_iterator i = exturis.begin(); i != exturis.end(); ++i)
        if (strlen(*i) == n && !strncmp(QName + 1, *i, n))
          return 1; // QName is in exturis
    }
  }
  return 0;
}

xsd__QName make_qname(xs__schema& schema, const char *name)
{
  const char *URI = schema.targetNamespace ? schema.targetNamespace : "";
  size_t n = strlen(URI) + strlen(name) + 3;
  char *s = (char*)soap_malloc(schema.soap, n + 1);
  (SOAP_SNPRINTF(s, n + 1, n), "\"%s\":%s", URI, name);
  return s;
}

////////////////////////////////////////////////////////////////////////////////
//
//      wsdl
//
////////////////////////////////////////////////////////////////////////////////

extern "C" {
int warn_ignore(struct soap*, const char*);
int show_ignore(struct soap*, const char*);
}

wsdl__definitions::wsdl__definitions()
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
  if (vflag)
    soap->fignore = show_ignore;
  else
    soap->fignore = warn_ignore;
  soap->encodingStyle = NULL;
  soap->proxy_host = proxy_host;
  soap->proxy_port = proxy_port;
  soap->proxy_userid = proxy_userid;
  soap->proxy_passwd = proxy_passwd;
  name = NULL;
  targetNamespace = (char*)"";
  documentation = NULL;
  types = NULL;
  updated = false;
  location = NULL;
  redirs = 0;
  appRef = NULL;
  used = false;
}

wsdl__definitions::wsdl__definitions(struct soap *copy)
{
  soap = soap_copy(copy);
  soap->socket = SOAP_INVALID_SOCKET;
  soap->recvfd = 0;
  soap->sendfd = 1;
  soap_strcpy(soap->host, sizeof(soap->host), copy->host);
  soap_default(soap);
  soap->fignore = warn_ignore;
  soap->encodingStyle = NULL;
  name = NULL;
  targetNamespace = (char*)"";
  documentation = NULL;
  types = NULL;
  updated = false;
  location = NULL;
  redirs = 0;
  appRef = NULL;
}

wsdl__definitions::wsdl__definitions(struct soap *copy, const char *cwd, const char *loc, const char *relloc)
{
  soap = soap_copy(copy);
  soap->socket = SOAP_INVALID_SOCKET;
  soap->recvfd = 0;
  soap->sendfd = 1;
  soap_strcpy(soap->host, sizeof(soap->host), copy->host);
  soap_default(soap);
  soap->fignore = warn_ignore;
  soap->encodingStyle = NULL;
  name = NULL;
  targetNamespace = (char*)"";
  documentation = NULL;
  types = NULL;
  updated = false;
  location = NULL;
  redirs = 0;
  appRef = NULL;
  read(cwd, loc, relloc);
}

wsdl__definitions::~wsdl__definitions()
{
  soap_destroy(soap);
  soap_end(soap);
  soap_done(soap);
  free(soap);
}

int wsdl__definitions::get(struct soap *soap)
{
  (void)soap;
  return preprocess();
}

int wsdl__definitions::read(int num, char **loc)
{
  if (num <= 0)
    return read(NULL, NULL, NULL);
  if (num == 1)
    return read(NULL, loc[0], NULL);
  wsdl__import im;
  im.namespace_ = NULL;
  name = soap_strdup(soap, "WSDL");
  targetNamespace = soap_strdup(soap, "");;
  for (int i = 0; i < num; i++)
  {
    im.location = loc[i];
    import.push_back(im);
  }
  return preprocess();
}

int wsdl__definitions::read(const char *cwd, const char *loc, const char *relloc)
{
  const char *cwd_temp;
  if (!cwd)
    cwd = cwd_path;
  if (vflag)
    fprintf(stderr, "\nOpening WSDL/WADL or XSD '%s' relative to '%s'\n", loc ? loc : "(stdin)", cwd ? cwd : "./");
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
      fprintf(stderr, "%*sConnecting to '%s' to retrieve WSDL/WADL or XSD...", 2*openfiles, "", loc);
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
      fprintf(stderr, "%*sConnecting to '%s' to retrieve '%s'...", 2*openfiles, "", location, loc);
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
          fprintf(stderr, "\nCannot open '%s'\n", loc);
          exit(1);
        }
      }
      else
      {
        location = soap_strdup(soap, loc);
      }
      fprintf(stderr, "%*sReading '%s'...\n", 2*openfiles, "", location);
      openfiles++;
    }
  }
  cwd_temp = cwd_path;
  cwd_path = location;
  if (!soap_begin_recv(soap))
    this->soap_in(soap, "wsdl:", NULL);
  if (soap->error)
  {
    // handle sloppy WSDLs that import schemas at the top level rather than
    // importing them in <types>
    // handle WADL when used in place of WSDL
    if (soap->error == SOAP_TAG_MISMATCH && soap->level == 0)
    {
      soap_retry(soap);
      xs__schema *schema = soap_new_xs__schema(soap);
      schema->sourceLocation(location);
      schema->soap_in(soap, "xs:schema", NULL);
      if (soap->error == SOAP_TAG_MISMATCH && soap->level == 0)
      {
        soap_retry(soap);
        wadl__application *app = soap_new_wadl__application(soap);
        app->soap_in(soap, "wadl:application", NULL);
        if (soap->error)
        {
          fprintf(stderr, "\nAn error occurred while parsing WSDL/WADL or XSD from '%s'\n", loc ? loc : "(stdin)");
          soap_print_fault(soap, stderr);
          if (soap->error < 200)
            soap_print_fault_location(soap, stderr);
          exit(1);
        }
        appRef = app;
        preprocess();
      }
      else
      {
        if (soap->error)
        {
          fprintf(stderr, "\nAn error occurred while parsing WSDL/WADL or XSD from '%s'\n", loc ? loc : "(stdin)");
          soap_print_fault(soap, stderr);
          if (soap->error < 200)
            soap_print_fault_location(soap, stderr);
          exit(1);
        }
        name = NULL;
        targetNamespace = schema->targetNamespace;
        if (vflag)
          std::cerr << "Found schema '" << (targetNamespace ? targetNamespace : "(null)") << "' when expecting WSDL" << std::endl;
        types = soap_new_wsdl__types(soap);
        types->documentation = NULL;
        types->xs__schema_.push_back(schema);
        types->preprocess(*this);
      }
    }
    // check HTTP redirect (socket was closed)
    else if ((soap->error >= 301 && soap->error <= 303) || soap->error == 307)
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
      }
      else
      {
        fprintf(stderr, "Authentication failed, use option -r:uid:pwd and (re)build with OpenSSL to enable digest authentication\n");
      }
      return r;
    }
    else
    {
      fprintf(stderr, "\nAn error occurred while parsing WSDL/WADL and XSD from '%s'\n", loc ? loc : "(stdin)");
      soap_print_fault(soap, stderr);
      if (soap->error < 200)
        soap_print_fault_location(soap, stderr);
      exit(1);
    }
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

int wsdl__definitions::preprocess()
{
  if (vflag)
    std::cerr << "Preprocessing wsdl definitions '" << (location ? location : "(null)") << "' namespace '" << (targetNamespace ? targetNamespace : "(null)") << "'" << std::endl;
  // process WADL app
  if (appPtr())
    appPtr()->preprocess(*this);
  // process import
  for (std::vector<wsdl__import>::iterator im1 = import.begin(); im1 != import.end(); ++im1)
    (*im1).preprocess(*this);
  // merge nested imported WSDLs into single import list
again:
  for (std::vector<wsdl__import>::iterator im2 = import.begin(); im2 != import.end(); ++im2)
  {
    if ((*im2).definitionsPtr())
    {
      for (std::vector<wsdl__import>::iterator i = (*im2).definitionsPtr()->import.begin(); i != (*im2).definitionsPtr()->import.end(); ++i)
      {
        if ((*i).definitionsPtr())
        {
          bool found = false;
          if (vflag)
            std::cerr << "Import WSDL '" << ((*i).location ? (*i).location : "(null)") << std::endl;
          for (std::vector<wsdl__import>::iterator j = import.begin(); j != import.end(); ++j)
          {
            if ((*i).definitionsPtr() == (*j).definitionsPtr()
             || ((*i).location && (*j).location && !strcmp((*i).location, (*j).location)))
            {
              found = true;
              break;
            }
          }
          if (!found)
          {
            if (vflag)
              std::cerr << "Adding imported WSDL '" << ((*i).location ? (*i).location : "(null)") << "' to '" << (location ? location : "(null)") << "' ('" << (name ? name : "(null)") << "') namespace '" << (targetNamespace ? targetNamespace : "(null)") << "'" << std::endl;
            import.push_back(*i);
            goto again;
          }
        }
      }
    }
  }
  // merge <types>
  for (std::vector<wsdl__import>::iterator im3 = import.begin(); im3 != import.end(); ++im3)
  {
    if ((*im3).definitionsPtr() && (*im3).definitionsPtr()->types)
    {
      if (!types)
      {
        types = soap_new_wsdl__types(soap);
        types->soap_default(soap);
      }
      // merge <types>, check for duplicates, add namespaces for sloppy imports
      for (std::vector<xs__schema*>::const_iterator i = (*im3).definitionsPtr()->types->xs__schema_.begin(); i != (*im3).definitionsPtr()->types->xs__schema_.end(); ++i)
      {
        bool found = false;
        std::vector<xs__schema*>::const_iterator j;
        if (!(*i)->targetNamespace)
        {
          if (!zflag || zflag > 10)
          {
            (*i)->targetNamespace = (char*)"";
          }
          else
          {
            (*i)->targetNamespace = targetNamespace;
            if (!Wflag)
              std::cerr << "\nWarning: schema imported without namespace, assigning namespace '" << (targetNamespace ? targetNamespace : "(null)") << "'" << std::endl;
          }
        }
        for (j = types->xs__schema_.begin(); j != types->xs__schema_.end(); ++j)
        {
          if ((*j)->targetNamespace && !strcmp((*i)->targetNamespace, (*j)->targetNamespace))
          {
            found = true;
            break;
          }
        }
        // add new schema only if not already in <types>, otherwise merge schema components
        if (found)
        {
          if (vflag)
            std::cerr << "\nSchemas with identical namespace '" << ((*i)->targetNamespace ? (*i)->targetNamespace : "(null)") << "' merged in WSDL '" << (name ? name : "(null)") << "' namespace '" << (targetNamespace ? targetNamespace : "(null)") << "'" << std::endl;
          (*j)->insert(*(*i));
        }
        else
        {
          if (vflag)
            std::cerr << "Adding schema with namespace '" << ((*i)->targetNamespace ? (*i)->targetNamespace : "(null)") << "' to types in WSDL '" << (name ? name : "(null)") << "' namespace '" << (targetNamespace ? targetNamespace : "(null)") << "'" << std::endl;
          types->xs__schema_.push_back(*i);
        }
      }
      // make imported WSDL <types> point to merged <types>
      (*im3).definitionsPtr()->types = types;
    }
  }
  // process the types
  if (types)
    types->preprocess(*this);
  return SOAP_OK;
}

int wsdl__definitions::traverse()
{
  if (updated)
    return SOAP_OK;
  if (vflag)
    std::cerr << "Analyzing definitions '" << (name ? name : "(null)") << "' in wsdl namespace '" << (targetNamespace ? targetNamespace : "(null)") << "'" << std::endl;
  updated = true;
  if (!targetNamespace)
  {
    if (vflag)
      std::cerr << "\nWarning: wsdl '" << (name ? name : "(null)") << "' has no targetNamespace" << std::endl;
    targetNamespace = soap_strdup(soap, "");
  }
  // process import first
  for (std::vector<wsdl__import>::iterator im = import.begin(); im != import.end(); ++im)
    (*im).traverse(*this);
  // then process the types
  if (types)
    types->traverse(*this);
  // process WADL
  if (appRef)
    appRef->traverse(*this);
  // process messages before portType
  for (std::vector<wsdl__message>::iterator mg = message.begin(); mg != message.end(); ++mg)
    (*mg).traverse(*this);
  // process portType before binding
  for (std::vector<wsdl__portType>::iterator pt = portType.begin(); pt != portType.end(); ++pt)
    (*pt).traverse(*this);
  // process interface before binding WSDL 2.0
  for (std::vector<wsdl__portType>::iterator in = interface_.begin(); in != interface_.end(); ++in)
    (*in).traverse(*this);
  // process binding
  for (std::vector<wsdl__binding>::iterator bg = binding.begin(); bg != binding.end(); ++bg)
    (*bg).traverse(*this);
  // process service
  for (std::vector<wsdl__service>::iterator sv = service.begin(); sv != service.end(); ++sv)
    (*sv).traverse(*this);
  if (vflag)
    std::cerr << "End of definitions '" << (name ? name : "(null)") << "' namespace '" << (targetNamespace ? targetNamespace : "(null)") << "'" << std::endl;
  for (std::vector<wsp__Policy>::iterator wsp = wsp__Policy_.begin(); wsp != wsp__Policy_.end(); ++wsp)
    (*wsp).traverse(*this);
  for (std::vector<plnk__tPartnerLinkType>::iterator plt = plnk__partnerLinkType.begin(); plt != plnk__partnerLinkType.end(); ++plt)
    (*plt).traverse(*this);
  for (std::vector<vprop__tProperty>::iterator vp = vprop__property.begin(); vp != vprop__property.end(); ++vp)
    (*vp).traverse(*this);
  for (std::vector<vprop__tPropertyAlias>::iterator vpa = vprop__propertyAlias.begin(); vpa != vprop__propertyAlias.end(); ++vpa)
    (*vpa).traverse(*this);
  return SOAP_OK;
}

const char *wsdl__definitions::sourceLocation()
{
  return location;
}

char *wsdl__definitions::absoluteLocation(const char *loc) const
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

int wsdl__definitions::error()
{
  return soap->error;
}

bool wsdl__definitions::is_updated()
{
  return updated;
}

void wsdl__definitions::print_fault()
{
  soap_print_fault(soap, stderr);
  if (soap->error < 200)
    soap_print_fault_location(soap, stderr);
}

void wsdl__definitions::builtinType(const char *type)
{
  builtinTypeSet.insert(type);
}

void wsdl__definitions::builtinTypeDerivation(xs__schema& schema, const char *base, const char *derived)
{
  builtinTypeMap[make_qname(schema, derived)] = base;
}

void wsdl__definitions::builtinTypes(const SetOfString& types)
{
  for (SetOfString::const_iterator tp = types.begin(); tp != types.end(); ++tp)
    builtinTypeSet.insert(*tp);
}

void wsdl__definitions::builtinTypeDerivations(const MapOfStringToString& derivations)
{
  for (MapOfStringToString::const_iterator dp = derivations.begin(); dp != derivations.end(); ++dp)
    builtinTypeMap.insert(*dp);
}

void wsdl__definitions::builtinElement(const char *element)
{
  builtinElementSet.insert(element);
}

void wsdl__definitions::builtinElements(const SetOfString& elements)
{
  for (SetOfString::const_iterator el = elements.begin(); el != elements.end(); ++el)
    builtinElementSet.insert(*el);
}

void wsdl__definitions::builtinAttribute(const char *attribute)
{
  builtinAttributeSet.insert(attribute);
}

void wsdl__definitions::builtinAttributes(const SetOfString& attributes)
{
  for (SetOfString::const_iterator at = attributes.begin(); at != attributes.end(); ++at)
    builtinAttributeSet.insert(*at);
}

const SetOfString& wsdl__definitions::builtinTypes() const
{
  return builtinTypeSet;
}

const MapOfStringToString& wsdl__definitions::builtinTypeDerivations() const
{
  return builtinTypeMap;
}

const SetOfString& wsdl__definitions::builtinElements() const
{
  return builtinElementSet;
}

const SetOfString& wsdl__definitions::builtinAttributes() const
{
  return builtinAttributeSet;
}

void wsdl__definitions::appPtr(wadl__application *app)
{
  appRef = app;
}

wadl__application *wsdl__definitions::appPtr() const
{
  return appRef;
}

void wsdl__definitions::mark()
{
  if (Oflag > 1 && !used)
  {
    used = true;
    for (std::vector<wsdl__import>::iterator im = import.begin(); im != import.end(); ++im)
      (*im).mark();
    if (types)
      types->mark();
    if (appRef)
      appRef->mark();
    for (std::vector<wsdl__portType>::iterator pt = interface_.begin(); pt != interface_.end(); ++pt)
      (*pt).mark();
    for (std::vector<wsdl__message>::iterator mg = message.begin(); mg != message.end(); ++mg)
      (*mg).mark();
    for (std::vector<wsdl__binding>::iterator bg = binding.begin(); bg != binding.end(); ++bg)
      (*bg).mark();
  }
}

int wsdl__service::traverse(wsdl__definitions& definitions)
{
  if (vflag)
    std::cerr << "Analyzing service '" << (name ? name : "(null)") << "' in wsdl namespace '" << (definitions.targetNamespace ? definitions.targetNamespace : "(null)") << "'" << std::endl;
  // process ports
  for (std::vector<wsdl__port>::iterator pt = port.begin(); pt != port.end(); ++pt)
    (*pt).traverse(definitions);
  for (std::vector<wsdl__port>::iterator ep = endpoint.begin(); ep != endpoint.end(); ++ep)
    (*ep).traverse(definitions);
  for (std::vector<wsp__Policy>::iterator py = wsp__Policy_.begin(); py != wsp__Policy_.end(); ++py)
    (*py).traverse(definitions);
  for (std::vector<wsp__PolicyReference>::iterator pr = wsp__PolicyReference_.begin(); pr != wsp__PolicyReference_.end(); ++pr)
    (*pr).traverse(definitions);
  return SOAP_OK;
}

wsdl__port::wsdl__port()
{
  bindingRef = NULL;
}

int wsdl__port::traverse(wsdl__definitions& definitions)
{
  if (vflag)
    std::cerr << " Analyzing service port/endpoint in wsdl namespace '" << (definitions.targetNamespace ? definitions.targetNamespace : "(null)") << "'" << std::endl;
  // search binding name
  const char *token = qname_token(binding, definitions.targetNamespace);
  bindingRef = NULL;
  if (token)
  {
    for (std::vector<wsdl__binding>::iterator binding = definitions.binding.begin(); binding != definitions.binding.end(); ++binding)
    {
      if ((*binding).name && !strcmp((*binding).name, token))
      {
        bindingRef = &(*binding);
        if (vflag)
          std::cerr << "  Found port/endpoint '" << (name ? name : "(null)") << "' binding '" << (token ? token : "(null)") << "'" << std::endl;
        break;
      }
    }
  }
  if (!bindingRef)
  {
    for (std::vector<wsdl__import>::iterator import = definitions.import.begin(); import != definitions.import.end(); ++import)
    {
      wsdl__definitions *importdefinitions = (*import).definitionsPtr();
      if (importdefinitions)
      {
        token = qname_token(binding, importdefinitions->targetNamespace);
        if (token)
        {
          for (std::vector<wsdl__binding>::iterator binding = importdefinitions->binding.begin(); binding != importdefinitions->binding.end(); ++binding)
          {
            if ((*binding).name && !strcmp((*binding).name, token))
            {
              bindingRef = &(*binding);
              if (vflag)
                std::cerr << "  Found port/endpoint '" << (name ? name : "(null)") << "' binding '" << (token ? token : "(null)") << "'" << std::endl;
              break;
            }
          }
        }
      }
    }
  }
  if (!bindingRef)
    if (!Wflag)
      std::cerr << "\nWarning: no port/endpoint '" << (name ? name : "(undefined)") << "' binding '" << (binding ? binding : "(undefined)") << "' in wsdl definitions '" << (definitions.name ? definitions.name : "(undefined)") << "' namespace '" << (definitions.targetNamespace ? definitions.targetNamespace : "(undefined)") << "'" << std::endl;
  if (wsp__Policy_)
    wsp__Policy_->traverse(definitions);
  if (wsp__PolicyReference_)
    wsp__PolicyReference_->traverse(definitions);
  return SOAP_OK;
}

void wsdl__port::bindingPtr(wsdl__binding *binding)
{
  bindingRef = binding;
  if (!bindingRef && vflag)
    std::cerr << "\nWarning: wsdl__port binding set to NULL" << std::endl;
}

wsdl__binding *wsdl__port::bindingPtr() const
{
  return bindingRef;
}

wsdl__binding::wsdl__binding()
{
  portTypeRef = NULL;
}

int wsdl__binding::traverse(wsdl__definitions& definitions)
{
  const char *token;
  if (vflag)
    std::cerr << " Analyzing binding '" << (name ? name : "(null)") << "' in wsdl namespace '" << (definitions.targetNamespace ? definitions.targetNamespace : "(null)") << "'" << std::endl;
  portTypeRef = NULL;
  if (interface_) // WSDL 2.0
    token = qname_token(interface_, definitions.targetNamespace);
  else
    token = qname_token(type, definitions.targetNamespace);
  if (token)
  {
    for (std::vector<wsdl__portType>::iterator portType = definitions.portType.begin(); portType != definitions.portType.end(); ++portType)
    {
      if ((*portType).name && !strcmp((*portType).name, token))
      {
        portTypeRef = &(*portType);
        if (vflag)
          std::cerr << "  Found binding '" << (name ? name : "(null)") << "' portType '" << (token ? token : "(null)") << "'" << std::endl;
        break;
      }
    }
    // WSDL 2.0
    for (std::vector<wsdl__portType>::iterator i = definitions.interface_.begin(); i != definitions.interface_.end(); ++i)
    {
      if ((*i).name && !strcmp((*i).name, token))
      {
        portTypeRef = &(*i);
        if (vflag)
          std::cerr << "  Found binding '" << (name ? name : "(null)") << "' interface '" << (token ? token : "(null)") << "'" << std::endl;
        break;
      }
    }
  }
  if (!portTypeRef)
  {
    for (std::vector<wsdl__import>::iterator import = definitions.import.begin(); import != definitions.import.end(); ++import)
    {
      wsdl__definitions *importdefinitions = (*import).definitionsPtr();
      if (importdefinitions)
      {
        if (interface_) // WSDL 2.0
          token = qname_token(interface_, importdefinitions->targetNamespace);
        else
          token = qname_token(type, importdefinitions->targetNamespace);
        if (token)
        {
          for (std::vector<wsdl__portType>::iterator portType = importdefinitions->portType.begin(); portType != importdefinitions->portType.end(); ++portType)
          {
            if ((*portType).name && !strcmp((*portType).name, token))
            {
              portTypeRef = &(*portType);
              if (vflag)
                std::cerr << "  Found binding '" << (name ? name : "(null)") << "' portType '" << (token ? token : "(null)") << "'" << std::endl;
              break;
            }
          }
          // WSDL 2.0
          for (std::vector<wsdl__portType>::iterator i = importdefinitions->interface_.begin(); i != importdefinitions->interface_.end(); ++i)
          {
            if ((*i).name && !strcmp((*i).name, token))
            {
              portTypeRef = &(*i);
              if (vflag)
                std::cerr << "  Found binding '" << (name ? name : "(null)") << "' interface '" << (token ? token : "(null)") << "'" << std::endl;
              break;
            }
          }
        }
      }
    }
  }
  if (!portTypeRef)
  {
    if (!Wflag)
    {
      if (interface_)
        std::cerr << "\nWarning: no binding '" << (name ? name : "(undefined)") << "' interface '" << (interface_ ? interface_ : "(undefined)") << "' in wsdl definitions '" << (definitions.name ? definitions.name : "(undefined)") << "' namespace '" << (definitions.targetNamespace ? definitions.targetNamespace : "(undefined)") << "'" << std::endl;
      else
        std::cerr << "\nWarning: no binding '" << (name ? name : "(undefined)") << "' portType '" << (type ? type : "(undefined)") << "' in wsdl definitions '" << (definitions.name ? definitions.name : "(undefined)") << "' namespace '" << (definitions.targetNamespace ? definitions.targetNamespace : "(undefined)") << "'" << std::endl;
    }
  }
  // WSDL 2.0
  for (std::vector<wsdl__ext_fault>::iterator f = fault.begin(); f != fault.end(); ++f)
    (*f).traverse(definitions, portTypeRef);
  for (std::vector<wsdl__ext_operation>::iterator i = operation.begin(); i != operation.end(); ++i)
    (*i).traverse(definitions, portTypeRef);
  for (std::vector<wsp__Policy>::iterator p = wsp__Policy_.begin(); p != wsp__Policy_.end(); ++p)
    (*p).traverse(definitions);
  for (std::vector<wsp__PolicyReference>::iterator r = wsp__PolicyReference_.begin(); r != wsp__PolicyReference_.end(); ++r)
    (*r).traverse(definitions);
  return SOAP_OK;
}

void wsdl__binding::portTypePtr(wsdl__portType *portType)
{
  portTypeRef = portType;
  if (!portTypeRef && vflag)
    std::cerr << "\nWarning: wsdl__binding portType set to NULL" << std::endl;
}

wsdl__portType *wsdl__binding::portTypePtr() const
{
  return portTypeRef;
}

void wsdl__binding::mark()
{
  if (Oflag > 1)
    for (std::vector<wsdl__ext_operation>::iterator i = operation.begin(); i != operation.end(); ++i)
      (*i).mark();
}

wsdl__ext_operation::wsdl__ext_operation()
{
  operationRef = NULL;
}

int wsdl__ext_operation::traverse(wsdl__definitions& definitions, wsdl__portType *portTypeRef)
{
  if (vflag)
    std::cerr << "  Analyzing binding operation '" << (name ? name : ref ? ref : "(null)") << "' in wsdl namespace '" << (definitions.targetNamespace ? definitions.targetNamespace : "(null)") << "'" << std::endl;
  if (input)
    input->traverse(definitions);
  if (output)
    output->traverse(definitions);
  for (std::vector<wsdl__ext_fault>::iterator f = fault.begin(); f != fault.end(); ++f)
    (*f).traverse(definitions, portTypeRef);
  // WSDL 2.0
  for (std::vector<wsdl__ext_fault>::iterator fi = infault.begin(); fi != infault.end(); ++fi)
    (*fi).traverse(definitions, portTypeRef);
  for (std::vector<wsdl__ext_fault>::iterator fo = outfault.begin(); fo != outfault.end(); ++fo)
    (*fo).traverse(definitions, portTypeRef);
  operationRef = NULL;
  if (portTypeRef)
  {
    // WSDL 2.0, assumption: ref refers to an operation in the interface for this binding
    const char *token = NULL;
    if (ref)
      token = qname_token(ref, portTypeRef->definitionsPtr()->targetNamespace);
    for (std::vector<wsdl__operation>::iterator i = portTypeRef->operation.begin(); i != portTypeRef->operation.end(); ++i)
    {
      if (token)
      {
        if ((*i).name && !strcmp((*i).name, token))
        {
          operationRef = &(*i);
          if (vflag)
            std::cerr << "   Found operation '" << token << "' in interface '" << portTypeRef->name << "'" << std::endl;
          break;
        }
      }
      else if (name && (*i).name && !strcmp((*i).name, name))
      {
        if ((!input
            || !input->name
            || ((*i).__union1 == SOAP_UNION_wsdl__union_ioput_input && (*i).__ioput1.input->name && !strcmp((*i).__ioput1.input->name, input->name))
            || ((*i).__union2 == SOAP_UNION_wsdl__union_ioput_input && (*i).__ioput2.input->name && !strcmp((*i).__ioput2.input->name, input->name))
            )
         && (!output
            || !output->name
            || ((*i).__union1 == SOAP_UNION_wsdl__union_ioput_output && (*i).__ioput1.output->name && !strcmp((*i).__ioput1.output->name, output->name))
            || ((*i).__union2 == SOAP_UNION_wsdl__union_ioput_output && (*i).__ioput2.output->name && !strcmp((*i).__ioput2.output->name, output->name))
            ))
        {
          operationRef = &(*i);
          if (vflag)
            std::cerr << "   Found operation '" << name << "' in portType '" << portTypeRef->name << "'" << std::endl;
          break;
        }
      }
    }
    if (name && !operationRef)
    {
      for (std::vector<wsdl__operation>::iterator j = portTypeRef->operation.begin(); j != portTypeRef->operation.end(); ++j)
      {
        if ((*j).name && !strcmp((*j).name, name))
        {
          if (input
           && input->name
           && (((*j).__union1 == SOAP_UNION_wsdl__union_ioput_input && (*j).__ioput1.input->name && !strcmp((*j).__ioput1.input->name, input->name))
            || ((*j).__union2 == SOAP_UNION_wsdl__union_ioput_input && (*j).__ioput2.input->name && !strcmp((*j).__ioput2.input->name, input->name))
              ))
            if (!Wflag)
              std::cerr << "\nWarning: no matching portType operation input name '" << input->name << "' in wsdl definitions '" << (definitions.name ? definitions.name : "(undefined)") << "' namespace '" << (definitions.targetNamespace ? definitions.targetNamespace : "(undefined)") << "'" << std::endl;
          if (output
           && output->name
           && (((*j).__union1 == SOAP_UNION_wsdl__union_ioput_output && (*j).__ioput1.output->name && !strcmp((*j).__ioput1.output->name, output->name))
            || ((*j).__union2 == SOAP_UNION_wsdl__union_ioput_output && (*j).__ioput2.output->name && !strcmp((*j).__ioput2.output->name, output->name))
              ))
            if (!Wflag)
              std::cerr << "\nWarning: no matching portType operation output name '" << output->name << "' in wsdl definitions '" << (definitions.name ? definitions.name : "(undefined)") << "' namespace '" << (definitions.targetNamespace ? definitions.targetNamespace : "(undefined)") << "'" << std::endl;
          operationRef = &(*j);
          if (vflag)
            std::cerr << "   Found operation '" << name << "'" << std::endl;
          break;
        }
      }
    }
  }
  if (!operationRef)
  {
    if (!Wflag)
    {
      if (ref)
        std::cerr << "\nWarning: no matching interface operation '" << (ref ? ref : "(undefined)") << "' in wsdl definitions '" << (definitions.name ? definitions.name : "(undefined)") << "' namespace '" << (definitions.targetNamespace ? definitions.targetNamespace : "(undefined)") << "'" << std::endl;
      else
        std::cerr << "\nWarning: no matching portType operation '" << (name ? name : "(undefined)") << "' in wsdl definitions '" << (definitions.name ? definitions.name : "(undefined)") << "' namespace '" << (definitions.targetNamespace ? definitions.targetNamespace : "(undefined)") << "'" << std::endl;
    }
  }
  else
  {
    for (std::vector<wsdl__ext_fault>::iterator i = fault.begin(); i != fault.end(); ++i)
    {
      if ((*i).name)
      {
        for (std::vector<wsdl__fault>::iterator j = operationRef->fault.begin(); j != operationRef->fault.end(); ++j)
        {
          if ((*j).name && !strcmp((*j).name, (*i).name))
          {
            (*i).faultPtr(&(*j));
            if (vflag)
              std::cerr << "   Found fault '" << (*j).name << "' message" << std::endl;
            break;
          }
        }
      }
      else if ((*i).soap__fault_ && (*i).soap__fault_->name) // try the soap:fault name, this is not elegant, but neither is WSDL 1.1 
      {
        for (std::vector<wsdl__fault>::iterator j = operationRef->fault.begin(); j != operationRef->fault.end(); ++j)
        {
          if ((*j).name && !strcmp((*j).name, (*i).soap__fault_->name))
          {
            (*i).faultPtr(&(*j));
            if (vflag)
              std::cerr << "   Found fault '" << ((*j).name ? (*j).name : "(null)") << "' message" << std::endl;
            break;
          }
        }
      }
      if (!(*i).faultPtr())
        if (!Wflag)
          std::cerr << "\nWarning: no soap:fault '" << ((*i).name ? (*i).name : "(undefined)") << "' message in wsdl definitions '" << (definitions.name ? definitions.name : "(undefined)") << "' operation '" << (name ? name : "(undefined)") << "' namespace '" << (definitions.targetNamespace ? definitions.targetNamespace : "(undefined)") << "'" << std::endl;
    }
  }
  if (wsp__Policy_)
    wsp__Policy_->traverse(definitions);
  if (wsp__PolicyReference_)
    wsp__PolicyReference_->traverse(definitions);
  return SOAP_OK;
}

void wsdl__ext_operation::operationPtr(wsdl__operation *operation)
{
  operationRef = operation;
  if (!operationRef && vflag)
    std::cerr << "\nWarning: wsdl__ext_operation operation set to NULL" << std::endl;
}

wsdl__operation *wsdl__ext_operation::operationPtr() const
{
  return operationRef;
}

void wsdl__ext_operation::mark()
{
  if (Oflag > 1)
  {
    if (input)
      input->mark();
    if (output)
      output->mark();
  }
}

int wsdl__ext_ioput::traverse(wsdl__definitions& definitions)
{
  if (vflag)
    std::cerr << "   Analyzing binding operation input/output in wsdl namespace '" << (definitions.targetNamespace ? definitions.targetNamespace : "(null)") << "'" << std::endl;
  for (std::vector<soap__header>::iterator hd = soap__header_.begin(); hd != soap__header_.end(); ++hd)
    (*hd).traverse(definitions);
  for (std::vector<wsoap__header>::iterator whd = wsoap__header_.begin(); whd != wsoap__header_.end(); ++whd)
    (*whd).traverse(definitions);
  if (mime__multipartRelated_)
    mime__multipartRelated_->traverse(definitions);
  if (wsp__Policy_)
    wsp__Policy_->traverse(definitions);
  if (wsp__PolicyReference_)
    wsp__PolicyReference_->traverse(definitions);
  return SOAP_OK;
}

void wsdl__ext_ioput::mark()
{
  if (Oflag > 1)
    for (std::vector<wsoap__header>::iterator sh = wsoap__header_.begin(); sh != wsoap__header_.end(); ++sh)
      (*sh).mark();
}

wsdl__ext_fault::wsdl__ext_fault()
{
  faultRef = NULL;
}

int wsdl__ext_fault::traverse(wsdl__definitions& definitions, wsdl__portType *portTypeRef)
{
  if (vflag)
    std::cerr << "   Analyzing binding operation fault in wsdl namespace '" << (definitions.targetNamespace ? definitions.targetNamespace : "(null)") << "'" << std::endl;
  if (ref && portTypeRef)
  {
    // WSDL 2.0, assumption: ref refers to a fault in the interface for this binding
    const char *token = qname_token(ref, portTypeRef->definitionsPtr()->targetNamespace);
    if (token)
    {
      for (std::vector<wsdl__fault>::iterator fault = portTypeRef->fault.begin(); fault != portTypeRef->fault.end(); ++fault)
      {
        if ((*fault).name && !strcmp((*fault).name, token))
        {
          faultRef = &(*fault);
          if (vflag)
            std::cerr << "   Found fault '" << (*fault).name << std::endl;
          break;
        }
      }
    }
    if (!faultRef)
      if (!Wflag)
        std::cerr << "\nWarning: no fault '" << (ref ? ref : "(undefined)") << "' in wsdl definitions '" << (definitions.name ? definitions.name : "(undefined)") << "' interface '" << (portTypeRef->name ? portTypeRef->name : "(undefined)") << "' namespace '" << (definitions.targetNamespace ? definitions.targetNamespace : "(undefined)") << "'" << std::endl;
  }
  if (wsp__Policy_)
    wsp__Policy_->traverse(definitions);
  if (wsp__PolicyReference_)
    wsp__PolicyReference_->traverse(definitions);
  return SOAP_OK;
}

void wsdl__ext_fault::faultPtr(wsdl__fault *fault)
{
  faultRef = fault;
  if (!faultRef && vflag)
    std::cerr << "\nWarning: wsdl__ext_fault fault ref set to NULL" << std::endl;
}

wsdl__fault *wsdl__ext_fault::faultPtr() const
{
  return faultRef;
}

wsdl__portType::wsdl__portType()
{
  definitionsRef = NULL;
}

int wsdl__portType::traverse(wsdl__definitions& definitions)
{
  if (vflag)
    std::cerr << " Analyzing portType/interface '" << (name ? name : "(null)") << "' in wsdl namespace '" << (definitions.targetNamespace ? definitions.targetNamespace : "(null)") << "'" << std::endl;
  definitionsRef = &definitions;
  // traverse faults before operations, WSDL 2.0
  for (std::vector<wsdl__fault>::iterator f = fault.begin(); f != fault.end(); ++f)
    (*f).traverse(definitions);
  for (std::vector<wsdl__operation>::iterator i = operation.begin(); i != operation.end(); ++i)
    (*i).traverse(definitions);
  if (wsp__Policy_)
    wsp__Policy_->traverse(definitions);
  if (wsp__PolicyReference_)
    wsp__PolicyReference_->traverse(definitions);
  return SOAP_OK;
}

void wsdl__portType::definitionsPtr(wsdl__definitions *definitions)
{
  definitionsRef = definitions;
}

wsdl__definitions *wsdl__portType::definitionsPtr() const
{
  return definitionsRef;
}

void wsdl__portType::mark()
{
  if (Oflag > 1)
  {
    for (std::vector<wsdl__fault>::iterator ft = fault.begin(); ft != fault.end(); ++ft)
      (*ft).mark();
    for (std::vector<wsdl__operation>::iterator op = operation.begin(); op != operation.end(); ++op)
      (*op).mark();
  }
}

int wsdl__operation::traverse(wsdl__definitions& definitions)
{
  if (vflag)
    std::cerr << "  Analyzing portType/interface operation '" << (name ? name : "(null)") << "' in wsdl namespace '" << (definitions.targetNamespace ? definitions.targetNamespace : "(null)") << "'" << std::endl;
  if (__union1 == SOAP_UNION_wsdl__union_ioput_input)
    if (__ioput1.input)
       __ioput1.input->traverse(definitions);
  if (__union1 == SOAP_UNION_wsdl__union_ioput_output)
    if (__ioput1.output)
       __ioput1.output->traverse(definitions);
  if (__union2 == SOAP_UNION_wsdl__union_ioput_input)
    if (__ioput2.input)
       __ioput2.input->traverse(definitions);
  if (__union2 == SOAP_UNION_wsdl__union_ioput_output)
    if (__ioput2.output)
       __ioput2.output->traverse(definitions);
  for (std::vector<wsdl__fault>::iterator i = fault.begin(); i != fault.end(); ++i)
    (*i).traverse(definitions);
  // WSDL 2.0
  for (std::vector<wsdl__fault>::iterator fi = infault.begin(); fi != infault.end(); ++fi)
    (*fi).traverse(definitions);
  for (std::vector<wsdl__fault>::iterator fo = outfault.begin(); fo != outfault.end(); ++fo)
    (*fo).traverse(definitions);
  if (wsp__Policy_)
    wsp__Policy_->traverse(definitions);
  if (wsp__PolicyReference_)
    wsp__PolicyReference_->traverse(definitions);
  return SOAP_OK;
}

void wsdl__operation::mark()
{
  if (Oflag > 1)
  {
    if (__union1 == SOAP_UNION_wsdl__union_ioput_input)
      if (__ioput1.input)
        __ioput1.input->mark();
    if (__union1 == SOAP_UNION_wsdl__union_ioput_output)
      if (__ioput1.output)
        __ioput1.output->mark();
    if (__union2 == SOAP_UNION_wsdl__union_ioput_input)
      if (__ioput2.input)
        __ioput2.input->mark();
    if (__union2 == SOAP_UNION_wsdl__union_ioput_output)
      if (__ioput2.output)
        __ioput2.output->mark();
    for (std::vector<wsdl__fault>::iterator i = fault.begin(); i != fault.end(); ++i)
      (*i).mark();
    for (std::vector<wsdl__fault>::iterator fi = infault.begin(); fi != infault.end(); ++fi)
      (*fi).mark();
    for (std::vector<wsdl__fault>::iterator fo = outfault.begin(); fo != outfault.end(); ++fo)
      (*fo).mark();
  }
}

wsdl__ioput::wsdl__ioput()
{
  messageRef = NULL;
  elementRef = NULL;
}

int wsdl__ioput::traverse(wsdl__definitions& definitions)
{
  if (vflag)
    std::cerr << "   Analyzing portType/interface operation input/output in wsdl namespace '" << (definitions.targetNamespace ? definitions.targetNamespace : "(null)") << "'" << std::endl;
  messageRef = NULL;
  elementRef = NULL;
  // WSDL 2.0
  if (element)
  {
    if (definitions.types)
    {
      for (std::vector<xs__schema*>::iterator schema = definitions.types->xs__schema_.begin(); schema != definitions.types->xs__schema_.end(); ++schema)
      {
        const char *token = qname_token(element, (*schema)->targetNamespace);
        if (token)
        {
          for (std::vector<xs__element>::iterator element = (*schema)->element.begin(); element != (*schema)->element.end(); ++element)
          {
            if ((*element).name && !strcmp((*element).name, token))
            {
              elementRef = &(*element);
              if (vflag)
                std::cerr << "   Found input/output '" << (messageLabel ? messageLabel : "(null)") << "' element '" << (token ? token : "(null)") << "'" << std::endl;
              break;
            }
          }
        }
      }
    }
    if (*element != '#' && !elementRef)
    {
      if (is_builtin_qname(element))
        definitions.builtinElement(element);
      else
        if (!Wflag)
          std::cerr << "\nWarning: no input/output '" << (messageLabel ? messageLabel : "(undefined)") << "' element '" << element << "' in wsdl definitions '" << (definitions.name ? definitions.name : "(undefined)") << "' namespace '" << (definitions.targetNamespace ? definitions.targetNamespace : "(undefined)") << "'" << std::endl;
    }
  }
  else
  {
    const char *token = qname_token(message, definitions.targetNamespace);
    if (token)
    {
      for (std::vector<wsdl__message>::iterator message = definitions.message.begin(); message != definitions.message.end(); ++message)
      {
        if ((*message).name && !strcmp((*message).name, token))
        {
          messageRef = &(*message);
          if (vflag)
            std::cerr << "    Found input/output '" << (name ? name : "(null)") << "' message '" << (token ? token : "(null)") << "'" << std::endl;
          break;
        }
      }
    }
    if (!messageRef)
    {
      for (std::vector<wsdl__import>::iterator import = definitions.import.begin(); import != definitions.import.end(); ++import)
      {
        wsdl__definitions *importdefinitions = (*import).definitionsPtr();
        if (importdefinitions)
        {
          token = qname_token(message, importdefinitions->targetNamespace);
          if (token)
          {
            for (std::vector<wsdl__message>::iterator message = importdefinitions->message.begin(); message != importdefinitions->message.end(); ++message)
            {
              if ((*message).name && !strcmp((*message).name, token))
              {
                messageRef = &(*message);
                if (vflag)
                  std::cerr << "    Found input/output '" << (name ? name : "(null)") << "' message '" << (token ? token : "(null)") << "'" << std::endl;
                break;
              }
            }
          }
        }
      }
    }
    if (!messageRef)
      if (!Wflag)
        std::cerr << "\nWarning: no input/output '" << (name ? name : "(undefined)") << "' message '" << (message ? message : "(undefined)") << "' in wsdl definitions '" << (definitions.name ? definitions.name : "(undefined)") << "' namespace '" << (definitions.targetNamespace ? definitions.targetNamespace : "(undefined)") << "'" << std::endl;
  }
  if (wsp__Policy_)
    wsp__Policy_->traverse(definitions);
  if (wsp__PolicyReference_)
    wsp__PolicyReference_->traverse(definitions);
  return SOAP_OK;
}

void wsdl__ioput::messagePtr(wsdl__message *message)
{
  messageRef = message;
}

wsdl__message *wsdl__ioput::messagePtr() const
{
  return messageRef;
}

void wsdl__ioput::elementPtr(xs__element *element)
{
  elementRef = element;
}

xs__element *wsdl__ioput::elementPtr() const
{
  return elementRef;
}

void wsdl__ioput::mark()
{
  if (Oflag > 1)
    if (elementPtr())
      elementPtr()->mark();
}

wsdl__fault::wsdl__fault()
{
  messageRef = NULL;
  elementRef = NULL;
}

int wsdl__fault::traverse(wsdl__definitions& definitions)
{
  if (vflag)
    std::cerr << "   Analyzing portType/interface operation faults in wsdl namespace '" << (definitions.targetNamespace ? definitions.targetNamespace : "(null)") << "'" << std::endl;
  messageRef = NULL;
  elementRef = NULL;
  // WSDL 2.0
  if (ref)
  {
    const char *token = qname_token(ref, definitions.targetNamespace);
    if (token)
    {
      for (std::vector<wsdl__portType>::iterator i = definitions.interface_.begin(); i != definitions.interface_.end(); ++i)
      {
        for (std::vector<wsdl__fault>::iterator fault = (*i).fault.begin(); fault != (*i).fault.end(); ++fault)
        {
          if ((*fault).name && !strcmp((*fault).name, token))
          {
            elementRef = (*fault).elementPtr();
            if (vflag)
              std::cerr << "   Found fault '" << (ref ? ref : "(null)") << "' element '" << (token ? token : "(null)") << "'" << std::endl;
            break;
          }
        }
      }
    }
    if (!elementRef)
    {
      for (std::vector<wsdl__import>::iterator import = definitions.import.begin(); import != definitions.import.end(); ++import)
      {
        wsdl__definitions *importdefinitions = (*import).definitionsPtr();
        if (importdefinitions)
        {
          token = qname_token(message, importdefinitions->targetNamespace);
          if (token)
          {
            for (std::vector<wsdl__portType>::iterator i = importdefinitions->interface_.begin(); i != importdefinitions->interface_.end(); ++i)
            {
              for (std::vector<wsdl__fault>::iterator fault = (*i).fault.begin(); fault != (*i).fault.end(); ++fault)
              {
                if ((*fault).name && !strcmp((*fault).name, token))
                {
                  elementRef = (*fault).elementPtr();
                  if (vflag)
                    std::cerr << "   Found fault '" << (ref ? ref : "(null)") << "' element '" << (token ? token : "(null)") << "'" << std::endl;
                  break;
                }
              }
            }
          }
        }
      }
    }
    if (!elementRef)
    {
      if (is_builtin_qname(element))
        definitions.builtinElement(element);
      else
        if (!Wflag)
          std::cerr << "\nWarning: no fault '" << (messageLabel ? messageLabel : "(undefined)") << "' ref '" << ref << "' in wsdl definitions '" << (definitions.name ? definitions.name : "(undefined)") << "' namespace '" << (definitions.targetNamespace ? definitions.targetNamespace : "(undefined)") << "'" << std::endl;
    }
  }
  else if (element)
  {
    if (definitions.types)
    {
      for (std::vector<xs__schema*>::iterator schema = definitions.types->xs__schema_.begin(); schema != definitions.types->xs__schema_.end(); ++schema)
      {
        const char *token = qname_token(element, (*schema)->targetNamespace);
        if (token)
        {
          for (std::vector<xs__element>::iterator element = (*schema)->element.begin(); element != (*schema)->element.end(); ++element)
          {
            if ((*element).name && !strcmp((*element).name, token))
            {
              elementRef = &(*element);
              if (vflag)
                std::cerr << "   Found fault '" << (messageLabel ? messageLabel : "(null)") << "' element '" << (token ? token : "(null)") << "'" << std::endl;
              break;
            }
          }
        }
      }
    }
    if (!elementRef)
    {
      if (is_builtin_qname(element))
        definitions.builtinElement(element);
      else
        if (!Wflag)
          std::cerr << "\nWarning: no fault '" << (messageLabel ? messageLabel : "(undefined)") << "' element '" << element << "' in wsdl definitions '" << (definitions.name ? definitions.name : "(undefined)") << "' namespace '" << (definitions.targetNamespace ? definitions.targetNamespace : "(undefined)") << "'" << std::endl;
    }
  }
  else
  {
    const char *token = qname_token(message, definitions.targetNamespace);
    if (token)
    {
      for (std::vector<wsdl__message>::iterator message = definitions.message.begin(); message != definitions.message.end(); ++message)
      {
        if ((*message).name && !strcmp((*message).name, token))
        {
          messageRef = &(*message);
          if (vflag)
            std::cerr << "    Found operation fault '" << (name ? name : "(null)") << "' message '" << (token ? token : "(null)") << "'" << std::endl;
          break;
        }
      }
    }
    if (!messageRef)
    {
      for (std::vector<wsdl__import>::iterator import = definitions.import.begin(); import != definitions.import.end(); ++import)
      {
        wsdl__definitions *importdefinitions = (*import).definitionsPtr();
        if (importdefinitions)
        {
          token = qname_token(message, importdefinitions->targetNamespace);
          if (token)
          {
            for (std::vector<wsdl__message>::iterator message = importdefinitions->message.begin(); message != importdefinitions->message.end(); ++message)
            {
              if ((*message).name && !strcmp((*message).name, token))
              {
                messageRef = &(*message);
                if (vflag)
                  std::cerr << "    Found operation fault '" << (name ? name : "(null)") << "' message '" << (token ? token : "(null)") << "'" << std::endl;
                break;
              }
            }
          }
        }
      }
    }
    if (!messageRef)
      if (!Wflag)
        std::cerr << "\nWarning: no operation fault '" << (name ? name : "(undefined)") << "' message '" << (message ? message : "(undefined)") << "' in wsdl definitions '" << (definitions.name ? definitions.name : "(undefined)") << "' namespace '" << (definitions.targetNamespace ? definitions.targetNamespace : "(undefined)") << "'" << std::endl;
  }
  if (wsp__Policy_)
    wsp__Policy_->traverse(definitions);
  if (wsp__PolicyReference_)
    wsp__PolicyReference_->traverse(definitions);
  return SOAP_OK;
}

void wsdl__fault::messagePtr(wsdl__message *message)
{
  messageRef = message;
}

wsdl__message *wsdl__fault::messagePtr() const
{
  return messageRef;
}

void wsdl__fault::elementPtr(xs__element *element)
{
  elementRef = element;
}

xs__element *wsdl__fault::elementPtr() const
{
  return elementRef;
}

void wsdl__fault::mark()
{
  if (Oflag > 1)
    if (elementPtr())
      elementPtr()->mark();
}

int wsdl__message::traverse(wsdl__definitions& definitions)
{
  if (vflag)
    std::cerr << " Analyzing message '" << (name ? name : "(null)") << "' in wsdl namespace '" << (definitions.targetNamespace ? definitions.targetNamespace : "(null)") << "'" << std::endl;
  for (std::vector<wsdl__part>::iterator i = part.begin(); i != part.end(); ++i)
    (*i).traverse(definitions);
  for (std::vector<wsp__Policy>::iterator p = wsp__Policy_.begin(); p != wsp__Policy_.end(); ++p)
    (*p).traverse(definitions);
  for (std::vector<wsp__PolicyReference>::iterator r = wsp__PolicyReference_.begin(); r != wsp__PolicyReference_.end(); ++r)
    (*r).traverse(definitions);
  return SOAP_OK;
}

void wsdl__message::mark()
{
  if (Oflag > 1)
    for (std::vector<wsdl__part>::iterator i = part.begin(); i != part.end(); ++i)
      (*i).mark();
}

wsdl__part::wsdl__part()
{
  elementRef = NULL;
  simpleTypeRef = NULL;
  complexTypeRef = NULL;
  optional = false;
  repeating = false;
  fixed = false;
  default_ = NULL;
}

int wsdl__part::traverse(wsdl__definitions& definitions)
{
  if (vflag)
    std::cerr << "  Analyzing message parts in wsdl namespace '" << (definitions.targetNamespace ? definitions.targetNamespace : "(null)") << "'" << std::endl;
  elementRef = NULL;
  simpleTypeRef = NULL;
  complexTypeRef = NULL;
  if (definitions.types)
  {
    for (std::vector<xs__schema*>::iterator schema = definitions.types->xs__schema_.begin(); schema != definitions.types->xs__schema_.end(); ++schema)
    {
      const char *token = qname_token(element, (*schema)->targetNamespace);
      if (token)
      {
        for (std::vector<xs__element>::iterator el = (*schema)->element.begin(); el != (*schema)->element.end(); ++el)
        {
          if ((*el).name && !strcmp((*el).name, token))
          {
            elementRef = &(*el);
            if (vflag)
              std::cerr << "   Found message part '" << (name ? name : "(null)") << "' element '" << (token ? token : "(null)") << "'" << std::endl;
            break;
          }
        }
      }
      token = qname_token(type, (*schema)->targetNamespace);
      if (token)
      {
        for (std::vector<xs__simpleType>::iterator st = (*schema)->simpleType.begin(); st != (*schema)->simpleType.end(); ++st)
        {
          if ((*st).name && !strcmp((*st).name, token))
          {
            simpleTypeRef = &(*st);
            if (vflag)
              std::cerr << "   Found message part '" << (name ? name : "(null)") << "' simpleType '" << (token ? token : "(null)") << "'" << std::endl;
            break;
          }
        }
      }
      token = qname_token(type, (*schema)->targetNamespace);
      if (token)
      {
        for (std::vector<xs__complexType>::iterator ct = (*schema)->complexType.begin(); ct != (*schema)->complexType.end(); ++ct)
        {
          if ((*ct).name && !strcmp((*ct).name, token))
          {
            complexTypeRef = &(*ct);
            if (vflag)
              std::cerr << "   Found message part '" << (name ? name : "(null)") << "' complexType '" << (token ? token : "(null)") << "'" << std::endl;
            break;
          }
        }
      }
    }
  }
  if (!elementRef && !simpleTypeRef && !complexTypeRef)
  {
    if (element)
    {
      if (is_builtin_qname(element))
        definitions.builtinElement(element);
      else
        if (!Wflag)
          std::cerr << "\nWarning: no message part '" << (name ? name : "(undefined)") << "' element '" << element << "' in wsdl definitions '" << (definitions.name ? definitions.name : "(undefined)") << "' namespace '" << (definitions.targetNamespace ? definitions.targetNamespace : "(undefined)") << "'" << std::endl;
    }
    else if (type)
    {
      if (is_builtin_qname(type))
        definitions.builtinType(type);
      else
        if (!Wflag)
          std::cerr << "\nWarning: no message part '" << (name ? name : "(undefined)") << "' type '" << type << "' in wsdl definitions '" << (definitions.name ? definitions.name : "(undefined)") << "' namespace '" << (definitions.targetNamespace ? definitions.targetNamespace : "(undefined)") << "'" << std::endl;
    }
    else
      if (!Wflag)
        std::cerr << "\nWarning: no message part '" << (name ? name : "(undefined)") << "' element or type in wsdl definitions '" << (definitions.name ? definitions.name : "(undefined)") << "' namespace '" << (definitions.targetNamespace ? definitions.targetNamespace : "(undefined)") << "'" << std::endl;
  }
  return SOAP_OK;
}

void wsdl__part::elementPtr(xs__element *element)
{
  elementRef = element;
  if (!elementRef && vflag)
    std::cerr << "\nWarning: wsdl__part element set to NULL" << std::endl;
}

void wsdl__part::simpleTypePtr(xs__simpleType *simpleType)
{
  simpleTypeRef = simpleType;
  if (!simpleTypeRef && vflag)
    std::cerr << "\nWarning: wsdl__part simpleType set to NULL" << std::endl;
}

void wsdl__part::complexTypePtr(xs__complexType *complexType)
{
  complexTypeRef = complexType;
  if (!complexTypeRef && vflag)
    std::cerr << "\nWarning: wsdl__part complexType set to NULL" << std::endl;
}

xs__element *wsdl__part::elementPtr() const
{
  return elementRef;
}

xs__simpleType *wsdl__part::simpleTypePtr() const
{
  return simpleTypeRef;
}

xs__complexType *wsdl__part::complexTypePtr() const
{
  return complexTypeRef;
}

void wsdl__part::set_optional(bool b)
{
  optional = b;
}

void wsdl__part::set_fixed(bool b)
{
  fixed = b;
}

void wsdl__part::set_repeating(bool b)
{
  repeating = b;
}

void wsdl__part::set_default(char* s)
{
  default_ = s;
}

void wsdl__part::set_option(char* s)
{
  if (s)
    option.push_back(s);
}

bool wsdl__part::is_optional() const
{
  return optional;
}

bool wsdl__part::is_fixed() const
{
  return fixed;
}

bool wsdl__part::is_repeating() const
{
  return repeating;
}

const char *wsdl__part::get_default() const
{
  return default_;
}

const std::vector<char*>& wsdl__part::options() const
{
  return option;
}

void wsdl__part::mark()
{
  if (Oflag > 1)
  {
    if (elementPtr())
      elementPtr()->mark();
    if (simpleTypePtr())
      simpleTypePtr()->mark();
    if (complexTypePtr())
      complexTypePtr()->mark();
  }
}

int wsdl__types::preprocess(wsdl__definitions& definitions)
{
  if (vflag)
    std::cerr << "Preprocessing wsdl types" << std::endl;
  if (xs__schema_.empty()) // WSDL 2.0 <types> w/o <schema>
  {
    targetNamespace = definitions.targetNamespace;
    sourceLocation(definitions.sourceLocation());
    xs__schema_.push_back(this);
  }
  for (std::vector<xs__schema*>::iterator schema = xs__schema_.begin(); schema != xs__schema_.end(); ++schema)
    (*schema)->sourceLocation(definitions.sourceLocation());
again:
  // link imported schemas, need to repeat when <types> is extended with new imported schema (from inside another schema, etc.)
  for (std::vector<xs__schema*>::iterator schema1 = xs__schema_.begin(); schema1 != xs__schema_.end(); ++schema1)
  {
    for (std::vector<xs__import>::iterator import = (*schema1)->import.begin(); import != (*schema1)->import.end(); ++import)
    {
      if ((*import).namespace_ && !(*import).schemaPtr())
      {
        for (std::vector<xs__schema*>::const_iterator schema2 = xs__schema_.begin(); schema2 != xs__schema_.end(); ++schema2)
        {
          if (schema2 != schema1 && (*schema2)->targetNamespace && !strcmp((*import).namespace_, (*schema2)->targetNamespace))
          {
            (*import).schemaPtr(*schema2);
            break;
          }
        }
      }
    }
  }
  // if a schema is imported but not in <types> then get it
  for (std::vector<xs__schema*>::iterator schema2 = xs__schema_.begin(); schema2 != xs__schema_.end(); ++schema2)
  {
    for (std::vector<xs__import>::iterator import = (*schema2)->import.begin(); import != (*schema2)->import.end(); ++import)
    {
      bool found = false;
      if (vflag)
        std::cerr << "Preprocessing schema '" << (*schema2)->targetNamespace << "' import '" << ((*import).namespace_ ? (*import).namespace_ : "(null)") << "'" << std::endl; 
      if (!found && (*import).namespace_)
      {
        for (SetOfString::const_iterator i = exturis.begin(); i != exturis.end(); ++i)
        {
          if (!soap_tag_cmp((*import).namespace_, *i))
          {
            found = true;
            break;
          }
        }
      }
      if (!found && !iflag) // don't import any of the schemas in the .nsmap table (or when -i option is used)
      {
        xs__schema *importschema = (*import).schemaPtr();
        if (importschema)
        {
#if 0 // no longer applicable as imports are preprocessed
        if (!importschema)
        {
          const char *s = (*import).schemaLocation;
          if (!s && (*import).location)
          {
            s = (*import).schemaLocation = (*import).location; // work around for Microsoft bugs
            std::cerr << "Schema import with namespace '" << ((*import).namespace_ ? (*import).namespace_ : "(null)") << "' has a 'location' attribute specified but a 'schemaLocation' attribute must be used, please inform the author of this WSDL to correct this problem" << std::endl;
          }
          else if (!s)
          {
            s = (*import).namespace_;
          }
          if (!s)
            continue;
          importschema = new xs__schema(definitions.soap, (*schema2)->sourceLocation(), (*schema2)->absoluteLocation(s));
          if (!(*import).namespace_)
          {
            if (importschema->targetNamespace)
              (*import).namespace_ = importschema->targetNamespace;
            else
              (*import).namespace_ = soap_strdup(definitions.soap, "");
          }
          if (!importschema->targetNamespace || !*importschema->targetNamespace)
            importschema->targetNamespace = (*import).namespace_;
          else if ((*import).namespace_ && strcmp(importschema->targetNamespace, (*import).namespace_))
            std::cerr << "Schema import namespace '" << ((*import).namespace_ ? (*import).namespace_ : "(null)") << "' does not correspond to imported namespace '" << importschema->targetNamespace << "'" << std::endl;
          for (std::vector<xs__schema*>::const_iterator schema3 = xs__schema_.begin(); schema3 != xs__schema_.end(); ++schema3)
          {
            if ((*schema3)->targetNamespace && !strcmp((*import).namespace_, (*schema3)->targetNamespace))
            {
              (*import).schemaPtr(*schema3);
              if ((*schema3) == this || // WSDL 2.0 <types> has no FormDefaults
                  (*schema3)->empty())  // schema w/o components, only imports
              {
                (*schema3)->elementFormDefault = importschema->elementFormDefault;
                (*schema3)->attributeFormDefault = importschema->attributeFormDefault;
              }
              (*schema3)->insert(*importschema); // merge content
              goto again;
            }
          }
        }
#endif
          if (!found)
          {
            for (std::vector<xs__schema*>::const_iterator schema3 = xs__schema_.begin(); schema3 != xs__schema_.end(); ++schema3)
            {
              if (*schema3 == importschema)
              {
                found = true;
                break;
              }
            }
          }
          if (!found)
          {
            for (std::vector<xs__schema*>::const_iterator schema4 = xs__schema_.begin(); schema4 != xs__schema_.end(); ++schema4)
            {
              if ((!(*schema4)->targetNamespace && !(*import).namespace_) || ((*schema4)->targetNamespace && (*import).namespace_ && !strcmp((*import).namespace_, (*schema4)->targetNamespace)))
              {
                (*import).schemaPtr(*schema4);
                if ((*schema4) == this || // WSDL 2.0 <types> has no FormDefaults
                    (*schema4)->empty())  // schema w/o components, only imports
                {
                  (*schema4)->elementFormDefault = importschema->elementFormDefault;
                  (*schema4)->attributeFormDefault = importschema->attributeFormDefault;
                }
                (*schema4)->insert(*importschema); // merge content
                goto again;
              }
            }
          }
          if (!found)
          {
            xs__schema_.push_back(importschema);
            if (vflag)
              std::cerr << "Adding schema '" << (importschema->targetNamespace ? importschema->targetNamespace : "(null)") << "'" << std::endl;
            goto again;
          }
        }
        else if (!Wflag)
        {
          fprintf(stderr, "\nWarning: no schemaLocation in <import namespace=\"%s\"> to load schema: unknown schema\n", (*import).namespace_ ? (*import).namespace_ : "(undefined)");
        }
      }
    }
  }
  if (vflag)
  {
    for (std::vector<xs__schema*>::iterator schema4 = xs__schema_.begin(); schema4 != xs__schema_.end(); ++schema4)
    {
      std::cerr << std::endl << "Schema " << ((*schema4)->targetNamespace ? (*schema4)->targetNamespace : "") << " " << ((*schema4)->sourceLocation() ? (*schema4)->sourceLocation() : "") << std::endl;
      for (std::vector<xs__import>::iterator im = (*schema4)->import.begin(); im != (*schema4)->import.end(); ++im)
        std::cerr << "  import " << ((*im).namespace_ ? (*im).namespace_ : "") << " " << ((*im).schemaLocation ? (*im).schemaLocation : "") << std::endl;
      for (std::vector<xs__simpleType>::iterator st = (*schema4)->simpleType.begin(); st != (*schema4)->simpleType.end(); ++st)
        std::cerr << "  simpleType " << ((*st).name ? (*st).name : "") << std::endl;
      for (std::vector<xs__complexType>::iterator ct = (*schema4)->complexType.begin(); ct != (*schema4)->complexType.end(); ++ct)
        std::cerr << "  complexType " << ((*ct).name ? (*ct).name : "") << std::endl;
      for (std::vector<xs__element>::iterator el = (*schema4)->element.begin(); el != (*schema4)->element.end(); ++el)
        std::cerr << "  element " << ((*el).name ? (*el).name : "") << std::endl;
      for (std::vector<xs__attribute>::iterator at = (*schema4)->attribute.begin(); at != (*schema4)->attribute.end(); ++at)
        std::cerr << "  attribute " << ((*at).name ? (*at).name : "") << std::endl;
    }
  }
  return SOAP_OK;
}

int wsdl__types::traverse(wsdl__definitions& definitions)
{
  if (vflag)
    std::cerr << " Analyzing types in wsdl namespace '" << (definitions.targetNamespace ? definitions.targetNamespace : "(null)") << "'" << std::endl;
  for (std::vector<xs__schema*>::iterator schema3 = xs__schema_.begin(); schema3 != xs__schema_.end(); ++schema3)
  {
    // artificially extend the <import> of each schema to include others so when we traverse schemas we can resolve references
    for (std::vector<xs__schema*>::iterator importschema = xs__schema_.begin(); importschema != xs__schema_.end(); ++importschema)
    {
      if (schema3 != importschema && (*importschema)->targetNamespace)
      {
        xs__import *import = soap_new_xs__import(definitions.soap);
        import->namespace_ = (*importschema)->targetNamespace;
        import->schemaPtr(*importschema);
        (*schema3)->import.push_back(*import);
      }
    }
    // check and report
    for (std::vector<xs__import>::iterator import = (*schema3)->import.begin(); import != (*schema3)->import.end(); ++import)
    {
      if ((*import).namespace_)
      {
        bool found = false;
        for (std::vector<xs__schema*>::const_iterator importschema = xs__schema_.begin(); importschema != xs__schema_.end(); ++importschema)
        {
          if ((*importschema)->targetNamespace && !strcmp((*import).namespace_, (*importschema)->targetNamespace))
          {
            found = true;
            break;
          }
        }
        if (!found && vflag)
          std::cerr << "Schema import namespace '" << (*import).namespace_ << "' refers to an unknown Schema" << std::endl;
      }
      else if (!Wflag)
        std::cerr << "\nWarning: schema import '" << ((*import).schemaLocation ? (*import).schemaLocation : "") << "' has no namespace" << std::endl;
    }
  }
  // traverse the schemas
  for (std::vector<xs__schema*>::iterator schema4 = xs__schema_.begin(); schema4 != xs__schema_.end(); ++schema4)
    (*schema4)->traverse();
  // find all built-in types, elements, and attributes
  for (std::vector<xs__schema*>::iterator schema5 = xs__schema_.begin(); schema5 != xs__schema_.end(); ++schema5)
  {
    if (vflag)
      for (SetOfString::const_iterator i = (*schema5)->builtinTypes().begin(); i != (*schema5)->builtinTypes().end(); ++i)
        std::cerr << " Built-in schema type '" << (*i) << "'" << std::endl;
    definitions.builtinTypes((*schema5)->builtinTypes());
    definitions.builtinTypeDerivations((*schema5)->builtinTypeDerivations());
    definitions.builtinElements((*schema5)->builtinElements());
    definitions.builtinAttributes((*schema5)->builtinAttributes());
  }
  return SOAP_OK;
}

void wsdl__types::mark()
{
  if (Oflag > 1)
  {
    xs__schema::mark();
    for (std::vector<xs__schema*>::iterator schema = xs__schema_.begin(); schema != xs__schema_.end(); ++schema)
      (*schema)->mark();
  }
}

int wsdl__import::preprocess(wsdl__definitions& definitions)
{
  static std::map<const char*, wsdl__definitions*, ltstr> included;
  bool found = false;
  const char *relative_location = soap_strdup(definitions.soap, location);
  location = definitions.absoluteLocation(location);
  if (vflag)
    std::cerr << "Preprocess wsdl import '" << (location ? location : "(null)") << "'" << std::endl;
  definitionsRef = NULL;
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
  if (!found && location)
  {
    std::map<const char*, wsdl__definitions*, ltstr>::iterator i = included.find(location);
    if (i != included.end())
    {
      if (vflag)
        fprintf(stderr, "\nWSDL/XSD '%s' already imported\n", location);
      found = true;
      definitionsRef = (*i).second;
    }
  }
  if (!found && location)
  {
    // parse imported definitions
    const char *source = definitions.sourceLocation();
    if (vflag)
      std::cerr << "Importing '" << location << "' into '" << (source ? source : "(source location not set)") << "'" << std::endl;
    definitionsRef = new wsdl__definitions(definitions.soap);
    if (!definitionsRef)
      return SOAP_EOM;
    included[location] = definitionsRef;
    definitionsRef->read(source, location, relative_location);
    if (!namespace_)
      namespace_ = definitionsRef->targetNamespace;
    else if (!definitionsRef->targetNamespace || !*definitionsRef->targetNamespace)
      definitionsRef->targetNamespace = namespace_;
    else if (strcmp(namespace_, definitionsRef->targetNamespace))
      std::cerr << "Error: wsdl definitions/import '" << location << "' namespace '" << namespace_ << "' does not match imported targetNamespace '" << definitionsRef->targetNamespace << "'" << std::endl;
  }
  else if (!location)
    std::cerr << "\nWarning: wsdl definitions/import has no location attribute" << std::endl;
  return SOAP_OK;
}

int wsdl__import::traverse(wsdl__definitions& definitions)
{
  if (definitionsRef)
  {
    if (vflag)
      std::cerr << " Analyzing imported wsdl namespace '" << (namespace_ ? namespace_ : "(null)") << "' in wsdl namespace '" << (definitions.targetNamespace ? definitions.targetNamespace : "(null)") << "'" << std::endl;
    if (!definitionsRef->targetNamespace)
    {
      if (namespace_)
        definitionsRef->targetNamespace = namespace_;
      else
        definitionsRef->targetNamespace = definitions.targetNamespace;
    }
    definitionsRef->traverse();
    // collect imported artifacts into parent collections
    definitions.builtinTypes(definitionsRef->builtinTypes());
    definitions.builtinTypeDerivations(definitionsRef->builtinTypeDerivations());
    definitions.builtinElements(definitionsRef->builtinElements());
    definitions.builtinAttributes(definitionsRef->builtinAttributes());
  }
  return SOAP_OK;
}

void wsdl__import::definitionsPtr(wsdl__definitions *definitions)
{
  definitionsRef = definitions;
  if (!definitionsRef && vflag)
    std::cerr << "\nWarning: wsdl__import definitions set to NULL" << std::endl;
}

wsdl__definitions *wsdl__import::definitionsPtr() const
{
  return definitionsRef;
}

wsdl__import::wsdl__import()
{
  definitionsRef = NULL;
}

void wsdl__import::mark()
{
  if (Oflag > 1)
  {
    if (definitionsRef)
      definitionsRef->mark();
  }
}

////////////////////////////////////////////////////////////////////////////////
//
//      streams
//
////////////////////////////////////////////////////////////////////////////////

std::ostream &operator<<(std::ostream &o, const wsdl__definitions &e)
{
  if (!e.soap)
  {
    struct soap soap;
    soap_init2(&soap, SOAP_IO_DEFAULT, SOAP_XML_TREE | SOAP_C_UTFSTRING);
    soap_set_namespaces(&soap, namespaces);
    e.soap_serialize(&soap);
    soap_begin_send(&soap);
    e.soap_out(&soap, "wsdl:definitions", 0, NULL);
    soap_end_send(&soap);
    soap_destroy(&soap);
    soap_end(&soap);
    soap_done(&soap);
  }
  else
  {
    std::ostream *os = e.soap->os;
    e.soap->os = &o;
    e.soap_serialize(e.soap);
    soap_begin_send(e.soap);
    e.soap_out(e.soap, "wsdl:definitions", 0, NULL);
    soap_end_send(e.soap);
    e.soap->os = os;
  }
  return o;
}

std::istream &operator>>(std::istream &i, wsdl__definitions &e)
{
  if (!e.soap)
  {
    e.soap = soap_new1(SOAP_XML_TREE | SOAP_C_UTFSTRING);
    soap_set_namespaces(e.soap, namespaces);
  }
  std::istream *is = e.soap->is;
  e.soap->is = &i;
  if (soap_begin_recv(e.soap)
   || !e.soap_in(e.soap, "wsdl:", NULL)
   || soap_end_recv(e.soap))
  {
    // handle error? Note: e.soap->error is set and app should check
  }
  e.soap->is = is;
  return i;
}

////////////////////////////////////////////////////////////////////////////////
//
//      Miscellaneous
//
////////////////////////////////////////////////////////////////////////////////

extern "C" {

int warn_ignore(struct soap *soap, const char *tag)
{
  // We don't warn if the omitted element was an annotation or a documentation in an unexpected place
  if (!Mflag && soap->mustUnderstand)
  {
    fprintf(stderr, "\nError: must understand element '%s' at level %d with wsdl:required='true'. Suppress this error with -M\n", tag, soap->level);
    return soap->error = SOAP_MUSTUNDERSTAND;
  }
  if (!Wflag
   && soap_match_tag(soap, tag, "xs:annotation")
   && soap_match_tag(soap, tag, "xs:documentation")
   && soap_match_tag(soap, tag, "xs:appinfo"))
    fprintf(stderr, "\nWarning: unexpected element '%s' at level %d is skipped (safe to ignore)\n", tag, soap->level);
  if (soap->body && !soap_string_in(soap, 0, -1, -1, NULL))
    return soap->error;
  return SOAP_OK;
}

int show_ignore(struct soap *soap, const char *tag)
{
  warn_ignore(soap, tag);
  soap_print_fault_location(soap, stderr);
  return SOAP_OK;
}

} // end extern "C"
