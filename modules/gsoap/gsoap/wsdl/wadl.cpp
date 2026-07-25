/*
        wadl.cpp

        WADL binding schema implementation

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2000-2016, Robert van Engelen, Genivia Inc. All Rights Reserved.
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

extern const char *qname_token(const char*, const char*);
extern int is_builtin_qname(const char*);

////////////////////////////////////////////////////////////////////////////////
//
//      wadl
//
////////////////////////////////////////////////////////////////////////////////

int wadl__application::preprocess(wsdl__definitions& definitions)
{
  if (vflag)
    std::cerr << "Preprocessing wadl application" << std::endl;
  if (grammars)
    return grammars->preprocess(definitions);
  return SOAP_OK;
}

int wadl__application::traverse(wsdl__definitions& definitions)
{
  if (vflag)
    std::cerr << "Analyzing wadl application" << std::endl;
  for (std::vector<wadl__resources>::iterator i1 = resources.begin(); i1 != resources.end(); ++i1)
    (*i1).traverse(definitions);
  for (std::vector<wadl__resource_USCOREtype>::iterator i2 = resource_USCOREtype.begin(); i2 != resource_USCOREtype.end(); ++i2)
    (*i2).traverse(definitions);
  for (std::vector<wadl__method>::iterator i3 = method.begin(); i3 != method.end(); ++i3)
    (*i3).traverse(definitions);
  for (std::vector<wadl__representation>::iterator i4 = representation.begin(); i4 != representation.end(); ++i4)
    (*i4).traverse(definitions);
  for (std::vector<wadl__param>::iterator i5 = param.begin(); i5 != param.end(); ++i5)
    (*i5).traverse(definitions);
  return SOAP_OK;
}

void wadl__application::mark()
{
  if (Oflag > 1)
  {
    for (std::vector<wadl__representation>::iterator i4 = representation.begin(); i4 != representation.end(); ++i4)
      (*i4).mark();
    for (std::vector<wadl__param>::iterator i5 = param.begin(); i5 != param.end(); ++i5)
      (*i5).mark();
  }
}

int wadl__grammars::preprocess(wsdl__definitions& definitions)
{
  if (vflag)
    std::cerr << "Preprocessing wadl:grammars" << std::endl;
  for (std::vector<wadl__include>::iterator i = include.begin(); i != include.end(); ++i)
    (*i).preprocess(definitions);
  return SOAP_OK;
}

int wadl__include::preprocess(wsdl__definitions& definitions)
{
  if (vflag)
    std::cerr << "Preprocessing wadl:include href='" << (href ? href : "") << "'" << std::endl;
  if (href)
  {
    wsdl__import import;
    import.soap_default(definitions.soap);
    import.location = href;
    definitions.import.push_back(import);
  }
  return SOAP_OK;
}

wadl__link::wadl__link()
{
  linkRef = NULL;
}

int wadl__link::traverse(wsdl__definitions& definitions)
{
  if (vflag)
    std::cerr << " Analyzing wadl:link resource_type '" << (resource_USCOREtype ? resource_USCOREtype : "") << "'" << std::endl;
  linkRef = NULL;
  if (resource_USCOREtype)
  {
    if (*resource_USCOREtype != '#')
    {
      if (!Wflag)
        std::cerr << "\nWarning: external link resource_type='" << resource_USCOREtype << "' found that is not supported" << std::endl;
    }
    else if (definitions.appPtr())
    {
      for (std::vector<wadl__resource_USCOREtype>::iterator i = definitions.appPtr()->resource_USCOREtype.begin(); i != definitions.appPtr()->resource_USCOREtype.end(); ++i)
      {
        if ((*i).id && !strcmp((*i).id, resource_USCOREtype + 1))
        {
          linkRef = &*i;
          break;
        }
      }
    }
    if (!linkRef)
      if (!Wflag)
        std::cerr << "\nWarning: no wadl:resource_type with id '" << resource_USCOREtype << "' found" << std::endl;
  }
  return SOAP_OK;
}

void wadl__link::linkPtr(wadl__resource_USCOREtype *resource)
{
  linkRef = resource;
  if (!linkRef && vflag)
    std::cerr << "\nWarning: wadl__link link set to NULL" << std::endl;
}

const wadl__resource_USCOREtype *wadl__link::linkPtr() const
{
  return linkRef;
}

wadl__param::wadl__param()
{
  paramRef = NULL;
  simpleTypeRef = NULL;
  complexTypeRef = NULL;
}

int wadl__param::traverse(wsdl__definitions& definitions)
{
  if (vflag)
    std::cerr << " Analyzing wadl:param '" << (name ? name : "") << "' id '" << (id ? id : "") << "'" << std::endl;
  paramRef = NULL;
  simpleTypeRef = NULL;
  complexTypeRef = NULL;
  if (href)
  {
    if (*href != '#')
    {
      if (!Wflag)
        std::cerr << "\nWarning: external reference href='" << href << "' found that is not supported" << std::endl;
    }
    else if (definitions.appPtr())
    {
      for (std::vector<wadl__param>::iterator i = definitions.appPtr()->param.begin(); i != definitions.appPtr()->param.end(); ++i)
      {
        if ((*i).id && !strcmp((*i).id, href + 1))
        {
          paramRef = &*i;
          break;
        }
      }
    }
    if (!paramRef)
      if (!Wflag)
        std::cerr << "\nWarning: no wadl:param with id '" << href << "' found" << std::endl;
  }
  else
  {
    if (definitions.types)
    {
      for (std::vector<xs__schema*>::iterator schema = definitions.types->xs__schema_.begin(); schema != definitions.types->xs__schema_.end(); ++schema)
      {
        const char *token = qname_token(type, (*schema)->targetNamespace);
        if (token)
        {
          for (std::vector<xs__simpleType>::iterator st = (*schema)->simpleType.begin(); st != (*schema)->simpleType.end(); ++st)
          {
            if ((*st).name && !strcmp((*st).name, token))
            {
              simpleTypeRef = &(*st);
              if (vflag)
                std::cerr << "   Found wadl:param simpleType '" << (token ? token : "(null)") << "'" << std::endl;
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
                std::cerr << "   Found wadl:param complexType '" << (token ? token : "(null)") << "'" << std::endl;
              break;
            }
          }
        }
      }
    }
    if (type && !simpleTypeRef && !complexTypeRef)
    {
      if (is_builtin_qname(type))
        definitions.builtinType(type);
      else
        if (!Wflag)
          std::cerr << "\nWarning: no wadl:param type '" << type << "' found" << std::endl;
    }
  }
  if (link)
    link->traverse(definitions);
  return SOAP_OK;
}

void wadl__param::paramPtr(wadl__param *param)
{
  paramRef = param;
  if (!paramRef && vflag)
    std::cerr << "\nWarning: wadl__param param set to NULL" << std::endl;
}

const wadl__param *wadl__param::paramPtr() const
{
  return paramRef;
}

void wadl__param::simpleTypePtr(xs__simpleType *simpleType)
{
  simpleTypeRef = simpleType;
  if (!simpleTypeRef && vflag)
    std::cerr << "\nWarning: wadl__param simpleType set to NULL" << std::endl;
}

xs__simpleType *wadl__param::simpleTypePtr() const
{
  return simpleTypeRef;
}

void wadl__param::complexTypePtr(xs__complexType *complexType)
{
  complexTypeRef = complexType;
  if (!complexTypeRef && vflag)
    std::cerr << "\nWarning: wadl__param complexType set to NULL" << std::endl;
}

xs__complexType *wadl__param::complexTypePtr() const
{
  return complexTypeRef;
}

void wadl__param::mark()
{
  if (Oflag > 1)
  {
    if (simpleTypePtr())
      simpleTypePtr()->mark();
    if (complexTypePtr())
      complexTypePtr()->mark();
  }
}

wadl__representation::wadl__representation()
{
  representationRef = NULL;
  elementRef = NULL;
}

int wadl__representation::traverse(wsdl__definitions& definitions)
{
  for (std::vector<wadl__param>::iterator i = param.begin(); i != param.end(); ++i)
    (*i).traverse(definitions);
  if (vflag)
    std::cerr << " Analyzing wadl:representation id '" << (id ? id : "") << "'" << std::endl;
  elementRef = NULL;
  if (href)
  {
    if (*href != '#')
    {
      if (!Wflag)
        std::cerr << "\nWarning: external representation reference href='" << href << "' found that is not supported" << std::endl;
    }
    else if (definitions.appPtr())
    {
      for (std::vector<wadl__representation>::iterator i = definitions.appPtr()->representation.begin(); i != definitions.appPtr()->representation.end(); ++i)
      {
        if ((*i).id && !strcmp((*i).id, href + 1))
        {
          representationRef = &*i;
          break;
        }
      }
    }
    if (!representationRef)
      if (!Wflag)
        std::cerr << "\nWarning: no wadl:representation with id '" << href << "' found" << std::endl;
  }
  else
  {
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
                std::cerr << "   Found wadl:representation element '" << (token ? token : "(null)") << "'" << std::endl;
              break;
            }
          }
        }
      }
    }
    if (element && !elementRef)
    {
      if (is_builtin_qname(element))
        definitions.builtinElement(element);
      else
        if (!Wflag)
          std::cerr << "\nWarning: no wadl:representation element '" << element << "'" << std::endl;
    }
  }
  return SOAP_OK;
}

void wadl__representation::representationPtr(wadl__representation *representation)
{
  representationRef = representation;
  if (!representationRef && vflag)
    std::cerr << "\nWarning: wadl__representation representation set to NULL" << std::endl;
}

const wadl__representation *wadl__representation::representationPtr() const
{
  return representationRef;
}

void wadl__representation::elementPtr(xs__element *element)
{
  elementRef = element;
  if (!elementRef && vflag)
    std::cerr << "\nWarning: wadl__representation element set to NULL" << std::endl;
}

xs__element *wadl__representation::elementPtr() const
{
  return elementRef;
}

void wadl__representation::mark()
{
  if (Oflag > 1)
  {
    if (elementPtr())
      elementPtr()->mark();
  }
}

int wadl__request::traverse(wsdl__definitions& definitions)
{
  for (std::vector<wadl__param>::iterator i = param.begin(); i != param.end(); ++i)
    (*i).traverse(definitions);
  for (std::vector<wadl__representation>::iterator j = representation.begin(); j != representation.end(); ++j)
    (*j).traverse(definitions);
  return SOAP_OK;
}

void wadl__request::mark()
{
  if (Oflag > 1)
  {
    for (std::vector<wadl__param>::iterator i = param.begin(); i != param.end(); ++i)
      (*i).mark();
    for (std::vector<wadl__representation>::iterator j = representation.begin(); j != representation.end(); ++j)
      (*j).mark();
  }
}

wadl__method::wadl__method()
{
  methodRef = NULL;
}

int wadl__method::traverse(wsdl__definitions& definitions)
{
  if (vflag)
    std::cerr << " Analyzing wadl:method name '" << soap_wadl__HTTPMethods2s(definitions.soap, name) << "' id '" << (id ? id : "") << "'" << std::endl;
  if (request)
    request->traverse(definitions);
  for (std::vector<wadl__response>::iterator i = response.begin(); i != response.end(); ++i)
    (*i).traverse(definitions);
  methodRef = NULL;
  if (href)
  {
    if (*href != '#')
    {
      if (!Wflag)
        std::cerr << "\nWarning: external method href='" << href << "' found that is not supported" << std::endl;
    }
    else if (definitions.appPtr())
    {
      for (std::vector<wadl__method>::iterator i = definitions.appPtr()->method.begin(); i != definitions.appPtr()->method.end(); ++i)
      {
        if ((*i).id && !strcmp((*i).id, href + 1))
        {
          methodRef = &*i;
          break;
        }
      }
    }
    if (!methodRef)
      if (!Wflag)
        std::cerr << "\nWarning: no wadl:method with id '" << href << "' found" << std::endl;
  }
  return SOAP_OK;
}

void wadl__method::methodPtr(wadl__method *method)
{
  methodRef = method;
  if (!methodRef && vflag)
    std::cerr << "\nWarning: wadl__method method set to NULL" << std::endl;
}

const wadl__method *wadl__method::methodPtr() const
{
  return methodRef;
}

void wadl__method::mark()
{
  if (Oflag > 1)
  {
    if (request)
      request->mark();
    for (std::vector<wadl__response>::iterator i = response.begin(); i != response.end(); ++i)
      (*i).mark();
  }
}

int __wadl__method_resource_choice::traverse(wsdl__definitions& definitions)
{
  if (method)
    method->traverse(definitions);
  if (resource)
    resource->traverse(definitions);
  return SOAP_OK;
}

int wadl__resource_USCOREtype::traverse(wsdl__definitions& definitions)
{
  for (std::vector<wadl__param>::iterator i = param.begin(); i != param.end(); ++i)
    (*i).traverse(definitions);
  for (std::vector<__wadl__method_resource_choice>::iterator j = __choice.begin(); j != __choice.end(); ++j)
    (*j).traverse(definitions);
  return SOAP_OK;
}

int wadl__resource::traverse(wsdl__definitions& definitions)
{
  wadl__resource_USCOREtype::traverse(definitions);
  if (type)
  {
    const char *t = type;
    while (true)
    {
      const char *s = strchr(t, ' ');
      size_t n = s ? s - t : strlen(t);
      if (*t != '#')
      {
        if (!Wflag)
          std::cerr << "\nWarning: external resource in type='" << type << "' found that is not supported" << std::endl;
      }
      else if (definitions.appPtr())
      {
        for (std::vector<wadl__resource_USCOREtype>::iterator i = definitions.appPtr()->resource_USCOREtype.begin(); i != definitions.appPtr()->resource_USCOREtype.end(); ++i)
        {
          if ((*i).id && !strncmp((*i).id, t + 1, n))
          {
            typeRefs.push_back(&*i);
            break;
          }
        }
      }
      if (!s)
        break;
      t = s + 1;
    }
  }
  return SOAP_OK;
}

void wadl__resource::typePtr(wadl__resource_USCOREtype *type)
{
  if (type)
    typeRefs.push_back(type);
}

const std::vector<wadl__resource_USCOREtype*>& wadl__resource::typePtrs() const
{
  return typeRefs;
}

int wadl__resources::traverse(wsdl__definitions& definitions)
{
  for (std::vector<wadl__resource>::iterator i = resource.begin(); i != resource.end(); ++i)
    (*i).traverse(definitions);
  return SOAP_OK;
}

