/*
	soap.cpp

	WSDL/SOAP binding schema

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2001-2008, Robert van Engelen, Genivia Inc. All Rights Reserved.
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

#include "wsdlH.h"		// cannot include "schemaH.h"
#include "includes.h"

extern const char *qname_token(const char*, const char*);
extern int is_builtin_qname(const char*);

////////////////////////////////////////////////////////////////////////////////
//
//	soap:header
//
////////////////////////////////////////////////////////////////////////////////

int soap__header::traverse(wsdl__definitions& definitions)
{
  if (vflag)
    std::cerr << "    Analyzing soap header in wsdl namespace '" << (definitions.targetNamespace?definitions.targetNamespace:"") << "'" << std::endl;
  messageRef = NULL;
  partRef = NULL;
  const char *token = qname_token(message, definitions.targetNamespace);
  if (token)
  {
    for (std::vector<wsdl__message>::iterator message = definitions.message.begin(); message != definitions.message.end(); ++message)
    {
      if ((*message).name && !strcmp((*message).name, token))
      {
	messageRef = &(*message);
        if (vflag)
	  std::cerr << "     Found soap header part '" << (part?part:"") << "' message '" << (token?token:"") << "'" << std::endl;
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
	        std::cerr << "     Found soap header part '" << (part?part:"") << "' message '" << (token?token:"") << "'" << std::endl;
              break;
            }
          }
	}
      }
    }
  }
  if (messageRef)
  {
    if (part)
    {
      for (std::vector<wsdl__part>::iterator pt = messageRef->part.begin(); pt != messageRef->part.end(); ++pt)
        if ((*pt).name && !strcmp((*pt).name, part))
	{
	  partRef = &(*pt);
	  break;
        }
    }
    if (!partRef)
      std::cerr << "Warning: soap header has no matching part in message '" << (message?message:"") << "' in wsdl definitions '" << definitions.name << "' namespace '" << (definitions.targetNamespace?definitions.targetNamespace:"") << "'" << std::endl;
  }
  else
    std::cerr << "Warning: could not find soap header part '" << (part?part:"") << "' message '" << (message?message:"") << "' in wsdl definitions '" << definitions.name << "' namespace '" << (definitions.targetNamespace?definitions.targetNamespace:"") << "'" << std::endl;
  for (std::vector<soap__headerfault>::iterator i = headerfault.begin(); i != headerfault.end(); ++i)
    (*i).traverse(definitions);
  return SOAP_OK;
}

void soap__header::messagePtr(wsdl__message *message)
{
  messageRef = message;
}

wsdl__message *soap__header::messagePtr() const
{
  return messageRef;
}

void soap__header::partPtr(wsdl__part *part)
{
  partRef = part;
}

wsdl__part *soap__header::partPtr() const
{
  return partRef;
}

////////////////////////////////////////////////////////////////////////////////
//
//	soap:headerfault
//
////////////////////////////////////////////////////////////////////////////////

int soap__headerfault::traverse(wsdl__definitions& definitions)
{
  if (vflag)
    std::cerr << "    Analyzing soap headerfault in wsdl namespace '" << (definitions.targetNamespace?definitions.targetNamespace:"") << "'" << std::endl;
  messageRef = NULL;
  partRef = NULL;
  const char *token = qname_token(message, definitions.targetNamespace);
  if (token)
  {
    for (std::vector<wsdl__message>::iterator message = definitions.message.begin(); message != definitions.message.end(); ++message)
    {
      if ((*message).name && !strcmp((*message).name, token))
      {
	messageRef = &(*message);
        if (vflag)
	  std::cerr << "     Found soap headerfault part '" << (part?part:"") << "' message '" << (token?token:"") << "'" << std::endl;
        break;
      }
    }
  }
  else
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
	        std::cerr << "     Found soap headerfault part '" << (part?part:"") << "' message '" << (token?token:"") << "'" << std::endl;
              break;
            }
          }
	}
      }
    }
  }
  if (messageRef)
  {
    if (part)
    {
      for (std::vector<wsdl__part>::iterator pt = messageRef->part.begin(); pt != messageRef->part.end(); ++pt)
        if ((*pt).name && !strcmp((*pt).name, part))
	{
	  partRef = &(*pt);
	  break;
        }
    }
    if (!partRef)
      std::cerr << "Warning: soap headerfault has no matching part in message '" << (message?message:"") << "' in wsdl definitions '" << definitions.name << "' namespace '" << (definitions.targetNamespace?definitions.targetNamespace:"") << "'" << std::endl;
  }
  else
    std::cerr << "Warning: could not find soap headerfault part '" << (part?part:"") << "' message '" << (message?message:"") << "' in wSDL definitions '" << definitions.name << "' namespace '" << (definitions.targetNamespace?definitions.targetNamespace:"") << "'" << std::endl;
  return SOAP_OK;
}

void soap__headerfault::messagePtr(wsdl__message *message)
{
  messageRef = message;
}

wsdl__message *soap__headerfault::messagePtr() const
{
  return messageRef;
}

void soap__headerfault::partPtr(wsdl__part *part)
{
  partRef = part;
}

wsdl__part *soap__headerfault::partPtr() const
{
  return partRef;
}

////////////////////////////////////////////////////////////////////////////////
//
//	wsoap:header
//
////////////////////////////////////////////////////////////////////////////////

int wsoap__header::traverse(wsdl__definitions& definitions)
{
  if (vflag)
    std::cerr << "    Analyzing soap header in wsdl namespace '" << (definitions.targetNamespace?definitions.targetNamespace:"") << "'" << std::endl;
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
                std::cerr << "   Found soap header element '" << (token?token:"") << "'" << std::endl;
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
          std::cerr << "Warning: no soap header element '" << element << "' in wsdl definitions '" << (definitions.name?definitions.name:"") << "' namespace '" << (definitions.targetNamespace?definitions.targetNamespace:"") << "'" << std::endl;
    }
  }
  return SOAP_OK;
}

void wsoap__header::elementPtr(xs__element *element)
{
  elementRef = element;
}

xs__element *wsoap__header::elementPtr() const
{
  return elementRef;
}

void wsoap__header::mark()
{
  if (Oflag > 1)
    if (elementPtr())
      elementPtr()->mark();
}
