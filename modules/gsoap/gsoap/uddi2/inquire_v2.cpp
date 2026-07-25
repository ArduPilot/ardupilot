/*

inquire_v2.cpp

UDDI V2 Inquiry Interface

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2004-2005, Robert van Engelen, Genivia Inc. All Rights Reserved.
This software is released under one of the following two licenses:
GPL or Genivia's license for commercial use.
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

#include "inqH.h"

#undef SOAP_NMAC
#define SOAP_NMAC static

#define uddiH_H

#include "InquireSoap.nsmap"

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:find_binding
//
////////////////////////////////////////////////////////////////////////////////

uddi2__find_USCOREbinding::uddi2__find_USCOREbinding(struct soap *soap)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->generic = "2.0";
}

uddi2__find_USCOREbinding::uddi2__find_USCOREbinding(struct soap *soap, const char *tModelKey)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  // Create a new UDDI tModelKey
  this->tModelBag = soap_new_uddi2__tModelBag(soap, -1);
  this->tModelBag->soap_default(soap);
  this->tModelBag->tModelKey.push_back(soap_strdup(soap, tModelKey));

  this->generic = "2.0";
}

uddi2__find_USCOREbinding::uddi2__find_USCOREbinding(struct soap *soap, std::vector<char*> tModelKeys)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  // Create a new UDDI tModelKey Bag
  this->tModelBag = soap_new_uddi2__tModelBag(soap, -1);
  this->tModelBag->soap_default(soap);
  this->tModelBag->tModelKey = tModelKeys;

  this->generic = "2.0";
}

uddi2__bindingDetail *uddi2__find_USCOREbinding::send(const char *endpoint)
{
  // Allocate result
  uddi2__bindingDetail *result = soap_new_uddi2__bindingDetail(soap, -1);

  // Invoke the wrapper
  soap_set_namespaces(soap, namespaces);
  if (soap_call___inq2__find_USCOREbinding(soap, endpoint, NULL, this, result))
    return NULL;

  return result;
} 

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:find_business
//
////////////////////////////////////////////////////////////////////////////////

uddi2__find_USCOREbusiness::uddi2__find_USCOREbusiness(struct soap *soap)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->generic = "2.0";
}

uddi2__find_USCOREbusiness::uddi2__find_USCOREbusiness(struct soap *soap, const char *name)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  // Create a new UDDI name
  uddi2__name *uname = soap_new_uddi2__name(soap, -1);
  uname->soap_default(soap);
  uname->__item = soap_strdup(soap, name);
  this->name.push_back(uname);

  this->generic = "2.0";
}

uddi2__find_USCOREbusiness::uddi2__find_USCOREbusiness(struct soap *soap, std::vector<uddi2__keyedReference*> keyedReferences)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  // Create a new UDDI tModelBag
  this->categoryBag = soap_new_uddi2__categoryBag(soap, -1);
  this->categoryBag->soap_default(soap);
  this->categoryBag->keyedReference = keyedReferences;

  this->generic = "2.0";
}

uddi2__find_USCOREbusiness::uddi2__find_USCOREbusiness(struct soap *soap, std::vector<char*> tModelKeys)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  // Create a new UDDI tModelBag
  this->tModelBag = soap_new_uddi2__tModelBag(soap, -1);
  this->tModelBag->soap_default(soap);
  this->tModelBag->tModelKey = tModelKeys;

  this->generic = "2.0";
}

uddi2__businessList *uddi2__find_USCOREbusiness::send(const char *endpoint)
{
  // Allocate result
  uddi2__businessList *result = soap_new_uddi2__businessList(soap, -1);

  // Invoke the wrapper
  soap_set_namespaces(soap, namespaces);
  if (soap_call___inq2__find_USCOREbusiness(soap, endpoint, NULL, this, result))
    return NULL;

  return result;
} 

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:find_relatedBusinesses
//
////////////////////////////////////////////////////////////////////////////////

uddi2__find_USCORErelatedBusinesses::uddi2__find_USCORErelatedBusinesses(struct soap *soap)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->generic = "2.0";
}

uddi2__find_USCORErelatedBusinesses::uddi2__find_USCORErelatedBusinesses(struct soap *soap, const char *businessKey)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->businessKey = soap_strdup(soap, businessKey);

  this->generic = "2.0";
}

uddi2__relatedBusinessesList *uddi2__find_USCORErelatedBusinesses::send(const char *endpoint)
{
  // Allocate result
  uddi2__relatedBusinessesList *result = soap_new_uddi2__relatedBusinessesList(soap, -1);

  // Invoke the wrapper
  soap_set_namespaces(soap, namespaces);
  if (soap_call___inq2__find_USCORErelatedBusinesses(soap, endpoint, NULL, this, result))
    return NULL;

  return result;
} 

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:find_service
//
////////////////////////////////////////////////////////////////////////////////

uddi2__find_USCOREservice::uddi2__find_USCOREservice(struct soap *soap)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->generic = "2.0";
}

uddi2__find_USCOREservice::uddi2__find_USCOREservice(struct soap *soap, const char *name)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  // Create a new UDDI name
  uddi2__name *uname = soap_new_uddi2__name(soap, -1);
  uname->soap_default(soap);
  uname->__item = soap_strdup(soap, name);
  this->name.push_back(uname);

  this->generic = "2.0";
}

uddi2__find_USCOREservice::uddi2__find_USCOREservice(struct soap *soap, std::vector<uddi2__keyedReference*> keyedReferences)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  // Create a new UDDI tModelBag
  this->categoryBag = soap_new_uddi2__categoryBag(soap, -1);
  this->categoryBag->soap_default(soap);
  this->categoryBag->keyedReference = keyedReferences;

  this->generic = "2.0";
}

uddi2__find_USCOREservice::uddi2__find_USCOREservice(struct soap *soap, std::vector<char*> tModelKeys)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  // Create a new UDDI tModelBag
  this->tModelBag = soap_new_uddi2__tModelBag(soap, -1);
  this->tModelBag->soap_default(soap);
  this->tModelBag->tModelKey = tModelKeys;

  this->generic = "2.0";
}

uddi2__serviceList* uddi2__find_USCOREservice::send(const char *endpoint)
{
  // Allocate result
  uddi2__serviceList *result = soap_new_uddi2__serviceList(soap, -1);

  // Invoke the wrapper
  soap_set_namespaces(soap, namespaces);
  if (soap_call___inq2__find_USCOREservice(soap, endpoint, NULL, this, result))
    return NULL;

  return result;
} 

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:find_tModel
//
////////////////////////////////////////////////////////////////////////////////

uddi2__find_USCOREtModel::uddi2__find_USCOREtModel(struct soap *soap)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->generic = "2.0";
}

uddi2__find_USCOREtModel::uddi2__find_USCOREtModel(struct soap *soap, const char *name)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  // Create a new UDDI name
  this->name = soap_new_uddi2__name(soap, -1);
  this->name->soap_default(soap);
  this->name->__item = soap_strdup(soap, name);

  this->generic = "2.0";
}

uddi2__find_USCOREtModel::uddi2__find_USCOREtModel(struct soap *soap, std::vector<uddi2__keyedReference*> keyedReferences)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  // Create a new UDDI tModelBag
  this->categoryBag = soap_new_uddi2__categoryBag(soap, -1);
  this->categoryBag->soap_default(soap);
  this->categoryBag->keyedReference = keyedReferences;

  this->generic = "2.0";
}

uddi2__tModelList* uddi2__find_USCOREtModel::send(const char *endpoint)
{
  // Allocate result
  uddi2__tModelList *result = soap_new_uddi2__tModelList(soap, -1);

  // Invoke the wrapper
  soap_set_namespaces(soap, namespaces);
  if (soap_call___inq2__find_USCOREtModel(soap, endpoint, NULL, this, result))
    return NULL;

  return result;
} 

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:get_bindingDetail
//
////////////////////////////////////////////////////////////////////////////////

uddi2__get_USCOREbindingDetail::uddi2__get_USCOREbindingDetail(struct soap *soap)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->generic = "2.0";
}

uddi2__get_USCOREbindingDetail::uddi2__get_USCOREbindingDetail(struct soap *soap, const char *bindingKey)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->bindingKey.push_back(soap_strdup(soap, bindingKey));

  this->generic = "2.0";
}

uddi2__get_USCOREbindingDetail::uddi2__get_USCOREbindingDetail(struct soap *soap, std::vector<char*> bindingKeys)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->bindingKey = bindingKeys;

  this->generic = "2.0";
}

uddi2__bindingDetail *uddi2__get_USCOREbindingDetail::send(const char *endpoint)
{
  // Allocate result
  uddi2__bindingDetail *result = soap_new_uddi2__bindingDetail(soap, -1);

  // Invoke the wrapper
  soap_set_namespaces(soap, namespaces);
  if (soap_call___inq2__get_USCOREbindingDetail(soap, endpoint, NULL, this, result))
    return NULL;

  return result;
} 

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:get_businessDetail
//
////////////////////////////////////////////////////////////////////////////////

uddi2__get_USCOREbusinessDetail::uddi2__get_USCOREbusinessDetail(struct soap *soap)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->generic = "2.0";
}

uddi2__get_USCOREbusinessDetail::uddi2__get_USCOREbusinessDetail(struct soap *soap, const char *businessKey)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->businessKey.push_back(soap_strdup(soap, businessKey));

  this->generic = "2.0";
}

uddi2__get_USCOREbusinessDetail::uddi2__get_USCOREbusinessDetail(struct soap *soap, std::vector<char*> businessKeys)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->businessKey = businessKeys;

  this->generic = "2.0";
}

uddi2__businessDetail *uddi2__get_USCOREbusinessDetail::send(const char *endpoint)
{
  // Allocate result
  uddi2__businessDetail *result = soap_new_uddi2__businessDetail(soap, -1);

  // Invoke the wrapper
  soap_set_namespaces(soap, namespaces);
  if (soap_call___inq2__get_USCOREbusinessDetail(soap, endpoint, NULL, this, result))
    return NULL;

  return result;
} 

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:get_businessDetailExt
//
////////////////////////////////////////////////////////////////////////////////

uddi2__get_USCOREbusinessDetailExt::uddi2__get_USCOREbusinessDetailExt(struct soap *soap)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->generic = "2.0";
}

uddi2__get_USCOREbusinessDetailExt::uddi2__get_USCOREbusinessDetailExt(struct soap *soap, const char *businessKey)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->businessKey.push_back(soap_strdup(soap, businessKey));

  this->generic = "2.0";
}

uddi2__get_USCOREbusinessDetailExt::uddi2__get_USCOREbusinessDetailExt(struct soap *soap, std::vector<char*> businessKeys)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->businessKey = businessKeys;

  this->generic = "2.0";
}

uddi2__businessDetailExt *uddi2__get_USCOREbusinessDetailExt::send(const char *endpoint)
{
  // Allocate result
  uddi2__businessDetailExt *result = soap_new_uddi2__businessDetailExt(soap, -1);

  // Invoke the wrapper
  soap_set_namespaces(soap, namespaces);
  if (soap_call___inq2__get_USCOREbusinessDetailExt(soap, endpoint, NULL, this, result))
    return NULL;

  return result;
} 

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:get_serviceDetail
//
////////////////////////////////////////////////////////////////////////////////

uddi2__get_USCOREserviceDetail::uddi2__get_USCOREserviceDetail(struct soap *soap)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->generic = "2.0";
}

uddi2__get_USCOREserviceDetail::uddi2__get_USCOREserviceDetail(struct soap *soap, const char *serviceKey)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->serviceKey.push_back(soap_strdup(soap, serviceKey));

  this->generic = "2.0";
}

uddi2__get_USCOREserviceDetail::uddi2__get_USCOREserviceDetail(struct soap *soap, std::vector<char*> serviceKeys)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->serviceKey = serviceKeys;

  this->generic = "2.0";
}

uddi2__serviceDetail *uddi2__get_USCOREserviceDetail::send(const char *endpoint)
{
  // Allocate result
  uddi2__serviceDetail *result = soap_new_uddi2__serviceDetail(soap, -1);

  // Invoke the wrapper
  soap_set_namespaces(soap, namespaces);
  if (soap_call___inq2__get_USCOREserviceDetail(soap, endpoint, NULL, this, result))
    return NULL;

  return result;
} 

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:get_tModelDetail
//
////////////////////////////////////////////////////////////////////////////////

uddi2__get_USCOREtModelDetail::uddi2__get_USCOREtModelDetail(struct soap *soap)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->generic = "2.0";
}

uddi2__get_USCOREtModelDetail::uddi2__get_USCOREtModelDetail(struct soap *soap, const char *tModelKey)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->tModelKey.push_back(soap_strdup(soap, tModelKey));

  this->generic = "2.0";
}

uddi2__get_USCOREtModelDetail::uddi2__get_USCOREtModelDetail(struct soap *soap, std::vector<char*> tModelKeys)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->tModelKey = tModelKeys;

  this->generic = "2.0";
}

uddi2__tModelDetail *uddi2__get_USCOREtModelDetail::send(const char *endpoint)
{
  // Allocate result
  uddi2__tModelDetail *result = soap_new_uddi2__tModelDetail(soap, -1);

  // Invoke the wrapper
  soap_set_namespaces(soap, namespaces);
  if (soap_call___inq2__get_USCOREtModelDetail(soap, endpoint, NULL, this, result))
    return NULL;

  return result;
} 

