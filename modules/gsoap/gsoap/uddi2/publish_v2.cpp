/*

publish_v2.cpp

UDDI V2 Publish Interface

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

#include "pubH.h"
#define SOAP_NMAC static
#define uddiH_H
#include "PublishSoap.nsmap"

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:add_publisherAssertions
//
////////////////////////////////////////////////////////////////////////////////

uddi2__add_USCOREpublisherAssertions::uddi2__add_USCOREpublisherAssertions(struct soap *soap)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->generic = "2.0";
}

uddi2__add_USCOREpublisherAssertions::uddi2__add_USCOREpublisherAssertions(struct soap *soap, std::vector<uddi2__publisherAssertion*> publisherAssertions)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->publisherAssertion = publisherAssertions;

  this->generic = "2.0";
}

uddi2__dispositionReport *uddi2__add_USCOREpublisherAssertions::send(const char *endpoint, char *authInfo)
{
  // Set authorization token
  this->authInfo = authInfo;

  // Allocate result
  uddi2__dispositionReport *result = soap_new_uddi2__dispositionReport(soap, -1);

  // Invoke the wrapper
  soap_set_namespaces(soap, namespaces);
  if (soap_call___pub2__add_USCOREpublisherAssertions(soap, endpoint, NULL, this, result))
    return NULL;

  return result;
} 

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:delete_binding
//
////////////////////////////////////////////////////////////////////////////////

uddi2__delete_USCOREbinding::uddi2__delete_USCOREbinding(struct soap *soap)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->generic = "2.0";
}

uddi2__delete_USCOREbinding::uddi2__delete_USCOREbinding(struct soap *soap, const char *bindingKey)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->bindingKey.push_back(soap_strdup(soap, bindingKey));

  this->generic = "2.0";
}

uddi2__delete_USCOREbinding::uddi2__delete_USCOREbinding(struct soap *soap, std::vector<char*> bindingKeys)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->bindingKey = bindingKeys;

  this->generic = "2.0";
}

uddi2__dispositionReport *uddi2__delete_USCOREbinding::send(const char *endpoint, char *authInfo)
{
  // Set authorization token
  this->authInfo = authInfo;

  // Allocate result
  uddi2__dispositionReport *result = soap_new_uddi2__dispositionReport(soap, -1);

  // Invoke the wrapper
  soap_set_namespaces(soap, namespaces);
  if (soap_call___pub2__delete_USCOREbinding(soap, endpoint, NULL, this, result))
    return NULL;

  return result;
}

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:delete_business
//
////////////////////////////////////////////////////////////////////////////////

uddi2__delete_USCOREbusiness::uddi2__delete_USCOREbusiness(struct soap *soap)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->generic = "2.0";
}

uddi2__delete_USCOREbusiness::uddi2__delete_USCOREbusiness(struct soap *soap, const char *businessKey)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->businessKey.push_back(soap_strdup(soap, businessKey));

  this->generic = "2.0";
}

uddi2__delete_USCOREbusiness::uddi2__delete_USCOREbusiness(struct soap *soap, std::vector<char*> businessKeys)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->businessKey = businessKeys;

  this->generic = "2.0";
}

uddi2__dispositionReport *uddi2__delete_USCOREbusiness::send(const char *endpoint, char *authInfo)
{
  // Set authorization token
  this->authInfo = authInfo;

  // Allocate result
  uddi2__dispositionReport *result = soap_new_uddi2__dispositionReport(soap, -1);

  // Invoke the wrapper
  soap_set_namespaces(soap, namespaces);
  if (soap_call___pub2__delete_USCOREbusiness(soap, endpoint, NULL, this, result))
    return NULL;

  return result;
}

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:delete_service
//
////////////////////////////////////////////////////////////////////////////////

uddi2__delete_USCOREservice::uddi2__delete_USCOREservice(struct soap *soap)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->generic = "2.0";
}

uddi2__delete_USCOREservice::uddi2__delete_USCOREservice(struct soap *soap, const char *serviceKey)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->serviceKey.push_back(soap_strdup(soap, serviceKey));

  this->generic = "2.0";
}

uddi2__delete_USCOREservice::uddi2__delete_USCOREservice(struct soap *soap, std::vector<char*> serviceKeys)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->serviceKey = serviceKeys;

  this->generic = "2.0";
}

uddi2__dispositionReport *uddi2__delete_USCOREservice::send(const char *endpoint, char *authInfo)
{
  // Set authorization token
  this->authInfo = authInfo;

  // Allocate result
  uddi2__dispositionReport *result = soap_new_uddi2__dispositionReport(soap, -1);

  // Invoke the wrapper
  soap_set_namespaces(soap, namespaces);
  if (soap_call___pub2__delete_USCOREservice(soap, endpoint, NULL, this, result))
    return NULL;

  return result;
}

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:delete_tModel
//
////////////////////////////////////////////////////////////////////////////////

uddi2__delete_USCOREtModel::uddi2__delete_USCOREtModel(struct soap *soap)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->generic = "2.0";
}

uddi2__delete_USCOREtModel::uddi2__delete_USCOREtModel(struct soap *soap, const char *tModelKey)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->tModelKey.push_back(soap_strdup(soap, tModelKey));

  this->generic = "2.0";
}

uddi2__delete_USCOREtModel::uddi2__delete_USCOREtModel(struct soap *soap, std::vector<char*> tModelKeys)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->tModelKey = tModelKeys;

  this->generic = "2.0";
}

uddi2__dispositionReport *uddi2__delete_USCOREtModel::send(const char *endpoint, char *authInfo)
{
  // Set authorization token
  this->authInfo = authInfo;

  // Allocate result
  uddi2__dispositionReport *result = soap_new_uddi2__dispositionReport(soap, -1);

  // Invoke the wrapper
  soap_set_namespaces(soap, namespaces);
  if (soap_call___pub2__delete_USCOREtModel(soap, endpoint, NULL, this, result))
    return NULL;

  return result;
}

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:delete_publisherAssertions
//
////////////////////////////////////////////////////////////////////////////////

uddi2__delete_USCOREpublisherAssertions::uddi2__delete_USCOREpublisherAssertions(struct soap *soap)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->generic = "2.0";
}

uddi2__delete_USCOREpublisherAssertions::uddi2__delete_USCOREpublisherAssertions(struct soap *soap, std::vector<uddi2__publisherAssertion*> publisherAssertions)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->publisherAssertion = publisherAssertions;

  this->generic = "2.0";
}

uddi2__dispositionReport *uddi2__delete_USCOREpublisherAssertions::send(const char *endpoint, char *authInfo)
{
  // Set authorization token
  this->authInfo = authInfo;

  // Allocate result
  uddi2__dispositionReport *result = soap_new_uddi2__dispositionReport(soap, -1);

  // Invoke the wrapper
  soap_set_namespaces(soap, namespaces);
  if (soap_call___pub2__delete_USCOREpublisherAssertions(soap, endpoint, NULL, this, result))
    return NULL;

  return result;
}

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:discard_authToken
//
////////////////////////////////////////////////////////////////////////////////

uddi2__discard_USCOREauthToken::uddi2__discard_USCOREauthToken(struct soap *soap)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->generic = "2.0";
}

uddi2__discard_USCOREauthToken::uddi2__discard_USCOREauthToken(struct soap *soap, const char *authInfo)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->authInfo = soap_strdup(soap, authInfo);

  this->generic = "2.0";
}

uddi2__dispositionReport *uddi2__discard_USCOREauthToken::send(const char *endpoint)
{
  // Allocate result
  uddi2__dispositionReport *result = soap_new_uddi2__dispositionReport(soap, -1);

  // Invoke the wrapper
  soap_set_namespaces(soap, namespaces);
  if (soap_call___pub2__discard_USCOREauthToken(soap, endpoint, NULL, this, result))
    return NULL;

  return result;
}

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:get_assertionStatusReport
//
////////////////////////////////////////////////////////////////////////////////

uddi2__get_USCOREassertionStatusReport::uddi2__get_USCOREassertionStatusReport(struct soap *soap)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->generic = "2.0";
}

uddi2__get_USCOREassertionStatusReport::uddi2__get_USCOREassertionStatusReport(struct soap *soap, const char *completionStatus)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->completionStatus = soap_strdup(soap, completionStatus);

  this->generic = "2.0";
}

uddi2__assertionStatusReport *uddi2__get_USCOREassertionStatusReport::send(const char *endpoint, char *authInfo)
{
  // Set authorization token
  this->authInfo = authInfo;

  // Allocate result
  uddi2__assertionStatusReport *result = soap_new_uddi2__assertionStatusReport(soap, -1);

  // Invoke the wrapper
  soap_set_namespaces(soap, namespaces);
  if (soap_call___pub2__get_USCOREassertionStatusReport(soap, endpoint, NULL, this, result))
    return NULL;

  return result;
}

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:get_authToken
//
////////////////////////////////////////////////////////////////////////////////

uddi2__get_USCOREauthToken::uddi2__get_USCOREauthToken(struct soap *soap)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->generic = "2.0";
}

uddi2__get_USCOREauthToken::uddi2__get_USCOREauthToken(struct soap *soap, const char *userid, const char *passwd)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->userID = soap_strdup(soap, userid);
  this->cred = soap_strdup(soap, passwd);

  this->generic = "2.0";
}

uddi2__authToken *uddi2__get_USCOREauthToken::send(const char *endpoint)
{
  // Allocate result
  uddi2__authToken *result = soap_new_uddi2__authToken(soap, -1);

  // Invoke the wrapper
  soap_set_namespaces(soap, namespaces);
  if (soap_call___pub2__get_USCOREauthToken(soap, endpoint, NULL, this, result))
    return NULL;

  return result;
}

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:get_publisherAssertions
//
////////////////////////////////////////////////////////////////////////////////

uddi2__get_USCOREpublisherAssertions::uddi2__get_USCOREpublisherAssertions(struct soap *soap)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->generic = "2.0";
}

uddi2__publisherAssertions *uddi2__get_USCOREpublisherAssertions::send(const char *endpoint, char *authInfo)
{
  // Set authorization token
  this->authInfo = authInfo;

  // Allocate result
  uddi2__publisherAssertions *result = soap_new_uddi2__publisherAssertions(soap, -1);

  // Invoke the wrapper
  soap_set_namespaces(soap, namespaces);
  if (soap_call___pub2__get_USCOREpublisherAssertions(soap, endpoint, NULL, this, result))
    return NULL;

  return result;
}

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:get_registeredInfo
//
////////////////////////////////////////////////////////////////////////////////

uddi2__get_USCOREregisteredInfo::uddi2__get_USCOREregisteredInfo(struct soap *soap)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->generic = "2.0";
}

uddi2__registeredInfo *uddi2__get_USCOREregisteredInfo::send(const char *endpoint, char *authInfo)
{
  // Set authorization token
  this->authInfo = authInfo;

  // Allocate result
  uddi2__registeredInfo *result = soap_new_uddi2__registeredInfo(soap, -1);

  // Invoke the wrapper
  soap_set_namespaces(soap, namespaces);
  if (soap_call___pub2__get_USCOREregisteredInfo(soap, endpoint, NULL, this, result))
    return NULL;

  return result;
}

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:save_binding
//
////////////////////////////////////////////////////////////////////////////////

uddi2__save_USCOREbinding::uddi2__save_USCOREbinding(struct soap *soap)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->generic = "2.0";
}

uddi2__save_USCOREbinding::uddi2__save_USCOREbinding(struct soap *soap, uddi2__bindingTemplate &bindingTemplate)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->bindingTemplate.push_back(&bindingTemplate);

  this->generic = "2.0";
}

uddi2__save_USCOREbinding::uddi2__save_USCOREbinding(struct soap *soap, std::vector<uddi2__bindingTemplate*> bindingTemplates)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->bindingTemplate = bindingTemplates;

  this->generic = "2.0";
}

uddi2__bindingDetail *uddi2__save_USCOREbinding::send(const char *endpoint, char *authInfo)
{
  // Set authorization token
  this->authInfo = authInfo;

  // Allocate result
  uddi2__bindingDetail *result = soap_new_uddi2__bindingDetail(soap, -1);

  // Invoke the wrapper
  soap_set_namespaces(soap, namespaces);
  if (soap_call___pub2__save_USCOREbinding(soap, endpoint, NULL, this, result))
    return NULL;

  return result;
}

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:save_business
//
////////////////////////////////////////////////////////////////////////////////

uddi2__save_USCOREbusiness::uddi2__save_USCOREbusiness(struct soap *soap)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->generic = "2.0";
}

uddi2__save_USCOREbusiness::uddi2__save_USCOREbusiness(struct soap *soap, uddi2__businessEntity &businessEntity)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->businessEntity.push_back(&businessEntity);

  this->generic = "2.0";
}

uddi2__save_USCOREbusiness::uddi2__save_USCOREbusiness(struct soap *soap, std::vector<uddi2__businessEntity*> businessEntities)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->businessEntity = businessEntities;

  this->generic = "2.0";
}

uddi2__businessDetail *uddi2__save_USCOREbusiness::send(const char *endpoint, char *authInfo)
{
  // Set authorization token
  this->authInfo = authInfo;

  // Allocate result
  uddi2__businessDetail *result = soap_new_uddi2__businessDetail(soap, -1);

  // Invoke the wrapper
  soap_set_namespaces(soap, namespaces);
  if (soap_call___pub2__save_USCOREbusiness(soap, endpoint, NULL, this, result))
    return NULL;

  return result;
}

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:save_service
//
////////////////////////////////////////////////////////////////////////////////

uddi2__save_USCOREservice::uddi2__save_USCOREservice(struct soap *soap)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->generic = "2.0";
}

uddi2__save_USCOREservice::uddi2__save_USCOREservice(struct soap *soap, uddi2__businessService &businessService)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->businessService.push_back(&businessService);

  this->generic = "2.0";
}

uddi2__save_USCOREservice::uddi2__save_USCOREservice(struct soap *soap, std::vector<uddi2__businessService*> businessServices)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->businessService = businessServices;

  this->generic = "2.0";
}

uddi2__serviceDetail *uddi2__save_USCOREservice::send(const char *endpoint, char *authInfo)
{
  // Set authorization token
  this->authInfo = authInfo;

  // Allocate result
  uddi2__serviceDetail *result = soap_new_uddi2__serviceDetail(soap, -1);

  // Invoke the wrapper
  soap_set_namespaces(soap, namespaces);
  if (soap_call___pub2__save_USCOREservice(soap, endpoint, NULL, this, result))
    return NULL;

  return result;
}

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:save_tModel
//
////////////////////////////////////////////////////////////////////////////////

uddi2__save_USCOREtModel::uddi2__save_USCOREtModel(struct soap *soap)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->generic = "2.0";
}

uddi2__save_USCOREtModel::uddi2__save_USCOREtModel(struct soap *soap, uddi2__tModel &tModel)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->tModel.push_back(&tModel);

  this->generic = "2.0";
}

uddi2__save_USCOREtModel::uddi2__save_USCOREtModel(struct soap *soap, std::vector<uddi2__tModel*> tModels)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->tModel = tModels;

  this->generic = "2.0";
}

uddi2__tModelDetail *uddi2__save_USCOREtModel::send(const char *endpoint, char *authInfo)
{
  // Set authorization token
  this->authInfo = authInfo;

  // Allocate result
  uddi2__tModelDetail *result = soap_new_uddi2__tModelDetail(soap, -1);

  // Invoke the wrapper
  soap_set_namespaces(soap, namespaces);
  if (soap_call___pub2__save_USCOREtModel(soap, endpoint, NULL, this, result))
    return NULL;

  return result;
}

////////////////////////////////////////////////////////////////////////////////
//
//	uddi2:set_publisherAssertions
//
////////////////////////////////////////////////////////////////////////////////

uddi2__set_USCOREpublisherAssertions::uddi2__set_USCOREpublisherAssertions(struct soap *soap)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->generic = "2.0";
}

uddi2__set_USCOREpublisherAssertions::uddi2__set_USCOREpublisherAssertions(struct soap *soap, std::vector<uddi2__publisherAssertion*> publisherAssertions)
{
  // Initialize this object and associate it with the gSOAP context
  soap_default(soap);

  this->publisherAssertion = publisherAssertions;

  this->generic = "2.0";
}

uddi2__publisherAssertions *uddi2__set_USCOREpublisherAssertions::send(const char *endpoint, char *authInfo)
{
  // Set authorization token
  this->authInfo = authInfo;

  // Allocate result
  uddi2__publisherAssertions *result = soap_new_uddi2__publisherAssertions(soap, -1);

  // Invoke the wrapper
  soap_set_namespaces(soap, namespaces);
  if (soap_call___pub2__set_USCOREpublisherAssertions(soap, endpoint, NULL, this, result))
    return NULL;

  return result;
}

