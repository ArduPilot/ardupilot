/*

example3.cpp

Example UDDI V2 Client

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

const char *server = "https://uddi.xmethods.net/publish";

const char *userid = "..."; // user ID to access UDDI server
const char *passwd = "..."; // password to access UDDI server

int main(int argc, char **argv)
{ 
  // Create a gSOAP context
  struct soap *soap = soap_new();

  // Setup SSL context (optional) to verify server's credentials
  if (soap_ssl_client_context(soap, SOAP_SSL_DEFAULT, NULL, NULL, "cacerts.pem", NULL, NULL))
  { 
    soap_print_fault(soap, stderr);
    exit(1);
  }

  // Step 1: Get an authorization token from the UDDI server
  uddi2__get_USCOREauthToken get_authToken(soap, userid, passwd);
  uddi2__authToken *authToken = get_authToken.send(server);

  // Check if authorized
  if (!authToken)
  {
    soap_print_fault(soap, stderr);
    exit(1);
  }

  // Authorization info provided by server for this session
  char *authInfo = authToken->authInfo;

  // Step 2: Create a tModel for the WSDL to be published
  uddi2__tModel tModel;
  tModel.soap_default(soap);

  // Create the tModel and service name
  tModel.name = soap_new_uddi2__name(soap, -1);
  tModel.name->__item = "...";
  tModel.name->xml__lang_ = "en";

  // Create XMethods description elements (see http://www.xmethods.net/ve2/UDDI.po)
  uddi2__description *description = soap_new_uddi2__description(soap, 6);
  description[0].__item = "SHORT DESCRIPTION: ...";
  description[0].xml__lang_ = "en";
  description[1].__item = "SHORT DESCRIPTION: ...";
  description[1].xml__lang_ = "en";
  description[2].__item = "USAGE NOTES: ...";
  description[2].xml__lang_ = "en";
  description[3].__item = "HOMEPAGE URL: ...";
  description[3].xml__lang_ = "en";
  description[4].__item = "CONTACT EMAIL: ...";
  description[4].xml__lang_ = "en";
  description[5].__item = "IMPLEMENTATION: ...";
  description[5].xml__lang_ = "en";

  // Add the four description elements to the tModel
  tModel.description.push_back(description + 0);
  tModel.description.push_back(description + 1);
  tModel.description.push_back(description + 2);
  tModel.description.push_back(description + 4);

  // Add an overviewDoc element with description and overviewURL
  tModel.overviewDoc = soap_new_uddi2__overviewDoc(soap, -1);
  tModel.overviewDoc->soap_default(soap);
  tModel.overviewDoc->description.push_back(soap_new_uddi2__description(soap, -1));
  tModel.overviewDoc->description[0]->__item = "WSDL source document";
  tModel.overviewDoc->description[0]->xml__lang_ = "en";
  tModel.overviewDoc->overviewURL = "http://.../my.wsdl#bindingName";

  // Omit identifier bag
  tModel.identifierBag = NULL;

  // Add a category with a WSDL-specific keyedReference
  tModel.categoryBag = soap_new_uddi2__categoryBag(soap, -1);
  tModel.categoryBag->soap_default(soap);
  tModel.categoryBag->keyedReference.push_back(soap_new_uddi2__keyedReference(soap, -1));
  tModel.categoryBag->keyedReference[0]->tModelKey = "...";
  tModel.categoryBag->keyedReference[0]->keyName = "uddi-org:types";
  tModel.categoryBag->keyedReference[0]->keyValue = "wsdlSpec";

  tModel.authorizedName = "...";
  tModel.operator_ = "...";
  tModel.tModelKey = "...";

  // Save the tModel
  uddi2__save_USCOREtModel save_tModel(soap, tModel);
  uddi2__tModelDetail *tModelDetail = save_tModel.send(server, authInfo);

  // Step 3: Create a new service to be published
  uddi2__businessService service;
  service.soap_default(soap);

  // Service name is the tModel name (XMethods)
  service.name.push_back(tModel.name);

  // Add two description elements to the service
  service.description.push_back(description + 4);
  service.description.push_back(description + 5);

  // Create binding template
  uddi2__bindingTemplate bindingTemplate;
  bindingTemplate.soap_default(soap);
  bindingTemplate.tModelInstanceDetails = soap_new_uddi2__tModelInstanceDetails(soap, -1);
  bindingTemplate.tModelInstanceDetails->tModelInstanceInfo.push_back(soap_new_uddi2__tModelInstanceInfo(soap, -1));
  bindingTemplate.tModelInstanceDetails->tModelInstanceInfo[0]->instanceDetails = NULL;
  bindingTemplate.tModelInstanceDetails->tModelInstanceInfo[0]->tModelKey = tModel.tModelKey;
  bindingTemplate.accessPoint = soap_new_uddi2__accessPoint(soap, -1);
  bindingTemplate.accessPoint->__item = "...";
  bindingTemplate.accessPoint->URLType = uddi2__URLType__http;
  bindingTemplate.hostingRedirector = NULL;
  bindingTemplate.serviceKey = "...";
  bindingTemplate.bindingKey = "...";

  // Add binding Template to service
  service.bindingTemplates = soap_new_uddi2__bindingTemplates(soap, -1);
  service.bindingTemplates->soap_default(soap);
  service.bindingTemplates->bindingTemplate.push_back(&bindingTemplate);

  service.categoryBag = NULL;
  service.serviceKey = "...";
  service.businessKey = "...";

  // Save the service
  uddi2__save_USCOREservice save_service(soap, service);
  uddi2__serviceDetail *serviceDetail = save_service.send(server, authInfo);

  // Step 4: Discard authorization token
  uddi2__discard_USCOREauthToken discard_authToken(soap, authInfo);
  uddi2__dispositionReport *dispositionReport = discard_authToken.send(server);

  // Remove deserialized objects
  soap_destroy(soap);

  // Remove temporary data
  soap_end(soap);

  // Detach and free context
  soap_done(soap);
  free(soap);

  return 0;
}
