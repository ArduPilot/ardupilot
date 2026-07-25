/*
        service.cpp

        Generate gSOAP service structures from WSDL.

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2000-2021, Robert van Engelen, Genivia Inc. All Rights Reserved.
This part of the software is released under one of the following licenses:
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

#include "types.h"
#include "service.h"

#include <algorithm>

#define URI_CHAR(c) ((c) > 32 && (c) != '"' && (c) != '<' && (c) != '>' && (c) != '\\' && (c) != '^' && (c) != '`' && (c) != '|' && (c) <= '~')

static const char *urienc(struct soap*, const char*);
static bool imported(const char *tag);
static void comment(const char *start, const char *middle, const char *end, const char *text);
static void page(const char *page, const char *title, const char *text);
static void section(const char *section, const char *title, const char *text);
static void banner(const char*, const char* = NULL);
static void ident();
static void gen_policy(Service&, const std::vector<const wsp__Policy*>&, const char*, Types&);
static void gen_policy_enablers(const Service&);
static void gen_plnk(const Service& service);
static void gen_vprop(const wsdl__definitions& definitions, Types&);

////////////////////////////////////////////////////////////////////////////////
//
//      Definitions methods
//
////////////////////////////////////////////////////////////////////////////////

Definitions::Definitions()
{ }

void Definitions::collect(const wsdl__definitions& definitions)
{
  // Collect information: analyze WSDL definitions and imported definitions
  analyze(definitions);
  for (std::vector<wsdl__import>::const_iterator import = definitions.import.begin(); import != definitions.import.end(); ++import)
    if ((*import).definitionsPtr())
      analyze(*(*import).definitionsPtr());
}

void Definitions::analyze(const wsdl__definitions& definitions)
{
  // Analyze WADL application linked within empty WSDL definitions
  analyze_application(definitions);
  // Analyze WSDL and build Service information
  binding_count = 0;
  // Determine number of relevant service bindings (service prefix option only)
  for (std::vector<wsdl__binding>::const_iterator i = definitions.binding.begin(); i != definitions.binding.end(); ++i)
  {
    for (std::vector<wsdl__ext_operation>::const_iterator j = (*i).operation.begin(); j != (*i).operation.end(); ++j)
    {
      if ((*j).operationPtr())
      {
        binding_count++;
        break;
      }
    }
  }
  if (vflag && definitions.name)
    fprintf(stderr, "\nAnalyzing \"%s\" which has %d service binding(s)\n", definitions.name, binding_count);
  if (binding_count == 0 && definitions.import.empty() && definitions.name && (!definitions.portType.empty() || !definitions.interface_.empty()))
    fprintf(stderr, "\nWarning: WSDL \"%s\" has no bindings to implement operations\n", definitions.name);
  else if (binding_count > 1 && !service_prefix)
  {
    // This puts all operations under a single binding
    fprintf(stderr, "\nWarning: %d service bindings found, but collected as one service (use option -Nname to produce a separate service for each binding)\n", binding_count);
  }
  // Analyze and collect service data
  for (std::vector<wsdl__binding>::const_iterator binding = definitions.binding.begin(); binding != definitions.binding.end(); ++binding)
  {
    // /definitions/binding/documentation
    const char *binding_documentation = (*binding).documentation;
    // /definitions/binding/soap:binding
    soap__binding *soap__binding_ = (*binding).soap__binding_;
    // /definitions/binding/soap:binding/@transport
    const char *soap__binding_transport = NULL;
    int version = 0;
    if (soap__binding_)
      soap__binding_transport = soap__binding_->transport;
    else if ((*binding).wsoap__version && strlen((*binding).wsoap__version) > 2)
      version = (*binding).wsoap__version[2]-'0'; // WSDL 2.0
    else if (!(*binding).http__binding_)
      version = 2; // assume WSDL 2.0
    if (version == 2)
      soap12 = true;
    // /definitions/binding/@soap:protocol
    if ((*binding).wsoap__protocol)
      soap__binding_transport = (*binding).wsoap__protocol;
    // /definitions/binding/@name
    const char *binding_name = "";
    if ((*binding).name)
      binding_name = (*binding).name;
    else if ((*binding).portTypePtr() && (*binding).portTypePtr()->name)
    {
      size_t l = strlen((*binding).portTypePtr()->name);
      char *s = (char*)soap_malloc(definitions.soap, l + 8);
      if (s)
      {
        soap_strcpy(s, l + 8, (*binding).portTypePtr()->name);
        soap_strcpy(s + l, 8, "Binding");
        binding_name = s;
      }
    }
    if (!Rflag &&
        (!soap__binding_transport
         || (strcmp(soap__binding_transport+strlen(soap__binding_transport)-4, "http")
          && strcmp(soap__binding_transport+strlen(soap__binding_transport)-5, "HTTP/"))))
    {
      // -R not set: skip WSDL REST bindings
      if (!Wflag)
        fprintf(stderr, "\nWarning: ignoring RESTful binding \"%s\" because of non-SOAP transport \"%s\" (use option -R to enable REST)\n", binding_name, soap__binding_transport ? soap__binding_transport : "");
    }
    else
    {
      // /definitions/binding/@type
      // TODO: need to find the Policy of portType, though it is never used...?
      // const wsp__Policy *portType_policy = NULL;
      // /definitions/binding/wsp:Policy and wsp:PolicyReference
      /*
         const wsp__Policy *binding_policy = NULL;
         if ((*binding).wsp__Policy_)
         binding_policy = (*binding).wsp__Policy_;
         if ((*binding).wsp__PolicyReference_)
         binding_policy = (*binding).wsp__PolicyReference_->policyPtr();
       */
      // /definitions/binding/http:binding
      http__binding *http__binding_ = (*binding).http__binding_;
      const char *http__binding_verb = NULL;
      if (http__binding_)
        http__binding_verb = http__binding_->verb; // HTTP POST and GET
      else if ((*binding).whttp__methodDefault)
        http__binding_verb = (*binding).whttp__methodDefault; // HTTP POST and GET
      // /definitions/binding/soap:binding/@style
      soap__styleChoice soap__binding_style = document;
      if (soap__binding_ && soap__binding_->style)
        soap__binding_style = *soap__binding_->style;
      // /definitions/binding/operation*
      for (std::vector<wsdl__ext_operation>::const_iterator operation = (*binding).operation.begin(); operation != (*binding).operation.end(); ++operation)
      {
        // /definitions/portType/operation/ associated with /definitions/binding/operation
        wsdl__operation *wsdl__operation_ = (*operation).operationPtr();
        // /definitions/binding/operation/soap:operation
        soap__operation *soap__operation_ = (*operation).soap__operation_;
        // /definitions/binding/operation/soap:operation/@style
        soap__styleChoice soap__operation_style = soap__binding_style;
        if (soap__operation_ && soap__operation_->style)
          soap__operation_style = *soap__operation_->style;
        // /definitions/binding/@wsoap:mepDefault
        const char *soap__operation_mep = (*binding).wsoap__mepDefault;
        // /definitions/binding/operation/@wsoap:mep
        if ((*operation).wsoap__mep)
          soap__operation_mep = (*operation).wsoap__mep;
        // /definitions/binding/operation/@whttp:method
        const char *http_method = http__binding_verb;
        if ((*operation).whttp__method)
          http_method = (*operation).whttp__method;
        // /definitions/binding/operation/@wsoap:action
        const char *soap__operation_action = NULL;
        if ((*operation).wsoap__action)
          soap__operation_action = (*operation).wsoap__action;
        // /definitions/binding/operation/http:operation
        http__operation *http__operation_ = (*operation).http__operation_;
        // /definitions/binding/wsp:Policy and wsp:PolicyReference
        const wsp__Policy *ext_operation_policy = NULL;
        if ((*operation).wsp__Policy_)
          ext_operation_policy = (*operation).wsp__Policy_;
        if ((*operation).wsp__PolicyReference_)
          ext_operation_policy = (*operation).wsp__PolicyReference_->policyPtr();
        // /definitions/binding/operation/http:operation/@location
        const char *http__operation_location = NULL;
        if (http__operation_)
          http__operation_location = http__operation_->location;
        else if ((*operation).whttp__location)
          http__operation_location = (*operation).whttp__location;
        // /definitions/binding/operation/input and output
        wsdl__ext_ioput *ext_input, *ext_output;
        // /definitions/portType/operation
        if (wsdl__operation_ && wsdl__operation_->name)
        {
          bool reversed = false;
          wsdl__ioput *input = NULL;
          wsdl__ioput *output = NULL;
          // normalize input/output order for solicit-response operations
          if (wsdl__operation_->__union1 == SOAP_UNION_wsdl__union_ioput_input)
            input = wsdl__operation_->__ioput1.input;
          else if (wsdl__operation_->__union1 == SOAP_UNION_wsdl__union_ioput_output)
          {
            reversed = true;
            input = wsdl__operation_->__ioput1.output;
          }
          if (wsdl__operation_->__union2 == SOAP_UNION_wsdl__union_ioput_input)
          {
            reversed = true;
            output = wsdl__operation_->__ioput2.input;
          }
          else if (wsdl__operation_->__union2 == SOAP_UNION_wsdl__union_ioput_output)
            output = wsdl__operation_->__ioput2.output;
          else // one input only or one output only (or none)
          {
            reversed = false;
            input = NULL;
            output = NULL;
            if (wsdl__operation_->__union1 == SOAP_UNION_wsdl__union_ioput_input)
              input = wsdl__operation_->__ioput1.input;
            else if (wsdl__operation_->__union1 == SOAP_UNION_wsdl__union_ioput_output)
              output = wsdl__operation_->__ioput1.output;
          }
          if (!reversed)
          {
            ext_input = (*operation).input;
            ext_output = (*operation).output;
          }
          else
          {
            ext_input = (*operation).output;
            ext_output = (*operation).input;
          }
          if ((http_method && !strcmp(http_method, "GET"))
            || (wsdl__operation_->pattern && !strstr(wsdl__operation_->pattern, "in")))
            input = NULL;
          if (wsdl__operation_->pattern && !strstr(wsdl__operation_->pattern, "out"))
            output = NULL;
          if (wsdl__operation_->pattern) // WSDL 2.0
          {
            soap__operation_style = document;
            if (wsdl__operation_->style && !strcmp(wsdl__operation_->style, "http://www.w3.org/ns/wsdl/rpc"))
              soap__operation_style = rpc;
            else if ((*binding).portTypePtr() && (*binding).portTypePtr()->styleDefault && !strcmp( (*binding).portTypePtr()->styleDefault, "http://www.w3.org/ns/wsdl/rpc"))
              soap__operation_style = rpc;
          }
          // /definitions/binding/wsp:Policy and wsp:PolicyReference
          const wsp__Policy *operation_policy = NULL;
          if (wsdl__operation_->wsp__Policy_)
            operation_policy = wsdl__operation_->wsp__Policy_;
          if (wsdl__operation_->wsp__PolicyReference_)
            operation_policy = wsdl__operation_->wsp__PolicyReference_->policyPtr();
          if (!Rflag && (http__operation_ || http__operation_location))
          {
            // skip WSDL REST HTTP operations
            if (!Wflag)
              fprintf(stderr, "\nWarning: ignoring RESTful operation \"%s\" in binding \"%s\" (use option -R to enable REST)\n", wsdl__operation_->name, binding_name);
          }
          else if (input)
          {
            soap__body *input_body = NULL;
            mime__mimeXml *input_mime = NULL;
            mime__content *input_mime_content = NULL;
            if (!reversed && ext_input)
            {
              input_body = ext_input->soap__body_;
              input_mime = ext_input->mime__mimeXml_;
              input_mime_content = ext_input->mime__content_;
              if (ext_input->mime__multipartRelated_)
              {
                for (std::vector<mime__part>::const_iterator part = ext_input->mime__multipartRelated_->part.begin(); part != ext_input->mime__multipartRelated_->part.end(); ++part)
                  if ((*part).soap__body_)
                  {
                    input_body = (*part).soap__body_;
                    break;
                  }
              }
            }
            else if (ext_output)
            {
              input_body = ext_output->soap__body_;
              input_mime = ext_output->mime__mimeXml_;
              input_mime_content = ext_output->mime__content_;
              if (ext_output->mime__multipartRelated_)
              {
                for (std::vector<mime__part>::const_iterator part = ext_output->mime__multipartRelated_->part.begin(); part != ext_output->mime__multipartRelated_->part.end(); ++part)
                  if ((*part).soap__body_)
                  {
                    input_body = (*part).soap__body_;
                    break;
                  }
              }
            }
            if (wsdl__operation_->style && !strcmp(wsdl__operation_->style, "http://www.w3.org/ns/wsdl/style/iri"))
            {
              input_mime_content = soap_new_mime__content(definitions.soap);
              input_mime_content->soap_default(definitions.soap);
              input_mime_content->type = (char*)"application/x-www-form-urlencoded";
            }
            // MUST have an input binding, otherwise can't generate a service operation
            if (input_body || input_mime || input_mime_content || input->element)
            {
              char *URI;
              if (input_body && soap__operation_style == rpc)
                URI = input_body->namespace_;
              else if ((binding_count == 1 || !service_prefix) && definitions.targetNamespace && *definitions.targetNamespace)
                URI = definitions.targetNamespace;
              else if ((binding_count == 1 || !service_prefix) && definitions.name && *definitions.name)
                URI = definitions.name;
              else
              {
                // multiple service bindings are used, each needs a unique new URI
                size_t l = strlen(definitions.targetNamespace) + strlen(binding_name);
                URI = (char*)soap_malloc(definitions.soap, l + 2);
                if (URI)
                {
                  soap_strcpy(URI, l + 2, definitions.targetNamespace);
                  size_t n = strlen(URI);
                  if (n > 0 && URI[n-1] != '/')
                  {
                    soap_strcpy(URI + n, l + 2 - n, "/");
                    ++n;
                  }
                  soap_strcpy(URI + n, l + 2 - n, binding_name);
                }
              }
              const char *prefix = types.nsprefix(service_prefix, URI);
              const char *name = types.aname(NULL, NULL, binding_name); // name of service is binding name
              Service *service = services[prefix];
              if (!service)
              {
                service = services[prefix] = new Service();
                service->prefix = prefix;
                service->URI = urienc(definitions.soap, URI);
                service->name = name;
                service->transport = soap__binding_transport;
                if ((*binding).portTypePtr() && (*binding).portTypePtr()->name)
                  service->type = types.aname(NULL, NULL, (*binding).portTypePtr()->name);
                else
                  service->type = NULL;
                // collect faults (TODO: this is not used anywhere)
                for (std::vector<wsdl__ext_fault>::const_iterator fault = (*binding).fault.begin(); fault != (*binding).fault.end(); ++fault)
                {
                  Message *f = analyze_fault(definitions, service, *fault);
                  if (f)
                    service->fault[f->name] = f;
                }
                // collect policies for the bindings
                for (std::vector<wsp__Policy>::const_iterator p = (*binding).wsp__Policy_.begin(); p != (*binding).wsp__Policy_.end(); ++p)
                  service->policy.push_back(&(*p));
                for (std::vector<wsp__PolicyReference>::const_iterator r = (*binding).wsp__PolicyReference_.begin(); r != (*binding).wsp__PolicyReference_.end(); ++r)
                  service->policy.push_back((*r).policyPtr());
                // collect policies for the service endpoints
                for (std::vector<wsdl__service>::const_iterator s = definitions.service.begin(); s != definitions.service.end(); ++s)
                {
                  for (std::vector<wsp__Policy>::const_iterator p = (*s).wsp__Policy_.begin(); p != (*s).wsp__Policy_.end(); ++p)
                    service->policy.push_back(&(*p));
                  for (std::vector<wsp__PolicyReference>::const_iterator r = (*s).wsp__PolicyReference_.begin(); r != (*s).wsp__PolicyReference_.end(); ++r)
                    service->policy.push_back((*r).policyPtr());
                }
                // collect BPEL 2.0 partner link roles
                for (std::vector<plnk__tPartnerLinkType>::const_iterator p = definitions.plnk__partnerLinkType.begin(); p != definitions.plnk__partnerLinkType.end(); ++p)
                {
                  for (std::vector<plnk__tRole>::const_iterator r = (*p).role.begin(); r != (*p).role.end(); ++r)
                  {
                    if ((binding_count > 1 && !service_prefix) || (*r).portTypePtr() == (*binding).portTypePtr())
                      service->role.push_back(&(*r));
                  }
                }  
              }
              for (std::vector<wsdl__service>::const_iterator s = definitions.service.begin(); s != definitions.service.end(); ++s)
              {
                for (std::vector<wsdl__port>::const_iterator port = (*s).port.begin(); port != (*s).port.end(); ++port)
                {
                  if ((*port).bindingPtr() == &(*binding))
                  {
                    if ((*port).soap__address_ && (*port).soap__address_->location)
                      service->location.insert(urienc(definitions.soap, (*port).soap__address_->location));
                    if ((*port).wsa__EndpointReference && (*port).wsa__EndpointReference->Address)
                      service->location.insert(urienc(definitions.soap, (*port).wsa__EndpointReference->Address));
                    if ((*port).http__address_ && (*port).http__address_->location)
                      service->location.insert(urienc(definitions.soap, (*port).http__address_->location));
                    if ((*port).wsaw__UsingAddressing || (*binding).wsaw__UsingAddressing)
                      service->add_import("wsa5.h");
                    // collect service documentation
                    if ((*s).documentation)
                      service->service_documentation[(*service).name] = (*s).documentation;
                    if ((*port).documentation && (*port).name)
                      service->port_documentation[(*port).name] = (*port).documentation;
                    if (binding_documentation)
                      service->binding_documentation[binding_name] = binding_documentation;
                    // collect policies for the service and endpoints
                    if ((*port).wsp__Policy_)
                      service->policy.push_back((*port).wsp__Policy_);
                    if ((*port).wsp__PolicyReference_ && (*port).wsp__PolicyReference_->policyPtr())
                      service->policy.push_back((*port).wsp__PolicyReference_->policyPtr());
                  }
                }
                for (std::vector<wsdl__port>::const_iterator endpoint = (*s).endpoint.begin(); endpoint != (*s).endpoint.end(); ++endpoint)
                {
                  if ((*endpoint).bindingPtr() == &(*binding))
                  {
                    if ((*endpoint).address)
                      service->location.insert(urienc(definitions.soap, (*endpoint).address));
                    if ((*endpoint).wsa__EndpointReference && (*endpoint).wsa__EndpointReference->Address)
                      service->location.insert(urienc(definitions.soap, (*endpoint).wsa__EndpointReference->Address));
                    if ((*endpoint).http__address_ && (*endpoint).http__address_->location)
                      service->location.insert(urienc(definitions.soap, (*endpoint).http__address_->location));
                    // TODO: locations need auth
                    // service->auth_scheme = (*endpoint).whttp__authenticationScheme;
                    // service->auth_realm = (*endpoint).whttp__authenticationRealm;
                    // collect service documentation
                    if ((*s).documentation)
                      service->service_documentation[(*service).name] = (*s).documentation;
                    if ((*endpoint).documentation && (*endpoint).name)
                      service->port_documentation[(*endpoint).name] = (*endpoint).documentation;
                    if (binding_documentation)
                      service->binding_documentation[binding_name] = binding_documentation;
                    // collect policies for the service and endpoints
                    if ((*endpoint).wsp__Policy_)
                      service->policy.push_back((*endpoint).wsp__Policy_);
                    if ((*endpoint).wsp__PolicyReference_ && (*endpoint).wsp__PolicyReference_->policyPtr())
                      service->policy.push_back((*endpoint).wsp__PolicyReference_->policyPtr());
                  }
                }
              }
              Operation *op = new Operation();
              op->operation = wsdl__operation_;
              op->name = types.aname(NULL, NULL, wsdl__operation_->name);
              op->prefix = prefix;
              op->URI = urienc(definitions.soap, URI);
              op->style = soap__operation_style;
              op->mep = soap__operation_mep;
              op->is_rest = false;
              if (soap__binding_transport
                  && (!strcmp(soap__binding_transport+strlen(soap__binding_transport)-4, "http")
                    || !strcmp(soap__binding_transport+strlen(soap__binding_transport)-5, "HTTP/")))
              {
                if ((op->mep && strstr(op->mep, "soap-response"))
                    || (http_method && !strcmp(http_method, "GET")))
                  op->protocol = "SOAP-GET";
                else if (version == 1)
                  op->protocol = "SOAP1.1";
                else if (version == 2)
                  op->protocol = "SOAP1.2";
                else
                  op->protocol = "SOAP";
              }
              else
              {
                op->is_rest = true;
                if (http_method)
                  op->protocol = http_method;
                else
                  op->protocol = "HTTP";
              }
              op->documentation = wsdl__operation_->documentation;
              op->operation_documentation = (*operation).documentation;
              op->parameterOrder = wsdl__operation_->parameterOrder;
              if (http__operation_location)
                op->action = http__operation_location; // TODO: for now, store HTTP location in action
              else
              {
                op->action = soap__operation_action;
                if ((*operation).soap__operation_)
                {
                  if ((*operation).soap__operation_->soapActionRequired)
                    op->action = (*operation).soap__operation_->soapAction;
                }
                else if (version != 2)
                  op->action = "";
              }
              if (operation_policy)
                op->policy.push_back(operation_policy);
              if (ext_operation_policy)
                op->policy.push_back(ext_operation_policy);
              op->input = new Message();
              op->input->name = wsdl__operation_->name;
              if (input_body && soap__operation_style == rpc && !input_body->namespace_)
              {
                op->input->URI = "";
                fprintf(stderr, "\nError: no soap:body namespace attribute\n");
              }
              else if (input_body)
                op->input->URI = urienc(definitions.soap, input_body->namespace_);
              else
                op->input->URI = service->URI;
              op->input->style = soap__operation_style;
              if (input_body)
              {
                op->input->use = input_body->use;
                op->input->encodingStyle = input_body->encodingStyle;
              }
              if (input->wsa__Action)
                op->input->action = input->wsa__Action;
              else if (input->wsam__Action)
                op->input->action = input->wsam__Action;
              else if (input->wsaw__Action)
                op->input->action = input->wsaw__Action;
              else if (op->action)
                op->input->action = op->action;
              else if (definitions.targetNamespace && (*binding).portTypePtr() && (*binding).portTypePtr()->name)
              {
                const char *name = input->name ? input->name : op->name;
                size_t l = strlen(definitions.targetNamespace) + strlen((*binding).portTypePtr()->name) + strlen(name);
                char *tmp = (char*)soap_malloc(definitions.soap, l + 3);
                if (tmp)
                {
                  (SOAP_SNPRINTF(tmp, l + 3, l + 2), "%s/%s/%s", definitions.targetNamespace, (*binding).portTypePtr()->name, name);
                  op->input->action = tmp;
                }
              }
              op->input->message = input->messagePtr();
              op->input->element = input->elementPtr();
              op->input->part = NULL;
              op->input->mustUnderstand = false;
              op->input->multipartRelated = NULL;
              op->input->content = input_mime_content;
              op->input->body_parts = NULL;
              op->input->layout = NULL;
              op->input->ext_documentation = NULL;
              if (ext_input)
              {
                op->input->multipartRelated = ext_input->mime__multipartRelated_;
                if (ext_input->mime__multipartRelated_ && !ext_input->mime__multipartRelated_->part.empty())
                  op->input->header = ext_input->mime__multipartRelated_->part.front().soap__header_;
                else if (!ext_input->soap__header_.empty())
                  op->input->header = ext_input->soap__header_;
                else if (!ext_input->wsoap__header_.empty())
                  op->input->wheader = ext_input->wsoap__header_;
                if (ext_input->mime__multipartRelated_ && !ext_input->mime__multipartRelated_->part.empty() && ext_input->mime__multipartRelated_->part.front().soap__body_)
                  op->input->body_parts = ext_input->mime__multipartRelated_->part.front().soap__body_->parts;
                else if (input_body)
                  op->input->body_parts = input_body->parts;
                if (ext_input->dime__message_)
                  op->input->layout = ext_input->dime__message_->layout;
                else
                  op->input->layout = NULL;
                op->input->ext_documentation = ext_input->documentation;
              }
              op->input->documentation = input->documentation;
              // collect input message policies
              if (op->input->message)
              {
                for (std::vector<wsp__Policy>::const_iterator p = op->input->message->wsp__Policy_.begin(); p != op->input->message->wsp__Policy_.end(); ++p)
                  op->input->policy.push_back(&(*p));
                for (std::vector<wsp__PolicyReference>::const_iterator r = op->input->message->wsp__PolicyReference_.begin(); r != op->input->message->wsp__PolicyReference_.end(); ++r)
                  op->input->policy.push_back((*r).policyPtr());
              }
              if (input->wsp__Policy_)
                op->input->policy.push_back(input->wsp__Policy_);
              if (input->wsp__PolicyReference_ && input->wsp__PolicyReference_->policyPtr())
                op->input->policy.push_back(input->wsp__PolicyReference_->policyPtr());
              if (ext_input)
              {
                if (ext_input->wsp__Policy_)
                  op->input->policy.push_back(ext_input->wsp__Policy_);
                if (ext_input->wsp__PolicyReference_ && ext_input->wsp__PolicyReference_->policyPtr())
                  op->input->policy.push_back(ext_input->wsp__PolicyReference_->policyPtr());
              }
              if (soap__operation_style == document)
                op->input_name = types.oname("__", op->URI, op->input->name);
              else
                op->input_name = types.oname(NULL, op->input->URI, op->input->name);
              if (output)
              {
                soap__body *output_body = NULL;
                mime__mimeXml *output_mime = NULL;
                mime__content *output_mime_content = NULL;
                if (ext_output)
                {
                  output_body = ext_output->soap__body_;
                  output_mime = ext_output->mime__mimeXml_;
                  output_mime_content = ext_output->mime__content_;
                  if (ext_output->mime__multipartRelated_)
                  {
                    for (std::vector<mime__part>::const_iterator part = ext_output->mime__multipartRelated_->part.begin(); part != ext_output->mime__multipartRelated_->part.end(); ++part)
                      if ((*part).soap__body_)
                      {
                        output_body = (*part).soap__body_;
                        break;
                      }
                  }
                }
                if (ext_output && ext_output->mime__content_)
                {
                  op->output = new Message();
                  op->output->name = NULL;
                  op->output->URI = NULL;
                  op->output->style = soap__operation_style;
                  op->output->use = literal;
                  op->output->encodingStyle = NULL;
                  op->output->action = NULL;
                  op->output->body_parts = NULL;
                  op->output->part = NULL;
                  op->output->mustUnderstand = false;
                  op->output->multipartRelated = NULL;
                  op->output->content = output_mime_content;
                  op->output->message = output->messagePtr();
                  op->output->element = output->elementPtr();
                  op->output->layout = NULL;
                }
                else if (output_body || output_mime || output_mime_content || output->element)
                {
                  op->output = new Message();
                  op->output->name = wsdl__operation_->name; // RPC uses operation/@name with suffix 'Response' as set below
                  op->output->style = soap__operation_style;
                  if (output_body)
                  {
                    op->output->use = output_body->use;
                    // the code below is a hack around the RPC encoded response message element tag mismatch with Axis:
                    if (!output_body->namespace_ || output_body->use == encoded)
                      op->output->URI = op->input->URI; // encoded seems (?) to require the request's namespace
                    else
                      op->output->URI = urienc(definitions.soap, output_body->namespace_);
                    op->output->encodingStyle = output_body->encodingStyle;
                  }
                  else
                    op->output->URI = service->URI;
                  if (output->wsa__Action)
                    op->output->action = output->wsa__Action;
                  else if (output->wsam__Action)
                    op->output->action = output->wsam__Action;
                  else if (output->wsaw__Action)
                    op->output->action = output->wsaw__Action;
                  else if (http__operation_location)
                    op->output->action = NULL;
                  else if (op->action)
                  {
                    size_t l = strlen(op->action);
                    char *tmp = (char*)soap_malloc(definitions.soap, l + 9);
                    if (tmp)
                    {
                      (SOAP_SNPRINTF(tmp, l + 9, l + 8), "%sResponse", op->action);
                      op->output->action = tmp;
                    }
                  }
                  else if (definitions.targetNamespace && (*binding).portTypePtr() && (*binding).portTypePtr()->name)
                  {
                    const char *name = output->name ? output->name : op->name;
                    size_t l = strlen(definitions.targetNamespace) + strlen((*binding).portTypePtr()->name) + strlen(name);
                    char *tmp = (char*)soap_malloc(definitions.soap, l + 11);
                    if (tmp)
                    {
                      (SOAP_SNPRINTF(tmp, l + 11, l + 10), "%s/%s/%s%s", definitions.targetNamespace, (*binding).portTypePtr()->name, name, output->name ? "" : "Response");
                      op->output->action = tmp;
                    }
                  }
                  op->output->message = output->messagePtr();
                  op->output->element = output->elementPtr();
                  op->output->part = NULL;
                  op->output->content = output_mime_content;
                  op->output->body_parts = NULL;
                  op->output->layout = NULL;
                  op->output->ext_documentation = NULL;
                  op->output->mustUnderstand = false;
                }
                if (ext_output)
                {
                  op->output->multipartRelated = ext_output->mime__multipartRelated_;
                  if (ext_output->mime__multipartRelated_ && !ext_output->mime__multipartRelated_->part.empty())
                    op->output->header = ext_output->mime__multipartRelated_->part.front().soap__header_;
                  else if (!ext_output->soap__header_.empty())
                    op->output->header = ext_output->soap__header_;
                  else if (!ext_output->wsoap__header_.empty())
                    op->output->wheader = ext_output->wsoap__header_;
                  if (ext_output->mime__multipartRelated_ && !ext_output->mime__multipartRelated_->part.empty() && ext_output->mime__multipartRelated_->part.front().soap__body_)
                    op->output->body_parts = ext_output->mime__multipartRelated_->part.front().soap__body_->parts;
                  else if (output_body)
                    op->output->body_parts = output_body->parts;
                  if (ext_output->dime__message_)
                    op->output->layout = ext_output->dime__message_->layout;
                  else
                    op->output->layout = NULL;
                  op->output->ext_documentation = ext_output->documentation;
                }
                if (op->output->name)
                {
                  size_t l = strlen(op->output->name);
                  char *s = (char*)soap_malloc(definitions.soap, l + 9);
                  if (s)
                  {
                    (SOAP_SNPRINTF(s, l + 9, l + 8), "%sResponse", op->output->name);
                    if (soap__operation_style == document)
                      op->output_name = types.oname("__", op->URI, s);
                    else
                      op->output_name = types.oname(NULL, op->output->URI, s);
                  }
                }
                op->output->documentation = output->documentation;
                // collect output message policies
                if (op->output->message)
                {
                  for (std::vector<wsp__Policy>::const_iterator p = op->output->message->wsp__Policy_.begin(); p != op->output->message->wsp__Policy_.end(); ++p)
                    op->output->policy.push_back(&(*p));
                  for (std::vector<wsp__PolicyReference>::const_iterator r = op->output->message->wsp__PolicyReference_.begin(); r != op->output->message->wsp__PolicyReference_.end(); ++r)
                    op->output->policy.push_back((*r).policyPtr());
                }
                if (output->wsp__Policy_)
                  op->output->policy.push_back(output->wsp__Policy_);
                if (output->wsp__PolicyReference_ && output->wsp__PolicyReference_->policyPtr())
                  op->output->policy.push_back(output->wsp__PolicyReference_->policyPtr());
                if (ext_output)
                {
                  if (ext_output->wsp__Policy_)
                    op->output->policy.push_back(ext_output->wsp__Policy_);
                  if (ext_output->wsp__PolicyReference_ && ext_output->wsp__PolicyReference_->policyPtr())
                    op->output->policy.push_back(ext_output->wsp__PolicyReference_->policyPtr());
                }
              }
              else
              {
                op->output_name = NULL;
                op->output = NULL;
              }
              analyze_headers(definitions, service, ext_input, ext_output);
              analyze_faults(definitions, service, op, operation);
              service->operation.push_back(op);
            }
            else
              fprintf(stderr, "\nError: no wsdl:definitions/binding/operation/input\n");
          }
          else if (output)
          {
            // This part is similar to the previous clause, limited to one-way output operations
            soap__body *output_body = NULL;
            mime__mimeXml *output_mime = NULL;
            mime__content *output_mime_content = NULL;
            if (!reversed && ext_output)
            {
              output_body = ext_output->soap__body_;
              output_mime = ext_output->mime__mimeXml_;
              output_mime_content = ext_output->mime__content_;
              if (ext_output->mime__multipartRelated_)
              {
                for (std::vector<mime__part>::const_iterator part = ext_output->mime__multipartRelated_->part.begin(); part != ext_output->mime__multipartRelated_->part.end(); ++part)
                  if ((*part).soap__body_)
                  {
                    output_body = (*part).soap__body_;
                    break;
                  }
              }
            }
            else if (ext_input)
            {
              output_body = ext_input->soap__body_;
              output_mime = ext_input->mime__mimeXml_;
              output_mime_content = ext_input->mime__content_;
              if (ext_input->mime__multipartRelated_)
              {
                for (std::vector<mime__part>::const_iterator part = ext_input->mime__multipartRelated_->part.begin(); part != ext_input->mime__multipartRelated_->part.end(); ++part)
                  if ((*part).soap__body_)
                  {
                    output_body = (*part).soap__body_;
                    break;
                  }
              }
            }
            if (wsdl__operation_->style && !strcmp(wsdl__operation_->style, "http://www.w3.org/ns/wsdl/style/iri"))
            {
              output_mime_content = soap_new_mime__content(definitions.soap);
              output_mime_content->soap_default(definitions.soap);
              output_mime_content->type = (char*)"application/x-www-form-urlencoded";
            }
            if (output_body || output_mime || output_mime_content || output->element)
            {
              char *URI;
              if (output_body && soap__operation_style == rpc)
                URI = output_body->namespace_;
              else if ((binding_count == 1 || !service_prefix) && definitions.targetNamespace && *definitions.targetNamespace)
                URI = definitions.targetNamespace;
              else if ((binding_count == 1 || !service_prefix) && definitions.name && *definitions.name)
                URI = definitions.name;
              else
              {
                // multiple service bidings are used, each needs a unique new URI
                size_t l = strlen(definitions.targetNamespace) + strlen(binding_name);
                URI = (char*)soap_malloc(definitions.soap, l + 2);
                if (URI)
                {
                  soap_strcpy(URI, l + 2, definitions.targetNamespace);
                  size_t n = strlen(URI);
                  if (n > 0 && URI[n-1] != '/')
                  {
                    soap_strcpy(URI + n, l + 2 - n, "/");
                    ++n;
                  }
                  soap_strcpy(URI + n, l + 2 - n, binding_name);
                }
              }
              const char *prefix = types.nsprefix(service_prefix, URI);
              const char *name = types.aname(NULL, NULL, binding_name); // name of service is binding name
              Service *service = services[prefix];
              if (!service)
              {
                service = services[prefix] = new Service();
                service->prefix = prefix;
                service->URI = urienc(definitions.soap, URI);
                service->name = name;
                service->transport = soap__binding_transport;
                if ((*binding).portTypePtr() && (*binding).portTypePtr()->name)
                  service->type = types.aname(NULL, NULL, (*binding).portTypePtr()->name);
                else
                  service->type = NULL;
                // collect faults (TODO: this is not used anywhere)
                for (std::vector<wsdl__ext_fault>::const_iterator fault = (*binding).fault.begin(); fault != (*binding).fault.end(); ++fault)
                {
                  Message *f = analyze_fault(definitions, service, *fault);
                  if (f)
                    service->fault[f->name] = f;
                }
                // collect policies for the bindings
                for (std::vector<wsp__Policy>::const_iterator p = (*binding).wsp__Policy_.begin(); p != (*binding).wsp__Policy_.end(); ++p)
                  service->policy.push_back(&(*p));
                for (std::vector<wsp__PolicyReference>::const_iterator r = (*binding).wsp__PolicyReference_.begin(); r != (*binding).wsp__PolicyReference_.end(); ++r)
                  service->policy.push_back((*r).policyPtr());
                // collect policies for the service endpoints
                for (std::vector<wsdl__service>::const_iterator s = definitions.service.begin(); s != definitions.service.end(); ++s)
                {
                  for (std::vector<wsp__Policy>::const_iterator p = (*s).wsp__Policy_.begin(); p != (*s).wsp__Policy_.end(); ++p)
                    service->policy.push_back(&(*p));
                  for (std::vector<wsp__PolicyReference>::const_iterator r = (*s).wsp__PolicyReference_.begin(); r != (*s).wsp__PolicyReference_.end(); ++r)
                    service->policy.push_back((*r).policyPtr());
                }
                // collect BPEL 2.0 partner link roles
                for (std::vector<plnk__tPartnerLinkType>::const_iterator p = definitions.plnk__partnerLinkType.begin(); p != definitions.plnk__partnerLinkType.end(); ++p)
                {
                  for (std::vector<plnk__tRole>::const_iterator r = (*p).role.begin(); r != (*p).role.end(); ++r)
                  {
                    if ((binding_count > 1 && !service_prefix) || (*r).portTypePtr() == (*binding).portTypePtr())
                      service->role.push_back(&(*r));
                  }
                }  
              }
              for (std::vector<wsdl__service>::const_iterator s = definitions.service.begin(); s != definitions.service.end(); ++s)
              {
                for (std::vector<wsdl__port>::const_iterator port = (*s).port.begin(); port != (*s).port.end(); ++port)
                {
                  if ((*port).bindingPtr() == &(*binding))
                  {
                    if ((*port).soap__address_ && (*port).soap__address_->location)
                      service->location.insert(urienc(definitions.soap, (*port).soap__address_->location));
                    else if ((*port).wsa__EndpointReference && (*port).wsa__EndpointReference->Address)
                      service->location.insert(urienc(definitions.soap, (*port).wsa__EndpointReference->Address));
                    if ((*port).wsaw__UsingAddressing)
                      service->add_import("wsa5.h");
                    // TODO: HTTP address for HTTP operations
                    // if ((*port).http__address_)
                    // http__address_location = http__address_->location;
                    // collect service documentation
                    if ((*s).documentation)
                      service->service_documentation[(*service).name] = (*s).documentation;
                    if ((*port).documentation && (*port).name)
                      service->port_documentation[(*port).name] = (*port).documentation;
                    if (binding_documentation)
                      service->binding_documentation[binding_name] = binding_documentation;
                    // collect policies for the service and endpoints
                    if ((*port).wsp__Policy_)
                      service->policy.push_back((*port).wsp__Policy_);
                    if ((*port).wsp__PolicyReference_ && (*port).wsp__PolicyReference_->policyPtr())
                      service->policy.push_back((*port).wsp__PolicyReference_->policyPtr());
                  }
                }
              }
              Operation *op = new Operation();
              op->operation = wsdl__operation_;
              op->input_name = NULL;
              op->input = NULL;
              op->name = types.aname(NULL, NULL, wsdl__operation_->name);
              op->prefix = prefix;
              op->URI = urienc(definitions.soap, URI);
              op->style = soap__operation_style;
              op->mep = soap__operation_mep;
              op->is_rest = false;
              if (soap__binding_transport
                  && (!strcmp(soap__binding_transport+strlen(soap__binding_transport)-4, "http")
                    || !strcmp(soap__binding_transport+strlen(soap__binding_transport)-5, "HTTP/")))
              {
                if ((op->mep && strstr(op->mep, "soap-response"))
                    || (http_method && !strcmp(http_method, "GET")))
                  op->protocol = "SOAP-GET";
                else if (version == 1)
                  op->protocol = "SOAP1.1";
                else if (version == 2)
                  op->protocol = "SOAP1.2";
                else
                  op->protocol = "SOAP";
              }
              else
              {
                op->is_rest = true;
                if (http_method)
                  op->protocol = http_method;
                else
                  op->protocol = "HTTP";
              }
              op->documentation = wsdl__operation_->documentation;
              op->operation_documentation = (*operation).documentation;
              op->parameterOrder = wsdl__operation_->parameterOrder;
              if (http__operation_location)
              {
                op->action = http__operation_location; // TODO: for now, store HTTP location in action
              }
              else
              {
                op->action = soap__operation_action;
                if ((*operation).soap__operation_)
                {
                  if ((*operation).soap__operation_->soapActionRequired)
                    op->action = (*operation).soap__operation_->soapAction;
                }
                else if (version != 2)
                  op->action = "";
              }
              if (operation_policy)
                op->policy.push_back(operation_policy);
              if (ext_operation_policy)
                op->policy.push_back(ext_operation_policy);
              op->output = new Message(); // one-way output operation
              op->output->name = wsdl__operation_->name; // RPC uses operation/@name
              if (output_body && soap__operation_style == rpc && !output_body->namespace_)
              {
                op->output->URI = "";
                fprintf(stderr, "\nError: no soap:body namespace attribute\n");
              }
              else if (output_body)
                op->output->URI = urienc(definitions.soap, output_body->namespace_);
              else
                op->output->URI = service->URI;
              op->output->style = soap__operation_style;
              if (output_body)
              {
                op->output->use = output_body->use;
                op->output->encodingStyle = output_body->encodingStyle;
              }
              if (output->wsa__Action)
                op->output->action = output->wsa__Action;
              else if (output->wsam__Action)
                op->output->action = output->wsam__Action;
              else if (output->wsaw__Action)
                op->output->action = output->wsaw__Action;
              else if (op->action)
                op->output->action = op->action;
              else if (definitions.targetNamespace && (*binding).portTypePtr() && (*binding).portTypePtr()->name)
              {
                const char *name = output->name ? output->name : op->name;
                size_t l = strlen(definitions.targetNamespace) + strlen((*binding).portTypePtr()->name) + strlen(name);
                char *tmp = (char*)soap_malloc(definitions.soap, l + 3);
                if (tmp)
                {
                  (SOAP_SNPRINTF(tmp, l + 3, l + 2), "%s/%s/%s", definitions.targetNamespace, (*binding).portTypePtr()->name, name);
                  op->output->action = tmp;
                }
              }
              op->output->message = output->messagePtr();
              op->output->element = output->elementPtr();
              op->output->part = NULL;
              op->output->mustUnderstand = false;
              op->output->multipartRelated = NULL;
              op->output->content = output_mime_content;
              op->output->body_parts = NULL;
              op->output->layout = NULL;
              op->output->ext_documentation = NULL;
              if (ext_output)
              {
                op->output->multipartRelated = ext_output->mime__multipartRelated_;
                if (ext_output->mime__multipartRelated_ && !ext_output->mime__multipartRelated_->part.empty())
                  op->output->header = ext_output->mime__multipartRelated_->part.front().soap__header_;
                else if (!ext_output->soap__header_.empty())
                  op->output->header = ext_output->soap__header_;
                else if (!ext_output->wsoap__header_.empty())
                  op->output->wheader = ext_output->wsoap__header_;
                if (ext_output->mime__multipartRelated_ && !ext_output->mime__multipartRelated_->part.empty() && ext_output->mime__multipartRelated_->part.front().soap__body_)
                  op->output->body_parts = ext_output->mime__multipartRelated_->part.front().soap__body_->parts;
                else if (output_body)
                  op->output->body_parts = output_body->parts;
                if (ext_output->dime__message_)
                  op->output->layout = ext_output->dime__message_->layout;
                else
                  op->output->layout = NULL;
                op->output->ext_documentation = ext_output->documentation;
              }
              op->output->documentation = output->documentation;
              // collect output message policies
              if (op->output->message)
              {
                for (std::vector<wsp__Policy>::const_iterator p = op->output->message->wsp__Policy_.begin(); p != op->output->message->wsp__Policy_.end(); ++p)
                  op->output->policy.push_back(&(*p));
                for (std::vector<wsp__PolicyReference>::const_iterator r = op->output->message->wsp__PolicyReference_.begin(); r != op->output->message->wsp__PolicyReference_.end(); ++r)
                  op->output->policy.push_back((*r).policyPtr());
              }
              if (output->wsp__Policy_)
                op->output->policy.push_back(output->wsp__Policy_);
              if (output->wsp__PolicyReference_ && output->wsp__PolicyReference_->policyPtr())
                op->output->policy.push_back(output->wsp__PolicyReference_->policyPtr());
              if (ext_output)
              {
                if (ext_output->wsp__Policy_)
                  op->output->policy.push_back(ext_output->wsp__Policy_);
                if (ext_output->wsp__PolicyReference_ && ext_output->wsp__PolicyReference_->policyPtr())
                  op->output->policy.push_back(ext_output->wsp__PolicyReference_->policyPtr());
              }
              if (soap__operation_style == document)
                op->input_name = types.oname("__", op->URI, op->output->name);
              else
                op->input_name = types.oname(NULL, op->output->URI, op->output->name);
              size_t l = strlen(op->output->name);
              char *s = (char*)soap_malloc(definitions.soap, l + 9);
              if (s)
              {
                (SOAP_SNPRINTF(s, l + 9, l + 8), "%sResponse", op->output->name);
                if (soap__operation_style == document)
                  op->output_name = types.oname("__", op->URI, s);
                else
                  op->output_name = types.oname(NULL, op->output->URI, s);
              }
              analyze_headers(definitions, service, ext_input, ext_output);
              analyze_faults(definitions, service, op, operation);
              service->operation.push_back(op);
            }
            else
              fprintf(stderr, "\nError: no wsdl:definitions/binding/operation/output\n");
          }
          else
            fprintf(stderr, "\nError: no wsdl:definitions/portType/operation/input and output\n");
        }
        else
          fprintf(stderr, "\nError: no wsdl:definitions/portType/operation\n");
      }
    }
  }
}

void Definitions::analyze_headers(const wsdl__definitions& definitions, Service *service, wsdl__ext_ioput *ext_input, wsdl__ext_ioput *ext_output)
{
  // collect input headers and headerfaults
  if (ext_input)
  {
    const std::vector<soap__header> *soap__header_ = NULL;
    // check if soap header is in mime:multipartRelated
    if (ext_input->mime__multipartRelated_)
    {
      for (std::vector<mime__part>::const_iterator part = ext_input->mime__multipartRelated_->part.begin(); part != ext_input->mime__multipartRelated_->part.end(); ++part)
      if (!(*part).soap__header_.empty())
      {
        soap__header_ = &(*part).soap__header_;
        break;
      }
    }
    if (!soap__header_)
      soap__header_ = &ext_input->soap__header_;
    for (std::vector<soap__header>::const_iterator header = soap__header_->begin(); header != soap__header_->end(); ++header)
    {
      Message *h = new Message();
      h->message = (*header).messagePtr();
      h->element = NULL;
      h->body_parts = NULL;
      h->part = (*header).partPtr();
      if ((*header).namespace_)
        h->URI = urienc(definitions.soap, (*header).namespace_);
      else
        h->URI = service->URI;
      if (h->part && h->part->element)
        h->name = types.aname(NULL, NULL, h->part->element);
      else if (h->URI && h->part && h->part->name && h->part->type)
        h->name = types.aname(NULL, h->URI, h->part->name);
      else
      {
        fprintf(stderr, "\nError in SOAP Header part definition: input part \"%s\" missing?\n", h->part && h->part->name ? h->part->name : "?");
        h->name = "";
      }
      h->encodingStyle = (*header).encodingStyle;
      h->style = document;      // irrelevant
      h->use = (*header).use;
      h->mustUnderstand = true;
      h->multipartRelated = NULL;
      h->content = NULL;
      h->layout = NULL;
      h->ext_documentation = NULL;      // TODO: may wanto to add document content
      h->documentation = NULL;          // TODO: may wanto to add document content
      service->header[h->name] = h;
      for (std::vector<soap__headerfault>::const_iterator headerfault = (*header).headerfault.begin(); headerfault != (*header).headerfault.end(); ++headerfault)
      {
        Message *hf = new Message();
        hf->message = (*headerfault).messagePtr();
        hf->element = NULL;
        hf->body_parts = NULL;
        hf->part = (*headerfault).partPtr();
        if ((*headerfault).namespace_)
          hf->URI = urienc(definitions.soap, (*headerfault).namespace_);
        else
          hf->URI = service->URI;
        if (hf->part && hf->part->element)
          hf->name = types.aname(NULL, NULL, hf->part->element);
        else if (hf->URI && hf->part && hf->part->name && hf->part->type)
          hf->name = types.aname(NULL, hf->URI, hf->part->name);
        else
        {
          fprintf(stderr, "\nError in SOAP HeaderFault part definition: input part \"%s\" missing?\n", hf->part && hf->part->name ? hf->part->name : "?");
          hf->name = "";
        }
        hf->encodingStyle = (*headerfault).encodingStyle;
        hf->action = NULL;
        hf->style = document;      // irrelevant
        hf->use = (*headerfault).use;
        hf->mustUnderstand = false;
        hf->multipartRelated = NULL;
        hf->content = NULL;
        hf->layout = NULL;
        hf->ext_documentation = NULL;      // TODO: may wanto to add document content
        hf->documentation = NULL;          // TODO: may wanto to add document content
        service->headerfault[hf->name] = hf;
      }
    }
    for (std::vector<wsoap__header>::const_iterator wheader = ext_input->wsoap__header_.begin(); wheader != ext_input->wsoap__header_.end(); ++wheader)
    {
      Message *h = new Message();
      h->message = NULL;
      h->element = (*wheader).elementPtr();
      h->body_parts = NULL;
      h->part = NULL;
      h->URI = NULL;
      h->mustUnderstand = (*wheader).mustUnderstand_;
      h->name = types.aname(NULL, NULL, (*wheader).element);
      h->encodingStyle = NULL;
      h->style = document;      // irrelevant
      h->use = literal;
      h->multipartRelated = NULL;
      h->content = NULL;
      h->layout = NULL;
      h->ext_documentation = NULL;      // TODO: may want to add document content
      h->documentation = NULL;          // TODO: may want to add document content
      service->header[h->name] = h;
    }
  }
  // collect output headers and headerfaults
  if (ext_output)
  {
    const std::vector<soap__header> *soap__header_ = NULL;
    // check if soap header is in mime:multipartRelated
    if (ext_output->mime__multipartRelated_)
    {
      for (std::vector<mime__part>::const_iterator part = ext_output->mime__multipartRelated_->part.begin(); part != ext_output->mime__multipartRelated_->part.end(); ++part)
      if (!(*part).soap__header_.empty())
      {
        soap__header_ = &(*part).soap__header_;
        break;
      }
    }
    if (!soap__header_)
      soap__header_ = &ext_output->soap__header_;
    for (std::vector<soap__header>::const_iterator header = soap__header_->begin(); header != soap__header_->end(); ++header)
    {
      Message *h = new Message();
      h->message = (*header).messagePtr();
      h->element = NULL;
      h->body_parts = NULL;
      h->part = (*header).partPtr();
      h->URI = urienc(definitions.soap, (*header).namespace_);
      if (h->part && h->part->element)
        h->name = types.aname(NULL, NULL, h->part->element);
      else if (h->URI && h->part && h->part->name && h->part->type)
        h->name = types.aname(NULL, h->URI, h->part->name);
      else
      {
        fprintf(stderr, "\nError in SOAP Header part definition: output part \"%s\" missing?\n", h->part && h->part->name ? h->part->name : "?");
        h->name = "";
      }
      h->encodingStyle = (*header).encodingStyle;
      h->style = document;      // irrelevant
      h->use = (*header).use;
      h->mustUnderstand = false;
      h->multipartRelated = NULL;
      h->content = NULL;
      h->layout = NULL;
      h->ext_documentation = NULL;      // TODO: may want to add document content
      h->documentation = NULL;          // TODO: may want to add document content
      service->header[h->name] = h;
      for (std::vector<soap__headerfault>::const_iterator headerfault = (*header).headerfault.begin(); headerfault != (*header).headerfault.end(); ++headerfault)
      {
        Message *hf = new Message();
        hf->message = (*headerfault).messagePtr();
        hf->element = NULL;
        hf->body_parts = NULL;
        hf->part = (*headerfault).partPtr();
        if ((*headerfault).namespace_)
          hf->URI = urienc(definitions.soap, (*headerfault).namespace_);
        else
          hf->URI = service->URI;
        if (hf->part && hf->part->element)
          hf->name = types.aname(NULL, NULL, hf->part->element);
        else if (hf->URI && hf->part && hf->part->name && hf->part->type)
          hf->name = types.aname(NULL, hf->URI, hf->part->name);
        else
        {
          fprintf(stderr, "\nError in SOAP HeaderFault part definition: output part \"%s\" missing?\n", hf->part && hf->part->name ? hf->part->name : "?");
          hf->name = "";
        }
        hf->encodingStyle = (*headerfault).encodingStyle;
        hf->action = NULL;
        hf->style = document;      // irrelevant
        hf->use = (*headerfault).use;
        hf->mustUnderstand = false;
        hf->multipartRelated = NULL;
        hf->content = NULL;
        hf->layout = NULL;
        hf->ext_documentation = NULL;      // TODO: may wanto to add document content
        hf->documentation = NULL;          // TODO: may wanto to add document content
        service->headerfault[hf->name] = hf;
      }
    }
    for (std::vector<wsoap__header>::const_iterator wheader = ext_output->wsoap__header_.begin(); wheader != ext_output->wsoap__header_.end(); ++wheader)
    {
      Message *h = new Message();
      h->message = NULL;
      h->element = (*wheader).elementPtr();
      h->body_parts = NULL;
      h->part = NULL;
      h->URI = NULL;
      h->mustUnderstand = (*wheader).mustUnderstand_;
      h->name = types.aname(NULL, NULL, (*wheader).element);
      h->encodingStyle = NULL;
      h->style = document;      // irrelevant
      h->use = literal;
      h->multipartRelated = NULL;
      h->content = NULL;
      h->layout = NULL;
      h->ext_documentation = NULL;      // TODO: may want to add document content
      h->documentation = NULL;          // TODO: may want to add document content
      service->header[h->name] = h;
    }
  }
}

void Definitions::analyze_faults(const wsdl__definitions& definitions, Service *service, Operation *op, std::vector<wsdl__ext_operation>::const_iterator& operation)
{
  for (std::vector<wsdl__ext_fault>::const_iterator fault = (*operation).fault.begin(); fault != (*operation).fault.end(); ++fault)
  {
    Message *f = analyze_fault(definitions, service, *fault);
    if (f)
    {
      op->outfault.push_back(f);
      service->fault[f->name] = f;
    }
  }
  for (std::vector<wsdl__ext_fault>::const_iterator infault = (*operation).infault.begin(); infault != (*operation).infault.end(); ++infault)
  {
    Message *f = analyze_fault(definitions, service, *infault);
    if (f)
    {
      op->infault.push_back(f);
      service->fault[f->name] = f;
    }
  }
  for (std::vector<wsdl__ext_fault>::const_iterator outfault = (*operation).outfault.begin(); outfault != (*operation).outfault.end(); ++outfault)
  {
    Message *f = analyze_fault(definitions, service, *outfault);
    if (f)
    {
      op->outfault.push_back(f);
      service->fault[f->name] = f;
    }
  }
}

Message *Definitions::analyze_fault(const wsdl__definitions& definitions, Service *service, const wsdl__ext_fault& ext_fault)
{
  Message *f = NULL;
  const wsdl__fault *fault = ext_fault.faultPtr();
  if (fault && (fault->messagePtr() || fault->elementPtr()))
  {
    f = new Message();
    f->message = fault->messagePtr();
    f->element = fault->elementPtr();
    f->body_parts = NULL;
    f->part = NULL;
    f->encodingStyle = NULL;
    if (ext_fault.soap__fault_)
      f->encodingStyle = ext_fault.soap__fault_->encodingStyle;
    f->action = NULL;
    if (fault->wsa__Action)
      f->action = fault->wsa__Action;
    else if (fault->wsam__Action)
      f->action = fault->wsam__Action;
    else if (fault->wsaw__Action)
      f->action = fault->wsaw__Action;
    if (f->element && f->element->schemaPtr())
      f->URI = urienc(definitions.soap, f->element->schemaPtr()->targetNamespace);
    else if (ext_fault.soap__fault_ && ext_fault.soap__fault_->namespace_)
      f->URI = urienc(definitions.soap, ext_fault.soap__fault_->namespace_);
    else
      f->URI = service->URI; // must have a unique URI
    f->style = document;        // irrelevant
    f->use = literal;
    if (ext_fault.soap__fault_)
      f->use = ext_fault.soap__fault_->use;
    if (ext_fault.wsoap__code)
    {
      size_t l = strlen(ext_fault.wsoap__code);
      if (ext_fault.wsoap__subcodes)
        l += strlen(ext_fault.wsoap__subcodes);
      char *s = (char*)soap_malloc(definitions.soap, l + 40);
      if (s)
      {
        (SOAP_SNPRINTF(s, l + 40, l + 19), "\"%s\" with subcodes \"%s\"", ext_fault.wsoap__code, ext_fault.wsoap__subcodes ? ext_fault.wsoap__subcodes : "");
        f->body_parts = s;
      }
    }
    else
      f->ext_documentation = ext_fault.documentation;
    f->mustUnderstand = false;
    f->multipartRelated = NULL;
    f->content = NULL;
    f->layout = NULL;
    if (f->message)
    {
      f->name = types.aname("_", f->URI, f->message->name);
      f->documentation = f->message->documentation;
    }
    else
    {
      f->name = types.aname(NULL, f->URI, fault->element);
      f->documentation = fault->documentation;
    }
    // collect fault message policies
    if (fault->wsp__Policy_)
      f->policy.push_back(fault->wsp__Policy_);
    if (fault->wsp__PolicyReference_ && fault->wsp__PolicyReference_->policyPtr())
      f->policy.push_back(fault->wsp__PolicyReference_->policyPtr());
    if (ext_fault.wsp__Policy_)
      f->policy.push_back(ext_fault.wsp__Policy_);
    if (ext_fault.wsp__PolicyReference_ && ext_fault.wsp__PolicyReference_->policyPtr())
      f->policy.push_back(ext_fault.wsp__PolicyReference_->policyPtr());
  }
  else if (ext_fault.soap__fault_ && ext_fault.soap__fault_->name)
    fprintf(stderr, "\nError: no wsdl:definitions/binding/operation/fault/soap:fault \"%s\"\n", ext_fault.soap__fault_->name);
  else
    fprintf(stderr, "\nError: no wsdl:definitions/binding/operation/fault/soap:fault\n");
  return f;
}

void Definitions::analyze_application(const wsdl__definitions& definitions)
{
  // Analyze WADL application resources and build Service information
  const wadl__application *app = definitions.appPtr();
  if (app)
  {
    if (vflag)
      fprintf(stderr, "\nAnalyzing and collecting WADL application resources\n");
    for (std::vector<wadl__resources>::const_iterator resources = app->resources.begin(); resources != app->resources.end(); ++resources)
    {
      const char *URI = (*resources).base;
      const char *prefix = types.nsprefix(service_prefix, URI);
      Service *service = services[prefix];
      if (!service)
      {
        service = services[prefix] = new Service();
        service->prefix = prefix;
        service->URI = urienc(definitions.soap, URI);
        size_t l = strlen(prefix);
        char *name = (char*)soap_malloc(definitions.soap, l + 5);
        (SOAP_SNPRINTF(name, l + 5, l + 4), "%sREST", prefix);
        service->name = name;
        service->transport = NULL;
        service->type = NULL;
        service->location.insert(URI);
        for (std::vector<wadl__resource>::const_iterator resource = (*resources).resource.begin(); resource != (*resources).resource.end(); ++resource)
        {
          const char *path = urienc(definitions.soap, (*resource).path);
          const char *queryType = (*resource).queryType;
          analyze_resource(definitions, service, &*resource, URI, path, queryType);
          for (std::vector<wadl__resource_USCOREtype*>::const_iterator type = (*resource).typePtrs().begin(); type != (*resource).typePtrs().end(); ++type)
            analyze_resource(definitions, service, *type, URI, path, queryType);
        }
      }
    }
  }
}

void Definitions::analyze_resource(const wsdl__definitions& definitions, Service *service, const wadl__resource_USCOREtype *type, const char *URI, const char *path, const char *queryType)
{
  for (std::vector<__wadl__method_resource_choice>::const_iterator choice = type->__choice.begin(); choice != type->__choice.end(); ++choice)
  {
    const wadl__method *method = (*choice).method;
    if (method)
    {
      while (method->methodPtr() && !method->id)
        method = method->methodPtr();
      Operation *op = new Operation();
      op->is_rest = true;
      op->protocol = soap_wadl__HTTPMethods2s(definitions.soap, method->name);
      op->operation = NULL; // wsdl__operation_;
      op->prefix = service->prefix;
      op->URI = service->URI;
      op->mep = NULL;
      op->style = document;
      op->parameterOrder = NULL;
      if (path && *path != '/')
      {
        size_t l = strlen(path);
        char *action = (char*)soap_malloc(definitions.soap, l + 2);
        (SOAP_SNPRINTF(action, l + 2, l + 1), "/%s", path);
        op->action = action;
      }
      else
      {
        op->action = path;
      }
      op->documentation = NULL;
      op->operation_documentation = NULL;
      if (!method->doc.empty())
      {
        const wadl__doc& doc = *method->doc.begin();
        op->documentation = doc.title;
        op->operation_documentation = doc.__mixed;
      }
      const char *name = method->id ? method->id : op->protocol;
      op->name = types.aname(NULL, URI, name);
      op->input_name = types.oname("__", URI, name);
      size_t l = strlen(name);
      char *resname = (char*)soap_malloc(definitions.soap, l + 9);
      if (resname)
      {
        (SOAP_SNPRINTF(resname, l + 9, l + 8), "%sResponse", name);
        op->output_name = types.oname("__", URI, resname);
      }
      op->input = NULL;
      if (method->request)
      {
        wsdl__message *message = NULL;
        const xs__element *element = NULL;
        const char *documentation = NULL;
        for (std::vector<wadl__representation>::const_iterator rep = method->request->representation.begin(); rep != method->request->representation.end(); ++rep)
        {
          const wadl__representation *representation = &*rep;
          while (representation->representationPtr() && !representation->id)
            representation = representation->representationPtr();
          if (!representation->mediaType || strstr(representation->mediaType, "xml"))
          {
            if (representation->elementPtr())
              element = representation->elementPtr();
            if (!representation->doc.empty())
            {
              const wadl__doc& doc = *representation->doc.begin();
              if (doc.__mixed && *doc.__mixed)
                documentation = doc.__mixed;
              else
                documentation = doc.title;
            }
            break;
          }
        }
        std::vector<wadl__param> allparam = type->param;
        for (std::vector<wadl__param>::const_iterator par = method->request->param.begin(); par != method->request->param.end(); ++par)
          allparam.push_back(*par);
        for (std::vector<wadl__param>::const_iterator par = allparam.begin(); par != allparam.end(); ++par)
        {
          const wadl__param *param = &*par;
          while (param->paramPtr() && !param->id)
            param = param->paramPtr();
          if (param->name)
          {
            if (param->style && (*param->style == matrix || *param->style == header || *param->style == plain))
            {
              if (!Wflag)
                fprintf(stderr, "\nWarning: ignoring param '%s' with unsupported style '%s' in method/response/representation\n", param->name, soap_wadl__ParamStyle2s(definitions.soap, *param->style));
            }
            else
            {
              if (!message)
              {
                message = soap_new_wsdl__message(definitions.soap);
                message->soap_default(definitions.soap);
              }
              wsdl__part part;
              part.soap_default(definitions.soap);
              part.name = param->name;
              if (param->type)
              {
                part.type = param->type;
                if (param->simpleTypePtr())
                  part.simpleTypePtr(param->simpleTypePtr());
                else if (param->complexTypePtr())
                  part.complexTypePtr(param->complexTypePtr());
              }
              else
              {
                part.type = (char*)"xsd:string";
              }
              part.set_optional(param->required == false);
              if (param->repeating)
              {
                part.set_optional(true);
                part.set_repeating(true);
              }
              else if (param->default_)
              {
                part.set_optional(true);
                // There is no point in setting the default value if param is optional
                // part.set_default(param->default_);
              }
              else if (param->fixed)
              {
                part.set_optional(false);
                part.set_fixed(true);
                part.set_default(param->fixed);
              }
              for (std::vector<wadl__option>::const_iterator i = param->option.begin(); i != param->option.end(); ++i)
                part.set_option((*i).value);
              if (!param->doc.empty())
              {
                const wadl__doc& doc = *param->doc.begin();
                if (doc.__mixed && *doc.__mixed)
                  part.documentation = doc.__mixed;
                else
                  part.documentation = doc.title;
              }
              message->part.push_back(part);
            }
          }
        }
        op->input = new Message;
        op->input->name = op->input_name;
        op->input->URI = service->URI;
        op->input->style = op->style;
        op->input->use = literal;
        op->input->encodingStyle = NULL;
        op->input->action = NULL;
        op->input->message = message;
        op->input->element = element;
        op->input->part = NULL;
        op->input->mustUnderstand = false;
        op->input->multipartRelated = NULL;
        op->input->content = soap_new_mime__content(definitions.soap);
        op->input->content->soap_default(definitions.soap);
        op->input->content->type = (char*)queryType;
        op->input->body_parts = NULL;
        op->input->layout = NULL;
        op->input->multipartRelated = NULL;
        op->input->layout = NULL;
        op->input->documentation = documentation;
        op->input->ext_documentation = NULL;
        if (!method->request->doc.empty())
        {
          const wadl__doc& doc = *method->request->doc.begin();
          if (doc.__mixed && *doc.__mixed)
            op->input->ext_documentation = doc.__mixed;
          else
            op->input->ext_documentation = doc.title;
        }
      }
      op->output = NULL;
      for (std::vector<wadl__response>::const_iterator response = method->response.begin(); response != method->response.end(); ++response)
      {
        const char *name = NULL;
        const xs__element *element = NULL;
        for (std::vector<wadl__representation>::const_iterator rep = (*response).representation.begin(); rep != (*response).representation.end(); ++rep)
        {
          const wadl__representation *representation = &*rep;
          while (representation->representationPtr() && !representation->id)
            representation = representation->representationPtr();
          if (!representation->mediaType || strstr(representation->mediaType, "xml"))
          {
            if (representation->elementPtr())
            {
              name = representation->element;
              element = representation->elementPtr();
            }
            for (std::vector<wadl__param>::const_iterator param = representation->param.begin(); param != representation->param.end(); ++param)
              if (!Wflag)
                fprintf(stderr, "\nWarning: ignoring param '%s' in method/response/representation\n", param->name ? param->name : "");
            break;
          }
        }
        if (element)
        {
          if (!op->output && (!(*response).status
                           || strstr((*response).status, "200")
                           || strstr((*response).status, "201")
                           || strstr((*response).status, "202")))
          {
            op->output = new Message();
            op->output->name = op->output_name;
            op->output->URI = service->URI;
            op->output->style = op->style;
            op->output->use = literal;
            op->output->encodingStyle = NULL;
            op->output->action = NULL;
            op->output->message = NULL;
            op->output->element = element;
            op->output->part = NULL;
            op->output->mustUnderstand = false;
            op->output->multipartRelated = NULL;
            op->output->content = NULL;
            op->output->body_parts = NULL;
            op->output->layout = NULL;
            op->output->ext_documentation = NULL;
            op->output->multipartRelated = NULL;
            op->output->layout = NULL;
            op->output->ext_documentation = NULL;
            op->output->documentation = NULL;
          }
          else
          {
            Message *fault = new Message();
            fault->name = types.aname(NULL, NULL, name);
            fault->URI = service->URI;
            fault->style = op->style;
            fault->use = literal;
            fault->encodingStyle = NULL;
            fault->action = NULL;
            fault->message = NULL;
            fault->element = element;
            fault->part = NULL;
            fault->mustUnderstand = false;
            fault->multipartRelated = NULL;
            fault->content = NULL;
            fault->body_parts = (*response).status;
            fault->layout = NULL;
            fault->ext_documentation = NULL;
            fault->multipartRelated = NULL;
            fault->layout = NULL;
            fault->ext_documentation = NULL;
            fault->documentation = NULL;
            op->outfault.push_back(fault);
          }
        }
      }
      service->operation.push_back(op);
    }
    wadl__resource *resource = (*choice).resource;
    if (resource)
    {
      const char *fullpath = NULL;
      if (resource->path)
        fullpath = urienc(definitions.soap, resource->path);
      if (path)
      {
        if (fullpath)
        {
          size_t l = strlen(path) + strlen(fullpath) + 2;
          char *s = (char*)soap_malloc(definitions.soap, l);
          (SOAP_SNPRINTF_SAFE(s, l), "%s/%s", path, fullpath + (*fullpath == '/'));
          fullpath = s;
        }
        else
        {
          fullpath = path;
        }
      }
      if (!type->param.empty())
      {
        std::vector<wadl__param> allparam = type->param;
        for (std::vector<wadl__param>::const_iterator par = resource->param.begin(); par != resource->param.end(); ++par)
          allparam.push_back(*par);
        resource->param.swap(allparam);
      }
      analyze_resource(definitions, service, resource, URI, fullpath, resource->queryType ? resource->queryType : queryType);
    }
  }
}

void Definitions::compile(const wsdl__definitions& definitions)
{
  // compile the definitions and generate gSOAP header file
  const char *defs;
  if (definitions.name)
    defs = types.aname(NULL, NULL, definitions.name);
  else
    defs = "Service";
  ident();
  fprintf(stream, "/**\n\n");
  if (!Lflag)
    fprintf(stream, "@page page_notes Notes\n\n@note HINTS:\n - Run soapcpp2 on %s to generate the SOAP/XML processing logic:\n   Use soapcpp2 -I to specify paths for #import\n   Use soapcpp2 -j to generate improved proxy and server classes.\n   Use soapcpp2 -r to generate a report.\n - Edit 'typemap.dat' to control namespace bindings and type mappings:\n   It is strongly recommended to customize the names of the namespace prefixes\n   generated by wsdl2h. To do so, modify the prefix bindings in the Namespaces\n   section below and add the modified lines to 'typemap.dat' to rerun wsdl2h.\n - Run Doxygen (www.doxygen.org) on this file to generate documentation.\n - Use wsdl2h -c to generate pure C code.\n - Use wsdl2h -R to include the REST operations defined by the WSDLs.\n - Use wsdl2h -O3 or -O4 to optimize by removing unused schema components.\n - Use wsdl2h -d to enable DOM support for xsd:any and xsd:anyType.\n - Use wsdl2h -F to simulate struct-type derivation in C (also works in C++).\n - Use wsdl2h -f to generate flat C++ class hierarchy, removes type derivation.\n - Use wsdl2h -g to generate top-level root elements with readers and writers.\n - Use wsdl2h -U to map XML names to C++ Unicode identifiers instead of _xNNNN.\n - Use wsdl2h -u to disable the generation of unions.\n - Use wsdl2h -L to remove this @note and all other @note comments.\n - Use wsdl2h -nname to use name as the base namespace prefix instead of 'ns'.\n - Use wsdl2h -Nname for service prefix and produce multiple service bindings\n - Struct/class members serialized as XML attributes are annotated with a '@'.\n - Struct/class members that have a special role are annotated with a '$'.\n\n@warning\n   DO NOT INCLUDE THIS ANNOTATED FILE DIRECTLY IN YOUR PROJECT SOURCE CODE.\n   USE THE FILES GENERATED BY soapcpp2 FOR YOUR PROJECT'S SOURCE CODE:\n   THE GENERATED soapStub.h FILE CONTAINS THIS CONTENT WITHOUT ANNOTATIONS.\n\n", outfile ? outfile : "this file");
  fprintf(stream, "@copyright LICENSE:\n@verbatim\n%s@endverbatim\n\n*/\n\n", licensenotice);
  // gsoap compiler options: 'w' disables WSDL/schema output to avoid file collisions
  if (cflag)
    fprintf(stream, "\n//gsoapopt c,w\n");
  else if (c11flag)
    fprintf(stream, "\n//gsoapopt c++11,w\n");
  else
    fprintf(stream, "\n//gsoapopt c++,w\n");
  banner("Definitions", definitions.targetNamespace ? definitions.targetNamespace : "");
  // copy documentation from WSDL definitions
  if (definitions.documentation)
  {
    fprintf(stream, "/* WSDL Documentation:\n\n");
    text(definitions.documentation);
    fprintf(stream, "*/\n\n");
  }
  if (definitions.version)
  {
    banner("Version", definitions.version);
    fprintf(stream, "#define SOAP_WSDL_VERSION \"%s\"\n", definitions.version);
  }
  if (!cflag)
  {
    const char *s;
    if (!sflag)
    {
      s = types.vname("$CONTAINER");
      if (s && *s != '*' && *s != '$')
      {
        banner("$CONTAINER typemap variable:", s);
        if (!strcmp(s, "std::deque"))
          fprintf(stream, "#include <deque>\n");
        else if (!strcmp(s, "std::list"))
          fprintf(stream, "#include <list>\n");
        else if (!strcmp(s, "std::vector"))
          fprintf(stream, "#include <vector>\n");
        else if (!strcmp(s, "std::set"))
          fprintf(stream, "#include <set>\n");
        fprintf(stream, "template <class T> class %s;\n", s);
      }
    }
    s = types.vname("$POINTER");
    if (s && *s != '*' && *s != '$')
    {
      banner("$POINTER typemap variable:", s);
      fprintf(stream, "volatile template <class T> class %s;\n", s);
    }
    s = types.vname("$SIZE");
    if (s && *s != '*' && *s != '$')
      banner("$SIZE typemap variable:", s);
  }
  else
  {
    const char *s = types.vname("$SIZE");
    if (s && *s != '*' && *s != '$')
      banner("$SIZE typemap variable:", s);
  }
  banner("Import");
  if (dflag)
  {
    fprintf(stream, "\n// dom.h declares the DOM xsd__anyType object (compiler and link with dom.cpp)\n");
    fprintf(stream, "#import \"dom.h\"\n");
  }
  if (mflag)
  {
    fprintf(stream, "#import \"");
    fprintf(stream, "xsd.h\"\t// import built-in XSD types.\n");
  }
  for (SetOfString::const_iterator u = exturis.begin(); u != exturis.end(); ++u)
  {
    bool found = false;
    size_t n = strlen(*u);
    for (SetOfString::const_iterator i = definitions.builtinTypes().begin(); i != definitions.builtinTypes().end(); ++i)
    {
      if (**i == '"' && !strncmp(*u, *i + 1, n) && (*i)[n+1] == '"')
      {
        found = true;
        break;
      }
    }
    if (!found)
    {
      for (SetOfString::const_iterator j = definitions.builtinElements().begin(); j != definitions.builtinElements().end(); ++j)
      {
        if (**j == '"' && !strncmp(*u, *j + 1, n) && (*j)[n+1] == '"')
        {
          found = true;
          break;
        }
      }
    }
    if (!found)
    {
      for (SetOfString::const_iterator k = definitions.builtinAttributes().begin(); k != definitions.builtinAttributes().end(); ++k)
      {
        if (**k == '"' && !strncmp(*u, *k + 1, n) && (*k)[n+1] == '"')
        {
          found = true;
          break;
        }
      }
    }
    if (found)
    {
      if (vflag)
        fprintf(stderr, "import %s\n", *u);
      fprintf(stream, "#import \"%s.h\"\t// %s = <%s>\n", types.nsprefix(NULL, *u), types.nsprefix(NULL, *u), *u);
    }
  }
  banner("Schema Namespaces");
  // Determine if bindings use SOAP 1.2
  soap12 = false;
  for (Namespace *p = definitions.soap->local_namespaces; p && p->id; p++)
  {
    if (p->out && !strcmp(p->id, "soap") && !strcmp(p->out, "http://schemas.xmlsoap.org/wsdl/soap12/"))
    {
      soap12 = true;
      break;
    }
  }
  if (!soap12)
  {
    for (std::vector<wsdl__import>::const_iterator i = definitions.import.begin(); i != definitions.import.end(); ++i)
    {
      if ((*i).definitionsPtr())
      {
        for (Namespace *p = (*i).definitionsPtr()->soap->local_namespaces; p && p->id; p++)
        {
          if (p->out && !strcmp(p->id, "soap") && !strcmp(p->out, "http://schemas.xmlsoap.org/wsdl/soap12/"))
          {
            soap12 = true;
            break;
          }
        }
      }
      if (soap12)
        break;
    }
  }
  if (definitions.types)
  {
    if (!Lflag)
    {
      fprintf(stream, "\n/* NOTE:\n\nIt is strongly recommended to customize the names of the namespace prefixes\ngenerated by wsdl2h. To do so, modify the prefix bindings below and add the\nmodified lines to 'typemap.dat' then rerun wsdl2h (use wsdl2h -t typemap.dat):\n\n");
      if (definitions.targetNamespace && *definitions.targetNamespace)
        fprintf(stream, "%s = \"%s\"\n", types.nsprefix(service_prefix, definitions.targetNamespace), definitions.targetNamespace);
      for (std::vector<xs__schema*>::const_iterator schema1 = definitions.types->xs__schema_.begin(); schema1 != definitions.types->xs__schema_.end(); ++schema1)
        if ((*schema1)->targetNamespace && *(*schema1)->targetNamespace && (!definitions.targetNamespace || strcmp((*schema1)->targetNamespace, definitions.targetNamespace)))
          fprintf(stream, "%s = \"%s\"\n", types.nsprefix(NULL, (*schema1)->targetNamespace), (*schema1)->targetNamespace);
      fprintf(stream, "\n*/\n");
    }
    for (std::vector<xs__schema*>::const_iterator schema2 = definitions.types->xs__schema_.begin(); schema2 != definitions.types->xs__schema_.end(); ++schema2)
    {
      const char *t = types.nsprefix(NULL, (*schema2)->targetNamespace);
      if (t && *t)
      {
        fprintf(stream, "\n");
        types.document((*schema2)->annotation);
        fprintf(stream, "#define SOAP_NAMESPACE_OF_%s\t\"%s\"\n", types.aname(NULL, NULL, t), urienc(definitions.soap, (*schema2)->targetNamespace));
        fprintf(stream, schemaformat, t, "namespace", urienc(definitions.soap, (*schema2)->targetNamespace));
        if ((*schema2)->elementFormDefault == (*schema2)->attributeFormDefault)
          fprintf(stream, schemaformat, t, "form", (*schema2)->elementFormDefault == qualified ? "qualified" : "unqualified");
        else
        {
          fprintf(stream, schemaformat, t, "elementForm", (*schema2)->elementFormDefault == qualified ? "qualified" : "unqualified");
          fprintf(stream, schemaformat, t, "attributeForm", (*schema2)->attributeFormDefault == qualified ? "qualified" : "unqualified");
        }
      }
    }
  }
  // generate the prototypes first: these should allow use before def, e.g. class names then generate the defs
  // check if xsd:anyType is used
  if (!pflag)
  {
    for (SetOfString::const_iterator i = definitions.builtinTypes().begin(); i != definitions.builtinTypes().end(); ++i)
    {
      if (!strcmp(*i, "xs:anyType"))
      {
        pflag = 1;
        if (!wflag && !Pflag && !fflag && (!cflag || Fflag))
          fprintf(stderr, "\nNote: option -p auto-enabled to generate wrappers for built-in types derived from xsd__anyType to support polymorphism in XML by (de)serializing any derived type of xsd:anyType (or xsd:anySimpleType) as elements annotated by xsi:type attributes in XML, use option -P to suppress and disable this feature\n");
        break;
      }
    }
  }
  // produce built-in primitive types
  banner("Built-in Schema Types and Top-Level Elements and Attributes");
  // define xsd:anyType first, if used
  if (pflag)
  {
    if (Fflag && !fflag && !Pflag)
    {
      if (cflag)
      {
        types.usetypemap["xsd__anyType"] = "struct xsd__anyType_";
        types.ptrtypemap["xsd__anyType"] = "struct xsd__anyType_*";
      }
      else
      {
        fprintf(stream, "/// Built-in root XSD type \"xs:anyType\".\n");
        fprintf(stream, "class xsd__anyType_;\n\n");
        types.usetypemap["xsd__anyType"] = "xsd__anyType_";
        types.ptrtypemap["xsd__anyType"] = "xsd__anyType_*";
        types.knames.insert("xsd__anyType_");
        const char *r = types.vname("$POINTER");
        if (r && *r != '*' && *r != '$')
        {
          MapOfStringToString::iterator i = types.smptypemap.find("xsd__anyType");
          if (i != types.smptypemap.end())
            types.smptypemap.erase(i);
          types.smptypemap["xsd__anyType"] = types.pname(true, true, NULL, NULL, "xs:anyType");
        }
      }
    }
    else if (!dflag)
    {
      const char *s, *t;
      t = types.cname(NULL, NULL, "xs:anyType");
      s = types.deftypemap[t];
      if (s)
      {
        if (*s)
        {
          if (!mflag)
          {
            fprintf(stream, "/// Built-in XSD root type \"xs:anyType\".\n///\n");
            if (!cflag && !Lflag)
              fprintf(stream, "/// @note Call virtual method soap_type() generated by soapcpp2 to check runtime type is SOAP_TYPE_%s or a derived type.  Use option -P to remove this class.\n", t);
            fprintf(stream, "%s\n\n", s);
          }
        }
        s = types.usetypemap[t];
        if (s)
        {
          if (mflag)
            fprintf(stream, "//  xsd.h: should define type %s\n\n", s);
          types.knames.insert(s);
        }
      }
      else
      {
        fprintf(stderr, "\nError: no xsd__anyType defined in type map\n");
        pflag = 0;
      }
    }
  }
  if (fflag || Pflag)
    pflag = 0;
  if (vflag)
    fprintf(stderr, "\nGenerating built-in types\n");
  for (SetOfString::const_iterator i = definitions.builtinTypes().begin(); i != definitions.builtinTypes().end(); ++i)
  {
    const char *s, *t;
    if (!cflag && !strcmp(*i, "xs:anyType"))
      continue;
    // make an exception for the rarely-used xs:anySimpleType to equal xs:anyType to make coding easier
    if (Qflag && !strcmp(*i, "xs:anySimpleType") && types.deftypemap.find("xsd__anySimpleType") == types.deftypemap.end())
    {
      if (pflag)
      {
        types.deftypemap["xsd__anySimpleType"] = "";
        if (Fflag)
        {
          if (cflag)
          {
            types.usetypemap["xsd__anySimpleType"] = "struct xsd__anyType_";
            types.ptrtypemap["xsd__anySimpleType"] = "struct xsd__anyType_*";
          }
          else
          {
            types.usetypemap["xsd__anySimpleType"] = "xsd__anyType_";
            types.ptrtypemap["xsd__anySimpleType"] = "xsd__anyType_*";
          }
        }
        else if (cflag)
        {
          types.usetypemap["xsd__anySimpleType"] = types.usetypemap["xsd__anyType"];
        }
        else
        {
          types.usetypemap["xsd__anySimpleType"] = "xsd__anyType*";
          types.ptrtypemap["xsd__anySimpleType"] = "xsd__anyType*";
        }
      }
      else
      {
        types.deftypemap["xsd__anySimpleType"] = "";
        types.usetypemap["xsd__anySimpleType"] = types.usetypemap["xsd__anyType"];
        types.ptrtypemap["xsd__anySimpleType"] = types.ptrtypemap["xsd__anyType"];
      }
    }
    // make an exception for SOAP-ENC:Array
    if (!strcmp(*i, "SOAP-ENC:Array") && types.deftypemap.find("SOAP_ENC__Array") == types.deftypemap.end())
    {
      if (pflag)
      {
        if (Fflag)
        {
          if (cflag)
          {
            types.deftypemap["SOAP_ENC__Array"] = "struct SOAP_ENC__Array\n{\n/// Pointer to array of elements <item> of type xs:anyType\n    struct xsd__anyType_ *__ptr;\n/// Size of the array\n    int __size;\n};";
            types.usetypemap["SOAP_ENC__Array"] = "struct SOAP_ENC__Array";
          }
          else
          {
            types.deftypemap["SOAP_ENC__Array"] = "class SOAP_ENC__Array\n{ public:\n/// Pointer to array of elements <item> of type xs:anyType\n    xsd__anyType_ *__ptr;\n/// Size of the array\n    int __size;\n};";
            types.usetypemap["SOAP_ENC__Array"] = "SOAP_ENC__Array";
          }
        }
        else if (cflag)
        {
          types.deftypemap["SOAP_ENC__Array"] = soap_strdup(definitions.soap, (std::string("struct SOAP_ENC__Array\n{\n/// Pointer to array of elements <item> of type xs:anyType\n    ") + std::string(types.usetypemap["xsd__anyType"]) + std::string(" *__ptr;\n/// Size of the array\n    int __size;\n};")).c_str());
          types.usetypemap["SOAP_ENC__Array"] = "struct SOAP_ENC__Array";
        }
        else
        {
          types.deftypemap["SOAP_ENC__Array"] = "class SOAP_ENC__Array : public xsd__anyType\n{ public:\n/// Pointer to array of elements <item> of type xs:anyType\n    xsd__anyType **__ptr;\n/// Size of the array\n    int __size;\n};";
          types.usetypemap["SOAP_ENC__Array"] = "SOAP_ENC__Array";
        }
      }
      else if (cflag)
      {
        if (dflag)
        {
          types.deftypemap["SOAP_ENC__Array"] = "struct SOAP_ENC__Array\n{\n/// Pointer to array of DOM nodes\n    xsd__anyType *__ptr;\n/// Size of the array\n    int __size;\n};";
          types.usetypemap["SOAP_ENC__Array"] = "struct SOAP_ENC__Array";
        }
        else
        {
          types.deftypemap["SOAP_ENC__Array"] = "struct SOAP_ENC__Array\n{\n/// Pointer to array of XML strings\n    _XML *__ptr;\n/// Size of the array\n    int __size;\n};";
          types.usetypemap["SOAP_ENC__Array"] = "struct SOAP_ENC__Array";
        }
      }
      else
      {
        if (dflag)
        {
          types.deftypemap["SOAP_ENC__Array"] = "class SOAP_ENC__Array\n{ public:\n/// Pointer to array of DOM nodes\n    xsd__anyType *__ptr;\n/// Size of the array\n    int __size;\n};";
          types.usetypemap["SOAP_ENC__Array"] = "SOAP_ENC__Array";
        }
        else
        {
          types.deftypemap["SOAP_ENC__Array"] = "class SOAP_ENC__Array\n{ public:\n/// Pointer to array of XML strings\n    _XML *__ptr;\n/// Size of the array\n    int __size;\n};";
          types.usetypemap["SOAP_ENC__Array"] = "SOAP_ENC__Array";
        }
      }
    }
    t = types.cname(NULL, NULL, *i);
    s = types.deftypemap[t];
    if (s)
    {
      if (*s)
      {
        if (**i == '"')
          fprintf(stream, "/// Imported type \"%s\" from typemap %s.\n", *i, mapfile ? mapfile : "");
        else
          fprintf(stream, "/// Built-in type \"%s\".\n", *i);
        if (mflag)
        {
          fprintf(stream, "//  (declaration of %s removed by option -m)\n\n", t);
        }
        else if (!iflag && !imported(*i))
        {
          types.format(s);
          fprintf(stream, "\n");
        }
      }
      s = types.usetypemap[t];
      if (s && *s)
      {
        if (mflag && **i != '"')
          fprintf(stream, "//  xsd.h: typemap override of type %s with %s\n\n", t, s);
        if (types.knames.find(s) == types.knames.end())
          types.knames.insert(s);
      }
    }
  }
  for (SetOfString::const_iterator i = definitions.builtinTypes().begin(); i != definitions.builtinTypes().end(); ++i)
  {
    const char *s, *t;
    if (!cflag && (!strcmp(*i, "xs:anyType") || (Qflag && !strcmp(*i, "xs:anySimpleType")) || !strcmp(*i, "SOAP-ENC:Array")))
      continue;
    t = types.cname(NULL, NULL, *i);
    s = types.deftypemap[t];
    if (!s)
    {
      if (!mflag)
      {
        if (**i == '"')
        {
          fprintf(stream, "// Imported type \"%s\" defined by %s.\n\n", *i, t);
        }
        else if (!iflag)
        {
          s = types.tname(NULL, NULL, "xsd:string");
          fprintf(stream, "/// Primitive built-in type \"%s\".\n", *i);
          fprintf(stream, "typedef %s %s;\n\n", s, t);
          types.deftname(TYPEDEF, false, types.is_ptr(NULL, NULL, "xsd:string"), NULL, NULL, *i, s);
          const char *u = types.uri(*i);
          if (u && !types.uris[u])
          {
            fprintf(stream, schemaformat, types.nsprefix(types.prefix(*i), u), "namespace", u);
            fprintf(stream, "\n");
          }
        }
      }
      else if (**i == '"')
      {
        fprintf(stream, "//  Imported type \"%s\" defined by %s.\n\n", *i, t);
      }
      else
      {
        fprintf(stream, "//  xsd.h: should define type %s\n\n", t);
      }
      types.deftname(TYPEDEF, false, false, NULL, NULL, *i, s);
    }
    if (pflag && !cflag && !Fflag)
    {
      if (!zflag || zflag > 8)
        s = types.wname(NULL, NULL, *i, NOLOOKUP);
      else
        s = types.aname(NULL, NULL, *i);
      if (!mflag)
      {
        fprintf(stream, "/// Class wrapper %s for built-in type \"%s\" extends xsd__anyType.\n", s, *i);
        if (!Lflag)
          fprintf(stream, "/// @note Call virtual method soap_type() generated by soapcpp2 to check runtime type is SOAP_TYPE_%s or a derived type.  Use option -P to remove this class.\n", s);
        fprintf(stream, "class %s : public xsd__anyType\n{ public:\n", s);
        fprintf(stream, elementformat, types.tname(NULL, NULL, *i), "__item");
        fprintf(stream, ";\n};\n\n");
      }
      types.knames.insert(s);
    }
  }
  // produce built-in primitive elements
  if (vflag)
    fprintf(stderr, "\nGenerating built-in elements\n");
  for (SetOfString::const_iterator j = definitions.builtinElements().begin(); j != definitions.builtinElements().end(); ++j)
  {
    const char *s, *t;
    t = types.cname("_", NULL, *j);
    s = types.deftypemap[t];
    if (s)
    {
      if (*s)
      {
        if (**j == '"')
          fprintf(stream, "/// Imported element \"%s\" from typemap %s.\n\n", *j, mapfile ? mapfile : "");
        else
          fprintf(stream, "/// Built-in element \"%s\".\n\n", *j);
        if (mflag)
        {
          fprintf(stream, "//  (declaration of %s removed by option -m)\n\n", t);
        }
        else if (!iflag && !imported(*j))
        {
          types.format(s);
          fprintf(stream, "\n");
        }
      }
      s = types.usetypemap[t];
      if (s && *s)
      {
        if (mflag && **j != '"')
          fprintf(stream, "//  xsd.h: typemap override of element %s with %s\n\n", t, s);
        if (types.knames.find(s) == types.knames.end())
          types.knames.insert(s);
      }
    }
    else
    {
      if (!mflag)
      {
        if (**j == '"')
        {
          fprintf(stream, "// Imported element \"%s\" declared as %s.\n\n", *j, t);
        }
        else if (!iflag && !imported(*j))
        {
          s = types.tname(NULL, NULL, "xsd:any");
          fprintf(stream, "/// Built-in element \"%s\".\n", *j);
          fprintf(stream, "typedef %s %s;\n\n", s, t);
          types.deftname(TYPEDEF, false, true, "_", NULL, *j, s);  // already pointer
          const char *u = types.uri(*j);
          if (u && !types.uris[u])
          {
            fprintf(stream, schemaformat, types.nsprefix(types.prefix(*j), u), "namespace", u);
            fprintf(stream, "\n");
          }
        }
      }
      else if (**j == '"')
      {
        fprintf(stream, "//  Imported element \"%s\" declared as %s.\n\n", *j, t);
      }
      else
      {
        fprintf(stream, "//  xsd.h: should define element %s\n\n", t);
      }
      types.deftname(TYPEDEF, false, false, "_", NULL, *j, s);
    }
  }
  // produce built-in primitive attributes
  if (vflag)
    fprintf(stderr, "\nGenerating built-in attributes\n");
  for (SetOfString::const_iterator k = definitions.builtinAttributes().begin(); k != definitions.builtinAttributes().end(); ++k)
  {
    const char *s, *t;
    t = types.cname("_", NULL, *k);
    s = types.deftypemap[t];
    if (s)
    {
      if (*s)
      {
        if (**k == '"')
          fprintf(stream, "/// Imported attribute \"%s\" from typemap %s.\n", *k, mapfile ? mapfile : "");
        else
          fprintf(stream, "/// Built-in attribute \"%s\".\n", *k);
        if (mflag)
          fprintf(stream, "//  (declaration of %s removed by option -m)\n\n", t);
        else if (!iflag && !imported(*k))
        {
          types.format(s);
          fprintf(stream, "\n");
        }
      }
      s = types.usetypemap[t];
      if (s && *s)
      {
        if (mflag && **k != '"')
          fprintf(stream, "//  xsd.h: typemap override of attribute %s with %s\n\n", t, s);
        if (types.knames.find(s) == types.knames.end())
          types.knames.insert(s);
      }
    }
    else
    {
      s = types.tname(NULL, NULL, "xsd:string");
      if (!mflag)
      {
        if (**k == '"')
        {
          fprintf(stream, "// Imported attribute \"%s\" declared as %s.\n", *k, t);
        }
        else if (!iflag && !imported(*k))
        {
          fprintf(stream, "/// Built-in attribute \"%s\".\n", *k);
          fprintf(stream, "typedef %s %s;\n\n", s, t);
          const char *u = types.uri(*k);
          if (u && !types.uris[u])
          {
            fprintf(stream, schemaformat, types.nsprefix(types.prefix(*k), u), "namespace", u);
            fprintf(stream, "\n");
          }
        }
      }
      else if (**k == '"')
      {
        fprintf(stream, "//  Imported attribute \"%s\" declared as %s.\n\n", *k, t);
      }
      else
      {
        fprintf(stream, "//  xsd.h: should define attribute %s\n\n", t);
      }
      types.deftname(TYPEDEF, false, types.is_ptr(NULL, NULL, "xsd:string"), "_", NULL, *k, s);
    }
  }
  // produce types
  // define class/struct types first
  if (!cflag)
    banner("Forward Declarations");
  if (definitions.types)
  {
    comment("Definitions", defs, "types", definitions.types->documentation);
    fprintf(stream, "\n");
    for (std::vector<xs__schema*>::const_iterator schema4 = definitions.types->xs__schema_.begin(); schema4 != definitions.types->xs__schema_.end(); ++schema4)
    {
      if (vflag)
        fprintf(stderr, "\nDefining types in %s\n", (*schema4)->targetNamespace);
      for (std::vector<xs__complexType>::const_iterator complexType = (*schema4)->complexType.begin(); complexType != (*schema4)->complexType.end(); ++complexType)
        types.define((*schema4)->targetNamespace, NULL, *complexType);
      if (vflag)
        fprintf(stderr, "\nDefining elements in %s\n", (*schema4)->targetNamespace);
      for (std::vector<xs__element>::const_iterator element = (*schema4)->element.begin(); element != (*schema4)->element.end(); ++element)
      {
        if (!(*element).type && !(*element).abstract)
        {
          if ((*element).complexTypePtr())
          {
            types.define((*schema4)->targetNamespace, (*element).name, *(*element).complexTypePtr());
          }
          else if (!(*element).simpleTypePtr())
          {
            fprintf(stream, "/// @brief Top-level root element \"%s\":%s.\n", (*schema4)->targetNamespace, (*element).name);
            if (gflag)
            {
              const char *t = types.deftname(TYPEDEF, false, false, "_", (*schema4)->targetNamespace, (*element).name, NULL);
              if (t)
                fprintf(stream, "typedef _XML %s;\n\n", t);
              else
                fprintf(stream, "// Element definition intentionally left blank.\n\n");
              types.ptrtypemap[t] = types.usetypemap[t] = "_XML";
            }
            else
            {
              const char *s = types.cname("_", (*schema4)->targetNamespace, (*element).name);
              types.ptrtypemap[s] = types.usetypemap[s] = "_XML";
              if (!Lflag)
                fprintf(stream, "/// @note Use wsdl2h option -g to auto-generate a top-level root element declaration.\n\n");
            }
          }
        }
      }
    }  
    // visit types with lowest base level first
    int baseLevel = 1;
    bool found;
    bool inheritance = false;
    do
    {
      found = (baseLevel == 1);
      for (std::vector<xs__schema*>::iterator schema = definitions.types->xs__schema_.begin(); schema != definitions.types->xs__schema_.end(); ++schema)
      {
        if (found)
          banner("Schema Types and Top-Level Elements and Attributes", (*schema)->targetNamespace);
        for (std::vector<xs__simpleType>::iterator simpleType = (*schema)->simpleType.begin(); simpleType != (*schema)->simpleType.end(); ++simpleType)
        {
          if ((*simpleType).baseLevel() == baseLevel)
          {
            found = true;
            types.gen((*schema)->targetNamespace, NULL, *simpleType, false, false);
          }
        }
        for (std::vector<xs__element>::iterator element = (*schema)->element.begin(); element != (*schema)->element.end(); ++element)
        {
          if (!(*element).type && (*element).simpleTypePtr() && (*element).simpleTypePtr()->baseLevel() == baseLevel)
          {
            found = true;
            if ((*element).type)
              fprintf(stream, "/// @brief Top-level root element \"%s\":%s of simpleType %s.\n", (*schema)->targetNamespace, (*element).name, (*element).type);
            types.document((*element).annotation);
            types.gen((*schema)->targetNamespace, (*element).name, *(*element).simpleTypePtr(), false, false);
          }
          if (!(*element).type && (*element).complexTypePtr() && (*element).complexTypePtr()->baseLevel() == baseLevel)
            found = true;
        }
        for (std::vector<xs__attribute>::const_iterator attribute = (*schema)->attribute.begin(); attribute != (*schema)->attribute.end(); ++attribute)
        {
          if (!(*attribute).type && (*attribute).simpleTypePtr() && (*attribute).simpleTypePtr()->baseLevel() == baseLevel)
          {
            found = true;
            if ((*attribute).type)
              fprintf(stream, "/// @brief Top-level attribute \"%s\":%s of simpleType %s.\n", (*schema)->targetNamespace, (*attribute).name, (*attribute).type);
            types.document((*attribute).annotation);
            types.gen((*schema)->targetNamespace, (*attribute).name, *(*attribute).simpleTypePtr(), false, false); // URI = NULL won't generate type in schema (type without namespace qualifier)
          }
        }
        for (std::vector<xs__complexType>::iterator complexType = (*schema)->complexType.begin(); complexType != (*schema)->complexType.end(); ++complexType)
        {
          if ((*complexType).baseLevel() == baseLevel)
            found = true;
        }
      }
      if (found && baseLevel > 1)
        inheritance = true;
      ++baseLevel;
    } while (found);
    if (cflag && !Fflag && !fflag && inheritance && !wflag)
      fprintf(stderr, "\nWarning: ignoring type inheritance by default for C, use option -F to generate struct declarations with simulated inheritance using transient pointer members pointing to derived types to serialize derived types as elements annotated by xsi:type attributes in XML.\n");
    // generate wrapper structs/classes for built-in XSD types with options -p and -F
    if (pflag && Fflag)
    {
      banner("Wrappers for built-in types with their derived types");
      for (SetOfString::const_iterator i = definitions.builtinTypes().begin(); i != definitions.builtinTypes().end(); ++i)
      {
        if (strcmp(*i, "xs:anyType") && strcmp(*i, "xs:anySimpleType"))
        {
          const char *s = types.wname(NULL, NULL, *i, NOLOOKUP);
          fprintf(stream, "/// Wrapper %s for built-in type \"%s\".\n///\n", s, *i);
          if (cflag)
          {
            if (!Lflag)
              fprintf(stream, "/// @note Use option -P to remove this struct.\n");
            fprintf(stream, "struct %s\n{\n", s);
          }
          else
          {
            if (!Lflag)
              fprintf(stream, "/// @note Call virtual method soap_type() generated by soapcpp2 to check runtime type is SOAP_TYPE_%s or a derived type.  Use option -P to remove this class.\n", s);
            fprintf(stream, "class %s\n{ public:\n", s);
          }
          fprintf(stream, elementformat, types.tname(NULL, NULL, *i), "__item");
          fprintf(stream, ";\n");
          for (MapOfStringToString::const_iterator j = definitions.builtinTypeDerivations().begin(); j != definitions.builtinTypeDerivations().end(); ++j)
          {
            if (!strcmp(*i, (*j).second))
            {
              fprintf(stream, "/// A transient pointer to a derived type value that replaces the value of this base type %s when non-NULL\n", s);
              fprintf(stream, derivedformat, types.tnamenoptr(NULL, NULL, (*j).first), types.aname(NULL, NULL, (*j).first));
              fprintf(stream, "\n");
            }
          }
          if (cflag)
          {
            fprintf(stream, "};\n\n");
          }
          else
          {
            fprintf(stream, pointerformat, "struct soap", soap_context);
            fprintf(stream, ";\n");
            fprintf(stream, "};\n\n");
          }
        }
      }
      fprintf(stream, "/// Wrapper xsd__anyType_ for built-in XSD type \"xs:anyType\".\n///\n");
      if (cflag)
      {
        if (!Lflag)
          fprintf(stream, "/// @note Use option -P to remove this struct.\n");
        fprintf(stream, "struct xsd__anyType_\n{\n");
      }
      else
      {
        if (!Lflag)
          fprintf(stream, "/// @note Call virtual method soap_type() generated by soapcpp2 to check runtime type is SOAP_TYPE_xsd__anyType_ or a derived type.  Use option -P to remove this class.\n");
        fprintf(stream, "class xsd__anyType_\n{ public:\n");
      }
      if (dflag)
        fprintf(stream, elementformat, "xsd__anyType", "__item");
      else
        fprintf(stream, elementformat, "_XML", "__item");
      fprintf(stream, ";\n");
      for (SetOfString::const_iterator i = definitions.builtinTypes().begin(); i != definitions.builtinTypes().end(); ++i)
      {
        if (strcmp(*i, "xs:anyType") && strcmp(*i, "xs:anySimpleType"))
        {
          fprintf(stream, "/// A transient pointer to a derived type value that replaces the value of this root type xsd__anyType when non-NULL\n");
          fprintf(stream, derivedformat, types.wname(NULL, NULL, *i, LOOKUP), types.aname(NULL, NULL, *i));
          fprintf(stream, "\n");
        }
      }
      for (std::vector<xs__schema*>::const_iterator schema = definitions.types->xs__schema_.begin(); schema != definitions.types->xs__schema_.end(); ++schema)
      {
        for (std::vector<xs__simpleType>::const_iterator simpleType = (*schema)->simpleType.begin(); simpleType != (*schema)->simpleType.end(); ++simpleType)
        {
          if ((*simpleType).name
           && !(*simpleType).restriction
           && !(*simpleType).list
           && !(*simpleType).union_)
          {
            fprintf(stream, "/// A transient pointer to a derived type value that replaces the value of this root type xsd__anyType when non-NULL\n");
            fprintf(stream, derivedformat, types.tnamenoptr(NULL, (*schema)->targetNamespace, (*simpleType).name), types.aname(NULL, (*schema)->targetNamespace, (*simpleType).name));
            fprintf(stream, "\n");
          }
        }
        for (std::vector<xs__complexType>::const_iterator complexType = (*schema)->complexType.begin(); complexType != (*schema)->complexType.end(); ++complexType)
        {
          if ((*complexType).name
           && !(*complexType).abstract
           && !((*complexType).simpleContent && ((*complexType).simpleContent->restriction || (*complexType).simpleContent->extension))
           && !((*complexType).complexContent && ((*complexType).complexContent->restriction || (*complexType).complexContent->extension)))
          {
            fprintf(stream, "/// A transient pointer to a derived type value that replaces the value of this root type xsd__anyType when non-NULL\n");
            fprintf(stream, derivedformat, types.tnamenoptr(NULL, (*schema)->targetNamespace, (*complexType).name), types.aname(NULL, (*schema)->targetNamespace, (*complexType).name));
            fprintf(stream, "\n");
          }
        }
      }
      if (!cflag)
      {
        fprintf(stream, pointerformat, "struct soap", soap_context);
        fprintf(stream, ";\n");
      }
      fprintf(stream, "};\n\n");
    }
    // generate complex type defs. Problem: what if a simpleType restriction/extension depends on a complexType simpleContent restriction/extension?
    int maxLevel = baseLevel;
    for (baseLevel = 1; baseLevel < maxLevel; ++baseLevel)
    {
      for (std::vector<xs__schema*>::iterator schema = definitions.types->xs__schema_.begin(); schema != definitions.types->xs__schema_.end(); ++schema)
      {
        if (baseLevel == 1)
          banner("Schema Complex Types and Top-Level Elements", (*schema)->targetNamespace);
        for (std::vector<xs__complexType>::iterator complexType = (*schema)->complexType.begin(); complexType != (*schema)->complexType.end(); ++complexType)
        {
          if ((*complexType).baseLevel() == baseLevel)
            types.gen((*schema)->targetNamespace, NULL, *complexType, false);
        }
        for (std::vector<xs__element>::iterator element = (*schema)->element.begin(); element != (*schema)->element.end(); ++element)
        {
          if (!(*element).type && (*element).complexTypePtr() && (*element).complexTypePtr()->baseLevel() == baseLevel)
          {
            fprintf(stream, "/// @brief Top-level root element \"%s\":%s\n", (*schema)->targetNamespace, (*element).name);
            types.document((*element).annotation);
            types.gen((*schema)->targetNamespace, (*element).name, *(*element).complexTypePtr(), false);
          }
        }
      }
    }
    for (std::vector<xs__schema*>::iterator schema = definitions.types->xs__schema_.begin(); schema != definitions.types->xs__schema_.end(); ++schema)
    {
      for (std::vector<xs__simpleType>::iterator simpleType = (*schema)->simpleType.begin(); simpleType != (*schema)->simpleType.end(); ++simpleType)
      {
        if ((*simpleType).baseLevel() <= 0)
        {
          const char *t = types.deftname(TYPEDEF, false, false, NULL, (*schema)->targetNamespace, (*simpleType).name, NULL);
          fprintf(stderr, "\nError: circular restriction/extension <xs:simpleType name=\"%s\"/>\n", (*simpleType).name ? (*simpleType).name : "");
          fprintf(stream, "\n/// @todo !FIXME! @warning %s defines a simpleType with circular restriction/extension.\n", (*simpleType).name ? (*simpleType).name : "");
          fprintf(stream, "typedef _XML %s;\n\n", t);
          types.ptrtypemap[t] = types.usetypemap[t] = "_XML";
        }
      }
      for (std::vector<xs__complexType>::iterator complexType = (*schema)->complexType.begin(); complexType != (*schema)->complexType.end(); ++complexType)
      {
        if ((*complexType).baseLevel() <= 0)
        {
          fprintf(stderr, "\nError: circular restriction/extension <xs:complexType name=\"%s\"/>\n", (*complexType).name ? (*complexType).name : "");
          fprintf(stream, "\n\n/// @todo !FIXME! @warning %s defines a complexType with circular restriction/extension.\n", (*complexType).name ? (*complexType).name : "");
          if (cflag)
            fprintf(stream, "struct %s { };\n\n", types.cname(NULL, (*schema)->targetNamespace, (*complexType).name));
          else
            fprintf(stream, "class %s { };\n\n", types.cname(NULL, (*schema)->targetNamespace, (*complexType).name));
        }
      }
    }
    // option to consider: generate local complexTypes iteratively
    /*
    for (MapOfStringToType::const_iterator local = types.locals.begin(); local != types.locals.end(); ++local)
    {
    types.gen(NULL, (*local).first, *(*local).second);
    }
    */
    for (std::vector<xs__schema*>::iterator schema = definitions.types->xs__schema_.begin(); schema != definitions.types->xs__schema_.end(); ++schema)
    {
      if (vflag)
        fprintf(stderr, "\nGenerating elements in %s\n", (*schema)->targetNamespace);
      banner("Additional Top-Level Elements", (*schema)->targetNamespace);
      for (std::vector<xs__element>::iterator element = (*schema)->element.begin(); element != (*schema)->element.end(); ++element)
      {
        if ((*element).name && (*element).type && !(*element).abstract)
        {
          fprintf(stream, "/// @brief Top-level root element \"%s\":%s of type %s.\n", (*schema)->targetNamespace, (*element).name, (*element).type);
          types.document((*element).annotation);
          if (Oflag > 1 && !(*element).is_used())
          {
            fprintf(stream, "// Optimization: element not used and removed\n\n");
          }
          else
          {
            if (!types.is_defined("_", (*schema)->targetNamespace, (*element).name))
            {
              if (gflag)
              {
                const char *s = types.tnamenoptr(NULL, (*schema)->targetNamespace, (*element).type);
                const char *t = types.deftname(TYPEDEF, false, false, "_", (*schema)->targetNamespace, (*element).name, s);
                fprintf(stream, "typedef %s %s;\n\n", s, t);
              }
              else if (!Lflag)
              {
                fprintf(stream, "/// @note Use wsdl2h option -g to auto-generate a top-level root element declaration.\n\n");
              }
            }
            else
            {
              const char *s = types.cname("_", (*schema)->targetNamespace, (*element).name);
              const char *t = types.deftypemap[s];
              if (t && *t)
              {
                fprintf(stream, "/// Imported element \"%s\" from typemap %s.\n", s, mapfile ? mapfile : "");
                types.format(t);
              }
              else
              {
                fprintf(stream, "// \"%s\" element definition intentionally left blank.\n", types.cname("_", (*schema)->targetNamespace, (*element).name));
              }
            }
          }
        }
      }
      if (vflag)
        fprintf(stderr, "\nGenerating attributes in %s\n", (*schema)->targetNamespace);
      banner("Additional Top-Level Attributes", (*schema)->targetNamespace);
      for (std::vector<xs__attribute>::iterator attribute = (*schema)->attribute.begin(); attribute != (*schema)->attribute.end(); ++attribute)
      {
        if ((*attribute).name && (*attribute).type)
        {
          fprintf(stream, "/// @brief Top-level attribute \"%s\":%s of simpleType %s.\n", (*schema)->targetNamespace, (*attribute).name, (*attribute).type);
          types.document((*attribute).annotation);
          if (Oflag > 1 && !(*attribute).is_used())
          {
            fprintf(stream, "// Optimization: attribute not used and removed\n\n");
          }
          else
          {
            if (!types.is_defined("_", (*schema)->targetNamespace, (*attribute).name))
            {
              if (gflag)
              {
                const char *s = types.tnamenoptr(NULL, (*schema)->targetNamespace, (*attribute).type);
                const char *t = types.deftname(TYPEDEF, false, false, "_", (*schema)->targetNamespace, (*attribute).name, s);
                fprintf(stream, "typedef %s %s;\n\n", s, t);
              }
              else if (!Lflag)
              {
                fprintf(stream, "/// @note Use wsdl2h option -g to auto-generate a top-level attribute declaration.\n\n");
              }
            }
            else
            {
              const char *s = types.cname("_", (*schema)->targetNamespace, (*attribute).name);
              const char *t = types.deftypemap[s];
              if (t && *t)
              {
                fprintf(stream, "/// Imported attribute \"%s\" from typemap %s.\n", s, mapfile ? mapfile : "");
                types.format(t);
              }
              else
                fprintf(stream, "// \"%s\" attribute definition intentionally left blank.\n", types.cname("_", (*schema)->targetNamespace, (*attribute).name));
            }
          }
        }
      }
    }
  }
  if (vflag)
    fprintf(stderr, "\nCollecting service bindings\n");
  collect(definitions);
  if (!services.empty())
  {
    banner("Services");
    if (soap12)
      fprintf(stream, "// This service supports SOAP 1.2 namespaces:\n#import \"soap12.h\"\n");
    for (MapOfStringToService::const_iterator service1 = services.begin(); service1 != services.end(); ++service1)
    {
      Service *sv = (*service1).second;
      if (sv)
      {
        fprintf(stream, "\n");
        if (sv->name)
          fprintf(stream, serviceformat, sv->prefix, "name", sv->name, "");
        if (sv->type)
          fprintf(stream, serviceformat, sv->prefix, "type", sv->type, "");
        for (SetOfString::const_iterator port = sv->location.begin(); port != sv->location.end(); ++port)
          fprintf(stream, serviceformat, sv->prefix, "port", (*port), "");
        if (sv->URI && *sv->URI)
          fprintf(stream, serviceformat, sv->prefix, "namespace", sv->URI, "");
        if (sv->transport)
          fprintf(stream, serviceformat, sv->prefix, "transport", sv->transport, "");
      }
    }
    fprintf(stream, "\n/** @mainpage %s Definitions\n", defs);
    if (definitions.version)
    {
      section(defs, "_version Version", NULL);
      text(definitions.version);
    }
    if (definitions.documentation)
    {
      section(defs, "_documentation Documentation", NULL);
      text(definitions.documentation);
    }
    if (definitions.types && definitions.types->documentation)
    {
      section(defs, "_types Schema Type Information", NULL);
      text(definitions.types->documentation);
    }
    section(defs, "_bindings Service Bindings", NULL);
    for (MapOfStringToService::const_iterator service2 = services.begin(); service2 != services.end(); ++service2)
    {
      Service *sv = (*service2).second;
      if (sv && sv->name)
        fprintf(stream, "\n  - @ref %s\n", sv->name);
    }
    section(defs, "_more More Information", NULL);
    fprintf(stream, "\n  - @ref page_notes \"Notes\"\n");
    if (!definitions.vprop__property.empty())
      fprintf(stream, "\n  - @ref page_bpel \"BPEL 2.0 Variables\"\n");
    fprintf(stream, "\n  - @ref page_XMLDataBinding \"XML Data Binding\"\n");
    if (!jflag)
      fprintf(stream, "\n  - @ref SOAP_ENV__Header \"SOAP Header Content\" (when applicable)\n");
    if (!jflag)
      fprintf(stream, "\n  - @ref SOAP_ENV__Detail \"SOAP Fault Detail Content\" (when applicable)\n");
    fprintf(stream, "\n\n*/\n");
    for (MapOfStringToService::const_iterator service3 = services.begin(); service3 != services.end(); ++service3)
    {
      Service *sv = (*service3).second;
      if (sv && sv->name)
      {
        fprintf(stream, "\n/** ");
        page(sv->name, " Binding", sv->name);
        for (MapOfStringToString::const_iterator service_doc = sv->service_documentation.begin(); service_doc != sv->service_documentation.end(); ++service_doc)
        {
          const char *name = types.aname(NULL, NULL, (*service_doc).first);
          section(name, "_service Service Documentation", (*service_doc).first);
          text((*service_doc).second);
        }
        for (MapOfStringToString::const_iterator port_doc = sv->port_documentation.begin(); port_doc != sv->port_documentation.end(); ++port_doc)
        {
          const char *name = types.aname(NULL, NULL, (*port_doc).first);
          section(name, "_port Port Documentation", (*port_doc).first);
          text((*port_doc).second);
        }
        for (MapOfStringToString::const_iterator binding_doc = sv->binding_documentation.begin(); binding_doc != sv->binding_documentation.end(); ++binding_doc)
        {
          const char *name = types.aname(NULL, NULL, (*binding_doc).first);
          section(name, "_binding Binding Documentation", (*binding_doc).first);
          text((*binding_doc).second);
        }
        section(sv->name, "_operations Operations of Binding", sv->name);
        for (std::vector<Operation*>::const_iterator op = sv->operation.begin(); op != sv->operation.end(); ++op)
        {
          if (*op && (*op)->input && (*op)->input_name)
            fprintf(stream, "\n  - @ref %s\n", (*op)->input_name);
          else if (*op && (*op)->output_name)
            fprintf(stream, "\n  - @ref %s\n", (*op)->output_name);
        }
        if (!sv->role.empty())
        {
          section(sv->name, "_plnk BPEL 2.0 Partner Links for Binding", sv->name);
          gen_plnk(*sv);
        }
        section(sv->name, "_ports Default endpoints of Binding", sv->name);
        for (SetOfString::const_iterator port = sv->location.begin(); port != sv->location.end(); ++port)
          fprintf(stream, "\n  - %s\n", *port);
        if (!sv->policy.empty())
        {
          section(sv->name, "_policy Policy of Binding", sv->name);
          fprintf(stream, "\nSee Section @ref %s_policy_enablers\n", sv->name);
          gen_policy(*sv, sv->policy, "service endpoint ports", types);
        }
        if (!service_prefix && !Lflag)
        {
          if (binding_count > 1)
            fprintf(stream, "\n@note Multiple service bindings collected as one, use wsdl2h option -Nname to produce a separate service for each binding\n");
          else
            fprintf(stream, "\n@note Use wsdl2h option -Nname to change the service binding prefix name\n");
        }
        fprintf(stream, "\n\n*/\n");
      }
    }
  }
  generate();
  if (definitions.types)
  {
    gen_vprop(definitions, types);
    banner("XML Data Binding");
    fprintf(stream, "\n/** ");
    page("page_XMLDataBinding", " XML Data Binding", NULL);
    fprintf(stream, "\nSOAP/XML services use data bindings that are contractually bound by WSDLs and\nare auto-generated by wsdl2h and soapcpp2 (see Service Bindings). Plain data\nbindings are adopted from XML schemas as part of the WSDL types section or when\nrunning wsdl2h on a set of schemas to produce non-SOAP-based XML data bindings.\n");
    if (!Lflag)
      fprintf(stream, "\n@note The following readers and writers are C/C++ data type (de)serializers\nauto-generated by wsdl2h and soapcpp2. Run soapcpp2 on this file to generate the\n(de)serialization code, which is stored in soapC.c[pp]. Include \"soapH.h\" in\nyour code to import these data type and function declarations. Only use the\nsoapcpp2-generated files in your project build. Do not include the wsdl2h-\ngenerated .h file in your code.\n\n@note Data can be read and deserialized from:\n  - an int file descriptor, using soap->recvfd = fd\n  - a socket, using soap->socket = (int)...\n  - a C++ stream (istream, stringstream), using soap->is = (istream*)...\n  - a C string, using soap->is = (const char*)...\n  - any input, using the soap->frecv() callback\n\n@note Data can be serialized and written to:\n  - an int file descriptor, using soap->sendfd = (int)...\n  - a socket, using soap->socket = (int)...\n  - a C++ stream (ostream, stringstream), using soap->os = (ostream*)...\n  - a C string, using soap->os = (const char**)...\n  - any output, using the soap->fsend() callback\n\n@note The following options are available for (de)serialization control:\n  - soap->encodingStyle = NULL; to remove SOAP 1.1/1.2 encodingStyle\n  - soap_set_mode(soap, SOAP_XML_TREE); XML without id-ref (no cycles!)\n  - soap_set_mode(soap, SOAP_XML_GRAPH); XML with id-ref (including cycles)\n  - soap_set_namespaces(soap, struct Namespace *nsmap); to set xmlns bindings\n\n");
    for (std::vector<xs__schema*>::const_iterator schema5 = definitions.types->xs__schema_.begin(); schema5 != definitions.types->xs__schema_.end(); ++schema5)
    {
      const char *prefix = types.nsprefix(NULL, (*schema5)->targetNamespace);
      fprintf(stream, "\n*/\n\n/**\n\n@section %s Top-level root elements of schema \"%s\"\n", prefix ? prefix : "default", (*schema5)->targetNamespace);
      for (std::vector<xs__element>::const_iterator element = (*schema5)->element.begin(); element != (*schema5)->element.end(); ++element)
      {
        if (prefix && *prefix)
          fprintf(stream, "\n  - <%s:%s> ", prefix, (*element).name);
        else
          fprintf(stream, "\n  - <%s> ", (*element).name);
        const char *cname = types.cname("_", (*schema5)->targetNamespace, (*element).name);
        if (types.is_defined("_", (*schema5)->targetNamespace, (*element).name))
        {
          fprintf(stream, "@ref %s\n", cname);
          if (!Lflag)
          {
            const char *pname = types.pname(true, false, "_", (*schema5)->targetNamespace, (*element).name);
            fprintf(stream, "    @code\n    // Reader (returns SOAP_OK on success):\n    soap_read_%s(struct soap*, %s);\n    // Writer (returns SOAP_OK on success):\n    soap_write_%s(struct soap*, %s);\n", cname, pname, cname, pname);
            fprintf(stream, "    // REST GET (returns SOAP_OK on success):\n    soap_GET_%s(struct soap*, const char *URL, %s);\n    // REST PUT (returns SOAP_OK on success):\n    soap_PUT_%s(struct soap*, const char *URL, %s);\n", cname, pname, cname, pname);
            fprintf(stream, "    // REST POST (returns SOAP_OK on success):\n    soap_POST_send_%s(struct soap*, const char *URL, %s);\n    soap_POST_recv_%s(struct soap*, %s);\n    @endcode\n", cname, pname, cname, pname);
          }
        }
        else
        {
          fprintf(stream, "(use wsdl2h option -g to auto-generate type %s)\n", cname);
        }
      }
    }
    fprintf(stream, "\n*/\n");
  }
  if (cppnamespace)
    fprintf(stream, "\n} // namespace %s\n", cppnamespace);
  fprintf(stream, "\n/* End of %s */\n", outfile ? outfile : "file");
}

void Definitions::generate()
{
  MapOfStringToMessage headers;
  MapOfStringToMessage headerfaults;
  MapOfStringToMessage faults;
  const char *t;
  for (MapOfStringToService::const_iterator service1 = services.begin(); service1 != services.end(); ++service1)
  {
    if ((*service1).second)
    {
      for (MapOfStringToMessage::const_iterator header = (*service1).second->header.begin(); header != (*service1).second->header.end(); ++header)
        headers[(*header).first] = (*header).second;
      for (MapOfStringToMessage::const_iterator headerfault = (*service1).second->headerfault.begin(); headerfault != (*service1).second->headerfault.end(); ++headerfault)
        headerfaults[(*headerfault).first] = (*headerfault).second;
      for (MapOfStringToMessage::const_iterator fault = (*service1).second->fault.begin(); fault != (*service1).second->fault.end(); ++fault)
        faults[(*fault).first] = (*fault).second;
    }
  }
  // Generate SOAP Header definition
  t = types.deftypemap["SOAP_ENV__Header"];
  if (t && *t)
  {
    banner("Custom SOAP Header");
    types.format(t);
  }
  else if (!jflag && (!headers.empty() || !headerfaults.empty()))
  {
    banner("SOAP Header");
    fprintf(stream, "/**\n\nThe SOAP Header is part of the gSOAP context and its content is accessed\nthrough the soap.header variable. You may have to set the soap.actor variable\nto serialize SOAP Headers with SOAP-ENV:actor or SOAP-ENV:role attributes.\nUse option -j to remove entire SOAP Header definition.\nUse option -k to remove the mustUnderstand qualifiers.\n\n*/\n");
    fprintf(stream, "mutable struct SOAP_ENV__Header\n{\n");
    for (MapOfStringToMessage::const_iterator header = headers.begin(); header != headers.end(); ++header)
    {
      if ((*header).second->use == encoded && (*header).second->URI && *(*header).second->URI)
        fprintf(stream, schemaformat, types.nsprefix(NULL, (*header).second->URI), "namespace", (*header).second->URI);
      comment("Header", (*header).first, "WSDL", (*header).second->ext_documentation);
      comment("Header", (*header).first, "SOAP", (*header).second->documentation);
      if ((*header).second->mustUnderstand && !kflag)
      {
        fprintf(stream, elementformat, "mustUnderstand", "// must be understood by receiver");
        fprintf(stream, "\n");
      }
      if ((*header).second->part && (*header).second->part->elementPtr())
      {
        fprintf(stream, "/// \"%s\" SOAP Header part element.\n", (*header).second->part->name);
        if ((*header).second->part->elementPtr()->type && (*header).second->part->element)
          fprintf(stream, elementformat, types.pname(true, false, NULL, NULL, (*header).second->part->elementPtr()->type), types.aname(NULL, NULL, (*header).second->part->element));
        else if ((*header).second->part->element)
          fprintf(stream, elementformat, types.pname(true, false, "_", NULL, (*header).second->part->element), types.aname(NULL, NULL, (*header).second->part->element));
        else
          fprintf(stream, elementformat, types.pname(true, false, "_", NULL, (*header).second->part->elementPtr()->name), (*header).first);
        fprintf(stream, ";\n");
      }
      else if ((*header).second->part && (*header).second->part->type)
      {
        fprintf(stream, "/// \"%s\" SOAP Header part type.\n", (*header).second->part->type);
        fprintf(stream, elementformat, types.pname(true, false, NULL, NULL, (*header).second->part->type), types.aname(NULL, (*header).second->URI, (*header).second->part->name));
        fprintf(stream, ";\n");
      }
      else if ((*header).second->element)
      {
        fprintf(stream, "/// \"%s\" SOAP Header element.", (*header).second->name);
        (*header).second->generate(types, ";", false, true, false, true, false);
        fprintf(stream, "\n");
      }
      else
      {
        if ((*header).second->part && (*header).second->part->element)
          fprintf(stream, elementformat, types.pname(true, false, "_", NULL, (*header).second->part->element), (*header).first);
        else
          fprintf(stream, pointerformat, (*header).first, (*header).first);
        fprintf(stream, ";\t///< @todo Please check element name and type (imported type)\n");
      }
    }
    for (MapOfStringToMessage::const_iterator headerfault = headerfaults.begin(); headerfault != headerfaults.end(); ++headerfault)
    {
      if ((*headerfault).second->use == encoded && (*headerfault).second->URI && *(*headerfault).second->URI)
        fprintf(stream, schemaformat, types.nsprefix(NULL, (*headerfault).second->URI), "namespace", (*headerfault).second->URI);
      comment("HeaderFault", (*headerfault).first, "WSDL", (*headerfault).second->ext_documentation);
      comment("HeaderFault", (*headerfault).first, "SOAP", (*headerfault).second->documentation);
      if ((*headerfault).second->mustUnderstand && !kflag)
      {
        fprintf(stream, elementformat, "mustUnderstand", "// must be understood by receiver");
        fprintf(stream, "\n");
      }
      if ((*headerfault).second->part && (*headerfault).second->part->elementPtr())
      {
        fprintf(stream, "/// \"%s\" SOAP HeaderFault part element optionally contains fault details.\n", (*headerfault).second->part->name);
        if ((*headerfault).second->part->elementPtr()->type && (*headerfault).second->part->element)
          fprintf(stream, elementformat, types.pname(true, false, NULL, NULL, (*headerfault).second->part->elementPtr()->type), types.aname(NULL, NULL, (*headerfault).second->part->element));
        else if ((*headerfault).second->part->element)
          fprintf(stream, elementformat, types.pname(true, false, "_", NULL, (*headerfault).second->part->element), types.aname(NULL, NULL, (*headerfault).second->part->element));
        else
          fprintf(stream, elementformat, types.pname(true, false, "_", NULL, (*headerfault).second->part->elementPtr()->name), (*headerfault).first);
        fprintf(stream, ";\n");
      }
      else if ((*headerfault).second->part && (*headerfault).second->part->type)
      {
        fprintf(stream, "/// \"%s\" SOAP HeaderFault part type optionally contains fault details.\n", (*headerfault).second->part->type);
        fprintf(stream, elementformat, types.pname(true, false, NULL, NULL, (*headerfault).second->part->type), types.aname(NULL, (*headerfault).second->URI, (*headerfault).second->part->name));
        fprintf(stream, ";\n");
      }
      else if ((*headerfault).second->element)
      {
        fprintf(stream, "/// \"%s\" SOAP HeaderFault element optionally contains fault details.", (*headerfault).second->name);
        (*headerfault).second->generate(types, ";", false, true, false, true, false);
        fprintf(stream, "\n");
      }
      else
      {
        if ((*headerfault).second->part && (*headerfault).second->part->element)
          fprintf(stream, elementformat, types.pname(true, false, "_", NULL, (*headerfault).second->part->element), (*headerfault).first);
        else
          fprintf(stream, pointerformat, (*headerfault).first, (*headerfault).first);
        fprintf(stream, ";\t///< @todo Please check element name and type (imported type)\n");
      }
    }
    types.modify("SOAP_ENV__Header");
    fprintf(stream, "\n};\n\n");
  }
  // Generate Fault detail element definitions
  for (MapOfStringToMessage::const_iterator fault = faults.begin(); fault != faults.end(); ++fault)
  {
    if ((*fault).second->use == encoded)
    {
      fprintf(stream, schemaformat, types.nsprefix(NULL, (*fault).second->URI), "namespace", (*fault).second->URI);
      fprintf(stream, "/// SOAP Fault message \"%s\":%s.\n", (*fault).second->URI, (*fault).second->message->name);
      comment("Fault", (*fault).first, "WSDL", (*fault).second->ext_documentation);
      comment("Fault", (*fault).first, "SOAP", (*fault).second->documentation);
      if (cflag)
        fprintf(stream, "struct %s\n{", (*fault).first);
      else
        fprintf(stream, "class %s\n{ public:", (*fault).first);
      (*fault).second->generate(types, ";", false, true, false, true, false);
      if (!cflag && soap_context && *soap_context)
      {
        fprintf(stream, "\n");
        fprintf(stream, pointerformat, "struct soap", soap_context);
        fprintf(stream, ";");
      }
      fprintf(stream, "\n};\n\n");
      if (cflag)
        fprintf(stream, "typedef struct %s %s;\n\n", (*fault).first, (*fault).first);
    }
  }
  t = types.deftypemap["SOAP_ENV__Detail"];
  if (t && *t)
  {
    banner("Custom SOAP Fault Detail");
    types.format(t);
  }
  else if (!jflag && !faults.empty())
  {
    SetOfString fault_elements;
    banner("SOAP Fault Detail");
    fprintf(stream, "/**\n\nThe SOAP Fault is part of the gSOAP context and its content is accessed\nthrough the soap.fault->detail variable (SOAP 1.1) or the\nsoap.fault->SOAP_ENV__Detail variable (SOAP 1.2).\nUse wsdl2h option -j to omit these declarations.\n\n*/\n");
    fprintf(stream, "mutable struct SOAP_ENV__Detail\n{\n");
    if (dflag)
    {
      const char *t = types.tname(NULL, NULL, "xsd:anyAttribute");
      fprintf(stream, attributeformat, t, "__anyAttribute");
      fprintf(stream, ";\t///< Catch any attribute content in DOM.\n");
    }
    else
    {
      fprintf(stream, "/// xsd:anyAttribute omitted: to parse attribute content of the Detail element into DOM anyAttribute, use wsdl2h option -d.\n");
    }
    types.modify("SOAP_ENV__Detail");
    /* See below */
    /* The DOM representation is not desired since faultdetail is NULL.
       However, future options may reenable this feature (see keep code here).
    const char *t = types.tname(NULL, NULL, "xsd:any");
    fprintf(stream, elementformat, t, "__any");
    if (dflag)
      fprintf(stream, ";\t///< Catch any element content in DOM.\n");
    else
      fprintf(stream, ";\t///< Catch any element content in XML string.\n");
    */
    for (MapOfStringToMessage::const_iterator fault = faults.begin(); fault != faults.end(); ++fault)
    {
      comment("Fault", (*fault).first, "WSDL", (*fault).second->ext_documentation);
      comment("Fault", (*fault).first, "SOAP", (*fault).second->documentation);
      if ((*fault).second->message)
      {
        const char *name = (*fault).second->message->name;
        if (!name)
          name = "?";
        if ((*fault).second->use == literal)
        {
          for (std::vector<wsdl__part>::const_iterator part = (*fault).second->message->part.begin(); part != (*fault).second->message->part.end(); ++part)
          {
            if ((*part).elementPtr())
            {
              if (fault_elements.find((*part).element) == fault_elements.end())
              {
                fprintf(stream, "/// SOAP Fault message \"%s\" part \"%s\" element \"%s\".\n", name, (*part).name ? (*part).name : "", (*part).element ? (*part).element : "");
                if ((*part).elementPtr()->type)
                  fprintf(stream, elementformat, types.pname(true, false, NULL, NULL, (*part).elementPtr()->type), types.aname(NULL, (*fault).second->URI, (*part).element));
                else
                  fprintf(stream, elementformat, types.pname(true, false, "_", NULL, (*part).element), types.aname(NULL, (*fault).second->URI, (*part).element));
                fprintf(stream, ";\n");
                fault_elements.insert((*part).element);
              }
            }
            else if ((*part).name && (*part).type)
            {
              if (fault_elements.find((*part).name) == fault_elements.end())
              {
                fprintf(stream, schemaformat, types.nsprefix(NULL, (*fault).second->URI), "namespace", (*fault).second->URI);
                fprintf(stream, "/// SOAP Fault message \"%s\" part \"%s\" type \"%s\".\n", name, (*part).name, (*part).type);
                fprintf(stream, elementformat, types.pname(true, false, NULL, NULL, (*part).type), types.aname(NULL, (*fault).second->URI, (*part).name));
                fprintf(stream, ";\n");
                fault_elements.insert((*part).name);
              }
            }
            else
              fprintf(stream, "// Unknown SOAP Fault message \"%s\":%s part \"%s\"\n", (*fault).second->URI, name, (*part).name ? (*part).name : "");
          }
        }
        else
        {
          fprintf(stream, "/// SOAP Fault message \"%s\":%s\n", (*fault).second->URI, name);
          fprintf(stream, pointerformat, (*fault).first, types.aname(NULL, (*fault).second->URI, (*fault).second->message->name));
          fprintf(stream, ";\n");
        }
      }
      else if ((*fault).second->element)
      {
        if (fault_elements.find((*fault).second->name) == fault_elements.end())
        {
          fprintf(stream, "/// SOAP Fault message \"%s\":%s\n", (*fault).second->URI, (*fault).second->element->name);
          fprintf(stream, elementformat, types.pname(true, false, "_", (*fault).second->URI, (*fault).second->element->name), (*fault).second->name);
          fprintf(stream, ";\n");
          fault_elements.insert((*fault).second->name);
        }
      }
    }
    fprintf(stream, elementformat, "int", "__type");
    fprintf(stream, ";\t///< set to SOAP_TYPE_T for a serializable type T\n");
    fprintf(stream, pointerformat, "void", "fault");
    fprintf(stream, ";\t///< points to serializable value of type T or NULL\n");
    fprintf(stream, elementformat, "_XML", "__any");
    fprintf(stream, ";\t///< Catch any other content in XML string.\n");
    fprintf(stream, "};\n\n");
  }
  /* The SOAP Fault struct below is autogenerated by soapcpp2 (kept here for future mods)
  if (!mflag && !faults.empty())
  {
    fprintf(stream, "struct SOAP_ENV__Code\n{\n"); 
    fprintf(stream, elementformat, "_QName", "SOAP_ENV__Value");
    fprintf(stream, ";\n");
    fprintf(stream, pointerformat, "char", "SOAP_ENV__Node");
    fprintf(stream, ";\n");
    fprintf(stream, pointerformat, "char", "SOAP_ENV__Role");
    fprintf(stream, ";\n};\n");
    fprintf(stream, "struct SOAP_ENV__Detail\n{\n"); 
    fprintf(stream, elementformat, "int", "__type");
    fprintf(stream, ";\n");
    fprintf(stream, pointerformat, "void", "fault");
    fprintf(stream, ";\n");
    fprintf(stream, elementformat, "_XML", "__any");
    fprintf(stream, ";\n};\n");
    fprintf(stream, "struct SOAP_ENV__Fault\n{\n"); 
    fprintf(stream, elementformat, "_QName", "faultcode");
    fprintf(stream, ";\n");
    fprintf(stream, pointerformat, "char", "faultstring");
    fprintf(stream, ";\n");
    fprintf(stream, pointerformat, "char", "faultactor");
    fprintf(stream, ";\n");
    fprintf(stream, pointerformat, "struct SOAP_ENV__Detail", "detail");
    fprintf(stream, ";\n");
    fprintf(stream, pointerformat, "struct SOAP_ENV__Code", "SOAP_ENV__Code");
    fprintf(stream, ";\n");
    fprintf(stream, pointerformat, "char", "SOAP_ENV__Reason");
    fprintf(stream, ";\n");
    fprintf(stream, pointerformat, "struct SOAP_ENV__Detail", "SOAP_ENV__Detail");
    fprintf(stream, ";\n};\n");
  }
  */
  for (MapOfStringToService::const_iterator service2 = services.begin(); service2 != services.end(); ++service2)
    if ((*service2).second)
      (*service2).second->generate(types);
}

////////////////////////////////////////////////////////////////////////////////
//
//      Service methods
//
////////////////////////////////////////////////////////////////////////////////

Service::Service()
{
  prefix = NULL;
  URI = NULL;
  name = NULL;
  type = NULL;
  transport = NULL;
}

void Service::generate(Types& types)
{
  const char *method_name;
  banner("Service Binding", name);
  for (std::vector<Operation*>::const_iterator op2 = operation.begin(); op2 != operation.end(); ++op2)
  {
    if (*op2 && ((*op2)->input || bflag))
    {
      bool flag = false, anonymous = ((*op2)->style != document && (*op2)->parameterOrder != NULL);
      if (!(*op2)->input)
        method_name = (*op2)->output_name;
      else
        method_name = (*op2)->input_name;
      banner("Service Operation", method_name);
      if ((*op2)->output && (*op2)->output_name)
      {
        if ((*op2)->style == document)
          flag = ((*op2)->output->element || ((*op2)->output->message && (*op2)->output->message->part.size() == 1));
        else if (!wflag)
          flag = ((*op2)->output->element || ((*op2)->output->message && (*op2)->output->use == encoded && (*op2)->output->message->part.size() == 1 && !(*(*op2)->output->message->part.begin()).simpleTypePtr() && !(*(*op2)->output->message->part.begin()).complexTypePtr()));
        if (flag && (*op2)->input && (*op2)->output && (*op2)->input->message && (*op2)->output->message && !(*op2)->output->message->part.empty() && (*(*op2)->output->message->part.begin()).element)
          for (std::vector<wsdl__part>::const_iterator part = (*op2)->input->message->part.begin(); part != (*op2)->input->message->part.end(); ++part)
            if ((*part).element && !strcmp((*part).element, (*(*op2)->output->message->part.begin()).element))
              flag = false;
        if (!flag)
        {
          if (bflag)
            fprintf(stream, "/* soapcpp2 generates the following struct automatically for your use:\n");
          fprintf(stream, "/// Operation response struct \"%s\" of operation \"%s\".\n", (*op2)->output_name, method_name);
          fprintf(stream, "struct %s\n{", (*op2)->output_name);
          (*op2)->output->generate(types, ";", anonymous, true, false, false, (*op2)->is_rest);
          fprintf(stream, "\n};\n");
          if (bflag)
            fprintf(stream, "*/\n");
        }
      }
      fprintf(stream, "\n/** Operation \"%s\" of service binding \"%s\".\n", method_name, name);
      if ((*op2)->documentation)
        text((*op2)->documentation);
      if ((*op2)->operation_documentation)
        text((*op2)->operation_documentation);
      if ((*op2)->input && (*op2)->input->documentation)
      {
        fprintf(stream, "Input request:\n");
        text((*op2)->input->documentation);
      }
      if ((*op2)->input && (*op2)->input->ext_documentation)
      {
        fprintf(stream, "Input request:\n");
        text((*op2)->input->ext_documentation);
      }
      if ((*op2)->output)
      {
        if ((*op2)->output->documentation)
        {
          fprintf(stream, "Output response:\n");
          text((*op2)->output->documentation);
        }
        if ((*op2)->output->ext_documentation)
        {
          fprintf(stream, "Output response:\n");
          text((*op2)->output->ext_documentation);
        }
      }
      gen_policy(*this, (*op2)->policy, "operation", types);
      if ((*op2)->input)
      {
        gen_policy(*this, (*op2)->input->policy, "request message", types);
        if ((*op2)->input->content)
        {
          fprintf(stream, "\n  - Request message MIME content");
          if ((*op2)->input->content->type)
          {
            fprintf(stream, " type: ");
            text((*op2)->input->content->type);
          }
          if ((*op2)->input->content->type && !strcmp((*op2)->input->content->type, "application/x-www-form-urlencoded"))
            fprintf(stream, "\n    Use the httpform.c plugin to retrieve key-value pairs from the REST request\n    message form data at the server side (client side is automated).\n");
          else
            fprintf(stream, "\n    @todo This form of MIME content is not automatically handled.\n");
        }
      }
      if ((*op2)->output)
      {
        gen_policy(*this, (*op2)->output->policy, "response message", types);
        if ((*op2)->output->content)
        {
          fprintf(stream, "\n  - Response message MIME content");
          if ((*op2)->output->content->type)
          {
            fprintf(stream, " type: ");
            text((*op2)->output->content->type);
          }
          fprintf(stream, "\n    @todo This form of MIME content response is not automatically handled.\n    Use one-way request and implement code to parse response.\n");
        }
      }
      else
        fprintf(stream, "\n  - One-way service request message\n");
      if ((*op2)->mep)
        fprintf(stream, "\n  - SOAP MEP: \"%s\"\n", ((*op2)->mep));
      if ((*op2)->style == document)
        fprintf(stream, "\n  - %s %s messaging\n", (*op2)->protocol, (*op2)->input && (*op2)->input->content && (*op2)->input->content->type ? (*op2)->input->content->type : "document/literal style");
      else if ((*op2)->input)
      {
        if ((*op2)->input->use == literal)
          fprintf(stream, "\n  - %s literal messaging\n", (*op2)->protocol);
        else if ((*op2)->input->encodingStyle)
          fprintf(stream, "\n  - %s RPC encodingStyle=\"%s\"\n", (*op2)->protocol, (*op2)->input->encodingStyle);
        else
          fprintf(stream, "\n  - %s RPC encoded messaging\n", (*op2)->protocol);
      }
      if ((*op2)->output)
      {
        if (!(*op2)->input || (*op2)->input->use != (*op2)->output->use)
        {
          if ((*op2)->output->use == literal)
            fprintf(stream, "\n  - %s literal response messages\n", (*op2)->protocol);
          else if ((*op2)->output->encodingStyle)
            fprintf(stream, "\n  - %s RPC response encodingStyle=\"%s\"\n", (*op2)->protocol, (*op2)->output->encodingStyle);
          else
            fprintf(stream, "\n  - %s RPC encoded response messages\n", (*op2)->protocol);
        }
      }
      if (!location.empty())
      {
        fprintf(stream, "\n  - Default endpoints:\n");
        for (SetOfString::const_iterator port = location.begin(); port != location.end(); ++port)
          fprintf(stream, "    - %s\n", *port);
      }
      if ((*op2)->action)
      {
        if (*(*op2)->action)
          fprintf(stream, "\n  - Default SOAP action or REST location path:\n    - \"%s\"\n", (*op2)->action);
      }
      if ((*op2)->input)
      {
        if ((*op2)->input->action)
          fprintf(stream, "\n  - Addressing input action: \"%s\"\n", (*op2)->input->action);
      }
      if ((*op2)->output)
      {
        if ((*op2)->output->action)
          fprintf(stream, "\n  - Addressing output action: \"%s\"\n", (*op2)->output->action);
      }
      for (std::vector<Message*>::const_iterator infault = (*op2)->infault.begin(); infault != (*op2)->infault.end(); ++infault)
      {
        if ((*infault)->message)
        {
          if ((*infault)->use == literal)
          {
            for (std::vector<wsdl__part>::const_iterator part = (*infault)->message->part.begin(); part != (*infault)->message->part.end(); ++part)
            {
              if ((*part).element)
                fprintf(stream, "\n  - Input Fault: %s (literal)\n", (*part).element);
              else if ((*part).name && (*part).type)
                fprintf(stream, "\n  - Input Fault: %s (literal)\n", (*part).name);
            }
          }
          else if ((*infault)->message->name)
            fprintf(stream, "\n  - Input Fault: %s\n", (*infault)->name);
          if ((*infault)->message->name && (*infault)->action)
            fprintf(stream, "    - Input Fault addressing action: \"%s\"\n", (*infault)->action);
        }
        else if ((*infault)->name)
          fprintf(stream, "\n  - Input Fault: %s (literal)\n", (*infault)->name);
        if ((*infault)->body_parts)
          fprintf(stream, "    Input Fault code: %s\n", (*infault)->body_parts);
        text((*infault)->documentation);        
        text((*infault)->ext_documentation);    
        gen_policy(*this, (*infault)->policy, "fault message", types);
      }
      for (std::vector<Message*>::const_iterator outfault = (*op2)->outfault.begin(); outfault != (*op2)->outfault.end(); ++outfault)
      {
        if ((*outfault)->message)
        {
          if ((*outfault)->use == literal)
          {
            for (std::vector<wsdl__part>::const_iterator part = (*outfault)->message->part.begin(); part != (*outfault)->message->part.end(); ++part)
            {
              if ((*part).element)
                fprintf(stream, "\n  - Output Fault: %s (literal)\n", (*part).element);
              else if ((*part).name && (*part).type)
                fprintf(stream, "\n  - Output Fault: %s (literal)\n", (*part).name);
            }
          }
          else if ((*outfault)->message->name)
            fprintf(stream, "\n  - Output Fault: %s\n", (*outfault)->name);
          if ((*outfault)->message->name && (*outfault)->action)
            fprintf(stream, "    - Output Fault addressing action: \"%s\"\n", (*outfault)->action);
        }
        else if ((*outfault)->name)
          fprintf(stream, "\n  - Output Fault: %s (literal)\n", (*outfault)->name);
        if ((*outfault)->body_parts)
          fprintf(stream, "    Output Fault code: %s\n", (*outfault)->body_parts);
        text((*outfault)->documentation);       
        text((*outfault)->ext_documentation);   
        gen_policy(*this, (*outfault)->policy, "fault message", types);
      }
      if ((*op2)->input)
      {
        if (!(*op2)->input->header.empty())
          fprintf(stream, "\n  - Request message has mandatory header part(s) (see @ref SOAP_ENV__Header):\n");
        for (std::vector<soap__header>::const_iterator inputheader = (*op2)->input->header.begin(); inputheader != (*op2)->input->header.end(); ++inputheader)
        {
          if ((*inputheader).part)
          {
            if ((*inputheader).use == encoded && (*inputheader).namespace_)
              fprintf(stream, "    - %s\n", types.aname(NULL, (*inputheader).namespace_, (*inputheader).part));
            else if ((*inputheader).partPtr() && (*inputheader).partPtr()->element)
              fprintf(stream, "    - %s\n", types.aname(NULL, NULL, (*inputheader).partPtr()->element));
          }
        }
      }
      if ((*op2)->input && (*op2)->input->multipartRelated)
      {
        int k = 2;
        fprintf(stream, "\n  - Request message MIME multipart/related attachments:\n");
        for (std::vector<mime__part>::const_iterator part = (*op2)->input->multipartRelated->part.begin(); part != (*op2)->input->multipartRelated->part.end(); ++part)
        {
          if ((*part).soap__body_)
          {
            fprintf(stream, "    -# MIME attachment with SOAP Body and mandatory header part(s):\n");
            for (std::vector<soap__header>::const_iterator header = (*part).soap__header_.begin(); header != (*part).soap__header_.end(); ++header)
            {
              if ((*header).part)
              {
                if ((*header).use == encoded && (*header).namespace_)
                  fprintf(stream, "       - %s\n", types.aname(NULL, (*header).namespace_, (*header).part));
                else if ((*header).partPtr() && (*header).partPtr()->element)
                  fprintf(stream, "       - %s\n", types.aname(NULL, NULL, (*header).partPtr()->element));
              }
            }
          }
          else
          {
            fprintf(stream, "    -# MIME attachment %d:\n", k++);
            for (std::vector<mime__content>::const_iterator content = (*part).content.begin(); content != (*part).content.end(); ++content)
            {
              if ((*content).part)
              {
                fprintf(stream, "       - part: ");
                text((*content).part);
              }
              if ((*content).type)
              {
                fprintf(stream, "       - type: ");
                text((*content).type);
              }
            }
          }
        }
      }
      if ((*op2)->input && (*op2)->input->layout)
        fprintf(stream, "\n  - Request message has DIME attachments in compliance with %s\n", (*op2)->input->layout);
      if ((*op2)->output)
      {
        if (!(*op2)->output->header.empty())
          fprintf(stream, "\n  - Response message has mandatory header part(s): (see @ref SOAP_ENV__Header)\n");
        for (std::vector<soap__header>::const_iterator outputheader = (*op2)->output->header.begin(); outputheader != (*op2)->output->header.end(); ++outputheader)
        {
          if ((*outputheader).part)
          {
            if ((*outputheader).use == encoded && (*outputheader).namespace_)
              fprintf(stream, "    - %s\n", types.aname(NULL, (*outputheader).namespace_, (*outputheader).part));
            else if ((*outputheader).partPtr() && (*outputheader).partPtr()->element)
              fprintf(stream, "    - %s\n", types.aname(NULL, NULL, (*outputheader).partPtr()->element));
          }
        }
      }
      if ((*op2)->output && (*op2)->output_name && (*op2)->output->multipartRelated)
      {
        int k = 2;
        fprintf(stream, "\n  - Response message MIME multipart/related attachments\n");
        for (std::vector<mime__part>::const_iterator part = (*op2)->output->multipartRelated->part.begin(); part != (*op2)->output->multipartRelated->part.end(); ++part)
        {
          if ((*part).soap__body_)
          {
            fprintf(stream, "    -# MIME attachment with SOAP Body and mandatory header part(s):\n");
            for (std::vector<soap__header>::const_iterator header = (*part).soap__header_.begin(); header != (*part).soap__header_.end(); ++header)
            {
              if ((*header).part)
              {
                if ((*header).use == encoded && (*header).namespace_)
                  fprintf(stream, "       - %s\n", types.aname(NULL, (*header).namespace_, (*header).part));
                else if ((*header).partPtr() && (*header).partPtr()->element)
                  fprintf(stream, "       - %s\n", types.aname(NULL, NULL, (*header).partPtr()->element));
              }
            }
          }
          else
          {
            fprintf(stream, "    -# MIME attachment %d:\n", k++);
            for (std::vector<mime__content>::const_iterator content = (*part).content.begin(); content != (*part).content.end(); ++content)
            {
              if ((*content).part)
              {
                fprintf(stream, "       - part: ");
                text((*content).part);
              }
              if ((*content).type)
              {
                fprintf(stream, "       - type: ");
                text((*content).type);
              }
            }
          }
        }
      }
      if ((*op2)->output && (*op2)->output_name && (*op2)->output->layout)
        fprintf(stream, "\n  - Response message has DIME attachments in compliance with %s\n", (*op2)->output->layout);
      if (!Lflag)
      {
        fprintf(stream, "\nC stub function (defined in soapClient.c[pp] generated by soapcpp2):\n@code\n  int soap_%s_%s(\n    struct soap *soap,\n    NULL, // char *endpoint = NULL selects default endpoint for this operation\n    NULL, // char *action = NULL selects default action for this operation\n    // input parameters:", (*op2)->output ? "call" : "send", method_name);
        if ((*op2)->input)
          (*op2)->input->generate(types, ",", false, false, false, false, (*op2)->is_rest);
        if ((*op2)->output && (*op2)->output_name)
        {
          fprintf(stream, "\n    // output parameters:");
          if (flag)
          {
            if ((*op2)->style == rpc && (*op2)->output->message && !(*op2)->output->message->part.empty() && (*(*op2)->output->message->part.begin()).name)
            {
              fprintf(stream, "\n");
              fprintf(stream, anonymous ? anonformat : paraformat, types.tnamenoptr(NULL, NULL, (*(*op2)->output->message->part.begin()).type), cflag ? "*" : "&", types.aname(NULL, NULL, (*(*op2)->output->message->part.begin()).name), "");
            }
            else
              (*op2)->output->generate(types, "", anonymous, false, true, false, (*op2)->is_rest);
          }
          else
            fprintf(stream, "\n    struct %s%s", (*op2)->output_name, cflag ? "*" : "&");
        }
        fprintf(stream, "\n  );\n@endcode\n\nC server function (called from the service dispatcher defined in soapServer.c[pp]):\n@code\n  int %s(\n    struct soap *soap,\n    // input parameters:", method_name);
        if ((*op2)->input)
          (*op2)->input->generate(types, ",", false, false, false, false, (*op2)->is_rest);
        fprintf(stream, "\n    // output parameters:");
        if ((*op2)->output && (*op2)->output_name)
        {
          if (flag)
          {
            if ((*op2)->style == rpc && (*op2)->output->message && !(*op2)->output->message->part.empty() && (*(*op2)->output->message->part.begin()).name)
            {
              fprintf(stream, "\n");
              fprintf(stream, anonymous ? anonformat : paraformat, types.tnamenoptr(NULL, NULL, (*(*op2)->output->message->part.begin()).type), cflag ? "*" : "&", types.aname(NULL, NULL, (*(*op2)->output->message->part.begin()).name), "");
            }
            else
              (*op2)->output->generate(types, "", anonymous, false, true, false, (*op2)->is_rest);
          }
          else
            fprintf(stream, "\n    struct %s%s", (*op2)->output_name, cflag ? "*" : "&");
        }
        fprintf(stream, "\n  );\n@endcode\n\n");
        if (!cflag)
        {
          fprintf(stream, "C++ proxy class (defined in soap%sProxy.h generated with soapcpp2):\n", name);
          fprintf(stream, "@code\n  class %sProxy;\n@endcode\n", name);
          fprintf(stream, "Important: use soapcpp2 option '-j' (or '-i') to generate improved and easy-to-use proxy classes;\n\n");
          fprintf(stream, "C++ service class (defined in soap%sService.h generated with soapcpp2):\n", name);
          fprintf(stream, "@code\n  class %sService;\n@endcode\n", name);
          fprintf(stream, "Important: use soapcpp2 option '-j' (or '-i') to generate improved and easy-to-use service classes;\n\n");
        }
      }
      fprintf(stream, "*/\n\n");
      (*op2)->generate(types, *this);
    }
    else if (*op2 && (*op2)->output)
    {
      if (!(*op2)->input)
        method_name = (*op2)->output_name;
      else
        method_name = (*op2)->input_name;
      banner("Service Operation", method_name);
      fprintf(stream, "\n/** Operation \"%s\" of service binding \"%s\".\n", method_name, name);
      if ((*op2)->documentation)
        text((*op2)->documentation);
      if ((*op2)->operation_documentation)
        text((*op2)->operation_documentation);
      if ((*op2)->output->documentation)
      {
        fprintf(stream, "Output response:\n");
        text((*op2)->output->documentation);
      }
      if ((*op2)->output->ext_documentation)
      {
        fprintf(stream, "Output response:\n");
        text((*op2)->output->ext_documentation);
      }
      gen_policy(*this, (*op2)->policy, "operation", types);
      gen_policy(*this, (*op2)->output->policy, "response message", types);
      fprintf(stream, "\n  - One-way service response message\n");
      if ((*op2)->mep)
        fprintf(stream, "\n  - SOAP MEP: \"%s\"\n", ((*op2)->mep));
      if ((*op2)->style == document)
        fprintf(stream, "\n  - %s document/literal style messaging\n", (*op2)->protocol);
      else
      {
        if ((*op2)->output->use == literal)
          fprintf(stream, "\n  - %s RPC literal messaging\n", (*op2)->protocol);
        else if ((*op2)->output->encodingStyle)
          fprintf(stream, "\n  - %s RPC encodingStyle=\"%s\"\n", (*op2)->protocol, (*op2)->output->encodingStyle);
        else
          fprintf(stream, "\n  - %s RPC encoded messaging\n", (*op2)->protocol);
      }
      if (!location.empty())
      {
        fprintf(stream, "\n  - Default endpoints:\n");
        for (SetOfString::const_iterator port = location.begin(); port != location.end(); ++port)
          fprintf(stream, "    - %s\n", *port);
      }
      if ((*op2)->action)
      {
        if (*(*op2)->action)
          fprintf(stream, "\n  - Default SOAP action or REST location path:\n    - \"%s\"\n", (*op2)->action);
      }
      if ((*op2)->output)
      {
        if ((*op2)->output->action)
          fprintf(stream, "\n  - Addressing output action: \"%s\"\n", (*op2)->output->action);
      }
      for (std::vector<Message*>::const_iterator outfault = (*op2)->outfault.begin(); outfault != (*op2)->outfault.end(); ++outfault)
      {
        if ((*outfault)->message)
        {
          if ((*outfault)->use == literal)
          {
            for (std::vector<wsdl__part>::const_iterator part = (*outfault)->message->part.begin(); part != (*outfault)->message->part.end(); ++part)
            {
              if ((*part).element)
                fprintf(stream, "\n  - SOAP Output Fault: %s (literal)\n", (*part).element);
              else if ((*part).name && (*part).type)
                fprintf(stream, "\n  - SOAP Output Fault: %s (literal)\n", (*part).name);
            }
          }
          else if ((*outfault)->message->name)
            fprintf(stream, "\n  - SOAP Output Fault: %s\n", (*outfault)->name);
          if ((*outfault)->message->name && (*outfault)->action)
            fprintf(stream, "    - SOAP Output Fault addressing action: \"%s\"\n", (*outfault)->action);
        }
        else if ((*outfault)->name)
          fprintf(stream, "\n  - SOAP Output Fault: %s (literal)\n", (*outfault)->name);
        gen_policy(*this, (*outfault)->policy, "fault message", types);
      }
      if (!(*op2)->output->header.empty())
        fprintf(stream, "\n  - Response message has mandatory header part(s) (see @ref SOAP_ENV__Header):\n");
      for (std::vector<soap__header>::const_iterator outputheader = (*op2)->output->header.begin(); outputheader != (*op2)->output->header.end(); ++outputheader)
      {
        if ((*outputheader).part)
        {
          if ((*outputheader).use == encoded && (*outputheader).namespace_)
            fprintf(stream, "    - %s\n", types.aname(NULL, (*outputheader).namespace_, (*outputheader).part));
          else if ((*outputheader).partPtr() && (*outputheader).partPtr()->element)
            fprintf(stream, "    - %s\n", types.aname(NULL, NULL, (*outputheader).partPtr()->element));
        }
      }
      if ((*op2)->output->multipartRelated)
      {
        int k = 2;
        fprintf(stream, "\n  - Response message MIME multipart/related attachments:\n");
        for (std::vector<mime__part>::const_iterator part = (*op2)->output->multipartRelated->part.begin(); part != (*op2)->output->multipartRelated->part.end(); ++part)
        {
          if ((*part).soap__body_)
          {
            fprintf(stream, "    -# MIME attachment with SOAP Body and mandatory header part(s):\n");
            for (std::vector<soap__header>::const_iterator header = (*part).soap__header_.begin(); header != (*part).soap__header_.end(); ++header)
            {
              if ((*header).part)
              {
                if ((*header).use == encoded && (*header).namespace_)
                  fprintf(stream, "       - %s\n", types.aname(NULL, (*header).namespace_, (*header).part));
                else if ((*header).partPtr() && (*header).partPtr()->element)
                  fprintf(stream, "       - %s\n", types.aname(NULL, NULL, (*header).partPtr()->element));
              }
            }
          }
          else
          {
            fprintf(stream, "    -# MIME attachment %d:\n", k++);
            for (std::vector<mime__content>::const_iterator content = (*part).content.begin(); content != (*part).content.end(); ++content)
            {
              if ((*content).part)
              {
                fprintf(stream, "       - part: ");
                text((*content).part);
              }
              if ((*content).type)
              {
                fprintf(stream, "       - type: ");
                text((*content).type);
              }
            }
          }
        }
      }
      if ((*op2)->output->layout)
        fprintf(stream, "\n  - Response message has DIME attachments in compliance with %s\n", (*op2)->output->layout);
      if (!Lflag)
      {
        fprintf(stream, "\nC stub function (defined in soapClient.c[pp] generated by soapcpp2):\n@code\n  int soap_call_%s(\n    struct soap *soap,\n    NULL, // char *endpoint = NULL selects default endpoint for this operation\n    NULL, // char *action = NULL selects default action for this operation\n    // parameters:", method_name);
        (*op2)->output->generate(types, ",", false, false, false, false, (*op2)->is_rest);
        fprintf(stream, "\n  );\n@endcode\n\nC server function (called from the service dispatcher defined in soapServer.c[pp]):\n@code\n  int %s(\n    struct soap *soap,\n    // parameters:", method_name);
        (*op2)->output->generate(types, ",", false, false, false, false, (*op2)->is_rest);
        fprintf(stream, "\n  );\n@endcode\n\n");
        if (!cflag)
        {
          fprintf(stream, "C++ proxy class (defined in soap%sProxy.h):\n", name);
          fprintf(stream, "@code\n  class %sProxy;\n@endcode\n", name);
          fprintf(stream, "Important: use soapcpp2 option '-j' (or '-i') to generate improved and easy-to-use proxy classes;\n\n");
          fprintf(stream, "C++ service class (defined in soap%sService.h):\n", name);
          fprintf(stream, "@code\n  class %sService;\n@endcode\n", name);
          fprintf(stream, "Important: use soapcpp2 option '-j' (or '-i') to generate improved and easy-to-use service classes;\n\n");
        }
      }
      fprintf(stream, "*/\n\n");
      (*op2)->generate(types, *this);
    }
  }
  gen_policy_enablers(*this);
}

void Service::add_import(const char *s)
{
  if (std::find_if(imports.begin(), imports.end(), eqstr(s)) == imports.end())
    imports.push_back(s);
}

void Service::del_import(const char *s)
{
  VectorOfString::iterator i = std::find_if(imports.begin(), imports.end(), eqstr(s));
  if (i != imports.end())
    imports.erase(i);
}

////////////////////////////////////////////////////////////////////////////////
//
//      Operation methods
//
////////////////////////////////////////////////////////////////////////////////

void Operation::generate(Types& types, Service& service)
{
  bool flag = false, anonymous = ((style != document) && parameterOrder != NULL);
  const char *method_name = NULL;
  (void)service;
  if (output)
  {
    if (style == document)
      flag = (output->element || (output->message && output->message->part.size() == 1));
    else if (!wflag)
      flag = (output->element || (output->message && output->use == encoded && output->message->part.size() == 1 && !(*output->message->part.begin()).simpleTypePtr() && !(*output->message->part.begin()).complexTypePtr()));
    if (flag && input && input->message && output->message && !output->message->part.empty() && (*output->message->part.begin()).element)
      for (std::vector<wsdl__part>::const_iterator part = input->message->part.begin(); part != input->message->part.end(); ++part)
        if ((*part).element && !strcmp((*part).element, (*output->message->part.begin()).element))
          flag = false;
  }
  if (output && output_name && bflag)
  {
    if (input)
    {
      method_name = strstr(output_name + 1, "__");
      if (method_name)
        method_name += 2;
      else
        method_name = output_name;
    }
    else
    {
      method_name = strstr(input_name + 1, "__");
      if (method_name)
        method_name += 2;
      else
        method_name = input_name;
    }
    if (protocol)
      fprintf(stream, serviceformat, prefix, "method-protocol", method_name, protocol);
    if (output->content && output->content->type)
      fprintf(stream, serviceformat, prefix, "method-mime-type", method_name, output->content->type);
    else if (style == document)
      fprintf(stream, serviceformat, prefix, "method-style", method_name, "document");
    else
      fprintf(stream, serviceformat, prefix, "method-style", method_name, "rpc");
    if (output->use == literal)
      fprintf(stream, serviceformat, prefix, "method-encoding", method_name, "literal");
    else if (output->encodingStyle)
      fprintf(stream, serviceformat, prefix, "method-encoding", method_name, output->encodingStyle);
    else
      fprintf(stream, serviceformat, prefix, "method-encoding", method_name, "encoded");
    if (output->action && *output->action)
      fprintf(stream, serviceformat, prefix, "method-action", method_name, output->action);
    else if (action)
    {
      if (*action)
        fprintf(stream, serviceformat, prefix, "method-action", method_name, action);
      else
        fprintf(stream, serviceformat, prefix, "method-action", method_name, "\"\"");
    }
    for (std::vector<Message*>::const_iterator message = outfault.begin(); message != outfault.end(); ++message)
    {
      if ((*message)->message)
      {
        if ((*message)->use == literal)
        {
          for (std::vector<wsdl__part>::const_iterator part = (*message)->message->part.begin(); part != (*message)->message->part.end(); ++part)
          {
            if ((*part).element)
              fprintf(stream, serviceformat, prefix, "method-fault", method_name, types.aname(NULL, NULL, (*part).element));
            else if ((*part).type)
              fprintf(stream, serviceformat, prefix, "method-fault", method_name, types.aname(NULL, (*message)->URI, (*part).name));
          }
        }
        else
        {
          if ((*message)->message->name)
            fprintf(stream, serviceformat, prefix, "method-fault", method_name, (*message)->name);
        }
        if ((*message)->action)
          fprintf(stream, serviceformat, prefix, "method-fault-action", method_name, (*message)->action);
      }
      else if ((*message)->name)
        fprintf(stream, serviceformat, prefix, "method-fault", method_name, (*message)->name);
    }
    if (output->multipartRelated)
    {
      for (std::vector<mime__part>::const_iterator outputmime = output->multipartRelated->part.begin(); outputmime != output->multipartRelated->part.end(); ++outputmime)
      {
        for (std::vector<soap__header>::const_iterator outputheader = (*outputmime).soap__header_.begin(); outputheader != (*outputmime).soap__header_.end(); ++outputheader)
        {
          if ((*outputheader).part)
          {
            if ((*outputheader).use == encoded && (*outputheader).namespace_)
              fprintf(stream, serviceformat, prefix, "method-header-part", method_name, types.aname(NULL, (*outputheader).namespace_, (*outputheader).part));
            else if ((*outputheader).partPtr() && (*outputheader).partPtr()->element)
              fprintf(stream, serviceformat, prefix, "method-header-part", method_name, types.aname(NULL, NULL, (*outputheader).partPtr()->element));
          }
        }
        for (std::vector<mime__content>::const_iterator content = (*outputmime).content.begin(); content != (*outputmime).content.end(); ++content)
          if ((*content).type)
            fprintf(stream, serviceformat, prefix, "method-mime-type", method_name, (*content).type);
      }
    }
    // TODO: add headerfault directives?
    for (std::vector<soap__header>::const_iterator outputheader = output->header.begin(); outputheader != output->header.end(); ++outputheader)
    {
      if ((*outputheader).part)
      {
        if ((*outputheader).use == encoded && (*outputheader).namespace_)
          fprintf(stream, serviceformat, prefix, "method-header-part", method_name, types.aname(NULL, (*outputheader).namespace_, (*outputheader).part));
        else if ((*outputheader).partPtr() && (*outputheader).partPtr()->element)
          fprintf(stream, serviceformat, prefix, "method-header-part", method_name, types.aname(NULL, NULL, (*outputheader).partPtr()->element));
      }
    }
    for (std::vector<wsoap__header>::const_iterator outputwheader = output->wheader.begin(); outputwheader != output->wheader.end(); ++outputwheader)
    {
      if ((*outputwheader).element)
        fprintf(stream, serviceformat, prefix, "method-header-part", method_name, types.aname(NULL, NULL, (*outputwheader).element));
    }
    if (input)
    {
      fprintf(stream, "/// Operation response \"%s\" of operation \"%s\".\n", output_name, input_name);
      fprintf(stream, "int %s(", output_name);
    }
    else
      fprintf(stream, "int %s(", input_name);
    output->generate(types, ",", anonymous, true, false, false, is_rest);
    fprintf(stream, "\n    void\t///< One-way message: no output parameter\n);\n");
  }
  if (!input && output && output_name)
  {
    method_name = strstr(output_name + 1, "__");
    if (method_name)
      method_name += 2;
    else
      method_name = output_name;
    if (protocol)
      fprintf(stream, serviceformat, prefix, "method-protocol", method_name, protocol);
    if (output->content && output->content->type)
      fprintf(stream, serviceformat, prefix, "method-mime-type", method_name, output->content->type);
    else if (style == document)
      fprintf(stream, serviceformat, prefix, "method-style", method_name, "document");
    else
      fprintf(stream, serviceformat, prefix, "method-style", method_name, "rpc");
    if (output->use == literal)
      fprintf(stream, serviceformat, prefix, "method-encoding", method_name, "literal");
    else if (output->encodingStyle)
      fprintf(stream, serviceformat, prefix, "method-encoding", method_name, output->encodingStyle);
    else
      fprintf(stream, serviceformat, prefix, "method-encoding", method_name, "encoded");
    if (output->action && *output->action)
      fprintf(stream, serviceformat, prefix, "method-action", method_name, output->action);
    else if (action)
    {
      if (*action)
        fprintf(stream, serviceformat, prefix, "method-action", method_name, action);
      else
        fprintf(stream, serviceformat, prefix, "method-action", method_name, "\"\"");
    }
    for (std::vector<Message*>::const_iterator message = outfault.begin(); message != outfault.end(); ++message)
    {
      if ((*message)->message)
      {
        if ((*message)->use == literal)
        {
          for (std::vector<wsdl__part>::const_iterator part = (*message)->message->part.begin(); part != (*message)->message->part.end(); ++part)
          {
            if ((*part).element)
              fprintf(stream, serviceformat, prefix, "method-fault", method_name, types.aname(NULL, NULL, (*part).element));
            else if ((*part).type)
              fprintf(stream, serviceformat, prefix, "method-fault", method_name, types.aname(NULL, (*message)->URI, (*part).name));
          }
        }
        else
        {
          if ((*message)->message->name)
            fprintf(stream, serviceformat, prefix, "method-fault", method_name, (*message)->name);
        }
        if ((*message)->action)
          fprintf(stream, serviceformat, prefix, "method-fault-action", method_name, (*message)->action);
      }
      else if ((*message)->name)
        fprintf(stream, serviceformat, prefix, "method-fault", method_name, (*message)->name);
    }
    if (output->multipartRelated)
    {
      for (std::vector<mime__part>::const_iterator outputmime = output->multipartRelated->part.begin(); outputmime != output->multipartRelated->part.end(); ++outputmime)
      {
        for (std::vector<soap__header>::const_iterator outputheader = (*outputmime).soap__header_.begin(); outputheader != (*outputmime).soap__header_.end(); ++outputheader)
        {
          if ((*outputheader).part)
          {
            if ((*outputheader).use == encoded && (*outputheader).namespace_)
              fprintf(stream, serviceformat, prefix, "method-header-part", method_name, types.aname(NULL, (*outputheader).namespace_, (*outputheader).part));
            else if ((*outputheader).partPtr() && (*outputheader).partPtr()->element)
              fprintf(stream, serviceformat, prefix, "method-header-part", method_name, types.aname(NULL, NULL, (*outputheader).partPtr()->element));
          }
        }
        for (std::vector<mime__content>::const_iterator content = (*outputmime).content.begin(); content != (*outputmime).content.end(); ++content)
          if ((*content).type)
            fprintf(stream, serviceformat, prefix, "method-mime-type", method_name, (*content).type);
      }
    }
    // TODO: add headerfault directives?
    for (std::vector<soap__header>::const_iterator outputheader = output->header.begin(); outputheader != output->header.end(); ++outputheader)
    {
      if ((*outputheader).part)
      {
        if ((*outputheader).use == encoded && (*outputheader).namespace_)
          fprintf(stream, serviceformat, prefix, "method-header-part", method_name, types.aname(NULL, (*outputheader).namespace_, (*outputheader).part));
        else if ((*outputheader).partPtr() && (*outputheader).partPtr()->element)
          fprintf(stream, serviceformat, prefix, "method-header-part", method_name, types.aname(NULL, NULL, (*outputheader).partPtr()->element));
      }
    }
    for (std::vector<wsoap__header>::const_iterator outputwheader = output->wheader.begin(); outputwheader != output->wheader.end(); ++outputwheader)
    {
      if ((*outputwheader).element)
        fprintf(stream, serviceformat, prefix, "method-header-part", method_name, types.aname(NULL, NULL, (*outputwheader).element));
    }
    fprintf(stream, "int %s(", output_name);
    if (flag)
    {
      if (style == rpc && output->message && (*output->message->part.begin()).name)
      {
        fprintf(stream, "\n");
        if (output->use == encoded)
          fprintf(stream, anonymous ? anonformat : paraformat, types.tnamenoptr(NULL, NULL, (*output->message->part.begin()).type), cflag ? "*:" : "&:", types.aname(NULL, NULL, (*output->message->part.begin()).name), "");
        else
          fprintf(stream, anonymous ? anonformat : paraformat, types.tnamenoptr(NULL, NULL, (*output->message->part.begin()).type), cflag ? "*" : "&", types.aname(NULL, NULL, (*output->message->part.begin()).name), "");
        fprintf(stream, "\t///< Output parameter%s%s", output->use == encoded ? ", :unqualified name as per RPC encoding" : "", anonymous ? ", _wildcard name as per RPC parameterOrder" : "");
      }
      else
        output->generate(types, "", anonymous, true, true, false, is_rest);
    }
    else
    {
      fprintf(stream, "\n    struct %-28s%s", output_name, cflag ? "*" : "&");
      fprintf(stream, "\t///< Output response struct parameter");
    }
    fprintf(stream, "\n);\n");
  }
  if (input && input_name)
  {
    method_name = strstr(input_name + 1, "__");
    if (method_name)
      method_name += 2;
    else
      method_name = input_name;
    if (protocol)
      fprintf(stream, serviceformat, prefix, "method-protocol", method_name, protocol);
    if (input->content && input->content->type)
      fprintf(stream, serviceformat, prefix, "method-input-mime-type", method_name, input->content->type);
    else if (style == document)
      fprintf(stream, serviceformat, prefix, "method-style", method_name, "document");
    else
      fprintf(stream, serviceformat, prefix, "method-style", method_name, "rpc");
    if (output && output->content && output->content->type)
      fprintf(stream, serviceformat, prefix, "method-output-mime-type", method_name, output->content->type);
    if (!input || input->use == literal)
      fprintf(stream, serviceformat, prefix, "method-encoding", method_name, "literal");
    else if (input->encodingStyle)
      fprintf(stream, serviceformat, prefix, "method-encoding", method_name, input->encodingStyle);
    else
      fprintf(stream, serviceformat, prefix, "method-encoding", method_name, "encoded");
    if (output)
    {
      if (!input || input->use != output->use)
      {
        if (output->use == literal)
          fprintf(stream, serviceformat, prefix, "method-response-encoding", method_name, "literal");
        else if (output->encodingStyle)
          fprintf(stream, serviceformat, prefix, "method-response-encoding", method_name, output->encodingStyle);
        else
          fprintf(stream, serviceformat, prefix, "method-response-encoding", method_name, "encoded");
      }
      if (style == rpc && (!input || (input->URI && output->URI && *output->URI && strcmp(input->URI, output->URI))))
        fprintf(stream, schemaformat, types.nsprefix(NULL, output->URI), "namespace", output->URI);
    }
    if (input && input->action && *input->action)
      fprintf(stream, serviceformat, prefix, "method-input-action", method_name, input->action);
    else if (action)
    {
      if (*action)
        fprintf(stream, serviceformat, prefix, "method-action", method_name, action);
      else
        fprintf(stream, serviceformat, prefix, "method-action", method_name, "\"\"");
    }
    if (output && output->action && *output->action)
      fprintf(stream, serviceformat, prefix, "method-output-action", method_name, output->action);
    for (std::vector<Message*>::const_iterator message = outfault.begin(); message != outfault.end(); ++message)
    {
      if ((*message)->message)
      {
        if ((*message)->use == literal)
        {
          for (std::vector<wsdl__part>::const_iterator part = (*message)->message->part.begin(); part != (*message)->message->part.end(); ++part)
          {
            if ((*part).element)
              fprintf(stream, serviceformat, prefix, "method-fault", method_name, types.aname(NULL, NULL, (*part).element));
            else if ((*part).type)
              fprintf(stream, serviceformat, prefix, "method-fault", method_name, types.aname(NULL, (*message)->URI, (*part).name));
          }
        }
        else
        {
          if ((*message)->message->name)
            fprintf(stream, serviceformat, prefix, "method-fault", method_name, (*message)->name);
        }
        if ((*message)->action)
          fprintf(stream, serviceformat, prefix, "method-fault-action", method_name, (*message)->action);
      }
      else if ((*message)->name)
        fprintf(stream, serviceformat, prefix, "method-fault", method_name, (*message)->name);
    }
    if (input)
    {
      if (input->multipartRelated)
      {
        for (std::vector<mime__part>::const_iterator inputmime = input->multipartRelated->part.begin(); inputmime != input->multipartRelated->part.end(); ++inputmime)
        {
          for (std::vector<soap__header>::const_iterator inputheader = (*inputmime).soap__header_.begin(); inputheader != (*inputmime).soap__header_.end(); ++inputheader)
          {
            if ((*inputheader).part)
            {
              if ((*inputheader).use == encoded && (*inputheader).namespace_)
                fprintf(stream, serviceformat, prefix, "method-input-header-part", method_name, types.aname(NULL, (*inputheader).namespace_, (*inputheader).part));
              else if ((*inputheader).partPtr() && (*inputheader).partPtr()->element)
                fprintf(stream, serviceformat, prefix, "method-input-header-part", method_name, types.aname(NULL, NULL, (*inputheader).partPtr()->element));
            }
          }
          for (std::vector<mime__content>::const_iterator content = (*inputmime).content.begin(); content != (*inputmime).content.end(); ++content)
            if ((*content).type)
              fprintf(stream, serviceformat, prefix, "method-input-mime-type", method_name, (*content).type);
        }
      }
      // TODO: add headerfault directives?
      for (std::vector<soap__header>::const_iterator inputheader = input->header.begin(); inputheader != input->header.end(); ++inputheader)
      {
        if ((*inputheader).part)
        {
          if ((*inputheader).use == encoded && (*inputheader).namespace_)
            fprintf(stream, serviceformat, prefix, "method-input-header-part", method_name, types.aname(NULL, (*inputheader).namespace_, (*inputheader).part));
          else if ((*inputheader).partPtr() && (*inputheader).partPtr()->element)
            fprintf(stream, serviceformat, prefix, "method-input-header-part", method_name, types.aname(NULL, NULL, (*inputheader).partPtr()->element));
        }
      }
      for (std::vector<wsoap__header>::const_iterator inputwheader = input->wheader.begin(); inputwheader != input->wheader.end(); ++inputwheader)
      {
        if ((*inputwheader).element)
          fprintf(stream, serviceformat, prefix, "method-input-header-part", method_name, types.aname(NULL, NULL, (*inputwheader).element));
      }
    }
    if (output)
    {
      if (output->multipartRelated)
      {
        for (std::vector<mime__part>::const_iterator outputmime = output->multipartRelated->part.begin(); outputmime != output->multipartRelated->part.end(); ++outputmime)
        {
          for (std::vector<soap__header>::const_iterator outputheader = (*outputmime).soap__header_.begin(); outputheader != (*outputmime).soap__header_.end(); ++outputheader)
          {
            if ((*outputheader).part)
            {
              if ((*outputheader).use == encoded && (*outputheader).namespace_)
                fprintf(stream, serviceformat, prefix, "method-output-header-part", method_name, types.aname(NULL, (*outputheader).namespace_, (*outputheader).part));
              else if ((*outputheader).partPtr() && (*outputheader).partPtr()->element)
                fprintf(stream, serviceformat, prefix, "method-output-header-part", method_name, types.aname(NULL, NULL, (*outputheader).partPtr()->element));
            }
          }
          for (std::vector<mime__content>::const_iterator content = (*outputmime).content.begin(); content != (*outputmime).content.end(); ++content)
            if ((*content).type)
              fprintf(stream, serviceformat, prefix, "method-output-mime-type", method_name, (*content).type);
        }
      }
      for (std::vector<soap__header>::const_iterator outputheader = output->header.begin(); outputheader != output->header.end(); ++outputheader)
      {
        if ((*outputheader).part)
        {
          if ((*outputheader).use == encoded && (*outputheader).namespace_)
            fprintf(stream, serviceformat, prefix, "method-output-header-part", method_name, types.aname(NULL, (*outputheader).namespace_, (*outputheader).part));
          else if ((*outputheader).partPtr() && (*outputheader).partPtr()->element)
            fprintf(stream, serviceformat, prefix, "method-output-header-part", method_name, types.aname(NULL, NULL, (*outputheader).partPtr()->element));
        }
      }
      for (std::vector<wsoap__header>::const_iterator outputwheader = output->wheader.begin(); outputwheader != output->wheader.end(); ++outputwheader)
      {
        if ((*outputwheader).element)
          fprintf(stream, serviceformat, prefix, "method-output-header-part", method_name, types.aname(NULL, NULL, (*outputwheader).element));
      }
    }
    if (input)
      fprintf(stream, "int %s(", input_name);
    else
      fprintf(stream, "int %s(", output_name);
    if (input)
      input->generate(types, ",", anonymous, true, false, false, is_rest);
    if (output)
    {
      if (flag)
      {
        if (style == rpc && output->message && (*output->message->part.begin()).name)
        {
          fprintf(stream, "\n");
          if (output->use == encoded)
            fprintf(stream, anonymous ? anonformat : paraformat, types.tnamenoptr(NULL, NULL, (*output->message->part.begin()).type), cflag ? "*:" : "&:", types.aname(NULL, NULL, (*output->message->part.begin()).name), "");
          else
            fprintf(stream, anonymous ? anonformat : paraformat, types.tnamenoptr(NULL, NULL, (*output->message->part.begin()).type), cflag ? "*" : "&", types.aname(NULL, NULL, (*output->message->part.begin()).name), "");
          fprintf(stream, "\t///< Output parameter%s%s", output->use == encoded ? ", :unqualified name as per RPC encoding" : "", anonymous ? ", _wildcard name as per RPC parameterOrder" : "");
        }
        else
          output->generate(types, "", anonymous, true, true, false, is_rest);
      }
      else
      {
        fprintf(stream, "\n    struct %-28s%s", output_name, cflag ? "*" : "&");
        fprintf(stream, "\t///< Output response struct parameter");
      }
      fprintf(stream, "\n);\n");
    }
    else
      fprintf(stream, "\n    void\t///< One-way message: no output parameter\n);\n");
  }
}

////////////////////////////////////////////////////////////////////////////////
//
//      Message methods
//
////////////////////////////////////////////////////////////////////////////////

void Message::generate(Types& types, const char *sep, bool anonymous, bool remark, bool response, bool optional, bool is_rest)
{
  if (message)
  {
    for (std::vector<wsdl__part>::const_iterator part = message->part.begin(); part != message->part.end(); ++part)
    {
      if (!(*part).name)
      {
        fprintf(stderr, "\nError: no wsdl:definitions/message/part/@name \"%s\"\n", message->name ? message->name : "");
      }
      else if (!body_parts || soap_tagsearch(body_parts, (*part).name))
      {
        if (remark && (*part).documentation)
          comment("", (*part).name, "parameter", (*part).documentation);
        else
          fprintf(stream, "\n");
        if ((*part).element)
        {
          if ((*part).elementPtr())
          {
            const char *name, *type, *nameURI = NULL, *typeURI = NULL, *prefix = NULL;
            if (style == rpc && use == encoded)
            {
              name = (*part).name;
            }
            else
            {
              name = (*part).elementPtr()->name;
              if (style == document && (*part).elementPtr()->schemaPtr())
                nameURI = (*part).elementPtr()->schemaPtr()->targetNamespace;
            }
            if ((*part).elementPtr()->type)
            {
              type = (*part).elementPtr()->type;
            }
            else
            {
              type = (*part).elementPtr()->name;
              prefix = "_";
            }
            if ((*part).elementPtr()->schemaPtr())
              typeURI = (*part).elementPtr()->schemaPtr()->targetNamespace;
            if ((*part).elementPtr()->xmime__expectedContentTypes)
              fprintf(stream, "    /// MTOM attachment with content types %s\n", (*part).elementPtr()->xmime__expectedContentTypes);
            if (response)
            {
              const char *t;
              bool flag = false;
              if (!zflag || zflag > 6)
                t = types.tnamenoptr(prefix, typeURI, type);
              else
              {
                t = types.tname(prefix, typeURI, type);
                flag = (strchr(t, '*') && strcmp(t, "char*") && strcmp(t, "char *"));
              }
              fprintf(stream, anonymous ? anonformat : paraformat, t, flag ? " " : cflag ? "*" : "&", types.aname(NULL, nameURI, name), sep);
              if (remark)
                fprintf(stream, "\t///< Output parameter%s", anonymous ? ", _wildcard name as per RPC parameterOrder" : "");
            }
            else
            {
              fprintf(stream, anonymous ? anonformat : paraformat, types.pname(optional, false, prefix, typeURI, type), " ", types.aname(NULL, nameURI, name), sep);
              if (remark)
              {
                if (*sep == ',')
                  fprintf(stream, "\t///< Input parameter%s", anonymous ? ", _wildcard name as per RPC parameterOrder" : "");
                else
                  fprintf(stream, "\t///< Output parameter%s", anonymous ? ", _wildcard name as per RPC parameterOrder" : "");
              }
            }
          }
          else
          {
            fprintf(stream, anonymous ? anonformat : paraformat, types.pname(optional, false, NULL, NULL, (*part).element), " ", types.aname(NULL, NULL, (*part).element), sep);
            if (remark)
              fprintf(stream, "\t///< @todo Check element type (imported type)");
          }
        }
        else if ((*part).type)
        {
          bool unqual = (style == rpc && use == encoded) && remark;
          const char *nameURI = NULL, *typeURI = NULL;
          if (!Xflag && (!zflag || zflag > 8))
          {
            if ((*part).simpleTypePtr() && (*part).simpleTypePtr()->schemaPtr())
              typeURI = (*part).simpleTypePtr()->schemaPtr()->targetNamespace;
            else if ((*part).complexTypePtr() && (*part).complexTypePtr()->schemaPtr())
              typeURI = (*part).complexTypePtr()->schemaPtr()->targetNamespace;
            if (style == document && use == literal && !is_rest)
            {
              if (remark && !Wflag)
                fprintf(stderr, "\nWarning: wsdl:definitions/message/part/@type schema type for part/@name=\"%s\" makes the name ambiguous resulting in a qualified part name when its type is not a primitive XSD type and when option -X is not used to disambiguate wrapped patterns, the document/literal wrapped pattern requires part/@element\n", (*part).name);
              /* this adds the schema namespace prefix, but not for primitive XSD types and other built-in types */
              nameURI = typeURI;
            }
          }
          if (response)
          {
            const char *t;
            bool flag = false;
            if (!zflag || zflag > 6)
              t = types.tnamenoptr(NULL, typeURI, (*part).type);
            else
            {
              t = types.tname(NULL, typeURI, (*part).type);
              flag = (strchr(t, '*') && strcmp(t, "char*") && strcmp(t, "char *"));
            }
            if (unqual)
              fprintf(stream, anonymous ? anonformat : paraformat, t, flag ? " :" : cflag ? "*:" : "&:", types.aname(NULL, NULL, (*part).name), sep);
            else
              fprintf(stream, anonymous ? anonformat : paraformat, t, flag ? " " : cflag ? "*" : "&", types.aname(NULL, nameURI, (*part).name), sep);
            if (remark)
              fprintf(stream, "\t///< Output parameter%s%s", unqual ? ", :unqualified name as per RPC encoding" : "", anonymous ? ", _wildcard name as per RPC parameterOrder" : "");
          }
          else
          {
            const char *t = types.pname((optional || (*part).is_optional()) && !(*part).is_repeating(), false, NULL, typeURI, (*part).type);
            if ((*part).is_repeating())
            {
              if (cflag)
              {
                fprintf(stream, sizeparaformat, "int", types.aname(NULL, NULL, (*part).name), ",");
                fprintf(stream, "\t///< Repeating parameter\n");
                fprintf(stream, anonymous ? anonformat : paraformat, t, unqual ? "*:" : "*", types.aname(NULL, nameURI, (*part).name), "");
              }
              else
              {
                fprintf(stream, pointertemplateparaformat, types.vname("$CONTAINER"), t, unqual ? " :" : " ", types.aname(NULL, nameURI, (*part).name), "");
              }
            }
            else
            {
              fprintf(stream, anonymous ? anonformat : paraformat, t, unqual ? " :" : " ", types.aname(NULL, nameURI, (*part).name), "");
              types.gendefault(NULL, (*part).type, NULL, (*part).simpleTypePtr(), (*part).get_default(), NULL, "=");
            }
            fprintf(stream, "%s", sep);
            if (remark)
            {
              if (*sep == ',')
                fprintf(stream, "\t///< Input parameter%s%s%s", (*part).is_repeating() ? " is repeating" : optional || (*part).is_optional() ? " is optional" : "", unqual ? ", :unqualified name as per RPC encoding" : "", anonymous ? ", _wildcard name as per RPC parameterOrder" : "");
              else
                fprintf(stream, "\t///< Output parameter%s%s%s", (*part).is_repeating() ? " is repeating" : optional || (*part).is_optional() ? " is optional" : "", unqual ? ", :unqualified name as per RPC encoding" : "", anonymous ? ", _wildcard name as per RPC parameterOrder" : "");
              const std::vector<char*>& options = (*part).options();
              if (!options.empty())
                fprintf(stream, " with one of the values");
              for (std::vector<char*>::const_iterator i = options.begin(); i != options.end(); ++i)
                fprintf(stream, " '%s'", *i);
              if ((*part).is_fixed() && (*part).get_default())
                fprintf(stream, " with fixed value '%s'", (*part).get_default());
            }
          }
        }
        else if (style == rpc)
        {
          fprintf(stderr, "\nError: no wsdl:definitions/message/part/@type in part \"%s\"\n", (*part).name);
        }
        else
        {
          fprintf(stderr, "\nError: no wsdl:definitions/message/part/@element in part \"%s\"\n", (*part).name);
        }
      }
    }
  }
  else if (element) // WSDL 2.0
  {
    const char *prefix = NULL;
    const char *URI = NULL;
    const char *name = element->name;
    const char *type = element->type;
    if (!type)
    {
      type = name;
      prefix = "_";
    }
    if (element->schemaPtr())
      URI = element->schemaPtr()->targetNamespace;
    if (response)
    {
      const char *t;
      bool flag = false;
      if (!zflag || zflag > 6)
        t = types.tnamenoptr(prefix, URI, type);
      else
      {
        t = types.tname(prefix, URI, type);
        flag = (strchr(t, '*') && strcmp(t, "char*") && strcmp(t, "char *"));
      }
      fprintf(stream, "\n");
      fprintf(stream, paraformat, t, flag ? " " : cflag ? "*" : "&", anonymous ? "" : types.aname(NULL, URI, name), sep);
      if (remark)
        fprintf(stream, "\t///< Output parameter%s", anonymous ? ", _wildcard name as per RPC parameterOrder" : "");
    }
    else if (style == document || element->simpleTypePtr())
    {
      // WSDL 2.0 document style
      fprintf(stream, "\n");
      fprintf(stream, anonymous ? anonformat : paraformat, types.pname(optional, false, prefix, URI, type), " ", types.aname(NULL, URI, name), sep);
      if (remark)
      {
        if (*sep == ',')
          fprintf(stream, "\t///< Input parameter%s", anonymous ? ", _wildcard name as er RPC parameterOrder" : "");
        else
          fprintf(stream, "\t///< Output parameter%s", anonymous ? ", _wildcard name as er RPC parameterOrder" : "");
      }
    }
    else if (element->complexTypePtr())
    {
      // WSDL 2.0 RPC style
      xs__seqchoice *seq = element->complexTypePtr()->sequence;
      for (std::vector<xs__contents>::const_iterator i = seq->__contents.begin(); i != seq->__contents.end(); ++i)
      {
        if ((*i).__union == SOAP_UNION_xs__union_content_element)
        {
          name = (*i).__content.element->name;
          type = (*i).__content.element->type;
          if (!type)
            type = name;
          fprintf(stream, "\n");
          fprintf(stream, anonymous ? anonformat : paraformat, types.pname(optional, false, prefix, URI, type), " ", types.aname(NULL, NULL, name), sep);
          if (remark)
          {
            if (*sep == ',')
              fprintf(stream, "\t///< Input parameter%s", anonymous ? ", _wildcard name as per RPC parameterOrder" : "");
            else
              fprintf(stream, "\t///< Output parameter%s", anonymous ? ", _wildcard name as per RPC parameterOrder" : "");
          }
        }
      }
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
//
//      Miscellaneous
//
////////////////////////////////////////////////////////////////////////////////

static const char *urienc(struct soap *soap, const char *uri)
{
  const char *r, *s;
  char *t;
  size_t n = 0;
  if (!uri)
    return NULL;
  for (s = uri; *s; s++)
    if (!URI_CHAR(*s))
      n++;
  if (!n)
    return uri;
  n = strlen(uri) + 2*n;
  r = t = (char*)soap_malloc(soap, n + 1);
  if (r)
  {
    for (s = uri; *s; s++)
    {
      if (URI_CHAR(*s))
      {
        *t++ = *s;
      }
      else
      {
        (SOAP_SNPRINTF(t, n + 1 - (t - r), 3), "%%%2.2x", *s);
        t += 3;
      }
    }
    *t = '\0';
  }
  return r;
}

static bool imported(const char *tag)
{
  if (!tag || *tag != '"')
    return false;
  for (SetOfString::const_iterator u = exturis.begin(); u != exturis.end(); ++u)
  {
    size_t n = strlen(*u);
    if (!strncmp(*u, tag + 1, n) && tag[n+1] == '"')
      return true;
  }
  return false;
}

static void comment(const char *start, const char *middle, const char *end, const char *text)
{
  if (text)
  {
    if (strchr(text, '\r') || strchr(text, '\n'))
      fprintf(stream, "\n/** %s %s %s documentation:\n%s\n*/\n\n", start, middle, end, text);
    else
      fprintf(stream, "\n/// %s %s %s: %s\n", start, middle, end, text);
  }
}

static void page(const char *page, const char *title, const char *text)
{
  if (text)
    fprintf(stream, "@page %s%s \"%s\"\n", page, title, text);
  else
    fprintf(stream, "@page %s%s\n", page, title);
}

static void section(const char *section, const char *title, const char *text)
{
  if (text)
    fprintf(stream, "\n@section %s%s \"%s\"\n", section, title, text);
  else
    fprintf(stream, "\n@section %s%s\n", section, title);
}

static void banner(const char *text1, const char *text2)
{
  int i;
  if (!text1)
    return;
  fprintf(stream, "\n/");
  for (i = 0; i < 78; i++)
    fputc('*', stream);
  if (text2)
    fprintf(stream, "\\\n *%76s*\n * %-75s*\n *   %-73s*\n *%76s*\n\\", "", text1, text2, "");
  else
    fprintf(stream, "\\\n *%76s*\n * %-75s*\n *%76s*\n\\", "", text1, "");
  for (i = 0; i < 78; i++)
    fputc('*', stream);
  fprintf(stream, "/\n\n");
  if (vflag)
    fprintf(stderr, "\n----<< %s: %s >>----\n\n", text1, text2 ? text2 : "");
}

static void ident()
{
  time_t t = time(NULL), *p = &t;
  char tmp[256];
  int i;
  strftime(tmp, 256, "%Y-%m-%d %H:%M:%S GMT", gmtime(p));
  fprintf(stream, "/* %s\n   Generated by wsdl2h " WSDL2H_VERSION " from ", outfile ? outfile : "");
  if (infiles)
  {
    for (i = 0; i < infiles; i++)
      fprintf(stream, "%s ", infile[i]);
  }
  else
    fprintf(stream, "(stdin) ");
  fprintf(stream, "and %s\n\
   %s\n\n\
   DO NOT INCLUDE THIS ANNOTATED FILE DIRECTLY IN YOUR PROJECT SOURCE CODE.\n\
   USE THE FILES GENERATED BY soapcpp2 FOR YOUR PROJECT'S SOURCE CODE.\n\n\
gSOAP XML Web services tools\n\
Copyright (C) 2000-2021, Robert van Engelen, Genivia Inc. All Rights Reserved.\n\
This program is released under the GPL with the additional exemption that\n\
compiling, linking, and/or using OpenSSL is allowed.\n\
--------------------------------------------------------------------------------\n\
A commercial use license is available from Genivia Inc., contact@genivia.com\n\
--------------------------------------------------------------------------------\n\
*/\n\n", mapfile, tmp);
}

void text(const char *text)
{
  const char *s;
  if (!text)
    return;
  size_t k = 0;
  for (s = text; *s; s++, k++)
  {
    switch (*s)
    {
      case '\n':
        if (k)
        {
          fputc('\n', stream);
          k = 0;
        }
        break;
      case '\t':
        k = 8 * ((k + 8) / 8) - 1;
        fputc('\t', stream);
        break;
      case '/':
        fputc(*s, stream);
        if (s[1] == '*')
          fputc(' ', stream);
        break;
      case '*':
        fputc(*s, stream);
        if (s[1] == '/')
          fputc(' ', stream);
        break;
      case ' ':
        if (k >= 79)
        {
          fputc('\n', stream);
          k = 0;
        }
        else
          fputc(' ', stream);
        break;
      default:
        if (*s >= 32)
          fputc(*s, stream);
    }
  }
  if (k)
    fputc('\n', stream);
}

////////////////////////////////////////////////////////////////////////////////
//
//      WS-Policy
//
////////////////////////////////////////////////////////////////////////////////

static void gen_policy(Service& service, const std::vector<const wsp__Policy*>& policy, const char *text, Types& types)
{
  if (!policy.empty())
  { fprintf(stream, "\n  - WS-Policy applicable to the %s:\n", text);
    for (std::vector<const wsp__Policy*>::const_iterator p = policy.begin(); p != policy.end(); ++p)
      if (*p)
        (*p)->generate(service, types, 0);
  }
}

static void gen_policy_enablers(const Service& service)
{
  fprintf(stream, "\n/** ");
  page(service.name, " Binding", service.name);
  section(service.name, "_policy_enablers Policy Enablers of Binding", service.name);
  if (service.imports.empty())
  {
    fprintf(stream, "\nNone specified.\n\n*/\n");
    return;
  }
  fprintf(stream, "\nBased on policies and protocol requirements, this service requires importing");
  for (VectorOfString::const_iterator i = service.imports.begin(); i != service.imports.end(); ++i)
    fprintf(stream, " \"%s\"", *i); 
  fprintf(stream, "\n\n  - WS-Policy reminders and enablers (when applicable):\n");
  fprintf(stream, "    - WS-Addressing 1.0 (2005/08, accepts 2004/08):\n\t@code\n\t#import \"wsa5.h\" // to be added to this header file for the soapcpp2 build step\n\t@endcode\n\t@code\n\t#include \"plugin/wsaapi.h\"\n\tsoap_register_plugin(soap, soap_wsa); // register the wsa plugin in your code\n\t// See the user guide gsoap/doc/wsa/html/index.html\n\t@endcode\n");
  fprintf(stream, "    - WS-Addressing (2004/08):\n\t@code\n\t#import \"wsa.h\" // to be added to this header file for the soapcpp2 build step\n\t@endcode\n\t@code\n\t#include \"plugin/wsaapi.h\"\n\tsoap_register_plugin(soap, soap_wsa); // register the wsa plugin in your code\n\t// See the user guide gsoap/doc/wsa/html/index.html\n\t@endcode\n");
  fprintf(stream, "    - WS-ReliableMessaging 1.0:\n\t@code\n\t#import \"wsrm5.h\" // to be added to this header file for the soapcpp2 build step\n\t@endcode\n\t@code\n\t#include \"plugin/wsrmapi.h\"\n\tsoap_register_plugin(soap, soap_wsa); // register the wsa plugin in your code\n\tsoap_register_plugin(soap, soap_wsrm); // register the wsrm plugin in your code\n\t// See the user guide gsoap/doc/wsrm/html/index.html\n\t@endcode\n");
  fprintf(stream, "    - WS-ReliableMessaging 1.1:\n\t@code\n\t#import \"wsrm.h\" // to be added to this header file for the soapcpp2 build step\n\t@endcode\n\t@code\n\t#include \"plugin/wsrmapi.h\"\n\tsoap_register_plugin(soap, soap_wsa); // register the wsa plugin in your code\n\tsoap_register_plugin(soap, soap_wsrm); // register the wsrm plugin in your code\n\t// See the user guide gsoap/doc/wsrm/html/index.html\n\t@endcode\n");
  fprintf(stream, "    - WS-Discovery 1.0:\n\t@code\n\t#import \"wsrm5.h\" // to be added to this header file for the soapcpp2 build step\n\t@endcode\n\t@code\n\t#include \"plugin/wsddapi.h\" // See the user guide gsoap/doc/wsdd/html/index.html\n\t@endcode\n");
  fprintf(stream, "    - WS-Discovery 1.1:\n\t@code\n\t#import \"wsrm.h\" // to be added to this header file for the soapcpp2 build step\n\t@endcode\n\t@code\n\t#include \"plugin/wsddapi.h\" // See the user guide gsoap/doc/wsdd/html/index.html\n\t@endcode\n");
  fprintf(stream, "    - WS-Security (SOAP Message Security) 1.0 (accepts 1.1):\n\t@code\n\t#import \"wsse.h\" // to be added to this header file for the soapcpp2 build step\n\t@endcode\n\t@code\n\t#include \"plugin/wsseapi.h\"\n\tsoap_register_plugin(soap, soap_wsse); // register the wsse plugin in your code\n\t// See the user guide gsoap/doc/wsse/html/index.html\n\t@endcode\n");
  fprintf(stream, "    - WS-Security (SOAP Message Security) 1.1 (accepts 1.0):\n\t@code\n\t#import \"wsse11.h\" // to be added to this header file for the soapcpp2 build step\n\t@endcode\n\t@code\n\t#include \"plugin/wsseapi.h\"\n\tsoap_register_plugin(soap, soap_wsse); // register the wsse plugin in your code\n\t// See the user guide gsoap/doc/wsse/html/index.html\n\t@endcode\n");
  fprintf(stream, "    - HTTP Digest Authentication:\n\t@code\n\t#include \"plugin/httpda.h\"\n\tsoap_register_plugin(soap, soap_http_da); // register the HTTP DA plugin in your code\n\t// See the user guide gsoap/doc/httpda/html/index.html\n\t@endcode\n");
  fprintf(stream, "*/\n\n");
  for (VectorOfString::const_iterator i = service.imports.begin(); i != service.imports.end(); ++i)
    fprintf(stream, "#import \"%s\"\n", *i); 
}

////////////////////////////////////////////////////////////////////////////////
//
//      BPEL 2.0
//
////////////////////////////////////////////////////////////////////////////////

static void gen_plnk(const Service& service)
{
  if (!service.role.empty())
  {
    for (std::vector<const plnk__tRole*>::const_iterator r = service.role.begin(); r != service.role.end(); ++r)
    {
      if (*r && (*r)->portTypePtr() && (*r)->plnkPtr())
      {
        fprintf(stream, "\n  - \"%s\" role \"%s\" port type \"%s\" operations.\n", (*r)->plnkPtr()->name, (*r)->portTypePtr()->name, (*r)->name);
        text((*r)->documentation);
        text((*r)->plnkPtr()->documentation);
      }
    }
  }
}

static void gen_vprop(const wsdl__definitions& definitions, Types& types)
{
  if (!definitions.vprop__property.empty())
  {
    banner("BPEL 2.0 Variables");
    fprintf(stream, "/** ");
    page("page_bpel", " BPEL 2.0 Variables", NULL);
    for (std::vector<vprop__tProperty>::const_iterator i1 = definitions.vprop__property.begin(); i1 != definitions.vprop__property.end(); ++i1)
    {
      if ((*i1).name)
      {
        fprintf(stream, "\nBPEL 2.0 Variable \"%s\"\n", (*i1).name);
        text((*i1).documentation);
        fprintf(stream, "\n");
        for (std::vector<vprop__tPropertyAlias>::const_iterator j = definitions.vprop__propertyAlias.begin(); j != definitions.vprop__propertyAlias.end(); ++j)
        {
          if ((*j).vpropPtr() == &(*i1) && (*j).query && (*j).query->__mixed)
          {
            if ((*j).messageType)
            {
              if ((*j).type)
                fprintf(stream, "  - Variable property alias for message %s/%s of type %s.\n", (*j).messageType, (*j).part, (*j).type);
              else if ((*j).element)
                fprintf(stream, "  - Variable property alias for message %s/%s element %s.\n", (*j).messageType, (*j).part, (*j).element);
            }
            else
              fprintf(stream, "  - Variable property alias.\n");
            text((*j).documentation);
            if ((*j).type)
              fprintf(stream, "    XPath %s: %s\n", types.tname(NULL, NULL, (*j).type), (*j).query->__mixed);
            else if ((*j).element)
              fprintf(stream, "    XPath %s: %s\n", types.tname(NULL, NULL, (*j).element), (*j).query->__mixed);
          }
        }
      }
    }
    fprintf(stream, "*/\n\n");
    for (std::vector<vprop__tProperty>::const_iterator i2 = definitions.vprop__property.begin(); i2 != definitions.vprop__property.end(); ++i2)
    {
      if ((*i2).name)
      {
        for (std::vector<vprop__tPropertyAlias>::const_iterator j = definitions.vprop__propertyAlias.begin(); j != definitions.vprop__propertyAlias.end(); ++j)
        {
          if ((*j).vpropPtr() == &(*i2) && (*j).query && (*j).query->__mixed)
          {
            if ((*j).type)
              fprintf(stream, "//xpath %s %s\n", types.tname(NULL, NULL, (*j).type), (*j).query->__mixed);
            else if ((*j).element)
              fprintf(stream, "//xpath %s %s\n", types.tname(NULL, NULL, (*j).element), (*j).query->__mixed);
          }
        }
        if ((*i2).type)
          fprintf(stream, elementformat, types.tname(NULL, NULL, (*i2).type), types.aname(NULL, NULL, (*i2).name));
        else if ((*i2).element)
          fprintf(stream, elementformat, types.tname(NULL, NULL, (*i2).element), types.aname(NULL, NULL, (*i2).name));
        fprintf(stream, ";\n");
      }
    }
    fprintf(stream, "\n");
  }
}
