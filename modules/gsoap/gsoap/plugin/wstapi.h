/*
        wstapi.h

        WS-Trust plugin.

        See wstapi.c for documentation and details.

gSOAP XML Web services tools
Copyright (C) 2000-2016, Robert van Engelen, Genivia Inc., All Rights Reserved.
This part of the software is released under one of the following licenses:
GPL or the gSOAP public license.
--------------------------------------------------------------------------------
gSOAP public license.

The contents of this file are subject to the gSOAP Public License Version 1.3
(the "License"); you may not use this file except in compliance with the
License. You may obtain a copy of the License at
http://www.cs.fsu.edu/~engelen/soaplicense.html
Software distributed under the License is distributed on an "AS IS" basis,
WITHOUT WARRANTY OF ANY KIND, either express or implied. See the License
for the specific language governing rights and limitations under the License.

The Initial Developer of the Original Code is Robert A. van Engelen.
Copyright (C) 2000-2016, Robert van Engelen, Genivia Inc., All Rights Reserved.
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

This program is released under the GPL with the additional exemption that
compiling, linking, and/or using OpenSSL is allowed.
--------------------------------------------------------------------------------
A commercial use license is available from Genivia, Inc., contact@genivia.com
--------------------------------------------------------------------------------
*/

#ifndef WSTAPI_H
#define WSTAPI_H

#include "wsaapi.h"     /* also includes soapH.h, see wsaapi.h if you are using a different fileH.h */
#include "wsseapi.h"    /* also includes soapH.h, see wsseapi.h if you are using a different fileH.h */
#include "smdevp.h"     /* digest algos */
#include "mecevp.h"     /* encryption algos */
#include "threads.h"    /* mutex for sequence database */

extern const char *soap_wst_rst_action;
extern const char *soap_wst_rstr_action;
extern const char *soap_wst_rstc_action;
extern const char *soap_wst_rstrc_action;

#ifdef __cplusplus
extern "C" {
#endif

SOAP_FMAC1 int SOAP_FMAC2 soap_wst_request_saml_token(struct soap *soap, const char *endpoint, int soapver, const char *applyto, const char *username, const char *password, saml1__AssertionType **saml1, saml2__AssertionType **saml2);

SOAP_FMAC1 int SOAP_FMAC2 soap_wst_request_psha1_token(struct soap *soap, const char *endpoint, int soapver, const char *applyto, const char *username, const char *password, char *psha1, size_t psha1len);

#ifdef __cplusplus
}
#endif

SOAP_FMAC5 int SOAP_FMAC6 soap_call___wst__RequestSecurityToken(struct soap *soap, const char *soap_endpoint, const char *soap_action, struct wst__RequestSecurityTokenType *wst__RequestSecurityToken, struct wst__RequestSecurityTokenResponseType *wst__RequestSecurityTokenResponse);

SOAP_FMAC5 int SOAP_FMAC6 soap_call___wst__RequestSecurityTokenResponse(struct soap *soap, const char *soap_endpoint, const char *soap_action, struct wst__RequestSecurityTokenResponseType *wst__RequestSecurityTokenResponse, struct wst__RequestSecurityTokenResponseCollectionType *wst__RequestSecurityTokenResponseCollection);

SOAP_FMAC5 int SOAP_FMAC6 soap_call___wst__RequestSecurityTokenCollection(struct soap *soap, const char *soap_endpoint, const char *soap_action, struct wst__RequestSecurityTokenCollectionType *wst__RequestSecurityTokenCollection, struct wst__RequestSecurityTokenResponseCollectionType *wst__RequestSecurityTokenResponseCollection);

#endif
