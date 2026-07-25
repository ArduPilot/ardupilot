/*
	wstx2.h

	WS-Trust definitions:
	SOAP Header definitions for WS-Trust 2005/02
	WS-Trust operations

	Imported by import/wst.h

gSOAP XML Web services tools
Copyright (C) 2000-2015, Robert van Engelen, Genivia Inc., All Rights Reserved.
This part of the software is released under ONE of the following licenses:
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
Copyright (C) 2000-2015, Robert van Engelen, Genivia Inc., All Rights Reserved.
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

mutable struct SOAP_ENV__Header
{ int								__sizeIssuedTokens 0; ///< size of the array
  struct wst__RequestSecurityTokenResponseCollectionType	*wst__IssuedTokens 0; ///< array of tokens
  char*								wst__RequestType  0;
};

//gsoap wst service name: wst

//gsoap wst service method-header-part:     RequestSecurityToken wsa5__MessageID
//gsoap wst service method-header-part:     RequestSecurityToken wsa5__RelatesTo
//gsoap wst service method-header-part:     RequestSecurityToken wsa5__From
//gsoap wst service method-header-part:     RequestSecurityToken wsa5__ReplyTo
//gsoap wst service method-header-part:     RequestSecurityToken wsa5__FaultTo
//gsoap wst service method-header-part:     RequestSecurityToken wsa5__To
//gsoap wst service method-header-part:     RequestSecurityToken wsa5__Action
//gsoap wst service method-header-part:     RequestSecurityToken wst__RequestType
//gsoap wst service method-action:          RequestSecurityToken http://schemas.xmlsoap.org/ws/2005/02/trust/RST/Issue
//gsoap wst service method-output-action:   RequestSecurityToken http://schemas.xmlsoap.org/ws/2005/02/trust/RSTR/Issue
int __wst__RequestSecurityToken(
  struct wst__RequestSecurityTokenType				*wst__RequestSecurityToken, ///< request message
  struct wst__RequestSecurityTokenResponseType          	*wst__RequestSecurityTokenResponse ///< response message
);

//gsoap wst service method-header-part:     RequestSecurityTokenResponse wsa5__MessageID
//gsoap wst service method-header-part:     RequestSecurityTokenResponse wsa5__RelatesTo
//gsoap wst service method-header-part:     RequestSecurityTokenResponse wsa5__From
//gsoap wst service method-header-part:     RequestSecurityTokenResponse wsa5__ReplyTo
//gsoap wst service method-header-part:     RequestSecurityTokenResponse wsa5__FaultTo
//gsoap wst service method-header-part:     RequestSecurityTokenResponse wsa5__To
//gsoap wst service method-header-part:     RequestSecurityTokenResponse wsa5__Action
//gsoap wst service method-header-part:     RequestSecurityTokenResponse wst__RequestType
//gsoap wst service method-action:          RequestSecurityTokenResponse http://schemas.xmlsoap.org/ws/2005/02/trust/RSTR/Issue
//gsoap wst service method-output-action:   RequestSecurityTokenCollection http://schemas.xmlsoap.org/ws/2005/02/trust/RSTRC/IssueFinal
int __wst__RequestSecurityTokenResponse(
  struct wst__RequestSecurityTokenResponseType			*wst__RequestSecurityTokenResponse, ///< request message
  struct wst__RequestSecurityTokenResponseCollectionType	*wst__RequestSecurityTokenResponseCollection ///< response message
);

//gsoap wst service method-header-part:     RequestSecurityTokenCollection wsa5__MessageID
//gsoap wst service method-header-part:     RequestSecurityTokenCollection wsa5__RelatesTo
//gsoap wst service method-header-part:     RequestSecurityTokenCollection wsa5__From
//gsoap wst service method-header-part:     RequestSecurityTokenCollection wsa5__ReplyTo
//gsoap wst service method-header-part:     RequestSecurityTokenCollection wsa5__FaultTo
//gsoap wst service method-header-part:     RequestSecurityTokenCollection wsa5__To
//gsoap wst service method-header-part:     RequestSecurityTokenCollection wsa5__Action
//gsoap wst service method-header-part:     RequestSecurityTokenCollection wst__RequestType
//gsoap wst service method-action:          RequestSecurityTokenCollection http://schemas.xmlsoap.org/ws/2005/02/trust/RSTC/Issue
//gsoap wst service method-output-action:   RequestSecurityTokenCollection http://schemas.xmlsoap.org/ws/2005/02/trust/RSTRC/IssueFinal
int __wst__RequestSecurityTokenCollection(
  struct wst__RequestSecurityTokenCollectionType		*wst__RequestSecurityTokenCollection, ///< request message
  struct wst__RequestSecurityTokenResponseCollectionType	*wst__RequestSecurityTokenResponseCollection ///< response message
);
