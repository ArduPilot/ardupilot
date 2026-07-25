/*
	curlapi.h

	cURL plugin.

gSOAP XML Web services tools
Copyright (C) 2000-2017, Robert van Engelen, Genivia Inc., All Rights Reserved.
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
Copyright (C) 2000-2017, Robert van Engelen, Genivia Inc., All Rights Reserved.
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

#ifndef CURLAPI_H
#define CURLAPI_H

#include "stdsoap2.h"
#include <curl/curl.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Plugin identification for plugin registry */
#define SOAP_CURL_ID "SOAP-CURL/1.2"

/** Plugin identification for plugin registry */
extern const char soap_curl_id[];

/**
@brief plugin data to store CURL handle and override callbacks
*/
struct soap_curl_data
{
  struct soap *soap;
  CURL *curl;             /**< CURL handle (passed as arg to plugin or internal) */
  short own;              /**< we own the CURL handle */
  short active;           /**< when true: override IO */
  struct curl_slist *hdr; /**< to add custom HTTP headers */
  char *blk;              /**< current block of data received from CURL stored in blist lst */
  char *ptr;              /**< points to data in blk */
  struct soap_blist *lst; /**< block list with data sent to CURL and received from CURL */
  soap_mode mode;
  char buf[CURL_ERROR_SIZE];
  int (*fconnect)(struct soap*, const char*, const char*, int);
  int (*fsend)(struct soap*, const char*, size_t);
  size_t (*frecv)(struct soap*, char*, size_t);
  int (*fprepareinitrecv)(struct soap*);
  int (*fpreparefinalrecv)(struct soap*);
};

SOAP_FMAC1 int SOAP_FMAC2 soap_curl(struct soap *soap, struct soap_plugin *p, void *arg);
SOAP_FMAC1 void SOAP_FMAC2 soap_curl_reset(struct soap *soap);

#ifdef __cplusplus
}
#endif

#endif
