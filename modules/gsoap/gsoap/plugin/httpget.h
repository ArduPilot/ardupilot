/*
	httpget.h

	gSOAP HTTP GET plugin.

	See httpget.c for usage instructions.

gSOAP XML Web services tools
Copyright (C) 2000-2008, Robert van Engelen, Genivia Inc., All Rights Reserved.
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
Copyright (C) 2000-2008 Robert A. van Engelen, Genivia inc. All Rights Reserved.
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
*/

#ifndef HTTPGET_H
#define HTTPGET_H

#include "stdsoap2.h"

#ifdef __cplusplus
extern "C" {
#endif

#define HTTP_GET_ID "SOAP-HTTP-GET/2.1" /* plugin identification */

extern const char http_get_id[];

/* This is the local plugin data shared among all copies of the soap struct: */
struct http_get_data
{
  int (*fparse)(struct soap*); /* to save and call the internal HTTP header parser */
  int (*fget)(struct soap*); /* user-defined server-side HTTP GET handler */
  size_t stat_get;  /* HTTP GET usage statistics */
  size_t stat_post; /* HTTP POST usage statistics */
  size_t stat_fail; /* HTTP failure statistics */
  size_t hist_min[60]; /* Hits by the minute */
  size_t hist_hour[24]; /* Hits by the hour */
  size_t hist_day[366]; /* Hits by day */
};

int http_get(struct soap*, struct soap_plugin*, void*);
int soap_http_get_connect(struct soap*, const char*, const char*);

void soap_http_get_stats(struct soap *soap, size_t *stat_get, size_t *stat_post, size_t *stat_fail, size_t **hist_min, size_t **hist_hour, size_t **hist_day);

#ifdef __cplusplus
}
#endif

#endif
