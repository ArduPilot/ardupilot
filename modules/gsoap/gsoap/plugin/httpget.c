/*
        httpget.c

        gSOAP HTTP GET plugin.

        See instructions below.

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

        Compile & link with stand-alone gSOAP server for HTTP GET support.
        Compile & link with gSOAP clients for client-side HTTP GET requests.

        Usage (server side):

        struct soap soap;
        soap_init(&soap);
        soap_register_plugin_arg(&soap, http_get, http_get_handler);
        ...
        ... = soap_copy(&soap); // copies plugin too but not its data: plugin data is shared since fcopy is not set
        ...
        soap_done(&soap); // detach plugin (calls plugin->fdelete)

        You need to define a HTTP GET handling function at the server-side:

        int http_get_handler(struct soap*)

        which will be called from the plugin upon an HTTP GET request.
        The function should return an error code or SOAP_OK;
        This function should produce a valid HTTP response, for example:

        soap_response(soap, SOAP_HTML); // use this to return HTML ...
        soap_response(soap, SOAP_OK); // ... or use this to return a SOAP message
        ...
        soap_send(soap, "<HTML>...</HTML>"); // example HTML
        ...
        soap_end_send(soap);

        To get query string key-value pairs within a service routine, use:

        char *s;
        s = soap_query(soap);
        while (s)
        {
          char *key = soap_query_key(soap, &s);
          char *val = soap_query_val(soap, &s);
          ...
        }

        This will garble soap->path, which contains the HTTP path of the form:
        /path?query
        where path and/or ?query may be absent. The path info is obtained from
        the HTTP request URL: http://domain/path?query
        The URL should not exceed the length of SOAP_TAGLEN. Adjust SOAP_TAGLEN
        in stdsoap2.h if necessary.

        Usage (client side):

	For SOAP GET method calls, declare the service method with protocol GET:

	//gsoap ns service method-protocol: someMethod GET
        int ns__someMethod(... in-params ..., struct ns__someMethodResponse { ... out-params ... } *);

	and to make the call in your code:

	struct ns__someMethodResponse res;
	soap_call_ns__someMethod(soap, "endpoint", "action", ... in-params ...., &res)

        Client code for non-SOAP REST GET:

        To use general HTTP GET, for example to retrieve an HTML document, use:

        struct soap soap;
        char *buf = NULL;
        size_t len = 0;
        soap_init(&soap);
        if (soap_GET(&soap, endpoint, action)
         || soap_begin_recv(&soap))
          ... connect/recv error ...
        else
          buf = soap_http_get_body(&soap, &len);
        soap_end_recv(&soap);
        ... process data in buf[0..len-1]
        soap_destroy(&soap);
        soap_end(&soap);
        soap_done(&soap);

        The soap_http_get_body() function above returns the HTTP body content
        as a string.
        
*/

#include "httpget.h"

#ifdef __cplusplus
extern "C" {
#endif

const char http_get_id[] = HTTP_GET_ID;

static int http_get_init(struct soap *soap, struct http_get_data *data, int (*handler)(struct soap*));
static void http_get_delete(struct soap *soap, struct soap_plugin *p);
static int http_get_parse(struct soap *soap);
static int http_get_handler(struct soap *soap);

int http_get(struct soap *soap, struct soap_plugin *p, void *arg)
{
  p->id = http_get_id;
  p->data = (void*)SOAP_MALLOC(soap, sizeof(struct http_get_data));
  /* p->fcopy = http_get_copy; obsolete, see note with http_get_copy() */
  p->fdelete = http_get_delete;
  if (!p->data)
    return SOAP_EOM;
  if (http_get_init(soap, (struct http_get_data*)p->data, (int (*)(struct soap*))arg))
  {
    SOAP_FREE(soap, p->data); /* error: could not init */
    return SOAP_EOM; /* return error */
  }
  return SOAP_OK;
}

static int http_get_init(struct soap *soap, struct http_get_data *data, int (*handler)(struct soap*))
{
  data->fparse = soap->fparse; /* save old HTTP header parser callback */
  data->fget = handler;
  data->stat_get = 0;
  data->stat_post = 0;
  data->stat_fail = 0;
  memset((void*)data->hist_min, 0, sizeof(data->hist_min));
  memset((void*)data->hist_hour, 0, sizeof(data->hist_hour));
  memset((void*)data->hist_day, 0, sizeof(data->hist_day));
  soap->fparse = http_get_parse; /* replace HTTP header parser callback with ours */
  soap->fget = http_get_handler; /* replace HTTP GET callback with ours */
  return SOAP_OK;
}

/* We will share the plugin data among all soap copies created with soap_copy(), so we don't have to define this
static int http_get_copy(struct soap *soap, struct soap_plugin *dst, struct soap_plugin *src)
{
 *dst = *src;
  return SOAP_OK;
}
*/

static void http_get_delete(struct soap *soap, struct soap_plugin *p)
{
  (void)soap;
  SOAP_FREE(soap, p->data); /* free allocated plugin data (this function is not called for shared plugin data, but only when the final soap_done() is invoked on the original soap struct) */
}

static int http_get_parse(struct soap *soap)
{
#ifndef WITH_LEAN
  time_t t;
#ifdef HAVE_LOCALTIME_R
  struct tm T;
#endif
  struct tm *pT;
#endif
  struct http_get_data *data = (struct http_get_data*)soap_lookup_plugin(soap, http_get_id);
  if (!data)
    return SOAP_PLUGIN_ERROR;
#ifndef WITH_LEAN
  time(&t);
#ifdef HAVE_LOCALTIME_R
  pT = localtime_r(&t, &T);
#else
  pT = localtime(&t);
#endif
  /* updates should be in mutex, but we don't mind some inaccuracy in the count to preserve performance */
  data->hist_day[pT->tm_yday]++;
  data->hist_day[(pT->tm_yday + 1) % 365] = 0;
  data->hist_hour[pT->tm_hour]++;
  data->hist_hour[(pT->tm_hour + 1) % 24] = 0;
  data->hist_min[pT->tm_min]++;
  data->hist_min[(pT->tm_min + 1) % 60] = 0;
#endif
  soap->error = data->fparse(soap); /* parse HTTP header */
  if (!soap->error)
  {
    if (soap->status == SOAP_POST)
      data->stat_post++; /* update should be in mutex, but we don't mind some inaccuracy in the count */
  }
  else
  {
    data->stat_fail++; /* update should be in mutex, but we don't mind some inaccuracy in the count */
  }
  return soap->error;
}

/******************************************************************************/

static int http_get_handler(struct soap *soap)
{
  struct http_get_data *data = (struct http_get_data*)soap_lookup_plugin(soap, http_get_id);
  if (!data)
    return SOAP_PLUGIN_ERROR;
  soap->error = data->fget(soap); /* call user-defined HTTP GET handler */
  if (soap->error)
  {
    /* update should be in mutex, but we don't mind some inaccuracy in the count */
    data->stat_fail++;
    return soap->error;
  }
  /* update should be in mutex, but we don't mind some inaccuracy in the count */
  data->stat_get++;
  return SOAP_OK;
}

/******************************************************************************/

void soap_http_get_stats(struct soap *soap, size_t *stat_get, size_t *stat_post, size_t *stat_fail, size_t **hist_min, size_t **hist_hour, size_t **hist_day)
{
  struct http_get_data *data = (struct http_get_data*)soap_lookup_plugin(soap, http_get_id);
  if (!data)
    return;
  if (stat_get)
    *stat_get = data->stat_get;
  if (stat_post)
    *stat_post = data->stat_post;
  if (stat_fail)
    *stat_fail = data->stat_fail;
  if (hist_min)
    *hist_min = data->hist_min;
  if (hist_hour)
    *hist_hour = data->hist_hour;
  if (hist_day)
    *hist_day = data->hist_day;
}

/* deprecated: use soap_GET instead */
int soap_http_get_connect(struct soap *soap, const char *endpoint, const char *action)
{
  return soap_GET(soap, endpoint, action);
}

#ifdef __cplusplus
}
#endif

