/*
        httppipe.c

        HTTP pipeline implementation for stand-alone gSOAP servers

        See instructions below.

gSOAP XML Web services tools
Copyright (C) 2000-2018, Robert van Engelen, Genivia, Inc. All Rights Reserved.

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
Copyright (C) 2000-2004 Robert A. van Engelen, Genivia inc. All Rights Reserved.
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

        Compile & link with stand-alone gSOAP server.

        HTTP pipelining allows multiple request messages to be dispatched
        by client applications.  This allows for some overlap of messages send
        and received.  This improves performance marginally and only under
        certain conditions when clients orgestrate the pipeline feed properly.

        A non-multi-threaded client application may be able to sent N requests
        to the server until request N+1 times out:

                client request 1 --> server
                client request 2 --> server
                ...
                client request N --> server
                if time out client request N+1 --> server then
                    client response 1 <-- server
                    client response 2 <-- server
                    ...
                    client response N <-- server
                client request N+1 --> server
                ...
        
        This plugin queues the next pending request(s) received in a buffer so
        that the next server loop iteration in soap_serve() continues to accept
        and serve these pending requests in the pipeline.

        Usage (server side):

        struct soap soap;
        soap_init1(&soap, SOAP_IO_KEEPALIVE);
        soap_register_plugin(&soap, http_pipe);
        ...
        ... = soap_copy(&soap); // copies plugin too
        ...
        soap_done(&soap); // delete plugin
*/

#include "httppipe.h"

#ifdef __cplusplus
extern "C" {
#endif

const char http_pipe_id[] = HTTP_PIPE_ID;

static int http_pipe_init(struct soap *soap, struct http_pipe_data *data);
static void http_pipe_delete(struct soap *soap, struct soap_plugin *p);
static int http_pipe_copy(struct soap *soap, struct soap_plugin *dst, struct soap_plugin *src);
static int http_pipe_init_recv(struct soap *soap);
static int http_pipe_final_recv(struct soap *soap);

int http_pipe(struct soap *soap, struct soap_plugin *p, void *arg)
{
  (void)arg;
  p->id = http_pipe_id;
  p->data = (void*)SOAP_MALLOC(soap, sizeof(struct http_pipe_data));
  p->fdelete = http_pipe_delete;
  p->fcopy = http_pipe_copy;
  if (!p->data)
    return SOAP_EOM;
  if (http_pipe_init(soap, (struct http_pipe_data*)p->data))
  {
    SOAP_FREE(soap, p->data); /* error: could not init */
    return SOAP_EOM; /* return error */
  }
  return SOAP_OK;
}

static int http_pipe_init(struct soap *soap, struct http_pipe_data *data)
{
  data->fprepareinitrecv = soap->fprepareinitrecv; /* save old callback */
  soap->fprepareinitrecv = http_pipe_init_recv; /* replace callback with ours */
  data->fpreparefinalrecv = soap->fpreparefinalrecv; /* save old callback */
  soap->fpreparefinalrecv = http_pipe_final_recv; /* replace callback with ours */
  data->len = 0;
  return SOAP_OK;
}

static void http_pipe_delete(struct soap *soap, struct soap_plugin *p)
{
  soap->fprepareinitrecv = ((struct http_pipe_data*)p->data)->fprepareinitrecv; /* replace callback with ours */
  soap->fpreparefinalrecv = ((struct http_pipe_data*)p->data)->fpreparefinalrecv; /* replace callback with ours */
  SOAP_FREE(soap, p->data); /* free allocated plugin data (this function is not called for shared plugin data, but only when the final soap_done() is invoked on the original soap struct) */
}

static int http_pipe_copy(struct soap *soap, struct soap_plugin *dst, struct soap_plugin *src)
{
  (void)soap;
  /* make a copy of the plugin data */
  dst->data = (void*)SOAP_MALLOC(soap, sizeof(struct http_pipe_data));
  if (!dst->data)
    return SOAP_EOM;
  ((struct http_pipe_data*)dst->data)->fprepareinitrecv = ((struct http_pipe_data*)src->data)->fprepareinitrecv;
  ((struct http_pipe_data*)dst->data)->fpreparefinalrecv = ((struct http_pipe_data*)src->data)->fpreparefinalrecv;
  ((struct http_pipe_data*)dst->data)->len = 0;
  return SOAP_OK;
}

static int http_pipe_init_recv(struct soap *soap)
{
  struct http_pipe_data *data = (struct http_pipe_data*)soap_lookup_plugin(soap, http_pipe_id);
  if (!data)
    return SOAP_PLUGIN_ERROR;
  /* if previous message exchange left data in the receive buffer, use that data */
  if (data->len && soap->keep_alive)
  {
    (void)soap_memcpy(soap->buf, sizeof(soap->buf), data->buf, data->len);
    soap->bufidx = 0;
    soap->buflen = data->len;
    DBGLOG(TEST,SOAP_MESSAGE(fdebug, "HTTP pipeline: restored %lu buffered bytes\n", (unsigned long)data->len));
  }
  data->len = 0;
  if (data->fprepareinitrecv)
    return data->fprepareinitrecv(soap);
  return SOAP_OK;
}

static int http_pipe_final_recv(struct soap *soap)
{
  struct http_pipe_data *data = (struct http_pipe_data*)soap_lookup_plugin(soap, http_pipe_id);
  if (!data)
    return SOAP_PLUGIN_ERROR;
  if (soap->keep_alive)
  {
    /* if message has data left in the receive buffer, save that data for later */
    data->len = soap->buflen - soap->bufidx;
    if (data->len)
    {
      (void)soap_memcpy(data->buf, sizeof(data->buf), soap->buf + soap->bufidx, data->len);
      DBGLOG(TEST,SOAP_MESSAGE(fdebug, "HTTP pipeline: saved %lu buffered bytes\n", (unsigned long)data->len));
    }
  }
  else
  {
    data->len = 0;
  }
  if (data->fpreparefinalrecv)
    return data->fpreparefinalrecv(soap);
  return SOAP_OK;
}

#ifdef __cplusplus
}
#endif
