/*
	logging.c

	Message logging plugin and message stats collector

	Register the plugin with:
		soap_register_plugin(soap, logging);

	Change logging destinations:
		soap_set_logging_inbound(struct soap*, FILE*);
		soap_set_logging_outbound(struct soap*, FILE*);
	Turn logging off by passing NULL FILE* descriptor.

	To obtain stats (sent and recv byte count):
		soap_logging_stats(soap, size_t *sent, size_t *recv);
        where sent and recv will be set to the number of bytes sent (outbound)
        and received (inbound) in total, respectively.  The stats are collected
        even when inbound and outbound logging is turned off.

        To reset the stats:
                soap_reset_loggin_stats(soap);

gSOAP XML Web services tools
Copyright (C) 2000-2008, Robert van Engelen, Genivia Inc., All Rights Reserved.
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
Copyright (C) 2000-2008, Robert van Engelen, Genivia Inc., All Rights Reserved.
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

#include "logging.h"

#ifdef __cplusplus
extern "C" {
#endif

const char logging_id[] = LOGGING_ID;

static int logging_init(struct soap *soap, struct logging_data *data);
static void logging_delete(struct soap *soap, struct soap_plugin *p);
static int logging_send(struct soap *soap, const char *buf, size_t len);
static size_t logging_recv(struct soap *soap, char *buf, size_t len);

/* plugin registry function, invoked by soap_register_plugin */
SOAP_FMAC1
int
SOAP_FMAC2
logging(struct soap *soap, struct soap_plugin *p, void *arg)
{
  (void)arg;
  p->id = logging_id;
  /* create local plugin data */
  p->data = (void*)SOAP_MALLOC(soap, sizeof(struct logging_data));
  /* register the destructor */
  p->fdelete = logging_delete;
  /* if OK then initialize */
  if (!p->data)
    return SOAP_EOM;
  if (logging_init(soap, (struct logging_data*)p->data))
  {
    SOAP_FREE(soap, p->data); /* error: could not init */
    return SOAP_EOM; /* return error */
  }
  return SOAP_OK;
}

/* set inbound logging FD, NULL to disable */
SOAP_FMAC1
void
SOAP_FMAC2
soap_set_logging_inbound(struct soap *soap, FILE *fd)
{
  struct logging_data *data = (struct logging_data*)soap_lookup_plugin(soap, logging_id);
  if (data)
    data->inbound = fd;
}

/* set outbound logging FD, NULL to disable */
SOAP_FMAC1
void
SOAP_FMAC2
soap_set_logging_outbound(struct soap *soap, FILE *fd)
{
  struct logging_data *data = (struct logging_data*)soap_lookup_plugin(soap, logging_id);
  if (data)
    data->outbound = fd;
}

/* get logging sent and recv octet counts */
SOAP_FMAC1
void
SOAP_FMAC2
soap_logging_stats(struct soap *soap, size_t *sent, size_t *recv)
{
  struct logging_data *data = (struct logging_data*)soap_lookup_plugin(soap, logging_id);
  if (data)
  {
    *sent = data->stat_sent;
    *recv = data->stat_recv;
  }
}

/* reset the logging stats */
SOAP_FMAC1
void
SOAP_FMAC2
soap_reset_logging_stats(struct soap *soap)
{
  struct logging_data *data = (struct logging_data*)soap_lookup_plugin(soap, logging_id);
  if (data)
  {
    data->stat_sent = 0;
    data->stat_recv = 0;
  }
}

/* used by plugin registry function */
static int
logging_init(struct soap *soap, struct logging_data *data)
{
  data->inbound = NULL;
  data->outbound = NULL;
  data->stat_sent = 0;
  data->stat_recv = 0;
  data->fsend = soap->fsend; /* save old recv callback */
  data->frecv = soap->frecv; /* save old send callback */
  soap->fsend = logging_send; /* replace send callback with ours */
  soap->frecv = logging_recv; /* replace recv callback with ours */
  return SOAP_OK;
}

static void
logging_delete(struct soap *soap, struct soap_plugin *p)
{ 
  (void)soap;
  /* free allocated plugin data. If fcopy() is not set, then this function is
     not called for all copies of the plugin created with soap_copy(). In this
     example, the fcopy() callback is omitted and the plugin data is shared by
     the soap copies created with soap_copy() */
  SOAP_FREE(soap, p->data);
}

static size_t
logging_recv(struct soap *soap, char *buf, size_t len)
{
  struct logging_data *data = (struct logging_data*)soap_lookup_plugin(soap, logging_id);
  size_t res;
  /* get data from old recv callback */
  res = data->frecv(soap, buf, len);
  /* update should be in mutex, but we don't mind some inaccuracy in stats */
  data->stat_recv += res;
  if (data->inbound)
    fwrite(buf, res, 1, data->inbound);
  return res;
}

static int
logging_send(struct soap *soap, const char *buf, size_t len)
{
  struct logging_data *data = (struct logging_data*)soap_lookup_plugin(soap, logging_id);
  /* update should be in mutex, but we don't mind some inaccuracy in stats */
  data->stat_sent += len;
  if (data->outbound)
    fwrite(buf, len, 1, data->outbound);
  return data->fsend(soap, buf, len); /* pass data on to old send callback */
}

#ifdef __cplusplus
}
#endif

