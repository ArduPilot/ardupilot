/*

	httpmd5.c

	gSOAP HTTP Content-MD5 digest plugin.

	Usage (both client and server, see httpmd5test.h/.c for example):
	soap_register_plugin(&soap, http_md5);
	This enables HTTP MD5 checksum generation and checking for SOAP/XML messages
	WITHOUT attachments.

	Compile with -DWITH_OPENSSL
	Link with OpenSSL (for md5evp.c), httpmd5.c, and md5evp.c

gSOAP XML Web services tools
Copyright (C) 2000-2005, Robert van Engelen, Genivia Inc., All Rights Reserved.
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
Copyright (C) 2000-2005, Robert van Engelen, Genivia, Inc., All Rights Reserved.
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

	Requires an md5 engine, see md5evp.h

	int md5_handler(struct soap *soap, void **context, enum md5_action action, char *buf, size_t len)
	context can be set and passed to subsequent calls. Parameters:
	action =
	MD5_INIT:	init context
	MD5_UPDATE:	update context with data from buf with size len
	MD5_FINAL:	fill buf with 16 bytes MD5 hash value
	MD5_DELETE:	delete context
	buf		input data, output MD5 128 bit hash value
	len		length of input data

	Example code:
	httpmd5test.h, httpmd5test.c

	Limitations:
	Does not work with combined chunked + compressed messages.
	When sending DIME/MIME attachments, you MUST use the SOAP_IO_STORE flag
	to compute the MD5 hash of the message with attachments. The flag
	disables streaming DIME.
*/

#include "httpmd5.h"

#ifdef __cplusplus
extern "C" {
#endif

const char http_md5_id[18] = HTTP_MD5_ID;

static int http_md5_init(struct soap *soap, struct http_md5_data *data);
static int http_md5_copy(struct soap *soap, struct soap_plugin *dst, struct soap_plugin *src);
static void http_md5_delete(struct soap *soap, struct soap_plugin *p);

static int http_md5_post_header(struct soap *soap, const char *key, const char *val);
static int http_md5_parse_header(struct soap *soap, const char *key, const char *val);
static int http_md5_prepareinitsend(struct soap *soap);
static int http_md5_prepareinitrecv(struct soap *soap);
static int http_md5_preparesend(struct soap *soap, const char *buf, size_t len);
static int http_md5_preparerecv(struct soap *soap, const char *buf, size_t len);
static int http_md5_preparefinalrecv(struct soap *soap);

int http_md5(struct soap *soap, struct soap_plugin *p, void *arg)
{ (void)arg;
  p->id = http_md5_id;
  p->data = (void*)SOAP_MALLOC(soap, sizeof(struct http_md5_data));
  p->fcopy = http_md5_copy;
  p->fdelete = http_md5_delete;
  if (!p->data)
    return SOAP_EOM;
  if (http_md5_init(soap, (struct http_md5_data*)p->data))
  { SOAP_FREE(soap, p->data); /* error: could not init */
    return SOAP_EOM; /* return error */
  }
  return SOAP_OK;
}

static int http_md5_init(struct soap *soap, struct http_md5_data *data)
{ data->fposthdr = soap->fposthdr;
  soap->fposthdr = http_md5_post_header;
  data->fparsehdr = soap->fparsehdr;
  soap->fparsehdr = http_md5_parse_header;
  data->fprepareinitsend = soap->fprepareinitsend;
  soap->fprepareinitsend = http_md5_prepareinitsend;
  data->fprepareinitrecv = soap->fprepareinitrecv;
  soap->fprepareinitrecv = http_md5_prepareinitrecv;
  data->fpreparesend = soap->fpreparesend;
  soap->fpreparesend = http_md5_preparesend;
  data->context = NULL;
  memset((void*)data->digest, 0, sizeof(data->digest));
  return SOAP_OK;
}

static int http_md5_copy(struct soap *soap, struct soap_plugin *dst, struct soap_plugin *src)
{ *dst = *src;
  dst->data = (void*)SOAP_MALLOC(soap, sizeof(struct http_md5_data));
  (void)soap_memcpy((void*)dst->data, sizeof(struct http_md5_data), (const void*)src->data, sizeof(struct http_md5_data));
  ((struct http_md5_data*)dst->data)->context = NULL;
  return SOAP_OK;
}

static void http_md5_delete(struct soap *soap, struct soap_plugin *p)
{ (void)p;
  struct http_md5_data *data = (struct http_md5_data*)soap_lookup_plugin(soap, http_md5_id);
  if (data)
  { md5_handler(soap, &data->context, MD5_DELETE, NULL, 0);
    SOAP_FREE(soap, data);
  }
}

static int http_md5_post_header(struct soap *soap, const char *key, const char *val)
{ struct http_md5_data *data = (struct http_md5_data*)soap_lookup_plugin(soap, http_md5_id);
  char buf64[25]; /* 24 base64 chars + '\0' */
  int err;
  if (!data)
    return SOAP_PLUGIN_ERROR;
  if (!key) /* last line */
  { if ((err = md5_handler(soap, &data->context, MD5_FINAL, data->digest, 0)))
      return err;
    data->fposthdr(soap, "Content-MD5", soap_s2base64(soap, (unsigned char*)data->digest, buf64, 16));
  }
  return data->fposthdr(soap, key, val);
}

static int http_md5_parse_header(struct soap *soap, const char *key, const char *val)
{ struct http_md5_data *data = (struct http_md5_data*)soap_lookup_plugin(soap, http_md5_id);
  if (!data)
    return SOAP_PLUGIN_ERROR;
  if (!soap_tag_cmp(key, "Content-MD5"))
  { soap_base642s(soap, val, data->digest, 16, NULL);
    data->fpreparerecv = soap->fpreparerecv;
    soap->fpreparerecv = http_md5_preparerecv;
    data->fpreparefinalrecv = soap->fpreparefinalrecv;
    soap->fpreparefinalrecv = http_md5_preparefinalrecv;
    md5_handler(soap, &data->context, MD5_INIT, NULL, 0);
    return SOAP_OK;
  }
  return data->fparsehdr(soap, key, val);
}

static int http_md5_prepareinitsend(struct soap *soap)
{ struct http_md5_data *data = (struct http_md5_data*)soap_lookup_plugin(soap, http_md5_id);
  if (!data)
    return SOAP_PLUGIN_ERROR;
  if ((soap->mode & SOAP_IO) != SOAP_IO_STORE && (soap->mode & (SOAP_ENC_DIME | SOAP_ENC_MIME)))
  { /* TODO: handle attachments automatically, does not work yet */
    soap->mode &= ~SOAP_IO;
    soap->mode |= SOAP_IO_STORE;
  }
  else
    md5_handler(soap, &data->context, MD5_INIT, NULL, 0);
  if (data->fprepareinitsend)
    return data->fprepareinitsend(soap);
  return SOAP_OK;
}

static int http_md5_prepareinitrecv(struct soap *soap)
{ struct http_md5_data *data = (struct http_md5_data*)soap_lookup_plugin(soap, http_md5_id);
  if (!data)
    return SOAP_PLUGIN_ERROR;
  if (soap->fpreparerecv == http_md5_preparerecv)
    soap->fpreparerecv = data->fpreparerecv;
  if (soap->fpreparefinalrecv == http_md5_preparefinalrecv)
    soap->fpreparefinalrecv = data->fpreparefinalrecv;
  if (data->fprepareinitrecv)
    return data->fprepareinitrecv(soap);
  return SOAP_OK;
}

static int http_md5_preparesend(struct soap *soap, const char *buf, size_t len)
{ struct http_md5_data *data = (struct http_md5_data*)soap_lookup_plugin(soap, http_md5_id);
  if (!data)
    return SOAP_PLUGIN_ERROR;
  md5_handler(soap, &data->context, MD5_UPDATE, (char*)buf, len);
  if (data->fpreparesend)
    return data->fpreparesend(soap, buf, len);
  return SOAP_OK;
}

static int http_md5_preparerecv(struct soap *soap, const char *buf, size_t len)
{ struct http_md5_data *data = (struct http_md5_data*)soap_lookup_plugin(soap, http_md5_id);
  if (!data)
    return SOAP_PLUGIN_ERROR;
  md5_handler(soap, &data->context, MD5_UPDATE, (char*)buf, len);
  if (data->fpreparerecv)
    return data->fpreparerecv(soap, buf, len);
  return SOAP_OK;
}

static int http_md5_preparefinalrecv(struct soap *soap)
{ struct http_md5_data *data = (struct http_md5_data*)soap_lookup_plugin(soap, http_md5_id);
  char digest[16];
  if (!data)
    return SOAP_PLUGIN_ERROR;
  md5_handler(soap, &data->context, MD5_FINAL, digest, 0);
  soap->fpreparerecv = data->fpreparerecv;
  soap->fpreparefinalrecv = data->fpreparefinalrecv;
  if (memcmp((void*)digest, (const void*)data->digest, sizeof(data->digest)))
    return soap_sender_fault(soap, "MD5 digest mismatch: message corrupted", NULL);
  if (soap->fpreparefinalrecv)
    return soap->fpreparefinalrecv(soap);
  return SOAP_OK;
}

#ifdef __cplusplus
}
#endif

