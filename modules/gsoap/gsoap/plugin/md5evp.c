/*
	md5evp.c

	gSOAP HTTP Content-MD5 digest EVP handler for httpmd5 plugin

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
*/

#include "md5evp.h"

#ifdef __cplusplus
extern "C" {
#endif

int md5_handler(struct soap *soap, void **context, enum md5_action action, char *buf, size_t len)
{ EVP_MD_CTX *ctx;
  unsigned char hash[EVP_MAX_MD_SIZE];
  unsigned int size;
  switch (action)
  { case MD5_INIT:
      soap_ssl_init();
      if (!*context)
      {
#if (OPENSSL_VERSION_NUMBER < 0x10100000L)
        *context = (void*)SOAP_MALLOC(soap, sizeof(EVP_MD_CTX));
        EVP_MD_CTX_init((EVP_MD_CTX*)*context);
#else
        *context = EVP_MD_CTX_new();
#endif
        EVP_MD_CTX_init((EVP_MD_CTX*)*context);
      }
      ctx = (EVP_MD_CTX*)*context;
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "-- MD5 Init %p\n", ctx));
      EVP_DigestInit_ex(ctx, EVP_md5(), NULL);
      break;
    case MD5_UPDATE:
      ctx = (EVP_MD_CTX*)*context;
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "-- MD5 Update %p (%lu) --\n", ctx, (unsigned long)len));
      DBGMSG(TEST, buf, len);
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "\n--"));
      EVP_DigestUpdate(ctx, (const void*)buf, (unsigned int)len);
      break;
    case MD5_FINAL:
      ctx = (EVP_MD_CTX*)*context;
      EVP_DigestFinal_ex(ctx, (unsigned char*)hash, &size);
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "-- MD5 Final %p --\n", ctx));
      DBGHEX(TEST, hash, size);
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "\n--"));
      (void)soap_memcpy((void*)buf, 16, (const void*)hash, 16);
      break;
    case MD5_DELETE:
      ctx = (EVP_MD_CTX*)*context;
      DBGLOG(TEST, SOAP_MESSAGE(fdebug, "-- MD5 Delete %p --\n", ctx));
      if (ctx)
      {
#if (OPENSSL_VERSION_NUMBER < 0x10100000L)
        EVP_MD_CTX_cleanup(ctx);
        SOAP_FREE(soap, ctx);
#else
        EVP_MD_CTX_free(ctx);
#endif
      }
      *context = NULL;
  }
  return SOAP_OK;
}

#ifdef __cplusplus
}
#endif

