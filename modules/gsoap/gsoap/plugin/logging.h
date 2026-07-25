/*
	logging.h

	Message logging plugin and message stats collector

	Register the plugin with:
	        #include "plugin/logging.h"
		soap_register_plugin(soap, logging);

	Set or change logging destinations:
		soap_set_logging_inbound(struct soap*, FILE*);
		soap_set_logging_outbound(struct soap*, FILE*);
	Turn logging off by passing NULL FILE* descriptor.

	To obtain stats (sent and recv byte count):
		soap_get_logging_stats(soap, size_t *sent, size_t *recv);
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

#ifndef LOGGING_H
#define LOGGING_H

#include "stdsoap2.h"

#ifdef __cplusplus
extern "C" {
#endif

#define LOGGING_ID "SOAP-LOGGING/1.4"

extern const char logging_id[];

struct logging_data {
  FILE *inbound;
  FILE *outbound;
  size_t stat_sent;
  size_t stat_recv;
  int (*fsend)(struct soap*, const char*, size_t); /* to save and use send callback */
  size_t (*frecv)(struct soap*, char*, size_t); /* to save and use recv callback */
};

SOAP_FMAC1 int SOAP_FMAC2 logging(struct soap *soap, struct soap_plugin *plugin, void *arg);
SOAP_FMAC1 void SOAP_FMAC2 soap_set_logging_inbound(struct soap *soap, FILE *fd);
SOAP_FMAC1 void SOAP_FMAC2 soap_set_logging_outbound(struct soap *soap, FILE *fd);
SOAP_FMAC1 void SOAP_FMAC2 soap_logging_stats(struct soap *soap, size_t *sent, size_t *recv);
SOAP_FMAC1 void SOAP_FMAC2 soap_reset_logging_stats(struct soap *soap);

#ifdef __cplusplus
}
#endif

#endif
