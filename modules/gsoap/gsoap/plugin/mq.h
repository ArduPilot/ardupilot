/*
	mq.h

	Inbound message queues

gSOAP XML Web services tools
Copyright (C) 2000-2013, Robert van Engelen, Genivia Inc., All Rights Reserved.
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
Copyright (C) 2000-2013, Robert van Engelen, Genivia Inc., All Rights Reserved.
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

#ifndef MQ_H
#define MQ_H

#include "stdsoap2.h"

#ifdef __cplusplus
extern "C" {
#endif

#define SOAP_MQ_ID "MQ-0.9"

extern const char soap_mq_id[];

/**
@brief Plugin data.
*/
struct soap_mq_data {
  char *buf; /** buffer to read via frecv() */
  size_t len; /** buffer length */
  size_t (*frecv)(struct soap*, char*, size_t); /** to save and use recv callback */
};

/**
@brief Queued inbound message (in linked list).
*/
struct soap_mq_msg {
  char *buf;		/**< inbound HTTP body */
  size_t len;		/**< inbound HTTP body length */
  struct soap soap;	/**< saved context to read HTTP body */
  struct soap_mq_msg *next;
};

/**
@brief Message queue.
*/
struct soap_mq_queue {
  struct soap_mq_msg *head, *tail;
};

SOAP_FMAC1 int SOAP_FMAC2 soap_mq(struct soap *soap, struct soap_plugin *plugin, void *arg);

SOAP_FMAC1 struct soap_mq_queue * SOAP_FMAC2 soap_mq_queue(struct soap *);

SOAP_FMAC1 struct soap_mq_msg * SOAP_FMAC2 soap_mq_get(struct soap *soap, struct soap_mq_queue *);

SOAP_FMAC1 struct soap_mq_msg * SOAP_FMAC2 soap_mq_begin(struct soap_mq_queue *);

SOAP_FMAC1 struct soap_mq_msg * SOAP_FMAC2 soap_mq_next(struct soap_mq_msg *);

SOAP_FMAC1 void SOAP_FMAC2 soap_mq_del(struct soap_mq_queue *, struct soap_mq_msg *);

#ifdef __cplusplus
}
#endif

#endif

