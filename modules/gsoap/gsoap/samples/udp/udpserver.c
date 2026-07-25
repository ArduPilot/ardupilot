/*
	udpserver.c

	SOAP-over-UDP demo server with zlib compression and WS-Addressing

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2001-2011, Robert van Engelen, Genivia, Inc. All Rights Reserved.
This software is released under one of the following two licenses:
GPL.
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
--------------------------------------------------------------------------------
A commercial use license is available from Genivia, Inc., contact@genivia.com
--------------------------------------------------------------------------------
*/

#include "soapH.h"
#include "udp.nsmap"

#define MAX_HISTORY (1000)

char *received_id[MAX_HISTORY];
int last_received;
void init_received();
int check_received(const char *id);

int id_count = 1;

#define MULTICAST_GROUP (NULL) /* use a group IP such as "225.0.0.37" */
#define PORT 10000

int main()
{ struct soap soap;
  init_received();
  soap_init1(&soap, SOAP_IO_UDP);
  /* reuse address */
  soap.bind_flags = SO_REUSEADDR;
  /* bind */
  if (!soap_valid_socket(soap_bind(&soap, NULL, PORT, 100)))
  { soap_print_fault(&soap, stderr);
    exit(1);
  }
  /* optionally join a multicast group */
  if (MULTICAST_GROUP)
  { struct ip_mreq mreq;
    mreq.imr_multiaddr.s_addr = inet_addr(MULTICAST_GROUP);
    mreq.imr_interface.s_addr = htonl(INADDR_ANY);
    if (setsockopt(soap.socket, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mreq, sizeof(mreq)) < 0)
      exit(1);
  }
  /* serve requests */
  for (;;)
  { printf("Accepting requests\n");
    /* accept (not really needed for UDP, so can be omitted) */
    if (!soap_valid_socket(soap_accept(&soap)))
    { soap_print_fault(&soap, stderr);
      exit(1);
    }
    if (soap_serve(&soap))
      soap_print_fault(&soap, stderr);
    soap_destroy(&soap);
    soap_end(&soap);
  }
  soap_done(&soap);
  return 0;
}

void init_received()
{ int i;
  for (i = 0; i < MAX_HISTORY; i++)
    received_id[i] = NULL;
  last_received = 0;
}

int check_received(const char *id)
{ int i;
  if (!id)
    return 1;
  /* Check if Message ID already received */
  for (i = 0; i < MAX_HISTORY; i++)
  { if (received_id[i] && !strcmp(id, received_id[i]))
      return 1;
  }
  if (received_id[last_received])
    free(received_id[last_received]);
  received_id[last_received++] = strdup(id);
  /* Wrap to overwrite old IDs */
  if (last_received >= MAX_HISTORY)
    last_received = 0;
  return 0;
}

int check_header(struct soap *soap)
{ /* MUST have received a SOAP Header */
  if (!soap->header)
    return soap_sender_fault(soap, "No SOAP header", NULL);
  /* ... with a Message ID */
  if (!soap->header->wsa__MessageID)
    return soap_sender_fault(soap, "No WS-Addressing MessageID", NULL);
  soap->header->wsa__RelatesTo = (struct wsa__Relationship*)soap_malloc(soap, sizeof(struct wsa__Relationship));
  soap_default_wsa__Relationship(soap, soap->header->wsa__RelatesTo);
  soap->header->wsa__RelatesTo->__item = soap->header->wsa__MessageID;
  return SOAP_OK;
}

int ns__echoString(struct soap *soap, char *str, char **res)
{ char port[16];

  /* Get Header info and setup response Header */
  if (check_header(soap))
  { printf("Malformed header\n");
    return SOAP_FAULT; /* there was a problem */
  }

  /* If message with MessageID already received, ignore it */
  if (check_received(soap->header->wsa__MessageID))
  { printf("Request message %s already received\n", soap->header->wsa__MessageID);
    return SOAP_STOP;
  }

  /* Get name of client */
  getnameinfo((struct sockaddr*)&soap->peer, soap->peerlen, soap->host, sizeof(soap->host), port, 16, NI_DGRAM | NI_NAMEREQD | NI_NUMERICSERV);

  printf("Request message %s from %s:%s accepted\n", soap->header->wsa__MessageID, soap->host, port);

  /* Check ReplyTo has Address */
  if (!soap->header->wsa__ReplyTo || !soap->header->wsa__ReplyTo->Address)
    return soap_sender_fault(soap, "No WS-Addressing ReplyTo address", NULL);
  /* Copy Header info into response Header */
  soap->header->wsa__To = soap->header->wsa__ReplyTo->Address;

  /* Add info to response Header */
  soap->header->wsa__MessageID = soap_strdup(soap, soap_int2s(soap, id_count++));
  soap->header->wsa__Action = "http://genivia.com/udp/echoStringResponse";

  /* Copy request string into response string */
  printf("Response message %s returned\n", soap->header->wsa__MessageID);
  *res = str;

  return SOAP_OK;
}

int ns__sendString(struct soap *soap, char *str)
{ if (check_header(soap))
    printf("Malformed header\n");
  else if (check_received(soap->header->wsa__MessageID))
    printf("One-way message %s already received\n", soap->header->wsa__MessageID);
  else
    printf("One-way message %s accepted and serviced\n", soap->header->wsa__MessageID);
  return SOAP_OK;
}

int SOAP_ENV__Fault(struct soap *soap, char *faultcode, char *faultstring, char *faultactor, struct SOAP_ENV__Detail *detail, struct SOAP_ENV__Code *SOAP_ENV__Code, struct SOAP_ENV__Reason *SOAP_ENV__Reason, char *SOAP_ENV__Node, char *SOAP_ENV__Role, struct SOAP_ENV__Detail *SOAP_ENV__Detail)
{
  printf("Received one-way SOAP Fault message:\n");
  /* populate the fault struct from the operation arguments to print it */
  soap_fault(soap);
  /* SOAP 1.1 */
  soap->fault->faultcode = faultcode;
  soap->fault->faultstring = faultstring;
  soap->fault->faultactor = faultactor;
  soap->fault->detail = detail;
  /* SOAP 1.2 */
  soap->fault->SOAP_ENV__Code = SOAP_ENV__Code;
  soap->fault->SOAP_ENV__Reason = SOAP_ENV__Reason;
  soap->fault->SOAP_ENV__Node = SOAP_ENV__Node;
  soap->fault->SOAP_ENV__Role = SOAP_ENV__Role;
  soap->fault->SOAP_ENV__Detail = SOAP_ENV__Detail;
  /* set error */
  soap->error = SOAP_FAULT;
  soap_print_fault(soap, stdout);
  return soap_send_empty_response(soap, SOAP_OK); /* HTTP 202 Accepted */
}

