/*
        mq.c

        Inbound message queues to support WS-RM NoDiscard behavior

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

/**

@mainpage

- @ref mq_0 documents the inbound message queue plugin.

*/

/**

@page mq_0 The mq plugin for inbound message queueing and message replay

The inbound message queueing plugin can be used to queue messages that should
not be discarded with the WS-RM protocol's NoDiscard behavior. Messages that
are out of sequence as per WS-RM protocol and should be handled by one thread
(or a thread pool) should be queued for later replay and service operation
invocation. If an unlimited number of threads is available, the simplest WS-RM
protocol NoDiscard behavior is implemented by starting a thread for each
inbound message and letting the thread block with the
`soap_wsrm_check_and_wait()` or
`soap_wsrm_check_send_empty_response_and_wait()` calls. However, that approach
is not efficient with HTTP keep-alive because the next messages on the
keep-alive socket will be blocked from being processed.  This plugin is
designed to process messages on an HTTP keep-alive socket even when operations
block.

@section mq_1 Server-Side Queueing of One-Way Messages

Queueing one-way messages for internal replay is implemented with the message
queueing plugin as follows, by queueing inbound messages received on a single
socket and then replaying them all in sequence as received from the socket:

~~~{.cpp}
#include "mq.h"

int main()
{
  struct soap *soap = soap_new1(SOAP_IO_KEEPALIVE);
  soap_register_plugin(soap, soap_mq);
  ...
  // initializations, port bind etc.
  ...
  while (soap_valid_socket(soap_accept(soap)))
  {
    // queue all messages on this socket (socket is HTTP keep alive)
    // for each message received, we immediately respond with HTTP 202 Accepted
    struct ms_queue *queue = soap_mq_queue(soap);
    struct ms_msg *msg;
    while (soap_mq_get(soap, queue))
      soap_send_empty_response(soap, 202); // 202 Accepted

    // we now internally replay all messages to invoke services
    // services are assumed to NOT send a response message back
    // i.e. one-way operations
    for (msg = soap_mq_begin(queue); msg; msg = soap_mq_next(msg))
      soap_serve(&msg->soap);

    // delete all queued messages, this also calls the following functions on each queued msg context:
    //   soap_destroy(&msg->soap);
    //   soap_end(&msg->soap);
    //   soap_done(&msg->soap);
    soap_mq_del(queue, NULL);

    // delete the queue (allocated in current context)
    soap_destroy(soap);
    soap_end(soap);
  }
  ...
  // finalize
  ...
  soap_free(soap);
}
~~~

Alternatively, it is also possible to call `soap_mq_del(queue, msg)` after
`soap_serve(&msg->soap)` to immediately delete the message after processing.
Calling `soap_mq_next(msg)` for the next loop iteration is still valid, of
course.

@section mq_2 WS-RM Server-Side Message Queueing for NoDiscard Behavior with Callback Services

When messages are controlled by the WS-ReliableMessaging protocol, we can keep
the WS-RM messages in a queue that were received out of order until the order
is restored and queued messages can be dispatched. This WS-RM behavior is
desirable with WS-RM NoDiscard. To implement this approach, we use an inbound
message queue for each socket accepted and processed by a thread.

~~~{.cpp}
#include "wsaapi.h"
#include "wsrmapi.h"
#include "mq.h"
#include "threads.h"

int main()
{
  struct soap *soap = soap_new1(SOAP_IO_KEEPALIVE);
  soap_register_plugin(soap, soap_wsa);
  soap_register_plugin(soap, soap_wsrm);
  soap_register_plugin(soap, soap_mq);
  ...
  // initializations, port bind etc.
  ...
  while (soap_valid_socket(soap_accept(soap)))
  {
    THREAD_TYPE tid;
    struct soap *tsoap = soap_copy(soap);
    if (!tsoap)
      soap_closesock(soap);
    else
      while (THREAD_CREATE(&tid, (void*(*)(void*))process_request, (void*)tsoap))
        sleep(1);
  }
  ...
  // finalize
  ...
  soap_free(soap);
}

void *process_request(void *tsoap)
{
  struct soap *soap = (struct soap*)tsoap;
  struct ms_queue *queue = soap_mq_queue(soap);
  struct ms_msg *msg;
  struct soap ctx;
  while ((msg = soap_mq_get(soap, queue)) != NULL)
  {
    // parse the message headers, if NoDiscard then keep message in queue to retry later
    // copy the context, since we want to preserve the original to retry later
    soap_copy_context(&ctx, &msg->soap);

    if (soap_begin_serve(&ctx))
    {
      soap_send_fault(&ctx); // send fault, close socket
      soap_mq_del(queue, msg); // delete message from queue
    }
    else if (!ctx.header || !ctx.header->wsrm__Sequence)
    {
      // this is not a WS-RM message, so serve immediately
      soap_serve(&msg->soap); // service operations
      soap_mq_del(queue, msg); // delete message from queue
    }
    else if (!soap_wsrm_check(&ctx))
    {
      // check is OK, process this WS-RM message now
      soap_serve(&msg->soap); // service operations SHOULD NOT call soap_wsrm_check()
      soap_mq_del(queue, msg); // delete message from queue
    }
    else if (ctx.error != SOAP_STOP)
    {
      // check failed, not a WS-RM message or other WS-RM error
      soap_send_fault(&ctx); // send fault, close socket
      soap_mq_del(queue, msg); // delete message from queue
    }
    soap_destroy(&ctx);
    soap_end(&ctx);
    soap_done(&ctx);
  }
  // as long as the queue is not empty and WS-RM sequence(s) not terminated, keep trying
  while ((msg = soap_mq_begin(queue)) != NULL)
  {
    // process queued WS-RM messages
    for (; msg != NULL; msg = soap_mq_next(msg))
    {
      // try next message in queue
      soap_copy_context(&ctx, &msg->soap);
      if (!soap_begin_serve(&ctx) && !soap_wsrm_check(&ctx))
      {
        // check is OK, process message
        soap_serve(&msg->soap);
        soap_mq_del(queue, msg);
      }
      else if (ctx.error != SOAP_STOP)
        soap_mq_del(queue, msg);
      soap_destroy(&ctx);
      soap_end(&ctx);
      soap_done(&ctx);
    }
    sleep(1); // sleep some before around we go again
  }
  return NULL;
}
~~~

In the first loop that runs over the messages received on the same keep-alive
socket, the messages will be processed and services dispatched immediately for
non-WS-RM messages and when the WS-RM check succeeds. This check is done in
the server dispatch loop as shown, which means that WS-RM-based service
operations SHOULD NOT call soap_wsrm_check() again. WS-RM messages that cannot
be processed yet since they are out of the sequence order will remain in the
queue.

The second loop over the queued messages will retry to dispatch service
operations according to the WS-RM message order as required by WS-RM NoDiscard
sequence behavior. The loop will run until the queue is empty or when the WS-RM
sequences are closed/terminated.

*/

#include "mq.h"

#ifdef __cplusplus
extern "C" {
#endif

const char soap_mq_id[] = SOAP_MQ_ID;

/******************************************************************************\
 *
 *      Static protos
 *
\******************************************************************************/

static int soap_mq_init(struct soap *soap, struct soap_mq_data *data);
static void soap_mq_delete(struct soap *soap, struct soap_plugin *p);
static size_t soap_mq_recv(struct soap *soap, char *buf, size_t len);
static int soap_mq_serveloop(struct soap *soap);
static void soap_mq_set(struct soap_mq_msg *msg);

/******************************************************************************\
 *
 *      Plugin registry functions
 *
\******************************************************************************/

/** plugin registry function, invoked by soap_register_plugin */
SOAP_FMAC1
int
SOAP_FMAC2
soap_mq(struct soap *soap, struct soap_plugin *p, void *arg)
{
  (void)soap; (void)arg;
  p->id = soap_mq_id;
  /* create local plugin data */
  p->data = (void*)SOAP_MALLOC(soap, sizeof(struct soap_mq_data));
  /* register the destructor */
  p->fdelete = soap_mq_delete;
  /* if OK then initialize */
  if (!p->data)
    return SOAP_EOM;
  if (soap_mq_init(soap, (struct soap_mq_data*)p->data))
  {
    SOAP_FREE(soap, p->data); /* error: could not init */
    return SOAP_EOM; /* return error */
  }
  return SOAP_OK;
}

/******************************************************************************/

/* used by plugin registry function */
static int
soap_mq_init(struct soap *soap, struct soap_mq_data *data)
{
  (void)soap;
  data->buf = NULL;
  data->len = 0;
  return SOAP_OK;
}

/******************************************************************************/

static void
soap_mq_delete(struct soap *soap, struct soap_plugin *p)
{ 
  /* free allocated plugin data. If fcopy() is not set, then this function is
     not called for all copies of the plugin created with soap_copy(). In this
     example, the fcopy() callback is omitted and the plugin data is shared by
     the soap copies created with soap_copy() */
  (void)soap;
  SOAP_FREE(soap, p->data);
}

/******************************************************************************\
 *
 *      Callbacks registered by plugin
 *
\******************************************************************************/

static size_t
soap_mq_recv(struct soap *soap, char *buf, size_t len)
{
  struct soap_mq_data *data = (struct soap_mq_data*)soap_lookup_plugin(soap, soap_mq_id);
  if (!data)
  {
    soap->error = SOAP_PLUGIN_ERROR;
    return 0;
  }
  if (data->len < len)
  {
    len = data->len;
    data->len = 0;
  }
  (void)soap_memcpy(buf, len, data->buf, len);
  data->buf += len;
  return len;
}

/******************************************************************************/

static int
soap_mq_serveloop(struct soap *soap)
{
  soap->keep_alive = 0;
  return soap->error = SOAP_STOP;
}

/******************************************************************************\
 *
 *      Queue Operations
 *
\******************************************************************************/

/**
@fn struct soap_mq_queue *soap_mq_queue(struct soap *soap)
@brief Create a new queue structure allocated in the current context.
Will be deallocated with soap_end(soap). Use soap_mq_get() to receive a message from
the current socket to add to the queue.
@param soap current context
@return pointer to the queue structure
*/
SOAP_FMAC1
struct soap_mq_queue *
SOAP_FMAC2
soap_mq_queue(struct soap *soap)
{
  struct soap_mq_queue *mq = (struct soap_mq_queue*)soap_malloc(soap, sizeof(struct soap_mq_queue));
  if (mq)
    mq->head = mq->tail = NULL;
  return mq;
}

/******************************************************************************/

/**
@fn struct soap_mq_msg *soap_mq_get(struct soap *soap, struct soap_mq_queue *mq)
@brief Receive message from socket and queue it at the end of the queue.
@param soap current context
@param mq pointer to the message queue structure created by soap_mq_queue()
@return pointer to the message received and queued, or NULL
*/
SOAP_FMAC1
struct soap_mq_msg *
SOAP_FMAC2
soap_mq_get(struct soap *soap, struct soap_mq_queue *mq)
{
  struct soap_mq_msg *msg;
  if (soap_begin_recv(soap))
    return NULL;
  msg = (struct soap_mq_msg*)soap_malloc(soap, sizeof(struct soap_mq_msg));
  if (!msg)
  {
    soap->error = SOAP_EOM;
    return NULL;
  }
  msg->next = NULL;
  soap_copy_context(&msg->soap, soap);
  msg->buf = soap_http_get_body(soap, &msg->len);
  if (!msg->buf || soap_end_recv(soap))
    return NULL;
  if (!msg->buf)
    return NULL;
  soap_mq_set(msg);
  if (!mq->head)
    mq->head = mq->tail = msg;
  else
    mq->tail = mq->tail->next = msg;
  return msg;
}

/******************************************************************************/

/**
@fn struct soap_mq_msg *soap_mq_begin(struct soap_mq_queue *mq)
@brief Get first message in queue. Use msg->soap to invoke service from the
queued message, as in soap_serve(&msg->soap).
@param mq pointer to the message queue structure created by soap_mq_queue()
@return pointer to first message in the queue, or NULL
*/
SOAP_FMAC1
struct soap_mq_msg *
SOAP_FMAC2
soap_mq_begin(struct soap_mq_queue *mq)
{
  struct soap_mq_msg *msg = mq->head;
  if (msg)
    soap_mq_set(msg);
  return msg;
}

/******************************************************************************/

/**
@fn struct soap_mq_msg *soap_mq_next(struct soap_mq_msg *msg)
@brief Get next message in queue. Use msg->soap to invoke service from the
queued message, as in soap_serve(&msg->soap).
@param msg pointer to current message in the queue
@return pointer to next message in the queue, or NULL
*/
SOAP_FMAC1
struct soap_mq_msg *
SOAP_FMAC2
soap_mq_next(struct soap_mq_msg *msg)
{
  if (msg)
    msg = msg->next;
  if (msg)
    soap_mq_set(msg);
  return msg;
}

/******************************************************************************/

/**
@fn void soap_mq_del(struct soap_mq_queue *mq, struct soap_mq_msg *msg)
@brief Delete message from queue, e.g. after processing it. Delete entire queue
when msg==NULL. Note: structures will be deallocated with deallocation with
soap_end().
@param mq pointer to the queue structure
@param msg pointer to a message in the queue, when NULL delete entire queue
*/
SOAP_FMAC1
void
SOAP_FMAC2
soap_mq_del(struct soap_mq_queue *mq, struct soap_mq_msg *msg)
{
  if (mq)
  {
    struct soap_mq_msg *p = mq->head;
    if (msg)
    {
      if (p == msg)
        mq->head = msg->next;
      else
      {
        while (p && p->next != msg)
          p = p->next;
        if (p)
        {
          p->next = msg->next;
          if (mq->tail == msg)
            mq->tail = p;
        }
      }
      soap_destroy(&msg->soap);
      soap_end(&msg->soap);
      soap_done(&msg->soap);
    }
    else
    {
      while (p)
      {
        soap_destroy(&p->soap);
        soap_end(&p->soap);
        soap_done(&p->soap);
        p = p->next;
      }
      mq->head = mq->tail = NULL;
    }
  }
}

/******************************************************************************/

static void
soap_mq_set(struct soap_mq_msg *msg)
{
  struct soap_mq_data *data = (struct soap_mq_data*)soap_lookup_plugin(&msg->soap, soap_mq_id);
  if (data)
  {
    data->buf = msg->buf;
    data->len = msg->len;
    soap_clr_imode(&msg->soap, SOAP_IO_CHUNK | SOAP_ENC_ZLIB | SOAP_ENC_SSL);
    msg->soap.frecv = soap_mq_recv;
    msg->soap.fserveloop = soap_mq_serveloop;
  }
}

/******************************************************************************/

#ifdef __cplusplus
}
#endif
