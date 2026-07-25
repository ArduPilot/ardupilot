/*
	plugin.c

	Example gSOAP plug-in

	Copyright (C) 2000-2002 Robert A. van Engelen. All Rights Reserved.

	Compile & link with gSOAP clients and services to view SOAP messages.

	Usage (client/server code):
	struct soap soap;
	soap_init(&soap);
	soap_register_plugin(&soap, plugin); // register plugin
	...
	... = soap_copy(&soap); // copies plugin too
	...
	soap_done(&soap); // detach plugin

	A plugin is copied with the soap_copy() call. Upon this call, two
	situations may arise depending on setting the fcopy() callback and
	the need to share or not share plugin data:

	1. if the plugin fcopy() callback is set, it will be called to allow
	   the plugin to copy its local data. When soap_done() is called on
	   the copy, the fdelete() callback is called for cleanup.

	2. if the plugin fcopy() callback is not set, then the plugin data
	   will be shared (i.e. the data pointer points to the same address).
	   The fdelete() callback will not be called upon a soap_done() on a
	   copy of the soap struct. The fdelete() callback will be called for
	   the original soap struct with which the plugin registered.
	   
*/

#include "plugin.h"

static const char plugin_id[] = PLUGIN_ID;

static int plugin_init(struct soap *soap, struct plugin_data *data);
static int plugin_copy(struct soap *soap, struct soap_plugin *dst, struct soap_plugin *src);
static void plugin_delete(struct soap *soap, struct soap_plugin *p);
static int plugin_send(struct soap *soap, const char *buf, size_t len);
static size_t plugin_recv(struct soap *soap, char *buf, size_t len);

int plugin(struct soap *soap, struct soap_plugin *p, void *arg)
{
  p->id = plugin_id;
  p->data = (void*)malloc(sizeof(struct plugin_data));
  /* optional: define fcopy() operation. When defined, fdelete() will be called for every copy of the plugin created with fcopy(), when NOT defined, fdelete() will only be called on the original non-copied plugin */
  p->fcopy = plugin_copy;
  p->fdelete = plugin_delete;
  if (!p->data)
    return SOAP_EOM;
  if (plugin_init(soap, (struct plugin_data*)p->data))
  {
    free(p->data); /* error: could not init */
    return SOAP_EOM; /* return error */
  }
  return SOAP_OK;
}

static int plugin_init(struct soap *soap, struct plugin_data *data)
{
  data->fsend = soap->fsend; /* save old recv callback */
  data->frecv = soap->frecv; /* save old send callback */
  soap->fsend = plugin_send; /* replace send callback with ours */
  soap->frecv = plugin_recv; /* replace recv callback with ours */
  return SOAP_OK;
}

static int plugin_copy(struct soap *soap, struct soap_plugin *dst, struct soap_plugin *src)
{
  if (!(dst->data = (struct plugin_data*)malloc(sizeof(struct plugin_data))))
    return SOAP_EOM;
  *dst->data = *src->data;
  return SOAP_OK;
}

static void plugin_delete(struct soap *soap, struct soap_plugin *p)
{
  free(p->data); /* free allocated plugin data. If fcopy() is not set, then this function is not called for all copies of the plugin created with soap_copy(). In this example, the fcopy() callback can be safely omitted. When omitted, the plugin data is shared by the soap copies created with soap_copy() */
}

static int plugin_send(struct soap *soap, const char *buf, size_t len)
{
  struct plugin_data *data = (struct plugin_data*)soap_lookup_plugin(soap, plugin_id);
  fwrite(buf, len, 1, stderr);
  return data->fsend(soap, buf, len); /* pass data on to old send callback */
}

static size_t plugin_recv(struct soap *soap, char *buf, size_t len)
{
  struct plugin_data *data = (struct plugin_data*)soap_lookup_plugin(soap, plugin_id);
  size_t res = data->frecv(soap, buf, len); /* get data from old recv callback */
  fwrite(buf, res, 1, stderr);
  return res;
}
