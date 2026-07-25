/*
	plugin.h

	Example gSOAP plug-in. Include this file and link with plugin.c

	Copyright (C) 2000-2002 Robert A. van Engelen. All Rights Reserved.
*/

#include "stdsoap2.h"

#define PLUGIN_ID "SOAP-PLUGIN/1.0" /* plugin identification */

#ifdef __cplusplus
extern "C" {
#endif

struct plugin_data {
  int (*fsend)(struct soap*, const char*, size_t); /* example: to save and use send callback */
  size_t (*frecv)(struct soap*, char*, size_t); /* example: to save and use recv callback */
};

int plugin(struct soap *soap, struct soap_plugin *plugin, void *arg);

#ifdef __cplusplus
}
#endif

