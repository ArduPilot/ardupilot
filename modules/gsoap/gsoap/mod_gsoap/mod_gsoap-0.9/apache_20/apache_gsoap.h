/**
  * Interface between the apache http - server (http://httpd.apache.org) and
  * the gsoap stack (http://genivia.com)
  * @file apache_gsoap.h
  */

#ifndef _APACHE_GSOAP_H_INCLUDED
#define _APACHE_GSOAP_H_INCLUDED

#undef HAVE_TIMEGM /* stop complaining */

/*
 * need to include httpd.h for request rec definition 
 */
#include <httpd.h>

#ifdef __cplusplus
extern "C" {
#endif

#define APACHE_GSOAP_INTERFACE_VERSION 7
#define APACHE_HTTPSERVER_ENTRY_POINT "apache_init_soap_interface"

/*
 * calls soap_serve inside shared library 
 */
typedef SOAP_FMAC1 void (SOAP_FMAC2 *apache_soap_init_fn)(struct soap *, request_rec *);

/*
 * calls soap_init inside shared library
 */
typedef SOAP_FMAC1 int (SOAP_FMAC2 *apache_soap_serve_fn)(struct soap *, request_rec *);

/*
 * calls soap_destroy inside shared library 
 */
typedef SOAP_FMAC1 void (SOAP_FMAC2 *apache_soap_destroy_fn)(struct soap *, request_rec *);

/*
 * calls soap_end inside shared library 
 */
typedef SOAP_FMAC1 void (SOAP_FMAC2 * apache_soap_end_fn) (struct soap *, request_rec *);

/*
 * calls soap_done inside shared library 
 */
typedef SOAP_FMAC1 void (SOAP_FMAC2 * apache_soap_done_fn) (struct soap *, request_rec *);

typedef SOAP_FMAC1 int (SOAP_FMAC2 * apache_soap_register_plugin_fn)(struct soap *, int (*fcreate) (struct soap *, struct soap_plugin *, void *), void *arg, request_rec *);

typedef SOAP_FMAC1 void *SOAP_FMAC2(*apache_soap_lookup_plugin_fn)(struct soap *, const char *, request_rec *);

/*
 * the callbacks normally used in the apache_soap_interface 
 */
SOAP_FMAC1 void SOAP_FMAC2 apache_soap_soap_destroy(struct soap *, request_rec * r);
SOAP_FMAC1 void SOAP_FMAC2 apache_default_soap_init(struct soap *soap, request_rec * r);
SOAP_FMAC1 int SOAP_FMAC2 apache_default_soap_serve(struct soap *soap, request_rec * r);
SOAP_FMAC1 void SOAP_FMAC2 apache_default_soap_end(struct soap *soap, request_rec * r);
SOAP_FMAC1 void SOAP_FMAC2 apache_default_soap_done(struct soap *soap, request_rec * r);
SOAP_FMAC1 int SOAP_FMAC2 apache_default_soap_register_plugin_arg(struct soap *, int (*fcreate) (struct soap *, struct soap_plugin *, void *), void *arg, request_rec *);
SOAP_FMAC1 void *SOAP_FMAC2 apache_default_soap_lookup_plugin(struct soap *soap, const char *plugin, request_rec *);

struct apache_soap_interface
{
    unsigned int len;       /* length of this struct in bytes (for version control). */
    unsigned int interface_version;
    apache_soap_init_fn fsoap_user_init;
    apache_soap_init_fn fsoap_init;
    apache_soap_serve_fn fsoap_serve;
    apache_soap_destroy_fn fsoap_destroy;
    apache_soap_end_fn fsoap_end;
    apache_soap_done_fn fsoap_done;
    apache_soap_register_plugin_fn fsoap_register_plugin_arg;
    apache_soap_lookup_plugin_fn fsoap_lookup_plugin;
    void *reserved;         /* variable reserved for apache module, must not be changed by server shared library. */
    struct Namespace *namespaces;
    void (*soap_serializeheader) (struct soap *soap);
    int (*soap_putheader) (struct soap *soap);
    int (*soap_getheader) (struct soap *soap);
    void (*soap_fault) (struct soap *soap);
    void (*soap_serializefault) (struct soap *soap);
    int (*soap_putfault) (struct soap *soap);
    int (*soap_getfault) (struct soap *soap);
    const char **(*soap_faultcode) (struct soap *soap);
    const char **(*soap_faultstring) (struct soap *soap);
    const char **(*soap_faultdetail) (struct soap *soap);
    void (*soap_markelement) (struct soap *soap, const void *, int);
    int (*soap_putelement) (struct soap *soap, const void *, const char *, int, int);
    void *(*soap_getelement) (struct soap *soap, const char *, int *);
};

typedef void (*apache_init_soap_interface_fn) (struct apache_soap_interface*, request_rec*);

/*
 * exported shared library function called by mod_gsoap from within apache http server 
 *  This function fills the members of the apache_soap_interface struct. 
 */
SOAP_FMAC1 void SOAP_FMAC2 apache_init_soap_interface(struct apache_soap_interface*, request_rec*);

#define IMPLEMENT_GSOAP_SERVER_INIT(INIT) \
  SOAP_FMAC1 void SOAP_FMAC2 apache_default_soap_init(struct soap *soap, request_rec *r) \
    { soap_init(soap); }\
  SOAP_FMAC1 void SOAP_FMAC2 apache_soap_soap_destroy(struct soap *soap, request_rec *r) \
    { soap_destroy(soap); }\
  SOAP_FMAC1 int SOAP_FMAC2 apache_default_soap_serve(struct soap *soap, request_rec *r) \
    { return soap_serve(soap); }\
  SOAP_FMAC1 void SOAP_FMAC2 apache_default_soap_end(struct soap *soap, request_rec *r) \
    { soap_end(soap); }\
  SOAP_FMAC1 void SOAP_FMAC2 apache_default_soap_done(struct soap *soap, request_rec *r) \
    { soap_done(soap); }\
  SOAP_FMAC1 int SOAP_FMAC2 apache_default_soap_register_plugin_arg(struct soap *soap, int (*fcreate)(struct soap *, struct soap_plugin *, void *), void *arg, request_rec *r) \
    { return soap_register_plugin_arg(soap, fcreate, arg); }\
  SOAP_FMAC1 void* SOAP_FMAC2 apache_default_soap_lookup_plugin(struct soap *soap, const char *plugin, request_rec *r) \
    { return soap_lookup_plugin(soap, plugin); }\
  void apache_init_soap_interface(struct apache_soap_interface *pInt, request_rec *r) {\
      pInt->len = sizeof(struct apache_soap_interface);\
      pInt->interface_version = APACHE_GSOAP_INTERFACE_VERSION;\
      pInt->fsoap_user_init = INIT;\
      pInt->fsoap_init = apache_default_soap_init;\
      pInt->fsoap_serve = apache_default_soap_serve;\
      pInt->fsoap_destroy = apache_soap_soap_destroy;\
      pInt->fsoap_end = apache_default_soap_end;\
      pInt->fsoap_done = apache_default_soap_done;\
      pInt->fsoap_register_plugin_arg = apache_default_soap_register_plugin_arg;\
      pInt->fsoap_lookup_plugin = apache_default_soap_lookup_plugin;\
      pInt->reserved = NULL;\
      pInt->namespaces = namespaces;\
      pInt->soap_serializeheader = soap_serializeheader;\
      pInt->soap_putheader = soap_putheader;\
      pInt->soap_getheader = soap_getheader;\
      pInt->soap_fault = soap_fault;\
      pInt->soap_serializefault = soap_serializefault;\
      pInt->soap_putfault = soap_putfault;\
      pInt->soap_getfault = soap_getfault;\
      pInt->soap_faultcode = soap_faultcode;\
      pInt->soap_faultstring = soap_faultstring;\
      pInt->soap_faultdetail = soap_faultdetail;\
      pInt->soap_markelement = soap_markelement;\
      pInt->soap_getelement = soap_getelement;\
      pInt->soap_putelement = soap_putelement;\
  }

#define IMPLEMENT_GSOAP_SERVER() IMPLEMENT_GSOAP_SERVER_INIT(NULL)

#ifdef __cplusplus
}
#endif                          /*__cplusplus */
#endif                          /*_APACHE_GSOAP_H_INCLUDED */
