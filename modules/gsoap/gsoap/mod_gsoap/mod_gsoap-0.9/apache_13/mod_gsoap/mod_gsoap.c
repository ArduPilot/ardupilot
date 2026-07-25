/** Apache SOAP module for Apache 1.3.x
 * @file mod_gsoap.c
 * @author Christian Aberger (http://www.aberger.at)
 * updated by David Viner (dviner@apache.org)
 */

#if !defined(__GNUC__) || __GNUC__ < 2 || \
    (__GNUC__ == 2 && __GNUC_MINOR__ < 7) ||\
    defined(NEXT)
#ifndef __attribute__
#define __attribute__(__x)
#endif
#endif

/*
 * Moved gsoap include to top.  Without this, there's a parse error
 * introduced if gsoap was compiled with -DWITH_OPENSSL.
 */
#include "stdsoap2.h"           // standard header for gsoap

#include <stdio.h>
#include <assert.h>
#include "httpd.h"
#include "http_config.h"
#include "http_core.h"
#include "http_log.h"
#include "http_main.h"
#include "http_protocol.h"
#include "http_request.h"
#include "util_script.h"
#include "apache_gsoap.h"

typedef int bool;

#define false 0
#define true ((int)0xffff)
#define IOBUF_CHUNK_SIZE 8192
#define GSOAP_ID "Apache mod_gsoap gsoap httpd extension 0.0.6"

/** A shared library containing a SOAP server.
 */
typedef struct SoapSharedLibrary_S {
    pool *m_pPool;
    void *m_hLibrary;           ///< handle of the loaded libray.
    const char *m_pszPath;      ///< the path of the library, including the name.
    bool m_bIsSOAPLibrary;      ///< is this library a SOAP library that contains the server factory entry point?
} SoapSharedLibrary;

/** Table of shared libraries that are already loaded.
 * a singleton.
 */
typedef struct SoapSharedLibraries_S {
    pool *m_pPool;
    SoapSharedLibrary *m_pSOAPLibrary;  ///< the main SOAP library that will serve our requests
    array_header *m_pLibraries; ///< the array where we store our libraries.
    apache_init_soap_interface_fn m_pfnEntryPoint;
    struct apache_soap_interface *m_pIntf;
    bool m_bAllLibrariesLoaded; ///< have all libraries in our libraries collection been already successfully loaded?
} SoapSharedLibraries;

/** Environment to which record applies (directory,
 * server, or combination).
 */
typedef enum enConfigurationType {
    ct_server = 1,              ///< used for per-server configuration
    ct_directory = 2,           ///< used for per-directory configuration
    ct_both = 3                 ///< used for both
} ConfigurationType;

/** Store the configuration information set in the Apache Server configuration directives.
 * These are set e.g. in the file httpd.conf
 * It's perfectly reasonable to have two different structures for the two
 * different environments.  The same command handlers will be called for
 * both, though, so the handlers need to be able to tell them apart.  One
 * possibility is for both structures to start with an int which is zero for
 * one and 1 for the other.
 *
 * Note that while the per-directory and per-server configuration records are
 * available to most of the module handlers, they should be treated as
 * READ-ONLY by all except the command and merge handlers.  Sometimes handlers
 * are handed a record that applies to the current location by implication or
 * inheritance, and modifying it will change the rules for other locations.
 */
typedef struct gsoapConfiguration_S {
    SoapSharedLibraries *m_pLibraries;
    ConfigurationType m_Type;   ///< the type of configuration environment
} gsoapConfiguration;

/** Our internal per request soap configuration
 */
typedef struct gsoapRequestConfiguration_S {
    request_rec *r;             ///< the current request record.
    char *m_pszAllHeaders;      ///< all headers received as a string, this is returned to gsoap's http_parse function before we return the body.
    const char *m_pszCurrentHeaderReadingPosition;  ///< the position where the next header read operation will start.
    unsigned int m_nHeaderLength;   ///< total length of all headers concatenated as string 
    char *m_pOutBuf;            ///< output buffer
    size_t m_nOutBufLength;     ///< allocated length of output buffer
    size_t m_nOutBufCount;      ///< bytes in output buffer
    int headers_sent;           ///< have http - headers already been sent to client us?
    int headers_received;       ///< have the request headers already been passed to gsoap ? 
    int (*http_parse) (struct soap *);  ///< the original gsoap parsing function.
    struct apache_soap_interface *pIntf;
} gsoapRequestConfiguration;


/*
 * To avoid leaking memory from pools other than the per-request one, we
 * allocate a module-private pool.
 */
static pool *the_gsoapPool = NULL;

/** @return our pool for gsoapConfiguration */
static pool *
gsoapConfiguration_getModulePool()
{
    if(NULL == the_gsoapPool)
    {
        the_gsoapPool = ap_make_sub_pool(NULL);
    }
    return the_gsoapPool;
}
static gsoapConfiguration *getConfiguration(request_rec * r);
static gsoapRequestConfiguration *getRequestConfiguration(struct soap *);

/**
   @param p the pool to use for memory allocations.
   @param pszPath the path of the library.
*/
static void
SoapSharedLibrary_init(SoapSharedLibrary * This, pool * p,
                       const SoapSharedLibrary * pLib)
{
    This->m_pPool = p;
    This->m_hLibrary = NULL;
    This->m_pszPath = ap_pstrdup(p, pLib->m_pszPath);
    This->m_bIsSOAPLibrary = pLib->m_bIsSOAPLibrary;
}

static void
SoapSharedLibrary_init2(SoapSharedLibrary * This, pool * p, const char *pszPath)
{
    This->m_pPool = p;
    This->m_hLibrary = NULL;
    This->m_pszPath = ap_pstrdup(p, pszPath);
    This->m_bIsSOAPLibrary = false;
}

static void
SoapSharedLibrary_clear(SoapSharedLibrary * This, pool * p)
{
    This->m_pPool = p;
    This->m_hLibrary = NULL;
    This->m_pszPath = NULL;
    This->m_bIsSOAPLibrary = false;
}

static SoapSharedLibrary *
SoapSharedLibrary_create(pool * p)
{
    SoapSharedLibrary *This =
        (SoapSharedLibrary *) ap_pcalloc(p, sizeof(SoapSharedLibrary));
    SoapSharedLibrary_clear(This, p);
    return This;
}

/**
 *	@param pTempPool pool to use for allocating temporary objects (e.g. error message).
 */
static const char *
SoapSharedLibrary_load(SoapSharedLibrary * This, pool * pTempPool)
{
    const char *pszError = NULL;
    const int nFlags = RTLD_LAZY | RTLD_GLOBAL;

    This->m_hLibrary = (void *)dlopen(This->m_pszPath, nFlags);
    pszError = dlerror();
    if(NULL == This->m_hLibrary)
    {
        pszError =
            ap_psprintf(NULL == pTempPool ? This->m_pPool : pTempPool,
                        "mod_gsoap: %s loading library %s", pszError,
                        This->m_pszPath);
    }
    return pszError;
}

/*-------------------------------------------------------*/
static void
SoapSharedLibraries_init(SoapSharedLibraries * This, pool * p)
{
    This->m_pPool = p;
    This->m_pSOAPLibrary = NULL;
    This->m_pLibraries = ap_make_array(p, 0, sizeof(SoapSharedLibrary **));
    This->m_bAllLibrariesLoaded = false;
    This->m_pfnEntryPoint = NULL;
    This->m_pIntf =
        (struct apache_soap_interface *)ap_pcalloc(p,
                                                   sizeof(struct
                                                          apache_soap_interface));
}

static SoapSharedLibrary *
SoapSharedLibraries_getLibrary(SoapSharedLibraries * This, unsigned nIndex)
{
    SoapSharedLibrary **ppLibs = NULL;
    SoapSharedLibrary *pLib = NULL;

    assert(NULL != This);
    assert(NULL != This->m_pLibraries);
    assert(nIndex >= 0);
    assert(nIndex < This->m_pLibraries->nelts);
    ppLibs = (SoapSharedLibrary **) This->m_pLibraries->elts;
    pLib = ppLibs[nIndex];
    return pLib;
}

/**
 * @param pszPath the operating system name of the library.
 */
static bool
SoapSharedLibraries_contains(SoapSharedLibraries * This, const char *pszPath)
{
    int i = 0;
    bool bContains = false;

    for(i = 0; i < This->m_pLibraries->nelts && !bContains; i++)
    {
        SoapSharedLibrary *pLib = SoapSharedLibraries_getLibrary(This, i);

        assert(NULL != pLib);
        if(0 == strcmp(pszPath, pLib->m_pszPath))
        {
            bContains = true;
        }
    }
    return bContains;
}

static void
SoapSharedLibraries_addLibrary(SoapSharedLibraries * This,
                               SoapSharedLibrary * pLibrary)
{
    assert(NULL != pLibrary);
    This->m_bAllLibrariesLoaded = false;
    if(!SoapSharedLibraries_contains(This, pLibrary->m_pszPath))
    {
        SoapSharedLibrary **ppNewLib =
            (SoapSharedLibrary **) ap_push_array(This->m_pLibraries);
        *ppNewLib = pLibrary;
        if(pLibrary->m_bIsSOAPLibrary)
        {
            This->m_pSOAPLibrary = pLibrary;
        }
    }
}

static const char *
SoapSharedLibraries_getEntryPoints(SoapSharedLibraries * This,
                                   SoapSharedLibrary * pLib, pool * pTempPool,
                                   request_rec * r)
{
    /*
     * now we also pass the request record 
     */
    (*This->m_pfnEntryPoint) (This->m_pIntf, r);
    return NULL;
}

/**
 * @return the error message if a load failed, NULL on success.
 */
static const char *
SoapSharedLibraries_loadAllLibraries(SoapSharedLibraries * This,
                                     pool * pTempPool, request_rec * r)
{
    bool bAllLibrariesLoaded = false;
    const char *pszError = NULL;
    bool bRetry = false;
    int i = 0;
    int nRetry = 0;

    assert(NULL != This);

    if(This->m_bAllLibrariesLoaded)
    {
        return NULL;
    }
    for(nRetry = 0; nRetry < 5 && !bAllLibrariesLoaded; nRetry++)
    {
        do
        {
            pszError = NULL;
            bRetry = false;     // don't try it again.
            bAllLibrariesLoaded = true;
            for(i = 0; i < This->m_pLibraries->nelts; i++)
            {
                SoapSharedLibrary *pLib =
                    SoapSharedLibraries_getLibrary(This, i);
                if(NULL == pLib->m_hLibrary)
                {
                    pszError = SoapSharedLibrary_load(pLib, pTempPool);
                    if(NULL == pszError)
                    {
                        assert(NULL != pLib->m_hLibrary);
                        bRetry = true;  ///< we loaded one, lets retry with all others, maybe they depend on that.
                    }
                    else
                    {
                        bAllLibrariesLoaded = false;
                    }
                    if(NULL != pLib->m_hLibrary && pLib->m_bIsSOAPLibrary)
                    {

                        void *pfun = (void *)dlsym(pLib->m_hLibrary,
                                                   APACHE_HTTPSERVER_ENTRY_POINT);

                        if(NULL == pfun)
                        {
                            pszError = ap_psprintf(pTempPool,
                                                   "gsoap: load library \"%s\" success, but failed to find the \"%s\" entry point",
                                                   pLib->m_pszPath,
                                                   APACHE_HTTPSERVER_ENTRY_POINT);
                            return pszError;
                        }
                        else
                        {
                            This->m_pfnEntryPoint =
                                (apache_init_soap_interface_fn) pfun;
                            pszError =
                                SoapSharedLibraries_getEntryPoints(This, pLib,
                                                                   pTempPool,
                                                                   r);
                        }
                    }
                }
            }
        }
        while(bRetry);
    }
    if(bAllLibrariesLoaded)
    {
        This->m_bAllLibrariesLoaded = true;
        pszError = NULL;
    }
    return pszError;
}

static void
SoapSharedLibraries_merge(SoapSharedLibraries * This,
                          SoapSharedLibraries * pLibs)
{
    int i = 0;

    assert(NULL != This);
    if(NULL == pLibs)
    {
        return;
    }
    This->m_bAllLibrariesLoaded = false;
    for(i = 0; i < pLibs->m_pLibraries->nelts; i++)
    {
        SoapSharedLibrary *pLib = SoapSharedLibraries_getLibrary(pLibs, i);

        if(!SoapSharedLibraries_contains(This, pLib->m_pszPath))
        {
            SoapSharedLibrary *pNewLib =
                SoapSharedLibrary_create(This->m_pPool);
            SoapSharedLibrary_init(pNewLib, This->m_pPool, pLib);
            SoapSharedLibraries_addLibrary(This, pNewLib);
        }
    }
}

static void
SoapSharedLibraries_merge3(SoapSharedLibraries * This,
                           SoapSharedLibraries * libraries1,
                           SoapSharedLibraries * libraries2)
{
    SoapSharedLibraries_merge(This, libraries1);
    SoapSharedLibraries_merge(This, libraries2);
}

static void
gsoapConfiguration_merge(gsoapConfiguration * This,
                         gsoapConfiguration * pParentConfig,
                         gsoapConfiguration * pNewConfig)
{
    assert(NULL != This);
    SoapSharedLibraries_merge3(This->m_pLibraries, pParentConfig->m_pLibraries,
                               pNewConfig->m_pLibraries);
    This->m_Type = ct_both;
}

static void
gsoapConfiguration_init(gsoapConfiguration * This, pool * p)
{
    This->m_pLibraries =
        (SoapSharedLibraries *) ap_pcalloc(p, sizeof(SoapSharedLibraries));
    SoapSharedLibraries_init(This->m_pLibraries, p);
    This->m_Type = ct_directory;
}

static gsoapConfiguration *
gsoapConfiguration_create(pool * p)
{
    gsoapConfiguration *pConfig =
        (gsoapConfiguration *) ap_pcalloc(p, sizeof(gsoapConfiguration));
    gsoapConfiguration_init(pConfig, p);
    return pConfig;
}

/*
 * forward declarations                                                     
 */
static int gsoap_handler(request_rec * r);
static void gsoap_init(server_rec * s, pool * p);
static void gsoap_child_init(server_rec * s, pool * p);
static void gsoap_child_exit(server_rec * s, pool * p);
static void *gsoap_create_dir_config(pool * p, char *dirspec);
static void *gsoap_merge_dir_config(pool * p, void *parent_conf,
                                    void *newloc_conf);
static void *gsoap_create_server_config(pool * p, server_rec * s);
static void *gsoap_merge_server_config(pool * p, void *server1_conf,
                                       void *server2_conf);
static int gsoap_post_read_request(request_rec * r);
static int gsoap_translate_handler(request_rec * r);
static int gsoap_check_user_id(request_rec * r);
static int gsoap_auth_checker(request_rec * r);
static int gsoap_access_checker(request_rec * r);
static int gsoap_type_checker(request_rec * r);
static int gsoap_fixer_upper(request_rec * r);
static int gsoap_logger(request_rec * r);
static int gsoap_header_parser(request_rec * r);

static bool AddSharedLibrary(gsoapConfiguration * pConfig, const char *pszPath,
                             const bool bIsSOAPLibrary);


/*
 * We prototyped the various syntax for command handlers (routines that     
 * are called when the configuration parser detects a directive declared    
 * by our module) earlier.  Now we actually declare a "real" routine that   
 * will be invoked by the parser when our "real" directive is               
 * encountered.                                                             
 *                                                                          
 * If a command handler encounters a problem processing the directive, it   
 * signals this fact by returning a non-NULL pointer to a string            
 * describing the problem.                                                  
 *                                                                          
 * The magic return value DECLINE_CMD is used to deal with directives       
 * that might be declared by multiple modules.  If the command handler      
 * returns NULL, the directive was processed; if it returns DECLINE_CMD,    
 * the next module (if any) that declares the directive is given a chance   
 * at it.  If it returns any other value, it's treated as the text of an    
 * error message.                                                           
 *
 */

/** Command handler for the TAKE1 "SOAPLibrary" directive.
 * We remember the load path for the shared library that contains the SOAP server.
 */
static const char *
cmd_SoapLibrary(cmd_parms * cmd, void *mconfig, const char *pszPath)
{
    gsoapConfiguration *pConfig = (gsoapConfiguration *) mconfig;

    AddSharedLibrary(pConfig, pszPath, true);
    return NULL;
}

/** Command handler for the TAKE1 "SOAPSupportLibrary" directive.
 * We remember the load path for a shared library that must additionally loaded.
 * This is a mechanism to load libraries that the SOAPLibrary depends on.
 * This type of libraries do not contain our soap server.
 */
static const char *
cmd_SupportLibrary(cmd_parms * cmd, void *mconfig, const char *pszPath)
{
    gsoapConfiguration *pConfig = (gsoapConfiguration *) mconfig;

    AddSharedLibrary(pConfig, pszPath, false);
    return NULL;
}

typedef const char *(*command_function_interface) ();

/** List of directives specific to our module.
 */
static const command_rec gsoap_cmds[] = {
    {
     "SOAPLibrary",             ///< directive name
     (command_function_interface) cmd_SoapLibrary,  ///< config action routine
     NULL,                      ///< argument to include in call
     ACCESS_CONF,               ///< where available
     TAKE1,                     ///< arguments
     "SOAP shared Library that will be dynamically loaded. - 1 argument (path)" ///< directive description
     },
    {
     "SupportLibrary",          ///< directive name
     (command_function_interface) cmd_SupportLibrary,   ///< config action routine
     NULL,                      ///< argument to include in call
     ACCESS_CONF,               ///< where available
     TAKE1,                     ///< arguments
     "additional library that must be dynamically loaded - 1 argument (path)"   ///< directive description
     },
    {NULL}
};

/*
 * --------------------------------------------------------------------------
 *                                                                          
 * Now the list of content handlers available from this module.             
 *                                                                          
 * 
 * List of content handlers our module supplies.  Each handler is defined by
 * two parts: a name by which it can be referenced (such as by
 * {Add,Set}Handler), and the actual routine name.  The list is terminated by
 * a NULL block, since it can be of variable length.
 *
 * Note that content-handlers are invoked on a most-specific to least-specific
 * basis; that is, a handler that is declared for "text/plain" will be
 * invoked before one that was declared for "text / *".  Note also that
 * if a content-handler returns anything except DECLINED, no other
 * content-handlers will be called.
 */
static const handler_rec gsoap_handlers[] = {
    {"gsoap-handler", gsoap_handler},
    {NULL}
};

/*
 *--------------------------------------------------------------------------
 *                                                                          
 * All of the routines have been declared now.  Here's the list of          
 * directives specific to our module, and information about where they      
 * may appear and how the command parser should pass them to us for         
 * processing.  Note that care must be taken to ensure that there are NO    
 * collisions of directive names between modules.                           
 *                                                                          
 */

/*
 * Module definition for configuration.  If a particular callback is not
 * needed, replace its routine name below with the word NULL.
 *
 * The number in brackets indicates the order in which the routine is called
 * during request processing.  Note that not all routines are necessarily
 * called (such as if a resource doesn't have access restrictions).
 */

/** List of callback routines and data structures that provide the hooks into our module.
 */
module MODULE_VAR_EXPORT gsoap_module = {
    STANDARD_MODULE_STUFF,
    gsoap_init,                 /* module initializer */
    gsoap_create_dir_config,    /* per-directory config creator */
    gsoap_merge_dir_config,     /* dir config merger */
    gsoap_create_server_config, /* server config creator */
    gsoap_merge_server_config,  /* server config merger */
    gsoap_cmds,                 /* command table */
    gsoap_handlers,             /* [9] list of handlers */
    gsoap_translate_handler,    /* [2] filename-to-URI translation */
    gsoap_check_user_id,        /* [5] check/validate user_id */
    gsoap_auth_checker,         /* [6] check user_id is valid *here* */
    gsoap_access_checker,       /* [4] check access by host address */
    gsoap_type_checker,         /* [7] MIME type checker/setter */
    gsoap_fixer_upper,          /* [8] fixups */
    gsoap_logger,               /* [10] logger */
    gsoap_header_parser,        /* [3] header parser */
    gsoap_child_init,           /* process initializer */
    gsoap_child_exit,           /* process exit/cleanup */
    gsoap_post_read_request     /* [1] post read_request handling */
};

/** helper to write out the headers */
static int
ListHeadersCallback(void *rec, const char *key, const char *value)
{
    request_rec *r = (request_rec *) rec;

    ap_rprintf(r, "%s: %s<br>", key, value);
    return 1;
}

/** write out the headers of the request. */
static void
ListHeaders(request_rec * r)
{
    ap_table_do(ListHeadersCallback, r, r->headers_in, NULL);
}

/** send the error message to the client browser */
static void
SendErrorMessage(request_rec * r, const char *pszError)
{
    /*
     * gsoapConfiguration *pConfig = getConfiguration(r); 
     */

    ap_log_error(APLOG_MARK, APLOG_ERR, r->server, "mod_gsoap: %s", pszError);
    r->content_type = "text/html";
    ap_send_http_header(r);
    ap_rputs(DOCTYPE_HTML_3_2, r);
    ap_rputs("<HTML>\n", r);
    ap_rputs(" <HEAD>\n", r);
    ap_rputs("  <TITLE>Apache Soap Handler\n", r);
    ap_rputs("  </TITLE>\n", r);
    ap_rputs(" </HEAD>\n", r);
    ap_rputs(" <BODY>\n", r);
    ap_rputs("  <H1>mod_gsoap Apache SOAP Server Error</H1>\n", r);
    ap_rprintf(r,
               "<p><strong>%s</strong><br>Please see the documentation at <a href=\"http://www.webware.at/SOAP\">WebWare</a> for details.</p>",
               pszError);
    ap_rputs("  <H2>Content headers of the request</H2>\n", r);
    ListHeaders(r);
    ap_rputs("</BODY></HTML>\n", r);
}
static int
send_header_to_gsoap(void *pvoid, const char *key, const char *value)
{
    gsoapRequestConfiguration *pRqConf = NULL;
    struct soap *psoap = (struct soap *)pvoid;

    if(NULL != psoap)
    {
        pRqConf = getRequestConfiguration(psoap);
    }
    if(NULL == pRqConf)
    {
        return 0;
    }
    if(0 == strcasecmp(key, "SOAPAction") ||
       0 == strcasecmp(key, "Content-Type") ||
       0 == strcasecmp(key, "Status") || 0 == strcasecmp(key, "Content-Length"))
    {
        psoap->fparsehdr(psoap, key, value);
    }
    return 1;
}

/*
 * Callback functions for gsoap. We must parse the headers ourselves and 
 * we must handle send/receive properly.  
 */
static int
http_post_header(struct soap *soap, const char *key, const char *value)
{
    gsoapRequestConfiguration *pRqConf = getRequestConfiguration(soap);
    request_rec *r = NULL == pRqConf ? NULL : pRqConf->r;

    if(NULL != value)
    {
        if(0 == strcasecmp(key, "SOAPAction"))
        {
            ap_table_set(r->headers_out, key, value);
        }
        else if(0 == strcasecmp(key, "Content-Type"))
        {
            r->content_type = ap_pstrdup(r->pool, value);
        }
        else if(0 == strcasecmp(key, "Content-Length"))
        {
            ap_set_content_length(r, atoi(value));
        }
    }
    return SOAP_OK;
}

/** gsoap function that requests the next piece of data from us */
static size_t
frecv(struct soap *psoap, char *pBuf, size_t len)
{
    request_rec *r = NULL;
    int nRet = 0;
    gsoapRequestConfiguration *pRqConf = getRequestConfiguration(psoap);

    if(NULL != pRqConf)
    {
        r = pRqConf->r;
        if(!pRqConf->headers_received)
        {
            ap_table_do(send_header_to_gsoap, psoap, r->headers_in, NULL);
            pRqConf->headers_received = true;
        }
        if(r->remaining > 0)
        {
            nRet =
                ap_get_client_block(r, pBuf,
                                    len > r->remaining ? r->remaining : len);
        }
    }
    return nRet;
}
static int
fsend(struct soap *psoap, const char *pBuf, size_t len)
{
    int nRet = SOAP_OK;
    gsoapRequestConfiguration *pRqConf = getRequestConfiguration(psoap);

    if(NULL != pRqConf)
    {
        request_rec *r = pRqConf->r;

        if(!pRqConf->headers_sent)
        {
            ap_send_http_header(r);
            pRqConf->headers_sent = true;
        }
        nRet = ap_rwrite(pBuf, len, r) == len ? SOAP_OK : SOAP_FAULT;
    }
    else
    {
        nRet = SOAP_FAULT;
    }
    return nRet;
}

/** instead of real header parsing we skip that. */
static int
http_parse(struct soap *psoap)
{
    return SOAP_OK;
}

/*
 * plugin functions 
 */
static int
mod_gsoap_plugin_copy(struct soap *soap, struct soap_plugin *dst,
                      struct soap_plugin *src)
{
    *dst = *src;
    /*
     * should this be a deep copy? 
     */
    return SOAP_OK;
}
static void
mod_gsoap_delete(struct soap *soap, struct soap_plugin *p)
{
}
static int
mod_gsoap_plugin(struct soap *soap, struct soap_plugin *p, void *arg)
{
    p->id = GSOAP_ID;
    p->data = arg;
    p->fcopy = mod_gsoap_plugin_copy;
    p->fdelete = mod_gsoap_delete;
    return SOAP_OK;
}
static void
set_callbacks(request_rec * r, gsoapRequestConfiguration * pRqConf,
              struct soap *psoap)
{
    gsoapConfiguration *pConfig = getConfiguration(r);
    struct apache_soap_interface *pIntf = pConfig->m_pLibraries->m_pIntf;

    pRqConf->r = r;
    pRqConf->http_parse = psoap->fparse;
    psoap->user = pRqConf;
    psoap->frecv = frecv;
    psoap->fsend = fsend;
    psoap->fparse = http_parse;
    psoap->fposthdr = http_post_header;
    if(NULL != pIntf->fsoap_init)
    {
        (*pIntf->fsoap_register_plugin_arg) (psoap, mod_gsoap_plugin,
                                             (void *)pRqConf, r);
    }
    else
    {
        psoap->user = pRqConf;
    }
}

/*
 *--------------------------------------------------------------------------
 *                                                                          
 * Now we declare our content handlers, which are invoked when the server   
 * encounters a document which our module is supposed to have a chance to   
 * see.  (See mod_mime's SetHandler and AddHandler directives, and the      
 * mod_info and mod_status examples, for more details.)                     
 *                                                                          
 * Since content handlers are dumping data directly into the connexion      
 * (using the r*() routines, such as rputs() and rprintf()) without         
 * intervention by other parts of the server, they need to make             
 * sure any accumulated HTTP headers are sent first.  This is done by       
 * calling send_http_header().  Otherwise, no header will be sent at all,   
 * and the output sent to the client will actually be HTTP-uncompliant.     
 *--------------------------------------------------------------------------
 */

/**
 * SOAP content handler.
 *
 * @return the value that instructs the caller concerning what happened and what to do next.
 *  OK ("we did our thing")
 *  DECLINED ("this isn't something with which we want to get involved")
 *  HTTP_mumble ("an error status should be reported")
 */
static int
gsoap_handler(request_rec * r)
{
    static const int nResponseBufferLen = IOBUF_CHUNK_SIZE;
    const char *pszError = NULL;
    struct soap *psoap = NULL;
    struct apache_soap_interface *pIntf = NULL;
    int nRet = 0;

    /*
     * char *pszResponse = ap_pcalloc(r->pool, nResponseBufferLen); 
     */
    gsoapConfiguration *pConfig = getConfiguration(r);
    gsoapRequestConfiguration *pRqConf = NULL;

    assert(NULL != pConfig);

    psoap = (struct soap *)ap_pcalloc(r->pool, sizeof(struct soap));
    pRqConf = ap_pcalloc(r->pool, sizeof(gsoapRequestConfiguration));
    pszError =
        SoapSharedLibraries_loadAllLibraries(pConfig->m_pLibraries, r->pool, r);
    pIntf = pConfig->m_pLibraries->m_pIntf;

    ap_update_mtime(r, r->request_time);
    ap_set_last_modified(r);
    if(NULL != pszError)
    {
        static bool bFirstTime = true;

        if(bFirstTime)
        {
            ap_log_error(APLOG_MARK, APLOG_ERR, r->server, pszError);
            bFirstTime = false;
        }
    }

    if(NULL == pszError)
    {
        if(M_POST != r->method_number && M_GET != r->method_number)
        {
            pszError = "Only POST and GET allowed as request for SOAP!";
        }
    }
    /*
     * as a next step, we prepare a buffer that sends the request as first line to gsoap. 
     * Then the remaining data. 
     * We start returning bytes on frecv from this buffer, until it is empty. 
     * then it is not necessary to fiddle around with gsoap's request line parsing.
     */
    if(NULL == pszError)
    {
        pRqConf->r = r;
        pRqConf->headers_sent = false;
        pRqConf->headers_received = false;
        pRqConf->m_pszAllHeaders = NULL;
        pRqConf->m_nHeaderLength = strlen(r->the_request) + 2;
        pRqConf->m_pszCurrentHeaderReadingPosition = NULL;
        pRqConf->m_nOutBufCount = 0;
        pRqConf->m_nOutBufLength = nResponseBufferLen;
        pRqConf->m_pOutBuf = ap_pcalloc(r->pool, nResponseBufferLen);
        pRqConf->http_parse = NULL;
        pRqConf->m_pszAllHeaders =
            ap_pcalloc(r->pool, pRqConf->m_nHeaderLength + 1);
        pRqConf->m_pszCurrentHeaderReadingPosition = pRqConf->m_pszAllHeaders;
        strcpy(pRqConf->m_pszAllHeaders, r->the_request);
        strcat(pRqConf->m_pszAllHeaders, "\r\n");
        pRqConf->pIntf = pIntf;
    }

    /*
     * We're about to start sending content, so we need to force the HTTP
     * headers to be sent at this point.  Otherwise, no headers will be sent
     * at all.  We can set any we like first, of course.  **NOTE** Here's
     * where you set the "Content-type" header, and you do so by putting it in
     * r->content_type, *not* r->headers_out("Content-type").  If you don't
     * set it, it will be filled in with the server's default type (typically
     * "text/plain").  You *must* also ensure that r->content_type is lower
     * case.
     *
     * We also need to start a timer so the server can know if the connection
     * is broken.
     */
    ap_soft_timeout("gsoap xml response", r);
    /*
     * If we're only supposed to send header information (HEAD request), we're
     * already there.
     */
    if(r->header_only)
    {
        ap_send_http_header(r);
        ap_kill_timeout(r);
        return OK;
    }
    if(NULL != pszError)
    {
        SendErrorMessage(r, pszError);
        ap_kill_timeout(r);
        return OK;
    }
    nRet = ap_setup_client_block(r, REQUEST_CHUNKED_DECHUNK);
    if(OK != nRet)
    {
        SendErrorMessage(r, "Failed to start receiving POST buffer");
        ap_kill_timeout(r);
        return OK;
    }
    nRet = ap_should_client_block(r);
    if(0 == nRet)
    {
        SendErrorMessage(r, "No body received");
        ap_kill_timeout(r);
        return OK;
    }

    if(NULL != pszError)
    {
        ap_log_error(APLOG_MARK, APLOG_ERR, r->server, pszError);
        SendErrorMessage(r, pszError);
        ap_kill_timeout(r);
        return OK;
    }
    if(NULL != pIntf->fsoap_init)
    {
        (*pIntf->fsoap_init) (psoap, r);
        psoap->namespaces = pIntf->namespaces;
        set_callbacks(r, pRqConf, psoap);
        if(NULL != pIntf->fsoap_serve)
        {
            (*pIntf->fsoap_serve) (psoap, r);
        }
        else
        {
            SendErrorMessage(r, "no soap_serve entry point");
            ap_kill_timeout(r);
            return OK;
        }
        if(NULL != pIntf->fsoap_destroy)
        {
            pIntf->fsoap_destroy(psoap, r); // not an error in 2.1.10 any more.
        }
        if(NULL != pIntf->fsoap_end)
        {
            pIntf->fsoap_end(psoap, r);
        }
        else
        {
            SendErrorMessage(r, "no soap_end entry point");
            ap_kill_timeout(r);
        }
        if(NULL != pIntf->fsoap_done)
        {
            pIntf->fsoap_done(psoap, r);
        }
        else
        {
            SendErrorMessage(r, "no soap_done entry point");
            ap_kill_timeout(r);
        }
    }
    else
    {
        SendErrorMessage(r, "no soap_init entry point");
        ap_kill_timeout(r);
        return OK;
    }

    /*
     * We did what we wanted to do, so tell the rest of the server we
     * succeeded. We need not delete pszResponse, because it was allocated from the request pool.
     */
    ap_kill_timeout(r);
    return OK;
}

/*
 * --------------------------------------------------------------------------
 * *                                                                          
 * * Now let's declare routines for each of the callback phase in order.      
 * * (That's the order in which they're listed in the callback list, *not     
 * * the order in which the server calls them!  See the command_rec           
 * * declaration).  Note that these may be                                    
 * * called for situations that don't relate primarily to our function - in   
 * * other words, the fixup handler shouldn't assume that the request has     
 * * to do with "gsoap" stuff.                                                
 * *                                                                          
 * * With the exception of the content handler, all of our routines will be   
 * * called for each request, unless an earlier handler from another module   
 * * aborted the sequence.                                                    
 * *                                                                          
 * * Handlers that are declared as "int" can return the following:            
 * *                                                                          
 * *  OK          Handler accepted the request and did its thing with it.     
 * *  DECLINED    Handler took no action.                                     
 * *  HTTP_mumble Handler looked at request and found it wanting.             
 * *                                                                          
 * * What the server does after calling a module handler depends upon the     
 * * handler's return value.  In all cases, if the handler returns            
 * * DECLINED, the server will continue to the next module with an handler    
 * * for the current phase.  However, if the handler return a non-OK,         
 * * non-DECLINED status, the server aborts the request right there.  If      
 * * the handler returns OK, the server's next action is phase-specific;      
 * * see the individual handler comments below for details.                   
 * *                                                                          
 * *--------------------------------------------------------------------------
 */

/*
 * This function is called during server initialisation.  Any information
 * that needs to be recorded must be in static cells, since there's no
 * configuration record.
 *
 * There is no return value.
 */

static void
gsoap_init(server_rec * s, pool * p)
{
    //    ap_log_error(APLOG_MARK, APLOG_ERR, s, "mod_gsoap initialized", NULL);
}

/*
 * This function is called during server initialisation when an heavy-weight
 * process (such as a child) is being initialised.  As with the
 * module-initialisation function, any information that needs to be recorded
 * must be in static cells, since there's no configuration record.
 *
 * There is no return value.
 */

static void
gsoap_child_init(server_rec * s, pool * p)
{
}

/*
 * This function is called when an heavy-weight process (such as a child) is
 * being run down or destroyed.  As with the child-initialisation function,
 * any information that needs to be recorded must be in static cells, since
 * there's no configuration record.
 *
 * There is no return value.
 */

static void
gsoap_child_exit(server_rec * s, pool * p)
{
    //gsoapConfiguration::getLibraries()->clear();
}

/*
 * This function gets called to create a per-directory configuration
 * record.  This will be called for the "default" server environment, and for
 * each directory for which the parser finds any of our directives applicable.
 * If a directory doesn't have any of our directives involved (i.e., they
 * aren't in the .htaccess file, or a <Location>, <Directory>, or related
 * block), this routine will *not* be called - the configuration for the
 * closest ancestor is used.
 *
 * The return value is a pointer to the created module-specific
 * structure.
 */
static void *
gsoap_create_dir_config(pool * p, char *dirspec)
{
    gsoapConfiguration *pConfig = gsoapConfiguration_create(p);

    pConfig->m_Type = ct_directory;
    return pConfig;
}

/*
 * This function gets called to merge two per-directory configuration
 * records.  This is typically done to cope with things like .htaccess files
 * or <Location> directives for directories that are beneath one for which a
 * configuration record was already created.  The routine has the
 * responsibility of creating a new record and merging the contents of the
 * other two into it appropriately.  If the module doesn't declare a merge
 * routine, the record for the closest ancestor location (that has one) is
 * used exclusively.
 *
 * The routine MUST NOT modify any of its arguments!
 *
 * The return value is a pointer to the created module-specific structure
 * containing the merged values.
 */
static void *
gsoap_merge_dir_config(pool * p, void *parent_conf, void *newloc_conf)
{
    gsoapConfiguration *pMergedConfig = gsoapConfiguration_create(p);
    gsoapConfiguration *pParentConfig = (gsoapConfiguration *) parent_conf;
    gsoapConfiguration *pNewConfig = (gsoapConfiguration *) newloc_conf;

    gsoapConfiguration_merge(pMergedConfig, pParentConfig, pNewConfig);
    return pMergedConfig;
}

/*
 * This function gets called to create a per-server configuration
 * record.  It will always be called for the "default" server.
 *
 * The return value is a pointer to the created module-specific
 * structure.
 */
static void *
gsoap_create_server_config(pool * p, server_rec * s)
{
    gsoapConfiguration *pConfig = gsoapConfiguration_create(p);

    pConfig->m_Type = ct_server;
    return pConfig;
}

/*
 * This function gets called to merge two per-server configuration
 * records.  This is typically done to cope with things like virtual hosts and
 * the default server configuration. The routine has the responsibility of
 * creating a new record and merging the contents of the other two into it
 * appropriately. If the module doesn't declare a merge routine, the more
 * specific existing record is used exclusively.
 *
 * The routine MUST NOT modify any of its arguments!
 *
 * The return value is a pointer to the created module-specific structure
 * containing the merged values.
 */
static void *
gsoap_merge_server_config(pool * p, void *server1_conf, void *server2_conf)
{
    gsoapConfiguration *pMergedConfig = gsoapConfiguration_create(p);
    gsoapConfiguration *pServer1Config = (gsoapConfiguration *) server1_conf;
    gsoapConfiguration *pServer2Config = (gsoapConfiguration *) server2_conf;

    gsoapConfiguration_merge(pMergedConfig, pServer1Config, pServer2Config);
    pMergedConfig->m_Type = ct_server;
    return (void *)pMergedConfig;
}

/*
 * This routine is called after the request has been read but before any other
 * phases have been processed.  This allows us to make decisions based upon
 * the input header fields.
 *
 * The return value is OK, DECLINED, or HTTP_mumble.  If we return OK, no
 * further modules are called for this phase.
 */
static int
gsoap_post_read_request(request_rec * r)
{
    return DECLINED;
}

/*
 * This routine gives our module an opportunity to translate the URI into an
 * actual filename.  If we don't do anything special, the server's default
 * rules (Alias directives and the like) will continue to be followed.
 *
 * The return value is OK, DECLINED, or HTTP_mumble.  If we return OK, no
 * further modules are called for this phase.
 */
static int
gsoap_translate_handler(request_rec * r)
{
    return DECLINED;
}

/*
 * This routine is called to check the authentication information sent with
 * the request (such as looking up the user in a database and verifying that
 * the [encrypted] password sent matches the one in the database).
 *
 * The return value is OK, DECLINED, or some HTTP_mumble error (typically
 * HTTP_UNAUTHORIZED).  If we return OK, no other modules are given a chance
 * at the request during this phase.
 */
static int
gsoap_check_user_id(request_rec * r)
{
    return DECLINED;
}

/*
 * This routine is called to check to see if the resource being requested
 * requires authorisation.
 *
 * The return value is OK, DECLINED, or HTTP_mumble.  If we return OK, no
 * other modules are called during this phase.
 *
 * If *all* modules return DECLINED, the request is aborted with a server
 * error.
 */
static int
gsoap_auth_checker(request_rec * r)
{
    return DECLINED;
}

/*
 * This routine is called to check for any module-specific restrictions placed
 * upon the requested resource.  (See the mod_access module for an example.)
 *
 * The return value is OK, DECLINED, or HTTP_mumble.  All modules with an
 * handler for this phase are called regardless of whether their predecessors
 * return OK or DECLINED.  The first one to return any other status, however,
 * will abort the sequence (and the request) as usual.
 */
static int
gsoap_access_checker(request_rec * r)
{
    return DECLINED;
}

/*
 * This routine is called to determine and/or set the various document type
 * information bits, like Content-type (via r->content_type), language, et
 * cetera.
 *
 * The return value is OK, DECLINED, or HTTP_mumble.  If we return OK, no
 * further modules are given a chance at the request for this phase.
 */
static int
gsoap_type_checker(request_rec * r)
{
    return DECLINED;
}

/*
 * This routine is called to perform any module-specific fixing of header
 * fields, et cetera.  It is invoked just before any content-handler.
 *
 * The return value is OK, DECLINED, or HTTP_mumble.  If we return OK, the
 * server will still call any remaining modules with an handler for this
 * phase.
 */
static int
gsoap_fixer_upper(request_rec * r)
{
    return OK;
}

/*
 * This routine is called to perform any module-specific logging activities
 * over and above the normal server things.
 *
 * The return value is OK, DECLINED, or HTTP_mumble.  If we return OK, any
 * remaining modules with an handler for this phase will still be called.
 */
static int
gsoap_logger(request_rec * r)
{
    return DECLINED;
}

/*
 * This routine is called to give the module a chance to look at the request
 * headers and take any appropriate specific actions early in the processing
 * sequence.
 *
 * The return value is OK, DECLINED, or HTTP_mumble.  If we return OK, any
 * remaining modules with handlers for this phase will still be called.
 */
static int
gsoap_header_parser(request_rec * r)
{
    return DECLINED;
}

/** helper funciton for library command handler.
 * @param pszPath the path of the library.
 * @param bIsSOAPLibrary true if it is a shared library containing a SOAP server.
 * @return true if the library was added, false if it was aleady in the collection.
 */
static bool
AddSharedLibrary(gsoapConfiguration * pConfig, const char *pszPath,
                 const bool bIsSOAPLibrary)
{
    bool bAdded = false;
    pool *pPool = gsoapConfiguration_getModulePool();

    if(!SoapSharedLibraries_contains(pConfig->m_pLibraries, pszPath))
    {
        SoapSharedLibrary *pLibrary = SoapSharedLibrary_create(pPool);

        SoapSharedLibrary_init2(pLibrary, pPool, ap_pstrdup(pPool, pszPath));
        pLibrary->m_bIsSOAPLibrary = bIsSOAPLibrary;
        SoapSharedLibraries_addLibrary(pConfig->m_pLibraries, pLibrary);
        bAdded = true;
    }
    return bAdded;
}
static gsoapConfiguration *
getConfiguration(request_rec * r)
{
    return (gsoapConfiguration *) ap_get_module_config(r->per_dir_config,
                                                       &gsoap_module);
}
static gsoapRequestConfiguration *
getRequestConfiguration(struct soap *soap)
{
    gsoapRequestConfiguration *pRqConf =
        (gsoapRequestConfiguration *) soap->fplugin(soap, GSOAP_ID);
    return pRqConf;
}


/*
 * Patch from Ryan Troll
 *
 * Implmement these as weak symbols, allowing symbol checking during
 * compilation to succeed, even when another object is actually
 * providing these symbols at runtime.
 */
SOAP_NMAC struct Namespace namespaces[] __attribute__ ((weak));
SOAP_NMAC struct Namespace namespaces[] = {
    {"SOAP-ENV", "http://schemas.xmlsoap.org/soap/envelope/",
     "http://www.w3.org/*/soap-envelope"},
    {"SOAP-ENC", "http://schemas.xmlsoap.org/soap/encoding/",
     "http://www.w3.org/*/soap-encoding"},
    {"xsi", "http://www.w3.org/2001/XMLSchema-instance",
     "http://www.w3.org/*/XMLSchema-instance"},
    {"xsd", "http://www.w3.org/2001/XMLSchema",
     "http://www.w3.org/*/XMLSchema"},
    {NULL, NULL}
};

SOAP_FMAC3 void SOAP_FMAC4 soap_serializeheader(struct soap *soap)
    __attribute__ ((weak));
SOAP_FMAC3 void SOAP_FMAC4
soap_serializeheader(struct soap *soap)
{
    gsoapRequestConfiguration *pRqConf = getRequestConfiguration(soap);

    pRqConf->pIntf->soap_serializeheader(soap);
}

SOAP_FMAC3 int SOAP_FMAC4 soap_putheader(struct soap *soap)
    __attribute__ ((weak));
SOAP_FMAC3 int SOAP_FMAC4
soap_putheader(struct soap *soap)
{
    gsoapRequestConfiguration *pRqConf = getRequestConfiguration(soap);

    return pRqConf->pIntf->soap_putheader(soap);
}

SOAP_FMAC3 int SOAP_FMAC4 soap_getheader(struct soap *soap)
    __attribute__ ((weak));
SOAP_FMAC3 int SOAP_FMAC4
soap_getheader(struct soap *soap)
{
    gsoapRequestConfiguration *pRqConf = getRequestConfiguration(soap);

    return pRqConf->pIntf->soap_getheader(soap);
}

SOAP_FMAC3 void SOAP_FMAC4 soap_fault(struct soap *soap) __attribute__ ((weak));
SOAP_FMAC3 void SOAP_FMAC4
soap_fault(struct soap *soap)
{
    gsoapRequestConfiguration *pRqConf = getRequestConfiguration(soap);

    pRqConf->pIntf->soap_fault(soap);
}

SOAP_FMAC3 void SOAP_FMAC4 soap_serializefault(struct soap *soap)
    __attribute__ ((weak));
SOAP_FMAC3 void SOAP_FMAC4
soap_serializefault(struct soap *soap)
{
    gsoapRequestConfiguration *pRqConf = getRequestConfiguration(soap);

    pRqConf->pIntf->soap_serializefault(soap);
}

SOAP_FMAC3 int SOAP_FMAC4 soap_putfault(struct soap *soap)
    __attribute__ ((weak));
SOAP_FMAC3 int SOAP_FMAC4
soap_putfault(struct soap *soap)
{
    gsoapRequestConfiguration *pRqConf = getRequestConfiguration(soap);

    return pRqConf->pIntf->soap_putfault(soap);
}

SOAP_FMAC3 int SOAP_FMAC4 soap_getfault(struct soap *soap)
    __attribute__ ((weak));
SOAP_FMAC3 int SOAP_FMAC4
soap_getfault(struct soap *soap)
{
    gsoapRequestConfiguration *pRqConf = getRequestConfiguration(soap);

    return pRqConf->pIntf->soap_getfault(soap);
}

SOAP_FMAC3 const char **SOAP_FMAC4 soap_faultcode(struct soap *soap)
    __attribute__ ((weak));
SOAP_FMAC3 const char **SOAP_FMAC4
soap_faultcode(struct soap *soap)
{
    gsoapRequestConfiguration *pRqConf = getRequestConfiguration(soap);

    return pRqConf->pIntf->soap_faultcode(soap);
}

SOAP_FMAC3 const char **SOAP_FMAC4 soap_faultstring(struct soap *soap)
    __attribute__ ((weak));
SOAP_FMAC3 const char **SOAP_FMAC4
soap_faultstring(struct soap *soap)
{
    gsoapRequestConfiguration *pRqConf = getRequestConfiguration(soap);

    return pRqConf->pIntf->soap_faultstring(soap);
}

SOAP_FMAC3 const char **SOAP_FMAC4 soap_faultdetail(struct soap *soap)
    __attribute__ ((weak));
SOAP_FMAC3 const char **SOAP_FMAC4
soap_faultdetail(struct soap *soap)
{
    gsoapRequestConfiguration *pRqConf = getRequestConfiguration(soap);

    return pRqConf->pIntf->soap_faultdetail(soap);
}

/*
 * Patch from Ryan Troll
 *
 * These may never be used within a server context.  However,
 * gsoap-2.7.0e requires these functions to be defined.  Thus,
 * we need to define them here.
 *
 */
SOAP_FMAC3 void SOAP_FMAC4 soap_markelement(struct soap *soap, const void *a,
                                            int b) __attribute__ ((weak));
SOAP_FMAC3 void SOAP_FMAC4
soap_markelement(struct soap *soap, const void *a, int b)
{
    gsoapRequestConfiguration *pRqConf = getRequestConfiguration(soap);

    return pRqConf->pIntf->soap_markelement(soap, a, b);
}

SOAP_FMAC3 int SOAP_FMAC4 soap_putelement(struct soap *soap, const void *a,
                                          const char *b, int c, int d)
    __attribute__ ((weak));
SOAP_FMAC3 int SOAP_FMAC4
soap_putelement(struct soap *soap, const void *a, const char *b, int c, int d)
{
    gsoapRequestConfiguration *pRqConf = getRequestConfiguration(soap);

    return pRqConf->pIntf->soap_putelement(soap, a, b, c, d);
}

SOAP_FMAC3 void *SOAP_FMAC4 soap_getelement(struct soap *soap, int *a)
    __attribute__ ((weak));
SOAP_FMAC3 void *SOAP_FMAC4
soap_getelement(struct soap *soap, int *a)
{
    gsoapRequestConfiguration *pRqConf = getRequestConfiguration(soap);

    return pRqConf->pIntf->soap_getelement(soap, a);
}
