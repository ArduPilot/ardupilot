/** Apache gSOAP module for Apache 2.0
 * @file mod_gsoap.c
 *
 * originator: Christian Aberger (http://www.aberger.at)
 * ported to Apache 2.0 by Mick Wall (mick@mickandwendy.com)
 * updated by Robert van Engelen (engelen@acm.org)
 * updated by David Viner (dviner@apache.org)
 * updated by Ryan Troll (patch removed)
 * updated by La Cam Chung and Robert van Engelen (HTTP GET ?wsdl to get WSDL)
 *
 * Contributed to the gSOAP package under the terms and conditions of the gSOAP
 * public license.
 *
 */
#if !defined(__GNUC__) || __GNUC__ < 2 || \
    (__GNUC__ == 2 && __GNUC_MINOR__ < 7) ||\
    defined(NEXT)
#ifndef __attribute__
#define __attribute__(__x)
#endif
#endif

#include <stdio.h>
#include <assert.h>

/* #include <ltdl.h> */
#include <dlfcn.h>
#include "apr_strings.h"
#include "apr_fnmatch.h"
#include "apr_strings.h"
#include "apr_lib.h"
#include "apr_pools.h"

#define APR_WANT_STRFUNC
#include "apr_want.h"

#include "httpd.h"
#include "http_request.h"
#include "http_config.h"
#include "http_core.h"
#include "http_log.h"
#include "http_main.h"
#include "http_protocol.h"
#include "http_request.h"
#include "util_script.h"
#undef HAVE_TIMEGM              /* stop complaining */
#include "stdsoap2.h"           /* standard header for gsoap */
#include "apache_gsoap.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef int Bool;

#define FALSE 0
#define TRUE ((int)0xffff)
#define IOBUF_CHUNK_SIZE 8192
#define GSOAP_ID "Apache 2.0 mod_gsoap gsoap httpd extension 0.0.9"

static char mod_gsoap_id[] = GSOAP_ID;

/** A shared library containing a SOAP server.
 */
typedef struct SoapSharedLibrary_S
{
    apr_pool_t *m_pPool;
#ifdef WIN32
#define DLSYM(a,b) GetProcAddress(a,b)
#define DLOPEN(a,b) LoadLibrary(a)
    HMODULE m_hLibrary;
#else
#define DLSYM(a,b) dlsym(a,b)
#define DLOPEN(a,b) dlopen(a,b)
    void *m_hLibrary;           /* handle of the loaded libray */
#endif
    const char *m_pszPath;      /* the path of the library, including the name */
    Bool m_bIsSOAPLibrary;      /* is this library a SOAP library that contains the server factory entry point? */
} SoapSharedLibrary;

/** Table of shared libraries that are already loaded.
 * a singleton.
 */
typedef struct SoapSharedLibraries_S
{
    apr_pool_t *m_pPool;
    SoapSharedLibrary *m_pSOAPLibrary;  /* the main SOAP library that will serve our requests */
    apr_array_header_t *m_pLibraries;   /* the array where we store our libraries. */
    apache_init_soap_interface_fn m_pfnEntryPoint;
    struct apache_soap_interface *m_pIntf;
    Bool m_bAllLibrariesLoaded; /* have all libraries in our libraries collection been already successfully loaded? */
} SoapSharedLibraries;

/** Environment to which record applies (directory,
 * server, or combination).
 */
typedef enum enConfigurationType
{
    ct_server = 1,              /* used for per-server configuration */
    ct_directory = 2,           /* used for per-directory configuration */
    ct_both = 3                 /* used for both */
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
typedef struct gsoapConfiguration_S
{
    SoapSharedLibraries *m_pLibraries;
    ConfigurationType m_Type;   /* the type of configuration environment */
} gsoapConfiguration;

/** Our internal per request soap configuration
 */
typedef struct gsoapRequestConfiguration_S
{
    request_rec *r;             /* the current request record. */
    char *m_pszAllHeaders;      /* all headers received as a string, this is returned to gsoap's http_parse function before we return the body. */
    const char *m_pszCurrentHeaderReadingPosition;  /* the position where the next header read operation will start. */
    unsigned int m_nHeaderLength;   /* total length of all headers concatenated as string */
    char *m_pOutBuf;            /* output buffer */
    size_t m_nOutBufLength;     /* allocated length of output buffer */
    size_t m_nOutBufCount;      /* bytes in output buffer */
    int headers_sent;           /* have http - headers already been sent to client us? */
    int headers_received;       /* have the request headers already been passed to gsoap ?  */
    int (*http_parse) (struct soap *);  /* the original gsoap parsing function. */
    struct apache_soap_interface *pIntf;
} gsoapRequestConfiguration;


/*
 * To avoid leaking memory from pools other than the per-request one, we
 * allocate a module-private pool.
 */
static apr_pool_t *the_gsoapPool = NULL;

/** @return our apr_pool_tfor gsoapConfiguration */
static apr_pool_t *
gsoapConfiguration_getModulePool()
{
    if (NULL == the_gsoapPool)
    {
        apr_pool_create_ex(&the_gsoapPool, NULL, NULL, NULL);
    }
    return the_gsoapPool;
}

static gsoapConfiguration *getConfiguration(request_rec *r);
static gsoapRequestConfiguration *getRequestConfiguration(struct soap *);

/**
   @param p the apr_pool_tto use for memory allocations.
   @param pszPath the path of the library.
*/
static void
SoapSharedLibrary_init(SoapSharedLibrary *This, apr_pool_t *p, const SoapSharedLibrary *pLib)
{
    This->m_pPool = p;
    This->m_hLibrary = NULL;
    This->m_pszPath = apr_pstrdup(p, pLib->m_pszPath);
    This->m_bIsSOAPLibrary = pLib->m_bIsSOAPLibrary;
}

static void
SoapSharedLibrary_init2(SoapSharedLibrary *This, apr_pool_t *p, const char *pszPath)
{
    This->m_pPool = p;
    This->m_hLibrary = NULL;
    This->m_pszPath = apr_pstrdup(p, pszPath);
    This->m_bIsSOAPLibrary = FALSE;
}

static void
SoapSharedLibrary_clear(SoapSharedLibrary *This, apr_pool_t *p)
{
    This->m_pPool = p;
    This->m_hLibrary = NULL;
    This->m_pszPath = NULL;
    This->m_bIsSOAPLibrary = FALSE;
}

static SoapSharedLibrary *
SoapSharedLibrary_create(apr_pool_t *p)
{
    SoapSharedLibrary *This = (SoapSharedLibrary *)apr_pcalloc(p, sizeof(SoapSharedLibrary));
    SoapSharedLibrary_clear(This, p);
    return This;
}

/*
 * SoapSharedLibrary_assign(SoapSharedLibrary *This, SoapSharedLibrary *pLib)
 * {
 * This->m_pPool = pLib->m_pPool;
 * This->m_hLibrary = NULL;
 * This->m_pszPath = ap_strdup(m_pPool, pLib->m_pszPath);
 * This->m_bIsSoapLibrary = pLib->m_bIsSoapLibrary;
 * }
 */
/**
 *      @param pTempPool pool to use for allocating temporary objects (e.g. error message).
 */
static const char *
SoapSharedLibrary_load(SoapSharedLibrary *This, apr_pool_t *pTempPool)
{
    const char *pszError = NULL;

#ifdef WIN32
    This->m_hLibrary = DLOPEN(This->m_pszPath);

    if (!This->m_hLibrary)
    {
        LPVOID lpMsgBuf;

        FormatMessage(FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS, NULL, GetLastError(), MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), /* Default language */
                      (LPTSTR) & lpMsgBuf, 0, NULL);
        pszError =
            apr_psprintf(NULL == pTempPool ? This->m_pPool : pTempPool,
                         "Load of %s failed with %s", This->m_pszPath,
                         lpMsgBuf);
        LocalFree(lpMsgBuf);

    }
#else
    const int nFlags = RTLD_LAZY | RTLD_LOCAL; /* RTLD_LOCAL allows multiple mod_gsoap modules to be concurrently loaded, was: RTLD_GLOBAL; */

    This->m_hLibrary = (void *)DLOPEN(This->m_pszPath, nFlags);
    pszError = dlerror();
#endif
    if (NULL == This->m_hLibrary)
    {
        pszError = apr_psprintf(NULL == pTempPool ? This->m_pPool : pTempPool,
                         "mod_gsoap: %s loading library %s", pszError, This->m_pszPath);
    }

    return pszError;
}

/**
 * @return NULL on success or error message if an error occurred.
 */
static const char *
SoapSharedLibrary_unload(SoapSharedLibrary *This)
{
    const char *pszError = NULL;
    if (NULL != This->m_hLibrary) 
    {
#ifdef WIN32
        FreeLibrary(This->m_hLibrary);
#else
        int nClose = dlclose(This->m_hLibrary);
        pszError = dlerror();
#endif
        This->m_hLibrary = NULL; 
    }
    return pszError;
}

static void
SoapSharedLibraries_init(SoapSharedLibraries *This, apr_pool_t *p)
{
    This->m_pSOAPLibrary = NULL;
    This->m_pPool = p;
    This->m_pSOAPLibrary = NULL;
    This->m_pLibraries = apr_array_make(p, 0, sizeof(SoapSharedLibrary **));
    This->m_bAllLibrariesLoaded = FALSE;
    This->m_pfnEntryPoint = NULL;
    This->m_pIntf = (struct apache_soap_interface *)apr_pcalloc(p, sizeof(struct apache_soap_interface));
}

static SoapSharedLibrary *
SoapSharedLibraries_getLibrary(SoapSharedLibraries *This, unsigned nIndex)
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
static Bool
SoapSharedLibraries_contains(SoapSharedLibraries *This, const char *pszPath)
{
    int i = 0;
    Bool bContains = FALSE;

    for (i = 0; i < This->m_pLibraries->nelts && !bContains; i++)
    {
        SoapSharedLibrary *pLib = SoapSharedLibraries_getLibrary(This, i);

        assert(NULL != pLib);
        if (0 == strcmp(pszPath, pLib->m_pszPath))
        {
            bContains = TRUE;
        }
    }
    return bContains;
}

static void
SoapSharedLibraries_addLibrary(SoapSharedLibraries *This, SoapSharedLibrary *pLibrary)
{
    assert(NULL != pLibrary);
    This->m_bAllLibrariesLoaded = FALSE;
    if (!SoapSharedLibraries_contains(This, pLibrary->m_pszPath))
    {
        SoapSharedLibrary **ppNewLib = (SoapSharedLibrary **) apr_array_push(This->m_pLibraries);
        *ppNewLib = pLibrary;
        if (pLibrary->m_bIsSOAPLibrary)
        {
            This->m_pSOAPLibrary = pLibrary;
        }
    }
}

static const char * 
SoapSharedLibraries_getEntryPoints(SoapSharedLibraries *This, SoapSharedLibrary *pLib, apr_pool_t *pTempPool, request_rec *r)
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
SoapSharedLibraries_loadAllLibraries(SoapSharedLibraries *This, apr_pool_t *pTempPool, request_rec *r)
{
    Bool bAllLibrariesLoaded = FALSE;
    const char *pszError = NULL;
    Bool bRetry = FALSE;
    int i = 0;
    int nRetry = 0;

    assert(NULL != This);

    if (This->m_bAllLibrariesLoaded)
    {
        return NULL;
    }
    for (nRetry = 0; nRetry < 5 && !bAllLibrariesLoaded; nRetry++)
    {
        do
        {
            pszError = NULL;
            bRetry = FALSE;     /* don't try it again. */
            bAllLibrariesLoaded = TRUE;
            for (i = 0; i < This->m_pLibraries->nelts; i++)
            {
                SoapSharedLibrary *pLib = SoapSharedLibraries_getLibrary(This, i);
                if (NULL != pLib && NULL == pLib->m_hLibrary)
                {
                    pszError = SoapSharedLibrary_load(pLib, pTempPool);
                    if (NULL == pszError)
                    {
                        assert(NULL != pLib->m_hLibrary);
                        bRetry = TRUE;  /* we loaded one, lets retry with all others, maybe they depend on that */
                    }
                    else
                    {
                        bAllLibrariesLoaded = FALSE;
                    }
                    if (NULL != pLib->m_hLibrary && pLib->m_bIsSOAPLibrary)
                    {
                        void *pfun = (void *)DLSYM(pLib->m_hLibrary, APACHE_HTTPSERVER_ENTRY_POINT);

                        if (NULL == pfun)
                        {
                            pszError = apr_psprintf(pTempPool, "gsoap: httpd.conf module mod_gsoap.c SOAPLibrary load \"%s\" success, but failed to find the \"%s\" function entry point defined by IMPLEMENT_GSOAP_SERVER()", pLib->m_pszPath, APACHE_HTTPSERVER_ENTRY_POINT);
                            return pszError;
                        }
                        else
                        {
                            This->m_pfnEntryPoint = (apache_init_soap_interface_fn) pfun;
                            pszError = SoapSharedLibraries_getEntryPoints(This, pLib, pTempPool, r);
                            pszError =
                                apr_psprintf(NULL == pTempPool ? This->m_pPool : pTempPool,
                                             "mod_gsoap: got entrypoint %s from library",
                                             APACHE_HTTPSERVER_ENTRY_POINT);
                        }
                    }
                }
            }
        }
        while (bRetry);
    }
    if (bAllLibrariesLoaded)
    {
        This->m_bAllLibrariesLoaded = TRUE;
        pszError = NULL;
    }
    return pszError;
}

static const char *
SoapSharedLibraries_unloadAllLibraries(SoapSharedLibraries *This)
{
    const char *pszError = NULL;
    int i = 0; 
    assert(NULL != This);
    This->m_bAllLibrariesLoaded = FALSE;
    for (i = 0; i < This->m_pLibraries->nelts && NULL == pszError; i++)
    {
        SoapSharedLibrary *pLib = SoapSharedLibraries_getLibrary(This, i);
        if (NULL != pLib && NULL != pLib->m_hLibrary)
        {
            const char *pszErr = SoapSharedLibrary_unload(pLib);
            if (NULL == pszError)
            {
                pszError = pszErr;
            }
        }
    }
    return pszError;
}

static void
SoapSharedLibraries_clear(SoapSharedLibraries *This)
{
    int i = 0;
    SoapSharedLibraries_unloadAllLibraries(This);
    for (i = 0; i < This->m_pLibraries->nelts; i++)
    {
        SoapSharedLibrary *pLib = SoapSharedLibraries_getLibrary(This, i);
        SoapSharedLibrary_clear(pLib, This->m_pPool);
    }
}

static void
SoapSharedLibraries_merge(SoapSharedLibraries *This, SoapSharedLibraries *pLibs)
{
    int i = 0;

    assert(NULL != This);
    if (NULL == pLibs)
    {
        return;
    }
    This->m_bAllLibrariesLoaded = FALSE;
    for (i = 0; i < pLibs->m_pLibraries->nelts; i++)
    {
        SoapSharedLibrary *pLib = SoapSharedLibraries_getLibrary(pLibs, i);

        if (!SoapSharedLibraries_contains(This, pLib->m_pszPath))
        {
            SoapSharedLibrary *pNewLib =
                SoapSharedLibrary_create(This->m_pPool);
            SoapSharedLibrary_init(pNewLib, This->m_pPool, pLib);
            SoapSharedLibraries_addLibrary(This, pNewLib);
        }
    }
}

static void
SoapSharedLibraries_merge3(SoapSharedLibraries *This, SoapSharedLibraries *libraries1, SoapSharedLibraries *libraries2)
{
    SoapSharedLibraries_merge(This, libraries1);
    SoapSharedLibraries_merge(This, libraries2);
}

static void
gsoapConfiguration_merge(gsoapConfiguration *This, gsoapConfiguration *pParentConfig, gsoapConfiguration *pNewConfig)
{
    assert(NULL != This);
    SoapSharedLibraries_merge3(This->m_pLibraries, pParentConfig->m_pLibraries, pNewConfig->m_pLibraries);
    This->m_Type = ct_both;
}

static void
gsoapConfiguration_init(gsoapConfiguration *This, apr_pool_t *p)
{
    This->m_pLibraries = (SoapSharedLibraries *) apr_pcalloc(p, sizeof(SoapSharedLibraries));
    SoapSharedLibraries_init(This->m_pLibraries, p);
    This->m_Type = ct_directory;
}

static gsoapConfiguration *
gsoapConfiguration_create(apr_pool_t *p)
{
    gsoapConfiguration *pConfig = (gsoapConfiguration *) apr_pcalloc(p, sizeof(gsoapConfiguration));
    gsoapConfiguration_init(pConfig, p);
    return pConfig;
}

/*
 * forward declarations                                                     
 */
static int gsoap_handler(request_rec *r);
static int gsoap_init(apr_pool_t *p, apr_pool_t *ptemp, apr_pool_t *plog, server_rec *psrec);
static void *gsoap_create_dir_config(apr_pool_t *p, char *dirspec);
static void *gsoap_merge_dir_config(apr_pool_t *p, void *parent_conf, void *newloc_conf);
static void *gsoap_create_server_config(apr_pool_t *p, server_rec *s);
static void *gsoap_merge_server_config(apr_pool_t *p, void *server1_conf, void *server2_conf);

static Bool AddSharedLibrary(gsoapConfiguration *pConfig, const char *pszPath, const Bool bIsSOAPLibrary);

/*
 * *
 * * We prototyped the various syntax for command handlers (routines that     
 * * are called when the configuration parser detects a directive declared    
 * * by our module) earlier.  Now we actually declare a "real" routine that   
 * * will be invoked by the parser when our "real" directive is               
 * * encountered.                                                             
 * 
 * * If a command handler encounters a problem processing the directive, it   
 * * signals this fact by returning a non-NULL pointer to a string            
 * * describing the problem.                                                  
 * 
 * * The magic return value DECLINE_CMD is used to deal with directives       
 * * that might be declared by multiple modules.  If the command handler      
 * * returns NULL, the directive was processed; if it returns DECLINE_CMD,    
 * * the next module (if any) that declares the directive is given a chance   
 * * at it.  If it returns any other value, it's treated as the text of an    
 * * error message.                                                           
 */

/** Command handler for the TAKE1 "SOAPLibrary" directive.
 * We remember the load path for the shared library that contains the SOAP server.
 */
static const char *
cmd_SoapLibrary(cmd_parms * cmd, void *mconfig, const char *pszPath)
{
    gsoapConfiguration *pConfig = (gsoapConfiguration *) mconfig;

    AddSharedLibrary(pConfig, pszPath, TRUE);
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

    AddSharedLibrary(pConfig, pszPath, FALSE);
    return NULL;
}

typedef const char *(*command_function_interface) ();

/** List of directives specific to our module.
 */
static const command_rec gsoap_cmds[] = {
    AP_INIT_TAKE1("SOAPLibrary",    /* directive name */
                  (command_function_interface) cmd_SoapLibrary, /* config action routine */
                  NULL,         /* argument to include in call */
                  ACCESS_CONF,  /* where available */
                  "SOAP shared library that will be dynamically loaded. - 1 argument (path)"    /* directive description */
        ),
    AP_INIT_TAKE1("SupportLibrary", /* directive name */
                  (command_function_interface) cmd_SupportLibrary,  /* config action routine */
                  NULL,         /* argument to include in call */
                  ACCESS_CONF,  /* where available */
                  "additional library that must be dynamically loaded - 1 argument (path)"  /* directive description */
        ),
    {NULL}
};

/*
 * All of the routines have been declared now.  Here's the list of          
 * directives specific to our module, and information about where they      
 * may appear and how the command parser should pass them to us for         
 * processing.  Note that care must be taken to ensure that there are NO    
 * collisions of directive names between modules.                           
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

static void
gsoap_hooks(apr_pool_t *p)
{
    /* I think this is the call to make to register a handler for method calls (GET PUT et. al.). */
    /* We will ask to be last so that the comment has a higher tendency to */
    /* go at the end. */
    ap_hook_handler(gsoap_handler, NULL, NULL, APR_HOOK_LAST);

    /* Register a handler for post config processing */
    ap_hook_post_config(gsoap_init, NULL, NULL, APR_HOOK_MIDDLE);
}

module AP_MODULE_DECLARE_DATA gsoap_module = {
    STANDARD20_MODULE_STUFF,
    gsoap_create_dir_config,    /* per-directory config creator */
    gsoap_merge_dir_config,     /* dir config merger */
    gsoap_create_server_config, /* server config creator */
    gsoap_merge_server_config,  /* server config merger */
    gsoap_cmds,                 /* command table */
    gsoap_hooks,                /* hooks */
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
    apr_table_do(ListHeadersCallback, r, r->headers_in, NULL);
}

/** send the error message to the client browser */
static void
SendErrorMessage(request_rec * r, const char *pszError)
{
    ap_log_error(APLOG_MARK, APLOG_ERR, 0, r->server, "mod_gsoap: %s",
                 pszError);
    r->content_type = "text/html";
    ap_rputs(DOCTYPE_HTML_3_2, r);
    ap_rputs("<HTML>\n", r);
    ap_rputs(" <HEAD>\n", r);
    ap_rputs("  <TITLE>Apache mod_gsoap Handler\n", r);
    ap_rputs("  </TITLE>\n", r);
    ap_rputs(" </HEAD>\n", r);
    ap_rputs(" <BODY>\n", r);
    ap_rputs("  <H1>mod_gsoap Apache SOAP Server Error</H1>\n", r);
    ap_rprintf(r, "<p><strong>%s</strong><br>Please see the README instructions with the mod_gsoap package for details.</p>", pszError);
 
    ap_rputs("  <H2>Content headers of the request</H2>\n", r);
    ListHeaders(r);
    ap_rputs("</BODY></HTML>\n", r);
}

/* This is to handle HTTP GET request -- La Cam Chung & R. van Engelen */
static void
HTTPGet_SendWSDL(request_rec *r, const char *path)
{
    FILE *fd = NULL;
    char *file;
    size_t n;
    char *tmp; 
    size_t size;
    size_t l;
    /* GET wsdl file */
    if (!path || !r->parsed_uri.query || strcmp(r->parsed_uri.query, "wsdl"))
    {
        SendErrorMessage(r, "HTTP GET: not a ?wsdl request");
        return;
    }
    l = strlen(path);
    file = (char*)malloc(l + 6);
    if (file != NULL)
    {
        soap_strcpy(file, l + 6, path);
        tmp = strstr(file, ".so");
        if (tmp)
            soap_strcpy(tmp, 9, ".wsdl");
        else
            soap_strcpy(file + l, 6, ".wsdl");
        fd = fopen(file, "rb");
        free(file);
    }
    if (fd == NULL)
    {
        SendErrorMessage(r, "Error: 404 FILE NOT FOUND");
        return;
    }
    /* Calculate the size of file */
    fseek(fd, 0, SEEK_END);
    size = ftell(fd);
    rewind(fd);
    tmp = (char*)malloc(size + 1);
    if (tmp)
        n = fread(tmp, 1, size, fd);
    if (!tmp || n != size)
        SendErrorMessage(r, "HTTP GET error reading file");
    else
    {
        tmp[size] = '\0';
        r->content_type = "text/xml";
        ap_rputs(tmp, r);
    }
    fclose(fd);
    if (tmp)
        free(tmp);
    return;
}

static int
send_header_to_gsoap(void *pvoid, const char *key, const char *value)
{
    struct soap *psoap = (struct soap *)pvoid;
    gsoapRequestConfiguration *pRqConf = NULL;

    if (NULL == psoap)
    {
      return 0;
    }

    pRqConf = getRequestConfiguration(psoap);

    if (NULL == pRqConf)
    {
        return 0;
    }

    if (0 == strcasecmp(key, "SOAPAction") ||
	0 == strcasecmp(key, "Content-Type") ||
	0 == strcasecmp(key, "Content-Length"))
    {
        psoap->fparsehdr(psoap, key, value);
    }

    return 1;
}

/*
 * Callback functions for gsoap. We must parse the headers ourselves and 
 *  we must handle send/receive properly.  
 */
static int
http_post_header(struct soap *soap, const char *key, const char *value)
{
    gsoapRequestConfiguration *pRqConf = getRequestConfiguration(soap);
    request_rec *r = NULL == pRqConf ? NULL : pRqConf->r;

    if (NULL != key && NULL != value)
    {
        if (0 == strcasecmp(key, "SOAPAction"))
        {
            apr_table_set(r->headers_out, key, value);
        }
        else if (0 == strcasecmp(key, "Content-Type"))
        {
            r->content_type = apr_pstrdup(r->pool, value);
        }
        else if (0 == strcasecmp(key, "Content-Length"))
        {
            ap_set_content_length(r, atoi(value));
        }
    }
    else if (NULL != key)
    {
        if (0 == strncmp(key, "Status: ", 8))
        {
            int status = atoi(key + 8);
            if (status > 200)
              r->status = status;
        }
    }

    return SOAP_OK;
}

/** 
gsoap function that requests the next piece of data from us
*/
static size_t
frecv(struct soap *psoap, char *pBuf, apr_size_t len)
{
    request_rec *r = NULL;
    apr_size_t nRet = 0;
    gsoapRequestConfiguration *pRqConf = getRequestConfiguration(psoap);

    if (NULL != pRqConf)
    {
        r = pRqConf->r;
        if (!pRqConf->headers_received)
        {
#if 0       /* useragent_ip is available since Apache 2.4 but preferred over client_ip */
            char *s = r->useragent_ip;
#else
            char *s = r->connection ? r->connection->client_ip : NULL;
#endif
            psoap->ip = 0;
            *psoap->host = '\0';
            if (s)
            {
              int i;
              soap_strcpy(psoap->host, sizeof(psoap->host), s);
              if (strchr(s, '.'))
              {
                /* client IP is IPv4 formatted */
                for (i = 0; i < 4 && *s; i++)
                {
                  psoap->ip = (psoap->ip << 8) + (unsigned int)soap_strtoul(s, &s, 10);
                  if (*s)
                    s++;
                }
              }
#ifndef WIN32
              else /* THIS BRANCH CAN BE REMOVED IF inet_pton() IS NOT AVAILABLE SUCH AS ON WINDOWS */
              {
                /* client IP is IPv6 formatted */
                struct in6_addr result;
                if (inet_pton(AF_INET6, s, &result) == 1)
                  for (i = 0; i < 16; i++)
                    psoap->ip6[i/4] = (psoap->ip6[i/4] << 8) + result.s6_addr[i];
              }
#endif
            }

            apr_table_do(send_header_to_gsoap, psoap, r->headers_in, NULL);
            pRqConf->headers_received = TRUE;
        }
        /*
         * Steven Elliott <selliott4@austin.rr.com>:
         * It's not obvious from Apache documentation how
         * exactly ap_get_client_block() should be called in both the chunked
         * and non-chunked cases.  In the chunked case r->remaining is zero,
         * so that can't be an upper bound to the number of bytes read.
         * However, this site:
         *   http://docstore.mik.ua/orelly/apache_mod/139.htm 
         * cautions that "If you are handling non-chunked data, do not try to
         * read more than the number of bytes declared in Content-length
         * because this may cause the attempted read to block indefinitely."
         * Consequently my best guess, with regard to what is safe, is in the
         * non-chunked case continue doing as mod_gsoap has done in the past
         * and limit the upper bound to r->remaining.  In the chunked case
         * read "len", which almost certainly exceeds what was posted.
         * Hopefully if ap_get_client_block() blocks it returns no later than 
         * when the current post is complete regardless of chunking or
         * keep-alive.
         */
        nRet = ap_get_client_block(r, pBuf,
            r->read_chunked ? len : (len > r->remaining ? r->remaining : len));
    }
    return nRet;
}

static int
fsend(struct soap *psoap, const char *pBuf, apr_size_t len)
{
    int nRet = SOAP_OK;
    gsoapRequestConfiguration *pRqConf = getRequestConfiguration(psoap);

    if (NULL != pRqConf)
    {
        request_rec *r = pRqConf->r;

        if (!pRqConf->headers_sent)
        {
            /*          ap_send_http_header(r); */
            pRqConf->headers_sent = TRUE;
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

/** make sure we go through soap_send_fault(), which calls fpoll */
static int
fpoll(struct soap *psoap)
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
    /*
     * should this be a deep copy? 
     */
    *dst = *src;
    return SOAP_OK;
}

static void
mod_gsoap_delete(struct soap *soap, struct soap_plugin *p)
{
}

static int
mod_gsoap_plugin(struct soap *soap, struct soap_plugin *p, void *arg)
{
    p->id = mod_gsoap_id;
    p->data = arg;
    p->fcopy = mod_gsoap_plugin_copy;
    p->fdelete = mod_gsoap_delete;
    return SOAP_OK;
}

static void
set_callbacks(request_rec * r, gsoapRequestConfiguration * pRqConf, struct soap *psoap)
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
    psoap->fpoll = fpoll;
    if (NULL != pIntf->fsoap_init)
    {
        (*pIntf->fsoap_register_plugin_arg)(psoap, mod_gsoap_plugin, (void *)pRqConf, r);
    }
    else
    {
        psoap->user = pRqConf;
    }
    if (NULL != pIntf->fsoap_user_init)
    {
        (*pIntf->fsoap_user_init)(psoap, r);
    }
}

/*
 * 
 * * Now we declare our content handlers, which are invoked when the server   
 * * encounters a document which our module is supposed to have a chance to   
 * * see.  (See mod_mime's SetHandler and AddHandler directives, and the      
 * * mod_info and mod_status examples, for more details.)                     
 * 
 * * Since content handlers are dumping data directly into the connection      
 * * (using the r*() routines, such as rputs() and rprintf()) without         
 * * intervention by other parts of the server, they need to make             
 * * sure any accumulated HTTP headers are sent first.  This is done by       
 * * calling send_http_header().  Otherwise, no header will be sent at all,   
 * * and the output sent to the client will actually be HTTP-uncompliant.     
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
    char *pszResponse = NULL;
    gsoapConfiguration *pConfig = getConfiguration(r);
    gsoapRequestConfiguration *pRqConf = NULL;

    /*
     * only handle soap requests 
     */
    if (!strstr(r->handler, "soap"))
        return DECLINED;

    /*
     * only handle POST and GET requests 
     */
    if (r->method_number != M_POST && r->method_number != M_GET)
        return DECLINED;

    pszResponse = apr_pcalloc(r->pool, nResponseBufferLen);
    assert(NULL != pConfig);

    psoap = (struct soap *)apr_pcalloc(r->pool, sizeof(struct soap));
    pRqConf = apr_pcalloc(r->pool, sizeof(gsoapRequestConfiguration));
    pszError = SoapSharedLibraries_loadAllLibraries(pConfig->m_pLibraries, r->pool, r);

    pIntf = pConfig->m_pLibraries->m_pIntf;

    ap_update_mtime(r, r->request_time);
    ap_set_last_modified(r);
    if (NULL != pszError)
    {
        static Bool bFirstTime = TRUE;

        if (bFirstTime)
        {
            ap_log_error(APLOG_MARK, APLOG_ERR, 0, r->server, "mod_gsoap: %s", pszError);
            bFirstTime = FALSE;
        }
    }

    /*
     * as a next step, we prepare a buffer that sends the request as first line to gsoap. 
     * Then the remaining data. 
     * We start returning bytes on frecv from this buffer, until it is empty. 
     * then it is not necessary to fiddle around with gsoap's request line parsing.
     */
    if (NULL == pszError)
    {
        pRqConf->r = r;
        pRqConf->headers_sent = FALSE;
        pRqConf->headers_received = FALSE;
        pRqConf->m_pszAllHeaders = NULL;
        pRqConf->m_nHeaderLength = strlen(r->the_request) + 2;
        pRqConf->m_pszCurrentHeaderReadingPosition = NULL;
        pRqConf->m_nOutBufCount = 0;
        pRqConf->m_nOutBufLength = nResponseBufferLen;
        pRqConf->m_pOutBuf = apr_pcalloc(r->pool, nResponseBufferLen);
        pRqConf->http_parse = NULL;
        pRqConf->m_pszAllHeaders = apr_pcalloc(r->pool, pRqConf->m_nHeaderLength + 3);
        pRqConf->m_pszCurrentHeaderReadingPosition = pRqConf->m_pszAllHeaders;
        soap_strcpy(pRqConf->m_pszAllHeaders, pRqConf->m_nHeaderLength + 3, r->the_request);
        soap_strcpy(pRqConf->m_pszAllHeaders + strlen(pRqConf->m_pszAllHeaders), 3, "\r\n");
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
     */

    /*
     * If we're only supposed to send header information (HEAD request), we're
     * already there.
     */
    if (r->header_only)
    {
        return OK;
    }
    if (NULL != pszError)
    {
        SendErrorMessage(r, pszError);
        return OK;
    }
    nRet = ap_setup_client_block(r, REQUEST_CHUNKED_DECHUNK);
    if (OK != nRet)
    {
        SendErrorMessage(r, "Failed to start receiving POST buffer");
        return OK;
    }
    nRet = ap_should_client_block(r);

    /* we check whether this request is HTTP GET or not and handle the GET request */
    if (r->method_number == M_GET)
    {  
        HTTPGet_SendWSDL(r, pConfig->m_pLibraries->m_pSOAPLibrary->m_pszPath);
        return OK;
    }

    if (0 == nRet)
    {
        SendErrorMessage(r, "No body received");
        return OK;
    }

    if (NULL != pszError)
    {
        ap_log_error(APLOG_MARK, APLOG_ERR, 0, r->server, "mod_gsoap: %s", pszError);
        SendErrorMessage(r, pszError);
        return OK;
    }

    if (NULL != pIntf->fsoap_init)
    {
        (*pIntf->fsoap_init)(psoap, r);
        psoap->namespaces = pIntf->namespaces;
        set_callbacks(r, pRqConf, psoap);
        if (NULL != pIntf->fsoap_serve)
        {
            nRet = (*pIntf->fsoap_serve) (psoap, r);
        }
        else
        {
            SendErrorMessage(r, "no soap_serve entry point");
            return OK;
        }
        if (NULL != pIntf->fsoap_destroy)
        {
            pIntf->fsoap_destroy(psoap, r); /* not an error in 2.1.10 anymore */
        }
        if (NULL != pIntf->fsoap_end)
        {
            pIntf->fsoap_end(psoap, r);
        }
        else
        {
            SendErrorMessage(r, "no soap_end entry point");
        }
        if (NULL != pIntf->fsoap_done)
        {
            pIntf->fsoap_done(psoap, r);
        }
        else
        {
            SendErrorMessage(r, "no soap_done entry point");
        }
    }
    else
    {
        SendErrorMessage(r, "no soap_init entry point");
        return OK;
    }

    /*
     * We did what we wanted to do, so tell the rest of the server we
     * succeeded. We need not delete pszResponse, because it was allocated from the request pool.
     */
    return OK;
}

/*
 * Now let's declare routines for each of the callback phase in order.      
 * (That's the order in which they're listed in the callback list, *not     
 * the order in which the server calls them!  See the command_rec           
 * declaration).  Note that these may be                                    
 * called for situations that don't relate primarily to our function - in   
 * other words, the fixup handler shouldn't assume that the request has     
 * to do with "gsoap" stuff.                                                
 
 * With the exception of the content handler, all of our routines will be   
 * called for each request, unless an earlier handler from another module   
 * aborted the sequence.                                                    
 *
 * Handlers that are declared as "int" can return the following:            
 * OK          Handler accepted the request and did its thing with it.     
 * DECLINED    Handler took no action.                                     
 * HTTP_mumble Handler looked at request and found it wanting.             
 
 * What the server does after calling a module handler depends upon the     
 * handler's return value.  In all cases, if the handler returns            
 * DECLINED, the server will continue to the next module with an handler    
 * for the current phase.  However, if the handler return a non-OK,         
 * non-DECLINED status, the server aborts the request right there.  If      
 * the handler returns OK, the server's next action is phase-specific;      
 * see the individual handler comments below for details.                   
 */

/*
 * 
 * * This function is called during server initialisation.  Any information
 * * that needs to be recorded must be in static cells, since there's no
 * * configuration record.
 * *
 * * There is no return value.
 */
static int
gsoap_init(apr_pool_t * p, apr_pool_t * ptemp, apr_pool_t * plog, server_rec * psrec)
{
    ap_log_error(APLOG_MARK, APLOG_INFO, 0, psrec, "mod_gsoap initialized");
    return OK;
}

/* 
 * This function is called during server initialisation when an heavy-weight
 * process (such as a child) is being initialised.  As with the
 * module-initialisation function, any information that needs to be recorded
 * must be in static cells, since there's no configuration record.
 *
 * There is no return value.
 */

static void gsoap_child_init(server_rec *s, apr_pool_t*p)
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

static void gsoap_child_exit(server_rec *s, apr_pool_t*p)
{
    /* gsoapConfiguration::getLibraries()->clear(); */
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
gsoap_create_dir_config(apr_pool_t * p, char *dirspec)
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
gsoap_merge_dir_config(apr_pool_t * p, void *parent_conf, void *newloc_conf)
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
gsoap_create_server_config(apr_pool_t * p, server_rec * s)
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
gsoap_merge_server_config(apr_pool_t * p, void *server1_conf, void *server2_conf)
{
    gsoapConfiguration *pMergedConfig = gsoapConfiguration_create(p);
    gsoapConfiguration *pServer1Config = (gsoapConfiguration *) server1_conf;
    gsoapConfiguration *pServer2Config = (gsoapConfiguration *) server2_conf;

    gsoapConfiguration_merge(pMergedConfig, pServer1Config, pServer2Config);
    pMergedConfig->m_Type = ct_server;
    return (void *)pMergedConfig;
}

/** helper funciton for library command handler.
 * @param pszPath the path of the library.
 * @param bIsSOAPLibrary true if it is a shared library containing a SOAP server.
 * @return true if the library was added, false if it was aleady in the collection.
 */
static Bool
AddSharedLibrary(gsoapConfiguration * pConfig, const char *pszPath, const Bool bIsSOAPLibrary)
{
    Bool bAdded = FALSE;
    apr_pool_t *pPool = gsoapConfiguration_getModulePool();

    if (!SoapSharedLibraries_contains(pConfig->m_pLibraries, pszPath))
    {
        SoapSharedLibrary *pLibrary = SoapSharedLibrary_create(pPool);

        SoapSharedLibrary_init2(pLibrary, pPool, apr_pstrdup(pPool, pszPath));
        pLibrary->m_bIsSOAPLibrary = bIsSOAPLibrary;
        SoapSharedLibraries_addLibrary(pConfig->m_pLibraries, pLibrary);
        bAdded = TRUE;
    }

    return bAdded;
}

static gsoapConfiguration *
getConfiguration(request_rec * r)
{
    return (gsoapConfiguration *) ap_get_module_config(r->per_dir_config, &gsoap_module);
}

static gsoapRequestConfiguration *
getRequestConfiguration(struct soap *soap)
{
    gsoapRequestConfiguration *pRqConf = (gsoapRequestConfiguration *) soap->fplugin(soap, mod_gsoap_id);
    return pRqConf;
}


/*
 * Patch from Ryan Troll
 *
 * Implement these as weak symbols, allowing symbol checking during
 * compilation to succeed, even when another object is actually
 * providing these symbols at runtime.
 */

/*
 * This patch may not work properly on some systems. The patch is commented
 * out in case someone is interested in using it (at your own risk).
 */

#if 0
SOAP_NMAC struct Namespace namespaces[] __attribute__ ((weak));
SOAP_NMAC struct Namespace namespaces[] = {
    {"SOAP-ENV", "http://schemas.xmlsoap.org/soap/envelope/",
     "http://www.w3.org/*\//soap-envelope"},
    {"SOAP-ENC", "http://schemas.xmlsoap.org/soap/encoding/",
     "http://www.w3.org/*\/soap-encoding"},
    {"xsi", "http://www.w3.org/2001/XMLSchema-instance",
     "http://www.w3.org/*\//XMLSchema-instance"},
    {"xsd", "http://www.w3.org/2001/XMLSchema",
     "http://www.w3.org/*\//XMLSchema"},
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

SOAP_FMAC3 void SOAP_FMAC4 soap_markelement(struct soap *soap, const void *a,
                                            int b) __attribute__ ((weak));
SOAP_FMAC3 void SOAP_FMAC4
soap_markelement(struct soap *soap, const void *a, int b)
{
    gsoapRequestConfiguration *pRqConf = getRequestConfiguration(soap);

    pRqConf->pIntf->soap_markelement(soap, a, b);
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

#endif

#ifdef __cplusplus
}
#endif

