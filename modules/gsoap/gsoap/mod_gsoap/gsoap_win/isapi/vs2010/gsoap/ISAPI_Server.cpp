/** Implementation of the ISAPI_Server class 
  * @file ISAPI_Server.cpp
  * @author Christian Aberger 
  * Copyright (C) 2001 Genivia Inc and WebWare
  * Load a gosap server dll dynamically (if not already loaded) and serve the request.
  */
#define WIN32_LEAN_AND_MEAN

#include <memory>
#include <sstream>
#ifdef __DEBUG_ISAPI
#include <fstream>
#endif
#include <cassert>
using namespace std;
#include "ISAPI_Server.h"
#include "isapistream.h"
#include "ISAPI_HttpContext.h"
#include "ISAPI_SoapServerFactory.h"

static const char *GSOAP_ID = "mod_gsoap isapi extension 0.0.2";
static const char *crlf = "\r\n";

///> helper function to build full path to dll.
static string MakeDllName(EXTENSION_CONTROL_BLOCK *pECB, const char *pszDllName); 

/* --- plugin functions -- */
static int mod_gsoap_plugin_copy(struct soap *soap, struct soap_plugin *dst, struct soap_plugin *src) {
    *dst = *src;
    return SOAP_OK;
}
static void mod_gsoap_delete(struct soap *soap, struct soap_plugin *p) {
}
static int mod_gsoap_plugin(struct soap *soap, struct soap_plugin *p, void *arg) {
    p->id = GSOAP_ID;
    p->data = arg;
    p->fcopy = mod_gsoap_plugin_copy;
    p->fdelete = mod_gsoap_delete;
    return SOAP_OK;
} 

/** bundle the request/response data into an object to be able to attach it to soap. */
class SoapTransaction {
public:
    SoapTransaction(soap *);
    virtual ~SoapTransaction();

    static SoapTransaction *transaction(soap *);
    void SendHeaders(); ///< send output headers of gsoap to client, if not done so already.
    size_t SupplyRequestHeaders(char *pBuf, const size_t len); // send input request headers to gsoap.
    void BuildHeaders(); ///< build the headers that must be sent to gsoap.
public:
    struct soap *_soap;
    std::string _header; ///< header part returned to client e.g. "200 OK".
    std::string _request_header; ///< the (remaining part of ) the first line of the request.
    DWORD _dwReturnCode; ///< the return code for the HttpExtensionProc.
    ISAPI_HttpRequest *_request;
    HttpResponse *_response;
    isapistream *_istream;
    const mod_gsoap_interface *_interface;
#ifdef _DEBUG_ISAPI
    ofstream *_debug_in; ///< debugging output stream
    ofstream *_debug_out; ///< debugging output stream
#endif
protected:
    bool _headers_sent; ///< true if Http headers have already been sent.
    bool _headers_supplied; ///< true if Http headers have already supplied to gsoap.
};

SoapTransaction::SoapTransaction(soap *soap) 
: _soap(soap)
, _dwReturnCode(HSE_STATUS_SUCCESS)
, _header("200 OK")
, _request(NULL)
, _response(NULL) 
, _interface(NULL) 
, _istream(NULL)
, _headers_sent(false)
, _headers_supplied(false)
#ifdef _DEBUG_ISAPI
, _debug_in(NULL)
, _debug_out(NULL)
#endif
{
    assert(soap);
#ifdef _DEBUG_ISAPI
    TCHAR szPath[_MAX_PATH];
    ::GetTempPath(sizeof szPath, szPath);
    string strIn = szPath;
    string strOut = szPath;
    strIn += "mod_gsoap_debug_in.log";
    strOut += "mod_gsoap_debug_out.log";
    strOut += "mod_gsoap_debug.log";
    _debug_in = new ofstream(strIn.c_str(), ios::out | ios::app);
    _debug_out = new ofstream(strOut.c_str(), ios::out | ios::app);
    *_debug_in << "--------------------------------START--------------------" << endl;
    *_debug_out << "--------------------------------START--------------------" << endl;
#endif
}
SoapTransaction::~SoapTransaction() {
#ifdef _DEBUG_ISAPI
    *_debug_in << "--------------------------------STOP---------------------" << endl;
    *_debug_out << "--------------------------------STOP---------------------" << endl;
    delete _debug_in;
    delete _debug_out;
#endif
}
inline SoapTransaction *SoapTransaction::transaction(soap *soap) {
    assert(NULL != soap);
    return reinterpret_cast<SoapTransaction *>(soap->fplugin(soap, GSOAP_ID));
}
void SoapTransaction::SendHeaders() {
    if (_headers_sent) {
        return;
    }
    ostringstream headers;
    for (HttpMessage::ContentHeaders::const_iterator it = _response->getContentHeaders().begin(); it != _response->getContentHeaders().end(); ++it) {
        if (0 != strcasecmp(it->first.c_str(), "Connection")) {
            headers << it->first << ": " << it->second << crlf;
        } 
    }
    headers << crlf;
    string strOut(headers.str());
#ifdef _DEBUG_ISAPI
    *_debug_out << strOut;
#endif
    BOOL bHeaders = _request->ECB()->ServerSupportFunction(_request->ECB()->ConnID, HSE_REQ_SEND_RESPONSE_HEADER,
        (LPVOID)"200 OK", NULL, (LPDWORD)strOut.c_str());
    _headers_sent = true;
}
/** Emits HTTP key: val header entries. 
    @return SOAP_OK, or a gSOAP error code. 
    Built-in gSOAP function: http_post_header. 

    IIS handles most of the headers (Connection etc.) itself. 
    The main thing we need to extract is SOAPAction.
*/
static int http_post_header(soap *soap, const char *key, const char *value) {
    if (NULL != key) {
        SoapTransaction *pTrans = SoapTransaction::transaction(soap);
        if (value) {
            pTrans->_response->getContentHeaders()[key] = value;
        }
    }
    return SOAP_OK;
}
/** set the http - response code from the soap error message. */
static int http_response(soap *soap, int soap_error, ULONG64 count) {
    SoapTransaction *pTrans = SoapTransaction::transaction(soap);
    if (SOAP_OK == soap_error) {
        pTrans->_dwReturnCode = HSE_STATUS_SUCCESS;
    } else {
        pTrans->_dwReturnCode = HSE_STATUS_ERROR;
        pTrans->_header = "500 ERR";
    }
    return SOAP_OK;
}
/** gsoap function to append the data to send to our output buffer */
static int fsend(soap *soap, const char *pBuf, size_t len) {
    SoapTransaction *pTrans = SoapTransaction::transaction(soap);
    pTrans->SendHeaders();
#ifdef _DEBUG_ISAPI
    pTrans->_debug_out->write(pBuf, len);
#endif
    pTrans->_istream->write(pBuf, len);
    return SOAP_OK;
}
size_t SoapTransaction::SupplyRequestHeaders(char *pBuf, const size_t len) {
    size_t nLen = 0;
    if (!_headers_supplied) {
        if (!_request_header.empty()) { // we must send 1st line so that gsoap parses everything correctly.
            nLen = _request_header.length();
            if (nLen > len) {
                nLen = len;
                memcpy_s(pBuf, len, _request_header.c_str(), len);
                _request_header = _request_header.substr(len);
            } else {
                memcpy_s(pBuf, len, _request_header.c_str(), nLen);
                _request_header.erase();                
                _headers_supplied = true;
            }
#ifdef _DEBUG_ISAPI
            _debug_in->write(pBuf, nLen);
#endif
        }
    }
    return nLen;
}
/** gsoap function that requests the next piece of data from us */
static size_t frecv(soap *soap, char *pBuf, size_t len) {
    SoapTransaction *pTrans = SoapTransaction::transaction(soap);
    size_t nLen = pTrans->SupplyRequestHeaders(pBuf, len); 
    if (0 == nLen) { // query string and headers have been sent already.
        assert(pTrans->_istream->good());
        if (!pTrans->_istream->eof()) {
            pTrans->_istream->readsome(pBuf, len);
            nLen = (unsigned int)pTrans->_istream->gcount();
#ifdef _DEBUG_ISAPI
            pTrans->_debug_in->write(pBuf, nLen);
#endif
            assert(pTrans->_istream->good());
        }
    }
    return nLen;
}

void SoapTransaction::BuildHeaders() {
    ostringstream s;
    /* we must rebuild the 1st request line, cause IIS has no API to get it :( */
    s << "POST /" + _request->_querystring + " HTTP/1.0\r\n";
    for (ISAPI_HttpRequest::ContentHeaders::const_iterator it = _request->getContentHeaders().begin(); it != _request->getContentHeaders().end(); ++it) {
        s << it->first << ": " << it->second << crlf;
    }
    s << crlf;
    _request_header = s.str();
}

BOOL ISAPI_Server::GetExtensionVersion(HSE_VERSION_INFO *pVer) {
    lstrcpyn((LPSTR) pVer->lpszExtensionDesc, "WebWare SOAP ISAPI extension", HSE_MAX_EXT_DLL_NAME_LEN);
    return TRUE;
}

BOOL ISAPI_Server::TerminateExtension(DWORD) {
    ISAPI_SoapServerFactory::instance()->shutdown();
    return TRUE;
}

/* forwards the real work to soap_serve */
static DWORD serve(
    const mod_gsoap_interface *pInterface, 
    ISAPI_HttpRequest& req,
    HttpResponse res,
    isapistream& is) 
{
    assert(NULL != pInterface && pInterface->linked());

    struct soap soap;

    (*pInterface->fsoap_init)(&soap);
    SoapTransaction trans(&soap);
    if (NULL != pInterface->fsoap_register_plugin_arg) {
        (*pInterface->fsoap_register_plugin_arg)(&soap, mod_gsoap_plugin, (void *)&trans);
    }
    //soap.user = &trans;
    trans._interface = pInterface;
    trans._istream = &is;
    trans._request = &req;
    trans._response = &res;

    trans.BuildHeaders();

#ifdef WITH_ZLIB
    //    always allow gzip in -- but only allow it out if the client can handle it
    soap_set_imode(&soap, SOAP_ENC_ZLIB);     
    string str = req.getContentHeaders()["Accept-Encoding"];
    if (strstr(str.c_str(), "gzip"))
    {    
        soap_set_omode(&soap, SOAP_ENC_ZLIB);         
        http_post_header(&soap, "Content-Encoding", "gzip" );    
    }
#endif

    // set callback functions:
    soap.frecv = frecv;
    soap.fsend = fsend;
    soap.fresponse = http_response;
    soap.fposthdr = http_post_header;

    (*pInterface->fsoap_serve)(&soap);
    if (NULL != pInterface->fsoap_delete) {
        (*pInterface->fsoap_delete)(&soap, NULL);
    }
    (*pInterface->fsoap_end)(&soap);
    (*pInterface->fsoap_done)(&soap);

    return trans._dwReturnCode;
}

static void SendErrorMessage(isapistream& is, const char *pszError) {
    is << "<html><title>gsoap error message</title><body>" << pszError << "<p>";
    is << "see <a href=\"http://www.genivia.com/doc/isapi/html/index.html\">gSOAP ISAPI module</a> documentation.";
    is << "</body><html>";
}

/** Parse the request, load dll if not already loaded, let it create the response and return the result.
  */
DWORD ISAPI_Server::HttpExtensionProc(EXTENSION_CONTROL_BLOCK *pECB) {
    DWORD dwRes = HSE_STATUS_SUCCESS;
    ISAPI_HttpRequest req(pECB);
    HttpResponse res;

    isapistream is(pECB);
    
    string strQuery = req.getQueryString();
    string strDll = strQuery;
    string::size_type pos = strQuery.find('&');
    if (string::npos != pos) {
        strDll = strQuery.substr(0, pos);
        strQuery = strQuery.substr(pos + 1);
    }
    if (!strDll.empty()) {
        strDll += ".dll";
        strDll = MakeDllName(pECB, strDll.c_str()); // due to security reasons we only allow loading of dlls from the local directory (which must not be writable!).
        const mod_gsoap_interface *pInterface = ISAPI_SoapServerFactory::instance()->getInterface(strDll.c_str());
        if (NULL != pInterface) {
            if (HttpRequest::POST == req.getMethod()) {
                dwRes = serve(pInterface, req, res, is);
            } else {
                SendErrorMessage(is, "You must use a POST request to get answer from gsoap !");
            }
        } else {
            is << "could not create server '" << strDll << "'. Check your request if it is correct. "
                << ISAPI_SoapServerFactory::instance()->getLastError();
        }
    } else {
        SendErrorMessage(is, "No gsoap server dll specified in request. Please add the name of your soap server dll, e.g. http://localhost/gsoap/mod_gsoap?calc");
    }
    is.flush();
    return dwRes;
}

/** Given a dllname make a fully qualified path to the dll in the current isapi directory 
    @param pECB the Extension Control Block of the Request
    @param pszDllName the unqualified dllname
*/
static string MakeDllName(EXTENSION_CONTROL_BLOCK *pECB, const char *pszDllName) {
    string strPath;
    char szPath[_MAX_PATH + 1];
    ZeroMemory(szPath, sizeof szPath);
    DWORD dwBufferSize = sizeof szPath;
    pECB->GetServerVariable(pECB->ConnID, "APPL_PHYSICAL_PATH", szPath, &dwBufferSize);
    strPath = szPath;
    strPath += pszDllName;
    return strPath;
}
