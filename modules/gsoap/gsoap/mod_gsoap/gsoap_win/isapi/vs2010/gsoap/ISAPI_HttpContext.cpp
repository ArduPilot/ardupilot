/** Implementation of the ISAPI_HttpRequest and related classes
  * @file ISAPI_HttpContext.cpp
  * @author Christian Aberger 
  * Copyright (C) 2001 Genivia Inc and WebWare
  */
#include <cassert>
#include <algorithm>
#include "ISAPI_HttpContext.h"
using namespace std;

static bool AddHeader(
    ISAPI_HttpRequest& req, 
    EXTENSION_CONTROL_BLOCK& ecb, 
    const char *pszHeaderName,
    LPCTSTR pszVariableName);

ISAPI_HttpRequest::ISAPI_HttpRequest(EXTENSION_CONTROL_BLOCK *pECB) 
: m_bHeadersParsed(false)
, m_pECB(pECB)
{
    if (NULL != pECB) {
        ParseHeaders(*pECB);
    }
}
ISAPI_HttpRequest::~ISAPI_HttpRequest() {
}
EXTENSION_CONTROL_BLOCK *ISAPI_HttpRequest::ECB() const {
    return m_pECB;
}
void ISAPI_HttpRequest::ParseHeaders(EXTENSION_CONTROL_BLOCK& ecb) {
    _contentlength = ecb.cbTotalBytes;
    AddHeader(ecb, "CONTENT-LENGTH", "CONTENT_LENGTH");
    AddHeader(ecb, "CONTENT-TYPE", "CONTENT_TYPE");
    AddHeader(ecb, "PATH-INFO", "PATH_INFO");
    string strValue;
    ReadHeader(ecb, "REQUEST_METHOD", strValue);
    if (0 == strcasecmp(strValue.c_str(), "GET")) {
        _method = GET;
    } else if (0 == strcasecmp(strValue.c_str(), "POST")) {
        _method = POST;
    }
    _querystring = ecb.lpszQueryString;
    ReadCustomHeaders(ecb);
    m_bHeadersParsed = true;
}
/** add a ECB header variable to our content headers collection.
    @param ecb the extension control block of the current IIS request
    @param pszHeaderName the HTTP - standard name of the content header.
    @param pszVariableName the name invented by Micro$oft for the same thing.
  */
bool ISAPI_HttpRequest::AddHeader(EXTENSION_CONTROL_BLOCK& ecb, const char *pszHeaderName, LPCTSTR pszVariableName) {
    assert(NULL != pszVariableName);

    string strValue; 
    bool bRet = ReadHeader(ecb, pszVariableName, strValue);
    if (bRet) {
        ::OutputDebugString((pszHeaderName + string(": ") + strValue + "\r\n").c_str());
        m_ContentHeaders[pszHeaderName] = strValue;
    }
    return bRet;
}
/** Read a variable from the control block.
    @param ecb the Extension Control Block.
    @param pszVariableName the name of the variable to read
    @strHeaderValue (output) the value of the variable
    @return true if the header was found.
  */ 
bool ISAPI_HttpRequest::ReadHeader(EXTENSION_CONTROL_BLOCK& ecb, LPCTSTR pszVariableName, string& strHeaderValue) {
    bool bRet = true;
    char szVarName[128]; // :( GetServerVariable uses non-const name parameter, so we make a copy to be safe.
    strncpy_s(szVarName, sizeof(szVarName), pszVariableName, _TRUNCATE);
    char szBuffer[2048];
    DWORD dwBufferSize = sizeof szBuffer;
    BOOL bGet = ecb.GetServerVariable(ecb.ConnID, szVarName, szBuffer, &dwBufferSize);
    if (bGet) {
        strHeaderValue = szBuffer;
    } else {
        bRet = false;
        strHeaderValue.erase();
    }
    return bRet;
}
/** read all HTTP headers that were not already parsed into one of the ISAPI - ECB variables.
  */
void ISAPI_HttpRequest::ReadCustomHeaders(EXTENSION_CONTROL_BLOCK& ecb) {
    const char szHTTP_[] = "HTTP_";
    char szBuffer[4096];
    DWORD dwBufferSize = sizeof szBuffer;
    BOOL bGet = ecb.GetServerVariable(ecb.ConnID, "ALL_HTTP", szBuffer, &dwBufferSize);
    if (bGet) {
        // Find lines, split key/data pair and write them as output
        LPTSTR pOpts = NULL;
        LPTSTR pEnd = NULL;
        for (LPTSTR pChar = szBuffer; '\0' != *pChar;) {
            if (*pChar == '\r' || *pChar == '\n') {
                pChar++;
                continue;
            }
            pOpts = strchr(pChar, ':');// findseparator
            if (pOpts && *pOpts) {
                pEnd = pOpts;
                while (*pEnd && *pEnd != '\r' && *pEnd != '\n') {
                    pEnd++;
                }
                *pOpts = '\0'; // split the strings
                *pEnd = '\0';
                string strName = pChar;
                if (0 == strncmp(strName.c_str(), szHTTP_, strlen(szHTTP_))) {
                    strName = strName.substr(strlen(szHTTP_));
                }
                replace(strName.begin(), strName.end(), '_', '-');
                ::OutputDebugString((strName + ":" + (pOpts + 1) + "\r\n").c_str());
                // don't let gsoap see TRANSFER ENCODING, since this is handled by IIS
                if ("TRANSFER-ENCODING" != strName)
                    m_ContentHeaders[strName] = pOpts + 1;
                pChar = pEnd + 1;
            }
        }
    }
}
