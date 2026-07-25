/** Interface for the ISAPI_HttpRequest and related classes
  * @file ISAPI_HttpContext.h
  * @author Christian Aberger 
  * Copyright (C) 2001 Genivia Inc and WebWare
  */
#ifndef ISAPIHTTPCONTEXT_H
#define ISAPIHTTPCONTEXT_H
#include <httpext.h>
#include <string>

#include "HttpContext.h"

/** encapsulate special content header parsing if we are running on a non-standard server.
  */
class ISAPI_HttpRequest : public HttpRequest {
public:
    /** Constructor, parsing also content headers from the control block.
      * @param pECB the ISAPI extension control block passed in by IIS.
      */
    ISAPI_HttpRequest(EXTENSION_CONTROL_BLOCK *pECB = NULL);
    virtual ~ISAPI_HttpRequest(); ///< destructor.
    EXTENSION_CONTROL_BLOCK *ECB() const;
protected:
    void ParseHeaders(EXTENSION_CONTROL_BLOCK& ecb); ///< extract the request headers from the content block
    bool AddHeader(EXTENSION_CONTROL_BLOCK& ecb, const char *pszHeaderName, LPCTSTR pszVariableName);
    bool ReadHeader(EXTENSION_CONTROL_BLOCK& ecb, LPCTSTR pszVariableName, std::string& strHeaderValue);
    void ReadCustomHeaders(EXTENSION_CONTROL_BLOCK& ecb);
protected:
    bool m_bHeadersParsed; ///< have the content headers already been parsed ?
    EXTENSION_CONTROL_BLOCK *m_pECB;
};

#endif //ISAPIHTTPCONTEXT_H
