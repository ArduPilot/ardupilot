/** Interface for the HttpMessage and HttpContext classes.
  * @file HttpContext.h
  * @author Christian Aberger 
  * Copyright (C) 2001 Genivia Inc and WebWare
  */
#ifndef HttpContext_H_F3A8FE10_2C12_11d5_8BEB_01A0CCD4FF57
#define HttpContext_H_F3A8FE10_2C12_11d5_8BEB_01A0CCD4FF57

//#include <utilily>
#include <cstring>
#include "opsysadjust.h"
#include <map>
#include <string>
#include <iostream>
#include "casecmpless.h"

/** HTTP message, either a request or a response.
  */
class HttpMessage {
public:
    HttpMessage(); ///< constructor.

    /** list of "Header: value" entries in an http request or response
      */
    typedef std::map<std::string, std::string, casecmpless> ContentHeaders;
    const ContentHeaders& getContentHeaders() const; ///< return the content headers of the message.
    ContentHeaders& getContentHeaders(); ///< return the content headers of the message.
protected:
    ContentHeaders m_ContentHeaders; ///< the content headers.
};
/** a <a href="http://www.ietf.org/rfc/rfc2068.txt?number=2068">HTTP</a> response.
  */
class HttpResponse : public HttpMessage {
public:
    HttpResponse() {}
    virtual ~HttpResponse() {}
};
/** a <a href="http://www.ietf.org/rfc/rfc2068.txt?number=2068">HTTP</a> request.
  * No platform dependent types are used here, unchanged also for <a href="http://www.apache.org">Apache-Server</a>
  * Platform dependency is in subclasses only.
  */
class HttpRequest : public HttpMessage {
public:
    /** 2 methods in HTTP:either GET or POST. */
    typedef enum Method {GET = 0, POST = 1};
    HttpRequest(); ///< constructor.
    virtual ~HttpRequest(); ///< destructor.

    virtual Method getMethod() const; ///< @return the method used in the request
    virtual const char *getQueryString(); ///< @return the url requested. 
    virtual const char *getBody(); ///< @return the body of the request as a string (guaranteed to be 0 terminated).
    virtual void *getData(); ///< @return the pointer to the start of the body data.
    virtual unsigned long getContentLength() const; ///< return the lenght of the body of the request.
    virtual bool operator<<(std::istream&);
public:
    std::string _querystring;
    char  *_body;
    unsigned long _contentlength;
    Method _method;
};
//inlines
inline const HttpMessage::ContentHeaders& HttpMessage::getContentHeaders() const {return m_ContentHeaders;}
inline HttpMessage::ContentHeaders& HttpMessage::getContentHeaders() {return m_ContentHeaders;}

#endif // HttpContext_H_F3A8FE10_2C12_11d5_8BEB_01A0CCD4FF57
