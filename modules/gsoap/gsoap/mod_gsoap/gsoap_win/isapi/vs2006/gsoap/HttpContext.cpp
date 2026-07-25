/** Implementation of the HttpContext class
  * @file HttpContext.cpp
  * @author Christian Aberger 
  * Copyright (C) 2001 Genivia Inc and WebWare
  */ 
#include "HttpContext.h"

HttpMessage::HttpMessage() {
}
HttpRequest::HttpRequest() 
: _method(GET)
, _body(NULL)
, _contentlength(0)
{
}
HttpRequest::~HttpRequest() {
    delete _body;
}
const char *HttpRequest::getQueryString() {
    return _querystring.c_str();
}
const char *HttpRequest::getBody() {
    static const char szEmpty[] = "";
    return NULL == _body ? szEmpty : _body;
}
void *HttpRequest::getData() {
    return _body;
}
HttpRequest::Method HttpRequest::getMethod() const {
    return _method;
}
unsigned long HttpRequest::getContentLength() const {
    return _contentlength;
}
bool HttpRequest::operator<<(std::istream& is) {
    delete _body, _body = NULL;
    bool bRet = true;
    if (0 != _contentlength) {
        _body = new char[_contentlength + 1];
        memset(_body, 0, _contentlength + 1);
        is.read(_body, _contentlength);
        bRet = is.good();
    } else {
        bRet = false;
    }
    return bRet;
}


