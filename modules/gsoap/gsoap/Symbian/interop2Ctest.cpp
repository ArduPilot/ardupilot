//===================================================================================
//
//    (C) COPYRIGHT International Business Machines Corp., 2002 All Rights Reserved
//    Licensed Materials - Property of IBM
//    US Government Users Restricted Rights - Use, duplication or
//    disclosure restricted by GSA ADP Schedule Contract with IBM Corp.
//
//    IBM DISCLAIMS ALL WARRANTIES WITH REGARD TO THIS SOFTWARE, INCLUDING
//    ALL IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
//    PURPOSE. IN NO EVENT SHALL IBM BE LIABLE FOR ANY SPECIAL, INDIRECT OR
//    CONSEQUENTIAL DAMAGES OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF
//    USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT, NEGLIGENCE OR
//    OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE
//    OR PERFORMANCE OF THIS SOFTWARE.
//
//    The program may be used, executed, copied, modified, and distributed
//    without royalty for the purpose of developing, using, marketing, or distributing.
// 
//=======================================================================================
// gSOAP v2 Interop B test round 2
#include "soapH.h"
extern "C" void displayText(char *text);
extern "C" int interopC(const char *site);


struct Namespace namespacesC[] =
{ {"SOAP-ENV", "http://schemas.xmlsoap.org/soap/envelope/"},
  {"SOAP-ENC", "http://schemas.xmlsoap.org/soap/encoding/"},
  {"xsi", "http://www.w3.org/2001/XMLSchema-instance", "http://www.w3.org/*/XMLSchema-instance"},
  {"xsd", "http://www.w3.org/2001/XMLSchema", "http://www.w3.org/*/XMLSchema"},
  {"m", "http://soapinterop.org/"},
  {"s", "http://soapinterop.org/xsd"},
  {"a", "http://xml.apache.org/xml-soap"},
  {"h", "http://soapinterop.org/echoheader/"},
  {NULL, NULL}
};
int interopC(const char *url)
{ struct soap *soap;
  char *site=(char*)url;
  bool ok=true;
  xsd__int n = 42;
  float f  = 3.40282e+38;
  struct SOAP_ENV__Header h;
  struct m__echoVoidResponse r;
  struct s__SOAPStruct s;

  h.h__echoMeStringRequest = NULL;
  h.h__echoMeStringRequest_ = NULL;
  h.h__echoMeStringResponse = NULL;
  h.h__echoMeStructRequest = NULL;
  h.h__echoMeStructRequest_ = NULL;
  h.h__echoMeStructResponse = NULL;
  h.h__someUnknownRequest = NULL;
  h.h__someUnknownRequest_ = NULL;
  s.varString = "hello world";
  s.varInt = &n;
  s.varFloat = &f;


  displayText("running test C on");
  displayText((char*)url);
  soap = soap_new();
  soap->namespaces = (struct Namespace *)namespacesC;

  soap->send_timeout = 300;
  soap->recv_timeout = 300;

  // request from client (with mustUnderstand="0", recipient is target)
  h.h__echoMeStringRequest = "hello world";
  soap->header = &h;
  soap->actor = "http://schemas.xmlsoap.org/soap/actor/next";
  if (soap_call_m__echoVoid(soap, site, "http://soapinterop.org/", r))
  { 
    displayText("echo1 failed");
    ok=false;
  }
  else
    displayText("echo1 passed");

  // request from client (with mustUnderstand="0", recipient is not target)
  soap->header = &h;
  soap->actor = "http://some/other/actor";
  if (soap_call_m__echoVoid(soap, site, "http://soapinterop.org/", r))
  { 
    displayText("echo2 failed");
    ok=false;
  }
  else
    displayText("echo2 passed");

  // request from client (with mustUnderstand="1", recipient is target)
  h.h__echoMeStringRequest = NULL;
  h.h__echoMeStructRequest_ = &s;
  soap->header = &h;
  soap->actor = "http://schemas.xmlsoap.org/soap/actor/next";
  if (soap_call_m__echoVoid(soap, site, "http://soapinterop.org/", r))
  { 
    displayText("echo3 failed");
    ok=false;
  }
  else
    displayText("echo3 passed");

  // request from client (with mustUnderstand="1", recipient is not target)
  soap->header = &h;
  soap->actor = "http://some/other/actor";
  if (soap_call_m__echoVoid(soap,site, "http://soapinterop.org/", r))
  { 
    displayText("echo4 failed");
    ok=false;
  }
  else if (soap->header && soap->header->h__echoMeStructResponse)
  { 
    displayText("echo4 should be no response header");
    ok=false;
   }
  else
    displayText("echo4 passed");

  // Unknown header element: request from client (with mustUnderstand="0", recipient is target)
  h.h__echoMeStructRequest_ = NULL;
  h.h__someUnknownRequest = "XYZ";
  soap->header = &h;
  soap->actor = "http://schemas.xmlsoap.org/soap/actor/next";
  if (soap_call_m__echoVoid(soap, site, "http://soapinterop.org/", r))
  { 
    displayText("echo5 failed");
    ok=false;
  }
  else
    displayText("echo5 passed");

  // Unknown header element: request from client (with mustUnderstand="1", recipient is target)
  h.h__someUnknownRequest = NULL;
  h.h__someUnknownRequest_ = "XYZ";
  soap->header = &h;
  soap->actor = "http://schemas.xmlsoap.org/soap/actor/next";
  if (soap_call_m__echoVoid(soap, site, "http://soapinterop.org/", r) != SOAP_MUSTUNDERSTAND)
  { 
    displayText("echo6 failed");
    ok=false;
  }
  else
    displayText("echo6 passed");

  // Unknown header element: request from client (with mustUnderstand="0", recipient is not target)
  h.h__someUnknownRequest_ = NULL;
  h.h__someUnknownRequest = "XYZ";
  soap->header = &h;
  soap->actor = "http://some/other/actor";
  if (soap_call_m__echoVoid(soap, site, "http://soapinterop.org/", r))
  { 
    displayText("echo7 failed");
    ok=false;
  }
  else
    displayText("echo7 passed");

  // Unknown header element: request from client (with mustUnderstand="1", recipient is not target)
  h.h__someUnknownRequest = NULL;
  h.h__someUnknownRequest_ = "XYZ";
  soap->header = &h;
  soap->actor = "http://some/other/actor";
  if (soap_call_m__echoVoid(soap, site, "http://soapinterop.org/", r))
  { 
    displayText("echo8 failed");
    ok=false;
  }
  else
    displayText("echo8 passed");


  if (ok)
  	displayText("All Passed");
  else
	  displayText("FAILURES");

  return 0;
}
