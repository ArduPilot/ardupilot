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
extern "C" int interopB(const char *site);

struct Namespace namespacesB[] =
{
  {"SOAP-ENV", "http://schemas.xmlsoap.org/soap/envelope/"},
  {"SOAP-ENC", "http://schemas.xmlsoap.org/soap/encoding/"},
  //{"SOAP-ENV", "http://www.w3.org/2002/06/soap-envelope"},	// SOAP 1.2
  //{"SOAP-ENC", "http://www.w3.org/2002/06/soap-encoding"},	// SOAP 1.2
  {"xsi", "http://www.w3.org/2001/XMLSchema-instance", "http://www.w3.org/*/XMLSchema-instance"},
  {"xsd", "http://www.w3.org/2001/XMLSchema", "http://www.w3.org/*/XMLSchema"},
  {"ns", "http://soapinterop.org/"},
  {"s", "http://soapinterop.org/xsd"},
  {NULL, NULL}
};

int interopB(const char *url)
{
  struct soap *soap;
  char *site=(char*)url;
//  char* site ="http://websrv.cs.fsu.edu/~engelen/interop2B.cgi";
  char* action = "http://soapinterop.org/";
  
  bool ok=true;
  int i, g;
  xsd__string s = "Hello World! <>&";
  xsd__int n = 2147483647;
  xsd__float f = 3.40282e+38;
  struct ArrayOfstring a;
  struct ArrayOfstring2D aai;
  struct ArrayOfstring2D aao;

  struct s__SOAPStruct sti;
  struct ns__echoStructAsSimpleTypesResponse Ro;
  struct ns__echoSimpleTypesAsStructResponse Rsto;
  struct s__SOAPStructStruct ssti;
  struct ns__echoNestedStructResponse Rssto;
  struct s__SOAPArrayStruct sati;
  struct ns__echoNestedArrayResponse Rsato;

  displayText("running test B on");
  displayText((char*)url);
  soap = soap_new();
  soap->namespaces = (struct Namespace *)namespacesB;

//  FILE *fd = fopen("interop2Bfaults.html", "a");

//  soap_init(&soap);

  soap->send_timeout = 30;
  soap->recv_timeout = 30;

  sti.varString = "Hello";
  sti.varInt = &n;
  sti.varFloat = &f;

  ssti.varString = "SOAP!";
  ssti.varInt = &n;
  ssti.varFloat = &f;
  ssti.varStruct = &sti;

  a.__size = 5;
  a.__offset = 2;
  a.__ptr = (char**)malloc(a.__size*sizeof(char*));
  a.__ptr[0] = "Interop";
  a.__ptr[1] = "Test";
  a.__ptr[2] = "Round";
  a.__ptr[3] = "2";
  a.__ptr[4] = a.__ptr[1];

  aai.__size[0] = 2;
  aai.__size[1] = 3;
  aai.__offset[0] = 0;
  aai.__offset[1] = 0;
  aai.__ptr = (char**)malloc(aai.__size[0]*aai.__size[1]*sizeof(char*));
  aai.__ptr[0] = "Interop Test";
  aai.__ptr[1] = NULL;
  aai.__ptr[2] = "Round";
  aai.__ptr[3] = "2";
  aai.__ptr[4] = a.__ptr[1];
  aai.__ptr[5] = NULL;

  sati.varString = a.__ptr[0];
  sati.varInt = &n;
  sati.varFloat = &f;
  sati.varArray = a;
   
  if (soap_call_ns__echoStructAsSimpleTypes(soap, site, "http://soapinterop.org/", sti, Ro))
  { 
    displayText("echoStructAsSimpleTypes failed");
    ok=false;
  }  else if (!Ro._outputString || strcmp(sti.varString, Ro._outputString) || !Ro._outputInteger || *sti.varInt != *Ro._outputInteger
  || !Ro._outputFloat || *sti.varFloat != *Ro._outputFloat)
  {
  	displayText("echoStructAsSimpleTypes mismatched");
    ok=false;
  }
  else
  	displayText("echoStructAsSimpleTypes passed");

  if (soap_call_ns__echoSimpleTypesAsStruct(soap, site, "http://soapinterop.org/", s, &n, &f, Rsto))
  { 
    displayText("echoSimpleTypesAsStruct failed");
    ok=false;
  }
  else if (!Rsto._return.varString || strcmp(s, Rsto._return.varString) || !Rsto._return.varInt || n != *Rsto._return.varInt || !Rsto._return.varFloat || f != *Rsto._return.varFloat)
  {
  	displayText("echoSimpleTypesAsStruct mismatched");
    ok=false;
  }
  else
  	displayText("echoSimpleTypesAsStruct passed");

  if (soap_call_ns__echo2DStringArray(soap,site, "http://soapinterop.org/", aai, aao))
  { 
    displayText("echo2DStringArray failed");
    ok=false;
  }
  else
  { g = 0;
    if (aai.__size[0] != aao.__size[0] || aai.__size[1] != aao.__size[1] || !aao.__ptr)
      g = 1;
    else
      for (i = 0; i < aai.__size[0]*aai.__size[1]; i++)
        if (aai.__ptr[i] && (!aao.__ptr[i] || strcmp(aai.__ptr[i], aao.__ptr[i])))
	  g = 1;
    if (g)
    {
    	displayText("echo2DStringArray mismatched");
    ok=false;
    }
    else
	  	displayText("echo2DStringArray passed");
  }

  if (soap_call_ns__echoNestedStruct(soap, site, "http://soapinterop.org/", ssti, Rssto))
  { 
    displayText("echoNestedStruct failed");
    ok=false;
  }
  else if (!Rssto._return.varString || strcmp(ssti.varString, Rssto._return.varString) ||
  !Rssto._return.varInt || *ssti.varInt != *Rssto._return.varInt ||
  !Rssto._return.varFloat || *ssti.varFloat != *Rssto._return.varFloat ||
  !Rssto._return.varStruct || 
  !Rssto._return.varStruct->varString || strcmp(ssti.varStruct->varString, Rssto._return.varStruct->varString) ||
  !Rssto._return.varStruct->varInt || *ssti.varStruct->varInt != *Rssto._return.varStruct->varInt || 
  !Rssto._return.varStruct->varFloat || *ssti.varStruct->varFloat != *Rssto._return.varStruct->varFloat)
    {
    	displayText("echoNestedStruct mismatched");
    ok=false;
  }
  else
  	displayText("echoNestedStruct passed");

  if (soap_call_ns__echoNestedArray(soap, site, "http://soapinterop.org/", sati, Rsato))
  { 
    displayText("echoNestedArray failed");
    ok=false;
  }
  else if (!Rsato._return.varString || strcmp(sati.varString, Rsato._return.varString) ||
  !Rsato._return.varInt || *sati.varInt != *Rsato._return.varInt ||
  !Rsato._return.varFloat || *sati.varFloat != *Rsato._return.varFloat ||
  sati.varArray.__size+sati.varArray.__offset != Rsato._return.varArray.__size+Rsato._return.varArray.__offset)
    {
    	displayText("echoNestedArray mismatched");
    ok=false;
  }
  else
  { g = 0;
    for (i = sati.varArray.__offset; i < sati.varArray.__size+sati.varArray.__offset; i++)
      if (!Rsato._return.varArray.__ptr[i-Rsato._return.varArray.__offset] ||
          strcmp(sati.varArray.__ptr[i-sati.varArray.__offset], Rsato._return.varArray.__ptr[i-Rsato._return.varArray.__offset]))
          g = 1;
    if (g)
    {
    	displayText("echoNestedArray mismatched");
	    ok=false;
    }
    else
	  	displayText("echoNestedArray passed");
  }

  if (ok)
  	displayText("All Passed");
  	else
	  displayText("FAILURES");

  return 0;
}


