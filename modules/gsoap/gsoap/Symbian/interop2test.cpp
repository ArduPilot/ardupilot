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
// gSOAP v2 Interop test round 2 base
//#include "interoptA.h"
#include "soapH.h" 

extern "C" void displayText(char *text);
extern "C" int interopA(const char *url);

struct Namespace namespacesA[] =
{ {"SOAP-ENV", "http://schemas.xmlsoap.org/soap/envelope/"},
  {"SOAP-ENC", "http://schemas.xmlsoap.org/soap/encoding/"},
  {"xsi", "http://www.w3.org/2001/XMLSchema-instance", "http://www.w3.org/*/XMLSchema-instance"},
  {"xsd", "http://www.w3.org/2001/XMLSchema", "http://www.w3.org/*/XMLSchema"},
  {"ns", "http://soapinterop.org/"},
  {"s", "http://soapinterop.org/xsd"},
  {"a", "http://xml.apache.org/xml-soap"},
  {"h", "http://soapinterop.org/echoheader/"},
  {NULL, NULL}
};

int interopA(const char *url)
{ 
  struct soap *soap;
  int i, g;
  xsd__string so, si = "Hello World! <>&";
  struct ArrayOfstring Asi, Aso;
  xsd__int no, ni = 1234567890;
  xsd__int n = 2147483647;
  struct ArrayOfint Ani, Ano;
//  xsd__float f1 = 3.40282e+38;
  xsd__float f1 = 123.5678;
  xsd__float f2 = 3.14;
  xsd__float fo, fi = 123.456;
//  xsd__float fo, fi = 3e2;
//#ifdef SYMBIAN
//  const struct soap_double_nan { unsigned int n1, n2; } soap_double_nan;
//#endif
  xsd__float nan = FLT_NAN, inf = FLT_PINFTY, ninf = FLT_NINFTY;
  struct ArrayOffloat Afi, Afo;
  struct s__SOAPStruct sti, *p;
  struct ns__echoStructResponse sto;
  struct ArrayOfSOAPStruct Asti, Asto;
  struct ns__echoVoidResponse Rv;
  struct xsd__base64Binary b64i, b64o;
  xsd__dateTime dto, dti = "1967-12-29T01:02:03";
  struct xsd__hexBinary hbi, hbo;
  xsd__decimal Do, Di = "1234567890.123456789";
  xsd__boolean bo, bi = true;
  //struct a__Map mi, mo;
  //struct ArrayOfMap Ami, Amo;
  // char buff[100];
  displayText("running test A on");
  displayText((char*)url);
  soap = soap_new();
  soap->namespaces = (struct Namespace *)namespacesA;

//  soap.send_timeout = 30;
//  soap.recv_timeout = 30;


  Asi.__size = 8;
  Asi.__offset = 0;
  Asi.__ptr = (xsd__string*)malloc(Asi.__size*sizeof(xsd__string));
  Asi.__ptr[0] = NULL;
  Asi.__ptr[1] = " Hello\tWorld";
  Asi.__ptr[2] = NULL;
  Asi.__ptr[3] = "! ";
  Asi.__ptr[4] = NULL;
  Asi.__ptr[5] = si;
  Asi.__ptr[6] = NULL;
  Asi.__ptr[7] = si;

  Ani.__size = 0;
  Ani.__offset = 0;
  Ani.__ptr = NULL; // (xsd__int*)malloc(Ani.__size*sizeof(xsd__int));

  Afi.__size = 5;
  Afi.__offset = 0;
  Afi.__ptr = (xsd__float**)malloc(Afi.__size*sizeof(xsd__float*));
  Afi.__ptr[0] = &f1;
  Afi.__ptr[1] = &nan; // FLT_NAN;
  Afi.__ptr[2] = &inf; // FLT_PINFTY;
  Afi.__ptr[3] = &ninf; // FLT_NINFTY;
  Afi.__ptr[4] = &f2;

  sti.varString = "Hello";
  sti.varInt = &n;
  sti.varFloat = &f1;

  Asti.__size = 3;
  Asti.__offset = 2;
  Asti.__ptr = (struct s__SOAPStruct**)malloc((Asti.__size+1)*sizeof(struct s__SOAPStruct*));
  p = (struct s__SOAPStruct*)malloc(Asti.__size*sizeof(struct s__SOAPStruct));
  Asti.__ptr[0] = p;
  Asti.__ptr[1] = p+1;
  Asti.__ptr[2] = p+2;
  Asti.__ptr[3] = p;
  Asti.__ptr[0]->varString = "Hello";
  Asti.__ptr[0]->varInt = &n;
  Asti.__ptr[0]->varFloat = &f1;
  Asti.__ptr[1]->varString = "World";
  Asti.__ptr[1]->varInt = &n;
  Asti.__ptr[1]->varFloat = &f2;
  Asti.__ptr[2]->varString = "!";
  Asti.__ptr[2]->varInt = &n;
  Asti.__ptr[2]->varFloat = &f2;

//  b64i.__ptr = (unsigned char*)"This is an example Base64 encoded string";
//  b64i.__size = strlen((char*)b64i.__ptr)+1;
  unsigned char b64data[4]={0x80, 0x81, 0x82, 0x83};
  b64i.__ptr = b64data;
  b64i.__size = 4;

  hbi.__ptr = (unsigned char*)"This is an example HexBinary encoded string";
  hbi.__size = strlen((char*)hbi.__ptr)+1;
/*
  mi.__size = 2;
  mi.__ptr = (struct _item*)malloc(mi.__size*sizeof(struct _item));
  mi.__ptr[0].key = new xsd__string_("hello");
  mi.__ptr[0].value = new xsd__string_("world");
  mi.__ptr[1].key = new xsd__int_(2);
  mi.__ptr[1].value = new xsd__boolean_(true);

  Ami.__size = 2;
  Ami.__ptr = (struct a__Map**)malloc(Ami.__size*sizeof(struct a__Map*));
  Ami.__ptr[0] = &mi;
  Ami.__ptr[1] = &mi; */
  char *site=(char*)url;
//    char* site ="http://websrv.cs.fsu.edu/~engelen/interop2.cgi";
//  char* site = "http://nagoya.apache.org:5049/axis/services/echo ";
  char* action = "http://soapinterop.org/";
  
  bool ok=true;
  
  if (soap_call_ns__echoString(soap, site, action, si, so))
  { 
    displayText("echoString fail");
    ok=false;
  }
  else if (!so || strcmp(si, so))
  { 
    ok=false;
    displayText("echoString mismatched");
  }
  else
    displayText("echoString pass");
 

  if (soap_call_ns__echoInteger(soap, site, "http://soapinterop.org/", ni, no))
  { 
    ok=false;
    displayText("echoInteger fail");
  }
  else if (ni != no)
  {  
    ok=false;
    displayText("echoInteger mismatched");
  }
  else
    displayText("echoInteger pass");

  if (soap_call_ns__echoFloat(soap, site, "http://soapinterop.org/", fi, fo))
  { 
    ok=false;
    displayText("echoFloat fail");
  }
  else if (fi != fo)
  {  
    ok=false;
    displayText("echoFloat mismatched");
  }
  else
    displayText("echoFloat pass");

  if (soap_call_ns__echoStruct(soap, site, "http://soapinterop.org/", sti, sto))
  { 
    ok=false;
  displayText("echoStruct fail");
  }
  else if (!sto._return.varString || strcmp(sti.varString, sto._return.varString)
  	 || !sto._return.varInt || *sti.varInt != *sto._return.varInt 
  	 || !sto._return.varFloat || *sti.varFloat != *sto._return.varFloat)
  { 
    ok=false;
	  displayText("echoStruct mismatch");
  }
  else 
    displayText("echoStruct pass");

  if (soap_call_ns__echoStringArray(soap, site, "http://soapinterop.org/", Asi, Aso))
  {
     soap_set_fault(soap); 
    soap_faultdetail(soap);
    ok=false;
  displayText("echoStringArray fail");
  }
  else
  { g = 0;
    if (Asi.__size != Aso.__size)
      g = 1;
    else
      for (i = 0; i < Asi.__size; i++)
        if (Asi.__ptr[i] && Aso.__ptr[i] && strcmp(Asi.__ptr[i], Aso.__ptr[i]))
          g = 1;
        else if (!Asi.__ptr[i])
	  ;
        else if (Asi.__ptr[i] && !Aso.__ptr[i])
	  g = 1;
    if (g)
    {
    ok=false;
    displayText("echoStringArray mismatch"); 
    }
    else
     displayText("echoStringArray pass");
  }

  if (soap_call_ns__echoIntegerArray(soap, site, "http://soapinterop.org/", Ani, Ano))
  { displayText("echoIntegerArray fail");
    ok=false;
  }
  else
  { g = 0;
    if (Ani.__size != Ano.__size)
      g = 1;
    else
      for (i = 0; i < Ani.__size; i++)
        if (Ani.__ptr[i] && (!Ano.__ptr[i] || *Ani.__ptr[i] != *Ano.__ptr[i]))
          g = 1;
    if (g)
    { displayText("echoIntegerArray mismatch");
    ok=false;
    }
    else
      displayText("echoIntegerArray pass");
  }

  if (soap_call_ns__echoFloatArray(soap, site, "http://soapinterop.org/", Afi, Afo))
  { displayText("echoFloatArray fail");
    ok=false;
  }
  else
  { g = 0;
    if (Afi.__size != Afo.__size)
      g = 1;
    else
      for (i = 0; i < Afi.__size; i++)
        if (Afi.__ptr[i] && Afo.__ptr[i] && soap_isnan(*Afi.__ptr[i]) && soap_isnan(*Afo.__ptr[i]))
          ;
        else if (Afi.__ptr[i] && (!Afo.__ptr[i] || *Afi.__ptr[i] != *Afo.__ptr[i]))
          g = 1;
    if (g)
    { displayText("echoFloatArray mismatch");
    ok=false;
    }
    else
     displayText("echoFloatArray pass");
  }

  if (soap_call_ns__echoStructArray(soap, site, "http://soapinterop.org/", Asti, Asto))
  { displayText("echoStructArray fail");
    ok=false;
  }
  else
  { g = 0;
    if (Asti.__size+Asti.__offset != Asto.__size+Asto.__offset)
      g = 1;
    else
      for (i = Asti.__offset; i < Asti.__size+Asti.__offset; i++)
        if (!Asto.__ptr[i-Asto.__offset] ||
	!Asto.__ptr[i-Asto.__offset]->varString ||
	strcmp(Asti.__ptr[i-Asti.__offset]->varString, Asto.__ptr[i-Asto.__offset]->varString) ||
	!Asto.__ptr[i-Asto.__offset]->varInt ||
	*Asti.__ptr[i-Asti.__offset]->varInt != *Asto.__ptr[i-Asto.__offset]->varInt ||
	!Asto.__ptr[i-Asto.__offset]->varFloat ||
	*Asti.__ptr[i-Asti.__offset]->varFloat != *Asto.__ptr[i-Asto.__offset]->varFloat)
          g = 1;
    if (g)
    { displayText("echoStructArray mismatch");
    ok=false;
    }
    else
      displayText("echoStructArray pass");
  }

  if (soap_call_ns__echoVoid(soap, site, "http://soapinterop.org/", Rv))
  { displayText("echoVoid fail");
    ok=false;
  }
  else
    displayText("echoVoid pass");
  

  if (soap_call_ns__echoBase64(soap, site, "http://soapinterop.org/", b64i, b64o))
  { displayText("echoBase64 fail");
    ok=false;
  }
  else if ((b64i.__size+2)/3 != (b64o.__size+2)/3 || strncmp((char*)b64i.__ptr, (char*)b64o.__ptr,b64i.__size))
  { 
  displayText("echoBase64 mismatch");
    ok=false;
  }
  else
   displayText("echoBase64 pass");

  if (soap_call_ns__echoDate(soap, site, "http://soapinterop.org/", dti, dto))
  { 
  displayText("echoDate fail");
    ok=false;
  }
  else if (!dto || strncmp(dti, dto, 19))
  { 
  displayText("echoDate mismatch");
    ok=false;
  }
  else
    displayText("echoDate pass");
 

  if (soap_call_ns__echoHexBinary(soap, site, "http://soapinterop.org/", hbi, hbo))
  { 
    ok=false;
  displayText("echoHexBinary fail");
  }
  else if (hbi.__size != hbo.__size || strcmp((char*)hbi.__ptr, (char*)hbo.__ptr))
  { 
    ok=false;
  displayText("echoHexBinary mismatch");
  }
  else
    displayText("echoHexBinary pass");
 

  if (soap_call_ns__echoDecimal(soap, site, "http://soapinterop.org/", Di, Do))
  { 
    ok=false;
  displayText("echoDecimal pass");
  }
  else if (strcmp(Di, Do))
  { 
    ok=false;
  displayText("echoDecimal mismatch");
  }
  else
    displayText("echoDecimal pass");
  

  if (soap_call_ns__echoBoolean(soap, site, "http://soapinterop.org/", bi, bo))
  { 
    ok=false;
  displayText("echoBoolean fail");
  }
  else if (bi != bo)
  { 
    ok=false;
  displayText("echoBoolean mismatch");
  }
  else
    displayText("echoBoolean pass");

  if (ok)
    displayText("ALL PASS");
  else
    displayText("FAILURES");
  return 0;
  
end:
  return 1;
}

