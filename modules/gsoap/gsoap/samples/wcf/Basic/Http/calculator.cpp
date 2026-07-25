/*
	calculator.cpp

	WCF BasicHttpBinding demo

	See the README.txt

gSOAP XML Web services tools
Copyright (C) 2000-2012, Robert van Engelen, Genivia Inc., All Rights Reserved.
This part of the software is released under one of the following licenses:
GPL.
--------------------------------------------------------------------------------
GPL license.

This program is free software; you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program; if not, write to the Free Software Foundation, Inc., 59 Temple
Place, Suite 330, Boston, MA 02111-1307 USA

Author contact information:
engelen@genivia.com / engelen@acm.org

This program is released under the GPL with the additional exemption that
compiling, linking, and/or using OpenSSL is allowed.
--------------------------------------------------------------------------------
A commercial use license is available from Genivia, Inc., contact@genivia.com
--------------------------------------------------------------------------------
*/

#include "soapBasicHttpBinding_USCOREICalculatorService.h"
#include "soapBasicHttpBinding_USCOREICalculatorProxy.h"
#include "BasicHttpBinding_USCOREICalculator.nsmap"

const char *URI = "http://10.0.1.5:8000/ServiceModelSamples/service"; // the service endpoint

int main(int argc, char **argv)
{
  if (argc >= 2)
  {
    // Service sample application
    int port = atoi(argv[1]);

    BasicHttpBinding_USCOREICalculatorService service(SOAP_XML_INDENT);

    service.soap->send_timeout = service.soap->recv_timeout = 10; // 10 sec

    std::cerr << "Server running" << std::endl;

    for (;;)
    {
      service.run(port);
      service.soap_stream_fault(std::cerr);
    }
  }
  else
  {
    // Client sample application
    BasicHttpBinding_USCOREICalculatorProxy proxy(URI, SOAP_XML_INDENT);

    proxy.soap->send_timeout = proxy.soap->recv_timeout = 10; // 10 sec

    double n1 = 3.14, n2 = 1.41;

    _mssamh__Add areq;
    _mssamh__AddResponse ares;
    areq.n1 = &n1;
    areq.n2 = &n2;
    if (proxy.Add(&areq, &ares) == SOAP_OK && ares.AddResult)
      printf("Add(%g, %g) = %g\n", *areq.n1, *areq.n2, *ares.AddResult);
    else
      proxy.soap_stream_fault(std::cerr);
    proxy.destroy();

    _mssamh__Subtract sreq;
    _mssamh__SubtractResponse sres;
    sreq.n1 = &n1;
    sreq.n2 = &n2;
    if (proxy.Subtract(&sreq, &sres) == SOAP_OK && sres.SubtractResult)
      printf("Subtract(%g, %g) = %g\n", *sreq.n1, *sreq.n2, *sres.SubtractResult);
    else
      proxy.soap_stream_fault(std::cerr);
    proxy.destroy();

    _mssamh__Multiply mreq;
    _mssamh__MultiplyResponse mres;
    mreq.n1 = &n1;
    mreq.n2 = &n2;
    if (proxy.Multiply(&mreq, &mres) == SOAP_OK && mres.MultiplyResult)
      printf("Multiply(%g, %g) = %g\n", *mreq.n1, *mreq.n2, *mres.MultiplyResult);
    else
      proxy.soap_stream_fault(std::cerr);
    proxy.destroy();

    _mssamh__Divide dreq;
    _mssamh__DivideResponse dres;
    dreq.n1 = &n1;
    dreq.n2 = &n2;
    if (proxy.Divide(&dreq, &dres) == SOAP_OK && dres.DivideResult)
      printf("Divide(%g, %g) = %g\n", *dreq.n1, *dreq.n2, *dres.DivideResult);
    else
      proxy.soap_stream_fault(std::cerr);
    proxy.destroy();
  }

  return 0;
}

/******************************************************************************\
 *
 *	Service operations
 *
\******************************************************************************/

int BasicHttpBinding_USCOREICalculatorService::Add(_mssamh__Add *Add, _mssamh__AddResponse *AddResponse)
{
  double *res = (double*)soap_malloc(soap, sizeof(double));

  if (Add && Add->n1 && Add->n2)
    *res = *Add->n1 + *Add->n2;
  else
    return soap_sender_fault(soap, "Invalid data", NULL);
  AddResponse->AddResult = res;

  return SOAP_OK;
}

int BasicHttpBinding_USCOREICalculatorService::Subtract(_mssamh__Subtract *Subtract, _mssamh__SubtractResponse *SubtractResponse)
{
  double *res = (double*)soap_malloc(soap, sizeof(double));

  if (Subtract && Subtract->n1 && Subtract->n2)
    *res = *Subtract->n1 - *Subtract->n2;
  else
    return soap_sender_fault(soap, "Invalid data", NULL);
  SubtractResponse->SubtractResult = res;

  return SOAP_OK;
}

int BasicHttpBinding_USCOREICalculatorService::Multiply(_mssamh__Multiply *Multiply, _mssamh__MultiplyResponse *MultiplyResponse)
{
  double *res = (double*)soap_malloc(soap, sizeof(double));

  if (Multiply && Multiply->n1 && Multiply->n2)
    *res = *Multiply->n1 * *Multiply->n2;
  else
    return soap_sender_fault(soap, "Invalid data", NULL);
  MultiplyResponse->MultiplyResult = res;

  return SOAP_OK;
}

int BasicHttpBinding_USCOREICalculatorService::Divide(_mssamh__Divide *Divide, _mssamh__DivideResponse *DivideResponse)
{
  double *res = (double*)soap_malloc(soap, sizeof(double));

  if (Divide && Divide->n1 && Divide->n2 && *Divide->n2 != 0.0)
    *res = *Divide->n1 / *Divide->n2;
  else
    return soap_sender_fault(soap, "Invalid data", NULL);
  DivideResponse->DivideResult = res;

  return SOAP_OK;
}

