/*
	calcserver.cpp

	Example calculator service in C++

	$ soapcpp2 -i calc.h
	$ c++ -o calcserver++ calcserver.cpp stdsoap2.cpp soapC.cpp soapcalcService.cpp
	where stdsoap2.cpp is in the 'gsoap' directory, or use libgsoap++:
	$ c++ -o calcserver++ calcserver.cpp soapC.cpp soapcalcService.cpp -lgsoap++

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2001-2008, Robert van Engelen, Genivia, Inc. All Rights Reserved.
This software is released under one of the following two licenses:
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
--------------------------------------------------------------------------------
A commercial use license is available from Genivia, Inc., contact@genivia.com
--------------------------------------------------------------------------------
*/

#include "soapcalcService.h"
#include "calc.nsmap"

int main(int argc, char **argv)
{
  calcService calc;
  if (argc < 2)
    calc.serve();	/* serve as CGI application */
  else
  {
    int port = atoi(argv[1]);
    if (!port)
    {
      fprintf(stderr, "Usage: calcserver++ <port>\n");
      exit(0);
    }
    /* run iterative server on port until fatal error */
    if (calc.run(port))
    {
      calc.soap_stream_fault(std::cerr);
      exit(1);
    }
  }
  return 0;
} 

int calcService::add(double a, double b, double *result)
{
  *result = a + b;
  return SOAP_OK;
} 

int calcService::sub(double a, double b, double *result)
{
  *result = a - b;
  return SOAP_OK;
} 

int calcService::mul(double a, double b, double *result)
{
  *result = a * b;
  return SOAP_OK;
} 

int calcService::div(double a, double b, double *result)
{
  if (b)
    *result = a / b;
  else
  {
    char *s = (char*)soap_malloc(this, 1024);
    (SOAP_SNPRINTF(s, 1024, 100), "<error xmlns=\"http://tempuri.org/\">Can't divide %f by %f</error>", a, b);
    return soap_senderfault("Division by zero", s);
  }
  return SOAP_OK;
} 

int calcService::pow(double a, double b, double *result)
{
  *result = ::pow(a, b);
  if (soap_errno == EDOM)	/* soap_errno is like errno, but compatible with Win32 */
  {
    char *s = (char*)soap_malloc(this, 1024);
    (SOAP_SNPRINTF(s, 1024, 100), "<error xmlns=\"http://tempuri.org/\">Can't take power of %f to %f</error>", a, b);
    return soap_senderfault("Power function domain error", s);
  }
  return SOAP_OK;
}
