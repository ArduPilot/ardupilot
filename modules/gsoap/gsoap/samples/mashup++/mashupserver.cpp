/*
	mashupserver.cpp

	Example mashup CGI service in C++

	soapcpp2 -i mashup.hpp
	cc -o mashupserver mashupserver.cpp stdsoap2.cpp soapC.cpp soapcalcProxy.cpp soapServiceProxy.cpp soapmashupService.cpp

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

#include "soapcalcProxy.h"
#include "soapgmtProxy.h"
#include "soapmashupService.h"
#include "mashup.nsmap"

int main()
{
  mashupService service;

  return service.serve();
}

/******************************************************************************\
 *
 *	Server operation
 *
\******************************************************************************/

int mashupService::dtx(_XML x, _ns3__commingtotown &response)
{
  (void)x; /* input param that is always empty is simply ignored */

  gmtProxy Time("http://www.cs.fsu.edu/~engelen/gmtlitserver.cgi");

  _ns1__gmt gmt;
  _ns1__gmtResponse gmtResponse;

  if (Time.gmt(&gmt, gmtResponse))
    return soap_receiverfault("Cannot connect to GMT server", NULL);

  time_t now = gmtResponse.param_1;

  struct tm tm;

  memset(&tm, 0, sizeof(struct tm));
  tm.tm_mday = 25;
  tm.tm_mon = 11;
  tm.tm_year = gmtime(&now)->tm_year; // this year

  time_t xmas = soap_timegm(&tm);

  if (xmas < now)
  {
    tm.tm_year++; // xmas just passed, go to next year
    xmas = soap_timegm(&tm);
  }

  double sec = difftime(xmas, now);
  
  calcProxy Calc;
  double days;

  if (Calc.div(sec, 86400.0, days))
    return soap_receiverfault("Cannot connect to calc server", NULL);

  response.days = (int)days;

  soap_delegate_deletion(&Time, this); // Time data to be deleted by 'this'
  soap_delegate_deletion(&Calc, this); // Calc data to be deleted by 'this'

  return SOAP_OK;
}
