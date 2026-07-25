/*
	xmas.cpp

	Example CGI service with multiple C++ proxy linkage

	To generate non-client-server header and fault handlers:
	$ soapcpp2 -CS -penv env.h

	The gmt client proxy in 'gmt' C++ namespace:
	$ soapcpp2 -i -C -qgmt gmt.h

	The calc client proxy in 'calc' C++ namespace:
	$ soapcpp2 -i -C -qcalc calc.h

	The xmas service in 'xmas' C++ namespace:
	$ soapcpp2 -i -S -qxmas xmas.hpp

	cc -o xmas.cgi xmas.cpp stdsoap2.cpp envC.cpp gmtProxy.cpp calccalcProxy.cpp xmasmashupService.cpp gmtC.cpp calcC.cpp xmasC.cpp

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

#include "envH.h"
#include "gmtProxy.h"
#include "calccalcProxy.h"
#include "xmasmashupService.h"

int main()
{
  xmas::mashupService service;
  return service.serve();
}

/******************************************************************************\
 *
 *	Server operation
 *
\******************************************************************************/

int xmas::mashupService::dtx(_XML x, struct _ns2__commingtotown *response)
{
  gmt::Proxy Time("http://www.cs.fsu.edu/~engelen/gmtlitserver.cgi");

  time_t now;
  if (Time.gmt(&now))
    return soap_receiverfault("Cannot connect to GMT server", NULL);

  struct tm tm;

  tm.tm_sec = 0;
  tm.tm_min = 0;
  tm.tm_hour = 0;
  tm.tm_mday = 25;
  tm.tm_mon = 11;
  tm.tm_year = gmtime(&now)->tm_year; /* this year */
  tm.tm_isdst = 0;
  tm.tm_zone = NULL;

  time_t xmas = soap_timegm(&tm);

  if (xmas < now)
  {
    tm.tm_year++; /* xmas just passed, go to next year */
    xmas = soap_timegm(&tm);
  }

  double sec = difftime(xmas, now);
  
  calc::calcProxy Calc;
  double days;

  if (Calc.div(sec, 86400.0, &days))
    return soap_receiverfault("Cannot connect to calc server", NULL);

  response->days = (int)days;

  return SOAP_OK;
}
