/*
	mashupclient.c

	Example mashup service client in C

	soapcpp2 -c mashup.h
	cc -o mashupclient mashupclient.c stdsoap2.c soapC.c soapClient.c

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

#include "soapH.h"
#include "mashup.nsmap"

int main()
{
  struct soap *soap = soap_new();
  struct _ns3__commingtotown response;

  if (soap_call___ns5__dtx(soap, NULL, NULL, "", &response))
    soap_print_fault(soap, stderr);
  else if (response.days == 0)
    printf("Today is the day!\n");
  else
    printf("Wait %d more days to xmas\n", response.days);

  return 0;
}
