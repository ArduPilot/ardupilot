/*
	mashupclient.cpp

	Example mashup service client in C++

	soapcpp2 -i mashup.hpp
	cc -o mashupclient mashupclient.cpp stdsoap2.cpp soapC.cpp soapmashupProxy.cpp

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

#include "soapmashupProxy.h"
#include "mashup.nsmap"

int main()
{
  mashupProxy proxy;
  _ns3__commingtotown response;

  if (proxy.dtx((char*)"", response))
    proxy.soap_stream_fault(std::cerr);
  else if (response.days == 0)
    std::cout << "Today is the day!" << std::endl;
  else
    std::cout << "Wait " << response.days << " more days to xmas" << std::endl;

  proxy.destroy(); // delete deserialized data

  return 0;
}
