/*

example2.cpp

Example UDDI V2 Client

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2004-2005, Robert van Engelen, Genivia Inc. All Rights Reserved.
This software is released under one of the following two licenses:
GPL or Genivia's license for commercial use.
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

#include "inqH.h"

int main(int argc, char **argv)
{ 
  char *search_string = "xmethods";

  if (argc > 1)
    search_string = argv[1];

  // Create a gSOAP context
  struct soap *soap = soap_new();

  // Create an object to find a business
  uddi2__find_USCOREbusiness fb(soap, search_string);

  // Send the request
  uddi2__businessList *bl = fb.send("http://uddi.xmethods.net/inquire");

  // Check if result is OK
  if (!bl)
    soap_print_fault(soap, stderr);

  // If OK, report the business name(s) and unique identification keys
  else if (bl->businessInfos)
  {
    std::cout << "Search results on " << search_string << ":" << std::endl << std::endl;

    for (std::vector<uddi2__businessInfo*>::const_iterator bi = bl->businessInfos->businessInfo.begin(); bi != bl->businessInfos->businessInfo.end(); ++bi)
    {
      // Report businessKey
      std::cout << "businessKey=" << (*bi)->businessKey << std::endl;

      // Report names
      for (std::vector<uddi2__name*>::const_iterator n = (*bi)->name.begin(); n != (*bi)->name.end(); ++n)
        std::cout << "name=" << (*n)->__item << std::endl;

      std::cout << std::endl;
    }
  }

  // Remove deserialized objects
  soap_destroy(soap);

  // Remove temporary data
  soap_end(soap);

  // Detach and free context
  soap_done(soap);
  free(soap);

  return 0;
}
