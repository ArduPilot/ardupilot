/*
	router.h

	Simple Web Service message router (relay server)

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2001-2016, Robert van Engelen, Genivia, Inc. All Rights Reserved.
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

struct t__Routing
{
  char *key;      // key matches SOAPAction or query string or endpoint URL
  char *endpoint;
  char *userid;	  // optional HTTP Authorization userid
  char *passwd;	  // optional HTTP Authorization passwd
};

struct t__RoutingTable
{
  int                __size; // size of array
  struct t__Routing *__ptr;  // array of Routing entries
};
