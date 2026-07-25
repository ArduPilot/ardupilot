/*
        GoogleDistanceMatrix.h

        Google MAP API Distance Matrix example in C

	This is a translation of GoogleDistanceMatrix.hpp to C:

	1. replace 'class' with 'struct'
	2. replace 'std::string' with 'char*'
	3. replace 'std::vector<type> name' with arrays:
	   $int   num_name;
	    type *name;

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2001-2017, Robert van Engelen, Genivia, Inc. All Rights Reserved.
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

////////////////////////////////////////////////////////////////////////////////
//
//	DistanceMatrixResponse
//
////////////////////////////////////////////////////////////////////////////////

enum DistanceMatrixStatus {
  DistanceMatrixStatus__OK,
  DistanceMatrixStatus__INVALID_USCOREREQUEST,
  DistanceMatrixStatus__MAX_USCOREELEMENTS_USCOREEXCEEDED,
  DistanceMatrixStatus__OVER_USCOREQUERY_USCORELIMIT,
  DistanceMatrixStatus__REQUEST_USCOREDENIED,
  DistanceMatrixStatus__UNKNOWN_USCOREERROR,
};

struct DistanceMatrixResponse {
  enum DistanceMatrixStatus     status;
  char*                         error_USCOREmessage;
 $int                           num_origin_address;
  char*                        *origin_USCOREaddress;
 $int                           num_destination_address;
  char*                        *destination_USCOREaddress;
 $int                           num_row;
  struct DistanceMatrixRow     *row;
};

////////////////////////////////////////////////////////////////////////////////
//
//	Rows
//
////////////////////////////////////////////////////////////////////////////////

struct DistanceMatrixRow {
 $int                           num_element;
  struct DistanceMatrixElement *element;
};

////////////////////////////////////////////////////////////////////////////////
//
//	Elements
//
////////////////////////////////////////////////////////////////////////////////

struct DistanceMatrixDuration {
  double                        value;
  char*                         text;
};

struct DistanceMatrixDistance {
  double                        value;
  char*                         text;
};

typedef char* DistanceMatrixCurrency 3:3;

struct DistanceMatrixFare {
  DistanceMatrixCurrency        currency;
  double                        value;
  char*                         text;
};

enum DistanceMatrixEltStatus {
  DistanceMatrixEltStatus__OK,
  DistanceMatrixEltStatus__NOT_USCOREFOUND,
  DistanceMatrixEltStatus__ZERO_USCORERESULTS,
  DistanceMatrixEltStatus__MAX_USCOREROUTE_USCORELENGTH_USCOREEXCEEDED,
};

struct DistanceMatrixElement {
  enum DistanceMatrixEltStatus   status;
  struct DistanceMatrixDuration  duration;
  struct DistanceMatrixDuration *duration_USCOREin_USCOREtraffic; // optional duration_in_traffic
  struct DistanceMatrixDistance  distance;
  struct DistanceMatrixFare     *fare; // optional fare
};
