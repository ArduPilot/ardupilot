/*
        GoogleDistanceMatrix.hpp

        Google Maps Distance Matrix API example C++ client

	This file is imported by gdm.hpp and its contents placed in the 'gdm'
	C++ namespace.

        For more details about this API, visit:

        https://developers.google.com/maps/documentation/distance-matrix/intro

        To get an API key:
        https://developers.google.com/maps/documentation/distance-matrix/get-api-key

        Build steps:
        soapcpp2 -0 GoogleDistanceMatrix.hpp
        soapcpp2 -0 -penv env.h
        c++ -DWITH_OPENSSL -o gdm gdm.cpp stdsoap2.cpp gdmC.cpp envC.cpp -lssl -lcrypto

        Usage:
        gdm <key> <mode> <units> 'origin addresses' 'destination addresses'
        where addresses should be separated by |

        Example:
        gdm <key> bicycling imperial 'Vancouver BC|Seattle' 'San Francisco|Vancouver BC'

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
//	Elements
//
////////////////////////////////////////////////////////////////////////////////

/// an ISO 4217 currency code indicating the currency that the amount is expressed in.
typedef std::string DistanceMatrixCurrency 3:3;

/// duration in seconds.
class DistanceMatrixDuration {
  /// indicates the duration in seconds.
  double      value;
  /// contains a human-readable representation of the duration.
  std::string text;
};

/// distance in meters.
class DistanceMatrixDistance {
  /// indicates the distance in meters
  double      value;
  /// contains a human-readable representation of the distance, displayed in units as used at the origin (or as overridden within the units parameter in the request). (For example, miles and feet will be used for any origin within the United States.) Note that regardless of what unit system is displayed as text, the distance.value field always contains a value expressed in meters.
  std::string text;
};

/// the total fare (that is, the total ticket costs) on this route. This property is only returned for transit requests and only for routes where fare information is available for all transit legs. 
class DistanceMatrixFare {
  /// an ISO 4217 currency code indicating the currency that the amount is expressed in.
  DistanceMatrixCurrency currency;
  /// the total fare amount, in the currency specified above.
  double                 value;
  /// the total fare amount, formatted in the requested language.
  std::string            text;
};

/// contains metadata on the element.
typedef enum DistanceMatrixEltStatus {
  /// indicates the response contains a valid result.
  DistanceMatrixEltStatus__OK,
  /// indicates that the origin and/or destination of this pairing could not be geocoded.
  DistanceMatrixEltStatus__NOT_USCOREFOUND,
  /// indicates no route could be found between the origin and destination.
  DistanceMatrixEltStatus__ZERO_USCORERESULTS,
  /// indicates the requested route is too long and cannot be processed.
  DistanceMatrixEltStatus__MAX_USCOREROUTE_USCORELENGTH_USCOREEXCEEDED,
} DistanceMatrixEltStatus;

/// contains information about a single origin-destination pairing.
class DistanceMatrixElement {
  /// contains metadata on the element.
  DistanceMatrixEltStatus status;
  /// contains the length of time it takes to travel this route, expressed in seconds (the value field) and as text. The textual representation is localized according to the query's language parameter.
  DistanceMatrixDuration  duration;
  /// contains the length of time it takes to travel this route, based on current and historical traffic conditions. See the traffic_model request parameter for the options you can use to request that the returned value is optimistic, pessimistic, or a best-guess estimate. The duration is expressed in seconds (the value field) and as text. The textual representation is localized according to the query's language parameter.
  DistanceMatrixDuration *duration_USCOREin_USCOREtraffic; // optional duration_in_traffic
  /// contains the total distance of this route, expressed in meters (value) and as text.
  DistanceMatrixDistance  distance;
  /// if present, contains the total fare (that is, the total ticket costs) on this route. This property is only returned for transit requests and only for transit providers where fare information is available.
  DistanceMatrixFare     *fare; // optional fare
};

////////////////////////////////////////////////////////////////////////////////
//
//	Rows
//
////////////////////////////////////////////////////////////////////////////////

/// contains one or more element entries, which in turn contain the information about a single origin-destination pairing.
class DistanceMatrixRow {
  /// contains an array of information about a single origin-destination pairing.
  std::vector<DistanceMatrixElement> element;
};

////////////////////////////////////////////////////////////////////////////////
//
//	DistanceMatrixResponse
//
////////////////////////////////////////////////////////////////////////////////

/// contains metadata on the request.
typedef enum DistanceMatrixStatus {
  /// indicates the response contains a valid result.
  DistanceMatrixStatus__OK,
  /// indicates that the provided request was invalid.
  DistanceMatrixStatus__INVALID_USCOREREQUEST,
  /// indicates that the product of origins and destinations exceeds the per-query limit.
  DistanceMatrixStatus__MAX_USCOREELEMENTS_USCOREEXCEEDED,
  /// indicates the service has received too many requests from your application within the allowed time period.
  DistanceMatrixStatus__OVER_USCOREQUERY_USCORELIMIT,
  /// indicates that the service denied use of the Distance Matrix service by your application.
  DistanceMatrixStatus__REQUEST_USCOREDENIED,
  /// indicates a Distance Matrix request could not be processed due to a server error. The request may succeed if you try again.
  DistanceMatrixStatus__UNKNOWN_USCOREERROR,
} DistanceMatrixStatus;

/// root element
class DistanceMatrixResponse {
  /// contains metadata on the request.
  DistanceMatrixStatus           status;
  /// contains more detailed information about the reasons behind the given status code.
  std::string                    error_USCOREmessage 0; // optional error_message
  /// contains an array of addresses as returned by the API from your original request. These are formatted by the geocoder and localized according to the language parameter passed with the request.
  std::vector<std::string>       origin_USCOREaddress;
  /// contains an array of addresses as returned by the API from your original request. As with origin_addresses, these are localized if appropriate.
  std::vector<std::string>       destination_USCOREaddress;
  /// contains an array of elements, which in turn each contain a status, duration, and distance element. Rows are ordered according to the values in the origin parameter of the request. Each row corresponds to an origin, and each element within that row corresponds to a pairing of the origin with a destination value.
  std::vector<DistanceMatrixRow> row;
};
