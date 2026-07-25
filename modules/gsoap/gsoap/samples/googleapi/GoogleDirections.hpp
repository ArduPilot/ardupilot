/*
        GoogleDirections.hpp

        Google Maps Directions API example C++ client

	This file is imported by gdm.hpp and its contents placed in the 'gdm'
	C++ namespace.

        For more details about this API, visit:

        https://developers.google.com/maps/documentation/directions/intro

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
//	GeocodedWaypoint
//
////////////////////////////////////////////////////////////////////////////////

/// indicates the status code resulting from the geocoding operation.
typedef enum DirectionsGeocoderStatus {
  /// indicates that no errors occurred; the address was successfully parsed and at least one geocode was returned.
  DirectionsGeocoderStatus__OK,
  /// indicates that the geocode was successful but returned no results. This may occur if the geocoder was passed a non-existent address.
  DirectionsGeocoderStatus__ZERO_USCORERESULTS,
} DirectionsGeocoderStatus;

/// contains details about the geocoding of origin, destination and waypoints.
class DirectionsGeocodedWaypoint {
  /// indicates the status code resulting from the geocoding operation.
  DirectionsGeocoderStatus    geocoder_USCOREstatus;
  /// indicates that the geocoder did not return an exact match for the original request, though it was able to match part of the requested address. You may wish to examine the original request for misspellings and/or an incomplete address.
  std::string                *partial_USCOREmatch; // optional partial_match
  /// a unique identifier that can be used with other Google APIs. For example, you can use the place_id from a Google Place Autocomplete response to calculate directions to a local business.
  std::string                 place_USCOREid;
  /// indicates the address type of the geocoding result used for calculating directions.
  std::vector<std::string>    type;
};

////////////////////////////////////////////////////////////////////////////////
//
//	Step
//
////////////////////////////////////////////////////////////////////////////////

/// distance in meters.
class DirectionsDistance {
  /// indicates the distance in meters
  double      value;
  /// contains a human-readable representation of the distance, displayed in units as used at the origin (or as overridden within the units parameter in the request). (For example, miles and feet will be used for any origin within the United States.) Note that regardless of what unit system is displayed as text, the distance.value field always contains a value expressed in meters.
  std::string text;
};

/// duration in seconds.
class DirectionsDuration {
  /// indicates the duration in seconds.
  double      value;
  /// contains a human-readable representation of the duration.
  std::string text;
};

/// contains a location as a single set of lat and lng fields.
class DirectionsLatLng {
  /// location lat
  double lat;
  /// location lng
  double lng;
};

/// contains a single points object that holds an encoded polyline representation of the step. This polyline is an approximate (smoothed) path of the step.
class DirectionsPolyline {
  /// encoded polyline representation https://developers.google.com/maps/documentation/utilities/polylinealgorithm
  std::string points;
};

/// contains information about the stop/station for this part of the trip.
class DirectionsStop {
  /// the name of the transit station/stop. eg. "Union Square".
  std::string      name;
  /// the location of the transit station/stop, represented as a lat and lng field.
  DirectionsLatLng location;
};

/// contain the arrival or departure times for this leg of the journey.
class DirectionsTime {
  std::string value; // contains Unix time, or seconds since 1/1/2070 UTC a JavaScript Date object
  std::string text;
  std::string time_USCOREzone;
};

/// contains an array of TransitAgency objects that each provide information about the operator of the line.
class DirectionsAgency {
  /// contains the name of the transit agency.
  std::string name;
  /// contains the URL for the transit agency.
  std::string url;
  /// contains the phone number of the transit agency.
  std::string phone;
};

/// contains the type of vehicle that runs on this line.
typedef enum DirectionsVehicleType {
  /// Rail.
  DirectionsVehicleType__RAIL,
  /// Light rail transit.
  DirectionsVehicleType__METRO_USCORERAIL,
  /// Underground light rail.
  DirectionsVehicleType__SUBWAY,
  /// Above ground light rail.
  DirectionsVehicleType__TRAM,
  /// Monorail.
  DirectionsVehicleType__MONORAIL,
  /// Heavy rail.
  DirectionsVehicleType__HEAVY_USCORERAIL,
  /// Commuter rail.
  DirectionsVehicleType__COMMUTER_USCORETRAIN,
  /// High speed train.
  DirectionsVehicleType__HIGH_USCORESPEED_USCORETRAIN,
  /// Bus.
  DirectionsVehicleType__BUS,
  /// Intercity bus.
  DirectionsVehicleType__INTERCITY_USCOREBUS,
  /// Trolleybus.
  DirectionsVehicleType__TROLLEYBUS,
  /// Share taxi is a kind of bus with the ability to drop off and pick up passengers anywhere on its route.
  DirectionsVehicleType__SHARE_USCORETAXI,
  /// Ferry.
  DirectionsVehicleType__FERRY,
  /// A vehicle that operates on a cable, usually on the ground. Aerial cable cars may be of the type GONDOLA_LIFT.
  DirectionsVehicleType__CABLE_USCORECAR,
  /// An aerial cable car.
  DirectionsVehicleType__GONDOLA_USCORELIFT,
  /// A vehicle that is pulled up a steep incline by a cable. A Funicular typically consists of two cars, with each car acting as a counterweight for the other.
  DirectionsVehicleType__FUNICULAR,
  /// All other vehicles will return this type.
  DirectionsVehicleType__OTHER,
} DirectionsVehicleType;

/// contains the type of vehicle used on this line.
class DirectionsVehicle {
  /// contains the name of the vehicle on this line. eg. "Subway."
  std::string           name;
  /// contains the type of vehicle that runs on this line.
  DirectionsVehicleType type;
  /// contains the URL for an icon associated with this vehicle type.
  std::string           icon;
  /// contains the URL for the icon associated with this vehicle type, based on the local transport signage.
  std::string           local_USCOREicon;
};

/// contains information about the transit line used in this step.
class DirectionsLine {
  /// contains the full name of this transit line. eg. "7 Avenue Express".
  std::string                   name;
  /// contains the short name of this transit line. This will normally be a line number, such as "M7" or "355".
  std::string                   short_USCOREname;
  /// contains the color commonly used in signage for this transit line. The color will be specified as a hex string such as: #FF0033.
  std::string                   color;
  /// contains an array of TransitAgency objects that each provide information about the operator of the line. You must display the names and URLs of the transit agencies servicing the trip results.
  std::vector<DirectionsAgency> agency;
  /// contains the URL for this transit line as provided by the transit agency.
  std::string                   url;
  /// contains the URL for the icon associated with this line.
  std::string                   icon;
  /// contains the color of text commonly used for signage of this line. The color will be specified as a hex string.
  std::string                   text_USCOREcolor;
  /// contains the type of vehicle used on this line.
  DirectionsVehicle             vehicle;
};

/// contains additional information that is not relevant for other modes of transportation. These additional properties are exposed through the transit_details object, returned as a field of an element in the steps[] array. From the TransitDetails object you can access additional information about the transit stop, transit line and transit agency.
class DirectionsTransitDetails {
  /// contains information about the stop/station for this part of the trip.
  DirectionsStop arrival_USCOREstop;
  /// contains information about the stop/station for this part of the trip.
  DirectionsStop departure_USCOREstop;
  /// contain the arrival or departure times for this leg of the journey.
  DirectionsTime arrival_USCOREtime;
  /// contain the arrival or departure times for this leg of the journey.
  DirectionsTime departure_USCOREtime;
  /// specifies the direction in which to travel on this line, as it is marked on the vehicle or at the departure stop. This will often be the terminus station.
  std::string    headsign;
  /// specifies the expected number of seconds between departures from the same stop at this time. For example, with a headway value of 600, you would expect a ten minute wait if you should miss your bus.
  unsigned long  headway;
  /// contains the number of stops in this step, counting the arrival stop, but not the departure stop. For example, if your directions involve leaving from Stop A, passing through stops B and C, and arriving at stop D, num_stops will return 3.
  unsigned long  num_USCOREstops;
  /// contains information about the transit line used in this step.
  DirectionsLine line;
};

/// defines a single step of the calculated directions. A step is the most atomic unit of a direction's route, containing a single step describing a specific, single instruction on the journey. E.g. "Turn left at W. 4th St." The step not only describes the instruction but also contains distance and duration information relating to how this step relates to the following step. For example, a step denoted as "Merge onto I-80 West" may contain a duration of "37 miles" and "40 minutes," indicating that the next step is 37 miles/40 minutes from this step.
class DirectionsStep {
  /// contains formatted instructions for this step, presented as an HTML text string.
  std::string                 html_USCOREinstructions;
  /// contains the mode of travel (not documented by Google Maps API).
  std::string                 travel_USCOREmode;
  /// contains maneuver instructions (not documented by the Google Maps API).
  std::string                *maneuver;
  /// contains the distance covered by this step until the next step. This field may be undefined if the distance is unknown.
  DirectionsDistance         *distance; // optional distance
  /// contains the typical time required to perform the step, until the next step. This field may be undefined if the duration is unknown.
  DirectionsDuration         *duration;
  /// contains the location of the starting point of this step, as a single set of lat and lng fields.
  DirectionsLatLng            start_USCORElocation;
  /// contains the location of the last point of this step, as a single set of lat and lng fields.
  DirectionsLatLng            end_USCORElocation;
  /// contains a single points object that holds an encoded polyline representation of the step. This polyline is an approximate (smoothed) path of the step.
  DirectionsPolyline          polyline;
  /// contains detailed directions for walking or driving steps in transit directions. Substeps are only available when travel_mode is set to "transit". The inner steps array is of the same type as steps.
  std::vector<DirectionsStep> step;
  /// contains transit specific information. This field is only returned with travel_mode is set to "transit".
  DirectionsTransitDetails   *transit_USCOREdetails;
};

////////////////////////////////////////////////////////////////////////////////
//
//	Leg
//
////////////////////////////////////////////////////////////////////////////////

/// specifies a single leg of the journey from the origin to the destination in the calculated route. For routes that contain no waypoints, the route will consist of a single "leg," but for routes that define one or more waypoints, the route will consist of one or more legs, corresponding to the specific legs of the journey.
class DirectionsLeg {
  /// contains an array of steps denoting information about each separate step of the leg of the journey.
  std::vector<DirectionsStep> step;
  /// indicates the total distance covered by this leg.
  DirectionsDistance         *distance; // optional distance
  /// indicates the total duration of this leg.
  DirectionsDuration         *duration; // optional duration
  /// indicates the total duration of this leg. This value is an estimate of the time in traffic based on current and historical traffic conditions.
  DirectionsDuration         *duration_USCOREin_USCOREtraffic; // optional duration_in_traffic
  /// contains the estimated time of arrival for this leg. This property is only returned for transit directions.
  DirectionsTime             *arrival_USOREtime; // optional arrival_time
  /// contains the estimated time of departure for this leg, specified as a Time object. The departure_time is only available for transit directions.
  DirectionsTime             *departure_USCOREtime; // optional departure_time
  /// contains the latitude/longitude coordinates of the origin of this leg. Because the Directions API calculates directions between locations by using the nearest transportation option (usually a road) at the start and end points, start_location may be different than the provided origin of this leg if, for example, a road is not near the origin.
  DirectionsLatLng            start_USCORElocation;
  /// contains the latitude/longitude coordinates of the given destination of this leg. Because the Google Maps Directions API calculates directions between locations by using the nearest transportation option (usually a road) at the start and end points, end_location may be different than the provided destination of this leg if, for example, a road is not near the destination.
  DirectionsLatLng            end_USCORElocation;
  /// contains the human-readable address (typically a street address) resulting from reverse geocoding the start_location of this leg.
  std::string                 start_USCOREaddress;
  /// contains the human-readable address (typically a street address) from reverse geocoding the end_location of this leg.
  std::string                 end_USCOREaddress;
};

////////////////////////////////////////////////////////////////////////////////
//
//	Route
//
////////////////////////////////////////////////////////////////////////////////

/// the viewport bounding box of the overview_polyline.
class DirectionsBound {
  /// NE corner of the bounding box.
  DirectionsLatLng northeast;
  /// SW corner of the bounding box.
  DirectionsLatLng southwest;
};

/// ISO 4217 currency code indicating the currency that the amount is expressed in.
typedef std::string DirectionsCurrency 3:3;

/// the total fare (that is, the total ticket costs) on this route. This property is only returned for transit requests and only for routes where fare information is available for all transit legs. 
class DirectionsFare {
  /// an ISO 4217 currency code indicating the currency that the amount is expressed in.
  DirectionsCurrency currency;
  /// the total fare amount, in the currency specified above.
  double             value;
  /// the total fare amount, formatted in the requested language.
  std::string        text;
};

/// contains a single result from the specified origin and destination. This route may consist of one or more legs depending on whether any waypoints were specified. As well, the route also contains copyright and warning information which must be displayed to the user in addition to the routing information.
class DirectionsRoute {
  /// contains a short textual description for the route, suitable for naming and disambiguating the route from alternatives.
  std::string                  summary;
  /// contains an array which contains information about a leg of the route, between two locations within the given route. A separate leg will be present for each waypoint or destination specified. (A route with no waypoints will contain exactly one leg within the legs array.) Each leg consists of a series of steps.
  std::vector<DirectionsLeg>   leg;
  /// contains an array indicating the order of any waypoints in the calculated route. This waypoints may be reordered if the request was passed optimize:true within its waypoints parameter.
  std::vector<unsigned long>   waypoint_USCOREindex;
  /// contains a single points object that holds an encoded polyline representation of the route. This polyline is an approximate (smoothed) path of the resulting directions.
  DirectionsPolyline           overview_USCOREpolyline;
  /// contains the viewport bounding box of the overview_polyline.
  std::vector<DirectionsBound> bounds;
  /// contains the copyrights text to be displayed for this route. You must handle and display this information yourself.
  std::string                  copyrights;
  /// contains an array of warnings to be displayed when showing these directions. You must handle and display these warnings yourself.
  std::vector<std::string>     warning;
  /// contains the total fare (that is, the total ticket costs) on this route. This property is only returned for transit requests and only for routes where fare information is available for all transit legs. 
  DirectionsFare              *fare; // optional fare
};

////////////////////////////////////////////////////////////////////////////////
//
//	DirectionsResponse
//
////////////////////////////////////////////////////////////////////////////////

/// contains the status of the request, and may contain debugging information to help you track down why the Directions service failed.
typedef enum DirectionsStatus {
  /// indicates the response contains a valid result.
  DirectionsStatus__OK,
  /// indicates at least one of the locations specified in the request's origin, destination, or waypoints could not be geocoded
  DirectionsStatus__NOT_USCOREFOUND,
  /// indicates no route could be found between the origin and destination.
  DirectionsStatus__ZERO_USCORERESULTS,
  /// indicates that too many waypoints were provided in the request. For applications using the Google Maps Directions API as a web service, or the directions service in the Google Maps JavaScript API, the maximum allowed number of waypoints is 23, plus the origin and destination. Google Maps APIs Premium Plan customers may submit requests with up to 23 waypoints, plus the origin and destination.
  DirectionsStatus__MAX_USCOREWAYPOINTS_USCOREEXCEEDED,
  /// indicates the requested route is too long and cannot be processed.
  DirectionsStatus__MAX_USCOREROUTE_USCORELENGTH_USCOREEXCEEDED,
  /// indicates that the provided request was invalid. Common causes of this status include an invalid parameter or parameter value.
  DirectionsStatus__INVALID_USCOREREQUEST,
  /// indicates the service has received too many requests from your application within the allowed time period.
  DirectionsStatus__OVER_USCOREQUERY_USCORELIMIT,
  /// indicates that the service denied use of the directions service by your application.
  DirectionsStatus__REQUEST_USCOREDENIED,
  /// indicates a directions request could not be processed due to a server error. The request may succeed if you try again.
  DirectionsStatus__UNKNOWN_USCOREERROR,
} DirectionsStatus;

/// root element
class DirectionsResponse {
  /// contains the status of the request, and may contain debugging information to help you track down why the Directions service failed.
  DirectionsStatus                        status;
  /// detailed information about the reasons behind the given status code.
  std::string                             error_USCOREmessage 0; // optional error_message
  /// contains an array with details about the geocoding of origin, destination and waypoints.
  std::vector<DirectionsGeocodedWaypoint> geocoded_USCOREwaypoint;
  /// contains an array of routes from the origin to the destination.
  std::vector<DirectionsRoute>            route;
  /// contains an array of available travel modes. This field is returned when a request specifies a travel mode and gets no results. The array contains the available travel modes in the countries of the given set of waypoints. This field is not returned if one or more of the waypoints are via: waypoints.
  std::vector<std::string>                available_USCOREtravel_USCOREmodes;
};
