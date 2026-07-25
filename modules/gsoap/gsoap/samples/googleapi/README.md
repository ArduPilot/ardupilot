Google Maps API C++ examples
============================

This example explains the implementation of two client applications for the
Google Maps API:

- `gdm` Google Distance Matrix: search for travel distance and time for a
  matrix of origins and destinations.  Returns a matrix of distances and time
  for each pair of origin and destination.  Specify mode of transportation,
  unit (metric or imperial), origin addresses, and destination addresses as as
  text strings or as latitude/longitude coordinates, or as place IDs.
  
- `gdx` Google Directions: search for directions for several modes of
  transportation, including transit, driving, walking or cycling.  Returns
  multi-part directions using a series of waypoints.  Specify mode of
  transportation, unit (metric or imperial), origin address, and destination
  address as as text strings or as latitude/longitude coordinates, or as place
  IDs.

These applications are executed from the command line.

To find all pair-wise bicycling distances and times from Vancouver BC or from
Seattle to San Francisco or to Vancouver BC:

    ./gdm <YOUR_API_KEY> bicycling imperial 'Vancouver BC|Seattle' 'San Francisco|Vancouver BC'
    ----------------------------------------
    from: Vancouver, BC, Canada
    to: San Francisco, CA, USA
    duration: 326774 (3 days 19 hours)
    distance: 1.70553e+06 (1,060 mi)
    to: Vancouver, BC, Canada
    duration: 0 (1 min)
    distance: 0 (1 ft)
    ----------------------------------------
    from: Seattle, WA, USA
    to: San Francisco, CA, USA
    duration: 274759 (3 days 4 hours)
    distance: 1.45204e+06 (902 mi)
    to: Vancouver, BC, Canada
    duration: 55726 (15 hours 29 mins)
    distance: 271157 (168 mi)
    ----------------------------------------

To find public transit directions from Times Square to Battery Park in New York
City:

    ./gdx <YOUR_API_KEY> transit imperial 'Times Square,New York,NY' 'Battery Park,New York,NY'
    ----------------------------------------
    summary: 
    warning: Walking directions are in beta.    Use caution – This route may be missing sidewalks or pedestrian paths.
    copyrights: Map data ©2017 Google

    from Manhattan, NY 10036, USA to Battery Park, New York, NY 10004, USA takes 7386 meters (4.6 mi) 1531 seconds (26 mins)

    Walk to Times Sq - 42 St
    WALKING 551 meters (0.3 mi) 353 seconds (6 mins)
      Head <b>southeast</b> on <b>W 46th St</b> toward <b>Broadway</b>
      => WALKING 8 meters (26 ft) 6 seconds (1 min)
      Turn <b>right</b> onto <b>Broadway</b>
      => turn-right, WALKING 107 meters (351 ft) 81 seconds (1 min)
      Slight <b>right</b> onto <b>7th Ave</b><div style="font-size:0.9em">Destination will be on the left</div>
      => turn-slight-right, WALKING 318 meters (0.2 mi) 254 seconds (4 mins)
      => WALKING 118 meters (387 ft) 12 seconds (1 min)

    Subway towards South Ferry
    TRANSIT 6705 meters (4.2 mi) 1080 seconds (18 mins)
      take transit at 11:37am from Times Sq - 42 St to South Ferry Station arriving at 11:56am:
      => Subway Broadway - 7 Avenue Local line 1 heading South Ferry and exit at stop 12
      (operated by MTA New York City Transit http://www.mta.info/) 

    Walk to Battery Park, New York, NY 10004, USA
    WALKING 130 meters (427 ft) 100 seconds (2 mins)
      Head <b>north</b> on <b>Peter Minuit Plaza</b>
      => WALKING 17 meters (56 ft) 13 seconds (1 min)
      Turn <b>left</b> to stay on <b>Peter Minuit Plaza</b>
      => turn-left, WALKING 68 meters (223 ft) 51 seconds (1 min)
      Turn <b>left</b> onto <b>Battery Bikeway</b><div style="font-size:0.9em">Destination will be on the right</div>
      => turn-left, WALKING 45 meters (148 ft) 36 seconds (1 min)

The `gdm` and `gdx` example client applications use the Google Maps APIs with
XML responses.

You can find a JSON-based `gdm` implementation `json-GoogleDistanceMatrix.cpp`
for the Google Maps Distance Matrix API in gsoap/samples/xml-rpc-json in the
gSOAP source code tree.

These are just two examples of the Google Maps API that can be developed quite
easily with gSOAP.

Using XML data bindings
-----------------------

Interestingly, Google refers to JSON as the preferred response format.
However, an XML API is more practical in C++ code with gSOAP's XML data
bindings than the JSON API (in any C/C++ JSON implementation).  An XML data
binding offers the benefit of strong typing in C++ for reliable API coding.  Of
course, this assumes that an XML data binding interface header file is already
defined in order to generate the XML data binding implementation with soapcpp2.
In order to auto-generate an XML data binding interface header file with wsdl2h
we need a WSDL or at least an XSD (XML schema) file.  A WSDL or XML schema
serves as a contract between the Web service and the client applications that
use the Web service.

Unfortunately, no WSDL or XML schema is provided for Google Maps APIs.
Without a WSDL or XSD we cannot auto-generate the data binding interface with
the gSOAP wsdl2h tool.

However, it is relatively easy to define the data binding interface header
file manually based on the Google Maps API documentation.

The Google Maps API documentation is somewhat incomplete.  Returned values are
poorly defined with respect to the type of the values returned.  Some elements,
such as the Directions API `maneuver` element, are not documented at all.  Or
values are not documented, such as the `establishment` value of the `type`
element in `geocoded_waypoints`.  Some XML elements (JSON objects) appear to be
optional, guessing from the examples and descriptions.  Numerical values appear
to be integer, but JSON numerical values are unconstrained decimal according to
the JSON "standard".  We assume that the C/C++ `double` type will suffice to
store numerical values returned by the API.

Google Maps Distance Matrix API
-------------------------------

The Google Maps Distance Matrix API returns information based on the
recommended route between start and end points, as calculated by the Google
Maps API, and consists of rows containing duration and distance values for each
pair.

See: <https://developers.google.com/maps/documentation/distance-matrix/intro>

A Google Maps Distance Matrix API GET request URL takes the following form:

    https://maps.googleapis.com/maps/api/distancematrix/outputFormat?parameters

where `outputFormat` may be either `json` or `xml`.

To use the Google Maps Distance Matrix API you need an API key.  To get an API
key visit:
<https://developers.google.com/maps/documentation/distance-matrix/get-api-key>

Google Maps Directions API
--------------------------

The  Google Maps Directions API returns the most efficient routes when
calculating directions.  Travel time is the primary factor optimized, but the
API may also take into account other factors such as distance, number of turns
and many more when deciding which route is the most efficient.  Calculating
directions is a time and resource intensive task. Whenever possible, use the
service described here to calculate known addresses ahead of time and store the
results in a temporary cache of your own design.

See: <https://developers.google.com/maps/documentation/directions/intro>

A Google Maps Directions API GET request URL takes the following form:

    https://maps.googleapis.com/maps/api/directions/outputFormat?parameters

where `outputFormat` may be either `json` or `xml`.

To use the Google Maps Distance Matrix API you need an API key.  To get an API
key visit:
<https://developers.google.com/maps/documentation/directions/get-api-key>

Google Maps Distance Matrix API interface header file
-----------------------------------------------------

The `gdm` example client applications uses the Google Maps APIs with XML
responses.

For example, given "Vancouver BC" and "Seattle" as origins, "San Fransicso" and
"Vancouver BS" as destinations, the following GET request computes the
bycycling durations and distance for the four origin-destination pair
combinations:

    https://maps.googleapis.com/maps/api/distancematrix/xml?origins=Vancouver+BC|Seattle&destinations=San+Francisco|Vancouver+BC&mode=bicycling&units=imperial&key=YOUR_API_KEY

This returns the following XML response by the API:

    <?xml version="1.0" encoding="UTF-8"?>
    <DistanceMatrixResponse>
     <status>OK</status>
     <origin_address>Vancouver, BC, Canada</origin_address>
     <origin_address>Seattle, WA, USA</origin_address>
     <destination_address>San Francisco, CA, USA</destination_address>
     <destination_address>Vancouver, BC, Canada</destination_address>
     <row>
      <element>
       <status>OK</status>
       <duration>
        <value>326774</value>
        <text>3 days 19 hours</text>
       </duration>
       <distance>
        <value>1705533</value>
        <text>1,060 mi</text>
       </distance>
      </element>
      <element>
       <status>OK</status>
       <duration>
        <value>0</value>
        <text>1 min</text>
       </duration>
       <distance>
        <value>0</value>
        <text>1 ft</text>
       </distance>
      </element>
     </row>
     <row>
      <element>
       <status>OK</status>
       <duration>
        <value>274759</value>
        <text>3 days 4 hours</text>
       </duration>
       <distance>
        <value>1452044</value>
        <text>902 mi</text>
       </distance>
      </element>
      <element>
       <status>OK</status>
       <duration>
        <value>55726</value>
        <text>15 hours 29 mins</text>
       </duration>
       <distance>
        <value>271157</value>
        <text>168 mi</text>
       </distance>
      </element>
     </row>
    </DistanceMatrixResponse>

To use XML data bindings with gSOAP to parse this XML directly into C++
objects, we define an interface header file `GoogleDistanceMatrix.hpp` as
follows.

The interface header file declares the `DistanceMatrixResponse` element that
contains a `row` container with `element` containers.

### DistanceMatrixResponse

Distance Matrix responses contain the following root elements:

- `status` contains metadata on the request.

- `origin_addresses` contains an array of addresses as returned by the API from
  your original request. These are formatted by the geocoder and localized
  according to the language parameter passed with the request.

- `destination_addresses` contains an array of addresses as returned by the API
  from your original request. As with `origin_addresses`, these are localized
  if appropriate.

- `rows` contains an array of elements, which in turn each contain a `status`,
  `duration`, and `distance` element.

When the top-level status code is other than OK, there may be an additional
`error_message` field within the Distance Matrix response object. This field
contains more detailed information about the reasons behind the given status
code.

For more details on the XML elements and their values, see
<https://developers.google.com/maps/documentation/distance-matrix/intro>

The `DistanceMatrixResponse` root element is declared in our data binding
interface file `GoogleDistanceMatrix.hpp` as:

    class DistanceMatrixResponse {
      DistanceMatrixStatus           status;
      std::string                    error_USCOREmessage 0; // optional error_message
      std::vector<std::string>       origin_USCOREaddress;
      std::vector<std::string>       destination_USCOREaddress;
      std::vector<DistanceMatrixRow> row;
    };

The gSOAP naming conventions for XML data bindings translate each underscore in
XML to `_USCORE`, so we use `_USCORE` in place of an underscore in C++ names.

The `error_USCOREmessage` element is optional.  This can be made a
pointer-based member to make it optional (the pointer is NULL when the element
is omitted), but in this case we just declare it optional with a `0` minOccurs
constraint.

We define the `status` code for our `DistanceMatrixResponse` class as an
enumeration, corresponding to the Distance Matrix API's status codes:

    typedef enum DistanceMatrixStatus {
      DistanceMatrixStatus__OK,
      DistanceMatrixStatus__INVALID_USCOREREQUEST,
      DistanceMatrixStatus__MAX_USCOREELEMENTS_USCOREEXCEEDED,
      DistanceMatrixStatus__OVER_USCOREQUERY_USCORELIMIT,
      DistanceMatrixStatus__REQUEST_USCOREDENIED,
      DistanceMatrixStatus__UNKNOWN_USCOREERROR,
    } DistanceMatrixStatus;

Again, we used `_USCORE` to translate underscores to `_USCORE`.  Note that we
prefix the enumeration constants with the enumeration type name.  This ensures
that the enumeration constants are unique, while still being serialized in
their short form.  Much nicer is to use C++11 scoped enumerations that do not
require prefixing of the enumeration constants.  But we will stick to C++ here.

Alternatively, we could have just used a string for the type of `status`, but
enumerations are strongly typed, meaning the C++ code we will write later to
check the API's status code is statically verified to be correct by the
compiler.

### Rows

When the Google Maps Distance Matrix API returns results, it places them within
an array. XML responses consist of zero or more `row` elements.

Rows are ordered according to the values in the origin parameter of the
request. Each row corresponds to an origin, and each element within that row
corresponds to a pairing of the origin with a destination value.

Each row array contains one or more `element` entries, which in turn contain
the information about a single origin-destination pairing.

The container of rows `std::vector<DistanceMatrixRow>` type of the
`DistanceMatrixResponse::row` member uses the `DistanceMatrixRow` class, which
has a container of elements:

    class DistanceMatrixRow {
      std::vector<DistanceMatrixElement> element;
    };

### Elements

The information about each origin-destination pairing is returned in an
`element` entry.

The container of elements `std::vector<DistanceMatrixElement>` type of the
`DistanceMatrixRow::element` member uses the `DistanceMatrixElement` class:

    class DistanceMatrixElement {
      DistanceMatrixEltStatus status;
      DistanceMatrixDuration  duration;
      DistanceMatrixDuration *duration_USCOREin_USCOREtraffic; // optional duration_in_traffic
      DistanceMatrixDistance  distance;
      DistanceMatrixFare     *fare; // optional fare
    };

We define the `DistanceMatrixElement::status` as an enumeration, corresponding
to the API's status codes for elements:

    typedef enum DistanceMatrixEltStatus {
      DistanceMatrixEltStatus__OK,
      DistanceMatrixEltStatus__NOT_USCOREFOUND,
      DistanceMatrixEltStatus__ZERO_USCORERESULTS,
      DistanceMatrixEltStatus__MAX_USCOREROUTE_USCORELENGTH_USCOREEXCEEDED,
    } DistanceMatrixEltStatus;

Again, we could have just used a string for the type of `status`.  But
enumerations are preferred as strongly typed.  However, if a return value is
received that is not listed as an enumeration constant then XML parsing and
validation will fail with a validation error.  This is not a problem with the
Google Maps API status codes that are well defined.

Elements contain `duration` and `distance` elements, which are stored in
the `DistanceMatrixElement::duration` and `DistanceMatrixElement::distance`
members.  The types for these members are declared as:

    class DistanceMatrixDuration {
      double      value;
      std::string text;
    };

    class DistanceMatrixDistance {
      double      value;
      std::string text;
    };

A `fare` element may be optionally included with an element, which we define
as:

    typedef std::string DistanceMatrixCurrency 3:3;

    class DistanceMatrixFare {
      DistanceMatrixCurrency currency;
      double                 value;
      std::string            text;
    };

a currency designation is a three-character code as per ISO 4217.  Therefore,
the min and max length of `DistanceMatrixCurrency` is set to 3.

### The XML data binding interface file

We put this all together in a file `GoogleDistanceMatrix.hpp` to declare our
XML data binding interface:

    ////////////////////////////////////////////////////////////////////////////////
    //
    //  Elements
    //
    ////////////////////////////////////////////////////////////////////////////////

    typedef std::string DistanceMatrixCurrency 3:3;

    class DistanceMatrixDuration {
      double      value;
      std::string text;
    };

    class DistanceMatrixDistance {
      double      value;
      std::string text;
    };

    class DistanceMatrixFare {
      DistanceMatrixCurrency currency;
      double                 value;
      std::string            text;
    };

    typedef enum DistanceMatrixEltStatus {
      DistanceMatrixEltStatus__OK,
      DistanceMatrixEltStatus__NOT_USCOREFOUND,
      DistanceMatrixEltStatus__ZERO_USCORERESULTS,
      DistanceMatrixEltStatus__MAX_USCOREROUTE_USCORELENGTH_USCOREEXCEEDED,
    } DistanceMatrixEltStatus;

    class DistanceMatrixElement {
      DistanceMatrixEltStatus status;
      DistanceMatrixDuration  duration;
      DistanceMatrixDuration *duration_USCOREin_USCOREtraffic; // optional duration_in_traffic
      DistanceMatrixDistance  distance;
      DistanceMatrixFare     *fare; // optional fare
    };

    ////////////////////////////////////////////////////////////////////////////////
    //
    //  Rows
    //
    ////////////////////////////////////////////////////////////////////////////////

    class DistanceMatrixRow {
      std::vector<DistanceMatrixElement> element;
    };

    ////////////////////////////////////////////////////////////////////////////////
    //
    //  DistanceMatrixResponse
    //
    ////////////////////////////////////////////////////////////////////////////////

    typedef enum DistanceMatrixStatus {
      DistanceMatrixStatus__OK,
      DistanceMatrixStatus__INVALID_USCOREREQUEST,
      DistanceMatrixStatus__MAX_USCOREELEMENTS_USCOREEXCEEDED,
      DistanceMatrixStatus__OVER_USCOREQUERY_USCORELIMIT,
      DistanceMatrixStatus__REQUEST_USCOREDENIED,
      DistanceMatrixStatus__UNKNOWN_USCOREERROR,
    } DistanceMatrixStatus;

    class DistanceMatrixResponse {
      DistanceMatrixStatus           status;
      std::string                    error_USCOREmessage 0; // optional error_message
      std::vector<std::string>       origin_USCOREaddress;
      std::vector<std::string>       destination_USCOREaddress;
      std::vector<DistanceMatrixRow> row;
    };

Google Maps Directions API interface header file
------------------------------------------------

The `gdx` example client applications uses the Google Maps APIs with XML
responses.

For the full listing, see `GoogleDirections.hpp`.

Google Maps Distance Matrix API client source code
--------------------------------------------------

The main program constructs and destructs a context `ctx` as follows:

    // new context with option to parse UTF-8 into strings
    struct soap *ctx = soap_new1(SOAP_C_UTFSTRING);
    
    ...

    // delete deserialized response and free the context
    soap_destroy(ctx);
    soap_end(ctx);
    soap_free(ctx);

For the complete source code, please see `gdm.cpp`.

### Constructing the GET URL request

Constructing the URL for the GET request is done by collecting the command-line
arguments as follows:

    // construct URL with query string parameters
    string key = argv[1];
    string mode = argv[2];
    string units = argv[3];
    string origins = argv[4];
    string destinations = argv[5];
    string URL = service;
    URL += "?key=" + urlencode(key);
    URL += "&mode=" + urlencode(mode);
    URL += "&units=" + urlencode(units);
    URL += "&origins=" + urlencode(origins);
    URL += "&destinations=" + urlencode(destinations);

    // Google API permits up to 8K URL lengths
    if (URL.size() > 8192)
    {
      printf("URL too long\n\n");
      exit(EXIT_FAILURE);
    }

where function `urlencode` is defined as:

    static string urlencode(const string& plain)
    {
      string encoded;
      for (size_t i = 0; i < plain.size(); ++i)
      {
        int c = plain.at(i);
        if (c == ' ') 
        {
          encoded.push_back('+');
        }
        else if (c == '!'
              || c == '$'
              || (c >= '(' && c <= '.')
              || (c >= '0' && c <= '9')
              || (c >= 'A' && c <= 'Z')
              || c == '_'
              || (c >= 'a' && c <= 'z'))
        {
          encoded.push_back(c);
        }
        else
        {
          encoded.push_back('%');
          encoded.push_back((c >> 4) + (c > 159 ? '7' : '0'));
          c &= 0xF;
          encoded.push_back(c + (c > 9 ? '7' : '0'));
        }
      }
      return encoded;
    }

This creates a `URL` string that we will use to invoke the Google Maps API.

### Connecting to the Google Maps API

The source code establishes a HTTPS connection.  The server is not verified
unless you enable:

    if (soap_ssl_client_context(ctx, SOAP_SSL_DEFAULT, NULL, NULL, "cacerts.pem", NULL, NULL))
    {
      soap_stream_fault(ctx, cerr);
      exit(EXIT_FAILURE);
    }

When running the command-line application, a `cacerts.pem` file in the current
directory is required.  Or compile `plugin/cacerts.c` with hard-coded CA
certificates (and change the source code as needed).

We implement exponential backoff when HTTP 5xx errors occur, as recommended by
Google in the
[Best Practices Using Google Maps APIs](https://developers.google.com/maps/documentation/distance-matrix/web-service-best-practices#ParsingXML).

    // construct XML response object
    DistanceMatrixResponse response;

    // exponential backoff
    float delay = 0.100;    // initial 100 ms delay
    float delay_factor = 2; // delay doubles for each retry
    float max_delay = 120;  // 2 minutes max delay

    // GET XML response - exponential backoff on HTTP 5xx errors
    while (soap_GET_DistanceMatrixResponse(ctx, URL.c_str(), &response))
    {
      if (delay <= max_delay && ctx->error >= 500 && ctx->error < 599)
      {
        usleep((useconds_t)(1000 * delay));
        delay *= delay_factor;
      }
      else
      {
        soap_stream_fault(ctx, cerr);
        break;
      }
    }

After the connection is made, the `response` structure contains the Google Maps
API response parsed from XML.

All the hard XML parsing and deserialization work is done by the
`DistanceMatrixResponse` class and the auto-generated
`soap_GET_DistanceMatrixResponse` function for this class.

### Printing the response

To print the response we check for success and then iterate over the rows:

    // success?
    if (ctx-error == SOAP_OK)
    {
      if (response.status == DistanceMatrixStatus__OK)
      {
        cout << "----------------------------------------" << endl;
        // for each origin and destination pair, print duration and distance
        size_t n = minsize(response.origin_USCOREaddress.size(), response.row.size());
        for (size_t i = 0; i < n; ++i)
        {
          cout << "from: " << response.origin_USCOREaddress[i] << endl;
          size_t m = minsize(response.destination_USCOREaddress.size(), response.row[i].element.size());
          for (size_t j = 0; j < m; ++j)
          {
            cout << "to: " << response.destination_USCOREaddress[j] << endl;
            const DistanceMatrixElement& e = response.row[i].element[j];
            if (e.status == DistanceMatrixEltStatus__OK)
            {
              cout << "duration: " << e.duration.value << " (" << e.duration.text << ")" << endl;
              if (e.duration_USCOREin_USCOREtraffic)
                cout << "duration in traffic: " << e.duration_USCOREin_USCOREtraffic->value << " (" << e.duration_USCOREin_USCOREtraffic->text << ")" << endl;
              cout << "distance: " << e.distance.value << " (" << e.distance.text << ")" << endl;
              if (e.fare)
                cout << "fare: " << e.fare->currency << " " << e.fare->value << " (" << e.fare->text << ")" << endl;
            }
            else
            {
              cout << "error: " << soap_DistanceMatrixEltStatus2s(ctx, e.status) << endl;
            }
          }
          cout << "----------------------------------------" << endl;
        }
        cout << endl;
      }
      else
      {
        cerr <<
          "error: " << soap_DistanceMatrixStatus2s(ctx, response.status) <<
          " " << response.error_USCOREmessage << endl;
        if (response.status == DistanceMatrixStatus__UNKNOWN_USCOREERROR)
          cerr << "UNKNOWN ERROR: try again" << endl;
      }
    }

Google Maps Directions API client source code
---------------------------------------------

The `gdx.cpp` source code has a similar structure as `gdm.cpp`.

For the complete source code, please see `gdx.cpp`.

### Constructing the GET URL request

    // construct URL with query string parameters
    string key = argv[1];
    string mode = argv[2];
    string units = argv[3];
    string origin = argv[4];
    string destination = argv[5];
    string URL = service;
    URL += "?key=" + urlencode(key);
    URL += "&mode=" + urlencode(mode);
    URL += "&units=" + urlencode(units);
    URL += "&origin=" + urlencode(origin);
    URL += "&destination=" + urlencode(destination);

    // Google API permits up to 8K URL lengths
    if (URL.size() > 8192)
    {
      printf("URL too long\n\n");
      exit(EXIT_FAILURE);
    }

### Connecting to the Google Maps API

    // construct XML response object
    DirectionsResponse response;

    // exponential backoff
    float delay = 0.100;    // initial 100 ms delay
    float delay_factor = 2; // delay doubles for each retry
    float max_delay = 120;  // 2 minutes max delay

    // GET XML response - exponential backoff on HTTP 5xx errors
    while (soap_GET_DirectionsResponse(ctx, URL.c_str(), &response))
    {
      if (delay <= max_delay && ctx->error >= 500 && ctx->error < 599)
      {
	usleep((useconds_t)(1000 * delay));
	delay *= delay_factor;
      }
      else
      {
	soap_stream_fault(ctx, cerr);
	break;
      }
    }

### Printing the response

    // success?
    if (ctx->error == SOAP_OK)
    {
      if (response.status == DirectionsStatus__OK)
      {
	for (std::vector<std::string>::const_iterator i = response.available_USCOREtravel_USCOREmodes.begin(); i != response.available_USCOREtravel_USCOREmodes.end(); ++i)
	  cout << "available travel mode: " << *i << endl;
	for (std::vector<DirectionsRoute>::const_iterator i = response.route.begin(); i != response.route.end(); ++i)
	{
	  cout << "----------------------------------------" << endl;
	  cout << "summary: " << i->summary << endl;
	  for (std::vector<std::string>::const_iterator j = i->warning.begin(); j != i->warning.end(); ++j)
	    cout << "warning: " << *j << endl;
	  if (i->fare)
	    cout << "fare: " << i->fare->currency << " " << i->fare->value << " (" << i->fare->text << ")" << endl;
	  cout << "copyrights: " << i->copyrights << endl << endl;
	  for (std::vector<DirectionsLeg>::const_iterator j = i->leg.begin(); j != i->leg.end(); ++j)
	  {
	    cout << "from " << j->start_USCOREaddress << " to " << j->end_USCOREaddress << " takes";
	    if (j->distance)
	      cout << " " << j->distance->value << " meters (" << j->distance->text << ")";
	    if (j->duration)
	      cout << " " << j->duration->value << " seconds (" << j->duration->text << ")";
	    if (j->duration_USCOREin_USCOREtraffic)
	      cout << " " << j->duration_USCOREin_USCOREtraffic->value << " seconds in traffic (" << j->duration_USCOREin_USCOREtraffic->text << ")";
	    cout << endl << endl;
	    for (std::vector<DirectionsStep>::const_iterator k = j->step.begin(); k != j->step.end(); ++k)
	    {
	      if (!k->html_USCOREinstructions.empty())
		cout << k->html_USCOREinstructions << endl;
	      if (k->maneuver)
		cout << *k->maneuver << ", ";
	      cout << k->travel_USCOREmode;
	      if (k->distance)
		cout << " " << k->distance->value << " meters (" << k->distance->text << ")";
	      if (k->duration)
		cout << " " << k->duration->value << " seconds (" << k->duration->text << ")";
	      cout << endl;
	      for (std::vector<DirectionsStep>::const_iterator l = k->step.begin(); l != k->step.end(); ++l)
	      {
		if (!l->html_USCOREinstructions.empty())
		  cout << "  " << l->html_USCOREinstructions << endl;
		cout << "  => ";
		if (l->maneuver)
		  cout << *l->maneuver << ", ";
		cout << l->travel_USCOREmode;
		if (l->distance)
		  cout << " " << l->distance->value << " meters (" << l->distance->text << ")";
		if (l->duration)
		  cout << " " << l->duration->value << " seconds (" << l->duration->text << ")";
		cout << endl;
	      }
	      if (k->transit_USCOREdetails)
	      {
		DirectionsTransitDetails& d = *k->transit_USCOREdetails;
		cout << "  take transit at " << d.departure_USCOREtime.text << " from " << d.departure_USCOREstop.name << " to " << d.arrival_USCOREstop.name << " arriving at " << d.arrival_USCOREtime.text << ":" << endl;
		cout << "  => " << d.line.vehicle.name << " " << d.line.name << " line " << d.line.short_USCOREname << " heading " << d.headsign << " and exit at stop " << d.num_USCOREstops << endl;
		for (std::vector<DirectionsAgency>::const_iterator l = d.line.agency.begin(); l != d.line.agency.end(); ++l)
		   cout << "  (operated by " << l->name << " " << l->url << ") " << endl;
	      }
	      cout << endl;
	    }
	  }
	}
	cout << endl;
      }
      else
      {
	cerr <<
	  "error: " << soap_DirectionsStatus2s(ctx, response.status) <<
	  " " << response.error_USCOREmessage << endl;
	if (response.status == DirectionsStatus__UNKNOWN_USCOREERROR)
	  cerr << "UNKNOWN ERROR: try again" << endl;
      }
    }

Using a C++ namespace for the APIs
----------------------------------

The `gdm.hpp` interface file combines the Google Maps API interface header
files into one interface file and places all definitions within the `gdm` C++
namespace:

    namespace gdm {

    // add or remove APIs as needed
    #import "GoogleDistanceMatrix.hpp"
    #import "GoogleDirections.hpp"

    }

Note the use of `#import` instead of `#include`.  There are important
differences between these two constructs.  The soapcpp2 tool imports header
files with `#import`, while `#include` is copied to the soapcpp2-generated
header file so that C/C++ headers can be included in the generated source code.

Build steps
-----------

Because we use a C++ namespace in the `gdm.hpp` interface header file, we also
need a `env.h` file with global non-C++-namespaced definitions.  This is where
we normally put the SOAP Header and SOAP Fault details.  However, we do not use
SOAP so this file is simply empty.

To build the Google Maps Directions `gdx` command-line application:

    soapcpp2 -0 gdm.hpp
    soapcpp2 -0 -penv env.h
    c++ -DWITH_OPENSSL -o gdx gdx.cpp stdsoap2.cpp gdmC.cpp envC.cpp -lssl -lcrypto

To build the Google Maps Distance Matrix `gdm` command-line application:

    soapcpp2 -0 gdm.hpp
    soapcpp2 -0 -penv env.h
    c++ -DWITH_OPENSSL -o gdm gdm.cpp stdsoap2.cpp gdmC.cpp envC.cpp -lssl -lcrypto

Usage
-----

To use the Google Maps Directions `gdx` command-line application:

    gdx <YOUR_API_KEY> <mode> <units> 'origin address' 'destination address'

where `YOUR_API_KEY` is the API key you received earlier, mode is either
`driving`, `walking`, `bicycling`, or `transit`.  The units parameter should be
`metric` or `imperial`.  An origina address and a destination address should be
provided.  The addresses are URL-encoded by `gdx` before being passed to the
Google Maps Directions API.

To use the Google Maps Distance Matrix `gdm` command-line application:

    gdm <YOUR_API_KEY> <mode> <units> 'origin addresses' 'destination addresses'

where `YOUR_API_KEY` is the API key you received earlier, mode is either
`driving`, `walking`, `bicycling`, or `transit`.  The units parameter should be
`metric` or `imperial`.  Multiple addresses, when provided, should be separated
by `|`.  The addresses are URL-encoded by `gdm` before being passed to the
Google Maps Distance Matrix API.

For C developers
----------------

A `GoogleDistanceMatrix.h` interface file for C is included to build clients in
C.  However, no C application example is provided in source code.
