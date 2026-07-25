/*
        gdx.cpp

        Google Maps Directions API example C++ client

        For more details, visit:

        https://developers.google.com/maps/documentation/directions/intro

        To get an API key:
        https://developers.google.com/maps/documentation/directions/get-api-key

        Build steps:
        soapcpp2 -0 gdm.hpp
        soapcpp2 -0 -penv env.h
        c++ -DWITH_OPENSSL -o gdx gdx.cpp stdsoap2.cpp gdmC.cpp envC.cpp -lssl -lcrypto

        Usage:
        gdx <key> <mode> <units> 'origin address' 'destination address'

        Example:
        gdx <key> bicycling imperial Chicago,IL' 'Los Angeles,CA'

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

#include <unistd.h> // usleep
#include "gdmH.h"

using namespace std;
using namespace gdm;

// define an empty XML namespace table (included to avoid link errors)
struct Namespace namespaces[] = { {NULL, NULL, NULL, NULL} };

// the Google Distance Matrix service endpoint
static const char *service = "https://maps.googleapis.com/maps/api/directions/xml";

// encode string in URL ("percentage") format
static string urlencode(const string& plain);

// inline min of size_t
inline size_t minsize(size_t a, size_t b) { return a < b ? a : b; }

int main(int argc, char **argv)
{
  if (argc <= 5)
  {
    printf("Usage: gdx key mode units 'origin address' 'destination address'\n\n");
    exit(EXIT_FAILURE);
  }

  // new context with option to parse UTF-8 into strings
  struct soap *ctx = soap_new1(SOAP_C_UTFSTRING);

  // to verify the server's authenticity, enable the code below and place cacerts.pem in the current dir:
#if 0
  // use SOAP_SSL_SKIP_HOST_CHECK in place of SOAP_SSL_DEFAULT to omit the host check if host mismatches occur (use at own risk!) 
  if (soap_ssl_client_context(ctx, SOAP_SSL_DEFAULT, NULL, NULL, "cacerts.pem", NULL, NULL))
  {
    soap_stream_fault(ctx, cerr);
    exit(EXIT_FAILURE);
  }
#endif

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

  // delete deserialized response and free the context
  soap_destroy(ctx);
  soap_end(ctx);
  soap_free(ctx);

  exit(EXIT_SUCCESS);
}

// encode string in URL "percentage" format
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

