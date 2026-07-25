/*
        json-GoogleDistanceMatrix.cpp

        Google Maps Distance Matrix API example C++ client

        For more details, visit:

        https://developers.google.com/maps/documentation/distance-matrix/intro

        To get an API key:
        https://developers.google.com/maps/documentation/distance-matrix/get-api-key

        Build steps (requires code from samples/xml-rpc-json):
        soapcpp2 -CSL xml-rpc.h
	c++ -DWITH_OPENSSL -o json-gdm json-GoogleDistanceMatrix.cpp stdsoap2.cpp xml-rpc.cpp json.cpp soapC.cpp -lssl -lcrypto

        Usage:
        json-gdm <key> <mode> <units> 'origin addresses' 'destination addresses'
        where addresses should be separated by |

        Example:
        json-gdm <key> bicycling imperial 'Vancouver BC|Seattle' 'San Francisco|Vancouver BC'

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

#include "json.h"

using namespace std;

// define an empty XML namespace table (included to avoid link errors)
struct Namespace namespaces[] = { {NULL, NULL, NULL, NULL} };

// the Google Distance Matrix service endpoint
static const char *service = "https://maps.googleapis.com/maps/api/distancematrix/json";

// encode string in URL ("percentage") format
static string urlencode(const string& plain);

// inline min of size_t
inline size_t minsize(size_t a, size_t b) { return a < b ? a : b; }

int main(int argc, char **argv)
{
  if (argc <= 5)
  {
    printf("Usage: gdm key mode units 'origin addresses' 'destination addresses'\n\n");
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

  // construct JSON response object
  value response(ctx);

  // exponential backoff
  float delay = 0.100;    // initial 100 ms delay
  float delay_factor = 2; // delay doubles for each retry
  float max_delay = 120;  // 2 minutes max delay

  // GET JSON response - exponential backoff on HTTP 5xx errors
  while (json_call(ctx, URL.c_str(), NULL, response))
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
    if (response.has("status") && response.has("origin_addresses") && response.has("destination_addresses") && response.has("rows"))
    {
      if (!strcmp(response["status"], "OK"))
      {
        cout << "----------------------------------------" << endl;
        // for each origin and destination pair, print duration and distance
        size_t n = minsize(response["origin_addresses"].size(), response["rows"].size());
        for (size_t i = 0; i < n; ++i)
        {
          cout << "from: " << response["origin_addresses"][i] << endl;
          if (response["rows"][i].has("elements"))
          {
            size_t m = minsize(response["destination_addresses"].size(), response["rows"][i]["elements"].size());
            for (size_t j = 0; j < m; ++j)
            {
              cout << "to: " << response["destination_addresses"][j] << endl;
              value& e = response["rows"][i]["elements"][j];
              if (e.has("status") && !strcmp(e["status"], "OK"))
              {
                if (e.has("duration") && e["duration"].has("value") && e["duration"].has("text"))
                  cout << "duration: " << e["duration"]["value"] << " (" << e["duration"]["text"] << ")" << endl;
                if (e.has("duration_in_traffic") && e["duration_in_traffic"].has("value") && e["duration_in_traffic"].has("text"))
                  cout << "duration in traffic: " << e["duration_in_traffic"]["value"] << " (" << e["duration_in_traffic"]["text"] << ")" << endl;
                if (e.has("distance") && e["distance"].has("value") && e["distance"].has("text"))
                  cout << "distance: " << e["distance"]["value"] << " (" << e["distance"]["text"] << ")" << endl;
                if (e.has("fare") && e["fare"].has("currency") && e["fare"].has("value") && e["fare"].has("text"))
                  cout << "fare: " << e["fare"]["currency"] << e["fare"]["value"] << " (" << e["fare"]["text"] << ")" << endl;
              }
              else
              {
                cout << "error: ";
                if (e.has("status"))
                  cout << e["status"];
                cout << endl;
              }
              cout << "----------------------------------------" << endl;
            }
          }
        }
      }
      else
      {
        cerr << "error: ";
        if (response.has("status"))
          cerr << response["status"];
        if (response.has("error_message"))
          cerr << " " << response["error_message"];
        cerr << endl;
      }
    }
  }
  else
  {
    soap_stream_fault(ctx, cerr);
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
