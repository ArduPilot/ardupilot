/*
	json-GitHub.c

	JSON GitHub API v3 (gSOAP C version)
	https://developer.github.com/v3/

	Compile:
	soapcpp2 -c -CSL xml-rpc.h
	cc -DWITH_OPENSSL -DWITH_GZIP -o json-GitHub json-GitHub.c xml-rpc.c json.c stdsoap2.c soapC.c -lcrypto -lssl -lz

	Usage:
	./json-GutHub URL [username password]

	Example:
	./json-GitHub https://api.github.com/orgs/Genivia/repos

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2000-2015, Robert van Engelen, Genivia, Inc. All Rights Reserved.
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

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    fprintf(stderr, "Usage: json-GitHub URL [username password]\nFor example: json-GitHub https://api.github.com/orgs/Genivia/repos\n\n");
    exit(1);
  }
  else
  {
    struct soap *ctx = soap_new1(SOAP_C_UTFSTRING | SOAP_XML_INDENT);
    struct value response;

    if (argc > 3)
    {
      /* Basic authentication with username password */
      if (strncmp(argv[1], "https", 5))
      {
	fprintf(stderr, "Basic authentication over http is not secure: use https\n");
	exit(1);
      }
      ctx->userid = argv[2];
      ctx->passwd = argv[3];
    }

    if (json_call(ctx, argv[1], NULL, &response))
      soap_print_fault(ctx, stderr);
    else
      json_write(ctx, &response);

    printf("\n\nOK\n");
    soap_end(ctx);
    soap_free(ctx);
  }
  return 0;
}

/* Don't need a namespace table. We put an empty one here to avoid link errors */
struct Namespace namespaces[] = { {NULL, NULL} };
