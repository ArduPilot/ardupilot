/*
	wsselite.c

	WS-Security lite demo application. See comments below.

gSOAP XML Web services tools
Copyright (C) 2000-2015, Robert van Engelen, Genivia Inc., All Rights Reserved.
This part of the software is released under one of the following licenses:
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

This application demonstrates the use of the wsse plugin.

Compile:

soapcpp2 -I import wssedemo.h
cc -o wsselite wsselite.c wsseapi.c stdsoap2.c soapC.c soapClient.c soapServer.c

Note:

The wsse.h, wsu.h, ds.h, xenc.h c14n.h files are located in 'import'.
The wsseapi-lite.* files are located in 'plugin'.

Usage: wsselite s [port]

with options:

s server (stand-alone)

To run a stand-alone server:

./wssedemo s 8080

And invoking it with a client:

./wssedemo c 8080

*/

#include "wsseapi-lite.h"
#include "wssetest.nsmap"

int main(int argc, char **argv)
{ struct soap *soap;
  int server = 0;
  int port = 0;
  double result;
  char *user;
  /* create context */
  soap = soap_new();
  /* options */
  if (argc >= 2)
  { if (strchr(argv[1], 'c'))
      soap_set_omode(soap, SOAP_IO_CHUNK);
    soap_set_omode(soap, SOAP_XML_INDENT);
    if (strchr(argv[1], 's'))
      server = 1;
  }
  /* soap->actor = "..."; */ /* set only when required */
  user = getenv("USER");
  if (!user)
    user = "anyone";
  /* port argument */
  if (argc >= 3)
    port = atoi(argv[2]);
  /* server or client/ */
  if (server)
  { if (port)
    { /* stand-alone server serving messages over port */
      if (!soap_valid_socket(soap_bind(soap, NULL, port, 100)))
      { soap_print_fault(soap, stderr);
        exit(1);
      }
      printf("Server started at port %d\n", port);
      while (soap_valid_socket(soap_accept(soap)))
      { if (soap_serve(soap))
        { soap_wsse_delete_Security(soap);
          soap_print_fault(soap, stderr);
          soap_print_fault_location(soap, stderr);
        }
	soap_destroy(soap);
	soap_end(soap);
      }
      soap_print_fault(soap, stderr);
      exit(1);
    }
    else
    { /* CGI-style server serving messages over stdin/out */
      if (soap_serve(soap))
      { soap_wsse_delete_Security(soap);
        soap_print_fault(soap, stderr);
        soap_print_fault_location(soap, stderr);
      }
      soap_destroy(soap);
      soap_end(soap);
    }
  }
  else /* client */
  { char endpoint[80];
    /* ns1:test data */
    struct ns1__add a;
    struct ns1__sub b;
    a.a = 123;
    a.b = 456;
    b.a = 789;
    b.b = -99999;
    /* client sending messages to stdout or over port */
    if (port)
      sprintf(endpoint, "http://localhost:%d", port);
    else
      strcpy(endpoint, "http://");

    /* message lifetime of 60 seconds */
    soap_wsse_add_Timestamp(soap, NULL, 60);
    /* add user name with text or digest password */
    soap_wsse_add_UsernameTokenText(soap, NULL, user, "userPass");
    /* invoke the server. You can choose add, sub, mul, or div operations
     * that show different security aspects (intentional message rejections)
     * for demonstration purposes (see server operations below) */
    if (!soap_call_ns1__add(soap, endpoint, NULL, 1.0, 2.0, &result))
    { if (!soap_wsse_verify_Timestamp(soap))
      { const char *servername = soap_wsse_get_Username(soap);
        if (servername
	 && !strcmp(servername, "server")
         && !soap_wsse_verify_Password(soap, "serverPass"))
          printf("Result = %g\n", result);
        else
	{ fprintf(stderr, "Server authentication failed\n");
          soap_print_fault(soap, stderr);
        }
      }
      else
      { fprintf(stderr, "Server response expired\n");
        soap_print_fault(soap, stderr);
      }
    }
    else
    { soap_print_fault(soap, stderr);
      soap_print_fault_location(soap, stderr);
    }
    /* clean up security header */
    soap_wsse_delete_Security(soap);
  }
  /* clean up gSOAP engine */
  soap_destroy(soap);
  soap_end(soap);
  soap_done(soap);
  free(soap);
  /* done */
  return 0;
}

int ns1__add(struct soap *soap, double a, double b, double *result)
{ const char *username = soap_wsse_get_Username(soap);
  if (username)
    fprintf(stderr, "Hello %s, want to add %g + %g = ?\n", username, a, b);
  if (soap_wsse_verify_Timestamp(soap)
   || soap_wsse_verify_Password(soap, "userPass"))
  { int err = soap->error; /* preserve error code */
    soap_wsse_delete_Security(soap); /* remove WS-Security information */
    return err;
  }
  soap_wsse_delete_Security(soap); /* remove WS-Security before setting new security information */
  soap_wsse_add_Timestamp(soap, NULL, 10);	/* lifetime of 10 seconds */
  soap_wsse_add_UsernameTokenText(soap, NULL, "server", "serverPass");
  *result = a + b;
  return SOAP_OK;
}

int ns1__sub(struct soap *soap, double a, double b, double *result)
{ const char *username = soap_wsse_get_Username(soap);
  if (username)
    fprintf(stderr, "Hello %s, want to subtract %g - %g = ?\n", username, a, b);
  if (soap_wsse_verify_Timestamp(soap)
   || soap_wsse_verify_Password(soap, "userPass"))
  { soap_wsse_delete_Security(soap);
    return soap->error;
  }
  soap_wsse_delete_Security(soap);
  /* In this case we leave out the timestamp, which is the sender's
   * responsibility to add. The receiver only complains if the timestamp is out
   * of date, not that it is absent. */
  *result = a - b;
  return SOAP_OK;
}

int ns1__mul(struct soap *soap, double a, double b, double *result)
{ const char *username = soap_wsse_get_Username(soap);
  if (username)
    fprintf(stderr, "Hello %s, want to multiply %g * %g = ?\n", username, a, b);
  if (soap_wsse_verify_Timestamp(soap)
   || soap_wsse_verify_Password(soap, "userPass"))
  { soap_wsse_delete_Security(soap);
    return soap->error;
  }
  soap_wsse_delete_Security(soap);
  soap_wsse_add_Timestamp(soap, NULL, 10);	/* lifetime of 10 seconds */
  /* In this case we leave out the server name and password. Because the
   * client receiver requires the presence of authentication information, the
   * client will reject the response. */
  *result = a * b;
  return SOAP_OK;
}

int ns1__div(struct soap *soap, double a, double b, double *result)
{ const char *username = soap_wsse_get_Username(soap);
  if (username)
    fprintf(stderr, "Hello %s, want to divide %g / %g = ?\n", username, a, b);
  if (soap_wsse_verify_Timestamp(soap)
   || soap_wsse_verify_Password(soap, "userPass"))
  { soap_wsse_delete_Security(soap);
    return soap->error;
  }
  soap_wsse_delete_Security(soap);
  soap_wsse_add_Timestamp(soap, NULL, 10);	/* lifetime of 10 seconds */
  soap_wsse_add_UsernameTokenText(soap, NULL, "server", "serverPass");
  /* In this case we leave out the signature and the receiver will reject this unsigned message. */
  if (b == 0.0)
    return soap_sender_fault(soap, "Division by zero", NULL);
  *result = a / b;
  return SOAP_OK;
}
