/*
        mtom-test.c

        This application includes a MTOM test client and server. As a client
        application, it fires four different base64 or MTOM attachments to the
        server. As a server, it will respond to the messages by converting
        base64 into MTOM attachments and vice versa.

        Usage (server):
        $ mtom <port>

        Usage (client):
        $ mtom http://localhost:<port> "<message1>" "<message2>" "<message3>" ...

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2000-2008, Robert van Engelen, Genivia, Inc. All Rights Reserved.
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

#include "soapH.h"
#include "mtom_test.nsmap"

int cgi_serve();
int run_serve(int port);
int run_tests(int, char**);

int main(int argc, char **argv)
{ if (argc < 2)
    return cgi_serve();
  if (argc < 3)
    return run_serve(atoi(argv[1]));
  return run_tests(argc, argv);
}

int cgi_serve()
{ /* CGI-style: serve request from stdin */
  /* enable MTOM XOP attachments (and XML indent) */
  return soap_serve(soap_new1(SOAP_XML_INDENT | SOAP_ENC_MTOM));
}

int run_serve(int port)
{ struct soap *soap = soap_new1(SOAP_ENC_MTOM); /* enable MTOM */
  int ret;
  if (!soap_valid_socket(soap_bind(soap, NULL, port, 100)))
    soap_print_fault(soap, stderr);
  else
  { fprintf(stderr, "Bind to port %d successful\n", port);
    soap->accept_timeout = 3600; /* let server time out after one hour */
    for (;;)
    { int sock = soap_accept(soap);
      if (!soap_valid_socket(sock))
      { if (soap->errnum)
          soap_print_fault(soap, stderr);
        else
          fprintf(stderr, "Server timed out\n");
        break;
      }
      fprintf(stderr, "Accepting socket %d connection from IP %d.%d.%d.%d... ", sock, (int)(soap->ip>>24)&0xFF, (int)(soap->ip>>16)&0xFF, (int)(soap->ip>>8)&0xFF, (int)soap->ip&0xFF);
      if (soap_serve(soap))
        soap_print_fault(soap, stderr);
      fprintf(stderr, "done\n");
      soap_destroy(soap);
      soap_end(soap);
    } 
  }
  ret = soap->error;
  soap_free(soap); /* done and free */
  return ret;
}

int run_tests(int argc, char **argv)
{ struct soap *soap = soap_new1(SOAP_ENC_MTOM); /* enable MTOM */
  int i, ret;
  struct x__DataType data;
  struct x__WrapperType wrap;
  struct m__EchoTestSingleResponse single;
  struct m__EchoTestMultipleResponse multiple;
  soap_default_x__DataType(soap, &data);
  soap_default_x__WrapperType(soap, &wrap);

  /* First test call */
  data.__union = SOAP_UNION_x__data_base64;
  data.choice.base64.__ptr = (unsigned char*)argv[2];
  data.choice.base64.__size = (int)strlen(argv[2]) + 1;
  if (soap_call_m__EchoTestSingle(soap, argv[1], NULL, &data, &single))
  { soap_print_fault(soap, stderr);
  }
  else
  { if (!single.x__Data || single.x__Data->__union != SOAP_UNION_x__data_xop__Include || !single.x__Data->choice.xop__Include.__ptr || single.x__Data->choice.xop__Include.__size != data.choice.base64.__size || strcmp((char*)single.x__Data->choice.xop__Include.__ptr, (char*)data.choice.base64.__ptr))
      fprintf(stderr, "EchoTestSingle 1: data transcription error\n");
    else
      fprintf(stderr, "EchoTestSingle 1: OK\n");
    /* Second test call */
    data.__union = SOAP_UNION_x__data_xop__Include;
    data.choice.xop__Include.__ptr = (unsigned char*)argv[2];
    data.choice.xop__Include.__size = (int)strlen(argv[2]) + 1;
    data.choice.xop__Include.id = NULL;
    data.choice.xop__Include.type = "text/xml";
    data.choice.xop__Include.options = NULL;
    data.xmime5__contentType = "text/xml";
#ifdef WITH_NOIDREF
    /* compiling with WITH_NOIDREF removes auto-detection of attachments */
    soap_set_mime(soap, NULL, NULL); /* so we explicitly set MIME attachments */
#endif
    if (soap_call_m__EchoTestSingle(soap, argv[1], NULL, &data, &single))
    { soap_print_fault(soap, stderr);
    }
    else
    { if (!single.x__Data
       || single.x__Data->__union != SOAP_UNION_x__data_base64
       || !single.x__Data->choice.base64.__ptr
       || single.x__Data->choice.base64.__size != data.choice.xop__Include.__size
       || strcmp((char*)single.x__Data->choice.base64.__ptr, (char*)data.choice.xop__Include.__ptr))
        fprintf(stderr, "EchoTestSingle 2: data transcription error\n");
      else
      { fprintf(stderr, "EchoTestSingle 2: OK\n");
        /* Third test call */
        wrap.__size = argc - 2;
        wrap.Data = (struct x__DataType*)soap_malloc(soap, wrap.__size * sizeof(struct x__DataType));
        for (i = 0; i < wrap.__size; ++i)
        { soap_default_x__DataType(soap, &wrap.Data[i]);
          wrap.Data[i].__union = SOAP_UNION_x__data_base64;
          wrap.Data[i].choice.base64.__ptr = (unsigned char*)argv[i + 2];
          wrap.Data[i].choice.base64.__size = (int)strlen(argv[i + 2]) + 1;
          wrap.Data[i].xmime5__contentType = "text/xml";
        }
        if (soap_call_m__EchoTestMultiple(soap, argv[1], NULL, &wrap, &multiple))
          soap_print_fault(soap, stderr);
        else
        { int okay = 1;
          if (!multiple.x__EchoTest
           || multiple.x__EchoTest->__size != wrap.__size)
            okay = 0;
          else
          { for (i = 0; i < multiple.x__EchoTest->__size; ++i)
              if (multiple.x__EchoTest->Data[i].__union != SOAP_UNION_x__data_xop__Include
               || !multiple.x__EchoTest->Data[i].choice.xop__Include.__ptr
               || multiple.x__EchoTest->Data[i].choice.xop__Include.__size != wrap.Data[i].choice.base64.__size
               || strcmp((char*)multiple.x__EchoTest->Data[i].choice.xop__Include.__ptr, (char*)wrap.Data[i].choice.base64.__ptr))
                okay = 0;
          }
          if (!okay)
            fprintf(stderr, "EchoTestMultiple 1: data transcription error\n");
          else
          { fprintf(stderr, "EchoTestMultiple 1: OK\n");
            /* Fourth test call */
            for (i = 0; i < wrap.__size; ++i)
            { soap_default_x__DataType(soap, &wrap.Data[i]);
              wrap.Data[i].__union = SOAP_UNION_x__data_xop__Include;
              wrap.Data[i].choice.xop__Include.__ptr = (unsigned char*)argv[i + 2];
              wrap.Data[i].choice.xop__Include.__size = (int)strlen(argv[i + 2]) + 1;
              wrap.Data[i].choice.xop__Include.id = NULL;
              wrap.Data[i].choice.xop__Include.type = "text/xml";
              wrap.Data[i].choice.xop__Include.options = NULL;
              wrap.Data[i].xmime5__contentType = "text/xml";
            }
            if (soap_call_m__EchoTestMultiple(soap, argv[1], NULL, &wrap, &multiple))
              soap_print_fault(soap, stderr);
            else
            { int okay = 1;
              if (!multiple.x__EchoTest
               || multiple.x__EchoTest->__size != wrap.__size)
                okay = 0;
              else
              { for (i = 0; i < multiple.x__EchoTest->__size; ++i)
                  if (multiple.x__EchoTest->Data[i].__union != SOAP_UNION_x__data_base64
                   || !multiple.x__EchoTest->Data[i].choice.base64.__ptr
                   || multiple.x__EchoTest->Data[i].choice.base64.__size != wrap.Data[i].choice.xop__Include.__size || strcmp((char*)multiple.x__EchoTest->Data[i].choice.base64.__ptr, (char*)wrap.Data[i].choice.xop__Include.__ptr))
                    okay = 0;
              }
              if (!okay)
                fprintf(stderr, "EchoTestMultiple 2: data transcription error\n");
              else
                fprintf(stderr, "EchoTestMultiple 2: OK\n");
            }
          }
        }
      }
    }
  }
  ret = soap->error;
  soap_destroy(soap);
  soap_end(soap);
  soap_done(soap);
  free(soap);
  return ret;
}

int m__EchoTestSingle(struct soap *soap, struct x__DataType *data, struct m__EchoTestSingleResponse *response)
{ if (!data)
  { /* To return a fault in an MTOM attachment:
       soap_set_mode(soap, SOAP_ENC_MTOM | SOAP_ENC_MIME); */
    return soap_sender_fault(soap, "No data", NULL);
  }
  /* allocate response */
  response->x__Data = (struct x__DataType*)soap_malloc(soap, sizeof(struct x__DataType));
  if (!response->x__Data)
    return SOAP_EOM;
  /* copy data into response, switching from base64 to MTOM and vice versa */
  switch (data->__union)
  { case SOAP_UNION_x__data_xop__Include:
      /* convert MTOM attachment to base64Binary */
      response->x__Data->__union = SOAP_UNION_x__data_base64;
      response->x__Data->choice.base64.__ptr = data->choice.xop__Include.__ptr;
      response->x__Data->choice.base64.__size = data->choice.xop__Include.__size;
      response->x__Data->xmime5__contentType = data->choice.xop__Include.type;
      break;
    case SOAP_UNION_x__data_base64:
      /* set MTOM/MIME attachments, if context is not already initialized with SOAP_ENC_MTOM */
      soap_set_mode(soap, SOAP_ENC_MTOM | SOAP_ENC_MIME);
      /* convert base64Binary to MTOM attachment */
      response->x__Data->__union = SOAP_UNION_x__data_xop__Include;
      response->x__Data->choice.xop__Include.__ptr = data->choice.base64.__ptr;
      response->x__Data->choice.xop__Include.__size = data->choice.base64.__size;
      response->x__Data->choice.xop__Include.id = NULL;
      response->x__Data->choice.xop__Include.type = data->xmime5__contentType;
      response->x__Data->choice.xop__Include.options = NULL;
      response->x__Data->xmime5__contentType = data->xmime5__contentType;
#ifdef WITH_NOIDREF
      /* compiling with WITH_NOIDREF removes auto-detection of attachments */
      soap_set_mime(soap, NULL, NULL); /* so we explicitly set MIME attachments */
#endif
      break;
    default:
      return soap_sender_fault(soap, "Wrong data format", NULL);
  }
  return SOAP_OK;
}

int m__EchoTestMultiple(struct soap *soap, struct x__WrapperType *x__EchoTest, struct m__EchoTestMultipleResponse *response)
{ int i;
  if (!x__EchoTest)
    return soap_sender_fault(soap, "No data", NULL);
  /* allocate response */
  response->x__EchoTest = (struct x__WrapperType*)soap_malloc(soap, sizeof(struct x__WrapperType));
  if (!response->x__EchoTest)
    return SOAP_EOM;
  response->x__EchoTest->__size = x__EchoTest->__size;
  response->x__EchoTest->Data = (struct x__DataType*)soap_malloc(soap, sizeof(struct x__DataType) * x__EchoTest->__size);
  if (!response->x__EchoTest->Data)
    return SOAP_EOM;
  /* copy data into response, switching from base64 to MTOM and vice versa */
  for (i = 0; i < x__EchoTest->__size; ++i)
  { switch (x__EchoTest->Data[i].__union)
    { case SOAP_UNION_x__data_xop__Include:
        /* convert MTOM attachment to base64Binary */
        response->x__EchoTest->Data[i].__union = SOAP_UNION_x__data_base64;
        response->x__EchoTest->Data[i].choice.base64.__ptr = x__EchoTest->Data[i].choice.xop__Include.__ptr;
        response->x__EchoTest->Data[i].choice.base64.__size = x__EchoTest->Data[i].choice.xop__Include.__size;
        response->x__EchoTest->Data[i].xmime5__contentType = x__EchoTest->Data[i].choice.xop__Include.type;
        break;
      case SOAP_UNION_x__data_base64:
        /* set MTOM/MIME attachments, if context is not already initialized with SOAP_ENC_MTOM */
        soap_set_mode(soap, SOAP_ENC_MTOM | SOAP_ENC_MIME); /* set MTOM/MIME attachments */
        /* convert base64Binary to MTOM attachment */
        response->x__EchoTest->Data[i].__union = SOAP_UNION_x__data_xop__Include;
        response->x__EchoTest->Data[i].choice.xop__Include.__ptr = x__EchoTest->Data[i].choice.base64.__ptr;
        response->x__EchoTest->Data[i].choice.xop__Include.__size = x__EchoTest->Data[i].choice.base64.__size;
        response->x__EchoTest->Data[i].choice.xop__Include.id = NULL;
        response->x__EchoTest->Data[i].choice.xop__Include.type = x__EchoTest->Data[i].xmime5__contentType;
        response->x__EchoTest->Data[i].choice.xop__Include.options = NULL;
        response->x__EchoTest->Data[i].xmime5__contentType = x__EchoTest->Data[i].xmime5__contentType;
#ifdef WITH_NOIDREF
        /* compiling with WITH_NOIDREF removes auto-detection of attachments */
        soap_set_mime(soap, NULL, NULL); /* so we explicitly set MIME attachments */
#endif
        break;
      default:
        return soap_sender_fault(soap, "Wrong data format", NULL);
    }
  }
  return SOAP_OK;
}
