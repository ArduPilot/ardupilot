/*
	calculator.cpp

	WCF wsDualHttpBinding demo

	See the README.txt

gSOAP XML Web services tools
Copyright (C) 2000-2012, Robert van Engelen, Genivia Inc., All Rights Reserved.
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

This program is released under the GPL with the additional exemption that
compiling, linking, and/or using OpenSSL is allowed.
--------------------------------------------------------------------------------
A commercial use license is available from Genivia, Inc., contact@genivia.com
--------------------------------------------------------------------------------
*/

/*
	To build with C++ service objects (proxies, services, and callback
	service objects), run soapcpp2 twice, first for the WS-RM logic and
	then for the app logic:
	$ soapcpp2 -L -I../../../../import -I../../../.. -A -pwsrx ../../../../import/wsrm5.h
	$ soapcpp2 -L -I../../../../import -I../../../.. -a -j calculator.h
	then compile:
	$ c++ -o calculator calculator.cpp soapC.cpp soapWSDualHttpBinding_USCOREICalculatorDuplexService.cpp soapWSDualHttpBinding_USCOREICalculatorDuplexProxy.cpp wsrxClient.cpp wsrxServer.cpp ../../../../plugin/wsrmapi.c ../../../../plugin/wsaapi.c ../../../../custom/duration.c ../../../../dom.cpp ../../../../stdsoap2.cpp

	Run service on port 8000:
	$ ./calculator 8000

	Run client:
	$ ./calculator

	Note:
	Set serverURI to "http://localhost:8000" and clientURI to
	"http://localhost:8001" in the code below to run the client and server
	on the same host.

	See README.txt for custimizations and instructions for WCF.

	When comppiled with -DCB_THREAD, the client will start a callback
	server to accept incoming messages instead of using a polling method.
*/

#include "soapWSDualHttpBinding_USCOREICalculatorDuplexService.h"
#include "soapWSDualHttpBinding_USCOREICalculatorDuplexProxy.h"
#include "WSDualHttpBinding_USCOREICalculatorDuplex.nsmap"

// Set to the service URI:
const char *serverURI = "http://localhost:8000";
// const char *serverURI = "http://192.168.2.2:8000/ServiceModelSamples/service";

// Set to the client callback URI and port:
const char *clientURI = "http://localhost:8001";
// const char *clientURI = "http://10.0.1.3:8001";
int clientPort = 8001;

#include <sstream>
#include "wsrmapi.h"

#ifdef CB_THREAD
void *callback_server(void*);
void *callback_thread(void *ctx);
#define CB_WAIT (1) /* threaded callback service operation may block for up to 1s with NoDiscard */
#else
#define CB_WAIT (0) /* callback polling service operations should not block with NoDiscard */
#endif

class Service : public WSDualHttpBinding_USCOREICalculatorDuplexService
{
  private:
    double result;
    std::stringstream equation;
    WSDualHttpBinding_USCOREICalculatorDuplexProxy callback;

  public:
    Service()
    {
      soap_set_mode(soap, SOAP_XML_INDENT);
      soap_register_plugin(soap, soap_wsa);
      soap_register_plugin(soap, soap_wsrm);
      soap->send_timeout = soap->recv_timeout = 10; // 10 sec

      soap_set_mode(callback.soap, SOAP_XML_INDENT);
      soap_register_plugin(callback.soap, soap_wsa);
      soap_register_plugin(callback.soap, soap_wsrm);
      callback.soap->send_timeout = callback.soap->recv_timeout = 10; // 10 sec

      result = 0.0;
      equation.str("");
      equation << 0.0;
    }
    ~Service()
    {
      destroy();
      callback.destroy();
    }
    int start(int port);
    int Clear(_mssadh__Clear*);
    int AddTo(_mssadh__AddTo*);
    int SubtractFrom(_mssadh__SubtractFrom*);
    int MultiplyBy(_mssadh__MultiplyBy*);
    int DivideBy(_mssadh__DivideBy*);
};

class Client : public WSDualHttpBinding_USCOREICalculatorDuplexProxy
{
  private:
    WSDualHttpBinding_USCOREICalculatorDuplexService callback;

  public:
#ifdef CB_THREAD
    THREAD_TYPE tid; // callback service thread
#endif
    Client(const char *serverURI)
    {
      soap_set_mode(soap, SOAP_XML_INDENT);
      soap_register_plugin(soap, soap_wsa);
      soap_register_plugin(soap, soap_wsrm);
      soap_endpoint = serverURI;
      soap->send_timeout = soap->recv_timeout = 10; // 10 sec

      soap_set_mode(callback.soap, SOAP_XML_INDENT);
      soap_register_plugin(callback.soap, soap_wsa);
      soap_register_plugin(callback.soap, soap_wsrm);
      callback.soap->send_timeout = callback.soap->recv_timeout = 10; // 10 sec

      callback.soap->bind_flags = SO_REUSEADDR; // allow immediate bind

      if (!soap_valid_socket(callback.bind(NULL, clientPort, 100)))
      {
        callback.soap_stream_fault(std::cerr);
        exit(1);
      }
#ifdef CB_THREAD
      THREAD_CREATE(&tid, (void*(*)(void*))callback_server, (void*)&callback);
#endif
    }
    ~Client()
    {
      destroy();
#ifdef CB_THREAD
      THREAD_JOIN(tid);
#endif
      callback.destroy();
    }
    int run();
    int poll(int timeout);
};

int main(int argc, char **argv)
{
  // Command line argument? If yes, then run the service on a port
  if (argc >= 2)
  {
    int port = atoi(argv[1]);
    Service service;
    service.start(port);
  }
  else
  {
    Client client(serverURI);
    client.run();
  }
}

int Client::run()
{
  // Create a reliable messaging sequence handle for client-initiated session
  soap_wsrm_sequence_handle seq;

  xsd__duration expires = 30000; /* 30000 ms = 30 seconds to expire */
  const char *id = soap_wsa_rand_uuid(soap);
  double n;
  int retry;

  printf("\n**** Creating the Sequence\n");

  // Create session for in-order messaging, init sequence handle
  if (soap_wsrm_create_offer(soap, serverURI, clientURI, id, expires, NoDiscard, soap_wsa_rand_uuid(soap), &seq))
  {
    soap_stream_fault(std::cerr);
    soap_wsrm_seq_free(soap, seq);
    return soap->error;
  }

  // poll 100 times for .1 second until created
  for (retry = 100; retry && !soap_wsrm_seq_created(soap, seq); retry--)
  {
    if (poll(-100000))
      return soap->error;
  }
  if (!retry)
  {
    fprintf(stderr, "CANNOT CREATE SEQUENCE - SERVER NOT RESPONDING\n");
    exit(1);
  }

  // Reliable messaging request message
  if (soap_wsrm_request_acks(soap, seq, soap_wsa_rand_uuid(soap), "http://Microsoft.Samples.DualHttp/ICalculatorDuplex/AddTo"))
  {
    soap_stream_fault(std::cerr);
    return soap->error;
  }

  n = 3.14;

  _mssadh__AddTo addTo;
  addTo.n = &n;
  if (AddTo(&addTo) == SOAP_OK)
    std::cout << std::endl << "**** AddTo(" << *addTo.n << ")" << std::endl;
  else
    soap_stream_fault(std::cerr);
  destroy();

#ifndef CB_THREAD
  // callback polling: 500 ms polling cycle
  if (poll(-500000))
    return soap->error;
#endif

  // Reliable messaging request message
  if (soap_wsrm_request_acks(soap, seq, soap_wsa_rand_uuid(soap), "http://Microsoft.Samples.DualHttp/ICalculatorDuplex/SubtractFrom"))
  {
    soap_stream_fault(std::cerr);
    return soap->error;
  }

  n = 1.41;

  _mssadh__SubtractFrom subtractFrom;
  subtractFrom.n = &n;
  if (SubtractFrom(&subtractFrom) == SOAP_OK)
    std::cout << std::endl << "**** SubtractFrom(" << *subtractFrom.n << ")" << std::endl;
  else
    soap_stream_fault(std::cerr);
  destroy();

#ifndef CB_THREAD
  // callback polling: 500 ms polling cycle
  if (poll(-500000))
    return soap->error;
#endif

  // Reliable messaging request message
  if (soap_wsrm_request_acks(soap, seq, soap_wsa_rand_uuid(soap), "http://Microsoft.Samples.DualHttp/ICalculatorDuplex/Clear"))
  {
    soap_stream_fault(std::cerr);
    return soap->error;
  }

  _mssadh__Clear clear;
  if (Clear(&clear) == SOAP_OK)
    std::cout << std::endl << "**** Clear()" << std::endl;
  else
    soap_stream_fault(std::cerr);
  destroy();

#ifndef CB_THREAD
  // callback polling: 500 ms polling cycle
  if (poll(-500000))
    return soap->error;
#endif

  printf("\n**** Closing the Sequence\n");

  if (soap_wsrm_close(soap, seq, soap_wsa_rand_uuid(soap)))
  {
    soap_stream_fault(std::cerr);
    soap_wsrm_seq_free(soap, seq);
    return soap->error;
  }

  for (retry = 30; retry; retry--)
  {
    // Receive more messages when last not yet received
    if (soap_wsrm_lastnum(seq) == 0)
    {
#ifdef CB_THREAD
      // we want to pulse here to send acks for incoming messages to keep flow going
      soap_wsrm_pulse(soap, -500000); /* 500 ms */
#endif
      printf("\n**** Receiving Messages Until Last\n");

      // callback polling or delay: 500 ms polling cycle
      if (poll(-500000))
        return soap->error;
    }

    // Resend messages marked as non-acked (as an option)
    if (soap_wsrm_nack(seq))
    {
      printf("\n**** Resending "SOAP_ULONG_FORMAT" Non-Acked Messages\n", soap_wsrm_nack(seq));
      soap_wsrm_resend(soap, seq, 0, 0); /* 0 0 means full range of msg nums */
    }
  }

  if (soap_wsrm_nack(seq) || soap_wsrm_lastnum(seq) == 0)
  {
    printf("\n**** Incomplete Sequence\n");
    soap_wsrm_dump(soap, stdout);
  }

  printf("\n**** Terminating the Sequence\n");

  // Termination fails if the server did not get all messages
  if (soap_wsrm_terminate(soap, seq, soap_wsa_rand_uuid(soap)))
  {
    soap_stream_fault(std::cerr);
    soap_wsrm_seq_free(soap, seq);
    return soap->error;
  }

#ifdef CB_THREAD

  soap_wsrm_dump(soap, stdout);

  printf("\n**** Waiting for Callback Server to Terminate\n");

#endif

  // callback polling: 1 s polling cycle
  if (poll(1))
    return soap->error;

  // Delete the reliable messaging session sequence (assuming no more inbound messages)
  soap_wsrm_seq_free(soap, seq);

  return 0;
}

/******************************************************************************\
 *
 *	Client-side callback polling
 *
\******************************************************************************/

int Client::poll(int timeout)
{
#ifdef CB_THREAD

  // We leave the acceptance of messages to the callback server thread
  if (timeout < 0)
    timeout = 1;
  sleep(timeout); // but we want to wait some until these messages arrive

#else

  callback.soap->accept_timeout = timeout;

  printf("\n**** Callback Polling\n");

  while (soap_valid_socket(callback.accept()))
  {
    /* chain the WSRM operations after callback operations */
    if (soap_begin_serve(callback.soap) == SOAP_OK)
      if (callback.dispatch() == SOAP_NO_METHOD)
        soap_serve_request(callback.soap);
    if (callback.soap->error)
      soap_send_fault(callback.soap);

    soap->error = callback.soap->error;

    callback.destroy();

    if (soap->error && soap->error != SOAP_STOP)
    {
      soap_stream_fault(std::cerr);
      return soap->error;
    }
    soap_wsrm_dump(callback.soap, stdout);
  }
#endif

  return SOAP_OK;
}

/******************************************************************************\
 *
 *	Optional Client-Side Threaded Callback Server
 *
\******************************************************************************/

#ifdef CB_THREAD
void *callback_server(void *ctx)
{
  WSDualHttpBinding_USCOREICalculatorDuplexService *callback = (WSDualHttpBinding_USCOREICalculatorDuplexService*)ctx;
  THREAD_TYPE tid;

  callback->soap->accept_timeout = 30; /* server quits after 30 seconds of inactivity */

  printf("\n**** Callback Server Running\n");

  while (soap_valid_socket(callback->accept()))
    THREAD_CREATE(&tid, (void*(*)(void*))callback_thread, (void*)callback->copy());

  printf("\n**** Callback Server Terminated\n");

  return NULL;
}

void *callback_thread(void *ctx)
{
  WSDualHttpBinding_USCOREICalculatorDuplexService *callback = (WSDualHttpBinding_USCOREICalculatorDuplexService*)ctx;
  THREAD_DETACH(THREAD_ID);

  /* chain the WSRM operations after callback operations */
  if (soap_begin_serve(callback->soap) == SOAP_OK)
    if (callback->dispatch() == SOAP_NO_METHOD)
      soap_serve_request(callback->soap);
  if (callback->soap->error)
    soap_send_fault(callback->soap);

  if (callback->soap->error != SOAP_STOP && callback->soap->error != SOAP_EOF)
    callback->soap_stream_fault(std::cerr);
  else if (callback->soap->error != SOAP_EOF || callback->soap->errnum)
    soap_wsrm_dump(callback->soap, stdout);

  callback->destroy();
  delete callback;

  return NULL;
}
#endif

/******************************************************************************\
 *
 *	Client-side callback operations via callback polling service
 *
\******************************************************************************/

int WSDualHttpBinding_USCOREICalculatorDuplexService::Result(_mssadh__Result *req)
{
  /* check for WS-RM/WSA, send empty response (this is a one-way operation over HTTP) */
  if (soap_wsrm_check_send_empty_response_and_wait(soap, CB_WAIT))
    return soap->error;

  if (req->result)
    std::cout << std::endl << "**** Result(" << *req->result << ")" << std::endl;

  return SOAP_OK;
}

int WSDualHttpBinding_USCOREICalculatorDuplexService::Equation(_mssadh__Equation *req)
{
  /* check for WS-RM/WSA, send empty response (this is a one-way operation over HTTP) */
  if (soap_wsrm_check_send_empty_response_and_wait(soap, CB_WAIT))
    return soap->error;

  if (req->eqn)
    std::cout << std::endl << "**** Equation(" << *req->eqn << ")" << std::endl;

  return SOAP_OK;
}

/******************************************************************************\
 *
 *	Service operations
 *
\******************************************************************************/

int Service::start(int port)
{
  if (!soap_valid_socket(bind(NULL, port, 100)))
  {
    soap_stream_fault(std::cerr);
    exit(1);
  }
  std::cerr << "Server Running" << std::endl;

  /* optional: set accept timeout to pulse acks every 500 ms, see below */
  soap->accept_timeout = -500000;

  for (;;)
  {
    if (soap_valid_socket(accept()))
    { 
      /* with iterative servers asynchronous messaging deadlock scenarios exist! */
      /* chain the WSRM service operations after the main service operations */
      if (soap_begin_serve(soap) == SOAP_OK)
        if (dispatch() == SOAP_NO_METHOD)
          soap_serve_request(soap);
      if (soap->error)
        soap_send_fault(soap);
      if (soap->error && soap->error != SOAP_STOP)
        soap_stream_fault(std::cerr);

      destroy();
      callback.destroy();

      soap_wsrm_dump(soap, stdout);
    }
    else
    {
      /* error or timeout? */
      if (soap->errnum)
      {
        soap_stream_fault(std::cerr);
        exit(1);
      }
      /* timeout occurs after 1 sec */
      /* send acks to peers (optional), 10 ms per message timeout */
      soap_wsrm_pulse(soap, -10000); /* 10 ms */
    }
  }
  return SOAP_OK;
}

int Service::Clear(_mssadh__Clear *req)
{
  /* check for WS-RM/WSA, send empty response (this is a one-way operation over HTTP) */
  if (soap_wsrm_check_send_empty_response(soap))
    return soap->error;

  /* get the handle to the current sequence of the inbound message */
  soap_wsrm_sequence_handle seq = soap_wsrm_seq(soap);

  /* TODO: send acknowledgement(s) immediately? */
  /* soap_wsrm_acknowledgement(soap, seq, NULL); */

  if (soap_wsrm_request_acks(callback.soap, seq, soap_wsa_rand_uuid(callback.soap), "http://Microsoft.Samples.DualHttp/ICalculatorDuplex/Equation"))
  { 
    callback.soap_stream_fault(std::cerr);
    soap_wsrm_seq_release(soap, seq);
    return callback.soap->error;
  }
  callback.soap_endpoint = soap_wsrm_to(seq);
  soap_wsrm_seq_release(soap, seq);

  equation << " = " << result;

  if (callback.soap_endpoint)
  {
    _mssadh__Equation eqn;
    std::string s = equation.str();
    eqn.eqn = &s;
    if (callback.Equation(&eqn) == SOAP_OK)
      std::cout << "Equation(" << s << ")" << std::endl;
    else
      callback.soap_stream_fault(std::cerr);
  }
  callback.destroy();

  result = 0.0;
  equation.str("");
  equation << 0.0;

  return SOAP_OK;
}

int Service::AddTo(_mssadh__AddTo *req)
{
  /* check for WS-RM/WSA */
  if (soap_wsrm_check(soap))
    return soap->error;

  if (req && req->n)
  {
    result += *req->n;
    equation << " + " << *req->n;
  }
  else
    return soap_wsrm_sender_fault(soap, "Invalid data in AddTo", NULL);

  /* this is a one-way operation over HTTP, so we're done with client */
  soap_send_empty_response(soap, 202);

  /* get the handle to the current sequence of the inbound message */
  soap_wsrm_sequence_handle seq = soap_wsrm_seq(soap);

  /* TODO: send acknowledgement(s) immediately? */
  /* soap_wsrm_acknowledgement(soap, seq, NULL); */

  if (soap_wsrm_request_acks(callback.soap, seq, soap_wsa_rand_uuid(callback.soap), "http://Microsoft.Samples.DualHttp/ICalculatorDuplex/Result"))
  { 
    callback.soap_stream_fault(std::cerr);
    soap_wsrm_seq_release(soap, seq);
    return callback.soap->error;
  }
  callback.soap_endpoint = soap_wsrm_to(seq);
  soap_wsrm_seq_release(soap, seq);

  if (callback.soap_endpoint)
  {
    _mssadh__Result res;
    res.result = &result;
    if (callback.Result(&res) == SOAP_OK)
      std::cout << "Result(" << result << ")" << std::endl;
    else
      callback.soap_stream_fault(std::cerr);
  }
  callback.destroy();

  return SOAP_OK;
}

int Service::SubtractFrom(_mssadh__SubtractFrom *req)
{
  /* check for WS-RM/WSA */
  if (soap_wsrm_check(soap))
    return soap->error;

  if (req && req->n)
  {
    result -= *req->n;
    equation << " - " << *req->n;
  }
  else
    return soap_wsrm_sender_fault(soap, "Invalid data in SubtractFrom", NULL);

  /* this is a one-way operation over HTTP, so we're done with client */
  soap_send_empty_response(soap, 202);

  /* get the handle to the current sequence of the inbound message */
  soap_wsrm_sequence_handle seq = soap_wsrm_seq(soap);

  /* TODO: send acknowledgement(s) immediately? */
  /* soap_wsrm_acknowledgement(soap, seq, NULL); */

  if (soap_wsrm_request_acks(callback.soap, seq, soap_wsa_rand_uuid(callback.soap), "http://Microsoft.Samples.DualHttp/ICalculatorDuplex/Result"))
  { 
    callback.soap_stream_fault(std::cerr);
    soap_wsrm_seq_release(soap, seq);
    return callback.soap->error;
  }
  callback.soap_endpoint = soap_wsrm_to(seq);
  soap_wsrm_seq_release(soap, seq);

  if (callback.soap_endpoint)
  {
    _mssadh__Result res;
    res.result = &result;
    if (callback.Result(&res) == SOAP_OK)
      std::cout << "Result(" << result << ")" << std::endl;
    else
      callback.soap_stream_fault(std::cerr);
  }
  callback.destroy();

  return SOAP_OK;
}

int Service::MultiplyBy(_mssadh__MultiplyBy *req)
{
  /* check for WS-RM/WSA */
  if (soap_wsrm_check(soap))
    return soap->error;

  if (req && req->n)
  {
    result *= *req->n;
    equation << " * " << *req->n;
  }
  else
    return soap_wsrm_sender_fault(soap, "Invalid data in MultiplyBy", NULL);

  /* this is a one-way operation over HTTP, so we're done with client */
  soap_send_empty_response(soap, 202);

  /* get the handle to the current sequence of the inbound message */
  soap_wsrm_sequence_handle seq = soap_wsrm_seq(soap);

  /* TODO: send acknowledgement(s) immediately? */
  /* soap_wsrm_acknowledgement(soap, seq, NULL); */

  if (soap_wsrm_request_acks(callback.soap, seq, soap_wsa_rand_uuid(callback.soap), "http://Microsoft.Samples.DualHttp/ICalculatorDuplex/Result"))
  { 
    callback.soap_stream_fault(std::cerr);
    soap_wsrm_seq_release(soap, seq);
    return callback.soap->error;
  }
  callback.soap_endpoint = soap_wsrm_to(seq);
  soap_wsrm_seq_release(soap, seq);

  if (callback.soap_endpoint)
  {
    _mssadh__Result res;
    res.result = &result;
    if (callback.Result(&res) == SOAP_OK)
      std::cout << "Result(" << result << ")" << std::endl;
    else
      callback.soap_stream_fault(std::cerr);
  }
  callback.destroy();

  return SOAP_OK;
}

int Service::DivideBy(_mssadh__DivideBy *req)
{
  /* check for WS-RM/WSA */
  if (soap_wsrm_check(soap))
    return soap->error;

  if (req && req->n)
  {
    result /= *req->n;
    equation << " / " << *req->n;
  }
  else
    return soap_wsrm_sender_fault(soap, "Invalid data in DivideBy", NULL);

  /* this is a one-way operation over HTTP, so we're done with client */
  soap_send_empty_response(soap, 202);

  /* get the handle to the current sequence of the inbound message */
  soap_wsrm_sequence_handle seq = soap_wsrm_seq(soap);

  /* TODO: send acknowledgement(s) immediately? */
  /* soap_wsrm_acknowledgement(soap, seq, NULL); */

  if (soap_wsrm_request_acks(callback.soap, seq, soap_wsa_rand_uuid(callback.soap), "http://Microsoft.Samples.DualHttp/ICalculatorDuplex/Result"))
  { 
    callback.soap_stream_fault(std::cerr);
    soap_wsrm_seq_release(soap, seq);
    return callback.soap->error;
  }
  callback.soap_endpoint = soap_wsrm_to(seq);
  soap_wsrm_seq_release(soap, seq);

  if (callback.soap_endpoint)
  {
    _mssadh__Result res;
    res.result = &result;
    if (callback.send_Result(&res) == SOAP_OK)
      std::cout << "Result(" << result << ")" << std::endl;
    else
      callback.soap_stream_fault(std::cerr);
  }
  callback.destroy();

  return SOAP_OK;
}

/******************************************************************************\
 *
 *	Catch and Report the SOAP Faults Sent by Peer
 *
\******************************************************************************/

int SOAP_ENV__Fault(struct soap *soap, 
        _QName			 faultcode,		// SOAP 1.1
        char			*faultstring,		// SOAP 1.1
        char			*faultactor,		// SOAP 1.1
        struct SOAP_ENV__Detail	*detail,		// SOAP 1.1
        struct SOAP_ENV__Code	*Code,			// SOAP 1.2
        struct SOAP_ENV__Reason	*Reason,		// SOAP 1.2
        char			*Node,			// SOAP 1.2
        char			*Role,			// SOAP 1.2
        struct SOAP_ENV__Detail	*Detail			// SOAP 1.2
)
{
  soap_send_empty_response(soap, 202);

  printf("\n**** Fault Received ****\n");
  if (faultstring)
    printf("reason: %s\n", faultstring);
  else if (Reason && Reason->SOAP_ENV__Text)
    printf("reason: %s\n", Reason->SOAP_ENV__Text);

  if (!detail)
    detail = Detail;
  if (detail && detail->__type == SOAP_TYPE__wsrm__Identifier)
  {
    /* the sequence id is in the Fault Detail __type and fault members */
    char *id = (char*)detail->fault;
    printf("id: %s\n", id);

    /* we opt to treat all faults fatal, so let's terminate the sequence */
    soap_wsrm_sequence_handle seq = soap_wsrm_seq_lookup_id(soap, id);
    if (seq)
    {
      soap_wsrm_error(soap, seq, wsrm__SequenceTerminated);
      soap_wsrm_seq_release(soap, seq);
      return soap->error;
    }
  }

  return SOAP_OK;
}

/******************************************************************************\
 *
 *	Base Service operations (inactive)
 *
\******************************************************************************/

/* These service methods are defined to avoid vtable linking errors.
   A more elegant alternative is to compile all code with
     #define SOAP_PURE_VIRTUAL = NULL
   which makes all base service operations pure virtual (abstract)
*/

int WSDualHttpBinding_USCOREICalculatorDuplexService::Clear(_mssadh__Clear *req) { return SOAP_NO_METHOD; }
int WSDualHttpBinding_USCOREICalculatorDuplexService::AddTo(_mssadh__AddTo *req) { return SOAP_NO_METHOD; }
int WSDualHttpBinding_USCOREICalculatorDuplexService::SubtractFrom(_mssadh__SubtractFrom *req) { return SOAP_NO_METHOD; }
int WSDualHttpBinding_USCOREICalculatorDuplexService::MultiplyBy(_mssadh__MultiplyBy *req) { return SOAP_NO_METHOD; }
int WSDualHttpBinding_USCOREICalculatorDuplexService::DivideBy(_mssadh__DivideBy *req) { return SOAP_NO_METHOD; }
