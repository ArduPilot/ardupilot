Simple examples illustrating asynchronous messaging with SOAP and REST.

The SOAP examples are a calculator web service and invoke 'add' and 'mul'
service operations.  The REST examples use GET, PUT, and POST of and XML
record to the REST server.  The web server used by these examples is the
gSOAP demo web server located in gsoap/samples/webserver, which should be build
to run the examples.  The gSOAP webserver optionally uses the httppipe HTTP
pipelining plugin to support HTTP pipelining.

Note that true asynchronous messaging expects non-blocking operations.
However, message sends and receives cannot be interrupted without losing their
internal state, i.e. deserialized data is lost in case of receives.  Message
receives can be placed in a thread to prevent blocking on receives.  Message
sends can either be placed in a thread or use timeouts and retries.

The following examples demonstrate alternative approaches, showing one-way
messaging with one thread (i.e. blocking send and receive), and mixing blocking
sends with async non-blocking receives using threads to receive responses.
The probability of blocking on sending short messages is quite low, e.g. HTTP
GET messages.

Examples:

- asyncsoap.c   C example SOAP asynchronous messaging with one thread
- asyncsoap.cpp C++ example SOAP asynchronous messaging with one thread
- asyncrest.c   C example REST asynchronous messaging with one thread
- asyncrest.cpp C++ example REST asynchronous messaging with one thread

- asyncsoap2.c   C example SOAP asynchronous messaging with threads
- asyncsoap2.cpp C++ example SOAP asynchronous messaging with threads
- asyncrest2.c   C example REST asynchronous messaging with threads
- asyncrest2.cpp C++ example REST asynchronous messaging with threads

- asyncsoap3.c   C example SOAP asynchronous pipelined messaging with threads
- asyncsoap3.cpp C++ example SOAP asynchronous pipelined messaging with threads
- asyncrest3.c   C example REST asynchronous pipelined messaging with threads
- asyncrest3.cpp C++ example REST asynchronous pipelined messaging with threads

To build the C examples:

    $ soapcpp2 -c -CL -r -wx async.h
    $ cc -I. -I../.. -I../../plugin -o asyncsoap asyncsoap.c ../../stdsoap2.c soapC.c soapClient.c
    $ cc -I. -I../.. -I../../plugin -o asyncrest asyncsoap.c ../../stdsoap2.c soapC.c
    $ cc -I. -I../.. -I../../plugin -o asyncsoap2 asyncsoap2.c ../../stdsoap2.c soapC.c soapClient.c ../../plugin/threads.c
    $ cc -I. -I../.. -I../../plugin -o asyncrest2 asyncsoap2.c ../../stdsoap2.c soapC.c threads.c ../../plugin/threads.c
    $ cc -I. -I../.. -I../../plugin -o asyncsoap3 asyncsoap3.c ../../stdsoap2.c soapC.c soapClient.c ../../plugin/threads.c
    $ cc -I. -I../.. -I../../plugin -o asyncrest3 asyncsoap3.c ../../stdsoap2.c soapC.c threads.c ../../plugin/threads.c

To build the C++ examples:

    $ soapcpp2 -j -CL -r -wx async.h
    $ c++ -I. -I../.. -I../../plugin -o asyncsoap++ asyncsoap.cpp ../../stdsoap2.cpp soapC.cpp soapasyncProxy.cpp
    $ -I. -I../.. -I../../plugin c++ -o asyncrest++ asyncrest.cpp ../../stdsoap2.cpp soapC.cpp
    $ c++ -I. -I../.. -I../../plugin -o asyncsoap2++ asyncsoap2.cpp ../../stdsoap2.cpp soapC.cpp soapasyncProxy.cpp ../../plugin/threads.c
    $ -I. -I../.. -I../../plugin c++ -o asyncrest2++ asyncrest2.cpp ../../stdsoap2.cpp soapC.cpp ../../plugin/threads.c
    $ c++ -I. -I../.. -I../../plugin -o asyncsoap3++ asyncsoap3.cpp ../../stdsoap2.cpp soapC.cpp soapasyncProxy.cpp ../../plugin/threads.c
    $ -I. -I../.. -I../../plugin c++ -o asyncrest3++ asyncrest3.cpp ../../stdsoap2.cpp soapC.cpp ../../plugin/threads.c

See also the build instructions in the C/C++ source code of the examples.

Start the webserver at port 8080, then run and example such as asyncsoap:

    $ ../webserver/webserver -e -k 8080 &
    $ ./asyncsoap

Where webserver options -e and -k enable pipelining and keep-alive,
respectively.

Non-pipelined asynchronous messaging splits the sending operation from the
receive operation.  Sends should be paired with receives, meaning that no other
send should occur on the same connection before the response is received.
A connection may or may not be persistent, though the examples with threads
require persistence to allow the receiving thread to receive responses over
the single HTTP keep-alive connection.

By contrast, HTTP pipelining allows multiple sends to be followed by multiple
receives over a persistent connection, i.e. with HTTP keep-alive enabled.  HTTP
pipelining requires at least two threads, one for sending and one for
receiving.  HTTP servers may not implement HTTP pipelining that is required to
handle multiple requests messages.

HTTP pipelining may boost performance, though it is not widely supported and
the performance gains are generally known to dissapoint.

HTTP pipelining on the server side is easy with gSOAP, just add:

    #include "plugin/httppipe.h"

    struct soap *soap = soap_new1(SOAP_IO_KEEPALIVE); // keep-alive required
    soap_register_plugin(soap, httppipe);             // register

The gsoap/samples/webserver/webserver can be started with HTTP pipelining
enabled, say on port 8080 with options -e (pipeline) and -k (keepalive) as
follows:

    $ ../webserver/webserver -e -k 8080

Implementing HTTP pipelining correctly on the client side requires threads and
logic to deal with failed requests and responses.  Using only one thread may
cause significant message blocking issues or even deadlock with infinite IO
timeouts:

    Client              Server
    send req1 --------> recv req1
    send req2 --->      processing
    waiting        <--- send res1
    waiting             waiting
    :                   :
    send req1 failed    send res1 failed

Using two client threads T1 and T2 end orks:

    Client T1           Client T2               Server
    send req1 --------------------------------> recv req1
    send req2 --->      waiting                 processing
    waiting       \     recv res1 <------------ send res1
    waiting        ---------------------------> recv req2
    waiting             waiting                 processing
    waiting             recv res2 <------------ send res2
    done                done

When the client thread T2 does not receive one or more responses from the
server it may be caused by the lack of HTTP pipeline support by the server
or by a transmission error.  In either case the client thread T1 should resend
the message.  Such a send-resend scheme requires messaging to be "idempotent",
meaning that the server state does not matter.  HTTP GET, HEAD, PUT, and DELETE
are idempotent but HTTP POST is not.  However, HTTP POST can be made idempotent
if the POST message contents do not depend on the server state, which is
typical for SOAP/XML Web services that commonly use POST message exchanges.  In
general, be forewarned that if any of the messages depend on the server state
then HTTP pipelining is a bad idea.
