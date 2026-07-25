
The Tandem NonStop bridge included in this directory enables the gSOAP toolkit
for Tandem NonStop platforms. The bridge replaces the IO operation of the gSOAP
engine with Tandem NonStop IO operations.

This approach requires the code to be generated on any platform (except Tandem
NonStop, e.g. linux) with the wsdl2h and soapcpp2 tools, that should be build
on the platform of choice to generate the source code. The generated code can
be compiled an run on Tandem using the bridge, together with the gSOAP engine
(implemented in stdsoap2.c/.cpp). The wsdl2h and soapcpp2 tools will not run on
Tandem.

When your environment supports OSS sockets then this bridge is not required.

Tandem NonStop bridge for gSOAP 2.8.0 and higher to support Guardian sockets:

  tandem.h	Tandem IO bridge interface
  tandem.c	Tandem blocking IO bridge
  tandemnw.c	Tandem nonblocking IO (no wait) bridge

Usage:

  Add to your client/server code the following:

  #include "tandem.h"
  struct soap *soap = soap_new();
  tandem_init(soap, "ProcName");
  socket_set_inet_name(argv[0]); // See Tandem docs
  ... // client or server code goes here as per gSOAP documentation
  soap_destroy(soap);
  soap_end(soap);
  tandem_done(soap);
  soap_free(soap);

Compile flags for compiling the gSOAP software:

  -DTANDEM_NONSTOP

Linkage requirements:

  Compile and link with tandem.c or tandemnw.c, and stdsoap2.c

Use soapcpp2 option -p to generate shorter file names, for example:

  soapcpp2 -c -L -pT calc.h

generates

TStub.h
TH.h
TC.c
TClient.c
TServer.c

To compile the examples:

  cc -DTANDEM_NONSTOP -o client client.c TC.c TClient.c stdsoap2.c tandem.c
    or
  cc -DTANDEM_NONSTOP -o client client.c TC.c TClient.c stdsoap2.c tandemnw.c

  cc -DTANDEM_NONSTOP -o server server.c TC.c TServer.c stdsoap2.c tandem.c
    or
  cc -DTANDEM_NONSTOP -o server server.c TC.c TServer.c stdsoap2.c tandemnw.c

where 'cc' refers to your C compiler.

Copyright

  Genivia Inc.

Licensing

  GPL (GNU Public License)
  Or the Commercial-use license from Genivia Inc.
