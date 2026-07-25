
GSOAP APPLICATION EXAMPLES

To build the example services and clients, run 'make' in the 'samples'
directory. The concurrent server examples require the Pthreads library. The SSL
example requires OpenSSL. The webserver example also requires Zlib.

async:                    Asynchronous messaging, optional HTTP pipelining (C)
atom:                     Atom 1.0 reading and writing, REST GET/PUT (C++)
autotest:                 Autotest code generation for XML databindings (C++)
aws-s3:                   AWS S3 SOAP client (C++)
calc:                     Simple calculator client and server (C)
calc++:                   Simple calculator client and server (C++)
calc_vs2005:              VS2005 project calc client example (C++)
calc_xcode:               Mac OS X Xcode project calc client example (C++)
chaining:                 Chaining multiple separately-compiled services (C)
chaining++:               Chaining multiple separately-compiled services (C++)
curl:                     CURL plugin example (C/C++)
databinding:              Data binding example for address.xsd addressbook (C++)
dime:                     DIME attachment client and server (C++,pthreads)
dom:                      DOM-based calculator client (C++)
factory:                  Remote object factory and simple ORB (C++)
factorytest:              Test client for remote object factory (C++)
gmt:                      One-liner service to tell the time in GMT (C++)
googleapi:                Google Maps API (C++,OpenSSL)
hello:                    One-liner greeting service (C++)
httpcookies:              HTTP cookie client and server (C++)
link:                     Linking multiple clients/services (C)
link++:                   Linking multiple clients/services (C++)
lu:                       Linear solver client and server (C++)
magic:                    Magic squares client and server (C++,pthreads)
mashup:                   Combining two services into a new service (C)
mashup++:                 Combining two services into a new service (C++)
mtom:                     MTOM attachment client and server (C)
mtom-stream:              MTOM streaming attachment client and server (C)
oneway:                   One-way SOAP messaging event handling (C,pthreads)
oneway++:                 One-way SOAP messaging event handling (C++,pthreads)
polytest:                 Polymorphic object exchange (C++)
rest:                     REST clients and services examples (C/C++)
roll:                     One-liner roll of a dice service (C++)
router:                   Router of HTTP messages over TCP/IP (C,pthreads)
rss:                      RSS 0.91, 0.92, and 2.0 server (C)
ssl:                      HTTPS/SSL example (C,pthreads,OpenSSL)
template:                 Shows how container templates can be serialized (C++)
testmsgr:                 Test messenger app for server & client testing (C++)
udp:                      SOAP-over-UDP example client and server (C)
varparam:                 Variable polymorphic parameters (C++)
wcf:                      Sample client/server interop with WCF (C++)
webserver:                Stand-alone Web server (C,pthreads,OpenSSL,Zlib)
wsa:                      WS-Addressing demo (C)
wsrm:                     WS-ReliableMessaging demo (C)
wsse:                     WS-Security demo (C,OpenSSL)
wst:                      WS-Trust, WS-Security, and SAML demo (C,OpenSSL)
xml-rpc-json:             XML-RPC and JSON support and examples (C/C++)

LIBRARY AND GSOAP PLUGIN SUPPORT

Some of the examples require the following support libraries and plugins:

plugin/threads.h/.c       Portable threads (Posix and MS Windows threads)
plugin/cacerts.h/.c       Hard-coded SSL public certificates
plugin/wsaapi.h/.c        WS-Addressing plugin
plugin/wsseapi.h/.c       WS-Security plugin
plugin/wsrmeapi.h/.c      WS-ReliableMessaging plugin
plugin/httpda.h/.c        HTTP digest authentication
plugin/httpget.h/.c       HTTP GET server support plugin
plugin/httppost.h/.c      HTTP POST/PUT/PATCH server support plugin
plugin/httppipe.h/.c      HTTP pipelining plugin

DISCLAIMER

WE DO OUR BEST TO PROVIDE YOU WITH REAL-WORLD EXAMPLES BUT WE CANNOT GUARANTEE
THAT ALL CLIENT EXAMPLES CAN CONNECT TO THIRD PARTY WEB SERVICES WHEN THESE
SERVICES ARE DOWN OR HAVE BEEN REMOVED.

MAC OS X XCODE PROJECTS

A Mac OS X Xcode project example is included in samples/calc_xcode.

MS VS2005 PROJECTS

A Visual Studio 2005 project example is included in samples/calc_vs2005. A
custom build step is defined for the gSOAP header file calc.h. The custom build
step invokes soapcpp2 to generate the source code for the project. Note that
the step to obtain the gSOAP header file calc.h is intentionally omitted, e.g.
wsdl2h is not part of this process. It is advised to run wsdl2h manually and
try various wsdl2h options and typemap.dat definitions to customize the wsdl2h
output.

SSL WITH OPENSSL

To try the SSL-secure SOAP server, install OpenSSL and change the occurrences
of "linprog2.cs.fsu.edu" in sslclient.c and sslserver.c to the machine name
(or machine IP) you are using. Example .pem files are included but you need to
create your own .pem files (see OpenSSL documentation).

The sslclient and sslserver codes can then be build as follows:

soapcpp2 -c ssl.h
gcc -DWITH_OPENSSL -o sslclient sslclient.c stdsoap2.c soapC.c soapClient.c -lssl -lcrypto
gcc -DWITH_OPENSSL -o sslserver sslserver.c stdsoap2.c soapC.c soapServer.c -lssl -lcrypto -lpthread

COPYRIGHT AND LICENSE

Several examples in the 'samples' directory are distributed under GPL or
commercial license. Please refer to the specific licensing comments provided in
the source code files.

A commercial license is available from Genivia, Inc. Please contact:
contact@genivia.com
