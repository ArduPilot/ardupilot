
SOAP/XML over HTTP message router.

Supports many HTTP message routing scenarios.

Command-line options:
        -a<action> action value override (SOAP Action)
	-c         connect directly to endpoint if routing table redirect fails
	-e<URL>    endpoint URL
	-g<URL     get content (instead of HTTP POST with -e)
	-p<port>   start stand-alone router on port
	-r<file>   routing table XML file
	-t<sec>    timeout

Forwarding of messages to a service
-----------------------------------

router [-c] [-e<endpoint> | -g<endpoint>] [-a<SOAPAction>] [-r<routingfile>] [-t<timeout>] [<msgfile>]

Examples:

1.
router -c request.soap
Sends the request message stored in file request.soap and returns
response to stdout where file request.soap contains a SOAP request with
HTTP header and SOAP/XML/DIME body. If the SOAPAction in the message is
present and matches one or more keys in the routing table, the
alternative service endpoints in the table will be tried first until
one service endpoint is found to accept the connection. If no
SOAPAction is given or the SOAPAction does not match any key, then the
endpoint in the HTTP header in request.soap is searched in the routing
table. If the endoint matches one or more keys in the routing table,
the alternative endpoints will be tried first until one endpoint is
found to accept the connection. Finally, the endpoint in the HTTP
header of request.soap is used to establish a connection if all other
service endpoints in the table failed and if option -c is enabled.

2.
router -c -ehttp://domain/path request.soap
Sends request message to http://domain/path and returns the service
response to stdout. If http://domain/path matches one or more keys in
the routing table, then the alternative service endpoints in the table
will be tried first until one service endpoint is found to accept the
connection. The http://domain/path endpoint is tried last when all
other service endpoints in the table failed. File request.soap MAY
contain an HTTP header but MUST of course contain a body.

To try this, compile the 'calc' client (samples/calc). Edit the
'calc.add.req.xml' SOAP/XML request file and replace <a> and <b> values. Then
run:
router -ehttp://www.cs.fsu.edu/~engelen/calcserver.cgi -a"" calc.add.req.xml
The SOAP/XML response is returned.

3.
router -aSOAPAction request.soap
When SOAPAction matches one or more keys in the routing table, then the
alternative endpoints in the table will be tried first until one
endpoint is found to accept the connection. When all endpoints fail,
or when SOAPAction does not match a key, the router fails. File
request.soap MAY contain an HTTP header but MUST of course contain a
body.

4.
router -c -rroutingtable.xml request.soap
Same as 1. but uses routingtable.xml as the routing table after
checking keys in the internal routing table. The XSD schema of
routingtable.xml is generated as t.xsd. The default routing table file
is router.xml.

5.
router -c -t5 request.soap
Same as 1. but searches the routing table for an endpoint that takes
less than 5 seconds to connect to. Use negative timeouts to specify a
timeout value in microseconds. The timeout also specifies the message
receive timeout AFTER the connection was established.

6.
cat request.soap | router -ehttp://domain/path | more
When request.soap does not contain an HTTP header, the router computes
the HTTP content length by buffering the entire request message which
allows you to use it as a filter as in this example. (fstat() is
generally tried first to determine file length.)

7.
router -ghttp://domain/path/file.html
Sends an HTTP GET request to the host and copies the response to stdout.

CGI-based relay server
----------------------

Install the router as CGI application. The CGI-based relay service uses
SOAPActions in the messages and HTTP query strings to index the routing
table.

Examples:

Messages addressed to "http://domain/cgi-bin/router?key" will be routed
by the router to the service endpoint associated with the key in the
routing table. When messages use SOAPActions, the SOAPActions will be
used to find service endpoints instead of a query string.

To tunnel SOAP through firewals to stateful stand-alone Web services:
run a stand-alone gSOAP Web service on a port, e.g. 18000. Add the
key-endpoint pair "myservice", "http://localhost:18000" to the router
table. After installing the router, all requests for endpoint
http://domain/cgi-bin/router?myservice will be tunneled to the
stand-alone Web service.

To add backup services: add multiple key-endpoint pairs to the routing
table with the same key. Given a key (e.g. SOAPAction or Query string)
the router will check the endpoints in sequence until it can connect.
If one or more of the backup services are down, an active service
endpoint will be selected.

Multi-threaded stand-alone relay server
---------------------------------------

router -p<port> [-r<routingfile>] [-t<timeout>] &

Examples:

router -p18000 -rtable.xml -t5 &
Runs a stand-alone router on port 18000 using table.xml as the external
routing table for lookup. Service endpoints are selected from
alternative endpoints that take less than 5 seconds to connect to.

Clients connect to the router with a service endpoint such as
"http://machine:<port>/path" where the endpoint "http://machine/path"
(note the absence of the port) will be used as a key in the routing
table to find an endpoint when no SOAPAction is present. For example, a
stand-alone Web service called "quote" runs on a machine named "zulu"
port 18080. To address this service through the router, add key
"http://zulu/quote" and endpoint "http://zulu:18080" to the routing
table. Run the router on port 18000.  Router requests with endpoint
"http://zulu:18000/quote" will be relayed to zulu:18080

Gateway keeper
--------------

When the routing table contains userid and passwd information, the
client requests are only tunnelled when the proper HTTP Authorization
userid and passwd are provided in the client request message. It is
possible to provide different service endpoint in the table depending
on the client's HTTP Authorization information.

Notes
-----

* Table lookup algorithm:
  SOAPActions (if provided) are used first to match routing table keys.
  Next, HTTP query string in the endpoint URL (CGI only) is used to
  match routing table keys.
  Next, the service endpoint is checked to match routing table keys.
  Finally, if the -c option is set the service endpoint URL itself is
  used to connect.
* Keys in routing table may contain * (multi-char) and - (single-char)
  wildcards to match multiple SOAPActions and endpoints.
* When a match is found but the endpoint is NULL in the table, the
  search is terminated. This can be used to prevent searches in the
  routing file for specific patterns.
* Optional HTTP Authorization userid and passwd are checked if present
  in the routing table. The userid and passwd may be patterns with '*'
  and '-' wildcards. An endpoint in the table is selected for which
  the userid and passwd match.
* <timeout> is TCP connect and I/O timeout for router-server connection
  in seconds (use negative value for timeout in microseconds).
* When an external routing table is once read by a stand-alone router,
  it will be cached to optimize speed. But this also means that
  changing the contents of the routing table file does not affect the
  actual routing while the stand-alone router is running.
* HTTP POST and HTTP GET styles of SOAP messaging is supported
  (but CGI-based router does not support HTTP GET)
* Supports any type of messages (e.g. DIME)
* HTTP cookies are not handled and will be deleted from the HTTP header
* Keep-alive support has not been tested and might not work
