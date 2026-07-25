This directory contains an example stand-alone gSOAP Web server. It uses the
gSOAP HTTP GET plugin to serve HTTP GET requests as well as SOAP/XML HTTP POST
requests.

This is a small but fully functional (embedded) Web server that can serve both
static and dynamic pages and provide SOAP/XML responses.

This Web server implements a simple calculator XML Web service for
demonstration purposes.

This application requires Zlib and Pthreads (you can replace Pthreads with
another thread library, but you need to study the OpenSSL Pthread
implementation when you plan to use OpenSSL).

On Unix/Linux, please enable SIGPIPE handling, see the main function in
webserver.c. SIGPIPE handling will avoid your server from termination when
sockets are disconnected by clients before the transaction was completed
(aka broken pipe).

Please don't be tempted to run the webserver with root permission! The best
way to protect your site is by creating a dummy account with very limited
system access and run the webserver under this account.

	Compile without OpenSSL:
	soapcpp2 -c -n -popt opt.h
	soapcpp2 -c webserver.h
	Customize your COOKIE_DOMAIN in webserver.c (if you use cookies)
	gcc -DWITH_COOKIES -DWITH_ZLIB -I../.. -o webserver webserver.c
	options.c ../../plugin/httpget.c ../../plugin/logging.c
	../../plugin/threads.c ../../stdsoap2.c soapC.c soapClient.c
	soapServer.c -lpthread -lz

	Compile with OpenSSL:
	soapcpp2 -c -n -popt opt.h
	soapcpp2 -c webserver.h
	gcc -DWITH_OPENSSL -DWITH_COOKIES -DWITH_ZLIB -I../.. -o webserver
	webserver.c options.c ../../plugin/httpget.c ../../plugin/logging.c
	../../plugin/threads.c ../../stdsoap2.c soapC.c
	soapClient.c soapServer.c -lpthread -lz -lssl -lcrypto

	Use (HTTP GET):
	Compile the web server as explained above
	Start the web server on an even numbered port (e.g. 8080):
	> webserver 8080 &
	Start a web browser and open a (localhost) location:
	http://127.0.0.1:8080
	and type userid 'admin' and passwd 'guest' to gain access
	Open the location:
	http://127.0.0.1:8080/calc.html
	and enter an expression
	Open the locations:
	http://127.0.0.1:8080/test.html
	http://127.0.0.1:8081/webserver.wsdl

	Use (HTTPS GET):
	Create the SSL certificate (see samples/ssl README and scripts)
	Compile the web server with OpenSSL as explained above
	Start the web server on an odd numbered port (e.g. 8081)
	> webserver 8081 &
	Actually, you can start two servers, one on 8080 and a secure one on
	8081
	Start a web browser and open a (localhost) location:
	https://127.0.0.1:8081
	and type userid 'admin' and passwd 'guest' to gain access
	Open the location:
	https://127.0.0.1:8081/calc.html
	and enter an expression
	Open the locations:
	https://127.0.0.1:8081/test.html
	https://127.0.0.1:8081/webserver.wsdl

	Use (HTTP POST):
	Serves SOAP/XML calculation requests

	Command-line options:
	-z		enables compression
	-c		enables chunking
	-k		enables keep-alive
	-i		enables non-threaded iterative server
	-v		enables verbose mode
	-t<val>		sets I/O timeout value (seconds)
	-s<val>		sets server timeout value (seconds)
	-d<host>	sets cookie domain
	-p<path>	sets cookie path
	-l[none inbound outbound both]
			enables logging

