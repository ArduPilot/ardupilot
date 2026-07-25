This demo illustrates the use of HTTP cookies to implement stateful services.

This code base can be used to implement your own cookie-based stateful
services, even with CGI which is inherently stateless.

gSOAP's cookie handling is fully automatic at the client-side. A database of
cookies is kept and returned to the appropriate servers. In this demo, the
value (int) of the (invisible) cookie is returned as an output parameter by
the service to demonstrate that each call uses a unique and updated cookie.
Cookies are not automatically saved to a file by the client. So the internal
cookie database is discarded when the program terminates.

The code is written in C++ and compiled with:
    soapcpp2 ck.h
    cc -o ckclient ckclient.cpp stdsoap2.cpp soapC.cpp soapClient.cpp
    cc -o ckserver ckserver.cpp stdsoap2.cpp soapC.cpp soapServer.cpp
