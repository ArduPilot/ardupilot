
The Apache module for gSOAP                                          {#mainpage}
===========================

[TOC]

By Christian Aberger, Mick Wall, Robert van Engelen, David Viner, Ryan Troll,
and La Cam Chung.


Overview                                                             {#overview}
========

SOAP/XML and REST Web services can be easily created and deployed as gSOAP
standalone services or installed as (Fast)CGI applications. In addition, the
`mod_gsoap` Apache module offers the ability to run gSOAP services directly
inside the Apache HTTP server. The `mod_gsoap` Apache module supports the
deployment of multiple gSOAP services that can run together with the usual
services on Apache. This approach offers a production-quality Web services
deployment scenario.

The `mod_gsoap` Apache module is designed to keep things simple so that
existing gSOAP services can be recompiled for Apache HTTP server deployment
without modification of the source code. The Apache `apxs` command compiles
your gSOAP service code and installs it with `mod_gsoap`. Add your new service
to Apache `httpd.conf` and presto!

The original Apache module for gSOAP home page is at
<http://mod-gsoap.sourceforge.net>. Newer versions are included in the
gSOAP distribution package in the `gsoap/mod_gsoap/mod_gsoap-0.9` directory.

Apache modules for gSOAP are provided for both Apache 1.3 and 2.x. In the
following we will discuss the Apache module for Apache 2.x.


Installation                                                          {#install}
============

First download the [Apache httpd](https://httpd.apache.org) source code and
install the httpd server in a new directory, say `apachegsoap`:

    mkdir apachegsoap
    cd apachegsoap
    tar -xjf httpd-2.4.35.tar.bz2
    cd httpd-2.4.35
    ./configure --prefix=`pwd`/.. --with-mpm=worker --enable-mods-shared=most
    make -j4
    make install

If configure fails with "error: APR not found" or "error: APR-util not found",
download and install the [Apache Portable Runtime](http://apr.apache.org)
(APR).

To use the Apache extension mechanism, your platform has to support the DSO
feature and your Apache `httpd` binary has to be built with the `mod_so`
module. The `apxs` tool automatically complains if this is not the case. You
can check this yourself by manually running the `httpd -l` command (installed
locally in `apachegsoap/bin` with the instructions above):

    cd apachegsoap
    bin/httpd -l

The module `mod_so.c` should be on the displayed list of modules.

Next, we will build and install `mod_gsoap` for Apache 2.x and up. The source
code files are located under `gsoap/mod_gsoap/mod_gsoap-0.9/apache_20` and
include `apache_gsoap.h`, `mod_gsoap.c`, and a Visual Studio project file
`mod_gsoap.vcproj`.

To compile `mod_gsoap`, execute:

    cd /path/to/gsoap/installation/gsoap/mod_gsoap/mod_gsoap-0.9/apache_20
    ln -s ../../../stdsoap2.h .
    sudo $HOME/apachegsoap/bin/apxs -a -i -c mod_gsoap.c

Root permissions are required, so we used `sudo apxs` here.

The `apxs` command should be on your path or located in `apachegsoap/bin` where
we installed httpd.  Make sure to use `$HOME/apachegsoap/bin/apxs` if multiple
httpd versions are installed.

If a specific C compiler is required, say `cc`, then try `apxs -S CC=cc ...`.


Deploying C services with the Apache module                            {#deploy}
===========================================

After building `mod_gsoap` we are ready to deploy gSOAP services written in C
with the Apache module.

The gSOAP package contains a calculator example. We will use this example to
walk you through the creation and deployment of an Apache module gSOAP service.

First, copy the calculator example:

    cd apachegsoap
    cp /path/to/gsoap/installation/gsoap/samples/calc/* .
    cp /path/to/gsoap/installation/gsoap/stdsoap2.* .
    cp /path/to/gsoap/installation/gsoap/mod_gsoap/mod_gsoap-0.9/apache_20/apache_gsoap.h .

Next, edit `calcserver.c` by removing `main()` and replace it with
`IMPLEMENT_GSOAP_SERVER()` as follows:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    #include "apache_gsoap.h"
    IMPLEMENT_GSOAP_SERVER() /* replaces main() { ... } */
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To initialize the engine context with flags and/or plugins, see \ref plugins.

Then compile and build the service:

    soapcpp2 -c -SL -wx calc.h
    sudo $HOME/apachegsoap/bin/apxs -a -c calcserver.c soapC.c soapServer.c stdsoap2.c
    chmod 755 .libs/calcserver.so

Again, the `apxs` command should be on your path or located in
`apachegsoap/bin` where we installed httpd.  Make sure to use
`$HOME/apachegsoap/bin/apxs` as shown above if multiple httpd versions are
installed.

This creates `.libs/calcserver.so` service module that is universally readable.
Also make sure that `.libs/calcserver.so` is readable through the entire path,
that is through `/home/username/apachegsoap/.libs` where `username` is your
user account name.

If you have installed source files in other directories, then you will need to
add appropriate `-I` and `-L` options with the `apxs` command shown above.

To deploy the service, we will need to add our module with its properties to
`httpd.conf` (for example we can add it at the end):

    <IfModule mod_gsoap.c>
     <Location /soap>
      SetHandler gsoap_handler
      SOAPLibrary /home/username/apachegsoap/.libs/calcserver.so
      Order allow,deny
      Allow from all
     </Location>
    </IfModule>

The `httpd.conf` file is usually found under `/private/etc/apache2/httpd.conf`.
However, since we use a locally installed Apache `httpd` server that searches
the `/home/username/apachegsoap` path, you will find `httpd.conf` in
`/home/username/apachegsoap/conf`.

The `Location` property sets part of the URL of the service, which in this case
will be `http://localhost/soap` or if you set a port that is different than the
standard HTTP port 80, say 9080, the full URL is `http://localhost:9080/soap`.
To change the port from 80 to 9080, edit `httpd.conf` and change `Listen`:

    Listen 9080

To start the service:

    cd apachegsoap
    bin/apachectl start

Point your browser to `http://localhost:9080` and the page should show the
message "It works!".

To stop the server:

    bin/apachectl stop

To use the service by client applications, direct the endpoint URL of clients
to "http://localhost:9080/soap", for example in the `calcclient.c` code that
came with the gSOAP example:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    const char server[] = "http://localhost:9080/soap";
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Then we build the client:

    soapcpp2 -c -CL -wx calc.h
    cc -o calcclient calcclient.c soapC.c soapClient.c stdsoap2.c

and run it:

    ./calcclient add 2 3
    result = 5

To let clients access the WSDL of a service, you can use the query `?wsdl` as
part of the URL such as `http://localhost:9080/soap?wsdl` to pull the file
`calc.wsdl` from the current location of the service. To do so, copy the
`calc.wsdl` file there to make it available to the Apache server.

Deployment of multiple modules is possible since gSOAP 2.8.71, by specifying
multiple `<Location>` entries in `<IfModule mod_gsoap.c>` in `httpd.conf`, one
for each service.  The change in gSOAP 2.8.71 modified function
`::SoapSharedLibrary_load` in `mod_gsoap.c` as follows to resolve linking
symbols locally:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    const int nFlags = RTLD_LAZY | RTLD_LOCAL; // was RTLD_LAZY | RTLD_GLOBAL
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Alternatively, if one module links against equally-named symbols coming from
another module, then it is recommended to pass a version info file to the
linker, instructing it to make all symbols local, except for the Apache module
table instance.  The version file (e.g. `myapachegsoap.ver`) looks something
like this:

    VERS_1.0 {
      global:
        myapachegsoap_module;
      local:
        *;
    };

This file is then passed to the linker with option `-Wl,myapachegsoapv.ver`.


Initialization and plugins                                            {#plugins}
==========================

By replacing `main()` with `IMPLEMENT_GSOAP_SERVER()`, several functions are
implemented that are used by the Apache module for gSOAP.  These functions
include `apache_default_soap_init()` to initialize a newly constructed context.

With gSOAP 2.8.54 and greater we can add our own initialization function to
initialize the context.  By doing so we can set context flags and register
plugins.  To define our own initialization function we use
`IMPLEMENT_GSOAP_SERVER_INIT(init_func)` instead of `IMPLEMENT_GSOAP_SERVER()`:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    #include "plugin/logging.h"
    #include "apache_gsoap.h"
    void mod_gsoap_init(struct soap *soap, request_rec *r)
    {
      FILE *fdi = fopen("/tmp/INBOUNDAUDIT.log", "a");
      FILE *fdo = fopen("/tmp/OUTBOUNDAUDIT.log", "a");
      soap_set_mode(soap, SOAP_XML_INDENT | SOAP_ENC_MTOM);
      soap_register_plugin(soap, logging);
      soap_set_logging_inbound(soap, fdi);
      soap_set_logging_outbound(soap, fdo);
    }
    IMPLEMENT_GSOAP_SERVER_INIT(mod_gsoap_init) /* replaces main() { ... } */
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This enables XML indentation, MTOM attachments, and saves the audit logs of
inbound and outbound messages (note that messages are not frequently flushed to
the log files by the logging plugin, meaning the logs may appear incomplete
until you stop httpd to close these files).  The gsoap/plugin/logging.h
and logging.c files are located in the gSOAP source code tree.  Add logging.c
to the apxs command to compile the source code files.

Your can include multiple registrations of plugins as needed.

@warning Do not use any of the `SOAP_IO` flags to initialize or set the
context, such as `SOAP_IO_KEEPALIVE` and `SOAP_IO_CHUNK`.


Dynamic libraries                                                   {#libraries}
=================

If you are using dynamic libraries to deploy services with `mod_gsoap`, then
those should be closed properly to avoid memory leaks. To do so add the following line:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    dlclose(pConfig->m_pLibraries->m_pSOAPLibrary->m_hLibrary);
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

at the end of the `gsoap_handler()` function in `mod_gsoap.c`.


Building C++ services from service classes                            {#classes}
==========================================

The Apache server is written in C. Building Apache modules in C++ is tricky and
cannot be fully guaranteed due to compiler differences. Several online
resources exist that can help to implement C++ modules for Apache 2.0. If this
fails, the best alternative is to use FastCGI (see gSOAP user guide on "FastCGI
Support").

When using C++ gSOAP service classes generated by `soapcpp2` options `-i` or
`-j` we need to implement the C function `soap_serve()` that dispatches the
services.

We will walk through the implementation of a service using the same calculator
example demonstrated above.

First, run `soapcpp2` with option `-j` to generate a service class:

    soapcpp2 -j -SL -wx calc.h

Create a new `calcerver.cpp` file with the following code:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    #include "soapcalcService.h"
    #include "calc.nsmap"
    #include "apache_gsoap.h"
    IMPLEMENT_GSOAP_SERVER()
    extern "C" int soap_serve(struct soap *soap)
    {
      calcService service(soap);
      int err = service.serve();
      service.destroy();
      return err;
    }
    int calcService::add(double a, double b, double *result)
    {
      *result = a + b;
      return SOAP_OK;
    } 
    int calcService::sub(double a, double b, double *result)
    {
      *result = a - b;
      return SOAP_OK;
    } 
    int calcService::mul(double a, double b, double *result)
    {
      *result = a * b;
      return SOAP_OK;
    } 
    int calcService::div(double a, double b, double *result)
    {
      *result = b != 0 ? a / b : 0.0;
      return SOAP_OK;
    } 
    int calcService::pow(double a, double b, double *result)
    {
      *result = ::pow(a, b);
      return SOAP_OK;
    }
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Here, `calcService` is the service class defined in the generated
`soapcalcService.h` and `soapcalcService.cpp` files.

The `apxs` command is used to compile as follows, with the `-S CC=c++` option:

    bin/apxs -a -c -S CC=c++ calcserver.cpp soapC.cpp soapcalcService.cpp stdsoap2.cpp
    chmod 755 .lib/calcserver.so

This creates `.libs/calcserver.so` service module that is universally readable.
Also make sure that `.libs/calcserver.so` is readable through the entire path,
that is through `/home/username/apachegsoap/.libs`.

After compilation, the new module should be added to `httpd.conf` as was
explained above.

When multiple service classes are defined, when `soapcpp2 -j` is applied to
multiple `.h` files, then you have two options:

-# create an `.so` library for each service with the `apxs` command and add
   each module to `httpd.conf` with a new `Location` property.
-# create one `.so` library with the services combined, all listening to the
   same service URL. Only one module is added to `httpd.conf` since the
   `Location` property is the same. Note that the `?wsdl` query is not as
   useful in this case, since we cannot publicize the service WSDLs combined.

The second option requires C++ namespaces as explained in section "How to Chain
C++ Server Classes to Accept Messages on the Same Port" in the gSOAP user guide.
Basically, you should chain the services as follows:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    #include "apache_gsoap.h"
    IMPLEMENT_GSOAP_SERVER()
    extern "C" int soap_serve(struct soap *soap)
    {
      int err;
      if ((err = soap_begin_serve(soap)) == SOAP_OK)
      {
        X::Service service_x(soap);
        if ((err = service_x.dispatch()) == SOAP_NO_METHOD)
        {
          Y::Service service_y(soap);
          if ((err = service_y.dispatch()) == SOAP_NO_METHOD)
          {
            Z::Service service_z(soap);
            err = service_z.dispatch();
          }
        }
      }               
      soap_destroy(soap);
      soap_end(soap);
      return err;
    }
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~


Troubleshooting                                                         {#debug}
===============

It is recommended to test the service first as a stand-alone server over a port
using `soap_bind()` and `soap_accept()`. Debugging is much easier this way.

After testing as a stand-alone server, to debug the service with the Apache
module, compile your service application with compiler option `-g` (apxs option
`-Wc,-g`to control debug information output.  For example:

    bin/apxs -Wc,-g -a -c calcserver.c soapC.c soapServer.c stdsoap2.c

Debug the service as it is deployed while `httpd` is running.  To do so, we
first stop the service and start it up in single threaded mode so we can attach
to it to a debugger such as `gdb` or `lldb`:

    cd apachegsoap
    bin/apachectl stop
    bin/httpd -X -f /home/username/apachegsoap/conf/httpd.conf &

You will get a process ID, say 12345, which we will attach to a debugger:

    gdb -p 12345

You will see a load of symbols loading, including `mod_gsoap.so`.

Next, we set a breakpoint on the `soap_serve` call (the main entry point), let
`gdb` know that we accept the pending breakpoint, and then continue the
process:

    (gdb) b soap_serve
    Function "soap_serve" not defined.
    Make breakpoint pending on future shared library load? (y or [n]) y
    Breakpoint 1 (soap_serve) pending.
    (gdb) c
    Continuing.

We execute our `calcclient` from another window, which triggers the breakpoint.

    [Switching to Thread 0x7f3fbd209950 (LWP 25493)]
    Breakpoint 1, 0x00007f3fb059e398 in soap_serve () from /home/username/apachegsoap/gsoap-2.8/gsoap/samples/calc/.libs/calcserver.so
    Current language: auto; currently asm
    (gdb) n
    Single stepping until exit from function soap_serve,
    which has no line number information.
    0x00007f3fb05a5eec in apache_default_soap_serve () from /home/username/apachegsoap/gsoap-2.8/gsoap/samples/calc/.libs/calcserver.so
    (gdb)

Note that when Apache `httpd` runs as user `wwwrun` (or another user) then you
will have to run the debugger as that same user. To do so, execute:

    sudo -u wwwrun gdb /usr/sbin/httpd2-worker 25487

When failures occur, see also the error logs that are saved by httpd to
`/home/username/apachegsoap/logs/error_log`.

Thanks to Jon Scobie for suggestions to improve this section.


Limitations                                                       {#limitations}
===========

The gSOAP Apache module does not support receiving DIME protocol messages with
attachments.  MIME and MTOM attachments are supported.


License                                                               {#license}
=======

The Apache modules for gSOAP are released under the gSOAP open source public
license and GPLv2.  The open source licensing is replaced by Genivia's license
for commercial use when a commercial-use license is purchased by customer.


Further reading                                                    {#references}
===============

[All things `mod_gsoap`](http://techiebitsandpieces.blogspot.com/2011/03/all-things-modgsoap.html) by Jon Scobie

[Apache HTTP server project](http://httpd.apache.org/download.cgi).
