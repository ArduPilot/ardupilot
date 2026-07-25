gSOAP user guide                                                    {#mainpage}
================

[TOC]

# User guide

<div align="right"><i>Copyright (c) 2000-2020, Genivia Inc.<br>All rights reserved.</i></div>

# Introduction                                                         {#intro}

The gSOAP toolkit offers C/C++ tools and libraries to implement efficient and
secure SOAP, XML, JSON and REST client and service Web API applications.  The
tools also offer XML data bindings for C and C++ to generate XML serializers
to efficiently read and write C/C++ data to and from files and streams.

The gSOAP wsdl2h tool consumes WSDL and XSD schema files to converts them to
C/C++ source code to implement XML messaging infrastructures.  This frees the
developer to focus on application functionality rather than on infrastructure.

More specifically, the wsdl2h tool consumes WSDLs to generate a C or C++
interface header file, which uses a developer-friendly C/C++ header file
syntax.  This allows developers to inspect Web services and XML schemas from a
functionality point of view, rather than getting bogged down into the
underlying SOAP-based infrastructure details of WSDLs and XSDs.

The soapcpp2 tool generates all the Web service binding source code with XML
serializers necessary to quickly develop server-side and client-side Web
service APIs.

The soapcpp2 tool can also be used to produce, rather than consume, WSDL and
XSD files to deploy XML Web services or to develop XML applications.  This
approach allows the deployment of legacy C/C++ applications as services.
Simply describe the Web API in a C or C++ interface header file for the
soapcpp2 tool to generate the C/C++ source code that glues everything together.

Besides SOAP-based services, also non-SOAP XML and JSON REST services can
be implemented with the gSOAP tools.  Either described by WSDLs or by XML
schemas converted to C/C++ source code by wsdl2h, or by using the JSON
libraries included with gSOAP to develop JSON applications.

Furthermore, the gSOAP tools can be just as easily used to develop C/C++
applications that efficiently consume and produce XML by leveraging XML data
bindings for C/C++ based on XML schemas.  Basically, an XML schema has an
equivalent set of C/C++ data types for the components described by the schema.
So XML schema strings are just C/C++ strings, XML schema enumerations are C/C++
enums, XML schema complex types are just structs and classes in C/C++, and so
on.  This enhances the reliability and safety of XML applications, because
type-safe serializable C/C++ data types are serialized and validated in XML
automatically.

This XML data binding means that your XML data is simply represented as C/C++
data.  Reading and writing XML is a lot easier than using a DOM or SAX library
for XML.  This is not more expensive or more complex than it sounds.  In fact,
the generated XML serializers are very efficient to parse and validate XML
and may run more than 30 times faster than validating XML parsers such as
Apache Xerces C++.

In summary, gSOAP offers a type-safe and transparent approach to develop XML
applications that has proven to be quicker to develop (by auto-coding), safer
(by XML validation and type-safety), more reliable (by auto-generation of XML
test messages and warnings), and higher performing (by efficient serializers
and XML parsers generated in C/C++), compared to DOM and SAX libraries.

This user guide explains the gSOAP tools and libraries.  This user guide and
additional documentation for the growing number of gSOAP plugins can be found
at <https://www.genivia.com/doc>.  A getting-started guide for developers is
available at <https://www.genivia.com/dev.html> with a tutorial on common
topics at <https://www.genivia.com/tutorials.html>. Various examples ranging
from simple calculator service APIs to very large protocols spanning dozens of
WSDLs can be found at <https://www.genivia.com/examples.html>.  For frequently
asked questions see <https://www.genivia.com/resources.html> for help.

🔝 [Back to table of contents](#)

# Notational conventions  {#conventions}

The typographical conventions used by this document are:

* `Courier` denotes C and C++ source code.

* <i>`Courier`</i> denotes XML content, JSON content, file and path names, and
  URIs.

* <b>`Courier`</b> denotes HTTP content, text file content, and shell commands
  with command line options and arguments.

The keywords "MUST", "MUST NOT", "REQUIRED", "SHALL", "SHALL NOT", "SHOULD",
"SHOULD NOT", "RECOMMENDED", "MAY", and "OPTIONAL" in this document are to
be interpreted as described in RFC-2119.

🔝 [Back to table of contents](#)

# Tooling characteristics {#features}

* **Safety**: the tools generate type-safe XML serialization
  functions for native and user-defined C and C++ data structures. 

* **Protocols**: WSDL 1.1, WSDL 2.0, REST, SOAP 1.1, SOAP 1.2, SOAP RPC encoding
  style, SOAP document/literal style, SOAP-over-UDP, WS-Security,
  WS-Addressing, WS-ReliableMessaging, WS-Discovery, WS-Trust, WS-Policy, JSON
  REST/RPC, XML-RPC, Atom and RSS. JSON is supported as a library bundled with
  the XML-RPC library to switch between XML-RPC and JSON protocols (since these
  are similar, speaking data wise). For more details, see the
  <i>`gsoap/samples/xml-rpc-json`</i> folder in the gSOAP package and the
  [XML-RPC and JSON](../../xml-rpc-json/html/index.html) documentation.

* **SOAP**: implements the full range of SOAP 1.1/1.2 specifications,
  including RPC encoding and document/literal messaging styles.

* **XSD**: supports all XML schema 1.0 and 1.1 schema type constructs and has
  been tested against the W3C XML Schema Patterns for Databinding
  Interoperability working group.

* **XML**: implements a fast schema-specific XML pull parser that does not
  require intermediate storage of XML in a DOM to deserialize data.

* **HTTP**: HTTP 1.0/1.1 (HTTP 2.0 will be available soon), IPv4 and IPv6,
  HTTPS (requires OpenSSL or GNUTLS), cookies, authentication, Zlib deflate and
  gzip compression, and connecting through HTTP proxies.

* **Attachments**: MIME (SwA), DIME, and MTOM attachments are supported.
  Streaming capabilities to direct the data stream to/from resources using
  user-defined callbacks.  gSOAP is the only toolkit that supports streaming
  MIME, DIME, and MTOM attachment transfers, which allows you to exchange
  binary data of practically unlimited size in the fastest possible way
  by streaming, while ensuring the usefulness of XML interoperability.

* **Debugging**: compilation of client and service applications in `#DEBUG` mode
  traces their engine activity for debugging, verifies memory usage (leak
  detection), and saves message logs for inspection.

* **Testing**: <b>`soapcpp2 -T`</b> generates server source code that
  automatically implements echo message services for testing.  The
  <b>`testmsgr`</b> tool generates XML messages from message templates for
  server-side and client-side black-box testing, see
  [Test Messenger](../../testmsgr/html/index.html) documentation.  In addition,
  the soapcpp2 tool generates sample SOAP/XML input and output messages for
  verification and testing.

* **Speed**: the soapcpp2-generated high-performance XML serializers are ideal
  for building efficient Web services that are compute-intensive and are
  therefore best written in C or C++.

* **Portability**: source code is portable and compiles on Windows, Unix,
  Linux, Mac OS X, Symbian, VXWorks, and embedded systems that run WinCE,
  Symbian, embedded Linux, and so on. The memory footprint of gSOAP client and
  service applications is small.

* **Footprint**: input and output buffering is used to increase efficiency and
  reduce memory usage.  Input and output messages are not fully buffered or
  stored in a DOM unless required or specified.  As a result, large messages
  can be transmitted by low-memory devices.

🔝 [Back to table of contents](#)

# API documentation modules                             {#start-modules}

The API documentation is broken down into the following functional
documentation [modules](modules.html) that drill down into the lower-level API
of macros, functions, context and context variables, plugins and more:

<table class="directory">
<tr id="row_0_" class="even"><td class="entry"><span style="width:16px;display:inline-block;"></span><a class="el" href="group__group__debug.html" target="_self">Debugging and logging</a></td><td class="desc">This module defines compile-time flags and functions for run-time debugging and logging </td></tr>
<tr id="row_1_"><td class="entry"><span style="width:16px;display:inline-block;"></span><a class="el" href="group__group__with.html" target="_self">WITH_MACRO compile-time flags</a></td><td class="desc">This module defines the <code>WITH_MACRO</code> compile-time flags to configure the engine build </td></tr>
<tr id="row_2_" class="even"><td class="entry"><span style="width:16px;display:inline-block;"></span><a class="el" href="group__group__soap.html" target="_self">SOAP_MACRO compile-time values</a></td><td class="desc">This module defines the <code>SOAP_MACRO</code> compile-time values to configure the engine build </td></tr>
<tr id="row_3_"><td class="entry"><span style="width:16px;display:inline-block;"></span><a class="el" href="group__group__flags.html" target="_self">SOAP_MACRO run-time flags</a></td><td class="desc">This module defines the <code>SOAP_MACRO</code> run-time <code><a class="el" href="group__group__flags.html#ga72b0491c9cbf2071f0c96c7d29b719bb" title="The soap_mode flags to initialize the soap context, flags can be combined with | (bit-wise or) ...">soap_mode</a></code> flags to set the engine mode </td></tr>
<tr id="row_4_" class="even"><td class="entry"><span style="width:16px;display:inline-block;"></span><a class="el" href="group__group__errors.html" target="_self">SOAP_MACRO run-time error codes</a></td><td class="desc">This module defines the <code>SOAP_MACRO</code> run-time <code><a class="el" href="group__group__errors.html#gac0eadf8f72bacb5b41b750beaeca0444" title="Status and error codes are int values, a zero value or SOAP_OK (0) means no error, nonzero means error. ">soap_status</a></code> error codes returned by functions and stored in <code><a class="el" href="structsoap.html#ab85f5d42702963d13ea540bd9876e6d2" title="The soap context soap_status (int) error code of the last operation or SOAP_OK (zero) ...">soap::error</a></code> </td></tr>
<tr id="row_5_"><td class="entry"><span style="width:16px;display:inline-block;"></span><a class="el" href="group__group__context.html" target="_self">Context with engine state</a></td><td class="desc">This module defines the <code><a class="el" href="structsoap.html" title="Context with the engine state. ">soap</a></code> context structure with the engine state and functions to allocate, initialize, copy and delete contexts </td></tr>
<tr id="row_6_" class="even"><td class="entry"><span style="width:16px;display:inline-block;"></span><a class="el" href="group__group__callbacks.html" target="_self">Callback functions</a></td><td class="desc">This module defines the callback functions of the <code><a class="el" href="structsoap.html" title="Context with the engine state. ">soap</a></code> context to modify its behavior, as is done by plugins </td></tr>
<tr id="row_7_"><td class="entry"><span style="width:16px;display:inline-block;"></span><a class="el" href="group__group__ssl.html" target="_self">SSL/TLS context and functions</a></td><td class="desc">This module defines functions to set the SSL/TLS context for HTTPS and WS-Security </td></tr>
<tr id="row_8_" class="even"><td class="entry"><span style="width:16px;display:inline-block;"></span><a class="el" href="group__group__io.html" target="_self">HTTP and IO functions</a></td><td class="desc">This module defines functions for HTTP operations and functions for receiving and sending data </td></tr>
<tr id="row_9_"><td class="entry"><span style="width:16px;display:inline-block;"></span><a class="el" href="group__group__cookies.html" target="_self">HTTP cookie functions</a></td><td class="desc">This module defines functions to set and get HTTP cookies at the server side </td></tr>
<tr id="row_10_" class="even"><td class="entry"><span style="width:16px;display:inline-block;"></span><a class="el" href="group__group__s2s.html" target="_self">Conversion functions</a></td><td class="desc">This module defines conversion functions of values of various types to and from strings </td></tr>
<tr id="row_11_"><td class="entry"><span style="width:16px;display:inline-block;"></span><a class="el" href="group__group__namespace.html" target="_self">XML namespace tables</a></td><td class="desc">This module defines the <code><a class="el" href="struct_namespace.html" title="Structure of each row in a namespace table. ">Namespace</a></code> XML namespace structure and function to activate a table </td></tr>
<tr id="row_12_" class="even"><td class="entry"><span style="width:16px;display:inline-block;"></span><a class="el" href="group__group__header.html" target="_self">Header structure and functions</a></td><td class="desc">This module defines the <code><a class="el" href="struct_s_o_a_p___e_n_v_____header.html" title="SOAP Header structure generated by wsdl2h and soapcpp2. ">SOAP_ENV__Header</a></code> structure and <code><a class="el" href="group__group__header.html#ga08d35d1900a1982fdde6f78e43fc9635" title="If soap::header is NULL then allocate SOAP_ENV__Header header and set soap::header to point to it...">soap_header</a></code> function to allocate the header </td></tr>
<tr id="row_13_"><td class="entry"><span style="width:16px;display:inline-block;"></span><a class="el" href="group__group__fault.html" target="_self">Fault structure and functions</a></td><td class="desc">This module defines the <code><a class="el" href="struct_s_o_a_p___e_n_v_____fault.html" title="SOAP Fault structure generated by soapcpp2. ">SOAP_ENV__Fault</a></code> structure and functions to set and get fault information </td></tr>
<tr id="row_14_" class="even"><td class="entry"><span style="width:16px;display:inline-block;"></span><a class="el" href="group__group__dime.html" target="_self">DIME attachment functions</a></td><td class="desc">This module defines functions to set and get DIME attachments </td></tr>
<tr id="row_15_"><td class="entry"><span style="width:16px;display:inline-block;"></span><a class="el" href="group__group__mime.html" target="_self">MIME attachment functions</a></td><td class="desc">This module defines functions to set and get MIME/MTOM attachments </td></tr>
<tr id="row_16_" class="even"><td class="entry"><span style="width:16px;display:inline-block;"></span><a class="el" href="group__group__plugin.html" target="_self">Plugins and plugin registry functions</a></td><td class="desc">This module defines plugin registry functions to register plugins </td></tr>
<tr id="row_17_"><td class="entry"><span style="width:16px;display:inline-block;"></span><a class="el" href="group__group__threads.html" target="_self">Thread and mutex functions</a></td><td class="desc">This module defines portable thread and mutex functions </td></tr>
<tr id="row_18_" class="even"><td class="entry"><span style="width:16px;display:inline-block;"></span><a class="el" href="group__group__misc.html" target="_self">Miscellaneous functions</a></td><td class="desc">This module defines other useful functions </td></tr>
</table>

🔝 [Back to table of contents](#)

# Getting started                                                      {#start}

To start using gSOAP, you will need:

* The gSOAP software from <https://www.genivia.com/Products/downloads.html>.
  Select the gSOAP toolkit commercial edition to download (a download key is
  provided with the commercial-use license), or download the GPLv2 open source
  version from SourceForge <https://sourceforge.net/projects/gsoap2>.

* A C or C++ compiler.

* OpenSSL (or GNUTLS) and the Zlib libraries to enable SSL (HTTPS) and
  compression. These libraries are available for most platforms and are often
  already installed.

* Flex <http://flex.sourceforge.net> and Bison <http://www.gnu.org/software/bison>
  to build the soapcpp2 tool.  You can also build soapcpp2 without Bison and
  Flex installed, see <https://www.genivia.com/downloads.html> for details.

The gSOAP source code package includes:

* The <b>`wsdl2h`</b> data binding tool that converts WSDLs and XSDs to
  generate interface header files for soapcpp2.  The source code of the wsdl2h
  tool is located in <i>`gsoap/wsdl`</i>.

* The <b>`soapcpp2`</b> code generation tool that takes an interface
  header file and generates the C/C++ Web service binding implementation source
  code.  The source code of the soapcpp2 tool is located in <i>`gsoap/src`</i>.

* The run-time engine <i>`gsoap/stdsoap2.h`</i> and source code
  <i>`gsoap/stdsoap2.c`</i> for C and <i>`gsoap/stdsoap2.cpp`</i> for C++.
  These are compiled into the C libraries <i>`gsoap/libgsoap.a`</i> (without
  OpenSSL/GNUTLS for SSL/TLS), <i>`gsoap/libgsoapssl.a`</i> (with OpenSSL/GNUTLS for
  SSL/TLS and with <i>`gsoap/dom.c`</i> for DOM API), and the C++ libraries
  <i>`gsoap/libgsoap++.a`</i> (without OpenSSL/GNUTLS for SSL/TLS),
  <i>`gsoap/libgsoapssl++.a`</i> (with OpenSSL/GNUTLS for SSL/TLS and with
  <i>`gsoap/dom.cpp`</i> for DOM API).  There are two more versions of these
  libraries with HTTP cookies enabled.

* Several examples of gSOAP applications and other development tools that are
  build with wsdl2h and soapcpp2 are located in <i>`gsoap/samples`</i>.

* XML DOM API and the domcpp code generation tool located in
  <i>`gsoap/samples/dom`</i>, see also the
  [XML DOM API and domcpp](../../dom/html/index.html) documentation.

* JSON and XML-RPC libraries and the jsoncpp code generation tool located in
  <i>`gsoap/samples/xml-rpc-json`</i>, see also the
  [XML-RPC and JSON](../../xml-rpc-json/html/index.html) documentation.

* An XML Web API testing tool located in <i>`gsoap/samples/testmsgr`</i>, see
  also the [Test Messenger](../../testmsgr/html/index.html) documentation.

* Plugins to enhance the capabilities of the engine and to support WS
  protocols such as WS-Security, WS-Addressing, WS-ReliableMessaging,
  and WS-Discovery. The plugins are located in <i>`gsoap/plugin`</i>
  and <i>`gsoap/mod_gsoap`</i>.  Most but not all plugins are imported into
  interface header files for soapcpp2 with the `#import` directive.  See also
  API documentation Module \ref group_plugin.

* Custom serializers for several C and C++ types to enhance the capabilities of
  XML serialization, located in <i>`gsoap/custom`</i>.  Custom serializers
  are imported into interface header files for soapcpp2 with the `#import`
  directive.  This is usually done via a <i>`typemap.dat`</i> file for wsdl2h
  that specifies bindings for XML schema types to C/C++ types, including custom
  serializers when desired.

The wsdl2h and soapcpp2 tools and the gSOAP libraries are build with
<b>`./configure`</b> and <b>`make`</b>, see the download and installation page
<https://www.genivia.com/downloads.html> for the most recent versions of gSOAP
and gSOAP software installation details.  The examples and other tools are build
with <b>`./configure --enable-samples`</b> and <b>`make`</b>.

Non-SSL-enabled (that is, not HTTPS capable) versions of the binaries of the
wsdl2h and soapcpp2 tools are also included in the gSOAP package in
<i>`gsoap/bin`</i> for Windows and Mac OS platforms.  The SSL-enabled and
HTTPS-capable wsdl2h tool is only available for download from
<https://www.genivia.com/downloads.html> with a commercial-use license and
download key.

Although gSOAP tools are available in binary format for several platforms, the
code generated by these tools is equivalent. This means that the generated
source code files can be transferred between platforms and locally compiled.

The examples given in this document do not require the libraries of the engine
to be build, but rather show the use of the source code:
<i>`gsoap/stdsoap2.c`</i> and <i>`gsoap/dom.c`</i> (or
<i>`gsoap/stdsoap2.cpp`</i> and <i>`gsoap/dom.cpp`</i> for C++).
Using the source code instead of the libraries gives more control when we use
the <b>`-DWITH_MACRO`</b> and <b>`-DSOAP_MACRO`</b> compile-time options, see also Modules
\ref group_with and \ref group_soap.

🔝 [Back to table of contents](#)

## Where to find examples                                      {#start-examples}

Introductory examples can be found online at <https://www.genivia.com/dev.html>.
Additional examples can be found online at <https://www.genivia.com/examples.html>.

The gSOAP package also contains numerous examples in the
<i>`gsoap/samples`</i> directory.  Run <b>`make`</b> inside these directories
to build the example applications. The examples are meant to demonstrate
basic to advanced features of gSOAP.  Some examples are actually development
tools and libraries, such as Test Messenger located in
<i>`gsoap/samples/testmsgr`</i> to test XML Web APIs, the XML DOM API and
domcpp located in <i>`gsoap/samples/dom`</i> to generate XML DOM API source
code from XML files, the JSON API and jsoncpp located in
<i>`gsoap/samples/xml-rpc-json`</i> to generate JSON API source code from JSON
files.

Advanced examples include a streaming MTOM attachment server and client
application demonstrate high-performance file exchanges, located in
<i>`gsoap/samples/mtom-stream`</i>. An SSL-secure Web server application
demonstrates the generation of dynamic content for Web browsing and Web
services functionality at the same time, located in
<i>`gsoap/samples/webservice`</i>.

🔝 [Back to table of contents](#)

## Creating a SOAP/XML client application                        {#start-client}

This section explains the basics to develop a SOAP/XML client application in C
and C++.  We refer to <https://www.genivia.com/dev.html> for additional
introductory examples of SOAP/XML, XML REST and JSON applications in C and C++.

The wsdl2h tool imports one or more WSDLs and XML schemas and generates a gSOAP
interface file for soapcpp2 with familiar C/C++ header file syntax to define
the Web service operations and the C/C++ data types. The soapcpp2 tool
then takes this header file and generates XML serializers for the data types
(<i>`soapStub.h`</i>, <i>`soapH.h`</i>, and <i>`soapC.cpp`</i>), the
client-side stub functions (<i>`soapClient.cpp`</i>), and server-side skeleton
functions (<i>`soapServer.cpp`</i>).

The soapcpp2 tool can also generate WSDL definitions to implement a service
from scratch, i.e. without defining a WSDL first. This "closes the loop" in
that it enables Web services development from WSDL or directly from a set of
C/C++ operations declared as functions in an interface header file for soapcpp2
without the need for users to be experts in WSDL and XSD.

You only need to follow a few steps to execute the tools from the command line
or using a Makefile (see also MSVC++ project examples in the
<i>`gsoap/samples`</i> directory with tool integration in the MSVC++ IDE).
For example, to generate code for the calculator Web service, we run the wsdl2h
tool from the command line on the URL of the WSDL and use
[<b>`wsdl2h -o calc.h`</b> option <b>`-o calc.h`</b>](#wsdl2h-o) to specify the
<i>`calc.h`</i> interface file to output:

     wsdl2h -o calc.h http://www.genivia.com/calc.wsdl

This generates the <i>`calc.h`</i> service definition interface file with service
operation definitions and types to pass with the operation parameters. This
interface file is then input to the soapcpp2 tool to generate the stub
and skeleton source code and XML serialization functions. The <i>`calc.h`</i> file
includes documentation extracted form the WSDL and introductions to process the
file with soapcpp2.  You can use Doxygen (<http://www.doxygen.org>) to
automatically generate the documentation pages for your development from the
generated <i>`calc.h`</i> interface file.  To generate a markdown report for
your client, use <b>`soapcpp2 -r`</b> option <b>`-r`</b> which has more details
than the <i>`calc.h`</i> file.

In this example we will develop a C++ API for the calculator service. By
default, the wsdl2h tool assumes C++ with STL.  To build without STL, use
[<b>`wsdl2h -s`</b> option <b>`-s`</b>](#wsdl2h-s):

     wsdl2h -s -o calc.h http://www.genivia.com/calc.wsdl

@note Visual Studio IDE users should make sure to compile all gSOAP
source code files in C++ compilation mode. If you migrate to a project file
`.vcproj`, please set `CompileAs="2"` in your `.vcproj` file.

We have not yet generated the stub functions and serializers for our C++ client
application to invoke remote service operations. To do so, we run the soapcpp2
tool as follows:

     soapcpp2 -j -C -Iimport calc.h

Option <b>`-j`</b> (and alternatively option <b>`-i`</b>) indicates that we
want C++ proxy and server objects that include the client (and server) code,
option <b>`-C`</b> indicates client-side only files are generated (soapcpp2
generates both client stub functions and server skeleton functions by default). Option
<b>`-I`</b> is needed to import the <i>`stlvector.h`</i> file from the
<i>`gsoap/import`</i> directory located in the gSOAP source code package to
support serialization of STL vectors, when applicable.

We use the generated `soapcalcProxy` class declared and defined in
<i>`soapcalcProxy.h`</i> and <i>`soapcalcProxy.cpp`</i>, and
<i>`calc.nsmap`</i> XML namespace mapping table to access the Web service.
The <i>`soapcalcProxy.h`</i> file includes documentation on the proxy class.
The `soapcalcProxy` class is a proxy to invoke the remote service:

~~~{.cpp}
    // File: calclient.cpp
    #include "soapcalcProxy.h" 
    #include "calc.nsmap" 

    int main() 
    {
      calcProxy service; 
      double result; 
      if (service.add(1.0, 2.0, result) == SOAP_OK) 
        std::cout << "The sum of 1.0 and 2.0 is " << result << std::endl; 
      else 
        service.soap_stream_fault(std::cerr); 
      service.destroy(); // delete data and release memory 
    }
~~~

To complete the build, compile the code above and compile this with
the generated <i>`soapC.cpp`</i> and <i>`soapcalcProxy.cpp`</i> files, and link the
engine with <b>`-lgsoap++`</b>:

    c++ -o calcclient calcclient.cpp soapcalcProxy.cpp soapC.cpp -lgsoap++

Alternatively, compile <i>`gsoap/stdsoap2.cpp`</i>:

    c++ -o calcclient calcclient.cpp soapcalcProxy.cpp soapC.cpp stdsoap2.cpp

In both cases it is assumed that <i>`stdsoap2.h`</i> and the soapcpp2-generated
<i>`soapcalcProxy.h`</i>, <i>`soapStub.h`</i>, <i>`soapH.h`</i>, and
<i>`calc.nsmap`</i> files are located in the current directory.

Then run the example:

    ./calcclient
    The sum of 1.0 and 2.0 is 3

To build a pure C application, use [<b>`wsdl2h -c`</b> option <b>`-c`</b>](#wsdl2h-c) and
run <b>`soapcpp2 -C`</b> to generate the client stub functions and serializers:

     wsdl2h -c -o calc.h http://www.genivia.com/calc.wsdl
     soapcpp2 -C -Iimport calc.h

In this case our code uses a simple C function call to invoke the service and
we also need to explicitly delete data and the context with `::soap_end` and
`::soap_free`:

~~~{.cpp}
    // File: calclient.c
    #include "soapH.h"    // include the generated declarations
    #include "calc.nsmap" // include the generated namespace table

    int main() 
    {
      struct soap *soap = soap_new(); 
      double result; 
      if (soap_call_ns__add(soap, 1.0, 2.0, &result) == SOAP_OK) 
        printf("The sum of 1.0 and 2.0 is %lg\n", result); 
      else 
        soap_print_fault(soap, stderr); 
      soap_destroy(soap); // delete managed objects
      soap_end(soap);     // delete managed data and temporaries 
      soap_free(soap);    // finalize and delete the context
    }
~~~

To complete the build, compile the code above and compile this with
the generated <i>`soapC.c`</i> and <i>`soapClient.c`</i> files, and link the
engine with <b>`-lgsoap`</b>:

    cc -o calcclient calclient.c soapClient.c soapC.c -lgsoap

Alternatively, compile <i>`gsoap/stdsoap2.c`</i>:

    cc -o calcclient calclient.c soapClient.c soapC.c stdsoap2.c

In both cases it is assumed that <i>`stdsoap2.h`</i> and the soapcpp2-generated
<i>`soapStub.h`</i>, <i>`soapH.h`</i>, and <i>`calc.nsmap`</i> files are
located in the current directory.

Then run the example:

    ./calcclient
    The sum of 1.0 and 2.0 is 3

The calculator example is fairly simple and used here to illustrate the
development steps from code generation to running the application. The
development steps for large-scale XML applications is similar.

See <https://www.genivia.com/dev.html> for additional
introductory examples of SOAP/XML, XML REST and JSON applications in C and C++.
See <https://www.genivia.com/examples> and the examples located in the gSOAP
source code package in the <i>`gsoap/samples`</i> directory.

🔝 [Back to table of contents](#)

## Creating a SOAP/XML service application                      {#start-service}

This section explains the basics to develop a SOAP/XML service application in C
and C++.  We refer to <https://www.genivia.com/dev.html> for additional
introductory examples of SOAP/XML, XML REST and JSON applications in C and C++.

Developing a service application is easy. In this example we will
demonstrate a service deployed with the Common Gateway Interface (CGI) because
it is a simple mechanism. This is not the preferred deployment mechanism.
Because CGI is slow and stateless.  FastCGI improves the speed of CGI
applications, but is still stateless.  Faster services can be developed as a
stand-alone gSOAP HTTP/HTTPS servers as explained in this section further
below.

To deploy services in a public and possibly hostile environment, we recommend
the use of Apache module or IIS ISAPI extension to manage and protect services.
An Apache module plugin and ISAPI extension plugin is included in the gSOAP
package under <i>`gsoap/mod_gsoap`</i>.  For details, see:

* [gSOAP Apache module](../../apache/html/index.html) documentation

* [gSOAP ISAPI extension](../../isapi/html/index.html) documentation

To deploy gSOAP stand-alone services in a public environment make sure to
protect your service application as explained in Sections \ref safety and
\ref timeout.  See also our tutorials <https://www.genivia.com/tutorials.html>
with instructions to protect your online gSOAP Web APIs.

Let's get started.  Suppose we want to implement a simple CGI-based service that returns the time
in GMT.  For this example we start with an interface header file for soapcpp2,
<i>`currentTime.h`</i> which contains the service definitions. Such a file can
be obtained from a WSDL using wsdl2h when a WSDL is available. When a WSDL is
not available, you can define the service in C/C++ definitions in a newly
created interface header file and let the soapcpp2 tool generate the source
code and WSDL for you.

The `currenTime` service operation of our Web service has only one output
parameter, which is the current time defined in our <i>`currentTime.h`</i>
service specification:

~~~{.cpp}
    // File: currentTime.h 
    //gsoap ns service name: currentTime 
    //gsoap ns service namespace: urn:currentTime 
    //gsoap ns service location: http://www.yourdomain.com/currentTime.cgi 
    int ns__currentTime(time_t& response);
~~~

Note that we associate an XML namespace prefix `ns` and namespace name
`urn:currentTime` with the service WSDL and SOAP/XML messages. The gSOAP tools
use a special convention for identifier names that are part of a namespace: a
namespace prefix (`ns` in this case) followed by a double underscore `__`. This
convention is used to resolve namespaces and to avoid name
clashes. The `ns` namespace prefix is bound to the <i>`urn:currentTime`</i>
namespace name with the `//gsoap` directive. The `//gsoap` directives
are used to set the properties of the service, in this case the name,
namespace, and location endpoint.

The service implementation for CGI simply requires a `::soap_serve` call on a
`::soap` context created with `::soap_new`. The service operations are
implemented as functions, which are called by the service skeleton functions
via the service request dispatcher `::soap_serve`:

~~~{.cpp}
    // File: currentTime.cpp 
    #include "soapH.h"           // include the generated declarations 
    #include "currentTime.nsmap" // include the generated namespace table

    int main() 
    {
      // create soap context and serve one CGI-based request: 
      struct soap *soap = soap_new1(SOAP_XML_INDENT);
      soap_serve(soap);
      soap_destroy(soap); // delete managed class instances 
      soap_end(soap);     // delete managed data and temporaries 
      soap_free(soap);    // finalize and free the context
    }

    int ns__currentTime(struct soap *soap, time_t& response) 
    {
      response = time(0); 
      return SOAP_OK; 
    }
~~~

Note that we pass the `::soap` context with the engine context information to the
service operation function as the first argument. The `::soap` context comes in
handy to determine properties of the connection and to dynamically allocate
data with `::soap_malloc` or with the generated `soap_new_T` functions for
serializable types `T` that will be automatically deleted by calling
`::soap_destroy` and `::soap_end` when the service operation is done
and the service loop rolls over.

We run the soapcpp2 tool on the header file to generate the
server-side code:

     soapcpp2 -S currentTime.h

and then compile the CGI binary:

     c++ -o currentTime.cgi currentTime.cpp soapServer.cpp soapC.cpp stdsoap2.cpp

To activate the service, copy the <i>`currentTime.cgi`</i> binary to your
<i>`bin-cgi`</i> directory and set the proper file permissions.

The soapcpp2 tool generated the WSDL definitions
<i>`currentTime.wsdl`</i>. You can use the WSDL to advertise your service.
You don't need to use this WSDL to develop a gSOAP client. You can use the
<i>`currentTime.h`</i> file with <b>`soapcpp2 -C`</b> option <b>`-C`</b> to
generate client-side code.

Since CGI is very simple, a convenient aspect of CGI is that it exchanges
messages over standard input and output. Therefore, you can run the CGI binary
on the auto-generated example request XML file
<i>`currentTime.currentTime.req.xml`</i> to test your server:

     ./currentTime.cgi < currentTime.currentTime.req.xml

and this displays the HTTP server response:

    Status: 200 OK
    Server: gSOAP/2.8
    X-Frame-Options: SAMEORIGIN
    Content-Type: text/xml; charset=utf-8
    Content-Length: 460
    Connection: close
~~~{.xml}
    <?xml version="1.0" encoding="UTF-8"?>
    <SOAP-ENV:Envelope xmlns:SOAP-ENV="http://schemas.xmlsoap.org/soap/envelope/" xmlns:SOAP-ENC="http://schemas.xmlsoap.org/soap/encoding/" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xmlns:xsd="http://www.w3.org/2001/XMLSchema" xmlns:ns="urn:currentTime">
            <SOAP-ENV:Body>
                    <ns:currentTimeResponse>
                            <response>2018-10-25T17:17:03Z</response>
                    </ns:currentTimeResponse>
            </SOAP-ENV:Body>
    </SOAP-ENV:Envelope>
~~~

The above approach works also for C. Just use <b>`soapcpp2 -c -S`</b> option <b>`-c`</b> in
addition to the <b>`-S`</b> option to generate C code. Of course, in C we use
pointers instead of references and the <i>`currentTime.h`</i> file should be
adjusted to use C syntax and types.

Run <b>`soapcpp2 -r -c -S`</b> option <b>`-r`</b> to generate a
<i>`soapReadme.md`</i> report.  This report includes many details about the
service operations and serializable C/C++ data types declared in the interface
header file.  This markdown file can be converted to HTML or other document
formats with tools such as [Doxygen](www.doxygen.org).

A more elegant server implementation in C++ can be obtained by using
<b>`soapcpp2 -j -S`</b> option <b>`-j`</b> (or option <b>`-i`</b>) to generate C++
client-side proxy and server-side service objects as classes that you can use
to build clients and services in C++. The option removes the generation of
<i>`soapClient.cpp`</i> and <i>`soapServer.cpp`</i>, since these are no longer
needed when we have classes that implement the client and server logic:

     soapcpp2 -j -S currentTime.h

This generates <i>`soapcurrentTimeService.h`</i> and
<i>`soapcurrentTimeService.cpp`</i> files, as well as auxiliary files
<i>`soapStub.h`</i>, <i>`soapH.h`</i>, and <i>`soapC.cpp`</i> and <i>`currentTime.nsmap`</i>.
The <i>`soapcurrentTimeService.h`</i> file includes documentation on the service class.

Now using the `currentTimeService` class we can improve the CGI service application:

~~~{.cpp}
    // File: currentTime.cpp 
    #include "soapcurrentTimeService.h" // include the proxy declarations 
    #include "currentTime.nsmap"        // include the generated namespace table

    int main() 
    {
      // create server and serve one CGI-based request: 
      currentTimeService server(SOAP_XML_INDENT); 
      server.serve(); 
      server.destroy(); 
    }

    int currentTimeService::currentTime(time_t& response) 
    {
      response = time(0); 
      return SOAP_OK; 
    }
~~~

We compile this with:

     c++ -o currentTime.cgi currentTime.cpp soapcurrentTimeService.cpp soapC.cpp stdsoap2.cpp

and install the binary as a CGI application.

To run the server as a stand-alone iterative (i.e. non-multi-hreaded) server
on port 8080 until a the accept times out or an error occurs:

~~~{.cpp}
    if (server.run(8080) != SOAP_OK) 
      server.soap_stream_fault(std::cerr); 
~~~

To run the server as a stand-alone iterative server on port 8080 while ignoring
common errors until a TCP occurs:

~~~{.cpp}
    while (server.run(8080) != SOAP_OK) 
    {
      if (server.soap->error == SOAP_TCP_ERROR)
        break;
      server.soap_stream_fault(std::cerr); 
    }
~~~

To implement stand-alone and multi-threaded services, see
Sections \ref stand-alone and \ref mt. These stand-alone Web Services handle multiple
SOAP requests by spawning a thread for each request. Thread pooling is also an
option. The use of Apache modules and ISAPI extensions to deploy gSOAP services
is recommended to ensure load balancing, access control, tracing, and so on.

For more information on server-side service classes, see
Section \ref object . For more information on client-side proxy classes,
see Section \ref proxy .

See <https://www.genivia.com/dev.html> for additional
introductory examples of SOAP/XML, XML REST and JSON applications in C and C++.
See <https://www.genivia.com/examples> and the gSOAP source code package
<i>`gsoap/samples`</i> for more examples.

🔝 [Back to table of contents](#)

# Introduction to XML data bindings                              {#databindings}

This section is a basic introduction to XML data bindings in C/C++.  Because
gSOAP offers many options to implement XML data bindings, we wrote a separate
[C and C++ XML data bindings](../../databinding/html/index.html) document on
this topic that contains an in-depth discussion of XML schema mappings to C/C++
types, using wsdl2h with <i>`typemap.dat`</i> to customize these bindings,
memory management to allocate and release serializable types, and how to use
[soapcpp2 options](#soapcpp2options) to generate deep data structure copy and
delete functions for serializable types.

Basically, the C/C++ XML data binding in gSOAP provides and automated mechanism to serialize
any C and C++ data structure in XML and to deserialize XML back into C/C++ data
structures. To facilitate XML data bindings, a WSDL or an XML schema (XSD file)
can converted with wsdl2h into a set of serializable C or C++ data type
declarations.  These C/C++ type declarations can be readily incorporated into
your application to manipulate XML directly as C/C++ data structures with much
more ease than DOM or SAX.  For example, XML schema strings are just C/C++
strings, XML schema enumerations are C/C++ enums, XML schema complex types are
just structs or classes in C/C++, and so on.  In this way, an automatic mapping
between XML elements of the XML schema and members of a class is created.  No
DOM traversals and SAX events are needed.

More importantly, the XML C/C++ data binding makes XML manipulation type safe.
That is, the type safety of C/C++ ensures that only valid XML documents can be
parsed and generated.

The wsdl2h tool performs the mapping of WSDL and XML schemas to C and/or C++
automatically. The output of wsdl2h is a "data binding interface file" which is
simply an annotated C/C++ header file with the serializable C/C++ data types
that represent XML schema components.  This file also includes comments and
documentation of the serializable data types.  For WSDLs, also functions are
declared in this interface file that represent XML Web services operations.

Let's illustrate XML data bindings with an example. Suppose we have an XML
document with a book record:

<div class="alt">
~~~{.xml}
    <book isbn="1234567890"> 
      <title>Farewell John Doe</title> 
      <publisher>ABC's is our Name</publisher> 
    </book>
~~~
</div>

An example XML schema (XSD file) that defines the book element and type could
be:

<div class="alt">
~~~{.xml}
    <schema ...> 
      <element name="book"> 
        <complexType> 
          <sequence> 
            <element name="title" type="string" minOccurs="1"/> 
            <element name="publisher" type="string" minOccurs="1"/> 
          </sequence> 
          <attribute name="isbn" type="unsignedLong" use="required"/> 
        </complexType> 
      </element> 
    </schema>
~~~
</div>

Now, using wsdl2h we translate this XML schema that defines the book type and
root element into a C++ class definition:

~~~{.cpp}
    class book 
    { public:
      @ ULONG64 isbn; 
        std::string title; 
        std::string publisher; 
    };
~~~

Note that annotations such as `@` are used to distinguish attributes from
elements. These annotations are gSOAP-specific and are handled by the soapcpp2
tool that reads this generated interface file and generates the XML data
binding implementation with serializers for the data types declared in the
interface file.

That is, the soapcpp2 tool generates all the necessary code to parse and
generate XML for `book` objects. Validation constraints such as
<i>`minOccurs="1"`</i> and <i>`use="required"`</i> are included in the
generated code as checks that are enforced with the `#SOAP_XML_STRICT`
flag.

To write the XML representation of a book object instantiated in our C++
application, we first create a `::soap` engine context and use it with
`soap_write_book` (a function generated by soapcpp2) to write the object in XML
to standard output:

~~~{.cpp}
    struct soap *soap = soap_new1(SOAP_XML_INDENT); // new context
    book bk; 
    bk.isbn = 1234567890; 
    bk.title = "Farewell John Doe"; 
    bk.publisher = "ABC's is our Name"; 
    if (soap_write_book(soap, &bk) != SOAP_OK) 
      ... // error
    soap_destroy(soap); // delete managed class instances 
    soap_end(soap);     // delete managed data and temporaries 
    soap_free(soap);    // finalize and free the context
~~~

The `::soap` context holds the engine state and run-time flags, such as
`#SOAP_XML_INDENT`, serialization options, and other I/O settings. This means
that we can simply set the output file descriptor `int ::soap::sendfd` or output
stream `std::ostream* ::soap::os` of the context to redirect the content to
a file or string. Also, when serializing a graph with `#SOAP_XML_GRAPH` rather
than an XML tree, the XML output contains id-ref attributes to reference nodes
in the XML graph, similar to the way SOAP encoding with multi-reference
id-ref/href works, see Section \ref bindings  for details.

To read the XML representation from standard input into a book class instance:

~~~{.cpp}
    struct soap *soap = soap_new1(SOAP_XML_STRICT); // new context
    book bk; 
    if (soap_read_book(soap, &bk) != SOAP_OK)
      ... // error
    else 
      cout << bk.isbn << ", " << bk.title << ", " << bk.publisher << endl; 
    ... // further use of bk
    soap_destroy(soap); // delete managed class instances 
    soap_end(soap);     // delete managed data and temporaries 
    soap_free(soap);    // finalize and free the context
~~~

Automatic built-in strict XML validation is enabled with `#SOAP_XML_STRICT`,
which ensures that data members are present so we can safely print them in this
example, thus ensuring consistency of data with the XML schema.

We can set the `int ::soap::recvfd` file descriptor or the
`std::istream* ::soap::is` input stream to read from a file or string stream
instead of stdin.

The `::soap_destroy` and `::soap_end` calls deallocate the deserialized
data, so use these with care. Memory management is automatic in gSOAP
to avoid leaks.

The above example uses a very simple example schema. The gSOAP toolkit handles
all XML schema constructs defined by the XML schema standard. The toolkit is
also able to serialize pointer-based C/C++ data structures, including
cyclic graphs, structs/classes, unions, enums, STL containers, and even
special data types such as `struct tm`. Therefore, the toolkit works in two
directions: from WSDL/schema to C/C++ and from C/C++ to WSDL/schema.

The gSOAP toolkit also handles multiple schemas defined in multiple namespaces.
Normally the namespace prefixes of XML namespaces are added to the C/C++ type
definitions to ensure type uniqueness. For example, if we would combine two
schemas in the same application where both schemas define a `book` object,
we need to resolve this conflict. In gSOAP this is done using namespace
prefixes, rather than C++ namespaces (research has pointed out that XML
namespaces are not equivalent to C++ namespaces). Thus, the `book` class
might actually be bound to an XML namespace and the class would be named
`ns__book`, where `ns` is bound to the corresponding namespace.

For example, the following run-time flags are available to control serialization
as an XML tree or graph:

~~~{.cpp}
    struct soap *soap = soap_new();
    soap_set_mode(soap, SOAP_XML_TREE);  // use this for XML without id-ref (no cycles!)
    soap_set_mode(soap, SOAP_XML_GRAPH); // or use this for XML with id-ref (including cycles)
~~~

Other flags can be used to format XML, see Section \ref flags .

For more details on XML databinding support for C and C++, see
Section \ref bindings and the 
[C and C++ XML data bindings](../../databinding/html/index.html) document.

🔝 [Back to table of contents](#)

# A quick user guide   {#guide}

This section of the user guide presents a quick overview to get started with
gSOAP.  In principle, XML SOAP and REST clients and services can be developed
in C and C++ with the soapcpp2 tool without a detailed understanding of
XML, XML schema, WSDL, and the XML SOAP protocol.

🔝 [Back to table of contents](#)

## How to build Web API clients        {#client}

The implementation of a client application that invokes remote service
operations by serializing application data in XML for the remote operation
request, also called XML marshalling in remote procedure calling.  This
requires a "stub function", also called "service proxy", for each
service operation that the client invokes. The primary stub's responsibility is
to serialize the parameter data in XML, send the request with the parameters to
the designated SOAP service over the wire, to wait for the response, and to
deserialize the parameter data of the response when it arrives.
With the stub function in place, the client application invokes a remote
service operation as if it would invoke a local function. To write a client stub
function in C or C++ by hand is a tedious task, especially if the input and
output parameters of a service operation contain elaborate data structures
that must meet XML validation constraints.  Fortunately, the wsdl2h tool
and soapcpp2 tool automate the development of SOAP/XML Web service client and
server applications.
 
The soapcpp2 tool generates the necessary gluing code (the client stub functions and
server skeleton functions) to build web service clients and services. The input to the soapcpp2
tool consists of an interface file with familiar C/C++ header file syntax.
This interface header file can be automatically generated from a WSDL (Web
Service Description Language) documentation of a service with the gSOAP
wsdl2h tool.

The following command:

     wsdl2h -o calc.h http://www.genivia.com/calc.wsdl

generates the <i>`calc.h`</i> interface header file for soapcpp2.
The WSDL specification may consist of multiple imported WSDL files and XSD
schema files.  The WSDLs and XSDs are then translated to C or C++, replacing
WSDL service operation by C/C++ functions and XML schema data types by C/C++
data types.

To generate C code, we use [<b>`wsdl2h -c`</b> option <b>`-c`</b>](#wsdl2h-c):

     wsdl2h -c -o calc.h http://www.genivia.com/calc.wsdl

For more details on the wsdl2h tool and its options, see Section \ref wsdlin .

When upgrading gSOAP to a newer version it is often not necessary to perform
this first step again, since newer versions are backward compatible to previous
interface header files generated by wsdl2h.

Looking into the file <i>`calc.h`</i> we see that the SOAP service methods are
specified as function prototypes. For example, the `add` function to add
two doubles is declared as:

~~~{.cpp}
    int ns2__add(double a, double b, double& result);
~~~

The `ns2__add` function uses an XML namespace prefix to distinguish it from
operations defined in other namespaces, thus nicely preventing name clashes in
this way.  The convention to add an XML namespace prefix to the names of
operations, types, and `struct` and `class` members is universally used by the
gSOAP tools and automatically added by wsdl2h.

Next, the <i>`calc.h`</i> header file is input to the soapcpp2 tool to generate
the gluing code's logic to invoke the Web service from a client application:

    soapcpp2 calc.h

The function prototypes in <i>`calc.h`</i> are converted by the soapcpp2 tool
to stub functions for remote calls:

* <i>`soapStub.h`</i> annotated copy of the header file's definitions.

* <i>`soapH.h`</i> XML serializers declarations

* <i>`soapC.cpp`</i> XML serializers implementations

* <i>`soapClient.cpp`</i> the client calling stub functions

The logic of the generated <i>`soapClient.cpp`</i> stub functions allow client
applications to seamlessly interact with existing SOAP Web services as
illustrated by the client code example in the next section.

The input and output parameters of a service operation may be primitive
data types or compound data types, such as containers and pointer-based
linked data structures. These are defined in the interface header file, which is either
generated by the wsdl2h tool or it may be specified by hand.  The soapcpp2 tool
automatically generates XML serializers and XML deserializers for these
data types to enable the generated stub functions to serialize the
contents of the parameters of the service operations in XML.

The soapcpp2 tool also generates "skeleton functions" <i>`soapServer.cpp`</i>
for each of the service operations specified in the interface header file. The
skeleton functions can be readily used to implement one or more of the service
operations in a new XML Web service.

To develop C++ client applications, a useful option to use with soapcpp2 is
<i>`-j`</i> to generate proxy classes to invoke services, instead of global
functions:

    soapcpp2 -j calc.h

The function prototypes in <i>`calc.h`</i> are converted by the soapcpp2 tool
to the following function:

* <i>`soapStub.h`</i> annotated copy of the header file's definitions.

* <i>`soapH.h`</i> XML serializers declarations

* <i>`soapC.cpp`</i> XML serializers implementations

* <i>`soapcalcProxy.h`</i> the client proxy class

* <i>`soapcalcProxy.cpp`</i> the client proxy class implementation

To use the proxy class, `#include "soapcalcProxy.h"` and compile and link
`soapcalcProxy.cpp`.  See the following section.

🔝 [Back to table of contents](#)

### Example        {#example1}

The `add` service operation declared in the <i>`calc.h`</i> file obtained
with the wsdl2h tool in the previous section, adds two doubles.  The WSDL
description of the service provides the endpoint to invoke the service
operations and the XML namespace used by the operations:

* Endpoint URL: <i>`http://websrv.cs.fsu.edu/~engelen/calcserver.cgi`</i>

* XML namespace: <i>`urn:calc`</i>

Each SOAP-specific service operation also has a "SOAP action", which is an
optional string to identify the operation, which is mainly used with WS-Addressing.
The request and response messages for SOAP RPC-encoded services are simply
represented by C functions with input and output parameters. For the `add`
operation, the SOAP binding details are:

* SOAP style: RPC 

* SOAP encoding: encoded 

* SOAP action: ""

This information is translated to the wsdl2h-generated interface header file
with the service definitions.  The <i>`calc.h`</i> header file for C++
generated by wsdl2h contains the following directives and declarations:

~~~{.cpp}
    //gsoap ns2 schema namespace:   urn:calc
    //gsoap ns2 schema form:        unqualified

    //gsoap ns2 service name:       calc
    //gsoap ns2 service type:       calcPortType
    //gsoap ns2 service port:       http://websrv.cs.fsu.edu/~engelen/calcserver.cgi 
    //gsoap ns2 service namespace:  urn:calc 
    //gsoap ns2 service transport:  http://schemas.xmlsoap.org/soap/http

    //gsoap ns2 service method-protocol:    add SOAP 
    //gsoap ns2 service method-style:       add rpc 
    //gsoap ns2 service method-encoding:    add http://schemas.xmlsoap.org/soap/encoding/ 
    //gsoap ns2 service method-action:      add "" 
    int ns2__add(double a, double b, double& result);
~~~

The actual contents may vary depending on the release version and the options
used to control the output.

The other calculator service operations are similar and were elided for clarity.

The `//gsoap` directives are interpreted by the soapcpp2 tool to generate code
that is compliant to the SOAP protocol. For this service the SOAP protocol with
the "SOAP 1.1 RPC encoding style" is used.  This produces XML messages with the familiar SOAP envelope and body in the SOAP 1.1 namespace and a <i>`SOAP-ENV:encodingStyle`</i> attribute for SOAP RPC encoding (a simple XML serialization format) as can be seen in the XML request message:

<div class="alt">
~~~{.xml}
    <?xml version="1.0" encoding="UTF-8"?>
    <SOAP-ENV:Envelope
        xmlns:SOAP-ENV="http://schemas.xmlsoap.org/soap/envelope/"
        xmlns:SOAP-ENC="http://schemas.xmlsoap.org/soap/encoding/"
        xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
        xmlns:xsd="http://www.w3.org/2001/XMLSchema"
        xmlns:ns="http://tempuri.org/ns.xsd">
      <SOAP-ENV:Body SOAP-ENV:encodingStyle="http://schemas.xmlsoap.org/soap/encoding/>
        <ns:add>
          <a>1.0</a>
          <b>2.0</b>
        </ns:add>
      </SOAP-ENV:Body>
    </SOAP-ENV:Envelope>
~~~
</div>

Another style is "document/literal"
which is also defined by SOAP 1.1/1.2.

To use SOAP document/literal style is indicated for each service operation as follows in the
interface file specification, which also switches to the SOAP 1.2 protocol:

~~~{.cpp}
    //gsoap ns2 service method-protocol:    add SOAP1.2
    //gsoap ns2 service method-style:       add document
    //gsoap ns2 service method-encoding:    add literal
    //gsoap ns2 service method-action:      add "" 
    int ns2__add(double a, double b, double& result);
~~~

This produces XML messages with the familiar SOAP envelope and body with the SOAP 1.2 namespace and without the encodingStyle attribute as can be seen in the XML request message:

<div class="alt">
~~~{.xml}
    <?xml version="1.0" encoding="UTF-8"?>
    <SOAP-ENV:Envelope
        xmlns:SOAP-ENV="http://www.w3.org/2003/05/soap-envelope"
        xmlns:SOAP-ENC="http://www.w3.org/2003/05/soap-encoding"
        xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
        xmlns:xsd="http://www.w3.org/2001/XMLSchema"
        xmlns:ns="urn:calc">
      <SOAP-ENV:Body>
        <ns:add>
          <a>1.0</a>
          <b>2.0</b>
        </ns:add>
      </SOAP-ENV:Body>
    </SOAP-ENV:Envelope>
~~~ 
</div>

REST instead of SOAP is specified with `HTTP` instead of `SOAP` with the
`//gsoap <prefix> service method-protocol:` directive to carry our XML messages
using the HTTP POST method without a SOAP envelope:

~~~{.cpp}
    //gsoap ns2 service method-protocol:    add HTTP
    int ns2__add(double a, double b, double& result);
~~~

This produces non-SOAP XML messages with HTTP POST.  For example:

<div class="alt">
~~~{.xml}
    <?xml version="1.0" encoding="UTF-8"?>
    <ns:add
        xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
        xmlns:xsd="http://www.w3.org/2001/XMLSchema"
        xmlns:ns="urn:calc">
      <a>1.0</a>
      <b>2.0</b>
    </ns:add>
~~~
</div>

Likewise you can specify `POST`, `PUT`, and `GET` HTTP methods for direct XML
messaging instead of `SOAP`.  However, all XML Web services protocols such as
WS-Security and WS-Addressing require SOAP to include the SOAP Headers.

Note that the function declaration itself and the client-side calling stub and
server-side skeleton functions are unchanged.  The internals of the generated
functions are changed to accommodate the protocol specified with the directives.

For more details about the `//gsoap` directives, see
Section \ref directives .

So as you can see, the Web service operations are declared as
function prototypes.  When the parameters of the function are structs and
classes, then all of these interdependent data types are included in the
wsdl2h-generated header file.

In this simple example the parameters are primitive types. The calculator `add`
operation takes two double floats `a` and `b`, and returns the sum in `result`.
By convention, all parameters are input parameters except the last parameter
which is an output parameter or specified
as `void` for one-way messages. The last parameter is always a single output
parameter, if not `void`.  A `struct` or `class` is used to wrap multiple
output parameters, see also Section \ref multiple . This last parameter must be
a pointer or reference. By contrast, the input parameters support pass by value
or by pointer, but not pass by C++ reference due to complications when
generating compilable source code for the stub and skeleton functions.

The function prototype associated with a service operation always returns an
`int`. The return value indicates success with `#SOAP_OK` (zero)
or failure with a nonzero value. See Section \ref errcodes 
for the error codes. 

The role of the namespace prefix (`ns2__`) in the service operation name specified as a
function prototype associates an XML namespace with the service operation as a qualified name, i.e. qualified by means of a namespace prefix.  This is discussed in detail in Section \ref namespace .
Basically, the namespace prefix is added to a function name or type name with a pair of underscores,
as in `ns2__add`, where `ns2` is the
namespace prefix and `add` is the service operation name. This mechanism ensures uniqueness of operations and types associated with a service.

When using wsdl2h it is strongly recommended to set the namespace prefix to a name of your
choice by modifying the <i>`typemap.dat`</i> file that is used by wsdl2h.  This
file can be copied from <i>`gsoap/typemap.dat`</i> and modified in the local
directory where you run wsdl2h. This avoids problems when running wsdl2h on
multiple WSDLs where the sequence of prefixes `ns1`, `ns2`, and so on are
arbitrarily assigned to the services. To choose a prefix name for all the
operations and types of a service, say prefix `c__` for the calculator service,
add the following line to <i>`typemap.dat`</i>:

    c = "urn:calc"

and rerun wsdl2h. Moreover, the <i>`typemap.dat`</i> configures wsdl2h to use
specific bindings and data types for services. The result is that `c__add` is used to uniquely identify the operation rather than the more arbitrary name `ns2__add`.

A note on the use of underscores in names: a single
underscore in an identifier name will be translated into a dash in XML, because
dashes are more frequently used in XML compared to underscores, see
Section \ref idtrans .  Double underscores separate the namespace prefix from the unqualified part of the qualified name.

Next, the soapcpp2 tool is invoked from the command line to process the <i>`calc.h`</i> service definitions:

     soapcpp2 calc.h

The tool generates the client stub functions for the service operations.
Stub functions can be invoked
by a client program to invoke the remote service operations.
The interface of the generated stub function is identical to the function prototype in the <i>`calc.h`</i> service definition file, but with additional parameters to pass the engine's context `::soap`, an endpoint URL (or NULL for the default), and a SOAP action (or NULL for the default):

~~~{.cpp}
    int soap_call_c__add(struct soap *soap, char *URL, char *action, double a, double b, double& result);
~~~

This stub function is saved in <i>`soapClient.cpp`</i>. The file <i>`soapC.cpp`</i>
contains the serializer and deserializer functions for the data
types used by the stub. You can use [<b>`wsdl2h -c`</b> option <b>`-c`</b>](#wsdl2h-c) to
generate pure C code.  Likewise, <b>`soapcpp2 -c`</b> option <b>`-c`</b> generates pure C code, if the input interface file is written in C of course.

The `::soap` parameter of the stub function shown above must be a valid pointer
to a `::soap` context. The `URL` parameter when non-NULL overrides the default
endpoint address defined by the WSDL and the `//gsoap <prefix> service port:`
directive.  The `action` parameter when non-NULL overrides the
default SOAP action defined by the WSDL and the
`//gsoap <prefix> service method-action:` directive.

The following example C/C++ client program uses the generated stub function to invoke the remote service operation:

~~~{.cpp}
    #include "soapH.h"    // include the generated declarations
    #include "calc.nsmap" // include the generated namespace table 

    int main() 
    {
      double sum; 
      struct soap *soap = soap_new(); // the context 
      if (soap_call_c__add(soap, NULL, NULL, 1.0, 2.0, &sum) == SOAP_OK) 
        std::cout << "Sum = " << sum << std::endl; 
      else // an error occurred 
        soap_print_fault(soap, stderr); // display the SOAP fault message on the stderr stream 
      soap_destroy(soap); // delete managed class instances 
      soap_end(soap);     // delete managed data and temporaries 
      soap_free(soap);    // finalize and free the context
      return 0; 
    }
~~~

The `soap_call_c__add` call returns `#SOAP_OK` (zero) on success and a nonzero error on
failure.  When an error occurred you can print the fault message with the
`soap_print_fault(struct soap*, FILE*)` function. Use `soap_sprint_fault(struct soap*, char *buf, size_t len)` to save the fault message to a string buffer `buf[0...len-1]`. Use `soap_stream_fault(struct soap*, std::ostream&)` to send the fault message to a stream.

The following functions are used to explicitly set up a `::soap` context and finalize it:

* `soap_new()` allocates, initializes, and returns a pointer to a new `::soap` context.

* `soap_new1(soap_mode mode)` allocates, initializes and sets both in- and out-mode flags to the same mode, and returns a pointer to a new `::soap` context.

* `soap_new2(soap_mode imode, soap_mode omode)` allocates, initializes and sets in- and out-mode flags, and returns a pointer to a new `::soap` context.

* `soap_copy(struct soap*)` allocates and returns a new context by copying the given context with `::soap_copy_context`, such that the new context returned does not share any data with the given context.

* `soap_init(struct soap*)` initializes a stack-allocated `::soap` context in  C, when not using `::soap_new`, for C++ use the `::soap` constructors instead.

* `soap_init1(struct soap*, soap_mode mode)` initializes a stack-allocated `::soap` context, when not using `::soap_new1`, and sets both in- and out-mode flags to the same mode, for C++ use the `::soap` constructors instead.

* `soap_init2(struct soap*, soap_mode imode, soap_mode omode)` initializes a stack-allocated `::soap` context, when not using `::soap_new2`, and sets in- and out-mode flags, for C++ use the `::soap` constructors instead.

* `soap_done(struct soap*)` finalizes a context but does not delete the specified `::soap` context (for example when stack-allocated).  The context can be re-initialized afterwards with `::soap_init`.  In C++, a stack-allocated context invokes this function in its destructor when the context is deleted.

* `soap_free(struct soap*)` finalizes and deletes the given context, when created with `::soap_new` or `::soap_copy`.

A `::soap` context can be reused as many times as necessary and does not need to be reinitialized when doing so.
However, a new context is required for each thread that runs independently to guarantee exclusive access
to a `::soap` context by each thread.

Also the use of any client calls within an active service operation implemented at the server side requires a new context, since `::soap_serve` is still processing with the current `::soap` context that must be maintained while the service operation and response has not been completed yet.

The soapcpp2 code generator tool generates a service proxy class for C++ client applications (and service objects for server applications) with the <b>`soapcpp2 -j`</b> option <b>`-j`</b> (or <b>`-i`</b> option):

     soapcpp2 -j calc.h

The proxy is defined in:

* <i>`soapcalcProxy.h`</i> client proxy class 

* <i>`soapcalcProxy.cpp`</i> client proxy class

Without the <b>`-j`</b> option, C-like <i>`soapClient.cpp`</i> and <i>`soapServer.cpp`</i> source codes are is generated.
Use option <b>`-i`</b>
as an alternative to <b>`-j`</b> to generate classes with the same functionality,
but that are inherited from the `::soap` struct.
With the <b>`-j`</b> option, the `::soap` engine context is a pointer member of the generated proxy and service classes and can therefore be shared with other proxy or
service class instances. The choice of option to use is application-dependent, but the choice is also important when services are chained to serve requests on the same server port, see Section \ref chaining .

The generated C++ proxy class initializes the context and offers the service interface as a collection of methods:

~~~{.cpp}
    #include "soapcalcProxy.h" // get proxy 
    #include "calc.nsmap"      // include the generated namespace table 

    int main() 
    {
      calcProxy calc(SOAP_XML_INDENT); 
      double sum; 
      if (calc.add(1.0, 2.0, sum) == SOAP_OK) 
        std::cout << "Sum = " << sum << std::endl; 
      else 
        calc.soap_stream_fault(std::cerr); 
      calc.destroy();
      return calc.soap->error; // nonzero when error 
    }
~~~

The proxy constructor takes context mode parameters to initialize the context, e.g. `#SOAP_XML_INDENT` in this example.

The code is compiled and linked with <i>`soapcalcProxy.cpp`</i>, <i>`soapC.cpp`</i>,
and <i>`gsoap/stdsoap2.cpp`</i>.

The proxy class name is extracted from the WSDL content and may not always be in a short format. You can change this directive to customize the service name:

~~~{.cpp}
    //gsoap c service name: calc
~~~

then rerun soapcpp2 to generate code that uses the new name.

When the example client application is invoked, a SOAP request is performed:

    POST /~engelen/calcserver.cgi HTTP/1.1 
    Host: websrv.cs.fsu.edu 
    User-Agent: gSOAP/2.8 
    Content-Type: text/xml; charset=utf-8 
    Content-Length: 464 
    Connection: close 
    SOAPAction: "" 
<div class="alt">
~~~{.xml}
    <?xml version="1.0" encoding="UTF-8"?> 
    <SOAP-ENV:Envelope 
        xmlns:SOAP-ENV="http://schemas.xmlsoap.org/soap/envelope/" 
        xmlns:SOAP-ENC="http://schemas.xmlsoap.org/soap/encoding/" 
        xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" 
        xmlns:xsd="http://www.w3.org/2001/XMLSchema" 
        xmlns:c="urn:calc"> 
      <SOAP-ENV:Body SOAP-ENV:encodingStyle="http://schemas.xmlsoap.org/soap/encoding/"> 
        <c:add> 
          <a>1</a> 
          <b>2</b> 
        </c:add> 
      </SOAP-ENV:Body> 
    </SOAP-ENV:Envelope>
~~~
</div>

The SOAP response message is:

    HTTP/1.1 200 OK 
    Date: Wed, 05 May 2010 16:02:21 GMT 
    Server: Apache/2.0.52 (Scientific Linux) 
    Content-Length: 463 
    Connection: close 
    Content-Type: text/xml; charset=utf-8 
<div class="alt">
~~~{.xml}
    <?xml version="1.0" encoding="UTF-8"?> 
    <SOAP-ENV:Envelope 
        xmlns:SOAP-ENV="http://schemas.xmlsoap.org/soap/envelope/" 
        xmlns:SOAP-ENC="http://schemas.xmlsoap.org/soap/encoding/" 
        xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" 
        xmlns:xsd="http://www.w3.org/2001/XMLSchema" 
        xmlns:ns="urn:calc"> 
      <SOAP-ENV:Body SOAP-ENV:encodingStyle="http://schemas.xmlsoap.org/soap/encoding/"> 
        <ns:addResponse> 
          <result>3</result> 
        </ns:addResponse> 
      </SOAP-ENV:Body> 
    </SOAP-ENV:Envelope>
~~~
</div>

A client can invoke a sequence of service operations, like so:

~~~{.cpp}
    #include "soapcalcProxy.h" // get proxy 
    #include "calc.nsmap"      // include the generated namespace table 

    int main() 
    {
      calcProxy calc(SOAP_IO_KEEPALIVE); // keep-alive improves connection performance 
      double sum = 0.0; 
      double val[] = { 5.0, 3.5, 7.1, 1.2 }; 
      for (int i = 0; i < 4; i++) 
        if (calc.add(sum, val[i], sum)) 
          return calc.soap->error; 
      std::cout << "Sum = " << sum << std::endl; 
      calc.destroy();
      return 0; 
    }
~~~

In the example shown above, no deserialized data is deallocated until `calc.destroy()`.  To deallocate deserialized data between the calls we change the loop as follows:

~~~{.cpp}
    for (int i = 0; i < 4; i++) 
    {
      if (calc.add(sum, val[i], sum)) 
        return calc.soap->error; 
      calc.destroy(); 
    }
~~~

Deallocation is safe here, since the float values were copied and saved in
`sum`. In other scenarios we should make sure data is copied to local data
structures when the data should be preserved.  There are tooling options for
deep copy and delete of entire data structures to simplify this task, see
Section \ref deep.

To delegate deletion to another context for later removal, use
`soap_delegate_deletion(struct soap *soap_from, struct soap *soap_to)`.  For
example:

~~~{.cpp}
    struct soap soap; 
    soap_init(&soap); 
    {
      // create proxy 
      calcProxy calc; 
      ... // data produced, e.g. deserialized data with client-side calls
      soap_delegate_deletion(&calc, &soap); 
      calc.destroy();
    }
    ... // data can still be used
    soap_destroy(&soap); // delete managed class instances 
    soap_end(&soap);     // delete managed data and temporaries 
    soap_done(&soap);    // finalize the context
~~~

In C we use [<b>`wsdl2h -c`</b> option <b>`-c`</b>](#wsdl2h-c) to generate C.  The example
client calculator program would be written as:

~~~{.cpp}
    #include "soapH.h" 
    #include "calc.nsmap" 

    int main() 
    {
      struct soap soap; 
      double sum = 0.0; 
      double val[] = { 5.0, 3.5, 7.1, 1.2 }; 
      int i; 
      soap_init1(&soap, SOAP_IO_KEEPALIVE); 
      for (i = 0; i < 4; i++) 
        if (soap_call_c__add(&soap, NULL, NULL, sum, val[i], &sum)) 
          return soap.error; 
      printf("Sum = %lg\n", sum); 
      soap_end(&soap); 
      soap_done(&soap); 
      return 0; 
    }
~~~

The code above is compiled and linked with the soapcpp2-generated
<i>`soapClient.c`</i> and <i>`soapC.c`</i> files, and <i>`gsoap/stdsoap2.c`</i>
(or compile with <b>`-bgsoap`</b>).

🔝 [Back to table of contents](#)

### XML namespace considerations        {#namespace}

The declaration of the `ns2__add` function prototype discussed in the previous
section uses the namespace prefix `ns2__` of the service operation XML namespace,
which is distinguished by a pair of underscores in the function name to
separate the namespace prefix from the service operation name.  The purpose of
a namespace prefix is to associate a service operation name with a service in
order to prevent naming conflicts, e.g. to distinguish identical service
operation names used by different services.

Note that the XML response of the service example uses a namespace prefix that
may be different (e.g. <i>`ns`</i>) as long as it bound to the same namespace
name <i>`urn:calc`</i> through the <i>`xmlns:ns="urn:calc"`</i> binding.  The
use of namespace prefixes and namespace names is also required to enable SOAP
applications to validate the content of SOAP messages.  The namespace name in
the service response is verified by the stub function by using the information
supplied in a namespace mapping table that is required to be part of gSOAP
client and service application codes.  The table is accessed at run time to
resolve namespace bindings, both by the generated stub's data structure
serializer for encoding the client request and by the generated stub's data
structure deserializer to decode and validate the service response. The
namespace mapping table should not be part of the header file input to the
soapcpp2 tool. Service details including namespace bindings may be provided
with gSOAP directives in a header file, see Section \ref directives .

The namespace mapping table is:

~~~{.cpp}
    struct Namespace namespaces[] = 
    {
      // { "prefix", "URI", "URI-pattern" (optional) } 
      { "SOAP-ENV", "http://schemas.xmlsoap.org/soap/envelope/"}, // must be first 
      { "SOAP-ENC", "http://schemas.xmlsoap.org/soap/encoding/"}, // must be second 
      { "xsi",      "http://www.w3.org/2001/XMLSchema-instance"}, // must be third 
      { "xsd",      "http://www.w3.org/2001/XMLSchema"},          // 2001 XML Schema 
      { "ns2",      "urn:calc"},                                  // given by the service description 
      { NULL,       NULL}                                         // end of table 
    };
~~~

The first four namespace entries in the table consist of the standard namespaces used by the SOAP protocol. In fact, the
namespace mapping table is explicitly declared to enable a programmer to specify the SOAP encoding style and to allow the
inclusion of namespace-prefix with namespace-name bindings to comply to the namespace requirements of a specific SOAP service. For
example, the namespace prefix <i>`ns2`</i>, which is bound to <i>`urn:calc`</i> by the namespace mapping table shown
above, is used by the generated stub function to encode the `add` request. This is performed automatically by the soapcpp2
tool by using the `ns2` prefix of the `ns2__add` method name specified in the <i>`calc.h`</i> header file. In
general, if a function name of a service operation, `struct` name, `class` name, `enum` name, or member name of a
`struct` or `class` has a pair of underscores, the name has a namespace prefix that must be defined in the namespace
mapping table.

The namespace mapping table will be output as part of the SOAP Envelope by the stub function. For example:

<div class="alt">
~~~{.xml}
    <SOAP-ENV:Envelope xmlns:SOAP-ENV="http://schemas.xmlsoap.org/soap/envelope/" 
        xmlns:SOAP-ENC="http://schemas.xmlsoap.org/soap/encoding/" 
        xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" 
        xmlns:xsd="http://www.w3.org/2001/XMLSchema" 
        xmlns:ns2="urn:calc">
    ...
~~~
</div>

The namespace bindings will be used by a SOAP service to validate the SOAP request.

🔝 [Back to table of contents](#)

### Example        {#example2}

The incorporation of namespace prefixes into C++ identifier names is necessary to distinguish service operations that
share the same name but are provided by separate Web services and/or
organizations. It avoids potential name clashes, while sticking to the C
syntax since C has no support for namespaces. The C++ proxy classes generated with <b>`soapcpp2 -j`</b> option <b>`-j`</b> (or option <b>`-i`</b>) drop the
namespace prefix from the method names.

The namespace prefix convention is also be applied to non-primitive types. For
example, `class` names are prefixed to avoid name clashes when the same name is
used by multiple XML schemas. This ensures that the XML databinding never
suffers from conflicting schema content. For example:

~~~{.cpp}
    class e__Address // an electronic address from schema 'e' 
    { public:
        char *email; 
        char *url; 
    }; 
    class s__Address // a street address from schema 's' 
    { public:
        char *street; 
        int number; 
        char *city; 
    };
~~~

At this point you may ask why the gSOAP tools do no use C++ namespaces to implement XML namespaces.  The answer is not too complicated. XML namespaces are semantically more rich than C++ namespaces.  For example, XML schema complexTypes may reference elements in another schema and these elements may be qualified in XML.  There could also be element name clashes when element names are the same but referenced in different schemas.  In gSOAP this is resolved by naming `struct` and `class` members with the namespace prefix notation, thereby ensuring that name clashes among members cannot occur.

An instance of `e__Address` is encoded by the generated serializer for this type as an Address element with namespace prefix `e`:

<div class="alt">
~~~{.xml}
    <e:Address> 
      <email>me@home</email> 
      <url>www.me.com</url> 
    </e:Address>
~~~
</div>

While an instance of `s__Address` is encoded by the generated serializer for this type as an Address element with namespace prefix `s`:

<div class="alt">
~~~{.xml}
    <s:Address> 
      <street>Technology Drive</street> 
      <number>5</number> 
      <city>Softcity</city> 
    </s:Address>
~~~
</div>

The namespace mapping table of the client program must have entries for `e` and `s` that refer to the XML Schemas of the data types:

~~~{.cpp}
    struct Namespace namespaces[] = 
    {
      { "SOAP-ENV", "http://schemas.xmlsoap.org/soap/envelope/" },
      { "SOAP-ENC", "http://schemas.xmlsoap.org/soap/encoding/" },
      { "xsi", "http://www.w3.org/2001/XMLSchema-instance" },
      { "xsd", "http://www.w3.org/2001/XMLSchema" },
      { "e", "http://www.me.com/schemas/electronic-address" }, 
      { "s", "http://www.me.com/schemas/street-address" }, 
      { NULL, NULL }
    };
~~~

This table is automatically generated by soapcpp2 and saved as a <i>`.nsmap`</i> file that can be included in the source code.

🔝 [Back to table of contents](#)

### How to generate C++ client proxy classes        {#proxy}

Proxy classes for C++ client applications are automatically generated by the soapcpp2 tool, as was shown in Section \ref example1 .

A new and improved code generation capability is implemented in soapcpp2 for C++ proxy classes by
using <b>`soapcpp2 -j`</b> option <b>`-j`</b> (or option <b>`-i`</b>). These new proxy classes have a cleaner interface and offer more
capabilities compared to the gSOAP 2.7 proxy and service classes.

In C++ you can also use [<b>`wsdl2h -q name`</b> option <b>`-q name`</b>](#wsdl2h-q) to generate
the proxy class and serializers in the specified C++ namespace `name`. This is very useful if you want to
create multiple proxies for services by repeated use of wsdl2h and then
combine them in one code.  Alternatively, you can run wsdl2h just once on
all service WSDLs and have soapcpp2 generate multiple proxies for you.
The latter approach does not use C++ namespaces and actually reduces the overall
amount of source code by avoiding code duplication.

To illustrate the generation of a proxy class, the
<i>`calc.h`</i> header file example of the previous section used.

~~~{.cpp}
    // Content of file "calc.h": 
    //gsoap ns2 schema namespace:   urn:calc
    //gsoap ns2 schema form:        unqualified

    //gsoap ns2 service name:       calc
    //gsoap ns2 service type:       calcPortType
    //gsoap ns2 service port:       http://websrv.cs.fsu.edu/~engelen/calcserver.cgi 
    //gsoap ns2 service namespace:  urn:calc 
    //gsoap ns2 service transport:  http://schemas.xmlsoap.org/soap/http

    //gsoap ns2 service method-protocol:    add SOAP 
    //gsoap ns2 service method-style:       add rpc 
    //gsoap ns2 service method-encoding:    add http://schemas.xmlsoap.org/soap/encoding/ 
    //gsoap ns2 service method-action:      add "" 
    int ns2__add(double a, double b, double& result); 

    //gsoap ns2 service method-protocol:    sub SOAP 
    //gsoap ns2 service method-style:       sub rpc 
    //gsoap ns2 service method-encoding:    sub http://schemas.xmlsoap.org/soap/encoding/ 
    //gsoap ns2 service method-action:      sub "" 
    int ns2__sub(double a, double b, double& result); 

    //gsoap ns2 service method-protocol:    mul SOAP 
    //gsoap ns2 service method-style:       mul rpc 
    //gsoap ns2 service method-encoding:    mul http://schemas.xmlsoap.org/soap/encoding/ 
    //gsoap ns2 service method-action:      mul "" 
    int ns2__mul(double a, double b, double& result); 
~~~

The `namespace` directives declare the XML schema namespace and WSDL service namespace, which are the same in this example.  The `name`, `type`, and `port` directives declare service details, which are used by soapcpp2 to name the proxy class, the WSDL port type, and the service location port which is the endpoint URL of the service.  The messaging `transport` mode is HTTP.

The groups of four directives per service operation declare the operation SOAP style (RPC) and encoding (SOAP encoded), and SOAP action string, which is optional and used mostly for HTTP-based routing of messages and by WS-Addressing. 
In this example, the protocol is SOAP 1.1 RPC encoding.  More recent uses of SOAP focus on document/literal style messaging, which is also declared with directives without affecting the rest of the interface header file.
For `//gsoap` directive details, see Section \ref directives .

Run <b>`soapcpp2 -j`</b> on this interface header file with the calculator service specification to generate <i>`soapcalcProxy.cpp`</i> and <i>`soapcalcProxy.h`</i> with the proxy class declaration:

~~~{.cpp}
    #include "soapH.h" 

    class SOAP_CMAC calcProxy {
      public:
        // Context to manage proxy IO and data
        struct soap *soap;
        // flag indicating that this context is owned by this proxy when context is shared
        bool soap_own;
        // Endpoint URL of service 'calcProxy' (change as needed)
        const char *soap_endpoint;
        // Variables globally declared in calc.h, if any
        // Construct a proxy with new managing context
        calcProxy();
        // Copy constructor
        calcProxy(const calcProxy& rhs);
        // Construct proxy given a shared managing context
        calcProxy(struct soap*);
        // Construct proxy given a shared managing context and endpoint URL
        calcProxy(struct soap*, const char *endpoint);
        // Constructor taking an endpoint URL
        calcProxy(const char *endpoint);
        // Constructor taking input and output mode flags for the new managing context
        calcProxy(soap_mode iomode);
        // Constructor taking endpoint URL and input and output mode flags for the new managing context
        calcProxy(const char *endpoint, soap_mode iomode);
        // Constructor taking input and output mode flags for the new managing context
        calcProxy(soap_mode imode, soap_mode omode);
        // Destructor deletes non-shared managing context only (use destroy() to delete deserialized data)
        virtual ~calcProxy();
        // Initializer used by constructors
        virtual void calcProxy_init(soap_mode imode, soap_mode omode);
        // Return a copy that has a new managing context with the same engine state
        virtual calcProxy *copy();
        // Copy assignment
        calcProxy& operator=(const calcProxy&);
        // Delete all deserialized data (uses soap_destroy() and soap_end())
        virtual void destroy();
        // Delete all deserialized data and reset to default
        virtual void reset();
        // Disables and removes SOAP Header from message by setting soap->header = NULL
        virtual void soap_noheader();
        // Get SOAP Header structure (i.e. soap->header, which is NULL when absent)
        virtual SOAP_ENV__Header *soap_header();
        // Get SOAP Fault structure (i.e. soap->fault, which is NULL when absent)
        virtual SOAP_ENV__Fault *soap_fault();
        // Get SOAP Fault subcode QName string (NULL when absent)
        virtual const char *soap_fault_subcode();
        // Get SOAP Fault string/reason (NULL when absent)
        virtual const char *soap_fault_string();
        // Get SOAP Fault detail XML string (NULL when absent)
        virtual const char *soap_fault_detail();
        // Close connection (normally automatic, except for send_X ops)
        virtual int soap_close_socket();
        // Force close connection (can kill a thread blocked on IO)
        virtual int soap_force_close_socket();
        // Print fault
        virtual void soap_print_fault(FILE*);
    #ifndef WITH_LEAN
    #ifndef WITH_COMPAT
        // Print fault to stream
        virtual void soap_stream_fault(std::ostream&);
    #endif
        // Write fault to buffer
        virtual char *soap_sprint_fault(char *buf, size_t len);
    #endif
        // Web service operation 'add' (returns SOAP_OK or error code)
        virtual int add(double a, double b, double *result)
        { return this->add(NULL, NULL, a, b, result); }
        virtual int add(const char *soap_endpoint, const char *soap_action, double a, double b, double *result);
        // Web service operation 'sub' (returns SOAP_OK or error code)
        virtual int sub(double a, double b, double *result)
        { return this->sub(NULL, NULL, a, b, result); }
        virtual int sub(const char *soap_endpoint, const char *soap_action, double a, double b, double *result);
        // Web service operation 'mul' (returns SOAP_OK or error code)
        virtual int mul(double a, double b, double *result)
        { return this->mul(NULL, NULL, a, b, result); }
        virtual int mul(const char *soap_endpoint, const char *soap_action, double a, double b, double *result);
    };
~~~

The above shows the raw source code with comments generated by soapcpp2. To
obtain an annotated markdown document with documented the proxy and service
classes and documented data types used by the service operations, run
<b>`soapcpp2 -r`</b> option <b>`-r`</b> to generate a <i>`soapReadme.md`</i>
report.  This markdown file can be converted to HTML or other document formats
with tools such as [Doxygen](www.doxygen.org).

This generated proxy class can be included into a client application together with the generated namespace table as shown in this example:

~~~{.cpp}
    #include "soapcalcProxy.h"
    #include "calc.nsmap"

    int main() 
    {
      calcProxy calc(SOAP_XML_INDENT); 
      double r; 
      if (calc.add(1.0, 2.0, r) == SOAP_OK) 
        std::cout << "Sum of 1.0 and 2.0 is " << r << std::endl; 
      else 
        calc.soap_stream_fault(std::cerr); 
      return 0; 
    }
~~~

The XML message sent by the client proxy:

<div class="alt">
~~~{.xml}
    <SOAP-ENV:Envelope xmlns:ns2="urn:calc" ...> 
      <SOAP-ENV:Body> 
        <ns2:add> 
          <a>1.0</a> 
          <b>2.0</b> 
        </ns2:add> 
      </SOAP-ENV:Body> 
    </SOAP-ENV:Envelope> 
~~~
</div>

The XML message returned by the service:

<div class="alt">
~~~{.xml}
    <SOAP-ENV:Envelope xmlns:ns2="urn:calc" ...> 
      <SOAP-ENV:Body> 
        <ns2:addResponse> 
          <result>3.0</result>
        </ns2:addResponse> 
      </SOAP-ENV:Body> 
    </SOAP-ENV:Envelope> 
~~~
</div>

With <b>`soapcpp2 -j`</b> option <b>`-j`</b>, the constructor of the proxy class allocates and initializes a `::soap` context as a pointer member of the class.  With <b>`soapcpp2 -i`</b> option <b>`-i`</b> the proxy class is derived from the `::soap` struct instead and this context is initialized when the proxy class constructor is invoked.

To place the proxy class in a C++ namespace `name`, use <b>`soapcpp2 -q name`</b> option <b>`-q name`</b>.
See Section \ref soapcpp2options.

🔝 [Back to table of contents](#)

### XSD type serialization                                           {#encoding}

XML Web services use XML schemas to define the data types of XML data
structures in the XML message payloads.  The default encoding assumed by
soapcpp2 is SOAP 1.1 document/literal style messaging but this is easily
changed using options, such as <b>`-2`</b> (SOAP 1.2), <b>`-0`</b> (non-SOAP XML
REST), and <b>`-e`</b> (SOAP with RPC encoding). See Section
\ref soapcpp2options.

Primitive XSD types are mapped to C/C++ primitive types by means of `typedef`
declarations in the interface header file for soapcpp2 to generate the XML data
binding serialization code.  A `typedef` binds an XML schema type name to a
C/C++ type.  For example:

~~~{.cpp}
    typedef char *xsd__anyURI;        // encode xsd:anyURI values as a strings
    typedef std::string xsd__NMTOKEN; // encode xsd:NMTOKEN values as strings
    typedef float xsd__float;         // encode xsd:float values as floats
~~~

This simple mechanism informs the soapcpp2 tool to generate serializers and
deserializers that explicitly encode and decode the primitive C++ types as
built-in primitive XSD types.  At the same time, the use of `typedef` does not
force any source code rewriting of a client or Web service application because
the internal types used by the application are not required to be changed by using this `typedef` mechanism.

The built-in XSD types are covered by `typedef` mappings and we could map XSD <i>`xsd:base64Binary`</i> and <i>`xsd:hexBinary`</i> to strings, but that would be cumbersome since the application should populate the strings properly.  Instead, we can defined the following structs or classes to contain binary content that the generated serializers serialize in base64 and hexadecimal, respectively:

~~~{.cpp}
    struct xsd__base64Binary
    {
        unsigned char *__ptr; // points to raw binary data
        int __size;           // length of raw binary data
    };

    struct xsd__hexBinary
    {
        unsigned char *__ptr; // points to raw binary data
        int __size;           // length of raw binary data
    };
~~~

Also <i>`xsd:boolean`</i> in C can be mapped to a `enum`:

~~~{.cpp}
    enum xsd__boolean { false_, true_ };
~~~

The trailing underscores are removed in XML payloads and are used here to avoid potential name clashes with C++ `false` and `true` keywords.

Annotations with `typedef` types introduce validation constraints that are verified by the XML parser:

~~~{.cpp}
    typedef float ns__price "%.2g" ;                     // 2 fractional digits
    typedef int ns__percent 0:100 ;                      // integer between 0 and 100
    typedef std::string ns__letter "[a-z]" 1:1 ;         // string of length 1 with letter (using XSD regex)
    typedef unsigned long long xsd__positiveInteger 1: ; // positive integer
~~~

The soapcpp2 tool generates a schema with the following types (<i>`xsd::positiveInteger`</i> is a built-in XSD type and therefore omitted from the generated schema):

<div class="alt">
~~~{.xml}
    <simpleType name="price"><!-- ns__price -->
      <restriction base="xsd:float">
        <fractionDigits value="2"/>
      </restriction>
    </simpleType>
    <simpleType name="percent"><!-- ns__percent -->
      <restriction base="xsd:int">
        <minInclusive value="0"/>
        <maxInclusive value="100"/>
      </restriction>
    </simpleType>
    <simpleType name="letter"><!-- ns__letter -->
      <restriction base="xsd:string">
        <pattern value="[a-z]"/>
        <length value="1"/>
      </restriction>
    </simpleType>
~~~
</div>

For more details on mapping C/C++ data types with their value space constraints to XML schema and vice versa, see [C and C++ XML Data Bindings](../../databinding/html/index.html) documentation.

🔝 [Back to table of contents](#)

### Example        {#example3}

Reconsider the calculator example of the previous sections, now rewritten with an explicit XSD type for `double` to illustrate the effect:

~~~{.cpp}
    // Contents of file "calc.h": 
    typedef double xsd__double; 
    int ns2__add(xsd__double a, xsd__double b, xsd__double& result);
~~~

In C a pointer is used instead of a reference for the output parameter `result`.

The soapcpp2 tool generates the client stub function:

~~~{.cpp}
    int soap_call_ns2__add(struct soap *soap, char *URL, char *action, double a, double b, double& result);
~~~

This means that the client application does not need to be rewritten and can still call the client stub or use the generated C++ proxy class as before.

Likewise, `typedef` can be used to declare user-defined schema types:

~~~{.cpp}
    // Contents of file "calc.h": 
    typedef double ns2__number; 
    int ns2__add(ns2__number a, ns2__number b, ns2__number& result);
~~~

This lets soapcpp2 generate a WSDL and XML schema that declares the <i>`ns2:number`</i> type:

<div class="alt">
~~~{.xml}
    <schema targetNamespace="urn:calc"
        xmlns:SOAP-ENV="http://schemas.xmlsoap.org/soap/envelope/"
        xmlns:SOAP-ENC="http://schemas.xmlsoap.org/soap/encoding/"
        xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
        xmlns:xsd="http://www.w3.org/2001/XMLSchema"
        xmlns:ns2="urn:calc"
        xmlns="http://www.w3.org/2001/XMLSchema"
        elementFormDefault="unqualified"
        attributeFormDefault="unqualified">
      <import namespace="http://schemas.xmlsoap.org/soap/encoding/"/>
      <simpleType name="number"><!-- ns2__number -->
        <restriction base="xsd:double">
        </restriction>
      </simpleType>
    </schema>
~~~
</div>

🔝 [Back to table of contents](#)

### How to change the response element name        {#response}

There is no standardized convention for the response element name in a SOAP RPC
encoded response message, although it is recommended that the response element
name is the method name ending with "<i>`Response`</i>". For example, the
response element of <i>`add`</i> is <i>`addResponse`</i>.

The response element name can be specified explicitly using a `struct` or
`class` declaration in the interface header file for soapcpp2. This name should
be qualified by a namespace prefix, just as the operation name should use a
namespace prefix. The `struct` or `class` name represents the SOAP response
element name used by the service. Consequently, the output parameter of the
service operation must be declared as a member of the `struct` or `class`.  The
use of a `struct` or a `class` for the service response is fully SOAP 1.1/1.2
compliant. In fact, the absence of a `struct` or `class` indicates to the
soapcpp2 tool to automatically generate a `struct` for the response which is
internally used by a stub.

🔝 [Back to table of contents](#)

### Example        {#example4}

Reconsider the calculator service operation specification which can be
rewritten with an explicit declaration of a SOAP response element as follows:

~~~{.cpp}
    // Contents of file "calc.h": 
    struct ns2__addResponse { double result; }; 
    int ns2__add(double a, double b, struct ns2__addResponse& r);
~~~

This wraps the output parameters in a struct `ns2__addResponse`.
Note that in C a pointer is used instead of a reference for the output wrapper parameter `r`.

In this example we just made an explicit output parameter wrapper, meaning that
SOAP request and response messages will be the same as before without this
wrapper:

<div class="alt">
~~~{.xml}
    <SOAP-ENV:Envelope xmlns:ns2="urn:calc" ...> 
      <SOAP-ENV:Body> 
        <ns2:addResponse> 
          <result>3.0</result>
        </ns2:addResponse> 
      </SOAP-ENV:Body> 
    </SOAP-ENV:Envelope> 
~~~
</div>

We can use any other name with a namespace prefix for the wrapper struct or class to change the response element name.

🔝 [Back to table of contents](#)

### How to specify multiple output parameters        {#multiple}

The soapcpp2 tool uses the convention that the last parameter of the function
prototype declaration of a service operation in an interface header file is the
output parameter of the service operation.  All other parameters are considered
input parameters of the service operation.

To specify a service operation with multiple output parameters, a `struct` or
`class` is declared to wrap the service operation response parameters, see also
Section \ref response .  The name of the `struct` or `class` should have a
namespace prefix, just as the service method name. The members of the `struct`
or `class` are the output parameters of the service operation.

The order of the input parameters in the function prototype and the order of
the output parameters (the members of the wrapper `struct` or `class`) is not
significant. However, the SOAP 1.1 RPC encoding specification states that input and output
parameters may be treated as anonymous parameter names, which requires a
particular ordering of these parameters, see Section \ref anonymous .

🔝 [Back to table of contents](#)

### Example        {#example5}

As an example, consider a hypothetical service operation `getNames` with a
single input parameter `SSN` and two output parameters `first` and `last`. This
can be specified as:

~~~{.cpp}
    // Contents of file "getNames.h": 
    int ns3__getNames(char *SSN, struct ns3__getNamesResponse { char *first; char *last; } &r);
~~~

The soapcpp2 tool takes this header file as input and generates source code for
the function `soap_call_ns3__getNames`. When invoked by a client application,
this stub function produces the XML request message:

<div class="alt">
~~~{.xml}
    <SOAP-ENV:Envelope xmlns:ns3="urn:names" ...> 
      <SOAP-ENV:Body>
        <ns3:getNames> 
          <SSN>999 99 9999</SSN> 
        </ns3:getNames> 
      </SOAP-ENV:Body>
    </SOAP-ENV:Envelope>
~~~
</div>

The response XML message:

<div class="alt">
~~~{.xml}
    <SOAP-ENV:Envelope xmlns:ns3="urn:names" ...> 
      <SOAP-ENV:Body>
        <ns3:getNamesResponse> 
          <first>John</first> 
          <last>Doe</last> 
        </ns3:getNamesResponse> 
      </SOAP-ENV:Body>
    </SOAP-ENV:Envelope>
~~~
</div>

where <i>`first`</i> and <i>`last`</i> are the output parameters wrapped
in the `getNamesResponse` struct.

As another example, consider a service operation `copy` with an input parameter
and an output parameter with identical parameter names (this is not prohibited
by the SOAP 1.1/1.2 protocols). This can be specified using a wrapper
struct for the output parameters, thus avoiding the name clash we would run
into without this wrapper:

~~~{.cpp}
    // Content of file "copy.h": 
    int X_rox__copy_name(char *name, struct X_rox__copy_nameResponse { char *name; } &r);
~~~

The use of a `struct` or `class` for the service operation response enables the
declaration of service operations that have parameters that are passed both as
input and output parameters.

The soapcpp2 tool takes the <i>`copy.h`</i> header file as input and generates
the `soap_call_X_rox__copy_name` stub function. When invoked by a client application,
the stub function produces the XML request message:

<div class="alt">
~~~{.xml}
    <SOAP-ENV:Envelope xmlns:X-rox="urn:copy" ...> 
      <SOAP-ENV:Body>
        <X-rox:copy-name> 
          <name>hello</name> 
        </X-rox:copy-name> 
      </SOAP-ENV:Body>
    </SOAP-ENV:Envelope>
~~~
</div>

The response XML message:

<div class="alt">
~~~{.xml}
    <SOAP-ENV:Envelope xmlns:X-rox="urn:copy" ...> 
      <SOAP-ENV:Body>
        <X-rox:copy-nameResponse> 
          <name>SOAP</name> 
        </X-rox:copy-nameResponse> 
      </SOAP-ENV:Body>
    </SOAP-ENV:Envelope>
~~~
</div>

The name will be parsed and decoded by the stub function and returned as the `name`
member of the `struct X_rox__copy_nameResponse &r` parameter.

You can use the service operation name as a wrapper for the response:

~~~{.cpp}
    // Content of file "copy.h": 
    int X_rox__copy_name(char *name, struct X_rox__copy_name &r);
~~~

where the struct `X_rox__copy_name` is generated by soapcpp2 automatically and does not need to be redeclared.

The response XML message in this case would be:

<div class="alt">
~~~{.xml}
    <SOAP-ENV:Envelope xmlns:X-rox="urn:copy" ...> 
      <SOAP-ENV:Body>
        <X-rox:copy-name> 
          <name>SOAP</name> 
        </X-rox:copy-name> 
      </SOAP-ENV:Body>
    </SOAP-ENV:Envelope>
~~~
</div>

🔝 [Back to table of contents](#)

### How to specify output parameters with compound data types        {#compound}

If the single output parameter of a service operation is a compound data type
such as a `struct` or `class` it is necessary to specify the response element
of the service operation as a `struct` or `class` at all times.  Otherwise, the
output parameter will be considered the response element (!), because of the
response element specification convention used by gSOAP, as discussed in
Section \ref response .

🔝 [Back to table of contents](#)

### Example        {#example6}

This is best illustrated with an example. Suppose we have a Flighttracker service
that provides real time flight information. It
requires an airline code and flight number as parameters.  The service
operation name is `getFlightInfo` that has two string parameters: the
airline code and flight number.
The method returns a `getFlightResponse` response
element with a `return` output parameter that is of a compound data type `FlightInfo`.
The type `FlightInfo` is represented by a `class` in the header file:

~~~{.cpp}
    // Contents of file "flight.h": 
    typedef char *xsd__string; 
    class ns2__FlightInfo 
    { public: 
        xsd__string airline; 
        xsd__string flightNumber; 
        xsd__string altitude; 
        xsd__string currentLocation; 
        xsd__string equipment; 
        xsd__string speed; 
    }; 
    struct ns1__getFlightInfoResponse { ns2__FlightInfo return_; }; 
    int ns1__getFlightInfo(xsd__string param1, xsd__string param2, struct ns1__getFlightInfoResponse &r);
~~~

The response element `ns1__getFlightInfoResponse` is explicitly declared and it
has one member: `return_` of type `ns2__FlightInfo`.  Note that `return_` has a
trailing underscore to avoid a name clash with the `return` keyword, see
Section \ref idtrans  for details on the translation of C/C++ identifiers to
XML names.

The soapcpp2 tool generates the `soap_call_ns1__getFlightInfo` stub function. Here is
an example fragment of a client application that uses this proxy to request
flight information:

~~~{.cpp}
    #include "soapH.h"
    #include "ns1.nsmap"
    
    int main()
    {
      struct soap soap; 
      soap_init1(&soap, SOAP_XML_INDENT); 
      ns2__FlightInfo flight;
      if (soap_call_ns1__getFlightInfo(&soap, "testvger.objectspace.com/soap/servlet/rpcrouter", "urn:galdemo:flighttracker", "UAL", "184", flight))
        soap_print_fault(&soap, stderr); // nonzero return means that an error occurred
      else
        cout << flight.return_.equipment << " flight " << flight.return_.airline << flight.return_.flightNumber 
             << " traveling " << flight.return_.speed << " mph " << " at " << flight.return_.altitude 
             << " ft, is located " << flight.return_.currentLocation << endl;
      soap_destroy(&soap); // delete managed class instances 
      soap_end(&soap);     // delete managed data and temporaries 
      soap_done(&soap);    // finalize the context
    }

    struct Namespace namespaces[] = 
    {
      { "SOAP-ENV", "http://schemas.xmlsoap.org/soap/envelope/" }, 
      { "SOAP-ENC","http://schemas.xmlsoap.org/soap/encoding/" }, 
      { "xsi", "http://www.w3.org/2001/XMLSchema-instance" }, 
      { "xsd", "http://www.w3.org/2001/XMLSchema" }, 
      { "ns1", "urn:galdemo:flighttracker" }, 
      { "ns2", "http://galdemo.flighttracker.com" }, 
      { NULL, NULL } 
    };
~~~

When invoked by a client application, the stub function produces the XML request:

    POST /soap/servlet/rpcrouter HTTP/1.1 
    Host: testvger.objectspace.com 
    Content-Type: text/xml 
    Content-Length: 634 
    SOAPAction: "urn:galdemo:flighttracker" 
<div class="alt">
~~~{.xml}
    <?xml version="1.0" encoding="UTF-8"?> 
    <SOAP-ENV:Envelope
        xmlns:SOAP-ENV="http://schemas.xmlsoap.org/soap/envelope/" 
        xmlns:SOAP-ENC="http://schemas.xmlsoap.org/soap/encoding/" 
        xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" 
        xmlns:xsd="http://www.w3.org/2001/XMLSchema" 
        xmlns:ns1="urn:galdemo:flighttracker" 
        xmlns:ns2="http://galdemo.flighttracker.com"> 
      <SOAP-ENV:Body SOAP-ENV:encodingStyle="http://schemas.xmlsoap.org/soap/encoding/"> 
        <ns1:getFlightInfo> 
          <param1>UAL</param1> 
          <param2>184</param2> 
        </ns1:getFlightInfo> 
      </SOAP-ENV:Body> 
    </SOAP-ENV:Envelope>
~~~
</div>

The service responds with:

    HTTP/1.1 200 OK 
    Date: Thu, 30 Aug 2011 00:34:17 GMT 
    Server: IBM_HTTP_Server/1.3.12.3 Apache/1.3.12 (Win32) 
    Content-Length: 861 
    Content-Type: text/xml; charset=utf-8 
<div class="alt">
~~~{.xml}
    <?xml version="1.0" encoding="UTF-8"?> 
    <SOAP-ENV:Envelope
        xmlns:SOAP-ENV="http://schemas.xmlsoap.org/soap/envelope/" 
        xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" 
        xmlns:xsd="http://www.w3.org/2001/XMLSchema"> 
      <SOAP-ENV:Body> 
        <ns1:getFlightInfoResponse
            xmlns:ns1="urn:galdemo:flighttracker"
            SOAP-ENV:encodingStyle="http://schemas.xmlsoap.org/soap/encoding/"> 
          <return xmlns:ns2="http://galdemo.flighttracker.com" xsi:type="ns2:FlightInfo"> 
            <equipment xsi:type="xsd:string">A320</equipment> 
            <airline xsi:type="xsd:string">UAL</airline> 
            <currentLocation xsi:type="xsd:string">188 mi W of Lincoln, NE</currentLocation> 
            <altitude xsi:type="xsd:string">37000</altitude> 
            <speed xsi:type="xsd:string">497</speed> 
            <flightNumber xsi:type="xsd:string">184</flightNumber> 
          </return> 
        </ns1:getFlightInfoResponse> 
      </SOAP-ENV:Body> 
    </SOAP-ENV:Envelope>
~~~
</div>

The stub function returns the service response in variable `flight` of type
`struct ns1__getFlightInfoResponse` and this information can be displayed by
the client application:

    A320 flight UAL184 traveling 497 mph at 37000 ft, is located 188 mi W of Lincoln, NE

Note that the response includes <i>`xsi:type`</i> attributes indicating the schema types of the elements in the XML message.  This was common practice in earlier SOAP implementations and some SOAP implementations relied on it, but it was never mandated by the specifications.  You can let soapcpp2 generate serializers that produce the <i>`xsi:type`</i> type information in XML messages with <b>`soapcpp2 -t`</b> option <b>`-t`</b>.  Otherwise, the serializers will not include <i>`xsi:type`</i> attributes in the XML message payloads unless a derived type value is used in the XML payload in place of a base type, for example a derived class in place of a base class.  In this way, SOAP/XML messaging implements object inheritance cleanly and efficiently because the leading <i>`xsi:type`</i> attribute value corresponding to the serialized derived class in an XML message lets an XML parser and deserializer instantiate the derived class to populate it immediately (something that can't be claimed of JSON since JSON has no attributes and object properties are unordered).

@note: the flight tracker service is no longer available since 9/11/2001. It is kept in the documentation as an example to illustrate the use of structs/classes and response types.

🔝 [Back to table of contents](#)

### How to specify anonymous parameter names        {#anonymous}

The SOAP RPC encoding protocol allows parameter names to be anonymous.  That
is, the name(s) of the output parameters of a service operation are not
strictly required to match a client's view of the parameters names.  Also, the
input parameter names of a service operation are not strictly required to match
a service's view of the parameter names.  The soapcpp2 tool can generate stub
and skeleton functions that support anonymous parameters.  Parameter names are
implicitly anonymous by omitting the parameter names in the function prototype
of the service operation. For example:

~~~{.cpp}
    // Contents of file "calc.h": 
    int ns2__add(double, double, double&);
~~~

This enumerates the parameter names as `_param_1`, `_param_2`, and `_param_3`, where the leading underscore makes these names anonymous, meaning that the XML parser and deserializer will accept any parameter name to extract their values, that is, even when the name of the XML element representing the parameter differs.

To make parameter names explicitly anonymous, specify parameter names that start with an
underscore (`_`) in the function prototype in the header file.

For example:

~~~{.cpp}
    // Contents of file "calc.h": 
    int ns2__add(double _a, double _b, double& _return);
~~~

In this example, the `_a`, `_b`, and `_return` are anonymous parameters.

@warning When anonymous parameter names are used, the order of the parameters
in the function prototype of a service operation is significant.

🔝 [Back to table of contents](#)

### How to specify a service operation with no input parameters  {#noinparam}

To specify a service operation that has no input parameters, just provide a
function prototype with one parameter which is the output parameter that is either a pointer or a reference, for example:

~~~{.cpp}
    int ns__getValue(double& value);
~~~

The soapcpp2 tool generates a struct for each service operation request message, which in this case is an empty struct.  To prevent C compilers from throwing an error, the empty struct is patched at compile time with the compile-time flag `#WITH_NOEMPTYSTRUCT`.

🔝 [Back to table of contents](#)

### How to specify a service operation with no output parameters  {#nooutparam}

To specify a service operation that has no output parameters, just define a function prototype with a response struct that is
empty, for example:

~~~{.cpp}
    enum ns__event { off, on, stand_by }; 
    int ns__signal(enum ns__event in, struct ns__signalResponse { } *out);
~~~

Since the response struct is empty, no output parameters are specified.  The
SOAP response message has an empty response element <i>`ns:signalResponse`</i>.

Specifying an empty response is not identical to SOAP one-way messaging, which
is asynchronous and does not expect an XML response message to be transmitted
at all, just an empty HTTP OK response to a HTTP POST request.  See Section
\ref oneway1 on one-way messaging.

### How to switch to REST from SOAP  {#restclient}

To switch to RESTful Web APIs from SOAP Web services APIs is simple, just use a directive.

To declare HTTP POST as the default HTTP method to use with client-side calls for all service operations associated with the `ns` namespace prefix:

~~~{.cpp}
    //gsoap ns service protocol: POST
~~~

To declare the HTTP POST method for a specific service operation, use:

~~~{.cpp}
    //gsoap ns service protocol: POST
    int ns__webmethod(...);
~~~

You can specify `GET`, `PUT`, `POST`, and `DELETE`.  With `GET` the input parameters of the service operations should be primitive types.  See Section \ref directives-1.

🔝 [Back to table of contents](#)

## How to build Web services APIs  {#services}

The soapcpp2 tool generates skeleton functions in C or C++ source code for each
of the service operations specified as function prototypes in the interface
header file input to soapcpp2.  The skeleton functions can be readily used to
implement the service operations in a new Web service. The compound data types
used by the input and output parameters of service operations must be declared
in the interface header file, such as structs, classes, arrays, C++ containers,
and pointer-based data structures (e.g. data structure trees and arbitrary
operation. The soapcpp2 tool automatically generates serializers and
deserializers for the data types to enable the generated skeleton functions to
encode and decode the contents of the parameters of the service operations.
The soapcpp2 tool also generates a service operation request dispatcher function
`::soap_serve` that serves requests by calling the appropriate skeleton.

🔝 [Back to table of contents](#)

### Example        {#example7}

The following example specifies three service operations of a Web service:

~~~{.cpp}
    // Contents of file "calc.h": 
    //gsoap ns service namespace: urn:simple-calc
    int ns__add(double a, double b, double& result); 
    int ns__sub(double a, double b, double& result); 
    int ns__sqrt(double a, double& result); 
~~~

The `add` and `sub` methods are intended to add and subtract two double
floating point numbers stored in input parameters `a` and `b` and should return
the result of the operation in the `result` output parameter. The `sqrt` method
is intended to take the square root of input parameter `a` and to return the
result in the output parameter `result`.

To generate the skeleton functions, the soapcpp2 tool is invoked from the
command line with:

     soapcpp2 calc.h

The soapcpp2 tool generates the skeleton functions in file
<i>`soapServer.cpp`</i> for the `add`, `sub`, and `sqrt` service operations
specified in the <i>`calc.h`</i> header file. The skeleton functions are
respectively `soap_serve_ns__add`, `soap_serve_ns__sub`, and
`soap_serve_ns__sqrt`. The generated file <i>`soapC.cpp`</i> contains
serializers and deserializers for the skeleton. The soapcpp2 tool also
generates a service dispatcher: the `::soap_serve` function handles client
requests and dispatches the service operation requests to the appropriate
skeleton functions to serve the requests. The skeleton in turn calls the service
operation implementation function. The function prototype of the service
operation implementation function is specified in the header file that is input
to the soapcpp2 tool.

Here is an example CGI service application that uses the generated `::soap_serve` skeleton function to handle client requests:

~~~{.cpp}
    // Contents of file "calc.cpp": 
    #include "soapH.h" 
    #include "ns.nsmap" 
    #include <math.h> // for sqrt() 

    int main() 
    {
      return soap_serve(soap_new());
    } 

    // Implementation of the "add" service operation: 
    int ns__add(struct soap *soap, double a, double b, double& result) 
    {
      result = a + b; 
      return SOAP_OK; 
    } 

    // Implementation of the "sub" service operation: 
    int ns__sub(struct soap *soap, double a, double b, double& result) 
    {
      result = a - b; 
      return SOAP_OK; 
    } 

    // Implementation of the "sqrt" service operation: 
    int ns__sqrt(struct soap *soap, double a, double& result) 
    {
      if (a < 0) 
        return soap_receiver_fault(soap, "Square root of negative number", "I can only take the square root of a non-negative number"); 
      result = sqrt(a); 
      return SOAP_OK; 
    } 

    struct Namespace namespaces[] = 
    {
      { "SOAP-ENV", "http://schemas.xmlsoap.org/soap/envelope/" }, 
      { "SOAP-ENC", "http://schemas.xmlsoap.org/soap/encoding/" }, 
      { "xsi",      "http://www.w3.org/2001/XMLSchema-instance" }, 
      { "xsd",      "http://www.w3.org/2001/XMLSchema" }, 
      { "ns",       "urn:simple-calc" }, // binds "ns" namespace prefix to schema URI
      { NULL, NULL } 
    };
~~~

Note that the service operations have an extra input parameter which is a pointer to the `::soap` context.

The implementation of the service operations must return `#SOAP_OK` or a nonzero error code. The code `#SOAP_OK` denotes success. A nonzero error code is returned with `::soap_receiver_fault` and `::soap_sender_fault`.
These functions also set up a SOAP Fault structure with the details of the fault returned.  This is done by setting
the `::soap::fault` which points to `::SOAP_ENV__Fault` structure.  Its `::SOAP_ENV__Fault::faultstring` string and `::SOAP_ENV__Fault::detail `details are populated with the fault string (XML text) and fault detail (XML string).
SOAP 1.2 requires the `::SOAP_ENV__Fault::SOAP_ENV__Reason` and the
`::SOAP_ENV__Fault::SOAP_ENV__Detail` strings to be assigned instead.

This service application can be readily installed as a CGI application, which is a simple stateless way to deploy services.  To deploy this service as a multi-threaded stand-alone server application see Sections \ref stand-alone and \ref mt.

Besides generating the skeleton functions and serializers in source code, the soapcpp2 tool also generates a WSDL file for this service, see Section \ref wsdl for details on WSDL.

As per SOAP protocol (when applicable), "SOAP actions" are HTTP headers that are specific to the SOAP protocol and provide a means
for routing service requests and for security reasons, for example firewall software can inspect SOAP action headers to grant or deny the
SOAP request. Use <b>`soapcpp2 -a`</b> option <b>`-a`</b> or  <b>`soapcpp2 -A`</b> option <b>`-A`</b> to let the generated skeleton functions dispatch the requests based on the SOAP action HTTP header together with (or instead) the name of the XML request element.

Note that soapcpp2 generates both clients and services based on the interface header file input.  Which means that there is no need to modify the interface header file for client side or server side deployments.
For example, the generated `soap_call_ns__add` stub function is saved to the <i>`soapClient.cpp`</i> file after invoking
the soapcpp2 tool on the <i>`calc.h`</i> header file.

🔝 [Back to table of contents](#)

### MSVC++ builds  {#msvc}

* Win32 builds require winsock2 (MS Visual C++ `ws2_32.lib`) To do this in
  Visual C++, go to **Project**, **Properties**, select **Link** and add
  `ws2_32.lib` to the **Object library modules** entry.

* Use files with extension <i>`.cpp`</i> only. This means that you may have to
  rename all <i>`.c`</i> files to <i>`.cpp`</i>.

* Turn pre-compiled headers off.

* When creating a new project, you can specify a custom build step to
  automatically invoke the soapcpp2 tool on a interface header file for
  soapcpp2. In this way you can incrementally build a new service by adding new
  operations and data types to the header file. For the latest instruction on
  how to specify a custom build step to run soapcpp2, see the gSOAP download
  and installation page <https://www.genivia.com/downloads.html>.

* You may want to use the ISAPI extension or WinInet plugin available in the
  <i>`gsoap/mod_gsoap`</i> directory of the gSOAP package to simplify Internet
  access and deal with encryption, proxies, and authentication.  See the gSOAP
  [ISAPI extension](../../isapi/html/index.html) documentation and the gSOAP
  [WinInet plugin](../../wininet/html/index.html) documentation.

🔝 [Back to table of contents](#)

### How to create a stand-alone server        {#stand-alone}

The deployment of a Web service as a CGI application is an easy means to
provide your service on the Internet. However, CGI is stateless and the performance of CGI is not
great.  Instead, gSOAP services can be run as stand-alone services on any port by
using the built-in HTTP and TCP/IP stacks of the engine.  The recommended
mechanism to deploy a service is through the gSOAP Apache module or ISAPI extension. These
servers and modules are designed for server load balancing and access control.
See the gSOAP
[Apache module](../../apache/html/index.html) documentation and the
gSOAP [ISAPI extension](../../isapi/html/index.html) documentation for details.

See also the getting started page <https://www.genivia.com/dev.html> and
tutorial page <https://www.genivia.com/tutorial.html> to get started with
developing client and stand-alone server applications using the gSOAP tools.

To create a stand-alone service, the `main` function of the service application should call `::soap_bind` to bind a port and then loop over `::soap_accept` to accept requests to serve with `::soap_serve`.  Also call `::soap_ssl_accept` when HTTPS is used, which is set up with `::soap_ssl_server_context`.

For example:

~~~{.cpp}
    #include "soapH.h"
    #include "ns.nsmap"

    int main() 
    {
      struct soap *soap = soap_new1(SOAP_XML_INDENT);
      soap->send_timeout = 10;     // 10 seconds max socket delay 
      soap->recv_timeout = 10;     // 10 seconds max socket delay 
      soap->accept_timeout = 3600; // server stops after 1 hour of inactivity 
      soap->max_keep_alive = 100;  // max keep-alive sequence 
      // soap_ssl_server_context(soap, ...); // call this function when HTTPS is used
      SOAP_SOCKET m, s;                      // master and slave sockets 
      m = soap_bind(soap, NULL, 18083, 10);  // small BACKLOG for iterative servers 
      if (!soap_valid_socket(m)) 
      {
        soap_print_fault(soap, stderr); 
      }
      else 
      {
        fprintf(stderr, "Socket connection successful: master socket = %d\n", m); 
        for (int i = 1; ; i++) 
        {
          s = soap_accept(soap); 
          if (!soap_valid_socket(s)) 
          {
            soap_print_fault(soap, stderr); 
            break; 
          } 
          // soap_ssl_accept(soap); // call when HTTPS is used
          fprintf(stderr, "%d: accepted connection from IP=%d.%d.%d.%d socket=%d", i, (soap->ip>>24)&0xFF, (soap->ip>>16)&0xFF, (soap->ip>>8)&0xFF, soap->ip&0xFF, s); 
          if (soap_serve(soap))
            soap_print_fault(soap, stderr);
          else
            fprintf(stderr, "request served\n"); 
          soap_destroy(soap); // delete managed class instances 
          soap_end(soap);     // delete managed data and temporaries 
        } 
      } 
      soap_free(soap); // finalize and delete the context 
    }
~~~

The `::soap_serve` dispatcher handles one request or multiple requests when HTTP keep-alive is enabled with the `#SOAP_IO_KEEPALIVE` flag, which should only be used with client applications or with stand-alone multi-threaded services, see the next section and Section \ref keepalive.

The gSOAP functions that are frequently used for server-side coding are:

* `soap_bind(struct soap *soap, char *host, int port, int backlog)` binds `::soap::master` socket to the specified port and host name (or NULL for the current machine), using a backlog queue size of pending requests, returns master socket. We check the return value with `#soap_valid_socket`.  The backlog queue size should be small, say 2 to 10, for iterative (not multi-threaded) stand-alone servers to ensure fairness among connecting clients.  A smaller value increases fairness and defends against denial of service, but hampers performance because connection requests may be refused.

* `soap_accept(struct soap *soap)` returns `#SOAP_SOCKET` socket `::soap::socket` when connected.  We check the return value with `#soap_valid_socket`.

* `soap_ssl_accept(struct soap *soap)` returns `#SOAP_OK` when the HTTPS handshake successfully completed.

The IPv4 address is stored in `::soap::ip`. If IPv6 is enabled with `#WITH_IPV6` then `::soap::ip6` contains the IPv6 address.

The `::soap::accept_timeout` context variable of the context specifies the timeout value for a non-blocking
`::soap_accept` call. See Section \ref timeout  for more details on timeout management.

The `::soap_serve` function parses the inbound HTTP request and dispatches the request to the skeleton functions that call the service operations implemented.

See Section \ref memory  for more details on memory management.

🔝 [Back to table of contents](#)

### How to create a multi-threaded stand-alone service        {#mt}

Stand-alone multi-threading a Web Service is essential when the response times for handling requests by the service are long or when HTTP keep-alive is enabled, see also Section \ref keepalive .

In case of long response times, the latencies introduced by the unrelated
requests may become prohibitive for a successful deployment of a stand-alone
service. When HTTP keep-alive is enabled, a client and server remain connected until 100 (`#SOAP_MAXKEEPALIVE`) request-response iterations later as specified by `::soap::max_keep_alive` or when a timeout occurred.  Thereby preventing other clients from connecting.

The recommended
mechanism to deploy a service is through the gSOAP Apache module or ISAPI extension. These
servers and modules are designed for server load balancing and access control.
See the gSOAP
[Apache module](../../apache/html/index.html) documentation and the
gSOAP [ISAPI extension](../../isapi/html/index.html) documentation for details.

See also the getting started page <https://www.genivia.com/dev.html> and
tutorial page <https://www.genivia.com/tutorial.html> to get started with
developing client and stand-alone server applications using the gSOAP tools.

The following example illustrates the use of threads to improve the quality of service by handling new requests in separate threads:

~~~{.cpp}
    #include "soapH.h" 
    #include "ns.nsmap"
    #include "plugin/threads.h"
    #define BACKLOG (100) // Max request backlog of pending requests

    int main(int argc, char **argv) 
    {
      struct soap soap; 
      soap_init(&soap); 
      if (argc < 2) // no args: assume this is a CGI application 
      {
        soap_serve(&soap);   // serve request, one thread, CGI style 
        soap_destroy(&soap); // delete managed class instances 
        soap_end(&soap);     // delete managed data and temporaries 
      } 
      else 
      {
        void *process_request(void*); 
        struct soap *tsoap; 
        THREAD_TYPE tid; 
        int port = atoi(argv[1]); // first command-line arg is port 
        SOAP_SOCKET m, s; 
        soap.send_timeout = 10; // 10 seconds max socket delay 
        soap.recv_timeout = 10; // 10 seconds max socket delay 
        soap.accept_timeout = 3600; // server stops after 1 hour of inactivity 
        soap.max_keep_alive = 100; // max keep-alive sequence 
        m = soap_bind(&soap, NULL, port, BACKLOG); 
        if (!soap_valid_socket(m)) 
          exit(EXIT_FAILURE); 
        fprintf(stderr, "Socket connection successful %d\n", m); 
        while (1)
        {
          s = soap_accept(&soap); 
          if (soap_valid_socket(s)) 
          {
            fprintf(stderr, "Accept socket %d connection from IP %d.%d.%d.%d\n", s, (soap.ip>>24)&0xFF, (soap.ip>>16)&0xFF, (soap.ip>>8)&0xFF, soap.ip&0xFF); 
            tsoap = soap_copy(&soap); // make a copy 
            if (!tsoap) 
              soap_force_closesock(&soap);
            else
              while (THREAD_CREATE(&tid, (void*(*)(void*))process_request, (void*)tsoap))
                sleep(1); // failed, try again
          }
          else if (soap.errnum) // accept failed, try again after 1 second
          {
            soap_print_fault(&soap, stderr); 
            sleep(1);
          } 
          else
          {
            fprintf(stderr, "server timed out\n"); 
            break; 
          } 
        } 
      } 
      soap_done(&soap); // finalize context
      return 0; 
    } 

    void *process_request(void* tsoap) 
    {
      struct soap *soap = (struct soap*)tsoap;
      THREAD_DETACH(THREAD_ID); 
      soap_serve(soap); 
      soap_destroy(soap); // delete managed class instances 
      soap_end(soap);     // delete managed data and temporaries 
      soap_free(soap);    // finalize and delete the context
      return NULL; 
    }

    // ... the service operations are defined here ...
~~~

For this multi-threaded application the <i>`gsoap/plugin/threads.h`</i> and
<i>`gsoap/plugin/threads.c`</i> portable threads and mutex API is used.

The server spawns a thread per request.  Each thread executes a `::soap_serve` using a copy of the `::soap` context created with `::soap_copy` (if `::soap_copy` fails due to out of memory then we can still recover as shown, recovery from errors is an important aspect of gSOAP's design and API implementation).  Note that the server does not wait for threads to join the main thread upon program termination.

The `::soap_serve` dispatcher handles one request or multiple requests when
HTTP keep-alive is set with `#SOAP_IO_KEEPALIVE`. The
`::soap::max_keep_alive` value can be set to the maximum keep-alive calls
allowed, which is important to avoid a client from holding a thread
indefinitely. The send and receive timeouts are set to avoid (intentionally)
slow clients from holding a socket connection too long. The accept timeout is used
to let the server terminate automatically after a period of inactivity.

The following example limits the number of concurrent threads to reduce the machine's CPU resource utilization:

~~~{.cpp}
    #include "soapH.h" 
    #include "ns.nsmap"
    #include "plugin/threads.h" 
    #define BACKLOG (100) // Max request backlog of pending requests
    #define MAX_THR (10)  // Max threads to serve requests 

    int main(int argc, char **argv) 
    {
      struct soap soap; 
      soap_init(&soap); 
      if (argc < 2) // no args: assume this is a CGI application 
      {
        soap_serve(&soap);   // serve request, one thread, CGI style 
        soap_destroy(&soap); // delete managed class instances 
        soap_end(&soap);     // delete managed data and temporaries 
      } 
      else 
      {
        struct soap *soap_thr[MAX_THR]; // each thread needs a context 
        THREAD_TYPE tid[MAX_THR];       // array of thread IDs
        int port = atoi(argv[1]);       // first command-line arg is port 
        SOAP_SOCKET m, s; 
        int i; 
        soap.send_timeout = 10;     // 10 seconds max socket delay 
        soap.recv_timeout = 10;     // 10 seconds max socket delay 
        soap.accept_timeout = 3600; // server stops after 1 hour of inactivity 
        soap.max_keep_alive = 100;  // max keep-alive sequence 
        m = soap_bind(&soap, NULL, port, BACKLOG); 
        if (!soap_valid_socket(m)) 
          exit(EXIT_FAILURE);
        fprintf(stderr, "Socket connection successful %d\n", m); 
        for (i = 0; i < MAX_THR; i++) 
          soap_thr[i] = NULL; 
        while (1)
        {
          for (i = 0; i < MAX_THR; i++) 
          {
            s = soap_accept(&soap); 
            if (soap_valid_socket(s)) 
            {
              fprintf(stderr, "Thread %d accepts socket %d connection from IP %d.%d.%d.%d\n", i, s, (soap.ip>>24)&0xFF, (soap.ip>>16)&0xFF, (soap.ip>>8)&0xFF, soap.ip&0xFF); 
              if (!soap_thr[i]) // first time around 
              {
                soap_thr[i] = soap_copy(&soap); 
                if (!soap_thr[i]) 
                  exit(EXIT_FAILURE); // could not allocate 
              } 
              else // recycle threaded soap contexts
              {
                // optionally, we can cancel the current thread when stuck on IO:
                // soap_close_connection(soap_thr[i]); // requires compiling 2.8.71 or greater with -DWITH_SELF_PIPE
                THREAD_JOIN(tid[i]); 
                fprintf(stderr, "Thread %d completed\n", i); 
                soap_destroy(soap_thr[i]);            // delete managed class instances of thread 
                soap_end(soap_thr[i]);                // delete managed data and temporaries of thread 
                soap_copy_stream(soap_thr[i], &soap); // pass the connection on to the thread
              } 
              while (THREAD_CREATE(&tid[i], (void*(*)(void*))soap_serve, (void*)soap_thr[i]))
                sleep(1); // failed, try again
            }
            else if (soap.errnum) // accept failed, try again after 1 second
            {
              soap_print_fault(&soap, stderr); 
              sleep(1);
            } 
            else
            {
              fprintf(stderr, "Server timed out\n"); 
              goto end;
            } 
          } 
        } 
    end:
        for (i = 0; i < MAX_THR; i++) 
        {
          if (soap_thr[i]) 
          {
            THREAD_JOIN(tid[i]); 
            soap_destroy(soap_thr[i]);
            soap_end(soap_thr[i]);
            soap_free(soap_thr[i]);
          }
        }
      } 
      soap_destroy(&soap);
      soap_end(&soap);
      soap_done(&soap);
      return 0; 
    }

    // ... the service operations are defined here ...
~~~

The advantage of the code shown above is that the machine cannot be overloaded with requests, since the number of active services is limited. However, threads are still started and terminated. This overhead can be eliminated using a queue of requests (a queue of accepted socket connections):

~~~{.cpp}
    #include "soapH.h" 
    #include "ns.nsmap"
    #include "plugin/threads.h"

    #define BACKLOG (100)    // Max. request backlog of pending requests
    #define MAX_THR (64)     // Size of thread pool 
    #define MAX_QUEUE (1000) // Max. size of request queue 

    void *process_queue(void*); 
    int enqueue(SOAP_SOCKET); 
    SOAP_SOCKET dequeue(); 

    static SOAP_SOCKET queue[MAX_QUEUE]; // The global request queue of sockets 
    static int head = 0, tail = 0;
    static MUTEX_TYPE queue_lock;    // mutex for queue ops critical sections
    static COND_TYPE queue_notempty; // condition variable when queue is empty
    static COND_TYPE queue_notfull;  // condition variable when queue is full

    int main(int argc, char **argv) 
    {
      struct soap soap; 
      soap_init(&soap); 
      if (argc < 2) // no args: assume this is a CGI application 
      {
        soap_serve(&soap);   // serve request, one thread, CGI style 
        soap_destroy(&soap); // delete managed class instances 
        soap_end(&soap);     // delete managed data and temporaries 
      } 
      else 
      {
        struct soap *soap_thr[MAX_THR]; // each thread needs a context 
        THREAD_TYPE tid[MAX_THR]; 
        int port = atoi(argv[1]); // first command-line arg is port 
        SOAP_SOCKET m, s; 
        int i; 
        m = soap_bind(&soap, NULL, port, BACKLOG); 
        if (!soap_valid_socket(m)) 
          exit(EXIT_FAILURE); 
        fprintf(stderr, "Socket connection successful %d\n", m); 
        MUTEX_SETUP(queue_lock); 
        COND_SETUP(queue_notempty); 
        COND_SETUP(queue_notfull); 
        for (i = 0; i < MAX_THR; i++) 
        {
          soap_thr[i] = soap_copy(&soap); 
          fprintf(stderr, "Starting thread %d\n", i); 
          while (THREAD_CREATE(&tid[i], (void*(*)(void*))process_queue, (void*)soap_thr[i]))
            sleep(1); // failed, try again
        } 
        while (1)
        {
          s = soap_accept(&soap); 
          if (soap_valid_socket(s)) 
          {
            fprintf(stderr, "Accept socket %d connection from IP %d.%d.%d.%d\n", s, (soap.ip>>24)&0xFF, (soap.ip>>16)&0xFF, (soap.ip>>8)&0xFF, soap.ip&0xFF); 
            enqueue(s);
          }
          else if (soap.errnum) // accept failed, try again after 1 second
          {
            soap_print_fault(&soap, stderr); 
            sleep(1);
          } 
          else
          {
            fprintf(stderr, "Server timed out\n"); 
            break; 
          } 
        } 
        for (i = 0; i < MAX_THR; i++) 
          enqueue(SOAP_INVALID_SOCKET);
        for (i = 0; i < MAX_THR; i++) 
        {
          fprintf(stderr, "Waiting for thread %d to terminate... ", i); 
          THREAD_JOIN(tid[i]); 
          fprintf(stderr, "terminated\n"); 
          soap_free(soap_thr[i]); 
        } 
        COND_CLEANUP(queue_notfull); 
        COND_CLEANUP(queue_notempty); 
        MUTEX_CLEANUP(queue_lock); 
      } 
      soap_destroy(&soap);
      soap_end(&soap);
      soap_done(&soap); 
      return 0; 
    } 

    void *process_queue(void *tsoap) 
    {
      struct soap *soap = (struct soap*)tsoap; 
      while (1)
      {
        soap->socket = dequeue(); 
        if (!soap_valid_socket(soap->socket)) 
          break; 
        soap_serve(soap); 
        soap_destroy(soap); 
        soap_end(soap); 
        fprintf(stderr, "served\n"); 
      } 
      soap_free(soap);
      return NULL; 
    } 

    /* add job (socket with pending request) to queue */
    void enqueue(SOAP_SOCKET s)
    {
      int next;
      MUTEX_LOCK(queue_lock);
      next = (tail + 1) % MAX_QUEUE;
      if (next == head)
        COND_WAIT(queue_notfull, queue_lock);
      queue[tail] = s;
      tail = next;
      COND_SIGNAL(queue_notempty);
      MUTEX_UNLOCK(queue_lock);
    }

    /* remove job (socket with request) from queue */
    SOAP_SOCKET dequeue()
    {
      SOAP_SOCKET s;
      MUTEX_LOCK(queue_lock);
      if (head == tail)
        COND_WAIT(queue_notempty, queue_lock);
      s = queue[head];
      head = (head + 1) % MAX_QUEUE;
      COND_SIGNAL(queue_notfull);
      MUTEX_UNLOCK(queue_lock);
      return s;
    }

    // ... the service operations are defined here ...
~~~

For this multi-threaded application the <i>`gsoap/plugin/threads.h`</i> and
<i>`gsoap/plugin/threads.c`</i> portable threads and mutex API is used.

🔝 [Back to table of contents](#)

### How to pass application state info to service operations  {#user}

The `void* ::soap::user` variable can be used to pass application state information to
service operations and to plugins. This variable can be set before the `::soap_serve` call.
The service method can access this variable to use the application-dependent data.
The following example shows how a non-static database handle is initialized and
passed to the service methods:

~~~{.cpp}
    int main()
    {
      struct soap soap; 
      database_handle_type database_handle; 
      soap_init(&soap);
      soap.user = (void*)database_handle; 
      ... //
      if (soap_serve(&soap))
        ... // error
      ... //
    } 

    int ns__webmethod(struct soap *soap, ...) 
    {
      fetch((database_handle_type*)soap->user); // use database handle
      ... //
      return SOAP_OK; 
    }
~~~

Another way to maintain and pass state information with the context is done with plugins, see Section \ref plugins .

🔝 [Back to table of contents](#)

### How to generate C++ server classes        {#object}

Server object classes for C++ server applications are automatically generated
by the soapcpp2 tool using <b>`soapcpp2 -j`</b> option <b>`-j`</b> or <b>`soapcpp2 -i`</b> option <b>`-i`</b>.
Without these options the soapcpp2 tool generates 
C-based stub and skeleton functions.

We illustrate the use of server classes with the following example interface header file:

~~~{.cpp}
    // Content of file "calc.h": 
    //gsoap ns service name: Calculator 
    //gsoap ns service protocol: SOAP1.2
    //gsoap ns service style: document
    //gsoap ns service encoding: literal 
    //gsoap ns service location: http://www.cs.fsu.edu/~engelen/calc.cgi 
    //gsoap ns schema namespace: urn:calc 
    int ns__add(double a, double b, double& result); 
    int ns__sub(double a, double b, double& result); 
    int ns__mul(double a, double b, double& result); 
    int ns__div(double a, double b, double& result);
~~~

The directives provide the service name which is used to name the service class, the protocol (SOAP 1.2) and style (document/literal), service location (endpoint URL), and
the schema namespace URI. 

We run <b>`soapcpp2 -i calc.h`</b> with option <b>`-i`</b> to generate <i>`soapCalculatorService.h`</i> which declares the C++ sever class that has the following structure:

~~~{.cpp}
    #include "soapH.h" 
    class CalculatorService : public soap 
    {
     public: 
      Calculator() { soap_init(this); }; 
      ... // more constructors, elided here for clarity
      ~Calculator() { soap_done(this); }; 
      SOAP_SOCKET bind(const char *host, int port, int backlog) { return soap_bind(this, host, port, backlog); }
      SOAP_SOCKET accept() { return soap_accept(this); }
      int serve() { return soap_serve(this); }; 
      void destroy() { soap_destroy(this); soap_end(this); }
      ... // more methods, elided here for clarity
      // user-defined service operations:
      int add(double a, double b, double& result); 
      int sub(double a, double b, double& result); 
      int mul(double a, double b, double& result); 
      int div(double a, double b, double& result);
    };
~~~

This generated server class `serve` method calls the `add`, `sub`, `mul`, and `div` methods upon receiving an XML request message.  These methods should be implemented, for example as follows in a CGI-based service:

~~~{.cpp}
    #include "soapCalculatorService.h"
    #include "Calculator.nsmap"

    int main() 
    {
      CalculatorService calc; 
      calc.serve();
      calc.destroy();
    } 

    int calc::add(double a, double b, double& result) 
    {
      result = a + b; 
      return SOAP_OK; 
    } 

    int calc::sub(double a, double b, double& result) 
    {
      result = a - b; 
      return SOAP_OK; 
    } 

    int calc::mul(double a, double b, double& result) 
    {
      result = a * b; 
      return SOAP_OK; 
    } 

    int calc::div(double a, double b, double& result) 
    {
      if (b == 0.0)
        return soap_sender_fault(this, "Division by zero", NULL);
      result = a / b; 
      return SOAP_OK; 
    } 
~~~

If we run <b>`soapcpp2 -j calc.h`</b> with option <b>`-j`</b> to generate <i>`soapCalculatorService.h`</i> then we get the same class as with option <b>`-i`</b> but the `::soap` context is not a base class but is a member of the service class:

~~~{.cpp}
    #include "soapH.h" 
    class CalculatorService
    {
     public: 
      struct soap *soap;
      Calculator() { soap = soap_new(); }; 
      ... // more constructors, elided here for clarity
      ~Calculator() { soap_free(soap); }; 
      SOAP_SOCKET bind(const char *host, int port, int backlog) { return soap_bind(soap, host, port, backlog); }
      SOAP_SOCKET accept() { return soap_accept(soap); }
      int serve() { return soap_serve(soap); }; 
      void destroy() { soap_destroy(soap); soap_end(soap); }
      ... // more methods, elided here for clarity
      // user-defined service operations:
      int add(double a, double b, double& result);
      int sub(double a, double b, double& result); 
      int mul(double a, double b, double& result); 
      int div(double a, double b, double& result);
    };
~~~

The only difference we make to implement the service application is to use the `::soap` member of the class instead of `this` when referring to the context, which in our example changes only one line of code in the `div` method:

~~~{.cpp}
      if (b == 0.0)
        return soap_sender_fault(soap, "Division by zero", NULL);
    } 
~~~

In fact, the service classes have `soap_sender_fault` and `soap_receiver_fault` methods that can be used instead.

You can declare a C++ namespace `name` with <b>`soapcpp2 -q name`</b> to create a
server class in the `name` namespace, see Section \ref codenamespace .
For more options, see also Sections \ref soapcpp2options 
and \ref dylibs.

The example above serves CGI requests. The generated service classes also have `bind` and `accept` methods, which can be used to implement stand-alone services, see also Section \ref stand-alone .

A better alternative is to use <b>`soapcpp2 -j`</b> option <b>`-j`</b> or
option <b>`-i`</b>. With option <b>`-j`</b> the C++ proxy and service classes
have a `soap` context pointer.  This context pointer can be set and shared
among many proxy and service classes.  With option <b>`-i`</b> the C++ proxy
and server classes are derived from the `::soap` context, which simplifies the
proxy invocation and service operation implementations.

Compilation of the above header file with <b>`soapcpp2 -i`</b> creates new
files <i>`soapCalculatorService.h`</i> and <i>`soapCalculatorService.cpp`</i>
(rather than the C-style <i>`soapServer.cpp`</i>).

This generated server object class can be included into a server application
together with the generated namespace table as shown in this example:

~~~{.cpp}
    #include "soapCalculatorService.h" // get server object 
    #include "Calculator.nsmap"        // include the generated namespace table

    int main() 
    {
      soapCalculatorService c; 
      return c.serve(); // calls soap_serve to serve as CGI application (using stdin/out) 
    } 

    // The 'add' service method (soapcpp2 w/ option -i) 
    int soapCalculatorService::add(double a, double b, double &result) 
    {
      result = a + b; 
      return SOAP_OK; 
    } 
    ... // sub(), mul(), and div() implementations
~~~

Note that the service operation does not need a prefix (`ns__`) and there
is no `::soap` context passed to the service operation since the service
object itself is the context (it is derived from the `::soap` context struct).

🔝 [Back to table of contents](#)

### How to chain C++ server classes to accept messages on the same port        {#chaining}

When combining multiple services into one application, you can run wsdl2h
on multiple WSDLs to generate the single all-inclusive service definitions
interface header file for soapcpp2. This header file is then processed with soapcpp2 to
generate skeleton functions in C or server classes in C++ when using <b>`soapcpp2 -j`</b> option <b>`-j`</b> (or option <b>`-i`</b>).

This approach works well for C and C++ too, but the problem in C++ is that we end up with multiple service classes, each
for a collection of service operations that the class is supposed to implement. But
what if we need to provide one endpoint port for all services and operations?
In this case invoking the server object's `serve` method is not sufficient, since only one
service can accept requests while we want multiple services to listen to the
same port.

For example, say we have three service classes `soapABCService`, `soapUVWService`, and `soapXYZService`.  We run <b>`soapcpp2 -i -S -q name`</b> three times (on the same interface file when applicable):

    soapcpp2 -i -S -qAbc file.h
    soapcpp2 -i -S -qUvw file.h
    soapcpp2 -i -S -qXyz file.h

To generate the common <i>`envH.h`</i> file for SOAP Header and SOAP Fault definitions is done on a <i>`env.h`</i> file that is empty or has the SOAP Header and SOAP Fault detail structures `::SOAP_ENV__Header` and `::SOAP_ENV__Detail` specified:

    soapcpp2 -CSL -penv env.h

The approach is to chain the service dispatchers, as shown below:

~~~{.cpp}
    #include "AbcABCService.h" 
    #include "UvwUVWService.h" 
    #include "XyzXYZService.h" 
    #include "envH.h" // include this file last, if this file is needed 

    int main()
    {
      Abc::soapABCService abc; // generated with soapcpp2 -i -S -qAbc 
      Uvw::soapUVWService uvw; // generated with soapcpp2 -i -S -qUvw 
      Xyz::soapXYZService xyz; // generated with soapcpp2 -i -S -qXyz 
      if (!soap_valid_socket(abc.bind(NULL, 8080, 100)))
        exit(EXIT_FAILURE);
      while (1)
      {
        if (!soap_valid_socket(abc.accept()))
          exit(EXIT_FAILURE);
        // abc.ssl_accept(); // when HTTPS is used
        ... //
        if (soap_begin_serve(&abc)) // available in 2.8.2 and later 
        {
          abc.soap_stream_fault(std::cerr); 
        }
        else if (abc.dispatch() == SOAP_NO_METHOD) 
        {
          soap_copy_stream(&uvw, &abc); 
          soap_free_stream(&abc); // abc no longer uses this stream 
          if (uvw.dispatch() == SOAP_NO_METHOD) 
          {
            soap_copy_stream(&xyz, &uvw); 
            soap_free_stream(&uvw); // uvw no longer uses this stream 
            if (xyz.dispatch()) 
            {
              soap_send_fault(&xyz); // send fault to client 
              xyz.soap_stream_fault(std::cerr); 
            } 
            xyz.destroy(); 
          } 
          else 
          {
            soap_send_fault(&uvw); // send fault to client 
            uvw.soap_stream_fault(std::cerr); 
          } 
          uvw.destroy(); 
        } 
        else 
        {
          abc.soap_stream_fault(std::cerr); 
        }
        abc.destroy(); 
      }
    }
~~~

The `dispatch` method parses the SOAP/XML request and invokes the service
operations, unless there is no matching operation and `#SOAP_NO_METHOD` is
returned. The `::soap_copy_stream` ensures that the service object uses the
currently open socket. The copied streams are freed with
`::soap_free_stream`. Do not enable keep-alive support, as the socket may
stay open indefinitely afterwards as a consequence. Also, the `dispatch`
method does not send a fault to the client, which has to be explicitly done
with the `::soap_send_fault` operation when an error occurs.

In this way, multiple services can be chained to accept messages on the same port. This approach also works with SSL for HTTPS services.

However, this approach is not recommended for certain plugins, because plugins
must be registered with all service objects and some plugins require state
information to be used across the service objects, which will add significantly
to the complexity.

Therefore, it is best to have all services share the same
context. This means that <b>`soapcpp2 -j`</b> with option <b>`-j`</b> should be used instead of option <b>`-i`</b>.
As a result, we can make each service class instance to share the same `::soap` context and the same plugins.

    soapcpp2 -j -S -qAbc file.h
    soapcpp2 -j -S -qUvw file.h
    soapcpp2 -j -S -qXyz file.h

Chaining the services is also simpler to implement since we use one `::soap` context:

~~~{.cpp}
    #include "AbcABCService.h" 
    #include "UvwUVWService.h" 
    #include "XyzXYZService.h" 
    #include "envH.h" // include this file last, if it is needed 

    int main()
    {
      struct soap *soap = soap_new(); 
      Abc::soapABCService abc(soap); // generated with soapcpp2 -j -S -qAbc 
      Uvw::soapUVWService uvw(soap); // generated with soapcpp2 -j -S -qUvw 
      Xyz::soapXYZService xyz(soap); // generated with soapcpp2 -j -S -qXyz 
      if (!soap_valid_socket(soap_bind(soap, NULL, 8080, BACKLOG)))
        exit(EXIT_FAILURE);
      while (1)
      {
        if (!soap_valid_socket(soap_accept(soap)))
          exit(EXIT_FAILURE);
        if (soap_begin_serve(soap)) 
        {
          soap_stream_fault(soap, std::cerr);
        }
        else if (abc.dispatch() == SOAP_NO_METHOD) 
        {
          if (uvw.dispatch() == SOAP_NO_METHOD) 
          {
            if (xyz.dispatch() == SOAP_NO_METHOD) 
              soap_send_fault(soap); // send fault to client 
          } 
        } 
        soap_destroy(soap); 
        soap_end(soap); 
      }
      soap_free(soap); // safe to delete when abc, uvw, xyz are also deleted
    }
~~~

However, the while loop iterates for each new connection that is established with `::soap_accept` and does not allow for HTTP keep-alive connections to persist.  For our final improvement we want to support HTTP keep-alive connections that require looping over the service dispatches until the connection closes on either end, after which we resume the outer loop.  The resulting code is very close to the soapcpp2-generated `::soap_serve` code and the `serve` service class methods, with the addition of the chain of service dispatches in the loop body:

~~~{.cpp}
    #include "AbcABCService.h" 
    #include "UvwUVWService.h" 
    #include "XyzXYZService.h" 
    #include "envH.h" // include this file last, if it is needed 

    int main()
    {
      struct soap *soap = soap_new(); 
      Abc::soapABCService abc(soap); // generated with soapcpp2 -j -S -qAbc 
      Uvw::soapUVWService uvw(soap); // generated with soapcpp2 -j -S -qUvw 
      Xyz::soapXYZService xyz(soap); // generated with soapcpp2 -j -S -qXyz 
      if (!soap_valid_socket(soap_bind(soap, NULL, 8080, BACKLOG)))
        exit(EXIT_FAILURE);
      while (1)
      {
        if (!soap_valid_socket(soap_accept(soap)))
          exit(EXIT_FAILURE);
        soap->keep_alive = soap->max_keep_alive + 1; // max keep-alive iterations
        do
        {
          if ((soap->keep_alive > 0) && (soap->max_keep_alive > 0))
            soap->keep_alive--;
          if (soap_begin_serve(soap))
          {
            if (soap->error >= SOAP_STOP) // if a plugin has served the request
              continue;                   // then continue with the next request
            break;                        // an error occurred
          }
          if (abc.dispatch() == SOAP_NO_METHOD) 
          {
            if (uvw.dispatch() == SOAP_NO_METHOD) 
            {
              if (xyz.dispatch() == SOAP_NO_METHOD) 
                soap_send_fault(soap); // send fault to client 
            } 
          } 
          soap_destroy(soap); 
          soap_end(soap); 
        } while (soap->keep_alive);
        soap_destroy(soap); 
        soap_end(soap); 
      }
      soap_free(soap); // safe to delete when abc, uvw, xyz are also deleted
    }
~~~

🔝 [Back to table of contents](#)

### How to generate WSDL service descriptions        {#wsdl}

The soapcpp2 tool generates WSDL (Web Service Description Language) service
descriptions and XML schema files (XSDs) when processing an interface header
file that wasn't generated with wsdl2h.  The soapcpp2 tool produces one WSDL
file for a set of service operations in the header file.  If the header file
has no service operations (i.e. no function prototypes) then no WSDL will be
generated.  The names of the function prototypes of the service operations must
use the same namespace prefix and the namespace prefix is used to name the WSDL
file.  The WSDL file and services can be named with a
`//gsoap <prefix> service name:` directive to specify a service name for each
namespace prefix.

If multiple namespace prefixes are used
to define service operations, then multiple WSDL files will be created and each file
describes the set of service operations belonging to that namespace prefix.

The soapcpp2 tool also generates XML schema files (XSD files) for all
serializable C/C++ types declared in the interface header file input to
soapcpp2.  These XSD files do not have to be published as the WSDL file already
contains the appropriate XML Schema definitions.

To customize the WSDL output, use `//gsoap` directives to declare the service
name, the endpoint port, and namespace etc:

~~~{.cpp}
    //gsoap ns service name: example 
    //gsoap ns service type: examplePortType
    //gsoap ns service port: http://www.example.com/example 
    //gsoap ns service namespace: urn:example
~~~

These are some examples and defaults will be used when directives are not specified.  Recommended is to specify at least the service name and namespace URI. More details and settings for the service can be declared as well. See Section \ref directives  for more details.

In addition to the generation of the WSDL files, a file with a namespace mapping table is generated by the gSOAP
soapcpp2 tool. An example mapping table is shown below:

~~~{.cpp}
    struct Namespace namespaces[] = 
    {
      { "SOAP-ENV", "http://schemas.xmlsoap.org/soap/envelope/" }, 
      { "SOAP-ENC", "http://schemas.xmlsoap.org/soap/encoding/" }, 
      { "xsi",      "http://www.w3.org/2001/XMLSchema-instance", "http://www.w3.org/*/XMLSchema-instance" }, 
      { "xsd",      "http://www.w3.org/2001/XMLSchema",          "http://www.w3.org/*/XMLSchema" }, 
      { "ns",       "urn:example" }, // binds "ns" namespace prefix to schema URI
      { NULL, NULL } 
    };
~~~

This file should be included in the client or service application, see Section
\ref nstable  for details on namespace mapping tables.

🔝 [Back to table of contents](#)

### Example        {#example8}

For example, suppose the following service operations are defined in the <i>`calc.h`</i> header file:

~~~{.cpp}
    // Content of file "calc.h": 
    //gsoap ns service name: calc
    int ns__add(double a, double b, double& result); 
    int ns__sub(double a, double b, double& result); 
    int ns__sqrt(double a, double& result); 
~~~

One WSDL file <i>`calc.wsdl`</i> will be generated that describes the three service operations:

<div class="alt">
~~~{.xml}
    <?xml version="1.0" encoding="UTF-8"?>
    <definitions name="Service"
      targetNamespace="http://tempuri.org/ns.xsd/Service.wsdl"
      xmlns:tns="http://tempuri.org/ns.xsd/Service.wsdl"
      xmlns:SOAP-ENV="http://schemas.xmlsoap.org/soap/envelope/"
      xmlns:SOAP-ENC="http://schemas.xmlsoap.org/soap/encoding/"
      xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
      xmlns:xsd="http://www.w3.org/2001/XMLSchema"
      xmlns:ns="http://tempuri.org/ns.xsd"
      xmlns:SOAP="http://schemas.xmlsoap.org/wsdl/soap/"
      xmlns:HTTP="http://schemas.xmlsoap.org/wsdl/http/"
      xmlns:MIME="http://schemas.xmlsoap.org/wsdl/mime/"
      xmlns:DIME="http://schemas.xmlsoap.org/ws/2002/04/dime/wsdl/"
      xmlns:WSDL="http://schemas.xmlsoap.org/wsdl/"
      xmlns="http://schemas.xmlsoap.org/wsdl/">

    <types>

      <schema targetNamespace="http://tempuri.org/ns.xsd"
          xmlns:SOAP-ENV="http://schemas.xmlsoap.org/soap/envelope/"
          xmlns:SOAP-ENC="http://schemas.xmlsoap.org/soap/encoding/"
          xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
          xmlns:xsd="http://www.w3.org/2001/XMLSchema"
          xmlns:ns="http://tempuri.org/ns.xsd"
          xmlns="http://www.w3.org/2001/XMLSchema"
          elementFormDefault="unqualified"
          attributeFormDefault="unqualified">
        <import namespace="http://schemas.xmlsoap.org/soap/encoding/"/>
        <!-- operation request element -->
        <element name="add">
          <complexType>
              <sequence>
                <element name="a" type="xsd:double" minOccurs="1" maxOccurs="1"/><!-- ns__add::a -->
                <element name="b" type="xsd:double" minOccurs="1" maxOccurs="1"/><!-- ns__add::b -->
              </sequence>
          </complexType>
        </element>
        <!-- operation response element -->
        <element name="addResponse">
          <complexType>
              <sequence>
                <element name="result" type="xsd:double" minOccurs="1" maxOccurs="1"/><!-- ns__add::result -->
              </sequence>
          </complexType>
        </element>
        <!-- operation request element -->
        <element name="sub">
          <complexType>
              <sequence>
                <element name="a" type="xsd:double" minOccurs="1" maxOccurs="1"/><!-- ns__sub::a -->
                <element name="b" type="xsd:double" minOccurs="1" maxOccurs="1"/><!-- ns__sub::b -->
              </sequence>
          </complexType>
        </element>
        <!-- operation response element -->
        <element name="subResponse">
          <complexType>
              <sequence>
                <element name="result" type="xsd:double" minOccurs="1" maxOccurs="1"/><!-- ns__sub::result -->
              </sequence>
          </complexType>
        </element>
        <!-- operation request element -->
        <element name="sqrt">
          <complexType>
              <sequence>
                <element name="a" type="xsd:double" minOccurs="1" maxOccurs="1"/><!-- ns__sqrt::a -->
              </sequence>
          </complexType>
        </element>
        <!-- operation response element -->
        <element name="sqrtResponse">
          <complexType>
              <sequence>
                <element name="result" type="xsd:double" minOccurs="1" maxOccurs="1"/><!-- ns__sqrt::result -->
              </sequence>
          </complexType>
        </element>
      </schema>

    </types>

    <message name="addRequest">
      <part name="Body" element="ns:add"/><!-- ns__add::ns__add -->
    </message>

    <message name="addResponse">
      <part name="Body" element="ns:addResponse"/>
    </message>

    <message name="subRequest">
      <part name="Body" element="ns:sub"/><!-- ns__sub::ns__sub -->
    </message>

    <message name="subResponse">
      <part name="Body" element="ns:subResponse"/>
    </message>

    <message name="sqrtRequest">
      <part name="Body" element="ns:sqrt"/><!-- ns__sqrt::ns__sqrt -->
    </message>

    <message name="sqrtResponse">
      <part name="Body" element="ns:sqrtResponse"/>
    </message>

    <portType name="ServicePortType">
      <operation name="add">
        <documentation>Service definition of function ns__add</documentation>
        <input message="tns:addRequest"/>
        <output message="tns:addResponse"/>
      </operation>
      <operation name="sub">
        <documentation>Service definition of function ns__sub</documentation>
        <input message="tns:subRequest"/>
        <output message="tns:subResponse"/>
      </operation>
      <operation name="sqrt">
        <documentation>Service definition of function ns__sqrt</documentation>
        <input message="tns:sqrtRequest"/>
        <output message="tns:sqrtResponse"/>
      </operation>
    </portType>

    <binding name="Service" type="tns:ServicePortType">
      <SOAP:binding style="document" transport="http://schemas.xmlsoap.org/soap/http"/>
      <operation name="add">
        <SOAP:operation soapAction=""/>
        <input>
              <SOAP:body use="literal" parts="Body"/>
        </input>
        <output>
              <SOAP:body use="literal" parts="Body"/>
        </output>
      </operation>
      <operation name="sub">
        <SOAP:operation soapAction=""/>
        <input>
              <SOAP:body use="literal" parts="Body"/>
        </input>
        <output>
              <SOAP:body use="literal" parts="Body"/>
        </output>
      </operation>
      <operation name="sqrt">
        <SOAP:operation soapAction=""/>
        <input>
              <SOAP:body use="literal" parts="Body"/>
        </input>
        <output>
              <SOAP:body use="literal" parts="Body"/>
        </output>
      </operation>
    </binding>

    <service name="Service">
      <documentation>gSOAP 2.8.70 generated service definition</documentation>
      <port name="Service" binding="tns:Service">
        <SOAP:address location="http://localhost:80"/>
      </port>
    </service>

    </definitions>
~~~
</div>

The above uses the default settings for the service name, port, and namespace which can be set in the header file with `//gsoap` directives, see Section \ref directives .

🔝 [Back to table of contents](#)

### How to make client-side calls within a service operation  {#clientinservice}

Invoking a server-side client call requires the use of a new `::soap` context in the service operation itself, which is best illustrated with an example.  The following example
combines the functionality of two Web services
into one new SOAP Web service.  The service provides a currency-converted stock
quote.  To serve a request, the service in turn requests the stock quote and
the currency-exchange rate from two services.  The currency-converted quote is then calculated and returned.

In addition to being a client of two Web services, this service
application can also be used as a client of itself to test the implementation.
As a client invoked from the command-line, it will return a currency-converted
stock quote by connecting to a copy of itself installed as a CGI application on
the Web to retrieve the quote after which it will print the quote on the
terminal.

The header file input to the soapcpp2 tool is given below. The example is for illustrative purposes only because the XMethods stock quote and currency rate services are no longer operational:

~~~{.cpp}
    // Contents of file "quotex.h": 
    // the first service: stock quotes
    //gsoap ns1 service namespace:  urn:xmethods-delayed-quotes
    //gsoap ns1 service style:      rpc
    //gsoap ns1 service encoding:   encoded
    int ns1__getQuote(char *symbol, float& result);
    // the second service: currency exchange
    //gsoap ns2 service namespace:  urn:xmethods-CurrencyExchange
    //gsoap ns2 service style:      rpc
    //gsoap ns2 service encoding:   encoded
    int ns2__getRate(char *country1, char *country2, float& result);
    // our new service operation: returns currency-converted stock quote
    //gsoap ns3 service name:       quotex
    //gsoap ns3 service style:      rpc
    //gsoap ns3 service encoding:   encoded
    //gsoap ns3 schema  namespace:  urn:quotex
    int ns3__getQuote(char *symbol, char *country, float& result);
~~~

We run:

    soapcpp2 quotex.h

This generates <i>`soapStub.h`</i>, <i>`soapH.h`</i>, <i>`soapC.cpp`</i> (serializers), <i>`soapClient.cpp`</i> (client stub functions), <i>`soapServer.cpp`</i> (server skeleton functions).

The <i>`quotex.cpp`</i> CGI service application is (for source code to create a stand-alone service, see Section \ref stand-alone):

~~~{.cpp}
    // Contents of file "quotex.cpp": 
    #include "soapH.h"
    #include "ns1.nsmap"

    int main()
    {
      struct soap soap; 
      soap_init(&soap); 
      soap_serve(&soap); 
      soap_destroy(&soap);
      soap_end(&soap);
      soap_done(&soap);
    } 

    int ns3__getQuote(struct soap *soap, char *symbol, char *country, float& result) 
    {
      struct soap tsoap;
      float q, r; 
      soap_init(&tsoap);
      if (soap_call_ns1__getQuote(tsoap, "http://services.xmethods.net/soap", "", symbol, &q)
       || soap_call_ns2__getRate(tsoap, "http://services.xmethods.net/soap", NULL, "us", country, &r)
      {
        soap_delegate_deletion(&tsoap, &soap); // move tsoap-deserialized data to the soap context
        if (tsoap->fault)
        {
          soap->fault = tsoap->fault; // if one of the calls returned a SOAP Fault, we use it
          soap_done(&tsoap);
          return SOAP_FAULT;
        }
        soap_done(&tsoap);
        return soap_receiver_fault(soap, "Cannot access services", NULL);
      }
      result = q * r; 
      soap_delegate_deletion(&tsoap, &soap); // move tsoap-deserialized data to the soap context
      soap_done(&tsoap);
      return SOAP_OK; 
    } 

    /* Since this app is a combined client-server, it is put together with 
     * one header file that describes all service operations. However, as a consequence we 
     * have to implement the methods that are not ours. Since these implementations are 
     * never called (this code is client-side), we can make them dummies as below. 
     */ 

    int ns1__getQuote(struct soap *soap, char *symbol, float &result) 
    {
      // dummy: will never be called 
      return SOAP_NO_METHOD;
    }

    int ns2__getRate(struct soap *soap, char *country1, char *country2, float &result) 
    {
      // dummy: will never be called 
      return SOAP_NO_METHOD;
    }

    struct Namespace namespaces[] = 
    {
      { "SOAP-ENV", "http://schemas.xmlsoap.org/soap/envelope/" }, 
      { "SOAP-ENC", "http://schemas.xmlsoap.org/soap/encoding/" }, 
      { "xsi",      "http://www.w3.org/2001/XMLSchema-instance", "http://www.w3.org/*/XMLSchema-instance" }, 
      { "xsd",      "http://www.w3.org/2001/XMLSchema",          "http://www.w3.org/*/XMLSchema" }, 
      { "ns1",      "urn:xmethods-delayed-quotes" }, 
      { "ns2",      "urn:xmethods-CurrencyExchange" }, 
      { "ns3",      "urn:quotex" }, 
      { NULL, NULL } 
    };
~~~

When combining clients and service functionalities, it is recommended to use a single
interface header file input to the soapcpp2 tool, since this header file declares both
client and server functionalities.  As a consequence, however, stub and
skeleton functions are generated for all service operations, while the client part
will only use the stub functions and the service part will use the skeleton functions.  Thus,
dummy implementations of the unused service operations are implemented as shown in the example above, which are in fact never used.

Three WSDL files are generated by soapcpp2: <i>`ns1.wsdl`</i>, <i>`ns2.wsdl`</i>, and
<i>`ns3.wsdl`</i>. Only the <i>`ns3.wsdl`</i> file is required to be published as it
contains the description of the combined service, while the others are
generated as a side-effect.

🔝 [Back to table of contents](#)

### How to switch to REST from SOAP  {#restservice}

To switch to RESTful Web APIs from SOAP Web services APIs is simple, just use a directive.

To declare HTTP POST as the default HTTP method to use with client-side calls for all service operations associated with the `ns` namespace prefix:

~~~{.cpp}
    //gsoap ns service protocol: POST
~~~

To declare the HTTP POST method for a specific service operation, use:

~~~{.cpp}
    //gsoap ns service protocol: POST
    int ns__webmethod(...);
~~~

You can specify `GET`, `PUT`, `POST`, and `DELETE`.  With `GET` the input parameters of the service operations should be primitive types.  See Section \ref directives-1.

🔝 [Back to table of contents](#)

## Asynchronous one-way message passing        {#oneway1}

SOAP messaging is typically synchronous: the client sends a HTTP POST and blocks until the server responds to the request.
The gSOAP tools also support asynchronous one-way messaging over HTTP.

One-way SOAP service operations are declared as function prototypes with the output parameter specified as a
`void` type to indicate the absence of a return value, for example:

~~~{.cpp}
    int ns__event(int eventNo, void);
~~~

The soapcpp2 tool generates the following functions in <i>`soapClient.cpp`</i>:

~~~{.cpp}
    int soap_send_ns__event(struct soap *soap, const char URL, const char action, int event); 
    int soap_recv_ns__event(struct soap *soap, struct ns__event *dummy); 
~~~

The `soap_send_ns__event` function transmits the message to the destination URL by opening a socket and sending the SOAP encoded
message. The socket will remain
open after the send.  To complete the HTTP POST operation we need to call `::soap_recv_empty_response` to accept the server's HTTP OK or Accept response message that should have an empty message body:

~~~{.cpp}
    if (soap_send_ns__event(soap, eventNo) || soap_recv_empty_response(soap)
      soap_print_fault(soap, stderr); 
~~~


The generated `soap_recv_ns__event` function can be used to parse a SOAP message, e.g. on the server side.  But it is not used on the client side.  The `ns__event` structure is declared as:

~~~{.cpp}
    struct ns__event 
    {
        int eventNo; 
    }
~~~

The gSOAP generated <i>`soapServer.cpp`</i> code includes a skeleton function called by `::soap_serve` to process the one-way request message:

~~~{.cpp}
    int soap_serve_ns__event(struct soap *soap);
~~~
This skeleton function calls the user-defined `ns__event(struct soap *soap, int eventNo)` function (note the absence of the void parameter!).
However, when this function returns, the skeleton function does not respond with a SOAP response message since no response data is specified.  Instead, the user-defined `ns__event` function should call `::soap_send_empty_response` to return an empty response message.  For example:

~~~{.cpp}
    int ns__event(struct soap *soap, int eventNo) 
    {
      ... // handle event 
      return soap_send_empty_response(soap, 202); // HTTP 202 Accepted
    }
~~~

🔝 [Back to table of contents](#)

## How to use XML serializers to save and load application data        {#bindings}

The gSOAP XML databindings for C and C++ allow a seamless integration of XML in
C and C++ applications. Data can be serialized in XML and vice versa. WSDL and
XML schema files can be converted to C or C++ definitions. C and C++
definitions can be translated to WSDL and schemas to support legacy ANSI C
applications for example.

This section explains the basics of mapping XML schema types to C/C++ types using the wsdl2h tool.
A more in-depth presentation of C/C++ XML data bindings in gSOAP is documented in [C and C++ XML data bindings](../../databinding/html/index.html).

🔝 [Back to table of contents](#)

### Converting WSDL, WADL, and XML schema to C/C++ with wsdl2h  {#wsdl2h}

The wsdl2h tool takes WSDL, WADL, and XSD files or URLs to WSDL, WADL, and XSD
and generates an interface header file with the command:

     wsdl2h [options] WSDL WADL and XSD files or URLs...

The WSDL 1.1 and 2.0 standards are supported and WADL. If you have any trouble
with wsdl2h being able to process WSDLs and XSD files or URLs, then please
contact Genivia technical support for assistance.

The gSOAP tools support the entire XML schema 1.1 standard,
except XPath expressions and assertions. This covers all of the
following schema components with their optional attributes shown:

<div class="alt">
~~~{.xml}
    <xsd:any minOccurs maxOccurs> 
    <xsd:anyAttribute> 
    <xsd:all> 
    <xsd:choice minOccurs maxOccurs> 
    <xsd:sequence minOccurs maxOccurs> 
    <xsd:group name ref> 
    <xsd:attributeGroup name ref> 
    <xsd:attribute name ref type use default fixed form wsdl:arrayType> 
    <xsd:element name ref type default fixed form nillable abstract substitutionGroup minOccurs maxOccurs> 
    <xsd:simpleType name> 
    <xsd:complexType name abstract mixed>
~~~
</div>

The supported schema facets are:

<div class="alt">
~~~{.xml}
    <xsd:enumeration>        maps to enum
    <xsd:simpleContent>      maps to primitive type or a typedef
    <xsd:complexContent>     maps to a struct or class
    <xsd:list>               maps to enum* (bitmask)
    <xsd:extension>          maps to extended struct or class with a base class
    <xsd:restriction>        maps to typedef, struct or class
    <xsd:length>             validates string lengths
    <xsd:minLength>          validates string lengths
    <xsd:maxLength>          validates string lengths
    <xsd:minInclusive>       validates integer and float types 
    <xsd:maxInclusive>       validates integer and float types 
    <xsd:minExclusive>       validates integer and float types 
    <xsd:maxExclusive>       validates integer and float types 
    <xsd:precision>          float with formatted output 
    <xsd:scale>              float with formatted output 
    <xsd:totalDigits>        float with formatted output 
    <xsd:fractionDigits>     float with formatted output 
    <xsd:pattern>            regex pattern, not automatically validated, see note below 
    <xsd:union>              maps to string, content not validated
~~~
</div>

Also supported are:

<div class="alt">
~~~{.xml}
    <xsd:import> 
    <xsd:include> 
    <xsd:redefine> 
    <xsd:override> 
    <xsd:annotation>
~~~
</div>

A subset of the default type mappings is shown below:

<div class="alt">
~~~{.xml}
    xsd:string              maps to string (char* or std::string)
    xsd:boolean             maps to bool (C++) or enum xsd__boolean (C) 
    xsd:float               maps to float 
    xsd:double              maps to double 
    xsd:decimal             maps to string, or use #import "custom/float128.h" 
    xsd:duration            maps to string, or use #import "custom/duration.h" 
    xsd:dateTime            maps to time_t, or use #import "custom/struct_tm.h" 
    xsd:time                maps to string (white space collapse applied)
    xsd:date                maps to string (white space collapse applied)
    xsd:gYearMonth          maps to string (white space collapse applied)
    xsd:gYear               maps to string (white space collapse applied) 
    xsd:gMonth              maps to string (white space collapse applied) 
    xsd:hexBinary           maps to struct xsd__hexBinary 
    xsd:base64Binary        maps to struct xsd__base64Binary 
    xsd:anyURI              maps to string (white space collapse applied)
    xsd:anyType             maps to an XML string or DOM with wsdl2h -d
    xsd:QName               maps to _QName (QName normalization applied)
    xsd:NOTATION            maps to string (white space collapse applied)
~~~
</div>

Automatic validation of <i>`xsd:pattern`</i>-restricted content is possible with a hook to a regex pattern matching engine, see the `::soap::fsvalidate` and `::soap::fwvalidate` callback documentation in Section \ref callback .

User-defined mappings can be added to <i>`typemap.dat`</i>, which is used by wsdl2h
to map schema types to C/C++ types.  For example, the map <i>`xsd:duration`</i> to a custom serializer, add this line to <i>`typemap.dat`</i>:

    xsd__duration = #import "custom/duration.h" | xsd__duration

Then run wsdl2h with the <i>`typemap.dat`</i> file in the current directory or use [<b>`wsdl2h -t mapfile.dat`</b> option `-t mapfile.dat`](#wsdl2h-t) to use <i>`mapfile.dat`</i> instead.  This requires compiling <i>`gsoap/custom/duration.c`</i> with your build.

Another example is <i>`xsd:dateTime`</i> which is mapped to `time_t`.  To expand the range and precision of <i>`xsd:dateTime`</i> we can map <i>`xsd:dateTime`</i> to `struct tm`:

    xsd__dateTime = #import "custom/struct_tm.h" | xsd__dateTime

or to `struct timeval`:

    xsd__dateTime = #import "custom/struct_timeval.h" | xsd__dateTime

This requires compiling <i>`gsoap/custom/struct_tm.c`</i> or <i>`gsoap/custom/struct_timeval.c`</i>, respectively.

Non-primitive XSD types are supported, with the default mapping shown below:

<div class="alt">
~~~{.xml}
    xsd:normalizedString    maps to string 
    xsd:token               maps to string 
    xsd:language            maps to string 
    xsd:IDREFS              maps to string 
    xsd:ENTITIES            maps to string 
    xsd:NMTOKEN             maps to string 
    xsd:NMTOKENS            maps to string 
    xsd:Name                maps to string 
    xsd:NCName              maps to string 
    xsd:ID                  maps to string 
    xsd:IDREF               maps to string 
    xsd:ENTITY              maps to string 
    xsd:integer             maps to string 
    xsd:nonPositiveInteger  maps to string
    xsd:negativeInteger     maps to string
    xsd:long                maps to LONG64 
    xsd:int                 maps to int 
    xsd:short               maps to short 
    xsd:byte                maps to byte 
    xsd:nonNegativeInteger  maps to string
    xsd:unsignedLong        maps to ULONG64 
    xsd:unsignedInt         maps to unsigned int 
    xsd:unsignedShort       maps to unsigned short 
    xsd:unsignedByte        maps to unsigned byte 
    xsd:positiveInteger     maps to string
    xsd:yearMonthDuration   maps to string
    xsd:dayTimeDuration     maps to string
    xsd:dateTimeStamp       maps to string
~~~
</div>

String targets are defined in the <i>`typemap.dat`</i> file used by
wsdl2h to map XSD types. This allows the use of `char*` and
`std::string`.  It is possible to map any string types to `wchar_t` and `std::wstring` by adding the following line to <i>`typemap.dat`</i>:

    xsd__string = | wchar_t* | wchar_t*

and

    xsd__string = | std::wstring

By default strings are either `char*` (for C) or `std::string` (for C++) which contain ASCII or UTF-8 content enabled with the runtime flag `#SOAP_C_UTFSTRING`.

Note that the XSD types for unlimited numeric values such as <i>`xsd:integer`</i> and <i>`xsd:decimal`</i> are mapped to strings, to preserve the value in case it is too large to store in a 64-bit integer or float.  The mapping can be redefined as follows in <i>`typemap.dat`</i>:

    xsd__decimal            = | double
    xsd__integer            = | LONG64
    xsd__nonNegativeInteger = typedef xsd__integer xsd__nonNegativeInteger 0 :   ; | xsd__nonNegativeInteger
    xsd__nonPositiveInteger = typedef xsd__integer xsd__nonPositiveInteger   : 0 ; | xsd__nonPositiveInteger
    xsd__positiveInteger    = typedef xsd__integer xsd__positiveInteger    1 :   ; | xsd__positiveInteger
    xsd__negativeInteger    = typedef xsd__integer xsd__negativeInteger      : -1; | xsd__negativeInteger

We can also use a `quadmath.h` 128 bit float to store <i>`xsd:decimal`</i>:

    xsd__decimal = #import "custom/float128.h" | xsd__decimal

where `xsd__decimal` is a `__float128` `quadmath.h` type.

There are several initialization flags to control XML serialization at run-time:
  
*  XML validation is more strictly enforced with `#SOAP_XML_STRICT`.
  
*  XML namespaces are enforced when parsing XML, unless disabled with `#SOAP_XML_IGNORENS`.
  
*  XML exclusive canonicalization is enabled with `#SOAP_XML_CANONICAL`.
  
*  XML default `xmlns="..."` namespace bindings are enforced with `#SOAP_XML_DEFAULTNS`.
  
*  XML is indented for enhanced readability with `#SOAP_XML_INDENT`.
  
*  XML <i>`xsi:nil`</i> for NULL struct and class members are serialized with `#SOAP_XML_NIL`.

*  UTF-8 is stored in `char*` and `std::string` with `#SOAP_C_UTFSTRING`.

Strict validation catches all structural XML validation violations. For
primitive type values it depends on the C/C++ target type that XSD types are
mapped to, to catch primitive value content pattern violations. Primitive value
content validation is performed on non-string types such as numerical and time
values. String values are not automatically validated, unless a
<i>`xsd:pattern`</i> is given and the `::soap::fsvalidate` and `::soap::fwvalidate`
callbacks are implemented by the user. Alternatively, deserialized string
content can be checked at the application level.

To obtain C or C++ type definitions for XML schema components, run
wsdl2h on the schemas to generate a data binding interface header file. This header file defines
the C/C++ type representations of the XML schema components. The header file
is then processed by the soapcpp2 tool to generate
the serializers for these types. See Section \ref databindings  for an overview to use wsdl2h and soapcpp2 to map schemas to C/C++ types to obtain XML data bindings.

🔝 [Back to table of contents](#)

### Mapping C/C++ to XML schema with soapcpp2  {#soapcpp2mapping}

To generate serialization code, execute:

     soapcpp2 [options] file.h

The following C/C++ types are supported in the data binding interface header file:

~~~{.cpp}
    bool 
    enum, enum * (enum * is a "product enumeration" representing a bitmask) 
    (signed or unsigned) char, int8_t, short, int16_t, int, int32_t, long, long long, int64_t, LONG64
    size_t (transient, not serializable)
    float, double, long double (#import "custom/long_double.h") 
    std::string, std::wstring, char[], char*, wchar_t* 
    _XML (a char* type to hold literal XML string content) 
    _QName (a char* type with normalized QName content of the form prefix:name) 
    struct, class (with single inheritance) 
    std::vector, std::list, std::deque, std::set
    union (requires preceding discriminant member) 
    typedef 
    time_t 
    template <> class (containers require begin(), end(), size(), and insert() methods)  
    void* (requires a preceding __type member to indicate the object pointed to) 
    struct xsd__hexBinary (special pre-defined type to hold binary content) 
    struct xsd__base64Binary (special pre-defined type to hold binary content) 
    struct tm (#import "custom/struct_tm.h") 
    struct timeval (#import "custom/struct_timeval.h") 
    pointers to any of the above (any pointer-linked structures are serializable, including cyclic graphs) 
    std::shared_ptr, std::unique_ptr, std::auto_ptr
    fixed-size arrays of all of the above
~~~

Additional features and C/C++ syntax requirements:
  
*  A header file should not include any code statements, only data type declarations.
  
*  Nested structs, classes, and unions are un-nested.
  
*  Use `#import "file.h"` instead of `#include` to import other header files. The
  `#include` and `#define` directives are fine to use, but these are moved
  into the generated code and then used by the C/C++ compiler.
  
*  C++ namespaces are supported, but must cover the entire header file content.
  
*  Optional DOM support can be used to store mixed content or literal XML
  content can be stored in `_XML` strings. Otherwise, mixed content may be lost. Use <b>`soapcpp2 -d`</b> option <b>`-d`</b> for DOM
  support.  See the [XML DOM API](../../dom/html/index.html) documentation for details.
  
*  Types are denoted transient using the `extern` qualifier, which prevents
  serialization of types or struct and class members:
~~~{.cpp}
    extern class classname; // this class is not serializable 
    struct structname
    {
        extern char *name; // this member is not serializable
        int num;
    };
~~~
  
*  Only public members of a class can be serialized:
~~~{.cpp}
    class name
    { private:
        char *secret; // private and protected members are not serializable
    };
~~~
   and members are public by default in the interface header file for soapcpp2.
  
*  Types may be declared `volatile` which means that they are declared elsewhere
  in the project's source code base and should not be redefined in the soapcpp2-generated code nor changed by the soapcpp2 tool, for example this makes `struct tm` of `time.h` serializable with a selection of its members specified, where `volatile` prevents soapcpp2 from declaring this struct again:
~~~{.cpp}
    volatile struct tm
    {
        int tm_sec;  ///< seconds (0 - 60)
        int tm_min;  ///< minutes (0 - 59)
        int tm_hour; ///< hours (0 - 23)
        int tm_mday; ///< day of month (1 - 31)
        int tm_mon;  ///< month of year (0 - 11)
        int tm_year; ///< year - 1900
    };
~~~
  
*  Classes and structs may be declared `mutable` means that they can be augmented with additional members using redefinitions of the struct or class:
~~~{.cpp}
    mutable class classname
    { public:
        int n; // classname has a member 'n' 
    };
    mutable class name
    { public:
        float x; // classname also has a member 'x'
    };
~~~
  The `::SOAP_ENV__Header` struct is mutable as well as the `::SOAP_ENV__Fault`, `::SOAP_ENV__Detail`, `::SOAP_ENV__Reason`, and `::SOAP_ENV__Code` structs.  The reason is that these structures are augmented with additional members by plugins such as WS-Addressing <i>`gsoap/plugin/wsaapi.h`</i> to support these SOAP-based protocols.
  
*  Members of a struct or class are serialized as XML attributes when qualified with '@', for example:
~~~{.cpp}
    struct record
    {
      @ char *name; // XML attribute name
        int num;    // XML element num
    };
~~~
  
*  Strings with 8-bit content hold ASCII by default or hold UTF-8 when enabled with runtime flag
  `#SOAP_C_UTFSTRING`. When enabled, all `std::string` and `char*` strings contain UTF-8.  In this way the deserializers populate strings with UTF-8 content and serializers will output strings as holding UTF-8 content.

The soapcpp2 tool generates serializers and deserializers
for all wsdl2h-generated or user-defined data structures that are specified in
the header file input to the soapcpp2 tool. The serializers and deserializers can be
found in the soapcpp2-generated
<i>`soapC.cpp`</i> file. These serializers and deserializers can be used separately by an application without the need to build a
Web services client or service application.  This is useful for applications that need to save or export their data in XML or need to
import or load data stored in XML format.

🔝 [Back to table of contents](#)

### Serializing C/C++ data to XML        {#serialize}

The soapcpp2 tool generates the following readers and writers for each serializable data type defined in the data bindings interface file input to soapcpp2:

* `int soap_read_T(struct soap*, T *data)` parse XML and deserialize into C/C++ data of type `T`, returns `#SOAP_OK` on success.

* `int soap_write_T(struct soap* T *data)` serialize C/C++ data of type `T` into XML, returns `#SOAP_OK` on success.

Where `T` is the name of the data type, such as the struct or class name.  For other types, see the table further below for the naming conventions used by soapcpp2 to generate these functions.

The following `::soap` context variables control the destination and source for XML serialization and deserialization:

* `SOAP_SOCKET ::soap::socket` socket file descriptor for socket connection input and output (or `#SOAP_INVALID_SOCKET` when not set).

* `ostream *::soap::os`      C++ only: output stream used for send operations when non-NULL.

* `const char **::soap::os`  C only: points to a string pointer to be set with the string content produced, the saved string is allocated and managed by the `::soap` context.

* `istream *::soap::is`      C++ only: input stream used for receive operations when non-NULL.

* `const char *::soap::is`   C only: string with input to parse, this pointer advances over the string until a `\0` is found.

* `int ::soap::sendfd` when `::soap::socket` = `#SOAP_INVALID_SOCKET`, this fd is used for send operations, default fd is 1 (stdout).

* `int ::soap::recvfd` when `::soap::socket` = `#SOAP_INVALID_SOCKET`, this fd is used for receive operations, default fd is 0 (stdin).

Additional functions are generated by soapcpp2 for each serializable data type `T` to dynamically allocate data of type `T` on the context-managed heap and to initialize data of type `T`:

* `T * soap_new_T(struct soap*)` allocates and initializes data of type `T` in context-managed heap memory, managed data is deleted with `::soap_destroy` (deletes C++ objects) and `::soap_end` (deletes all other data), and you can also use `::soap_malloc` to allocate uninitialized context-managed memory.

* `void soap_default_T(struct soap*, T*)` initializes data of type `T`, but C++ classes are augmented with a `soap_default(struct soap*)` method that should be called instead to (re)initialize the class instance.  If the class has a `::soap` context pointer member then this member will be set to the first argument passed to this function.

The following extra functions are generated by soapcpp2 for deep copying and deletion of entire data structures when using <b>`soapcpp2 -Ecd`</b> options <b>`-Ec`</b> (deep copy) and <b>`-Ed`</b> (deep deletion):

- `T * soap_dup_T(struct soap*, T *dst, const T *src)`
  deep copy `src` into `dst`, replicating all deep cycles and shared pointers
  when a managing `::soap` context is provided.  When `dst` is NULL, allocates
  space for `dst` and returns a pointer to the allocated copy.  Deep copy results in a
  tree when the `::soap` context is NULL, but the presence of deep cycles will
  lead to non-termination.  Use flag `SOAP_XML_TREE` with managing context to
  copy into a tree without cycles and pointers to shared objects.  Returns
  `dst` or the allocated copy when `dst` is NULL.

- `void soap_del_T(const T*)` deletes all
  heap-allocated members of this object by deep deletion ONLY IF this object
  and all of its (deep) members are not managed by a soap context AND the deep
  structure is a tree (no cycles and co-referenced objects by way of multiple
  (non-smart) pointers pointing to the same data).  Can be safely used after
  `soap_dup(NULL)` to delete the deep copy.  Does not delete the object itself.

The following initializing and finalizing functions should be used before and after calling lower-level IO functions such as `::soap_send`, `::soap_send_raw`, `::soap_get0`, `::soap_get1`, and `::soap_http_get_body` (this is not needed when calling the `soap_read_T` and `soap_write_T` functions):

* `int soap_begin_send(struct soap*)`    start a sending phase.

* `int soap_end_send(struct soap*)`       flush the send buffer.

* `int soap_begin_recv(struct soap*)`     start a receiving phase, if an HTTP header is present, parse it first.

* `int soap_end_recv(struct soap*)`       finalize receiving, read attachments if any, perform SOAP id/href consistency check on deserialized data.

These operations do not setup or open or close files or connections. The application should open and close connections or files and set the `::soap::socket`, `::soap::os` or `::soap::sendfd`, `::soap::is` or `::soap::recvfd` streams or descriptors.
When `::soap::socket` is `#SOAP_INVALID_SOCKET` and none of the streams and descriptors are set, then the standard input and output will be used.

The following options are available to control serialization:

~~~{.cpp}
    soap_set_mode(soap, SOAP_XML_TREE);                 // use this for XML without id-ref (no cycles!)
    soap_set_mode(soap, SOAP_XML_GRAPH);                // or use this for XML with id-ref (including cycles)
    soap_set_namespaces(soap, struct Namespace *nsmap); // set a XML namespace table with xmlns bindings
~~~

See also Section \ref flags  to control the I/O buffering and content encoding such as compression.

To accurately and safely serialize data structures with cycles and co-referenced objects to an XML stream, two generated functions are called by `soap_write_T` to serialize data of type `T`: 
`soap_serialize_T` to perform a deep analysis of pointers to detect co-referenced data and cycles,
and `soap_put_T` to output the data in XML with id-href (or id-ref) attributes for co-referenced data and cycles.  Multi-references with id-href (and id-ref) are part of the SOAP protocol to serialize data accurately, i.e. retaining the structural integrity of the data sent and received.  
Flag `#SOAP_XML_TREE` turns id-href (id-ref) attributes off (makes `soap_serialize_T` a no-op and ignores them on the receiving end).  Flag `#SOAP_XML_GRAPH` should be used with non-SOAP XML output to accurately and safely serialize data structure graphs with co-referenced objects and cycles.

The `soap_serialize_T` and `soap_put_T` calls are performed by the generated `soap_write_T` functions, which also call `::soap_begin_send` and `::soap_end_send`.

The following table lists the type naming conventions used by soapcpp2 to generate functions:

type                 | name
-------------------- | ----------
`char*`              | `string` 
`wchar_t*`           | `wstring` 
`std::string`        | `std__string` 
`std::wstring`       | `std__wstring` 
`char`               | `byte` 
`bool`               | `bool` 
`double`             | `double` 
`int`                | `int` 
`float`              | `float` 
`long`               | `long` 
`long long`          | `LONG64`
`short`              | `short` 
`time_t`             | `time` 
`unsigned char`      | `unsignedByte` 
`unsigned int`       | `unsignedInt` 
`unsigned long`      | `unsignedLong` 
`unsigned long long` | `ULONG64`
`unsigned short`     | `unsignedShort` 
`T`[N]               | `ArrayN`OfType where Type is the type name of T 
`T*`                 | `PointerToType` where Type is the type name of T 
`std::vector<T>`     | `TemplateOfType` where Type is the type name of T 
`struct Name`        | `Name` 
`class Name`         | `Name` 
`enum Name`          | `Name` 

Consider for example the following interface header file for soapcpp2 declares a
`struct ns__Person`:

~~~{.cpp}
    struct ns__Person
    {
        char *name;
    };
~~~

To parse and deserialize a person variable `p` from XML:

~~~{.cpp}
    struct soap *soap = soap_new();
    struct ns__Person *p = soap_new_ns__Person(soap, -1); // -1 is one (non-array)
    soap->recvfd = 0;       // parse from stdout
    // soap->is = &std::in; // parse from stdin stream (C++ only)
    // soap->is = cs;       // parse from a char *cs string (C only)
    if (soap_read_ns__Person(soap, p))
      soap_print_fault(soap, stderr);
    ... // use value p
    soap_destroy(soap);
    soap_end(soap);
    soap_free(soap);
~~~

To parse and deserialize XML from a file:

~~~{.cpp}
    soap->recvfd = open(file, O_RDONLY); 
    if (soap->recvfd >= 0)
      if (soap_read_ns__Person(soap, p))
        soap_print_fault(soap, stderr);
    close(soap->recvfd);
    soap->recvfd = 0;
~~~

To parse and deserialize XML from a C++ file stream:

~~~{.cpp}
    std::fstream fs;
    fs.open(file, std::ios::in);
    if (fs)
    {
      soap->is = &fs;
      if (soap_read_ns__Person(soap, p))
        soap_print_fault(soap, stderr);
      fd.close();
      soap->is = NULL;
    }
~~~

Or to parse and deserialize XML from a string stream in C++:

~~~{.cpp}
    std::stringstream ss;
    ... // populate stream ss
    soap->is = &ss;
    if (soap_read_ns__Person(soap, p))
      soap_print_fault(soap, stderr);
    soap->is = NULL;
~~~

To parse and deserialize XML from a string `cs` in C:

~~~{.cpp}
    const char *cs = "..."; // populate string cs
    soap->is = &cs;
    if (soap_read_ns__Person(soap, p))
      soap_print_fault(soap, stderr);
    soap->is = NULL;
~~~

To serialize a person variable `p` in XML:

~~~{.cpp}
    struct soap *soap = soap_new1(SOAP_XML_INDENT);
    struct ns__Person *p = soap_new_ns__Person(soap, -1); // -1 is one (non-array)
    p->name = "Joe";
    soap->sendfd = 1;         // send to stdout
    // soap->os = &std::cout; // send to stdout stream (C++ only)
    // soap->os = &cs;        // send to a char *cs string (C only)
    if (soap_write_ns__Person(soap, p))
      soap_print_fault(soap, stderr);
    soap_destroy(soap);
    soap_end(soap);
    soap_free(soap);
~~~

This produces:

<div class="alt">
~~~{.xml}
    <ns:Person xmlns:ns="..." ... > 
      <name>Joe</name> 
    </ns:Person>
~~~
</div>

To send the output to a file:

~~~{.cpp}
    soap->sendfd = open(file, O_RDWR|O_CREAT, S_IWUSR|S_IRUSR); 
    if (soap->sendfd >= 0)
      if (soap_write_ns__Person(soap, p))
        soap_print_fault(soap, stderr);
    close(soap->sendfd);
    soap->sendfd = 1;
~~~

To send the output to a C++ file stream:

~~~{.cpp}
    std::fstream fs;
    fs.open(file, std::ios::out);
    if (fs)
    {
      soap->os = &fs;
      if (soap_write_ns__Person(soap, p))
        soap_print_fault(soap, stderr);
      fd.close();
      soap->os = NULL;
    }
~~~

Or send the output to a C++ string stream to save XML in a string:

~~~{.cpp}
    std::stringstream ss;
    soap->os = &ss;
    if (soap_write_ns__Person(soap, p))
      soap_print_fault(soap, stderr);
    std::strings s = ss.str();
    soap->os = NULL;
~~~

To save the output to a string `cs` in C:

~~~{.cpp}
    char *cs;
    soap->os = &cs;
    if (soap_write_ns__Person(soap, p))
      soap_print_fault(soap, stderr);
    soap->os = NULL;
~~~

The string `cs` is populated with XML when successful.  This string is managed
by the context and deleted with `::soap_end`.

As we explained, the `soap_write_T` functions call `soap_serialize_T`, which
must be called when the data structure graph to serialize contains
co-referenced data and cycles.  It must be called to preserve the logical
coherence of pointer-based data structures, where pointers may refer to
co-referenced objects.  By calling `soap_serialize_T`, data structures shared
through pointers are serialized only once and referenced in XML using id-refs
attributes.  The actual id-refs used depend on the SOAP encoding. To turn off
SOAP encoding, remove or avoid using the SOAP-ENV and SOAP-ENC namespace
bindings in the namespace table.  In addition, the `#SOAP_XML_TREE` and
`#SOAP_XML_GRAPH` flags can be used to control the output by restricting
serialization to XML trees or by enabling multi-ref graph serialization with
id-ref attributes.

To save the data as an XML tree (with one root) without any id-ref attributes, use the
`#SOAP_XML_TREE` flag. The data structure must not contain pointer-based cycles.
This flag also instructs the XML parser and deserializer to ignore id-ref attributes.

To preserve the exact structure of the data object graph and create XML with
one root, use the `#SOAP_XML_GRAPH` output-mode flag (see Section \ref flags ).
Using the `#SOAP_XML_GRAPH` flag assures the preservation of the logical
structure of the data.

Using `#SOAP_XML_TREE` means that no id-refs are output or parsed.  With this
flag the output will serialize nodes as a tree in XML, which means that nodes
may be duplicated when shared by multiple pointers and cycles are broken to
prevent infinite serialization.  To preserve the graph structure of the nodes
in the data structure, use `#SOAP_XML_GRAPH` or use SOAP 1.1 or 1.2
multi-reference serialization (this is the default mode with SOAP
serialization).

Consider for example the following `struct`:

~~~{.cpp}
    struct Tricky
    {
        int *p; 
        int n; 
        int *q; 
    };
~~~

The following fragment initializes the pointer members `p` and `q` to point to the value of member `n`:

~~~{.cpp}
    struct soap *soap = soap_new(); 
    struct Tricky X; 
    X.n = 123; 
    X.p = &X.n; 
    X.q = &X.n; 
    soap_write_Tricky(soap, &X);
    soap_destroy(soap);
    soap_end(soap);
~~~

What is special about this data structure is that members `p` and `q` both point to member `n`. When using SOAP 1.1 with gSOAP, the serializers strategically place id elements (also called SOAP 1.1 independent elements) after the root element to identify shared values, where the href attributes of elements <i>`p`</i> and <i>`q`</i> point to:

<div class="alt">
~~~{.xml}
    <Tricky> 
      <p href="#_1"/>
      <n>1</n>
      <q href="#_1"/>
    </Tricky>
    <id id="_1">123</id>
~~~
</div>

The above is not valid as plain XML, because there is no single root element, but it is valid XML when placed in a SOAP Body element as intended with SOAP 1.1 messaging.

When SOAP 1.2 is used with gSOAP, the output is more accurate, because now both elements <i>`p`</i> and <i>`q`</i> point to element <i>`n`</i>:

<div class="alt">
~~~{.xml}
    <Tricky>
      <p SOAP-ENC:ref="_1"/>
      <n SOAP-ENC:id="_1">123</n>
      <q SOAP-ENC:ref="_1"/>
    </ns:Tricky>
~~~
</div>

Without using SOAP encoding but using plain XML instead with the `#SOAP_XML_GRAPH` flag set, the output is also accurate with both elements <i>`p`</i> and <i>`q`</i> pointing to element <i>`n`</i>:

<div class="alt">
~~~{.xml}
    <Tricky> 
      <p ref="_1"/>
      <n id="_1">123</n>
      <q ref="_1"/>
    </Tricky>
~~~
</div>

In the last two cases, the generated deserializer for this data type will be able to accurately reconstruct the instance with members `p` and `q` pointing to member `n`.

Finally, serialization with `#SOAP_XML_TREE` produces XML trees, which may benefit interoperability but sacrifices the true meaning of serialization, giving three copies of the shared value `1`:

<div class="alt">
~~~{.xml}
    <Tricky> 
      <p>123</p>
      <n>123</n>
      <q>123</q>
    </Tricky>
~~~
</div>

With the soapcpp2-generated serializers you can define a C++ operator that serializes a specified class instance or type as follows, assuming the `ns__Person` class is declared in an interface header file for soapcpp2:

~~~{.cpp}
    class ns__Person
    { public:
        ns__Person();
        ~ns__Person();
        void set_name(const char *);
        const char *get_name();
        const char *name;
        struct soap *soap;
    };
~~~

Run <b>`soapcpp2 -0`</b> on this file to generate the serializers with non-SOAP XML namespaces, which we then use in our main program as follows:

~~~{.cpp}
    #include "soapH.h"
    #include "ns.nsmap"

    ns__Person::ns__Person()
    {
      name = NULL;
      soap = soap_new1(SOAP_XML_INDENT | SOAP_XML_TREE); // or SOAP_XML_GRAPH
    }

    ns__Person::~ns__Person()
    {
      soap_destroy(soap);
      soap_end(soap);
      soap_free(soap);
    }

    void ns__Person::set_name(const char *name)
    {
      this->name = soap_strdup(this->soap, name);
    }

    const char *ns__Person::get_name()
    {
      return this->name;
    }

    std::ostream& operator<<(std::ostream& o, ns__Person& p)
    {
      p.soap->os = &o;
      soap_write_ns__Person(p.soap, &p);
      p.soap->os = NULL;
      return o;
    }

    std::istream& operator>>(std::istream& i, ns__Person& p)
    {
      p.soap->is = &i;
      soap_read_ns__Person(p.soap, &p);
      p.soap->is = NULL;
      return i;
    }

    int main()
    {
      ns__Person p;
      p.set_name("Joe");
      // serialize person p in XML to stdout:
      std::cout << p << std::endl;
      // then parse and deserialize XML from stdin into person p:
      std::cin >> p;
      // destructor cleans up person p and its deserialized data
    }
~~~

In this example we construct an instance of `ns__Person` by setting its `::soap` context struct pointer data member to a new valid context that is deleted by the destructor of this instance.

@warning Deserialized class instances with a `::soap` context struct pointer member will have their `::soap` contexts set automatically by the deserializer's context, because `soap_default_T` (or `soap_default` class method) is called that sets the `::soap` context struct pointer of the instance.  For example, `soap_read_ns__Person` sets the deserialized `ns__Person::soap` member to the first argument `soap` of `soap_read_ns__Person`, which happens to be `ns__Person::soap` anyway in the example shown above. See Section \ref classmemory .

The output of this program is:

<div class="alt">
~~~{.xml}
    <?xml version="1.0" encoding="UTF-8"?>
    <ns:Person
        xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
        xmlns:xsd="http://www.w3.org/2001/XMLSchema"
        xmlns:ns="http://tempuri.org/ns.xsd">
      <name>Joe</name>
    </ns:Person>
~~~
</div>

The <i>`xsi`</i> and <i>`xsd`</i> namespaces are used when attributes such as
<i>`xsi:nil`</i> and <i>`xsi:type`</i> are serialized, where <i>`xsi:type`</i>
may refer to a XSD type such as <i>`xsd:string`</i>.  The <i>`xsi:nil`</i>
attribute is output when an element is nillable but its corresponding pointer
member is NULL and <i>`xsi:type`</i> is output for structs and classes with
polymorphic members that are declared with `int __type` member and a `void*`
pointer to serialize the value pointed to.  Attribute <i>`xsi:type`</i> is also
output to serialize derived instances in place of base class instances.
Therefore, removing these namespaces from the XML namespace table
(<i>`ns.nsmap`</i>) may cause XML parsing and validation issues.

🔝 [Back to table of contents](#)

### Example        {#example9}

As an example, consider the following data type declarations:

~~~{.cpp}
    // Contents of file "person.h": 
    enum ns__Gender { male, female }; 

    class ns__Address 
    { public: 
        const char *street; 
        uint32_t    number; 
        const char *city; 
    }; 

    class ns__Person 
    { public: 
        const char     *name; 
        enum ns__Gender gender; 
        ns__Address     address; 
        ns__Person     *mother; 
        ns__Person     *father; 
    };
~~~

The following program uses these data types to write to standard output a data structure that contains the data of a person named "John" living at Downing st. 10 in Londen. He has a mother
"Mary" and a father "Stuart". After initialization, the class instance for "John" is serialized and encoded in XML to the
standard output stream using gzip compression (requires the Zlib library, compile sources with the compile-time flag `#WITH_GZIP`):

~~~{.cpp}
    // Contents of file "person.cpp": 
    #include "soapH.h" 
    #include "ns.nsmap"

    int main() 
    {
      struct soap *soap = soap_new1(SOAP_XML_GRAPH); 
      ns__Person mother, father, john; 
      mother.soap_default(soap);
      father.soap_default(soap);
      john.soap_default(soap);
      mother.name = "Mary"; 
      mother.gender = female; 
      mother.address.street = "Downing st."; 
      mother.address.number = 10; 
      mother.address.city = "London"; 
      mother.mother = NULL; 
      mother.father = NULL; 
      father.name = "Stuart"; 
      father.gender = male; 
      father.address.street = "Main st."; 
      father.address.number = 5; 
      father.address.city = "London"; 
      father.mother = NULL; 
      father.father = NULL; 
      john.name = "John"; 
      john.gender = male; 
      john.address = mother.address; 
      john.mother = &mother; 
      john.father = &father; 
      soap_write_ns__Person(soap, &john);
      soap_destroy(soap); 
      soap_end(soap); 
      soap_free(soap); 
    } 
~~~

The <i>`person.h`</i> interface header file is input to soapcpp2 and the generated code compiled together with <i>`person.cpp`</i>:

     soapcpp2 -0 person.h
     c++ -o person person.cpp soapC.cpp stdsoap2.cpp

We run the application:

     ./person

The output is:

<div class="alt">
~~~{.xml}
    <ns:Person
        xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" 
        xmlns:xsd="http://www.w3.org/2001/XMLSchema" 
        xmlns:ns="urn:person">
      <name>John</name> 
      <gender>male</gender> 
      <address> 
        <street id="_3">Dowling st.</street> 
        <number>10</number> 
        <city id="_4">London</city> 
      </address> 
      <mother> 
        <name>Mary</name> 
        <gender>female</gender> 
        <address> 
          <street ref="_3"/> 
          <number>5</number> 
          <city ref="_4"/> 
        </address> 
      </mother> 
      <father> 
        <name>Stuart</name> 
        <gender>male</gender> 
        <address> 
          <street>Main st.</street> 
          <number>13</number> 
          <city ref="_4"/> 
        </address> 
      </father> 
    </ns:Person>
~~~
</div>

Because the C++ compiler stores the constant strings `"Dowling st."` and `"London"` just once, references are included in the output with flag `#SOAP_XML_GRAPH` that preserves the original structure of the data structure serialized.

The following program decodes this content from standard input and reconstructs the original data structure on the heap:

~~~{.cpp}
    #include "soapH.h" 
    #include "ns.nsmap"

    int main() 
    {
      struct soap *soap = soap_new(); 
      ns__Person john;
      if (soap_read_ns__Person(soap, &john))
      {
        soap_print_fault(soap, stderr);
      }
      else
      {
        ns__Person *mother = john->mother; 
        ns__Person *father = john->father; 
        ... // use the data
      }
      soap_destroy(soap); // deletes john, mother and father
      soap_end(soap);     // deletes other managed data and temporaries
      soap_free(soap);    // finalize and delete the context
    } 
~~~

🔝 [Back to table of contents](#)

### Default values for omitted XML elements and attributes        {#default}

The soapcpp2 tool generates `soap_default_T` functions for serializable types
`T` specified in an interface header file for soapcpp2  The default values of
primitive C/C++ types can be easily specified by defining any one or all of the
following macros before including the <i>`gsoap/stdsoap2.h`</i> file or by using
`#SOAPDEFS_H` or `#WITH_SOAPDEFS_H`:

~~~{.cpp}
    #define SOAP_DEFAULT_bool 
    #define SOAP_DEFAULT_byte 
    #define SOAP_DEFAULT_double 
    #define SOAP_DEFAULT_float 
    #define SOAP_DEFAULT_int 
    #define SOAP_DEFAULT_long 
    #define SOAP_DEFAULT_LONG64 
    #define SOAP_DEFAULT_short 
    #define SOAP_DEFAULT_string 
    #define SOAP_DEFAULT_time 
    #define SOAP_DEFAULT_unsignedByte 
    #define SOAP_DEFAULT_unsignedInt 
    #define SOAP_DEFAULT_unsignedLong 
    #define SOAP_DEFAULT_unsignedLONG64 
    #define SOAP_DEFAULT_unsignedShort 
    #define SOAP_DEFAULT_wstring
~~~

The absence of a data value in a receiving SOAP message will result in the
assignment of a default value to a primitive type upon deserialization.

Default values can also be assigned to individual `struct` and `class` members of primitive type or pointers to primitive types. For example:

~~~{.cpp}
    struct MyRecord 
    {
        char *name = "Unknown"; // optional
        int value = 9999; 
        enum Status { active, passive } status = passive; 
    }
~~~

Default values are assigned to the members of a struct or class when parsing and deserializing XML into data when XML elements or attributes with the respective values are absent.  Assigning default values to members makes these members optional elements and attributes in the corresponding XML schema.

Because service operation requests and responses are essentially structs (internally they are structs), default values can also be assigned to service operation parameters. These
default parameter values do not specify optional parameters as we normally see with C/C++ function calls.
Rather, the default parameter values are used in case an inbound request or response message lacks the XML
elements that comprise these parameters. For example, a Web service can use default values to fill-in absent parameters in a
SOAP request as follows:

~~~{.cpp}
    int ns__login(char *uid = "anonymous", char *pwd = "guest", bool granted = true);
~~~

When the request message lacks <i>`uid`</i> or <i>`pwd`</i> elements then the default values are assigned instead.

In addition, the default values will show up in the SOAP or XML request and response message examples generated by the soapcpp2 tool.

🔝 [Back to table of contents](#)

# The wsdl2h tool           {#wsdlin}

The wsdl2h tool is an advanced XML data binding tool to convert WSDLs
and XML schemas (XSD files) to C or C++. The tool takes WSDL and XSD files
or URLs to WSDLs and XSDs, then converts these to a C or C++ interface header file that specifies
the properties of the WSDLs and XSDs in a familiar C/C++ syntax.
This header file is not intended to be included in your
code directly. It should be converted by soapcpp2 to generate the logic
for the data bindings. It can however be safely converted by a documentation
tool such as Doxygen to analyze and represent the service operations and data
in a convenient layout. To this end, the generated interface header file is self-explanatory.

The wsdl2h tool can also be used without WSDLs to convert XML schemas (XSDs)
to C/C++ to implement XML data bindings in C and C++.  The wsdl2h tool generates the XML data binding interface header file with the C/C++ data type equivalents to the XML schema types and components.

The soapcpp2 tool then generates the XML data binding implementation source
code from the data binding interface header file, meaning the serialization source code to serialize C/C++ data in XML and the client-side stub functions to invoke remote service operations and the server-side skeleton functions to implement XML Web services.

Therefore, the creation of C and C++ applications from one of more WSDLs or XSDs
is a two-step process.

First, to convert a WSDL to C++ we use:

     wsdl2h file.wsdl

This generates an interface header file <i>`file.h`</i>. When using a URL to the WSDL we use [<b>`wsdl2h -o file.h`</b> option <b>`-o file.h`</b>](#wsdl2h-o) to save the file:

     wsdl2h -ofile.h http://www.example.com/file.wsdl

Web service operations in the generated <i>`file.h`</i> header file are converted to function prototypes. Schema types are
converted to the equivalent C/C++ types, using file <i>`typemap.dat`</i> to map XML schema types to C/C++ types.

The generated header file also contains instructions for the user and has
documentation copies from the WSDL as well as various directives related to the
Web service properties defined in the WSDL.

Multiple WSDL specifications can be processed at once and saved to one interface header file with [<b>`wsdl2h -o file.h`</b> option <b>`-o file.h`</b>](#wsdl2h-o):

     wsdl2h -o file.h file1.wsdl file2.wsdl file3.wsdl

To generate C source code, use [<b>`wsdl2h -c`</b> option <b>`-c`</b>](#wsdl2h-c):

     wsdl2h -c file.wsdl

The wsdl2h tool does not require WSDLs, it also works for XSDs:

     wsdl2h -o file.h file1.xsd file2.xsd file3.xsd

In this case no service operations are found and therefore the interface header file generated does not contain function prototypes representing service operations.

When upgrading gSOAP to a newer version it is often not necessary to perform this first step again, since newer versions are backward compatible to previous interface header files generated by wsdl2h.

Next, the wsdl2h-generated interface header file <i>`file.h`</i> is input to the
soapcpp2 tool to generate the XML data binding implementation logic in C or C++:

     soapcpp2 file.h

You can use soapcpp2 without wsdl2h, by specifying an input interface header file that is not generated by wsdl2h but written by hand for example, see \ref soapcpp2.

There are many cases when wsdl2h generates code with `#import` directives, such as `#import "stlvector.h"`, that requires the soapcpp2 tool to import definitions from the <i>`gsoap/import`</i> directory, which can be specified as follows:

     soapcpp2 -I some_path_to/gsoap/import file.h

When WSDLs are converted to C++ source code, you may want to use [<b>`wsdl2h -j`</b> option <b>`-j`</b>](#wsdl2h-j) (or [<b>`wsdl2h -j`</b> option <b>`-i`</b>](#wsdl2h-i)) to generate proxy and service classes:

     soapcpp2 -j file.h

This command generates a couple of C++ source files, more details will follow
in Section \ref soapcpp2.

Consider for example the following commands to implement a C++ client application:

     wsdl2h -o calc.h http://www.genivia.com/calc.wsdl 
     soapcpp2 -C -j -I path_to/gsoap/import calc.h

The first command generates <i>`calc.h`</i> from the WSDL at the specified URL.
The header file is then processed by the soapcpp2 tool to generate the
proxy class declared in <i>`soapcalcProxy.h`</i> class and defined in <i>`soapcalcProxy.cpp`</i>.
It also generates a file <i>`calc.nsmap`</i> with a XML namespace table which should be
included in our source code.  The tool also generates <i>`soapStub`</i>, <i>`soapH.h`</i>, and
<i>`soapC.cpp`</i>.  The latter is also compiled with our application together
with <i>`gsoap/stdsoap2.cpp`</i>.

🔝 [Back to table of contents](#)

## wsdl2h options  {#wsdl2hoptions}

The wsdl2h tool generates one XML data binding interface header file, a file that includes
all of the information gathered from the WSDLs and XSDs input to the
wsdl tool as command-line arguments. The default output file name of wsdl2h
is the first WSDL/schema input file name but with extension <i>`.h`</i> that replaces
<i>`.wsdl`</i> (or replaces <i>`.xsd`</i> in case of XSD files specified). When
an input file is absent or a WSDL file is loaded from a Web URL, the header
output will be produced on the standard
output unless [<b>`wsdl2h -o file.h`</b> option <b>`-o file.h`</b>](#wsdl2h-o) is used to save
the output to <i>`file.h`</i> (or any other file name specified).

The wsdl2h command-line options are:

option                                    | result
----------------------------------------- | ------
[`-a`](#wsdl2h-a)                         | generate indexed struct names for local elements with anonymous types 
[`-b`](#wsdl2h-b)                         | generate bi-directional operations to serve one-way response messages (duplex)
[`-c`](#wsdl2h-c)                         | generate C source code 
[`-c++`](#wsdl2h-c)                       | generate C++ source code (default)
[`-c++11`](#wsdl2h-c)                     | generate C++11 source code
[`-D`](#wsdl2h-D)                         | make attribute members with default/fixed values optional with pointers
[`-d`](#wsdl2h-d)                         | generate DOM code for xsd:any and xsd:anyType elements 
[`-e`](#wsdl2h-e)                         | don't qualify enum names
[`-F`](#wsdl2h-F)                         | add transient members to structs to simulate struct-type derivation in C
[`-f`](#wsdl2h-f)                         | generate flat C++ class hierarchy by removing inheritance
[`-g`](#wsdl2h-g)                         | generate global top-level element and attribute declarations 
[`-h`](#wsdl2h-h)                         | display help info and exit
[`-I`](#wsdl2h-I) `path`                  | use `path` to locate WSDL and XSD files
[`-i`](#wsdl2h-i)                         | don't import (advanced option)
[`-j`](#wsdl2h-j)                         | don't generate `::SOAP_ENV__Header` and `::SOAP_ENV__Detail` definitions 
[`-k`](#wsdl2h-k)                         | don't generate `::SOAP_ENV__Header` `mustUnderstand` qualifiers 
[`-L`](#wsdl2h-L)                         | generate less documentation by removing generic `@note` comments
[`-l`](#wsdl2h-l)                         | display license information
[`-M`](#wsdl2h-M)                         | suppress error "must understand element with wsdl:required='true'"
[`-m`](#wsdl2h-m)                         | use xsd.h module to import primitive types 
[`-N`](#wsdl2h-N) `name`                  | use `name` for service prefixes to produce a service for each binding 
[`-n`](#wsdl2h-n) `name`                  | use `name` as the base namespace prefix name instead of `ns` 
[`-O1`](#wsdl2h-O)                        | optimize by omitting duplicate choice/sequence members
[`-O2`](#wsdl2h-O)                        | optimize `-O1` and omit unused schema types (unreachable from roots)
[`-O3`](#wsdl2h-O)                        | optimize `-O2` and omit unused schema root attributes
[`-O4`](#wsdl2h-O)                        | optimize `-O3` and omit unused schema root elements (use only with WSDLs)
[`-Ow2`](#wsdl2h-O)                       | optimize `-O2` while retaining all derived types of used base types
[`-Ow3`](#wsdl2h-O)                       | optimize `-O3` while retaining all derived types of used base types
[`-Ow4`](#wsdl2h-O)                       | optimize `-O4` while retaining all derived types of used base types
[`-o`](#wsdl2h-o) `file`                  | output to file 
[`-P`](#wsdl2h-P)                         | don't create polymorphic types inherited from `xsd__anyType` 
[`-p`](#wsdl2h-p)                         | create polymorphic types inherited from base `xsd__anyType` (automatic when the WSDL or XSD contains polymorphic definitions)
[`-Q`](#wsdl2h-Q)                         | make `xsd__anySimpleType` equal to `xsd__anyType` to use as the base type
[`-q`](#wsdl2h-q) `name`                  | use `name` for the C++ namespace of all declarations 
[`-R`](#wsdl2h-R)                         | generate REST operations for REST bindings in the WSDL 
[`-r`](#wsdl2h-r) `host[:port[:uid:pwd]]` | connect via proxy `host`, `port`, and proxy credentials `uid` and `pwd`
[`-r`](#wsdl2h-r) `:uid:pwd`              | connect with authentication credentials `uid` and `pwd`
[`-S`](#wsdl2h-S) `name`                  | use `name` instead of `soap` for the soap context included in C++ classes as a member variable or use `-S ""` to remove it
[`-s`](#wsdl2h-s)                         | don't generate STL code (no std::string and no std::vector) 
[`-t`](#wsdl2h-t) `file`                  | use type map file instead of the default file typemap.dat 
[`-U`](#wsdl2h-U)                         | map Unicode XML names to UTF-8-encoded Unicode C/C++ identifiers 
[`-u`](#wsdl2h-u)                         | don't generate unions 
[`-V`](#wsdl2h-V)                         | display the current version and exit
[`-v`](#wsdl2h-v)                         | verbose output 
[`-W`](#wsdl2h-W)                         | suppress warnings 
[`-w`](#wsdl2h-w)                         | always wrap response parameters in a response struct 
[`-X`](#wsdl2h-X)                         | don't qualify part names to disambiguate doc/lit wrapped patterns
[`-x`](#wsdl2h-x)                         | don't generate `_XML any` and `_XML anyAttribute` extensibility elements 
[`-y`](#wsdl2h-y)                         | generate typedef synonyms for structs and enums 
[`-z1`](#wsdl2h-z)                        | compatibility with 2.7.6e: generate pointer-based arrays
[`-z2`](#wsdl2h-z)                        | compatibility with 2.7.15: (un)qualify element/attribute referenced members
[`-z3`](#wsdl2h-z)                        | compatibility with 2.7.16 to 2.8.7: (un)qualify element/attribute referenced members
[`-z4`](#wsdl2h-z)                        | compatibility up to 2.8.11: don't generate union structs in std::vector 
[`-z5`](#wsdl2h-z)                        | compatibility up to 2.8.15: don't include minor improvements
[`-z6`](#wsdl2h-z)                        | compatibility up to 2.8.17: don't include minor improvements
[`-z7`](#wsdl2h-z)                        | compatibility up to 2.8.59: don't generate `std::vector` of class of union
[`-z8`](#wsdl2h-z)                        | compatibility up to 2.8.74: don't generate qualifiers for doc/lit wrapped patterns
[`-z9`](#wsdl2h-z)                        | compatibility up to 2.8.93: always qualify element/attribute referenced members, even when defined in the same namespace with default forms unqualified
[`-z10`](#wsdl2h-z)                       | compatibility up to 2.8.96: generate qualifiers even when defined without namespace
[`-_`](#wsdl2h-_)                         | don't generate `_USCORE` (replace with Unicode `_x005f`) 

The following subsections explain the options in detail.  The source code
examples generated by wsdl2h are slightly simplified by removing comments
and some other details without changing their meaning to improve readability.

🔝 [Back to table of contents](#)

### wsdl2h -a {#wsdl2h-a}

This option generates indexed identifier names for structs, classes, unions,
and enums declared for local elements with local (i.e. anonymous) types.  When
local elements and attributes have local types that are mapped to a struct,
class, union, or enum, the generated type name is normally the outer
struct/class name concatenated with the element/attribute name.

For example:

<div class="alt">
~~~{.xml}
    <xsd:schema targetNamespace="urn:example" xmlns:xsd="http://www.w3.org/2001/XMLSchema">
      <xsd:complexType name="TypeWithNestedType">
        <xsd:sequence>
          <xsd:element name="element">
            <xsd:complexType>
              <xsd:sequence>
                <xsd:element name="nested-element" type="xsd::string"/>
              </xsd:sequence>
            </xsd:complexType>
          </xsd:element>
        </xsd:sequence>
      </xsd:complexType>
    </xsd:schema>
~~~
</div>

By default without this option, this schema is translated by wsdl2h to the
following interface header file declaration:

~~~{.cpp}
    class ns__TypeWithNestedType             // complexType
    { public:
        class ns__TypeWithNestedType_element // local complexType
        { public:
            std::string nested_element;      // nested required element
        } element;                           // required element
    };
~~~

By contrast, with <b>`wsdl2h -a`</b> option <b>`-a`</b> we obtain an indexed
local class `_ns__struct_1` in the generated interface header file for
soapcpp2:

~~~{.cpp}
    class ns__TypeWithNestedType          // complexType
    { public:
        class _ns__struct_1               // local complexType
        { public:
            std::string nested_element;   // nested required element
        } element;                        // required element
    };
~~~

The next local struct or class is named `_ns__struct_2` and so on.  The same
indexing applies to local unions and enums.

@note The soapcpp2 tool always un-nests nested struct, class, union and enum
declarations, which means that the above eventially results in the following
source code generated by soapcpp2:

~~~{.cpp}
    class _ns__struct_1
    { public:
        std::string nested_element;
    };
    class ns__TypeWithNestedType
    { public:
        _ns__struct_1 element;
    };
~~~

🔝 [Back to table of contents](#)

### wsdl2h -b {#wsdl2h-b}

This option generates bi-directional operations (duplex operations) intended
for asynchrounous server operations.  The bi-directional operations for server
response messages are generated in addition to the request-response operations.

For example, the wsdl2h tool generates the following declaration of a service
operation `ns__add` for a hypothetical calculator Web service:

~~~{.cpp}
    int ns__add(
        double a,
        double b,
        double& result
    );
~~~

By contrast, with this option <b>`-b`</b> we obtain an additional one-way
operation `ns__addResponse` to send and receive one-way response messages:

~~~{.cpp}
    int ns__addResponse(
        double result,
        void
    );
    int ns__add(
        double a,
        double b,
        double& result
    );
~~~

Where `void` as a result parameter means that the operation uses "one-way"
messaging, in this case to send and receive response messages one-way
asynchronously:

- `int soap_send_ns__addResponse(struct soap *soap, const char *endpoint, const char *action, double& result)`
- `int soap_recv_ns__addResponse(struct soap *soap, double& result)`

At the sender side use `soap_send_ns__addResponse` to send the message one-way,
followed by `::soap_recv_empty_response` to receive the HTTP acknowledgment.
At the receiver side use `soap_recv_ns__addResponse`.  To develop a server,
simply implement `soap_ns__addResponse` to handle the service operation and
in this function call `::soap_send_empty_response` to send the HTTP
acknowledgment.  The same applies to C++ proxy classes generated by soapcpp2.

@note Version 2.8.75 of gSOAP and greater generate send and receive
functions for each client-side call function.  This means that a client
application can simply call `soap_send_ns__add` to send the request and then
call `soap_recv_ns__add` to receive the response after polling the server
connection with `::soap_ready` to check if the server is ready (`::soap_ready`
returns `#SOAP_OK`) to send the response message as a reply message to be
received by the client.  Therefore, this option <b>`-b`</b> is not required to
implement asynchronous request-response messaging but rather adds one-way
asynchronous response messaging as well.

🔝 [Back to table of contents](#)

### wsdl2h -c -c++ -c++11 {#wsdl2h-c}

This option sets the source code output to C, C++, or C++11, respectively.

For C++ and C++11 you can also use [<b>`wsdl2h -s`</b> option <b>`-s`</b>](#wsdl2h-s)
to replace `std::vector` by arrays and replaces `std::string` by `char*`.  Use
a [<i>`typemap.dat`</i> file](#typemap) to specify further details for the
source code output generated by wsdl2h.

🔝 [Back to table of contents](#)

### wsdl2h -D {#wsdl2h-D}

This option makes attribute members of a struct or class with default or fixed
values optional with pointers.  Elements with default and fixed values are not
affected by this option.

Without this option, optional attributes with default or fixed values are
always output in XML, because the struct/class attribute member is not a
pointer.  This does not negatively affect the meaning of the XML produced,
because omitted attributes are replaced by their default or fixed value.

For example:

<div class="alt">
~~~{.xml}
    <xsd:schema targetNamespace="urn:example" xmlns:xsd="http://www.w3.org/2001/XMLSchema">
      <xsd:complexType name="data">
        <xsd:sequence>
          <xsd:element name="foo" type="xsd:string" minOccurs="0" default="abc"/>
        </xsd:sequence>
        <xsd:attribute name="bar" type="xsd:int" use="optional" default="123"/>
      </xsd:complexType>
    </xsd:schema>
~~~
</div>

By default without this option, this schema is translated by wsdl2h to the
following interface header file declaration:

~~~{.cpp}
    class ns__data
    { public:
        std::string* foo 0 = "abc"; // optional element with default value "abc"
      @ int bar 0 = 123;            // optional with default value 123
    };
~~~

The deserializer populates the attribute value with the default or fixed
value when the attribute is omitted from XML.  The element is populated
when the element is empty, i.e. <i>`<bar/>`</i> or <i>`<bar></bar>`</i>,
but not when it is omitted, as per the W3C XML Schema standards.

This option forces the optional attributes to be pointer-based
members, meaning that their output can be turned on or off by setting the
pointer to a value or to NULL:

~~~{.cpp}
    class ns__data
    { public:
        std::string* foo 0 = "abc"; // optional element with default value "abc"
      @ int* bar 0 = 123;           // optional with default value 123
    };
~~~

@warning In this case the deserializer will not populate the attribute value
with the default or fixed value when the attribute is omitted from XML, which
differs from the W3C XML schema standards.

🔝 [Back to table of contents](#)

### wsdl2h -d {#wsdl2h-d}

This option replaces literal XML strings `_XML` (a `char*` string with XML
content) with DOM nodes that are used to store the content of
<i>`xsd:any`</i>, <i>`xsd:anyAttribute`</i> <i>`xsd:anyType`</i>, and mixed
content values.  The [DOM API](#dom) offers more features to manipulate XML
content compared to the literal `_XML` string type.

The DOM node type `xsd__anyType` of the gSOAP [DOM API](#dom) is imported in the
wsdl2h-generated interface header file with `#import "dom.h"` where
<i>`dom.h`</i> is located in the <i>`gsoap/import`</i> directory.  This
requires compiling <i>`gsoap/dom.c`</i> in C and <i>`gsoap/dom.cpp`</i> in C++.

For example:

<div class="alt">
~~~{.xml}
    <xsd:schema targetNamespace="urn:example" xmlns:xsd="http://www.w3.org/2001/XMLSchema">
      <xsd:complexType name="data">
        <xsd:sequence>
          <xsd:element name="foo" type="xsd:anyType"/>
          <xsd:any/>
        </xsd:sequence>
        <xsd:anyAttribute processContents="lax"/>
      </xsd:complexType>
    </xsd:schema>
~~~
</div>

By default without this option, this schema is translated by wsdl2h to the
following interface header file declarations:

~~~{.cpp}
    class xsd__anyType
    { public:
        _XML __item; // XML string content
    };
    class ns__data : public xsd__anyType
    { public:
        xsd__anyType* foo;
        _XML __any;	     // Store any element content in XML string
      @ _XML __anyAttribute; // A placeholder that has no effect
    };
~~~

The `xsd__anyType` type has `_XML` simpleContent stored in `__item`.
Names starting with double underscores have no representation in XML
as elements or attribute names, meaning that only their values matter.
Therefore, `_XML __any` holds the element and its content in a string.

With <b>`wsdl2h -d`</b> option <b>`-d`</b> we obtain:

~~~{.cpp}
    #import "dom.h" // imports xsd__anyType as a DOM node

    class ns__data : public xsd__anyType
    { public:
        xsd__anyType* foo;                // Store <foo> element in DOM soap_dom_element
        xsd__anyType __any;	          // Store any element content in DOM soap_dom_element
      @ xsd__anyAttribute __anyAttribute; // Store anyAttribute content in DOM soap_dom_attribute linked node structure
    };
~~~

See [DOM API](#dom) for details on how to use the `xsd__anyType` and
`xsd__anyAttribute`.

🔝 [Back to table of contents](#)

### wsdl2h -e {#wsdl2h-e}

This option removes the prefix qualifier from enumeration names.

Without this option all enumeration names are prefixed by their `enum` name to
ensure that enumeration names do not clash with other constants and enumeration
names.

For example:

<div class="alt">
~~~{.xml}
    <xsd:schema targetNamespace="urn:example" xmlns:xsd="http://www.w3.org/2001/XMLSchema">
      <xsd:simpleType name="engine">
        <xsd:restriction base="xsd:string">
          <xsd:enumeration value="ON"/>
          <xsd:enumeration value="OFF"/>
        </xsd:restriction>
      </xsd:simpleType>
      <xsd:simpleType name="light">
        <xsd:restriction base="xsd:string">
          <xsd:enumeration value="ON"/>
          <xsd:enumeration value="OFF"/>
        </xsd:restriction>
      </xsd:simpleType>
    </xsd:schema>
~~~
</div>

By default without this option, this schema is translated by wsdl2h to the
following interface header file declarations:

~~~{.cpp}
    enum ns__engine { ns__engine__ON, ns__engine__OFF };
    enum ns__light { ns__light__ON, ns__light__OFF, ns__light__BROKEN };
~~~

By contrast, with <b>`wsdl2h -e`</b> option <b>`-e`</b> we obtain:

~~~{.cpp}
    enum ns__engine { ON, OFF };
    enum ns__light { ON_, OFF_, BROKEN };
~~~

Where enumeration names are suffixed with underscores to make them unique.

Note that C++11 scoped enumerations can be used with
[<b>`wsdl2h -c++11`</b> option <b>`-c++11`</b>](#wsdl2h-c), which makes option
<b>`-e`</b> useless.

🔝 [Back to table of contents](#)

### wsdl2h -F {#wsdl2h-F}

This option produces interface header files with struct/class
declarations that simulate inheritance using transient pointer members to
derived types.  This option is particularly useful for C source code generation
when derived types are required by the application.  Derived type values are
indicated by <i>`xsi:type`</i> attributes in XML with the derived type name.

This option can also be used for C++ to replace class inheritance by simulated
inheritance using transient pointer members in base classes that point to the
value of a derived type, meaning that the base class instance is replaced by
the derived class instance.  This option also removes pointers from array and
container item types.  These pointers are normally added to ensure containers
can contain derived type values, but pointers are no longer needed by the
simulated approach that add pointer members to the base classes.

For example:

<div class="alt">
~~~{.xml}
    <xsd:schema targetNamespace="urn:example" xmlns:tns="urn:example" xmlns:xsd="http://www.w3.org/2001/XMLSchema">
      <xsd:complexType name="base">
        <xsd:sequence>
          <xsd:element name="value" type="xsd:int"/>
        </xsd:sequence>
      </xsd:complexType>
      <xsd:complexType name="derived1">
        <xsd:complexContent>
          <xsd:extension base="tns:base">
            <xsd:sequence>
              <xsd:element name="name" type="xsd:string"/>
            </xsd:sequence>
          </xsd:extension>
        </xsd:complexContent>
      </xsd:complexType>
      <xsd:complexType name="derived2">
        <xsd:complexContent>
          <xsd:extension base="tns:base">
            <xsd:sequence>
              <xsd:element name="x" type="xsd:float"/>
            </xsd:sequence>
          </xsd:extension>
        </xsd:complexContent>
      </xsd:complexType>
      <xsd:complexType name="derived3">
        <xsd:complexContent>
          <xsd:extension base="tns:derived1">
            <xsd:sequence>
              <xsd:element name="x" type="xsd:float"/>
            </xsd:sequence>
          </xsd:extension>
        </xsd:complexContent>
      </xsd:complexType>
    </xsd:schema>
~~~
</div>

By default without this option, this schema is translated by <b>`wsdl2h -c`</b>
option <b>`-c`</b> to the following interface header file declaration in C that
lacks inheritance:

~~~{.cpp}
    struct ns__base
    {
        int value;
    };
    struct ns__derived1
    {
        int value;  // base type value of ns__base
        char *name; // extension
    };
    struct ns__derived2
    {
        int value;  // base type value of ns__base
        float x;    // extension
    };
    struct ns__derived3
    {
        int value;  // derived1 type value of ns__base
        char *name; // derived1 type
        float x;    // extension
    };
~~~

By contrast, with <b>`wsdl2h -c -F`</b> option <b>`-F`</b> we obtain an interface
header file with simulated inheritance using transient pointer members of base
types pointing to derived types:

~~~{.cpp}
    struct ns__base
    {
        [ struct ns__derived1 *ns__derived1; ] // points to derived type
        [ struct ns__derived2 *ns__derived2; ] // points to derived type
        int value;
    };
    struct ns__derived1
    {
        int value;                             // base type value of ns__base
        char *name;                            // extension
        [ struct ns__derived3 *ns__derived3; ] // points to derived type
    };
    struct ns__derived2
    {
        int value;                             // base type value of ns__base
        float x;                               // extension
    };
    struct ns__derived3
    {
        int value;                             // derived1 type value of ns__base
        char *name;                            // derived1 type
        float x;                               // extension
    };
~~~

Each transient pointer member name that is used to point to a derived type must
match the type name as shown, but trailing underscores are allowed in the
member name and type name, to prevent name clashes.

This latter form supports <i>`xsi:type`</i> attributes in XML with the derived
type name to replace base type values by derived type values at runtime by
setting one of the transient pointer members to non-NULL.  For example, assume
<i>`ns:data`</i> has a base type `ns__base` (i.e. declared as `struct ns__base
data`) then the following is legal and serializable:

<div class="alt">
~~~{.xml}
    <ns:data>
      <value>123</value>
    </ns:data>
~~~
</div>

This is serialized XML for `data.value` with `data.ns__derived1` and
`data.ns__derived2` both set to NULL.

<div class="alt">
~~~{.xml}
    <ns:data xsi:type="ns:derived1">
      <value>123</value>
      <name>abc</name>
    </ns:data>
~~~
</div>

This is serialized XML for `data.ns__derived1->value` and
`data.ns__derived1->name` where `data.ns__derived1` is non-NULL, for example
allocated and set with `data.ns__derived1 = soap_new_ns__derived1(soap)`.

<div class="alt">
~~~{.xml}
    <ns:data xsi:type="ns:derived2">
      <value>123</value>
      <x>3.14</x>
    </ns:data>
~~~
</div>

This is serialized XML for `data.ns__derived2->value` and
`data.ns__derived2->x` where `data.ns__derived1` is NULL and
`data.ns__derived2` is non-NULL, for example allocated and set with
`data.ns__derived2 = soap_new_ns__derived2(soap)`.

<div class="alt">
~~~{.xml}
    <ns:data xsi:type="ns:derived3">
      <value>123</value>
      <name>abc</name>
      <x>3.14</x>
    </ns:data>
~~~
</div>

This is serialized XML for `data.ns__derived1->ns__derived3->value`,
`data.ns__derived1->ns__derived3->name`, and
`data.ns__derived1->ns__derived3->x` where `data.ns__derived1` and
`data.ns__derived1->ns__derived3` are non-NULL.

Note that C++ class inheritance achieves the same results for base and derived
types, but without the use of transient pointer members.  However, this requires
container values to be pointers to support type derivation (class members are
already pointers), as generated by wsdl2h for C++.

🔝 [Back to table of contents](#)

### wsdl2h -f {#wsdl2h-f}

This option removes C++ class inheritance to produce a flat C++ class hierarchy
similar to structs in C as generated by wsdl2h.

As a side effect, derived type values can no longer be serialized in place of
base type values, see also [<b>`wsdl2h -F`</b> option <b>`-F`</b>](#wsdl2h-F).

Basically this option removes support for <i>`xsi:type`</i> in XML that
indicates a derived type that is restricted or extended from its base type.

This option also removes pointers from array and container item types, because
there are no derived types that could extend the item value types.  These
pointers are normally added to ensure containers can contain derived type
values in addition to the base type values.

For example:

<div class="alt">
~~~{.xml}
    <xsd:schema targetNamespace="urn:example" xmlns:tns="urn:example" xmlns:xsd="http://www.w3.org/2001/XMLSchema">
      <xsd:complexType name="base">
        <xsd:sequence>
          <xsd:element name="value" type="xsd:int"/>
        </xsd:sequence>
      </xsd:complexType>
      <xsd:complexType name="derived1">
        <xsd:complexContent>
          <xsd:extension base="tns:base">
            <xsd:sequence>
              <xsd:element name="name" type="xsd:string"/>
            </xsd:sequence>
          </xsd:extension>
        </xsd:complexContent>
      </xsd:complexType>
      <xsd:complexType name="derived2">
        <xsd:complexContent>
          <xsd:extension base="tns:base">
            <xsd:sequence>
              <xsd:element name="x" type="xsd:float"/>
            </xsd:sequence>
          </xsd:extension>
        </xsd:complexContent>
      </xsd:complexType>
      <xsd:complexType name="derived3">
        <xsd:complexContent>
          <xsd:extension base="tns:derived1">
            <xsd:sequence>
              <xsd:element name="x" type="xsd:float"/>
            </xsd:sequence>
          </xsd:extension>
        </xsd:complexContent>
      </xsd:complexType>
    </xsd:schema>
~~~
</div>

By default without this option, this schema is translated to the following
interface header file declaration in C++ with base and derived classes:

~~~{.cpp}
    class ns__base
    { public:
        int value;
    };
    class ns__derived1 : public ns__base
    { public:
        char *name; // extension
    };
    class ns__derived2 : public ns__base
    { public:
        float x;    // extension
    };
    class ns__derived3 : public ns__derived1
    { public:
        float x;    // extension
    };
~~~

By contrast, with <b>`wsdl2h -f`</b> option <b>`-f`</b> we obtain an interface
header file without inheritance but with classes that are extended with the
base class members:

~~~{.cpp}
    class ns__base
    { public:
        int value;
    };
    class ns__derived1
    { public:
        int value;  // base type value of ns__base
        char *name; // extension
    };
    class ns__derived2
    { public:
        int value;  // base type value of ns__base
        float x;    // extension
    };
    class ns__derived3
    { public:
        int value;  // derived1 type value of ns__base
        char *name; // derived1 type
        float x;    // extension
    };
~~~

This former form supports <i>`xsi:type`</i> attributes in XML with the derived
type name to replace base type values by derived type values at runtime.  But
this latter form does not support <i>`xsi:type`</i> attributes in XML and
only the base class can be serialized, for example `ns__base data`:

<div class="alt">
~~~{.xml}
    <ns:data>
      <value>123</value>
    </ns:data>
~~~
</div>

This is serialized XML for `data.value`.

<div class="alt">
~~~{.xml}
    <ns:data xsi:type="ns:derived1">
      <value>123</value>
      <name>abc</name>
    </ns:data>
~~~
</div>

This example can be deserialized when `#SOAP_XML_STRICT` is not enabled, but
only the `value` is retained in `data.value`.  However, when `#SOAP_XML_STRICT`
is enabled, deserialization fails due to the additional element <i>`name`</i>
that is rejected.

🔝 [Back to table of contents](#)

### wsdl2h -g {#wsdl2h-g}

This option adds global top-level element and attribute declarations to the
interface header file generated by wsdl2h.

For example:

<div class="alt">
~~~{.xml}
    <xsd:schema targetNamespace="urn:example" xmlns:tns="urn:example" xmlns:xsd="http://www.w3.org/2001/XMLSchema">
      <xsd:attribute name="type" type="xsd:QName"/>
      <xsd:element name="data" type="tns:record"/>
      <xsd:complexType name="record">
        <xsd:sequence>
          <xsd:element name="name" type="xsd:string"/>
          <xsd:element name="value" type="xsd:int"/>
        </xsd:sequence>
      </xsd:complexType>
    </xsd:schema>
~~~
</div>

By default without this option, this schema is translated to the following
C++ interface header file (the C interface header file is similar) that
declares the `ns__record` type for <i>`tns:record`</i> but does not declare
attribute <i>`tns:type`</i> and element <i>`tns:data`</i>:

~~~{.cpp}
    typedef std::string xsd__QName;
    class ns__record
    { public:
        std::string name;
        int value;
    };
~~~

By contrast, with <b>`wsdl2h -g`</b> option <b>`-g`</b> we obtain an interface
header file with the attribute and element declarations:

~~~{.cpp}
    typedef std::string xsd__QName;
    class ns__record
    { public:
        std::string name;
        int value;
    };
    typedef ns__record _ns__data; 
    typedef xsd__QName _ns__type;
~~~

This defines `_ns__type` and `_ns__data`, where the latter can be used as a
root element to serialize its content with the soapcpp2-generated readers and
writers:

~~~{.cpp}
    struct soap *soap = soap_new();
    _ns__data data;
    if (soap_read__ns__data(soap, &data))
      ... // error
    if (soap_write__ns__data(soap, &data))
      ... // error
~~~

which parses and re-writes the XML fragment:

<div class="alt">
~~~{.xml}
    <ns:data xmlns:ns="urn:example">
      <name>abc</name>
      <value>123</value>
    </ns:data>
~~~
</div>

Note that top-level element and attribute type names start with an underscore
to distinguish them from types.  This convention is also used by soapcpp2 to
generate schemas that define top-level attributes and elements.

Note that a schema may define a global top-level element with a local type, for
example:

<div class="alt">
~~~{.xml}
    <xsd:schema targetNamespace="urn:example" xmlns:tns="urn:example" xmlns:xsd="http://www.w3.org/2001/XMLSchema">
      <xsd:element name="record">
        <xsd:complexType>
          <xsd:sequence>
            <xsd:element name="name" type="xsd:string"/>
            <xsd:element name="value" type="xsd:int"/>
          </xsd:sequence>
        </xsd:complexType>
      </xsd:element>
    </xsd:schema>
~~~
</div>

This schema is translated to the following C++ interface header file (the C
interface header file is simular) that declares the `_ns__record` type and
element for the <i>`tns:record`</i> top-level element:

~~~{.cpp}
    class _ns__record
    { public:
        std::string name;
        int value;
    };
~~~

In this case option <b>`-g`</b> has no effect, because <i>`tns:record`</i> has
a local type that may be used elsewhere in the schema.

🔝 [Back to table of contents](#)

### wsdl2h -h {#wsdl2h-h}

This option displays help info and then exits.

🔝 [Back to table of contents](#)

### wsdl2h -I {#wsdl2h-I}

This option specifies a directory path to search for WSDL and XSD files.

For example:

    wsdl2h -I path file.wsdl

This searches <i>`path`</i> for <i>`.wsdl`</i> and <i>`.xsd`</i> files that are
imported by <i>`file.wsdl`</i> and by other imported files.

When a WSDL or XSD file imports another file then:

- a file name referenced by `http://` or by `https://` is retrieved from the
  specified URL.
- a file name referenced by `file://` is retrieved from the path specified
  relative to the directory in which <b>`wsdl2h`</b> is run and the <b>`-I`</b>
  option can be used to change that location to import from.
- a file name without a path (i.e. has no `/`) or a file name with path stating
  with `../` are considered files located at relative path locations with
  respect to the current WSDL and XSD that is importing this file
- otherwise, imported files are considered relative to the directory in which
  <b>`wsdl2h`</b> is run and the <b>`-I`</b> option can be used to change that
  location to import from.

WSDL and XSD files that import other WSDL and XSD files typically use relative
paths, at least that is recommended by best practices.  If absolute paths are
used then wsdl2h may fail to find the imported WSDLs and XSDs.  This option
resolves relative paths but does not help to resolve absolute paths.  In
the worst case one must edit the WSDLs and XSDs to refer to proper file
locations.

🔝 [Back to table of contents](#)

### wsdl2h -i {#wsdl2h-i}

This option skips over schema <i>`import`</i> and as a result none of the
imported schemas and their components are imported.

There are two reasons to use this option:

- when imported components are already declared in interface header files that
  are imported into the main interface header file with `#import`, and
- when imported schemas are explicitly provided with the wsdl2h command as
  command line arguments, which means that the specified schemas will be used
  instead of the imported schemas.  This may help to resolve issues when
  imported files are not found by wsdl2h.  The schema <i>`targetNamespace`</i>
  namespace names are relevant when schemas reference imported schemas by their
  namespace, not the schema file name.

🔝 [Back to table of contents](#)

### wsdl2h -j {#wsdl2h-j}

This option skips the generation of `::SOAP_ENV__Header` and `::SOAP_ENV__Detail`
structure definitions, assuming that these are manually replaced in the
generated interface header file for soapcpp2.

🔝 [Back to table of contents](#)

### wsdl2h -k {#wsdl2h-k}

This option skips the generation of `mustUnderstand` qualifiers for
`::SOAP_ENV__Header` members.  This removes the <i>`mustUnderstand="true"`</i>
XML attributes from SOAP Headers in SOAP messages.  As per SOAP standard,
SOAP Headers with <i>`mustUnderstand="true"`</i> must not be ignored by
receivers.

🔝 [Back to table of contents](#)

### wsdl2h -L {#wsdl2h-L}

This option generates less documentation by removing generic `@note` comments
from the interface header file output, thereby reducing the size of the output
without removing critical information.

🔝 [Back to table of contents](#)

### wsdl2h -l {#wsdl2h-l}

This option displays license information.

🔝 [Back to table of contents](#)

### wsdl2h -M {#wsdl2h-M}

This option suppresses the wsdl2h error message

    "must understand element with wsdl:required='true'"

This error indicates that a (special) WSDL construct was used that is marked
<i>`wsdl:required="true"`</i>, meaning that must not be ignored by the WSDL
processor (unless the developer knows what he or she is doing).

🔝 [Back to table of contents](#)

### wsdl2h -m {#wsdl2h-m}

This option tells wsdl2h to use <i>`xsd.h`</i> to define the primitive XSD
types instead of generating them in the interface header file for soapcpp2.
This option offers an alternative to the use of <i>`typemap.dat`</i> to
redefine primitive XSD types by defining them all together instead of on a
type-by-type basis.  The interface header file output by wsdl2h includes
`#import "xsd.h"`.

🔝 [Back to table of contents](#)

### wsdl2h -N {#wsdl2h-N}

This option specifies a name to be used as a service namespace prefix for each
WSDL binding.

By default without this option, the wsdl2h tool warns when it reads one or more
WSDLs that define multiple bindings:

    Warning: 3 service bindings found, but collected as one service (use option -Nname to produce a separate service for each binding)

This means that all 3 services will be collected under one name.  When proxy
and service classes are generated with <b>`soapcpp2 -i`</b> option <b>`-i`</b>
or with <b>`soapcpp2 -j`</b> option <b>`-j`</b> then the service operations are
collected into one proxy and service class.  Essentially only one namespace is
used.  This may lead to clashes when multiple bindings define the same Web
service operations (name clashes are resolved by wsdl2h by adding trailing
underscores).

By contrast, with <b>`wsdl2h -N name`</b> option <b>`-N name`</b> we obtain an interface
header file that uses the specified name as a prefix to define the service
bindings and service operations.

For example:

    wsdl2h -N foo file.wsdl

If <i>`file.wsdl`</i> has multiple bindings, then the Web service operations
associated with each binding are identified by their prefix `foo1`, `foo2`,
`foo3`, and so on.  As a result, we obtain more than one proxy and service
class generated by soapcpp2, one for each binding.

🔝 [Back to table of contents](#)

### wsdl2h -n {#wsdl2h-n}

This option changes the default `ns` namespace prefix to the specified prefix
name.

By default without this option, the XML namespace prefix is `ns` which results
in the generation of prefixes `ns1`, `ns2`, `ns3`, and so on.

For example:

    wsdl2h -n foo file.wsdl

This generates namespace prefixes `foo1`, `foo2`, `foo3`, and so on.

@warning It is strongly recommended to define namespace prefixes in the
[<i>`typemap.dat`</i> file](#typemap) to prevent future runs of wsdl2h to
produce namespace prefixes that are not in the same original order.  For
example when the order of WSDLs and XSDs changes or if new WSDLs and XSDs are
added.  Therefore, do not use this option unless the single WSDL processed by
wsdl2h is relatively simple and does not import WSDLs and XSDs.

🔝 [Back to table of contents](#)

### wsdl2h -O {#wsdl2h-O}

This option optimizes the generated interface header file:

- <b>`-O1`</b> removes duplicate choice/sequence members;
- <b>`-O2`</b> optimize with <b>`-O1`</b> and remove unused schema types (types
  that are unreachable from top-level schema element and attribute roots);
- <b>`-O3`</b> optimize with <b>`-O2`</b> and remove unused schema top-level
  root attributes;
- <b>`-O4`</b> optimize with <b>`-O3`</b> and remove unused schema top-level
  root elements, only retain the root elements used by WSDLs.  Use this
  option only when converting WSDLs (and their associated XSD schemas) to
  source code, not when solely converting XSD schemas to source code.

Option <b>`-O4`</b> is the most aggressive.  When used only for one or more
XSDs as input to wsdl2h, the output will be empty because removing the root
elements (and attributes) results in removing all types from the schema.
However, this option is safe to use with WSDLs to aggressively remove all
unused schema components that are unreachable from the Web service operation
parameter elements and types.  Option <b>`-O3`</b> is safe to use with one or
more XSDs as input to wsdl2h instead of WSDLs, for example when developing an
XML application that serializes data as XML root elements (<b>`wsdl2h -g`</b>
[option <b>`-g`</b>](#wsdl2h-g) is recommended in this case).

Optimization by schema slicing removes unused types, which are types that are
unreachable from top-level schema element and attribute roots.  A type is
marked as *used* when:

- it is explicitly used by one or more top-level elements and attributes;
- it is used as a type by a child element or attribute of a complexType that is
  marked as *used*;
- it is used as the base type of an extension or restriction of a simpleType or
  complexType that is marked as *used*.

Marking proceeds recursively until no more types can be marked.  All remaining
unused types are removed.  Top-level elements and attributes are selectively
marked unused and removed depending on the level of optimization applied, with
<b>`-O3`</b> removing unused top-level attributes and <b>`-O4`</b> removing
unused top-level elements except for all elements used in the specified WSDLs.

Aggressive optimization with options <b>`-O2`</b>, <b>`-O3`</b>, and
<b>`-O4`</b> removes derived type extensions of a base type when the derived
types are not marked as *used*.  However, in certain messaging scenarios this
may have the undesired effect that this limits the choice of derived types that
can be used to replace a base type in XML messages, because a derived type may
have been removed when it is not marked as *used* elsewhere in the WSDLs and
XSD schemas.  A derived type that replaces a base type in an XML message is
indicated by a <i>`xsi:type`</i> attribute with the QName value of the derived
type.  The wsdl2h tool generates a C++ class hierarchy to support type
derivation, so assigning a derived type value instead of a base type value to a
pointer member is automatically serialized in XML with the specified derived
value (which is indicated by <i>`xsi:type`</i> attribute in the XML message).
For C applications, we should use <b>`wsdl2h -c -F`</b>
[option <b>`-c`</b>](#wsdl2h-c) and [option <b>`-F`</b>](#wsdl2h-F) to simulate
inheritance in C.  In both cases it is recommended to use the following options
to retain all derived type extensions of a base type that is marked as *used*:

- <b>`-Ow2`</b> optimize with <b>`-O2`</b> to remove unused schema types, but
  retain types that are derived types of base types that are marked as used.  .
- <b>`-Ow3`</b> optimize with <b>`-O3`</b> to remove unused schema top-level
  root attributes, but retain types that are derived types of base types that
  are marked as used..
- <b>`-Ow4`</b> optimize with <b>`-O4`</b> to remove unused schema top-level
  root elements, but retain types that are derived types of base types that are
  marked as used.

This permits a base type value (typically a struct or class member that is a
pointer to a base type) to be assigned a derived type in C++, which is
serialized in XML with a <i>`xsi:type`</i> attribute to indicate the type of
the derived value.  Likewise, XML data with derived type values are
deserialized to C/C++ data automatically.  Inheritance is simulated in C,
see [option <b>`-F`</b>](#wsdl2h-F).

🔝 [Back to table of contents](#)

### wsdl2h -o {#wsdl2h-o}

This option specifies a file name for the wsdl2h interface header file output.

By default without this option, the wsdl2h tool writes the interface header
file to the file named after the first file name input at the command line, but
using <i>`.h`</i> as the file name extension.

When the input to the wsdl2h tool consists of URLs, the wsdl2h tool writes its
output to standard output (usually the screen).  Use this option to specify
a file instead.

For example:

    wsdl2h calc.wsdl

This saves <i>`calc.h`</i> because the first file specified on the command line
is <i>`calc.wsdl`</i>.

Option <b>`-o`</b> should be used when a URL is specified on the command line:

    wsdl2h -o calc.h http://www.genivia.com/calc.wsdl

This saves the interface header file <i>`calc.h`</i>.

🔝 [Back to table of contents](#)

### wsdl2h -P {#wsdl2h-P}

This option disables the generation of types inherited from the `xsd__anyType`
base type.

This option has effect only when [<b>`wsdl2h -p`</b> option <b>`-p`</b>](#wsld2h-p)
is used or when the wsdl2h tool detects that <i>`xsd:anyType`</i> is used
(thereby implicitly and automatically enabling option <b>`-p`</b>), which means
that `xsd__anyType` should be a base type for all possible types defined in the
schemas.

🔝 [Back to table of contents](#)

### wsdl2h -p {#wsdl2h-p}

This option makes all types inherit `xsd__anyType` to support full polymorphism.

This option is automatically enabled when the wsld2h tool detects that
<i>`xsd:anyType`</i> is used, which means that
`xsd__anyType` should be a base type for all possible types defined in the
schemas.  To disable, use [<b>`wsdl2h -P`</b> option <b>`-P`</b>](#wsld2h-P).

For example:

<div class="alt">
~~~{.xml}
    <xsd:schema targetNamespace="urn:example" xmlns:xsd="http://www.w3.org/2001/XMLSchema">
      <xsd:element name="data" type="data"/>
      <xsd:complexType name="data">
        <xsd:sequence>
          <xsd:element name="value" type="xsd:string"/>
          <xsd:element name="item" type="xsd:anyType" minOccurs="0" maxOccurs="unbounded"/>
        </xsd:sequence>
      </xsd:complexType>
    </xsd:schema>
~~~
</div>

This schema is translated to the following C++ interface header file that
declares the `xsd__anyType` type with `_XML` simpleContent (meaning that
`__item` contains element content as per gSOAP convention) and the `ns__data`
class:

~~~{.cpp}
    class xsd__anyType
    { public:
        _XML __item;
    };
    class xsd__string_ : public xsd__anyType
    { public:
        std::string __item;
    };
    class ns__data : public xsd__anyType
    { public:
        std::string value;
        std::vector<xsd__anyType*> item;
    };
~~~

The `xsd__anyType` pointer values of the items of the vector can be assigned
derived class instances to serialize any type of value declared in the
interface header file, including the `xsd__string_` wrapper class with
simpleContent and the `ns__data` class with complexContent.

This schema is translated to the following C interface header file with
<b>`wsdl2h -c -F`</b> [option <b>`-c`</b>](#wsdl2h-c) and
[option <b>`-F`</b>](#wsdl2h-F) to simulate inheritance in C:

~~~{.cpp}
    struct xsd__string_
    {
        char* __item;
    };
    struct xsd__anyType_
    {
        _XML __item;
      [ struct xsd__string_ *xsd__string; ]
      [ struct ns__data *ns__data; ]
    };
    struct ns__data
    {
        char* value;
      $ int __sizeitem;
        struct xsd__anyType_* item;
    };
~~~

The `xsd__anyType_` values of items of the dynamic array (`item` points to an
array of size `__sizeitem` which is a special member to indicate dynamic
arrays) can be assigned base `xsd__anyType_` and derived types, see
[<b>`wsdlh2 -F`</b> option <b>`-F`</b>](#wsdl2h-F).

🔝 [Back to table of contents](#)

### wsdl2h -Q {#wsdl2h-Q}

This option makes `xsd__anySimpleType` equal to `xsd__anyType` to use as the
base type for derivation.  This option is more effective when used with
[<b>`wsdl2h -p`</b> option <b>`-p`</b>](#wsdl2h-p) for C++ applications and
[<b>`wsdl2h -F`</b> option <b>`-F`</b>](#wsdl2h-F) for C applications.
This option can also be used with [<b>`wsdl2h -d`</b> option <b>`-d`</b>](#wsdl2h-d)
to make `xsd__anySimpleType` equal to a DOM node.

Without option <b>`-Q`</b>, the `xsd__anySimpleType` type is just a C/C++
string generated by wsdl2h:

~~~{.cpp}
    typedef char* xsd__anySimpleType;        // in case of C

    typedef std::string xsd__anySimpleType;  // in case of C++
~~~

The reason for this choice is that some WSDLs and XSD schemas use
<i>`xsd:anySimpleType`</i> to declare XML attributes of any type (because XML
attributes must be simple types <i>`xsd:anyType`</i> is invalid to use for
attributes).  The values of XML attributes of type <i>`xsd:anySimpleType`</i>
can be any character data essentially.  There is no mechanism to indicate the
actual type of the attribute value used, unlike elements that are annotated
with <i>`xsi:type`</i> attribute with the derived type as its QName value.
Therefore, by considering `xsd__anySimpleType` just strings we can provide any
value for XML attributes of type <i>`xsd:anySimpleType`</i>.

However, there are other uses of `xsd__anySimpleType` in XSD schemas, where
essentially `xsd__anySimpleType` serves the same purpose as `xsd__anyType` to
provide a base type for derived types, but restricts the derived types to
simple types.

Unfortunately, these two cases clash: we want to use C/C++ strings for XML
attributes of type <i>`xsd:anySimpleType`</i> and also use
<i>`xsd:anySimpleType`</i> as a base class for derived types.

Option <b>`-Q`</b> enables the latter case by making `xsd__anySimpleType` equal
to `xsd__anyType` so that elements of type <i>`xsd:anySimpleType`</i> can be
serialized with a derived type, using inheritance in C++ and by using simulated
inheritance in C using [<b>`wsdl2h -F`</b> option <b>`-F`</b>](#wsdl2h-F).
  
For example, option <b>`-Q`</b> changes this generated code for C++
applications:

~~~{.cpp}
    class xsd__anyType
    { public:
        _XML __item;
        struct soap *soap;
    };

    typedef std::string xsd__anySimpleType;

    class ns__record : public xsd__anyType
    { public:
        xsd__anySimpleType value;  // non-polymorphic xsd:anySimpleType value
    }
~~~

into:

~~~{.cpp}
    class xsd__anyType
    { public:
        _XML __item;
        struct soap *soap;
    };

    class ns__record : public xsd__anyType
    { public:
        xsd__anyType *value;  // polymorphic xsd:anySimpleType value
    }
~~~

where all other classes generated by wsdl2h [option <b>`-p`</b>](#wsdl2h-p) are
derived from `xsd__anyType`, meaning that `value` can be assigned any one of
these classes as long as the class is a simple type wrapper (wsdl2h generates
comments to indicate that the polymorhpic value should be a
<i>`xsd:anySimpleType`</i>).

Similar code is generated by wsdl2h [option <b>`-F`</b>](#wsdl2h-F) for C
applications.

On the other hand this option invalidates XML attributes of type
<i>`xsd:anySimpleType`</i>.  The soapcpp2 tool warns about this invalid
attribute type as a result.

🔝 [Back to table of contents](#)

### wsdl2h -q {#wsdl2h-q}

This option specifies a C++ namespace name.  The interface header file
declarations are placed in the given C++ namespace.

For example:

    wsdl2h -q api file.wsdl

The generated interface header file for soapcpp2 places all declarations in the
`api` C++ namespace:

~~~{.cpp}
    namespace api {
      ...
    }
~~~

@warning It is more difficult to use SOAP Headers and SOAP Faults when C++
namespaces are used.  When wsdl2h finds SOAP Headers and SOAP Fault Details it
collects these into `::SOAP_ENV__Header` and `::SOAP_ENV__Detail` structures,
which become part of the given C++ namespace.  However, to use the
`::SOAP_ENV__Header` and `::SOAP_ENV__Detail` structures these should be
declared at the global scope.  This option places these structures
with the types used by their members in the given C++ namespace, making them
unavailable to the global scope.

See \ref codenamespace for details on using C++ namespaces to build client and
server applications, which requires a <i>`env.h`</i> file with SOAP Header and
Fault definitions to be compiled with:

     soapcpp2 -penv env.h

The generated <i>`envC.cpp`</i> file holds the SOAP Header and Fault serializers and you can
link this file with your client and server applications.

This option has no effect for C source code output.

🔝 [Back to table of contents](#)

### wsdl2h -R {#wsdl2h-R}

This option enables the generation of REST service operations in the interface header
file saved by wsdl2h for soapcpp2.

By default without this option, REST service operations defined in one or more
WSDLs are ignored.

With this option, both REST and SOAP service operations are declared in the
interface header file.

🔝 [Back to table of contents](#)

### wsdl2h -r {#wsdl2h-r}

This option specifies a proxy host name and port number with proxy credentials
to connect to web sites through a proxy server.

This option can also be used to specify credentials to access a web site that
requires authentication (HTTP basic or digest authentication).

For example:

    wsdl2h -r proxy.example.org:80:proxyuserid:proxypasswd -r userid:passwd

🔝 [Back to table of contents](#)

### wsdl2h -S {#wsdl2h-S}

This option renames the `soap` members of the generated C++ classes in the
interface header file for soapcpp2.

By default without this option, wsdl2h adds `struct soap *soap` members to
classes and structs.  This member points to the `::soap` context that manages
the instance, when the instance was allocated by the gSOAP engine.

To remove the `struct soap *soap` members use this option with an empty name:

    wsdl2h -S '' file.wsdl

To rename the `struct soap *soap` members, specify a name for the member, for
example `ctx`:

    wsdl2h -S ctx file.wsdl

This option has no effect for C source code output.

🔝 [Back to table of contents](#)

### wsdl2h -s {#wsdl2h-s}

This option replaces STL `std::vector` and `std::string` by C-like equivalents
and is intended for systems with limited support for C++ libraries.

The `std::vector` struct/class member is replaced by a dynamic array, declared
with a `__size` member followed by a pointer member to the array items.

The `std::string` is replaced by `char*`.

This option has no effect for C source code output.

🔝 [Back to table of contents](#)

### wsdl2h -t {#wsdl2h-t}

This option specifies an alternate file or path for <i>`typemap.dat`</i>.
See [<i>`typemap.dat`</i> file](#typemap).

For example:

    wsdl2h -t $GSOAP/gsoap/typemap.dat file.wsdl

🔝 [Back to table of contents](#)

### wsdl2h -U {#wsdl2h-U}

This option allows UTF-8-encoded Unicode C/C++ identifier names in the
generated interface header file for soapcpp2.  This assumes that the C/C++
compiler that is used to compile a gSOAP client or server application supports
Unicode identifier names.

By default without this option, Unicode XML names in WSDLs and XSDs are
preserved using the gSOAP convention for UCS-2 characters in identifier names
with `_xHHHH` where `HHHH` is a hexadecimal Unicode character code point.

With this option, Unicode XML names in WSDLs and XSDs are preserved "as is" in
C/C++ identifier names.

🔝 [Back to table of contents](#)

### wsdl2h -u {#wsdl2h-u}

This option replaces unions with structs/classes in the generated interface
header file for soapcpp2.  Union members are used to represent
<i>`xsd:choice`</i> of elements.  A choice of elements can also be represented
by pointer members of a struct/class such that only one member is non-NULL.
However, when using a struct/class instead of a union, the deserialization
validator will not reject additional elements when present.

For example:

<div class="alt">
~~~{.xml}
    <xsd:schema targetNamespace="urn:example" xmlns:xsd="http://www.w3.org/2001/XMLSchema">
      <xsd:element name="data" type="data"/>
      <xsd:complexType name="data">
        <xsd:sequence>
          <xsd:element name="value" type="xsd:string"/>
          <xsd:choice>
            <xsd:element name="string" type="xsd:string"/>
            <xsd:element name="number" type="xsd:float"/>
          </xsd:choice>
        </xsd:sequence>
      </xsd:complexType>
    </xsd:schema>
~~~
</div>

By default without this option, wsdl2h generates a tagged `union` for the
<i>`xsd:choice`</i>, where the tag is a special member `int __union_data`
that is set to `SOAP_UNION__ns__union_data_string` when the `string` union
member is valid or `SOAP_UNION__ns__union_data_number` when the `number` union
member is valid:

~~~{.cpp}
    class ns__data
    { public:
        std::string value;
      $ int __union_data;
        union _ns__union_data
        {
            std::string* string;
            float number;
        } union_data;
    };
~~~

With this option, wsdl2h removes the `union` and replaces it with pointer
members to produce a simpler structure:

~~~{.cpp}
    class ns__data
    { public:
        std::string value;
    //  BEGIN CHOICE <xs:choice>
        std::string* string; // Choice of element (one of multiple choices)
        float* number;       // Choice of element (one of multiple choices)
    //  END OF CHOICE
    };
~~~

@warning This option removes the uniqueness check on choices from the
deserialization validator.  When serializing data, only one of the choice
pointer members should be non-NULL.

🔝 [Back to table of contents](#)

### wsdl2h -V {#wsdl2h-V}

This option displays the current wsdl2h tool version and then exits.

🔝 [Back to table of contents](#)

### wsdl2h -v {#wsdl2h-v}

This option enables verbose output to assist in debugging the wsdl2h tool.

🔝 [Back to table of contents](#)

### wsdl2h -W {#wsdl2h-W}

This option suppresses all warnings produced by wsdl2h.  Errors are not
suppressed.

🔝 [Back to table of contents](#)

### wsdl2h -w {#wsdl2h-w}

This option wraps response parameters in a response struct.

The last parameter of a service operation declared as a function in the
interface header file is the response parameter.  When multiple response
parameters are returned by the service operation or if the response parameter
is a complexType (a struct or class), then the parameters should be wrapped in
a special "response struct".  However, if a single response parameter is a
primitive type value then this parameter does not need to be wrapped in a
response struct.

This option consistently wraps response parameters in a response struct, even
when a single response parameter is a primitive type value.

🔝 [Back to table of contents](#)

### wsdl2h -X {#wsdl2h-X}

Document/literal wrapped patterns may cause ambiguities with respect to message
namespace qualification.  A <i>`part`</i> name associated with a <i>`type`</i> is
implicitly qualified by the targetNamespace of the WSDL but may also be
associated with the namespace of the type.  By default, the wsdl2h tool uses
the namespace of the type when the type is not a primitive XSD type, otherwise
the WSDL targetNamespace is used.

As an example of a document/literal wrapped pattern message, consider:

<div class="alt">
~~~{.xml}
    <wsdl:types>
      <xs:schema ...>
        <xs:complexType name="Record">
          ...
    </wsdl:types>
    ...
    <wsdl:message name="Message">
      <wsdl:part name="Name" type="xs:string"/>
      <wsdl:part name="Info" type="tns:Record"/>
    </wsdl:message>
    ...
    <wsdl:operation name="Operation">
      <wsdl:input message="tns:Message"/>
      <wsdl:output message="tns:MessageResponse"/>
    </wsdl:operation>
    ...
    <wsdl:binding name="Binding" type="tns:PortType">
      <soap:binding style="document" transport="http://schemas.xmlsoap.org/soap/http"/>
      <wsdl:operation name="Operation">
        <soap:operation soapAction="Action"/>
        <wsdl:input>
          <soap:body use="literal"/>
        </wsdl:input>
        <wsdl:output>
          <soap:body use="literal"/>
        </wsdl:output>
      </wsdl:operation>
      ...
~~~
</div>

Note that <i>`message name="Message"`</i> has two parts with both a type, which
makes these part namespaces amiguous.  The generated interface header file
declares a wrapper for the <i>`Name`</i> request message and the <i>`Info`</i> response
message:

~~~{.cpp}
    int __ns1__Operation(
        std::string Name,
        ns1__Record ns2__Info
        );
~~~

Here, <i>`Name`</i> belongs to the <i>`ns1`</i> namespace, i.e. by the <i>`__ns1__Operation`</i>,
whereas <i>`Info`</i> belongs to the `ns2` namespace.  The <i>`__ns1__Operation`</i> is just
a wrapper for the operation and is not visible in XML.  Only <i>`Name`</i> and <i>`Info`</i>
are serialized in XML as the request and response message, respectively.

With option <b>`-X`</b> the <i>`ns2`</i> qualifier is removed:

~~~{.cpp}
    int __ns1__Operation(
        std::string Name,
        ns1__Record Info
        );
~~~

Now both <i>`Name`</i> and <i>`Info`</i> belong to the <i>`ns1`</i> namespace,
i.e. by the <i>`__ns1__Operation`</i>.

However, best practices for document/literal messaging recommend to avoid this wrapped
pattern construct in favor of using elements defined in schemas:

<div class="alt">
~~~{.xml}
    <wsdl:types>
      <xs:schema ...>
        <xs:element name="Name" type="xs:string"/>
        <xs:element name="Info" type="tns:Record"/>
        <xs:complexType name="Record">
          ...
        ...
    </wsdl:types>
    ...
    <wsdl:message name="Message">
      <wsdl:part name="Name" element="tns:Name"/>
      <wsdl:part name="Info" element="tns:Record"/>
    </wsdl:message>
    ...
    <wsdl:operation name="Operation">
      <wsdl:input message="tns:Message"/>
      <wsdl:output message="tns:MessageResponse"/>
    </wsdl:operation>
    ...
    <wsdl:binding name="netfileOverSoap" type="tns:netfileOverSoap">
      <soap:binding style="document" transport="http://schemas.xmlsoap.org/soap/http"/>
      <wsdl:operation name="Operation">
        <soap:operation soapAction="Action"/>
        <wsdl:input>
          <soap:body use="literal"/>
        </wsdl:input>
        <wsdl:output>
          <soap:body use="literal"/>
        </wsdl:output>
      </wsdl:operation>
      ...
~~~
</div>

The elements <i>`Name`</i> and <i>`Record`</i> are the actual message names, qualified by the
schema's targetNamespace:

~~~{.cpp}
    int __ns1__Operation(
        std::string ns2__Name,
        ns1__Record ns2__Info
        );
~~~

See also wsdl2h [option <b>`-z7`</b>](#wsdl2h-z).

🔝 [Back to table of contents](#)

### wsdl2h -x {#wsdl2h-x}

This option removes `_XML` type members of structs and classes that are
generated for <i>`xsd:any`</i> and <i>`xsd:anyAttribute`</i> components.

There are two options to represent <i>`xsd:any`</i> and
<i>`xsd:anyAttribute`</i> components: the literal `_XML` string type with XML
content (a `char*` string) or a DOM node.  DOM nodes are generated for
<i>`xsd:any`</i> and <i>`xsd:anyAttribute`</i> components with
[<b>`wsdl2h -d`</b> option <b>`-d`</b>](#wsdl2h-d), which also defines
<i>`xsd:anyType`</i> as the DOM node `xsd__anyType` in C and C++.

🔝 [Back to table of contents](#)

### wsdl2h -y {#wsdl2h-y}

This option adds typedef synonyms for structs and enums to the interface
header file, which is useful for C source code.  A typedef synonym for a struct
is declared by `typedef struct name name;` and for an enum is declared by
`typedef enum name name;`.

🔝 [Back to table of contents](#)

### wsdl2h -z {#wsdl2h-z}

These options are for backward compatiility with older gSOAP releases:

- <b>`-z1`</b> compatibility with 2.7.6e: generate pointer-based arrays
- <b>`-z2`</b> compatibility with 2.7.15: (un)qualify element/attribute referenced members
- <b>`-z3`</b> compatibility with 2.7.16 to 2.8.7: (un)qualify element/attribute referenced members
- <b>`-z4`</b> compatibility up to 2.8.11: don't generate union structs in std::vector 
- <b>`-z5`</b> compatibility up to 2.8.15: don't include minor improvements
- <b>`-z6`</b> compatibility up to 2.8.17: don't include minor improvements
- <b>`-z7`</b> compatibility up to 2.8.59: don't generate `std::vector` of class of union
- <b>`-z8`</b> compatibility up to 2.8.74: don't generate qualifiers for doc/lit wrapped patterns
- <b>`-z9`</b> compatibility up to 2.8.93: always qualify element/attribute referenced members, even when defined in the same namespace with default forms unqualified
- <b>`-z10`</b> compatibility up to 2.8.96: generate qualifiers even when defined without namespace

🔝 [Back to table of contents](#)

### wsdl2h -_ {#wsdl2h-_}

This option replaces `_USCORE` by the Unicode `_x005f` character code point in
identifier names in C and C++ in the generated interface header file.

🔝 [Back to table of contents](#)

## Customizing XML data bindings with the typemap.dat file        {#typemap}

The <i>`typemap.dat`</i> file for the wsdl2h tool can be used to customize or optimize
the type bindings by mapping schema types to C/C++ types.  This file contains custom
XML schema to C/C++ type bindings and XML namespace bindings for namespace prefixes
to be generated by the wsdl2h tool.  You can edit this file to enable features
such as custom serializers for schema types, C++11 smart pointers to replace
regular pointers, bind XML namespace prefixes to XML namespace URIs, and
specify bindings for schema types.

Here is a simple example of a <i>`typemap.dat`</i> file:

    #       This file contains custom definitions of the XML Schema types and 
    #       C/C++ types for your project, and XML namespace prefix definitions. 
    #       The wsdl2h WSDL importer consults this file to determine bindings. 
    [ 
    // This comment will be included in the generated .h file 
    // You can include any additional declarations, includes, imports, etc. 
    // within [ ] sections. The brackets must appear at the start of a line 
    ] 
    #       XML namespace prefix definitions can be provided to override the 
    #       default choice of ns1, ns2, ... prefixes. For example: 
    i = "http://www.soapinterop.org/" 
    s = "http://www.soapinterop.org/xsd"

The `i` and `s` prefixes are declared such that the header file output by wsdl2h uses these prefixes instead of the default `ns1`, `ns2`, etc.  It is strongly recommended to name the prefixes in this way, because future runs of wsdl2h may result in a different assignment of the default `ns1`, `ns2`, ... prefixes.  Therefore, it is recommended that application code should not rely on the default prefixes.

Type bindings can be provided to bind XML schema types to C/C++
types for your project.
These type bindings have four parts:

    prefix__type = declaration | use | ptr-use

where `prefix__type` is the C/C++ type name of the schema type (using gSOAP's
    type naming conventions), the `declaration` part declares the C/C++ type in
the generated header file which may be empty to omit, the optional `use` part
specifies how the type is used by other types such as by member declarations
and as function parameters, and the optional `ptr-use` part specifies how the
type is used as a pointer type by other types and as function parameters.

    #       Example XML Schema and C/C++ type bindings: 
    xsd__int          = | int 
    xsd__string       = | char* | char* 
    xsd__boolean      = enum xsd__boolean { false_, true_ }; | enum xsd__boolean 
    xsd__base64Binary = class xsd__base64Binary { unsigned char *__ptr; int __size; }; | xsd__base64Binary | xsd__base64Binary
    #       You can extend structs and classes with member data and functions. 
    #       For example, adding a constructor to ns__myClass:
    ns__myClass       = $ ns__myClass(); 
    #       The general form is
    #       class_name = $ member; 

XML Schema types are associated with an optional C/C++ type declaration, a use reference, and a pointer-use reference. The pointer-use reference of the `xsd__byte` type for example, is `int*` because `char*` is reserved for strings.

For example, you can replace the `std::string` that used by default for C++ with a wide string:

    xsd__string = | std::wstring

Or replace the `char*` strings that are used by default for C with `wchar_t*`:

    xsd__string = | wchar_t* | wchar_t*

When a type binding requires only the usage to be changed, the
declaration part can be an ellipsis `...`, as in:

    prefix__type = ... | use | ptr-use

The `...` ellipsis ensures that the wsdl2h-generated type definition is preserved,
while the `use` and `ptr-use` parts are amended as specified.

This method is useful to serialize types dynamically, when XML elements
carry the <i>`xsi:type`</i> attribute indicating the type of element content.
The following illustrates an "any" type mapping for the <i>`ns:sometype`</i>
XSD type in a schema. This type will be replaced with a "any" type wrapper that
supports dynamic serialization of element types indicated by the
<i>`xsi:type`</i> attribute:

    [ 
    struct __any 
    {
      int __type;   // set to a SOAP_TYPE_T value
      void *__item; // points to data of type T as serialized per SOAP_TYPE_T
    } 
    ] 
    xsd__anyType = ... | struct __any | struct __any

where `__type` and `__item` are used to serialize any data type in the wrapper.
The `__item` member points to the value (de)serialized, with the type of this value
indicated by `__type` which is a `SOAP_TYPE_T` value for type named `T`.

To match an element with content to (de)serialize, rename the `__item` member
to the XML element name, as usual.

Additional members can be specified to extend a generated struct or class.
Class and struct extensions are of the form:

    prefix__type = $ member-declaration

For example, to add getter and setter methods to class `myns__record` (see also Section \ref gettersetter):

    myns__record = $ int get(struct soap *soap) const;
    myns__record = $ int set(struct soap *soap);

Another way to use <i>`typemap.dat`</i> is to remap one C/C++ type to another type:

    prefix__type1 == prefix__type2

This replaces `prefix__type1` by `prefix__type2` in the wsdl2h output.
For example:

    SOAP_ENC__boolean == xsd__boolean

where <i>`SOAP_ENC__boolean`</i> is replaced by <i>`xsd__boolean`</i>, which in turn may be mapped to a C `enum xsd__boolean` type or C++ `bool` type.

The <b>`$CONTAINER`</b> variable defines the container type to use in
the wsdl2h-generated declarations for C++, which is `std::vector` by default.
For example, to use `std::list` as the container in the wsdl2h-generated
declarations we add the following line to <i>`typemap.dat`</i>:

    $CONTAINER = std::list

Also a Qt container can be used instead of the default `std::vector`, for
example `QVector`:

    [
    #include <QVector>
    ]
    $CONTAINER = QVector

To remove containers, use <b>`wsdl2h -s`</b>.  This also removes `std::string`,
but you can re-introduce `std::string` with

    xsd__string = | std::string

The <i>`typemap.dat`</i> <b>`$POINTER`</b> variable defines the smart pointer to use in the
wsdl2h-generated declarations for C++, which replaces the use of `*` pointers.
For example:

    $POINTER = std::shared_ptr

Not all pointers in the generated output are replaced by smart pointers by
wsdl2h, such as pointers as union members and pointers as struct/class members
that point to arrays of values.

The variable <b>`$SIZE`</b> defines the type of array sizes, which is `int` by
default.  For example, to change array size types to `size_t`:

    $SIZE = size_t

Permissible types are `int` and `size_t`.  This variable does not affect the
size of dynamic arrays, `xsd__hexBinary` and `xsd__base64Binary` types, which
is always `int`.

🔝 [Back to table of contents](#)

# The soapcpp2 tool {#soapcpp2}

The soapcpp2 tool is invoked from the command line
and optionally takes the name of a header file as an argument or, when the file
name is absent, parses the standard input:

     soapcpp2 file.h

where <i>`file.h`</i> is an interface header file generated by wsdl2h or
developed manually to specify the service operations as function
prototypes and C/C++ data types to serialize in XML.

The soapcpp2 tool produces the XML data binding implementation source code, client-side stub functions, and server-side skeleton functions.

The type of files generated by soapcpp2 are:

* <i>`soapStub.h`</i> a modified and annotated header file produced from the input interface header file, this file is compilable by C/C++ compilers while the input interface header file is not.

* <i>`soapH.h`</i> the main header file to be included by the application source code, this file also includes <i>`soapStub.h`</i>.

* <i>`soapC.cpp`</i> (or <i>`.c`</i> for C) the serializers for the C/C++ types specified in the interface header file.

* <i>`soapClient.cpp`</i> (or <i>`.c`</i> for C) the client-side stub functions to invoke remote service operations.

* <i>`soapServer.cpp`</i> (or <i>`.c`</i> for C) the server-side skeleton functions to dispatch service requests to user-define service functions.

* <i>`soapClientLib.cpp`</i> (or <i>`.c`</i> for C) the client-side stub functions combined with local static serializers to be integrated as one big "library".

* <i>`soapServerLib.cpp`</i> (or <i>`.c`</i> for C) the service-side skeleton functions combined with local static serializers to be integrated as one big "library"

* <i>`soapXYZProxy.h`</i> the C++ proxy class declaration generated with <b>`soapcpp2 -i`</b> or <b>`soapcpp2 -j`</b>

* <i>`soapXYZProxy.cpp`</i> the C++ proxy class implementation generated with <b>`soapcpp2 -i`</b> or <b>`soapcpp2 -j`</b>

* <i>`soapXYZService.h`</i> the C++ service class declaration generated with <b>`soapcpp2 -i`</b> or <b>`soapcpp2 -j`</b>

* <i>`soapXYZService.cpp`</i> the C++ service class implementation generated with <b>`soapcpp2 -i`</b> or <b>`soapcpp2 -j`</b>

* <i>`*.xsd`</i> files are generated containing XML schemas for each namespace prefix `ns` used in the interface header file input to the soapcpp2 tool, see also Section \ref wsdl . Not applicable when the interface header file was generated with wsdl2h. 

* <i>`*.wsdl`</i> files are generated containing WSDL descriptions for each namespace prefix `ns` used by service operations in the interface header file input to the soapcpp2 tool, see also Section \ref wsdl . Not applicable when the interface header file was generated with wsdl2h. 

* <i>`*.xml`</i> files with SOAP or XML request and response messages are generated.

* <i>`*.nsmap`</i> the XML namespace mapping table, generated for the first namespace prefix `ns` found in the interface header file input to the soapcpp2 tool.

If client and service applications are to be
developed for the same Web services API then the same interface header file can be used to generate the source code for both the client and the service.  There is no need to generate a WSDL with soapcpp2 and then use that WSDL to generate a new interface header file with wsdl2h.  The new header file generated by this approach will not be identical to the original header file.

The <i>`soapClientLib.cpp`</i> and <i>`soapServerLib.cpp`</i> can be used to build client and server libraries. The serialization functions are declared static to avoid link symbol conflicts. For this approach to compile, we also should create a separate interface header file <i>`env.h`</i> with SOAP Header and Fault structures with serializers that are non-static, i.e. globally declared and implemented, as described in Section \ref dylibs .

The following files are part of the gSOAP source code package and are required to build gSOAP applications:

* <i>`gsoap/stdsoap2.h`</i> the header file to include in your source code, but already included when including <i>`soapH.h`</i>.

* <i>`gsoap/stdsoap2.c`</i> the C source code of the entire gSOAP engine.

* <i>`gsoap/stdsoap2.cpp`</i> (or <i>`.c`</i> for C) the source code of the entire gSOAP engine.

* <i>`gsoap/dom.cpp`</i> (or <i>`.c`</i> for C) the source code of the DOM parser, which is optional and only required when using DOM such as with WS-Security.

🔝 [Back to table of contents](#)

## soapcpp2 options        {#soapcpp2options}

The soapcpp2 tool supports the following command-line options:

option                     | result
-------------------------- | ------
[`-0`](#soapcpp2-0)        | no SOAP, generate REST source code
[`-1`](#soapcpp2-1)        | generate SOAP 1.1 source code
[`-2`](#soapcpp2-2)        | generate SOAP 1.2 source code
[`-A`](#soapcpp2-A)        | require HTTP SOAPAction headers to invoke server-side operations
[`-a`](#soapcpp2-a)        | use HTTP SOAPAction headers with WS-Addressing to invoke server-side operations
[`-b`](#soapcpp2-b)        | serialize byte arrays `char[N]` as string
[`-C`](#soapcpp2-C)        | generate client-side source code only
[`-c`](#soapcpp2-c)        | generate C source code
[`-c++`](#soapcpp2-c)      | generate C++ source code (default)
[`-c++11`](#soapcpp2-c)    | generate C++ source code optimized for C++11 (compile with `-std=c++11`)
[`-d`](#soapcpp2-d) `path` | use `path` to save files
[`-Ec`](#soapcpp2-E)       | generate extra functions for deep copying
[`-Ed`](#soapcpp2-E)       | generate extra functions for deep deletion
[`-Et`](#soapcpp2-E)       | generate extra functions for data traversals with callback functions
[`-e`](#soapcpp2-e)        | generate SOAP RPC encoding style bindings (also use `-1` or `-2`)
[`-f`](#soapcpp2-f) `N`    | multiple `soapC` files, with `N` serializer definitions per file (N>=10)
[`-g`](#soapcpp2-g)        | generate XML sample messages in template format for [testmsgr](../../testmsgr/html/index.html)
[`-h`](#soapcpp2-h)        | display help info and exit
[`-I`](#soapcpp2-I) `path` | use `path`(s) for `#import` (paths separated with `:`)
[`-i`](#soapcpp2-i)        | generate C++ service proxies and objects inherited from soap struct
[`-j`](#soapcpp2-j)        | generate C++ service proxies and objects that share a soap struct
[`-L`](#soapcpp2-L)        | don't generate `soapClientLib` and `soapServerLib`
[`-l`](#soapcpp2-l)        | generate linkable modules (experimental)
[`-m`](#soapcpp2-m)        | generate source code for the Matlab(tm) MEX compiler (deprecated)
[`-n`](#soapcpp2-n)        | use service name to rename service functions and namespace table
[`-p`](#soapcpp2-p) `name` | save files with new prefix `name` instead of `soap`
[`-Q`](#soapcpp2-Q) `name` | use `name` as the C++ namespace, including custom serializers
[`-q`](#soapcpp2-q) `name` | use `name` as the C++ namespace, excluding custom serializers
[`-r`](#soapcpp2-r)        | generate soapReadme.md report
[`-S`](#soapcpp2-S)        | generate server-side source code only
[`-s`](#soapcpp2-s)        | generate stub and skeleton functions with strict XML validation checks
[`-T`](#soapcpp2-T)        | generate server auto-test source code
[`-t`](#soapcpp2-t)        | generate source code for fully `xsi:type` typed SOAP/XML messages
[`-u`](#soapcpp2-u)        | uncomment WSDL/schema output by suppressing XML comments
[`-V`](#soapcpp2-V)        | display the current version and exit
[`-v`](#soapcpp2-v)        | verbose output
[`-w`](#soapcpp2-w)        | don't generate WSDL and schema files
[`-x`](#soapcpp2-x)        | don't generate sample XML message files
[`-y`](#soapcpp2-y)        | include C/C++ type access information in sample XML messages
[`-z1`](#soapcpp2-z)       | compatibility: generate old-style C++ service proxies and objects
[`-z2`](#soapcpp2-z)       | compatibility with 2.7.x: omit XML output for NULL pointers
[`-z3`](#soapcpp2-z)       | compatibility up to 2.8.30: `_param_N` indexing and nillable pointers
[`-z4`](#soapcpp2-z)       | compatibility up to 2.8.105: `char*` member defaults, even when the XML element is omitted

For example

     soapcpp2 -L -c -d projects -p my -x file.h

This saves the following source code files:

- <i>`projects/myH.h`</i> serialization functions, this file should be included in projects.
- <i>`projects/myC.c`</i> serialization functions
- <i>`projects/myClient.c`</i> client call stub functions
- <i>`projects/myServer.c`</i> server request dispatcher
- <i>`projects/myStub.h`</i> annotated copy of the source interface header file
- <i>`projects/ns.nsmap`</i> namespace table, this file should be included or used in projects.
- <i>`projects/ns.wsdl`</i> WSDL with Web service definitions
- <i>`projects/ns.xsd`</i>  XML schema

Windows users can use the usual <b>`/`</b> for compile-time flags as well as <b>`-`</b>, for example:

    soapcpp2 -L -c /d projects /p my /x file.h

Options <b>`-A`</b>, <b>`-a`</b>, <b>`-c`</b>, <b>`-c++`</b>, <b>`-c++11`</b>, <b>`-e`</b>, <b>`-i`</b>, <b>`-j`</b>, <b>`-n`</b>, <b>`-s`</b>, <b>`-t`</b>, <b>`-w`</b>, and <b>`-x`</b> can also be specified in the interface header file for soapcpp2 using the <b>`//gsoapopt`</b> directive, for example:

~~~{.cpp}
    // Generate pure C and do not produce WSDL output: 
    //gsoapopt cw 
    int ns__webmethod(char *in, char **out);
~~~

🔝 [Back to table of contents](#)

### soapcpp2 -0 {#soapcpp2-0}

This option generates XML REST source code by disabling SOAP bindings,
essentially disabling the SOAP protocol and replacing it by direct XML REST
messaging.

This option uses `::soap_set_version` at the client side in the generated
source code to enable XML REST messaging, disabling SOAP.

In addition, the soapcpp2 tool nullifies the SOAP namespaces from the the
generated namespace table file to force a server application that uses this
table to use XML REST only:

~~~{.cpp}
    struct Namespace namespaces[] = {
        { "SOAP-ENV", NULL, NULL, NULL },
        { "SOAP-ENC", NULL, NULL, NULL },
        { "xsi", "http://www.w3.org/2001/XMLSchema-instance", "http://www.w3.org/*/XMLSchema-instance", NULL },
        { "xsd", "http://www.w3.org/2001/XMLSchema", "http://www.w3.org/*/XMLSchema", NULL },
        ...
        { NULL, NULL, NULL, NULL}
    };
~~~

@note Web services applications developed with gSOAP support both REST and SOAP
messaging automatically when the namespace table is left intact (i.e. generated
without option <b>`-0`</b>) with the SOAP namespaces present in the table.  XML
REST request messages are served and REST messages returned.  Likewise, SOAP
1.1 request messages are served and SOAP 1.1 messages returned, SOAP 1.2
request messages are served and SOAP 1.2 messages returned.

For example, the following example calculator service SOAP and XML REST request
messages are served by a gSOAP service developed with SOAP 1.1 as the default
protocol:

<div class="alt">
~~~{.xml}
    <SOAP-ENV:Envelope
        xmlns:SOAP-ENV="http://schemas.xmlsoap.org/soap/envelope/"
        xmlns:SOAP-ENC="http://schemas.xmlsoap.org/soap/encoding/"
        xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
        xmlns:xsd="http://www.w3.org/2001/XMLSchema"
        xmlns:ns="urn:calc">
      <SOAP-ENV:Body SOAP-ENV:encodingStyle="http://schemas.xmlsoap.org/soap/encoding/">
        <ns:add>
          <a>2</a>
          <b>3</b>
        </ns:add>
      </SOAP-ENV:Body>
    </SOAP-ENV:Envelope>
~~~
</div>

<div class="alt">
~~~{.xml}
    <ns:add
        xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
        xmlns:xsd="http://www.w3.org/2001/XMLSchema"
        xmlns:ns="urn:calc">
      <a>2</a>
      <b>3</b>
    </ns:add>
~~~
</div>

The server returns the following XML SOAP 1.1 and XML REST responses:

<div class="alt">
~~~{.xml}
    <SOAP-ENV:Envelope
        xmlns:SOAP-ENV="http://schemas.xmlsoap.org/soap/envelope/"
        xmlns:SOAP-ENC="http://schemas.xmlsoap.org/soap/encoding/"
        xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
        xmlns:xsd="http://www.w3.org/2001/XMLSchema"
        xmlns:ns="urn:calc">
      <SOAP-ENV:Body SOAP-ENV:encodingStyle="http://schemas.xmlsoap.org/soap/encoding/">
        <ns:addResponse>
          <result>5</result>
        </ns:addResponse>
      </SOAP-ENV:Body>
    </SOAP-ENV:Envelope>
~~~
</div>

<div class="alt">
~~~{.xml}
    <ns:addResponse
        xmlns:SOAP-ENV="http://schemas.xmlsoap.org/soap/envelope/"
        xmlns:SOAP-ENC="http://schemas.xmlsoap.org/soap/encoding/"
        xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
        xmlns:xsd="http://www.w3.org/2001/XMLSchema"
        xmlns:ns="urn:calc">
      <result>0</result>
    </ns:addResponse>
~~~
</div>

By default all XML namespaces are included with the root element, which
improves messaging performance at the sending and receiving sides, because
a stack of <i>`xmlns`</i> binding scopes does not need to be maintained.
Use `#SOAP_XML_CANONICAL` to emit <i>`xmlns`</i> binding pairs when the
XML namespace prefix is used.  This is slower but may or may not reduce the
message size.

🔝 [Back to table of contents](#)

### soapcpp2 -1 {#soapcpp2-1}

This option forces SOAP 1.1 bindings globally in the generated source code,
thereby overriding the SOAP protocol version used in the interface header file
input.

This option uses `::soap_set_version` at the client to enable SOAP 1.1 request
and response messages, disallowing SOAP 1.2.

In addition, the soapcpp2 tool saves the SOAP 1.1 namespaces in the
second column of the generated namespace table file and the SOAP 1.2 in the
third column to allow the server to accept SOAP 1.1 and SOAP 1.2 requests:

~~~{.cpp}
    struct Namespace namespaces[] = {
        { "SOAP-ENV", "http://schemas.xmlsoap.org/soap/envelope/", "http://www.w3.org/*/soap-envelope", NULL },
        { "SOAP-ENC", "http://schemas.xmlsoap.org/soap/encoding/", "http://www.w3.org/*/soap-encoding", NULL },
        { "xsi", "http://www.w3.org/2001/XMLSchema-instance", "http://www.w3.org/*/XMLSchema-instance", NULL },
        { "xsd", "http://www.w3.org/2001/XMLSchema", "http://www.w3.org/*/XMLSchema", NULL },
        ...
        { NULL, NULL, NULL, NULL}
    };
~~~

🔝 [Back to table of contents](#)

### soapcpp2 -2 {#soapcpp2-2}

This option forces SOAP 1.2 bindings globally in the generated source code,
thereby overriding the SOAP protocol version used in the interface header file
input.

This option uses `::soap_set_version` at the client to enable SOAP 1.2 request
and response messages, disallowing SOAP 1.1.

In addition, the soapcpp2 tool saves the SOAP 1.2 namespaces in the
second column of the generated namespace table file and the SOAP 1.1 in the
third column to allow the server to accept SOAP 1.1 and SOAP 1.2 requests:

~~~{.cpp}
    struct Namespace namespaces[] = {
        { "SOAP-ENV", "http://www.w3.org/2003/05/soap-envelope", "http://schemas.xmlsoap.org/soap/envelope/", NULL },
        { "SOAP-ENC", "http://www.w3.org/2003/05/soap-encoding", "http://schemas
        { "xsi", "http://www.w3.org/2001/XMLSchema-instance", "http://www.w3.org/*/XMLSchema-instance", NULL },
        { "xsd", "http://www.w3.org/2001/XMLSchema", "http://www.w3.org/*/XMLSchema", NULL },
        ...
        { NULL, NULL, NULL, NULL}
    };
~~~

🔝 [Back to table of contents](#)

### soapcpp2 -A {#soapcpp2-A}

This option generates server-side source code that requires HTTP SOAPAction
headers to be present.  The server invokes server-side operations
based on the SOAPAction header value in request messages, instead of the
SOAP/XML request message name which is ignored.  This option is used with
WS-Addressing, WS-ReliableMessaging, and WS-Discovery servers to relay messages
based on HTTP SOAPAction headers and/or the SOAP Header <i>`wsa:Action`</i>
when present (the latter requires the [WS-Addressing plugin](wsaplugin)).

Alternatively, use [<b>`soapcpp2 -a`</b> option <b>`-a`</b>](#soapcpp2-a) to
let the server invoke server-side operations based on the SOAPAction header
value in request messages when present, otherwise when not present this lets
the server invoke server-side operations based on the SOAP/XML request message
name as usual.

This option can also be specified by the `//gsoapopt A` directive in the
interface header file.

🔝 [Back to table of contents](#)

### soapcpp2 -a {#soapcpp2-a}

This option generates server-side source code that uses HTTP SOAPAction headers
when present to invoke server-side operations based on the SOAPAction header
value in request messages, otherwise when not present lets the server invoke
server-side operations based on the SOAP/XML request message name as usual.
This option is used with WS-Addressing, WS-ReliableMessaging, and WS-Discovery
servers to relay messages based on HTTP SOAPAction headers and/or the SOAP
Header <i>`wsa:Action`</i> when present (the latter requires the
[WS-Addressing plugin](wsaplugin)).

Alternatively, use [<b>`soapcpp2 -A`</b> option <b>`-A`</b>](#soapcpp2-A) to
require HTTP SOAPAction headers to be present in SOAP request messages
to invoke server-side operations.

This option can also be specified by the `//gsoapopt a` directive in the
interface header file.

🔝 [Back to table of contents](#)

### soapcpp2 -b {#soapcpp2-b}

This option serializes byte arrays specified as `char[N]` as strings.
Without this option `char[N]` is serialized as an array of bytes.
Fixed-size arrays specified in the interface header file input are generally
serialized as arrays in XML using <i>`item`</i> elements.

For example:

~~~{.cpp}
    struct ns__record
    {
        char bytes[3];
        int ints[2];
    };
~~~

By default without this option the `ns__record` struct is serialized as:

<div class="alt">
~~~{.xml}
    <ns:record>
      <bytes>
        <item>65</item>
        <item>66</item>
        <item>0</item>
      </bytes>
      <ints>
        <item>1</item>
        <item>2</item>
      </ints>
    </ns:record>
~~~
</div>

With this option, the `ns__record` struct is serialized as:

<div class="alt">
~~~{.xml}
    <ns:record>
      <bytes>AB</bytes>
      <ints>
        <item>1</item>
        <item>2</item>
      </ints>
    </ns:record>
~~~
</div>

🔝 [Back to table of contents](#)

### soapcpp2 -C {#soapcpp2-C}

This option restricts soapcpp2 to generate client-side source code only.  When
this option is combined with
[<b>`soapcpp2 -CS`</b> option <b>`-S`</b>](#soapcpp2-S), no client and server
source code is generated.

🔝 [Back to table of contents](#)

### soapcpp2 -c -c++ -c++11 {#soapcpp2-c}

Option <b>`-c`</b> generates C source code, <b>`-c++`</b> generates C++ source
code, and <b>`-c++11`</b> generates C++11 source code.

@note The `//gsoapopt` directive in the interface header file takes priority
over this option, when `c`, `c++`, or `c++11` is declared with this directive
in the interface header file.

🔝 [Back to table of contents](#)

### soapcpp2 -d {#soapcpp2-d}

This option specifies a path to save the generated files.  For example:

    soapcpp2 -d source file.h

This saves files to the <i>`source/`</i> directory located within the current
directory, which should exist and should be writable.

🔝 [Back to table of contents](#)

### soapcpp2 -Ec -Ed -Et {#soapcpp2-E}

These options generate extra functions for deep copying of serializable C/C++
data, deep deletion of serializable C/C++ data, and deep data traversals with
user-defined callback functions over serializable C/C++ data.

For a serializable type `T` declared in the interface header file for soapcpp2,
option <b>`-Ec`</b> generates:

- `virtual T * T::soap_dup(struct soap*) const` where `T` is a class,
  returns a duplicate of this object by deep copying, replicating all deep
  cycles and shared pointers when a managing `soap` context is provided as
  argument.  Deep copy is a tree when argument is NULL, but the presence of
  deep cycles will lead to non-termination.  Use flag `SOAP_XML_TREE` with the
  managing context to copy into a tree without cycles and pointers to shared
  objects.

- `T * soap_dup_T(struct soap*, T *dst, const T *src)` where `T` is not a class,
  deep copy `src` into `dst`, replicating all deep cycles and shared pointers
  when a managing `soap` context is provided as argument.  When `dst` is NULL,
  allocates space for `dst` and returns a pointer to the allocated copy.  Deep
  copy results in a tree when the `soap` context is NULL, but the presence of
  deep cycles will lead to non-termination.  Use flag `SOAP_XML_TREE` with
  managing context to copy into a tree without cycles and pointers to shared
  objects.  Returns `dst` or allocated copy when `dst` is NULL.

For a serializable type `T` declared in the interface header file for soapcpp2,
option <b>`-Ed`</b> generates:

- `virtual void T::soap_del() const` where `T` is a class, deletes all
  heap-allocated members of this object by deep deletion ONLY IF this object
  and all of its (deep) members are not managed by a `soap` context AND the deep
  structure is a tree (no cycles and co-referenced objects by way of multiple
  (non-smart) pointers pointing to the same data).  Can be safely used after
  `T::soap_dup(NULL)` to delete the deep copy.  Does not delete the object
  itself.

- `void soap_del_T(const T*)` where `T` is not a class, deletes all
  heap-allocated members of this object by deep deletion ONLY IF this object
  and all of its (deep) members are not managed by a `soap` context AND the deep
  structure is a tree (no cycles and co-referenced objects by way of multiple
  (non-smart) pointers pointing to the same data).  Can be safely used after
  `soap_dup_T(NULL, NULL, const T*)` to delete the deep copy returned.  Does
  not delete the object itself.

For a serializable type `T` declared in the interface header file for soapcpp2,
option <b>`-Et`</b> generates:

- `virtual void T::soap_traverse(struct soap *soap, const char *tag, soap_walker p, soap_walker q)`
  where `T` is a class, uses function callbacks `p` and `q` to traverse this
  object by deep ordered tree traversals over its members when non-NULL.
  Function `p` is a pre-order function that is called before objects and data
  are visited recursively and function `q` is a post-order function that is
  called after objects and data are visited recursively.  Either `p` or `q` may
  be NULL.  The `tag` string is passed to `p` and `q` and should not be NULL.
  Cyclic graphs are treated as trees by pruning pointer back-edges, though this
  method does not always prevent a data node from being visited twice.

- `void soap_traverse_T(struct soap *soap, T *data, const char *tag, soap_walker p, soap_walker q)`
  where `T` is not a class, uses function callbacks `p` and `q` to traverse this
  data by deep ordered tree traversals over its members when present and
  non-NULL.  Function `p` is a pre-order function that is called before objects
  and data are visited recursively and function `q` is a post-order function
  that is called after objects and data are visited recursively.  Either `p` or
  `q` may be NULL.  The `tag` string is passed to `p` and `q` and should not be
  NULL.  Cyclic graphs are treated as trees by pruning pointer back-edges.
  though this method does not always prevent a data node from being visited twice.

The pre-order `p` and post-order `q` callback functions should be declared as
a `soap_walker` function, which has the following function signature:

~~~{.cpp}
    void soap_walker(struct soap *soap, void *data, int soap_type, const char *tag, const char *type)
~~~

where `data` points to the data node visited which is of type `soap_type` (a
`SOAP_TYPE_T` constant), `tag` is the non-NULL element or attribute
tag name (qualified or unqualified), and `type` is the non-NULL C/C++ type of
the data.  The `void* ::soap::user` member can be used to pass user-defined
data to the callbacks.

For example:

~~~{.cpp}
    // file: record.h
    //gsoap ns schema namespace: urn:example
    struct ns__record
    {
      @ char *name;
        int value;
        struct ns__record *subrecord;
    };
~~~

    soapcpp2 -Ecdt record.h

The main program:

~~~{.cpp}
    #include "soapH.h"
    #include "ns.nsmap"

    int main()
    {
      struct soap *soap = soap_new();
      ns__record record;                            // a serializable type
      ns__record rec_dup;
      int indent = 0;
      soap->recvfd = open(file, O_RDONLY); 
      if (soap->recvfd < 0)
        ... // error
      if (soap_read_ns__record(soap, &record))      // deserialize from file into managed memory
        ... // error
      close(soap->recvfd);
      if (soap_write_ns__record(soap, &record))     // serialize to standard output
        ... // error
      soap->user = (void*)&indent;
      soap_traverse_ns__record(soap, record, "record", pre, post);
      soap_dup_ns__record(NULL, &rec_dup, &record); // deep copy the record to unmanaged memory
      soap_destroy(soap);                           // delete managed objects
      soap_end(soap);                               // delete managed data and temporaries
      soap_free(soap);                              // free the context
      soap_del_ns__record(&rec_dup)                 // deep delete unmanaged record
    }

    void pre(struct soap *soap, void *a, int n, const char *s, const char *t)
    {
      printf("\n%*s%s %s = {", (*(int*)soap->user)++, "", t, s);
      if (n == SOAP_TYPE_int)
        printf(" %d", *(int*)a);
      else if (n == SOAP_TYPE_string)
        printf(" %s", *(char**)a ? *(char**)a : "(n/a)");
    }

    void post(struct soap *soap, void *a, int n, const char *s, const char *t)
    {
      printf(" }");
      (*(int*)soap->user)--;
    }
~~~

The `soap_read_ns__record` deserializes the following XML:

<div class="alt">
~~~{.xml}
    <ns:record xmlns:ns="urn:example" name="foo">
      <value>123</value>
      <subrecord name="bar">
        <value>456</name>
      </subrecord>
    </ns:record>
~~~
</div>

Then `soap_traverse_ns__record` call displays the contenst of `record` using
the `pre` and `post` walker functions:

    struct ns__record record = {
     char * name = { foo }
     int value = { 123 }
     struct ns__record subrecord = {
      char * name = { bar }
      int value = { 456 } } }

🔝 [Back to table of contents](#)

### soapcpp2 -e {#soapcpp2-e}

This option forces SOAP RPC encoding bindings globally in the generated source
code, when the SOAP messaging style is not declared in the interface header
file with directives.

SOAP document/literal style messaging it the default messaging style.  The
messaging style can be specified with the `//gsoap <prefix> service style:` and
`//gsoap <prefix> service encoding:` [directives](#directives).  See also
[SOAP RPC encoded versus document/literal style](#literal).

This option can also be specified by the `//gsoapopt e` directive in the
interface header file.

🔝 [Back to table of contents](#)

### soapcpp2 -f {#soapcpp2-f}

This option splits the serialization source code saved to <i>`soapC.c`</i> and
<i>`soapC.cpp`</i> files into multiple <i>`soapC_NNN`</i> files as specified by
the numeric parameter.  This option alleviates compilation issues with very
large source code files.

For example:

    soapcpp2 -f40 file.h

This generates multiple <i>`soapC_NNN.cpp`</i> files each with 40 serializers, with
<i>`NNN`</i> counting from <i>`001`</i> onward.

The value of this option must be larger or equal to 10.

🔝 [Back to table of contents](#)

### soapcpp2 -g {#soapcpp2-g}

This option generates XML sample messages in template format for the gSOAP
Test Messenger [testmsgr](../../testmsgr/html/index.html) tool to test SOAP and
REST XML clients and servers.

By default without this option, soapcpp2 generates sample XML messages with the
proper XML structure but without useful data.  The Test Messenger tool generates
random messages directed by the template parameters included by
<b>`soapcpp2 -g`</b> option <b>`-g`</b>.

This option only has effect when
[<b>`soapcpp2 -x`</b> option <b>`-x`</b>](#soapcpp2-x) is not used, which skips
the generation of sample messages.

🔝 [Back to table of contents](#)

### soapcpp2 -h {#soapcpp2-h}

This option displays help info and then exits.

🔝 [Back to table of contents](#)

### soapcpp2 -I {#soapcpp2-I}

This option specifies one or more directory paths to search for imported
interface header files.  Multiple paths are separated by a colon.

For example:

    soapcpp2 -I path1:path2 file.h

This searches <i>`path1`</i> and then <i>`path2`</i> for files that are
imported with `#import` in <i>`file.h`</i>.

🔝 [Back to table of contents](#)

### soapcpp2 -i {#soapcpp2-i}

This option generates C++ client-side proxy classes and server-side service
classes, where the classes inherit the `::soap` context struct with the engine
state to handle communications and manage memory independently of other class
instances.

By contrast, [<b>`soapcpp2 -j`</b> option <b>`-j`</b>](#soapcpp2-j) allows a
`::soap` context to be used and reused for multiple proxy and server instances.

This option can also be specified by the `//gsoapopt i` directive in the
interface header file.

This option has no effect for C source code output.

🔝 [Back to table of contents](#)

### soapcpp2 -j {#soapcpp2-j}

This option generates C++ client-side proxy classes and server-side service
classes, where the classes have a pointer member `soap` to a `::soap` context
struct that handles communications and manages memory.

By contrast to [<b>`soapcpp2 -i`</b> option <b>`-i`</b>](#soapcpp2-i), this
option allows a `::soap` context to be used and reused for multiple proxy and
server instances.

This option can also be specified by the `//gsoapopt j` directive in the
interface header file.

This option has no effect for C source code output.

🔝 [Back to table of contents](#)

### soapcpp2 -L {#soapcpp2-L}

This option skips the generation of the <i>`soapClientLib`</i> and <i>`soapServerLib`</i>
files.  These files are generally not needed to build client and server applications.

These files are useful to compile multiple "libraries" of client and
server applications, such that all serialization source code is declared static
and kept hidden from the global scope, which makes the serialization functions
inaccessible to the global scope to prevent global name clashes.

Alternatively, use <b>`soapcpp2 -q name`</b> option <b>`-q name`</b> to develop
C++ applications with C++ namespaces to prevent global name clashes.

🔝 [Back to table of contents](#)

### soapcpp2 -l {#soapcpp2-l}

This option is experimental and should only be used to generate source code for
modules.  This option is auto-enabled when a `#module` directive is found
in an interface header file for soapcpp2, see
[how to build modules and libraries with the #module directive](#module).

🔝 [Back to table of contents](#)

### soapcpp2 -m {#soapcpp2-m}

This option to generate source code for the Matlab(tm) MEX compiler is
deprecated.

🔝 [Back to table of contents](#)

### soapcpp2 -n {#soapcpp2-n}

This option renames the generated service functions `::soap_serve` to
`name_serve` and the generated namespace table `::namespaces` to
`name_namespaces` to the <b>`name`</b> specified with the
[<b>`soapcpp2 -n -p name`</b> option <b>`-p name`</b>](#soapcpp2-p).

This option is useful to prevent name clashes when soapcpp2 is invoked multiple
times to generate source code for different parts of an application.  See also
[how to create client/server libraries](#dylibs).

This option can also be specified by the `//gsoapopt n` directive in the
interface header file.

🔝 [Back to table of contents](#)

### soapcpp2 -p {#soapcpp2-p}

This option saves source code files with the specified file name prefix
<i>`name`</i> with <b>`soapcpp2 -p name`</b> instead of <i>`soap`</i> as the
file name prefix.

This option is useful to prevent name clashes when soapcpp2 is invoked multiple
times to generate source code for different parts of an application.  See also
[how to create client/server libraries](#dylibs).

For example:

    soapcpp2 -p foo file.h

This saves `fooStub.h`, `fooH.h`, `fooC.cpp`, and so on.

When the main application is build from the renamed <i>`name`</i>-prefixed
source code files, plugins and custom serializers that are compiled and linked
with the application should include <i>`nameH.h`</i> instead of
<i>`soapH.h`</i>.  This can be done with the <b>`-D SOAP_H_FILE=nameH.h`</b>
option to the C/C++ compiler to rename this file to include instead of
<i>`soapH.h`</i>.

🔝 [Back to table of contents](#)

### soapcpp2 -Q {#soapcpp2-Q}

This option specifies a C++ namespace name for the generated source code,
including for the custom serializers when used.  See also
[<b>`soapcpp2 -q name`</b> option <b>`-q name`</b>](#soapcpp2-q) for details on
specifying C++ namespaces.

The source code files are saved with <i>`name`</i> as prefix instead of
<i>`soap`</i>.  This means that all plugins and custom serializers
that are compiled and linked with the application should include <i>`nameH.h`</i>
instead of <i>`soapH.h`</i>.  This can be done with the <b>`-D SOAP_H_FILE=nameH.h`</b>
option to the C/C++ compiler to rename this file to include instead of <i>`soapH.h`</i>.

This option has no effect for C source code output.

🔝 [Back to table of contents](#)

### soapcpp2 -q {#soapcpp2-q}

This option specifies a C++ namespace name for the generated source code,
excluding the custom serializers when used.  See also
[<b>`soapcpp2 -Q name`</b> option <b>`-Q name`</b>](#soapcpp2-Q).

This option is the same as specifying a C++ namespace in the interface header
file that encapsulates all declarations:

~~~{.cpp}
    namespace name {

      ... // all of the interface header file content goes here

    }
~~~

This interface header file format is generated with
[<b>`wsdl2h -q name`</b> option <b>`-q name`</b>](#wsdl2h-q).

The source code files are saved with <i>`name`</i> as prefix instead of
<i>`soap`</i>.  This means that all plugins and custom serializers
that are compiled and linked with the application should include <i>`nameH.h`</i>
instead of <i>`soapH.h`</i>.  This can be done with the <b>`-D SOAP_H_FILE=nameH.h`</b>
option to the C/C++ compiler to rename this file to include instead of <i>`soapH.h`</i>.

See \ref codenamespace for details on using C++ namespaces to build client and
server applications, which requires a <i>`env.h`</i> file with SOAP Header and
Fault definitions to be compiled with:

     soapcpp2 -penv env.h

The generated <i>`envC.cpp`</i> file holds the SOAP Header and Fault serializers and you can
link this file with your client and server applications.

This option has no effect for C source code output.

🔝 [Back to table of contents](#)

### soapcpp2 -r {#soapcpp2-r}

This option generates a <i>`soapReadme.md`</i> markdown report.  This report
includes details pertaining the serializable data types and Web client and
service operations, covering XML type details, serialization functions, and
SOAP/REST API programming details.

The markdown report is readable as it is,
but can be converted to HTML for improved readability with Doxygen or with
pandoc, or can be browsed in Firefox with
[https://www.genivia.com/files/readmeviewer.html.zip](readmeviewer.html).

🔝 [Back to table of contents](#)

### soapcpp2 -S {#soapcpp2-S}

This option restricts soapcpp2 to generate server-side source code only.  When
this option is combined with
[<b>`soapcpp2 -CS`</b> option <b>`-C`</b>](#soapcpp2-C), no client and server
source code is generated.

🔝 [Back to table of contents](#)

### soapcpp2 -s {#soapcpp2-s}

This option generates client-side stub functions and proxy classes, server-side
skeleton functions and service classes with strict XML validation checks
enabled.  This option effectively hard-codes the `#SOAP_XML_STRICT` run time
mode flag. 

This option can also be specified by the `//gsoapopt s` directive in the
interface header file.

@warning This option is not recommended for SOAP RPC encoding style messaging,
but XML REST and SOAP/XML document/literal style messages can be validated.

🔝 [Back to table of contents](#)

### soapcpp2 -T {#soapcpp2-T}

This option generates server auto-test source code.  The generated source code
implements a test server <i>`soapTester.c`</i> (for C) or
<i>`soapTester.cpp`</i> (for C++) that can be deployed to echo client requests,
for example for testing purposes.

For example:

    soapcpp2 -T file.h
    c++ -o tester soapTester.cpp soapServer.cpp soapC.cpp stdsoap2.cpp
    ./tester 8192 8080

This runs the <i>`tester`</i> server on port 8080 with `::soap` context
initialization mode flag 8192 = 0x2000 = `#SOAP_XML_INDENT`.

See [generating an auto test server for client testing](#autotest) for more details.
More advanced servers for testing are available with the gSOAP Test Messenger
[testmsgr](../../testmsgr/html/index.html) tool to test SOAP and REST XML
clients and servers.

🔝 [Back to table of contents](#)

### soapcpp2 -t {#soapcpp2-t}

This option generates source code to fully annotate SOAP/XML messages with
<i>`xsi:type`</i> attribute values.  This option is useful for SOAP RPC encoded
messaging with SOAP applications that require <i>`xsi:type`</i> attributes for
all XML elements in SOAP messages.

This option can also be specified by the `//gsoapopt t` directive in the
interface header file.

🔝 [Back to table of contents](#)

### soapcpp2 -u {#soapcpp2-u}

This option uncomments WSDL and XSD files generated by soapcpp2 by supressing
the inclusion of `<!-- -->` comments to annotate WSDL and XSD files.

🔝 [Back to table of contents](#)

### soapcpp2 -V {#soapcpp2-V}

This option displays the current soapcpp2 tool version and then exits.

🔝 [Back to table of contents](#)

### soapcpp2 -v {#soapcpp2-v}

This option enables verbose output to assist in debugging the soapcpp2 tool.

🔝 [Back to table of contents](#)

### soapcpp2 -w {#soapcpp2-w}

This option skips the generation of WSDL and XSD files.

This option can also be specified by the `//gsoapopt w` directive in the
interface header file.

🔝 [Back to table of contents](#)

### soapcpp2 -x {#soapcpp2-x}

This option skips the generation of sample XML message files.

This option can also be specified by the `//gsoapopt x` directive in the
interface header file.

🔝 [Back to table of contents](#)

### soapcpp2 -y {#soapcpp2-y}

This option adds C/C++ type information to the sample XML message files
generated by soapcpp2.

🔝 [Back to table of contents](#)

### soapcpp2 -z {#soapcpp2-z}

These options are for backward compatiility with older gSOAP releases:

- <b>`-z1`</b> compatibility: generate old-style C++ service proxies and objects
- <b>`-z2`</b> compatibility with 2.7.x: omit XML output for NULL pointers
- <b>`-z3`</b> compatibility up to 2.8.30: `_param_N` indexing; nillable pointers
- <b>`-z4`</b> compatibility up to 2.8.105: `char*` member defaults, even when the XML element is omitted

🔝 [Back to table of contents](#)

## The #import directive        {#import}

The `#import` directive is used to include interface header files into other interface header files for
soapcpp2.
By contrast, the `#include` directive (and `#define` directive for that matter) is moved by the soapcpp2 tool into the generated source code file <i>`soapStub`</i>, see Section \ref pragmas .

The `#import` directive is used for two purposes: we use it to include the contents of one interface header file into another interface header file and to import a module, see Section \ref module .

An example of the `#import` directive:

~~~{.cpp}
    #import "mydefs.h" 

    int ns__webmethod(ns__record *in, struct ns__webmethodResponse { ns__record out; } *out);
~~~

where `"mydefs.h"` is an interface header file that defines `ns__record`:

~~~{.cpp}
    struct ns__record
    {
      const char *name;
      const char *address;
    };
    typedef struct ns__record ns__record;
~~~

🔝 [Back to table of contents](#)

## The #include and #define directives        {#pragmas}

The `#include` and `#define` directives are copied by the soapcpp2 tool into the generated source code.
These  directives are added to the top of the generated <i>`soapStub.h`</i> before
any other header file is included. Therefore, `#include` and `#define` directives can be used to influence the
generated source code files.

The following example interface header file for soapcpp2 refers to `std::ostream`:

~~~{.cpp}
    #include <ostream> 
    #define SOME_VALUE 123

    // std::ostream can't be serialized, but need to be declared to make it visible:
    extern class std::ostream;

    class ns__myClass 
    {
     public:
      virtual void print(std::ostream &s) const; // we need std::ostream here 
      ... //
    };
~~~

This example also uses an `#include` and a `#define` directive that will be added to the top of <i>`soapStub.h`</i> before <i>`gsoap/stdsoap2.h`</i> is included.

@warning Using `#define` to override `WITH_MACRO` and `SOAP_MACRO` compile-time flags is not recommended because the <i>`gsoap/stdsoap2.cpp`</i> (<i>`gsoap/stdsoap2.c`</i> for C) is used to build the <b>`-lgsoap++`</b> (and <b>`-lgsoap`</b> for C) library, which is not affected by these macros whereas the `::soap` context is, as used by the application, leading to predictable crashes.  Use `#SOAPDEFS_H` or `#WITH_SOAPDEFS_H` to define macros that are visible to all source code compiled.

🔝 [Back to table of contents](#)

## Service operation specification format   {#specformat}

A service operation is specified as a function prototype in an interface header
file for soapcpp2.  For the function prototypes specified, the soapcpp2 tool
generates client stub functions to invoke remote services and generates server
skeleton functions to implement services.

The service operation specified by a function prototype should return `int`, which is either `#SOAP_OK` for success and
a `::soap_status` error code for failure, see Section \ref errcodes .

The general format of a service operation specification is:

~~~{.cpp}
    int prefix__method_name(inparam1, inparam2, ..., inparamn, outparam);
~~~

where

* `prefix__` is the XML namespace prefix of the method

* `method_name`  is the service operation name

* `inparam1`, ..., `inparamn` are the input parameters to the service operation, which are either values or pointer types, but not references

* `outparam`     is the single output parameter of the service operation, which must be a pointer or a reference type.

A single output parameter is specified and multiple output parameters should be wrapped in a struct or class,
see Section \ref param .
The fully qualified name of the function `namespace_prefix__method_name` must be unique and cannot match the name of a `struct`,
`class`, or `enum` declared in the same header file.

The method request is send as an XML message using the qualified function name with the input parameters in XML:

<div class="alt">
~~~{.xml}
    <prefix:method-name>
      <inparam1>...</inparam1> 
      <inparam2>...</inparam2> 
      ... 
      <inparamn>...</inparamn> 
    </prefix:method-name>
~~~
</div>

where the <i>`inparam1`</i>, ..., <i>`inparamn`</i> elements are the XML element representations of the `inparam` parameter name declarations.

The XML response by the Web service is of the form:

<div class="alt">
~~~{.xml}
    <prefix:method-nameResponse>
      <outparam>...</outparam> 
    </prefix:method-nameResponse>
~~~
</div>

where the <i>`outparam`</i> element is the XML element representation of the `outparam` parameter name declaration, see
Section \ref idtrans . By convention, the response element name is the method name ending in <i>`Response`</i>.
See Section \ref param  on how to change the declaration if the service response element name is different.

With SOAP messaging the request and response XML messages are placed in the
<i>`SOAP-ENV:Envelope`</i> and <i>`SOAP-ENV:Body`</i> elements.  SOAP 1.1
document/literal messaging is the default messaging mode in gSOAP, which are
modified to SOAP or REST with <i>`//gsoap <prefix> service method-protocol:`</i>
directives, see Section \ref directives.

The soapcpp2 tool generates a client stub function for the service
operation. This stub is of the form:

~~~{.cpp}
    int soap_call_prefix__method_name(struct soap *soap, char *endpoint, char *action, inparam1, inparam2, ..., outparam);
~~~

This stub is called by a client application to perform the service operation
call.

The soapcpp2 tool generates a skeleton functions for the
service operation. The skeleton function called by `::soap_serve` is:

~~~{.cpp}
    int soap_serve_prefix__method_name(struct soap *soap);
~~~

which after deserializing the XML request message calls the `prefix__method_name` service operation defined by the service application and serializes the XML response message when the service operation returns `#SOAP_OK`.

Alternatively, <b>`soapcpp2 -j`</b> option <b>`-j`</b> or option <b>`-i`</b> generates a C++ client proxy class and a service class.  These classes have methods corresponding to the service operations, which on the client side can be invoked to invoke remote service operations and on the server side are implemented by the service application to execute the service operations.

🔝 [Back to table of contents](#)

### Service operation parameter passing        {#param}

The input parameters of a service operation must be passed by value or by
pointer.  Input parameters cannot be passed by reference.  Passing a pointer to
the data is preferred when the size of the data of the parameter is non trivial
such as values of primitive type.

The output parameter must be passed by pointer or by reference.

The input and output parameter types must be serializable, which means that
there are some limitations on the types of data that can be passed, see Section
\ref limitations .

If the output parameter is a pointer or reference to a `struct` or `class`
type, it is considered a service operation response element instead of a simple
output parameter value. That is, the name of the `struct` or `class` is the
name of the response element and the `struct` or `class` members are the output
parameters of the service operation, see also Section \ref response .
Therefore, if the output parameter has to be a `struct` or `class`, a response
`struct` or `class` must be declared to wrap that `struct` or `class` type
parameter.  Likewise, if a service operation returns multiple output parameters
then a response `struct` or `class` should be used to wrap the output
parameters.  By SOAP conventions, the response element is the service operation
name ending with "<i>`Response`</i>".

The general form of a response struct or class wrapper is:

~~~{.cpp}
    struct prefix__method_nameResponse 
    {
      outparam1; 
      outparam2; 
      ... //
      outparamn; 
    };
~~~

where

* `prefix__` is the optional namespace prefix of the response element.

* `response_element_name` it the name of the response element.

* `outparam1`, ..., `outparamn` are the output parameters of the service operation.

The general form of a service operation specification with a response element declaration is:

~~~{.cpp}
    int prefix__method_name(inparam1, inparam2, ..., inparamn, struct prefix__method_nameResponse { outparam1; outparam2; ...; outparamn; } *anyname);
~~~

The choice of name for `anyname` has no effect on the SOAP encoding and decoding and is only used as a place holder for the response.  In C++ this parameter can be passed by reference instead of by pointer.

The request message is:

<div class="alt">
~~~{.xml}
    <prefix:method-name> 
      <inparam1>...</inparam1> 
      <inparam2>...</inparam2> 
      ... 
      <inparamn>...</inparamn> 
    </prefix:method-name>
~~~
</div>

where the <i>`inparam1`</i>, ..., <i>`inparamn`</i> elements are the XML element representations of
the `inparam` parameters.

The response message is of the form:

<div class="alt">
~~~{.xml}
    <prefix:method-nameResponse> 
      <outparam1>...</outparam1> 
      <outparam2>...</outparam2> 
      ... 
      <outparamn>...</outparamn> 
    </prefix:method-nameResponse>
~~~
</div>

where the <i>`outparam1`</i>, ..., <i>`outparamn`</i> elements are the XML element representations of
the `outparam` parameters.

The input and output parameters can be made anonymous, which allows the
deserialization of requests/responses with different parameter names as is
endorsed by the SOAP 1.1 specification, see Section \ref anonymous .

🔝 [Back to table of contents](#)

## C/C++ identifier name to XML tag name translation        {#idtrans}

One of the nice aspects of gSOAP is its powerful C/C++ XML data binding and the
flexibility to specify names for XML, such as service operation names, class
names, type identifiers, and struct or class members. The first aspect is the
use of namespace prefixes with C/C++ names to qualify the names with XML
namespaces, which is specified with a `prefix__` or as we will see later can be
specified with a colon `prefix:` in the C/C++ name.  A C/C++ identifier name of
the form

~~~{.cpp}
    prefix__element_name
~~~

is be encoded in XML as

<div class="alt">
~~~{.xml}
    prefix:element-name
~~~
</div>

The underscore pair (`__`) separates the namespace prefix from the
element name.  Each namespace prefix has a namespace URI specified by a
`//gsoap <prefix> schema namespace: <URI>` directive that is saved to the soapcpp2-generated
namespace mapping table, see Sections \ref nstable and \ref namespace .  The namespace URI is a unique identification that
can be associated with the service operations and data types.  The namespace URI
disambiguates potentially identical service operation names and data type names
used by disparate organizations.

XML element names are XSD NCNames (non-colon names) that may contain
letters, digits, underscores, hyphens, dots, and other special characters except reserved characters and colon.
To add non-element names of service operations, structs, classes, typedefs, and members can be
A single underscore `_` in a C/C++
prefix or identifier name is replaced by a hyphen <i>`-`</i> in the
XML encoding.  For example, the identifier name `SOAP_ENC__ur_type`
is represented in XML as <i>`SOAP-ENC:ur-type`</i>.  A `_DOT` is
replaced by a dot <i>`.`</i> in XML, and `_USCORE` is replaced by
an underscore <i>`_`</i> in XML.  For example:

~~~{.cpp}
    class n_s__biz_DOTcom 
    {
      char * n_s__biz_USCOREname; 
    };
~~~

is serialized in XML as:

<div class="alt">
~~~{.xml}
    <n-s:biz.com> 
      <n-s:biz_name>Bizybiz</n-s:biz_name> 
    </n-s:biz.com>
~~~
</div>

Other special characters are added to C/C++ names as `_xHHHH` where `HHHH` is
the hexadecimal code of a Unicode character code point.

Trailing underscores in an identifier name are stripped from the XML encoding.
This is useful when an identifier name clashes with a C++
keyword. For example, <i>`return`</i> may be used as an XML element.
This <i>`return`</i> element can be specified as `return_`, for example as a struct or class member or function parameter.

By default the soapcpp2 tool generates data binding source code in which all
local XML elements are and attributes are unqualified:

~~~{.cpp}
    //gsoap x schema namespace: urn:x 
    struct x__record
    {
      @ char * type; // maps to unqualified type attribute
        char * name; // maps to unqualified name element
    };
~~~

where the `name` element and the `type` attribute are unqualified in the XML content (for example to facilitate SOAP RPC encoding).

To force qualification of elements and attributes, use the "form" directive:

~~~{.cpp}
    //gsoap x schema namespace: urn:x 
    //gsoap x schema form: qualified 
    struct x__record 
    {
      @ char * type; // maps to qualified x:type attribute
        char * name; // maps to qualified x:name element
    };
~~~

You can also use "elementForm" and "attributeForm" directives to (un)qualify local element and attributes, respectively.

Because the soapcpp2-generated serializers follow the
qualified/unqualified forms of the schemas, there is normally no need to
explicitly qualify struct/class members because automatic encoding rules will
be used.

If explicit qualification is needed, this can be done using the prefix convention:

~~~{.cpp}
    //gsoap x schema namespace: urn:x 
    //gsoap y schema namespace: urn:y 
    struct x__record 
    {
      @ char * xsi__type; // maps to qualified xsi:type attribute
        char * y__name;   // maps to qualified y:name element
    };
~~~

which ensures that there cannot be any name clashes between members of the same name defined in different schemas (consider for example `name` and `y__name`), but this can clutter the representation when clashes do not occur.

An alternative to the prefix convention is the use of "colon notation" in the
interface header file for soapcpp2. This extra addition to the the C/C++ syntax allows you to bind
type names and struct and class members to qualified and unqualified XML tag names explicitly,
thus bypassing the default mechanism that automatically qualifies or
unqualifies element and attribute tag names based on the schema
element or attribute forms.

The colon notation for type names, struct and class names, and members overrides the prefix qualification rules explicitly:

~~~{.cpp}
    //gsoap x schema namespace: urn:x 
    //gsoap y schema namespace: urn:y 
    struct x:record 
    {
      @ char * xsi:type; // maps to qualified xsi:type attribute
        char * y:name;   // maps to qualified y:name element
    };
~~~

where `x` and `y` are namespace prefixes that are declared with a directive. The `xsi:type` member is an XML attribute in the `xsi` namespace.
The soapcpp2 tool generates data binding implementation source code with the following cleaned-up struct without the annotations:

~~~{.cpp}
    // This code is generated in soapStub.h: 
    struct record 
    {
        char * type; /* optional attribute of type xsd:string */ 
        char * name; /* optional element of type xsd:string */ 
    };
~~~

The soapcpp2 tool also generates XML schemas with element and attribute
references. That is, `y:name` is referenced from the `y` schema by the
<i>`x:record`</i> complexType defined in the `x` schema.

The colon notation also allows you to override the element and attribute forms to unqualified for qualified schemas:

~~~{.cpp}
    //gsoap x schema namespace: urn:x 
    //gsoap x schema form: qualified 
    struct x:record 
    {
      @ char * :type; // maps to unqualified type attribute
        char * :name; // maps to unqualified name element
    };
~~~

where the colon notation ensures that both `type` and `name` are
unqualified in the XML content, which overrides the default qualified forms of
the `x` schema.

Note that the use of colon notation to bind namespace prefixes to type names
(typedef, enum, struct, and class names) translates to code without the
prefixes. This means that name clashes can occur between types with identical unqualified names:

~~~{.cpp}
    enum x:color { RED, WHITE, BLUE }; 
    enum y:color { YELLOW, ORANGE }; // illegal enum name: name clash with x:color
~~~

while prefixing with double underscores never lead to clashes:

~~~{.cpp}
    enum x__color { RED, WHITE, BLUE }; 
    enum y__color { YELLOW, ORANGE }; // no name clash
~~~

Also note that colon notation has a very different role than the C++ scope
operator `::`. The scope operator cannot be used in places where we need
colon notation, such as struct and class member members.

The default mechanism that associates XML tag names with the names of struct
and class member members can be overridden by "re-tagging" names with the
annotation of a tag placed next to the member member name. This is particularly useful to support legacy code for which the fixed naming of member members cannot be easily changed. For example:

~~~{.cpp}
    //gsoap x schema namespace: urn:x 
    //gsoap x schema form: qualified 
    struct x:record 
    {
      @ char * t `:type`; // maps to unqualified type attribute
        char * s `name`;  // maps to qualified x:name element
    };
~~~

This maps the `t` member to the <i>`type`</i> XML attribute tag and `s`
member to the <i>`x:name`</i> XML element tag. Tags will be namespace
qualified as per schema element and attribute forms, unless preceded by a colon.

As of gSOAP 2.8.23 and greater, Unicode characters in C/C++ identifiers are
accepted by soapcpp2 when the source file is encoded in UTF-8. C/C++ Unicode
names are mapped to Unicode XML tags. For C/C++ source code portability
reasons, the wsdl2h tool still converts Unicode XML tag names to ASCII C/C++
identifiers using the `_xHHHH` naming convention for `HHHH` character code
points. Use [<b>`wsdl2h -U`</b> option <b>`-U`</b>](#wsdl2h-U) to map Unicode letters in XML
tag names to UTF-8-encoded Unicode letters in C/C++ identifiers.

🔝 [Back to table of contents](#)

## Generating a SOAP/XML client application with soapcpp2  {#compilingclient}

After invoking the soapcpp2 tool on an interface header file description of a service to generate <i>`soapStub`</i>, <i>`soapH.h`</i>, and <i>`soapC.cpp`</i> for the XML serializers, and <i>`soapClient.cpp`</i> for the client stub functions, the client application is compiled in C++ as follows:

     c++ -o myclient myclient.cpp stdsoap2.cpp soapC.cpp soapClient.cpp

For C we use <b>`soapcpp2 -c`</b> option <b>`-c`</b> to generate C source code that is compiled with:

     cc -o myclient myclient.c stdsoap2.c soapC.c soapClient.c

Depending on your system configuration, such as with Unix, linking with <b>`-lsocket`</b>,
<b>`-lxnet`</b>, and <b>`-lnsl`</b> may be required.

The <i>`myclient.cpp`</i> file should include <i>`soapH.h`</i> and must include or define a global namespace mapping table, unless `#WITH_NONAMESPACES` is used.

For examples of SOAP and REST client applications, see <i>`gsoap/samples`</i> in the gSOAP source code package.
The online getting-started guide covers example client and server applications in C and C++, visit <https://www.genivia.com/dev.html> to read more.  Various examples ranging from simple calculator service APIs to very large protocols spanning dozens of WSDLs can be found at <https://www.genivia.com/examples.html>

To test client applications using an auto-generated echo test server, use <b>`soapcpp2 -T`</b> option <b>`-T`</b>, see the next section. You can also test a client application with the gSOAP [Test Messenger](../../testmsgr/html/index.html).

🔝 [Back to table of contents](#)

## Generating a SOAP/XML Web service application with soapcpp2   {#compilingservice}

After invoking the soapcpp2 tool on an interface header file description of a service to generate <i>`soapStub`</i>, <i>`soapH.h`</i>, and <i>`soapC.cpp`</i> for the XML serializers, and <i>`soapServer.cpp`</i> for the server skeleton functions, the service application is compiled in C++ as follows:

     c++ -o myserver myserver.cpp stdsoap2.cpp soapC.cpp soapServer.cpp

For C we use <b>`soapcpp2 -c`</b> option <b>`-c`</b> to generate C source code that is compiled with:

     cc -o myserver myserver.c stdsoap2.c soapC.c soapServer.c

Depending on your system configuration, such as with Unix, linking with <b>`-lsocket`</b>,
<b>`-lxnet`</b>, and <b>`-lnsl`</b> may be required.

The <i>`myserver.cpp`</i> file should include <i>`soapH.h`</i> and should include or define a global namespace mapping table, unless `#WITH_NONAMESPACES` is used.

A gSOAP service can be installed as:

* A simple stateless CGI application where the application just calls `::soap_serve` to serve requests on standard input and output.

* A FastCGI application, see Section \ref fastcgi.

* A stand-alone server, see Section \ref stand-alone.

* A stand-alone multi-threaded server, see Section \ref mt.

* An Apache module, see the gSOAP [Apache module](../../apache/html/index.html) documentation.

* An ISAPI extension running in IIS, see the gSOAP [ISAPI extension](../../isapi/html/index.html) documentation.

To test a service, see the gSOAP [Test Messenger](../../testmsgr/html/index.html).

Furthermore, an echo test server application <i>`soapTester.cpp`</i> is generated with <b>`soapcpp2 -T`</b> option <b>`-T`</b>, which is a stand-alone iterative test server that echos SOAP/XML requests and runs on the specified port.  Compile this with:

     c++ -o testserver soapTester.cpp stdsoap2.cpp soapC.cpp soapServer.cpp

Then run on a port, say 8080:

     ./testServer 12288 8080

The 12288 value is a combination of the `#SOAP_XML_INDENT` (0x2000) and `#SOAP_XML_STRICT` (0x1000) integer flag values (8192 + 4096 = 12288).

For examples of SOAP and REST Web service applications, see <i>`gsoap/samples`</i> in the gSOAP source code package.
The online getting-started guide covers example client and server applications in C and C++, visit <https://www.genivia.com/dev.html> to read more.  Various examples ranging from simple calculator service APIs to very large protocols spanning dozens of WSDLs can be found at <https://www.genivia.com/examples.html>

🔝 [Back to table of contents](#)

## Generating an auto test server for client testing   {#autotest}

The <b>`soapcpp2 -T`</b> option <b>`-T`</b> generates an echo test server
application source code <i>`soapTester.cpp`</i>, which is to be compiled and
linked with the code generated for a server implementation
<i>`soapServer.cpp`</i> (or with the generated service class file) and
<i>`soapC.cpp`</i>. The feature also supports C source code, use the
<b>`soapcpp2 -c -T`</b> options <b>`-c`</b> and <b>`-T`</b> to generate a C
test server.

The echo test server can be used to test a client application, by the client
sending messages to the echo test server that echos responses back to the
client.  These responses are structurally valid but may lack sufficient details
to consider the response messages useful.

The generated source code is compiled with:

    c++ -o tester soapTester.cpp soapServer.cpp soapC.cpp stdsoap2.cpp

To run the <b>`tester`</b> auto-test service on a port to test a client
against, use two command-line arguments: the first argument is a combined
integer of OR-ed values of the context flags such as 12288 which is a
combination of `#SOAP_XML_INDENT` (0x1000 = 4096) and `#SOAP_XML_STRICT`
(0x1000 = 8196) and the second argument is the port number:

     ./tester 12288 8080

This starts an iterative stand-alone server on port 8080. Messages can be sent
to <i>`http://localhost:8080`</i> to test a client application against the echo
test server. The data in the response messages are copied from the request
messages when possible, or XML default values, or empty otherwise.

More advanced servers for testing are available with the gSOAP Test Messenger
[testmsgr](../../testmsgr/html/index.html) tool to test SOAP and REST XML
clients and servers.

🔝 [Back to table of contents](#)

## Generating deep copy and deletion functions   {#deep}

The [<b>`soapcpp2 -Ec`</b> option <b>`-Ec`</b>](#soapcpp2-E) generates deep copy code for each
serializable type `T` declared in an interface header file for soapcpp2.  The
[<b>`soapcpp2 -Ed`</b> option <b>`-Ed`</b>](#soapcpp2-E) generates deep
deletion code.

For a serializable type `T` declared in the interface header file for soapcpp2,
option <b>`-Ec`</b> generates:

- `virtual T * T::soap_dup(struct soap*) const` where `T` is a class,
  returns a duplicate of this object by deep copying, replicating all deep
  cycles and shared pointers when a managing `soap` context is provided as
  argument.  Deep copy is a tree when argument is NULL, but the presence of
  deep cycles will lead to non-termination.  Use flag `SOAP_XML_TREE` with the
  managing context to copy into a tree without cycles and pointers to shared
  objects.

- `T * soap_dup_T(struct soap*, T *dst, const T *src)` where `T` is not a class,
  deep copy `src` into `dst`, replicating all deep cycles and shared pointers
  when a managing `soap` context is provided as argument.  When `dst` is NULL,
  allocates space for `dst` and returns a pointer to the allocated copy.  Deep
  copy results in a tree when the `soap` context is NULL, but the presence of
  deep cycles will lead to non-termination.  Use flag `SOAP_XML_TREE` with
  managing context to copy into a tree without cycles and pointers to shared
  objects.  Returns `dst` or allocated copy when `dst` is NULL.

For a serializable type `T` declared in the interface header file for soapcpp2,
option <b>`-Ed`</b> generates:

- `virtual void T::soap_del() const` where `T` is a class, deletes all
  heap-allocated members of this object by deep deletion ONLY IF this object
  and all of its (deep) members are not managed by a `soap` context AND the deep
  structure is a tree (no cycles and co-referenced objects by way of multiple
  (non-smart) pointers pointing to the same data).  Can be safely used after
  `T::soap_dup(NULL)` to delete the deep copy.  Does not delete the object
  itself.

- `void soap_del_T(const T*)` where `T` is not a class, deletes all
  heap-allocated members of this object by deep deletion ONLY IF this object
  and all of its (deep) members are not managed by a `soap` context AND the deep
  structure is a tree (no cycles and co-referenced objects by way of multiple
  (non-smart) pointers pointing to the same data).  Can be safely used after
  `soap_dup_T(NULL, NULL, const T*)` to delete the deep copy returned.  Does
  not delete the object itself.

For example:

~~~{.cpp}
    #include "soapH.h"
    #include "ns.nsmap"

    int main()
    {
      struct soap *soap = soap_new();
      ns__record record;                            // a serializable type
      ns__record rec_dup;
      soap->recvfd = open(file, O_RDONLY); 
      if (soap->recvfd < 0)
        ... // error
      if (soap_read_ns__record(soap, &record))      // deserialize from file into managed memory
        ... // error
      close(soap->recvfd);
      soap_dup_ns__record(NULL, &rec_dup, &record); // deep copy the record to unmanaged memory
      soap_destroy(soap);                           // delete managed objects
      soap_end(soap);                               // delete managed data and temporaries
      soap_free(soap);                              // free the context
      soap_del_ns__record(&rec_dup)                 // deep delete unmanaged record
    }
~~~

🔝 [Back to table of contents](#)

# Serialization and deserialization rules   {#rules}

This section describes the serialization and deserialization of C and C++ data types in XML, and therefore by implication in SOAP 1.1 and 1.2.  First, the difference between SOAP RPC encoding and document/literal style is explained and how to switch between SOAP 1.1 and 1.2 or support both in applications. Then the general XML representations of C/C++ data in XML and XML schema is explained.

To obtain more information about the code generated by soapcpp2 for the data types specified in an interface header file for soapcpp2, use <b>`soapcpp2 -r`</b> option <b>`-r`</b> to generate a <i>`soapReadme.md`</i> report with all the details.

For additional details on serialization of data types in XML, see the [C and C++ XML Data Bindings](../../databinding/html/index.html) documentation.

🔝 [Back to table of contents](#)

## SOAP RPC encoding versus document/literal style messaging {#rpcversusdoclit}

The serialization and deserialization rules
is almost identical for these two different styles, except for the
following:

* With SOAP RPC encoding, generic <i>`complexTypes`</i> with <i>`maxOccurs="unbounded"`</i> are not allowed and SOAP-encoded arrays must be used instead.

* XML attributes and unions (XML schema <i>`choice`</i>) are not allowed with SOAP RPC encoding.

* In XML messages conforming to SOAP RPC encoding we often see <i>`xsi:type`</i> attributed messages although the <i>`xsi:type`</i> attribute is not required and the gSOAP engine does not produce <i>`xsi:type`</i> attributes unless required, e.g. to identify derived classes from base classes serialized.  Use <b>`soapcpp2 -t`</b> option <b>`-t`</b> to force <i>`xsi:type`</i> attributes in the XML output.

* In XML messages conforming to SOAP RPC encoding, multi-reference accessors using id-href/ref attributes are common to encode
co-referenced data. By contrast, multi-referenced data is not
accurately represented in document/literal style, which means that data structure graphs
cannot be accurately serialized.  Document/literal is strictly "tree shaped".

The soapcpp2 tool uses SOAP 1.1 document/literal style by default. 
Use the `//gsoap` directives to control the SOAP protocol version and messaging style,
see Section \ref directives, or use <b>`soapcpp2`</b> options <b>`-2`</b> (SOAP 1.2), <b>`-e`</b> (encoding style), and
<b>`-t`</b> (add  <i>`xsi:type`</i> attributes).

🔝 [Back to table of contents](#)

## SOAP 1.1 versus SOAP 1.2 and dynamic switching  {#switching}

SOAP 1.1 is the default protocol. SOAP 1.2 support is automatically turned on
when the appropriate SOAP 1.2 namespace is used in the WSDL input to wsdl2h,
which then generates `#import "soap12.h"` in the interface header file:

~~~{.cpp}
    #import "soap12.h"
~~~

This interface header file when input to soapcpp2 results in a XML namespace
mapping table with SOAP 1.2 namespaces:

~~~{.cpp}
    struct Namespace namespaces[] = 
    {
      { "SOAP-ENV", "http://www.w3.org/2003/05/soap-envelope" }, 
      { "SOAP-ENC", "http://www.w3.org/2003/05/soap-encoding" }, 
      ... //
      { NULL, NULL }
    }
~~~

The soapcpp2-generated default SOAP 1.1 namespace table allow for dynamic
switching between SOAP 1.1 to SOAP 1.2 by providing the SOAP 1.2 namespace as a
pattern in the third column of a namespace table:

~~~{.cpp}
    struct Namespace namespaces[] = 
    {
      { "SOAP-ENV", "http://schemas.xmlsoap.org/soap/envelope/", "http://www.w3.org/*/soap-encoding" }, 
      { "SOAP-ENC", "http://schemas.xmlsoap.org/soap/encoding/", "http://www.w3.org/*/soap-envelope" }, 
      ... //
      { NULL, NULL }
    }
~~~

where the `*` in the third column of the namespace URI pattern is a wildcard
for any sequence of character. This is used to match inbound <i>`xmlns`</i> XML
namespace bindings that are then associated with the prefix in the table.  For
example, when the inbound XML contains a
<i>`xmlsn:soap="http://www.w3.org/2003/05/soap-envelope"`</i> binding then the
<i>`soap`</i> prefix used in the inbound XML is actually equivalent to
<i>`SOAP-ENV`</i> used in gSOAP as determined by the matching pattern in the
third column of the XML namespace table shown above.

In this way, gSOAP Web services can respond to both SOAP 1.1 or SOAP 1.2
requests.  Moreover, the gSOAP engine will automatically return a SOAP 1.2
message response to a SOAP 1.2 message request when the XML namespace table
shown above is used.  This works by using the specified pattern in the third
column, when it matches the namespace URI of the inbound XML request message of
course.  However, the use of SOAP 1.1 or 1.2 is overridden for one or more
service operations with the `//gsoap <prefix> service method-protocol:` directive.

A gSOAP client that sends a request message will always send it using the SOAP
protocol specified by the namespace in the second column, unless this is
overridden with a `//gsoap <prefix> service method-protocol:` directive.

To make the XML namespace table available to the developer, the soapcpp2 tool
generates a <i>`.nsmap`</i> file with the `SOAP-ENV` and `SOAP-ENC` namespaces
and patterns as shown in the example above.

To use SOAP 1.2 by default and accept SOAP 1.1 messages to be received, use the
<b>`soapcpp2 -2`</b> option <b>`-2`</b> to generate SOAP 1.2 <i>`.nsmap`</i>
and <i>`.wsdl`</i> files. Alternatively, add the following line to your
interface header file (generated by wsdl2h) for soapcpp2:

~~~{.cpp}
    #import "soap12.h"
~~~

The <i>`soap12.h`</i> file is located in <i>`gsoap/import`</i>.

@warning SOAP 1.2 does not support SOAP "partially transmitted arrays". So the `__offset` member of a dynamic array is meaningless in SOAP 1.2.

@warning SOAP 1.2 uses `::SOAP_ENV__Code`, `::SOAP_ENV__Reason`, and `::SOAP_ENV__Detail` members
of a `::SOAP_ENV__Fault` fault struct, while SOAP 1.1 uses `faultcode`, `faultstring`, and `detail` members.

🔝 [Back to table of contents](#)

## Primitive type serialization  {#primtype}

The default encoding rules for primitive C and C++ data types in XML are given in the table below:

C/C++ type                                       | XSD type
------------------------------------------------ | ---------------
`bool`                                           | `xsd:boolean`
`char` and `int8_t`                              | `xsd:byte`
`short` and `int16_t`                            | `xsd:short`
`int`, `long`, and `int32_t`                     | `xsd:int`
`LONG64`, `long long` and `int64_t`              | `xsd:long`
`unsigned char` and `uint8_t`                    | `xsd:unsignedByte`
`unsigned short` and `uint16_t`                  | `xsd:unsignedShort`
`unsigned int`, `unsigned long` and `uint32_t`   | `xsd:unsignedInt`
`ULONG64`, `long long`, and `uint64_t`           | `xsd:unsignedLong`
`float`                                          | `xsd:float`
`double`                                         | `xsd:double`
`long double`                                    | `xsd:decimal` with `#import "custom/long_double.h"`
`time_t`                                         | `xsd:dateTime`
`struct tm`                                      | `xsd:dateTime` with `#import "custom/struct_tm.h"` 
`struct timeval`                                 | `xsd:dateTime` with `#import "custom/struct_timeval.h"`
`char*`, `const char*`, and `std::string`        | `xsd:string`
`wchar_t*`, `const wchar_t*`, and `std::wstring` | `xsd:string`

Enumerations and bit masks are also supported, see Section \ref enum .

Custom serializers for `long double`, `struct tm`, and `struct timeval` and
many other specialized C and C++ types are available, see the
[C and C++ XML Data Bindings](../../databinding/html/index.html) documentation
for details.

The previous table shows how C/C++ primitive types are mapped to XSD types.  To
define and use the full range of XSD types is done with typedefs to define
namespace-qualified types in C/C++ corresponding to the XSD types (or to any
schema type for that matter).

XSD types such as <i>`xsd:positiveInteger`</i>, <i>`xsd:anyURI`</i>, and
<i>`xsd:date`</i> for which no built-in data structures in C and C++ exist can
always be represented by strings and some can be represented by integers or
floats.  Validation constraints can be added to validate the XSD type values as
explained further below.

A `typedef` in an interface header file for soapcpp2 declares a schema type
name.  The soapcpp2 tool interprets `typedef` declarations the same way as a
regular C compiler interprets them. However, the soapcpp2 tool also uses the
type name when generating WSDLs and XSD files and in <i>`xsi:type`</i>
attributes when present. 

For example, the declaration:

~~~{.cpp}
    typedef uint64_t xsd__positiveInteger;
~~~

creates a named type `xsd__positiveInteger` represented by `ulong64_t` and
serialized as XSD type <i>`xsd:positiveInteger`</i>.

The built-in primitive and derived numerical XSD types are listed below
together with their recommended `typedef` declarations. Note that the SOAP
encoding schemas for primitive types are derived from the built-in XML Schema
types, so `SOAP_ENC__` can be used as a namespace prefix instead of `xsd__`.
However, the use of <i>`SOAP_ENC`</i> XML types is obsolete and redundant
because XSD primitive types can be used instead.

Other XSD types not mentioned in this section, such as <i>`gYearMonth`</i>,
<i>`gYear`</i>, <i>`gMonthDay`</i>, <i>`gDay`</i>, <i>`xsd:gMonth`</i>,
<i>`QName`</i>, <i>`NOTATION`</i>, etc., can be encoded similarly using a
`typedef` declaration with a string type.

For additional in-depth details, see the [C and C++ XML Data Bindings](../../databinding/html/index.html) documentation.

### <i>`xsd:anyURI`</i>
Represents a Uniform Resource Identifier Reference (URI).
Each URI scheme imposes specialized syntax rules for URIs in that scheme, including restrictions
on the syntax of allowed fragment identifiers.
It is recommended to use strings to store <i>`xsd:anyURI`</i> XML Schema types. The recommended type declaration is:
~~~{.cpp}
    typedef char *xsd__anyURI;
~~~
or
~~~{.cpp}
    typedef std::string xsd__anyURI;
~~~

### <i>`xsd:base64Binary`</i>
Represents Base64-encoded arbitrary binary data.
For using the <i>`xsd:base64Binary`</i> XSD type, the use of the base64Binary representation of a dynamic array is strongly recommended,
see Section \ref base64binary . However, the
type can also be declared as a string and the encoding will be string-based:
~~~{.cpp}
    typedef char *xsd__base64Binary;
~~~
or
~~~{.cpp}
    typedef std::string xsd__base64Binary;
~~~
However, it is the responsibility of the application to make sure the string content is according to the Base64 Content-Transfer-Encoding defined in Section 6.8 of RFC 2045.
Better is to use the base64 serializer that serializes binary data as <i>`xsd:base64Binary`</i>:
~~~{.cpp}
    struct xsd__base64Binary
    {
        unsigned char *__ptr; // point to data to serialize
        int __size;           // length of the data to serialize
    };
~~~

### <i>`xsd:boolean`</i>
For declaring an <i>`xsd:boolean`</i> XSD type, the use of a bool is recommended in C++.
For C, see Section \ref boolean .
The corresponding type declaration is:
~~~{.cpp}
    typedef bool xsd__boolean;
~~~

### <i>`xsd:byte`</i>
Represents a byte (-128...127). The corresponding type declaration is:
~~~{.cpp}
    typedef char xsd__byte;
~~~

### <i>`xsd:dateTime`</i>
Represents a date and time. The lexical representation is according to the ISO 8601 extended format CCYY-MM-DDThh:mm:ss where "CC"
represents the century, "YY" the year, "MM" the month and "DD" the day, preceded by an optional leading "-" sign to indicate a
negative number. If the sign is omitted, "+" is assumed. The letter "T" is the date/time separator and "hh", "mm", "ss" represent
hour, minute and second respectively.
It is recommended to use the `time_t` type to store <i>`xsd:dateTime`</i> XSD types and the type declaration is:
~~~{.cpp}
    typedef time_t xsd__dateTime;
~~~
However, note that calendar times  before  the year 1902  or after
the year 2037 cannot be represented. Upon receiving a date outside this range,
the `time_t` value will be set to -1.
Also strings can be used to store <i>`xsd:dateTime`</i> types:
~~~{.cpp}
    typedef char *xsd__dateTime;
~~~
Best is to use a custom serializer `struct tm`, `struct timeval`, or `std::chrono::system_clock::time_point` defined by <i>`gsoap/custom/struct_tm.h`</i>, <i>`gsoap/custom/struct_timeval.h`</i>, and <i>`gsoap/custom/chrono_timepoint.h`</i> to represent <i>`xsd:dateTime`</i> accurately.

### <i>`xsd:date`</i>
Represents a date.
The lexical representation for date is the reduced (right truncated) lexical representation for dateTime: CCYY-MM-DD.
It is recommended to use strings (`char*`) to store <i>`xsd:date`</i> XSD types. The type declaration is:
~~~{.cpp}
    typedef char *xsd__date;
~~~
Best is to use a custom serializer `struct tm` defined by <i>`gsoap/custom/struct_tm_date.h`</i> to represent <i>`xsd:date`</i> accurately.

### <i>`xsd:decimal`</i>
Represents arbitrary precision decimal numbers.
It is recommended to use the {double} type to store <i>`xsd:decimal`</i> XSD types and the type declaration is:
~~~{.cpp}
    typedef double xsd__decimal;
~~~
Better is to use a custom serializer <i>`gsoap/custom/long_double.h`</i> to represent <i>`xsd:decimal`</i> or a string to avoid losing accuracy of very large numbers.

### <i>`xsd:double`</i>
Corresponds to the IEEE double-precision 64-bit floating point type. The type declaration is:
~~~{.cpp}
    typedef double xsd__double;
~~~

### <i>`xsd:duration`</i>
Represents a duration of time.
The lexical representation for duration is the ISO 8601 extended format PnYn MnDTnH nMnS, where nY represents
the number of years, nM the number of months, nD the number of days, T is the date/time separator, nH the number of
hours, nM the number of minutes and nS the number of seconds. The number of seconds can include decimal digits to
arbitrary precision.
It is recommended to use strings (`char*`) to store <i>`xsd:duration`</i> XSD types. The type declaration is:
~~~{.cpp}
    typedef char *xsd__duration;
~~~
Better is to use a custom serializer <i>`gsoap/custom/duration.h`</i> or <i>`gsoap/custom/chrono_duration.h`</i> to represent <i>`xsd:duration`</i> or a string to avoid losing accuracy of very large numbers.

### <i>`xsd:float`</i>
Corresponds to the IEEE single-precision 32-bit floating point type. The type declaration is:
~~~{.cpp}
    typedef float xsd__float;
~~~

### <i>`xsd:hexBinary`</i>
Represents arbitrary hex-encoded binary data.  It has a lexical representation where each binary octet is encoded as a character
tuple, consisting of two hexadecimal digits ([0-9a-fA-F]) representing the octet code. For example, "0FB7" is a hex encoding for
the 16-bit integer 4023 (binary representation is 111110110111.
For using the <i>`xsd:hexBinary`</i> XSD type, the use of the hexBinary representation of a dynamic array is strongly recommended,
see Section \ref hexbinary . However, the
type can also be declared as a string and the encoding will be string-based:
~~~{.cpp}
    typedef char *xsd__hexBinary;
~~~
or
~~~{.cpp}
    typedef std::string xsd__hexBinary;
~~~
However, it is the responsibility of the application to make sure the string content is hex formatted.
Better is to use the hex serializer that serializes binary data as <i>`xsd:hexBinary`</i>:
~~~{.cpp}
    struct xsd__hexBinary
    {
        unsigned char *__ptr; // point to data to serialize
        int __size;           // length of the data to serialize
    };
~~~

### <i>`xsd:int`</i>
Corresponds to a 32-bit integer in the range -2147483648 to 2147483647.
~~~{.cpp}
    typedef int xsd__int;
~~~

### <i>`xsd:integer`</i>
Corresponds to an unbounded integer.
C/C++ does not support unbounded integers as a standard feature. The recommended type declaration is:
~~~{.cpp}
    typedef int64_t xsd__integer;
~~~
Another possibility is to use strings to represent unbounded integers and do the translation in the application itself.

### <i>`xsd:long`</i>
Corresponds to a 64-bit integer in the range -9223372036854775808 to 9223372036854775807.
The type declaration is:
~~~{.cpp}
    typedef int64_t xsd__long;
~~~

### <i>`xsd:negativeInteger`</i>
Corresponds to a negative unbounded integer.
C/C++ does not support unbounded integers as a standard feature. The recommended type declaration is:
~~~{.cpp}
    typedef int64_t xsd__negativeInteger : -1 ;
~~~
Another possibility is to use strings to represent unbounded integers and do the translation in the application itself.

### <i>`xsd:nonNegativeInteger`</i>
Corresponds to a non-negative unbounded integer.
Since C++ does not support unbounded integers as a standard feature, the recommended type declaration is:
~~~{.cpp}
    typedef uint64_t xsd__nonNegativeInteger 0 : ;
~~~
Another possibility is to use strings to represent unbounded integers and do the translation in the application itself.

### <i>`xsd:nonPositiveInteger`</i>
Corresponds to a non-positive unbounded integer.
Since C++ does not support unbounded integers as a standard feature, the recommended type declaration is:
~~~{.cpp}
    typedef int64_t xsd__nonPositiveInteger : 0 ;
~~~
Another possibility is to use strings to represent unbounded integers and do the translation in code.

### <i>`xsd:normalizedString`</i>
Represents normalized character strings.
Normalized character strings do not contain the carriage return (#xD), line feed (#xA) nor tab (#x9) characters.
It is recommended to use strings to store <i>`xsd:normalizedString`</i> XSD types.
The type declaration is:
~~~{.cpp}
    typedef char *xsd__normalizedString;
~~~
or
~~~{.cpp}
    typedef std::string xsd__normalizedString;
~~~

### <i>`xsd:positiveInteger`</i>
Corresponds to a positive unbounded integer.
C/C++ does not support unbounded integers as a standard feature. The recommended type declaration is:
~~~{.cpp}
    typedef uint64_t xsd__positiveInteger 1 : ;
~~~
Another possibility is to use strings to represent unbounded integers and do the translation in the application itself.

### <i>`xsd:short`</i>
Corresponds to a 16-bit integer in the range -32768 to 32767.
The type declaration is:
~~~{.cpp}
    typedef short xsd__short;
~~~

### <i>`xsd:string`</i>
Represents character strings. The type declaration is:
~~~{.cpp}
    typedef char *xsd__string;
~~~
or
~~~{.cpp}
    typedef std::string xsd__string;
~~~
The type declaration for wide character strings is:
~~~{.cpp}
    typedef wchar_t *xsd__string;
~~~
or
~~~{.cpp}
    typedef std::wstring xsd__string;
~~~
Both types of regular and wide strings can be used at the same time, by using a typedef name with a trailing underscore as follows:
~~~{.cpp}
    typedef wchar_t *xsd__string_;
~~~
or
~~~{.cpp}
    typedef std::wstring xsd__string_;
~~~

### <i>`xsd:time`</i>
Represents a time.  The lexical representation for time is the left truncated lexical representation for dateTime: hh:mm:ss.sss
with optional following time zone indicator.
It is recommended to use strings (`char*`) to store <i>`xsd:time`</i> XSD types. The type declaration is:
~~~{.cpp}
    typedef char *xsd__time;
~~~
or
~~~{.cpp}
    typedef std::string xsd__time;
~~~
Better is to use a custom serializer <i>`gsoap/custom/long_time.h`</i> to represent <i>`xsd:time`</i> or a string to avoid losing accuracy.

### <i>`xsd:token`</i>
Represents tokenized strings.
Tokens are strings that do not contain the
line feed (#xA) nor tab (#x9) characters, that have no leading or trailing spaces (#x20) and that have no internal
sequences of two or more spaces.
It is recommended to use strings to store <i>`xsd:token`</i> XSD types.
The type declaration is:
~~~{.cpp}
    typedef char *xsd__token;
~~~

### <i>`xsd:unsignedByte`</i>
Corresponds to an 8-bit unsigned integer in the range 0 to 255.
The type declaration is:
~~~{.cpp}
    typedef uint8_t xsd__unsignedByte;
~~~

### <i>`xsd:unsignedInt`</i>
Corresponds to a 32-bit unsigned integer in the range 0 to 4294967295.
The type declaration is:
~~~{.cpp}
    typedef uint32_t xsd__unsignedInt;
~~~

### <i>`xsd:unsignedLong`</i>
Corresponds to a 64-bit unsigned integer in the range 0 to 18446744073709551615.
The type declaration is:
~~~{.cpp}
    typedef uint64_t xsd__unsignedLong;
~~~

### <i>`xsd:unsignedShort`</i>
Corresponds to a 16-bit unsigned integer in the range 0 to 65535.
The type declaration is:
~~~{.cpp}
    typedef uint16_t xsd__unsignedShort;
~~~

🔝 [Back to table of contents](#)

### How to use multiple C/C++ types for a single primitive XSD type  {#multiprim}

As explained in Section \ref idtrans, trailing underscores in a type name are
not relevant in XML and in the XML schemas generated by soapcpp2.  Therefore,
we can map multiple C/C++ types to XSD types (or any XML schema type).  For
example, the following declaration in the interface header file for soapcpp2
permits us to use regular strings and wide strings while mapping these both to
the XSD <i>`xsd:string`</i> type:

~~~{.cpp}
    typedef char *xsd__string; 
    typedef wchar_t *xsd__string_;
~~~

🔝 [Back to table of contents](#)

### How to use C++ wrapper classes to specify polymorphic primitive types        {#primclass}

XSD schema types form a hierarchy of types, with <i>`xsd:anyType`</i> at the
root.  A container or array of <i>`xsd:anyType`</i> may actually contain any
mix of types, i.e. this container or array is polymorphic.

On the one hand, the `typedef` construct provides a convenient way to
associate existing C/C++ types with XML schema types and makes it easy to
incorporate these types in a (legacy) C/C++ application without having to
replace application types in the source code.  On the other hand the `typedef`
declarations cannot be used to support polymorphic types.

To create a derivable primitive type `T`, a wrapper class is declared as follows:

~~~{.cpp}
    class prefix__type_name : public xsd__super_type_name
    { public:
        T __item; 
        ... // other members, see note below
    };
~~~

where `T` is a primitive C/C++ type. The `__item` member must be the first
member of the wrapper class and all other members are not serialized.

For example, the a large portion of the XML type hierarchy can be implemented in C++ as follows:

~~~{.cpp}
    class xsd__anyType                                            { }; 
    class xsd__anySimpleType : public xsd__anyType                { }; 
    typedef char *xsd__string; 
    class xsd__string_ : public xsd__anySimpleType                { public: xsd__string __item; }; 
    typedef xsd__string xsd__anyURI; 
    class xsd__anyURI_ : public xsd__anySimpleType                { public: xsd__anyURI __item; }; 
    typedef bool xsd__boolean; 
    class xsd__boolean_ : public xsd__anySimpleType               { public: xsd__boolean __item; }; 
    typedef xsd__string xsd__date; 
    class xsd__date_ : public xsd__anySimpleType                  { public: xsd__date __item; }; 
    typedef time_t xsd__dateTime; 
    class xsd__dateTime_ : public xsd__anySimpleType              { public: xsd__dateTime __item; }; 
    typedef double xsd__double; 
    class xsd__double_ : public xsd__anySimpleType                { public: xsd__double __item; }; 
    typedef xsd__string xsd__duration; 
    class xsd__duration_ : public xsd__anySimpleType              { public: xsd__duration __item; }; 
    typedef float xsd__float; 
    class xsd__float_ : public xsd__anySimpleType                 { public: xsd__float __item; }; 
    typedef xsd__string xsd__time; 
    class xsd__time_ : public xsd__anySimpleType                  { public: xsd__time __item; }; 
    typedef xsd__string xsd__decimal; 
    class xsd__decimal_ : public xsd__anySimpleType               { public: xsd__decimal __item; }; 
    typedef xsd__string xsd__integer; 
    class xsd__integer_ : public xsd__decimal_                    { public: xsd__integer __item; }; 
    typedef LONG64 xsd__long; 
    class xsd__long_ : public xsd__integer_                       { public: xsd__long __item; }; 
    typedef long xsd__int; 
    class xsd__int_ : public xsd__long_                           { public: xsd__int __item; }; 
    typedef short xsd__short; 
    class xsd__short_ : public xsd__int_                          { public: xsd__short __item; }; 
    typedef char xsd__byte; 
    class xsd__byte_ : public xsd__short_                         { public: xsd__byte __item; }; 
    typedef xsd__string xsd__nonPositiveInteger; 
    class xsd__nonPositiveInteger_ : public xsd__integer_         { public: xsd__nonPositiveInteger __item; }; 
    typedef xsd__string xsd__negativeInteger; 
    class xsd__negativeInteger_ : public xsd__nonPositiveInteger_ { public: xsd__negativeInteger __item; }; 
    typedef xsd__string xsd__nonNegativeInteger; 
    class xsd__nonNegativeInteger_ : public xsd__integer_         { public: xsd__nonNegativeInteger __item; }; 
    typedef xsd__string xsd__positiveInteger; 
    class xsd__positiveInteger_ : public xsd__nonNegativeInteger_ { public: xsd__positiveInteger __item; }; 
    typedef ULONG64 xsd__unsignedLong; 
    class xsd__unsignedLong_ : public xsd__nonNegativeInteger_    { public: xsd__unsignedLong __item; }; 
    typedef unsigned long xsd__unsignedInt; 
    class xsd__unsignedInt_ : public xsd__unsignedLong_           { public: xsd__unsignedInt __item; }; 
    typedef unsigned short xsd__unsignedShort; 
    class xsd__unsignedShort_ : public xsd__unsignedInt_          { public: xsd__unsignedShort __item; }; 
    typedef unsigned char xsd__unsignedByte; 
    class xsd__unsignedByte_ : public xsd__unsignedShort_         { public: xsd__unsignedByte __item; }; 
    typedef xsd__string xsd__normalizedString; 
    class xsd__normalizedString_ : public xsd__string_            { public: xsd__normalizedString __item; }; 
    typedef xsd__string xsd__token; 
    class xsd__token_ : public xsd__normalizedString_             { public: xsd__token __item; }; 
~~~

Note the use of the trailing underscores for the `class` names to distinguish the `typedef` type names from the
`class` names.  The `char*` type of `xsd__string` can be replaced with `std::string` or a wide string type.
We can also add the <i>`xsd:base64Binary`</i> and <i>`xsd:hexBinary`</i> types that serialize raw binary data in the hierarchy as follows:

~~~{.cpp}
    class xsd__base64Binary : public xsd__anySimpleType           { public: unsigned char *__ptr; int __size; }; 
    class xsd__hexBinary : public xsd__anySimpleType              { public: unsigned char *__ptr; int __size; };
~~~

See Sections \ref base64binary  and \ref hexbinary .

Methods can be added to these classes, such as constructors and getter/setter methods, see Section \ref gettersetter .

Wrapper structs are supported as well, similar to wrapper classes.  But they cannot be used
to implement polymorphism.  Rather, the wrapper structs are used to represent a <i>`xsd:sequence`</i> of elements or to add attributes to primitive types as explained in Section \ref attributes .

For additional details, see the [C and C++ XML Data Bindings](../../databinding/html/index.html) documentation.

🔝 [Back to table of contents](#)

### Multi-reference strings   {#multirefstrings}

If more than one `char` pointer points to the same string, the string is encoded as a multi-reference value, unless `#SOAP_XML_TREE` is used or the `#WITH_NOIDREF` compile-time flag.

Consider for example:

~~~{.cpp}
    class ns__record
    { public:
        const char *s;
        const char *t;
    };
~~~

A record instance is populated as follows and then serialized:

~~~{.cpp}
    struct soap *soap = soap_new1(SOAP_XML_GRAPH);
    ns__record record;
    record.s = "hello";
    record.t = s;
    soap_write_ns__record(soap, &record);
~~~

The `s` and `t` variables are assigned the same string. When serialized, `t` refers to the content of `s`:

<div class="alt">
~~~{.xml}
    <ns:record>
      <s id="_1">hello</s> 
      <t ref="_1"/>
    </ns:record>
~~~
</div>

However, strings declared with different typedef names will never be considered multi-reference even when they point
to the same string.  For example:

~~~{.cpp}
    typedef char *xsd__string;
    typedef char *ns__string;
    class ns__record
    { public:
        const xsd__string s;
        const ns__string t;
    };
~~~

This avoids type conflicts when a receiver considers these types incompatible.

To enable multi-references in XML use `#SOAP_XML_GRAPH`. To disable multi-references in SOAP 1.1 and 1.2 RPC encoded messages, use `#SOAP_XML_TREE`.

🔝 [Back to table of contents](#)

### Smart string mixed-content deserialization {#smart}

The implementation of string deserialization permits mixed content. When XML contains mixed text and tags when a string is expected, the text with tags are collected into the deserialized string.

For example, suppose the `getInfo` service operation returns some detailed information. The service operation is declared as:

~~~{.cpp}
    // Contents of file "getInfo.h": 
    getInfo(char *detail);
~~~

The proxy of the service is used by a client to request a piece of information and the service responds with:

    HTTP/1.1 200 OK 
    Content-Type: text/xml 
    Content-Length: nnn 
<div class="alt">
~~~{.xml}
    <SOAP-ENV:Envelope
        xmlns:SOAP-ENV="http://schemas.xmlsoap.org/soap/envelope/" 
        xmlns:SOAP-ENC="http://schemas.xmlsoap.org/soap/encoding/" 
        xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" 
        xmlns:xsd="http://www.w3.org/2001/XMLSchema" 
      <SOAP-ENV:Body> 
        <getInfoResponse> 
          <detail> 
            <picture>Mona Lisa by <i>Leonardo da Vinci</i></picture> 
          </detail> 
        </getInfoResponse> 
      </SOAP-ENV:Body> 
    </SOAP-ENV:Envelope>
~~~
</div>

The `detail` string will contain `"<picture>Mona Lisa by <i>Leonardo da Vinci</i></picture>"`.

Note that serialization of this string will not produce mixed content but rather the XML output:

<div class="alt">
~~~{.xml}
    Mona Lisa by &lt;i&gt;Leonardo da Vinci&lt;/i&gt;
~~~
</div>

To serialize XML stored in strings, use the `_XML` type (a `char*`) in the interface header file for soapcpp2.  For example:

~~~{.cpp}
    // Contents of file "getInfo.h": 
    getInfo(_XML detail);
~~~

In C++ you can use a `std::string` instead, as follows:

~~~{.cpp}
    // Contents of file "getInfo.h": 
    typedef std::string XML;
    getInfo(XML detail);
~~~

The `_XML` and typedef `XML` are literal XML strings, see also Section \ref literal2.

🔝 [Back to table of contents](#)

### Changing the precision of float and double types   {#precision}

The format used to output double precision floating point values in XML is by default set to "`%.17lG`", which means that
at most 17 digits of precision are output.
The format used by the gSOAP engine to output single precision floating point values is by default "`%.9G`".

The format of a double can be set by assigning a format string to `::soap::double_format`.
For example:

~~~{.cpp}
    struct soap soap; 
    soap_init(&soap); // sets double_format = "%.18G" 
    soap.double_format = "%e"; // redefine
~~~

which causes all doubles to be output in XML and JSON in scientific notation.
Likewise, the encoding format of a float type can be set by assigning a format string to the `::soap::float_format` string variable. For example:

~~~{.cpp}
    struct soap soap; 
    soap_init(&soap); // sets float_format = "%.9G" 
    soap.float_format = "%.4f"; // redefine
~~~

which causes all floats to be output in XML and JSON with four digits precision.

A new feature to specify format patterns was introduced in gSOAP 2.8.18. A
format string can be used as a pattern for a typedef float or double in the
interface header file for soapcpp2 to specify the representation in XML.
For example:

~~~{.cpp}
    typedef float time__ratio "%5.2f";
~~~

This will output the float in XML with 5 digits total and 2 digits after the decimal
point.

The soapcpp2 tool also generates an XML schema with <i>`xsd:totalDigits`</i> and <i>`xsd:fractionDigits`</i> for this type:

<div class="alt">
~~~{.xml}
    <simpleType name="ratio"> 
      <restriction base="xsd:float"> 
        <totalDigits value="5"/> 
        <fractionDigits value="2"/> 
      </restriction> 
    </simpleType>
~~~
</div>

The wsdl2h tool converts WSDLs and XSDs with <i>`xsd:totalDigits`</i> and <i>`xsd:fractionDigits`</i> to typedefs with format patterns.

🔝 [Back to table of contents](#)

### INF, -INF, and NaN values of float and double types    {#floatinfnan}

IEEE INF, -INF, and NaN
values of floats are output in XML as <i>`INF`</i>, <i>`-INF`</i>, and <i>`NaN`</i>, respectively, as supported by the XML schema standards. 


For portability, the following macros can be used containing the float and double values <i>`INF`</i>, <i>`-INF`</i>, and <i>`NaN`</i>:

~~~{.cpp}
    float x = FLT_PINFTY;
    float x = FLT_NINFTY;
    float x = FLT_NAN;
    double x = DBL_PINFTY;
    double x = DBL_NINFT;
    double x = DBL_NAN;
~~~

To check for <i>`INF`</i>, <i>`-INF`</i>, and <i>`NaN`</i> use:

~~~{.cpp}
    soap_isinf(x) && x > 0 // x is INF
    soap_isinf(x) && x < 0 // x is -INF
    soap_isnan(x)          // x is NaN
~~~

🔝 [Back to table of contents](#)

## Enumeration serialization        {#enum}

Enumerations are generally useful for the declaration of named integer-valued constants.

For additional details, see the [C and C++ XML Data Bindings](../../databinding/html/index.html) documentation.

🔝 [Back to table of contents](#)

### Serialization of symbolic enumeration constants   {#enumserialization}

The soapcpp2 tool encodes the constants of enumeration-typed variables in symbolic form using the names of the constants when possible to comply to SOAP's enumeration encoding style. Consider for example the following enumeration of weekdays:

~~~{.cpp}
    enum weekday { Mon, Tue, Wed, Thu, Fri, Sat, Sun };
~~~

The enumeration-constant `Mon`, for example, is encoded as

<div class="alt">
~~~{.xml}
    <weekday>Mon</weekday>
~~~
</div>

An XML namespace prefix can be specified as part of the enumeration-type identifier's name, with the usual namespace prefix conventions for identifiers. For example:

~~~{.cpp}
    enum ns__weekday { Mon, Tue, Wed, Thu, Fri, Sat, Sun };
~~~

The `ns__weekday` type with enumeration-constant `Sat`, for example, is output in XML as:

<div class="alt">
~~~{.xml}
    <ns:weekday>Sat</ns:weekday>
~~~
</div>

The corresponding XML schema type for this enumeration type is:

<div class="alt">
~~~{.xml}
    <xsd:simpleType name="weekday"> 
      <xsd:restriction base="xsd:string"> 
        <xsd:enumeration value="Mon"/> 
        <xsd:enumeration value="Tue"/> 
        <xsd:enumeration value="Wed"/> 
        <xsd:enumeration value="Thu"/> 
        <xsd:enumeration value="Fri"/> 
        <xsd:enumeration value="Sat"/> 
        <xsd:enumeration value="Sun"/> 
      </xsd:restriction> 
    </xsd:simpleType>
~~~
</div>

C++11 scoped enumerations are supported by soapcpp2 with option <b>`-c++11`</b>:

~~~{.cpp}
    enum class ns__weekday : int { Mon, Tue, Wed, Thu, Fri, Sat, Sun };
~~~

Enumeration constants can be initialized, for example:

~~~{.cpp}
    enum ns__relation { LESS = -1, EQUAL = 0, GREATER = 1 };
~~~

The symbolic names <i>`LESS`</i>, <i>`EQUAL`</i>, and <i>`GREATER`</i> will appear in the XML output.

If the value of an enumeration-typed variable has no corresponding named constant, the value is encoded as a signed integer literal. For example, the following declaration of a `workday` enumeration type lacks named constants for Saturday and Sunday:

~~~{.cpp}
    enum ns__workday { Mon, Tue, Wed, Thu, Fri };
~~~

If the constant `5` (Saturday) or `6` (Sunday) is assigned to a variable of the `workday` enumeration type, the variable will be encoded with the integer literals <i>`5`</i> and <i>`6`</i>, respectively. For example:

<div class="alt">
~~~{.xml}
    <ns:workday>5</ns:workday>
~~~
</div>

Since this is legal in C/C++ and in SOAP RPC encoding, but not XML validators, we cam transmit integer literals as well as enumeration constants with an enumeration type.

When enumeration constants are numeric, we can use the following simple trick:

~~~{.cpp}
    enum ns__nums { _1 = 1, _2 = 2, _3 = 3 };
~~~

The corresponding XML schema type for this enumeration type is:

<div class="alt">
~~~{.xml}
    <simpleType name="nums">
      <restriction base="xsd:long">
        <enumeration value="1"/>
        <enumeration value="2"/>
        <enumeration value="3"/>
      </restriction>
    </simpleType>
~~~
</div>

🔝 [Back to table of contents](#)

### How to reuse symbolic enumeration constants   {#enumreuse}

A well-known deficiency of C and C++ enumeration types before C++11 scoped enumerations is the lack of a mechanism to reuse symbolic names by multiple enumerations. This issue is largely resolved with scoped enumerations in C++11, which the soapcpp2 tool supports.

In plain C and C++ we can use trailing underscores to avoid name clashes, for example:

Consider for example:

~~~{.cpp}
    enum ns__workday { Mon, Tue, Wed, Thu, Fri }; 
    enum ns__weekday { Mon_, Tue_, Wed_, Thu_, Fri_, Sat_, Sun_ };
~~~

which will result in the encoding of the constants of `enum ns__weekday` without the underscore, for example as <i>`Mon`</i>.

However, the soapcpp2 tool is a bit smarter than your average C/C++ compiler and also permits the following declarations that reuse enumeration constants, because the enumeration constants have the same enumerating integer values:

~~~{.cpp}
    enum ns__workday { Mon, Tue, Wed, Thu, Fri }; 
    enum ns__weekday { Mon, Tue, Wed, Thu, Fri, Sat, Sun };
~~~

The soapcpp2 tool generates <i>`soapStub.h`</i> with amended enumeration definitions that the C/C++ compiler can handle, so you can still use the shared enumeration constants in your application code.

To avoid name clashes with enumeration constants, you can use the following convention with double underscores to add the enum name to the enum constants:

~~~{.cpp}
    enum prefix__name { prefix__name__enumconst1, prefix__name__enumconst2, ... };
~~~

where the type name of the enumeration `prefix__name` is a prefixed name,
such as:

~~~{.cpp}
    enum ns__workday {
      ns__workday__Mon,
      ns__workday__Tue,
      ns__workday__Wed,
      ns__workday__Thu,
      ns__workday__Fri
    }; 
    enum ns__weekday {
      ns__workday__Mon,
      ns__workday__Tue,
      ns__workday__Wed,
      ns__workday__Thu,
      ns__workday__Fri,
      ns__workday__Sat,
      ns__workday__Sun
    }; 
~~~

This ensures that the XML schema enumeration values are still simply <i>`Mon`</i>, <i>`Tue`</i>, <i>`Wed`</i>, <i>`Thu`</i>, <i>`Fri`</i>, <i>`Sat`</i>, and <i>`Sun`</i>.

@warning The following declaration:
~~~{.cpp}
    enum ns__workday { Mon, Tue, Wed, Thu, Fri }; 
    enum ns__weekday { Sat = 5, Sun = 6};
~~~
will not properly encode the `weekday` enumeration when you assume that workdays are part of weekdays, because it lacks the named constants for `workday` in its enumeration list. All enumerations must be self-contained and cannot use enum constants of other enumerations.

🔝 [Back to table of contents](#)

### Boolean enumeration serialization for C        {#boolean}

The C++ `bool` type that is serialized as <i>`xsd:boolean`</i> XSD type cannot be used in C.
Instead, an enumeration type should be used to serialize true and false values as <i>`xsd:boolean`</i> XSD type values.
The <i>`xsd:boolean`</i> XSD type is defined as an enumeration in C as:

~~~{.cpp}
    enum xsd__boolean { false_, true_ };
~~~

The value `false_`, for example, is output in XML as:

<div class="alt">
~~~{.xml}
    <xsd:boolean>false</xsd:boolean> 
~~~
</div>

Peculiar of the SOAP encoding boolean type is that it only defines the values <i>`0`</i> and <i>`1`</i>, while the XSD <i>`xsd:boolean`</i> type defines <i>`false`</i> and <i>`true`</i> as valid values. While SOAP encoding types are rarely used since almost all SOAP/XML Web services rely on XSD types for primitive values, we can still define the following:

~~~{.cpp}
    typedef int SOAP_ENC__boolean;
~~~

🔝 [Back to table of contents](#)

### Bitmask enumeration serialization   {#bitmask}

A bitmask is an enumeration of power-of-two flags.  The soapcpp2 tool makes it easy to define bitmasks using an annotated `enum` with a `*`:

~~~{.cpp}
    enum * name { enum-constant, enum-constant, ... };
~~~

This declares a regular `enum` but enumerates the enumeration constants as a series of powers of 2 starting with 1.
This means that the enumeration constants can be bitwise or-ed with the `|` operator to form a bitvector (bitmask) which is serialized in XML as a list of symbolic values.
For example:

~~~{.cpp}
    enum * ns__machineStatus { ON, BELT, VALVE, HATCH}; 
    int ns__setMachineStatus(enum ns__machineStatus status, enum ns__machineStatus *result);
~~~

Note that the use of the `enum` name as a parameter does not require the asterisk, only the definition does.
The soapcpp2 tool generates a proper C/C++ enumeration in <i>`soapStub.h`</i>
that is included by <i>`soapH.h`</i> by your application:

~~~{.cpp}
    enum ns__machineStatus { ON=1, BELT=2, VALVE=4, HATCH=8 };
~~~

The corresponding XML schema type for this enumeration type is:

<div class="alt">
~~~{.xml}
    <simpleType name="machineStatus">
      <list>
        <restriction base="xsd:string">
          <enumeration value="ON"/>
          <enumeration value="BELT"/>
          <enumeration value="VALVE"/>
          <enumeration value="HATCH"/>
        </restriction>
      </list>
    </simpleType>
~~~
</div>

The values of `enum ns__machineStatus` can be or-ed, for example `ON|VALVE` is output in XML as:

<div class="alt">
~~~{.xml}
    <ns:machineStatus>ON VALVE</ns:machineStatus> 
~~~
</div>

C++11 scoped enumerations for bitmasks are supported by soapcpp2, for example:

~~~{.cpp}
    enum * class ns__machineStatus { ON, BELT, VALVE, HATCH}; 
    int ns__setMachineStatus(ns__machineStatus status, ns__machineStatus *result);
~~~

🔝 [Back to table of contents](#)

## Struct and class serialization        {#struct}

This section gives a brief overview of struct and class serialization.  

Structs do not support inheritance when declared in an interface header file
for soapcpp2.  This makes serialization of structs is more efficient compared
to classes.  Serialization functions for structs are global functions.
By contrast, soapcpp2 augments classes with serialization methods and `soap_type()`
method that returns the type of the class instance, which is necessary to distinguish
base class instances from derived class instances for (smart) pointers to base class instances.

For additional details not covered here,
see the [C and C++ XML Data Bindings](../../databinding/html/index.html) documentation.

A class and struct instance is serialized as an XML element with attributes and
sub-elements, which is represented in XML schema as a <i>`complexType`</i>.
The class name is the XML schema type name and the
member variables of the class are the type's accessors.

Consider the general declaration of an inherited class:

~~~{.cpp}
    class prefix__class_name1 : public prefix__class_name2
    { public:
        field1; 
        field2; 
        ... // more fields
        method1; 
        method2; 
        ... // more methods
    }; 
~~~

then

* `prefix__` is the optional namespace prefix associated with the class.

* `class_name1`  is the name of the <i>`complexType`</i> for this class.

* `class_name2`  is an optional base class.

* `field`        is a member variable that is serialized when public and non-const.

* `method` is a method declaration. Abstract methods are not allowed for serializable classes.

A class name is required to be unique and cannot have the same name as a
`struct`, `enum`, or a service operation name specified in the interface header file
for soapcpp2.

Only single inheritance is supported by the soapcpp2 tool. Multiple
inheritance is not supported because of the limitations of the XML schema extensibility.

If a constructor is present, there must also be a constructor
declaration with an empty parameter list.
If no constructors are present, then soapcpp2 generates constructors to initialize the members with the generated `soap_default` method of this class.

To obtain more information about the code generated by soapcpp2 for a struct or class, use <b>`soapcpp2 -r`</b> option <b>`-r`</b> to generate a <i>`soapReadme.md`</i> report with all the details.

Classes and structs may be declared `volatile` if you don't want soapcpp2 to generate the class definition, see Section \ref volatile  for more details.

Class templates are supported with only one template argument, see Section \ref templates .

Member variables of a class can be serialized as XML
attributes using the `@` type qualifier, if the member is a primitive type or pointer to a primitive type.  See Section \ref attributes 
for more details.

See Section \ref idtrans  for more details on the struct/class member
serialization and the resulting element and attribute qualified forms.

Arrays may be embedded within a class and a struct using a pointer member and
size information, see Section \ref list .

Void pointers may be used in a class or a struct, but you have to add a type
field so the engine can determine the type of object pointed to, see
Section \ref void .

A class instance is output in XML as:

<div class="alt">
~~~{.xml}
    <prefix:class-name>
      <basefield1>...</basefield1> 
      <basefield2>...</basefield2> 
      ... 
      <field1>...</field1> 
      <field2>...</field2> 
      ... 
    </prefix:class-name>
~~~
</div>

where the <i>`field`</i> accessors have element-name representations of the
class members and the <i>`basefield`</i> accessors have element-name
representations of the base class members.

If a derived class instance is used in place of a base class instance, then the serialized XML form carries a <i>`xsi:type`</i> attribute with the derived class type to distinguish it from the base class type:

<div class="alt">
~~~{.xml}
    <prefix:class-name xsi:type="prefix:class-name">
      <basefield1>...</basefield1> 
      <basefield2>...</basefield2> 
      ... 
      <field1>...</field1> 
      <field2>...</field2> 
      ... 
    </prefix:class-name>
~~~
</div>

The deserialization of a class instance allows any ordering of the accessors in the
XML message. However, if a base class member name is identical to a derived
class member name, because the member is overloaded, the base class member name
must precede the derived class member name in the XML message.

For additional details, see the [C and C++ XML Data Bindings](../../databinding/html/index.html) documentation.

🔝 [Back to table of contents](#)

### Example   {#example10}

The following example declares a base class `ns__Object` and a derived class `ns__Shape`:

~~~{.cpp}
    // Contents of file "shape.h": 
    class ns__Object 
    { public: 
      @ char *name; 
    }; 
    class ns__Shape : public ns__Object 
    { public: 
      @ int sides; 
      @ enum ns__Color { Red, Green, Blue } color; 
        std::string description;
        ns__Shape(); 
        ns__Shape(int sides, enum ns__Color color, std::string& description); 
        ~ns__Shape(); 
    };
~~~

The implementation of the `class ns__Shape` methods cannot be part of the interface header file for soapcpp2 and are defined in a separate <i>`shape.cpp`</i> C++ source code file.

An instance of `class ns__Shape` with name Triangle, 3 sides, and color Green is output in XML as:

<div class="alt">
~~~{.xml}
    <ns:Shape name="Triangle" sides="3" color="Green">
      <description>This is a green triangle</description>
    </ns:shape>
~~~
</div>

🔝 [Back to table of contents](#)

### Class methods   {#methods}

A class declaration in the interface header file for soapcpp2 may
include method declarations.  The method implementations must not be part of
the header file but should be defined in another C++ source file, because soapcpp2 parses C/C++ type declarations but does not parse C/C++ code statements and constructor initializer lists.

If constructors are not defined, then soapcpp2 generates constructors for the class to initialize the class with default values for member variables or the initialization values for member variables given in the class declaration.

If destructors are not defined, then soapcpp2 generates destructors for the class.

To obtain more information about the code generated by soapcpp2 for a class, use <b>`soapcpp2 -r`</b> option <b>`-r`</b> to generate a <i>`soapReadme.md`</i> report with all the details.

🔝 [Back to table of contents](#)

### Get and set methods        {#gettersetter}

Setter and getter methods are invoked at run time upon serialization and
deserialization of class instances, respectively. The use of setter and getter methods adds more flexibility to the serialization and deserialization process.

A setter method is called by the serializer.  You can use
setter methods to update a class instance just before it is serialized. For example, you can
use setter methods to update a class instance right before serialization. Setters are methods
for "set to serialize" operations.

Getter methods are immediately invoked after deserialization of a class instance. You can use
them to adjust the contents of class instances right after the instance was populated by the deserializer

Getter and setter methods have the following class method signature:

~~~{.cpp}
    int get(struct soap *soap);
    int set(struct soap *soap);
~~~

These methods may be declared `virtual` and may be declared `const`.

The active `::soap` context will be passed to the `get` and `set` methods. The methods should return `#SOAP_OK` when successful. A setter method should prepare the contents of the class instance for serialization. A getter method should process the instance after deserialization.

Here is an example of a base64 binary class:

~~~{.cpp}
    class xsd__base64Binary 
    { public: 
        unsigned char *__ptr; 
        int __size; 
        int get(struct soap *soap); 
        int set(struct soap *soap); 
    };
~~~

Suppose that the type and options members of the attachment should be set when
the class is about to be serialized. This can be accomplished with the
`set` method from the information provided by the `__ptr` to the data
and the `::soap` context passed to the `set` method (you can pass data via the
`void* ::soap::user` member).

The `get` method is invoked after the base64 data has been processed. You
can use it for post-processing purposes.

Here is another example. It defines a primitive `update` type. The class is a wrapper for the `time_t` type, see Section \ref primclass . Therefore, elements of this type contain <i>`xsd:dateType`</i> data.

~~~{.cpp}
    class update 
    { public: 
        time_t __item; 
        int set(struct soap *soap); 
    };
~~~

The setter method assigns the current time just before the instance is serialized:

~~~{.cpp}
    int update::set(struct soap *soap) 
    {
      this->__item = time(NULL); 
      return SOAP_OK; 
    }
~~~

This means that serialization in XML results in the inclusion of an up-to-date time stamp.

A `get` method is invoked immediately after the instance is populated by the deserializer.
The method is not invoked when the element is an <i>`xsi:nil`</i> element or has a
SOAP <i>`href`</i> or <i>`ref`</i> attribute referencing a value located elsewhere in the XML message or document.

@note The `soap_out` method of a class calls the setter method
However, the `soap_out` method is declared `const`
while the setter should be allowed to modify the contents of the class
instance. Therefore, the soapcpp2-generated code recasts the instance and the
`const` is removed when invoking the setter.

🔝 [Back to table of contents](#)

### Updating and checking instances with get and set methods        {#streaming}

Getter methods enable streaming XML operations. A getter method is invoked
when the object is deserialized and the rest of the XML message has not
been parsed yet. For example, you can add a getter method to the SOAP Header
class to implement header processing logic that is activated as soon as the
SOAP Header is received.  An example is shown below:

~~~{.cpp}
    class h__Authentication 
    { public: 
        char *id; 
        int get(struct soap *soap); 
    }; 
    class SOAP_ENV__Header 
    { public: 
        h__Authentication *h__authentication; 
    };
~~~

The `Authentication` SOAP Header member is instantiated and decoded. After
decoding, the getter method is invoked, which can be used to check the `id`
before the rest of the SOAP message is parsed.

🔝 [Back to table of contents](#)

### Polymorphism, derived types, and dynamic binding in C++     {#polymorph}

Polymorphism through C++ inheritance is supported by the gSOAP tools, which
means that derived XML schema types are (de)serialized when an
<i>`xsi:type`</i> attribute is present.

Because C does not support inheritance, a different approach is use for C code,
see Section \ref polymorphC for details.

Base and derived C++ classes can be used anywhere, including service
operation parameters and in struct and class members, provided that parameters
and members are pointers to classes to allow dynamic binding at run time.  Base
and derived classes can also be used with containers such as `std::vector` and
smart pointers such as `std::shared_ptr`.

The following example interface header file for soapcpp2 declares `ns__Base` and `ns__Derived`
classes and a service operation that takes a pointer to a `ns__Base` class instance
and returns a `ns__Base` class instance:

~~~{.cpp}
    // Contents of file "derived.h" 
    class ns__Base 
    { public: 
        char *name; 
        ns__Base(); 
        virtual void print(); 
    }; 
    class ns__Derived : public ns__Base 
    { public: 
        int num; 
        ns__Derived(); 
        virtual void print(); 
    }; 
    int ns__webmethod(ns__Base *in, struct ns__webmethodResponse { ns__Base *out; } & result);
~~~

The service operation input parameter may point to a `ns__Derived` class instance
that will be serialized as `ns__Derived` class instance instead of a `ns__Base` class
instance.  Likewise, the service operation output parameter that is placed in a
wrapper struct (because structs and classes are always considered wrappers to
define the response message with output parameters) may point to a
`ns__Derived` class instance.

The `ns__Base` and `ns__Derived` class method implementations are:

~~~{.cpp}
    // Method implementations of the ns__Base and ns__Derived classes: 
    #include "soapH.h" 

    ns__Base::ns__Base() 
    {
      std::cout << "created a Base class instance" << std::endl; 
    } 

    ns__Derived::ns__Derived() 
    {
      std::cout << "created a Derived class instance" << std::endl; 
    } 

    ns__Base::print() 
    {
      std::cout << "print(): Base class instance " << name << std::endl; 
    } 

    ns__Derived::print() 
    {
      std::cout << "print(): Derived class instance " << name << " " << num << std::endl; 
    }
~~~

Below is an example client application that creates a `ns__Derived` class instance
that is passed as the input parameter of the `ns__webmethod` service operation:

~~~{.cpp}
    // CLIENT 
    #include "soapH.h" 

    int main() 
    {
      struct soap soap; 
      soap_init(&soap); 
      ns__Derived obj; 
      struct ns__webmethodResponse r; 
      soap_default_ns__Derived(&soap, &obj);
      obj.name = "X"; 
      obj.num = 3; 
      if (soap_call_ns__webmethod(&soap, endpoint, NULL, &obj, r) == SOAP_OK)
        if (r.out)
          r.out->print(); 
      soap_destroy(&soap);
      soap_end(&soap);
      soap_done(&soap);
    } 
~~~

The following example server application copies a class instance (`ns__Base` or
`ns__Derived`) from the input to the output parameter:

~~~{.cpp}
    // SERVER
    #include "soapH.h" 

    int main() 
    {
      struct soap soap; 
      soap_init(&soap); 
      soap_serve(&soap);
      soap_destroy(&soap);
      soap_end(&soap);
      soap_done(&soap);
    } 

    int ns__webmethod(struct soap *soap, ns__Base *in, struct ns__webmethodResponse &result) 
    {
      if (in)
        in->print(); 
      result.out = in; 
      return SOAP_OK; 
    } 
~~~

The following messages are produced by the client and server applications:

    CLIENT: created a Derived class instance 
    SERVER: print(): Derived class instance X 3 
    CLIENT: created a Derived class instance 
    CLIENT: print(): Derived class instance X 3

This shows that the `Derived` class instance kept its identity as it passed
through the server.

Another way to serialize polymorphic values in XML that are indicated with
<i>`xsi:type`</i> attributes is with `void*` members that point to a
serializable value.  See Section \ref void for details.

🔝 [Back to table of contents](#)

### Polymorphism, derived types, and dynamic binding in C     {#polymorphC}

Because C does not support object-oriented inheritance, derived types are
obviously not declared as base structs or classes as in C++.  Instead, we add
the derived type structs to the base structs as members that point to the
derived type value when the base type is dynamically overridden by one of the
derived types.  In this way we can (de)serialize a base type struct as usual or one
of the derived structs when the base type is overridden.  To serialize a
derived type struct in place of the base struct, we set its corresponding
member point to the derived struct value, which is serialized with the
<i>`xsi:type`</i> attribute to indicate a derived type is used in XML.
Deserialization of a derived type struct is done automatically when the
<i>`xsi:type`</i> attribute is present.

This approach with additional members pointing to derived types was introduced
with gSOAP 2.8.75.  This approach has the benefit of type safety compared to
attempts to replicate C++ inheritance by trying to overlay derived types with
base types in memory, which would be fragile.

This method is fully automated for the wsdl2h tool to generate an interface
header file for soapcpp2 with the type derivations in C.  To use this method to
generate code from WSDLs and XSDs, use [<b>`wsdl2h -F`</b> option <b>`-F`</b>](#wsdl2h-F).
This also works in C++, but C++ inheritance works fine without this method.

Using this method with soapcpp2 alone using a manually-specified interface
header file produces the specified type inheritance in the soapcpp2-generated
WSDL and XML schema files as complexType extensions.

The soapcpp2 tool warns if a derived type has multiple base types.  At most one
base type for a derived type may be specified.

To illustrate this method, consider the following interface header file
example for soapcpp2 based on \ref polymorph.  This example declares `ns__Base` and
`ns__Derived` structs and a service operation that takes a pointer to a `ns__Base`
value and returns a `ns__Base` value:

~~~{.cpp}
    // Contents of file "derived.h" 
    struct ns__Base 
    {
        char *name; 
      [ struct ns__Derived *ns__Derived; ] // points to derived type when non-NULL
    }; 
    struct ns__Derived
    {
        char *name; 
        int num; 
    }; 
    int ns__webmethod(struct ns__Base *in, struct ns__webmethodResponse { struct ns__Base *out; } *result);
~~~

The `ns__Base` struct includes the special member `ns__Derived` that points to a
`ns__Derived` value.  This special member must be:

- a transient member (i.e. non-serializable) by placing the declaration within
  `[` and `]`, and
- the member name must match the type name (to be more precise, at least the
  initial part of the member name must match the type name as in the example
  `ns__Derived_` works too).

To serialize the `ns__Base` value make sure to set the `ns__Derived` member to NULL.
The soapcpp2-generated `soap_default_ns__Base()` function default initializes a given
`ns__Base` value for you.  To serialize the `ns__Derived` value make sure to set the
`ns__Derived` member to point to the address of a `ns__Derived` value.  This is easy by
calling `soap_new_ns__Derived()` that allocates and default initializes a `ns__Derived`
value, whose address is returned by this function.

When multiple derived types are declared for a base type, all immediately
derived struct types are added as transient pointer members to the base type.
Indirectly derived types do not need to be added to the base type as members,
but it is perfectly fine to do so.

To properly declare derived types, make sure to include all base type members
in the derived type.  In our example the `ns__Derived` struct contains the `ns__Base`
struct members (except for the `ns__Derived` member) and adds additional members as
extensions.

Below is an example client application based on the example in Section \ref
polymorph that creates a `ns__Derived` value that is passed as the input parameter
of the `ns__webmethod` service operation:

~~~{.cpp}
    // CLIENT 
    #include "soapH.h" 

    int main() 
    {
      struct soap soap; 
      soap_init(&soap); 
      struct ns__Base obj; 
      struct ns__Derived der; 
      struct ns__webmethodResponse r; 
      soap_default_ns__Base(&soap, &obj);
      soap_default_ns__Derived(&soap, &der);
      obj.ns__Derived = &der;
      der.name = "X"; 
      der.num = 3; 
      if (soap_call_ns__webmethod(&soap, endpoint, NULL, &obj, &r) == SOAP_OK)
      {
        if (r->out && r.out->ns__Derived)
          printf("print(): Derived class instance %s %d\n",
              r.out->ns__Derived->name,
              r.out->ns__Derived->num);
        else if (r->out)
          printf("print(): Base class instance %s\n",
              r.out->name);
      }
      soap_destroy(&soap);
      soap_end(&soap);
      soap_done(&soap);
    } 
~~~

The following example server application copies a class instance (`ns__Base` or
`ns__Derived` class) from the input to the output parameter:

~~~{.cpp}
    // SERVER
    #include "soapH.h" 

    int main() 
    {
      struct soap soap; 
      soap_init(&soap); 
      soap_serve(&soap);
      soap_destroy(&soap);
      soap_end(&soap);
      soap_done(&soap);
    } 

    int ns__webmethod(struct soap *soap, struct ns__Base *in, struct ns__webmethodResponse *result) 
    {
      if (in && in->ns__Derived)
        printf("print(): Derived class instance %s %d\n",
            in->ns__Derived->name,
            in->ns__Derived->num);
      else if (in)
        printf("print(): ns__Base class instance %s\n",
            in->name);
      result.out = in; 
      return SOAP_OK; 
    } 
~~~

The following messages are produced by the client and server applications:

    SERVER: print(): Derived class instance X 3 
    CLIENT: print(): Derived class instance X 3

This shows that the `Derived` class instance kept its identity as it passed
through the server.

Another way to serialize polymorphic values in XML that are indicated with
<i>`xsi:type`</i> attributes is with `void*` members that point to a
serializable value.  See \ref void for details.

Deeper levels of simulated inheritance are possible, for example:

~~~{.cpp}
    // Contents of file "derived.h" 
    struct ns__Base 
    {
        char *name; 
      [ struct ns__Derived *ns__Derived; ] // points to derived type when non-NULL
    }; 
    struct ns__Derived
    {
        char *name; 
        int   num; 
      [ struct ns__Derived2 *ns__Derived2; ] // points to derived type when non-NULL
    }; 
    struct ns__Derived2
    {
        char *name; 
        int   num; 
        char *value;
    }; 
    int ns__webmethod(struct ns__Base *in, struct ns__webmethodResponse { struct ns__Base *out; } *result);
~~~

This requires two pointer traversals from the base type `ns__Base` via
`ns__Derived` to reach `ns__Derived2`:

~~~{.cpp}
    int ns__webmethod(struct soap *soap, struct ns__Base *in, struct ns__webmethodResponse *result) 
    {
      if (in && in->ns__Derived && in->ns__Derived->ns__Derived2)
        printf("print(): Derived2 class instance %s %d %s\n",
            in->ns__Derived->ns__Derived2->name,
            in->ns__Derived->ns__Derived2->num,
            in->ns__Derived->ns__Derived2->value);
      else if (in && in->ns__Derived)
        printf("print(): Derived class instance %s %d\n",
            in->ns__Derived->name,
            in->ns__Derived->num);
      else if (in)
        printf("print(): ns__Base class instance %s\n",
            in->name);
      result.out = in; 
      return SOAP_OK; 
    } 
~~~

🔝 [Back to table of contents](#)

### How to declare XML attributes        {#attributes}

The gSOAP tools support the full XML schema standards, so XML attributes are nothing special.
However, with respect to SOAP standards it is important to note that
SOAP RPC/literal and SOAP document/literal styles support XML attributes in
SOAP messages, but SOAP RPC with "Section 5" encoding does not support XML
attributes other than some built-in attributes.

The idea behind SOAP RPC Section 5 encoding was to keep SOAP as simple as possible as a limited subset of XML, while offering advantages for cross-language interoperability of data types, including data structure graph serialization with multi-referenced data.

Attributes are primitive XSD types, such as strings, enumerations, boolean, and
numeric types. To declare an XML attribute in a struct or class, the qualifier
`@` is used with the type of the attribute. The type must be
primitive type or a pointer to a primitive type, including enumerations and `::xsd__base64Binary` and `::xsd__hexBinary` structures.
For example:

~~~{.cpp}
    typedef char *xsd__string; 
    typedef bool *xsd__boolean; 
    enum ns__state { _0, _1, _2 }; 
    struct ns__myStruct 
    {
      @ std::string         *type;
      @ bool                 flag = false;
      @ enum ns__state       state = _2;
        struct ns__myStruct *next; 
    };
~~~

The `@` qualifier declares an XML attribute for the
`type`, `flag`, and `state` members.

Default values can be associated with any member that has
a primitive type in a struct or class, as is illustrated in this example. The
default values are used when the receiving message does not contain the
corresponding values.

Pointers make the members optional. So `type` is an optional attribute.

Because a service operation request and response message is essentially a struct, XML
attributes can also be associated with method requests and responses. For
example:

~~~{.cpp}
    int ns__webmethod(@ char *ns__name, ...);
~~~

Attributes can also be attached to the dynamic arrays, binary types, and wrapper classes and structs of primitive
types. Wrapper classes are described in Section \ref primclass .  For
example:

~~~{.cpp}
    class xsd__string 
    { public:
        char * __item; 
      @ bool flag; 
    };
~~~

and

~~~{.cpp}
    class xsd__base64Binary 
    { public:
        unsigned char *__ptr; 
        int __size; 
      @ bool flag; 
    };
~~~

The attribute declarations must be placed after the special `__item`, `__ptr`, and `__size` members.

For additional details, see the [C and C++ XML Data Bindings](../../databinding/html/index.html) documentation.

🔝 [Back to table of contents](#)

### How to use QName attributes and elements   {#qname}

An element or attribute with type QName (Qualified Name) contains a namespace prefix and a local name.
We can explicitly declare a QName as a string type with `typedef char *xsd__QName` and the serializer recognizes the `QName` type as a special type that requires QName normalization.  A built-in QName type `_QName` is recognized by soapcpp2, which is a `char*` type with QName content.

QName normalization by the deserializer is applied to convert the prefix in the inbound XML message to the corresponding prefix defined in the XML namespace table, which means that the QName string is always received in normalized form.

For example:

~~~{.cpp}
    //gsoap ns schema namespace: urn:example
    typedef char *xsd__QName; 
    struct ns__myStruct 
    {
        xsd__QName elt = "ns:xyz"; // QName element with default value "ns:xyz" 
      @ xsd__QName att = "ns:abc"; // QName attribute with default value "ns:abc" 
    };
~~~

When the `elt` and `att` members are serialized,
their string contents are just output.
When the
members are deserialized however, the deserializer converts the prefix in the parsed QName to the prefix defined in the namespace table that corresponds to the same namespace URI.
For example, suppose that the inbound XML message contains <i>`<elt xmlns:x="urn:example">x:def</elt>`</i>.
The prefix <i>`x`</i> matches the namespace URI <i>`urn:example`</i> of prefix `ns` as declared by the `//gsoap ns schema namespace: urn:example` directive, which populates the namespace table <i>`ns.nsmap`</i> generated by soapcpp2.
Therefore, the <i>`x:def`</i> QName value is converted to <i>`ns:def`</i> and saved in the `elt` member of `ns__myStruct`.

If the namespace URI used in the inbound XML message is not in the namespace table, for example when <i>`<elt xmlns:x="urn:x">x:def</elt>`</i> is parsed, then
<i>`x:def`</i> is converted to `"URI":def` where `"URI"` is the namespace
URI bound to <i>`x`</i>, which is `"urn:x"` in this case.  This value `"urn:x":def`is saved in the `elt` member of `ns__myStruct`.

For additional details, see the [C and C++ XML Data Bindings](../../databinding/html/index.html) documentation.

🔝 [Back to table of contents](#)

## Union serialization        {#union}

A union is only serialized if the union is used within a
struct or class declaration that includes an `int __union`
member that acts as a selector (also called discriminant) for the union
members. The selector stores run-time usage information about the union
member that is activated.

A union within a struct or class with a selector member
represents <i>`xsd:choice`</i> XML schema component. For example:

~~~{.cpp}
    struct ns__PO 
    {
        ... // members of ns__PO
    }; 
    struct ns__Invoice 
    {
        ... // members of ns__Invoice
    }; 
    union ns__PO_or_Invoice 
    {
        struct ns__PO po; 
        struct ns__Invoice invoice; 
    }; 
    struct ns__composite 
    {
        char *name; 
        int __union; 
        union ns__PO_or_Invoice value; 
    };
~~~

The `union ns__PO_or_Invoice` appears as a <i>`xsd:choice`</i> in the generated XML schema:

<div class="alt">
~~~{.xml}
    <complexType name="composite"> 
      <sequence> 
        <element name="name" type="xsd:string"/> 
        <choice> 
          <element name="po" type="ns:PO"/> 
          <element name="invoice" type="ns:Invoice"/> 
        </choice> 
      </sequence> 
    </complexType>
~~~
</div>

The union name should be qualified, as shown in the example, to
ensure correct serialization when the XML schemas is declared with <i>`elementFormDefault="qualified"`</i>, with `//gsoap ns schema elementForm: qualified`.

The `int __union` member selector's values are generated by the
soapcpp2 tool. Each union member name has a selector value
defined by:

~~~{.cpp}
    SOAP_UNION_unionname_fieldname
~~~

These selector values enumerate the union members. The
special value 0 (or any negative value) can be assigned to omit the serialization of the union altogether, but only
if explicitly allowed by validation rules, which requires <i>`minOccurs="0"`</i>
for the <i>`xsd:choice`</i>:

~~~{.cpp}
    struct ns__composite 
    {
        char *name; 
        int __union 0; // declares <choice minOccurs="0">
        union ns__PO_or_Invoice value; 
    };
~~~

This way we can treat the union as an optional data item by setting `__union = 0`.

Since 2.7.16 it is also possible to use a '`$`' as a special marker to annotate a
selector member that must be of type `int` and the member name can be chosen arbitrarily:

~~~{.cpp}
    struct ns__composite 
    {
        char *name; 
      $ int select 0; // declares <choice minOccurs="0">
        union ns__PO_or_Invoice value; 
    };
~~~

The following example shows how the `struct ns__composite` instance is
initialized for serialization using the above declaration:

~~~{.cpp}
    struct ns__composite data; 
    data.name = "..."; 
    data.select = SOAP_UNION_ns__PO_or_Invoice_po; // select ns__PO_or_Invoice::po union member
    data.value.po.number = ...; // populate the PO
~~~

While the gSOAP serializers are designed to be robust, failing to set the
selector to a valid union member can lead to a crash of the serializer,
because it will attempt to serialize an invalid union member.

The deserializer of a union type sets the selector value to the currently active union member that was deserialized.
The selector will be set to a non-positive value (0 or -1) when no union member was deserialized, if permitted by the validator,
where -1 indicates that a member was required by validation rules, if the validator was non-strict.
Strict validation enabled with `#SOAP_XML_STRICT` results in a validation fault in this case.

When more than one union is used in a struct or class, the
`__union` selectors should use `$` to identify them and named to avoid name clashes, for example:

~~~{.cpp}
    struct ns__composite 
    {
        char *name; 
      $ int sel_value; // = SOAP_UNION_ns__PO_or_Invoice_[po|invoice] 
        union ns__PO_or_Invoice value; 
      $ int sel_data; // = SOAP_UNION_ns__Email_or_Fax_[email|fax] 
        union ns__Email_or_Fax data; 
    };
~~~

For additional details, see the [C and C++ XML Data Bindings](../../databinding/html/index.html) documentation.

🔝 [Back to table of contents](#)

## Pointer type serialization        {#pointer}

Basically, the serialization of a pointer amounts to the serialization of the data pointed
to.  However, if more than one pointer points to a node in a data structure to serialize, the node is either duplicated in the serialized output meaning the data is serialized as a tree, or the node is output only once in the serialized output meaning that the data is serialized as a graph.  The latter is referred to as multi-reference encoding in SOAP 1.1/1.2 RPC encoding style.  This style ensures that data structures maintain their structural integrity when transmitted, as intended by the true meaning of serialization.  The `#SOAP_XML_GRAPH` runtime flag can be used with plain non-SOAP XML to achieve the same.

To achieve this, the gSOAP serializers for SOAP RPC encoding and the `#SOAP_XML_GRAPH` flag check for
multi-referenced data in the data structure to serialize, i.e. the data nodes
that are co-referenced by other nodes, by adding id-ref/href attributes to the
XML output that refer to the co-referenced data.  The soapcpp2 tool generates
serializers that perform this check automatically on C/C++ pointers and smart
pointers, such as `std::shared_ptr`.  Furthermore, the soapcpp2 tool generates
serializers that prevent infinite serialization when a cyclic data structure is serialized as a tree, by breaking the cycles, when using the SOAP document/literal style or when `#SOAP_XML_TREE` is enabled with the SOAP RPC encoding style.

For additional details on the use of C/C++ pointers and smart pointers, see the [C and C++ XML Data Bindings](../../databinding/html/index.html) documentation.

🔝 [Back to table of contents](#)

### Multi-referenced data serialization   {#multiref}

A node in the data structure that is pointed to by more than one pointer is serialized as
multi-reference data when the SOAP RPC encoding style is used or when `#SOAP_XML_GRAPH` is enabled. This means that co-referenced data is identified in XML with a unique <i>`id`</i> attribute. References in XML are made with
<i>`href`</i> (SOAP 1.1 RPC encoding), <i>`SOAP-ENC:ref`</i> (SOAP 1.2 RPC encoding), or <i>`ref`</i> (`#SOAP_XML_GRAPH`) attributes
to refer to the co-referenced data. See Section \ref flags  on
options to control the serialization of multi-reference data. To turn multi-ref off, use `#SOAP_XML_TREE` to process plain tree-based XML. To completely eliminate multi-ref serialization use the `#WITH_NOIDREF` compile-time flag with all source code (including <i>`gsoap/stdsoap2.c`</i> and <i>`gsoap/stdsoap2.cpp`</i>) to permanently disable id-href processing.

Consider for example the following a linked list data structure:

~~~{.cpp}
    typedef char *xsd__string; 
    struct ns__list
    {
        xsd__string value; 
        struct ns__list *next; 
    };
~~~

Suppose a cyclic linked list is created. The first node contains the value "abc" and points to a node with value
"def" which in turn points to the first node. This is encoded as:

<div class="alt">
~~~{.xml}
    <ns:list id="1""> 
      <value>abc</value> 
      <next> 
        <value>def</value> 
        <next href="#1"/> 
      </next> 
    </ns:list>
~~~
</div>

In case multi-referenced data is received that "does not fit in a pointer-based structure", the data is copied.
For example, the following two structs are similar, except that the first uses pointer-based members while the other uses
non-pointer-based members:

~~~{.cpp}
    typedef long xsd__int; 

    struct ns__record 
    {
        xsd__int *a; 
        xsd__int *b; 
    } P; 

    struct ns__record
    {
        xsd__int a; 
        xsd__int b; 
    } R; 

    int main()
    {
      P.a = &n; 
      P.b = &n; 
      ... //
    }
~~~

Since both `a` and `b` members of `P` point to the same integer, the serialization of `P` produces a multi-reference in SOAP 1.1 RPC encoding:

<div class="alt">
~~~{.xml}
    <ns:record> 
      <a href="#1"/> 
      <b href="#1"/> 
    </ns:record> 
    <id id="1">123</id>
~~~
</div>

The deserialization of the content in the `R` data structure that does not use pointers to integers results in a copy of each
multi-reference integer.  Note that the two structs resemble the same XML data type because the trailing underscore will be
ignored in XML encoding and decoding.

🔝 [Back to table of contents](#)

### NULL pointers and nil elements    {#null}

A NULL pointer is not serialized, unless the pointer member of a struct or class is declared in the interface header file as nillable with `nullptr` or in the unlikely case the pointer itself is pointed to by another pointer (but see
Section \ref flags  to control the serialization of NULLs), for example:

~~~{.cpp}
    struct X 
    {
        int *p; 
        int **q; 
        int *r nullptr 1;
    }
~~~

The types section of a WSDL description contains information on the "nillability" of data, which is declared as `nullptr` members where the `1` indicates that the member is required (<i>`minOccurs`</i> and <i>`maxOccurs`</i> are 1 set with `1:1` or simply `1`).

Suppose pointer `q` points to pointer `p` and suppose `p` and `r` are NULL.
In that case the `X` struct is serialized with `#SOAP_XML_GRAPH` as:

<div class="alt">
~~~{.xml}
    <X>
      <p id="1" xsi:nil="true"/>
      <q ref="1/>
      <r id="1" xsi:nil="true"/>
    </X>
~~~
</div>

The deserializer reconstructs the struct `X` from this form of XML, thereby preserving the integrity of the data structure serialized.

When the deserializer encounters an XML element that has a <i>`xsi:nil="true"`</i> attribute but the corresponding C/C++ data is not a pointer or reference,
the deserializer will terminate with a `#SOAP_NULL` fault when the `#SOAP_XML_STRICT` flag is set.

🔝 [Back to table of contents](#)

## Void pointer serialization        {#void}

Void pointers (`void*`) cannot be serialized in XML because the
type of data referred to is untyped.  To enable the serialization of
void pointers that are members of structs and classes, you can insert a
`int __type` member right before the void pointer member.  The `int __type` member contains run time information on the type of the data pointed to by
`void*` member in a struct/class to enable the serialization of this
data.  The `int __type` member is set to a `SOAP_TYPE_T` value, where `T` is the name of a type.  The soapcpp2 tool generates the `SOAP_TYPE_T` definitions in <i>`soapH.h`</i> and uses them internally to uniquely identify the type of each object.
The type naming conventions outlined in
Section \ref serialize  are used to determine the type name for `T`.
Values serialized in XML with this approach always carry the <i>`xsi:type`</i>
attribute in XML to indicate the type of content serialized.

Here is an example to illustrate the serialization of a `void*` member in a struct/class:

~~~{.cpp}
    struct ns__record 
    {
        int __type; // the SOAP_TYPE_T pointed to by val
        void *val;  // serialize any type in element <val>
    };
~~~

The `__type` integer can be set to 0 at run time to omit the serialization
of the void pointer member.

The following example illustrates the initialization of `myStruct` with a
void pointer to an int:

~~~{.cpp}
    struct ns__record S; 
    int n = 123; 
    S.val = (void*)&n; 
    S.__type = SOAP_TYPE_int; 
~~~

The serialized output of `S` contains the integer in its <i>`val`</i> element:

<div class="alt">
~~~{.xml}
    <ns:record>
      <val xsi:type="xsd:int">123</val>
    </ns:record>
~~~
</div>

The deserializer for `ns__record` will automatically set the `__type`
field and void pointer when deserializing the data, provided that the
XML element <i>`val`</i> carries the <i>`xsi:type`</i> attribute from which it
can determine the type.

@note when serializing strings via a `void*` member, the `void*` pointer must
directly point to the string value rather than indirectly as with all other
types. For example:

~~~{.cpp}
    struct ns__record S; 
    S.val = (void*)"Hello"; 
    S.__type = SOAP_TYPE_string; 
~~~

This is the case for all string-based types, including types defined with `typedef char*`.

You may use an arbitrary suffix with the `__type` members to handle
multiple void pointers in structs/classes.  For example:

~~~{.cpp}
    struct ns__record 
    {
        int __typeOfp; // the SOAP_TYPE_T pointed to by p 
        void *p;       // element <p>
        int __typeOfq; // the SOAP_TYPE_T pointed to by q 
        void *q;       // element <q>
    };
~~~

Because service method parameters are stored within structs, you can use
`__type` and `void*` parameters to pass polymorphic arguments without
having to define a C++ class hierarchy (Section \ref polymorph ), provided that
<i>`xsi:type`</i> attributes are present in the XML elements.  For example:

~~~{.cpp}
    typedef char *xsd__string; 
    typedef int xsd__int; 
    typedef float xsd__float; 
    enum ns__status { on, off }; 
    struct ns__widget
    {
        char *name;
        int part;
    };
    int ns__webmethod(int __type, void *data, struct ns__webmethodResponse { int __type; void *return_; } *out);
~~~

This method has a polymorphic input parameter `data` and a polymorphic
output parameter `return_`.  The `__type` parameters can be one of
`SOAP_TYPE_xsd__string`, `SOAP_TYPE_xsd__int`,
`SOAP_TYPE_xsd__float`, `SOAP_TYPE_ns__status`, or
`SOAP_TYPE_ns__widget`.  The WSDL and XSD files produced by the soapcpp2 tool
declare the `void*` polymorphic members as <i>`xsd:anyType`</i> elements.

To declare a wrapper struct/class for `void*` pointers allows us to reuse this
mechanism when we use `__self` as a member name that refers to the current XML
element tag name:

~~~{.cpp}
    struct __any 
    {
        int __type;   // the SOAP_TYPE_T pointed to by __self
        void *__self; // serialize any type of content of the current element
    };
    struct ns__record
    {
        __any val;
    };
~~~

The following example illustrates the initialization of `__ns__record` with a
void pointer to an int:

~~~{.cpp}
    struct ns__record S; 
    int n = 123; 
    S.val.__item = (void*)&n; 
    S.val.__type = SOAP_TYPE_int; 
~~~

The serialized output of `S` contains the integer:

<div class="alt">
~~~{.xml}
    <ns:record>
      <val xsi:type="xsd:int">123</val>
    </ns:record>
~~~
</div>

🔝 [Back to table of contents](#)

## Fixed-size array serialization   {#fixedarrays}

Fixed size arrays are serialized as repetitions of <i>`item`</i> elements with the array values in XML.
Multi-dimensional fixed size arrays are serialized as nested <i>`item`</i> elements, where the outer elements are arrays.

The serialization of fixed-size arrays supports the SOAP RPC encoding multi-dimensional array format as well as partially transmitted and sparse array formats standardized in SOAP 1.1 and 1.2.

For example:

~~~{.cpp}
    // Contents of file "fixed.h": 
    struct Example 
    {
        float a[3]; 
    };
~~~

This specifies a fixed-size array part of the `struct Example`. The serialized output of array `a` is:

<div class="alt">
~~~{.xml}
    <a>
      <item>1.0</item>
      <item>2.0</item>
      <item>3.0</item>
    </a>
~~~
</div>

Any deserialized items of an array that do not fit in the fixed size array,
i.e. are out of bounds, are ignored by the deserializer when the
`#SOAP_C_NOIOB` flag is set, otherwise `#SOAP_IOB` errors will be generated by
the deserializer.

🔝 [Back to table of contents](#)

## Dynamic array serialization        {#dynarray}

Dynamic arrays are much more flexible than fixed-size
arrays. Dynamic arrays declared in the interface header file for soapcpp2 are a special struct or class or are part of a struct or class with a member pointing to an array of elements and a member that stores the size of the array.
Dynamic array allocations are easy using the soapcpp-generated `soap_new_T` functions for type `T`.  This function is used to allocate an array of values which can then be assigned to the pointer member of the struct/class that stores the array pointer with its size.

To facilitate SOAP RPC encoding, SOAP-encoded arrays require special treatment. SOAP-encoded arrays are single- or multi-dimensional arrays with bounds that appear in XML.  These arrays may also have offsets that differ from zero.  The intent of SOAP-encoded arrays is to replicate multi-dimensional arrays commonly found in programming languages.

However, XML also provides a simple way to represent a sequence of values with a sequence of XML elements.  This differs from SOAP-encoded arrays in that SOAP-encoded arrays are elements with nested <i>`item`</i> elements with values, though SOAP deserializers may ignore the name of these elements when parsing XML as stated in the SOAP specifications.

Both SOAP-encoded arrays and sequences of XML elements are supported in gSOAP, using dynamic arrays and containers.  The basics will be described next.  For additional details, see the [C and C++ XML Data Bindings](../../databinding/html/index.html) documentation.

🔝 [Back to table of contents](#)

### SOAP-encoded array bounds {#arraybounds}

SOAP-encoded arrays use the <i>`SOAP-ENC:Array`</i> attribute in XML to identify the array and the <i>`SOAP-ENC:arrayType`</i> attribute to identify the array dimensionality and its size.

As a security measure to avoid denial of service attacks based on sending a huge array size value using the <i>`SOAP-ENC:arrayType`</i> attribute, requiring the allocation of large chunks of memory, the total number of array elements set by the <i>`SOAP-ENC:arrayType`</i> attribute cannot exceed `#SOAP_MAXARRAYSIZE`, which is set to 100000 by default. This limit is not a hard limit on the number of array elements, but rather to avoid pre-allocating large arrays as stated. The hard limit on the number of array elements received is `::soap::maxoccurs` which is set to `#SOAP_MAXOCCURS` by default.  By contrast, the `#SOAP_MAXARRAYSIZE` limit only negatively affects multi-dimensional arrays because the dimensionality of the receiving array may be lost when the number of elements exceeds 100000. One-dimensional arrays are not affected and populated after this limit by simply deserializing the array elements received.

🔝 [Back to table of contents](#)

### One-dimensional dynamic SOAP-encoded arrays   {#oned}

A special form of struct or class is used to define one-dimensional dynamic
SOAP-encoded arrays in an interface header file for soapcpp2. Each array has a
pointer variable and a member that records the number of elements the pointer
points to in memory.

The general form of the struct or class declaration that contains a one-dimensional dynamic SOAP-encoded array is:

~~~{.cpp}
    struct array_name 
    {
        Type *__ptr;  // pointer to array of elements in memory
        int __size;   // number of elements pointed to 
        int __offset; // optional SOAP 1.1 array offset
        ...           // anything that follows here will be ignored 
    };
~~~

where the `array_name` must be a non-qualified name and `Type` is the type for the elements of the array.  The `__ptr` member points to the array values and `__size` is the array size.  The `__offset` member specifies an optional array offset, when nonzero, see Section \ref onedoffset.

If the `array_name` is qualified with a namespace prefix then the array is not a SOAP-encoded array but rather represents a sequence of XML elements, see Section \ref list.

The soapcpp2-generated deserializer of a one-dimensional dynamic array can deserialize partially transmitted and/or
SOAP-encoded sparse arrays, and even multi-dimensional arrays which will be collapsed
into a one-dimensional array with row-major ordering.

@warning SOAP 1.2 does not support partially transmitted arrays and the `__offset` member of a dynamic array is ignored.

🔝 [Back to table of contents](#)

### One-dimensional dynamic SOAP-encoded arrays with non-zero offsets   {#onedoffset}

The declaration of a dynamic array as described in Section \ref oned  may
include an `int __offset` member. When set to an integer value, the
serializer of the dynamic array will use this member as the start index of the
array and the SOAP-encoded array offset attribute <i>`SOAP-ENC:offset`</i> will appear in the XML message.
Note that array offsets is a SOAP 1.1 specific feature which is not supported
in SOAP 1.2.

For example, the following header file declares a numeric `Vector`
class, which is a dynamic array of floating point values with an index that
starts at 1:

~~~{.cpp}
    // Contents of file "vector.h": 
    class Vector 
    { public:
        float *__ptr; 
        int __size; 
        int __offset; 
        Vector();
        Vector(struct soap *, int n);
        float& operator[](int i); 
        struct soap *soap;
    };
~~~

The implementations of the `Vector` methods are:

~~~{.cpp}
    Vector::Vector()
    {
      this->soap_default(NULL);
    }
    Vector::Vector(struct soap *soap, int n) 
    {
      this->soap = soap;
      __ptr = soap_new_float(soap, n); 
      __size = n; 
      __offset = 1; 
    } 
    float& Vector::operator[](int i) 
    {
      return __ptr[i - __offset]; 
    }
~~~

An example program fragment that serializes a vector of 3 elements:

~~~{.cpp}
    struct soap *soap = soap_new(); 
    Vector v(soap, 3);
    v[1] = 1.0; 
    v[2] = 2.0; 
    v[3] = 3.0; 
    soap_write_Vector(soap, &v);
    soap_destroy(soap);
    soap_end(soap);
    soap_free(soap);
~~~

The output is a partially transmitted array:

<div class="alt">
~~~{.xml}
    <SOAP-ENC:Array SOAP-ENC:arrayType="xsd:float[4]" SOAP-ENC:offset="[1]"> 
      <item>1</item> 
      <item>2</item> 
      <item>3</item> 
    </SOAP-ENC:Array>
~~~
</div>

Note that <i>`xsd:float[4]`</i> is the type and shape of the encoded array, which starts at offset 1 and therefore the element at 0 is omitted.

🔝 [Back to table of contents](#)

### Nested one-dimensional dynamic SOAP-encoded arrays        {#nested}

One-dimensional SOAP-encoded dynamic arrays may be nested.
For example, using `class Vector` declared in the previous section, `class Matrix` is declared:

~~~{.cpp}
    // Contents of file "matrix.h": 
    class Matrix 
    { public: 
        Vector *__ptr; 
        int __size; 
        int __offset; 
        Matrix(); 
        Matrix(struct soap *soap, int n, int m); 
        Vector& operator[](int i); 
        struct soap *soap;
    }; 
~~~

The Matrix type is essentially an array of pointers to arrays which make up the rows of a matrix.
The serialization of the two-dimensional dynamic array in is nested form in XML.

🔝 [Back to table of contents](#)

### Multi-dimensional dynamic SOAP-encoded arrays    {#multid}

A special form of `struct` or `class` is used to define multi-dimensional
dynamic SOAP-encoded arrays. Each array has a pointer variable and a member that records the
number of elements per dimension.  A `K`-dimensional array is declared as:

~~~{.cpp}
    struct array_name 
    {
        Type *__ptr;     // pointer to array of elements in memory
        int __size[K];   // number of elements per dimension
        int __offset[K]; // optional SOAP 1.1 array offset
        ...              // anything that follows here will be ignored 
    };
~~~

where the `array_name` must be a non-qualified name and `Type` is the type for the elements of the array.  The `__ptr` member points to the array values.  The `__size` array specifies the number of array elements per dimension. The `__offset` array specifies an optional offset per dimension.

For example, the following declaration specifies a matrix class:

~~~{.cpp}
    class Matrix 
    { public: 
        float *__ptr; 
        int __size[2]; 
        int __offset[2]; 
    }; 
~~~

By contrast to the matrix class of Section \ref nested  that defines a matrix as an array of pointers to matrix rows, this
class has one pointer to a matrix stored in row-major order.  The size of the matrix is determined by the `__size` member:
`__size[0]` holds the number of rows and `__size[1]` holds the number of columns of the matrix.  Likewise, `__offset[0]` is the row offset and `__offset[1]` is the columns offset.

@warning SOAP 1.2 does not support partially transmitted arrays and the `__offset` member of a dynamic array is ignored.

🔝 [Back to table of contents](#)

### Non-SOAP dynamic arrays        {#list}

An array is serialized as a sequence of XML elements.  By contrast, a SOAP-encoded array is serialized as an element with a sequence of sub-elements, whose tag names are irrelevant to the SOAP processor, see \ref oned.

An array is declared in an interface header file for soapcpp2 as a struct or class with a name that is qualified with
a namespace prefix.  There are two forms. The first form is similar to the SOAP-encoded array declaration that wraps the `__ptr` and `__size` members:

~~~{.cpp}
    struct prefix__array_name 
    {
        Type *__ptr;  // pointer to array of elements in memory
        int __size;   // number of elements pointed to 
        ...           // anything that follows here will be ignored 
    };
~~~

The second form is more generic, because the array can be declared anywhere in the struct or class and multiple arrays can be used as members, each with a `__size` member (`__sizeName` is also allowed) that precedes a pointer member:

~~~{.cpp}
    struct prefix__array_name 
    {
        ...                   // other members that are serialized
        int __size_of_array1; // number of elements pointed to 
        Type1 *array1;        // pointer to array of elements in memory
        ...                   // other members that are serialized
        int __size_of_array1; // number of elements pointed to 
        Type2 *array2;        // pointer to array of elements in memory
        ...                   // other members that are serialized
    };
~~~

The `__size` member should be an `int` type and cannot be a `size_t` type or other integer type.

For example, we define a Map structure that contains a sequence of key-val pairs:

~~~{.cpp}
    struct ns__Map 
    {
        int __size; // number of pairs 
        struct ns__Pair
        {
            char *key;
            char *val;
        } *pair;    // array of pairs
    };
~~~

Since 2.7.16 it is also possible to use a '`$`' as a special marker to annotate a
size member instead of requiring these members to start with `__size`:

~~~{.cpp}
    struct ns__Map 
    {
      $ int size;  // number of pairs 
        struct ns__Pair
        {
            char *key;
            char *val;
        } *pair;    // array of pairs
    };
~~~

The array will be serialized in XML as a sequence of pairs:

<div class="alt">
~~~{.xml}
    <ns:Map> 
      <pair> 
        <key>Joe</key> 
        <val>555 77 1234</val> 
      </pair> 
      <pair> 
        <key>Susan</key> 
        <val>555 12 6725</val> 
      </pair> 
      <pair> 
        <key>Pete</key> 
        <val>555 99 4321</val> 
      </pair> 
    </ns:Map>
~~~
</div>

Deserialization is less efficient compared to a SOAP-encoded array, because the size of the
sequence is not part of the SOAP encoding. Buffering is used by the
deserializer to collect the elements in memory. When the end of the list is reached, the
buffered elements are copied to a newly allocated managed space on the heap for the
dynamic array.

Multiple arrays can be part of a struct or class.  For example:

~~~{.cpp}
    struct ns__Contact 
    {
        char *firstName; 
        char *lastName; 
      $ int nPhones;          // number of Phones
        ULONG64 *phoneNumber; // array of phone numbers 
      $ int nEmails;          // number of emails 
        char **emailAddress;  // array of email addresses 
    };
~~~

The XML serialization of an example `ns__Contact` is:

<div class="alt">
~~~{.xml}
    <ns:Contact> 
      <firstName>Joe</firstName> 
      <lastName>Smith</lastName> 
      <phoneNumber>5551112222</phoneNumber> 
      <phoneNumber>5551234567</phoneNumber> 
      <phoneNumber>5552348901</phoneNumber> 
      <emailAddress>Joe.Smith@mail.com</emailAddress> 
      <emailAddress>Joe@Smith.com</emailAddress> 
    </ns:Contact>
~~~
</div>

For C++, a better alternative to arrays are containers, which are described next.

🔝 [Back to table of contents](#)

### STL containers        {#templates}

The STL containers `std::deque`, `std::list`,
`std::set`, and `std::vector` are serializable in XML by the soapcpp2-generated serializers.

In order to use containers in an interface header file for soapcpp2, import <i>`stldeque.h`</i>, <i>`stllist.h`</i>, <i>`stlset.h`</i>, or <i>`stlvector.h`</i> to enable `std::deque`, `std::list`, `std::set`,
and `std::vector`, respectively.  For example:

~~~{.cpp}
    #import "stlvector.h" 
    class ns__myClass 
    { public: 
        std::vector<int>          number 1:10; // 1 to 10 numbers
        std::vector<std::string> *name   2;    // more than 2 names
    };
~~~

The use of pointer members such as for `name` shown above is possible, but not required.  Also `minOccurs : maxOccurs` and `minOccurs` length constraints can be specified as shown in the example above.
The XML schema that corresponds to the `ns__myClass` type is:

<div class="alt">
~~~{.xml}
    <complexType name="myClass"> 
      <sequence> 
        <element name="number" type="xsd:int" minOccurs="1" maxOccurs="10"/> 
        <element name="name" type="xsd:string" minOccurs="2" maxOccurs="unbounded"/> 
      </sequence> 
    </complexType>
~~~
</div>

You can also implement your own
containers. The containers must be class templates and should define a forward iterator type, and provide the following methods:


*  `void clear()` empty the container;

*  `iterator begin()` return iterator to beginning;

*  `const_iterator begin() const` return const iterator to beginning;

*  `iterator end()` return iterator to end;

*  `const_iterator end() const` return const iterator to end;

*  `size_t size()` return size;

*  `iterator insert(iterator pos, const_reference val)` insert element.

The `iterator` should be a forward iterator with a dereference operator to
access the container's elements, it must be comparable (equal/unequal), and be pre-incrementable (`++it`).  The const iterator is used by its soapcpp2-generated serializer to
send a sequence of XML element values.  The `insert` method is used to populate a container with `Container::iterator i = container.insert(container.end(), val)`.

Here is in example container template class:

~~~{.cpp}
    // simple_vector.h 
    template <class T> 
    class simple_vector 
    { public: 
        typedef T                       value_type; 
        typedef value_type            * pointer; 
        typedef const value_type      * const_pointer; 
        typedef value_type            & reference; 
        typedef const value_type      & const_reference; 
        typedef pointer                 iterator; 
        typedef const_pointer           const_iterator; 
      protected: 
        iterator                        head; 
        iterator                        tail; 
        size_t                          capacity; 
      public: 
                                        simple_vector()       { head = tail = NULL; } 
                                        simple_vector(const simple_vector& v) 
                                                              { operator=(v); } 
                                        ~simple_vector()      { if (head) delete[] head; } 
        void                            clear()               { tail = head; } 
    /* the member functions below are required for serialization of templates */ 
        iterator                        begin()               { return head; } 
        const_iterator                  begin() const { return head; } 
        iterator                        end()                 { return tail; } 
        const_iterator                  end() const { return tail; } 
        size_t                          size() const { return tail - head; } 
        iterator                        insert(iterator pos, const_reference val) 
        {
          if (!head) 
            head = tail = new value_type[capacity = 1]; 
          else if (tail >= head + capacity) 
          {
            iterator i = head; 
            iterator j = new value_type[capacity *= 2]; 
            iterator k = j; 
            while (i < tail) 
              *k++ = *i++; 
            if (pos) 
              pos = j + (pos - head); 
            tail = j + (tail - head); 
            delete[] head; 
            head = j; 
          } 
          if (pos && pos >= head && pos < tail) 
          {
            iterator i = tail; 
            iterator j = i - 1; 
            while (j != pos) 
               *i-- = *j--; 
            *pos = val; 
          } 
          else 
          {
            pos = tail; 
            *tail++ = val; 
          } 
          return pos; 
        } 
        simple_vector& operator=(const simple_vector& v) 
        {
          head = tail = NULL; 
          capacity = v.capacity; 
          if (v.head) 
          {
            head = tail = new value_type[capacity]; 
            iterator i = v.head; 
            while (i != v.tail) 
              *tail++ = *i++; 
          } 
          return *this; 
        } 
    };
~~~

To enable the container, we add the following two lines to the interface header file for soapcpp2:

~~~{.cpp}
    #include "simpleVector.h" 
    template <class T> class simpleVector;
~~~

The container class itself
should not be defined in the interface header file, only the template declaration suffices for soapcpp2 to generate serializers.  Recall that the `#include` directives are not executed by soapcpp2 but simply passed on to the generated source code.  This include specifies in the generated source code where the container is actually defined.

🔝 [Back to table of contents](#)

### Polymorphic dynamic arrays and lists    {#polymorphicarrays}

Polymorphic arrays, that is, arrays of values of any type, can be serialized in XML when
declared as an array of pointers to a base class. For example:

~~~{.cpp}
    class ns__Object 
    { public: 
        ... // members of ns__Object
    }; 
    class ns__Data : public ns__Object 
    { public: 
        ... // members of ns__Data
    }; 
    class ArrayOfObject 
    { public: 
        ns__Object **__ptr; // pointer to array of pointers to base or derived objects 
        int __size;         // size of the array
    }; 
    class ns__Objects 
    { public: 
        std::vector<ns__Object> objects; // vector of base or derived objects 
    };
~~~

The pointers in the array can point to the `ns__Object` base instances or
`ns__Data` derived instances, which will be serialized 
accordingly in XML.  Derived instances are indicated by <i>`xsi:type`</i> attribute in XML with the qualified name of the class, to distinguish derived instances from the base instances.  Without this attribute the deserializer will not instantiate the derived instance but a base instance since there is no identifying information to distinguish the XML forms except for the <i>`xsi:type`</i> attribute.

Since we cannot use dynamic binding to support polymorphism in C, another
mechanism we can use is void pointers .  Here is an example of a polymorphic SOAP-encoded array `ArrayOfObject` and a non-SOAP dynamic array `ns__Objects` that hold values of any serializable type:

~~~{.cpp}
    struct __wrapper 
    {
        int __type;   // identify the type below by SOAP_TYPE_T 
        void *__item; // pointer to data of type T 
    }; 
    struct ArrayOfObject 
    {
        struct __wrapper *__ptr; // pointer to array
        int __size;              // size of the array
    }; 
    struct ns__Objects 
    {
        int __size;                // size of the array
        struct __wrapper *objects; // pointer to array
    };
~~~

This example uses an "invisible" type `__wrapper` and member `__array`, which start with a double underscore.  These names are never visible in serialized XML.  The `__type` member of `__wrapper` is a `SOAP_TYPE_T` value that identifies the type `T` that `__item` points to, see Section \ref void.

🔝 [Back to table of contents](#)

### How to change the tag names of array item elements   {#arrayitems}

The default XML element tag name for array elements is <i>`item`</i>, which can be changed.
The `__ptr` member in a struct or class of a dynamic array may have an optional suffix part that
specifies the name of the element tag in XML.  That is, the suffix is part of the `__ptr` member name:

~~~{.cpp}
    Type *__ptrarray_elt_name
~~~

Consider for example:

~~~{.cpp}
    struct ArrayOfstring 
    {
        char* *__ptrstring;
        int __size;
    };
~~~

The array is serialized as:

<div class="alt">
~~~{.xml}
    <SOAP-ENC:Array SOAP-ENC:arrayType="xsd:string[2]"> 
      <string>Hello</string> 
      <string>World</string> 
    </SOAP-ENC:Array>
~~~
</div>

SOAP 1.1/1.2 does not mandate a specific tag name for SOAP-encoded array elements and the soapcpp2-generated serializers will ignore the name used to itemize SOAP-encoded array values.

🔝 [Back to table of contents](#)

## base64Binary serialization        {#base64binary}

The <i>`base64Binary`</i> XSD type is introduced in an interface header file for soapcpp2 using a struct or class that contains an array of `unsigned char` values:

~~~{.cpp}
    struct xsd__base64Binary 
    {
        unsigned char *__ptr; 
        int __size; 
    };
~~~

The advantage of this struct or class is the ability to serializer raw binary data from memory, since the soapcpp2-generated serializer converts the binary data to/from base64 in XML.

To introduce a new XML schema type derived from <i>`base64Binary`</i> use the same struct or class structure, but with another name.  For example:

~~~{.cpp}
    struct ns__binary
    {
        unsigned char *__ptr; 
        int __size; 
    };
~~~

The resulting XML schema type is:

<div class="alt">
~~~{.xml}
    <simpleType name="binary">
      <restriction base="xsd:base64Binary">
      </restriction>
    </simpleType>
~~~
</div>

🔝 [Back to table of contents](#)

## hexBinary serialization        {#hexbinary}

The <i>`base64Binary`</i> XSD type is introduced in an interface header file for soapcpp2 using a struct or class that contains an array of `unsigned char` values:

~~~{.cpp}
    struct xsd__hexBinary 
    {
        unsigned char *__ptr; 
        int __size; 
    };
~~~

The advantage of this struct or class is the ability to serializer raw binary data from memory, since the soapcpp2-generated serializer converts the binary data to/from hexadecimal in XML.


If a binary type such as `xsd__base64Binary` is already defined, then we can simply use a `typedef` to introduce the hex variant:

~~~{.cpp}
    class xsd__base64Binary
    { public: 
        unsigned char *__ptr; 
        int __size; 
    }; 
    typedef xsd__base64Binary xsd__hexBinary; // serializes into hex content
~~~

This lets soapcpp2 generate <i>`xsd:base64Binary`</i> and <i>`xsd:hexBinary`</i> serializers.

🔝 [Back to table of contents](#)

## SOAP RPC encoded versus document/literal style        {#literal}

SOAP has several styles:

* SOAP RPC encoding uses XML that is restricted to SOAP structures to ensure programming-language interoperability.  Not allowed are values serialized as XML attributes, arrays should be serialized as SOAP-encoded arrays instead of XML element repetitions (i.e. <i>`xsd:sequence maxOccurs="unbounded"`</i> is not allowed), and <i>`xsd:choice`</i> components are not allowed.  Multi-referenced elements are used to serialize data structure graphs.  Because additional SOAP-encoding specific attributes are present that are not defined in the XML schema (of the WSDL), strict XML schema validators may reject SOAP-encoded content.  The SOAP Body contains at most one service operation request element or at most one service operation response element and the encoding style is indicated with the <i>`SOAP-ENV:encodingStyle="..."`</i> attribute in the SOAP Body or one or more of its sub-elements.  This style is specified for the entire service declared under namespace prefix `ns` with:
~~~{.cpp}
    //gsoap ns service style:    rpc
    //gsoap ns service encoding: encoded
~~~

* SOAP document/literal uses XML constrained to the XML schema that defines the XML content.  The serialization of tree-based data structures is accurate in XML.  The serialization of digraph-shaped data structures results in the duplication of data nodes that are co-references.  Cyclic data structures cannot be accurately serialized, but you can use `#SOAP_XML_GRAPH` to force the use of id-ref to accurately serialize digraphs and cyclic data structures.  The SOAP Body may contain any number of XML elements, as if the SOAP Body is the root of an XML document.  No <i>`SOAP-ENV:encodingStyle="..."`</i> attribute should appear in literal content.  This style is specified for the entire service declared under namespace prefix `ns` with:
~~~{.cpp}
    //gsoap ns service style:    document
    //gsoap ns service encoding: literal
~~~

* SOAP RPC literal also uses XML constrained to the XML schema that defines the XML content. The difference with document/literal is that the SOAP Body contains at most one service operation request element or at most one service operation response element. No <i>`SOAP-ENV:encodingStyle="..."`</i> attribute should appear in literal content. This style is specified for the entire service declared under namespace prefix `ns` with:
~~~{.cpp}
    //gsoap ns service style:    rpc
    //gsoap ns service encoding: literal
~~~

Besides `//gsoap ns service style` and `//gsoap ns service encoding` there are also the service operation specific versions `//gsoap ns service method-style`, `//gsoap ns service method-response-style`, `//gsoap ns service method-encoding`, and `//gsoap ns service method-response-encoding` that explicitly specify SOAP RPC encoded, document/literal, or RPC literal style messages for the indicated service operation methods.

To enable SOAP RPC encoding for a particular service operation, use:

~~~{.cpp}
    //gsoap ns service method-style:    webmethod rpc 
    //gsoap ns service method-encoding: webmethod encoded 
    int ns__webmethod(...)
~~~

To enable SOAP RPC encoding for a particular service operation response, use:

~~~{.cpp}
    //gsoap ns service method-response-style:    webmethod rpc 
    //gsoap ns service method-response-encoding: webmethod encoded 
    int ns__webmethod(...)
~~~

Likewise, you can specify document/literal and RPC literal messages.  The default style is document/literal, unless <b>`soapcpp2 -e`</b> option <b>`-e`</b> is used to set SOAP RPC encoding by default.

For the `style` directives you can specify `rpc` or `document`.  For the `encoding` directives you can specify `literal`, `encoded`, or even a custom URI that indicates some custom or proprietary encoding format in XML which will not interoperate with SOAP processors that are not compatible with the specified encoding format.  See also Section \ref directives.

See also
[C and C++ XML data bindings](../../databinding/html/index.html) documentation
for differences in XML serialization when using the SOAP RPC encoded and
document/literal messaging styles.

🔝 [Back to table of contents](#)

### Serializing mixed content with literal XML strings        {#literal2}

XML is stored in "literal" XML strings which are the built-in `_XML` type that is a regular `char*` string or you can declare a wide character string in an interface header file for soapcpp2 as follows:

~~~{.cpp}
    typedef wchar_t *XML;
~~~

To declare a C++ `std::string` literal XML type:

~~~{.cpp}
    typedef std::string XML;
~~~

Or use a wide character string:

~~~{.cpp}
    typedef std::wstring XML;
~~~

To use both at the same time:

~~~{.cpp}
    typedef std::string  XML;
    typedef std::wstring XML_;
~~~

The differences between the use of regular 8-bit strings versus wide character strings for XML documents are:

*  XML literal strings must hold UTF-8 XML content.

*  Wide character XML literal strings are converted to and from UTF-8.

🔝 [Back to table of contents](#)

# XML validation        {#validation}

Some XML validation constraints are not automatically verified unless explicitly
set using the `#SOAP_XML_STRICT` flag.  SOAP RPC encoding is an XML format that does not afford strict XML validation, because of the addition of SOAP-specific attributes and other small deviations that will be detected by an aggressive XML validator, leading the messaging failures.  By toning XML validation down, it helps to improve SOAP RPC encoding interoperability.

Strict validation constraints are enabled with the `#SOAP_XML_STRICT` mode
flag set, e.g. with `soap_set_imode(soap, SOAP_XML_STRICT)` or
`soap_new(SOAP_XML_STRICT)`, see Section \ref flags  for the complete list
of flags. 

The next sections describe the type of constraints validated when `#SOAP_XML_STRICT` is enabled and validation constraints are specified in the interface header file.

Use compiler flag `#WITH_REPLACE_ILLEGAL_UTF8` to force strict UTF-8 text
conversions, which replaces invalid UTF-8 with U+FFFD.

See also
[C and C++ XML data bindings](../../databinding/html/index.html) documentation for more details.

🔝 [Back to table of contents](#)

## Default values  {#defaultvalues}

Default values can be defined for optional elements and attributes, which means
that the default value will be used when the element or attribute value is not
present in the parsed XML. See also Section \ref default  and  examples in
subsequent subsections below.

Default values must be primitive types, integer, float, string, etc. or pointers to primitive types.
Default values can be specified for struct and class members, as shown in the example below:

~~~{.cpp}
    struct ns__MyRecord 
    {
        int n = 5;           // optional element with default value 5 
        char *name = "none"; // optional element with default value "none" 
      @ enum ns__color { RED, WHITE, BLUE } color = RED; // optional attribute with default value RED 
    };
~~~

Upon deserialization of absent data, these members will be set accordingly.
When classes are instantiated with `soap_new_ClassName` the instance will
be initialized with default values.

See also [C and C++ XML data bindings](../../databinding/html/index.html)
documentation for more details.

🔝 [Back to table of contents](#)

## Occurrence constraints  {#occurrence}

Occurrence constraints specify the minimum and/or maximum frequency or optionality of of attributes and elements.

🔝 [Back to table of contents](#)

### Elements with minOccurs and maxOccurs restrictions   {#occurs}

To force the validation of minOccurs and maxOccurs constraints the `#SOAP_XML_STRICT` input mode flag must be set.
The minOccurs and maxOccurs constraints are specified for members of a struct and members of a class in a header file using the following syntax:

~~~{.cpp}
    Type membername nullptr minOccurs : maxOccurs = value;
~~~

The `nullptr` is optional and indicates that the member is nillable (gSOAP version 2.8.24 or greater), which means that when NULL an empty element with <i>`xsi:nil="true"`</i> is added in the serialized XML.

The `minOccurs` and `maxOccurs` are optional values that must be integer literals.  When `maxOccurs` is not specified then the colon can be omitted.
When `minOccurs` is not specified it is assumed to be one (1) for non-pointer members that are elements and zero (0) for members that are pointers or are attributes (i.e. have a `@` qualifier).

A default initialization `value` may be provided and is optional.

A fixed initialization value can be specified with `==` (gSOAP version 2.8.48 or greater).

For example

~~~{.cpp}
    struct ns__MyRecord 
    {
        int n 0 = 5;      // element with default value 5, minOccurs=0, maxOccurs=1
        int m;            // element with minOccurs=1 
        int *k nullptr 1; // element with minOccurs=1 and nillable=true 
        int v == 2;       // element with minOccurs=1 and fixed value 2 
        int __size 0:10;  // sequence <item> with minOccurs=0, maxOccurs=10
        int *item; 
        std::vector<double> nums 2; // sequence <nums> with minOccurs=2, maxOccurs=unbounded 
    }; 

    struct arrayOfint 
    {
        int *__ptr 1:100; // minOccurs=1, maxOccurs=100 
        int size; 
    };
~~~

See also [C and C++ XML data bindings](../../databinding/html/index.html)
documentation for more details.

🔝 [Back to table of contents](#)

### Required and prohibited attributes  {#requiredandprohibited}

Similar to the minOccurs and maxOccurs annotations defined in the previous
section, attributes in a struct or class can be annotated with occurrence
constraints to make them optional (0), required (1), or prohibited (0:0).
Default values can be assigned to optional attributes.

For example

~~~{.cpp}
    struct ns__MyRecord 
    {
      @ int m 1;   // required attribute (occurs at least once) 
      @ int n = 5; // optional attribute with default value 5
      @ int o 0;   // optional attribute (may or may not occur) 
      @ int p 0:0; // prohibited attribute 
    };
~~~

Remember to set the `#SOAP_XML_STRICT` input mode flag to
enable the validation of attribute occurrence constraints.

See also [C and C++ XML data bindings](../../databinding/html/index.html)
documentation for more details.

🔝 [Back to table of contents](#)

## Value constraints   {#valueconstraints}

Value constraints restrict the length of strings and the range of values of numeric types.

🔝 [Back to table of contents](#)

### Data length restrictions   {#lengthconstraints}

A schema simpleType is defined with a `typedef` by taking a base primitive to defined a derived simpleType.  For example:

~~~{.cpp}
    typedef int time__seconds;
~~~

This defines the following schema type in <i>`time.xsd`</i>:

<div class="alt">
~~~{.xml}
    <simpleType name="seconds"> 
      <restriction base="xsd:int"/> 
    </simpleType>
~~~
</div>

A complexType with simpleContent is defined with a wrapper struct/class:

~~~{.cpp}
    struct time__date 
    {
        char *__item; // some custom format date (restriction of string) 
      @ enum time__zone { EST, GMT, ... } zone; 
    }
~~~

This defines the following schema type in <i>`time.xsd`</i>:

<div class="alt">
~~~{.xml}
    <complexType name="date"> 
      <simpleContent> 
        <extension base="xsd:string"/> 
      </simpleContent> 
      <attribute name="zone" type="time:zone" use="optional"/> 
    </complexType>
    <simpleType name="zone"> 
      <restriction base="xsd:string"> 
        <enumeration value="EST"/> 
        <enumeration value="GMT"/> 
        ... 
      </restriction> 
    </simpleType>
~~~
</div>

Data value length constraints of simpleTypes and complexTypes with simpleContent are defined as follows:

~~~{.cpp}
    typedef char *ns__string256 0:256; // simpleType restriction of string with max length 256 characters 
    typedef char *ns__string10 10:10; // simpleType restriction of string with length of 10 characters 
    typedef std::string *ns__string8 8; // simpleType restriction of string with at least 8 characters 
    struct ns__data // simpleContent wrapper 
    {
        char *__item :256; // simpleContent with at most 256 characters 
      @ char *name 1;      // required name attribute 
    }; 
    struct time__date // simpleContent wrapper 
    {
        char *__item :100; 
      @ enum time__zone { EST, GMT, ... } zone = GMT; 
    }
~~~

Set the `#SOAP_XML_STRICT` mode flag to
enable the validation of value length constraints.

See also [C and C++ XML data bindings](../../databinding/html/index.html)
documentation for more details.

🔝 [Back to table of contents](#)

### Value range restrictions   {#valuerangeconstraints}

Similar to data length constraints for string-based data, integer and floating point value range
constraints on numeric simpleTypes and complexTypes with simpleContent are
declared with `low : high`, where `low` and `high` are optional.

As of gSOAP 2.8.26, floating point value ranges and integer ranges can be
exclusive by adding `<` on either side of the `:` range operator:

range          | validation check
-------------- | ------------------
`1           ` | 1 <= x
`1 :         ` | 1 <= x
`  : 10      ` | x <= 10
`1 : 10      ` | 1 <= x <= 10
`1 < : < 10  ` | 1 < x < 10
`1     < 10  ` | 1 < x < 10
`1     : < 10` | 1 <= x < 10 
`      : < 10` | x < 10
`        < 10` | x < 10
`1 < :       ` | 1 < x
`1 <         ` | 1 < x
`1 < : 10    ` | 1 < x <= 10

For example:

~~~{.cpp}
    typedef int ns__int10 0:10; // simpleType restriction of int 0..10 
    typedef LONG64 ns__long -1000000:1000000; // simpleType restriction of long64 -1000000..1000000 
    typedef float ns__float -1.0 <:< 10.5; // simpleType restriction of float in (-1,10.5) 
    struct ns__data // simpleContent wrapper 
    {
        int __item 0:10; // simpleContent range 0..10 
      @ char *name 1;    // required name attribute 
    };
~~~

Set the `#SOAP_XML_STRICT` mode flag to
enable the validation of value range constraints.

See also [C and C++ XML data bindings](../../databinding/html/index.html)
documentation for more details.

🔝 [Back to table of contents](#)

### Pattern restrictions   {#patternconstraints}

Patterns can be defined for simpleType content. However, pattern validation is not enforced unless the `::soap::fsvalidate` and `::soap::fwvalidate` callbacks are set to a regex matcher.

To associate a pattern with a simpleType, you can define a simpleType with a `typedef` and a pattern string:

~~~{.cpp}
    typedef int time__second "[1-5]?[0-9]|60";
~~~

This defines the following schema type in <i>`time.xsd`</i>:

<div class="alt">
~~~{.xml}
    <simpleType name="second"> 
      <restriction base="xsd:int"> 
        <pattern value="[1-5]?[0-9]|60"/> 
      </restriction> 
    </simpleType>
~~~

</div>
The pattern string must contain a valid regular expression.

A special case for C format string patterns is introduced in gSOAP 2.8.18.
When <i>`xsd:totalDigits`</i> and <i>`xsd:fractionDigits`</i> are given in a XSD file,
then a C format string is produced to output floating point values with the
proper precision and scale. For example:

<div class="alt">
~~~{.xml}
    <simpleType name="ratio"> 
      <restriction base="xsd:float"> 
        <totalDigits value="5"/> 
        <fractionDigits value="2"/> 
      </restriction> 
    </simpleType>
~~~
</div>

produces:

~~~{.cpp}
    typedef float time__ratio "%5.2f";
~~~

The format string is used to format the output the floating point value in XML.

See also [C and C++ XML data bindings](../../databinding/html/index.html)
documentation for more details.

🔝 [Back to table of contents](#)

## Element and attribute qualified/unqualified forms   {#qualified}

Struct, class, and union members represent elements and attributes that are
automatically qualified or unqualified depending on the schema element and
attribute default forms specified. The engine always validates the prefixes of
elements and attributes. When a namespace mismatch occurs, the element or
attribute is not consumed which can lead to a validation error (unless the
complexType is extensible or when `#SOAP_XML_STRICT` is turned off).

Consider for example:

~~~{.cpp}
    //gsoap ns schema elementForm: qualified 
    //gsoap ns schema attributeForm: unqualified 
    struct ns__record 
    {
      @ char * type; 
        char * name; 
    };
~~~

Here, the `ns__record` struct is serialized with qualified element `name` and unqualified attribute `type`:

<div class="alt">
~~~{.xml}
    <ns:record type="..."> 
      <ns:name>...</ns:name> 
    </ns:record>
~~~
</div>

The "colon notation" for struct/class/union member names is used to
override element and attribute qualified or unqualified forms. 
To override the form for individual members that represent elements and attributes, use a namespace prefix and colon with the member name:

~~~{.cpp}
    //gsoap ns schema elementForm: qualified 
    //gsoap ns schema attributeForm: unqualified 
    struct ns__record 
    {
      @ char * ns:type; 
        char * :name; 
    };
~~~

where `name` is unqualified and `type` is qualified:

<div class="alt">
~~~{.xml}
    <ns:record ns:type="..."> 
      <name>...</name> 
    </ns:record>
~~~
</div>

The colon notation is a syntactic notation used only in the interface header
file syntax, it is not translated to the C/C++ output.

The colon notation does not avoid name clashes between members. For example:

~~~{.cpp}
    struct x__record 
    {
      @ char * name; 
        char * x:name; 
    };
~~~

results in a redefinition error, since both members have the same name. To avoid name clashes, use a underscore suffix:

~~~{.cpp}
    struct x__record 
    {
      @ char * name; 
        char * x:name_; 
    };
~~~

Not that the namespace prefix convention can be used instead:

~~~{.cpp}
    struct x__record 
    {
      @ char * name; 
        char * x__name; 
    };
~~~

which avoids the name clash. However, the resulting schema is different
since the last example generates a global `name` element definition that is
referenced by the local element.

More specifically, the difference between the namespace prefix convention with double underscores
and colon notation is that the namespace prefix convention generates schema
element/attribute references to elements/attributes at the top level,
while the colon notation only affects the local element/attribute namespace
qualification by form overriding. This is best illustrated by an example:

~~~{.cpp}
    struct x__record 
    {
        char * :name; 
        char * x:phone; 
        char * x__fax; 
        char * y__zip; 
    };
~~~

which generates the following <i>`x.xsd`</i>schema:

<div class="alt">
~~~{.xml}
    <complexType name="record"> 
      <sequence> 
        <element name="name" type="xsd:string" minOccurs="0" maxOccurs="1" nillable="true" form="unqualified"/> 
        <element name="phone" type="xsd:string" minOccurs="0" maxOccurs="1" nillable="true" form="qualified"/> 
        <element ref="x:fax" minOccurs="0" maxOccurs="1"/> 
        <element ref="y:zip" minOccurs="0" maxOccurs="1"/> 
      </sequence> 
    </complexType> 
    <element name="fax" type="xsd:string"/>
~~~
</div>

and the <i>`y.xsd`</i> schema defines contains:

<div class="alt">
~~~{.xml}
    <element name="zip" type="xsd:string"/>
~~~
</div>

See also [C and C++ XML data bindings](../../databinding/html/index.html)
documentation for more details.

🔝 [Back to table of contents](#)

# XML namespaces and the namespace mapping table        {#nstable}

A namespace mapping table should be included in the source code of  client and
service applications.  The mapping table is used by the serializers and
deserializers of the stub and skeleton functions to produce valid XML messages
and to parse and validate XML messages.  A typical mapping table is shown
below:

~~~{.cpp}
    struct Namespace namespaces[] = 
    {
      // { "prefix", "URI", "URI-pattern" (optional) } 
      { "SOAP-ENV", "http://schemas.xmlsoap.org/soap/envelope/" }, // must be first 
      { "SOAP-ENC", "http://schemas.xmlsoap.org/soap/encoding/" }, // must be second 
      { "xsi",      "http://www.w3.org/2001/XMLSchema-instance" }, // must be third 
      { "xsd",      "http://www.w3.org/2001/XMLSchema" },          // must be fourth
      { "ns",       "urn:my-service-URI" }, // binds "ns" namespace prefix to schema URI
      { NULL, NULL } // end of table 
    }; 
~~~

Each namespace prefix used by a identifier name in the header file
specification, see Section \ref idtrans,  must have a binding to a
namespace URI in the mapping table. The end of the namespace mapping table must
be indicated by the NULL pair.  The namespace URI matching is case
insensitive. A namespace prefix is distinguished by the occurrence of a pair
of underscores (`__`) in an identifier or by using colon notation, see Section \ref idtrans.

An optional third column in the namespace mapping table may be specified that contains a namespace URI pattern.
The patterns provide an alternative namespace for the
validation of parsed XML messages.  In this pattern, dashes (`-`) are
single-character wildcards and asterisks (`*`) are multi-character
wildcards.  For example, to accept alternative versions of XML schemas with
different authoring dates, four dashes can be used in place of the specific
dates in the namespace mapping table pattern:

~~~{.cpp}
    struct Namespace namespaces[] = 
    {
      // { "prefix", "URI", "URI-pattern" (optional) } 
      { "SOAP-ENV", "http://schemas.xmlsoap.org/soap/envelope/" }, // must be first 
      { "SOAP-ENC", "http://schemas.xmlsoap.org/soap/encoding/" }, // must be second 
      { "xsi",      "http://www.w3.org/2001/XMLSchema-instance", "http://www.w3.org/----/XMLSchema-instance" }, 
      { "xsd",      "http://www.w3.org/2001/XMLSchema", "http://www.w3.org/----/XMLSchema" }, 
      ... //
      { NULL, NULL } // end of table 
    };
~~~

Or alternatively, asterisks can be used as wildcards for multiple characters:

~~~{.cpp}
    struct Namespace namespaces[] = 
    {
      // { "prefix", "URI", "URI-pattern" (optional) } 
      { "SOAP-ENV", "http://schemas.xmlsoap.org/soap/envelope/" }, // must be first 
      { "SOAP-ENC", "http://schemas.xmlsoap.org/soap/encoding/" }, // must be second 
      { "xsi",      "http://www.w3.org/2001/XMLSchema-instance", "http://www.w3.org/*/XMLSchema-instance" }, 
      { "xsd",      "http://www.w3.org/2001/XMLSchema",          "http://www.w3.org/*/XMLSchema" }, 
      ... //
      { NULL, NULL} // end of table 
    };
~~~

A namespace mapping table is automatically generated with prefixes and URIs in
the table that are declared with `//gsoap <prefix> schema namespace:` directives in
the interface header file, see Section \ref directives.  If directives are not
provided in the header file then default URIs of the form
<i>`http://tempuri.org/prefix.xsd`</i> for each namespace <i>`prefix`</i>.  The
soapcpp2 tool also generates a WSDL and one or more XSD files, one for each XML
namespace.

When parsing XML and deserializing data, namespace URIs in the XML messages are
matched against the second and third column of the namespace mapping table,
searching from the top to the bottom of the table.  The actual prefix used in
the XML message is irrelevant as the URI associated with the prefix is relevant
to define the XML namespace to which a qualified element or attribute belongs.
When a match is found, the namespace prefix in the first column of the table is
considered semantically identical to the namespace prefix used by the qualified
XML element and attribute parsed, even when the prefix names differ.  This
normalization of prefixes is invisible to the user of gSOAP and makes coding
with XML easier.  For example, when XSD QNames are parsed into strings using
the built-in soapcpp2 `_QName` type or a QName declared with `typedef
std::string xsd__QName`, then this QName string will always contain qualified
names with normalized prefixes, i.e. prefixes defined in the namespace mapping
table, unless the table has no entry, see Section \ref qname.

For example, let's say we have the following structs:

~~~{.cpp}
    struct a__elt { ... }; 
    struct b__elt { ... }; 
    struct k__elt { ... }; 
~~~

The namespace mapping table generated by soapcpp2 has the following entries:

~~~{.cpp}
    struct Namespace namespaces[] = 
    {
      // { "prefix", "URI", "URI-pattern" (optional) } 
      ... // the four SOAP and XSD namespace bindings
      { "a", "http://tempuri.org/a.xsd" }, 
      { "b", "http://tempuri.org/b.xsd" }, 
      { "c", "http://tempuri.org/c.xsd" }, 
      ... //
      { NULL, NULL }
    };
~~~

Then, the following XML elements will match these structs:

<div class="alt">
~~~{.xml}
    <x:elt xmlns:x="http://tempuri.org/a.xsd">...</x:elt>
    <elt xmlns="http://tempuri.org/b.xsd">...</elt>
    <zzz:elt xmlns:zzz=http://tempuri.org/c.xsd">...</zzz:elt>
~~~
</div>

Instead of one big namespace table that contains all XML namespace prefixes with their URIs, there are cases when it is desirable to use multiple namespace tables, one for each service.  This avoids leaking namespace prefixes in XML messages that have nothing to do with the service invoked.  In principle there is no harm to leak these namespace prefixes, but the messages are less compact and not as clean. To use multiple namespace tables at the client side is done as follows:

~~~{.cpp}
    struct Namespace namespacesTable1[] = { ... }; 
    struct Namespace namespacesTable2[] = { ... }; 
    struct Namespace namespacesTable3[] = { ... }; 
    struct Namespace *namespaces; 
    ... //
    struct soap *soap = soap_new(); 
    soap_set_namespaces(soap, namespaceTable1); // use the first namespace table
    if (soap_call_remote_method(soap, endpoint, Action, ...))
      ... // error
~~~

Likewise, on the server side call `::soap_set_namespaces` before calling `::soap_serve`.  Changing the namespaces table in service operations has no effect.

The XML messages produced by the gSOAP engine include all <i>`xmlsn`</i> namespace bindings in the root element, which is generally more efficient for larger XML documents in which otherwise the  <i>`xmlsn`</i> namespace bindings will be sprinkled throughout.  By contrast, canonical XML requires <i>`xmlsn`</i> namespace bindings only to be included when utilized.  Therefore, the `#SOAP_XML_CANONICAL` context flag produces C14N exclusive XML messages and documents, which eliminates unused  <i>`xmlsn`</i> namespace bindings from XML.  Unfortunately, the current C14N standard is buggy with respect to XSD QName content, because prefixes used in QName content are not considered utilized.  The gSOAP engine considers QName content prefixes utilized and therefore produces corrected canonicalized XML output that prevents the loss of namespace information for QNames.

🔝 [Back to table of contents](#)

# SOAP Header processing        {#header}

A built-in SOAP Header data structure `::SOAP_ENV__Header` is generated by the
soapcpp2 tool for exchanging SOAP headers in SOAP messages.  This structure is
empty unless headers are added by plugins and headers specified by WSDL
specifications (i.e. wsdl2h adds SOAP Headers).

You can create your own SOAP Header struct simply by declaring it in an
interface header file for soapcpp2 and by adding members that must be qualified
with namespace prefixes to conform to the SOAP Header processing requirements
that SOAP Header elements must be namespace qualified.

For example, assume that transaction data should be piggy-backed with SOAP
messages in SOAP Header:

~~~{.cpp}
    struct t__transaction
    {
        int number;
        const char *dscription;
    };
    struct SOAP_ENV__Header 
    {
        mustUnderstand struct t__transaction *t__transaction; 
    };
    //gsoap ns service method-input-header-part: webmethod t__transaction
    int ns__webmethod(...);
~~~

The `mustUnderstand` qualifier specifies that the element must be processed by
the SOAP processor and cannot be ignored if the processor has no logic in place
for this SOAP header, which is done by adding a
<i>`SOAP-ENV:mustUnderstand="true"`</i> attribute to the <i>`t:transaction`</i>
element.  The soapcpp2-generated serializers obey this safety principle.

This declares a service operation that sends messages with an input SOAP header
`t__transaction`, as can be seen in the generated WSDL binding:

<div class="alt">
~~~{.xml}
    <binding name="Service" type="tns:ServicePortType">
      <SOAP:binding style="document" transport="http://schemas.xmlsoap.org/soap/http"/>
      <operation name="webmethod">
        <SOAP:operation soapAction=""/>
        <input>
              <SOAP:body use="literal" parts="Body"/>
              <SOAP:header use="literal" message="tns:ServiceHeader" part="transaction"/>
        </input>
        <output>
              <SOAP:body use="literal" parts="Body"/>
        </output>
      </operation>
    </binding>
~~~
</div>

Likewise, you can specify that both input and output messages have the same header with `//gsoap ns service method-header-part:` or the output message has a header with `//gsoap ns service method-output-header-part:`.  Multiple headers can be specified this way.

To set up a SOAP Header at the client side that contains the `t__transaction` member:

~~~{.cpp}
    struct soap *soap = soap_new(); 
    ... //
    soap.header = NULL; // erase any SOAP Header we have so far
    soap_header(soap);  // allocate and initialize a new SOAP Header
    soap->header->t__transaction = soap_new_t__transaction(soap, -1);
    soap->header->t__transaction->number = num;
    soap->header->t__transaction->description = "Transactional data";
    if (soap_call_webmethod(soap, endpoint, NULL, ...))
      ... // error
~~~

The SOAP Web service request includes the SOAP Header with the transaction data:

<div class="alt">
~~~{.xml}
    <SOAP-ENV:Envelope ...> 
      <SOAP-ENV:Header> 
        <t:transaction SOAP-ENV:mustUnderstand="true">
          <number>12345</number>
          <description>Transactional data</description>
        </t:transaction> 
      </SOAP-ENV:Header> 
      <SOAP-ENV:Body> 
        <ns:webmethod>
          ...
        </ns:webmethod>
      </SOAP-ENV:Body> 
    </SOAP-ENV:Envelope>
~~~
</div>

At the receiving side, a SOAP Header can be inspected by checking for a non-NULL `::soap::header`.

@warning When SOAP Headers are used, you must make sure to set `::soap::header` to NULL when no SOAP Header should be sent, otherwise any SOAP Headers currently present in the `::SOAP_ENV__Header` struct pointed to by `::soap::header` will be included in the message.

At the client side, the `::soap::actor` string variable can be set to set the SOAP <i>`SOAP-ENV:actor`</i> attribute.
The <i>`SOAP-ENV:mustUnderstand="true"`</i> attribute then indicates the requirement that the recipient 
corresponding to the <i>`SOAP-ENV:actor`</i> attribute value is responsible to process the SOAP Header element.  The details of which can be found in the SOAP 1.1/1.2 specifications that the gSOAP tools conform to.

The SOAP Header structure `::SOAP_ENV__Header` is declared `mutable`, which means that re-declarations of the structures are permitted and additional members are collected into one final structure by the soapcpp2 tool.

For another example, consider:

~~~{.cpp}
    struct SOAP_ENV__Header 
    {
        char *h__transaction; 
        struct UserAuth *h__authentication; 
    };
~~~

Suppose method `ns__login` uses both header parts (at most), then this is declared as:

~~~{.cpp}
    //gsoap ns service method-header-part: login h__transaction 
    //gsoap ns service method-header-part: login h__authentication 
    int ns__login(...);
~~~

Suppose method `ns__search` uses only the first header part, then this is declared as:

~~~{.cpp}
    //gsoap ns service method-header-part: search h__transaction 
    int ns__search(...);
~~~

Note that the method name and header part names must be namespace qualified.
The headers must be present in all operations that declared the header parts.

To specify the header parts for the method input (method request message), use:

~~~{.cpp}
    //gsoap namespace-prefix service method-input-header-part: method-name header-part
~~~

Similarly, to specify the header parts for the method output (method response message), use:

~~~{.cpp}
    //gsoap namespace-prefix service method-output-header-part: method-name header-part
~~~

The declarations only affect the WSDL.
For example:

~~~{.cpp}
    struct SOAP_ENV__Header 
    {
        char *h__transaction; 
        struct UserAuth *h__authentication; 
    }; 
    //gsoap ns service method-input-header-part: login h__authentication 
    //gsoap ns service method-input-header-part: login h__transaction 
    //gsoap ns service method-output-header-part: login h__transaction 
    int ns__login(...);
~~~

The headers must be present in all operations that declared the header parts.

See also API documentation Module \ref group_header.

🔝 [Back to table of contents](#)

# SOAP Fault processing        {#fault}

A built-in SOAP Fault data structure `::SOAP_ENV__Fault` is generated by the soapcpp2 tool for exchanging exception messages.
This structure has the general form:

~~~{.cpp}
    struct SOAP_ENV__Fault 
    {
        _QName faultcode; // _QName is built-in 
        char *faultstring; 
        char *faultactor; 
        struct SOAP_ENV__Detail *detail; 
        struct SOAP_ENV__Code *SOAP_ENV__Code; // must be a SOAP_ENV__Code struct defined below 
        char *SOAP_ENV__Reason; 
        char *SOAP_ENV__Node; 
        char *SOAP_ENV__Role; 
        struct SOAP_ENV__Detail *SOAP_ENV__Detail; // SOAP 1.2 detail member 
    }; 
    struct SOAP_ENV__Code 
    {
        _QName SOAP_ENV__Value; 
        struct SOAP_ENV__Code *SOAP_ENV__Subcode; 
    }; 
    struct SOAP_ENV__Detail 
    {
        int __type;  // The SOAP_TYPE_ of the object serialized as Fault detail 
        void *fault; // pointer to the fault object, or NULL 
        _XML __any;  // any other detail element content (stored in XML format) 
    };
~~~

The first four members in `::SOAP_ENV__Fault` are SOAP 1.1 specific. The last five members are SOAP 1.2 specific.
You can redefine these structures in the interface header file for soapcpp2. For example, you can use a `class` for the `::SOAP_ENV__Fault` and add methods for convenience.

The `::SOAP_ENV__Detail` structure can be changed to the needs of Web service application to communicate specific fault data structures, but this is generally not necessary because the application-specific SOAP Fault details can be serialized via the `__type` and `fault` members in the `::SOAP_ENV__Detail` member, see Section \ref void  on the serialization of data referred to by `__type` and `fault`.

When a user-define service operation function returns an error with `::soap_sender_fault` or `::soap_receiver_fault`, then the SOAP Fault structure is populated and `::soap::fault` points to this SOAP Fault.  The SOAP Fault is sent to the client.  The client populates a SOAP Fault structure that contains the SOAP Fault message with details.

Server-side faults are raised with `::soap_sender_fault` or `::soap_receiver_fault`. The `::soap_sender_fault` call should be used to inform that the sender is at fault and the sender (client) should not re-send the request. The `::soap_receiver_fault` call should be used to indicate a temporary server-side problem, so a sender (client) can re-send the request later. For example:

~~~{.cpp}
    int ns1__myMethod(struct soap *soap, ...) 
    {
      ... //
      return soap_receiver_fault(soap, "Resource temporarily unavailable", NULL); // return fault to sender 
    }
~~~

In the example, the SOAP Fault details were empty (NULL). You may pass an XML fragment, which will be literally included in the SOAP Fault message. For WS-I Basic Profile compliance, you must pass an XML string with one or more namespace qualified elements, such as:

~~~{.cpp}
    return soap_receiver_fault(soap, "Resource temporarily unavailable", "<errorcode xmlns='http://tempuri.org'>123</errorcode><errorinfo xmlns='http://tempuri.org'>abc</errorinfo>");
~~~

When a service operation needs to populate SOAP Fault details with a application-specific data, it does so by assigning the `::soap::fault` member of the current reference to the
context with 
appropriate data associated with the exception and by returning the error `#SOAP_FAULT`.
For example:

~~~{.cpp}
    soap_receiver_fault(soap, "Error message", NULL); 
    if (soap->version == 2) // SOAP 1.2 is used 
    {
      soap->fault->SOAP_ENV__Detail = soap_new_SOAP_ENV__Detail(soap, -1);
      soap->fault->SOAP_ENV__Detail->__type = SOAP_TYPE_ns1__myStackDataType; // stack type 
      soap->fault->SOAP_ENV__Detail->fault = sp;                              // point to stack 
      soap->fault->SOAP_ENV__Detail->__any = NULL;                            // no other XML data 
    } 
    else 
    {
      soap->fault->detail = soap_new_SOAP_ENV__Detail(soap, -1); 
      soap->fault->detail->__type = SOAP_TYPE_ns1__myStackDataType; // stack type 
      soap->fault->detail->fault = sp;                              // point to stack 
      soap->fault->detail->__any = NULL;                            // no other XML data 
    } 
    return SOAP_FAULT; // return from service operation call with the fault
~~~

Here, `::soap_receiver_fault` allocates a fault struct then we set the SOAP Fault details as shown.

Note that SOAP 1.2 supports nested fault sub-codes.  These can be set as follows:

~~~{.cpp}
    struct SOAP_ENV__Code *subcode1 = soap_new_SOAP_ENV__Code(soap);
    struct SOAP_ENV__Code *subcode2 = soap_new_SOAP_ENV__Code(soap);
    soap_sender_fault(soap, "The requested profile token ProfileToken does not exist.", NULL); 
    subcode1->SOAP_ENV__Value = (char*)"ter:InvalidArgs"; // a QName value
    subcode1->SOAP_ENV__Subcode = subcode2;
    subcode2->SOAP_ENV__Value = (char*)"ter:NoProfile"; // a QName value
    subcode2->SOAP_ENV__Subcode = NULL;
    soap->fault->SOAP_ENV__Code->SOAP_ENV__Subcode = subcode1;
    return SOAP_FAULT;
~~~

This produces:

<div class="alt">
~~~{.xml}
    <SOAP-ENV:Fault>
      <SOAP-ENV:Code>
        <SOAP-ENV:Value>SOAP-ENV:Sender</SOAP-ENV:Value>
        <SOAP-ENV:Subcode>
          <SOAP-ENV:Value>ter:InvalidArgs</SOAP-ENV:Value>
          <SOAP-ENV:Subcode>
            <SOAP-ENV:Value>ter:NoProfile </SOAP-ENV:Value>
          </SOAP-ENV:Subcode>
        </SOAP-ENV:Subcode>
      </SOAP-ENV:Code>
      <SOAP-ENV:Reason>
        <SOAP-ENV:Text xml:lang="en">The requested profile token ProfileToken does not exist.</SOAP-ENV:Text>
      </SOAP-ENV:Reason>
    </SOAP-ENV:Fault>
~~~
</div>

Service operations implementation in a Web service application can return various SOAP Faults customized in this way.

SOAP Fault structures are declared `mutable`, which means that re-declarations of the structures are permitted and additional members are collected into one final structure by the soapcpp2 tool.

For an example that used the SOAP Fault detail structure:

~~~{.cpp}
    struct SOAP_ENV__Detail 
    {
        const char *f__invalid;
        const char *f__unavailable;
        int __type;
        void *fault;
        _XML __any;
    }; 
    //gsoap ns service method-fault: login f__invalid 
    //gsoap ns service method-fault: login f__unavailable 
    int ns__login(...);
~~~

See also API documentation Module \ref group_fault.

🔝 [Back to table of contents](#)

# MIME attachments        {#MIME}

The gSOAP toolkit supports MIME attachments as per SOAP with Attachments (SwA)
specification <http://www.w3.org/TR/SOAP-attachments>.

MTOM attachments that are essentially MIME attachments that conform to the
MTOM specification <http://www.w3.org/TR/soap12-mtom> are also supported which
are the preferred way to include MIME attachments with SOAP messages, see
Section \ref MTOM.

In the following discussion, MIME attachment data is assumed to be resident in
memory for sending operations and MIME attachments received will be stored in
memory managed by the context. MTOM and DIME attachments on the other hand can
be streamed and therefore MTOM/DIME attachment data does not need to be stored
in memory, see Sections \ref DIME  and \ref MTOM .

Transmitting multipart/related MIME attachments with a SOAP/XML message is
accomplished with two functions, `::soap_set_mime` and
`::soap_set_mime_attachment`. The first function is for initialization
purposes and the latter function is used to specify meta data and content data
for each attachment.

See also API documentation Module \ref group_mime.

🔝 [Back to table of contents](#)

## Sending a collection of MIME attachments (SwA)  {#SWAsending}

The following functions are used to set up a collection of
multipart/related MIME attachments for transmission with a SOAP or XML message.

* `void soap_set_mime(struct soap *soap, const char *boundary, const char *start)` 
  This function enables sending MIME attachments.  This function is generally not required when the context is initialized with `#SOAP_ENC_MIME`, because MIME attachments are automatically detected as `::xsd__base64Binary` and `::_xop__Include` structures in the data to serialize as an XML message with the attachments automatically added or MIME attachments can be explicitly added with `::soap_set_mime_attachment`. Parameter `boundary` specifies a MIME boundary string or NULL to have the engine generate a MIME boundary string.  Parameter `start` specifies the start content ID for the first MIME body containing the SOAP or XML message.  When NULL, the start ID of the SOAP message is <i>`<SOAP-ENV:Envelope>`</i>.

* `int soap_set_mime_attachment(struct soap *soap, char *ptr, size_t size, enum soap_mime_encoding encoding, const char *type, const char *id, const char *location, const char *description)` 
  This function adds a MIME attachment to a SOAP/XML message to send.  The specified `ptr` points to the data to send of length specified by `size`.  The `encoding` parameter is a `::soap_mime_encoding` value that is recommended to be specified as `#SOAP_MIME_NONE` to specify that the MIME data content is not encoded in any way (the MIME attachment function simply copies the raw data to the MIME block without encoding).  The `type` parameter is required and indicates the MIME type of the data, such as "image/jpg".  The `id` parameter uniquely identifies the attachment in the message, which can be omitted by specifying NULL.  The `location` parameter specifies a location string or NULL.  The `description` parameter is a string that describes the data or NULL.  Returns `#SOAP_OK` or a `::soap_status` error code.

* `void soap_clr_mime(struct soap *soap)` 
  This function disables MIME attachments such as after sending a multipart-related message with attachments to switch back to non-multipart-related messaging, unless the data to serialize as a message contains attachments such as `::xsd__base64Binary` for MIME attachments and `::_xop__Include` for MTOM attachments.

When providing a MIME boundary with `::soap_set_mime`, you have to make
sure the boundary cannot match any parts of the message and attachments that you are sending, because the boundary delineates the attachments.

The internal list of attachments specified with `::soap_set_mime_attachment` is destroyed with `::soap_end` or when a message is received.  Therefore, call `::soap_set_mime_attachment` to set attachments before sending a message.

The following example shows how a multipart/related HTTP message with three
MIME attachments is set up and transmitted to a server. 
In this example we let the message body refer to the attachments using
XML <i>`href`</i> attributes.  The `struct claim__form` data type includes a
definition of a `href` attribute for this purpose.

~~~{.cpp}
    struct claim__form form1, form2; 
    form1.href = "cid:claim061400a.tiff@claiming-it.com"; 
    form2.href = "cid:claim061400a.jpeg@claiming-it.com"; 
    /* initialize and enable MIME */ 
    soap_set_mime(soap, NULL, "<claim061400a.xml@claiming-it.com>"); 
    /* add a base64 encoded tiff image (tiffImage points to base64 data) */ 
    soap_set_mime_attachment(soap, tiffImage, tiffLen, SOAP_MIME_BASE64, "image/tiff", "<claim061400a.tiff@claiming-it.com>", NULL, NULL); 
    /* add a raw binary jpeg image (jpegImage points to raw data) */ 
    soap_set_mime_attachment(soap, jpegImage, jpegLen, SOAP_MIME_BINARY, "image/jpeg", "<claim061400a.jpeg@claiming-it.com>", NULL, NULL); 
    /* send the forms as MIME attachments with this invocation */ 
    if (soap_call_claim__insurance_claim_auto(soap, form1, form2, ...)) 
      ... // an error occurred 
    else 
      ... // process the response
    soap_clr_mime(soap);
~~~

The `claim__form` struct is declared in the interface header file as:

~~~{.cpp}
    struct claim__form 
    {
      @ char *href; 
    };
~~~

The claim forms in
the message consist of <i>`href`</i>s to the claim forms attached.
The use of <i>`href`</i> or other attributes for referring to the MIME attachments
is optional according to the SwA standard.  MTOM on the other hand mandates the use of <i>`href`</i> with XOP elements.

To associate MIME attachments with the request and response of a service operation in the generated WSDL, please see Section \ref MIMEWSDL .

The server-side code to transmit MIME attachments back to a client:

~~~{.cpp}
    int claim__insurance_claim_auto(struct soap *soap, ...) 
    {
      const char *htmlDoc = ...; // an HTML message to send as an attachment
      soap_set_mime(soap, NULL, NULL); // enable MIME
      // add a HTML document (htmlDoc points to data, where the HTML doc is stored in compliance with 7bit encoding RFC2045) 
      if (soap_set_mime_attachment(soap, htmlDoc, strlen(htmlDoc), SOAP_MIME_7BIT, "text/html", "<claim061400a.html@claiming-it.com>", NULL, NULL)) 
      {
        soap_clr_mime(soap); // don't want fault with attachments 
        return soap->error; 
      } 
      return SOAP_OK; 
    }
~~~

🔝 [Back to table of contents](#)

## Retrieving a collection of MIME/MTOM attachments (SwA)  {#SWAreceiving}

MIME attachments are automatically parsed and stored in memory managed by the `::soap` context.
After receiving a set of MIME/MTOM attachments, either at the client-side or
the server-side, the list of MIME/MTOM attachments can be traversed to extract
meta data and the attachment content. The first attachment in the collection of
MIME/MTOM attachments always contains meta data about the SOAP message
itself (because the SOAP message was processed the attachment does not contain
any useful data).

To traverse the list of MIME attachments in C, you use a loop similar to:

~~~{.cpp}
    int n = 0;
    struct soap_multipart *attachment;
    for (attachment = soap->mime.list; attachment; attachment = attachment->next)
    {
      ++n;
      printf("Part %d:\n", n);
      printf("ptr        =%p\n", attachment->ptr);
      printf("size       =%ul\n", attachment->size);
      printf("id         =%s\n", attachment->id ? attachment->id : "");
      printf("type       =%s\n", attachment->type ? attachment->type : "");
      printf("location   =%s\n", attachment->location ? attachment->location : "");
      printf("description=%s\n", attachment->description ? attachment->description : "");
    }
~~~

C++ programmers can use an iterator instead:

~~~{.cpp}
    int n = 0;
    for (soap_multipart::iterator i = soap->mime.begin(); i != soap->mime.end(); ++i)
    {
      ++n;
      printf("Part %d:\n", n);
      printf("ptr        =%p\n", i->ptr);
      ... // etc
    }
~~~

Note: keep in mind that the first attachment is associated with the SOAP
message and you may want to ignore it.

A call to `::soap_end` removes all of the received MIME data. To preserve an
attachment in memory, use `::soap_unlink` on the `ptr` member of the
`::soap_multipart` struct. The `::soap_unlink` function can be
used to prevent deallocation of deserialized data.

🔝 [Back to table of contents](#)

# DIME attachments        {#DIME}

Applications developed with the gSOAP tools can transmit binary DIME attachments stored in memory or in streaming mode by fetching data from a resource or sending data to a resource using callback functions.
The maximum DIME attachment size is limited to 8 MB by default as set with `#SOAP_MAXDIMESIZE`. This limit can be changed as needed. With streaming DIME using callback functions, data handlers are used to pass the
data to and from a resource from which to fetch the data to send or data to store, such as a file or device.  See Section \ref DIMEstreaming .

For details on DIME attachments, see
<http://msdn.microsoft.com/library/en-us/dnglobspec/html/draft-nielsen-dime-02.txt>

See also API documentation Module \ref group_dime.

🔝 [Back to table of contents](#)

## Sending a collection of DIME attachments  {#DIMEsending}

The following functions can be used to explicitly set up a collection of
DIME attachments for transmission with a message.
These attachments can be streamed, as described in
Section \ref DIMEstreaming .  Without streaming, each attachment must refer
to a block of data in memory.

* `void soap_set_dime(struct soap *soap)` 
  This function enables sending DIME attachments.  This function is generally not required because DIME attachments are automatically detected as `::xsd__base64Binary` and `::_xop__Include` structures in the data to serialize as an XML message with the attachments automatically added or DIME attachments can be explicitly added with `::soap_set_dime_attachment`.

* `int soap_set_dime_attachment(struct soap *soap, char *ptr, size_t size, const char *type, const char *id, unsigned short optype, const char *option)` 
  This function adds a DIME attachment to the XML message to send.  The specified `ptr` points to the data to send of length specified by `size`.  The `type` parameter indicates the MIME type of the data or can be NULL.  The `id` parameter uniquely identifies the attachment in the message, which can be omitted by specifying NULL.  The `option` parameter is an option such as a description of the data and `optype` is a user-defined option type (as per DIME option specification format).  The `ptr` parameter must be persistent.  The `ptr` parameter passed to this function must be persistent in memory until the attachment was sent.  Returns `#SOAP_OK` or a `::soap_status` error code.

* `void soap_clr_dime(struct soap *soap)` 
  This function disables DIME attachments, unless the data to serialize as an XML message contains attachments defined by `::xsd__base64Binary` and `::_xop__Include` structures.

These functions allow DIME attachments to be added to SOAP messages without requiring message
body references. This is also referred to as the open DIME attachment style.
The closed attachment style requires all DIME attachments to be referenced from
the SOAP message body with <i>`href`</i> (or similar) references. For the closed
style, gSOAP supports an automatic binary data serialization method, see
Section \ref DIMEbinary .

🔝 [Back to table of contents](#)

## Retrieving a collection of DIME attachments   {#DIMEreceiving}

DIME attachments are automatically parsed and stored in memory (or passed to
the streaming handlers via the DIME callback functions when defined).  After receiving a set of DIME
attachments, either at the client-side or the server-side, the list of DIME
attachments can be traversed to extract meta data and the attachment content.

To traverse the list of DIME attachments in C, you use a loop similar to:

~~~{.cpp}
    int n = 0;
    struct soap_multipart *attachment;
    for (attachment = soap->dime.list; attachment; attachment = attachment->next)
    {
      ++n;
      printf("Part %d:\n", n);
      printf("ptr        =%p\n", attachment->ptr);
      printf("size       =%ul\n", attachment->size);
      printf("id         =%s\n", attachment->id ? attachment->id : "");
      printf("type       =%s\n", attachment->type ? attachment->type : "");
    }
~~~

C++ programmers can use an iterator instead:

~~~{.cpp}
    int n = 0;
    for (soap_multipart::iterator i = soap->dime.begin(); i != soap->dime.end(); ++i)
    {
      ++n;
      printf("Part %d:\n", n);
      printf("ptr        =%p\n", i->ptr);
      ... // etc
    }
~~~

The `options` member is available as well, but not shown in the code above. The `options` content is
formatted according to the DIME specification: the first two bytes are reserved
for the option type, the next two bytes store the size of the option data,
followed by the (binary) option data.

A call to `::soap_end` removes all of the received DIME data. To preserve an
attachment in memory, use `::soap_unlink` on the `ptr` member of the
`::soap_multipart` struct. The `::soap_unlink` function can
be used to prevent deallocation of deserialized data.

🔝 [Back to table of contents](#)

## Serializing binary data with DIME attachments        {#DIMEbinary}

Binary data stored in extended `::xsd__base64Binary` and `::xsd__hexBinary`
types can be serialized and deserialized as DIME attachments when one or more of the extra members `id`, `type`, and `options` are non-NULL. These attachments
will be automatically transmitted prior to the sequence of secondary DIME attachments defined
by the user with `::soap_set_dime_attachment` as explained in the
previous section. The serialization process is automated for SOAP encoded
messages and DIME attachments will be send even when `::soap_set_dime` or
`::soap_set_dime_attachment` are not used.  For non-SOAP-encoded messages
such as document/literal messages you must still call `::soap_set_dime` to
enable sending messages with attachments.

To enable serialization of the data as DIME attachments instead of inline
base64, we extend the `::xsd__base64Binary` type with three additional members:

~~~{.cpp}
    struct xsd__base64Binary 
    {
        unsigned char *__ptr; 
        int __size; 
        char *id; 
        char *type; 
        char *options; 
    };
~~~

The `id` member is for attachment
referencing, typically a content id (CID) or a UUID which can be obtained with `::soap_rand_uuid`, a `type` member is used
to specify the MIME type of the data, the `options` member is used to
piggy-back additional information with a DIME attachment.  The order of the
declaration of the members is significant. In addition, no other members or
methods may be declared before any of these members in the struct/class, but
additional members and methods may appear after the member declarations. The
extended `::xsd__hexBinary` type is similar.

The `id` and `type` members contain text. The set the DIME-specific
options member, you can use the `::soap_dime_option` function 
`char *soap_dime_option(struct soap *soap, unsigned short type, const char *option)`.

This function returns a string with this encoding. For example

~~~{.cpp}
    struct xsd__base64Binary image; 
    image.__ptr = ...; 
    image.__size = ...; 
    image.id = "uuid:09233523-345b-4351-b623-5dsf35sgs5d6"; 
    image.type = "image/jpeg"; 
    image.options = soap_dime_option(soap, 0, "My wedding picture");
~~~

When either the `id` or `type` member values are non-NULL at run time,
the data will be serialized as a DIME attachment.

The SOAP/XML message refers
to the attachments using <i>`href`</i> attributes. This generally works will with
SOAP RPC encoded messaging, because <i>`href`</i> attributes are permitted. However, with document/literal style the referencing mechanism must be explicitly defined
in the schema of the binary type. Therefore, MTOM is the preferred attachment mechanism for document/literal style messaging.

The declaration of an extended binary type in the interface header file for soapcpp2 is:

~~~{.cpp}
    struct ns__myBinaryDataType 
    {
        unsigned char *__ptr; 
        int __size; 
        char *id; 
        char *type; 
        char *options; 
    };
~~~

C++ programmers can use inheritance instead of textual extension required in C:

~~~{.cpp}
    class xsd__base64Binary 
    { public:
        unsigned char *__ptr; 
        int __size; 
    }; 
    class ns__myBinaryDataType : public xsd__base64Binary 
    { public:
        char *id; 
        char *type; 
        char *options; 
    };
~~~

This defines an extension of <i>`xsd:base64Binary`</i>, such that the data can be 
serialized as DIME attachments using <i>`href`</i> attributes for referencing.
When a different attribute is to be used, this must be explicitly defined:

~~~{.cpp}
    //gsoap WSref schema import: http://schemas.xmlsoap.org/ws/2002/04/reference/ 
    struct ns__myBinaryDataType 
    {
        unsigned char *__ptr; 
        int __size; 
        char *id; 
        char *type; 
        char *options; 
      @ char *WSref__location; 
    };
~~~

The example above uses the <i>`location`</i> attribute defined in the content reference schema, as defined in one of the vendor's specific WSDL extensions for DIME <http://www.gotdotnet.com/team/xml_wsspecs/dime/WSDL-Extension-for-DIME.htm>.

When receiving DIME attachments, the DIME meta data and binary content are
stored in the specified `::xsd__base64Binary` and `::xsd__hexBinary` binary data types only when the XML parts of the message uses
<i>`href`</i> attributes to refer to these attachments.  If so, the binary data type `__ptr` and `__size` members are set to the location in memory of the attachment data and length, respectively.

Messages may contain binary data that references external resources not
provided as attachments. In that case, the `__ptr` member is NULL and the
`id` member refers to the external data source.

The `dime_id_format` attribute of the current context
can be set to the default format of DIME id members.  The format string must
contain a `%d` format specifier (or any other `int`-based format
specifier). The value of this specifier is a non-negative integer, with zero
being the value of the DIME attachment id for the SOAP message.  For example,

~~~{.cpp}
    struct soap *soap = soap_new();
    soap->dime_id_format = "id-%x"; 
~~~

As a result, all attachments with a NULL `id` member will use a
auto-generated id value based on the format string.

@warning Care must be taken not to introduce duplicate content id values,
when assigning content id values to the id members of DIME extended binary data
types. Content ids must be unique.

🔝 [Back to table of contents](#)

## Streaming DIME        {#DIMEstreaming}

Streaming DIME is achieved with callback functions to fetch and store data
during transmission.  Three function callbacks for streaming DIME output and
three callbacks for streaming DIME input are available.

* `void *(*soap.fdimereadopen)(struct soap *soap, void *handle, const char *id, const char *type, const char *options)` 
This callback is called by the engine to start sending a streaming DIME attachment.  This callback opens a stream to start reading the attachment data to send.  The actual data stream will be read in chunks using the `::soap::fdimeread` callback until no more data is available and the `::soap::fdimereadclose` callback is called to close the stream.  The `handle` parameter contains the value of the `__ptr` member variable of the attachment struct/class with data (e.g. `::xsd__base64Binary` or `::_xop__Include` with `__ptr`, `__size`, `id`, `type` and `options` members), which should be a pointer to specific information such as a file descriptor or a pointer to a some application-specific data to be passed to this callback.  Both the `__ptr` and `__size` members of the attachment struct/class should have been set by the application prior to the serialization of the message with attachments.  If the `__size` is zero and HTTP chunking is enabled (with `#SOAP_IO_CHUNK`), then chunked DIME attachments are sent, see `::soap::fdimeread`.  The `id`, `type` and `options` parameters are the `id` (optional ID), `type` (a MIME type) and `options` (DIME options are set with `::soap_dime_option`) of the attachment struct/class, respectively, of which at least one member should be non-NULL.  The callback should return the `handle` parameter value or another pointer value, which is passed as the new `handle` parameter to `::soap::fdimeread` and `::soap::fdimereadclose` callbacks.  When an error occurred in this callback, the callback should return NULL and set `::soap::error` to an error code, e.g. using `::soap_receiver_fault`.  The callback may return NULL and set `::soap::error` to `#SOAP_OK` when this specific DIME attachment should not to be streamed and the engine will simply skip it.

* `size_t (*soap.fdimeread)(struct soap *soap, void *handle, char *buf, size_t len)` 
This callback is called by the engine to read a chunk of attachment data to transmit.  The `handle` parameter contains the handle returned by the `::soap::fdimereadopen` callback.  The `buf` parameter is the buffer of length `len` into which a chunk of data should be written by the callback.  The actual amount of data written into the buffer may be less than `len` and this actual amount should be returned by the callback.  A return value of zero indicates an error and `::soap::error` should be set.  The `__size` member variable of the attachment struct/class with data (e.g. `::xsd__base64Binary` or `::_xop__Include` with `__ptr`, `__size`, `id`, `type` and `options` members) should be set by the application prior to the serialization of the message with attachments.  The value of `__size` indicates the total size of the attachment data to be transmitted.  If the `__size` member variable is zero and HTTP chunking is enabled (with `#SOAP_IO_CHUNK`), then DIME chunked transfers are activated by the engine, which is more flexible since the attachment data size does not need to be determined in advance.  To use DIME chunked transfers, enable HTTP chunking with `#SOAP_IO_CHUNK` (also `#SOAP_IO_STORE` can be used, but this buffers the entire message in memory before transmission) and set the `__size` member variable of the attachment struct/class to zero.  When DIME attachment chunking is enabled, this callback should completely fill the `buf` buffer with `len` bytes unless the last data chunk is reached and fewer bytes are returned.

* `void (*soap.fdimereadclose)(struct soap *soap, void *handle)` 
This callback is called by the engine to close the DIME attachment stream after reading.  The `handle` parameter contains the handle returned by the `::soap::fdimereadopen` callback.

* `void *(*soap.fdimewriteopen)(struct soap *soap, const char *id, const char *type, const char *options)` 
Called by the to start receiving a streaming DIME attachment.  This callback opens a stream to start writing the attachment data received.  The actual data stream will be written in chunks using the `::soap::fdimewrite` callback until no more data is available and the `::soap::fdimewriteclose` callback is called to close the stream.  The `id`, `type` and `options` parameters are the `id`, `type` and `options` of the attachment struct/class (e.g. `::xsd__base64Binary` or `::_xop__Include` with `__ptr`, `__size`, `id`, `type` and `options` members), respectively.  The callback should return a handle, which is passed to the `::soap::fdimewrite` and `::soap::fdimewriteclose` callbacks.  The `__ptr` member variable of the attachment struct/class is set by the engine to the value of this handle.  The `__size` member variable is set to the size of the attachment received.  The maximum DIME attachment size received is limited by `#SOAP_MAXDIMESIZE`.

* `int (*soap.fdimewrite)(struct soap *soap, void *handle, const char *buf, size_t len)` 
This callback is called by the engine to write a chunk of attachment data received.  The `handle` parameter contains the handle returned by the `::soap::fdimewriteopen` callback.  The `buf` parameter contains the data of length `len`.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.

* `void (*soap.fdimewriteclose)(struct soap *soap, void *handle)` 
This callback is called by the engine to close the DIME attachment stream after writing.  The `handle` parameter contains the handle returned by the `::soap::fdimewriteopen` callback.

In addition, a `void* ::soap::user` member 
is available to pass user-defined data to the callbacks.  This way, you can set
`void* ::soap::user` to point to application data that the callbacks need such as a
file name for example.

The following example illustrates the client-side initialization of an image
attachment struct to stream a file into a DIME attachment:

~~~{.cpp}
    int main() 
    {
      struct soap soap; 
      struct xsd__base64Binary image; 
      FILE *fd; 
      struct stat sb; 
      soap_init(&soap); 
      if (!fstat(fileno(fd), &sb) && sb.st_size > 0) 
      {
        // because we can get the length of the file, we can stream it 
        soap.fdimereadopen = dime_read_open; 
        soap.fdimereadclose = dime_read_close; 
        soap.fdimeread = dime_read; 
        image.__ptr = (unsigned char*)fd; // must set to non-NULL (this is our fd handle which we need in the callbacks) 
        image.__size = sb.st_size; // must set size 
      } 
      else 
      {
        // don't know the size, so buffer it 
        size_t i; 
        int c; 
        image.__ptr = (unsigned char*)soap_malloc(&soap, MAX_FILE_SIZE); 
        for (i = 0; i < MAX_FILE_SIZE; i++) 
        {
          if ((c = fgetc(fd)) == EOF) 
            break; 
          image.__ptr[i] = c; 
        } 
        fclose(fd); 
        image.__size = i; 
      } 
      image.type = "image/jpeg"; 
      image.options = soap_dime_option(&soap, 0, "My picture"); 
      if (soap_call_ns__webmethod(&soap, ...))
        ... // error
      else
        ... // success
    } 

    void *dime_read_open(struct soap *soap, void *handle, const char *id, const char *type, const char *options) 
    {
      return handle; 
    } 

    void dime_read_close(struct soap *soap, void *handle) 
    {
      fclose((FILE*)handle); 
    } 

    size_t dime_read(struct soap *soap, void *handle, char *buf, size_t len) 
    {
      return fread(buf, 1, len, (FILE*)handle); 
    }
~~~

The following example illustrates the streaming of a DIME attachment into a file by a client:

~~~{.cpp}
    int main() 
    {
      struct soap soap; 
      soap_init(&soap); 
      soap.fdimewriteopen = dime_write_open; 
      soap.fdimewriteclose = dime_write_close; 
      soap.fdimewrite = dime_write; 
      if (soap_call_ns__webmethod(&soap, ...))
        ... // error
      else
        ... // success
    } 

    void *dime_write_open(struct soap *soap, const char *id, const char *type, const char *options) 
    {
      FILE *handle = fopen("somefile", "wb"); 
      if (!handle) 
      {
        soap->error = SOAP_EOF; 
        soap->errnum = errno; // get reason 
      } 
      return (void*)handle; 
    } 

    void dime_write_close(struct soap *soap, void *handle) 
    {
      fclose((FILE*)handle); 
    } 

    int dime_write(struct soap *soap, void *handle, const char *buf, size_t len) 
    {
      size_t nwritten; 
      while (len) 
      {
        nwritten = fwrite(buf, 1, len, (FILE*)handle); 
        if (!nwritten) 
        {
          soap->errnum = errno; // get reason 
          return SOAP_EOF; 
        } 
        len -= nwritten; 
        buf += nwritten; 
      } 
      return SOAP_OK; 
    }
~~~

Message compression with `#SOAP_ENC_ZLIB` can be used with DIME to compress the entire message.
However, compression requires buffering to determine the HTTP content length
header, which cancels the benefits of streaming DIME. To avoid this, you should
use chunked HTTP (with the output-mode `#SOAP_IO_CHUNK` flag) with
compression and streaming DIME. At the server side, when you set
`#SOAP_IO_CHUNK` before calling `::soap_serve`, the engine will
automatically revert to buffering (`#SOAP_IO_STORE` flag is set).  You can
check this flag with `(soap->omode & SOAP_IO) == SOAP_IO_CHUNK` to see
if the client accepts chunking. More information about streaming chunked DIME
can be found in Section \ref dimechunking .

@warning The `options` member is a DIME-specific data structure,
consisting of a 4 byte header containing the option type info (hi byte, lo
byte), option string length (hi byte, lo byte), followed by a non-'\0'
terminated string. The DIME handler recognizes one option at most.

🔝 [Back to table of contents](#)

## Streaming chunked DIME        {#dimechunking}

To send DIME attachments, the attachment sizes
must be determined in advance to calculate HTTP message length required to
stream DIME over HTTP.  However, chunked DIME together with chunked HTTP can be used to omit this step.
First set the `#SOAP_IO_CHUNK` flag.
Then, to stream chunked DIME, set the `__size` member of an attachment to zero
and enable DIME chunking.  The DIME `::soap::fdimeread` callback then fetches data
in chunks and it is important to fill the entire buffer unless the end of the
data has been reached and the last chunk is to be send.  That is,
`::soap::fdimeread` should return the value of the last `len` parameter and
fill the entire buffer `buf` for all chunks except the last. For the last it returns 0.

You can also use the `#SOAP_IO_STORE` flag, but that cancels
the benefits of streaming DIME.

🔝 [Back to table of contents](#)

## WSDL bindings for DIME attachments  {#DIMEWSDL}

The wsdl2h tool recognizes DIME attachments and produces an
annotated header file. Both open and closed layouts are supported for
transmitting DIME attachments. For closed formats, all DIME attachments must be
referenced from the SOAP message, e.g. using hrefs with SOAP encoding and using
the application-specific reference attribute included in the `::xsd__base64Binary`
struct or class for document/literal messaging.

The soapcpp2 tool does not produce a WSDL with DIME extensions.
DIME is an older binary format that has no WSDL protocol support, unlike MIME
and MTOM.

🔝 [Back to table of contents](#)

# MTOM attachments        {#MTOM}

MTOM (Message Transmission Optimization Mechanism) is a relatively new format
for transmitting attachments with SOAP messages (see
<http://www.w3.org/TR/soap12-mtom>). MTOM attachments are essentially MIME attachments with standardized
mechanisms for cross referencing attachments from the SOAP body, which is
absent in (plain) MIME attachments and optional with DIME attachments.

Unlike the name suggests, the speed by which attached data is transmitted is
not increased compared to MIME, DIME, or even XML encoded base64 data, because 
the performance differences when using gSOAP will be small. The advantage of the
format is the standardized attachment reference mechanism, which should improve
interoperability.

The MTOM specification mandates SOAP 1.2 and the use of the XOP namespace. The
XOP Include element <i>`xop:Include`</i> is defined in the interface header file as a `::_xop__Include` struct or class, that is used to reference attachment(s) from the SOAP message body.

Because references from within the SOAP message body to attachments are
mandatory with MTOM, the implementation of the serialization and deserialization of MTOM
MIME attachments uses the extended binary type comparable to DIME support. This binary type is predefined in the <i>`import/xop.h`</i> file:

~~~{.cpp}
    //gsoap xop schema import: http://www.w3.org/2004/08/xop/include 
    struct _xop__Include 
    {
        unsigned char *__ptr; 
        int __size; 
        char *id; 
        char *type; 
        char *options; 
    }; 
    typedef struct _xop__Include _xop__Include;
~~~

The additional `id`, `type`, and `option` members
enable MTOM attachments for the data pointed to by `__ptr` of size `__size`. The process for sending and receiving MTOM XOP
attachments is fully automated.
The `id` member references the attachment, typically a content id CID or UUID which can be obtained with `::soap_rand_uuid`. When set to NULL, a unique CID is automatically used. The `type`
field specifies the required MIME type of the binary data, and the optional
`options` member can be used to piggy-back descriptive text with an attachment.  The order of the
declaration of the members is significant.

You can import <i>`xop.h`</i> in your interface header file to use the MTOM attachments, for example:

~~~{.cpp}
    #import "import/xop.h" 
    #import "import/xmime5.h" 
    #import "import/soap12.h" 
    /* alternatively, without the import above, use: 
    //gsoap SOAP-ENV schema namespace: http://www.w3.org/2003/05/soap-envelope 
    //gsoap SOAP-ENC schema namespace: http://www.w3.org/2003/05/soap-encoding 
    */ 

    //gsoap x schema namespace: http://my.first.mtom.net 
    struct x__myData 
    {
        _xop__Include xop__Include; // attachment 
      @ char *xmime5__contentType;  // and its contentType 
    }; 
    int x__myMTOMtest(struct x__myData *in, struct x__myData *out);
~~~

When an instance of `x__myDataType` is serialized and either or both the
`id` and `type` members are non-NULL, the data is transmitted as MTOM
MIME attachment if the `#SOAP_ENC_MTOM` flag is set in the gSOAP's soap
struct context:

~~~{.cpp}
    struct soap *soap = soap_new1(SOAP_ENC_MTOM);
~~~

Without this flag, the attachments will be transmitted in DIME format, which is not what we want.
If your current clients and services are based on
non-streaming DIME attachments using the SOAP body reference mechanism (thus,
without using the `::soap_set_dime_attachment` function) or plain base64
binary XML data elements, it is very easy to adopt MTOM by renaming the binary types to `xop__Include` and using the
`#SOAP_ENC_MTOM` flag with the SOAP 1.2 namespace.

See also API documentation Module \ref group_mime.

🔝 [Back to table of contents](#)

## Generating MultipartRelated MIME attachment bindings in WSDL        {#MIMEWSDL}

To generate multipartRelated bindings in the WSDL file indicating the use of MIME attachments, use:

~~~{.cpp}
    //gsoap <prefix> service method-mime-type: ...
~~~

This directive directive can be repeated for each attachment you want to
associate with a method's request and response message.  see also Section
\ref directives .

For example:

~~~{.cpp}
    #import "import/xop.h" 
    #import "import/xmime5.h" 
    #import "import/soap12.h" 

    //gsoap x schema namespace: http://my.first.mtom.net 
    struct x__myData 
    {
        _xop__Include xop__Include; // attachment 
      @ char *xmime5__contentType;  // and its contentType 
    }; 
    //gsoap x service method-mime-type: myMTOMtest text/xml 
    int x__myMTOMtest(struct x__myData *in, struct x__myData *out);
~~~

The `//gsoap x service method-mime-type:` directive indicates that this
operation accepts <i>`text/xml`</i> MIME attachments. See the SOAP-with-Attachment
specification for the MIME types to use (for example, `*/*` is a wildcard).
If the operation has more than one attachment, just repeat this directive for
each attachment you want to bind to the operation.

To bind attachments only to the request message of an operation, use

~~~{.cpp}
    //gsoap <prefix> service method-input-mime-type: ...
~~~

Similarly, to bind attachments
only to the response message of an operation, use:

~~~{.cpp}
    //gsoap <prefix> service method-output-mime-type: ...
~~~

The wsdl2h tool recognizes MIME attachments and produces an
annotated header file.

You can repeat these directives for all multipartRelated MIME attachments you
want to associate with the service operation input and output.

🔝 [Back to table of contents](#)

## Sending and receiving MTOM attachments   {#MTOMsendingreceiving}

A receiver must be informed to recognize MTOM attachments by setting the
`#SOAP_ENC_MTOM` flag of the `::soap` context. Otherwise, the regular MIME
attachment mechanism (SwA) will be used to store attachments.

When using wsdl2h to build clients and/or services, you should use the
<i>`typemap.dat`</i> file included in the gSOAP source code package. The
<i>`typemap.dat`</i> file defines the XOP namespace and XML MIME namespaces as
imported namespaces:

    xop    = <http://www.w3.org/2004/08/xop/include> 
    xmime5 = <http://www.w3.org/2005/05/xmlmime> 
    xmime4 = <http://www.w3.org/2004/11/xmlmime>

The wsdl2h tool uses the <i>`typemap.dat`</i> file to
convert WSDL into an interface header file. In this case we don't want the
wsdl2h tool to read the XOP schema and translate it, since we have a
pre-defined `::_xop__Include` element to handle XOP for MTOM. This
`::_xop__Include` element is defined in <i>`xop.h`</i>. Therefore, the
bindings shown above will not translate the XOP and XML MIME schemas to code,
but generates `#import` statements instead in the generated interface header file:

~~~{.cpp}
    #import "xop.h" 
    #import "xmime5.h"
~~~

The `#import` statements are only added for those namespaces that are
actually used.

Let's take a look at an example.
The wsdl2h importer generates a header file with `#import "xop.h"` from a WSDL that references XOP, for example:

~~~{.cpp}
    #import "xop.h" 
    #import "xmime5.h" 
    struct ns__Data 
    {
        _xop__Include xop__Include; 
      @ char *xmime5__contentType;  
    };
~~~

Suppose the WSDL defines an operation:

~~~{.cpp}
    int ns__echoData(struct ns__Data *in, struct ns__Data *out);
~~~

After generating the stub functions with the soapcpp2 tool, we can invoke the stub at the client side with:

~~~{.cpp}
    struct soap *soap = soap_new1(SOAP_ENC_MTOM); 
    struct ns__Data data; 
    data.xop__Include.__ptr = (unsigned char*)"<b>Hello world!</b>"; 
    data.xop__Include.__size = 20; 
    data.xop__Include.id = NULL;            // CID automatically generated by engine 
    data.xop__Include.type = "text/html";   // MIME type 
    data.xop__Include.options = NULL;       // no descriptive info added 
    data.xmime5__contentType = "text/html"; // MIME type 
    if (soap_call_ns__echoData(soap, endpoint, action, &data, &data))
      soap_print_fault(soap, stderr);
    else 
      printf("Got data\n"); 
    soap_destroy(soap);
    soap_end(soap);
    soap_free(soap);
~~~

Note that the `xop__Include.type` member must be set to transmit MTOM attachments, otherwise inline base64 XML will be sent.

At the server side, we show an example of an operation handler that just copies the input data to output:

~~~{.cpp}
    int ns__echoData(struct soap *soap, struct ns__Data *in, struct ns__data *out) 
    {
      *out = *in; 
      return SOAP_OK; 
    }
~~~

The server must use the `#SOAP_ENC_MTOM` flag to initialize the `::soap` context to receive and send MTOM attachments.

🔝 [Back to table of contents](#)

## Streaming MIME/MTOM        {#MTOMstreaming}

Streaming MIME/MTOM is achieved with callback functions to fetch and store data
during transmission. Three function callbacks for streaming MIME/MTOM output and
three callbacks for streaming MIME/MTOM input are available.

* `void *(*soap.fmimereadopen)(struct soap *soap, void *handle, const char *id, const char *type, const char *description)` 
  This callback is called by the engine to start sending a streaming MIME/MTOM attachment.  This callback opens a stream to start reading the attachment data to send.  The actual data stream will be read in chunks using the `::soap::fmimeread` callback until no more data is available and the `::soap::fmimereadclose` callback is called to close the stream.  The `handle` parameter contains the value of the `__ptr` member variable of the attachment struct/class with data (e.g. `::xsd__base64Binary` or `::_xop__Include` with `__ptr`, `__size`, `id`, `type` and `options` members), which should be a pointer to specific information such as a file descriptor or a pointer to a some application-specific data to be passed to this callback.  Both the `__ptr` and `__size` members of the attachment struct/class should have been set by the application prior to the serialization of the message with attachments.  If the `__size` is zero and HTTP chunking is enabled (with `#SOAP_IO_CHUNK`), then chunked MIME/MTOM attachments are sent, see `::soap::fmimeread`.  The `id`, `type` and `options` parameters are the `id` (an optional ID), `type` (a MIME type) and `options` (a descriptive string) of the attachment struct/class, respectively, of which at least one member should be non-NULL.  The callback should return the `handle` parameter value or another pointer value, which is passed as the new `handle` parameter to `::soap::fmimeread` and `::soap::fmimereadclose` callbacks.  When an error occurred in this callback, the callback should return NULL and set `::soap::error` to an error code, e.g. using `::soap_receiver_fault`.  The callback may return NULL and set `::soap::error` to `#SOAP_OK` when this specific MIME/MTOM attachment should not to be streamed and the engine will simply skip it.

* `size_t (*soap.fmimeread)(struct soap *soap, void *handle, char *buf, size_t len)` 
  This callback is called by the engine to read a chunk of attachment data to transmit.  The `handle` parameter contains the handle returned by the `::soap::fmimereadopen` callback.  The `buf` parameter is the buffer of length `len` into which a chunk of data should be written by the callback.  The actual amount of data written into the buffer may be less than `len` and this actual amount should be returned by the callback.  A return value of zero indicates an error and `::soap::error` should be set.  The `__size` member variable of the attachment struct/class with data (e.g. `::xsd__base64Binary` or `::_xop__Include` with `__ptr`, `__size`, `id`, `type` and `options` members) should be set by the application prior to the serialization of the message with attachments.  The value of `__size` indicates the total size of the attachment data to be transmitted.  If the `__size` member variable is zero and HTTP chunking is enabled (with `#SOAP_IO_CHUNK`), then MIME/MTOM chunked transfers are activated by the engine, which is more flexible since the attachment data size does not need to be determined in advance.  To use MIME/MTOM chunked transfers, enable HTTP chunking with `#SOAP_IO_CHUNK` (also `#SOAP_IO_STORE` can be used, but this buffers the entire message in memory before transmission) and set the `__size` member variable of the attachment struct/class to zero.  When MIME/MTOM attachment chunking is enabled, this callback should completely fill the `buf` buffer with `len` bytes unless the last data chunk is reached and fewer bytes are returned.

* `void (*soap.fmimereadclose)(struct soap *soap, void *handle)` 
  This callback is called by the engine to close the MIME/MTOM attachment stream after reading.  The `handle` parameter contains the handle returned by the `::soap::fmimereadopen` callback.

* `void *(*soap.fmimewriteopen)(struct soap *soap, void *handle, const char *id, const char *type, const char *description, enum soap_mime_encoding encoding)` 
  Called by the to start receiving a streaming MIME/MTOM attachment.  This callback opens a stream to start writing the attachment data received.  The actual data stream will be written in chunks using the `::soap::fmimewrite` callback until no more data is available and the `::soap::fmimewriteclose` callback is called to close the stream.  The `id`, `type` and `options` parameters are the `id`, `type` and `options` of the attachment struct/class (e.g. `::xsd__base64Binary` or `::_xop__Include` with `__ptr`, `__size`, `id`, `type` and `options` members), respectively.  The callback should return a handle, which is passed to the `::soap::fmimewrite` and `::soap::fmimewriteclose` callbacks.  The `__ptr` member variable of the attachment struct/class is set by the engine to the value of this handle.  The `__size` member variable is set to the size of the attachment received.

* `int (*soap.fmimewrite)(struct soap *soap, void *handle, const char *buf, size_t len)` 
  This callback is called by the engine to write a chunk of attachment data received.  The `handle` parameter contains the handle returned by the `::soap::fmimewriteopen` callback.  The `buf` parameter contains the data of length `len`.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.

* `void (*soap.fmimewriteclose)(struct soap *soap, void *handle)` 
  This callback is called by the engine to close the MIME/MTOM attachment stream after writing.  The `handle` parameter contains the handle returned by the `::soap::fmimewriteopen` callback.

In addition, a `void* ::soap::user` member
is available to pass user-defined data to the callbacks.  This way, you can set
`void* ::soap::user` to point to application data that the callbacks need such as a
file name for example.

The following example illustrates the client-side initialization of an image
attachment struct to stream a file into a MTOM attachment without HTTP chunking (HTTP streaming chunked MTOM transfer is presented in Section \ref mimechunking ):

~~~{.cpp}
    int main() 
    {
      struct soap soap; 
      struct xsd__base64Binary image; 
      FILE *fd; 
      struct stat sb; 
      soap_init1(&soap, SOAP_ENC_MTOM); // mandatory to enable MTOM 
      if (!fstat(fileno(fd), &sb) && sb.st_size > 0) 
      {
        // because we can get the length of the file, we can stream it without chunking 
        soap.fmimereadopen = mime_read_open; 
        soap.fmimereadclose = mime_read_close; 
        soap.fmimeread = mime_read; 
        image.__ptr = (unsigned char*)fd; // must set to non-NULL (this is our fd handle which we need in the callbacks) 
        image.__size = sb.st_size; // must set size 
      } 
      else 
      {
        // don't know the size, so buffer it 
        size_t i; 
        int c; 
        image.__ptr = (unsigned char*)soap_malloc(&soap, MAX_FILE_SIZE); 
        for (i = 0; i < MAX_FILE_SIZE; i++) 
        {
          if ((c = fgetc(fd)) == EOF) 
            break; 
          image.__ptr[i] = c; 
        } 
        fclose(fd); 
        image.__size = i; 
      } 
      image.type = "image/jpeg"; // MIME type 
      image.options = "This is my picture"; // description of object 
      if (soap_call_ns__webmethod(&soap, ...))
        ... // error
      else
        ... // success
    } 

    void *mime_read_open(struct soap *soap, void *handle, const char *id, const char *type, const char *description) 
    {
      return handle; 
    } 

    void mime_read_close(struct soap *soap, void *handle) 
    {
      fclose((FILE*)handle); 
    } 

    size_t mime_read(struct soap *soap, void *handle, char *buf, size_t len) 
    {
      return fread(buf, 1, len, (FILE*)handle); 
    }
~~~

The following example illustrates the streaming of a MIME/MTOM attachment into a file by a client:

~~~{.cpp}
    int main() 
    {
      struct soap soap; 
      soap_init(&soap); 
      soap.fmimewriteopen = mime_write_open; 
      soap.fmimewriteclose = mime_write_close; 
      soap.fmimewrite = mime_write; 
      if (soap_call_ns__webmethod(&soap, ...))
        ... // error
      else
        ... // success
    } 

    void *mime_write_open(struct soap *soap, const char *id, const char *type, const char *description, enum soap_mime_encoding encoding) 
    {
      FILE *handle = fopen("somefile", "wb"); 
      // We ignore the MIME content transfer encoding here, but should check 
      if (!handle) 
      {
        soap->error = SOAP_EOF; 
        soap->errnum = errno; // get reason 
      } 
      return (void*)handle; 
    } 

    void mime_write_close(struct soap *soap, void *handle) 
    {
      fclose((FILE*)handle); 
    } 

    int mime_write(struct soap *soap, void *handle, const char *buf, size_t len) 
    {
      size_t nwritten; 
      while (len) 
      {
        nwritten = fwrite(buf, 1, len, (FILE*)handle); 
        if (!nwritten) 
        {
          soap->errnum = errno; // get reason 
          return SOAP_EOF; 
        } 
        len -= nwritten; 
        buf += nwritten; 
      } 
      return SOAP_OK; 
    }
~~~

Message compression with `#SOAP_ENC_ZLIB` can be used with MIME to compress the entire message.
However, compression requires buffering to determine the HTTP content length
header, which cancels the benefits of streaming MIME. To avoid this, you should
use chunked HTTP (with the output-mode `#SOAP_IO_CHUNK` flag) with
compression and streaming MIME. At the server side, when you set
`#SOAP_IO_CHUNK` before calling `::soap_serve`, the engine will
automatically revert to buffering (`#SOAP_IO_STORE` flag is set).  You can
check this flag with `(soap->omode & SOAP_IO) == SOAP_IO_CHUNK` to see
if the client accepts chunking. More information about streaming chunked MIME
can be found in Section \ref mimechunking ..

Note that the example above for `mime_read` uses a handle that points to the open file
`FILE*`.
The simple example above is not recommended when the
platform imposes a limit on the number of open file descriptors.
You can use the handle to pass along more information than just
the file descriptor. So for example, when the number of open file descriptors
is limited on your platform, you should let the handle point to a structure
with file-related information. The C++ example below illustrates this:

~~~{.cpp}
    file.xop__Include = soap_new__xop__Include(soap); 
    file.xop__Include->id = NULL; 
    file.xop__Include->type = type; 
    file.xop__Include->options = NULL; 

    file.xmime5__contentType = type; 
    file.filename = filename; 

    // The object holding all information to read data 
    FileStreamIn *ins = new FileStreamIn(errorhandler); 
    ins->setFilePath(path); 
    ins->setFileName(filename); 

    file.xop__Include->__size = size; 
    file.xop__Include->__ptr = (unsigned char*)ins;
~~~

To read the MTOM data for transmission:

~~~{.cpp}
    void *mime_read_open(struct soap *soap, void *handle, const char *id, const char *type, const char *description) 
    {
      if (!handle) 
        return NULL; 
      FileStreamIn *ins = (FileStreamIn*)handle; 
      if (!ins->open()) 
      {
        soap->error = SOAP_ERR; 
        return NULL; 
      } 
      return handle; 
    } 
    void mime_read_close(struct soap *soap, void *handle) 
    {
      if (!handle) 
        return; 
      FileStreamIn *ins = (FileStreamIn*)handle; 
      delete ins; 
    } 
    size_t mime_read(struct soap *soap, void *handle, char *buf, size_t len) 
    {
      if (!handle) 
        return 0; 
      FileStreamIn *ins = (FileStreamIn*)handle; 
      size_t nread = ins->read(buf, len); 
      if (ins->streamError()) 
      {
        soap->error = ins->streamError(); 
        return 0; 
      } 
      return nread; 
    }
~~~

🔝 [Back to table of contents](#)

## Redirecting inbound MIME/MTOM streams based on SOAP body content        {#MTOMpoststreaming}

When it is preferable or required to redirect inbound MIME/MTOM attachment
streams based on SOAP message body content, where for example the names of the
resources are listed in the SOAP message body, an alternative mechanism must be
used to handle the attachments. This mechanism can be used at the client and server side.

Because the routing of the streams is accomplished with explicit function
calls, this method should only be used when required and should not be
considered optional. That is, when you enable this method, you must check for
pending MIME/MTOM attachments and handle them appropriately. This is true even
when you don't expect MIME/MTOM attachments in the payload, because the peer
may trick you by sending attachments anyway and you should be prepared to
accept or reject them.

The explicit MIME/MTOM streaming mechanism consists of three API functions:

* `void soap_post_check_mime_attachments(struct soap *soap)` 
  This function enables post-processing of MIME/MTOM attachments received.  This means that the presence of MIME/MTOM attachments must be explicitly checked by calling `::soap_check_mime_attachments` after the message was received.  When this function returns nonzero (true), then the attachments can be retrieved by calling `::soap_recv_mime_attachment` repeatedly to retrieve each attachment until this function returns NULL. This function returns a pointer to a `struct soap_multipart` attachment.

* `int soap_check_mime_attachments(struct soap *soap)`
  This function checks the presence of a MIME/MTOM attachment after calling a service operation by returning nonzero when attachments are present.  Returns nonzero if attachments are present.  Requires `::soap_post_check_mime_attachments`.

* `struct soap_multipart *soap_recv_mime_attachment(struct soap *soap, void *handle)` 
This function parses an attachment and invokes the MIME callbacks when set.  The `handle` parameter is passed to `fmimewriteopen`.  The handle may contain any data that is extracted from the SOAP message body to guide the redirection of the stream in the callbacks.  Returns a struct with a `char *ptr` member that contains the handle value returned by the `fmimewriteopen` callback, and `char *id`, `char *type`, and `char *description` member variables with the MIME id, type, and description info when present in the attachment.

Example client in C:

~~~{.cpp}
    struct soap *soap = soap_new1(SOAP_ENC_MTOM); 
    soap_post_check_mime_attachments(soap); 
    if (soap_call_ns__myMethod(soap, ...)) 
    {
      soap_print_fault(soap, stderr); // an error occurred 
    }
    else 
    {
      if (soap_check_mime_attachments(soap))
      {
        // attachments are present, channel is still open 
        do
        {
          ... // get data 'handle' from SOAP response and pass to callbacks 
          ... // set the fmime callbacks, if needed 
          struct soap_multipart *content = soap_recv_mime_attachment(soap, (void*)handle); 
          printf("Received attachment with id=%s and type=%s\n", content->id?content->id:"", content->type?content->type:""); 
        } while (content); 
        if (soap->error) 
          soap_print_fault(soap, stderr); 
      }
    } 
    soap_destroy(soap); 
    soap_end(soap); 
    soap_free(soap);
~~~

The server-side service operations are implemented as usual, but with additional checks for MIME/MTOM attachments:

~~~{.cpp}
    int main()
    {
      struct soap *soap = soap_new1(SOAP_ENC_MTOM); 
      soap_post_check_mime_attachments(soap); 
      soap_serve(soap); 
    }

    int ns__myMethod(struct soap *soap, ...) 
    {
      ... // server-side processing logic 
      if (soap_check_mime_attachments(soap))
      {
        // attachments are present, channel is still open 
        do 
        {
          ... // get data 'handle' from SOAP request and pass to callbacks 
          ... // set the fmime callbacks, if needed 
          struct soap_multipart *content = soap_recv_mime_attachment(soap, (void*)handle); 
          printf("Received attachment with id=%s and type=%s\n", content->id?content->id:"", content->type?content->type:""); 
        } while (content); 
        if (soap->error) 
          return soap->error; 
      } 
      ... // server-side processing logic 
      return SOAP_OK; 
    }
~~~

🔝 [Back to table of contents](#)

## Streaming chunked MIME/MTOM        {#mimechunking}

To send MIME/MTOM attachments, the attachment sizes
must be determined in advance to calculate HTTP message length required to
stream MIME/MTOM over HTTP.  However, chunked MIME/MTOM together with chunked HTTP can be used to omit this step.
First set the `#SOAP_IO_CHUNK` flag.
Then, to stream chunked MIME/MTOM, set the `__size` member of an attachment to zero
and enable MIME/MTOM chunking.  The MIME/MTOM `::soap::fmimeread` callback then fetches data
in chunks and it is important to fill the entire buffer unless the end of the
data has been reached and the last chunk is to be send.  That is,
`::soap::fmimeread` should return the value of the last `len` parameter and
fill the entire buffer `buf` for all chunks except the last.  For the last it returns 0.

🔝 [Back to table of contents](#)

# SOAP/XML over UDP        {#UDP}

UDP is a simple, unreliable datagram protocol: UDP sockets are connectionless.
UDP address formats are identical to those used by TCP.  In particular UDP
provides a port identifier in addition to the normal Internet address format.
The UDP port space is separate from the TCP port space (i.e. a UDP port may not
be "connected" to a TCP port).  In addition broadcast packets may be sent
(assuming the underlying network supports this) by using a reserved "broadcast
address"; this address is network interface dependent.

Client-side messages with SOAP-over-UDP endpoint URLs
(`soap.udp://...`) are automatically transmitted as datagrams.
Server-side applications should set the `#SOAP_IO_UDP` mode flag to
accept UDP requests, e.g. using `::soap_new1`, `::soap_init1`, or
`::soap_set_mode`.

The maximum message length for datagram packets is restricted by the buffer
size `#SOAP_BUFLEN`, which is 65536 by default, unless
compiled with `#WITH_LEAN` to support small-scale embedded systems.
For UDP transport `#SOAP_BUFLEN` must not exceed the maximum UDP packet size
65536 (the size of datagram messages is constrained by the
UDP packet size 2^16=65536 as per UDP standard). You can use compression
with `#SOAP_ENC_ZLIB` to reduce the message size, but note that compressed SOAP-over-UDP is a
gSOAP-specific feature because it is not part of the SOAP-over-UDP
specification.

The SOAP-over-UDP specification relies on WS-Addressing. The <i>`wsa.h`</i>
file in the `import` directory defines the WS-Addressing elements for
client and server applications.

The gSOAP implementation conforms to the SOAP-over-UDP requirements:


*  SOAP-over-UDP server endpoint URL format: <i>`soap.udp://host:port/path`</i>

*  Support one-way message-exchange pattern (MEP) where a SOAP envelope is
  carried in a user datagram.

*  Support request-response message-exchange pattern (MEP) where SOAP envelopes
  are carried in user datagrams.

*  Support multicast transmission of SOAP envelopes carried in user datagrams.

*  Support both SOAP 1.1 and SOAP 1.2 envelopes.

The following additional features are also available, but are not supported by the SOAP-over-UDP specification:

*  Zlib/gzip message compression (use compile-time flag `#WITH_GZIP`).

*  SOAP with DIME attachments over UDP.

*  SOAP with MIME/MTOM attachments over UDP.

*  Support for IPv6 (use compile-time flag `#WITH_IPV6`)


🔝 [Back to table of contents](#)

## Using WS-Addressing with SOAP-over-UDP        {#wsaudp}

A SOAP-over-UDP application may use WS-Addressing to control message delivery
as per SOAP-over-UDP specification.

The <i>`wsa.h`</i> file in the `import` directory defines the
WS-Addressing elements.  To include the WS-Addressing elements in the SOAP
Header for messaging, a WS-Addressing capable `::SOAP_ENV__Header` struct should be
defined in your header file by importing <i>`gsoap/import/wsa.h`</i>
or <i>`gsoap/import/wsa5.h`</i>:

~~~{.cpp}
    #import "wsa.h" 
~~~

We also included a `//gsoap wsa schema import:` directive in the <i>`wsa.h`</i> file to enable the generation of WSDL
specifications that import (instead of includes) the WS-Addressing elements.
Note that the `//gsoapopt w` directive (which adds option <b>`-w`</b> to run <b>`soapcpp2 -w`</b>) must not be present in your header file to enable WSDL generation.

One-way SOAP-over-UDP messages (see Section \ref oneway1 ) should be
declared to include the <i>`wsa:MessageID`</i>, <i>`wsa:To`</i>, and <i>`wsa:Action`</i>
elements in the SOAP Header of the request message as follows:

~~~{.cpp}
    //gsoap ns service method-header-part:          sendString wsa__MessageID 
    //gsoap ns service method-header-part:          sendString wsa__To 
    //gsoap ns service method-header-part:          sendString wsa__Action 
    int ns__sendString(char *str, void);
~~~

Request-response SOAP-over-UDP messages should be declared to include the
<i>`wsa:MessageID`</i>, <i>`wsa:To`</i>, <i>`wsa:Action`</i>, and <i>`wsa:ReplyTo`</i>
elements in the SOAP Header of the request message, and the the
<i>`wsa:MessageID`</i>, <i>`wsa:To`</i>, <i>`wsa:Action`</i>, and <i>`wsa:RelatesTo`</i>
elements in the SOAP Header of the response message:

~~~{.cpp}
    //gsoap ns service method-header-part:          echoString wsa__MessageID 
    //gsoap ns service method-header-part:          echoString wsa__To 
    //gsoap ns service method-header-part:          echoString wsa__Action 
    //gsoap ns service method-input-header-part:    echoString wsa__ReplyTo 
    //gsoap ns service method-output-header-part:   echoString wsa__RelatesTo 
    int ns__echoString(char *str, char **res);
~~~

For the content requirements of these elements, please consult the
SOAP-over-UDP specification and/or read the next sections explaining
SOAP-over-UDP unicast, multicast, one-way, and request-response client and
server applications.

🔝 [Back to table of contents](#)

### Client-side one-way UDP unicast   {#unicast}

This example assumes that the interface header file includes the SOAP Header with
WS-Addressing elements, see \ref wsaplugin, and the `ns__sendString` function discussed in
Section \ref wsaudp. 

~~~{.cpp}
    #include "plugin/wsaapi.h"

    struct soap soap; 
    soap_init(&soap); 
    soap_register_plugin(&soap, soap_wsa);
    soap.send_timeout = 5; // 5 seconds max socket delay 
    // set up WS-Addressing header
    soap_wsa_request(&soap, "message ID", "endpoint", "SOAP action");
    // Send the message over UDP: 
    if (soap_send_ns__echoString(&soap, "soap.udp://endpoint", "SOAP action", "hello world!")) 
      soap_print_fault(&soap, stderr); // error 
    soap_destroy(&soap);
    soap_end(&soap);
    soap_done(&soap);
~~~

🔝 [Back to table of contents](#)

### Client-side one-way UDP multicast   {#multicast}

This example is similar to the one-way unicast example discussed above, but
uses a broadcast address and the `SO_BROADCAST` socket option:

~~~{.cpp}
    #include "plugin/wsaapi.h"

    struct soap soap; 
    in_addr_t addr = inet_addr("1.2.3.4"); // optional 
    soap_init(&soap); 
    soap_register_plugin(&soap, soap_wsa);
    soap.send_timeout = 5; // 5 seconds max socket delay
    soap.connect_flags = SO_BROADCAST; // required for broadcast 
    soap.ipv4_multicast_if = &addr; // optional for IPv4: see setsockopt IPPROTO_IP IP_MULTICAST_IF 
    soap.ipv6_multicast_if = addr; // optional for IPv6: multicast sin6_scope_id 
    soap.ipv4_multicast_ttl = 1; // optional, see setsockopt IPPROTO_IP, IP_MULTICAST_TTL 
    // set up WS-Addressing header
    soap_wsa_request(&soap, "message ID", "endpoint", "SOAP action");
    // Send the message over UDP to a broadcast address: 
    if (soap_send_ns__echoString(&soap, "soap.udp://endpoint", "SOAP action", "hello world!")) 
      soap_print_fault(&soap, stderr); // report error 
    soap_destroy(&soap);
    soap_end(&soap);
    soap_done(&soap);
~~~

Please refer to the socket options for `IPPROTO_IP` `IP_MULTICAST_IF`
to specify
the default interface for multicast datagrams to be sent from. This
is a `struct in_addr` (`in_addr_t` for `sin6_scope_id`)
interface value. Otherwise, the default interface set by the system
administrator will be used (if any).

Please refer to the socket options for `IPPROTO_IP` `IP_MULTICAST_TTL` to limit
the lifetime of the packet. Multicast datagrams are sent with a default value
of 1, to prevent them to be forwarded beyond the local network. This parameter
can be set between 1 to 255.

🔝 [Back to table of contents](#)

### Client-side request-response UDP unicast   {#requestresponseunicast}

This example assumes that the interface header file for soapcpp2 includes the SOAP Header with
WS-Addressing elements imported with `#import "wsa.h"` and the `ns__echoString` function discussed in
Section \ref wsaudp .

~~~{.cpp}
    #include "plugin/wsaapi.h"

    struct soap soap; 
    struct wsa__EndpointReferenceType replyTo; // (anonymous) reply address 
    char *res; // server response 
    soap_init(&soap); 
    soap_register_plugin(&soap, soap_wsa);
    soap.send_timeout = 5; // 5 seconds max socket delay 
    soap.recv_timeout = 5; // 5 seconds max socket delay 
    // set up WS-Addressing header
    soap_wsa_request(&soap, "message ID", "endpoint", "SOAP action");
    soap_wsa_add_ReplyTo(&soap, NULL); // anonymous ReplyTo address
    // Send and receive messages over UDP: 
    if (soap_call_ns__echoString(&soap, "soap.udp://endpoint "SOAP action", "hello world!", &res)) 
    {
      if (soap.error == SOAP_EOF && soap.errnum == 0) 
        ... // Timeout: no response from server (message already delivered?) 
      else 
        soap_print_fault(&soap, stderr); 
    } 
    else 
      ... // UDP server response is stored in 'res' 
    // check SOAP header received, if applicable 
    check_header(&soap.header); 
    soap_destroy(&soap);
    soap_end(&soap);
    soap_done(&soap);
~~~

🔝 [Back to table of contents](#)

### Client-side request-response multicast  {#requestresponsemulticast}

This example is similar to the request-response unicast example discussed
above, but uses a broadcast address and the `SO_BROADCAST` socket option.
Because we expect to receive multiple responses, we also need to use separate
request-response messages to send one request and consume multiple responses.
In this example we defined a `bcastString` request and a
`bcastStringResponse` response message, which are essentially declared as
one-way messages in the header file:

~~~{.cpp}
    //gsoap ns service method-header-part:          bcastString wsa__MessageID 
    //gsoap ns service method-header-part:          bcastString wsa__To 
    //gsoap ns service method-header-part:          bcastString wsa__Action 
    //gsoap ns service method-header-part:          bcastString wsa__ReplyTo 
    int ns__bcastString(char *str, void); 
    //gsoap ns service method-header-part:          bcastStringResponse wsa__MessageID 
    //gsoap ns service method-header-part:          bcastStringResponse wsa__To 
    //gsoap ns service method-header-part:          bcastStringResponse wsa__Action 
    //gsoap ns service method-header-part:          bcastStringResponse wsa__RelatesTo 
    int ns__bcastStringResponse(char *res, void);
~~~

To obtain response one-way operations, use [<b>`wsdl2h -b`</b> option <b>`-b`</b>](#wsdl2h-b).

The client code includes a loop to receive response messages until a timeout occurs:

~~~{.cpp}
    #include "plugin/wsaapi.h"

    struct soap soap; 
    struct SOAP_ENV__Header header; 
    struct wsa__EndpointReferenceType replyTo; 
    char *res; 
    soap_init(&soap); 
    soap_register_plugin(&soap, soap_wsa);
    soap.connect_flags = SO_BROADCAST; 
    soap.send_timeout = 5; // 5 seconds max socket delay 
    soap.recv_timeout = 5; // 5 seconds max socket delay 
    // set up WS-Addressing header
    soap_wsa_request(&soap, "message ID", "endpoint", "SOAP action");
    soap_wsa_add_ReplyTo(&soap, NULL); // anonymous ReplyTo address
    if (soap_send_ns__bcastString(&soap, "soap.udp://endpoint", "SOAP action", "hello world!")) 
    {
      soap_print_fault(&soap, stderr); 
    }
    else 
    {
      while (1)
      {
        if (soap_recv_ns__bcastStringResponse(&soap, &res)) 
          break; 
        ... // Got response 'res' from a server 
      } 
      if (soap.error == SOAP_EOF && soap.errnum == 0) 
        ... // Timeout: no more messages received 
      else 
        soap_print_fault(&soap, stderr); 
    } 
    soap_destroy(&soap);
    soap_end(&soap);
    soap_done(&soap);
~~~

Note that a server for the `bcastString` does not need to use two-one way
messages.  Thus, multicast request-response message pattern can be declared and
implemented as request-response operations at the server side.

🔝 [Back to table of contents](#)

## SOAP-over-UDP server        {#soapoverudp}

The following example code illustrates a SOAP-over-UDP server for one-way
`sendString` and request-response `echoString` messages.  This example assumes
that the interface header file includes the SOAP Header with WS-Addressing
elements imported with `#import "wsa.h"` and the `ns__echoString` function
discussed in Section \ref wsaudp .

~~~{.cpp}
    #include "plugin/wsaapi.h"

    int main() 
    {
      struct soap soap; 
      soap_init1(&soap, SOAP_IO_UDP); // must set UDP flag 
      soap_register_plugin(&soap, soap_wsa);
      // bind to host (NULL = current host) and port: 
      if (!soap_valid_socket(soap_bind(&soap, host, port, BACKLOG))) 
      {
        soap_print_fault(&soap, stderr); 
        exit(EXIT_FAILURE); 
      } 
      while (1)
      {
        if (soap_serve(&soap)) 
          soap_print_fault(&soap, stderr); // report the problem 
        soap_destroy(&soap); 
        soap_end(&soap); 
      } 
      soap_done(&soap); // close connection 
    } 

    int ns__echoString(struct soap *soap, char *str, char **res) 
    {
      // check if WS-Addressing headers are present and correct
      if (soap_wsa_check(soap))
        return soap->error;
      // should check for duplicate messages (something that WS-ReliableMessaging does too)
      if (check_received(soap->header->wsa__MessageID)) 
      {
        // Request message already received 
        return SOAP_STOP; // don't return response 
      } 
      *res = str; 
      // return OK with WS-Addressing reply headers, message ID is id_count+1
      return soap_wsa_reply(soap, soap_int2s(soap, id_count++), "http://genivia.com/udp/echoStringResponse");
    } 

    int ns__sendString(struct soap *soap, char *str) 
    {
      if (soap_wsa_check(soap))
        return soap->error;
      // should check for duplicate messages 
      if (check_received(soap->header->wsa__MessageID)) 
      {
        // Request message already received 
        return SOAP_STOP;
      } 
      return SOAP_OK;
    } 

    int ns__sendStringResponse(struct soap *soap, char *res) 
    {
      return SOAP_NO_METHOD; // we don't serve this operation
    }
~~~

The server binds to a host and port and accepts messages in a 
loop.  Because UDP does not have the equivalent of an accept, the messages
cannot be dispatched to threads. Instead the `::soap_serve` waits for a message
and immediately accepts it. You can use a receive timeout value for
`::soap::recv_timeout` to make `::soap_serve` non-blocking.

To obtain response one-way operations from a WSDL, use [<b>`wsdl2h -b`</b> option <b>`-b`</b>](#wsdl2h-b).
This produces additional one-way operations to support
asynchronous handling of response messages in the same way requests are
handled.

🔝 [Back to table of contents](#)

### SOAP-over-UDP multicast receiving server   {#soapoverudpmulticast}

For UDP multicast support, follow the suggestions in
Section \ref soapoverudp  and change the initialization parts of the code
to enable UDP multicast port binding by to telling the kernel which multicast
groups you are interested in:

~~~{.cpp}
    #include "plugin/wsaapi.h"

    int main() 
    {
      struct soap soap; 
      struct ip_mreq mcast; 
      soap_init1(&soap, SOAP_IO_UDP); 
      soap_register_plugin(&soap, soap_wsa);
      if (!soap_valid_socket(soap_bind(&soap, host, port, BACKLOG))) 
      {
        soap_print_fault(&soap, stderr); 
        exit(EXIT_FAILURE); 
      } 
      mcast.imr_multiaddr.s_addr = inet_addr(put IP multicast address of group here); 
      mcast.imr_interface.s_addr = htonl(INADDR_ANY); 
      if (setsockopt(soap.master, IPPROTO_IP, IP_ADD_MEMBERSHIP, &mcast, sizeof(mcast))<0) 
        ... // error
      ... //
      soap_destroy(&soap);
      soap_end(&soap);
      soap_done(&soap);
    }
~~~

🔝 [Back to table of contents](#)

# Compile-time flags        {#compilerflags}

The following macros are defined in the API documentation Module \ref group_with.  These macros are used to enable or disable features as specified below, by compiling source code files with compiler option <b>`-D`</b> to set the macro:

define                        | result
----------------------------- | ------
`#SOAPDEFS_H`                 | the header file to include, if different from `soapdefs.h`
`#WITH_SOAPDEFS_H`            | includes the `soapdefs.h` file for custom settings, see Section \ref soapdefs  
`#WITH_COMPAT`                | removes dependency on C++ stream libraries and C++ exceptions 
`#WITH_LEAN`                  | creates a small-footprint executable, see Section \ref lean  
`#WITH_LEANER`                | creates an even smaller footprint executable, see Section \ref lean  
`#WITH_FAST`                  | use faster memory allocation when used with `#WITH_LEAN` or `#WITH_LEANER` 
`#WITH_COOKIES`               | enables HTTP cookies, see Sections \ref clientcookie and \ref servercookie  
`#WITH_INSECURE_COOKIES`      | enables HTTP cookies and allows cookies with their Secure flag set to be sent over insecure channels
`#WITH_IPV6`                  | enables IPv6 support
`#WITH_IPV6_V6ONLY`           | enables IPv6 support with IPv6-only server option
`#WITH_OPENSSL`               | enables OpenSSL, see Sections \ref clientopenssl and \ref serveropenssl  
`#WITH_GNUTLS`                | enables GNUTLS, see Sections \ref clientopenssl and \ref serveropenssl  
`#WITH_GZIP`                  | enables gzip and deflate compression, see Section \ref compression  
`#WITH_ZLIB`                  | enables deflate compression only, see Section \ref compression  
`#WITH_NTLM`                  | enables NTLM support
`#WITH_C_LOCALE`              | force the use locale functions when available to ensure locale-independent number conversions
`#WITH_NO_C_LOCALE`           | remove the use of locale functions to improve portability
`#WITH_INCLUDE_XLOCALE_H`     | force the inclusion of `<xlocale.h>` to define `locale_t` and `_l` functions, to improve portability
`#WITH_DOM`                   | enable DOM parsing in the engine, required by the WS-Security plugin
`#WITH_REPLACE_ILLEGAL_UTF8`  | enable strict UTF-8, replaces UTF-8 content that is outside the allowed range with U+FFFD 
`#WITH_FASTCGI`               | enables FastCGI, see Section \ref fastcgi  
`#WITH_NOIO`                  | removes IO operations, to eliminate the use of BSD sockets, see Section \ref noio  
`#WITH_NOIDREF`               | removes id and href/ref multi-reference data, more aggressive than using the `#SOAP_XML_TREE` runtime flag 
`#WITH_NOHTTP`                | removes the HTTP stack to reduce code size 
`#WITH_NOZONE`                | disables and ignores the timezone in `xsd:dateTime` values
`#WITH_NOEMPTYNAMESPACES`     | disables xmlns="" default empty namespaces from XML messages
`#WITH_NOEMPTYSTRUCT`         | inserts a dummy member in empty structs to allow compilation 
`#WITH_NOGLOBAL`              | omit SOAP Header and Fault serialization code, prevents duplicate definitions with generated soapXYZLib code 
`#WITH_NONAMESPACES`          | disables dependence on global `namespaces` table, a table must be set explicitly with `::soap_set_namespaces` see also Section \ref nstable  
`#WITH_CDATA`                 | retains the parsed CDATA sections in literal XML strings
`#WITH_PURE_VIRTUAL`          | enables C++ abstract service classes with pure virtual methods, requires soapcpp2 option `-i` or `-j`
`#WITH_DEFAULT_VIRTUAL`       | enables C++ base service classes with default virtual methods returning fault `#SOAP_NO_METHOD`, requires soapcpp2 option `-i` or `-j`
`#WITH_CASEINSENSITIVETAGS`   | enables case insensitive XML parsing 
`#WITH_SOCKET_CLOSE_ON_EXIT`  | prevents a server port from staying in listening mode after exit by internally setting `fcntl(sock, F_SETFD, FD_CLOEXEC)` 
`#WITH_TCPFIN`                | enables TCP FIN after sends when socket is ready to close 
`#WITH_SELF_PIPE`             | enables a "self pipe" to enable the `::soap_close_connection` function (gSOAP 2.8.71 or greater)

The following subset of macros are defined in the API documentation Module \ref group_soap.  These macros are used to enable or disable features as specified below, by compiling source code files with compiler option <b>`-D`</b> to set the macro:

define                       | result
---------------------------- | ------
`#SOAP_NOTHROW`              | expands to `(std::nothrow)` to prevent memory allocation exceptions (`#SOAP_EOM` is used instead), this macro can be changed
`#SOAP_BUFLEN`               | the length of the internal message buffer, impacts communications performance
`#SOAP_HDRLEN`               | the maximum length of HTTP headers
`#SOAP_TAGLEN`               | the maximum length of XML tags and URLs
`#SOAP_TMPLEN`               | the maximum length of temporary string values, short strings and brief error messages
`#SOAP_MAXALLOCSIZE`         | the maximum size of a block of memory that `malloc` can allocate
`#SOAP_MAXARRAYSIZE`         | the maximum allocation threshold to pre-allocate SOAP arrays in memory
`#SOAP_MAXDIMESIZE`          | the maximum DIME attachment size allowed to receive
`#SOAP_MAXEINTR`             | maximum number of EINTR interrupts to ignore while polling a socket for pending activity
`#SOAP_MAXINFLATESIZE`       | trusted inflated content size (1 MB by default)
`#SOAP_MAXKEEPALIVE`         | maximum iterations in the `::soap_serve` loop on HTTP keep-alive connections
`#SOAP_MAXLENGTH`            | maximum string content length for strings not already constrained by XML schema validation constraints
`#SOAP_MAXLEVEL`             | maximum XML nesting depth level permitted by the XML parser
`#SOAP_MAXOCCURS`            | maximum number of array or container elements for containers that are not already constrained by XML schema validation constraints
`#SOAP_MINDEFLATERATIO`      | trusted deflation ratio after `#SOAP_MAXINFLATESIZE` is reached
`#SOAP_PURE_VIRTUAL`         | set to `= 0` when `#WITH_PURE_VIRTUAL` is defined
`#SOAP_SSL_RSA_BITS`         | length of the RSA key (2048 by default) 
`#SOAP_UNKNOWN_CHAR`         | an 8 bit integer that represents a character that could not be converted to an ASCII char, i.e. when converting an XML Unicode character
`#SOAP_UNKNOWN_UNICODE_CHAR` | integer Unicode value representing a character that replaces an invalid Unicode code point
`#SOAP_LONG_FORMAT`          | macro that represents the `#LONG64` printf %-format
`#SOAP_ULONG_FORMAT`         | macro that represents the `#ULONG64` printf %-format
`#SOAP_INVALID_SOCKET`       | portable invalid socket value, can also use `#soap_valid_socket(sock)` to check if `sock` is valid

@warning It is important that any of these macros when defined should be
consistently defined when compiling source code files, such as
<i>`gsoap/stdsoap2.cpp`</i>, <i>`soapC.cpp`</i>, <i>`soapClient.cpp`</i>,
<i>`soapServer.cpp`</i>, and all application source code files that include
<i>`gsoap/stdsoap2.h`</i> or <i>`soapH.h`</i>. If the macros are not consistently
defined at compile time then the application will likely crash due to a
mismatches in the declaration and use of the `::soap` context that is
customized by these flags.

🔝 [Back to table of contents](#)

## Using the soapdefs.h header file        {#soapdefs}

The <i>`soapdefs.h`</i> header file is included in <i>`gsoap/stdsoap2.h`</i> when compiling with compile-time flag `#WITH_SOAPDEFS_H`:

     c++ -D WITH_SOAPDEFS_H -c stdsoap2.cpp

The <i>`soapdefs.h`</i> file allows users to include definitions and add includes without requiring changes to <i>`gsoap/stdsoap2.h`</i>. You can also specify the header file name to include as a macro `SOAPDEFS_H` to override the name <i>`soapdefs.h`</i>:

     c++ -D SOAPDEFS_H=mydefs.h -c stdsoap2.cpp

For example,

~~~{.cpp}
    // Contents of file "soapdefs.h"
    #include <ostream> 
~~~

The following header file for soapcpp2 refers to `std::ostream` without soapcpp2 throwing errors, by using `extern` to declare `class std::ostream`:

~~~{.cpp}
    // std::ostream can't be serialized, but need to be declared to make it visible to gSOAP 
    extern class std::ostream;

    class ns__myClass 
    { public:
        virtual void print(std::ostream &s) const; // we need std::ostream here 
        ... //
    };
~~~

See also Section \ref transient , non-serializable data types.

🔝 [Back to table of contents](#)

# Run-time flags        {#flags}

The gSOAP engine state is stored in the `::soap` context that is initialized
and controlled by various optional runtime flags.  Most flags are applicable to
either processing input or to processing output, but some are applicable to
both input and output message and document processing.

Furthermore, these flags are divided into four categories: transport (IO),
content encoding (ENC), XML parsing and generation (XML), and C/C++ usage (C).

The input-mode and output-mode flags for inbound (in) and outbound (out) message processing are:

mode flag             | in/out | result
--------------------- | ------ | ------
`#SOAP_IO_FLUSH`      | out    | flush output immediately
`#SOAP_IO_BUFFER`     | out    | enable output buffering, the default mode for socket connections
`#SOAP_IO_STORE`      | out    | store the entire outbound message to calculate HTTP content length 
`#SOAP_IO_CHUNK`      | out    | enable HTTP chunking
`#SOAP_IO_LENGTH`     | out    | used with two-phase serialization, first phase with this flag to calculate HTTP content length 
`#SOAP_IO_KEEPALIVE`  | in+out | keep socket connections alive, when supported by the HTTP peer
`#SOAP_IO_UDP`        | in+out | use UDP (datagram) transport, maximum message length is `#SOAP_BUFLEN` 
`#SOAP_ENC_PLAIN`     | in+out | use plain messages without parsing or emitting HTTP headers 
`#SOAP_ENC_XML`       |        | alias for `#SOAP_ENC_PLAIN` 
`#SOAP_ENC_DIME`      | out    | use DIME encoding, automatic when DIME attachments are used
`#SOAP_ENC_MIME`      | out    | use MIME encoding instead of DIME, activated using `::soap_set_mime`
`#SOAP_ENC_MTOM`      | out    | use MTOM XOP attachments instead of DIME and MIME
`#SOAP_ENC_ZLIB`      | out    | compress output with Zlib, using deflate or gzip format
`#SOAP_ENC_SSL`       | in+out | use SSL/TLS, automatic when connecting "https:" endpoints
`#SOAP_XML_INDENT`    | out    | output indented XML and JSON
`#SOAP_XML_CANONICAL` | out    | output canonical XML
`#SOAP_XML_DEFAULTNS` | out    | output XML with default namespace bindings `xmlns="..."`
`#SOAP_XML_IGNORENS`  | in     | ignores XML namespaces in XML input 
`#SOAP_XML_STRICT`    | in     | apply strict validation of XML input
`#SOAP_XML_TREE`      | in+out | out: serialize data as XML trees (no multi-ref, duplicate data when necessary); in: ignore id attributes (do not resolve id-ref)
`#SOAP_XML_GRAPH`     | out    | serialize application data as an XML graph with multi-ref id/ref attributes
`#SOAP_XML_NIL`       | out    | serialize NULL data as xsi:nil attributed elements 
`#SOAP_XML_NOTYPE`    | out    | disable <i>`xsi:type`</i> attributes 
`#SOAP_C_NOIOB`       | in     | do not fault with `#SOAP_IOB` 
`#SOAP_C_UTFSTRING`   | in+out | serialize 8-bit character strings "as is", meaning 8-bit strings have UTF-8 content
`#SOAP_C_MBSTRING`    | in+out | enable multibyte character support for 8-bit character strings with `wctomb` and `mbtowc` using the current locale
`#SOAP_C_NILSTRING`   | out    | serialize empty strings as xsi:nil attributed elements 

All flags are independent and can be combined using a bitwise or (`|`), except for
`#SOAP_IO_FLUSH`,
`#SOAP_IO_BUFFER`,
`#SOAP_IO_STORE`, and
`#SOAP_IO_CHUNK`
which are enumerations and only one of these I/O flags can be used.  Also the
XML serialization flags
`#SOAP_XML_TREE` and
`#SOAP_XML_GRAPH` should not be mixed.

To allocate and initialize a `::soap` context with input and output mode flags:

~~~{.cpp}
    struct soap * soap_new1(soap_mode iomode);
    struct soap * soap_new2(soap_mode imode, soap_mode omode);
~~~

To initialize a stack-allocated `::soap` context with input and output mode flags:

~~~{.cpp}
    void soap_init1(struct soap *soap, soap_mode iomode);
    void soap_init2(struct soap *soap, soap_mode imode, soap_mode omode);
~~~

To set and clear mode flags use:

~~~{.cpp}
    void soap_set_mode(struct soap *soap, soap_mode iomode); // set input and output mode flags
    void soap_set_imode(struct soap *soap, soap_mode imode); // set input mode flags
    void soap_set_omode(struct soap *soap, soap_mode omode); // set output mode flags
    void soap_clr_mode(struct soap *soap, soap_mode iomode); // clear input and output mode flags
    void soap_clr_imode(struct soap *soap, soap_mode imode); // clear input mode flags
    void soap_clr_omode(struct soap *soap, soap_mode omode); // clear output mode flags
~~~

@note The `#SOAP_XML_TREE` mode flag can be used to improve interoperability
with SOAP implementations that are not fully SOAP 1.1/1.2 RPC encoding
compliant with respect to processing id-href/ref attributes.  However, tree
serialization will duplicate data when this data is co-referenced.  Cycles are
detected and broken to avoid infinite serialization.

In addition to the context mode flags, the following context flags can be used to set `setsockopt` level `SOL_SOCKET` options when sockets are created, though some options may not be applicable to your operating system:

context flag with setsockopt option      | result
---------------------------------------- | ------
`::soap::connect_flags` = `SO_NOSIGPIPE` | disables SIGPIPE
`::soap::connect_flags` = `SO_DEBUG`     | turns on recording of debugging information in the underlying protocol modules 
`::soap::connect_flags` = `SO_BROADCAST` | permits sending of broadcast messages, for example with UDP, when permitted 
`::soap::connect_flags` = `SO_LINGER`    | sets client-side linger time to the value of `::soap::linger_time` 
`::soap::connect_flags` = `SO_DONTROUTE` | enables routing bypass for outgoing messages 
`::soap::accept_flags` = `SO_NOSIGPIPE`  | disables SIGPIPE (check your OS, this is not portable) 
`::soap::accept_flags` = `SO_DEBUG`      | turns on recording of debugging information in the underlying protocol modules 
`::soap::accept_flags` = `SO_LINGER`     | sets server-side linger time to the value of `::soap::linger_time` 
`::soap::accept_flags` = `SO_DONTROUTE`  | enables routing bypass for outgoing messages 
`::soap::bind_flags` = `SO_REUSEADDR`    | enables local address reuse immediately, use with caution
`::soap::bind_flags` = `SO_REUSEPORT`    | enables duplicate address and port bindings

For example, when `::soap::accept_flags` is set to `(SO_NOSIGPIPE | SO_LINGER)` this disables SIGPIPE signals and set linger time value given by `::soap::linger_time`, which is zero by default.

The `::soap::socket_flags` context flag can be used to pass options to the `<sys/socket.h>` `send`, `sendto`, `recv`, and `recvfrom` calls made by the engine, though some options may not be applicable to your operating system:

context flag with sent/recv flags        | result
---------------------------------------- | ------
`::soap::socket_flags` = `MSG_NOSIGNAL`  | disables SIGPIPE
`::soap::socket_flags` = `MSG_DONTROUTE` | enables routing bypass for outgoing messages

Furthermore, the `setsockopt` level `SOL_SOCKET` options `SO_SNDBUF` and `SO_RCVBUF` are set the engine when `::soap::sndbuf` and `::soap::rcvbuf` are set to a nonzero value.  The default value is `#SOAP_BUFLEN`, which is the same size used by the internal buffer `::soap::buf` to send and receive messages.  A zero value omits the internal `setsockopt` call to set these options.  Setting these values to zero activates auto-tuning with Linux 2.4 and greater.

The `setsockopt` level `SOL_SOCKET` option `SO_KEEPALIVE` is set when keep-alive is enabled with context flag `#SOAP_IO_KEEPALIVE` or when `::soap::tcp_keep_alive` is nonzero.  With `::soap::tcp_keep_alive` additional options can be specified with  `::soap::tcp_keep_idle`,  `::soap::tcp_keep_intvl`, and  `::soap::tcp_keep_cnt`. For example:

~~~{.cpp}
    struct soap *soap = soap_new();
    soap->tcp_keep_alive = 1;   // setsockopt SO_KEEPALIVE
    soap->tcp_keep_idle = 30;   // time in seconds the connection needs to remain idle before TCP starts sending keepalive probes
    soap->tcp_keep_intvl = 120; // time in seconds between individual keepalive probes
    soap->tcp_keep_cnt = 5;     // maximum number of keepalive probes TCP should send before dropping the connection
~~~

For UDP messaging, use `#SOAP_IO_UDP`.  See also Section \ref UDP.  The context flags that can be set at the client side for UDP messaging are `::soap::ipv4_multicast_if`, `::soap::ipv4_multicast_ttl`, and `::soap::ipv6_multicast_if`:

context flag                 | result
---------------------------- | ------
`::soap::ipv4_multicast_if`  | set `setsockopt` level `IPPROTO_IP` to `IP_MULTICAST_IF` with value `::soap::ipv4_multicast_if` when non-NULL
`::soap::ipv4_multicast_ttl` | set `setsockopt` level `IPPROTO_IP` to `IP_MULTICAST_TTL` with value `::soap::ipv4_multicast_ttl` when nonzero
`::soap::ipv6_multicast_if`  | set `sockaddr_in6::sin6_scope_id` to `::soap::ipv6_multicast_if` when nonzero

🔝 [Back to table of contents](#)

# Run-time error codes        {#errcodes}

Status error codes are integer values, which are returned by the gSOAP API functions.  The full list of `::soap_status` error codes is listed below:

Error code                  | Description
--------------------------- | -----------
`#SOAP_OK`                  | No error (zero)
`#SOAP_CLI_FAULT`           | The service returned a SOAP 1.1 client fault / SOAP 1.2 sender fault to the client
`#SOAP_DATAENCODINGUNKNOWN` | SOAP 1.2 DataEncodingUnknown fault 
`#SOAP_DEL_METHOD`          | An HTTP DELETE request was received by the service but the DELETE request callback `soap::fdel` is not implemented, see Section \ref callback 
`#SOAP_DIME_END`            | End of DIME attachments protocol error 
`#SOAP_DIME_ERROR`          | DIME formatting error or DIME attachment size exceeds `#SOAP_MAXDIMESIZE`
`#SOAP_DIME_HREF`           | DIME attachment has no href from SOAP body and no DIME callbacks were defined to save the attachment 
`#SOAP_DIME_MISMATCH`       | DIME version error 
`#SOAP_DUPLICATE_ID`        | XML element has duplicate id attribute value (applicable to SOAP multi-ref encoding and `#SOAP_XML_GRAPH` serialization) 
`#SOAP_EMPTY`               | XML element or attribute is empty when a value is required
`#SOAP_EOF`                 | Unexpected end of file, no input, transmission interrupted or timed out, same as `EOF`
`#SOAP_EOM`                 | Out of memory 
`#SOAP_ERR`                 | Same as `EOF`, but indicates an unspecified error
`#SOAP_FAULT`               | The fault code to be returned by a service operation when calling `::soap_sender_fault` (client is at fault) or `::soap_receiver_fault` (server is at fault), clients receive the fault as `#SOAP_CLI_FAULT` or `#SOAP_SVR_FAULT` respectively
`#SOAP_FD_EXCEEDED`         | Too many open sockets
`#SOAP_FIXED`               | XML element or attribute value is fixed and the parsed value does not match the fixed value
`#SOAP_GET_METHOD`          | An HTTP GET request was received by the service but the GET request callback `soap::fget` is not implemented, see Sections \ref get and \ref callback 
`#SOAP_HREF`                | Reference to object in XML identified by its id attribute is incompatible with the object referred to by the ref or href attribute (applicable to SOAP multi-ref encoding and `#SOAP_XML_GRAPH` serialization) 
`#SOAP_HTTP_ERROR`          | An unspecified HTTP error occurred 
`#SOAP_HTTP_METHOD`         | An HTTP request was received by the service that cannot be handled
`#SOAP_IOB`                 | SOAP array index out of bounds 
`#SOAP_LENGTH`              | XML element or attribute length validation error or `#SOAP_MAXLENGTH` exceeded
`#SOAP_LEVEL`               | XML nesting depth level exceeds `#SOAP_MAXLEVEL`
`#SOAP_MIME_END`            | End of MIME attachments protocol error 
`#SOAP_MIME_ERROR`          | MIME attachment parsing error 
`#SOAP_MIME_HREF`           | MIME attachment has no href from SOAP body and no MIME callbacks were defined to save the attachment
`#SOAP_MISSING_ID`          | An XML element with id attribute is missing that should match the element with href/ref attribute (applicable to SOAP multi-ref encoding and `#SOAP_XML_GRAPH` serialization) 
`#SOAP_MOE`                 | Memory overflow or memory corruption error (applicable in `#DEBUG` mode only) 
`#SOAP_MUSTUNDERSTAND`      | An XML element is present with a mustUnderstand attribute which must be understood but is not deserialized
`#SOAP_NAMESPACE`           | XML namespace name mismatch validation error
`#SOAP_NO_DATA`             | No data in the HTTP body of the message received 
`#SOAP_NO_METHOD`           | The service request dispatcher did not find a matching service operation for a service request
`#SOAP_NO_TAG`              | No XML element tag was found when one was expected 
`#SOAP_NTLM_ERROR`          | An NTLM authentication handshake error occurred 
`#SOAP_NULL`                | XML element has an `xsi:nil` attribute when the element is not nillable, causing a validation error 
`#SOAP_OCCURS`              | XML element minOccurs or maxOccurs validation error or `#SOAP_MAXOCCURS` exceeded 
`#SOAP_PATCH_METHOD`        | An HTTP PATCH request was received by the service but the PATCH request callback `soap::fpatch` is not implemented, see Section \ref callback 
`#SOAP_PATTERN`             | XML element or attribute value pattern mismatch
`#SOAP_PLUGIN_ERROR`        | Failed to register plugin 
`#SOAP_PROHIBITED`          | XML attribute is prohibited but present
`#SOAP_PUT_METHOD`          | An HTTP PUT request was received by the service but the PUT request callback `soap::fput` is not implemented, see Section \ref callback  
`#SOAP_REQUIRED`            | XML attribute is required but not present
`#SOAP_SSL_ERROR`           | An SSL error occurred 
`#SOAP_SVR_FAULT`           | The service returned a SOAP 1.1 server fault or SOAP 1.2 receiver fault to the client
`#SOAP_SYNTAX_ERROR`        | An XML syntax error occurred while parsing the input 
`#SOAP_TAG_MISMATCH`        | XML element tag parsed does not match anything that is expected
`#SOAP_TCP_ERROR`           | A TCP/IP connection error occurred 
`#SOAP_TYPE`                | XML element or attribute has a mismatching type or value that is causing a validation error
`#SOAP_UDP_ERROR`           | A UDP/IP connection error occurred or the message too large to store in a UDP packet 
`#SOAP_USER_ERROR`          | soap::user not set to non-NULL
`#SOAP_UTF_ERROR`           | An UTF-8 decoding error occurred 
`#SOAP_VERSIONMISMATCH`     | SOAP version mismatch or no SOAP message is received
`#SOAP_ZLIB_ERROR`          | A zlib error occurred 

A status code of `#SOAP_OK` (zero) is returned by a gSOAP API function when the function call was successful, otherwise a non-zero error code is returned.  The status error code is also stored in the current `::soap` context as `::soap::error`.  For IO and socket-related errors, also the `::soap::errnum` variable is set to the `errno` value of the system error for example when a `#SOAP_EOF` error occurred.

To display the error, use `soap_print_fault(struct soap *soap, FILE *fd)` where the current value of
`::soap::error` is used by the function to print the error to the specified `fd` file.  Alternatively, in C++ you can use `soap_stream_fault(struct soap *soap, std::ostream& os)` to print the error on the specified `os` output stream.

To display the location of an XML parsing and validation error, use `soap_print_fault_location(struct soap *soap, FILE *fd)` or `soap_stream_fault_location(struct soap *soap, std::ostream& os)` to print part of the XML with the error location marked in the XML output.

To save the error in a string buffer `buf[0...len-1]`, use `soap_sprint_fault(struct soap*, char *buf, size_t len)`, where `buf` is populated with the fault message terminating with a `\0`.

To determine the type of error that occurred, use:
- `#soap_xml_error_check(soap_status e)` checks for XML parsing and validation errors, returns true if the specified error code is an XML error.
- `#soap_soap_error_check(soap_status e)` checks for SOAP protocol faults and errors, returns true if the specified error code is a SOAP protocol error.
- `#soap_http_error_check(soap_status e)` checks for HTTP protocol errors, returns true if the specified error code is an HTTP protocol error or a HTTP status code between 100 and 599 returned by an HTTP server.
- `#soap_tcp_error_check(soap_status e)` checks for TCP protocol errors, returns true if the specified error code is a TCP error, when true use `::soap::errnum` to retrieve the `errno` value to determine the cause.
- `#soap_udp_error_check(soap_status e)` checks for UDP protocol errors, returns true if the specified error code is a UDP error, when true use `::soap::errnum` to retrieve the `errno` value to determine the cause.
- `#soap_ssl_error_check(soap_status e)` checks for SSL/TLS protocol errors, returns true if the specified error code is a SSL/TLS error, when true use `::soap::errnum` to retrieve the `errno` value to determine the cause.
- `#soap_zlib_error_check(soap_status e)` checks for zlib library errors, returns true if the specified error code is a zlib error.

An HTTP status code is returned when the client fails to connect to an HTTP server and the HTTP server response with an error.  The list of HTTP status codes is given below:

Code | Description
---- | -----------
200  | OK (no error) 
201  | Created 
202  | Accepted 
203  | Non-Authoritative Information 
204  | No Content 
205  | Reset Content 
206  | Partial Content 
300  | Multiple Choices 
301  | Moved Permanently 
302  | Found 
303  | See Other 
304  | Not Modified 
305  | Use Proxy 
307  | Temporary Redirect 
400  | Bad Request 
401  | Unauthorized 
402  | Payment Required 
403  | Forbidden 
404  | Not Found 
405  | Method Not Allowed 
406  | Not Acceptable 
407  | Proxy Authentication Required 
408  | Request Time-out 
409  | Conflict 
410  | Gone 
411  | Length Required 
412  | Precondition Failed 
413  | Request Entity Too Large 
414  | Request-URI Too Large 
415  | Unsupported Media Type 
416  | Requested range not satisfiable 
417  | Expectation Failed 
500  | Internal Server Error 
501  | Not Implemented 
502  | Bad Gateway 
503  | Service Unavailable 
504  | Gateway Time-out 
505  | HTTP Version not supported 

HTTP status code 200 is not flagged as an error by the engine.  Status codes 201 and 202 are informative and should not be considered errors by the application logic.

Server-side implementations of service operations should return `#SOAP_OK` when the operation was successful, which returns a "200 OK" HTTP header with the XML response message.  Server-side implementations of service operations in gSOAP may directly return an HTTP status code when an HTTP-related error should be returned.  For example, `return 404` returns "404 Not Found" to the client and the `::soap::error` variable is set to 404 at the client side.

To return a SOAP Fault from a server-side implementation of a service operation, use one of the following functions to populate the SOAP Fault message:
- `int soap_sender_fault(struct soap *soap, const char *faultstring, const char *faultdetail)` returns `#SOAP_FAULT` and populates a SOAP Fault message indicating a SOAP 1.1 client fault / SOAP 1.2 sender fault that is populated with the text `faultstring` and an XML fragment `faultdetail` or NULL when omitted.
- `int soap_sender_fault_subcode(struct soap *soap, const char *faultsubcode, const char *faultstring, const char *faultdetail)` same as `::soap_sender_fault` with the addition of a QName-formatted string `faultsubcode`.
- `int soap_receiver_fault(struct soap *soap, const char *faultstring, const char *faultdetail)` returns `#SOAP_FAULT` and populates a SOAP Fault message indicating a SOAP 1.1 server fault / SOAP 1.2 receiver fault that is populated with the text `faultstring` and an XML fragment `faultdetail` or NULL when omitted.
- `int soap_receiver_fault_subcode(struct soap *soap, const char *faultsubcode, const char *faultstring, const char *faultdetail)` same as `::soap_receiver_fault` with the addition of a QName-formatted string `faultsubcode`.

A receiver error indicates that the service could not handle the client request, but it can possibly recover from the error later, for example when resources are temporarily unavailable.  A sender error indicates that the client request is faulty and s rejected by the service.

See Section \ref fault for more details on how to use these functions.

🔝 [Back to table of contents](#)

# Memory management        {#memory}

Memory management with gSOAP is automatic.  The engine context manages all memory allocated to serialize data and for temporary storage.  Deserialized data is allocated in managed memory and data structures can be allocated in managed memory by the user when desired using `soap_new_T` functions generated by soapcpp2 for each serializable type `T`.  All memory managed by a context is deallocated with `::soap_destroy` to destroy managed C++ objects and `::soap_end` to delete all other managed data.  When a context is finalized or freed with `soap_done(struct soap*)` and `soap_free(struct soap*)` then managed memory is not released, so it is important to call the deallocation functions first.  This was done to allow managed data to outlive the context, such as deserialized objects, but this is rarely if ever used because deleting the outlived data explicitly is prone to mistakes.

If you want to let deserialized data outlive a `::soap` context that you are about to free, then you can delegate management of the data to another `::soap` context with `soap_delegate_deletion(struct soap *soap_from, struct soap *soap_to)`. This moves all deserialized and temporary data to the other `::soap` context `soap_to`, which will delete its data and all the delegated data it is responsible for when you call `::soap_destroy` and `::soap_end`. This can be particularly useful for making client calls inside a server operation, i.e. a mixed server and client. The client call inside the server operation requires a new `::soap` context, e.g. copied from the server's with `::soap_copy`. Before destroying the client context with `::soap_free`, the data can be delegated to the server's context with `::soap_delegate_deletion`. See for example <i>`gsoap/samples/mashup/mashupserver.c`</i> in the gSOAP source code package.

The functions related to memory management by the context are:

* `void * soap_malloc(struct soap *soap, size_t n)` return pointer to `n` bytes of managed memory.

* `char * soap_strdup(struct soap *soap, const char *s)` return pointer to duplicate of string `s` in managed memory.

* `char * soap_wstrdup(struct soap *soap, const wchar_t *s)` return pointer to duplicate of string `s` in managed memory.

* `T * soap_new_T(struct soap *soap)` allocates and default-initializes data of type `T`, C++ only.

* `T * soap_new_T(struct soap *soap, int n)` allocates and default-initializes and array of type `T`, where `n` = -1 allocates a single non-array value.

* `T * soap_new_set_T(struct soap *soap, m1, ..., mn)` allocates a struct or class `T` and initializes its members with the values `m1` to `mn`.

* `T * soap_new_req_T(struct soap *soap, m1, ..., mn)` allocates a struct or class `T` and initializes its required members with the values `m1` to `mn`, required means non-optional member as required by XML validation when this data is serialized in XML.

* `void soap_destroy(struct soap *soap)` deletes all context-managed C++ objects and must be called before `::soap_end`.

* `void soap_end(struct soap *soap)` deletes temporary data and all deserialized data except C++ objects, see above.

* `void soap_free_temp(struct soap *soap)` deletes temporary data but leaves deserialized context-managed data intact.

* `void soap_dealloc(struct soap *soap, void *p)` deallocates context-managed objects or data at location `p` in memory.

* `int soap_unlink(struct soap *soap, const void *p)` unlink object or data at location `p` from management by the context, this object or data must be deallocated by the user.

* `void soap_done(struct soap *soap)` finalizes the context but does not delete any managed objects or data.

* `void soap_free(struct soap *soap)` finalizes and frees the context (contexts allocated with `::soap_new` or `::soap_copy`) but does not delete any managed objects or data.

* `void soap_delegate_deletion(struct soap *soap_from, struct soap *soap_to)` moves all C++ objects, data, and temporary data managed by the `soap_from` context to another context `soap_to`, which manages the data until deleted with `::soap_destroy` and `::soap_end`.

To help understand the differences between managed objects, managed data, and
managed temporary data: temporary data is created by the engine to keep track
of things, such as hash tables to keep pointer reference information for
serialization and hash tables to keep XML id/href information for
multi-reference object deserialization.  Deserialized data is allocated by the
context in managed memory when constructing data structures by deserializing XML and
JSON messages.  Data is stored in memory managed by
the context using calls to `::soap_malloc` to allocate heap space with `malloc`.
A tiny bit more space is allocated to keep track of the allocations and to
add a "canary" word to detect heap memory overflows.  Heap memory overflows are
detected when context-managed data is deallocated with `::soap_end`.  C++
objects are allocated with `new` instead of `malloc` and are tracked as well.
These objects are deallocated with `::soap_destroy`.

When the gSOAP application is compiled with <b>`-DDEBUG`</b> using the compile-time flag `#DEBUG`, the engine reports memory leaks which are detected by `::soap_done` and `::soap_free`.  To improve the accuracy of detection, no memory is actually freed until `::soap_done` or `::soap_free` are called to detect memory issues, so any calls to `::soap_destroy` and `::soap_end` are actually deferred to be executed when the context finalizes.

While memory management in gSOAP is automatic, it does not enforce its own memory management policy on the user.  To move managed objects and data into unmanaged heap space, the `soap_dup_T` deep copy functions generated by <b>`soapcpp2 -Ec`</b> option <b>`-Ec`</b> can be used.  To delete deep copies, the `soap_del_T` deep deletion functions generated by <b>`soapcpp2 -Ed`</b> option <b>`-Ed`</b> can be used. After copying the usual `::soap_destroy` and `::soap_end` functions remove the managed originals.  See also Section \ref deep.

Furthermore, the memory allocation functions `malloc` and `new` used by the engine internally can be replaced with other allocators by defining `SOAP_MALLOC` and `SOAP_FREE` to replace `malloc` and `free`, and define `SOAP_NEW`, `SOAP_NEW_ARRAY`, `SOAP_PLACEMENT_NEW`, and `SOAP_DELETE`, `SOAP_DELETE_ARRAY` to replace `new` and `delete` used by the engine to allocate managed memory.  One can for example use a garbage collector with gSOAP by defining suitable replacements.

More information on memory management can be found in the [C and C++ XML Data Bindings](../../databinding/html/index.html) documentation that has separate sections on memory management in C and in C++.

🔝 [Back to table of contents](#)

# Intra-class memory management        {#classmemory}

When a class `T` has a `struct soap * T::soap` member declared in an interface header file
for soapcpp2, then this member will be set to point to the current context by
the deserializers and by the `soap_default` method of the class and by the
`soap_default_T` and `soap_new_T` functions for this class named `T`.  This
simplifies memory management by class methods that allocate data associated
with the class instance that must be managed by the context.

Consider for example the following class declaration:

~~~{.cpp}
    class Class 
    { public:
        Class();
        ~Class();
        struct soap *soap;
        char *name; 
        void setName(const char *s); 
        char *getName();
    };
~~~

Since the `name` member is a character pointer to a string, where should we
allocate this string? In most cases we will add a constructor that initially
sets `name` to NULL and a destructor that deletes `name` when non-NULL, like
so:

~~~{.cpp}
    Class::Class() 
    {
      soap = NULL;
      name = NULL;
    } 
    Class::~Class() 
    {
      if (name)
        free(name);
    }
~~~

However, when instances of `Class` are deserialized we have a problem with this
approach because `free(name)` deletes a managed string, which is managed by the
context.  Because the deserializer also sets the `soap` member of this class,
there is an easy solution to this problem:

~~~{.cpp}
    Class::~Class() 
    {
      if (!soap && name)
        free(name);
    }
~~~

This only frees `name` if the `soap` context pointer member is NULL, meaning
that a managed string `name` will be deleted as usual with the deserialized
class using `::soap_destroy` and `::soap_end`.

The other methods are also made cognizant of the presence of a context:

~~~{.cpp}
    void Class::setName(const char *s)
    {
      if (soap)
        name = soap_strdup(soap, s);
      else
        name = strdup(s);
    }
    const char *getName()
    {
      return name;
    }
~~~

Another approach is to use `::soap_unlink` to unlink data managed by the
context and make all allocations explicitly:

~~~{.cpp}
    Class::~Class() 
    {
      if (name)
      {
        soap_unlink(soap, name); // unlinks the name if soap is non-NULL
        free(name);
      }
    }
    void Class::setName(const char *s)
    {
      name = strdup(s);
    }
    const char *getName()
    {
      return name;
    }
~~~

The `::soap_unlink` call unlinks data and objects from managed memory.  In this
way `::soap_destroy` and `::soap_end` can be called later even when this class
was deserialized.

🔝 [Back to table of contents](#)

# Debugging  {#debugging}

To activate debugging and message logging compile the source code with compile-time flag `#DEBUG`.
Or when using <b>`-lgsoap`</b> or <b>`-lgsoap++`</b> then reinstall gSOAP with
<b>`./configure --enable-debug`</b> and <b>`make`</b>.

When your gSOAP client or server applications run, they will log their activity in three
separate files:

* <i>`SENT.log`</i> a concatenation of all messages sent

* <i>`RECV.log`</i> a concatenation of all messages received

* <i>`TEST.log`</i> a log of the engine's operations

@warning The gSOAP client and server applications will run slow due to debugging and message logging.

You can set macro `#DEBUG_STAMP` instead of `#DEBUG` to add time
stamps to `TEST.log`. This works on all operating systems supporting the
`gettimeofday` function.

When installing a CGI service application compiled with debugging enabled, the log files may sometimes not be created due to file
access permission restrictions imposed on CGI applications. To get around this, create empty log files with universal write
permissions. Be careful about the security implication of this.

You can actually test a CGI service application without deploying it on the Web.
To do this, create the CGI service application and send it a request message using redirection as follows:

    ./service.cgi < ns.add.req.xml

this should display the service response on the terminal,
where <i>`ns.add.req.xml`</i> was generated by soapcpp2 or a modified version of this file.  You can also use a <i>`SENT.log`</i> file produced by a client application to redirect to the CGI service application.
You can also use the gSOAP [Test Messenger](../../testmsgr/html/index.html) application to generate randomized messages to test your servers.

The file names of the log files and the logging activity can be controlled at the application level. This allows the creation of
separate log files by separate services, clients, and threads.
For example, the following service logs all messages (but no debug messages) in separate directories:

~~~{.cpp}
    struct soap *soap = soap_new(); 
    soap_set_recv_logfile(soap, "logs/recv/service12.log"); // append all messages received in /logs/recv/service12.log 
    soap_set_sent_logfile(soap, "logs/sent/service12.log"); // append all messages sent in /logs/sent/service12.log 
    soap_set_test_logfile(soap, NULL);                      // no file name: do not save debug messages 
    ... //
    soap_serve(soap); 
    ... //
    soap_free(soap);
~~~

Likewise, messages can be logged for individual client-side service operation calls in a client application.

To log messages more efficiently, use the `::logging` plugin.

🔝 [Back to table of contents](#)

# Limitations {#limitations}

From the perspective of the C/C++ language, a few C/C++ language features are not supported by gSOAP and these features cannot be used in an interface header file for soapcpp2.

* STL: the soapcpp2 tool supports the serialization of C++ strings `std::string` and `std::wstring` and the containers `std::deque`, `std::list`, `std::vector`, and `std::set`, (see Section \ref templates ).  Also `std::shared_ptr`, `std::unique_ptr`, and `std::auto_ptr` are supported.  Other STL types are not serializable.

* Templates: the soapcpp2 tool assumes that templates classes have only one template parameter type and these templates are containers of values of this template parameter type.  This template class should define `begin()`, `end()`, `size()`, `clear()`, and `insert()` methods.

* Inheritance: single class inheritance is supported.

* Abstract methods: a class must be instantiable to support serialization of that class.  An abstract class when used should be declared transient, see Section \ref transient.

* Directives: directives and pragmas such as `#include` and `#define` are moved to the generated <i>`soapStub.h`</i> source code by soapcpp2. These directives are all placed at the top of that file, see \ref pragmas.
Use the `#import` directive to import interface header files into other interface header files, see Section \ref import .

* C and C++ code statements: no code statements are allows in interface header files. Class methods should be declared without code in `{` and `}`.  Also constructor initializers are not allowed.  Class method implementations should be defined in a separate C++ source file.

* C++ references: these cannot be serialized, use pointers instead.

The following C/C++ data types require some attention to ensure they can be serialized:

* Union types: a union data type can not be serialized unless run-time information is associated with a union for the serializer to determine which union member is valid.  To serialize a union, place the union in a struct or class as a member and precede this member with an integer member that serves as a selector.  See Section \ref union for details.

* `void*` types: the `void*` data type cannot be serialized unless run-time type information is associated with the pointer using a `int __type` member in the struct or class that contains the `void*` member. See Section \ref void for details.

* Pointers to arrays: any pointer, except for C strings which are pointers to an array of
characters, are treated by the soapcpp2 tool as if the pointer points to only one value. Consequently,
the serialization functions for pointers only serialize the first value pointed to, when the pointer is non-NULL.  To declare pointers to arrays as struct and class members, a size member is included in the struct or class, see Section \ref dynarray  for details.

* Constant values: `const` constant values such as `const` members of a struct or class cannot be serialized except for `const char*` and `const wchar_t*` strings.

* Uninitialized pointers: Obviously, all pointers that are part of a data structure to serialize must be valid or NULL.  Otherwise the serializer will crash.

🔝 [Back to table of contents](#)

# Advanced features        {#advanced}

🔝 [Back to table of contents](#)

## Internationalization    {#internationalization}

Regular 8-bit strings cannot hold
wide characters outside of the character range [1,255].  Of course you 
can use wide strings instead of 8-bit strings in the interface header file for soapcpp2.
Alternatively, set the `#SOAP_C_UTFSTRING` mode flag to encode wide characters in 8-bit strings in UTF-8 format.

For example, the <i>`xsd:string`</i> string schema type
can be declared as a wide-character string and used subsequently:

~~~{.cpp}
    typedef wchar_t *xsd__string; 
    int ns__myMethod(xsd__string input, xsd__string *output);
~~~

To do so automatically with wsdl2h, edit <i>`typemap.dat`</i> and add the line:

    xsd__string = | wchar_t* | wchar_t*

or for C++:

    xsd__string = | std::wstring | std::wstring*

Alternatively, 8-bit strings can hold UTF-8 formatted wide characters when the
`#SOAP_C_UTFSTRING` flag is enabled, see Section \ref flags .
The application is responsible for filling
regular strings with UTF-8 content.

Also the `#SOAP_C_MBSTRING` flag can be used to activate multibyte character support using the current locale. Multibyte support depends on the locale settings for dealing with extended natural language encodings.

Both 8-bit strings and wide-character strings can be used together within an application.
For example, the following header file declaration introduces two string schema types:

~~~{.cpp}
    typedef wchar_t *xsd__string; 
    typedef char *xsd__string_; // trailing '_' avoids name clash 
    int ns__myMethod(xsd__string input, xsd__string_ *output);
~~~

Please consult the UTF-8 specification for details on the UTF-8 format when processing 8-bit strings with UTF-8 content.
Note that the ASCII character set [1-127] is a subset of UTF-8. Therefore, with the `#SOAP_C_UTFSTRING` flag set, strings may hold ASCII character data and UTF-8 extensions.

See also [C and C++ XML data bindings](../../databinding/html/index.html)
documentation for more details on using <i>`typemap.dat`</i> for wsdl2h.

🔝 [Back to table of contents](#)

## Directives        {#directives}

An interface header file for soapcpp2 may include `//gsoap` directives to configure messaging protocols such as SOAP or REST, to associate SOAP Headers and Faults with messages, and to set properties of the Web service defined in the soapcpp2-generated WSDL and XSD files.
Directives for soapcpp2 are specified as `//gsoap`-comments that are processed by soapcpp2.

### Service directives                                               {#directives-1}

A service directive must start at a new line and is of the form:

~~~{.cpp}
    //gsoap <prefix> service <property>: <value>
~~~

where `<prefix>` is the XML namespace prefix of a service binding.  The
`<property>` and `<value>` fields are one of the following:

property        | value
--------------- | -----
`name`          | name of the service, optionally followed by text describing the service
`namespace`     | URI of the WSDL targetNamespace
`documentation` | text describing the service (see also the `name` property), multiple permitted
`doc`           | an alias for the `documentation` property
`style`         | `document` (default) SOAP messaging style or `rpc` for SOAP RPC
`encoding`      | `literal` (default), `encoded` for SOAP encoding, or a custom URI
`protocol`      | specifies SOAP or REST, see below
`port`          | URL of the service endpoint, usually an http or https address, to use in the WSDL definitions/service/port/address/\@location
`location`      | an alias for the `port` property
`endpoint`      | an alias for the `port` property
`transport`     | URI declaration of the transport, usually `http://schemas.xmlsoap.org/soap/http`
`definitions`   | name of the WSDL definitions/\@name
`type`          | name of the WSDL definitions/portType/\@name (WSDL2.0 interface/\@name)
`portType`      | an alias for the `type` property (`portType` follows SOAP 1.1 naming conventions)
`interface`     | an alias for the `type` property (`interface` follows SOAP 1.2 naming conventions)
`binding`       | name of the WSDL definitions/binding/\@name
`portName`      | name of the WSDL definitions/service/port/\@name
`executable`    | name of the "executable" to use in the WSDL definitions/service/port/address/\@location

The service `name` and `namespace` properties are required in order to generate
a valid WSDL with soapcpp2.  The other properties are optional.

The `style` and `encoding` property defaults are changed with
<b>`soapcpp2 -e`</b> option <b>`-e`</b> to `rpc` and `encoded`.

You can override the `port` endpoint URL at runtime in the auto-generated
`soap_call_prefix__func` service call (C/C++ client side) and in the C++ proxy
class service call.

Protocol property values are:

protocol value | description
-------------- | -----------
`SOAP`         | SOAP transport, supporting both SOAP 1.1 and 1.2
`SOAP1.1`      | SOAP 1.1 transport (same as `soapcpp2 -1`)
`SOAP1.2`      | SOAP 1.2 transport (same as `soapcpp2 -2`)
`SOAP-GET`     | one-way SOAP 1.1 or 1.2 with HTTP GET
`SOAP1.1-GET`  | one-way SOAP 1.1 with HTTP GET
`SOAP1.2-GET`  | one-way SOAP 1.2 with HTTP GET
`HTTP`         | an alias for `POST` (same as `soapcpp2 -0`)
`POST`         | non-SOAP REST protocol with HTTP POST
`GET`          | non-SOAP REST protocol with HTTP GET
`PUT`          | non-SOAP REST protocol with HTTP PUT
`DELETE`       | non-SOAP REST protocol with HTTP DELETE

The `protocol` property is `SOAP` by default.  The default is changed with
<b>`soapcpp2 -1`</b> option <b>`-1`</b> to SOAP 1.1, <b>`soapcpp2 -2`</b>
option <b>`-2`</b> to SOAP 1.2, and non-SOAP REST with <b>`soapcpp2 -0`</b>
option <b>`-0`</b>

The `GET` protocols for SOAP and REST require that the service operations only
use primitive types with their input parameters, because these parameters are
encoded with the URL as URL query values.

To let directives take effect with service operations, you should bind the
service operations to the WSDL namespace of a service by using the namespace
prefix as part of the identifier name of the function that defines the service
operation:

~~~{.cpp}
    int prefix__func(arg1, arg2, ..., argn, result);
~~~

where `prefix` can now be used to let directives take effect on this service
operation.

🔝 [Back to table of contents](#)

### Service method directives                                        {#directives-2}

Service properties are applicable to a service and to all of its operations.
Service method directives are specifically applicable to a service operation.

A service method directive is of the form:

~~~{.cpp}
    //gsoap <prefix> service method-<property>: <method> <value>
~~~

where `<prefix>` is the XML namespace prefix of a service binding and
`<method>` is the unqualified name of a service operation.  The `<property>`
and `<value>` fields are one of the following:

method property             | value
--------------------------- | -----
`method-documentation`      | text describing the service operation
`method`                    | same as above, shorthand form
`method-action`             | `""` or URI SOAPAction HTTP header, or URL query string for REST protocols
`method-input-action`       | `""` or URI SOAPAction HTTP header of service request messages
`method-output-action`      | `""` or URI SOAPAction HTTP header of service response messages
`method-fault-action`       | `""` or URI SOAPAction HTTP header of service fault messages
`method-header-part`        | member name of the `SOAP_ENV__Header` struct used in SOAP Header
`method-input-header-part`  | member name of the `SOAP_ENV__Header` struct used in SOAP Headers of requests
`method-output-header-part` | member name of the `SOAP_ENV__Header` struct used in SOAP Headers of responses
`method-fault`              | type name of a struct or class member used in `SOAP_ENV__Details` struct
`method-mime-type`          | REST content type or SOAP MIME attachment content type(s)
`method-input-mime-type`    | REST content type or SOAP MIME attachment content type(s) of request message
`method-output-mime-type`   | REST content type or SOAP MIME attachment content type(s) of response message
`method-style`              | `document` or `rpc`
`method-encoding`           | `literal`, `encoded`, or a custom URI for encodingStyle of messages
`method-response-encoding`  | `literal`, `encoded`, or a custom URI for encodingStyle of response messages
`method-protocol`           | SOAP or REST, see \ref directives-1

The `method-header-part` properties can be repeated for a service operation to
declare multiple SOAP Header parts that the service operation requires.  You
can use `method-input-header-part` and `method-output-header-part` to
differentiate between request and response messages.

The `method-fault` property can be repeated for a service operation to declare
multiple faults that the service operation may return.

The `method-action` property serves two purposes:

-# To set the SOAPAction header for SOAP protocols, i.e. sets the
   definitions/binding/operation/SOAP:operation/\@soapAction.

-# To set the URL query string for endpoints with REST protocols, i.e. sets the
   definitions/binding/operation/HTTP:operation/\@location, which specifies
   a URL query string (starts with a `?`) to complete the service endpoint URL
   or extends the endpoint URL with a local path (starts with a `/`).

Use `method-input-action` and `method-output-action` to differentiate the
SOAPAction between SOAP request and response messages.

You can always override the port endpoint URL and action values at runtime in
the auto-generated `soap_call_prefix__func` service call (C/C++ client side)
and in the auto-generated C++ proxy class service calls.  A runtime NULL
endpoint URL and/or action uses the defaults set by these directives.

The `method-mime-type` property serves two purposes:

-# To set the type of MIME/MTOM attachments used with SOAP protocols.  Multiple
   attachment types can be declared for a SOAP service operation, i.e. adds
   definitions/binding/operation/input/MIME:multipartRelated/MIME:part/MIME:content/\@type
   for each type specified.

-# To set the MIME type of a REST operation.  This replaces XML declared in
   WSDL by definitions/binding/operation/(input|output)/MIME:mimeXml with
   MIME:content/\@type.  Use `application/x-www-form-urlencoded` with REST POST
   and PUT protocols to send encoded form data automatically instead of XML.
   Only primitive type values can be transmitted with form data, such as
   numbers and strings, i.e. only types that are legal to use as
   attributes members.

Use `method-input-mime-type` and `method-output-mime-type` to differentiate the
attachment types between request and response messages.

🔝 [Back to table of contents](#)

### Schema directives                                                {#directives-3}

A schema directive is of the form:

~~~{.cpp}
    //gsoap <prefix> schema <property>: <value>
~~~

where `<prefix>` is the XML namespace prefix of a schema.  The `<property>` and
`<value>` fields are one of the following:

property        | value
--------------- | -----
`namespace`     | URI of the XSD targetNamespace
`namespace2`    | alternate URI pattern for the XSD namespace (i.e. URI is also accepted by the XML parser)
`import`        | URI of an imported namespace, as an alternative or in addition to `namespace`, adds `xsd:import` to the generated WSDL and XSD files
`form`          | `unqualified` (default) or `qualified` local element and attribute form defaults
`elementForm`   | `unqualified` (default) or `qualified` local element form default
`attributeForm` | `unqualified` (default) or `qualified` local attribute form default
`typed`         | `no` (default) or `yes` for serializers to add `xsi:type` attributes to XML

The `namespace2` URI is a pattern with `*` matching any sequence of characters
and `-` matching any character.  This pattern instructs the XML parser and validator
to also accept the URI pattern as a valid namespace for the specified `<prefix>`.

The `typed` property is `no` by default and can be changed to `yes` with
<b>`soapcpp2 -t`</b> option <b>`-t`</b>.

🔝 [Back to table of contents](#)

### Schema type directives                                           {#directives-4}

A schema type directive is of the form:

~~~{.cpp}
    //gsoap <prefix> schema type-<property>: <name> <value>
    //gsoap <prefix> schema type-<property>: <name>::<member> <value>
~~~

where `<prefix>` is the XML namespace prefix of a schema and `<name>` is an
unqualified name of a C/C++ type, and the optional `<member>` is a class/struct
members or enum constant.

You can describe a type with one of the following:

type property        | value
-------------------- | -----
`type-documentation` | text describing the schema type
`type`               | an alias for the `type-documentation` property

For example, you can add a description to an enumeration:

~~~{.cpp}
    //gsoap ns schema type: Vowels The letters A, E, I, O, U, and sometimes Y
    //gsoap ns schema type: Vowels::Y A vowel, sometimes
    enum class ns__Vowels : char { A = 'A', E = 'E', I = 'I', O = 'O', U = 'U', Y = 'Y' };
~~~

This documented enumeration maps to a simpleType restriction of <i>`xsd:string`</i> in
the soapcpp2-generated schema:

<div class="alt">
~~~{.xml}
    <simpleType name="Vowels">
      <annotation>
        <documentation>The letters A, E, I, O, U, and sometimes Y</documentation>
      </annotation>
      <restriction base="xsd:string">
        <enumeration value="A"/>
        <enumeration value="E"/>
        <enumeration value="I"/>
        <enumeration value="O"/>
        <enumeration value="U"/>
        <enumeration value="Y">
          <annotation>
            <documentation>A vowel, sometimes</documentation>
          </annotation>
        <enumeration/>
      </restriction>
    </simpleType>
~~~
</div>

🔝 [Back to table of contents](#)

### Example  {#example12}

The use of directives is best illustrated with an example. The example uses a
hypothetical stock quote service and exchange rate service, actual services 
such as these are available for free on the web.

~~~{.cpp}
    //gsoap ns1 service namespace:  urn:GetQuote 
    int ns1__getQuote(char *symbol, float &result); 
     
    //gsoap ns2 service namespace:  urn:CurrencyExchange 
    int ns2__getRate(char *country1, char *country2, float &result); 
     
    //gsoap ns3 service name:       quotex 
    //gsoap ns3 service style:      rpc 
    //gsoap ns3 service encoding:   encoded 
    //gsoap ns3 service port:       http://www.mydomain.com/quotex.cgi 
    //gsoap ns3 service namespace:  urn:quotex 
    int ns3__getQuote(char *symbol, char *country, float &result);
~~~

The <i>`quotex.h`</i> example is a new Web Service created by combining two existing Web Services:
a Stock Quote service and a Currency Exchange service.

The namespace prefix `ns3` is used for the new `quotex` Web Service with namespace URI <i>`urn:quotex`</i>,
service name `quotex`, and endpoint port <i>`http://www.mydomain.com/quotex.cgi`</i>.

Since the new Web Service invokes the `ns1__getQuote` and `ns2__getRate` service operations,
the service namespaces and other details such as style and encoding of these methods are given by directives.
After invoking the soapcpp2 tool on the <i>`quotex.h`</i> header file:

     soapcpp2 quotex.h

the WSDL of the new `quotex` Web Service is saved as <i>`quotex.wsdl`</i>.
Since the service name, endpoint port, and namespace URI
were provided in the header file, the generated WSDL file can be published
together with the compiled Web Service installed as a CGI application.

The namespace mapping table for the <i>`quotex.cpp`</i> Web Service implementation
is saved as <i>`quotex.nsmap`</i>.  This file can be directly included in
<i>`quotex.cpp`</i> instead of specified by hand in the source of
<i>`quotex.cpp`</i>.

🔝 [Back to table of contents](#)

## Transient data types        {#transient}

There are situations when certain types have to be used in an interface header
file, but the types are not serializable and therefore have to be "invisible"
to the soapcpp2 tool.  These types are called transient.

This feature is useful when a library type is used in the interface header file
as part of a struct or class, for example the `FILE` type or `std::ostream`
that are clearly not serializable.  If these types are introduced in an
interface header file then these types must be declared transient.  Otherwise,
soapcpp2 will throw an error.

To declare a transient type use `extern`.  For example:

~~~{.cpp}
    extern class FILE;
~~~

Even though `FILE` is not a class, this declaration merely introduces the
`FILE` type name without specifying its details.

In C we can still use the class keyword, because no code is generated by
soapcpp2 for this transient type.

We can then use this type elsewhere, for example:

~~~{.cpp}
    struct ns__record
    {
        FILE *fd;
        const char *name;
    };
~~~

Only the `name` member is serialized of `ns__record`.

Another example:

~~~{.cpp}
    extern class std::ostream; // std::ostream can't be serialized, but need to be declared
    class ns__myClass 
    { public:
        virtual void print(std::ostream &s) const; // need ostream here 
        ... //
    };
~~~

In other cases we do want to declare a type that soapcpp2 should copy into <i>`soapStub.h`</i> for the application source code, but which should not be serializable.  We can use `extern` for this or put the declaration in `[` and `]` brackets:

~~~{.cpp}
    [ 
      class ns__myBase // base class need not be serializable
      {
        ... // members
      }; 
    ] 
    class ns__myDerived : ns__myBase 
    {
      ... // members
    }; 
~~~

We can use `[` and `]` brackets for parts of the code, for example to make several class members transient:

~~~{.cpp}
    [ typedef int transientInt; ] 
    class ns__myClass 
    {
      int a; // will be serialized 
      [ 
      int b; // transient member 
      char s[256]; // transient member 
      ]  
      extern float d; // transient type float and member
      char *t; // will be serialized 
      transientInt *n; // transient type int and member 
      [ 
      virtual void method(char buf[1024]); // does not create a char[1024] serializer 
      ]  
    };
~~~

In this example, `ns__myClass` has three transient members:
`b`, `s`, and `n` which will not be serialized. Member
`n` is transient because the type is declared within a transient block.
Pointers, references, and arrays of transient types are transient.  The single
class method is encapsulated within `[` and `]` to prevent soapcpp2 from
creating serializers for the `char[1024]` type.

We also use `[` and `]` brackets to introduce a transient integer type
`transientInt` using a typedef.  We cannot use `extern` with `typedef` because
this construct is reserved for custom serializers.

🔝 [Back to table of contents](#)

## Serialization "as is" of volatile data types        {#volatile}

Types declared `volatile` in an interface header file are serializable but not
copied by soapcpp2 to <i>`soapStub.h`</i>.  These types are typically library
types that are included in the source code and should not be redefined in the
application source code.  We want to serialize these library types "as is"
without redefining them.

Consider for example `struct tm`, declared in <i>`time.h`</i>. The structure may actually vary between platforms, but the `tm` structure includes at least the following members as declared in the following interface header file for soapcpp2:

~~~{.cpp}
    #include <time.h>
    volatile struct tm 
    {
        int tm_sec;         /* seconds (0 - 60) */ 
        int tm_min;         /* minutes (0 - 59) */ 
        int tm_hour;        /* hours (0 - 23) */ 
        int tm_mday;        /* day of month (1 - 31) */ 
        int tm_mon;         /* month of year (0 - 11) */ 
        int tm_year;        /* year - 1900 */ 
        int tm_wday;        /* day of week (Sunday = 0) */ 
        int tm_yday;        /* day of year (0 - 365) */ 
        int tm_isdst;       /* is summer time in effect? */ 
        char *tm_zone;        /* abbreviation of timezone name */ 
        long tm_gmtoff;      /* offset from UTC in seconds */ 
    };
~~~

By declaring struct `tm` volatile, soapcpp2 does not redefine it in the output source code.  The `#include <time.h>` is copied to the source code output.
We can now serialize the `tm` structure. The following example serializes the local time stored in a `tm` structure to stdout:

~~~{.cpp}
    struct soap *soap = soap_new(); 
    time_t T = time(NULL); 
    struct tm *t = localtime(&T); 
    soap_write_tm(soap, t); 
    soap_destroy(soap); 
    soap_end(soap); 
    soap_free(soap);
~~~

It is also possible to serialize the `tm` members as XML attributes by declaring
the members as attributes with the `@` qualifier, see Section \ref attributes .

If you want to produce a schema file, say <i>`time.xsd`</i>, that defines an XML
schema and namespace for the `tm` struct, you can add a `typedef`
declaration to the header file:

~~~{.cpp}
    #include <time.h>
    volatile struct tm
    {
        ... // see above
    };
    typedef struct tm time__struct_tm;
~~~

or simply use colon notation since we keep the `tm` name:

~~~{.cpp}
    #include <time.h>
    volatile struct time:tm 
    {
        int tm_sec;         /* seconds (0 - 60) */ 
        int tm_min;         /* minutes (0 - 59) */ 
        int tm_hour;        /* hours (0 - 23) */ 
        int tm_mday;        /* day of month (1 - 31) */ 
        int tm_mon;         /* month of year (0 - 11) */ 
        int tm_year;        /* year - 1900 */ 
        int tm_wday;        /* day of week (Sunday = 0) */ 
        int tm_yday;        /* day of year (0 - 365) */ 
        int tm_isdst;       /* is summer time in effect? */ 
        char *tm_zone;      /* abbreviation of timezone name */ 
        long tm_gmtoff;     /* offset from UTC in seconds */ 
    };
~~~

We used the typedef name `time__struct_tm` rather than `time__tm`, because a schema name clash will occur for the first example above that has `tm` already declared as serializable type in the schema of the generated WSDL.  The second example with the colon notation avoids this altogether.

🔝 [Back to table of contents](#)

## How to declare custom serializers and deserializers        {#extern}

You can implement your own custom serializers for data types.
A custom serializer is declared in an interface header file for soapcpp2 using the pair of keywords `extern typedef`.
For example:

~~~{.cpp}
    extern typedef char *MyData; 
    struct Sample 
    {
        MyData s; // use custom serializer for this member 
        char *t;  // use auto-generated serializer
    };
~~~

Then provide the following functions for each `extern typedef` declares type `T`:

~~~{.cpp}
    int soap_serialize_T(struct soap *soap, const T *a);
    void soap_default_T(struct soap *soap, T *a);
    int soap_out_T(struct soap *soap, const char *tag, int id, const T *a, const char *type);
    T *soap_in_T(struct soap *soap, const char *tag, T *a, const char *type);
~~~

The function prototypes of these functions can be found in the soapcpp2-generated <i>`soapH.h`</i> file.

For example, the serialization of `MyData` can be done with the following code:

~~~{.cpp}
    int soap_serialize_MyData(struct soap *soap, MyData const*a) 
    {
      // no need to mark this node (for multi-ref and cycle detection) 
      return SOAP_OK;
    }
    void soap_default_MyData(struct soap *soap, MyData *a) 
    {
      *a = NULL;
    } 
    int soap_out_MyData(struct soap *soap, const char *tag, int id, MyData const*a, const char *type) 
    {
      if (soap_element_begin_out(soap, tag, id, type) // print XML beginning tag 
       || soap_send(soap, *a) // just print the string (no XML conversion) 
       || soap_element_end_out(soap, tag)) // print XML ending tag 
        return soap->error; 
      return SOAP_OK; 
    } 
    MyData **soap_in_MyData(struct soap *soap, const char *tag, MyData *a, const char *type) 
    {
      if (soap_element_begin_in(soap, tag)) 
        return NULL; 
      if (!a) 
        a = (MyData*)soap_malloc(soap, sizeof(MyData)); 
      if (soap->null) 
        *a = NULL; // xsi:nil element 
      if (*soap->type && soap_match_tag(soap, soap->type, type)) 
      {
        soap->error = SOAP_TYPE; 
        return NULL; // type mismatch 
      } 
      if (*soap->href) 
        a = (MyData**)soap_id_forward(soap, soap->href, a, 0, SOAP_TYPE_MyData, 0, sizeof(MyData), 0, NULL, NULL) 
      else if (soap->body) 
      {
        char *s = soap_value(soap); // fill buffer 
        *a = (char*)soap_malloc(soap, strlen(s)+1); 
        strcpy(*a, s); 
      } 
      if (soap->body && soap_element_end_in(soap, tag)) 
        return NULL; 
      return a; 
    }
~~~

More information on custom serialization is available in the gSOAP source code
package in the <i>`gsoap/custom`</i> directory, where you can also find several
custom serializers to use with your projects.

🔝 [Back to table of contents](#)

## Function callbacks for customized I/O and HTTP handling        {#callback}

The following list of functions can be used for customized HTTP handling and I/O.

See also API documentation Module \ref group_callbacks.

To reset the callback functions to the internal functions of the engine, use `::soap_done` followed by `::soap_init`.  This re-initializes the `::soap` context, removes all plugins, and resets function callbacks.

### fpost

`int (soap::fpost)(struct soap *soap, const char *endpoint, const char *host, int port, const char *path, const char *action, ULONG64 count)` 

This callback is called at the client side by the engine to send HTTP headers to the connected server.  The parameters `host`, `port`, and `path` were micro-parsed from the `endpoint` prior to passing them to this callback.  Parameter `action` is the SOAP Action header.  Parameter `count` is the length of the HTTP body with the message or 0 when HTTP chunking is used.  This callback sends the headers with POST by default, or when `::soap::status` == `#SOAP_POST` or `::soap::status` == `#SOAP_POST_FILE`. Alternatively, sends the HTTP headers with GET when `::soap::status` == `#SOAP_GET`, PATCH when `::soap::status` == `#SOAP_PATCH`, PUT when `::soap::status` == `#SOAP_PUT`, DELETE when `::soap::status` == `#SOAP_DEL`, CONNECT when `::soap::status` == `#SOAP_CONNECT`, HEAD when `::soap::status` == `#SOAP_HEAD` or OPTIONS when `::soap::status` == `#SOAP_OPTIONS`.  Extra HTTP headers are added when `::soap::http_extra_header` is set to one or more header lines separated by CRLF.  When redefining this callback, use function `::soap_send` to write the header contents.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.  The built-in function assigned to `::soap:fpost` is `http_post`.

### fresponse

`int (soap::fresponse)(struct soap *soap, int soap_error_code, ULONG64 count)` 

This callback is called at the server side by the engine to send the HTTP headers to the connected client.  The parameter `status` should be an HTTP status error code or `#SOAP_OK` (200 OK) or `#SOAP_HTML` or `#SOAP_FILE`.  Using `#SOAP_HTML` sets the content-type header to `text/html; charset=utf-8`.  Using `#SOAP_FILE` sets the content-type header to the value of `::soap::http_content`.  Extra HTTP headers are added when `::soap::http_extra_header` is set to one or more header lines separated by CRLF.  When redefining this callback, use function `::soap_send` to write the header contents.  Returns `#SOAP_OK` or a `::soap_status` error code.  The built-in function assigned to `::soap::fresponse` is `http_response`.

### fposthdr
  
`int (soap::fposthdr)(struct soap *soap, const char *key, const char *val)` 

This callback is called by `::soap::fpost` and `::soap::fresponse` to send an HTTP header with a key and an optional value.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.  The built-in function assigned to `::soap::fposthdr` is `http_post_header`.

### fparse

`int (soap::fparse)(struct soap *soap)` 

This callback is called by the engine (as a client or server) to read and parse HTTP headers or MIME headers.  When redefined, this function should at read or skip the entire HTTP header to reach the message body.  Function `::soap_getline` is used by this callback to read each header line into an internal buffer `::soap::msgbuf` with `::soap_getline(soap, soap->msgbuf, sizeof(soap->msgbuf))`.  Returns `#SOAP_OK`, or a gSOAP error code.  The built-in function assigned to `::soap::fparse` is `http_parse`.

### fparsehdr

`int (soap::fparsehdr)(struct soap *soap, const char *key, const char *val)` 

This callback is called by `::soap::fparse`, consumes an HTTP header that is split in a key-value pair and updates the `::soap` context state accordingly.  The context is updated with the HTTP header information received, but HTTP headers are not literally retained by the engine.  Returns `#SOAP_OK` or `#SOAP_STOP` to prevent further reading of the HTTP body, or a `::soap_status` (int) error code.  The built-in function assigned to `::soap::fparsehdr` is `http_parse_header`.

### fget

`int (soap::fget)(struct soap *soap)` 

This callback is called by the service dispatcher when an HTTP GET request is pending.  Redefine this callback to respond to HTTP GET requests with content, see the `::http_get` HTTP GET plugin for more details.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.  The built-in function assigned to `::soap::fget` is the internal static function `http_get` that returns the `#SOAP_GET_METHOD` error.

### fput

`int (soap::fput)(struct soap *soap)` 

This callback is called by the service dispatcher when an HTTP PUT request is pending.  Redefine this callback to respond to HTTP PUT requests, see the `::http_post` HTTP POST plugin for more details.  Returns `#SOAP_OK` or a `::soap_status` error code.  The built-in function assigned to `::soap::fput` is the internal static function `http_put` that returns the `#SOAP_PUT_METHOD` error.

### fpatch

`int (soap::fpatch)(struct soap *soap)` 

This callback is called by the service dispatcher when an HTTP PATCH request is pending.  Redefine this callback to respond to HTTP PATCH requests, see the `::http_post` HTTP POST plugin for more details.  Returns `#SOAP_OK` or a `::soap_status` error code.  The built-in function assigned to `::soap::fpatch` is the internal static function `http_patch` that returns the `#SOAP_PATCH_METHOD` error.

### fdel

`int (soap::fdel)(struct soap *soap)` 

This callback is called by the service dispatcher when an HTTP DELETE request is pending.  Redefine this callback to respond to HTTP DELETE requests, see the `::http_post` HTTP POST plugin for more details.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.  The built-in function assigned to `::soap::fdel` is the internal static function `http_del` that returns the `#SOAP_DEL_METHOD` error.

### fopt

`int (soap::fopt)(struct soap *soap)` 

Called by the service dispatcher when an HTTP OPTION request is pending.  Redefine this callback to respond to HTTP OPTION requests, see the `::http_post` HTTP POST plugin for more details.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.  The built-in function assigned to `::soap::fopt` is the internal static function `http_200` that returns HTTP 200 OK.

### fhead

`int (soap::fhead)(struct soap *soap)` 

This callback is called by the service dispatcher when an HTTP HEAD request is pending.  Redefine this callback to respond to HTTP HEAD requests more specifically.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.  The built-in function assigned to `::soap::fhead` is the internal static function `http_200` that returns HTTP 200 OK.

### fform

`int (soap::fform)(struct soap *soap)` 

This callback is called by the HTTP FORM handler plugin to parse HTML forms received with HTTP POST and PUT requests, see the `;:http_form` HTTP FORM plugin for more details.  The HTTP body with the form data should be parsed by this callback, otherwise HTTP keep-alive messages will end up out of sync as a result of the current position not being advanced to the end of the HTTP body.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.  No built-in function is assigned to `::soap::fform`.

### fheader

`int (soap::fheader)(struct soap *soap)` 

This callback is called immediately after parsing a SOAP Header into the `::soap::header` structure.  The SOAP Header structure `::soap::header` can be inspected by this function and verified or rejected before the rest of the message with the SOAP Body is consumed.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.  No built-in function is assigned to `::soap::fheader`.

### fignore

`int (soap::fignore)(struct soap *soap, const char *tag)` 

This callback is called when an unrecognized XML element was encountered on the input that could be ignored depending on some specified logic.  The `tag` parameter is the offending XML element tag name string.  The callback should return `#SOAP_OK` to ignore the element or return an `::soap_status` error code such as `#SOAP_TAG_MISMATCH` to trigger a validation error.  This callback also overrides `mustUnderstand` attributes on unrecognized SOAP Header elements that normally raise faults.  It is strongly recommended that the callback returns `#SOAP_MUSTUNDERSTAND` when `::soap::mustUnderstand` != `0`.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.  No built-in function is assigned to `::soap::fignore`.

### fsvalidate

`int (soap::fsvalidate)(struct soap *soap, const char *pattern, const char *string)` 

This callback is called to validate a string against an XML regex pattern.  Patterns use XML schema regex syntax.  This callback allows user-defined pattern validation that is normally disabled.  Returns `#SOAP_OK` when the string matches the pattern or `#SOAP_TYPE` when the string does not match.  No built-in function is assigned to `::soap::fsvalidate`.

### fwvalidate

`int (soap::fwvalidate)(struct soap *soap, const char *pattern, const wchar_t *string)` 
This callback is called to validate a wide string against an XML regex pattern.  Patterns use XML schema regex syntax.  This callback allows user-defined pattern validation that is normally disabled.  Returns `#SOAP_OK` when the string matches the pattern or `#SOAP_TYPE` when the string does not match.  No built-in function is assigned to `::soap::fwvalidate`.

### fseterror

`void (soap::fseterror)(struct soap *soap, const char **code, const char **string)` 

This callback is called by the engine when an error is raised to allow inspection or overriding of the fault code or fault string messages before the error is reported or transmitted.  No built-in function is assigned to `::soap::fseterror`.

### fopen

`SOAP_SOCKET (soap::fopen)(struct soap *soap, const char *endpoint, const char *host, int port)` 

This callback is called by the engine at the client-side by `::soap_connect` or `::soap_connect_command` to open a TCP or UDP connection to a server specified at an endpoint.  Parameters `host` and `port` are micro-parsed from `endpoint` before being passed to `::soap::fopen`.  Returns a valid socket or `#SOAP_INVALID_SOCKET` with a `::soap::error` set to a `::soap_status` (int) error code and `::soap::errnum` set to `errno` of the connection failure.  The built-in function assigned to `::soap::fopen` is `tcp_connect`.

### faccept

`SOAP_SOCKET (soap::faccept)(struct soap *soap, SOAP_SOCKET s, struct sockaddr *a, int *n)` 

This callback is called by `::soap_accept` (or the C++ service class `accept` method) to wait for and accept a socket connection requested by a client.  Returns a valid socket or `#SOAP_INVALID_SOCKET` when an error occurred and sets `::soap::error` to a `::soap_status` value.  The built-in function assigned to `::soap::faccept` is `tcp_accept`.

### fclose

`int (soap::fclose)(struct soap *soap)` 

This callback is called by the engine at the client-side to close the current socket connection before a new socket connection is established.  This callback may be called multiple times (e.g. by the engine and by plugins) to close the same socket `::soap::socket`.  Checks internally if `::soap::socket` == `#SOAP_INVALID_SOCKET` before closing, which means that the socket was already closed.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.  The built-in function assigned to `::soap::fclose` is `tcp_disconnect`.

### fresolve

`int (soap::fresolve)(struct soap *soap, const char *addr, struct in_addr *inaddr)` 

This callback is called by `::soap_bind` (or the C++ service class `bind` method) at the server-side and by `::soap_connect` or `::soap_connect_command` at the client-side with a host `name` parameter to resolve to address `inaddr` by address translation.  When successful sets parameter `inaddr` and returns `#SOAP_OK` or a `::soap_status` (int) error code.  The built-in function assigned to `::soap::fresolve` is `tcp_gethost`. 

### fconnect

`int (soap::fconnect)(struct soap *soap, const char *endpoint, const char *host, int port)` 

This callback is called by the engine to optionally override client-side connecting.  The parameters `host` and `port` were micro-parsed from the `endpoint` prior to passing them to this callback.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.  No built-in function is assigned to `::soap::fconnect`.

### fdisconnect

`int (soap::fdisconnect)(struct soap *soap)`

This callback is called by the engine `::soap_closesock` before the `::soap::fclose` callback is called to shutdown/disconnect.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.  No built-in function is assigned to `::soap::fdisconnect`.

### fclosesocket

`int (soap::fclosesocket)(struct soap *soap, SOAP_SOCKET sock)`

This callback is called to close a socket by the engine.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.  The built-in function assigned to `::soap::fclosesocket` is `tcp_closesocket`.

### fshutdownsocket

`int (soap::fshutdownsocket)(struct soap *soap, SOAP_SOCKET sock, int how)`

This callback is called to shut down a socket by the engine.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.  The built-in function assigned to `::soap::fshutdownsocket` is `tcp_shutdownsocket`.

### fpoll

`int (soap::fpoll)(struct soap *soap)` 

This callback is called by the engine to wait for activity on the `::soap::socket` or `::soap::master` socket using `poll` or `select`.  Times out when `::soap::send_timeout` or `::soap::recv_timeout` are nonzero.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.  The built-in function assigned to `::soap::fpoll` is `soap_poll`. 

### frecv

`size_t (soap::frecv)(struct soap *soap, char *buf, size_t len)` 

This callback is called by the engine to receive (or read) data into a specified buffer `buf` and `len`.  The source for the data to read by this callback is `::soap::is` when non-NULL, `::soap::socket` when valid, or `::soap::recvfd`.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.  The built-in function assigned to `::soap::frecv` is `frecv`.

### fsend

`int (soap::fsend)(struct soap *soap, const char *buf, size_t len)` 

This callback is called by the engine to send (or write) data specified by `data` bytes of length `len`.  The sink for the data to be sent to is typically `::soap::socket`, `::soap::sendfd` or `::soap::os`.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.  The built-in function assigned to `::soap::fsend` is `fsend`.

### fserverloop

`int (soap::fserveloop)(struct soap *soap)` 

This callback is called after each successful completion of a server operation in the server loop.  Executes immediately after sending the response to a client and before the next keep-alive server loop iteration when enabled with `#SOAP_IO_KEEPALIVE`.  This callback can be used to reclaim resources in the keep-alive server loop, for example managed memory can be reclaimed by calling `::soap_destroy` and `::soap_end` in that order and all deserialized and other dynamically-allocated data managed by the context will be deallocated.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.  No built-in function is assigned to `::soap::fserveloop`.

### fmalloc

`void (soap::fmalloc)(struct soap *soap, size_t size)` 

This callback can be used to override memory allocation and management done by `::soap_malloc` in C.  Memory allocated via this callback will not be managed and not be automatically released by the engine.  Instead, the application using this callback should release allocated memory.  All allocations done by `::soap_malloc` are replaced with a call to `::soap::fmalloc`.  However, no other allocations, such as `::soap_new` and `soap_new_T` for C++ classes `T` are affected, because objects are allocated differently.  This callback is therefore not useful for C++ applications.  Returns a pointer to dynamically allocated memory or NULL on failure to allocate.  No built-in function is assigned to `::soap::fmalloc`.

@warning Deprecated since 2.8.72.  Define `#SOAP_MALLOC` and `#SOAP_FREE` instead.

### user variable

A `void* ::soap::user` variable is available to pass user-defined data to the callbacks.

### Examples

The following example uses I/O callbacks for customized serialization of data
into a fixed-size buffer and deserialization back into a data structure:

~~~{.cpp}
    char buf[10000]; // XML buffer 
    int len1 = 0;    // #chars written 
    int len2 = 0;    // #chars read 

    // mysend: put XML in buf[] 
    int mysend(struct soap *soap, const char *s, size_t n) 
    {
      if (len1 + n > sizeof(buf)) 
        return SOAP_EOF; 
      strcpy(buf + len1, s); 
      len1 += n; 
      return SOAP_OK; 
    } 

    // myrecv: get XML from buf[] 
    size_t myrecv(struct soap *soap, char *s, size_t n) 
    {
      if (len2 + n > len1) 
        n = len1 - len2; 
      strncpy(s, buf + len2, n); 
      len2 += n; 
      return n; 
    } 

    int main() 
    {
      struct soap soap; 
      struct ns__person p; 
      soap_init1(&soap, SOAP_XML_TREE); 
      len1 = len2 = 0;     // reset buffer pointers 
      p.name = "John Doe"; 
      p.age = 25; 
      soap.fsend = mysend; // assign callback 
      soap.frecv = myrecv; // assign callback 
      if (soap_write_ns__person(&soap, &p))
      {
        soap_print_fault(&soap, stdout); 
        exit(EXIT_FAILURE); 
      } 
      if (soap_read_ns__Person(&soap, &p))
      {
        soap_print_fault(&soap, stdout); 
        exit(EXIT_FAILURE); 
      } 
      soap_destroy(&soap); 
      soap_end(&soap); 
      soap_done(&soap);
    }
~~~

A fixed-size buffer to store the outbound message sent is not flexible to handle large content. To store the message in an expanding buffer:

~~~{.cpp}
    struct buffer 
    {
      size_t len; 
      size_t max; 
      char *buf; 
    }; 

    int main() 
    {
      struct soap soap; 
      struct ns__person p; 
      struct buffer *h = malloc(sizeof(struct buffer)); 
      h->len = 0; 
      h->max = 0; 
      h->buf = NULL; 
      soap_init1(&soap, SOAP_XML_TREE); 
      soap.user = (void*)h; // pass buffer as a handle to the callback 
      soap.fsend = mysend;  // assign callback 
      if (soap_write_ns__person(&soap, &p))
      {
        soap_print_fault(&soap, stdout); 
        exit(EXIT_FAILURE); 
      } 
      if (h->len) 
      {
        ... // use h->buf[0..h->len-1] content 
        // then cleanup: 
        h->len = 0; 
        h->max = 0; 
        free(h->buf); 
        h->buf = NULL; 
      } 
      soap_destroy(&soap); 
      soap_end(&soap); 
      soap_done(&soap);
    } 

    int mysend(struct soap *soap, const char *s, size_t n) 
    {
      struct buffer *h = (struct buffer*)soap->user; // get buffer through handle 
      int m = h->max, k = h->len + n; 
      // need to increase space? 
      if (m == 0) 
        m = 1024; 
      else 
        while (k >= m) 
          m *= 2; 
      if (m != h->max) 
      {
        char *buf = malloc(m); 
        memcpy(buf, h->buf, h->len); 
        h->max = m; 
        if (h->buf) 
          free(h->buf); 
        h->buf = buf; 
      } 
      memcpy(h->buf + h->len, s, n); 
      h->len += n; 
      return SOAP_OK; 
    }
~~~

The following example illustrates customized I/O and HTTP header handling.
The XML message is saved to a file to demonstrate I/O and HTTP callbacks. The client
proxy then reads the file contents as the service response. To perform this
trick, the service response has exactly the same structure as the request. This
is declared by the `struct ns__test` output parameter part of the service
operation declaration.  This struct resembles the service request (see the
generated <i>`soapStub.h`</i> file generated by soapcpp2 from the interface
header file input).

The interface header file for soapcpp2 is:

~~~{.cpp}
    //gsoap ns service name: callback 
    //gsoap ns service namespace: urn:callback 
    struct ns__person 
    {
        char *name; 
        int age; 
    }; 
    int ns__test(struct ns__person in, struct ns__test &out);
~~~

The client program is:

~~~{.cpp}
    #include "soapH.h" 
    #include "ns.nsmap" 

    SOAP_SOCKET myopen(struct soap *soap, const char *endpoint, const char *host, int port) 
    {
      if (strncmp(endpoint, "file:", 5)) 
      {
        printf("File name expected\n"); 
        return SOAP_INVALID_SOCKET; 
      } 
      if ((soap->sendfd = soap->recvfd = open(host, O_RDWR|O_CREAT, S_IWUSR|S_IRUSR)) < 0) 
        return SOAP_INVALID_SOCKET; 
      return soap->sendfd; 
    } 

    void myclose(struct soap *soap) 
    {
      if (soap->sendfd > 2)  // still open? 
        close(soap->sendfd); // then close it 
      soap->recvfd = 0;      // set back to stdin 
      soap->sendfd = 1;      // set back to stdout 
    } 

    int mypost(struct soap *soap, const char *endpoint, const char *host, const char *path, const char *action, ULONG64 count) 
    {
      return soap_send(soap, "Custom-generated file\n"); // writes to soap->sendfd 
    } 

    int myparse(struct soap *soap) 
    {
      char buf[256]; 
      if (lseek(soap->recvfd, 0, SEEK_SET) < 0 || soap_getline(soap, buf, 256)) // go to begin and skip custom header 
        return SOAP_EOF; 
      return SOAP_OK; 
    } 

    int main() 
    {
      struct soap soap; 
      struct ns__test r; 
      struct ns__person p; 
      soap_init(&soap);
      p.name = "John Doe"; 
      p.age = 99; 
      soap.fopen = myopen;   // use custom open 
      soap.fpost = mypost;   // use custom post 
      soap.fparse = myparse; // use custom response parser 
      soap.fclose = myclose; // use custom close 
      soap_call_ns__test(&soap, "file://test.xml", "", p, r); 
      if (soap.error) 
      {
        soap_print_fault(&soap, stdout); 
        exit(EXIT_FAILURE); 
      } 
      soap_destroy(&soap);
      soap_end(&soap); 
      soap_done(&soap);
    }
~~~

SOAP 1.1/1.2 RPC encoding specifies that SOAP/XML elements may be ignored when present in a SOAP payload on the receiving side.
However, document/literal style messaging validates XML messages and extra elements cannot just be ignored as such.
With SOAP document/literal style it is recommended to enable the `#SOAP_XML_STRICT` mode flag.
With SOAP RPC encoding, the engine ignores XML elements that are unknown by default, unless the XML attribute <i>`mustUnderstand="true"`</i> is present in the XML element.
It may be undesirable for elements to be ignored when the outcome of the omission is uncertain.
The `::soap::fignore` callback can be set to a function that returns `#SOAP_OK` in case the element can be safely ignored, or
`#SOAP_MUSTUNDERSTAND` to throw an exception, or to perform some application-specific action.
For example, to throw an exception as soon as an unknown element is encountered on the input, use:

~~~{.cpp}
    int main()
    {
      struct soap *soap = soap_new();
      soap->fignore = myignore; 
      ... // soap_call_ns__webmethod(soap, ...) or soap_serve(soap) 
    }

    int myignore(struct soap *soap, const char *tag) 
    {
      return SOAP_MUSTUNDERSTAND; // never skip elements (secure) 
    } 
~~~

To selectively throw an exception when <i>`mustUnderstand="true"`</i> SOAP Header element is encountered or when an unknown element is encountered except for element <i>`ns:xyz`</i>, use:

~~~{.cpp}
    int main()
    {
      struct soap *soap = soap_new();
      soap->fignore = ignore; 
      ... // soap_call_ns__webmethod(soap, ...) or soap_serve(soap) 
    }

    int ignore(struct soap *soap, const char *tag) 
    {
      // do not ignore mustUnderstand="true" 
      if (!soap->mustUnderstand)
      {
        // tags <ns:someElement> can be safely ignored
        if (soap_match_tag(soap, tag, "ns:someElement") == SOAP_OK)
          return SOAP_OK;
      }
      return SOAP_MUSTUNDERSTAND; 
    }

    struct Namespace namespaces[] = 
    {
      { "SOAP-ENV", "http://schemas.xmlsoap.org/soap/envelope/" }, 
      { "SOAP-ENC", "http://schemas.xmlsoap.org/soap/encoding/" }, 
      { "xsi",      "http://www.w3.org/2001/XMLSchema-instance" }, 
      { "xsd",      "http://www.w3.org/2001/XMLSchema" }, 
      { "ns",       "some-URI"}, // binds "ns" namespace prefix to schema URI
      { NULL, NULL} 
    ];
~~~

Function `::soap_match_tag` compares two tags. The third parameter may be a
pattern where `*` is a wildcard and `-` is a single character wildcard. So for
example `soap_match_tag(soap, tag, "ns:*")` will match any element in namespace
`ns` or when no namespace prefix is present in the XML message.

The callback can also be used to keep track of unknown elements in an internal data structure such as a list:

~~~{.cpp}
    struct Unknown 
    {
      char *tag; 
      struct Unknown *next; 
    }; 
    int myignore(struct soap *soap, const char *tag) 
    {
      char *s = (char*)soap_malloc(soap, strlen(tag)+1); 
      struct Unknown *u = (struct Unknown*)soap_malloc(soap, sizeof(struct Unknown)); 
      if (s && u) 
      {
        strcpy(s, tag); 
        u->tag = s; 
        u->next = ulist; 
        ulist = u; 
      } 
    } 

    int main()
    {
      struct soap *soap; 
      struct Unknown *ulist = NULL; 
      soap_init(&soap); 
      soap.fignore = myignore; 
      ... // soap_call_ns__webmethod(soap, ...) or soap_serve(soap) 
      ... // print the list of unknown elements stored in ulist
      soap_destroy(&soap);
      soap_end(&soap);
      soap_done(&soap);
    }
~~~

🔝 [Back to table of contents](#)

## How to handle HTTP 307 temporary redirect {#http307}

The client-side handling of HTTP 307 code "Temporary Redirect" and any of the redirect codes 301, 302, and 303 are not automated. Client application developers may want to consider adding a few lines of code to support redirects. It was decided not to automatically support redirects for the following reasons:

*  Redirecting a secure HTTPS address to a non-secure HTTP address via 307 creates a security vulnerability.

*  Cyclic redirects must be detected (e.g. allowing only a limited number of redirect levels).

*  Redirecting HTTP POST will result in re-serialization and re-post of the entire SOAP request. The SOAP request message must be re-posted in its entirety when re-issuing the SOAP operation to a new address.

To implement client-side 307 redirect, add the following lines of code:

~~~{.cpp}
    const char *endpoint = NULL; // use default endpoint given in WSDL or specify one here
    int redirects = 10;          // max redirect count 
    while (redirects--) 
    {
      if (soap_call_ns1__myMethod(soap, endpoint, ...)) 
      {
        if ((soap->error >= 301 && soap->error <= 303) || soap->error == 307) 
        {
          endpoint = soap_strdup(soap, soap->endpoint); // endpoint from HTTP 301, 302, 303, 307 Location header 
        }
        else 
        {
          ... // handle error 
          break; 
        } 
      } 
      else 
      {
        break; // all OK now
      }
    }
~~~

🔝 [Back to table of contents](#)

## How to implement HTTP GET, PUT, and PATCH services {#get}

To implement HTTP GET request responses, define the `::soap::fget` callback function. The callback is required to produce a response to the request in textual form, such as a Web page or an XML or JSON response.

You can also use the `::http_get` plugin which essentially sets the `::soap::fget` callback and also keeps track of the number of HTTP GET and POST requests made at the server side.

The following example produces a Web page upon a HTTP GET request (e.g. from a browser):

~~~{.cpp}
    int main()
    {
      struct soap *soap = soap_new(); 
      soap->fget = http_get; 
      ... //
      soap_serve(soap); 
      ... //
    }

    int http_get(struct soap *soap) 
    {
      if (soap_response(soap, SOAP_HTML) // HTTP response header with text/html 
       || soap_send(soap, "<HTML>My Web server is operational.</HTML>")
       || soap_end_send(soap))
        return soap_closesock(soap);
      return soap_closesock(soap);
    }
~~~

The example below produces a WSDL file upon a HTTP GET with path `?wsdl`:

~~~{.cpp}
    int http_get(struct soap *soap) 
    {
      FILE *fd = NULL;
      char *s = strchr(soap->path, '?'); 
      if (!s || strcmp(s, "?wsdl")) 
        return SOAP_GET_METHOD; 
      fd = fopen("myservice.wsdl", "rb"); // open WSDL file to copy 
      if (!fd) 
        return 404; // return HTTP not found error 
      soap->http_content = "text/xml"; // HTTP header with text/xml content 
      if (soap_response(soap, SOAP_FILE) == SOAP_OK)
      {
        while (1)
        {
          size_t r = fread(soap->tmpbuf, 1, sizeof(soap->tmpbuf), fd); 
          if (!r || soap_send_raw(soap, soap->tmpbuf, r)) 
            break;
        } 
      }
      fclose(fd); 
      soap_end_send(soap);
      return soap_closesock(soap);
    }
~~~

This example shows how one predetermined file is served.  The <i>`gsoap/samples/webserver`</i> demonstrates how files should be served in general, by adding the necessary logic to add media types to HTTP headers and to restrict the selection of files that should be served.

@warning When serving files as responses to requests, we need to be vary careful, because we don't want requests to snoop around in directories and serve files that should be protected from public view.  Therefore, when adding logic to serve files, we must reject request that have `::soap::path` values with a `/` or a `\` (`::soap::path` is a string with the path part of the URL, starting with a `/`).  If these are allowed, then we must at least check for `..` in the path to avoid request from snooping around in higher directories all the way up to the root.

For a one-way SOAP/XML message, you can also return a SOAP/XML response:

~~~{.cpp}
    int http_get(struct soap *soap) 
    {
      soap_response(soap, SOAP_OK); 
      return soap_send_ns1__mySendMethodResponse(soap, "", NULL, ... params ...); 
    } 
~~~

where `ns1__mySendMethodResponse` is a one-way message declared in a interface header file as:

~~~{.cpp}
    int ns1__mySendMethodResponse(... params ..., void);
~~~

The generated <i>`soapClient.cpp`</i> includes the sending-side stub function.

The examples above are for HTTP GET.  To implement HTTP PUT and PATCH set the `::soap::fput` and `::soap::fpatch` callback functions or use the `::http_post` plugin which is more capable.

🔝 [Back to table of contents](#)

## TCP and HTTP keep-alive        {#keepalive}

To activate HTTP keep-alive, set the `#SOAP_IO_KEEPALIVE` flag for both input
and output modes, see Section \ref flags .  For example

~~~{.cpp}
    struct soap soap; 
    soap_init1(&soap, SOAP_IO_KEEPALIVE); 
~~~

When a client or a service communicates with another client or service that supports keep alive, the
`::soap::keep_alive` variable will be set to 1, otherwise it is reset to 0 indicating that the other party 
wants to close the connection.
The connection maybe terminated on either end before the communication completed, for example when the server keep-alive
connection has timed out.
This may generate a "Broken Pipe" signal on Unix/Linux platforms. This signal can be caught with a signal handler:

~~~{.cpp}
    signal(SIGPIPE, sigpipe_handle);
~~~

where, for example:

~~~{.cpp}
    void sigpipe_handle(int x) { }
~~~

Alternatively, broken pipes can be kept silent by setting:

~~~{.cpp}
    soap.socket_flags = MSG_NOSIGNAL;
~~~

This setting will not generate a SIGPIPE but read/write operations return `#SOAP_EOF` instead.
Note that Windows does not support signals and lack the `MSG_NOSIGNAL` flag.

If the client does not close the connection, the server will wait forever when
no `::soap::recv_timeout` is specified, so be careful to set timeouts, See Section \ref timeout.  In addition, other clients will be denied
service as long as a client keeps the connection to the server open.  To
prevent this from happening, the service should be multi-threaded such that
each thread handles the client connection:

~~~{.cpp}
    int main(int argc, char **argv) 
    {
      struct soap soap, *tsoap; 
      THREAD_TYPE tid; 
      int m, s; 
      soap_init2(&soap, SOAP_IO_KEEPALIVE, SOAP_IO_KEEPALIVE); 
      soap.max_keep_alive = 100; // at most 100 calls per keep-alive session 
      soap.accept_timeout = 600; // optional: let server time out after ten minutes of inactivity 
      m = soap_bind(&soap, NULL, 18000, BACKLOG); // use port 18000 on the current machine 
      if (!soap_valid_socket(m)) 
      {
        soap_print_fault(&soap, stderr); 
        exit(EXIT_FAILURE); 
      } 
      fprintf(stderr, "Socket connection successful %d\n", m); 
      for (count = 0; count >= 0; count++) 
      {
        soap.socket_flags = MSG_NOSIGNAL; // use this 
        soap.accept_flags = SO_NOSIGPIPE; // or this to prevent SIGPIPE 
        s = soap_accept(&soap); 
        if (soap_valid_socket(s)) 
        {
          fprintf(stderr, "Accept socket %d connection from IP %d.%d.%d.%d\n", s, (int)(soap.ip>>24)&0xFF, (int)(soap.ip>>16)&0xFF, (int)(soap.ip>>8)&0xFF, (int)soap.ip&0xFF); 
          tsoap = soap_copy(&soap); 
          if (!tsoap)
            soap_force_closesock(&soap);
          else
            while (THREAD_CREATE(&tid, (void*(*)(void*))process_request, (void*)tsoap))
              sleep(1); // failed, try again
        }
        else if (soap.errnum) // accept failed, try again after 1 second
        {
          soap_print_fault(&soap, stderr); 
          sleep(1);
        }
        else
        {
          fprintf(stderr, "Server timed out\n");
          break; 
        } 
      } 
      return 0; 
    } 

    void *process_request(void *tsoap) 
    {
      struct *soap = (struct soap*)tsoap;
      THREAD_DETACH(THREAD_ID); 
      ((struct soap*)soap)->recv_timeout = 60; // Timeout after 1 minute stall on recv 
      ((struct soap*)soap)->send_timeout = 10; // Timeout after 10 second stall on send 
      soap_serve(soap); 
      soap_destroy(soap); 
      soap_end(soap); 
      soap_free(soap); 
      return NULL; 
    }
~~~

A client call will automatically attempt to re-establish a connection to
a server when the server has terminated the connection for any reason.  This
way, a sequence of calls can be made to the server while keeping the connection
open.  Client stub functions poll the server to check if the connection is still
open.  When the connection was terminated by the server, the client will
automatically reconnect.  

A client may clear the `#SOAP_IO_KEEPALIVE` flag just before the last call to a
server to let the server know it wants to close the connection after this last
call. This will close the socket after the call and also informs the server to
gracefully close the connection.

The client-side can also set the TCP keep-alive socket properties, using the
`::soap::tcp_keep_alive` flag (set to 1 to enable), `::soap::tcp_keep_idle` to
set the `TCP_KEEPIDLE` value, `::soap::tcp_keep_intvl` to set the
`TCP_KEEPINTVL` value, and `::soap::tcp_keep_cnt` to set the `TCP_KEEPCNT`
value.

If a client is in the middle of soap call that might take a long time and the
server goes away/down the caller does not get any feedback until the
`::soap::recv_timeout` is reached. Enabling TCP keep alive on systems that
support it allows for a faster connection teardown detection for applications
that need it.

🔝 [Back to table of contents](#)

## HTTP chunked transfer encoding        {#chunked}

Outbound HTTP messages are not chunked unless the
`#SOAP_IO_CHUNK` flag is enabled.  Chunking may improve the speed of message
sending with HTTP, because the message length does not need to be determined in
advance for the HTTP content length header.

🔝 [Back to table of contents](#)

## HTTP buffered sends   {#buffered}

The entire outbound message can be stored in a buffer to determine the HTTP content length
rather than the two-phase encoding used by gSOAP client-side stub functions and server-side skeleton functions generated by soapcpp2, which perform a separate pass
over the data to determine the length of the outbound message.  Setting the
flag `#SOAP_IO_STORE` for the output mode will buffer the entire message.
This may or may not speed up the transmission of messages, depending on the content, but
may require significant storage space to hold large messages temporarily.

🔝 [Back to table of contents](#)

## HTTP authentication   {#authentication}

The following sections explain how to authenticate with HTTP bearer, basic, digest, and NTLM.  Proxy authentication is also covered.

You could also use the WinInet plugin available in the <i>`gsoap/mod_gsoap`</i>
directory of the gSOAP source code package to simplify Internet access for
gSOAP client applications and deal with encryption, proxies, and
authentication, see the gSOAP [WinInet plugin](../../wininet/html/index.html)
documentation.

The gSOAP CURL plugin can also be used to develop gSOAP client applications and CURL implements various HTTP authentication methods, see the gSOAP [CURL plugin](../../curl/html/index.html) documentation.

🔝 [Back to table of contents](#)

### HTTP bearer authentication   {#bearerauthentication}

HTTP bearer authentication is enabled at the client-side by setting the `::soap::bearer` string to the bearer token:

~~~{.cpp}
    struct soap *soap = soap_new(); 
    soap->bearer = "..."; 
    if (soap_call_ns__webmethod(soap, ...))
      ... // error
    else
      ... // OK
~~~

🔝 [Back to table of contents](#)

### HTTP basic authentication   {#basicauthentication}

HTTP basic authentication is enabled at the client-side by setting the
`const char* ::soap::userid` and `const char* ::soap::passwd` strings to a username and password,
respectively.  A server may request user authentication
and denies access (HTTP 401 error) when the client tries to connect without HTTP authentication (or with the wrong authentication information).

Basic authentication should only be used over HTTPS, because the credentials are sent in the clear with HTTP.
See Section \ref httpdaplugin to use the HTTP digest plugin that is safe for authentication over HTTP.

Here is an example client code fragment to set the HTTP authentication username and password:

~~~{.cpp}
    struct soap *soap = soap_new(); 
    soap->userid = "guest"; 
    soap->passwd = "visit"; 
    if (soap_call_ns__webmethod(soap, ...))
      ... // error
    else
      ... // success
~~~

A client SOAP request will have the following HTTP header:

    POST /XXX HTTP/1.1 
    Host: YYY 
    User-Agent: gSOAP/2.8 
    Content-Type: text/xml; charset=utf-8 
    Content-Length: ZZZ 
    Authorization: Basic Z3Vlc3Q6Z3Vlc3Q= 

A client must set the `const char* ::soap::userid` and `const char* ::soap::passwd` strings for each call that requires client authentication. The strings are reset after each successful or unsuccessful call.

When present, the `WWW-Authenticate` HTTP header returned by the server with the authentication realm is stored in the the `::soap::authrealm` string. This is useful for clients to use this domain information to respond to authentication requests.

A stand-alone gSOAP Web Service application can enforce HTTP authentication on clients by checking the `const char* ::soap::userid` and `const char* ::soap::passwd` strings. These strings are set when a client request contains HTTP authentication headers. The strings should be checked in each service method (that requires authentication to execute).

Here is an example service method implementation that enforced client authentication:

~~~{.cpp}
    int ns__webmethod(struct soap *soap, ...) 
    {
      if (!soap->userid
       || !soap->passwd
       || strcmp(soap->userid, "guest")
       || strcmp(soap->passwd, "visit")) 
      {
        soap->authrealm = "..."; // domain realm accessed (optional, NULL to omit)
        return 401; 
      }
      ... // webmethod logic
      return SOAP_OK;
    }
~~~

When the authentication fails, the service response with a SOAP Fault message and a HTTP error code "401 Unauthorized".
The HTTP error codes are described in Section \ref errcodes .

To return a non-SOAP error, use:

~~~{.cpp}
    return soap_send_empty_response(soap, 401); 
~~~

🔝 [Back to table of contents](#)

### HTTP NTLM authentication    {#NTLMauthentication}

HTTP NTLM authentication is enabled at the client-side by installing
`libntlm` from <http://www.nongnu.org/libntlm> and compiling all
project source codes with the compile-time flag `#WITH_NTLM`.

In your application code set the `const char* ::soap::userid` and `const char* ::soap::passwd` strings to a username and password.
A server may request NTLM
authentication and denies access (HTTP 401 authentication required or HTTP 407 HTTP proxy authentication required) when the client tries to
connect without HTTP authentication (or with the wrong authentication
information).

Here is an example client code fragment to set the NTLM authentication username and password:

~~~{.cpp}
    struct soap *soap = soap_new1(SOAP_IO_KEEPALIVE); 
    if (soap_call_ns__webmethod(soap, ...)) 
    {
      if (soap->error == 401) 
      {
        soap->userid = "Zaphod"; 
        soap->passwd = "Beeblebrox"; 
        if (soap_call_ns__webmethod(soap, ...)) 
          ... // error
        else
          ... // success
      }
    }
~~~

The following NTLM handshake between the client C and server S is performed:

    1: C  --> S & POST ... 
                & Content-Type: text/xml; charset=utf-8 
         
    2: C <--  S & 401 Unauthorized 
                & WWW-Authenticate: NTLM 
         
    3: C  --> S & GET ... 
                & Authorization: NTLM <base64-encoded type-1-message> 
         
    4: C <--  S & 401 Unauthorized 
                & WWW-Authenticate: NTLM <base64-encoded type-2-message> 
         
    5: C  --> S & POST ... 
                & Content-Type: text/xml; charset=utf-8 
                & Authorization: NTLM <base64-encoded type-3-message> 
         
    6: C <--  S & 200 OK

where stages 1 and 2 indicates a client attempting to connect without
authorization information, which is the first method call in the code above. Stage 3 to 6 happen with the proper client
authentication set with `const char* ::soap::userid` and `const char* ::soap::passwd`.  Optionally, the 
`const char* ::soap::authrealm` string should be set as well to indicate the domain accessed (this string is normally set when the server responds with HTTP 401 so the client receives this server domain information). NTLM authenticates connections, not requests. When the connection is kept alive, subsequent messages can be exchanged without re-authentication.

To avoid the overhead of the first rejected call, use:

~~~{.cpp}
    struct soap *soap = soap_new1(SOAP_IO_KEEPALIVE); 
    soap->ntlm_challenge = ""; 
    soap->userid = "Zaphod"; 
    soap->passwd = "Beeblebrox"; 
    soap->authrealm = "Ursa-Minor"; 
    if (soap_call_ns__webmethod(soap, ...)) 
      ... // error
    else
      ... // success
~~~

When the authentication fails (stage 1 and 2), the service response with 
HTTP error code "401 Unauthorized" and `::soap::error` is set to HTTP code 401.

🔝 [Back to table of contents](#)

### HTTP proxy NTLM authentication    {#NTLMproxyauthentication}

For HTTP 407 Proxy Authentication Required set the `::soap::proxy_userid` and `::soap::proxy_passwd`:

~~~{.cpp}
    struct soap *soap = soap_new1(SOAP_IO_KEEPALIVE); 
    soap->proxy_host = "..."; 
    soap->proxy_port = ...; 
    if (soap_call_ns__webmethod(soap, ...)) 
    {
      if (soap->error == 407) 
      {
        soap->proxy_userid = "Zaphod"; 
        soap->proxy_passwd = "Beeblebrox"; 
        soap->authrealm = "Ursa-Minor"; 
        if (soap_call_ns__webmethod(soap, ...)) 
          ... // error
        else
          ... // success
~~~

To avoid the overhead of the first rejected call, use:

~~~{.cpp}
    struct soap *soap = soap_new1(SOAP_IO_KEEPALIVE); 
    soap->proxy_host = "..."; 
    soap->proxy_port = ...; 
    soap->proxy_userid = "Zaphod"; 
    soap->proxy_passwd = "Beeblebrox"; 
    soap->authrealm = "Ursa-Minor"; 
    soap->ntlm_challenge = ""; 
    if (soap_call_ns__webmethod(soap, ...)) 
      ... // error
    else
      ... // success
~~~

🔝 [Back to table of contents](#)

### HTTP proxy basic authentication   {#proxyauthentication}

HTTP proxy authentication (basic) is enabled at the client-side by setting the
`::soap::proxy_userid` and `::soap::proxy_passwd` strings to a username and
password, respectively.  For example, a proxy server may request user
authentication. Otherwise, access is denied by the proxy (HTTP 407 error).
Example client code fragment to set proxy server, username, and password:

~~~{.cpp}
    struct soap *soap = soap_new(); 
    soap->proxy_host = "xx.xx.xx.xx"; // IP or domain 
    soap->proxy_port = 8080; 
    soap->proxy_userid = "guest"; 
    soap->proxy_passwd = "guest"; 
~~~

A client SOAP request will have the following HTTP header:

    POST /XXX HTTP/1.1 
    Host: YYY 
    User-Agent: gSOAP/2.8 
    Content-Type: text/xml; charset=utf-8 
    Content-Length: ZZZ 
    Proxy-Authorization: Basic Z3Vlc3Q6Z3Vlc3Q= 

When X-Forwarded-For headers are returned by the proxy, the header can be accessed in the `::soap::proxy_from` string.

The CONNECT method is used for HTTP proxy authentication:

    CONNECT server.example.com:80 HTTP/1.1

In some cases, you will notice that the Host HTTP header uses the CONNECT
protocol:

    CONNECT server.example.com:80 HTTP/1.1 
    Host: server.example.com:80

🔝 [Back to table of contents](#)

## Performance improvement tips    {#tips}

Here are some tips you can use to speed up gSOAP. The default settings are chosen to maximize portability and compatibility. The settings can be tweaked to optimize the performance as follows:

*  Increase the buffer size `#SOAP_BUFLEN` by changing the `#SOAP_BUFLEN` macro in <i>`gsoap/stdsoap2.h`</i>. Use buffer size 2^18=262144 for example.

*  Use HTTP keep-alive at the client-side, see Section \ref keepalive , when the client needs to make a series of calls to the same server. Server-side keep-alive support can greatly improve performance of both client and server. But be aware that clients and services under Unix/Linux require signal handlers to catch dropped connections.

*  Use HTTP chunked transfers, see Section \ref chunked .

*  Do not use gzip compression, since the overhead of compression is typically higher than the bandwidth gains.

*  Set the `#SOAP_XML_TREE` flag to disable id-ref multi-ref object serialization. This boosts performance significantly and works with SOAP document/literal style (i.e. no id-ref graph serialization as required with SOAP encoding style).  Do not use this for SOAP RPC encoded messaging.

*  Compile <i>`gsoap/stdsoap2.c`</i> and <i>`gsoap/stdsoap2.cpp`</i> and all other source codes with the compile time flag `#WITH_NOIDREF` to improve performance even better by permanently disabling id-ref multi-ref object serialization.  Do not use this for SOAP RPC encoded messaging.

*  Do not use DEBUG mode, since the overhead of logging is significant.

🔝 [Back to table of contents](#)

## Safety guards        {#safety}

The following settings are important to ensure that XML messaging is safe by
defining reasonable XML message restrictions.  Also messaging timeouts should
be set as explained in Section \ref timeout.

The XML parser is configured to restrict the XML nesting depth level to
`#SOAP_MAXLEVEL` and restricts the repeated occurrence of elements that are
deserialized into arrays and containers by `#SOAP_MAXOCCURS`. These macros can
be changed, but you can also change the following context attributes at
run-time, e.g. to enhance the safety for specific service and/or client
operations:

* `::soap::maxlevel` restricts the XML nesting depth level, where the default value is `#SOAP_MAXLEVEL` = 10000.

*  `::soap::maxoccurs` restricts the number of repeated occurrences of elements that are deserialized into arrays and structs, where the default value is `#SOAP_MAXOCCURS` = 100000.

*  `::soap::maxlength` restricts the length of strings deserialized from XML.  A zero or negative value is unrestricted length. When restricted, the XML schema validation maxLength takes precedence over this length restriction. So setting a smaller value will not interfere with the XML validation rules. The default value is `#SOAP_MAXLENGTH` = 0. Note that string length is expressed in number of characters, not bytes. So UTF-8 encodings are not truncated.

* `::soap::recv_maxlength` the maximum length of messages received, 2GB by default.

XML schema validation constraints are enforced with the `#SOAP_XML_STRICT` context flag. The schema maxLength validation constraint overrules the `::soap::maxlength` guard. The schema maxOccurs validation constraint does not overrule the `::soap::maxoccurs` guard, so arrays and containers are always restricted in length by this guard.

Other compile-time configuration settings are:

* `#SOAP_MAXALLOCSIZE` the maximum size of a block of memory that `malloc` can allocate.

* `#SOAP_MAXARRAYSIZE` the maximum allocation threshold to pre-allocate SOAP arrays in memory.

* `#SOAP_MAXDIMESIZE` the maximum DIME attachment size allowed to receive.

* `#SOAP_MAXEINTR` maximum number of EINTR interrupts to ignore while polling a socket for pending activity.

🔝 [Back to table of contents](#)

## Timeout management for non-blocking operations        {#timeout}

Socket connect, accept, send, and receive timeout values can be set to manage
socket communication timeouts.  The `::soap::connect_timeout`,
`::soap::accept_timeout`, `::soap::send_timeout`,
`::soap::recv_timeout` and `::soap::transfer_timeout` context attributes of
the current context `::soap` can be set to the appropriate
user-defined socket send, receive, and accept timeout values. A positive value
measures the timeout in seconds. A negative timeout value measures the timeout
in microseconds (10^-6 sec).

The `::soap::connect_timeout` specifies the timeout for `soap_call_ns__webmethod` stub function calls and for C++ proxy class calls.

The `::soap::accept_timeout` specifies the timeout for `::soap_accept` calls.

The `::soap::send_timeout` and `::soap::recv_timeout` specify the timeout
for non-blocking socket I/O operations.  This is the maximum delay on the socket operation permitted.

The `::soap::transfer_timeout` is new since 2.8.48 and limits the time a message send and a message receive operation can take.  This value should be used in combination with `::soap::send_timeout` and `::soap::recv_timeout` for accurate timeout control.  The `::soap::recv_maxlength` value when non-zero limits the length of messages that can be received in bytes in total in decompressed form (message length includes HTTP headers and HTTP chunk size fields).  The value of `::soap::recv_maxlength` is 2GB by default.

Example:

~~~{.cpp}
    struct soap *soap = soap_new(); 
    soap->send_timeout = 10; 
    soap->recv_timeout = 10; 
    soap->recv_timeout = 60; 
~~~

This will result in a timeout if no data can be send in 10 seconds and no data is received within 10 seconds after initiating
a send or receive operation over the socket. A value of zero disables timeout, for example:

~~~{.cpp}
    struct soap *soap = soap_new(); 
    soap->send_timeout = 0; 
    soap->recv_timeout = 0;  
    soap->recv_timeout = 0; 
~~~

When a timeout occurs in the send or receive operations, a `#SOAP_EOF` exception will be raised ("end of file or no input").
Negative timeout values measure timeouts in microseconds, for example:

~~~{.cpp}
    #define uSec *-1 
    #define mSec *-1000 

    struct soap *soap = soap_new(); 
    soap->accept_timeout = 10 uSec; 
    soap->send_timeout = 20 mSec; 
    soap->recv_timeout = 20 mSec; 
    soap->recv_timeout = 10;
~~~

@warning Many Linux versions do not support non-blocking `connect()`.
Therefore, setting `::soap::connect_timeout` for non-blocking
`soap_call_ns__webmethod` calls may not work under Linux.

@warning Interrupts (EINTR) can affect the blocking time in I/O operations.
The maximum number of EINTR that will not trigger an error is set by
`#SOAP_MAXEINTR` in <i>`gsoap/stdsoap2.h`</i>, which is 10 by default. Each EINTR
may increase the blocking time by up to one second, up to `#SOAP_MAXEINTR`
seconds total.

🔝 [Back to table of contents](#)

## Closing connections by force {#forceclose}

To close a socket connection by force, you can use `soap_force_closesock(soap)`, which closes the connection regardless if keep-alive is active.  By contrast, `soap_closesock(soap)` only closes the connection when keep-alive is not active.

To force-close a connection from another thread, compile stdsoap2.c or stdsoap2.cpp and your project source code with `#WITH_SELF_PIPE` to enable this feature.  Use `soap_close_connection(soap)` on the `::soap` context that must close.  You can make this call from another thread and pass the `::soap` context to this function of the thread that must close connections.

🔝 [Back to table of contents](#)

## Socket options and flags    {#socketopt}

Socket communications can be controlled with socket options and flags.
The `::soap` context flags are:

* `::soap::socket_flags` to set flags for the socket `send` and `recv` calls.

* `::soap::connect_flags` to set client-side `setsockopt` `SOL_SOCKET` socket options when connecting.

* `::soap::bind_flags` to set server-side `setsockopt` `SOL_SOCKET` socket options when executing `::soap_bind`.

* `::soap::bind_v6only` set to 1 to set `setsockopt` `IPPROTO_IPV6` `IPV6_V6ONLY`.

* `::soap::accept_flags` to set flags to the `setsockopt` socket options when executing `::soap_accept`.

See the operating system manual pages of `send` and `recv` for `::soap::socket_flags` values and
see the operating system manual pages of `setsockopt` for `::soap::connect_flags`,
`::soap::bind_flags`, and `::soap::accept_flags` (level `SOL_SOCKET`) values.  These `SO_`
socket option flags (see `setsockopt` manual pages) can be bit-wise or-ed to
set multiple socket options at once.

The client-side flag `::soap::connect_flags` = `SO_LINGER` is supported with values `l_onoff` = 1 and `l_linger` = `::soap::linger_time`. The `::soap::linger_time` determines the wait time (the time resolution is system dependent, though according to some experts only zero and nonzero values matter). The linger option can be used to manage the number of connections that remain in `TIME_WAIT` state at the server side.

For example, to disable SIGPIPE signals on Unix/Linux platforms use: 
`::soap::socket_flags` = `MSG_NOSIGNAL` and/or
`::soap::connect_flags` = `SO_NOSIGPIPE` (i.e. client-side connect) depending
on your platform.

Use `::soap::bind_flags` = `SO_REUSEADDR` to enable server-side port reuse and local port
sharing (but be aware of the possible security implications such as port hijacking).

Note that you have access to the `::soap->master` socket value returned by `::soap_bind` so you can set multiple socket options by calling `setsockopt` as follows:

~~~{.cpp}
    int sock = soap_bind(soap, host, port, backlog); 
    if (soap_valid_socket(sock)) 
    {
      setsockopt(sock, ..., ..., ..., ...);
      setsockopt(sock, ..., ..., ..., ...);
~~~

🔝 [Back to table of contents](#)

## Overriding the host and port to connect   {#hostportconnect}

To override the host and port of the client connecting to a server, set `::soap::override_host` and `::soap::override_port`:

~~~{.cpp}
    soap->override_host = "example.com"; // host name or IP address 
    soap->override_port = 80;            // port number to use when overriding the address
~~~

🔝 [Back to table of contents](#)

## Secure Web services with HTTPS {#serveropenssl}

To enable SSL for stand-alone gSOAP Web servers, first install OpenSSL and use
option the compile-time flag `#WITH_OPENSSL` to compile the sources with your C or C++ compiler
(or use the compile-time flag `#WITH_GNUTLS` if you prefer GNUTLS), for example:

     c++ -DWITH_OPENSSL -o myprog myprog.cpp stdsoap2.cpp soapC.cpp soapServer.cpp -lssl -lcrypto

With GNUTLS:

     c++ -DWITH_GNUTLS -o myprog myprog.cpp stdsoap2.cpp soapC.cpp soapServer.cpp -lgnutls -lgcrypt -lgpg-error

SSL support for stand-alone gSOAP Web services is enabled by calling
`::soap_ssl_accept` to perform the SSL/TLS handshake after `::soap_accept`.  In
addition, a key file, a CA file (or path to certificates), DH file (if RSA is
not used), and password need to be supplied. Instructions on how to do this can
be found in the OpenSSL documentation <http://www.openssl.org>. See also
Section \ref ssl .

Let's take a look at an example SSL secure
multi-threaded stand-alone SOAP Web Service:

~~~{.cpp}
    int main() 
    {
      SOAP_SOCKET m, s; 
      THREAD_TYPE tid; 
      struct soap *soap, *tsoap; 
      soap_ssl_init(); /* init OpenSSL (skipping this or calling multiple times is OK, since the engine will init SSL automatically) */
      /* soap_ssl_noinit(); */ /* do not init OpenSSL (if SSL is already initialized elsewhere in this application) */
      if (CRYPTO_thread_setup()) /* OpenSSL thread mutex setup */
      {
        fprintf(stderr, "Cannot setup thread mutex\n"); 
        exit(EXIT_FAILURE); 
      } 
      soap = soap_new(); 
      if (soap_ssl_server_context(soap, 
        SOAP_SSL_DEFAULT, 
        "server.pem",      /* keyfile: required when server must authenticate to clients (see SSL docs on how to obtain this file) */ 
        "password",        /* password to read the key file (not used with GNUTLS) */ 
        "cacert.pem",      /* optional cacert file to store trusted certificates */ 
        NULL,              /* optional capath to directory with trusted certificates */ 
        "dh512.pem",       /* DH file name or DH key len bits (minimum is 512, e.g. "512") to generate DH param, if NULL use RSA */ 
        NULL,              /* if randfile!=NULL: use a file with random data to seed randomness */  
        NULL               /* optional server identification to enable SSL session caching to speed up TLS (must be a unique name) */
      )) 
      {
        soap_print_fault(soap, stderr); 
        exit(EXIT_FAILURE); 
      } 
      m = soap_bind(soap, NULL, 18000, BACKLOG); /* use port 18000 */
      if (!soap_valid_socket(m)) 
      {
        soap_print_fault(soap, stderr); 
        exit(EXIT_FAILURE); 
      } 
      fprintf(stderr, "Socket connection successful: master socket = %d\n", m); 
      while (1)
      {
        s = soap_accept(soap); 
        fprintf(stderr, "Socket connection successful: slave socket = %d\n", s); 
        if (!soap_valid_socket(s)) 
        {
          if (soap->errnum) /* accept failed, try again after 1 second */
          {
            soap_print_fault(soap, stderr); 
            sleep(1);
            continue; /* retry */
          }
          fprintf(stderr, "Server timed out\n");
          break; 
        } 
        tsoap = soap_copy(soap);
        if (!tsoap) 
          soap_force_closesock(soap);
        else
          while (THREAD_CREATE(&tid, (void*(*)(void*))&process_request, (void*)tsoap))
            sleep(1); /* failed, try again */
      } 
      soap_free(soap); /* deallocates SSL context */
      CRYPTO_thread_cleanup(); /* OpenSSL thread mutex cleanup */
      return 0; 
    }  

    void *process_request(void *tsoap) 
    {
      struct soap *soap = (struct soap*)tsoap;
      THREAD_DETACH(THREAD_ID); 
      if (soap_ssl_accept(soap)) 
        soap_print_fault(soap, stderr); 
      else 
        soap_serve(soap); 
      soap_destroy(soap); 
      soap_end(soap); 
      soap_free(soap);
      return NULL; 
    }
~~~

The `::soap_ssl_server_context` function initializes the server-side SSL
context. The <i>`server.pem`</i> key file is the server's private key concatenated
with its certificate. The <i>`cacert.pem`</i> is used to authenticate clients and
contains the client certificates. Alternatively a directory name can be
specified. This directory is assumed to contain the certificates. The
<i>`dh512.pem`</i> file specifies that DH will be used for key agreement instead of
RSA. A numeric value greater than 512 can be provided instead as a string
constant (e.g. `"512"`) to allow the engine to generate the DH parameters on
the fly (this can take a while) rather than retrieving them from a file. The
randfile entry can be used to seed the PRNG. The last entry enable server-side
session caching to speed up TLS. A unique server name is required.

You can specify a cipher list to use with TLSv1.2 and below with
`SSL_CTX_set_cipher_list(soap->ctx, "...")` where `soap->ctx` is the SSL
context created by `::soap_ssl_server_context()`.  Likewise, use
`SSL_CTX_set_ciphersuites(soap->ctx, "...")` to configure the available TLSv1.3
ciphersuites.

We refer to the OpenSSL documentation and manual pages of
`SSL_CTX_set_cipher_list` and `SSL_CTX_set_ciphersuites` for details on the
latest cipher lists and suites available to use.

The GNUTLS mutex lock setup is automatically performed in the engine, but only
when POSIX threads are detected and available.

All OpenSSL versions prior to 1.1.0 require mutex locks to be explicitly set up
in your code for multi-threaded applications by calling `CRYPTO_thread_setup()`
and `CRYPTO_thread_cleanup()` as was shown in the code above.  OpenSSL 1.1.0
and greater does not require these locks to be set up.  If you are not sure
which version of OpenSSL you may be using with your multi-threaded application,
      then set up the locks.

For Unix and Linux, make sure you have signal handlers set in your service
and/or client applications to catch broken connections (`SIGPIPE`):

~~~{.cpp}
    signal(SIGPIPE, sigpipe_handle);
~~~

where, for example:

~~~{.cpp}
    void sigpipe_handle(int x) { }
~~~

By default, clients are not required to authenticate. To require client
authentication use the following:

~~~{.cpp}
    if (soap_ssl_server_context(&soap, 
        SOAP_SSL_REQUIRE_CLIENT_AUTHENTICATION, 
        "server.pem", 
        "password", 
        "cacert.pem", 
        NULL, 
        "dh512.pem", 
        NULL, 
        NULL)) 
    {
      soap_print_fault(&soap, stderr); 
      exit(EXIT_FAILURE); 
    }
~~~

This requires each client to authenticate with its certificate, in addition for
the server to authenticate to the client.

Since release version 2.8.20, SSL v3 is disabled. To enable SSL v3 together with
TLS 1.0 and higher, use `#SOAP_SSLv3_TLSv1` with `::soap_ssl_server_context`.
To use TLS 1.1 and 1.2 use `SOAP_TLSv1_1 | SOAP_TLSv1_2`. To use TLS 1.2 only
use `#SOAP_TLSv1_2`. To use SSL v3 only use `#SOAP_SSLv3`.

The `cacert` file and `capath` are optional. Either one can be specified when
clients must run on non-trusted systems (`capath` is not used with GNUTLS). We
want to avoid storing trusted certificates in the default location on the file
system when that is not secure. Therefore, a flat <i>`cacert.pem`</i> file or
directory can be specified to store trusted certificates.

The gSOAP package includes a <i>`cacerts.pem`</i> file with the certificates
of all certificate authorities. You can use this file to
verify the authentication of servers that provide certificates issued by these
CAs.

The <i>`cacert.pem`</i>, <i>`client.pem`</i>, and <i>`server.pem`</i> files in the gSOAP
package are examples of self-signed certificates.
The <i>`client.pem`</i> and <i>`server.pem`</i> contain the client/server
private key concatenated with the certificate. The keyfiles
(<i>`client.pem`</i> and <i>`server.pem`</i>) are created by concatenating the
private key PEM with the certificate PEM. The keyfile should not be shared with
any party. With OpenSSL, you can encrypt the keyfiles with a password to offer
some protection and the password is used in the client/server code to read the
keyfile. GNUTLS does not support this feature and cannot encrypt or decrypt a
keyfile.

@warning It is important that the `#WITH_OPENSSL` macro must be consistently defined to
compile the sources, such as <i>`gsoap/stdsoap2.cpp`</i>, <i>`soapC.cpp`</i>,
<i>`soapClient.cpp`</i>, <i>`soapServer.cpp`</i>, and all application sources that
include <i>`gsoap/stdsoap2.h`</i> or <i>`soapH.h`</i>. If the macros are not consistently
used, the application will crash due to a mismatches in the declaration and
access of the `::soap` context.

See also API documentation Module \ref group_ssl for more details on the SSL/TLS functions.

🔝 [Back to table of contents](#)

## Secure clients with HTTPS        {#clientopenssl}

To utilize HTTPS/SSL, you need to install the OpenSSL library on your platform
or GNUTLS for a light-weight SSL/TLS library.  After installation, compile all
the sources of your application with compile-time flag `#WITH_OPENSSL` (or
`#WITH_GNUTLS` when using GNUTLS). For example on Linux:

     c++ -DWITH_OPENSSL myclient.cpp stdsoap.cpp soapC.cpp soapClient.cpp -lssl -lcrypto

or Unix:

     c++ -DWITH_OPENSSL myclient.cpp stdsoap.cpp soapC.cpp soapClient.cpp -lxnet -lsocket -lnsl -lssl -lcrypto

or you can add the following line to <i>`soapdefs.h`</i>:

~~~{.cpp}
    #define WITH_OPENSSL
~~~

and compile with compile-time flag `#WITH_SOAPDEFS_H` to include <i>`soapdefs.h`</i> in your project.
Alternatively, compile with GNUTLS:

     c++ -DWITH_GNUTLS myclient.cpp stdsoap.cpp soapC.cpp soapClient.cpp -lgnutls -lgcrypt -lgpg-error


A client program simply uses the prefix <i>`https:`</i> instead of <i>`http:`</i> in the endpoint URL of a service operation call to a
Web Service to use encrypted transfers (if the service supports HTTPS). You need to specify the client-side key file and password of the keyfile:

~~~{.cpp}
    struct soap soap;
    soap_ssl_init(); /* init OpenSSL (skipping this or calling multiple times is OK, since the engine will init SSL automatically) */
    /* soap_ssl_noinit(); */ /* do not init OpenSSL (if SSL is already initialized elsewhere in this application) */
    soap_init(&soap);
    if (soap_ssl_client_context(&soap, 
      SOAP_SSL_DEFAULT, 
      "client.pem",        /* keyfile: required only when client must authenticate to server (see SSL docs on how to obtain this file) */ 
      "password",          /* password to read the key file (not used with GNUTLS) */ 
      "cacerts.pem",       /* cacert file to store trusted certificates (needed to verify server) */ 
      NULL,                /* capath to directory with trusted certificates */ 
      NULL         /* if randfile!=NULL: use a file with random data to seed randomness */  
    )) 
    {
      soap_print_fault(&soap, stderr); 
      exit(EXIT_FAILURE); 
    } 
    soap_call_ns__mymethod(&soap, "https://domain/path/secure.cgi", "", ...);
~~~

By default, server authentication is enabled and the <i>`cacerts.pem`</i> or
`capath` (not used with GNUTLS) must be set so that the CA certificates of the server(s) are
accessible at run time. The <i>`cacerts.pem`</i> file included in the gSOAP source code package
contains the certificates of common CAs. This file must be supplied with the
client, if server authentication is required. Alternatively, you can use the
<i>`gsoap/plugin/cacerts.h`</i> and <i>`gsoap/plugin/cacerts.c`</i> code to embed CA certificates
in your client code.

You can specify a cipher list to use with TLSv1.2 and below with
`SSL_CTX_set_cipher_list(soap->ctx, "...")` where `soap->ctx` is the SSL
context created by `::soap_ssl_client_context()`.  Likewise, use
`SSL_CTX_set_ciphersuites(soap->ctx, "...")` to configure the available TLSv1.3
ciphersuites.

We refer to the OpenSSL documentation and manual pages of
`SSL_CTX_set_cipher_list` and `SSL_CTX_set_ciphersuites` for details on the
latest cipher lists and suites available to use.

Other client-side SSL options are `#SOAP_SSL_SKIP_HOST_CHECK` to skip the host name verification check and `#SOAP_SSL_ALLOW_EXPIRED_CERTIFICATE` to allow connecting to a host with an expired certificate. For example,

~~~{.cpp}
    struct soap soap;
    soap_ssl_init(); /* init OpenSSL (skipping this or calling multiple times is OK, since the engine will init SSL automatically) */
    /* soap_ssl_noinit(); */ /* do not init OpenSSL (if SSL is already initialized elsewhere in this application) */
    soap_init(&soap);
    if (soap_ssl_client_context(&soap, 
      SOAP_SSL_REQUIRE_SERVER_AUTHENTICATION  
      | SOAP_SSL_SKIP_HOST_CHECK, 
      | SOAP_SSL_ALLOW_EXPIRED_CERTIFICATE, 
      "client.pem",        /* keyfile: required only when client must authenticate to server (see SSL docs on how to obtain this file) */ 
      "password",          /* password to read the key file (not used with GNUTLS) */ 
      "cacerts.pem",       /* cacert file to store trusted certificates (needed to verify server) */
      NULL,                /* capath to directory with trusted certificates */ 
      NULL         /* if randfile!=NULL: use a file with random data to seed randomness */  
    )) 
    {
      soap_print_fault(&soap, stderr); 
      exit(EXIT_FAILURE); 
    } 
    soap_call_ns__mymethod(&soap, "https://domain/path/secure.cgi", "", ...);
~~~

For systems based on Microsoft windows, the WinInet module can be used instead, see `mod_gsoap/gsoap_win/wininet`.

Since release version 2.8.20 SSL v3 is disabled. To enable SSL v3 together with TLS 1.0 and higher, use `#SOAP_SSLv3_TLSv1` with `::soap_ssl_server_context`. To use TLS 1.1 and 1.2 use `SOAP_TLSv1_1 | SOAP_TLSv1_2`. To use TLS 1.2 only use `#SOAP_TLSv1_2`. To use SSL v3 only use `#SOAP_SSLv3`.

To disable server authentication for testing purposes, use the following:

~~~{.cpp}
    if (soap_ssl_client_context(&soap, 
      SOAP_SSL_NO_AUTHENTICATION, 
      NULL, 
      NULL, 
      NULL, 
      NULL, 
      NULL 
    )) 
    {
      soap_print_fault(&soap, stderr); 
      exit(EXIT_FAILURE); 
    } 
~~~

This also assumes that the server does not require clients to authenticate (the keyfile is absent).

Make sure you have signal handlers set in your application to catch broken connections (`SIGPIPE`):

~~~{.cpp}
    signal(SIGPIPE, sigpipe_handle);
~~~

where, for example:

~~~{.cpp}
    void sigpipe_handle(int x) { }
~~~

@warning It is important that the `#WITH_OPENSSL` macro is consistently defined to
compile the sources, such as <i>`gsoap/stdsoap2.cpp`</i>, <i>`soapC.cpp`</i>,
<i>`soapClient.cpp`</i>, <i>`soapServer.cpp`</i>, and all application sources that
include <i>`gsoap/stdsoap2.h`</i> or <i>`soapH.h`</i>. If the macros are not consistently
used, the application will crash due to a mismatches in the declaration and
access of the `::soap` context.  Alternatively, use library <b>`-lgsoapssl`</b>
or <b>`-lgsoapssl++`</b> and compile everything else with `#WITH_OPENSSL`.

@warning Concurrent client calls with threads should use separate `::soap`
contexts In addition, the thread initialization code discussed in Section
\ref serveropenssl  must be used to properly setup OpenSSL in a multi-threaded
client application.

See also API documentation Module \ref group_ssl for more details on the SSL/TLS functions.

You can also use the WinInet interface to establish secure HTTPS connections on
Windows machines, available in the <i>`gsoap/mod_gsoap`</i> directory of the
gSOAP source code package, see also Section \ref wininetplugin.  Or you can use the CURL
plugin to use CURL to establish secure HTTPS connections, see Section
\ref curlplugin.

🔝 [Back to table of contents](#)

## SSL authentication callbacks   {#sslauthenticationcallbacks}

The `fsslauth` callback function controls OpenSSL/GNUTLS authentication initialization:

* `int (soap::fsslauth)(struct soap *soap)` 
  This callback is called to initialize the OpenSSL or GNUTLS context for HTTPS connections configured with the parameters passed to `::soap_ssl_client_context` and `::soap_ssl_server_context`.  Returns `#SOAP_OK` or a `::soap_status` (int) error code.  The built-in function assigned to `::soap::fsslauth` is `ssl_auth_init`.

The `fsslverify` callback function controls OpenSSL peer certificate
verification, via internally invoking `SSL_CTX_set_verify`:

* `int (soap::fssverify)(int ok, X509_STORE_CTX *store` 
  This callback is called by the engine to manage the verification of the certificate provided by a peer, such as the certificate provided by a server connected over HTTPS or to verify the certificate included with a WS-Security message.  To require certificate verification of a server connected via HTTPS, use `::soap_ssl_client_context` with `#SOAP_SSL_REQUIRE_SERVER_AUTHENTICATION`.  To require certificate verification of a client connected to a server, use `::soap_ssl_server_context` with `#SOAP_SSL_REQUIRE_CLIENT_AUTHENTICATION`.  The `ok` parameter of this callback indicates whether the verification of the certificate in question passed (`ok` == 1) or failed (`ok` == 0) as determined by the OpenSSL library based on the `::soap_ssl_client_context` or `::soap_ssl_server_context` configuration.  If the callback returns 1 then the handshake is continued and the connection maybe established.  To return 1 when `ok` == 0 requires resetting the error state with `X509_STORE_CTX_set_error(store, X509_V_OK)`.  If the callback returns 0 then the handshake is immediately terminated with "verification failed" and a verification failure alert is sent to the peer.  The built-in function assigned to `::soap::fsslverify` is `ssl_verify_callback` or when `#SOAP_SSL_ALLOW_EXPIRED_CERTIFICATE` is used `ssl_verify_callback_allow_expired_certificate`.

🔝 [Back to table of contents](#)

## SSL certificates and key files        {#ssl}

The gSOAP package includes a <i>`cacerts.pem`</i> file with the certificates
of all certificate authorities. You can use this file to
verify the authentication of servers that provide certificates issued by these
CAs. Just set the `cafile` parameter to the location of this file on your
file system.  Therefore, when you obtain a certificate signed by a trusted CA
you can simply use the <i>`cacerts.pem`</i> file to develop
client applications that can verify the authenticity of your server.

Alternatively, you can use the <i>`gsoap/plugin/cacerts.h`</i> and
<i>`gsoap/plugin/cacerts.c`</i> code to embed CA certificates in your client code.

For systems based on Microsoft windows, the WinInet module can be used instead,
see the `README.txt` located in the gSOAP source code package under
`mod_gsoap/gsoap_win/wininet`, see the gSOAP [WinInet plugin](../../wininet/html/index.html) documentation.

The other <i>`.pem`</i> files in the gSOAP package are examples
of self-signed certificates for testing purposes (<i>`cacert.pem`</i>,
<i>`client.pem`</i>, <i>`server.pem`</i>). The <i>`client.pem`</i> and <i>`server.pem`</i> contain the
private key and certificate of the client or server, respectively. The keyfiles
(<i>`client.pem`</i> and <i>`server.pem`</i>) are created by concatenating the private key
PEM with the certificate PEM. The keyfile should not be shared with any party.
With OpenSSL, you can encrypt the keyfiles with a password to offer some
protection and the password is used in the client/server code to read the
keyfile. GNUTLS does not support this feature and cannot encrypt or decrypt a
keyfile.

You can also create your own self-signed certificates.  There is more than one
way to generate the necessary files for clients and servers.
See <http://www.openssl.org> for information on OpenSSL and
<http://sial.org/howto/openssl/ca/> on how to setup and manage a local CA
and <http://sial.org/howto/openssl/self-signed> on how to setup self-signed
test certificates.

It is possible to convert IIS-generated certificates to PEM format with the openssl library and openssl command-line tool:

    openssl x509 -in mycert.cer -inform DER -out mycert.pem -outform PEM

This converts the CRT-formatted mycert.cer to PEM-formatted mycert.pem.

Here is the simplest way to setup self-signed certificates. First you need to create a private Certificate Authority (CA).  The CA is used in SSL to verify the authenticity of a given
certificate. The CA acts as a trusted third party who has authenticated the
user of the signed certificate as being who they say. The certificate is
signed by the CA, and if the client trusts the CA, it will trust your
certificate. For use within your organization, a private CA will probably
serve your needs. However, if you intend use your certificates for a public
service, you should probably obtain a certificate from a known CA.
In addition to identification, your certificate is also used for encryption.

Creating certificates should be done through a CA to obtain signed certificates. But you can create your own certificates for testing purposes as follows.


*  Go to the OpenSSL bin directory (`/usr/local/ssl` by default and
`/System/Library/OpenSSL` on Mac OS X)

*  There should be a file called openssl.cnf

*  Create a new directory in your home account, e.g. $HOME/CA, and copy the openssl.cnf file to this directory

*  Modify openssl.cnf by changing the 'dir' value to HOME/CA

*  Copy the README.txt, root.sh, and cert.sh scripts from the gSOAP source code
   package located in the <i>`gsoap/samples/ssl`</i> directory to HOME/CA

*  Follow the README.txt instructions

You now have a self-signed CA root certificate cacert.pem and a server.pem (or client.pem) certificate in PEM format.
The cacert.pem certificate is used in the `cafile` parameter of the `::soap_ssl_client_context` (or `::soap_ssl_server_context`) at the client (or server) side to verify the authenticity of the peer. You can also provide a capath parameter to these trusted certificates. The server.pem (or client.pem) must be provided with the `::soap_ssl_server_context` at the server side (or `::soap_ssl_client_context` at the client side) together with the password you entered when generating the certificate using cert.sh to access the file. These certificates must be present to grant authentication requests by peers. In addition, the server.pem (and client.pem) include the host name of the machine on which the application runs (e.g. localhost), so you need to generate new certificates when migrating a server (or client).

Finally, you need to generate Diffie-Helmann (DH) parameters for the server if
you wish to use DH instead of RSA. There are two options:


*  Set the `dhfile` parameter to the numeric DH prime length in bits
required (for example "1024") to let the engine generate DH parameters at
initialization. This can be time consuming.

*  Provide a file name for the `dhfile` parameter of
`::soap_ssl_server_context`. The file should be generated beforehand. To
do so with the OpenSSL command line tool, use:

     openssl dhparam -outform PEM -out dh.pem 512

File <i>`dh512.pem`</i> is the output file and 512 is the number of bits used.

🔝 [Back to table of contents](#)

## SSL hardware acceleration   {#sslhw}

You can specify a hardware engine to enable hardware support for cryptographic acceleration. This can be done once in a server or client with the following statements:

~~~{.cpp}
    static const char *engine = "cswift"; /* engine name */ 
    int main() 
    {
      ENGINE *e; 
      if (!(e = ENGINE_by_id(engine))) 
        fprintf(stderr, "Error finding engine %s\n", engine); 
      else if (!ENGINE_set_default(e, ENGINE_METHOD_ALL)) 
        fprintf(stderr, "Error using engine %s\n", engine); 
      ... //
    }
~~~

The following table lists the names of the hardware and software engines:

engine               | result
-------------------- | ------
`openssl`            | The default software engine for cryptographic operations 
`openbsd_dev_crypto` | OpenBSD supports kernel level cryptography 
`cswift`             | CryptoSwift acceleration hardware 
`chil`               | nCipher CHIL acceleration hardware 
`atalla`             | Compaq Atalla acceleration hardware 
`nuron`              | Nuron acceleration hardware 
`ubsec`              | Broadcom uBSec acceleration hardware 
`aep`                | Aep acceleration hardware 
`sureware`           | SureWare acceleration hardware 

🔝 [Back to table of contents](#)

## SSL on Windows   {#sslwin}

Set the full path to <i>`libssl.lib`</i> and <i>`libcrypto.lib`</i>
under the MSVC++ "Projects" menu, then choose "Link": "Object/Modules".
Please make sure <i>`libssl32.dll`</i> and <i>`libeay32.dll`</i> can be loaded
by your applications, thus they must be installed properly on the target
machine.

If you're using compilation settings such as `/MTd` then link to the correct
<i>`libeay32MTd.lib`</i> and <i>`ssleay32MTd.lib`</i> libraries.

Alternatively, you can use the WinInet interface for secure connections,
available in the <i>`gsoap/mod_gsoap`</i> directory of the gSOAP package, see
also Section \ref wininetplugin.  Or you can use the CURL plugin to use
CURL for secure connections, see Section \ref curlplugin.

🔝 [Back to table of contents](#)

## Zlib compression        {#compression}

To enable deflate and gzip compression with Zlib, install Zlib from
<http://www.zlib.org> if not already installed on your system.  Compile
<i>`gsoap/stdsoap2.cpp`</i> (or <i>`gsoap/stdsoap2.c`</i>) and all your sources that include
<i>`gsoap/stdsoap2.h`</i> or <i>`soapH.h`</i> with compile-time flag `#WITH_GZIP` and
link your code with the Zlib library, e.g. <b>`-lz`</b> on Unix/Linux platforms.

The gzip compression is orthogonal to all transport encodings such as HTTP,
SSL, DIME, and can be used with other transport layers.  You can even save and
load compressed XML data to/from files.

Two compression formats are supported by the engine: deflate and gzip. The gzip format is
used by default. The gzip format has several benefits over deflate. Firstly,
the engine automatically detects gzip compressed inbound messages, even without
HTTP headers, by checking for the presence of a gzip header in the message
content. Secondly, gzip includes a CRC32 checksum to ensure messages have been
correctly received. Thirdly, gzip compressed content can be decompressed with
other compression software, so you can decompress XML data saved by a gSOAP
application in gzip format.

Gzip compression is enabled by compiling the sources with compile-time flag `#WITH_GZIP`.
To transmit gzip compressed SOAP/XML data, set the output mode flags to
`#SOAP_ENC_ZLIB`. For example:

~~~{.cpp}
    soap_init(&soap); 
    soap_set_omode(&soap, SOAP_ENC_ZLIB); // enable Zlib's gzip 
    if (soap_call_ns__myMethod(&soap, ...)) 
      ... // error
    else
      ... // success
    soap_clr_omode(&soap, SOAP_ENC_ZLIB); // disable Zlib's gzip 
~~~

This will send a compressed SOAP/XML request to a service, provided that Zlib is
installed and linked with the application and the compile-time flag `#WITH_GZIP` option was used to compile the sources.
Receiving compressed SOAP/XML over HTTP either in gzip or deflate formats is automatic. The `#SOAP_ENC_ZLIB` flag does not have
to be set at the server side to accept compressed messages. Reading and receiving gzip compressed SOAP/XML without HTTP headers (e.g. with other transport protocols) is also automatic.

To control the level of compression for outbound messages, you can set the `::soap::z_level` to a value between 1 and 9, where 1 is the best speed and 9 is the best compression (default is 6).  For example

~~~{.cpp}
    soap_init(&soap); 
    soap_set_omode(&soap, SOAP_ENC_ZLIB); 
    soap.z_level = 9; // best compression 
~~~

To verify and monitor compression rates, you can use the values `::soap::z_ratio_in` and `::soap::z_ratio_out`. These two float values lie between 0.0 and 1.0 and express the ratio of the compressed message length over uncompressed message length.

~~~{.cpp}
    if (soap_call_ns__myMethod(&soap, ...))
    {
      ... // error
    }
    else
    {
      printf("Compression ratio: %f%% (in) %f%% (out)\n", 100*soap.z_ratio_out, 100*soap.z_ratio_in); 
      ... // success
    }
~~~

Note: lower ratios mean higher compression rates.

Compressed transfers require buffering the entire output message to determine HTTP message length.
This means that the `#SOAP_IO_STORE` flag is
automatically set when the `#SOAP_ENC_ZLIB` flag is set to send compressed messages. The use of HTTP chunking
significantly reduces memory usage and may speed up the transmission of compressed SOAP/XML messages.
This is accomplished by setting the `#SOAP_IO_CHUNK` flag with
`#SOAP_ENC_ZLIB` for the output mode.
However, some Web servers do not accept HTTP chunked request messages (even when they return HTTP chunked messages!). Stand-alone gSOAP services always accept chunked request messages.

To restrict the compression to the deflate format only, compile the sources with the compile-time flag `#WITH_ZLIB`. This limits compression and decompression to the deflate format. Only plain and deflated messages can be exchanged, gzip is not supported with this option.
Receiving gzip compressed content is automatic, even in the absence of HTTP headers.
Receiving deflate compressed content is not automatic in the absence of HTTP headers and requires the flag
`#SOAP_ENC_ZLIB` to be set for the input mode to decompress deflated data.

@warning It is important that the `#WITH_GZIP` and `#WITH_ZLIB` macros must be consistently defined to
compile the sources, such as <i>`gsoap/stdsoap2.cpp`</i>, <i>`soapC.cpp`</i>,
<i>`soapClient.cpp`</i>, <i>`soapServer.cpp`</i>, and all application sources that
include <i>`gsoap/stdsoap2.h`</i> or <i>`soapH.h`</i>. If the macros are not consistently
used, the application will crash due to a mismatches in the declaration and
access of the `::soap` context.

🔝 [Back to table of contents](#)

## Client-side cookie support        {#clientcookie}

Client-side cookie support is optional. To enable cookie support, compile all sources with the compile-time flag `#WITH_COOKIES`, for example:

     c++ -DWITH_COOKIES -o myclient stdsoap2.cpp soapC.cpp soapClient.cpp

or add the following line to <i>`stdsoap.h`</i>:

~~~{.cpp}
    #define WITH_COOKIES
~~~

Client-side cookie support is fully automatic. So just compile <i>`gsoap/stdsoap2.cpp`</i> with the compile-time flag `#WITH_COOKIES` to enable
cookie-based session control in your client.

A cookie store with cookies is kept and returned to the appropriate servers when the client connects to these servers.
Cookies are not automatically saved to a file by a client. An example cookie
file manager is included as an extras in the gSOAP package. You can
remove all cookies from a `::soap` context by
calling `soap_free_cookies(soap)`, which also happens when you call `soap_done(soap)`.

To avoid "cookie storms" caused by malicious servers that return an 
unreasonable amount of cookies, gSOAP clients/servers are limited to
a cookie store size of `::soap::cookie_max` that the user can change, for example:

~~~{.cpp}
    struct soap *soap = soap_new(); 
    soap->cookie_max = 10;
~~~

The cookie store is a linked list of `::soap_cookie` structures pointed to by `::soap::cookies`.

Since the cookie store is linked to the current `::soap` context, and each thread must use its own context, each thread also has its own cookie store.

🔝 [Back to table of contents](#)

## Server-side cookie support        {#servercookie}

This feature is not recommended to implement state in stand-alone servers.  Cookies may require a fair amount of processing overhead and are not in fact needed to implement stateful services, which is typically performed with session IDs in XML/JSON messages or by passing the session IDs via the URL.

Server-side cookie support is optional. To enable cookie support, compile all sources with compile-time flag `#WITH_COOKIES`:

     c++ -DWITH_COOKIES -o myserver ...

See API documentation Module \ref group_cookies for the cookie API functions.  See the [HTTP sessions plugin](../../sessions/html/index.html) for HTTP session management with cookies that is required for server-side session control.  The `#WITH_COOKIES` flag is useless without server-side session management and control.

Here is an overview of the cookie API functions:

* `struct soap_cookie *soap_set_cookie(struct soap *soap, const char *name, const char *value, const char *domain, const char *path);` 
  This function adds a cookie to the cookie store at the server side, if not already there, with the specified `name` and `value`.  The `domain` and `path` parameters can be specified or can be NULL to use the current domain and path given by `::soap::cookie_domain` and `::soap::cookie_path`.  Returns pointer to the cookie structure in the database or NULL when an error occurred.

* `int soap_set_cookie_expire(struct soap *soap, const char *name, long expire, const char *domain, const char *path);` 
  This function sets the expiration of the specified cookie `name` in seconds and updates the cookie store at the server side.  The `domain` and `path` parameters can be specified or can be NULL to use the current domain and path given by `::soap::cookie_domain` and `::soap::cookie_path`.  Returns `#SOAP_OK` or a `::soap_status` error code.

* `int soap_set_cookie_secure(struct soap *soap, const char *name, const char *domain, const char *path);` 
  This function sets the "secure" property of the specified cookie `name` and updates the cookie store at the server side.  The "secure" property means that this cookie should be sent by the client to the server only when a secure HTTPS connection can be established.  When HTTPS is enabled all cookies are sent by the server to the client with the "secure" property set, which means that this function is generally not needed unless the server is not HTTPS-enabled but cookies must be secure.  The `domain` and `path` parameters can be specified or can be NULL to use the current domain and path given by `::soap::cookie_domain` and `::soap::cookie_path`.  Returns `#SOAP_OK` or a `::soap_status` error code.

* `int soap_set_cookie_session(struct soap *soap, const char *name, const char *domain, const char *path);` 
  This function makes the specified cookie `name` a "session cookie" and updates the cookie store at the server side by marking the cookie as a session cookie.  This means that the cookie will be sent to clients that connect to the server.  This function is not needed when a cookie is modified with `::soap_set_cookie_expire`, for example, because modified cookies are always sent back to the client.  The `domain` and `path` parameters can be specified or can be NULL to use the current domain and path given by `::soap::cookie_domain` and `::soap::cookie_path`.  Returns `#SOAP_OK` or a `::soap_status` error code.

* `void soap_clr_cookie(struct soap *soap, const char *name, const char *domain, const char *path);` 
  This function deletes the specified cookie `name` from the cookie store at the server side.  The `domain` and `path` parameters can be specified or can be NULL to use the current domain and path given by `::soap::cookie_domain` and `::soap::cookie_path`.

* `int soap_clr_cookie_session(struct soap *soap, const char *name, const char *domain, const char *path);` 
  This function clears the session property of the specified cookie `name` and updates the cookie store at the server side.  The `domain` and `path` parameters can be specified or can be NULL to use the current domain and path given by `::soap::cookie_domain` and `::soap::cookie_path`.  Returns `#SOAP_OK` or a `::soap_status` error code.

* `struct soap_cookie *soap_cookie(struct soap *soap, const char *name, const char *domain, const char *path);` 
  This function returns the cookie structure of the specified cookie `name` or NULL when not found by searching the cookie store at the server side.  The `domain` and `path` parameters can be specified or can be NULL to use the current domain and path given by `::soap::cookie_domain` and `::soap::cookie_path`.

* `const char *soap_cookie_value(struct soap *soap, const char *name, const char *domain, const char *path);` 
  This function returns the cookie value of the specified cookie `name` or NULL when not found by searching the cookie store at the server side.  The `domain` and `path` parameters can be specified or can be NULL to use the current domain and path given by `::soap::cookie_domain` and `::soap::cookie_path`.

* `time_t soap_cookie_expire(struct soap *soap, const char *name, const char *domain, const char *path);` 
  This function returns the cookie expiration time `time_t` of the specified cookie `name` or -1 when not found by searching the cookie store at the server side.  The `domain` and `path` parameters can be specified or can be NULL to use the current domain and path given by `::soap::cookie_domain` and `::soap::cookie_path`.

* `int soap_getenv_cookies(struct soap *soap);` 
  This function initializes the cookie store at the server side by reading the `HTTP_COOKIE` environment variable.  This provides a means for a CGI application to read cookies sent by a client.  Returns `#SOAP_OK` or a `::soap_status` error code when the `HTTP_COOKIE` variable was not found.

* `void soap_free_cookies(struct soap *soap);` 
  This function frees the cookie store and deletes all cookies.

The following variables of the `::soap` context are used to define the current domain and path:

* `const char * soap::cookie_domain` should be set to the domain (host) of the service 

* `const char * soap::cookie_path` may be set to the default path to the service 

* `int soap::cookie_max` maximum cookie database size (default=32) 

The `::soap::cookie_path` value is used to filter cookies intended for this service according to the path prefix rules outlined in
RFC2109.

The following example server adopts cookies for session control:

~~~{.cpp}
    int main() 
    {
      struct soap soap; 
      int m, s; 
      soap_init(&soap); 
      soap.cookie_domain = "..."; 
      soap.cookie_path = "/"; // the path which is used to filter/set cookies with this destination 
      if (argc < 2) 
      {
        soap_getenv_cookies(&soap); // CGI app: grab cookies from 'HTTP_COOKIE' env var 
        soap_serve(&soap); 
      } 
      else 
      {
        m = soap_bind(&soap, NULL, atoi(argv[1]), 10); // small BACKLOG for iterative servers
        if (!soap_valid_socket(m)) 
          exit(EXIT_FAILURE); 
        for (int i = 1; ; i++) 
        {
          s = soap_accept(&soap); 
          if (!soap_valid_socket(s)) 
            exit(EXIT_FAILURE); 
          soap_serve(&soap); 
          soap_end(&soap);
          soap_free_cookies(&soap); // remove all old cookies from database so no interference occurs with the arrival of new cookies 
        } 
      } 
      return 0; 
    } 

    int ns__webmethod(struct soap *soap, ...) 
    {
      const char *cookie_value = soap_cookie_value(soap, "cookie_name", NULL, NULL);
      if (!cookie_value)                // cookie returned by client? 
        cookie_value = "initial_value"; // no: set initial cookie value 
      else 
        ...                             // yes: modify the cookie value to reflect the new state
      soap_set_cookie(soap, "cookie_name", cookie_value, NULL, NULL); 
      soap_set_cookie_expire(soap, "cookie_name", 60, NULL, NULL); // cookie expires in 60 seconds 
      return SOAP_OK; 
    }
~~~

🔝 [Back to table of contents](#)

## Connecting clients through proxy servers   {#proxyservers}

When a client needs to connect to a Web Service through a proxy server, set the `::soap::proxy_host` string and
`::soap::proxy_port` integer attributes of the current `::soap` context to the proxy's host name and port, respectively. For example:

~~~{.cpp}
    struct soap soap; 
    soap_init(&soap); 
    soap.proxy_host = "proxyhostname"; 
    soap.proxy_port = 8080; 
    if (soap_call_ns__webmethod(&soap, "http://host:port/path", "action", ...)) 
      soap_print_fault(&soap, stderr); 
    else 
      ... // success
~~~

The context attributes `::soap::proxy_host` and `::soap::proxy_port` keep their values through a sequence of service operation calls,
so they only need to be set once.

When X-Forwarded-For headers are returned by the proxy, the header can be accessed in the `::soap::proxy_from` string.

See also Sections \ref proxyauthentication and \ref NTLMproxyauthentication.

🔝 [Back to table of contents](#)

## Bind before connect and setting the client interface address   {#bindbeforeconnect}

To bind the client to a port before connect, set the `::soap::client_port` to a non-negative port number:

~~~{.cpp}
    struct soap *soap = soap_new(); 
    soap->client_port = ...; // non-negative port number 
    if (soap_call_ns__webmethod(soap, "http://host:port/path", "action", ...)) 
      ... // error
    else 
      ... // success
~~~

This port number is used only once and reset to -1 (disabled).  Set it again for the next call.

To set a client interface address for the connection that is an IP address of the client:

~~~{.cpp}
    struct soap *soap = soap_new(); 
    soap->client_interface = "..."; // IP address 
    if (soap_call_ns__webmethod(soap, "http://host:port/path", "action", ...)) 
      ... // error
    else 
      ... // success
~~~

This client interface address string is used only once and reset to NULL (disabled).  Set it again for the next call.  This feature is not available when compiling the code on windows.

🔝 [Back to table of contents](#)

## FastCGI {#fastcgi}

To enable FastCGI support, install FastCGI and compile all soapcpp2-generated
source code files and your application sources with the compile-time flag `#WITH_FASTCGI`
or add:

~~~{.cpp}
    #define WITH_FASTCGI
~~~

to <i>`gsoap/stdsoap2.h`</i> and recompile the project code.

@warning Do not link against the <i>`gsoap/libgsoap*`</i> libraries as these are not suitable
for FastCGI. Compile <i>`gsoap/stdsoap2.c`</i> (or <i>`gsoap/stdsoap2.cpp`</i>)
instead.

🔝 [Back to table of contents](#)

## How to minimize application memory footprint        {#lean}

To compile gSOAP applications intended for small memory devices, you may want
to remove all non-essential features that consume precious code and data space.
To do this, compile the source code files with the compile-time flag `#WITH_LEAN` (i.e. `#define WITH_LEAN`) to remove many
non-essential features. The features that will be disabled are:

*  No UDP support

*  No HTTP keep alive support

*  No HTTP bearer, basic, and digest authentication

*  No HTTP chunked output with `#SOAP_IO_CHUNK`, but chunked HTTP input is accepted

*  No HTTP compressed output with `#SOAP_ENC_ZLIB`, but compressed input is accepted

*  No canonical XML output with `#SOAP_XML_CANONICAL`

*  No `::soap::connect_timeout`, `::soap::send_timeout`, `::soap::recv_timeout` and `::soap::transfer_timeout` timeouts

*  No support for socket flags, no `::soap::socket_flags`, `::soap::connect_flags`, `::soap::bind_flags`, `::soap::accept_flags`

*  No support for `time_t` serialization with the `xsd__dateTime` type, use a string to store the date and time instead.

Use the compile-time flag `#WITH_LEANER` to make the executable even smaller by removing DIME
and MIME attachment handling, `#LONG64` (64 bit) serialization, `wchar_t*` serialization, and support for XML DOM operations.
Note that DIME/MIME attachments are not essential to achieve
SOAP/XML interoperability.  DIME attachments are a convenient way to exchange
non-text-based (i.e. binary) content, but are not required for basic SOAP/XML
interoperability.  Attachment requirements are predictable.  That is,
applications won't suddenly decide to use DIME or MIME instead of XML to exchange
content.

It is safe to try to compile your application with the compile-time flag `#WITH_LEAN`, provided
that your application does not rely on I/O timeouts. When no linkage error
occurs in the compilation process, it is safe to assume that your application
will run just fine.

🔝 [Back to table of contents](#)

## How to remove the BSD socket library requirement        {#noio}

The <i>`gsoap/stdsoap2.c`</i> and <i>`gsoap/stdsoap2.cpp`</i> libraries should be linked with a BSD socket library in the project build. To remove the need to link a socket library, you can compile <i>`gsoap/stdsoap2.c`</i> (for C) and <i>`gsoap/stdsoap2.cpp`</i> (for C++) with the the compile-time flag `#WITH_NOIO` macro set (i.e. `#define WITH_NOIO`). This removes the dependency on the BSD socket API, IO streams, `FILE` type, and `errno`.

You should define callbacks to replace the missing socket stack. To do so, add to your code the following definitions:

~~~{.cpp}
    struct soap soap; 
    soap_init(&soap); 
    /* fsend is used to transmit data in blocks */ 
    soap.fsend = my_send; 
    /* frecv is used to receive data in blocks */ 
    soap.frecv = my_recv; 
    /* fopen is used to connect */ 
    soap.fopen = my_tcp_connect; 
    /* fclose is used to disconnect */ 
    soap.fclose = my_tcp_disconnect; 
    /* fclosesocket is used only to close the master socket in a server upon soap_done() */ 
    soap.fclosesocket = my_tcp_closesocket; 
    /* fshutdownsocket is used after completing a send operation to send TCP FIN */ 
    soap.fshutdownsocket = my_tcp_shutdownsocket; 
    /* setting fpoll is optional, leave it NULL to omit polling the server */ 
    soap.fpoll = my_poll; 
    /* faccept is used only by a server application */ 
    soap.faccept = my_accept;
~~~

These functions should provide a minimal input/output stack (`::soap::frecv` and `::soap::fsend`) to read/write XML and socket-like stack (the other functions) when developing client and server applications.

See Section \ref callback  for more details on the use of these callbacks.
All callback function pointers should be non-NULL, except `::soap::fpoll`.

You cannot use `::soap_print_fault` and `::soap_print_fault_location` to print error diagnostics. Instead, the value of `::soap::error`, which contains the gSOAP error code, can be used to determine the cause of a fault.

🔝 [Back to table of contents](#)

## How to combine multiple client and server implementations into one executable    {#combine}

The wsdl2h tool can be used to import multiple WSDLs and schemas at once.
The service definitions are combined in one header file to be parsed by
soapcpp2. It is important to assign namespace prefixes to namespace URIs
using the <i>`typemap.dat`</i> file. Otherwise, wsdl2h will assign namespace
prefixes `ns1`, `ns2`, and so on to the service operations and schema
types. Thus, any change to a WSDL or schema may result in a new prefix
assignment. For more details, please see Section \ref typemap .

Another approach to combine multiple client and service applications into one
executable is by using C++ namespaces to structurally separate the definitions.
This is automated with [<b>`wsdl2h -q name`</b> option <b>`-q name`</b>](#wsdl2h-q).
Or by creating libraries in C for the client/server objects as explained in
subsequent sections

Both approaches are demonstrated by examples in the gSOAP source code
package, the <i>`gsoap/samples/link`</i> (C only) and
<i>`gsoap/samples/link++`</i> (C++ with C++ namespaces) examples.

🔝 [Back to table of contents](#)

## How to build a client or server in a C++ code namespace        {#codenamespace}

You can use a C++ code namespace of your choice in your interface header file to build
a client or server in that C++ namespace. In this way, you can create multiple
clients and servers that can be combined and linked together without conflicts,
which is explained in more detail in the next section (which also shows an
example combining two client libraries defined in two C++ code namespaces).

Use [<b>`wsdl2h -q name`</b> option <b>`-q name`</b>](#wsdl2h-q) to generate definitions in the C++ `name` namespace. This option can also be used in combination with C++ proxy and server object generation, using <b>`soapcpp2 -j`</b> option <b>`-j`</b> or option <b>`-i`</b>).

At most one namespace can be defined for the entire interface header file for
soapcpp2. The C++ namespace must completely encapsulate the entire contents of
the header file:

~~~{.cpp}
    namespace NamespaceName {
      ... interface header file contents ... 
    }
~~~

When compiling this interface header file with the soapcpp2 tool, all type definitions,
the serializers for these types, and the stub and skeleton functions will be placed
in this namespace. The XML namespace mapping table (saved in a <i>`.nsmap`</i>
file) will not be placed in the code namespace to allow it to be linked as a
global object. You can use <b>`soapcpp2 -n`</b> option <b>`-n`</b> to create local XML namespace
tables, see Section \ref soapcpp2options  (but remember that you explicitly need to
initialize the namespaces to point to a table at run time using `::soap_set_namespaces`). The
generated files are prefixed with the code namespace name instead of the usual
<i>`soap`</i> file name prefix to enable multiple client/server codes to be build
in the same project directory.

Because the SOAP Header and Fault serializers will also be placed in
the namespace, they cannot be called from the <i>`gsoap/stdsoap2.cpp`</i> run time
library code and are therefore rendered unusable. Therefore, these serializers
are not compiled at all (enforced with `#define WITH_NOGLOBAL`). To add SOAP
Header and Fault serializers, you must compile them separately as follows.
First, create a new header file <i>`env.h`</i> with the SOAP Header
`::SOAP_ENV__Header` and SOAP Fault `::SOAP_ENV__Fault` structures, including `::SOAP_ENV__Detail` if this structure contains members that are serialized as fault details.  You can
leave this header file empty if you want to use the default SOAP Header and
Fault.  However, if SOAP Headers are required then you cannot leave the `::SOAP_ENV__Header` structure empty.  For example, the WS-Addressing and WS-Security plugins require SOAP Headers which can be imported by adding `#import "wsa5.h"` and `#import "wsse.h"`, respectively to <i>`env.h`</i>. Then compile this header file with:

     soapcpp2 -penv env.h

The generated <i>`envC.cpp`</i> file holds the SOAP Header and Fault serializers and you can
link this file with your client and server applications.

🔝 [Back to table of contents](#)

## How to create client/server libraries        {#dylibs}

The soapcpp2 tool produces <i>`soapClientLib.cpp`</i> and <i>`soapServerLib.cpp`</i>
source code files that are specifically intended for building static or dynamic
client and server libraries in C or C++. These files export the stub and skeleton functions, but keep
all serialization code static, thus hidden to avoid link symbol conflicts when combining multiple
clients and services into one executable. Note that it is far simpler to use
the wsdl2h tool on multiple WSDL files to generate a header file that
combines all service definitions. However, the approach presented in this
section is useful when creating (dynamic) libraries for client and server
objects, such as DLLs as described in Section \ref dll .

@note One major disadvantage of the approach presented here is that the serializer
functions are no longer accessible in the user's source code, because
serializers will be converted to static functions (to be used by the generated
stub and skeleton functions only).  For example, functions such as
`soap_new_T`, `soap_default_T`, `soap_write_T`, `soap_read_T`, `soap_put_T`,
and `soap_get_T` cannot be used any longer.

Using C++ namespaces is a better alternative when building applications in C++
that combine multiple clients and services.  To build multiple libraries,
you can define a C++ namespace in your header file with
<b>`soapcpp2 -qname`</b> option <b>`-qname`</b>, see Section \ref codenamespace
for details.

For C, you can use <b>`soapcpp2 -c -p name`</b> option <b>`-p name`</b> to
rename the generated <i>`soapClientLib.c`</i> and <i>`soapServerLib.c`</i> (and
associated) files. The <b>`-p name`</b> option specifies the file <i>`name`</i>
prefix to replace the <i>`soap`</i> file name prefix.

The engine does not define SOAP Header and Fault serializers that the engine
needs.  We therefore add SOAP Header and Fault serializers, which are compiled
separately as follows.  First, create a new header file <i>`env.h`</i> with the
SOAP Header `::SOAP_ENV__Header` and SOAP Fault `::SOAP_ENV__Fault` structures,
including `::SOAP_ENV__Detail` if this structure contains members that are
serialized as fault details.  You can leave this header file empty if you want
to use the default SOAP Header and Fault.  However, if SOAP Headers are
required then you cannot leave the `::SOAP_ENV__Header` structure empty.  For
example, the WS-Addressing and WS-Security plugins require SOAP Headers which
can be imported by adding `#import "wsa5.h"` and `#import "wsse.h"`,
respectively to <i>`env.h`</i>. Then compile this header file with:

     soapcpp2 -penv env.h

The generated <i>`envC.cpp`</i> file holds the SOAP Header and Fault serializers and you can
create a (dynamic) library for it to link it with your client and server applications.

You should then compile the <i>`gsoap/stdsoap2.cpp`</i> library with the compile-time `#WITH_NONAMESPACES` flag:

     c++ -DWITH_NONAMESPACES -c stdsoap2.cpp

This omits the reference to the global namespaces table, which is nowhere to be
defined since we will use XML namespaces for each client/service separately.
Therefore, you must explicitly set the namespaces value of the `::soap` context
in your code every time after initialization of the `::soap` context with the
`soap_set_namespaces(struct soap*, const struct Namespace*)` function.

For example, suppose we have two clients defined in header files
<i>`client1.h`</i> and <i>`client2.h`</i>. We first generate the
<i>`envH.h`</i> file for the SOAP Header and Fault definitions:

     soapcpp2 -c -penv env.h

Then we generate the code for client1 and client2:

     soapcpp2 -c -n -pmyClient1 client1.h
     soapcpp2 -c -n -pmyClient2 client2.h

This generates <i>`myClient1ClientLib.c`</i> and <i>`myClient2ClientLib.c`</i> (among several other files).
These two files should be compiled and linked with your application.
The source code of your application should include the generated <i>`envH.h`</i>, <i>`myClient1H.h`</i>, <i>`myClient2.h`</i> files and <i>`myClient1.nsmap`</i>, <i>`myClient2.nsmap`</i> files:

~~~{.cpp}
    #include "myClient1H.h" // include client 1 stubs 
    #include "myClient2H.h" // include client 2 stubs 
    #include "envH.h" 
    ... //
    #include "myClient1H.nsmap" // include client 1 nsmap 
    #include "myClient2H.nsmap" // include client 2 nsmap 

    struct soap *soap = soap_new();
    soap_set_namespaces(soap, myClient1_namespaces); 
    ... // make Client 1 invocations
    ... //
    soap_set_namespaces(soap, myClient2_namespaces); 
    ... // make Client 2 invocations
~~~

It is important to use <b>`soapcpp2 -n`</b> option <b>`-n`</b>, see Section
\ref soapcpp2options, to rename the namespace tables so we can include them all
without running into redefinitions.

@note Link conflicts may still occur in the unlikely situation that identical
service operation names are defined in two or more stub or skeleton functions
when these methods share the same XML namespace prefix.

🔝 [Back to table of contents](#)

### C++ examples   {#example13}

As an example we will build a Delayed Stock Quote client library and a Currency
Exchange Rate client library.

First, we create an empty header file <i>`env.h`</i>, which may be empty or
should contain SOAP Header and Fault definitions as explained in Section \ref
dylibs, and compile it as follows:

     soapcpp2 -penv env.h
     c++ -c envC.cpp

We also compile <i>`gsoap/stdsoap2.cpp`</i> without namespaces:

     c++ -c -DWITH_NONAMESPACES stdsoap2.cpp

If you do not use `#WITH_NONAMESPACES` then you will get an unresolved link
error for the global `namespaces` table. You can define a dummy table to avoid
having to recompile <i>`gsoap/stdsoap2.cpp`</i>.

Second, we create the Delayed Stock Quote header file specification, which may
be obtained using the WSDL importer. If you want to use C++ namespaces then you
need to manually add the `namespace` declaration to the generated header file:

~~~{.cpp}
    namespace quote {

    //gsoap ns service name: Service 
    //gsoap ns service style: rpc 
    //gsoap ns service encoding: encoded 
    //gsoap ns service location: http://services.xmethods.net/soap 
    //gsoap ns schema namespace: urn:xmethods-delayed-quotes 
    //gsoap ns service method-action: getQuote "" 
    int ns__getQuote(char *symbol, float &Result); 

    } // namespace quote
~~~

We then compile it as a library and we use option <b>`-n`</b> to prefix the generated files and to rename the namespace table to avoid link conflicts later:

     soapcpp2 -n quote.h
     c++ -c quoteClientLib.cpp

If you don't want to use a C++ code namespace, you should compile <i>`quote.h`</i> "as is" with option <b>`-pquote`</b>:

     soapcpp2 -n -pquote quote.h
     c++ -c quoteClientLib.cpp

Third, we create the Currency Exchange Rate header file specification:

~~~{.cpp}
    namespace rate {

    //gsoap ns service name: Service 
    //gsoap ns service style: rpc 
    //gsoap ns service encoding: encoded 
    //gsoap ns service location: http://services.xmethods.net/soap 
    //gsoap ns schema namespace: urn:xmethods-CurrencyExchange 
    //gsoap ns service method-action: getRate "" 
    int ns__getRate(char *country1, char *country2, float &Result); 

    } // namespace rate
~~~

Similar to the Quote example above, we generate source code using option <b>`-n`</b> to prefix the generated files and to rename the namespace table to avoid link conflicts:

     soapcpp2 -n rate.h

Fourth, we use the generated source code libraries with the main program.

~~~{.cpp}
    #include "quoteH.h.h"
    #include "rateH.h.h"
    #include "quote.nsmap"
    #include "rate.nsmap"

    int main(int argc, char *argv[]) 
    {
      if (argc <= 1) 
      {
        std::cerr << "Usage: main ticker [currency]" << std::endl 
        exit(EXIT_FAILURE); 
      } 
      struct soap *soap = soap_new();
      float q; 
      soap_set_namespaces(soap, quote_namespaces); 
      if (soap_call_ns__getQuote(soap, NULL, NULL, argv[1], q)) // get quote 
      {
        soap_print_fault(soap, stderr); 
      }
      else 
      {
        if (argc > 2) 
        {
          float r; 
          soap_set_namespaces(soap, rate_namespaces); 
          if (soap_call_ns__getRate(soap, NULL, NULL, "us", argv[2], r)) // get rate in US dollars 
            soap_print_fault(soap, stderr); 
          else 
            q *= r; // convert the quote 
        } 
        std::cout << argv[1] << ": " << q << std::endl; 
      } 
      soap_destroy(soap);
      soap_end(soap);
      soap_free(soap);
      return 0; 
    }
~~~

Compile and link this application with <i>`stdsoap2.o`</i>, <i>`envC.o`</i>, <i>`quoteServerProxy.o`</i>, and <i>`rateServerProxy.o`</i>.

Instead of the generated `soap_call_webmethod` stub functions you can also use <b>`soapcpp2 -j`</b> option <b>`-j`</b> or option <b>`-i`</b> to generate C++ proxy classes.

To compile server is very similar. For example, assume that we need to implement a calculator service and we want to create a library for it.

~~~{.cpp}
    namespace calc {

    //gsoap ns service name: Calc 
    //gsoap ns service style: rpc 
    //gsoap ns service encoding: encoded 
    //gsoap ns service location: http://www.cs.fsu.edu/~engelen/calc.cgi 
    //gsoap ns schema namespace: urn:calc 
    int ns__add(double a, double b, double &result); 
    int ns__sub(double a, double b, double &result); 
    int ns__mul(double a, double b, double &result); 
    int ns__div(double a, double b, double &result); 

    } // namespace calc
~~~

We generate code:

     soapcpp2 -j -n calc.h

Here we used option <b>`-j`</b> to generate a C++ service class <i>`calcCalcService.h`</i> and <i>`calcCalcService.cpp`</i>. The effect of the <b>`-n`</b> option is that it creates local namespace tables and uses <i>`calc`</i> to prefix the generated files.

~~~{.cpp}
    #include "calcCalcService.h"
    #include "calc.nsmap"

    int main()
    {
      calc::Calc calc; 
      ... //
      calc.serve(); // calls request dispatcher to invoke one of the functions below 
      ... //
    }

    int calc::Calc::add(double a, double b, double &result)
    {
      result = a + b;
      return SOAP_OK;
    } 

    int calc::Calc::sub(double a, double b, double &result)
    {
      result = a - b;
      return SOAP_OK;
    } 

    int calc::Calc::mul(double a, double b, double &result)
    {
      result = a * b;
      return SOAP_OK;
    } 

    int calc::Calc::div(double a, double b, double &result)
    {
      result = a / b;
      return SOAP_OK;
    } 
~~~

The example above serves requests over stdin/out. Use the bind and accept calls to create a stand-alone server to service inbound requests over sockets, see Section \ref stand-alone .

🔝 [Back to table of contents](#)

### C examples    {#example14}

This is the same example as above, but the clients are build in C.

We create a <i>`env.h`</i> that contains the joint SOAP Header and SOAP Fault
definitions. You may have to copy-paste these from the other header files.
Then, compile it as follows:

     soapcpp2 -c -penv env.h
     cc -c envC.c

We also compile <i>`gsoap/stdsoap2.c`</i> without namespaces:

     cc -c -DWITH_NONAMESPACES stdsoap2.c

Second, we create the Delayed Stock Quote header file specification, which may be obtained using the WSDL importer.

~~~{.cpp}
    //gsoap ns service name: Service 
    //gsoap ns service style: rpc 
    //gsoap ns service encoding: encoded 
    //gsoap ns service location: http://services.xmethods.net/soap 
    //gsoap ns schema namespace: urn:xmethods-delayed-quotes 
    //gsoap ns service method-action: getQuote "" 
    int ns__getQuote(char *symbol, float *Result);
~~~

We compile it as a library and we use options <b>`-n`</b> and <b>`-pquote`</b> to prefix the generated files and to rename the namespace table to avoid link conflicts:

     soapcpp2 -c -n -pquote quote.h
     cc -c quoteClientLib.c

Third, we create the Currency Exchange Rate header file specification:

~~~{.cpp}
    //gsoap ns service name: Service 
    //gsoap ns service style: rpc 
    //gsoap ns service encoding: encoded 
    //gsoap ns service location: http://services.xmethods.net/soap 
    //gsoap ns schema namespace: urn:xmethods-CurrencyExchange 
    //gsoap ns service method-action: getRate "" 
    int ns__getRate(char *country1, char *country2, float *Result);
~~~

We compile it as a library and we use options <b>`-n`</b> and <b>`-prate`</b> to prefix the generated files and to rename the namespace table to avoid link conflicts:

     soapcpp2 -c -n -prate rate.h
     cc -c rateClientLib.c

The main program is:

~~~{.cpp}
    #include "quoteH.h"
    #include "rateH.h"
    #include "quote.nsmap"
    #include "rate.nsmap"

    int main(int argc, char *argv[]) 
    {
      if (argc <= 1) 
      {
        fprintf(stderr, "Usage: main ticker [currency]\n"); 
        exit(EXIT_FAILURE); 
      } 
      struct soap *soap = soap_new(); 
      float q; 
      soap_set_namespaces(soap, quote_namespaces); 
      if (soap_call_ns__getQuote(soap, NULL, NULL, argv[1], &q)) // get quote 
        soap_print_fault(soap, stderr); 
      else 
      {
        if (argc > 2) 
        {
          soap_set_namespaces(soap, rate_namespaces); 
          float r; 
          if (soap_call_ns__getRate(soap, NULL, NULL, "us", argv[2], &r)) // get rate in US dollars 
            soap_print_fault(soap, stderr); 
          else 
            q *= r; // convert the quote 
        } 
        printf("%s: %f \n", argv[1], q); 
      } 
      soap_destroy(soap);
      soap_end(soap);
      soap_free(soap);
      return 0; 
    }
~~~

Compile and link this application with <i>`stdsoap2.o`</i>, <i>`envC.o`</i>, <i>`quoteClientLib.o`</i>, and <i>`rateClientLib.o`</i>.

🔝 [Back to table of contents](#)

### How to chain C services to accept messages on the same port        {#example15}

When combining multiple services into one application, you can run wsdl2h
on multiple WSDLs to generate the single all-inclusive service definitions
interface header file for soapcpp2. This header file is then processed with soapcpp2 to
generate skeleton functions in C.

What if we generate multiple services, each from a WSDL separately, and want to deploy them on the same port?  This requires listening to the same port and then chaining the service dispatches so that each service can serve a request.

First we create a <i>`env.h`</i> that contains the joint SOAP Header and SOAP Fault
definitions, for example by copy-pasting these from the other header files generated by wsdl2h.  Or this file is empty if no specialized SOAP Headers and Faults are used.
We compile it as follows:

     soapcpp2 -c -penv env.h
     cc -c envC.c

We also compile <i>`gsoap/stdsoap2.c`</i> without namespaces:

     cc -c -DWITH_NONAMESPACES stdsoap2.c

Say for example that we have a service definition in <i>`quote.h`</i>. We compile it as a library and we use options <b>`-n`</b> and <b>`-pquote`</b> to prefix the generated files and to rename the namespace table to avoid link conflicts:

     soapcpp2 -c -n -pquote quote.h
     cc -c quoteClientLib.c

We do the same for a service definition in <i>`rate.h`</i>:

     soapcpp2 -c -n -prate rate.h
     cc -c rateClientLib.c

To serve both the quote and rate services on the same port, we chain the service dispatchers as follows:

~~~{.cpp}
    #include "quoteH.h"
    #include "rateH.h"
    #include "quote.nsmap"
    #include "rate.nsmap"

    struct soap *soap = soap_new(); 
    if (soap_valid_socket(soap_bind(soap, NULL, 8080, 10))) // small BACKLOG for iterative servers
    {
      while (1)
      {
        if (soap_valid_socket(soap_accept(soap)))
        {
          if (soap_begin_serve(soap)) 
          {
            soap_print_fault(soap, stderr); 
            continue;
          }
          soap_set_namespaces(soap, quote_namespaces);
          if (quote_serve_request(soap) == SOAP_NO_METHOD) 
          {
            soap_set_namespaces(soap, rate_namespaces);
            if (rate_serve_request(soap))
              soap_send_fault(soap); // send fault to client 
          } 
          else if (soap->error) 
          {
            soap_send_fault(soap); // send fault to client 
          }
        }
        else if (soap->errnum) // accept failed, try again after 1 second
        {
          soap_print_fault(soap, stderr); 
          sleep(1);
        } 
        else
        {
          fprintf(stderr, "server timed out\n"); 
          break; 
        }
        soap_destroy(soap); 
        soap_end(soap); 
      }
    }
    soap_destroy(soap); 
    soap_end(soap); 
    soap_free(soap);
~~~

This chaining can be arbitrarily deep. When the previous request fails with a `#SOAP_NO_METHOD` then next request dispatcher can be tried.

The server should also define the service operations:

~~~{.cpp}
    int ns__getQuote(struct soap *soap, char *symbol, float *Result); 
    {
      *Result = ... ; 
      return SOAP_OK; 
    } 

    int ns__getRate(struct soap *soap, char *country1, char *country2, float *Result); 
    {
      *Result = ... ; 
      return SOAP_OK; 
    }
~~~

However, the while loop iterates for each new connection that is established with `::soap_accept` and does not allow for HTTP keep-alive connections to persist.  For our final improvement we want to support HTTP keep-alive connections that require looping over the service dispatches until the connection closes on either end, after which we resume the outer loop.  The resulting code is very close to the soapcpp2-generated `::soap_serve` code and the `serve` service class methods, with the addition of the chain of service dispatches in the loop body:

~~~{.cpp}
    #include "quoteH.h"
    #include "rateH.h"
    #include "quote.nsmap"
    #include "rate.nsmap"

    struct soap *soap = soap_new(); 
    if (soap_valid_socket(soap_bind(soap, NULL, 8080, 10))) // small BACKLOG for iterative servers
    {
      while (1)
      {
        if (soap_valid_socket(soap_accept(soap)))
        {
          soap->keep_alive = soap->max_keep_alive + 1; // max keep-alive iterations
          do
          {
            if ((soap->keep_alive > 0) && (soap->max_keep_alive > 0))
              soap->keep_alive--;
            if (soap_begin_serve(soap))
            {
              if (soap->error >= SOAP_STOP) // if a plugin has served the request
                continue;                   // then continue with the next request
              break;                        // an error occurred
            }
            soap_set_namespaces(soap, quote_namespaces);
            if (quote_serve_request(soap) == SOAP_NO_METHOD) 
            {
              soap_set_namespaces(soap, rate_namespaces);
              if (rate_serve_request(soap))
                soap_send_fault(soap); // send fault to client 
            } 
            else if (soap->error) 
            {
              soap_send_fault(soap); // send fault to client 
            }
            soap_destroy(soap); 
            soap_end(soap); 
          } while (soap->keep_alive);
        }
        else if (soap->errnum) // accept failed, try again after 1 second
        {
          soap_print_fault(soap, stderr); 
          sleep(1);
        } 
        else
        {
          fprintf(stderr, "server timed out\n"); 
          break; 
        }
        soap_destroy(soap); 
        soap_end(soap); 
      }
    }
    soap_destroy(soap); 
    soap_end(soap); 
    soap_free(soap);
~~~

🔝 [Back to table of contents](#)

## How to create DLLs        {#dll}

🔝 [Back to table of contents](#)

### Creating the base stdsoap2.dll  {#basedll}

The engine does not define SOAP Header and Fault serializers that the engine needs when installed as a library.  We therefore
add SOAP Header and Fault serializers, which are compiled
separately as follows.
First, create a new header file <i>`env.h`</i> with the SOAP Header
`::SOAP_ENV__Header` and SOAP Fault `::SOAP_ENV__Fault` structures, including `::SOAP_ENV__Detail` if this structure contains members that are serialized as fault details.  You can
leave this header file empty if you want to use the default SOAP Header and
Fault.  However, if SOAP Headers are required then you cannot leave the `::SOAP_ENV__Header` structure empty.  For example, the WS-Addressing and WS-Security plugins require SOAP Headers which can be imported by adding `#import "wsa5.h"` and `#import "wsse.h"`, respectively to <i>`env.h`</i>. Then compile this header file with:

     soapcpp2 -penv env.h

The generated <i>`envC.cpp`</i> file holds the SOAP Header and Fault serializers.  We can either create a separate <i>`envC.dll`</i> DLL for this that all clients and service applications will use, or combine <i>`envC.cpp`</i> with the  <i>`stdsoap2.dll`</i> we create, which we will explain further.

The next step is to create <i>`stdsoap2.dll`</i> which consists of the file
<i>`gsoap/stdsoap2.cpp`</i> and <i>`envC.cpp`</i> and optionally the plugins you want to use such as <i>`wsseapi.cpp`</i> (we need to rename all <i>`.c`</i> files to <i>`.cpp`</i> files to avoid issues with MSVC++). This DLL contains all common functions
needed for all other clients and servers based on gSOAP.  Compile
<i>`envC.cpp`</i> and <i>`gsoap/stdsoap2.cpp`</i> into <i>`stdsoap2.dll`</i> using the
compiler option `/D` `#WITH_NONAMESPACES` and the MSVC Pre-Processor
definitions `SOAP_FMAC1=__declspec(dllexport)`, `SOAP_FMAC3=__declspec(dllexport)`, and the `#SOAP_STD_EXPORTS` macro set as shown below from the MSVC command prompt:

    C:> cl /c /I. /EHsc /DWITH_NONAMESPACES /DSOAP_FMAC1=__declspec(dllexport) /DSOAP_FMAC3=__declspec(dllexport) /DSOAP_STD_EXPORTS envC.cpp stdsoap2.cpp
    C:> link /LIBPATH ws2_32.lib /OUT:mygsoap.dll /DLL envC.obj stdsoap2.obj

Note: as of gSOAP 2.8.30 and later, the DLL export macros shown here are all set with one pre-processor definition `#SOAP_STD_EXPORTS`.

Alternatively, you can compile with
`/D` `#WITH_SOAPDEFS_H` and put the macro definitions in <i>`soapdefs.h`</i>.
This exports all functions which are preceded by the macro `#SOAP_FMAC1` in
the <i>`soapcpp2.cpp`</i> source file and macro `#SOAP_FMAC3` in the <i>`envC.cpp`</i> source file.

Finally, note that the gSOAP source code package contains a lot of <i>`.c`</i>
source code files.  Mixing C with C++ files is not recommended with Visual Studio and
will lead to run-time errors when building DLLs.  Therefore, always rename <i>`.c`</i>
source code files to <i>`.cpp`</i> source code files when creating DLLs.

🔝 [Back to table of contents](#)

### Creating client and server DLLs   {#clientserverdll}

Compile the <i>`soapClientLib.cpp`</i> and <i>`soapServerLib.cpp`</i> sources as DLLs
by using the MSVC Pre-Processor definitions `SOAP_FMAC5=__declspec(dllexport)` and `SOAP_CMAC=__declspec(dllexport)`, and by using the C++ compiler option `/D` `#WITH_NONAMESPACES`. All of these macros are set as a shorthand with one pre-processor definition `#SOAP_STD_EXPORTS` (requires gSOAP 2.8.30 or later).

This DLL links to <i>`stdsoap2.dll`</i> we created in Section \ref basedll.

To create multiple DLLs in the same project directory, you should use <b>`soapcpp2 -p name`</b> option
<b>`-p name`</b> to rename the generated <i>`soapClientLib.cpp`</i> and
<i>`soapServerLib.cpp`</i> (and associated) files. The <b>`-p name`</b> option specifies
a <i>`name`</i> prefix to replace the <i>`soap`</i> file name prefix.
Another way is to use C++ namespaces with  <b>`soapcpp2 -q name`</b> option
<b>`-q name`</b>, if the interface header file input to soapcpp2 does not already declare a C++ namespace.
A clean separation of libraries can also be achieved with C++ namespaces, see Section \ref codenamespace .

Unless you use the client proxy and server object classes (<i>`soapXYZProxy.h`</i> and <i>`soapXYZService.h`</i>), all client and server applications must explicitly set the namespaces value of the `::soap` context
with:

~~~{.cpp}
    soap_set_namespaces(soap, namespaces);
~~~

where the `namespaces[]` table should be defined in the client/server source. These tables are generated in the <i>`.nsmap`</i> files. The tables are renamed for convenience using <b>`soapcpp2 -n`</b> option <b>`-n`</b>, see Section \ref soapcpp2options .

🔝 [Back to table of contents](#)

## How to build modules and libraries with the #module directive        {#module}

The `#module` directive is used to build modules with soapcpp2. A library can
be built from a module and linked with multiple Web services applications. The
directive should appear at the top of the interface header file for soapcpp2
and has the following formats:

~~~{.cpp}
    #module "name" 
~~~

and

~~~{.cpp}
    #module "name" "fullname"
~~~

where the `name` must be a unique short name for the module. The name is case
insensitive and must not exceed 4 characters in length. The `fullname`, when
present, represents the full name of the module and is used to prefix the
function names of the generated serializers replacing the usual `soap` prefix.
If absent, the short name is used to prefix the function names of the
serializers.

The rest of the content of the interface header file includes type declarations
and optionally the declarations of service operations and SOAP Headers and
Faults that are universally used by SOAP services, when applicable. When the
soapcpp2 tool processes the header file module, it will generate the source
codes for a library. The Web services application that uses the library should
use a header file that imports the module with the `#import` directive, for
example:

~~~{.cpp}
    /* Contents of file "module.h" */ 
    #module "test"

    // types and typedefs become module-specific
    typedef LONG64 xsd__long;
    char*;

    // a module-specific struct
    struct ns__S 
    {
        ... // members
    };
~~~

The <i>`module.h`</i> data binding interface header file for soapcpp2 declares
module-specific serializable types `LONG64`, `xsd__long`, `char*`, and a
`struct ns__S`. The module name is "test", so the soapcpp2 tool produces a
<i>`testC.cpp`</i> file with the XML serializers for these types. The
<i>`testC.cpp`</i> data binding implementation source code can be separately
compiled and linked with an application. If service operations are declared in
the interface header file as function prototypes, we also get
<i>`testClient.cpp`</i> client stub functions and <i>`testServer.cpp`</i>
server skeleton functions.

There are some limitations for module imports:

- A module must be imported into another interface header to use the module's
  type definitions.
- When multiple modules are imported, the types that they declare must be
  declared in one module only to avoid name clashes and link errors. You cannot
  create two modules that declare or use the same type and import these modules
  separately into another header file. When using modules, consider creating a
  module hierarchy such that types are declared only once and by only one
  module when these modules must be linked.

With modules, the source code serializers for the types defined in the modules
are generated with soapcpp2 separately.  For example, assume that we have a
module <i>`module.h`</i>:

~~~{.cpp}
    /* Contents of file "module.h" */
    #module "test"

    struct ns__S
    {
        char *name;
        int amount;
    };
~~~

and a header file <i>`example.h`</i> that uses it:

~~~{.cpp}
    /* Contents of file "example.h" */
    #import "module.h"

    int ns__webmethod(const char *code, struct ns__S *record, int *result);    
~~~

The module is compiled as follows:

    soapcpp2 module1.h

This generates the files <i>`testStub.h`</i>, <i>`testH.h`</i>, and
<i>`testC.cpp`</i> with serializers for `struct ns__S` but also for `char*` and
`int`.

Running soapcpp2 on <i>`example.h`</i> imports the module definitions, but does
not generate serializers for `struct ns__S`, `char*` and `int` since these are
defined by <i>`module.h`</i>:

    soapcpp2 -CL example.h

An example client application that calls the client stub function:

~~~{.cpp}
    #include "soapH.h"
    #include "ns.nsmap"

    int main()
    {
      struct soap *soap = soap_new();
      struct ns__S s;
      soap_default_ns__S(soap, &s);
      s.name = soap_strdup(soap, "name");
      s.amount = 124;
      int n;
      if (soap_call_ns__webmethod(soap, endpoint, NULL, "code", &s, &n))
        soap_print_fault(soap, stderr);
      else
        printf("OK n = %d\n", n);
      soap_destroy(soap);
      soap_end(soap);
      soap_free(soap);
    }
~~~

We compile this example as follows:

    c++ -o example example.cpp testC.cpp soapC.cpp soapClient.cpp stdsoap2.cpp

Modules may help to define a modular hierarchy of libraries with reusable
serializable types.

🔝 [Back to table of contents](#)

## Plugins, modules, and extensions        {#plugins}

Plugins offer a convenient extension mechanism for the gSOAP toolkit by
extending the capabilities of its engine.  When the plugin registers with the
gSOAP engine, it has full access to the run-time settings and the engine's
function callbacks.  Upon registry, the plugin's local data is associated with
the `::soap` context.  By overriding the callbacks with the plugin's callbacks,
the plugin extends or modifies the engine's capabilities. The local plugin data
can be accessed through a lookup function, usually invoked within a callback
function to access the plugin data.  The registry and lookup functions are:

~~~{.cpp}
    int soap_register_plugin_arg(struct soap *soap, int (*fcreate)(struct soap *soap, struct soap_plugin *p, void *arg), void *arg)
~~~
~~~{.cpp}
    void* soap_lookup_plugin(struct soap*, const char*);
~~~

The `::soap_copy` function returns a new dynamically allocated `::soap`
context that is a copy of another, such that no data is shared between the
copy and the original context. The `::soap_copy` function invokes the
copy functions of the registered plugins to copy the plugins' local data.
The `::soap_done` and `::soap_free` functions de-registers all plugin.

The follow example overrides the send and receive callbacks to copy all messages
that are sent and received to the terminal (stderr).

First, we write a header file <i>`plugin.h`</i> to define the local plugin data
structure(s) and we define a global name to identify the plugin:

~~~{.cpp}
    #include "stdsoap2.h" 
    #define PLUGIN_ID "PLUGIN-1.0" // some name to identify plugin 

    struct plugin_data // local plugin data 
    {
      int (*fsend)(struct soap*, const char*, size_t); // to save and use send callback 
      size_t (*frecv)(struct soap*, char*, size_t); // to save and use recv callback 
    }; 
    int plugin(struct soap *soap, struct soap_plugin *plugin, void *arg);
~~~

Then, we write the plugin registry function and the callbacks:

~~~{.cpp}
    #include "plugin.h" 

    static const char plugin_id[] = PLUGIN_ID; // the plugin id 
    static int plugin_init(struct soap *soap, struct plugin_data *data); 
    static int plugin_copy(struct soap *soap, struct soap_plugin *dst, struct soap_plugin *src); 
    static void plugin_delete(struct soap *soap, struct soap_plugin *p); 
    static int plugin_send(struct soap *soap, const char *buf, size_t len); 
    static size_t plugin_recv(struct soap *soap, char *buf, size_t len); 

    // the registry function: 
    int plugin(struct soap *soap, struct soap_plugin *p, void *arg) 
    {
      p->id = plugin_id; 
      p->data = (void*)malloc(sizeof(struct plugin_data)); 
      p->fcopy = plugin_copy; /* optional: when set the plugin must copy its local data */
      p->fdelete = plugin_delete; 
      if (p->data) 
      {
        if (plugin_init(soap, (struct plugin_data*)p->data)) 
        {
          free(p->data); // error: could not init 
          return SOAP_EOM; // return error 
        } 
      }
      return SOAP_OK; 
    } 

    static int plugin_init(struct soap *soap, struct plugin_data *data) 
    {
      data->fsend = soap->fsend; // save old recv callback 
      data->frecv = soap->frecv; // save old send callback 
      soap->fsend = plugin_send; // replace send callback with new 
      soap->frecv = plugin_recv; // replace recv callback with new 
      return SOAP_OK; 
    } 

    // copy plugin data, called by soap_copy()
    // This is important: we need a deep copy to avoid data sharing by two contexts 
    static int plugin_copy(struct soap *soap, struct soap_plugin *dst, struct soap_plugin *src) 
    {
      if (!(dst->data = (struct plugin_data*)malloc(sizeof(struct plugin_data)))) 
        return SOAP_EOM; 
      *dst->data = *src->data; 
      return SOAP_OK; 
    } 

    // plugin deletion, called by soap_done() 
    static void plugin_delete(struct soap *soap, struct soap_plugin *p) 
    {
      free(p->data); // free allocated plugin data 
    } 

    // the new send callback 
    static int plugin_send(struct soap *soap, const char *buf, size_t len) 
    {
      struct plugin_data *data = (struct plugin_data*)soap_lookup_plugin(soap, plugin_id); // fetch plugin's local data 
      fwrite(buf, len, 1, stderr); // write message to stderr 
      return data->fsend(soap, buf, len); // pass data on to old send callback 
    } 

    // the new receive callback 
    static size_t plugin_recv(struct soap *soap, char *buf, size_t len) 
    {
      struct plugin_data *data = (struct plugin_data*)soap_lookup_plugin(soap, plugin_id); // fetch plugin's local data 
      size_t res = data->frecv(soap, buf, len); // get data from old recv callback 
      fwrite(buf, res, 1, stderr); 
      return res; 
    }
~~~

It is the responsibility of the plugin to
handle registry (init), copy, and deletion of the plugin data and reset callbacks.

The `fdelete` callback of `struct soap_plugin`
must be set to de-register the plugin and let it delete its resources. 

A plugin is copied along with its corresponding `::soap` context with the `::soap_copy` call. This function copies a `::soap` context
and the chain of plugins. It is up to the plugin implementation to share the plugin data
or not as follows: 

*  if the `fcopy()` callback is set by the plugin initialization, this callback
   will be called to allow the plugin to copy its local data upon a
   `::soap_copy` call. When `::soap_done` or `::soap_free` is called on this
   `::soap` context copy, the `fdelete()` callback is called for deallocation
   and cleanup of the local data.

* if the `fcopy()` callback is not set, then the plugin data
  will be shared (i.e. the data pointer points to the same address).  The
  `fdelete()` callback will not be called upon a `::soap_done` on a copy of the
  `::soap` context . The `fdelete()` callback will be called for the original
  `::soap` context with which the plugin was registered.

The example plugin should be used as follows:

~~~{.cpp}
    struct soap *soap = soap_new(); 
    soap_register_plugin(soap, plugin); 
~~~

To pass a `void*` argument to the plugin's registry function use:

~~~{.cpp}
    struct soap *soap = soap_new(); 
    soap_register_plugin_arg(soap, plugin, arg); 
~~~

Additional documentation for the growing number of gSOAP plugins can be found
at <https://www.genivia.com/doc>.  A number of example plugins are included in
the gSOAP source code package's <i>`gsoap/plugin`</i> directory. Some of these plugins are
discussed in the next sections.

See also API documentation Module \ref group_plugin .

🔝 [Back to table of contents](#)

### DOM API overview {#dom}

The DOM API is not a plugin, but an extension that provides a DOM API as a
separate source code file to compile and link with gSOAP applications to
enabled the gSOAP DOM.  XML DOM processing is optional, and enabled with
[<b>`wsdl2h -d`</b> option <b>`-d`</b>](#wsdl2h-d) to generate DOM structures for
<i>`xsd:anyType`</i>, <i>`xsd:any`</i>, and <i>`xsd:anyAttribute`</i> schema
components, which are enabled with `#import "dom.h"` in an interface header
file for soapcpp2.  This imports <i>`gsoap/import/dom.h`</i>.  Then compile
<i>`gsoap/dom.c`</i> for C or <i>`gsoap/dom.cpp`</i> for C++ with your
application.

To use the DOM API with Web services, add `#import "dom.h"` to the interface
header file or use [<b>`wsdl2h -d`</b> option <b>`-d`</b>](#wsdl2h-d):

~~~{.cpp}
    #import "dom.h"
~~~

This declares `xsd__anyType` and `xsd__anyAttribute` types.

A DOM element node is serialized with the `xsd__anyType` serializable type.
The underlying implementation type of `xsd__anyType` is `soap_dom_element`.
One or more DOM attribute nodes are serialized with the `xsd__anyAttribute`
serializable type.  The underlying implementation type of `xsd__anyAttribute`
is `soap_dom_attribute`, which is a linked list.

~~~{.cpp}
    #import "dom.h" // imports xsd__anyType as a DOM node

    class ns__data : public xsd__anyType
    { public:
        xsd__anyType* foo;                // Store <foo> element in DOM soap_dom_element
        xsd__anyType __any;	          // Store any element content in DOM soap_dom_element
      @ xsd__anyAttribute __anyAttribute; // Store anyAttribute content in DOM soap_dom_attribute linked node structure
    };
~~~

To manipulate the DOM elements and attributes we use the DOM API functions
documented in the gSOAP [XML DOM API documentation](../../dom/html/index.html).

🔝 [Back to table of contents](#)

### The message logging plugin   {#loggingplugin}

The message `::logging` plugin can be used to selectively log inbound and outbound messages to a file or stream. It also keeps access statistics to log the total number of bytes sent and received.

To use the plugin, compile and link your application with <i>`logging.c`</i> located in the <i>`gsoap/plugin`</i> directory of the gSOAP package.
To enable the plugin in your code, register the plugin and set the streams as follows:

~~~{.cpp}
    #include "logging.h" 

    struct soap *soap = soap_new();
    size_t bytes_in; 
    size_t bytes_out; 
    if (soap_register_plugin(soap, logging)) 
      ... // failed to register 
    ... //
    soap_set_logging_inbound(soap, stdout); 
    soap_set_logging_outbound(soap, stdout); 
    ... // process messages
    soap_set_logging_inbound(soap, NULL); // disable logging 
    soap_set_logging_outbound(soap, NULL); // disable logging 
    soap_logging_stats(soap, &bytes_out, &bytes_in);
    ... //
    soap_reset_logging_stats(soap);
~~~

If you use `::soap_copy` to copy the `::soap` context with the plugin, the plugin's state will be shared by the copy.
The plugin is thread-safe, but does not use a lock to protect the internal statistics counters to ensure the speed of messaging is not compromised, meaning that you should not fully rely on the statistics to be 100% accurate for multi-threaded services.

See also `::logging`.

🔝 [Back to table of contents](#)

### RESTful server-side API with the HTTP GET plugin  {#RESTfulserverGET}

Server-side use of RESTful HTTP GET operations is supported with the `::http_get` HTTP GET
plugin <i>`gsoap/plugin/httpget.c`</i>.

The HTTP GET plugin allows your server to handle RESTful HTTP GET requests and
at the same time still serve SOAP-based POST requests. The plugin provides
support to client applications to issue HTTP GET operations to a server.

Note that HTTP GET requests can also be handled at the server side with the
`::soap::fget` callback, see Section \ref callback . However, the HTTP GET
plugin also keeps statistics on the number of successful POST and GET
exchanges and failed operations (HTTP faults, SOAP Faults, etc.). It also keeps
hit histograms accumulated for up to a year of running time.

To use the `::http_get` plugin, compile and link your application with <i>`httpget.c`</i> located in the <i>`gsoap/plugin`</i> directory of the gSOAP package.
To enable the plugin in your code, register the plugin with your HTTP GET handler function as follows:

~~~{.cpp}
    #include "httpget.h"

    int main()
    {
      struct soap *soap = soap_new();
      if (soap_register_plugin_arg(soap, http_get, (void*)my_http_get_handler)) 
        soap_print_fault(soap, stderr); // failed to register 
      ... //
      struct http_get_data *httpgetdata; 
      httpgetdata = (struct http_get_data*)soap_lookup_plugin(soap, http_get_id); 
      if (!httpgetdata) 
        ... // if the plugin registered OK, there is certainly data but can't hurt to check 
      ... // process messages
      size_t get_ok = httpgetdata->stat_get; 
      size_t post_ok = httpgetdata->stat_post; 
      size_t errors = httpgetdata->stat_fail; 
      time_t now = time(NULL); 
      struct tm *T; 
      T = localtime(&now); 
      size_t hitsthisminute = httpgetdata->hist_min[T->tm_min]; 
      size_t hitsthishour = httpgetdata->hist_hour[T->tm_hour]; 
      size_t hitstoday = httpgetdata->hist_day[T->tm_yday];
    }
~~~

An HTTP GET handler can simply produce some HTML content, or any other type of content by sending data using `::soap_response`:

~~~{.cpp}
    int my_http_get_handler(struct soap *soap) 
    {
      soap->http_content = "text/html"; 
      if (soap_response(soap, SOAP_FILE)
       || soap_send(soap, "<html>Hello</html>")
       || soap_end_send(soap)
        return soap_closesock(soap);
      return SOAP_OK; // return SOAP_OK or HTTP error code, e.g. 404 
    }
~~~

If you use `::soap_copy` to copy the `::soap` context with the plugin, the plugin's data will be shared by the copy.
Therefore, the statistics are not 100% guaranteed to be accurate for multi-threaded services since race conditions on the counters may occur. Mutex is not used to update the counters to avoid introducing expensive synchronization points. If 100% server-side accuracy is required, add mutex at the points indicated in the <i>`httpget.c`</i> code.

The client-side use of HTTP GET is provided by the `::soap_GET` operation. To receive a SOAP/XML (response) message <i>`ns:methodResponse`</i>, use:

~~~{.cpp}
    int main()
    {
      struct soap *soap = soap_new();
      if (soap_GET(soap, endpoint, NULL)) 
        ... // error
      else if (soap_recv_ns__webmethodResponse(soap, ...)) 
        ... // error
      else 
        ... // success
    soap_destroy(soap); 
    soap_end(soap); 
    soap_done(soap);
~~~

To receive any HTTP Body data into a buffer, use:

~~~{.cpp}
    int main()
    {
      struct soap *soap = soap_new();
      char *response = NULL; 
      size_t response_len;
      if (soap_GET(soap, endpoint, NULL)
       || (response = soap_http_get_body(soap, &response_len)) == NULL
       || soap_end_recv(&soap))
        ... // error
      else
        ... // use the response string (NULL indicates no body or error)
      soap_destroy(&soap); 
      soap_end(&soap); 
      soap_done(&soap);
    }
~~~

See also `::http_get`.

🔝 [Back to table of contents](#)

### RESTful server-side API with the HTTP POST plugin   {#RESTfulservicePOST}

Server-side use of RESTful HTTP POST, PUT, PATCH, and DELETE operations are supported
with the `::http_post` HTTP POST plugin <i>`gsoap/plugin/httppost.c`</i>.

The `::http_post` HTTP POST plugin allows your server to handle RESTful HTTP POST requests
and at the same time still serve SOAP-based POST requests. The plugin also
provides support for client applications to issue HTTP POST operations to a
server.

To simplify the server-side handling of POST requests, handlers can be associated with media types:

~~~{.cpp}
    struct http_post_handlers my_handlers[] = 
    {
      { "image/jpg", jpeg_handler }, 
      { "image/*",   image_handler }, 
      { "text/html", html_handler }, 
      { "text/*",    text_handler }, 
      { "POST",      generic_POST_handler }, 
      { "PUT",       generic_PUT_handler }, 
      { "PATCH",     generic_PATCH_handler }, 
      { "DELETE",    generic_DELETE_handler }, 
      { NULL } 
    };
~~~

Note that `*` and `-` can be used as wildcards to match any text and any
character, respectively.  Media types may have optional parameters after `;`
such as `charset` and `boundary`.  These parameters can be matched by the media
type patterns in the table.  Patterns that are more specific must precede
patterns that are less specific in the table.  For example,
`"text/xml;*charset=utf-8*"` must precede `"text/xml"` which must precede
`"text/*"`.  Note that `"text/xml"` also matches any parameters of the media
type of the message reveived, such as `"text/xml; charset=utf-8"` (only
since gSOAP version 2.8.75).

The handlers are functions that will be invoked when a POSTed request message
matching media type is sent to the server.

An example image handler that checks the specific image type:

~~~{.cpp}
    int image_handler(struct soap *soap) 
    {
      const char *buf; 
      size_t len; 
      // if necessary, check type in soap->http_content 
      if (soap->http_content && !soap_tag_cmp(soap->http_content, "image/gif") 
        return 404; // HTTP error 404 
      if ((buf = soap_http_get_body(soap, &len)) == NULL)
        return soap->error; 
      // ... now process image in buf 
      // reply with empty HTTP 200 OK response: 
      return soap_send_empty_response(soap, 200); 
    }
~~~

The above example returns HTTP OK. If content is supposed to be returned, then use:

~~~{.cpp}
    struct soap *soap = soap_new();
    ... //
    soap->http_content = "image/jpeg"; // a jpeg image to return back 
    if (soap_response(soap, SOAP_FILE)) // SOAP_FILE sets custom http content 
     || soap_send_raw(soap, buf, len) // send image 
     || soap_end_send(soap)
      return soap_closesock(soap);
    return SOAP_OK;
~~~

For client applications to use HTTP POST, use the `::soap_POST` operation:

~~~{.cpp}
    struct soap *soap = soap_new();
    ... //
    char *buf; // holds the HTTP request/response body data 
    size_t len; // length of data 
    ... // populate buf and len with message to send
    if (soap_POST(soap, "URL", "SOAP action or NULL", "media type") 
     || soap_send_raw(soap, buf, len); 
     || soap_end_send(soap)) 
     ... // error
    if (soap_begin_recv(&soap) 
     || (buf = soap_http_get_body(soap, &len)) == NULL
     || soap_end_recv(&soap)) 
      ... // error
    // ... use buf[0..len-1] 
    soap_destroy(soap);
    soap_end(soap);
    soap_free(soap);
~~~

Similarly, `::soap_PUT`, `::soap_PATCH`, and `::soap_DELETE` commands are
provided for PUT, PATCH, and DELETE handling.

To support HTTP pipelining we use the `::http_pipe` plugin, which can be used
at the server side to enable HTTP pipelining automatically, when registered.
The plugin can also be used at the client side, though this is only necessary
in scenarios that require the client to receive multiple messages without
intermittend sends, i.e. multiple sends followed by multiple receives by the
same thread using the same `::soap` context.  However, clients should use
multiple threads when HTTP pipelining is used to prevent blocking.  See
the <i>`gsoap/samples/async`</i> folder in the gSOAP package for explanation
and examples.

See also `::http_post` and `::http_pipe`.

🔝 [Back to table of contents](#)

### The HTTP digest authentication plugin   {#httpdaplugin}

The HTTP digest authentication plugin enables a more secure authentication
scheme compared to basic authentication. HTTP basic authentication sends
unencrypted userids and passwords over the net, while digest authentication
does not exchange passwords but exchanges checksums of passwords (and other
data such as nonces to avoid replay attacks). For more details, please see
RFC 2617.

The HTTP digest authentication can be used next to the built-in basic
authentication, or basic authentication can be rejected to tighten security.
The server must have a database with userid's and passwords (in plain text
form). The client, when challenged by the server, checks the authentication
realm provided by the server and sets the userid and passwords for digest
authentication. The client application can temporarily store the userid and
password for a sequence of message exchanges with the server, which is faster
than repeated authorization challenges and authentication responses.

At the client side, the plugin is registered and service invocations are
checked for authorization challenges (HTTP error code 401). When the server
challenges the client, the client should set the userid and password and retry
the invocation. The client can determine the userid and password based on the
authentication realm part of the server's challenge. The authentication information can be temporarily saved for multiple invocations.

Client-side example:

~~~{.cpp}
    #include "httpda.h" 

    int main()
    {
      struct soap *soap = soap_new();
      if soap_register_plugin(soap, http_da)) 
        exit(EXIT_FAILURE); // failed to register 
      if (soap_call_ns__webmethod(soap, ...)) 
      {
        if (soap->error == 401) // challenge: HTTP authentication required 
        {
          if (!strcmp(soap->authrealm, authrealm)) // optionally determine authentication realm
          {
            struct http_da_info info; // to store userid and passwd 
            http_da_save(soap, &info, authrealm, userid, passwd); // set userid and passwd for this realm
            if (soap_call_ns__webmethod(soap, ...)) // retry 
            {
              ... //
              soap_end(soap); // userid and passwd were deallocated 
              http_da_restore(soap, &info); // restore userid and passwd 
              if (!soap_call_ns__webmethod(soap, ...)) // another call 
                ... //
            }
          }
        }
      }
      http_da_release(soap, &info); // to remove all userid and passwd
      soap_destroy(soap);
      soap_end(soap);
      soap_free(soap);
    }
~~~

This code supports both basic and digest authentication.

The server can challenge a client using HTTP code 401. With the plugin, HTTP digest authentication challenges are send. Without the plugin, basic authentication challenges are send.

Each server method can implement authentication as desired and may enforce
digest authentication or may also accept basic authentication responses. To
verify digest authentication responses, the server should compute and compare
the checksums using the plugin's `http_da_verify_post` function for
HTTP POST requests (and `http_da_verify_get` for HTTP GET requests with
the HTTP GET plugin) as follows:

~~~{.cpp}
    #include "httpda.h" 

    int main()
    {
      struct soap *soap = soap_new();
      if (soap_register_plugin(soap, http_da)) 
        exit(EXIT_FAILURE); // failed to register 
      ... // use soap_bind and soap_accept in a (multi-threaded) service loop
      if (soap_serve(soap))
        soap_print_fault(soap, stderr);
      ... //
      soap_destroy(soap);
      soap_end(soap);
      soap_free(soap);
    }

    int ns__webmethod(struct soap *soap, ...) 
    {
      if (soap->userid && soap->passwd) // client used basic authentication 
      {
        // may decide not to handle, but if ok then go ahead and compare info: 
        if (!strcmp(soap->userid, userid) && !strcmp(soap->passwd, passwd)) 
        {
          ... // handle request
          return SOAP_OK; 
        } 
      } 
      else if (soap->authrealm && soap->userid) // Digest authentication 
      {
        passwd = ...; // database lookup on userid and authrealm to find passwd 
        if (!strcmp(soap->authrealm, authrealm) && !strcmp(soap->userid, userid)) 
        {
          if (!http_da_verify_post(soap, passwd)) 
          {
            ... // handle request
            return SOAP_OK; 
          } 
        } 
      } 
      soap->authrealm = authrealm; // set realm for challenge 
      return 401; // Not authorized, challenge digest authentication 
    }
~~~

For more details, including how to configure HTTP Digest authentication for
proxies, see the 
[HTTP digest authentication plugin](../../httpda/html/httpda.html) documentation.

🔝 [Back to table of contents](#)

### The HTTP sessions plugin   {#sessionsplugin}

The plugin code is located in the <i>`gsoap/plugin`</i> directory containing
<i>`sessions.h`</i> and <i>`sessions.c`</i>.

For more details, see the [HTTP sessions plugin](../../sessions/html/index.html) documentation.

🔝 [Back to table of contents](#)

### The Apache module plugin   {#apacheplugin}

The plugin code is located in the <i>`gsoap/mod_gsoap/mod_gsoap-0.9/apache_20`</i> directory.

For more details, see the [Apache module](../../apache/html/index.html) documentation.

🔝 [Back to table of contents](#)

### The ISAPI extension plugin   {#isapiplugin}

The plugin code is located in the <i>`gsoap/mod_gsoap/gsoap_win/isapi`</i> directory.

For more details, see the [ISAPI extension](../../isapi/html/index.html) documentation.

🔝 [Back to table of contents](#)

### The CURL plugin   {#curlplugin}

The plugin code is located in the <i>`gsoap/plugin`</i> directory containing
<i>`curlapi.h`</i> and <i>`curlapi.c`</i>.

For more details, see the [CURL plugin](../../curl/html/index.html) documentation.

🔝 [Back to table of contents](#)

### The WinInet plugin   {#wininetplugin}

The plugin code is located in the <i>`gsoap/mod_gsoap/gsoap_win/wininet`</i> directory.

For more details, see the [WinInet plugin](../../wininet/html/index.html) documentation.

🔝 [Back to table of contents](#)

### The WS-Addressing plugin   {#wsaplugin}

The plugin code is located in the <i>`gsoap/plugin`</i> directory containing
<i>`wsaapi.h`</i> and <i>`wsaapi.c`</i> (to be used in C and C++).

To enable WS-Addressing 2005 (and support for 8/2004), the service definitions header file for soapcpp2 should include the following imports:

~~~{.cpp}
    #import "import/wsa5.h"
~~~

This imports the SOAP header elements required by WS-Addressing.

For more details, see the [WS-Addressing plugin](../../wsa/html/wsa_0.html) documentation.

🔝 [Back to table of contents](#)

### The WS-ReliableMessaging plugin   {#wsrmplugin}

The plugin code is located in the <i>`gsoap/plugin`</i> directory containing <i>`wsrmapi.h`</i> and <i>`wsrmapi.c`</i> (to be used in C and C++).

Also needed are <i>`threads.h`</i> and <i>`threads.c`</i> for multi-threading and locking support.

To enable WS-ReliableMessaging, the service definitions header file for soapcpp2 should include the following imports:

~~~{.cpp}
    #import "import/wsrm.h" 
    #import "import/wsa5.h"
~~~

This imports the SOAP header elements required by WS-ReliableMessaging.

For more details, see the [WS-ReliableMessaging plugin](../../wsrm/html/wsrm_0.html) documentation.

🔝 [Back to table of contents](#)

### The WS-Security plugin   {#wsseplugin}

The plugin code is located in the <i>`gsoap/plugin`</i> directory containing <i>`wsseapi.h`</i> and <i>`wsseapi.c`</i> (to be used in C and C++).

Also needed are: <i>`smdevp.h`</i> and <i>`smdevp.c`</i> for streaming XML signature and message digest engine,
<i>`mecevp.h`</i> and <i>`mecevp.c`</i> for the streaming XML encryption engine, <i>`threads.h`</i> and <i>`threads.c`</i> for multi-threading and locking support.

To enable WS-Security, the service definitions header file for soapcpp2 should include the following imports:

~~~{.cpp}
    #import "import/wsse.h"
~~~

This imports the SOAP header elements required by WS-Security.

For more details, see the [WS-Security plugin](../../wsse/html/wsse.html) documentation.

🔝 [Back to table of contents](#)

### The WS-Discovery plugin  {#wsddplugin}

Basically, to add WS-Discovery support the following event handlers must be
defined and linked:

~~~{.cpp}
    void wsdd_event_Hello(struct soap *soap, 
    unsigned int InstanceId, 
    const char *SequenceId, 
    unsigned int MessageNumber, 
    const char *MessageID, 
    const char *RelatesTo, 
    const char *EndpointReference, 
    const char *Types, 
    const char *Scopes, 
    const char *MatchBy, 
    const char *XAddrs, 
    unsigned int MetadataVersion)
~~~

~~~{.cpp}
    void wsdd_event_Bye(struct soap *soap, 
    unsigned int InstanceId, 
    const char *SequenceId, 
    unsigned int MessageNumber, 
    const char *MessageID, 
    const char *RelatesTo, 
    const char *EndpointReference, 
    const char *Types, 
    const char *Scopes, 
    const char *MatchBy, 
    const char *XAddrs, 
    unsigned int MetadataVersion)
~~~

~~~{.cpp}
    soap_wsdd_mode wsdd_event_Probe(struct soap *soap, 
    const char *MessageID, 
    const char *ReplyTo, 
    const char *Types, 
    const char *Scopes, 
    const char *MatchBy, 
    struct wsdd__ProbeMatchesType *ProbeMatches)
~~~

~~~{.cpp}
    void wsdd_event_ProbeMatches(struct soap *soap, 
    unsigned int InstanceId, 
    const char *SequenceId, 
    unsigned int MessageNumber, 
    const char *MessageID, 
    const char *RelatesTo, 
    struct wsdd__ProbeMatchesType *ProbeMatches)
~~~

~~~{.cpp}
    soap_wsdd_mode wsdd_event_Resolve(struct soap *soap, 
    const char *MessageID, 
    const char *ReplyTo, 
    const char *EndpointReference, 
    struct wsdd__ResolveMatchesType *ResolveMatches)
~~~

~~~{.cpp}
    void wsdd_event_ResolveMatches(struct soap *soap, 
    unsigned int InstanceId, 
    const char *SequenceId, 
    unsigned int MessageNumber, 
    const char *MessageID, 
    const char *RelatesTo, 
    const char *EndpointReference, 
    const char *Types, 
    const char *Scopes, 
    const char *MatchBy, 
    const char *XAddrs, 
    unsigned int MetadataVersion)
~~~

These event handlers will be invoked when inbound WS-Discovery messages arrive using:

~~~{.cpp}
    if (!soap_valid_socket(soap_bind(soap, NULL, port, BACKLOG))) 
      ... // error 
    if (soap_wsdd_listen(soap, timeout)) 
      ... // error
~~~

which will listen for `timeout` seconds to inbound WS-Discovery messages
on a port and dispatches them to the event handlers. A negative `timeout` value specifies
the timeout in microseconds.

For more details, see the [WS-Discovery plugin](../../wsdd/html/wsdd_0.html) documentation.

Copyright                                                           {#copyright}
=========

<i>Copyright (c) 2000-2020, Robert A. van Engelen, Genivia Inc.<br>All rights reserved.</i>
