================================================================================

Thanks for using gSOAP!

The gSOAP toolkit provides a cross-platform software development toolkit for C
and C++ server and client Web service applications, and simplifies the overall
use of XML in any type of application. The toolkit supports SOAP 1.1/1.2 RPC
encoding and document/literal styles, WSDL 1.1, MTOM/MIME/DIME attachments
(streaming), SOAP-over-UDP, request-response and one-way messaging. The toolkit
also supports WS-Addressing and WS-Security, with several other WS-* available
or under development. See the official open-source gSOAP website
http://gsoap2.sourceforge.net for project status and latest news.

* The gSOAP 'soapcpp2' compiler and code generator and 'stdsoap2' runtime
  engine are stable since version release 2.1.3.

* The gSOAP 'wsdl2h' WSDL/schema parser and code generator is stable since
  wsdl2h version release 1.1.0. The 'wsdl2h' tool fully supports WSDL 1.1,
  XML schemas, WS-Policy, and other WS-* protocols (details in the fact sheet).

The software is provided "as is", without any warranty. However, gSOAP
has been extensively field-tested in production-quality software development.
We continue to improve gSOAP and add new features.

See NOTES.txt for distributed notes and an overview of the package contents.
See INSTALL.txt for installation instructions.
See LICENSE.txt for software licensing.

================================================================================
WHAT'S COOL?
================================================================================

The gSOAP 'wsdl2h' tool is a gSOAP application itself, which demonstrates the
capabilities of the generic XML handling by the toolkit to parse WSDL, XML
schemas, and SOAP/XML.

The gSOAP toolkit supports streaming technologies to expedite SOAP/XML and
MTOM/MIME attachment transfers of potentially unlimited data lengths.

The gSOAP toolkit is the only toolkit that supports the serialization of native
C and C++ data types directly in XML. You can use it to export and import your
application data in XML without having to write wrapper routines.

The gSOAP toolkit ensures as small memory footprint. XML is a processed as a
transient format and not buffered. Many optimizations have been applied to
reduce resource requirements and to expedite XML parsing.

The gSOAP toolkit provides stand-alone HTTP(S) web server functionality as well
as Apache mod and IIS hooks (located in gsoap/mod_gsoap). Also CGI and FastCGI
are supported. A web server example demonstrating the stand-alone functionality
is included in the package.

================================================================================
FEATURES
================================================================================

XML data binding tools for C/C++
XML schema <=> C/C++ type binding means XML and C/C++ data is type safe
XML streaming auto-serialization of C/C++ data (with optional use of DOM)
XML-RPC from/to JSON from/to C/C++ serialization (also in streaming mode)
No need to alter C/C++ types for serialization (declare type as 'volatile')
WSDL 1.1/2.0, XSD 1.0/1.1, SOAP 1.1/1.2 compliant
REST HTTP(S) 1.0/1.1 operations (GET,PUT,POST etc) for XML, JSON, etc
Send and recieve XML over sockets, file FD, and C++ streams
WS-I Basic Profile 1.0a, 1.1, and 1.2 compliant
W3C schema patterns for data binding full test pattern coverage
RSS 0.91, 0.92, 2.0 XML support
MIME and MTOM attachment support (also in streaming mode)
WS-Security XML authentication, signatures, encryption (also in streaming mode)
WS-Policy 1.2, 1.5 and WS-SecurityPolicy 1.2 compliant
WS-Addressing 2003/03, 2004/03, 2005/03 compliant
WS-ReliableMessaging 1.0 and 1.1 compliant
WS-Discovery 1.0/1.1
UDDI v2 API
NTLM authentication
HTTP basic and digest authentication
SSL/TLS communications with SSL session caching (OpenSSL or GNUTLS)
Proxy and proxy authentication support
Compression (HTTP compression and zlib)
IPv4 and IPv6, including direct TCP and UDP data transfer
SOAP-over-UDP
Apache 1.x and 2.0 modules
IIS (ISAPI) and WinInet modules
CGI and FastCGI support
Stand-alone Web server included (multithreaded, SSL, compression)
Integrated memory management with deallocation and leak detection
Plug-ins for additional capabilities
Internationalization/localization support (UTF8, UCS4, MB encodings, etc)
WSDL/XSD conversion to C or C++ and vice versa
Portable to small devices (WinCE, Palm, Symbian, VxWorks, Andriod, iPhone)
Auto-test server code generation for (dummy) server testing
Automatic XML document and message generation from WSDL and XSD
C/C++ (cyclic) object graph auto-serialization (with SOAP id-href encoding)
STL container auto-serialization and custom C++ container auto-serialization
Over 40 example client and server applications included

================================================================================
PACKAGE
================================================================================

This distribution package contains platform-independent source code. Pre-built
'soapcpp2' and 'wsdl2h' binaries are included for the following platforms:

	* Win32 i386 compatible
	* MAC OS X

The binaries are located in 'gsoap/bin'.

Important: these 'wsdl2h' binaries use the default configuration without SSL
support (no HTTPS site access).

To configure and build the toolkit binaries and libraries, please see the
installation instructions in the 'INSTALLATION' section below.

================================================================================
GETTING STARTED
================================================================================

Follow the installation instructions in INSTALL.txt first.

The gSOAP 'wsdl2h' tool converts WSDLs into a gSOAP header file for processing
with the gSOAP code gnerator 'soapcpp2' to generate XML serialization, stubs,
and skeletons code to build Web services applications.  Use 'wsdl2h' followed
by 'soapcpp2' to translate an entire set of WSDL and XML schemas into
representative C or C++ data structures and associated XML parsers. You can
also use the gSOAP 'soapcpp2' tool directly on existing C/C++ data structure
declarations to create XML serialization routines for these types to simplify
the storage of data in XML.

Example translation of WSDL to code in two steps:

	$ wsdl2h -s -o calc.h http://www.cs.fsu.edu/~engelen/calc.wsdl
	$ soapcpp2 -CL -I/path/to/gsoap/import calc.h

The 'calc.h' header file contains the services and XML schema types represented
in C/C++, together with other useful information copied from the WSDL related
to the service. Run Doxygen on it to generate a nice set of pages.

Do not include the wsdl2h-generated 'calc.h' header file directly into your
code (the declarations are replicated in the generated code). The header file
is processed by the gSOAP stub compiler 'soapcpp2' to generate the following
files for your project:

	soapClient.cpp	client-side stub routines for service invocation
	soapServer.cpp	server-side skeleton routines for server development
	soapC.cpp	C/C++ parameter marshalling code
	calc.nsmap	An XML-to-C/C++ namespare mapping table

To compile a client, all you need to do is to compile and link 'soapC.cpp',
'soapClient.cpp', and 'stdsoap2.cpp' (or the installed libgsoap++, see
INSTALLATION) with your code. To access the service in your code:

	#include "soapH.h"
	#include "calc.nsmap"
        main()
	{ struct soap *soap = soap_new(); // alloc 'soap' engine context
	  double result;
	  if (soap_call_ns2__add(soap, NULL, NULL, 1.0, 2.0, result) == SOAP_OK)
	    std::cout << "The sum of 1.0 and 2.0 is " << result << std::endl;
	  else
	    soap_stream_fault(soap, std::cerr);
	  soap_destroy(soap); // dealloc serialization data
	  soap_end(soap);     // dealloc temp data
	  soap_free(soap);    // dealloc 'soap' engine context
	}

First, this imports all soapcpp-generated definitions and the namespace mapping
table. Then the soap_call_ns2__add() invokes the service. This function is
generated from the calc.h file by soapcpp2. The calc.h file includes
instructions on what functions to call.

To develop a C++ client application based on C++ proxy objects rather than
C-like functions, use 'soapcpp2' option -j:

	$ wsdl2h -s -o calc.h http://www.cs.fsu.edu/~engelen/calc.wsdl
	$ soapcpp2 -j -CL -I/path/to/gsoap/import calc.h

This generates 'soapcalcProxy.h' and 'soapcalcProxy.cpp' with a calcProxy
class with service methods that you can use to invoke services. For example:

	#include "soapcalcProxy.h"
	#include "calc.nsmap"
	main()
	{ calcProxy service;
	  double result;
	  if (service.add(1.0, 2.0, result) == SOAP_OK)
	    std::cout << "The sum of 1.0 and 2.0 is " << result << std::endl;
  	  else
	    service.soap_stream_fault(std::cerr);
	  service.destroy(); // dealloc serialization and temp data
	}

Compile the above program and link with 'soapC.cpp', 'soapcalcProxy.cpp', and
'stdsoap2.cpp' (or -lgsoap++).

To develop a C client, use 'wsdl2h' option -c to generate pure C code.

There are many options that you can use depending on the need to develop C
applications, C++ with or without STL, or C++ proxy and server objects. In
addition, the XML schema type mapping is defined by 'typemap.dat', located in
the project root and 'WS' folders. The 'typemap.dat' file is used to customize
the 'wsdl2h' output. It is important to use this file for larger projects that
depend in WS-* protocols, such as WS-Addressing and WS-Security.

More information about the 'wsdl2h' and 'soapcpp2' tools and their options can
be found in the gSOAP documentation and the Quick How-To page on the gSOAP Web
site, see: http://gsoap2.sourceforge.net

See also the 'gsoap/wsdl/README.txt' for more details on the WSDL parser and
installation. The 'gsoap/bin' folder includes pre-built 'soapcpp2' and
'wsdlh2' executables for various platforms.

================================================================================
BUILD NOTES
================================================================================

QNX

On QNX the bison.simple file is located in $QNX_HOST/usr/share/bison.simple
Update your .profile to include:

	export BISON_SIMPLE=$QNX_HOST/usr/share/bison/bison.simple 
	export BISON_HAIRY=$QNX_HOST/usr/share/bison/bison.hairy 

WIN32

Bison 1.6 can crash on Win32 systems if YYINITDEPTH is too small Compile with
/DYYINITDEPTH=5000

================================================================================
CHANGELOG
================================================================================

Visit http://www.cs.fsu.edu/~engelen/changelog.html to view the latest changes.

================================================================================
LICENSE
================================================================================

See LICENSE.txt

================================================================================
COPYRIGHT
================================================================================

Copyright (C) 2000-2015 Robert van Engelen, Genivia, Inc. All Rights Reserved.

================================================================================
USE RESTRICTIONS
================================================================================

You may not: (i) transfer rights to gSOAP or claim authorship; or (ii) remove
any product identification, copyright, proprietary notices or labels from gSOAP.

================================================================================
WARRANTY 
================================================================================

GENIVIA INC. EXPRESSLY DISCLAIMS ALL WARRANTIES, WHETHER EXPRESS, IMPLIED OR
STATUTORY, INCLUDING, WITHOUT LIMITATION, THE IMPLIED WARRANTIES OF
MERCHANTABILITY, OF FITNESS FOR A PARTICULAR PURPOSE, NONINFRINGEMENT OF THIRD
PARTY INTELLECTUAL PROPERTY RIGHTS, AND ANY WARRANTY THAT MAY ARISE BY REASON
OF TRADE USAGE, CUSTOM, OR COURSE OF DEALING.  WITHOUT LIMITING THE
FOREGOING, YOU ACKNOWLEDGE THAT THE SOFTWARE IS PROVIDED "AS IS" AND THAT
GENIVIA INC. DO NOT WARRANT THE SOFTWARE WILL RUN UNINTERRUPTED OR ERROR FREE.
LIMITED LIABILITY: THE ENTIRE RISK AS TO RESULTS AND PERFORMANCE OF THE
SOFTWARE IS ASSUMED BY YOU.  UNDER NO CIRCUMSTANCES WILL GENIVIA INC. BE LIABLE
FOR ANY SPECIAL, INDIRECT, INCIDENTAL, EXEMPLARY OR CONSEQUENTIAL DAMAGES OF
ANY KIND OR NATURE WHATSOEVER, WHETHER BASED ON CONTRACT, WARRANTY, TORT
(INCLUDING NEGLIGENCE), STRICT LIABILITY OR OTHERWISE, ARISING OUT OF OR IN
ANY WAY RELATED TO THE SOFTWARE, EVEN IF GENIVIA INC. HAS BEEN ADVISED ON THE
POSSIBILITY OF SUCH DAMAGE OR IF SUCH DAMAGE COULD HAVE BEEN REASONABLY
FORESEEN, AND NOTWITHSTANDING ANY FAILURE OF ESSENTIAL PURPOSE OF ANY
EXCLUSIVE REMEDY PROVIDED.  SUCH LIMITATION ON DAMAGES INCLUDES, BUT IS NOT
LIMITED TO, DAMAGES FOR LOSS OF GOODWILL, LOST PROFITS, LOSS OF DATA OR
SOFTWARE, WORK STOPPAGE, COMPUTER FAILURE OR MALFUNCTION OR IMPAIRMENT OF
OTHER GOODS.  IN NO EVENT WILL GENIVIA INC. BE LIABLE FOR THE COSTS OF
PROCUREMENT OF SUBSTITUTE SOFTWARE OR SERVICES.  YOU ACKNOWLEDGE THAT THIS
SOFTWARE IS NOT DESIGNED FOR USE IN ON-LINE EQUIPMENT IN HAZARDOUS
ENVIRONMENTS SUCH AS OPERATION OF NUCLEAR FACILITIES, AIRCRAFT NAVIGATION OR
CONTROL, OR LIFE-CRITICAL APPLICATIONS.  GENIVIA INC. EXPRESSLY DISCLAIM ANY
LIABILITY RESULTING FROM USE OF THE SOFTWARE IN ANY SUCH ON-LINE EQUIPMENT IN
HAZARDOUS ENVIRONMENTS AND ACCEPTS NO LIABILITY IN RESPECT OF ANY ACTIONS OR
CLAIMS BASED ON THE USE OF THE SOFTWARE IN ANY SUCH ON-LINE EQUIPMENT IN
HAZARDOUS ENVIRONMENTS BY YOU.  FOR PURPOSES OF THIS PARAGRAPH, THE TERM
"LIFE-CRITICAL APPLICATION" MEANS AN APPLICATION IN WHICH THE FUNCTIONING OR
MALFUNCTIONING OF THE SOFTWARE MAY RESULT DIRECTLY OR INDIRECTLY IN PHYSICAL
INJURY OR LOSS OF HUMAN LIFE.

================================================================================
EXTERNAL THIRD-PARTY LIBRARIES
================================================================================

The gSOAP toolkit is self-contained and does not require any third-party
software to run in its standard configuration. When compression and SSL
encryption are required the Zlib and OpenSSL libraries must be installed.

To build the gSOAP 'soapcpp2' compiler, you must have Bison and Flex installed
or the older Yacc and Lex equivalents. Note that licensing differs for Flex
versus Lex, and Bison versus Yacc.

Win32 builds of clients and services requires winsock.dll. To do this in
Visual C++ 6.0, go to "Project", "settings", select the "Link" tab (the
project file needs to be selected in the file view) and add "wsock32.lib" to
the "Object/library modules" entry. The distribution contains a Visual Studio
2005 project example in the 'samples/calc_vs2005' folder with the necessary
project settings to link libraries and automatically invoke to soapcpp2
compiler as a custom build.

================================================================================
DISCLAIMER
================================================================================

WE PROVIDE YOU WITH "REAL-WORLD" EXAMPLES BUT WE CANNOT GUARANTEE THAT ALL
CLIENT EXAMPLES CAN CONNECT TO THIRD PARTY WEB SERVICES WHEN THESE SERVICES ARE
DOWN OR HAVE BEEN REMOVED.

================================================================================
