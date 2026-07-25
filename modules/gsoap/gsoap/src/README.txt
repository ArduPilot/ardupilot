The gSOAP 'soapcpp2' source-to-source compiler

INSTRUCTIONS

The gSOAP soapcpp2 tool translates annotated C/C++ header files with interface
defitions for services and clients to service and client implementation code.
It also maps the C/C++ types to XML types, with the ability to generate XML
schema and WSDL documents.

When starting from WSDL and/or XML schemas, first use the gSOAP 'wsdl2h' tool
to translate these into C/C++ header file with interface definitions. Then use
'soapcpp2' to translate these into implementation code.

See also the README.txt in the 'wsdl' directory and documentation on the use of
'wsdl2h' with 'soapcpp2'.

CONTENTS

This part of the distribution contains the following files:

README.txt	This file
MakefileManual	Extra makefile when autoconf/automake fail to produce one
soapcpp2.h	Main header file
soapcpp2.c	Main application
symbol2.c	Symbol table handling and code generation module
error2.h	Header file for error2.c
error2.c	Error handling routines
init2.c		Compiler symbol table initialization
soapcpp2_lex.l	Flex/Lex tokens
soapcpp2_yacc.y	Yacc/Bison grammar

INSTALLATION

Use './configure' and 'make' in the root directory, as explained in the
installation instructions.

To build 'soapcpp2' when autoconf/automake fail, use:

	make -f MakefileManual

The above commands to build 'soapcpp2' assume you have Bison and Flex
installed. To use Yacc instead, please use:

	$ cd gsoap/src
	$ make YACC='yacc -d -v -s soapcpp2_yacc' CMFLAGS='-DWITH_YACC -DWITH_FLEX' -f MakefileManual

If you do not have the Bison tool, please download and install it from here:

	http://www.gnu.org/software/bison/

If you do not have the Flex tool, please download and install it from here:

      	http://flex.sourceforge.net

What if you do not have Bison and Flex?

Included in gsoap/src are the flex-generated file 'lex.yy.c' and
bison-generated files 'soapcpp2_yacc.tab.h' and 'soapcpp2_yacc.tab.c'. These
files may suffice (no guarantee however) to build 'soapcpp2' as follows:

	$ cd gsoap/src
	$ make -f MakefileManual soapcpp2

In case the files 'lex.yy.c', 'soapcpp2_yacc.tab.h', and 'soapcpp2_yacc.tab.c'
were deleted in a prior build run, please unarchive the gSOAP package again
to retrieve these original files.

For your project builds, use the stdsoap2.c and stdsoap2.cpp sources rather
than the libgsoap libs, as the libs are not built. To enable SSL, GZIP, HTTP
cookies, IPv6 support, and/or force C locale usage, use the compiler flags:

	-DWITH_OPENSSL		to enable SSL, link with OpenSSL
	-DWITH_GNUTLS		to enable SSL, link with GNUTLS
	-DWITH_GZIP		to enable compression, link with Zlib
	-DWITH_COOKIES		to enable HTTP cookies
	-DWITH_IPV6		to enable IPv6
	-DWITH_C_LOCALE		to force C locale

Note: these flags when set must be used to compile ALL your sources to ensure
consistency.
QNX INSTALLATION

On QNX the bison.simple file is located in $QNX_HOST/usr/share/bison.simple
Update your .profile to include:

export BISON_SIMPLE=$QNX_HOST/usr/share/bison/bison.simple 
export BISON_HAIRY=$QNX_HOST/usr/share/bison/bison.hairy 

WIN32 INSTALLATION

You need to install Flex and Bison to build soapcpp2.

An MSN article explains how to do this with MS VS2005:

http://msdn.microsoft.com/en-us/library/aa730877(VS.80).aspx#vccustombr_topic3

The older Bison v1.6 can crash on Win32 systems if YYINITDEPTH is too small:
Compile with /DYYINITDEPTH=5000

COMMAND LINE OPTIONS

-0      no SOAP, generate REST source code
-1      generate SOAP 1.1 source code
-2      generate SOAP 1.2 source code
-A	require SOAPAction headers to invoke server-side operations
-a	use SOAPAction with WS-Addressing to invoke server-side operations
-b	serialize byte arrays char[N] as string
-C	generate client-side code only
-c      generate C source code
-c++    generate C++ source code (default)
-c++11  generate C++ source code optimized for C++11 (compile with -std=c++11)
-dpath  use path to save files
-Ec	generate extra functions for deep copying
-Ed	generate extra functions for deep deletion
-Et     generate extra functions for data traversals with callback functions
-e	generate SOAP RPC encoding style bindings (also use -1 or -2)
-fN	multiple soapC files, with N serializer definitions per file (N>=10)
-h	display help info and exit
-Ipath  use path(s) for #import (paths separated with ':', or ';' for windows)
-i      generate C++ service proxies and objects inherited from soap struct
-j      generate C++ service proxies and objects that share a soap struct
-L	don't generate soapClientLib/soapServerLib
-l      generate linkable modules (experimental)
-m      generate source code for the Matlab(tm) MEX compiler (deprecated)
-n      use service name to rename service functions and namespace table
-pname  save files with new prefix name instead of 'soap'
-Qname  use name as the C++ namespace, including custom serializers
-qname  use name as the C++ namespace, exclusing custom serializers
-r      generate soapReadme.md report
-S	generate server-side code only
-s      generate stub and skeleton functions with strict XML validation checks
-T	generate server auto-test source code
-t      generate source code for fully xsi:type typed SOAP/XML messages
-u	uncomment comments in WSDL/schema output by suppressing XML comments
-V	display the current version and exit
-v	verbose output
-w	don't generate WSDL and schema files
-x	don't generate sample XML message files
-y	include C/C++ type access information in sample XML messages
-z1	generate deprecated old-style C++ service proxies and objects
-z2	compatibility with 2.7.x: omit XML output for NULL pointers
-z3     compatibility up to 2.8.30: _param_N indexing and nillable pointers
-z4     compatibility up to 2.8.105: char* member defaults, even when omitted
infile	header file to parse (or stdin)

DOCUMENTATION

See soapdoc2.pdf for documentation.

A NOTE ON HEAP MEMORY USAGE

Parts of the soapcpp2 tool allocate heap memory during its execution to
maintain critical state information during its execution.  No attempt is made
to release this memory before the program terminates.

LICENSE

The gSOAP 'soapcpp2' tool and (generated) source code are released under GPL or
a commercial license. The commercial license is available from Genivia.
Please visit http://genivia.com/Products/gsoap/contract.html

COPYRIGHT NOTICE

gSOAP XML Web services tools
Copyright (C) 2000-2011, Robert van Engelen, Genivia, Inc. All Rights Reserved.
