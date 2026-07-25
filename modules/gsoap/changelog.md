
[![Go down](images/go-down.png) Go down](#latest) to view the latest changes

Version 1.1 (1/1/2000)
---
- Added typedef conventions.
- Added enumerations.
- Added hexBinary encoding.
- Added base64 encoding.
- Enable pass by reference operator (&) for output parameter.
- Enable specification of methods in classes.

Version 1.2 (9/9/2001)
---
- Added WSDL generator.

Version 1.2w (9/14/2001)
---
- Win32 port.
- Fixed a socket communication problem in win32 version.
- Added namespace name pattern matching to ease validation.

Version 1.2.1 (10/24/2001)
---
- Added chunked HTTP 1.1 transfer support.
- Improved buffering.
- Fixed a problem with nested vector allocation.

Version 1.2.2 (11/2/2001)
---
- Support for special XML element names with dots, underscores, and center dots (see documentation section 5.3).
- Fixed a decoding problem with dynamic array of pointers to polymorphic objects.
- Fixed an encoding problem with enumerations.
- Added a "safe-mode" flag to disable serialization of multi-referenced objects with `soap_disable_href = 1;`
- You can set this global flag anywere in the code, but at least before serialization is performed. It will disable the use of href attributes when sending multi-reference data. Instead, the data is copied in the payload. When set, this flag will hang the serializer when sending cyclic data structures.

Version 1.2.3 (12/5/2001)
---

- Added bool type encoding/decoding.
- Added dynamic multi-dimensional arrays.
- Added support for primitive polymorphic types.
- Added full support for CDATA content decoding.
- More convenient customization of SOAP Headers and Faults. No separate soapheader.h and soapfault.h files required (and therefore no .cpp files have to be created for these). Instead, the compiler generates customized SOAP Header and SOAP Fault marshalling routines when struct/class `SOAP_ENV__Fault` and/or `SOAP_ENV__Header` are specified in the header file input to the compiler.
- On demand generation of the marshalling routines for the primitive types, which reduces the size of the executables.
- Fixed a WSDL incompatibility problem in the WSDL generator.
- Improved decoding of multi-reference elements (no *`xsi:type`* required anymore in receiving message so gSOAP does not break).
- Improved support for encoding/decoding indirect data (e.g. pointers to pointers to data).
- Improved encoding of data with the same pointers to shared data but with the shared data declared with different XML schema types (formerly encoded as multi-reference data which could cause type incmpatibilities at the receiving side). For example, in the declarations `typedef char *xsd__NCName; xsd__NCName *s="SOAP"; char *t = s;` the pointers s and t point to the same data, but will not be encoded as multi-reference data since the types are different.
- Added flag `soap_enable_null = 1;`
- When set, all NULL pointers will be explicitly encoded. By default, NULL pointers are omitted from the SOAP payload. For example, with this flag set all NULL strings and other NULL pointer data will appear as nil values in the SOAP payload.
- Added flag: `soap_enable_embedding = 1;`
- When set, multi-reference data will be encoded inline which will guarantee the exact preservation of the structure of the data transmitted. However, some SOAP implementations do not support embedding inline multi-reference data although this encoding style is likely to be included in future verions of the SOAP protocol. The flag is useful when creating C++ applications that need to communicate data to eachother and the data structures need to be preserved. Setting this flag may cause a duplication of multi-reference data by the receiver created with the SOAP stub compiler when the data is part of a struct/class or array. The data is not copied by the receiver when the struct/class or arrays use pointers to the data type.
- Added the ability to declare virtual destructors.
- Fixed a compilation error with fixed-size array of pointers.
- Fixed a problem with padding in Base64 (en/de)coding.
- Fixed XML entity decoding (hexadecimal).
- Added encoding of sparse arrays. A sparse array MUST be declared as a (fixed-size of dynamic) array of pointers. NULL pointer values are omitted from the output and *`SOAP-ENC:position`* attributes are used for non-NULL elements.
- Changed `soap_put` and `soap_get` functions to include explicit schema type parameters. This fixed a problem with Web services that need to return typedef-ed XML schema types The new stdsoap.cpp and stdsoap.h files are incompatible with the previous version(s).
- Added the ability to ignore the names of accessors when decoding fields through the use of a leading underscore (`_`) in a field name. For example: `ns__mymethod(xsd__string in, xsd__string &_return);`
- When used to invoke a remote method, the actual element name of the remote return parameter used by the service is insignificant.
- Improved memory management. Added new function: `soap_destroy()` to remove all class instances.
- Improved documentation.
- Added `time_t` type support, but still need to add encoding/decoding routines to the runtime (next version).
- Added `wchar_t` type support, but still need to add encoding/decoding routines to the runtime (next version).

Version 1.2.4 (12/17/2001)
---
- Added support for wide character strings (`wchar_t*`).
- Added support for `time_t` type.
- Added support for SOAP literal encoding.
- Fixed an obscure bug in the deserialization of data from the SOAP4R toolkit that caused the deserializer to hang.
- Fixed a problem with the `soap_disable_href` flag.
- Fixed a problem with the position attributes in sparse multi-dimensional arrays.
- Fixed a problem with the generation of .nsmap files.
- Fixed a problem with mixed content decoding in strings.

Version 1.2.5 (12/27/2001)
---
- Fixed a memory leak in the block allocation scheme used for strings, hexBinary, and base64 types.
- Fixed a bug in the WSDL generator for fixed-size arrays.
- Fixed a problem with the use of trailing underscores in field names in struct/class declarations.

Version 1.2.6 (1/25/2002)
---
- Improved portability.
- Improved id/href hash table efficiency and storage.
- Improved namespace URI matching with wildcards.
- Improved stand-alone deployment.
- Added `soap_disable_response_count` flag.
- Fixed operator declaration parsing (cast operators).
- Fixed a WSDL generator bug for output parameters and enumeration types.
- Renamed function `soap_return_fault()` into `soap_send_fault()`.

Version 2.0 (2/2/2002)
---
- Rewrote the compiler (soapcpp2) and the run-time engine libraries to create a thread safe implementation.

Versions 1.2.7 and 2.0.1 (2/11/2002)
---
- Added new soapcpp2 compiler options (-h -c -d -p).
- Added optional specification of service name, location, and namespace URI in header file.
- Improved interoperability.

Versions 1.2.8 and 2.0.2 (2/24/2002)
---
- Added function callbacks to support customized I/O and HTTP operations. This allows for plug-in HTTP code, NSAPI interface code, reading/writing from/to files or string buffers, etc.
- Added HTTP 1.0/1.1 keep-alive support.
- Added HTTP 1.1 chunked transfer support.
- Added `soap_disable_request_count` flag.
- Added `soap_enable_array_overflow` flag.
- Added type naming conventions to serialize elements without generating *`xsi:type`* attributes.
- Fixed a fixed-size array decoding problem (type mismatch).
- Fixed `</sequence/>` print bug in the WSDL generator.

Versions 1.3 and 2.1 (3/10/2002)
---
- Added client-side SSL (HTTPS) support (thanks to Massimo Cafaro for his suggestions).
- Added a naming convention for dynamic array `__ptr` fields which enables the naming of the XML tags of array elements.
- Added WSDL Header and Fault schema generation.
- Improved doc/literal encoding style.
- Improved WSDL service/schema namespace generation.
- Fixed *`SOAP-ENV:Header`* to be self contained.
- Fixed WSDL generation of xsd schema types declared as string.
- Fixed a method overriding type clash for derived classes.
- Fixed spurious occurrence of `id="-1"` with `soap_enable_null` flag enabled.
- Fixed a front-end gSOAP compiler problem with forward and cyclic struct declarations.

Versions 1.3.1 and 2.1.1 (3/25/2002)
---
- Added header file source constructs to support transient data types and transient struct/class fields to control gSOAP's code generatiion window for (de)serialization routines.
- Added callback for handling XML elements that are ignored on the receiving side (e.g. because of schema type problems). This feature is intended to enhance the reliability of services/clients.
- Added server-side SSL support and included an example multi-threaded stand-alone SSL secure SOAP server example.
- Added proxy server support.
- Improved message logging in gSOAP 2.1.1 (thanks to Jessie Ragsdale for his suggestions).
- Fixed WSDL generation for dynamic arrays.
- Fixed deserialization of empty multi-ref derived class instances.

Versions 1.3.2 and 2.1.2 (4/15/2002)
---
- Added socket timeout management.
- Added optional client and server cookie support.
- Added support for `maxOccurs="unbounded"` for arbitrary elements in complexTypes (not just limited to SOAP arrays).
- Improved generation of schema *`xs:extension`* definitions to allow XML validation in the presence of class overriding.
- Fixed ULONG64 type handling.
- Fixed light validation problem (now rejects invalid XML schema URIs).
- Fixed a deserialization type mismatch with typedef-ed pointer primitives.
- Fixed WSDL and schema interoperability problems.

Versions 1.3.3 and 2.1.3 (4/27/2002)
---
- Added `mustUnderstand` declaration qualifier for SOAP Header processing.
- Added `soap::actor` attribute for SOAP Header processing.
- Added method-header-part directive to identify methods with header message associations (WSDL requirement).
- Added bitmask (de)serialization.
- Added FastCGI support.
- Improved DLL build support (gSOAP 2.1.3 only).
- Improved WinCE support (gSOAP 2.1.3 only, thanks to Sean Ryan for his suggestions).
- Improved Mac OS X support.
- WSDL generator improvements and fixes.
- Workaround a bug in .NET SOAP Headers (non-unique id name values in .NET).
- Fixed `disable_href=1` bug.
- Fixed XML in string parsing code generation problem.

Versions 1.3.4 and 2.1.4 (6/9/2002)
---
- Added non-blocking `soap_accept()` with timeout management.
- Added support for SOAP Header and Fault declared as classes.
- Added syntax for `extern` declarations for transient types and fields.
- Added support for SOAP Headers in SOAP Fault messages.
- Added one-way SOAP messages (2.1.4 only).
- Added support for user-defined (de)serializers (2.1.4 only).
- Improved object memory management.
- Improved support for AIX and HP platforms.
- Fixed proxy server connect bug.
- Fixed an OpenSSL bug.
- Fixed `soap_instantiate_ClassX()` derived class allocation problem.
- Fixed generation of spurious .xsd files when lots of trailing underscores are used by identifiers in the header file.
- Fixed syntax error that occured with header files containing cyclic class declarations.
- Fixed 'class in struct' and 'vector of classes' VMT initialization bugs.

Version 2.1.5 (6/22/2002)
---
- Added non-blocking remote method calls (not supported in win32 version).
- Added specification of default values for struct/class fields.
- Added specification of minOccurs and maxOccurs for struct/class fields.
- Added support for the resolution operator `::` in identifiers.
- Added abstract method declaration support.
- Fixed WSDL generation of SOAP doc/lit.
- Fixed `WITH_FASTCGI` option.

Version 2.1.6 (7/10/2002)
---
- Added DIME attachment support.
- Added win32 non-blocking sockets.
- Changed `"%lld"` and `"%llu"` format strings into `"%I64d"` and `"%I64u"` format string for win32.
- Improved I/O.
- Fixed multi-dimensional dynamic array (de)serialization.
- Fixed WSDL enum and header namespace generation.

Version 2.1.7 (8/6/2002)
---
- Added generation of example SOAP/XML request and response messages by the compiler.
- Added RSA to SSL stack.
- Added two callbacks to the HTTP stack for efficient handling of HTTP headers in Apache-mod and IIS.
- Added compiler option -m (malloc() memory control) and updated doc on memory allocation policies.
- Added compiler option -i to support #include and #define directives.
- Improved HTTP cookie support to handle new and old cookie protocols.
- Improved external class (de)serialization support (std::string example is included).
- Fixed a SSL+proxy connect problem.
- Fixed uninitialized '//gsoap..executable:' string which caused sporadic exits.
- Fixed XML literal string encoding problem introduced with DIME.
- Fixed and removed WSDL duplicate response elements.
- Fixed LONG64 and ULONG64 types for WinCE.

Version 2.1.8 (8/28/2002)
---
- Added client-side proxy class source code generation.
- Added `soap.enable_strict` flag to enable very strict validation of messages (unknown namespace URIs and XML elements are not allowed).
- Added `soap.socket_flags` to control socket send and recv flags, e.g. to disable broken pipe signals (`soap.socket_flags=MSG_NOSIGNAL`).
- Added `//gsoap ns service method-action` header file directive.
- Added server-side `soap_receiver_fault()` and `soap_sender_fault()` functions to return SOAP 1.1 and SOAP 1.2 faults.
- Added seeding of the PRNG for SSL.
- Added soapcpp2 compiler option '-2' to automate SOAP 1.2 support (SOAP 1.2 features are Beta-release).
- Changed keep-alive support (see docs).
- Improved WSDL output.
- Workaround Bison 1.6 for win32 bug in gSOAP win32 version.
- Fixed a wild-card matching bug in `soap_tag_cmp()`.
- Fixed a string deserialization problem.
- Fixed a deserialization bug that can occur in a client-server combination with gSOAP and Apache Axis.

Version 2.1.9 (9/08/2002)
---
- Added HTTP proxy authentication.
- Added plug-in registry (Beta).
- Changed compiler exit status for compilation errors.
- Fixed stdin/stdout binary mode for DIME transfer with gSOAP for win32.
- Fixed win32 soapcpp2.exe bug in .res.xml file generation.

Version 2.1.10 (10/14/2002)
---
- Added `//gsoap ns service method-documentation:` directive.
- Added provision for additional documentation text with `//gsoap ns service name:` directive.
- Added #import pragma to (recursively) import gSOAP header files.
- Added plugin features and included a plugin example in 'extras' directory.
- Added automake/autoconf to generic gSOAP package (thanks to Christian Aberger).
- Added `-DWITH_LOCALTIME` compilation option to control `time_t` (de)serialization.
- Changed `time_t` (de)serialization (now uses timegm() and gmtime() to handle UTC). Found that mktime() library call is buggy on some systems.
- Improved and reorganized package directory structure.
- Fixed DDL linkage problem with instantiate/delete routines dealing with class instance memory management.
- Fixed compilation problem with Sun Workshop CC compiler and other compilers that adopt a specfic class VMT structure that could break object (de)serializers.
- Fixed bug in WSDL generator with multiple service namespaces.

Version 2.1.11 (11/10/2002)
---
- Added a multi-functional SOAP router application (message forwarding and message relay server).
- Added keep-alive support for asynchronous one-way messages.
- Improved parsing and handling of function prototypes and class methods.
- Improved modular design to enable the compilation and linkage of multiple client and service modules.
- Improved user-defined SOAP Fault detail handling.
- Fixed `SSL_accept` bug.
- Fixed serialization of pointers to dynamic arrays with multi-references.

Version 2.2 (12/12/2002)
---
- Added XML attribute (de)serialization support.
- Added XSD QName parsing and conversion support (`typedef char *xsd__QName`).
- Added compression support (`-DWITH_ZLIB` requires Zlib).
- Changed and improved transport and encoding settings by splitting up sending and receiving side flags into separate sets. Deprecated `soap.enable_X` and `soap.disable_X` flags.
- Improved keep-alive support (automatic client and server sides).
- Fixed a bug in mustUnderstand handling.
- Fixed a sporadic crash of the gSOAP compiler under win32.
- Fixed user-defined SOAP Fault output in WSDL generator.

Version 2.2.1 (12/18/2002)
---
- Added callbacks to enable custom streaming of DIME attachments.
- Fixed broken serialization of pointers to dynamic arrays with NULL `__ptr`.
- Fixed some WSDL issues.

Version 2.2.2 (1/25/2003)
---
- Added gzip support (`-DWITH_GZIP` requires Zlib).
- Added `faccept()` callback.
- Improved HTTP chunking.
- Fixed OpenSSL and `accept_timeout` (OpenSSL requires blocking sockets).
- Fixed HTTP header buffering.
- Fixed UTF8 decoding of a subset of characters (2-octet UTF).
- Fixed `operator==` parsing.
- Fixed a couple of WSDL issues.

Version 2.2.3 (3/2/2003)
---
- Added server-side HTTP basic authentication.
- Improved speed.
- Improved Tru64 portability.
- Changed `fpost()` function callback signature: added `int port` argument and added port number to the Host: HTTP header.
- Fixed memory leak in SSL connect when using OpenSSL DH.
- Fixed custom (de)serializer definitions parsing.
- Fixed a gzip/deflate bug with large transfers.
- Fixed use of `ftime()` for `time_t` (de)serialization with timezones.
- Fixed a problem with concurrent access to global namespace table by multiple threads.
- Fixed crash with //gsoap name directive.
- Fixed deserialization bug with multi-ref object encoding by Apache Axis.

Version 2.3.1-8 (9/24/2003)
---
- Updated SOAP 1.2 support (implements the SOAP 1.2 recommendation).
- Added STL container support and built-in `std::string` encoding.
- Added stand-alone Web server example to serve static and dynamic pages with a new HTTP GET plugin.
- Added `void*` (de)serialization, e.g. useful to support polymorphism with C applications and as an alternative to union types.
- Added XML DOM parser to support SOAP document encoding and for mixing of application data (de)serialization within an XML DOM.
- Added `WITH_LEAN` and `WITH_LEANER` compilation flags to improve support for small-memory devices such as WinCE and Palm.
- Added `SOAP_XML_CANONICAL` flag for canonical XML output (XML-C14N support and SOAP 1.2 normalization support).
- Added `//gsoap ns method-encoding:` directive.
- Added `//gsoap ns schema import:` directive.
- Added optional class getter and setter methods for object (de)serialization.
- Added `volatile` qualifier to indicate that certain type declarations should be treated 'as-is' when types are declared externally (e.g. part of a library).
- Added a new string notation for declarations to specify simpleType restriction patterns.
- Added soapcpp2 -I option to indicate #import path.
- Added support for (de)serialization of application data to C++ iostreams.
- Added `fsslauth()` callback.
- Added HTTP code error values.
- Added `SOAP_MALLOC` and `SOAP_FREE` macros to replace `malloc()` and `free()`.
- Added `soap_strdup(soap, string)` to copy strings in gSOAP's memory space.
- Added chunked streaming DIME output (thanks to J. Li for his suggestions).
- Added `SOAP_SOCKET` for win32 support (thanks to B. Gille for his suggestions).
- Added `SO_LINGER` support with `soap.connect_flags`.
- Added support for iso-8859-1 encoded XML parsing (default is utf8).
- Added #module directive to build shared libraries.
- Added C++ namespaces support to enable the compilation and linkage of multiple client and service codes.
- Added `WITH_CASEINSENSITIVETAGS` compilation flag to enable case insensitive XML parsing.
- Improved WSDL types schema output.
- Improved custom (de)serialization (note: extras/stringtest example has been changed).
- Improved SOAP literal encoding.
- Improved speed.
- Improved HTTP1.0/1.1 support.
- Removed `soap.defaultNamespace` in favor of automatic SOAP literal output based on `soap.encodingStyle` value.
- Fixed GMT to DST localtime conversion, but still issues with Sun Solaris.
- Fixed dynamic array `__offset` handling.
- Fixed class definition order.
- Fixed sporadic memory leak in HTTP cookie reader.
- Fixed multi-threaded server-side OpenSSL initialization.
- Fixed enumeration-based XML attribute deserialiation.
- Fixed *`SOAP-ENC:position`* attribute placement when arrays of pointers are mixed with nillable data.
- Fixed crash when parsing incorrect SOAP/XML attribute content.

Version 2.4 (12/20/2003)
---
- New WSDL parser and importer with improved support for SOAP document style and literal encoding.
- Added optional full schema validation (partial by default). Use `SOAP_XML_STRICT` flag.
- Added SSL session caching.
- Added `soap_set_namespaces()` to switch namespace tables.
- Fixed plugin memory leak. REQUIRED CHANGE to plugin allocation and copying.
- Fixed server-side blocking SSL accept.

Version 2.4.1 (01/07/2004)
---
- Changed `_USCORE_` and `_DOT_` naming conventions into `_USCORE` and `_DOT` to improve name mapping.
- Updated DOM parser (integrated into core gSOAP library).
- Improved WSDL parser and included several new features.
- Fixed WSDL schema import and WSDL literal output.

Version 2.5 (01/28/2004)
---
- Changed default encoding to SOAP RPC literal, as mandated by WS-I Basic Profile 1.0a and best practices.
- Added soapcpp2 compiler option -e for backward compatibility to gSOAP 2.4.1 and earlier that use SOAP RPC encoding defaults (helpful when migrating gSOAP projects to 2.5).
- Added automatic compiler-side WS-I Basic Profile 1.0a compliance warnings (warns before deployment).
- Added `fget()` callback for HTTP GET responses.
- Added IPv6 support contributed by Wind River Systems.
- Added VxWorks support contributed by Wind River Systems.
- Improved SOAP Fault handling.
- Improved interop with Axis RPC encoded response messages.
- Fixed `std::string` *`xsi:type`* attribute value.
- Fixed gSOAP 2.4 problems with serialization of enumeration values outside enumeration range.
- Fixed gSOAP 2.4 Pocket PC build problem.
- Fixed wsdl2h SOAP Fault and Header output.

Version 2.5.1 (02/12/2004)
---
- Improved WSDL output and WS-I BP1.0a compliance.
- Improved WSDL parser stability.
- Fixed problem parsing empty *`xsd:base64Binary`* elements.
- Fixed VxWorks and TRU64 portability issues.
- Fixed HTTP 400 error problem.
- Fixed soapcpp2 2.5 stability problem with one-way messages.

Version 2.5.2 (02/23/2004)
---
- Fixed WSDL generation warnings.
- Fixed LONG64 problem for PalmOS.
- Fixed module build link problem and 'components' example.

Version 2.6.0 (03/28/2004)
---
- Changed default style to doc/lit.
- Improved doc/lit WSDL handling.
- Improved soapcpp2 sample SOAP/XML message output.
- Added proxy support for wsdl2h.
- Fixed CONNECT method for proxy+SSL+gzip.

Version 2.6.1 (06/07/2004)
---
- Added optional multibyte character support.
- Improved WSDL output.
- Fixed STL container multi-ref deserialization problem with missing data.
- Fixed doc/lit server code generation from doc/lit WSDL definitions.

Version 2.6.2 (06/12/2004)
---
- Added Matlab(tm) code output (documentation is not availabel yet).
- Improved wsdl2h output for schema facets.
- Updated dom.cpp.
- Changed wsdl2h `std::string*` to `std::string` generation (to avoid double pointers `std::string**`). To obtain old behavior, add entry `xsd__string = | std::string* | std::string*` to typemap.dat.
- Fixed wsdl2h handling of *`xs:include>`*.
- Fixed wsdl2h handling of empty namespaces and absent targetNamespace schema attributes.
- Fixed MB char handling.

Version 2.7.0a/b/c/d/e/f (09/10/2004, 2/1/2005)
---
- Added multipart/related support for SOAP with MIME attachments (SwA).
- Added API functions for DIME open and closed layout support.
- Added `//gsoap schema elementForm:` and `//gsoap schema attributeForm` directives.
- Added `fclosesocket`, `fshutdownsocket`, `fpoll`, `fserveloop` callbacks.
- Added `soap.authrealm` string attribute to set basic authentication realm.
- Added `soap.prolog` string attribute to output optional XML prolog with processing instructions and directives.
- Added soapcpp2 option -t.
- Added wsdl2h options -e and -i.
- Added Palm OS and Symbian instructions and examples.
- Added xml-rpc support and examples.
- New `mod_gsoap` directory with modules for Apache 1.x/2.x, IIS, and WinInet
- Improved XML validation with `SOAP_XML_STRICT` flag.
- Improved memory footprint (reduced code size and reduced serialization overhead).
- Improved runtime id-ref serialization and deserialization.
- Improved wsdl2h translation.
- Updated dom.cpp.
- Updated webserver.c.
- Fixed issues with namespace qualification in doc/lit messages.
- Fixed `SOAP_HREF` error.
- Fixed proxy+SSL+zlib+chunking.
- Fixed SSL `select()` polling in `frecv()`.
- Fixed wsdl2h multidimensional SOAP arrays generate problem.

Version 2.7.1 (03/31/2005)
---
- Added new style of C++ proxy/object generation (objects derived from soap struct), enabled with soapcpp2 option -i.
- Added new features to wsdl2h generation to enable user-defined class/struct extensions such as member data and functions which are declared in typemap.dat.
- Added MIME multipart/form-data parsing to support HTTP POST HTML form handling (example included in samples/webserver).
- Added built-in memory leak detection enabled with `-DDEBUG`.
- Added UDDI v2 API and examples.
- Added 'WS' directory with WS protocols, such as WS-Addressing. This part will mature as more WS protocols will be added over time.
- Added `fseterror()` callback.
- Improved wsdl2h code output.
- Enhanced Web server sample code, and httpget and logging plugins.
- Fixed base64/hexBinary XML attribute serialization.
- Fixed a missing min/maxOccurs check for validation.

Version 2.7.2 (05/11/2005)
---
- Added SOAP-over-UDP IPv4/IPv6 support with examples.
- Added UDDI v2 API publish example.
- Improved wsdl2h parser accepting multiple WSDL/XSD files from the command line.
- Fixed wsdl2h schema import from included schema bug.
- Fixed SOAP 1.2 Fault Subcode handling.

Version 2.7.3 (06/27/2005)
---
- Added MTOM attachment support.
- Added 'plugin' directory with example plug-ins.
- Added 'import' directory for commonly #import-ed files such as stlvector.h.
- Added HTTP digest authentication plug-in 'plugins/httpda.h'.
- Added new import feature to wsdl2h using `ns=URI` convention for typemap.dat files, which enables wsdl2h to parse a WSDL that imports known WSDLs/schemas such as XOP, XML MIME, and WS-Addressing without actually reading the imported WSDL/schemas.
- Added soapcpp2 options -C and -S to generate client- or server-only codes.
- Added multi-path support for soappcpp2 option -I.
- Added C and C++ examples to combine multiple clients and services into one executable, see the samples/components directory in the package.
- Improved soapcpp2 option -e to generate SOAP RPC encoding style client/server codes by default (instead of document/literal by default).
- Fixed wsdl2h circular import of schemas.
- Fixed attributeGroup handling in wsdl2h.
- Fixed soapcpp2 handling of 'invisible' tags, e.g. `__any`, that should only be populated after all else failed to match the delivered XML content.
- Fixed validation of numeric enum constants with `SOAP_XML_STRICT` validation.
- Fixed engine's spurious log file generation issue.

Version 2.7.4 (07/24/2005)
---
- Added union serialization.
- Added `//gsoap ... type-documentation:` directive.
- Added `//gsoap ... method-mime-type:` directive.
- Added wsdl2h option -g for generating global element/attribute declarations for building modular .h files.
- Added wsdl2 option -y for generating typedef synonyms for structs and enums, with improved handling of typedefs by soapcpp2.
- Added wsdl2 option -u to disable union generation for *`xs:choice`*.
- Improved wsdl2h documentation generation.
- Improved wsdl2h and soapcpp2 MIME bindings.
- Improved dynamic memory allocation in engine with look-aside buffering.
- Fixed wsdl2h *`xs:complexContent/extension/attributeGroup`* translation.
- Fixed wsdl2h *`xs:choice`* with min/maxOccurs != 1 translation.
- Fixed wsdl2h *`soap:body parts`* attribute.

Version 2.7.5 (08/01/2005)
---
- Added *`xs:redefine`* processing to wsdl2h.
- Added HTML POST application/x-www-form-urlencoded plugin httpform.c (demo samples/webserver).
- Improved wsdl2h output.
- Fixed wsdl2h handling of duplicate values in single enum list.

Version 2.7.6 revision a/b/c/d/e (08/19/2005-02/18/06)
---
- Added WS-Security authentication, tokens, and signatures with new 'wsse' plugin, see documentation in 'doc' directory and 'samples/wsse'.
- Added 'doc' directory for documentation and moved soapdoc2.html and soapdoc2.pdf.
- Added `SOAP_XML_INDENT` flag.
- Added `soap_sender_fault_subcode()` and `soap_receiver_fault_subcode()` for SOAP 1.2 subcodes.
- Added `WITH_DOM` compile flag to enable flushing serialized and deserialized data to DOM (as well as a stream).
- Added soapcpp2 option -L.
- Added soapcpp2 option -a.
- Added wsdl2h option -a (for backward compatibility).
- Added wsdl2h option -d.
- Added `WITH_TCPFIN` comile flag to enable optional shutdown with how=1 (TCPFIN) after final sends to force EOF on other side (used to be the default behavior).
- Added schema substitutionGroup support.
- Added `soap_ssl_init()`.
- Improved DOM implementation and documentation.
- Improved AS400 portability.
- Improved schema choice support.
- Fixed wsdl2h multiple schema include issue.
- Fixed `soaps2dateTime` and `soap_timegm` functions when `timegm` is not available.
- Fixed exc-c14n formatting.
- Fixed SOAP 1.2 fault handling.
- Fixed missing `soap_flag___item2` issue.
- Fixed partial MIME boundary parsing issue.
- Fixed base64 parsing with `WITH_FAST`.
- Fixed MIME encoding of `\r\r` sequence.
- Fixed QName normalization issue.
- Fixed relative path schema import.
- Fixed MTOM cid matching with URL encoded IDs.
- Fixed wide-character (`wchar_t*`) XML attribute handling.
- Fixed `std::vector` element id-ref ordering in deserializer.

Version 2.7.7 (04/07/2006)
---
- Added streaming MTOM support (see also example in samples/mtom-stream).
- Added `long double` serialization support with custom serializer (`custom/long_double.h`).
- Added automatic detection of application's `soap_malloc()` data overruns in `-DDEBUG` mode.
- Improved `time_t` and `struct tm` (`custom/struct_tm.h`) serialization.
- Fixed issue with a wrongly generated namespace map entry.
- Fixed PalmOS socket connection issues.
- Fixed XML attributes with bitmask enumeration (`enum*`) values.

Version 2.7.8 revisions a/b/c (06/05/2006-06/24/2006))
---
- Added X509 extension checks.
- Added advanced MTOM streaming features.
- Added WS-Addressing plugin and demo.
- Moved soapcpp2 and wsdl2h to bin directory.
- Improved SOAP1.2 MTOM interop.
- Changed `soap_send_empty_response` function signature (additional status argument) for HTTP optional response binding.
- Changed `fmimewriteopen()` callback signature.
- Fixed wsdl2h top-level element and complexType name clash issue.
- Fixed HTTP chunking with empty response message body.
- Fixed WSSE digest authentication.
- Fixed WSDL response element output.
- Fixed SOAP 1.2 *`RPC:result`* handling with `SOAP_XML_STRICT` flag.
- Fixed WSSE signed response messages.
- Fixed Win32 max open connection exceeded checking.
- Fixed *`xsi:type`* matching for `__name` fields, which required a signature change of `soap_element_begin_in()` with an additional 'type' parameter (affecting custom deserializers, see custom dir).

Version 2.7.9 revisions a/b/c/d/e/f/g/h/i/j/k/l (10/24/2006-09/26/2007)
---
- Added wsdl2h option -j to omit the generation of SOAP Header and Faults for customization of these.
- Added wsdl2h option -q to generate C++ namespace qualified header files.
- Added `SOAP_SSLv3`, `SOAP_TLSv1`, `SOAP_SSL_SKIP_HOST_CHECK` flags.
- Added file input and output specifications (with `<` and `>`) to wsdl2h's typemap.dat to specify input and output for wsdl2h.
- Added WS-Addressing 2005/03 support.
- Added function `soap_free(soap)` that executes `soap_done(soap); free(soap)` and renamed previous `soap_free()` to `soap_temp_free()`.
- Added wsdl2h option `-_` to translate `_USCORE`.
- Added *`xsd:anyAttribute`* (using DOM parser) with wsdl2h option -d.
- Added multi-WSDL types section schema component merging.
- Added TCP keep-alive settings.
- Added QName list normalization.
- Changed soapcpp2 default rule for generating attribute form qualified to unqualified.
- Improved wsdl2h SOAP Fault coding.
- Improved performance and stability.
- Improved portability.
- Improved wsdl2h empty namespace handling ("" namespace).
- Improved wsdl2h and schema conversion to C and C++.
- Improved SSL timeouts and error handling.
- Improved soapcpp2 option -i (generate proxy and service classes).
- Improved class instance initialization.
- Improved use of IPv6.
- Improved proxy and server code output for soapcpp2 option -i.
- Improved soapcpp2 code output with additional exception checks.
- Fixed MIME/MTOM start id handling (SOAP message no longer required to be first attachment).
- Fixed WS-Security wsse plugin canonicalization format issue.
- Fixed ZIP content remainder parsing (CRC check).
- Fixed WSA API handling of messages with MIME/DIME attachments.
- Fixed wsdl2h *`xs:include`* relative path includes.
- Fixed `_XML` literal string parsing of root attributes.

Version 2.7.10 (01/27/2008)
---
- Combined pre-built win32, Linux i386, and Mac universal binaries into a single software distribution package to streamline the distribution.
- Added new Web service examples and removed XMethods dependence.
- Added soapcpp2 option -qname for automatic C++ namespace qualification.
- Added HTTP PUT, DELETE, HEAD callback support.
- Added ability to derive classes from soap struct (in gSOAP header files).
- Updated certificates and fixed cacerts.c.
- Improved soapcpp2 option -i proxy/server object generation.
- Improved socket timeout handling.
- Improved HTTP chunking handling and efficiency.
- Improved web server implementation.
- Improved soapcpp2 and wsdl2h warning and error reporting.
- Fixed dom parser handling of default xml namespace bindings.
- Fixed wsdl2h default ("") namespace handling.

Version 2.7.11 (07/26/2008)
---
- Added new wsseapi signature verification API functions.
- Added mutability property of `SOAP_ENV__Header` and `SOAP_ENV__Fault` struct to allow consistent redefinitions and on-the-fly member additions.
- Added wsdl2h -r option parameter for web proxy access.
- Added MINGW portability.
- Added configure option --disable-openssl to build wsdl2h and libraries without OpenSSL.
- Added `WITH_C_LOCALE` compilation flag to enable the use of `c_locale` and local-specific numeric conversion routines.
- Added WS-Addressing metadata (wsam) support.
- Fixed *`xml:lang`* in SOAP 1.2 *`SOAP-ENV:Text`*.
- Fixed XML literal strings with maxOccurs="unbounded"
- Fixed SSL connection engine issue.
- Fixed RPC literal part name handling issue.
- Fixed in update: SSL alloc/free issue and s2dateTime.

Version 2.7.12 (10/01/2008)
---
- Added multi-endpoint connect capability. Endpoint string can be a space-separated list of endpoint URLs, where each URL is tried in turn.
- Added `SOAP_SSL_NO_DEFAULT_CA_PATH` option.
- Fixed SSL connect issue when the SSL handshake or network connection fails.
- Fixed *`xsi:nil`* handling for strings (NULL vs. "") in case of empty elements.
- Fixed DOM xmlns="" namespace issue in XML output.
- Fixed `IP_MULTICAST_TTL` failure to compile.
- Fixed non-critical WSSE API `soap_wsse_verify_nested` code.

Version 2.7.13 (03/21/2009)
---
- Improved wsdl2h code output.
- Updated gsoapWinInet.
- Updated DOM xmlns namespace binding handling.
- Fixed OpenSSL memory leak.

Version 2.7.14 (09/07/2009)
---
- Added support for `poll()` to replace `select()` to prevent `FD_EXCEEDED`.
- Fixed enum local simpleType restriction attribute initialization.
- Fixed utf8 string parsing issue when first char in CDATA could cause problems.
- Fixed UDP IPv6 multicast-unicast exchange.
- Fixed gzip end-of-stream check.
- Fixed wsdl2h portType operation-input/output matching to ensure operations can be safely overloaded.

Version 2.7.15 (11/07/2009)
---
- Added `soap_read_X()` and `soap_write_X()` macros to simplify object reading and writing.
- Fixed `soap_new_X()` functions compilation problem.
- Fixed SSL session in context copy.
- Fixed win32 `select()` in `tcp_select()`.
- Fixed wstrings in unions.
- Fixed preservation of CDATA sections in literal XML strings.

Version 2.7.16 (04/05/2010)
---
- Added soapcpp2 option -T to generate auto-test echo server code with an example in samples/autotest based on 294 patterns for data binding W3C workgroup.
- Added soapcpp2 option -b to serialize byte arrays char[] as fixed-size strings (array of *`xsd:byte`* otherwise).
- Added *`xsd:duration`* serializer: import custom/duration.h.
- Added gsoap header syntax '$' type qualifier to indicate special struct/class members (array length, union discriminators).
- Added "colon notation" to attach XML prefixes to C/C++ names without changing the names, an alternative to the `ns__` prefix notation by writing `ns:name` instead of `ns__name` where the former stands for just `name` to the C/C++ compiler.
- Added wsdl2h options -z# for backward compatibility flags (-z1, -z2).
- Added `SOAP_XML_IGNORENS` to optionally ignore XML namespaces.
- Added WS-ReliableMessaging plugin.
- Added custom *`xsd:duration`* serializer.
- Added WSSE plugin `soap_wsse_set_wsu_id()` to simplify signing of elements (sets the wsu:Id of a given element tag name).
- Improved handling of struct/class member name clashes resulting from XML schemas definitions.
- Improved SOAP RPC ecoding response element tag acceptance.
- Improved VxWorks compatibility.
- Fixed FastCGI SOAP/XML processing with MIME attachments.
- Fixed `typedef std::string XML`, for literal XML serialization problem.
- Fixed a rare incorrect well-formedness error when xmlns default namespaces are absent.
- Fixed a zlib compression error when obsolete crc check is used.
- Fixed WS-Addressing HTTPS message relay (OpenSSL crash).

Version 2.7.17 (05/10/2010)
---
- Added WS-Policy 1.2 and 1.5, WS-SecurityPolicy 1.2 support for wsdl2h.
- Improved WS-ReliableMessaging interop and verified MT-safety.
- Improved WS-Security WSSE plugin documentation.
- Fixed soapcpp2 line number error reporting.
- Fixed WSSE plugin `soap_wsse_set_wsu_id()`.
- Fixed wsdl2h cycle problem with recursive *`xs:include`* and *`xs:import`*.
- Fixed enum translation of enum symbols with colons.
- Fixed `std::wstring` used as XML attribute value UTF8 encoding (internal stdsoap2 API changed).

Version 2.8.0 (09/20/2010)
---
- Added wsdl2h option -W: suppress warnings.
- Added wsdl2h option -i: for advanced uses only to skip schema imports.
- Added automatic *`xsi:nil`* output for NULL pointer members when minOccurs>0 (member has '1' occurence annotation).
- Improved wstring and `wchar_t*` handling when `wchar_t` is less than 4 bytes.
- Improved use of custom serializers in builds.
- Fixed wsdl2h type mapping of nested *`xs:sequence`* and *`xs:choice`*.
- Fixed soapcpp2 handling of extension base qualified elements and derived type unqualified elements.
- Fixed WSRM message resend buffer leak.
- Licensing change for reasons of consistency and transparency: the soapcpp2 tool (2.8.0 and up) is licensed under the same conditions as the wsdl2h tool, which means that soapcpp2 is no longer covered by the gSOAP public license.

Version 2.8.1 (01/14/2011)
---
- Added WS-Security WSSE plugin-integrated streaming XML encryption.
- Added WS-Security WSSE plugin `soap_wsse_sign_only()`.
- Added soapcpp2 -f flag to split large output source code files into parts.
- Fixed C++ mapping of *`xs:sequence`* with maxOccurs larger than 1.

Version 2.8.2 (04/17/2011)
---
- Added soapcpp2 -j option to generate C++ proxies and services in alternative format to the -i option.
- Fixed wsdl2h issue with optional elements with default value.
- Fixed cyclic WSDL import dependences in wsdl2h.
- Fixed WSRM server-side fault state handing.
- Fixed leading unicode character parsing and conversion issue in literal XML strings.

Version 2.8.3 (06/24/2011)
---
- Added soapcpp2 -y option to generate C/C++ type access information to the generated sample XML messages, allowing for the verification of C/C++ data access to the actual XML content.
- Added JSON support in addition to XML-RPC, see the XML-RPC content in the samples folder.
- Improved DOM processing (an optional feature).
- Fixed bugs, compilation and porting issues.

Version 2.8.4 (10/24/2011)
---
- Added NTLM support (enabled with `-DWITH_NTLM`, requires libntlm).
- Added WS-Discovery 1.1 support (see doc/wsdd and plugin/wsddapi.h and wsddapi.c).
- Added `//gsoap` typed directive to control *`xsi:type`* attribuation per namespace.
- Added `SOAP_XML_NOTYPE` flag to disable *`xsi:type`* attributes.
- Improved WS-Security encryption (AES, `soap_wsse_add_EncryptedKey_encrypt_only`).
- Improved HTTP proxy authentication support (digest and NTLM).
- Fixed bugs and portability issues.

Version 2.8.5 (11/19/2011)
---
- Fixed build issues with WS-Discovery.
- Fixed HTTP basic auth crash.
- Fixed `WITH_COMPAT` and `WITH_C_LOCALE`.
- Fixed wsdl2h output for XSD abstract element extraneous ';'.

Version 2.8.6 (12/14/2011)
---
- Improved WS-Security compatibility.
- Fixed multi and cyclic WSDL import.
- Fixed tags of qualified referenced elements.
- Fixed C builds for improved portability.

Version 2.8.7 (02/07/2012)
---
- Added new REST examples.
- Added versioning checks to ensure project builds are consistent.
- Improved interoperability of wsdl2h output wrt. to element qualification.
- Renamed `-DTANDEM` to `-DTANDEM_NONSTOP` to prevent naming conflicts.
- Fixed WS-Discovery URL.
- Fixed soapcpp2 option -i and -j server chaining.

Version 2.8.8 (02/20/2012)
---
- Added REST plugin (httppost.c) enhancements and examples.
- Added static version checks to prevent linkage to old incompatible libraries that could lead to runtime failures.

Version 2.8.9 (06/10/2012)
---
- Added examples to interoperate with WCF basicHttp, basicTransportSecurity, basicMessageSecurity, and wsDualHttp.
- Added soapcpp2 option -A for service dispatching based on Action values only.
- Added wsdl2h option -z3.
- Added wsse support for WS-SecureConversation.
- Added wsrm5 WS-ReliableMessaging 1.0 2005.
- Added iOS plugin with examples.
- Improved WS-Policy analysis and reporting.
- Fixed and improved wsse plugin for WS-Security interoperability with WCF and other implementations (required a small API change).
- Fixed HTTP headers for NTLM.
- Fixed httpda plugin OpenSSL init crash in md5evp.c.

Version 2.8.10 (08/16/2012)
---
- Added soapcpp2 -Q option (C++ namespaces).
- Added wsdl2h -b option (bi-directional operations for duplex services).
- Updated WS-Addressing and WS-ReliableMessaging plugins, documentation, and examples.

Version 2.8.11 (10/14/2012)
---
- Added soapcpp2 -0 option to remove SOAP bindings when applicable.
- Changed wsdl2h output for the few cases that multiple service bindings are defined, use wsdl2h -Nns for backward compatibility.
- Improved DOM processing.
- Improved C++0x/C++11 compatibility.
- Fixed httpda plugin crash.
- Fixed wsdl2h processing of *`xs:group maxOccurs="unbounded"`*.
- Fixed `mod_gsoap` plugin compilation issues.
- Fixed literal XML UTF8 (UTF8 is now retained with `SOAP_C_UTFSTRING`).

Version 2.8.12 (12/8/2012)
---
- Added WSDL 2.0 support to wsdl2h.
- Added //gsoap method-protocol directive.
- Improved XML REST support, soapcpp2 -0 option for "no-SOAP" services.
- Improved class instantiation with `soap_new_X`, `soap_new_set_X`, `soap_new_req_X`.
- Improved soapcpp2 -b option.
- Improved WCF WS-Addressing interop (duplex) channel.
- Changed code to normalize in/out MEP for solicit-response messaging.
- Fixed client-side OpenSSL host check that is known to crash in OpenSSL.

Version 2.8.13 (1/21/2013)
---
- Improved XML-RPC/JSON API and documentation, with one change: now must use `SOAP_C_UTFSTRING` to hold UTF8 in 8-bit strings.
- Improved VxWorks compatibility.
- Updated `soap_read_X` and `soap_write_X` to serialize object graphs with SOAP encoding enabled: use `soap_set_version(soap, V)` with V=0 (no SOAP), V=1 (SOAP1.1), V=2 (SOAP1.2).
- Fixed broken WSDL *`types`* nested schema XSD import.
- Fixed operation action overriding by input/output SOAP action.
- Deprecated old-style C++ service proxies and objects (use soapcpp2 -z1 flag to generate).

Version 2.8.14 (2/4/2013)
---
- Added explanation of wsdl2h option -Nname which is used to generate a service (and its prefix name) for each service binding (bindings are collected in one service by default).
- Fixed PRId64 formatting.
- Fixed broken MIME with doc/lit (when SOAP encodingStyle is not used).

Version 2.8.15 (5/12/2013)
---
- Added wsdl2h option -R for auto-generation of REST-based service operations (also added examples/rest example).
- Change `soap_read_X()` and `soap_write_X()` to prevent removal of HTTP headers after calling `soap_connect()`.
- Fixed automatic detection of DIME/MIME transfers.
- Fixed HTTP 1.0 chunking issue.
- Fixed HTTP digest authentication with DIME/MIME transfers.
- Fixed OpenSSL subject alt name check.
- Fixed HTTP 100 message handling issue.

Version 2.8.16 (8/12/2013)
---
- Improved support for Android platform.
- Improved WCF interop duplex messaging requiring ChannelInstances.
- Fixed complexType restriction of schema types with redefined attributes in wsdl2h output.
- Fixed wcf/WS/DualHttp example (wrong use of `send_X()` replaced by `X()` for client-side operations X, to allow HTTP keep-alive to be used).
- Fixed use of sprintf's that trigger Valgrind and Fortify warnings.
- Fixed parsing character strings from CDATA sections ending in ']'.
- Fixed DOM output for `SOAP_XML_CANONICAL` c14n normalization of xmlns namespace bindings.

Version 2.8.17r (12/1/2013)
---
- Added WS-RM message queueing option for WS-RM NoDiscard.
- Added wsdl2h option -r for authorized web access and proxy access.
- Improved C++11 compatibility.
- Improved ISO compatibility (compile -pedantic).
- Improved I/O timeouts timing accuracy in the presence of EINTR interrupts (set `SOAP_MAXEINTR`).
- Fixed in 2.8.17r: fixed socket timeout issue.

Version 2.8.18 (8/24/2014)
---
- Added *`xs:precision`* and *`xs:scale`* support.
- Added support for BPEL PartnerLink and VariableProperties to wsdl2h in support of BPEL projects.
- Added soapcpp -z2 flag for backward compatibility with to 2.7.x: omit XML output of elements for pointers that are NULL even when minOccurs=1 (required).
- Changed wsdl2h C++ output to always generate pass-by-reference response parameters (disable with -z6).
- Improved WS-RM interoperability.
- Fixed message part elements and types in default (xmlns="") namespace.
- Fixed `&#;`-entity code conversion bug in XML attribute deserialization.
- Fixed wsdl2h nested simpleType restriction with itemTypes.
- Fixed wsdl2h substitutionGroup element in other namespace.
- Fixed *`xs:choice`* strict validation.
- Fixed timeout/EINTR issue.

Version 2.8.19 (11/8/2014)
---
- Added string validation callbacks fsvalidate and fwvalidate for regex pattern-based validation of strings and wide strings against XML schema regex patterns.
- Fixed *`xs:choice`* strict validation with *`minOccurs="0"`* element(s).
- Fixed `TCP_FASTOPEN` with UDP.
- Fixed MTOM Content-Disposition ID overriding problem.
- AIX portability fixes.

Version 2.8.20 (11/30/2014)
---
- Added XML 1.1 control char encoding support.
- Fixed C/C++ compiler warnings.

Version 2.8.21 (12/05/2014)
---
- Added `-DWITH_REPLACE_ILLEGAL_UTF8` compilation option to replace UTF8 content that is outside the allowed range of XML 1.0.
- Improved WS-RM (memory usage).
- Fixed additional C/C++ compiler warnings.
- Fixed `strftime` `%F` and `%T` use (removed).

Version 2.8.22 (4/12/2015)
---
- Added XSD 1.1 support (*`xs:override`*, *`xs:defaultAttributes`*, *`xs:targetNamespace`* on element and attribute declarations, and other additions).
- Added import/wsdd5.h to support WS-Discovery 1.0 with WS-Addressing 2005/08 (typemap.dat now sets wsdd5.h instead of wsdd10.h by default).
- Improved portability, including Solaris and AIX porting.
- Improved option -R is now required to enable REST WSDL bindings and non-HTTP transport (in addition to enabling REST WSDL operations).
- Fixed wsdl2h handling of *`xs:redefine`*.
- Fixed deep class inherited `xsd__anyType` deserializer issue causing serialization and validation errors.
- Fixed `std::vector*` deserialization bug that occurs under very specific circumstances.
- Fixed wsdd plugin `SOAP_WSDD_APP_MAX_DELAY`.
- Fixed wsse/smdevp memory leak.

Version 2.8.23 (8/17/2015)
---
- Added TLS SNI.
- Added Unicode C++11 identifier support to soapcpp2, supported directly by UTF8 encoding and the UCN `\uNNNN` extension of identifier naming in the C99/C98++ standards.
- Added wsdl2h option -U to allow UTF8-encoded Unicode C/C++ identifiers upon mapping universal XML tag names to C/C++ identifier names (this is a C++ standard-dependent and compiler-dependent feature).
- Replaced C lib functions `memcpy`, `memmove`, `strcpy`, `strcat`, `strncpy`, `strncat`, `sscanf`, `sprintf` by more secure alternatives when available on target platform.  On windows you should add an [Invalid Parameter Routine](https://msdn.microsoft.com/en-us/library/ksazx244.aspx), to ignore and let gsoap safely continue with truncated strings (and deal with soft errors later).
- Updated OpenSSL and GNUTLS integration.
- Fixed soapcpp2 SOAP RPC literal WSDL output (supported are SOAP RPC encoded, RPC literal, document literal, and REST).
- Fixed C++ struct soap context constructor memory leak when `WITH_GZIP` is enabled and changed struct soap context allocation in C++.
- Fixed redundant call to `OpenSSL_add_all_digests()`, since `OpenSSL_add_all_algorithms()` already adds digests. This caused issues on certain platforms with OpenSSL 1.0.1 and higher.
- Fixed missing client-side `soap_recv_fault` with empty doc/lit responses.
- Fixed missing *`xs:restriction`* tag in soapcpp2 WSDL/XSD output.

Version 2.8.24r (11/01/2015)
---
- Added new soapcpp2 option `-c++11`, enabling support for C++11 syntax and features, including:
    - scoped enumerations (`enum class`) with underlying types (`enum class : type` is 8, 16, 32, or 64 signed int);
    - serialization of smart pointers `std::shared_ptr` and `std::unique_ptr` (and deprecated `std::auto_ptr` if needed);
    - virtual method `final` and `override` syntax;
    - `nullptr` constant member initializer.
    -  More efficient C++11 STL vector, list, and deque deserialization code based on `emplace_back()`.
- Added new `xsd__date` custom serializer `custom/struct_tm_date.h` to bind `struct tm` to *`xsd:date`*.
- Added new `xsd__time` custom serializer `custom/long_time.h` to bind `ULONG64` to *`xsd:time`* with microsecond precision.
- Added new `xsd__duration` custom serializer `custom/chrono_duration.h` to bind C++11 `std::chrono::nanoseconds` to *`xsd:duration`*.
- Added `xsd__dateTime` custom serializer `custom/chrono_time_point.h` to bind C++11 `std::chrono::time_point` to *`xsd:dateTime`*.
- Added new TLS/SSL flags `SOAP_TLSv1_0`, `SOAP_TLSv1_1`, and `SOAP_TLSv1_2` to limit TLS protocol selections.
- Added new SHA224/SHA384 digest options, ECDSA (requires OpenSSL 0.9.8 or later), and HMACOutputLength truncation (within limits) to WSSE.
- Added Galois counter mode (GCM) authenticated encryption to WSSE with AES128/AES192/AES256, requires OpenSSL 1.0.2 or later.
- Added the concept of "mutable" structs and classes, which allow for incremental compile-time addition of new members to the struct/class, which gives them a tuple-like behavior with the tuple being populated in distinct parts of the soapcpp2 input source. This is similar to the way the `SOAP_ENV__Header` struct was already handled. This is not a standard C/C++ feature.
- Added `$CONTAINER = name` and `$POINTER = name` definitions to `typemap.dat` for specifying the C++ container to use (`std::vector` by default) and C++11 smart pointer to use (no smart pointer by default), see [databindings](https://www.genivia.com/doc/databinding/html).
- Improved soapcpp2 input source code analysis and C/C++ syntax coverage.
- Improved soapcpp2 output WSDL and XSD with added root-level elements and attributes on demand, see [databindings](https://www.genivia.com/doc/databinding/html).
- Improved efficiency and case coverage of SOAP encoded messages with id-ref anchors using redesigned pre- and post node graph construction analysis algorithms (`soap_id_enter`, `soap_id_forward`, `soap_id_lookup`, and `soap_update_pointers` have changed).
- Improved cyclic data structure detection and XML rendering with non-SOAP-encoded literal style (and/or the `SOAP_XML_TREE` flag) to prune cycles from trees. `SOAP_XML_TREE` produces XML document "trees" rather than id-ref XML graphs. Graphs are produced with SOAP encoding or with the `SOAP_XML_GRAPH` flag.
- Improved primitive `union` members access in C++ by removing unnecessary pointers.
- Improved checking of signatures in WSSE plugin.
- Updated soapcpp2 `size_t` mapping to transient (non-serializable) type. The wsdl2h tool does not generate `size_t`. If `size_t` is used in your header files for soapcpp2, then use `unsigned long` to replace `size_t`, which replicates the old `size_t` mapping to `unsigned long` (beware, this is not portable).
- Updated *`xsi:nil`* serialization rules for pointer-type struct/class members, by forcing *`xsi:nil="true"`* element output for required **nillable** elements (no longer requiring the `SOAP_XML_NIL` flag).
- Updated wsdl2h Nillable optional element annotations (i.e.g for `nillable=true` and `minOccurs>0`), now requiring inbound elements with *`xsi:nil`* or validation error. NULL pointer-based outbound nillable elements with `minOccurs>0` are now serialized with *`xsi:nil="true"`*, see above.
- Updated dateTime content validation with `SOAP_XML_STRICT`, otherwise accepts ISO8601 variants.
- Fixed `SOAP_UNION_name_type_member` macro constants for C++ namespace `name` (wsdl2h option `-qname` to ensure that the generated macro names are unique.
- Fixed attribute serialization when attribute of type `std::string` has a default value while actual value is set to "" (empty).
- Fixed class inheritance from base class `soap_dom_element` (declared in `dom.h`) which should include the content of the DOM node(s) together with the (de)serialized content of the derived class instance.
- Fixed default/fixed values of element/attribute references to global elements/attributes with global default/fixed values.
- Fixed *`xs:group`* and *`xs:substitutionGroup`* element namespace prefix qualifications of struct and classe members.
- Fixed deserialization of nested *`xs:sequence`* class pointers to NULL (structs are OK) when *`xs:sequence`* is optional and sequence data is not present on the inbound stream.
- Fixed missing `custom/struct_timeval.c` inbound microsecond fraction of *`xsd:dateTime`* values.
- Fixed `typedef` of a custom serializer type.
- Fixed `typedef` pointer type use in `@type` attribute declarations.
- Fixed linking against newer EVP functions in older OpenSSL versions to compile WSSE plugin.
- Fixed WSSE decryption with HTTP-chunked and fragmented messages in transport.

Version 2.8.25 (11/11/2015)
---
- Added `xsd__decimal` custom serializer `custom/float128.h` to bind `<quadmath.h>` quad precision float `__float128` to *`xsd:decimal`*.
- Added group circular reference detection to wsdl2h. Generates code for circular group references that is safe to use, but with loss of occurrence validation constraints, if any are present.
- Updated and improved XML-RPC and JSON implementation with wider `_int`, now 64 bit, while keeping XML-RPC type `_i4` to just 32 bit. Existing project C code that uses the XML-RPC and JSON plugin SHOULD be updated to use `_i4` (`int`) instead of `_int` when `_int` is still used as a 32 bit `int` in their project.
- Fixed strict validation of empty elements unless the element is href/ref-attributed with SOAP encoding (this validation check was temporarily lost in the 2.8.24r release).
- Fixed missing `#include <memory>` for smart pointers.
- Fixed float type minInclusive and maxInclusive fractional bounds validation.
- Fixed compilation issues when C++ namespaces (soapcpp2 option `-q`) are used with custom serializers.
- Fixed wsdl2h option `-p`, which may cause type name clashes with generated C++ `xsd__anyType` wrapper classes for simple types.

Version 2.8.26 (11/30/2015)
---
- Added new **jsoncpp** code generator that takes a JSON sample document and renders it in JSON API code for C or C++. The jsoncpp tool is located in `gsoap/samples/xml-rpc-json` with the XML-RPC and JSON APIs and samples.
- Added new C/C++ XML-RPC and JSON APIs `xml-rpc.c` and `json.c[pp]` to simplify JSON and XML-RPC in C and C++. The old `json_c.h` and `json_c.c` are deprecated and removed in favor of the improved APIs `json.h` and `json.c[pp]`.
- Added `xsd__integer` custom serializer `custom/int128.h` to bind big int `__int128_t` to *`xsd:integer`*.
- Added floating point exclusive range bounds notation to gSOAP header file syntax, with `<` on the side of `:` for an exclusive bound (`min <: max` makes `min` exclusive, `min <:< max` makes both exclusive, and `min :< max` makes `max` exclusive), while keeping `min : max` for inclusive bounds.
- Added C++11 syntax for struct/class member initialization with `{ initval }`, which has an identical effect as declaring members with `= initval` initializers. This works in C and C++.
- Added HTTP2 support to WinInet plugin.
- Expanded [XML-RPC and JSON documentation](https://www.genivia.com/doc/xml-rpc-json/html).
- Expanded [Data Bindings documentation](https://www.genivia.com/doc/databinding/html).
- Improved wsdl2h schema import processing when WSDL and XSD have the same namespace and *`xs:import`* is used instead of *`xs:include`* to populate the schema.
- Improved wsdl2h output for integer enumeration constants, omits trailing underscores for pseudo-numeric enum constants, updated soapcpp2 to accept duplicate enum constants in different enum lists when their values are the same.
- Fixed `soap_del_string` memory issue in `char*` string deletion.
- Fixed soapcpp2 option `-s` by enforcing `SOAP_XML_STRICT` on the body of inbound messages for client and server.
- Fixed a rare issue where elements end up missing in a substitutionGroup when several WSDLs with the same namespace are imported into one WSDL.
- Fixed strict validation rejection of empty element body when empty when permitted.

Version 2.8.27 (12/7/2015)
---
- Added new JSONPath code generation feature to jsoncpp in `gsoap/samples/xml-rpc-json` with new option `-p`. Supports C++ (and C in the next release).
- Improved JSON API functionality and robustness.
- Improved substitutionGroup union member: removed prefix `__` from member name and removed rogue `;`.
- Fixed `WITH_NOIDREF` broken `iht` initialization introduced in 2.8.25/26, which may lead to a crash.

Version 2.8.28 (02/01/2016)
---
- Added new **domcpp** code generator that takes an XML sample document and renders it in XML DOM API code for C or C++. Can also convert XPath to XML DOM API code with option `-p`. The domcpp tool is located in `gsoap/samples/dom` with the XML DOM samples.
- Added many new API functions to a fully redesigned XML DOM API v5 for C and C++ with new DOM API documentation.  The redesigned DOM API v5 is mostly backward compatible to DOM API v4, but with DOM string `data` members changed to `text` members.  Many API functions are added to define a clean interface without having to use the DOM structure data members.  The `soap_dom_next_element()` function now takes a second parameter that is an element pointer to stop deep traversal at (use NULL for old behavior).  See updated [XML DOM and XPath](https://www.genivia.com/doc/dom/html) documentation for details.
- Added C serialization to/from strings with new C `soap` struct members `const char *soap::is` input string to parse from and `const char **soap:os` pointer to string that is set by the engine to point to the output.  Note that C++ code is unchanged and the `soap::is` and `soap::os` are still members pointers to `istream` and `ostream` streams (use `std::stringstream` for input/output from/to strings).
- Added JSON API C function `set_size` to set/change array sizes and JSON API C function `set_struct` to create an empty JSON object.
- Improved jsoncpp command line tool to generate JSONPath C code and improved overall to generate more compact code.
- Improved `SOAP_XML_DEFAULTNS` flag to emit XML default namespace declarations.
- Improved strictness of XML verification under default settings (i.e. without requiring `SOAP_XML_STRICT`).
- Changed string deserialization when XML content with tags is encountered: XML tags are no longer parsed as part of strings.  You must use `XML` string type to parse literal XML content or use DOM.
- Fixed C++ JSON/XML-RPC API `time_t` type usage to `ULONG64` to ensure portability.  This affects the `value(time_t)` constructor, the `value::operator=(time_t)` assignment operator, and `value::operator time_t()` cast, which have been replaced by versions using `ULONG64`.  To use these in C++ for `time_t` ISO 8601 date time serialization, simply cast `time_t` values to `ULONG64` values.
- Fixed gsoapWinInet.cpp:243, `pDst` should be `a_pDst`.
- Fixed SNI `SSL_set_tlsext_host_name` call for OpenSSL 0.9.8f.
- Fixed `soap_instantiate` of smart pointer when instantiating derived class by *`xsi:type`*.
- Fixed friend `__declspec(dllexport)` declarations in generated classes.

Version 2.8.29 (02/24/2016)
---
- Added new soapcpp2 option `-r` to generate a soapReadme.md report that summarizes the input .h file information, the serializable C/C++ types, the services, and the generated code. Use a markdown converter to browse or download the free readmeviewer.html from https://www.genivia.com/files/readmeviewer.html.zip to view the soapReadme.md report in a browser.
- Added new wsdl2h option `-M` to suppress error "must understand element with wsdl:required='true'".
- Upgraded HTTP digest authentication `httpda` plugin to 2.0 to support RFC7616 "HTTP Digest Access Authentication" with SHA2 (replacing MD5) that is compatible with RFC2617; compile and link upgraded `plugin/httpda.c` with `plugin/smdevp.c` instead of `plugin/md5evp.c`.
- Updated wsdl2h HTTPS-enabled build steps to use upgraded `httpda` plugin.
- Updated `samples/webserver` to use upgraded `httpda` plugin.
- Updated the samples with new AWS S3 SOAP API example `samples/aws`.
- Improved soapcpp2 code generation of inline object read/write functions in C++, instead of macros generated in C.
- Fixed and improved soapcpp2 code generation with options `-p` and `-q`.
- Fixed definitions/service/port/wsaw:UsingAddressing processing.
- Fixed SNI `SSL_set_tlsext_host_name` call for OpenSSL 0.9.8f.
- Fixed wsdl2h skipping *`mime:multipartRelated/mime:part`*.

Version 2.8.30 (04/02/2016)
---

- Added backtick XML tag name syntax to interface files for soapcpp2, which allows for the overriding of the translated tag names of struct/class members and service operation parameters, see the [Data Bindings documentation](https://www.genivia.com/doc/databinding/html#toxsd9-5). Older gSOAP versions do not support the backtick tag in the generated WSDL and schemas (messages are OK).
- Added macro `SOAP_MAXLEVEL` to trigger `SOAP_LEVEL` error when XML nesting level of inbound XML exceeds the value of `SOAP_MAXLEVEL`.  Default value is 10000.  Redefine `SOAP_MAXLEVEL` as needed, with lower values to restrict XML nesting depth for receivers to accept.
- Added macro `SOAP_MAXLENGTH` to trigger `SOAP_LENGTH` content length error when string content in inbound XML exceeds the value of `SOAP_MAXLENGTH`.  Applies to strings that are potentially unbounded, i.e. that are not already constrained by XML validation maxLength constaints (which could be larger than `SOAP_MAXLENGTH`).  Default value is zero (0) which means that string length is unconstrained if XML validation maxLength is not given.  Redefine `SOAP_MAXLENGTH` as needed, with lower values to restrict string lengths for receivers to accept.
- Added macro `SOAP_MAXOCCURS` to trigger `SOAP_OCCURS` content error when array and container lengths exceed the value of `SOAP_MAXOCCURS`.  Must be greater than zero (0).  Default value is 100000.  Redefine `SOAP_MAXOCCURS` as needed, with lower values to restrict array and container lengths for receivers to accept.
- Updated wsdl2h WS-Policy processing to include WS-RM protocol versioning.
- Updated `soap_ignore_element` processing efficiency.
- Updated `import/saml1.h` and `import/saml2.h`.
- Improved soapcpp2 generation of WSDL and XSD for unqualified C/C++ types or when mixing qualified and unqualified C/C++ types and type names.
- Changed soapcpp2 default behavior for generating WSDL and XSD files with *`elementFormDefault="unqualified"`* from the old default behavior with *`elementFormDefault="qualified"`*, i.e. when `//gsoap ns elementForm:` directive is NOT explicitly included in the interface .h file for soapcpp2 and when soapcpp2 option -e is not used.  *This change DOES NOT affect interface files produced by wsdl2h*, only those interface .h files that are not auto-generated and do not have elementForm directives.  *If you use soapcpp2 to generate WSDL and schemas AND you need backward compatible SOAP document/literal style messaging AND you DO NOT have `//gsoap ns service style: rpc` (or soapcpp2 option -e) AND you DO NOT have `//gsoap ns schema elementForm:` directives already in your .h file, then you SHOULD add `//gsoap ns schema elementForm: qualified` directives to your .h file for each `ns` prefix*.  This recommendation ensures backward compatibility in your current SOAP document/literal style messaging projects.  Changing the soapcpp2 default behavior to *`elementFormDefault="unqualified"`* simplifies data bindings.  It is also the accurate form to use when C/C++ types are not namespace qualified in the .h interface file for soapcpp2.
- Improved DLL export with compiler flag `/DSOAP_STD_EXPORTS` (which defines `SOAP_STD_EXPORTS`) to dllexport the API, classes, and the proxy and service classes.  Modified the plugins to support dllexport.
- Fixed soapcpp2 crash with enum constants > 255 due to libc `isalpha` crashing on some Linux systems.
- Fixed `import/wsp.h` soapcpp2 compilation error.
- Fixed soapcpp2 option `-w` that may cause a message response element tag name inconsistency with SOAP doc/lit style.
- Fixed deserialization issue with dynamic arrays of STL containers/smart-pointers (i.e. a pointer to an array of containers/smart-pointers, which is an unlikely combination to use, but should work).

Version 2.8.31 (05/1/2016)
---

- Added engine context `sndbuf` and `rcvbuf` attributes to (re)set `setsockopt` values for `SO_SNDBUF` and `SO_RCVBUF`, respectively.  Default value is `SOAP_BUFLEN`, same as engine's internal message buffer size.  Setting to zero forces the engine to omit the `setsockopt` `SO_SNDBUF` and `SOAP_RCVBUF` calls, which for example can be used to enable TCP buffer autotuning with Linux (Linux 2.4 and up).
- Added userinfo syntax for HTTP Basic and NTLM authentication, i.e. this automatically sets `soap::userid` and `soap::passwd` from the userinfo and also sets `soap::authrealm` to the domain if not already set. HTTP Basic authentication is not recommended without secure https. For NTLM authentication, set `soap::ntlm_challenge = ""` to proceed immediately with NTLM instead of Basic authentication.
- Improved strengthening of `SOAP_XML_STRICT` and the soapcpp2 `-s` flag to reject all extra (non-deserializable) XML and character data by the parser and deserializers.
- Improved client-side certificate checking (DNS or IP, and wildcards).
- Improved soapcpp2 option `-t` and `//gsoap ns schema typed: y` directive that force the addition of *`xsi:type`* attributes to XML content except for types whose type names are prefixed with an underscore (i.e. root elements w/o type as per wsdl2h data bindings rules).
- Fixed crash with nested dynamic arrays in C++ (C is fine) i.e. classes and structs with `__ptr` and `__size` members as arrays of elements, where these arrays contain nested dynamic arrays.  [See here for patch](https://www.genivia.com/advisory.html).
- Fixed typedef of `xsd__hexBinary` struct/class, which should map to hexBinary but instead mapped to base64Binary.
- Fixed `nc` and `cnonce`, which are removed when `qop` directive is absent in HTTP digest authentication as per RFC2617.
- Fixed the digest authentication plugin from blocking basic authentication.
- Fixed soapcpp2 naming of unnamed function/method parameters as `_param_N` with N now counting up sequentially *per function* instead of globally counting up.  Use soapcpp2 option `-z3` for backward compatible global parameter indexing.
- Fixed (smart) pointer deserialization of non-base (derived) objects that are XML serialized with SOAP id-ref encoding using a href/ref that references a derived object (i.e. should result in a (smart) pointer to a derived instance, not a base instance or missing instance).
- Fixed strict validation (`SOAP_XML_STRICT`) of unqualified attributes in elements with a default namespace.
- Fixed wsdl2h 2.8.28-30 parsing of *`xs:unique`* causing wsdl2h to skip over schema components.  The fix also improved string-based parsing of XML content with a fix for 2.8.28-30 XML string handling.

Version 2.8.32 (05/10/2016)
---

- Improved soapcpp2 code generation of type converters `int soap_s2T(soap*, const char*, T*)` and `const char *soap_T2s(soap*, T)` for primitive and binary types T.
- Fixed unqualified *`xsi:type`* content matching with default namespace, which may lead to a failure in the 2.8.31 release to instantiate derived instances for complexType extensions.
- Fixed Solaris-specific memory issues in `soap_wstrdup`.

Version 2.8.33 (06/14/2016)
---

- Updated WS-Trust gsoap/import/wstx.h `__wst__RequestSecurityToken` response message parameter to `wst__RequestSecurityTokenResponseCollection`
- Updated WS-Security gsoap/import/wsse.h with optional SAML assertions in the Security header.
- Updated response processing for empty HTTP body with HTTP code 200 to 202: no longer forces socket close when HTTP keep-alive is enabled.
- Updated `_XML` literal string XML serialization for qualified tag names, no longer uses default namespace (as in *`xmlns="URI"`*) but a prefixed tag name only.
- Updated wsdl2h options `-p` and `-d`, now generates `xsd__anyType*` (i.e. with pointer) data members without requiring the user to define a typemap.dat rule to do so. This change reinstates some of the old behavior of 2.8.23 and earlier versions.
- Fixed wsddapi.c compilation issue on Windows (`usleep` replaced).
- Fixed validation of *`simpleType/restriction/length`* when restriction base is a list that is mapped to a string.
- Fixed validation of *`simpleType/restriction`* with base type string length bounds restrictions.

Version 2.8.34 (08/17/2016)
---

- Added custom serializers for QT primitive types and QT containers.  This serializes many QT types directly in XML.  The QT types to use for XSD types are specified in the typemap.dat.  See the updated gsoap/typemap.dat file in the distribution package and the updated [databindings](https://www.genivia.com/doc/databinding/html) documentation for details.
- Added HTTP server session management plugin to manage server-side session cookies.
- Added basic common WADL support to wsdl2h to generate code for WADL REST XML applications.
- Improved client-side URL query generation and support for URL templates for REST operations.
- Updated the iOS plugin and its Web service examples.
- Updated SSL calls for improved iOS portability.
- Updated wsdl2h headerfault support (optional fault details in SOAP Headers, not widely used by Web services tools).
- Updated VC++ ISAPI plugin.
- Fixed `soap_send_empty_response()` with HTTP keep-alive to prevent exit of the server loop back to `soap_accept()`, i.e. prematurely killing the keep-alive connection causing EOF/RST.
- Fixed `#import "wsse.h"` in `gsoap/import/ds.h` by moving it down to avoid missing type declaration of `_ds__KeyInfo` which may cause a soapcpp2 syntax error.
- Fixed WS-Security interoperability issues, fixes issues with XML encryption. Token handler callback has new parameters to pass the key data of SecurityTokenReference/KeyIdentifier.
- Fixed the internal `feltbegout` and `feltendout` callbacks: when set no longer emits XML and (alternative) output is expected to be emitted by these callbacks.

Version 2.8.35 (09/19/2016)
---

- Added auto-generation of new C functions `T * soap_new_T(struct soap*, int n)` to allocate and default initialize one (or more with `n>1`) value(s) of type `T`. Uses `soap_malloc(soap, n * sizeof(T))` and applies `soap_default_T(struct soap*, T*)` to each value allocated.
- Added WS-Trust wst extensible framework with SAML 1.0/2.0 tokens, PSHA1 algorithm, and an example WS-Trust client and server to request, create, sign, and verify SAML 2.0 tokens.
- Changed `soap_new_block` to `soap_alloc_block` in stdsoap2.h and in the gsoap libs to prevent potential name clashes with generated code for a `block` type.
- Improved UDP connectivity with WS-Discovery to reuse current socket connection, i.e. preventing premature socket close when `soap::socket` is the `soap::master` socket.
- Fixed wsdl2h option `-u` to prevent a deserialization issue that occurs in the specific case of an *`<xs:any>`* within a *`<xs:choice maxOccurs="unbounded">`*.

Version 2.8.36 (09/21/2016)
---

- Fixed a problem with `SOAP_ENV__Header` missing in wsdl2h-generated .h file (problem occurs with 2.8.34/35 due to an update for headerfaults).

Version 2.8.37 (10/25/2016)
---

- Added server-side HTTP cross-origin resource sharing (CORS) access control using HTTP OPTIONS `fopt()` callback.
- Fixed `make -j n` parallel builds.
- Fixed an issue causing UDP message fragmentation.
- Fixed `soap_mq` message queue plugin dropping connections.
- Fixed missing nested [] in arrayType value in SOAP 1.1 encoding (SOAP 1.2 not affected).

Version 2.8.38 (11/11/2016)
---

- Added auto-generated client-side REST API functions to simplify REST GET, PUT, POST operations with XML data: `soap_GET_T`, `soap_PUT_T`, `soap_POST_send_T`, and `soap_POST_recv_T` for XML elements/types `T`.  Also `soap_DELETE` REST DELETE added.  See [get started](http://www.genivia.com/dev.html#services) on how to use these functions.
- Updated samples/webserver to use both the httpform and httppost plugins to serve HTML form data and REST PUT, POST, DELETE operations for the updated samples/rest example person.c and person.cpp clients.

Version 2.8.39 (11/17/2016)
---

- Minor improvements: the engine sets the temporary "C" locale for floating point conversion on most systems.  This is now the default on most systems, rather than an option.  To disable, compile source code with `-DWITH_NO_C_LOCALE`.  To enable, compile with `-DWITH_C_LOCALE`.  As before, the setting is temporary and thread-local in the engine so it does not affect the application's locale.

Version 2.8.40 (12/10/2016)
---

- Improved wsdl2h import/include relative path search.
- Fixed wsdl2h schema import when imports are deeply nested in imports/includes.
- Fixed MinGW compilation issue.

Version 2.8.41 (01/11/2017)
---

- Added updates to support OpenSSL 1.1.0.
- Added HTTP header `Accept: multipart/related,application/xop+xml,*/*;q=0.8` when MTOM is expected (i.e. when the input mode flag is set to `SOAP_ENC_MTOM`).
- Improved CORS internals and compatibility.
- Fixed minor issues with `WITH_NOIO` and `WITH_NO_C_LOCALE`.
- Fixed bug in XML attribute serialization of QT QByteArray and QString types.
- Fixed WinCE7 `IP_MULTICAST_IF` issue.

Version 2.8.42 (01/20/2017)
---

- Improved `WITH_REPLACE_ILLEGAL_UTF8` flag to compile the source code of the gSOAP libraries:  this replaces illegal UTF-8 input/output with the replacement character U+FFFD (or define your own `SOAP_UNKNOWN_UNICODE_CHAR`).
- Fixed shared pointer to QName string QName output normalization.
- Fixed wsdl2h pointer member to vector for minOccurs="0" by removing the unnecessary pointer to the container, i.e. just using the container.

Version 2.8.43 (02/05/2017)
---

- Added `SSL_CTX_need_tmp_RSA()` check (OpenSSL 1.0.1 and greater).
- Fixed string length limiting issue in QT QString type serializer `custom/qstring.h` and addressed compilation issue with other QT types serializers.
- Fixed documentation of `soap_copy_stream()` followed by `soap_free_stream()` to chain services.
- Fixed `soap_psha1` string buffering.

Version 2.8.44 upd (03/04/2017)
---

- Improved windows portability and stability.
- Fixed WS-Discovery `soap_wsdd_listen` memory cleanup on timeouts.
- Fixed `soap::os` saved message string NUL termination (a problem in C code, not in C++).
- Fixed Cygwin and MinGW missing xlocale.h error.

Version 2.8.45 upd (04/07/2017)
---

- Fixed an issue with the WSSE plugin that caused WS-Security SignedInfo/Reference/Transforms/Transform/InclusiveNamespaces/@PrefixList ending up being ignored by the canonicalizer.
- Fixed Windows and WinCE compilation issues.
- Improvements.

Version 2.8.46 (05/16/2017)
---

- Improved WS-Trust API, updated import/wst.h, import/saml1.h, import/saml2.h and import/wsdd.h definitions.
- Added `soap::client_interface` string to set the client IP address interface with `inet_pton()` (not generally available on windows).
- Fixes for minor issues, improvements.

Version 2.8.47 (06/07/2017)
---

- Added CURL plugin to use libcurl for gSOAP client applications.
- Fixed spurious occurrences of `<root/>` elements in XML renderings of DOM nodes.
- Improvements.

Version 2.8.48 upd (06/21/2017)
---

- Improved element and attribute `default` and `fixed` value validation.  Changed the code generation by wsdl2h slightly for optional elements with default values.  This fixes an issue when an optional element is omitted in XML and becomes indistinguishable from an empty element because in both cases a default value is assigned.  An omitted optional element has no default value.  New XML validation error codes `SOAP_FIXED` and `SOAP_EMPTY`.
- Added `soap::transfer_timeout` max transfer timeout, to use in combination with `soap::send_timeout` and `soap::recv_timeout`.
- Fixed a potential vulnerability that may be exposed with a large and specific XML message over 2 GB in size.  After receiving this 2 GB message, a buffer overflow can cause an open unsecured application to crash or malfunction.  Clients communicating with HTTPS with trusted servers are not affected.

Version 2.8.49 upd (07/28/2017,07/28/2017)
---

- Improved JSON API to compile with XML data bindings, see updated JSON API documentation on "Compiling XML-RPC/JSON together with gSOAP XML data binding code"
- Improved white space handling of built-in XSD types that have "replace" and "collapse" white space properties. Further, types derived from these built-in XSD types will now inherit the white space "replace" or "collapse" property, meaning that white space of inbound strings are normalized (`xsd__anyURI`, `xsd__language` `xsd__ENTITY`, `xsd__ENTITIES`, `xsd__ID`, `xsd__IDREF`, `xsd__IDREFS`, `xsd__Name`, `xsd__NCName`, `xsd__NMTOKEN`, `xsd__NMTOKENS`, `xsd__normalizedString`, `xsd__token`, etc).
- Fixed a memory leak in the deserializer of `std::vector<xsd__anyType>` (and dynamic arrays of `xsd__anyType`) where `xsd__anyType` is a DOM node imported with `#import "dom.h"`.
- Fixed WSSE plugin recanonicalization of inclusive C14N SignedInfo.
- Fixes for minor issues, improvements.

Version 2.8.50 upd (07/23/2017,07/28/2017)
---

- Added samples/atom Atom 1.0 XML REST example.
- Added `soap::recv_maxlength` to change the limit on the length of messages received.  Default is 2GB max.  Greater lengths are possible, but at your own risk.  It is recommended for services deployed in uncontrolled environments to use the [Apache module](https://www.genivia.com/doc/apache/html/index.html) and [ISAPI extension](https://www.genivia.com/doc/isapi/html/index.html), see the gSOAP [tutorial](https://www.genivia.com/tutorials.html) and [documentation](https://www.genivia.com/docs.html).
- Removed client-side `SOAP_PURE_VIRTUAL` from `copy()` in the code generated by soapcpp2 for options `-i` and `-j`.
- Updated memory deallocation of `soap_del_xsd__anyAttribute`.
- Updated the callback function signatures of `fpost` and `fresponse`.
- Improvements.

Version 2.8.51 (07/28/2017)
---

- Additional stability and robustness improvements.
- Fixed WinInet HTTP GET blocking issue resulting in EOF.

Version 2.8.52 (08/18/2017)
---

- Added testmsgr "Test Messenger" with documentation.  The Test Messenger application is used for functional and non-functional testing of Web services and clients.  The Test Messenger is located in samples/testmsgr.
- Added `SOAP_MAXALLOCSIZE` macro to limit the size argument of `malloc()` calls, where zero means unlimited (default).  Exceeding the limit results in `SOAP_EOM` (out of memory) errors.  Some systems raise signals or throw exceptions when the the size argument of `malloc()` exceeds a certain size.  To avoid signals and exceptions set the `SOAP_MAXALLOCSIZE` as needed.
- Fixed 2.8.51 issue in parsing HTTP empty lines, fixed with: `if (i == len) /* empty line: end of HTTP/MIME header */`
- Fixed WS-RM plugin blocking issue on fatal errors.
- Improved use of STL containers: `#import "stl.h"` is no longer required.
- Improvements.

Version 2.8.53 (08/29/2017)
---

- Improved testmsgr "Test Messenger" to handle element repetitions, selections, and optional values for complete XML message randomization to test services and clients.  Updated soapcpp2 option `-g` to emit XML message templates with the new template indicators.
- Updated plugin/threads.h to let `THREAD_CREATE` return 0 (OK) on Windows like pthreads, thereby making the `THREAD_CREATE` return value portable.
- Fixed DIME receiver looping on specific malformed DIME headers.

Version 2.8.54 (09/17/2017)
---

- Added Google Map Directions and Distance Matrix API XML examples, updating and replacing the old `gsoap/samples/googleapi` example.
- Added Google Map Distance Matrix API JSON example gsoap/xml-rpc-json/json-GoogleDistanceMatrix.cpp.
- Added wsdl2h option `-S` to specify a name for the soap context member variable of generated classes (C++ only), use `-S ''` to remove the `soap` member variable.
- Improved handling of empty SOAP Body responses by client applications using doc/lit style messaging, returns `SOAP_OK` instead of HTTP 200 error code.
- Improved Apache module with new `IMPLEMENT_GSOAP_SERVER_INIT(user_func)` to specify a user-defined initialization function to initialize the soap context and register plugins, and service operation faults are now returned with proper HTTP status codes.
- Improved Borland C++ builds.
- Improved `soap_GET_Type` to close socket if connection does not use HTTP keep-alive.
- Improved DIME/MTOM/MIME attachment handling and detection.
- Improvements.

Version 2.8.55 (10/26/2017)
---

- Improved Test Messenger to generate and consume XML test messages with MTOM/MIME attachments, new options `-A`, `-C`, `-H` and `-M`.
- Updated WinInet plugin.
- Updated DOM node serialization of embedded serializable data: to serialize types defined in C++ namespaces, please see the updated DOM documentation about the new `-DSOAP_DOM_EXTERNAL_NAMESPACE=namespace_name` flag and how to "register" additional C++ namespaces.
- Fixed deserialization of pointers to Qt types with the custom serializers `custom/qbytearray_base64.h`, `custom/qbytearray_hex.h`, `custom/qdate.h`, `custom/qstring.h`, and `custom/qtime.h`.
- Fixed `WITH_NOIO` compilation errors (`close()` and/or `gettimeofday()` not found).

Version 2.8.56 (12/07/2017)
---

- Added new `soap::bind_v6only` context flag, replacing compile-time flag `WITH_IPV6_V6ONLY` (`soap::bind_v6only=1`) and `WITH_NO_IPV6_V6ONLY` (`soap::bind_v6only=0`).
- Added wsdl2h option `-D` to make attribute members with default values optional by adding pointers to member types, so assigning a NULL to an attribute member is identical to assigning the default value, this flag does not affect deserializing from XML (i.e. the default value is set when attribute is missing).
- Improved wsdl2h code generation for struct/class members with default/fixed values.
- Improved JSON and XML-RPC C and C++ APIs.
- Improved `SOAP_NEW` and `SOAP_DELETE` family of macros by passing the `soap` context to user-defined replacements, updated `soap_link()`.
- Improved `import/saml1.h` and `import/saml2.h` to fix WS/WS-typemap.dat `wsu__Id` optional attribute.
- Improved required attribute pointer member rendition in XML, producing default XML value if attribute pointer member is NULL.
- Improved HTTP/S cookie handling.
- Fixed non-portable `isalnum()` call in JSON parser.
- Fixed missing C++ custom serializer type object allocators (e.g. `gsoap/custom/qstring.cpp`), when pointers to custom serialized C++ types are used.
- Fixed memory leak in JSON C++ API `json.cpp` struct/array append `operator+`.

Version 2.8.57 (12/10/2017)
---

- Fixed an issue with the `WITH_IPV6_V6ONLY` compiler flag.

Version 2.8.58 (12/17/2017)
---

- Improvements to prevent macro `USE_32BIT_TIME_T` to cause misaligned `soap` contexts, due to inconsistent size of `time_t`.
- Improvements to support obsolete HTTP cookie formats.
- Fixed Windows tools wsdl2h.exe and soapcpp2.exe "The application was unable to start correctly (0xc000007b)" error.

Version 2.8.59 (12/30/2017)
---

- Removed `#include soapH.h` from generated `.nsmap` file to promote transparency and to prevent accidental inclusions that may cause definition clashes when soapcpp2 option -q is used.
- Fixed UDP message transport compression, compile flag `-DWITH_ZLIB` and runtime flag `SOAP_ENC_ZLIB`.

Version 2.8.60 (1/15/2018)
---

- Added wsdl2h options `-O1` and `-O2` to optimize schemas internally after reading WSDL and XSD files: `-O1` removes duplicate members from nested choice/sequence and `-O2` also removes unused schema types that are unreachable from WSDL and XSD root definitions by a new "schema slicing" algorithm.
- Added ability to specify `$SIZE = TYPE` in `typemap.dat` for setting `size_t` or `int` type for array sizes (default is `int` array size), see [databindings](https://www.genivia.com/doc/databinding/html).
- Changed wsdl2h C++ source code generated for *`xs:choice maxOccurs>0`* with simpler `std::vector` instead of a dynamic array with size and pointer members, use wsdl2h option `-z7` to revert to the old wsdl2h behavior for backward compatibility.
- Improved soapcpp2-generated sample XML messages and fixed a special case where base class namespace prefixes may be rendered incorrectly in an XML sample or test message.
- Improved handling of nested *`xs:sequence`* in *`xs:choice`* by removing duplicate name warnings, the duplicate member is still generated by wsdl2h however as a reminder, but should be ignored in your code.  Use the new wsdl2h option `-O1` to remove the duplicate member altogether.
- Fixed a soapcpp2 problem with bitmask enumerations of QName values.

Version 2.8.61 (1/27/2018)
---

- Added `WITH_DEFAULT_VIRTUAL` to generate default methods that return `SOAP_NO_METHOD` for C++ services generated with soapcpp2 option `-j` or `-i`.  A derived class can then selectively implement service methods as needed.  This addition required the use of a new macro `SOAP_PURE_VIRTUAL_COPY` for virtual `::copy()` methods, replacing `SOAP_PURE_VIRTUAL` for `::copy()`.
- Added `SOAP_H_FILE` macro to improve the use of plugins and custom serializers in project builds with soapcpp2 options `-p` and `-q` that rename the generated files such as `soapH.h`, the macro changes the `#include "soapH.h"` to `#include "nameH.h"` by invoking the C/C++ compiler with option `-DSOAP_H_FILE=nameH.h`.
- Fixed a glitch in soapcpp2 to generate `soap_write_T` functions for `typedef ... T` types that represent XML elements.
- Fixed double free in CURL plugin.

Version 2.8.62 (2/10/2018)
---

- Added wsdl2h options `-O3` and `-O4` to aggressively optimize WSDLs internally by "schema slicing": `-O3` applies `-O2` and also removes unused root attributes, `-O4` applies `-O3` and also removes unused root elements.  It only makes sense to use `-O4` with one or more WSDLs (and XSDs that are imported by the WSDL), because all schema components will be removed from XSDs that are not used by WSDLs.
- Added the inclusion of `xlocale.h` for GNU Linux to avoid compilation issues.
- Updated HTTP digest plugin.
- Improved soapcpp2 options `-g` and `-y`, may be used together to generate sample XML messages.
- Fixed Borland C++ compilation issue.

Version 2.8.63 (2/17/2018)
---

- Improved Test Messenger: easy randomized testing of Web Services; added `__PERMUTE` indicator and automatic handling of SOAP 1.1/1.2 array dimension adjustments (`arrayType` and `arraySize` attributes), among other improvements.
- Minor improvements.

Version 2.8.64 (3/5/2018)
---

- Upgraded TLS/SSL engine to support GNUTLS 3.3.0 and greater. To use the GNUTLS library in place of OpenSSL run `./configure --enable=gnutls` and compile all source code with `-DWITH_GNUTLS` instead of `-DWITH_OPENSSL`.
- Improved Test Messenger with new options `-d num` and `-u`, where `-dnum` specifies the number of iterations to hit a server with test messages, and `-u` includes Unicode characters in randomized content. Other usability improvements and a bug fix.
- Improved WSSE WS-Security plugin and added documentation section with clarifications on how the plugin defends against signature wrapping attacks when you perform signature verification function calls.
- Updated `xlocale.h` inclusion for GNU Linux, again. Some Red Hat Linux versions require `xlocale.h` in order to compile stdsoap2.c/stdsoap2.cpp and should therefore be compiled with the new compilation flag `-DWITH_INCLUDE_XLOCALE_H`, or by using `./configure --enable-xlocale`, to force the inclusion of `xlocale.h`.

Version 2.8.65 (3/8/2018)
---

- Corrected an issue in soapcpp2 to parse negative floating range bounds declared in typedefs.

Version 2.8.66 (4/9/2018)
---

- Added `soap_close_connection()` to close a connection from another thread.
- Fixed C++ proxy and server class `copy()` and `operator=()` methods to prevent a possible memory leak which may occur in certain usage scenarios.
- Fixed an issue in wsdl2h, generating an incorrect simpleType element name that leads to a soapcpp2 error.  The element has a local simpleType restriction of a simpleType with the same name as the element type, where this simpleType in turn is a restriction.

Version 2.8.67 (6/11/2018)
---

- Changed `typemap.dat` to disable `xsd__duration` custom serializer by default, meaning that `xsd__duration` is serialized as a string by default. To serialize `xsd__duration` as an integer with the `gsoap/custom/duration.c` custom serializer e.g. in ONVIF, please re-enable the `xsd__duration` custom serializer by removing the `#` comment from the `xsd__duration` specification in `typemap.dat`.
- Fixed an issue where the 64 bit integer types `LONG64` and `ULONG64` and their serializers would be downcast to 32 bit when compiling C code with newer GCC versions, due to `__STDC_VERSION__` no longer being defined by the compiler.
- Fixed Apache module URL query `?wsdl` handling.
- Fixed `gsoap/custom/qstring.cpp` deserializer, converts XML entities to/from chars.

Version 2.8.68 (6/29/2018)
---

- Minor improvements.

Version 2.8.69 (7/18/2018)
---

- Improved *`xs:redefine`* processing, fixing the remaining "circular group reference" warnings.
- Improved XML sample message generation.

Version 2.8.70 (8/27/2018)
---

- Renewed the PEM files for the SSL examples included with gSOAP.
- Updated `typemap.dat` for ONVIF and upgraded `wsdd10.h` (WS-Discovery 1.0 with WS-Addressing 2004/08) to `wsdd5.h` (WS-Discovery 1.0 with WS-Addressing 2005/08).
- Fixed a deserialization issue with Qt `QString` used in a wrapper class (as `__item` member), when the wrapper class is used in a container, such as `std::vector`.

Version 2.8.71 (11/12/2018)
---

- Improved user guide and added API documentation modules.
- Added TLSv3 support with OpenSSL 1.1.1.
- Added `./configure --enable-ipv6-v6only` option to build the libraries with `-DWITH_IPV6_V6ONLY`
- Added HTTP PATCH support.
- Added `soap::bearer` string for HTTP authorization with bearer tokens, assign the token value at the client side, the string contains the token value at the server side.
- Added `json_send_fault` to use in place of SOAP `soap_send_fault` calls, and `json_send_error` to use in place of SOAP `soap_sender_fault` calls.  Use `soap_send_fault` when an internal server-side error occurred and `soap_send_error` for the server to respond with an error message.
- Updated SSL/TLS options for `soap_ssl_client_context` and `soap_ssl_server_context` to allow combinations of `SOAP_TLSv1_0`, `SOAP_TLSv1_1`, `SOAP_TLSv1_2`, `SOAP_TLSv1_3` protocols (v1.3 only available with OpenSSL 1.1.1 and greater), a change from the single TLS protocol flag.
- Updated Apache mod gSOAP to set the client IP `soap::ip` and host `soap::host` values.
- Improved `soap_response` with `SOAP_FILE` parameter: it is now possible to return a HTTP status code with `SOAP_FILE + code` which returns the given http status code (200 to 599) with the http content type.
- Improved `json_call` performance.
- Improved `soap_close_connection` enabled with `-DWITH_SELF_PIPE`.
- Improved `soap_get_http_body` memory use.
- Changed `soap_get_http_body` to return "" (empty string) when no HTTP body is detected instead of NULL, to distinguish receiving an empty HTTP body (returning "") from errors (returning NULL with `soap::error` set).
- Changed Apache mod gSOAP `mod_gsoap.c` to use `RTLD_LOCAL` instead of `RTLD_GLOBAL` to permit multiple concurrent gSOAP modules to be loaded in Apache 2 with dlopen.
- Renamed the `soap_check_faultX` functions to `soap_fault_X` functions: `soap_check_faultstring` is replaced by `soap_fault_string`, `soap_check_faultdetail` is replaced by `soap_fault_detail`, `soap_check_faultsubcode` is replaced by `soap_fault_subcode`.
- Renamed the `SOCKET_CLOSE_ON_EXIT` macro to `WITH_SOCKET_CLOSE_ON_EXIT`.
- Renamed the `query` functions of the HTTP GET plugin gsoap/plugin/httpget.c to `soap_query`, `soap_query_key`, `soap_query_val`.
- Renamed the `form` function of the HTTP POST form plugin gsoap/plugin/httpform.cto `soap_get_form`.
- Fixed `-DWITH_INCLUDE_XLOCALE_H` and `configure` script: the problem caused build failures on Linux.  It is possible to force the use of `xlocale.h` with `./configure --enable-xlocale` but only use this when necessary, when `locale_t` is not declared.
- Fixed C14N-related WS-Security signature issue introduced in 2.8.28, which in most cases made no difference but could lead to a signature validation failure.
- Fixed soapcpp2 code generation issue for single- and multi-dimensional fixed-size arrays.
- Fixed wsdl2h missing built-in XSD types when multiple WSDLs are imported.

Version 2.8.72 (11/24/2018)
---

- Improved the HTTP GET `http_get` and HTTP POST `http_post` plugins, handling of a HTTP POST request that has an empty body is now supported.
- Updated user guide, corrected `soap_rand_uuid` description: string returned is stored in a temporary buffer, not stored in managed memory.
- Fixed spurious constant initialization problem for `enum` types in soapcpp2-generated code, the problem was introduced with soapcpp2 2.8.71 C/C++ grammar expansion.
- Fixed a CURL plugin issue that prevented PUT and DELETE methods to work properly.

Version 2.8.73 (12/3/2018)
---

- Implemented a work around an OpenSSL bug that may cause `SSL_get_error()` to crash in `soap_ssl_accept()`.  The crash depends on the configuration used and occurs in versions 2.8.71-72.  See [advisories](https://www.genivia.com/advisory.html) for details.
- Improved `soap_ssl_accept()` timeout settings to improve the performance of gSOAP stand-alone HTTPS servers.
- Renamed `soap_get_http_body()` to `soap_http_get_body()` to avoid name clashes with soapcpp2-generated `soap_get_T` functions.
- Renamed `soap_get_form()` to `soap_http_get_form()` to avoid name clashes with soapcpp2-generated `soap_get_T` functions.
- Renamed `soap_get_mime_attachment()` to `soap_recv_mime_attachment()` to avoid name clashes with soapcpp2-generated `soap_get_T` functions.
- Renamed `soap_get_stats()` to `soap_http_get_stats()` of the httpget plugin to avoid name clashes with soapcpp2-generated `soap_get_T` functions.
- Renamed `soap_get_logging_stats()` to `soap_logging_stats()` of the logging plugin to avoid name clashes with soapcpp2-generated `soap_get_T` functions.
- Moved `soap_http_get_form()`, `soap_query()`, `soap_query_key()`, and `soap_query_val()` functions from the httpget and httpform plugin APIs to the stdsoap2.c[pp] library API.  No project rebuilds should be necessary when using these plugins with this upgrade.
- Updated `gsoap/samples/webserver` example and documentation with improved job queueing when the thread pool option is used.

Version 2.8.74 (12/11/2018)
---

- Fixed an issue with MIME/MTOM attachment sending when the HTTP-digest plugin is used or when compression is enabled, returning incorrect `SOAP_EOM` error (i.e. not caused by out-of-memory).  The problem was introduced in 2.8.70.

Version 2.8.75 (1/14/2019)
---

- Added httppipe plugin for HTTP pipelining, as requested by users.
- Added asynchronous messaging example `gsoap/samples/async` to demonstrate asynchronous messaging.
- Added wsdl2h option `-F` to add transient pointer members to structs to simulate type derivation with structs in C.  This addition makes it easier to (de)serialize derived types in C that are indicated with *`xsi:type`* attributes in XML.  This approach greatly improves the use of derived types in C compared to the `void*` pointer to anything serialization trick.  This option can also be used in C++ to replace class inheritance by this approach, when desired.
- Added wsdl2h option `-L` to generate less documentation in interface header files by removing all @note comments that are generic.
- Added `WITH_NOEMPTYNAMESPACES` compile-time flag to disable *`xmlns=""`*, this is intended for backward compatibility with old XML parsers and old gSOAP versions that do not support *`xmlns=""`* empty default namespaces.  When used with the runtime `SOAP_XML_DEFAULTNS` mode flag, produces XML that lacks *`xmlns=""`* which should only be used for special cases and is not recommended in general.
- Improved interface header file format and content generated by wsdl2h.
- Improved wsdl2h *`import schemaLocation`* logic.
- Updated wsdl2h option `-D` to make attributes with fixed values pointer members in structs and classes, not just attributes with default values.
- Updated wsdl2h option `-f` to flatten C++ class hierarchy by removing inheritance.  This option removes support for type derivation with *`xsi:type`* in XML (indicating restricted and extended type values which cannot be used as a result).  This update also removes pointers from the item value types of arrays and containers, because there are no derived types that could extend the item value types.
- Updated soapcpp2 to split up the `soap_call_ns__webmethod` functions into new `soap_send_ns__webmethod` and `soap_recv_ns__webmethod` functions called by `soap_call_ns__webmethod`.  The new functions can be used for asynchronous messaging, while `soap_call_webmethod` is used for synchronous messaging.  This update also applies to C++ proxy classes with the addition of new `send_webmethod` and `recv_webmethod` methods for asynchronous messaging.
- Updated the httppost plugin with improved media type pattern matching of media patterns in the table of handlers, thereby removing the need to specify also the `media/type;*` patterns to match parameters.  Media type patterns without a `;` will match the media type of the message whether or not the media type of the message includes parameters.
- Updated `gsoap/samples/webserver` example and fixed the webserver HTTP keep-alive connection persistence configuration setting that was turned off (because both input and output mode flags should be set to `SOAP_IO_KEEPALIVE`).
- Updated soapcpp2 option `-j` to call `destroy()` in destructor to deallocate managed heap data (like option `-i`), if the `soap` context was allocated by the proxy and service object constructors.
- Updated wsdl2h to generate qualified operations for the document/literal wrapped pattern with incorrect use of part/@type in WSDLs, fixing an issue with wsdl2h option `-Nname` that may cause interop problems.
- Updated the CURL plugin to support the proxy settings specified by the user as proxy host `soap::proxy_host`, proxy port `soap::proxy_port`, and proxy credentials `soap::proxy_userid` and `soap::proxy_passwd`.
- Updated the WinInet plugin to support the proxy settings specified by the user as proxy host `soap::proxy_host`, proxy port `soap::proxy_port`, and proxy credentials `soap::proxy_userid` and `soap::proxy_passwd`.
- Updated call to OpenSSL `ERR_remove_state` (deprecated) by `ERR_remove_thread_state`.
- Fixed a bug in HTTP cookie handling by the engine.  HTTP cookies are disabled by default, but enabled with the `-DWITH_COOKIES` compile-time flag or when using the C/C++ `libgsoapck`/`libgsoapck++` and `libgsoapssl`/`libgsoapssl++` libraries.  Removed `-DWITH_COOKIES` from the `libgsoapssl`/`libgsoapssl++` build, disabling HTTP cookies by default for this library.  Instead, compile `stdsoap2.c`/`stdsoap2.cpp` and `dom.c`/`dom.cpp` with `-DWITH_IPV6` `-DWITH_OPENSSL` `-DWITH_GZIP` `-DWITH_DOM` `-DWITH_COOKIES` to obtain the same functionality as the old `libgsoapssl`/`libgsoapssl++` libraries.

Version 2.8.76 (1/21/2019)
---

- Improved soapcpp2 option `-a`.
- Updated to remove GCC 8.2 warnings.
- Updated wsdl2h WSDL and schema imports of files specified by relative paths: file name without path or file name with path stating with ../ are considered relative locations with respect to the current WSDL and schema that is importing, otherwise imported files are considered relative to the directory in which wsdl2h is run (the `-I` option can be used to change that location).
- Minor improvements.

Version 2.8.77 (1/23/2019)
---

- Updated wsdl2h to display warnings for invalid complexType and simpleType extensions/restrictions but generates valid code in such cases by inference, the update also fixes a valid extension case.
- Updated `gsoap/samples/async` examples.
- Fixed compilation error for soapcpp2 options `-i` and `-j` caused by special case with empty input arguments to service operations.

Version 2.8.78 (1/27/2019)
---

- Added jsoncpp new option `-k`.
- Updated soapcpp2 to remove C compiler warnings, a minor change to reorder the source code output.
- Updated `gsoap/samples/async`.

Version 2.8.79 (2/10/2019)
---

- Updated to remove OpenSSL deprecated API warnings.
- Fixed a bug in wsdl2h option `-c` for C source code output, resulting in a missing `*` pointer for `_XML __any` member declaration when declared after the `$ int __size` array size member.  The bug may lead to validation errors in extensible types when extra elements are present in the XML payload received.  The fix produces the correct `_XML *__any` member declaration.
- Minor improvements.

Version 2.8.80 (2/20/2019)
---

- Updated to remove VS2017 compiler warnings and to fix `soap_faultcode` (or similar) link errors.
- Updated to remove GCC 8.2 warnings.
- Updated examples to link against libpthread when required.
- Improved wsdl2h handling of relative file paths.

Version 2.8.81 (3/6/2019)
---

- Added the ability to specify `nullptr` web service operation arguments, similar to `nullptr` struct and class members.  This enables `xs:nillable="true"` elements corresponding to web service operation arguments.
- Updated wsdl2h *`import schemaLocation`* logic to handle relative paths.
- Updated to improve checking of `_GNU_SOURCE`, `_POSIX_C_SOURCE`, `_XOPEN_SOURCE` for GNU-specific non-XSI-compliant `gethostbyname_r` and `strerror_r` function.  If you run into compilation problems with these two functions, please contact Genivia technical support.
- Updated DOM API for embedded serializable data types: `SOAP_DOM_ASIS` removes XML namespace bindings (`xmlns`) from the XML output of the embedded data type, which are normally added to ensure namespace prefixes are always valid when serializable data is embedded in XML.  Using `SOAP_DOM_ASIS` requires the DOM to include `xmlns` namespace bindings explicitly.

Version 2.8.82 (3/14/2019)
---

- Minor fixes and improvements.

Version 2.8.83 (4/18/2019)
---

- Added wsdl2h optimization options `-Ow2`, `-Ow3`, and `-Ow4` to optimize the generated source code by schema slicing, while retaining all derived extensions of base types.  The new optimization options are generally recommended instead of the more aggressive `-O2`, `-O3`, and `-O4`, respectively, when derived type extensions of a base type are used in XML messages, which are referenced by *`xsi:type`* attributes in XML messages.  If base types are overridden by derived types indicated with *`xsi:type`* in XML, then the `-Ow` options should be used instead of the more aggressive `-O` options.
- Added wsdl2h option `-Q` to make `xsd__anySimpleType` equal to `xsd__anyType` to use as the base type for derived types, so that elements of type *`xsd:anySimpleType`* can be serialized with a derived type, using inheritance in C++ and by using simulated inheritance in C using wsdl2h option `-F`.  On the other hand this option invalidates XML attributes of type *`xsd:anySimpleType`*.  The soapcpp2 tool warns about this invalid attribute type as a result.
- Updated wsdl2h options `-p` and `-F` to generate additional wrappers for primitive types that aren't XSD primitive types, such as `SOAP-ENC:base64`.  This allows serialization of `xs:anyType` and `xs:anySimpleType` with additional derived types such as `SOAP-ENC:base64`.
- Improved wsdl2h output for the infrequently-used `SOAP-ENC:Array` type.  To regress to the old behavior, add this line `SOAP_ENC__Array = | struct { _XML *__ptr; int __size; } | struct { _XML *__ptr; int __size; }` to your copy of typemap.dat and rerun wsdl2h with the updated typemap.dat definitions.
- Fixed an issue with soapcpp2 option `-A` that resulted in error 13 `SOAP_NO_METHOD`.
- Minor improvements.

Version 2.8.84 (5/14/2019)
---

- Minor fixes and improvements.

Version 2.8.85 (6/24/2019)
---

- Added `soap::client_addr` string to specify a IPv4 or IPv6 or a host address to bind to before connecting.  This can be used at the client side to bind to an address before connecting to a server endpoint, similar to `soap::client_port`.
- Fixed wsdl2h compilation issue with C++17.
- Fixed `custom/duration.c` custom deserializer `SOAP_TYPE` error caused by parsing duration fractional seconds.

Version 2.8.86 (6/24/2019)
---

- Fixed a problem with the `SOAP_SSL_DEFAULT` settings parameter used with `soap_ssl_client_context` and `soap_ssl_server_context` when `SOAP_SSL_DEFAULT` is used without any `SOAP_TLSv1_X` or `SOAP_SSLv3` values.

Version 2.8.87 (7/1/2019)
---

- Added `soap::connect_retry` to specify a number of retries at the client side when connecting to a server fails, with exponential backoff of 2^n seconds between retries (1s, 2s, 4s, 8s, ... up to 32s per iteration).  Zero by default, meaning no retries.
- Added `soap::client_addr_ipv6` to optionally specify a IPv6 or host address to bind to at the client side, when the destination is a IPv6 server, otherwise uses `soap::client_addr` to bind.
- Improved portability for Cygwin, `gethostbyname_r` not available on Cygwin.

Version 2.8.88 (7/25/2019)
---

- Fixed an issue with wsdl2h `typemap.dat` for WS-Trust WSDLs causing missing types in the generated header file.
- Portability fixes and improvements.

Version 2.8.89 (8/5/2019)
---

- Added wsdl2h option `-X` to do not qualify part names in order to disambiguate document/literal wrapped patterns (as the other choice to disambiguate instead of the default qualification with schema namespaces).
- Added wsdl2h option `-z8` for backward compatibility with 2.8.74 and earlier: don't qualify part names to disambiguate doc/lit wrapped patterns and revert to the old wrapper class/struct naming used for `xs:anyType` inheritance.
- Improved wsdl2h option `-L` to skip non-essential comments from the output.
- Faster soapcpp2 and other improvements.

Version 2.8.90 (8/14/2019)
---

- Improved software bundling to prevent autoheader from running with `./configure`, other minor improvements.

Version 2.8.91 (8/15/2019)
---

- Correction to fix soapcpp2 2.8.90 `-z#` flag enforcement problem.

Version 2.8.92 (9/16/2019)
---

- Fixed soapcpp2-generated call to `soap_DELETE` for REST DELETE operations.
- Minor improvements.

Version 2.8.93 (9/24/2019)
---

- Fixed a wsdl2h schema import/include issue when a `./` occurs in `schemaLocation` and schema import/include dependencies are cyclic, causing wsdl2h to not be able to locate and read schema files.
- Removed empty substitutionGroup and duplicate substitutionGroup elements in wsdl2h-generated `SUBSTITUTIONS` sections.

Version 2.8.94 (10/17/2019)
---

- Fixed a wsdl2h issue that caused it to omit names for local simpleType restrictions in the generated `enum` types of struct/class members; improved soapcpp2 to avoid `enum` symbol numbering clashes in the generated source code.
- Removed unnecessary namespace prefixes from some class/struct members in the source code generated by wsdl2h in a specific case, to prevent XML validation issues.  This specific case for removing the prefix occurs for a reference to an element/attribute in the same schema, when the schema has an unqualified element/attribute default form.  That is, `ns1__member` is changed to `member` if `member` corresponds to a reference to an element in the schema to which the class/struct belongs and the `ns1` schema has `elementForm: unqualified`. This change has no effect on the generated code for typical WSDLs and schemas that use qualified default forms.  Added wsdl2h option `-z9` for backward compatibility of 2.8.94 and greater to versions 2.8.93 and lesser, which reverts this change.

Version 2.8.95 (11/14/2019)
---

- Upgraded `smdevp.c` to replace deprecated OpenSSL API function.
- Updated WS-Security WSSE plugin, documentation, and demo.
- Improved soapcpp2 execution speed to generate WSDL and XSD files.

Version 2.8.96 (12/4/2019)
---

- Improved `soap_check_mime_attachments()` and `soap_recv_mime_attachment()` functions and documentation, ensure proper close when MIME/MTOM errors occur.
- Minor improvements.

Version 2.8.97 (1/7/2020)
---

- Fixed wsdl2h processing of schemas with a cyclic schema `<xs:include>` that may cause wsdl2h to hang when schemas have no `targetNamespace` attribute.
- Improved wsdl2h code generation of unqualified types and names defined in imported schemas (with `<xs:import>`) when these schemas have no `targetNamespace`.  Use wsdl2h option `-z10` or lesser to revert to the code generation behavior of versions prior to 2.8.97.
- Other improvements.

Version 2.8.98 (2/16/2020)
---

- Updated the WS-Security and WS-Trust APIs that use SAML with higher precision timestamps in microseconds, using the `custom/struct_timeval.h` serializer for `xsd__dateTime`.  The WS-Security and WS-Trust updates require compiling and linking with `custom/struct_timeval.c`.
- Updated `ds__KeyInfoType` declared in `ds.h` to include a `xenc__EncryptedKey` member when `xenc.h` is imported, i.e. by making `ds__KeyInfoType` a mutable struct that is extended in `xenc.h`.
- Updated Test Messenger tool for unit testing, regression testing, and fuzz testing.  New options `-X` and `-Y`.
- Updated DOM API, whitespace at both edges of text elements in the DOM is no longer trimmed.
- Fixed an issue with soapcpp2 code generation of `wchar_t*` serializers when combined with a custom serializer with base type `wchar_t*`, i.e. when `extern typedef wchar_t* name` is declared.
- Fixed an issue with soapcpp2 code generation when an element tag names starts with an underscore and the element is namespace qualified.
- Other improvements.

Version 2.8.99 (3/12/2020)
---

- Improved performance of the soapcpp2 tool.
- Improved proxy connectivity on the client side with `soap::proxy_host`, `soap::proxy_port`, and NTLM, to maintain HTTP headers, e.g. `soap::http_content` and `soap::http_extra_header`.
- Fixed a bug in HTTP cookie handling when the optional `-DWITH_COOKIES` flag is used (the optional `libgsoapck`/`libgsoapck++` libraries are compiled with `-DWITH_COOKIES`).  Note that cookie support is disabled by default or has no effect when deploying robust services with the gSOAP Apache modules and ISAPI extensions that handle cookies differently.
- Other improvements.

Version 2.8.100 (3/24/2020)
---

- Improved proxy connectivity on the client side to handle bearer authentication.
- Improved soapcpp2 handling of the `#module` directive.
- Improved AIX 7.2 portability.
- Fixed an MTOM flag clearing issue hampering MTOM usability.

Version 2.8.101 (4/8/2020)
---

- Updated IPV6 support to fix a serious issue that affects versions 2.8.99 and 2.8.100 when the library is compiled with the optional `WITH_IPV6` flag.
- Other improvements.

Version 2.8.102 (5/4/2020)
---

- Improved HTTP digest authentication plugin to cover additional HTTP methods.

Version 2.8.103 (5/23/2020)
---

- Minor changes for enhanced platform portability.

Version 2.8.104 (6/30/2020)
---

- Minor improvements.

Version 2.8.105 (7/22/2020)
---

- Improved WSSE plugin to correct a digest verification issue when the signed XML parts use default `xmlns` bindings in elements that are not qualified.

Version 2.8.106 (8/22/2020)
---

- Minor update for struct/class `char*` and `wchar_t*` members declared with explicit default/fixed values: if the corresponding XML element value is absent in the XML payload then their deserialized value will be NULL (instead of the default value as in prior versions).  Note that empty XML element values in the XML payload always produce default values.  See [databinding: default and fixed values](https://www.genivia.com/doc/databinding/html/index.html#toxsd9-4).  New soapcpp2 option `-z4` to revert to the old behavior.

Version 2.8.107 (10/6/2020)
---

- Faster `soap_http_get_body()` to extract a HTTP message body to a string.
- Added `WITH_CRTOLF` compile-time flag to force serialization of C/C++ strings containing CRLF to LF and plain CR to LF in XML.
- Improved digital signature verification when XML is indented with white space containing CRLF between XML elements instead of plain LF.

Version 2.8.108 (10/16/2020)
---

- Minor update for compliance with C++17 and C++20 compilers and improved portability.

Version 2.8.109 (11/19/2020)
---

- Fixed wsdl2h output for a special case when schemas have no namespaces.
- Improved streaming MIME/MTOM attachment output.
- Removed C/C++ compiler warnings.
- Updated WS-Addressing and WS-Security plugins to harden code robustness.

Version 2.8.110 (01/17/2021)
---

- Updated wsdl2h to correct an XML parsing rejection problem for the special case when XML schemas have no targetNamespaces (i.e. empty namespace, no namespace prefixes) for elements defined in a `<sequence>` schema component, the wsdl2h-generated struct/class member names require two underscores instead of one, such as `__Name_sequence`.)

Version 2.8.111 (01/22/2021)
---

- Updated WS-Addressing plugin to harden code robustness.

Version 2.8.112 (03/19/2021)
---

- Added support to wsdl2h for the outdated "wsaw" Web Services Addressing 1.0 W3C candidate recommendation 2006, to complement the built-in wsdl2h support for "wsam" Web Services Addressing 1.0 W3C recommendation 2007.

Version 2.8.113 (04/15/2021)
---

- Updated self-signed SSL certificates, included with the example applications and demos.
- Additional WS-Trust request and response member definitions included, e.g. RequestedTokenCancelled and CancelTarget.
- Compression performance improvement.

Version 2.8.114 (04/20/2021) {#latest}
---

- Minor change to `_GNU_SOURCE` checking to resolve a source code portability issue.

[![To top](https://www.genivia.com/images/go-up.png) To top](changelog.html)

