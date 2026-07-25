
C and C++ XML Data Bindings                                          {#mainpage}
===========================

[TOC]

Introduction                                                            {#intro}
============

This article presents a detailed overview of the gSOAP XML data bindings for C
and C++.  The XML data bindings for C and C++ are extensively used with gSOAP
Web services to serialize C and C++ data in XML as part of the SOAP/XML Web
services payloads.  Also REST XML with gSOAP relies on XML serialization of C
and C++ data via XML data bindings.

The major advantage of XML data bindings is that your application data is
always **type safe** in C and C++ by binding XML schema types to C/C++ types.
So integers in XML are bound to C integers, strings in XML are bound to C or
C++ strings, complex types in XML are bound to C structs or C++ classes, and so
on.  The structured data you create and accept will fit the data model and is
**static type safe**.  In other words, by leveraging strong typing in C/C++,
your XML data meets **XML schema validation requirements** and satisfies **XML
interoperability requirements**.

In fact, gSOAP data bindings are more powerful than simply representing C/C++
data in XML.  The gSOAP tools implement true and tested **structure-preserving
serialization** of C/C++ data in XML, including the serialization of cyclic
graph structures with id-ref XML attributes.  The gSOAP tools also generate
routines for deep copying and deep deletion of C/C++ data structures to
simplify memory management.  In addition, C/C++ structures are deserialized
into managed memory, managed by the gSOAP `soap` context.

At the end of this article two examples are given to illustrate the application
of XML data bindings.  The first simple example <i>`address.cpp`</i> shows how to use
wsdl2h to bind an XML schema to C++.  The C++ application reads and writes an
XML file into and from a C++ "address book" data structure as a simple example.
The C++ data structure is an STL vector of address objects.  The second example
<i>`graph.cpp`</i> shows how C++ data can be accurately serialized as a tree, digraph,
and cyclic graph in XML.  The digraph and cyclic graph serialization rules
implement SOAP 1.1/1.2 multi-ref encoding with id-ref attributes to link
elements through IDREF XML references, creating a an XML graph with pointers to
XML nodes that preserves the structural integrity of the serialized C++ data.

These examples demonstrate XML data bindings only for relatively simple data
structures and types.  The gSOAP tools support more than just these type of
structures to serialize in XML.  There are practically no limits to the
serialization of C and C++ data types in XML.

Also the support for XML schema (XSD) components is unlimited.  The wsdl2h tool
maps schemas to C and C++ using built-in intuitive mapping rules, while
allowing the mappings to be customized using a <i>`typemap.dat`</i> file with mapping
instructions for wsdl2h.

The information in this article is applicable to gSOAP 2.8.26 and greater that
support C++11 features.  However, C++11 is not required.  The material and the
examples in this article use plain C and C++, until the point where we
introduce C++11 smart pointers and scoped enumerations.  While most of the
examples in this article are given in C++, the concepts also apply to C with
the exception of containers, smart pointers, classes and their methods.  None
of these exceptions limit the use of the gSOAP tools for C in any way.

The data binding concepts described in this article were first envisioned in
1999 by Prof. Robert van Engelen at the Florida State University.  An
implementation was created in 2000, named "stub/skeleton compiler".  The first
articles on its successor version "gSOAP" appeared in 2002.  The principle of
mapping XSD components to C/C++ types and vice versa is now widely adopted in
systems and programming languages, including Java web services and by C# WCF.

We continue to be committed to our goal to empower C/C++ developers with
powerful autocoding tools for XML.  Our commitment started in the very early
days of SOAP by actively participating in
[SOAP interoperability testing](http://www.whitemesa.com/interop.htm),
participating in the development and testing of the
[W3C XML Schema Patterns for Databinding Interoperability](http://www.w3.org/2002/ws/databinding),
and continues by contributing to the development of
[OASIS open standards](https://www.oasis-open.org) in partnership with leading
IT companies in the world.

🔝 [Back to table of contents](#)

Notational Conventions                                            {#conventions}
======================

The typographical conventions used by this document are:

* `Courier` denotes C and C++ source code.

* <i>`Courier`</i> denotes XML content, JSON content, file and path names, and URIs.

* <b>`Courier`</b> denotes HTTP content, text file content, and shell commands with command line options and arguments.

The keywords "MUST", "MUST NOT", "REQUIRED", "SHALL", "SHALL NOT", "SHOULD",
"SHOULD NOT", "RECOMMENDED", "MAY", and "OPTIONAL" in this document are to
be interpreted as described in RFC-2119.

🔝 [Back to table of contents](#)

Mapping WSDL and XML schemas to C/C++                                   {#tocpp}
=====================================

To convert WSDL and XML schemas (XSD files) to code, we use the wsdl2h command
on the command line (or command prompt), after opening a terminal.  The wsdl2h
command generates the data binding interface code that is saved to a special
Web services and data bindings interface header file with extension <i>`.h`</i>
that contains the WSDL service declarations and the data binding interface
declarations in a familiar C/C++ format:

    wsdl2h [options] -o file.h ... XSD and WSDL files ...

This command converts WSDL and XSD files to C++ (or pure C with
<b>`wsdl2h -c`</b>) and saves the data binding interface to a interface header
file <i>`file.h`</i> that uses familiar C/C++ syntax extended with `//gsoap`
[directives](#directives) and annotations.  Notational conventions are used in
the data binding interface to declare serializable C/C++ types and functions
for Web service operations.

The WSDL 1.1/2.0, SOAP 1.1/1.2, and XSD 1.0/1.1 standards are supported by the
gSOAP tools.  In addition, the most popular WS specifications are also
supported, including WS-Addressing, WS-ReliableMessaging, WS-Discovery,
WS-Security, WS-Policy, WS-SecurityPolicy, and WS-SecureConversation.

This article focusses mainly on XML data bindings.  XML data bindings for C/C++
bind XML schema types to C/C++ types.  So integers in XML are bound to C
integers, strings in XML are bound to C or C++ strings, complex types in XML
are bound to C structs or C++ classes, and so on.

A data binding is dual, meaning supporting a two way direction for development.
Either you start with WSDLs and/or XML schemas that are mapped to equivalent
C/C++ types, or you start with C/C++ types that are mapped to XSD types.
Either way, the end result is that you can serialize C/C++ types in XML such
that your XML is an instance of XML schema(s) and is validated against these
schema(s).

This covers all of the following standard XSD components with their optional
attributes and properties:

XSD component  | attributes and properties
-------------- | -------------------------
schema         | targetNamespace, version, elementFormDefault, attributeFormDefault, defaultAttributes
attribute      | name, ref, type, use, default, fixed, form, targetNamespace, wsdl:arrayType
element        | name, ref, type, default, fixed, form, nillable, abstract, substitutionGroup, minOccurs, maxOccurs, targetNamespace
simpleType     | name
complexType    | name, abstract, mixed, defaultAttributesApply
all            | *n/a*
choice         | minOccurs, maxOccurs
sequence       | minOccurs, maxOccurs
group          | name, ref, minOccurs, maxOccurs
attributeGroup | name, ref
any            | minOccurs, maxOccurs
anyAttribute   | *n/a*

And also the following standard XSD directives are covered:

directive  | description
---------- | -----------
import     | Imports a schema into the importing schema for referencing
include    | Include schema component definitions into a schema
override   | Override by replacing schema component definitions
redefine   | Extend or restrict schema component definitions
annotation | Annotates a component

The XSD facets and their mappings to C/C++ are:

XSD facet      | maps to
-------------- | -------
enumeration    | `enum`
simpleContent  | class/struct wrapper with `__item` member
complexContent | class/struct
list           | `enum*` bitmask (`enum*` enumerates a bitmask up to 64 bits)
extension      | class/struct inheritance/extension
restriction    | `typedef` and class/struct inheritance/redeclaration
length         | `typedef` with restricted content length annotation
minLength      | `typedef` with restricted content length annotation
maxLength      | `typedef` with restricted content length annotation
minInclusive   | `typedef` with numerical value range restriction annotation
maxInclusive   | `typedef` with numerical value range restriction annotation
minExclusive   | `typedef` with numerical value range restriction annotation
maxExclusive   | `typedef` with numerical value range restriction annotation
precision      | `typedef` with pattern annotation (pattern used for output, but input is not validated)
scale          | `typedef` with pattern annotation (pattern used for output, but input is not validated)
totalDigits    | `typedef` with pattern annotation (pattern used for output, but input is not validated)
fractionDigits | `typedef` with pattern annotation (pattern used for output, but input is not validated)
pattern        | `typedef` with pattern annotation (define `soap::fsvalidate` callback to validate patterns)
union          | string with union of value

All primitive XSD types are supported, including but not limited to the
following XSD types:

XSD type         | maps to
---------------- | -------
any/anyType      | `_XML` string with literal XML content (or enable DOM with wsdl2h option `-d`)
anyURI           | string (i.e. `char*`, `wchar_t*`, `std::string`, `std::wstring`)
string           | string (i.e. `char*`, `wchar_t*`, `std::string`, `std::wstring`)
boolean          | `bool` (C++) or `enum xsd__boolean` (C)
byte             | `char` (i.e. `int8_t`)
short            | `short` (i.e. `int16_t`)
int              | `int` (i.e. `int32_t`)
long             | `LONG64` (i.e. `long long` and `int64_t`)
unsignedByte     | `unsigned char` (i.e. `uint8_t`)
unsignedShort    | `unsigned short` (i.e. `uint16_t`)
unsignedInt      | `unsigned int` (i.e. `uint32_t`)
unsignedLong     | `ULONG64` (i.e. `unsigned long long` and `uint64_t`)
float            | `float`
double           | `double`
integer          | string or `#import "custom/int128.h"` to use 128 bit `xsd__integer`
decimal          | string or `#import "custom/long_double.h"` to use `long double`
precisionDecimal | string
duration         | string or `#import "custom/duration.h"` to use 64 bit `xsd__duration`
dateTime         | `time_t` or `#import "custom/struct_tm.h"` to use `struct tm` for `xsd__dateTime`
time             | string or `#import "custom/long_time.h"` to use 64 bit `xsd__time`
date             | string or `#import "custom/struct_tm_date.h"` to use `struct tm` for `xsd__date`
hexBinary        | special class/struct `xsd__hexBinary`
base64Binary     | special class/struct `xsd__base64Binary`
QName            | `_QName` string (URI normalization rules are applied)

All other primitive XSD types not listed above are mapped to strings, by
wsdl2h generating a `typedef` to string for these types.  For example,
<i>`xsd:token`</i> is bound to a C++ or C string:

~~~{.cpp}
    typedef std::string  xsd__token; // C++
    typedef char        *xsd__token; // C (wsdl2h option -c)
~~~

This associates a compatible value space to the type with the appropriate XSD
type name used by the soapcpp2-generated serializers.

It is possible to remap types by adding the appropriate mapping rules to
<i>`typemap.dat`</i> as we will explain in more detail in the next section.

Imported custom serializers are intended to extend the C/C++ type bindings when
the default binding to string is not satisfactory to your taste and if the
target platform supports these C/C++ types.  To add custom serializers to
<i>`typemap.dat`</i> for wsdl2h, see [adding custom serializers](#custom) below.

🔝 [Back to table of contents](#)

Using typemap.dat to customize data bindings                          {#typemap}
============================================

Use a <i>`typemap.dat`</i> file to redefine namespace prefixes and to customize type
bindings for the the generated header files produced by the wsdl2h tool.  The
<i>`typemap.dat`</i> is the default file processed by wsdl2h.  Use <b>`wsdl2h -tfile.dat`</b>
option <b>`-tfile.dat`</b> to specify a different mapping file <i>`file.dat`</i>.

Declarations in <i>`typemap.dat`</i> can be broken up over multiple lines by
continuing on the next line by ending each line to be continued with a
backslash <b>`\`</b>.  The limit is 4095 characters per line, whether the line is
broken up or not.

🔝 [Back to table of contents](#)

XML namespace bindings                                               {#typemap1}
----------------------

The wsdl2h tool generates C/C++ type declarations that use `ns1`, `ns2`, etc.
as schema-binding URI prefixes.  These default prefixes are generated somewhat
arbitrarily for each schema targetNamespace URI, meaning that their ordering
may change depending on the WSDL and XSD order of processing with wsdl2h.

Therefore, it is **strongly recommended** to declare your own prefix for each
schema URI in <i>`typemap.dat`</i> to reduce maintaince effort of your code.  This
is more robust when anticipating possible changes of the schema(s) and/or the
binding URI(s) and/or the tooling algorithms.

The first and foremost important thing to do is to define prefix-URI bindings
for our C/C++ code by adding the following line(s) to our <i>`typemap.dat`</i> or make
a copy of this file and add the line(s) that bind our choice of prefix name to
each URI:

    prefix = "URI"

For example, to use `g` as a prefix for the "urn:graph" XML namespace:

    g = "urn:graph"

This produces `g__name` C/C++ type names that are bound to the "urn:graph"
schema by association of `g` to the generated C/C++ types.

This means that <i>`<g:name xmlns:g="urn:graph">`</i> is parsed as an instance of a
`g__name` C/C++ type.  Also <i>`<x:name xmlns:x="urn:graph">`</i> parses as an
instance of `g__name`, because the prefix <i>`x`</i> has the same URI value
<i>`urn:graph`</i>.  Prefixes in XML have local scopes (like variables in a block).

The first run of wsdl2h will reveal the XML namespace URIs, so you do not need
to search WSDLs and XSD files for all of the target namespaces.  Just copy them
from the generated header file after the first run into <i>`typemap.dat`</i> for
editing.

@note Only define a namespace prefix once in <i>`typemap.dat`</i>.  That is, do not
use the same prefix for multiple XML namespace URIs.  This is to avoid
namespace conflicts that may cause failed builds and failures in XML parsing
and XML schema validation.

🔝 [Back to table of contents](#)

XSD type bindings                                                    {#typemap2}
-----------------

Custom C/C++ type bindings can be declared in <i>`typemap.dat`</i> to associate C/C++
types with specific schema types.  These type bindings have four parts:

    prefix__type = declaration | use | ptruse

where

- <b>`prefix__type`</b> is the schema type to be customized (the <b>`prefix__type`</b> name
  uses the common double underscore naming convention);

- <b>`declaration`</b> declares the C/C++ type in the wsdl2h-generated header file.
  This part can be empty if no explicit declaration is needed;

- <b>`use`</b> is an optional part that specifies how the C/C++ type is used in the
  code.  When omitted, it is the same as <b>`prefix__type`</b>;

- <b>`ptruse`</b> is an optional part that specifies how the type is used as a pointer
  type.  By default it is the <b>`use`</b> type name with a <b>`*`</b> or C++11
  <b>`std::shared_ptr<type>`</b> when enabled (see further below).  If <b>`use`</b> is already a
  pointer type by the presence of a <b>`*`</b> in the <b>`use`</b> part, then the default
  <b>`ptruse`</b> type is the same as the <b>`use`</b> type (that is, no double
  pointers <b>`**`</b> will be created in this case).

For example, to map <i>`xsd:duration`</i> to a `long long` (`LONG64`) type that holds
millisecond duration values, we can use the custom serializer declared in
<i>`gsoap/custom/duration.h`</i> by adding the following line to <i>`typemap.dat`</i>:

    xsd__duration = #import "custom/duration.h"

Here, we omitted the second and third parts, because `xsd__duration` is the
name that wsdl2h uses for this type in our generated code so we should leave
the <b>`use`</b> part unspecified.  The third part is omitted to let wsdl2h use
`xsd__duration *` for pointers or `std::shared_ptr<xsd__duration>` if smart
pointers are enabled.

To map <i>`xsd:string`</i> to `wchar_t*` wide strings for C source code output:

    xsd__string = | wchar_t* | wchar_t*

For C++ we can use the `std::wstring` wide string:

    xsd__string = | std::wstring

Note that the first part is empty, because these types do not require a
declaration.  A <b>`ptruse`</b> part is also defined for `wchar_t*`, but this
is actually needed because the wsdl2h tool recognizes that the <b>`use`</b>
part `wchar_t*` is already a pointer.  By contrast, when using 8-bit strings,
it is recommended to use the `SOAP_C_UTFSTRING` flag to enable UTF-8 formatted
strings.

When the auto-generated declaration should be preserved but the <b>`use`</b> or
<b>`ptruse`</b> parts replaced, then we use an ellipsis for the declaration part:

    prefix__type = ... | use | ptruse

This is useful to map schema polymorphic types to C types for example, where we
need to be able to both handle a base type and its extensions as per schema
extensibility.  Say we have a base type called <i>`ns:base`</i> that is extended, then
we can remap this to a C type that permits referening the extended types via a
`void*` as follows:

    ns__base = ... | int __type_base; void*

such that `__type_base` and `void*` will be used to (de)serialize any data
type, including base and its derived types.  The `__type_base` integer is set
to a `SOAP_TYPE_T` value to indicate what type of data the `void*` pointer
points to.

🔝 [Back to table of contents](#)

Custom serializers for XSD types                                       {#custom}
--------------------------------

In the previous part we saw how a custom serializer is used to bind
<i>`xsd:duration`</i> to a `long long` (`LONG64` or `int64_t`) type to store millisecond
duration values:

    xsd__duration = #import "custom/duration.h"

The `xsd__duration` type is an alias of `long long` (`LONG64` or `int64_t`).

While wsdl2h will use this binding declared in <i>`typemap.dat`</i>
automatically, you will also need to compile <i>`gsoap/custom/duration.c`</i>.
Each custom serializer has an interface header file to be imported into another
interface header file that declares the custom type for soapcpp2 and a
serializer implementation file written in C, which should be compiled with the
application.  You can compile these in C++ (rename files to <i>`.cpp`</i> if
needed).

A custom serializer is declared in an interface header file for soapcpp2 using
`extern typedef`.  The typedef name declared is serializable, whereas the
type on which it is based is not serializable.  This declaration can be
combined with `volatile` when the type should not be redeclared, see
[volatile classes and structs](#toxsd9-2).  For example, the custom serializer
for `struct tm` is the type `xsd__datetime` declared as follows in
`gsoap/custom/struct_tm.h`:

~~~{.cpp}
    extern typedef volatile struct tm
    {
        int   tm_sec;   ///< seconds (0 - 60)
        int   tm_min;   ///< minutes (0 - 59)
        int   tm_hour;  ///< hours (0 - 23)
        int   tm_mday;  ///< day of month (1 - 31)
        int   tm_mon;   ///< month of year (0 - 11)
        int   tm_year;  ///< year - 1900
        int   tm_wday;  ///< day of week (Sunday = 0) (NOT USED)
        int   tm_yday;  ///< day of year (0 - 365) (NOT USED)
        int   tm_isdst; ///< is summer time in effect?
        char* tm_zone;  ///< abbreviation of timezone (NOT USED)
    } xsd__dateTime;
~~~

Another example is `xsd__duration` as a custom serializer for the C++11 type
`std::chrono::nanoseconds`:

~~~{.cpp}
    extern typedef class std::chrono::nanoseconds xsd__duration;
~~~

Next, we present all pre-defined custom serializers that are available to you.

🔝 [Back to table of contents](#)

### xsd:integer                                                      {#custom-1}

The wsdl2h tool maps <i>`xsd:integer`</i> to a string by default.  To map <i>`xsd:integer`</i> to
the 128 bit big int type `__int128_t`:

    xsd__integer = #import "custom/int128.h"

The `xsd__integer` type is an alias of `__int128_t`.

@warning Beware that the <i>`xsd:integer`</i> value space of integers is in principle
unbounded and values can be of arbitrary length.  A value range fault
`SOAP_TYPE` (value exceeds native representation) or `SOAP_LENGTH` (value
exceeds range bounds) will be thrown by the deserializer if the value is out of
range.

Other XSD integer types that are restrictions of <i>`xsd:integer`</i>, are
<i>`xsd:nonNegativeInteger`</i> and <i>`xsd:nonPositiveInteger`</i>, which are further restricted
by <i>`xsd:positiveInteger`</i> and <i>`xsd:negativeInteger`</i>.  To bind these types to
`__int128_t` add the following definitions to <i>`typemap.dat`</i>:

    xsd__nonNegativeInteger = typedef xsd__integer xsd__nonNegativeInteger 0 :    ;
    xsd__nonPositiveInteger = typedef xsd__integer xsd__nonPositiveInteger   : 0  ;
    xsd__positiveInteger    = typedef xsd__integer xsd__positiveInteger    1 :    ;
    xsd__negativeInteger    = typedef xsd__integer xsd__negativeInteger      : -1 ;

Or simply uncomment these definitions in <i>`typemap.dat`</i> when you are using the
latest gSOAP releases.

@note If `__int128_t` 128 bit integers are not supported on your platform and if it
is certain that <i>`xsd:integer`</i> values are within 64 bit value bounds for your
application's use, then you can map this type to `LONG64`:

    xsd__integer = typedef LONG64 xsd__integer;

@note Again, a value range fault `SOAP_TYPE` or `SOAP_LENGTH` will be thrown by
the deserializer if the value is out of range.

After running wsdl2h and soapcpp2, compile <i>`gsoap/custom/int128.c`</i> with your project.

@see Section [numerical types](#toxsd5).

🔝 [Back to table of contents](#)

### xsd:decimal                                                      {#custom-2}

The wsdl2h tool maps <i>`xsd:decimal`</i> to a string by default.  To map <i>`xsd:decimal`</i> to
extended precision floating point:

    xsd__decimal = #import "custom/long_double.h" | long double

By contrast to all other custom serializers, this serializer enables `long
double` natively without requiring a new binding name (`xsd__decimal` is NOT
defined).

If your system supports <i>`quadmath.h`</i> quadruple precision floating point
`__float128`, you can map <i>`xsd:decimal`</i> to `xsd__decimal` that is an alias of
`__float128`:

    xsd__decimal = #import "custom/float128.h"

@warning Beware that <i>`xsd:decimal`</i> is in principle a decimal value with arbitraty
lengths.  A value range fault `SOAP_TYPE` will be thrown by the deserializer if
the value is out of range.

In the XML payload the special values <i>`INF`</i>, <i>`-INF`</i>, <i>`NaN`</i>
represent plus or minus infinity and not-a-number, respectively.

After running wsdl2h and soapcpp2, compile <i>`gsoap/custom/long_double.c`</i> with your
project.

@see Section [numerical types](#toxsd5).

🔝 [Back to table of contents](#)

### xsd:dateTime                                                     {#custom-3}

The wsdl2h tool maps <i>`xsd:dateTime`</i> to `time_t` by default.

The trouble with `time_t` when represented as 32 bit `long` integers is that it
is limited to dates between 1970 and 2038.  A 64 bit `time_t` is safe to use if
the target platform supports it, but lack of 64 bit `time_t` portability may
still cause date range issues.

For this reason `struct tm` should be used to represent wider date ranges.  This
custom serializer avoids using date and time information in `time_t`.  You get
the raw date and time information.  You only lose the day of the week
information.  It is always Sunday (`tm_wday=0`).

To map <i>`xsd:dateTime`</i> to `xsd__dateTime` which is an alias of `struct tm`:

    xsd__dateTime = #import "custom/struct_tm.h"

If the limited date range of `time_t` is not a problem but you want to increase
the time precision with fractional seconds, then we suggest to map <i>`xsd:dateTime`</i>
to `struct timeval`:

    xsd__dateTime = #import "custom/struct_timeval.h"

If the limited date range of `time_t` is not a problem but you want to use the
C++11 time point type `std::chrono::system_clock::time_point` (which internally
uses `time_t`):

    xsd__dateTime = #import "custom/chrono_time_point.h"

Again, we should make sure that the dates will not exceed the date range when
using the default `time_t` binding for <i>`xsd:dateTime`</i> or when binding
<i>`xsd:dateTime`</i> to `struct timeval` or to `std::chrono::system_clock::time_point`.
These are safe to use in applications that use <i>`xsd:dateTime`</i> to record date
stamps within a given window.  Otherwise, we recommend the `struct tm` custom
serializer.

After running wsdl2h and soapcpp2, compile <i>`gsoap/custom/struct_tm.c`</i> with your
project.

You could even map <i>`xsd:dateTime`</i> to a plain string (use `char*` with C and
`std::string` with C++).  For example:

    xsd__dateTime = | char*

@see Section [date and time types](#toxsd7).

🔝 [Back to table of contents](#)

### xsd:date                                                         {#custom-4}

The wsdl2h tool maps <i>`xsd:date`</i> to a string by default.  We can map <i>`xsd:date`</i> to
`struct tm`:

    xsd__date = #import "custom/struct_tm_date.h"

The `xsd__date` type is an alias of `struct tm`.  The serializer ignores the
time part and the deserializer only populates the date part of the struct,
setting the time to 00:00:00.  There is no unreasonable limit on the date range
because the year field is stored as an integer (`int`).

After running wsdl2h and soapcpp2, compile <i>`gsoap/custom/struct_tm_date.c`</i> with your
project.

@see Section [date and time types](#toxsd7).

🔝 [Back to table of contents](#)

### xsd:time                                                         {#custom-5}

The wsdl2h tool maps <i>`xsd:time`</i> to a string by default.  We can map <i>`xsd:time`</i> to
an `unsigned long long` (`ULONG64` or `uint64_t`) integer with microsecond time
precision:

    xsd__time = #import "custom/long_time.h"

This type represents 00:00:00.000000 to 23:59:59.999999, from `0` to an upper
bound of `86399999999`.  A microsecond resolution means that a 1 second
increment requires an increment of 1000000 in the integer value.  The serializer
adds a UTC time zone.

After running wsdl2h and soapcpp2, compile <i>`gsoap/custom/long_time.c`</i> with your
project.

@see Section [date and time types](#toxsd7).

🔝 [Back to table of contents](#)

### xsd:duration                                                     {#custom-6}

The wsdl2h tool maps <i>`xsd:duration`</i> to a string by default, unless <i>`xsd:duration`</i>
is mapped to a `long long` (`LONG64` or `int64_t`) type with with millisecond
(ms) time duration precision:

    xsd__duration = #import "custom/duration.h"

The `xsd__duration` type is a 64 bit signed integer that can represent
106,751,991,167 days forwards (positive) and backwards (negative) in time in
increments of 1 ms (1/1000 of a second).

Rescaling of the duration value by may be needed when adding the duration value
to a `time_t` value, because `time_t` may or may not have a seconds resolution,
depending on the platform and possible changes to `time_t`.

Rescaling is done automatically when you add a C++11 `std::chrono::nanoseconds`
value to a `std::chrono::system_clock::time_point` value.  To use
`std::chrono::nanoseconds` as <i>`xsd:duration`</i>:

    xsd__duration = #import "custom/chrono_duration.h"

This type can represent 384,307,168 days (2^63 nanoseconds) forwards and
backwards in time in increments of 1 ns (1/1,000,000,000 of a second).

Certain observations with respect to receiving durations in years and months
apply to both of these serializer decoders for <i>`xsd:duration`</i>.

After running wsdl2h and soapcpp2, compile <i>`gsoap/custom/duration.c`</i> with your
project.

@see Section [time duration types](#toxsd8).

🔝 [Back to table of contents](#)

Custom Qt serializers for XSD types                                        {#qt}
-----------------------------------

The gSOAP distribution includes several custom serializers for Qt types.  Also
Qt container classes are supported, see
[the built-in typemap.dat variables $CONTAINER, $POINTER and $SIZE](#typemap5).

This feature requires gSOAP 2.8.34 or higher and Qt 4.8 or higher.

Each Qt custom serializer has an interface header file for soapcpp2 and a C++
implementation file to be compiled with your project.

Other Qt primitive types that are Qt `typedef`s of C/C++ types do not require a
custom serializer.

🔝 [Back to table of contents](#)

### xsd:string                                                           {#qt-1}

To use Qt strings instead of C++ strings, add the following definition to
<i>`typemap.dat`</i>:

    xsd__string = #import "custom/qstring.h"

After running wsdl2h and soapcpp2, compile <i>`gsoap/custom/qstring.cpp`</i> with your
project.

🔝 [Back to table of contents](#)

### xsd:base64Binary                                                     {#qt-2}

To use Qt byte arrays for <i>`xsd:base64Binary`</i> instead of the
`xsd__base64Binary` class, add the following definition to <i>`typemap.dat`</i>:

    xsd__base64Binary = #import "custom/qbytearray_base64.h"

After running wsdl2h and soapcpp2, compile <i>`gsoap/custom/qbytearray_base64.cpp`</i> with
your project.

🔝 [Back to table of contents](#)

### xsd:hexBinary                                                        {#qt-3}

To use Qt byte arrays for <i>`xsd:hexBinary`</i> instead of the `xsd__base64Binary`
class, add the following definition to <i>`typemap.dat`</i>:

    xsd__hexBinary = #import "custom/qbytearray_hex.h"

After running wsdl2h and soapcpp2, compile <i>`gsoap/custom/qbytearray_hex.cpp`</i> with
your project.

🔝 [Back to table of contents](#)

### xsd:dateTime                                                         {#qt-4}

To use Qt QDateTime for <i>`xsd:dateTime`</i>, add the following definition to
<i>`typemap.dat`</i>:

    xsd__dateTime = #import "custom/datetime.h"

After running wsdl2h and soapcpp2, compile <i>`gsoap/custom/qdatetime.cpp`</i> with
your project.

🔝 [Back to table of contents](#)

### xsd:date                                                             {#qt-5}

To use Qt QDate for <i>`xsd:date`</i>, add the following definition to
<i>`typemap.dat`</i>:

    xsd__date = #import "custom/qdate.h"

After running wsdl2h and soapcpp2, compile <i>`gsoap/custom/qdate.cpp`</i> with your
project.

🔝 [Back to table of contents](#)

### xsd:time                                                             {#qt-6}

To use Qt QDate for <i>`xsd:time`</i>, add the following definition to
<i>`typemap.dat`</i>:

    xsd__time = #import "custom/qtime.h"

After running wsdl2h and soapcpp2, compile <i>`gsoap/custom/qtime.cpp`</i> with your
project.

🔝 [Back to table of contents](#)

Class/struct member additions                                        {#typemap3}
-----------------------------

All generated classes and structs can be augmented with additional
members such as methods, constructors and destructors, and private members:

    prefix__type = $ member-declaration

For example, we can add method declarations and private members to a class, say
`ns__record` as follows:

    ns__record = $ ns__record(const ns__record &); // copy constructor
    ns__record = $ void print();                   // a print method
    ns__record = $ private: int status;            // a private member

Method declarations cannot include any code, because soapcpp2's input permits
only type declarations, not code.

🔝 [Back to table of contents](#)

Replacing XSD types by equivalent alternatives                       {#typemap4}
----------------------------------------------

Type replacements can be given to replace one type entirely with another given
type:

    prefix__type1 == prefix__type2

This replaces all `prefix__type1` by `prefix__type2` in the wsdl2h output.

@warning Do not agressively replace types, because this can cause XML schema
validation to fail when a value-type mismatch is encountered in the XML input.
Therefore, only replace similar types with other similar types that are wider
(e.g.  `short` by `int` and `float` by `double`).

🔝 [Back to table of contents](#)

The built-in typemap.dat variables $CONTAINER, $POINTER and $SIZE    {#typemap5}
-----------------------------------------------------------------

The <i>`typemap.dat`</i> <b>`$CONTAINER`</b> variable defines the container type to use in
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
<b>`xsd__string = | std::string`</b> in <i>`typemap.dat`</i>.

The <i>`typemap.dat`</i> <b>`$POINTER`</b> variable defines the smart pointer to use in the
wsdl2h-generated declarations for C++, which replaces the use of `*` pointers.
For example:

    $POINTER = std::shared_ptr

Not all pointers in the generated output are replaced by smart pointers by
wsdl2h, such as pointers as union members and pointers as struct/class members
that point to arrays of values.

@note The standard smart pointer `std::shared_ptr` is generally safe to use.
Other smart pointers such as `std::unique_ptr` and `std::auto_ptr` may cause
compile-time errors when classes have smart pointer members but no copy
constructor (a default copy constructor).  A copy constructor is required for
non-shared smart pointer copying or swapping.

Alternatives to `std::shared_ptr` of the form `NAMESPACE::shared_ptr` can be
assigned to <b>`$POINTER`</b> when the namespace `NAMESPACE` also implements
`NAMESPACE::make_shared` and when the shared pointer class provides `reset()`
and`get()` methods and the dereference operator.  For example Boost
`boost::shared_ptr`:

    [
    #include <boost/shared_ptr.hpp>
    ]
    $POINTER = boost::shared_ptr

The user-defined content between <b>`[`</b> and <b>`]`</b> ensures that we include the Boost
header files that are needed to support `boost::shared_ptr` and
`boost::make_shared`.

The variable <b>`$SIZE`</b> defines the type of array sizes, which is `int` by
default.  For example, to change array size types to `size_t`:

    $SIZE = size_t

Permissible types are `int` and `size_t`.  This variable does not affect the
size of dynamic arrays, `xsd__hexBinary` and `xsd__base64Binary` types, which
is always `int`.

🔝 [Back to table of contents](#)

User-defined content                                                 {#typemap6}
--------------------

Any other content to be generated by wsdl2h can be included in <i>`typemap.dat`</i> by
enclosing it within brackets <b>`[`</b> and <b>`]`</b> anywhere in the <i>`typemap.dat`</i> file.
Each of the two brackets must appear at the start of a new line.

For example, we can add an `#import "wsa5.h"` to the wsdl2h-generated output as
follows:

    [
    #import "import/wsa5.h"
    ]

which emits the `#import "import/wsa5.h"` literally at the start of the
wsdl2h-generated header file.

🔝 [Back to table of contents](#)

Mapping C/C++ to XML schema                                             {#toxsd}
===========================

The soapcpp2 command generates the data binding implementation code from a data
binding interface <i>`file.h`</i>:

    soapcpp2 [options] file.h

where <i>`file.h`</i> is a interface header file that declares the XML data
binding interface.  The <i>`file.h`</i> is typically generated by wsdl2h, but
you can also declare one yourself.  If so, add `//gsoap`
[directives](#directives) and declare in this file all our C/C++ types you want
to serialize in XML.

You can also declare functions that will be converted to Web service operations
by soapcpp2.  Global function declarations define service operations, which are
of the form:

~~~{.cpp}
    int prefix__func(arg1, arg2, ..., argn, result);
~~~

where `arg1`, `arg2`, ..., `argn` are formal argument declarations of the input
and `result` is a formal argument for the output, which must be a pointer or
reference to the result object to be populated.  More information on declaring
and implementing service operation functions can be found in the
[gSOAP user guide.](../../guide/html/index.html)

🔝 [Back to table of contents](#)

Overview of serializable C/C++ types                                   {#toxsd1}
------------------------------------

The following C/C++ types are supported by soapcpp2 and mapped to XSD types
and constructs.  See the subsections below for more details or follow the links.

🔝 [Back to table of contents](#)

### List of Boolean types

Boolean Type                  | Notes
----------------------------- | -----
`bool`                        | C++ bool
`enum xsd__boolean`           | C alternative to C++ `bool` with `false_` and `true_`
 
@see Section [C++ bool and C alternative](#toxsd3).

🔝 [Back to table of contents](#)

### List of enumeration and bitmask types

Enumeration Type              | Notes
----------------------------- | -----
`enum`                        | enumeration
`enum class`                  | C++11 scoped enumeration, requires `soapcpp2 -c++11`
`enum*`                       | a bitmask that enumerates values 1, 2, 4, 8, ...
`enum* class`                 | C++11 scoped enumeration bitmask, requires `soapcpp2 -c++11`

@see Section [enumerations and bitmasks](#toxsd4).

🔝 [Back to table of contents](#)

### List of numerical types

Numerical Type                | Notes
----------------------------- | -----
`char`                        | byte
`short`                       | 16 bit integer
`int`                         | 32 bit integer
`long`                        | 32 bit integer
`LONG64`                      | 64 bit integer
`xsd__integer`                | 128 bit integer, use `#import "custom/int128.h"`
`long long`                   | same as `LONG64`
`unsigned char`               | unsigned byte
`unsigned short`              | unsigned 16 bit integer
`unsigned int`                | unsigned 32 bit integer
`unsigned long`               | unsigned 32 bit integer
`ULONG64`                     | unsigned 64 bit integer
`unsigned long long`          | same as `ULONG64`
`int8_t`                      | same as `char`
`int16_t`                     | same as `short`
`int32_t`                     | same as `int`
`int64_t`                     | same as `LONG64`
`uint8_t`                     | same as `unsigned char`
`uint16_t`                    | same as `unsigned short`
`uint32_t`                    | same as `unsigned int`
`uint64_t`                    | same as `ULONG64`
`size_t`                      | transient type (not serializable)
`float`                       | 32 bit float
`double`                      | 64 bit float
`long double`                 | extended precision float, use `#import "custom/long_double.h"`
`xsd__decimal`                | `quadmath.h` library 128 bit quadruple precision float, use `#import "custom/float128.h"`
`typedef`                     | declares a type name, with optional value range and string length bounds

@see Section [numerical types](#toxsd5).

🔝 [Back to table of contents](#)

### List of string types

String Type                   | Notes
----------------------------- | -----
`char*`                       | string (may contain UTF-8 with flag `SOAP_C_UTFSTRING`)
`wchar_t*`                    | wide string
`std::string`                 | C++ string (may contain UTF-8 with flag `SOAP_C_UTFSTRING`)
`std::wstring`                | C++ wide string
`char[N]`                     | fixed-size string, requires `soapcpp2 -b`
`_QName`                      | normalized QName content
`_XML`                        | literal XML string content with wide characters in UTF-8
`typedef`                     | declares a new string type name, may restrict string length

@see Section [string types](#toxsd6).

🔝 [Back to table of contents](#)

### List of date and time types

Date and Time Type                      | Notes
--------------------------------------- | -----
`time_t`                                | date and time point since epoch
`struct tm`                             | date and time point, use `#import "custom/struct_tm.h"`
`struct tm`                             | date point, use `#import "custom/struct_tm_date.h"`
`struct timeval`                        | date and time point, use `#import "custom/struct_timeval.h"`
`unsigned long long`                    | time point in microseconds, use `#import "custom/long_time.h"`
`std::chrono::system_clock::time_point` | date and time point, use `#import "custom/chrono_time_point.h"`

@see Section [date and time types](#toxsd7).

🔝 [Back to table of contents](#)

### List of time duration types

Time Duration Type            | Notes
----------------------------- | -----
`long long`                   | duration in milliseconds, use `#import "custom/duration.h"`
`std::chrono::nanoseconds`    | duration in nanoseconds, use `#import "custom/chrono_duration.h"`

@see Section [time duration types](#toxsd8).

🔝 [Back to table of contents](#)

### List of classes, structs, unions, pointers, containers, and arrays

Classes, Structs, and Members | Notes
----------------------------- | -----
`class`                       | C++ class with single inheritance only
`struct`                      | C struct or C++ struct without inheritance
`std::shared_ptr<T>`          | C++11 smart shared pointer
`std::unique_ptr<T>`          | C++11 smart pointer
`std::auto_ptr<T>`            | C++ smart pointer
`std::deque<T>`               | use `#import "import/stldeque.h"`
`std::list<T>`                | use `#import "import/stllist.h"`
`std::vector<T>`              | use `#import "import/stlvector.h"`
`std::set<T>`                 | use `#import "import/stlset.h"`
`template<T> class`           | a container with `begin()`, `end()`, `size()`, `clear()`, and `insert()` methods
`T*`                          | pointer to data of type `T`
`T*`                          | as a class or struct member: points to data of type `T` or array of `T` with member `__size`
`T[N]`                        | as a class or struct member: fixed-size array of type `T`
`union`                       | as a class or struct member: requires a variant selector member `__union`
`void*`                       | as a class or struct member: requires a `__type` member to indicate the type of object pointed to

@see Section [classes and structs](#toxsd9).

🔝 [Back to table of contents](#)

### List of special classes and structs

Special Classes and Structs   | Notes
----------------------------- | -----
Special Array class/struct    | single and multidimensional SOAP Arrays
Special Wrapper class/struct  | complexTypes with simpleContent, wraps `__item` member
`xsd__hexBinary`              | binary content
`xsd__base64Binary`           | binary content and optional DIME/MIME/MTOM attachments
`xsd__anyType`                | DOM elements, use `#import "dom.h"`
`@xsd__anyAttribute`          | DOM attributes, use `#import "dom.h"`

@see Section [special classes and structs](#toxsd10).

🔝 [Back to table of contents](#)

Colon notation versus name prefixing with XML tag name translation     {#toxsd2}
------------------------------------------------------------------

To bind C/C++ type names to XSD types, a simple form of name prefixing is used
by the gSOAP tools by prepending the XML namespace prefix to the C/C++ type
name with a pair of undescrores.  This also ensures that name clashes cannot
occur when multiple WSDL and XSD files are converted to C/C++.  Also, C++
namespaces are not sufficiently rich to capture XML schema namespaces
accurately, for example when class members are associated with schema elements
defined in another XML namespace and thus the XML namespace scope of the
member's name is relevant, not just its type.

However, from a C/C++ centric point of view this can be cumbersome.  Therefore,
colon notation is an alternative to physically augmenting C/C++ names with
prefixes.

For example, the following class uses colon notation to bind the `record` class
to the <i>`urn:types`</i> schema:

~~~{.cpp}
    //gsoap ns schema namespace: urn:types
    class ns:record        // binding 'ns:' to a type name
    {
     public:
        std::string name;
        uint64_t    SSN;
        ns:record   *spouse; // using 'ns:' with the type name
        ns:record();         // using 'ns:' here too
        ~ns:record();        // and here
    };
~~~

The colon notation is stripped away by soapcpp2 when generating the data
binding implementation code for our project.  So the final code just uses
`record` to identify this class and its constructor/destructor.

When using colon notation make sure to be consistent and not use colon notation
mixed with prefixed forms.  The qualified name `ns:record` differs from `ns__record`,
because `ns:record` is compiled to an unqualified `record` name in the source
code output by the soapcpp2 tool.

Colon notation also facilitates overruling the elementFormDefault and
attributeFormDefault declaration that is applied to local elements and
attributes, when declared as members of classes, structs, and unions.  For more
details, see [qualified and unqualified members](#toxsd9-6).

A C/C++ identifier name (a type name, member name, function name, or parameter
name) is translated to an XML tag name by the following rules:

- Two leading underscores indicates that the identifier name has no XML tag
  name, i.e. this name is not visible in XML and is not translated.

- A leading underscore is removed, but the underscore indicates that: **a**) a
  struct/class member name or parameter name has a wildcard XML tag name (i.e.
  matches any XML tag), or **b**) a type name that has a
  [document root element definition](#toxsd9-7).

- Trailing underscores are removed (i.e. trailing underscores can be used to
  avoid name clashes with keywords).

- Underscores within names are translated to hyphens (hyphens are more common
  in XML tag names).

- `_USCORE` is translated to an underscore in the translated XML tag name.

- `_DOT` is translated to a dot (<i>`.`</i>) in the translated XML tag name.

- `_xHHHH` is translated to the Unicode character with code point HHHH (hex).

- C++11 Unicode identifier name characters in UTF-8 are translated as-is.

For example, the C/C++ namespace qualified identifier name `s_a__my_way` is
translated to the XML tag name <i>`s-a:my-way`</i> by translating the prefix `s_a`
and the local name `my_way`.

Struct/class member and parameter name translation can be overruled by using
[backtick XML tags](#toxsd9-5-1) (with gSOAP 2.8.30 and greater).

🔝 [Back to table of contents](#)

C++ bool and C alternative                                             {#toxsd3}
-------------------------- 

The C++ `bool` type is bound to built-in XSD type <i>`xsd:boolean`</i>.

The C alternative is to define an enumeration:

~~~{.cpp}
    enum xsd__boolean { false_, true_ };
~~~

or by defining an enumeration in C with pseudo-scoped enumeration constants:

~~~{.cpp}
    enum xsd__boolean { xsd__boolean__false, xsd__boolean__true };
~~~

The XML value space of these types is <i>`false`</i> and <i>`true`</i>, but also accepted
are <i>`0`</i> and <i>`1`</i> values for <i>`false`</i> and <i>`true`</i>, respectively.

To prevent name clashes, `false_` and `true_` have a trailing underscore in
their `enum` symbols.  Trailing underscores are removed from the XML value space.

🔝 [Back to table of contents](#)

Enumerations and bitmasks                                              {#toxsd4}
-------------------------

Enumerations are mapped to XSD simpleType enumeration restrictions of
<i>`xsd:string`</i>, <i>`xsd:QName`</i>, and <i>`xsd:long`</i>.

Consider for example:

~~~{.cpp}
    enum ns__Color { RED, WHITE, BLUE };
~~~

which maps to a simpleType restriction of <i>`xsd:string`</i> in the soapcpp2-generated
schema:

<div class="alt">
~~~{.xml}
    <simpleType name="Color">
      <restriction base="xsd:string">
        <enumeration value="RED"/>
        <enumeration value="WHITE"/>
        <enumeration value="BLUE"/>
      </restriction>
    </simpleType>
~~~
</div>

Enumeration name constants can be pseudo-scoped to prevent name clashes,
because enumeration name constants have a global scope in C and C++:

~~~{.cpp}
    enum ns__Color { ns__Color__RED, ns__Color__WHITE, ns__Color__BLUE };
~~~

You can also use C++11 scoped enumerations to prevent name clashes:

~~~{.cpp}
    enum class ns__Color : int { RED, WHITE, BLUE };
~~~

Here, the enumeration class base type `: int` is optional.  In place of `int`
in the example above, we can also use `int8_t`, `int16_t`, `int32_t`, or
`int64_t`.

The XML value space of the enumertions defined above is <i>`RED`</i>, <i>`WHITE`</i>, and
<i>`BLUE`</i>.

Prefix-qualified enumeration name constants are mapped to simpleType
restrictions of <i>`xsd:QName`</i>, for example:

~~~{.cpp}
    enum ns__types { xsd__int, xsd__float };
~~~

which maps to a simpleType restriction of <i>`xsd:QName`</i> in the soapcpp2-generated
schema:

<div class="alt">
~~~{.xml}
    <simpleType name="types">
      <restriction base="xsd:QName">
        <enumeration value="xsd:int"/>
        <enumeration value="xsd:float"/>
      </restriction>
    </simpleType>
~~~
</div>

Enumeration name constants can be pseudo-numeric as follows:

~~~{.cpp}
    enum ns__Primes { _3 = 3, _5 = 5, _7 = 7, _11 = 11 };
~~~

which maps to a simpleType restriction of <i>`xsd:long`</i>:

<div class="alt">
~~~{.xml}
    <simpleType name="Color">
      <restriction base="xsd:long">
        <enumeration value="3"/>
        <enumeration value="5"/>
        <enumeration value="7"/>
        <enumeration value="11"/>
      </restriction>
    </simpleType>
~~~
</div>

The XML value space of this type is <i>`3`</i>, <i>`5`</i>, <i>`7`</i>, and <i>`11`</i>.

Besides (pseudo-) scoped enumerations, another way to prevent name clashes
accross enumerations is to start an enumeration name constant with one
underscore or followed it by any number of underscores, which makes it
unique.  The leading and trailing underscores are removed from the XML value
space.

~~~{.cpp}
    enum ns__ABC { A, B, C };
    enum ns__BA  { B, A };    // BAD: B = 1 but B is already defined as 2
    enum ns__BA_ { B_, A_ };  // OK
~~~

The gSOAP soapcpp2 tool permits reusing enumeration name constants across
(non-scoped) enumerations as long as these values are assigned the same
constant.  Therefore, the following is permitted:

~~~{.cpp}
    enum ns__Primes { _3 = 3, _5 = 5, _7 = 7, _11 = 11 };
    enum ns__Throws { _1 = 1, _2 = 2, _3 = 3, _4 = 4, _5 = 5, _6 = 6 };
~~~

A bitmask type is an `enum*` "product enumeration" with a geometric,
power-of-two sequence of values assigned to the enumeration constants:

~~~{.cpp}
    enum* ns__Options { SSL3, TLS10, TLS11, TLS12, TLS13 };
~~~

where the product enum assigns 1 to `SSL3`, 2 to `TLS10`, 4 to `TLS11`, 8
to `TLS12`, and 16 to `TLS13`, which allows these enumeration constants to be
used in composing bitmasks with `|` (bitwise or) `&` (bitwise and), and `~`
(bitwise not):

~~~{.cpp}
    enum ns__Options options = (enum ns__Options)(SSL3 | TLS10 | TLS11 | TLS12 | TLS13);
    if (options & SSL3) // if SSL3 is an option, warn and remove from options
    {
      warning();
      options &= ~SSL3;
    }
~~~

The bitmask type maps to a simpleType list restriction of <i>`xsd:string`</i> in the
soapcpp2-generated XML schema:

<div class="alt">
~~~{.xml}
    <simpleType name="Options">
      <list>
        <restriction base="xsd:string">
          <enumeration value="SSL3"/>
          <enumeration value="TLS10"/>
          <enumeration value="TLS11"/>
          <enumeration value="TLS12"/>
          <enumeration value="TLS13"/>
        </restriction>
      </list>
    </simpleType>
~~~
</div>

The XML value space of this type consists of all 16 possible subsets of the
four values, represented by an XML string with space-separated values.  For
example, the bitmask `TLS10 | TLS11 | TLS12` equals 14 and is represented by
the XML text <i>`TLS10 TLS11 TLS12`</i>.

You can also use C++11 scoped enumerations with bitmasks using `enum*` product
enumerations:

~~~{.cpp}
    enum* class ns__Options { SSL3, TLS10, TLS11, TLS12, TLS13 };
~~~

The base type of a scoped enumeration bitmask, when explicitly given, is
ignored.  The base type is either `int` or `int64_t`, depending on the number
of constants enumerated in the bitmask.

To convert `enum` name constants and bitmasks to a string, we use the
auto-generated function for enum `T`:

~~~{.cpp}
    const char *soap_T2s(struct soap*, enum T val)
~~~

The string returned is stored in an internal buffer of the current `soap`
context, so you should copy it to keep it from being overwritten.  For example,
use `char *soap_strdup(struct soap*, const char*)`.

To convert a string to an `enum` constant or bitmask, we use the auto-generated
function

~~~{.cpp}
    int soap_s2T(struct soap*, const char *str, enum T *val)
~~~
  
This function takes the name (or names, space-separated for bitmasks) of
the enumeration constant in a string `str`.  Names should be given without the
pseudo-scope prefix and without trailing underscores.  The function sets `val`
to the corresponding integer enum constant or to a bitmask.  The function
returns `SOAP_OK` (zero) on success or an error if the string is not a valid
enumeration name.

🔝 [Back to table of contents](#)

Numerical types                                                        {#toxsd5}
---------------

Integer and floating point types are mapped to the equivalent built-in XSD
types with the same sign and bit width.

The `size_t` type is transient (not serializable) because its width is platform
dependent.  We recommend to use `uint64_t` instead.

The XML value space of integer types are their decimal representations without
loss of precision.

The XML value space of floating point types are their decimal representations.
The decimal representations are formatted with the printf format string `"%.9G"`
for floats and the printf format string `"%.17lG"` for double.  To change the
format strings, we can assign new strings to the following `soap` context
members:

~~~{.cpp}
    soap.float_format       = "%g";
    soap.double_format      = "%lg";
    soap.long_double_format = "%Lg";
~~~

Decimal representations may result in a loss of precision of the least
significant decimal.  Therefore, the format strings that are used by default
are sufficiently precise to avoid loss, but this may result in long decimal
fractions in the XML value space.

The `long double` extended floating point type requires a custom serializer:

~~~{.cpp}
    #import "custom/long_double.h"
    ... // use long double
~~~

You can now use `long double`, which has a serializer that serializes this type
as <i>`xsd:decimal`</i>.  Compile and link your code with the file
<i>`gsoap/custom/long_double.c`</i>.

The value space of floating point values includes the special values
<i>`INF`</i>, <i>`-INF`</i>, and <i>`NaN`</i>.  You can check a value for plus
or minus infinity and not-a-number as follows:

~~~{.cpp}
    soap_isinf(x) && x > 0 // is x INF?
    soap_isinf(x) && x < 0 // is x -INF?
    soap_isnan(x)          // is x NaN?
~~~

To assign these values, use:

~~~{.cpp}
    // x is float       // x is double, long double, or __float128
    x = FLT_PINFY;      x = DBL_PINFTY;
    x = FLT_NINFY;      x = DBL_NINFTY;
    x = FLT_NAN;        x = DBL_NAN;
~~~

If your system supports `__float128` then you can also use this 128 bit
floating point type with a custom serializer:

~~~{.cpp}
    #import "custom/float128.h"
    ... // use xsd__decimal
~~~

Then use the `xsd__decimal` alias of `__float128`, which has a serializer.  Do
not use `__float128` directly, which is transient (not serializable).

To check for <i>`INF`</i>, <i>`-INF`</i>, and <i>`NaN`</i> of a `__float128`
value use:

~~~{.cpp}
    isinfq(x) && x > 0 // is x INF?
    isinfq(x) && x < 0 // is x -INF?
    isnanq(x)          // is x NaN?
~~~

The range of a `typedef`-defined numerical type can be restricted using the range
`:` operator with inclusive lower and upper bounds.  For example:

~~~{.cpp}
    typedef int ns__narrow -10 : 10;
~~~

This maps to a simpleType restriction of <i>`xsd:int`</i> in the soapcpp2-generated
schema:

<div class="alt">
~~~{.xml}
    <simpleType name="narrow">
      <restriction base="xsd:int">
        <minInclusive value="-10"/>
        <maxInclusive value="10"/>
      </restriction>
    </simpleType>
~~~
</div>

The lower and upper bound of a range are optional.  When omitted, values are
not bound from below or from above, respectively.

The range of a floating point `typedef`-defined type can be restricted within
floating point constant bounds.

Also with a floating point `typedef` a `printf`-format pattern can be given of the
form `"%[width][.precision]f"` to format decimal values using the given width
and precision fields:

~~~{.cpp}
    typedef float ns__PH "%5.2f" 0.0 : 14.0;
~~~

This maps to a simpleType restriction of <i>`xsd:float`</i> in the soapcpp2-generated
schema:

<div class="alt">
~~~{.xml}
    <simpleType name="PH">
      <restriction base="xsd:float">
        <totalDigits value="5"/>
        <fractionDigits value="2"/>
        <minInclusive value="0"/>
        <maxInclusive value="14"/>
      </restriction>
    </simpleType>
~~~
</div>

For exclusive bounds, we use the `<` operator instead of the `:` range
operator:

~~~{.cpp}
    typedef float ns__epsilon 0.0 < 1.0;
~~~

Values `eps` of `ns__epsilon` are restricted between `0.0 < eps < 1.0`.

This maps to a simpleType restriction of <i>`xsd:float`</i> in the soapcpp2-generated
schema:

<div class="alt">
~~~{.xml}
    <simpleType name="epsilon">
      <restriction base="xsd:float">
        <minExclusive value="0"/>
        <maxExclusive value="1"/>
      </restriction>
    </simpleType>
~~~
</div>

To make just one of the bounds exclusive, while keeping the other bound
inclusive, we add a `<` on the left or on the right side of the range ':'
operator.  For example:

~~~{.cpp}
    typedef float ns__pos 0.0 < : ; // 0.0 < pos
    typedef float ns__neg : < 0.0 ; // neg < 0.0
~~~

It is valid to make both left and right side exclusive with `< : <` which is in
fact identical to the exlusive range `<` operator:

~~~{.cpp}
    typedef float ns__epsilon 0.0 < : < 1.0; // 0.0 < eps < 1.0
~~~

It helps to think of the `:` as a placeholder of the value between the two
bounds, which is easier to memorize than the shorthand forms of bounds from
which the `:` is removed:

| bounds       | validation check | shorthand   |
| ------------ | ---------------- | ----------- |
| `1 :       ` | 1 <= x           | `1        ` |
| `1 : 10    ` | 1 <= x <= 10     |             |
| `  : 10    ` |      x <= 10     |             |
| `1 < : < 10` | 1 <  x <  10     | `1 < 10   ` |
| `1   : < 10` | 1 <= x <  10     |             |
| `    : < 10` |      x <  10     | `  < 10   ` |
| `1 < :     ` | 1 <  x           | `1 <      ` |
| `1 < : 10  ` | 1 <  x <= 10     |             |

Besides `float`, also `double` and `long double` values can be restricted.  For
example, consider a nonzero probability extended floating point precision type:

~~~{.cpp}
    #import "custom/long_double.h"
    typedef long double ns__probability "%16Lg" 0.0 < : 1.0;
~~~

Value range restrictions are validated by the parser for all inbound XML data.
A type fault `SOAP_TYPE` will be thrown by the deserializer if the value is out
of range.

Finally, if your system supports `__int128_t` then you can also use this 128
bit integer type with a custom serializer:

~~~{.cpp}
    #import "custom/int128.h"
    ... // use xsd__integer
~~~

Use the `xsd__integer` alias of `__int128_t`, which has a serializer.  Do not
use `__int128_t` directly, which is transient (not serializable).

To convert numeric values to a string, we use the auto-generated function for
numeric type `T`:

~~~{.cpp}
    const char *soap_T2s(struct soap*, T val)
~~~

For numeric types `T`, the string returned is stored in an internal buffer of
the current `soap` context, so you should copy it to keep it from being
overwritten.  For example, use `char *soap_strdup(struct soap*, const char*)`.

To convert a string to a numeric value, we use the auto-generated function

~~~{.cpp}
    int soap_s2T(struct soap*, const char *str, T *val)
~~~
  
where `T` is for example `int`, `LONG64`, `float`, `decimal` (the custom
serializer name of `long double`) or `xsd__integer` (the custom serializer name
of `__int128_t`).  The function `soap_s2T` returns `SOAP_OK` on success or an
error when the value is not numeric.  For floating point types, `"INF"`, `"-INF"`
and `"NaN"` are valid strings to convert to numbers.

🔝 [Back to table of contents](#)

String types                                                           {#toxsd6}
------------

String types are mapped to the built-in <i>`xsd:string`</i> and <i>`xsd:QName`</i> XSD types.

The wide strings `wchar_t*` and `std::wstring` may contain Unicode that is
preserved in the XML value space.

Strings `char*` and `std::string` can only contain extended Latin, but we can
store UTF-8 content that is preserved in the XML value space when the `soap`
context is initialized with the flag `SOAP_C_UTFSTRING`.

@warning Beware that many XML 1.0 parsers reject all control characters (those
between `#x1` and `#x1F`) except for `#x9`, `#xA`, and `#xD`.  With the
newer XML 1.1 version parsers (including gSOAP) you should be fine.

The length of a string of a `typedef`-defined string type can be restricted:

~~~{.cpp}
    typedef std::string ns__password 6 : 16;
~~~

which maps to a simpleType restriction of <i>`xsd:string`</i> in the soapcpp2-generated
schema:

<div class="alt">
~~~{.xml}
    <simpleType name="password">
      <restriction base="xsd:string">
        <minLength value="6"/>
        <maxLength value="16"/>
      </restriction>
    </simpleType>
~~~
</div>

String length restrictions are validated by the parser for inbound XML data.
A value length fault `SOAP_LENGTH` will be thrown by the deserializer if the
string is too long or too short.

In addition, an XSD regex pattern restriction can be associated with a string
typedef:

~~~{.cpp}
    typedef std::string ns__password "([a-zA-Z]|[0-9]|-)+" 6 : 16;
~~~

which maps to a simpleType restriction of <i>`xsd:string`</i> in the soapcpp2-generated
schema:

<div class="alt">
~~~{.xml}
    <simpleType name="password">
      <restriction base="xsd:string">
        <pattern value="([a-zA-Z0-9]|-)+"/>
        <minLength value="6"/>
        <maxLength value="16"/>
      </restriction>
    </simpleType>
~~~
</div>

Pattern restrictions are validated by the parser for inbound XML data only if
the `soap::fsvalidate` and `soap::fwvalidate` callbacks are defined.

Exclusive length bounds can be used with strings:

~~~{.cpp}
    typedef std::string ns__string255 : < 256; // same as 0 : 255
~~~

Fixed-size strings (`char[N]`) are rare occurrences in the wild, but apparently
still used in some projects to store strings.  To facilitate fixed-size string
serialization, use <b>`soapcpp2 -b`</b> option <b>`-b`</b>.  For example:

~~~{.cpp}
    typedef char ns__buffer[10]; // requires soapcpp2 option -b
~~~

which maps to a simpleType restriction of <i>`xsd:string`</i> in the soapcpp2-generated
schema:

<div class="alt">
~~~{.xml}
    <simpleType name="buffer">
      <restriction base="xsd:string">
        <maxLength value="9"/>
      </restriction>
    </simpleType>
~~~
</div>

Fixed-size strings must contain NUL-terminated text and should not contain raw
binary data.  Also, the length limitation is more restrictive for UTF-8 content
(enabled with the `SOAP_C_UTFSTRING`) that requires multibyte character
encodings.  As a consequence, UTF-8 content may be truncated to fit.

Raw binary data can be stored in a `xsd__base64Binary` or `xsd__hexBinary`
structure, or transmitted as a MIME attachment.

The built-in `_QName` type is a regular C string type (`char*`) that maps to
<i>`xsd:QName`</i> but has the added advantage that it holds normalized qualified names.
There are actually two forms of normalized QName content, to ensure any QName
is represented accurately:

~~~{.cpp}
    "prefix:name"
    "\"URI\":name"
~~~

The first form of string is used when the prefix (and the binding URI) is
defined in the namespace table and is bound to a URI (see the .nsmap file).
The second form is used when the URI is not defined in the namespace table and
therefore no prefix is available to bind and normalize the URI to.

A `_QName` string may contain a sequence of space-separated QName values, not
just one, and all QName values are normalized to the format shown above.

To define a `std::string` base type for <i>`xsd:QName`</i>, we use a `typedef`:

~~~{.cpp}
    typedef std::string xsd__QName;
~~~

The `xsd__QName` string content is normalized, just as with the `_QName`
normalization.

To serialize strings that contain literal XML content to be reproduced in the
XML value space, use the built-in `_XML` string type, which is a regular C
string type (`char*`) that maps to plain XML CDATA.

To define a `std::string` base type for literal XML content, use a `typedef`:

~~~{.cpp}
    typedef std::string XML;
~~~

Strings can hold any of the values of the XSD built-in primitive types.  We can
use a string `typedef` to declare the use of the string type as a XSD built-in
type:

~~~{.cpp}
    typedef std::string xsd__token;
~~~

You must ensure that the string values we populate in this type conform to the
XML standard, which in case of <i>`xsd:token`</i> is the lexical and value spaces of
<i>`xsd:token`</i> are the sets of all strings after whitespace replacement of any
occurrence of `#x9`, `#xA` , and `#xD` by `#x20` and collapsing.

As of version 2.8.49, the gSOAP parser will automatically collapse or replace
the white space content when receiving data for XSD types that require white
space collapsed or replaced.  This normalization is applied to strings
directly.  The decision to collapse or replace is based on the `typedef` name
corresponding to the built-in string-based XSD type.

To copy `char*` or `wchar_t*` strings with a context that manages the allocated
memory, use functions

~~~{.cpp}
    char *soap_strdup(struct soap*, const char*)
    wchar_t *soap_wstrdup(struct soap*, const wchar_t*)
~~~

To convert a wide string to a UTF-8 encoded string, use function

~~~{.cpp}
    const char* SOAP_FMAC2 soap_wchar2s(struct soap*, const wchar_t *s)
~~~

The function allocates and returns a string, with its memory being managed by
the context.

To convert a UTF-8 encoded string to a wide string, use function

~~~{.cpp}
    int soap_s2wchar(struct soap*, const char *from, wchar_t **to, long minlen, long maxlen)
~~~

where `to` is set to point to an allocated `wchar_t*` string.  Pass `-1` for
`minlen` and `maxlen` to ignore length constraints on the target string.  The
function returns `SOAP_OK` or an error when the length constraints are not met.

🔝 [Back to table of contents](#)

Date and time types                                                    {#toxsd7}
-------------------

The C/C++ `time_t` type is mapped to the built-in <i>`xsd:dateTime`</i> XSD type that
represents a date and time within a time zone (typically UTC).

The XML value space contains ISO 8601 Gregorian time instances of the form
<i>`[-]CCYY-MM-DDThh:mm:ss.sss[Z|(+|-)hh:mm]`</i>, where <i>`Z`</i> is the UTC time zone
or a time zone offset <i>`(+|-)hh:mm]`</i> from UTC is used.

A `time_t` value is considered and represented in UTC by the serializer.

Because the `time_t` value range is restricted to dates after 01/01/1970 and
before 2038 assuming `time_t` is a `long` 32 bit, care must be taken to ensure
the range of <i>`xsd:dateTime`</i> values in XML exchanges do not exceed the `time_t`
range.

This restriction does not hold for `struct tm` (<i>`time.h`</i> library), which we can use
to store and exchange a date and time in UTC without date range restrictions.
The serializer uses the `struct tm` members directly for the XML value space of
<i>`xsd:dateTime`</i>:

~~~{.cpp}
    struct tm
    {
        int    tm_sec;   // seconds (0 - 60)
        int    tm_min;   // minutes (0 - 59)
        int    tm_hour;  // hours (0 - 23)
        int    tm_mday;  // day of month (1 - 31)
        int    tm_mon;   // month of year (0 - 11)
        int    tm_year;  // year - 1900
        int    tm_wday;  // day of week (Sunday = 0) (NOT USED)
        int    tm_yday;  // day of year (0 - 365) (NOT USED)
        int    tm_isdst; // is summer time in effect?
        char*  tm_zone;  // abbreviation of timezone (NOT USED)
    };
~~~

You will lose the day of the week information.  It is always Sunday
(`tm_wday=0`) and the day of the year is not set either.  The time zone is UTC.

This `struct tm` type is mapped to the built-in <i>`xsd:dateTime`</i> XSD type and
serialized with the custom serializer <i>`gsoap/custom/struct_tm.h`</i> that declares a
`xsd__dateTime` type:

~~~{.cpp}
    #import "custom/struct_tm.h" // import typedef struct tm xsd__dateTime;
    ... // use xsd__dateTime
~~~

Compile and link your code with <i>`gsoap/custom/struct_tm.c`</i>.

The `struct timeval` (<i>`sys/time.h`</i> library) type is mapped to the
built-in <i>`xsd:dateTime`</i> XSD type and serialized with the custom serializer
<i>`gsoap/custom/struct_timeval.h`</i> that declares a `xsd__dateTime` type:

~~~{.cpp}
    #import "custom/struct_timeval.h" // import typedef struct timeval xsd__dateTime;
    ... // use xsd__dateTime
~~~

Compile and link your code with <i>`gsoap/custom/struct_timeval.c`</i>.

The same value range restrictions apply to `struct timeval` as they apply to
`time_t`.  The added benefit of `struct timeval` is the addition of a
microsecond-precise clock:

~~~{.cpp}
    struct timeval
    {
        time_t       tv_sec;  // seconds since Jan. 1, 1970
        suseconds_t  tv_usec; // and microseconds
    };
~~~

A C++11 `std::chrono::system_clock::time_point` type is mapped to the built-in
<i>`xsd:dateTime`</i> XSD type and serialized with the custom serializer
<i>`gsoap/custom/chrono_time_point.h`</i> that declares a `xsd__dateTime` type:

~~~{.cpp}
    #import "custom/chrono_time_point.h" // import typedef std::chrono::system_clock::time_point xsd__dateTime;
    ... // use xsd__dateTime
~~~

Compile and link your code with <i>`gsoap/custom/chrono_time_point.cpp`</i>.

The `struct tm` type is mapped to the built-in <i>`xsd:date`</i> XSD type and serialized
with the custom serializer <i>`gsoap/custom/struct_tm_date.h`</i> that declares a
`xsd__date` type:

~~~{.cpp}
    #import "custom/struct_tm_date.h" // import typedef struct tm xsd__date;
    ... // use xsd__date
~~~

Compile and link your code with <i>`gsoap/custom/struct_tm_date.c`</i>.

The XML value space of <i>`xsd:date`</i> are Gregorian calendar dates of the form
<i>`[-]CCYY-MM-DD[Z|(+|-)hh:mm]`</i> with a time zone.

The serializer ignores the time part and the deserializer only populates the
date part of the struct, setting the time to 00:00:00.  There is no unreasonable
limit on the date range because the year field is stored as an integer (`int`).

An `unsigned long long` (`ULONG64` or `uint64_t`) type that contains a 24 hour
time in microseconds UTC is mapped to the built-in <i>`xsd:time`</i> XSD type and
serialized with the custom serializer <i>`gsoap/custom/long_time.h`</i> that declares a
`xsd__time` type:

~~~{.cpp}
    #import "custom/long_time.h" // import typedef unsigned long long xsd__time;
    ... // use xsd__time
~~~

Compile and link your code with <i>`gsoap/custom/long_time.c`</i>.

This type represents `00:00:00.000000` to `23:59:59.999999`, from 0 to an
upper bound of 86,399,999,999.  A microsecond resolution means that a 1 second
increment requires an increment of 1,000,000 in the integer value.

The XML value space of <i>`xsd:time`</i> are points in time recurring each day of the
form <i>`hh:mm:ss.sss[Z|(+|-)hh:mm]`</i>, where <i>`Z`</i> is the UTC time zone or a time
zone offset from UTC is used.  The `xsd__time` value is always considered and
represented in UTC by the serializer.

To convert date and/or time values to a string, we use the auto-generated
function for type `T`:

~~~{.cpp}
    const char *soap_T2s(struct soap*, T val)
~~~

For date and time types `T`, the string returned is stored in an internal
buffer of the current `soap` context, so you should copy it to keep it from being
overwritten.  For example, use `char *soap_strdup(struct soap*, const char*)`.

To convert a string to a date/time value, we use the auto-generated function

~~~{.cpp}
    int soap_s2T(struct soap*, const char *str, T *val)
~~~
  
where `T` is for example `dateTime` (for `time_t`), `xsd__dateTime` (for
`struct tm`, `struct timeval`, or `std::chrono::system_clock::time_point`).
The function `soap_s2T` returns `SOAP_OK` on success or an error when the value
is not a date/time.

🔝 [Back to table of contents](#)

Time duration types                                                    {#toxsd8}
-------------------

The XML value space of <i>`xsd:duration`</i> are values of the form <i>`PnYnMnDTnHnMnS`</i>
where the capital letters are delimiters.  Delimiters may be omitted when the
corresponding member is not used.

A `long long` (`LONG64` or `int64_t`) type that contains a duration (time
lapse) in milliseconds is mapped to the built-in <i>`xsd:duration`</i> XSD type and
serialized with the custom serializer <i>`gsoap/custom/duration.h`</i> that declares a
`xsd__duration` type:

~~~{.cpp}
    #import "custom/duration.h" // import typedef long long xsd__duration;
    ... // use xsd__duration
~~~

Compile and link your code with <i>`gsoap/custom/duration.c`</i>.

The duration type `xsd__duration` can represent 106,751,991,167 days forward
and backward with millisecond precision.

Durations that exceed a month are always output in days, rather than months to
avoid days-per-month conversion inacurracies.

Durations that are received in years and months instead of total number of days
from a reference point are not well defined, since there is no accepted
reference time point (it may or may not be the current time).  The decoder
simple assumes that there are 30 days per month.  For example, conversion of
"P4M" gives 120 days.  Therefore, the durations "P4M" and "P120D" are assumed
to be identical, which is not necessarily true depending on the reference point
in time.

Rescaling of the duration value by may be needed when adding the duration value
to a `time_t` value, because `time_t` may or may not have a seconds resolution,
depending on the platform and possible changes to `time_t`.

Rescaling is done automatically when you add a C++11 `std::chrono::nanoseconds`
value to a `std::chrono::system_clock::time_point` value.  To use
`std::chrono::nanoseconds` as <i>`xsd:duration`</i>:

~~~{.cpp}
    #import "custom/chrono_duration.h" // import typedef std::chrono::duration xsd__duration;
    ... // use xsd__duration
~~~

Compile and link your code with <i>`gsoap/custom/chrono_duration.cpp`</i>.

This type can represent 384,307,168 days (2^63 nanoseconds) forwards and
backwards in time in increments of 1 ns (1/1000000000 second).

The same observations with respect to receiving durations in years and months
apply to this serializer's decoder.

To convert duration values to a string, we use the auto-generated function

~~~{.cpp}
    const char *soap_xsd__duration2s(struct soap*, xsd__duration val)
~~~

The string returned is stored in an internal buffer, so you should copy it to
keep it from being overwritten,  Use `soap_strdup(struct soap*, const char*)`
for example to copy this string.

To convert a string to a duration value, we use the auto-generated function

~~~{.cpp}
    int soap_s2xsd__dateTime(struct soap*, const char *str, xsd__dateTime *val)
~~~

The function returns `SOAP_OK` on success or an error when the value is not a
duration.

🔝 [Back to table of contents](#)

Classes and structs                                                    {#toxsd9}
-------------------

Classes and structs are mapped to XSD complexTypes.  The XML value space
consists of XML elements with attributes and subelements, possibly constrained
by XML schema validation rules that enforce element and attribute occurrence
contraints, numerical value range constraints, and string length and pattern
constraints.

Classes that are declared with the gSOAP tools are limited to single
inheritence only.  The soapcpp2 tool does not allow structs to be inherited.

The class and struct name is bound to an XML namespace by means of the prefix
naming convention or by using [colon notation](#toxsd1):

~~~{.cpp}
    //gsoap ns schema namespace: urn:types
    class ns__record
    {
     public:
        std::string  name;
        uint64_t     SSN;
        ns__record  *spouse;
        ns__record();
        ~ns__record();
      protected:
        struct soap  *soap;
    };
~~~

In the example above, we also added a context pointer to the `soap` context that
manages this instance.  It is set when the instance is created in the engine's
context, for example when deserialized and populated by the engine.

The class maps to a complexType in the soapcpp2-generated XML schema:

<div class="alt">
~~~{.xml}
    <complexType name="record">
      <sequence>
        <element name="name" type="xsd:string" minOccurs="1" maxOccurs="1"/>
        <element name="SSN" type="xsd:unsignedLong" minOccurs="1" maxOccurs="1"/>
        <element name="spouse" type="ns:record" minOccurs="0" maxOccurs="1" nillable="true"/>
      </sequence>
    </complexType>
~~~
</div>

The following sections apply to both structs and classes.  Structs require the
use of the `struct` keyword with the struct name, otherwise soapcpp2 will throw
a syntax error.  As is often done in C, use a `typedef` to declare a `struct`
that can be used without the `struct` keyword.

🔝 [Back to table of contents](#)

### Serializable versus transient types and data members             {#toxsd9-1}

Public data members of a class or struct are serializable when their types are
serializable.  Private and protected members are transient and not
serializable.

Also `const` and `static` members are not serializable, with the exception of
`const char*` and `const wchar_t*`.  Types and specific class/struct members
can be made transient with the `extern` qualifier for types and by marking
members with `[` and `]`:

~~~{.cpp}
    extern class std::ostream;     // declare std::ostream transient
    class ns__record
    {
     public:
      [ int              num; ]      // not serialized: member is marked transient with [ ]
        std::ostream     out;        // not serialized: std:ostream is transient
        static const int MAX = 1024; // not serialized: static const member
      private:
        std::string      id;         // not serialized: private member
    };
~~~

By declaring `std::ostream` transient with `extern` you can use this type
wherever you need it without soapcpp2 complaining that this class and any other 
class or type declared as `extern` is not defined.  Do not use `extern` with
`typedef`, because this declares a custom serializer, see
[adding custom serializers](#custom).

Marking members transient with `[` and `]` makes them transient (and visually
makes them stand out).  This has otherwise no effect on the generated code for
the class or struct to be used in your application code.

🔝 [Back to table of contents](#)

### Derived types in C++                                           {#toxsd9-1-1}

Extensible and restricted types in XML schemas are derived types from single
simple and complex base types.  XML schema derived types are naturally
represented by C++ derived classes using single inheritance.  Besides the
concept of extensions versus restrictions, there are two kinds of derived
types: complexTypes with simpleContent, meaning types with XML CDATA values,
and complexTypes with complexContent, meaning types with sub-elements.  Both
are permitted to have one or more XML attributes.

A complexType with simpleContent is defined as a wrapper to contain XML CDATA
values and any number of attributes, see
[wrapper class/struct with simpleContent](#toxsd10-4).
Wrapper class/struct types can form a hierarchy of derived types in C++ using
inheritance.  For example:

~~~{.cpp}
    class xsd__anyType
    {
     public:
        std::string __item; // string to hold any simpleContent
    };
    class ns__data : public xsd__anyType
    {
     public:
      @ std::string value 1; // extends xsd:anyType with a required attribute
    };
~~~

The `ns__data` class maps to a complexType in the soapcpp2-generated XML schema:

<div class="alt">
~~~{.xml}
    <complexType name="string">
      <simpleContent>
        <extension base="xsd:string">
          <attribute name="value" type="xsd:string" use="required"/>
        </extension>
      </simpleContent>
    </complexType>
~~~
</div>

The XML value space consists of an element with the string contents an optional
attribute:

<div class="alt">
~~~{.xml}
    <ns:data value="abc">xyz</ns:data>
~~~
</div>

By contrast, a complexType with complexContent typically extends a given base
complexType.  For example:

~~~{.cpp}
    class ns__base
    {
     public:
        std::string name   1;
        int         number 1;
    };
    class ns__derived : public ns__base
    {
     public:
      @ std::string value 1; // extends ns:base with an attribute
        std::string text  1; // extends ns:base with an element
    };
~~~

The `ns__base` and `ns__derived` classes maps to complexTypes in the soapcpp2-generated XML schema:

<div class="alt">
~~~{.xml}
    <complexType name="base">
      <sequence>
        <element name="name" type="xsd:string" minOccurs="1" maxOccurs="1"/>
        <element name="number" type="xsd:int" minOccurs="1" maxOccurs="1"/>
      </sequence>
    </complexType>
    <complexType name="derived">
      <complexContent>
        <extension base="ns:base">
          <sequence>
            <element name="text" type="xsd:string" minOccurs="1" maxOccurs="1"/>
          </sequence>
        </extension>
      </complexContent>
      <attribute name="value" type="xsd:string" use="required"/>
    </complexType>
~~~
</div>

The XML value space of `ns__derived` consists of three requires child elements
and an optional attribute:

<div class="alt">
~~~{.xml}
    <ns:derived value="abc">
      <name>def</name>
      <number>123</number>
      <text>xyz</text>
    </ns:derived>
~~~
</div>

Derived types can be used for two main purposes in XML schema by extending or
restricting base types.  One purpose is to reuse a base type when defining a
derived type, such that common parts do not need to be replicated.  The second
purpose is to be able to use a derived type in place of a base type in XML, which
is indicated by an <i>`xsi:type`</i> attribute with the qualified name of the
derived type.  Consider for example the following class that uses the
previously declared base types `xsd__anyType` and `ns__base`:

~~~{.cpp}
    class ns__record
    {
     public:
        xsd__anyType *base1 1; // required element
        ns__base     *base2 1; // required element
    };
~~~

We can assign base type values to the `ns_record` members:

~~~{.cpp}
    ns__record record;
    record.base1 = soap_new_xsd__anyType(soap);
    record.base2 = soap_new_ns__base(soap);
    soap_write_ns__record(soap, &record);
~~~

This produces the following XML fragment populated with default values (empty
text for strings and zeros for numbers), where element <i>`base1`</i> has a
simpleContent value and element <i>`base2`</i> has two child elements:

<div class="alt">
~~~{.xml}
    <ns:record>
      <base1></base1>
      <base2>
        <name></name>
        <number>0</number>
      </base2>
    </ns:record>
~~~
</div>

We can also assign derived type values to the `ns_record` members:

~~~{.cpp}
    ns__record record;
    record.base1 = soap_new_ns__data(soap);
    record.base2 = soap_new_ns__derived(soap);
    soap_write_ns__record(soap, &record);
~~~

This produces the following XML fragment populated with default values (empty
text for strings and zeros for numbers), where element <i>`base1`</i> has
schema type <i>`ns:data`</i> with simpleContent and an attribute, and
<i>`base2`</i> has schema type <i>`ns:derived`</i> with three child elements
and an attribute:

<div class="alt">
~~~{.xml}
    <ns:record>
      <base1 xsi:type="ns:data" value=""></base1>
      <base2 xsi:type="ns:derived" value="">
        <name></name>
        <number>0</number>
        <text></text>
      </base2>
    </ns:record>
~~~
</div>

Deserialization automatically allocates and assigns a `ns__base` class instance to a
`ns__base` pointer when deserializing the <i>`ns:base`</i> schema type and allocates and
assigns a `ns__derived` class instance to a `ns__base` pointer when deserializing the
<i>`ns:derived`</i> type when an element with <i>`xsi:type="ns:derived"`</i> is
parsed.  All classes are extended by soapcpp2 by a `soap_type()` method that
returns the unique `SOAP_TYPE_T` value of the class `T`.  This makes it easy to
check whether the deserialized data contains a derived type to implement
type-safe code, for example:

~~~{.cpp}
    ns__record record;
    soap_read_ns__record(soap, &record);
    if (record.base1->soap_type() == SOAP_TYPE_ns__data)
      std::cout << "Derived ns:data "
                << dynamic_cast<ns__data*>(record.base1)->value
                << std::endl;
    else
      std::cout << "Base xsd:anyType" << std::endl;
    if (record.base2->soap_type() == SOAP_TYPE_ns__derived)
      std::cout << "Derived ns:derived "
                << dynamic_cast<ns__derived*>(record.base2)->value
                << std::endl;
    else
      std::cout << "Base ns:base" << std::endl;
~~~

This example should use the `SOAP_XML_STRICT` mode flag to initialize the
`soap` context to ensure that all required values are present in the
deserialized structures.

🔝 [Back to table of contents](#)

### Derived types in C                                             {#toxsd9-1-2}

While single inheritance works well in C++ to represent derived types as we
discussed in the previous section, this will obviously not work in C.  Two
methods to serialize derived types in C are presented here.  The first method
uses `void*` to serialize anything.  The second method is more accurate and is
relatively new in gSOAP.

To serialize any type is possible with [tagged void pointer members](#toxsd9-12) to
serialize data pointed to by a `void*` member, which can be any serializable
type, such as derived types.  For `void*` deserialization to work the XML
parsed must contain an <i>`xsi:type`</i> attribute with a schema type.  Only
then can the deserializer instantiate the corresponding serializable C/C++
type.  Base types serialized do not require an <i>`xsi:type`</i> to indicate
the base schema type, so this approach is not guaranteed to work and requires a
workaround with an anonymous wrapper struct/class that contains both the base
type and a `void*`.  For example:

~~~{.cpp}
    struct ns__base // a base type
    {
        char *name   1;
        int   number 1;
    };
    struct ns__derived // extends ns__base with two additional members
    {
        char  *name   1;
        int    number 1;
        char  *text   1;
      @ char *value   1;
    };
    struct __ns__base // a wrapper, not visible in XML
    {
        int __type;              // the SOAP_TYPE_T pointed to by __self
        void *__self;            // points to any type
        struct ns__base *__self; // wraps ns__base for the current element tag
    }
    class ns__record
    {
      struct __ns__base base;
    };
~~~

The `__ns__base` wrapper wraps the `ns__base` type to (de)serialize the
<i>`base`</i> element that has no <i>`xsi:type`</i> attribute and uses `void*`
to (de)serialize the <i>`base`</i> element that has <i>`xsi:type`</i>
attribute.  This works fine at the XML parsing level, but the generated
XML schema components do not accurately represent the derived type, because it
lacks the extension/restriction of the derived type (and the `__ns__base`
wrapper is invisible).

Using `void*` to represent derived types in a base type wrapper is not very
accurate because we can serialize anything, not just derived types of a given
base type.  The wrapper may also hold two values: the base type value and a
derived type value.  Furthermore, using arrays or containers that hold base and
derived types becomes quite tricky because an array item could hold both the
base and derived type.

As of gSOAP version 2.8.75, `wsdl2h -F` option `-F` generates base type structs
extended with transient pointer members to its derived types.  To serialize the
base type itself, all of the pointer members are NULL. If one of the pointer
members points to a derived type the derived type is serialized instead.
Deserialization is automatic, in that the base type is deserialized if the
element has no <i>`xsi:type`</i> attribute or the attribute is the base schema
type, and a derived type is deserialized if the element has an
<i>`xsi>type`</i> attribute with the derived schema type.

This method is fully automated for the wsdl2h tool to generate an interface
header file for soapcpp2 with the type derivations in C.  To use this method to
generate code from WSDLs and XSDs, use `wsdl2h -F` option `-F`.  This also
works in C++ if desired, but C++ inheritance works fine without this method.

Using this method with soapcpp2 alone using a manually-specified interface
header file produces the specified type inheritance in the soapcpp2-generated
WSDL and XML schema files as complexType extensions.

The soapcpp2 tool warns if a derived type has multiple base types.  At most one
base type for a derived type may be specified.

This method with transient pointers to derived types makes it easy to use base
and derived types in C:

~~~{.cpp}
    struct ns__base // a base type
    {
        char *name   1;
        int   number 1;
      [ struct ns__derived *ns__derived; ] // points to derived type if non-NULL
    };
    struct ns__derived // extends ns__base with two additional members
    {
        char  *name   1;
        int    number 1;
        char  *text   1;
      @ char *value   1;
    };
    struct ns__record
    {
        struct ns__base base; // contains base type or derived type value
    };
~~~

The `ns__base` struct includes the special member `ns__derived` that points to
a `ns__derived` value.  This special member must be:

- a transient member (i.e. non-serializable) by placing the declaration within
  `[` and `]`, and
- the member name must match the type name (to be more precise, at least the
  initial part of the member name must match the type name as in the example
  `ns__derived_` works too).

To serialize the `ns__base` value requires the `ns__derived` member to be NULL.
To serialize the `ns__derived` value requires the `ns__derived` member to point
to the `ns__derived` value to serialize and the `ns__base` members are
irrelevant.

We can assign the base type value to the `ns_record::base` member:

~~~{.cpp}
    struct ns__record record;
    soap_default_ns__record(soap, &record);
    soap_write_ns__record(soap, &record);
~~~

This produces the following XML fragment populated with default values (empty
text for strings and zeros for numbers), where element <i>`base`</i> has two
child elements:

<div class="alt">
~~~{.xml}
    <ns:record>
      <base>
        <name></name>
        <number>0</number>
      </base>
    </ns:record>
~~~
</div>

We can also assign the derived type value to the `ns_record::base` member:

~~~{.cpp}
    struct ns__record record;
    soap_default_ns__record(soap, &record);
    record.base.ns__derived = soap_new_ns__derived(soap, -1);
    soap_write_ns__record(soap, &record);
~~~

This produces the following XML fragment populated with default values (empty
text for strings and zeros for numbers), where element <i>`base`</i> has schema
type <i>`ns:derived`</i> with three child elements and an attribute:

<div class="alt">
~~~{.xml}
    <ns:record>
      <base xsi:type="ns:derived" value="">
        <name></name>
        <number>0</number>
        <text></text>
      </base>
    </ns:record>
~~~
</div>

Deserialization automatically assigns values to the base members for the
`ns__base` type and populates the `ns__derived` member when a derived type with
<i>`xsi:type="ns:derived"`</i> is parsed.  This makes it easy to decompose the
deserialized data:

~~~{.cpp}
    struct ns__record record;
    soap_read_ns__record(soap, &record);
    if (record.ns__derived)
      printf("Derived type with name=%s number=%d text=%s value=%s\n",
          record.ns__derived->name,
          record.ns__derived->number,
          record.ns__derived->text,
          record.ns__derived->value);
    else
      printf("Base type with name=%s number=%d\n",
          record.name,
          record.number);
~~~

This example requires the `SOAP_XML_STRICT` mode flag to initialize the `soap`
context to ensure that all required values are present in the deserialized
structures, otherwise the `char*` strings may be NULL since XML validation
constraints are not enforced on the XML input.

Deeper levels of simulated inheritance are possible, for example:

~~~{.cpp}
    struct ns__base // a base type
    {
        char *name   1;
        int   number 1;
      [ struct ns__derived *ns__derived; ] // points to derived type if non-NULL
    };
    struct ns__derived // extends ns__base with two additional members
    {
        char  *name   1;
        int    number 1;
        char  *text   1;
      @ char *value   1;
      [ struct ns__derived_derived *ns__derived_derived; ] // points to derived_derived type if non-NULL
    };
    struct ns__derived_derived // extends ns__derived with an additional member
    {
        char  *name   1;
        int    number 1;
        char  *text   1;
      @ char *value   1;
      @ char *type    1;
    };
~~~

This requires two pointer traversals from the base type `ns__base` via
`ns__derived` to reach `ns__derived_derived`.

🔝 [Back to table of contents](#)

### Volatile classes and structs                                     {#toxsd9-2}

Classes and structs can be declared `volatile` in the interface header file for
soapcpp2, which only has meaning for the gSOAP tools.  This annotation means
that these types are already declared elsewhere in your project's source code
and you do not want soapcpp2 to generate code with a second declaration of
these types.

For example, `struct tm` is declared in the <i>`time.h`</i> library.  You can
make it serializable and include a partial list of data members that you want
to serialize:

~~~{.cpp}
    volatile struct tm
    {
        int tm_sec;  // seconds (0 - 60)
        int tm_min;  // minutes (0 - 59)
        int tm_hour; // hours (0 - 23)
        int tm_mday; // day of month (1 - 31)
        int tm_mon;  // month of year (0 - 11)
        int tm_year; // year - 1900
    };
~~~

You can declare classes and structs `volatile` for any such types you want to
serialize by only providing the public data members you want to serialize.

In addition, [colon notation](#toxsd2) is a simple and effective way to bind an
existing class or struct to a schema.  For example, you can change the `tm` name
as follows without affecting the code that uses `struct tm` generated by
soapcpp2:

~~~{.cpp}
    volatile struct ns:tm { ... }
~~~

This struct maps to a complexType in the soapcpp2-generated XML schema:

<div class="alt">
~~~{.xml}
    <complexType name="tm">
      <sequence>
        <element name="tm-sec" type="xsd:int" minOccurs="1" maxOccurs="1"/>
        <element name="tm-min" type="xsd:int" minOccurs="1" maxOccurs="1"/>
        <element name="tm-hour" type="xsd:int" minOccurs="1" maxOccurs="1"/>
        <element name="tm-mday" type="xsd:int" minOccurs="1" maxOccurs="1"/>
        <element name="tm-mon" type="xsd:int" minOccurs="1" maxOccurs="1"/>
        <element name="tm-year" type="xsd:int" minOccurs="1" maxOccurs="1"/>
      </sequence>
    </complexType>
~~~
</div>

🔝 [Back to table of contents](#)

### Mutable classes and structs                                      {#toxsd9-3}

Classes and structs can be declared `mutable` with the gSOAP tools.  This means
that their definition can be spread out over the source code.  This promotes the
concept of a class or struct as a *row of named values*, also known as a *named
tuple*, that can be extended at compile time in your source code with additional
members.  Because these types differ from the traditional object-oriented
principles and design concepts of classes and objects, constructors and
destructors cannot be defined (also because we cannot guarantee merging these
into one such that all members will be initialized).  A default constructor,
copy constructor, assignment operation, and destructor will be assigned
automatically by soapcpp2.

~~~{.cpp}
    mutable struct ns__tuple
    {
      @ std::string id;
    };

    mutable struct ns__tuple
    {
        std::string name;
        std::string value;
    };
~~~

The members are collected into one definition generated by soapcpp2.  Members
may be repeated from one definition to another, but only if their associated
types are identical.  So, for example, a third extension with a `value` member
with a different type fails:

~~~{.cpp}
    mutable struct ns__tuple
    {
        float value; // BAD: value is already declared std::string
    };
~~~

The `mutable` concept has proven to be very useful when declaring and
collecting SOAP Headers for multiple services, which are collected into one
`struct SOAP_ENV__Header` by the soapcpp2 tool.

🔝 [Back to table of contents](#)

### Default and fixed member values                                  {#toxsd9-4}

Class and struct data members in C and C++ may be declared with an optional
default initialization value that is provided "inline" with the declaration of
the member:

~~~{.cpp}
    class ns__record
    {
     public:
        std::string name = "Joe";
        ...
    };
~~~

Alternatively, you can use C++11 default initialization syntax:

~~~{.cpp}
    class ns__record
    {
     public:
        std::string name { "Joe" };
        ...
    };
~~~

Any member with a primitive type can be initialized in this way.

These initializations are performed by the default constructor that is added by
soapcpp2 to each class and struct (in C++ only).  A constructor is only added
when a default constructor is not already defined with the class declaration.

You can explicitly (re)initialize an object with these initial values by using
the soapcpp2 auto-generated functions:

- `void T::soap_default(struct soap*)` for `class T` (C++ only)

- `void soap_default_T(struct soap*, T*)` for `struct T` (C and C++).

If `T` is a struct or class that has a `soap` pointer member to a `::soap`
context then this pointer member will be set to the first argument passed to
these functions to initialize their `soap` pointer member.

Default value initializations can be provided for members that have primitive
types (`bool`, `enum`, `time_t`, numeric and string types).

Default value initializations of pointer members is permitted, but the effect
is different.  To conform to XML schema validation, an attribute member that is
a pointer to a primitive type will be assigned the default value when parsed
from XML.  An element member that is a pointer to a primitive type will be
assigned when the element is empty when parsed from XML.

As of gSOAP 2.8.48 and greater, a fixed value can be assigned with a `==`.  A
fixed value is also verified by the parser's validator.

Default and fixed values for members with or without pointers are best
explained with the following two example fragments.

A record class with default values for `std::string` (or `std::wstring`)
attributes and elements is declared as follows:

~~~{.cpp}
    class ns__record_with_default
    {
     public:
      @ std::string  a   = "A"; // optional XML attribute with default value "A"
      @ std::string  b 1 = "B"; // required XML attribute with default value "B"
      @ std::string *c   = "C"; // optional XML attribute with default value "C"
        std::string  d 0 = "D"; // optional XML element with default value "D"
        std::string  e   = "E"; // required XML element with default value "E"
        std::string *f   = "F"; // optional XML element with default value "F"
        ...
    };
~~~

Also `std::unique_ptr` and `std::shared_ptr` may be used instead of a regular
pointer to strings.

With C `char*` (or `const char*`, `const wchar_t*`) strings in a struct, this
becomes:

~~~{.cpp}
    struct ns__record_with_default
    {
      @ char* a   = "A"; // optional XML attribute with default value "A"
      @ char* b 1 = "B"; // required XML attribute with default value "B"
        char* e 1 = "E"; // required XML element with default value "E"
        char* f   = "F"; // optional XML element with default value "F"
        ...
    };
~~~

By contrast to `std::string e`, `char* e` must be marked `1` to make it
required, because pointer members are optional by default.

Attributes are considered optional by default, unless marked as required with
the occurrence constraint `1`.  Elements are considered required unless the
member type is a pointer or if the member is marked optional with occurrence
constraint `0`.

Instead of default values, fixed values indicate that the attribute or element
must contain that value, and only that value, when provided in XML.  A fixed
value is specified with a `==`.

Attributes with default or fixed values may be omitted in XML.  When absent,
the default/fixed value is used at the receiving side, i.e. the deserializer
assigns the default/fixed value when the attribute is absent.  Therefore, there
is no need to make attributes with default/fixed values pointer based, because
there is no way to distinguish an omitted attribute from a populated attribute
on the receiving side.  The `c` member in the example above can be a
non-pointer for this reason.  The wsdl2h tool does not generate pointers for
attributes with default/fixed values.

Elements with default or fixed values may be optional and the use of
default/fixed values with elements differs from attributes.  The default/fixed
value of an element is only used for elements that are empty in the XML payload
received.  Omitted optional elements in the XML payload received are simply
absent; no default/fixed value is assigned.

@note gSOAP 2.8.106 and greater treat `char*` and `wchar_t*` with explicit
default and fixed values differently than previous versions.  Versions prior to
2.8.106 assign the default value when the corresponding XML element is absent,
whereas 2.8.106 and greater assign NULL when the XML element is absent, exactly
as documented in this updated version of this document.  To revert to the old
behavior, use <b>`soapcpp2 -z4`</b> option <b>`-z4`</b>.  The change affects
members `char* f` and `char* l` (see below).

A record class (can be a struct in C) with fixed values for attributes and
elements is declared as follows:

~~~{.cpp}
    class ns__record_with_fixed
    {
     public:
      @ std::string  g   == "G"; // optional XML attribute with fixed value "G"
      @ std::string  h 1 == "H"; // required XML attribute with fixed value "H"
      @ std::string *i   == "I"; // optional XML attribute with fixed value "I"
        std::string  j 0 == "J"; // optional XML element with fixed value "J"
        std::string  k   == "K"; // required XML element with fixed value "K"
        std::string *l   == "L"; // optional XML element with fixed value "L"
        ...
    };
~~~

With C `char*` (or `const char*`, `const wchar_t*`) strings in a struct, this
becomes:

~~~{.cpp}
    struct ns__record_with_fixed
    {
      @ char* g   == "G"; // optional XML attribute with fixed value "G"
      @ char* h 1 == "H"; // required XML attribute with fixed value "H"
        char* k 1 == "K"; // required XML element with fixed value "K"
        char* l   == "L"; // optional XML element with fixed value "L"
        ...
    };
~~~

The XML schema validation rules for the examples above are as follows:

Member | Notes
------ | ---------------------------------------------------------------------
`a`    | attribute may appear once; if it does not appear its value is "A", otherwise its value is that given (also note: instantiating `ns__record_with_default` assigns the default value "A")
`b`    | has no effect when parsing XML (but note: instantiating `ns__record_with_default` assigns the default value "B")
`c`    | attribute may appear once; if it does not appear its value is "C", otherwise its value is that given (also note: instantiating `ns__record_with_default` assigns NULL)
`d`    | element may appear once; if it does not appear or if it is empty, its value is "D"; otherwise its value is that given (also note: instantiating `ns__record_with_default` assigns the default value "D")
`e`    | has no effect when parsing XML (but note: instantiating `ns__record_with_default` assigns the default value "E")
`f`    | element may appear once; if it does not appear it is not provided; if it does appear and it is empty, its value is "F"; otherwise its value is that given (also note: instantiating `ns__record_with_default` assigns NULL)
`g`    | attribute may appear once; if it does not appear its value is "G", if it does not appear its value is "G" (also note: instantiating `ns__record_with_fixed` assigns the fixed value "G")
`h`    | attribute must appear once, its value must be "H" (also note: instantiating `ns__record_with_fixed` assigns the fixed value "H")
`i`    | attribute may appear once; if it does not appear its value is "I", if it does not appear its value is "I" (also note: instantiating `ns__record_with_fixed` assigns NULL)
`j`    | element may appear once, if it does not appear it is not provided; if it does appear and it is empty, its value is "J"; if it does appear and it is not empty, its value must be "J" (also note: instantiating `ns__record_with_fixed` assigns the fixed value "J")
`k`    | element must appear once, its value must be "K" (also note: instantiating `ns__record_with_fixed` assigns the fixed value "K")
`l`    | element may appear once, if it does not appear it is not provided; if it does appear and it is empty, its value is "J"; if it does appear and it is not empty, its value must be "J" (also note: instantiating `ns__record_with_fixed` assigns NULL)

Members of type `char[N]` (fixed length string) can have default and fixed
values, when <b>`soapcpp2 -b`</b> option <b>`-b`</b> is used.  Also `char**`
(pointer to a string) members can have default and fixed values.  However,
members of this type will be initialized to NULL.  The default/fixed values
will be assigned with the same rules as for `char*` when deserialized from XML.

@see Section [operations on classes and structs](#toxsd9-14).

🔝 [Back to table of contents](#)

### Attribute members                                                {#toxsd9-5}

Class and struct data members are declared as XML attributes by annotating
their type with a `@` qualifier:

~~~{.cpp}
    class ns__record
    {
     public:
      @ std::string name;    // required (non-pointer means required)
      @ uint64_t    SSN;     // required (non-pointer means required)
        ns__record  *spouse; // optional (pointer means minOccurs=0)
    };
~~~

This class maps to a complexType in the soapcpp2-generated XML schema:

<div class="alt">
~~~{.xml}
    <complexType name="record">
      <sequence>
        <element name="spouse" type="ns:record" minOccurs="0" maxOccurs="1" nillable="true"/>
      </sequence>
      <attribute name="name" type="xsd:string" use="required"/>
      <attribute name="SSN" type="xsd:unsignedLong" use="required"/>
    </complexType>
~~~
</div>

An example XML instance of `ns__record` is:

<div class="alt">
~~~{.xml}
    <ns:record xmlns:ns="urn:types" name="Joe" SSN="1234567890">
      <spouse name="Jane" SSN="1987654320">
      </spouse>
    </ns:record>
~~~
</div>

Attribute data members are restricted to primitive types (`bool`, `enum`,
`time_t`, numeric and string types), `xsd__hexBinary`, `xsd__base64Binary`, and
custom serializers, such as `xsd__dateTime`.  Custom serializers for types that
may be used as attributes should define `soap_s2T` and `soap_T2s` functions that
convert values of type `T` to strings and back.

Attribute data members can be pointers and smart pointers to these types, which
permits attributes to be optional.

🔝 [Back to table of contents](#)

### Backtick XML tags                                                {#toxsd9-5-1}

The XML tag name of a class/struct member is the name of the member with the
usual XML tag translation, see [colon notation](#toxsd2).

To override the standard translation of identifier names to XML tag names of
attributes and elements, add the XML tag name in backticks (requires gSOAP
2.8.30 or greater):

~~~{.cpp}
    class ns__record
    {
     public:
      @ std::string name    `full-name`;
      @ uint64_t    SSN     `tax-id`;
        ns__record *spouse  `married-to`;
    };
~~~

This class maps to a complexType in the soapcpp2-generated XML schema:

<div class="alt">
~~~{.xml}
    <complexType name="record">
      <sequence>
        <element name="married-to" type="ns:record" minOccurs="0" maxOccurs="1"/>
      </sequence>
      <attribute name="full-name" type="xsd:string" use="required"/>
      <attribute name="tax-id" type="xsd:unsignedLong" use="required"/>
    </complexType>
~~~
</div>

An example XML instance of `ns__record` is:

<div class="alt">
~~~{.xml}
    <ns:record xmlns:ns="urn:types" full-name="Joe" tax-id="1234567890">
      <married-to full-name="Jane" tax-id="1987654320">
      </married-to>
    </ns:record>
~~~
</div>

A backtick XML tag name may contain any non-empty sequence of ASCII and UTF-8
characters except white space and the backtick character.  A backtick tag can
be combined with member constraints and default member initializers:

~~~{.cpp}
    @ uint64_t SSN `tax-id` 0:1 = 999;
~~~

🔝 [Back to table of contents](#)

### Qualified and unqualified members                                {#toxsd9-6}

Class, struct, and union data members are mapped to namespace qualified or
unqualified tag names of local elements and attributes.  If a data member has
no prefix then the default form of qualification is applied based on the
element/attribute form that is declared with the XML schema of the class, struct,
or union type.  If the member name has a namespace prefix by colon notation,
then the prefix overrules the default (un)qualified form.  Therefore,
[colon notation](#toxsd2) is an effective mechanism to control qualification of
tag names of individual members of classes, structs, and unions.

The XML schema elementFormDefault and attributeFormDefault declarations control
the tag name qualification of local elements and attributes, respectively.

- "unqualified" indicates that local elements/attributes are not qualified with
  the namespace prefix.

- "qualified" indicates that local elements/attributes must be qualified with
  the namespace prefix.

Individual schema declarations of local elements and attributes may overrule
this by using the form declaration in an XML schema and by using colon notation
to add namespace prefixes to class, struct, and union members in the header
file for soapcpp2.

Consider for example an `ns__record` class in the `ns` namespace in which local
elements are qualified and local attributes are unqualified by default:

~~~{.cpp}
    //gsoap ns schema namespace:     urn:types
    //gsoap ns schema elementForm:   qualified
    //gsoap ns schema attributeForm: unqualified
    class ns__record
    {
     public:
      @ std::string name;
      @ uint64_t    SSN;
        ns__record *spouse;
    };
~~~

This class maps to a complexType in the soapcpp2-generated XML schema with
targetNamespace "urn:types", elementFormDefault qualified and
attributeFormDefault unqualified:

<div class="alt">
~~~{.xml}
    <schema targetNamespace="urn:types"
      ...
      elementFormDefault="qualified"
      attributeFormDefault="unqualified"
      ...  >
      <complexType name="record">
        <sequence>
          <element name="spouse" type="ns:record" minOccurs="0" maxOccurs="1"/>
        </sequence>
        <attribute name="name" type="xsd:string" use="required"/>
        <attribute name="SSN" type="xsd:unsignedLong" use="required"/>
      </complexType>
    </schema>
~~~
</div>

An example XML instance of `ns__record` is:

<div class="alt">
~~~{.xml}
    <ns:record xmlns:ns="urn:types" name="Joe" SSN="1234567890">
      <ns:spouse> name="Jane" SSN="1987654320">
      </ns:spouse>
    </ns:record>
~~~
</div>

Here the root element <i>`<ns:record>`</i> is qualified because it is a root
element of the XML schema with target namespace "urn:types".  Its local element
<i>`<ns:spouse>`</i> is namespace qualified because the elementFormDefault of
local elements is qualified.  Attributes are unqualified.

The default namespace (un)qualification of local elements and attributes can be
overruled by adding a prefix to the member name by using colon notation:

~~~{.cpp}
    //gsoap ns schema namespace:     urn:types
    //gsoap ns schema elementForm:   qualified
    //gsoap ns schema attributeForm: unqualified
    class ns__record
    {
     public:
      @ std::string ns:name; // 'ns:' qualified
      @ uint64_t    SSN;
        ns__record *:spouse; // ':' unqualified (empty prefix)
    };
~~~

The colon notation for member <i>`ns:name`</i> forces qualification of its attribute
tag in XML.  The colon notation for member <i>`:spouse`</i> removes qualification from
its local element tag:

<div class="alt">
~~~{.xml}
    <schema targetNamespace="urn:types"
      ...
      elementFormDefault="unqualified"
      attributeFormDefault="unqualified"
      ... >
      <complexType name="record">
        <sequence>
          <element name="spouse" type="ns:record" minOccurs="0" maxOccurs="1" form="unqualified"/>
        </sequence>
        <attribute name="name" type="xsd:string" use="required" form="qualified"/>
        <attribute name="SSN" type="xsd:unsignedLong" use="required"/>
      </complexType>
    </schema>
~~~
</div>

XML instances of `ns__record` have unqualified spouse elements and qualified
ns:name attributes:

<div class="alt">
~~~{.xml}
    <ns:record xmlns:ns="urn:types" ns:name="Joe" SSN="1234567890">
      <spouse> ns:name="Jane" SSN="1987654320">
      </spouse>
    </ns:record>
~~~
</div>

Members of a class or struct can also be prefixed using the `prefix__name`
convention or using colon notation `prefix:name`.  However, this has a
different effect by referring to global (root) elements and attributes, see
[document root element definitions](#toxsd9-7).

[Backtick XML tags](#toxsd9-5-1) can be used in place of the member name
annotations and will achieve the same effect as described when these tag names
are (un)qualified (requires gSOAP 2.8.30 or greater).

@note You must declare a target namespace with a `//gsoap ns schema namespace:`
directive to enable the `elementForm` and `attributeForm` directives in order
to generate valid XML schemas with soapcpp2.  See [directives](#directives) for
more details.

🔝 [Back to table of contents](#)

### Defining document root elements                                 {#toxsd9-7}

To define and reference XML document root elements we use type names that start
with an underscore:

~~~{.cpp}
    class _ns__record
~~~

Alternatively, we can use a `typedef` to define a document root element with a
given type:

~~~{.cpp}
    typedef ns__record _ns__record;
~~~

This `typedef` maps to a global root element that is added to the
soapcpp2-generated XML schema:

<div class="alt">
~~~{.xml}
    <element name="record" type="ns:record"/>
~~~
</div>

An example XML instance of `_ns__record` is:

<div class="alt">
~~~{.xml}
    <ns:record xmlns:ns="urn:types">
      <name>Joe</name>
      <SSN>1234567890</SSN>
      <spouse>
        <name>Jane</name>
        <SSN>1987654320</SSN>
      </spouse>
    </ns:record>
~~~
</div>

Global-level element/attribute definitions are also referenced and/or added to
the generated XML schema when serializable data members reference these by
their qualified name:

~~~{.cpp}
    typedef std::string _ns__name 1 : 100;
    class _ns__record
    {
     public:
      @ _QName       xsi__type; // built-in XSD attribute xsi:type
        _ns__name    ns__name;  // ref to global ns:name element
        uint64_t     SSN;
        _ns__record *spouse;
    };
~~~

These types map to the following comonents in the soapcpp2-generated XML
schema:

<div class="alt">
~~~{.xml}
    <simpleType name="name">
      <restriction base="xsd:string">
        <minLength value="1"/>
        <maxLength value="100"/>
      </restriction>
    </simpleType>
    <element name="name" type="ns:name"/>
    <complexType name="record">
      <sequence>
        <element ref="ns:name" minOccurs="1" maxOccurs="1"/>
        <element name="SSN" type="xsd:unsignedLong" minOccurs="1" maxOccurs="1"/>
        <element name="spouse" type="ns:record" minOccurs="0" maxOccurs="1"/>
      </sequence>
      <attribute ref="xsi:type" use="optional"/>
    </complexType>
    <element name="record" type="ns:record"/>
~~~
</div>

Use only use qualified member names when their types match the global-level
element types that they refer to.  For example:

~~~{.cpp}
    typedef std::string _ns__name; // global element ns:name of type xsd:string
    class _ns__record
    {
     public:
        int         ns__name;   // BAD: global element ns:name is NOT type int
        _ns__record ns__record; // OK: ns:record is a global-level root element
        ...
    };
~~~

Therefore, we recommend to use qualified member names only when necessary to
refer to standard XSD elements and attributes, such as `xsi__type`, and
`xsd__lang`.

By contrast, colon notation has the desired effect to (un)qualify local tag
names by overruling the default element/attribute namespace qualification, see
[qualified and unqualified members](#toxsd9-6).

As an alternative to prefixing member names, use the backtick tag (requires
gSOAP 2.8.30 or greater):

~~~{.cpp}
    typedef std::string _ns__name 1 : 100;
    class _ns__record
    {
     public:
      @ _QName       t    <i>`xsi:type`</i>; // built-in XSD attribute xsi:type
        _ns__name    s    <i>`ns:name`</i>;  // ref to global ns:name element
        uint64_t     SSN;
        _ns__record *spouse;
    };
~~~

🔝 [Back to table of contents](#)

### (Smart) pointer members and their occurrence constraints         {#toxsd9-8}

A public pointer-typed data member is serialized by following its (smart)
pointer(s) to the value pointed to.  To serialize pointers to dynamic arrays of
data, please see the next section on
[container and array members and their occurrence constraints](#toxsd9-9).

Pointers that are NULL and smart pointers that are empty are serialized to
produce omitted element and attribute values, unless an element is required
and is nillable (struct/class members marked with `nullptr`) in which case the
element is rendered as an empty element with <i>`xsi:nil="true"`</i>.

To control the occurrence requirements of pointer-based data members,
occurrence constraints are associated with data members in the form of a range
`minOccurs : maxOccurs`.  For non-repeatable (meaning, not a container or array)
data members, there are only three reasonable occurrence constraints:

- `0:0` means that this element or attribute is prohibited.

- `0:1` means that this element or attribute is optional.

- `1:1` means that this element or attribute is required.

Pointer-based data members have a default `0:1` occurrence constraint, making
them optional, and their XML schema local element/attribute definition is
marked as nillable.  Non-pointer data members have a default `1:1` occurence
constraint, making them required.

A `nullptr` occurrence constraint may be applicable to required elements that
are nillable pointer types, thus `nullptr 1:1`.  This indicates that the
element is nillable (can be `NULL` or `nullptr`).  A pointer data member that
is explicitly marked as required and nillable with `nullptr 1:1` will be
serialized as an element with an <i>`xsi:nil`</i> attribute, thus effectively
revealing the NULL property of its value.

A non-pointer data member that is explicitly marked as optional with `0:1` will
be set to its default value when no XML value is presented to the deserializer.
A default value can be assigned to a data member that has a primitive type or
is a (smart) pointer to primitive type.

Consider for example:

~~~{.cpp}
    class ns__record
    {
     public:
        std::shared_ptr<std::string>  name;               // optional (pointer means minOccurs=0)
        uint64_t                      SSN    0:1 = 999;   // force optional with default 999
        ns__record                   *spouse nullptr 1:1; // force required and nillabe when absent
    };
~~~

This class maps to a complexType in the soapcpp2-generated XML schema:

<div class="alt">
~~~{.xml}
    <complexType name="record">
      <sequence>
        <element name="name" type="xsd:string" minOccurs="0" maxOccurs="1" nillable="true"/>
        <element name="SSN" type="xsd:unsignedLong" minOccurs="0" maxOccurs="1" default="999"/>
        <element name="spouse" type="ns:record" minOccurs="1" maxOccurs="1" nillable="true"/>
      </sequence>
    </complexType>
~~~
</div>

An example XML instance of `ns__record` with its `name` string value set to
`Joe`, `SSN` set to its default, and `spouse` set to NULL:

<div class="alt">
~~~{.xml}
    <ns:record xmlns:ns="urn:types" ...>
      <name>Joe</name>
      <SSN>999</SSN>
      <spouse xsi:nil="true"/>
    </ns:record>
~~~
</div>

@note In general, a smart pointer is simply declared as a `volatile` template
in a interface header file for soapcpp2:
~~~{.cpp}
    volatile template <class T> class NAMESPACE::shared_ptr;
~~~

@note The soapcpp2 tool generates code that uses `NAMESPACE::shared_ptr` and
`NAMESPACE::make_shared` to create shared pointers to objects, where
`NAMESPACE` is any valid C++ namespace such as `std` and `boost` if you have
Boost installed.

🔝 [Back to table of contents](#)

### Container and array members and their occurrence constraints     {#toxsd9-9}

Class and struct data member types that are containers `std::deque`,
`std::list`, `std::vector` and `std::set` are serialized as a collection of
the values they contain.  You can also serialize dynamic arrays, which is the
alternative for C to store collections of data.  Let's start with STL containers.

You can use `std::deque`, `std::list`, `std::vector`, and `std::set` containers
by importing:

~~~{.cpp}
    #import "import/stl.h"       // import all containers
    #import "import/stldeque.h"  // import deque
    #import "import/stllist.h"   // import list
    #import "import/stlvector.h" // import vector
    #import "import/stlset.h"    // import set
~~~

For example, to use a vector data mamber to store names in a record:

~~~{.cpp}
    #import "import/stlvector.h"
    class ns__record
    {
     public:
        std::vector<std::string>  names;
        uint64_t                  SSN;
    };
~~~

To limit the number of names in the vector within reasonable bounds, occurrence
constraints are associated with the container.  Occurrence constraints are of
the form `minOccurs : maxOccurs`:

~~~{.cpp}
    #import "import/stlvector.h"
    class ns__record
    {
     public:
        std::vector<std::string>  names 1:10;
        uint64_t                  SSN;
    };
~~~

This class maps to a complexType in the soapcpp2-generated XML schema:

<div class="alt">
~~~{.xml}
    <complexType name="record">
      <sequence>
        <element name="name" type="xsd:string" minOccurs="1" maxOccurs="10"/>
        <element name="SSN" type="xsd:unsignedLong" minOccurs="1" maxOccurs="1"/>
      </sequence>
    </complexType>
~~~
</div>

@note In general, a container is simply declared as a template in an interface
header file for soapcpp2.  All class templates are considered containers
(except when declared `volatile`, see smart pointers).  For example,
`std::vector` is declared in <i>`gsoap/import/stlvector.h`</i> as:
~~~{.cpp}
    template <class T> class std::vector;
~~~

@note You can define and use your own containers.  The soapcpp2 tool generates
code that uses the following members of the `template <typename T> class C`
container:
~~~{.cpp}
    void              C::clear()
    C::iterator       C::begin()
    C::const_iterator C::begin() const
    C::iterator       C::end()
    C::const_iterator C::end() const
    size_t            C::size() const
    C::iterator       C::insert(C::iterator pos, const T& val)
~~~

@note For more details see the example `simple_vector` container with
documentation in the package under <i>`gsoap/samples/template`</i>.

Because C does not support a container template library, we can use a
dynamically-sized array of values.  This array is declared as a size-pointer
pair of members within a struct or class.  The array size information is stored
in a special size tag member with the name `__size` or `__sizeX`, where `X` can
be any name, or by an `$int` member to identify the member as a special size
tag:

~~~{.cpp}
    struct ns__record
    {
      $ int      sizeofnames; // array size
        char*    *names;       // array of char* names
        uint64_t  SSN;
    };
~~~

This struct maps to a complexType in the soapcpp2-generated XML schema:

<div class="alt">
~~~{.xml}
    <complexType name="record">
      <sequence>
        <element name="name" type="xsd:string" minOccurs="0" maxOccurs="unbounded" nillable="true"/>
        <element name="SSN" type="xsd:unsignedLong" minOccurs="1" maxOccurs="1"/>
      </sequence>
    </complexType>
~~~
</div>

To limit the number of names in the array within reasonable bounds, occurrence
constraints are associated with the array size member.  Occurrence constraints
are of the form `minOccurs : maxOccurs`:

~~~{.cpp}
    struct ns__record
    {
      $ int      sizeofnames 1:10; // array size 1..10
        char*   *names;            // array of one to ten char* names
        uint64_t SSN;
    };
~~~

This struct maps to a complexType in the soapcpp2-generated XML schema:

<div class="alt">
~~~{.xml}
    <complexType name="record">
      <sequence>
        <element name="name" type="xsd:string" minOccurs="1" maxOccurs="10" nillable="true"/>
        <element name="SSN" type="xsd:unsignedLong" minOccurs="1" maxOccurs="1"/>
      </sequence>
    </complexType>
~~~
</div>

Arrays can also be declared as nested elements, similar to SOAP-encoded dynamic arrays, and these arrays can be used with or without SOAP applications.  This requires a separate struct or class with the name of the SOAP array, which should not be qualified with a namespace prefix:

~~~{.cpp}
    struct ArrayOfstring
    {
        char* *__ptr 1:100; // array of 1..100 strings
        int    __size;      // array size
    };
    struct ns__record
    {
        struct ArrayOfstring names; // array of char* names
        uint64_t             SSN;
    };
~~~

The `ns__record` struct maps to a complexType that references the <i>`ArrayOfstring`</i> complexType with an sequence of 1 to 100 <i>`item`</i> elements:

<div class="alt">
~~~{.xml}
    <complexType name="ArrayOfstring">
      <sequence>
        <element name="item" type="xsd:string" minOccurs="1" maxOccurs="100"/>
      </sequence>
    </complexType>
    <complexType name="record">
      <sequence>
        <element name="names" type="ns:ArrayOfstring" minOccurs="1" maxOccurs="1"/>
        <element name="SSN" type="xsd:unsignedLong" minOccurs="1" maxOccurs="1"/>
      </sequence>
    </complexType>
~~~
</div>

To change the <i>`item`</i> element name in the WSDL, XML schema, and XML messages, use `__ptrName` where `Name` is the name you want to use.

@note When <b>`soapcpp2 -e`</b> option <b>`-e`</b> is used, the <i>`ArrayOfstring`</i> becomes a SOAP-encoded array for SOAP 1.1/1.2 RPC encoded messaging:
<div class="alt">
~~~{.xml}
    <import namespace="http://schemas.xmlsoap.org/soap/encoding/"/>
    <complexType name="ArrayOfstring">
      <complexContent>
        <restriction base="SOAP-ENC:Array">
          <sequence>
            <element name="item" type="xsd:string" minOccurs="1" maxOccurs="100"/>
          </sequence>
          <attribute ref="SOAP-ENC:arrayType" WSDL:arrayType="xsd:string[]"/>
        </restriction>
      </complexContent>
    </complexType>
    <complexType name="record">
      <sequence>
        <element name="names" type="ns:ArrayOfstring" minOccurs="1" maxOccurs="1"/>
        <element name="SSN" type="xsd:unsignedLong" minOccurs="1" maxOccurs="1"/>
      </sequence>
    </complexType>
~~~
</div>

Fixed-size arrays can be used to store a fixed number of values:

~~~{.cpp}
    struct ns__record
    {
        char*    names[10]; // array of 10 char* names
        uint64_t SSN;
    };
~~~

The fixed-size array is similar to a SOAP-encoded array, which can be used with or without SOAP applications.  This struct maps to a complexType that references a <i>`Array10Ofstring`</i> complexType with ten <i>`item`</i> elements:

<div class="alt">
~~~{.xml}
    <complexType name="record">
      <sequence>
        <element name="names" type="ns:Array10Ofstring" minOccurs="1" maxOccurs="1"/>
        <element name="SSN" type="xsd:unsignedLong" minOccurs="1" maxOccurs="1"/>
      </sequence>
    </complexType>
    <complexType name="Array10Ofstring">
      <sequence>
        <element name="item" type="xsd:string" minOccurs="0" maxOccurs="10"/>
      </sequence>
    </complexType>
~~~
</div>

@note When <b>`soapcpp2 -e`</b> option <b>`-e`</b> is used, the <i>`Array10Ofstring`</i> becomes a SOAP-encoded array for SOAP 1.1/1.2 RPC encoded messaging, see previous note.

🔝 [Back to table of contents](#)

### Sequencing with hidden members                                  {#toxsd9-10}

A member becomes a hidden XML element, i.e. not visibly rendered in XML, when
its name starts with a double underscore.  This makes it possible to sequence a
collection of data members, basically by forming a sequence of elements that
can be optional or repeated together.

To create a sequence of members that are optional, use a pointer-based hidden
member that is a struct with the collection of members to sequence:

~~~{.cpp}
    struct ns__record
    {
        std::string   name;       // required name
        struct __ns__optional
        {
            uint64_t    SSN;      // SSN in optional group
            std::string phone;    // phone number in optional group
        } *__optional;            // optional group
    };
~~~

Here we used a hidden struct type `__ns__optional` which starts with a double
underscore, because we do not want to define a new global type for the XML
schema we generate.  We just need a unique name for a structure that sequences
the two members.

This struct maps to a complexType in the soapcpp2-generated XML schema:

<div class="alt">
~~~{.xml}
    <complexType name="record">
      <sequence>
        <element name="name" type="xsd:string" minOccurs="1" maxOccurs="1"/>
        <sequence minOccurs="0" maxOccurs="1">
          <element name="SSN" type="xsd:unsignedLong" minOccurs="1" maxOccurs="1"/>
          <element name="phone" type="xsd:string" minOccurs="1" maxOccurs="1"/>
        </sequence>
      </sequence>
    </complexType>
~~~
</div>

The `name` member is a required element of the <i>`ns:record`</i> complexType.
The <i>`ns:record`</i> complexType has an optional sequence of `SSN` and
`phone` elements.

To create repetitions of a sequence of members, use an array as follows:

~~~{.cpp}
    struct ns__record
    {
        std::string   name;        // required name
      $ int           sizeofarray; // size of group array
        struct __ns__array
        {
            uint64_t    SSN;       // SSN in group
            std::string phone;     // phone number in group
        } *__array;                // group array
    };
~~~

This struct maps to a complexType in the soapcpp2-generated XML schema:

<div class="alt">
~~~{.xml}
    <complexType name="record">
      <sequence>
        <element name="name" type="xsd:string" minOccurs="1" maxOccurs="1"/>
        <sequence minOccurs="0" maxOccurs="unbounded">
          <element name="SSN" type="xsd:unsignedLong" minOccurs="1" maxOccurs="1"/>
          <element name="phone" type="xsd:string" minOccurs="1" maxOccurs="1"/>
        </sequence>
      </sequence>
    </complexType>
~~~
</div>

The `name` member is a required element of the <i>`ns:record`</i> complexType.
The <i>`ns:record`</i> complexType has a potentially unbounded sequence of
`SSN` and `phone` elements.  You can specify array bounds instead of zero to
unbounded, see [container and array members and their occurrence constraints](#toxsd9-9).

The XML value space consists of a sequence of SSN and phone elements:

<div class="alt">
~~~{.xml}
    <ns:record>
      <name>numbers</name>
      <SSN>1234567890</SSN>
      <phone>555-123-4567</phone>
      <SSN>1987654320</SSN>
      <phone>555-789-1234</phone>
      <SSN>2345678901</SSN>
      <phone>555-987-6543</phone>
    </ns:record>
~~~
</div>

🔝 [Back to table of contents](#)

### Tagged union members                                            {#toxsd9-11}

A union member in a class or in a struct cannot be serialized unless a
discriminating *variant selector* member is provided that tells the serializer
which union field to serialize.  This effectively creates a *tagged union*.

The variant selector is associated with the union as a selector-union pair of members.
The variant selector is a member with the name `__union` or `__unionX`, where
`X` can be any name, or by an `$int` member to identify the member as a variant
selector tag:

~~~{.cpp}
    class ns__record
    {
     public:
      $ int  xORnORs;    // variant selector with values SOAP_UNION_fieldname
        union ns__choice
        {
            float x;
            int   n;
            char *s;
        } u;
        std::string name;
    };
~~~

The variant selector values are auto-generated based on the union name `choice`
and the names of its members `x`, `n`, and `s`:

- `xORnORs = SOAP_UNION_ns__choice_x` when `u.x` is valid.

- `xORnORs = SOAP_UNION_ns__choice_n` when `u.n` is valid.

- `xORnORs = SOAP_UNION_ns__choice_s` when `u.s` is valid.

- `xORnORs = 0` when none are valid (should only be used with great care,
  because XSD validation may fail when content is required but absent).

This class maps to a complexType with a sequence and choice in the
soapcpp2-generated XML schema:

<div class="alt">
~~~{.xml}
    <complexType name="record">
      <sequence>
        <choice>
          <element name="x" type="xsd:float" minOccurs="1" maxOccurs="1"/>
          <element name="n" type="xsd:int" minOccurs="1" maxOccurs="1"/>
          <element name="s" type="xsd:string" minOccurs="0" maxOccurs="1" nillable="true"/>
        </choice>
        <element name="names" type="xsd:string" minOccurs="1" maxOccurs="1" nillable="true"/>
      </sequence>
    </complexType>
~~~
</div>

An STL container or dynamic array of a union requires wrapping the variant
selector and union member in a struct:

~~~{.cpp}
    class ns__record
    {
     public:
        std::vector<
        struct ns__data  // data with a choice of x, n, or s
        {
          $ int  xORnORs; // variant selector with values SOAP_UNION_fieldname
            union ns__choice
            {
                float x;
                int   n;
                char *s;
            } u;
        }> data;         // vector with data
    };
~~~

and an equivalent definition with a dynamic array instead of a `std::vector`
(you can use this in C with structs):

~~~{.cpp}
    class ns__record
    {
     public:
      $ int  sizeOfdata; // size of dynamic array
        struct ns__data   // data with a choice of x, n, or s
        {
          $ int  xORnORs;  // variant selector with values SOAP_UNION_fieldname
            union ns__choice
            {
                float x;
                int   n;
                char *s;
            } u;
        } *data;          // points to the data array of length sizeOfdata
    };
~~~

This maps to two complexTypes in the soapcpp2-generated XML schema:

<div class="alt">
~~~{.xml}
    <complexType name="data">
      <choice>
        <element name="x" type="xsd:float" minOccurs="1" maxOccurs="1"/>
        <element name="n" type="xsd:int" minOccurs="1" maxOccurs="1"/>
        <element name="s" type="xsd:string" minOccurs="0" maxOccurs="1" nillable="true"/>
      </choice>
    </complexType>
    <complexType name="record">
      <sequence>
        <element name="data" type="ns:data" minOccurs="0" maxOccurs="unbounded"/>
      </sequence>
    </complexType>
~~~
</div>

The XML value space consists of a sequence of item elements each wrapped in an
data element:

<div class="alt">
~~~{.xml}
    <ns:record xmlns:ns="urn:types" ...>
      <data>
        <n>123</n>
      </data>
      <data>
        <x>3.1</x>
      </data>
      <data>
        <s>hello</s>
      </data>
      <data>
        <s>world</s>
      </data>
    </ns:record>
~~~
</div>

To remove the wrapping data element, simply rename the wrapping struct to
`__ns__data` and the member to `__data` to make this member invisible to the
serializer.  The double underscore prefix naming convention is used for the
struct name and member name.  Also use a dynamic array instead of a STL
container (so you can also use this approach in C with structs):

~~~{.cpp}
    class ns__record
    {
     public:
      $ int sizeOfdata;   // size of dynamic array
        struct __ns__data // contains choice of x, n, or s
        {
          $ int  xORnORs; // variant selector with values SOAP_UNION_fieldname
            union ns__choice
            {
                float x;
                int   n;
                char *s;
            } u;
        } *__data;        // points to the data array of length sizeOfdata
    };
~~~

This maps to a complexType in the soapcpp2-generated XML schema:

<div class="alt">
~~~{.xml}
    <complexType name="record">
      <sequence minOccurs="0" maxOccurs="unbounded">
        <choice>
          <element name="x" type="xsd:float" minOccurs="1" maxOccurs="1"/>
          <element name="n" type="xsd:int" minOccurs="1" maxOccurs="1"/>
          <element name="s" type="xsd:string" minOccurs="0" maxOccurs="1" nillable="true"/>
        </choice>
      </sequence>
    </complexType>
~~~
</div>

The XML value space consists of a sequence of <i>`<x>`</i>, <i>`<n>`</i>, and/or <i>`<s>`</i>
elements:

<div class="alt">
~~~{.xml}
    <ns:record xmlns:ns="urn:types" ...>
      <n>123</n>
      <x>3.1</x>
      <s>hello</s>
      <s>world</s>
    </ns:record>
~~~
</div>

Please note that structs, classes, and unions are unnested by soapcpp2 (as in
the C standard of nested structs and unions).  Therefore, the `ns__choice`
union in the `ns__record` class is redeclared at the top level despite its
nesting within the `ns__record` class.  This means that you will have to choose
a unique name for each nested struct, class, and union.

🔝 [Back to table of contents](#)

### Tagged void pointer members                                     {#toxsd9-12}

To serialize data pointed to by `void*` requires run-time type information that
tells the serializer what type of data to serialize by means of a *tagged void
pointer*.  This type information is stored in a special type tag member of a
struct/class with the name `__type` or `__typeX`, where `X` can be any name, or
alternatively by an `$int` special member of any name as a type tag:

~~~{.cpp}
    class ns__record
    {
     public:
      $ int  typeOfdata; // type tag with values SOAP_TYPE_T
        void *data;      // points to some data of type T
    };
~~~

A type tag member has nonzero values `SOAP_TYPE_T` where `T` is the name of a
struct/class or the name of a primitive type, such as `int`, `std__string` (for
`std::string`), `string` (for `char*`).

This class maps to a complexType with a sequence in the soapcpp2-generated
XML schema:

<div class="alt">
~~~{.xml}
    <complexType name="record">
      <sequence>
        <element name="data" type="xsd:anyType" minOccurs="0" maxOccurs="1"/>
      </sequence>
    </complexType>
~~~
</div>

The XML value space consists of the XML value space of the type with the
addition of an <i>`xsi:type`</i> attribute to the enveloping element:

<div class="alt">
~~~{.xml}
    <ns:record xmlns:ns="urn:types" ...>
      <data xsi:type="xsd:int">123</data>
    </ns:record>
~~~
</div>

This <i>`xsi:type`</i> attribute is important for the receiving end to distinguish
the type of data to instantiate.  The receiver cannot deserialize the data
without an <i>`xsd:type`</i> attribute.

You can find the `SOAP_TYPE_T` name of each serializable type in the
auto-generated <i>`soapStub.h`</i> file.

Also all serializable C++ classes have a virtual `int T::soap_type()` member
that returns their `SOAP_TYPE_T` value that you can use.

When the `void*` pointer is NULL or when `typeOfdata` is zero, the data is not
serialized.

An STL container or dynamic array of `void*` pointers to <i>`xsd:anyType`</i> data
requires wrapping the type tag and `void*` members in a struct:

~~~{.cpp}
    class ns__record
    {
     public:
        std::vector<
        struct ns__data      // data with an xsd:anyType item
        {
          $ int  typeOfitem; // type tag with values SOAP_TYPE_T
            void *item;      // points to some item of type T
        }> data;             // vector with data
    };
~~~

and an equivalent definition with a dynamic array instead of a `std::vector`
(you can use this in C with structs):

~~~{.cpp}
    class ns__record
    {
     public:
      $ int sizeOfdata;      // size of dynamic array
        struct ns__data      // data with an xsd:anyType item
        {
          $ int  typeOfitem; // type tag with values SOAP_TYPE_T
            void *item;      // points to some item of type T
        } *data;             // points to the data array of length sizeOfdata
    };
~~~

This maps to two complexTypes in the soapcpp2-generated XML schema:

<div class="alt">
~~~{.xml}
    <complexType name="data">
      <sequence>
        <element name="item" type="xsd:anyType" minOccurs="1" maxOccurs="1" nillable="true"/>
      </sequence>
    </complexType>
    <complexType name="record">
      <sequence>
        <element name="data" type="ns:data" minOccurs="0" maxOccurs="unbounded"/>
      </sequence>
    </complexType>
~~~
</div>

The XML value space consists of a sequence of item elements each wrapped in a
data element:

<div class="alt">
~~~{.xml}
    <ns:record xmlns:ns="urn:types" ...>
      <data>
        <item xsi:type="xsd:int">123</item>
      </data>
      <data>
        <item xsi:type="xsd:double">3.1</item>
      </data>
      <data>
        <item xsi:type="xsd:string">abc</item>
      </data>
    </ns:record>
~~~
</div>

To remove the wrapping data elements, simply rename the wrapping struct and
member to `__data` to make this member invisible to the serializer with the
double underscore prefix naming convention.  Also use a dynamic array instead
of a STL container (you can use this in C with structs):

~~~{.cpp}
    class ns__record
    {
     public:
      $ int sizeOfdata;      // size of dynamic array
        struct __data        // contains xsd:anyType item
        {
          $ int typeOfitem;  // type tag with values SOAP_TYPE_T
            void *item;      // points to some item of type T
        } *__data;           // points to the data array of length sizeOfdata
    };
~~~

This maps to a complexType in the soapcpp2-generated XML schema:

<div class="alt">
~~~{.xml}
    <complexType name="record">
      <sequence minOccurs="0" maxOccurs="unbounded">
        <element name="item" type="xsd:anyType" minOccurs="1" maxOccurs="1"/>
      </sequence>
    </complexType>
~~~
</div>

The XML value space consists of a sequence of data elements:

<div class="alt">
~~~{.xml}
    <ns:record xmlns:ns="urn:types" ...>
      <item xsi:type="xsd:int">123</item>
      <item xsi:type="xsd:double">3.1</item>
      <item xsi:type="xsd:string">abc</item>
    </ns:record>
~~~
</div>

Again, please note that structs, classes, and unions are unnested by soapcpp2
(as in the C standard of nested structs and unions).  Therefore, the `__data`
struct in the `ns__record` class is redeclared at the top level despite its
nesting within the `ns__record` class.  This means that you will have to choose
a unique name for each nested struct, class, and union.

@see Section [XSD type bindings](#typemap2).

🔝 [Back to table of contents](#)

### Adding get and set methods                                      {#toxsd9-13}

A public `get` method may be added to a class or struct, which will be
triggered by the deserializer.  This method will be invoked right after the
instance is populated by the deserializer.  The `get` method can be used to
update or verify deserialized content.  It should return `SOAP_OK` or set
`soap::error` to a nonzero error code and return it.

A public `set` method may be added to a class or struct, which will be
triggered by the serializer.  The method will be invoked just before the
instance is serialized.  Likewise, the `set` method should return `SOAP_OK` or
set set `soap::error` to a nonzero error code and return it.

For example, adding a `set` and `get` method to a class declaration:

~~~{.cpp}
    class ns__record
    {
     public:
        int set(struct soap*); // triggered before serialization
        int get(struct soap*); // triggered after deserialization
        ...
    };
~~~

To add these and othe rmethods to classes and structs with wsdl2h and
<i>`typemap.dat`</i>, please see [class/struct member additions](#typemap3).

🔝 [Back to table of contents](#)

### Operations on classes and structs                               {#toxsd9-14}

The following functions/macros are generated by soapcpp2 for each type `T`,
which should make it easier to send, receive, and copy XML data in C and in
C++:

- `int soap_write_T(struct soap*, T*)` writes an instance of `T` to a file via
   file descriptor `int soap::sendfd)` or to a stream via `std::ostream
   *soap::os` (C++ only) or saves into a NUL-terminated string by setting
   `const char **soap::os` to a string pointer to be set (C only).  Returns
   `SOAP_OK` on success or an error code, also stored in `soap::error`.

- `int soap_read_T(struct soap*, T*)` reads an instance of `T` from a file via
   file descriptor `int soap::recvfd)` or from a stream via `std::istream
   *soap::is` (C++ only) or reads from a NUL-termianted string `const char
   *soap::is` (C only).  Returns `SOAP_OK` on success or an error code, also
   stored in `soap::error`.

- `void soap_default_T(struct soap*, T*)` sets an instance `T` to its default
  value, resetting members of a struct to their initial values (for classes we
  use method `T::soap_default`, see below).  If `T` is a struct that has a
  `soap` pointer member to a `::soap` context then this pointer member will be
  set to the first argument passed to this function to initialize its `soap`
  pointer member.

- `T * soap_dup_T(struct soap*, T *dst, const T *src)` (requires <b>`soapcpp2 -Ec`</b>)
  deep copy `src` into `dst`, replicating all deep cycles and shared pointers
  when a managing `soap` context is provided as argument.  When `dst` is NULL,
  allocates space for `dst` and returns a pointer to the allocated copy.  Deep
  copy results in a tree when the `soap` context is NULL, but the presence of
  deep cycles will lead to non-termination.  Use flag `SOAP_XML_TREE` with
  managing context to copy into a tree without cycles and pointers to shared
  objects.  Returns `dst` or allocated copy when `dst` is NULL.

- `void soap_del_T(const T*)` (requires <b>`soapcpp2 -Ed`</b>) deletes all
  heap-allocated members of this object by deep deletion ONLY IF this object
  and all of its (deep) members are not managed by a `soap` context AND the deep
  structure is a tree (no cycles and co-referenced objects by way of multiple
  (non-smart) pointers pointing to the same data).  Can be safely used after
  `T * soap_dup_T(NULL, NULL, const T*)` to delete the deep copy returned.
  Does not delete the object itself.

When in C++ mode, soapcpp2 tool adds several methods to classes in addition to
adding a default constructor and destructor (when these were not explicitly
declared).

The public methods added to a class `T`:

- `virtual int T::soap_type(void)` returns a unique type ID (`SOAP_TYPE_T`).
  This numeric ID can be used to distinguish base from derived instances.

- `virtual void T::soap_default(struct soap*)` sets all data members to
  default values.  If class `T` has a `soap` pointer member to a `::soap`
  context then this pointer member will be set to the argument passed to this
  function to initialize its `soap` pointer member.

- `virtual void T::soap_serialize(struct soap*) const` serializes object to
  prepare for SOAP 1.1/1.2 encoded output (or with `SOAP_XML_GRAPH`) by
  analyzing its (cyclic) structures.

- `virtual int T::soap_put(struct soap*, const char *tag, const char *type) const`
  emits object in XML, compliant with SOAP 1.1 encoding style, return error
  code or `SOAP_OK`.  Requires `soap_begin_send(soap)` and
  `soap_end_send(soap)`.

- `virtual int T::soap_out(struct soap*, const char *tag, int id, const char *type) const`
  emits object in XML, with tag and optional id attribute and <i>`xsi:type`</i>,
  return error code or `SOAP_OK`.  Requires `soap_begin_send(soap)` and
  `soap_end_send(soap)`.

- `virtual void * T::soap_get(struct soap*, const char *tag, const char *type)`
  Get object from XML, compliant with SOAP 1.1 encoding style, return pointer
  to object or NULL on error.  Requires `soap_begin_recv(soap)` and
  `soap_end_recv(soap)`.

- `virtual void *soap_in(struct soap*, const char *tag, const char *type)`
  Get object from XML, with matching tag and type (NULL matches any tag and
  type), return pointer to object or NULL on error.  Requires
  `soap_begin_recv(soap)` and `soap_end_recv(soap)`

- `virtual T * T::soap_alloc(void) const` returns a new object of type `T`,
  default initialized and not managed by a `soap` context.

- `virtual T * T::soap_dup(struct soap*) const` (requires <b>`soapcpp2 -Ec`</b>)
  returns a duplicate of this object by deep copying, replicating all deep
  cycles and shared pointers when a managing `soap` context is provided as
  argument.  Deep copy is a tree when argument is NULL, but the presence of
  deep cycles will lead to non-termination.  Use flag `SOAP_XML_TREE` with the
  managing context to copy into a tree without cycles and pointers to shared
  objects.

- `virtual void T::soap_del() const` (rquires <b>`soapcpp2 -Ed`</b>) deletes all
  heap-allocated members of this object by deep deletion ONLY IF this object
  and all of its (deep) members are not managed by a `soap` context AND the deep
  structure is a tree (no cycles and co-referenced objects by way of multiple
  (non-smart) pointers pointing to the same data).  Can be safely used after
  `soap_dup(NULL)` to delete the deep copy.  Does not delete the object itself.

Also, there are four variations of `soap_new_T` for
class/struct/template type `T` that soapcpp2 auto-generates to create instances
on a context-managed heap:

- `T * soap_new_T(struct soap*)` returns a new instance of `T` with default data
  member initializations that are set with the soapcpp2 auto-generated `void
  T::soap_default(struct soap*)` method), but ONLY IF the soapcpp2
  auto-generated default constructor is used that invokes `soap_default()` and
  was not replaced by a user-defined default constructor.

- `T * soap_new_T(struct soap*, int n)` returns an array of `n` new instances of
  `T`.  Similar to the above, instances are initialized.

- `T * soap_new_req_T(struct soap*, ...)` returns a new instance of `T` and sets
  the required data members to the values specified in `...`.  The required data
  members are those with nonzero minOccurs, see the subsections on
  [(smart) pointer members and their occurrence constraints](#toxsd9-8) and
  [container and array members and their occurrence constraints](#toxsd9-9).

- `T * soap_new_set_T(struct soap*, ...)` returns a new instance of `T` and sets
  the public/serializable data members to the values specified in `...`.

The above functions can be invoked with a NULL `soap` context, but we will be
responsible to use `delete T` to remove this instance from the unmanaged heap.

The allocation functions return NULL when memory allocation failed.

🔝 [Back to table of contents](#)

Special classes and structs                                           {#toxsd10}
---------------------------

The following applies to both structs and classes.  The examples show classes
in C++.  For C, use structs and omit the C++ features.  Structs also require
the use of the `struct` keyword, otherwise soapcpp2 will throw a syntax error.

### SOAP-encoded arrays                                             {#toxsd10-1}

A class or struct with the following layout is a one-dimensional SOAP-encoded
array type:

~~~{.cpp}
    class ArrayOfT
    {
     public:
        T   *__ptr;  // array pointer
        int  __size; // array size
    };
~~~

where `T` is the array element type.  A multidimensional SOAP Array is:

~~~{.cpp}
    class ArrayOfT
    {
     public:
        T   *__ptr;     // array pointer
        int  __size[N]; // array size of each dimension
    };
~~~

where `N` is the constant number of dimensions.  The pointer points to an array
of `__size[0]*__size[1]* ... * __size[N-1]` elements.

This maps to a complexType restriction of SOAP-ENC:Array in the
soapcpp2-generated XML schema:

<div class="alt">
~~~{.xml}
    <complexType name="ArrayOfT">
      <complexContent>
        <restriction base="SOAP-ENC:Array">
          <sequence>
            <element name="item" type="T" minOccurs="0" maxOccurs="unbounded" nillable="true"/>
          </sequence>
          <attribute ref="SOAP-ENC:arrayType" WSDL:arrayType="ArrayOfT[]"/>
        </restriction>
      </complexContent>
    </complexType>
~~~
</div>

The name of the class can be arbitrary.  We often use `ArrayOfT` without a
prefix to distinguish arrays from other classes and structs.

With SOAP 1.1 encoding, an optional offset member can be added that controls
the start of the index range for each dimension:

~~~{.cpp}
    class ArrayOfT
    {
     public:
        T   *__ptr;       // array pointer
        int  __size[N];   // array size of each dimension
        int  __offset[N]; // array offsets to start each dimension
    };
~~~

For example, we can define a matrix of floats as follows:

~~~{.cpp}
    class Matrix
    {
     public:
        double *__ptr;
        int     __size[2];
    };
~~~

The following code populates the matrix and serializes it in XML:

~~~{.cpp}
    soap *soap = soap_new1(SOAP_XML_INDENT);
    Matrix A;
    double a[6] = { 1, 2, 3, 4, 5, 6 };
    A.__ptr = a;
    A.__size[0] = 2;
    A.__size[1] = 3;
    soap_write_Matrix(soap, &A);
~~~

Matrix A is serialized as an array with 2x3 values:

<div class="alt">
~~~{.xml}
    <SOAP-ENC:Array SOAP-ENC:arrayType="xsd:double[2,3]" ...>
      <item>1</item>
      <item>2</item>
      <item>3</item>
      <item>4</item>
      <item>5</item>
      <item>6</item>
    </SOAP-ENC:Array>
~~~
</div>

🔝 [Back to table of contents](#)

### XSD hexBinary and base64Binary types                            {#toxsd10-2}

A special case of a one-dimensional array is used to define <i>`xsd:hexBinary`</i> and
<i>`xsd:base64Binary`</i> types when the pointer type is `unsigned char`:

~~~{.cpp}
    class xsd__hexBinary
    {
     public:
        unsigned char *__ptr;  // points to raw binary data
        int            __size; // size of data
    };
~~~

and

~~~{.cpp}
    class xsd__base64Binary
    {
     public:
        unsigned char *__ptr;  // points to raw binary data
        int            __size; // size of data
    };
~~~

To create a new binary type, use either one of the following three forms that
declare a new `ns__binary` type that is a <i>`simpleType`</i> restriction of
<i>`xsd:base64Binary`</i>:

~~~{.cpp}
    typedef xsd__base64Binary ns__binary;
~~~
~~~{.cpp}
    class ns__binary : public xsd__base64Binary
    {
        ... // attribute members (@) and class methods
    };
~~~
~~~{.cpp}
    class ns__binary
    {
     public:
        unsigned char *__ptr;  // points to raw binary data
        int            __size; // size of data
        ...                    // attribute members (@) and class methods (optional)
    };
~~~

Here, `xsd__base64Binary` is reused in the first two cases, where
`xsd__base64Binary` is declared as shown above.

@see [DIME/MIME/MTOM attachment binary types](#toxsd10-3)

🔝 [Back to table of contents](#)

### DIME/MIME/MTOM attachment binary types                          {#toxsd10-3}

A class or struct with a binary content layout can be extended to support
attachments.  The following struct or class type can be used as DIME, MIME, and
MTOM attachment and also be used for <i>`xsd:base64Binary`</i> type values:

~~~{.cpp}
    class xsd__base64Binary
    {
     public:
        unsigned char *__ptr;   // points to raw binary data
        int            __size;  // size of data
        char          *id;      // NULL to generate an id, or set to a unique UUID
        char          *type;    // MIME type of the data
        char          *options; // optional description of MIME attachment
    };
~~~

When the `id`, `type`, or `options` members are non-NULL, an attachment will be
used instead of base64 XML content.  DIME attachments are the default.  To
switch to MIME use the `SOAP_ENC_MIME` context flag.  To switch to MTOM use the
`SOAP_ENC_MTOM` context flag.

MTOM is typically used with XOP <i>`<xop:Include>`</i> elements, which is
preferred and declared as follows:

~~~{.cpp}
    //gsoap xop schema import: http://www.w3.org/2004/08/xop/include
    class _xop__Include
    {
     public:
        unsigned char *__ptr;   // points to raw binary data
        int            __size;  // size of data
        char          *id;      // NULL to generate an id, or set to a unique UUID
        char          *type;    // MIME type of the data
        char          *options; // optional description of MIME attachment
    };
~~~

Attachments are beyond the scope of this article.  See the
[gSOAP user guide.](../../guide/html/index.html) for more details.

🔝 [Back to table of contents](#)

### Wrapper class/struct with simpleContent                         {#toxsd10-4}

A class or struct with the following layout is a complexType that wraps
simpleContent:

~~~{.cpp}
    class ns__simple
    {
     public:
        T   __item; // primitive type for the simpleContent
        ...         // attribute members (@) and class methods (optional)
    };
~~~

The type `T` is a primitive type (`bool`, `enum`, `time_t`, numeric and string
types), `xsd__hexBinary`, `xsd__base64Binary`, and custom serializers, such as
`xsd__dateTime`.

This maps to a complexType with simpleContent in the soapcpp2-generated XML
schema:

<div class="alt">
~~~{.xml}
    <complexType name="simple">
      <simpleContent>
        <extension base="T"/>
      </simpleContent>
    </complexType> 
~~~
</div>

A wrapper class/struct may include any number of members that are declared as
attributes with `@`, which should be placed after the `__item` member.

🔝 [Back to table of contents](#)

### DOM anyType and anyAttribute                                    {#toxsd10-5}

Use of a DOM is optional and enabled by `#import "dom.h"` to use the DOM
`xsd__anyType` element node and `xsd__anyAttribute` attribute node:

~~~{.cpp}
    #import "dom.h"

    class ns__record
    {
     public:
      @ xsd__anyAttribute attributes;  // optional DOM attributes
        xsd__anyType      *name;       // optional DOM element (pointer means minOccurs=0)
        xsd__anyType       address;    // required DOM element (minOccurs=1)
        xsd__anyType       email   0;  // optional DOM element (minOccurs=0)
        ...                            // other members
    };
~~~

where `name` contains XML stored in a DOM node set and `attributes` is a list
of all visibly rendered attributes.  The name `attributes` is arbitrary and any
name will suffice.

You should place the `xsd__anyType` members at the end of the struct or class.
This ensures that the DOM members are populated last as a "catch all".  A
member name starting with double underscore is a wildcard member.  These
members are placed at the end of a struct or class automatically by soapcpp2.

An `#import "dom.h"` import is automatically added by <b>`wsdl2h -d`</b> with
option <b>`-d`</b> to bind <i>`xsd:anyType`</i> to DOM nodes, and also to
populate <i>`xsd:any`</i>, <i>`xsd:anyAttribute`</i> and <i>`xsd:mixed`</i> XML
content:

~~~{.cpp}
    #import "dom.h"

    class ns__record
    {
     public:
      @ xsd__anyAttribute        __anyAttribute;  // optional DOM attributes
        std::vector<xsd__anyType> __any   0;      // optional DOM elements (minOccurs=0)
        xsd__anyType              __mixed 0;      // optional mixed content (minOccurs=0)
        ...                                       // other members
    };
~~~

where the members prefixed with `__` are "invisible" to the XML parser, meaning
that these members are not bound to XML tag names.

In C you can use a dynamic arrary instead of `std::vector`:

~~~{.cpp}
    #import "dom.h"

    struct ns__record
    {
      @ xsd__anyAttribute        __anyAttribute; // optional DOM attributes
      $ int                      __sizeOfany;    // size of the array
        xsd__anyType             *__any;         // optional DOM elements (pointer means minOccurs=0)
        xsd__anyType              __mixed 0;     // optional mixed content (minOccurs=0)
        ...                                      // other members
    };
~~~

Classes can inherit DOM, which enables full use of polymorphism with one base
DOM class:

~~~{.cpp}
    #import "dom.h"

    class ns__record : public xsd__anyType
    {
     public:
        std::vector<xsd__anyType*> array; // array of objects of any class
        ...                               // other members
    };
~~~

This permits an `xsd__anyType` pointer to refer to a derived class such as
`ns__record`, which will be serialized with an <i>`xsi:type`</i> attribute that is
set to "ns:record".  The <i>`xsi:type`</i> attributes add the necessary type information
to distinguish the XML content from the DOM base type.  This is important for
the receiving end: without <i>`xsd:type`</i> attributes with type names, only base DOM
objects are recognized and instantiated.

Because C lacks object-oriented programming concepts such as class inheritance
and polymorphism, you should consider using [derived types in C and C++](#toxsd9-1-1).

An alternative is to use the special [tagged void pointer members](#toxsd9-12)
to serialize data pointed to by a `void*` member, which can be any serializable
type, such as derived types.  This approach uses <i>`xsi:type`</i> attributes
to identify the type of value serialized.

To ensure that wsdl2h generates pointer-based `xsd__anyType` DOM nodes with
<b>`wsdl2h -d`</b> using option <b>`-d`</b> for <i>`xsd:any`</i>, add the
following line to <i>`typemap.dat`</i>:

    xsd__any = | xsd__anyType*

This lets wsdl2h produce class/struct members and containers with
`xsd__anyType*` for <i>`xsd:any`</i> instead of `xsd__anyType`.  To just force all
<i>`xsd:anyType`</i> uses to be pointer-based, declare in <i>`typemap.dat`</i>:

    xsd__anyType = | xsd__anyType*

If you use <b>`wsdl2h -d -p`</b> using options <b>`-d`</b> and <b>`-p`</b> then
every class will inherit DOM as shown above.  Without option `-d`, an
`xsd__anyType` type is generated to serve as the root type in the type
hierarchy:

~~~{.cpp}
    class xsd__anyType { _XML __item; struct soap *soap; };

    class ns__record : public xsd__anyType
    {
        ...
    };
~~~

where the `_XML __item` member holds any XML content as a literal XML string.

To use the DOM API, compile <i>`dom.c`</i> (or <i>`dom.cpp`</i> for C++), or
link the gSOAP library with <b>`-lgsoapssl`</b> (or <b>`-lgsoapssl++`</b> for C++).

@see Documentation of [XML DOM and XPath](http://www.genivia.com/doc/dom/html)
for more details.

🔝 [Back to table of contents](#)

Directives                                                         {#directives}
==========

You can use `//gsoap` directives in the interface header file with the data
binding interface for soapcpp2.  These directives are used to configure the
code generated by soapcpp2 by declaring various.  properties of Web services
and XML schemas.  When using the wsdl2h tool, you will notice that wsdl2h
generates directives automatically based on the WSDL and XSD input.

Service directives are applicable to service and operations described by WSDL.
Schema directives are applicable to types, elements, and attributes defined by
XML schemas.

🔝 [Back to table of contents](#)

Service directives                                               {#directives-1}
------------------

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
<b>`soapcpp2 -e`</b> option <b>`-e`</b> to `rpc` and `encoded`, respectively.

The `protocol` property is `SOAP` by default (SOAP 1.1).  Protocol property
values are:

protocol value | description
-------------- | -----------
`SOAP`         | SOAP transport, supporting both SOAP 1.1 and 1.2
`SOAP1.1`      | SOAP 1.1 transport (same as `soapcpp2 -1`)
`SOAP1.2`      | SOAP 1.2 transport (same as `soapcpp2 -2`)
`SOAP-GET`     | one-way SOAP 1.1 or 1.2 with HTTP GET
`SOAP1.1-GET`  | one-way SOAP 1.1 with HTTP GET
`SOAP1.2-GET`  | one-way SOAP 1.2 with HTTP GET
`HTTP`         | non-SOAP REST protocol with HTTP POST
`POST`         | non-SOAP REST protocol with HTTP POST
`GET`          | non-SOAP REST protocol with HTTP GET
`PUT`          | non-SOAP REST protocol with HTTP PUT
`DELETE`       | non-SOAP REST protocol with HTTP DELETE

You can bind service operations to the WSDL namespace of a service by using the
namespace prefix as part of the identifier name of the function that defines
the service operation:

~~~{.cpp}
    int prefix__func(arg1, arg2, ..., argn, result);
~~~

You can override the `port` endpoint URL at runtime in the auto-generated
`soap_call_prefix__func` service call (C/C++ client side) and in the C++ proxy
class service call.

🔝 [Back to table of contents](#)

Service method directives                                        {#directives-2}
-------------------------

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
`method`                    | an alias for the `method-documentation` property
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
`method-protocol`           | SOAP or REST, see [service directives](#directives-1)

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
   [attributes members](#toxsd9-5).  

Use `method-input-mime-type` and `method-output-mime-type` to differentiate the
attachment types between request and response messages.

🔝 [Back to table of contents](#)

Schema directives                                                {#directives-3}
-----------------

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

To learn more about the local form defaults, see [qualified and unqualified members.](#toxsd9-6)

The `namespace2` URI is a pattern with `*` matching any sequence of characters
and `-` matching any character.  This pattern instructs the XML parser and validator
to also accept the URI pattern as a valid namespace for the specified `<prefix>`.

The `typed` property is implicitly `yes` when <b>`soapcpp2 -t`</b> option <b>`-t`</b> is used.

🔝 [Back to table of contents](#)

Schema type directives                                           {#directives-4}
----------------------

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

Serialization rules                                                     {#rules}
===================

A presentation on XML data bindings is not complete without discussing the
serialization rules and options that put your data in XML on the wire or store
it a file or buffer.

There are several options to choose from to serialize data in XML.  The choice
depends on the use of the SOAP protocol or if SOAP is not required.  The wsdl2h
tool automates this for you by taking the WSDL transport bindings into account
when generating the service functions in C and C++ that use SOAP or REST.

The gSOAP tools are not limited to SOAP.  The tools implement generic XML data
bindings for SOAP, REST, and other uses of XML.  So you can read and write XML
using the serializing [operations on classes and structs](#toxsd9-14).

The following sections briefly explain the serialization rules with respect to
the SOAP protocol for XML Web services.  A basic understanding of the SOAP
protocol is useful when developing client and server applications that must
interoperate with other SOAP applications.

SOAP/REST Web service client and service operations are represented as
functions in your interface header file with the data binding interface for
soapcpp2.  The soapcpp2 tool will translate these function to client-side
service invocation calls and server-side service operation dispatchers.

A discussion of SOAP clients and servers is beyond the scope of this article.
However, the SOAP options discussed here also apply to SOAP client and server
development.

🔝 [Back to table of contents](#)

SOAP document versus rpc style                                        {#doc-rpc}
------------------------------

The `wsdl:binding/soap:binding/@style` attribute in the <i>`<wsdl:binding>`</i>
section of a WSDL is either "document" or "rpc".  The "rpc" style refers to
SOAP RPC (Remote Procedure Call), which is more restrictive than the "document"
style by requiring one XML element in the SOAP Body to act as the procedure
name with XML subelements as its parameters.

For example, the following directives in the interface header file for soapcpp2
declare that `DBupdate` is a SOAP RPC encoding service method:

~~~{.cpp}
    //gsoap ns service namespace:       urn:DB
    //gsoap ns service method-protocol: DBupdate SOAP
    //gsoap ns service method-style:    DBupdate rpc
    int ns__DBupdate(...);
~~~

The XML payload has a SOAP envelope, optional SOAP header, and a SOAP body with
one element representing the operation with the parameters as subelements:

<div class="alt">
~~~{.xml}
    <SOAP-ENV:Envelope
      xmlns:SOAP-ENV="http://schemas.xmlsoap.org/soap/envelope/"
      xmlns:SOAP-ENC="http://schemas.xmlsoap.org/soap/encoding/"
      xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
      xmlns:xsd="http://www.w3.org/2001/XMLSchema"
      xmlsn:ns="urn:DB">
      <SOAP-ENV:Body>
        <ns:DBupdate>
          ...
        </ns:DBupdate>
      </SOAP-ENV:Body>
    </SOAP-ENV:Envelope>
~~~
</div>

The "document" style puts no restrictions on the SOAP Body content.  However, we
recommend that the first element's tag name in the SOAP Body should be unique
to each type of operation, so that the receiver can dispatch the operation
based on this element's tag name.  Alternatively, the HTTP URL path can be used
to specify the operation, or the HTTP action header can be used to dispatch
operations automatically on the server side (soapcpp2 options -a and -A).

🔝 [Back to table of contents](#)

SOAP literal versus encoding                                          {#lit-enc}
----------------------------

The `wsdl:operation/soap:body/@use` attribute in the <i>`<wsdl:binding>`</i> section
of a WSDL is either "literal" or "encoded".  The "encoded" use refers to the
SOAP encoding rules that support id-ref multi-referenced elements to serialize
data as graphs.

SOAP encoding is very useful if the data internally forms a graph (including
cycles) and we want the graph to be serialized in XML in a format that ensures
that its structure is preserved.  In that case, SOAP 1.2 encoding is the best
option.

SOAP encoding also adds encoding rules for [SOAP arrays](#toxsd10) to serialize
multi-dimensional arrays.  The use of XML attributes to exchange XML data in
SOAP encoding is not permitted.  The only attributes permitted are the standard
XSD attributes, SOAP encoding attributes (such as for arrays), and id-ref.

For example, the following directives in the interface header file for soapcpp2
declare that `DBupdate` is a SOAP RPC encoding service method:

~~~{.cpp}
    //gsoap ns service namespace:       urn:DB
    //gsoap ns service method-protocol: DBupdate SOAP
    //gsoap ns service method-style:    DBupdate rpc
    //gsoap ns service method-encoding: DBupdate encoded
    int ns__DBupdate(...);
~~~

The XML payload has a SOAP envelope, optional SOAP header, and a SOAP body with
an encodingStyle attribute for SOAP 1.1 encoding and an element representing the
operation with parameters that are SOAP 1.1 encoded:

<div class="alt">
~~~{.xml}
    <SOAP-ENV:Envelope
      xmlns:SOAP-ENV="http://schemas.xmlsoap.org/soap/envelope/"
      xmlns:SOAP-ENC="http://schemas.xmlsoap.org/soap/encoding/"
      xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
      xmlns:xsd="http://www.w3.org/2001/XMLSchema"
      xmlsn:ns="urn:DB">
      <SOAP-ENV:Body SOAP-ENV:encodingStyle="http://schemas.xmlsoap.org/soap/encoding/">
        <ns:DBupdate>
          <records SOAP-ENC:arrayType="ns:record[3]">
            <item>
              <name href="#_1"/>
              <SSN>1234567890</SSN>
            </item>
            <item>
              <name>Jane</name>
              <SSN>1987654320</SSN>
            </item>
            <item>
              <name href="#_1"/>
              <SSN>2345678901</SSN>
            </item>
          </records>
        </ns:DBupdate>
        <id id="_1" xsi:type="xsd:string">Joe</id>
      </SOAP-ENV:Body>
    </SOAP-ENV:Envelope>
~~~
</div>

In the XML fragment shown above the name "Joe" is shared by two records and the
string is referenced by SOAP 1.1 href and id attributes.

While the soapcpp-generated serializers only introduce multi-referenced
elements in the payload when they are actually multi-referenced in the data
graph, other SOAP applications may render multi-referenced elements more
aggressively.  The example could also be rendered as:

<div class="alt">
~~~{.xml}
    <SOAP-ENV:Envelope
      xmlns:SOAP-ENV="http://schemas.xmlsoap.org/soap/envelope/"
      xmlns:SOAP-ENC="http://schemas.xmlsoap.org/soap/encoding/"
      xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
      xmlns:xsd="http://www.w3.org/2001/XMLSchema"
      xmlsn:ns="urn:DB">
      <SOAP-ENV:Body SOAP-ENV:encodingStyle="http://schemas.xmlsoap.org/soap/encoding/">
        <ns:DBupdate>
          <records SOAP-ENC:arrayType="ns:record[3]">
            <item href="#id1"/>
            <item href="#id2"/>
            <item href="#id3"/>
          </records>
        </ns:DBupdate>
        <id id="id1" xsi:type="ns:record">
          <name href="#id4"/>
          <SSN>1234567890</SSN>
        </id>
        <id id="id2" xsi:type="ns:record">
          <name href="#id5"/>
          <SSN>1987654320</SSN>
        </id>
        <id id="id3" xsi:type="ns:record">
          <name href="#id4"/>
          <SSN>2345678901</SSN>
        </id>
        <id id="id4" xsi:type="xsd:string">Joe</id>
        <id id="id5" xsi:type="xsd:string">Jane</id>
      </SOAP-ENV:Body>
    </SOAP-ENV:Envelope>
~~~
</div>

SOAP 1.2 encoding is cleaner and produces more accurate XML encodings of data
graphs by setting the id attribute on the element that is referenced:

<div class="alt">
~~~{.xml}
    <SOAP-ENV:Envelope
      xmlns:SOAP-ENV="http://www.w3.org/2003/05/soap-envelope"
      xmlns:SOAP-ENC="http://www.w3.org/2003/05/soap-encoding"
      xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
      xmlns:xsd="http://www.w3.org/2001/XMLSchema"
      xmlsn:ns="urn:DB">
      <SOAP-ENV:Body>
        <ns:DBupdate SOAP-ENV:encodingStyle="http://www.w3.org/2003/05/soap-encoding">
          <records SOAP-ENC:itemType="ns:record" SOAP-ENC:arraySize="3">
            <item>
              <name SOAP-ENC:id="_1">Joe</name>
              <SSN>1234567890</SSN>
            </item>
            <item>
              <name>Jane</name>
              <SSN>1987654320</SSN>
            </item>
            <item>
              <name SOAP-ENC:ref="_1"/>
              <SSN>2345678901</SSN>
            </item>
          </records>
        </ns:DBupdate>
      </SOAP-ENV:Body>
    </SOAP-ENV:Envelope>
~~~
</div>

@note Some SOAP 1.2 applications consider the namespace `SOAP-ENC` of
<i>`SOAP-ENC:id`</i> and <i>`SOAP-ENC:ref`</i> optional.  With gSOAP, the SOAP
1.2 encoding serialization follows the 2007 standard, while accepting
unqualified id and ref attributes.

To remove all rendered id-ref multi-referenced elements, use the
`SOAP_XML_TREE` flag to initialize the `soap` context.

Some XSD validation rules are turned off with SOAP encoding, because of the
presence of additional attributes, such as id and ref/href, SOAP arrays with
arbitrary element tags for array elements, and the occurrence of additional
multi-ref elements in the SOAP 1.1 Body.

The use of "literal" puts no restrictions on the XML in the SOAP Body.  Full
XSD validation is possible, which can be enabled with the `SOAP_XML_STRICT`
flag to initialize the `soap` context.  However, data graphs will be
serialized as trees and cycles in the data will be cut from the XML rendition.

🔝 [Back to table of contents](#)

SOAP 1.1 versus SOAP 1.2                                                 {#soap}
------------------------

There are two SOAP protocol versions: 1.1 and 1.2. The gSOAP tools can switch
between the two versions seamlessly.  You can declare the default SOAP version
for a service operation as follows:

~~~{.cpp}
    //gsoap ns service method-protocol: DBupdate SOAP1.2
~~~

Use `SOAP` (SOAP 1.1), `SOAP1.1`, `SOAP1.2`, and `HTTP` to switch SOAP versions
or enable REST methods with HTTP POST.  See [service directives](#directives-1)
and [XML serialization](#non-soap).

The soapcpp2 tool auto-generates client and server code.  At the client side,
this operation sends data with SOAP 1.2 but accepts responses also in SOAP 1.1.
At the server side, this operation accepts requests in SOAP 1.1 and 1.2 and
will return responses in the same SOAP version.

As we discussed in the previous section, the SOAP 1.2 protocol has a cleaner
multi-referenced element serialization format that greatly enhances the
accuracy of data graph serialization with SOAP RPC encoding and is therefore
recommended.

The SOAP 1.2 protocol default can also be set by importing and loading
<i>`gsoap/import/soap12.h`</i>:

~~~{.cpp}
   #import "soap12.h"
~~~

Finally, the soapcpp2 tool has options to force SOAP 1.1, SOAP 1.2, or remove
SOAP altogether with <b>`soapcpp2 -1`</b> (SOAP 1.1), <b>`soapcpp2 -2`</b>
(SOAP 1.2) and <b>`soapcpp2 -0`</b> (plain XML, no SOAP).

🔝 [Back to table of contents](#)

XML serialization                                           {#non-soap}
-----------------

You can serialize data to XML that is stored on the heap, on the stack (locals), and
static data as long as the serializable (i.e. non-transient) values are
properly initialized and pointers in the data structures are either NULL or
point to valid structures.

When XML is deserialized into data, the data is put on the heap and managed by the
`::soap` context, see also [memory management](#memory).

You can read and write XML directly to a file or stream with the serializing
[operations on classes and structs](#toxsd9-14).

To define and use XML Web service client and service operations, we can declare
these operations in your interface header file with the data binding interface
for soapcpp2 as functions.  The function are translated by soapcpp2 to
client-side service invocation calls and server-side service operation
dispatchers.

The REST operations POST, GET, and PUT are declared with `//gsoap` directives
in the interface header file for soapcpp2.  For example, a REST HTTP POST
operation is declared as follows:

~~~{.cpp}
    //gsoap ns service namespace:       urn:DB
    //gsoap ns service method-protocol: DBupdate POST
    int ns__DBupdate(...);
~~~

There are no SOAP Envelope and SOAP Body elements in the payload for
`DBupdate`.  Also the XML serialization rules are identical to SOAP
document/literal, meaning no SOAP RPC encoding XML structures are implicitly
used.  The XML payload only has the operation name as an element with its
parameters serialized as subelements:

<div class="alt">
~~~{.xml}
    <ns:DBupdate xmln:ns="urn:DB" ...>
     ...
    </ns:DBupdate>
~~~
</div>

To force id-ref serialization with REST similar to SOAP 1.2 multi-reference
encoding, use the `SOAP_XML_GRAPH` flag to initialize the `soap` context.
The XML serialization includes id and ref attributes for multi-referenced
elements as follows:

<div class="alt">
~~~{.xml}
    <ns:DBupdate xmln:ns="urn:DB" ...>
      <records>
        <item>
          <name id="_1">Joe</name>
          <SSN>1234567890</SSN>
        </item>
        <item>
          <name>Jane</name>
          <SSN>1987654320</SSN>
        </item>
        <item>
          <name ref="_1"/>
          <SSN>2345678901</SSN>
        </item>
      </records>
    </ns:DBupdate>
~~~
</div>

🔝 [Back to table of contents](#)

Input and output                                                           {#io}
================

Reading and writing XML from/to files, streams and string buffers is done via
the managing `soap` context by setting one of the following context variables that
control IO sources and sinks:

~~~{.cpp}
    soap->recvfd = fd; // an int file descriptor to read from (0 by default)
    soap->sendfd = fd; // an int file descriptor to write to (1 by default)
    soap->is = &is;    // C++ only: a std::istream is object to read from
    soap->os = &os;    // C++ only: a std::ostream os object to write to
    soap->is = cs;     // C only: a const char* string to read from (soap->is will advance)
    soap->os = &cs;    // C only: pointer to a const char*, will be set to point to the string output
~~~

Normally, all of these context variables are NULL, which is required to send and
receive data over sockets by gSOAP client and server applications.  Therefore,
if you set any of these context variables in a client or server application
then you should reset them to NULL to ensure that socket communications are not
blocked.

@note The use of `soap::is` and `soap::os` in C requires gSOAP 2.8.28 or greater.

In the following sections, we present more details on how to read and write to
files and streams, and use string buffers as sources and sinks for XML data.

In addition, you can set IO callback functions to handle IO at a lower level.
For more details on defining your own callback functions, see the
[gSOAP user guide.](../../guide/html/index.html)

🔝 [Back to table of contents](#)

Reading and writing from/to files and streams                             {#io1}
---------------------------------------------

The default IO is standard input and output.  Other sources and sinks (those
listed above) will be used until you (re)set them.  For example with file-based
input and output:

~~~{.cpp}
    FILE *fp = fopen("record.xml", "r");
    if (fp != NULL)
    {
      soap->recvfd = fileno(fp);    // get file descriptor of file to read from
      if (soap_read_ns__record(soap, &pers1))
        ... // handle IO error
      fclose(fp);
      soap->recvfd = 0;             // read from stdin, or -1 to block reading
    }

    FILE *fp = fopen("record.xml", "w");
    if (fp != NULL)
    {
      soap->sendfd = fileno(fp);    // get file descriptor of file to write to
      if (soap_write_ns__record(soap, &pers1))
        ... // handle IO error
      fclose(fp);
      soap->sendfd = 1;             // write to stdout, or -1 to block writing
    }
~~~

Similar code with streams in C++:

~~~{.cpp}
    #include <fstream>

    std::fstream fs;
    fs.open("record.xml", std::ios::in);
    if (fs)
    {
      soap->is = &fs;
      if (soap_read__ns__record(soap, &pers1))
        ... // handle IO error
      fs.close();
      soap->is = NULL;
    }

    fs.open("record.xml", std::ios::out);
    if (fs)
    {
      soap->os = &fs;
      if (soap_write__ns__record(soap, &pers1))
        ... // handle IO error
      fs.close();
      soap->os = NULL;
    }
~~~

🔝 [Back to table of contents](#)

Reading and writing from/to string buffers                                {#io2}
------------------------------------------

For C++ we recommend to use `std::stringstream` objects from the
<i>`sstream`</i> C++ library as illustrated in the following example:

~~~{.cpp}
    #include <sstream>

    std::stringstream ss;
    ss.str("..."); // XML to parse
    soap->is = &ss;
    if (soap_read__ns__record(soap, &pers1))
      ... // handle IO error
    soap->is = NULL;

    soap->os = &ss;
    if (soap_write__ns__record(soap, &pers1))
      ... // handle IO error
    soap->os = NULL;
    std::string s = ss.str(); // string with XML
~~~

For C we can use `soap::is` and `soap::os` to point to strings of XML content
as follows (this requires gSOAP 2.8.28 or later):

~~~{.cpp}
    soap->is = "..."; // XML to parse
    if (soap_read__ns__record(soap, &pers1))
      ... // handle IO error
    soap->is = NULL;

    const char *cs = NULL;
    soap->os = &cs;
    if (soap_write__ns__record(soap, &pers1))
      ... // handle IO error
    soap->os = NULL;
    ... = cs; // string with XML (do not free(cs): managed by the context and freed with soap_end())
~~~

The type of `soap::os` is a pointer to a `const char*` string.  The pointer is
set by the managing `soap` context to point to the XML data that is stored on
the context-managed heap.

For earlier gSOAP versions we recommend to use IO callbacks `soap::frecv` and
`soap::fsend`, see the [gSOAP user guide.](../../guide/html/index.html).

🔝 [Back to table of contents](#)

Memory management                                                      {#memory}
=================

Memory management with the `soap` context enables us to allocate data in
context-managed heap space that can be collectively deleted.  All deserialized
data is placed on the context-managed heap by the `soap` context of the engine.

🔝 [Back to table of contents](#)

Memory management in C                                                {#memory1}
----------------------

When working with gSOAP in C (i.e. using <b>`wsdl2h -c`</b> option <b>`-c`</b>
or <b>`soapcpp2 -c`</b> option <b>`-c`</b>), data is allocated on the managed heap with:

- `void *soap_malloc(struct soap*, size_t len)`.

This function allocates `len` bytes on the heap managed by the specified
context and returns NULL when allocation failed.

You can also make shallow copies of data with `soap_memdup` that uses
`soap_malloc` and a safe version of `memcpy` to copy a chunk of data `src` with
length `len` to the context-managed heap:

- `void * soap_memdup(struct soap*, const void *src, size_t len)`

This function returns a pointer to the copy.  This function requires gSOAP
2.8.27 or later.

In gSOAP 2.8.35 and greater versions, you can use an auto-generated function to
allocate and initialize data of type `T` on the managed heap:

- `T * soap_new_T(struct soap*, int n)`

This function returns an array of length `n` of type `T` data that is default
initialized (by internally calling `soap_malloc(soap, n * sizeof(T))` and then
`soap_default_T(soap, T*)` on each array value).  Use a negative value or `n=1`
to allocate and initialize a single value.  This function returns NULL when
allocation failed.

The `soap_malloc` function is essentially a wrapper around `malloc`, but
permits the `soap` context to track all heap allocations for collective
deletion with `soap_end(soap)`:

~~~{.cpp}
    #include "soapH.h"
    #include "ns.nsmap"
    ...
    struct soap *soap = soap_new(); // new context
    ...
    struct ns__record *record = (struct ns__record*)soap_malloc(soap, sizeof(struct ns__record));
    soap_default_ns__record(soap, record); // auto-generated struct initializer
    ...
    soap_destroy(soap); // only for C++, see section on C++ below
    soap_end(soap);     // delete record and all other heap allocations
    soap_free(soap);    // delete context
~~~

All data on the managed heap is mass-deleted with `soap_end(soap)` which must
be called before `soap_done(soap)` or `soap_free(soap)`, which end the use of
the `soap` context and free the context, respectively.  Use
`soap_free(soap)` only when the context is allocated with `soap_new()`.  Use
`soap_done(soap)` only when the context is stack allocated (so cannot be
deleted from the heap).

The managed heap is checked for memory leaks at run time when the source code
is compiled with option <b>`-DDEBUG`</b>.

The soapcpp2 auto-generated deserializers in C use `soap_malloc` to allocate
and populate deserialized structures, which are managed by the context for
collective deletion.

To make `char*` and `wchar_t*` string copies to the context-managed heap, we
can use the functions:

- `char *soap_strdup(struct soap*, const char *str)` and

- `wchar_t *soap_wstrdup(struct soap*, const wchar_t *wstr)`.

If your C compiler supports `typeof` then you can use the following macro to
simplify the managed heap allocation and initialization of primitive values:

~~~{.cpp}
    #define soap_assign(soap, lhs, rhs) (*(lhs = (typeof(lhs))soap_malloc(soap, sizeof(*lhs))) = rhs)
~~~

Pointers to primitive values are often used for optional members.  For example,
assume we have the following struct:

~~~{.cpp}
    struct ns__record
    {
        const char        *name 1; // required (minOccurs=1)
        uint64_t          *SSN;    // optional (pointer means minOccurs=0)
        struct ns__record *spouse; // optional (pointer means minOccurs=0)
    };
~~~

Use `soap_assign` to create a SSN value on the managed heap:

~~~{.cpp}
    struct soap *soap = soap_new(); // new context
    ...
    struct ns__record *record = (struct ns__record*)soap_malloc(soap, sizeof(struct ns__record));
    soap_default_ns__record(soap, record);
    record->name = soap_strdup(soap, "Joe");
    soap_assign(soap, record->SSN, 1234567890UL);
    ...
    soap_end(soap);     // delete managed soap_malloc'ed heap data
    soap_free(soap);    // delete context
~~~

Without the `soap_assign` macro, you will need two lines of code, one to
allocate and one to assign (you should also use this if your system can run out
of memory):

~~~{.cpp}
    assert((record->SSN = (uint64_t*)soap_malloc(soap, sizeof(utint64_t))) != NULL);
    *record->SSN = 1234567890UL;
~~~

The serializer can serialize any heap, stack, or static allocated data.  So we
can also create a new record as follows:

~~~{.cpp}
    struct soap *soap = soap_new(); // new context
    ...
    struct ns__record *record = (struct ns__record*)soap_malloc(soap, sizeof(struct ns__record));
    static uint64_t SSN = 1234567890UL;
    soap_default_ns__record(soap, record);
    record->name = "Joe";
    record->SSN = &SSN; // safe to use static values: the value of record->SSN is never changed
    ...
    soap_end(soap);     // delete managed soap_malloc'ed heap data
    soap_free(soap);    // delete context
~~~

Use the soapcpp2 auto-generated `soap_dup_T` functions to duplicate data into
another `soap` context (this requires <b>`soapcpp2 -Ec`</b> option <b>`-Ec`</b> to
generate), here shown for C with the second argument `dst` NULL because we want
to allocate a new managed structure:

~~~{.cpp}
    struct soap *other_soap = soap_new(); // another context
    struct ns__record *other_record = soap_dup_ns__record(other_soap, NULL, record);
    ...
    soap_destroy(other_soap); // only for C++, see section on C++ below
    soap_end(other_soap);     // delete other_record and all of its deep data
    soap_free(other_soap);    // delete context
~~~

The only reason to use another `soap` context and not to use the primary `soap`
context is when the primary context must be destroyed together with all of the
objects it manages while some of the objects must be kept alive.  If the
objects that are kept alive contain deep cycles then this is the only option we
have, because deep copy with a managing `soap` context detects and preserves
these cycles unless the `SOAP_XML_TREE` flag is used with the `soap` context:

~~~{.cpp}
    struct soap *other_soap = soap_new1(SOAP_XML_TREE); // another context
    struct ns__record *other_record = soap_dup_ns__record(other_soap, NULL, record);
~~~

The resulting deep copy will be a full copy of the source data structure as a
tree without co-referenced data (i.e. no digraph) and without cycles.  Cycles
are pruned and (one of the) pointers that forms a cycle is repaced by NULL.

You can also deep copy into unmanaged space and use the auto-generated
`soap_del_T()` function (requires <b>`soapcpp2 -Ed`</b> option <b>`-Ed`</b> to generate) to delete
it later:

~~~{.cpp}
    struct ns__record *other_record = soap_dup_ns__record(NULL, NULL, record);
    ...
    soap_del_ns__record(other_record); // deep delete record data members
    free(other_record);                // delete the record
~~~

But you should not do this for any data that has deep cycles in its runtime
data structure.  Cycles in the data structure will lead to non-termination when
making unmanaged deep copies.  Consider for example:

~~~{.cpp}
    struct ns__record
    {
        const char         *name 1; // required (minOccurs=1)
        uint64_t            SSN;    // required (non-pointer means minOccurs=1)
        struct ns__record  *spouse; // optional (pointer means minOccurs=0)
    };
~~~

The code to populate a structure with a mutual spouse relationship:

~~~{.cpp}
    struct soap *soap = soap_new();
    ...
    struct ns__record pers1, pers2;
    soap_default_ns__record(soap, &pers1);
    soap_default_ns__record(soap, &pers2);
    pers1.name = "Joe";                     // OK to serialize static data
    pers1.SSN = 1234567890;
    pers1.spouse = &pers2;
    pers2.name = soap_strdup(soap, "Jane"); // allocates and copies a string
    pers2.SSN = 1987654320;
    pers2.spouse = &pers1;
    ...
    struct ns__record *pers3 = soap_dup_ns__record(NULL, NULL, &pers1); // BAD
    struct ns__record *pers4 = soap_dup_ns__record(soap, NULL, &pers1); // OK
    soap_set_mode(soap, SOAP_XML_TREE);
    struct ns__record *pers5 = soap_dup_ns__record(soap, NULL, &pers1); // OK
~~~

The bad case is where there is no context used in the first argument.  The deep
copy functions use a context to keep track of co-referenced data nodes and
cycles in the data structure copies, to copy co-referenced nodes just once.
Co-references in a data structure are formed by pointers and smart pointers
such as `std::shared_ptr`, such that at least two pointers point to the same
data.

The serializer can serialize any heap, stack, or static allocated data, such as
in the code shown above.  So we can serialize the stack-allocated `pers1`
record as follows:

~~~{.cpp}
    FILE *fp = fopen("record.xml", "w");
    if (fp != NULL)
    {
      soap->sendfd = fileno(fp);           // file descriptor to write to
      soap_set_mode(soap, SOAP_XML_GRAPH); // support id-ref w/o requiring SOAP
      soap_clr_mode(soap, SOAP_XML_TREE);  // if set, clear
      soap_write_ns__record(soap, &pers1);
      fclose(fp);
      soap->sendfd = -1;                   // block further writing
    }
~~~

which produces an XML document record.xml that is similar to:

<div class="alt">
~~~{.xml}
    <ns:record xmlns:ns="urn:types" id="Joe">
      <name>Joe</name>
      <SSN>1234567890</SSN>
      <spouse id="Jane">
        <name>Jane</name>
        <SSN>1987654320</SSN>
        <spouse ref="#Joe"/>
      </spouse>
    </ns:record>
~~~
</div>

Deserialization of an XML document with a SOAP 1.1/1.2 encoded id-ref graph
leads to the same non-termination problem when we later try to copy the data
into unmanaged memory heap space:
    
~~~{.cpp}
    struct soap *soap = soap_new1(SOAP_XML_GRAPH); // support id-ref w/o SOAP
    ...
    struct ns__record pers1;
    FILE *fp = fopen("record.xml", "r");
    if (fp != NULL)
    {
      soap->recvfd = fileno(fp);
      if (soap_read_ns__record(soap, &pers1))
        ... // handle IO error
      fclose(fp);
      soap->recvfd = -1;                    // blocks further reading
    }
    ...
    struct ns__record *pers3 = soap_dup_ns__record(NULL, NULL, &pers1); // BAD
    struct ns__record *pers4 = soap_dup_ns__record(soap, NULL, &pers1); // OK
    soap_set_mode(soap, SOAP_XML_TREE);
    struct ns__record *pers5 = soap_dup_ns__record(soap, NULL, &pers1); // OK
~~~

Copying data with `soap_dup_T(soap)` into managed heap memory space is always
safe.  Copying into unmanaged heap memory space requires diligence.  But
deleting unmanaged data is easy with `soap_del_T()`.

You can also use `soap_del_T()` to delete structures that you created in C, but
only if these structures are created with `malloc` and do NOT contain pointers
to stack and static data.

You can unlink one or more allocated objects from the managed heap to allow the
object to live after `soap_end(soap)` by using:

- `void soap_unlink(struct soap *soap, void *ptr)`

The unlinked heap-allocated data pointed to by `ptr` can be accessed after
`soap_end(soap)`.  Do not forget to free the data with `free(ptr)`.  Be aware
that `soap_unlink(soap, ptr)` does not perform a deep unlinkage.  If `ptr` is a
struct, pointer members will become invalid when pointing to objects on the
managed heap.  Use `soap_unlink(soap, ptr->member)` to unlink `member` as well.

Finally, when data is allocated in managed memory heap space, either explicitly
with the allocation functions shown above or by the soapcpp2-generated
deserializers, you can delegate the management and deletion of this data to
another `soap` context.  That context will be responsible to delete the data
with `soap_end(soap)` later:

- `void delegate_deletion(struct soap *soap_from, struct soap *soap_to)`

This allows the `soap_from` context to be deleted with `soap_free(soap_from)`
(assuming it is allocated with `soap_new()`, use `soap_done(soap_from)` when
`soap_from` is stack-allocated) while the managed data remains intact.  You
can use this function any time, to delegate management and deletion to another
context `soap_to` and then continue with the current context.  You can also use
different source `soap_from` contexts to delegate management and deletion to
the other `soap_to` context.  To mass delete all managed data, use
`soap_end(soap_to)`.

🔝 [Back to table of contents](#)

Memory management in C++                                              {#memory2}
------------------------

When working with gSOAP in C++, the engine allocates data on a managed heap
using `soap_new_T(soap)` to allocate a type with type name `T`.  Managed heap
allocation is tracked by the `soap` context for collective deletion with
`soap_destroy(soap)` for structs, classes, and templates and with
`soap_end(soap)` for everything else.

You should only use `soap_malloc(struct soap*, size_t len)` to allocate
primitive types because constructors are not invoked.  Therefore, `soap_new_T`
is preferred.  The auto-generated `T * soap_new_T(struct soap*)` returns data
allocated on the managed heap for type `T`.  The data is mass-deleted with
`soap_destroy(soap)` followed by `soap_end(soap)`.

The `soap_new_T` functions return NULL when allocation fails.  C++ exceptions
are never raised by the engine and serializers when data is allocated.

There are four variations of `soap_new_T` functions to allocate data of type
`T` that soapcpp2 auto-generates:

- `T * soap_new_T(struct soap*)` returns a new instance of `T` that is default
  initialized.  For classes, initialization is internally performed using the
  soapcpp2 auto-generated `void T::soap_default(struct soap*)` method of the
  class, but ONLY IF the soapcpp2 auto-generated default constructor is used
  that invokes `soap_default()` and was not replaced by a user-defined default
  constructor.

- `T * soap_new_T(struct soap*, int n)` returns an array of `n` new instances of
  `T`.  The instances in the array are default initialized as described above.

- `T * soap_new_req_T(struct soap*, ...)` (structs and classes only) returns a
  new instance of `T` and sets the required data members to the values
  specified in `...`.  The required data members are those with nonzero
  minOccurs, see the subsections on
  [(smart) pointer members and their occurrence constraints](#toxsd9-8) and
  [container and array members and their occurrence constraints](#toxsd9-9).

- `T * soap_new_set_T(struct soap*, ...)` (structs and classes only) returns a
  new instance of `T` and sets the public/serializable data members to the values
  specified in `...`.

The above functions can be invoked with a NULL `soap` context, but you are then
responsible to use `delete T` to remove this instance from the unmanaged heap.

For example, to allocate a managed `std::string` you can use:

~~~{.cpp}
    std::string *s = soap_new_std__string(soap);
~~~

To throw a `std::bad_alloc` exception when memory allocation fails, we can define the
following class and macro:

~~~{.cpp}
    class alloc_check {
     public:
      template<typename T>
      T operator=(T ptr)
      {
        if (ptr == NULL)
          throw std::bad_alloc();
        return ptr;
      }
    };

    #define CHECK alloc_check() =
~~~

And use `CHECK` as follows to throw an exception when memory allocation fails:

~~~{.cpp}
    std::string *s = CHECK soap_new_std__string(soap);
~~~

To throw a `std::runtime_exception` when memory allocation fails, with file
and line number information where the error occurred, we can define the
following revised version of our exception-throwing macro:

~~~{.cpp}
    class alloc_failure : public std::runtime_error {
     public:
      alloc_failure(const char *file, size_t line) : std::runtime_error(error(file, line))
      { }
     private:
      std::string error(const char *file, size_t line) const
      {
        std::stringstream ss;
        ss << "Memory allocation failed in " << file << " at line " << line;
        return ss.str();
      }
    };

    class alloc_check_with_info {
     public:
      alloc_check_with_info(const char *file, size_t line) : file(file), line(line)
      { }
      template<typename T>
      T operator=(T ptr) const
      {
        if (ptr == NULL)
          throw alloc_failure(file, line);
        return ptr;
      }
      const char *file;
      size_t line;
    };

    #define CHECK alloc_check_with_info(__FILE__, __LINE__) =
~~~

And use `CHECK` as follows to throw an exception with the file and line number
of the location where memory allocation failed:

~~~{.cpp}
    std::string *s = CHECK soap_new_std__string(soap);
~~~

Primitive types and arrays of primitive values may be allocated with
`soap_malloc` (actually, `soap_new_T` calls `soap_malloc` for primitive type
`T`).  All primitive types (i.e. no classes, structs, class templates,
containers, and smart pointers) are allocated with `soap_malloc` for reasons of
efficiency.

You can use a C++ template to simplify the managed allocation and initialization
of primitive values as follows (this is for primitive types only):

~~~{.cpp}
    template<class T>
    T * soap_make(struct soap *soap, T val)
    {
      T *p = (T*)soap_malloc(soap, sizeof(T));
      if (p == NULL)
        throw std::bad_alloc();
      *p = val;
      return p;
    }
~~~

For example, assuming we have the following class:

~~~{.cpp}
    class ns__record
    {
     public:
        std::string  name;   // required (non-pointer means minOccurs=1)
        uint64_t    *SSN;    // optional (pointer means minOccurs=0)
        ns__record  *spouse; // optional (pointer means minOccurs=0)
    };
~~~

You can instantiate a record by using the auto-generated
`soap_new_set_ns__record` and use `soap_make` to create a SSN value on the
managed heap as follows:

~~~{.cpp}
    soap *soap = soap_new(); // new context
    ...
    ns__record *record = soap_new_set_ns__record(
        soap,
        "Joe",
        soap_make<uint64_t>(soap, 1234567890UL),
        NULL);
    ...
    soap_destroy(soap); // delete record and all other managed instances
    soap_end(soap);     // delete managed soap_malloc'ed heap data
    soap_free(soap);    // delete context
~~~

All data on the managed heap is mass-deleted with `soap_end(soap)` which must
be called before `soap_done(soap)` or `soap_free(soap)`, which end the use of
the `soap` context and free the context, respectively.  Use
`soap_free(soap)` only when the context is allocated with `soap_new()`.  Use
`soap_done(soap)` only when the context is stack allocated (so cannot be
deleted from the heap).

The managed heap is checked for memory leaks at run time when the source code
is compiled with option <b>`-DDEBUG`</b>.

However, the serializer can serialize any heap, stack, or static allocated
data.  So we can also create a new record as follows:

~~~{.cpp}
    uint64_t SSN = 1234567890UL;
    ns__record *record = soap_new_set_ns__record(soap, "Joe", &SSN, NULL);
~~~

which will be fine to serialize this record as long as the local `SSN`
stack-allocated value remains in scope when invoking the serializer and/or
using `record`.  It does not matter if `soap_destroy` and `soap_end` are called
beyond the scope of `SSN`.

To facilitate class methods to access the managing context, we can add a soap
context pointer to a class/struct:

~~~{.cpp}
    class ns__record
    {
     public:
        ...
        void create_more(); // needs a context to create more internal data
      protected:
        struct soap *soap;  // the context that manages this instance, or NULL
    };
~~~

The `soap` context pointer member of the class is set when invoking
`soap_new_T` (and similar) with a non-NULL context argument that will be
assigned to the `soap` member of the class.

You can also use a template when an array of pointers to values is required.
To create an array of pointers to values, define the following template:

~~~{.cpp}
    template<class T>
    T **soap_make_array(struct soap *soap, T* array, int n) throw (std::bad_alloc)
    { 
      T **p = (T**)soap_malloc(soap, n * sizeof(T*));
      if (p == NULL)
        throw std::bad_alloc();
      for (int i = 0; i < n; ++i)
        p[i] = &array[i];
      return p;
    }
~~~

The `array` parameter is a pointer to an array of `n` values.  The template
returns an array of `n` pointers that point to the values in that array:

~~~{.cpp}
    // create an array of 100 pointers to 100 records
    int n = 100;
    ns__record **precords = soap_make_array(soap, soap_new_ns__record(soap, n), n);
    for (int i = 0; i < n; ++i)
    {
      precords[i]->name = "...";
      precords[i]->SSN = soap_make<uint64_t>(1234567890UL + i);
    }
~~~

Note that `soap_new_ns__record(soap, n)` returns a pointer to an array of `n`
records, which is then used to create an array of `n` pointers to these records.

Use the soapcpp2 auto-generated `soap_dup_T` functions to duplicate data into
another `soap` context (this requires <b>`soapcpp2 -Ec`</b> option <b>`-Ec`</b> to generate), here shown
for C++ with the second argument `dst` NULL to allocate a new managed object:

~~~{.cpp}
    soap *other_soap = soap_new(); // another context
    ns__record *other_record = soap_dup_ns__record(other_soap, NULL, record);
    ...
    soap_destroy(other_soap); // delete record and other managed instances
    soap_end(other_soap);     // delete other data (the SSNs on the heap)
    soap_free(other_soap);    // delete context
~~~

To duplicate base and derived instances when a base class pointer or reference
is provided, use the auto-generated method `T * T::soap_dup(struct soap*)`:

~~~{.cpp}
    soap *other_soap = soap_new(); // another context
    ns__record *other_record = record->soap_dup(other_soap);
    ...
    soap_destroy(other_soap); // delete record and other managed instances
    soap_end(other_soap);     // delete other data (the SSNs on the heap)
    soap_free(other_soap);    // delete context
~~~

The only reason to use another context and not to use the primary `soap`
context is when the primary context must be destroyed together with all of the
objects it manages while some of the objects must be kept alive.  If the
objects that are kept alive contain deep cycles then this is the only option we
have, because deep copy with a managing `soap` context detects and preserves
these cycles unless the `SOAP_XML_TREE` flag is used with the context:

~~~{.cpp}
    soap *other_soap = soap_new1(SOAP_XML_TREE);             // another context
    ns__record *other_record = record->soap_dup(other_soap); // deep tree copy
~~~

The resulting deep copy will be a full copy of the source data structure as a
tree without co-referenced data (i.e. no digraph) and without cycles.  Cycles
are pruned and (one of the) pointers that forms a cycle is repaced by NULL.

You can also deep copy into unmanaged space and use the auto-generated
`soap_del_T()` function or the `T::soap_del()` method (requires
<b>`soapcpp2 -Ec`</b> option <b>`-Ec`</b> to generate) to delete it later,
but we should not do this for any data that has deep cycles in its runtime data
structure graph:

~~~{.cpp}
    ns__record *other_record = record->soap_dup(NULL);
    ...
    other_record->soap_del(); // deep delete record data members
    delete other_record;      // delete the record
~~~

Cycles in the data structure will lead to non-termination when making unmanaged
deep copies.  Consider for example:

~~~{.cpp}
    class ns__record
    {
     public:
        const char  *name 1; // required (minOccurs=1)
        uint64_t     SSN;    // required (non-pointer means minOccurs=1)
        ns__record  *spouse; // optional (pointer means minOccurs=1)
    };
~~~

The code to populate a structure with a mutual spouse relationship:

~~~{.cpp}
    soap *soap = soap_new();
    ...
    ns__record pers1, pers2;
    pers1.name = "Joe";
    pers1.SSN = 1234567890;
    pers1.spouse = &pers2;
    pers2.name = "Jane";
    pers2.SSN = 1987654320;
    pers2.spouse = &pers1;
    ...
    ns__record *pers3 = soap_dup_ns__record(NULL, NULL, &pers1); // BAD
    ns__record *pers4 = soap_dup_ns__record(soap, NULL, &pers1); // OK
    soap_set_mode(soap, SOAP_XML_TREE);
    ns__record *pers5 = soap_dup_ns__record(soap, NULL, &pers1); // OK
~~~

The serializer can serialize any heap, stack, or static allocated data, such as
shown in the code shown above.  So we can serialize the stack-allocated `pers1`
record as follows:

~~~{.cpp}
    FILE *fp = fopen("record.xml", "w");
    if (fp != NULL)
    {
      soap->sendfd = fileno(fp);           // file descriptor to write to
      soap_set_mode(soap, SOAP_XML_GRAPH); // support id-ref w/o requiring SOAP
      soap_clr_mode(soap, SOAP_XML_TREE);  // if set, clear
      if (soap_write_ns__record(soap, &pers1))
        ... // handle IO error
          fclose(fp);
      soap->sendfd = -1;                   // block further writing
    }
~~~

which produces an XML document record.xml that is similar to:

<div class="alt">
~~~{.xml}
    <ns:record xmlns:ns="urn:types" id="Joe">
      <name>Joe</name>
      <SSN>1234567890</SSN>
      <spouse id="Jane">
        <name>Jane</name>
        <SSN>1987654320</SSN>
        <spouse ref="#Joe"/>
      </spouse>
    </ns:record>
~~~
</div>

Deserialization of an XML document with a SOAP 1.1/1.2 encoded id-ref graph
leads to the same non-termination problem when we later try to copy the data
into unmanaged space:
    
~~~{.cpp}
    soap *soap = soap_new1(SOAP_XML_GRAPH); // support id-ref w/o SOAP
    ...
    ns__record pers1;
    FILE *fp = fopen("record.xml", "r");
    if (fp != NULL)
    {
      soap->recvfd = fileno(fp);            // file descriptor to read from
      if (soap_read_ns__record(soap, &pers1))
        ... // handle IO error
      fclose(fp);
      soap->recvfd = -1;                    // block further reading
    }
    ...
    ns__record *pers3 = soap_dup_ns__record(NULL, NULL, &pers1); // BAD
    ns__record *pers4 = soap_dup_ns__record(soap, NULL, &pers1); // OK
    soap_set_mode(soap, SOAP_XML_TREE);
    ns__record *pers5 = soap_dup_ns__record(soap, NULL, &pers1); // OK
~~~

Copying data with `soap_dup_T(soap)` into managed space is always safe.  Copying
into unmanaged space requires diligence.  But deleting unmanaged data is easy
with `soap_del_T()`.

You can also use `soap_del_T()` to delete structures in C++, but only if these
structures are created with `new` (and `new []` for arrays when applicable) for
classes, structs, and class templates and with `malloc` for anything else, and
the structures do NOT contain pointers to stack and static data.

You can unlink one or more allocated objects from the managed heap to allow the
object to live after `soap_destroy(soap)` and `soap_end(soap)` by using:

- `void soap_unlink(struct soap *soap, void *ptr)`

The unlinked heap-allocated data pointed to by `ptr` can be accessed after
`soap_destroy(soap)` and `soap_end(soap)`.  Do not forget to free the data with
`delete ptr` (C++ class instance only) or with `free(ptr)` (non-class data).
Be aware that `soap_unlink(soap, ptr)` does not perform a deep unlinkage.  If
`ptr` is a struct or class, pointer members will become invalid when pointing
to objects on the managed heap.  Use `soap_unlink(soap, ptr->member)` to unlink
`member` as well.

Finally, when data is allocated in managed memory heap space, either explicitly
with the allocation functions shown above or by the soapcpp2-generated
deserializers, you can delegate the management and deletion of this data to
another `soap` context.  That context will be responsible to delete the data
with `soap_destroy(soap)` and `soap_end(soap)` later:

- `void delegate_deletion(struct soap *soap_from, struct soap *soap_to)`

This allows the `soap_from` context to be deleted with `soap_free(soap_from)`
(assuming it is allocated with `soap_new()`, use `soap_done(soap_from)` when
`soap_from` is stack-allocated) while the managed data remains intact.  You
can use this function any time, to delegate management and deletion to another
context `soap_to` and then continue with the current context.  You can also use
different source `soap_from` contexts to delegate management and deletion to
the other `soap_to` context.  To mass delete all managed data, use
`soap_destroy(soap_to)` followed by `soap_end(soap_to)`.

🔝 [Back to table of contents](#)

Context flags to initialize the soap struct                             {#flags}
===========================================

There are several `soap` context initialization flags and context mode flags to
control XML serialization at runtime.  The flags are set with `soap_new1()` to
allocate and initialize a new context:

~~~{.cpp}
    struct soap *soap = soap_new1(flag1 | flag2 | ... | flagn);
    ...
    soap_destroy(soap); // delete objects
    soap_end(soap);     // delete other data and temp data
    soap_free(soap);    // free context
~~~

and with `soap_init1()` for stack-allocated contexts:

~~~{.cpp}
    struct soap soap;
    soap_init1(&soap, flag1 | flag2 | ... | flagn);
    ...
    soap_destroy(&soap); // delete objects
    soap_end(&soap);     // delete other data and temp data
    soap_done(&soap);    // clear context
~~~

where `flag1`, `flag2`, ..., `flagn` is one of:

- `SOAP_C_UTFSTRING`: enables all `std::string` and `char*` strings to
  contain UTF-8 content.  This option is recommended.

- `SOAP_C_NILSTRING`: treat empty strings as if they were NULL pointers, i.e.
  omits elements and attributes when empty.

- `SOAP_XML_STRICT`: strictly validates XML while deserializing.  Should not be
  used together with SOAP 1.1/1.2 encoding style of messaging.  Use
  <b>`soapcpp2 -s`</b> option <b>`-s`</b> to hard code `SOAP_XML_STRICT` in the
  generated serializers.  Not recommended with SOAP 1.1/1.2 encoding style
  messaging.

- `SOAP_XML_INDENT`: produces indented XML.

- `SOAP_XML_CANONICAL`: c14n canonocalization, removes unused `xmlns` bindings
  and adds them to appropriate places by applying c14n normalization rules.
  Should not be used together with SOAP 1.1/1.2 encoding style messaging.

- `SOAP_XML_TREE`: write tree XML without id-ref, while pruning data structure
  cycles to prevent nontermination of the serializer for cyclic structures.

- `SOAP_XML_GRAPH`: write graph (digraph and cyclic graphs with shared pointers
  to objects) using id-ref attributes.  That is, XML with SOAP multi-ref
  encoded id-ref elements.  This is a structure-preserving serialization format,
  because co-referenced data and also cyclic relations are accurately represented.

- `SOAP_XML_DEFAULTNS`: uses xmlns default namespace declarations, assuming
   that the schema attribute form is "qualified" by default (be warned if it is
   not, since attributes in the null namespace will get bound to namespaces!).

- `SOAP_XML_NIL`: emit empty element with <i>`xsi:nil`</i> for all NULL pointers
  serialized.

- `SOAP_XML_IGNORENS`: the XML parser ignores XML namespaces, i.e. element and
  attribute tag names match independent of their namespace.

- `SOAP_XML_NOTYPE`: removes all <i>`xsi:type`</i> attribuation.  This option is usually
  not needed unless the receiver rejects all <i>`xsi:type`</i> attributes.  This option
  may affect the quality of the deserializer, which relies on <i>`xsi:type`</i>
  attributes to distinguish base class instances from derived class instances
  transported in the XML payloads.

- `SOAP_IO_CHUNK`: to enable HTTP chunked transfers.

- `SOAP_IO_STORE`: full buffering of outbound messages.

- `SOAP_ENC_ZLIB`: compress messages, requires compiling with option <b>`-DWITH_GZIP`</b> and
   linking with zlib using option <b>`-lz`</b>.

- `SOAP_ENC_MIME`: enable MIME attachments, see
  [DIME/MIME/MTOM attachment binary types](#toxsd10-3).

- `SOAP_ENC_MTOM`: enable MTOM attachments, see
  [DIME/MIME/MTOM attachment binary types](#toxsd10-3).

@note C++ Web service proxy and service classes have their own `soap` context, either
as a base class (with <b>`soapcpp2 -i`</b> option <b>`-i`</b>) or as a pointer member `soap` that points to
a context (with <b>`soapcpp2 -j`</b> option <b>`-j`</b>).  These contexts are allocated when the proxy or
service is instantiated with context flags that are passed to the constructor.

🔝 [Back to table of contents](#)

Context parameter settings                                             {#params}
==========================

After allocation and initializtion of a `soap` context, several context
parameters can be set (some parameters may require 2.8.31 or greater):

- `unsigned int soap::maxlevel` is the maximum XML nesting depth levels that
  the parser permits.  Default initialized to `SOAP_MAXLEVEL` (10000), which is
  a redefinable macro in <i>`gsoap/stdsoap2.h`</i>.  Set `soap::maxlevel` to a
  lower value to restrict XML parsing nesting depth.

- `long soap::maxlength` is the maximum string content length if not already
  constrained by an XML schema validation `maxLength` constraint.  Zero means
  unlimited string lengths are permitted (unless restricted by XML schema
  `maxLength`).  Default initialized to `SOAP_MAXLENGTH` (0), which is a
  redefinable macro in <i>`gsoap/stdsoap2.h`</i>.  Set `soap::maxlength` to a
  positive value to restrict the number of (wide) characters in strings parsed,
  restrict hexBinary byte length, and restrict base64Binary byte length.

- `size_t soap::maxoccurs` is the maximum number of array or container elements
  permitted by the parser.  Must be greater than zero (0).  Default initialized
  to `SOAP_MAXOCCURS` (100000), which is a redefinable macro in
  <i>`gsoap/stdsoap2.h`</i>.  Set `soap::maxoccurs` to a positive value to
  restrict the number of array and container elements that can be parsed.

- `soap::version` is the SOAP version used, with 0 for non-SOAP, 1 for SOAP1.1,
  and 2 for SOAP1.2.  This value is normally set by web service operations, and
  is otherwise 0 (non-SOAP).  Use `soap_set_version(struct soap*, short)` to
  set the value.  This controls XML namespaces and SOAP id-ref serialization
  when applicable with an encodingStyle (see below).

- `const char *soap::encodingStyle` is a string that is used with SOAP
  encoding, normally NULL for non-SOAP XML.  Set this string to "" (empty
  string) to enable SOAP encoding style, which supports id-ref graph
  serialization (see also the `SOAP_XML_GRAPH` [context flag](#flags)).

- `int soap::recvfd` is the file descriptor to read and parse source data from.
  Default initialized to 0 (stdin).  See also [input and output](#io).

- `int soap::sendfd` is the file descriptor to write data to.  Default
  initialized to 1 (stdout).  See also [input and output](#io).

- `const char *is` for C: string to read and parse source data from, overriding
  the `recvfd` source.  Normally NULL.  This value must be reset to NULL or
  the parser will continue to read from this string content until the NUL
  character.  See also [input and output](#io).

- `std::istream *is` for C++: an input stream to read and parse source data
  from, overriding the `recvfd` source.  Normally NULL.  This value must be
  reset to NULL or the parser will continue to read from this stream until EOF.
  See also [input and output](#io).

- `const char **os` for C: points to a string (a `const char *`) that will be
  set to point to the string output.  Normally NULL.  This value must be reset
  to NULL or the next output will result in reassigning the pointer to point to
  the next string that is output.  The strings are automatically deallocated by
  `soap_end(soap)`.  See also [input and output](#io).

- `std::ostream *os` for C++: an output stream to write output to.  Normally
  NULL.  This value must be reste to NULL or the next output will be send to
  this stream.  See also [input and output](#io).

🔝 [Back to table of contents](#)

Error handling and reporting                                           {#errors}
============================

The gSOAP API functions return `SOAP_OK` (zero) or a non-zero error code.  The
error code is stored in `int soap::error` of the current `soap` context.
Error messages can be displayed with:

- `void soap_stream_fault(struct soap*, std::ostream &os)` for C++ only, prints
  the error message to an output stream.

- `void soap_print_fault(struct soap*, FILE *fd)` prints the error message to a
  FILE descriptor.

- `void soap_sprint_fault(struct soap*, char *buf, size_t len)` saves the error
  message to a fixed-size buffer allocated with a maximum length.

- `void soap_print_fault_location(struct soap*, FILE *fd)` prints the location
  and part of the XML where the parser encountered an error.

C++ exceptions are never raised by the engine or serializers, even when data is
allocated.

A `SOAP_EOM` error code is returned when memory was exhausted during
processing of input and/or output of data.

An EOF (`SOAP_EOF` or -1) error code is returned when the parser has hit EOF
but expected more input, or when socket communications timed out.  In addition
to the `SOAP_EOF` error, the `int soap::errnum` of the `soap` context is
set to the `errno` value of the operation that failed.  For timeouts, the
`soap::ernum` value is always 0 instead of an `errno` error code.

Use `soap_xml_error_check(soap->error)` to check for XML errors.  This returns
true (non-zero) when a parsing and validation error has occurred.

For example:

~~~{.cpp}
    #include <sstream>

    struct soap *soap = soap_new1(SOAP_XML_INDENT | SOAP_XML_STRICT | SOAP_XML_TREE);
    struct ns__record person;
    std::stringstream ss;
    ss.str("...");      // XML to parse
    soap->is = &ss;
    if (soap_read__ns__record(soap, &person))
    {
      if (soap_xml_error_check(soap->error))
        std::cerr << "XML parsing error!" << std::endl;
      else
        soap_stream_fault(soap, std::cerr);
    }
    else
    {
      ...               // all OK, use person record
    }
    soap_destroy(soap); // delete objects
    soap_end(soap);     // delete other data and temp data
    soap_free(soap);    // free context
~~~

When deploying your application on UNIX and Linux systems, UNIX signal handlers
should be added to your code handle signals, in particular `SIGPIPE`:

~~~{.cpp}
    signal(SIGPIPE, sigpipe_handler);
~~~

where the `sigpipe_handler` is a function:

~~~{.cpp}
    void sigpipe_handler(int x) { }
~~~

Other UNIX signals may have to be handled as well.

The engine is designed for easy memory cleanup after being interrupted.  Use
`soap_destroy(soap)` and `soap_end(soap)`, after which the `soap` context can
be reused.

🔝 [Back to table of contents](#)

Features and limitations                                             {#features}
========================

In general, to use the generated code:

- Make sure to `#include "soapH.h"` in your code and also define a namespace
  table or `#include "ns.nsmap"` with the generated table, where `ns` is the
  namespace prefix for services.

- Use <b>`soapcpp2 -j`</b> option <b>`-j`</b> (C++ only) to generate C++ proxy and service objects.
  The auto-generated files include documented inferfaces.  Compile with
  <i>`soapC.cpp`</i> and link with <b>`-lgsoap++`</b>, or alternatively compile
  <i>`gsoap/stdsoap2.cpp`</i>.

- Without <b>`soapcpp2 -j`</b> option <b>`-j`</b>: client-side uses the auto-generated
  <i>`soapClient.cpp`</i> and <i>`soapC.cpp`</i> (or C versions of those).
  Compile and link with <b>`-lgsoap++`</b> (<b>`-lgsoap`</b> for C), or
  alternatively compile <i>`gsoap/stdsoap2.cpp`</i> (<i>`gsoap/stdsoap2.c`</i>
  for C).

- Without <b>`soapcpp2 -j`</b> option <b>`-j`</b>: server-side uses the
  auto-generated <i>`soapServer.cpp`</i> and <i>`soapC.cpp`</i> (or C versions
  of those).  Compile and link with <b>`-lgsoap++`</b> (<b>`-lgsoap`</b> for
  C), or alternatively compile <i>`gsoap/stdsoap2.cpp`</i> (<i>`stdsoap2.c`</i>
  for C).

- Use `soap_new()` or `soap_new1(int flags)` to allocate and initialize a
  heap-allocated `soap` context with or without flags.  Delete this `soap` context with
  `soap_free(struct soap*)`, but only after `soap_destroy(struct soap*)` and
  `soap_end(struct soap*)`.

- Use `soap_init(struct *soap)` or `soap_init1(struct soap*, int flags)` to
  initialize a stack-allocated `soap` context with or without flags.  End the use of
  this context with `soap_done(struct soap*)`, but only after
  `soap_destroy(struct soap*)` and `soap_end(struct soap*)`.

Additional notes with respect to the wsdl2h and soapcpp2 tools:

- Nested classes, structs, and unions in a interface header file are unnested
  by soapcpp2.

- Use `#import "file.h"` instead of `#include` to import other header files in
  a interface header file for soapcpp2.  The `#include`, `#define`, and
  `#pragma` are accepted by soapcpp2, but are moved to the very start of the
  generated code for the C/C++ compiler to include before all generated
  definitions.  Often it is useful to add an `#include` with a
  [volatile type](#toxsd9-2) that includes the actual type declaration, and to
  ensure transient types are declared when these are used in a data binding
  interface declared in a interface header file for soapcpp2.

- To remove any SOAP-specific bindings, use <b>`soapcpp2 -0`</b> option <b>`-0`</b>.

- A interface header file for soapcpp2 should not include any code statements,
  only data type declarations.  This includes constructor initialization lists
  that are not permitted.  Use member initializations instead.

- C++ namespaces are supported.  Use <b>`wsdl2h -qname`</b> option
  <b>`-qname`</b> to add C++ namespace `name`.  Or add a `namespace name { ... }`
  to the header file, but the `{ ... }` must cover the entire
  header file content from begin to end.
  
- Optional XML DOM support can be used to store mixed content or literal XML
  content.  Otherwise, mixed content may be lost.  Use <b>`wsdl2h -d`</b>
  option <b>`-d`</b> for XML DOM support and compile and link with
  <i>`gsoap/dom.c`</i> or <i>`gsoap/dom.cpp`</i>.  For details,
  see [XML DOM and XPath](http://www.genivia.com/doc/dom/html).

🔝 [Back to table of contents](#)

Removing SOAP namespaces from XML payloads                              {#nsmap}
==========================================

The soapcpp2 tool generates a <i>`.nsmap`</i> file that includes two bindings for SOAP
namespaces.  We can remove all SOAP namespaces (and SOAP processing logic) with
<b>`soapcpp2 -0`</b> option <b>`-0`</b> or by simply setting the two entries to NULL:

~~~{.cpp}
    struct Namespace namespaces[] =
    {
      {"SOAP-ENV", NULL, NULL, NULL},
      {"SOAP-ENC", NULL, NULL, NULL},
      ...
    };
~~~

Once the <i>`.nsmap`</i> is generated, you can copy-paste the content into your
project code.  However, if we rerun wsdl2h on updated WSDL/XSD files or
<i>`typemap.dat`</i> declarations then we need to use the updated table.

In cases that no XML namespaces are used at all, for example with
[XML-RPC](http://www.genivia.com/doc/xml-rpc-json/html), you may use an empty
namespace table:

~~~{.cpp}
    struct Namespace namespaces[] = {{NULL,NULL,NULL,NULL}};
~~~

However, beware that any built-in xsi attributes that are rendered will lack
the proper namespace binding.  At least we suggest to use `SOAP_XML_NOTYPE` for
this reason.

🔝 [Back to table of contents](#)

Examples                                                             {#examples}
========

Select the project files below to peruse the source code examples.

🔝 [Back to table of contents](#)

Source files
------------

- <i>`address.xsd`</i>         Address book schema
- <i>`address.cpp`</i>         Address book app (reads/writes address.xml file)
- <i>`addresstypemap.dat`</i>  Schema namespace prefix name preference for wsdl2h
- <i>`graph.h`</i>             Graph data binding (tree, digraph, cyclic graph)
- <i>`graph.cpp`</i>           Test graph serialization as tree, digraph, and cyclic

🔝 [Back to table of contents](#)

Generated files
---------------

- <i>`address.h`</i>      data binding interface generated from address.xsd
- <i>`addressStub.h`</i>  C++ data binding definitions
- <i>`addressH.h`</i>     Serializers
- <i>`addressC.cpp`</i>   Serializers
- <i>`address.xml`</i>    Address book data generated by address app
- <i>`graphStub.h`</i>    C++ data binding definitions
- <i>`graphH.h`</i>       Serializers
- <i>`graphC.cpp`</i>     Serializers
- <i>`g.xsd`</i>          XSD schema with <i>`g:Graph`</i> complexType
- <i>`g.nsmap`</i>        xmlns bindings namespace mapping table

🔝 [Back to table of contents](#)

Build steps
-----------

Building the AddressBook example:

    wsdl2h -g -taddresstypemap.dat address.xsd
    soapcpp2 -0 -C -S -paddress -I../../import address.h
    c++ -I../.. address.cpp addressC.cpp -o address -lgsoap++

Using <b>`wsdl2h -g -taddresstypemap.dat`</b> option <b>`-g`</b> produces
bindings for global (root) elements in addition to types and option
<b>`-taddresstypemap.dat`</b> specifies a mapping file, see further below.

In this case the root element <i>`a:address-book`</i> is bound to `_a__address_book`.
The complexType <i>`a:address`</i> is bound to class `a__address`, which is also the
type of `_a__address_book`.  This option is not required, but allows you to use
global element tag names when referring to their serializers, instead of their
type name.  Using <b>`soapcpp2 -0 -C -S -paddress`</b> option <b>`-0`</b> removes the
SOAP protocol and the combination of the two options <b>`-C`</b> and
<b>`-S`</b> removes client and server code generation (using option <b>`-C`</b>
alone generates client code and using option <b>`-S`</b> alone generates server
code).  Option <b>`-paddress`</b> renames the output <i>`soap`</i>-prefixed files to
<i>`address`</i>-prefixed files.

See the <i>`address.cpp`</i> implementation and [related pages](pages.html).

The <i>`addresstypemap.dat`</i> file specifies the XML namespace prefix for the
bindings:

    #       Bind the address book schema namespace to prefix 'a'

    a = "urn:address-book-example"

    #       By default the xsd:dateTime schema type is translated to time_t
    #       To map xsd:dateTime to struct tm, enable the following line:

    # xsd__dateTime = #import "../../custom/struct_tm.h"

    #       ... and compile/link with custom/struct_tm.c

The DOB field is a <i>`xsd:dateTime`</i>, which is bound to `time_t` by default.  To
change this to `struct tm`, enable the import of the `xsd__dateTime` custom
serializer by uncommenting the definition of `xsd__dateTime` in
<i>`addresstypemap.dat`</i>.  Then change `soap_dateTime2s` to `soap_xsd__dateTime2s`
in the code.

Building the graph serialization example:

    soapcpp2 -C -S -pgraph -I../../import graph.h
    c++ -I../.. graph.cpp graphC.cpp -o graph -lgsoap++

To compile without using the <b>`-lgsoap++`</b> library: simply compile
<i>`stdsoap2.cpp`</i> together with the above.

🔝 [Back to table of contents](#)

Usage
-----

To execute the AddressBook example:

    ./address

To execute the Graph serialization example:

    ./graph

