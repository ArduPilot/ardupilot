
XML-RPC/JSON and jsoncpp                                             {#mainpage}
========================

[TOC]

Introduction                                                            {#intro}
============

XML-RPC predates JSON and shares the same goal to offer a simple data format
for Web applications that interoperate via "remote procedure calls" (RPC) over
"stateless" HTTP via HTTP POST.  Applications are not limited to the exchange
of RPC messages via HTTP POST.  Other REST methods can be used to manage the
state of resources via URL references, allowing for the storage of data (HTTP
PUT), retrieval of data (HTTP GET), and removal of data (HTTP DELETE) from a
resource.

XML-RPC is a generic, self-describing (and very verbose) XML format to compose
XML messages for platform-neutral data exchange.  XML-RPC defines a collection
of frequently used XML types with common programming language equivalents.
XML-RPC does NOT provide a data binding to XML and does NOT support a
validation mechanism to ensure that data content is validated against a schema.
XML-RPC serialization proceeds by marshaling parameters in predefined XML
elements for each data type.  XML-RPC has primitive types (bool, int, double,
string, dateTime, base64) and two compound types (structs and arrays).

This document does not describe XML-RPC in detail.  For more details, please
visit <http://www.xmlrpc.com>.

JSON (JavaScript Object Notation) is an even simpler data format to support
platform-neutral data interchange that is highly compatible across programming
languages by restricting data representation to a set of five common types:
bool, number, string, array, and object.  A JSON object is the same as an
XML-RPC struct.  Only the syntax differs.  Both are composed of fieldname-value
member pairs (i.e. dictionaries or hashmaps) and have no other special
properties.  Which is in contrast to XML data "as objects" that are namespace
scoped and may include xsi:type attributes to distinguish derived from base
types, and may include id-ref attributes to cross-reference data, and other
properties that make XML more suitable to achieve lossless C/C++ serialization.

This document does not describe JSON (and JSON RPC/REST) in detail.  For more
details, please visit <http://www.json.org>.

This document describes both the C and C++/C++11 XML-RPC and JSON library APIs,
see table of contents.

JSON and gSOAP                                                       {#intro-1}
--------------

The gSOAP JSON API is compact and lightweight.  It is straightforward to write
JSON RPC and JSON REST code.  For example, a JSON REST call in C++:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    #include "json.h"
    int main()
    {
      soap *ctx = soap_new1(SOAP_C_UTFSTRING);    // set up context to manage memory
      const char *endpoint = "http://www.cs.fsu.edu/~engelen/currentTimeJSON.cgi";
      value req(ctx), res(ctx);                   // new JSON values req and res
      req = "getCurrentTime";                     // request current time
      json_call(ctx,                              // make a call (POST)
          endpoint,                               // the service endpoint URL
          req,                                    // value with the request string
          res)                                    // response, if call is OK
      );
      if (ctx->error)
        ...                                       // handle IO error or HTTP status code
      else
        cout << "Current time = " << res << endl; // JSON response to cout
    }
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For more details on JSON-RPC and JSON REST operations, see \ref cpp-rpc.

To compile this example see the \ref cpp-files.

Furthermore, to help you quickly develop C/C++ JSON code, we offer a code
generator **jsoncpp** with the gSOAP package (version 2.8.26 and up).  You can
find the jsoncpp tool with the JSON examples in `gsoap/samples/xml-rpc-json`.

The jsoncpp command-line tool auto-generates C or C++ code from a JSON
fragment.  The generated code creates a JSON node graph for this fragment,
which can be further tweaked as necessary.  We demonstrate this on an example
`menu.json` file, where we show each command executed in a command shell
followed by the results displayed in the terminal:

    cat menu.json

<div class="alt">
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    {
      "menu": {
        "id": "file",
        "value": "File",
        "popup": {
          "menuitem": [
            {"value": "New", "onclick": "CreateNewDoc()"},
            {"value": "Open", "onclick": "OpenDoc()"},
            {"value": "Close", "onclick": "CloseDoc()"}
          ]
        }
      }
    }
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
</div>

    jsoncpp menu.json

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    #include "json.h"
    { // Generated by jsoncpp menu.json
      struct soap *ctx = soap_new1(SOAP_C_UTFSTRING | SOAP_XML_INDENT);
      ctx->double_format = "%lG";

      value x(ctx);
      x["menu"]["id"] = "file";
      x["menu"]["value"] = "File";
      x["menu"]["popup"]["menuitem"][0]["value"] = "New";
      x["menu"]["popup"]["menuitem"][0]["onclick"] = "CreateNewDoc()";
      x["menu"]["popup"]["menuitem"][1]["value"] = "Open";
      x["menu"]["popup"]["menuitem"][1]["onclick"] = "OpenDoc()";
      x["menu"]["popup"]["menuitem"][2]["value"] = "Close";
      x["menu"]["popup"]["menuitem"][2]["onclick"] = "CloseDoc()";
      std::cout << x << std::endl;

      soap_destroy(ctx);
      soap_end(ctx);
      soap_free(ctx);
    }
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can use jsoncpp option `-e` to add explanatory comments to the generated
code, which explains what the code does to help you understand the JSON API.

Use jsoncpp option `-M` to narrow the generated code down to the essentials,
without the initialization and cleanup parts of the code.  This makes the
generated code suitable for direct inclusion in your codebase.

Generating code to populate a node graph is one option.  Another option is to
generate code to inspect a node graph.  Use jsoncpp option `-i` (gSOAP 2.8.28
and up) to generate code to inspect the node graph of a value parsed from JSON
input, given that the JSON file provided with option `-i` serves as a generic
template:

    jsoncpp -i menu.json

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    #include "json.h"
    { // Generated by jsoncpp -i menu.json
      struct soap *ctx = soap_new1(SOAP_C_UTFSTRING | SOAP_XML_INDENT);
      ctx->double_format = "%lG";

      value x(ctx);
      std::cin >> x;
      if (x.soap->error)
        exit(EXIT_FAILURE); // error parsing JSON
      #define USE_VAL(path, val) std::cout << path << " = " << val << std::endl
      if (x.has("menu"))
      {
        if (x["menu"].has("id"))
          USE_VAL("$.menu.id", x["menu"]["id"]);
        if (x["menu"].has("value"))
          USE_VAL("$.menu.value", x["menu"]["value"]);
        if (x["menu"].has("popup"))
        {
          if (x["menu"]["popup"].has("menuitem"))
          {
            for (int i3 = 0; i3 < x["menu"]["popup"]["menuitem"].size(); i3++)
            {
              if (x["menu"]["popup"]["menuitem"][i3].has("value"))
                USE_VAL("$.menu.popup.menuitem[].value", x["menu"]["popup"]["menuitem"][i3]["value"]);
              if (x["menu"]["popup"]["menuitem"][i3].has("onclick"))
                USE_VAL("$.menu.popup.menuitem[].onclick", x["menu"]["popup"]["menuitem"][i3]["onclick"]);
            }
          }
        }
      }
      std::cout << x << std::endl;

      soap_destroy(ctx);
      soap_end(ctx);
      soap_free(ctx);
    }
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

As you notice, each access to deeper JSON values in objects is guarded by a
check that the object property is present.  This helps to identify missing data
in the JSON data received, but this checking is not strictly necessary to
access values.  When a non-existent obect property is accessed, its value
returned is `null`.  For example, you can access `x["menu"]["id"]` directly to
obtain the `id` value, even when `menu` is not an object because this will cast
`menu` into an object (i.e. internally changing the JSON graph without
triggering exceptions or errors):
  
    jsoncpp -k menu.json

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    #include "json.h"
    { // Generated by jsoncpp -k menu.json
      struct soap *ctx = soap_new1(SOAP_C_UTFSTRING | SOAP_XML_INDENT);
      ctx->double_format = "%lG";

      value x(ctx);
      std::cin >> x;
      if (x.soap->error)
        exit(EXIT_FAILURE); // error parsing JSON
      #define USE_VAL(path, val) std::cout << path << " = " << val << std::endl
      USE_VAL("$.menu.id", x["menu"]["id"]);
      USE_VAL("$.menu.value", x["menu"]["value"]);
      for (int i0 = 0; i0 < x["menu"]["popup"]["menuitem"].size(); i0++)
      {
        USE_VAL("$.menu.popup.menuitem[].value", x["menu"]["popup"]["menuitem"][i0]["value"]);
        USE_VAL("$.menu.popup.menuitem[].onclick", x["menu"]["popup"]["menuitem"][i0]["onclick"]);
      }
      std::cout << x << std::endl;

      soap_destroy(ctx);
      soap_end(ctx);
      soap_free(ctx);
    }
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

On the other hand, checking for the presence of data before accessing it
preserves its original structure, which may be important with respect to the
type of application that uses JSON.

The two examples above display the JSON values with the macro `USE_VAL`, which
takes a JSON `val` of type `value`.

Furthermore, we can assign the JSON values to auto-generated local variables
using option `-l` with options `-i` or `-k`:

    jsoncpp -k -l menu.json

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    #include "json.h"
    { // Generated by jsoncpp -k -l menu.json
      struct soap *ctx = soap_new1(SOAP_C_UTFSTRING | SOAP_XML_INDENT);
      ctx->double_format = "%lG";

      value x(ctx);
      std::cin >> x;
      if (x.soap->error)
        exit(EXIT_FAILURE); // error parsing JSON
      #define USE_VAL(path, val) std::cout << path << " = " << val << std::endl
      value& x_menu = x["menu"];
      value& x_menu_id = x_menu["id"];
      USE_VAL("$.menu.id", x_menu_id);
      value& x_menu_value = x_menu["value"];
      USE_VAL("$.menu.value", x_menu_value);
      value& x_menu_popup = x_menu["popup"];
      value& x_menu_popup_menuitem = x_menu_popup["menuitem"];
      for (int i = 0; i < x_menu_popup_menuitem.size(); i++)
      {
        value& x_menu_popup_menuitemi = x_menu_popup_menuitem[i];
        value& x_menu_popup_menuitemi_value = x_menu_popup_menuitemi["value"];
        USE_VAL("$.menu.popup.menuitem[].value", x_menu_popup_menuitemi_value);
        value& x_menu_popup_menuitemi_onclick = x_menu_popup_menuitemi["onclick"];
        USE_VAL("$.menu.popup.menuitem[].onclick", x_menu_popup_menuitemi_onclick);
      }
      std::cout << x << std::endl;

      soap_destroy(ctx);
      soap_end(ctx);
      soap_free(ctx);
    }
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

JSON values of type `value` can be easily cast to and from C/C++ types, see
\ref cpp-ex for details.

You can also use the new jsoncpp option `-p` (gSOAP 2.8.27 and up) to generate
efficient JSONPath query code to query and retrieve specific values.

For example, let's write a JSONPath query to display the authors of books in a
store.  We will read the JSON data from `std:cin` and filter the authors with
the query `$.store.book[*].author` to collect them in a JSON array `y` of
results with jsoncpp option `-y`.  We generate the code from the command line
with jsoncpp as follows:

    jsoncpp -M -p'$.store.book[*].author' -y

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    value x(ctx);
    std::cin >> x;
    // JSONPath: $.store.book[*].author
    value y(ctx);
    y.size(0);
    #define QUERY_YIELD(val) y[y.size()] = val
    if (x.has("store"))
    {
      if (x["store"].has("book"))
      {
        value::iterator j = x["store"]["book"].begin();
        value::iterator k = x["store"]["book"].end();
        for (value::iterator i = j; i != k; ++i)
        {
          if ((*i).has("author"))
          {
            QUERY_YIELD((*i)["author"]);
          }
        }
      }
    }
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Let's apply this query to the `store.json` file that you can find in section
\ref jsoncpp-4:

    ./query < store.json

<div class="alt">
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    [
      "Nigel Rees",
      "Evelyn Waugh",
      "Herman Melville",
      "J. R. R. Tolkien"
    ]
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
</div>

You can compile this example yourself with:

    jsoncpp -o query.cpp -m -p'$.store.book[*].author' -y
    soapcpp2 -CSL xml-rpc.h
    c++ -o query query.cpp json.cpp xml-rpc.cpp soapC.cpp stdsoap2.cpp

You can also embed C/C++ code in JSONPath queries to filter and select values
from JSON data at runtime.

The jsoncpp code generator aims to produce clean, high-quality and readable C
and C++ code.  We will present jsoncpp in more detail in the next section.  The
remainder of this document explains how you can use the XML-RPC/JSON C and C++
APIs to create JSON data, access JSON data, send/recv data via REST, and to
read/write JSON data to files, streams, and string buffers.

It should be stated that JSON as a data format is not a true-and-tested
alternative to XML and XML schema.  XML data bindings provide a strongly typed
interface to exchange validated data with RPC and REST.  However, XML can be
more complex to provide strong guarantees for object polymorphism (base and
derived classes), to accurately represent tree and graph structures, to include
binary content natively with base64 (and mechanisms for streaming MIME/MTOM
attachments), to support extensibility (to extend data types and to add new
data types), and schema namespaces referenced by XML elements and attributes to
avoid ambiguity.

The jsoncpp command-line tool                                         {#jsoncpp}
=============================

The jsoncpp command produces high-quality, readable and reusable source code.
The generated code can be readily used in your projects to populate JSON data
and retrieve data, thereby saving you substantial time and effort to write
code.  You may not have to write any C or C++ code to manipulate JSON data with
your application's code base when taking full advantage of the jsoncpp
autocoding tool.

The jsoncpp command-line tool generates C or C++ source code to populate a
JSON node graph with the data given in a JSON file.  The command also has an
option `-i` to generate source code to inspect parsed JSON values by using a
JSON file as a generic template for this code.  And option `-p` generates
efficient source code for JSONPath queries.  Even stand-alone JSONPath query
filter applications can be auto-generated with option `-m` (for main).

Building and installing jsoncpp                                     {#jsoncpp-1}
-------------------------------

You will find jsoncpp and the XML-RPC/JSON examples in the gSOAP package in
`gsoap/samples/xml-rpc-json`.

To build jsoncpp, [install gSOAP](http://www.genivia.com/downloads.html) and build
jsoncpp as follows:

    cd gsoap/samples/xml-rpc-json
    make jsoncpp

This builds the command-line tool jsoncpp in `gsoap/samples/xml-rpc-json` from
where you can use it and/or copy it for use with your projects.

If you do not have the samples built, you can use soapcpp2 from the command
line to generate the C++ code required to compile jsoncpp and that is also
required by the C++ JSON API components:

    cd gsoap/samples/xml-rpc-json
    soapcpp2 -CSL xml-rpc.h
    c++ -I../.. -o jsoncpp jsoncpp.cpp json.cpp xml-rpc.cpp soapC.cpp ../../stdsoap2.cpp

This builds the jsoncpp command-line tool.

For users of Windows, visit
[download and installation](http://www.genivia.com/downloads.html) to download
`jsoncpp.exe`.

Command-line options                                                {#jsoncpp-2}
--------------------

The jsoncpp command takes several options and an optional JSON input file:

    jsoncpp [-c] [-e] [-f%fmt] [-h] [-i] [-k] [-l] [-m] [-M] [-O] [-ofile] [-ppath] [-rroot] [-xcode] [-y] [infile]

where the jsoncpp command-line options are:

| Option   | Description                                                       |
| -------- | ----------------------------------------------------------------- |
| `-c`     | generate C code instead of C++                                    |
| `-e`     | add explanatory comments to the generated code                    |
| `-f%%fmt`| use `%%fmt` to format double floats, e.g. `-f%%lG`                |
| `-h`     | display help message                                              |
| `-i`     | generate code to inspect JSON node graph with data type checks    |
| `-k`     | generate code to inspect JSON node graph directly without checks  |
| `-l`     | generate code for option `-i` to store values in local variables  |
| `-m`     | generate stand-alone code by adding `main()`                      |
| `-M`     | generate minimal code unadorned with initialization and cleanup   |
| `-O`     | optimize code by factoring common indices                         |
| `-ofile` | save source code to `file`                                        |
| `-ppath` | generate JSONPath query code for `path`                           |
| `-rroot` | use `root` instead of root value `x` in the generated code        |
| `-xcode` | generate code that executes `code` for each JSONPath query result |
| `-y`     | generate code that yields an array `y` of JSONPath query results  |
| `infile` | optional JSON file to parse                                       |
| `-`      | read JSON from standard input                                     |

The jsoncpp command takes a JSON input file `infile` to generate code to
construct this JSON value in C/C++ or, with options `-i` or `-k`, to generate
code that reads JSON data from input and traverses it to inspect its value by
using the JSON input file `infile` as a template to match against.  For options
`-i` and `-k`, if you want additional code that uses local variables to store
boolean, integer, and floating point values retrieved from the JSON node graph,
then also use option `-l`.

Use option `-c` to generate C code instead of C++ and use option `-e` to add
explanatory comments to the generated code.

The jsoncpp command emits source code to standard output or to the file
specified with option `-o`.

Minimalistic code is generated with option `-M`, which is useful to automate
pasting of the unadorned source code into the source code of your project.

Optimized code is generated with option `-O` by factoring common array indices
and object field names.  This produces more elaborate code that is more
efficient but may be harder to read and modify.  This option has no effect on
the code generated with options `-i` and `-k`.

The default name of the root value in the generated source code is `x`.  To
change this name use option `-r`.  Do not use the name `v`, which represents
the current value.  Other variable names to avoid are `i`, `j`, `k`, `p`, `q`,
`r`, `s`, and `S`, since these are internally used by the generated JSONPath
query code.

Use option `-p` to generate code that filters JSON data from a source of input
with a JSONPath query `path`.  Option `-x` specifies a JSONPath query code to
execute for each query result.  The default action in the generated code is
to print each query result value in JSON format separated by newlines.  Option
`-y` yields a JSON array of query result values that are incrementally
collected.  Option `-x` overrides option `-y`.

To generate a stand-alone application use option `-m`.  This option is useful
for testing JSONPath query filters given with option `-p`.

Option `-f%%fmt` sets the floating point double precision format to use in the
generated code.  By default, jsoncpp emits floating point numbers with up to 17
digit mantissas to preserve precision.  Use `-f%%lG` for the shortest floating
point representation.

JSONPath syntax                                                     {#jsoncpp-3}
---------------

The concept behind JSONPath is identical to XPath for XML: to select elements
(value nodes) from a DOM node structure of a document by matching the path
expression against descendent nodes in the node tree structure.  A JSONPath
query returns the JSON values selected.

We adopt the JSONPath syntax suggested by
[Goessner](http://goessner.net/articles/JsonPath), but extended with `?`
("where") and `!` ("where not") operators.  Our JSONPath syntax also supports
the `[?(expr)]` and `[(expr)]` constructs to insert your own C/C++ expressions
for filtering and selection of nodes in JSONPath queries.  Our syntax also
supports multiple comma-separated alternatives in the `[ ]` selector.  But the
syntax does not support `|` (alternation).  We recommend to write a JSONPath
query for each alternation.

JSON data structures are represented internally as a node graph consisting of
atomic values (null, bool, int/double, string), arrays, and "objects".  JSON
objects are structs with fieldname-value pairs.  A JSONPath expression
specifies a JSON data query, typically starting from the root node, and
descending deeper into the node graph to match child nodes.

For example, suppose we have a `store` object with a `book` array.  Each `book`
object has a `title` string and some other properties which we will ignore for
now.  The following JSONPath query returns the titles of all books in the
store:

    $.store.book[*].title

We can also write the same query in bracket notation:

    $["store"]["book"][*]["title"]

Note that the syntax of this query has a close similarity to the C++ JSON API
for accessing object fields (a.k.a. object properties) and array elements.

Basically, a JSONPath expression is a sequence of operations to match nodes:

| Operator      | Nodes matched and returned                                   |
| ------------- | ------------------------------------------------------------ |
| `$`           | the root node of the node graph                              |
| `.f` or `[f]` | child node at field named `f` of the current object node     |
| `[n]`         | nth node of the current array node, if indexed within bounds |
| `[b:e:s]`     | array slice of the current array node                        |
| `[x,y]`       | child nodes matching `x` or `y` (fields, indices and slices) |
| `*`           | "wildcard": any child node of the current object/array node  |
| `..`          | "recurse": any matching descendant nodes of the current node |
| `?`           | "where": current node if the rest of the query path matches  |
| `!`           | "where not": the complement of `?`                           |
| `[(e)]`       | use value of C/C++ expression `e` to match a field or index  |
| `[?(e)]`      | evaluate C/C++ expression `e`, continue matching when true   |

Field names (`f` in the table) in JSON and in JSONPath queries may contain
UTF-8 Unicode characters.

Throughout this document we refer to the *field names* of objects and structs.
Also commonly used are JSON object *property names* and *key names* (as in
key-value pairs),

Other JSONPath implementations require quotes for field names in brackets, as
in `["store"]`.  With jsoncpp you will only need to add quotes when field names
contain control characters, spaces, or punctuation, such as a `unit-price`
field name in the query `$..["unit-price"]`.  To promote orthogonality of the
JSONPath syntax (no arbitrary rules and exceptions depending on a context),
quoted field names are also valid with dot notation in our JSONPath syntax,
such as the query `$.."unit-price"`.

JSONPath by example                                                 {#jsoncpp-4}
-------------------

A JSONPath query expression uses dot or bracket operators to match JSON data
located at increasingly deeper levels of the data structure.

Consider the following JSON data:

<div class="alt">
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    {
      "store": {
        "book": [ 
          {
            "category": "reference",
            "author": "Nigel Rees",
            "title": "Sayings of the Century",
            "price": 8.95
          },
          {
            "category": "fiction",
            "author": "Evelyn Waugh",
            "title": "Sword of Honour",
            "price": 12.99
          },
          {
            "category": "fiction",
            "author": "Herman Melville",
            "title": "Moby Dick",
            "isbn": "0-553-21311-3",
            "price": 8.99
          },
          {
            "category": "fiction",
            "author": "J. R. R. Tolkien",
            "title": "The Lord of the Rings",
            "isbn": "0-395-19395-8",
            "price": 22.99
          }
        ],
        "bicycle": {
          "color": "red",
          "price": 19.95
        }
      }
    }
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
</div>

To match the `title` of the first book (`book[0]`) in a `store`, starting at
the root node indicated by `$`, we use the following JSONPath query expression:

    $.store.book[0].title

This query returns `"Sayings of the Century"` when applied to the JSON data.

To try the JSONPath examples yourself, we suggest to create a `store.json` file
with the above JSON data and run jsoncpp from the command line to compile a
JSONPath query as follows:

    cd gsoap/samples/xml-rpc-json
    soapcpp2 -CSL xml-rpc.h
    jsoncpp -o test-json.cpp -m -p'$.store.book[0].title'
    c++ -I../.. -o test-json test-json.cpp json.cpp xml-rpc.cpp soapC.cpp ../../stdsoap2.cpp
    ./test-json < store.json

The compiled JSONPath query is applied to the `store.json` data and returns the
matching values found.  Use jsoncpp option `-y` to return matches in a JSON
array.

The `soapC.cpp` file, and the header files `soapStub.h` and `soapH.h`, are
generated with the command `soapcpp2 -CSL xml-rpc.h`, which is done just once
for all C++ JSON applications.  Use soapcpp2 option `-c` to generate `soapC.c`
for C (with the corresponding `soapStub.h` and `soapH.h` in C).

To match any field of an object or any array element, we use a wildcard `*`:

    $.store.*.price

This matches and returns the bicycle price `19.95`, but not the book prices
that are located one level deeper in the array of books, which can be matched
with:

    $.store.*.*.price

This returns the values `8.95`, `12.99`, `8.99`, and `22.99`.

Note that in the latter case we only get the book prices, because the first `*`
matches `book` and `bicycle` and the second `*` matches the book array and the
`red` and `price` fields.  Only the book prices are returned, because `red` and
`price` are atomic and have no `price` child node.

To match and return all prices in the store we use `..` called "recursive
descent" or simply "recurse", as follows:

    $..price

Array elements are matched with brackets `[n]` where `n` is an array index.
Negative indices can be used to access array elements from the end of an array,
where -1 refers to the last element.  We can list the array elements to match
with `[x,y]`, for example:

    $.store.book[0,1,-1].title

This matches and returns the titles of the first two books and the last.

JSONPath queries do not modify the node graph searched.  So you do not need to
worry about indices that are out of bounds or fields that are not part of an
object.

Arrays can also be sliced for matching from a starting index `b` until
(excluding) an ending index `e` with `[b:e]`, where `b` and `e` values are
optional.  When omitted, the slice runs from the start and/or from the end of
the array.

For example:

    $.store.book[:].title

matches and returns the titles of all books in the store, and

    $.store.book[:2].title

matches and returns the first two books (at 0 and 1) in the store.

We can use an optional step `s` to slice arrays with `[b:e:s]` and even reverse
array element-by-element matching with a negative unit step:

    $.store.book[::-1].title

This matches and returns the titles of all books in reverse order.

The following JSONPath queries return the same results for the example JSON
data, where we used slices and `[x,y]` to match multiple array entries:

    $.store.book[1:3].title
    $.store.book[1:-1].title
    $.store.book[-3:-1].title
    $.store.book[1,2].title
    $.store.book[-3,-2].title

Basically, JSONPath array slices in our implementation follow the intuitive
Python array slice syntax and meaning.  Beware that many other JSONPath
implementations do not implement the step parameter consistently or do not
support stepping.

Note that `[:]` is not the same as `[*]` because `[:]` only matches arrays.

A conditional JSONPath expression contains a `?` ("where") operator.  The
operator returns the results that match the left side of the `?` but only when
the right-side matches:

    $.store.book[:]?isbn

This matches and returns only books that have an `isbn` field.

The complement of the `?` operator is `!` ("where not"), which returns the
results that match the left side of the `!` but only when the right-side does
not match.

More complex queries can be formulated by embedding C/C++ expressions in the
query to filter `[?(e)]` and select `[(e)]` nodes.  For example, in C++:

    $.store.book[:][?((double)v["price"] < 10.0)].title

and in C:

    $.store.book[:][?(*double_of(value_at(v, "price")) < 10.0)].title

This filters books with prices lower than 10.0 and returns the title of each
book found.

Embedded C/C++ expressions to filter nodes can inspect the current JSONPath
node value by accessing variable `v`, as is shown above.  Here we used
`(double)v["price"]` to obtain the price of the current node for comparison.
Besides the current node `v` you can also access the JSONPath root node value
`x`.  Instead of the default name `x`, you can change `x` to another name with
jsoncpp option `-r`.

You can access variables and functions in embedded C/C++ expressions, but do
not access or modify `i`, `j`, `k`, `p`, `q`, `r`, `s` and `S`, which are
internally used by the generated JSONPath query code.

@warning In this respect we should caution you about using C/C++ expressions
that modify node values, since this may affect the query results in
unpredictable ways.  In fact, `v["price"]` will add a price to any current node
value `v` that has no `"price"` field!  To make field accesses safe we should
first check if the field exists in the current node before we access it:

    $.store.book[:][?((v.has("price") ? (double)v["price"] : 9999) < 10.0)].title

@warning Guarding field accesses with `has()` is the only safe way to combine
`..` with C/C++ filters, since we may visit all nodes in the graph, for example
to find all prices < 10.0:

    $..[?((v.has("price") ? (double)v["price"] : 9999) < 10.0)].price

Object fields and array elements can be accessed in a JSONPath query with C/C++
expressions that evaluate to string field names or to integers indices,
respectively.  For example, we can use the string `argv[1]` of `main()` as a
field name in C++:

    $.store.book[:][(argv[1])]

In C we have to explicitly use `value_at` to access the field of the current
`v` node (and we use `nth_value` to access array elements of the current `v`
node, not shown here):

    $.store.book[:][(value_at(v, argv[1]))]

This assumes that the command-line argument (`argv[1]`) of the application is a
book field name.  Otherwise, no results are returned.

After compiling the JSONPath query with

    jsoncpp -o test-json.cpp -m -p'$.store.book[:][(argv[1])]'
    c++ -I../.. -o test-json test-json.cpp json.cpp xml-rpc.cpp soapC.cpp ../../stdsoap2.cpp

we can obtain the book titles with:

    ./test-json title < store.json

You can use multiple C/C++ expressions in brackets and combine them with other
field and array expressions separated by commas:

    $.store.book[:][title,(argv[1])]

This prints the title and the value of the field name given by the command-line
argument, if there is a field that matches the given name.

Finally, let's use the value of `argv` to filter products in the store by a
given command-line argument price:

    jsoncpp -m -p'$.store..[?((v.has("price") ? (double)v["price"] : 9999) < strtod(argv[1], NULL))]'

And in C:

    jsoncpp -c -m -p'$.store..[?((nth_at(v, "price") >= 0 ? *double_of(value_at(v, "price")) : 9999) < strtod(argv[1], NULL))]'

C/C++ expressions cannot be used as array slice bounds, which must be constant.

C++ XML-RPC and JSON                                                      {#cpp}
====================

XML-RPC and JSON data is interchangeable in this implementation, with the only
exception that the dateTime and base64 types are handled as strings in JSON.
Also, JSON's only numeric type is floating point.  However, integers are handled
just fine by this JSON implementation as 64 bit (i.e. `long long`, `int64_t`,
`LONG64`) without internal conversion to/from double floating point values
that could cause a loss of precision for large values.

List of files                                                       {#cpp-files}
-------------

The following files define XML-RPC operations and data types for C++:

- `xml-rpc-io.h`:       XML-RPC serialization over streams
- `xml-rpc-io.cpp`:     XML-RPC serialization over streams
- `xml-rpc-iters.h`:    iterators for structs, arrays, and XML-RPC parameters
- `xml-rpc.cpp`:        XML-RPC C++ data binding API
- `xml-rpc.h`:          XML-RPC data binding as a gSOAP .h file for soapcpp2
                        (do not #include this file in your project builds)

For JSON we use the following files for C++:

- `json.h`:             JSON C++ API and JSON serialization over streams
- `json.cpp`:           JSON C++ API and JSON serialization over streams
- `xml-rpc.cpp`:        XML-RPC C++ data binding API
- `xml-rpc-iters.h`:    iterators for structs/objects and arrays
- `xml-rpc.h`:          XML-RPC data binding as a gSOAP .h file for soapcpp2
                        (do not #include this file in your project builds)

The gSOAP header file `xml-rpc.h` defines all XML-RPC and JSON types as struct
with C++ member functions to create XML-RPC and JSON data and REST messages.

A note about the following auto-generated files: `soapH.h`, `soapStub.h` and
`soapC.cpp`.  These are required for XML-RPC and JSON.  To auto-generate these
files, execute:

    soapcpp2 -CSL xml-rpc.h

Then compile and link the `.cpp` files listed above for XML-RPC and JSON with
the auto-generated `soapC.cpp` and `stdsoap2.cpp`:

    c++ -I../.. -o myapp myapp.cpp json.cpp xml-rpc.cpp soapC.cpp ../../stdsoap2.cpp

Instead of `stdsoap2.cpp` you can link `libgsoap++.a` with `-lgsoap++`, when
installed by the gSOAP package.

To enable OpenSSL for HTTPS compile with `-DWITH_OPENSSL` and link
`-lssl`, and `-lcrypto`:

    c++ -DWITH_OPENSSL -I../.. -o myapp myapp.cpp json.cpp xml-rpc.cpp soapC.cpp ../../stdsoap2.cpp -lssl -lcrypto

For OpenSSL support, instead of `stdsoap2.cpp` you can link `libgsoapssl++.a`
with `-lgsoapssl++`, when installed by the gSOAP package.

Because XML namespaces are not used, we can either use `-DWITH_NONAMESPACES` to
compile `stdsoap2.cpp` without complaining about a missing global `Namespace`,
or we can define an empty namespaces table somewhere in our code:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    struct Namespace namespaces[] = {{NULL,NULL,NULL,NULL}};
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Overview                                                               {#cpp-ex}
--------

An XML-RPC/JSON data value is created in C++ as follows, which requires a
context `ctx` with the engine state (the soap struct).  The context manages the
memory that is internally allocated to hold values:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    #include "soapH.h"

    soap *ctx = soap_new1(SOAP_C_UTFSTRING);  // new context

    value v(ctx);       // a new local value v

    soap_destroy(ctx);  // delete all values
    soap_end(ctx);      // delete temp data
    soap_free(ctx);     // free context
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Note that `soapH.h` is an auto-generated file (see previous section).  This file
needs to be generated only once and for all.  It also references `stdsoap2.h`
and the auto-generated `soapStub.h`.  Compile and link your code with
`stdsoap2.cpp` and the auto-generated `soapC.cpp` XML-RPC serializers.  Also
compile and link `xml-rpc.cpp`.  For JSON, compile and link `json.cpp`.

We can stack-allocate local values as shown above.  To allocate a value on the
heap that is managed by the engine context, use `new_value(ctx)`:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    #include "soapH.h"

    soap *ctx = soap_new1(SOAP_C_UTFSTRING);  // new context

    value *v = new_value(ctx);

    soap_destroy(ctx);  // delete all values
    soap_end(ctx);      // delete temp data
    soap_free(ctx);     // free context
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Allocating values on the heap may throw `std::bad_alloc` exceptions, but
allocating a context does not.  To safeguard your code use a try-catch:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    #include "soapH.h"

    soap *ctx = soap_new1(SOAP_C_UTFSTRING);  // new context (does not throw)
    if (ctx == NULL)
      ...               // handle out-of-memory error

    try {

      value *v = new_value(ctx);
      ...               // create other values

    }
    catch (std::bad_alloc& e)
    {
      ...               // handle out-of-memory error
    }

    soap_destroy(ctx);  // delete all values
    soap_end(ctx);      // delete temp data
    soap_free(ctx);     // free context
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can use wide strings with Unicode stored in UTF-8-formattted 8-bit `char`
strings.  For compatibility with XML-RPC serialization of UTF-8-encoded strings,
we MUST use the `SOAP_C_UTFSTRING` flag to initialize the context with
`soap_new1(SOAP_C_UTFSTRING)`.  We can optionally use `SOAP_XML_INDENT` to
indent XML and JSON.

The code shown above creates an empty value `v`.  Values can be assigned any one
of the following data types:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    v = 12345LL;          // 64 bit int

    v = 12.34;            // double float

    v = "abc";            // string

    v = string("abc");    // std::string

    v = L"xyz";           // wide string (converted to UTF-8)

    v = wstring(L"xyz");  // std::wstring (converted to UTF-8)

    v = false;            // Boolean

    v = (ULONG64)time(0); // ULONG64 values are serialized as ISO 8601 date time

    // create an array [24, 99.99, "abc"]
    v[0] = 24;
    v[1] = 99.99;
    v[2] = "abc";

    // copy a C array or a C++ container to a JSON array
    int ints[] = { 1, 2, 3 };
    v.size(3);            // array size of JSON value v must conform
    std::copy(ints, ints + 3, v.begin());

    // create a struct (JSON object) {"name": "gsoap", "major": 2.8, "©": 2015}
    v["name"]  = "gsoap";
    v["major"] = 2.8;
    v[L"©"]    = 2015;    // wide string tags are OK

    // create a base64-encoded image object
    _base64 img(ctx, 100, ptr_to_rawimage100bytes); // block of 100 raw bytes
    v = img;
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

We can combine these operations in many possible ways to create arrays of
arrays, arrays of structs, and so on.  For example:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    v[0]["name"] = "bob";
    v[0]["toys"][0] = "ball";
    v[0]["toys"][1] = "furby";
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This creates a singleton array containing an object with two members: `name`
set to `"bob"` and `toys` set to an array containing `"ball"` and `"furby"`.
In JSON format this is represented as:

               [ { "name": "bob", "toys": ["ball", "furby"] } ]
               ^ ^ ^              ^
               | | |              |
      an array_/ | |              |
     of 1 struct_/ |              |
    with 2 members_/______________/

Let's see what happens when we assign a variable the value of another.

The JSON C++ API uses the *value model* for variables with atomic values,
meaning that atomic values are copied when assigning a target variable the
atomic value of another variable.  It uses the *reference model* for arrays and
structs, meaning that array and struct contents are shared when assigning a
target variable the array/struct value of another variable.  We illustrate the
effect below:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    value x(ctx), y(ctx);
    x = 1;
    y = x;
    x = 2;
    std::cout << "x = " << x << " and y = " << y << std::endl;
    value a(ctx), b(ctx);
    a["num"] = 1;
    b = a;
    a["num"] = 2;
    std::cout << "a.num = " << a["num"] << " and b.num = " << b["num"] << std::endl;
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This prints `x = 2 and y = 1` and `a.num = 2 and b.num = 2`.

You can make deep copies of values by using the auto-generated `soap_dup_value`
function in `soapC.cpp`:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    value a(ctx), b(ctx);
    a["num"] = 1;
    soap_dup_value(ctx, &b, &a);
    a["num"] = 2;
    std::cout << "a.num = " << a["num"] << " and b.num = " << b["num"] << std::endl;
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This prints `a.num = 2 and b.num = 1`.

@warning Do not extend arrays and structs (JSON objects) returned by
`soap_dup_value` with new entries.  The new entries may use unallocated memory.
You can always loop over arrays and structs to copy their contents to a new
array or struct managed by another context, if you must add new entries later.

To auto-generate `soap_dup_value`, use option `-Ec` with soapcpp2:

    soapcpp2 -Ec -CSL xml-rpc.h

With gSOAP 2.8.28 and later you can use the following operators on values in
C++:

| Operator             | Type of JSON Value Operands         | Result after Operand Type Promotion             |
| -------------------- | ----------------------------------- | ----------------------------------------------- |
| `+`                  | bool, number, string, struct, array | sum, string concat, struct concat, array concat |
| `-`, `*`, `/`, `%`   | bool, number                        | difference, product, division, modulo           |
| `==`, `!=`           | bool, number, string, struct, array | C++ `bool`                                      |
| `<=`, `>=`, `<`, `>` | bool, number, string                | C++ `bool`                                      |

The (un)equal operators compare the two values by deep value comparison of
array elements and object fields and values.  For the arithmetic operations,
operands are converted by type promotion until the two operands conform to the
type required for the operation:

    atomic   int    double    string    struct    array

    false -> 0
    true  -> 1
             int -> double
    null -------------------> "null"
    false ------------------> "false"
    true -------------------> "true"
             int -> double -> string
    null ---------------------------------------> [null]
    false --------------------------------------> [false]
    true ---------------------------------------> [true]
             int -------------------------------> [int]
                    double ---------------------> [double]
                              string -----------> [string]
                                        struct -> [struct]

An example:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    #include <sstream>
    std::stringstream ss("[0,1,2]");

    // parse JSON array into 'x'
    value x(ctx);
    ss >> x;            // x = [0,1,2]
    x[3] = x[1] + x[2]; // x = [0,1,2,3]

    value y(ctx);
    y[0] = "abc";       // y = ["abc"]
    y[1] = y[0] + 123;  // y = ["abc","abc123"]

    std::cout << x + y; // [0,1,2,3] + ["abc","abc123"] is [0,1,2,3,"abc","abc123"]
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Arithmetic and concatenation operations may incur significant memory overhead
due to temporaries, type conversions (when applicable), and managed heap
storage.  Use them only when CPU and memory usage are not critical.

When receiving a value in XML-RPC/JSON, we generally want to check its type to
obtain its value.  To check the type of a value, we use `is_Type` methods:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    bool value::is_null()     // true if value is not set (JSON null)
    bool value::is_bool()     // true if value is a Boolean "true" or "false" value
    bool value::is_true()     // true if value is Boolean "true"
    bool value::is_false()    // true if value is Boolean "false"
    bool value::is_number()   // true if value is a number (int or float)
    bool value::is_int()      // true if value is a 32 or a 64 bit int
    bool value::is_double()   // true if value is a 64 bit double floating point (not integer)
    bool value::is_string()   // true if value is a string or wide string
    bool value::is_dateTime() // true if ISO 8601, always false for received JSON
    bool value::is_array()    // true if array of values
    bool value::is_struct()   // true if structure, a.k.a. a JSON object
    bool value::is_base64()   // true if base64, always false for received JSON
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When working with JSON data, do not use `is_double()` but `is_number()`.  JSON
data does not differentiate between integers and floats.  However, gSOAP makes
sure that 64 bit integer values are accurately represented in JSON and decoded
without loss from JSON.  That is, when receiving an integer that you checked
with `is_int()` you can then safely cast value with e.g.  `int64_t n = v`.

The following methods can be used to inspect arrays and structs (JSON objects):

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    void value::size(int)                  // reset array size or pre-allocate space
    int value::size()                      // returns array or struct size or 0
    bool value::empty()                    // true if array or struct is empty
    bool value::has(int)                   // true if index is within array bounds
    bool value::has(const char*)           // true if struct has field
    bool value::has(const wchar_t*)        // true if struct has field
    int value::nth(int)                    // returns index >= 0 if index is in array bounds, < 0 otherwise
    int value::nth(const char*)            // returns index >= 0 of field in struct, < 0 otherwise
    int value::nth(const wchar_t*)         // returns index >= 0 of field in struct, < 0 otherwise
    value& value::operator[int]            // returns value at index in array or struct
    value& value::operator[const char*]    // returns value at field in struct
    value& value::operator[const wchar_t*] // returns value at field in struct
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For example, let's take the value `v` that was assigned the array shown above.
We have the following properties of this value:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    v.is_null() == false             // v is not null
    v.is_array() == true             // v is an array
    v.size() == 1                    // v has one element
    v.has(1) == false                // v has no array element at index 1
    v.nth(-1) == 0                   // v last element is at index 0
    v[0].is_struct() == true         // v[0] is a struct
    v[0].has("name") == true         // v[0] has field name "name"
    v[0].nth("name") == 0            // v[0] has field name "name" at index 0
    v[0][0].is_string() == true      // v[0][0] == v[0]["name"] is a string
    v[0].has("toys") == true         // v[0] has field name "toys"
    v[0]["toys"].is_array() == true  // v[0]["toys"] is an array
    v[0]["toys"].empty() == false    // v[0]["toys"] is not empty
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When accessing structs (JSON objects) by field name, make sure to use existing
member field names only.  A new member fieldname-value pair is dynamically
added to the structure to accomodate the new entry for the field.

Also arrays are extended to accommodate the indexed array element.  A negative
index accesses elements from the array's end, with index -1 accessing the last
value.  Also the `value::has(int)` and `value::nth(int)` methods take a
negative index for bounds checking on arrays and will return `false` or
negative, respectively.

Use `value::size(int)` to change array size or set arrays to zero length.  Use
negative size with `value::size(int)` to remove elements from the end.

You may want to use iterators to retrieve data from structs and arrays (see
further below).

To retrieve atomic data we can use casts on a value `v` as follows:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    (double)v                   // 64 bit double or 0.0 if not numeric
    (int)v                      // 32 bit int or 0 if not numeric
    (LONG64)v                   // 64 bit int or 0 if not numeric
    (char*)v                    // convert to string
    (string)v                   // convert to std::string
    (wchar_t*)v                 // convert to wide string
    (wstring)v                  // convert to std::wstring
    (bool)v                     // same as is_true()
    (ULONG64)v                  // nonzero time_t if v contains an ISO 8601 date time
    (_base64)v                  // base64 encoding of v
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

It is valid to cast a primitive type to any of the other primitive types shown
above and no runtime error will occur, although you may loose some information
when an atomic value has no representation in the target type's value space.
Casting a number to a string is fine, but casting a string to a number only
gives a nonzero numeric value if the string is numeric.  Casting a value to
base64 produces its base64 encoding.

To access base64 binary raw data of a value `v`, we use the following methods:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    _base64& base64 = v;
    unsigned char *raw = base64.ptr();  // points to raw binary data
    int size = base64.size();           // that is of this size
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Arrays and structs are compound types that cannot be cast to or from other
types (but if you do cast, an empty array or struct is created an no runtime
error will occur).  So we should check for array and struct types to obtain
their content.  For example:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    if (v.is_array())
    {
      for (int i = 0; i < v.size(); ++i)
      {
        value& array_value = v[i];
        ... // use and/or set array_value
      }
    }
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

We use the iterators `value::iterator` and `value::const_iterator` to loop over
the values in structs and arrays:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    if (v.is_struct() || v.is_array())
    {
      for (value::iterator i = v.begin(); i != v.end(); ++i)
      {
        int index = i.index();              // index of element
        const char *name = i.name();        // name() is nonempty for structs
        value& element = *i;
        ... // use index, name, and/or use/set the element value
      }
    }
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The iterator method `value::iterator::index()` returns the integer index of the
struct or array element.  The `value::iterator::name()` method returns the name
of the struct member, or empty `""` if the type is not a struct.  The
`value::const_iterator` does not permit the value referenced by the iterator to
be modified.

There are two lower level iterators for structs and arrays, which are slightly
more efficient to use compared to the `value::iterator`.  These array and
struct iterators have an `index()` method to obtain the index (an int).  Struct
iterators have a `name()` method to obtain a member's name (a string).

For example, to traverse a value `v` that is an array or a struct:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    if (v.is_array())
    {
      _array& vec = v;  // cast to _array
      for (_array::iterator i = vec.begin(); i != vec.end(); ++i)
      {
        int array_index = i.index();
        value& array_value = *i;
        ... // use array_index, use and/or set array_value
      }
    }
    else if (v.is_struct())
    {
      _struct& rec = v;  // cast to _struct
      for (_struct::iterator i = rec.begin(); i != rec.end(); ++i)
      {
        const char *member_name = i.name();
        value& member_value = *i;
        ... // use member_name, use and/or set member_value
      }
    }
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

As usual, the `_struct::const_iterator` and `_array::const_iterator` do not
permit the values referenced by the iterator to be modified.

XML-RPC parameter lists are similar to arrays and its values are indexed.  We
can also iterate over response parameters after an XML-RPC REST call:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    methodCall rpc(ctx, "endpoint URL", "methodName");
    params request(ctx, 2);
    request[0] = ...;                // first request parameter
    request[1] = ...;                // second request parameter
    params response = rpc(request);  // execute the call
    if (!rpc.error())
    {
      for (params::iterator i = response.begin(); i != response.end(); ++i)
      {
        int index = i.index();
        value& param_value = *i;
        ... // use param_value of response params
      }
    }
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

We should note that JSON REST does not require parameter types, for example:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    #include "json.h"
    value request(ctx);
    value response(ctx);
    request[0] = ...;                // first request parameter
    request[1] = ...;                // second request parameter
    if (!json_call(cts, "endpoint URL", request, response))
    {
      ... // use response value
    }
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

There are two additional methods to invoke on parameters:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    int params::size();         // returns number of parameters
    bool params::empty();       // true if no parameters
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

All dynamically allocated memory that is internally used to store data is
deallocated with:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    soap_destroy(ctx);  // delete all values
    soap_end(ctx);      // delete temp data
    soap_free(ctx);     // delete context allocated with soap_new()
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Additional C++ examples are located in `gsoap/samples/xml-rpc-json`:

- `xml-rpc-currentTime.cpp`:            XML-RPC C++ client, also uses JSON
- `xml-rpc-currentTimeServer.cpp`:      XML-RPC C++ server (CGI and multi-threaded stand-alone)
- `xml-rpc-weblogs.cpp`:                XML-RPC C++ client
- `xml-rpc-json.cpp`:                   XML-RPC to/from JSON example
- `json-currentTime.cpp`:               JSON REST C++ client
- `json-currentTimeServer.cpp`:         JSON REST C++ server (CGI and multi-threaded stand-alone)
- `json-GitHub.cpp`:                    JSON REST C++ client for GitHub API v3

C++ XML-RPC client example                                             {#cpp-cl}
--------------------------

A typical XML-RPC calling sequence in C++ is:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    #include "soapH.h"      // generated by the command: soapcpp2 -CSL xml-rpc.h
    #include "xml-rpc-io.h" // to serialize XML-RPC data to streams
    struct Namespace namespaces[] = {{NULL,NULL,NULL,NULL}}; // no XML namespaces
    // create a context
    soap *ctx = soap_new1(SOAP_C_UTFSTRING); // store Unicode in UTF-8 format
    ctx->send_timeout = 10; // 10 sec, stop if server is not accepting msg
    ctx->recv_timeout = 10; // 10 sec, stop if server does not respond in time
    // create an XML-RPC method call object
    methodCall rpc(ctx, "endpoint URL", "methodName");
    // populate the parameters
    rpc[0] = 123;           // first parameter is an integer
    rpc[1] = "abc";         // second is a string
    rpc[2]["name"] = "joe"; // a record, first member "name"
    rpc[2]["age"] = 23;     // a record, second member "age"
    rpc[3][0] = 456.789;    // an array, first element (a float)
    rpc[3][1] = "widget";   // an array, second element (a string)
    rpc[3][2] = true;       // an array, third element (a bool)
    // make the XML-RPC call and retrieve response
    params response = rpc(request);
    // check result
    if (rpc.error())
      soap_stream_fault(ctx, std::err);
    else if (response.empty())
      std::cout << rpc.fault() << std::endl;
    else if (response.size() > 1)
      std::cout << "More than one response data" << std::endl;
    else if (response[0].is_array() && !response[0].empty())
      for (int i = 0; i < response[0].size(); i++)
        ... = response[0][i];
    else if (response[0].is_struct())
    {
      ... = response[0]["membername1"];
      ... = response[0]["membername2"];
    }
    else if (response[0].is_base64())
    {
      _base64 base64& = response[0];
      unsigned char *raw = base64.ptr();
      int size = base64.size();
      ... // use raw[0..size-1] data
    }
    else if (response[0].is_bool())
    {
      bool flag = response[0];
      ... // use boolean flag
    }
    else if (response[0].is_int())
    {
      LONG64 num = response[0];
      ... // use integer
    }
    else if (response[0].is_double())
    {
      double num = response[0];
      ... // use double float
    }
    else if (response[0].is_string())
    {
      const char *str = response[0];
      // use string, note that also legal is:
      const std::string& st = response[0];
      // and conversion from UTF-8 to wide string unicode:
      const wchar_t *w = response[0];
      const std::string& ws = response[0];
    }
    else if (response[0].is_dateTime())
    {
      time_t t = (ULONG64)response[0];
      ... // use time
    }
    // deallocate all
    soap_destroy(ctx);
    soap_end(ctx);
    soap_free(ctx);
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Alternatively, parameters of a methodCall can be passed with the methodCall
itself as follows:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    methodCall rpc(ctx, "endpoint URL", "methodName");
    // create 5 parameters
    params request(ctx, 5);
    // populate the parameters
    request[0] = 123;           // first parameter is an integer
    request[1] = "abc";         // second is a string
    request[2]["name"] = "joe"; // a record, first member "name"
    request[2]["age"] = 23;     // a record, second member "age"
    request[3][0] = 456.789;    // an array, first element (a float)
    request[3][1] = "widget";   // an array, second element (a string)
    request[3][2] = true;       // an array, third element (a bool)
    // make the XML-RPC call with request parameters, retrieve response
    params response = rpc(request);
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Note that in the client code, after the response is retrieved, the implicit
type casts done by assignments will retrieve the values that are type-cast
converted if needed.  These casts can be used anywhere to retrieve values:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    params response = rpc();
    double sum = 0.0;
    for (int i = 0; i < response.size(); i++)
      if (response[i].is_number())    // is this parameter an int or float?
        sum += (double)response[i];
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Type casts can also be used to convert data, which means they never produce an
exception.  Casting to string `(const char*)` converts atomic values to strings,
  but does not convert compound types such as arrays and structs.

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    params response = rpc();
    for (int i = 0; i < response.size(); i++)
      printf("response[%d] = %s\n", i, (const char*)response[i]);
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

which prints a string representation of the int, double, boolean, time, or
base64 values of parameters.  An empty string is printed for arrays and structs.
Use iterators to walk over arrays and structs to print values.  Or use the
JSON API `json.h` and `json.cpp` to print values in JSON format, see further on
JSON below.

C++ XML-RPC server example                                             {#cpp-sr}
--------------------------

A typical C++ XML-RPC server sequence is:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    // create an allocation context
    soap *ctx = soap_new1(SOAP_C_UTFSTRING);
    // create an XML-RPC methodCall object
    methodCall rpc(ctx);
    // Option 1: parse and write to/from stdin/out for CGI
    // (no code needed)
    // Option 2: parse and write to/from FILE or socket
    // ctx->recvfd = ...; // set input file descriptor
    // ctx->sendfd = ...; // set output file descriptor
    // Option 3: parse and write to/from IO streams
    // ctx->is = ...; // set input stream
    // ctx->os = ...; // set output stream
    if (rpc.recv() != SOAP_OK)
      soap_print_fault(ctx, stderr);
    else
    {
      // create response
      methodResponse response(ctx);
      // check method name
      if (!strcmp(rpc.name(), "methodName"))
      {
        // method name matches: populate response parameters with values:
        response[0] = ...;
        response[1] = ...;
        ... // add response data 
      }
      else
      {
        // otherwise, set fault
        response.set_fault("Wrong method");
      }
      // send response
      if (response.send() != SOAP_OK)
        soap_print_fault(ctx, stderr);
    }
    // close (but keep-alive setting keeps socket open)
    soap_closesock(ctx);
    // clean up
    soap_destroy(ctx);
    soap_end(ctx);
    soap_free(ctx);
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

With option 1 the server code above uses standard in/out and thus runs over
CGI.  Other possibilities are given by options 2 and 3.

To serve requests at a port, we use the `soap_bind()` and `soap_accept()` calls
to bind the server to a port and accept requests via socket, see also the docs
and examples for these calls (see for example `gsoap/samples/webserver.c`):

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    // create an allocation context
    soap *ctx = soap_new1(SOAP_IO_KEEPALIVE | SOAP_C_UTFSTRING);
    // bind to port 8080
    if (!soap_valid_socket(soap_bind(ctx, NULL, 8080, 100)))
      ... // error, stop
    // accept messages in server loop
    for (;;)
    {
      if (!soap_valid_socket(soap_accept(ctx)))
        ... // error, stop
      // create a method object
      methodCall rpc(ctx);
      // parse it from socket
      if (rpc.recv() != SOAP_OK)
        soap_print_fault(ctx, stderr);
      ... // process request, produce result to send as shown above
      // close (but keep-alive setting keeps socket open)
      soap_closesock(ctx);
      // clean up
      soap_destroy(ctx);
      soap_end(ctx);
    }
    // free context
    soap_free(ctx);
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

C++ XML-RPC serialization from/to streams                              {#cpp-io}
-----------------------------------------

To send and receive XML over streams, use `xml-rpc-io.h` and `xml-rpc-io.cpp`.
For example:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    #include "xml-rpc-io.h"
    std::cout << response[0] << std::endl;
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

which will display the data in XML-RPC format.  To parse XML-RPC data from a
stream, use:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    #include "xml-rpc-io.h"
    value v(ctx);
    std::cin >> v;
    if (ctx->error) ... // check for parse errors (can also use v.soap->error)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Compile and link together with `soapC.cpp`, `xml-rpc.cpp`, `xml-rpc-io.cpp`,
and `stdsoap2.cpp`.

C++ JSON serialization from/to streams                                 {#cpp-js}
--------------------------------------

To display values in JSON format or to parse JSON data, use the `json.h` and
`json.cpp` JSON serializers in combination with `xml-rpc.cpp` and the
auto-generated `soapH.h` and `soapC.cpp`.  It is also possible to send and
receive JSON data over HTTP as JSON REST operations, but this requires some
more coding (see \ref cpp-jr below).

Because the internal data is the same for XML-RPC and JSON, You can write data
in XML-RPC or in JSON format.  You can also parse XML-RPC data and write to JSON
data and vice versa.

To write JSON to a stream you can use the `<<` operator on an output stream and
a JSON value.

To parse JSON from a stream you can use the `>>` operator on an input stream
and populate a JSON value.

For example, you can parse a JSON-formatted string and use that data to make an
XML-RPC call.  The result of the call is displayed in JSON, nicely indented
using the `SOAP_XML_INDENT` flag (this XML indent flag also works for JSON):

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    #include "json.h"
    #include <sstream>
    struct Namespace namespaces[] = {{NULL,NULL,NULL,NULL}}; // no XML namespaces

    // SOAP_C_UTFSTRING: UTF-8 content in char*, SOAP_XML_INDENT: indent JSON
    soap *ctx = soap_new1(SOAP_C_UTFSTRING | SOAP_XML_INDENT);
    value request(ctx);
    std::istringstream in;
    in.str("[ [1, \"2\", 3.14, true], {\"name\": \"john\", \"age\": 24} ]");
    in >> request;                      // parse JSON, store as XML-PRC data
    if (ctx->error) ...                 // check for parse errors
    params response = rpc(request);     // make the XML-RPC call
    std::cout << response << std::endl; // display result in JSON (indented)
    if (ctx->error) ...                 // check for write errors
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To force reading and writing JSON in ISO 8859-1 format, use the
`SOAP_ENC_LATIN` flag to set the context (not recommended).

Optionally use `SOAP_XML_INDENT` to indent XML and JSON.

To parse JSON values from a file or any other non-HTTP source, it is
recommended to set the `SOAP_ENC_PLAIN` context flag to prevent the parser from
attempting to read a MIME or HTTP headers (HTTP headers are required with
JSON-RPC):

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
    soap *ctx = soap_new1(SOAP_C_UTFSTRING | SOAP_XML_INDENT | SOAP_ENC_PLAIN);
    value v(ctx);
    in >> v;       // SOAP_ENC_PLAIN: read JSON without parsing MIME/HTTP header
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Compile and link your code together with `soapC.cpp` (generated),
`xml-rpc.cpp`, `json.cpp`, and `stdsoap2.cpp`.

The JSON protocol has fewer data types than XML-RPC, so type information can be
lost when serializing to JSON:

- XML-RPC uses a base64 type to exchange raw binary data.  The base64 data is
  converted to a string with base64 content by the JSON serializer.
  See also \ref base64.

- XML-RPC has a dateTime type, JSON does not.  The JSON serializer converts the
  dateTime type to a dateTime-formatted string.
  See also \ref dateTime.

See the section on C++ examples on how to populate and retrieve C++ data.

Strings are stored and exchanged in UTF-8 format in 8-bit strings (`char*` and
`std::string`) by using the `SOAP_C_UTFSTRING` flag.  Wide strings (i.e.
`wchar_t*` and `std::wstring` ) are converted to UTF-8.

C++ JSON REST clients and servers                                      {#cpp-jr}
---------------------------------

To use JSON REST on the client side, we use `json_call`:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    #include "json.h"            // also compile and link json.cpp
    struct Namespace namespaces[] = {{NULL,NULL,NULL,NULL}}; // no XML namespaces

    // SOAP_C_UTFSTRING: UTF-8 content in char*, SOAP_XML_INDENT: indent JSON
    soap *ctx = soap_new1(SOAP_C_UTFSTRING | SOAP_XML_INDENT);
    value request(ctx), response(ctx);
    ... // now populate the request data to send

    if (json_call(ctx, "URL", &request, &response) != SOAP_OK)) // POST
      ... // error
    else
      ... // use the response data
    ... // make other calls etc.
    soap_destroy(ctx); // delete all values
    soap_end(ctx);
    ... // make other calls etc.
    soap_free(ctx);    // free context
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A `json_call` takes a context, an endpoint URL (with URL query string
parameters as needed), and optional `in` and `out` values to send and receive,
respectively.  This function returns `SOAP_OK` (zero) for success or `EOF`,
`SOAP_SYNTAX_ERROR`, or an HTTP error code.

To use the HTTP POST method, pass both `in` and `out` values to `json_call`.
For the GET method, pass a NULL to `in`.  For the PUT method, pass a NULL to
`out`.  For the DELETE method, pass both NULL to `in` and `out`.

Besides `json_call`, there are other JSON API functions:

- `int json_call(soap *ctx, const char *URL, value *in, value *out)` makes a
  POST, GET, PUT, DELETE call, returns `SOAP_OK` or error code.
  POST method: pass both `in` and `out`.  GET method: pass a NULL to `in`.  PUT
  method: pass a NULL to `out`.  DELETE method: pass both NULL to `in` and
  `out`.  `SOAP_OK` is returned for POST and GET methods upon success.  An HTTP
  status code of 200 (OK) or 202 (Accepted) is returned for PUT and DELETE
  methods upon success.

- `int json_write(soap *ctx, const value *v)` writes JSON value to current
  file, socket, or stream.  Returns `SOAP_OK` or error.  Set file/socket file
  descriptor to write to with `ctx->sendfd = fd` (1 by default).  In C++, set
  output stream with `ctx->os = ostream` to write to.

- `int json_read(soap *ctx, value *v)` reads JSON value from current file,
  socket, or stream.  Returns `SOAP_OK` or error.  Set file/socket file
  descriptor with `ctx->recvfd = fd` to read from (0 by default).  In C++, set
  input stream with `ctx->is = istream`.

The are two other lower-level functions `json_send` and `json_recv` that are
similar to `json_write` and `json_read` but do not initialize the sending and
receiving operations and do not flush after the sending and receiving
operations.

To implement a JSON REST server for CGI (e.g. install in cgi-bin):

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    #include "json.h"            // also compile and link json.cpp
    struct Namespace namespaces[] = {{NULL,NULL,NULL,NULL}}; // no XML namespaces

    int main()
    {
      soap *ctx = soap_new1(SOAP_C_UTFSTRING | SOAP_XML_INDENT | SOAP_ENC_PLAIN);
      value request(ctx), response(ctx);
      if (soap_begin_recv(ctx)
       || json_recv(ctx, request)
       || soap_end_recv(ctx))
      {
        json_send_fault(ctx); // return a JSON-formatted fault
      }
      else
      {
        ... // use the 'request' value
        ... // set the 'response' value
        // set http content type
        ctx->http_content = "application/json; charset=utf-8";
        // send http header 200 OK and JSON response
        if (soap_response(ctx, SOAP_FILE)
         || json_send(ctx, response)
         || soap_end_send(ctx))
          soap_closesock(ctx);
      }
      // dealloc all
      soap_destroy(ctx);
      soap_end(ctx);
      soap_free(ctx);
    }
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The above server works with CGI, which is rather slow and stateless.  A
stand-alone JSON REST server is recommended.  You can also use the Apache and
IIS plugins for gSOAP to deploy JSON REST services.  See the
[documentation](https://www.genivia.com/docs.html).

Note that we use `json_send_fault()` instead of `soap_send_fault()` when an
internal error occurred, since we want the error to be reported in JSON format
as per Google JSON Style Guide.  For application-specific errors, we use
`json_send_error()` as follows:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    if (some application error occurred)
      json_send_error(ctx, 400, "Error message", "Error details");
    else
      ... // send the response
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can specify an HTTP error code such as 400 in this case, which means that a
HTTP 400 Bad Request is returned to the client.

The `json_send_error()` function is a convenient shortcut to return an error
message with an HTTP error code.  To return a JSON response with a HTTP error
code use the following:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    int status = 400; // HTTP 400 Bad Request
    soap->http_content = "application/json; charset=utf-8";
    if (soap_response(soap, SOAP_FILE + status)
     || json_send(soap, v)
     || soap_end_send(soap))
      soap_closesock(soap);
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The code for a stand-alone server is the same, except that we call `soap_bind()`
and `soap_accept()` to serve requests on a port.  Here is a simple iterative
server serving requests on port 8080:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    // create an allocation context
    soap *ctx = soap_new1(SOAP_IO_KEEPALIVE | SOAP_C_UTFSTRING);
    // bind to port 8080
    if (!soap_valid_socket(soap_bind(ctx, NULL, 8080, 100)))
      ... // error, stop
    // accept messages in server loop
    for (;;)
    {
      if (!soap_valid_socket(soap_accept(ctx)))
        ... // error, stop
      value request(ctx), response(ctx);
      if (soap_begin_recv(ctx)
       || json_recv(ctx, request)
       || soap_end_recv(ctx))
      {
        json_send_fault(ctx); // return a JSON-formatted fault
      }
      else
      {
        ... // use the 'request' value
        ... // set the 'response' value
        // set http content type
        ctx->http_content = "application/json; charset=utf-8";
        // send http header 200 OK and JSON response
        if (soap_response(ctx, SOAP_FILE)
         || json_send(ctx, response)
         || soap_end_send(ctx))
          soap_closesock(ctx);
      }
      // dealloc all
      soap_destroy(ctx);
      soap_end(ctx);
    }
    soap_free(ctx);
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Note that `SOAP_ENC_PLAIN` should be used with CGI, since we are reading from a
non-HTTP source with CGI.  We do not use this flag with gSOAP stand-alone servers.
Iterative stand-alone services are not performant and may block requests.  We
recommend stand-alone multi-threaded services.

Compile and link your code together with `soapC.cpp` (generated with
`soapcpp2 -CSL xml-rpc.h`), `xml-rpc.cpp`, `json.cpp`, and `stdsoap2.cpp`.

For C++ client and server examples, please see the gSOAP package content
`gsoap/samples/xml-rpc-json`:

- `json-currentTime.cpp`:               JSON REST C++ client
- `json-currentTimeServer.cpp`:         JSON REST C++ server (CGI and multi-threaded stand-alone)

The presentation in this section concerned stand-alone JSON REST services.  To
combine SOAP/XML with JSON REST services, see \ref soap for details.

C++ JSON-RPC clients and servers                                      {#cpp-rpc}
--------------------------------

The [JSON-RPC 1.0 specification](http://json-rpc.org/wiki/specification) (the
"original version") adds `method`, `parameter` and `id` fields to the
request message:

<div class="alt">
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    {
      "method": "echo",
      "params": [ "Hello World!" ],
      "id": 1
    }
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
</div>

where:

- `method` is a string with the name of the method to be invoked.
- `params` is an array of objects to be passed as parameters to the defined method.
- `id` is a value of any type, which is used to match the response with the request that it is replying to.

A response message has a `result` field, an `error` field, and an `id`:

<div class="alt">
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    {
      "result": "Welcome!",
      "error": null,
      "id": 1
    }
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
</div>

where:

- `result` is the data returned by the invoked method. If an error occurred while invoking the method, this value must be null.
- `error` is  a specified error code if there was an error invoking the method, otherwise null.
- `id` is the id of the request it is responding to.

The [JSON-RPC 2.0 specification](http://www.jsonrpc.org/specification) makes
all 1.0 fields REQUIRED, except for `error` which MUST NOT be present if there
was no error triggered during invocation.  The 2.0 specification adds a
`jsonrpc` field in the request message:

<div class="alt">
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    {
      "jsonrpc": 2.0,
      "method": "echo",
      "params": [ "Hello World!" ],
      "id": 1
    }
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
</div>

and also adds the `jsonrpc` field to the response message:

<div class="alt">
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    {
      "jsonrpc": 2.0,
      "result": "Welcome!",
      "id": 1
    }
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
</div>

The C++ operations are straightforward to conform to the JSON-RPC 1.0 or 2.0
specifications.  The example JSON-RPC 2.0 request message shown
above is created by the following code that uses `json_call()` to invoke a
JSON-RPC service:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
      const char *endpoint = "http://...";
      soap *ctx = soap_new1(SOAP_C_UTFSTRING | SOAP_XML_INDENT);

      value req(ctx), res(ctx);
      req["jsonrpc"]   = 2.0;
      req["method"]    = "echo";
      req["params"][0] = "Hello World!";
      req["id"]        = 1;

      if (json_call(ctx, endpoint, req, res))               // JSON-RPC call
        soap_stream_fault(ctx, std::cerr);
      else if (res.has("error") && !res["error"].is_null()) // JSON-RPC error?
        std::cerr << res["error"];
      else if ((int)res["id"] != 1)                         // matching id field?
        std::cerr << "response id != request id\n";
      else                                                  // all OK!
        std::cout << res["result"];                         // display result
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For HTTPS use an https endpoint URL.  You can control the HTTPS context as
explained in the [tutorials.](http://www.genivia.com/tutorials.html)

The server-side creates response messages similar to the example shown above.
To implement a C++ JSON-RPC and JSON REST server, please see the example
`json-currentTimeServer.cpp` located in `gsoap/samples/xml-rpc-json` in the
gSOAP package.

Embedding and serializing raw JSON data as a JSON value in C++        {#cpp-raw}
--------------------------------------------------------------

A JSON value represented by a `value` may contain raw JSON data as a string to
serialize literally in the output.  Deserializing JSON into a JSON `value`
never populates this string, so this feature is to augment JSON output with
embedded raw JSON data only.

To add raw JSON data create a `_rawdata` object and assign it to a `value`:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    #include "json.h"
    soap *ctx = soap_new1(SOAP_C_UTFSTRING | SOAP_XML_INDENT);
    json::value v(ctx);

    // create a blob of raw JSON
    _rawdata blob(ctx, "[ true, 123, { \"key\": 456 } ]");
    v = blob;
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The `_rawdata blob` can be assigned anywhere where a `value` is expected, i.e.
as an item of an array or as a value property of a JSON object.

To check that a `value` contains a `_rawdata` value and to retrieve it as raw
data:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    #include "json.h"
    soap *ctx = soap_new1(SOAP_C_UTFSTRING | SOAP_XML_INDENT);
    json::value v(ctx);

    ...
    if (v.is_rawdata())
    {
      _rawdata blob = (_rawdata)v;
      size_t size = blob.size();
      char *data = blob.ptr();
      ...
    }
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

If the raw data blob does not contain `\0` bytes, then it is simpler to cast
the `value` to a string:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    #include "json.h"
    soap *ctx = soap_new1(SOAP_C_UTFSTRING | SOAP_XML_INDENT);
    json::value v(ctx);

    ...
    if (v.is_rawdata())
    {
      char *data = v;
      ...
    }
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Moving JSON types and operations into a C++ namespace                 {#json-ns}
-----------------------------------------------------

A C++ namespace is preferred to separate JSON types and operations from other
project-related types and operations.  This allows you to cleanly compile the
JSON API files together with other gSOAP XML data binding files.

To put all JSON (and the internal XML-RPC) types and operations in a `json` C++
namespace, execute the following commands:

    soapcpp2 -qjson -CSL xml-rpc.h
    soapcpp2 -penv -CSL env.h

where `env.h` is an empty file.  This generates `jsonStub.h` and `jsonH.h` and
two more files `jsonC.cpp` and `envC.cpp` to compile with `xml-rpc.cpp`,
`json.cpp`, and `stdsoap2.cpp`.

When combining JSON with a wsdl2h-generated header file that declares an XML
data binding interface, use this header file instead of an empty `env.h`.  Use
soapcpp2 to generate the data binding and client/server code as usual (without
option `-penv`).

Compile the source files together with `xml-rpc.cpp` and `json.cpp` and set the
macro `-DJSON_NAMESPACE`, for example:

    c++ -DJSON_NAMESPACE xml-rpc.cpp json.cpp jsonC.cpp envC.cpp stdsoap2.cpp ...

To enable OpenSSL for HTTPS also use `-DWITH_OPENSSL` to compile.  Then link
with `-lssl` and `-lcrypto`.

Your project should now use the `json` namespace with the `value` type, for example:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    #include "json.h"
    soap *ctx = soap_new1(SOAP_C_UTFSTRING | SOAP_XML_INDENT);
    json::value v(ctx);
    std::cin >> v;        // parse JSON
    if (ctx->error) ...   // check for errors (can also check v.soap->error)
    std::cout << v;       // output JSON
    if (ctx->error) ...   // check for errors (can also check v.soap->error)
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

C XML-RPC and JSON                                                          {#c}
==================

With the release of gSOAP 2.8.26, the XML-RPC and JSON C APIs have been greatly
improved.  The material in this section pertains to gSOAP 2.8.26 and later.

The new C API for XML-RPC and JSON makes it much easier to populate and retrieve
data, but not as simple and easy as the C++ API.

List of files                                                         {#c-files}
-------------

The following files define XML-RPC operations and data types:

- `xml-rpc.c`:  XML-RPC C API
- `xml-rpc.h`:  XML-RPC data binding as a gSOAP .h file for soapcpp2
                (do not #include this file in your project builds)

In addition to the files above, for JSON we also need the following files:

- `json.h`:     JSON C API and JSON serialization
- `json.c`:     JSON C API and JSON serialization

The gSOAP header file `xml-rpc.h` defines all XML-RPC and JSON types and the C
API functions to create XML-RPC and JSON data for REST messages.

A note about the following auto-generated files: `soapH.h`, `soapStub.h` and
`soapC.c`: these are required for XML-RPC and JSON.  To auto-generate these
files, execute:

    soapcpp2 -c -CSL xml-rpc.h

Then compile and link the `.c` files listed above for XML-RPC and JSON with the
auto-generated `soapC.c` and `stdsoap2.c`:

    cc -I../.. -o myapp myapp.c json.c xml-rpc.c soapC.c ../../stdsoap2.c

Instead of `stdsoap2.c` you can link `libgsoap.a` with `-lgsoap`, when
installed by the gSOAP package.

To enable OpenSSL for HTTPS compile with `-DWITH_OPENSSL` and link `-lssl`, and
`-lcrypto`:

    cc -DWITH_OPENSSL -I../.. -o myapp myapp.c json.c xml-rpc.c soapC.c ../../stdsoap2.c -lssl -lcrypto

For OpenSSL support, instead of `stdsoap2.c` you can link `libgsoapssl.a` with
`-lgsoapssl`, when installed by the gSOAP package.

Because XML namespaces are not used, we can either use `-DWITH_NONAMESPACES` to
compile `stdsoap2.c` without complaining about a missing global `Namespace`,
or we can define an empty namespaces table somewhere in our code:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
    struct Namespace namespaces[] = {{NULL,NULL,NULL,NULL}};
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Overview                                                                 {#c-ex}
--------

An XML-RPC/JSON data value is created in C as follows, which requires a context
`ctx` with the engine state (the soap struct).  The context manages the memory
that is internally allocated to hold values.

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
    #include "soapH.h"

    struct soap *ctx = soap_new1(SOAP_C_UTFSTRING);  /* new context */

    struct value *v = new_value(ctx);

    soap_end(ctx);      /* delete all values */
    soap_free(ctx);     /* free context */
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Note that `soapH.h` is an auto-generated file (see previous section).  This
file needs to be generated only once and for all.  It also references
`stdsoap2.h` and the auto-generated `soapStub.h`.  Compile and link your code
with `stdsoap2.cpp` and the auto-generated `soapC.cpp` XML-RPC serializers.
Also compile and link `xml-rpc.cpp`.  For JSON, compile and link `json.cpp`.

With the C API, allocating a context and values on the heap may return NULL
when your system runs out of memory.  On most systems this is unlikely to
happen, but to safeguard your code check for NULL returns as follows:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
    #include "soapH.h"

    struct soap *ctx = soap_new1(SOAP_C_UTFSTRING);  /* new context */
    if (ctx == NULL)
      ...               /* out-of-memory error */

    struct value *v = new_value(ctx);
    if (v == NULL)
      ...               /* out-of-memory error */

    soap_end(ctx);      /* delete all values */
    soap_free(ctx);     /* free context */
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Also, when enlarging structs and arrays by indexing them beyond their current
content, the enlargement may fail and a NULL pointer is returned.

You can use wide strings with Unicode stored in UTF-8-formattted 8-bit `char`
strings.  For compatibility with XML-RPC serialization of UTF-8-encoded strings,
we MUST use the `SOAP_C_UTFSTRING` flag to initialize the context with
`soap_new1(SOAP_C_UTFSTRING)`.

The code shown above creates an empty value `v`.  Values can be assigned any one
of the following data types:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
    *int_of(v) = 12345LL;    /* 64 bit int */

    *double_of(v) = 12.34;   /* double float */

    *string_of(v) = "abc";   /* string */

    *string_of(v) = soap_wchar2s(ctx, L"xyz");
                             /* wide string (converted to UTF-8) */

    *bool_of(v) = 0;         /* Boolean false (0) or true (1) */

    *dateTime_of(v) = soap_dateTime2s(ctx, time(0));
                             /* time_t value serialized as ISO 8601 date time */

    /* create an array [24, 99.99, "abc"] */
    *int_of(nth_value(v, 0)) = 24;
    *double_of(nth_value(v, 1)) = 99.99;
    *string_of(nth_value(v, 2)) = "abc";

    /* create a struct (JSON object) {"name": "gsoap", "major": 2.8, "©": 2015} */
    *string_of(value_at(v, "name")) = "gsoap";
    *double_of(value_at(v, "major")) = 2.8;
    *int_of(value_atw(v, L"©")) = 2015;  /* wide string tags are OK */

    /* create a base64-encoded image object */
    struct _base64 *img = base64_of(v);
    img->__ptr = ptr_to_rawimage100bytes;  /* block of 100 raw bytes */
    img->__size = 100;
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The functions above return a pointer to a specific type of value and this value
can be assigned as shown above but also read.  So we use these functions also to
retrieve data, for example after receiving XML-RPC or JSON data.

We can combine this syntax in many possible ways to create arrays of arrays,
arrays of structs, and so on.  For example:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
    *string_of(value_at(nth_value(v, 0), "name")) = "bob";
    *string_of(nth_value(value_at(nth_value(v, 0), "toys"), 0)) = "ball";
    *string_of(nth_value(value_at(nth_value(v, 0), "toys"), 1)) = "furby";
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This creates a singleton array containing an object with two members: `name`
set to `"bob"` and `toys` set to an array containing `"ball"` and `"furby"`.  In
JSON format this is represented as:

               [ { "name": "bob", "toys": ["ball", "furby"] } ]
               ^ ^ ^              ^
               | | |              |
      an array_/ | |              |
     of 1 struct_/ |              |
    with 2 members_/______________/

Let's see what happens when we assign a variable the value of another.

The JSON C API uses the *reference model* for variables, meaning that values
are shared when assigning a target variable the value of another variable.
The variable values are shared until one or the other variables is assigned a
different type of value.  We illustrate the effect below:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
    struct value *x = new_value(ctx), *y = new_value(ctx);
    *int_of(x) = 1;
    *y = *x;
    *int_of(x) = 2;
    printf("x = %lld and y = %lld\n", *int_of(x), *int_of(y));
    *double_of(x) = 3.1;
    printf("x = %g and y = %lld\n", *double_of(x), *int_of(y));
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This prints `x = 2 and y = 2` and `x = 3.1 and y = 2`.

You can make a copy of an atomic value with one of the `Type_of()` functions.
The following code illustrates how:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
    struct value *x = new_value(ctx), *y;
    *int_of(x) = 1;
    *int_of(y) = *int_of(x);
    *int_of(x) = 2;
    printf("x = %lld and y = %lld\n", *int_of(x), *int_of(y));
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This prints `x = 2 and y = 1`.

You can also make deep copies of values by using the auto-generated
`soap_dup_value` function in `soapC.c`:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
    struct value *a = new_value(ctx), *b;
    *int_of(nth_value(a, 0)) = 1;
    b = soap_dup_value(ctx, NULL, a);
    *int_of(nth_value(a, 0)) = 2;
    printf("a[0] = %lld and b[0] = %lld\n", *int_of(nth_value(a, 0)), *int_of(nth_value(b, 0)));
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

This prints `a[0] = 2 and b[0] = 1`.

To auto-generate `soap_dup_value`, use option `-Ec` with soapcpp2:

    soapcpp2 -c -Ec -CSL xml-rpc.h

When receiving a value in XML-RPC or JSON, we generally want to check its type
to obtain its value.  To check the type of a value, we use `is_Type` functions:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
    is_null(v)     /* true if value is not set (JSON null) */
    is_bool(v)     /* true if value is a Boolean "true" or "false" value */
    is_true(v)     /* true if value is Boolean "true" */
    is_false(v)    /* true if value is Boolean "false" */
    is_number(v)   /* true if value is a number (int or float) */
    is_int(v)      /* true if value is a 32 or a 64 bit int */
    is_double(v)   /* true if value is a 64 bit double floating point (not integer) */
    is_string(v)   /* true if value is a string */
    is_dateTime(v) /* true if ISO 8601, always false for received JSON */
    is_array(v)    /* true if array of values */
    is_struct(v)   /* true if structure, a.k.a. a JSON object */
    is_base64(v)   /* true if base64, always false for received JSON */
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When working with JSON data, do not use `is_double()` but `is_number()`.  JSON
data does not differentiate between integers and floats.  However, gSOAP makes
sure that 64 bit integer values are accurately represented in JSON and decoded
without loss from JSON.  That is, when receiving an integer that you checked
with `is_int()` you can then safely retrieve the value with `int_of()`, for
example `int64_t n = *int_of(v)`.

The following functions can be used with arrays and structs (JSON objects):

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
    void set_struct(v)                         /* reset/create an empty struct */
    void set_size(v, int)                      /* reset/change array size or pre-allocate space */
    int has_size(v)                            /* returns array or struct size or 0 */
    int is_empty(v)                            /* returns 1 if array or struct is empty or 0 */
    struct value *nth_value(v, int)            /* returns nth value in array or struct */
    struct value *value_at(v, const char*)     /* returns value at field in struct */
    struct value *value_atw(v, const wchar_t*) /* returns value at field in struct */
    int nth_at(v, const char*)                 /* returns nth index of field in struct or -1 */
    int nth_atw(v, const wchar_t*)             /* returns nth index of field in struct or -1 */
    int nth_nth(v, int)                        /* returns nth index if nth index exists in array or -1 */
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

When accessing structs (JSON objects) with `value_at`, make sure to use
existing member field names only.  A new member fieldname-value pair is
dynamically added to the structure to accomodate the new entry for the field.

Also arrays are extended with `nth_value` to accommodate the indexed array
element.

A negative array index indexes elements from the end of the array, with index
-1 accessing the array's last value.

Use `set_size` to change array size or set arrays to zero length.  Use negative
size with `set_size` to remove elements from the end.

For example, let's take the value `v` that was assigned the array shown above.
We have the following properties of this value:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
    is_null(v) == false
    is_array(v) == true
    has_size(v) == 1
    is_empty(v) == 0
    is_struct(nth_value(v, 0)) == true
    nth_at(nth_value(v, 0), "name") == 0
    is_string(value_at(nth_value(v, 0), "name")) == true
    is_string(nth_value(value_at(nth_value(v, 0), "toys"), 0)) == true
    is_string(nth_value(value_at(nth_value(v, 0), "toys"), 1)) == true
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Considering that the code verbosity quickly increases when accessing deeper
levels of your structures, you are probably inclined to define your own macros
to create and access deep data more conveniently, such as:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
    #define string_at(v, s) string_of(value_at((v), (s)))
    #define nth_string(v, n) string_of(nth_value((v), (n)))
    #define nth_string_at(v, s, n) string_of(nth_value(value_at((v), (s)), (n)))
    #define string_at_nth(v, n, s) string_of(value_at(nth_value((v), (n)), (s)))
    #define nth_string_at_nth(v, n, s, m) string_of(nth_value(value_at(nth_value((v), (n)), (s)), (m)))
    ... etc ...
    *string_at_nth(v, 0, "name") = "bob";
    *nth_string_at_nth(v, 0, "toys", 0) = "ball";
    *nth_string_at_nth(v, 0, "toys", 1) = "furby";
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To iterate over array and struct values, we use a loop over `nth_value` and
`nth_member` as follows:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
    if (is_array(v))
    {
      int i;
      for (i = 0; i < has_size(v); i++)
      {
        struct value *array_value = nth_value(v, i);
        ... /* use and/or set array_value */
      }
    }
    else if (is_struct(v))
    {
      int i;
      for (i = 0; i < has_size(v); i++)
      {
        struct member *member = nth_member(v, i);
        const char *member_name = member->name;
        struct value *member_value = &member->value;
        ... /* use member_name and member_value */
      }
    }
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To access base64 binary raw data of a value `v`, we use the following code:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
    struct _base64 *base64 = base64_of(v);
    unsigned char *raw = base64->__ptr;  /* point to raw binary data */
    int size = base64->__size;           /* that is of this size */
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

XML-RPC parameter lists are similar to arrays and its values are indexed.  We
can also loop over response parameters after an XML-RPC REST call:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
    struct params *request = new_params(ctx);
    struct methodResponse response;
    *string_of(nth_param(request, 0)) = "hello";
    *string_of(nth_param(request, 1)) = "world";
    if (call_method(ctx, "endpoint URL", "methodName", request, &response) == SOAP_OK)
    {
      int i;
      for (i = 0; i < response.params->__size; i++)
      {
        struct value *param_value = nth_param(response.params, i);
        ... /* use param_value */
      }
    }
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

We should note that JSON REST does not require parameter types, for example:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
    #include "json.h"
    struct value *request = new_value(ctx);
    struct value *response = new_value(ctx);
    *string_of(nth_value(request, 0)) = "hello";
    *string_of(nth_value(request, 1)) = "world";
    if (json_call(cts, "endpoint URL", request, response) == SOAP_OK)
    {
      ... /* use response value */
    }
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

All dynamically allocated memory that is internally used to store data is
deallocated with:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
    soap_end(ctx);      /* delete all values */
    soap_free(ctx);     /* delete context allocated with soap_new() */
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Additional examples are located in `gsoap/samples/xml-rpc-json`:

- `json-currentTime.c`                  JSON REST C client
- `json-currentTimeServer.c`            JSON REST C server (CGI and multi-threaded stand-alone)
- `json-GitHub.c`:                      JSON C client for GitHub API v3
- `xml-rpc-currentTime.c`               XML-RPC C client
- `xml-rpc-weblogs.c`                   XML-RPC C client

C XML-RPC client example                                                 {#c-cl}
------------------------

An XML-RPC method call in C with the new XML-RPC C API:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
    #include "soapH.h"  /* generated by the command: soapcpp2 -CSL xml-rpc.h */
    struct soap *ctx = soap_new1(SOAP_C_UTFSTRING); /* UTF-8 in char* strings */
    struct params *request = new_params(ctx);
    struct methodResponse response;
    ctx->send_timeout = 10; /* 10 sec, stop if server is not accepting msg */
    ctx->recv_timeout = 10; /* 10 sec, stop if server does not respond in time */
    /* first parameter is an integer */
    *int_of(nth_param(request, 0)) = 123;
    /* second parameter is a string */
    *string_of(nth_param(request, 1)) = "abc";
    /* third parameter is a struct {"name": "joe", "age": 23} */
    *string_of(value_at(nth_param(request, 2), "name")) = "joe";
    *int_of(value_at(nth_param(request, 2), "age")) = 23;
    /* fourth parameter is an array [456.789, "widget", true] */
    *double_of(nth_value(nth_param(request, 3), 0)) = 456.789
    *string_of(nth_value(nth_param(request, 3), 1)) = "widget";
    *bool_of(nth_value(nth_param(request, 3), 2)) = 1;
    /* connect, send request, and receive response */
    if (call_method(ctx, "endpoint UTL", "methodName" request, &response))
    {
      soap_print_fault(ctx, stderr);
    }
    else if (response.fault)
    {
      /* write fault to stdout */
      soap_write_fault(ctx, response.fault);
    }
    else
    {
      /* print response parameters */
      int i;
      for (i = 0; i < response.params->__size; i++)
      {
        printf("Return parameter %d = ", i+1);
        display(nth_param(response.params, i)); /* see below */
        printf("\n");
      }
    }
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The following example shows how to traverse the node graph to display a value:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
    void display(struct value *v)
    {
      if (is_bool(v))
        printf(is_true(v) ? "true" : "false");
      else if (is_int(v))
        printf("%lld", int_of(v));
      else if (is_double(v))
        printf("%lG", double_of(v));
      else if (is_string(v))
        printf("\"%s\"", string_of(v));
      else if (is_array(v))
      {
        int i;
        printf("[");
        for (i = 0; i < has_size(v); i++)
        {
          if (i) printf(",");
          display(nth_value(v, i));
        }
        printf("]");
      }
      else if (is_struct(v))
      {
        int i;
        printf("{");
        for (i = 0; i < has_size(v); i++)
        {
          if (i) printf(",");
          printf("\"%s\": ", nth_member(v, i)->name);
          display(&nth_member(v, i)->value);
        }
        printf("}");
      }
      else if (is_dateTime(v))
        printf("\"%s\"", dateTime_of(v));
      else if (is_base64(v))
        printf("(%d bytes of raw data at %p)", base64_of(v)->__size, base64_of(v)->__ptr);
    }
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Compile and link your code together with `soapC.c` (generated), `xml-rpc.c`,
and `stdsoap2.c`.

C JSON serialization                                                     {#c-js}
--------------------

To write values in JSON format or parse JSON data, we use the `json.h` and
`json.c` JSON C API.  It is also possible to send and receive JSON data over
HTTP (JSON REST).

Reading and writing XML from/to files, streams and string buffers is done via
the managing context by setting one of the following context members that
control IO sources and sinks:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    ctx->recvfd = fd; // an int file descriptor to read from (0 by default)
    ctx->sendfd = fd; // an int file descriptor to write to (1 by default)
    ctx->is = cs;     // C only: a const char* string to read from (soap->is will advance)
    ctx->os = &cs;    // C only: pointer to a const char*, will be set to point to the string output
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For example, to read and write JSON data from/to a file descriptor:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
    #include "json.h" /* also compile and link json.c */
    struct Namespace namespaces[] = {{NULL,NULL,NULL,NULL}}; /* no XML namespaces */

    /* SOAP_C_UTFSTRING: UTF-8 content in char*, SOAP_XML_INDENT: indent JSON */
    struct soap *ctx = soap_new1(SOAP_C_UTFSTRING | SOAP_XML_INDENT);
    struct value *v = new_value(ctx);

    ctx->recvfd = ...;         /* set int file descriptor for reading */
    json_read(ctx, v);         /* read JSON into value v */
    if (ctx->error) ...        /* handle IO error (error message is in 'v' */

    ctx->sendfd = ...;         /* set int file descriptor for writing */
    json_write(ctx, v);        /* write value v in JSON format (indented) */
    if (ctx->error) ...        /* handle IO error (error message is in 'v' */
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

To force reading and writing JSON in ISO 8859-1 format, use the
`SOAP_ENC_LATIN` flag to initialize the context (not recommended).

Optionally use `SOAP_XML_INDENT` to indent XML and JSON output.

To parse JSON values from a file or from any other non-HTTP source, it is
recommended to set the `SOAP_ENC_PLAIN` context flag to prevent the parser from
attempting to read a MIME or HTTP headers (HTTP headers are required with
JSON-RPC):

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
    struct soap *ctx = soap_new1(SOAP_C_UTFSTRING | SOAP_XML_INDENT | SOAP_ENC_PLAIN);
    struct value *v = new_value(ctx);
    json_read(ctx, v);         /* read JSON without parsing MIME/HTTP header */
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can also read and write JSON data from/to NUL-terminated strings:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
    #include "json.h"
    struct Namespace namespaces[] = {{NULL,NULL,NULL,NULL}};

    struct soap *ctx = soap_new1(SOAP_C_UTFSTRING | SOAP_XML_INDENT | SOAP_ENC_PLAIN);
    struct value *v = new_value(ctx);

    const char *cs = "[1, 2, 3]";

    ctx->is = cs;
    json_read(ctx, v);         /* read JSON array from cs into value v */
    if (ctx->error) ...        /* handle IO error (error message is in 'v' */

    cs = NULL;
    ctx->os = &cs;
    json_write(ctx, v);        /* write value v in JSON format (indented) and set cs */
    if (ctx->error) ...        /* handle IO error (error message is in 'v' */
    printf("JSON data:\n%s\n", cs);
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Compile and link your code together with `soapC.c` (generated), `xml-rpc.c`,
`json.c`, and `stdsoap2.c`.

You can also convert XML-RPC data to/from JSON and populate XML-RPC from JSON
data.  The XML-RPC parsing and sending functions are `soap_read_value` and
`soap_write_value`, respectively.

The JSON protocol has fewer data types than XML-RPC, so type information can be
lost when serializing to JSON:

- XML-RPC uses a base64 type to exchange raw binary data.  The base64 data is
  converted to a string with base64 content by the JSON serializer.
  See also \ref base64.

- XML-RPC has a dateTime type, JSON does not.  The JSON serializer converts the
  dateTime type to a dateTime-formatted string.
  See also \ref dateTime.

Strings are stored and exchanged in UTF-8 format in 8-bit strings (i.e. `char*`
strings) with the `SOAP_C_UTFSTRING` flag.  Wide strings (i.e. `wchar_t*`
strings) are converted to UTF-8.

To read JSON from a string and write JSON to a string, we suggest to use gSOAP
2.8.28 or later.  With these newer versions you can set the contex input string
`ctx->is` to the string to parse (reset to NULL afterwards) as follows:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
    value *v = new_value(ctx);
    ctx->is = "[ [1, \"2\", 3.14, true], {\"name\": \"john\", \"age\": 24} ]": 
    json_read(ctx, v);
    if (ctx->error) ... /* handle IO error (error message is in 'v' */
    ctx->is = NULL;     /* stop reading from string */
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can set the context output string pointer `ctx->os` to point to the `const
char*` string that you want to be set.  This string will point to the JSON data
created by the engine and managed by the context as follows:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
    value *v = new_value(ctx);
    const char *json_out = NULL;
    ... /* populate value v */
    ctx->os = &json_out; /* the string to point to JSON data */
    json_write(ctx, v);
    if (ctx->error) ...
    ctx->os = NULL;      /* stop writing to strings */
    if (json_out)
    {
      ... /* use json_out string, do not free() (managed by the context) */
    }
    ...
    soap_end(ctx);  /* deletes json_out string */
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For older versions prior to 2.8.28, the gSOAP engine's IO `frecv` callback
function can be used as follows to parse JSON from a string buffer:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
    size_t buf_recv(struct soap *ctx, char *buf, size_t len)
    {
      const char *in = (char*)ctx->user;  /* get handle to input string */
      size_t n = strlen(in);
      if (n > len) /* if in[] is larger than buf[] len */
        n = len;   /* then cap length at len */
      memcpy(buf, in, n);
      in += n;
      ctx->user = (void*)in;  /* update the handle */
      return n;
    }
    ...
    value *v = new_value(ctx);
    const char *json_in = "[ [1, \"2\", 3.14, true], {\"name\": \"john\", \"age\": 24} ]": 
    ctx->frecv = buf_recv;
    ctx->user = (void*)json_in;  /* a user handle that is passed to buf_recv */
    json_read(ctx, v);
    if (ctx->error) ...
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For older versions prior to 2.8.28, the gSOAP engine IO `fsend` callback
function can be used to write JSON to strings as follows:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
    #define BUFFER_INCREMENT (1024)
    int buf_send(struct soap *ctx, const char *buf, size_t len)
    {
      char *out = (char*)ctx->user;  /* get handle to current buffer, if any */
      size_t n = out ? strlen(out) : 0;
      size_t k = (n + len + 1)/BUFFER_INCREMENT;
      if (!out)  /* first time around? */
      {
        out = malloc((k + 1) * BUFFER_INCREMENT);
      }
      else if (n/BUFFER_INCREMENT < k)  /* need to increase buffer? */
      {
        char *more = malloc((k + 1) * BUFFER_INCREMENT);
        memcpy(more, out, n);
        free(out);
        out = more;
      }
      memcpy(out + n, buf, len);
      out[n + len] = '\0';
      ctx->user = (void*)out;
      return SOAP_OK;
    }
    ...
    value *v = new_value(ctx);
    const char *json_out;
    ... /* populate value v */
    ctx->fsend = buf_send;
    ctx->user = NULL;
    json_write(ctx, v);
    if (ctx->error) ...
    json_out = (char*)ctx->user;
    if (json_out)
    {
      ... /* use json_out string */
      free(json_out);
      ctx->user = NULL;
    }
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

C JSON REST clients and servers                                          {#c-jr}
-------------------------------

To use JSON REST on the client side, we use `json_call`:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
    #include "json.h"             /* also compile and link json.c */
    struct Namespace namespaces[] = {{NULL,NULL,NULL,NULL}}; /* no XML namespaces */

    /* SOAP_C_UTFSTRING: UTF-8 content in char*, SOAP_XML_INDENT: indent JSON */
    struct soap *ctx = soap_new1(SOAP_C_UTFSTRING | SOAP_XML_INDENT);
    struct value *request = new_value(ctx);
    struct value response;
    ... /* here we populate the request data to send */
    if (json_call(ctx, "endpoint URL", request, &response))
      ... /* error */
    ... /* use the response data */
    soap_end(ctx); /* delete all values */
    ... /* here we can make other calls etc. */
    soap_free(ctx); /* delete the context */
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The `json_call` function takes a context, an endpoint URL (with query string
parameters as needed), and optional `in` and `out` values to send and receive,
respectively.  The function returns `SOAP_OK` (zero) for success or `EOF`,
`SOAP_SYNTAX_ERROR`, or an HTTP error code.

To use the JSON REST POST method, pass both `in` and `out` values to
`json_call`.  For the GET method, pass a NULL to `in`.  For the PUT method, pass
a NULL to `out`.  For the DELETE method, pass both NULL to `in` and `out`.

Besides `json_call`, there are other JSON API functions:

- `int json_call(struct soap *ctx, const char *URL, struct value *in, struct
  value *out)` makes a POST, GET, PUT, DELETE call, returns `SOAP_OK` or error
  code.  POST method: pass both `in` and `out`.  GET method: pass a NULL to
  `in`.  PUT method: pass a NULL to `out`.  DELETE method: pass both NULL to `in`
  and `out`.  `SOAP_OK` is returned for POST and GET methods upon success.  An
  HTTP status code of 200 (OK) or 202 (Accepted) is returned for PUT and DELETE
  methods upon success.

- `int json_write(struct soap *ctx, const struct value *v)` Writes JSON value
  to current file or socket.  Returns `SOAP_OK` or error.  Set current
  file/socket file descriptor with `ctx->sendfd = fd`.  Or save JSON to a
  string by setting `ctx->os = &cs` with `const char *cs` a string pointer that
  will be set to a NUL-terminated string when the write completed.

- `int json_read(struct soap *ctx, struct value *v)` Reads JSON value from
  current file or socket.  Returns `SOAP_OK` or error.  Set current file/socket
  file descriptor with `ctx->recvfd = fd`.  Or read from string by setting
  `ctx->is = cs` to a `const char *cs` NUL-terminated string.

The are two other lower-level functions `json_send` and `json_recv` that are
similar to `json_write` and `json_read` but do not initialize the sending and
receiving operations and do not flush after the sending and receiving
operations.

Compile and link your code together with `soapC.c` (generated), `xml-rpc.c`,
`json.c`, and `stdsoap2.c`.

To implement a JSON REST server for CGI (e.g. install in cgi-bin):

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    #include "json.h"            // also compile and link json.c
    struct Namespace namespaces[] = {{NULL,NULL,NULL,NULL}}; // no XML namespaces

    int main()
    {
      struct soap *ctx = soap_new1(SOAP_C_UTFSTRING | SOAP_XML_INDENT | SOAP_ENC_PLAIN);
      struct value *request = new_value(ctx);
      struct value *response = new_value(ctx);
      if (soap_begin_recv(ctx)
       || json_recv(ctx, request)
       || soap_end_recv(ctx))
      {
        json_send_fault(ctx); // return a JSON-formatted fault
      }
      else
      {
        ... // use the 'request' value
        ... // set the 'response' value
        // set http content type
        ctx->http_content = "application/json; charset=utf-8";
        // send http header 200 OK and JSON response
        if (soap_response(ctx, SOAP_FILE)
         || json_send(ctx, response)
         || soap_end_send(ctx))
          soap_closesock(ctx);
      }
      // dealloc all
      soap_end(ctx);
      soap_free(ctx);
    }
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The above server works with CGI, which is rather slow and stateless.  A
stand-alone JSON REST server is recommended.  You can also use the Apache and
IIS plugins for gSOAP to deploy JSON REST services.  See the
[documentation](https://www.genivia.com/docs.html).

Note that we use `json_send_fault()` instead of `soap_send_fault()` when an
internal error occurred, since we want the error to be reported in JSON format
as per Google JSON Style Guide.  For application-specific errors, we use
`json_send_error()` as follows:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    if (some application error occurred)
      json_send_error(ctx, 400, "Error message", "Error details");
    else
      ... // send the response
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can specify an HTTP error code such as 400 in this case, which means that a
HTTP 400 Bad Request is returned to the client.

The `json_send_error()` function is a convenient shortcut to return an error
message with an HTTP error code.  To return a JSON response with a HTTP error
code use the following:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    int status = 400; // HTTP 400 Bad Request
    soap->http_content = "application/json; charset=utf-8";
    if (soap_response(soap, SOAP_FILE + status)
     || json_send(soap, v)
     || soap_end_send(soap))
      soap_closesock(soap);
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The code for a stand-alone server is the same, except that we call `soap_bind()`
and `soap_accept()` to serve requests on a port.  Here is a simple iterative
server serving requests on port 8080:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    // create an allocation context
    struct soap *ctx = soap_new1(SOAP_IO_KEEPALIVE | SOAP_C_UTFSTRING);
    // bind to port 8080
    if (!soap_valid_socket(soap_bind(ctx, NULL, 8080, 100)))
      ... // error, stop
    // accept messages in server loop
    for (;;)
    {
      struct value *request = new_value(ctx);
      struct value *response = new_value(ctx);
      if (!soap_valid_socket(soap_accept(ctx)))
        ... // error, stop
      if (soap_begin_recv(ctx)
       || json_recv(ctx, request)
       || soap_end_recv(ctx))
      {
        json_send_fault(ctx); // return a JSON-formatted fault
      }
      else
      {
        ... // use the 'request' value
        ... // set the 'response' value
        // set http content type
        ctx->http_content = "application/json; charset=utf-8";
        // send http header 200 OK and JSON response
        if (soap_response(ctx, SOAP_FILE)
         || json_send(ctx, response)
         || soap_end_send(ctx))
          soap_closesock(ctx);
      }
      // dealloc all
      soap_end(ctx);
    }
    soap_free(ctx);
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Note that `SOAP_ENC_PLAIN` should be used with CGI, since we are reading from a
non-HTTP source with CGI.  Do not use this flag with gSOAP stand-alone servers.
Iterative stand-alone services are not performant and may block requests.  We
recommend stand-alone multi-threaded services.

Compile and link your code together with `soapC.c` (generated with
`soapcpp2 -c -CSL xml-rpc.h`), `xml-rpc.c`, `json.c`, and `stdsoap2.c`.

For client and server examples, please see the gSOAP package content in
`gsoap/samples/xml-rpc-json`:

- `json-currentTime.c`                  JSON REST C client
- `json-currentTimeServer.c`            JSON REST C server (CGI and multi-threaded stand-alone)

The presentation in this section concerned stand-alone JSON REST services.  To
combine SOAP/XML with JSON REST services, see \ref soap for details.

C JSON-RPC clients and servers                                          {#c-rpc}
------------------------------

The [JSON-RPC 1.0 specification](http://json-rpc.org/wiki/specification) (the
"original version") adds `method`, `parameter` and `id` fields to the
request message:

<div class="alt">
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    {
      "method": "echo",
      "params": [ "Hello World!" ],
      "id": 1
    }
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
</div>

where:

- `method` is a string with the name of the method to be invoked.
- `params` is an array of objects to be passed as parameters to the defined method.
- `id` is a value of any type, which is used to match the response with the request that it is replying to.

A response message has a `result` field, an `error` field, and an `id`:

<div class="alt">
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    {
      "result": "Welcome!",
      "error": null,
      "id": 1
    }
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
</div>

where:

- `result` is the data returned by the invoked method. If an error occurred while invoking the method, this value must be null.
- `error` is  a specified error code if there was an error invoking the method, otherwise null.
- `id` is the id of the request it is responding to.

The [JSON-RPC 2.0 specification](http://www.jsonrpc.org/specification) makes
all 1.0 fields REQUIRED, except for `error` which MUST NOT be present if there
was no error triggered during invocation.  The 2.0 specification adds a
`jsonrpc` field in the request message:

<div class="alt">
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    {
      "jsonrpc": 2.0,
      "method": "echo",
      "params": [ "Hello World!" ],
      "id": 1
    }
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
</div>

and also adds the `jsonrpc` field to the response message:

<div class="alt">
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    {
      "jsonrpc": 2.0,
      "result": "Welcome!",
      "id": 1
    }
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
</div>

The C operations are straightforward to conform to the JSON-RPC 1.0 or 2.0
specifications.  The example JSON-RPC 2.0 request message shown above is
created by the following code that uses `json_call()` to invoke a JSON-RPC
service:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
      const char *endpoint = "http://...";
      struct soap *ctx = soap_new1(SOAP_C_UTFSTRING | SOAP_XML_INDENT);

      struct value *req = new_value(ctx);
      struct value *res = new_value(ctx);
      struct value *args;
      *double_of(value_at(req, "jsonrpc")) = 2.0;
      *string_of(value_at(req, "method"))  = "echo";
      args = value_at(ref, "params");
      *string_of(nth_value(args, 0))       = "Hello World!";
      *int_of(value_at(req, "id"))         = 1;

      if (json_call(ctx, endpoint, req, res))              // JSON-RPC call
        soap_printf_fault(ctx, stderr);
      else if (nth_at(res, "error") >= 0)                  // JSON-RPC error?
        json_write(ctx, value_at(res, "error"));
      else if ((int)res["id"] != 1)                        // matching id field?
        printf("response id != request id\n");
      else                                                 // all OK!
        json_write(ctx, value_at(res, "result"));          // display result
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

For HTTPS use an https endpoint URL.  You can control the HTTPS context as
explained in the [tutorials.](http://www.genivia.com/tutorials.html)

The server-side creates response messages similar to the example shown above.
To implement a C JSON-RPC and JSON REST server, please see the example
`json-currentTimeServer.c` located in `gsoap/samples/xml-rpc-json` in the
gSOAP package.

Embedding and serializing raw JSON data as a JSON value in C            {#c-raw}
------------------------------------------------------------

A JSON value represented by a `value` may contain raw JSON data as a string to
serialize literally in the output.  Deserializing JSON into a JSON `value`
never populates this string, so this feature is to augment JSON output with
embedded raw JSON data only.

To add raw JSON data create a `_rawdata` structure and assign it to a `value`:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    #include "json.h"
    struct soap *ctx = soap_new1(SOAP_C_UTFSTRING | SOAP_XML_INDENT);
    struct value *v = new_value(ctx);

    /* create a blob of raw data */
    struct _rawdata *blob = rawdata_of(v);
    const char *data = "[ true, 123, { \"key\": 456 } ]";
    blob->__ptr = (void*)data;
    blob->__size = strlen(data);
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The `_rawdata blob` can be assigned anywhere where a `value` is expected, i.e.
as an item of an array or as a value property of a JSON object.

To check that a `value` contains a `_rawdata` value and to retrieve it as raw
data:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    #include "json.h"
    struct soap *ctx = soap_new1(SOAP_C_UTFSTRING | SOAP_XML_INDENT);
    struct value *v = new_value(ctx);
    ...
    if (is_rawdata(v))
    {
      struct _rawdata *blob = rawdata_of(v);
      char *data = (char*)blob.__ptr;
      size_t size = blob.__size;
      ...
    }
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Miscellaneous                                                            {#misc}
=============

Compiling XML-RPC/JSON together with gSOAP SOAP services and XML      {#json-cc}
----------------------------------------------------------------

To use JSON (and XML-RPC) with other gSOAP XML data binding code requires a few
simple steps to ensure that your project compiles cleanly.

For C++, arguably the best option is to \ref json-ns.

We present three methods that you can follow.  These methods follow different
strategies to compile a combined set of files with JSON (and XML-RPC) types and
operations with other files with XML data binding types and operations:

### Method 1: #import "xml-rpc.h"

Before processing a gSOAP XML data binding header file with soapcpp2, add
`#import "xml-rpc.h"` to this header file to include XML-RPC and JSON data
types.  Then compile the generated files as usual together with `jcon.c` (or
`json.cpp` for C++) and `xml-rpc.c` (or `xml-rpc.cpp` for C++).  Also `#include
"json.h"` in your code to use the JSON API.

This is the simplest method.  When using wsdl2h, you can automate this method
by adding the following three lines to `typemap.dat`:

    [
    #import "xml-rpc.h"
    ]

This automatically imports the JSON/XML-RPC types and operations into the XML
data binding code.  You will still need to compile your code together with
`jcon.c` (or `json.cpp` for C++) and `xml-rpc.c` (or `xml-rpc.cpp` for C++).

### Method 2: soapcpp2 -qjson (works with C++ only)

Use soapcpp2 option `-qjson` to generate and compile the JSON (and XML-RPC) API
code to combine with your other XML data binding code generated with soapcpp2:
  
    soapcpp2 -qjson -CSL xml-rpc.h

This generates `jsonStub.h`, `jsonH.h`, and `jsonC.cpp`.

Compile the files `jsonC.cpp`, `xml-rpc.cpp`, and `json.cpp` with option
`-DJSON_NAMESPACE` and your other source code files, such as those generated by
soapcpp2 for your other .h file:

    c++ -DJSON_NAMESPACE json.cpp jsonC.cpp xml-rpc.cpp ...

Make sure to use `#include "json.h"` in your code.

The JSON C++ API is in the `json` C++ namespace.  See also \ref json-ns.

### Method 3: soapcpp2 -pjson

Use soapcpp2 option `-pjson` to generate and compile the JSON (and XML-RPC) API
code to combine with your other XML data binding code generated with soapcpp2:

    soapcpp2 -c -pjson -CSL xml-rpc.h

This generates `jsonStub.h`, `jsonH.h`, and `jsonC.c`.

Compile the files `jsonC.c`, `xml-rpc.c`, and `json.c` with option
`-DJSON_NAMESPACE` and your other source code files, such as `soapClientLib.c`
generated by soapcpp2 with option `-C` for your other .h file:

    cc -DJSON_NAMESPACE json.c jsonC.c xml-rpc.c soapClientLib.c stdsoap2.c ...

or the `soapServerLib.c` file generated by soapcpp2 with option `-S`:

    cc -DJSON_NAMESPACE json.c jsonC.c xml-rpc.c soapServerLib.c stdsoap2.c ...

Make sure to use `#include "json.h"` in your code.

This method also works in C++, but it is recommended to use the second method
for C++ applications.

Serving JSON and SOAP requests on the same server port                   {#soap}
------------------------------------------------------

SOAP requests are served using the usual `soap_serve()` call generated by
soapcpp2 given an interface header file with SOAP/XML Web service definitions.

We can also serve JSON requests on the same port by using the gSOAP HTTP POST
plugin.  This plugin serves non-SOAP requests when `soap_serve()` is called by
capturing the HTTP content type of the request and also handles PUT, PATCH and
DELETE requests.  The httpget plugin is located in `gsoap/plugin/httppost.h`
and `gsoap/plugin/httppost.c`.

The plugin captures POST requests that have a content type matching a content
type entry specified in a table of POST, PUT, PATCH and DELETE handlers.  Each
entry in this table specifies a content type of a POST request with the
function to invoke that will handle it.  We can also specify generic POST, PUT,
PATCH and DELETE request handlers in this table:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
    struct http_post_handlers my_handlers[] = {
      { "application/json",   json_handler },
      { "application/json;*", json_handler },
      { "POST",               generic_POST_handler },
      { "PUT",                generic_PUT_handler },
      { "PATCH",              generic_PATCH_handler },
      { "DELETE",             generic_DELETE_handler },
      { NULL }
    };
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Note that `*` can be used as a wildcard, in this case we use `*` to also capture
`"application/json; charset=utf-8"` content type variations.

To register the plugin:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
    #include "plugin/httppost.h"

    struct soap *ctx = soap_new();
    soap_register_plugin_arg(ctx, http_post, my_handlers);
    ...
    soap_serve(ctx); // serve SOAP and other POST/PUT/PATCH/DELETE requests
    ...
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The handlers in the example `my_handlers` table given above are used by the
plugin to dispatch the request.  If a non-SOAP or non-POST request is made, the
handler in the table that matches the HTTP content type of the request is
invoked.  Also generic POST, PUT, PATCH and DELETE handlers can be optionally
specified.

A handler in the `my_handlers` table is a function that takes the context and
returns `SOAP_OK` or an error code.  Here is an example `json_handler` in C:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
    int json_handler(struct soap *ctx)
    {
      struct value *request = new_value(ctx);
      struct value *response = new_value(ctx);
      if (json_recv(ctx, request) || soap_end_recv(ctx))
        return json_send_fault(ctx); // return a JSON-formatted fault
      ... // use the 'request' value
      ... // set the 'response' value
      // set http content type
      ctx->http_content = "application/json; charset=utf-8";
      // send http header 200 OK and JSON response
      if (soap_response(ctx, SOAP_FILE)
       || json_send(ctx, response)
       || soap_end_send(ctx))
        soap_closesock(ctx);
      return SOAP_OK;
    }
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Note that we use `json_send_fault()` instead of `soap_send_fault()` when an
internal error occurred, since we want the error to be reported in JSON format
as per Google JSON Style Guide.  For application-specific errors, we use
`json_send_error()` as follows:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    if (some application error occurred)
      json_send_error(ctx, 400, "Error message", "Error details");
    else
      ... // send the response
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

You can specify an HTTP error code such as 400 in this case, which means that a
HTTP 400 Bad Request is returned to the client.

The `json_send_error()` function is a convenient shortcut to return an error
message with an HTTP error code.  To return a JSON response with a HTTP error
code use the following:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    int status = 400; // HTTP 400 Bad Request
    soap->http_content = "application/json; charset=utf-8";
    if (soap_response(soap, SOAP_FILE + status)
     || json_send(soap, v)
     || soap_end_send(soap))
      soap_closesock(soap);
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

A `generic_POST_handler` should be similar to the above.  The HTTP content type
of the request is stored in the `http_content` string variable of the context.

A `generic_PUT_handler` and `generic_PATCH_handler` should not return a
response message but should call `soap_send_empty_response()` instead.
Likewise, these handlers should return a HTTP status code instead of calling
`json_send_error()` since the HTTP body should be empty:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
    int generic_PUT_handler(struct soap *ctx)
    {
      struct value *request = new_value(ctx);
      if (!ctx->http_content || soap_tag_cmp(ctx->http_content, "application/json*"))
      {
        return 400; // HTTP Bad Request
      }
      if (json_recv(ctx, request) || soap_end_recv(ctx))
        return 400; // HTTP Bad Request
      ... // use the 'request' value
      // send http header 200 OK and empty http body
      return soap_send_empty_response(ctx);
    }
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The `soap_tag_cmp()` function is similar to `strcmp()` but it is case
insensitive and supports `*` (any number of characters) and `-` (any single
character) wildcards.

To compile your server application that serves JSON and SOAP requests, see
\ref json-cc.

Floating point format                                                      {#fp}
---------------------

The floating point format used to output values in XML-RPC and JSON is by
default ".17lG' to ensure accuracy up to the last digit.  The format can be set
as follows:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
    struct soap *ctx = soap_new1(SOAP_C_UTFSTRING | SOAP_XML_INDENT);
    ctx->double_format = "%lG";
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

JSON and base64                                                        {#base64}
---------------

JSON has no binary type to transmit binary data.  Sending binary data in JSON
strings as text is not recommended, due to NULs and problems with
Unicode/UTF-8 sequences.

Base64 is a common encoding format for binary data.  A JSON string with base64
content is our recommended option.

To populate JSON data with base64-encoded binary content, you can simply create
and assign a `_base64` value as described earlier (e.g.  by casting a `_base64`
structure to a value in C++).  Receiving base64-encoded content with JSON is
not possible, because the necessary type information is lost in transit.  The
base64 content will arrive at the receiver simply as a string with base64
content.

You can explicitly decode the base64 string back to binary as shown here for
C++:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    if (v.is_string())
    {
      /* assuming base64 content in string value v, decoded it */
      int len;
      unsigned char *ptr = (unsigned char*)soap_base642s(ctx, (const char*)v, NULL, 0, &len);
      /* ptr points to binary of length len or is NULL when decoding failed */
      if (ptr)
        ... // success
      ctx->error = SOAP_OK; // fail and reset error
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

And for C:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
    if (is_string(v))
    {
      /* assuming base64 content in string value v, decoded it */
      int len;
      unsigned char *ptr = (unsigned char*)soap_base642s(ctx, *string_of(v), NULL, 0, &len);
      /* ptr points to binary of length len or is NULL when decoding failed */
      if (ptr)
        ... // success
      ctx->error = SOAP_OK; // fail and reset error
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

JSON and ISO 8601 dateTime                                           {#dateTime}
--------------------------

To populate JSON data with ISO 8601 date time content, you can simply assign a
`ULONG64` value cast from a `time_t` value as described earlier.  Receiving ISO
8601 date time content with JSON is not possible, because the necessary type
information is lost in transit.  The content will arrive at the receiver simply
as a string with a date and time.

You can explicitly convert a string with an ISO 8601 date time to a `time_t`
value as shown here for C++:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.cpp}
    if (v.is_string())
    {
      time_t tm;
      if (soap_s2dateTime(ctx, (const char*)v, &tm) == SOAP_OK)
        ... // success
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

And for C:

~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
    if (is_string(v))
    {
      time_t tm;
      if (soap_s2dateTime(ctx, *string_of(v), &tm) == SOAP_OK)
        ... // success
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The disable UTC time zone `Z` in the dateTime string, use `-DWITH_NOZONE` to
compile `stdsoap2.c` (for C) or `stdsoap2.cpp` (for C++).

Potential issues                                                       {#issues}
----------------

The JSON parser follows the published JSON "standards" but is a bit more
forgiving, meaning that it parses the following extensions:

- The parser admits floating point values that are formatted as C floating
  point values in addition to JSON formats for numbers (which are more
  restrictive).
- The parser admits `NaN`, `+Inf`, `-Inf` as floating point values.
- The parser admits hexadecimal integer values of the form `0xHHHH`.
- Any additional trailing content after a valid JSON object or array is
  silently ignored.
- To parse JSON data from files or from any other non-HTTP source use the
  `SOAP_ENC_PLAIN` flag to initialize the context, otherwise files containing
  just the JSON values `true`, `false`, and `null` are not parsed.
 
Copyright                                                           {#copyright}
=========

Copyright (c) 2017, Robert van Engelen, Genivia Inc. All rights reserved.
