/*
        jsoncpp.cpp
        
        A JSON C/C++ code generator for the gSOAP JSON API.

        1. Take a JSON sample document and render it in JSON API code to
           construct this JSON value:

        echo '\
        { "menu": {\
            "id": "file",\
            "value": "File",\
            "popup": {\
              "menuitem": [\
                {"value": "New", "onclick": "CreateNewDoc()"},\
                {"value": "Open", "onclick": "OpenDoc()"},\
                {"value": "Close", "onclick": "CloseDoc()"}\
              ]\
            }\
          }\
        }' | ./jsoncpp

        #include "json.h"
        {
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

          soap_destroy(ctx); // delete objects
          soap_end(ctx);     // delete data
          soap_free(ctx);    // free context
        }

        2. Take a JSON sample document and render it in JSON API code to
           inspect and extract values from JSON input data, by using the JSON
           sample document as a template:

        echo '\
        { "menu": {\
            "id": "file",\
            "value": "File",\
            "popup": {\
              "menuitem": [\
                {"value": "New", "onclick": "CreateNewDoc()"},\
                {"value": "Open", "onclick": "OpenDoc()"},\
                {"value": "Close", "onclick": "CloseDoc()"}\
              ]\
            }\
          }\
        }' | ./jsoncpp -i

        #include "json.h"
        {
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

          soap_destroy(ctx); // delete objects
          soap_end(ctx);     // delete data
          soap_free(ctx);    // free context
        }

        3. Take a JSONPath to generate a stand-alone application that filters
           JSON documents with a store of books to return their titles:

        ./jsoncpp -m -p'$.store.book[:].title'

        #include "json.h"
        struct Namespace namespaces[] = {{NULL,NULL,NULL,NULL}};
        int main(int argc, char **argv)
        {
          struct soap *ctx = soap_new1(SOAP_C_UTFSTRING | SOAP_XML_INDENT);
          ctx->double_format = "%lG";

          value x(ctx);
          std::cin >> x;
          if (x.soap->error)
            exit(EXIT_FAILURE); // error parsing JSON
          // $.store.book[:].title
          #define QUERY_YIELD(val) std::cout << val << std::endl
          if (x.has("store"))
          {
            if (x["store"].has("book"))
            {
              if (x["store"]["book"].is_array())
              {
                int j, k = x["store"]["book"].size()-1;
                for (j = 0; j <= k; j += 1)
                {
                  value& r = x["store"]["book"][j];
                  if (r.has("title"))
                  {
                    QUERY_YIELD(r["title"]);
                  }
                }
              }
            }
          }

          soap_destroy(ctx); // delete objects
          soap_end(ctx);     // delete data
          soap_free(ctx);    // free context
          return 0;
        }

        4. To build jsoncpp:

        cd gsoap/samples/xml-rpc-json
        soapcpp2 -CSL xml-rpc.h
        c++ -I../.. -o jsoncpp jsoncpp.cpp json.cpp xml-rpc.cpp soapC.cpp ../../stdsoap2.cpp

        5. To compile and link jsoncpp-generated C++ code:

        ./jsoncpp -o myjson.cpp ...
        soapcpp2 -CSL xml-rpc.h
        c++ -I../.. -o myjson myjson.cpp json.cpp xml-rpc.cpp soapC.cpp ../../stdsoap2.cpp

        6. To compile and link jsoncpp-generated C code:

        ./jsoncpp -c -o myjson.c ...
        soapcpp2 -c -CSL xml-rpc.h
        cc -I../.. -o myjson myjson.c json.c xml-rpc.c soapC.c ../../stdsoap2.c

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2000-2016, Robert van Engelen, Genivia, Inc. All Rights Reserved.
This software is released under one of the following two licenses:
GPL.
--------------------------------------------------------------------------------
GPL license.

This program is free software; you can redistribute it and/or modify it under
the terms of the GNU General Public License as published by the Free Software
Foundation; either version 2 of the License, or (at your option) any later
version.

This program is distributed in the hope that it will be useful, but WITHOUT ANY
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A
PARTICULAR PURPOSE. See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License along with
this program; if not, write to the Free Software Foundation, Inc., 59 Temple
Place, Suite 330, Boston, MA 02111-1307 USA

Author contact information:
engelen@genivia.com / engelen@acm.org
--------------------------------------------------------------------------------
A commercial use license is available from Genivia, Inc., contact@genivia.com
--------------------------------------------------------------------------------
*/

#include "json.h"
#include <fstream>
#include <sstream>
#include <iomanip>

struct Namespace namespaces[] = {{NULL,NULL,NULL,NULL}};

////////////////////////////////////////////////////////////////////////////////
//
//      Command line options
//
////////////////////////////////////////////////////////////////////////////////

static bool coutput = false;            // -c      generate C code instead of C++
static bool explain = false;            // -e      add explanatory comments to the generated code
static bool genread = false;            // -i      generate code to inspect JSON node graph with data type checks
static bool nocheck = false;            // -k      generate code to inspect JSON node graph directly without checks
static bool genvars = false;            // -l      generate code for option -i to store values in local variables
static bool genmain = false;            // -m      generate stand-alone code by adding main()
static bool minimal = false;            // -M      generate minimal code unadorned with initialization and cleanup
static bool optimal = false;            // -O      optimize code by factoring common indices when applicable
static bool collect = false;            // -y      generate code that yields an array 'y' of JSONPath query results
static const char *dform = NULL;        // -f%fmt  use %%fmt to format double floats, e.g. -f%%lG
static const char *ofile = NULL;        // -ofile  save source code to file
static const char *jpath = NULL;        // -ppath  generate JSONPath query code for 'path'
static const char *jroot = "x";         // -rroot  use 'root' instead of root value 'x' in the generated code
static const char *jcode = NULL;        // -xcode  generate code that executes 'code' for each JSONPath query result
static const char *ifile = NULL;        // [infile]

////////////////////////////////////////////////////////////////////////////////
//
//      Proto
//
////////////////////////////////////////////////////////////////////////////////

static void out_gen_c(soap*, value&, std::string, const std::string&, int);
static void in_gen_c(soap*, value&, std::string, const std::string&, int);
static void out_gen_cpp(soap*, value&, std::string, const std::string&, int);
static void in_gen_cpp(soap*, value&, std::string, const std::string&, int);
static void path_gen_c(soap*, const char*, std::string&, int, bool);
static void path_exec_c(soap*, std::string&, int);
static void path_gen_cpp(soap*, const char*, std::string&, int, bool);
static void path_exec_cpp(soap*, std::string&, int);
static std::ostream& indent(soap*, int);
static std::string gen_ident(const std::string&, const char*);
static std::string gen_counter(int);
static std::string putstrname(const char*);
static std::string putname(const char*);
static std::string getname(const char**);

////////////////////////////////////////////////////////////////////////////////
//
//      Pretty printer
//
////////////////////////////////////////////////////////////////////////////////

struct putstr
{
  const char *str;
  int quote;
  putstr(const char *str, int quote = '"') : str(str), quote(quote) { }
  putstr(const std::string& str, int quote = '"') : str(str.c_str()), quote(quote) { }
};

std::ostream& operator<<(std::ostream& os, const putstr& p)
{
  if (p.str)
  {
    if (p.quote)
      os << (char)p.quote;
    for (const char *s = p.str; *s; ++s)
    {
      switch (*s)
      {
        case '\t':
          os << "\\t";
          break;
        case '\n':
          os << "\\n";
          break;
        case '\r':
          os << "\\r";
          break;
        default:
          if (*s > 0 && *s < 0x10)
            os << "\\x0" << std::hex << *s << std::dec;
          else if (*s < 0x20 || *s >= 0x7f)
            os << "\\x" << std::hex << (*s & 0xff) << std::dec;
          else if (coutput && !p.quote && s[0] == '*' && s[1] == '/')
            os << "\\x2a";
          else if (p.quote && s[0] == '?' && s[1] == '?')
            os << "?\\";
          else
          {
            if (p.quote && (*s == '"' || *s == '\\'))
              os << "\\";
            os << (char)*s;
          }
      }
    }
    if (p.quote)
      os << (char)p.quote;
  }
  else
    os << "NULL";
  return os;
}

////////////////////////////////////////////////////////////////////////////////
//
//      jsoncpp main
//
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{
  if (argc >= 2)
  {
    for (int i = 1; i < argc; i++)
    {
      const char *a = argv[i];
      if ((a[0] == '-' && a[1])
#ifdef WIN32
        || a[0] == '/'
#endif
         )
      {
        bool f = true;
        while (f && *++a)
        {
          switch (*a)
          {
            case 'c':
              coutput = true;
              break;
            case 'e':
              explain = true;
              break;
            case 'f':
              a++;
              f = false;
              if (*a)
                dform = a;
              else if (i < argc)
                dform = argv[++i];
              break;
            case 'i':
              genread = true;
              if (!ifile)
                ifile = "-";
              break;
            case 'k':
              genread = true;
              nocheck = true;
              if (!ifile)
                ifile = "-";
              break;
            case 'l':
              genvars = true;
              break;
            case 'm':
              genmain = true;
              break;
            case 'M':
              minimal = true;
              break;
            case 'O':
              optimal = true;
              break;
            case 'o':
              a++;
              f = false;
              if (*a)
                ofile = a;
              else if (i < argc)
                ofile = argv[++i];
              break;
            case 'p':
              a++;
              f = false;
              if (*a)
                jpath = a;
              else if (i < argc)
                jpath = argv[++i];
              break;
            case 'r':
              a++;
              f = false;
              if (*a)
                jroot = a;
              else if (i < argc)
                jroot = argv[++i];
              break;
            case 'x':
              a++;
              f = false;
              if (*a)
                jcode = a;
              else if (i < argc)
                jcode = argv[++i];
              break;
            case 'y':
              collect = true;
              break;
            case '?':
            case 'h':
              fprintf(stderr,
                  "Usage: jsoncpp [-c] [-e] [-f%%fmt] [-h] [-i] [-l] [-m] [-M] [-O] [-ofile] [-ppath] [-rroot] [-xcode] [-y] [infile]\n\n"
                  "-c      generate C code instead of C++\n"
                  "-e      add explanatory comments to the generated code\n"
                  "-f%%fmt  use %%fmt to format double floats, e.g. -f%%lG\n"
                  "-h      display help message\n"
                  "-i      generate code to inspect node graph parsed from JSON input\n"
                  "-l      generate code for option -i to store values in local variables\n"
                  "-m      generate stand-alone code by adding main()\n"
                  "-M      generate minimal code unadorned with initialization and cleanup\n"
                  "-O      optimize code by factoring common indices when applicable\n"
                  "-ofile  save source code to file\n"
                  "-ppath  generate JSONPath query code for 'path'\n"
                  "-rroot  use 'root' instead of root value 'x' in the generated code\n"
                  "-xcode  generate code that executes 'code' for each JSONPath query result\n"
                  "-y      generate code that yields an array 'y' of JSONPath query results\n"
                  "infile  optional JSON file to parse\n"
                  "-       read JSON from standard input\n\n");
              exit(EXIT_SUCCESS);
            default:
              fprintf(stderr, "jsoncpp: Unknown option %s\n\n", a);
              exit(EXIT_FAILURE);
          }
        }
      }
      else
      {
        if (ifile && strcmp(ifile, "-"))
          fprintf(stderr, "jsoncpp: Input already specified as %s, ignoring %s\n\n", ifile, a);
        else
          ifile = argv[i];
      }
    }
  }
  else
  {
    ifile = "-";
  }

  if (genmain)
    minimal = false;

  soap *ctx = soap_new1(SOAP_C_UTFSTRING);

  if (dform && *dform == '%' && *(dform+1))
    ctx->double_format = dform;

  std::ifstream ifs;

  if (ifile)
  {
    if (!strcmp(ifile, "-"))
    {
      ctx->is = &std::cin;
    }
    else
    {
    ifs.open(ifile, std::ifstream::in);
    if (!ifs.is_open())
    {
      fprintf(stderr, "Cannot open %s for reading\n", ifile);
      exit(EXIT_FAILURE);
    }
    ctx->is = &ifs;
    }
  }

  std::ofstream ofs;

  if (ofile)
  {
    ofs.open(ofile, std::ofstream::out);
    if (!ofs.is_open())
    {
      fprintf(stderr, "Cannot open %s for writing\n", ofile);
      exit(EXIT_FAILURE);
    }
    ctx->os = &ofs;
  }
  else
  {
    ctx->os = &std::cout;
  }

  value v(ctx);

  if (ifile)
  {
    // read plain JSON, no HTTP/MIME headers
    soap_set_imode(ctx, SOAP_ENC_PLAIN);
    if (json_read(ctx, v))
    {
      soap_print_fault(ctx, stderr);
      soap_print_fault_location(ctx, stderr);
      exit(EXIT_FAILURE);
    }
  }

  if (ifile && strcmp(ifile, "-"))
    ifs.close();

  if (!minimal)
  {
    if (coutput)
    {
      *ctx->os <<
        "/* Dependencies:\n"
        "     json.h xml-rpc-iters.h stdsoap2.h\n"
        "     json.c xml-rpc.c stdsoap2.c\n"
        "   Build:\n"
        "     soapcpp2 -c -CSL xml-rpc.h    (generates soapStub.h, soapH.h, soapC.c)\n"
        "     cc ... soapC.c json.c xml-rpc.c stdsoap2.c ...\n"
        "*/\n";
    }
    else
    {
      *ctx->os <<
        "// Dependencies:\n"
        "//   json.h xml-rpc-iters.h stdsoap2.h\n"
        "//   json.cpp xml-rpc.cpp stdsoap2.cpp\n"
        "// Build:\n"
        "//   soapcpp2 -CSL xml-rpc.h    (generates soapStub.h, soapH.h, soapC.cpp)\n"
        "//   c++ ... soapC.cpp json.cpp xml-rpc.cpp stdsoap2.cpp ...\n";
    }
    *ctx->os << "\n#include \"json.h\"\n";
    if (!coutput && jpath && strstr(jpath, ".."))
      *ctx->os << "#include <stack>\n";
  }

  if (genmain)
    *ctx->os <<
      "struct Namespace namespaces[] = {{NULL,NULL,NULL,NULL}};\n"
      "int main(int argc, char **argv)\n";

  if (!minimal)
  {
    if (coutput)
      *ctx->os << "{ /* Generated by jsoncpp";
    else
      *ctx->os << "{ // Generated by jsoncpp";
    for (int i = 1; i < argc; ++i)
      *ctx->os << " " << putstr(argv[i], 0);
    if (coutput)
      *ctx->os <<
	" */\n"
	"  /* jsoncpp tool Copyright (C) 2000-2016, Robert van Engelen, Genivia Inc. */\n";
    else
      *ctx->os <<
	"\n"
	"  // jsoncpp tool Copyright (C) 2000-2016, Robert van Engelen, Genivia Inc.\n";
    *ctx->os <<
      "  struct soap *ctx = soap_new1(SOAP_C_UTFSTRING | SOAP_XML_INDENT);\n"
      "  ctx->double_format = \"" << (dform ? dform : "%lG") << "\";\n\n";
  }

  if (coutput)
  {
    *ctx->os << "  struct value *" << jroot << " = new_value(ctx);\n";

    if ((ifile && genread && genvars) || jpath)
    {
      if (explain)
        *ctx->os << "  /* index to inspect structs and arrays */\n";
      *ctx->os << "  int j;\n";
    }

    std::string path = "$";

    if (ifile && !genread)
    {
      out_gen_c(ctx, v, jroot, path, 2);
    }
    else
    {
      if (explain)
        *ctx->os << "  /* parse JSON into value " << jroot << " */\n";
      *ctx->os << "  json_read(ctx, " << jroot << ");\n";
      if (!minimal)
        *ctx->os <<
          "  if (" << jroot << "->soap->error)\n"
          "    exit(EXIT_FAILURE); /* error parsing JSON */\n";

      if (ifile)
      {
        *ctx->os << "  #define USE_VAL(path, val) printf(\"%s = %s\\n\", path, *string_of(val))\n";

        in_gen_c(ctx, v, jroot, path, 2);
      }
    }

    if (jpath)
    {
      *ctx->os << "  /* JSONPath: " << putstr(jpath, 0) << " */\n";

      if (!jcode)
      {
        if (collect)
        {
          if (explain)
            *ctx->os << "  /* JSONPath query yields an array 'y' of JSONPath query results: */\n";
          *ctx->os <<
            "  struct value *y = new_value(ctx);\n"
            "  set_size(y, 0);\n"
            "  #define QUERY_YIELD(val) *nth_value(y, has_size(y)) = *val\n";
        }
        else
        {
          if (explain)
            *ctx->os << "  /* JSONPath query yields are shown for each JSONPath result: */\n";
          *ctx->os << "  #define QUERY_YIELD(val) json_write(ctx, val), putchar('\\n')\n";
        }
      }
      *ctx->os << "  #define QUERY_STACK_SIZE 65536\n";
      *ctx->os << "  #define QUERY_STACK_OVFL exit(EXIT_FAILURE)\n";
      std::string root(jroot);
      if (explain)
      {
        *ctx->os <<
          "  /* Synopsis of variables used:\n"
          "         " << root << "\troot node value pointer\n"
          << ( collect ? "         y\tyield array of query results\n" : "" ) <<
          "         v\tcurrent node value pointer to use in [(expr)]\n"
          "         i\tarray/struct loop index\n"
          "         j\tarray/struct index\n"
          "         k\tswitch case counter / loop end\n"
          "         p\tvalue pointer to current node\n"
          "         q\tvalue pointer to current node\n"
          "         s\tstack pointer\n"
          "         S\tvalue stack for recursive descent\n"
          "     Synopsis of constants used:\n"
          "         QUERY_STACK_SIZE\tmax stack size for recursive descent\n"
          "         QUERY_STACK_OVFL\tcode to execute on stack overflow\n"
          "  */\n";
      }

      path_gen_c(ctx, jpath, root, 2, false);

      if (!minimal && collect)
        *ctx->os << "  json_write(ctx, y), putchar('\\n');\n";
    }
    else if (!minimal)
    {
      if (explain)
        *ctx->os << "  /* output value " << jroot << " in JSON */\n";
      *ctx->os << "  json_write(ctx, " << jroot << "), putchar('\\n');\n";
    }
    if (!minimal)
    {
      *ctx->os <<
        "\n"
        "  soap_end(ctx);  /* delete data */\n"
        "  soap_free(ctx); /* free context */\n"
        << (genmain ? "  return 0;\n" : "") <<
        "}\n";
    }
  }
  else
  {
    *ctx->os << "  value " << jroot << "(ctx);\n";

    std::string path = "$";

    if (ifile && !genread)
    {
      out_gen_cpp(ctx, v, jroot, path, 2);
    }
    else
    {
      if (explain)
        *ctx->os << "  // parse JSON into value " << jroot << "\n";
      *ctx->os << "  std::cin >> " << jroot << ";\n";
      if (!minimal)
        *ctx->os <<
          "  if (" << jroot << ".soap->error)\n"
          "    exit(EXIT_FAILURE); // error parsing JSON\n";

      if (ifile)
      {
        *ctx->os << "  #define USE_VAL(path, val) std::cout << path << \" = \" << val << std::endl\n";

        in_gen_cpp(ctx, v, jroot, path, 2);
      }
    }

    if (jpath)
    {
      *ctx->os << "  // JSONPath: " << jpath << "\n";

      if (optimal)
      {
        if (explain)
          *ctx->os << "  // index to inspect structs and arrays\n";
        *ctx->os << "  int j;\n";
      }

      if (!jcode)
      {
        if (collect)
        {
          if (explain)
            *ctx->os << "  // JSONPath query yields an array 'y' of JSONPath query results:\n";
          *ctx->os <<
            "  value y(ctx);\n"
            "  y.size(0);\n"
            "  #define QUERY_YIELD(val) y[y.size()] = val\n";
        }
        else
        {
          if (explain)
            *ctx->os << "  // JSONPath query yields are shown for each JSONPath result:\n";
          *ctx->os << "  #define QUERY_YIELD(val) std::cout << val << std::endl\n";
        }
      }
      std::string root(jroot);
      if (explain)
      {
        *ctx->os <<
          "  // Synopsis of variables used:\n"
          "  //     " << root << "\troot node value\n"
          << ( collect ? "  //     y\tyield array of query results\n" : "" ) <<
          "  //     v\tcurrent node value to use in [(expr)]\n"
          "  //     i\titerator over arrays and structs\n"
          "  //     j\tarray/struct index\n"
          "  //     k\tswitch case counter / loop end\n"
          "  //     p\tvalue pointer to current node\n"
          "  //     q\tvalue pointer to current node\n"
          "  //     r\tvalue reference to current node\n"
          "  //     s\tvalue reference to current node\n"
          "  //     S\tvalue stack for recursive descent\n";
      }

      path_gen_cpp(ctx, jpath, root, 2, false);

      if (!minimal && collect)
        *ctx->os << "  std::cout << y << std::endl;\n";
    }
    else if (!minimal)
    {
      if (explain)
        *ctx->os << "  // output value " << jroot << " in JSON\n";
      *ctx->os << "  std::cout << " << jroot << " << std::endl;\n";
    }

    if (!minimal)
    {
      *ctx->os <<
        "\n"
        "  soap_destroy(ctx); // delete objects\n"
        "  soap_end(ctx);     // delete data\n"
        "  soap_free(ctx);    // free context\n"
        << (genmain ? "  return 0;\n" : "") <<
        "}\n";
    }
  }

  if (ofile)
    ofs.close();

  soap_destroy(ctx);
  soap_end(ctx);
  soap_free(ctx);

  return 0;
}

////////////////////////////////////////////////////////////////////////////////
//
//      C gen writer/reader from infile
//
////////////////////////////////////////////////////////////////////////////////

static void out_gen_c(soap *ctx, value& v, std::string lhs, const std::string& path, int k)
{
  switch (v.__type)
  {
    case SOAP_TYPE__boolean:
      if (explain)
        indent(ctx, k) << "// " << putstr(path, 0) << " = " << putstr((const char*)v, 0) << "\n";
      indent(ctx, k) << "*bool_of(" << lhs << ") = " << (v.is_true() ? 1 : 0) << ";\n";
      break;
    case SOAP_TYPE__i4:
    case SOAP_TYPE__int:
      if (explain)
        indent(ctx, k) << "// " << putstr(path, 0) << " = " << putstr((const char*)v, 0) << "\n";
      indent(ctx, k) << "*int_of(" << lhs << ") = ";
      json_send(ctx, v);
      *ctx->os << ";\n";
      break;
    case SOAP_TYPE__double:
      if (explain)
        indent(ctx, k) << "// " << putstr(path, 0) << " = " << putstr((const char*)v, 0) << "\n";
      indent(ctx, k) << "*double_of(" << lhs << ") = ";
      json_send(ctx, v);
      *ctx->os << ";\n";
      break;
    case SOAP_TYPE__string:
      if (explain)
        indent(ctx, k) << "// " << putstr(path, 0) << " = " << putstr((const char*)v, 0) << "\n";
      indent(ctx, k) << "*string_of(" << lhs << ") = " << putstr((const char*)v) << ";\n";
      break;
    case SOAP_TYPE__struct:
      if (v.empty())
      {
        indent(ctx, k) << "set_struct(" << lhs << ");\n";
      }
      else
      {
        _struct& s = v;
        for (_struct::iterator i = s.begin(); i != s.end(); ++i)
        {
          std::string lhsidx;
          if (optimal && (i->is_struct() || i->is_array()) && !i->empty())
          {
            lhsidx = gen_ident(lhs, i.index());
            indent(ctx, k) << "{\n";
            k += 2;
            indent(ctx, k) << "struct value *" << lhsidx << " = value_at(" << lhs << ", " << putstr(i.index()) << ");\n";
          }
          else
          {
            lhsidx = "value_at(";
            lhsidx.append(lhs).append(", ").append(putstrname(i.index())).append(")");
          }
          std::string newpath = path;
          newpath.append(".").append(putname(i.index()));
          out_gen_c(ctx, *i, lhsidx, newpath, k);
          if (optimal && (i->is_struct() || i->is_array()) && !i->empty())
          {
            k -= 2;
            indent(ctx, k) << "}\n";
          }
        }
      }
      break;
    case SOAP_TYPE__array:
      if (v.empty())
      {
        indent(ctx, k) << "set_size(" << lhs << ", 0);\n";
      }
      else
      {
        _array& a = v;
        for (_array::iterator i = a.begin(); i != a.end(); ++i)
        {
          const char *s = soap_int2s(ctx, i.index());
          std::string lhsidx;
          if (optimal && (i->is_struct() || i->is_array()) && !i->empty())
          {
            lhsidx = lhs;
            lhsidx.append(s);
            indent(ctx, k) << "{\n";
            k += 2;
            indent(ctx, k) << "struct value *" << lhsidx << " = nth_value(" << lhs << ", " << s << ");\n";
          }
          else
          {
            lhsidx = "nth_value(";
            lhsidx.append(lhs).append(", ").append(s).append(")");
          }
          std::string newpath = path;
          newpath.append("[").append(s).append("]");
          out_gen_c(ctx, *i, lhsidx, newpath, k);
          if (optimal && (i->is_struct() || i->is_array()) && !i->empty())
          {
            k -= 2;
            indent(ctx, k) << "}\n";
          }
        }
      }
      break;
    default:
      break;
  }
}

static void in_gen_c(soap *ctx, value& v, std::string lhs, const std::string& path, int k)
{
  switch (v.__type)
  {
    case SOAP_TYPE__boolean:
      if (explain)
        indent(ctx, k) << "/* " << putstr(path, 0) << " bool value, is_true(" << lhs << ") and is_false(" << lhs << ") */\n";
      indent(ctx, k) << "USE_VAL(" << putstr(path) << ", " << lhs << ");\n";
      break;
    case SOAP_TYPE__i4:
    case SOAP_TYPE__int:
      if (explain)
        indent(ctx, k) << "/* " << putstr(path, 0) << " int value, is_int(" << lhs << ") and *int_of(" << lhs << ") */\n";
      indent(ctx, k) << "USE_VAL(" << putstr(path) << ", " << lhs << ");\n";
      break;
    case SOAP_TYPE__double:
      if (explain)
        indent(ctx, k) << "/* " << putstr(path, 0) << " double value, is_double(" << lhs << ") and *double_of(" << lhs << ") */\n";
      indent(ctx, k) << "USE_VAL(" << putstr(path) << ", " << lhs << ");\n";
      break;
    case SOAP_TYPE__string:
      if (explain)
        indent(ctx, k) << "/* " << putstr(path, 0) << " string value, is_string(" << lhs << ") and *string_of(" << lhs << ") */\n";
      indent(ctx, k) << "USE_VAL(" << putstr(path) << ", " << lhs << ");\n";
      break;
    case SOAP_TYPE__struct:
      {
        _struct& s = v;
        if (explain)
          indent(ctx, k) << "/* " << putstr(path, 0) << " object, is_struct(" << lhs << ") */\n";
        for (_struct::iterator i = s.begin(); i != s.end(); ++i)
        {
          std::string lhsidx;
          if (genvars)
          {
            lhsidx = gen_ident(lhs, i.index());
          }
          else
          {
            lhsidx = "value_at(";
            lhsidx.append(lhs).append(", ").append(putstrname(i.index())).append(")");
          }
          std::string newpath = path;
          newpath.append(".").append(putname(i.index()));
          if (!nocheck)
          {
            if (genvars)
              indent(ctx, k) << "if ((j = nth_at(" << lhs << ", " << putstr(i.index()) << ")) >= 0)\n";
            else
              indent(ctx, k) << "if (nth_at(" << lhs << ", " << putstr(i.index()) << ") >= 0)\n";
            if (explain || genvars || i->is_struct() || i->is_array())
              indent(ctx, k) << "{\n";
            if (genvars)
              indent(ctx, k + 2) << "struct value *" << lhsidx << " = nth_value(" << lhs << ", j);\n";
            in_gen_c(ctx, *i, lhsidx, newpath, k + 2);
            if (explain || genvars || i->is_struct() || i->is_array())
              indent(ctx, k) << "}\n";
          }
          else
          {
            if (genvars)
              indent(ctx, k) << "struct value *" << lhsidx << " = nth_value(" << lhs << ", j);\n";
            in_gen_c(ctx, *i, lhsidx, newpath, k);
          }
        }
      }
      break;
    case SOAP_TYPE__array:
      {
        if (explain)
          indent(ctx, k) << "/* " << putstr(path, 0) << " array, is_array(" << lhs << ") */\n";
        std::string idx;
        std::string lhsidx;
        if (genvars)
        {
          idx = "i";
          lhsidx = lhs;
          lhsidx.append(idx);
        }
        else
        {
          idx = gen_counter(k/2 - 1);
          lhsidx = "nth_value(";
          lhsidx.append(lhs).append(", ").append(idx).append(")");
        }
        indent(ctx, k) << "int " << idx << ";\n";
        indent(ctx, k) << "for (" << idx << " = 0; " << idx << " < has_size(" << lhs << "); " << idx << "++)\n";
        std::string newpath = path;
        newpath.append("[]");
        if (explain || genvars || v[0].is_struct() || v[0].is_array())
          indent(ctx, k) << "{\n";
        if (genvars)
          indent(ctx, k + 2) << "struct value *" << lhsidx << " = nth_value(" << lhs << ", " << idx << ");\n";
        in_gen_c(ctx, v[0], lhsidx, newpath, k + 2);
        if (explain || genvars || v[0].is_struct() || v[0].is_array())
          indent(ctx, k) << "}\n";
      }
      break;
    default:
      indent(ctx, k) << "; // null\n";
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////
//
//      C++ gen writer/reader from infile
//
////////////////////////////////////////////////////////////////////////////////

static void out_gen_cpp(soap *ctx, value& v, std::string lhs, const std::string& path, int k)
{
  switch (v.__type)
  {
    case SOAP_TYPE__boolean:
    case SOAP_TYPE__i4:
    case SOAP_TYPE__int:
    case SOAP_TYPE__double:
      if (explain)
        indent(ctx, k) << "// " << putstr(path, 0) << " = " << putstr((const char*)v, 0) << "\n";
      indent(ctx, k) << lhs << " = ";
      json_send(ctx, v);
      if (v.__type == SOAP_TYPE__int)
        *ctx->os << "LL";
      *ctx->os << ";\n";
      break;
    case SOAP_TYPE__string:
      if (explain)
        indent(ctx, k) << "// " << putstr(path, 0) << " = " << putstr((const char*)v, 0) << "\n";
      indent(ctx, k) << lhs << " = " << putstr((const char*)v) << ";\n";
      break;
    case SOAP_TYPE__struct:
      if (v.empty())
      {
        indent(ctx, k) << lhs << " = _struct(ctx);\n";
      }
      else
      {
        _struct& s = v;
        for (_struct::iterator i = s.begin(); i != s.end(); ++i)
        {
          std::string lhsidx;
          if (optimal && (i->is_struct() || i->is_array()) && !i->empty())
          {
            lhsidx = gen_ident(lhs, i.index());
            indent(ctx, k) << "{\n";
            k += 2;
            indent(ctx, k) << "value& " << lhsidx << " = " << lhs << "[" << putstr(i.index()) << "];\n";
          }
          else
          {
            lhsidx = lhs;
            lhsidx.append("[").append(putstrname(i.index())).append("]");
          }
          std::string newpath = path;
          newpath.append(".").append(putname(i.index()));
          out_gen_cpp(ctx, *i, lhsidx, newpath, k);
          if (optimal && (i->is_struct() || i->is_array()) && !i->empty())
          {
            k -= 2;
            indent(ctx, k) << "}\n";
          }
        }
      }
      break;
    case SOAP_TYPE__array:
      if (v.empty())
      {
        indent(ctx, k) << lhs << ".size(0);\n";
      }
      else
      {
        _array& a = v;
        for (_array::iterator i = a.begin(); i != a.end(); ++i)
        {
          const char *s = soap_int2s(ctx, i.index());
          std::string lhsidx;
          if (optimal && (i->is_struct() || i->is_array()) && !i->empty())
          {
            lhsidx = lhs;
            lhsidx.append(s);
            indent(ctx, k) << "{\n";
            k += 2;
            indent(ctx, k) << "value& " << lhsidx << " = " << lhs << "[" << s << "];\n";
          }
          else
          {
            lhsidx = lhs;
            lhsidx.append("[").append(s).append("]");
          }
          std::string newpath = path;
          newpath.append("[").append(s).append("]");
          out_gen_cpp(ctx, *i, lhsidx, newpath, k);
          if (optimal && (i->is_struct() || i->is_array()) && !i->empty())
          {
            k -= 2;
            indent(ctx, k) << "}\n";
          }
        }
      }
      break;
    default:
      break;
  }
}

static void in_gen_cpp(soap *ctx, value& v, std::string lhs, const std::string& path, int k)
{
  switch (v.__type)
  {
    case SOAP_TYPE__boolean:
      if (explain)
        indent(ctx, k) << "// " << putstr(path, 0) << " bool value\n";
      indent(ctx, k) << "USE_VAL(" << putstr(path) << ", " << lhs << ");\n";
      break;
    case SOAP_TYPE__i4:
    case SOAP_TYPE__int:
      if (explain)
        indent(ctx, k) << "// " << putstr(path, 0) << " int value\n";
      indent(ctx, k) << "USE_VAL(" << putstr(path) << ", " << lhs << ");\n";
      break;
    case SOAP_TYPE__double:
      if (explain)
        indent(ctx, k) << "// " << putstr(path, 0) << " double value\n";
      indent(ctx, k) << "USE_VAL(" << putstr(path) << ", " << lhs << ");\n";
      break;
    case SOAP_TYPE__string:
      if (explain)
        indent(ctx, k) << "// " << putstr(path, 0) << " string value\n";
      indent(ctx, k) << "USE_VAL(" << putstr(path) << ", " << lhs << ");\n";
      break;
    case SOAP_TYPE__struct:
      {
        _struct& s = v;
        if (explain)
          indent(ctx, k) << "// " << putstr(path, 0) << " object\n";
        for (_struct::iterator i = s.begin(); i != s.end(); ++i)
        {
          std::string lhsidx;
          if (genvars)
          {
            lhsidx = gen_ident(lhs, i.index());
          }
          else
          {
            lhsidx = lhs;
            lhsidx.append("[").append(putstrname(i.index())).append("]");
          }
          std::string newpath = path;
          newpath.append(".").append(putname(i.index()));
          if (!nocheck)
          {
            indent(ctx, k) << "if (" << lhs << ".has(" << putstr(i.index()) << "))\n";
            if (explain || genvars || i->is_struct() || i->is_array())
              indent(ctx, k) << "{\n";
            if (genvars)
              indent(ctx, k + 2) << "value& " << lhsidx << " = " << lhs << "[" << putstr(i.index()) << "];\n";
            in_gen_cpp(ctx, *i, lhsidx, newpath, k + 2);
            if (explain || genvars || i->is_struct() || i->is_array())
              indent(ctx, k) << "}\n";
          }
          else
          {
            if (genvars)
              indent(ctx, k) << "value& " << lhsidx << " = " << lhs << "[" << putstr(i.index()) << "];\n";
            in_gen_cpp(ctx, *i, lhsidx, newpath, k);
          }
        }
      }
      break;
    case SOAP_TYPE__array:
      {
        if (explain)
          indent(ctx, k) << "// " << putstr(path, 0) << " array\n";
        std::string idx;
        std::string lhsidx;
        if (genvars)
        {
          idx = "i";
          lhsidx = lhs;
          lhsidx.append(idx);
        }
        else
        {
          idx = gen_counter(k/2 - 1);
          lhsidx = lhs;
          lhsidx.append("[").append(idx).append("]");
        }
        indent(ctx, k) << "for (int " << idx << " = 0; " << idx << " < " << lhs << ".size(); " << idx << "++)\n";
        std::string newpath = path;
        newpath.append("[]");
        if (explain || genvars || v[0].is_struct() || v[0].is_array())
          indent(ctx, k) << "{\n";
        if (genvars)
          indent(ctx, k + 2) << "value& " << lhsidx << " = " << lhs << "[" << idx << "];\n";
        in_gen_cpp(ctx, v[0], lhsidx, newpath, k + 2);
        if (explain || genvars || v[0].is_struct() || v[0].is_array())
          indent(ctx, k) << "}\n";
      }
      break;
    default:
      indent(ctx, k) << "; // null\n";
      break;
  }
}

////////////////////////////////////////////////////////////////////////////////
//
//      C gen JSONPath from -ppath
//
////////////////////////////////////////////////////////////////////////////////

static void path_gen_c(struct soap *ctx, const char *jpath, std::string& v, int k, bool throwup)
{
  if (*jpath == '$' || *jpath == '@') // strip $ and @ from path
    ++jpath;
  while (isspace(*jpath) || (*jpath == '.' && *(jpath + 1) != '.')) // skip space and single .
    ++jpath;
  if (*jpath == '.') // path: ..
  {
    ++jpath;
    if (*jpath == '.')
    {
      if (explain)
        indent(ctx, k) << "/* iterate over descendants of current node " << v << " to match " << jpath << " */\n";
      indent(ctx, k) << "struct value *S[QUERY_STACK_SIZE], **s = S;\n";
      indent(ctx, k) << "*s++ = " << v << ";\n";
      indent(ctx, k) << "while (S < s)\n";
      indent(ctx, k) << "{\n";
      indent(ctx, k + 2) << "struct value *p = *--s;\n";
      indent(ctx, k + 2) << "int i;\n";
      indent(ctx, k + 2) << "for (i = has_size(p)-1; i >= 0; --i)\n";
      indent(ctx, k + 2) << "{\n";
      indent(ctx, k + 4) << "if (s < S + QUERY_STACK_SIZE)\n";
      indent(ctx, k + 6) << "*s++ = nth_value(p, i);\n";
      indent(ctx, k + 4) << "else\n";
      indent(ctx, k + 6) << "QUERY_STACK_OVFL;\n";
      indent(ctx, k + 2) << "}\n";
      indent(ctx, k + 2) << "{\n";
      v = "p";
      path_gen_c(ctx, jpath, v, k + 4, throwup);
      indent(ctx, k + 2) << "}\n";
      indent(ctx, k) << "}\n";
    }
    else
    {
      fprintf(stderr, "jsoncpp: JSONPath unexpected end at '.'\n\n");
      exit(EXIT_FAILURE);
    }
  }
  else if (*jpath == '[') // path: [*]..., [?x], or [(),:]...
  {
    ++jpath;
    if (*jpath == '*') // path: [*]...
    {
      ++jpath;
      if (*jpath != ']')
      {
        fprintf(stderr, "jsoncpp: JSONPath ']' expected at ...->%s\n\n", jpath);
        exit(EXIT_FAILURE);
      }
      ++jpath;
      if (explain)
        indent(ctx, k) << "/* iterate over current array/struct node " << v << " to match " << jpath << " */\n";
      indent(ctx, k) << "int i, k = has_size(" << v << ");\n";
      indent(ctx, k) << "for (i = 0; i < k; ++i)\n";
      indent(ctx, k) << "{\n";
      if (v[0] == 'p')
      {
        indent(ctx, k + 2) << "struct value *q = nth_value(" << v << ", i);\n";
        v = "q";
      }
      else
      {
        indent(ctx, k + 2) << "struct value *p = nth_value(" << v << ", i);\n";
        v = "p";
      }
      path_gen_c(ctx, jpath, v, k + 2, throwup);
      indent(ctx, k) << "}\n";
    }
    else if (*jpath == '?') // path: [?x]... where x is a C++ bool expression
    {
      ++jpath;
      if (*jpath != '(')
      {
        fprintf(stderr, "jsoncpp: JSONPath '(' expected at ...->%s\n\n", jpath);
        exit(EXIT_FAILURE);
      }
      ++jpath;
      int nest = 0;
      const char *s;
      for (s = jpath; *s; ++s)
      {
        if (*s == ')')
        {
          if (nest == 0)
            break;
          --nest;
        }
        else if (*s == '(')
          ++nest;
      }
      if (*s != ')' || *(s + 1) != ']')
      {
        fprintf(stderr, "jsoncpp: JSONPath ')]' expected at ...->%s\n\n", jpath);
        exit(EXIT_FAILURE);
      }
      std::string code(jpath, s - jpath);
      jpath = s + 2;
      if (explain)
        indent(ctx, k) << "/* filter current node " << v << " with 'if (" << code << ")' to match " << jpath << " */\n";
      indent(ctx, k) << "struct value *v = " << v << ";\n";
      indent(ctx, k) << "if (" << code << ")\n";
      indent(ctx, k) << "{\n";
      path_gen_c(ctx, jpath, v, k + 2, throwup);
      indent(ctx, k) << "}\n";
    }
    else // path: [(),:]...
    {
      std::string u;
      int subs = 0;
      std::string code;
      std::string name;
      int start, end, step;
      while (true)
      {
        ++subs;
        code.clear();
        name.clear();
        start = end = step = 0;
        if (*jpath == '(')
        {
          ++jpath;
          int nest = 0;
          const char *s;
          for (s = jpath; *s; ++s)
          {
            if (*s == ')')
            {
              if (nest == 0)
                break;
              --nest;
            }
            else if (*s == '(')
              ++nest;
          }
          if (*s != ')')
          {
            fprintf(stderr, "jsoncpp: JSONPath ')' expected at ...->%s\n\n", jpath);
            exit(EXIT_FAILURE);
          }
          code = std::string(jpath, s - jpath);
          jpath = s + 1;
        }
        else if (*jpath == ':')
        {
          ++jpath;
          bool hasend = true;
          if (*jpath == '-' || *jpath == '+' || isdigit(*jpath))
            end = soap_strtol(jpath, (char**)&jpath, 10);
          else
            hasend = false;
          if (*jpath == ':')
            step = soap_strtol(jpath + 1, (char**)&jpath, 10);
          else
            step = 1;
          if (step > 0) // end is exclusive (Python array slicing)
            --end;
          else if (hasend)
            ++end;
          if (step < 0)
            start = -1;
        }
        else if (*jpath == '\'' || *jpath == '"' || isalpha(*jpath))
        {
          name = getname(&jpath);
        }
        else if (*jpath == '-' || *jpath == '+' || isdigit(*jpath))
        {
          start = soap_strtol(jpath, (char**)&jpath, 10);
          if (*jpath == ':')
          {
            ++jpath;
            bool hasend = true;
            if (*jpath == '-' || *jpath == '+' || isdigit(*jpath))
              end = soap_strtol(jpath, (char**)&jpath, 10);
            else
              hasend = false;
            if (*jpath == ':')
              step = soap_strtol(jpath + 1, (char**)&jpath, 10);
            else
              step = 1;
            if (step > 0) // end is exclusive (Python array slicing)
              --end;
            else if (hasend)
              ++end;
          }
        }
        if ((step > 0 && !(start <= end || (start >= 0 && end < 0))) ||
            (step < 0 && !(start >= end || (start < 0 && end >= 0))))
          {
            fprintf(stderr, "jsoncpp: JSONPath inverted bounds at ...->%s\n\n", jpath);
            exit(EXIT_FAILURE);
          }
        if (subs == 1 && *jpath != ',') // path: [x]... with x a name or index or [start]:[end][:step] slice
          break;
        if (subs == 1) // path: [x,...]... with x a name or index or [start]:[end][:step] slice
        {
          if (explain)
            indent(ctx, k) << "/* for each case */\n";
          indent(ctx, k) << "int i, k = 1;\n";
          if (v[0] == 'p')
          {
            indent(ctx, k) << "struct value *q;\n";
            u = "q";
          }
          else
          {
            indent(ctx, k) << "struct value *p;\n";
            u = "p";
          }
          indent(ctx, k) << "do\n";
          indent(ctx, k) << "{\n";
          indent(ctx, k + 2) << "switch (k)\n";
          indent(ctx, k + 2) << "{\n";
        }
        indent(ctx, k + 4) << "case " << 2*subs-1 << ":\n";
        if (!code.empty())
        {
          indent(ctx, k + 4) << "{\n";
          if (explain)
            indent(ctx, k + 6) << "/* if current node " << v << " is an array/struct with field/element [(" << code << ")] ... */\n";
          indent(ctx, k + 6) << "struct value *v = " << v << ";\n";
          indent(ctx, k + 6) << "k = " << 2*subs+1 << ";\n";
          indent(ctx, k + 6) << u << " = (" << code << ");\n"; // only value pointer expressions in C
          indent(ctx, k + 6) << "if (" << u << ")\n";
          indent(ctx, k + 8) << "break;\n";
          indent(ctx, k + 4) << "}\n";
        }
        else if (!name.empty())
        {
          if (explain)
            indent(ctx, k + 6) << "/* if current node " << v << " is a struct with field '" << name << "' ... */\n";
          indent(ctx, k + 6) << "k = " << 2*subs+1 << ";\n";
          indent(ctx, k + 6) << "if ((i = nth_at(" << v << ", " << putstr(name) << ")) >= 0)\n";
          indent(ctx, k + 6) << "{\n";
          indent(ctx, k + 8) << u << " = nth_value(" << v << ", i);\n";
          indent(ctx, k + 8) << "break;\n";
          indent(ctx, k + 6) << "}\n";
        }
        else if (step > 0)
        {
          if (explain)
            indent(ctx, k + 6) << "/* if current node " << v << " is an array then iterate from " << start << " to " << end << " by " << step << " ... */\n";
          indent(ctx, k + 6) << "if (!is_array(" << v << ")\n";
          indent(ctx, k + 6) << "{\n";
          indent(ctx, k + 8) << "k = " << 2*subs+1 << ";\n";
          indent(ctx, k + 8) << "continue;\n";
          indent(ctx, k + 6) << "}\n";
          if (start < 0)
            indent(ctx, k + 6) << "i = (" << start << " >= -has_size(" << v << ") ? " << start << " : -has_size(" << v << "))-" << step << ";\n";
          else
            indent(ctx, k + 6) << "i = " << start-step << ";\n";
          indent(ctx, k + 6) << "k = " << 2*subs << ";\n";
          indent(ctx, k + 4) << "case " << 2*subs << ":\n";
          indent(ctx, k + 6) << "i += " << step << ";\n";
          if (start >= 0 && end < 0) // : 0 <= start <= size+end where end < 0
            indent(ctx, k + 6) << "if (i <= has_size(" << v << ")+" << end << ")\n";
          else if (start >= 0 && end >= 0) // : 0 <= start <= end < size
            indent(ctx, k + 6) << "if (i <= " << end << " && i < has_size(" << v << "))\n";
          else // start < 0 && end < 0 : -size <= start <= end < 0
            indent(ctx, k + 6) << "if (i <= " << end << ")\n";
          indent(ctx, k + 6) << "{\n";
          indent(ctx, k + 8) << u << " = nth_value(" << v << ", i);\n";
          indent(ctx, k + 8) << "break;\n";
          indent(ctx, k + 6) << "}\n";
          indent(ctx, k + 6) << "else\n";
          indent(ctx, k + 8) << "k = " << 2*subs+1 << ";\n";
        }
        else if (step < 0)
        {
          if (explain)
            indent(ctx, k + 6) << "/* if current node " << v << " is an array then iterate from " << start << " to " << end << " by " << step << " ... */\n";
          indent(ctx, k + 6) << "if (!is_array(" << v << "))\n";
          indent(ctx, k + 6) << "{\n";
          indent(ctx, k + 8) << "k = " << 2*subs+1 << ";\n";
          indent(ctx, k + 8) << "continue;\n";
          indent(ctx, k + 6) << "}\n";
          if (start < 0 && end >= 0)
            indent(ctx, k + 6) << "i = has_size(" << v << ")+" << start-step << ";\n";
          else if (start >= 0 && end >= 0)
            indent(ctx, k + 6) << "i = (" << start << " < has_size(" << v << ") ? " << start << " : has_size(" << v << ")-1)+" << -step << ";\n";
          else // start < 0 && end < 0
            indent(ctx, k + 6) << "i = " << start-step << ";\n";
          indent(ctx, k + 6) << "k = " << 2*subs << ";\n";
          indent(ctx, k + 4) << "case " << 2*subs << ":\n";
          indent(ctx, k + 6) << "i -= " << -step << ";\n";
          if (start < 0 && end >= 0) // : 0 <= end <= size+start where start < 0
            indent(ctx, k + 6) << "if (i >= " << end << ")\n";
          else if (start >= 0 && end >= 0) // : 0 <= end <= start < size
            indent(ctx, k + 6) << "if (i >= " << end << ")\n";
          else // start < 0 && end < 0 : -size <= end <= start < 0
            indent(ctx, k + 6) << "if (i >= " << end << " && i >= -has_size(" << v << "))\n";
          indent(ctx, k + 6) << "{\n";
          indent(ctx, k + 8) << u << " = nth_value(" << v << ", i);\n";
          indent(ctx, k + 8) << "break;\n";
          indent(ctx, k + 6) << "}\n";
          indent(ctx, k + 6) << "else\n";
          indent(ctx, k + 8) << "k = " << 2*subs+1 << ";\n";
        }
        else
        {
          if (explain)
            indent(ctx, k + 6) << "/* if current node " << v << " is an array with element [" << start << "] ... */\n";
          indent(ctx, k + 6) << "k = " << 2*subs+1 << ";\n";
          indent(ctx, k + 6) << "if ((i = nth_nth(" << v << ", " << start << ")) >= 0)\n";
          indent(ctx, k + 6) << "{\n";
          indent(ctx, k + 8) << u << " = nth_value(" << v << ", i);\n";
          indent(ctx, k + 8) << "break;\n";
          indent(ctx, k + 6) << "}\n";
        }
        if (*jpath != ',')
          break;
        ++jpath;
      }
      if (subs == 0 && *jpath == ']')
      {
        fprintf(stderr, "jsoncpp: JSONPath empty '[]' at ...->%s\n\n", jpath);
        exit(EXIT_FAILURE);
      }
      if (*jpath != ']')
      {
        fprintf(stderr, "jsoncpp: JSONPath ']' expected at ...->%s\n\n", jpath);
        exit(EXIT_FAILURE);
      }
      ++jpath;
      if (subs == 1) // path: [x]... with x a name or index or [start]:[end][:step] range
      {
        if (!code.empty())
        {
          if (explain)
            indent(ctx, k) << "/* if current node " << v << " is an array/struct with field/element [(" << code << ")] then match " << jpath << " */\n";
          indent(ctx, k) << "struct value *v = " << v << ";\n";
          indent(ctx, k) << "struct value *p = (" << code << ");\n"; // only value pointer expressions in C
          indent(ctx, k) << "if (p)\n";
          indent(ctx, k) << "{\n";
          v = "p";
          path_gen_c(ctx, jpath, v, k + 2, throwup);
          indent(ctx, k) << "}\n";
        }
        else if (!name.empty())
        {
          if (explain)
            indent(ctx, k) << "/* if current node " << v << " is a struct with field '" << name << "' then match " << jpath << " */\n";
          indent(ctx, k) << "if ((j = nth_at(" << v << ", " << putstr(name) << ")) >= 0)\n";
          indent(ctx, k) << "{\n";
          if (v[0] == 'p')
          {
            indent(ctx, k + 2) << "struct value *q = nth_value(" << v << ", j);\n";
            v = "q";
          }
          else
          {
            indent(ctx, k + 2) << "struct value *p = nth_value(" << v << ", j);\n";
            v = "p";
          }
          path_gen_c(ctx, jpath, v, k + 2, throwup);
          indent(ctx, k) << "}\n";
        }
        else if (step > 0)
        {
          if (explain)
            indent(ctx, k) << "/* if current node " << v << " is an array then iterate from " << start << " to " << end << " by " << step << " to match " << jpath << " */\n";
          indent(ctx, k) << "if (is_array(" << v << "))\n";
          indent(ctx, k) << "{\n";
          if (start >= 0 && end < 0)
          {
            indent(ctx, k + 2) << "int i, k = has_size(" << v << ")-" << -end << ";\n";
            indent(ctx, k + 2) << "for (i = " << start << "; i <= k; i += " << step << ")\n";
          }
          else if (start >= 0 && end >= 0)
          {
            indent(ctx, k + 2) << "int i, k = (" << end << " < has_size(" << v << ") ? " << end << " : has_size(" << v << ")-1);\n";
            indent(ctx, k + 2) << "for (i = " << start << "; i <= k; i += " << step << ")\n";
          }
          else // start < 0 && end < 0
          {
            indent(ctx, k + 2) << "int i, k = (" << start << " >= -has_size(" << v << ") ? " << start << " : -has_size(" << v << "));\n";
            indent(ctx, k + 2) << "for (i = k; i <= " << end << "; i += " << step << ")\n";
          }
          indent(ctx, k + 2) << "{\n";
          if (v[0] == 'p')
          {
            indent(ctx, k + 4) << "struct value *q = nth_value(" << v << ", i);\n";
            v = "q";
          }
          else
          {
            indent(ctx, k + 4) << "struct value *p = nth_value(" << v << ", i);\n";
            v = "p";
          }
          path_gen_c(ctx, jpath, v, k + 4, throwup);
          indent(ctx, k + 2) << "}\n";
          indent(ctx, k) << "}\n";
        }
        else if (step < 0)
        {
          if (explain)
            indent(ctx, k) << "/* if current node " << v << " is an array then iterate from " << start << " to " << end << " by " << step << " to match " << jpath << " */\n";
          indent(ctx, k) << "if (is_array(" << v << "))\n";
          indent(ctx, k) << "{\n";
          if (start < 0 && end >= 0)
          {
            indent(ctx, k + 2) << "int i, k = has_size(" << v << ")-" << -start << ";\n";
            indent(ctx, k + 2) << "for (i = k; i >= " << end << "; i -= " << -step << ")\n";
          }
          else if (start >= 0 && end >= 0)
          {
            indent(ctx, k + 2) << "int i, k = (" << start << " < has_size(" << v << ") ? " << start << " : has_size(" << v << ")-1);\n";
            indent(ctx, k + 2) << "for (i = k; i >= " << end << "; i -= " << -step << ")\n";
          }
          else // start < 0 && end < 0
          {
            indent(ctx, k + 2) << "int i, k = (" << end << " >= -has_size(" << v << ") ? " << end << " : -has_size(" << v << "));\n";
            indent(ctx, k + 2) << "for (i = " << start << "; i >= k; i -= " << -step << ")\n";
          }
          indent(ctx, k + 2) << "{\n";
          if (v[0] == 'p')
          {
            indent(ctx, k + 4) << "struct value *q = nth_value(" << v << ", i);\n";
            v = "q";
          }
          else
          {
            indent(ctx, k + 4) << "struct value *p = nth_value(" << v << ", i);\n";
            v = "p";
          }
          path_gen_c(ctx, jpath, v, k + 4, throwup);
          indent(ctx, k + 2) << "}\n";
          indent(ctx, k) << "}\n";
        }
        else
        {
          if (explain)
            indent(ctx, k) << "/* if current node " << v << " is an array with element [" << start << "] then match " << jpath << " */\n";
          indent(ctx, k) << "if ((j = nth_nth(" << v << ", " << start << ")) >= 0)\n";
          indent(ctx, k) << "{\n";
          if (v[0] == 'p')
          {
            indent(ctx, k + 2) << "struct value *q = nth_value(" << v << ", j);\n";
            v = "q";
          }
          else
          {
            indent(ctx, k + 2) << "struct value *p = nth_value(" << v << ", j);\n";
            v = "p";
          }
          path_gen_c(ctx, jpath, v, k + 2, throwup);
          indent(ctx, k) << "}\n";
        }
      }
      else
      {
        indent(ctx, k + 4) << "default:\n";
        indent(ctx, k + 6) << "continue;\n";
        indent(ctx, k + 2) << "}\n";
        if (explain)
          indent(ctx, k + 2) << "/* ... then match " << jpath << " */\n";
        indent(ctx, k + 2) << "{\n";
        path_gen_c(ctx, jpath, u, k + 4, throwup);
        indent(ctx, k + 2) << "}\n";
        indent(ctx, k) << "} while (k <= " << 2*subs << ");\n";
      }
    }
  }
  else if (*jpath == '?') // path: P?Q means all values P such that Q (has at least one result)
  {
    if (throwup)
    {
      fprintf(stderr, "jsoncpp: JSONPath cannot nest '?' and '!' ...->%s\n\n", jpath);
      exit(EXIT_FAILURE);
    }
    ++jpath;
    if (explain)
      indent(ctx, k) << "/* value of current node " << v << " if its descendants match " << jpath << " */\n";
    std::string u = v;
    path_gen_c(ctx, jpath, v, k, true);
    indent(ctx, k) << "if (0)\n";
    indent(ctx, k) << "{\n";
    indent(ctx, 2) << "found:\n";
    path_exec_c(ctx, u, k + 2);
    indent(ctx, k) << "}\n";
  }
  else if (*jpath == '!') // path: P!Q means all values P such that not Q (has no results)
  {
    if (throwup)
    {
      fprintf(stderr, "jsoncpp: JSONPath cannot nest '?' and '!' ...->%s\n\n", jpath);
      exit(EXIT_FAILURE);
    }
    ++jpath;
    if (explain)
      indent(ctx, k) << "/* value of current node " << v << " if its descendants do not match " << jpath << " */\n";
    std::string u = v;
    path_gen_c(ctx, jpath, v, k, true);
    path_exec_c(ctx, u, k);
    indent(ctx, 2) << "found:\n";
    indent(ctx, k) << ";\n";
  }
  else if (*jpath == '*') // path: *...
  {
    ++jpath;
    if (explain)
      indent(ctx, k) << "/* iterate over current array/struct node " << v << " to match " << jpath << " */\n";
    indent(ctx, k) << "int i, k = has_size(" << v << ");\n";
    indent(ctx, k) << "for (i = 0; i < k; ++i)\n";
    indent(ctx, k) << "{\n";
    if (v[0] == 'p')
    {
      indent(ctx, k + 2) << "struct value *q = nth_value(" << v << ", i);\n";
      v = "q";
    }
    else
    {
      indent(ctx, k + 2) << "struct value *p = nth_value(" << v << ", i);\n";
      v = "p";
    }
    path_gen_c(ctx, jpath, v, k + 2, throwup);
    indent(ctx, k) << "}\n";
  }
  else if (!*jpath)
  {
    if (throwup)
      indent(ctx, k) << "goto found;\n";
    else
      path_exec_c(ctx, v, k);
  }
  else // path: name...
  {
    std::string name = getname(&jpath);
    if (explain)
      indent(ctx, k) << "/* if current node " << v << " is a struct with field '" << name << "' then match " << jpath << " */\n";
    indent(ctx, k) << "if ((j = nth_at(" << v << ", " << putstr(name) << ")) >= 0)\n";
    indent(ctx, k) << "{\n";
    if (v[0] == 'p')
    {
      indent(ctx, k + 2) << "struct value *q = nth_value(" << v << ", j);\n";
      v = "q";
    }
    else
    {
      indent(ctx, k + 2) << "struct value *p = nth_value(" << v << ", j);\n";
      v = "p";
    }
    path_gen_c(ctx, jpath, v, k + 2, throwup);
    indent(ctx, k) << "}\n";
  }
}

static void path_exec_c(struct soap *ctx, std::string& v, int k)
{
  if (jcode)
  {
    indent(ctx, k) << "value *v = " << v << ";\n";
    indent(ctx, k) << jcode << "\n";
  }
  else
  {
    indent(ctx, k) << "QUERY_YIELD(" << v << ");\n";
  }
}

////////////////////////////////////////////////////////////////////////////////
//
//      C++ gen JSONPath from -ppath
//
////////////////////////////////////////////////////////////////////////////////

static void path_gen_cpp(struct soap *ctx, const char *jpath, std::string& v, int k, bool throwup)
{
  if (*jpath == '$' || *jpath == '@') // strip $ and @ from path
    ++jpath;
  while (isspace(*jpath) || (*jpath == '.' && *(jpath + 1) != '.')) // skip space and single .
    ++jpath;
  if (*jpath == '.') // path: ..
  {
    ++jpath;
    if (*jpath == '.')
    {
      ++jpath;
      if (explain)
        indent(ctx, k) << "// iterate over descendants of current node " << v << " to match " << jpath << "\n";
      indent(ctx, k) << "std::stack<value*> S;\n";
      indent(ctx, k) << "S.push(&" << v << ");\n";
      indent(ctx, k) << "while (!S.empty())\n";
      indent(ctx, k) << "{\n";
      indent(ctx, k + 2) << "value& r = *S.top();\n";
      indent(ctx, k + 2) << "S.pop();\n";
      indent(ctx, k + 2) << "for (int i = r.size()-1; i >= 0; --i)\n";
      indent(ctx, k + 4) << "S.push(&r[i]);\n";
      v = "r";
      path_gen_cpp(ctx, jpath, v, k + 2, throwup);
      indent(ctx, k) << "}\n";
    }
    else
    {
      fprintf(stderr, "jsoncpp: JSONPath unexpected end at '.'\n\n");
      exit(EXIT_FAILURE);
    }
  }
  else if (*jpath == '[') // path: [*]..., [?x], or [(),:]...
  {
    ++jpath;
    if (*jpath == '*') // path: [*]...
    {
      ++jpath;
      if (*jpath != ']')
      {
        fprintf(stderr, "jsoncpp: JSONPath ']' expected at ...->%s\n\n", jpath);
        exit(EXIT_FAILURE);
      }
      ++jpath;
      if (explain)
        indent(ctx, k) << "// iterate over current array/struct node " << v << " to match " << jpath << "\n";
      indent(ctx, k) << "value::iterator k = " << v << ".end();\n";
      indent(ctx, k) << "for (value::iterator i = " << v << ".begin(); i != k; ++i)\n";
      indent(ctx, k) << "{\n";
      indent(ctx, k + 2) << "value& r = *i;\n";
      v = "r";
      path_gen_cpp(ctx, jpath, v, k + 2, throwup);
      indent(ctx, k) << "}\n";
    }
    else if (*jpath == '?') // path: [?x]... where x is a C++ bool expression
    {
      ++jpath;
      if (*jpath != '(')
      {
        fprintf(stderr, "jsoncpp: JSONPath '(' expected at ...->%s\n\n", jpath);
        exit(EXIT_FAILURE);
      }
      ++jpath;
      int nest = 0;
      const char *s;
      for (s = jpath; *s; ++s)
      {
        if (*s == ')')
        {
          if (nest == 0)
            break;
          --nest;
        }
        else if (*s == '(')
          ++nest;
      }
      if (*s != ')' || *(s + 1) != ']')
      {
        fprintf(stderr, "jsoncpp: JSONPath ')]' expected at ...->%s\n\n", jpath);
        exit(EXIT_FAILURE);
      }
      std::string code(jpath, s - jpath);
      jpath = s + 2;
      if (explain)
        indent(ctx, k) << "// filter current node " << v << " with 'if (" << code << ")' to match " << jpath << "\n";
      indent(ctx, k) << "value& v = " << v << ";\n";
      indent(ctx, k) << "if (" << code << ")\n";
      indent(ctx, k) << "{\n";
      path_gen_cpp(ctx, jpath, v, k + 2, throwup);
      indent(ctx, k) << "}\n";
    }
    else // path: [(),:]...
    {
      std::string p, u;
      int subs = 0;
      std::string code;
      std::string name;
      int start, end, step;
      while (true)
      {
        ++subs;
        code.clear();
        name.clear();
        start = end = step = 0;
        if (*jpath == '(')
        {
          ++jpath;
          int nest = 0;
          const char *s;
          for (s = jpath; *s; ++s)
          {
            if (*s == ')')
            {
              if (nest == 0)
                break;
              --nest;
            }
            else if (*s == '(')
              ++nest;
          }
          if (*s != ')')
          {
            fprintf(stderr, "jsoncpp: JSONPath ')' expected at ...->%s\n\n", jpath);
            exit(EXIT_FAILURE);
          }
          code = std::string(jpath, s - jpath);
          jpath = s + 1;
        }
        else if (*jpath == ':')
        {
          ++jpath;
          bool hasend = true;
          if (*jpath == '-' || *jpath == '+' || isdigit(*jpath))
            end = soap_strtol(jpath, (char**)&jpath, 10);
          else
            hasend = false;
          if (*jpath == ':')
            step = soap_strtol(jpath + 1, (char**)&jpath, 10);
          else
            step = 1;
          if (step > 0) // end is exclusive (Python array slicing)
            --end;
          else if (hasend)
            ++end;
          if (step < 0)
            start = -1;
        }
        else if (*jpath == '\'' || *jpath == '"' || isalpha(*jpath))
        {
          name = getname(&jpath);
        }
        else if (*jpath == '-' || *jpath == '+' || isdigit(*jpath))
        {
          start = soap_strtol(jpath, (char**)&jpath, 10);
          if (*jpath == ':')
          {
            ++jpath;
            bool hasend = true;
            if (*jpath == '-' || *jpath == '+' || isdigit(*jpath))
              end = soap_strtol(jpath, (char**)&jpath, 10);
            else
              hasend = false;
            if (*jpath == ':')
              step = soap_strtol(jpath + 1, (char**)&jpath, 10);
            else
              step = 1;
            if (step > 0) // end is exclusive (Python array slicing)
              --end;
            else if (hasend)
              ++end;
          }
        }
        if ((step > 0 && !(start <= end || (start >= 0 && end < 0))) ||
            (step < 0 && !(start >= end || (start < 0 && end >= 0))))
          {
            fprintf(stderr, "jsoncpp: JSONPath inverted bounds at ...->%s\n\n", jpath);
            exit(EXIT_FAILURE);
          }
        if (subs == 1 && *jpath != ',') // path: [x]... with x a name or index or [start]:[end][:step] slice
          break;
        if (subs == 1) // path: [x,...]... with x a name or index or [start]:[end][:step] slice
        {
          if (explain)
            indent(ctx, k) << "// for each case\n";
          indent(ctx, k) << "int i, k = 1;\n";
          if (v.compare(0, 4, "(*p)") == 0)
          {
            indent(ctx, k) << "value *q;\n";
            p = "q";
            u = "(*q)";
          }
          else
          {
            indent(ctx, k) << "value *p;\n";
            p = "p";
            u = "(*p)";
          }
          indent(ctx, k) << "do\n";
          indent(ctx, k) << "{\n";
          indent(ctx, k + 2) << "switch (k)\n";
          indent(ctx, k + 2) << "{\n";
        }
        indent(ctx, k + 4) << "case " << 2*subs-1 << ":\n";
        if (!code.empty())
        {
          indent(ctx, k + 4) << "{\n";
          if (explain)
            indent(ctx, k + 6) << "// if current node " << v << " is an array/struct with field/element [(" << code << ")] ...\n";
          indent(ctx, k + 6) << "k = " << 2*subs+1 << ";\n";
          indent(ctx, k + 6) << "value& v = " << v << ";\n";
          indent(ctx, k + 6) << "if ((i = v.nth(" << code << ")) >= 0)\n";
          indent(ctx, k + 6) << "{\n";
          indent(ctx, k + 8) << p << " = &v[i];\n";
          indent(ctx, k + 8) << "break;\n";
          indent(ctx, k + 6) << "}\n";
          indent(ctx, k + 4) << "}\n";
        }
        else if (!name.empty())
        {
          if (explain)
            indent(ctx, k + 6) << "// if current node " << v << " is a struct with field '" << name << "' ...\n";
          indent(ctx, k + 6) << "k = " << 2*subs+1 << ";\n";
          if (optimal)
          {
            indent(ctx, k + 6) << "if ((i = " << v << ".nth(" << putstr(name) << ")) >= 0)\n";
            indent(ctx, k + 6) << "{\n";
            indent(ctx, k + 8) << p << " = &" << v << "[i];\n";
            indent(ctx, k + 8) << "break;\n";
            indent(ctx, k + 6) << "}\n";
          }
          else
          {
            indent(ctx, k + 6) << "if (" << v << ".has(" << putstr(name) << "))\n";
            indent(ctx, k + 6) << "{\n";
            indent(ctx, k + 8) << p << " = &" << v << "[" << putstr(name) << "];\n";
            indent(ctx, k + 8) << "break;\n";
            indent(ctx, k + 6) << "}\n";
          }
        }
        else if (step > 0)
        {
          if (explain)
            indent(ctx, k + 6) << "// if current node " << v << " is an array then iterate from " << start << " to " << end << " by " << step << " ...\n";
          indent(ctx, k + 6) << "if (!" << v << ".is_array())\n";
          indent(ctx, k + 6) << "{\n";
          indent(ctx, k + 8) << "k = " << 2*subs+1 << ";\n";
          indent(ctx, k + 8) << "continue;\n";
          indent(ctx, k + 6) << "}\n";
          if (start < 0)
            indent(ctx, k + 6) << "i = (" << start << " >= -" << v << ".size() ? " << start << " : -" << v << ".size())-" << step << ";\n";
          else
            indent(ctx, k + 6) << "i = " << start-step << ";\n";
          indent(ctx, k + 6) << "k = " << 2*subs << ";\n";
          indent(ctx, k + 4) << "case " << 2*subs << ":\n";
          indent(ctx, k + 6) << "i += " << step << ";\n";
          if (start >= 0 && end < 0) // : 0 <= start <= size+end where end < 0
            indent(ctx, k + 6) << "if (i <= " << v << ".size()+" << end << ")\n";
          else if (start >= 0 && end >= 0) // : 0 <= start <= end < size
            indent(ctx, k + 6) << "if (i <= " << end << " && i < " << v << ".size())\n";
          else // start < 0 && end < 0 : -size <= start <= end < 0
            indent(ctx, k + 6) << "if (i <= " << end << ")\n";
          indent(ctx, k + 6) << "{\n";
          indent(ctx, k + 8) << p << " = &" << v << "[i];\n";
          indent(ctx, k + 8) << "break;\n";
          indent(ctx, k + 6) << "}\n";
          indent(ctx, k + 6) << "else\n";
          indent(ctx, k + 8) << "k = " << 2*subs+1 << ";\n";
        }
        else if (step < 0)
        {
          if (explain)
            indent(ctx, k + 6) << "// if current node " << v << " is an array then iterate from " << start << " to " << end << " by " << step << " ...\n";
          indent(ctx, k + 6) << "if (!" << v << ".is_array())\n";
          indent(ctx, k + 6) << "{\n";
          indent(ctx, k + 8) << "k = " << 2*subs+1 << ";\n";
          indent(ctx, k + 8) << "continue;\n";
          indent(ctx, k + 6) << "}\n";
          if (start < 0 && end >= 0)
            indent(ctx, k + 6) << "i = " << v << ".size()+" << start-step << ";\n";
          else if (start >= 0 && end >= 0)
            indent(ctx, k + 6) << "i = (" << start << " < " << v << ".size() ? " << start << " : " << v << ".size()-1)+" << -step << ";\n";
          else // start < 0 && end < 0
            indent(ctx, k + 6) << "i = " << start-step << ";\n";
          indent(ctx, k + 6) << "k = " << 2*subs << ";\n";
          indent(ctx, k + 4) << "case " << 2*subs << ":\n";
          indent(ctx, k + 6) << "i -= " << -step << ";\n";
          if (start < 0 && end >= 0) // : 0 <= end <= size+start where start < 0
            indent(ctx, k + 6) << "if (i >= " << end << ")\n";
          else if (start >= 0 && end >= 0) // : 0 <= end <= start < size
            indent(ctx, k + 6) << "if (i >= " << end << ")\n";
          else // start < 0 && end < 0 : -size <= end <= start < 0
            indent(ctx, k + 6) << "if (i >= " << end << " && i >= -" << v << ".size())\n";
          indent(ctx, k + 6) << "{\n";
          indent(ctx, k + 8) << p << " = &" << v << "[i];\n";
          indent(ctx, k + 8) << "break;\n";
          indent(ctx, k + 6) << "}\n";
          indent(ctx, k + 6) << "else\n";
          indent(ctx, k + 8) << "k = " << 2*subs+1 << ";\n";
        }
        else
        {
          if (explain)
            indent(ctx, k + 6) << "// if current node " << v << " is an array with element [" << start << "] ...\n";
          indent(ctx, k + 6) << "k = " << 2*subs+1 << ";\n";
          indent(ctx, k + 6) << "if (" << v << ".has(" << start << "))\n";
          indent(ctx, k + 6) << "{\n";
          indent(ctx, k + 8) << p << " = &" << v << "[" << start << "];\n";
          indent(ctx, k + 8) << "break;\n";
          indent(ctx, k + 6) << "}\n";
        }
        if (*jpath != ',')
          break;
        ++jpath;
      }
      if (subs == 0 && *jpath == ']')
      {
        fprintf(stderr, "jsoncpp: JSONPath empty '[]' at ...->%s\n\n", jpath);
        exit(EXIT_FAILURE);
      }
      if (*jpath != ']')
      {
        fprintf(stderr, "jsoncpp: JSONPath ']' expected at ...->%s\n\n", jpath);
        exit(EXIT_FAILURE);
      }
      ++jpath;
      if (subs == 1) // path: [x]... with x a name or index or [start]:[end][:step] range
      {
        if (!code.empty())
        {
          if (explain)
            indent(ctx, k) << "// if current node " << v << " is an array/struct with field/element [(" << code << ")] then match " << jpath << "\n";
          indent(ctx, k) << "value& v = " << v << ";\n";
          indent(ctx, k) << "if ((j = v.nth(" << code << ")) >= 0)\n";
          indent(ctx, k) << "{\n";
          indent(ctx, k + 2) << "value& r = v[j];\n";
          v = "r";
          path_gen_cpp(ctx, jpath, v, k + 2, throwup);
          indent(ctx, k) << "}\n";
        }
        else if (!name.empty())
        {
          if (optimal)
          {
            if (explain)
              indent(ctx, k) << "// if current node " << v << " is a struct with field '" << name << "' then match " << jpath << "\n";
            indent(ctx, k) << "if ((j = " << v << ".nth(" << putstr(name) << ")) >= 0)\n";
            indent(ctx, k) << "{\n";
            if (v[0] == 'r')
            {
              indent(ctx, k + 2) << "value& s = " << v << "[j];\n";
              v = "s";
            }
            else
            {
              indent(ctx, k + 2) << "value& r = " << v << "[j];\n";
              v = "r";
            }
            path_gen_cpp(ctx, jpath, v, k + 2, throwup);
            indent(ctx, k) << "}\n";
          }
          else
          {
            indent(ctx, k) << "if (" << v << ".has(" << putstr(name) << "))\n";
            indent(ctx, k) << "{\n";
            v.append("[").append(putstrname(name.c_str())).append("]");
            path_gen_cpp(ctx, jpath, v, k + 2, throwup);
            indent(ctx, k) << "}\n";
          }
        }
        else if (step > 0)
        {
          if (explain)
            indent(ctx, k) << "// if current node " << v << " is an array then iterate from " << start << " to " << end << " by " << step << " to match " << jpath << "\n";
          indent(ctx, k) << "if (" << v << ".is_array())\n";
          indent(ctx, k) << "{\n";
          if (start >= 0 && end < 0)
          {
            indent(ctx, k + 2) << "int k = " << v << ".size()-" << -end << ";\n";
            indent(ctx, k + 2) << "for (int i = " << start << "; i <= k; i += " << step << ")\n";
          }
          else if (start >= 0 && end >= 0)
          {
            indent(ctx, k + 2) << "int k = (" << end << " < " << v << ".size() ? " << end << " : " << v << ".size()-1);\n";
            indent(ctx, k + 2) << "for (int i = " << start << "; i <= k; i += " << step << ")\n";
          }
          else // start < 0 && end < 0
          {
            indent(ctx, k + 2) << "int k = (" << start << " >= -" << v << ".size() ? " << start << " : -" << v << ".size());\n";
            indent(ctx, k + 2) << "for (int i = k; i <= " << end << "; i += " << step << ")\n";
          }
          indent(ctx, k + 2) << "{\n";
          if (v[0] == 'r')
          {
            indent(ctx, k + 4) << "value& s = " << v << "[i];\n";
            v = "s";
          }
          else
          {
            indent(ctx, k + 4) << "value& r = " << v << "[i];\n";
            v = "r";
          }
          path_gen_cpp(ctx, jpath, v, k + 4, throwup);
          indent(ctx, k + 2) << "}\n";
          indent(ctx, k) << "}\n";
        }
        else if (step < 0)
        {
          if (explain)
            indent(ctx, k) << "// if current node " << v << " is an array then iterate from " << start << " to " << end << " by " << step << " to match " << jpath << "\n";
          indent(ctx, k) << "if (" << v << ".is_array())\n";
          indent(ctx, k) << "{\n";
          if (start < 0 && end >= 0)
          {
            indent(ctx, k + 2) << "int k = " << v << ".size()-" << -start << ";\n";
            indent(ctx, k + 2) << "for (int i = k; i >= " << end << "; i -= " << -step << ")\n";
          }
          else if (start >= 0 && end >= 0)
          {
            indent(ctx, k + 2) << "int k = (" << start << " < " << v << ".size() ? " << start << " : " << v << ".size()-1);\n";
            indent(ctx, k + 2) << "for (int i = k; i >= " << end << "; i -= " << -step << ")\n";
          }
          else // start < 0 && end < 0
          {
            indent(ctx, k + 2) << "int k = (" << end << " >= -" << v << ".size() ? " << end << " : -" << v << ".size());\n";
            indent(ctx, k + 2) << "for (int i = " << start << "; i >= k; i -= " << -step << ")\n";
          }
          indent(ctx, k + 2) << "{\n";
          if (v[0] == 'r')
          {
            indent(ctx, k + 4) << "value& s = " << v << "[i];\n";
            v = "s";
          }
          else
          {
            indent(ctx, k + 4) << "value& r = " << v << "[i];\n";
            v = "r";
          }
          path_gen_cpp(ctx, jpath, v, k + 4, throwup);
          indent(ctx, k + 2) << "}\n";
          indent(ctx, k) << "}\n";
        }
        else
        {
          if (explain)
            indent(ctx, k) << "// if current node " << v << " is an array with element [" << start << "] then match " << jpath << "\n";
          indent(ctx, k) << "if (" << v << ".has(" << start << "))\n";
          indent(ctx, k) << "{\n";
          v.append("[").append(soap_int2s(ctx, start)).append("]");
          path_gen_cpp(ctx, jpath, v, k + 2, throwup);
          indent(ctx, k) << "}\n";
        }
      }
      else
      {
        indent(ctx, k + 4) << "default:\n";
        indent(ctx, k + 6) << "continue;\n";
        indent(ctx, k + 2) << "}\n";
        if (explain)
          indent(ctx, k + 2) << "// ... then match " << jpath << "\n";
        path_gen_cpp(ctx, jpath, u, k + 2, throwup);
        indent(ctx, k) << "} while (k <= " << 2*subs << ");\n";
      }
    }
  }
  else if (*jpath == '?') // path: P?Q means all values P such that Q (has at least one result)
  {
    if (throwup)
    {
      fprintf(stderr, "jsoncpp: JSONPath cannot nest '?' and '!' ...->%s\n\n", jpath);
      exit(EXIT_FAILURE);
    }
    ++jpath;
    if (explain)
      indent(ctx, k) << "// value of current node " << v << " if its descendants match " << jpath << "\n";
    indent(ctx, k) << "try\n";
    indent(ctx, k) << "{\n";
    std::string u = v;
    path_gen_cpp(ctx, jpath, v, k + 2, true);
    indent(ctx, k) << "}\n";
    indent(ctx, k) << "catch (const bool&)\n";
    indent(ctx, k) << "{\n";
    path_exec_cpp(ctx, u, k + 2);
    indent(ctx, k) << "}\n";
  }
  else if (*jpath == '!') // path: P!Q means all values P such that not Q (has no results)
  {
    if (throwup)
    {
      fprintf(stderr, "jsoncpp: JSONPath cannot nest '?' and '!' ...->%s\n\n", jpath);
      exit(EXIT_FAILURE);
    }
    ++jpath;
    if (explain)
      indent(ctx, k) << "// value of current node " << v << " if its descendants do not match " << jpath << "\n";
    indent(ctx, k) << "try\n";
    indent(ctx, k) << "{\n";
    std::string u = v;
    path_gen_cpp(ctx, jpath, v, k + 2, true);
    indent(ctx, k + 2) << "throw false;\n";
    indent(ctx, k) << "}\n";
    indent(ctx, k) << "catch (const bool& k)\n";
    indent(ctx, k) << "{\n";
    indent(ctx, k + 2) << "if (k == false)\n";
    path_exec_cpp(ctx, u, k + 4);
    indent(ctx, k) << "}\n";
  }
  else if (*jpath == '*') // path: *...
  {
    ++jpath;
    if (explain)
      indent(ctx, k) << "// iterate over current array/struct node " << v << " to match " << jpath << "\n";
    indent(ctx, k) << "value::iterator k = " << v << ".end();\n";
    indent(ctx, k) << "for (value::iterator i = " << v << ".begin(); i != k; ++i)\n";
    indent(ctx, k) << "{\n";
    indent(ctx, k + 2) << "value& r = " << v << "*i;\n";
    v = "r";
    path_gen_cpp(ctx, jpath, v, k + 2, throwup);
    indent(ctx, k) << "}\n";
  }
  else if (!*jpath)
  {
    if (throwup)
      indent(ctx, k) << "throw true;\n";
    else
      path_exec_cpp(ctx, v, k);
  }
  else // path: name...
  {
    std::string name = getname(&jpath);
    if (explain)
      indent(ctx, k) << "// if current node " << v << " is a struct with field '" << name << "' then match " << jpath << "\n";
    if (optimal)
    {
      indent(ctx, k) << "if ((j = " << v << ".nth(" << putstr(name) << ")) >= 0)\n";
      indent(ctx, k) << "{\n";
      if (v[0] == 'r')
      {
        indent(ctx, k + 2) << "value& s = " << v << "[j];\n";
        v = "s";
      }
      else
      {
        indent(ctx, k + 2) << "value& r = " << v << "[j];\n";
        v = "r";
      }
      path_gen_cpp(ctx, jpath, v, k + 2, throwup);
      indent(ctx, k) << "}\n";
    }
    else
    {
      indent(ctx, k) << "if (" << v << ".has(" << putstr(name) << "))\n";
      indent(ctx, k) << "{\n";
      v.append("[").append(putstrname(name.c_str())).append("]");
      path_gen_cpp(ctx, jpath, v, k + 2, throwup);
      indent(ctx, k) << "}\n";
    }
  }
}

static void path_exec_cpp(struct soap *ctx, std::string& v, int k)
{
  if (jcode)
  {
    indent(ctx, k) << "value& v = " << v << ";\n";
    indent(ctx, k) << jcode << "\n";
  }
  else
  {
    indent(ctx, k) << "QUERY_YIELD(" << v << ");\n";
  }
}

////////////////////////////////////////////////////////////////////////////////
//
//      Misc
//
////////////////////////////////////////////////////////////////////////////////

static std::ostream& indent(soap *ctx, int k)
{
  *ctx->os << std::setw(k) << "" << std::setw(0);
  return *ctx->os;
}

static std::string gen_ident(const std::string& lhs, const char *name)
{
  std::string id = lhs;
  id.append("_");
  for (const char *s = name; *s; name = s)
  {
    while (isalnum(*s))
      s++;
    id.append(name, s - name);
    if (*s)
      s++;
  }
  return id;
}

static std::string gen_counter(int k)
{
  std::string id = "i";
  char tmp[2];
  if (k <= 9)
    k += '0';
  else if (k <= 36)
    k += 'a' - 10;
  else
    k += 'A' - 36;
  tmp[0] = k;
  tmp[1] = '\0';
  id.append(tmp);
  return id;
}

static std::string putstrname(const char *name)
{
  std::stringstream ss;
  ss << putstr(name);
  return ss.str();
  
}

static std::string putname(const char *name)
{
  const char *s;
  for (s = name; isalnum(*s) || *s == '_'; s++)
    continue;
  if (!*s)
    return name;
  return putstrname(name);
}

static std::string getname(const char **jpath)
{
  const char *s = *jpath;
  const char *t = s;
  if (*t == '\'' || *t == '"')
  {
    while (*++t && *t != *s)
      if (*t == '\\' && *(t+1))
        ++t;
    if (!*t)
    {
      fprintf(stderr, "jsoncpp: JSONPath closing quote expected at ...->%s\n\n", s);
      exit(EXIT_FAILURE);
    }
    *jpath = ++t;
    return std::string(s + 1, t - s - 2);
  }
  while (*t && !ispunct(*t) && !isspace(*t))
    ++t;
  if (t == s)
  {
    fprintf(stderr, "jsoncpp: JSONPath empty field name at ...->%s\n\n", s);
    exit(EXIT_FAILURE);
  }
  *jpath = t;
  return std::string(s, t - s);
}
