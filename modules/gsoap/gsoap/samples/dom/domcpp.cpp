/*
        domcpp.cpp
        
        An XML DOM C/C++ code generator for the gSOAP DOM API.

        1. Take an XML sample document and render it in DOM API code to
           construct this XML as a DOM object:

        echo '\
        <menu id="file" value="File">\
          <popup>\
            <menuitem value="New" onclick="CreateNewDoc()" />\
            <menuitem value="Open" onclick="OpenDoc()" />\
            <menuitem value="Close" onclick="CloseDoc()" />\
          </popup>\
        </menu>' | ./domcpp

        #include "soapH.h"
        {
          struct soap *ctx = soap_new1(SOAP_C_UTFSTRING | SOAP_XML_INDENT | SOAP_DOM_TREE);
          ctx->double_format = "%lG";

          xsd__anyType dom(ctx, "menu");
          dom.att("id") = "file";
          dom.att("value") = "File";
          dom["popup"]["menuitem"][1].att("value") = "New";
          dom["popup"]["menuitem"][1].att("onclick") = "CreateNewDoc()";
          dom["popup"]["menuitem"][2].att("value") = "Open";
          dom["popup"]["menuitem"][2].att("onclick") = "OpenDoc()";
          dom["popup"]["menuitem"][3].att("value") = "Close";
          dom["popup"]["menuitem"][3].att("onclick") = "CloseDoc()";
          std::cout << dom << std::endl;

          soap_destroy(ctx); // delete objects
          soap_end(ctx);     // delete DOM data
          soap_free(ctx);    // free context
        }

        2. Take a XML sample document and render it in DOM API code to inspect
           and extract values from XML input, by using the XML sample document
           as a template:

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
        }' | ./domcpp -i

        #include "soapH.h"
        {
          struct soap *ctx = soap_new1(SOAP_C_UTFSTRING | SOAP_XML_INDENT | SOAP_DOM_TREE);
          ctx->double_format = "%lG";

          xsd__anyType dom(ctx);
          std::cin >> dom;
          if (dom.soap->error)
            exit(EXIT_FAILURE); // error parsing XML
          xsd__anyAttribute *att;
          xsd__anyType *elt;
          #define USE_ATT(path, text) std::cout << path << " = " << text << std::endl
          #define USE_ELT(path, text) std::cout << path << " = " << text << std::endl
          if ((att = dom.att_get("id")))
            USE_ATT("/menu/@id", att->get_text());
          if ((att = dom.att_get("value")))
            USE_ATT("/menu/@value", att->get_text());
          if ((elt = dom.elt_get("popup")))
          {
            xsd__anyType& dom_popup = *elt;
            for (xsd__anyType *it = dom_popup.elt_get("menuitem"); it; it = it->get_next())
            {
              xsd__anyType& dom_popup_menuitem = *it;
              if ((att = dom_popup_menuitem.att_get("value")))
                USE_ATT("/menu/popup/menuitem/@value", att->get_text());
              if ((att = dom_popup_menuitem.att_get("onclick")))
                USE_ATT("/menu/popup/menuitem/@onclick", att->get_text());
            }
          }
          std::cout << dom << std::endl;

          soap_destroy(ctx); // delete objects
          soap_end(ctx);     // delete DOM data
          soap_free(ctx);    // free context
        }

        3. Take an XPath to generate a stand-alone application that filters
           XML documents with a store of books to return their titles:

        ./domcpp -m -p'/store/book/@title'

        #include "soapH.h"
        struct Namespace namespaces[] = {{NULL,NULL,NULL,NULL}};
        int main(int argc, char **argv)
        {
          struct soap *ctx = soap_new1(SOAP_C_UTFSTRING | SOAP_XML_INDENT | SOAP_DOM_TREE);
          ctx->double_format = "%lG";

          xsd__anyType dom(ctx);
          std::cin >> dom;
          if (dom.soap->error)
            exit(EXIT_FAILURE); // error parsing XML
          // XPath: /store/book/@title
          xsd__anyAttribute *att;
          xsd__anyType *elt;
          #define QUERY_YIELD_ATT(v) std::cout << v.get_text() << std::endl
          #define QUERY_YIELD_ELT(v) std::cout << v << std::endl
          if (dom.match("store"))
          {
            size_t pos = 1;
            for (xsd__anyType *it = dom.elt_get("book"); it; it = it->get_next(), ++pos)
            {
              xsd__anyType& v = *it;
              if ((att = v.att_get("title")))
              {
                xsd__anyAttribute& v = *att;
                QUERY_YIELD_ATT(v);
              }
            }
          }

          soap_destroy(ctx); // delete objects
          soap_end(ctx);     // delete DOM data
          soap_free(ctx);    // free context
          return 0;
        }

        4. To build domcpp:

        cd gsoap/samples/dom
        soapcpp2 -CSL ../../import/dom.h
        c++ -I../.. -o domcpp domcpp.cpp soapC.cpp ../../dom.cpp ../../stdsoap2.cpp

        5. To compile and link domcpp-generated C++ code:

        ./domcpp -o mydom.cpp ...
        soapcpp2 -CSL ../../import/dom.h
        c++ -I../.. -o mydom mydom.cpp soapC.cpp ../../dom.cpp ../../stdsoap2.cpp

        6. To compile and link domcpp-generated C code:

        ./domcpp -c -o mydom.c ...
        soapcpp2 -c -CSL ../../import/dom.h
        cc -I../.. -o mydom mydom.c soapC.c ../../dom.c ../../stdsoap2.c

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

#include "soapH.h"              // generated by soapcpp2 -CSL import/dom.h
#include <fstream>
#include <sstream>
#include <iomanip>
#include <map>

#define DOM_ROOT "root"         // unnamed root element is named "root"

struct Namespace namespaces[] = {{NULL,NULL,NULL,NULL}};

////////////////////////////////////////////////////////////////////////////////
//
//      Command line options
//
////////////////////////////////////////////////////////////////////////////////

static bool coutput = false;            // -c      generate C code instead of C++
static bool explain = false;            // -e      add explanatory comments to the generated code
static bool genread = false;            // -i      generate code to inspect DOM node graph parsed from XML input
static bool genvars = false;            // -l      generate code for option -i to store values in local variables
static bool genmain = false;            // -m      generate stand-alone code by adding main()
static bool minimal = false;            // -M      generate minimal code unadorned with initialization and cleanup
static bool nstable = false;            // -n      generate XML namespace table
static bool optimal = false;            // -O      optimize code by factoring common indices when applicable
static bool collect = false;            // -y      generate code that yields an array 'y' of XPath query results
static const char *dform = NULL;        // -f%fmt  use %fmt to format double floats, e.g. -f%lG
static const char *ofile = NULL;        // -ofile  save source code to file
static const char *xpath = NULL;        // -ppath  generate XPath query code for 'path'
static const char *xroot = "dom";       // -rroot  use 'root' instead of root 'dom' in the generated code
static const char *xcode = NULL;        // -xcode  generate code that executes 'code' for each XPath query result
static const char *ifile = NULL;        // [infile]

////////////////////////////////////////////////////////////////////////////////
//
//      Proto
//
////////////////////////////////////////////////////////////////////////////////

static void out_gen_c(soap*, xsd__anyType&, const char*, const std::map<std::string,const char*>&, const std::string&, const std::string&, int);
static void in_gen_c(soap*, xsd__anyType&, const std::map<std::string,const char*>&, const std::string&, const std::string&, int);
static void out_gen_cpp(soap*, xsd__anyType&, const char*, const std::map<std::string,const char*>&, const std::string&, const std::string&, int);
static void in_gen_cpp(soap*, xsd__anyType&, const std::map<std::string,const char*>&, const std::string&, const std::string&, int);
static void path_gen_c(soap*, const char*, std::string&, bool, bool, int, size_t, size_t);
static void path_exec_c(soap*, std::string&, bool, int);
static void path_gen_cpp(soap*, const char*, std::string&, bool, bool, int, size_t, bool);
static void path_exec_cpp(soap*, std::string&, bool, int);
static std::ostream& indent(soap*, int);
static bool is_bool(const char*);
static bool is_int(soap*, const char*, LONG64&);
static bool is_double(soap*, const char*, double&);
static bool need_ns(const char*, const char*, const std::map<std::string,const char*>&, const char*);
static bool is_qualified(const char*);
static bool is_current(const char*, const char*);
static std::string prefix(const char*);
static const char *name(const char*);
static std::string gen_ident(const std::string&, const char*);
static std::string putstrname(const char*);
static std::string getname(const char**);
static size_t getnth(const char**);

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
  {
    os << DOM_ROOT;
  }
  return os;
}

////////////////////////////////////////////////////////////////////////////////
//
//      domcpp main
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
            case 'l':
              genvars = true;
              break;
            case 'm':
              genmain = true;
              break;
            case 'M':
              minimal = true;
              break;
            case 'n':
              nstable = true;
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
                xpath = a;
              else if (i < argc)
                xpath = argv[++i];
              break;
            case 'r':
              a++;
              f = false;
              if (*a)
                xroot = a;
              else if (i < argc)
                xroot = argv[++i];
              break;
            case 'x':
              a++;
              f = false;
              if (*a)
                xcode = a;
              else if (i < argc)
                xcode = argv[++i];
              break;
            case 'y':
              collect = true;
              break;
            case '?':
            case 'h':
              fprintf(stderr,
                  "Usage: domcpp [-c] [-e] [-f%%fmt] [-h] [-i] [-l] [-m] [-M] [-n] [-O] [-ofile] [-ppath] [-rroot] [-xcode] [-y] [infile]\n\n"
                  "-c      generate C code instead of C++\n"
                  "-e      add explanatory comments to the generated code\n"
                  "-f%%fmt  use %%fmt to format double floats, e.g. -f%%lG\n"
                  "-h      display help message\n"
                  "-i      generate code to inspect DOM node graph parsed from XML input\n"
                  "-l      generate code for option -i to store values in local variables\n"
                  "-m      generate stand-alone code by adding main()\n"
                  "-M      generate minimal code unadorned with initialization and cleanup\n"
                  "-n      generate XML namespace table\n"
                  "-O      optimize code by factoring common indices when applicable\n"
                  "-ofile  save source code to file\n"
                  "-ppath  generate XPath query code for 'path'\n"
                  "-rroot  use 'root' instead of root 'x' in the generated code\n"
                  "-xcode  generate code that executes 'code' for each XPath query result\n"
                  "-y      generate code that yields an array 'y' of XPath query results\n"
                  "infile  optional XML file to parse\n"
                  "-       read XML from standard input\n\n");
              exit(EXIT_SUCCESS);
            default:
              fprintf(stderr, "domcpp: Unknown option %s\n\n", a);
              exit(EXIT_FAILURE);
          }
        }
      }
      else
      {
        if (ifile && strcmp(ifile, "-"))
          fprintf(stderr, "domcpp: Input already specified as %s, ignoring %s\n\n", ifile, a);
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
        fprintf(stderr, "domcpp: Cannot open %s for reading\n", ifile);
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
      fprintf(stderr, "domcpp: Cannot open %s for writing\n", ofile);
      exit(EXIT_FAILURE);
    }
    ctx->os = &ofs;
  }
  else
  {
    ctx->os = &std::cout;
  }

  xsd__anyType dom(ctx);

  if (ifile)
  {
    if (soap_read_xsd__anyType(ctx, &dom))
    {
      soap_print_fault(ctx, stderr);
      soap_print_fault_location(ctx, stderr);
      exit(EXIT_FAILURE);
    }
    if (!dom.tag())
    {
      fprintf(stderr, "domcpp: No root element in %s\n", ifile);
      exit(EXIT_FAILURE);
    }
  }
  else
  {
    nstable = false;
  }

  if (ifile && strcmp(ifile, "-"))
    ifs.close();

  if (!minimal)
  {
    if (coutput)
    {
      *ctx->os <<
        "/* Dependencies:\n"
        "     dom.h stdsoap2.h\n"
        "     dom.c stdsoap2.c\n"
        "   Build:\n"
        "     soapcpp2 -c -CSL dom.h    (generates soapStub.h, soapH.h, soapC.c)\n"
        "     cc ... soapC.c dom.c stdsoap2.c ...\n"
        "*/\n";
    }
    else
    {
      *ctx->os <<
        "// Dependencies:\n"
        "//   dom.h stdsoap2.h\n"
        "//   dom.cpp stdsoap2.cpp\n"
        "// Build:\n"
        "//   soapcpp2 -CSL dom.h    (generates soapStub.h, soapH.h, soapC.cpp)\n"
        "//   c++ ... soapC.cpp dom.cpp stdsoap2.cpp ...\n";
    }
    *ctx->os << "\n#include \"soapH.h\"\n";
    if (!coutput && xpath && strstr(xpath, ".."))
      *ctx->os << "#include <stack>\n";
  }

  std::map<std::string,const char*> nsmap;

  if ((nstable || genmain) && explain)
  {
    *ctx->os <<
      "/*\n"
      "   Global namespace table with xmlns:prefix=\"URI\" bindings of the form:\n"
      "     { \"prefix\", \"URI\", \"alt-URI\", NULL }\n"
      "\n"
      "   When using qualified element and attribute tag names with the DOM API\n"
      "   functions without setting the corresponding namespace URI as a function\n"
      "   argument, you MUST include a global namespace table that defines the\n"
      "   namespace prefixes and URIs that are used.  This permits the DOM API engine\n"
      "   to locate the URI that corresponds to each prefix in a qualified tag name.\n"
      "\n"
      "   Use domcpp option -n with an XML file to collect the xmlns bindings from that\n"
      "   file and generate the namespace table automatically.\n";
    if (nstable)
      *ctx->os <<
        "\n"
        "   The first four entries are required in this order for gSOAP client/server\n"
        "   applications.  When combining this code with a gSOAP client/server\n"
        "   application then use the soapcpp2-generated .nsmap file with namespace table\n"
        "   for that application and add entries below that are missing from the table.\n";
    *ctx->os << "*/\n";
  }

  if (nstable)
  {
    for (xsd__anyType::iterator i = dom.begin(); i != dom.end(); ++i)
    {
      for (xsd__anyAttribute::iterator j = i->att_begin(); j != i->att_end(); ++j)
      {
        if (is_qualified(j->tag()) &&
            strncmp(j->tag(), "SOAP-ENV:", 9) &&
            strncmp(j->tag(), "SOAP-ENC:", 9) &&
            strncmp(j->tag(), "xsi:", 4) &&
            strncmp(j->tag(), "xsd:", 4) &&
            strncmp(j->tag(), "xml", 3))
        {
          std::string id = prefix(j->tag());
          nsmap[id] = j->ns();
        }
      }
      if (is_qualified(i->tag()) &&
          strncmp(i->tag(), "SOAP-ENV:", 9) &&
          strncmp(i->tag(), "SOAP-ENC:", 9) &&
          strncmp(i->tag(), "xsi:", 4) &&
          strncmp(i->tag(), "xsd:", 4) &&
          strncmp(i->tag(), "xml", 3))
      {
        std::string id = prefix(i->tag());
        nsmap[id] = i->ns();
      }
    }

    *ctx->os <<
      "struct Namespace namespaces[] = {\n"
      "  {\"SOAP-ENV\", \"http://schemas.xmlsoap.org/soap/envelope/\", \"http://www.w3.org/*/soap-envelope\",      NULL},\n"
      "  {\"SOAP-ENC\", \"http://schemas.xmlsoap.org/soap/encoding/\", \"http://www.w3.org/*/soap-encoding\",      NULL},\n"
      "  {\"xsi\",      \"http://www.w3.org/2001/XMLSchema-instance\", \"http://www.w3.org/*/XMLSchema-instance\", NULL},\n"
      "  {\"xsd\",      \"http://www.w3.org/2001/XMLSchema\",          \"http://www.w3.org/*/XMLSchema\",          NULL},\n";
    for (std::map<std::string,const char*>::const_iterator i = nsmap.begin(); i != nsmap.end(); ++i)
      *ctx->os << "  {\"" << i->first << "\", " << putstr(i->second) << ", NULL, NULL },\n";
    *ctx->os << "  {NULL, NULL, NULL, NULL}\n};\n\n";

    nsmap["SOAP-ENV"] = "http://schemas.xmlsoap.org/soap/envelope/";
    nsmap["SOAP-ENC"] = "http://schemas.xmlsoap.org/soap/encoding/";
    nsmap["xsi"] = "http://www.w3.org/2001/XMLSchema-instance";
    nsmap["xsd"] = "http://www.w3.org/2001/XMLSchema";
  }

  if (genmain)
  {
    if (!nstable)
      *ctx->os << "struct Namespace namespaces[] = {{NULL,NULL,NULL,NULL}};\n\n";

    *ctx->os << "int main(int argc, char **argv)\n";
  }

  if (!minimal)
  {
    if (coutput)
      *ctx->os << "{ /* Generated by domcpp";
    else
      *ctx->os << "{ // Generated by domcpp";
    for (int i = 1; i < argc; ++i)
      *ctx->os << " " << putstr(argv[i], 0);
    if (coutput)
      *ctx->os <<
        " */\n"
        "  /* domcpp tool Copyright (C) 2000-2016, Robert van Engelen, Genivia Inc. */\n";
    else
      *ctx->os <<
        "\n"
        "  // domcpp tool Copyright (C) 2000-2016, Robert van Engelen, Genivia Inc.\n";
    *ctx->os <<
      "  struct soap *ctx = soap_new1(SOAP_C_UTFSTRING | SOAP_XML_INDENT | SOAP_DOM_TREE);\n"
      "  ctx->double_format = \"" << (dform ? dform : "%lG") << "\";\n\n";
  }

  if (coutput)
  {
    if (!genread && explain && dom.tag())
    {
      *ctx->os << "  /* <" << putstr(dom.tag(), 0);
      for (xsd__anyAttribute::iterator i = dom.att_begin(); i != dom.att_end(); ++i)
        *ctx->os << " " << putstr(i->tag(), 0) << "=" << putstr(*i);
      *ctx->os << "> */\n";
    }

    if (genread || !ifile)
      *ctx->os << "  xsd__anyType *" << xroot << " = soap_elt_new(ctx, NULL, NULL);\n";
    else if (need_ns(dom.ns(), dom.tag(), nsmap, NULL))
      *ctx->os << "  xsd__anyType *" << xroot << " = soap_elt_new(ctx, " << putstr(dom.ns()) << ", " << putstr(dom.tag()) << ");\n";
    else
      *ctx->os << "  xsd__anyType *" << xroot << " = soap_elt_new(ctx, NULL, " << putstr(dom.tag()) << ");\n";

    std::string path = "/";
    if (dom.tag() != NULL)
      path.append(dom.tag());
    else
      path.append(DOM_ROOT);

    if (ifile && !genread)
    {
      out_gen_c(ctx, dom, NULL, nsmap, xroot, path, 2);
      if (explain && dom.tag())
        *ctx->os << "  /* </" << putstr(dom.tag(), 0) << "> */\n";
    }
    else
    {
      if (explain)
        *ctx->os << "  /* parse XML into DOM object " << xroot << " */\n";
      *ctx->os << "  soap_read_xsd__anyType(ctx, " << xroot << ");\n";
      if (!minimal)
        *ctx->os <<
          "  if (" << xroot << "->soap->error)\n"
          "    exit(EXIT_FAILURE); /* error parsing XML */\n";

      if (ifile)
      {
        if (explain)
          *ctx->os <<
            "  /* DOM attribute and element pointers to inspect DOM */\n";
        *ctx->os <<
          "  xsd__anyAttribute *att;\n"
          "  xsd__anyType *elt;\n";

        if (genvars)
          *ctx->os <<
            "  #define USE_ATT(path, var, text) printf(\"%s = %s\\n\", path, text)\n"
            "  #define USE_ELT(path, var, text) printf(\"%s = %s\\n\", path, text)\n";
        else
          *ctx->os <<
            "  #define USE_ATT(path, text) printf(\"%s = %s\\n\", path, text)\n"
            "  #define USE_ELT(path, text) printf(\"%s = %s\\n\", path, text)\n";

        in_gen_c(ctx, dom, nsmap, xroot, path, 2);
      }
    }

    if (xpath)
    {
      *ctx->os <<
        "  /* XPath: " << putstr(xpath, 0) << " */\n";

      if (!ifile || !genread)
      {
        if (explain)
          *ctx->os <<
            "  /* DOM attribute and element pointers to inspect DOM */\n";
        *ctx->os <<
          "  xsd__anyAttribute *att;\n"
          "  xsd__anyType *elt;\n";
      }

      if (!xcode)
      {
        if (collect)
        {
          if (explain)
            *ctx->os << "  /* XPath query yields an array 'y' of XPath query results: */\n";
          *ctx->os <<
            "  xsd__anyType *y = soap_elt_new(ctx, NULL, \"results\");\n"
            "  #define QUERY_YIELD_ATT(v) soap_add_elt(y, soap_add_att(soap_elt_new(ctx, NULL, \"result\"), v));\n"
            "  #define QUERY_YIELD_ELT(v) soap_add_elt(y, soap_add_elt(soap_elt_new(ctx, NULL, \"result\"), v));\n";
        }
        else
        {
          if (explain)
            *ctx->os << "  /* XPath query yields are shown for each XPath result: */\n";
          *ctx->os <<
            "  #define QUERY_YIELD_ATT(v) printf(\"%s\\n\", soap_att_get_text(v))\n"
            "  #define QUERY_YIELD_ELT(v) soap_write_xsd__anyType(ctx, v), putchar('\\n')\n";
        }
      }

      std::string root(xroot);

      if (explain)
      {
        *ctx->os <<
          "  /* Synopsis of variables used:\n"
          "         " << root << "\troot element node pointer\n"
          << ( collect ? "         y\tyield query results\n" : "" ) <<
          "         v\tcurrent element or attribute node pointer, to use in [?expr]\n"
          "         att\tattribute node pointer\n"
          "         elt\telement node pointer\n"
          "         pos\tXPath position()\n"
          "  */\n";
      }

      path_gen_c(ctx, xpath, root, true, false, 2, 0, 0);

      if (!minimal && collect)
        *ctx->os << "  soap_write_xsd__anyType(ctx, y), putchar('\\n');\n";
    }
    else if (!minimal)
    {
      if (explain)
        *ctx->os << "  /* output DOM " << xroot << " in XML */\n";
      *ctx->os << "  soap_write_xsd__anyType(ctx, " << xroot << "), putchar('\\n');\n";
    }

    if (!minimal)
    {
      *ctx->os <<
        "\n"
        "  soap_end(ctx);  /* delete DOM data */\n"
        "  soap_free(ctx); /* free context */\n"
        << (genmain ? "  return 0;\n" : "") <<
        "}\n";
    }
  }
  else
  {
    if (!genread && explain && dom.tag())
    {
      *ctx->os << "  // <" << putstr(dom.tag(), 0);
      for (xsd__anyAttribute::iterator i = dom.att_begin(); i != dom.att_end(); ++i)
        *ctx->os << " " << putstr(i->tag(), 0) << "=" << putstr(*i);
      *ctx->os << "> \n";
    }

    if (genread || !ifile)
      *ctx->os << "  xsd__anyType " << xroot << "(ctx);\n";
    else if (need_ns(dom.ns(), dom.tag(), nsmap, NULL))
      *ctx->os << "  xsd__anyType " << xroot << "(ctx, " << putstr(dom.ns()) << ", " << putstr(dom.tag()) << ");\n";
    else
      *ctx->os << "  xsd__anyType " << xroot << "(ctx, " << putstr(dom.tag()) << ");\n";

    std::string path = "/";
    if (dom.tag() != NULL)
      path.append(dom.tag());
    else
      path.append(DOM_ROOT);

    if (ifile && !genread)
    {
      out_gen_cpp(ctx, dom, NULL, nsmap, xroot, path, 2);
      if (explain && dom.tag())
        *ctx->os << "  // </" << putstr(dom.tag(), 0) << ">\n";
    }
    else
    {
      if (explain)
        *ctx->os << "  // parse XML into DOM object " << xroot << " \n";
      *ctx->os << "  std::cin >> " << xroot << ";\n";
      if (!minimal)
        *ctx->os <<
          "  if (" << xroot << ".soap->error)\n"
          "    exit(EXIT_FAILURE); // error parsing XML\n";

      if (ifile)
      {
        if (explain)
          *ctx->os <<
            "  // DOM attribute and element pointers to inspect DOM\n";
        *ctx->os <<
          "  xsd__anyAttribute *att;\n"
          "  xsd__anyType *elt;\n";

        if (genvars)
          *ctx->os <<
            "  #define USE_ATT(path, var, text) std::cout << path << \" = \" << text << std::endl\n"
            "  #define USE_ELT(path, var, text) std::cout << path << \" = \" << text << std::endl\n";
        else
          *ctx->os <<
            "  #define USE_ATT(path, text) std::cout << path << \" = \" << text << std::endl\n"
            "  #define USE_ELT(path, text) std::cout << path << \" = \" << text << std::endl\n";

        in_gen_cpp(ctx, dom, nsmap, xroot, path, 2);
      }
    }

    if (xpath)
    {
      *ctx->os <<
        "  // XPath: " << xpath << "\n";

      if (!ifile || !genread)
      {
        if (explain)
          *ctx->os <<
            "  // DOM attribute and element pointers to inspect DOM\n";
        *ctx->os <<
          "  xsd__anyAttribute *att;\n"
          "  xsd__anyType *elt;\n";
      }

      if (!xcode)
      {
        if (collect)
        {
          if (explain)
            *ctx->os << "  // XPath query yields an array 'y' of XPath query results:\n";
          *ctx->os <<
            "  xsd__anyType y(ctx, \"results\");\n"
            "  #define QUERY_YIELD_ATT(v) y.add(xsd__anyType(ctx, \"result\").add(v));\n"
            "  #define QUERY_YIELD_ELT(v) y.add(xsd__anyType(ctx, \"result\").add(v));\n";
        }
        else
        {
          if (explain)
            *ctx->os << "  // XPath query yields are shown for each XPath result:\n";
          *ctx->os <<
            "  #define QUERY_YIELD_ATT(v) std::cout << v.get_text() << std::endl\n"
            "  #define QUERY_YIELD_ELT(v) std::cout << v << std::endl\n";
        }
      }

      std::string root(xroot);

      if (explain)
      {
        *ctx->os <<
          "  // Synopsis of variables used:\n"
          "  //     " << root << "\troot element node\n"
          << ( collect ? "  //     y\tyields query results\n" : "" ) <<
          "  //     v\tcurrent element or attribute node reference, to use in [?expr]\n"
          "  //     it\tloop iterator or pointer over elements and attributes\n"
          "  //     att\tattribute node pointer\n"
          "  //     elt\telement node pointer\n"
          "  //     pos\tXPath position()\n";
      }

      path_gen_cpp(ctx, xpath, root, true, false, 2, 0, false);

      if (!minimal && collect)
        *ctx->os << "  std::cout << y << std::endl;\n";
    }
    else if (!minimal)
    {
      if (explain)
        *ctx->os << "  // output DOM " << xroot << " in XML\n";
      *ctx->os << "  std::cout << " << xroot << " << std::endl;\n";
    }

    if (!minimal)
    {
      *ctx->os <<
        "\n"
        "  soap_destroy(ctx); // delete objects\n"
        "  soap_end(ctx);     // delete DOM data\n"
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
//      C gen DOM writer/reader from infile
//
////////////////////////////////////////////////////////////////////////////////

static void out_gen_c(soap *ctx, xsd__anyType& v, const char *current, const std::map<std::string,const char*>& nsmap, const std::string& lhs, const std::string& path, int k)
{
  for (xsd__anyAttribute::iterator i = v.att_begin(); i != v.att_end(); ++i)
  {
    if (i->tag())
    {
      if (strncmp(i->tag(), "xmlns", 5))
      {
        if (explain)
          indent(ctx, k) << "/* " << path << "/@" << putstr(i->tag(), 0) << " = " << putstr(*i, 0) << " */\n";
        bool is_b, is_n, is_x;
        LONG64 n = 0;
        double x = 0.0;
        const char *text = *i;
        if (text)
        {
          if ((is_b = is_bool(text)))
            indent(ctx, k) << "soap_att_bool(";
          else if ((is_n = is_int(ctx, text, n)))
            indent(ctx, k) << "soap_att_int(";
          else if ((is_x = is_double(ctx, text, x)))
            indent(ctx, k) << "soap_att_double(";
          else
            indent(ctx, k) << "soap_att_text(";
        }
        else
          indent(ctx, k);
        if (need_ns(i->ns(), i->tag(), nsmap, current))
          *ctx->os << "soap_att(" << lhs << ", " << putstr(i->ns()) << ", " << putstr(i->tag()) << ")";
        else
          *ctx->os << "soap_att(" << lhs << ", NULL, " << putstr(i->tag()) << ")";
        if (text)
        {
          *ctx->os << ", ";
          if (is_b)
            *ctx->os << (*text == 't');
          else if (is_n)
            *ctx->os << n << "LL";
          else if (is_x)
            *ctx->os << x;
          else
            *ctx->os << putstr(text);
          *ctx->os << ")";
        }
        *ctx->os << ";\n";
      }
    }
  }

  for (xsd__anyType::iterator i = v.elt_begin(); i != v.elt_end(); ++i)
  {
    size_t attno = 0;
    for (xsd__anyAttribute::iterator j = i->att_begin(); j != i->att_end(); ++j)
      if (j->tag() && strncmp(j->tag(), "xmlns", 5))
        attno++;

    if (attno > 0 || i->elt_size() > 0)
    {
      if (i->tag() != NULL)
      {
        std::string newpath = path;
        newpath.append("/").append(i->tag());
        int nth = i->nth();
        if (explain)
        {
          indent(ctx, k) << "/* " << newpath << " */\n";
          indent(ctx, k) << "/*" << std::setw(2*i->depth()) << "" << std::setw(0) << "<" << putstr(i->tag(), 0);
          for (xsd__anyAttribute::iterator j = i->att_begin(); j != i->att_end(); ++j)
            *ctx->os << " " << putstr(j->tag(), 0) << "=" << putstr(*j);
          if (i->elt_size() > 0 || i->get_text())
            *ctx->os << "> */\n";
          else
            *ctx->os << "/> */\n";
        }
        std::string newlhs;
        if (optimal)
        {
          indent(ctx, k) << "{\n";
          k += 2;
          newlhs = gen_ident(lhs, i->tag());
          if (nth > 0)
          {
            if (need_ns(i->ns(), i->tag(), nsmap, current))
              indent(ctx, k) << "xsd__anyType *" << newlhs << " = soap_nth_elt(" << lhs << ", " << putstr(i->ns()) << ", " << putstr(i->tag()) << ", " << nth << ");\n";
            else
              indent(ctx, k) << "xsd__anyType *" << newlhs << " = soap_nth_elt(" << lhs << ", NULL, " << putstr(i->tag()) << ", " << nth << ");\n";
          }
          else
          {
            if (need_ns(i->ns(), i->tag(), nsmap, current))
              indent(ctx, k) << "xsd__anyType *" << newlhs << " = soap_elt(" << lhs << ", " << putstr(i->ns()) << ", " << putstr(i->tag()) << ");\n";
            else
              indent(ctx, k) << "xsd__anyType *" << newlhs << " = soap_elt(" << lhs << ", NULL, " << putstr(i->tag()) << ");\n";
          }
        }
        else
        {
          if (nth > 0)
          {
            newlhs = "soap_nth_elt(";
            newlhs.append(lhs);
            if (need_ns(i->ns(), i->tag(), nsmap, current))
              newlhs.append(", ").append(putstrname(i->ns())).append(", ").append(putstrname(i->tag())).append(", ").append(soap_int2s(ctx, nth)).append(")");
            else
              newlhs.append(", NULL, ").append(putstrname(i->tag())).append(", ").append(soap_int2s(ctx, nth)).append(")");
          }
          else
          {
            newlhs = "soap_elt(";
            newlhs.append(lhs);
            if (need_ns(i->ns(), i->tag(), nsmap, current))
              newlhs.append(", ").append(putstrname(i->ns())).append(", ").append(putstrname(i->tag())).append(")");
            else
              newlhs.append(", NULL, ").append(putstrname(i->tag())).append(")");
          }
        }
        if (nth == 0 && i->ns() && !is_qualified(i->tag()))
          out_gen_c(ctx, *i, i->ns(), nsmap, newlhs, newpath, k);
        else
          out_gen_c(ctx, *i, current, nsmap, newlhs, newpath, k);
        if (optimal)
        {
          k -= 2;
          indent(ctx, k) << "}\n";
        }
        if (explain && (i->elt_size() > 0 || i->get_text()))
          indent(ctx, k) << "/*" << std::setw(2*i->depth()) << "" << std::setw(0) << "</" << putstr(i->tag(), 0) << "> */\n";
      }
    }
    else
    {
      const char *text = *i;
      int nth = i->nth();
      if (explain)
      {
        if (i->tag() == NULL)
        {
          if (text && *text)
            indent(ctx, k) << "/* " << path << " = " << putstr(text, 0) << " */\n";
        }
        else
        {
          indent(ctx, k) << "/* " << path << "/" << putstr(i->tag(), 0);
          if (nth > 0)
            *ctx->os << "[" << nth << "]";
          if (text && *text)
            *ctx->os << " = " << putstr(text, 0) << "*/\n";
          else
            *ctx->os << "*/\n";
          if (i->att_size() > 0)
          {
            indent(ctx, k) << "/*" << std::setw(2*i->depth()) << "" << std::setw(0) << "<" << putstr(i->tag(), 0);
            for (xsd__anyAttribute::iterator j = i->att_begin(); j != i->att_end(); ++j)
              *ctx->os << " " << putstr(j->tag(), 0) << "=" << putstr(*j);
            if (text)
              *ctx->os << ">" << putstr(text, 0) << "</" << putstr(i->tag(), 0) << "> */\n";
            else
              *ctx->os << "/> */\n";
          }
        }
      }
      bool is_b, is_n, is_x;
      LONG64 n = 0;
      double x = 0.0;
      if (text)
      {
        if ((is_b = is_bool(text)))
          indent(ctx, k) << "soap_elt_bool(";
        else if ((is_n = is_int(ctx, text, n)))
          indent(ctx, k) << "soap_elt_int(";
        else if ((is_x = is_double(ctx, text, x)))
          indent(ctx, k) << "soap_elt_double(";
        else
          indent(ctx, k) << "soap_elt_text(";
      }
      else
        indent(ctx, k);
      if (i->tag() == NULL)
        *ctx->os << "soap_elt(" << lhs << ", NULL, NULL)";
      else
      {
        if (nth > 0)
        {
          if (need_ns(i->ns(), i->tag(), nsmap, current))
            *ctx->os << "soap_nth_elt(" << lhs << ", " << putstr(i->ns()) << ", " << putstr(i->tag()) << ", " << nth << ")";
          else
            *ctx->os << "soap_nth_elt(" << lhs << ", NULL, " << putstr(i->tag()) << ", " << nth << ")";
        }
        else
        {
          if (need_ns(i->ns(), i->tag(), nsmap, current))
            *ctx->os << "soap_elt(" << lhs << ", " << putstr(i->ns()) << ", " << putstr(i->tag()) << ")";
          else
            *ctx->os << "soap_elt(" << lhs << ", NULL, " << putstr(i->tag()) << ")";
        }
      }
      if (text)
      {
        *ctx->os << ", ";
        if (is_b)
          *ctx->os << (*text == 't');
        else if (is_n)
          *ctx->os << n << "LL";
        else if (is_x)
          *ctx->os << x;
        else
          *ctx->os << putstr(text);
        *ctx->os << ")";
      }
      *ctx->os << ";\n";
    }
  }

  const char *text = v;
  if (text)
  {
    if (explain && *text)
      indent(ctx, k) << "/* " << path << " = " << putstr(text, 0) << " */\n";
    LONG64 n;
    double x;
    if (is_bool(text))
      indent(ctx, k) << "soap_elt_bool(" << lhs << ", " << (*text == 't');
    else if (is_int(ctx, text, n))
      indent(ctx, k) << "soap_elt_int(" << lhs << ", " << n << "LL";
    else if (is_double(ctx, text, x))
      indent(ctx, k) << "soap_elt_double(" << lhs << ", " << x;
    else
      indent(ctx, k) << "soap_elt_text(" << lhs << ", " << putstr(text);
    *ctx->os << ");\n";
  }
}

static void in_gen_c(soap *ctx, xsd__anyType& v, const std::map<std::string,const char*>& nsmap, const std::string& lhs, const std::string& path, int k)
{
  for (xsd__anyAttribute::iterator i = v.att_begin(); i != v.att_end(); ++i)
  {
    if (i->tag())
    {
      if (strncmp(i->tag(), "xmlns", 5))
      {
        if (explain)
          indent(ctx, k) << "/* " << path << "/@" << putstr(i->tag(), 0) << " = " << putstr(*i, 0) << " */\n";
        if (genvars)
        {
          bool is_b = false;
          bool is_n = false;
          bool is_x = false;
          std::string newlhs = gen_ident(lhs, i->tag());
          indent(ctx, k) << "{\n";
          const char *text = *i;
          if (text && *text)
          {
            LONG64 n;
            double x;
            if ((is_b = is_bool(text)))
              indent(ctx, k + 2) << "short " << newlhs << "_att = " << (*text == 't') << "; /* a default bool value */\n";
            else if ((is_n = is_int(ctx, text, n)))
              indent(ctx, k + 2) << "LONG64 " << newlhs << "_att = " << n << "LL; /* a default int value */\n";
            else if ((is_x = is_double(ctx, text, x)))
              indent(ctx, k + 2) << "double " << newlhs << "_att = " << x << "; /* a default float value */\n";
            else
              indent(ctx, k + 2) << "const char *" << newlhs << "_att = " << putstr(text) << "; /* a default string value */\n";
          }
          else
          {
            indent(ctx, k + 2) << "const char *" << newlhs << "_att = NULL;\n";
          }
          if (need_ns(i->ns(), i->tag(), nsmap, NULL))
            indent(ctx, k + 2) << "if ((att = soap_att_get(" << lhs << ", " << putstr(i->ns()) << ", " << putstr(name(i->tag())) << ")))\n";
          else
            indent(ctx, k + 2) << "if ((att = soap_att_get(" << lhs << ", NULL, " << putstr(i->tag()) << ")))\n";
          if (is_b)
            indent(ctx, k + 4) << "USE_ATT(\"" << path << "/@" << putstr(i->tag(), 0) << "\", " << newlhs << "_att = soap_att_is_true(att), soap_att_get_text(att));\n";
          else if (is_n)
            indent(ctx, k + 4) << "USE_ATT(\"" << path << "/@" << putstr(i->tag(), 0) << "\", " << newlhs << "_att = soap_att_get_LONG64(att), soap_att_get_text(att));\n";
          else if (is_x)
            indent(ctx, k + 4) << "USE_ATT(\"" << path << "/@" << putstr(i->tag(), 0) << "\", " << newlhs << "_att = soap_att_get_double(att), soap_att_get_text(att));\n";
          else
            indent(ctx, k + 4) << "USE_ATT(\"" << path << "/@" << putstr(i->tag(), 0) << "\", " << newlhs << "_att = soap_att_get_text(att), soap_att_get_text(att));\n";
          indent(ctx, k + 2) << "(void)" << newlhs << "_att;\n";
          indent(ctx, k) << "}\n";
        }
        else
        {
          if (need_ns(i->ns(), i->tag(), nsmap, NULL))
            indent(ctx, k) << "if ((att = soap_att_get(" << lhs << ", " << putstr(i->ns()) << ", " << putstr(name(i->tag())) << ")))\n";
          else
            indent(ctx, k) << "if ((att = soap_att_get(" << lhs << ", NULL, " << putstr(i->tag()) << ")))\n";
          indent(ctx, k + 2) << "USE_ATT(\"" << path << "/@" << putstr(i->tag(), 0) << "\", soap_att_get_text(att));\n";
        }
      }
    }
  }

  for (xsd__anyType::iterator i = v.elt_begin(); i != v.elt_end(); ++i)
  {
    if (i->tag())
    {
      bool is_done = true;
      bool is_repeat = false;
      for (xsd__anyType::iterator j = v.elt_begin(); j != v.elt_end(); ++j)
      {
        if (j->tag() && !strcmp(i->tag(), j->tag()))
        {
          if (i == j)
            is_done = false;
          else
          {
            if (!is_done)
              is_repeat = true;
            break;
          }
        }
      }
      if (!is_done)
      {
        std::string newpath = path;
        newpath.append("/").append(i->tag());
        if (explain)
        {
          if (is_repeat)
            indent(ctx, k) << "/* a repetition of elements " << newpath << " */\n";
          else
            indent(ctx, k) << "/* " << newpath << " */\n";
        }
        std::string newlhs = gen_ident(lhs, i->tag());
        if (is_repeat)
        {
          indent(ctx, k) << "xsd__anyType *" << newlhs << ";\n";
          if (need_ns(i->ns(), i->tag(), nsmap, NULL))
            indent(ctx, k) << "for (" << newlhs << " = soap_elt_get(" << lhs << ", " << putstr(i->ns()) << ", " << putstr(name(i->tag())) << "); " << newlhs << "; " << newlhs << " = soap_elt_get_next(" << newlhs << "))\n";
          else
            indent(ctx, k) << "for (" << newlhs << " = soap_elt_get(" << lhs << ", NULL, " << putstr(i->tag()) << "); " << newlhs << "; " << newlhs << " = soap_elt_get_next(" << newlhs << "))\n";
        }
        else
        {
          if (need_ns(i->ns(), i->tag(), nsmap, NULL))
            indent(ctx, k) << "if ((elt = soap_elt_get(" << lhs << ", " << putstr(i->ns()) << ", " << putstr(name(i->tag())) << ")))\n";
          else
            indent(ctx, k) << "if ((elt = soap_elt_get(" << lhs << ", NULL, " << putstr(i->tag()) << ")))\n";
        }
        indent(ctx, k) << "{\n";
        if (!is_repeat)
          indent(ctx, k + 2) << "xsd__anyType *" << newlhs << " = elt;\n";
        in_gen_c(ctx, *i, nsmap, newlhs, newpath, k + 2);
        indent(ctx, k) << "}\n";
      }
    }
  }

  const char *text = v;
  if (text)
  {
    if (explain && *text)
      indent(ctx, k) << "/* " << path << " = " << putstr(text, 0) << " */\n";
    if (genvars)
    {
      bool is_b = false;
      bool is_n = false;
      bool is_x = false;
      indent(ctx, k) << "{\n";
      if (*text)
      {
        LONG64 n;
        double x;
        if ((is_b = is_bool(text)))
          indent(ctx, k + 2) << "short " << lhs << "_elt = " << (*text == 't') << "; /* a default bool value */\n";
        else if ((is_n = is_int(ctx, text, n)))
          indent(ctx, k + 2) << "LONG64 " << lhs << "_elt = " << n << "LL; /* a default int value */\n";
        else if ((is_x = is_double(ctx, text, x)))
          indent(ctx, k + 2) << "double " << lhs << "_elt = " << x << "; /* a default float value */\n";
        else
          indent(ctx, k + 2) << "const char *" << lhs << "_elt = " << putstr(text) << "; /* a default string value */\n";
      }
      else
      {
        indent(ctx, k + 2) << "const char *" << lhs << "_elt = NULL;\n";
      }
      indent(ctx, k + 2) << "if (soap_elt_get_text(" << lhs << "))\n";
      if (is_b)
        indent(ctx, k + 4) << "USE_ELT(\"" << path << "\", " << lhs << "_elt = soap_elt_is_true(" << lhs << "), soap_elt_get_text(" << lhs << "));\n";
      else if (is_n)
        indent(ctx, k + 4) << "USE_ELT(\"" << path << "\", " << lhs << "_elt = soap_elt_get_LONG64(" << lhs << "), soap_elt_get_text(" << lhs << "));\n";
      else if (is_x)
        indent(ctx, k + 4) << "USE_ELT(\"" << path << "\", " << lhs << "_elt = soap_elt_get_double(" << lhs << "), soap_elt_get_text(" << lhs << "));\n";
      else
        indent(ctx, k + 4) << "USE_ELT(\"" << path << "\", " << lhs << "_elt = soap_elt_get_text(" << lhs << "), soap_elt_get_text(" << lhs << "));\n";
      indent(ctx, k + 2) << "(void)" << lhs << "_elt;\n";
      indent(ctx, k) << "}\n";
    }
    else
    {
      indent(ctx, k) << "if (soap_elt_get_text(" << lhs << "))\n";
      indent(ctx, k + 2) << "USE_ELT(\"" << path << "\", soap_elt_get_text(" << lhs << "));\n";
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
//
//      C++ gen DOM writer/reader from infile
//
////////////////////////////////////////////////////////////////////////////////

static void out_gen_cpp(soap *ctx, xsd__anyType& v, const char *current, const std::map<std::string,const char*>& nsmap, const std::string& lhs, const std::string& path, int k)
{
  for (xsd__anyAttribute::iterator i = v.att_begin(); i != v.att_end(); ++i)
  {
    if (i->tag())
    {
      if (strncmp(i->tag(), "xmlns", 5))
      {
        if (explain)
          indent(ctx, k) << "// " << path << "/@" << putstr(i->tag(), 0) << " = " << putstr(*i, 0) << "\n";
        if (need_ns(i->ns(), i->tag(), nsmap, current))
          indent(ctx, k) << lhs << ".att(" << putstr(i->ns()) << ", " << putstr(i->tag()) << ")";
        else
          indent(ctx, k) << lhs << ".att(" << putstr(i->tag()) << ")";
        const char *text = *i;
        if (text)
        {
          *ctx->os << " = ";
          LONG64 n;
          double x;
          if (is_bool(text))
            *ctx->os << text;
          else if (is_int(ctx, text, n))
            *ctx->os << n << "LL";
          else if (is_double(ctx, text, x))
            *ctx->os << x;
          else
            *ctx->os << putstr(text);
        }
        *ctx->os << ";\n";
      }
    }
  }

  for (xsd__anyType::iterator i = v.elt_begin(); i != v.elt_end(); ++i)
  {
    size_t attno = 0;
    for (xsd__anyAttribute::iterator j = i->att_begin(); j != i->att_end(); ++j)
      if (j->tag() && strncmp(j->tag(), "xmlns", 5))
        attno++;

    if (attno > 0 || i->elt_size() > 0)
    {
      if (i->tag())
      {
        std::string newpath = path;
        newpath.append("/").append(i->tag());
        int nth = i->nth();
        if (nth > 0)
          newpath.append("[").append(soap_int2s(ctx, nth)).append("]");
        if (explain)
        {
          indent(ctx, k) << "// " << newpath << "\n";
          indent(ctx, k) << "//" << std::setw(2*i->depth()) << "" << std::setw(0) << "<" << putstr(i->tag(), 0);
          for (xsd__anyAttribute::iterator j = i->att_begin(); j != i->att_end(); ++j)
            *ctx->os << " " << putstr(j->tag(), 0) << "=" << putstr(*j);
          if (i->elt_size() > 0 || i->get_text())
            *ctx->os << ">\n";
          else
            *ctx->os << "/>\n";
        }
        std::string newlhs;
        if (optimal)
        {
          indent(ctx, k) << "{\n";
          k += 2;
          newlhs = gen_ident(lhs, i->tag());
          if (need_ns(i->ns(), i->tag(), nsmap, current))
            indent(ctx, k) << "xsd__anyType& " << newlhs << " = " << lhs << ".elt(" << putstr(i->ns()) << ", " << putstr(i->tag()) << ")";
          else
            indent(ctx, k) << "xsd__anyType& " << newlhs << " = " << lhs << "[" << putstr(i->tag()) << "]";
          if (nth > 0)
            *ctx->os << "[" << nth << "]";
          *ctx->os << ";\n";
        }
        else
        {
          newlhs = lhs;
          if (need_ns(i->ns(), i->tag(), nsmap, current))
            newlhs.append(".elt(").append(putstrname(i->ns())).append(", ").append(putstrname(i->tag())).append(")");
          else
            newlhs.append("[").append(putstrname(i->tag())).append("]");
          if (nth > 0)
            newlhs.append("[").append(soap_int2s(ctx, nth)).append("]");
        }
        if (nth == 0 && i->ns() && !is_qualified(i->tag()))
          out_gen_cpp(ctx, *i, i->ns(), nsmap, newlhs, newpath, k);
        else
          out_gen_cpp(ctx, *i, current, nsmap, newlhs, newpath, k);
        if (optimal)
        {
          k -= 2;
          indent(ctx, k) << "}\n";
        }
        if (explain && (i->elt_size() > 0 || i->get_text()))
          indent(ctx, k) << "//" << std::setw(2*i->depth()) << "" << std::setw(0) << "</" << putstr(i->tag(), 0) << ">\n";
      }
    }
    else
    {
      const char *text = *i;
      int nth = i->nth();
      if (explain)
      {
        if (i->tag() == NULL)
        {
          if (text && *text)
            indent(ctx, k) << "// " << path << " = " << putstr(text, 0) << "\n";
        }
        else
        {
          indent(ctx, k) << "// " << path << "/" << putstr(i->tag(), 0);
          if (nth > 0)
            *ctx->os << "[" << nth << "]";
          if (text && *text)
            *ctx->os << " = " << putstr(text, 0) << "\n";
          else
            *ctx->os << "\n";
          if (i->att_size() > 0)
          {
            indent(ctx, k) << "//" << std::setw(2*i->depth()) << "" << std::setw(0) << "<" << putstr(i->tag(), 0);
            for (xsd__anyAttribute::iterator j = i->att_begin(); j != i->att_end(); ++j)
              *ctx->os << " " << putstr(j->tag(), 0) << "=" << putstr(*j);
            if (text)
              *ctx->os << ">" << putstr(text, 0) << "</" << putstr(i->tag(), 0) << ">\n";
            else
              *ctx->os << "/>\n";
          }
        }
      }
      if (i->tag() == NULL)
        indent(ctx, k) << lhs << ".elt()";
      else
      {
        if (need_ns(i->ns(), i->tag(), nsmap, current))
          indent(ctx, k) << lhs << ".elt(" << putstr(i->ns()) << ", " << putstr(i->tag()) << ")";
        else
          indent(ctx, k) << lhs << "[" << putstr(i->tag()) << "]";
        if (nth > 0)
          *ctx->os << "[" << nth << "]";
      }
      if (text)
      {
        *ctx->os << " = ";
        LONG64 n;
        double x;
        if (is_bool(text))
          *ctx->os << text;
        else if (is_int(ctx, text, n))
          *ctx->os << n << "LL";
        else if (is_double(ctx, text, x))
          *ctx->os << x;
        else
          *ctx->os << putstr(text);
      }
      *ctx->os << ";\n";
    }
  }

  const char *text = v;
  if (text)
  {
    if (explain && *text)
      indent(ctx, k) << "// " << path << " = " << putstr(text, 0) << "\n";
    indent(ctx, k) << lhs << " = ";
    LONG64 n;
    double x;
    if (is_bool(text))
      *ctx->os << text;
    else if (is_int(ctx, text, n))
      *ctx->os << n << "LL";
    else if (is_double(ctx, text, x))
      *ctx->os << x;
    else
      *ctx->os << putstr(text);
    *ctx->os << ";\n";
  }
}

static void in_gen_cpp(soap *ctx, xsd__anyType& v, const std::map<std::string,const char*>& nsmap, const std::string& lhs, const std::string& path, int k)
{
  for (xsd__anyAttribute::iterator i = v.att_begin(); i != v.att_end(); ++i)
  {
    if (i->tag())
    {
      if (strncmp(i->tag(), "xmlns", 5))
      {
        if (explain)
          indent(ctx, k) << "// " << path << "/@" << putstr(i->tag(), 0) << " = " << putstr(*i, 0) << "\n";
        if (genvars)
        {
          std::string newlhs = gen_ident(lhs, i->tag());
          const char *text = *i;
          if (text && *text)
          {
            LONG64 n;
            double x;
            if (is_bool(text))
              indent(ctx, k) << "bool " << newlhs << "_att = " << text << "; // a default bool value\n";
            else if (is_int(ctx, text, n))
              indent(ctx, k) << "LONG64 " << newlhs << "_att = " << n << "LL; // a default int value\n";
            else if (is_double(ctx, text, x))
              indent(ctx, k) << "double " << newlhs << "_att = " << x << "; // a default float value\n";
            else
              indent(ctx, k) << "const char *" << newlhs << "_att = " << putstr(text) << "; // a default string value\n";
          }
          else
          {
            indent(ctx, k) << "const char *" << newlhs << "_att = NULL;\n";
          }
          if (need_ns(i->ns(), i->tag(), nsmap, NULL))
            indent(ctx, k) << "if ((att = " << lhs << ".att_get(" << putstr(i->ns()) << ", " << putstr(name(i->tag())) << ")))\n";
          else
            indent(ctx, k) << "if ((att = " << lhs << ".att_get(" << putstr(i->tag()) << ")))\n";
          indent(ctx, k + 2) << "USE_ATT(\"" << path << "/@" << putstr(i->tag(), 0) << "\", " << newlhs << "_att = *att, att->get_text());\n";
          indent(ctx, k) << "(void)" << newlhs << "_att;\n";
        }
        else
        {
          if (need_ns(i->ns(), i->tag(), nsmap, NULL))
            indent(ctx, k) << "if ((att = " << lhs << ".att_get(" << putstr(i->ns()) << ", " << putstr(name(i->tag())) << ")))\n";
          else
            indent(ctx, k) << "if ((att = " << lhs << ".att_get(" << putstr(i->tag()) << ")))\n";
          indent(ctx, k + 2) << "USE_ATT(\"" << path << "/@" << putstr(i->tag(), 0) << "\", att->get_text());\n";
        }
      }
    }
  }

  for (xsd__anyType::iterator i = v.elt_begin(); i != v.elt_end(); ++i)
  {
    if (i->tag())
    {
      bool is_done = true;
      bool is_repeat = false;
      for (xsd__anyType::iterator j = v.elt_begin(); j != v.elt_end(); ++j)
      {
        if (j->tag() && !strcmp(i->tag(), j->tag()))
        {
          if (i == j)
            is_done = false;
          else
          {
            if (!is_done)
              is_repeat = true;
            break;
          }
        }
      }
      if (!is_done)
      {
        std::string newpath = path;
        newpath.append("/").append(i->tag());
        if (explain)
        {
          if (is_repeat)
            indent(ctx, k) << "// a repetition of elements " << newpath << "\n";
          else
            indent(ctx, k) << "// " << newpath << "\n";
        }
        std::string newlhs = gen_ident(lhs, i->tag());
        if (is_repeat)
        {
          if (need_ns(i->ns(), i->tag(), nsmap, NULL))
            indent(ctx, k) << "for (xsd__anyType *it = " << lhs << ".elt_get(" << putstr(i->ns()) << ", " << putstr(name(i->tag())) << "); it; it = it->get_next())\n";
          else
            indent(ctx, k) << "for (xsd__anyType *it = " << lhs << ".elt_get(" << putstr(i->tag()) << "); it; it = it->get_next())\n";
        }
        else
        {
          if (need_ns(i->ns(), i->tag(), nsmap, NULL))
            indent(ctx, k) << "if ((elt = " << lhs << ".elt_get(" << putstr(i->ns()) << ", " << putstr(name(i->tag())) << ")))\n";
          else
            indent(ctx, k) << "if ((elt = " << lhs << ".elt_get(" << putstr(i->tag()) << ")))\n";
        }
        indent(ctx, k) << "{\n";
        if (is_repeat)
          indent(ctx, k + 2) << "xsd__anyType& " << newlhs << " = *it;\n";
        else
          indent(ctx, k + 2) << "xsd__anyType& " << newlhs << " = *elt;\n";
        in_gen_cpp(ctx, *i, nsmap, newlhs, newpath, k + 2);
        indent(ctx, k) << "}\n";
      }
    }
  }

  const char *text = v;
  if (text)
  {
    if (explain && *text)
      indent(ctx, k) << "// " << path << " = " << putstr(text, 0) << "\n";
    if (genvars)
    {
      if (*text)
      {
        LONG64 n;
        double x;
        if (is_bool(text))
          indent(ctx, k) << "bool " << lhs << "_elt = " << text << "; // a default bool value\n";
        else if (is_int(ctx, text, n))
          indent(ctx, k) << "LONG64 " << lhs << "_elt = " << n << "LL; // a default int value\n";
        else if (is_double(ctx, text, x))
          indent(ctx, k) << "double " << lhs << "_elt = " << x << "; // a default float value\n";
        else
          indent(ctx, k) << "const char *" << lhs << "_elt = " << putstr(text) << "; // a default string value\n";
      }
      else
      {
        indent(ctx, k) << "const char *" << lhs << "_elt = NULL;\n";
      }
      indent(ctx, k) << "if (" << lhs << ".get_text())\n";
      indent(ctx, k + 2) << "USE_ELT(\"" << path << "\", " << lhs << "_elt = " << lhs << ", " << lhs << ".get_text());\n";
      indent(ctx, k) << "(void)" << lhs << "_elt;\n";
    }
    else
    {
      indent(ctx, k) << "if (" << lhs << ".get_text())\n";
      indent(ctx, k + 2) << "USE_ELT(\"" << path << "\", " << lhs << ".get_text());\n";
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
//
//      C gen XPath from -ppath
//
////////////////////////////////////////////////////////////////////////////////

static void path_gen_c(struct soap *ctx, const char *xpath, std::string& v, bool atroot, bool attrib, int k, size_t parens, size_t label)
{
  if (*xpath == ')') // path: ...)...
  {
    if (parens == 0)
    {
      fprintf(stderr, "domcpp: XPath ')' without '(' ...->%s\n\n", xpath);
      exit(EXIT_FAILURE);
    }
    ++xpath;
    --parens;
    indent(ctx, k) << "size_t pos = ++pos" << parens << ";\n";
  }
  if (*xpath == '/' && *(xpath + 1) != '/') // skip single /
  {
    if (attrib)
    {
      fprintf(stderr, "domcpp: XPath / follows attribute at ...->%s\n\n", xpath);
      exit(EXIT_FAILURE);
    }
    ++xpath;
  }
  if (*xpath == '(') // path: (...
  {
    ++xpath;
    indent(ctx, k) << "size_t pos" << parens << " = 0;\n";
    ++parens;
  }
  if (*xpath == '.')
  {
    ++xpath;
    if (*xpath == '.')
    {
      if (atroot)
      {
        fprintf(stderr, "domcpp: XPath .. at root ...->%s\n\n", xpath);
        exit(EXIT_FAILURE);
      }
      indent(ctx, k) << "if ((elt = soap_elt_parent(" << v << ")))\n";
      indent(ctx, k) << "{\n";
      indent(ctx, k + 2) << "xsd__anyType *v = elt\n";
      v = "v";
      path_gen_c(ctx, xpath, v, false, attrib, k + 2, parens, label);
      indent(ctx, k) << "}\n";
    }
    else
    {
      path_gen_c(ctx, xpath, v, false, attrib, k, parens, label);
    }
  }
  else if (*xpath == '/') // path: //...
  {
    ++xpath;
    if (*xpath == '/')
    {
      if (attrib)
      {
        fprintf(stderr, "domcpp: XPath // follows attribute at ...->%s\n\n", xpath);
        exit(EXIT_FAILURE);
      }
      ++xpath;
      if (explain)
      {
        if (atroot)
          indent(ctx, k) << "/* iterate over descendants and current node " << v << " to match " << putstr(xpath, 0) << " */\n";
        else
          indent(ctx, k) << "/* iterate over descendants of current node " << v << " to match " << putstr(xpath, 0) << " */\n";
      }
      indent(ctx, k) << "xsd__anyType *it;\n";
      indent(ctx, k) << "for (it = " << v << "; it; it = soap_dom_next_element(it, " << v << "))\n";
      indent(ctx, k) << "{\n";
      indent(ctx, k + 2) << "xsd__anyType *v = it;\n";
      v = "v";
      path_gen_c(ctx, xpath, v, atroot, false, k + 2, parens, label);
      indent(ctx, k) << "}\n";
    }
    else
    {
      fprintf(stderr, "domcpp: XPath unexpected end at '/'\n\n");
      exit(EXIT_FAILURE);
    }
  }
  else if (*xpath == '[') // path: [...]...
  {
    size_t nth = getnth(&xpath);
    if (nth) // path: [n]... where n is an integer > 0 position
    {
      if (explain)
        indent(ctx, k) << "/* if position of current node " << v << " is " << nth << " then match " << putstr(xpath, 0) << " */\n";
      indent(ctx, k) << "if (pos == " << nth << ")\n";
      indent(ctx, k) << "{\n";
      path_gen_c(ctx, xpath, v, false, attrib, k + 2, parens, label);
      indent(ctx, k) << "}\n";
    }
    else
    {
      ++xpath;
      int nest = 0;
      const char *s;
      for (s = xpath; *s; ++s)
      {
        if (*s == ']')
        {
          if (nest == 0)
            break;
          --nest;
        }
        else if (*s == '[')
          ++nest;
      }
      if (*s != ']')
      {
        fprintf(stderr, "domcpp: XPath ']' expected at ...->%s\n\n", xpath);
        exit(EXIT_FAILURE);
      }
      if (*xpath == '?') // path: [?e]... where e is a C boolean (nonzero is true) expression
      {
        std::string code(xpath + 1, s - xpath - 1);
        xpath = s + 1;
        if (explain)
          indent(ctx, k) << "/* filter current node " << v << " with 'if (" << code << ")' to match " << putstr(xpath, 0) << " */\n";
        if (v != "v")
        {
          if (attrib)
            indent(ctx, k) << "xsd__anyAttribute *v = " << v << ";\n";
          else
            indent(ctx, k) << "xsd__anyType *v = " << v << ";\n";
        }
        indent(ctx, k) << "if (" << code << ")\n";
        indent(ctx, k) << "{\n";
        path_gen_c(ctx, xpath, v, atroot, attrib, k + 2, parens, label);
        indent(ctx, k) << "}\n";
      }
      else // path: [p]... where p is an XPath to match to continue
      {
        if (explain)
          indent(ctx, k) << "/* check current node " << v << " matches " << putstr(xpath, 0) << " */\n";
        std::string u = v;
        static size_t state = 0;
        size_t found = ++state;
        if (*xpath == '/' && *(xpath + 1) != '/')
        {
          v = "dom";
          path_gen_c(ctx, xpath, v, true, attrib, k, 0, found);
        }
        else
        {
          path_gen_c(ctx, xpath, v, atroot, attrib, k, 0, found);
        }
        indent(ctx, k) << "if (0)\n";
        indent(ctx, 2) << "found" << found << ":\n";
        indent(ctx, k) << "{\n";
        xpath = s + 1;
        path_gen_c(ctx, xpath, u, atroot, attrib, k + 2, parens, label);
        indent(ctx, k) << "}\n";
      }
    }
  }
  else if (*xpath == ']')
  {
    if (!label)
    {
      fprintf(stderr, "domcpp: XPath ']' without '[' ...->%s\n\n", xpath);
      exit(EXIT_FAILURE);
    }
    indent(ctx, k) << "(void)" << v << ";\n";
    indent(ctx, k) << "goto found" << label << ";\n";
  }
  else if (!*xpath)
  {
    if (parens)
    {
      fprintf(stderr, "domcpp: XPath ')' expected ...->%s\n\n", xpath);
      exit(EXIT_FAILURE);
    }
    path_exec_c(ctx, v, attrib, k);
  }
  else if (!strncmp(xpath, "text()", 6)) // path: text()
  {
    xpath += 6;
    size_t nth = getnth(&xpath);
    if (nth == 1)
    {
      if (explain)
        indent(ctx, k) << "/* if current node " << v << " has a text-only element then match " << putstr(xpath, 0) << " */\n";
      indent(ctx, k) << "if ((elt = soap_elt_get(" << v << ", NULL, \"\")))\n";
    }
    else if (nth > 1)
    {
      if (explain)
        indent(ctx, k) << "/* if current node " << v << " has a " << nth << "-th text-only element then match " << putstr(xpath, 0) << " */\n";
      indent(ctx, k) << "if ((elt = soap_elt_get(" << v << ", NULL, \"\")) && (elt = soap_elt_get_nth(elt, " << nth << ")))\n";
    }
    else
    {
      if (explain)
        indent(ctx, k) << "/* for each text-only element of current node " << v << " match " << putstr(xpath, 0) << " */\n";
      indent(ctx, k) << "size_t pos = 1;\n";
      indent(ctx, k) << "xsd__anyType *it;\n";
      indent(ctx, k) << "for (it = soap_elt_get(" << v << ", NULL, \"\"); it; it = soap_elt_get_next(it), ++pos)\n";
    }
    indent(ctx, k) << "{\n";
    if (nth)
      indent(ctx, k + 2) << "xsd__anyType *v = elt;\n";
    else
      indent(ctx, k + 2) << "xsd__anyType *v = it;\n";
    v = "v";
    path_gen_c(ctx, xpath, v, false, false, k + 2, parens, label);
    indent(ctx, k) << "}\n";
  }
  else if (atroot && *xpath != '@') // path: /name...
  {
    const char *tag = getname(&xpath).c_str();
    if (explain)
      indent(ctx, k) << "/* if node " << v << " is '" << tag << "' then match " << putstr(xpath, 0) << " */\n";
    indent(ctx, k) << "if (soap_elt_match(" << v << ", NULL, " << putstr(tag) << "))\n";
    indent(ctx, k) << "{\n";
    path_gen_c(ctx, xpath, v, false, false, k + 2, parens, label);
    indent(ctx, k) << "}\n";
  }
  else // path: name..., name[n]..., @name, *..., *:*..., @*..., @*:*...
  {
    const char *tag = getname(&xpath).c_str();
    size_t nth = 0;
    bool is_att = (*tag == '@');
    bool is_patt = strchr(tag, '*') != NULL;
    bool is_wild = !strcmp(tag + is_att, "*") || !strcmp(tag + is_att, "*:*");
    if (!is_patt)
      nth = getnth(&xpath);
    if (is_att && is_wild)
    {
      if (explain)
        indent(ctx, k) << "/* for each attribute of current node " << v << " match " << putstr(xpath, 0) << " */\n";
      indent(ctx, k) << "xsd__anyAttribute *it;\n";
      indent(ctx, k) << "for (it = soap_att_first(" << v << "); it; it = soap_att_next(it))\n";
    }
    else if (is_att && is_patt)
    {
      if (explain)
        indent(ctx, k) << "/* for each attribute '" << tag + 1 << "' of current node " << v << " match " << putstr(xpath, 0) << " */\n";
      indent(ctx, k) << "xsd__anyAttribute *it;\n";
      indent(ctx, k) << "for (it = soap_att_find(" << v << ", NULL, " << putstr(tag) << "); it; it = soap_att_find_next(it, NULL, " << putstr(tag) << "))\n";
    }
    else if (is_att)
    {
      if (explain)
        indent(ctx, k) << "/* if current node " << v << " has an attribute '" << tag + 1 << "' then match " << putstr(xpath, 0) << " */\n";
      indent(ctx, k) << "if ((att = soap_att_get(" << v << ", NULL, " << putstr(tag + 1) << ")))\n";
    }
    else if (is_wild)
    {
      if (explain)
        indent(ctx, k) << "/* for each element of current node " << v << " match " << putstr(xpath, 0) << " */\n";
      indent(ctx, k) << "size_t pos = 1;\n";
      indent(ctx, k) << "xsd__anyType *it;\n";
      indent(ctx, k) << "for (it = soap_elt_first(" << v << "); it; it = soap_elt_next(it), ++pos)\n";
    }
    else if (is_patt)
    {
      if (explain)
        indent(ctx, k) << "/* for each element '" << tag << "' of current node " << v << " match " << putstr(xpath, 0) << " */\n";
      indent(ctx, k) << "size_t pos = 1;\n";
      indent(ctx, k) << "xsd__anyType *it;\n";
      indent(ctx, k) << "for (it = soap_elt_find(" << v << ", NULL, " << putstr(tag) << "); it; it = soap_elt_find_next(it, NULL, " << putstr(tag) << ", ++pos)\n";
    }
    else if (nth == 1)
    {
      if (explain)
        indent(ctx, k) << "/* if current node " << v << " has an element '" << tag << "' then match " << putstr(xpath, 0) << " */\n";
      indent(ctx, k) << "if ((elt = soap_elt_get(" << v << ", NULL, " << putstr(tag) << ")))\n";
    }
    else if (nth > 1)
    {
      if (explain)
        indent(ctx, k) << "/* if current node " << v << " has a " << nth << "-th element '" << tag << "' then match " << putstr(xpath, 0) << " */\n";
      indent(ctx, k) << "if ((elt = soap_elt_get(" << v << ", NULL, " << putstr(tag) << ")) && (elt = soap_elt_get_nth(elt, " << nth << ")))\n";
    }
    else
    {
      if (explain)
        indent(ctx, k) << "/* for each element '" << tag << "' of current node " << v << " match " << putstr(xpath, 0) << " */\n";
      indent(ctx, k) << "size_t pos = 1;\n";
      indent(ctx, k) << "xsd__anyType *it;\n";
      indent(ctx, k) << "for (it = soap_elt_get(" << v << ", NULL, " << putstr(tag) << "); it; it = soap_elt_get_next(it), ++pos)\n";
    }
    indent(ctx, k) << "{\n";
    if (is_att && is_patt)
      indent(ctx, k + 2) << "xsd__anyAttribute *v = it;\n";
    else if (is_att)
      indent(ctx, k + 2) << "xsd__anyAttribute *v = att;\n";
    else if (nth)
      indent(ctx, k + 2) << "xsd__anyType *v = elt;\n";
    else
      indent(ctx, k + 2) << "xsd__anyType *v = it;\n";
    v = "v";
    path_gen_c(ctx, xpath, v, false, is_att, k + 2, parens, label);
    indent(ctx, k) << "}\n";
  }
}

static void path_exec_c(struct soap *ctx, std::string& v, bool attrib, int k)
{
  if (xcode)
  {
    if (attrib)
      indent(ctx, k) << "xsd__anyAttribute *v = " << v << ";\n";
    else
      indent(ctx, k) << "xsd__anyType *v = " << v << ";\n";
    indent(ctx, k) << xcode << "\n";
  }
  else
  {
    if (attrib)
      indent(ctx, k) << "QUERY_YIELD_ATT(" << v << ");\n";
    else
      indent(ctx, k) << "QUERY_YIELD_ELT(" << v << ");\n";
  }
}

////////////////////////////////////////////////////////////////////////////////
//
//      C++ gen XPath from -ppath
//
////////////////////////////////////////////////////////////////////////////////

static void path_gen_cpp(struct soap *ctx, const char *xpath, std::string& v, bool atroot, bool attrib, int k, size_t parens, bool throwup)
{
  if (*xpath == ')') // path: ...)...
  {
    if (parens == 0)
    {
      fprintf(stderr, "domcpp: XPath ')' without '(' ...->%s\n\n", xpath);
      exit(EXIT_FAILURE);
    }
    ++xpath;
    --parens;
    indent(ctx, k) << "size_t pos = ++pos" << parens << ";\n";
  }
  if (*xpath == '/' && *(xpath + 1) != '/') // skip single /
  {
    if (attrib)
    {
      fprintf(stderr, "domcpp: XPath / follows attribute at ...->%s\n\n", xpath);
      exit(EXIT_FAILURE);
    }
    ++xpath;
  }
  if (*xpath == '(') // path: (...
  {
    ++xpath;
    indent(ctx, k) << "size_t pos" << parens << " = 0;\n";
    ++parens;
  }
  if (*xpath == '.')
  {
    ++xpath;
    if (*xpath == '.')
    {
      if (atroot)
      {
        fprintf(stderr, "domcpp: XPath .. at root ...->%s\n\n", xpath);
        exit(EXIT_FAILURE);
      }
      indent(ctx, k) << "if ((elt = " << v << ".parent()))\n";
      indent(ctx, k) << "{\n";
      indent(ctx, k + 2) << "xsd__anyType& v = *elt\n";
      v = "v";
      path_gen_cpp(ctx, xpath, v, false, attrib, k + 2, parens, throwup);
      indent(ctx, k) << "}\n";
    }
    else
    {
      path_gen_cpp(ctx, xpath, v, false, attrib, k, parens, throwup);
    }
  }
  else if (*xpath == '/') // path: //...
  {
    ++xpath;
    if (*xpath == '/')
    {
      if (attrib)
      {
        fprintf(stderr, "domcpp: XPath // follows attribute at ...->%s\n\n", xpath);
        exit(EXIT_FAILURE);
      }
      ++xpath;
      if (explain)
      {
        if (atroot)
          indent(ctx, k) << "// iterate over descendants and current node " << v << " to match " << putstr(xpath, 0) << "\n";
        else
          indent(ctx, k) << "// iterate over descendants of current node " << v << " to match " << putstr(xpath, 0) << "\n";
      }
      indent(ctx, k) << "for (xsd__anyType::iterator it = " << v << ".begin(); it != " << v << ".end(); ++it)\n";
      indent(ctx, k) << "{\n";
      indent(ctx, k + 2) << "xsd__anyType& v = *it;\n";
      v = "v";
      path_gen_cpp(ctx, xpath, v, atroot, false, k + 2, parens, throwup);
      indent(ctx, k) << "}\n";
    }
    else
    {
      fprintf(stderr, "domcpp: XPath unexpected end at '/'\n\n");
      exit(EXIT_FAILURE);
    }
  }
  else if (*xpath == '[') // path: [...]...
  {
    size_t nth = getnth(&xpath);
    if (nth) // path: [n]... where n is an integer > 0 position
    {
      if (explain)
        indent(ctx, k) << "// if position of current node " << v << " is " << nth << " then match " << putstr(xpath, 0) << "\n";
      indent(ctx, k) << "if (pos == " << nth << ")\n";
      indent(ctx, k) << "{\n";
      path_gen_cpp(ctx, xpath, v, false, attrib, k + 2, parens, throwup);
      indent(ctx, k) << "}\n";
    }
    else
    {
      ++xpath;
      int nest = 0;
      const char *s;
      for (s = xpath; *s; ++s)
      {
        if (*s == ']')
        {
          if (nest == 0)
            break;
          --nest;
        }
        else if (*s == '[')
          ++nest;
      }
      if (*s != ']')
      {
        fprintf(stderr, "domcpp: XPath ']' expected at ...->%s\n\n", xpath);
        exit(EXIT_FAILURE);
      }
      if (*xpath == '?') // path: [?e]... where e is a C++ bool expression
      {
        std::string code(xpath + 1, s - xpath - 1);
        xpath = s + 1;
        if (explain)
          indent(ctx, k) << "// filter current node " << v << " with 'if (" << code << ")' to match " << putstr(xpath, 0) << "\n";
        if (v != "v")
        {
          if (attrib)
            indent(ctx, k) << "xsd__anyAttribute& v = " << v << ";\n";
          else
            indent(ctx, k) << "xsd__anyType& v = " << v << ";\n";
        }
        indent(ctx, k) << "if (" << code << ")\n";
        indent(ctx, k) << "{\n";
        path_gen_cpp(ctx, xpath, v, atroot, attrib, k + 2, parens, throwup);
        indent(ctx, k) << "}\n";
      }
      else // path: [p]... where p is an XPath to match to continue
      {
        if (explain)
          indent(ctx, k) << "// check current node " << v << " matches " << putstr(xpath, 0) << "\n";
        indent(ctx, k) << "try\n";
        indent(ctx, k) << "{\n";
        std::string u = v;
        if (*xpath == '/' && *(xpath + 1) != '/')
        {
          v = "dom";
          path_gen_cpp(ctx, xpath, v, true, attrib, k + 2, 0, true);
        }
        else
        {
          path_gen_cpp(ctx, xpath, v, atroot, attrib, k + 2, 0, true);
        }
        indent(ctx, k) << "}\n";
        indent(ctx, k) << "catch (const bool&)\n";
        indent(ctx, k) << "{\n";
        xpath = s + 1;
        path_gen_cpp(ctx, xpath, u, atroot, attrib, k + 2, parens, throwup);
        indent(ctx, k) << "}\n";
      }
    }
  }
  else if (*xpath == ']')
  {
    if (!throwup)
    {
      fprintf(stderr, "domcpp: XPath ']' without '[' ...->%s\n\n", xpath);
      exit(EXIT_FAILURE);
    }
    indent(ctx, k) << "(void)" << v << ";\n";
    indent(ctx, k) << "throw true;\n";
  }
  else if (!*xpath)
  {
    if (parens)
    {
      fprintf(stderr, "domcpp: XPath ')' expected ...->%s\n\n", xpath);
      exit(EXIT_FAILURE);
    }
    path_exec_cpp(ctx, v, attrib, k);
  }
  else if (!strncmp(xpath, "text()", 6)) // path: text()
  {
    xpath += 6;
    size_t nth = getnth(&xpath);
    if (nth == 1)
    {
      if (explain)
        indent(ctx, k) << "// if current node " << v << " has a text-only element then match " << putstr(xpath, 0) << "\n";
      indent(ctx, k) << "if ((elt = " << v << ".elt_get()))\n";
    }
    else if (nth > 1)
    {
      if (explain)
        indent(ctx, k) << "// if current node " << v << " has a " << nth << "-th text-only element then match " << putstr(xpath, 0) << "\n";
      indent(ctx, k) << "if ((elt = " << v << ".elt_get()) && (elt = elt->get_nth(" << nth << ")))\n";
    }
    else
    {
      if (explain)
        indent(ctx, k) << "// for each text-only element of current node " << v << " match " << putstr(xpath, 0) << "\n";
      indent(ctx, k) << "size_t pos = 1;\n";
      indent(ctx, k) << "for (xsd__anyType *it = " << v << ".elt_get(); it; it = it->get_next(), ++pos)\n";
    }
    indent(ctx, k) << "{\n";
    if (nth)
      indent(ctx, k + 2) << "xsd__anyType& v = *elt;\n";
    else
      indent(ctx, k + 2) << "xsd__anyType& v = *it;\n";
    v = "v";
    path_gen_cpp(ctx, xpath, v, false, false, k + 2, parens, throwup);
    indent(ctx, k) << "}\n";
  }
  else if (atroot && *xpath != '@') // path: /name...
  {
    const char *tag = getname(&xpath).c_str();
    if (explain)
      indent(ctx, k) << "// if node " << v << " is '" << tag << "' then match " << putstr(xpath, 0) << "\n";
    indent(ctx, k) << "if (" << v << ".match(" << putstr(tag) << "))\n";
    indent(ctx, k) << "{\n";
    path_gen_cpp(ctx, xpath, v, false, false, k + 2, parens, throwup);
    indent(ctx, k) << "}\n";
  }
  else // path: name..., name[n]..., @name, *..., *:*..., @*..., @*:*...
  {
    const char *tag = getname(&xpath).c_str();
    size_t nth = 0;
    bool is_att = (*tag == '@');
    bool is_patt = strchr(tag, '*') != NULL;
    bool is_wild = !strcmp(tag + is_att, "*") || !strcmp(tag + is_att, "*:*");
    if (!is_patt)
      nth = getnth(&xpath);
    if (is_att && is_wild)
    {
      if (explain)
        indent(ctx, k) << "// for each attribute of current node " << v << " match " << putstr(xpath, 0) << "\n";
      indent(ctx, k) << "for (xsd__anyAttribute::iterator it = " << v << ".att_begin(); it != " << v << ".att_end(); ++it)\n";
    }
    else if (is_att && is_patt)
    {
      if (explain)
        indent(ctx, k) << "// for each attribute '" << tag + 1 << "' of current node " << v << " match " << putstr(xpath, 0) << "\n";
      indent(ctx, k) << "for (xsd__anyAttribute::iterator it = " << v << ".att_find(" << putstr(tag) << "); it != " << v << ".att_end(); ++it)\n";
    }
    else if (is_att)
    {
      if (explain)
        indent(ctx, k) << "// if current node " << v << " has an attribute '" << tag + 1 << "' then match " << putstr(xpath, 0) << "\n";
      indent(ctx, k) << "if ((att = " << v << ".att_get(" << putstr(tag + 1) << ")))\n";
    }
    else if (is_wild)
    {
      if (explain)
        indent(ctx, k) << "// for each element of current node " << v << " match " << putstr(xpath, 0) << "\n";
      indent(ctx, k) << "size_t pos = 1;\n";
      indent(ctx, k) << "for (xsd__anyType::iterator it = " << v << ".elt_begin(); it != " << v << ".elt_end(); ++it, ++pos)\n";
    }
    else if (is_patt)
    {
      if (explain)
        indent(ctx, k) << "// for each element '" << tag << "' of current node " << v << " match " << putstr(xpath, 0) << "\n";
      indent(ctx, k) << "size_t pos = 1;\n";
      indent(ctx, k) << "for (xsd__anyType::iterator it = " << v << ".elt_find(" << putstr(tag) << "); it != " << v << ".elt_end(); ++it, ++pos)\n";
    }
    else if (nth == 1)
    {
      if (explain)
        indent(ctx, k) << "// if current node " << v << " has an element '" << tag << "' then match " << putstr(xpath, 0) << "\n";
      indent(ctx, k) << "if ((elt = " << v << ".elt_get(" << putstr(tag) << ")))\n";
    }
    else if (nth > 1)
    {
      if (explain)
        indent(ctx, k) << "// if current node " << v << " has a " << nth << "-th element '" << tag << "' then match " << putstr(xpath, 0) << "\n";
      indent(ctx, k) << "if ((elt = " << v << ".elt_get(" << putstr(tag) << ")) && (elt = elt->get_nth(" << nth << ")))\n";
    }
    else
    {
      if (explain)
        indent(ctx, k) << "// for each element '" << tag << "' of current node " << v << " match " << putstr(xpath, 0) << "\n";
      indent(ctx, k) << "size_t pos = 1;\n";
      indent(ctx, k) << "for (xsd__anyType *it = " << v << ".elt_get(" << putstr(tag) << "); it; it = it->get_next(), ++pos)\n";
    }
    indent(ctx, k) << "{\n";
    if (is_att && is_patt)
      indent(ctx, k + 2) << "xsd__anyAttribute& v = *it;\n";
    else if (is_att)
      indent(ctx, k + 2) << "xsd__anyAttribute& v = *att;\n";
    else if (nth)
      indent(ctx, k + 2) << "xsd__anyType& v = *elt;\n";
    else
      indent(ctx, k + 2) << "xsd__anyType& v = *it;\n";
    v = "v";
    path_gen_cpp(ctx, xpath, v, false, is_att, k + 2, parens, throwup);
    indent(ctx, k) << "}\n";
  }
}

static void path_exec_cpp(struct soap *ctx, std::string& v, bool attrib, int k)
{
  if (xcode)
  {
    if (v != "v")
    {
      if (attrib)
        indent(ctx, k) << "xsd__anyAttribute& v = " << v << ";\n";
      else
        indent(ctx, k) << "xsd__anyType& v = " << v << ";\n";
    }
    indent(ctx, k) << xcode << "\n";
  }
  else
  {
    if (attrib)
      indent(ctx, k) << "QUERY_YIELD_ATT(" << v << ");\n";
    else
      indent(ctx, k) << "QUERY_YIELD_ELT(" << v << ");\n";
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

static std::string gen_ident(const std::string& lhs, const char *tag)
{
  std::string id = lhs;
  id.append("_");
  if (tag == NULL)
    tag = DOM_ROOT;
  for (const char *s = tag; *s; tag = s)
  {
    while (isalnum(*s))
      s++;
    id.append(tag, s - tag);
    if (*s)
      s++;
  }
  return id;
}

static bool is_bool(const char *text)
{
  if (!text)
    return false;
  return !strcmp(text, "true") || !strcmp(text, "false");
}

static bool is_int(soap *ctx, const char *text, LONG64& n)
{
  if (!text)
    return false;
  if (soap_s2LONG64(ctx, text, &n))
  {
    ctx->error = SOAP_OK;
    return false;
  }
  return true;
}

static bool is_double(soap *ctx, const char *text, double& x)
{
  if (!text)
    return false;
  if (soap_s2double(ctx, text, &x))
  {
    ctx->error = SOAP_OK;
    return false;
  }
  return !soap_isnan(x) && !soap_isinf(x);
}

static bool need_ns(const char *ns, const char *tag, const std::map<std::string,const char*>& nsmap, const char *current)
{
  if (ns == NULL || tag == NULL)
    return false;
  if (!is_qualified(tag) && is_current(ns, current))
    return false;
  if (nstable && is_qualified(tag))
  {
    std::map<std::string,const char*>::const_iterator i = nsmap.find(prefix(tag));
    if (i != nsmap.end() && !strcmp(ns, i->second))
      return false;
  }
  return true;
}

static bool is_qualified(const char *tag)
{
  return tag && strchr(tag, ':');
}

static bool is_current(const char *ns, const char *current)
{
  return ns && current && !strcmp(ns, current);
}

static std::string prefix(const char *tag)
{
  const char *s;
  if (tag == NULL)
    return std::string(DOM_ROOT);
  s = strchr(tag, ':');
  if (s)
    return std::string(tag, s - tag);
  return std::string(tag);
}

static const char *name(const char *tag)
{
  const char *s;
  if (tag == NULL)
    return DOM_ROOT;
  s = strchr(tag, ':');
  if (s)
    return s + 1;
  return tag;
}

static std::string putstrname(const char *name)
{
  std::stringstream ss;
  ss << putstr(name);
  return ss.str();
}

static std::string getname(const char **xpath)
{
  const char *s = *xpath;
  const char *t = s;
  if (*t == '@')
    ++t;
  while (*t && (isalnum(*t) || *t == '_' || *t == '*' || *t == ':' || *t == '-' || *t == '.' || (*t & 0x80)))
    ++t;
  if (t == s)
  {
    fprintf(stderr, "domcpp: XPath empty field name at ...->%s\n\n", s);
    exit(EXIT_FAILURE);
  }
  *xpath = t;
  return std::string(s, t - s);
}

static size_t getnth(const char **xpath)
{
  size_t nth = 0;
  if (**xpath == '[')
  {
    char *r;
    nth = soap_strtoul(*xpath + 1, &r, 10);
    if (*r == ']')
      *xpath = r + 1;
  }
  return nth;
}
