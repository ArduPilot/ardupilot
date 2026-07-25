/*
        wsdl2h.cpp

        WSDL/WADL/XSD parser and translator to the gSOAP header file format

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2000-2021, Robert van Engelen, Genivia Inc. All Rights Reserved.
This software is released under one of the following licenses:
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

#include "includes.h"
#include "types.h"
#include "service.h"

#ifndef WSDL2H_IMPORT_PATH
#define WSDL2H_IMPORT_PATH (NULL)
#endif

#ifndef WSDL_TYPEMAP_FILE
#define WSDL_TYPEMAP_FILE "typemap.dat"
#endif

static void init();
static void options(int argc, char **argv);

int _flag = 0,
    aflag = 0,
    bflag = 0,
    cflag = 0,
    c11flag = 0,
    Dflag = 0,
    dflag = 0,
    eflag = 0,
    Fflag = 0,
    fflag = 0,
    gflag = 0,
    iflag = 0,
    jflag = 0,
    kflag = 0,
    Lflag = 0,
    Mflag = 0,
    mflag = 0,
    Oflag = 0,
    Owflag = 0,
    Pflag = 0,
    pflag = 0,
    Qflag = 0,
    Rflag = 0,
    sflag = 0,
    Uflag = 0,
    uflag = 0,
    vflag = 0,
    Wflag = 0,
    wflag = 0,
    Xflag = 0,
    xflag = 0,
    yflag = 0,
    zflag = 0;

int openfiles = 0;
int infiles = 0;
char *infile[MAXINFILES],
     *outfile = NULL,
     *proxy_host = NULL,
     *proxy_userid = NULL,
     *proxy_passwd = NULL,
     *auth_userid = NULL,
     *auth_passwd = NULL;
const char
     *mapfile = WSDL_TYPEMAP_FILE,
     *import_path = WSDL2H_IMPORT_PATH,
     *cwd_path = NULL,
     *cppnamespace = NULL;

int proxy_port = 8080;

FILE *stream = stdout;

SetOfString exturis;

extern struct Namespace namespaces[];

const char *service_prefix = NULL;
const char *schema_prefix = "ns";
const char *soap_context = "soap";

const char elementformat[]             = "    %-35s  %-30s";
const char pointerformat[]             = "    %-35s *%-30s";
const char attributeformat[]           = "  @ %-35s  %-30s";
const char templateformat[]            = "    %s<%-23s> %-30s";
const char pointertemplateformat[]     = "    %s<%-22s> *%-30s";
const char templateformat_open[]       = "    %s<%s";
const char attrtemplateformat_open[]   = "  @ %s<%s";
const char arrayformat[]               = "    %-35s *__ptr%-25s";
const char arraysizeformat[]           = "    %-35s  __size%-24s";
const char arrayoffsetformat[]         = "//  %-35s  __offset%-22s";
const char sizeformat[]                = "  $ %-35s  __size%-24s";
const char choiceformat[]              = "  $ %-35s  __union%-23s";
const char schemaformat[]              = "//gsoap %-5s schema %s:\t%s\n";
const char serviceformat[]             = "//gsoap %-4s service %s:\t%s %s\n";
const char paraformat[]                = "    %-35s%s%s%s";
const char anonformat[]                = "    %-35s%s_%s%s";
const char sizeparaformat[]            = "  $ %-35s __size%s%s";
const char pointertemplateparaformat[] = "    %s<%-21s>*%s%s%s";
const char derivedformat[]             = "  [ %-35s *%-30s; ]";

const char copyrightnotice[] = "\n**  The gSOAP WSDL/WADL/XSD processor for C and C++, wsdl2h release " WSDL2H_VERSION "\n**  Copyright (C) 2000-2021 Robert van Engelen, Genivia Inc.\n**  All Rights Reserved. This product is provided \"as is\", without any warranty.\n**  The wsdl2h tool and its generated software are released under the GPL.\n**  ----------------------------------------------------------------------------\n**  A commercial use license is available from Genivia Inc., contact@genivia.com\n**  ----------------------------------------------------------------------------\n\n";

const char licensenotice[]   = "\
--------------------------------------------------------------------------------\n\
gSOAP XML Web services tools\n\
Copyright (C) 2000-2021, Robert van Engelen, Genivia Inc. All Rights Reserved.\n\
The wsdl2h tool and its generated software are released under the GPL.\n\
This software is released under the GPL with the additional exemption that\n\
compiling, linking, and/or using OpenSSL is allowed.\n\
--------------------------------------------------------------------------------\n\
GPL license.\n\
\n\
This program is free software; you can redistribute it and/or modify it under\n\
the terms of the GNU General Public License as published by the Free Software\n\
Foundation; either version 2 of the License, or (at your option) any later\n\
version.\n\
\n\
This program is distributed in the hope that it will be useful, but WITHOUT ANY\n\
WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR A\n\
PARTICULAR PURPOSE. See the GNU General Public License for more details.\n\
\n\
You should have received a copy of the GNU General Public License along with\n\
this program; if not, write to the Free Software Foundation, Inc., 59 Temple\n\
Place, Suite 330, Boston, MA 02111-1307 USA\n\
\n\
Author contact information:\n\
engelen@genivia.com / engelen@acm.org\n\
\n\
This program is released under the GPL with the additional exemption that\n\
compiling, linking, and/or using OpenSSL is allowed.\n\
--------------------------------------------------------------------------------\n\
A commercial-use license is available from Genivia, Inc., contact@genivia.com\n\
--------------------------------------------------------------------------------\n";

int main(int argc, char **argv)
{
  options(argc, argv);
  fprintf(stderr, "%s", copyrightnotice);
  if (!infiles)
    fprintf(stderr, "Reading from stdin...\n");
  init();
  if (cppnamespace)
    fprintf(stream, "namespace %s {\n", cppnamespace);
  Definitions def;
  wsdl__definitions definitions;
  definitions.read(infiles, infile);
  if (definitions.error())
  {
    definitions.print_fault();
    exit(1);
  }
  definitions.traverse();
  if (Oflag > 1)
    definitions.mark();
  def.compile(definitions);
  if (Oflag > 1)
  {
    if (def.types.omitted)
    {
      fprintf(stderr, "\nOptimization (-O%s%d) removed %zu definitions of unused schema components (%.1f%%)\n", Owflag ? "w" : "", Oflag, def.types.omitted, 100.0*def.types.omitted/def.types.total);
      if (pflag && !Owflag)
        fprintf(stderr, "\nWarning: option -O%d removes type definitions that may be used as derived types of base types in XML (which are indicated by xsi:type attributes).  Use option -Ow%d instead of -O%d to retain all derived types of base types\n", Oflag, Oflag, Oflag);
      else if (Fflag && !Owflag)
        fprintf(stderr, "\nWarning: option -O%d removes type definitions that may be used as derived types of base types in XML (which are indicated by xsi:type attributes).  Use option -Ow%d instead of -O%d to retain all derived types of base types\n", Oflag, Oflag, Oflag);
    }
    else
    {
      fprintf(stderr, "\nOptimization (-O%s%d) found no unused schema components to remove\n", Owflag ? "w" : "", Oflag);
    }
  }
  if (outfile)
  {
    fclose(stream);
    fprintf(stderr, "\nTo finalize code generation, execute:\n> soapcpp2 %s\n", outfile);
    if (!cflag)
      fprintf(stderr, "Or to generate C++ proxy and service classes:\n> soapcpp2 -j %s\n", outfile);
    fprintf(stderr, "\n");
  }
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
//
//      Initialization
//
////////////////////////////////////////////////////////////////////////////////

static void init()
{
  struct Namespace *p = namespaces;
  if (p)
  {
    for (; p->id; p++)
    {
      if (p->in && *p->in)
        exturis.insert(p->in);
      if (p->ns && *p->ns)
        exturis.insert(p->ns);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
//
//      Parse command line options
//
////////////////////////////////////////////////////////////////////////////////

static void options(int argc, char **argv)
{
  int i;
  infiles = 0;
  openfiles = 0;
  for (i = 1; i < argc; i++)
  {
    char *a = argv[i];
    if (*a == '-'
#ifdef WIN32
     || *a == '/'
#endif
    )
    {
      int g = 1;
      while (g && *++a)
      {
        switch (*a)
        {
          case '_':
            _flag = 1;
            break;
          case 'a':
            aflag = 1;
            break;
          case 'b':
            bflag = 1;
            break;
          case 'c':
            if (a[1] == '+' && a[2] == '+')
            {
              a += 2;
              if (a[1] == '1' && a[2] == '1')
              {
                a += 2;
                c11flag = 1;
              }
              cflag = 0;
            }
            else
            {
              cflag = 1;
              if (cppnamespace)
                fprintf(stderr, "wsdl2h: Options -c and -q cannot be used together\n");
            }
            break;
          case 'D':
            Dflag = 1;
            break;
          case 'd':
            dflag = 1;
            break;
          case 'e':
            eflag = 1;
            break;
          case 'F':
            Fflag = 1;
            break;
          case 'f':
            fflag = 1;
            break;
          case 'g':
            gflag = 1;
            break;
          case 'i':
            iflag = 1;
            break;
          case 'j':
            jflag = 1;
            break;
          case 'k':
            kflag = 1;
            break;
          case 'I':
            a++;
            g = 0;
            if (*a)
              import_path = a;
            else if (i < argc && argv[++i])
              import_path = argv[i];
            else
              fprintf(stderr, "wsdl2h: Option -I requires a path argument\n");
            break;
          case 'L':
            Lflag = 1;
            break;
          case 'l':
            fprintf(stderr, "%s", licensenotice);
            exit(0);
            break;
          case 'm':
            mflag = 1;
            break;
          case 'M':
            Mflag = 1;
            break;
          case 'n':
            a++;
            g = 0;
            if (*a)
              schema_prefix = a;
            else if (i < argc && argv[++i])
              schema_prefix = argv[i];
            else
              fprintf(stderr, "wsdl2h: Option -n requires a prefix name argument\n");
            break;
          case 'N':
            a++;
            g = 0;
            if (*a)
              service_prefix = a;
            else if (i < argc && argv[++i])
              service_prefix = argv[i];
            else
              fprintf(stderr, "wsdl2h: Option -N requires a prefix name argument\n");
            break;
          case 'o':
            a++;
            if (outfile)
              fprintf(stderr, "wsdl2h: Option -o specified twice\n");
            g = 0;
            if (*a)
              outfile = a;
            else if (i < argc && argv[++i])
              outfile = argv[i];
            else
              fprintf(stderr, "wsdl2h: Option -o requires an output file argument\n");
            break;
          case 'O':
            a++;
            if (*a == 'w')
            {
              Owflag = 1;
              a++;
            }
            if (Oflag)
              fprintf(stderr, "wsdl2h: Option -O specified twice\n");
            g = 0;
            if (*a)
              Oflag = soap_strtol(a, NULL, 10);
            else if (i < argc && argv[++i])
              Oflag = soap_strtol(argv[i], NULL, 10);
            else
              fprintf(stderr, "wsdl2h: Option -O requires an argument number\n");
            break;
          case 'p':
            pflag = 1;
            break;
          case 'P':
            Pflag = 1;
            break;
          case 'Q':
            Qflag = 1;
            break;
          case 'q':
            a++;
            g = 0;
            if (*a)
              cppnamespace = a;
            else if (i < argc && argv[++i])
              cppnamespace = argv[i];
            else
              fprintf(stderr, "wsdl2h: Option -q requires a C++ namespace name argument\n");
            if (cflag)
              fprintf(stderr, "wsdl2h: Options -c and -q cannot be used together\n");
            break;
          case 'r':
            a++;
            g = 0;
            if (*a)
              proxy_host = a;
            else if (i < argc && argv[++i])
              proxy_host = argv[i];
            else
              fprintf(stderr, "wsdl2h: Option -r requires proxy host:port:userid:passwd or :userid:passwd authentication argument\n");
            if (proxy_host)
            {
              size_t l = strlen(proxy_host);
              char *s = (char*)emalloc(l + 1);
              soap_strcpy(s, l + 2, proxy_host);
              proxy_host = s;
              s = strchr(proxy_host, ':');
              if (s)
              {
                *s = '\0';
                if (*proxy_host)
                {
                  proxy_port = soap_strtol(s + 1, &s, 10);
                  if (s && *s == ':')
                  {
                    *s = '\0';
                    proxy_userid = s + 1;
                    s = strchr(proxy_userid, ':');
                    if (s && *s == ':')
                    {
                      *s = '\0';
                      proxy_passwd = s + 1;
                    }
                  }
                }
                else
                {
                  s = proxy_host;
                  proxy_host = NULL;
                  auth_userid = s + 1;
                  s = strchr(auth_userid, ':');
                  if (s && *s == ':')
                  {
                    *s = '\0';
                    auth_passwd = s + 1;
                  }
                }
              }
            }
            break;
          case 'R':
            Rflag = 1;
            break;
          case 's':
            sflag = 1;
            break;
          case 'S':
            a++;
            g = 0;
            if (*a)
              soap_context = a;
            else if (i < argc && argv[++i])
              soap_context = argv[i];
            else
              fprintf(stderr, "wsdl2h: Option -S requires a name argument\n");
            break;
          case 't':
            a++;
            g = 0;
            if (*a)
              mapfile = a;
            else if (i < argc && argv[++i])
              mapfile = argv[i];
            else
              fprintf(stderr, "wsdl2h: Option -t requires a type map file argument\n");
            break;
          case 'U':
            Uflag = 1;
            break;
          case 'u':
            uflag = 1;
            break;
          case 'V':
            printf("%s", WSDL2H_VERSION);
            exit(0);
          case 'v':
            vflag = 1;
            break;
          case 'w':
            wflag = 1;
            break;
          case 'W':
            Wflag = 1;
            break;
          case 'X':
            Xflag = 1;
            break;
          case 'x':
            xflag = 1;
            break;
          case 'y':
            yflag = 1;
            break;
          case 'z':
          {
            int z = zflag;
            a++;
            if (zflag)
              fprintf(stderr, "wsdl2h: Option -z specified twice\n");
            g = 0;
            if (*a)
              z = soap_strtol(a, NULL, 10);
            else if (i < argc && argv[++i])
              z = soap_strtol(argv[i], NULL, 10);
            else
              fprintf(stderr, "wsdl2h: Option -z requires an argument\n");
            if (zflag == 0 || z < zflag)
              zflag = z;
            break;
          }
          case '?':
          case 'h':
            fprintf(stderr, "Usage: wsdl2h [-a] [-b] [-c|-c++|-c++11] [-D] [-d] [-e] [-F] [-f] [-g] [-h] [-I path] [-i] [-j] [-k] [-L] [-l] [-M] [-m] [-N name] [-n name] [-O1|-O2|-O3|-O4|-Ow2|-Ow3|-Ow4] [-P|-p] [-Q] [-q name] [-R] [-r proxyhost[:port[:uid:pwd]]] [-r:uid:pwd] [-Sname] [-s] [-T] [-t typemapfile] [-U] [-u] [-V] [-v] [-w] [-W] [-x] [-y] [-z#] [-_] [-o outfile.h] infile.wsdl infile.xsd http://www... ...\n\n");
            fprintf(stderr, "\
-a      generate indexed struct names for local elements with anonymous types\n\
-b      bi-directional operations (duplex ops) added to serve one-way responses\n\
-c      generate C source code\n\
-c++    generate C++ source code (default)\n\
-c++11  generate C++11 source code\n\
-D      make attribute members with default/fixed values optional with pointers\n\
-d      use DOM to populate xs:any, xs:anyType, and xs:anyAttribute\n\
-e      don't qualify enum names\n\
-F      add transient members to structs to simulate struct-type derivation in C\n\
-f      generate flat C++ class hierarchy by removing inheritance\n\
-g      generate global top-level element and attribute declarations\n\
-h      display help info and exit\n\
-Ipath  use path to locate WSDL and XSD files\n\
-i      don't import (advanced option)\n\
-j      don't generate SOAP_ENV__Header and SOAP_ENV__Detail definitions\n\
-k      don't generate SOAP_ENV__Header mustUnderstand qualifiers\n\
-L      generate less documentation by removing generic @note comments\n\
-l      display license information\n\
-M      suppress error \"must understand element with wsdl:required='true'\"\n\
-m      use xsd.h module to import primitive types\n\
-Nname  use name for service prefixes to produce a service for each binding\n\
-nname  use name as the base namespace prefix instead of 'ns'\n\
-O1     optimize by omitting duplicate choice/sequence members\n\
-O2     optimize -O1 and omit unused schema types (unreachable from roots)\n\
-O3     optimize -O2 and omit unused schema root attributes\n\
-O4     optimize -O3 and omit unused schema root elements (use only with WSDLs)\n\
-Ow2    optimize -O2 while retaining all derived types of used base types\n\
-Ow3    optimize -O3 while retaining all derived types of used base types\n\
-Ow4    optimize -O4 while retaining all derived types of used base types\n\
-ofile  output to file\n\
-P      don't create polymorphic types inherited from xsd__anyType\n\
-p      create polymorphic types inherited from base xsd__anyType\n\
-Q      make xsd__anySimpleType equal to xsd__anyType to use as the base type\n\
-qname  use name for the C++ namespace of all declarations\n\
-R      generate REST operations for REST bindings specified in a WSDL\n\
-rhost[:port[:uid:pwd]]\n\
        connect via proxy host, port, and proxy credentials uid and pwd\n\
-r:uid:pwd\n\
        connect with authentication credentials uid and pwd\n\
-Sname  use name instead of 'soap' for the C++ class members with soap contexts\n\
-s      don't generate STL code (no std::string and no std::vector)\n\
-tfile  use type map file instead of the default file typemap.dat\n\
-U      allow UTF-8-encoded Unicode C/C++ identifiers when mapping XML tag names\n\
-u      don't generate unions\n\
-V      display the current version and exit\n\
-v      verbose output\n\
-W      suppress warnings\n\
-w      always wrap response parameters in a response struct (<=1.1.4 behavior)\n\
-X      don't qualify part names to disambiguate doc/lit wrapped patterns\n\
-x      don't generate _XML any/anyAttribute extensibility elements\n\
-y      generate typedef synonyms for structs and enums\n\
-z1     compatibility with 2.7.6e: generate pointer-based arrays\n\
-z2     compatibility with 2.7.7-2.7.15: (un)qualify element/attribute refs\n\
-z3     compatibility with 2.7.16-2.8.7: (un)qualify element/attribute refs\n\
-z4     compatibility up to 2.8.11: don't generate union structs in std::vector\n\
-z5     compatibility up to 2.8.15: don't include minor improvements\n\
-z6     compatibility up to 2.8.17: don't include minor improvements\n\
-z7     compatibility up to 2.8.59: don't generate std::vector of class of union\n\
-z8     compatibility up to 2.8.74: don't gen quals for doc/lit wrapped patterns\n\
-z9     compatibility up to 2.8.93: always qualify element/attribute refs\n\
-z10    compatibility up to 2.8.96: gen quals even when defined w/o namespace\n\
-_      don't generate _USCORE (replace with Unicode code point _x005f)\n\
infile.wsdl infile.xsd http://www... list of input sources (if none reads stdin)\n\
\n");
            exit(0);
          default:
            fprintf(stderr, "wsdl2h: Unknown option %s\n", a);
            exit(1);
        }
      }
    }
    else
    {
      infile[infiles++] = argv[i];
      if (infiles >= MAXINFILES)
      {
        fprintf(stderr, "wsdl2h: too many files\n");
        exit(1);
      }
    }
  }
  if (infiles)
  {
    if (!outfile)
    {
      if (strncmp(infile[0], "http://", 7) && strncmp(infile[0], "https://", 8))
      {
        const char *s = strrchr(infile[0], '.');
        if (s && (!soap_tag_cmp(s, ".wsdl") || !soap_tag_cmp(s, ".wadl") || !soap_tag_cmp(s, ".gwsdl") || !soap_tag_cmp(s, ".xsd")))
        {
          outfile = estrdup(infile[0]);
          outfile[s - infile[0] + 1] = 'h';
          outfile[s - infile[0] + 2] = '\0';
        }
        else
        {
          size_t l = strlen(infile[0]);
          outfile = (char*)emalloc(l + 3);
          soap_strcpy(outfile, l + 3, infile[0]);
          soap_strcpy(outfile + l, 3, ".h");
        }
      }
    }
    if (outfile)
    {
      stream = fopen(outfile, "w");
      if (!stream)
      {
        fprintf(stderr, "Cannot write to %s\n", outfile);
        exit(1);
      }
      fprintf(stderr, "Saving %s\n\n", outfile);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
//
//      Namespaces
//
////////////////////////////////////////////////////////////////////////////////

struct Namespace namespaces[] =
{
  {"SOAP-ENV", "http://schemas.xmlsoap.org/soap/envelope/", "http://www.w3.org/*/soap-envelope", NULL},
  {"SOAP-ENC", "http://schemas.xmlsoap.org/soap/encoding/", "http://www.w3.org/*/soap-encoding", NULL},
  {"xsi", "http://www.w3.org/2001/XMLSchema-instance", NULL, NULL},
  {"xsd", "-", NULL, NULL}, // http://www.w3.org/2001/XMLSchema"}, // don't use this, it might conflict with xs
  {"xml", "http://www.w3.org/XML/1998/namespace", NULL, NULL},
  {"xs", "http://www.w3.org/2001/XMLSchema", "http://www.w3.org/*/XMLSchema", NULL},
  {"vc", "http://www.w3.org/2007/XMLSchema-versioning", NULL, NULL},
  {"http", "http://schemas.xmlsoap.org/wsdl/http/", NULL, NULL},
  {"soap", "http://schemas.xmlsoap.org/wsdl/soap/", "http://schemas.xmlsoap.org/wsdl/soap*/", NULL},
  {"mime", "http://schemas.xmlsoap.org/wsdl/mime/", NULL, NULL},
  {"xmime", "http://www.w3.org/2005/05/xmlmime", NULL, NULL},
  {"dime", "http://schemas.xmlsoap.org/ws/2002/04/dime/wsdl/", "http://schemas.xmlsoap.org/ws/*/dime/wsdl/", NULL},
  {"wadl", "http://wadl.dev.java.net/2009/02", NULL, NULL},
  {"wsdl", "http://schemas.xmlsoap.org/wsdl/", "http://www.w3.org/ns/wsdl", NULL},
  {"wsdli", "http://www.w3.org/ns/wsdl-instance", NULL, NULL},
  {"wsdlx", "http://www.w3.org/ns/wsdl-extensions", NULL, NULL},
  {"wsoap", "http://www.w3.org/ns/wsdl/soap", NULL, NULL},
  {"whttp", "http://www.w3.org/ns/wsdl/http", NULL, NULL},
  {"wrpc", "http://www.w3.org/ns/wsdl/rpc", NULL, NULL},
  {"wsaw", "http://www.w3.org/2006/05/addressing/wsdl", NULL, NULL},
  {"gwsdl", "http://www.gridforum.org/namespaces/2003/03/gridWSDLExtensions", NULL, NULL},
  {"wsa_", "http://www.w3.org/2005/08/addressing", NULL, NULL},
  {"wsam", "http://www.w3.org/2007/05/addressing/metadata", NULL, NULL},
  {"wsrmp", "http://docs.oasis-open.org/ws-rx/wsrm/200702", "http://docs.oasis-open.org/ws-rx/wsrm/*", NULL},
  {"wsrmp5", "http://schemas.xmlsoap.org/ws/2005/02/rm/policy", NULL, NULL},
  {"sp", "http://docs.oasis-open.org/ws-sx/ws-securitypolicy/200702", "http://schemas.xmlsoap.org/ws/2005/07/securitypolicy", NULL},
  {"wsp_", "http://www.w3.org/ns/ws-policy", "http://schemas.xmlsoap.org/ws/2004/09/policy", NULL},
  {"wst_", "http://docs.oasis-open.org/ws-sx/ws-trust/200512", NULL, NULL},
  {"wsu_", "http://docs.oasis-open.org/wss/2004/01/oasis-200401-wss-wssecurity-utility-1.0.xsd", NULL, NULL},
  {"plnk","http://docs.oasis-open.org/wsbpel/2.0/plnktype", NULL, NULL},
  {"vprop","http://docs.oasis-open.org/wsbpel/2.0/varprop", NULL, NULL},
#ifdef WITH_BPEL
  {"bpel","http://docs.oasis-open.org/wsbpel/2.0/process/executable", NULL, NULL},
  {"sref","http://docs.oasis-open.org/wsbpel/2.0/serviceref", NULL, NULL},
#endif
  {NULL, NULL, NULL, NULL}
};
