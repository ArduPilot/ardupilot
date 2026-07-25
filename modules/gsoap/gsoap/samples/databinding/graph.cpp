/**
	graph.cpp

	Demonstrated tree, digraph, and cyclic graph serialization.

	Copyright (C) 2000-2015 Robert A. van Engelen. All Rights Reserved.

	Project Source Files
	- graph.h	Graph (tree, digraph, cyclic graph) data binding
	- graph.cpp	Test graph serialization as tree, digraph, and cyclic

	Generated Files
	- graphH.h	Serializers
	- graphC.cpp	Serializers

	Build:
	> soapcpp2 -CS -Iimport -p graph graph.h
	> c++ -o graph graph.cpp graphC.cpp stdsoap2.cpp

	Usage:
	> ./graph
*/
/*
--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2000-2015, Robert van Engelen, Genivia, Inc. All Rights Reserved.
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

int main();

// include all gSOAP header files:
#include "graphH.h"
#include "g.nsmap"

/// Namespace mapping table for non-SOAP use
static struct Namespace nosoap_nsmap[] =
{
	{"SOAP-ENV", NULL, NULL, NULL},
	{"SOAP-ENC", NULL, NULL, NULL},
	{"xsi", "http://www.w3.org/2001/XMLSchema-instance", "http://www.w3.org/*/XMLSchema-instance", NULL},
	{"xsd", "http://www.w3.org/2001/XMLSchema", "http://www.w3.org/*/XMLSchema", NULL},
	{"g", "urn:graph", NULL, NULL},
	{NULL, NULL, NULL, NULL}
};

/// Namespace mapping table for SOAP 1.1
static struct Namespace soap11_nsmap[] =
{
	{"SOAP-ENV", "http://schemas.xmlsoap.org/soap/envelope/", "http://www.w3.org/*/soap-envelope", NULL},
	{"SOAP-ENC", "http://schemas.xmlsoap.org/soap/encoding/", "http://www.w3.org/*/soap-encoding", NULL},
	{"xsi", "http://www.w3.org/2001/XMLSchema-instance", "http://www.w3.org/*/XMLSchema-instance", NULL},
	{"xsd", "http://www.w3.org/2001/XMLSchema", "http://www.w3.org/*/XMLSchema", NULL},
	{"g", "urn:graph", NULL, NULL},
	{NULL, NULL, NULL, NULL}
};

/// Namespace mapping table for SOAP 1.2
static struct Namespace soap12_nsmap[] =
{
	{"SOAP-ENV", "http://www.w3.org/2003/05/soap-envelope", "http://schemas.xmlsoap.org/soap/envelope/", NULL},
	{"SOAP-ENC", "http://www.w3.org/2003/05/soap-encoding", "http://schemas.xmlsoap.org/soap/encoding/", NULL},
	{"xsi", "http://www.w3.org/2001/XMLSchema-instance", "http://www.w3.org/*/XMLSchema-instance", NULL},
	{"xsd", "http://www.w3.org/2001/XMLSchema", "http://www.w3.org/*/XMLSchema", NULL},
	{"g", "urn:graph", NULL, NULL},
	{NULL, NULL, NULL, NULL}
};

int main()
{
  struct soap *ctx = soap_new1(SOAP_XML_INDENT);

  soap_set_namespaces(ctx, nosoap_nsmap);

  // a tree:
  Graph tree;
  // with 5 branches:
  for (int i = 0; i < 4; ++i)
  {
    Graph *branch = soap_new_Graph(ctx);
    tree.edges.push_back(branch);
    // each branch has a couple of leaves:
    for (int j = 0; j < i; ++j)
      branch->edges.push_back(soap_new_Graph(ctx));
  }

  std::cout << "**** XML TREE FROM C++ TREE ****" << std::endl;
  soap_write_Graph(ctx, &tree);
  std::cout << std::endl << std::endl;

  std::cout << "**** XML TREE FROM C++ DIGRAPH ****" << std::endl;
  tree.edges[0] = tree.edges[1]; // first pair of edges point to shared node
  tree.edges[2] = tree.edges[3]; // second pair of edges point to shared node
  soap_write_Graph(ctx, &tree);
  std::cout << std::endl << std::endl;

  std::cout << "**** XML ID-REF DIGRAPH FROM C++ DIGRAPH ****" << std::endl;
  soap_set_omode(ctx, SOAP_XML_GRAPH);
  soap_write_Graph(ctx, &tree);
  std::cout << std::endl << std::endl;
  soap_clr_omode(ctx, SOAP_XML_GRAPH);

  std::cout << "**** XML ID-REF DIGRAPH FROM C++ CYCLIC GRAPH ****" << std::endl;
  tree.edges[0]->edges = tree.edges;   // create cycle
  soap_set_omode(ctx, SOAP_XML_GRAPH);
  soap_write_Graph(ctx, &tree);
  std::cout << std::endl << std::endl;
  soap_clr_omode(ctx, SOAP_XML_GRAPH);

  std::cout << "**** XML TREE (PRUNED CYCLIC BRANCHES) FROM C++ CYCLIC GRAPH ****" << std::endl;
  soap_set_omode(ctx, SOAP_XML_TREE);
  soap_write_Graph(ctx, &tree);
  std::cout << std::endl << std::endl;
  soap_clr_omode(ctx, SOAP_XML_TREE);

  std::cout << "**** SOAP 1.1 ENCODED GRAPH FROM C++ CYCLIC GRAPH ****" << std::endl;
  soap_set_namespaces(ctx, soap11_nsmap);
  soap_set_version(ctx, 1);        // enable SOAP 1.1
  ctx->encodingStyle = "";	   // encoded
  soap_write_Graph(ctx, &tree);
  std::cout << std::endl << std::endl;

  std::cout << "**** SOAP 1.2 ENCODED GRAPH FROM C++ CYCLIC GRAPH ****" << std::endl;
  soap_set_namespaces(ctx, soap12_nsmap);
  soap_set_version(ctx, 2);        // enable SOAP 1.2
  ctx->encodingStyle = "";	   // encoded
  soap_write_Graph(ctx, &tree);
  std::cout << std::endl << std::endl;

  soap_destroy(ctx); // delete objects
  soap_end(ctx);     // free temp data
  soap_free(ctx);    // release context

  return 0;
}

