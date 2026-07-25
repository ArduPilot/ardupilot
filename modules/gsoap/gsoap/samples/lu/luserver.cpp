/*
	luserver.h

	LU factorization Web service.

--------------------------------------------------------------------------------
gSOAP XML Web services tools
Copyright (C) 2001-2008, Robert van Engelen, Genivia, Inc. All Rights Reserved.
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

#include "soapH.h"
#include <math.h>

////////////////////////////////////////////////////////////////////////////////
//
//  main
//
////////////////////////////////////////////////////////////////////////////////

int main(int argc, char **argv)
{ struct soap *soap;
  int m, s;
  soap = soap_new();
  if (argc < 3)
    soap_serve(soap); // run as CGI application over the Web
  else // run as stand-alone server on machine given by argv[1] listening to port argv[2]
  { m = soap_bind(soap, argv[1], atoi(argv[2]), 100);
    if (m < 0)
    { soap_print_fault(soap, stderr);
      exit(-1);
    }
    fprintf(stderr, "Socket connection successful: master socket = %d\n", m);
    for (int i = 1; ; i++)
    { s = soap_accept(soap);
      if (s < 0)
      { soap_print_fault(soap, stderr);
        exit(-1);
      }
      fprintf(stderr, "%d: accepted connection from IP = %d.%d.%d.%d socket = %d ... ", i, (int)(soap->ip>>24)&0xFF, (int)(soap->ip>>16)&0xFF, (int)(soap->ip>>8)&0xFF, (int)soap->ip&0xFF, s);
      soap_serve(soap);		// process request
      fprintf(stderr, "request served\n");
      soap_destroy(soap);	// delete class instances
      soap_end(soap);		// clean up everything and close socket
    }
  }
  soap_done(soap);
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
//
//  LU decomposition: remote method interface
//
////////////////////////////////////////////////////////////////////////////////

int ludcmp(struct soap*, matrix&, ivector&, double&);

int ns1__ludcmp(struct soap *soap, matrix *a, struct ns1__ludcmpResponse &result)
{ result.a = a;
  result.i = soap_new_ivector(soap, -1);
  if (ludcmp(soap, *result.a, *result.i, result.d))
    return soap_sender_fault(soap, "Singular matrix in routine ludcmp", NULL);
  return SOAP_OK;
}

////////////////////////////////////////////////////////////////////////////////
//
//  LU decomposition: algorithm
//
////////////////////////////////////////////////////////////////////////////////

int ludcmp(struct soap *soap, matrix &a, ivector &indx, double &d)
{ int i, imax = 0, j, k, n;
  double big, dum, sum, temp;
  n = a.size();
  vector vv(soap);
  vv.resize(n);
  indx.resize(n);
  d = 1.0;
  for (i = 1; i <= n; i++)
  { big = 0.0;
    a[i].resize(n);
    for (j = 1; j <= n; j++)
      if ((temp = fabs(a[i][j])) > big)
        big = temp;
    if (big == 0.0)
      return -1;
    vv[i] = 1.0/big;
  }
  for (j = 1; j <= n; j++)
  { for (i = 1; i < j; i++)
    { sum = a[i][j];
      for (k = 1; k < i; k++)
        sum -= a[i][k]*a[k][j];
      a[i][j] = sum;
    }
    big = 0.0;
    for (i = j; i <= n; i++)
    { sum = a[i][j];
      for (k = 1; k < j; k++)
        sum -= a[i][k]*a[k][j];
      a[i][j] = sum;
      if ((dum = vv[i]*fabs(sum)) >= big)
      { big = dum;
        imax = i;
      }
    }
    if (j != imax)
    { for (k = 1; k <= n; k++)
      { dum = a[imax][k];
        a[imax][k] = a[j][k];
        a[j][k] = dum;
      }
      d = -d;
      vv[imax] = vv[j];
    }
    indx[j] = imax;
    if (a[j][j] == 0.0)
      a[j][j] = 1.0e-20;
    if (j != n)
    { dum = 1.0/a[j][j];
      for (i = j+1; i <= n; i++)
        a[i][j] *= dum;
    }
  }
  for (i = 1; i <= n; i++)
  { for (j = 1; j <= n; j++)
      if (fabs(a[i][j]) > 1.0e-15)
        break;
    for (k = n; k > j; k--)
      if (fabs(a[i][k]) > 1.0e-15)
        break;
    a[i].resize(j, k);
  }
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
//
//  Forward- and backsubstitution: remote method interface
//
////////////////////////////////////////////////////////////////////////////////

int lubksb(matrix&, ivector&, vector &b);

int ns1__lubksb(struct soap *soap, matrix *a, ivector *i, vector *b, vector *x)
{ lubksb(*a, *i, *b);
  *x = *b;
  return SOAP_OK;
}

////////////////////////////////////////////////////////////////////////////////
//
//  Forward- and backsubstitution: algorithm
//
////////////////////////////////////////////////////////////////////////////////

int lubksb(matrix &a, ivector &indx, vector &b)
{ int i, j, k, ip, n, m, ii = 0;
  double sum;
  n = a.size();
  b.resize(n);
  for (i = 1; i <= n; i++)
  { ip = indx[i];
    sum = b[ip];
    b[ip] = b[i];
    if (ii)
    { k = a[i].start();
      if (ii > k)
        k = ii;
      m = a[i].end();
      if (i-1 < m)
        m = i-1;
      for (j = k; j <= m; j++)
        sum -= a[i][j]*b[j];
    }
    else if (sum)
      ii = i;
    b[i] = sum;
  }
  for (i = n; i >= 1; i--)
  { sum = b[i];
    k = a[i].start();
    if (i+1 > k)
      k = i+1;
    m = a[i].end();
    if (n < m)
      m = n;
    for (j = k; j <= m; j++)
      sum -= a[i][j]*b[j];
    b[i] = sum/a[i][i];
  }
  return SOAP_OK;
}

////////////////////////////////////////////////////////////////////////////////
//
//  LU solve: remote method interface
//
////////////////////////////////////////////////////////////////////////////////

int ns1__lusol(struct soap *soap, matrix *a, vector *b, vector *x)
{ ivector indx(soap);
  double d;
  if (ludcmp(soap, *a, indx, d))
    return soap_sender_fault(soap, "Singular matrix in routine ludcmp", NULL);
  lubksb(*a, indx, *b);
  *x = *b;
  return SOAP_OK;
}

////////////////////////////////////////////////////////////////////////////////
//
//  LU solve multiple: remote method interface
//
////////////////////////////////////////////////////////////////////////////////

int ns1__lusols(struct soap *soap, matrix *a, matrix *b, matrix *x)
{ ivector indx(soap);
  double d;
  if (ludcmp(soap, *a, indx, d))
    return soap_sender_fault(soap, "Singular matrix in routine ludcmp", NULL);
  for (int i = 1; i <= b->size(); i++)
    lubksb(*a, indx, (*b)[i]);
  *x = *b;
  return SOAP_OK;
}

////////////////////////////////////////////////////////////////////////////////
//
//  LU inverse: remote method interface
//
////////////////////////////////////////////////////////////////////////////////

int ns1__luinv(struct soap *soap, matrix *a, matrix *b)
{ vector col(soap);
  ivector indx(soap);
  double d;
  int i, j, k, n;
  if (ludcmp(soap, *a, indx, d))
    return soap_sender_fault(soap, "Singular matrix in routine ludcmp", NULL);
  n = a->size();
  col.resize(n);
  b->resize(n, n);
  for (j = 1; j <= n; j++)
  { for (i = 1; i <= n; i++)
      col[i] = 0.0;
    col[j] = 1.0;
    lubksb(*a, indx, col);
    for (i = 1; i <= n; i++)
      (*b)[i][j] = col[i];
  }
  for (i = 1; i <= n; i++)
  { for (j = 1; j <= n; j++)
      if (fabs((*b)[i][j]) > 1.0e-15)
        break;
    for (k = n; k > j; k--)
      if (fabs((*b)[i][k]) > 1.0e-15)
        break;
    (*b)[i].resize(j, k);
  }
  return SOAP_OK;
}

////////////////////////////////////////////////////////////////////////////////
//
//  LU determinant: remote method interface
//
////////////////////////////////////////////////////////////////////////////////

int ns1__ludet(struct soap *soap, matrix *a, double &d)
{ ivector indx(soap);
  if (ludcmp(soap, *a, indx, d))
    return soap_sender_fault(soap, "Singular matrix in routine ludcmp", NULL);
  for (int i = 1; i <= a->__size; i++)
    d *= (*a)[i][i];
  return SOAP_OK;
}

struct Namespace namespaces[] =
{// "ns-prefix", "ns-name", "ns-pattern"
  {"SOAP-ENV", "http://schemas.xmlsoap.org/soap/envelope/"},
  {"SOAP-ENC", "http://schemas.xmlsoap.org/soap/encoding/"},
  {"xsi", "http://www.w3.org/2001/XMLSchema-instance", "http://wwww.w3.org/*/XMLSchema-instance"},
  {"xsd", "http://www.w3.org/2001/XMLSchema", "http://www.w3.org/*/XMLSchema"},
  {"ns1", "urn:lu"},
  {NULL, NULL}
};
