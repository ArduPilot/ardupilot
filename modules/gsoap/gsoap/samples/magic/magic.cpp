/*
	magic.cpp

	Magic squares client.

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
#include "magic.nsmap"

////////////////////////////////////////////////////////////////////////////////
//
//	Magic Squares Client
//
////////////////////////////////////////////////////////////////////////////////

// To access a stand-alone server on a port: magicserver[] = "IP:PORT";
// use "http://" to include HTTP header, e.g: magicserver[] = "http://IP:PORT";
const char magicserver[] = "http://www.cs.fsu.edu/~engelen/magicserver.cgi";
// const char magicserver[] = "http://localhost:8080";
// To send request to stdout and read response from stdin, use:
// const char magicserver[] = "http://";

int main(int argc, char **argv)
{ struct soap soap;
  int r;
  soap_init(&soap);
  matrix *A = soap_new_matrix(&soap, -1);
  if (argc <= 1)
  { char *s = getenv("QUERY_STRING");
    if (!s || (r = atoi(s)) == 0)
      r = 7;
  }
  else
    r = atoi(argv[1]);
  printf("Content-type: text/html\r\n\r\n<html><h1>Magic Square of Rank %d</h1><pre>\n", r);
  if (soap_call_ns1__magic(&soap, magicserver, NULL, r, A))
  { soap_print_fault(&soap, stderr);
    soap_print_fault_location(&soap, stderr);
  }
  else
  { for (int i = 0; i < (*A).__size; i++)
    { int sum = 0;
      for (int j = 0; j < (*A)[i].__size; j++)
      { sum += (*A)[i][j];
        printf("%4d", (*A)[i][j]);
      }
      printf(" =%4d\n", sum);
    }
    for (int j = 0; j < (*A)[0].__size; j++)
      printf("  ||");
    printf("\n");
    for (int j = 0; j < (*A)[0].__size; j++)
    { int sum = 0;
      for (int i = 0; i < (*A).__size; i++)
	sum += (*A)[i][j];
      printf("%4d", sum);
    }
    printf("\n");
    if (r % 2 == 0)
      printf("<br/><b>Magic only for odd ranks!</b>\n");
  }
  printf("</pre></html>\n");
  soap_destroy(&soap);
  soap_end(&soap);
  soap_done(&soap);
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
//
//	Class vector Methods
//
////////////////////////////////////////////////////////////////////////////////

vector::vector()
{ __ptr = 0;
  __size = 0;
}

vector::vector(int size)
{ __ptr = (int*)soap_malloc(soap, size*sizeof(int));
  __size = size;
}

vector::~vector()
{ soap_unlink(soap, this);
}

void vector::resize(int size)
{ int *p;
  if (__size == size)
    return;
  p = (int*)soap_malloc(soap, size*sizeof(int));
  if (__ptr)
  { for (int i = 0; i < (size <= __size ? size : __size); i++)
      p[i] = __ptr[i];
    soap_unlink(soap, __ptr);
    free(__ptr);
  }
  __ptr = p;
  __size = size;
}

int& vector::operator[](int idx) const
{ if (!__ptr || idx < 0 || idx >= __size)
    fprintf(stderr, "Array index out of bounds\n");
  return __ptr[idx];
}

////////////////////////////////////////////////////////////////////////////////
//
//	Class matrix Methods
//
////////////////////////////////////////////////////////////////////////////////

matrix::matrix()
{ __ptr = 0;
  __size = 0;
}

matrix::~matrix()
{ soap_unlink(soap, this); // not required, but just to make sure if someone calls delete on this
}

matrix::matrix(int rows, int cols)
{ 
  __ptr = soap_new_vector(soap, rows);
  for (int i = 0; i < cols; i++)
    __ptr[i].resize(cols);
  __size = rows;
}

void matrix::resize(int rows, int cols)
{ int i;
  vector *p;
  if (__size != rows)
  { if (__ptr)
    { p = soap_new_vector(soap, rows);
      for (i = 0; i < (rows <= __size ? rows : __size); i++)
      { if (this[i].__size != cols)
          (*this)[i].resize(cols);
	(p+i)->__ptr = __ptr[i].__ptr;
	(p+i)->__size = cols;
      }
      for (; i < rows; i++)
        __ptr[i].resize(cols);
    }
    else
    { 
      __ptr = soap_new_vector(soap, rows);
      for (i = 0; i < rows; i++)
        __ptr[i].resize(cols);
      __size = rows;
    }
  }
  else
    for (i = 0; i < __size; i++)
      __ptr[i].resize(cols);
}

vector& matrix::operator[](int idx) const
{ if (!__ptr || idx < 0 || idx >= __size)
    fprintf(stderr, "Array index out of bounds\n");
  return __ptr[idx];
}
