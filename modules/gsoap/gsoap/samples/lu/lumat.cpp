/*
	lumat.cpp

	LU factorization vector and matrix operations.

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

////////////////////////////////////////////////////////////////////////////////
//
//  vector
//
////////////////////////////////////////////////////////////////////////////////

vector::vector()
{ vector(NULL);
}

vector::vector(struct soap *env)
{ soap = env;
  __ptr = 0;
  __size = 0;
  __offset = 0;
}

vector::vector(struct soap *env, int size)
{ soap = env;
  __size = size;
  __offset = 1;
  __ptr = (double*)soap_malloc(soap, size*sizeof(double));
  for (int i = 0; i < size; i++)
    __ptr[i] = 0.0;
}

vector::vector(struct soap *env, int start, int end)
{ soap = env;
  __size = end-start+1;
  __offset = start;
  __ptr = (double*)soap_malloc(soap, __size*sizeof(double));
  for (int i = 0; i < __size; i++)
    __ptr[i] = 0.0;
}

vector::~vector()
{	// no deallocation: let gSOAP handle all dynamic allocation and deallocation
}

int vector::start()
{ return __offset;
}

int vector::end()
{ return __size+__offset-1;
}

int vector::size()
{ return __size;
}

void vector::resize(int size)
{ resize(1, size);
}

void vector::resize(int start, int end)
{ double *p;
  int n, size = end-start+1;
  if (__offset == start && __size == size)
    return; // nothing to do
  p = (double*)soap_malloc(soap, size*sizeof(double));
  for (int i = 0; i < size; i++)
    p[i] = 0.0;
  if (__ptr)
  { if (start < __offset)
    { n = (size-__offset+start <= __size ? size-__offset+start : __size);
      for (int i = 0; i < n; i++)
        p[__offset-start+i] = __ptr[i];
    }
    else
    { n = (__size-start+__offset <= size ? __size-start+__offset : size);
      for (int i = 0; i < n; i++)
        p[i] = __ptr[start-__offset+i];
    }
    soap_dealloc(soap, __ptr);
  }
  __ptr = p;
  __size = size;
  __offset = start;
}

double& vector::operator[](int i)
{ return __ptr[i-__offset];
}

double vector::operator()(int i)
{ if (i >= __offset && i < __size+__offset)
    return __ptr[i-__offset];
  return 0.0;
}

void vector::print()
{ int i;
  for (i = 1; i <= end(); i++)
    printf("% f ", (*this)(i));
  printf("\n");
}

////////////////////////////////////////////////////////////////////////////////
//
//  ivector
//
////////////////////////////////////////////////////////////////////////////////

ivector::ivector()
{ ivector(NULL);
}

ivector::ivector(struct soap *env)
{ soap = env;
  __ptr = 0;
  __size = 0;
}

ivector::ivector(struct soap *env, int size)
{ soap = env;
  __size = size;
  __offset = 1;
  __ptr = (int*)soap_malloc(soap, size*sizeof(int));
  for (int i = 0; i < size; i++)
    __ptr[i] = 0;
}

ivector::ivector(struct soap *env, int start, int end)
{ soap = env;
  __size = end-start+1;
  __offset = start;
  __ptr = (int*)soap_malloc(soap, __size*sizeof(int));
  for (int i = 0; i < __size; i++)
    __ptr[i] = 0;
}

ivector::~ivector()
{	// no deallocation: let gSOAP handle all dynamic allocation and deallocation
}

int ivector::start()
{ return __offset;
}

int ivector::end()
{ return __size+__offset-1;
}

int ivector::size()
{ return __size;
}

void ivector::resize(int size)
{ resize(1, size);
}

void ivector::resize(int start, int end)
{ int *p;
  int n, size = end-start+1;
  if (__offset == start && __size == size)
    return; // nothing to do
  p = (int*)soap_malloc(soap, size*sizeof(int));
  for (int i = 0; i < size; i++)
    p[i] = 0;
  if (__ptr)
  { if (start < __offset)
    { n = (size-__offset+start <= __size ? size-__offset+start : __size);
      for (int i = 0; i < n; i++)
        p[__offset-start+i] = __ptr[i];
    }
    else
    { n = (__size-start+__offset <= size ? __size-start+__offset : size);
      for (int i = 0; i < n; i++)
        p[i] = __ptr[start-__offset+i];
    }
    soap_dealloc(soap, __ptr);
  }
  __ptr = p;
  __size = size;
  __offset = start;
}

int& ivector::operator[](int i)
{ return __ptr[i-__offset];
}

int ivector::operator()(int i)
{ if (i >= __offset && i < __size+__offset)
    return __ptr[i-__offset];
  return 0;
}

void ivector::print()
{ int i;
  for (i = 1; i <= end(); i++)
    printf("%4d ", (*this)(i));
  printf("\n");
}

////////////////////////////////////////////////////////////////////////////////
//
//  matrix
//
////////////////////////////////////////////////////////////////////////////////

matrix::matrix()
{ matrix(NULL);
}

matrix::matrix(struct soap *env)
{ soap = env;
  __ptr = 0;
  __size = 0;
  __offset = 0;
}

matrix::matrix(struct soap *env, int rows)
{ soap = env;
  __ptr = soap_new_vector(soap, rows);
  __size = rows;
  __offset = 1;
}

matrix::matrix(struct soap *env, int rows, int cols)
{ soap = env;
  __size = rows;
  __offset = 1;
  __ptr = soap_new_vector(soap, __size);
  for (int i = 0; i < __size; i++)
    __ptr[i].resize(cols);
}

matrix::matrix(struct soap *env, int rowstart, int rowend, int colstart, int colend)
{ soap = env;
  __size = rowend-rowstart+1;
  __offset = rowstart;
  __ptr = soap_new_vector(soap, __size);
  for (int i = 0; i <= __size-__offset; i++)
    __ptr[i].resize(colstart, colend);
}

matrix::~matrix()
{ 	// no deallocation: let gSOAP handle all dynamic allocation and deallocation
}

int matrix::start()
{ return __offset;
}

int matrix::end()
{ return __size+__offset-1;
}

int matrix::size()
{ return __size;
}

void matrix::resize(int rows, int cols)
{ resize(1, rows, 1, cols);
}

void matrix::resize(int rowstart, int rowend, int colstart, int colend)
{ int i;
  vector *p;
  int n, size = rowend-rowstart+1;
  if (__offset != rowstart || __size != rowend-rowstart+1)
  { if (__ptr)
    { p = soap_new_vector(soap, size);
      if (rowstart < __offset)
      { for (i = 0; i < __offset-rowstart; i++)
          p[i].resize(colstart, colend);
        n = (size-__offset+rowstart <= __size ? size-__offset+rowstart : __size);
        for (i = 0; i < n; i++)
        { __ptr[i].resize(colstart, colend);
          p[__offset-rowstart+i] = __ptr[i];
        }
	for (; i < size-__offset+rowstart; i++)
	  p[i].resize(colstart, colend);
      }
      else
      { n = (__size-rowstart+__offset <= size ? __size-rowstart+__offset : size);
        for (i = 0; i < n; i++)
        { __ptr[i].resize(colstart, colend);
          p[i] = __ptr[rowstart-__offset+i];
        }
        for (; i < size; i++)
          p[i].resize(colstart, colend);
      }
      __ptr = p;
      __size = size;
      __offset = rowstart;
    }
    else
    { __size = size;
      __offset = rowstart;
      __ptr = soap_new_vector(soap, size);
      for (i = 0; i < size; i++)
        __ptr[i].resize(colstart, colend);
    }
  }
  else
    for (i = 0; i < size; i++)
      __ptr[i].resize(colstart, colend);
}

vector& matrix::operator[](int i)
{ return __ptr[i-__offset];
}

double matrix::operator()(int i, int j)
{ int s;
  if (i >= __offset && i < __size+__offset)
  { s = __ptr[i-__offset].__offset;
    if (j >= s && j < __ptr[i-__offset].__size+s)
      return __ptr[i-__offset].__ptr[j-s];
  }
  return 0.0;
}

void matrix::print()
{ int i, j;
  for (i = start(); i <= end(); i++)
  { for (j = 1; j < (*this)[i].start(); j++)
      printf("          ");
    for (; j <= (*this)[i].end(); j++)
      printf("% f ", (*this)(i, j));
    printf("\n");
  }
}
