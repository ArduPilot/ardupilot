/*
	lu.h

	LU factorization Web service

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

//gsoap ns1 service name:	lu
//gsoap ns1 service style:	rpc
//gsoap ns1 service encoding:	encoded
//gsoap ns1 service namespace:	http://websrv.cs.fsu.edu/~engelen/lu.wsdl
//gsoap ns1 service location:	http://websrv.cs.fsu.edu/~engelen/luserver.cgi

//gsoap ns1 schema namespace: urn:lu
typedef double xsd__double;
typedef int xsd__int;
class vector // dynamic array of type SOAP-ENC:Array with arrayType="double[]"
{ public:
	xsd__double		*__ptr; // pointer to array of double
	int			__size; // number of elements pointed to
	int			__offset;
	struct soap		*soap;	// gSOAP env. instance was created
				vector();
				vector(struct soap *env);
				vector(struct soap *env, int size);
				vector(struct soap *env, int start, int end);
	virtual			~vector();
	virtual int		start(); // index of first element (=__offset)
	virtual int		end();   // index of last element
	virtual int		size();  // vector size
	virtual void		resize(int size);
	virtual void		resize(int start, int end);
	virtual double&		operator[](int i);
	virtual double		operator()(int i);
	virtual void		print();
};

class ivector // dynamic array of type SOAP-ENC:Array with arrayType="int[]"
{ public:
	xsd__int		*__ptr; // pointer to array of int
	int			__size; // number of elements pointed to
	int			__offset;
	struct soap		*soap;	// gSOAP env. instance was created
				ivector();
				ivector(struct soap *env);
				ivector(struct soap *env, int size);
				ivector(struct soap *env, int start, int end);
	virtual			~ivector();
	virtual int		start(); // index of first element (=__offset)
	virtual int		end();   // index of last element
	virtual int		size();  // vector size
	virtual void		resize(int size);
	virtual void		resize(int start, int end);
	virtual int&		operator[](int i);
	virtual int		operator()(int i);
	virtual void		print();
};

class matrix // dynamic array of type SOAP-ENC:Array with arrayType="double[][]"
{ public:
	vector			*__ptr; // pointer to array of vectors
	int			__size; // number of vectors pointed to
	int			__offset;
	struct soap		*soap;	// gSOAP env. instance was created
				matrix();
				matrix(struct soap *env);
				matrix(struct soap *env, int rows);
				matrix(struct soap *env, int rows, int cols);
				matrix(struct soap *env, int rowstart, int rowend, int colstart, int colend);
	virtual			~matrix();
	virtual int		start();
	virtual int		end();
	virtual int		size();
	virtual void		resize(int rows, int cols);
	virtual void		resize(int rowstart, int rowend, int colstart, int colend);
	virtual vector&		operator[](int i);
	virtual double		operator()(int i, int j);
	virtual void		print();
};
// LU decomposition, see Numerical Recipies for C
ns1__ludcmp(matrix *a, struct ns1__ludcmpResponse {matrix *a; ivector *i; xsd__double d;} &result);
// backsubstitution, see Numerical Recipies for C
ns1__lubksb(matrix *a, ivector *i, vector *b, vector *x);
// Linear system solver using LU decomposition: solves ax=b
ns1__lusol(matrix *a, vector *b, vector *x);
// Linear systems solver using LU decomposition: solves ax=b for all rows in x and b
ns1__lusols(matrix *a, matrix *b, matrix *x);
// Matrix inversion using LU decomposition
ns1__luinv(matrix *a, matrix *b);
// Determinant of matrix
ns1__ludet(matrix *a, xsd__double &d);
