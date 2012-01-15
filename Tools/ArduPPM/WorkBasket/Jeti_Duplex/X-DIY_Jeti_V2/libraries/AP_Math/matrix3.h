// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

// Copyright 2010 Michael Smith, all rights reserved.

//	This library is free software; you can redistribute it and / or
//	modify it under the terms of the GNU Lesser General Public
//	License as published by the Free Software Foundation; either
//	version 2.1 of the License, or (at your option) any later version.

// Inspired by:
/****************************************
 * 3D Vector Classes
 * By Bill Perone (billperone@yahoo.com)
 */

//
// 3x3 matrix implementation.
//
// Note that the matrix is organised in row-normal form (the same as
// applies to array indexing).
//
// In addition to the template, this header defines the following types:
//
// Matrix3i		3x3 matrix of signed integers
// Matrix3ui	3x3 matrix of unsigned integers
// Matrix3l		3x3 matrix of signed longs
// Matrix3ul	3x3 matrix of unsigned longs
// Matrix3f		3x3 matrix of signed floats
//

#ifndef MATRIX3_H
#define MATRIX3_H

#include "vector3.h"

// 3x3 matrix with elements of type T
template <typename T>
class Matrix3 {
public:

	// Vectors comprising the rows of the matrix
	Vector3<T>	a, b, c;

	// trivial ctor
	Matrix3<T>() {}

	// setting ctor
	Matrix3<T>(const Vector3<T> a0, const Vector3<T> b0, const Vector3<T> c0): a(a0), b(b0), c(c0) {}

	// setting ctor
	Matrix3<T>(const T ax, const T ay, const T az, const T bx, const T by, const T bz, const T cx, const T cy, const T cz): a(ax,ay,az), b(bx,by,bz), c(cx,cy,cz) {} 	
	
	// function call operator
	void operator () (const Vector3<T> a0, const Vector3<T> b0, const Vector3<T> c0)
	{	a = a0; b = b0; c = c0;  }

	// test for equality
	bool operator == (const Matrix3<T> &m)
	{	return (a==m.a && b==m.b && c==m.c);	}

	// test for inequality
	bool operator != (const Matrix3<T> &m)
	{	return (a!=m.a || b!=m.b || c!=m.c);	}

	// negation
	Matrix3<T> operator - (void) const
	{	return Matrix3<T>(-a,-b,-c);	}

	// addition
	Matrix3<T> operator + (const Matrix3<T> &m) const
	{   return Matrix3<T>(a+m.a, b+m.b, c+m.c);	 }
	Matrix3<T> &operator += (const Matrix3<T> &m)
	{	return *this = *this + m;	}

	// subtraction
	Matrix3<T> operator - (const Matrix3<T> &m) const
	{   return Matrix3<T>(a-m.a, b-m.b, c-m.c);	 }
	Matrix3<T> &operator -= (const Matrix3<T> &m)
	{	return *this = *this - m;	}
	   
	// uniform scaling
	Matrix3<T> operator * (const T num) const
	{	return Matrix3<T>(a*num, b*num, c*num);	}
	Matrix3<T> &operator *= (const T num)
	{	return *this = *this * num;	}
	 Matrix3<T> operator / (const T num) const
	{	return Matrix3<T>(a/num, b/num, c/num);	}
	Matrix3<T> &operator /= (const T num)
	{	return *this = *this / num;	}
	
	// multiplication by a vector
	Vector3<T> operator *(const Vector3<T> &v) const
	{
		return Vector3<T>(a.x * v.x + a.y * v.y + a.z * v.z,
						  b.x * v.x + b.y * v.y + b.z * v.z,
						  c.x * v.x + c.y * v.y + c.z * v.z);						  
	}

	// multiplication by another Matrix3<T>
	Matrix3<T> operator *(const Matrix3<T> &m) const
	{
		Matrix3<T> temp (Vector3<T>(a.x * m.a.x + a.y * m.b.x + a.z * m.c.x,
									a.x * m.a.y + a.y * m.b.y + a.z * m.c.y,
									a.x * m.a.z + a.y * m.b.z + a.z * m.c.z),
						 Vector3<T>(b.x * m.a.x + b.y * m.b.x + b.z * m.c.x,
									b.x * m.a.y + b.y * m.b.y + b.z * m.c.y,
									b.x * m.a.z + b.y * m.b.z + b.z * m.c.z),
						 Vector3<T>(c.x * m.a.x + c.y * m.b.x + c.z * m.c.x,
									c.x * m.a.y + c.y * m.b.y + c.z * m.c.y,
									c.x * m.a.z + c.y * m.b.z + c.z * m.c.z));
		return temp;
	}
	Matrix3<T> &operator *=(const Matrix3<T> &m)
	{	return *this = *this * m;	}

	// transpose the matrix
	Matrix3<T> transposed(void)
	{
		return Matrix3<T>(Vector3<T>(a.x, b.x, c.x),
						  Vector3<T>(a.y, b.y, c.y),
						  Vector3<T>(a.z, b.z, c.z));
	}
	Matrix3<T> transpose(void)
	{	return *this = transposed();	}

};

typedef Matrix3<int>			Matrix3i;
typedef Matrix3<unsigned int>	Matrix3ui;
typedef Matrix3<long>			Matrix3l;
typedef Matrix3<unsigned long>	Matrix3ul;
typedef Matrix3<float>			Matrix3f;

#endif // MATRIX3_H
