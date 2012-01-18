// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

// Copyright 2010 Michael Smith, all rights reserved.

//	This library is free software; you can redistribute it and / or
//	modify it under the terms of the GNU Lesser General Public
//	License as published by the Free Software Foundation; either
//	version 2.1 of the License, or (at your option) any later version.

// Derived closely from:
/****************************************
 * 2D Vector Classes
 * By Bill Perone (billperone@yahoo.com)
 * Original: 9-16-2002
 * Revised: 19-11-2003
 *          18-12-2003
 *          06-06-2004
 *
 * © 2003, This code is provided "as is" and you can use it freely as long as
 * credit is given to Bill Perone in the application it is used in
 ****************************************/

#ifndef VECTOR2_H
#define VECTOR2_H

#include <math.h>

template <typename T>
struct Vector2
{
	T x, y;


	// trivial ctor
	Vector2<T>() {}

	// setting ctor
	Vector2<T>(const T x0, const T y0): x(x0), y(y0) {}

	// function call operator
	void operator ()(const T x0, const T y0)
	{	x= x0; y= y0;	}

	// test for equality
	bool operator==(const Vector2<T> &v)
	{	return (x==v.x && y==v.y);	}

	// test for inequality
	bool operator!=(const Vector2<T> &v)
	{	return (x!=v.x || y!=v.y);	}

	// negation
	Vector2<T> operator -(void) const
	{	return Vector2<T>(-x, -y);	}

	// addition
	Vector2<T> operator +(const Vector2<T> &v) const
	{	return Vector2<T>(x+v.x, y+v.y);	}

	// subtraction
	Vector2<T> operator -(const Vector2<T> &v) const
	{   return Vector2<T>(x-v.x, y-v.y);	}

	// uniform scaling
	Vector2<T> operator *(const T num) const
	{
		Vector2<T> temp(*this);
		return temp*=num;
	}

	// uniform scaling
	Vector2<T> operator /(const T num) const
	{
		Vector2<T> temp(*this);
		return temp/=num;
	}

	// addition
	Vector2<T> &operator +=(const Vector2<T> &v)
	{
		x+=v.x;	y+=v.y;
		return *this;
	}

	// subtraction
	Vector2<T> &operator -=(const Vector2<T> &v)
	{
		x-=v.x;	y-=v.y;
		return *this;
	}

	// uniform scaling
	Vector2<T> &operator *=(const T num)
	{
		x*=num;	y*=num;
		return *this;
	}

	// uniform scaling
	Vector2<T> &operator /=(const T num)
	{
		x/=num;	y/=num;
		return *this;
	}

	// dot product
	T operator *(const Vector2<T> &v) const
	{	return x*v.x + y*v.y;	}

	// gets the length of this vector squared
	T length_squared() const
	{	return (T)(*this * *this);   }

	// gets the length of this vector
	T length() const
	{	return (T)sqrt(*this * *this);   }

	// normalizes this vector
	void normalize()
	{	*this/=length();	}

	// returns the normalized vector
	Vector2<T> normalized() const
	{   return  *this/length();  }

	// reflects this vector about n
	void reflect(const Vector2<T> &n)
	{
		Vector2<T> orig(*this);
		project(n);
		*this= *this*2 - orig;
	}

	// projects this vector onto v
	void project(const Vector2<T> &v)
	{	*this= v * (*this * v)/(v*v);	}

	// returns this vector projected onto v
	Vector2<T> projected(const Vector2<T> &v)
	{   return v * (*this * v)/(v*v);	}

	// computes the angle between 2 arbitrary vectors
	static inline T angle(const Vector2<T> &v1, const Vector2<T> &v2)
	{   return (T)acosf((v1*v2) / (v1.length()*v2.length()));  }

	// computes the angle between 2 normalized arbitrary vectors
	static inline T angle_normalized(const Vector2<T> &v1, const Vector2<T> &v2)
	{   return (T)acosf(v1*v2);  }

};

typedef Vector2<int>			Vector2i;
typedef Vector2<unsigned int>	Vector2ui;
typedef Vector2<long>			Vector2l;
typedef Vector2<unsigned long>	Vector2ul;
typedef Vector2<float>			Vector2f;

#endif // VECTOR2_H
