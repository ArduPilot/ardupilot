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

	// array indexing
	T &operator [](unsigned int i)
	{   return *(&x+i);   }

	// array indexing
	const T &operator [](unsigned int i) const
	{	return *(&x+i);   }

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
	const Vector2<T> operator -(void) const
	{	return Vector2<T>(-x, -y);	}

	// addition
	const Vector2<T> operator +(const Vector2<T> &v) const
	{	return Vector2<T>(x+v.x, y+v.y);	}

	// subtraction
	const Vector2<T> operator -(const Vector2<T> &v) const
	{   return Vector2<T>(x-v.x, y-v.y);	}

	// uniform scaling
	const Vector2<T> operator *(const T num) const
	{
		Vector2<T> temp(*this);
		return temp*=num;
	}

	// uniform scaling
	const Vector2<T> operator /(const T num) const
	{
		Vector2<T> temp(*this);
		return temp/=num;
	}

	// addition
	const Vector2<T> &operator +=(const Vector2<T> &v)
	{
		x+=v.x;	y+=v.y;
		return *this;
	}

	// subtraction
	const Vector2<T> &operator -=(const Vector2<T> &v)
	{
		x-=v.x;	y-=v.y;
		return *this;
	}

	// uniform scaling
	const Vector2<T> &operator *=(const T num)
	{
		x*=num;	y*=num;
		return *this;
	}

	// uniform scaling
	const Vector2<T> &operator /=(const T num)
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


// macro to make creating the ctors for derived vectors easier
#define VECTOR2_CTORS(name, type)   \
	/* trivial ctor */				\
	name() {}						\
	/* down casting ctor */			\
	name(const Vector2<type> &v): Vector2<type>(v.x, v.y) {}	\
	/* make x,y combination into a vector */					\
	name(type x0, type y0): Vector2<type>(x0, y0) {}



struct Vector2i: public Vector2<int>
{
	VECTOR2_CTORS(Vector2i, int)
};


struct Vector2ui: public Vector2<unsigned int>
{
	VECTOR2_CTORS(Vector2ui, unsigned int)
};


struct Vector2l: public Vector2<long>
{
	VECTOR2_CTORS(Vector2l, long)
};


struct Vector2ul: public Vector2<unsigned long>
{
	VECTOR2_CTORS(Vector2ul, unsigned long)
};


struct Vector2f: public Vector2<float>
{
	VECTOR2_CTORS(Vector2f, float)
};

#endif // VECTOR2_H
