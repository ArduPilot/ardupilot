/*
 * Vector.h
 * Copyright (C) James Goppert 2010 <james.goppert@gmail.com>
 *
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef Vector_H
#define Vector_H

#include "../FastSerial/BetterStream.h"
#include <stdlib.h>
#include <inttypes.h>
#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include <WProgram.h>
#endif

#ifdef ASSERT
const static char vectorSource[] ="Vector.hpp";
#endif

// vector
template <class dataType,class sumType=dataType>
class Vector
{
private:
    size_t size;
    size_t extraAllocationSize; // extra space to add after each allocation
    size_t sizeAllocated; // total allocated size
    dataType* data;
public:
    // default constructor
    Vector(const size_t & _size=0, const size_t & _extraAllocationSize=0) : size(0), extraAllocationSize(_extraAllocationSize), sizeAllocated(0), data(NULL) {
        setSize(_size);
    }
    // 3 vector constructor
    Vector(const dataType & a, const dataType & b, const dataType & c) : size(3), extraAllocationSize(extraAllocationSize), sizeAllocated(0), data(NULL) {
        setSize(size);
        (*this)[0]=a;
        (*this)[1]=b;
        (*this)[2]=c;
    }

    // construct from array
    Vector(const dataType* array, const size_t & _size, const size_t & _extraAllocationSize=0) : size(0), extraAllocationSize(_extraAllocationSize), sizeAllocated(0), data(NULL) {
        setSize(_size);
        for (size_t i=0; i<getSize(); i++)
            (*this)[i]=array[i];
    }
    // copy constructor
    Vector(const Vector &v) : size(0), extraAllocationSize(0), sizeAllocated(0), data(NULL) {
        setSize(v.getSize());
        for (size_t i=0; i<getSize(); i++)
            (*this)[i]=v[i];
    }
    // convert to float vector
    Vector<float> toFloat() const {
        Vector<float> v(getSize());
        for (size_t i=0; i<getSize(); i++)
            v[i]=(*this)[i];
        return v;
    }
    // destructor
    virtual ~Vector() {
        empty();
    }
    void empty() {
        if (data) delete [] data;
        data = NULL;
        sizeAllocated=0;
        size=0;
    }
    // set the size
    void setSize(const size_t & n) {
        if (n==0) {
            if (data) delete [] data;
            data = NULL;
            sizeAllocated=0;
        }
        if (n>sizeAllocated) {
            dataType * newData = new dataType[n+extraAllocationSize];
            memcpy(newData,data,sizeof(dataType)/sizeof(char)*getSize());
            memset(newData+size,0,sizeof(dataType)/sizeof(char)*(n-getSize()));
            delete[] data;
            data = newData;
            sizeAllocated=n+extraAllocationSize;
        }
        size=n;
    }
    // return size
    const size_t & getSize() const {
        return size;
    }
    // insert
    void insert(const size_t index, const dataType value) {
        //Serial.println("insert called");
#ifdef ASSERT
        assert(index<size+1,vectorSource,__LINE__);
#endif
        //Serial.print("Old Size: ");
        //Serial.println(getSize());
        setSize(getSize()+1);
        //Serial.print("New Size: ");
        //Serial.println(getSize());
        //Serial.print("Size of dataType");
        //Serial.println(sizeof(dataType));
        if (index != getSize()-1) {
            memmove(data+index+1,data+index,sizeof(dataType)/sizeof(char)*(getSize()-1-index));
            //Serial.println("memmove called and completed");
        }
        (*this)[index]=value;
        //Serial.println("insert done");
    }
    // remove
    void remove(const size_t & index) {
#ifdef ASSERT
        assert(index<size,vectorSource,__LINE__);
#endif
        memmove(data+index,data+index+1,getSize()-index-1);
        setSize(getSize()-1);
    }
    // push_back
    void push_back(const dataType & value) {
        //Serial.println("push_back called");
        insert(getSize(),value);
        //Serial.println("push_back done");
    }
    // pop_front
    dataType & pop_front() {
        dataType tmp = (*this)[0];
        remove(0);
        return tmp;
    }
    // push_back a vector
    void push_back(const Vector & vector) {
        for (size_t i=0; i<vector.getSize(); i++)
            push_back(vector[i]);
    }
    // const array access operator
    const dataType & operator[](const size_t & index) const {
#ifdef ASSERT
        assert(index<getSize(),vectorSource,__LINE__);
#endif
        return data[index];
    }
    // array access operator
    dataType & operator[](const size_t & index) {
#ifdef ASSERT
        assert(index<getSize(),vectorSource,__LINE__);
#endif
        return data[index];
    }
    // assignment operator
    Vector & operator=(const Vector & v) {
        setSize(v.getSize());
        for (size_t i=0; i<getSize(); i++)
            (*this)[i]=v[i];
        return *this;
    }
    // equal
    const bool operator==(const Vector& v) const {
#ifdef ASSERT
        assert(getSize()==v.getSize(),vectorSource,__LINE__);
#endif
        for (size_t i=0; i<getSize(); i++) {
            if ((*this)[i]!=v[i]) return false;
        }
        return true;
    }
    // not equal
    const bool operator!=(const Vector& v) const {
        return !((*this)==v);
    }
    // addition
    const Vector operator+(const Vector& v) const {
#ifdef ASSERT
        assert(v.getSize() == getSize(),vectorSource,__LINE__);
#endif
        Vector result(getSize());
        for (size_t i=0; i<getSize(); i++)
            result(i)=(*this)[i]+v[i];
        return result;
    }
    // addition
    const Vector operator+(const dataType& s) const {
        Vector result(getSize());
        for (size_t i=0; i<getSize(); i++)
            result[i]=(*this)[i]+s;
        return result;
    }
    // subtraction
    const Vector operator-(const Vector& v) const {
#ifdef ASSERT
        assert(v.getSize() == getSize(),vectorSource,__LINE__);
#endif
        Vector result(getSize());
        for (size_t i=0; i<getSize(); i++)
            result[i]=(*this)[i]-v[i];
        return result;
    }
    // negation
    const Vector operator-() const {
        Vector result(getSize());
        for (size_t i=0; i<getSize(); i++)
            result[i]=-(*this)[i];
        return result;
    }
    // +=
    Vector& operator+=(const Vector& v) {
#ifdef ASSERT
        assert(v.getSize() == getSize(),vectorSource,__LINE__);
#endif
        Vector result(getSize());
        for (size_t i=0; i<getSize(); i++)
            (*this)(i)+=v(i);
        return *this;
    }
    // -=
    Vector& operator-=( const Vector& v) {
#ifdef ASSERT
        assert(v.getSize() == getSize(),vectorSource,__LINE__);
#endif
        Vector result(getSize());
        for (size_t i=0; i<getSize(); i++)
            (*this)(i)-=v(i);
        return *this;
    }
    // elementwise  mult.
    const Vector operator*(const Vector & v) const {
        Vector result(getSize());
        for (size_t i=0; i<getSize(); i++)
            result(i)=(*this)(i)*v(i);
        return result;
    }

    // mult. by a scalar
    const Vector operator*(const dataType & s) const {
        Vector result(getSize());
        for (size_t i=0; i<getSize(); i++)
            result(i)=(*this)(i)*s;
        return result;
    }
    // div. by a scalar
    const Vector operator/(const dataType & s) const {
        Vector result(getSize());
        for (size_t i=0; i<getSize(); i++)
            result(i)=(*this)(i)/s;
        return result;
    }
    // elementwise div.
    const Vector operator/(const Vector & v) const {
        Vector result(getSize());
        for (size_t i=0; i<getSize(); i++)
            result(i)=(*this)(i)/v(i);
        return result;
    }

    // div. by a scalar
    Vector & operator/=(const dataType & s) {
        for (size_t i=0; i<getSize(); i++)
            (*this)(i)/=s;
        return *this;
    }
    // mult. by a scalar
    Vector & operator*=(const dataType & s) {
        for (size_t i=0; i<getSize(); i++)
            (*this)(i)*=s;
        return *this;
    }
    // cross/vector product
    const Vector cross(const Vector& v) const {
        Vector result(3), u=*this;
#ifdef ASSERT
        assert(u.getSize()==3 && v.getSize()==3,vectorSource,__LINE__);
#endif
        result(0) = u(1)*v(2)-u(2)*v(1);
        result(1) = -u(0)*v(2)+u(2)*v(0);
        result(2) = u(0)*v(1)-u(1)*v(0);
        return result;
    }
    // dot/scalar product
    const dataType dot(const Vector& v) const {
#ifdef ASSERT
        assert(getSize()==v.getSize(),vectorSource,__LINE__);
#endif
        dataType result;
        for (size_t i=0; i<getSize(); i++) result += (*this)(i)*v(i);
        return result;
    }
    // norm
    const dataType norm() const {
        return sqrt(dot(*this));
    }
    // unit vector
    const Vector unit() const {
        return (*this)*(1/norm());
    }
    // sum
    const sumType sum(const size_t & start=0,const int16_t & end=-1) const {
        size_t end2;
        if (end==-1) end2=getSize()-1;
        else end2=end;
        sumType _sum = 0;
        for (size_t i=start; i<=end2; i++) _sum += (*this)(i);
        return _sum;
    }
    void sumFletcher(uint8_t & CK_A, uint8_t & CK_B, const size_t & start=0,const int16_t & end=-1) const {
        size_t end2;
        if (end==-1) end2=getSize()-1;
        else end2=end;

        for (size_t i = start; i<=end2; i++) {
            CK_A += (*this)(i);
            CK_B += CK_A;
        }
    }
    // range
    const Vector range(const size_t & start, const size_t & stop) const {
        Vector v(stop-start+1);
        for (size_t i=start; i<=stop; i++) v(i-start) = (*this)(i);
        return v;
    }
    // to Array
    const dataType* toArray() const {
        dataType array[getSize()];
        for (size_t i=0; i<getSize(); i++) array[i] = (*this)(i);
        return array;
    }
    // printing
    void print(Stream & serial=Serial, const char * msg="", size_t format=0) const {
        serial.print(msg);
        for (size_t i=0; i<getSize(); i++) {
            serial.print((*this)(i),format);
            serial.print(" ");
        }
        serial.println();
    }
    // self test
    static bool selfTest(Stream & serial=Serial) {
        serial.println("Vector self test.");
        Vector u(3),v(3),w(3);
        u(0) = 1;
        u(1) = 2;
        u(2) = 3;
        v(0) = -4;
        v(1) = -5;
        v(2) = -6;
        u.print(serial,"u: ");
        v.print(serial,"v: ");
        (u+v).print(serial,"u + v: ");
        (u-v).print(serial,"u - v: ");
        Serial.print("u dot v: ");
        Serial.println(u.dot(v));
        Serial.print("size of u: ");
        Serial.println(u.getSize());
        Serial.print("size of v: ");
        Serial.println(v.getSize());
        w=u.cross(v);
        w.print(serial,"u cross v: ");
        Serial.print("size of u cross v: ");
        Serial.println(w.getSize());
        return true;
    }

};

#endif

// vim:ts=4:sw=4:expandtab
