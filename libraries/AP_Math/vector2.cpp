/*
 * vector3.cpp
 * Copyright (C) Andrew Tridgell 2012
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

#pragma GCC optimize("O3")

#include "AP_Math.h"

template <typename T>
float Vector2<T>::length(void) const
{
	return norm(x, y);
}


// dot product
template <typename T>
T Vector2<T>::operator *(const Vector2<T> &v) const
{
    return x*v.x + y*v.y;
}

// cross product
template <typename T>
T Vector2<T>::operator %(const Vector2<T> &v) const
{
    return x*v.y - y*v.x;
}

template <typename T>
Vector2<T> &Vector2<T>::operator *=(const T num)
{
    x*=num; y*=num;
    return *this;
}

template <typename T>
Vector2<T> &Vector2<T>::operator /=(const T num)
{
    x /= num; y /= num;
    return *this;
}

template <typename T>
Vector2<T> &Vector2<T>::operator -=(const Vector2<T> &v)
{
    x -= v.x; y -= v.y;
    return *this;
}

template <typename T>
bool Vector2<T>::is_nan(void) const
{
    return isnan(x) || isnan(y);
}

template <typename T>
bool Vector2<T>::is_inf(void) const
{
    return isinf(x) || isinf(y);
}

template <typename T>
Vector2<T> &Vector2<T>::operator +=(const Vector2<T> &v)
{
    x+=v.x; y+=v.y;
    return *this;
}

template <typename T>
Vector2<T> Vector2<T>::operator /(const T num) const
{
    return Vector2<T>(x/num, y/num);
}

template <typename T>
Vector2<T> Vector2<T>::operator *(const T num) const
{
    return Vector2<T>(x*num, y*num);
}

template <typename T>
Vector2<T> Vector2<T>::operator -(const Vector2<T> &v) const
{
    return Vector2<T>(x-v.x, y-v.y);
}

template <typename T>
Vector2<T> Vector2<T>::operator +(const Vector2<T> &v) const
{
    return Vector2<T>(x+v.x, y+v.y);
}

template <typename T>
Vector2<T> Vector2<T>::operator -(void) const
{
    return Vector2<T>(-x,-y);
}

template <typename T>
bool Vector2<T>::operator ==(const Vector2<T> &v) const
{
    return (is_equal(x,v.x) && is_equal(y,v.y));
}

template <typename T>
bool Vector2<T>::operator !=(const Vector2<T> &v) const
{
    return (!is_equal(x,v.x) || !is_equal(y,v.y));
}

template <typename T>
float Vector2<T>::angle(const Vector2<T> &v2) const
{
    float len = this->length() * v2.length();
    if (len <= 0) {
        return 0.0f;
    }
    float cosv = ((*this)*v2) / len;
    if (cosv >= 1) {
        return 0.0f;
    }
    if (cosv <= -1) {
        return M_PI;
    }
    return acosf(cosv);
}

// only define for float
template float Vector2<float>::length(void) const;
template float Vector2<float>::operator *(const Vector2<float> &v) const;
template float Vector2<float>::operator %(const Vector2<float> &v) const;
template Vector2<float> &Vector2<float>::operator *=(const float num);
template Vector2<float> &Vector2<float>::operator /=(const float num);
template Vector2<float> &Vector2<float>::operator -=(const Vector2<float> &v);
template Vector2<float> &Vector2<float>::operator +=(const Vector2<float> &v);
template Vector2<float> Vector2<float>::operator /(const float num) const;
template Vector2<float> Vector2<float>::operator *(const float num) const;
template Vector2<float> Vector2<float>::operator +(const Vector2<float> &v) const;
template Vector2<float> Vector2<float>::operator -(const Vector2<float> &v) const;
template Vector2<float> Vector2<float>::operator -(void) const;
template bool Vector2<float>::operator ==(const Vector2<float> &v) const;
template bool Vector2<float>::operator !=(const Vector2<float> &v) const;
template bool Vector2<float>::is_nan(void) const;
template bool Vector2<float>::is_inf(void) const;
template float Vector2<float>::angle(const Vector2<float> &v) const;

// define for int
template bool Vector2<int>::operator ==(const Vector2<int> &v) const;
