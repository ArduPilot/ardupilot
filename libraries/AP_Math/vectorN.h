// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef VECTORN_H
#define VECTORN_H

#include <math.h>
#include <string.h>
#if MATH_CHECK_INDEXES
#include <assert.h>
#endif

template <typename T, uint8_t N>
class VectorN
{
public:
    // trivial ctor
    inline VectorN<T,N>() {
        memset(_v, 0, sizeof(T)*N);
    }

    inline T & operator[](uint8_t i) {
#if MATH_CHECK_INDEXES
        assert(i >= 0 && i < N);
#endif
        return _v[i];
    }

    inline const T & operator[](uint8_t i) const {
#if MATH_CHECK_INDEXES
        assert(i >= 0 && i < N);
#endif
        return _v[i];
    }

    // test for equality
    bool operator ==(const VectorN<T,N> &v) const {
        for (uint8_t i=0; i<N; i++) {
            if (_v[i] != v[i]) return false;
        }
        return true;
    }

    // zero the vector
    inline void zero()
    {
        memset(_v, 0, sizeof(T)*N);
    }

private:
    T _v[N];
};

#endif // VECTORN_H
