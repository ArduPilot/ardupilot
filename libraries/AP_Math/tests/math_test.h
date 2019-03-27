/*
 * Copyright (C) 2016  Intel Corporation. All rights reserved.
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
#pragma once

#include <iostream>

#include <AP_gtest.h>
#include <AP_Math/AP_Math.h>

/*
 * Overload operator << for AP_Math types to allow nice printing for failed
 * tests.
 */

template <typename T>
::std::ostream& operator<<(::std::ostream& os, const Matrix3<T>& m)
{
    return os << "{{" << m.a.x << ", " << m.a.y << ", " << m.a.z << "}, " <<
                  "{" << m.b.x << ", " << m.b.y << ", " << m.b.z << "}, " <<
                  "{" << m.c.x << ", " << m.c.y << ", " << m.c.z << "}}";
}

template <typename T>
::std::ostream& operator<<(::std::ostream& os, const Vector3<T>& v)
{
    return os << "{" << v.x << ", " << v.y << ", " << v.z << "}";
}
