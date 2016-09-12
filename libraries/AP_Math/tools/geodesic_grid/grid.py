# Copyright (C) 2016  Intel Corporation. All rights reserved.
#
# This file is free software: you can redistribute it and/or modify it
# under the terms of the GNU General Public License as published by the
# Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This file is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program.  If not, see <http://www.gnu.org/licenses/>.
import icosahedron as ico

def section_triangle(s):
    a, b, c = ico.triangles[s / 4]
    # project the middle points to the sphere
    alpha = a.length() / (2.0 * ico.g)
    ma, mb, mc = alpha * (a + b), alpha * (b + c), alpha * (c + a)

    sub = s % 4
    if sub == 0:
        return ico.Triangle(ma, mb, mc)
    elif sub == 1:
        return ico.Triangle(a, ma, mc)
    elif sub == 2:
        return ico.Triangle(ma, b, mb)
    else:
        return ico.Triangle(mc, mb, c)
