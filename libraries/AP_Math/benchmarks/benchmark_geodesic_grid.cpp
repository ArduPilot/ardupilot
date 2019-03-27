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
#include <AP_gbenchmark.h>

#include <AP_Math/AP_GeodesicGrid.h>

static const Vector3f triangles[20][3] = {
    {{-M_GOLDEN, 1, 0}, {-1, 0,-M_GOLDEN}, {-M_GOLDEN,-1, 0}},
    {{-1, 0,-M_GOLDEN}, {-M_GOLDEN,-1, 0}, { 0,-M_GOLDEN,-1}},
    {{-M_GOLDEN,-1, 0}, { 0,-M_GOLDEN,-1}, { 0,-M_GOLDEN, 1}},
    {{-1, 0,-M_GOLDEN}, { 0,-M_GOLDEN,-1}, { 1, 0,-M_GOLDEN}},
    {{ 0,-M_GOLDEN,-1}, { 0,-M_GOLDEN, 1}, { M_GOLDEN,-1, 0}},
    {{ 0,-M_GOLDEN,-1}, { 1, 0,-M_GOLDEN}, { M_GOLDEN,-1, 0}},
    {{ M_GOLDEN,-1, 0}, { 1, 0,-M_GOLDEN}, { M_GOLDEN, 1, 0}},
    {{ 1, 0,-M_GOLDEN}, { M_GOLDEN, 1, 0}, { 0, M_GOLDEN,-1}},
    {{ 1, 0,-M_GOLDEN}, { 0, M_GOLDEN,-1}, {-1, 0,-M_GOLDEN}},
    {{ 0, M_GOLDEN,-1}, {-M_GOLDEN, 1, 0}, {-1, 0,-M_GOLDEN}},

    {{ M_GOLDEN,-1, 0}, { 1, 0, M_GOLDEN}, { M_GOLDEN, 1, 0}},
    {{ 1, 0, M_GOLDEN}, { M_GOLDEN, 1, 0}, { 0, M_GOLDEN, 1}},
    {{ M_GOLDEN, 1, 0}, { 0, M_GOLDEN, 1}, { 0, M_GOLDEN,-1}},
    {{ 1, 0, M_GOLDEN}, { 0, M_GOLDEN, 1}, {-1, 0, M_GOLDEN}},
    {{ 0, M_GOLDEN, 1}, { 0, M_GOLDEN,-1}, {-M_GOLDEN, 1, 0}},
    {{ 0, M_GOLDEN, 1}, {-1, 0, M_GOLDEN}, {-M_GOLDEN, 1, 0}},
    {{-M_GOLDEN, 1, 0}, {-1, 0, M_GOLDEN}, {-M_GOLDEN,-1, 0}},
    {{-1, 0, M_GOLDEN}, {-M_GOLDEN,-1, 0}, { 0,-M_GOLDEN, 1}},
    {{-1, 0, M_GOLDEN}, { 0,-M_GOLDEN, 1}, { 1, 0, M_GOLDEN}},
    {{ 0,-M_GOLDEN, 1}, { M_GOLDEN,-1, 0}, { 1, 0, M_GOLDEN}},
};

static bool section_triangle(unsigned int section_index,
                             Vector3f& a,
                             Vector3f& b,
                             Vector3f& c) {
    if (section_index >= 80) {
        return false;
    }

    unsigned int i = section_index / 4;
    unsigned int j = section_index % 4;
    auto& t = triangles[i];
    Vector3f mt[3]{(t[0] + t[1]) / 2, (t[1] + t[2]) / 2, (t[2] + t[0]) / 2};

    switch (j) {
        case 0:
            a = mt[0];
            b = mt[1];
            c = mt[2];
            break;
        case 1:
            a = t[0];
            b = mt[0];
            c = mt[2];
            break;
        case 2:
            a = mt[0];
            b = t[1];
            c = mt[1];
            break;
        case 3:
            a = mt[2];
            b = mt[1];
            c = t[2];
            break;
    }

    return true;
}

static void BM_GeodesicGridSections(benchmark::State& state)
{
    Vector3f v, a, b, c;
    int section = state.range_x();

    section_triangle(section, a, b, c);
    v = (a + b + c) / 3.0f;

    while (state.KeepRunning()) {
        int s = AP_GeodesicGrid::section(v);
        gbenchmark_escape(&s);
    }
}

/* Benchmark each section */
BENCHMARK(BM_GeodesicGridSections)->DenseRange(0, 79);

BENCHMARK_MAIN()
