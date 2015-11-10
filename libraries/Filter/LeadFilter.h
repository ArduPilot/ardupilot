// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 T his program is free s*oftware: you can redistribute it and/or modify
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

#ifndef LEADFILTER_H
#define LEADFILTER_H

class LeadFilter {
public:
    LeadFilter();
    void set_params(float omega, float ratio, float sample_freq);
    float apply(float u);
private:
    void compute_params();
    float _omega;
    float _ratio;
    float _sample_freq;
    float _B0;
    float _B1;
    float _A1;

    float _u_prev;
    float _y_prev;
};





#endif // LEADFILTER_H
