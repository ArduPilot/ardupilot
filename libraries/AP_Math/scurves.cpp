
#define ALLOW_DOUBLE_MATH_FUNCTIONS

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

#include <AP_Math/AP_Math.h>
#include "scurves.h"
#include <AP_HAL/AP_HAL.h>

extern const AP_HAL::HAL& hal;

void scurves::Segment(float T, enum jtype_t Jtype, float J, float A, float V, float P)
{
    if(num_items >= array_size_max) {
        hal.console->printf("You fucked up");
        return;
    }
    oT[num_items] = T;
    oJtype[num_items] = Jtype;
    oJ[num_items] = J;
    oA[num_items] = A;
    oV[num_items] = V;
    oP[num_items] = P;
    num_items++;
    //    hal.console->printf("Segment - J %4.2f, T %4.2f, A %4.2f, V %4.2f, P %4.2f\n", J, T, A, V, P);
}

void scurves::Cal_Init(float T0, float J0, float A0, float V0, float P0)
{
    _t = 0.0f;
    num_items = 0;
    enum jtype_t Jtype = JTYPE_CONSTANT;
    float J = J0;
    float T = T0;
    float A = A0;
    float V = V0;
    float P = P0;

    Segment(T, Jtype, J, A, V, P);
}

void scurves::Cal_T(float tin, float J0)
{
    enum jtype_t Jtype = JTYPE_CONSTANT;
    float J = J0;
    float T = oT[num_items - 1] + tin;
    float A = oA[num_items - 1] + J0*tin;
    float V = oV[num_items - 1] + oA[num_items - 1]*tin + 0.5*J0*sq(tin);
    float P = oP[num_items - 1] + oV[num_items - 1]*tin + 0.5*oA[num_items - 1]*sq(tin) + (1/6.0)*J0*powf(tin,3.0);
    Segment(T, Jtype, J, A, V, P);
}

void scurves::Cal_JS1(float tj, float Jp)
{
    float Beta = M_PI/tj;
    float Alpha = Jp/2.0;
    float AT = Alpha*tj;
    float VT = Alpha*(sq(tj)/2.0 - 2.0/sq(Beta));
    float PT = Alpha*((-1.0/sq(Beta))*tj + (1/6.0)*powf(tj,3.0));

    enum jtype_t Jtype = JTYPE_POSITIVE;
    float J = Jp;
    float T = oT[num_items - 1] + tj;
    float A = oA[num_items - 1] + AT;
    float V = oV[num_items - 1] + oA[num_items - 1]*tj + VT;
    float P = oP[num_items - 1] + oV[num_items - 1]*tj + 0.5*oA[num_items - 1]*sq(tj) + PT;
    Segment(T, Jtype, J, A, V, P);
}

void scurves::Cal_JS2(float tj, float Jp)
{
    float Beta = M_PI/tj;
    float Alpha = Jp/2.0;
    float AT = Alpha*tj;
    float VT = Alpha*(sq(tj)/2.0 - 2.0/sq(Beta));
    float PT = Alpha*((-1.0/sq(Beta))*tj + (1/6.0)*powf(tj,3.0));
    float A2T = Jp*tj;
    float V2T = Jp*sq(tj);
    float P2T = Alpha*((-1.0/sq(Beta))*2.0*tj +(4.0/3.0)*powf(tj,3.0));

//    hal.console->printf("Cal_JS2 - Beta %4.2f, Alpha %4.2f, AT %4.2f, VT %4.2f, PT %4.2f, A2T %4.2f, V2T %4.2f, P2T %4.2f\n", Beta, Alpha, AT, VT, PT, A2T, V2T, P2T);

    enum jtype_t Jtype = JTYPE_NEGATIVE;
    float J = Jp;
    float T = oT[num_items - 1] + tj;
    float A = (oA[num_items - 1]-AT) + A2T;
    float V = (oV[num_items - 1]-VT) + (oA[num_items - 1]-AT)*tj + V2T;
    float P = (oP[num_items - 1]-PT) + (oV[num_items - 1]-VT)*tj + 0.5*(oA[num_items - 1]-AT)*sq(tj) + P2T;
    Segment(T, Jtype, J, A, V, P);
}

void scurves::Cal_tj_Jp_Tcj(float tj, float Jp, float Tcj)
{
    Cal_JS1(tj, Jp);
//    hal.console->printf("Cal_JS1 - tj %4.2f, Tcj %4.2f\n", tj, Jp);
    Cal_T(Tcj, Jp);
//    hal.console->printf("Cal_T - Tcj %4.2f, Tcj %4.2f\n", Tcj, Jp);
    Cal_JS2(tj, Jp);
//    hal.console->printf("Cal_JS2 - tj %4.2f, Tcj %4.2f\n", tj, Jp);
}

void scurves::Cal_Pn(float Pp)
{
    _t = 0.0f;
//    timer == 40;

    if (is_zero(Pp)) {
        return;
    }

    hal.console->printf("Cal_Pn : %4.2f, otj %4.2f, oJp %4.2f, oAp %4.2f, oVp %4.2f\n", Pp, otj, oJp, oAp, oVp);
    float tj = otj;
    float Jp = oJp;
    float Ap = oAp;
    float Vp = oVp;
    float V0 = oV[num_items - 1];
    float P0 = oP[num_items - 1];

    float t2, t4, t6;
    Cal_Pos(tj, V0, P0, Jp, Ap, Vp, Pp/2.0, Jp, t2, t4, t6);
//    hal.console->printf("Block1 : Jp %4.2f, t2 %4.2f, t4 %4.2f, t6 %4.2f\n", Jp, t2, t4, t6);
    Cal_tj_Jp_Tcj(tj, Jp, t2);
    Cal_T(t4, 0.0);
    Cal_tj_Jp_Tcj(tj, -Jp, t6);

    float Tcv = (Pp/2.0-oP[num_items - 1])/oV[num_items - 1];
    Cal_T(Tcv, 0.0);
    Cal_T(Tcv, 0.0);
//    hal.console->printf("Block3 : tin %4.2f, J0 %4.2f\n", Tcv, 0.0);

    Cal_Pos(tj, 0.0f, P0, Jp, Ap, Vp, Pp/2.0, Jp, t2, t4, t6);
//    hal.console->printf("Block4 : Jp %4.2f, t2 %4.2f, t4 %4.2f, t6 %4.2f\n", Jp, t2, t4, t6);
    Cal_tj_Jp_Tcj(tj, -Jp, t6);
    Cal_T(t4, 0.0);
    Cal_tj_Jp_Tcj(tj, Jp, t2);

//    hal.console->printf("T, Jt, J, A, V, P\n");
//    for (uint8_t i = 0; i < num_items; i++) {
//        hal.console->printf("%4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2f\n", oT[i], (float)oJtype[i], oJ[i], oA[i], oV[i], oP[i]);
//    }
}

void scurves::Cal_Pos(float tj, float V0, float P0, float Jp, float Ap, float Vp, float Pp,
                      float& Jp_out, float& t2_out, float& t4_out, float& t6_out)
{
    Ap = MIN(MIN(Ap, (Vp - V0)/(2.0*tj)), (Pp - P0 + 4.0*V0*tj)/(4.0*sq(tj)));
    if (fabsf(Ap) < Jp*tj) {
        Jp = Ap/tj;
        Jp_out = Jp;
        t2_out = 0.0;
        if ((Vp <= V0 + 2.0*Ap*tj) || (Pp <= P0 + 4.0*V0*tj + 4.0*Ap*sq(tj))) {
            t4_out = 0.0;
        } else {
            t4_out = MIN(-(V0-Vp+Ap*tj+(Ap*Ap)/Jp)/Ap, \
                    MAX( \
                        ((Ap*Ap)*(-3.0/2.0)+safe_sqrt((Ap*Ap*Ap*Ap)*(1.0/4.0)+(Jp*Jp)*(V0*V0)+(Ap*Ap)*(Jp*Jp)*(tj*tj)*(1.0/4.0)-Ap*(Jp*Jp)*P0*2.0+Ap*(Jp*Jp)*Pp*2.0-(Ap*Ap)*Jp*V0+(Ap*Ap*Ap)*Jp*tj*(1.0/2.0)-Ap*(Jp*Jp)*V0*tj)-Jp*V0-Ap*Jp*tj*(3.0/2.0))/(Ap*Jp), \
                        ((Ap*Ap)*(-3.0/2.0)-safe_sqrt((Ap*Ap*Ap*Ap)*(1.0/4.0)+(Jp*Jp)*(V0*V0)+(Ap*Ap)*(Jp*Jp)*(tj*tj)*(1.0/4.0)-Ap*(Jp*Jp)*P0*2.0+Ap*(Jp*Jp)*Pp*2.0-(Ap*Ap)*Jp*V0+(Ap*Ap*Ap)*Jp*tj*(1.0/2.0)-Ap*(Jp*Jp)*V0*tj)-Jp*V0-Ap*Jp*tj*(3.0/2.0))/(Ap*Jp) \
                        ));
        }
    } else {
        if ((Vp < V0+Ap*tj+(Ap*Ap)/Jp) || (Pp < P0+1.0/(Jp*Jp)*(Ap*Ap*Ap+Ap*Jp*(V0*2.0+Ap*tj*2.0))+V0*tj*2.0+Ap*(tj*tj))) {
            Ap = MIN(MIN(Ap, \
                    MAX( \
                            Jp*(tj+safe_sqrt((V0*-4.0+Vp*4.0+Jp*(tj*tj))/Jp))*(-1.0/2.0), \
                            Jp*(tj-safe_sqrt((V0*-4.0+Vp*4.0+Jp*(tj*tj))/Jp))*(-1.0/2.0))), \
                    Jp*tj*(-2.0/3.0)+((Jp*Jp)*(tj*tj)*(1.0/9.0)-Jp*V0*(2.0/3.0))*1.0/powf(safe_sqrt(powf((Jp*Jp)*P0*(1.0/2.0)-(Jp*Jp)*Pp*(1.0/2.0)+(Jp*Jp*Jp)*(tj*tj*tj)*(8.0/2.7E1)-Jp*tj*((Jp*Jp)*(tj*tj)+Jp*V0*2.0)*(1.0/3.0)+(Jp*Jp)*V0*tj,2.0)-powf((Jp*Jp)*(tj*tj)*(1.0/9.0)-Jp*V0*(2.0/3.0),3.0))-(Jp*Jp)*P0*(1.0/2.0)+(Jp*Jp)*Pp*(1.0/2.0)-(Jp*Jp*Jp)*(tj*tj*tj)*(8.0/2.7E1)+Jp*tj*((Jp*Jp)*(tj*tj)+Jp*V0*2.0)*(1.0/3.0)-(Jp*Jp)*V0*tj,1.0/3.0)+powf(safe_sqrt(powf((Jp*Jp)*P0*(1.0/2.0)-(Jp*Jp)*Pp*(1.0/2.0)+(Jp*Jp*Jp)*(tj*tj*tj)*(8.0/2.7E1)-Jp*tj*((Jp*Jp)*(tj*tj)+Jp*V0*2.0)*(1.0/3.0)+(Jp*Jp)*V0*tj,2.0)-powf((Jp*Jp)*(tj*tj)*(1.0/9.0)-Jp*V0*(2.0/3.0),3.0))-(Jp*Jp)*P0*(1.0/2.0)+(Jp*Jp)*Pp*(1.0/2.0)-(Jp*Jp*Jp)*(tj*tj*tj)*(8.0/2.7E1)+Jp*tj*((Jp*Jp)*(tj*tj)+Jp*V0*2.0)*(1.0/3.0)-(Jp*Jp)*V0*tj,1.0/3.0));
            t4_out = 0;
        } else {
            t4_out = MIN(-(V0-Vp+Ap*tj+(Ap*Ap)/Jp)/Ap, \
                    MAX( \
                            ((Ap*Ap)*(-3.0/2.0)+safe_sqrt((Ap*Ap*Ap*Ap)*(1.0/4.0)+(Jp*Jp)*(V0*V0)+(Ap*Ap)*(Jp*Jp)*(tj*tj)*(1.0/4.0)-Ap*(Jp*Jp)*P0*2.0+Ap*(Jp*Jp)*Pp*2.0-(Ap*Ap)*Jp*V0+(Ap*Ap*Ap)*Jp*tj*(1.0/2.0)-Ap*(Jp*Jp)*V0*tj)-Jp*V0-Ap*Jp*tj*(3.0/2.0))/(Ap*Jp), \
                            ((Ap*Ap)*(-3.0/2.0)-safe_sqrt((Ap*Ap*Ap*Ap)*(1.0/4.0)+(Jp*Jp)*(V0*V0)+(Ap*Ap)*(Jp*Jp)*(tj*tj)*(1.0/4.0)-Ap*(Jp*Jp)*P0*2.0+Ap*(Jp*Jp)*Pp*2.0-(Ap*Ap)*Jp*V0+(Ap*Ap*Ap)*Jp*tj*(1.0/2.0)-Ap*(Jp*Jp)*V0*tj)-Jp*V0-Ap*Jp*tj*(3.0/2.0))/(Ap*Jp)) );
        }
        t2_out = Ap/Jp - tj;
    }
    t6_out = t2_out;
}

bool scurves::runme(float t, float& Jt_out, float& At_out, float& Vt_out, float& Pt_out)
{
    jtype_t Jtype;
    int8_t pnt = num_items;
    float Jp, T0, A0, V0, P0;

    for (uint8_t i = 0; i < num_items; i++) {
        if (t < oT[num_items-1-i]){
            pnt = num_items-1-i;
        }
    }
    if (pnt == 0) {
        Jtype = JTYPE_CONSTANT;
        Jp = 0.0f;
        T0 = oT[pnt];
        A0 = oA[pnt];
        V0 = oV[pnt];
        P0 = oP[pnt];
    } else if (pnt == num_items) {
        Jtype = JTYPE_CONSTANT;
        Jp = 0.0;
        T0 = oT[pnt-1];
        A0 = oA[pnt-1];
        V0 = oV[pnt-1];
        P0 = oP[pnt-1];
    } else {
        Jtype = oJtype[pnt];
        Jp = oJ[pnt];
        T0 = oT[pnt-1];
        A0 = oA[pnt-1];
        V0 = oV[pnt-1];
        P0 = oP[pnt-1];
    }

    switch (Jtype) {
        case JTYPE_POSITIVE:
            JSegment1(t-T0, Jp, A0, V0, P0, Jt_out, At_out, Vt_out, Pt_out);
            break;
        case JTYPE_NEGATIVE:
            JSegment2(t-T0, Jp, A0, V0, P0, Jt_out, At_out, Vt_out, Pt_out);
            break;
        default:
            JConst(t-T0, Jp, A0, V0, P0, Jt_out, At_out, Vt_out, Pt_out);
            break;
    }
//    timer++;
//    if(timer == 40 ){
//        timer = 0;
//        hal.console->printf("%4.2f, %4.2f, %4.2f, %4.2f, %4.2f, %4.2u\n", t, Jt_out, At_out, Vt_out, Pt_out, pnt);
//    }

    return pnt == num_items;
}

void scurves::JConst(float t, float J0, float A0, float V0, float P0, float& Jt, float& At, float& Vt, float& Pt)
{
    Jt = J0;
    At = A0 + J0*t;
    Vt = V0 + A0*t + 0.5*J0*(t*t);
    Pt = P0 + V0*t + 0.5*A0*(t*t) + (1/6.0)*J0*(t*t*t);
}

void scurves::JSegment1(float t, float Jp, float A0, float V0, float P0, float& Jt, float& At, float& Vt, float& Pt)
{
    float tj = otj;
    float Alpha = Jp/2.0;
    float Beta = M_PI/tj;
    Jt = Alpha*(1.0 - cosf(Beta*t));
    At = A0 + Alpha*t - (Alpha/Beta)*sinf(Beta*t);
    Vt = V0 + A0*t + (Alpha/2.0)*(t*t) + (Alpha/(Beta*Beta))*cosf(Beta*t) - Alpha/(Beta*Beta);
    Pt = P0 + V0*t + 0.5*A0*(t*t) + (-Alpha/(Beta*Beta))*t + Alpha*(t*t*t)/6.0 + (Alpha/(Beta*Beta*Beta))*sinf(Beta*t);
}

void scurves::JSegment2(float t, float Jp, float A0, float V0, float P0, float& Jt, float& At, float& Vt, float& Pt)
{
    float tj = otj;
    float Alpha = Jp/2.0;
    float Beta = M_PI/tj;
    float AT = Alpha*tj;
    float VT = Alpha*((tj*tj)/2.0 - 2.0/(Beta*Beta));
    float PT = Alpha*((-1.0/(Beta*Beta))*tj + (1/6.0)*(tj*tj*tj));
    Jt = Alpha*(1.0 - cosf(Beta * (t+tj)));
    At = (A0-AT) + Alpha*(t+tj) - (Alpha / Beta) * sinf(Beta * (t+tj));
    Vt = (V0-VT) + (A0-AT)*t + 0.5*Alpha*(t+tj)*(t+tj) + (Alpha/(Beta*Beta))*cosf(Beta*(t+tj)) - Alpha/(Beta*Beta);
    Pt = (P0-PT) + (V0-VT)*t + 0.5*(A0-AT)*(t*t) + (-Alpha/(Beta*Beta))*(t+tj) + (Alpha/6.0)*(t+tj)*(t+tj)*(t+tj) + (Alpha/(Beta*Beta*Beta))*sinf(Beta*(t+tj));
}
