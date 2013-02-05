#include "AP_L1_Control.h"
#define RADIUS_OF_EARTH 6378100

Vector2f geo2planar(const Vector2f &ref, const Vector2f &wp)
{
    Vector2f out;

    out.x=radians((wp.x-ref.x));
    out.y=radians((wp.y-ref.y)*cos(radians(ref.x)));

    return out;
}

Vector2f planar2geo(const Vector2f &ref, const Vector2f &wp)
{
    Vector2f out;

    out.x=degrees(wp.x)+ref.x;
    out.y=degrees(wp.y*(1/cos(radians(ref.x))))+ref.y;

    return out;
}

void calc_L1_circ(  uint8_t                 L1,
                    const uint8_t                 turn_radius,
                    const struct Location & turn_center_loc,
                    const struct Location & current_loc,
                    struct Location &       L1_ref)
{
    int8_t side, dir=1;
    float beta, gamma, wp_distance;
    Vector2f q1, q2(0,1);
    Vector2f air((current_loc.lat/1.0e7), (current_loc.lng/1.0e7));
    Vector2f X(turn_center_loc.lat/1.0e7, turn_center_loc.lng/1.0e7);

    air=geo2planar(X, air)*RADIUS_OF_EARTH;
    wp_distance=sqrt(sq(air.x)+sq(air.y)); //distance from vehicle to circle center.

    q1=(-air).normalized(); //unit vector from vehicle to circle center.

    //Calculate which side of circle vehicle is located.
    if (q1.x > 0) {
        side=1;
    }else{
        side=-1;
    }

    if (wp_distance > (turn_radius+L1) || wp_distance < (turn_radius-L1)) {
        //Vehicle is far away from circle or very far within the circle
        gamma=acos(q1* -q2);
        L1_ref=turn_center_loc;
        location_offset(&L1_ref, turn_radius*sin(-side*gamma), turn_radius*cos(-side*gamma));
    } else {
        gamma=acos(q1*q2);
        L1_ref=current_loc;
        //beta is angle between vector from vehicle to circle center, and vehcile to L1_ref
        beta=dir* -acos (( sq(wp_distance) + sq(L1) - sq(turn_radius) ) / (2*wp_distance*L1));
        location_offset(&L1_ref, L1*sin(beta+side*gamma), L1*cos(beta+side*gamma));
    }
}

void calc_L1_line(  uint8_t                 L1,
                    const struct Location & A,
                    const struct Location & B,
                    const struct Location & current_loc,
                    struct Location &       L1_ref)
{
    float psi_seg, tmp_L1, p_dist, lambda, d1;
    Vector2f q1, q2, B_p;

    Vector2f air((current_loc.lat/1.0e7), (current_loc.lng/1.0e7));
    Vector2f A_v((A.lat/1.0e7), (A.lng/1.0e7));
    Vector2f B_v((B.lat/1.0e7), (B.lng/1.0e7));

    //Calculate angle from A to B
    //Note: this could be pulled out of the function for efficiency.
    B_p=geo2planar(A_v, B_v)*RADIUS_OF_EARTH;
    q2=(B_p).normalized();
    psi_seg=atan2(q2.x,q2.y);       //Angle from A to B

    air=geo2planar(A_v, air)*RADIUS_OF_EARTH;
    d1=sqrt(sq(air.x)+sq(air.y));   //distance from A to aircraft
    q1=(air).normalized();          //Unit vector from A to aircraft

    lambda=acos(q1*q2);             //Angle between A and aircraft

    if (fabs(lambda) >= 1.5708) {   //Aircraft is somewhere behind A
        L1=max(L1, d1);
    } else {
        tmp_L1=d1*sin(lambda);
        if(fabs(tmp_L1) >= L1) {    //Aircraft is farther than L1 from segment AB
            L1=tmp_L1*1.05;
        }
    }

    //p_dist is distance from A to L1_ref, as measured along segment AB
    //see:law of cosines
    p_dist=d1*cos(lambda) + sqrt( sq(L1)-sq(d1)*sq(sin(lambda)) );
    L1_ref=A;
    location_offset(&L1_ref, p_dist*sin(psi_seg), p_dist*cos(psi_seg));
}