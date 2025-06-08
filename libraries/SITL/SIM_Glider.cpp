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
/*
  glider model for high altitude balloon drop

  controls:
    - servo6: balloon lift, 1000 for no lift, 2000 for maximum lift
    - servo10: balloon cut, this cuts away the balloon when high

  Note that the glider starts off in a lifted by tail pose, with pitch
  -80 degrees. The balloon then lifts the glider above the ground. The
  balloon automatically bursts at a height of SIM_GLD_BLN_BRST meters,
  or can be cut away early with servo10.

  The maximum rate of the balloon lift is in SIM_GLD_BLN_RATE, in m/s

  To perform a takeoff first arm on the ground then use
    servo set 6 2000
  to release the ground hold. Use this to cut away the balloon:
    servo set 10 2000

  For an automatic mission, NAV_ALTITUDE_WAIT should be used to wait
  for a desired altitude under balloon lift. Then a DO_SET_SERVO with
  servo 10 and a value of 2000 to cut away the balloon.
 */

#include "SIM_Glider.h"

#if AP_SIM_GLIDER_ENABLED

#include <stdio.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_AHRS/AP_AHRS.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

using namespace SITL;

// SITL glider parameters
const AP_Param::GroupInfo Glider::var_info[] = {
    // @Param: BLN_BRST
    // @DisplayName: balloon burst height
    // @Description: balloon burst height
    // @Units: m
    AP_GROUPINFO("BLN_BRST", 1, Glider, balloon_burst_amsl, 30000),

    // @Param: BLN_RATE
    // @DisplayName: balloon climb rate
    // @Description: balloon climb rate. If the value is less than zero then the balloon is disabled.
    // @Units: m/s
    AP_GROUPINFO("BLN_RATE",  2, Glider, balloon_rate, 5.5),

    AP_GROUPEND
};

Glider::Glider(const char *frame_str) :
    Aircraft(frame_str)
{
    ground_behavior = GROUND_BEHAVIOR_NO_MOVEMENT;
    carriage_state = carriageState::WAITING_FOR_PICKUP;
    AP::sitl()->models.glider_ptr = this;
    AP_Param::setup_object_defaults(this, var_info);
}


// Torque calculation function
Vector3f Glider::getTorque(float inputAileron, float inputElevator, float inputRudder, const Vector3f &force) const
{
    // Calculate dynamic pressure
    const auto &m = model;
    double qPa = 0.5*air_density*sq(velocity_air_bf.length());
    const float aileron_rad = inputAileron * radians(m.aileronDeflectionLimitDeg);
    const float elevator_rad = inputElevator * radians(m.elevatorDeflectionLimitDeg);
    const float rudder_rad = inputRudder * radians(m.rudderDeflectionLimitDeg);
    const float tas = MAX(airspeed * AP::ahrs().get_EAS2TAS(), 1);

    float Cl = (m.Cl2 * sq(alpharad) + m.Cl1 * alpharad + m.Cl0) * betarad;
    float Cm = m.Cm2 * sq(alpharad) + m.Cm1 * alpharad + m.Cm0;
    float Cn = (m.Cn2 * sq(alpharad) + m.Cn1 * alpharad + m.Cn0) * betarad;

    Cl += m.deltaClperRadianElev * elevator_rad;
    Cm += m.deltaCmperRadianElev * elevator_rad;
    Cn += m.deltaCnperRadianElev * elevator_rad;

    Cl += m.deltaClperRadianRud * rudder_rad;
    Cm += m.deltaCmperRadianRud * rudder_rad;
    Cn += m.deltaCnperRadianRud * rudder_rad;

    Cl += (m.deltaClperRadianAil2 * sq(alpharad) + m.deltaClperRadianAil1 * alpharad + m.deltaClperRadianAil0) * aileron_rad;
    Cm += m.deltaCmperRadianAil * aileron_rad;
    Cn += (m.deltaCnperRadianAil2 * sq(alpharad) + m.deltaCnperRadianAil1 * alpharad + m.deltaCnperRadianAil0) * aileron_rad;

    // derivatives
    float Clp = m.Clp2 * sq(alpharad) + m.Clp1 * alpharad + m.Clp0;
    float Clr = m.Clr2 * sq(alpharad) + m.Clr1 * alpharad + m.Clr0;
    float Cnp = m.Cnp2 * sq(alpharad) + m.Cnp1 * alpharad + m.Cnp0;
    float Cnr = m.Cnr2 * sq(alpharad) + m.Cnr1 * alpharad + m.Cnr0;

    // normalise gyro rates
    Vector3f pqr_norm = gyro;
    pqr_norm.x *= 0.5 * m.refSpan / tas;
    pqr_norm.y *= 0.5 * m.refChord / tas;
    pqr_norm.z *= 0.5 * m.refSpan / tas;

    Cl += pqr_norm.x * Clp;
    Cl += pqr_norm.z * Clr;
    Cn += pqr_norm.x * Cnp;
    Cn += pqr_norm.z * Cnr;

    Cm += pqr_norm.y * m.Cmq;

    float Mx = Cl * qPa * m.Sref * m.refSpan;
    float My = Cm * qPa * m.Sref * m.refChord;
    float Mz = Cn * qPa * m.Sref * m.refSpan;


#if 0
    // @LoggerMessage: GLT
    // @Description: Simulated Glider Angles and coefficients
    // @Field: TimeUS: Time since system startup
    // @Field: Alpha: alpha angle
    // @Field: Beta: beta angle
    // @Field: Cl: lift coefficent
    // @Field: Cm: roll coffecient
    // @Field: Cn: yaw coefficient
    AP::logger().Write("GLT", "TimeUS,Alpha,Beta,Cl,Cm,Cn", "Qfffff",
                       AP_HAL::micros64(),
                       degrees(alpharad),
                       degrees(betarad),
                       Cl, Cm, Cn);
#endif

    return Vector3f(Mx/m.IXX, My/m.IYY, Mz/m.IZZ);
}

// Force calculation, return vector in Newtons
Vector3f Glider::getForce(float inputAileron, float inputElevator, float inputRudder)
{
    const auto &m = model;
    const float aileron_rad = inputAileron * radians(m.aileronDeflectionLimitDeg);
    const float elevator_rad = inputElevator * radians(m.elevatorDeflectionLimitDeg);
    const float rudder_rad = inputRudder * radians(m.rudderDeflectionLimitDeg);

    // dynamic pressure
    double qPa = 0.5*air_density*sq(velocity_air_bf.length());

    float CA = m.CA2 * sq(alpharad) + m.CA1 * alpharad + m.CA0;
    float CY = (m.CY2 * sq(alpharad) + m.CY1 * alpharad + m.CY0) * betarad;
    float CN = m.CN2 * sq(alpharad) + m.CN1 * alpharad + m.CN0;

    CN += m.deltaCNperRadianElev * elevator_rad;
    CA += m.deltaCAperRadianElev * elevator_rad;
    CY += m.deltaCYperRadianElev * elevator_rad;

    CN += m.deltaCNperRadianRud * rudder_rad;
    CA += m.deltaCAperRadianRud * rudder_rad;
    CY += m.deltaCYperRadianRud * rudder_rad;

    CN += m.deltaCNperRadianAil * aileron_rad;
    CA += m.deltaCAperRadianAil * aileron_rad;
    CY += m.deltaCYperRadianAil * aileron_rad;
    
    float Fx = -CA * qPa * m.Sref;
    float Fy =  CY * qPa * m.Sref;
    float Fz = -CN * qPa * m.Sref;

    Vector3f ret = Vector3f(Fx, Fy, Fz);

    float Flift = Fx * sin(alpharad)  - Fz * cos(alpharad);
    float Fdrag = -Fx * cos(alpharad) - Fz * sin(alpharad);

    if (carriage_state == carriageState::RELEASED) {
        uint32_t now = AP_HAL::millis();
        sim_LD = 0.1 * constrain_float(Flift/MAX(1.0e-6,Fdrag),0,20) + 0.9 * sim_LD;
        if (now - last_drag_ms > 10 &&
            airspeed > 1) {
            last_drag_ms = now;
#if HAL_LOGGING_ENABLED
            // @LoggerMessage: SLD
            // @Description: Simulated Glider Dropped Calculations
            // @Field: TimeUS: Time since system startup
            // @Field: AltFt: altitude in feet
            // @Field: AltM: altitude in metres
            // @Field: EAS: equivalent airspeed
            // @Field: TAS: true airspeed
            // @Field: AD: air density
            // @Field: Fl: lift
            // @Field: Fd: drag
            // @Field: LD: lift/drag ratio
            // @Field: Elev: elevator output
            // @Field: AoA: angle of attack
            // @Field: Fx: X-axis force
            // @Field: Fy: Y-axis force
            // @Field: Fz: Z-axis force
            // @Field: q: air pressure
            AP::logger().Write("SLD", "TimeUS,AltFt,AltM,EAS,TAS,AD,Fl,Fd,LD,Elev,AoA,Fx,Fy,Fz,q", "Qffffffffffffff",
                               AP_HAL::micros64(),
                               (location.alt*0.01)/FEET_TO_METERS,
                               location.alt*0.01,
                               velocity_air_bf.length()/eas2tas,
                               velocity_air_bf.length(),
                               air_density,
                               Flift, Fdrag, sim_LD,
                               degrees(elevator_rad),
                               degrees(alpharad),
                               Fx, Fy, Fz,
                               qPa);
            // @LoggerMessage: SL2
            // @Description: More Simulated Glider Dropped Calculations
            // @Field: TimeUS: Time since system startup
            // @Field: AltFt: altitude in feet
            // @Field: KEAS: equivalent airspeed in knots
            // @Field: KTAS: true airspeed in knots
            // @Field: AD: air density
            // @Field: Fl: lift
            // @Field: Fd: drag
            // @Field: LD: lift/drag ratio
            // @Field: Elev: elevator output
            // @Field: Ail: aileron output
            // @Field: Rud: rudder output
            // @Field: AoA: Angle of Attack
            // @Field: SSA: Side Slip Angle
            // @Field: q: air pressire
            // @Field: Az: z-axis body-frame acceleration
            AP::logger().Write("SL2", "TimeUS,AltFt,KEAS,KTAS,AD,Fl,Fd,LD,Elev,Ail,Rud,AoA,SSA,q,Az", "Qffffffffffffff",
                               AP_HAL::micros64(),
                               (location.alt*0.01)/FEET_TO_METERS,
                               M_PER_SEC_TO_KNOTS*velocity_air_bf.length()/eas2tas,
                               M_PER_SEC_TO_KNOTS*velocity_air_bf.length(),
                               air_density,
                               Flift, Fdrag, sim_LD,
                               degrees(elevator_rad),
                               degrees(aileron_rad),
                               degrees(rudder_rad),
                               degrees(alpharad),
                               degrees(betarad),
                               qPa,
                               accel_body.z);

            // @LoggerMessage: SCTL
            // @Description: Simulated Glider Drop control outputs
            // @Field: TimeUS: Time since system startup
            // @Field: Ail: aileron output
            // @Field: Elev: elevator output
            // @Field: Rudd: rudder output
            AP::logger().Write("SCTL", "TimeUS,Ail,Elev,Rudd", "Qfff",
                               AP_HAL::micros64(),
                               degrees(aileron_rad),
                               degrees(elevator_rad),
                               degrees(rudder_rad));
#endif // HAL_LOGGING_ENABLED
        }
    }


    return ret;
}

void Glider::calculate_forces(const struct sitl_input &input, Vector3f &rot_accel, Vector3f &body_accel)
{
    filtered_servo_setup(1, 1100, 1900, model.aileronDeflectionLimitDeg);
    filtered_servo_setup(4, 1100, 1900, model.aileronDeflectionLimitDeg);
    filtered_servo_setup(2, 1100, 1900, model.elevatorDeflectionLimitDeg);
    filtered_servo_setup(3, 1100, 1900, model.rudderDeflectionLimitDeg);
    
    float aileron  = 0.5*(filtered_servo_angle(input, 1) + filtered_servo_angle(input, 4));
    float elevator = filtered_servo_angle(input, 2);
    float rudder   = filtered_servo_angle(input, 3);
    float balloon  = MAX(0.0f, filtered_servo_range(input, 5)); // Don't let the balloon receive downwards commands.
    float balloon_cut = filtered_servo_range(input, 9);

    // Move balloon upwards using balloon velocity from channel 6
    // Aircraft is released from ground constraint when channel 6 PWM > 1010
    // Once released, plane will be dropped when balloon_burst_amsl is reached or channel 6 is set to PWM 1000
    if (carriage_state == carriageState::WAITING_FOR_RELEASE) {
        balloon_velocity = Vector3f(-wind_ef.x, -wind_ef.y, -wind_ef.z -balloon_rate * balloon);
        balloon_position += balloon_velocity * (1.0e-6 * (float)frame_time_us);
        const float height_AMSL = 0.01f * (float)home.alt - position.z;
        // release at burst height or when balloon cut output goes high
        if (hal.scheduler->is_system_initialized() &&
            (height_AMSL > balloon_burst_amsl || balloon_cut > 0.8)) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "pre-release at %i m AMSL", (int)height_AMSL);
            carriage_state = carriageState::PRE_RELEASE;
        }
    } else if (carriage_state == carriageState::PRE_RELEASE) {
        // slow down for release
        balloon_velocity *= 0.999;
        balloon_position += balloon_velocity * (1.0e-6 * (float)frame_time_us);
        if (balloon_velocity.length() < 0.5) {
            carriage_state = carriageState::RELEASED;
            use_smoothing = false;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "released at %.0f m AMSL", (0.01f * home.alt) - position.z);
        }
    } else if (carriage_state == carriageState::WAITING_FOR_PICKUP) {
        // Don't allow the balloon to drag sideways until the pickup
        balloon_velocity = Vector3f(0.0f, 0.0f, -balloon_rate * balloon);
        balloon_position += balloon_velocity * (1.0e-6 * (float)frame_time_us);
    }

    // calculate angle of attack
    alpharad = atan2f(velocity_air_bf.z, velocity_air_bf.x);
    betarad = atan2f(velocity_air_bf.y,velocity_air_bf.x);

    alpharad = constrain_float(alpharad, -model.alphaRadMax, model.alphaRadMax);
    betarad = constrain_float(betarad, -model.betaRadMax, model.betaRadMax);

    Vector3f force;

    if (!update_balloon(balloon, force, rot_accel)) {
        force = getForce(aileron, elevator, rudder);
        rot_accel = getTorque(aileron, elevator, rudder, force);
    }

    accel_body = force / model.mass;

    if (on_ground()) {
        // add some ground friction
        Vector3f vel_body = dcm.transposed() * velocity_ef;
        accel_body.x -= vel_body.x * 0.3f;
    }

    // constrain accelerations
    accel_body.x = constrain_float(accel_body.x, -16*GRAVITY_MSS, 16*GRAVITY_MSS);
    accel_body.y = constrain_float(accel_body.y, -16*GRAVITY_MSS, 16*GRAVITY_MSS);
    accel_body.z = constrain_float(accel_body.z, -16*GRAVITY_MSS, 16*GRAVITY_MSS);

}
    
/*
  update the plane simulation by one time step
 */
void Glider::update(const struct sitl_input &input)
{
    Vector3f rot_accel;

    update_wind(input);
    
    calculate_forces(input, rot_accel, accel_body);
    
    if (carriage_state == carriageState::WAITING_FOR_PICKUP) {
        // Handle special case where plane is being held nose down waiting to be lifted
        accel_body = dcm.transposed() * Vector3f(0.0f, 0.0f, -GRAVITY_MSS);
        velocity_ef.zero();
        gyro.zero();
        dcm.from_euler(0.0f, radians(-80.0f), radians(home_yaw));
        use_smoothing = true;
        adjust_frame_time(constrain_float(sitl->loop_rate_hz, rate_hz-1, rate_hz+1));
    } else {
        update_dynamics(rot_accel);
    }
    update_external_payload(input);

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}

/*
   return true if we are on the ground
*/
bool Glider::on_ground() const
{
    switch (carriage_state) {
    case carriageState::NONE:
    case carriageState::RELEASED:
        return hagl() <= 0.001;
    case carriageState::WAITING_FOR_PICKUP:
    case carriageState::WAITING_FOR_RELEASE:
    case carriageState::PRE_RELEASE:
        // prevent bouncing around ground
        // don't do ground interaction if being carried
        break;
    }
    return false;
}

/*
  implement balloon lift
  controlled by SIM_BLN_BRST and SIM_BLN_RATE
 */
bool Glider::update_balloon(float balloon, Vector3f &force, Vector3f &rot_accel)
{
    // by setting a negative balloon rate we disable the balloon
    if (balloon_rate < 0) {
        carriage_state = carriageState::RELEASED;
    }
    
    if (!hal.util->get_soft_armed()) {
        return false;
    }

    switch (carriage_state) {
    case carriageState::NONE:
    case carriageState::RELEASED:
        // balloon not active
        disable_origin_movement = false;
        return false;

    case carriageState::WAITING_FOR_PICKUP:
    case carriageState::WAITING_FOR_RELEASE:
    case carriageState::PRE_RELEASE:
        // while under balloon disable origin movement to allow for balloon position calculations
        disable_origin_movement = true;
        break;
    }

    // assume a 50m tether with a 1Hz pogo frequency and damping ratio of 0.2
    Vector3f tether_pos_bf{-1.0f,0.0f,0.0f}; // tether attaches to vehicle tail approx 1m behind c.g.
    const float omega = model.tetherPogoFreq * M_2PI; // rad/sec
    const  float zeta = 0.7f;
    float tether_stiffness = model.mass * sq(omega); // N/m
    float tether_damping = 2.0f * zeta * omega / model.mass; // N/(m/s)
    // NED relative position vector from tether attachment on plane to balloon attachment
    Vector3f relative_position = balloon_position - (position.tofloat() + (dcm * tether_pos_bf));
    const float separation_distance = relative_position.length();

    // NED unit vector pointing from tether attachment on plane to attachment on balloon
    Vector3f tether_unit_vec_ef = relative_position.normalized();

    // NED velocity of attachment point on plane
    Vector3f attachment_velocity_ef = velocity_ef + dcm * (gyro % tether_pos_bf);

    // NED velocity of attachment point on balloon as seen by observer on attachemnt point on plane
    Vector3f relative_velocity = balloon_velocity - attachment_velocity_ef;

    float separation_speed = relative_velocity * tether_unit_vec_ef;

    // rate increase in separation between attachment point on plane and balloon
    // tension force in tether due to stiffness and damping
    float tension_force = MAX(0.0f, (separation_distance - model.tetherLength) * tether_stiffness);
    if (tension_force > 0.0f) {
        tension_force += constrain_float(separation_speed * tether_damping, 0.0f, 0.05f * tension_force);
    }

    if (carriage_state == carriageState::WAITING_FOR_PICKUP && tension_force > 1.2f * model.mass * GRAVITY_MSS && balloon > 0.01f) {
        carriage_state = carriageState::WAITING_FOR_RELEASE;
    }

    if (carriage_state == carriageState::WAITING_FOR_RELEASE ||
        carriage_state == carriageState::PRE_RELEASE) {
        Vector3f tension_force_vector_ef = tether_unit_vec_ef * tension_force;
        Vector3f tension_force_vector_bf = dcm.transposed() * tension_force_vector_ef;
        force = tension_force_vector_bf;

        // drag force due to lateral motion assuming projected area from Y is 20% of projected area seen from Z and
        // assuming bluff body drag characteristic. In reality we would need an aero model that worked flying backwards,
        // but this will have to do for now.
        Vector3f aero_force_bf = Vector3f(0.0f, 0.2f * velocity_air_bf.y * fabsf(velocity_air_bf.y), velocity_air_bf.z * fabsf(velocity_air_bf.z));
        aero_force_bf *= air_density * model.Sref;
        force -= aero_force_bf;

        Vector3f tension_moment_vector_bf = tether_pos_bf % tension_force_vector_bf;
        Vector3f tension_rot_accel = Vector3f(tension_moment_vector_bf.x/model.IXX, tension_moment_vector_bf.y/model.IYY, tension_moment_vector_bf.z/model.IZZ);
        rot_accel = tension_rot_accel;

        // add some rotation damping due to air resistance assuming a 2 sec damping time constant at SL density
        // TODO model roll damping with more accuracy using Clp data for zero alpha as a first approximation
        rot_accel -= gyro * 0.5 * air_density;
    } else {
        // tether is either slack awaiting pickup or released
        rot_accel.zero();
        force = dcm.transposed() * Vector3f(0.0f, 0.0f, -GRAVITY_MSS * model.mass);
    }

    // balloon is active
    return true;
}

#endif // AP_SIM_GLIDER_ENABLED
