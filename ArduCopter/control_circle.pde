/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

/*
 * control_circle.pde - init and run calls for circle flight mode
 */

// circle_init - initialise circle controller
static bool circle_init(bool ignore_checks)
{
    if ((GPS_ok() && inertial_nav.position_ok()) || ignore_checks) {
        circle_pilot_yaw_override = false;
        return true;
    }else{
        return false;
    }
}

// loiter_run - runs the loiter controller
// should be called at 100hz or more
static void circle_run()
{
    float target_yaw_rate = 0;
    float target_climb_rate = 0;

    // if not auto armed set throttle to zero and exit immediately
    if(!ap.auto_armed || ap.land_complete) {
        wp_nav.init_loiter_target();
        attitude_control.init_targets();
        attitude_control.set_throttle_out(0, false);
        return;
    }

    // process pilot inputs
    if (!failsafe.radio) {
        // get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(g.rc_4.control_in);
        if (target_yaw_rate != 0) {
            circle_pilot_yaw_override = true;
        }

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(g.rc_3.control_in);

        // check for pilot requested take-off
        if (ap.land_complete && target_climb_rate > 0) {
            // indicate we are taking off
            set_land_complete(false);
            // clear i term when we're taking off
            set_throttle_takeoff();
        }
    }

    // run circle controller
    circle_nav.update();

    // call attitude controller
    if (circle_pilot_yaw_override) {
        attitude_control.angleef_rp_rateef_y(circle_nav.get_roll(), circle_nav.get_pitch(), target_yaw_rate);
    }else{
        attitude_control.angleef_rpy(circle_nav.get_roll(), circle_nav.get_pitch(), circle_nav.get_yaw());
    }

    // run altitude controller
    if (sonar_alt_health >= SONAR_ALT_HEALTH_MAX) {
        // if sonar is ok, use surface tracking
        target_climb_rate = get_throttle_surface_tracking(target_climb_rate,G_Dt);
    }
    // update altitude target and call position controller
    pos_control.set_alt_target_from_climb_rate(target_climb_rate, G_Dt);
    pos_control.update_z_controller();

    // re-fetch angle targets for reporting
    const Vector3f angle_target = attitude_control.angle_ef_targets();
    control_roll = angle_target.x;
    control_pitch = angle_target.y;
    control_yaw = angle_target.z;
}

//////////////////////////////////////////////////////////
// circle navigation controller
//////////////////////////////////////////////////////////
/*
// circle_set_center -- set circle controller's center position and starting angle
static void
circle_set_center(const Vector3f current_position, float heading_in_radians)
{
    float max_velocity;
    float cir_radius = g.circle_radius * 100;

    // set circle center to circle_radius ahead of current position
    circle_center.x = current_position.x + cir_radius * cos_yaw;
    circle_center.y = current_position.y + cir_radius * sin_yaw;

    // if we are doing a panorama set the circle_angle to the current heading
    if( g.circle_radius <= 0 ) {
        circle_angle = heading_in_radians;
        circle_angular_velocity_max = ToRad(g.circle_rate);
        circle_angular_acceleration = circle_angular_velocity_max;  // reach maximum yaw velocity in 1 second
    }else{
        // set starting angle to current heading - 180 degrees
        circle_angle = wrap_PI(heading_in_radians-PI);

        // calculate max velocity based on waypoint speed ensuring we do not use more than half our max acceleration for accelerating towards the center of the circle
        max_velocity = min(wp_nav.get_horizontal_velocity(), safe_sqrt(0.5f*wp_nav.get_wp_acceleration()*g.circle_radius*100.0f));

        // angular_velocity in radians per second
        circle_angular_velocity_max = max_velocity/((float)g.circle_radius * 100.0f);
        circle_angular_velocity_max = constrain_float(ToRad(g.circle_rate),-circle_angular_velocity_max,circle_angular_velocity_max);

        // angular_velocity in radians per second
        circle_angular_acceleration = wp_nav.get_wp_acceleration()/((float)g.circle_radius * 100);
        if (g.circle_rate < 0.0f) {
            circle_angular_acceleration = -circle_angular_acceleration;
        }
    }

    // initialise other variables
    circle_angle_total = 0;
    circle_angular_velocity = 0;

    // initialise loiter target.  Note: feed forward velocity set to zero
    // To-Do: modify circle to use position controller and pass in zero velocity.  Vector3f(0,0,0)
    wp_nav.init_loiter_target();
}

// update_circle - circle position controller's main call which in turn calls loiter controller with updated target position
static void
update_circle()
{
    static float last_update;    // time of last circle call

    // calculate dt
    uint32_t now = hal.scheduler->millis();
    float dt = (now - last_update) / 1000.0f;

    // ensure enough time has passed since the last iteration
    if (dt >= 0.095f) {
        float cir_radius = g.circle_radius * 100;
        Vector3f circle_target;

        // range check dt
        if (dt >= 1.0f) {
            dt = 0;
        }

        // update time of circle call
        last_update = now;

        // ramp up angular velocity to maximum
        if (g.circle_rate >= 0) {
            if (circle_angular_velocity < circle_angular_velocity_max) {
                circle_angular_velocity += circle_angular_acceleration * dt;
                circle_angular_velocity = constrain_float(circle_angular_velocity, 0, circle_angular_velocity_max);
            }
        }else{
            if (circle_angular_velocity > circle_angular_velocity_max) {
                circle_angular_velocity += circle_angular_acceleration * dt;
                circle_angular_velocity = constrain_float(circle_angular_velocity, circle_angular_velocity_max, 0);
            }
        }

        // update the target angle
        circle_angle += circle_angular_velocity * dt;
        circle_angle = wrap_PI(circle_angle);

        // update the total angle travelled
        circle_angle_total += circle_angular_velocity * dt;

        // if the circle_radius is zero we are doing panorama so no need to update loiter target
        if( g.circle_radius != 0.0 ) {
            // calculate target position
            circle_target.x = circle_center.x + cir_radius * cosf(-circle_angle);
            circle_target.y = circle_center.y - cir_radius * sinf(-circle_angle);
            circle_target.z = wp_nav.get_desired_alt();

            // re-use loiter position controller
            wp_nav.set_loiter_target(circle_target);
        }
    }

    // call loiter controller
    wp_nav.update_loiter();
}

// get_look_at_yaw - updates bearing to look at center of circle or do a panorama
// should be called at 100hz
static void get_circle_yaw()
{
   static uint8_t look_at_yaw_counter = 0;     // used to reduce update rate to 10hz

   // if circle radius is zero do panorama
   if( g.circle_radius == 0 ) {
       // slew yaw towards circle angle
       control_yaw = get_yaw_slew(control_yaw, ToDeg(circle_angle)*100, AUTO_YAW_SLEW_RATE);
   }else{
       look_at_yaw_counter++;
       if( look_at_yaw_counter >= 10 ) {
           look_at_yaw_counter = 0;
           yaw_look_at_WP_bearing = pv_get_bearing_cd(inertial_nav.get_position(), yaw_look_at_WP);
       }
       // slew yaw
       control_yaw = get_yaw_slew(control_yaw, yaw_look_at_WP_bearing, AUTO_YAW_SLEW_RATE);
   }

   // call stabilize yaw controller
   get_stabilize_yaw(control_yaw);
}
*/
