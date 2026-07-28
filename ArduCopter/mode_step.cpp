#include "Copter.h"

bool ModeStep::init(bool ignore_checks)
{

    can_receive_cmd_xy = false;
    can_receive_cmd_z = false; 
    received_cmd_xy = false;
    received_cmd_z = false;
    gcs().send_text(MAV_SEVERITY_NOTICE,"Mode STEP");

    move_x = 0.0f;
    move_y = 0.0f;
    move_z = 0.0f;

    xy = 0;
    z = 0;

    move_start_ms = 0;

    step_m = g2.user_parameters.get_step_dist();

    current_loc_vec = pos_control->get_pos_estimate_NED_m();
    target_loc_vec.zero();
    stop_loc_vec = pos_control->get_pos_estimate_NED_m();

    // initialise horizontal speed, acceleration
    pos_control->NE_set_max_speed_accel_cm(wp_nav->get_default_speed_NE_cms(), wp_nav->get_wp_acceleration_cmss());
    pos_control->NE_set_correction_speed_accel_cm(wp_nav->get_default_speed_NE_cms(), wp_nav->get_wp_acceleration_cmss());

    // initialize vertical speeds and acceleration
    pos_control->D_set_max_speed_accel_cm(wp_nav->get_default_speed_down_cms(), wp_nav->get_default_speed_up_cms(), wp_nav->get_accel_D_cmss());
    pos_control->D_set_correction_speed_accel_cm(wp_nav->get_default_speed_down_cms(), wp_nav->get_default_speed_up_cms(), wp_nav->get_accel_D_cmss());

    // initialise velocity controller
    pos_control->D_init_controller();
    pos_control->NE_init_controller();

    // initialise yaw
    auto_yaw.set_mode_to_default(false);

    return true;
}

void ModeStep::run()
{
    // apply simple mode transform to pilot inputs

    current_loc_vec = pos_control->get_pos_estimate_NED_m();
    update_simple_mode();

    //set motors to full range
    motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

    //Appelle différentes fonctions selon le mode du drone
    switch(Step_state)
    {
        case SubMode::Waiting : {waiting();break;}
        case SubMode::Moving_xy : {moving_xy();break;}
        case SubMode::Moving_z : {moving_z();break;}
    }
    //Change le mode du drone
    if(received_cmd_xy){Step_state = SubMode::Moving_xy;}
    else if(received_cmd_z){Step_state = SubMode::Moving_z;}
    else{Step_state = SubMode::Waiting;}
}

void ModeStep::waiting()
{
    //enregistre l'état des joystics de la RC
    float pilot_roll = channel_roll->norm_input_dz(); //joystick gauche/droite
    float pilot_pitch = channel_pitch->norm_input_dz(); //joystick avant/arrière
    float pilot_throttle = channel_throttle->norm_input_dz(); //joystick haut/bas

    moving();

    if(can_receive_cmd_xy)//si le drone est prêt à recevoir une commande de déplacement sur le plan horizontal
    {
        if(fabsf(pilot_roll) >= 0.5f || fabsf(pilot_pitch) >= 0.5f)//on vérifie les joysticks
        {
            //if(pilot_roll > 0.25){move_x = step_m;} 
            //else if(pilot_roll < -0.25){move_x = -step_m;} 

            //if(pilot_pitch > 0.25){move_y = step_m;} 
            //else if(pilot_pitch < -0.25){move_y = -step_m;}

            if(pilot_roll > 0.25){move_y = step_m;} 
            else if(pilot_roll < -0.25){move_y = -step_m;} 

            if(pilot_pitch > 0.25){move_x = -step_m;} 
            else if(pilot_pitch < -0.25){move_x = step_m;} 
 
            received_cmd_xy = true;
            can_receive_cmd_xy = false;
            can_receive_cmd_z = false;
            start_loc_vec = current_loc_vec;//enregistre la position au moment ou on reçoit la commande de déplacement
        }
    }
    else if(is_zero(pilot_roll) && is_zero(pilot_pitch)){can_receive_cmd_xy = true;}//autorise le nouvel envoi d'une commande uniquement si les joysticks sont revenus à zéro

    if(can_receive_cmd_z)//si le drone est prêt à recevoir une commande de déplacement sur l'axe vertical
    {
        if(fabsf(pilot_throttle) >= 0.5f)
        {
            if(pilot_throttle > 0.0f){ move_z = -step_m;} //Axe z inversé
            else{move_z = step_m;}

            received_cmd_z = true;
            can_receive_cmd_z = false;
            start_loc_vec = current_loc_vec;//enregistre la position au moment ou on reçoit la commande de déplacement
        }
    }
    else if(is_zero(pilot_throttle)){can_receive_cmd_z = true;}//autorise le nouvel envoi d'une commande uniquement si le joystick est revenu à zéro
}

void ModeStep::moving_xy()
{
    if (xy == 0) 
    {
        target_loc_vec.x = start_loc_vec.x + move_x;
        target_loc_vec.y = start_loc_vec.y + move_y;
        target_loc_vec.z = start_loc_vec.z;
        move_x = 0.0f;
        move_y = 0.0f;
        xy++;
        move_start_ms = AP_HAL::millis();
    }
    
    moving();
    printf("current_loc_vec : x = %f, y = %f, z = %f \n", current_loc_vec.x, current_loc_vec.y, current_loc_vec.z);
    printf("target_loc_vec : x = %f, y = %f, z = %f \n", target_loc_vec.x, target_loc_vec.y, target_loc_vec.z);

    if(fabsf(current_loc_vec.x - target_loc_vec.x) < 0.01 && fabsf(current_loc_vec.y - target_loc_vec.y) < 0.02)
    {
        printf("Destination reached xy\n");
        received_cmd_xy = false;
        stop_loc_vec = pos_control->get_pos_estimate_NED_m();
        printf("Stop loc : x = %f, y = %f, z = %f \n", stop_loc_vec.x, stop_loc_vec.y, stop_loc_vec.z);
        xy = 0;
        move_start_ms = 0;
    }
    else if(AP_HAL::millis() - move_start_ms >= MOVE_TIMEOUT_MS)
    {
        printf("Destination not reached xy\n");
        received_cmd_xy = false;
        stop_loc_vec = pos_control->get_pos_estimate_NED_m();
        printf("Stop loc : x = %f, y = %f, z = %f \n", stop_loc_vec.x, stop_loc_vec.y, stop_loc_vec.z);
        xy = 0;
        move_start_ms = 0;
    }
}

void ModeStep::moving_z()
{
    if (z == 0) 
    {
        target_loc_vec.x = start_loc_vec.x;
        target_loc_vec.y = start_loc_vec.y;
        target_loc_vec.z = start_loc_vec.z + move_z;
        move_z = 0.0f;
        move_start_ms = AP_HAL::millis();
        z++;
    }

    moving();
    printf("current_loc_vec : x = %f, y = %f, z = %f \n", current_loc_vec.x, current_loc_vec.y, current_loc_vec.z);
    printf("target_loc_vec : x = %f, y = %f, z = %f \n", target_loc_vec.x, target_loc_vec.y, target_loc_vec.z);
    
    if(fabsf(current_loc_vec.z - target_loc_vec.z) < 0.02)
    {
        printf("Destination reached z\n");
        received_cmd_z = false;
        stop_loc_vec = pos_control->get_pos_estimate_NED_m();
        printf("Stop loc : x = %f, y = %f, z = %f \n", stop_loc_vec.x, stop_loc_vec.y, stop_loc_vec.z);
        z = 0;
        move_start_ms = 0;
    }
    else if(AP_HAL::millis() - move_start_ms >= MOVE_TIMEOUT_MS)
    {
        printf("Destination not reached z\n");
        received_cmd_z = false;
        stop_loc_vec = pos_control->get_pos_estimate_NED_m();
        printf("Stop loc : x = %f, y = %f, z = %f \n", stop_loc_vec.x, stop_loc_vec.y, stop_loc_vec.z);
        z = 0;
        move_start_ms = 0;
    }
}

void ModeStep::moving()
{
    //calcul du décalage entre la position du drone et celle voulu
    if (Step_state == SubMode::Waiting)
    //{pos_control->input_pos_NED_m(stop_loc_vec,0.0f,copter.wp_nav->get_terrain_margin_m());}//Objectif : sur-place
    {copter.mode_stabilize.run();}//Objectif : sur-place
    else 
    {pos_control->input_pos_NED_m(target_loc_vec,0.0f,copter.wp_nav->get_terrain_margin_m());}//Objectif : se déplacer vers la position voulu
    //calcul de comment aller à la position voulu
    pos_control->NE_update_controller();
    pos_control->D_update_controller();
    //correction automatique pour retourner à la position voulu
    attitude_control->input_thrust_vector_rate_heading_rads(pos_control->get_thrust_vector(),get_pilot_desired_yaw_rate_rads());
}