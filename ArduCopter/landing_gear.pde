/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

// Run landing gear controller at 10Hz
static void landinggear_update(){

    // If landing gear control is active, run update function.
    if (g.ch7_option == AUX_SWITCH_LANDING_GEAR || g.ch8_option == AUX_SWITCH_LANDING_GEAR){

        // last status (deployed or retracted) used to check for changes
        static bool last_deploy_status;

        // if we are doing an automatic landing procedure, force the landing gear to deploy.
        // To-Do: should we pause the auto-land procedure to give time for gear to come down?
        if (control_mode == LAND || (control_mode==RTL && rtl_state == Land) || (control_mode == AUTO && auto_mode == Auto_Land)){
            landinggear.force_deploy(true);
        }            

        landinggear.update();

        // send event message to datalog if status has changed
        if (landinggear.deployed() != last_deploy_status){
            if (landinggear.deployed()) {
                Log_Write_Event(DATA_LANDING_GEAR_DEPLOYED);
            } else {
                Log_Write_Event(DATA_LANDING_GEAR_RETRACTED);
            }
        }

        last_deploy_status = landinggear.deployed();        
    }
}
