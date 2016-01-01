/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-

#include "Copter.h"


// Run landing gear controller at 10Hz
void Copter::landinggear_update(){

    // If landing gear control is active, run update function.
    if (check_if_auxsw_mode_used(AUXSW_LANDING_GEAR)){

        // last status (deployed or retracted) used to check for changes
        static bool last_deploy_status;

        // if we are doing an automatic landing procedure, force the landing gear to deploy.
        // To-Do: should we pause the auto-land procedure to give time for gear to come down?
        if (control_mode == LAND ||
           (control_mode==RTL && (rtl_state == RTL_LoiterAtHome || rtl_state == RTL_Land || rtl_state == RTL_FinalDescent)) ||
           (control_mode == AUTO && auto_mode == Auto_Land) ||
           (control_mode == AUTO && auto_mode == Auto_RTL && (rtl_state == RTL_LoiterAtHome || rtl_state == RTL_Land || rtl_state == RTL_FinalDescent))) {
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
