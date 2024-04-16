#include "mode.h"
#include "Plane.h"

bool ModePistTakip::_enter()
{

    gcs().send_text(MAV_SEVERITY_INFO, "Pist Takip Moduna Girildi.");

    return true;
   
}

void ModePistTakip::navigate()
{


}

void ModePistTakip::update()
{
    
}

void ModePistTakip::run()
{


}

void ModePistTakip::_exit()
{

gcs().send_text(MAV_SEVERITY_INFO, "Pist Takip Modundan Cikildi.");

}




