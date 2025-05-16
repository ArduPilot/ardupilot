#include "AP_CiS_Modell.h"
#include <AP_Vehicle/AP_Vehicle.h>

CiS_Modell::CiS_Modell(){}

CiS_Modell& CiS_Modell::get_instance() {
    static CiS_Modell instance;
    return instance;
}

void CiS_Modell::init() {}

void CiS_Modell::update(){
 AP::vehicle()->get_cis_param().update();
}

void CiS_Modell::loop() {}
