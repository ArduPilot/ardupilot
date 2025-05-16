#include "CiS_Modell.h"
#include <AP_Vehicle/AP_Vehicle.h>

CiS_Modell::CiS_Modell(){}

CiS_Modell& CiS_Modell::get_instance() {
    static CiS_Modell instance;
    return instance;
}

void CiS_Modell::init() {}

void CiS_Modell::update(){
    static bool last_state = false;
    bool current = AP::vehicle()->get_cis_param().greeting.get();

    if (current && !last_state) {
        //ِactive
        gcs().send_text(MAV_SEVERITY_INFO, "CiS_Modell is Active");
    }
    last_state = current;
}

void CiS_Modell::loop() {}
