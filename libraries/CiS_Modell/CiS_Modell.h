#pragma once

#include <GCS_MAVLink/GCS.h>

#include "CiS_parameter.h"
#include "CiS_config.h"


class CiS_Modell {
public:
    static CiS_Modell& get_instance();

    void init();
    void update();
    void loop();

private:
    CiS_Modell();
    CiS_Modell(const CiS_Modell&) = delete;
    CiS_Modell& operator=(const CiS_Modell&) = delete;
};
