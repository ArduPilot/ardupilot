#pragma once

#include "AP_ESC_Telem_Backend.h"

#if AP_ESC_TELEM_DDS_ENABLED

class AP_ESC_Telem_DDS : public AP_ESC_Telem_Backend
{
public:
    AP_ESC_Telem_DDS();

    void handle_external(/*todo_add_data_type*/) override;

    void update();
};

#endif  // AP_ESC_TELEM_DDS_ENABLED
