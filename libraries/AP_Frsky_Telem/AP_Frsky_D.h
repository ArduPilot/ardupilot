#pragma once

#include "AP_Frsky_Backend.h"

class AP_Frsky_D : public AP_Frsky_Backend {

public:

    using AP_Frsky_Backend::AP_Frsky_Backend;

protected:

    void send() override;
    uint32_t initial_baud() const override { return 9600; }

private:

    struct {
        uint32_t last_200ms_frame;
        uint32_t last_1000ms_frame;
    } _D;

};
