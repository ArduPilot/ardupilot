#pragma once

#include <inttypes.h>
#include <AP_Common/AP_Common.h>
#include <AP_Param/AP_Param.h>

class RCMixer
{
public:
    /// Constructor
    ///
    RCMixer();
    void request_mixer_data(uint16_t group, uint16_t mixer, uint16_t submixer, uint16_t type);

private:
};
