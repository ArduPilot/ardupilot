#include <AP_Param/AP_Param.h>

class RC_ModeSwitch
{

public:

    RC_ModeSwitch(AP_Int8 &_mode_channel)
        : mode_channel(_mode_channel)
    { }

    typedef uint8_t RC_ModeNum;

    // return false on failure
    bool readSwitch(RC_ModeNum &modenum) const;

private:

    AP_Int8 &mode_channel;

};
