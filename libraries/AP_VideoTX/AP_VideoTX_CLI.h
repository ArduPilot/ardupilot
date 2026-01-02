#pragma once 

#include <cstring>
#include <GCS_MAVLink/GCS.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <RC_Channel/RC_Channel.h>

#define VTX_CLI_NUM_COMMANDS 10

class VTX_CLI_Command {
public:
    AP_Int8  rc_channel;
    AP_Int16 pwm_begin;
    AP_Int16 pwm_end;
    AP_Int8  band;
    AP_Int8  channel;
    AP_Int8  power;

    static const struct AP_Param::GroupInfo var_info[];

    bool in_range(uint16_t pwm) const {
        return (pwm >= pwm_begin) && (pwm < pwm_end);
    }

    bool is_active() const {
        if (rc_channel < 1) {
            return false;
        }

        RC_Channels* rc = RC_Channels::get_singleton();
        if (rc == nullptr) {
            return false;
        }

        RC_Channel* ch = rc->channel((uint8_t)rc_channel.get()-1);
        if (ch == nullptr) {
            return false;
        }

        int16_t ri = ch->get_radio_in();
        if(ri > 0) {
            return in_range((uint16_t)ri);
        }

        return false;
    }

    bool is_configured() const {
        return rc_channel >= 0 && pwm_begin > 0 && pwm_end > 0;
    }
};

class AP_VideoTX_CLI 
{
    public:
        AP_VideoTX_CLI(/* args */);

        CLASS_NO_COPY(AP_VideoTX_CLI);

        static const struct AP_Param::GroupInfo var_info[];

        static AP_VideoTX_CLI* _singleton;
        static AP_VideoTX_CLI* get_singleton()
        {
            return _singleton;
        }

        VTX_CLI_Command commands[VTX_CLI_NUM_COMMANDS];

        void handle_commands();
    private:
        bool vtxCliOptionsInitialized;
};