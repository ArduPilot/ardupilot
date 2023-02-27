#ifndef ARDUPILOT_CLIENT_HPP_
#define ARDUPILOT_CLIENT_HPP_

#include "client_communication.hpp"

class ArdupilotClient : public ClientAbstract {
    public:
        ArdupilotClient(uint8_t obj_idn):
            ClientAbstract(      0, obj_idn),
            drive_spin_pwm_(    50, obj_idn, kSubDriveSpinPwm),
            obs_velocity_(      50, obj_idn, kSubObsVelocity),
            volts_(             69, obj_idn, kSubVolts),
            amps_(              69, obj_idn, kSubAmps),
            joules_(            69, obj_idn, kSubJoules),
            uc_temp_(           73, obj_idn, kSubUcTemp),
            temp_(              77, obj_idn, kSubTemp),
            time_(               5, obj_idn, kSubTime)
            {};

        // Client Entries
        ClientEntry<float>      drive_spin_pwm_; // 50, 4
        ClientEntry<float>      obs_velocity_; // 50, 13
        ClientEntry<float>      volts_; // 69, 0
        ClientEntry<float>      amps_; // 69, 1
        ClientEntry<float>      joules_; // 69, 3
        ClientEntry<float>      uc_temp_; // 73, 0
        ClientEntry<float>      temp_; // 77, 0
        ClientEntry<float>      time_; // 5, 15 

        void ReadMsg(uint8_t * rx_data, uint8_t rx_length) override
        {
            static const uint8_t kEntryLength = 8;
            ClientEntryAbstract* entry_array[8] = {
                &drive_spin_pwm_,
                &obs_velocity_,
                &volts_,
                &amps_,
                &joules_,
                &uc_temp_,
                &temp_,
                &time_
            };
            ParseMsgLoop(rx_data, rx_length, entry_array, kEntryLength);
        }
        
    
    private:
        static const uint8_t kSubDriveSpinPwm   =  3;
        static const uint8_t kSubObsVelocity    = 13;
        static const uint8_t kSubVolts          =  0;
        static const uint8_t kSubAmps           =  1;
        static const uint8_t kSubJoules         =  3;
        static const uint8_t kSubUcTemp         =  0;
        static const uint8_t kSubTemp           =  0;
        static const uint8_t kSubTime           = 15;

        uint8_t ParseMsgLoop(uint8_t* rx_data, uint8_t rx_length, ClientEntryAbstract** entry_array, uint8_t entry_length)
        {
            uint8_t type_idn = rx_data[0];
            uint8_t sub_idn = rx_data[1];
            uint8_t obj_idn = rx_data[2] >> 2; // high 6 bits are obj_idn
            Access dir = static_cast<Access>(rx_data[2] & 0b00000011); // low two bits
            if (dir == kReply)
            {
                if (obj_idn == obj_idn_)
                {
                    for (uint8_t ii = 0; ii < entry_length; ++ii)
                    {
                        if (entry_array[ii] != nullptr && entry_array[ii]->sub_idn_ == sub_idn && entry_array[ii]->type_idn_ == type_idn)
                        {
                            entry_array[ii]->Reply(&rx_data[3],rx_length-3);
                            return 1;
                        }
                    }
                }
            }
            return 0;
        }


};

#endif /* ARDUPILOT_CLIENT_HPP_ */