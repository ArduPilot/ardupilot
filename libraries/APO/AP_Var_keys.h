#ifndef AP_Var_keys_H
#define AP_Var_keys_H

enum keys {

    // general
    k_config = 0,
    k_cntrl,
    k_guide,
    k_sensorCalib,
    k_nav,

    k_radioChannelsStart=10,

    k_controllersStart=30,

    k_customStart=100,

    // 200-256 reserved for commands
    k_commands = 200
};

// max 256 keys

#endif
// vim:ts=4:sw=4:expandtab
