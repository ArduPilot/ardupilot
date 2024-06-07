#pragma once

// This structure provides information on the internal member data of
// a PID.  It provides an abstract way to pass PID information around,
// useful for logging and sending mavlink messages.

// It is also used to pass PID information into controllers...

struct AP_PIDInfo {
    float target;
    float actual;
    float error;
    float P;
    float I;
    float D;
    float FF;
    float DFF;
    float Dmod;
    float slew_rate;
    bool limit;
    bool PD_limit;
    bool reset;
    bool I_term_set;
};
