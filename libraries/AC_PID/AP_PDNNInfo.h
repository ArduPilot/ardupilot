#pragma once

// This structure provides information on the internal member data of
// a PDNN.  
// It is also used to pass PDNN information into controllers...

struct AP_PDNNInfo {
    float target;
    float actual;
    float error;
    float P;
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
