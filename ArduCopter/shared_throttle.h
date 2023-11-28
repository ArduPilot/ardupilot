#ifndef SHARED_DATA_H
#define SHARED_DATA_H

struct SharedData {
    float throttle_input_total = 0.0;
};

extern SharedData sharedData; // Extern keyword for global declaration

#endif // SHARED_DATA_H
