// #include <iostream>
#include <time.h>
#include <chrono>
#include <stdlib.h>

#include "libAP_JSON.cpp"

uint16_t servo_out[16];

uint64_t micros()
{
    uint64_t us = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::
                  now().time_since_epoch()).count();
    return us; 
}

int main()
{
    // init the ArduPilot connection
    libAP_JSON ap;
    if (ap.InitSockets("127.0.0.1", 9002))
    {
        std::cout << "connected to AP!" << std::endl;
    }

    // send and receive data from AP
    while (true)
    {
        double timestamp = (double) micros() / 1000000.0;

        if (ap.ReceiveServoPacket(servo_out))
        {
#if DEBUG_ENABLED
            std::cout << "servo_out PWM: [";
            for (int i = 0; i < MAX_SERVO_CHANNELS - 1; ++i)
            {
                std::cout << servo_out[i] << ", ";
            }
            std::cout << servo_out[MAX_SERVO_CHANNELS - 1] << "]" << std::endl;
#endif
        }

        ap.SendState(timestamp,
                     0, 0, 0,    // gyro
                     0, 0, 9.81, // accel
                     0, 0, 0,    // position
                     0, 0, 0,    // attitude
                     0, 0, 0,    // velocity
                     0);         // airspeed
        
        usleep(10000); // run this test program at about 100 Hz. FDMs would idealy be much faster than this.
    }
    return 0;
}
