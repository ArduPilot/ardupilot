// #pragma once // this is to ensure that the file is handeled and included only once during compilation

// #include <AP_HAL/AP_HAL.h> // gives access to hardware abstraction layer (HAL) so I2C can be used
// #include "AP_Airspeed_Backend.h" // base classs for all airspeed sensor backends


// class AP_Airspeed_PitotHoneywell : public AP_Airspeed_Backend // defines a new class so that it can replace any other airspeed sensor backend and ardupilot can call standard functions regardless of sensor type

// {
//     public:  // everything below is accessible from outside the class
//     AP_Airspeed_PitotHoneywell(AP_Airspeed &frontend, int8_t bus, uint8_t address); 
    
//     /*  constructor that i called when the backend is created. frontend is the reference to the main airspeed mananger (AP_Airspeed)
//      bus is the I2C bus number to use for the AP (cube orange plus is = 1)
//       address is the I2C address of the sensor (Honeywell = 0x28 from datasheet)
//     */

//     // bool init() override; // called once at startup and registers perodically timer callback to read sensor regularly (20Hz)
//     // void update() override; // called regularly by ardupilot to publish new readings into ekf/tecs
//     // float get_differential_pressure_pa() const override; // returns latest dynamic pressure in pascals
//     // bool healthy() const override; //returns true if the sensor is working used by EKF and TECS so that it can reject if unhealthy
    
//     bool init() override;
//     void read() override;
//     bool healthy() const override;
//     bool get_differential_pressure(float &pressure) override;



//     private: // below here is internal implementation details not accessible from outside the class

//     void timer(); // function triggered by the hardware timer callback at ~ 20 Hz. performs I2C read and pressure calculation. runs outside main loop so non blocking

//     const uint8_t _address; // stores I2C address of the sensor and making it const ensures that it never changes after creation
//     AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev; // smart point used by ArduPilot HAL. represents the actual I2C device hardware connection. handles auto cleanup memory and low-level read/write

//     float _last_q = 0.0f; // stores the last measured differential pressure. EKF reads this in update()
//     bool _healthy = false; // tracks whether the last read succeeded. if read ok -> true, if error/no response -> false. it prevents bad data feeding into flight controller
    




// };
#pragma once
#include <AP_HAL/AP_HAL.h>
#include "AP_Airspeed_Backend.h"

class AP_Airspeed_PitotHoneywell : public AP_Airspeed_Backend {
public:
    AP_Airspeed_PitotHoneywell(AP_Airspeed &frontend, uint8_t inst, int8_t bus, uint8_t address);

    bool init() override;
    bool get_differential_pressure(float &pressure) override;
    bool get_temperature(float &temperature) override;
    bool has_airspeed() override { return false; }
    bool get_airspeed(float &airspeed) override { return false; }

private:
    void timer();

    AP_HAL::OwnPtr<AP_HAL::I2CDevice> _dev;
    uint8_t _address;
    float _last_q = 0.0f;
    float _last_temperature = 0.0f;
    bool _healthy = false;
};
