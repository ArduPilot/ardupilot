#include <FastSerial.h>
#include <AP_Controller.h>
#include <AP_Var.h>
#include <AP_RcChannel.h>
#include <APM_RC.h>

FastSerialPort0(Serial);

class CarController : public AP_Controller
{
private:
    // state
    AP_Var & velocity;
    AP_Var & heading;

    // control variables
    AP_Var & headingCommand;
    AP_Var & velocityCommand;

    // channels
    AP_RcChannel * steeringCh;
    AP_RcChannel * throttleCh;

public:
    CarController(AP_Var::Key pidStrKey, AP_Var::Key pidThrKey,
            AP_Var & heading, AP_Var & velocity,
            AP_Var & headingCommand, AP_Var & velocityCommand,
            AP_RcChannel * steeringCh, AP_RcChannel * throttleCh) :
        heading(heading), velocity(velocity),
        headingCommand(headingCommand), velocityCommand(velocityCommand), 
        steeringCh(steeringCh), throttleCh(throttleCh)
    {
      
        // steering control loop
        addBlock(new SumGain(headingCommand,AP_Float_unity,heading,AP_Float_negative_unity));
        addBlock(new Pid(pidStrKey,PSTR("STR"),0.1,0,0,1,20));
        addBlock(new ToServo(steeringCh));

        // throttle control loop
        addBlock(new SumGain(velocityCommand,AP_Float_unity,velocity,AP_Float_negative_unity));
        addBlock(new Pid(pidThrKey,PSTR("THR"),0.1,0,0,1,20));
        addBlock(new ToServo(throttleCh));
    }
};

AP_Controller * controller;
Vector<AP_RcChannel * > ch;

AP_Float heading, velocity; // measurements
AP_Float headingCommand, velocityCommand; // guidance data

enum keys
{
    config,
    chStrKey,
    chThrKey,
    pidStrKey,
    pidThrKey
}; 

void setup()
{
    Serial.begin(115200);

    // variables
    heading = 0;
    velocity = 0;
    headingCommand = 1;
    velocityCommand = 1;

    // rc channels
    APM_RC.Init();
    ch.push_back(new AP_RcChannel(chStrKey,PSTR("STR"),APM_RC,1,45));
    ch.push_back(new AP_RcChannel(chThrKey,PSTR("THR"),APM_RC,2,100));

    // controller
    controller = new CarController(pidStrKey,pidThrKey,heading,velocity,headingCommand,velocityCommand,ch[0],ch[1]);
}

void loop()
{
    Serial.println("Looping");
    heading += 0.1;
    velocity += 0.1;
    delay(1000);
    controller->update(.1);
    Serial.printf("heading:\t%f\tcommand(%f)\tservo(%f)\n", (float)heading,(float) headingCommand, ch[0]->getNormalized());
    Serial.printf("velocity:\t%f\tcommand(%f)\tservo(%f)\n", (float)velocity, (float) velocityCommand, ch[1]->getNormalized());
}

// vim:ts=4:sw=4:expandtab
