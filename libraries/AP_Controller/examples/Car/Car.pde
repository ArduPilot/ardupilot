#include <FastSerial.h>
#include <AP_Controller.h>
#include <AP_Var.h>
#include <AP_RcChannel.h>
#include <APM_RC.h>
#include <AP_Common.h>

FastSerialPort0(Serial);
float test1;
float test2;

class CarController : public AP_Controller
{
private:
    // state
    float * velocity;
    float *  heading;

    // control variables
    float *  headingCommand;
    float *  velocityCommand;

    // channels
    AP_RcChannel * steeringCh;
    AP_RcChannel * throttleCh;

    static float one;
    static float negOne;

public:
    CarController(AP_Var::Key pidStrKey, AP_Var::Key pidThrKey,
    		float *  heading, float *  velocity,
    		float *  headingCommand, float *  velocityCommand,
            AP_RcChannel * steeringCh, AP_RcChannel * throttleCh) :
        heading(heading), velocity(velocity),
        headingCommand(headingCommand), velocityCommand(velocityCommand),
        steeringCh(steeringCh), throttleCh(throttleCh)
    {
        // steering control loop
        addBlock(new SumGain(headingCommand,&one,heading,&negOne));
        addBlock(new Pid(pidStrKey,PSTR("STR"),1,1,1,1,20));
        //addBlock(new Sink(test1));
        addBlock(new ToServo(steeringCh));


        // throttle control loop
        addBlock(new SumGain(velocityCommand,&one,velocity,&negOne));
        addBlock(new Pid(pidThrKey,PSTR("THR"),1,1,1,1,20));
        //addBlock(new Sink(test2));
        addBlock(new ToServo(throttleCh));
    }
};
float CarController::negOne = -1;
float CarController::one = 1;

AP_Controller * controller;
Vector<AP_RcChannel * > ch;

float heading, velocity; // measurements
float headingCommand, velocityCommand; // guidance data
int8_t sign = 1;
float value = 0;

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
    headingCommand = 0;
    velocityCommand = 0;

    // rc channels
    APM_RC.Init();
    ch.push_back(new AP_RcChannel(chStrKey,PSTR("STR"),APM_RC,1,45));
    ch.push_back(new AP_RcChannel(chThrKey,PSTR("THR"),APM_RC,2,45));

    // controller
    controller = new CarController(pidStrKey,pidThrKey,&heading,&velocity,&headingCommand,&velocityCommand,ch[0],ch[1]);
}

void loop()
{
    // loop rate 20 Hz
    delay(1000.0/20);

    // feedback signal, 1/3 Hz sawtooth
    if (value > 1)
    {
        value = 1;
        sign = -1;
    }
    else if (value < -1)
    {
        value = -1;
        sign = 1;
    }
    value += sign/20.0/3.0;
    velocity = value;
    heading = value;

    // update controller
    controller->update(1./20);

    // output
    Serial.printf("hdng:%f\tcmd:(%f)\tservo:%f\t", heading,headingCommand, ch[0]->getNormalized());
    Serial.printf("| thr:%f\tcmd:(%f)\tservo:%f\n", velocity, velocityCommand, ch[1]->getNormalized());

    //Serial.printf("test 1: %f test 2: %f\n", test1, test2);
}

// vim:ts=4:sw=4:expandtab
