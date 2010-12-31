#include <FastSerial.h>
#include <AP_Controller.h>
#include <AP_Var.h>
#include <AP_EEProm.h>
#include <AP_RcChannel.h>
#include <APM_RC.h>

FastSerialPort0(Serial);

class CarController : public AP_Controller
{
private:
    // state
    AP_Float * roll;
    AP_Float * airspeed;
    AP_Float * velocity;
    AP_Float * heading;

    // servo positions
    AP_Float * steering;
    AP_Float * throttle;

    // control variables
    AP_Float * headingCommand;
    AP_Float * velocityCommand;

    // channels
    uint8_t chStr;
    uint8_t chThr;
public:
    CarController() :
        chStr(0), chThr(1)
    {
        // rc channels
        addCh(new AP_RcChannel("STR",APM_RC,chStr,45));
        addCh(new AP_RcChannel("THR",APM_RC,chThr,100));

        // steering control loop
        addBlock(new SumGain(headingCommand,&AP_unity,heading,&AP_negativeUnity));
        addBlock(new Pid("HDNG",0.1,0,0,1,20));
        addBlock(new ToServo(getRc(chStr)));

        // throttle control loop
        addBlock(new SumGain(velocityCommand,&AP_unity,velocity,&AP_negativeUnity));
        addBlock(new Pid("THR",0.1,0,0,1,20));
        addBlock(new ToServo(getRc(chThr)));
    }
};

AP_Controller * controller;

void setup()
{
    Serial.begin(115200);
	//controller = new CarController;
}

void loop()
{
    Serial.println("Looping");
    delay(1000);
	//controller->update();
}
