#ifndef _APO_COMMON_H
#define _APO_COMMON_H

FastSerialPort0(Serial);
FastSerialPort1(Serial1);
FastSerialPort2(Serial2);
FastSerialPort3(Serial3);

/*
 * Required Global Declarations
 */

static apo::AP_Autopilot * autoPilot;

void setup() {

    using namespace apo;

    AP_Var::load_all();

    // Declare all parts of the system
    AP_Navigator * navigator = NULL;
    AP_Guide * guide = NULL;
    AP_Controller * controller = NULL;
    AP_HardwareAbstractionLayer * hal = NULL;

    /*
     * Communications
     */
    Serial.begin(debugBaud, 128, 128); // debug

    // hardware abstraction layer
    hal = new AP_HardwareAbstractionLayer(
        halMode, board, vehicle, heartBeatTimeout);

    // debug serial
    hal->debug = &Serial;
    hal->debug->println_P(PSTR("initializing debug line"));

    /*
     * Sensor initialization
     */
    if (hal->getMode() == MODE_LIVE) {

        hal->debug->println_P(PSTR("initializing adc"));
        hal->adc = new ADC_CLASS;
        hal->adc->Init();

        if (batteryMonitorEnabled) {
            hal->batteryMonitor = new AP_BatteryMonitor(batteryPin,batteryVoltageDivRatio,batteryMinVolt,batteryMaxVolt);
        }

        if (gpsEnabled) {
            Serial1.begin(gpsBaud, 128, 16); // gps
            hal->debug->println_P(PSTR("initializing gps"));
            AP_GPS_Auto gpsDriver(&Serial1, &(hal->gps));
            hal->gps = &gpsDriver;
            hal->gps->callback = delay;
            hal->gps->init();
        }

        if (baroEnabled) {
            hal->debug->println_P(PSTR("initializing baro"));
            hal->baro = new BARO_CLASS;
            hal->baro->Init();
        }

        if (compassEnabled) {
            Wire.begin();
            hal->debug->println_P(PSTR("initializing compass"));
            hal->compass = new COMPASS_CLASS;
            hal->compass->set_orientation(compassOrientation);
            hal->compass->set_offsets(0,0,0);
            hal->compass->set_declination(0.0);
            hal->compass->init();
        }

        /**
         * Initialize ultrasonic sensors. If sensors are not plugged in, the navigator will not
         * initialize them and NULL will be assigned to those corresponding pointers.
         * On detecting NULL assigned to any ultrasonic sensor, its corresponding block of code
         * will not be executed by the navigator.
         * The coordinate system is assigned by the right hand rule with the thumb pointing down.
         * In set_orientation, it is defined as (front/back,left/right,down,up)
         */

        if (rangeFinderFrontEnabled) {
            hal->debug->println_P(PSTR("initializing front range finder"));
            RangeFinder * rangeFinder = new RANGE_FINDER_CLASS(NULL,new ModeFilter);
            rangeFinder->set_analog_port(1);
            rangeFinder->set_orientation(1, 0, 0);
            hal->rangeFinders.push_back(rangeFinder);
        }

        if (rangeFinderBackEnabled) {
            hal->debug->println_P(PSTR("initializing back range finder"));
            RangeFinder * rangeFinder = new RANGE_FINDER_CLASS(hal->adc,new ModeFilter);
            rangeFinder->set_analog_port(2);
            rangeFinder->set_orientation(-1, 0, 0);
            hal->rangeFinders.push_back(rangeFinder);
        }

        if (rangeFinderLeftEnabled) {
            hal->debug->println_P(PSTR("initializing left range finder"));
            RangeFinder * rangeFinder = new RANGE_FINDER_CLASS(hal->adc,new ModeFilter);
            rangeFinder->set_analog_port(3);
            rangeFinder->set_orientation(0, -1, 0);
            hal->rangeFinders.push_back(rangeFinder);
        }

        if (rangeFinderRightEnabled) {
            hal->debug->println_P(PSTR("initializing right range finder"));
            RangeFinder * rangeFinder = new RANGE_FINDER_CLASS(hal->adc,new ModeFilter);
            rangeFinder->set_analog_port(4);
            rangeFinder->set_orientation(0, 1, 0);
            hal->rangeFinders.push_back(rangeFinder);
        }

        if (rangeFinderUpEnabled) {
            hal->debug->println_P(PSTR("initializing up range finder"));
            RangeFinder * rangeFinder = new RANGE_FINDER_CLASS(hal->adc,new ModeFilter);
            rangeFinder->set_analog_port(5);
            rangeFinder->set_orientation(0, 0, -1);
            hal->rangeFinders.push_back(rangeFinder);
        }

        if (rangeFinderDownEnabled) {
            hal->debug->println_P(PSTR("initializing down range finder"));
            RangeFinder * rangeFinder = new RANGE_FINDER_CLASS(hal->adc,new ModeFilter);
            rangeFinder->set_analog_port(6);
            rangeFinder->set_orientation(0, 0, 1);
            hal->rangeFinders.push_back(rangeFinder);
        }

    }

    /*
     * Select guidance, navigation, control algorithms
     */
    navigator = new NAVIGATOR_CLASS(hal);
    guide = new GUIDE_CLASS(navigator, hal, velCmd, xt, xtLim);
    controller = new CONTROLLER_CLASS(navigator,guide,hal);

    /*
     * CommLinks
     */
    if (board==BOARD_ARDUPILOTMEGA_2)
    {
        Serial2.begin(telemBaud, 128, 128); // gcs
        hal->gcs = new COMMLINK_CLASS(&Serial2, navigator, guide, controller, hal);
    }
    else
    {
        Serial3.begin(telemBaud, 128, 128); // gcs
        hal->gcs = new COMMLINK_CLASS(&Serial3, navigator, guide, controller, hal);
    }

    /*
     * Hardware in the Loop
     */
    if (hal->getMode() == MODE_HIL_CNTL) {
        Serial.println("HIL line setting up");
        Serial1.begin(hilBaud, 128, 128);
        delay(1000);
        Serial1.println("starting hil");
        hal->hil = new COMMLINK_CLASS(&Serial1, navigator, guide, controller, hal);
    }

    /*
     * Start the autopilot
     */
    hal->debug->printf_P(PSTR("initializing autopilot\n"));
    hal->debug->printf_P(PSTR("free ram: %d bytes\n"),freeMemory());

    autoPilot = new apo::AP_Autopilot(navigator, guide, controller, hal,
                                      loopRate, loop0Rate, loop1Rate, loop2Rate, loop3Rate);
}

void loop() {
    autoPilot->update();
}

#endif //_APO_COMMON_H
// vim:ts=4:sw=4:expandtab
