/*
 *  AP_RangeFinder_test
 *  Code by DIYDrones.com
 */

// includes
#include <AP_Common.h>
#include <AP_Progmem.h>
#include <AP_Param.h>
#include <AP_HAL.h>
#include <AP_Math.h>
#include <AP_RangeFinder.h>
#include <AP_ADC.h>
#include <Filter.h>
#include <AP_Buffer.h>
#include <AP_HAL_AVR.h>

// uncomment appropriate line corresponding to your sonar
#define SONAR_TYPE AP_RANGEFINDER_MAXSONARXL      // 0 - XL (default)
//#define SONAR_TYPE AP_RANGEFINDER_MAXSONARLV      // 1 - LV (cheaper)
//#define SONAR_TYPE AP_RANGEFINDER_MAXSONARXLL     // 2 - XLL (XL with 10m range)
//#define SONAR_TYPE AP_RANGEFINDER_MAXSONARHRLV    // 3 - HRLV-MaxSonar-EZ0 (5m range)
//#define SONAR_TYPE AP_RANGEFINDER_MAXSONARI2CXL   // 4 - XLI2C (XL with I2C interface and 7m range)

// For APM1 we use built in ADC for sonar reads from an analog pin
#if CONFIG_HAL_BOARD == HAL_BOARD_APM1 && SONAR_TYPE <= AP_RANGEFINDER_MAXSONARHRLV
# define USE_ADC_ADS7844  // use APM1's built in ADC and connect sonar to pitot tube
#endif

// define Pitot tube's ADC Channel
#define AP_RANGEFINDER_PITOT_TYPE_ADC_CHANNEL 7

////////////////////////////////////////////////////////////////////////////////
// hal.console-> ports
////////////////////////////////////////////////////////////////////////////////


#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;
#else
const AP_HAL::HAL& hal = AP_HAL_AVR_APM1;
#endif

// declare global instances
ModeFilterInt16_Size5 mode_filter(2);

#ifdef USE_ADC_ADS7844
AP_ADC_ADS7844 adc;
AP_ADC_AnalogSource adc_analog_source(&adc,
        AP_RANGEFINDER_PITOT_TYPE_ADC_CHANNEL, 0.25);// use Pitot tube
#endif

AP_RangeFinder_MaxsonarXL *rf;

void setup()
{
    hal.console->println("Range Finder Test v1.1");
    hal.console->print("Sonar Type: ");
    hal.console->println(SONAR_TYPE);

#ifdef USE_ADC_ADS7844
    adc.Init();   // APM ADC initialization
    AP_HAL::AnalogSource *analog_source = &adc_analog_source;
    float scaling = 3.3;
#else     
    AP_HAL::AnalogSource *analog_source = hal.analogin->channel(3);
    float scaling = 5;
#endif
    rf = new AP_RangeFinder_MaxsonarXL(analog_source, &mode_filter);
    rf->calculate_scaler(SONAR_TYPE, scaling);   // setup scaling for sonar
}

void loop()
{
    hal.console->print("dist:");
    hal.console->print(rf->read());
    hal.console->print("\traw:");
    hal.console->print(rf->raw_value);
    hal.console->println();

    hal.scheduler->delay(100);

}

AP_HAL_MAIN();
