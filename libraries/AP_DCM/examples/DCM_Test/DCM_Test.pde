// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

//
// Simple test for the AP_DCM library
//

#include <FastSerial.h>
#include <SPI.h>
#include <I2C.h>
#include <Arduino_Mega_ISR_Registry.h>
#include <AP_PeriodicProcess.h>
#include <AP_InertialSensor.h>
#include <AP_ADC.h>
#include <AP_IMU.h>
#include <AP_DCM.h>
#include <AP_Math.h>
#include <AP_Common.h>
#include <AP_Compass.h>

// uncomment this for a APM2 board
// #define APM2_HARDWARE


FastSerialPort(Serial, 0);

Arduino_Mega_ISR_Registry isr_registry;
AP_TimerProcess  scheduler;
AP_Compass_HMC5843 compass(AP_Var::k_key_none);

#ifdef APM2_HARDWARE
  AP_InertialSensor_MPU6000 ins( 53 );
# else
  AP_ADC_ADS7844          adc;
  AP_InertialSensor_Oilpan ins( &adc );
#endif // CONFIG_IMU_TYPE

static GPS         *g_gps;

AP_IMU_INS imu( &ins, AP_Var::k_key_none );
AP_DCM  dcm(&imu, g_gps);


#ifdef APM2_HARDWARE
# define A_LED_PIN        27
# define C_LED_PIN        25
# define LED_ON           LOW
# define LED_OFF          HIGH
# define MAG_ORIENTATION  AP_COMPASS_APM2_SHIELD
#else
# define A_LED_PIN        37
# define C_LED_PIN        35
# define LED_ON           HIGH
# define LED_OFF          LOW
# define MAG_ORIENTATION  AP_COMPASS_COMPONENTS_DOWN_PINS_FORWARD
#endif


static void flash_leds(bool on)
{
    digitalWrite(A_LED_PIN, on?LED_OFF:LED_ON);
    digitalWrite(C_LED_PIN, on?LED_ON:LED_OFF);
}

void setup(void)
{
	Serial.begin(115200);
	Serial.println("Starting up...");

	isr_registry.init();
	scheduler.init(&isr_registry);

    I2c.begin();
    I2c.timeOut(5);
    I2c.setSpeed(true);

    SPI.begin();
    SPI.setClockDivider(SPI_CLOCK_DIV16);

	imu.init(IMU::COLD_START, delay, flash_leds, &scheduler);
	imu.init_accel(delay, flash_leds);

	compass.set_orientation(MAG_ORIENTATION);
	if (compass.init()) {
		printf("Enabling compass\n");
		compass.null_offsets_enable();
		dcm.set_compass(&compass);
	}
}

void loop(void)
{
	Vector3f	accel;
	Vector3f	gyro;
	static uint8_t counter;

	delay(20);

	dcm.update_DCM();
	if (counter++ == 10) {
		counter = 0;
		gyro  = imu.get_gyro();
		accel = imu.get_accel();
		Serial.printf_P(PSTR("r:%4d  p:%4d  y:%3d  g=(%5.1f %5.1f %5.1f)  a=(%5.1f %5.1f %5.1f)\n"),
				(int)dcm.roll_sensor / 100,
				(int)dcm.pitch_sensor / 100,
				(uint16_t)dcm.yaw_sensor / 100,
				gyro.x, gyro.y, gyro.z,
				accel.x, accel.y, accel.z);
	}
}
