
#include "IMU.h"

/* Empty implementations for the IMU functions.
 * Although these will never be used, in certain situations with
 * optimizations turned off, having empty implementations in an object
 * file will help satisify the linker.
 */

IMU::IMU () {}


void IMU::init( Start_style style,
		void (*delay_cb)(unsigned long t),
		void (*flash_leds_cb)(bool on),
		AP_PeriodicProcess * scheduler )
{ }

void IMU::init_accel(void (*delay_cb)(unsigned long t), void (*flash_leds_cb)(bool on))
{ }

void IMU::init_gyro(void (*delay_cb)(unsigned long t), void (*flash_leds_cb)(bool on))
{ }

bool IMU::update(void) { return false; }

float	IMU::gx(void) { return 0.0; }
float	IMU::gy(void) { return 0.0; }
float	IMU::gz(void) { return 0.0; }
float	IMU::ax(void) { return 0.0; }
float	IMU::ay(void) { return 0.0; }
float	IMU::az(void) { return 0.0; }
void	IMU::ax(const float v) { }
void	IMU::ay(const float v) { }
void	IMU::az(const float v) { }
