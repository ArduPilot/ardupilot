#ifndef PID_h
#define PID_h

#include <avr/eeprom.h>
#include "WProgram.h"

//#define PID_SIZE 8

class PID {
public:
	// note, if using the eeprom load_gains/save_gains functions
	// it is not necessary to pass a gain_array pointer, instead
	// you need to pass address to load_gains/ save_gains functions
	PID(float * gain_array = 0);
	long 	get_pid(long err, long dt, float scaler);
	void	reset_I();
	void 	load_gains(int address);
	void 	save_gains(int address);
	void 	load_gains();
	void 	save_gains();
	void 	set_P(float p);
	void 	set_I(float i);
	void 	set_D(float d);
	void 	set_imax(int imax);
	void 	test(void);

	float 	get_P(void);
	float 	get_I(void);
	float 	get_D(void);
	int 	get_imax(void);
	float _imax;		// integrator max
	float _integrator;	// integrator value
	float _last_error;	// last error for derivative
	float _kp;
	float _ki;
	float _kd;
	

private:
	// low pass filter cut frequency
	// for derivative calculation,
	// set to 20 Hz becasue anything over that
	// is probably noise, see
	// http://en.wikipedia.org/wiki/Low-pass_filter
	static uint8_t RC = 20; 
	
	// pointer to the gains for this pid,
	// if not using eeprom save/load
	float * gain_array; 
};

#endif
