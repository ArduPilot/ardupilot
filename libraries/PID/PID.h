#ifndef PID_h
#define PID_h

#include <avr/eeprom.h>
#include "WProgram.h"

//#define PID_SIZE 8

class PID {
public:
	PID();
	long 	get_pid(long err, long dt, float scaler);
	void	reset_I();
	void 	load_gains(int address);
	void 	save_gains(int address);
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
};

#endif