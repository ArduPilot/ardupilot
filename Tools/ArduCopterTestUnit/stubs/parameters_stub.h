#ifndef PARAMETERS_STUB_H_
#define PARAMETERS_STUB_H_

#include "ap_int16_stub.h"

class Parameters {
public:
	bool compass_enabled = true;
	bool optflow_enabled = true;
	bool sonar_enabled   = true;
	int8_t telem_delay     = 0;
	int16_t sysid_my_gcs  = 1;
	RC_Channel rc_1;
	RC_Channel rc_2;
	RC_Channel rc_3;
	RC_Channel rc_4;
	RC_Channel rc_5;
	RC_Channel rc_6;
	RC_Channel rc_7;
	RC_Channel rc_8;
    AP_Int16   log_bitmask;
};//Parameters

#endif /* PARAMETERS_STUB_H_ */
