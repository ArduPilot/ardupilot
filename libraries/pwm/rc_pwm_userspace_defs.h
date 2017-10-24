// ti_pwm_userspace_defs.h
//
// this is a list of the userspace directories created by the ti pwm driver
// all directories derived from 4.4.9-bone10


// export directories
const char* pwm_export_path[2][3] = {{ \
"/sys/devices/platform/ocp/48300000.epwmss/48300200.ehrpwm/pwm/pwmchip0/export", \
"/sys/devices/platform/ocp/48302000.epwmss/48302200.ehrpwm/pwm/pwmchip2/export", \
"/sys/devices/platform/ocp/48304000.epwmss/48304200.ehrpwm/pwm/pwmchip4/export"},{ \
"/sys/devices/platform/ocp/48300000.epwmss/48300200.pwm/pwm/pwmchip0/export", \
"/sys/devices/platform/ocp/48302000.epwmss/48302200.pwm/pwm/pwmchip2/export", \
"/sys/devices/platform/ocp/48304000.epwmss/48304200.pwm/pwm/pwmchip4/export"}};\


const char* pwm_unexport_path[2][3] = {{ \
"/sys/devices/platform/ocp/48300000.epwmss/48300200.ehrpwm/pwm/pwmchip0/unexport", \
"/sys/devices/platform/ocp/48302000.epwmss/48302200.ehrpwm/pwm/pwmchip2/unexport", \
"/sys/devices/platform/ocp/48304000.epwmss/48304200.ehrpwm/pwm/pwmchip4/unexport"},{ \
"/sys/devices/platform/ocp/48300000.epwmss/48300200.pwm/pwm/pwmchip0/unexport", \
"/sys/devices/platform/ocp/48302000.epwmss/48302200.pwm/pwm/pwmchip2/unexport", \
"/sys/devices/platform/ocp/48304000.epwmss/48304200.pwm/pwm/pwmchip4/unexport"}};\


// channel enable
const char* pwm_chA_enable_path[2][3] = {{ \
"/sys/devices/platform/ocp/48300000.epwmss/48300200.ehrpwm/pwm/pwmchip0/pwm0/enable", \
"/sys/devices/platform/ocp/48302000.epwmss/48302200.ehrpwm/pwm/pwmchip2/pwm0/enable", \
"/sys/devices/platform/ocp/48304000.epwmss/48304200.ehrpwm/pwm/pwmchip4/pwm0/enable"},{
"/sys/devices/platform/ocp/48300000.epwmss/48300200.pwm/pwm/pwmchip0/pwm0/enable", \
"/sys/devices/platform/ocp/48302000.epwmss/48302200.pwm/pwm/pwmchip2/pwm0/enable", \
"/sys/devices/platform/ocp/48304000.epwmss/48304200.pwm/pwm/pwmchip4/pwm0/enable"}};

const char* pwm_chB_enable_path[2][3] = {{ \
"/sys/devices/platform/ocp/48300000.epwmss/48300200.ehrpwm/pwm/pwmchip0/pwm1/enable", \
"/sys/devices/platform/ocp/48302000.epwmss/48302200.ehrpwm/pwm/pwmchip2/pwm1/enable", \
"/sys/devices/platform/ocp/48304000.epwmss/48304200.ehrpwm/pwm/pwmchip4/pwm1/enable"},{
"/sys/devices/platform/ocp/48300000.epwmss/48300200.pwm/pwm/pwmchip0/pwm1/enable", \
"/sys/devices/platform/ocp/48302000.epwmss/48302200.pwm/pwm/pwmchip2/pwm1/enable", \
"/sys/devices/platform/ocp/48304000.epwmss/48304200.pwm/pwm/pwmchip4/pwm1/enable"}};

// channel polarity
const char* pwm_chA_polarity_path[2][3] = {{ \
"/sys/devices/platform/ocp/48300000.epwmss/48300200.ehrpwm/pwm/pwmchip0/pwm0/polarity", \
"/sys/devices/platform/ocp/48302000.epwmss/48302200.ehrpwm/pwm/pwmchip2/pwm0/polarity", \
"/sys/devices/platform/ocp/48304000.epwmss/48304200.ehrpwm/pwm/pwmchip4/pwm0/polarity"},{
"/sys/devices/platform/ocp/48300000.epwmss/48300200.pwm/pwm/pwmchip0/pwm0/polarity", \
"/sys/devices/platform/ocp/48302000.epwmss/48302200.pwm/pwm/pwmchip2/pwm0/polarity", \
"/sys/devices/platform/ocp/48304000.epwmss/48304200.pwm/pwm/pwmchip4/pwm0/polarity"}};

const char* pwm_chB_polarity_path[2][3] = {{ \
"/sys/devices/platform/ocp/48300000.epwmss/48300200.ehrpwm/pwm/pwmchip0/pwm1/polarity", \
"/sys/devices/platform/ocp/48302000.epwmss/48302200.ehrpwm/pwm/pwmchip2/pwm1/polarity", \
"/sys/devices/platform/ocp/48304000.epwmss/48304200.ehrpwm/pwm/pwmchip4/pwm1/polarity"},{
"/sys/devices/platform/ocp/48300000.epwmss/48300200.pwm/pwm/pwmchip0/pwm1/polarity", \
"/sys/devices/platform/ocp/48302000.epwmss/48302200.pwm/pwm/pwmchip2/pwm1/polarity", \
"/sys/devices/platform/ocp/48304000.epwmss/48304200.pwm/pwm/pwmchip4/pwm1/polarity"}};


// channel period
const char* pwm_chA_period_path[2][3] = {{ \
"/sys/devices/platform/ocp/48300000.epwmss/48300200.ehrpwm/pwm/pwmchip0/pwm0/period", \
"/sys/devices/platform/ocp/48302000.epwmss/48302200.ehrpwm/pwm/pwmchip2/pwm0/period", \
"/sys/devices/platform/ocp/48304000.epwmss/48304200.ehrpwm/pwm/pwmchip4/pwm0/period"},{
"/sys/devices/platform/ocp/48300000.epwmss/48300200.pwm/pwm/pwmchip0/pwm0/period", \
"/sys/devices/platform/ocp/48302000.epwmss/48302200.pwm/pwm/pwmchip2/pwm0/period", \
"/sys/devices/platform/ocp/48304000.epwmss/48304200.pwm/pwm/pwmchip4/pwm0/period"}};

const char* pwm_chB_period_path[2][3] = {{ \
"/sys/devices/platform/ocp/48300000.epwmss/48300200.ehrpwm/pwm/pwmchip0/pwm1/period", \
"/sys/devices/platform/ocp/48302000.epwmss/48302200.ehrpwm/pwm/pwmchip2/pwm1/period", \
"/sys/devices/platform/ocp/48304000.epwmss/48304200.ehrpwm/pwm/pwmchip4/pwm1/period"},{
"/sys/devices/platform/ocp/48300000.epwmss/48300200.pwm/pwm/pwmchip0/pwm1/period", \
"/sys/devices/platform/ocp/48302000.epwmss/48302200.pwm/pwm/pwmchip2/pwm1/period", \
"/sys/devices/platform/ocp/48304000.epwmss/48304200.pwm/pwm/pwmchip4/pwm1/period"}};

// channel duty cycle
const char* pwm_chA_duty_path[2][3] = {{ \
"/sys/devices/platform/ocp/48300000.epwmss/48300200.ehrpwm/pwm/pwmchip0/pwm0/duty_cycle", \
"/sys/devices/platform/ocp/48302000.epwmss/48302200.ehrpwm/pwm/pwmchip2/pwm0/duty_cycle", \
"/sys/devices/platform/ocp/48304000.epwmss/48304200.ehrpwm/pwm/pwmchip4/pwm0/duty_cycle"},{
"/sys/devices/platform/ocp/48300000.epwmss/48300200.pwm/pwm/pwmchip0/pwm0/duty_cycle", \
"/sys/devices/platform/ocp/48302000.epwmss/48302200.pwm/pwm/pwmchip2/pwm0/duty_cycle", \
"/sys/devices/platform/ocp/48304000.epwmss/48304200.pwm/pwm/pwmchip4/pwm0/duty_cycle"}};

const char* pwm_chB_duty_path[2][3] = {{ \
"/sys/devices/platform/ocp/48300000.epwmss/48300200.ehrpwm/pwm/pwmchip0/pwm1/duty_cycle", \
"/sys/devices/platform/ocp/48302000.epwmss/48302200.ehrpwm/pwm/pwmchip2/pwm1/duty_cycle", \
"/sys/devices/platform/ocp/48304000.epwmss/48304200.ehrpwm/pwm/pwmchip4/pwm1/duty_cycle"},{
"/sys/devices/platform/ocp/48300000.epwmss/48300200.pwm/pwm/pwmchip0/pwm1/duty_cycle", \
"/sys/devices/platform/ocp/48302000.epwmss/48302200.pwm/pwm/pwmchip2/pwm1/duty_cycle", \
"/sys/devices/platform/ocp/48304000.epwmss/48304200.pwm/pwm/pwmchip4/pwm1/duty_cycle"}};

