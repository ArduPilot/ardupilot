// Collection of register names in the TI PWM Subsystem
// for use with BeagleBone and TI Sitara AM335X series

//top level subsystem config and clock registers
#define PWMSS_CLKCONFIG 		0x8
#define PWMSS_EQEPCLK_EN		(0x01<<4)


// eQEP register offsets from its base IO address
#define QPOSCNT    0x0000
#define QPOSMAX    0x0008
#define QUPRD      0x0020
#define QDECCTL    0x0028
#define QEPCTL     0x002A
#define QEINT      0x0030


// Bits for the QEPCTL register

#define SWI        (0x0001 << 7)
#define IEL0       (0x0001 << 4)
#define PHEN       (0x0001 << 3)
#define QCLM       (0x0001 << 2)
#define UTE        (0x0001 << 1)

// Bits for the interrupt registers
#define EQEP_INTERRUPT_MASK (0x0FFF)
#define UTOF                (0x0001 << 11)

//// eQep and pwmss registers
#define PWMSS0_BASE   0x48300000
#define PWMSS1_BASE   0x48302000
#define PWMSS2_BASE   0x48304000
#define EQEP_OFFSET  0x180
#define PWM_OFFSET  0x200
#define PWMSS_MEM_SIZE 8192 // 8kb



// eHRPWM control register locations

#define TBPRD 0x0A
#define CMPA 0x12
#define CMPB 0x14


/*********************************
* clock control registers
*********************************/
#ifndef CM_PER
	#define CM_PER 0x44E00000 //base of Clock Module Peripheral control
	#define CM_PER_PAGE_SIZE 1024 //1kb
#endif

#define CM_PER_EPWMSS1_CLKCTRL 0xCC //16 bit register
#define CM_PER_EPWMSS0_CLKCTRL 0xD4 //16 bit register
#define CM_PER_EPWMSS2_CLKCTRL 0xD8 //16 bit register

#define MODULEMODE_DISABLED 0x0
#define MODULEMODE_ENABLE 	0x2 //aa


