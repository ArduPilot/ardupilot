// Collection of register names in the TI PWM Subsystem
// for use with BeagleBone and TI Sitara AM335X series

//top level subsystem config and clock registers
#define PWMSS_SYSCONFIG			0x4
#define PWMSS_CLKCONFIG 		0x8
#define PWMSS_CLKSTATUS 		0xC
#define PWMSS_ECAPCLK_EN		(0x01<<0)
#define PWMSS_ECAPCLK_STOP_REQ	(0x01<<1)
#define PWMSS_EQEPCLK_EN		(0x01<<4)
#define PWMSS_EQEPCLK_STOP_REQ	(0x01<<5)
#define PWMSS_EPWMCLK_EN		(0x01<<8)
#define PWMSS_EPWMCLK_STOP_REQ	(0x01<<9)

#define PWMSS_ECAPCLK_EN_ACK	(0x01<<0)
#define PWMSS_EPWMCLK_EN_ACK	(0x01<<8)

// eQEP register offsets from its base IO address
#define QPOSCNT    0x0000
#define QPOSINIT   0x0004
#define QPOSMAX    0x0008
#define QPOSCMP    0x000C
#define QPOSILAT   0x0010
#define QPOSSLAT   0x0014
#define QPOSLAT    0x0018
#define QUTMR      0x001C
#define QUPRD      0x0020    
#define QWDTMR     0x0024
#define QWDPRD     0x0026
#define QDECCTL    0x0028
#define QEPCTL     0x002A
#define QCAPCTL    0x002C
#define QPOSCTL    0x002E
#define QEINT      0x0030
#define QFLG       0x0032
#define QCLR       0x0034
#define QFRC       0x0036
#define QEPSTS     0x0038
#define QCTMR      0x003A
#define QCPRD      0x003C
#define QCTMRLAT   0x003E
#define QCPRDLAT   0x0040
#define QREVID     0x005C

// Bits for the QDECTL register
#define QSRC1      (0x0001 << 15)
#define QSRC0      (0x0001 << 14)
#define SOEN       (0x0001 << 13)
#define SPSEL      (0x0001 << 12)
#define XCR        (0x0001 << 11)
#define SWAP       (0x0001 << 10)
#define IGATE      (0x0001 << 9)
#define QAP        (0x0001 << 8)
#define QBP        (0x0001 << 7)
#define QIP        (0x0001 << 6)
#define QSP        (0x0001 << 5)

// Bits for the QEPCTL register
#define FREESOFT1  (0x0001 << 15)
#define FREESOFT0  (0x0001 << 14)
#define PCRM1      (0x0001 << 13)
#define PCRM0      (0x0001 << 12)
#define SEI1       (0x0001 << 11)
#define SEI0       (0x0001 << 10)
#define IEI1       (0x0001 << 9)
#define IEI0       (0x0001 << 8)
#define SWI        (0x0001 << 7)
#define SEL        (0x0001 << 6)
#define IEL1       (0x0001 << 5)
#define IEL0       (0x0001 << 4)
#define PHEN       (0x0001 << 3)
#define QCLM       (0x0001 << 2)
#define UTE        (0x0001 << 1)
#define WDE        (0x0001 << 0)

// Bits for the QCAPCTL register
#define CEN        (0x0001 << 15)
#define CCPS2      (0x0001 << 6)
#define CCPS0      (0x0001 << 5)
#define CCPS1      (0x0001 << 4)
#define UPPS3      (0x0001 << 3)
#define UPPS2      (0x0001 << 2)
#define UPPS1      (0x0001 << 1)
#define UPPS0      (0x0001 << 0)

// Bits for the QPOSCTL register
#define PCSHDW     (0x0001 << 15)
#define PCLOAD     (0x0001 << 14)
#define PCPOL      (0x0001 << 13)
#define PCE        (0x0001 << 12)
#define PCSPW11    (0x0001 << 11)
#define PCSPW10    (0x0001 << 10)
#define PCSPW9    (0x0001 << 9)
#define PCSPW8    (0x0001 << 8)
#define PCSPW7    (0x0001 << 7)
#define PCSPW6    (0x0001 << 6)
#define PCSPW5    (0x0001 << 5)
#define PCSPW4    (0x0001 << 4)
#define PCSPW3    (0x0001 << 3)
#define PCSPW2    (0x0001 << 2)
#define PCSPW1    (0x0001 << 1)
#define PCSPW0    (0x0001 << 0)

// Bits for the interrupt registers
#define EQEP_INTERRUPT_MASK (0x0FFF)
#define UTOF                (0x0001 << 11)

// Modes for the eQEP unit
//  Absolute - the position entry represents the current position of the encoder.
//             Poll this value and it will be notified every period nanoseconds
//  Relative - the position entry represents the last latched position of the encoder
//             This value is latched every period nanoseconds and the internal counter
//             is subsequenty reset
#define TIEQEP_MODE_ABSOLUTE    0
#define TIEQEP_MODE_RELATIVE    1

//// eQep and pwmss registers
#define PWMSS0_BASE   0x48300000
#define PWMSS1_BASE   0x48302000
#define PWMSS2_BASE   0x48304000
#define EQEP_OFFSET  0x180
#define PWM_OFFSET  0x200
#define PWMSS_MEM_SIZE 8192 // 8kb

// TBCTL (Time-Base Control register)
// TBCNT MODE bits
#define TB_COUNT_UP 0x0<<14
#define TB_COUNT_DOWN 0x1<<14
#define TB_COUNT_UPDOWN 0x2<<14
#define TB_FREEZE 0x3<<14
// PHSDIR bit
#define TB_DOWN 0x0<<13
#define TB_UP 0x1<<13

// CLKDIV bits
#define TB_DIV1 0x0<<10
#define TB_DIV2 0x1<<10
#define TB_DIV4 0x2<<10
#define TB_DIV8 0x3<<10
#define TB_DIV16 0x4<<10
#define TB_DIV32 0x5<<10
#define TB_DIV64 0x6<<10
#define TB_DIV128 0x7<<10

// HSPCLKDIV bits
#define TB_HDIV1 0x0<<7
#define TB_HDIV2 0x1<<7
#define TB_HDIV4 0x2<<7
#define TB_HDIV6 0x3<<7
#define TB_HDIV8 0x4<<7
#define TB_HDIV10 0x5<<7
#define TB_HDIV12 0x6<<7
#define TB_HDIV14 0x7<<7

// SYNCOSEL bits
#define TB_SYNC_IN 0x0<<4
#define TB_CTR_ZERO 0x1<<4
#define TB_CTR_CMPB 0x2<<4
#define TB_SYNC_DISABLE 0x3<<4
// PRDLD bit
#define TB_SHADOW 0x0<<3
#define TB_IMMEDIATE 0x1<<3
// PHSEN bit
#define TB_DISABLE 0x0<<2
#define TB_ENABLE 0x1<<2

/*********************************
* CMPCTL (Compare Control)
*********************************/
// SHDWAMODE and SHDWBMODE bits
#define CC_SHADOW_B 0x0<<6
#define CC_IMMEDIATE_B 0x1<<6
#define CC_SHADOW_A 0x0<<4
#define CC_IMMEDIATE_A 0x1<<4
// LOADAMODE and LOADBMODE bits
#define CC_CTR_ZERO_B 0x0<<2
#define CC_CTR_PRD_B 0x1<<2
#define CC_CTR_EITHER_B 0x2<<2
#define CC_LOAD_FREEZE_B 0x3<<2
#define CC_CTR_ZERO_A 0x0
#define CC_CTR_PRD_A 0x1
#define CC_CTR_EITHER_A 0x2
#define CC_LOAD_FREEZE_A 0x3


/*********************************
* AQCTLA and AQCTLB (Action-qualifier Control)
*********************************/
// ZRO, PRD, CAU, CAD, CBU, CBD bits
#define AQ_ZRO_NO_ACTION 0x0
#define AQ_ZRO_CLEAR 0x1
#define AQ_ZRO_SET 0x2
#define AQ_ZRO_TOGGLE 0x3
#define AQ_CAU_NO_ACTION 0x0<<4
#define AQ_CAU_CLEAR 0x1<<4
#define AQ_CAU_SET 0x2<<4
#define AQ_CAU_TOGGLE 0x3<<4
#define AQ_CBU_NO_ACTION 0x0<<8
#define AQ_CBU_CLEAR 0x1<<8
#define AQ_CBU_SET 0x2<<8
#define AQ_CBU_TOGGLE 0x3<<8


// eHRPWM control register locations
#define TBCTL 0x00
#define TBSTS 0x02
#define TBPHSHR 0x04
#define TBPHS 0x06
#define TBCNT 0x08
#define TBPRD 0x0A

#define CMPCTL 0x0E
#define CMPAHR 0x10
#define CMPA 0x12
#define CMPB 0x14

#define AQ_CTLA	0x16
#define AQ_CTLB 0x18
#define AQ_SFRC 0x1A
#define AQ_CSFRC 0x1C

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
#define MODULEMODE_ENABLE 	0x2


