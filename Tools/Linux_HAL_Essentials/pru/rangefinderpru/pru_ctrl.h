

/* PRU_CTRL register set */
typedef struct{

	/* PRU_CTRL_CONTROL register bit field */
	union {
		volatile uint32_t CTRL;

		volatile struct{
			unsigned SOFT_RST_N : 1;
			unsigned EN : 1;
			unsigned SLEEPING : 1;
			unsigned CTR_EN : 1;
			unsigned rsvd4 : 4;
			unsigned SINGLE_STEP : 1;
			unsigned rsvd9 : 6;
			unsigned RUNSTATE : 1;
			unsigned PCTR_RST_VAL : 16;
		} CONTROL_bit;
	} ;	// 0x0


	/* PRU_CTRL_STATUS register bit field */
	union {
		volatile uint32_t STS;

		volatile struct{
			unsigned PCTR : 16;
			unsigned rsvd16 : 16;
		} STATUS_bit;
	} ;	// 0x4


	/* PRU_CTRL_WAKEUP_EN register bit field */
	union {
		volatile uint32_t WAKEUP_EN;

		volatile struct{
			unsigned BITWISE_ENS : 32;
		} WAKEUP_EN_bit;
	} ;	// 0x8


	/* PRU_CTRL_CYCLE register bit field */
	union {
		volatile uint32_t CYCLE;

		volatile struct{
			unsigned CYCLECOUNT : 32;
		} CYCLE_bit;
	} ;	// 0xC


	/* PRU_CTRL_STALL register bit field */
	union {
		volatile uint32_t STALL;

		volatile  struct{
			unsigned STALLCOUNT : 32;
		} STALL_bit;
	} ;	// 0x10


	uint32_t rsvd14[3];	// 0x14 - 0x1C


	/* PRU_CTRL_CTBIR0 register bit field */
	union {
		volatile uint32_t CTBIR0;

		volatile struct{
			unsigned C24_BLK_IDX : 8;
			unsigned rsvd8 : 8;
			unsigned C25_BLK_IDX : 8;
			unsigned rsvd24 : 8;
		} CTBIR0_bit;
	} ;	// 0x20


	/* PRU_CTRL_CTBIR1 register bit field */
	union {
		volatile uint32_t CTBIR1;

		volatile struct{
			unsigned C26_BLK_IDX : 8;
			unsigned rsvd8 : 8;
			unsigned C27_BLK_IDX : 8;
			unsigned rsvd24 : 8;
		} CTBIR1_bit;
	} ;	// 0x24


	/* PRU_CTRL_CTPPR0 register bit field */
	union {
		volatile uint32_t CTPPR0;

		volatile struct{
			unsigned C28_BLK_POINTER : 16;
			unsigned C29_BLK_POINTER : 16;
		} CTPPR0_bit;
	} ;	// 0x28


	/* PRU_CTRL_CTPPR1 register bit field */
	union {
		volatile uint32_t CTPPR1;

		volatile struct{
			unsigned C30_BLK_POINTER : 16;
			unsigned C31_BLK_POINTER : 16;
		} CTPPR1_bit;
	} ;	// 0x2C

} pruCtrl;

/* Definition of control register structures. */
#define PRU0_CTRL (*((volatile pruCtrl*)0x22000))
#define PRU1_CTRL (*((volatile pruCtrl*)0x24000))
