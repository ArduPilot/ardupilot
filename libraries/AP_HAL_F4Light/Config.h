// no includes from here! only defines in one place
#pragma once

#define DEBUG_BUILD 1

#define USART_SAFE_INSERT // ignore received bytes if buffer overflows

#define USE_WFE


#define F4Light_RC_INPUT_MIN_CHANNELS 4
#define F4Light_RC_INPUT_NUM_CHANNELS 20

#define USE_MPU // guard page in process stack


//#define DEBUG_LOOP_TIME for AP_Scheduler


#ifdef DEBUG_BUILD
// profiling
//#define ISR_PERF - now all time-consuming calculations moved out from ISR to io_completion level
//#define SEM_PROF - now semaphores are part of scheduler
#define SHED_PROF 
#define MTASK_PROF

//#define SHED_DEBUG
//#define SEM_DEBUG
//#define MPU_DEBUG
//#define I2C_DEBUG
//#define DEBUG_SPI

#endif

/*
 interrupts priorities:
*/

#define PWM_INT_PRIORITY       0 // PWM input (10uS between interrupts)
#define SOFT_UART_INT_PRIORITY 1 // soft_uart
#define I2C_INT_PRIORITY       2 // i2c 
#define TIMER_I2C_INT_PRIORITY 3 // timer_i2C (2uS between interrupts)
#define MICROS_INT_PRIORITY    4 // micros() Timer5
#define SYSTICK_INT_PRIORITY   5 // SysTick
#define UART_INT_PRIORITY      6 // uart
#define DMA_IOC_INT_PRIORITY   7 // dma IO complete
#define GPIO_INT_PRIORITY      8 // gpio pin
#define MPU_INT_PRIORITY       9 // MPU6000 DataReady
#define VSI_INT_PRIORITY      10 // OSD VSI
#define USB_INT_PRIORITY      11 // usb
#define IOC_INT_PRIORITY      12 // driver's io_completion
//                            13
#define SVC_INT_PRIORITY      14 // scheduler - Timer7, tail timer, svc
#define PENDSV_INT_PRIORITY   15 // Pend_Sw


#define SPI_INT_PRIORITY I2C_INT_PRIORITY

