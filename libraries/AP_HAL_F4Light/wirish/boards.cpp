/******************************************************************************
 * The MIT License
 *
    (c) 2017 night_ghost@ykoctpa.ru
 
based on:
 
 * Copyright (c) 2010 Perry Hung.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

/**
 * @brief Generic board initialization routines.
 *
 */

#pragma GCC optimize ("O2")

#include "boards.h"
#include <usb.h>

static void setupNVIC(void);
static void enableFPU(void);
static void setupCCM(void);


void setupADC(void);
void setupTimers(void);
void usb_init(void);


void usb_init(void){


    usb_attr_t usb_attr;
    usb_open();

    usb_default_attr(&usb_attr);
    usb_attr.preempt_prio = USB_INT_PRIORITY;
    usb_attr.sub_prio = 0;
    usb_attr.use_present_pin = 1;
    usb_attr.present_port = PIN_MAP[BOARD_USB_SENSE].gpio_device;
    usb_attr.present_pin =  PIN_MAP[BOARD_USB_SENSE].gpio_bit;

    usb_configure(&usb_attr);

}

static INLINE void enableFPU(void){
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
	SCB->CPACR |= ((3UL << 10*2)|(3UL << 11*2));	// set CP10 and CP11 Full Access
	
/*
FPU_FPCCR_ASPEN_Msk
FPU_FPCCR_LSPEN_Msk
*/	
	
#endif
}




static INLINE void setupCCM(){
    extern unsigned _sccm,_eccm; // defined by link script
/* enabled by startup code
    RCC->AHB1ENR |= RCC_AHB1ENR_CCMDATARAMEN;
    asm volatile("dsb \n");
*/


//    volatile unsigned *src = &_siccm; // CCM initializers in flash
    volatile unsigned  *dest = &_sccm; // start of CCM

#if 0 // no support for initialized data in CCM

    while (dest < &_eccm) {
        *dest++ = *src++;
    }
#endif
    while (dest < &_eccm) {
        *dest++ = 0;
    }

// only for stack debugging
#if 0
    uint32_t sp;
    
    // Get stack pointer
    asm volatile ("mov %0, sp\n\t"  : "=rm" (sp) );

 #if 0 //  memset is much faster but uses too much stack for own needs
    memset((void *)dest,0x55, (sp-(uint32_t)dest) -128); 
 #else
    while ((uint32_t)dest < (sp-8)) {
        *dest++ = 0x55555555; // fill stack to check it's usage
    }
 #endif
#endif
}



static INLINE void setupNVIC()
{
    /* 4 bit preemption,  0 bit subpriority */
    NVIC_PriorityGroupConfig( NVIC_PriorityGroup_4);
    
    exti_init();
}


/*
[..] To enable access to the RTC Domain and RTC registers, proceed as follows:
    (+) Enable the Power Controller (PWR) APB1 interface clock using the
       RCC_APB1PeriphClockCmd() function.
   (+) Enable access to RTC domain using the PWR_BackupAccessCmd() function.
   (+) Select the RTC clock source using the RCC_RTCCLKConfig() function.
   (+) Enable RTC Clock using the RCC_RTCCLKCmd() function.
*/

void board_set_rtc_register(uint32_t sig, uint16_t reg)
{
        PWR->CR   |= PWR_CR_DBP;
    
        RTC_WriteBackupRegister(reg, sig);

        PWR->CR   &= ~PWR_CR_DBP;
}


uint32_t board_get_rtc_register(uint16_t reg)
{
        // enable the backup registers.
        PWR->CR   |= PWR_CR_DBP;

        uint32_t ret = RTC_ReadBackupRegister(reg);

        PWR->CR   &= ~PWR_CR_DBP;    
        return ret;
}


// 1st executing function

void inline init(void) {
    
    // now we can use stack

// turn on and enable RTC
    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
    RCC->AHB1ENR |= RCC_AHB1ENR_BKPSRAMEN;
    PWR_BackupAccessCmd(ENABLE);

    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
    RCC_RTCCLKCmd(ENABLE);

    // enable the backup registers.
    RCC->BDCR |= RCC_BDCR_RTCEN;

    RTC_WriteProtectionCmd(DISABLE);
    for(volatile int i=0; i<50; i++); // small delay
// RTC is ready
    if(board_get_rtc_register(RTC_SIGNATURE_REG) == DFU_RTC_SIGNATURE) {
        board_set_rtc_register(0, RTC_SIGNATURE_REG);
        for(volatile int i=0; i<50; i++); // small delay
        uint32_t reg=board_get_rtc_register(RTC_SIGNATURE_REG); // read again
        if(reg==0) {
            goDFU();        // just after reset - so all hardware is in boot state
        }
    }


    bool overclock_failed = false;
    uint8_t overclock=0;
    
    uint32_t g = board_get_rtc_register(RTC_OV_GUARD_REG);

    if(g == OV_GUARD_FAIL_SIGNATURE) {
        overclock_failed = true; // never reset it to 0 if failed once
    } else if(g == OV_GUARD_SIGNATURE) {  // overclock fails
        overclock_failed = true; // never reset it to 0 if failed once
        board_set_rtc_register(OV_GUARD_FAIL_SIGNATURE, RTC_OVERCLOCK_REG); // 
    } else {
        uint32_t sig=board_get_rtc_register(RTC_OVERCLOCK_REG);
        if((sig & ~OVERCLOCK_SIG_MASK ) == OVERCLOCK_SIGNATURE) {
            board_set_rtc_register(0, RTC_OVERCLOCK_REG); // 
            overclock = (uint8_t)sig & OVERCLOCK_SIG_MASK;
            
            if(overclock) {
                board_set_rtc_register(OV_GUARD_SIGNATURE, RTC_OV_GUARD_REG); // set guard in case overclock fails
            } else {
                board_set_rtc_register(0, RTC_OV_GUARD_REG); // clear guard
            }
        }
    }


    systemInit(overclock);       //  calls SetSysClock
    SystemCoreClockUpdate();     //  update SystemCoreClock variable to current frequency

    enableFPU();

    setupNVIC();
    systick_init(SYSTICK_RELOAD_VAL);

    stopwatch_init(); // will use stopwatch_delay_us() and stopwatch_get_ticks()

#ifdef DEBUG_BUILD
//*///    enable clock in sleep for debugging
    DBGMCU->CR |= DBGMCU_STANDBY | DBGMCU_STOP | DBGMCU_SLEEP;
    DBGMCU->APB1FZ |= DBGMCU_TIM4_STOP | DBGMCU_TIM5_STOP | DBGMCU_TIM7_STOP;  // stop internal timers
    DBGMCU->APB2FZ |= DBGMCU_TIM10_STOP | DBGMCU_TIM11_STOP;
//*///    
#endif

    boardInit(); // board-specific part of init
/*
     only CPU init here, all another moved to modules .init() functions
*/
    interrupts();

    if(!overclock_failed) {
        // comes here - all ok, we can clear guard
        board_set_rtc_register(0, RTC_OV_GUARD_REG); // 
    }

}

// called with stack in MSP
void pre_init(){ // before any stack usage @NG
    setupCCM(); // needs because stack in CCM

    init();
}

// частота неправильная и штатными функциями задержки мы не можем пользоваться
void emerg_delay(uint32_t n){
    volatile uint32_t i;

    while(n){
        for (i=4000; i!=0; i--) { // 16MHz, command each tick - ~4MHz or 0.25uS * 4000 = 1ms
            asm volatile("nop \n");
        }
        n--;
    }
}


void NMI_Handler() {

    //Очищаем флаг прерывания CSS
    RCC->CIR |= RCC_CIR_CSSC;
    //Ждем некоторое время после сбоя, если он кратковременный
    //Возможно удастся перезапустить
    emerg_delay(100);  // clock is wrong so all micros() etc lies!


    //Пытаемся запустить HSE
    RCC_HSEConfig(RCC_HSE_ON);
    emerg_delay(1);     //Задержка на запуск кварца


    if (RCC_WaitForHSEStartUp() == SUCCESS){
        //Если запустился - проводим установку заново
        SetSysClock(0); // without overclocking 

    } else {

// кварц не запустился, переключаемся на HSI и выставляем полную частоту

/* PLL_VCO = (HSE_VALUE or HSI_VALUE / PLL_M) * PLL_N */
#define PLL_M      8
#define PLL_N      168

/* SYSCLK = PLL_VCO / PLL_P */
#define PLL_P      2

/* USB OTG FS, SDIO and RNG Clock =  PLL_VCO / PLLQ */
#define PLL_Q      7

        /* Enable high performance mode, System frequency up to 168 MHz */
        RCC->APB1ENR |= RCC_APB1ENR_PWREN;
        PWR->CR |= PWR_CR_PMODE;  

        /* HCLK = SYSCLK / 1*/
        RCC->CFGR |= RCC_CFGR_HPRE_DIV1;
      
        /* PCLK2 = HCLK / 2*/
        RCC->CFGR |= RCC_CFGR_PPRE2_DIV2;
    
        /* PCLK1 = HCLK / 4*/
        RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;

        /* Configure the main PLL */
        RCC->PLLCFGR = PLL_M | (PLL_N << 6) | (((PLL_P >> 1) -1) << 16) |
                   (RCC_PLLCFGR_PLLSRC_HSI) | (PLL_Q << 24);

        /* Enable the main PLL */
        RCC->CR |= RCC_CR_PLLON;

        /* Wait till the main PLL is ready */
        while((RCC->CR & RCC_CR_PLLRDY) == 0)   {   } // TODO: do something on failure?
   
        /* Select the main PLL as system clock source */
        RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
        RCC->CFGR |= RCC_CFGR_SW_PLL;    
        
        SystemCoreClock=168000000;
    }
}

