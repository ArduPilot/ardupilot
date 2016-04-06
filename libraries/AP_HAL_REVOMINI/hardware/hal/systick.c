#include <systick.h>
#include <hal.h>
#include <timer.h>


volatile uint64_t systick_uptime_millis;
volatile uint32_t uart1_lic_millis;
volatile uint32_t uart2_lic_millis;
volatile uint32_t uart3_lic_millis;
volatile uint32_t uart4_lic_millis;
volatile uint32_t uart5_lic_millis;
volatile uint32_t uart6_lic_millis;


static void (*systick_user_callback)(void);

/**
 * @brief Initialize and enable SysTick.
 *
 * Clocks the system timer with the core clock, turns it on, and
 * enables interrupts.
 *
 * @param reload_val Appropriate reload counter to tick every 1 ms.
 */
void systick_init(uint32_t reload_val) {
    SysTick->LOAD = reload_val;
    systick_enable();
}

/**
 * Clock the system timer with the core clock, but don't turn it
 * on or enable interrupt.
 */
void systick_disable() {
    SysTick->CTRL = SysTick_CTRL_CLKSOURCE_Msk;
}

/**
 * Clock the system timer with the core clock and turn it on;
 * interrupt every 1 ms, for systick_timer_millis.
 */
void systick_enable() {
    /* re-enables init registers without changing reload val */
	SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
						SysTick_CTRL_TICKINT_Msk   |
						SysTick_CTRL_ENABLE_Msk;
                                            
}

/**
 * @brief Attach a callback to be called from the SysTick exception handler.
 *
 * To detach a callback, call this function again with a null argument.
 */
void systick_attach_callback(void (*callback)(void)) {
    systick_user_callback = callback;
}

/*
 * SysTick ISR
 */



void SysTick_Handler(void)
{
    systick_uptime_millis++;
    uart1_lic_millis++;
    uart2_lic_millis++;
    uart3_lic_millis++;
    uart4_lic_millis++;

    if (systick_user_callback) {
        systick_user_callback();
    }
}

#define LED_GRN (*((unsigned long int *) 0x42408294)) // PB5
#define LED_YLW (*((unsigned long int *) 0x42408298)) // PB6 // Not included
#define LED_RED (*((unsigned long int *) 0x42408290)) // PB4

void HardFault_Handler(void)
{
    timer_disable_all();
    /* Go to infinite loop when Hard Fault exception occurs */
    while (1) {
	LED_YLW = 0;
        LED_RED = 0;
        LED_GRN = 1;
    }
}
/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
    timer_disable_all();
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
	  LED_YLW = 0;
	  LED_GRN = 0;
	  LED_RED = 1;
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
    timer_disable_all();
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
	  LED_YLW = 0;
	  LED_GRN = 0;
	  LED_RED = 0;
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
    timer_disable_all();
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
	  LED_YLW = 0;
	  LED_GRN = 1;
	  LED_RED = 1;
  }
}

