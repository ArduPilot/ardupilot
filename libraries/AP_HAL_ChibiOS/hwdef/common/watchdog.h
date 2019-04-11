
#ifdef __cplusplus
extern "C" {
#endif

/*
  setup the watchdog
 */
void stm32_watchdog_init(void);

/*
  pat the dog, to prevent a reset. If not called for 1s
  after stm32_watchdog_init() then MCU will reset
 */
void stm32_watchdog_pat(void);

#ifdef __cplusplus
}
#endif
    
