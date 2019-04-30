
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

/*
  return true if reboot was from a watchdog reset
 */
bool stm32_was_watchdog_reset(void);

/*
  save the reset reason code
 */
void stm32_watchdog_save_reason(void);

/*
  clear reset reason code
 */
void stm32_watchdog_clear_reason(void);

/*
  set the safety state in backup register
 */
void stm32_set_backup_safety_state(bool safety_on);

/*
  get the safety state in backup register from initial boot
*/
bool stm32_get_boot_backup_safety_state(void);

/*
  set the armed state in backup register
 */
void stm32_set_backup_armed(bool armed);

/*
  get the armed state in backup register from initial boot
*/
bool stm32_get_boot_backup_armed(void);

/*
  set home state in backup
 */
void stm32_set_backup_home(int32_t lat, int32_t lon, int32_t alt_cm);

/*
  get home state from backup
 */
void stm32_get_backup_home(int32_t *lat, int32_t *lon, int32_t *alt_cm);

/*
  set attitude in backup
 */
void stm32_set_attitude(int32_t roll_cd, int32_t pitch_cd, int32_t yaw_cd);

/*
  get attitude from backup
 */
void stm32_get_attitude(int32_t *roll_cd, int32_t *pitch_cd, int32_t *yaw_cd);
    
#ifdef __cplusplus
}
#endif
    
