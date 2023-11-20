#pragma once


#ifdef __cplusplus
extern "C" {
#endif
void hrt_init(void);
/* inlining these gives a 20% speed improvement on external flash */
static inline uint64_t hrt_micros64(void);
static inline uint32_t hrt_micros32(void);
static inline uint32_t hrt_millis32(void);
uint32_t get_systime_us32(void);

#ifdef __cplusplus
}
#endif

/*
  for the exposed functions we use chSysGetStatusAndLockX() to prevent
  an interrupt changing the globals while allowing this call from any
  context
*/
extern uint64_t timer_base_us64;

uint64_t hrt_micros64()
{
    syssts_t sts = chSysGetStatusAndLockX();
    uint32_t now = get_systime_us32();
    uint64_t ret = timer_base_us64 + now;
    chSysRestoreStatusX(sts);
    return ret;
}

uint32_t hrt_micros32()
{
    syssts_t sts = chSysGetStatusAndLockX();
    uint32_t ret = get_systime_us32();
    chSysRestoreStatusX(sts);
    return ret;
}

uint32_t hrt_millis32()
{
    return (uint32_t)(hrt_micros64() / 1000U);
}
