#pragma once


#ifdef __cplusplus
extern "C" {
#endif
void hrt_init(void);
uint64_t hrt_micros64(void);
uint32_t hrt_micros32(void);
uint32_t hrt_millis32(void);
#ifdef __cplusplus
}
#endif
