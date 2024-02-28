#pragma once


#ifdef __cplusplus
extern "C" {
#endif

void hrt_init(void);
uint64_t hrt_micros64(void);
uint64_t hrt_micros64_from_ISR(void); // from an ISR
uint32_t hrt_micros32(void);
uint32_t hrt_millis32(void);
uint32_t hrt_millis32I(void); // from locked context
uint32_t hrt_millis32_from_ISR(void); // from an ISR
uint64_t hrt_millis64(void);

#ifdef __cplusplus
}
#endif
