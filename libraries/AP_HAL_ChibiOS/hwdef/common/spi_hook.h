#pragma once

#include "hal.h"

#ifdef __cplusplus
extern "C" {
#endif

#if HAL_USE_MMC_SPI == TRUE
void spiStartHook(SPIDriver *spip, const SPIConfig *config);
void spiStopHook(SPIDriver *spip);
void spiSelectHook(SPIDriver *spip);
void spiUnselectHook(SPIDriver *spip);
void spiIgnoreHook(SPIDriver *spip,size_t n);
void spiSendHook(SPIDriver *spip,size_t n, const void *txbuf);
void spiReceiveHook(SPIDriver *spip, size_t n, void *rxbuf);
#endif

#ifdef __cplusplus
}
#endif
