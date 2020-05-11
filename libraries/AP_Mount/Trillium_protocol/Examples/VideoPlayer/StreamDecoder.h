#ifndef STREAMDECODER_H
#define STREAMDECODER_H

#include <stdint.h>

int StreamOpen(const char *pUrl, const char *pRecordPath);
void StreamClose(void);

int StreamProcess(void);

int StreamGetVideoFrame(uint8_t *pFrameData, int *pWidth, int *pHeight, int MaxBytes);
int StreamGetMetaData(uint8_t *pMetaData, int *pBytes, int MaxBytes);

#endif // STREAMDECODER_H
