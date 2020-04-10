#ifndef KLVTREE_H
#define KLVTREE_H

#include <stdint.h>

int KlvTreeSetValue(uint8_t Key, uint32_t Length, const uint8_t *pData);

double KlvTreeGetValueDouble(uint8_t Key, double Min, double Max, int *pResult);
int64_t KlvTreeGetValueInt(uint8_t Key, int *pResult);
uint64_t KlvTreeGetValueUInt(uint8_t Key, int *pResult);
const char *KlvTreeGetValueString(uint8_t Key);

void KlvTreePrint(void);

#endif // KLVTREE_H
