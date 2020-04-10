#include "KlvTree.h"

#include "fielddecode.h"
#include "scaleddecode.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <math.h>

typedef struct KlvTag_t
{
    uint8_t Key;
    uint32_t Length;
    uint8_t *pData;
    struct KlvTag_t *pNext;
} KlvTag_t;

static KlvTag_t *pHeadTag = NULL;

KlvTag_t *KlvCTrereateTag(void)
{
    KlvTag_t *pTag = (KlvTag_t *)calloc(1, sizeof(KlvTag_t)), *pHead = pHeadTag;

    // As long as we have a pointer to a tag and its next sibling, keep iterating
    while (pHead && pHead->pNext) { pHead = pHead->pNext; }

    // If we have a valid tag pointer, attach the new node as its sibling
    if (pHead) pHead->pNext = pTag;
    
    // Return a pointer to the new tag
    return pTag;

}// KlvCTrereateTag

KlvTag_t *KlvFindTag(uint8_t Key)
{
    // Start at the head tag
    KlvTag_t *pHead = pHeadTag;

    // As long as we have a valid tag pointer
    while (pHead)
    {
        // If this is the one we're looking for, stop looping
        if (pHead->Key == Key)
            break;
        // Otherwise, move down to the next element in the list
        else
            pHead = pHead->pNext;
    }

    // Return a pointer to the given tag, or NULL if no result
    return pHead;

}// KlvFindTag

int KlvTreeSetValue(uint8_t Key, uint32_t Length, const uint8_t *pData)
{
    KlvTag_t *pTag = KlvFindTag(Key);

    // If we found an existing tag or are able to create a new one
    if (pTag || (pTag = KlvCTrereateTag()))
    {
        // Copy in the key and length, and also try to allocate enough space for the data
        pTag->Key = Key;
        pTag->Length = Length;
        pTag->pData = (uint8_t *)realloc(pTag->pData, Length);

        // If we have a good data pointer
        if (pTag->pData)
        {
            // If we don't have a valid head element yet, install this one
            if (pHeadTag == NULL)
                pHeadTag = pTag;

            // Copy in the incoming data and return 1 to indicate success
            memcpy(pTag->pData, pData, Length);
            return 1;
        }
    }

    // No dice - return 0
    return 0;
    
}// KlvTreeSetValue

double KlvTreeGetValueDouble(uint8_t Key, double Min, double Max, int *pResult)
{
    // Try to grab the right tag from our linked list
    KlvTag_t *pTag = KlvFindTag(Key);
    double Value = 0;

    // Check for valid tag and data array
    if (pTag && pTag->pData)
    {
        // Compute the inverse scale for this element
        double Scale = (Max - Min) / (pow(2, pTag->Length * 8));
        int Index = 0;

        // Default result is success
        *pResult = 1;

        // If this is an asymmetrical value, we have to decode as unsigned and convert
        if (Min + Max != 0)
        {
            // Switch on tag length
            switch (pTag->Length)
            {
            // Use the appropriate Protogen function to get a value
            case 1: Value = float64ScaledFrom1UnsignedBytes(pTag->pData,   &Index, Min, Scale); break;
            case 2: Value = float64ScaledFrom2UnsignedBeBytes(pTag->pData, &Index, Min, Scale); break;
            case 3: Value = float64ScaledFrom3UnsignedBeBytes(pTag->pData, &Index, Min, Scale); break;
            case 4: Value = float64ScaledFrom4UnsignedBeBytes(pTag->pData, &Index, Min, Scale); break;
            case 5: Value = float64ScaledFrom5UnsignedBeBytes(pTag->pData, &Index, Min, Scale); break;
            case 6: Value = float64ScaledFrom6UnsignedBeBytes(pTag->pData, &Index, Min, Scale); break;
            case 7: Value = float64ScaledFrom7UnsignedBeBytes(pTag->pData, &Index, Min, Scale); break;
            case 8: Value = float64ScaledFrom8UnsignedBeBytes(pTag->pData, &Index, Min, Scale); break;

            // Unhandled length: no dice
            default:
                *pResult = 0;
                break;
            }
        }
        else
        {
            // Switch on tag length
            switch (pTag->Length)
            {
            // Use the appropriate Protogen function to get a value
            case 1: Value = float64ScaledFrom1SignedBytes(pTag->pData,   &Index, Scale); break;
            case 2: Value = float64ScaledFrom2SignedBeBytes(pTag->pData, &Index, Scale); break;
            case 3: Value = float64ScaledFrom3SignedBeBytes(pTag->pData, &Index, Scale); break;
            case 4: Value = float64ScaledFrom4SignedBeBytes(pTag->pData, &Index, Scale); break;
            case 5: Value = float64ScaledFrom5SignedBeBytes(pTag->pData, &Index, Scale); break;
            case 6: Value = float64ScaledFrom6SignedBeBytes(pTag->pData, &Index, Scale); break;
            case 7: Value = float64ScaledFrom7SignedBeBytes(pTag->pData, &Index, Scale); break;
            case 8: Value = float64ScaledFrom8SignedBeBytes(pTag->pData, &Index, Scale); break;

            // Unhandled length: no dice
            default:
                *pResult = 0;
                break;
            }
        }
    }
    // No tag means a bogus value, so zero out the result flag
    else
        *pResult = 0;

    return Value;

}// KlvTreeGetValueDouble

int64_t KlvTreeGetValueInt(uint8_t Key, int *pResult)
{
    // Try to grab the right tag from our linked list
    KlvTag_t *pTag = KlvFindTag(Key);
    int64_t Value = 0;

    // Check for valid tag and data array
    if (pTag && pTag->pData)
    {
        int Index = 0;

        // Default result is success
        *pResult = 1;

        // Switch on tag length
        switch (pTag->Length)
        {
        // Use the appropriate Protogen function to get a value
        case 1: Value = int8FromBytes(pTag->pData,    &Index); break;
        case 2: Value = int16FromBeBytes(pTag->pData, &Index); break;
        case 3: Value = int24FromBeBytes(pTag->pData, &Index); break;
        case 4: Value = int32FromBeBytes(pTag->pData, &Index); break;
        case 5: Value = int40FromBeBytes(pTag->pData, &Index); break;
        case 6: Value = int48FromBeBytes(pTag->pData, &Index); break;
        case 7: Value = int56FromBeBytes(pTag->pData, &Index); break;
        case 8: Value = int64FromBeBytes(pTag->pData, &Index); break;

        // Unhandled length: no dice
        default:
            *pResult = 0;
            break;
        }
    }
    // No tag means a bogus value, so zero out the result flag
    else
        *pResult = 0;

    return Value;

}// KlvTreeGetValueInt

uint64_t KlvTreeGetValueUInt(uint8_t Key, int *pResult)
{
    // Try to grab the right tag from our linked list
    KlvTag_t *pTag = KlvFindTag(Key);
    uint64_t Value = 0;

    // Check for valid tag and data array
    if (pTag && pTag->pData)
    {
        int Index = 0;

        // Default result is success
        *pResult = 1;

        // Switch on tag length
        switch (pTag->Length)
        {
        // Use the appropriate Protogen function to get a value
        case 1: Value = uint8FromBytes(pTag->pData,    &Index); break;
        case 2: Value = uint16FromBeBytes(pTag->pData, &Index); break;
        case 3: Value = uint24FromBeBytes(pTag->pData, &Index); break;
        case 4: Value = uint32FromBeBytes(pTag->pData, &Index); break;
        case 5: Value = uint40FromBeBytes(pTag->pData, &Index); break;
        case 6: Value = uint48FromBeBytes(pTag->pData, &Index); break;
        case 7: Value = uint56FromBeBytes(pTag->pData, &Index); break;
        case 8: Value = uint64FromBeBytes(pTag->pData, &Index); break;

        // Unhandled length: no dice
        default:
            *pResult = 0;
            break;
        }
    }
    // No tag means a bogus value, so zero out the result flag
    else
        *pResult = 0;

    return Value;

}// KlvTreeGetValueInt

const char *KlvTreeGetValueString(uint8_t Key)
{
    // If we can find the right tag, just cast its value data to a string and return
    KlvTag_t *pTag = KlvFindTag(Key);
    return (pTag && pTag->pData) ? (const char *)pTag->pData : 0;

}// KlvTreeGetValueString

void KlvTreePrint(void)
{
    KlvTag_t *pTag = pHeadTag;

    while (pTag)
    {
        int i;

        printf("TAG: Key = 0x%02x, Length = %d, Value = { ", pTag->Key, pTag->Length);

        for (i = 0; i < pTag->Length; i++)
            printf("0x%02x%s", pTag->pData[i], (i < pTag->Length - 1) ? ", " : " }\n");

        pTag = pTag->pNext;
    }

}
