/*
  MCU tables for STM32H7
 */

#if defined(STM32H7)

#define STM32_UNKNOWN 0
#define STM32_H743    0x450

mcu_des_t mcu_descriptions[] = {
    { STM32_UNKNOWN,     "STM32H7???",    '?'},
    { STM32_H743,        "STM32H743/753", '?'},
};

const mcu_rev_t silicon_revs[] = {
    {0x1001, 'Z', false},
    {0x1003, 'Y', false},
    {0x2001, 'X', false},
    {0x2003, 'V', false},
};

#endif // STM32H7
