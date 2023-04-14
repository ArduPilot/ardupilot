/*
  MCU tables for STM32H7
 */

#if defined(STM32H7)

#define STM32_UNKNOWN 0
#define STM32_H743    0x450

mcu_des_t mcu_descriptions[] = {
    { STM32_UNKNOWN,     "STM32H7???" },
    { STM32_H743,        "STM32H743/753" },
};

const mcu_rev_t silicon_revs[] = {
    {0x1001, 'Z'},
    {0x1003, 'Y'},
    {0x2001, 'X'},
    {0x2003, 'V'},
};

#endif // STM32H7
