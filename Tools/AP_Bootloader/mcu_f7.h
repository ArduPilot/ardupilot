/*
  MCU tables for STM32F7
 */

#if defined(STM32F7)

#define STM32_UNKNOWN	0
#define STM32F74x_75x	0x449
#define STM32F76x_77x	0x451

typedef enum mcu_rev_e {
    MCU_REV_STM32F7_REV_A = 0x1000,
    MCU_REV_STM32F7_REV_Z = 0x1001,
} mcu_rev_e;

// The default CPU ID  of STM32_UNKNOWN is 0 and is in offset 0
// Before a rev is known it is set to ?
// There for new silicon will result in STM32F4..,?
mcu_des_t mcu_descriptions[] = {
    { STM32_UNKNOWN,	"STM32F??????" },
    { STM32F74x_75x, 	"STM32F7[4|5]x" },
    { STM32F76x_77x, 	"STM32F7[6|7]x" },
};

const mcu_rev_t silicon_revs[] = {
    {MCU_REV_STM32F7_REV_A, 'A'}, /* Revision A */
    {MCU_REV_STM32F7_REV_Z, 'Z'}, /* Revision Z */
};

#endif // STM32F7
