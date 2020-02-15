/*
  support tables for STM32F4
 */

#if defined(STM32F4)
/* magic numbers from reference manual */
enum {
    MCU_REV_STM32F4_REV_A = 0x1000,
    MCU_REV_STM32F4_REV_Z = 0x1001,
    MCU_REV_STM32F4_REV_Y = 0x1003,
    MCU_REV_STM32F4_REV_1 = 0x1007,
    MCU_REV_STM32F4_REV_3 = 0x2001
};


#define STM32_UNKNOWN	0
#define STM32F40x_41x	0x413
#define STM32F42x_43x	0x419
#define STM32F42x_446xx	0x421

// The default CPU ID  of STM32_UNKNOWN is 0 and is in offset 0
// Before a rev is known it is set to ?
// There for new silicon will result in STM32F4..,?
const mcu_des_t mcu_descriptions[] = {
    { STM32_UNKNOWN,	"STM32F???",    '?'},
    { STM32F40x_41x, 	"STM32F40x",	'?'},
    { STM32F42x_43x, 	"STM32F42x",	'?'},
    { STM32F42x_446xx, 	"STM32F446XX",	'?'},
};

const mcu_rev_t silicon_revs[] = {
    {MCU_REV_STM32F4_REV_3, '3', false}, /* Revision 3 */

    {MCU_REV_STM32F4_REV_A, 'A', true}, /* Revision A */
    {MCU_REV_STM32F4_REV_Z, 'Z', true}, /* Revision Z */
    {MCU_REV_STM32F4_REV_Y, 'Y', true}, /* Revision Y */
    {MCU_REV_STM32F4_REV_1, '1', true}, /* Revision 1 */
};

#endif // STM32F4

