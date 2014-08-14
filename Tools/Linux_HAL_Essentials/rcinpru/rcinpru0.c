/*
 * testpru
 *
 */

#define PRU0
#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <strings.h>

#include "linux_types.h"
#include "pru_defs.h"
#include "prucomm.h"


void add_to_ring_buffer(uint8_t v, uint16_t deltat)
{
    RBUFF->buffer[RBUFF->ring_tail].pin_value = v;
    RBUFF->buffer[RBUFF->ring_tail].delta_t = deltat;
    RBUFF->ring_tail = (RBUFF->ring_tail + 1) % NUM_RING_ENTRIES;
}

static inline u32 read_PIEP_COUNT(void)
{
    return PIEP_COUNT;
}

uint32_t read_pin(void){
    return ((__R31&(1<<15)) != 0);
}

void main()
{
     uint32_t last_time_us = 0;
     uint8_t last_pin_value = 0;
     
     /*PRU Initialisation*/
     PRUCFG_SYSCFG &= ~SYSCFG_STANDBY_INIT;
     PRUCFG_SYSCFG = (PRUCFG_SYSCFG &
             ~(SYSCFG_IDLE_MODE_M | SYSCFG_STANDBY_MODE_M)) |
             SYSCFG_IDLE_MODE_NO | SYSCFG_STANDBY_MODE_NO;

     /* our PRU wins arbitration */
     PRUCFG_SPP |=  SPP_PRU1_PAD_HP_EN;
    
    /* configure timer */
    PIEP_GLOBAL_CFG = GLOBAL_CFG_DEFAULT_INC(1) |
              GLOBAL_CFG_CMP_INC(1);
    PIEP_CMP_STATUS = CMD_STATUS_CMP_HIT(1); /* clear the interrupt */
        PIEP_CMP_CMP1   = 0x0;
    PIEP_CMP_CFG |= CMP_CFG_CMP_EN(1);
        PIEP_GLOBAL_CFG |= GLOBAL_CFG_CNT_ENABLE;

     
     RBUFF->ring_tail = 20;
     while (1) {
        uint32_t v;
        while ((v=read_pin()) == last_pin_value) {
          // noop
        }
        uint32_t now = read_PIEP_COUNT()/200;
        uint32_t delta_time_us = now - last_time_us;
        last_time_us = now;

        add_to_ring_buffer(last_pin_value, delta_time_us);
        last_pin_value = v;
     }
}
