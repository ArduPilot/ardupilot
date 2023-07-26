#include <AP_HAL/AP_HAL.h>

#ifdef WITH_INT_OSD

#include "freertos/FreeRTOS.h"

#include "soc/mcpwm_struct.h"
#include "soc/mcpwm_reg.h"
#include "soc/i2s_struct.h"
#include "soc/i2s_reg.h"

#include "driver/mcpwm.h"
#include "driver/i2s.h"

#include <AP_Math/AP_Math.h>
#include <AP_OSD/AP_OSD_INT.h>

#include <stdio.h>

#define LINE_S 22
#define LINE_F 310

uint32_t *osd_buffer_mask;
uint32_t *osd_buffer_levl;
uint32_t line;

//We use assembly cause high priority interrupt can not be written in C.
//C reference version is below.
__asm__(R"(
    .data
_l5_intr_stack:
    .space      36
    .section        .iram1,"ax",@progbits
    .literal_position
    .literal .LC100, 1073078548
    .literal .LC102, 1073078524
    .literal .LC103, line
    .literal .LC104, osd_buffer_mask
    .literal .LC105, osd_buffer_levl
    .literal .LC106, 1073016840
    .literal .LC107, 1073139720
    .literal .LC108, 1073016832
    .literal .LC109, 1073139712
    .literal .LC110, 1073078556
    .align  4
    .global xt_highint5
    .type   xt_highint5, @function
xt_highint5:
    movi    a0, _l5_intr_stack
    s32i    a8, a0, 0
    s32i    a9, a0, 4
    s32i    a10, a0, 8
    s32i    a11, a0, 12
    s32i    a12, a0, 16
    s32i    a13, a0, 20
    s32i    a14, a0, 24
    s32i    a15, a0, 28
    s32i    a2, a0, 32

    l32r    a8, .LC100
    l32i.n  a8, a8, 0
    bbci    a8, 27, .L103
    l32r    a8, .LC102
    movi    a10, 0x62
    l32i.n  a9, a8, 0
    movi    a8, -0x12d
    add.n   a8, a9, a8
    bltu    a10, a8, .L104
    l32r    a10, .LC103
    l32i.n  a8, a10, 0
    addi.n  a9, a8, 1
    s32i.n  a9, a10, 0
    addi    a8, a8, -21
    movi    a10, 0x11f
    bltu    a10, a8, .L103
    l32r    a8, .LC104
    l32r    a14, .LC106
    l32i.n  a10, a8, 0
    l32r    a8, .LC105
    movi.n  a12, 0
    l32r    a13, .LC107
    l32i.n  a8, a8, 0
    s32i.n  a12, a14, 0
    s32i.n  a12, a13, 0
    movi.n  a11, 1
    s32i.n  a11, a14, 0
    s32i.n  a11, a13, 0
    addx2   a9, a9, a9
    movi    a11, -0x108
    addx4   a9, a9, a11
    s32i.n  a12, a14, 0
    slli    a9, a9, 2
    s32i.n  a12, a13, 0
    add.n   a10, a10, a9
    l32r    a11, .LC108
    l32i.n  a15, a10, 0
    add.n   a8, a8, a9
    s32i.n  a15, a11, 0
    l32r    a9, .LC109
    l32i.n  a2, a8, 0
    movi.n  a15, 0x10
    s32i.n  a2, a9, 0
    s32i.n  a15, a14, 0
    s32i.n  a15, a13, 0
    l32i.n  a13, a10, 4
    s32i.n  a13, a11, 0
    l32i.n  a13, a8, 4
    s32i.n  a13, a9, 0
    l32i.n  a13, a10, 8
    s32i.n  a13, a11, 0
    l32i.n  a13, a8, 8
    s32i.n  a13, a9, 0
    l32i.n  a13, a10, 12
    s32i.n  a13, a11, 0
    l32i.n  a13, a8, 12
    s32i.n  a13, a9, 0
    l32i.n  a13, a10, 16
    s32i.n  a13, a11, 0
    l32i.n  a13, a8, 16
    s32i.n  a13, a9, 0
    l32i.n  a13, a10, 20
    s32i.n  a13, a11, 0
    l32i.n  a13, a8, 20
    s32i.n  a13, a9, 0
    l32i.n  a13, a10, 24
    s32i.n  a13, a11, 0
    l32i.n  a13, a8, 24
    s32i.n  a13, a9, 0
    l32i.n  a13, a10, 28
    s32i.n  a13, a11, 0
    l32i.n  a13, a8, 28
    s32i.n  a13, a9, 0
    l32i.n  a13, a10, 32
    s32i.n  a13, a11, 0
    l32i.n  a13, a8, 32
    s32i.n  a13, a9, 0
    l32i.n  a13, a10, 36
    s32i.n  a13, a11, 0
    l32i.n  a13, a8, 36
    s32i.n  a13, a9, 0
    l32i.n  a13, a10, 40
    s32i.n  a13, a11, 0
    l32i.n  a13, a8, 40
    s32i.n  a13, a9, 0
    l32i.n  a10, a10, 44
    s32i.n  a10, a11, 0
    l32i.n  a8, a8, 44
    s32i.n  a8, a9, 0
    s32i.n  a12, a11, 0
    s32i.n  a12, a9, 0
    j   .L103
.L104:
    movi    a8, -0x7d1
    add.n   a9, a9, a8
    movi    a8, 0x1f2
    bltu    a8, a9, .L103
    l32r    a8, .LC103
    movi.n  a9, 0
    s32i.n  a9, a8, 0
.L103:
    l32r    a8, .LC110
    movi.n  a9, -1
    s32i.n  a9, a8, 0

    movi    a0, _l5_intr_stack
    l32i    a8, a0, 0
    l32i    a9, a0, 4
    l32i    a10, a0, 8
    l32i    a11, a0, 12
    l32i    a12, a0, 16
    l32i    a13, a0, 20
    l32i    a14, a0, 24
    l32i    a15, a0, 28
    l32i    a2, a0, 32
    rsync
    memw
    rsr     a0, 213
    rfi     5
)");

inline bool normal_sync(uint32_t sync) {
    return (sync > 300) && (sync < 400);
}

inline bool long_sync(uint32_t sync) {
    return (sync > 2000) && (sync < 2500);
}

//This function is not linked in the final binary,
//it exist only to generate xt_highint5 above.
//In xt_hightint we have fixed prolog and epilog and we removed
//some 'memw' instructions, otherwise they are the same.
void IRAM_ATTR __attribute__((optimize("O3"))) osd_mcpwm_isr(void *) {
    if ((READ_PERI_REG(MCMCPWM_INT_RAW_MCPWM_REG(0)) & MCPWM_CAP0_INT_RAW_M)
            != 0) {
        uint32_t sync = READ_PERI_REG(MCPWM_CAP_CH0_REG(0));
        if (normal_sync(sync)) {
            line++;
            if (line >= LINE_S && line < LINE_F) {
                int disp = (line - LINE_S) * (AP_OSD_INT::video_x / 32);
                uint32_t *tmp_mask = osd_buffer_mask + disp;
                uint32_t *tmp_levl = osd_buffer_levl + disp;

                WRITE_PERI_REG(I2S_CONF_REG(0), 0);
                WRITE_PERI_REG(I2S_CONF_REG(1), 0);

                WRITE_PERI_REG(I2S_CONF_REG(0), I2S_TX_RESET_M);
                WRITE_PERI_REG(I2S_CONF_REG(1), I2S_TX_RESET_M);

                WRITE_PERI_REG(I2S_CONF_REG(0), 0);
                WRITE_PERI_REG(I2S_CONF_REG(1), 0);

                WRITE_PERI_REG(REG_I2S_BASE(0), tmp_mask[0]);
                WRITE_PERI_REG(REG_I2S_BASE(1), tmp_levl[0]);

                WRITE_PERI_REG(I2S_CONF_REG(0), I2S_TX_START_M);
                WRITE_PERI_REG(I2S_CONF_REG(1), I2S_TX_START_M);

                for (int ix = 1; ix < AP_OSD_INT::video_x / 32; ix++) {
                    WRITE_PERI_REG(REG_I2S_BASE(0), tmp_mask[ix]);
                    WRITE_PERI_REG(REG_I2S_BASE(1), tmp_levl[ix]);
                }

                WRITE_PERI_REG(REG_I2S_BASE(0), 0);
                WRITE_PERI_REG(REG_I2S_BASE(1), 0);

            }
        } else if (long_sync(sync)) {
            line = 0;
        }
    }
    WRITE_PERI_REG(MCMCPWM_INT_CLR_MCPWM_REG(0), 0xFFFFFFFF);
}

void config_mcpwm()
{
    periph_module_enable(PERIPH_PWM0_MODULE);
    MCPWM0.cap_timer_cfg.timer_en = 1;
    MCPWM0.cap_timer_cfg.synci_en = 1;
    MCPWM0.cap_timer_cfg.synci_sel = 4; //SYNC0
    MCPWM0.cap_timer_phase = 0;
    MCPWM0.int_ena.cap0_int_ena = 1;
    MCPWM0.cap_cfg_ch[0].en = 1;
    MCPWM0.cap_cfg_ch[0].mode = (1 << MCPWM_NEG_EDGE);
    MCPWM0.cap_cfg_ch[0].prescale = 0;
}

void config_i2s(i2s_dev_t *I2S) {
    if (I2S == &I2S1) {
        periph_module_enable(PERIPH_I2S1_MODULE);
    } else {
        periph_module_enable(PERIPH_I2S0_MODULE);
    }

    I2S->conf2.val = 0;
    
    I2S->pdm_conf.val = 0;
    
    I2S->conf_chan.tx_chan_mod = 0;
    
    I2S->fifo_conf.tx_fifo_mod = 0;   
    I2S->fifo_conf.dscr_en = 0; //no dma
    I2S->fifo_conf.tx_fifo_mod_force_en = 1;

    I2S->conf.val = 0;   
        
    I2S->clkm_conf.clka_en = 0;
    I2S->clkm_conf.clkm_div_a = 63;
    I2S->clkm_conf.clkm_div_b = 0;
    I2S->clkm_conf.clkm_div_num = 2;
    
    I2S->sample_rate_conf.tx_bck_div_num = 12;
    I2S->sample_rate_conf.tx_bits_mod = 16;
}

void config_gpio()
{
    gpio_set_direction(OSD_SYNC_PIN, GPIO_MODE_INPUT);
    gpio_set_direction(OSD_MASK_PIN, GPIO_MODE_OUTPUT);
    gpio_set_direction(OSD_LEVL_PIN, GPIO_MODE_OUTPUT);

    gpio_matrix_in(OSD_SYNC_PIN, PWM0_SYNC0_IN_IDX, true);
    gpio_matrix_in(OSD_SYNC_PIN, PWM0_CAP0_IN_IDX, true);

    gpio_matrix_out(OSD_MASK_PIN, I2S0O_DATA_OUT23_IDX, true, false);
    gpio_matrix_out(OSD_LEVL_PIN, I2S1O_DATA_OUT23_IDX, false, false);
}

void config_isr()
{
    esp_err_t err = esp_intr_alloc(ETS_PWM0_INTR_SOURCE, ESP_INTR_FLAG_IRAM|ESP_INTR_FLAG_LEVEL5,
                   nullptr, nullptr, nullptr);
    printf("alloc intr error code %d\n", err);
}

void osd_setup(AP_OSD_INT *d)
{
    printf("osd setup start %d\n", xPortGetCoreID());
    osd_buffer_mask = &(d->frame_mask[0][0]);
    osd_buffer_levl = &(d->frame_levl[0][0]);
    config_mcpwm();
    config_i2s(&I2S0);
    config_i2s(&I2S1);
    config_gpio();
    config_isr();
    printf("osd setup finish\n");
}

#endif
