/*
  example outputting DShot on 4 TIM1 channels on fmuv4 board
 */

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include "ch.h"
#include "hal.h"
#include "hwdef.h"

void setup();
void loop();

const AP_HAL::HAL& hal = AP_HAL::get_HAL();

// use TIM1_CH1 to TIM1_CH4
#define DMA_STREAM STM32_TIM_TIM1_UP_DMA_STREAM
#define PWMD PWMD1
#define DMA_CH STM32_TIM_TIM1_UP_DMA_CHAN

// choose DShot rate. Can be 150, 300, 600 or 1200
#define DSHOT_RATE 600U
#define DSHOT_BIT_PERIOD 19U

#define DSHOT_PWM_FREQUENCY (DSHOT_RATE * 1000U * DSHOT_BIT_PERIOD)

#define DSHOT_BIT_LENGTH 18 // includes two reset bits

// pulse width for 0 and 1 bits
#define DSHOT_MOTOR_BIT_0 7
#define DSHOT_MOTOR_BIT_1 14

static uint32_t *buffer;
static uint16_t buffer_length = DSHOT_BIT_LENGTH*4*sizeof(uint32_t);
const stm32_dma_stream_t *dma;

/*
  setup pwm for all 4 channels enabled
 */
static const PWMConfig pwm_config = {
        .frequency          = DSHOT_PWM_FREQUENCY,
        .period             = DSHOT_BIT_PERIOD,
        .callback           = NULL,
        .channels = {
            {.mode = PWM_OUTPUT_ACTIVE_HIGH,  .callback = NULL},
            {.mode = PWM_OUTPUT_ACTIVE_HIGH,  .callback = NULL},
            {.mode = PWM_OUTPUT_ACTIVE_HIGH,  .callback = NULL},
            {.mode = PWM_OUTPUT_ACTIVE_HIGH,  .callback = NULL},
        },
        .cr2                = 0,
        .dier               = TIM_DIER_UDE,                                                 // DMA on update event for next period
};


void setup(void) {
    hal.console->printf("Starting DShot test\n");
    dma = STM32_DMA_STREAM(DMA_STREAM);
    buffer = (uint32_t *)hal.util->malloc_type(buffer_length, AP_HAL::Util::MEM_DMA_SAFE);

    dmaStreamAllocate(dma, 10, NULL, NULL);    

    pwmStart(&PWMD, &pwm_config);
}


/*
  create a DSHOT 16 bit packet. Based on prepareDshotPacket from betaflight
 */
static uint16_t create_dshot_packet(const uint16_t value)
{
    uint16_t packet = (value << 1); // no telemetry request

    // compute checksum
    uint16_t csum = 0;
    uint16_t csum_data = packet;
    for (uint8_t i = 0; i < 3; i++) {
        csum ^= csum_data;
        csum_data >>= 4;
    }
    csum &= 0xf;
    // append checksum
    packet = (packet << 4) | csum;

    return packet;
}

/*
  fill in a DMA buffer for dshot
 */
static void fill_DMA_buffer_dshot(uint32_t *buffer, uint8_t stride, uint16_t packet)
{
    for (int i = 0; i < 16; i++) {
        buffer[i * stride] = (packet & 0x8000) ? DSHOT_MOTOR_BIT_1 : DSHOT_MOTOR_BIT_0;
        packet <<= 1;
    }
}

static void test_dshot_send(void)
{
    uint16_t packets[4];
    for (uint8_t i=0; i<4; i++) {
        packets[i] = create_dshot_packet(200 * (i+1));
        fill_DMA_buffer_dshot(buffer + i, 4, packets[i]);
    }
    
    dmaStreamSetPeripheral(dma, &(PWMD.tim->DMAR));
    dmaStreamSetMemory0(dma, buffer);
    dmaStreamSetTransactionSize(dma, buffer_length/sizeof(uint32_t));
    dmaStreamSetFIFO(dma, STM32_DMA_FCR_DMDIS | STM32_DMA_FCR_FTH_FULL);
    dmaStreamSetMode(dma,
                     STM32_DMA_CR_CHSEL(DMA_CH) |
                     STM32_DMA_CR_DIR_M2P | STM32_DMA_CR_PSIZE_WORD | STM32_DMA_CR_MSIZE_WORD |
                     STM32_DMA_CR_MINC | STM32_DMA_CR_PL(3));

    for (uint8_t i=0; i<4; i++) {
        pwmEnableChannel(&PWMD, i, 0);
    }

    // setup for 4 burst strided transfers. 0x0D is the register
    // address offset of the CCR registers in the timer peripheral
    PWMD.tim->DCR = 0x0D | STM32_TIM_DCR_DBL(3);

    dmaStreamEnable(dma);
}

static void test_dshot_cleanup(void)
{
    dmaStreamDisable(dma);
}

void loop(void)
{
    hal.console->printf("tick\n");
    test_dshot_send();
    hal.scheduler->delay(1);
    test_dshot_cleanup();
    hal.scheduler->delay(1000);
    if (hal.console->available() > 10) {
        hal.scheduler->reboot(false);
    }
}

AP_HAL_MAIN();
