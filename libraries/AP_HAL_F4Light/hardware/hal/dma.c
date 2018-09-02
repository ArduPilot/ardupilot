/******************************************************************************
 * The MIT License
 *
 (c) 2017 night_ghost@ykoctpa.ru
 
based on:
 
 * Copyright (c) 2010 Michael Hope.
 *
 * Permission is hereby granted, free of charge, to any person
 * obtaining a copy of this software and associated documentation
 * files (the "Software"), to deal in the Software without
 * restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies
 * of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS
 * BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN
 * ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *****************************************************************************/

 
/**
 *  Direct Memory Access peripheral support
 
 !!! read this first!!! https://blog.frankvh.com/2012/01/13/stm32f2xx-stm32f4xx-dma-maximum-transactions/
 
 */
#pragma GCC optimize ("O2")

#include "dma.h"
#include "bitband.h"
#include <string.h>
#include "util.h"
#include "nvic.h"

/*
 * Devices
 */

static Handler dma1_handlers[8] IN_CCM;

static const dma_dev dma1 = {
    .regs     = (dma_reg_map *)DMA1_BASE,
    .clk_id   = RCC_AHB1Periph_DMA1,
    .irq_lines = { DMA1_Stream0_IRQn, DMA1_Stream1_IRQn, DMA1_Stream2_IRQn, DMA1_Stream3_IRQn, DMA1_Stream4_IRQn, DMA1_Stream5_IRQn, DMA1_Stream6_IRQn, DMA1_Stream7_IRQn},
    .handlers  = dma1_handlers
};
/** DMA1 device */
//const dma_dev * const _DMA1 = &dma1;


static Handler dma2_handlers[8] IN_CCM;

static const dma_dev dma2 = {
    .regs     = (dma_reg_map *)DMA2_BASE,
    .clk_id   = RCC_AHB1Periph_DMA2,
    .irq_lines = { DMA2_Stream0_IRQn, DMA2_Stream1_IRQn, DMA2_Stream2_IRQn, DMA2_Stream3_IRQn, DMA2_Stream4_IRQn, DMA2_Stream5_IRQn, DMA2_Stream6_IRQn, DMA2_Stream7_IRQn},
    .handlers  = dma2_handlers

};
/** DMA2 device */
//const dma_dev * const _DMA2 = &dma2;

static const dma_dev * const DMAS[] = { &dma1, &dma2 };

/*
 * Convenience routines
 */

/**
 * @brief Initialize a DMA device.
 * @param dev Device to initialize.
 */
//void dma_init(const dma_dev *dev) {
void dma_init(dma_stream stream) {
    const dma_dev * dev=DMAS[(stream>>4) & 3]; 
    stream &= 0xF;
    
    memset(dev->handlers, 0, 8 * sizeof(Handler));
    
    RCC_AHB1PeriphClockCmd(dev->clk_id, ENABLE);

    dev->regs->STREAM[stream].CR &= ~DMA_CR_EN;
}

/**
 * @brief Attach an interrupt to a DMA transfer.
 *
 * Interrupts are enabled using appropriate mode flags in
 * dma_setup_transfer().
 *
 * @param dev DMA device
 * @param stream Stream to attach handler to
 * @param handler Interrupt handler to call when channel interrupt fires.
 * @see dma_setup_transfer()
 * @see dma_detach_interrupt()
 */
void dma_attach_interrupt(dma_stream stream, Handler handler, uint8_t flag) {

    const dma_dev * dev=DMAS[(stream>>4) & 3]; 
    stream &= 0xF;

    dev->handlers[stream] = handler;
    IRQn_Type irq = dev->irq_lines[stream];

    enable_nvic_irq(irq, DMA_IOC_INT_PRIORITY);
    
    dev->regs->STREAM[stream].CR |= flag;
}

/**
 * @brief Detach a DMA transfer interrupt handler.
 *
 * After calling this function, the given channel's interrupts will be
 * disabled.
 *
 * @param dev DMA device
 * @param stream Stream whose handler to detach
 * @sideeffect Clears interrupt enable bits in the channel's CCR register.
 * @see dma_attach_interrupt()
 */
void dma_detach_interrupt(dma_stream stream) {
    const dma_dev * dev=DMAS[(stream>>4) & 3]; 
    stream &= 0xF;

    NVIC_DisableIRQ(dev->irq_lines[stream]);
    dev->handlers[stream] = 0;
    
    dev->regs->STREAM[stream].CR &= ~ (DMA_CR_TCIE | DMA_CR_HTIE | DMA_CR_TEIE | DMA_CR_DMEIE ); //0x1e;
}


// ST similar way
void dma_init_transfer(dma_stream stream, DMA_InitType *v){
    uint32_t tmpreg;

    const dma_dev * dev=DMAS[(stream>>4) & 3]; 
    stream &= 0xF;
    dma_stream_t *DMAy_Streamx = &dev->regs->STREAM[stream];
    
    DMAy_Streamx->CR  &= ~DMA_CR_EN; // disable
    DMAy_Streamx->PAR  = (uint32_t)v->DMA_PeripheralBaseAddr;
    DMAy_Streamx->M0AR = (uint32_t)v->DMA_Memory0BaseAddr;
    DMAy_Streamx->NDTR = v->DMA_BufferSize;

    /*------------------------- DMAy Streamx CR Configuration ------------------*/
    /* Get the DMAy_Streamx CR value */
    tmpreg = DMAy_Streamx->CR;

    /* Clear CHSEL, MBURST, PBURST, PL, MSIZE, PSIZE, MINC, PINC, CIRC and DIR bits */
    tmpreg &= ((uint32_t)~(DMA_SxCR_CHSEL | DMA_SxCR_MBURST | DMA_SxCR_PBURST | \
                           DMA_SxCR_PL | DMA_SxCR_MSIZE | DMA_SxCR_PSIZE | \
                           DMA_SxCR_MINC | DMA_SxCR_PINC | DMA_SxCR_CIRC | \
                           DMA_SxCR_DIR));

  /* Configure DMAy Streamx: */
  /* Set CHSEL bits according to DMA_CHSEL value */
  /* Set DIR bits according to DMA_DIR value */
  /* Set PINC bit according to DMA_PeripheralInc value */
  /* Set MINC bit according to DMA_MemoryInc value */
  /* Set PSIZE bits according to DMA_PeripheralDataSize value */
  /* Set MSIZE bits according to DMA_MemoryDataSize value */
  /* Set CIRC bit according to DMA_Mode value */
  /* Set PL bits according to DMA_Priority value */
  /* Set MBURST bits according to DMA_MemoryBurst value */
  /* Set PBURST bits according to DMA_PeripheralBurst value */
    tmpreg |= v->DMA_flags; 
/*
  tmpreg |= v->DMA_Channel | v->DMA_DIR |
            v->DMA_PeripheralInc | v->DMA_MemoryInc |
            v->DMA_PeripheralDataSize | v->DMA_MemoryDataSize |
            v->DMA_Mode | v->DMA_Priority |
            v->DMA_MemoryBurst | v->DMA_PeripheralBurst;
*/         

    /* Write to DMAy Streamx CR register */
    DMAy_Streamx->CR = tmpreg;

    /*------------------------- DMAy Streamx FCR Configuration -----------------*/
    /* Get the DMAy_Streamx FCR value */
    tmpreg = DMAy_Streamx->FCR;

    /* Clear DMDIS and FTH bits */
    tmpreg &= (uint32_t)~(DMA_SxFCR_DMDIS | DMA_SxFCR_FTH);
  
    /* Configure DMAy Streamx FIFO: 
      Set DMDIS bits according to DMA_FIFOMode value 
      Set FTH bits according to DMA_FIFOThreshold value */
    tmpreg |= v->DMA_FIFO_flags;

    /* Write to DMAy Streamx FCR */
    DMAy_Streamx->FCR = tmpreg;
}

    
void dma_set_num_transfers(dma_stream stream, uint16_t num_transfers) {
    const dma_dev * dev=DMAS[(stream>>4) & 3]; 
    stream &= 0xF;
    dev->regs->STREAM[stream].NDTR = num_transfers;
}


void dma_enable(dma_stream stream) {
    const dma_dev * dev=DMAS[(stream>>4) & 3]; 
    stream &= 0xF;
    dev->regs->STREAM[stream].CR |= DMA_CR_EN;
}    

void dma_disable(dma_stream stream) {
    const dma_dev * dev=DMAS[(stream>>4) & 3]; 
    stream &= 0xF;
    dev->regs->STREAM[stream].CR &= ~DMA_CR_EN;
}    

/**
 * @brief Check if a DMA stream is enabled
 * @param dev DMA device
 * @param stream Stream whose enabled bit to check.
 */
uint8_t dma_is_stream_enabled(dma_stream stream) {
    const dma_dev * dev=DMAS[(stream>>4) & 3]; 
    stream &= 0xF;
    return (uint8_t)(dev->regs->STREAM[stream].CR & DMA_CR_EN);
}


void dma_clear_isr_bits(dma_stream stream) {
    const dma_dev * dev=DMAS[(stream>>4) & 3]; 
    stream &= 0xF;

    switch (stream) {
    case 0:
        dev->regs->LIFCR|= 0x0000003d;
        break;
    case 1:
        dev->regs->LIFCR|=0x00000f40;
        break;
    case 2:
        dev->regs->LIFCR|=0x003d0000;
        break;
    case 3:
        dev->regs->LIFCR|=0x0f400000;
        break;
    case 4:
        dev->regs->HIFCR|=0x0000003d;
        break;
    case 5:
        dev->regs->HIFCR|=0x00000f40;
        break;
    case 6:
        dev->regs->HIFCR|=0x003d0000;
        break;
    case 7:
        dev->regs->HIFCR|=0x0f400000;
        break;
    
    default:
        break;
    }
}

uint8_t dma_get_isr_bits(dma_stream stream) {
    const dma_dev * dev=DMAS[(stream>>4) & 3]; 
    stream &= 0xF;

    uint32_t reg;
    
    switch (stream) {
    case 0:
    case 1:
    case 2:
    case 3:
        reg = dev->regs->LISR;
        break;
        
    default:
        reg = dev->regs->HISR;
        break;    
    }


    switch (stream) {
    case 0:
        return (reg >> 0) & 0x0000003d;
    case 1:
        return (reg >> 6) & 0x0000003d;
    case 2:
        return (reg >> 16) & 0x0000003d;
    case 3:
        return (reg >> 22) & 0x0000003d;

    case 4:
        return (reg >> 0) & 0x0000003d;
    case 5:
        return (reg >> 6) & 0x0000003d;
    case 6:
        return (reg >> 16) & 0x0000003d;
    case 7:
        return (reg >> 22) & 0x0000003d;

    default:
        return 0;
    }
}


/*
 * IRQ handlers
 */

static void dispatch_handler(dma_stream stream) {
#ifdef ISR_PERF
    t = stopwatch_getticks();
#endif
    const dma_dev * dev=DMAS[(stream>>4) & 3]; 

    Handler handler = dev->handlers[stream & 0xF];
    if (handler) {
        revo_call_handler(handler, (uint32_t)stream);
    }
    dma_clear_isr_bits(stream); /* in case handler doesn't */
#ifdef ISR_PERF
    t = stopwatch_getticks() - t;
    isr_time += t;
    if(t>max_isr_time) max_isr_time=t;
#endif
}

void DMA1_Stream0_IRQHandler(void) {
    dispatch_handler(DMA1_STREAM0);
}

void DMA1_Stream1_IRQHandler(void) {
    dispatch_handler(DMA1_STREAM1);
}

void DMA1_Stream2_IRQHandler(void) {
    dispatch_handler(DMA1_STREAM2);
}

void DMA1_Stream3_IRQHandler(void) {
    dispatch_handler(DMA1_STREAM3);
}

void DMA1_Stream4_IRQHandler(void) {
    dispatch_handler(DMA1_STREAM4);
}

void DMA1_Stream5_IRQHandler(void) {
    dispatch_handler(DMA1_STREAM5);
}

void DMA1_Stream6_IRQHandler(void) {
    dispatch_handler(DMA1_STREAM6);
}

void DMA1_Stream7_IRQHandler(void) {
    dispatch_handler(DMA1_STREAM7);
}

void DMA2_Stream0_IRQHandler(void) {
    dispatch_handler(DMA2_STREAM0);
}

void DMA2_Stream1_IRQHandler(void) {
    dispatch_handler(DMA2_STREAM1);
}

void DMA2_Stream2_IRQHandler(void) {
    dispatch_handler(DMA2_STREAM2);
}

void DMA2_Stream3_IRQHandler(void) {
    dispatch_handler(DMA2_STREAM3);
}

void DMA2_Stream4_IRQHandler(void) {
    dispatch_handler(DMA2_STREAM4);
}

void DMA2_Stream5_IRQHandler(void) {
    dispatch_handler(DMA2_STREAM5);
}

void DMA2_Stream6_IRQHandler(void) {
    dispatch_handler(DMA2_STREAM6);
}

void DMA2_Stream7_IRQHandler(void) {
    dispatch_handler(DMA2_STREAM7);
}

