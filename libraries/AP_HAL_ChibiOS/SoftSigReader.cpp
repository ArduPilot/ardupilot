/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */

#include "SoftSigReader.h"
#include "hwdef/common/stm32_util.h"

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS

using namespace ChibiOS;
extern const AP_HAL::HAL& hal;

#if HAL_USE_ICU == TRUE

bool SoftSigReader::attach_capture_timer(ICUDriver* icu_drv, icuchannel_t chan, uint8_t dma_stream, uint32_t dma_channel)
{
    if (chan > ICU_CHANNEL_2) {
        return false;
    }
    signal = (uint32_t*)hal.util->malloc_type(sizeof(uint32_t)*SOFTSIG_BOUNCE_BUF_SIZE, AP_HAL::Util::MEM_DMA_SAFE);
    if (signal == nullptr) {
        return false;
    }
    _icu_drv = icu_drv;
    //Setup Burst transfer of period and width measurement
    osalDbgAssert(dma == nullptr, "double DMA allocation");
    chSysLock();
    dma = dmaStreamAllocI(dma_stream,
                          12,  //IRQ Priority
                          (stm32_dmaisr_t)_irq_handler,
                          (void *)this);
    osalDbgAssert(dma, "stream allocation failed");
    chSysUnlock();
#if STM32_DMA_SUPPORTS_DMAMUX
    dmaSetRequestSource(dma, dma_channel);
#endif
    //setup address for full word transfer from Timer
    dmaStreamSetPeripheral(dma, &icu_drv->tim->DMAR);

    dmamode = STM32_DMA_CR_DMEIE | STM32_DMA_CR_TEIE;
    dmamode |= STM32_DMA_CR_CHSEL(dma_channel);
    dmamode |= STM32_DMA_CR_PL(0);
    dmamode |= STM32_DMA_CR_DIR_P2M | STM32_DMA_CR_PSIZE_WORD |
        STM32_DMA_CR_MSIZE_WORD | STM32_DMA_CR_MINC | STM32_DMA_CR_TCIE;
    dmaStreamSetMemory0(dma, signal);
    dmaStreamSetTransactionSize(dma, SOFTSIG_BOUNCE_BUF_SIZE);
    dmaStreamSetMode(dma, dmamode);

    icucfg.frequency = INPUT_CAPTURE_FREQUENCY;
    icucfg.channel = chan;
    icucfg.width_cb = NULL;
    icucfg.period_cb = NULL;
    icucfg.overflow_cb = NULL;

    if (chan == ICU_CHANNEL_1) {
        icucfg.dier = STM32_TIM_DIER_CC1DE;
        icucfg.mode = ICU_INPUT_ACTIVE_HIGH;
        need_swap = true;
    } else {
        icucfg.mode = ICU_INPUT_ACTIVE_LOW;
        icucfg.dier = STM32_TIM_DIER_CC2DE;
    }
#ifdef HAL_RCIN_IS_INVERTED
    icucfg.mode = (icucfg.mode==ICU_INPUT_ACTIVE_LOW)?ICU_INPUT_ACTIVE_HIGH:ICU_INPUT_ACTIVE_LOW;
#endif
    icuStart(_icu_drv, &icucfg);
    //Extended Timer Setup to enable DMA transfer
    //selected offset for TIM_CCR1 and for two words
    _icu_drv->tim->DCR = STM32_TIM_DCR_DBA(0x0D) | STM32_TIM_DCR_DBL(1);
    //Enable DMA
    dmaStreamEnable(dma);

    //sets input filtering to 4 timer clock
    stm32_timer_set_input_filter(_icu_drv->tim, chan, 2);

    //Start Timer
    icuStartCapture(_icu_drv);
    return true;
}

void SoftSigReader::disable(void)
{
    icuStopCapture(_icu_drv);
    dmaStreamDisable(dma);
}

void SoftSigReader::_irq_handler(void* self, uint32_t flags)
{
    SoftSigReader* sig_reader = (SoftSigReader*)self;
    // we need to restart the DMA as quickly as possible to prevent losing pulses, so we
    // make a fixed length copy to a 2nd buffer. On the F100 this reduces the time with DMA
    // disabled from 20us to under 1us
    stm32_cacheBufferInvalidate(sig_reader->signal, SOFTSIG_BOUNCE_BUF_SIZE*4);
    memcpy(sig_reader->signal2, sig_reader->signal, SOFTSIG_BOUNCE_BUF_SIZE*4);
    //restart the DMA transfers
    dmaStreamDisable(sig_reader->dma);
    dmaStreamSetPeripheral(sig_reader->dma, &sig_reader->_icu_drv->tim->DMAR);
    dmaStreamSetMemory0(sig_reader->dma, sig_reader->signal);
    dmaStreamSetTransactionSize(sig_reader->dma, SOFTSIG_BOUNCE_BUF_SIZE);
    dmaStreamSetMode(sig_reader->dma, sig_reader->dmamode);
    dmaStreamEnable(sig_reader->dma);
    sig_reader->sigbuf.push((const pulse_t *)sig_reader->signal2, SOFTSIG_BOUNCE_BUF_SIZE/2);
}


#endif // HAL_USE_ICU

#endif //CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
