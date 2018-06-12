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
    signal = (uint32_t*)hal.util->malloc_type(sizeof(uint32_t)*_bounce_buf_size, AP_HAL::Util::MEM_DMA_SAFE);
    if (signal == nullptr) {
        return false;
    }
    _icu_drv = icu_drv;
    //Setup Burst transfer of period and width measurement
    dma = STM32_DMA_STREAM(dma_stream);
    chSysLock();
    bool dma_allocated = dmaStreamAllocate(dma,
                                            12,  //IRQ Priority
                                            (stm32_dmaisr_t)_irq_handler,
                                            (void *)this);
    osalDbgAssert(!dma_allocated, "stream already allocated");
    chSysUnlock();
    //setup address for full word transfer from Timer
    dmaStreamSetPeripheral(dma, &icu_drv->tim->DMAR);

    uint32_t dmamode = STM32_DMA_CR_DMEIE | STM32_DMA_CR_TEIE;
    dmamode |= STM32_DMA_CR_CHSEL(dma_channel);
    dmamode |= STM32_DMA_CR_PL(0);
    dmaStreamSetMemory0(dma, signal);
    dmaStreamSetTransactionSize(dma, _bounce_buf_size);
    dmaStreamSetMode(dma, dmamode | STM32_DMA_CR_DIR_P2M | STM32_DMA_CR_PSIZE_WORD |
                            STM32_DMA_CR_MSIZE_WORD | STM32_DMA_CR_MINC | STM32_DMA_CR_TCIE);
    
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

void SoftSigReader::_irq_handler(void* self, uint32_t flags)
{
    SoftSigReader* sig_reader = (SoftSigReader*)self;
    sig_reader->sigbuf.push(sig_reader->signal, sig_reader->_bounce_buf_size);
    //restart the DMA transfers
    dmaStreamSetMemory0(sig_reader->dma, sig_reader->signal);
    dmaStreamSetTransactionSize(sig_reader->dma, sig_reader->_bounce_buf_size);
    dmaStreamEnable(sig_reader->dma);
}


bool SoftSigReader::read(uint32_t &widths0, uint32_t &widths1)
{
    if (sigbuf.pop(widths0) && sigbuf.pop(widths1)) {
        //the data is period and width, order depending on which
        //channel is used and width type (0 or 1) depends on mode
        //being set to HIGH or LOW. We need to swap the words when on
        //channel 1
        if (need_swap) {
            uint32_t tmp = widths1;
            widths1 = widths0;
            widths0 = tmp;
        }
        widths1 -= widths0;
    } else {
        return false;
    }
    return true;
}


bool SoftSigReader::set_bounce_buf_size(uint16_t buf_size)
{
    if (buf_size > _bounce_buf_size) {
        if (signal) {
            hal.util->free_type(signal, sizeof(uint32_t)*_bounce_buf_size, AP_HAL::Util::MEM_DMA_SAFE);
        }
        signal = (uint32_t*)hal.util->malloc_type(sizeof(uint32_t)*buf_size, AP_HAL::Util::MEM_DMA_SAFE);
        if (signal == nullptr) {
            return false;
        }
        _bounce_buf_size = buf_size;
    } else {
        _bounce_buf_size = buf_size;
    }
    return true;
}

#endif // HAL_USE_ICU

#endif //CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
