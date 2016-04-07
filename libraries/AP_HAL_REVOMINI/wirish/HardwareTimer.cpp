/******************************************************************************
 * The MIT License
 *
 * Copyright (c) 2010 Bryan Newbold.
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

#include "HardwareTimer.h"
#include "boards.h"             // for CYCLES_PER_MICROSECOND
#include "AP_Math/AP_Math.h"

// TODO [0.1.0] Remove deprecated pieces

#define NR_TIMERS 8

#define MAX_RELOAD ((1 << 16) - 1)

HardwareTimer::HardwareTimer(uint8 timerNum) {
    if (timerNum > NR_TIMERS) {
        assert_param(0);
    }
    timer_dev *devs[] = {
        TIMER1,
        TIMER2,
        TIMER3,
        TIMER4,
        TIMER5,
        TIMER6,
        TIMER7,
        TIMER8,
    };
    this->dev = devs[timerNum - 1];
}

void HardwareTimer::pause(void) {
    timer_pause(this->dev);
}

void HardwareTimer::resume(void) {
    timer_resume(this->dev);
}

uint32 HardwareTimer::getPrescaleFactor(void) {
    return timer_get_prescaler(this->dev) + 1;
}

void HardwareTimer::setPrescaleFactor(uint32 factor) {
    timer_set_prescaler(this->dev, (uint16)(factor - 1));
}

uint16 HardwareTimer::getOverflow() {
    return timer_get_reload(this->dev);
}

void HardwareTimer::setOverflow(uint16 val) {
    timer_set_reload(this->dev, val);
}

uint16 HardwareTimer::getCount(void) {
    return timer_get_count(this->dev);
}

void HardwareTimer::setCount(uint16 val) {
    uint16 ovf = this->getOverflow();
    timer_set_count(this->dev, MIN(val, ovf));
}

uint16 HardwareTimer::setPeriod(uint32 microseconds) {
    // Not the best way to handle this edge case?
    if (!microseconds) {
        this->setPrescaleFactor(1);
        this->setOverflow(1);
        return this->getOverflow();
    }

	uint32 period_cyc;
	/* SystemCoreClock is set to 168 MHz for STM32F4xx devices */
	if (dev->regs == TIM1 || dev->regs == TIM8 || dev->regs == TIM9 || dev->regs == TIM10 || dev->regs == TIM11)
	{
		period_cyc = microseconds * CYCLES_PER_MICROSECOND;
	} 
	else
	{
		period_cyc = microseconds * CYCLES_PER_MICROSECOND / 2;
	}
	
    uint16 prescaler = (uint16)(period_cyc / MAX_RELOAD + 1);
    uint16 overflow = (uint16)round(period_cyc / prescaler);
    this->setPrescaleFactor(prescaler);
    this->setOverflow(overflow);
    return overflow;
}

void HardwareTimer::setMode(int channel, timer_mode mode) {
    timer_set_mode(this->dev, (uint8)channel, (timer_mode)mode);
}

uint16 HardwareTimer::getCompare(int channel) {
    return timer_get_compare(this->dev, (uint8)channel);
}

void HardwareTimer::setCompare(int channel, uint16 val) {
    uint16 ovf = this->getOverflow();
    timer_set_compare(this->dev, (uint8)channel, MIN(val, ovf));
}

void HardwareTimer::attachInterrupt(int channel, voidFuncPtr handler) {
    timer_attach_interrupt(this->dev, (uint8)channel, handler);
}

void HardwareTimer::detachInterrupt(int channel) {
    timer_detach_interrupt(this->dev, (uint8)channel);
}

void HardwareTimer::refresh(void) {
    timer_generate_update(this->dev);
}

/* -- Deprecated predefined instances -------------------------------------- */

//HardwareTimer Timer1(1);
HardwareTimer Timer2(2);
HardwareTimer Timer3(3);
HardwareTimer Timer4(4);
HardwareTimer Timer5(5);
HardwareTimer Timer6(6);
HardwareTimer Timer7(7);
//HardwareTimer Timer8(8);
