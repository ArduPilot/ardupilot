#pragma once

/*
 * IAR specific functions for IRQ disable/enable
 */

typedef __istate_t atomic_t;

static inline atomic_t atomic_begin(void)
{
	__istate_t a = __get_interrupt_state();
	__disable_interrupt();
	return a;
}

static inline void atomic_end(atomic_t a)
{
	__set_interrupt_state(a);
}

