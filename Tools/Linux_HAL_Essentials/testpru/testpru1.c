/*
 * testpru
 *
 */

#define PRU1

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>
#include <stdio.h>
#include <strings.h>

#include "linux_types.h"
#include "pru_defs.h"
#include "prucomm.h"


extern void delay_cycles(u32 delay);
extern void delay_cycles_accurate(u32 delay);
extern void delay_cycles_accurate2(u32 delay);
extern u32 read_other_r30(void);

extern void update_gpo(u32 clrmsk, u32 setmsk);

void pwm_loop(u32 hi, u32 lo)
{
	while (!pru_signal()) {
		__R30 |=  (1 << 13);
		delay_cycles(hi);
		__R30 &= ~(1 << 13);
		delay_cycles(lo);
	}
}

void pwm_loop2(u32 hi, u32 lo)
{
	while (!pru_signal()) {
		__R30 |=  (1 << 13);
		delay_cycles_accurate2(hi);
		__R30 &= ~(1 << 13);
		delay_cycles_accurate2(lo);
	}
}

#define T1 asm (" .global T1\nT1:");
#define T2 asm (" .global T2\nT2:");

struct pwm_cmd_l cfg;

static void pwm_setup(void)
{
	u8 i;

	cfg.enmask = 0;
	for (i = 0; i < MAX_PWMS; i++)
		cfg.hilo[i][0] = cfg.hilo[i][1] = PRU_us(200);

#if 0
	cfg.enmask = BIT(13) | BIT(12);
	cfg.hilo[12][0] = PRU_us(333);
	cfg.hilo[12][1] = PRU_us(333);
	cfg.hilo[13][0] = PRU_us(100);
	cfg.hilo[13][1] = PRU_us(100);
#endif
}

#undef USE_PWM_LOOP
#define USE_PWM_MACRO

struct cxt {
	u32 cnt;
	u32 next;
	u32 enmask;
	u32 stmask;
	u32 setmsk;
	u32 clrmsk;
	u32 deltamin;
	u32 *next_hi_lo;
};

static inline u32 read_PIEP_COUNT(void)
{
	return PIEP_COUNT;

}

static void handle_pwm_cmd(struct cxt *cxt)
{
	u8 i;
	u32 msk, setmsk, clrmsk;
	u32 enmask, stmask, cnt, deltamin, next;
	u32 *nextp;
	u32 *next_hi_lop;
	const u32 *hilop;
	cnt = cxt->cnt;
	next = cxt->next;
	enmask = cxt->enmask;
	stmask = cxt->stmask;
	setmsk = cxt->setmsk;
	clrmsk = cxt->clrmsk;
	deltamin = cxt->deltamin;
	next_hi_lop = cxt->next_hi_lo;

	// sc_printf("cnt=%x next=%x deltamin=%x", cnt, next, deltamin);

	for (i = 0; i < MAX_PWMS; i++){
	    cfg.hilo[i][0] = PWM_CMD->periodhi[i][1];
            cfg.hilo[i][1] = PWM_CMD->periodhi[i][0] - PWM_CMD->periodhi[i][1];
            PWM_CMD->hilo_read[i][0] = cfg.hilo[i][0];
            PWM_CMD->hilo_read[i][1] = cfg.hilo[i][1];
        }

        cfg.enmask = PWM_CMD->enmask;
        PWM_CMD->enmask_read = cfg.enmask;
	enmask = cfg.enmask;
	stmask = 0;		/* starting all low */

	clrmsk = 0;
	for (i = 0, msk = 1, nextp = next_hi_lop, hilop = &cfg.hilo[0][0];
			i < MAX_PWMS;
			i++, msk <<= 1, nextp += 3, hilop += 2) {
		if ((enmask & msk) == 0) {
			nextp[1] = PRU_us(100);	/* default */
			nextp[2] = PRU_us(100);
			continue;
		}
		nextp[0] = cnt;		/* next */
		nextp[1] = hilop[0];	/* hi */
		nextp[2] = hilop[1];	/* lo */
	}

	clrmsk = enmask;
	setmsk = 0;
	/* guaranteed to be immediate */
	deltamin = 0;

	cxt->cnt = cnt;
	cxt->next = next;
	cxt->enmask = enmask;
	cxt->stmask = stmask;
	cxt->setmsk = setmsk;
	cxt->clrmsk = clrmsk;
        cxt->deltamin = deltamin;
}


int main(int argc, char *argv[])
{
       	u8 i;
	u32 cnt, next, magic;
	u32 msk, setmsk, clrmsk;
	u32 delta, deltamin, tnext, hi, lo;
	u32 *nextp;
	const u32 *hilop;
	u32 enmask;	/* enable mask */
	u32 stmask;	/* state mask */
	static u32 next_hi_lo[MAX_PWMS][3];
	static struct cxt cxt;
	/* enable OCP master port */
	PRUCFG_SYSCFG &= ~SYSCFG_STANDBY_INIT;
	PRUCFG_SYSCFG = (PRUCFG_SYSCFG &
			~(SYSCFG_IDLE_MODE_M | SYSCFG_STANDBY_MODE_M)) |
			SYSCFG_IDLE_MODE_NO | SYSCFG_STANDBY_MODE_NO;

	/* our PRU wins arbitration */
#if defined(PRU0)
	PRUCFG_SPP &= ~SPP_PRU1_PAD_HP_EN;
#elif defined(PRU1)
	PRUCFG_SPP |=  SPP_PRU1_PAD_HP_EN;
#endif
	pwm_setup();

	/* configure timer */
	PIEP_GLOBAL_CFG = GLOBAL_CFG_DEFAULT_INC(1) |
			  GLOBAL_CFG_CMP_INC(1);
	PIEP_CMP_STATUS = CMD_STATUS_CMP_HIT(1); /* clear the interrupt */
        PIEP_CMP_CMP1   = 0x0;
	PIEP_CMP_CFG |= CMP_CFG_CMP_EN(1);
        PIEP_GLOBAL_CFG |= GLOBAL_CFG_CNT_ENABLE;

	/* copy from cfg to cxt */


	/* initialize */
	cnt = read_PIEP_COUNT();

	enmask = cfg.enmask;
	stmask = 0;		/* starting all low */

	clrmsk = 0;
	for (i = 0, msk = 1, nextp = &next_hi_lo[0][0], hilop = &cfg.hilo[0][0];
			i < MAX_PWMS;
			i++, msk <<= 1, nextp += 3, hilop += 2) {
		if ((enmask & msk) == 0) {
			nextp[1] = PRU_us(100);	/* default */
			nextp[2] = PRU_us(100);
			continue;
		}
		nextp[0] = cnt;		/* next */
		nextp[1] = hilop[0];	/* hi */
		nextp[2] = hilop[1];	/* lo */
	}

	clrmsk = enmask;
	setmsk = 0;
	/* guaranteed to be immediate */
	deltamin = 0;
	next = cnt + deltamin;
        PWM_CMD->magic = 0;

	while(1) {

		/* signalled interrupt from either PRU0 or host */
		if(PWM_CMD->magic == PWM_CMD_MAGIC) {
      			cxt.cnt = cnt;
			cxt.next = next;
			cxt.enmask = enmask;
			cxt.stmask = stmask;
			cxt.setmsk = setmsk;
			cxt.clrmsk = clrmsk;
			cxt.deltamin = deltamin;
			cxt.next_hi_lo = &next_hi_lo[0][0];

 			handle_pwm_cmd(&cxt);

			cnt = cxt.cnt;
			next = cxt.next;
			enmask = cxt.enmask;
			stmask = cxt.stmask;
			setmsk = cxt.setmsk;
			clrmsk = cxt.clrmsk;
			deltamin = cxt.deltamin;
                   	PWM_CMD->magic = PWM_REPLY_MAGIC;
		}
                PWM_CMD->enmask_read = enmask;
		/* if nothing is enabled just skip it all */
		if (enmask == 0)
			continue;

		setmsk = 0;
		clrmsk = (u32)-1;
		deltamin = PRU_ms(100); /* (1U << 31) - 1; */
		next = cnt + deltamin;

#define SINGLE_PWM(_i) \
	do { \
		if (enmask & (1U << (_i))) { \
			nextp = &next_hi_lo[(_i)][0]; \
			tnext = nextp[0]; \
			hi = nextp[1]; \
			lo = nextp[2]; \
			/* avoid signed arithmetic */ \
			while (((delta = (tnext - cnt)) & (1U << 31)) != 0) { \
				/* toggle the state */ \
				if (stmask & (1U << (_i))) { \
					stmask &= ~(1U << (_i)); \
					clrmsk &= ~(1U << (_i)); \
					tnext += lo; \
				} else { \
					stmask |= (1U << (_i)); \
					setmsk |= (1U << (_i)); \
					tnext += hi; \
				} \
			} \
			if (delta <= deltamin) { \
				deltamin = delta; \
				next = tnext; \
			} \
			nextp[0] = tnext; \
		} \
	} while (0)

#ifdef USE_PWM_LOOP
		for (i = 0, msk = 1, nextp = &next_hi_lo[0][0]; i < MAX_PWMS; i++, msk <<= 1, nextp += 3) {

			if ((enmask & msk) == 0)
				continue;

			tnext = nextp[0];
			hi = nextp[1];
			lo = nextp[2];

			/* avoid signed arithmetic */
			while (((delta = (tnext - cnt)) & (1U << 31)) != 0) {
				/* toggle the state */
				if (stmask & msk) {
					stmask &= ~msk;
					clrmsk &= ~msk;
					tnext += lo;
				} else {
					stmask |= msk;
					setmsk |= msk;
					tnext += hi;
				}
			}
			if (delta <= deltamin) {
				deltamin = delta;
				next = tnext;
			}
			nextp[0] = tnext;
		}
#endif

#ifdef USE_PWM_MACRO

#if MAX_PWMS > 0 && (PWM_EN_MASK & BIT(0))
		SINGLE_PWM(0);
#endif
#if MAX_PWMS > 1 && (PWM_EN_MASK & BIT(1))
		SINGLE_PWM(1);
#endif
#if MAX_PWMS > 2 && (PWM_EN_MASK & BIT(2))
		SINGLE_PWM(2);
#endif
#if MAX_PWMS > 3 && (PWM_EN_MASK & BIT(3))
		SINGLE_PWM(3);
#endif
#if MAX_PWMS > 4 && (PWM_EN_MASK & BIT(4))
		SINGLE_PWM(4);
#endif
#if MAX_PWMS > 5 && (PWM_EN_MASK & BIT(5))
		SINGLE_PWM(5);
#endif
#if MAX_PWMS > 6 && (PWM_EN_MASK & BIT(6))
		SINGLE_PWM(6);
#endif
#if MAX_PWMS > 7 && (PWM_EN_MASK & BIT(7))
		SINGLE_PWM(7);
#endif
#if MAX_PWMS > 8 && (PWM_EN_MASK & BIT(8))
		SINGLE_PWM(8);
#endif
#if MAX_PWMS > 9 && (PWM_EN_MASK & BIT(9))
		SINGLE_PWM(9);
#endif
#if MAX_PWMS > 10 && (PWM_EN_MASK & BIT(10))
		SINGLE_PWM(10);
#endif
#if MAX_PWMS > 11 && (PWM_EN_MASK & BIT(11))
		SINGLE_PWM(11);
#endif
#if MAX_PWMS > 12 && (PWM_EN_MASK & BIT(12))
		SINGLE_PWM(12);
#endif
#if MAX_PWMS > 13 && (PWM_EN_MASK & BIT(13))
		SINGLE_PWM(13);
#endif
#if MAX_PWMS > 14 && (PWM_EN_MASK & BIT(14))
		SINGLE_PWM(14);
#endif
#if MAX_PWMS > 15 && (PWM_EN_MASK & BIT(15))
		SINGLE_PWM(15);
#endif
#if MAX_PWMS > 16 && (PWM_EN_MASK & BIT(16))
		SINGLE_PWM(16);
#endif
#if MAX_PWMS > 17 && (PWM_EN_MASK & BIT(17))
		SINGLE_PWM(17);
#endif
#if MAX_PWMS > 18 && (PWM_EN_MASK & BIT(18))
		SINGLE_PWM(18);
#endif
#if MAX_PWMS > 19 && (PWM_EN_MASK & BIT(19))
		SINGLE_PWM(19);
#endif
#if MAX_PWMS > 20 && (PWM_EN_MASK & BIT(20))
		SINGLE_PWM(20);
#endif
#if MAX_PWMS > 21 && (PWM_EN_MASK & BIT(21))
		SINGLE_PWM(21);
#endif
#if MAX_PWMS > 22 && (PWM_EN_MASK & BIT(22))
		SINGLE_PWM(22);
#endif
#if MAX_PWMS > 23 && (PWM_EN_MASK & BIT(23))
		SINGLE_PWM(23);
#endif
#if MAX_PWMS > 24 && (PWM_EN_MASK & BIT(24))
		SINGLE_PWM(24);
#endif
#if MAX_PWMS > 25 && (PWM_EN_MASK & BIT(25))
		SINGLE_PWM(25);
#endif
#if MAX_PWMS > 26 && (PWM_EN_MASK & BIT(26))
		SINGLE_PWM(26);
#endif
#if MAX_PWMS > 27 && (PWM_EN_MASK & BIT(27))
		SINGLE_PWM(27);
#endif
#if MAX_PWMS > 28 && (PWM_EN_MASK & BIT(28))
		SINGLE_PWM(28);
#endif
#if MAX_PWMS > 29 && (PWM_EN_MASK & BIT(29))
		SINGLE_PWM(29);
#endif
#if MAX_PWMS > 30 && (PWM_EN_MASK & BIT(30))
		SINGLE_PWM(30);
#endif
#if MAX_PWMS > 31 && (PWM_EN_MASK & BIT(31))
		SINGLE_PWM(30);
#endif
#endif
		/* results in set bits where there are changes */
		delta = ~clrmsk | setmsk;

		if ((delta & 0xffff) != 0)
			__R30 = (__R30 & (clrmsk & 0xffff)) | (setmsk & 0xffff);
		if ((delta >> 16) != 0)
			pru_other_and_or_reg(30, (clrmsk >> 16) | 0xffff0000, setmsk >> 16);

#if 0
		cnt = read_PIEP_COUNT();
		if (((next - cnt) & (1U << 31)) == 0 && delta > PRU_ms(1)) {
			sc_printf("bad next=%x cnt=%x", next, cnt);
		}
#endif
		/* loop while nothing changes */
		do {
			cnt = read_PIEP_COUNT();
			if(PWM_CMD->magic == PWM_CMD_MAGIC){
				break;
			}		
		} while (((next - cnt) & (1U << 31)) == 0);
	}
}
