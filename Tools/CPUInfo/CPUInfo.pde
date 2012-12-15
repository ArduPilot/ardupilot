/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/*
  test CPU speed
  Andrew Tridgell September 2011
*/

#include <AP_HAL.h>
#include <AP_HAL_AVR.h>
#include <AP_HAL_AVR_SITL.h>
#include <AP_Common.h>
#include <AP_Baro.h>
#include <AP_Compass.h>
#include <AP_Declination.h>
#include <SITL.h>
#include <Filter.h>
#include <AP_Param.h>
#include <AP_Progmem.h>
#include <AP_Math.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_APM2
const AP_HAL::HAL& hal = AP_HAL_AVR_APM2;
#elif CONFIG_HAL_BOARD == HAL_BOARD_APM1
const AP_HAL::HAL& hal = AP_HAL_AVR_APM1;
#elif CONFIG_HAL_BOARD == HAL_BOARD_AVR_SITL
const AP_HAL::HAL& hal = AP_HAL_AVR_SITL;
#else
#error "Unknown CONFIG_HAL_BOARD type"
#endif

void setup() {
}

static void show_sizes(void)
{
	hal.console->println("Type sizes:");
	hal.console->printf("char      : %d\n", sizeof(char));
	hal.console->printf("short     : %d\n", sizeof(short));
	hal.console->printf("int       : %d\n", sizeof(int));
	hal.console->printf("long      : %d\n", sizeof(long));
	hal.console->printf("long long : %d\n", sizeof(long long));
	hal.console->printf("bool      : %d\n", sizeof(bool));
	hal.console->printf("void*     : %d\n", sizeof(void *));
}

#define TENTIMES(x) do { x; x; x; x; x; x; x; x; x; x; } while (0)
#define FIFTYTIMES(x) do { TENTIMES(x); TENTIMES(x); TENTIMES(x); TENTIMES(x); TENTIMES(x); } while (0)

#define TIMEIT(name, op, count) do { \
	uint32_t us_end, us_start; \
	us_start = hal.scheduler->micros(); \
	for (uint8_t i=0; i<count; i++) { \
		FIFTYTIMES(op);				\
	} \
	us_end = hal.scheduler->micros(); \
	hal.console->printf("%-10s %7.2f usec/call\n", name, double(us_end-us_start)/(count*50.0)); \
} while (0)

volatile float v_f = 1.0;
volatile float v_out;
volatile double v_d = 1.0;
volatile double v_out_d;
volatile uint32_t v_32 = 1;
volatile uint32_t v_out_32 = 1;
volatile uint16_t v_16 = 1;
volatile uint16_t v_out_16 = 1;
volatile uint8_t v_8 = 1;
volatile uint8_t v_out_8 = 1;
volatile uint8_t mbuf1[128], mbuf2[128];
volatile uint64_t v_64 = 1;
volatile uint64_t v_out_64 = 1;

static void show_timings(void)
{

	v_f = 1+(hal.scheduler->micros() % 5);
	v_out = 1+(hal.scheduler->micros() % 3);

	v_32 = 1+(hal.scheduler->micros() % 5);
	v_out_32 = 1+(hal.scheduler->micros() % 3);

	v_16 = 1+(hal.scheduler->micros() % 5);
	v_out_16 = 1+(hal.scheduler->micros() % 3);

	v_8 = 1+(hal.scheduler->micros() % 5);
	v_out_8 = 1+(hal.scheduler->micros() % 3);


	hal.console->println("Operation timings:");
	hal.console->println("Note: timings for some operations are very data dependent");

	TIMEIT("nop", asm volatile("nop"::), 255);
	TIMEIT("cli/sei", hal.scheduler->begin_atomic(); hal.scheduler->end_atomic(), 255);

	TIMEIT("micros()", hal.scheduler->micros(), 200);
	TIMEIT("millis()", hal.scheduler->millis(), 200);

	TIMEIT("fadd", v_out += v_f, 100);
	TIMEIT("fsub", v_out -= v_f, 100);
	TIMEIT("fmul", v_out *= v_f, 100);
	TIMEIT("fdiv", v_out /= v_f, 100);

	TIMEIT("dadd", v_out_d += v_d, 100);
	TIMEIT("dsub", v_out_d -= v_d, 100);
	TIMEIT("dmul", v_out_d *= v_d, 100);
	TIMEIT("ddiv", v_out_d /= v_d, 100);

	TIMEIT("sin()", v_out = sin(v_f), 20);
	TIMEIT("cos()", v_out = cos(v_f), 20);
	TIMEIT("tan()", v_out = tan(v_f), 20);
	TIMEIT("acos()", v_out = acos(v_f * 0.2), 20);
	TIMEIT("asin()", v_out = asin(v_f * 0.2), 20);
	TIMEIT("atan2()", v_out = atan2(v_f * 0.2, v_f * 0.3), 20);
	TIMEIT("sqrt()",v_out = sqrt(v_f), 20);

	TIMEIT("iadd8", v_out_8 += v_8, 100);
	TIMEIT("isub8", v_out_8 -= v_8, 100);
	TIMEIT("imul8", v_out_8 *= v_8, 100);
	TIMEIT("idiv8", v_out_8 /= v_8, 100);

	TIMEIT("iadd16", v_out_16 += v_16, 100);
	TIMEIT("isub16", v_out_16 -= v_16, 100);
	TIMEIT("imul16", v_out_16 *= v_16, 100);
	TIMEIT("idiv16", v_out_16 /= v_16, 100);

	TIMEIT("iadd32", v_out_32 += v_32, 100);
	TIMEIT("isub32", v_out_32 -= v_32, 100);
	TIMEIT("imul32", v_out_32 *= v_32, 100);
	TIMEIT("idiv32", v_out_32 /= v_32, 100);

	TIMEIT("iadd64", v_out_64 += v_64, 100);
	TIMEIT("isub64", v_out_64 -= v_64, 100);
	TIMEIT("imul64", v_out_64 *= v_64, 100);
	TIMEIT("idiv64", v_out_64 /= v_64, 100);

	TIMEIT("memcpy128", memcpy((void*)mbuf1, (const void *)mbuf2, sizeof(mbuf1)), 20);
	TIMEIT("memset128", memset((void*)mbuf1, 1, sizeof(mbuf1)), 20);
	TIMEIT("delay(1)", hal.scheduler->delay(1), 5);
}

void loop()
{
	show_sizes();
	hal.console->println("");
	show_timings();
	hal.console->println("");
	hal.scheduler->delay(3000);
}

AP_HAL_MAIN();
