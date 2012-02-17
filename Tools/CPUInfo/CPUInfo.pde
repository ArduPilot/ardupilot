/// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: t -*-

/*
  test CPU speed
  Andrew Tridgell September 2011
*/

#include <math.h>
#include <FastSerial.h>

FastSerialPort0(Serial);

#define SERIAL0_BAUD 115200

void setup() {
	Serial.begin(SERIAL0_BAUD, 128, 128);
}

static void show_sizes(void)
{
	Serial.println("Type sizes:");
	Serial.printf("char      : %d\n", sizeof(char));
	Serial.printf("short     : %d\n", sizeof(short));
	Serial.printf("int       : %d\n", sizeof(int));
	Serial.printf("long      : %d\n", sizeof(long));
	Serial.printf("long long : %d\n", sizeof(long long));
	Serial.printf("bool      : %d\n", sizeof(bool));
	Serial.printf("void*     : %d\n", sizeof(void *));
}

#define TENTIMES(x) do { x; x; x; x; x; x; x; x; x; x; } while (0)
#define FIFTYTIMES(x) do { TENTIMES(x); TENTIMES(x); TENTIMES(x); TENTIMES(x); TENTIMES(x); } while (0)

#define TIMEIT(name, op, count) do { \
	uint32_t us_end, us_start; \
	us_start = micros(); \
	for (uint8_t i=0; i<count; i++) { \
		FIFTYTIMES(op);				\
	} \
	us_end = micros(); \
	Serial.printf("%-10s %7.2f usec/call\n", name, double(us_end-us_start)/(count*50.0)); \
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

	v_f = 1+(micros() % 5);
	v_out = 1+(micros() % 3);

	v_32 = 1+(micros() % 5);
	v_out_32 = 1+(micros() % 3);

	v_16 = 1+(micros() % 5);
	v_out_16 = 1+(micros() % 3);

	v_8 = 1+(micros() % 5);
	v_out_8 = 1+(micros() % 3);


	Serial.println("Operation timings:");
	Serial.println("Note: timings for some operations are very data dependent");

	TIMEIT("nop", asm volatile("nop"::), 255);
	TIMEIT("cli/sei", cli(); sei(), 255);

	TIMEIT("micros()", micros(), 200);
	TIMEIT("millis()", millis(), 200);

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
	TIMEIT("delay(1)", delay(1), 5);
}

void loop()
{
	show_sizes();
	Serial.println("");
	show_timings();
	Serial.println("");
	delay(3000);
}
