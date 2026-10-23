#include <AP_HAL/ap_setjmp.h>

#if AP_SETJMP_CORTEX_ENABLE

int __attribute__((naked, noinline)) ap_setjmp(ap_jmp_buf env) {
    __asm__(
        "mov ip, sp\n\t" // ip must be free on a function call; can't store sp
        "stmia r0!, {r4-r10, fp, ip, lr}\n\t" // store 10 regular registers
        "vstm r0, {d8-d15}\n\t" // store 16 FP registers. use 8 byte store for
                                // speed as our buffer is 8 byte aligned
        "movs r0, #0\n\t" // return 0
        "bx lr\n\t" // do return
    );
}

void __attribute__((noreturn, naked, noinline)) ap_longjmp(ap_jmp_buf env, int val) {
    __asm__(
        "ldmia r0!, {r4-r10, fp, ip, lr}\n\t" // load 10 regular registers
        "mov sp, ip\n\t" // restore stack pointer from scratch register
        "vldm r0, {d8-d15}\n\t" // load 16 FP registers. use 8 byte load for
                                // speed as our buffer is 8 byte aligned
        "movs r0, r1\n\t" // prepare return of value
        "it eq\n\t" // but if value is zero
        "moveq r0, #1\n\t" // return one
        "bx lr\n\t" // do return
    );
}

#endif
