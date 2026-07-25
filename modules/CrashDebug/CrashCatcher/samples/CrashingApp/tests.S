/* Copyright (C) 2018  Adam Green (https://github.com/adamgreen)

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/
/* Implementation of code to generate various crash scenarios for testing. */
    .text
    .syntax unified

    .global testMspMultipleOf8
    .type testMspMultipleOf8, %function
    .thumb_func
    /* extern "C" void testMspMultipleOf8(void);
       Uses MSP and has it pointing to an even multiple of 8.
    */
testMspMultipleOf8:
    // Make SP, an even multiple of 8.
    mov     r0, sp
    movs    r1, #4
    bics    r0, r1
    mov     sp, r0
    // Load 0xFFFFFFFF into LR
    movs    r1, #1
    rsbs    r0, r1, #0
    mov     lr, r0
    // Load known values into R0-R12
    movs    r0, #0
    // r1 was already set correctly above when initializing LR.
    movs    r2, #2
    movs    r3, #3
    movs    r4, #4
    movs    r5, #5
    movs    r6, #6
    movs    r7, #7
    mov     r8, r7
    mov     r9, r7
    mov     r10, r7
    mov     r11, r7
    mov     r12, r7
    add     r8, r1
    add     r9, r2
    add     r10, r3
    add     r11, r4
    add     r12, r5
    // Generate a hard fault by executing an undefined instruction.
    .word 0xDE00
    // CrashCatcher_Entry() shouldn't return but if it does, just infinite loop here.
    b       .
    // Let assembler know that we have hit the end of the function.
    .pool
    .size   testMspMultipleOf8, .-testMspMultipleOf8


    .global testMspNotMultipleOf8
    .type testMspNotMultipleOf8, %function
    .thumb_func
    /* extern "C" void testMspNotMultipleOf8(void);
       Uses MSP and has it not pointing to an even multiple of 8.
    */
testMspNotMultipleOf8:
    // Make SP, not an even multiple of 8.
    mov     r0, sp
    subs    r0, #8
    movs    r1, #4
    orrs    r0, r1
    mov     sp, r0
    // Load 0xFFFFFFFF into LR
    movs    r1, #1
    rsbs    r0, r1, #0
    mov     lr, r0
    // Load known values into R0-R12
    movs    r0, #0
    // r1 was already set correctly above when initializing LR.
    movs    r2, #2
    movs    r3, #3
    movs    r4, #4
    movs    r5, #5
    movs    r6, #6
    movs    r7, #7
    mov     r8, r7
    mov     r9, r7
    mov     r10, r7
    mov     r11, r7
    mov     r12, r7
    add     r8, r1
    add     r9, r2
    add     r10, r3
    add     r11, r4
    add     r12, r5
    // Generate a hard fault by executing an undefined instruction.
    .word 0xDE00
    // CrashCatcher_Entry() shouldn't return but if it does, just infinite loop here.
    b       .
    // Let assembler know that we have hit the end of the function.
    .pool
    .size   testMspNotMultipleOf8, .-testMspNotMultipleOf8


    .global testPspMultipleOf8
    .type testPspMultipleOf8, %function
    .thumb_func
    /* extern "C" void testPspMultipleOf8(void);
       Uses MSP and has it pointing to an even multiple of 8.
    */
testPspMultipleOf8:
    // Make PSP, an even multiple of 8.
    mov     r0, sp
    movs    r1, #4
    bics    r0, r1
    msr     psp, r0
    // Switch to use of PSP.
    movs    r0, #3
    msr     control,r0
    // Flush instructions so that control mods take effect.
    isb
    // Load 0xFFFFFFFF into LR
    movs    r1, #1
    rsbs    r0, r1, #0
    mov     lr, r0
    // Load known values into R0-R12
    movs    r0, #0
    // r1 was already set correctly above when initializing LR.
    movs    r2, #2
    movs    r3, #3
    movs    r4, #4
    movs    r5, #5
    movs    r6, #6
    movs    r7, #7
    mov     r8, r7
    mov     r9, r7
    mov     r10, r7
    mov     r11, r7
    mov     r12, r7
    add     r8, r1
    add     r9, r2
    add     r10, r3
    add     r11, r4
    add     r12, r5
    // Generate a hard fault by executing an undefined instruction.
    .word 0xDE00
    // CrashCatcher_Entry() shouldn't return but if it does, just infinite loop here.
    b       .
    // Let assembler know that we have hit the end of the function.
    .pool
    .size   testPspMultipleOf8, .-testPspMultipleOf8


    .global testBreakpoints
    .type testBreakpoints, %function
    .thumb_func
    /* extern "C" void testBreakpoints(void);
       Set registers to known values, breakpoint, allow continuation, and breakpoint again to see if anything changed.
    */
testBreakpoints:
    // Save away non-volatile registers that will be modified for testing.
    mov     r0, r8
    mov     r1, r9
    mov     r2, r10
    mov     r3, r11
    push    {r0-r7}
    
    // Load incrementing values into R0-R12
    movs    r0, #0
    movs    r1, #1
    movs    r2, #2
    movs    r3, #3
    movs    r4, #4
    movs    r5, #5
    movs    r6, #6
    movs    r7, #7
    mov     r8, r7
    mov     r9, r7
    mov     r10, r7
    mov     r11, r7
    mov     r12, r7
    add     r8, r1
    add     r9, r2
    add     r10, r3
    add     r11, r4
    add     r12, r5

    // Issue one hard coded breakpoint.
    bkpt    #0

    // Issue another hard coded breakpoint and see if dumps contain same values for all registers other than PC.
    bkpt    #255

    // Restore non-volatile registers and return to caller.
    pop     {r0-r7}
    mov     r8, r0
    mov     r9, r1
    mov     r10, r2
    mov     r11, r3
    bx      lr

    // Let assembler know that we have hit the end of the function.
    .pool
    .size   testBreakpoints, .-testBreakpoints


#if defined(TARGET_M4)

    .global testInitFPURegisters
    .type testInitFPURegisters, %function
    .thumb_func
    /* extern "C" void testInitFPURegisters(void);
       Initialize FPU registers to known values.
    */
testInitFPURegisters:
    vmov.f32    s0, #-1.0
    vmov.f32    s1, #1.0
    vmov.f32    s2, #2.0
    vmov.f32    s3, #3.0
    vmov.f32    s4, #4.0
    vmov.f32    s5, #5.0
    vmov.f32    s6, #6.0
    vmov.f32    s7, #7.0
    vmov.f32    s8, #8.0
    vmov.f32    s9, #9.0
    vmov.f32    s10, #10.0
    vmov.f32    s11, #11.0
    vmov.f32    s12, #12.0
    vmov.f32    s13, #13.0
    vmov.f32    s14, #14.0
    vmov.f32    s15, #15.0
    vmov.f32    s16, #16.0
    vmov.f32    s17, #17.0
    vmov.f32    s18, #18.0
    vmov.f32    s19, #19.0
    vmov.f32    s20, #20.0
    vmov.f32    s21, #21.0
    vmov.f32    s22, #22.0
    vmov.f32    s23, #23.0
    vmov.f32    s24, #24.0
    vmov.f32    s25, #25.0
    vmov.f32    s26, #26.0
    vmov.f32    s27, #27.0
    vmov.f32    s28, #28.0
    vmov.f32    s29, #29.0
    vmov.f32    s30, #30.0
    vmov.f32    s31, #31.0
    ldr         r0, =0xBAADFEED
    vmsr        fpscr, r0
    bx          lr
    .pool
    .size   testInitFPURegisters, .-testInitFPURegisters

#else

    .global testInitFPURegisters
    .type testInitFPURegisters, %function
    .thumb_func
    /* extern "C" void testInitFPURegisters(void);
       Initialize FPU registers to known values.
    */
testInitFPURegisters:
    bx      lr
    .pool
    .size   testInitFPURegisters, .-testInitFPURegisters

#endif


    .end