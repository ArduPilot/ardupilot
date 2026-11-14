/* SPDX-License-Identifier: GPL-3.0-or-later */
#ifdef __arm__
.syntax unified
.thumb
.text
.global ap_setjmp
.type ap_setjmp, %function
.thumb_func
ap_setjmp:
    mov ip, sp
    stmia r0!, {r4-r10, fp, ip, lr}
    vstm r0, {d8-d15}
    movs r0, #0
    bx lr
.size ap_setjmp, .-ap_setjmp
.global ap_longjmp
.type ap_longjmp, %function
.thumb_func
ap_longjmp:
    ldmia r0!, {r4-r10, fp, ip, lr}
    mov sp, ip
    vldm r0, {d8-d15}
    movs r0, r1
    it eq
    moveq r0, #1
    bx lr
.size ap_longjmp, .-ap_longjmp
#else
.text
.global ap_setjmp
.type ap_setjmp, @function
ap_setjmp:
    movq %rbx, 0(%rdi)
    movq %rbp, 8(%rdi)
    movq %r12, 16(%rdi)
    movq %r13, 24(%rdi)
    movq %r14, 32(%rdi)
    movq %r15, 40(%rdi)
    leaq 8(%rsp), %rax
    movq %rax, 48(%rdi)
    movq (%rsp), %rax
    movq %rax, 56(%rdi)
    xorl %eax, %eax
    ret
.size ap_setjmp, .-ap_setjmp
.global ap_longjmp
.type ap_longjmp, @function
ap_longjmp:
    movl %esi, %eax
    testl %eax, %eax
    jnz 1f
    incl %eax
1:
    movq 0(%rdi), %rbx
    movq 8(%rdi), %rbp
    movq 16(%rdi), %r12
    movq 24(%rdi), %r13
    movq 32(%rdi), %r14
    movq 40(%rdi), %r15
    movq 48(%rdi), %rsp
    jmp *56(%rdi)
.size ap_longjmp, .-ap_longjmp
.section .note.GNU-stack,"",@progbits
#endif
