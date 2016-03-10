

;;  NOTE: To allow the use of this file for both ARMv6M and ARMv7M,
;;        we will only use 16-bit Thumb intructions.

        .extern _lc_ub_stack            ; usr/sys mode stack pointer
        .extern _lc_ue_stack            ; symbol required by debugger
        .extern _lc_ub_table            ; ROM to RAM copy table
        .extern main
        .extern _Exit
        .extern exit
        .weak   exit
        .global __get_argcv
        .weak   __get_argcv
        .extern __argcvbuf
        .weak   __argcvbuf
        ;;.extern __init_hardware

		.extern  SystemInit
        
    .if @defined('__PROF_ENABLE__')
        .extern __prof_init
    .endif
    .if @defined('__POSIX__')
        .extern posix_main
        .extern _posix_boot_stack_top
    .endif

        .global _START

        .section .text.cstart

        .thumb
_START: 
        ;; anticipate possible ROM/RAM remapping
        ;; by loading the 'real' program address
        ldr     r1,=_Next
        bx      r1
_Next:
        ;; initialize the stack pointer
        ldr     r1,=_lc_ub_stack        ; TODO: make this part of the vector table
        mov     sp,r1
	
	; Call the clock system intitialization function.
		bl  SystemInit

        ;; copy initialized sections from ROM to RAM
        ;; and clear uninitialized data sections in RAM

        ldr     r3,=_lc_ub_table
        movs    r0,#0
cploop:
        ldr     r4,[r3,#0]      ; load type
        ldr     r5,[r3,#4]      ; dst address
        ldr     r6,[r3,#8]      ; src address
        ldr     r7,[r3,#12]     ; size

        cmp     r4,#1
        beq     copy
        cmp     r4,#2
        beq     clear
        b       done

copy:
        subs    r7,r7,#1
        ldrb    r1,[r6,r7]
        strb    r1,[r5,r7]
        bne     copy

        adds    r3,r3,#16
        b       cploop

clear:
        subs    r7,r7,#1
        strb    r0,[r5,r7]
        bne     clear
        
        adds    r3,r3,#16
        b       cploop
        
done:   

 
      .if @defined('__POSIX__')
        
        ;; posix stack buffer for system upbringing
        ldr     r0,=_posix_boot_stack_top
        ldr     r0, [r0]
        mov     sp,r0
     
     .else
        
        ;; load r10 with end of USR/SYS stack, which is
        ;; needed in case stack overflow checking is on
        ;; NOTE: use 16-bit instructions only, for ARMv6M
        ldr     r0,=_lc_ue_stack
        mov     r10,r0
     
     .endif

    .if @defined('__PROF_ENABLE__')
        bl      __prof_init
    .endif

    .if @defined('__POSIX__')
        ;; call posix_main with no arguments
        bl      posix_main
    .else
        ;; retrieve argc and argv (default argv[0]==NULL & argc==0)
        bl      __get_argcv
        ldr     r1,=__argcvbuf
        ;; call main
        bl       main
    .endif

        ;; call exit using the return value from main()
        ;; Note. Calling exit will also run all functions
        ;; that were supplied through atexit().
        bl      exit
        
__get_argcv:                    ; weak definition
        movs     r0,#0
        bx      lr

        .ltorg
        .endsec

        .calls  '_START', '  '
        .calls  '_START','__init_vector_table'
    .if @defined('__PROF_ENABLE__')
        .calls  '_START','__prof_init'
    .endif
    .if @defined('__POSIX__')
        .calls  '_START','posix_main'
    .else
        .calls  '_START','__get_argcv'
        .calls  '_START','main'
    .endif
        .calls  '_START','exit'
        .calls  '_START','',0

        .end
