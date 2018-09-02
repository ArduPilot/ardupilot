#include <p16f690.inc>

    list    p=16f690

    __config _INTRC_OSC_NOCLKOUT & _WDT_OFF & _PWRTE_ON & _MCLRE_OFF & _CP_OFF & _CPD_OFF & _BOR_ON & _IESO_OFF & _FCMEN_OFF

;
; This program will read until eight pwm inputs and will send its values
; through serial port at 115200bps.
;
;
;        +5V -| VDD    VSS |- GND
;    input 6 -| RA5    RA0 |- input 1
;    input 5 -| RA4    RA1 |- input 2
;    input 4 -| RA3    RA2 |- input 3
;   output 4 -| RC5    RC0 |- n. c.
;   output 3 -| RC4    RC1 |- status led
;   output 2 -| RC3    RC2 |- output 1
;       n.c. -| RC6    RB4 |- input 8
;       n.c. -| RC7    RB5 |- rx
;        tx* -| RB7    RB6 |- input 7
;
;
; * In order to keep IO voltage at 3.3V use a voltage divider on TX.
;
; PS: outputs are not covered by this program yet.
;

; The INPUT_NUM is attached to input_mask and the
; interruption. Do not change this value without
; changing the interruption and mask.
#define INPUT_NUM 8
#define MAGIC 0x55AA

    cblock 0x020
    ; input_value, input_rise and input_mirror
    ; are at the bank's beginning to make easy
    ; the exchange of data between them.
    input_value:(INPUT_NUM * 2)
    input_previous
    input_current
    input_changed
    input_mask
    input_updated
    sent_update
    endc

    cblock 0x72
    save_w
    save_status
    save_fsr
    timer_l
    timer_h
    count
    tmr1if_count
    endc

    cblock 0x0a0
    ; see input_value
    input_rise:(INPUT_NUM * 2)
    endc

    cblock 0x120
    ; see input_value
    input_mirror:(INPUT_NUM * 2)
    endc

; Reset Vector
    org     0x0000
    nop
    nop
    nop
    goto    Main

; Interrupts Vector
    org     0x0004
    ; save the current context
    movwf   save_w
    swapf   STATUS, w
    clrf    STATUS
    movwf   save_status
    movf    FSR, w
    movwf   save_fsr

    ; it's a input event
    btfss   INTCON, RABIF
    goto    IntInputDone

    ; capture timer 1
    movf    TMR1H, w
    movwf   timer_h
    movf    TMR1L, w
    movwf   timer_l
    movf    TMR1H, w
    subwf   timer_h, w
    btfsc   STATUS, Z
    goto    IntTMR1Done
    movf    TMR1H, w
    movwf   timer_h
    movf    TMR1L, w
    movwf   timer_l
IntTMR1Done:

    ; capture the current input state
    movf    PORTA, w
    btfsc   PORTB, RB6
    iorlw   (1<<6)
    btfsc   PORTB, RB4
    iorlw   (1<<7)

    bcf     INTCON, RABIF

    ; identify the changed inputs
    movwf   input_current
    xorwf   input_previous, w
    movwf   input_changed
    movf    input_current, w
    movwf   input_previous

    ; prepare for loop
    movlw   .1
    movwf   input_mask
    movlw   input_rise
    movwf   FSR
    goto    IntInputLoop

IntInputNext:
    incf    FSR, f

IntInputNextHalf:
    ; ... and ensure bank 0 selection
    incf    FSR, f
    bsf     FSR, 7

    ; done if no more channels
    bcf     STATUS, C
    rlf     input_mask, f
    btfsc   STATUS, C
    goto    IntInputDone

IntInputLoop:
    ; skip if channel not changed
    movf    input_mask, w
    andwf   input_changed, w
    btfsc   STATUS, Z
    goto    IntInputNext

    ; check if it is raising or falling
    andwf   input_current, w
    btfss   STATUS, Z
    goto    IntInputRise

    ; calculate and move the lsb to input_value
    movf    INDF, w
    subwf   timer_l, w
    bcf     FSR, 7
    movwf   INDF

    ; calculate and move the msb to input_value
    bsf     FSR, 7
    incf    FSR, f
    movf    INDF, w
    btfss   STATUS, C
    incf    INDF, w
    subwf   timer_h, w
    bcf     FSR, 7
    movwf   INDF

    goto    IntInputNextHalf

IntInputRise:
    ; tell main loop we have all channels
    btfsc   input_mask, 1
    incf    input_updated, f

    ; save the rise instant
    movf    timer_l, w
    movwf   INDF
    incf    FSR, f
    movf    timer_h, w
    movwf   INDF
    goto    IntInputNextHalf

IntInputDone:

    ; restore the previous context
    movf    save_fsr, w
    movwf   FSR
    swapf   save_status, w
    movwf   STATUS
    swapf   save_w, f
    swapf   save_w, w
    retfie

Main:
    ; bank 0
    clrf    STATUS

    clrf    PORTA
    clrf    PORTB
    clrf    PORTC

    ; bank 1
    bsf     STATUS, RP0

    movlw   (b'111'<<IRCF0)
    movwf   OSCCON ; set internal oscillator frequency to 8 MHz

    movlw   (0<<NOT_RABPU)
    movwf   OPTION_REG

            ;   ,------ input 6
            ;   |,----- input 5
            ;   ||,---- input 4
            ;   |||,--- input 3
            ;   ||||,-- input 2
            ;   |||||,- input 1
    movlw   b'11111111'
    movwf   TRISA ; set PORTA as input
    movwf   WPUA ; enable weak pull-up
    movwf   IOCA ; enable interrupt-on-change

            ; ,-------- tx
            ; |,------- input 6
            ; ||,------ rx
            ; |||,----- input 7
    movlw   b'01111111'
    movwf   TRISB

            ;   ,------ output 4
            ;   |,----- output 3
            ;   ||,---- output 2
            ;   |||,--- output 1
            ;   ||||,-- status
            ;   |||||,- n. c.
    movlw   b'11000001'
    movwf   TRISC

    movlw   (1<<TXEN|1<<BRGH)
    movwf   TXSTA ; enable async transmitter and select high baud rate

    movlw   (1<<BRG16)
    movwf   BAUDCTL ; 16-bit baud rate generator

    movlw   .16 ; use value 16 for SPBRG:SPBRGH to get 115200bps baud rate
    movwf   SPBRG
    clrf    SPBRGH

    ; Bank 2
    bcf     STATUS, RP0
    bsf     STATUS, RP1

    clrf    ANSEL
    clrf    ANSELH

            ;  ,------- input 6
            ;  | ,----- input 7
    movlw   b'01010000'
    movwf   WPUB
    movwf   IOCB

    ; bank 0
    clrf    STATUS

    ; clear the first two banks
    movlw   0x020
    call    ClearBank
    movlw   0x0A0
    call    ClearBank

    movlw   (1<<SPEN|1<<CREN)
    movwf   RCSTA

    movlw   (1<<T1CKPS0|1<<TMR1ON)
    movwf   T1CON ; enable timer and set frequency to 1MHz (=8Mhz/4/2)

    movlw   (1<<GIE|1<<PEIE|1<<RABIE)
    movwf   INTCON

MainLoopNoUpdate:
    ; toggle status led if idle for more than INPUT_NUM * 2000 usec
    movf    timer_h, w
    subwf   TMR1H, w
    sublw   high(.2000 * INPUT_NUM)
    btfsc   STATUS, C
    goto    MainLoop
    ; toggle status led at each 8 timer1 overflows, i.e., about 0.52 sec
    btfss   PIR1, TMR1IF
    goto    MainLoop
    bcf     PIR1, TMR1IF
    incf    tmr1if_count, f
    movlw   H'07'
    andwf   tmr1if_count, w
    btfss   STATUS, Z
    goto    MainLoop
    ; toggle status led
    movf    PORTC, w
    xorlw   (1<<RC1)
    movwf   PORTC
MainLoop:
    ; loop if no updates
    movf    sent_update, w
    subwf   input_updated, w
    btfsc   STATUS, Z
    goto    MainLoopNoUpdate

    ; prepare for mirroring
    addwf   sent_update, f
    movlw   (INPUT_NUM * 2)
    movwf   count
    movlw   input_value
    movwf   FSR

MirrorLoop:
    movf    INDF, w
    bsf     STATUS, IRP
    movwf   INDF
    bcf     STATUS, IRP
    incf    FSR, f
    decfsz  count, f
    goto    MirrorLoop

    ; restart if there was update while mirroring
    movf    input_updated, w
    subwf   sent_update, w
    btfss   STATUS, Z
    goto    MainLoop

    ; finally send the captured values
    ; send a magic number to sync
    movlw   low(MAGIC)
    call    Send
    movlw   high(MAGIC)
    call    Send

    ; prepare for send loop
    movlw   (INPUT_NUM * 2)
    movwf   count
    movlw   low(input_mirror)
    bsf     STATUS, IRP
    movwf   FSR

SendLoop:
    movf    INDF, w
    call    Send
    incf    FSR, f
    decfsz  count, f
    goto    SendLoop

    bcf     STATUS, IRP

    ; set led on
    bcf     PORTC, RC1

    goto    MainLoop

Send:
    btfss   PIR1, TXIF
    goto    $ - 1
    movwf   TXREG
    return

ClearBank:
    ; clears the gpr bank pointed by w plus irp,
    ; but preserves the common 16 bytes.

    ; ensure the beginning of the bank
    andlw   b'10100000'
    movwf   FSR
    movlw   .80
    movwf   count

ClearBankLoop:
    clrf    INDF
    incf    FSR, f
    decfsz  count, f
    goto    ClearBankLoop
    return

    end

