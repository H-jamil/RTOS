;/*****************************************************************************/
; OSasm.s: low-level OS commands, written in assembly                       */
; Runs on TM4C123
; Lab 4 starter file Real Time Bluetooth Network Course.
; March 25, 2016
; Author: Jamil Hasibul
;


        AREA |.text|, CODE, READONLY, ALIGN=2
        THUMB
        REQUIRE8
        PRESERVE8

        EXTERN  RunPt            ; currently running thread
        EXPORT  StartOS
        EXPORT  SysTick_Handler
        IMPORT  Scheduler


SysTick_Handler                 ; 1) Saves R0-R3,R12,LR,PC,PSR
    CPSID   I                   ; 2) Prevent interrupt during switch
    PUSH    {R4-R11}            ; 3) Save remainning R4-11
    LDR     R0, =RunPt          ; 4) R0 = pointer RunPt to the old thread
    LDR     R1, [R0]            ;    R1 = value of RunPt
    STR     SP, [R1]            ; 5) Save SP into TCB

    PUSH    {R0, LR}
    BL      Scheduler
    POP     {R0, LR}
    LDR     R1, [R0]

    LDR     SP, [R1]            ; 7) New thread SP = RunPt->sp
    POP     {R4-R11}            ; 8) Restore R4-11
    CPSIE   I                   ; 9) tasks run with interrupts enabled
    BX      LR                  ; 10) restore R0-R3,R12,LR,PC,PSR

StartOS
    LDR     R0, =RunPt          ; Currently running thread
    LDR     R1, [R0]            ; R1 = value of RunPt
    LDR     SP, [R1]            ; New thread SP = RunPt->sp
    POP     {R4-R11}            ; restore register 4-11
    POP     {R0-R3}             ; restore register 0-3
    POP     {R12}
    ADD     SP, SP, #4          ; discard LR from initial stack
    POP     {LR}                ; start location
    ADD     SP, SP, #4          ; discard PSR
    CPSIE   I                   ; Enable interrupts at processor level
    BX      LR                  ; start first thread

    ALIGN
END
