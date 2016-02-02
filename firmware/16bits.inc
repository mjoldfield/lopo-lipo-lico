; 16BITS.INC      16 bit functions
; This Version    01-FEB-02    
; vim: set syntax=pic :
; RCS Version   
; $Id: 16bits.inc,v 1.2 2002-04-22 22:35:05-07 cmcmanis Exp cmcmanis $
;
               NOLIST
; Written by      Chuck McManis (http://www.mcmanis.com/chuck)
; Copyright (c) 2001 Charles McManis, All Rights Reserved
;
; The first byte is the LOW byte and the second byte is the HIGH
; byte in this set of routines. 
;
; $Id:$
;
; Change Log:
;       26-JAN-13       Removed declaration of _REG_A and _REG_B
;       22-JAN-13       Fixed ADD16 and SUB16 Macros
;       20-APR-O2       Updated CMP16, CMPI16
;       06-JAN-02       Created from the MATH16 file. Now
;                       it has a variety of operations.
;       30-DEC-01       Created this file
;
; NOTICE: THIS CODE COMES WITHOUT WARRANTY OF ANY KIND EITHER
;         EXPRESSED OR IMPLIED. USE THIS CODE AT YOUR OWN RISK!
;         I WILL NOT BE HELD RESPONSIBLE FOR ANY DAMAGES, DIRECT 
;         OR CONSEQUENTIAL THAT YOU MAY EXPERIENCE BY USING IT.
;
; * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
; 
; This include file provides some macros for dealing with
; 16 bit quantities. It assumes a little endian format
; where the least significant byte is lower in address than
; the most significant byte.
;
; * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

;
; 16 bit move from SRC to DST
;
MOV16   MACRO   SRC, DST
        MOVF    SRC,W
        MOVWF   DST
        MOVF    SRC+1,W
        MOVWF   DST+1
        ENDM

;
; Move Immediate macro
;        
MOVI16  MACRO   CONS, DST
        MOVLW   low (CONS)
        MOVWF   DST
        MOVLW   high (CONS)
        MOVWF   DST+1
        ENDM
              
;
; 16 bit unsigned compare, returns Z and C set appropriately
; Compares the registers "CMP_A" and "CMP_B"
; Flag states and what they mean:
;
;       Z true - X & Y are equal
;       C true - X > Y
;       C false - X < Y
;        
CMP16   MACRO   X, Y
        LOCAL   C_DONE
        
        MOVF    Y+1,W           ; Put it into W
        SUBWF   X+1,W           ; Compare with high byte of X (X MSB - Y MSB)
        BTFSS   STATUS,Z        ; If they are not zero we're done.
        GOTO    C_DONE
        MOVF    Y,W             ; Now get the low byte
        SUBWF   X,W             ; And compare it
C_DONE:        
        ENDM

;
; Compute X - Y and save the flags. Y is a literal.
;
CMPI16  MACRO   X, Y
        LOCAL   C_DONE          
        
        MOVLW   high Y          ; Put Y(high) into W
        SUBWF   X+1,W           ; Compute Xhigh - W == X - Y
        BTFSS   STATUS,Z        ; If they are the same compare low bytes
        GOTO    C_DONE          ; Else we know the answer already.
        MOVLW   low Y           ; Get the low byte
        SUBWF   X,W             ; Test it against X
C_DONE:                         ; bits are now set correctly. (C and Z)
        ENDM        

;
; Initialize a 16 bit value
;
INIT16  MACRO   VAR, CONST
        MOVLW   low CONST
        MOVWF   VAR
        MOVLW   high CONST
        MOVWF   VAR+1
        ENDM        

;
; Initialize a 16 bit value to zero
;
CLR16   MACRO   VAR
        CLRF    VAR
        CLRF    VAR+1
        ENDM
        
;
; Macro to do a logical shift right on a 16 bit value
; (0 is shifted into the MSB)
;
LSR16   MACRO   VAR16
        BCF     STATUS, C       ; Clear carry
        RRF     (VAR16)+1,F     ; Rotate high byte right
        RRF     (VAR16),F       ; Rotate low byte right
        ENDM
        
LSL16   MACRO   VAR16
        BCF     STATUS, C       ; Clear carry
        RLF     (VAR16),F       ; Rotate low byte left
        RLF     (VAR16)+1,F     ; Rotate upper byte left
        ENDM        
                
;
; 16 bit unsigned subtraction with carry out.
; Word format is little endian (LSB at lower address)
; Operation is DST = DST - SRC
;
; (This from the "tips and tricks" seminar handout)
;
; DST is replaced, SRC is preserved, Carry is set correctly
;
;
SUB16   MACRO   DST, SRC
        MOVF    (SRC),W         ; Get low byte of subtrahend
        SUBWF   (DST),F         ; Subtract DST(low) - SRC(low)
        MOVF    (SRC)+1,W       ; Now get high byte of subtrahend
        BTFSS   STATUS,C        ; If there was a borrow, rather than
        INCFSZ  (SRC)+1,W       ; add 1 to subtrahend, skip sub if 0
        SUBWF   (DST)+1,F       ; Subtract the high byte and we're done
        ENDM
        
SUBI16  MACRO   DST, SB
        MOVLW   LOW (SB)
        SUBWF   (DST), F
        MOVLW   HIGH (SB)
        BTFSS   STATUS, C
        MOVLW   (HIGH (SB))+1
        SUBWF   (DST)+1,F
        ENDM
                

;
; 16 bit unsigned addition with carry out.
; Operation: DST = DST + SRC                       
;
; DST is replaced, SRC is preserved, Carry is set correctly
;
ADD16   MACRO   DST,SRC 
        MOVF    (SRC),W         ; Get low byte
        ADDWF   (DST),F         ; Add to destination
        MOVF    (SRC)+1,W       ; Get high byte
        BTFSC   STATUS,C        ; Check for carry
        INCFSZ  (SRC)+1,W       ; Add one for carry skip if overflow
        ADDWF   (DST)+1,F       ; Add high byte into DST
        ENDM

;
; 16 bit Add Immediate
; Operation: DST = DST + Constant
;
; DST is updated, carry is set correctly.
;
ADDI16  MACRO   DST,AD
        MOVLW   LOW (AD)
        ADDWF   DST,F
        MOVLW   HIGH (AD)
        BTFSC   STATUS,C
        MOVLW   (HIGH (AD)) + 1
        ADDWF   (DST)+1,F
        ENDM
;
; Negate 16 bit value
; Find two's complement value of a 16 bit number
;
NEG16   MACRO   DST
        COMF    (DST)
        COMF    (DST)+1
        INC16   DST
        ENDM
                
;
; Increment 16 bit value, sets Z on exit.
;
; Operation: DST++
;
INC16   MACRO   DST        
        INCFSZ  (DST),W         ; Add one to low byte
        DECF    (DST)+1,F       ; No carry (negates next step)
        INCF    (DST)+1,F       ; Add one to high byte
        MOVWF   (DST)           ; Store updated low byte back.
        IORWF   (DST)+1,W       ; Set Z flag
        ENDM

;
; Decrement 16 bit value, sets Z on exit
;        
; Operation: DST--
;
DEC16   MACRO   DST
        DECF    (DST),F         ; Decrement low byte
        INCFSZ  (DST),W         ; Check for underflow
        INCF    (DST)+1,F       ; Update
        DECF    (DST)+1,F       ; Fixup
        MOVF    (DST),W
        IORWF   (DST)+1,W       ; Set Z bit
        ENDM
        
;
; Read a 16 bit value from the EEPROM
;
EEREAD16        MACRO   ADDR, VAR
        BSF     STATUS, RP0
        MOVLW   ADDR
        MOVWF   EEADR
        BSF     EECON1, RD
        MOVF    EEDATA,W
        MOVWF   EE_TMP
        INCF    EEADR,F
        BSF     EECON1, RD
        BTFSC   EECON1, RD
        GOTO    $-1
        MOVF    EEDATA,W
        MOVWF   EE_TMP+1
        BCF     STATUS, RP0
        MOV16   EE_TMP, VAR
        ENDM

        LIST
        
