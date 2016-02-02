;;;
;;; Light controller firmware
;;;
;;; This is the PIC assembler source to the toy light
;;; controller I built for my shed. Read about it at
;;; http://mjoldfield.com/atelier/2016/01/lopo-lipo.html
;;;
;;; Copyright (c) Martin Oldfield, 2016
;;; 
;;; NOTICE: THIS CODE COMES WITHOUT WARRANTY OF ANY KIND EITHER
;;;         EXPRESSED OR IMPLIED. USE THIS CODE AT YOUR OWN RISK!
;;;         I WILL NOT BE HELD RESPONSIBLE FOR ANY DAMAGES, DIRECT 
;;;         OR CONSEQUENTIAL THAT YOU MAY EXPERIENCE BY USING IT.
;;;
;;; It uses Chuck McManis' 16-bit arithmetic library from
;;; https://github.com/ChuckM/PIC-Software/blob/master/16bits.inc
;;; 
;;; All multibyte values are little endiaan i.e. lsb first
;;; 

;;;
;;; Code conventions
;;; 
;;; The code implicitly assumes that code will fit in the first
;;; section of program memory.
;;;
;;; All the modules adopt the convention that they are in BANK 0
;;; when they are called, and must return to BANK 0 by the time they
;;; end.
;;; 
	
#include <P16F690.inc> 

        __CONFIG (_INTOSCIO & _WDT_OFF & _PWRTE_OFF & _MCLRE_OFF & _CP_OFF & _IESO_OFF & _FCMEN_OFF & _BOD_OFF)

;;; ex https://raw.githubusercontent.com/ChuckM/PIC-Software/master/16bits.inc
#Include "16bits.inc"

		RADIX	dec

;;; time light stays on in half-seconds
LIGHT_TIME	EQU	512
	
;;; if non-zero include serial debug stuff
SERIAL		EQU	0

;;; ADC configuration
;;;
;;; The ADC system samples a number of channels, and uses discarded
;;; conversions as a delay mechanism. No effort has been made to
;;; optimize these delays. Note that e.g. the status LED also hangs
;;; off this code, so changing them will affect its brightness too.
;;; 
ADC_N_CHANNELS		EQU	4	; if > 4 look at adc_scale_reading
ADC_N_DISCARD_OFF	EQU	50      ; XXX
ADC_N_DISCARD_ON	EQU	22      ; XXX
				
;;; Voltage thresholds for each LiPo cell
;;; Above V_HIGH or below V_SHUTDOWN are errors, below V_LOW is a warning
V_HIGH		EQU	0xf0	; with 3.3k / 22k pot. div & Vref = 2.5, 0xf0 => 4.5V
V_LOW		EQU	0xba	; with 3.3k / 22k pot. div & Vref = 2.5, 0xba => 3.5V
V_SHUTDOWN	EQU	0xaa	; with 3.3k / 22k pot. div & Vref = 2.5, 0xaa => 3.2V
	

;;;
;;; Macros to force a bank
;;; 	
BANK0	macro
	bcf	STATUS,RP1
	bcf	STATUS,RP0
	endm

BANK1	macro
	bcf	STATUS,RP1
	bsf	STATUS,RP0
	endm

BANK2	macro
	bsf	STATUS,RP1
	bcf	STATUS,RP0
	endm

BANK3	macro
	bsf	STATUS,RP1
	bsf	STATUS,RP0
	endm

;;;
;;; Basic Registers
;;; - some of these could be overlapped
;;; 
        cblock 0x20
clock_l				; system clock, ticks at 2Hz
clock_h
light_off_l			; time to turn off the main light
light_off_h			; in clock units
adc_l				; ADC reading in in ADC units
adc_h	
adc_old_l			; Previous ADC reading in ADC units
adc_old_h	
adc_discard			; Discarded sample counter
adc_ch_and_ph			; 2 * channel number | phase 
adc_ch_mosfet_mask		; Control byte for sensing MOSFETs
adc_delta_l			; Difference between new and old ADC
adc_delta_h
adc_cell_status			; Status flags for cell voltages

;;; the following registers are used only if the serial subsystem
;;; is enabled
serial_tmp			; workspace

adc_save			; 4 x 16-bit ADC readings
adc_save1	
adc_save2	
adc_save3	
adc_save4	
adc_save5	
adc_save6	
adc_save7
        endc
	
;;; These registers need to be common to all banks for the interrupt handler
        cblock 0x71
w_inth_save
status_inth_save
pclath_inth_save
        endc

;;; 
;;; The code starts here: reset and interrupt vectors
;;; 
        
        org     0x000
        goto    main

        org     0x004
        goto    int_handler

main:
	BANK0

	;;  Bank 0 init
	clrf	PORTA
	clrf	PORTB
	clrf	PORTC

	;;  Bank 2 init
	BANK2

	clrf	ANSEL
	clrf	ANSELH

	;; Back to Bank 0
	BANK0

	call	adc_init
	call	clock_init	
	call	light_init
	call	serial_init	; if not using serial sets RB7 to o/p

	call	unused_init
	
	;; Interrupts on: off we go!
	bsf	INTCON, PEIE
	bsf	INTCON, GIE

mainloop:
	sleep	
	goto 	mainloop
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; MASTER INTERRUPT HANDLER
;;;
int_handler:    
        ;; Save status
        movwf   w_inth_save
        swapf   STATUS, w
        movwf   status_inth_save
        movf    PCLATH, w
        movwf   pclath_inth_save

        clrf    PCLATH
        clrf    STATUS	

	btfsc   PIR1, TMR1IF            ; Timer1 is used for timekeeping
        goto    clock_int_handler
	
	btfsc	INTCON, RABIF		; All pin changes 
	goto	light_int_handler

	btfsc   PIR1, ADIF		; ADC conversion done
	goto    adc_int_handler
	
int_return:
        movfw   pclath_inth_save
        movwf   PCLATH
        swapf   status_inth_save, w
        movwf   STATUS
        swapf   w_inth_save,f
        swapf   w_inth_save,w

	retfie

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; UNUSED I/O
;;;
unused_init:
	;; set unused pins to be outputs

	BANKSEL	TRISA
	bcf	TRISA, RA0
	
	BANKSEL	TRISB
	bcf	TRISB, RB5

	BANK0
	return
	
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; SYSTEM CLOCK
;;; 
	;; Initialization: assume we have a 32.768kHz Xtal on timer 1
	;; Assume too that register bank 0 is selected
clock_init:
	clrf   clock_h             ; zero our counter
	clrf   clock_l
	                
	;; assume the T1CON, TMR1x are all in BANK 0

	bcf    T1CON, TMR1ON       ; stop timer
	
	movlw  0xff               ; roll-over on soon
	movwf  TMR1H
	clrf   TMR1L
	
	BANKSEL PIE1
	bsf    PIE1, TMR1IE        ; Enable timer 1 interrupt
	
	BANKSEL T1CON
	movlw  0x0e                ; 1:1 prescale, enabled, async, external
	movwf  T1CON
	bsf    T1CON, TMR1ON       ; start!

	;; assume T1CON in BANK 0 thus no need for explicit call

	return

clock_int_handler:
	;; assume the PIR1, TMR1x, PORTC are all in BANK 0

        bcf    PIR1, TMR1IF
        bsf    TMR1H, 7            ; Add 0xc0 to timer => 2Hz with 32.768kHz Xtal
        bsf    TMR1H, 6            

        incfsz clock_l,1
        goto   $ + 2
        incf   clock_h,1

	;; END OF CLOCK HANDLER. What follows is code for other modules
	;; which are tied to the timer tick
	
	call	light_tick
#IF SERIAL
	call	serial_tick
#ENDIF

	movfw	clock_l
	andlw	0x1
	btfsc	STATUS, Z
	call	adc_fresh_start
	
	goto	int_return

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; MAIN LIGHT CONTROL
;;;  - button RB6, light on RC0
;;; 

	;; Initialization: assume button on RB6
light_init:
	;; Watch RB7 for changes
	BANKSEL IOCB
	clrf	IOCB
	bsf	IOCB,   IOCB6

	;; Make RC0 an output
	BANKSEL	TRISC
	bcf	TRISC, RC0
	
	;;  assume INTCON, PORTB all in BANK 0
	BANK0

	bsf	INTCON, RABIE

	movfw	PORTB		; reads port and clears mismatch
	bcf	INTCON, RABIF

	return

;;; triggered on a pin change
light_int_handler:
	;;  assume INTCON, PORTB all in BANK 0
	movfw	PORTB		; reads port and clears mismatch
	bcf	INTCON, RABIF
	
	bsf	PORTC, RC0	; turn on light

	MOV16	clock_l, light_off_l
	ADDI16	light_off_l, LIGHT_TIME
	
	goto	int_return

light_tick:
	;; assume PORTC in BANK 0
	movfw	clock_l
	subwf	light_off_l, w
	btfss	STATUS, Z
	goto	light_stays_on

	movfw	clock_h
	subwf	light_off_h, w
	btfsc	STATUS, Z

light_off:	
	bcf	PORTC, RC0

light_stays_on:	
	return

	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;; 
;;; ADC code - see adc_fresh_start for # channels &c.;
;;;          - note adc_init does NOT start a conversion;
;;; 
adc_init:
	BANKSEL	ANSEL
	bsf	ANSEL, ANS2

	BANKSEL	ADCON1
	movlw	0x30		; use RC oscillator
	movwf	ADCON1

	BANKSEL ADCON0
	movlw  	0xc8		; right justified, Vref is Vref,  channel 2
	movwf  	ADCON0	

	BANKSEL	PIE1
	bsf    	PIE1, ADIE

	BANKSEL	TRISB
	bcf	TRISB, RB4	; RB4 heartbeat/status
	
	BANKSEL	TRISC
	movfw	TRISC	        ; RC1,2 heartbeat/status
	andlw	0x01		; RC3 needs to be an output to drive Vref,
	andwf	TRISC,F		; RC4,5,6,7 to drive MOSFETs on front-end

	;; turn on all status/heartbeat LEDs
	BANKSEL	PORTB
	bsf	PORTB, RB4

	BANKSEL	PORTC
	bsf	PORTC, RC1
	bsf	PORTC, RC2
	
	BANK0

	;; Assume PIR1, ADCON0 in BANK 0
	bcf    	PIR1, ADIF
	
	return

;;;
;;; Entry point for a new round of voltage readings
;;; 
adc_fresh_start:
	bsf	ADCON0, ADON	; turn on ADC
	
#IF SERIAL
	movlw	0xff		; initialize save space to silly values
	movwf	adc_save+0
	movwf	adc_save+1
	movwf	adc_save+2
	movwf	adc_save+3
	movwf	adc_save+4
	movwf	adc_save+6
	movwf	adc_save+7

	movlw	0x42
	call	serial_send_and_wait
	
#ENDIF
	bsf	PORTC, RC3	; supply power to voltage reference

	;; turn on status/heartbeat LEDs as appropriate
	;; - note we are showing the status from the last
	;;   set of readings!
	btfsc	adc_cell_status, 0
	bsf	PORTC, RC1	; heartbeat - red
	btfss	adc_cell_status, 1
	bsf	PORTC, RC2	; heartbeat - green
	btfsc	adc_cell_status, 2
	bsf	PORTB, RB4	; heartbeat - blue

	clrf	adc_ch_and_ph 	; channel 0, MOSFETs off

	clrf	adc_cell_status	; bit 0 -> shutdown voltage, bit 1 -> low voltage
	
	movlw	0x10
	movwf	adc_ch_mosfet_mask

	;; set all input channels to off
	bcf	PORTC, RC7
	bcf	PORTC, RC6
	bcf	PORTC, RC5
	bcf	PORTC, RC4

	CLR16	adc_old_l

adc_next_channel:
	movlw	ADC_N_DISCARD_ON	; Number of readings to discard from each channel
	btfsc	adc_ch_and_ph, 0
	movlw	ADC_N_DISCARD_OFF	
	movwf	adc_discard

	bsf	ADCON0, GO
	return
		
adc_int_handler:
	;; Assume PIR1, ADRESH, PORTC, ADCON0 all in BANK 0
	bcf	PIR1, ADIF
	
	movf	adc_discard,F
	btfsc	STATUS, Z
	goto	adc_wait_over
	
	decf	adc_discard, F

	;; waiting for something interesting to happen
	;; so just fire ADC off again
	bsf	ADCON0, GO
	goto	int_return

adc_wait_over:
	btfss	adc_ch_and_ph, 0
	goto	adc_inc_channel

adc_use_reading:	
	;; adc reading into adc_lh
	movfw   ADRESH
	movwf	adc_h

	BANKSEL	ADRESL
	movfw   ADRESL

	BANK0
	movwf	adc_l

	MOV16	adc_l, adc_delta_l
	SUB16	adc_delta_l, adc_old_l

#IF SERIAL
	;; save voltage reading to adc_save
	;; HIGH BYTE FIRST, for ease of printing
	movfw	adc_ch_and_ph	
	andlw	0xfe		; ignore phase in bit 0
	addlw	adc_save
	movwf	FSR
	movfw	adc_h
	movwf	INDF
	incf	FSR,F
	movfw	adc_l
	movwf	INDF
#ENDIF

	;; see if cell voltage too high (or -ve bcs using unsigned arith)
	CMPI16	adc_delta_l, V_HIGH
	btfss	STATUS, C	
	goto	adc_cell_below_overvoltage

	bsf	adc_cell_status, 2
	call	light_off
	
adc_cell_below_overvoltage

	;; see if cell voltage below shutdown
	CMPI16	adc_delta_l, V_SHUTDOWN
	btfsc	STATUS, C	
	goto	adc_cell_above_shutdown

	bsf	adc_cell_status, 1
	call	light_off

adc_cell_above_shutdown:	
	CMPI16	adc_delta_l, V_LOW
	btfsc	STATUS, C	
	goto	adc_cell_ok

	bsf	adc_cell_status, 0
		
adc_cell_ok:	
	MOV16	adc_l, adc_old_l

adc_inc_channel:
	;; toggle relevant MOSFET driver
	movfw	adc_ch_mosfet_mask
	xorwf	PORTC,F
	
	;; rotate mask every second iteration
	bcf	STATUS, C
	btfsc	adc_ch_and_ph, 0
	rlf	adc_ch_mosfet_mask, F

	;; now increment channel number
	incf	adc_ch_and_ph, f
	movfw	adc_ch_and_ph
	sublw	(ADC_N_CHANNELS * 2)
	btfsc	STATUS, Z
	goto	adc_all_done
	
	call	adc_next_channel
	goto	int_return
	
adc_all_done:
	bcf	ADCON0, ADON	; turn off ADC

	;; turn off all LEDs
	bcf	PORTC, RC1	
	bcf	PORTC, RC2
	bcf	PORTB, RB4

	;; turn off voltage regulator
	bcf	PORTC, RC3

	;; turn off input mosfets
	bcf	PORTC, RC7
	bcf	PORTC, RC6
	bcf	PORTC, RC5
	bcf	PORTC, RC4

#IF SERIAL
	call	serial_send_nl

	movlw	0x41		; 'A'
	call	serial_send_and_wait
	
	movlw	32
	call	serial_send_and_wait

	movfw	adc_cell_status
	call	serial_send_hex_byte

	movlw	adc_save
	movwf	FSR

	movlw	ADC_N_CHANNELS	; 
	movwf	adc_discard	; reuse this here bcs not in use atm

adc_l0:	
	movlw	32
	call	serial_send_and_wait

	movfw	INDF
	call	serial_send_hex_byte
	incf	FSR,F
	movfw	INDF
	call	serial_send_hex_byte
	incf	FSR,F

	decfsz	adc_discard,F
	goto	adc_l0

	call	serial_send_nl
#ENDIF

	goto	int_return
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; HEARTBEAT LED on RC1
;;;

;; HEARTBEAT now done in ADC module
	

#IF SERIAL	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;;;
;;; SERIAL DEBUG CONSOLE - 38400 on RB7, assuming 4MHz system clock
;;;

serial_init:
	BANKSEL	SPBRGH
	clrf	SPBRGH

	BANKSEL	SPBRG
	movlw	25
	movwf	SPBRG

	BANKSEL	BAUDCTL
	bsf	BAUDCTL, BRG16

	BANKSEL	RCSTA
	bsf	RCSTA, SPEN

	BANKSEL	TXSTA
	bsf	TXSTA, BRGH
	bcf	TXSTA, SYNC
	bsf	TXSTA, TXEN	

	;; send CR LF > SPC
	call	serial_send_nl
	movlw	0x3e
	call	serial_send_and_wait
	movlw	0x20
	call	serial_send_and_wait
	
	return

serial_tick:			; send .
	movlw	46
	;; deliberate fall through
	
serial_send_and_wait:		; transmit char in W
	BANKSEL	TXREG
	movwf	TXREG

	BANKSEL	TXSTA
not_yet_sent:
	btfss	TXSTA, TRMT
	goto	not_yet_sent

	BANK0

	return

serial_send_nl
	movlw	0x0a
	call	serial_send_and_wait
	movlw	0x0d
	goto	serial_send_and_wait
	
serial_send_hex_nybble:
	andlw	0x0f
	addlw	246		; W = W - 10
	btfsc	STATUS, C
	addlw	7		; 10 + 48 = 58, ord('A') = 65, 65 - 58 = 7
	addlw	58		; ord('0') = 48, but have to undo the sub 10 too
	goto	serial_send_and_wait


serial_send_hex_byte:
	movwf	serial_tmp
	swapf	serial_tmp,W
	call	serial_send_hex_nybble
	movfw	serial_tmp
	goto	serial_send_hex_nybble
#ELSE
serial_init:
	BANKSEL	TRISB
	bcf	TRISB, RB7

	BANK0
	return	
#ENDIF
	
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

	end

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
