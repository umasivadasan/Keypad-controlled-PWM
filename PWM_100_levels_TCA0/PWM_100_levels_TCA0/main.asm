;
; PWM_16_levels.asm
;
; Created: 11/2/2023 6:15:45 PM
; Author : umasi
;


/***********************Initialize LCD DOG*****************************/
.equ PERIOD_EXAMPLE_VALUE = 100		; 1% resolution for duty cycle setting
//.equ DUTY_CYCLE_EXAMPLE_VALUE = 25	;desired duty cycle as percent

.CSEG
     ;interrupt vector table, with several 'safety' stubs
     rjmp RESET      ;Reset/Cold start vector
     reti            ;External Intr0 vector
     reti            ;External Intr1 vector 

.org PORTE_PORT_vect
	jmp porte_ISR		;vector for all PORTE pin change IRQs

RESET:
    sbi VPORTA_DIR, 7		; set PA7 = output.                   
    sbi VPORTA_OUT, 7		; set /SS of DOG LCD = 1 (Deselected)

    rcall init_lcd_dog    ; init display, using SPI serial interface
    rcall clr_dsp_buffs   ; clear all three SRAM memory buffer lines

   rcall update_lcd_dog		;display data in memory buffer on LCD

  /********************The buffers for each line ***************************/
  ;load_line_1 into dbuff1:
   ldi  ZH, high(Test_message_1<<1)  ; pointer to line 1 memory buffer
   ldi  ZL, low(Test_Message_1<<1)   ;
   rcall load_msg          ; load message into buffer(s).

	;breakpoint followin instr. to see blanked LCD and messages in buffer
   rcall update_lcd_dog		;breakpoint here to see blanked LCD

/********************Configure Pins****************************/

start:
	ldi r22, 0xF0
	out VPORTC_DIR, r22 ;configure r16 as inputs
	ldi r16, 0xFF
	cbi VPORTB_DIR, 5 ;configure FF flag as the output (q)
	sbi VPORTB_DIR, 4 ;confgigure FF clear as the output
	sbi VPORTB_OUT, 4 ;clearing the FF
	sbi VPORTD_DIR, 0 //configure LED as outputs
	cbi VPORTD_OUT, 0 //output LED
	sbi VPORTD_DIR, 1 //configure LED as outputs
	cbi VPORTD_OUT, 1 //output LED
	ldi r19, 0x00 ;counter for number of digits entered

;Configure interrupt
	lds r16, PORTE_PIN0CTRL	;set ISC for PE0 to pos. edge
	ori r16, 0x02
	sts PORTE_PIN0CTRL, r16
	sei
				/******************TCA0 Conficturation*****************/
TCA:
	;Route WO0 to PD0 instead of its default pin PA0
	ldi r16, 0x03		;mux TCA0 WO0 to PD0
	sts PORTMUX_TCAROUTEA, r16

	;Set CTRLB to use CMP0 and single slope PWM
	ldi r16, TCA_SINGLE_CMP0EN_bm | TCA_SINGLE_WGMODE_SINGLESLOPE_gc ;CMP0EN and single slope PWM
	sts TCA0_SINGLE_CTRLB, r16

	;Load low byte then high byte of PER period register
	ldi r16, LOW(PERIOD_EXAMPLE_VALUE)		;set the period
	sts TCA0_SINGLE_PER, r16
	ldi r16, HIGH(PERIOD_EXAMPLE_VALUE)
	sts TCA0_SINGLE_PER + 1, r16

	;Load low byte and the high byte of CMP0 compare register
	//ldi r16, LOW(DUTY_CYCLE_EXAMPLE_VALUE)		;set the duty cycle
	sts TCA0_SINGLE_CMP0BUF, r20	;use TCA0_SINGLE_CMP0BUF for double buffering
	ldi r22, 0x00
	sts TCA0_SINGLE_CMP0BUF+1, r22
	//ldi r16, HIGH(DUTY_CYCLE_EXAMPLE_VALUE)
	//sts TCA0_SINGLE_CMP0 + 1, r16

	;Set clock and start timer/counter TCA0
	ldi r16, TCA_SINGLE_CLKSEL_DIV64_gc | TCA_SINGLE_ENABLE_bm
	sts TCA0_SINGLE_CTRLA, r16
	

/****************************Check the interrupt*****************************/
read_waveform:
	rjmp TCA
	rjmp read_waveform

porte_ISR:
	cli				;clear global interrupt enable, I = 0
	push r16		;save r16 then SREG, note I = 0
	in r16, CPU_SREG
	push r16

	lds r16, PORTE_INTFLAGS	;check for PE0 IRQ flag set
	sbrc r16, 0
	rcall update_PWM_sbr
	pop r16			;restore SREG then r16
	out CPU_SREG, r16	;note I in SREG now = 0
	pop r16
	sei				;SREG I = 1
	reti			;return from PORTE pin change ISR

update_PWM_sbr:
/***************************Read the keypad**********************/
read_keypad:
	in r17, VPORTC_IN
	com r17
	rcall keycode_to_scancode
	cpi r17, 'C' //checking if the key value was to enter a value
	breq inc_count
	cpi r17, 'A' //checking if the key value was to clear the value
	breq clear_code_label_2
;check for only numbers
	rcall check_letters
;check number limit
	rjmp input_check
check_waves:
	rcall update_lcd_dog
	ldi r16, PORT_INT0_bm	;clear IRQ flag for PE0
	sts PORTE_INTFLAGS, r16
	ret
	
/********************Check which line**********************/
inc_count:
	inc r19
	ldi r16, PORT_INT0_bm	;clear IRQ flag for PE0
	sts PORTE_INTFLAGS, r16
	ret

input_check:
	cpi r19, 0x00
	breq check_valid_output_cycle  
	cpi r19, 0x01
	breq valid_period_output
	ldi r19, 0x00
	rjmp input_check
/******************Enter Code*****************************/

valid_period_output:
rjmp check_valid_output_period
clear_code_label_2:
	rjmp clear_code
;do the loop 13, 14, 15
clear_FF_label_2:
	ldi r16, PORT_INT0_bm	;clear IRQ flag for PE0
	sts PORTE_INTFLAGS, r16
	ret

check_valid_output_cycle:
	ldi YH, high (dsp_buff_1) ; Load YH and YL as a pointer to 1st
    ldi YL, low (dsp_buff_1)  ; byte of dsp_buff_1 (Note - assuming
	
	ldd r18, Y+13
	std Y+12, r18
	
	ldd r18, Y+14
	std Y+13, r18
	
	std Y+14, r17 

	ldd r20, Y+14
	ldd r23, Y+13
	ldd r24, Y+12
	subi r20, 48
	subi r23, 48
	subi r24, 48
	ldi r16, 10
	mul r24, r16
	mov r24, r0
	add r24, r23
	mul r24, r16
	mov r24, r0
	add r20, r24
	mov r4, r20
	mov r16, r1
	cpi r16, 1
	brsh interrupt_clear
	
;check if r20 <100
	cpi r20, 101
	brlo check_waves
	ldi r16, PORT_INT0_bm	;clear IRQ flag for PE0
	sts PORTE_INTFLAGS, r16
	ret
	
wave_form:
	mov r17, r4
	mov r20, r17
	cpi r19,1
	breq keep
	ldi r23, 100
keep:
	mov r22, r23
	sub r22, r20

	cpi r20, 0
	breq pwm_low
	sbi VPORTD_OUT, 0

pwm_high:
	dec r20
	nop
	nop
	brne pwm_high

	cpi r22, 0
	breq interrupt_clear
	cbi VPORTD_OUT, 0

//pwm_low_count:
pwm_low:
	dec r22
	nop
	nop
	brne pwm_low

;clear FF
interrupt_clear:
	ldi r16, PORT_INT0_bm	;clear IRQ flag for PE0
	sts PORTE_INTFLAGS, r16
	ret
	
clear_code_label:
	rjmp clear_code
/**********************Changing the Period************************/
check_valid_output_period:
//29, 30, 31
	ldi YH, high (dsp_buff_1) ; Load YH and YL as a pointer to 1st
    ldi YL, low (dsp_buff_1)  ; byte of dsp_buff_1 (Note - assuming
	
	ldd r18, Y+30
	std Y+29, r18
	
	ldd r18, Y+31
	std Y+30, r18
	
	std Y+31, r17 
	ldd r20, Y+31
	ldd r23, Y+30
	ldd r24, Y+29
	subi r20, 48
	subi r23, 48
	subi r24, 48
	ldi r16, 10
	mul r24, r16
	mov r24, r0
	add r24, r23
	mul r24, r16
	mov r24, r0
	add r20, r24
	mov r23, r20
	mov r16, r1
	cpi r16, 1
	brsh interrupt_clear
	
;check if r20 <100
	cpi r23, 101
	brlo multiply_period
	ldi r16, PORT_INT0_bm	;clear IRQ flag for PE0
	sts PORTE_INTFLAGS, r16
	ret
	
multiply_period:
	ldi r22, 100
	mul r23, r22
	mov r23, r0
	rjmp check_waves

/******************Clear Code*******************/
;clear the numbers
clear_code:
	rcall clr_dsp_buffs
	rcall update_lcd_dog
	ldi ZH, high(Test_message_1<<1)  ; pointer to line 1 memory buffer
	ldi ZL, low(Test_Message_1<<1)   ;
	rcall load_msg          ; load message into buffer(s).

;update lcd
	rcall update_lcd_dog	
;clear FF
	ldi r16, PORT_INT0_bm	;clear IRQ flag for PE0
	sts PORTE_INTFLAGS, r16
	ret

/******************Check For letters***********************/
check_letters:
	cpi r17, 'B'
	breq int_clr
	cpi r17, 'D'
	breq int_clr
	cpi r17, 'E'
	breq int_clr
	cpi r17, 'F'
	breq int_clr
	ret

int_clr:
	ldi r16, PORT_INT0_bm	;clear IRQ flag for PE0
	sts PORTE_INTFLAGS, r16
	ret

	Test_message_1: .db 1, "Dty Cycle = 000%T Multiply = 001"

.include "lcd_dog_asm_driver_avr128.inc"
/***********************Subroutines**************************/

;************************
;NAME:      clr_dsp_buffs
;FUNCTION:  Initializes dsp_buffers 1, 2, and 3 with blanks (0x20)
;ASSUMES:   Three CONTIGUOUS 16-byte dram based buffers named
;           dsp_buff_1, dsp_buff_2, dsp_buff_3.
;RETURNS:   nothing.
;MODIFIES:  r25,r26, Z-ptr
;CALLS:     none
;CALLED BY: main application and diagnostics
;********************************************************************
clr_dsp_buffs:
     ldi R25, 48               ; load total length of both buffer.
     ldi R26, ' '              ; load blank/space into R26.
     ldi ZH, high (dsp_buff_1) ; Load ZH and ZL as a pointer to 1st
     ldi ZL, low (dsp_buff_1)  ; byte of buffer for line 1.
   
    ;set DDRAM address to 1st position of first line.
store_bytes:
     st  Z+, R26       ; store ' ' into 1st/next buffer byte and
                       ; auto inc ptr to next location.
     dec  R25          ; 
     brne store_bytes  ; cont until r25=0, all bytes written.
     ret

;************************
;NAME:      load_msg
;FUNCTION:  Initializes dsp_buffers 1, 2, and 3 with blanks (0x20)
;ASSUMES:   Three CONTIGUOUS 16-byte dram based buffers named
;           dsp_buff_1, dsp_buff_2, dsp_buff_3.
;RETURNS:   nothing.
;MODIFIES:  r25,r26, Z-ptr
;CALLS:     none
;CALLED BY: main application and diagnostics
;********************************************************************
load_msg:
     ldi YH, high (dsp_buff_1) ; Load YH and YL as a pointer to 1st
     ldi YL, low (dsp_buff_1)  ; byte of dsp_buff_1 (Note - assuming 
                               ; (dsp_buff_1 for now).
     lpm R16, Z+               ; get dsply buff number (1st byte of msg).
     cpi r16, 1                ; if equal to '1', ptr already setup.
     breq get_msg_byte         ; jump and start message load.
     adiw YH:YL, 16            ; else set ptr to dsp buff 2.
     cpi r16, 2                ; if equal to '2', ptr now setup.
     breq get_msg_byte         ; jump and start message load.
     adiw YH:YL, 16            ; else set ptr to dsp buff 2.
        
get_msg_byte:
     lpm R16, Z+               ; get next byte of msg and see if '0'.        
     cpi R16, 0                ; if equal to '0', end of message reached.
     breq msg_loaded           ; jump and stop message loading operation.
     st Y+, R16                ; else, store next byte of msg in buffer.
     rjmp get_msg_byte         ; jump back and continue...
msg_loaded:
     ret

keycode_to_scancode:
	andi r17, $0F
	brlo lookup
	clc
	ldi r17, 0 
	ret
lookup:
	ldi ZH, high(segtable *2)
	ldi ZL, low (segtable * 2)
	ldi r21, $00
	add ZL, r17
	adc ZH, r21
	lpm r17, Z
	ret
//lookup table:
segtable: .db 'C', 'B', '0', 'A', 'D', '9', '8', '7', 'E', '6', '5', '4', 'F', '3', '2', '1'




