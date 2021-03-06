;*******************************************************************************
; PIC16F1937
; I2C SLAVE DRIVER CODE
; Author: Chris Best
; Microchip Technologies
; DATE: 07/03/2013

;-------------------------------------------------------------------------------
; Software License Agreement
;
; The software supplied herewith by Microchip Technology Incorporated 
; (the 'Company') is intended and supplied to you, the Company's customer,
; for use solely and exclusively with products manufactured by the Company.
; The software is owned by the Company and/or its supplier, and is protected 
; under applicable copyright laws. All rights are reserved. Any use in
; violation of the foregoing restrictions may subject the user to criminal
; sanctions under applicable laws, as well as to civil liability for the breach
; of the terms and conditions of this license.
;
; THIS SOFTWARE IS PROVIDED IN AN 'AS IS' CONDITION. NO WARRANTIES,
; WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO,
; IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
; APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT, IN ANY CIRCUMSTANCES, BE
; LIABLE FOR SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES, FOR ANY REASON
; WHATSOEVER.
;
; INTEGRATED DEVELOPMENT ENVIRONMENT: MPLABX IDE v1.80
; PROGRAMMER/DEBUGGER: PICKIT 3
; SIMULATION TOOL: PICKIT SERIAL ANALYZER
; LANGUAGE TOOLSUITE: MICROCHIP MPASMWIN (v5.49) 
;
; NOTES:
; The code implements the MSSP (or SSP) module as an I2C slave.
; This will allow the user to read from, or write to, a data array
; (32 bytes long).
; The user can read from or write to a specific location within the array,
; similar to using EEPROM memory. When using the PICkit Serial Analyzer
; set up as an I2C master, just enter the slave address, in this case 0x30,
; the word address to read or write, then up to eight bytes of data. There is
; also a fail-safe built in to protect from writing data into critical memory
; locations. It is important to keep in mind that this code is for demonstration
; of the MSSP module for slave I2C communications. It does not include
; other interrupt possibilities, which would need to be added, and may require
; this code to be modified. 
;
;-------------------------------------------------------------------------------


#include <p16F1937.inc> 
; CONFIG1
; __config 0xFE5C
__CONFIG _CONFIG1, _FOSC_INTOSC & _WDTE_ON & _PWRTE_ON & _MCLRE_ON & _CP_ON & _CPD_ON & _BOREN_ON & _CLKOUTEN_OFF & _IESO_ON & _FCMEN_ON 
; CONFIG2
; __config 0xFFFF
 __CONFIG _CONFIG2, _WRT_OFF & _VCAPEN_OFF & _PLLEN_ON & _STVREN_ON & _BORV_LO & _LVP_ON

#define I2C_ADDRESS 	0x30				; Slave address
#define	RX_ELEMENTS		.32					; number of allowable array elements, in this case 32

;------------------------------------ variables--------------------------------------------------

cblock 0x70									; set up in shared memory for easy access
		INDEX_I2C							; index used to point to array location
		FIRST								; used to determine if it is first data byte		
endc
		udata
		I2C_ARRAY 		res RX_ELEMENTS 	; array to write to, array memory size
		

;-----------------------------------------------------------------------------------------------
; The macro LOADFSR loads FSR1 with the I2C address and the I2C index value to read or write to
; and makes code easier to read.
;------------------------------------- LOADFSR macro -------------------------------------------

LOADFSR macro 	ADDRESS,INDEX 				; ADDRESS = I2C_ARRAY, INDEX = INDEX_I2C			
		movlw 	ADDRESS 					; load address 						
		addwf	INDEX,W						; add the index value to determine location in array
		movwf 	FSR1L						; load FSR1L with pointer info
		clrf	FSR1H
		endm		
;-----------------------------------------------------------------------------------------------


;----------------------------------- start vectors ---------------------------------------------
 		ORG    	0x0000          			; reset vector
    	goto   	Main                    	; jump to main
;-----------------------------------------------------------------------------------------------

;----------------------------------- interrupt vector ------------------------------------------
    	ORG    	0x0004          			; interrupt vector
		banksel	PIR1
		btfss 	PIR1,SSPIF 					; Is this a SSP interrupt?
		goto 	BUS_COLL 					; if not, bus collision int occurred
    	banksel	SSPSTAT						
		btfsc	SSPSTAT,2					; is it a master read:
		goto	READ						; if so go here
		goto	WRITE						; if not, go here
;-----------------------------------------------------------------------------------------------


;-----------------------------------------------------------------------------------------------
;	                                         Main
;-----------------------------------------------------------------------------------------------
		ORG		0x0040        				; start of Flash memory  

Main	
		call	INITIALIZE					; set up uC

LOOP	
		clrwdt								; infinite while loop
		goto	LOOP
;------------------------------------- end main ------------------------------------------------
	
;-----------------------------------------------------------------------------------------------
;	Initialize: Sets up register values 
;-----------------------------------------------------------------------------------------------
INITIALIZE
;uC set up
		banksel	OSCCON
		movlw	b'01111010'					; Internal OSC @ 16MHz, Internal OSC block selected 
		movwf	OSCCON                      ; 4xPLL is enabled (set in config word) 
		movlw	b'11010111'                 ; WPU disabled, INT on rising edge, FOSC/4,
		movwf	OPTION_REG					; Prescaler assigned to TMR0, prescaler rate 1:256
		movlw	b'00010111'					; WDT prescaler 1:65536 period is 2 sec (RESET value)
		movwf	WDTCON 
		banksel LATC
    	clrf    LATC                        ; Clear PORTC latches
		banksel TRISC                   	
        bsf     TRISC,3                     ; Set RC3 to input (must be done)
        bsf     TRISC,4                     ; Set RC4 to input (must be done)
		clrf	INDEX_I2C                   ; Clear the index pointer
		clrf	FIRST                       ; Clear FIRST
		bsf		FIRST,0                     ; set bit 0 in FIRST
;I2C set up
		banksel	SSPSTAT
		bsf		SSPSTAT,SMP					; Slew rate control disabled for standard speed mode
		movlw	b'00110110'					; Enable serial port, I2C slave mode, 7-bit address
		movwf	SSPCON1
		bsf		SSPCON2,SEN					; enable clock stretching
		bsf		SSPCON3,BOEN				; SSPBUF is updated and NACK is generated 	
											; for a received address/data byte, 
											; ignoring the state of SSPOV bit only if BF bit = 0.
		bsf		SSPCON3,SDAHT				; Minimum of 300 ns hold time
		movlw	I2C_ADDRESS					; load the slave address
		movwf	SSPADD
	
		banksel	PIR1
		bcf		PIR1,SSPIF					; clear the SSP interrupt flag	
		banksel	PIE1
		bsf		PIE1,SSPIE					; enable SSP interrupts
		banksel	PIR2
		bcf		PIR2,BCLIF					; clear the SSP interrupt flag
		banksel	PIE2
		bsf		PIE2,BCLIE					; enable SSP interrupts
		bsf		INTCON,PEIE					; enable peripheral interrupts
		bsf		INTCON,GIE					; enable global interrupts
		return
;------------------------------------- END INITIALIZE ------------------------------------------

;-----------------------------------------------------------------------------------------------				
;                                Interrupt Service Routine (ISR)
;-----------------------------------------------------------------------------------------------
READ	
		banksel	SSPSTAT						
		btfss	SSPSTAT,5					; was last byte an address or data?
		goto	R_ADDRESS					; if clear, it was an address
		goto	R_DATA						; if set, it was data
		
R_ADDRESS		
		LOADFSR	I2C_ARRAY,INDEX_I2C			; call LOADFSR macro
		movf	INDF1,W						; move value into W to load to SSP buffer
		banksel	SSPBUF
		movwf	SSPBUF						; load SSP buffer
		incf	INDEX_I2C,F					; increment INDEX_I2C 'pointer'
        banksel	SSPCON1
		btfsc	SSPCON1,WCOL				; did a write collision occur?
		goto  	WRITE_COLL					; if so, go clear bit
		banksel	SSPCON1				
		bsf		SSPCON1,CKP					; release clock stretch
		banksel	PIR1
		bcf 	PIR1,SSPIF					; clear the SSP interrupt flag
		goto    EXIT 						; Go to EXIT to return from interrupt

R_DATA		
		LOADFSR	I2C_ARRAY,INDEX_I2C			; call LOADFSR macro
		movf	INDF1,W						; move value into W to load to SSP buffer
		banksel	SSPBUF						
		movwf	SSPBUF						; load SSP buffer
		incf	INDEX_I2C,F					; increment INDEX_I2C 'pointer'
        banksel	SSPCON1
		btfsc	SSPCON1,WCOL				; did a write collision occur?
		goto  	WRITE_COLL					; if so, go clear bit
		banksel	SSPCON1
		bsf		SSPCON1,CKP					; release clock stretch
		banksel	PIR1
		bcf 	PIR1,SSPIF					; clear the SSP interrupt flag
		goto    EXIT 						; Go to EXIT to return from interrupt


WRITE
		banksel	SSPSTAT						
		btfss	SSPSTAT,5					; was last byte an address or data?
		goto	W_ADDRESS					; if clear, it was an address
		goto	W_DATA						; if set, it was data

W_ADDRESS
        bsf		FIRST,0						; set bit 0 of FIRST
		banksel	SSPBUF
		movfw	SSPBUF						; move the contents of the buffer into W
                                            ; dummy read to clear the BF bit
		banksel	SSPCON1
		bsf		SSPCON1,CKP					; release clock stretch
		banksel	PIR1
		bcf 	PIR1,SSPIF					; clear the SSP interrupt flag
		goto    EXIT 						; Go to EXIT to return from interrupt

W_DATA
		btfss	FIRST,0						; is bit 0 set?
		goto	W_DATA_SUB					; if clear, go write data
		banksel	SSPBUF						; if set, first data byte is location to write to
		movfw	SSPBUF
		movwf	INDEX_I2C					; load INDEX_I2C with data address to write to
		clrf	FIRST						; clear the FIRST register
        banksel	SSPCON1
		btfsc	SSPCON1,WCOL				; did a write collision occur?
		goto  	WRITE_COLL					; if so, go clear bit
		bsf		SSPCON1,CKP					; release clock stretch
		banksel	PIR1
		bcf 	PIR1,SSPIF					; clear the SSP interrupt flag	
		goto    EXIT 						; Go to EXIT to return from interrupt

W_DATA_SUB
		clrf	W                           ; clear W
		movlw	RX_ELEMENTS					; load array elements value
		banksel STATUS						
		btfsc	STATUS,Z					; is Z clear?  
		subwf	INDEX_I2C,W					; if Z = 1, subtract index from number of elements
		banksel	STATUS		
		btfsc	STATUS,0					; did a carry occur after subtraction?
		goto	NO_MEM_OVERWRITE			; if so, Master is trying to write to many bytes
		LOADFSR	I2C_ARRAY,INDEX_I2C			; call LOADFSR macro
		banksel	SSPBUF
		movfw	SSPBUF						; move the contents of the buffer into W
		movwf 	INDF1						; load INDF1 with data to write
		incf	INDEX_I2C,F					; increment 'pointer' to next memory location
        banksel	SSPCON1
		btfsc	SSPCON1,WCOL				; did a write collision occur?
		goto  	WRITE_COLL					; if so, go clear bit
		banksel	SSPCON1
		bsf		SSPCON1,CKP					; release clock stretch
		banksel	PIR1
		bcf 	PIR1,SSPIF					; clear the SSP interrupt flag
		goto    EXIT 						; Go to EXIT to return from interrupt
		
NO_MEM_OVERWRITE
		banksel	SSPBUF
		movfw	SSPBUF						; move SSP buffer to W
                    						; clear buffer so no overwrite occurs
		banksel	SSPCON1
		bsf		SSPCON1,CKP					; release clock stretch
		banksel	PIR1
		bcf 	PIR1,SSPIF					; clear the SSP interrupt flag
		goto    EXIT 						; Go to EXIT to return from interrupt

WRITE_COLL
		banksel	SSPCON1
		bcf		SSPCON1,WCOL				; clear WCOL bit
		banksel	SSPBUF
		movfw	SSPBUF						; move SSP buffer to W
                    						; clear SSP buffer
		banksel	SSPCON1
		bsf		SSPCON1,CKP					; release clock stretch
		banksel	PIR1
		bcf 	PIR1,SSPIF					; clear the SSP interrupt flag
		goto    EXIT 						; Go to EXIT to return from interrupt

BUS_COLL
		banksel	SSPBUF						
		clrf	SSPBUF						; clear the SSP buffer
		banksel	PIR2
		bcf		PIR2,BCLIF					; clear the SSP interrupt flag	
		banksel	SSPCON1
		bsf		SSPCON1,CKP					; release clock stretch
		goto    EXIT						; Go to EXIT to return from interrupt

EXIT
        retfie                              ; Return from interrupt.
		end									; END OF PROGRAM