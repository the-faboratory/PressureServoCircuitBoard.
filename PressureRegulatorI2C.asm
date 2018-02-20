; This assembly file runs on the PCB "Pressure Regulator Rev. 0" for networked 
; control of a pneumatic valve. Operation modes: picBangBang (0), masterDirectControl (1)

    
; Interfacing options:
;- To write memory, write [the memory address] followed by [the data].
;- To read memory, write [the memory address] and then read [the desired number of data bytes].
; Typical use case: 
; Arduino send INDEX_OPERATION_MODE (22), followed by an operation mode (0 for 
; PIC implemeted 3-state bang-bang; Arduino send INDEX_STATE (18), followed by a
; desired pressure (0 through 255, although 126 is atmospheric pressure). 
; Arduino send value of INDEX_PRESSURE_HIGH (19). Arduino request
; 2 bytes. PIC sends two bytes of pressure data.

    
; Circuit information
    ; Inputs
    ; RA2 - Pressure Feedback. Circuit diagram "PRESSURE"
    
    ; Outputs
    ; RC4 - release. Circuit diagram "SOL 1"
    ; RC5 - inflate. Circuit diagram "Sol 2"
    
    ; ICSP Programming
    ; RA0 - ICSPDAT = PGD
    ; RA1 - ICSPCLK = PGC
    ; RA3 - VPP
    
    ; I2C
    ; RC0 I2C_CLK = SCK
    ; RC1 I2C_DAT = SDI

;TODO: 
    
;*******************************************************************************
; Processor Inclusion & Configuration words
;*******************************************************************************

#include "p16f1825.inc"
; CONFIG1
; __config 0xFFE4
 __CONFIG _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _PWRTE_OFF & _MCLRE_ON & _CP_OFF & _CPD_OFF & _BOREN_ON & _CLKOUTEN_OFF & _IESO_ON & _FCMEN_ON
; CONFIG2
; __config 0xFFFF
 __CONFIG _CONFIG2, _WRT_OFF & _PLLEN_ON & _STVREN_ON & _BORV_LO & _LVP_ON

;*******************************************************************************
; Variable Definitions
;*******************************************************************************

#define I2C_ADDRESS		.86 ; Slave I2C address. 8 bit: 0000 0000 Arduino shifts left, i.e. PIC = Arduino*2
#define	RX_ELEMENTS		.32 ; number of allowable array elements, in this case 32
#define	INDEX_DESIRED_PRESSURE_HIGH    .16 ; High byte of our desired actuator power
#define	INDEX_DESIRED_PRESSURE_LOW	.17	; Low byte of our desired actuator power
#define	INDEX_STATE    .18 ; High byte of our desired actuator pressure
#define	INDEX_PRESSURE_HIGH .19	; High byte of our actuator pressure
#define	INDEX_PRESSURE_LOW .20	; Low byte of our actuator pressure
#define INDEX_HALF_BAND .21 ; 1/2*How wide our bang-bang deadband should be
#define INDEX_OPERATION_MODE .22 ; What mode should we operate in? picBangBang = 0. masterDirectControl = 1
    cblock 0x70					; set up in shared memory for easy access
		INDEX_I2C			; index used to point to array location
		BYTENUM				; used to determine if it is first data byte
    endc
		udata
i2cArray 	res RX_ELEMENTS			; array to write to, array memory size http://stackoverflow.com/questions/29592816/pic-18f8722-variable-declaration-adresses
GPR_VAR		    UDATA
setPointHigh	    RES	    1	    ; high byte of the setpoint
setpointLow	    RES	    1	    ; low byte of the setpoint
pressureHigh	    RES	    1	    ; high byte of the ADC reading
pressureLow	    RES	    1	    ; low byte of the ADC reading
state		    RES     1       ; State options: 0 = release, 1 = hold, 2 = inflate
countdown           RES     1       ; acquisition timer for the ADCs
tempIndex	    RES	    1	    ; Temporary index
halfBand	    RES	    1	    ; 1/2*How wide our bang-bang deadband should be
topBand		    RES	    1	    ; Top of our deadBand
bottomBand	    RES	    1	    ; Bottom of our deadBand
operationMode	    RES	    1	    ; What mode should we operate in? bangBang = 0. stateMachine = 1
	    
;*******************************************************************************
; loadFsr Macro
;*******************************************************************************
; loadFsr loads FSR1 w/ an I2C memory address.
; IMPORTANT: Don't use a "#define" value as argument to the macro. 
; TODO: Verify whether this is superstitiuos or useful
; Typical use: address = i2cArray, index = tempIndex
loadFsr macro 	address,index
    movlw 	address 		; load address
    addwf	index,W			; add the index value to determine location in array (store in w)
    movwf 	FSR1L			; load FSR1L with pointer info
    clrf	FSR1H
    endm

;*******************************************************************************
; Reset Vector
;*******************************************************************************

RES_VECT  CODE    0x0000            ; processor reset vector
    goto    start                   ; go to beginning of program
    
;*******************************************************************************
; Interrupt Service Routine
;*******************************************************************************

ISR    CODE    0x0004
    ; Check For flags
    movlw	0x02 ; 1+n_states
    banksel PIR1
    btfsc   PIR1,SSP1IF		; I/Nn p92. SSP interrupt
    movlw   0x00
    banksel PIR2
    btfsc   PIR2,BCL1IF		; I/Nn p92. Bus Collision
    movlw   0x01
    
    addwf    PCL
    goto	sspint
    goto	busCollision
    retfie
    
;*******************************************************************************
; MAIN PROGRAM
;*******************************************************************************

MAIN_PROG CODE                      ; let linker place main program

start
    PAGESEL initialize
    call    initialize
    goto loop

;*******************************************************************************
; INITIALIZATION
;
; Configure the clock
; Configure the ADC and I/O
; Initialize variables
; Configure the I2C module
;*******************************************************************************
initialize  
    ; Configure the oscillator
    banksel OSCCON
    movlw   B'11110000'		    ; 4xPLL enabeled, 32MHz, internal clock
    movwf   OSCCON
    
    ; Input (ADC) Configuration
    banksel TRISA
    bsf	    TRISA,2		    ; RA2 set as input p 120
    banksel ANSELA
    bsf	    ANSELA,2		    ; ANSA2 set as input
    banksel ADCON0
    bsf	    ADCON0,0		    ; enable the ADC module
    banksel ADCON1
    ; TODO: Next 2 Lines: practically speaking, what is difference and why was second line chosen?
    ;movlw   B'00110000'		    ; left justified, Frc, Vss, Vdd
    movlw   B'01100000'		    ; left justified, Fosc/64, Vss, Vdd
    movwf   ADCON1
    
    ; MOSFET Configurations
    banksel TRISC     ; RC4
    bcf     TRISC,4
    banksel LATC
    bcf     LATC,4
    banksel TRISC    ; RC5
    bcf     TRISC,5
    banksel LATC
    bcf     LATC,5
    
    ; Initialize band, state, operation mode
    movlw   .10
    banksel halfBand
    movwf   halfBand
    movlw   .0
    banksel state
    movwf   state
    movlw   INDEX_OPERATION_MODE
    movwf   tempIndex
    loadFsr i2cArray,tempIndex	; call loadFsr macro
    movlw   .0 ; Default operation mode is picBangBang
    movwf   INDF1			; load INDF1 with data to write
    
    ; uC set up
    ;banksel TRISC ; Don't change RC3 or RC4 to inputs. We use those, although this is "not recommended".
    ;bsf     TRISC,3	; Set RC3 to input (must be done) ; Dyl SS
    ;bsf     TRISC,4	; Set RC4 to input (must be done) ; Dyl EUSART Tx. RA4 is SDO
    clrf	INDEX_I2C               ; Clear the index pointer
    clrf	BYTENUM                  
    bsf		BYTENUM,0			; set bit 0 in BYTENUM
    banksel SSP1STAT
    bsf	    SSP1STAT,SMP	; Slew rate control disabled for standard speed mode ;dyl 100 khz
    movlw   b'00110110'		; Enable serial port, I2C slave mode, 7-bit address
    movwf   SSP1CON1
    bcf	    SSP1CON2,SEN	; disable clock stretching ;Dyl was bsf. 
    ; Clock stretching helps with troubleshooting. For some reason I disabled it. TODO: Decide whether clock stretching is usefull
    ;    bsf	    SSP1CON2,GCEN	; global call enable TODO: General call -> all sensors charge
    bsf	    SSP1CON3,BOEN	; SSPBUF is updated and NACK is generated
				; for a received address/data byte,
				; ignoring the state of SSPOV bit only if BF bit = 0.
    bsf	    SSP1CON3,SDAHT	; Minimum of 300 ns hold time
    movlw   I2C_ADDRESS		; load the slave address
    movwf   SSP1ADD
    ; I2C Setup
    banksel PIR1
    bcf	    PIR1,SSP1IF	; clear the SSP interrupt flag
    banksel PIE1
    bsf	    PIE1,SSP1IE	; enable SSP interrupts
    banksel PIR2
    bcf	    PIR2,BCL1IF	; clear the SSP interrupt flag
    banksel PIE2
    bsf	    PIE2,BCL1IE	; enable SSP interrupts
    bsf	    INTCON,PEIE	; enable peripheral interrupts
    bsf	    INTCON,GIE	; enable global interrupts
    return
    
;*******************************************************************************
; SUBROUTINES
;*******************************************************************************
loop
    ; 1. Retrieve our setPoint bytes from i2cArray
    movlw   INDEX_DESIRED_PRESSURE_HIGH
    movwf   tempIndex
    loadFsr i2cArray,tempIndex	; call loadFsr macro
    movf    INDF1,W		; move value into W to load to SSP buffer
    banksel setPointHigh
    movwf   setPointHigh
    movwf   topBand ; Temporary
    movwf   bottomBand ; Temporary
    incf    tempIndex,F		; increment INDEX_I2C 'pointer'
    loadFsr i2cArray,tempIndex	; call loadFsr macro
    movf    INDF1,W		; move value into W to load to SSP buffer
    banksel setpointLow
    movwf   setpointLow
    
    ; 2. Retrieve halfBand from memory
    movlw   INDEX_HALF_BAND
    movwf   tempIndex
    loadFsr i2cArray,tempIndex	; call loadFsr macro
    movf    INDF1,W		; move value into W to load to SSP buffer
    banksel halfBand
    movwf   halfBand
    addwf   topBand,F ; topBand = setPointHigh + halfBand
    subwf   bottomBand,F ; bottomBand = setPointHigh - halfBand
    
;   3. Read the pressure (AN2 is ADC for port RA2)
    movlw   B'00001001' ; x unimp, xxxxx channel select,, x status, x adc enable
    banksel ADCON0
    movwf   ADCON0
    call    sampledelay
    banksel ADCON0
    bsf     ADCON0,1                  ; The ADC Starts on this line
    btfsc   ADCON0,1 ; wait until done
    goto    $-1
    banksel ADRESH
    MOVF    ADRESH,W
    banksel pressureHigh
    movwf   pressureHigh
    banksel ADRESL
    MOVF    ADRESL,W
    banksel pressureLow
    movwf   pressureLow
    movlw   B'11000000' ; Only keep 2 MSB's as the low byte
    andwf   pressureLow,1
    
    ; 4. Write pressure reading to memory
    movlw   INDEX_PRESSURE_HIGH
    movwf   tempIndex
    loadFsr i2cArray,tempIndex	; call loadFsr macro
    movfw   pressureHigh
    movwf   INDF1			; load INDF1 with data to write
    movlw   INDEX_PRESSURE_LOW
    movwf   tempIndex
    loadFsr i2cArray,tempIndex	; call loadFsr macro
    movfw   pressureLow
    movwf   INDF1
    
    ; 5. Retrieve Operation Mode from memory
    movlw   INDEX_OPERATION_MODE
    movwf   tempIndex
    loadFsr i2cArray,tempIndex	; call loadFsr macro
    movf    INDF1,W		; move value into W to load to SSP buffer
    banksel operationMode
    movwf   operationMode    
    
    ; 6. Call appropriate function (by adding a value to program counter)
    movlw B'00000001' ; Only acceptible states are 0,1
    andwf operationMode,W
    addwf   PCL
    goto picBangBang
    goto masterDirectControl

;**************************************
; Allow the I2C master to directly control the valve states
;**************************************
masterDirectControl
    ; Compute the new command by reading state from i2cArray
    movlw   INDEX_STATE
    movwf   tempIndex
    loadFsr i2cArray,tempIndex; call loadFsr macro
    movfw   INDF1		; move value into W to load to SSP buffer
    banksel state
    movwf   state
    
    ; Set valves according to state (max acceptible state is 2)
    movlw B'00000011'
    banksel state
    andwf state,W
    addwf   PCL
    goto release
    goto hold
    goto inflate
    goto loop ; Do nothing if state % 4 = 3 (above operations equivalent to "modulo 4")

;**************************************
; Pic implements a 3-state bang-bang controller
;**************************************
picBangBang
    ; 3. Set valves according to state (max acceptible state is 2)
    movlw B'00000011'
    banksel state
    andwf state,W
    addwf   PCL
    goto currentlyReleasing
    goto currentlyHolding
    goto currentlyInflating

; 3 states of the State Machine
currentlyReleasing
    ; 3.1 Compare pressure to set point. Should we hold?
    banksel setPointHigh
    movfw   setPointHigh
    subwf   pressureHigh,W ; pressure minus setPoint
    
    ; 4.1 Hold if p-sp < 0 (we crossed set point)
    banksel LATC ; LATC arbitrary
    btfss   STATUS,C       ; Check the carry bit
    goto hold
    
    goto release ; Else, Don't switch states

currentlyHolding
    ; 3.1 Compare pressure to bottom of band. Should we inflate?
    banksel bottomBand
    movfw   bottomBand
    subwf   pressureHigh,W ; pressure minus bottom
    
    ; 4.1 Inflate if p-b < 0 (we crossed bottom)
    banksel LATC ; LATC arbitrary
    btfss   STATUS,C       ; Check the carry bit
    goto inflate

    ; 3.2 Compare pressure to top of band. Should we release?
    banksel pressureHigh
    movfw   pressureHigh
    subwf   topBand,W ; top minus pressure
    
    ; 4.2 Release if t-p < 0 (we crossed top)
    banksel LATC ; LATC arbitrary
    btfss   STATUS,C       ; Check the carry bit
    goto release
    
    goto hold ; Don't switch states
    
currentlyInflating
    ; 3.1 Compare pressure to set point. Should we hold?
    banksel pressureHigh
    movfw   pressureHigh
    subwf   setPointHigh,W ; set point minus bottom
    
    ; 4.1 Hold if sp-p < 0 (we crossed set point)
    banksel LATC ; LATC arbitrary
    btfss   STATUS,C       ; Check the carry bit
    goto hold
    
    goto inflate ; Don't switch states

;**************************************
; Valve Functions    SOL 1 Release RC4, SOL 2 Inflate RC5
;**************************************
release
    ; Update state
    movlw .0
    banksel state
    movwf state
    ; Control Valves
    banksel LATC
    bsf LATC,LATC4
    bcf LATC,LATC5
    goto loop
hold
    ; Update state
    movlw .1
    banksel state
    movwf state
    ; Control Valves
    banksel LATC
    bcf LATC,LATC4
    bcf LATC,LATC5
    goto loop
    
inflate
    ; Update state
    movlw .2
    banksel state
    movwf state
    ; Control Valves
    banksel LATC
    bcf LATC,LATC4
    bsf LATC,LATC5
    goto loop
    
    
    ; Wait for a designated time to allow ADC to stabilize (necessary?)
sampledelay
    movlw   0x20
    banksel countdown
    movwf   countdown
    DECFSZ  countdown,1
    goto   $-1
    return
    
;**************************************
; I2C here onward
;**************************************

sspint
    banksel SSP1STAT
    btfsc   SSP1STAT,2		; R/Wn p276; is it a master read:
    goto    read		; yes
    goto    write		; no

read
    banksel SSP1STAT
    btfss   SSP1STAT,5		; D/An p196; was last byte an address or data?
    goto    read_PreviousByteWasAddress		; clear -> address
    goto    read_PreviousByteWasData		; set -> data

read_PreviousByteWasAddress
    loadFsr i2cArray,INDEX_I2C
    movf    INDF1,W		; move value into W to load to SSP buffer
    banksel SSP1BUF
    movwf   SSP1BUF		; load SSP buffer
    incf    INDEX_I2C,F		; increment INDEX_I2C 'pointer'
    banksel SSP1CON1
    btfsc   SSP1CON1,WCOL	; did a write collision occur?
    goto    writeCollision		; yes -> clear bit
    banksel SSP1CON1
    bsf	    SSP1CON1,CKP	; release clock stretch
    banksel PIR1
    bcf	    PIR1,SSP1IF		; clear the SSP interrupt flag
    goto    exit 		; return from interrupt

read_PreviousByteWasData
    loadFsr i2cArray,INDEX_I2C	; call loadFsr macro
    movf    INDF1,W		; move value into W to load to SSP buffer
    banksel SSP1BUF
    movwf   SSP1BUF		; load SSP buffer
    incf    INDEX_I2C,F		; increment INDEX_I2C 'pointer'
    banksel SSP1CON1
    btfsc   SSP1CON1,WCOL	; did a write collision occur?
    goto    writeCollision		; yes -> clear bit
    banksel SSP1CON1
    bsf	    SSP1CON1,CKP	; release clock stretch
    banksel PIR1
    bcf	    PIR1,SSP1IF		; clear the SSP interrupt flag
    goto    exit 		; return from interrupt


write
    banksel SSP1STAT
    btfss   SSP1STAT,5		; was last byte an address or data?
    goto    write_PreviousByteWasAddress		; if clear, it was an address
    goto    write_PreviousByteWasData		; if set, it was data

write_PreviousByteWasAddress ; NOTE: BYTENUM is used as a flag for first byte in the "Write mode"
    bsf	    BYTENUM,0		; set bit 0 of BYTENUM
    banksel SSP1BUF
    movfw   SSP1BUF		; move the contents of the buffer into W
				; dummy read to clear the BF bit
    banksel SSP1CON1
    bsf	    SSP1CON1,CKP	; release clock stretch
    banksel PIR1
    bcf	    PIR1,SSP1IF		; clear the SSP interrupt flag
    goto    exit 		; Go to exit to return from interrupt

write_PreviousByteWasData ; NOTE: BYTENUM is used as a flag for first byte in the "Write mode"
    btfss   BYTENUM,0		; is bit 0 set?
    goto    writeSsp1BufferToMemory		; if clear, go write data
    banksel SSP1BUF		; if set, first data byte is location to write to
    movfw   SSP1BUF
    movwf   INDEX_I2C		; load INDEX_I2C with data address to write to
    clrf    BYTENUM		; clear the BYTENUM register
    banksel SSPCON1
    btfsc   SSP1CON1,WCOL	; did a write collision occur?
    goto    writeCollision		; if so, go clear bit
    bsf	    SSP1CON1,CKP	; release clock stretch
    banksel PIR1
    bcf	    PIR1,SSP1IF		; clear the SSP interrupt flag
    goto    exit 		; Go to exit to return from interrupt

writeSsp1BufferToMemory
    clrf    W                   ; clear W
    movlw   RX_ELEMENTS		; load array elements value
    banksel STATUS
    btfsc   STATUS,Z		; is Z clear?
    subwf   INDEX_I2C,W		; if Z = 1, subtract index from number of elements
    banksel STATUS
    btfsc   STATUS,0		; did a carry occur after subtraction?
    goto    noMemOverwrite	; if so, Master is trying to write to many bytes
    loadFsr i2cArray,INDEX_I2C	; call loadFsr macro
    banksel SSP1BUF
    movfw   SSP1BUF		; move the contents of the buffer into W
    movwf   INDF1		; load INDF1 with data to write
    incf    INDEX_I2C,F		; increment 'pointer' to next memory location
    banksel SSPCON1
    btfsc   SSP1CON1,WCOL	; did a write collision occur?
    goto    writeCollision		; if so, go clear bit
    banksel SSP1CON1
    bsf	    SSP1CON1,CKP	; release clock stretch
    banksel PIR1
    bcf	    PIR1,SSP1IF		; clear the SSP interrupt flag
    goto    exit 		; return from interrupt

noMemOverwrite
    banksel SSP1BUF
    movfw   SSP1BUF		; move SSP buffer to W
                    		; clear buffer so no overwrite occurs
    banksel SSP1CON1
    bsf	    SSP1CON1,CKP	; release clock stretch
    banksel PIR1
    bcf	    PIR1,SSP1IF		; clear the SSP interrupt flag
    goto    exit 		; return from interrupt

writeCollision
    banksel SSP1CON1		
    bcf	    SSP1CON1,WCOL	; clear WCOL bit
    banksel SSP1BUF		
    movfw   SSP1BUF		; move SSP buffer to W
                   		; clear SSP buffer
    banksel SSP1CON1		
    bsf	    SSP1CON1,CKP	; release clock stretch
    banksel PIR1
    bcf	    PIR1,SSP1IF		; clear the SSP interrupt flag
    goto    exit 		; return from interrupt

busCollision
    banksel SSP1BUF		
    clrf    SSP1BUF		; clear the SSP buffer
    banksel PIR2
    bcf	    PIR2,BCL1IF		; clear the SSP interrupt flag
    banksel SSP1CON1		
    bsf	    SSP1CON1,CKP	; release clock stretch
    goto    exit		; return from interrupt

exit
        retfie			; Return from interrupt.

end				; end OF PROGRAM