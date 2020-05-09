
/*******************************************************************************
PIC16F1937 
I2C SLAVE DRIVER CODE 
Author: Chris Best 
Microchip Technologies
DATE: 07/03/2013
--------------------------------------------------------------------------------
 Software License Agreement

 The software supplied herewith by Microchip Technology Incorporated
 (the 'Company') is intended and supplied to you, the Company's customer,
 for use solely and exclusively with products manufactured by the Company.

 The software is owned by the Company and/or its supplier, and is protected
 under applicable copyright laws. All rights are reserved. Any use in
 violation of the foregoing restrictions may subject the user to criminal
 sanctions under applicable laws, as well as to civil liability for the breach
 of the terms and conditions of this license.

 THIS SOFTWARE IS PROVIDED IN AN 'AS IS' CONDITION. NO WARRANTIES,
 WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT NOT LIMITED TO,
 IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 APPLY TO THIS SOFTWARE. THE COMPANY SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE
 FOR SPECIAL, INCIDENTAL OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

 INTEGRATED DEVELOPMENT ENVIRONMENT: MPLAB X IDE v1.80
 PROGRAMMER/DEBUGGER: PICKIT 3
 LANGUAGE TOOLSUITE: MICROCHIP XC8 v1.20

 NOTES:
 This code was developed for use with the PIC16F1937, but can be used with 
 other enhanced mid-range devices. Please check the specific part's data sheet
 for register names, such as SSPBUF, since some parts may have multiple I2C
 modules, or they may use a slightly different name (SSP1BUF). Also, the
 CONFIG words may need to be adjusted.

 CODE FUNCTION:
 The code implements the MSSP (or SSP) module as an I2C slave.
 This will allow the user to read from, or write to, a data array 
 (32 bytes long). There is also a 'fail-safe' built in to protect from writing
 data into critical memory locations. It is important to keep in mind that this
 code is for demonstration of the MSSP module for slave I2C communications.
 It does not include other interrupt possibilities, which would need to be
 added, and may require this code to be modified. Also, for simplicity, the
 initialization of the basic necessary registers are done in this C file.
 This can also be done in a separate header (.h) file. Having a separate .h
 file can make it easy to re-use the code in multiple applications.
 The code is written to work directly with the
 'I2C MASTER CODE VERSION 2' and can be used either the assembly or C versions
 since they both do the same thing.

------------------------------------------------------------------------------------------------
/****************************************************************/
#include <pic.h> 
#include <xc.h>

//__PROG_CONFIG (1,0x3FDC);
//__PROG_CONFIG (2,0x3FFF);

// CONFIG1
#pragma config FOSC = INTOSC    // INTOSC oscillator: I/O function on CLKIN pin
#pragma config WDTE = ON        // Watchdog Timer Enable (WDT enabled)
#pragma config PWRTE = ON       // Power-up Timer Enable (PWRT enabled)
#pragma config MCLRE = ON       // MCLR/VPP pin function is MCLR
#pragma config CP = ON          // Flash Program Memory Code Protection enabled
#pragma config CPD = ON         // Data memory code protection is enabled
#pragma config BOREN = ON       // Brown-out Reset enabled
#pragma config CLKOUTEN = OFF   // CLKOUT function is disabled.
                                // I/O or oscillator function on the CLKOUT pin
#pragma config IESO = ON        // Internal/External Switch-over mode is enabled
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor is enabled

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection off
#pragma config VCAPEN = OFF     // All VCAP pin functionality is disabled
#pragma config PLLEN = ON       // PLL Enable (4x PLL enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow will cause a Reset
#pragma config BORV = LO        // Brown-out Reset Voltage (Vbor),
                                // low trip point selected.
#pragma config LVP = ON         // Low-voltage programming enabled

#define RX_ELMNTS	32
#define I2C_slave_address 0x30

// array for master to read from
volatile unsigned char I2C_Array_Tx[RX_ELMNTS] =
{0xAA,0xEE,0xDD,0xCC,0xBB,0xAA,0x99,0x88,
0x77,0x66,0x55,0x44,0x33,0x22,0x11,0xFA,
0xEA,0xDA,0xCA,0xBA,0xFB,0xFC,0xFD,0xFE,
0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08};

// array for master to write to
volatile unsigned char I2C_Array_Rx[RX_ELMNTS] = 0;	

unsigned char index_i2c = 0;     // index pointer
unsigned char junk = 0;          // used to place unnecessary data
unsigned char clear = 0x00;      // used to clear array location once
                                 // it has been read

void initialize(void);           // initialize routine

/**************************** MAIN ROUTINE ************************************/
void main(void) 
{ 
    initialize();               // call initialize routine
    while(1)                    // main while() loop
    {                           // program will wait here for ISR to be called
        asm("CLRWDT");          // clear WDT
    }
}// end main
/******************************************************************************/

/**************************** INITIALIZE ROUTINE ******************************/
void initialize(void)
{
//uC SET UP
    OSCCON = 0b01111010; 		// Internal OSC @ 16MHz
    OPTION_REG = 0b11010111;    // WPU disabled, INT on rising edge, FOSC/4
                                // Prescaler assigned to TMR0, rate @ 1:256
    WDTCON = 0b00010111;        // Prescaler 1:65536
                                // period is 2 sec (RESET value)
    PORTC = 0x00;               // Clear PORTC
    LATC = 0x00;                // Clear PORTC latches
    TRISC = 0b00011000;         // Set RC3, RC4 as inputs for I2C
//I2C SLAVE MODULE SET UP 
    SSPSTAT = 0b10000000;       // Slew rate control disabled for standard
                                // speed mode (100 kHz and 1 MHz)
    SSPCON1 = 0b00110110; 		// Enable serial port, I2C slave mode, 
                                // 7-bit address
    SSPCON2bits.SEN = 1;        // Clock stretching is enabled
    SSPCON3bits.BOEN = 1;       // SSPBUF is updated and NACK is generated
                                // for a received address/data byte,
                                // ignoring the state of the SSPOV bit
                                // only if the BF bit = 0
    SSPCON3bits.SDAHT = 1;		// Minimum of 300 ns hold time on SDA after
                                // the falling edge of SCL
    SSPCON3bits.SBCDE = 1;		// Enable slave bus collision detect interrupts
    SSPADD = I2C_slave_address; // Load the slave address
    SSPIF = 0;                  // Clear the serial port interrupt flag
    BCLIF = 0;                  // Clear the bus collision interrupt flag
    BCLIE = 1;                  // Enable bus collision interrupts
    SSPIE = 1;                  // Enable serial port interrupts
    PEIE = 1;                   // Enable peripheral interrupts
    GIE = 1; 					// Enable global interrupts
}//end initialize
/******************************************************************************/

/*************************** ISR ROUTINE **************************************/
void interrupt ISR(void)
{
    if(SSPIF)                               // check to see if SSP interrupt
    {
        if(SSPSTATbits.R_nW)                // Master read (R_nW = 1)
        {
            if(!SSPSTATbits.D_nA)         // last byte was an address (D_nA = 0)
            {
                junk = SSPBUF;                  // dummy read to clear BF bit
                index_i2c = 0;                     // clear index pointer
                if(index_i2c < RX_ELMNTS)
                {               // Does data exceed number of allocated bytes?
                    SSPBUF = I2C_Array_Tx[index_i2c];   // load SSPBUF with data
                    I2C_Array_Tx[index_i2c++] = clear;  // clear that location
                }                                       // and increment index
                else                       // trying to exceed array size
                {
                    junk = SSPBUF;          // dummy read to clear BF bit
                }
                SSPCON1bits.CKP = 1;        // release CLK
            }
            if(SSPSTATbits.D_nA)       // last byte was data
            {
                if (index_i2c < RX_ELMNTS)
                {               // Does data exceed number of allocated bytes?
                    SSPBUF = I2C_Array_Tx[index_i2c];   // load SSPBUF with data
                    I2C_Array_Tx[index_i2c++] = clear;  // clear that location
                }                                       // and increment index
                else
                {
                    junk = SSPBUF;      // dummy read to clear BF bit
                }
                SSPCON1bits.CKP = 1;    // release CLK
            }
        }
        
        if(!SSPSTATbits.R_nW)			// master write (R_nW = 0)
        {
            if(!SSPSTATbits.D_nA)        // last byte was an address (D_nA = 0)
            {
                junk = SSPBUF;			// read buffer to clear BF
				SSPCON1bits.CKP = 1;            // release CLK
            }
            if(SSPSTATbits.D_nA)                // last byte was data (D_nA = 1)
            {
            	if (index_i2c < RX_ELMNTS)	
                {               // Does data exceed number of allocated bytes?
                    I2C_Array_Rx[index_i2c++] = SSPBUF;
                }                                       // load array with data
                else
                {
                    index_i2c = 0;                   // reset the index pointer
                }
				if(SSPCON1bits.WCOL)		// Did a write collision occur?
				{
                    SSPCON1bits.WCOL = 0;       // clear WCOL bit
                    junk = SSPBUF;              // clear SSPBUF
				}
				SSPCON1bits.CKP = 1;    		// release CLK
            }
    	}
    }
    if(BCLIF)                       // Did a bus collision occur?
    {
        junk = SSPBUF;              // clear SSPBUF
		BCLIF = 0;                  // clear BCLIF
		SSPCON1bits.CKP = 1;        // Release CLK
    }
    SSPIF = 0;                      // clear SSPIF
}


