/*******************************************************************************  
This C file runs on the PCB "Pressure Regulator Rev. 0" for networked 
control of a pneumatic valve. Operation modes: automatic (0), manual (1).
A master can read the pressure data, which is a 10 bit adc reading of the pressure sensor.
Pic stores this as 16 bits. This means you should drop the low 6 bits of the data.
Pressure sensor calibration equation:
volts = 0.8*V_s/(P_max-P_min)*(P_applied - P_min) + 0.1*V_s
Power consumption: ~0.021A in "hold state" (valves off); each valve uses ~0.074 A

Typical parameters a user wants to edit are marked by **USER**

Communication:
    - To write memory, write [the memory address] followed by [the data].
    - To read memory, write [the memory address] and then read [the desired number of data bytes].


Automatic mode (3-state bang-bang), typical use case: 
    1. Arduino send INDEX_OPERATION_MODE (22), followed by an operation mode (0 for 
    automatic)//
    2. Arduino send INDEX_SETPOINT_HIGH_BYTE (16), followed by a desired pressure (0 through 255. 126 is atmospheric pressure). 
    3. Arduino send value of INDEX_PRESSURE_SENSOR_HIGH_BYTE (19). 41
    4. Arduino request 2 bytes. PIC sends two bytes of pressure data.

Manual mode (arduino directly controls state), typical use case: 
    1. Arduino send INDEX_OPERATION_MODE (22), followed by an operation mode (1 for 
    manual)//
    2. Arduino send INDEX_DESIRED_STATE (16), followed by a desired state (State options: 0 = release, 1 = hold, 2 = inflate)
    3. Arduino send value of INDEX_PRESSURE_SENSOR_HIGH_BYTE (19). 
    4. Arduino request 2 bytes. PIC sends two bytes of pressure data.

Circuit information
    Inputs
        RA2 - Pressure Feedback. Circuit diagram "PRESSURE"

    Outputs
        RC4 - release. Circuit diagram "SOL 1"
        RC5 - inflate. Circuit diagram "SOL 2"

    ICSP Programming
        RA0 - ICSPDAT = PGD
        RA1 - ICSPCLK = PGC
        RA3 - VPP

    I2C
        RC0 I2C_CLK = SCK
        RC1 I2C_DAT = SDI

/******************************************************************************/
#include <pic.h> 
#include <xc.h>

#include <pic16f1825.h> // Dyl define registers and aliases

// CONFIG1
#pragma config FOSC = INTOSC    // INTOSC oscillator: I/O function on CLKIN pin
#pragma config WDTE = ON        // Watchdog Timer Enable (WDT enabled) Dyl off
#pragma config PWRTE = ON       // Power-up Timer Enable (PWRT enabled) Dyl Off
#pragma config MCLRE = ON       // MCLR/VPP pin function is MCLR
#pragma config CP = ON          // Program memory code protection is enabled Dyl Off
#pragma config CPD = ON         // Data memory code protection is enabled Dyl off
#pragma config BOREN = ON       // Brown-out Reset enabled
#pragma config CLKOUTEN = OFF   // CLKOUT function is disabled. Dyl On
                                // I/O or oscillator function on the CLKOUT pin
#pragma config IESO = ON        // Internal/External Switch-over mode is enabled
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor is enabled

// CONFIG2
#pragma config WRT = OFF        // Flash Memory Self-Write Protection off
//#pragma config VCAPEN = OFF     // All VCAP pin functionality is disabled
#pragma config PLLEN = ON       // PLL Enable (4x PLL enabled)
#pragma config STVREN = ON      // Stack Overflow/Underflow will cause a Reset
#pragma config BORV = LO        // Brown-out Reset Voltage (Vbor)
                                // low trip point selected.
#pragma config LVP = ON         // Low-Voltage Programming enabled

#define RX_ELMNTS   32
unsigned char i2c_address =     4*2; // slave address (doubled) ** USER Should Modify **
unsigned int DELAY_MS = 30; // delay [ms]. This can be used to reduce chatter on the regulators * USER can modify *

volatile unsigned char I2C_Array[RX_ELMNTS] =   // array for master to write to
{0xFF, 0xEE, 0xDD, 0xCC, 0xBB, 0xAA, 0x99, 0x88,
0x77, 0x66, 0x55, 0x44, 0x33, 0x22, 0x11, 0xFA,
0xEA, 0xDA, 0xCA, 0xBA, 0xFB, 0xFC, 0xFD, 0xFE,
0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08};

unsigned int index_i2c = 0;            // used as an index pointer in array
unsigned char junk = 0;                // used to place unnecessary data
unsigned char first = 1;               // used to determine whether data address 
                                       // location or actual data


// Global algorithm variables
#define INDEX_SETPOINT_HIGH_BYTE    16 // High byte of our desired actuator set point.  Default setpoint is atmospheric pressure (2.5V -> HighByte = 127 = -0.147 PSI)
#define INDEX_SETPOINT_LOW_BYTE 17 // Low byte of our desired actuator power
#define INDEX_DESIRED_STATE    18 // State. 0 = Release, 1 = Hold, 2 = Inflate. Our desired state. This is only active during manual mode.  Default valve state is hold (both valves off).
#define INDEX_PRESSURE_SENSOR_HIGH_BYTE 19 // High byte of our actuator's pressure sensor
#define INDEX_PRESSURE_SENSOR_LOW_BYTE 20  // Low byte of our actuator's pressure sensor
#define INDEX_HALF_BAND 21 // 1/2*How wide our bang-bang deadband should be. Default Half-band is 10 [ADC values] = 2.94 [PSI]
#define INDEX_OPERATION_MODE 22 // What mode should we operate in? automatic = 0. manual = 1.  Default mode is manual
#define INDEX_ONLY_SEND_PRESSURE 23 // If == 0, then the i2cArray can be read. ElseIf == 1, every read results in reading pressure. Default read mode is "only send pressure"
#define Nop() {_asm nop _endasm} // Dyl define the NOP function for doing nothing https://www.microchip.com/forums/m156162.aspx
#define _XTAL_FREQ 32000000 // internal oscillator: 32 MHz
unsigned int setPointHigh;       // high byte of the setpoint
unsigned int setPointLow; // low byte of the setpoint
unsigned int pressureHigh;    // high byte of the ADC reading
unsigned int pressureLow; // low byte of the ADC reading
unsigned int state;      // State options: 0 = release, 1 = hold, 2 = inflate
unsigned int countdown;      // acquisition timer for the ADCs
unsigned int tempIndex;   // Temporary index
unsigned int halfBand;    // 1/2*How wide our bang-bang deadband should be
unsigned int topBand;     // Top of our deadBand
unsigned int bottomBand;  // Bottom of our deadBand
unsigned int operationMode;   // What mode should we operate in? bangBang = 0. stateMachine = 1
unsigned int onlySendPressureBool; // If == 0, then the i2cArray can be read. ElseIf == 1, every read results in reading pressure


// Function declarations (self-explanatory)
void initialize(void);
void manualMode(void);
void automaticMode(void);
void currentlyReleasing(void);
void currentlyHolding(void);
void currentlyInflating(void);
void release(void);
void hold(void);
void inflate();
void sampleDelay(void);

/*************************** MAIN ROUTINE *************************************/
void main(void) {
    // Initialize variables, then run infinite loop: read set point, read pressure, set valves
    initialize();               // call initialize routine
    while (1)                   // main while() loop
    {
        // Read set point, assign top and bottom bands
        setPointLow = I2C_Array[INDEX_SETPOINT_LOW_BYTE];
        setPointHigh = I2C_Array[INDEX_SETPOINT_HIGH_BYTE];
        halfBand = I2C_Array[INDEX_HALF_BAND];
        topBand = setPointHigh + halfBand;
        bottomBand = setPointHigh - halfBand;
        
        // Read the pressure
        ADCON0 = 0b00001001;
        sampleDelay(); // Wait for the ADC to do its reading. TODO: Remove this. Should be unnecessary.
        ADCON0bits.GO_nDONE = 1;
        while(ADCON0bits.GO_nDONE){
            // wait until done
        }
        pressureHigh = ADRESH;
        pressureLow = ADRESL;
        pressureLow = pressureLow & 0b11000000; // Only keep 2 MSB's as the low byte
        
        // Store in our array, to allow Arduino to read it
        I2C_Array[INDEX_PRESSURE_SENSOR_HIGH_BYTE] = pressureHigh;
        I2C_Array[INDEX_PRESSURE_SENSOR_LOW_BYTE] = pressureLow;
        
        
        // Do the appropriate control scheme (set valves)
        switch(I2C_Array[INDEX_OPERATION_MODE]){
            case 0:
                automaticMode();
                break;
            case 1:
                manualMode();
                break;

        }

        asm("CLRWDT"); // clear WDT
        __delay_ms(DELAY_MS); // delay the loop for "DELAY_MS" milliseconds
    }                           
} // end main
/******************************************************************************/

/*************************** INITIALIZE ROUTINE *******************************/
void initialize(void) {
    // initialize: Configures the clock, ADC, I/O, I2C module, and Initialize variables
    
    // Use comparitor 1
    // Capacitor voltage on C1IN+ (Pin 13) RA0
    // Vdd/3 on C12IN0- (Pin 12) RA1 Dylan VDD/21
    // 2Vdd/3 on C12IN2- (Pin 8) RC2 Dylan VDD*11/21
    // Capacitor charge (Pin 6) RC4
    // Charging indicator on (Pin 5) RC5

    // Filter measurement on AN2 (Pin 11) RA2

    // SPI Port:
    // SCK on (Pin 10) RC0
    // SDI on (Pin 9) RC1
    // SDO on (Pin 3) RA4 [APFCON0]
    // SS1 on (Pin 7) RC3 [APFCON0]
    // I2C Port: SDA (Pin 9) RC1 / SCL (Pin 10) RC0 Dyl


    // uC setup
    OSCCON = 0b01111010; // Internal OSC @ 16MHz Dyl: diff from asm
    OPTION_REG = 0b11010111; // WPU disabled, INT on rising edge, FOSC/4. TMR0 prescalar = 1:256
    WDTCON = 0b00010111; // prescaler 1:65536 -> period is 2 sec(RESET value)
    PORTC = 0x00; // Clear PORTC
    
    //I2C SLAVE MODULE SET UP
    SSPSTAT = 0b10000000; // Slew rate control disabled for standard speed mode (100 kHz)
    SSPCON1 = 0b00110110; // Enable serial port, I2C slave mode, 7-bit address
    SSPCON2bits.SEN = 1; // Disable clock stretching to prevent glitched PIC's from holding the line (ineffective)
    SSPCON3bits.BOEN = 1; // SSPBUF is updated and NACK is generated for a received address/data byte, ignoring the state of the SSPOV bit only if the BF bit = 0
    SSPCON3bits.SDAHT = 1; // Minimum of 300 ns hold time on SDA after the falling edge of SCL
    SSPCON3bits.SBCDE = 1; // Enable slave bus collision detect interrupts // Dyl: SSPCON3bits is an alias of SSP1CON3bits
    SSPADD = i2c_address; // Load the slave address
    SSP1IF = 0; // Clear the serial port interrupt flag
    BCL1IF = 0; // Clear the bus collision interrupt flag
    BCL1IE = 1; // Enable bus collision interrupts
    SSP1IE = 1; // Enable serial port interrupts
    PEIE = 1; // Enable peripheral interrupts
    GIE = 1; // Enable global interrupts

    // ADC Configuration
    TRISAbits.TRISA2 = 1; // RA2 set as input p 120
    ANSELAbits.ANSA2 = 1;
    ADCON0bits.ADON = 1; // enable the ADC module
    ADCON1 = 0b01100000; // left justified, Fosc/64, Vss, Vdd
    
    
    // MOSFET Configurations
    TRISCbits.TRISC4 = 0; // RC4
    LATCbits.LATC4 = 0;
    TRISCbits.TRISC5 = 0; // RC5
    LATCbits.LATC5 = 0;
    
    // Initialize values in i2cArray
    
    // Default Half-band is 10 = 2.94 [PSI]
    I2C_Array[INDEX_HALF_BAND] = 10;

    // Default mode is manual
    I2C_Array[INDEX_OPERATION_MODE] = 1;
    
    // Default valve state is hold (both valves off)
    I2C_Array[INDEX_DESIRED_STATE] = 1;
    state = 0;
    
    // Default read mode is "only send pressure"
    I2C_Array[INDEX_ONLY_SEND_PRESSURE] = 1;
    
    // Default setpoint is atmospheric pressure (2.5V -> HighByte = 127 )
    I2C_Array[INDEX_SETPOINT_HIGH_BYTE] = 127;
} //end initialize
/******************************************************************************/


/*************************** I2C master directly controls the valves *****************************/
void manualMode(void){
    // automaticMode: Set valves according to state in I2C_Array (max acceptible state is 2)
    switch(I2C_Array[INDEX_DESIRED_STATE]){
        case 0:
            release();
            break;
        case 1:
            hold();
            break;
        case 2:
            inflate();
            break;
//        default: // TODO: implement default action as nop
//            Nop();
    }
}
/*************************** Bang-Bang control the valves *****************************/
void automaticMode(void) {
    // automaticMode: Set valves according to state (max acceptable state is 2)
    switch(state){
        case 0:
            currentlyReleasing();
            break;
        case 1:
            currentlyHolding();
            break;
        case 2:
            currentlyInflating();
            break;
    }
}
    
// 3 states of the State Machine
void currentlyReleasing(void) {
    // currentlyReleasing: release unless we just crossed the set point
    if (pressureHigh < setPointHigh){
        hold();
    }
    else{
        release();
    }
}

void currentlyHolding(void) {
    // currentlyHolding: chooses valve states given that we are currently holding
    if (pressureHigh > topBand){
        release();
    }
    else if (pressureHigh < bottomBand){
        inflate();
    }
    else{
        hold();
    }
}
 
void currentlyInflating(void) {
    // currentlyInflating: Inflate unless we just crossed the set point
    if (pressureHigh > setPointHigh){
        hold();
    }
    else{
        inflate();
    }
}   

//**************************************
// Valve Functions   - SOL 1 Release RC4, SOL 2 Inflate RC5
//**************************************
void release(void){
    state = 0;
    LATCbits.LATC4 = 1;
    LATCbits.LATC5 = 0;
}

void hold(void){
    state = 1;
    LATCbits.LATC4 = 0;
    LATCbits.LATC5 = 0;
}
    
void inflate(){
    state = 2;
    LATCbits.LATC4 = 0;
    LATCbits.LATC5 = 1;
}
        
void sampleDelay(void){
    // sampleDelay: wait for a designated time to allow ADC to stabilize (necessary?)
    countdown = 0x20;
    while(countdown--){
        // Wait for countdown to equal zero
    }
}

/****************************** ISR ROUTINE ***********************************/
// TODO: Update ISR. 
void interrupt ISR(void) {
    // ISR: Deals with interrupts. Currently this is just the I2C (MSSP) read and write interrupts, plus the bus collision interrupt.
    if (SSP1IF)                              // check to see if SSP interrupt
    {
        if (SSPSTATbits.R_nW)               // Master read (R_nW = 1)
        {
            if (!SSPSTATbits.D_nA)        // Last byte was an address (D_nA = 0) TODO: Replace with else
            {
                if(I2C_Array[INDEX_ONLY_SEND_PRESSURE]){
                    index_i2c = INDEX_PRESSURE_SENSOR_HIGH_BYTE; // Always send the timing data
                }
                
                SSPBUF = I2C_Array[index_i2c++]; // load with value from array
                SSPCON1bits.CKP = 1;             // Release CLK
            }
            if (SSPSTATbits.D_nA)               // Last byte was data (D_nA = 1) 
            {
                SSPBUF = I2C_Array[index_i2c++]; // load with value from array
                SSPCON1bits.CKP = 1;             // Release CLK
            }

        }
        if (!SSPSTATbits.R_nW) // Master write (R_nW = 0) TODO: Replace with else
        {
            if (!SSPSTATbits.D_nA) // Last byte was an address (D_nA = 0)
            {
                first = 1; //last byte was address, next will be data location
                junk = SSPBUF; // read buffer to clear BF
                SSPCON1bits.CKP = 1; // Release CLK
            }
            if (SSPSTATbits.D_nA) // Last byte was data (D_nA = 1)  TODO: Replace with else
            {
                if (first) // If this is the first byte of data, assume that it contains the desired index in our array
                {
                    index_i2c = SSPBUF; // load index with array location
                    first = 0; // clear this since we have location to read from/write to
                }
                else
                {
                    if (index_i2c < RX_ELMNTS) // make sure index is not
                    { //out of range of array
                        I2C_Array[index_i2c++] = SSPBUF; //load array with data
                    } 
                    else
                    {
                        junk = SSPBUF; //array location not valid, discard data
                    }
                }
                if (SSPCON1bits.WCOL) // Did a write collision occur?
                {
                    SSPCON1bits.WCOL = 0; //  clear WCOL
                    junk = SSPBUF; // dummy read to clear BF bit
                }
                SSPCON1bits.CKP = 1; // Release CLK
            }
        }
    }
    if (BCL1IF) // If a bus collision occurred, clear buffer and prepare for next interrupt event
    {
        junk = SSPBUF; // dummy read SSPBUF to clear BF bit
        BCL1IF = 0; // clear bus collision Int Flag bit
        SSPCON1bits.CKP = 1; // Release CLK
    }
    SSP1IF = 0; // clear SSP1IF flag bit
}
