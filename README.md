# Pneumatic Regulator git repository
This repository contains (hopefully) everything needed to replicate the pneumatic regulator presented in

J. W. Booth, J. C. Case, E. L. White, D. S. Shah, and R. Kramer-Bottiglio, “[An addressable pneumatic regulator for distributed control of soft robots](10.1109/ROBOSOFT.2018.8404892),” in 2018 IEEE International Conference on Soft Robotics (RoboSoft).

It will be called "pneumatic regulator" and "pressure regulator" interchangeably.

If you have comments, suggestions, or want additional information, including how to obtain a pre-programmed and pre-assembled pressure regulator, please contact Joran Booth at JoranBooth@gmail.com. Being a research lab affiliated with a non-profit university, we are unable to sell regulators through our Yale affiliations. However, we are exploring commercializing the boards out of a small garage operation, depending on interest levels and applicable laws.

## Getting Started

### Prerequisites

Download MPLAB IDE(http://www.microchip.com/mplab/mplab-x-ide), with the XC compilers.

A detailed programming guide, with notes on the PressureRegulator's state machine, is in repository_guide.docx.

There are two versions of the hardware, with different connectors: one with 0.1" headers, and one with a MOLEX connector.

### Installing / Using
Programming a device is as easy as 1,2,3,...4:
```
1. Open the PIC_I2C.X project in MPLAB.
2. Change the variable I2C_ADDRESS to contain 2x the address you want the Arduino to call your PIC. Example: Arduino talks to sensor at address 8. I2C_ADDRESS = 2x8 = 16
3. Click "clean and build" to make sure it is compiling properly for your computer. (If you get a compiler error, try opening project properties and use C standard 90 (Project properties -> XC8 Global Options -> C Standard))
4. Click "Make and Program Device Main Project", to download the program to your PIC using a PicIt3 (the 'red box').
```

## Testing The Firmware

For your first test, use the Arduino driver pneumatic_regulator_driver.ino, or the sample Arduino Wire library code. For your I2C pullup, use 3.3k or 4.7k.

## Explanation of files (see each file for details):

File/Directory |  Purpose |
----- | --------- |
arduino_driver | Folder containing sample Arduino driver for communicating with the pressure regulators
auxiliary_files | Additional documentation, and a Microchip I2C application note.
pcb_files | Eagle files, bill of materials, and gerber files (for PCB.ng) for both versions of the regulator: 0.1" headers, and MOLEX connector
Pic_I2C.X | Project folder for programming the pneumatic regulator's PIC microcontroller using MPLAB.
PicIt3_user_guide | [A guide to using the PICkit3](http://ww1.microchip.com/downloads/en/DeviceDoc/50002010B.pdf). Made by Microchip.
pressure_regulator_i2c.c | Firmware for the pressure regulator's PIC microcontroller, to control pressure manually or use Bang-Bang with a deadband to regulate the pressure at the output (i.e., in your actuator).
repository_guide.docx | An explanation of how to download firmware to a PIC, using this repository. 
