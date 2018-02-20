/*************************************
arduinoPython.ino

This program is meant to work with
a Python script that records what the
Arduino sends out.

Note that this sketch outlines how to
write future Arduino sketches. It needs
to wait for the Python to be ready
before sending messages, so it is
necessary to keep this skeleton code or 
data will be lost.

Written by Jennifer Case
Nov 6, 2017
*************************************/

#include<Wire.h>

// Declare global variables here
bool runLoop = false; // indicates if loop is running

void setup() {
  // put your setup code here
  Wire.begin();
  Serial.begin(2000000); // Serial speed can be changed
}

void loop() {
  if (runLoop) {
    // put your main code here to run repeatedly
    
    Serial.println("Send to python"); // change the message
  }
}

// serialEvent is a built in Arduino function that will check
// for incoming serial activity after every loop execution.
void serialEvent() {
  // Having something in the serial means that Python
  // is telling us to turn on or off the loop.
  char inChar = Serial.read(); // receive serial
  if (inChar == 'I') {Serial.println('a');}
  
  if (inChar == 'a') {
    if (runLoop) {runLoop = false;}
    else {runLoop = true;}
  }
}

