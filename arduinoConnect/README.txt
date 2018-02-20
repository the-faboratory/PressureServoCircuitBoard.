/***************************
* README.txt
*
* Explain files in the
* arduinoPython folder.
*
* Written by Jennifer Case
***************************/

The Arduino behaves as a master to the Python code
running with it. This is the basic outline for the
Arduino code. The control is done through Serial
communication.

Python is used to simply record the Arduino Serial
output. Python turns the Arduino loop function on
and off as an indication of when it is prepared to
record data. Note that Python is not needed and the
loop function can also be turned on and off through
serial monitor as well.

This code is merely a skeleton that allows this
recording ability. Users should enter their own code
appropriately in this skeleton.

This code currently has serial communication at
2000000 baud. This should be adjusted appropriately.


