/*************************************
  cap_sensor_driver.ino

  This driver is for testing pneumatic regulator(s). Assumes your PIC firmware will send, in sequence, the memory located at [the most recent sent index] upon a read request. (All commonly-used firmware as of 2019-10-29)
  Writes state to and reads pressure from a pressure regulator.
  Typical parameters a user wants to edit are marked by **USER**

  TODO:
     1. (Low priority) Verify that the Arduino won't send more than 2 bytes to the pressure regulator. That could lead to a data race (overwrite pressure reading on PIC)

  Written by Dylan Shah. Last edited 2019-10-29 YMD.
*************************************/


// ***** 1. Import libraries *****
#include <Wire.h>


// ***** 2. Initialize Variables *****
// 2.1. System configuration variables
const int NUM_ACTUATORS = 2; // **USER**
int regulator_addresses[NUM_ACTUATORS] = {4, 29}; // **USER**

// 2.2. Indices for the PIC's internal memory (PIC has a chunk of program memory reserved for storing variables that external devices are allowed to modify.)
const int INDEX_SETPOINT_HIGH_BYTE = 16; // Set point (stored unit = "bits of ADC readings"). Default set point is atmospheric pressure (-0.147 [PSI])
const int INDEX_DESIRED_STATE = 18; // Our desired state. This is only active during manual mode. State. 0 = Release, 1 = Hold (default, both valves off), 2 = Inflate.
const int INDEX_PRESSURE_SENSOR_HIGH_BYTE = 19; // High byte of the pressure sensor
const int INDEX_HALF_BAND = 21; // 1/2*How wide our bang-bang deadband should be. Default Half-band is 10 [ADC values] = 2.94 [PSI]
const int INDEX_OPERATION_MODE = 22; // What mode should we operate in? 0 = Automatic. 1 = manual (default)
const int INDEX_ONLY_SEND_PRESSURE = 23; // If == 0, then the i2cArray can be read. ElseIf == 1, every read results in reading pressure (default).

// 2.3. General variables
const int nBytes = 2; // How many bytes to read
float values[NUM_ACTUATORS]; // Array of received data
float filtered_values[NUM_ACTUATORS]; // Array of filtered sensor data
int iteration_state = 0; // State in the iteration (used for cycling between commands)
bool change_state = false; // Should we change state?
unsigned long current_millis = 0.0; // The current program time
unsigned long command_delay = 1000; // How long to delay between commands [ms]
unsigned long cycle_delay = 10; // How long to delay between cycles [ms]

// 2) Pressure Regulator Variables
const int AUTOMATIC = 0;
const int MANUAL = 1;
int control_mode = AUTOMATIC; // **USER**
bool only_send_pressure = 0; // **USER**
float command = 0; // FLOAT. Was unsigned long
unsigned long command_to_send = 0; // Use unsigned int to allow bitshift operator
const int halfBand = 3; // **USER** 1/2 * how wide our bang-bang deadband should be
float expConst = .9; // **USER** Decay constant for the optional exponential filter


// ***** 3. Set up Arduino *****
void setup() {
  Wire.begin();        // join i2c bus (address optional for master)
  Serial.begin(2000000);  // start serial for output
  // Wire.setClock(400000); // Change I2C speed to 4kHz
  
  current_millis = millis(); // Initial time
  for (int actuator = 0; actuator < NUM_ACTUATORS; actuator++) {
    write_i2c(regulator_addresses[actuator], INDEX_OPERATION_MODE, control_mode); // Send pic firmware mode to the actuator
    write_i2c(regulator_addresses[actuator], INDEX_HALF_BAND, halfBand); // Send inHalfBand to the actuator
    write_i2c(regulator_addresses[actuator], INDEX_ONLY_SEND_PRESSURE, only_send_pressure); // Send "don't only send pressure" to the actuator
  }
}



// ***** 4. Change pressure, read sensors and send readings to PC over Serial, forever *****
void loop() {

  // Periodically, change command
  if (millis() > current_millis + command_delay) {
    current_millis = millis();
    iteration_state++;
    change_state = true;
  }

  // Write to and read from regulators
  for (int actuator = 0; actuator < NUM_ACTUATORS; actuator++) {
    read_pressure_regulator(regulator_addresses[actuator], filtered_values[actuator], expConst, values[actuator]); // Read regulator's pressure

    // Change command if it is a "command cycle"
    if (change_state) {
      // Send a set point to the actuator, if it is in automatic mode
      if (control_mode == AUTOMATIC) {
        command = 4 * (iteration_state % 3)+4; // Command to send [psi]
        float command_to_volt = psi_to_volt(command); // Convert to volts
        command_to_send = volt_to_ten_bit(command_to_volt) >> 2; // Convert to bits, and only send high byte (high 8 bits)
        write_i2c(regulator_addresses[actuator], INDEX_SETPOINT_HIGH_BYTE, command_to_send); // Send command to the actuator

        // Print to serial, for debugging
        Serial.print(command_to_volt);
        Serial.print("\t");
        Serial.print(command_to_send);
        Serial.print("\n");
      }

      // Send a desired state, if in manual mode
      if (control_mode == MANUAL) {
        command = 2 - iteration_state % 3; // Only valid states are 0,1,2. PIC will also 'modulo' on its own.
        write_i2c(regulator_addresses[actuator], INDEX_DESIRED_STATE, command); // Send command to the actuator
      }
    }
  }

  finishCycle(); // Print to serial, and delay for 'cycle_delay' [ms]
}



// ***** 5. Helper Functions *****
void finishCycle() {
  // Print over Serial, clear 'change state' flag and delay for 'cycle_millis' [ms]
  Serial.print(command);
  Serial.print("\t");
  Serial.println("");
  for (int actuator = 0; actuator < NUM_ACTUATORS; actuator++) {
    Serial.print(regulator_addresses[actuator]); Serial.print("\t");
    Serial.print(values[actuator]); Serial.print("\t");
  }
  change_state = false;
  delay(cycle_delay);
}


// 5.1. Communicate with Pressure Regulator
void read_pressure_regulator(const int &in_address, float &in_filtered, const float &in_exp_constant, float &in_reading) {
  // Read the pressure regulator's air-pressure sensor, and filter it.

  // If we are not in "only_send_pressure" mode, then tell the PIC which address to read from.
  if (!only_send_pressure) {
    Wire.beginTransmission(in_address); // Prepare internal variables
    Wire.write(INDEX_PRESSURE_SENSOR_HIGH_BYTE); // Write memory location to buffer
    Wire.endTransmission(); // Transmit
  }

  // Request and process the pressure sensor's reading
  Wire.requestFrom(in_address, 2); byte b1_local = Wire.read(); byte b2_local = Wire.read(); // Read data
  float temp_reading = ((float)b1_local * 256 + (float)b2_local) / pow(2, 6); // Only keep 10 high bits. Convert to voltage
  temp_reading = ten_bit_to_volt(temp_reading);
  in_reading = volt_to_psi(temp_reading);
  in_filtered = (in_exp_constant * in_filtered + (1 - in_exp_constant) * in_reading); // Apply an exponential filter
}


void write_i2c(int in_address, int in_index, int in_value) {
  // Write data to the device over I2C
  Wire.beginTransmission(in_address); // Prepare internal variables
  Wire.write(in_index); // Write memory location to buffer
  Wire.write(in_value); // Write value to buffer
  Wire.endTransmission(); // Transmit
}
// End communicate with Pressure Regulator


// 5.2. Unit conversions
float ten_bit_to_volt(float in_ten_bit) {
  // Convert a 10-bit "5V source 10 bit ADC reading" to voltage.
  float volts = in_ten_bit * 5 / pow(2, 10);
  return volts;
}


unsigned long volt_to_ten_bit(float in_volts) {
  // Convert from 5V to 10 bit ADC reading (scaled as 2^10 distinct values over 5 volts)
  unsigned long tenBit = in_volts * pow(2, 10) / 5;
  return tenBit;
}


float volt_to_psi(float in_volts) {
  // Convert voltage reading to pressure [psi]
  // This calibration function is specific to our pressure regulator. "ASDX RR X 030PG A A 5". Read from ASDX series spec sheet, p.4.
  float max_pressure = 30;
  float min_pressure = 0;
  float supply_voltage = 5;
  float transfer_function_limits = 0.1;
  float psi = ((in_volts - transfer_function_limits * supply_voltage) * (max_pressure - min_pressure)) / (0.8 * supply_voltage) + min_pressure;
  return psi;
}


float psi_to_volt(float in_psi) {
  // Convert pressure to voltage reading [V]
  // This calibration function is specific to our pressure regulator. "ASDX RR X 030PG A A 5". Read from ASDX series spec sheet, p.4.
  float max_pressure = 30;
  float min_pressure = -30;
  float supply_voltage = 5; // supply voltage
  float transfer_function_limits = 0.1;
  float volt = 0.8 * supply_voltage / (max_pressure - min_pressure) * (in_psi - min_pressure) + transfer_function_limits * supply_voltage;
  return volt;
}
// End Unit conversions
