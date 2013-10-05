// Wire Slave Sender
// by Nicholas Zambetti <http://www.zambetti.com>
// Demonstrates use of the Wire library
// Sends data as an I2C/TWI slave device
// Refer to the "Wire Master Reader" example for use with this
// Created 29 March 2006
// This example code is in the public domain.

#include <Wire.h>
#include <module_communications.h>
const int sensorPin = A0;
int readings[5];
int readingsStart = 0;
int tstep = 100;
int height = 0;
int setHeight = 100;

#define KP 0.85 // Same as in pd_digitalIMU.ino. Test this.

// Initializes serial communications (for debugging?)
// Initializes I2C communications for communicating with
// main arduino on quadrotor
// Sets up variables for reading from the sensor
void setup(){  
  Serial.begin(9600);
  init_communications(requestEvent); // register I2C event handler
  height = analogRead(sensorPin);
  for(int i = 0; i <5; i++)  {
    readings[i] = -1;
  }
}

// Reads from the height sensor
// Once the sensor has 4 or more valid readings, this code
// drops the largest and smallest (to remove outliers?)
void loop() {
  readings[readingsStart] = analogRead(sensorPin);
  int max = readings[readingsStart];
  int min = max;
  int total = 0;
  int tally = 0;
  for (int i = 0; i < 5; i++) {
    if (readings[i] >= 0) {
      tally++;
      total += readings[i];
      if (readings[i] > max)
        max = readings[i];
      if (readings[i] < min)
        min = readings[i];
    }
  }
  if (tally >= 4) {
    total -= max;
    total -= min;
    total = total/(tally-2);
  } else {
    total = total/tally;
  }
  height = total;
  Serial.println(height);
  readingsStart = (readingsStart+1)%5;
  delay(tstep);
}

// function that executes whenever data is requested by master
// this function is registered as an event, see setup()
// Code taken from module_PID.ino
// Proportionally controlls the power of the rotors based on
// the difference between the target height and the current height
void requestEvent() {
  int height_error = setHeight - height; // No velocity variable
  int throttle = constrain(height_error * KP, -255, 255);

  if (throttle >= 0)
    up(throttle);
  else
    down(-throttle);
}
