/* module_communications.h - Library for communicating between
 * quadrotor modules and the main quadrotor arduino.
 */

#include "module_communications.h"
#include <Wire.h>
#include <Arduino.h>

// It seems that all magnitude values should be constrained
// to [-255,255]. 

// Initializes the Wire library, taking a request handler function
// that responds to the main arduino's requests
void init_communications(void (*requestHandler)()) {
  Wire.begin(PERSONALITYADDR);
  Wire.onRequest(requestHandler);
}

void pstop() {
  Wire.write(0);
  Wire.write(0); // Unused, but 2 bytes are expected
}

// Helper function for forward, backward, etc
void sendDirection(int command, int magnitude) {
  Wire.write(command);
  Wire.write(constrain(magnitude, -255, 255));
}

void forward(int magnitude) {
  sendDirection(1, magnitude);
}

void backward(int magnitude) {
  sendDirection(2, magnitude);
}

void right(int magnitude) {
  sendDirection(3, magnitude);
}

void left(int magnitude) {
  sendDirection(4, magnitude);
}

void up(int magnitude) {
  sendDirection(5, magnitude);
}

void down(int magnitude) {
  sendDirection(6, magnitude);
}

// I assume that this gets the current IMU data from the quadrotor,
// but we don't have an IMU right now, and there is no code written
// on the master side for this function.
void getIMU() {
  Wire.write(7);
  Wire.write(0); // Unused, but 2 bytes are expected
}

void return_control() {
  Wire.write(8);
  Wire.write(0); // Unused, but 2 bytes are expected
}
