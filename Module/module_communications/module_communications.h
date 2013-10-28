/* module_communications.h - Library for communicating between
 * quadrotor modules and the main quadrotor arduino.
 */

#ifndef MODULE_COMMUNICATIONS_H
#define MODULE_COMMUNICATIONS_H

#define PERSONALITYADDR (0xFF >> 1) // Same as in pd_digitalIMU.ino

void init_communications(void (*requestHandler)());

void pstop(); // Hold position / Stop moving?
void forward(int magnitude);
void backward(int magnitude);
void right(int magnitude);
void left(int magnitude);
void up(int magnitude);
void down(int magnitude);
void getIMU(); // I'm not sure what this is supposed to do
void return_control(); // Tell the main arduino to stop listening to the module

#endif
