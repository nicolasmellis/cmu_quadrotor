/*

  PD control only:
  P => getting to target
  I => remove error over time
  D => Reduce overshooting
  
  Too much:
  P => overshoot too much
  I => lots of oscillation
  D => never reach target

*/

#include <Wire.h>
#include <Time.h>
#include "minimu9.h"
//#include <LSM303.h>
//#include <L3G.h>

//LSM303 compass;
//L3G gyro;

#define START 255
#define GYRADDR (0xD6 >> 1) //GYR_ADDRESS = (0xD2 >> 1)
#define ACCADDR (0x32 >> 1) //ACC_ADDRESS = (0x30 >> 1)
#define PERSONALITYADDR (0xFF >> 1) // 

#define DT 0.01         // [s/loop] loop period
#define AA 0.99         // complementary filter constant

#define V_MIN 100
#define V_MAX 250
#define V_RANGE 130.0

#define pitchStartBias 0.00
#define rollStartBias -0.05

#define A_GAIN_X A_GAIN * 0.7071    // sqrt(2) for x mode
#define G_GAIN_X G_GAIN * 0.7071    // sqrt(2) for x mode

#define A_GAIN 0.0573      // [deg/LSB]
#define G_GAIN 0.00875     // [deg/s/LSB]

#define MAX_ANGLE 30.0     // [deg]
#define MAX_RATE 180.0     // [deg/s]
#define INT_SAT 0.5

// 4in x 2.5in 2-blade props, + mode
#define KP 0.85             // [command/deg]
#define KI 0.0             //
#define KD 0.22             // [command/deg/s]
#define KY 0.22             // [command/deg/s]

/*
// 4in x 2.5in 2-blade props, x mode
#define KP 0.7             // [command/deg]
#define KI 0.0             //
#define KD 0.2             // [command/deg/s]
#define KY 0.3             // [command/deg/s]
*/

/*
// Testing
#define KP 0.0
#define KI 0.0
#define KD 0.0
#define KY 0.0
*/

/* 
// 110mm x 80mm 3-blade props, + mode
#define KP 0.6             // [command/deg]
#define KI 0.0             //
#define KD 0.2             // [command/deg/s]
#define KY 0.2             // [command/deg/s]
*/

void writeGyroReg(byte reg, byte value);
void writeAccReg(byte reg, byte value);
void writeMagReg(byte reg, byte value);
void readGyro(void);
void readAcc(void);
void readMag(void);
void rx();

unsigned char SQRT_LUT[256];

int gyro_x, gyro_y, gyro_z;
int accel_x, accel_y, accel_z;
int mag_x, mag_y, mag_z;

float angle_pitch = 0.0;  // [deg]
float angle_roll = 0.0;   // [deg]
float rate_pitch = 0.0;   // [deg/s]
float rate_roll = 0.0;    // [deg/s]
float rate_yaw = 0.0;     // [deg/s]

float altitude = 0.0;     // [in]

float pitch_error_integral = 0.0;
float roll_error_integral = 0.0;

signed int accel_x_zero = 0;
signed int accel_y_zero = 0;
signed int accel_z_zero = 0;
signed int gyro_y_zero = 0;
signed int gyro_x_zero = 0;
signed int gyro_z_zero = 0;

volatile unsigned char rx_i;
volatile unsigned char rx_packet[8];
unsigned char tx_packet[17];
volatile unsigned char rx_timeout = 0;

signed int throttle_command;
signed int pitch_command;
signed int roll_command;
signed int yaw_command;
signed int pitch_personality;
signed int roll_personality;
signed int yaw_personality;
signed int throttle_personality;
signed int personalityEnable;
signed int special;
signed int pitchBias;
signed int rollBias;

void setup()
{
  for(unsigned int i = 0; i <= 255; i++)
  {
    SQRT_LUT[i] = (unsigned char) (255.0 * sqrt((float) i / 255.0));
  }
    
  pinMode(2, OUTPUT);
  pinMode(4, OUTPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);
  
  pinMode(3, OUTPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(11, OUTPUT);
  pinMode(13, OUTPUT);
  
  Wire.begin();
  
  Serial.begin(57600);
  UCSR0A |= (1 << U2X0);
  UBRR0L = 33;
  
  writeGyroReg(L3G4200D_CTRL_REG1, 0x4F);
  writeAccReg(LSM303_CTRL_REG1_A, 0x27);
  writeMagReg(LSM303_MR_REG_M, 0x00);
  
  // go really fast 62.5kHz???
  TCCR0A = 0xA3;
  TCCR0B = 0x01; 
  TCCR2A = 0xA3;
  TCCR2B = 0x01;
 
  digitalWrite(2, HIGH);
  digitalWrite(4, LOW);
  digitalWrite(7, LOW);
  digitalWrite(8, HIGH);
  
  pitch_personality = 0;
  roll_personality = 0;
  yaw_personality = 0;
  throttle_personality = 0;
  personalityEnable = 0;
  
  //  if (!gyro.init())
  //  {
  //    Serial.println("Failed to autodetect gyro type!");
  //   while (1);
  //  }
  
  //compass.init();
  //compass.enableDefault();
  //gyro.enableDefault();
}

void loop()
{
  /*
    delay(400000); //4Sec
    for(int i = 0; i < 100; i++){
    analogWrite(3, 100); // LEFT;
    analogWrite(5, 100); // LEFT;
    analogWrite(6, 100); // LEFT;
    analogWrite(11, 100); // LEFT
    delay(200000);
    analogWrite(3, 0); // LEFT
    analogWrite(5, 0); // LEFT
    analogWrite(6, 0); // LEFT
    analogWrite(11, 0); // LEFT
    delay(200000);
    analogWrite(3, 150); // LEFT;
    analogWrite(5, 150); // LEFT;
    analogWrite(6, 150); // LEFT;
    analogWrite(11, 150); // LEFT
    delay(200000);
    analogWrite(3, 0); // LEFT
    analogWrite(5, 0); // LEFT
    analogWrite(6, 0); // LEFT
    analogWrite(11, 0); // LEFT
    delay(200000);
    }
    while(true)
    {
    delay(200000000); 
    }
  */
  signed int a_roll = 0;
  signed int a_pitch = 0;
  signed int a_z = 0;
  signed int g_roll = 0;
  signed int g_pitch = 0;
  signed int g_yaw = 0;
  signed int vref = 0;
  
  float pitch_error = 0.0;
  float roll_error = 0.0;
  
  unsigned char rx_len = 0;
  unsigned char rx_n = 0;
  unsigned char rx_byte = 0;
  
  float front_command_f = 0.0;
  float rear_command_f = 0.0;
  float left_command_f = 0.0;
  float right_command_f = 0.0;
  float base_command_f = 0.0;
  
  signed int front_command = 0;
  signed int rear_command = 0;
  signed int left_command = 0;
  signed int right_command = 0;
  
  unsigned int rate_pitch_int = 0;
  unsigned int angle_pitch_int = 0;
  unsigned int rate_roll_int = 0;
  unsigned int angle_roll_int = 0;
  unsigned int pitch_error_int = 0;
  unsigned int roll_error_int = 0;
  
  //unsigned char tx_packet[17];

  digitalWrite(13, HIGH);
  
  rx_len = Serial.available();
  if(rx_len > 0)
  {
    for(rx_n = 0; rx_n < (rx_len - 1); rx_n++)
    {
      rx_byte = Serial.read();
      if(rx_byte == START) 
      {
        rx_packet[0] = START;
        rx_i = 1; 
      }
      else if(rx_i <= 7)
      {
        rx_packet[rx_i] = rx_byte;
        rx_i++;
      }
      if(rx_i == 8)
      {
        rx();
        rx_timeout = 0;
        rx_i = 9;
      }
    }
  }
      
  rx_timeout++;
  if(rx_timeout >= 50)
  {
    throttle_command = 0;
    rx_timeout = 0;
  }
  
  //readGyro();   ///UNOCOMMENT FOR GYRO AND ACC USE
  //readAcc();  
   
  // START OF + MODE ----------------------------------------------------------------------
  a_pitch = (accel_x_zero - accel_x);
  a_roll = (accel_y - accel_y_zero);
  // a_z = (signed int) analogRead(A_Z) - a_z_z;
  g_pitch = (gyro_y - gyro_y_zero);
  g_roll = (gyro_x - gyro_x_zero);
  g_yaw = (gyro_z - gyro_z_zero);
  
  rate_pitch = (float) g_pitch * G_GAIN;
  rate_roll = (float) g_roll * G_GAIN;
  rate_yaw = (float) g_yaw * G_GAIN;
  
  angle_pitch = AA * (angle_pitch + rate_pitch * DT); 
  angle_pitch += (1.0 - AA) * (float) a_pitch * A_GAIN;
  pitch_error = (float) (pitch_command - 127) / 127.0 * MAX_ANGLE;// - angle_pitch;
  
  angle_roll = AA * (angle_roll + rate_roll * DT); 
  angle_roll += (1.0 - AA) * (float) a_roll * A_GAIN;
  roll_error = (float) (roll_command - 127) / 127.0 * MAX_ANGLE;// - angle_roll;

  /*
    Serial.print("rate_pitch: ");
    Serial.print(rate_pitch);
    Serial.print(" rate_roll: ");
    Serial.print(rate_roll);
    Serial.print(" rate_yaw: ");
    Serial.println(rate_yaw);
    Serial.print("pitch_error: ");
    Serial.print(pitch_error);
    Serial.print(" roll_error: ");
    Serial.println(roll_error);
  */ 
  
  // Control
  
  front_command_f -= pitch_error * KP;
  front_command_f -= pitch_error_integral * KI;
  front_command_f += rate_pitch * KD;
  front_command_f -= ((float) (yaw_command - 127) / 127.0 * MAX_RATE - rate_yaw) * KY;
  
  rear_command_f += pitch_error * KP;
  rear_command_f += pitch_error_integral * KI;
  rear_command_f -= rate_pitch * KD;
  rear_command_f -= ((float) (yaw_command - 127) / 127.0 * MAX_RATE - rate_yaw) * KY;
  
  right_command_f -= roll_error * KP;
  right_command_f -= roll_error_integral * KI;
  right_command_f += rate_roll * KD;      //WTF?? * 1.3
  right_command_f += ((float) (yaw_command - 127) / 127.0 * MAX_RATE - rate_yaw) * KY;
  
  left_command_f += roll_error * KP;
  left_command_f += roll_error_integral * KI;
  left_command_f -= rate_roll * KD;
  left_command_f += ((float) (yaw_command - 127) / 127.0 * MAX_RATE - rate_yaw) * KY;
  // END OF + MODE
  
  /*
    Serial.print("LeftF: ");
    Serial.print(left_command_f);
    Serial.print(" FrontF: ");
    Serial.print(front_command_f);
    Serial.print(" RightF: ");
    Serial.print(right_command_f);
    Serial.print(" RearF: ");
    Serial.println(rear_command_f);
  */
  
  
  if(SQRT_LUT[throttle_command] > V_MIN)
  {
    front_command = (signed int) front_command_f + throttle_command;
    rear_command = (signed int) rear_command_f + throttle_command;
    left_command = (signed int) left_command_f + throttle_command;
    right_command = (signed int) right_command_f + throttle_command; 
   
    //  right_command = (38 * right_command) >> 5;

    front_command = constrain(front_command, 0, 255);
    rear_command = constrain(rear_command, 0, 255);
    left_command = constrain(left_command, 0, 255);
    right_command = constrain(right_command, 0, 255);

    /*  front_command = SQRT_LUT[front_command];
        rear_command = SQRT_LUT[rear_command];
        left_command = SQRT_LUT[left_command];
        right_command = SQRT_LUT[right_command];
    */  
    
    //Change 200 to higher values to allow fine tuning!
    front_command = front_command - front_command * (double)((pitchBias-127)/200.0 + pitchStartBias);
    rear_command = rear_command + rear_command * (double)((pitchBias-127)/200.0 + pitchStartBias);
    
    right_command = right_command - right_command * (double)((rollBias-127)/200.0 + rollStartBias); 
    left_command = left_command + left_command * (double)((rollBias-127)/200.0 + rollStartBias);

    
    front_command = constrain(front_command, V_MIN, V_MAX);
    rear_command = constrain(rear_command, V_MIN, V_MAX);
    left_command = constrain(left_command, V_MIN, V_MAX);
    right_command = constrain(right_command, V_MIN, V_MAX);
  }
  else
  {
    pitch_error_int = 0.0;
    roll_error_int = 0.0;
  }
  /*
    if(special == 56)
    {
    analogWrite(3, V_MAX);
    analogWrite(6, V_MIN);
    delay(24000);
    analogWrite(3, V_MIN);
    analogWrite(6, V_MAX);
    delay(13000);
    special == 0;
    }
  */
  
  if(personalityEnable){
   
    readPersonality(); 
    
    left_command = 127-roll_personality + throttle_personality; //+Throttle on all
    right_command = 127+roll_personality + throttle_personality;
    front_command = 127+pitch_personality + throttle_personality;
    rear_command = 127-pitch_personality + throttle_personality; 
    throttle_command = 127 + throttle_personality + throttle_personality; 
    //Make sure none exceed VMax (and set to 0 those below VMin?)
   
    if(front_command < V_MIN) { front_command = V_MIN; }
    if(front_command > V_MAX) { front_command = V_MAX; }
    if(rear_command < V_MIN) { rear_command = V_MIN; }
    if(rear_command > V_MAX) { rear_command = V_MAX; }
    if(left_command < V_MIN) { left_command = V_MIN; }
    if(left_command > V_MAX) { left_command = V_MAX; }
    if(right_command < V_MIN) { right_command = V_MIN; }
    if(right_command > V_MAX) { right_command = V_MAX; }
    
  }
  
  
  
  analogWrite(3, left_command); // LEFT
  analogWrite(5, front_command); // FRONT
  analogWrite(6, right_command); // RIGHT
  analogWrite(11, rear_command); // REAR
  
  
  /*
    Serial.print("Left: ");
    Serial.print(left_command);
    Serial.print(" Front: ");
    Serial.print(front_command);
    Serial.print(" Right: ");
    Serial.print(right_command);
    Serial.print(" Rear: ");
    Serial.println(rear_command);
  */
  rate_pitch_int = (unsigned int)(rate_pitch * 10.0 + 8192.0);
  angle_pitch_int = (unsigned int)(angle_pitch * 10.0 + 8192.0); 
  rate_roll_int = (unsigned int)(rate_roll * 10.0 + 8192.0);
  angle_roll_int = (unsigned int)(angle_roll * 10.0 + 8192.0); 
  pitch_error_int = (unsigned int)(pitch_error * 10.0 + 8192.0);
  roll_error_int = (unsigned int)(roll_error * 10.0 + 8192.0);
  
  /*
    tx_packet[0] = 255;
    tx_packet[1] = (unsigned char) (rate_pitch_int >> 7);
    tx_packet[2] = (unsigned char) (rate_pitch_int & 0x7F);
    tx_packet[3] = (unsigned char) (angle_pitch_int >> 7);
    tx_packet[4] = (unsigned char) (angle_pitch_int & 0x7F); 
    tx_packet[5] = (unsigned char) (rate_roll_int >> 7);
    tx_packet[6] = (unsigned char) (rate_roll_int & 0x7F);
    tx_packet[7] = (unsigned char) (angle_roll_int >> 7);
    tx_packet[8] = (unsigned char) (angle_roll_int & 0x7F);
    tx_packet[9] = (unsigned char) (pitch_error_int >> 7);
    tx_packet[10] = (unsigned char) (pitch_error_int & 0x7F);
    tx_packet[11] = (unsigned char) (roll_error_int >> 7);
    tx_packet[12] = (unsigned char) (roll_error_int & 0x7F);
    tx_packet[13] = (unsigned char) front_command;
    tx_packet[14] = (unsigned char) right_command;
    tx_packet[15] = (unsigned char) rear_command;
    tx_packet[16] = (unsigned char) left_command;
  */  /*
        unsigned char x = 0;
  
        tx_packet[0] = x;
        tx_packet[1] = x;
        tx_packet[2] = x;
        tx_packet[3] = x;
        tx_packet[4] = x;
        tx_packet[5] = x;
        tx_packet[6] = x;
        tx_packet[7] = x;
        tx_packet[8] = x;
        tx_packet[9] = x;
        tx_packet[10] = x;
        tx_packet[11] = x;
        tx_packet[12] = x;
        tx_packet[13] = x;
        tx_packet[14] = x;
        tx_packet[15] = x;
        tx_packet[16] = x;
      */
  //Serial.write(tx_packet,17);
 
  digitalWrite(13, LOW);
  delay(375);
}

void rx()
{
  throttle_command = rx_packet[1];
  throttle_command = ((double)throttle_command/255 * 230);
  pitch_command = rx_packet[2];
  roll_command = rx_packet[3];
  yaw_command = rx_packet[4];
  special = rx_packet[5];
  pitchBias = rx_packet[6];
  rollBias = rx_packet[7];
  
  /*
    if(special){
    personalityEnable = 1; 
    }
  */
}

void readGyro()
{
  
  Wire.beginTransmission(GYRADDR);
  //(0xD2 >> 1)
  
  // assert the MSB of the address to get the gyro 
  // to do slave-transmit subaddress updating.
  Wire.write(L3G4200D_OUT_X_L | (1 << 7)); 
  Wire.endTransmission();
  Wire.requestFrom(GYRADDR, 6);

  while (Wire.available() < 6){
    Serial.println("Gyro Wire.available() < 6");
  }

  uint8_t xla = Wire.read();
  uint8_t xha = Wire.read();
  uint8_t yla = Wire.read();
  uint8_t yha = Wire.read();
  uint8_t zla = Wire.read();
  uint8_t zha = Wire.read();

  gyro_x = xha << 8 | xla;
  gyro_y = yha << 8 | yla;
  gyro_z = zha << 8 | zla;
}

void readAcc(void)
{
  Wire.beginTransmission(ACCADDR);
  // assert the MSB of the address to get the accelerometer 
  // to do slave-transmit subaddress updating.
  Wire.write(LSM303_OUT_X_L_A | (1 << 7)); 
  Wire.endTransmission();
  Wire.requestFrom(ACCADDR, 6);

  while (Wire.available() < 6){
    Serial.println("Acc Wire.available() < 6");
  }

  byte xla = Wire.read();
  byte xha = Wire.read();
  byte yla = Wire.read();
  byte yha = Wire.read();
  byte zla = Wire.read();
  byte zha = Wire.read();

  accel_x = (xha << 8 | xla) >> 4;
  accel_y = (yha << 8 | yla) >> 4;
  accel_z = (zha << 8 | zla) >> 4;
}

void readPersonality()
{
  
  ///Wire.beginTransmission(PERSONALITYADDR);
  //(0xD2 >> 1)
  
  // assert the MSB of the address to get the gyro 
  // to do slave-transmit subaddress updating.
  //Wire.write(L3G4200D_OUT_X_L | (1 << 7)); //Change to MSB of personlaity
  //Wire.endTransmission();
  
  //Request two bytes
  Wire.requestFrom(PERSONALITYADDR, 2);

  //Wait until two bytes were sent
  while (Wire.available() < 2){
    Serial.println("Personality Wire.available() < 2");
  }

  uint8_t command = Wire.read();
  uint8_t magnitude = Wire.read();
    
  // Reset Personality commands
  pitch_personality = 0;
  roll_personality = 0;
  yaw_personality = 0;
  personalityEnable = 1;
    
  switch(command) {
  case(0): pstop();
  case(1): forward(magnitude); break;
  case(2): backward(magnitude); break;
  case(3): right(magnitude); break;
  case(4): left(magnitude); break;
  case(5): up(magnitude); break;
  case(6): down(magnitude); break;
  case(7): getIMU(); break;
  case(8): return_control(); break;
  default: Serial.println("Invalid command from Personality");
  }
  //can have turn_right, turn_left using yaw etc.
}

void pstop(){
}

void forward(uint8_t magnitude){
  pitch_personality = magnitude;
}

void backward(uint8_t magnitude){
  pitch_personality = -magnitude;
}

void left(uint8_t magnitude){
  //pitch_personality = 0;
  roll_personality = -magnitude;
  //yaw_personality = 0;
  //personalityEnable = 1;
}

void right(uint8_t magnitude){
  //pitch_personality = 0;
  roll_personality = magnitude;
  //yaw_personality = 0;
  //personalityEnable = 1;
}

// Maximum change throttle allowed
#define MAX_THROTTLE_CHANGE 30
#define MAX_MAG 0xFF
// Approximately magnitude *= .118 about dividing by 8

void up(uint8_t magnitude){
  //throttle_personality = (uint8_t)(((float)magnitude / (float)MAX_MAG) * MAX_THROTTLE_CHANGE);
  throttle_personality += (((double)magnitude)/255)*30;
}

void down(uint8_t magnitude){
  //throttle_personality = -(uint8_t)(((float)magnitude / (float)MAX_MAG) * MAX_THROTTLE_CHANGE);
  throttle_personality -= (((double)magnitude)/255)*30;
}

void getIMU(){  
  
}

void return_control()
{
  personalityEnable = 0; 
}

void readMag(void)
{
  Wire.beginTransmission(MAG_ADDRESS);
  Wire.write(LSM303_OUT_X_H_M);
  Wire.endTransmission();
  Wire.requestFrom(MAG_ADDRESS, 6);

  while (Wire.available() < 6);

  byte xhm = Wire.read();
  byte xlm = Wire.read();

  byte yhm, ylm, zhm, zlm;

  zhm = Wire.read();
  zlm = Wire.read();
  yhm = Wire.read();
  ylm = Wire.read();

  mag_x = (xhm << 8 | xlm);
  mag_y = (yhm << 8 | ylm);
  mag_z = (zhm << 8 | zlm);
}

void writeGyroReg(byte reg, byte value)
{
  Wire.beginTransmission(GYRADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void writeAccReg(byte reg, byte value)
{
  Wire.beginTransmission(ACCADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

void writeMagReg(byte reg, byte value)
{
  Wire.beginTransmission(MAG_ADDRESS);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}
