import procontroll.*;
import net.java.games.input.*;
import processing.serial.*; // Load serial library 

Serial arduinoPort; // Set arduinoPort as serial connection 
ControllIO controllIO;
ControllDevice joypad;
ControllStick stick1;
ControllStick stick2;

//Biasing buttons:
ControllButton button1; //roll left
ControllButton button2; //pitch back
ControllButton button3; //roll right
ControllButton button4; //pitch forward
ControllButton button5; //personality_enable


int throttle;
int yaw;
int pitch;
int roll;
int special;
int pitchBias;
int rollBias;
int personality_enable = 0;
final int START = 255;
boolean unpressed1 = true;
boolean unpressed2 = true;
boolean unpressed3 = true;
boolean unpressed4 = true;
boolean unpressed5 = true;

void setup() 
{ 

  //Get list of arduino devices connected
  for(String str:Serial.list())
  {
    println(str);
  }
  
  //Set arduinoPort to the one with the XBee
  arduinoPort = new Serial(this, Serial.list()[0], 57600);
  
  //Pin 1 -> 3.3V
  //Pin 2 -> TX
  //Pin 3 -> RX
  //Pin 10 -> GND

  //get joypad 
  controllIO = ControllIO.getInstance(this);

  // For Windows:
  // joypad = controllIO.getDevice("Logitech Dual Action");

  // For Linux:
  joypad = controllIO.getDevice("Logitech Logitech Dual Action");

  stick1 = joypad.getStick(0);
  stick2 = joypad.getStick(1);
  button1 = joypad.getButton(1);
  button2 = joypad.getButton(2);
  button3 = joypad.getButton(3);
  button4 = joypad.getButton(4);
  button5 = joypad.getButton(6);
  
  pitchBias = 127;  //Start bias
  rollBias = 127;   //Start bias
  
   print("Pitch Bias: ");
    print(pitchBias);
    print(" Roll Bias: ");
    println(rollBias);
} 

void draw() 
{ 

  //Get values from joypad
  throttle = (int) (-stick2.getY() * 254.0);
  
  //We dont want to fly downward
  if(throttle < 1)
    throttle = 0;
    
  yaw = 127;//(int) (stick2.getX() * 127.0)+127;
  pitch = (int) (-stick1.getY() * 127.0)+127;
  roll = (int) (stick1.getX() * 127.0)+127;
  special = 0;
  
  if(button1.pressed() && unpressed1){
    rollBias -= 1;
    print("Pitch Bias: ");
    print(pitchBias);
    print(" Roll Bias: ");
    println(rollBias);
    unpressed1 = false;
  }
  else unpressed1 = true;
  
  if(button3.pressed()){
    rollBias += 1;
    print("Pitch Bias: ");
    print(pitchBias);
    print(" Roll Bias: ");
    println(rollBias);
    unpressed3 = false;
  }
  else unpressed3 = true;
  
  if(button2.pressed()){
    pitchBias -= 1;
    print("Pitch Bias: ");
    print(pitchBias);
    print(" Roll Bias: ");
    println(rollBias);
  }
  if(button4.pressed()){
    pitchBias += 1;
    print("Pitch Bias: ");
    print(pitchBias);
    print(" Roll Bias: ");
    println(rollBias);
  }
  if(button5.pressed()){
   if(personality_enable == 1)
     personality_enable = 0;
   else
     personality_enable = 1;
   print("Personality Enable: ");
   println(personality_enable);
  }
  else
    unpressed5 = true;
    
  //Note: values are often +/- 1 even in 0 position
  //Does not matter for throttle since it does not
  //turn until V_MIN (100)

  //write values to arduino port
  
  arduinoPort.write(START);
  arduinoPort.write(throttle);
  arduinoPort.write(pitch);
  arduinoPort.write(roll);
  arduinoPort.write(yaw);
  arduinoPort.write(personality_enable);
  arduinoPort.write(pitchBias);
  arduinoPort.write(rollBias);
  
  //Add delay to allow for receiver to process input
  delay(50);

//println(personality_enable);
/*
  print(throttle);
  print(" ");
  print(yaw);
  print(" ");
  print(pitch);
  print(" ");
  println(roll);

  print(pitchBias);
  print(" ");
  println(rollBias);
  */
}

